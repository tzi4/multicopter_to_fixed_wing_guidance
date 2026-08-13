#!/usr/bin/env python3
"""
Drone Kamera → Redis Dedektörü + Gömülü Karar Verici
=====================================================
- ROS kamera topic'inden görüntü alır
- Kırmızı hedef tespiti yapar (HSV renk filtresi)
- BBox verisini Redis 'tracker_bbox' kanalına yayınlar
- decider.py mantığını kendi içinde çalıştırır (ayrı script gerektirmez)
- OpenCV ekranında ROL/GÖREV/HEDEF durumunu gösterir
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import time
from collections import deque


class SimRedisDetector:
    # ─── KARAR VERİCİ PARAMETRELERİ (decider.py'den) ───
    WINDOW_SIZE = 25                    # Son kaç kareye bakılacak
    TRANSITION_THRESHOLD = 20           # konumlu→görüntülü için geçerli kare eşiği
    REVERT_THRESHOLD = 3                # görüntülü→konumlu için düşük eşik
    REVERT_DWELL_SECONDS = 2            # Geri dönüş bekleme süresi (sn)
    MIN_COVERAGE_TRANSITION = 0.85      # Geçiş için minimum coverage %
    MIN_COVERAGE_HOLD = 0.7             # Görüntülüde kalma için minimum coverage %

    def __init__(self):
        # --- REDIS BAĞLANTISI ---
        print("Redis sunucusuna bağlanılıyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.set('gorev', 'yok')
            self.r.set('komut_yetkisi', 'konumlu')
            print("Redis bağlantısı başarılı! Yayın kanalı: 'tracker_bbox'")
        except Exception as e:
            print(f"Redis Bağlantı Hatası: {e}")
            exit(1)

        # --- ROS BAĞLANTISI ---
        rospy.init_node('sim_redis_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/drone_1/webcam/image_raw", Image, self.image_callback)
        print("ROS Node başlatıldı, /drone_1/webcam/image_raw kanalından görüntü bekleniyor...")

        # --- RENK AYARLARI (Kırmızı) ---
        self.lower_red1 = np.array([0, 70, 50])   
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 70, 50]) 
        self.upper_red2 = np.array([180, 255, 255])

        # --- KARAR VERİCİ İÇ DURUMU ---
        self.current_mode = 'konumlu'               # Başlangıç modu
        self.decision_window = deque(maxlen=self.WINDOW_SIZE)
        self.revert_pending_since = None             # Dwell zamanlayıcısı
        self.last_frame_time = time.time()

        # --- VİDEO KAYIT ---
        self.video_writer = None
        self.video_filename = f"kayit_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        print(f"Video kaydı aktif: {self.video_filename}")

    # ═══════════════════════════════════════════════════
    # KARAR VERİCİ — GÖMÜLÜ (decider.py mantığı)
    # ═══════════════════════════════════════════════════

    def _evaluate_frame(self, valid_detection, coverage):
        """
        Tek bir frame'in geçiş kriterlerini sağlayıp sağlamadığını değerlendir.
        Renk tespiti kullanıldığı için confidence yerine valid_detection kullanılır.
        Coverage eşiği MODA GÖRE değişir (hysteresis).
        """
        if not valid_detection:
            return False, "hedef_yok"

        if self.current_mode == 'konumlu':
            # Konumludayken: görüntülüye geçmek için hedef YAKIN olmalı
            if coverage < self.MIN_COVERAGE_TRANSITION:
                return False, "uzak_hedef"
        else:
            # Görüntülüdeyken: hedef görünür olduğu sürece kabul et
            if coverage < self.MIN_COVERAGE_HOLD:
                return False, "cok_uzak"

        return True, "gecerli"

    def _make_decision(self, valid_count):
        """
        Pencere durumuna göre mod kararı üret.
        Dwell time mantığı: görüntülü→konumlu dönüşü geciktirir.
        """
        now = time.time()

        # Manuel durdurma kontrolü
        try:
            val = self.r.get('manuel_durdur')
            manuel = val and val.decode('utf-8') == '1'
        except Exception:
            manuel = False

        if manuel:
            self.revert_pending_since = None
            if self.current_mode != 'konumlu':
                return 'konumlu', True
            return 'konumlu', False

        # Pencere yeterince dolmadıysa karar değiştirme
        if len(self.decision_window) < self.WINDOW_SIZE:
            return self.current_mode, False

        if self.current_mode == 'konumlu':
            # Konumlu → görüntülü: yüksek eşik
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                return 'goruntulu', True
            return 'konumlu', False

        elif self.current_mode == 'goruntulu':
            # Görüntülü → konumlu: düşük eşik + dwell time
            if valid_count <= self.REVERT_THRESHOLD:
                if self.revert_pending_since is None:
                    self.revert_pending_since = now
                    print(f"[KARAR] Eşik altı ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell başladı ({self.REVERT_DWELL_SECONDS}s)", flush=True)

                dwell_elapsed = now - self.revert_pending_since
                if dwell_elapsed >= self.REVERT_DWELL_SECONDS:
                    self.revert_pending_since = None
                    return 'konumlu', True

                return 'goruntulu', False
            else:
                # Eşik üstüne çıktı — dwell iptal
                if self.revert_pending_since is not None:
                    elapsed = now - self.revert_pending_since
                    print(f"[KARAR] Hedef geri göründü ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell iptal ({elapsed:.2f}s)", flush=True)
                    self.revert_pending_since = None
                return 'goruntulu', False

        return self.current_mode, False

    def _update_decision(self, valid_detection, coverage):
        """Her frame'de çağrılır: pencereyi güncelle, karar üret, Redis'e yaz."""
        frame_valid, reason = self._evaluate_frame(valid_detection, coverage)

        self.decision_window.append(frame_valid)
        valid_count = sum(1 for v in self.decision_window if v)

        yeni_mod, degisti = self._make_decision(valid_count)

        if degisti:
            eski = self.current_mode
            self.current_mode = yeni_mod
            self.decision_window.clear()
            self.revert_pending_since = None
            try:
                self.r.set('komut_yetkisi', self.current_mode)
            except Exception:
                pass
            print(f"[KARAR] >>> MOD DEĞİŞTİ: {eski} → {yeni_mod} "
                  f"(pencere: {valid_count}/{self.WINDOW_SIZE})", flush=True)

        return self.current_mode

    # ═══════════════════════════════════════════════════
    # KAMERA CALLBACK
    # ═══════════════════════════════════════════════════

    def image_callback(self, data):
        t_start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- Hedef Vuruş Alanı (Sarı Kutu) ---
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        cv2.rectangle(cv_image, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 1)
        cv2.putText(cv_image, "Vurus Alani", (av_left + 4, av_top + 14),
                    cv2.FONT_HERSHEY_DUPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # --- HSV renk tespiti ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_detection = False
        horizontal_coverage = 0.0

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            target_area = cv2.contourArea(c)

            if target_area > 18:
                x, y, w, h = cv2.boundingRect(c)
                valid_detection = True

                horizontal_coverage = (w / w_img) * 100
                validity_flag = 1

                t_detect = time.time()
                detect_ms = (t_detect - t_start) * 1000
                self.r.set('timing_bbox_to_redis', f"{detect_ms:.2f}")

                bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag, t_start]
                self.r.publish('tracker_bbox', str(bbox))

                center_x = int(x + w/2)
                center_y = int(y + h/2)
                print(f"Hedef Merkez: ({center_x}, {center_y}) | Coverage: {horizontal_coverage:.1f}%")

                # Hedefi kırmızı kutu ile işaretle
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

        # --- KARAR VERİCİ GÜNCELLE ---
        komut_yetkisi = self._update_decision(valid_detection, horizontal_coverage)

        # --- EKRAN HUD ---
        FONT = cv2.FONT_HERSHEY_DUPLEX
        AA = cv2.LINE_AA

        if komut_yetkisi == 'goruntulu':
            rol_text = "LIDER"
            gorev_text = "GORUNTULU"
            hud_color = (0, 255, 0)      # Yeşil
        else:
            rol_text = "UYE"
            gorev_text = "KONUMLU"
            hud_color = (0, 165, 255)    # Turuncu

        # Üst bant arka planı (yarı saydam koyu şerit)
        overlay = cv_image.copy()
        cv2.rectangle(overlay, (0, 0), (w_img, 50), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, cv_image, 0.45, 0, cv_image)

        # Sol üst: Rol | Görev (tek satırda)
        cv2.putText(cv_image, f"{rol_text}  |  {gorev_text}", (8, 20),
                    FONT, 0.5, hud_color, 1, AA)

        # Hedef durumu
        hedef_text = "HEDEF: BULUNDU" if valid_detection else "HEDEF: YOK"
        hedef_color = (0, 255, 0) if valid_detection else (100, 100, 255)
        cv2.putText(cv_image, hedef_text, (8, 40),
                    FONT, 0.45, hedef_color, 1, AA)

        # Sağ üst: Pencere durumu
        valid_count = sum(1 for v in self.decision_window if v)
        fill = len(self.decision_window)
        pencere_text = f"Pencere: {valid_count}/{fill}"
        # Metin genişliğini hesapla ve sağa yasla
        (tw, _), _ = cv2.getTextSize(pencere_text, FONT, 0.4, 1)
        cv2.putText(cv_image, pencere_text, (w_img - tw - 10, 20),
                    FONT, 0.4, (180, 180, 180), 1, AA)

        # Sağ üst alt: Coverage (hedef varsa)
        if valid_detection:
            cov_text = f"Cov: {horizontal_coverage:.1f}%"
            (tw2, _), _ = cv2.getTextSize(cov_text, FONT, 0.4, 1)
            cv2.putText(cv_image, cov_text, (w_img - tw2 - 10, 40),
                        FONT, 0.4, (180, 220, 180), 1, AA)

        # --- VİDEO KAYIT ---
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.video_filename, fourcc, 20.0, (w_img, h_img))
            print(f"VideoWriter başlatıldı: {w_img}x{h_img} @ 20fps")
        self.video_writer.write(cv_image)

        cv2.imshow("Simulasyon Redis Dedektoru", cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    detector = None
    try:
        detector = SimRedisDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if detector and detector.video_writer:
            detector.video_writer.release()
            print(f"Video kaydedildi: {detector.video_filename}")
        cv2.destroyAllWindows()
