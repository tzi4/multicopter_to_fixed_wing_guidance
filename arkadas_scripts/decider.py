#!/usr/bin/env python3
"""
KARAR VERİCİ (TransitionDecider)
================================

Görev: Konumlu güdüm ile görüntülü güdüm arasındaki geçişe otonom karar verir.

Mantık:
- Her frame'de YOLO'dan gelen tespit verilerini değerlendirir.
- Son N karenin verisini hafızasında tutar (deque).
- Geçiş kriterleri: confidence + coverage + persistence
- Hysteresis kullanır: yüksek eşik geçiş, düşük eşik geri dönüş.
- DWELL TIME: Görüntülü→konumlu dönüş için ekstra bekleme süresi.
  Hedef yarım saniye gibi kısa süre kayboldu ama hemen geri geldiyse
  oscillation olmasın diye geri dönüş kararı dwell süresi kadar gecikir.
- Manuel durdurma aktifse otomatik olarak konumlu moda döner.

Çıktı: Redis'e 'komut_yetkisi' anahtarına 'konumlu' veya 'goruntulu' yazar.
       Konumlu güdüm ve görüntülü güdüm bu anahtarı okuyarak kendi davranışlarını ayarlar.
"""

import time
import redis
import ast
import threading
import csv
from datetime import datetime
from collections import deque


class TransitionDecider:
    # ─── EŞİK PARAMETRELERİ  ───
    
    # Pencere boyutu — son kaç kareye bakacak
    # YOLO 30 FPS varsayımı ile N=20 ≈ 0.66 saniye
    WINDOW_SIZE = 25
    
    # Geçiş eşiği (yüksek): konumlu → görüntülü için
    # Son WINDOW_SIZE karenin EN AZ kaç tanesinde tüm kriterler sağlanmalı
    TRANSITION_THRESHOLD = 20  # %75
    
    # Geri dönüş eşiği (düşük): görüntülü → konumlu için (hysteresis)
    # Son WINDOW_SIZE karenin EN FAZLA kaç tanesinde kriter sağlanırsa geri dön
    REVERT_THRESHOLD = 3  # %25
    
    # ─── DWELL TIME (Geri Dönüş Bekleme Süresi) ───
    # Görüntülüden konumluya dönüş kararı verildikten sonra,
    # bu süre boyunca hedef geri görünmezse karar gerçekleşir.
    # Hedef geri görünürse karar iptal olur. Oscillation'ı önler.
    REVERT_DWELL_SECONDS = 2 # 2 saniye
     
    # NOT: Konumlu → görüntülü geçiş için dwell yok, çünkü zaten WINDOW_SIZE süresi
    # boyunca yüksek eşik (%75) sağlanması gerekiyor, bu kendi başına debounce.
    
    # Bir karenin "geçerli" sayılması için kriterler
    MIN_CONFIDENCE = 0.65
    # Eski tek eşik kaldırıldı — yerine moda göre değişen iki eşik:
    # Konumlu modda: hedef YAKIN olmalı (görüntülüye geçmek için)
    # Görüntülü modda: hedef görünür olduğu sürece tamam (görüntülüde kalmak için)
    MIN_COVERAGE_TRANSITION = 1  # %10 — sadece kilit mesafe geçiş yapar
    MIN_COVERAGE_HOLD = 0.75          # %3  — görüntülüde uzaklaşsa bile bir süre tut
    
    # ─── REDIS ANAHTARLARI ───
    KEY_KOMUT_YETKISI = 'komut_yetkisi'    # Çıktı: 'konumlu' / 'goruntulu'
    KEY_MANUEL_DURDUR = 'manuel_durdur'    # Giriş: '1' aktif, başka değer pasif
    CHANNEL_TRACKER = 'tracker_bbox'       # Detection.py'nin yayını
    
    # ─── LOG AYARLARI ───
    ENABLE_CSV_LOG = True
    
    def __init__(self, redis_host='localhost', redis_port=6379):
        # Redis bağlantısı
        print("[KARAR] Redis sunucusuna bağlanılıyor...", flush=True)
        try:
            self.r = redis.Redis(host=redis_host, port=redis_port, db=0)
            self.r.ping()
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe(self.CHANNEL_TRACKER)
            print(f"[KARAR] Redis '{self.CHANNEL_TRACKER}' kanalına abone olundu.", flush=True)

            # ─── PUB/SUB BUFFER FLUSH ───
            # Decider başlarken kanalda biriken eski mesajları at
            # (Sistem yeniden başlatılınca eski tespitlerin pencereyi anında doldurmaması için)
            print("[KARAR] Pub/sub buffer temizleniyor (eski mesajları at)...", flush=True)
            flush_start = time.time()
            flushed_count = 0
            while time.time() - flush_start < 0.5:  # 0.5 saniye boyunca temizle
                msg = self.pubsub.get_message(ignore_subscribe_messages=True, timeout=0.01)
                if msg is None:
                    break
                if msg.get('type') == 'message':
                    flushed_count += 1
            print(f"[KARAR] {flushed_count} eski mesaj atıldı, başlangıç temiz.", flush=True)
        except Exception as e:
            print(f"[KARAR] Redis Bağlantı Hatası: {e}", flush=True)
            raise
        
        # Başlangıç durumu: konumlu (sistem başlangıçta konumlu güdümle çalışır)
        self.current_mode = 'konumlu'
        self._publish_mode()  # Başlangıçta Redis'e yaz
        
        # Pencere: her elemanı (frame_valid: bool, conf: float, cov: float, reason: str) tutar
        self.window = deque(maxlen=self.WINDOW_SIZE)
        
        # Tespit yokken de pencereyi doldurabilmek için zamanlayıcı
        self.last_frame_time = time.time()
        
        # ─── DWELL TIME STATE ───
        # Geri dönüş için pending durumu
        # None = pending yok, normal çalışma
        # float = pending başlangıç zamanı (time.time())
        self.revert_pending_since = None
        
        # Loglama
        self.csv_writer = None
        self.log_file = None
        if self.ENABLE_CSV_LOG:
            self._init_logger()
        
        # Thread güvenliği için lock
        self.lock = threading.Lock()
        
        print(f"[KARAR] Başlatıldı | Mod: {self.current_mode}", flush=True)
        print(f"[KARAR] Parametreler: N={self.WINDOW_SIZE}, "
            f"Geçiş={self.TRANSITION_THRESHOLD}, Dönüş={self.REVERT_THRESHOLD}, "
            f"Dwell={self.REVERT_DWELL_SECONDS}s, "
            f"MinConf={self.MIN_CONFIDENCE}, "
            f"Cov(Geçiş≥{self.MIN_COVERAGE_TRANSITION}% / Tut≥{self.MIN_COVERAGE_HOLD}%)", flush=True)
        
        # ─── STATUS PRINT STATE ───
        self.last_status_time = 0.0           # En son ne zaman print ettik
        self.HEARTBEAT_INTERVAL = 1.0         # Değişim yoksa bile saniyede 1 bas
        self.last_status_snapshot = None      # Son print'teki durum (değişim tespiti için)
    
    def _init_logger(self):
        """CSV log dosyasını başlat"""
        log_filename = f"karar_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        headers = [
            "timestamp", "current_mode",
            "frame_valid", "confidence", "coverage",
            "reason",
            "valid_count_in_window",
            "window_fill",
            "revert_pending",
            "dwell_elapsed",
            "decision_change"
        ]
        self.csv_writer.writerow(headers)
        print(f"[KARAR] Log dosyası: {log_filename}", flush=True)
    
    def _publish_mode(self):
        """Mevcut modu Redis'e yaz"""
        try:
            self.r.set(self.KEY_KOMUT_YETKISI, self.current_mode)
        except Exception as e:
            print(f"[KARAR] Redis publish hatası: {e}", flush=True)
    
    def _check_manuel_durdur(self):
        """Manuel durdurma aktif mi?"""
        try:
            val = self.r.get(self.KEY_MANUEL_DURDUR)
            return val and val.decode('utf-8') == '1'
        except Exception:
            return False
    

    def _evaluate_frame(self, bbox_data):
        """
        Tek bir frame'in karar kriterlerine uyup uymadığını değerlendir.
        bbox_data: [x, y, w, h, h_coverage, validity_flag, confidence]
        
        Coverage eşiği MODA GÖRE değişir (hysteresis):
        - Konumlu modda: yakın hedef ister (MIN_COVERAGE_TRANSITION)
        → Sadece kilit mesafede görüntülüye geçer, uzaktaki hedefler tetiklemez
        - Görüntülü modda: hedef hâlâ görünür olduğu sürece geçerli say (MIN_COVERAGE_HOLD)
        → Geçici uzaklaşmalarda görüntülüde kalır, oscillation yapmaz
        
        Returns: (frame_valid: bool, confidence: float, coverage: float, reason: str)
        """
        if bbox_data is None or len(bbox_data) < 7:
            return False, 0.0, 0.0, "veri_eksik"
        
        try:
            x, y, w, h = bbox_data[0:4]
            coverage = float(bbox_data[4])
            validity = int(bbox_data[5])
            confidence = float(bbox_data[6])
        except (ValueError, IndexError) as e:
            return False, 0.0, 0.0, f"parse_hatasi:{e}"
        
        # Validity: detection.py grace period'daysa eski veri, geçersiz say
        if validity == 0:
            return False, confidence, coverage, "grace_period"
        
        if confidence < self.MIN_CONFIDENCE:
            return False, confidence, coverage, "dusuk_confidence"
        
        # ─── COVERAGE EŞİĞİ MODA GÖRE DEĞİŞİR (HYSTERESIS) ───
        if self.current_mode == 'konumlu':
            # Konumludayken: görüntülüye geçmek için hedef YAKIN olmalı
            if coverage < self.MIN_COVERAGE_TRANSITION:
                return False, confidence, coverage, "uzak_hedef"
        else:
            # Görüntülüdeyken: hedef görünür olduğu sürece kabul et
            if coverage < self.MIN_COVERAGE_HOLD:
                return False, confidence, coverage, "cok_uzak"
        
        return True, confidence, coverage, "gecerli"

    
    def _make_decision(self, valid_count, manuel_durdur):
        """
        Pencere durumuna göre karar üret.
        Returns: (yeni_mod: str, mod_degisti: bool, dwell_elapsed: float)
        
        DWELL TIME mantığı (sadece görüntülü→konumlu için):
        - valid_count REVERT_THRESHOLD altına düştü → dwell zamanlayıcısı başlat
        - Dwell süresi içinde valid_count tekrar yükseldi → dwell iptal, görüntülüde kal
        - Dwell süresi doldu ve hala düşük → konumluya geç
        """
        now = time.time()
        dwell_elapsed = 0.0
        
        # Manuel durdurma her şeyi ezer
        if manuel_durdur:
            self.revert_pending_since = None
            if self.current_mode != 'konumlu':
                return 'konumlu', True, 0.0
            return 'konumlu', False, 0.0
        
        # Pencere yeterince dolmadıysa karar değiştirme
        if len(self.window) < self.WINDOW_SIZE:
            return self.current_mode, False, 0.0
        
        if self.current_mode == 'konumlu':
            # Konumlu → görüntülü: yüksek eşik, anlık karar (pencere zaten debounce)
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                return 'goruntulu', True, 0.0
            return 'konumlu', False, 0.0
        
        elif self.current_mode == 'goruntulu':
            # Görüntülü → konumlu: düşük eşik + dwell time
            
            if valid_count <= self.REVERT_THRESHOLD:
                # Eşik altında — dwell başlat veya devam ettir
                if self.revert_pending_since is None:
                    self.revert_pending_since = now
                    print(f"[KARAR] Eşik altı tespit edildi ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell başladı ({self.REVERT_DWELL_SECONDS}s bekleme)", flush=True)
                
                dwell_elapsed = now - self.revert_pending_since
                
                if dwell_elapsed >= self.REVERT_DWELL_SECONDS:
                    # Dwell doldu, geri dönüşü gerçekleştir
                    self.revert_pending_since = None
                    return 'konumlu', True, dwell_elapsed
                
                # Hala bekliyoruz, görüntülüde kal
                return 'goruntulu', False, dwell_elapsed
            
            else:
                # Eşik üstüne çıktı — varsa dwell iptal et
                if self.revert_pending_since is not None:
                    elapsed = now - self.revert_pending_since
                    print(f"[KARAR] Hedef geri göründü ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell iptal edildi (geçen süre: {elapsed:.2f}s)", flush=True)
                    self.revert_pending_since = None
                
                return 'goruntulu', False, 0.0
        
        return self.current_mode, False, 0.0
    
    def _maybe_print_status(self, valid_count, frame_valid, reason, conf, cov,
                            manuel_durdur, dwell_elapsed, mod_degisti):
        """
        Status satırını terminale bas — ama sadece:
          (a) anlamlı bir değişim olduysa, VEYA
          (b) son print'ten beri HEARTBEAT_INTERVAL geçtiyse.
        Mod değişimi zaten başka yerde bastırılıyor, burada onu tekrar basmıyoruz.
        """
        now = time.time()
        dwell_active = self.revert_pending_since is not None
        
        # Mevcut durumun "imzası" — bunlardan biri değiştiyse print edilir
        snapshot = (
            self.current_mode,
            frame_valid,
            reason,
            dwell_active,
            manuel_durdur,
        )
        
        changed = (snapshot != self.last_status_snapshot)
        heartbeat_due = (now - self.last_status_time) >= self.HEARTBEAT_INTERVAL
        
        # Mod değişimi zaten ayrıca bastırılıyor; burada tekrar basmayalım
        if mod_degisti:
            self.last_status_snapshot = snapshot
            self.last_status_time = now
            return
        
        if not (changed or heartbeat_due):
            return
        
        # ─── Satırı oluştur ───
        # Son frame özeti
        if frame_valid:
            son_info = f"son: GEÇERLİ (conf={conf:.2f}, cov={cov:.1f}%)"
        else:
            son_info = f"son: GEÇERSİZ ({reason})"
        
        # Dwell bilgisi (varsa)
        dwell_info = ""
        if dwell_active:
            dwell_info = f" | DWELL {dwell_elapsed:.2f}/{self.REVERT_DWELL_SECONDS}s"
        
        # Manuel durdurma uyarısı
        manuel_info = " | [MANUEL DURDUR]" if manuel_durdur else ""
        
        # Pencere doluluğu (henüz dolmadıysa belirt)
        if len(self.window) < self.WINDOW_SIZE:
            pencere_info = f"pencere: {valid_count}/{self.WINDOW_SIZE} (dolmadı)"
        else:
            pencere_info = f"pencere: {valid_count}/{self.WINDOW_SIZE}"
        
        # Heartbeat mi yoksa değişim mi olduğunu işaretle
        tag = "Δ" if changed else "·"  # Δ=değişim, ·=heartbeat
        
        print(f"[KARAR] {tag} MOD:{self.current_mode.upper():10s} | "
              f"{pencere_info} | {son_info}{dwell_info}{manuel_info}",
              flush=True)
        
        self.last_status_snapshot = snapshot
        self.last_status_time = now


    
    def process_frame(self, bbox_data):
        """Yeni bir frame geldiğinde çağrılır."""
        with self.lock:
            frame_valid, conf, cov, reason = self._evaluate_frame(bbox_data)
            
            self.window.append({
                'valid': frame_valid,
                'conf': conf,
                'cov': cov,
                'reason': reason,
                'time': time.time()
            })
            
            valid_count = sum(1 for f in self.window if f['valid'])
            manuel_durdur = self._check_manuel_durdur()
            yeni_mod, degisti, dwell_elapsed = self._make_decision(valid_count, manuel_durdur)
            
            
            if degisti:
                eski_mod = self.current_mode
                self.current_mode = yeni_mod
                self._publish_mode()
                
                if manuel_durdur:
                    print(f"[KARAR] >>> MANUEL DURDURMA AKTİF: {eski_mod} → {yeni_mod}", flush=True)
                elif yeni_mod == 'konumlu':
                    print(f"[KARAR] >>> MOD DEĞİŞTİ: {eski_mod} → {yeni_mod} "
                        f"(dwell tamamlandı: {dwell_elapsed:.2f}s, pencere: {valid_count}/{self.WINDOW_SIZE})", flush=True)
                else:
                    print(f"[KARAR] >>> MOD DEĞİŞTİ: {eski_mod} → {yeni_mod} "
                        f"(pencere: {valid_count}/{self.WINDOW_SIZE} geçerli)", flush=True)
                
                # ─── MOD DEĞİŞİMİNDE PENCEREYİ SIFIRLA ───
                # Eski moddan kalan veriler yeni moddaki kararı kirletmesin
                # Pencere baştan dolacak, böylece tam sınırda salınma yaşanmaz
                self.window.clear()
                self.revert_pending_since = None   # Varsa pending dwell'i de iptal et
                print(f"[KARAR] Pencere ve dwell state sıfırlandı (mod geçişi sonrası temiz başlangıç)", flush=True)
            

            # ─── STATUS PRINT (değişim VEYA heartbeat) ───
            self._maybe_print_status(
                valid_count=valid_count,
                frame_valid=frame_valid,
                reason=reason,
                conf=conf,
                cov=cov,
                manuel_durdur=manuel_durdur,
                dwell_elapsed=dwell_elapsed,
                mod_degisti=degisti
            )


            if self.csv_writer:
                self.csv_writer.writerow([
                    f"{time.time():.4f}",
                    self.current_mode,
                    int(frame_valid),
                    f"{conf:.3f}",
                    f"{cov:.2f}",
                    reason,
                    valid_count,
                    f"{len(self.window)}/{self.WINDOW_SIZE}",
                    int(self.revert_pending_since is not None),
                    f"{dwell_elapsed:.2f}",
                    int(degisti)
                ])
                if len(self.window) % 30 == 0:
                    self.log_file.flush()
            
            return self.current_mode
    
    def run(self):
        """Ana döngü: Redis'i dinle, gelen her bbox için karar üret"""
        print("[KARAR] Aktif. Tracker bbox dinleniyor...", flush=True)
        try:
            while True:
                message = self.pubsub.get_message(
                    ignore_subscribe_messages=True, 
                    timeout=0.1
                )
                
                bbox_data = None
                if message and message['type'] == 'message':
                    try:
                        data_str = message['data'].decode('utf-8')
                        bbox_data = ast.literal_eval(data_str)
                    except Exception as e:
                        print(f"[KARAR] Bbox parse hatası: {e}", flush=True)
                        bbox_data = None
                
                if bbox_data is not None:
                    self.process_frame(bbox_data)
                    self.last_frame_time = time.time()
                else:
                    # Mesaj gelmedi — uzun süre tespit yoksa pencereye yapay 'tespit yok' ekle
                    now = time.time()
                    if (now - self.last_frame_time) > 0.1:
                        self.process_frame(None)
                        self.last_frame_time = now
        
        except KeyboardInterrupt:
            print("\n[KARAR] Kapatılıyor...", flush=True)
        finally:
            if self.log_file:
                self.log_file.close()
            try:
                self.pubsub.close()
            except Exception:
                pass


if __name__ == '__main__':
    decider = TransitionDecider()
    decider.run()
