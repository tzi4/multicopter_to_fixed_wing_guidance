#!/usr/bin/env python3
"""MASA DEMOSU: gercek gimbal + kamera ile SUREKLI HEDEF TAKIBI (Faz C IRL).

Mor bir cismi elinle yukari/asagi gezdir -> servo kamerayi cisme dondurur.
Sim'deki zincirin birebir kucultulmus hali:
    kare -> mor HSV tespiti -> ey [deg] -> hedef_elev = tilt_mevcut - ey
         -> TiltTakip (suzgec+slew+kelepce) -> MavlinkTiltKomutcu (ArduPilot)

VARSAYIMLAR (masa kosulu): drone sabit ve duz duruyor (roll/pitch ~ 0),
yani govde cercevesi = dunya cercevesi ve eklem acisi = tilt. Ucustaki tam
zincir (canli tutum + eklem_acisi()) bbox_to_redis'te; bu arac bilerek
yalin -- amaci donanim dogrulamasi.

KULLANIM:
  python3 tools/gimbal_bench_takip.py --kaynak 0 --baglanti udpin:0.0.0.0:14550
    --kaynak : cv2.VideoCapture kaynagi (0 = ilk kamera, dosya yolu,
               'udp://...', 'rtsp://...', ya da Pi uzerinde calisiyorsan 0)
    --baglanti: pymavlink adresi (MP acikken ayni portu PAYLASAMAZSINIZ;
               MP'yi kapatin ya da MP'den MAVLink Mirror acip ona baglanin)
  --kuru   : MAVLink YOK, yalnizca gorus+yasa dongusu (gelistirme testi)
  --goster : OpenCV penceresi (DISPLAY varsa)

Cikis: Ctrl-C. Guvenlik: tespit kaybolursa 3 s tutar, sonra 0 dereceye
yavasca doner; kelepce [-40, +55] (senin olctugun -44..+45 komut banti
icinde guvenli pay).
"""

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.gz_gimbal import TiltTakip                       # noqa: E402

# Kamera ici parametreler -- IMX500 1280x720 (yildizlar_gimbal ile ayni).
# Farkli cozunurluk gelirse fy orantiyla olceklenir.
HFOV_RAD = 1.1519
MOR_LO = (135, 90, 50)       # gercek IMX500 testinde calisan genis bant
MOR_HI = (165, 255, 255)


class PicamKaynak:
    """Donanimda calisan Picamera2/IMX500 kaynagi.

    Kamera govdeye 180 derece ters monte edildigi icin varsayilan donus hem
    goruntuyu hem de LOS isaretlerini birlikte duzeltir."""

    def __init__(self, genislik=1280, yukseklik=720, dondur_180=True):
        from picamera2 import Picamera2
        from libcamera import Transform
        self.cam = Picamera2()
        tr = Transform(hflip=1, vflip=1) if dondur_180 else Transform()
        cfg = self.cam.create_video_configuration(
            main={'size': (genislik, yukseklik), 'format': 'RGB888'},
            transform=tr)
        self.cam.configure(cfg)
        self.cam.start()

    def isOpened(self):
        return True

    def read(self):
        return True, self.cam.capture_array()

    def release(self):
        self.cam.stop()
        self.cam.close()


def mor_bul(kare):
    hsv = cv2.cvtColor(kare, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, MOR_LO, MOR_HI)
    cek = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cek)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cek)
    kon, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not kon:
        return None
    en = max(kon, key=cv2.contourArea)
    if cv2.contourArea(en) < 40:
        return None
    x, y, w, h = cv2.boundingRect(en)
    M = cv2.moments(en)
    if M['m00'] > 0:
        return M['m10'] / M['m00'], M['m01'] / M['m00'], w, h
    return x + w / 2.0, y + h / 2.0, w, h


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--kaynak', default='0')
    p.add_argument('--baglanti', default='udpin:0.0.0.0:14550')
    p.add_argument('--kuru', action='store_true',
                   help='MAVLink gondermeden yalniz gorus+yasa (test)')
    p.add_argument('--goster', action='store_true')
    p.add_argument('--duz', action='store_true',
                   help='kamerayi 180 dondurme (varsayilan ters montaj)')
    p.add_argument('--tilt-alt', type=float, default=-40.0)
    p.add_argument('--tilt-ust', type=float, default=55.0)
    p.add_argument('--sure', type=float, default=0.0,
                   help='saniye; 0 = Ctrl-C''ye kadar')
    a = p.parse_args()

    dosya_mi = not a.kaynak.isdigit() and a.kaynak != 'picam'
    cap = None
    if a.kaynak != 'picam':
        kaynak = int(a.kaynak) if a.kaynak.isdigit() else a.kaynak
        cap = cv2.VideoCapture(kaynak)
        ok = cap.isOpened()
        if ok and not dosya_mi:
            for _ in range(10):
                ok, _ = cap.read()
                if ok:
                    break
                time.sleep(0.1)
        if not ok:
            cap.release()
            cap = None
            if dosya_mi:
                raise SystemExit(f"HATA: kaynak acilamadi: {a.kaynak}")
            print("V4L2'den kare gelmedi -> Picamera2'ye geciliyor",
                  flush=True)
    if cap is None:
        cap = PicamKaynak(dondur_180=not a.duz)
        print(f"kamera: Picamera2 (IMX500) 1280x720"
              f"{'' if a.duz else ' (180 cevrildi)'}", flush=True)

    komutcu = None
    if not a.kuru:
        from tools.mavlink_tilt import MavlinkTiltKomutcu
        komutcu = MavlinkTiltKomutcu(
            a.baglanti, olu_bant_deg=0.1, min_aralik_s=0.02,
            servo_slew_dps=120.0, tick_s=0.01).basla(ilk_hedef_deg=0.0)
        print(f"MAVLink bagli: {a.baglanti}")

    takip = TiltTakip(varsayilan_deg=0.0, tau_s=0.25, slew_dps=60.0,
                      alt_deg=a.tilt_alt, ust_deg=a.tilt_ust,
                      kayip_tut_s=3.0, donus_dps=10.0)
    print(f"takip acik: kelepce [{a.tilt_alt:+.0f}, {a.tilt_ust:+.0f}] deg. "
          "Mor cismi dikeyde gezdir; Ctrl-C ile cik.")

    t_onceki = time.monotonic()
    son_rapor = 0.0
    t_bitis = None if a.sure <= 0 else time.monotonic() + a.sure
    iz = []                              # (t, tespit_var, hedef_elev, cmd)
    try:
        while t_bitis is None or time.monotonic() < t_bitis:
            ok, kare = cap.read()
            if not ok:
                if dosya_mi:                     # dosya bitti
                    break
                time.sleep(0.05)
                continue
            h_img = kare.shape[0]
            fy = (kare.shape[1] / 2.0) / math.tan(HFOV_RAD / 2.0)
            simdi = time.monotonic()
            dt = simdi - t_onceki
            t_onceki = simdi

            tespit = mor_bul(kare)
            if tespit is not None:
                mx, my, w, h = tespit
                # masa varsayimi: ey = kameradan sapma; hedefin dunya
                # yukselisi = mevcut tilt - ey  (govde duz -> eklem = tilt)
                ey = math.degrees(math.atan((my - h_img / 2.0) / fy))
                if abs(ey) < 0.4:
                    ey = 0.0
                hedef_elev = takip.cmd - ey
            else:
                hedef_elev = None
            cmd = takip.guncelle(hedef_elev, dt, simdi=simdi)
            if komutcu is not None:
                komutcu.hedef(cmd)
            iz.append((simdi, tespit is not None,
                       float('nan') if hedef_elev is None else hedef_elev, cmd))

            if simdi - son_rapor > 1.0:
                son_rapor = simdi
                durum = ('YOK' if tespit is None
                         else f"elev {hedef_elev:+6.2f}")
                print(f"  tespit {durum}  ->  tilt cmd {cmd:+6.2f} deg",
                      flush=True)
            if a.goster:
                if tespit is not None:
                    cv2.drawMarker(kare, (int(mx), int(my)), (0, 255, 255),
                                   cv2.MARKER_CROSS, 24, 2)
                cv2.putText(kare, f"tilt {cmd:+.1f}", (8, 28),
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 1)
                cv2.imshow('bench takip', kare)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if komutcu is not None:
            komutcu.hedef(0.0)
            time.sleep(0.5)
            komutcu.dur()
        cv2.destroyAllWindows()
    return iz


if __name__ == '__main__':
    iz = main()
    n_tespit = sum(1 for _, t, _, _ in iz if t)
    print(f"\nozet: {len(iz)} kare, {n_tespit} tespit; "
          f"tilt son {iz[-1][3]:+.2f} deg" if iz else "kare islenmedi")
