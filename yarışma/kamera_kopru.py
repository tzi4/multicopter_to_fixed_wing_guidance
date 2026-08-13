#!/usr/bin/env python3
"""
donanim/kamera_kopru.py - ROS'SUZ kamera -> Redis + gimbal koprusu
==================================================================
GERCEK DONANIMDAKI BOSLUK: sim'de 'tracker_bbox' / 'tracker_bbox_stab'
kanallarini bbox_to_redis.py uretiyor, ama o dosya bir ROS Image abonesidir.
Raspberry Pi 5'te ROS YOK; yani su an donanimda O IKI KANALI YAYINLAYACAK
KIMSE YOK. Gudum tarafi (guidance_allstar/goruntulu_temel.py,
donanim/gudum_tek_dugum.py) ise SADECE o kanallari dinler. Bu kopru o
boslugu kapatir: kamera (ya da sahte hedef) -> tespit -> sanal gimbal
zinciri -> AYNI iki Redis kanali -> (istege bagli) gercek gimbal komutu.

KOPYA YOK - HEPSI IMPORT:
  bbox_to_redis.TutumOkuyucu    MAVLink ATTITUDE, zaman damgali tampon +
                                karenin yakalandigi ana interpolasyon
  bbox_to_redis.hsv_tespit      mor/kirmizi HSV tespiti (sim ile ayni kod)
  bbox_to_redis.build_color_ranges   HSV pencereleri (+ YILDIZ_* env'leri)
  yildizlar_gimbal.SanalGimbal  roll de-rotasyonu, ic parametreler
  yildizlar_gimbal.eklem_acisi  kameranin dunya elevasyonu -> govde eklemi
  tools.gz_gimbal.TiltTakip     EMA + slew + kelepce + kayip politikasi
  tools.mavlink_tilt.MavlinkTiltKomutcu   GERCEK gimbal (Gazebo DEGIL)

Bu kopru KARAR VERICI DEGILDIR: 'komut_yetkisi' anahtarina DOKUNMAZ.
Sim'deki bbox_to_redis hem olcer hem angajman karari verir; donanimda karar
gudum dugumunundur (donanim/gudum_tek_dugum.py, AngajmanKapisi). Burasi
yalniz OLCUM yayinlar.

REDIS SOZLESMESI (bbox_to_redis ile BIREBIR - degistirilemez):
  tracker_bbox       [x, y, w, h, kapsama_pct, gecerli, t_capture]
  tracker_bbox_stab  [sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]
t_capture time.monotonic() tabanlidir (sim'de ROS saatiydi). Tuketiciler
onu kare IMZASI ve yas hesabi icin kullanir; monotonic her iki surecte de
ayni sistem saatidir (Linux CLOCK_MONOTONIC sistem capinda).

=== EN KRITIK TUZAK: COZUNURLUK ===
Piksel -> aci donusumu SanalGimbal'in ic parametreleriyle yapilir:
    fx = (genislik/2) / tan(hfov/2),  cx = genislik/2
Varsayilan cerceve 1280x720 / hfov 66 deg -> fx=985.5, cx=640.
Kameraya 1920x1080 beslersen ve cerceveyi guncellemezsen, kadrajin TAM
ORTASINDAKI hedef px=960'ta gorunur ve zincir onu
    ex = atan((960-640)/985.5) = +18.0 derece  saga sapmis
sanar. Hicbir hata mesaji vermez, gudum donup durur. SESSIZ FELAKET.
Bu yuzden: --genislik/--yukseklik/--hfov MUTLAKA SanalGimbal'e gecirilir
(bbox_to_redis onlari gecmiyor, biz geciyoruz) ve gelen kare farkli
boyuttaysa UYARI + yeniden olcekleme yapilir (--boyut-kati ile cikis).

=== UC KADEMELI DEVREYE ALMA (masada) ===
  1) kamerasiz, gimbalsiz : --dedektor sahte --kaynak dosya --no-gimbal
  2) kamerali, gimbalsiz  : --dedektor hsv   --kaynak cv2  --no-gimbal
  3) kamerali, gimbal ACIK: --dedektor hsv   --kaynak cv2  --mavlink <adres>
Ayrintili anlatim: donanim/README.md "kamera koprusu" bolumu.
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import redis                                                    # noqa: E402
from bbox_to_redis import (DEFAULT_TARGET_COLOR, build_color_ranges,  # noqa: E402
                           hsv_tespit, TutumOkuyucu, _opt_int)
from tools.gz_gimbal import TiltTakip                           # noqa: E402
from yildizlar_gimbal import SanalGimbal, eklem_acisi           # noqa: E402


# ===================================================================
# KARE KAYNAKLARI
# ===================================================================

class Cv2Kaynak:
    """cv2.VideoCapture: USB kamera (--cihaz N), video dosyasi, udp://, rtsp://

    Pi'nin libcamera yigininda VideoCapture(0) ACILMAYABILIR (bilinen tuzak,
    bkz. donanim/GIMBAL_TAKIP_TESTI.md). O durumda goruntuyu rpicam-vid ile
    UDP'ye basip --kaynak dosya --dosya udp://127.0.0.1:8554 kullan, ya da
    --kaynak picamera2 dene.
    """

    def __init__(self, hedef, genislik, yukseklik, dongu=False):
        self.hedef = hedef
        self.dongu = dongu
        self.cap = cv2.VideoCapture(hedef)
        if not self.cap.isOpened():
            raise SystemExit(f"HATA: kamera kaynagi acilamadi: {hedef!r}")
        if isinstance(hedef, int):
            # Cozunurlugu SURUCUYE de soyle. Surucu kabul etmezse kare gene
            # farkli boyutta gelir; ana dongu bunu yakalar ve olcekler.
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, genislik)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, yukseklik)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        print(f"[kaynak] cv2 {hedef!r} acildi, surucu {w}x{h} bildiriyor",
              flush=True)

    def oku(self):
        ok, kare = self.cap.read()
        if not ok:
            if self.dongu and not isinstance(self.hedef, int):
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, kare = self.cap.read()
                if ok:
                    return kare
            return None
        return kare

    def kapat(self):
        self.cap.release()


class Picamera2Kaynak:
    """Raspberry Pi kamera (libcamera). picamera2 yoksa ANLASILIR hata."""

    def __init__(self, genislik, yukseklik):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise SystemExit(
                f"HATA: --kaynak picamera2 icin picamera2 gerekiyor ({exc}).\n"
                "Pi'de:  sudo apt install -y python3-picamera2\n"
                "Alternatif: --kaynak cv2 --cihaz 0, ya da rpicam-vid ile "
                "UDP'ye basip --kaynak dosya --dosya udp://127.0.0.1:8554")
        self.picam = Picamera2()
        # 'RGB888' picamera2'de BELLEKTE BGR sirasindadir (bilinen ve
        # kafa karistirici adlandirma). cv2/HSV zinciri BGR bekledigi icin
        # dogru format budur; 'BGR888' verirsek kanallar TERS doner ve mor
        # hedef HSV penceresinden kacar (H 150 -> ~90 civari kayar).
        cfg = self.picam.create_video_configuration(
            main={"size": (genislik, yukseklik), "format": "RGB888"})
        self.picam.configure(cfg)
        self.picam.start()
        self.son_metadata = {}
        print(f"[kaynak] picamera2 {genislik}x{yukseklik} basladi", flush=True)

    def oku(self):
        istek = self.picam.capture_request()
        try:
            kare = istek.make_array("main")
            self.son_metadata = istek.get_metadata()
        finally:
            istek.release()
        return kare

    def kapat(self):
        try:
            self.picam.stop()
        except Exception:
            pass


class SentetikKaynak:
    """KAMERASIZ kare uretici: gri gokyuzu + istenen yere MOR dikdortgen.

    NEDEN VAR: kademe-1 masa testi (kamera yok, gimbal yok) tum zinciri --
    sanal gimbal, Redis yayini, tilt takip yasasi, log -- kossun diye. Ayrica
    'sahte' ve 'hsv' dedektorlerini YAN YANA dogrulamayi mumkun kilar: ayni
    karede sahte kutu ile HSV'nin buldugu kutu ortusmeli.
    """

    def __init__(self, genislik, yukseklik, bbox=None):
        self.w, self.h = genislik, yukseklik
        self.bbox = bbox
        self._temel = np.zeros((yukseklik, genislik, 3), np.uint8)
        self._temel[:] = (140, 120, 100)          # BGR: soluk mavi-gri gokyuzu

    def oku(self):
        kare = self._temel.copy()
        if self.bbox is not None:
            x, y, w, h = [int(v) for v in self.bbox]
            # Gazebo/Purple = RGB(1,0,1) -> BGR (255,0,255), HSV H=150:
            # bbox_to_redis'in PURPLE penceresinin (H 140-160) tam ortasi.
            cv2.rectangle(kare, (x, y), (x + w, y + h), (255, 0, 255), -1)
        return kare

    def kapat(self):
        pass


# ===================================================================
# DEDEKTORLER  ->  (x, y, w, h) ya da None, ISLEME CERCEVESI pikselinde
# ===================================================================

class HsvDedektor:
    """bbox_to_redis.hsv_tespit'i cagirir (ayni kod, ayni esikler)."""

    def __init__(self, renk, min_blob_h=0.0, min_blob_fill=0.0):
        self.renk, self.pencereler = build_color_ranges(
            renk, _opt_int('YILDIZ_HSV_SMIN'), _opt_int('YILDIZ_HSV_VMIN'))
        self.min_blob_h = float(min_blob_h)
        self.min_blob_fill = float(min_blob_fill)
        ozet = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                          for lo, hi in self.pencereler)
        print(f"[dedektor] HSV, hedef rengi {self.renk.upper()} -> {ozet}",
              flush=True)

    def bul(self, kare):
        return hsv_tespit(kare, self.pencereler,
                          self.min_blob_h, self.min_blob_fill)


class SahteDedektor:
    """Kamerasiz: hedefi kadrajin ISTENEN yerine 'koyar'.

    NEDEN VAR (kullanicinin istegi): "hedefi kadrajin su noktasina koyunca
    zincir ne diyor / MPC nereye komut veriyor" sorusu masada, ucus olmadan
    cevaplanabilsin. Kutu sabittir; tum alt zincir (stabilizasyon + Redis +
    gimbal komutu) gercekmis gibi calisir.

    ISARET SOZLESMESI (sahada bunu ezberle):
      kutu SAGDA  (x > cx) -> ex > 0
      kutu YUKARIDA (y < cy) -> ey < 0  ve hedef yukselisi (-ey) > 0
    """

    def __init__(self, bbox, genislik, yukseklik):
        if bbox is None:
            # Varsayilan: kadrajin TAM ORTASI. Beklenen sonuc ex~0, ey~0;
            # sifir olmayan bir deger gorursen ic parametreler yanlistir.
            w, h = 60, 30
            bbox = (genislik // 2 - w // 2, yukseklik // 2 - h // 2, w, h)
        self.bbox = tuple(int(v) for v in bbox)
        print(f"[dedektor] SAHTE bbox={self.bbox} (kamera kullanilmiyor)",
              flush=True)

    def bul(self, kare):
        return self.bbox


class YoloDedektor:
    """ultralytics YOLO. Cikti bbox'i ISLEME CERCEVESINE geri olceklenmis olur.

    === KLASIK HATA (bu yuzden bu yorum uzun) ===
    YOLO agi 640x640 gibi SABIT bir girise letterbox (en-boy koruyarak
    olcekleme + gri dolgu) ile besleniyor. Agin verdigi kutular O 640'lik
    tuvalin pikselindedir. Onlari cerceveye geri tasimak icin letterbox'in
    TERSI gerekir:
        olcek = min(640/W, 640/H);  dolgu_x = (640 - W*olcek)/2  ...
        x_cerceve = (x_640 - dolgu_x) / olcek
    Bu geri donusum ATLANIRSA hedef kadrajin sol ustune kayar ve ex/ey
    ~2 kat kucuk okunur -- gudum hedefi surekli 'daha yakinda ve daha
    ortada' sanir. Hicbir hata mesaji cikmaz.

    ultralytics'in predict()'i bu geri donusumu KENDISI yapar ve kutulari
    verdigin karenin pikselinde dondurur; sart, kareyi OLDUGU GIBI vermek
    (kendin resize edip sonra geri olceklemeyi unutma tuzagi). Biz de tam
    olarak isleme cercevesini veriyoruz, dolayisiyla cikti zaten o cercevede.
    Yine de her karede SAGLAMA yapiyoruz: kutu cerceve disina tasarsa
    sessizce yanlis aci uretmek yerine BAGIRIYORUZ.
    """

    def __init__(self, model_yolu, conf=0.35, sinif=None, imgsz=640):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise SystemExit(
                f"HATA: --dedektor yolo icin ultralytics gerekiyor ({exc}).\n"
                "  pip install ultralytics\n"
                "Pi AI Camera (IMX500) kullanacaksan --yolo-model <ag>.rpk "
                "ver (agi sensor kosar, ultralytics gerekmez).")
        try:
            self.model = YOLO(model_yolu)
        except Exception as exc:
            raise SystemExit(
                f"HATA: YOLO modeli yuklenemedi: {model_yolu}\n  ({exc})\n"
                "--yolo-model ile agirlik dosyasinin TAM YOLUNU ver. Pi'de "
                "internet yoksa ultralytics modeli indiremez; agirligi once "
                "elle kopyala.")
        self.conf = float(conf)
        self.sinif = sinif
        self.imgsz = int(imgsz)
        self._tasma_uyarisi = False
        print(f"[dedektor] YOLO {model_yolu} conf={self.conf} "
              f"imgsz={self.imgsz} sinif={self.sinif}", flush=True)

    def bul(self, kare):
        H, W = kare.shape[:2]
        r = self.model.predict(kare, imgsz=self.imgsz, conf=self.conf,
                               classes=self.sinif, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return None
        # En GUVENLI degil, en BUYUK kutu (HSV yolundaki 'en buyuk kontur'
        # ile ayni kural: hedef yaklastikca buyur, parazit kucuk kalir).
        xyxy = r.boxes.xyxy.cpu().numpy()
        alanlar = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        x1, y1, x2, y2 = xyxy[int(np.argmax(alanlar))]
        if not self._tasma_uyarisi and (x2 > W + 2 or y2 > H + 2
                                        or x1 < -2 or y1 < -2):
            self._tasma_uyarisi = True
            print(f"UYARI: YOLO kutusu ({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                  f"isleme cercevesinin ({W}x{H}) DISINDA. Letterbox geri "
                  f"donusumu bozuk demektir -- uretilen ex/ey GUVENILMEZ.",
                  flush=True)
        x1 = max(0.0, min(float(x1), W - 1.0))
        y1 = max(0.0, min(float(y1), H - 1.0))
        x2 = max(0.0, min(float(x2), W - 1.0))
        y2 = max(0.0, min(float(y2), H - 1.0))
        if x2 - x1 < 1 or y2 - y1 < 1:
            return None
        return int(x1), int(y1), int(x2 - x1), int(y2 - y1)


class Imx500Dedektor:
    """Raspberry Pi AI Camera (IMX500): ag SENSORDE kosar, kutular metadata'da.

    DONANIMDA HENUZ DOGRULANMADI (bu depoda IMX500 yok). Yapi ve olcek geri
    donusumu bilerek acik yazildi; Pi'de ilk kosuda 'sahte' ve 'hsv' kollariyla
    karsilastirilarak dogrulanmali.

    OLCEK KURALI: IMX500 API'si kutulari agin GIRIS TUVALINDE (or. 640x640)
    verir; get_outputs(..., add_batch=True) + convert_inference_coords()
    donusumu ISR/olcekleme farkini kapatir. Biz ayrica sonucu isleme
    cercevesine kelepceleriz -- YoloDedektor ile ayni gerekce.
    """

    def __init__(self, rpk_yolu, kaynak, conf=0.35, sinif=None):
        if not isinstance(kaynak, Picamera2Kaynak):
            raise SystemExit(
                "HATA: IMX500 (.rpk) dedektoru --kaynak picamera2 ister "
                "(ag sensorde kosar, kutular kare metadata'sindan gelir).")
        try:
            from picamera2.devices import IMX500
            from picamera2.devices.imx500 import postprocess_nanodet_detection  # noqa: F401
        except ImportError as exc:
            raise SystemExit(
                f"HATA: IMX500 destegi yok ({exc}). Pi'de:\n"
                "  sudo apt install -y imx500-all python3-picamera2")
        self.imx = IMX500(rpk_yolu)
        self.kaynak = kaynak
        self.conf = float(conf)
        self.sinif = sinif
        print(f"[dedektor] IMX500 {rpk_yolu} conf={self.conf}", flush=True)

    def bul(self, kare):
        md = self.kaynak.son_metadata or {}
        cikti = self.imx.get_outputs(md, add_batch=True)
        if cikti is None:
            return None
        kutular, skorlar, siniflar = cikti[0][0], cikti[1][0], cikti[2][0]
        H, W = kare.shape[:2]
        en_iyi, en_alan = None, 0.0
        for kutu, skor, snf in zip(kutular, skorlar, siniflar):
            if skor < self.conf:
                continue
            if self.sinif is not None and int(snf) not in self.sinif:
                continue
            # convert_inference_coords: ag tuvali -> KARE pikseli (olcek geri
            # donusumu burada yapilir; elle carpma YAPMA).
            x, y, w, h = self.imx.convert_inference_coords(kutu, md,
                                                           self.kaynak.picam)
            x = max(0, min(int(x), W - 1))
            y = max(0, min(int(y), H - 1))
            w = max(1, min(int(w), W - x))
            h = max(1, min(int(h), H - y))
            if w * h > en_alan:
                en_iyi, en_alan = (x, y, w, h), w * h
        return en_iyi


# ===================================================================
# COZUNURLUK KAPISI
# ===================================================================

class BoyutKapisi:
    """Gelen kareyi isleme cercevesine oturtur; ilk uyusmazlikta BAGIRIR.

    Bu sinif modul docstring'indeki 'SESSIZ FELAKET'in tek savunmasidir:
    kadrajin ortasindaki hedefin kac derece sapmis GORUNECEGINI hesaplayip
    uyariya basar, boylece sahada 'niye hedef hep saga kayik' sorusu
    tahmin degil olcum olur.
    """

    def __init__(self, genislik, yukseklik, fx, kati=False):
        self.W, self.H, self.fx, self.kati = genislik, yukseklik, fx, kati
        self.uyarildi = False
        self.olcekleme_n = 0

    def uydur(self, kare):
        h, w = kare.shape[:2]
        if (w, h) == (self.W, self.H):
            return kare
        if not self.uyarildi:
            self.uyarildi = True
            sapma = math.degrees(math.atan((w / 2.0 - self.W / 2.0) / self.fx))
            mesaj = (f"UYARI: kare {w}x{h} geldi, isleme cercevesi "
                     f"{self.W}x{self.H}. Olceklenmeseydi kadrajin TAM "
                     f"ORTASINDAKI hedef ex={sapma:+.1f} deg sapmis "
                     f"gorunurdu (cx={self.W/2:.0f}, fx={self.fx:.1f}). "
                     f"Dogrusu: --genislik {w} --yukseklik {h} verip DOGRU "
                     f"--hfov'u da vermek.")
            if self.kati:
                raise SystemExit("HATA (--boyut-kati): " + mesaj)
            print(mesaj + " Simdilik kare yeniden olcekleniyor.", flush=True)
        self.olcekleme_n += 1
        return cv2.resize(kare, (self.W, self.H),
                          interpolation=cv2.INTER_AREA)


# ===================================================================
# KOPRU
# ===================================================================

class KameraKopru:

    def __init__(self, a):
        self.a = a
        self.W, self.H = int(a.genislik), int(a.yukseklik)
        self.gimbal_acik = bool(a.mavlink) and not a.no_gimbal

        # --- SANAL GIMBAL: cozunurluk ve hfov MUTLAKA buraya gecer ---
        # (bbox_to_redis bunlari gecmiyor, varsayilan 1280x720 kaliyordu.)
        # mount: fiziksel gimbal ACIKSA 0 -- kameranin govdeye gore acisi
        # sabit degil, her karede eklem_acisi() ile CANLI hesaplanir.
        # Gimbal kapaliysa kamera govdeye sabittir, --mount gecerlidir.
        self.gimbal = SanalGimbal(
            width=self.W, height=self.H, hfov_rad=math.radians(a.hfov),
            mount_phys_pitch_deg=(0.0 if self.gimbal_acik else a.mount),
            aim_pitch_deg=0.0)
        # aim=0 BILEREK: aim bir DC ofsettir ve dikey referansi kaydirir.
        # Bu kopru ham olcum yayinlar; hedefin kadrajda nerede durmasi
        # gerektigine gudum (mpc_gudum ey_ref) karar verir.
        print("[gimbal] " + self.gimbal.ozet(), flush=True)

        self.boyut = BoyutKapisi(self.W, self.H, self.gimbal.fx, a.boyut_kati)

        # --- REDIS ---
        self.r = redis.Redis(host=a.redis_host, port=a.redis_port, db=0)
        try:
            self.r.ping()
        except Exception as exc:
            raise SystemExit(
                f"Redis baglanti hatasi ({a.redis_host}:{a.redis_port}): {exc}\n"
                "Pi'de:  sudo systemctl start redis-server")
        print(f"[redis] {a.redis_host}:{a.redis_port} bagli; kanallar "
              f"'tracker_bbox' + 'tracker_bbox_stab'", flush=True)
        # 'komut_yetkisi'ne DOKUNULMAZ: bu kopru karar verici degildir.

        # --- KAYNAK ---
        self.kaynak = self._kaynak_kur()
        # --- DEDEKTOR ---
        self.dedektor = self._dedektor_kur()

        # --- MAVLINK: tutum + gimbal (TEK PAYLASILAN BAGLANTI) ---
        self.tutum = None
        self.komutcu = None
        self.takip = None
        self._mav = None
        if a.mavlink:
            self._mavlink_kur()

        # --- LOG ---
        self._log_f = self._log_w = None
        self._log_n = 0
        if a.log:
            self._log_f = open(a.log, 'w', newline='')
            self._log_w = csv.writer(self._log_f)
            # TESHIS KOLONLARI (donanim/README.md'de nasil okunacagi yazili):
            # ham_* yanlis  -> ic parametre / cozunurluk / tespit sorunu
            # ham dogru, stab yanlis -> tutum, roll isareti ya da zaman senkronu
            self._log_w.writerow([
                't', 'tilt_cmd_deg', 'tilt_status_deg',
                'ham_ex_deg', 'ham_ey_deg', 'stab_ex_deg', 'stab_ey_deg',
                'bbox_w', 'bbox_h', 'gecerli', 'fps'])
            print(f"[log] {a.log}", flush=True)

        # --- video kaydi ---
        self.yazici = None
        self._kayit_yolu = a.kaydet
        self._kayit_tampon = []

        # --- olcum/durum ---
        self._kare_t = deque(maxlen=60)
        self._t_ilk = None
        self._son_ozet = 0.0
        self._takip_son_t = None
        self.kare_n = 0
        self.tespit_n = 0
        self.yayin_n = 0
        self._dur = False

    # ------------------------------------------------------------ kurulum

    def _kaynak_kur(self):
        a = self.a
        if a.kaynak == 'picamera2':
            return Picamera2Kaynak(self.W, self.H)
        if a.kaynak == 'cv2':
            return Cv2Kaynak(int(a.cihaz), self.W, self.H, dongu=False)
        # 'dosya': yol verilmezse SENTETIK kare (kamerasiz masa testi)
        if not a.dosya:
            bbox = a.sahte_bbox_coz
            if bbox is None:
                w, h = 60, 30
                bbox = (self.W // 2 - w // 2, self.H // 2 - h // 2, w, h)
            print("[kaynak] --dosya verilmedi -> SENTETIK kare uretiliyor "
                  f"({self.W}x{self.H}, mor kutu {tuple(bbox)})", flush=True)
            return SentetikKaynak(self.W, self.H, bbox)
        return Cv2Kaynak(a.dosya, self.W, self.H, dongu=a.dongu)

    def _dedektor_kur(self):
        a = self.a
        if a.dedektor == 'sahte':
            return SahteDedektor(a.sahte_bbox_coz, self.W, self.H)
        if a.dedektor == 'yolo':
            if str(a.yolo_model).endswith('.rpk'):
                return Imx500Dedektor(a.yolo_model, self.kaynak,
                                      a.yolo_conf, a.yolo_sinif)
            return YoloDedektor(a.yolo_model, a.yolo_conf, a.yolo_sinif,
                                a.yolo_imgsz)
        return HsvDedektor(a.renk,
                           float(os.environ.get('YILDIZ_MIN_BLOB_H', '0') or 0),
                           float(os.environ.get('YILDIZ_MIN_BLOB_FILL', '0') or 0))

    def _mavlink_kur(self):
        """TEK baglanti: tutum okuyucu (recv) + tilt komutcusu (send).

        NEDEN TEK: 'udpin:...' adresini iki kez acmak PORTA IKI KEZ BIND
        etmektir (ikincisi hata verir); seri portu iki kez acmak bozuk
        cerceve uretir. mavlink_tilt.MavlinkTiltKomutcu'nun sozlesmesi de
        bunu soyluyor: komutcu.basla() TEK recv'ini senkron yapar, ondan
        SONRA baska bir thread tek recv tuketicisi olabilir. Sira burada
        bilerek boyle: once komutcu.basla(), sonra tutum.start().
        """
        a = self.a
        from pymavlink import mavutil
        print(f"[mavlink] baglaniliyor: {a.mavlink}", flush=True)
        self._mav = mavutil.mavlink_connection(a.mavlink, source_system=250)
        self._mav.wait_heartbeat(timeout=30)
        print(f"[mavlink] heartbeat alindi (sys={self._mav.target_system})",
              flush=True)

        if self.gimbal_acik:
            from tools.mavlink_tilt import MavlinkTiltKomutcu
            self.komutcu = MavlinkTiltKomutcu(self._mav).basla(
                ilk_hedef_deg=a.tilt)
            if a.tilt_sabit:
                print(f"[gimbal] takip KAPALI (--tilt-sabit): tilt "
                      f"{a.tilt:+.1f} deg sabit", flush=True)
            else:
                # TiltTakip: EMA (tau) + slew siniri + kelepce + kayip
                # politikasi. Girdi hedefin OLCULEN dunya yukselisi (-ey);
                # ey ufka gore olculdugu icin bu KAPALI DONGU DEGIL, olculen
                # buyuklugun suzgecli takibidir (kararlilik riski yok).
                self.takip = TiltTakip(varsayilan_deg=a.tilt, tau_s=a.tilt_tau,
                                       slew_dps=a.tilt_slew,
                                       alt_deg=a.tilt_alt, ust_deg=a.tilt_ust,
                                       kayip_tut_s=3.0, donus_dps=10.0)
                print(f"[gimbal] takip ACIK: varsayilan {a.tilt:+.1f} deg, "
                      f"kelepce [{a.tilt_alt:+.0f}, {a.tilt_ust:+.0f}], "
                      f"slew {a.tilt_slew:.0f} deg/s", flush=True)
        else:
            print("[gimbal] --no-gimbal: tilt komutu YOK, kamera govdeye "
                  f"sabit varsayiliyor (--mount {a.mount:+.1f} deg)",
                  flush=True)

        self.tutum = TutumOkuyucu(a.mavlink, gecikme_s=a.kamera_gecikme_ms / 1000.0,
                                  mav=self._mav)
        self.tutum.start()
        print(f"[mavlink] tutum okuyucu basladi (kamera gecikmesi "
              f"{a.kamera_gecikme_ms:.0f} ms geriye tasinir)", flush=True)

    # ------------------------------------------------------------- yardim

    def _tilt_eps(self):
        """Kameranin DUNYA elevasyonu [deg] ya da None (gimbal kapali).

        GERCEK DONANIM SINIRI: bu kopru mount'tan bagimsiz aci GERI BESLEMESI
        okumaz (sim'deki gimbal_tilt_status'un karsiligi MOUNT_ORIENTATION
        olurdu; onu okumak ayni baglantida IKINCI bir recv tuketicisi ister
        ve pymavlink thread-safe degildir). Bu yuzden 'status' = son
        YAYINLANAN komut. ArduPilot mount surucusu komutu <1 s'de oturttugu
        icin yavas DC rejimde guvenli; hizli terminal manevrada gecikme
        payi vardir. ACIK IS: MOUNT_ORIENTATION okuyucusu.
        """
        if self.komutcu is None:
            return None
        return (self.komutcu.yayinlanan_deg
                if self.komutcu.yayinlanan_deg is not None
                else self.komutcu.hedef_deg)

    def _tutum_al(self, t_kare):
        """(roll_rad, pitch_rad) - tutum yoksa (0, 0) = masa varsayimi."""
        if self.tutum is None or not self.tutum.hazir:
            return 0.0, 0.0
        ornek = self.tutum.tutum_al(t_kare)
        if ornek is None:
            return 0.0, 0.0
        return ornek[0], ornek[1]

    def _fps(self):
        if len(self._kare_t) < 2:
            return 0.0
        return (len(self._kare_t) - 1) / max(
            self._kare_t[-1] - self._kare_t[0], 1e-6)

    # --------------------------------------------------------------- ana

    def calistir(self):
        a = self.a
        t_bitis = None if a.sure <= 0 else time.monotonic() + a.sure
        periyot = 0.0 if a.fps <= 0 else 1.0 / a.fps
        son_kare_t = 0.0
        while not self._dur and (t_bitis is None or time.monotonic() < t_bitis):
            if periyot > 0:
                bekle = periyot - (time.monotonic() - son_kare_t)
                if bekle > 0:
                    time.sleep(bekle)
            son_kare_t = time.monotonic()

            kare = self.kaynak.oku()
            if kare is None:
                print("[kaynak] kare yok (dosya bitti / kamera kesildi)",
                      flush=True)
                break
            self._kare_isle(kare)
        return 0

    def _kare_isle(self, kare):
        # TEK ORTAK SAAT: kareler de tutum ornekleri de time.monotonic().
        # Redis'e basilan t_capture da bu saattir (bkz. modul docstring'i).
        t_kare = time.monotonic()
        kare = self.boyut.uydur(kare)
        self.kare_n += 1
        self._kare_t.append(t_kare)
        if self._t_ilk is None:
            self._t_ilk = t_kare
            print(f"ILK KARE: {kare.shape[1]}x{kare.shape[0]}", flush=True)

        kutu = self.dedektor.bul(kare)
        gecerli = kutu is not None

        ham_ex = ham_ey = ex = ey = None
        sx = sy = None
        menzil = None
        if gecerli:
            self.tespit_n += 1
            x, y, w, h = [int(v) for v in kutu]
            mx, my = x + w / 2.0, y + h / 2.0
            kapsama = (w / float(self.W)) * 100.0

            # --- KANAL 1: HAM bbox (sozlesme bbox_to_redis ile birebir) ---
            self.r.publish('tracker_bbox', json.dumps(
                [x, y, w, h, round(kapsama, 3), 1, round(t_kare, 4)]))

            roll, pitch = self._tutum_al(t_kare)
            eps = self._tilt_eps()
            # FIZIKSEL GIMBAL: sabit mount yerine CANLI eklem acisi.
            # eklem None -> SanalGimbal eski sabit-mount yoluna duser.
            eklem = (None if eps is None
                     else eklem_acisi(eps, pitch, roll))
            menzil = self.gimbal.menzil_tahmin(w, self.a.hedef_genislik_m)

            sx, sy = self.gimbal.stabilize(mx, my, roll, pitch, menzil,
                                           eklem_deg=eklem)
            ex, ey = self.gimbal.aci_hatasi(mx, my, roll, pitch, menzil,
                                            eklem_deg=eklem)
            # HAM hata: hicbir de-rotasyon olmasaydi okunacak aci. Teshiste
            # bu kolon ile stab kolonunun AYRISMASI arizayi ikiye boler.
            ham_ex = math.degrees(math.atan((mx - self.gimbal.cx) / self.gimbal.fx))
            ham_ey = math.degrees(math.atan((my - self.gimbal.cy) / self.gimbal.fy))

            # --- KANAL 2: STABILIZE (8. eleman = o karede kullanilan
            #     kamera elevasyonu; mpc_gudum ey_ref'i bundan kurar) ---
            self.r.publish('tracker_bbox_stab', json.dumps(
                [round(sx, 2), round(sy, 2), int(w), int(h),
                 round(ex, 4), round(ey, 4), round(t_kare, 4),
                 None if eps is None else round(eps, 3)]))
            self.yayin_n += 1

        # --- TILT TAKIBI (tespit varsa hedefe, yoksa kayip politikasina) ---
        if self.takip is not None and self.komutcu is not None:
            simdi = time.monotonic()
            dt = (1.0 / 30 if self._takip_son_t is None
                  else simdi - self._takip_son_t)
            self._takip_son_t = simdi
            # ey ufka gore olculur -> hedefin dunya yukselisi = -ey.
            self.komutcu.hedef(self.takip.guncelle(
                None if not gecerli else -ey, dt, simdi=simdi))

        self._logla(t_kare, ham_ex, ham_ey, ex, ey, kutu, gecerli)
        self._ozet(t_kare, ex, ey, kutu, gecerli)
        if self.a.goster or self._kayit_yolu is not None:
            self._ciz(kare, kutu, ex, ey, menzil)

    # -------------------------------------------------------------- cikti

    def _logla(self, t, ham_ex, ham_ey, ex, ey, kutu, gecerli):
        if self._log_w is None:
            return
        cmd = None if self.komutcu is None else self.komutcu.hedef_deg
        st = self._tilt_eps()

        def _s(v, f='{:.4f}'):
            return '' if v is None else f.format(v)

        self._log_w.writerow([
            f"{t:.6f}", _s(cmd, '{:.3f}'), _s(st, '{:.3f}'),
            _s(ham_ex), _s(ham_ey), _s(ex), _s(ey),
            '' if kutu is None else int(kutu[2]),
            '' if kutu is None else int(kutu[3]),
            1 if gecerli else 0, f"{self._fps():.2f}"])
        # PERIYODIK FLUSH: cakilma/SIGKILL'de kaza anina en yakin blok
        # kaybolmasin (bbox_to_redis'teki ayni ders, 20 satirda bir).
        self._log_n += 1
        if self._log_n % 20 == 0:
            self._log_f.flush()

    def _ozet(self, t, ex, ey, kutu, gecerli):
        """~1 s'de bir stderr'e TEK satir. stdout'a degil: stdout'u boru
        hattina veren bir kurulumda ozet veriyle karismasin."""
        if t - self._son_ozet < 1.0:
            return
        self._son_ozet = t
        cmd = None if self.komutcu is None else self.komutcu.hedef_deg
        st = self._tilt_eps()
        kutu_s = ('-' if kutu is None
                  else f"({kutu[0]},{kutu[1]},{kutu[2]},{kutu[3]})")
        tilt_s = (f"{'-' if cmd is None else f'{cmd:+.1f}'}"
                  f"/{'-' if st is None else f'{st:+.1f}'}")
        oran = (100.0 * self.tespit_n / self.kare_n) if self.kare_n else 0.0
        print(f"[KOPRU] fps={self._fps():5.1f} kare={self.kare_n} "
              f"tespit=%{oran:.0f} bbox={kutu_s} "
              f"ex={'-' if ex is None else f'{ex:+6.2f}'} "
              f"ey={'-' if ey is None else f'{ey:+6.2f}'} deg "
              f"tilt(cmd/st)={tilt_s} deg "
              f"{'' if gecerli else '[TESPIT YOK]'}",
              file=sys.stderr, flush=True)

    def _ciz(self, kare, kutu, ex, ey, menzil):
        F, AA = cv2.FONT_HERSHEY_DUPLEX, cv2.LINE_AA
        cv2.drawMarker(kare, (int(self.gimbal.cx), int(self.gimbal.cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 22, 1)
        if kutu is not None:
            x, y, w, h = kutu
            cv2.rectangle(kare, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.drawMarker(kare, (x + w // 2, y + h // 2), (255, 0, 255),
                           cv2.MARKER_CROSS, 12, 1)
        cmd = None if self.komutcu is None else self.komutcu.hedef_deg
        satir = (f"ex {'-' if ex is None else f'{ex:+.2f}'}  "
                 f"ey {'-' if ey is None else f'{ey:+.2f}'} deg  "
                 f"tilt {'-' if cmd is None else f'{cmd:+.1f}'}  "
                 f"fps {self._fps():.1f}")
        cv2.putText(kare, satir, (8, 24), F, 0.55, (0, 255, 255), 1, AA)
        if menzil is not None:
            cv2.putText(kare, f"~{menzil:.0f} m (bbox'tan, kaba)",
                        (8, kare.shape[0] - 12), F, 0.45, (200, 200, 200), 1, AA)
        if self._kayit_yolu is not None:
            self._kaydet(kare)
        if self.a.goster:
            cv2.imshow('kamera_kopru', kare)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self._dur = True

    def _kaydet(self, kare):
        """Yazici GERCEK olculen hizla kurulur.

        DERS (bbox_to_redis 2026-08-01): ilk karede kurulan yazici basliga
        5 fps yaziyordu ve 30 Hz kayitlar 6 KAT agir cekim oynuyordu. Once
        2 s kare biriktir, hizi OLC, sonra yaziciyi kur.
        """
        if self.yazici is None:
            gecen = time.monotonic() - (self._t_ilk or time.monotonic())
            if gecen < 2.0:
                self._kayit_tampon.append(kare.copy())
                return
            fps = max(1.0, min(60.0, self.kare_n / max(gecen, 1e-3)))
            self.yazici = cv2.VideoWriter(
                self._kayit_yolu, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (kare.shape[1], kare.shape[0]))
            print(f"[kayit] {self._kayit_yolu} ({kare.shape[1]}x"
                  f"{kare.shape[0]} @ {fps:.1f} fps, "
                  f"{len(self._kayit_tampon)} kare tamponundan)", flush=True)
            for k in self._kayit_tampon:
                self.yazici.write(k)
            self._kayit_tampon = []
        self.yazici.write(kare)

    # ------------------------------------------------------------- kapan

    def kapat(self):
        self._dur = True
        if self.komutcu is not None:
            # Cikista varsayilan (standoff/duz) poza don: gimbal en son
            # komutlanan acida ASILI KALMASIN.
            try:
                self.komutcu.hedef(self.a.tilt)
                time.sleep(0.4)
            except Exception:
                pass
            self.komutcu.dur()
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
        # KISA KOSU TUZAGI: yazici ancak 2 s'lik hiz olcumunden SONRA
        # kuruluyor; daha kisa bir kosu (or. --sure 1.5) aksi halde hic
        # dosya uretmezdi ve "kayit calismiyor" sanilirdi. Tamponda kare
        # kaldiysa burada olculen hizla bosaltilir.
        if self.yazici is None and self._kayit_tampon:
            gecen = max(time.monotonic() - (self._t_ilk or 0.0), 1e-3)
            fps = max(1.0, min(60.0, self.kare_n / gecen))
            k0 = self._kayit_tampon[0]
            self.yazici = cv2.VideoWriter(
                self._kayit_yolu, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (k0.shape[1], k0.shape[0]))
            print(f"[kayit] kisa kosu: {len(self._kayit_tampon)} kare "
                  f"tampondan {fps:.1f} fps ile yaziliyor", flush=True)
            for k in self._kayit_tampon:
                self.yazici.write(k)
            self._kayit_tampon = []
        if self.yazici is not None:
            self.yazici.release()
            print(f"[kayit] kaydedildi: {self._kayit_yolu}", flush=True)
        try:
            self.kaynak.kapat()
        except Exception:
            pass
        cv2.destroyAllWindows()
        oran = (100.0 * self.tespit_n / self.kare_n) if self.kare_n else 0.0
        print(f"[OZET] kare={self.kare_n} tespit={self.tespit_n} (%{oran:.1f}) "
              f"stab_yayin={self.yayin_n} "
              f"olceklenen_kare={self.boyut.olcekleme_n}", flush=True)


# ===================================================================
# CLI
# ===================================================================

def _sahte_bbox_coz(metin):
    if not metin:
        return None
    try:
        p = [int(float(v)) for v in metin.replace(' ', '').split(',')]
    except ValueError:
        raise SystemExit(f"HATA: --sahte-bbox '{metin}' cozulemedi; "
                         "bicim: x,y,w,h (or. 640,360,60,30)")
    if len(p) != 4:
        raise SystemExit(f"HATA: --sahte-bbox 4 sayi ister (x,y,w,h), "
                         f"{len(p)} geldi")
    return tuple(p)


def arg_ayristir(argv=None):
    """CLI ayristirma AYRI: cevrimdisi dogrulama harness'i (kamera/MAVLink
    olmadan KameraKopru kurup _kare_isle'yi surmek) ayni varsayilanlari
    kullanabilsin diye. Elle Namespace kurmak varsayilanlarin ikinci bir
    kopyasini yaratirdi."""
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--kaynak', default='cv2',
                   choices=['picamera2', 'cv2', 'dosya'],
                   help='kare kaynagi (dosya + --dosya yoksa SENTETIK kare)')
    p.add_argument('--cihaz', type=int, default=0,
                   help='--kaynak cv2 icin kamera indeksi')
    p.add_argument('--dosya', default=None,
                   help='video dosyasi / udp:// / rtsp:// (--kaynak dosya)')
    p.add_argument('--dongu', action='store_true',
                   help='video dosyasi bitince bastan basla')
    p.add_argument('--genislik', type=int, default=1280)
    p.add_argument('--yukseklik', type=int, default=720)
    p.add_argument('--hfov', type=float, default=66.0,
                   help='yatay gorus acisi [derece]. IMX500 1280x720 -> 66')
    p.add_argument('--boyut-kati', action='store_true',
                   help='gelen kare cerceveden farkliysa olcekleme, CIK')
    p.add_argument('--dedektor', default='hsv', choices=['hsv', 'yolo', 'sahte'])
    p.add_argument('--sahte-bbox', default=None, metavar='x,y,w,h',
                   help='SAHTE dedektor icin kutu (or. 960,360,60,30)')
    p.add_argument('--renk', default=os.environ.get('YILDIZ_TARGET_COLOR',
                                                    DEFAULT_TARGET_COLOR),
                   help='HSV hedef rengi: purple | red')
    p.add_argument('--yolo-model', default='yolov8n.pt',
                   help='.pt (ultralytics) ya da .rpk (IMX500, sensorde kosar)')
    p.add_argument('--yolo-conf', type=float, default=0.35)
    p.add_argument('--yolo-imgsz', type=int, default=640)
    p.add_argument('--yolo-sinif', type=int, nargs='*', default=None,
                   help='yalniz bu sinif indisleri (bos = hepsi)')
    p.add_argument('--mavlink', default=None, metavar='BAGLANTI',
                   help="pymavlink adresi: /dev/ttyACM0 | udpin:127.0.0.1:14601 "
                        "| tcp:127.0.0.1:5760. Verilmezse tutum ve gimbal YOK "
                        "(roll=pitch=0 masa varsayimi).")
    p.add_argument('--no-gimbal', action='store_true',
                   help='tilt komutu gonderme (yalniz tutum okunur)')
    p.add_argument('--mount', type=float, default=0.0,
                   help='gimbal KAPALIYKEN kameranin govdeye montaj acisi '
                        '[derece, + yukari]')
    p.add_argument('--tilt', type=float, default=0.0,
                   help='gimbal varsayilan/yeniden-edinim elevasyonu [derece]')
    p.add_argument('--tilt-sabit', action='store_true',
                   help='tilt takibi KAPALI: --tilt degerinde sabit kal')
    p.add_argument('--tilt-alt', type=float, default=-35.0)
    p.add_argument('--tilt-ust', type=float, default=55.0)
    p.add_argument('--tilt-tau', type=float, default=0.4)
    p.add_argument('--tilt-slew', type=float, default=45.0)
    p.add_argument('--kamera-gecikme-ms', type=float, default=0.0,
                   help='kamera boru hatti gecikmesi; tutum bu kadar GERIYE '
                        'interpolasyonla tasinir (tools/gimbal_zaman_kalibre.py)')
    p.add_argument('--hedef-genislik-m', type=float, default=1.6,
                   help='hedefin gorunen genisligi [m]; bbox genisliginden '
                        'KABA menzil kestirimi icin (gudum menzili BURADAN '
                        'ALMAZ, telemetriden alir)')
    p.add_argument('--goster', action='store_true',
                   help='OpenCV penceresi (bassiz Pi\'de KAPALI birak)')
    p.add_argument('--kaydet', nargs='?', const='', default=None,
                   metavar='DOSYA', help='cizili kareleri mp4 kaydet')
    p.add_argument('--log', default=None, metavar='CSV',
                   help='kare basina teshis CSV\'si')
    p.add_argument('--fps', type=float, default=0.0,
                   help='dongu hiz siniri [Hz]; 0 = kaynak ne veriyorsa')
    p.add_argument('--sure', type=float, default=0.0,
                   help='saniye; 0 = Ctrl-C\'ye kadar')
    p.add_argument('--redis-host', default='127.0.0.1')
    p.add_argument('--redis-port', type=int, default=6379)
    a = p.parse_args(argv)

    a.sahte_bbox_coz = _sahte_bbox_coz(a.sahte_bbox)
    if a.kaydet == '':
        kok = Path(__file__).resolve().parent
        (kok / 'videos').mkdir(exist_ok=True)
        a.kaydet = str(kok / 'videos' /
                       f"kopru_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    if a.goster and not os.environ.get('DISPLAY'):
        print("UYARI: DISPLAY yok, --goster yoksayiliyor.", flush=True)
        a.goster = False
    # Sentetik kaynak sinirsiz hizda doner; CPU'yu bosuna yakmasin.
    if a.fps <= 0 and a.kaynak == 'dosya' and not a.dosya:
        a.fps = 30.0
    return a


def main(argv=None):
    a = arg_ayristir(argv)
    kopru = KameraKopru(a)

    def _kapat(signum, frame):
        print(f"\nsinyal {signum} alindi, temiz kapaniyor...", flush=True)
        kopru._dur = True
    signal.signal(signal.SIGINT, _kapat)
    signal.signal(signal.SIGTERM, _kapat)

    try:
        return kopru.calistir()
    except KeyboardInterrupt:
        return 0
    finally:
        kopru.kapat()


if __name__ == '__main__':
    raise SystemExit(main())
