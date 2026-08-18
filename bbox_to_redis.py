#!/usr/bin/env python3
"""
Drone kamerasi -> Redis 'tracker_bbox' renk-tespit koprusu (+ mod karar verici)
==============================================================================
Iskelet arkadastan gelen arkadas_scripts/drone_cam_bbox_to_redis.py'dir
(topic, karar verici penceresi, HUD). Uzerine bumblebee paketinden OLCUME
DAYALI dersler tasindi:

  1) HEDEF RENGI MOR (kirmizi DEGIL).
     bumblebee olcumu (3 kayit, 900 kare, 1.99e8 kromatik piksel; S>=70,V>=50):
       H 110-114 (gokyuzu kubbesi) : %54.4   H 45-49 (cim) : %26.1
       H  30- 34 (pist)            : %13.8
       H   0-  4 (KIRMIZI pencere) : % 1.9   <-- arka planla CAKISIYOR
       H 140-160 (MOR pencere)     : % 0.0012 <-- ayrim payi ~1700x
     Ucak yatinca ufuk bandi (BGR~127,127,255) kadraj genisliginde SAHTE
     hedef uretiyordu. Cozum: dunyayi fakirlestirmek yerine HEDEFIN rengini
     degistirmek -> models/gazebo-plane6 Gazebo/Purple'a boyandi.
     YILDIZ_TARGET_COLOR=red ile eski kirmizi pencereye donulur.

  2) SEKIL KAPISI (opsiyonel, varsayilan KAPALI).
     Gazebo ~2 km'den sonra yer duzleminde 1 px yuksekliginde, ucak renklerini
     tasiyan cizgi artefaktlari uretiyor; dilate sonrasi h~5 px kontur olup
     "en buyuk kontur" secimini calabiliyorlar.

  3) Video kaydi display'den BAGIMSIZ (headless'ta da calisir).

Yayin formati (JSON, guidance kodlari json.loads ile okur):
    [x, y, w, h, horizontal_coverage_%, validity, t_capture]
yildizlar25/ altindaki guidance kodlari data[:4] okur; fazlalik zararsizdir.
"""

import argparse
import csv
import json
import math
import os
import signal
import threading
import time
from collections import deque

import cv2
import numpy as np
import redis

# --- OPENCV THREAD SAYISI (bumblebee dersi, 2026-07 olcumu) ---
# OpenCV varsayilan 16 thread ile AYNI isi ~4x CPU'ya yapiyor (1080p olcum:
# 7.7 ms @%411 CPU vs tek thread 10.3 ms @%100 CPU). 30 Hz kamera icin tek
# thread zaten fazlasiyla yetiyor, o yuzden varsayilan 1.
# YILDIZ_CV_THREADS ile ezilir (0/negatif = OpenCV varsayilani; OpenCV'de
# setNumThreads(0) "tum cekirdekler" DEGIL "sirali" demek, o yuzden -1).
_CV_THREADS = int(os.environ.get('YILDIZ_CV_THREADS', '1'))
cv2.setNumThreads(_CV_THREADS if _CV_THREADS > 0 else -1)
# --- ROS ISTEGE BAGLI (2026-08-07) ------------------------------------
# NEDEN TRY/EXCEPT: bu dosyanin ROS DISI parcalari (TutumOkuyucu, HSV
# tespiti, sanal gimbal zinciri, karar verici) GERCEK DONANIMDA da gerekli,
# ama Raspberry Pi 5'te ROS YOK. Modul seviyesindeki sert import, dosyanin
# IMPORT EDILMESINI bile engelliyordu; donanim koprusu (donanim/kamera_
# kopru.py) o parcalari YENIDEN YAZMAK zorunda kalirdi -- yani ayni mantigin
# iki kopyasi, iki ayri gercek. Kopya yerine import edilebilir olsun diye.
#
# DAVRANIS DEGISMEZ: ROS varsa (sim) uc modul de eskisi gibi baglanir ve
# tum kod yolu birebir aynidir. ROS yoksa yalnizca ROS'a BAGIMLI yol
# (SuruRedisDetector.__init__ icindeki abone + main/rospy.spin) calismaz;
# ikisi de asagida anlasilir hata verir.
try:
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image
except ImportError as _ros_import_hatasi:      # ROS yok (Pi) -- ROS disi yollar calisir
    rospy = None
    CvBridge = None
    Image = None

    class CvBridgeError(Exception):
        """ROS yokken image_callback'teki 'except CvBridgeError' cozulebilsin."""

    ROS_HATASI = _ros_import_hatasi
else:
    ROS_HATASI = None


def ros_gerekli(nerede):
    """ROS'a bagimli yollarin girisinde cagrilir: sessiz AttributeError
    yerine ne yapilmasi gerektigini soyleyen bir hata."""
    if rospy is None:
        raise SystemExit(
            f"HATA: {nerede} ROS gerektirir ama ROS import edilemedi "
            f"({ROS_HATASI}).\n"
            "Gercek donanimda (ROS'suz) bunun yerine "
            "'python3 donanim/kamera_kopru.py' kullanin.")

from yildizlar_gimbal import AimTrim, SanalGimbal, analitik_aim, eklem_acisi
from tools.gz_gimbal import (TiltDurumOkuyucu, TiltKomutcu, TiltTakip,
                             model_adi_topikten)


class TutumOkuyucu(threading.Thread):
    """MAVLink ATTITUDE'u surekli okur ve ZAMAN DAMGALI tampona yazar.

    NEDEN THREAD: kare basina tek recv_match cagirmak UDP tamponunda birikme
    yapar ve BAYAT tutum okunur (ayni hata tools/suru_komut.py konum_al'da
    olculdu: 6 s'de 9 mesaj birikip 6 s eski veri geliyordu).

    NEDEN TAMPON + INTERPOLASYON (2026-08-02): yalniz "en son tutum"u tutmak
    yetmiyordu. Tutum 50 Hz, kameralar 30 Hz ve iki akis birbirinden bagimsiz
    geldigi icin aralarinda 0-30 ms kayma kaliyor; 30 derece/s govde hizinda
    bu ~1 derecelik de-rotasyon hatasi demek. Olculdu: stabilize dikey hatanin
    pitch ile korelasyonu 0.873'ten yalnizca 0.290'a inebiliyordu.
    Cozum: son N ornegi zaman damgasiyla sakla, karenin YAKALANDIGI ana
    dogrusal interpolasyonla tasi.

    GERCEK UCUSTA DA CALISIR - simulasyon saatine bagli DEGIL:
      * Her ornek GELDIGI anda time.monotonic() ile damgalanir. Ayni saat
        kareler icin de kullanilir; yani iki akis TEK ORTAK saatte bulusur.
        (Pi uzerinde kamera da MAVLink de ayni makinede damgalanir.)
      * Ayrica FCU'nun kendi saati (time_boot_ms) kaydedilir ve
        (t_yerel - t_fcu) farkinin MINIMUMU tutulur. Minimum, en dusuk
        gecikmeli ornege karsilik geldigi icin iki saat arasindaki gercek
        ofsetin en iyi tahminidir; link gecikmesini raporlamak ve gerekirse
        FCU saatine gecmek icin kullanilir.
      * Kameranin boru hatti gecikmesi (yakalama -> bize ulasma) sabit bir
        sayidir ve tools/gimbal_zaman_kalibre.py ile OLCULUR; burada
        gecikme_s olarak uygulanir.
    """

    def __init__(self, port, hz=50.0, tampon=512, gecikme_s=0.0, mav=None):
        """port: SAYI ise 'udpin:127.0.0.1:<port>' kurulur (sim davranisi,
        DEGISMEDI). METIN ise dogrudan pymavlink adresi olarak kullanilir
        ('/dev/ttyACM0', 'tcp:127.0.0.1:5760', 'udpin:0.0.0.0:14601' ...).
        NEDEN: gercek donanimda otopilot USB/seri ya da mavproxy bolucusu
        arkasindadir; sabit 'udpin:127.0.0.1:N' kalibi orada yok.

        mav: HAZIR bir mavutil baglantisi (varsayilan None = eskisi gibi
        kendi baglantisini kurar). NEDEN: gercek donanimda ayni adres IKI
        KEZ acilamaz -- 'udpin:' porta bind eder (ikinci acilis hata verir),
        seri port ikinci acilista bozuk cerceve verir. Tilt komutcusu
        (tools/mavlink_tilt.MavlinkTiltKomutcu) ile ayni baglantiyi
        PAYLASMAK icin. Sozlesme mavlink_tilt'in docstring'iyle ayni:
        komutcu.basla() ONCE cagrilir (tek recv'ini orada yapar), sonra bu
        thread tek recv tuketicisi olarak calisir."""
        super().__init__(daemon=True)
        self.port = port
        self.mav = mav
        self.hz = hz
        self.gecikme_s = float(gecikme_s)
        self.buf = deque(maxlen=tampon)      # (t_yerel, t_fcu_s, roll, pitch, yaw)
        self.lock = threading.Lock()
        self.roll = self.pitch = self.yaw = 0.0     # en son (geriye uyumluluk)
        self.sayac = 0
        self.hazir = False
        self.saat_ofseti = None              # min(t_yerel - t_fcu)
        self.son_bosluk_s = 0.0              # interpolasyonda kullanilan aralik
        self._log_f = self._log_w = None
        self._log_sayac = 0                  # periyodik flush sayaci

    def log_ac(self, yol):
        self._log_f = open(yol, 'w', newline='')
        self._log_w = csv.writer(self._log_f)
        self._log_w.writerow(['t_yerel', 't_fcu_s', 'roll_deg', 'pitch_deg', 'yaw_deg'])

    def log_kapat(self):
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
            self._log_w = None

    def run(self):
        from pymavlink import mavutil
        adres = (self.port if isinstance(self.port, str)
                 else f'udpin:127.0.0.1:{self.port}')
        try:
            if self.mav is not None:
                self._m = self.mav          # PAYLASILAN baglanti (bkz. __init__)
            else:
                self._m = mavutil.mavlink_connection(adres, source_system=253)
                self._m.wait_heartbeat(timeout=30)
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                int(1e6 / self.hz), 0, 0, 0, 0, 0)
            print(f"Sanal gimbal telemetrisi: {adres} baglandi "
                  f"({self.hz:.0f} Hz istendi)", flush=True)
        except Exception as exc:
            print(f"UYARI: tutum baglantisi kurulamadi ({exc}); "
                  f"sanal gimbal DEVRE DISI", flush=True)
            return
        while True:
            msg = self._m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if msg is None:
                continue
            t_yerel = time.monotonic()
            t_fcu = msg.time_boot_ms / 1000.0
            ofset = t_yerel - t_fcu
            if self.saat_ofseti is None or ofset < self.saat_ofseti:
                self.saat_ofseti = ofset
            with self.lock:
                self.buf.append((t_yerel, t_fcu, msg.roll, msg.pitch, msg.yaw))
            self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            self.sayac += 1
            self.hazir = True
            if self._log_w is not None:
                self._log_w.writerow([f"{t_yerel:.6f}", f"{t_fcu:.3f}",
                                      f"{math.degrees(msg.roll):.4f}",
                                      f"{math.degrees(msg.pitch):.4f}",
                                      f"{math.degrees(msg.yaw):.4f}"])
                # PERIYODIK FLUSH: cakilmada son tampon kaybolmasin
                # (gimbal CSV ile ayni gerekce, bkz. 2026-08-07 notu).
                self._log_sayac += 1
                if self._log_sayac % 20 == 0:
                    self._log_f.flush()

    @staticmethod
    def _aci_karistir(a, b, k):
        """En kisa yoldan aci interpolasyonu (roll +-180'de sarmali gecer)."""
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        return a + d * k

    def tutum_al(self, t_kare):
        """Karenin YAKALANDIGI ana (t_kare - gecikme) ait roll/pitch.

        Iki komsu ornek arasinda dogrusal; tampon disinda kalirsa en yakin
        ornege duser (ekstrapolasyon YAPILMAZ - gurultuyu buyutur).
        """
        t_hedef = t_kare - self.gecikme_s
        with self.lock:
            n = len(self.buf)
            if n == 0:
                return None
            if n == 1 or t_hedef <= self.buf[0][0]:
                self.son_bosluk_s = 0.0
                return self.buf[0][2], self.buf[0][3], t_hedef
            if t_hedef >= self.buf[-1][0]:
                self.son_bosluk_s = t_hedef - self.buf[-1][0]
                return self.buf[-1][2], self.buf[-1][3], t_hedef
            # ikili arama yerine sondan tarama: hedef genelde son birkac ornekte
            for i in range(n - 1, 0, -1):
                t0 = self.buf[i - 1][0]
                t1 = self.buf[i][0]
                if t0 <= t_hedef <= t1:
                    k = 0.0 if t1 == t0 else (t_hedef - t0) / (t1 - t0)
                    self.son_bosluk_s = t1 - t0
                    return (self._aci_karistir(self.buf[i - 1][2], self.buf[i][2], k),
                            self._aci_karistir(self.buf[i - 1][3], self.buf[i][3], k),
                            t_hedef)
            self.son_bosluk_s = 0.0
            return self.buf[-1][2], self.buf[-1][3], t_hedef


# --- "SU AN CALISAN KOD" SATIRI (bumblebee/bbox_to_redis.py:200 fikri) ---
# Videoyu sonradan izlerken "bu kayit hangi kodla alindi" sorusunun cevabi
# kadrajin USTUNDE yazili olsun diye. CPU MALIYETI ONEMLI oldugundan:
#   * tarama AYRI THREAD'de ve KODU_TARA_S'de bir yapilir,
#   * kare isleme yolu yalnizca hazir bir string okur (bir attribute erisimi),
#   * /proc/<pid>/cmdline okunur - pgrep/subprocess fork'u YOK.
# Olcum: overlay + video kaydinin toplam ek maliyeti zaten %5 idi; bu satir
# onun icinde kalir.
KOD_TARA_S = 5.0
KOD_DESENLERI = (
    ('simple_guided_follow', 'konumlu(allstar)'),
    ('gorev_baslat', 'gorev_baslat'),
    ('suru_komut', 'suru_komut'),
    ('kamera_kalibrasyon', 'kamera_kalib'),
    ('aim_olc', 'aim_olc'),
    ('senaryo.sh', 'senaryo'),
)


class CalisanKod(threading.Thread):
    """Calisan gudum/test script'lerini periyodik tespit eder."""

    def __init__(self, period=KOD_TARA_S):
        super().__init__(daemon=True)
        self.period = float(period)
        self.text = 'Kod: bbox_to_redis'
        self._dur = threading.Event()

    def _tara(self):
        bulunan = []
        try:
            pidler = [d for d in os.listdir('/proc') if d.isdigit()]
        except OSError:
            return
        for pid in pidler:
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().decode('utf-8', 'ignore')
            except OSError:
                continue
            for desen, ad in KOD_DESENLERI:
                if desen in cmd and ad not in bulunan:
                    bulunan.append(ad)
        self.text = 'Kod: bbox_to_redis' + (' + ' + ' + '.join(bulunan) if bulunan else '')

    def run(self):
        while not self._dur.is_set():
            self._tara()
            self._dur.wait(self.period)

    def stop(self):
        self._dur.set()


# --- HEDEF RENGI ---------------------------------------------------------
TARGET_COLOR_WINDOWS = {
    # Gazebo/Purple = RGB(1,0,1) -> OpenCV HSV H=150. Tek pencere yeter.
    'purple': ((140, 160),),
    # Kirmizi H=0'da sarmal yaptigi icin iki pencere gerekir (eski davranis).
    'red': ((0, 10), (170, 180)),
}
TARGET_COLOR_SV = {
    'purple': (120, 60),   # (S_min, V_min) - H.264 kroma gurultusunu eler
    'red': (70, 50),       # eski kirmizi esikleri AYNEN
}
DEFAULT_TARGET_COLOR = 'purple'

MIN_CONTOUR_AREA = 18      # arkadasin degeri; 5 fps'te uzak hedefi kacirmiyor


def build_color_ranges(color, s_min=None, v_min=None):
    """Renk adindan (lower, upper) HSV pencere listesi uretir."""
    color = (color or '').strip().lower() or DEFAULT_TARGET_COLOR
    if color not in TARGET_COLOR_WINDOWS:
        print(f"UYARI: bilinmeyen YILDIZ_TARGET_COLOR='{color}'; gecerli: "
              f"{', '.join(sorted(TARGET_COLOR_WINDOWS))}. "
              f"'{DEFAULT_TARGET_COLOR}' kullanilacak.")
        color = DEFAULT_TARGET_COLOR
    default_s, default_v = TARGET_COLOR_SV[color]
    s_min = default_s if s_min is None else s_min
    v_min = default_v if v_min is None else v_min
    ranges = [(np.array([lo, s_min, v_min]), np.array([hi, 255, 255]))
              for lo, hi in TARGET_COLOR_WINDOWS[color]]
    return color, ranges


def _opt_int(name):
    raw = os.environ.get(name, '').strip()
    if not raw:
        return None
    try:
        return max(0, min(255, int(float(raw))))
    except ValueError:
        print(f"UYARI: {name} gecersiz ({raw}), yoksayildi.")
        return None


# --- DONUK DIKDORTGEN (YILDIZ_MINRECT) ----------------------------------
# BAYRAK: varsayilan KAPALI. Kapaliyken tespit yolu ve tracker_bbox_stab
# yuku BIT-AYNI kalir (minAreaRect cagrilmaz, payload 8 elemanli kalir).
MINRECT_ACIK = (os.environ.get('YILDIZ_MINRECT', '0').strip().lower()
                in ('1', 'true', 'yes', 'on'))
# NEDEN: hedef sabit kanat; DONERKEN BANK yapar. Eksene-hizali bbox (AABB)
# hem kendi roll'umuzu hem hedefin bankini ayni w/h oranina karistirir ve
# ISARETSIZ'dir (saga mi sola mi doniyor ayirt EDILEMEZ; olculdu: aspect
# sinyali zamanin ancak %13'unde okunabiliyor, R^2=0.32). AYNI kontura
# cv2.minAreaRect uygulanirsa donuk dikdortgenin GERCEK w/h'si cikar
# (kamera roll'u kaynaginda yok olur) ve UZUN EKSENIN ISARETLI acisi
# olculur -> hedef bank -> omega = g*tan(bank)/v.
#
# ACI TANIMI (OpenCV konvansiyonuna GUVENMIYORUZ; surum 4.5.1'de [-90,0)
# araligi (0,90]'a degisti ve w/h siralamasi da onunla birlikte dondu --
# bu tuzaga dusmemek icin aci minAreaRect'in kendi 'angle' alanindan DEGIL,
# boxPoints() kose noktalarindan turetilir):
#   * rot_w_px  = dikdortgenin UZUN kenari [px]  (her zaman >= rot_h_px)
#   * rot_h_px  = KISA kenar [px]
#   * rot_aci_deg = UZUN EKSENIN goruntu yataya gore acisi, EKRANDA
#     saat-yonunun-TERSI pozitif (goruntu y ekseni asagi baktigi icin
#     atan2(-dy, dx) kullanilir). Bir EKSEN acisidir: yon (hangi ucun bas
#     oldugu) bilinmez, bu yuzden mod 180 tanimlidir ve [-90, +90)
#     araligina katlanir.
#
# SUREKLILIK: katlama yalnizca gorunur uzun eksen DIKEYE yaklasirken
# (|aci| -> 90) sarar. Kovalarken uzun eksen kanat cizgisidir ve
# gorunur aci ~ (hedef bank) -/+ (avci roll); ikisi birden ~90 dereceyi
# bulmadikca sarma OLMAZ. Tuketici yine de ardisik ornekler arasinda
# 180 derecelik sicramayi acmalidir (ornek: d = ((a2-a1+90) % 180) - 90).
#
# OFFLINE DOGRULAMA (2026-08-09, 3 elips kosusu: b5k2a / b5k3a / b5k3b;
# kayitli videolardan ayni HSV zinciri + hedefin BIN ATT roll'u gercek):
#   * ROLL AYRISMASI CALISIYOR: corr(rot_aci, avci_roll) = +0.77..+0.80,
#     yani gorunur eksen acisinin BASKIN terimi kendi roll'umuz ve
#     bank ~ katla(roll - rot_aci) ile temiz sekilde cikariliyor.
#   * AMA KALAN SINYAL ZAYIF: bank kestirimi ile GERCEK hedef bank'i
#     arasinda r = +0.33..+0.43 (R^2 0.11..0.18), RMS ~9-11 deg; kare
#     basina gurultu ~4.5 deg RMS (gercek bank kareler arasi 0.3 deg
#     degisiyor). omega = g*tan(bank)/v korelasyonu yalniz +0.22..+0.35.
#   * Menzil bandina gore ISARET bile donuyor (25-40 m'de r = -0.65).
#   Sebep: kosularda menzil 8-30 m ve goz acisi p50 18-55 deg -- yani saf
#   kuyruk takibi degil; bu kadar yakinda perspektif + govde/kuyruk
#   siluetinin katkisi kanat cizgisini boguyor. TAM GEOMETRIK ILERI MODEL
#   (gercek hedef tutumu + kamera tutumu -> beklenen ekran acisi) olculen
#   aciyi r = 0.47..0.82 ile aciklıyor: olcum kotu degil, "bank = roll -
#   aci" YAKLASIMI kuyruk-takibi disinda gecersiz.
#   HUKUM: alanlar LOG/ANALIZ icin acildi; MPC'ye baglanmaya HAZIR DEGIL.
#   Eksik olan: (1) iki yone donen hedef (missions/hedef_s.plan) --
#   elipste hedef hep saga bandigi icin isaret ayrimi HIC test edilemedi;
#   (2) goz acisini hesaba katan tersine cevirme ya da kapi (goz < ~30 deg
#   ve boy > ~60 px'te r 0.56'ya cikiyor).
def _donuk_dikdortgen(contour):
    """Kontura minAreaRect uygular; (uzun_px, kisa_px, eksen_aci_deg) doner.

    Aci [-90, +90) araliginda, EKRANDA CCW pozitif. Yukaridaki blokta
    belgelenen tanim. Hata durumunda (dejenere kontur) None doner.
    """
    try:
        kutu = cv2.boxPoints(cv2.minAreaRect(contour))
    except cv2.error:
        return None
    kenar = [(float(kutu[(i + 1) % 4][0]) - float(kutu[i][0]),
              float(kutu[(i + 1) % 4][1]) - float(kutu[i][1]))
             for i in range(2)]          # ardisik iki kenar zaten DIK
    boy = [math.hypot(dx, dy) for dx, dy in kenar]
    i_uzun = 0 if boy[0] >= boy[1] else 1
    dx, dy = kenar[i_uzun]
    if boy[i_uzun] <= 0.0:
        return None
    # y ASAGI bakiyor -> -dy ile ekran-CCW pozitife cevir
    aci = math.degrees(math.atan2(-dy, dx))
    aci = ((aci + 90.0) % 180.0) - 90.0          # eksen acisi, [-90, +90)
    return boy[i_uzun], boy[1 - i_uzun], aci


def hsv_tespit(cv_image, color_ranges, min_blob_h=0.0, min_blob_fill=0.0,
               min_alan=MIN_CONTOUR_AREA, donuk=False):
    """En buyuk renk konturunu bul; (x, y, w, h) ya da None dondur.

    MODUL SEVIYESINE CIKARILDI (2026-08-07). NEDEN: ayni tespit mantigini
    donanim/kamera_kopru.py (ROS'suz gercek donanim girisi) da kullaniyor.
    SuruRedisDetector'in metodu olarak kalsaydi, o sinifin __init__'i ROS
    abonesi kurdugu icin Pi'de ERISILEMEZDI ve kopyalanmak zorunda kalirdi
    -- ayni mantigin iki kopyasi, sessizce ayrisan iki gercek demektir.

    Govde AYNEN tasindi; sekil kapisi esikleri artik parametre ve
    varsayilanlari eski davranisla BIREBIR ayni (0 = kapali).

    donuk=True verilirse donus degeri ((x, y, w, h), rot) ciftidir; rot
    _donuk_dikdortgen()'in (uzun_px, kisa_px, aci_deg) uclusu (ya da None).
    VARSAYILAN False -> donus degeri ve hesap eski haliyle BIREBIR ayni
    (minAreaRect HIC cagrilmaz).
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower, upper = color_ranges[0]
    mask = cv2.inRange(hsv, lower, upper)
    for lower, upper in color_ranges[1:]:
        mask = mask + cv2.inRange(hsv, lower, upper)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0.0
    best_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= min_alan or area <= best_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        # Sekil kapisi: yer paraziti ince/uzun ve dusuk dolulukta olur.
        if min_blob_h > 0 and h < min_blob_h:
            continue
        if min_blob_fill > 0 and w * h > 0 and area / (w * h) < min_blob_fill:
            continue
        best, best_area = (x, y, w, h), area
        best_contour = contour
    if not donuk:
        return best
    # minAreaRect YALNIZ kazanan kontur icin (dongude degil) -- ek maliyet
    # kare basina tek cagri.
    return best, (None if best_contour is None
                  else _donuk_dikdortgen(best_contour))


class SuruRedisDetector:
    # --- KARAR VERICI (arkadasin decider.py mantigi) ---
    # Gecis kurali: "hedef ~1 s kadrajda kalinca konumlu -> goruntulu".
    # PENCERE 45 -> 25 KARE (2026-08-07 kullanici karari). NEDEN: pencere
    # KARE cinsindendir, SURE degil. Simde kamera 30 fps (45 kare = 1.5 s)
    # ama gercek donanimda YOLO ~20 Hz kosacak; ayni 45 kare orada 2.25 s
    # eder, yani kod degismeden devir yarim saniye GECIKIR. 25 kare iki
    # tarafi da makul bir bantta tutar: 30 fps -> 0.83 s, 20 Hz -> 1.25 s.
    # Gecerli kare orani %80'de KALIR (20/25), yani kararlilik olcutu ayni.
    # Env ile ayarlanir: YILDIZ_PENCERE_KARE / YILDIZ_PENCERE_ORAN.
    WINDOW_SIZE = max(3, int(float(os.environ.get('YILDIZ_PENCERE_KARE', '25'))))
    TRANSITION_THRESHOLD = max(          # konumlu -> goruntulu icin gecerli kare
        2, int(round(WINDOW_SIZE
                     * float(os.environ.get('YILDIZ_PENCERE_ORAN', '0.8')))))
    REVERT_THRESHOLD = 3             # goruntulu -> konumlu icin dusuk esik
    REVERT_DWELL_SECONDS = 2
    # Kapsama esikleri YUZDE cinsindendir (horizontal_coverage = w/w_img*100).
    # Eski 0.85/0.7 degerleri ORAN gibi yazilmisti; kapi hep acik kaliyordu ve
    # gecis hedef 2 km otedeyken bile tetikleniyordu (README "Bilinen sorun").
    # OLCUM (tutucu_elips 2026-08-03): bbox genisligi menzille GUVENILIR
    # olceklenmiyor -- 204 m'de (virajda, planform gorunumu) 31 px olculurken
    # standoff'ta (26 m, arkadan) ortanca 34 / max 64 px kaldi (kapsama max
    # %5.0). Yani kapsama tek basina menzil kapisi OLAMAZ; %2 "hedef anlamli
    # buyuklukte gorunuyor" suzgecidir, gercek menzil kapisi asagidaki
    # GECIS_MENZIL_M'dir (konumlunun devir_durumu'na yazdigi estimator
    # menzili; kural geregi 3D veriden yalniz menzil kullanilir). Kalma esigi
    # bilerek dusuk: geri devri TESPIT KAYBI (REVERT_THRESHOLD+dwell) yonetir.
    MIN_COVERAGE_TRANSITION = float(os.environ.get('YILDIZ_COV_GECIS', '2.0'))
    MIN_COVERAGE_HOLD = float(os.environ.get('YILDIZ_COV_KAL', '0.3'))
    # ALAN OLCUTU -- VARSAYILAN (2026-08-05 kullanici karari: "35 metreye koy").
    # Gecis esigi kadrajin (p% x p%) dikdortgeninin alanidir; 0 = KAPALI
    # (eski yatay-kapsama davranisina doner).
    # OLCULEN menzil karsiligi: p=2 -> 35 m | p=3 -> 24 m | p=4 -> 18 m
    #                           p=5 -> 14 m | p=6 -> 12 m | p=8 ->  9 m
    # (kalibrasyon: bbox_alan ~ 4.65e5/r^2, n=1945 gercek kosu karesi;
    #  menzil karsiligi karsilasma tipinden bagimsiz olculdu)
    # NEDEN ALAN: yatay kapsama TEK eksen olcer, hedefin en/boy orani
    # (ortanca 2.33) degistikce kayar; alan iki ekseni birden gorur.
    GECIS_ALAN_PCT = float(os.environ.get('YILDIZ_GECIS_ALAN_PCT', '2'))
    GECIS_MENZIL_M = float(os.environ.get('YILDIZ_GECIS_MENZIL', '60'))
    # --- BASIT GECIS KURALI (2026-08-07 kullanici karari, VARSAYILAN ACIK) ---
    # "Tespit art arda N karede geldiyse devret." Gercek ucusta yukaridaki
    # uc kapili kural (alan %2 + menzil 60 m + 45 karelik pencerede 36 gecerli)
    # devri COK geciktirebiliyor; sahada istenen davranis basit ve ongorulebilir
    # olsun. Basit modda ATLANAN kapilar: alan kapisi, menzil kapisi, 36/45
    # penceresi. ATLANMAYANLAR: manuel_durdur, goruntulu_birak (ISKA) ve
    # OLU-ADAM kapisi -- yani yine yalnizca CALISAN bir goruntulu kontrolcuye
    # devredilir. GERI DONUS kurallari (REVERT_THRESHOLD + dwell, olu-adam,
    # goruntulu_birak) HIC DEGISMEZ; basit kural yalnizca konumlu -> goruntulu
    # yonunu etkiler.
    # VARSAYILAN 0 (2026-08-07 aksami, kullanici karari): sahaya once DUN
    # AKSAM VURAN surumun devir kurali gidiyor; basit kural kodda hazir ama
    # KAPALI, once simde dogrulanacak. Acmak icin YILDIZ_GECIS_BASIT=1.
    # ACARKEN BIRLIKTE ACILMALI (olculdu, yoksa ping-pong):
    #   YILDIZ_DEVIR_SOGUMA_S=3  ve  mpc_gudum MpcAyar.iska_mutlak_m=300
    # Sebep: basit kural menzil kapisini kaldirdigi icin devir 150+ m'de
    # gelebiliyor; 120 m mutlak ISKA siniri devri ~1 s'de iptal ediyor,
    # bbox 0.17 s'de yeniden devrediyor -> cevrim ~1.2 s, dakikada ~50 devir.
    GECIS_BASIT = (os.environ.get('YILDIZ_GECIS_BASIT', '0').strip().lower()
                   in ('1', 'true', 'yes', 'on'))
    GECIS_KARE = max(1, int(float(os.environ.get('YILDIZ_GECIS_KARE', '5'))))
    # DONANIM/LOS KURALI: art arda N kare YALNIZ tespitli degil, ayni
    # zamanda alan kapisindan gecmis (yeterince buyuk bbox) olmali. Tam
    # merkez, hedef telemetrisi ve uzun 20/25 pencere aranmaz. 0=kapali;
    # acikken BASIT kuraldan once gelir.
    GECIS_BUYUK_KARE = max(
        0, int(float(os.environ.get('YILDIZ_GECIS_BUYUK_KARE', '0'))))
    # DEVIR SOGUMASI: goruntulu -> konumlu donusunden sonra yeniden devir
    # icin asgari bekleme. Basit kuralda hedef gorunur kaldikca sayac
    # GECIS_KARE'yi 30 Hz'de ~0.17 s'de doldurur; ISKA ile yetkiyi birakan
    # MPC'ye aninda geri devretmek ping-pong uretir (olculdu: cevrim
    # ~1.2 s, dakikada ~50 devir). VARSAYILAN 0 = dun aksamki davranis;
    # basit kural acilirken YILDIZ_DEVIR_SOGUMA_S=3 ile birlikte acilmali.
    DEVIR_SOGUMA_S = float(os.environ.get('YILDIZ_DEVIR_SOGUMA_S', '0'))
    DEVIR_BAYAT_S = 3.0              # devir_durumu bundan eskiyse menzil yok say
    # OLU-ADAM: goruntulu kontrolcunun kalp atisi bundan eskiyse "yok" sayilir.
    # goruntulu_temel.HAYATTA_TTL_S (2 s) ile uyumlu; TTL zaten anahtari
    # dusurur, bu esik yalnizca ikinci emniyet.
    HAYATTA_BAYAT_S = 2.5

    def __init__(self, topic, display=True, record_path=None, record_fps=0.0,
                 gimbal=None, tutum=None, aim_trim=None, gimbal_log=None,
                 tilt_okuyucu=None, tilt_komutcu=None, tilt_takip=None):
        self.gimbal = gimbal
        self.tutum = tutum
        self.aim_trim = aim_trim
        # FAZ C: tilt_takip verilirse tilt hedefi her karede hedefin olculen
        # yukselisine surulur (tespit kaybinda standoff acisina doner)
        self.tilt_takip = tilt_takip
        self._takip_son_t = None
        # FIZIKSEL GIMBAL (gimbal dali): tilt_komutcu/okuyucu verilirse kamera
        # stabilize fiziksel gimbaldedir; sabit mount yerine CANLI eklem acisi
        # (eklem_acisi(eps, pitch, roll)) zincire girer.
        self.tilt_okuyucu = tilt_okuyucu
        self.tilt_komutcu = tilt_komutcu
        self._son_eklem_deg = None
        # YILDIZ_MINRECT: son karenin donuk dikdortgeni (uzun, kisa, aci) ya
        # da None. Bayrak kapaliyken HIC yazilmaz, hep None kalir.
        self._son_rot = None
        self._son_kare_t = None
        self._log_f = self._log_w = None
        self._log_sayac = 0          # periyodik flush sayaci (20 satirda bir)
        if gimbal_log:
            self._log_f = open(gimbal_log, 'w', newline='')
            self._log_w = csv.writer(self._log_f)
            self._log_w.writerow(['t_kare', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h',
                                  'roll_deg', 'pitch_deg', 'menzil_m',
                                  'ham_ex_deg', 'ham_ey_deg',
                                  'stab_ex_deg', 'stab_ey_deg',
                                  'aim_deg', 'aim_etkin_deg',
                                  'ros_stamp', 'interp_bosluk_ms', 'gecikme_ms',
                                  'tilt_cmd_deg', 'tilt_status_deg',
                                  'tilt_yas_ms', 'eklem_deg'])
            print(f"Gimbal logu: {gimbal_log}", flush=True)
        self.topic = topic
        self.display = display
        self.record_path = record_path
        self.record_fps = record_fps
        self.draw = display or (record_path is not None)

        print("Redis sunucusuna baglaniliyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.ping()
            self.r.set('komut_yetkisi', 'konumlu')
            print("Redis baglantisi basarili. Yayin kanali: 'tracker_bbox'")
        except Exception as exc:
            raise SystemExit(f"Redis baglanti hatasi: {exc}")

        self.target_color, self.color_ranges = build_color_ranges(
            os.environ.get('YILDIZ_TARGET_COLOR', DEFAULT_TARGET_COLOR),
            _opt_int('YILDIZ_HSV_SMIN'), _opt_int('YILDIZ_HSV_VMIN'))
        pencere = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                             for lo, hi in self.color_ranges)
        print(f"Hedef rengi: {self.target_color.upper()}  ->  {pencere}")

        # Sekil kapisi: verilmezse davranis birebir eskisi gibi.
        self.min_blob_h = float(os.environ.get('YILDIZ_MIN_BLOB_H', '0') or 0)
        self.min_blob_fill = float(os.environ.get('YILDIZ_MIN_BLOB_FILL', '0') or 0)
        if self.min_blob_h > 0 or self.min_blob_fill > 0:
            print(f"Sekil kapisi acik: min_h={self.min_blob_h:.0f} px, "
                  f"min_doluluk={self.min_blob_fill:.2f}")

        # --- karar verici ic durumu ---
        self.current_mode = 'konumlu'
        self.decision_window = deque(maxlen=self.WINDOW_SIZE)
        self.revert_pending_since = None
        # BASIT KURAL sayaci: art arda GECERLI TESPIT karesi (alan/menzil
        # kapisindan gecmesi gerekmez); gecersiz tek kare sifirlar.
        self.ardisik_gecerli = 0
        self.ardisik_buyuk = 0
        self._gecis_sebep = ''       # son mod degisikligini tetikleyen kural
        self._son_konumluya_donus = 0.0  # monotonic; 0 = acilistan beri donus yok
        if self.GECIS_BUYUK_KARE:
            print(f"Gecis kurali: BUYUK_KARE -- art arda "
                  f"{self.GECIS_BUYUK_KARE} tespit + alan "
                  f"%{self.GECIS_ALAN_PCT:g} (merkez/menzil aranmaz)")
        elif self.GECIS_BASIT:
            print(f"Gecis kurali: BASIT -- art arda {self.GECIS_KARE} gecerli "
                  f"tespit karesi (alan/menzil/pencere kapilari ATLANIR). "
                  f"Eski kural icin YILDIZ_GECIS_BASIT=0")
        else:
            print(f"Gecis kurali: ESKI -- {self.TRANSITION_THRESHOLD}/"
                  f"{self.WINDOW_SIZE} kare + alan %{self.GECIS_ALAN_PCT:g} + "
                  f"menzil <= {self.GECIS_MENZIL_M:.0f} m")

        # --- olcum ---
        self.frame_count = 0
        self.detect_count = 0
        self.first_frame_time = None
        # Kadraj boyutu ilk kareyle guncellenir; alan esigi bunu kullanir.
        self._kadraj_w, self._kadraj_h = 1280.0, 720.0
        self.last_report_time = time.time()
        self.last_report_frames = 0

        self.video_writer = None
        self._kayit_tampon = []
        # Kod satiri yalnizca CIZIM yapiliyorsa anlamli; thread de
        # yalniz o zaman baslar (headless yayin yolu etkilenmez).
        self.kod = None
        self.son_stab = None

        if self.draw:
            self.kod = CalisanKod()
            self.kod.start()

        ros_gerekli('bbox_to_redis.SuruRedisDetector (ROS Image abonesi)')
        rospy.init_node('yildizlar_bbox', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.image_callback,
                                          queue_size=1)
        print(f"ROS dugumu basladi, '{topic}' bekleniyor...")

    # ---------------- karar verici ----------------

    def _devir_menzil(self):
        """Konumlunun 'devir_durumu'ndaki estimator menzili [m] ya da None.

        t_mono her iki surecte de time.monotonic()'tir; Linux'ta
        CLOCK_MONOTONIC sistem capinda oldugu icin dogrudan karsilastirilir.
        """
        try:
            ham = self.r.get('devir_durumu')
            if not ham:
                return None
            devir = json.loads(ham)
            if time.monotonic() - float(devir['t_mono']) > self.DEVIR_BAYAT_S:
                return None
            return float(devir['range_m'])
        except Exception:
            return None

    def _goruntulu_hayatta(self):
        """Goruntulu kontrolcu son HAYATTA_BAYAT_S icinde yasam belirtisi
        gosterdi mi? Anahtar TTL'li yazildigi icin surec olurse kendiliginden
        kaybolur; ayrica t_mono ile ikinci bir tazelik kontrolu yapilir
        (TTL'in yuvarlama payina guvenmemek icin).

        Redis erisilemiyorsa True doner: yetki mekanizmasinin kendisi zaten
        Redis'e dayali, Redis yoksa bu kapi karar veremez ve ESKI DAVRANIS
        korunur (kapi yanlislikla surekli 'konumlu'ya kilitlemesin).
        """
        try:
            ham = self.r.get('goruntulu_hayatta')
            if not ham:
                return False
            d = json.loads(ham)
            return (time.monotonic() - float(d['t_mono'])) <= self.HAYATTA_BAYAT_S
        except Exception:
            return True

    def _evaluate_frame(self, valid_detection, coverage, alan_px2=None):
        """Bu kare 'gecis icin sayilir mi?'

        IKI OLCUT (YILDIZ_GECIS_ALAN_PCT ile secilir):
          * VARSAYILAN -- yatay kapsama: w / kadraj_genisligi * 100 >= esik.
          * ALAN olcutu (YILDIZ_GECIS_ALAN_PCT > 0) -- bbox ALANI, kadrajin
            (p% x p%) buyuklugundeki bir dikdortgeninden buyuk mu.
            p=5 icin esik 0.05*1280 x 0.05*720 = 2304 px^2.

        NEDEN ALAN SECENEGI: yatay kapsama tek eksen olcer ve hedefin en/boy
        orani aspect ile degisir (olculdu: w/h ortancasi 2.33). Alan iki
        ekseni birden gorur. OLCULEN kalibrasyon (n=1945, gercek kosular):
            bbox_alan ~ 4.65e5 / r^2   [px^2, r metre]
            bbox_w    ~ 1015  / r      [px]
        yani  %2 yatay kapsama  -> ~40 m ;  %5x%5 alan -> ~14 m.
        Alan esiginin menzil karsiligi karsilasma tipinden BAGIMSIZ cikti
        (kuyruk 14.3 / capraz 13.8 / kafa-kafaya 14.1 m).
        """
        if not valid_detection:
            return False
        if self.GECIS_ALAN_PCT > 0.0 and alan_px2 is not None:
            if self.current_mode == 'konumlu':
                return alan_px2 >= self._gecis_alan_esigi()
            # gorunutuluede KALMA olcutu kapsamada kalir (dusuk esik, histerezis)
            return coverage >= self.MIN_COVERAGE_HOLD
        threshold = (self.MIN_COVERAGE_TRANSITION if self.current_mode == 'konumlu'
                     else self.MIN_COVERAGE_HOLD)
        return coverage >= threshold

    def _gecis_alan_esigi(self):
        """(p% x p%) dikdortgeninin alani [px^2]. Kadraj boyutu ilk kareden."""
        p = self.GECIS_ALAN_PCT / 100.0
        return (p * self._kadraj_w) * (p * self._kadraj_h)

    def _make_decision(self, valid_count):
        now = time.time()
        try:
            val = self.r.get('manuel_durdur')
            manuel = bool(val) and val.decode('utf-8') == '1'
        except Exception:
            manuel = False

        if manuel:
            self.revert_pending_since = None
            if self.current_mode != 'konumlu':
                self._gecis_sebep = 'manuel_durdur'
                return 'konumlu', True
            return 'konumlu', False

        # ISKA: goruntulu tarafindaki kontrolcu (mpc_gudum) yetkiyi kendi
        # istegiyle birakti (bkz. guidance_allstar/goruntulu_temel.py, 'cmd =
        # self.k.komut(o)' alti). Tazelik penceresi 2 s: daha eski bir
        # anahtar, bu karar dongusu onu gormeden yeniden 'goruntulu'ya
        # gecilmis olabilecegi icin gormezden gelinir.
        try:
            ham = self.r.get('goruntulu_birak')
            if ham:
                d = json.loads(ham)
                if (self.current_mode == 'goruntulu'
                        and time.monotonic() - float(d['t_mono']) < 2.0):
                    self.r.delete('goruntulu_birak')
                    self._gecis_sebep = 'goruntulu_birak (ISKA)'
                    return 'konumlu', True
        except Exception:
            pass

        # OLU-ADAM KAPISI (2026-08-05): goruntulu kontrolcu CALISMIYORSA
        # yetki ona devredilmez; devredilmisse geri alinir. Kontrolcu
        # 'goruntulu_hayatta' anahtarini TTL ile tazeler (goruntulu_temel.
        # _hayatta_bildir). Surec hic baslatilmamissa ya da _baglan()'da
        # asili kalmissa anahtar YOKTUR.
        # ARIZA KAYDI: bu kapi olmadan, kontrolcu calismazken yetki
        # devredildi ve araci KIMSE komutlamadi -- 5.9 s'de menzil 40 -> 124 m
        # acildi, yaw cirpintisi 6.6x artti. Disaridan belirti "MPC titriyor,
        # hedefi takip etmiyor"du; oysa MPC hic kosmuyordu.
        if not self._goruntulu_hayatta():
            if self.current_mode == 'goruntulu':
                print("[KARAR] goruntulu kontrolcu YANIT VERMIYOR "
                      "('goruntulu_hayatta' bayat/yok) -> konumluya donuluyor",
                      flush=True)
                self._gecis_sebep = 'olu-adam (goruntulu_hayatta bayat)'
                return 'konumlu', True
            # Kontrolcu yokken gorulen kareleri gelecekteki bir angajmana
            # kredi sayma. Aksi halde surec sonradan acildiginda sayac 200+
            # olabilir ve ilk YENI kareyi bile beklemeden devir olur.
            self.ardisik_gecerli = 0
            self.ardisik_buyuk = 0
            if time.time() - getattr(self, '_son_hayatta_log', 0) > 10.0:
                self._son_hayatta_log = time.time()
                print("[KARAR] goruntulu kontrolcu YOK "
                      "('goruntulu_hayatta' anahtari bayat/yok); gecis "
                      "engellendi. Goruntulu gudumu baslatmayi unuttun mu?",
                      flush=True)
            return 'konumlu', False

        # --- BASIT KURAL: konumlu -> goruntulu ---
        # Yalnizca ILERI yonu kisa devre eder. Pencerenin dolmasini beklemez
        # (45 kare @30 Hz = 1.5 s bekleme), alan/menzil kapilarina bakmaz.
        # Geri donus yolu asagida DEGISMEDEN kalir: basit modda da goruntulu
        # iken pencere + REVERT_THRESHOLD + dwell isler (pencere devirde
        # temizlendigi icin dolmasi 1.5 s surer; bu sure boyunca geri donus
        # yalnizca olu-adam/goruntulu_birak/manuel ile olur -- ESKI DAVRANIS).
        if ((self.GECIS_BUYUK_KARE or self.GECIS_BASIT)
                and self.current_mode == 'konumlu'):
            self.revert_pending_since = None
            beklemede = time.monotonic() - getattr(
                self, '_son_konumluya_donus', 0.0)
            if beklemede < self.DEVIR_SOGUMA_S:
                if time.time() - getattr(self, '_son_soguma_log', 0) > 5.0:
                    self._son_soguma_log = time.time()
                    print(f"[KARAR] devir sogumasi: {beklemede:.1f}/"
                          f"{self.DEVIR_SOGUMA_S:.0f} s, gecis bekliyor",
                          flush=True)
                return 'konumlu', False
            sayac = (self.ardisik_buyuk if self.GECIS_BUYUK_KARE
                     else self.ardisik_gecerli)
            gereken = (self.GECIS_BUYUK_KARE if self.GECIS_BUYUK_KARE
                       else self.GECIS_KARE)
            if sayac >= gereken:
                self._gecis_sebep = (
                    f"buyuk_kare({sayac} ardisik, alan "
                    f"%{self.GECIS_ALAN_PCT:g})" if self.GECIS_BUYUK_KARE
                    else f"basit({sayac} ardisik kare)")
                return 'goruntulu', True
            return 'konumlu', False

        if len(self.decision_window) < self.WINDOW_SIZE:
            return self.current_mode, False

        if self.current_mode == 'konumlu':
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                # MENZIL KAPISI: kapsama aspect'e bagli oldugundan (virajda
                # 200+ m'de bile %2.4 olculdu) gecis icin konumlunun estimator
                # menzili de yakin olmali. devir_durumu yoksa/bayatsa (konumlu
                # kosmadan tek basina test) kapi atlanir -- eski davranis.
                menzil = self._devir_menzil()
                if menzil is not None and menzil > self.GECIS_MENZIL_M:
                    if time.time() - getattr(self, '_son_menzil_log', 0) > 5.0:
                        self._son_menzil_log = time.time()
                        print(f"[KARAR] pencere dolu ({valid_count}/"
                              f"{self.WINDOW_SIZE}) ama menzil {menzil:.0f} m > "
                              f"{self.GECIS_MENZIL_M:.0f} m, gecis bekliyor",
                              flush=True)
                    return 'konumlu', False
                self._gecis_sebep = (
                    f"eski({valid_count}/{self.WINDOW_SIZE} kare, alan "
                    f"%{self.GECIS_ALAN_PCT:g}, menzil kapisi "
                    f"{self.GECIS_MENZIL_M:.0f} m)")
                return 'goruntulu', True
            return 'konumlu', False

        # goruntulu -> konumlu: dusuk esik + dwell
        if valid_count <= self.REVERT_THRESHOLD:
            if self.revert_pending_since is None:
                self.revert_pending_since = now
                print(f"[KARAR] Esik alti ({valid_count}/{self.WINDOW_SIZE}), "
                      f"dwell basladi ({self.REVERT_DWELL_SECONDS}s)", flush=True)
            if now - self.revert_pending_since >= self.REVERT_DWELL_SECONDS:
                self.revert_pending_since = None
                self._gecis_sebep = (f"tespit kaybi ({valid_count}/"
                                     f"{self.WINDOW_SIZE} + dwell)")
                return 'konumlu', True
            return 'goruntulu', False

        if self.revert_pending_since is not None:
            print(f"[KARAR] Hedef geri gorundu ({valid_count}/{self.WINDOW_SIZE}), "
                  f"dwell iptal", flush=True)
            self.revert_pending_since = None
        return 'goruntulu', False

    def _update_decision(self, valid_detection, coverage, alan_px2=None):
        gecis_gecerli = self._evaluate_frame(valid_detection, coverage,
                                              alan_px2)
        self.decision_window.append(gecis_gecerli)
        valid_count = sum(self.decision_window)
        # BASIT KURAL sayaci: HAM tespit gecerliligi sayilir (sekil kapisi
        # dahil, alan/menzil kapisi HARIC). Tek gecersiz kare sifirlar.
        if valid_detection:
            self.ardisik_gecerli += 1
        else:
            self.ardisik_gecerli = 0
        if self.current_mode == 'konumlu' and gecis_gecerli:
            self.ardisik_buyuk += 1
        else:
            self.ardisik_buyuk = 0
        yeni_mod, degisti = self._make_decision(valid_count)
        if degisti:
            eski = self.current_mode
            self.current_mode = yeni_mod
            self.decision_window.clear()
            self.revert_pending_since = None
            self.ardisik_gecerli = 0
            self.ardisik_buyuk = 0
            if yeni_mod == 'konumlu':
                self._son_konumluya_donus = time.monotonic()
            try:
                self.r.set('komut_yetkisi', self.current_mode)
                # Hangi kural tetikledi? goruntulu_temel bunu 'devir_alindi'
                # olayinin detayina yazar (LOG_SOZLUGU.md).
                self.r.set('gecis_sebep', self._gecis_sebep or 'bilinmiyor')
            except Exception:
                pass
            print(f"[KARAR] >>> MOD DEGISTI: {eski} -> {yeni_mod} "
                  f"[{self._gecis_sebep or 'bilinmiyor'}]", flush=True)
        return self.current_mode, valid_count

    # ---------------- tespit ----------------

    def _detect(self, cv_image):
        """En buyuk renk konturunu bul; (x, y, w, h) ya da None dondur.

        Govde modul seviyesindeki hsv_tespit()'e tasindi (donanim koprusu de
        AYNI fonksiyonu cagirsin diye); davranis birebir ayni.

        YILDIZ_MINRECT=1 iken ek olarak ayni konturun donuk dikdortgeni
        self._son_rot'a yazilir (yayin tarafinda okunur)."""
        if not MINRECT_ACIK:
            return hsv_tespit(cv_image, self.color_ranges,
                              self.min_blob_h, self.min_blob_fill)
        box, rot = hsv_tespit(cv_image, self.color_ranges,
                              self.min_blob_h, self.min_blob_fill, donuk=True)
        self._son_rot = rot
        return box

    def image_callback(self, data):
        # TEK ORTAK SAAT: her sey time.monotonic() ile damgalanir - kareler de,
        # tutum ornekleri de. Simulasyon saatine (header.stamp) BAGLI DEGIL,
        # bu yuzden gercek uckta da ayni kod calisir. header.stamp yalniz
        # bilgi amacli loglanir.
        t_kare = time.monotonic()
        t_capture = data.header.stamp.to_sec() or time.time()
        t_start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr(exc)
            return

        h_img, w_img = cv_image.shape[:2]
        self.frame_count += 1
        if self.first_frame_time is None:
            self.first_frame_time = t_start
            print(f"ILK KARE ALINDI: {w_img}x{h_img}", flush=True)

        box = self._detect(cv_image)
        valid_detection = box is not None
        horizontal_coverage = 0.0

        if valid_detection:
            x, y, w, h = box
            horizontal_coverage = (w / w_img) * 100.0
            self.detect_count += 1
            bbox = [int(x), int(y), int(w), int(h),
                    round(horizontal_coverage, 3), 1, round(t_capture, 4)]
            self.r.publish('tracker_bbox', json.dumps(bbox))

            # --- SANAL GIMBAL ---
            # Ham bbox kanali AYNEN korunur (yildizlar25/ altindaki gudum
            # kodlari onu okuyor); stabilize edilmis degerler AYRI kanala
            # basilir ki iki yol yan yana karsilastirilabilsin.
            tutum_ornek = (self.tutum.tutum_al(t_kare)
                           if (self.tutum is not None and self.tutum.hazir) else None)
            if self.gimbal is not None and tutum_ornek is not None:
                roll_i, pitch_i, t_hedef = tutum_ornek
                mx, my = x + w / 2.0, y + h / 2.0
                # Menzil bbox genisliginden kestirilir; YALNIZ aim
                # sonumlemesi icin (gudum menzili telemetriden alir).
                menzil = self.gimbal.menzil_tahmin(w)
                # FIZIKSEL GIMBAL: sabit mount yerine canli eklem acisi.
                # eklem None ise (tilt modu kapali) eski yol aynen calisir.
                eklem, tilt_eps, tilt_yas = self._eklem_hesapla(roll_i, pitch_i)
                self._son_eklem_deg = eklem
                sx, sy = self.gimbal.stabilize(mx, my, roll_i, pitch_i, menzil,
                                               eklem_deg=eklem)
                ex, ey = self.gimbal.aci_hatasi(mx, my, roll_i, pitch_i, menzil,
                                                eklem_deg=eklem)
                # HAM hata: yazilim de-rotasyonu HIC olmasaydi gudumun gorecegi
                # buyukluk. DIKKAT (gimbal dali): fiziksel gimbal varken "ham"
                # zaten stabilize kameranin ciktisidir -- gövde pitch'i ham_ey
                # icinde ARTIK GORUNMEZ (gimbal_kanit.py olcutu buna gore
                # guncellenmeli; eski r_sy < 0.5*r_hy olcutu bos kaldi).
                ham_ex = math.degrees(math.atan((mx - self.gimbal.cx) / self.gimbal.fx))
                ham_ey = math.degrees(math.atan((my - self.gimbal.cy) / self.gimbal.fy))
                self.son_stab = (sx, sy, ex, ey)

                # --- AIM TRIM (dusuk bantgenislikli, kelepceli) ---
                simdi = time.time()
                dt = 0.0 if self._son_kare_t is None else (simdi - self._son_kare_t)
                self._son_kare_t = simdi
                if self.aim_trim is not None and 0 < dt < 1.0:
                    yeni_aim = self.aim_trim.guncelle(ey, menzil, dt)
                    if yeni_aim != self.gimbal.aim_pitch_deg:
                        self.gimbal.aim_pitch_deg = yeni_aim

                if self._log_w is not None:
                    self._log_w.writerow([
                        f"{t_kare:.6f}", f"{mx:.2f}", f"{my:.2f}", w, h,
                        f"{math.degrees(roll_i):.3f}",
                        f"{math.degrees(pitch_i):.3f}",
                        '' if menzil is None else f"{menzil:.1f}",
                        f"{ham_ex:.4f}", f"{ham_ey:.4f}",
                        f"{ex:.4f}", f"{ey:.4f}",
                        f"{self.gimbal.aim_pitch_deg:.3f}",
                        f"{self.gimbal.aim_etkin_deg(menzil):.3f}",
                        f"{t_capture:.4f}", f"{self.tutum.son_bosluk_s*1000:.2f}",
                        f"{self.tutum.gecikme_s*1000:.1f}",
                        ('' if self.tilt_komutcu is None
                         or self.tilt_komutcu.hedef_deg is None
                         else f"{self.tilt_komutcu.hedef_deg:.3f}"),
                        '' if tilt_eps is None else f"{tilt_eps:.4f}",
                        '' if tilt_yas is None else f"{tilt_yas*1000:.0f}",
                        '' if eklem is None else f"{eklem:.4f}"])
                    # PERIYODIK FLUSH (2026-08-07): bu CSV eskiden yalniz
                    # kapanista bosaltiliyordu; cakilmada/SIGKILL'de son
                    # blok tamponu (~8 KB, 30 Hz'de ~2 s) diske hic
                    # yazilmiyordu -- yani tilt komut-vs-gerceklesen
                    # kaydinin kaza anina en yakin kismi kayipti. mpc_tani
                    # ve goruntulu CSV ile ayni desen: 20 satirda bir.
                    self._log_sayac += 1
                    if self._log_sayac % 20 == 0:
                        self._log_f.flush()
                # --- FAZ C: TILT TAKIBI ---
                # ey ufka gore olculur (canli-eklem zinciri) -> hedefin dunya
                # yukselisi = -ey, TILT'TEN BAGIMSIZ. Yani bu bir kapali
                # geri-besleme degil, olculen buyuklugun suzgecli takibi.
                if self.tilt_takip is not None and self.tilt_komutcu is not None:
                    simdi_t = time.monotonic()
                    dt_t = (1.0 / 30 if self._takip_son_t is None
                            else simdi_t - self._takip_son_t)
                    self._takip_son_t = simdi_t
                    self.tilt_komutcu.hedef(
                        self.tilt_takip.guncelle(-ey, dt_t, simdi=simdi_t))

                # 8. eleman (FAZ C): o karede zincirin kullandigi kamera
                # elevasyonu -- gudum ey_ref'i bundan kurar (Olcum.tilt_deg).
                _stab_yuk = [round(sx, 2), round(sy, 2), int(w), int(h),
                             round(ex, 4), round(ey, 4), round(t_capture, 4),
                             None if tilt_eps is None else round(tilt_eps, 3)]
                # 9-11. elemanlar (YILDIZ_MINRECT): donuk dikdortgen --
                # rot_w_px (uzun kenar), rot_h_px (kisa kenar), rot_aci_deg
                # (uzun eksenin ekran-CCW acisi, [-90,+90), bkz.
                # _donuk_dikdortgen). BAYRAK KAPALIYKEN EKLENMEZ: payload
                # 8 elemanli, eski tuketicilerle bit-ayni.
                if MINRECT_ACIK:
                    _rot = self._son_rot
                    _stab_yuk += ([None, None, None] if _rot is None else
                                  [round(_rot[0], 2), round(_rot[1], 2),
                                   round(_rot[2], 3)])
                self.r.publish('tracker_bbox_stab', json.dumps(_stab_yuk))
            self.r.set('timing_bbox_to_redis', f"{(time.time() - t_start) * 1000:.2f}")

        # FAZ C kayip yolu: tespit yoksa takip yasasi kayip sayacini isletsin
        # (kayip_tut_s tutar, sonra standoff acisina yavas donus)
        if ((not valid_detection) and self.tilt_takip is not None
                and self.tilt_komutcu is not None):
            simdi_t = time.monotonic()
            dt_t = (1.0 / 30 if self._takip_son_t is None
                    else simdi_t - self._takip_son_t)
            self._takip_son_t = simdi_t
            self.tilt_komutcu.hedef(
                self.tilt_takip.guncelle(None, dt_t, simdi=simdi_t))

        self._kadraj_w, self._kadraj_h = float(w_img), float(h_img)
        alan_px2 = None
        if valid_detection:
            _bx, _by, _bw, _bh = box
            alan_px2 = float(_bw) * float(_bh)
        komut_yetkisi, valid_count = self._update_decision(valid_detection,
                                                           horizontal_coverage,
                                                           alan_px2)
        self._report(valid_detection, box, horizontal_coverage)

        if self.draw:
            self._draw(cv_image, box, komut_yetkisi, valid_count,
                       horizontal_coverage, w_img, h_img)

    def _eklem_hesapla(self, roll_i, pitch_i):
        """Gazebo kamera elevasyonundan govdeye gore eklem acisini dondurur.

        ``TiltDurumOkuyucu`` burada Gazebo ``gimbal_tilt_status`` topic'ini
        okur. ``GimbalSmall2dPlugin`` stabilize modunda bu topic'e eklem
        acisini degil, kameranin DUNYA elevasyonunu yazar. Dolayisiyla taze
        status da bayat-status yedegi olan komut da ``eklem_acisi`` ile q'ya
        cevrilmelidir.

        Gercek ArduPilot servo-mount status'u farkli bir sozlesmeye sahiptir:
        govdeye gore eklem acisidir. O yol ROS'suz ``donanim/kamera_kopru.py``
        icinde ayrica ele alinir; donanim yorumu buraya tasinamaz.
        """
        if self.tilt_okuyucu is None:
            return None, None, None
        yas = self.tilt_okuyucu.yas_s()
        if yas is not None and yas < 1.5 and self.tilt_okuyucu.deger_rad is not None:
            eps = math.degrees(self.tilt_okuyucu.deger_rad)
        elif (self.tilt_komutcu is not None
              and self.tilt_komutcu.hedef_deg is not None):
            eps = self.tilt_komutcu.hedef_deg
        else:
            return None, None, yas
        return eklem_acisi(eps, pitch_i, roll_i), eps, yas

    def _report(self, valid_detection, box, coverage):
        """Saniyede bir ozet satiri: fps + tespit orani (log dosyasi icin)."""
        now = time.time()
        if valid_detection:
            x, y, w, h = box
            print(f"HEDEF merkez=({x + w // 2},{y + h // 2}) bbox=({x},{y},{w},{h}) "
                  f"cov={coverage:.2f}% t_unix={now:.3f}", flush=True)
        if now - self.last_report_time >= 5.0:
            frames = self.frame_count - self.last_report_frames
            fps = frames / (now - self.last_report_time)
            oran = (self.detect_count / self.frame_count * 100) if self.frame_count else 0
            ek = ''
            if self.son_stab is not None and self.tutum is not None:
                sx, sy, ex, ey = self.son_stab
                ek = (f" | gimbal: sanal=({sx:.0f},{sy:.0f}) "
                      f"hata=({ex:+.2f},{ey:+.2f}) deg "
                      f"tutum=({math.degrees(self.tutum.roll):+.1f},"
                      f"{math.degrees(self.tutum.pitch):+.1f})")
                if self.aim_trim is not None:
                    ek += (f" aim={self.gimbal.aim_pitch_deg:+.2f}"
                           f"{'[K]' if self.aim_trim.kelepcede else ''}")
                if self.tilt_okuyucu is not None:
                    yas = self.tilt_okuyucu.yas_s()
                    cmd = (self.tilt_komutcu.hedef_deg
                           if self.tilt_komutcu is not None else None)
                    # Gazebo status'u DUNYA elevasyonudur. Rapor hem ayni
                    # cercevedeki komut/status'u hem de bunlardan tureyen
                    # govdeye-gore eklem acilarini yan yana gosterir.
                    st_eps = (None if self.tilt_okuyucu.deger_rad is None
                              else math.degrees(self.tilt_okuyucu.deger_rad))
                    q_cmd = (None if cmd is None else
                             eklem_acisi(cmd, self.tutum.pitch,
                                         self.tutum.roll))
                    q_st = (None if st_eps is None else
                            eklem_acisi(st_eps, self.tutum.pitch,
                                        self.tutum.roll))
                    ek += (f" tilt_world={'-' if cmd is None else f'{cmd:+.1f}'}"
                           f"/{'-' if st_eps is None else f'{st_eps:+.1f}'}"
                           f" joint={'-' if q_st is None else f'{q_st:+.1f}'}"
                           f"/{'-' if q_cmd is None else f'{q_cmd:+.1f}'}deg")
                    # olu-adam dersi (goruntulu-olu-adam-anahtari): sessiz
                    # sapma/bayatlik OZET'te bagirsin
                    if (st_eps is not None and cmd is not None
                            and abs(st_eps - cmd) > 2.0):
                        ek += " [TILT SAPMA UYARI]"
                    if yas is None or yas > 3.0:
                        ek += " [TILT STATUS BAYAT]"
            print(f"[OZET] kare={self.frame_count} fps={fps:.1f} "
                  f"tespit_orani=%{oran:.1f} mod={self.current_mode}{ek}", flush=True)
            self.last_report_time = now
            self.last_report_frames = self.frame_count

    # ---------------- cizim ----------------

    def _draw(self, cv_image, box, komut_yetkisi, valid_count, coverage,
              w_img, h_img):
        FONT, AA = cv2.FONT_HERSHEY_DUPLEX, cv2.LINE_AA

        # Vurus alani (kadrajin orta bandi)
        cv2.rectangle(cv_image, (int(w_img * 0.25), int(h_img * 0.10)),
                      (int(w_img * 0.75), int(h_img * 0.90)), (0, 255, 255), 1)
        cv2.putText(cv_image, "Vurus Alani", (int(w_img * 0.25) + 4,
                    int(h_img * 0.10) + 14), FONT, 0.35, (0, 255, 255), 1, AA)

        if box is not None:
            x, y, w, h = box
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.drawMarker(cv_image, (x + w // 2, y + h // 2), (255, 0, 255),
                           cv2.MARKER_CROSS, 12, 1)

        # --- SANAL GIMBAL GORSELLESTIRMESI ---
        # Videoda gimbalin CALISTIGI gorunsun diye: sanal ufuk cizgisi (govde
        # yattikca kadrajda egilir/kayar - de-rotasyonun ta kendisi), sanal
        # kadraj merkezi ve hedefin sanal karsiligi.
        if self.gimbal is not None and self.tutum is not None and self.tutum.hazir:
            g, tt = self.gimbal, self.tutum
            _ornek = tt.tutum_al(time.monotonic())
            roll, pitch = (_ornek[0], _ornek[1]) if _ornek else (tt.roll, tt.pitch)
            # Ufuk: yukselis 0 olan noktalarin ham kadrajdaki izi.
            # Fiziksel gimbal modunda canli eklem kullanilir: ufuk artik
            # kadrajda sabit yukseklikte durur, yalniz roll'le egilir --
            # gimbalin calistiginin gorsel kaniti.
            _eklem = self._son_eklem_deg
            noktalar = []
            for yan in range(-30, 31, 3):
                pt = g.piksel_uret(0.0, yan, roll, pitch, eklem_deg=_eklem)
                if pt and -2000 < pt[0] < 3000 and -2000 < pt[1] < 3000:
                    noktalar.append((int(pt[0]), int(pt[1])))
            for i in range(1, len(noktalar)):
                cv2.line(cv_image, noktalar[i - 1], noktalar[i], (0, 200, 255), 1, AA)
            if noktalar:
                cv2.putText(cv_image, "ufuk", (noktalar[0][0] + 4, noktalar[0][1] - 6),
                            FONT, 0.4, (0, 200, 255), 1, AA)
            # Sanal kadraj merkezi (aim uygulanmis): ufka gore -aim yukseliste
            merkez = g.piksel_uret(-g.aim_pitch_deg, 0.0, roll, pitch,
                                   eklem_deg=_eklem)
            if merkez and 0 <= merkez[0] < w_img and 0 <= merkez[1] < h_img:
                cv2.drawMarker(cv_image, (int(merkez[0]), int(merkez[1])),
                               (0, 255, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
                cv2.putText(cv_image, "sanal merkez",
                            (int(merkez[0]) + 14, int(merkez[1]) + 4),
                            FONT, 0.4, (0, 255, 255), 1, AA)
            if self.son_stab is not None:
                sx, sy, ex, ey = self.son_stab
                cv2.putText(cv_image, f"gimbal hata: {ex:+.2f} / {ey:+.2f} deg",
                            (8, h_img - 34), FONT, 0.5, (0, 255, 255), 1, AA)
            cv2.putText(cv_image,
                        f"roll {math.degrees(roll):+5.1f}  pitch {math.degrees(pitch):+5.1f}"
                        f"  aim {g.aim_pitch_deg:+.2f}  gecikme {tt.gecikme_s*1000:.0f}ms",
                        (8, h_img - 12), FONT, 0.5, (200, 200, 200), 1, AA)

        overlay = cv_image.copy()
        cv2.rectangle(overlay, (0, 0), (w_img, 72), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, cv_image, 0.45, 0, cv_image)

        if komut_yetkisi == 'goruntulu':
            hud_color, gorev = (0, 255, 0), "GORUNTULU"
        else:
            hud_color, gorev = (0, 165, 255), "KONUMLU"
        cv2.putText(cv_image, f"MOD: {gorev}", (8, 20), FONT, 0.5, hud_color, 1, AA)
        if self.kod is not None:
            cv2.putText(cv_image, self.kod.text, (8, 62), FONT, 0.45,
                        (255, 255, 255), 1, AA)

        hedef_text = "HEDEF: BULUNDU" if box is not None else "HEDEF: YOK"
        hedef_color = (0, 255, 0) if box is not None else (100, 100, 255)
        cv2.putText(cv_image, hedef_text, (8, 40), FONT, 0.45, hedef_color, 1, AA)

        pencere_text = f"Pencere: {valid_count}/{len(self.decision_window)}"
        (tw, _), _ = cv2.getTextSize(pencere_text, FONT, 0.4, 1)
        cv2.putText(cv_image, pencere_text, (w_img - tw - 10, 20), FONT, 0.4,
                    (180, 180, 180), 1, AA)
        if box is not None:
            cov_text = f"Cov: {coverage:.2f}%"
            (tw2, _), _ = cv2.getTextSize(cov_text, FONT, 0.4, 1)
            cv2.putText(cv_image, cov_text, (w_img - tw2 - 10, 40), FONT, 0.4,
                        (180, 220, 180), 1, AA)

        if self.record_path is not None:
            self._record(cv_image, w_img, h_img)
        if self.display:
            cv2.imshow("Yildizlar bbox", cv_image)
            cv2.waitKey(1)

    # Yazici kurulmadan once kareleri burada biriktiririz (asagidaki hataya bak).
    OLCUM_SURESI_S = 2.0

    def _record(self, cv_image, w_img, h_img):
        """bbox cizili kareyi videoya yaz.

        HATA VE DUZELTMESI (2026-08-01): yazici ILK karede kuruluyordu; o anda
        gecen sure ~0 oldugu icin "olcum yoksa 5 fps varsay" dalina dusuyor ve
        dosyanin basligina 5 fps yaziliyordu. Kameralar 30 Hz oldugu icin
        kayitlar 6 KAT AGIR CEKIM oynuyordu (olcum: 360 s'lik kosudan cikan
        dosyada 10913 kare ama baslikta fps=5 -> dosya 2183 s gorunuyor).
        Kamera degil YAZICI hataliydi.
        Cozum: yazici kurulmadan once OLCUM_SURESI_S kadar kare biriktir,
        gercek hizi olc, sonra yaziciyi o hizla kur ve birikeni bosalt.
        --record-fps ile hala elle sabitlenebilir.
        """
        if self.video_writer is None:
            if self.record_fps > 0:
                fps = self.record_fps
            else:
                gecen = time.time() - (self.first_frame_time or time.time())
                if gecen < self.OLCUM_SURESI_S:
                    self._kayit_tampon.append(cv_image.copy())
                    return
                # frame_count ilk kareden beri artiyor; olculen hiz budur.
                fps = self.frame_count / max(gecen, 1e-3)
                fps = max(1.0, min(60.0, fps))
            self.video_writer = cv2.VideoWriter(
                self.record_path, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (w_img, h_img))
            print(f"Video kaydi: {self.record_path} ({w_img}x{h_img} @ "
                  f"{fps:.1f} fps, {len(self._kayit_tampon)} kare tamponundan)",
                  flush=True)
            for kare in self._kayit_tampon:
                self.video_writer.write(kare)
            self._kayit_tampon = []
        self.video_writer.write(cv_image)

    def close(self):
        if self.kod is not None:
            self.kod.stop()
        if self.tilt_komutcu is not None:
            self.tilt_komutcu.dur()
        if self.tilt_okuyucu is not None:
            self.tilt_okuyucu.dur()
        if self.tutum is not None:
            self.tutum.log_kapat()
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
        if self.video_writer is not None:
            self.video_writer.release()
            print(f"Video kaydedildi: {self.record_path}", flush=True)
        cv2.destroyAllWindows()


def main():
    # ROS yolu: bu script'in kendisi ROS Image abonesidir. ROS yoksa
    # argumanlari ayristirmadan ONCE anlasilir sekilde cik.
    ros_gerekli('bbox_to_redis.py (ROS Image abonesi)')
    root = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default=os.environ.get(
        'YILDIZ_CAM_TOPIC', '/drone_1/webcam/image_raw'),
        help='dinlenecek ROS kamera topic\'i')
    parser.add_argument('--no-display', action='store_true',
                        help='OpenCV penceresi acma (headless)')
    parser.add_argument('--record', nargs='?', const='', default=None,
                        metavar='DOSYA',
                        help='bbox cizili kareleri kaydet; yol verilmezse '
                             'videos/gudum_YYYYmmdd_HHMMSS.mp4')
    parser.add_argument('--record-fps', type=float, default=0.0,
                        help='kayit fps (0 = olculen hizi kullan)')
    parser.add_argument('--mavlink-port', type=int, default=0,
                        help='sanal gimbal icin ATTITUDE portu; 0 = gimbal kapali')
    parser.add_argument('--mount', type=float, default=30.0,
                        help='fiziksel kamera montaj acisi (derece, + = yukari). '
                             'YALNIZ --no-tilt (eski govdeye-sabit kamera) yolunda '
                             'kullanilir; fiziksel gimbal modunda canli eklem '
                             'acisi gecerlidir')
    parser.add_argument('--tilt', type=float, default=None,
                        help='FIZIKSEL GIMBAL: komutlanacak kamera dunya '
                             'elevasyonu (derece, + = yukari). Verilmezse '
                             'standoff geometrisinden atan(down/back) turetilir')
    parser.add_argument('--no-tilt', dest='tilt_acik', action='store_false',
                        default=True,
                        help='fiziksel gimbal komut/okuma KAPALI: eski '
                             'govdeye-sabit kamera zinciri (--mount) kullanilir')
    parser.add_argument('--tilt-sabit', action='store_true',
                        help='FAZ C KAPALI: tilt standoff degerinde sabit '
                             'kalir (Faz A davranisi). Varsayilan: tilt '
                             'hedefin olculen yukselisini IZLER')
    parser.add_argument('--gz-model', default=None,
                        help='gazebo sarmalayici model adi (or. iris-1); '
                             'verilmezse --topic /drone_N/... adresinden turetilir')
    parser.add_argument('--aim', type=float, default=None,
                        help='aim ofseti (derece); verilmezse --back/--down\'dan '
                             'analitik hesaplanir. sanal kadraj merkezi = -aim')
    parser.add_argument('--back', type=float, default=25.0,
                        help='gudumun standoff geri mesafesi (m) - analitik aim icin')
    parser.add_argument('--down', type=float, default=4.0,
                        help='gudumun standoff dikey ofseti (m). Tilt/aim '
                             'turetiminin girdisi. VARSAYILAN 4 = '
                             'standoff_geom.sh YILDIZ_DOWN_TASARIM ile ayni '
                             '(eski 13, +30 montaj doneminin degeriydi; '
                             'yanlis varsayilan tilt 27.5 baslatip hedefi '
                             'FOV disina atiyordu)')
    parser.add_argument('--aim-trim', dest='aim_trim', action='store_true',
                        default=True, help='yavas aim trim ACIK (varsayilan)')
    parser.add_argument('--no-aim-trim', dest='aim_trim', action='store_false',
                        help='aim analitik degerde SABIT kalsin')
    parser.add_argument('--aim-kelepce', type=float, default=6.0,
                        help='trim baslangictan en fazla bu kadar sapabilir (derece)')
    parser.add_argument('--gimbal-log', default=None, metavar='CSV',
                        help='kare basina ham/stabilize hata + tutum CSV\'ye')
    parser.add_argument('--tutum-log', default=None, metavar='CSV',
                        help='ham ATTITUDE zaman serisi CSV\'ye (zaman kalibrasyonu icin)')
    parser.add_argument('--kamera-gecikme-ms', type=float, default=0.0,
                        help='kamera boru hatti gecikmesi (ms). tutum bu kadar '
                             'GERIYE interpolasyonla tasinir. '
                             'tools/gimbal_zaman_kalibre.py ile olculur.')
    args = parser.parse_args()

    record_path = None
    env_video = os.environ.get('YILDIZ_VIDEO', '').strip().lower()
    if args.record is not None or env_video in ('1', 'true', 'yes', 'on'):
        record_path = args.record or ''
        if not record_path:
            os.makedirs(os.path.join(root, 'videos'), exist_ok=True)
            # YILDIZ_VIDEO_ETIKET: deneme kampanyasinda videolarin izlenebilir
            # adlanmasi icin (or. "los_elips" -> los_elips_20260803_...mp4).
            etiket = os.environ.get('YILDIZ_VIDEO_ETIKET', '').strip() or 'gudum'
            record_path = os.path.join(
                root, 'videos',
                f"{etiket}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")

    display = not args.no_display and bool(os.environ.get('DISPLAY'))
    if not args.no_display and not display:
        print("DISPLAY yok; pencere acilmayacak.")

    detector = None
    try:
        gimbal = tutum = trim = None
        tilt_okuyucu = tilt_komutcu = tilt_takip = None
        if args.mavlink_port:
            gz_model = args.gz_model or model_adi_topikten(args.topic)
            if args.tilt_acik and gz_model is None:
                print("UYARI: --gz-model turetilemedi (topic beklenen kalipta "
                      "degil); fiziksel gimbal KAPALI, eski zincire dusuluyor.")
            if args.tilt_acik and gz_model is not None:
                # FIZIKSEL GIMBAL MODU (gimbal dali, Faz A): kamera stabilize
                # gimbalde. Standoff geometrisi tilt KOMUTUNU belirler
                # (eps = atan(down/back)); sanal aim ofseti ve sabit mount
                # OLUR. Aim trim bu fazda KAPALI (Faz B'de tilt setpoint'ine
                # yonlenecek).
                if args.tilt is not None:
                    eps_cmd = args.tilt
                    print(f"tilt elle verildi: {eps_cmd:+.2f} deg")
                else:
                    eps_cmd = -analitik_aim(args.back, args.down)
                    print(f"tilt (standoff geometrisi): back={args.back:.0f} "
                          f"down={args.down:.0f} -> {eps_cmd:+.2f} deg")
                gimbal = SanalGimbal(mount_phys_pitch_deg=0.0,
                                     aim_pitch_deg=0.0)
                print(gimbal.ozet())
                print(f"FIZIKSEL GIMBAL: {gz_model} <- tilt {eps_cmd:+.2f} deg "
                      "(aim/mount devre disi, canli eklem zincirde)")
                tilt_okuyucu = TiltDurumOkuyucu(gz_model).basla()
                tilt_komutcu = TiltKomutcu(gz_model).basla()
                tilt_komutcu.hedef(eps_cmd)
                if args.tilt_sabit:
                    tilt_takip = None
                    print("FAZ C KAPALI (--tilt-sabit): tilt standoff "
                          "degerinde sabit")
                else:
                    # FAZ C: tilt hedefin olculen yukselisini izler; tespit
                    # kaybinda standoff acisina yavasca doner (yeniden edinim)
                    tilt_takip = TiltTakip(varsayilan_deg=eps_cmd)
                    print(f"FAZ C ACIK: tilt takibi (varsayilan/yeniden-edinim "
                          f"{eps_cmd:+.2f} deg, arka uc "
                          f"{'KALICI' if tilt_komutcu.kalici else 'gz CLI'})")
            else:
                # ESKI YOL (govdeye sabit kamera): (a) sikki - aim'i KENDI
                # komut geometrinden baslat. --aim verilirse o ezer.
                if args.aim is None:
                    aim0 = analitik_aim(args.back, args.down)
                    print(f"aim baslangici (analitik): back={args.back:.0f} "
                          f"down={args.down:.0f} -> {aim0:+.2f} deg")
                else:
                    aim0 = args.aim
                    print(f"aim elle verildi: {aim0:+.2f} deg")
                gimbal = SanalGimbal(mount_phys_pitch_deg=args.mount,
                                     aim_pitch_deg=aim0)
                print(gimbal.ozet())
                if args.aim_trim:
                    trim = AimTrim(aim0, kelepce_deg=args.aim_kelepce)
                    print(trim.ozet())
                else:
                    print("aim trim KAPALI - aim sabit")
            tutum = TutumOkuyucu(args.mavlink_port,
                                 gecikme_s=args.kamera_gecikme_ms / 1000.0)
            if args.tutum_log:
                tutum.log_ac(args.tutum_log)
                print(f"Tutum logu: {args.tutum_log}", flush=True)
            tutum.start()
            print(f"Kamera boru hatti gecikmesi: {args.kamera_gecikme_ms:.1f} ms "
                  f"(tutum bu kadar geriye tasinir)")
        detector = SuruRedisDetector(args.topic, display, record_path,
                                     args.record_fps, gimbal, tutum,
                                     trim, args.gimbal_log,
                                     tilt_okuyucu=tilt_okuyucu,
                                     tilt_komutcu=tilt_komutcu,
                                     tilt_takip=tilt_takip)

        # --- TEMIZ KAPANIS ---
        # HATA (2026-08-01): yildizlar_gudum.sh --stop surece SIGTERM/SIGKILL
        # gonderiyordu; burada yalniz SIGINT (KeyboardInterrupt) yakalaniyordu,
        # dolayisiyla VideoWriter.release() HIC calismiyor ve mp4'un moov
        # atomu yazilmiyordu -> 75 MB'lik dosya acilmiyordu. rospy kendi SIGINT
        # isleyicisini kurdugu icin init_node'dan SONRA kaydediyoruz.
        def _kapat(signum, frame):
            print(f"sinyal {signum} alindi, temiz kapaniyor...", flush=True)
            rospy.signal_shutdown('kapat')
        signal.signal(signal.SIGTERM, _kapat)
        signal.signal(signal.SIGINT, _kapat)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None:
            detector.close()


if __name__ == '__main__':
    main()
