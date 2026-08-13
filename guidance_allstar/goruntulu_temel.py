#!/usr/bin/env python3
"""
goruntulu_temel.py - GORUNTULU gudum ortak iskeleti (LOS / PID / MPC tabani)
============================================================================
Konumlu gudum (simple_guided_follow.py) hedefi kadraja sokar; bbox_to_redis.py
karar vericisi hedef ~1.5 s kadrajda kalinca Redis 'komut_yetkisi' anahtarini
'goruntulu' yapar. O andan itibaren ARAC BU ISKELETIN uzerine kurulu kodun
elindedir: konumlu surec setpoint gondermeyi keser (yetki kapisi orada) ama
olmeye devam ETMEZ -- karar verici 'konumlu'ya geri donerse gudumu o surdurur.

SOZLESME (uc yontem de buna uyar):
  * Hata sinyali SANAL GIMBALDEN gelir: bbox_to_redis.py 'tracker_bbox_stab'
    kanalina [sx, sy, w, h, ex_deg, ey_deg, t_capture] basar. ex/ey govde
    salinimindan arindirilmis acisal hatadir (yildizlar_gimbal.aci_hatasi).
    Ham piksel okumak YASAK degildir ama gereksizdir; gimbal katmani asagida
    hazir veriyor.
  * MENZIL YALNIZ ESTIMATORDAN: hedefin 3D konum telemetrisine az guveniyoruz;
    ondan turetilmesine izin verilen TEK buyukluk menzildir (LOS vektorunun
    uzunlugu). MenzilKestirici sinifi filterwndr IMM'ini besler ve DISARIYA
    SADECE menzil() verir. Hedef hizi, yonu, ivmesi vb. TURETILMEZ,
    KULLANILMAZ (kullanici kurali, 2026-08-03). Kacirma yardimi (lost assist)
    ILERIDE gelebilir; simdilik yok.
  * Komut HIZDIR: SET_POSITION_TARGET_LOCAL_NED, yalniz vx,vy,vz (+istege
    bagli yaw_rate; FOV kontrolcu yaw'i bununla surer). Attitude KOMUTLANMAZ.
  * GECIS YUMUSAKTIR: konumlu her dongude son komutunu 'devir_durumu'
    anahtarina yazar. Devir aninda kontrolcunun tohumla() cagrisi ve komut
    LPF'sinin bu hizla tohumlanmasi sayesinde arac sicrama gormez
    ("integral tohumlama").
  * DT OLCULUR: dongu dt'si duvar saatinden olculur, nominale guvenilmez
    (2 Hz'e dusen dongu + sabit dt varsayimi dev daire/titreme koku idi).

KULLANIM (yontem kodu ornegi -- los_gudum.py / pid_gudum.py / mpc_gudum.py):

    from goruntulu_temel import (GoruntuluKontrolcu, GoruntuluDongu, Komut)

    class LosKontrolcu(GoruntuluKontrolcu):
        ad = "los"
        def tohumla(self, devir):        # devir: dict|None ('devir_durumu')
            self.v_int = np.array(devir["cmd_vel_ned"]) if devir else np.zeros(3)
        def komut(self, o):              # o: Olcum
            ...                          # o.ex_deg, o.ey_deg, o.menzil_m, ...
            return Komut(vel_ned=v, yaw_rate_dps=r)

    GoruntuluDongu(LosKontrolcu()).calistir()

AMAC FONKSIYONU (deney karsilastirmasi icin loglanir): hedefi merkezde tut
(|ex|,|ey| kucuk) VE sqrt(bbox alani) buyusun (collision'a yaklasma). Log
kolonlari deneme_ozeti/karsilastirma araclarina girdi olur.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import redis
from pymavlink import mavutil

import guidance_config as cfg
import mavlink_utils
from filterwndr import clamp_filter_dt, predict_imm_over_dt, setup_imm_filter
from numeric_differentiation import VelocityDifferentiator

# SET_POSITION_TARGET_LOCAL_NED tip maskeleri:
# pozisyon+ivme+yaw yoksay, hiz kullan (yaw_rate istege bagli).
_MASK_HIZ = (1 + 2 + 4) + (64 + 128 + 256) + 1024 + 2048          # yalniz vx,vy,vz
_MASK_HIZ_YAWRATE = (1 + 2 + 4) + (64 + 128 + 256) + 1024         # + yaw_rate

# OLU-ADAM ANAHTARI: goruntulu kontrolcu her dongude 'goruntulu_hayatta'
# yazar, TTL ile. Surec olurse/asili kalirsa anahtar TTL sonunda dusER ve
# bbox_to_redis yetkiyi goruntuluye DEVRETMEZ (ya da geri alir). TTL, en
# yavas kabul edilebilir donguden (2 Hz) belirgin buyuk secildi: 20 Hz'lik
# normal koşuda anahtar surekli tazelenir, gercek olumde 2 s'de dusER.
HAYATTA_TTL_S = 2

# --- KADRAJ GEOMETRISI (yalniz LOG icin; gudum sanal gimbalin ex/ey'sini
# kullanir). Raspberry Pi AI Camera (IMX500) 1280x720, hfov 66 deg ->
# fx = (1280/2)/tan(33 deg) = 985.5. bbox_to_redis.YildizlarGimbal ile ayni.
_KADRAJ_W = float(getattr(cfg, "KAMERA_GENISLIK_PX", 1280))
_KADRAJ_H = float(getattr(cfg, "KAMERA_YUKSEKLIK_PX", 720))
_KADRAJ_HFOV_RAD = float(getattr(cfg, "KAMERA_HFOV_RAD", 1.1519))
_KADRAJ_FX = (_KADRAJ_W / 2.0) / math.tan(_KADRAJ_HFOV_RAD / 2.0)

# Karsilasma tipi esikleri [deg]. Aci tanimi _karsilasma_geometrisi'nde.
_TIP_KAFA_KAFAYA_DEG = 60.0
_TIP_KUYRUK_DEG = 120.0

# --- KISA BOSLUKTA YAW SURDURME (env: YILDIZ_TUT_YAW, VARSAYILAN
# ACIK 2026-08-10'dan beri -- kapamak icin YILDIZ_TUT_YAW=0) ----------
#
# BUGUNKU DAVRANIS VE OLCULEN BEDELI. Asagidaki 'tut' kolunda (bbox
# kisa sureligine bayat, son gecerli hiz komutu tutuluyor) yaw_rate
# BILINCLI OLARAK None birakiliyor; gerekcesi "kor donus hedefi
# yatayda da kaybettirir" idi. Elips hedefte olculdu: hedefin DONUS
# karelerinin %38-42'sinde araca HIC yaw komutu gitmiyor. Sonuc bir
# POZITIF GERI BESLEME: hedef viraj yonunde kadrajdan kayar -> tespit
# duser -> daha cok 'tut' karesi -> daha cok yawsiz kare. Duz bacakta
# bu zararsiz (hedef kadrajda kalir), donuste angajmani bitiriyor.
#
# DUZELTME VE TARIHSEL ENDISENIN KARSILANMASI. Yaw'i "dondurmak"
# (sabit tutmak) gercekten tehlikelidir: bbox bir daha gelmezse arac
# sabit hizla donmeye devam eder. Bu yuzden surdurme (a) USSEL
# SONUMLU -- her karede exp(-dt/tau) ile kucululur, (b) SURE SINIRLI
# -- azami TUT_YAW_AZAMI_S sonra tamamen birakilir (None). Yani en
# kotu durumda araca giden toplam ek yaw acisi
#   |yaw0| * tau * (1 - exp(-azami/tau)) ~ 0.78*|yaw0| derece-saniye
# kadardir; 30 dps'lik bir komut icin ~23 deg. Kor DONUS degil, kor
# SONUMLENME. 'suz' (uzun kayip) koluna DOKUNULMAZ.
TUT_YAW_TAU_S = 1.0        # sonumleme zaman sabiti
TUT_YAW_AZAMI_S = 1.5      # bu sureden sonra yaw tamamen birakilir


def _cevre_bayrak(anahtar: str, varsayilan: float = 0.0) -> float:
    """Ortamdan sayi oku; bozuksa/yoksa varsayilana dus."""
    try:
        deger = os.environ.get(anahtar)
        return varsayilan if deger is None else float(deger)
    except (TypeError, ValueError):
        return varsayilan


# ---------------------------------------------------------------------- veri

@dataclass
class Olcum:
    """Kontrolcuye her dongude verilen tam olcum paketi."""
    t: float                    # time.monotonic()
    dt: float                   # OLCULEN dongu adimi [s]
    # --- sanal gimbal (tracker_bbox_stab) ---
    ex_deg: float | None        # yatay acisal hata (+: hedef sagda)
    ey_deg: float | None        # dikey acisal hata (+: hedef asagida)
    bbox_w: float | None        # ham bbox genisligi [px]
    bbox_h: float | None
    alan_kok: float | None      # sqrt(w*h) -- buyutmeye calistigimiz buyukluk
    kapsama_pct: float | None   # yatay kapsama [%]
    bbox_yas_s: float           # son gecerli tespitten beri gecen sure
    # --- estimator (SADECE menzil) ---
    menzil_m: float | None
    # --- kendi durumumuz (LOCAL_POSITION_NED / ATTITUDE) ---
    pos_ned: np.ndarray | None
    vel_ned: np.ndarray | None
    yaw_rad: float | None
    roll_rad: float | None
    pitch_rad: float | None
    # --- istege bagli (varsayilanli; eski kurucular kirilmasin) ---
    t_capture: float | None = None
    # Karenin YAKALANMA ani (ROS saati, tracker_bbox_stab[6]). Baska saatle
    # karistirma; yalniz ardisik olcumlerin FARKI anlamlidir (lambda-nokta
    # faz eslemesi gibi gecikme-duyarli turevler icin).
    px_sanal_x: float | None = None     # sanal (stabilize) piksel [px]
    px_sanal_y: float | None = None
    px_ham_cx: float | None = None      # HAM bbox merkezi [px] (kadraj 1280x720)
    px_ham_cy: float | None = None
    acc_ned: np.ndarray | None = None   # KENDI ivmemiz [m/s^2, NED] (turev+LPF)
    tilt_deg: float | None = None
    # FAZ C (gimbal dali): kameranin o karedeki DUNYA elevasyonu
    # (tracker_bbox_stab[7], bbox_to_redis yayinlar). Tilt artik dinamik
    # oldugu icin gudum ey_ref'i statik YILDIZ_TILT yerine BUNDAN kurmali
    # (mpc_gudum._kadraj_sabiti oyle yapar; None ise statik degere duser).
    # Not: px_* ve acc_ned LOG icin eklendi; kontrolculer bunlari kullanmak
    # ZORUNDA DEGIL (sozlesme hala ex/ey + menzil). Ham piksel yasak degil.
    # --- DONUK DIKDORTGEN (bbox_to_redis YILDIZ_MINRECT=1 iken yayinlar) ---
    # tracker_bbox_stab[8..10]. Bayrak kapaliyken alanlar YOK -> hepsi None
    # kalir (tilt_deg ile ayni opsiyonel-alan deseni).
    # rot_w_px : donuk dikdortgenin UZUN kenari [px] (>= rot_h_px)
    # rot_h_px : KISA kenar [px]
    # rot_aci_deg : uzun eksenin ekran yatayina gore acisi, saat-yonunun-
    #   TERSI pozitif, [-90,+90) EKSEN acisi (yon bilgisi yok, mod 180).
    #   Hedef banki bundan ve KENDI roll'umuzden turetilir -- yani AABB
    #   aspect'inden farkli olarak kamera roll'u kaynakta ayrisir ve isaret
    #   (sag/sol donus) korunur. Turetme tuketiciye birakildi; burada
    #   yalniz TASINIR (Olcum sozlesmesi: olculen, yorumlanmamis buyukluk).
    rot_w_px: float | None = None
    rot_h_px: float | None = None
    rot_aci_deg: float | None = None
    vibe_max: float | None = None       # KENDI VIBRATION'imizin en buyuk ekseni
    # 2026-08-05: iskelet bu sayiyi ZATEN okuyup logluyordu (vibe_max kolonu)
    # ama kontrolcuye VERMIYORDU. Kontrolcuye acildi cunku FIZIKSEL TEMASIN
    # tek ic gostergesi budur (tur-3 sim: 2.32 m'de vibe 2.0 -> 17.4, 1.04
    # m'de 25.5; degmedigimiz tur-2 geciste 0.85 m'de yalnizca 3.3).
    # KURAL IHLALI DEGIL: bu BIZIM telemetrimiz, hedefin degil -- hedeften
    # hala yalnizca menzil kullaniliyor.


@dataclass
class Komut:
    """Kontrolcunun dondurdugu komut. vel_ned NED cercevesindedir [m/s]."""
    vel_ned: np.ndarray
    yaw_rate_dps: float | None = None    # None = yaw otopilotta kalir
    birak: bool = False                  # True = yetkiyi birak (ISKA)
    birak_sebep: str = ''                # birak=True ise insan-okunur sebep
    olay: str = ''                       # kontrolcunun ilan ettigi AYRIK olay
    olay_detay: str = ''
    # 2026-08-05: kontrolcunun olay logunu (_olay.csv) besleyebilmesi icin.
    # Ilk tuketici VURUS_BASARILI (bkz. mpc_gudum._vurus_basarili_kontrol):
    # vurus sayisi kosu ozetinden DOGRUDAN okunabilsin diye -- eskiden CPA
    # tahmininden cikariliyordu ve her basarili vurus kosuyu bitirdigi icin
    # (arac takla atip dusuyor) ornek sayisi 5'te kaliyordu. Bos string =
    # olay yok; iskelet yalnizca DOLU olanlari yazar.


class GoruntuluKontrolcu:
    """Yontem siniflarinin taban sinifi."""
    ad = "temel"

    def tohumla(self, devir):
        """Devir aninda bir kez cagrilir; devir 'devir_durumu' dict'i ya da
        None (konumlu hic yazamadiysa). Entegratorler burada tohumlanir."""

    def komut(self, olcum: Olcum) -> Komut:
        raise NotImplementedError


def govde_ileri_ned(yaw_rad, ileri, sag, asagi):
    """Govde-duzlem (ileri, sag, asagi) hizini NED'e cevirir (yalniz yaw)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([c * ileri - s * sag, s * ileri + c * sag, asagi])


# ------------------------------------------------- LOG-ONLY analiz yardimcisi
#
# UYARI -- KONTROL YOLU BU BOLUMU CAGIRMAZ.
# Kullanici kurali (2026-08-03): hedefin 3D telemetrisinden GUDUM yalniz
# MENZILI kullanabilir. Asagidaki fonksiyon hedefin konumunu/hizini/ivmesini
# kullanir; ciktisi SADECE CSV'ye yazilir, hicbir kontrolcuye verilmez.
# Kolon adlarinda 'ref_' oneki bu ayrimin gorunur isaretidir: bir kolon
# 'ref_' ile basliyorsa o sayi GUDUME GIRMEMISTIR, yalniz analiz icindir.
# Bir kontrolcunun ref_* okudugunu gorursen bu bir KURAL IHLALIDIR.


def _karsilasma_geometrisi(pos, vel, hedef_pos, hedef_vel):
    """Avci/hedef durumundan karsilasma geometrisi (LOG/ANALIZ ICIN).

    Isaret konvansiyonlari:
      kerteriz_deg   : bizden hedefe pusula kertizi, 0=kuzey, +dogu, [0,360)
      yukselis_deg   : hedefin bize gore yukselis acisi, + = hedef YUKARIDA
      yaklasim_deg   : hedefin GIDIS YONU ile 'hedeften bize' vektoru arasindaki
                       aci. 0 deg = hedef tam uzerimize geliyor (KAFA KAFAYA),
                       180 deg = hedef bizden uzaklasiyor, biz arkasindayiz
                       (KUYRUK TAKIBI). 90 deg civari = CAPRAZ.
      kapanma_mps    : menzilin kapanma hizi, + = mesafe KISALIYOR
      tgo_s          : menzil / kapanma (yalniz kapaniyorken)
      cpa_m / cpa_s  : sabit hiz varsayimiyla en yakin gecis mesafesi ve ona
                       kalan sure. cpa_s < 0 ise en yakin gecis GECMISTE kaldi
                       (0'a kelepcelenir, cpa_m o an menzile esitlenir).
    Eksik veri alanlari None doner; sozluk anahtarlari her zaman ayni.
    """
    bos = {'menzil_m': None, 'kerteriz_deg': None, 'yukselis_deg': None,
           'yaklasim_deg': None, 'tip': '', 'kapanma_mps': None,
           'tgo_s': None, 'cpa_m': None, 'cpa_s': None}
    if pos is None or hedef_pos is None:
        return bos
    r = np.asarray(hedef_pos, float).reshape(3) - np.asarray(pos, float).reshape(3)
    menzil = float(np.linalg.norm(r))
    g = dict(bos)
    g['menzil_m'] = menzil
    yatay = math.hypot(float(r[0]), float(r[1]))
    g['kerteriz_deg'] = math.degrees(math.atan2(float(r[1]), float(r[0]))) % 360.0
    g['yukselis_deg'] = math.degrees(math.atan2(-float(r[2]), max(yatay, 1e-6)))
    if vel is None or hedef_vel is None or menzil < 1e-6:
        return g

    v_bizim = np.asarray(vel, float).reshape(3)
    v_hedef = np.asarray(hedef_vel, float).reshape(3)
    u_los = r / menzil
    v_bagil = v_hedef - v_bizim                      # hedefin bize gore hizi
    g['kapanma_mps'] = -float(np.dot(v_bagil, u_los))
    if g['kapanma_mps'] > 0.1:
        g['tgo_s'] = menzil / g['kapanma_mps']

    # En yakin gecis (CPA): sabit bagil hizla dogrusal ekstrapolasyon.
    n2 = float(np.dot(v_bagil, v_bagil))
    if n2 > 1e-6:
        t_cpa = -float(np.dot(r, v_bagil)) / n2
        if t_cpa < 0.0:                              # gecis geride kaldi
            g['cpa_s'], g['cpa_m'] = 0.0, menzil
        else:
            g['cpa_s'] = t_cpa
            g['cpa_m'] = float(np.linalg.norm(r + v_bagil * t_cpa))

    # Karsilasma tipi: hedefin gidis yonu ile 'hedeften bize' yonu arasi aci.
    h_hiz = float(np.linalg.norm(v_hedef))
    if h_hiz < 1.0:
        g['tip'] = 'durgun'                          # hedef pratikte duruyor
        return g
    kosinus = float(np.dot(v_hedef / h_hiz, -u_los))
    g['yaklasim_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, kosinus))))
    if g['yaklasim_deg'] < _TIP_KAFA_KAFAYA_DEG:
        g['tip'] = 'kafa_kafaya'
    elif g['yaklasim_deg'] > _TIP_KUYRUK_DEG:
        g['tip'] = 'kuyruk'
    else:
        g['tip'] = 'capraz'
    return g


# ------------------------------------------------------------------- okuyucu

class BboxOkuyucu(threading.Thread):
    """Redis 'tracker_bbox_stab' + 'tracker_bbox' aboneligi (ayri thread).

    Stab kanali gimbal acik oldugunda gelir; ham kanal her tespitte gelir.
    Kapsama ham kanaldan alinir (stab'da yok)."""

    def __init__(self, r):
        super().__init__(daemon=True)
        self.pubsub = r.pubsub(ignore_subscribe_messages=True)
        self.pubsub.subscribe('tracker_bbox_stab', 'tracker_bbox')
        self.lock = threading.Lock()
        self._stab = None           # (sx, sy, w, h, ex, ey, t_capture)
        self._stab_wall = 0.0
        self._kapsama = None
        self._kapsama_wall = 0.0
        # HAM kanal (tracker_bbox): [x, y, w, h, kapsama_%, gecerli, t_capture]
        # x,y SOL UST kosedir. Merkez = x + w/2, y + h/2. Yalniz LOG icin
        # saklanir: "hedef kadrajin neresindeydi" sorusunun ham cevabi, sanal
        # gimbalin duzeltmesinden ONCE.
        self._ham = None
        self._ham_wall = 0.0

    def run(self):
        for mesaj in self.pubsub.listen():
            try:
                veri = json.loads(mesaj['data'])
                kanal = (mesaj['channel'].decode()
                         if isinstance(mesaj['channel'], bytes)
                         else mesaj['channel'])
                simdi = time.monotonic()
                with self.lock:
                    if kanal == 'tracker_bbox_stab':
                        self._stab = veri
                        self._stab_wall = simdi
                    else:
                        # [x, y, w, h, kapsama_%, gecerli, t_capture]
                        if len(veri) >= 6 and veri[5]:
                            self._kapsama = float(veri[4])
                            self._kapsama_wall = simdi
                            self._ham = veri
                            self._ham_wall = simdi
            except Exception:
                continue

    def son(self):
        """(stab_listesi|None, bbox_yas_s, kapsama|None) dondurur."""
        with self.lock:
            stab, wall = self._stab, self._stab_wall
            kapsama = self._kapsama
        yas = (time.monotonic() - wall) if stab is not None else float('inf')
        return stab, yas, kapsama

    def ham(self):
        """(ham_listesi|None, ham_yas_s) -- YALNIZ LOG icin ham bbox kanali."""
        with self.lock:
            veri, wall = self._ham, self._ham_wall
        yas = (time.monotonic() - wall) if veri is not None else float('inf')
        return veri, yas


class MenzilKestirici(threading.Thread):
    """Hedef telemetrisi -> filterwndr IMM -> YALNIZ menzil.

    Kullanici kurali (2026-08-03): hedefin 3D konum verisinden yalniz RANGE
    turetilir. Bu sinifin tek disari acilan olcumu menzil()'dir; imm nesnesine
    disaridan dokunulmaz. Hint/freeze makinesi (simple_guided_follow'daki)
    burada YOKTUR -- menzil icin konum takibi yeterlidir, o makine hiz/donus
    kestirimini inceltir.
    """

    def __init__(self, target_conn_str, home_lat, home_lon, home_alt, hz=10.0):
        super().__init__(daemon=True)
        self.hz = hz
        self._imm = setup_imm_filter(1.0 / hz)
        self._ilk = True
        self._son_stamp = None
        self._son_guncelleme_wall = 0.0
        self.lock = threading.Lock()
        conn = mavutil.mavlink_connection(target_conn_str, source_system=252)
        conn.wait_heartbeat(timeout=30)
        # Hedef akisini iste (konumluyla ayni mesaj, ayni oran).
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            int(1e6 / hz), 0, 0, 0, 0, 0)
        use_rel = bool(getattr(cfg, "TARGET_ALT_USE_RELATIVE", True))
        self._reader = mavlink_utils.MavStateReader(
            conn, "GLOBAL_POSITION_INT",
            lambda msg: mavlink_utils.parse_global_int(
                msg, home_lat, home_lon, home_alt, use_relative_alt=use_rel))
        self._reader.start()

    def run(self):
        periyot = 1.0 / self.hz
        while True:
            time.sleep(periyot)
            pos, _, stamp = self._reader.get_with_stamp()
            if pos is None or stamp <= 0.0:
                continue
            if self._son_stamp is not None and stamp <= self._son_stamp:
                continue
            z = np.asarray(pos, dtype=float).reshape(3)
            with self.lock:
                if self._ilk:
                    for filt in self._imm.filters:
                        filt.x[0:3] = z
                    self._imm.x = self._imm.filters[0].x.copy()
                    self._ilk = False
                else:
                    dt = clamp_filter_dt(stamp - self._son_stamp)
                    predict_imm_over_dt(self._imm, dt)
                    self._imm.update(z)
                self._son_stamp = stamp
                self._son_guncelleme_wall = time.monotonic()

    def menzil(self, kendi_pos_ned, bayat_s=2.0):
        """|hedef_kestirimi - kendi konum| [m]; veri yok/bayatsa None."""
        if kendi_pos_ned is None:
            return None
        with self.lock:
            if self._ilk:
                return None
            if time.monotonic() - self._son_guncelleme_wall > bayat_s:
                return None
            est = np.asarray(self._imm.x[0:3], dtype=float).reshape(3)
        fark = est - np.asarray(kendi_pos_ned, dtype=float).reshape(3)
        return float(np.linalg.norm(fark))

    # ------------------------------------------------------------------
    # LOG-ONLY. KONTROL YOLU BU METODU CAGIRMAZ.
    #
    # Kullanici kurali (2026-08-03): gudum hedefin telemetrisinden yalniz
    # MENZILI kullanabilir -> menzil() metodu. Ama ANALIZ hedefin konumunu,
    # hizini ve ivmesini bilmeden yapilamiyor: 2026-08-04'te kullanici
    # "hedef bana dogru geliyordu, MPC kaciyor gibi" dedi ve bunu dogrulamak
    # icin hedefin gidis yonu ile kerteriz arasindaki aci ELLE hesaplandi --
    # cunku hicbir logda yoktu. Bu metot o bosluğu kapatir; ciktisi yalnizca
    # 'ref_' onekli CSV kolonlarina gider.
    # ------------------------------------------------------------------
    def ref_hedef_durum(self, bayat_s=2.0):
        """LOG ICIN hedef durumu; kontrolcuye VERILMEZ.

        Doner: {'pos': ndarray|None (olculen NED),
                'vel': ndarray|None (olculen NED [m/s]),
                'acc': ndarray|None (IMM kestirimi [m/s^2]),
                'donus_dps': float|None (IMM donus hizi),
                'est_pos': ndarray|None (IMM konum kestirimi)}
        Veri yok/bayatsa hepsi None.
        """
        bos = {'pos': None, 'vel': None, 'acc': None,
               'donus_dps': None, 'est_pos': None}
        try:
            olcum_pos, olcum_vel = self._reader.get()
        except Exception:
            return bos
        with self.lock:
            hazir = not self._ilk
            bayat = (time.monotonic() - self._son_guncelleme_wall) > bayat_s
            x = np.asarray(self._imm.x, dtype=float).copy() if hazir else None
        if olcum_pos is None or not hazir or bayat:
            return bos
        return {
            'pos': np.asarray(olcum_pos, dtype=float).reshape(3),
            'vel': (np.asarray(olcum_vel, dtype=float).reshape(3)
                    if olcum_vel is not None else None),
            # IMM durumu 10D: [x,y,z, vx,vy,vz, ax,ay,az, omega]
            'acc': x[6:9].copy(),
            'donus_dps': math.degrees(float(x[9])),
            'est_pos': x[0:3].copy(),
        }


# ------------------------------------------------------------------ komutcu

class HizKomutcu:
    """Avci baglantisi: durum okuyucu + hiz setpoint gonderici."""

    def __init__(self, pursuer_conn_str):
        self.conn = mavutil.mavlink_connection(pursuer_conn_str,
                                               source_system=251)
        self.conn.wait_heartbeat(timeout=30)
        self.boot = time.monotonic()
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        # HEARTBEAT (2026-08-07): YALNIZ LOG. Otopilotun ucus modu ve
        # baglanti sagligi (heartbeat yasi) hicbir CSV'de yoktu; gercek
        # ucusta "arac GUIDED'dan dustu" ya da "telemetri kesildi"
        # senaryolari kaza sonrasi loglardan ayirt edilemiyordu. Akis
        # istenmez -- otopilot heartbeat'i zaten 1 Hz basar.
        self.okuyucu = mavlink_utils.MavStateReader(
            self.conn,
            ["LOCAL_POSITION_NED", "ATTITUDE", "VIBRATION", "HEARTBEAT"],
            mavlink_utils.parse_local_ned)
        self.okuyucu.start()

    def hiz_gonder(self, vel_ned, yaw_rate_rad=None):
        mask = _MASK_HIZ if yaw_rate_rad is None else _MASK_HIZ_YAWRATE
        self.conn.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self.boot) * 1000.0) & 0xFFFFFFFF,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            0.0, 0.0, 0.0,
            float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2]),
            0.0, 0.0, 0.0,
            0.0, float(yaw_rate_rad or 0.0))


# ---------------------------------------------------------------------- olay

class OlayIzleyici:
    """Surekli sayilardan AYRIK OLAYLAR uretir (log'un 'ne oldu' katmani).

    NICIN: 2644 satirlik bir CSV "hangi sayilar vardi"yi anlatiyor ama "ne
    oldu"yu anlatmiyor. Devir ani, bbox kaybi, kisitlarin devreye girmesi,
    menzil esikleri, en yakin gecis ve iska ayri ayri metin loglarina
    dagilmisti. Burada hepsi TEK yerde, zaman damgali ve makine okunur olur.

    Kullanim: her dongude izle(...) cagrilir, o dongude olusan olaylarin
    listesini doner (cogunlukla bos). Kenar (edge) tetiklidir: bir kisit
    100 dongu acik kalirsa BIR kez 'acik', bir kez 'kapali' olayi uretir.
    """

    MENZIL_ESIKLERI = (100.0, 50.0, 30.0, 20.0, 10.0, 5.0, 3.0)

    def __init__(self):
        self._durum = None            # onceki tespit durumu
        self._bayrak = {}             # kisit adi -> acik mi
        self._gecilen_esik = set()
        self._en_yakin = float('inf')
        self._kapaniyordu = None
        self._iska_bildirildi = False

    def _kenar(self, olaylar, ad, acik, detay=''):
        if bool(acik) != bool(self._bayrak.get(ad, False)):
            self._bayrak[ad] = bool(acik)
            olaylar.append((f"{ad}_{'acik' if acik else 'kapali'}", detay))

    def izle(self, durum, menzil, geo, kelepceler):
        """Doner: [(olay_adi, detay_metni), ...] -- cogunlukla bos liste."""
        olaylar = []

        # 1) Tespit durumu gecisleri: taze <-> tut (kisa bosluk) <-> suz (kayip)
        if durum != self._durum:
            if self._durum is not None:
                # taze=bbox var | tut=kisa bosluk, son komut tutuluyor |
                # suz=uzun kayip, olculen hiza sonumleniyor (coast)
                olaylar.append((f"tespit_{self._durum}_to_{durum}",
                                f"tip={geo.get('tip', '')}"))
            self._durum = durum

        # 2) Kisitlar/kelepceler (kenar tetikli)
        for ad, acik in kelepceler.items():
            self._kenar(olaylar, ad, acik, f"menzil={_ms(menzil)}")

        # 3) Menzil esikleri: ilk gecis bir kez bildirilir
        if menzil is not None:
            for esik in self.MENZIL_ESIKLERI:
                if menzil <= esik and esik not in self._gecilen_esik:
                    self._gecilen_esik.add(esik)
                    olaylar.append((f"menzil_{esik:.0f}m",
                                    f"tip={geo.get('tip', '')} "
                                    f"kapanma={_ms(geo.get('kapanma_mps'))}"))
            self._en_yakin = min(self._en_yakin, menzil)

        # 4) En yakin gecis: kapanma isaret degistirdiginde (kapaniyor->aciliyor)
        kap = geo.get('kapanma_mps')
        if kap is not None:
            simdi_kapaniyor = kap > 0.0
            if (self._kapaniyordu is True and not simdi_kapaniyor
                    and menzil is not None and menzil < 200.0):
                olaylar.append(('en_yakin_gecis',
                                f"menzil={_ms(menzil)} tip={geo.get('tip','')} "
                                f"yaklasim_deg={_ms(geo.get('yaklasim_deg'))}"))
                self._iska_bildirildi = False
            self._kapaniyordu = simdi_kapaniyor

        # 5) Iska: en yakin gecisten sonra menzil iki katina cikti
        if (menzil is not None and math.isfinite(self._en_yakin)
                and not self._iska_bildirildi
                and self._en_yakin > 3.0
                and menzil > max(2.0 * self._en_yakin, self._en_yakin + 15.0)):
            self._iska_bildirildi = True
            olaylar.append(('iska', f"en_yakin={self._en_yakin:.1f}m "
                                    f"simdi={menzil:.1f}m"))
        return olaylar


def _ms(v, bicim='{:.1f}'):
    """None-guvenli kisa sayi bicimleyici (olay detay metinleri icin)."""
    return '-' if v is None else bicim.format(v)


def _stab_ops(stab, i, taze):
    """tracker_bbox_stab'in OPSIYONEL i. alanini float'a cevir ya da None.

    Yayinci surumune gore payload kisa olabilir (eski bbox_to_redis ya da
    YILDIZ_MINRECT kapali) -- o zaman alan yoktur. tilt_deg icin elle
    yazilmis kalibin ortak hali."""
    if not taze or stab is None or len(stab) <= i or stab[i] is None:
        return None
    try:
        return float(stab[i])
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------- dongu

# CSV kolon duzeni. TEK KAYNAK: hem baslik satiri hem LOG_SOZLUGU.md bunu
# refere eder. Yeni kolon EKLERKEN SONA ekle (eski araclar DictReader
# kullaniyor, sira degil ad onemli; yine de sona eklemek diff'i okunur tutar).
LOG_KOLONLARI = [
    # --- zaman ---
    't', 't_mono', 't_unix', 'dt', 'yetki', 'durum',
    # --- goruntu / image plane ---
    'ex_deg', 'ey_deg', 'bbox_w', 'bbox_h', 'alan_kok', 'alan_px2',
    'kapsama_pct', 'bbox_yas_s', 't_capture',
    'px_sanal_x', 'px_sanal_y', 'px_ham_cx', 'px_ham_cy',
    'kadraj_kenar_px', 'kadraj_kenar_deg', 'ham_yas_s', 'tilt_deg',
    # --- gudumun kullandigi menzil (IZINLI tek hedef buyuklugu) ---
    'menzil_m',
    # --- komut ---
    'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_hiz_mps', 'cmd_yaw_rate_dps',
    'kelepce_hiz', 'kelepce_irtifa', 'kelepce_yaw_slew',
    # --- kendi durumumuz ---
    'pos_x', 'pos_y', 'pos_z', 'irtifa_m',
    'vel_x', 'vel_y', 'vel_z', 'hiz_mps',
    'acc_x_mps2', 'acc_y_mps2', 'acc_z_mps2',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'rota_deg', 'vibe_max',
    # --- HEDEF DURUMU: YALNIZ ANALIZ (gudume GIRMEZ, 'ref_' oneki bunun
    #     gorunur isaretidir) ---
    'ref_hedef_x', 'ref_hedef_y', 'ref_hedef_z',
    'ref_hedef_vx', 'ref_hedef_vy', 'ref_hedef_vz',
    'ref_hedef_hiz_mps', 'ref_hedef_rota_deg',
    'ref_hedef_ax_mps2', 'ref_hedef_ay_mps2', 'ref_hedef_az_mps2',
    'ref_hedef_donus_dps',
    # --- KARSILASMA GEOMETRISI: yalniz analiz ---
    'ref_menzil_gercek_m', 'ref_kerteriz_deg', 'ref_yukselis_deg',
    'ref_yaklasim_acisi_deg', 'ref_karsilasma_tipi',
    'ref_kapanma_hizi_mps', 'ref_tgo_s', 'ref_cpa_m', 'ref_cpa_s',
    # --- olaylar (bos degilse '|' ile ayrilmis olay adlari) ---
    'olay',
    # --- SISTEM SAGLIGI (2026-08-07, gercek ucus loglamasi) -------------
    # Hepsi LOG-ONLY; hicbiri kontrol kararina girmez. Kaza sonrasi ilk
    # sorulan uc soruyu cevaplarlar: dongu gercek zamanda miydi, surec
    # hayatta miydi, otopilot bizi dinliyor muydu.
    'dongu_hz_ort', 'dt_asim', 'hayatta_ttl', 'ap_mod', 'hb_yas_s',
    # --- DONUK DIKDORTGEN (SONA eklendi, indis-koruma) ------------------
    # bbox_to_redis YILDIZ_MINRECT=1 iken dolar, aksi halde BOS. rot_aci_deg
    # ekran-CCW eksen acisi [-90,+90); hedef bank ADAYI bundan + roll_deg'den
    # turetilir: bank ~ katla(roll_deg - rot_aci_deg). DIKKAT: bu bagintinin
    # OFFLINE olculen gucu zayif (bkz. bbox_to_redis'teki not) -- kolon
    # ANALIZ icin var, kontrol kararina baglanmadan once dogrulanmali.
    'rot_w_px', 'rot_h_px', 'rot_aci_deg',
]


class GoruntuluDongu:
    """Yetkiyi bekler, devri tohumlar, kontrolcuyu OLCULEN dt ile kosturur.

    Komut yolu uzerinde iki ortak koruma vardir (yontemden bagimsiz):
      * hiz kelepcesi: |v| <= GORUNTULU_MAX_SPEED_MPS (varsayilan 18,
        WPNAV_SPEED tavaniyla ayni).
      * komut LPF'si: tau_s birinci mertebe. Devir aninda LPF durumu
        konumlunun son komutuyla TOHUMLANIR -> gecis sicramasiz.
    bbox kaybolursa (yas > bbox_bayat_s) komut son degerinden sifira dogru
    sonumlenir; karar verici zaten dwell sonrasi 'konumlu'ya geri doner.
    """

    def __init__(self, kontrolcu, loop_hz=20.0, tau_s=0.35, bbox_bayat_s=0.7,
                 bosluk_tut_s=1.0, irtifa_taban_m=15.0,
                 yaw_tau_s=0.15, yaw_slew_dps2=120.0,
                 log_yolu=None, pursuer=None, target=None):
        self.k = kontrolcu
        self.loop_dt = 1.0 / float(loop_hz)
        self.tau = float(tau_s)
        self.bbox_bayat_s = float(bbox_bayat_s)
        self.bosluk_tut_s = float(bosluk_tut_s)
        self.irtifa_taban_m = float(irtifa_taban_m)
        self.yaw_tau_s = float(yaw_tau_s)
        self.yaw_slew_dps2 = float(yaw_slew_dps2)
        self.hiz_tavani = float(getattr(cfg, "GORUNTULU_MAX_SPEED_MPS", 18.0))
        # DIKKAT: konumlunun portlari (14652/14603) DEGIL -- iki surec ayni
        # udpin portunu baglayamaz. Goruntuluye ayrilmis cikislar kullanilir.
        self.pursuer = pursuer or getattr(cfg, "GORUNTULU_PURSUER_CONN_STR",
                                          "udpin:127.0.0.1:14654")
        self.target = target or getattr(cfg, "GORUNTULU_TARGET_CONN_STR",
                                        "udpin:127.0.0.1:14604")
        damga = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_yolu = log_yolu or str(
            Path(__file__).resolve().parent / 'logs'
            / f"goruntulu_{self.k.ad}_{damga}.csv")
        self._calisiyor = True
        self.r = None                 # _baglan() doldurur
        self._ttl_sayac = 0           # olu-adam TTL ornekleme sayaci
        self._son_ttl = -1            # son ornekle(nen) TTL [s]

    # -- kurulum ----------------------------------------------------------

    def _baglan(self):
        print(f"[goruntulu:{self.k.ad}] Redis'e baglaniliyor...")
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.r.ping()
        self.bbox = BboxOkuyucu(self.r)
        self.bbox.start()
        print(f"[goruntulu:{self.k.ad}] avci baglantisi: {self.pursuer}")
        self.komutcu = HizKomutcu(self.pursuer)
        # Home'u avcidan al (hedef global->NED donusumu icin; konumluyla ayni).
        # Yigin hizli yeniden baslatildiginda ilk istek yarisa girip
        # cevapsiz kalabiliyor (2026-08-04'te iki kosu boyle oldu); tekrar
        # dene, tek denemede pes etme.
        msg = None
        for deneme in range(1, 4):
            self.komutcu.conn.mav.command_long_send(
                self.komutcu.conn.target_system,
                self.komutcu.conn.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
            msg = self.komutcu.conn.recv_match(type="HOME_POSITION",
                                               blocking=True, timeout=10.0)
            if msg is not None:
                break
            print(f"[goruntulu:{self.k.ad}] HOME_POSITION cevapsiz "
                  f"(deneme {deneme}/3)")
        if msg is None:
            raise SystemExit("HOME_POSITION alinamadi (3 deneme)")
        home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
        print(f"[goruntulu:{self.k.ad}] home={home[0]:.7f},{home[1]:.7f} "
              f"alt={home[2]:.1f}  hedef: {self.target}")
        self.menzilci = MenzilKestirici(self.target, *home)
        self.menzilci.start()

    def _devir_oku(self):
        try:
            ham = self.r.get('devir_durumu')
            return json.loads(ham) if ham else None
        except Exception:
            return None

    def _yetki(self):
        try:
            v = self.r.get('komut_yetkisi')
            return v.decode('utf-8', 'replace') if v else 'konumlu'
        except Exception:
            return 'konumlu'

    def _gecis_sebep(self):
        """Devri hangi kural tetikledi? bbox_to_redis._update_decision yazar
        ('basit(5 ardisik kare)' / 'eski(38/45 kare, ...)'). Anahtar yoksa
        (eski bbox_to_redis surumu) bos doner -- olay detayi eskisi gibi kalir.
        """
        try:
            v = self.r.get('gecis_sebep')
            return v.decode('utf-8', 'replace') if v else ''
        except Exception:
            return ''

    def _hayatta_bildir(self):
        """OLU-ADAM ANAHTARI: 'ben buradayim ve komut verebilirim'.

        NEDEN (2026-08-05 arizasi): bbox_to_redis yetkiyi 'goruntulu'ya
        cevirirken goruntulu kontrolcunun CALISIP CALISMADIGINI hic
        sormuyordu. Kontrolcu baslatilmadan (ya da _baglan()'da asili
        kalmisken) yetki devredilince konumlu setpoint gondermeyi kesiyor
        ve aracI KIMSE komutlamiyor: 5.9 s'lik bir pencerede menzil 40 m'den
        124 m'ye acildi, yaw cirpintisi 6.6x arti (olculdu, bkz.
        guided_follow_20260805_145608.csv). Belirti "MPC titriyor ve hedefi
        hic takip etmiyor" gibi gorunuyor ama MPC hic KOSMUYORDU.

        Anahtar SURESIZ degil: TTL ile yaziliyor, yani surec olurse anahtar
        kendiliginden kaybolur ve karar verici goruntuluye GECMEZ / geri
        doner. Redis yoksa sessizce atlanir (eski davranis)."""
        if self.r is None:
            return
        try:
            self.r.set('goruntulu_hayatta',
                       json.dumps({'ad': self.k.ad, 't_mono': time.monotonic()}),
                       ex=HAYATTA_TTL_S)
        except Exception:
            pass

    def _hayatta_ttl(self):
        """LOG-ONLY: 'goruntulu_hayatta' anahtarinin KALAN TTL'i [s].

        NICIN LOGLANIR: olu-adam anahtari sessiz calisir -- surec asili
        kalirsa anahtar duser, karar verici yetkiyi geri alir ve logda
        yalnizca "satirlar kesildi" gorunur. Bu kolon anahtarin kosu
        boyunca gercekten tazelendigini KANITLAR: normal kosuda hep 2
        (HAYATTA_TTL_S), 1'e dusuyorsa dongu ~1 s'den uzun takiliyor
        demektir, -1 anahtar hic yok / Redis cevapsiz.

        MALIYET: her dongu ayri bir Redis gidis-donusu pahali (20 Hz'de
        dongunun olcusuz bir dilimi). 10 dongude bir ORNEKLENIR, arada
        son deger tekrarlanir -- kolon yine de her satirda dolu olur.
        """
        self._ttl_sayac += 1
        if self._ttl_sayac % 10 == 1:
            try:
                v = self.r.ttl('goruntulu_hayatta')
                # redis-py: -2 anahtar yok, -1 TTL'siz. Ikisi de "saglikli
                # degil" demek; tek bir -1'de birlestirilir.
                self._son_ttl = int(v) if v is not None and v >= 0 else -1
            except Exception:
                self._son_ttl = -1
        return self._son_ttl

    # -- ana dongu --------------------------------------------------------

    def calistir(self, sure_s=None):
        def _sinyal(signum, frame):
            self._calisiyor = False
        signal.signal(signal.SIGINT, _sinyal)
        signal.signal(signal.SIGTERM, _sinyal)

        self._baglan()
        os.makedirs(os.path.dirname(self.log_yolu), exist_ok=True)
        log_f = open(self.log_yolu, 'w', newline='')
        log = csv.writer(log_f)
        log.writerow(LOG_KOLONLARI)
        # OLAY LOGU: ayri, seyrek, insan okunur. Ana CSV'nin 'olay' kolonu
        # ayni bilgiyi tasir ama burada detay metniyle birlikte durur.
        olay_yolu = self.log_yolu.replace('.csv', '_olay.csv')
        if olay_yolu == self.log_yolu:
            olay_yolu = self.log_yolu + '.olay.csv'
        olay_f = open(olay_yolu, 'w', newline='')
        olay_w = csv.writer(olay_f)
        olay_w.writerow(['t', 't_unix', 'olay', 'menzil_m', 'detay'])
        izleyici = OlayIzleyici()
        turev = VelocityDifferentiator(max_history=5)
        acc_lpf = np.zeros(3)       # ivme turevinin gurultusunu kesen LPF
        ACC_TAU = 0.20              # [s] -- turev 20 Hz'de gurultulu
        print(f"[goruntulu:{self.k.ad}] log: {self.log_yolu}")
        print(f"[goruntulu:{self.k.ad}] olay logu: {olay_yolu}")
        print(f"[goruntulu:{self.k.ad}] yetki bekleniyor "
              f"(Redis 'komut_yetkisi' == 'goruntulu')...")

        def _olay_yaz(t_m, t_u, olaylar, menzil):
            for ad, detay in olaylar:
                olay_w.writerow([f"{t_m:.4f}", f"{t_u:.3f}", ad,
                                 '' if menzil is None else f"{menzil:.2f}",
                                 detay])
            if olaylar:
                olay_f.flush()      # olaylar seyrek: aninda diske yaz

        _basliksiz_uyari = [False]  # kolon sayisi uyarisi bir kez basilsin
        baslangic = time.monotonic()
        aktif = False
        lpf_vel = np.zeros(3)
        onceki_t = None
        son_istek = None            # kisa bosluk tutucusu (bosluk_tut_s)
        # KISA BOSLUKTA YAW SURDURME (bkz. TUT_YAW_TAU_S ustundeki not).
        # Bayrak KAPALIYKEN bu iki degisken yazilir ama HIC okunmaz.
        tut_yaw_acik = _cevre_bayrak('YILDIZ_TUT_YAW', 1.0) > 0.0
        son_yaw_cmd = None          # son GERCEKTEN komutlanan yaw_rate
        son_yaw_t = None            # o komutun monotonic zamani
        lpf_yaw = 0.0               # yaw komutu duzgunlestirme durumu
        birak_bekliyor = False      # ISKA sonrasi sahte-devir kilidi (asagida)
        # --- GERCEK-ZAMAN SAGLIGI (2026-08-07) -- yalniz olcum/uyari,
        # kontrol yoluna DOKUNMAZ. Desen simple_guided_follow.py'den
        # kopyalandi (orada 2 Hz'e dusen dongu + sabit dt varsayimi dev
        # dairelerin ve titremenin kok nedeniydi; goruntulu iskelette ayni
        # olcum HIC yoktu). Pencere ~2 s: 20 Hz'de 40 ornek.
        dt_pencere = deque(maxlen=max(5, int(round(2.0 / self.loop_dt))))
        yavas_streak = 0            # ardisik "dt buyuk" dongu sayisi
        son_yavas_uyari = 0.0       # uyari spam kelepcesi [monotonic]
        onceki_ap_mod = None        # otopilot mod degisimi kenar tetigi
        satir_sayaci = 0            # ana CSV periyodik flush sayaci
        while self._calisiyor:
            simdi = time.monotonic()
            if sure_s is not None and simdi - baslangic > sure_s:
                break

            # OLU-ADAM: yetkiyi ISTEMEDEN ONCE hayatta oldugumuzu bildir.
            # Yetki beklerken de yazilir -- karar verici ancak boylece
            # "devredebilecegim biri var" diyebilir.
            self._hayatta_bildir()
            yetkili = (self._yetki() == 'goruntulu')
            if not aktif:
                if birak_bekliyor:
                    # ISKA'dan hemen sonrayiz: 'goruntulu_birak' yazildi ama
                    # bbox_to_redis 'komut_yetkisi'ni 'goruntulu' -> 'konumlu'
                    # yapana kadar 1-37 ms araya giriyor (olculdu). O arada
                    # yetki hala 'goruntulu' gorunur ve aktif=False oldugundan
                    # asagidaki DEVIR ANI bloğu SAHTE bir yeniden devralma
                    # yapiyordu (sim kosusunda 9 gercek devire karsi 17
                    # devir_alindi olayi). Yetkinin GERCEKTEN goruntulu
                    # disina ciktigini gorene kadar devralma; bu bayrak bir
                    # sonraki GERCEK devri (konumlu -> goruntulu) engellemez,
                    # cunku yetkili False olur olmaz temizlenir.
                    if yetkili:
                        time.sleep(0.02)
                        continue
                    birak_bekliyor = False
                if not yetkili:
                    time.sleep(0.1)
                    continue
                # --- DEVIR ANI ---
                devir = self._devir_oku()
                self.k.tohumla(devir)
                if devir and 'cmd_vel_ned' in devir:
                    lpf_vel = np.asarray(devir['cmd_vel_ned'], dtype=float)
                else:
                    _, v = self.komutcu.okuyucu.get()
                    lpf_vel = (np.asarray(v, dtype=float)
                               if v is not None else np.zeros(3))
                onceki_t = None
                son_istek = None      # onceki angajmanin komutu tasinmasin
                son_yaw_cmd = None    # ayni gerekce: yaw da tasinmasin
                son_yaw_t = None
                lpf_yaw = 0.0
                aktif = True
                p_pos, _ = self.komutcu.okuyucu.get()
                sebep = self._gecis_sebep()
                _olay_yaz(simdi, time.time(), [(
                    'devir_alindi',
                    f"tohum_hiz={np.round(lpf_vel, 2).tolist()} "
                    f"devir_durumu={'var' if devir else 'YOK'}"
                    + (f" kural={sebep}" if sebep else ''))],
                    self.menzilci.menzil(p_pos))
                print(f"[goruntulu:{self.k.ad}] >>> YETKI DEVRALINDI, "
                      f"tohum hiz={np.round(lpf_vel, 2).tolist()}"
                      f"{' (devir_durumu yok, kendi hizimiz)' if not devir else ''}"
                      f"{(' kural=' + sebep) if sebep else ''}")
                continue

            if not yetkili:
                aktif = False
                p_pos, _ = self.komutcu.okuyucu.get()
                _olay_yaz(simdi, time.time(),
                          [('yetki_konumluya_dondu', 'komut kesildi')],
                          self.menzilci.menzil(p_pos))
                print(f"[goruntulu:{self.k.ad}] yetki 'konumlu'ya dondu; "
                      f"komut kesildi, tekrar bekleniyor")
                continue

            # --- olcum paketi ---
            # ham_dt: KELEPCESIZ duvar saati adimi. Kontrolcuye verilen dt
            # asagida 0.5 s'de tavanlanir (uzun bir takilma entegratorleri
            # firlatmasin) -- ama saglik olcumu tam da o tavani ASAN
            # gecikmeleri gormek icin var, o yuzden ham degeri kullanir.
            ham_dt = (self.loop_dt if onceki_t is None
                      else max(simdi - onceki_t, 1e-6))
            dt = (self.loop_dt if onceki_t is None
                  else min(max(simdi - onceki_t, 0.5 * self.loop_dt), 0.5))
            onceki_t = simdi
            dt_pencere.append(ham_dt)
            dt_asim = 1 if ham_dt > 1.5 * self.loop_dt else 0
            dongu_hz_ort = (len(dt_pencere)
                            / max(float(sum(dt_pencere)), 1e-6))
            # YAVAS DONGU UYARISI (streak mantigi, spam yok): tek bir
            # gecikme (GC, disk) normaldir; 10 ARDISIK dongu gecikmesi
            # sistemin bogulmasidir ve o kosunun sonuclari supheli olur.
            # stderr'e basilir -- stdout gudum akisidir, uyari orada
            # kaybolmasin.
            yavas_streak = yavas_streak + 1 if dt_asim else 0
            if yavas_streak >= 10 and simdi - son_yavas_uyari > 5.0:
                son_yavas_uyari = simdi
                print(f"[goruntulu:{self.k.ad}] *** DONGU YAVAS: olculen "
                      f"{dongu_hz_ort:.2f} Hz (istenen {1.0/self.loop_dt:.1f} "
                      f"Hz, dt {1000.0*float(np.mean(dt_pencere)):.0f} ms) -- "
                      f"{yavas_streak} ardisik dongu. CPU yukunu azaltin ***",
                      file=sys.stderr, flush=True)
            t_unix = time.time()          # ortak MUTLAK zaman (log hizalamasi)
            stab, bbox_yas, kapsama = self.bbox.son()
            ham_bbox, ham_yas = self.bbox.ham()
            pos, vel = self.komutcu.okuyucu.get()
            att = self.komutcu.okuyucu.get_attitude()
            vibe = self.komutcu.okuyucu.get_vibration()
            menzil = self.menzilci.menzil(pos)
            taze = stab is not None and bbox_yas <= self.bbox_bayat_s
            # KENDI ivmemiz: LOCAL_POSITION_NED hizinin sayisal turevi + LPF.
            # (Hedefin ivmesi asagida IMM'den gelir; ikisi de yalniz LOG.)
            if vel is not None:
                turev.update(simdi, float(vel[0]), float(vel[1]), float(vel[2]))
                ham_acc = np.asarray(turev.get_acceleration('backwards'), float)
                aa = dt / (dt + ACC_TAU) if ACC_TAU > 1e-6 else 1.0
                acc_lpf = acc_lpf + aa * (ham_acc - acc_lpf)
            o = Olcum(
                t=simdi, dt=dt,
                ex_deg=float(stab[4]) if taze else None,
                ey_deg=float(stab[5]) if taze else None,
                bbox_w=float(stab[2]) if taze else None,
                bbox_h=float(stab[3]) if taze else None,
                alan_kok=(math.sqrt(float(stab[2]) * float(stab[3]))
                          if taze else None),
                kapsama_pct=kapsama if taze else None,
                bbox_yas_s=bbox_yas,
                t_capture=(float(stab[6]) if taze and len(stab) > 6 else None),
                tilt_deg=(float(stab[7]) if taze and len(stab) > 7
                          and stab[7] is not None else None),
                menzil_m=menzil,
                pos_ned=np.asarray(pos, dtype=float) if pos is not None else None,
                vel_ned=np.asarray(vel, dtype=float) if vel is not None else None,
                yaw_rad=att[2] if att is not None else None,
                roll_rad=att[0] if att is not None else None,
                pitch_rad=att[1] if att is not None else None,
                px_sanal_x=float(stab[0]) if taze else None,
                px_sanal_y=float(stab[1]) if taze else None,
                px_ham_cx=(float(ham_bbox[0]) + float(ham_bbox[2]) / 2.0
                           if ham_bbox is not None and ham_yas <= self.bbox_bayat_s
                           else None),
                px_ham_cy=(float(ham_bbox[1]) + float(ham_bbox[3]) / 2.0
                           if ham_bbox is not None and ham_yas <= self.bbox_bayat_s
                           else None),
                acc_ned=acc_lpf.copy() if vel is not None else None,
                # DONUK DIKDORTGEN passthrough (tilt_deg ile ayni desen):
                # alanlar YILDIZ_MINRECT=1 iken payload'a EKLENIR; yoksa
                # len(stab) <= 8 olur ve hepsi None kalir.
                rot_w_px=_stab_ops(stab, 8, taze),
                rot_h_px=_stab_ops(stab, 9, taze),
                rot_aci_deg=_stab_ops(stab, 10, taze),
                vibe_max=None if vibe is None else float(max(vibe)),
            )

            # --- kontrolcu ---
            yaw_rate_dps = None
            durum = 'taze'          # LOG: hangi kolda gittigimiz (asagida set)
            k_olaylari = []         # kontrolcunun ilan ettigi ayrik olaylar
            if taze:
                cmd = self.k.komut(o)
                if getattr(cmd, 'olay', ''):
                    k_olaylari.append((cmd.olay,
                                       getattr(cmd, 'olay_detay', '')))
                    print(f"[goruntulu:{self.k.ad}] OLAY: {cmd.olay} "
                          f"{getattr(cmd, 'olay_detay', '')}")
                if getattr(cmd, 'birak', False):
                    # ISKA: kontrolcu (mpc_gudum'daki durum makinesi) yetkiyi
                    # kendi istegiyle birakiyor. 'goruntulu_birak' anahtari
                    # bbox_to_redis._make_decision tarafindan okunur (bkz.
                    # oradaki 'manuel_durdur' bloğunun yanı). 'komut_yetkisi'ne
                    # DOGRUDAN yazilmaz: bbox_to_redis onu her karede (~33 ms)
                    # kendi moduyla eziyor, yani yazdigimiz deger aninda
                    # silinirdi. 'manuel_durdur' de kullanilmaz: o bir OPERATOR
                    # kill-switch'i, mandallidir (temizlenmezse bir daha
                    # 'goruntulu'ya donulmez) ve paylasimlidir.
                    self.r.set('goruntulu_birak', json.dumps({
                        't_mono': simdi, 'sebep': getattr(cmd, 'birak_sebep', ''),
                        'yontem': self.k.ad}))
                    self.komutcu.hiz_gonder(np.asarray(cmd.vel_ned, float), None)  # SUZULME
                    aktif = False
                    birak_bekliyor = True   # bkz. DEVIR ANI oncesindeki guard
                    _olay_yaz(simdi, time.time(),
                              [('ISKA:birak', getattr(cmd, 'birak_sebep', ''))],
                              menzil)
                    print(f"[goruntulu:{self.k.ad}] <<< ISKA: yetki birakiliyor, "
                          f"sebep={getattr(cmd, 'birak_sebep', '')!r}")
                    continue
                istek = np.asarray(cmd.vel_ned, dtype=float).reshape(3)
                yaw_rate_dps = cmd.yaw_rate_dps
                son_istek = istek.copy()
                if yaw_rate_dps is not None:
                    son_yaw_cmd = float(yaw_rate_dps)
                    son_yaw_t = simdi
            elif son_istek is not None and bbox_yas <= (self.bbox_bayat_s
                                                        + self.bosluk_tut_s):
                # KISA BOSLUK: son gecerli komutu TUT (yaw_rate haric).
                # Eski davranis (aninda sifira sonumleme) los_elips kosusunda
                # olculen kisir donguyu kuruyordu: fren -> burun yukari pitch
                # -> sabit kamera yukari bakar -> zaten alt kenardaki hedef
                # busbutun cikar -> kayip kalicilasiir. Referans davranis
                # 2LOSKF2._apply_last_command (1 s grace). yaw_rate
                # VARSAYILAN OLARAK tutulmaz: kor donus hedefi yatayda da
                # kaybettirir. YILDIZ_TUT_YAW=1 iken son yaw komutu
                # SONUMLENEREK ve SURE SINIRLI olarak surdurulur -- gerekce
                # ve olcum icin bkz. TUT_YAW_TAU_S ustundeki blok.
                istek = son_istek
                durum = 'tut'
                if (tut_yaw_acik and son_yaw_cmd is not None
                        and son_yaw_t is not None):
                    gecen = simdi - son_yaw_t
                    if gecen <= TUT_YAW_AZAMI_S:
                        # Kare-kare carpim yerine GECEN SUREDEN hesap:
                        # dt dalgalansa (dongu yavaslasa) bile sonumleme
                        # ayni fiziksel egriyi izler.
                        yaw_rate_dps = son_yaw_cmd * math.exp(
                            -gecen / TUT_YAW_TAU_S)
                # 'durum' BILINCLI OLARAK 'tut' kalir: taban kosularinin
                # faz istatistikleri (taze/tut/suz %) kiyaslanabilir
                # olsun. Surdurmenin kac karede calistigi zaten
                # "durum=='tut' VE cmd_yaw_rate_dps bos degil"
                # kosulundan tam olarak okunur.
            else:
                # UZUN kayip: SIFIR DEGIL, SUZULME (coast).
                #
                # ESKI DAVRANIS VE NICIN YANLISTI (2026-08-04, kullanicinin
                # videoda gordugu devir sarsintisinin koku): burada sifir
                # yaziliyordu. Ama sifir "komut vermemek" DEGIL, 18 m/s'den
                # tam duruşa FREN komutudur. Olculdu (mpc_20260804_160604):
                # devirden 1.2 s sonra hedef kayboluyor, 1 s tutucu bitiyor
                # ve komut 18.0 -> 13.8 -> 8.1 -> 4.7 -> 2.8 -> 0.19 m/s'ye
                # cokuyor. Karar verici dwell'i (2 s) dolmadigi icin yetki
                # hala bizde; yani aracin "dur" dedigimiz bir bosluk var.
                # Ustelik bu kendini besliyor: fren -> burun yukari pitch ->
                # sabit kamera yukari bakar -> zaten kenardaki hedef busbutun
                # cikar -> kayip kalicilasir (ayni zinciri LOS ajani da
                # bagimsiz olarak bulmustu).
                #
                # DOGRUSU: ivme komutu SIFIR olsun, hiz komutu degil. Olculen
                # hiza sonumlemek fiziksel olarak "duz uc, hicbir sey yapma"
                # demektir; geometri bozulmaz ve konumlu yetkiyi devralana
                # kadar arac kararli kalir.
                if o.vel_ned is not None:
                    istek = o.vel_ned.copy()
                elif son_istek is not None:
                    istek = son_istek
                else:
                    istek = np.zeros(3)
                durum = 'suz'

            # ortak korumalar: LPF + hiz kelepcesi
            a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
            lpf_vel = lpf_vel + a * (istek - lpf_vel)
            n = float(np.linalg.norm(lpf_vel))
            kelepce_hiz = n > self.hiz_tavani          # LOG: tavana degdi mi
            v_cmd = lpf_vel * (self.hiz_tavani / n) if kelepce_hiz else lpf_vel

            # MUTLAK IRTIFA TABANI (2026-08-04 cakilma dersi): uc rotada da
            # kontrolcu katmanindaki bir hata sureklI alcalma komutlayip
            # kopteri YERE cakti (hedef 128+ m uzaktayken). Yontemden bagimsiz
            # son savunma: tabanin altinda ALCALMA komutu iletilmez. NED'de
            # pos[2] asagi-pozitif degil, asagi buyuyen negatif yukseklik:
            # z = -irtifa; taban = z > -irtifa_taban_m. Kelepce yalniz komutun
            # dikey bilesenini keser, LPF durumuna dokunmaz.
            kelepce_irtifa = (pos is not None
                              and float(pos[2]) > -self.irtifa_taban_m
                              and v_cmd[2] > 0.0)
            if kelepce_irtifa:
                v_cmd = v_cmd.copy()
                v_cmd[2] = 0.0

            # YAW KOMUTU DUZGUNLESTIRME (2026-08-04 titresim analizi): yaw_rate
            # buraya kadar HAM geliyordu -- LPF yalniz hiz kanallarina
            # uygulaniyordu. Olculdu: MPC'nin sert FOV kisiti acikken yaw
            # komutu 4 Hz'de +-16 dps cirpiniyor (adim farki rms 4.3-6.2 dps,
            # LOS 0.6-0.8 / PID 1.3), artimlarin lag-1 otokorelasyonu ~0 yani
            # beyaz gurultuyle surulen rastgele yuruyus. Hiz kanallari ayni
            # kosuda temiz (0.42-0.59) cunku onlar LPF'den geciyordu.
            # Iki kademeli koruma: once ivme (slew) kelepcesi -- yalniz sicrama
            # yapan adimlari keser, mesru hizli donuse dokunmaz; sonra hafif
            # LPF. Ikisi de yontemden BAGIMSIZ, komut yolunun ortak hijyeni.
            kelepce_yaw = False
            if yaw_rate_dps is not None:
                sinir = self.yaw_slew_dps2 * dt
                kirpik = float(np.clip(yaw_rate_dps,
                                       lpf_yaw - sinir, lpf_yaw + sinir))
                kelepce_yaw = abs(kirpik - yaw_rate_dps) > 1e-6   # LOG
                yaw_rate_dps = kirpik
                ay = dt / (dt + self.yaw_tau_s) if self.yaw_tau_s > 1e-6 else 1.0
                lpf_yaw += ay * (yaw_rate_dps - lpf_yaw)
                yaw_cmd_dps = lpf_yaw
            else:
                lpf_yaw = 0.0          # yaw otopilotta: durumu tazele
                yaw_cmd_dps = None

            self.komutcu.hiz_gonder(
                v_cmd,
                None if yaw_cmd_dps is None else math.radians(yaw_cmd_dps))

            # ================= LOG (kontrol yolu BITTI, komut gonderildi) ====
            # Buradan asagisi ARACIN DAVRANISINI ETKILEMEZ. Ozellikle
            # ref_hedef_durum() cagrisi burada, komuttan SONRA yapilir ki
            # "hedef telemetrisi gudume sizdi mi" sorusunun cevabi kodun
            # sirasindan bile okunabilsin: sizamaz, komut coktan gitti.
            ref = self.menzilci.ref_hedef_durum()
            geo = _karsilasma_geometrisi(pos, vel, ref['pos'], ref['vel'])
            h_vel = ref['vel']
            h_hiz = None if h_vel is None else float(np.linalg.norm(h_vel))
            h_rota = (None if h_vel is None or h_hiz < 0.5 else
                      math.degrees(math.atan2(float(h_vel[1]),
                                              float(h_vel[0]))) % 360.0)
            biz_hiz = None if vel is None else float(np.linalg.norm(vel))
            biz_rota = (None if vel is None or biz_hiz < 0.5 else
                        math.degrees(math.atan2(float(vel[1]),
                                                float(vel[0]))) % 360.0)
            # Kadraj kenarina uzaklik: HAM merkez kullanilir -- hedefin fiziksel
            # olarak kadrajdan cikmasina ne kadar kaldigini o soyler (sanal
            # piksel kadraj disina tasabilir, o bir hesap sonucudur).
            kenar_px = kenar_deg = None
            if o.px_ham_cx is not None:
                kenar_px = min(o.px_ham_cx, _KADRAJ_W - o.px_ham_cx,
                               o.px_ham_cy, _KADRAJ_H - o.px_ham_cy)
                kenar_deg = math.degrees(math.atan(kenar_px / _KADRAJ_FX))

            # --- SISTEM SAGLIGI (LOG-ONLY, komut coktan gonderildi) ---
            # Otopilot modu ve heartbeat yasi: "arac bizim setpointlerimizi
            # DINLIYOR muydu" sorusunun tek kanitidir. GUIDED disi bir mod
            # (LAND/RTL/STABILIZE/failsafe) gorunuyorsa gudum kolonlarinin
            # tamami yaniltir -- komut gidiyor ama araca gecmiyordur.
            hb = self.komutcu.okuyucu.get_heartbeat()
            ap_mod = '' if hb is None else (hb[0] or str(hb[1]))
            hb_yas_s = None if hb is None else hb[2]
            hayatta_ttl = self._hayatta_ttl()

            kelepceler = {'kelepce_hiz': kelepce_hiz,
                          'kelepce_irtifa': kelepce_irtifa,
                          'kelepce_yaw_slew': kelepce_yaw}
            olaylar = izleyici.izle(durum, menzil, geo, kelepceler)
            olaylar = k_olaylari + olaylar      # kontrolcu olaylari da yazilir
            # MOD DEGISIMI: kenar tetikli, olay logunun en degerli satiri.
            # Ilk gorulen mod da yazilir (onceki None) -- devir aninda
            # aracin hangi modda oldugunu sabitler.
            if ap_mod and ap_mod != onceki_ap_mod:
                olaylar = olaylar + [('ap_mod_degisti',
                                      f"{onceki_ap_mod or '-'} -> {ap_mod}")]
                print(f"[goruntulu:{self.k.ad}] OTOPILOT MODU: "
                      f"{onceki_ap_mod or '-'} -> {ap_mod}")
                onceki_ap_mod = ap_mod
            _olay_yaz(simdi, t_unix, olaylar, menzil)

            def _n(v, b='{:.2f}'):
                return '' if v is None else b.format(v)

            satir = [
                f"{simdi:.4f}", f"{simdi:.4f}", f"{t_unix:.3f}",
                f"{dt:.4f}", 'goruntulu', durum,
                _n(o.ex_deg, '{:.4f}'), _n(o.ey_deg, '{:.4f}'),
                _n(o.bbox_w, '{:.0f}'), _n(o.bbox_h, '{:.0f}'),
                _n(o.alan_kok), _n(None if o.bbox_w is None
                                   else o.bbox_w * o.bbox_h, '{:.0f}'),
                _n(o.kapsama_pct, '{:.3f}'),
                f"{bbox_yas:.3f}" if math.isfinite(bbox_yas) else '',
                _n(o.t_capture, '{:.4f}'),
                _n(o.px_sanal_x, '{:.1f}'), _n(o.px_sanal_y, '{:.1f}'),
                _n(o.px_ham_cx, '{:.1f}'), _n(o.px_ham_cy, '{:.1f}'),
                _n(kenar_px, '{:.1f}'), _n(kenar_deg, '{:.2f}'),
                f"{ham_yas:.3f}" if math.isfinite(ham_yas) else '',
                _n(o.tilt_deg, '{:.3f}'),
                _n(menzil),
                f"{v_cmd[0]:.3f}", f"{v_cmd[1]:.3f}", f"{v_cmd[2]:.3f}",
                f"{float(np.linalg.norm(v_cmd)):.3f}",
                _n(yaw_cmd_dps), int(kelepce_hiz), int(kelepce_irtifa),
                int(kelepce_yaw),
                *(('', '', '', '') if pos is None else
                  (f"{pos[0]:.2f}", f"{pos[1]:.2f}", f"{pos[2]:.2f}",
                   f"{-float(pos[2]):.2f}")),
                *(('', '', '', '') if vel is None else
                  (f"{vel[0]:.2f}", f"{vel[1]:.2f}", f"{vel[2]:.2f}",
                   f"{biz_hiz:.2f}")),
                *(('', '', '') if o.acc_ned is None else
                  (f"{o.acc_ned[0]:.2f}", f"{o.acc_ned[1]:.2f}",
                   f"{o.acc_ned[2]:.2f}")),
                *(('', '', '') if att is None else
                  (f"{math.degrees(att[0]):.2f}", f"{math.degrees(att[1]):.2f}",
                   f"{math.degrees(att[2]):.2f}")),
                _n(biz_rota, '{:.1f}'),
                '' if vibe is None else f"{max(vibe):.1f}",
                # --- ref_* : YALNIZ ANALIZ, gudume girmedi ---
                *(('', '', '') if ref['pos'] is None else
                  (f"{ref['pos'][0]:.2f}", f"{ref['pos'][1]:.2f}",
                   f"{ref['pos'][2]:.2f}")),
                *(('', '', '') if h_vel is None else
                  (f"{h_vel[0]:.2f}", f"{h_vel[1]:.2f}", f"{h_vel[2]:.2f}")),
                _n(h_hiz), _n(h_rota, '{:.1f}'),
                *(('', '', '') if ref['acc'] is None else
                  (f"{ref['acc'][0]:.2f}", f"{ref['acc'][1]:.2f}",
                   f"{ref['acc'][2]:.2f}")),
                _n(ref['donus_dps'], '{:.2f}'),
                _n(geo['menzil_m']), _n(geo['kerteriz_deg'], '{:.1f}'),
                _n(geo['yukselis_deg'], '{:.2f}'),
                _n(geo['yaklasim_deg'], '{:.1f}'), geo['tip'],
                _n(geo['kapanma_mps']), _n(geo['tgo_s']),
                _n(geo['cpa_m']), _n(geo['cpa_s']),
                '|'.join(ad for ad, _ in olaylar),
                # --- sistem sagligi (SONA eklendi, indis-koruma) ---
                f"{dongu_hz_ort:.2f}", dt_asim, hayatta_ttl, ap_mod,
                _n(hb_yas_s, '{:.2f}'),
                # --- donuk dikdortgen (SONA eklendi, indis-koruma) ---
                _n(o.rot_w_px, '{:.1f}'), _n(o.rot_h_px, '{:.1f}'),
                _n(o.rot_aci_deg, '{:.2f}'),
            ]
            if len(satir) != len(LOG_KOLONLARI) and not _basliksiz_uyari[0]:
                _basliksiz_uyari[0] = True
                print(f"[goruntulu:{self.k.ad}] LOG UYARISI: satirda "
                      f"{len(satir)} alan var, baslikta {len(LOG_KOLONLARI)}. "
                      f"Kolonlar KAYMIS olabilir!")
            log.writerow(satir)
            # PERIYODIK FLUSH (2026-08-07): ana CSV eskiden YALNIZ kapanista
            # bosaltiliyordu -- yani cakilmada/SIGKILL'de son ~2 s (8 KB'lik
            # blok tamponu) diske hic yazilmiyordu ve kaza aninin satirlari
            # tam da kayip olan kisimdi. mpc_tani'deki desenle ayni: 20
            # satirda bir, 20 Hz'de ~1 Hz.
            satir_sayaci += 1
            if satir_sayaci % 20 == 0:
                log_f.flush()

            kalan = self.loop_dt - (time.monotonic() - simdi)
            if kalan > 0:
                time.sleep(kalan)

        log_f.close()
        olay_f.close()
        print(f"[goruntulu:{self.k.ad}] dongu bitti, log kapatildi")


# ----------------------------------------------------------------- kuru test

class _TutucuKontrolcu(GoruntuluKontrolcu):
    """Entegrasyon tutkalini sinamak icin en yalin kontrolcu: devirden gelen
    hizi aynen surdurur. Gudum YAPMAZ; yalniz devir mekanigi + komut yolunun
    calistigini kanitlamak icindir."""
    ad = "tutucu"

    def __init__(self):
        self.v = np.zeros(3)

    def tohumla(self, devir):
        if devir and 'cmd_vel_ned' in devir:
            self.v = np.asarray(devir['cmd_vel_ned'], dtype=float)

    def komut(self, olcum):
        return Komut(vel_ned=self.v.copy())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tutucu', action='store_true',
                   help='tutucu kuru-test kontrolcusuyle kos')
    p.add_argument('--sure', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    a = p.parse_args()
    if not a.tutucu:
        raise SystemExit("Bu modul iskelettir; yontem kodundan import edin "
                         "ya da --tutucu ile kuru test kosun.")
    GoruntuluDongu(_TutucuKontrolcu(), loop_hz=a.loop_hz).calistir(a.sure)


if __name__ == '__main__':
    main()
