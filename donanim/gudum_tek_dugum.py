#!/usr/bin/env python3
"""
gudum_tek_dugum.py - BAGIMSIZ, TEK SURECLI goruntulu gudum dugumu
=================================================================
NEDEN VAR (mimari itiraz, 2026-08-07 takim talebi)
--------------------------------------------------
Bugunku zincirde KARARI TESPIT SURECI VERIYOR:

    bbox_to_redis.py  --('komut_yetkisi' = goruntulu/konumlu)-->  gudum
    goruntulu_temel.GoruntuluDongu  ->  yetkiyi BEKLER, gelince MPC'yi kosar

Yani "angaje olayim mi" sorusunu goren goz cevapliyor, silahi tutan el degil.
Takim arkadasinin itirazi bu: GUDUM DUGUMU KENDI KARARINI KENDI VERMELI.
Bu dosya AYNI ISLEVI tek surecte ve KENDI karariyla yapar:

    bbox_to_redis.py --('tracker_bbox_stab' yayini, YALNIZ OLCUM)--> BU DUGUM
    BU DUGUM: kapilari degerlendir -> ANGAJE -> LOS/PN -> hiz komutu -> BIRAK

'komut_yetkisi' HIC OKUNMAZ. Karar, asagidaki angajman kapisindadir.

GUNCEL YASA VE DONANIM KARARI (2026-08-11)
------------------------------------------
  * Varsayilan goruntulu yasa terminal_los_gudum.TerminalLosKontrolcu'dur.
    MPC yalniz karsilastirma/geri donus icin ``--gudum mpc`` ile yuklenir;
    varsayilan kurulum optimizasyon kutuphanelerine bagimli degildir.
  * Donanim kapisi ``--buyuk-kare 5 --alan-pct 3``: art arda bes TAZE bbox
    alan esigini gecince angaje olur. Merkez ve hedef telemetrisi aranmaz.
    YOLO confidence filtresi kamera koprusunda tespitten ONCE uygulanir;
    dolayisiyla sayaca yalniz detectorun kabul ettigi kareler girer.
  * Konumlu yaklaşim hedefin arkasindaki slotu kurmaya devam eder. LOS/PN
    yalniz goruntulu yetki alindiktan sonra calisir.

KORUNAN PARCALAR (bilerek): olcum sozlesmesi, emniyet ve birakma katmani
---------------------------------------------------------------------------
  * ANGAJMAN KURALI da ayni: bbox_to_redis'in bugun simde dogrulanan UC
    KAPILI kurali (pencere + alan + menzil) buraya TASINDI. Kopyalandi
    cunku o mantik SuruRedisDetector sinifinin ic durumuna (ROS kare
    dongusu, decision_window) gomulu; import edilebilecek bir yuzeyi yok.
    Esikler ayni env degiskenlerinden okunur ki iki taraf ayrismasin.
  * Hata sinyali sanal gimbalden gelir: Redis 'tracker_bbox_stab'
    [sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]. Ham 7 elemanli
    'tracker_bbox' YEDEK yoldur (stabilize kanal yoksa; bkz. _ham_stab).
  * MENZIL YALNIZ MENZILDIR: hedefin telemetrisinden turetilen tek buyukluk
    menzildir (kullanici kurali 2026-08-03). Iki kaynak secilebilir:
      estimator -> goruntulu_temel.MenzilKestirici (IMM, hedefin
                   GLOBAL_POSITION_INT'i, kendi MAVLink baglantisi 14604)
      redis     -> 'devir_durumu'.range_m (yer istasyonu / konumlu besliyorsa)
    Menzil kesilirse SON GECERLI DEGER DONDURULUR ve acikca uyarilir --
    LOS/PN de PN kazanci, t_go, dikey kanal ve ISKA icin menzili kullanir.
    Bu nedenle gorsel GECIS telemetrisiz olsa da mevcut terminal yasa henuz
    tamamen telemetrisiz degildir.
  * Komut HIZDIR: SET_POSITION_TARGET_LOCAL_NED, yalniz vx,vy,vz (+yaw_rate).
    Emniyet katmani (hiz kelepcesi, irtifa tabani, komut LPF'si, yaw slew +
    LPF) goruntulu_temel.GoruntuluDongu.calistir()'dan TASINDI. Kopyalandi
    cunku o katman tek bir dev metodun govdesine gomulu, ayri fonksiyon
    degil. Her parcanin gerekcesi yaninda korundu ki iki dosya ayrisirsa
    NEDEN'i kaybolmasin.

ANGAJMAN DURUM MAKINESI (asil fark: karar burada)
-------------------------------------------------
    BEKLE  --(angajman kapisi acildi)-->  ANGAJE
    ANGAJE --(kayip merdiveni | tespit orani coktu | ISKA)--> BIRAK -> BEKLE

  VARSAYILAN KURAL -- UC KAPI (bbox_to_redis ile AYNI):
     1) KARARLILIK: son 25 karenin >= 20'si gecerli tespit olmali (%80).
        Esikler bbox_to_redis ile ayni env'den: YILDIZ_PENCERE_KARE /
        YILDIZ_PENCERE_ORAN.
     2) ALAN: bbox alani kadrajin (%2 x %2) dikdortgeninden buyuk olmali
        (kalibrasyon: bbox_alan ~ 4.65e5/r^2 -> %2x%2 ~ 35 m).
     3) MENZIL: menzil <= 60 m. Menzil yoksa/bayatsa bu kapi ATLANIR
        (bbox_to_redis'teki ayni davranis).

  PENCERE KARE CINSINDENDIR, SURE DEGIL (2026-08-07 kullanici karari):
  25 kare 30 fps'lik simde 0.83 s, ~20 Hz'lik YOLO donaniminda 1.25 s
  eder. (Eski 45 kare sirasiyla 1.5 s ve 2.25 s idi -- ayni kod donanimda
  yarim saniye gec angaje oluyordu; 25 kare iki tarafi da makul bantta
  tutar.) Bu farki logdan gormek icin tahmin gerekmesin diye IKI hiz da
  her satirda yazilir:
      tespit_fps : bbox_to_redis'in YAYIN hizi (ayri sayac thread'i olcer)
      gozlem_fps : dongunun GORDUGU ayri kare hizi = min(loop_hz, tespit_fps)
  Pencere gozlenen karelerle isler -- gudum ancak gordugu kareye tepki
  verebilir. Yani 20 Hz'lik bir donguda 25 kare 1.25 s eder, kamera 30 fps
  olsa bile. Daha kisa istiyorsan --loop-hz 30 (ya da --pencere-kare).
  Isteyen pencereyi SURE cinsinden de kurabilir: --pencere-s (varsayilan
  DEGIL).

  BASIT KURAL (opsiyon, VARSAYILAN KAPALI): --basit-kare N verilirse uc
  kapi atlanir, "art arda N gecerli kare" yeter. bbox_to_redis'teki
  YILDIZ_GECIS_BASIT ile ayni mantik; sahada ongorulebilir davranis
  isteyen icin. Env karsiligi: YILDIZ_TEKDUGUM_KARE.

  KAYIP MERDIVENI (goruntulu_temel ile ayni esikler, ayni gerekcelerle):
      yas <= 0.7 s          -> 'taze', MPC kosar
      0.7 < yas <= 1.7 s    -> 'tut',  son gecerli komut TUTULUR
      yas > 1.7 s           -> 'suz',  OLCULEN hiza sonumleme (sifir DEGIL:
                               sifir "komut yok" degil, tam fren komutudur)
      yas > birak_s (2.5 s) -> BIRAK
  Ikinci birakma yolu (bbox_to_redis'in geri donus kurali): penceredeki
  gecerli kare sayisi 3'un altina duser ve 2 s dwell dolarsa BIRAK. Kayip
  merdiveni SUREKLI kaybi yakalar, bu yol TITREK tespiti yakalar.
  BIRAK sonrasi SOGUMA (varsayilan 3 s) vardir: bbox yeniden gorunur
  gorunmez angaje olmak ping-pong uretir (bbox_to_redis'te olculdu:
  cevrim ~1.2 s, dakikada ~50 devir -> DEVIR_SOGUMA_S).

DISARIYA ILAN (formation_KILLER protokolu)
------------------------------------------
  Redis 'tekdugum_durum'  : JSON, HER durum degisiminde + periyodik tazeleme
                            {'durum','sebep','t_mono','t_unix','menzil_m',...}
  Redis 'tekdugum_hayatta': TTL'li kalp atisi (olu-adam anahtari)
  MAVLink STATUSTEXT      : "GORUNTULU: komut bende" / "GORUNTULU: biraktim,
                            dinliyorum"  (--no-statustext ile kapanir)
  Otorite kanal REDIS'tir; STATUSTEXT insan/GCS icin ekodur (formation_KILLER
  zaten STATUSTEXT logluyor, bkz. _handle_statustext).

KULLANIM
--------
    python3 donanim/gudum_tek_dugum.py --gudum los --buyuk-kare 5 --alan-pct 3
    python3 donanim/gudum_tek_dugum.py --gudum mpc            # yalniz A/B
    python3 donanim/gudum_tek_dugum.py --dry-run --sahte-mavlink   # masa testi

Ayrintili aciklama: donanim/README.md
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
from datetime import datetime
from pathlib import Path

import numpy as np
import redis

# --- repo ici modul yolu -------------------------------------------------
# Bu dosya donanim/ altinda ama gudum kodu guidance_allstar/ altinda ve o
# paket kendi icinde DUZ import kullaniyor ("import guidance_config as cfg").
# Bu yuzden paket olarak degil, dizini sys.path'e ekleyerek import ediyoruz;
# baska turlu goruntulu_temel'in kendi importlari kirilir.
_KOK = Path(__file__).resolve().parent.parent
_GA = _KOK / 'guidance_allstar'
if str(_GA) not in sys.path:
    sys.path.insert(0, str(_GA))

import guidance_config as cfg                                   # noqa: E402
from goruntulu_temel import (                                   # noqa: E402
    HAYATTA_TTL_S,
    BboxOkuyucu,
    HizKomutcu,
    LOG_KOLONLARI,
    MenzilKestirici,
    Olcum,
    OlayIzleyici,
    _KADRAJ_FX,
    _KADRAJ_H,
    _KADRAJ_W,
    _karsilasma_geometrisi,
)
from terminal_los_gudum import TerminalLosKontrolcu             # noqa: E402
from numeric_differentiation import VelocityDifferentiator      # noqa: E402

# ALT CIZGILI ISIMLER: _KADRAJ_* ve _karsilasma_geometrisi goruntulu_temel'de
# "modul ici" olarak isaretlenmis ama ikisi de LOG-ONLY yardimci. Kopyalamak
# yerine import ediyoruz -- kadraj geometrisi ve karsilasma analizi iki
# dosyada AYRISIRSA loglar karsilastirilamaz hale gelir.

# ---------------------------------------------------------------- sabitler

# Angajman durumlari (Redis 'tekdugum_durum' bunlari aynen yayinlar).
BEKLE = 'BEKLE'
ANGAJE = 'ANGAJE'

# Redis anahtarlari. 'goruntulu_hayatta' ile BILEREK CAKISMIYOR: bbox_to_redis
# olu-adam kapisinda o anahtari ariyor ve orada bir surec varmis gibi
# gorunmek bu dugumun isi degil (biz yetki istemiyoruz, yetkiyi ALIYORUZ).
ANAHTAR_DURUM = 'tekdugum_durum'
ANAHTAR_HAYATTA = 'tekdugum_hayatta'
ANAHTAR_DEVIR = 'devir_durumu'          # yalniz OKUNUR (tohum + redis menzili)
ANAHTAR_YETKI = 'komut_yetkisi'         # yalniz ISTEGE BAGLI yazilir, OKUNMAZ

# Menzil kesildiginde: son deger dondurulur, bu suredan sonra yuksek sesle
# uyarilir. Uyari periyodiktir (spam yok).
MENZIL_DONMA_UYARI_S = 1.0
MENZIL_UYARI_PERIYOT_S = 5.0

# STATUSTEXT metinleri -- PROTOKOL. Degistirirsen formation_KILLER tarafinda
# da degistir (README'deki tabloda ayni metinler var).
ST_ANGAJE = "GORUNTULU: komut bende"
ST_BIRAK = "GORUNTULU: biraktim, dinliyorum"

# Ana CSV'ye tek dugume OZGU kolonlar. LOG_KOLONLARI'nin SONUNA eklenir:
# eski araclar DictReader kullaniyor (ad onemli, sira degil) ve sona ekleme
# diff'i okunur tutar -- goruntulu_temel'deki ayni kural.
EK_KOLONLAR = [
    'tekdugum_durum',     # BEKLE | ANGAJE
    'kural',              # uc_kapi_kare | uc_kapi_sure | basit
    'tespit_fps',         # YAYIN hizi: bbox_to_redis kac Hz basiyor (sim ~30,
                          # donanimda YOLO ~20). KareSayaci olcer, LOG-ONLY.
    'gozlem_fps',         # dongunun GORDUGU ayri kare hizi = min(loop_hz,
                          # tespit_fps). Angajman penceresi BUNUNLA isler.
    'pencere_ornek',      # kararlilik penceresindeki ornek sayisi (<= 25)
    'gecerli_kare',       # penceredeki gecerli ornek sayisi (esik 20)
    'gecerli_oran',       # penceredeki gecerli ornek orani [0..1]
    'kapi_alan',          # 1 = alan kapisi acik (bbox yeterince buyuk)
    'kapi_menzil',        # 1 = menzil kapisi acik (ya da atlandi)
    'ardisik_kare',       # basit kural sayaci
    'ardisik_buyuk_kare', # kisa LOS kapisi: art arda alan-gecerli bbox
    'kayip_s',            # kesintisiz bbox kaybi suresi [s]
    'menzil_kaynak',      # estimator | redis | sahte
    'menzil_taze',        # 1 = olculdu, 0 = DONDURULMUS (son gecerli deger)
    'menzil_yas_s',       # son gecerli menzilden beri gecen sure
    'kuru',               # 1 = --dry-run (komut MAVLink'e GITMEDI)
]


# ------------------------------------------------------------ menzil kaynagi
#
# Uc kaynak da ayni yuzeyi verir:  menzil(pos_ned) -> float|None
#                                  ref_hedef_durum() -> dict (LOG-ONLY)
# Boylece --menzil-kaynak secimi dongu kodunda TEK satir kalir.


class EstimatorMenzil:
    """goruntulu_temel.MenzilKestirici sarmalayicisi (VARSAYILAN kaynak).

    Kestiriciyi KOPYALAMIYORUZ: IMM kurulumu, bayatlik esigi ve
    "disariya yalniz menzil" sozlesmesi orada tek yerde duruyor."""

    ad = 'estimator'

    def __init__(self, target_conn_str, home):
        self._k = MenzilKestirici(target_conn_str, *home)
        self._k.start()

    def menzil(self, pos_ned):
        return self._k.menzil(pos_ned)

    def ref_hedef_durum(self):
        return self._k.ref_hedef_durum()


class RedisMenzil:
    """'devir_durumu'.range_m -- konumlu surec / yer istasyonu besliyorsa.

    NICIN VAR: bu dugum tek basina ucabilsin diye kendi MAVLink baglantisini
    (14604) acan estimator varsayilandir; ama ayni menzili ZATEN hesaplayan
    bir surec kosuyorsa (simple_guided_follow her dongude 'devir_durumu'na
    range_m yaziyor) ikinci bir IMM kosturmak bosuna CPU ve ikinci bir
    kestirim hatasi kaynagidir. KURAL IHLALI DEGIL: okunan tek alan range_m.
    """

    ad = 'redis'
    BAYAT_S = 3.0          # bbox_to_redis.DEVIR_BAYAT_S ile ayni esik

    def __init__(self, r):
        self.r = r

    def menzil(self, pos_ned):
        try:
            ham = self.r.get(ANAHTAR_DEVIR)
            if not ham:
                return None
            d = json.loads(ham)
            menzil = d.get('range_m')
            t = d.get('t_mono')
            if menzil is None:
                return None
            # t_mono YAZAN SURECIN monotonic saatidir; ayni makinede ayni
            # saat oldugu icin fark anlamli (Linux'ta CLOCK_MONOTONIC sistem
            # capinda). Farkli makinede (Pi <-> yer) bu kontrol yanilir ->
            # o kurulumda estimator kullanin.
            if t is not None and (time.monotonic() - float(t)) > self.BAYAT_S:
                return None
            return float(menzil)
        except Exception:
            return None

    def ref_hedef_durum(self):
        # Hedefin konum/hiz/ivmesine erisimimiz yok -> ref_* kolonlari bos.
        return {'pos': None, 'vel': None, 'acc': None,
                'donus_dps': None, 'est_pos': None}


class SahteMenzil:
    """MASA TESTI: kapanan sahte bir menzil uretir (60 m -> 5 m, 5 m/s)."""

    ad = 'sahte'

    def __init__(self, baslangic_m=60.0, kapanma_mps=5.0):
        self.t0 = time.monotonic()
        self.r0 = float(baslangic_m)
        self.v = float(kapanma_mps)

    def menzil(self, pos_ned):
        return max(5.0, self.r0 - self.v * (time.monotonic() - self.t0))

    def ref_hedef_durum(self):
        return {'pos': None, 'vel': None, 'acc': None,
                'donus_dps': None, 'est_pos': None}


class KareSayaci(threading.Thread):
    """'tracker_bbox_stab' varislarini SAYAR -- tek isi YAYIN hizini olcmek.

    NICIN AYRI THREAD (kuru testte olculdu): gudum dongusu 20 Hz kosarken
    kamera 30 fps yayinliyorsa dongu icinden sayilan "ayri kare" sayisi en
    fazla DONGU hizina cikar (olculen deger 19.9 idi). Yani dongu, yayinin
    gercek hizini PRENSIP OLARAK goremez. Bu thread her mesaji gordugu icin
    gercek hizi olcer.

    Sayi YALNIZ LOG icindir (tespit_fps kolonu): "sim 30 fps / donanim
    ~20 Hz YOLO" farki loglardan tahminsiz okunabilsin diye. Angajman
    penceresi bu sayiyla DEGIL, dongunun GORDUGU karelerle isler -- gudum
    ancak gordugu kareye tepki verebilir."""

    def __init__(self, r, kanal='tracker_bbox_stab'):
        super().__init__(daemon=True)
        self.pubsub = r.pubsub(ignore_subscribe_messages=True)
        self.pubsub.subscribe(kanal)
        self.kilit = threading.Lock()
        self._t = deque(maxlen=120)

    def run(self):
        for _ in self.pubsub.listen():
            with self.kilit:
                self._t.append(time.monotonic())

    def fps(self):
        simdi = time.monotonic()
        with self.kilit:
            while self._t and simdi - self._t[0] > 3.0:
                self._t.popleft()
            n = len(self._t)
            if n < 2:
                return 0.0
            return (n - 1) / max(self._t[-1] - self._t[0], 1e-6)


class MenzilTutucu:
    """Kaynagin ustune DONDURMA + UYARI katmani.

    NICIN (kullanici talebi 2026-08-07): menzil kesildiginde None gecirmek
    MPC'yi MpcAyar.menzil_yoksa_m (55 m) varsayimina dusurur. 20 m'de
    kapanirken 55 m varsaymak c = KDEG/r katsayilarini ~3 kat kucultur,
    yani MPC "vaktim var" diye yumusak komut uretir. Son GERCEK olcumu
    dondurmak yanlis ama TUTARLI bir varsayimdir ve durum kolonlarindan
    (menzil_taze / menzil_yas_s) sonradan ayirt edilebilir."""

    def __init__(self, kaynak, yaz=print):
        self.kaynak = kaynak
        self._son = None
        self._son_t = 0.0
        self._son_uyari = 0.0
        self._yaz = yaz
        self.taze = False
        self.yas_s = float('inf')

    def menzil(self, pos_ned):
        m = self.kaynak.menzil(pos_ned)
        simdi = time.monotonic()
        if m is not None and math.isfinite(m):
            self._son, self._son_t = float(m), simdi
            self.taze, self.yas_s = True, 0.0
            return self._son
        self.taze = False
        if self._son is None:
            self.yas_s = float('inf')
            self._uyar(simdi, "MENZIL HIC GELMEDI -- MPC menzil_yoksa_m "
                              "(varsayilan 55 m) sabitine dusecek!")
            return None
        self.yas_s = simdi - self._son_t
        if self.yas_s > MENZIL_DONMA_UYARI_S:
            self._uyar(simdi, f"MENZIL KESILDI ({self.yas_s:.1f} s) -- son "
                              f"gecerli deger {self._son:.1f} m DONDURULDU")
        return self._son

    def _uyar(self, simdi, metin):
        if simdi - self._son_uyari < MENZIL_UYARI_PERIYOT_S:
            return
        self._son_uyari = simdi
        self._yaz(f"[tekdugum] *** {metin} ***")


# ------------------------------------------------------------ angajman kapisi

class AngajmanKapisi:
    """"Angaje olayim mi?" sorusunun TEK cevap yeri.

    KAYNAK: bbox_to_redis.SuruRedisDetector karar vericisi (_evaluate_frame /
    _make_decision). Oradan TASINDI cunku o mantik ROS kare dongusune ve
    sinif ic durumuna gomulu -- import edilebilir bir yuzeyi yok. Esikler
    AYNI env degiskenlerinden okunur (YILDIZ_PENCERE_KARE / _ORAN /
    YILDIZ_GECIS_ALAN_PCT / YILDIZ_GECIS_MENZIL) ki iki taraf ayni ayarla
    ayni davransin.

    VARSAYILAN: KARE pencereli uc kapi -- 25 karede >= 20 gecerli (%80).
    NOT (2026-08-07 kullanici karari): pencere KARE cinsindendir, sure
    degil; 25 kare 30 fps'te 0.83 s, 20 Hz'lik YOLO'da 1.25 s eder.
    Eski 45 kare simde 1.5 s / donanimda 2.25 s ediyordu -- kod degismeden
    devir yarim saniye gecikiyordu. 25 kare iki tarafi da makul bantta
    tutar. Olculen tespit hizi (fps) her halukarda LOGLANIR (tespit_fps
    kolonu) ki bu farki logdan gormek icin tahmin gerekmesin.

    SECENEK: --pencere-s ile pencere SURE tabanli olur (o zaman kare
    sayisi degil, son X saniyedeki orneklerin orani bakilir). Varsayilan
    DEGIL; donanimda fps cok oynarsa diye hazir duruyor.

    KARE PENCERESINI NASIL KURUYORUZ (onemli fark): bbox_to_redis her
    KAMERA karesini gorur ve gecersizler icin de pencereye False yazar.
    Biz yalniz GECERLI tespitleri goruyoruz (bbox_to_redis Redis'e sadece
    onlari yayinliyor). Bu yuzden kacan kareler OLCULEN kare periyodundan
    (1/fps) yeniden kurulur: son kareden beri bir periyottan uzun sure
    gectiyse pencereye False yazilir. Sonuc bbox_to_redis'in penceresiyle
    ayni anlama gelir; kuantalama farki bir kare periyodudur.
    """

    # bbox_to_redis.REVERT_THRESHOLD ile ayni: 25 karelik pencerede 3
    # gecerli kare kalirsa (+2 s dwell) tespit KAYBEDILMIS sayilir.
    BIRAK_KARE = 3
    VARSAYILAN_FPS = 30.0        # fps olculene kadar kullanilan kare periyodu

    def __init__(self, pencere_kare=None, gecerli_oran=None, pencere_s=None,
                 alan_pct=None, menzil_esigi_m=None, basit_kare=None,
                 buyuk_kare=None, birak_dwell_s=2.0):
        # Esikler bbox_to_redis ile AYNI env'den; senaryo.sh bir kez
        # ayarlayinca iki taraf da ayni sayiyi gorur.
        self.pencere_kare = max(3, int(float(
            pencere_kare if pencere_kare is not None
            else os.environ.get('YILDIZ_PENCERE_KARE', '25'))))
        self.gecerli_oran = float(
            gecerli_oran if gecerli_oran is not None
            else os.environ.get('YILDIZ_PENCERE_ORAN', '0.8'))
        self.gecerli_esik = max(2, int(round(self.pencere_kare
                                             * self.gecerli_oran)))
        self.pencere_s = None if pencere_s is None else float(pencere_s)
        self.alan_pct = float(alan_pct if alan_pct is not None
                              else os.environ.get('YILDIZ_GECIS_ALAN_PCT', '2'))
        self.menzil_esigi_m = float(
            menzil_esigi_m if menzil_esigi_m is not None
            else os.environ.get('YILDIZ_GECIS_MENZIL', '60'))
        self.basit_kare = None if not basit_kare else max(1, int(basit_kare))
        self.buyuk_kare = None if not buyuk_kare else max(1, int(buyuk_kare))
        self.birak_dwell_s = float(birak_dwell_s)
        # KALMA esigi (histerezis): angajedeyken gecerlilik olcutu ALAN degil
        # kapsamadir ve esigi cok dusuktur -- bbox_to_redis.MIN_COVERAGE_HOLD.
        # Gerekce orada: geri devri tespit KAYBI yonetmeli, boyut degil.
        self.kalma_kapsama_pct = float(os.environ.get('YILDIZ_COV_KAL', '0.3'))
        self._pencere = deque(maxlen=(None if self.pencere_s is not None
                                      else self.pencere_kare))  # (t, gecerli)
        self._kare_t = deque(maxlen=90)  # AYRI kare varislari (fps olcumu)
        self._son_slot_t = None          # son yazilan pencere slotunun ani
        self._son_imza = None
        self.ardisik_kare = 0
        self.ardisik_buyuk_kare = 0
        self._birak_dwell_t0 = None
        # Son degerlendirme sonuclari (LOG kolonlari bunlari yazar).
        # self.fps = dongunun GORDUGU ayri kare hizi ('gozlem_fps' kolonu),
        # yayin hizi DEGIL: dongu 20 Hz ise 30 fps'lik yayindan en fazla
        # 20 ayri kare gorulebilir. Yayin hizini KareSayaci olcer.
        self.fps = 0.0
        self.oran = 0.0
        self.ornek = 0
        self.gecerli_sayi = 0
        self.kapi_alan = False
        self.kapi_menzil = False

    @property
    def mod(self):
        if self.buyuk_kare:
            return 'buyuk_kare'
        if self.basit_kare:
            return 'basit'
        return 'uc_kapi_sure' if self.pencere_s is not None else 'uc_kapi_kare'

    # -- olcum girisi -----------------------------------------------------

    def alan_esigi_px2(self):
        """(p% x p%) dikdortgeninin alani [px^2] -- bbox_to_redis ile ayni."""
        p = self.alan_pct / 100.0
        return (p * _KADRAJ_W) * (p * _KADRAJ_H)

    def kare_periyodu(self):
        """GOZLENEN kare periyodu [s]; henuz olculmediyse varsayilan.

        Pencere slotlari bu periyotla ilerler. Yayin hizi (tespit_fps)
        DEGIL gozlem hizi kullanilir; yoksa dongunun goremedigi kareler
        'kacirilmis' sayilir ve pencere hicbir zaman dolmaz."""
        return 1.0 / (self.fps if self.fps > 1e-6 else self.VARSAYILAN_FPS)

    def kare_bayat_s(self):
        """Bir tespitin "yeni kare" sayildigi ust sure [s].

        OLCULEN fps'ten turetilir (~2.5 kare periyodu). Sabit bir esik
        (orn. 0.15 s) 30 fps'te dogru, 10 Hz'lik bir YOLO'da her kareyi
        bayat sayardi. Kelepce: en az 0.10 s, en cok 0.35 s."""
        return float(min(0.35, max(0.10, 2.5 * self.kare_periyodu())))

    def ornekle(self, t, imza, bbox_yas_s, kapsama_pct, alan_px2, menzil,
                menzil_taze, angaje):
        """Her DONGUDE bir kez cagrilir. Pencereyi ve sayaclari gunceller."""
        # 1) AYRI kare varislari -> fps. Ayni kareyi iki kez saymamak icin
        #    imza (t_capture + piksel/boyut) kullanilir: kamera 30 Hz,
        #    dongu 20 Hz -- imzasiz sayim fps'i dongu hizina esitlerdi.
        yeni_kare = imza is not None and imza != self._son_imza
        if yeni_kare:
            self._son_imza = imza
            self._kare_t.append(t)
        while self._kare_t and t - self._kare_t[0] > 3.0:
            self._kare_t.popleft()
        if len(self._kare_t) >= 2:
            self.fps = (len(self._kare_t) - 1) / max(
                self._kare_t[-1] - self._kare_t[0], 1e-6)
        else:
            self.fps = 0.0

        # 2) Bu kare 'gecerli' mi? (bbox_to_redis._evaluate_frame)
        #    BEKLE'de olcut ALAN kapisi, ANGAJE'de dusuk kapsama esigi.
        kare_taze = bbox_yas_s <= self.kare_bayat_s()
        self.kapi_alan = bool(kare_taze and alan_px2 is not None
                              and alan_px2 >= self.alan_esigi_px2())
        if angaje:
            gecerli = bool(kare_taze and (kapsama_pct is None
                                          or kapsama_pct >= self.kalma_kapsama_pct))
        else:
            gecerli = self.kapi_alan

        # 3) PENCEREYI ILERLET.
        if self.pencere_s is not None:
            # SURE MODU (opsiyon): her dongude bir ornek, eski ornekler
            # zamanla dusER.
            self._pencere.append((t, gecerli))
            while self._pencere and t - self._pencere[0][0] > self.pencere_s:
                self._pencere.popleft()
        else:
            # KARE MODU (VARSAYILAN): slot slot ilerler. Kacan kareler
            # olculen periyottan yeniden kurulur (bkz. sinif docstring'i).
            periyot = self.kare_periyodu()
            if self._son_slot_t is None:
                if yeni_kare:
                    self._pencere.append((t, gecerli))
                    self._son_slot_t = t
            else:
                # Kacirilan slotlar: gecersiz kare olarak yazilir. Dongu
                # basi en fazla pencere boyu kadar doldurulur (uzun bir
                # duraklamadan sonra sonsuz donguye girmesin).
                doldur = 0
                while (t - self._son_slot_t) > 1.5 * periyot and \
                        doldur < self.pencere_kare:
                    self._son_slot_t += periyot
                    self._pencere.append((self._son_slot_t, False))
                    doldur += 1
                if doldur >= self.pencere_kare:
                    self._son_slot_t = t - periyot
                if yeni_kare:
                    self._pencere.append((t, gecerli))
                    self._son_slot_t = t
        self.ornek = len(self._pencere)
        self.gecerli_sayi = sum(1 for _, g in self._pencere if g)
        self.oran = (self.gecerli_sayi / self.ornek) if self.ornek else 0.0

        # 4) BASIT KURAL sayaci: art arda GECERLI TESPIT karesi (alan/menzil
        #    kapisi ARANMAZ -- bbox_to_redis.ardisik_gecerli ile ayni).
        if not kare_taze:
            self.ardisik_kare = 0
            self.ardisik_buyuk_kare = 0
        elif yeni_kare:
            self.ardisik_kare += 1
            if self.kapi_alan:
                self.ardisik_buyuk_kare += 1
            else:
                self.ardisik_buyuk_kare = 0

        # 5) MENZIL KAPISI. Menzil yok/bayatsa ATLANIR (bbox_to_redis:
        #    "devir_durumu yoksa/bayatsa kapi atlanir -- eski davranis").
        #    DONDURULMUS menzil de "bayat" sayilir: kapi kararini eski bir
        #    olcumle vermek, kapiyi hic sormamaktan daha tehlikeli degil --
        #    ama en azindan logda (menzil_taze=0) gorunur.
        self.kapi_menzil = (menzil is None or not menzil_taze
                            or menzil <= self.menzil_esigi_m)

    # -- kararlar ---------------------------------------------------------

    def pencere_dolu(self):
        """bbox_to_redis: 'len(decision_window) < WINDOW_SIZE -> karar yok'."""
        if self.pencere_s is None:
            return self.ornek >= self.pencere_kare
        # SURE MODU: hem sure yayilimi hem asgari ornek sayisi gerekli --
        # dongu 2 Hz'e duserse 1.5 s'lik pencerede 3 ornek olur ve %80
        # orani uc ornekle 'dolmus' sayilirdi.
        if self.ornek < 5:
            return False
        return (self._pencere[-1][0] - self._pencere[0][0]) >= 0.95 * self.pencere_s

    def angaje_olmali(self):
        """(evet_mi, sebep_metni)."""
        if self.buyuk_kare is not None:
            if self.ardisik_buyuk_kare >= self.buyuk_kare:
                return True, (f"buyuk_kare({self.ardisik_buyuk_kare} ardisik, "
                              f"alan %{self.alan_pct:g})")
            return False, ''
        if self.basit_kare is not None:
            if self.ardisik_kare >= self.basit_kare:
                return True, f"basit({self.ardisik_kare} ardisik kare)"
            return False, ''
        if not self.pencere_dolu():
            return False, ''
        if self.pencere_s is None:
            if self.gecerli_sayi < self.gecerli_esik:
                return False, ''
            pencere_metni = (f"{self.gecerli_sayi}/{self.pencere_kare} kare")
        else:
            if self.oran < self.gecerli_oran:
                return False, ''
            pencere_metni = (f"oran {self.oran:.2f} @ {self.pencere_s:.1f} s")
        if not self.kapi_menzil:
            return False, ''
        return True, (f"uc_kapi({pencere_metni}, alan %{self.alan_pct:g}, "
                      f"menzil <= {self.menzil_esigi_m:.0f} m, "
                      f"fps={self.fps:.1f})")

    def birakmali(self, t):
        """Tespit KAYBI (pencere + dwell) -> (evet_mi, sebep).

        bbox_to_redis'in geri donus kurali: valid_count <= REVERT_THRESHOLD
        (3) ve 2 s dwell. Kayip MERDIVENI surekli kaybi yakalar; bu yol
        TITREK tespiti yakalar (hicbir zaman 2.5 s kesintisiz kaybolmaz
        ama saniyede yalnizca bir kare gorunur)."""
        if self.pencere_s is None:
            koptu = self.pencere_dolu() and self.gecerli_sayi <= self.BIRAK_KARE
            metin = f"{self.gecerli_sayi}/{self.pencere_kare} kare"
        else:
            koptu = self.pencere_dolu() and self.oran <= (
                float(self.BIRAK_KARE) / self.pencere_kare)
            metin = f"oran {self.oran:.2f}"
        if not koptu:
            self._birak_dwell_t0 = None
            return False, ''
        if self._birak_dwell_t0 is None:
            self._birak_dwell_t0 = t
            return False, ''
        if t - self._birak_dwell_t0 >= self.birak_dwell_s:
            self._birak_dwell_t0 = None
            return True, f"tespit kaybi ({metin} + {self.birak_dwell_s:.0f}s dwell)"
        return False, ''

    def sifirla(self):
        """Durum degisiminde pencereyi temizle.

        NICIN: gecerlilik olcutu ANGAJE'de degisiyor (alan -> kapsama).
        Eski olcutle doldurulmus bir pencereyle yeni karar vermek,
        bbox_to_redis'in devirde decision_window.clear() yapmasiyla ayni
        sebepten yanlis olur."""
        self._pencere.clear()
        self._son_slot_t = None
        self.ardisik_kare = 0
        self.ardisik_buyuk_kare = 0
        self._birak_dwell_t0 = None
        self.oran = 0.0
        self.ornek = 0
        self.gecerli_sayi = 0


# --------------------------------------------------------- sahte MAVLink ucu
#
# MASA TESTI (--sahte-mavlink): arac, SITL ve ag YOKKEN tum zincir kossun.
# HizKomutcu ile AYNI yuzey; dongu kodu hangisiyle kostugunu bilmez.


class _SahteOkuyucu:
    def __init__(self):
        self.t0 = time.monotonic()

    def get(self):
        # 30 m irtifada, 12 m/s kuzeye giden bir arac taklidi.
        t = time.monotonic() - self.t0
        return (np.array([12.0 * t, 0.0, -30.0]), np.array([12.0, 0.0, 0.0]))

    def get_attitude(self):
        return (0.0, math.radians(-3.0), 0.0)      # roll, pitch, yaw [rad]

    def get_vibration(self):
        return (1.0, 1.0, 1.0)

    def get_heartbeat(self):
        return ('GUIDED', 4, 0.1)                  # (mod, mod_no, yas_s)


class SahteKomutcu:
    """HizKomutcu yerine gecen kuru uc: hicbir sey gondermez, sayar."""

    def __init__(self):
        self.conn = None
        self.okuyucu = _SahteOkuyucu()
        self.gonderilen = 0

    def hiz_gonder(self, vel_ned, yaw_rate_rad=None):
        self.gonderilen += 1


# ------------------------------------------------------------------- yardimci

def _ham_stab(ham_bbox):
    """HAM 'tracker_bbox' [x,y,w,h,kapsama,gecerli,t_capture] -> stab bicimi.

    YEDEK YOL. Stabilize kanal (tracker_bbox_stab) sanal gimbalin ciktisidir;
    gelmiyorsa (gimbal katmani kapali / eski bbox_to_redis) gudum komple
    kor kalmasin diye ham piksel merkezinden acisal hata TURETILIR:
        ex = atan((cx - W/2) / fx),  ey = atan((cy - H/2) / fx)
    UYARI: bu deger GOVDE SALINIMINDAN ARINDIRILMAMISTIR (roll/pitch dogrudan
    ex/ey'ye sizar). Kabul edilebilir tek kullanimi "hic veri yok"tan iyi
    olmasidir; kosuda gorulurse gimbal zincirinde ariza var demektir."""
    if ham_bbox is None or len(ham_bbox) < 4:
        return None
    x, y, w, h = (float(ham_bbox[0]), float(ham_bbox[1]),
                  float(ham_bbox[2]), float(ham_bbox[3]))
    cx, cy = x + w / 2.0, y + h / 2.0
    ex = math.degrees(math.atan((cx - _KADRAJ_W / 2.0) / _KADRAJ_FX))
    ey = math.degrees(math.atan((cy - _KADRAJ_H / 2.0) / _KADRAJ_FX))
    t_cap = float(ham_bbox[6]) if len(ham_bbox) > 6 else None
    return [cx, cy, w, h, ex, ey, t_cap, None]


def _n(v, b='{:.2f}'):
    """None-guvenli sayi bicimleyici (goruntulu_temel'deki ile ayni)."""
    return '' if v is None else b.format(v)


def _mavutil():
    """pymavlink.mavutil'i GEC import eder.

    NICIN: masa testi (--sahte-mavlink + sahte Redis) pymavlink kurulu
    olmayan bir makinede de kosabilsin."""
    from pymavlink import mavutil
    return mavutil


# --------------------------------------------------------------- ana dongu

class TekDugumGudum:
    """Kendi kararini veren, tek surecli goruntulu gudum dongusu."""

    ad = 'tekdugum'

    def __init__(self, kontrolcu, kapi=None, loop_hz=20.0,
                 menzil_kaynak='estimator', statustext=True, kuru=False,
                 sahte_mavlink=False, yetki_yaz=True, tau_s=0.35,
                 bbox_bayat_s=0.7, bosluk_tut_s=1.0, birak_s=2.5,
                 soguma_s=3.0, irtifa_taban_m=15.0, yaw_tau_s=0.15,
                 yaw_slew_dps2=120.0, log_yolu=None, pursuer=None, target=None):
        self.k = kontrolcu
        self.kapi = kapi or AngajmanKapisi()
        self.loop_dt = 1.0 / float(loop_hz)
        self.menzil_kaynak_adi = menzil_kaynak
        self.statustext = bool(statustext)
        self.kuru = bool(kuru)
        self.sahte_mavlink = bool(sahte_mavlink)
        # YETKI YAZMA: bu dugum yetki BEKLEMEZ ama eski konumlu surec
        # (simple_guided_follow) hala 'komut_yetkisi' kapisina bakiyor.
        # Ayni araca iki surec komut yazmasin diye angaje olurken
        # 'goruntulu', birakirken 'konumlu' yaziyoruz. Bu bir IZIN ISTEGI
        # DEGIL, bir ILANDIR: yazip devam ediyoruz, cevap beklemiyoruz.
        # Konumlu surec hic kosmuyorsa --no-yetki-yaz ile kapatin.
        self.yetki_yaz = bool(yetki_yaz)
        self.tau = float(tau_s)
        self.bbox_bayat_s = float(bbox_bayat_s)
        self.bosluk_tut_s = float(bosluk_tut_s)
        self.birak_s = float(birak_s)
        self.soguma_s = float(soguma_s)
        self.irtifa_taban_m = float(irtifa_taban_m)
        self.yaw_tau_s = float(yaw_tau_s)
        self.yaw_slew_dps2 = float(yaw_slew_dps2)
        self.hiz_tavani = float(getattr(cfg, 'GORUNTULU_MAX_SPEED_MPS', 18.0))
        # DIKKAT: konumlunun portlari (14652/14603) DEGIL -- iki surec ayni
        # udpin portunu baglayamaz (goruntulu_temel'deki ayni uyari).
        self.pursuer = pursuer or getattr(cfg, 'GORUNTULU_PURSUER_CONN_STR',
                                          'udpin:127.0.0.1:14654')
        self.target = target or getattr(cfg, 'GORUNTULU_TARGET_CONN_STR',
                                        'udpin:127.0.0.1:14604')
        damga = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_yolu = log_yolu or str(
            Path(__file__).resolve().parent / 'logs'
            / f"tekdugum_{self.k.ad}_{damga}.csv")
        self._calisiyor = True
        self.r = None
        self.durum = BEKLE
        self._durum_sebep = 'baslangic'
        self._son_durum_yayin = 0.0
        self._yayin_fps = 0.0        # KareSayaci olcumu (LOG + durum ilani)

    # -- kurulum ----------------------------------------------------------

    def _baglan(self):
        print("[tekdugum] Redis'e baglaniliyor...")
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.r.ping()
        self.bbox = BboxOkuyucu(self.r)
        self.bbox.start()
        self.sayaci = KareSayaci(self.r)     # LOG-ONLY yayin hizi olcumu
        self.sayaci.start()

        if self.sahte_mavlink:
            # Masa testi: arac yok. Kuru mod ZORUNLU (gonderecek bir ucumuz
            # yok zaten) -- yanlislikla "gercek kostu" sanilmasin diye
            # acikca yaziyoruz.
            self.kuru = True
            self.komutcu = SahteKomutcu()
            print("[tekdugum] *** SAHTE MAVLINK: arac baglantisi YOK, "
                  "komutlar yalniz loglaniyor ***")
            self.menzilci = MenzilTutucu(SahteMenzil())
            return

        print(f"[tekdugum] avci baglantisi: {self.pursuer}")
        self.komutcu = HizKomutcu(self.pursuer)

        if self.menzil_kaynak_adi == 'redis':
            # Home'a ihtiyac YOK: menzil hazir sayi olarak geliyor.
            self.menzilci = MenzilTutucu(RedisMenzil(self.r))
            print("[tekdugum] menzil kaynagi: Redis 'devir_durumu'.range_m")
            return

        # Home'u AVCIDAN al (hedefin global konumunu NED'e cevirmek icin).
        # Uc deneme: yigin hizli yeniden baslatildiginda ilk istek yarisa
        # girip cevapsiz kalabiliyor (goruntulu_temel'de olculdu, 2026-08-04).
        msg = None
        for deneme in range(1, 4):
            self.komutcu.conn.mav.command_long_send(
                self.komutcu.conn.target_system,
                self.komutcu.conn.target_component,
                _mavutil().mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
            msg = self.komutcu.conn.recv_match(type='HOME_POSITION',
                                               blocking=True, timeout=10.0)
            if msg is not None:
                break
            print(f"[tekdugum] HOME_POSITION cevapsiz (deneme {deneme}/3)")
        if msg is None:
            raise SystemExit("HOME_POSITION alinamadi (3 deneme)")
        home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
        print(f"[tekdugum] home={home[0]:.7f},{home[1]:.7f} "
              f"alt={home[2]:.1f}  hedef: {self.target}")
        self.menzilci = MenzilTutucu(EstimatorMenzil(self.target, home))

    # -- Redis ilanlari ---------------------------------------------------

    def _hayatta_bildir(self):
        """OLU-ADAM ANAHTARI (TTL'li kalp atisi).

        NICIN: 2026-08-05'te goruntulu kontrolcu asili kalirken yetki ona
        devredilmisti ve araci 5.9 s boyunca KIMSE komutlamadi (menzil 40 ->
        124 m). Bu dugum yetki istemiyor ama ayni soru disaridan sorulabilir
        olmali: 'gudum hayatta mi?'. Anahtar TTL ile yazilir -> surec olurse
        kendiliginden kaybolur. CSV YOKSA SUREC HIC BASLAMAMISTIR."""
        if self.r is None:
            return
        try:
            self.r.set(ANAHTAR_HAYATTA, json.dumps({
                'ad': self.ad, 'yontem': self.k.ad, 'pid': os.getpid(),
                't_mono': time.monotonic(), 't_unix': time.time(),
                'durum': self.durum}), ex=HAYATTA_TTL_S)
        except Exception:
            pass

    def _durum_yayinla(self, menzil=None, zorla=False):
        """'tekdugum_durum': angajman durumunun TEK OTORITE kaynagi.

        TTL YOK (bilerek): gec baglanan bir okuyucu (formation_KILLER paneli,
        yer istasyonu) son durumu ANINDA gormeli. Bayatlik t_mono/t_unix ile
        ayirt edilir; 'surec yasiyor mu' sorusunun cevabi ayri anahtardadir
        (tekdugum_hayatta, TTL'li)."""
        if self.r is None:
            return
        simdi = time.monotonic()
        if not zorla and simdi - self._son_durum_yayin < 0.5:
            return
        self._son_durum_yayin = simdi
        try:
            self.r.set(ANAHTAR_DURUM, json.dumps({
                'durum': self.durum,
                'sebep': self._durum_sebep,
                'yontem': self.k.ad,
                'kural': self.kapi.mod,
                'pid': os.getpid(),
                't_mono': simdi,
                't_unix': time.time(),
                'menzil_m': None if menzil is None else float(menzil),
                'tespit_fps': round(self._yayin_fps, 2),
                'gozlem_fps': round(self.kapi.fps, 2),
                'gecerli_oran': round(self.kapi.oran, 3),
                'kuru': bool(self.kuru),
            }))
        except Exception:
            pass

    def _yetki_ilan(self, deger):
        """'komut_yetkisi' -> ILAN (bkz. __init__ yetki_yaz notu).

        UYARI: bbox_to_redis karar vericisi de bu anahtari yaziyor (yalniz
        mod DEGISIMINDE). Ikisi ayni anda kosarsa son yazan kazanir. Tek
        dugum kurulumunda bbox_to_redis SALT TESPIT rolunde kalmali."""
        if not self.yetki_yaz or self.r is None:
            return
        try:
            self.r.set(ANAHTAR_YETKI, deger)
        except Exception:
            pass

    def _statustext(self, metin):
        """MAVLink STATUSTEXT duyurusu (formation_KILLER bunu logluyor).

        Kuru modda ve --no-statustext'te gonderilmez; ekrana yine basilir --
        masa testinde 'ne duyururdum' gorunur olmali."""
        print(f"[tekdugum] >>> {metin}")
        if not self.statustext or self.kuru or self.komutcu.conn is None:
            return
        try:
            self.komutcu.conn.mav.statustext_send(
                _mavutil().mavlink.MAV_SEVERITY_INFO,
                metin.encode('ascii', 'replace')[:50])
        except Exception as exc:
            print(f"[tekdugum] STATUSTEXT gonderilemedi: {exc}")

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
        kolonlar = list(LOG_KOLONLARI) + EK_KOLONLAR
        log.writerow(kolonlar)
        olay_yolu = self.log_yolu.replace('.csv', '_olay.csv')
        if olay_yolu == self.log_yolu:
            olay_yolu = self.log_yolu + '.olay.csv'
        olay_f = open(olay_yolu, 'w', newline='')
        olay_w = csv.writer(olay_f)
        olay_w.writerow(['t', 't_unix', 'olay', 'menzil_m', 'detay'])
        izleyici = OlayIzleyici()
        turev = VelocityDifferentiator(max_history=5)
        acc_lpf = np.zeros(3)
        ACC_TAU = 0.20
        print(f"[tekdugum] log: {self.log_yolu}")
        print(f"[tekdugum] olay logu: {olay_yolu}")
        if self.kapi.buyuk_kare:
            print(f"[tekdugum] KENDI KARARIM -- BUYUK_KARE: art arda "
                  f"{self.kapi.buyuk_kare} tespit + alan "
                  f"%{self.kapi.alan_pct:g} (merkez/menzil aranmaz)")
        elif self.kapi.basit_kare:
            print(f"[tekdugum] KENDI KARARIM -- BASIT kural: art arda "
                  f"{self.kapi.basit_kare} gecerli kare "
                  f"(alan/menzil/pencere kapilari ATLANIR)")
        else:
            pencere = (f"{self.kapi.gecerli_esik}/{self.kapi.pencere_kare} kare"
                       if self.kapi.pencere_s is None else
                       f"%{100 * self.kapi.gecerli_oran:.0f} @ "
                       f"{self.kapi.pencere_s:.1f} s")
            print(f"[tekdugum] KENDI KARARIM -- UC KAPI: {pencere} + alan %"
                  f"{self.kapi.alan_pct:g} + menzil <= "
                  f"{self.kapi.menzil_esigi_m:.0f} m "
                  f"('komut_yetkisi' BEKLENMIYOR)")
        if self.kuru:
            print("[tekdugum] *** KURU KOSU (--dry-run): komutlar MAVLink'e "
                  "GITMIYOR, yalniz loglaniyor ***")

        def _olay_yaz(t_m, t_u, olaylar, menzil):
            for ad, detay in olaylar:
                olay_w.writerow([f"{t_m:.4f}", f"{t_u:.3f}", ad,
                                 '' if menzil is None else f"{menzil:.2f}",
                                 detay])
            if olaylar:
                olay_f.flush()

        baslangic = time.monotonic()
        lpf_vel = np.zeros(3)
        lpf_yaw = 0.0
        onceki_t = None
        son_istek = None
        son_birak_t = -1e9
        kayip_t0 = None              # kesintisiz kaybin baslangici
        dt_pencere = deque(maxlen=max(5, int(round(2.0 / self.loop_dt))))
        yavas_streak = 0
        son_yavas_uyari = 0.0
        onceki_ap_mod = None
        satir_sayaci = 0
        self._durum_yayinla(zorla=True)

        while self._calisiyor:
            simdi = time.monotonic()
            if sure_s is not None and simdi - baslangic > sure_s:
                break
            self._hayatta_bildir()

            # --- olculen dt (nominale GUVENILMEZ; 2 Hz'e dusen dongu +
            #     sabit dt varsayimi dev daire/titremenin koku idi) ---
            ham_dt = (self.loop_dt if onceki_t is None
                      else max(simdi - onceki_t, 1e-6))
            dt = (self.loop_dt if onceki_t is None
                  else min(max(simdi - onceki_t, 0.5 * self.loop_dt), 0.5))
            onceki_t = simdi
            dt_pencere.append(ham_dt)
            dt_asim = 1 if ham_dt > 1.5 * self.loop_dt else 0
            dongu_hz_ort = len(dt_pencere) / max(float(sum(dt_pencere)), 1e-6)
            yavas_streak = yavas_streak + 1 if dt_asim else 0
            if yavas_streak >= 10 and simdi - son_yavas_uyari > 5.0:
                son_yavas_uyari = simdi
                print(f"[tekdugum] *** DONGU YAVAS: olculen "
                      f"{dongu_hz_ort:.2f} Hz (istenen "
                      f"{1.0 / self.loop_dt:.1f} Hz) -- {yavas_streak} "
                      f"ardisik dongu. CPU yukunu azaltin ***",
                      file=sys.stderr, flush=True)

            t_unix = time.time()
            stab, bbox_yas, kapsama = self.bbox.son()
            ham_bbox, ham_yas = self.bbox.ham()
            # YEDEK YOL: stabilize kanal yok/bayat ama ham kanal tazeyse ham
            # pikselden acisal hata turet (bkz. _ham_stab uyarisi).
            if (stab is None or bbox_yas > self.bbox_bayat_s) and \
                    ham_bbox is not None and ham_yas <= self.bbox_bayat_s:
                yedek = _ham_stab(ham_bbox)
                if yedek is not None:
                    stab, bbox_yas = yedek, ham_yas
            pos, vel = self.komutcu.okuyucu.get()
            att = self.komutcu.okuyucu.get_attitude()
            vibe = self.komutcu.okuyucu.get_vibration()
            menzil = self.menzilci.menzil(pos)
            taze = stab is not None and bbox_yas <= self.bbox_bayat_s

            # Kare imzasi: AYNI karenin iki kez sayilmasini engeller (kamera
            # ~30 Hz, dongu 20 Hz). t_capture varsa o yeter; yoksa piksel +
            # boyut dortlusu pratikte benzersiz.
            imza = None
            if stab is not None:
                imza = (stab[6] if len(stab) > 6 else None,
                        stab[0], stab[1], stab[2], stab[3])
            alan_px2 = (float(stab[2]) * float(stab[3])) if stab is not None else None
            if kapsama is None and stab is not None:
                # Kapsama HAM kanaldan gelir; yoksa bbox genisliginden
                # turet (w / kadraj_genisligi * 100) -- ayni tanim.
                kapsama_kapi = 100.0 * float(stab[2]) / _KADRAJ_W
            else:
                kapsama_kapi = kapsama
            self.kapi.ornekle(simdi, imza, bbox_yas, kapsama_kapi, alan_px2,
                              menzil, self.menzilci.taze,
                              angaje=(self.durum == ANGAJE))

            # --- kayip saati ---
            if taze:
                kayip_t0 = None
            elif kayip_t0 is None:
                kayip_t0 = simdi
            kayip_s = 0.0 if kayip_t0 is None else simdi - kayip_t0

            # --- kendi ivmemiz (LOG-ONLY) ---
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
                t_capture=(float(stab[6]) if taze and len(stab) > 6
                           and stab[6] is not None else None),
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
                           if ham_bbox is not None
                           and ham_yas <= self.bbox_bayat_s else None),
                px_ham_cy=(float(ham_bbox[1]) + float(ham_bbox[3]) / 2.0
                           if ham_bbox is not None
                           and ham_yas <= self.bbox_bayat_s else None),
                acc_ned=acc_lpf.copy() if vel is not None else None,
                vibe_max=None if vibe is None else float(max(vibe)),
            )

            olaylar = []
            durum_kolonu = 'bekle'
            yaw_cmd_dps = None
            v_cmd = np.zeros(3)
            kelepce_hiz = kelepce_irtifa = kelepce_yaw = False

            # ================= ANGAJMAN DURUM MAKINESI ==================
            if self.durum == BEKLE:
                soguma_kalan = self.soguma_s - (simdi - son_birak_t)
                evet, sebep = self.kapi.angaje_olmali()
                if evet and soguma_kalan <= 0.0:
                    # --- ANGAJE OL: karari BIZ verdik ---
                    devir = self._devir_oku()
                    self.k.tohumla(devir)
                    # Komut LPF'sinin TOHUMLANMASI: angajman aninda sicrama
                    # olmasin diye (integral tohumlama). Konumlunun son
                    # komutu varsa o, yoksa KENDI olculen hizimiz -- sifir
                    # DEGIL, sifir tam fren komutudur.
                    if devir and 'cmd_vel_ned' in devir:
                        lpf_vel = np.asarray(devir['cmd_vel_ned'], dtype=float)
                    elif vel is not None:
                        lpf_vel = np.asarray(vel, dtype=float)
                    else:
                        lpf_vel = np.zeros(3)
                    lpf_yaw = 0.0
                    son_istek = None
                    onceki_t = None
                    self.durum = ANGAJE
                    self._durum_sebep = sebep
                    self.kapi.sifirla()      # olcut degisti: pencereyi tazele
                    self._yetki_ilan('goruntulu')
                    self._durum_yayinla(menzil, zorla=True)
                    self._statustext(ST_ANGAJE)
                    olaylar.append(('ANGAJE', f"{sebep} menzil={_n(menzil)} "
                                              f"tohum_hiz="
                                              f"{np.round(lpf_vel, 2).tolist()}"))

            if self.durum == ANGAJE:
                birak_sebep = ''
                if taze:
                    cmd = self.k.komut(o)
                    if getattr(cmd, 'olay', ''):
                        olaylar.append((cmd.olay, getattr(cmd, 'olay_detay', '')))
                        print(f"[tekdugum] OLAY: {cmd.olay} "
                              f"{getattr(cmd, 'olay_detay', '')}")
                    istek = np.asarray(cmd.vel_ned, dtype=float).reshape(3)
                    yaw_cmd_dps = cmd.yaw_rate_dps
                    son_istek = istek.copy()
                    durum_kolonu = 'taze'
                    if getattr(cmd, 'birak', False):
                        # ISKA: MPC'nin kendi durum makinesi "gectim/kacirdim"
                        # dedi. Yetkiyi BIZ birakiyoruz -- 'goruntulu_birak'
                        # anahtarina yazip bir karar vericinin ONAYLAMASINI
                        # BEKLEMIYORUZ (mimari farkin ta kendisi).
                        durum_kolonu = 'suz'
                        birak_sebep = f"ISKA: {getattr(cmd, 'birak_sebep', '')}"
                elif son_istek is not None and \
                        bbox_yas <= (self.bbox_bayat_s + self.bosluk_tut_s):
                    # KISA BOSLUK: son gecerli komutu TUT (yaw_rate haric --
                    # kor donus hedefi yatayda da kaybettirir).
                    istek = son_istek
                    durum_kolonu = 'tut'
                else:
                    # UZUN kayip: SIFIR DEGIL, SUZULME. Sifir "komut vermemek"
                    # degil, 18-35 m/s'den tam durusa FREN komutudur; fren ->
                    # burun yukari pitch -> kamera yukari bakar -> zaten
                    # kenardaki hedef busbutun cikar (2026-08-04 olcumu).
                    istek = (o.vel_ned.copy() if o.vel_ned is not None
                             else (son_istek if son_istek is not None
                                   else np.zeros(3)))
                    durum_kolonu = 'suz'
                    if kayip_s > self.birak_s:
                        birak_sebep = (f"bbox kaybi {kayip_s:.1f} s > "
                                       f"{self.birak_s:.1f} s")
                if not birak_sebep:
                    # Ikinci birakma yolu: TITREK tespit (pencere orani coktu).
                    kop, kop_sebep = self.kapi.birakmali(simdi)
                    if kop:
                        birak_sebep = kop_sebep

                # ---- ORTAK EMNIYET KATMANI (goruntulu_temel'den tasindi) ----
                # 1) komut LPF'si
                a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
                lpf_vel = lpf_vel + a * (istek - lpf_vel)
                # 2) hiz kelepcesi
                n = float(np.linalg.norm(lpf_vel))
                kelepce_hiz = n > self.hiz_tavani
                v_cmd = (lpf_vel * (self.hiz_tavani / n) if kelepce_hiz
                         else lpf_vel.copy())
                # 3) MUTLAK IRTIFA TABANI (2026-08-04 cakilma dersi): yontemden
                #    bagimsiz son savunma. NED'de z = -irtifa, taban:
                #    z > -irtifa_taban_m iken ALCALMA komutu iletilmez.
                kelepce_irtifa = (pos is not None
                                  and float(pos[2]) > -self.irtifa_taban_m
                                  and v_cmd[2] > 0.0)
                if kelepce_irtifa:
                    v_cmd = v_cmd.copy()
                    v_cmd[2] = 0.0
                # 4) yaw: once slew (ivme) kelepcesi, sonra hafif LPF.
                #    Olculdu (2026-08-04): ham yaw_rate 4 Hz'de +-16 dps
                #    cirpiniyordu cunku LPF yalniz hiz kanallarindaydi.
                if yaw_cmd_dps is not None:
                    sinir = self.yaw_slew_dps2 * dt
                    kirpik = float(np.clip(yaw_cmd_dps,
                                           lpf_yaw - sinir, lpf_yaw + sinir))
                    kelepce_yaw = abs(kirpik - yaw_cmd_dps) > 1e-6
                    ay = dt / (dt + self.yaw_tau_s) if self.yaw_tau_s > 1e-6 else 1.0
                    lpf_yaw += ay * (kirpik - lpf_yaw)
                    yaw_cmd_dps = lpf_yaw
                else:
                    lpf_yaw = 0.0
                    yaw_cmd_dps = None

                # ---- KOMUT ----
                # KURU KOSU: MAVLink'e YAZILMAZ ama komut TAM olarak
                # hesaplanip loglanir -- masa testinde "ne gonderirdim"
                # sorusunun cevabi CSV'de durur.
                if not self.kuru:
                    self.komutcu.hiz_gonder(
                        v_cmd,
                        None if yaw_cmd_dps is None
                        else math.radians(yaw_cmd_dps))

                # BIRAKMA en SONDA: son (suzulme) komutu gonderildikten
                # sonra. Once biraksaydik arac o dongude komutsuz kalirdi.
                if birak_sebep:
                    self._birak(birak_sebep, menzil, olaylar)
                    son_birak_t = simdi

            # ================= LOG (komut yolu BITTI) ====================
            # ref_* uretimi KOMUTTAN SONRA: "hedef telemetrisi gudume sizdi
            # mi" sorusunun cevabi kodun sirasindan bile okunabilsin.
            ref = self.menzilci.kaynak.ref_hedef_durum()
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
            kenar_px = kenar_deg = None
            if o.px_ham_cx is not None:
                kenar_px = min(o.px_ham_cx, _KADRAJ_W - o.px_ham_cx,
                               o.px_ham_cy, _KADRAJ_H - o.px_ham_cy)
                kenar_deg = math.degrees(math.atan(kenar_px / _KADRAJ_FX))
            hb = self.komutcu.okuyucu.get_heartbeat()
            ap_mod = '' if hb is None else (hb[0] or str(hb[1]))
            hb_yas_s = None if hb is None else hb[2]
            # YAYIN hizi (LOG-ONLY): bbox_to_redis kac Hz basiyor. Dongunun
            # gordugu hizdan (kapi.fps) AYRI olcusun diye ayri thread.
            yayin_fps = self._yayin_fps = self.sayaci.fps()

            kelepceler = {'kelepce_hiz': kelepce_hiz,
                          'kelepce_irtifa': kelepce_irtifa,
                          'kelepce_yaw_slew': kelepce_yaw}
            olaylar = olaylar + izleyici.izle(durum_kolonu, menzil, geo,
                                              kelepceler)
            if ap_mod and ap_mod != onceki_ap_mod:
                olaylar.append(('ap_mod_degisti',
                                f"{onceki_ap_mod or '-'} -> {ap_mod}"))
                print(f"[tekdugum] OTOPILOT MODU: "
                      f"{onceki_ap_mod or '-'} -> {ap_mod}")
                onceki_ap_mod = ap_mod
            _olay_yaz(simdi, t_unix, olaylar, menzil)
            self._durum_yayinla(menzil)

            # Satir SOZLUKTEN kurulur, indisten degil: kolon listesi
            # goruntulu_temel'den IMPORT ediliyor ve orada bir kolon
            # eklenirse burada sessizce kaymasin -- eksik ad bos kalir.
            s = {
                't': f"{simdi:.4f}", 't_mono': f"{simdi:.4f}",
                't_unix': f"{t_unix:.3f}", 'dt': f"{dt:.4f}",
                'yetki': 'goruntulu' if self.durum == ANGAJE else 'konumlu',
                'durum': durum_kolonu,
                'ex_deg': _n(o.ex_deg, '{:.4f}'), 'ey_deg': _n(o.ey_deg, '{:.4f}'),
                'bbox_w': _n(o.bbox_w, '{:.0f}'), 'bbox_h': _n(o.bbox_h, '{:.0f}'),
                'alan_kok': _n(o.alan_kok),
                'alan_px2': _n(None if o.bbox_w is None
                               else o.bbox_w * o.bbox_h, '{:.0f}'),
                'kapsama_pct': _n(o.kapsama_pct, '{:.3f}'),
                'bbox_yas_s': (f"{bbox_yas:.3f}" if math.isfinite(bbox_yas)
                               else ''),
                't_capture': _n(o.t_capture, '{:.4f}'),
                'px_sanal_x': _n(o.px_sanal_x, '{:.1f}'),
                'px_sanal_y': _n(o.px_sanal_y, '{:.1f}'),
                'px_ham_cx': _n(o.px_ham_cx, '{:.1f}'),
                'px_ham_cy': _n(o.px_ham_cy, '{:.1f}'),
                'kadraj_kenar_px': _n(kenar_px, '{:.1f}'),
                'kadraj_kenar_deg': _n(kenar_deg, '{:.2f}'),
                'ham_yas_s': f"{ham_yas:.3f}" if math.isfinite(ham_yas) else '',
                'tilt_deg': _n(o.tilt_deg, '{:.3f}'),
                'menzil_m': _n(menzil),
                'cmd_vx': f"{v_cmd[0]:.3f}", 'cmd_vy': f"{v_cmd[1]:.3f}",
                'cmd_vz': f"{v_cmd[2]:.3f}",
                'cmd_hiz_mps': f"{float(np.linalg.norm(v_cmd)):.3f}",
                'cmd_yaw_rate_dps': _n(yaw_cmd_dps),
                'kelepce_hiz': int(kelepce_hiz),
                'kelepce_irtifa': int(kelepce_irtifa),
                'kelepce_yaw_slew': int(kelepce_yaw),
                'roll_deg': _n(None if att is None else math.degrees(att[0])),
                'pitch_deg': _n(None if att is None else math.degrees(att[1])),
                'yaw_deg': _n(None if att is None else math.degrees(att[2])),
                'rota_deg': _n(biz_rota, '{:.1f}'),
                'vibe_max': '' if vibe is None else f"{max(vibe):.1f}",
                'ref_hedef_hiz_mps': _n(h_hiz),
                'ref_hedef_rota_deg': _n(h_rota, '{:.1f}'),
                'ref_hedef_donus_dps': _n(ref['donus_dps'], '{:.2f}'),
                'ref_menzil_gercek_m': _n(geo['menzil_m']),
                'ref_kerteriz_deg': _n(geo['kerteriz_deg'], '{:.1f}'),
                'ref_yukselis_deg': _n(geo['yukselis_deg'], '{:.2f}'),
                'ref_yaklasim_acisi_deg': _n(geo['yaklasim_deg'], '{:.1f}'),
                'ref_karsilasma_tipi': geo['tip'],
                'ref_kapanma_hizi_mps': _n(geo['kapanma_mps']),
                'ref_tgo_s': _n(geo['tgo_s']),
                'ref_cpa_m': _n(geo['cpa_m']), 'ref_cpa_s': _n(geo['cpa_s']),
                'olay': '|'.join(ad for ad, _ in olaylar),
                'dongu_hz_ort': f"{dongu_hz_ort:.2f}", 'dt_asim': dt_asim,
                # 'hayatta_ttl': kalp atisi TTL'i. goruntulu_temel her 10
                # donguda bir Redis'e sorup ornekliyordu; burada anahtari
                # HER dongu biz yaziyoruz, yani sabit TTL'i tekrar sormak
                # bilgi katmaz -- yazdigimiz degeri raporluyoruz.
                'hayatta_ttl': HAYATTA_TTL_S,
                'ap_mod': ap_mod, 'hb_yas_s': _n(hb_yas_s, '{:.2f}'),
                # --- tek dugume ozgu ---
                'tekdugum_durum': self.durum,
                'kural': self.kapi.mod,
                'tespit_fps': f"{yayin_fps:.2f}",
                'gozlem_fps': f"{self.kapi.fps:.2f}",
                'pencere_ornek': self.kapi.ornek,
                'gecerli_kare': self.kapi.gecerli_sayi,
                'gecerli_oran': f"{self.kapi.oran:.3f}",
                'kapi_alan': int(self.kapi.kapi_alan),
                'kapi_menzil': int(self.kapi.kapi_menzil),
                'ardisik_kare': self.kapi.ardisik_kare,
                'ardisik_buyuk_kare': self.kapi.ardisik_buyuk_kare,
                'kayip_s': f"{kayip_s:.2f}",
                'menzil_kaynak': self.menzilci.kaynak.ad,
                'menzil_taze': int(self.menzilci.taze),
                'menzil_yas_s': ('' if not math.isfinite(self.menzilci.yas_s)
                                 else f"{self.menzilci.yas_s:.2f}"),
                'kuru': int(self.kuru),
            }
            if pos is not None:
                s.update({'pos_x': f"{pos[0]:.2f}", 'pos_y': f"{pos[1]:.2f}",
                          'pos_z': f"{pos[2]:.2f}",
                          'irtifa_m': f"{-float(pos[2]):.2f}"})
            if vel is not None:
                s.update({'vel_x': f"{vel[0]:.2f}", 'vel_y': f"{vel[1]:.2f}",
                          'vel_z': f"{vel[2]:.2f}", 'hiz_mps': f"{biz_hiz:.2f}"})
            if o.acc_ned is not None:
                s.update({'acc_x_mps2': f"{o.acc_ned[0]:.2f}",
                          'acc_y_mps2': f"{o.acc_ned[1]:.2f}",
                          'acc_z_mps2': f"{o.acc_ned[2]:.2f}"})
            if ref['pos'] is not None:
                s.update({'ref_hedef_x': f"{ref['pos'][0]:.2f}",
                          'ref_hedef_y': f"{ref['pos'][1]:.2f}",
                          'ref_hedef_z': f"{ref['pos'][2]:.2f}"})
            if h_vel is not None:
                s.update({'ref_hedef_vx': f"{h_vel[0]:.2f}",
                          'ref_hedef_vy': f"{h_vel[1]:.2f}",
                          'ref_hedef_vz': f"{h_vel[2]:.2f}"})
            if ref['acc'] is not None:
                s.update({'ref_hedef_ax_mps2': f"{ref['acc'][0]:.2f}",
                          'ref_hedef_ay_mps2': f"{ref['acc'][1]:.2f}",
                          'ref_hedef_az_mps2': f"{ref['acc'][2]:.2f}"})
            log.writerow([s.get(ad, '') for ad in kolonlar])
            # PERIYODIK FLUSH: cakilmada/SIGKILL'de son saniyeler diske
            # yazilmadan kaybolmasin (20 satirda bir ~= 1 Hz).
            satir_sayaci += 1
            if satir_sayaci % 20 == 0:
                log_f.flush()

            kalan = self.loop_dt - (time.monotonic() - simdi)
            if kalan > 0:
                time.sleep(kalan)

        # --- kapanis: birakip oyle cikiyoruz (sessizce olmek yok) ---
        if self.durum == ANGAJE:
            self._birak('surec kapaniyor', None, [])
            self._hayatta_bildir()      # kalp atisinda 'durum' guncel kalsin
        log_f.close()
        olay_f.close()
        print(f"[tekdugum] dongu bitti, log kapatildi: {self.log_yolu}")

    # -- durum gecisi -----------------------------------------------------

    def _birak(self, sebep, menzil, olaylar):
        """ANGAJE -> BEKLE. Komutu KESMEZ: son suzulme komutu zaten
        gonderildi; burada yalniz ilan ve durum sifirlamasi var."""
        self.durum = BEKLE
        self._durum_sebep = sebep
        self.kapi.sifirla()
        self._yetki_ilan('konumlu')
        self._durum_yayinla(menzil, zorla=True)
        self._statustext(ST_BIRAK)
        olaylar.append(('BIRAK', sebep))
        print(f"[tekdugum] <<< BIRAK: {sebep}")

    def _devir_oku(self):
        """'devir_durumu' (konumlunun son komutu) -- VARSA warm-start tohumu.

        Yoksa sorun degil: tohum kendi olculen hizimiz olur. Bu dugum
        konumlu bir surecin varligina BAGIMLI DEGILDIR."""
        try:
            ham = self.r.get(ANAHTAR_DEVIR)
            return json.loads(ham) if ham else None
        except Exception:
            return None


# ------------------------------------------------------------------ CLI

def main(argv=None):
    p = argparse.ArgumentParser(
        description="Tek dugumlu, kendi kararini veren goruntulu gudum")
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--sure', type=float, default=None,
                   help='kosu suresi [s] (varsayilan: sinirsiz)')
    p.add_argument('--menzil-kaynak', choices=['estimator', 'redis'],
                   default=os.environ.get('YILDIZ_TEKDUGUM_MENZIL', 'estimator'),
                   help="estimator: kendi IMM'imiz (varsayilan) | "
                        "redis: 'devir_durumu'.range_m")
    # --- angajman kapisi ---
    p.add_argument('--pencere-kare', type=int, default=None,
                   help='kararlilik penceresi [KARE] (varsayilan '
                        '$YILDIZ_PENCERE_KARE ya da 25; 30 fps -> 0.83 s, '
                        '20 Hz -> 1.25 s)')
    p.add_argument('--gecerli-oran', type=float, default=None,
                   help='penceredeki asgari gecerli kare orani (varsayilan '
                        '$YILDIZ_PENCERE_ORAN ya da 0.8 -> 20/25)')
    p.add_argument('--pencere-s', type=float, default=None,
                   help='ISTEGE BAGLI: pencereyi KARE yerine SURE cinsinden '
                        'kur [s]. Verilmezse kare penceresi kullanilir.')
    p.add_argument('--basit-kare', type=int,
                   default=(int(os.environ['YILDIZ_TEKDUGUM_KARE'])
                            if os.environ.get('YILDIZ_TEKDUGUM_KARE') else None),
                   help='BASIT kural: art arda N gecerli kare yeter '
                        '(uc kapi ATLANIR). Verilmezse uc kapili kural.')
    p.add_argument('--buyuk-kare', type=int, default=None,
                   help='LOS icin kisa kapi: art arda N taze kare AYNI ZAMANDA '
                        '--alan-pct esigini gecsin; merkez ve menzil aranmaz')
    p.add_argument('--alan-pct', type=float, default=None,
                   help='alan kapisi: kadrajin (p%% x p%%) dikdortgeni '
                        '(varsayilan $YILDIZ_GECIS_ALAN_PCT ya da 2)')
    p.add_argument('--gecis-menzil', type=float, default=None,
                   help='menzil kapisi [m] (varsayilan $YILDIZ_GECIS_MENZIL '
                        'ya da 60); menzil yoksa/bayatsa kapi atlanir')
    p.add_argument('--birak-s', type=float, default=2.5,
                   help='kesintisiz bbox kaybi bu sureyi asinca BIRAK')
    p.add_argument('--soguma-s', type=float, default=3.0,
                   help='BIRAK sonrasi yeniden angajman icin asgari bekleme '
                        '(ping-pong engeli)')
    # --- ilan / kosu bicimi ---
    p.add_argument('--no-statustext', dest='statustext', action='store_false',
                   help='MAVLink STATUSTEXT duyurularini kapat')
    p.add_argument('--no-yetki-yaz', dest='yetki_yaz', action='store_false',
                   help="'komut_yetkisi' ilanini kapat (konumlu surec yoksa)")
    p.add_argument('--dry-run', dest='kuru', action='store_true',
                   help="MAVLink'e YAZMADAN tum zinciri kostur (komutlar "
                        'yalniz loglanir) -- donanim oncesi masa testi')
    p.add_argument('--sahte-mavlink', action='store_true',
                   help='arac/SITL YOKKEN kos: sahte durum + sahte menzil '
                        '(--dry-run kendiliginden acilir)')
    p.add_argument('--log', default=None, help='ana CSV yolu')
    p.add_argument('--tani-log', default=None, help='MPC tani CSV yolu')
    # --- gudum yasasi ---
    p.add_argument('--gudum', choices=['los', 'mpc'], default='los',
                   help='goruntulu tek yasa (varsayilan: los)')
    # --- MPC gecisleri (yalniz --gudum mpc) ---
    p.add_argument('--ufuk', type=int, default=None)
    p.add_argument('--adim-s', type=float, default=None)
    p.add_argument('--mount', type=float, default=None,
                   help='kamera montaj acisi [deg, YUKARI +]')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw KOMUTLAMA (yaw otopilotta kalir)')
    p.add_argument('--hiz-tavani', type=float, default=None)
    p.add_argument('--no-iska', action='store_true',
                   help='MPC ISKA durum makinesini kapat')
    # --- LOS/PN ayarlari (yalniz --gudum los) ---
    p.add_argument('--n-pn', type=float, default=4.0)
    p.add_argument('--vur-ivme', type=float, default=4.0)
    p.add_argument('--komut-ufku', type=float, default=0.70)
    p.add_argument('--terminal-menzil', type=float, default=3.0)
    p.add_argument('--terminal-tgo', type=float, default=0.25)
    a = p.parse_args(argv)

    if a.basit_kare and a.buyuk_kare:
        p.error('--basit-kare ile --buyuk-kare birlikte kullanilamaz')

    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    tani = a.tani_log or str(Path(__file__).resolve().parent / 'logs'
                             / f"tekdugum_mpc_tani_{damga}.csv")
    if a.gudum == 'los':
        kontrolcu = TerminalLosKontrolcu(
            n_pn=a.n_pn, vur_ivme_mps2=a.vur_ivme,
            komut_ufku_s=a.komut_ufku,
            terminal_menzil_m=a.terminal_menzil,
            terminal_tgo_s=a.terminal_tgo,
            yaw_komutu_ver=not a.no_yaw)
        print(f"[tekdugum] gudum yasasi: terminal LOS/PN (tek yasa) | "
              f"N={a.n_pn:g} vur_ivme={a.vur_ivme:g} "
              f"ufuk={a.komut_ufku:.2f}s "
              f"yaw={'otopilotta' if a.no_yaw else 'acik'}")
    else:
        # MPC yalniz acikca istendiginde yuklenir. Varsayilan LOS kurulumu
        # optimizasyon bagimliliklarini donanima tasimak zorunda kalmaz.
        from mpc_gudum import MpcAyar, MpcKontrolcu, _blok_uret
        ayar = MpcAyar()
        if a.ufuk is not None:
            ayar = MpcAyar(n_adim=a.ufuk, bloklar=_blok_uret(a.ufuk))
        if a.adim_s is not None:
            ayar.adim_s = a.adim_s
        if a.mount is not None:
            ayar.mount_pitch_deg = a.mount
        if a.aim is not None:
            ayar.aim_deg = a.aim
        if a.no_yaw:
            ayar.yaw_komutu_ver = False
        if a.hiz_tavani is not None:
            ayar.hiz_tavani_mps = a.hiz_tavani
        if a.no_iska:
            ayar.iska_modu = False
        kontrolcu = MpcKontrolcu(ayar, tani_log=tani)
        print(f"[tekdugum] gudum yasasi: mpc_gudum.MpcKontrolcu (IMPORT, "
              f"kopya DEGIL) | ufuk={ayar.n_adim * ayar.adim_s:.2f} s | "
              f"hiz tavani={ayar.hiz_tavani_mps:.1f} m/s")
    kapi = AngajmanKapisi(pencere_kare=a.pencere_kare,
                          gecerli_oran=a.gecerli_oran, pencere_s=a.pencere_s,
                          alan_pct=a.alan_pct, menzil_esigi_m=a.gecis_menzil,
                          basit_kare=a.basit_kare, buyuk_kare=a.buyuk_kare)
    dongu = TekDugumGudum(
        kontrolcu, kapi=kapi,
        loop_hz=a.loop_hz, menzil_kaynak=a.menzil_kaynak,
        statustext=a.statustext, kuru=a.kuru or a.sahte_mavlink,
        sahte_mavlink=a.sahte_mavlink, yetki_yaz=a.yetki_yaz,
        birak_s=a.birak_s, soguma_s=a.soguma_s, log_yolu=a.log)
    dongu.calistir(a.sure)
    # Dongu nesnesi DONER: kuru test kosumu (masa testi harness'i) komutun
    # gercekten gonderilmedigini boyle dogrular (komutcu.gonderilen == 0).
    return dongu


if __name__ == '__main__':
    main()
