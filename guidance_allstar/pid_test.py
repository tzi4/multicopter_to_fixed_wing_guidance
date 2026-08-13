#!/usr/bin/env python3
"""
pid_test.py - pid_gudum.PidKontrolcu icin CEVRIMDISI dogrulama
==============================================================
Sim, Gazebo, SITL, Redis veya MAVLink GEREKTIRMEZ. Sentetik Olcum dizileri
uretir ve kontrolcunun isaret/yon/limit/anti-windup davranisini sinar.

    python3 pid_test.py            # hepsini kos
    python3 pid_test.py -v         # ayrinti

NICIN BU TESTLER: bu projede en pahali hatalar isaret hatalari (hedefe ters
donmek), sabit-dt varsayimlari (2 Hz'e dusen donguyle dev daireler) ve
entegrator sismesiydi. Asagidaki testler tam olarak bu uc sinifi hedefler.
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from goruntulu_temel import Olcum                       # noqa: E402
from pid_gudum import PidAyar, PidKontrolcu, TurevSuzgeci   # noqa: E402


# ------------------------------------------------------------------ yardimci

def olcum_yap(t=0.0, dt=0.05, ex=0.0, ey=0.0, menzil=30.0, alan_kok=25.0,
              yaw=0.0, irtifa=50.0, vel=(18.0, 0.0, 0.0), pitch=0.0):
    """Tek bir sentetik Olcum paketi. irtifa: home'a gore metre (NED z = -h).

    alan_kok VARSAYILANI 25 px: kosu 2 kalibrasyonunda 25-35 m bandinin
    medyani (26). Yani varsayilan olcum "terminal bandda, iyi kaliteli"
    demektir ve kalite katmani (q=1) testlerin yolundan cekilir. Kalite
    davranisi ayrica kendi testlerinde kucuk alan_kok verilerek sinanir."""
    return Olcum(
        t=t, dt=dt,
        ex_deg=ex, ey_deg=ey,
        bbox_w=30.0, bbox_h=10.0,
        alan_kok=alan_kok, kapsama_pct=2.5, bbox_yas_s=0.03,
        menzil_m=menzil,
        pos_ned=np.array([0.0, 0.0, -float(irtifa)]),
        vel_ned=np.asarray(vel, dtype=float),
        yaw_rad=yaw, roll_rad=0.0, pitch_rad=pitch,
    )


def kostur(k, adimlar, dt=0.05, t0=0.0, **sabit):
    """adimlar: (ex, ey) ikilileri ya da olcum uretici. Son istegi dondurur."""
    son = None
    t = t0
    for ex, ey in adimlar:
        son = k.govde_istegi(olcum_yap(t=t, dt=dt, ex=ex, ey=ey, **sabit))
        t += dt
    return son


class PidTest(unittest.TestCase):

    def yeni(self, **ezme):
        # MONTAJ 30'A SABITLENIR (2026-08-04): sim varsayilani 0 dereceye
        # gecti (pitch-servo gimbal karari) ve kamera_montaj_deg artik
        # $YILDIZ_MOUNT'tan okunuyor. Bu paketin sayisal beklentileri
        # (ey_hedefi = -(montaj+pitch)) PID'in GELISTIRILIP DOGRULANDIGI
        # +30 geometrisine ait. PID dondurulmus kiyas artifakti; testi
        # yeni geometriye tabanlamak yerine gecerli oldugu geometriyi
        # ACIK yaziyoruz.
        ezme.setdefault('kamera_montaj_deg', 30.0)
        return PidKontrolcu(PidAyar(**ezme))

    # -- 1. ISARET DOGRULAMA ------------------------------------------------

    def test_ex_pozitif_saga_gider(self):
        """ex>0 (hedef SAGDA) -> yaw_rate>0 VE saga yanal hiz>0."""
        k = self.yeni()
        s = kostur(k, [(10.0, 0.0)] * 5)
        self.assertGreater(s['yaw_rate_dps'], 0.0)
        self.assertGreater(s['sag'], 0.0)

    def test_ex_negatif_sola_gider(self):
        k = self.yeni()
        s = kostur(k, [(-10.0, 0.0)] * 5)
        self.assertLess(s['yaw_rate_dps'], 0.0)
        self.assertLess(s['sag'], 0.0)

    def test_ey_pozitif_alcalir(self):
        """ey>0 = hedef merkezin ALTINDA (aim=0 ile: bizden ASAGIDA) -> alcal."""
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        s = kostur(k, [(0.0, 8.0)] * 5)
        self.assertGreater(s['asagi'], 0.0)

    def test_ey_negatif_tirmanir(self):
        """ey<0 = hedef YUKARIDA -> tirman (asagi hizi negatif)."""
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        s = kostur(k, [(0.0, -8.0)] * 5)
        self.assertLess(s['asagi'], 0.0)

    def test_ned_donusumu_yaw_ile(self):
        """yaw=90 deg (burun DOGU) iken ileri komut NED'de +y (dogu) olmali."""
        k = self.yeni()
        c = None
        for i in range(5):
            c = k.komut(olcum_yap(t=0.05 * i, ex=0.0, ey=0.0,
                                  yaw=math.radians(90.0)))
        self.assertGreater(c.vel_ned[1], 10.0)
        self.assertLess(abs(c.vel_ned[0]), 1e-6)

    def test_atalet_los_hizi(self):
        """PN terimi ATALET LOS hizini kullanmali: sabit donuste ex sabit
        (ex_dot~0) ama yaw doner -> yanal komut SIFIR OLMAMALI.

        Bu, kuru kosuda yakalanan hatanin regresyon testidir: ex burna gore
        oldugu icin yaw dongusu onu sifirlar ve PN terimi olurdu."""
        k = self.yeni()
        t, yaw = 0.0, 0.0
        son = None
        for _ in range(60):                       # 3 s, 8 deg/s donus
            t += 0.05
            yaw += math.radians(8.0) * 0.05
            son = k.govde_istegi(olcum_yap(t=t, ex=1.0, yaw=yaw, menzil=30.0))
        self.assertAlmostEqual(k.tani['ex_dot'], 0.0, delta=0.5)
        self.assertAlmostEqual(k.tani['lambda_dot'], 8.0, delta=1.0)
        self.assertGreater(k.tani['yanal_d'], 3.0)   # PN terimi CANLI
        self.assertGreater(son['sag'], 3.0)

    def test_lambda_dot_kelepcesi(self):
        """ATTITUDE sicramasi (yaw basamagi) yanal komutu patlatmamali."""
        k = self.yeni()
        k.govde_istegi(olcum_yap(t=0.0, yaw=0.0))
        k.govde_istegi(olcum_yap(t=0.05, yaw=math.radians(90.0)))
        self.assertLessEqual(abs(k.tani['lambda_dot']),
                             k.a.lambda_dot_max + 1e-6)

    def test_ileri_ekseni_los_azimutunda(self):
        """ileri ekseni yaw+ex olmali: yaw=0, ex=+30 -> NED y bileseni pozitif
        ve buyuk (ileri hizin sin(30)'u kadar), yalniz yanal terimden degil."""
        k = self.yeni(yanal_max=0.0)      # yanal kanali kapat -> saf eksen testi
        k.komut(olcum_yap(t=0.0, ex=30.0, yaw=0.0))
        c = k.komut(olcum_yap(t=0.05, ex=30.0, yaw=0.0))
        oran = c.vel_ned[1] / max(1e-6, np.linalg.norm(c.vel_ned))
        self.assertAlmostEqual(oran, math.sin(math.radians(30.0)), delta=0.03)

    def test_govde_ileri_ablasyonu(self):
        """--govde-ileri: ileri ekseni BURUN olmali (ex donusu yok)."""
        k = self.yeni(govde_ileri=True, yanal_max=0.0)
        k.komut(olcum_yap(t=0.0, ex=30.0, yaw=0.0))
        c = k.komut(olcum_yap(t=0.05, ex=30.0, yaw=0.0))
        self.assertLess(abs(c.vel_ned[1]), 0.2)

    # -- 2. MENZIL ZAMANLAMASI ---------------------------------------------

    def test_kazanc_menzille_buyur(self):
        """Ayni acisal hata, iki kat menzil -> ~iki kat yanal hiz istegi.
        (aci -> metre cevrimi: ofset = R*tan(e))"""
        s20 = kostur(self.yeni(), [(6.0, 0.0)] * 3, menzil=20.0)
        s40 = kostur(self.yeni(), [(6.0, 0.0)] * 3, menzil=40.0)
        self.assertGreater(s40['sag'], 1.8 * s20['sag'])
        self.assertLess(s40['sag'], 2.2 * s20['sag'])

    def test_menzil_kelepcesi(self):
        """Bozuk/asiri menzil kazanci patlatmamali (kelepce 12..80 m)."""
        s = kostur(self.yeni(), [(6.0, 0.0)] * 3, menzil=5000.0)
        s80 = kostur(self.yeni(), [(6.0, 0.0)] * 3, menzil=80.0)
        self.assertAlmostEqual(s['sag'], s80['sag'], delta=1e-6)

    def test_menzil_yoksa_varsayilan(self):
        """menzil None -> varsayilan kullanilir, cokme olmaz."""
        k = self.yeni()
        s = k.govde_istegi(olcum_yap(menzil=None))
        self.assertTrue(math.isfinite(s['ileri']))

    def test_dikey_setpoint_kamera_eksenine_bagli(self):
        """Setpoint UFKA gore sabit degil, KAMERA EKSENINE goredir:
        eksen = montaj(30) + govde pitch. Uzakta tam eksen, yakinda kesri."""
        k = self.yeni()
        # pitch = 0 -> eksen 30 deg
        self.assertAlmostEqual(k._ey_hedefi(80.0, 0.0), -30.0, delta=1e-6)
        self.assertAlmostEqual(k._ey_hedefi(12.0, 0.0),
                               -30.0 * k.a.kadraj_agirlik_yakin, delta=1e-6)
        # pitch ARTINCA eksen yukari kayar -> setpoint de buyur (daha negatif)
        self.assertLess(k._ey_hedefi(80.0, 6.8), k._ey_hedefi(80.0, 0.0))
        self.assertAlmostEqual(k._ey_hedefi(80.0, 6.8), -36.8, delta=1e-6)

    def test_duz_rota_surukleme_alcalma_komutlar(self):
        """REGRESYON (yaw-kilit kosusu geometrisi): duz rotada konumlu
        standoff'u tutamayip 41.8 m'ye surukleniyor, dikey ofset 13 m sabit
        kaliyor -> yukselis 17.3 deg, pitch +6.8 -> kamera ekseni +36.8, yani
        hedef eksenin 19.5 deg ALTINDA (kadraj kenari 20.1).

        DOGRU hamle ALCALMAKTIR (derinligi artir -> yukselisi buyut -> hedefi
        kadraj merkezine getir). Eski surum burada TIRMANIS komutluyordu."""
        k = self.yeni()
        s = kostur(k, [(0.0, -17.3)] * 20, menzil=41.8, alan_kok=9.0,
                   irtifa=60.0, pitch=math.radians(6.8))
        self.assertGreater(s['asagi'], 0.5)          # ALCALIYOR
        self.assertGreater(k.tani['ey_hedef'], -36.9)
        self.assertLess(k.tani['ey_hedef'], -25.0)   # eksene yakin setpoint

    def test_derinlik_tavani_kacak_alcalmayi_keser(self):
        """Menzil acilirken sabit aci korumak derinligi buyuturdu; tavan
        (max_derinlik_m) bunu kesmeli."""
        k = self.yeni()
        # 70 m menzil, 30 deg yukselis -> derinlik 35 m > 25 m tavan
        s = kostur(k, [(0.0, -30.0)] * 20, menzil=70.0, alan_kok=20.0,
                   irtifa=80.0)
        self.assertTrue(k.tani['derinlik_asildi'])
        self.assertLessEqual(s['asagi'], 0.0)        # alcalma kesildi

    def test_devirde_kadraj_merkezine_yaklasir(self):
        """Devir tipigi (menzil 50, ey -25, pitch 0 -> eksen +30 deg):
        hedef eksenin 5 deg ALTINDA. Dogru hamle onu eksene getirmek, yani
        HAFIF ALCALMA (derinlik artar -> yukselis buyur). Komut doygun olmamali.

        Genel invaryant: dikey komut, hedefin kamera eksenine gore acisini
        KUCULTEN yonde olmali -- bu, tespiti ayakta tutan seydir."""
        for ey, pitch_d, beklenen_alcalma in ((-25.0, 0.0, True),
                                              (-17.3, 6.8, True),
                                              (-40.0, 0.0, False)):
            k = self.yeni()
            s = kostur(k, [(0.0, ey)] * 6, menzil=50.0,
                       pitch=math.radians(pitch_d))
            eksen = 30.0 + pitch_d
            kadraj = -ey - eksen          # <0: hedef eksenin altinda
            if beklenen_alcalma:
                self.assertLess(kadraj, 0.0)
                self.assertGreater(s['asagi'], 0.0)   # alcal -> yukselis buyur
            else:
                self.assertGreater(kadraj, 0.0)
                self.assertLess(s['asagi'], 0.0)      # tirman -> yukselis kucul
            self.assertLess(abs(s['asagi']), k.a.tirmanis_max)   # doygun degil

    # -- 3. TUREV / KOSE KESME (PN terimi) ---------------------------------

    def test_turev_onculuk_katar(self):
        """Buyuyen ex (LOS donuyor) sabit ex'ten DAHA COK yanal hiz istemeli."""
        sabit = kostur(self.yeni(), [(6.0, 0.0)] * 20)
        rampa = kostur(self.yeni(), [(0.3 * i, 0.0) for i in range(1, 21)])
        # rampanin son hatasi 6.0 ile ayni; fark tamamen D (+I) terimidir
        self.assertGreater(rampa['sag'], sabit['sag'])

    def test_turev_ilk_karede_sifir(self):
        """Ilk ornekte turev 0 olmali (miras dersi: 1127-2817 deg/s sahte
        turevler)."""
        d = TurevSuzgeci(0.15)
        self.assertEqual(d.guncelle(25.0, 0.0, 0.05), 0.0)

    def test_turev_zoh_tekrarina_dayanikli(self):
        """Ayni bbox iki kez okundugunda turev TAZELENMEZ (tarak gurultusu)."""
        d = TurevSuzgeci(0.15)
        d.guncelle(0.0, 0.00, 0.05)
        v1 = d.guncelle(1.0, 0.05, 0.05)
        v2 = d.guncelle(1.0, 0.10, 0.05)   # ayni deger -> elde tut
        self.assertAlmostEqual(v1, v2, delta=1e-9)
        self.assertGreater(v1, 0.0)

    # -- 4. OLCULEN DT (dev daire kok nedeni) ------------------------------

    def test_dt_bagimsizligi(self):
        """AYNI zaman fonksiyonu 5 Hz ve 30 Hz orneklenirse komut yakin
        cikmali. Sabit-dt varsayimi olsaydi bu test PATLARDI."""
        def kos(hz):
            k = self.yeni()
            dt = 1.0 / hz
            n = int(round(2.0 * hz))
            t = 0.0
            son = None
            for i in range(1, n + 1):
                t = i * dt
                ex = 4.0 * t          # sabit LOS hizi: 4 deg/s
                son = k.govde_istegi(olcum_yap(t=t, dt=dt, ex=ex))
            return son
        a, b = kos(5.0), kos(30.0)
        self.assertAlmostEqual(a['sag'], b['sag'], delta=0.15 * abs(b['sag']) + 0.3)
        self.assertAlmostEqual(a['yaw_rate_dps'], b['yaw_rate_dps'],
                               delta=0.15 * abs(b['yaw_rate_dps']) + 1.0)

    # -- 5. LIMITLER / HIZ BUTCESI -----------------------------------------

    def test_hiz_tavani_asilmaz(self):
        """Genis bir olcum taramasinda |v| <= tavan (iskelet kelepcesi hic
        devreye girmemeli)."""
        k = self.yeni()
        tavan = k.a.v_ileri_tavan
        rng = np.random.default_rng(7)
        t = 0.0
        for _ in range(600):
            t += 0.05
            c = k.komut(olcum_yap(
                t=t, ex=float(rng.uniform(-40, 40)),
                ey=float(rng.uniform(-40, 40)),
                menzil=float(rng.uniform(8, 90)),
                yaw=float(rng.uniform(-math.pi, math.pi)),
                irtifa=float(rng.uniform(20, 120))))
            self.assertLessEqual(float(np.linalg.norm(c.vel_ned)), tavan + 1e-6)

    def test_yaw_rate_kelepcesi(self):
        k = self.yeni()
        s = kostur(k, [(90.0, 0.0)] * 20)
        self.assertAlmostEqual(abs(s['yaw_rate_dps']), k.a.yaw_rate_max,
                               delta=1e-6)

    def test_dikey_asimetrik_limit(self):
        """Alcalma <= alcalma_max, tirmanis <= tirmanis_max (otopilot
        WPNAV_SPEED_DN/UP ile hizali)."""
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        asagi = kostur(k, [(0.0, 40.0)] * 40, menzil=80.0, irtifa=200.0)['asagi']
        self.assertLessEqual(asagi, k.a.alcalma_max + 1e-6)
        k2 = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        yukari = kostur(k2, [(0.0, -40.0)] * 40, menzil=80.0)['asagi']
        self.assertGreaterEqual(yukari, -k2.a.tirmanis_max - 1e-6)

    def test_manevra_butcesi_ileriyi_korur(self):
        """En sert merkezlemede bile ileri hiz sqrt(18^2-12^2)=13.4'un
        altina inmemeli (butce paylasimi)."""
        k = self.yeni()
        s = kostur(k, [(35.0, 35.0)] * 60, menzil=80.0, irtifa=200.0)
        self.assertLessEqual(math.hypot(s['sag'], s['asagi']),
                             k.a.manevra_max + 1e-6)
        self.assertGreater(s['ileri'], 13.0)

    def test_ileri_egim_siniri(self):
        """Ileri hiz adim basina ivme_max*dt'den fazla degismemeli."""
        k = self.yeni()
        k.v_ileri = 0.0
        onceki = 0.0
        t = 0.0
        for _ in range(10):
            t += 0.05
            s = k.govde_istegi(olcum_yap(t=t, dt=0.05))
            self.assertLessEqual(s['ileri'] - onceki,
                                 k.a.ileri_ivme_mps2 * 0.05 + 1e-9)
            onceki = s['ileri']

    def test_taban_tavana_esit(self):
        """VARSAYILANDA tam gaz: hedef bizden hizli, gonullu yavaslamak
        kalici kapanma kaybidir (kosu 2 olcumu: 16 m/s taban -> menzil
        29 m'den 95 m'ye acildi ve yetki geri dondu)."""
        k = self.yeni()
        self.assertAlmostEqual(k.a.v_ileri_taban, k.a.v_ileri_tavan, delta=1e-9)
        s = kostur(k, [(0.0, 0.0)] * 120, menzil=60.0, alan_kok=6.0)
        self.assertGreater(s['ileri'], 17.9)      # uzakta bile tam gaz

    def test_commit_rampasi(self):
        """Taban ELLE dusurulurse rampa calismali (deney dugmesi)."""
        def son_ileri(menzil, alan):
            k = self.yeni(v_ileri_taban=12.0)
            k.v_ileri = 10.0
            return kostur(k, [(0.0, 0.0)] * 60, menzil=menzil, alan_kok=alan)['ileri']
        uzak = son_ileri(60.0, 6.0)
        yakin = son_ileri(18.0, 6.0)
        self.assertGreater(yakin, uzak + 1.0)
        # alan_kok tek basina da commit tetikleyebilmeli (menzil bayat)
        k = self.yeni(v_ileri_taban=12.0)
        k.v_ileri = 10.0
        alanla = kostur(k, [(0.0, 0.0)] * 60, menzil=None, alan_kok=30.0)['ileri']
        self.assertGreater(alanla, uzak + 1.0)

    # -- 5b. OLCUM KALITESI (kucuk / bayat bbox) ---------------------------

    def test_kucuk_bbox_pn_terimini_kisar(self):
        """alan_kok kucukse (duz rotanin kuyruk goruntusu, ~8 px) D terimi
        kisilmali; P terimi AYNI kalmali."""
        def kos(alan):
            k = self.yeni()
            t, yaw = 0.0, 0.0
            for _ in range(60):
                t += 0.05
                yaw += math.radians(10.0) * 0.05      # sabit LOS donusu
                k.govde_istegi(olcum_yap(t=t, ex=6.0, yaw=yaw, menzil=55.0,
                                         alan_kok=alan))
            return k.tani
        buyuk = kos(20.0)      # q_alan = 1
        kucuk = kos(7.0)       # q_alan = 0.125
        self.assertGreater(buyuk['q'], 0.9)
        self.assertLess(kucuk['q'], 0.2)
        # D kisildi, P degismedi
        self.assertLess(abs(kucuk['yanal_d']), 0.35 * abs(buyuk['yanal_d']))
        self.assertAlmostEqual(kucuk['yanal_p'], buyuk['yanal_p'], delta=1e-6)

    def test_dusuk_kalitede_turev_daha_agir_suzulur(self):
        """q dustukce turev zaman sabiti buyumeli (SNR dusuk -> cok ortalama)."""
        k = self.yeni()
        k.govde_istegi(olcum_yap(t=0.05, alan_kok=20.0))
        iyi = k.tani['turev_tau']
        k2 = self.yeni()
        k2.govde_istegi(olcum_yap(t=0.05, alan_kok=5.0))
        kotu = k2.tani['turev_tau']
        self.assertAlmostEqual(iyi, k.a.turev_tau_s, delta=1e-6)
        self.assertAlmostEqual(kotu, k.a.turev_tau_s * 3.0, delta=1e-6)

    def test_bayat_bbox_kaliteyi_dusurur(self):
        """bbox_yas buyudukce q dusmeli (tespit kesintisi)."""
        k = self.yeni()
        taze = olcum_yap(t=0.05, alan_kok=20.0); taze.bbox_yas_s = 0.03
        k.govde_istegi(taze); q_taze = k.tani['q']
        k2 = self.yeni()
        bayat = olcum_yap(t=0.05, alan_kok=20.0); bayat.bbox_yas_s = 0.60
        k2.govde_istegi(bayat); q_bayat = k2.tani['q']
        self.assertGreater(q_taze, 0.9)
        self.assertAlmostEqual(q_bayat, k.a.yas_kalite_taban, delta=1e-6)
        self.assertGreater(q_bayat, 0.0)   # tamamen olmesin: biraz onculuk kalsin

    def test_dusuk_kalitede_entegrator_donar(self):
        """Gurultulu/bayat olcum kalici yanlilik biriktirmemeli."""
        k = self.yeni()
        kostur(k, [(5.0, 5.0)] * 100, alan_kok=5.0)   # q ~ 0 -> dondur
        self.assertAlmostEqual(k.i_yanal.deger, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_dikey.deger, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_yaw.deger, 0.0, delta=1e-6)

    def test_pn_mutlak_kelepcesi(self):
        """Ne olursa olsun D terimi pn_max'i asmamali."""
        k = self.yeni()
        t, yaw = 0.0, 0.0
        for _ in range(80):
            t += 0.05
            yaw += math.radians(70.0) * 0.05          # cok hizli LOS donusu
            k.govde_istegi(olcum_yap(t=t, ex=8.0, yaw=yaw, menzil=80.0,
                                     alan_kok=40.0))
        self.assertLessEqual(abs(k.tani['yanal_d']), k.a.pn_max_mps + 1e-6)

    def test_terminal_bandda_pn_yetkisi_tam(self):
        """EN ONEMLI KALITE INVARYANTI: carpismanin olacagi bandda q=1 olmali.

        kosu 2 kalibrasyonu: 25-35 m -> alan_kok medyan 26, 15-25 m -> 36.
        Kalite katmani gurultuyu uzak menzilde bastirmali ama terminal
        fazda PN terimini KISMAMALI, yoksa kose kesme kabiliyeti gider."""
        k = self.yeni()
        for alan in (26.0, 36.0):
            k.govde_istegi(olcum_yap(t=0.05, alan_kok=alan, menzil=30.0))
            self.assertAlmostEqual(k.tani['q'], 1.0, delta=1e-6)
            self.assertAlmostEqual(k.tani['turev_tau'], k.a.turev_tau_s,
                                   delta=1e-6)

    def test_kalite_ileri_hizi_etkilemez(self):
        """En kotu olcumde bile kapanma surmeli: ileri kanal q'dan bagimsiz."""
        k = self.yeni()
        s = kostur(k, [(0.0, 0.0)] * 120, alan_kok=5.0, menzil=55.0)
        self.assertGreater(s['ileri'], 17.9)

    # -- 6. ANTI-WINDUP ----------------------------------------------------

    def test_kapi_disinda_entegre_etmez(self):
        """|hata| > int_kapi_deg iken entegrator sismemeli."""
        k = self.yeni()
        kostur(k, [(30.0, 30.0)] * 200)     # 10 s boyunca kadraj kenari
        self.assertAlmostEqual(k.i_yanal.deger, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_dikey.deger, 0.0, delta=1e-6)

    def test_devirdeki_buyuk_ey_entegratore_girmez(self):
        """Devir tipigi ey=-25 (setpoint sonrasi ~-9) hizli tirmanista
        doygunlasir; entegrator sismemeli (katki tavani asilmamali)."""
        k = self.yeni()
        kostur(k, [(0.0, -25.0)] * 200, menzil=50.0)
        self.assertLessEqual(abs(k.i_dikey.ki * k.i_dikey.deger),
                             k.a.int_katki_max + 1e-6)

    def test_doygunlukta_entegrator_sonumlenir(self):
        """Kanal kelepcedeyken biriken entegrator sonumlenmeli."""
        k = self.yeni()
        kostur(k, [(5.0, 0.0)] * 60)                    # kapi icinde: biriksin
        birikmis = abs(k.i_yanal.deger)
        self.assertGreater(birikmis, 0.0)
        k._doygun_yanal = True
        kostur(k, [(0.0, 0.0)] * 20, t0=3.1)            # doygun + hatasiz
        self.assertLess(abs(k.i_yanal.deger), birikmis)

    def test_bbox_kaybi_sonrasi_yeniden_kilit(self):
        """komut() cagrilari arasinda uzun bosluk -> tam sifirlama."""
        k = self.yeni()
        kostur(k, [(5.0, 0.0)] * 60)
        self.assertGreater(abs(k.i_yanal.deger), 0.0)
        k.govde_istegi(olcum_yap(t=100.0, ex=5.0))       # 97 s bosluk
        # Sifirlandiktan sonra yalniz TEK adim birikti: (hata - olu bant)*dt
        beklenen = (5.0 - k.a.olu_bant_deg) * 0.05
        self.assertAlmostEqual(k.i_yanal.deger, beklenen, delta=1e-3)
        self.assertAlmostEqual(k.d_ex._suzulmus, 0.0, delta=1e-9)

    def test_olu_bant(self):
        """Olu bant icindeki kucuk hata komut uretmemeli (limit cevrimi)."""
        k = self.yeni()
        s = kostur(k, [(0.1, 0.1)] * 10,
                   menzil=30.0)
        self.assertAlmostEqual(s['yaw_rate_dps'], 0.0, delta=1e-6)

    # -- 7. DEVIR TOHUMLAMASI ----------------------------------------------

    def test_tohum_ilk_komutu_esitler(self):
        """tohumla(devir) sonrasi ILK komut ~ konumlunun son komutu olmali,
        dikey hata buyuk olsa BILE (kalinti temelli tohumlama)."""
        devir = {'cmd_vel_ned': [17.0, 2.5, -1.5], 'cmd_yaw_rad': 0.0,
                 'range_m': 45.0}
        k = self.yeni()
        k.tohumla(devir)
        s = k.govde_istegi(olcum_yap(t=0.0, dt=0.05, ex=0.0, ey=-25.0,
                                     menzil=45.0))
        self.assertAlmostEqual(s['sag'], 2.5, delta=0.05)
        self.assertAlmostEqual(s['asagi'], -1.5, delta=0.05)
        self.assertAlmostEqual(s['ileri'], 17.0, delta=0.4)

    def test_tohum_soner(self):
        """Tohum tau_tohum ile sifira gitmeli (kalici yanlilik birakmaz)."""
        devir = {'cmd_vel_ned': [17.0, 6.0, -4.0], 'cmd_yaw_rad': 0.0,
                 'range_m': 45.0}
        k = self.yeni()
        k.tohumla(devir)
        kostur(k, [(0.0, -25.0)] * 200, menzil=45.0)     # 10 s
        self.assertLess(abs(k.tohum_sag), 0.05)
        self.assertLess(abs(k.tohum_asagi), 0.05)

    def test_tohum_yaw_ile_dondurulur(self):
        """cmd_yaw_rad=90 deg iken NED [0,17,0] govdede ILERI 17 olmali."""
        devir = {'cmd_vel_ned': [0.0, 17.0, 0.0],
                 'cmd_yaw_rad': math.radians(90.0), 'range_m': 40.0}
        k = self.yeni()
        k.tohumla(devir)
        self.assertAlmostEqual(k.v_ileri, 17.0, delta=1e-6)
        self.assertAlmostEqual(k._tohum_istegi[0], 0.0, delta=1e-6)

    def test_devir_yoksa_cokmez(self):
        k = self.yeni()
        k.tohumla(None)
        k.tohumla({})
        k.tohumla({'cmd_vel_ned': None, 'cmd_yaw_rad': None, 'range_m': 'x'})
        s = k.govde_istegi(olcum_yap())
        self.assertTrue(math.isfinite(s['ileri']))

    # -- 8. EMNIYET / DAYANIKLILIK -----------------------------------------

    def test_irtifa_tabani_alcalmayi_engeller(self):
        """Irtifa tabaninin altinda alcalma komutlanmamali."""
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        s = kostur(k, [(0.0, 30.0)] * 20, irtifa=8.0)
        self.assertLessEqual(s['asagi'], 0.0)
        self.assertLess(s['asagi'], 0.0)   # ihlal -> yumusak tirmanis

    def test_irtifa_tabani_entegratoru_dondurur(self):
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        kostur(k, [(0.0, 6.0)] * 100, irtifa=8.0)
        self.assertAlmostEqual(k.i_dikey.deger, 0.0, delta=1e-6)

    def test_yaw_yoksa_sifir_komut(self):
        """Attitude hic gelmediyse NED'e cevrilemez -> sifir komut."""
        k = self.yeni()
        o = olcum_yap(yaw=None)
        c = k.komut(o)
        self.assertAlmostEqual(float(np.linalg.norm(c.vel_ned)), 0.0, delta=1e-9)
        self.assertIsNone(c.yaw_rate_dps)

    def test_yaw_kaybolursa_son_yaw_tutulur(self):
        k = self.yeni()
        k.komut(olcum_yap(t=0.0, yaw=math.radians(45.0)))
        c = k.komut(olcum_yap(t=0.05, yaw=None))
        self.assertGreater(float(np.linalg.norm(c.vel_ned)), 1.0)

    def test_nan_ve_none_dayanikliligi(self):
        k = self.yeni()
        o = olcum_yap(menzil=None, alan_kok=None)
        o.ex_deg = None
        o.ey_deg = None
        c = k.komut(o)
        self.assertTrue(np.all(np.isfinite(c.vel_ned)))

    def test_sifir_hata_sifir_manevra(self):
        """Hedef tam merkezde ve durgunsa: yalniz ileri hiz."""
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        s = kostur(k, [(0.0, 0.0)] * 40)
        self.assertAlmostEqual(s['sag'], 0.0, delta=1e-6)
        self.assertAlmostEqual(s['asagi'], 0.0, delta=1e-6)
        self.assertAlmostEqual(s['yaw_rate_dps'], 0.0, delta=1e-6)
        self.assertGreater(s['ileri'], 15.0)

    # -- 9. KAPALI DONGU (kinematik kuru kosu) -----------------------------

    def test_kapali_dongu_yatay_yakinsar(self):
        """Basit 2B kinematik: yanal hiz LOS acisini kapatmali.

        Model: sabit menzil R, d(ex)/dt = -v_sag/R (rad). Bu, kontrolcunun
        varsaydigi kinematigin ta kendisidir; test kazanc/isaret/kelepce
        zincirinin gercekten kapali dongu kararli oldugunu gosterir."""
        k = self.yeni()
        R = 30.0
        ex = 20.0
        dt = 0.05
        t = 0.0
        gecmis = []
        for _ in range(400):                              # 20 s
            t += dt
            s = k.govde_istegi(olcum_yap(t=t, dt=dt, ex=ex, menzil=R))
            # yaw_rate kadraji cevirir, yanal hiz LOS'u cevirir; ikisi de ex'i
            # kuculten yonde (basitlestirilmis: yalniz yanal hiz).
            ex += math.degrees(-s['sag'] / R) * dt
            gecmis.append(ex)
        self.assertLess(abs(gecmis[-1]), 1.0)
        self.assertLess(max(gecmis[200:]), 2.0)           # asim/limit cevrimi yok

    def test_kapali_dongu_dikey_yakinsar(self):
        k = self.yeni(kadraj_agirlik_uzak=0.0, kadraj_agirlik_yakin=0.0)
        R = 30.0
        ey = -20.0
        dt = 0.05
        t = 0.0
        for _ in range(600):
            t += dt
            s = k.govde_istegi(olcum_yap(t=t, dt=dt, ey=ey, menzil=R,
                                         irtifa=80.0))
            ey += math.degrees(-s['asagi'] / R) * dt
        self.assertLess(abs(ey), 1.5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
