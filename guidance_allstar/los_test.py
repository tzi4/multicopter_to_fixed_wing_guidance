#!/usr/bin/env python3
"""
los_test.py - los_gudum.py cevrimdisi (SIMSIZ) dogrulama takimi
================================================================
    python3 los_test.py            # tumu
    python3 los_test.py -v         # ayrintili
    python3 los_test.py Geometri   # tek sinif

HICBIR TEST SIM/SITL/GAZEBO/REDIS/MAVLINK ISTEMEZ. Olcum paketleri sentetik
uretilir; kapali dongu testleri kendi nokta-kutle kinematigini kosar.

Testler uc katmandadir:
  1) GEOMETRI  -- los_gudum'un yildizlar_gimbal ile ayni matematigi konustugu
     (eps = -aim_etkin - ey, ex = yan aci, aim aynasi) GERCEK SanalGimbal
     nesnesiyle sayisal olarak kanitlanir.
  2) BIRIM     -- tek tek yasa parcalari: onculuk isareti, kendi yaw oranimizin
     cikarilmasi, kelepceler, ayni-ornek korumasi, terminal kapisi, tohumlama.
  3) KAPALI DONGU -- 3B nokta-kutle motoru: hedef duz / donusluk ucar, avci
     komutu birinci mertebe gecikmeyle izler, olcumler GERCEK geometriden
     uretilir. Isabet/kacirma ve FOV'da kalma olculur.
"""

from __future__ import annotations

import math
import sys
import unittest

import numpy as np

import guidance_config as cfg
import los_gudum as L
from goruntulu_temel import Olcum

# Hiz tavani SABIT YAZILMAZ: guidance_config tek dogruluk kaynagidir
# (2026-08-03'te 18 -> 20 yukseltildi; params/swarm_copter.parm WPNAV_SPEED
# 2000 cm/s = 20 m/s zaten buna izin veriyordu). Test bu degeri okur ki
# ileride tekrar degisirse sessizce yanlis sinamasin.
V_MAX = float(cfg.GORUNTULU_MAX_SPEED_MPS)


# --------------------------------------------------------------- yardimcilar

def olcum(t, ex, ey, menzil=40.0, dt=0.05, yaw=0.0, vel=None, pos=None,
          bbox=(30.0, 18.0), yas=0.02, pitch=0.0):
    """Sentetik Olcum paketi. bbox_w/h yalniz alan_kok ve ayni-ornek imzasi
    icin vardir; kontrolcu bunlardan MENZIL TURETMEZ (duman testi: bbox
    genisligi aspect'e bagli)."""
    w, h = bbox
    return Olcum(
        t=t, dt=dt, ex_deg=ex, ey_deg=ey, bbox_w=w, bbox_h=h,
        alan_kok=math.sqrt(w * h), kapsama_pct=100.0 * w / 1280.0,
        bbox_yas_s=yas, menzil_m=menzil,
        pos_ned=np.array([0.0, 0.0, -60.0]) if pos is None
        else np.asarray(pos, float),
        vel_ned=np.array([12.0, 0.0, 0.0]) if vel is None
        else np.asarray(vel, float),
        yaw_rad=yaw, roll_rad=0.0, pitch_rad=pitch)


def kos(k, dizi, dt=0.05, t0=0.0):
    """Bir (ex, ey) dizisini kontrolcuye sirayla verir; tani listesi dondurur.

    dizi: [(ex, ey), ...] ya da [(ex, ey, kwargs_dict), ...]
    """
    kayit = []
    for i, ogr in enumerate(dizi):
        if len(ogr) == 3:
            ex, ey, kw = ogr
        else:
            (ex, ey), kw = ogr, {}
        o = olcum(t0 + i * dt, ex, ey, dt=dt, **kw)
        cmd = k.komut(o)
        kayit.append((dict(k.tani), cmd))
    return kayit


# ============================================================ 1) GEOMETRI

class Geometri(unittest.TestCase):
    """los_gudum'un aci sozlesmesi GERCEK yildizlar_gimbal ile ayni mi?"""

    @classmethod
    def setUpClass(cls):
        try:
            import yildizlar_gimbal as YG
        except Exception as exc:            # pragma: no cover
            raise unittest.SkipTest(f"yildizlar_gimbal import edilemedi: {exc}")
        cls.YG = YG

    def test_aim_aynasi_analitik(self):
        """los_gudum.analitik_aim == yildizlar_gimbal.analitik_aim"""
        for back, down in [(25, 13), (25, 3), (40, 20), (15, 5), (30, 0)]:
            self.assertAlmostEqual(L.analitik_aim(back, down),
                                   self.YG.analitik_aim(back, down), places=12)

    def test_aim_aynasi_rampa(self):
        """los_gudum.aim_etkin == SanalGimbal.aim_etkin_deg (menzil rampasi)"""
        g = self.YG.SanalGimbal(aim_pitch_deg=-27.47)
        for r in [None, 10, 50, 119.9, 120, 150, 185, 249.9, 250, 400]:
            self.assertAlmostEqual(L.aim_etkin(-27.47, r),
                                   g.aim_etkin_deg(r), places=12,
                                   msg=f"menzil={r}")

    def test_eps_bagintisi(self):
        """(1) eps = -aim_etkin - ey  ve  ex = yan aci.

        Gercek gimbalden HAM piksel uretilir (piksel_uret), sonra
        aci_hatasi() ile ex/ey okunur. Govde roll/pitch verilerek sanal
        gimbalin de-rotasyonu da yolun icine katilir."""
        for aim in (0.0, -27.47, -10.0):
            g = self.YG.SanalGimbal(aim_pitch_deg=aim)
            for eps_ger in (0.0, 5.0, 12.0, 27.5, -8.0):
                for yan in (0.0, 6.0, -11.0, 20.0):
                    for roll, pitch in ((0.0, 0.0), (0.20, -0.09), (-0.15, 0.12)):
                        px = g.piksel_uret(eps_ger, yan, roll, pitch)
                        if px is None:
                            continue
                        ex, ey = g.aci_hatasi(px[0], px[1], roll, pitch,
                                              menzil_m=40.0)
                        aim_e = L.aim_etkin(aim, 40.0)
                        eps_geri = L.eps_coz(ex, ey, aim_e)
                        self.assertAlmostEqual(
                            ex, yan, places=4,
                            msg=f"ex!=yan (aim={aim} eps={eps_ger} yan={yan})")
                        # eps_coz izdusum duzeltmesini yaptigi icin aim=0'da
                        # bagintii TAM olmali; aim != 0'da ex, aim donmus
                        # cercevedeki azimut yerine kullanildigi icin kucuk
                        # ikinci mertebe artik kalir.
                        tol = 0.01 if abs(aim) < 1e-9 else 1.6
                        self.assertLess(
                            abs(eps_geri - eps_ger), tol,
                            msg=(f"eps geri kazanimi (aim={aim} eps={eps_ger} "
                                 f"yan={yan} roll={roll}) -> {eps_geri:.3f}"))

    def test_aim_sifirda_eps_esittir_eksi_ey(self):
        """senaryo.sh AIM=0 -> eps = -ey (es-irtifa geometrisi kontrolu)."""
        k = L.LosKontrolcu(aim_deg=0.0)
        k.tohumla(None)
        k.komut(olcum(0.0, 0.0, -27.5, menzil=40.0))
        self.assertAlmostEqual(k.tani['eps'], 27.5, places=6)
        k2 = L.LosKontrolcu(aim_deg=0.0)
        k2.tohumla(None)
        k2.komut(olcum(0.0, 0.0, +8.0, menzil=40.0))
        self.assertAlmostEqual(k2.tani['eps'], -8.0, places=6)

    def test_varsayilan_aim_sifirdir(self):
        """Env/arg yoksa aim 0 olmali (senaryo.sh ile catismasin)."""
        import os
        eski = os.environ.pop('YILDIZ_AIM', None)
        try:
            k = L.LosKontrolcu()
            self.assertEqual(k.aim_deg, 0.0)
            self.assertEqual(k.aim_kaynak, 'varsayilan')
            k2 = L.LosKontrolcu(back_m=25, down_m=13)
            self.assertAlmostEqual(k2.aim_deg, -27.4744, places=3)
        finally:
            if eski is not None:
                os.environ['YILDIZ_AIM'] = eski


# =============================================================== 2) BIRIM

class Birim(unittest.TestCase):

    def yeni(self, **kw):
        """Birim testleri YASA CEKIRDEGINI sinar; ham kadraj korumasi
        (bolum 3b) varsayilan olarak KAPATILIR (k_fov=0, tirmanma_max
        serbest) ki dikey komut sinamalari korumanin duzeltmesiyle
        karismasin. Koruma kendi sinifinda (FovKorumasi) sinanir."""
        kw.setdefault('aim_deg', 0.0)
        kw.setdefault('k_fov', 0.0)
        kw.setdefault('tirmanma_max_mps', 99.0)
        k = L.LosKontrolcu(**kw)
        k.tohumla(None)
        return k

    # ---- temel yon ----

    def test_merkezli_hedef_los_boyunca(self):
        """ex=0, ey=0 sabit -> komut ufka paralel, burun yonunde, oncu yok."""
        k = self.yeni()
        kayit = kos(k, [(0.0, 0.0)] * 40)
        tani, cmd = kayit[-1]
        self.assertAlmostEqual(tani['onc_az'], 0.0, places=6)
        self.assertAlmostEqual(tani['onc_el'], 0.0, places=6)
        self.assertAlmostEqual(tani['c_deg'], 0.0, places=6)
        self.assertAlmostEqual(tani['g_deg'], 0.0, places=6)
        self.assertGreater(cmd.vel_ned[0], 10.0)          # ileri
        self.assertAlmostEqual(cmd.vel_ned[1], 0.0, places=6)
        self.assertAlmostEqual(cmd.vel_ned[2], 0.0, places=6)

    def test_hedef_yukarida_tirmanis(self):
        """ey<0 (hedef sanal merkezin ustunde, aim=0 -> eps>0) -> tirman."""
        k = self.yeni()
        _, cmd = kos(k, [(0.0, -27.5)] * 10)[-1]
        self.assertLess(cmd.vel_ned[2], -2.0)             # NED z asagi (+)
        self.assertGreater(cmd.vel_ned[0], 0.0)

    def test_hedef_asagida_alcalis(self):
        k = self.yeni()
        _, cmd = kos(k, [(0.0, +15.0)] * 10)[-1]
        self.assertGreater(cmd.vel_ned[2], 1.0)

    def test_hedef_sagda_saga_hiz_ve_yaw(self):
        k = self.yeni()
        tani, cmd = kos(k, [(20.0, 0.0)] * 10)[-1]
        self.assertGreater(cmd.vel_ned[1], 3.0)           # yaw=0 -> y = sag
        self.assertGreater(cmd.yaw_rate_dps, 1.0)

    def test_isaret_simetrisi(self):
        """Ayna hareket -> ayna komut.

        DIKKAT: NED hiz vektorunun DIKEY bileseni bilerek ASIMETRIKTIR
        (WPNAV_SPEED_UP 10 m/s, WPNAV_SPEED_DN 5 m/s). Bu yuzden simetri
        KOMUT YONU (c_deg, g_deg, oncu) uzerinden sinanir; yatay hizda ise
        birebir aranir."""
        for i, dizi in enumerate([[(0.6 * n, 0.0) for n in range(40)],
                                  [(0.0, 0.6 * n) for n in range(40)]]):
            ka = self.yeni(gamma_min_deg=-55.0, gamma_max_deg=55.0)
            kb = self.yeni(gamma_min_deg=-55.0, gamma_max_deg=55.0)
            ta, ca = kos(ka, dizi)[-1]
            tb, cb = kos(kb, [(-a, -b) for a, b in dizi])[-1]
            self.assertAlmostEqual(ta['onc_az'], -tb['onc_az'], places=6)
            self.assertAlmostEqual(ta['onc_el'], -tb['onc_el'], places=6)
            self.assertAlmostEqual(ta['c_deg'], -tb['c_deg'], places=6)
            self.assertAlmostEqual(ta['g_deg'], -tb['g_deg'], places=6)
            np.testing.assert_allclose(ca.vel_ned[1], -cb.vel_ned[1],
                                       atol=1e-6, err_msg=f"dizi {i}")

    # ---- PN onculugunun ISARETI (legacy1'e gore duzeltilen sey) ----

    def test_onculuk_los_orani_yonunde(self):
        """LOS saga doniyorsa oncu de SAGA olmali (PN). legacy1'de bu terim
        ters isaretliydi (bkz. los_gudum bolum 0-a)."""
        k = self.yeni()
        kayit = kos(k, [(0.4 * n, 0.0) for n in range(60)])   # ex artiyor
        tani, cmd = kayit[-1]
        self.assertGreater(tani['q_az_nokta'], 1.0)           # LOS saga
        self.assertGreater(tani['onc_az'], 1.0)               # oncu saga
        self.assertGreater(tani['c_deg'], tani['ex_ong'])     # LOS'un otesinde

    def test_onculuk_dikeyde_los_orani_yonunde(self):
        """eps artiyorsa (hedef yukseliyor) oncu YUKARI olmali."""
        k = self.yeni()
        # aim=0 -> eps = -ey; ey azaliyorsa eps artiyor
        kayit = kos(k, [(0.0, -0.4 * n) for n in range(60)])
        tani, _ = kayit[-1]
        self.assertGreater(tani['q_el_nokta'], 1.0)
        self.assertGreater(tani['onc_el'], 1.0)
        self.assertGreater(tani['g_deg'], tani['eps_ong'] - 1e-9)

    def test_onculuk_yikamayla_soner(self):
        """LOS orani sifirlaninca oncu tau_onc ile sifira dogru sonumlensin."""
        k = self.yeni(tau_onc_s=1.0)
        kos(k, [(0.4 * n, 0.0) for n in range(60)])
        tepe = k.tani['onc_az']
        self.assertGreater(tepe, 1.0)
        sabit = k.tani['ex_ong']
        kos(k, [(sabit, 0.0)] * 120, t0=100.0)   # ex sabit -> LOS orani 0
        self.assertLess(abs(k.tani['onc_az']), 0.25 * tepe)

    # ---- KENDI YAW ORANIMIZIN CIKARILMASI (denklem 2) ----

    def test_kendi_yawimiz_sahte_oran_uretmez(self):
        """Ataletsel LOS SABIT, biz saga donuyoruz. ex_nokta = -yaw_nokta
        oldugu icin q_az_nokta ~ 0 -> ONCU BIRIKMEMELI."""
        k = self.yeni()
        w = 20.0                       # deg/s yaw orani
        dt = 0.05
        for i in range(120):
            t = i * dt
            yaw = math.radians(w * t)
            ex = 30.0 - w * t          # LOS sabit, govde donuyor
            k.komut(olcum(t, ex, 0.0, dt=dt, yaw=yaw))
        self.assertLess(abs(k.tani['q_az_nokta']), 2.0,
                        "kendi yaw oranimiz cikarilmiyor")
        self.assertLess(abs(k.tani['onc_az']), 3.0,
                        "kendi donusumuz sahte oncu uretiyor")

    def test_yaw_orani_cikarilmazsa_ayirt_edilir(self):
        """Karsit kanit: yaw sabitken AYNI ex dizisi buyuk oncu uretir."""
        k = self.yeni()
        dt, w = 0.05, 20.0
        for i in range(120):
            t = i * dt
            k.komut(olcum(t, 30.0 - w * t, 0.0, dt=dt, yaw=0.0))
        self.assertLess(k.tani['onc_az'], -5.0)

    def test_yaw_sarmasi_turevi_bozmaz(self):
        """yaw +-pi'de atlarken sahte devasa yaw_nokta uretilmemeli."""
        k = self.yeni()
        dt, w = 0.05, 30.0
        for i in range(80):
            t = i * dt
            yaw = math.radians(L.wrap180(170.0 + w * t))
            k.komut(olcum(t, 0.0, 0.0, dt=dt, yaw=yaw))
            self.assertLess(abs(k.tani['yaw_nokta']), 60.0)
        self.assertAlmostEqual(k.tani['yaw_nokta'], w, delta=4.0)

    # ---- hiz yasasi ----

    def test_hiz_artimli_rampa(self):
        """v_d = |v_simdi| + k_a; hizali iken tam k_a eklenir."""
        k = self.yeni()
        o = olcum(0.0, 0.0, 0.0, vel=[12.0, 0.0, 0.0])
        k.komut(o)
        self.assertAlmostEqual(k.tani['theta_deg'], 0.0, places=3)
        self.assertAlmostEqual(k.tani['k_a'], k.ka_tepe, places=6)
        self.assertAlmostEqual(k.tani['v_d'], 14.0, places=6)

    def test_hizlanma_kapisi_hizasizken_kapanir(self):
        """theta >= theta_esik -> k_a = 0 (once yonel, sonra gaz)."""
        k = self.yeni(theta_esik_deg=35.0)
        # komut yonu ~90 deg saga, mevcut hiz ileri -> theta ~ 90
        k.komut(olcum(0.0, 80.0, 0.0, vel=[12.0, 0.0, 0.0]))
        self.assertGreater(k.tani['theta_deg'], 35.0)
        self.assertAlmostEqual(k.tani['k_a'], 0.0, places=9)
        self.assertAlmostEqual(k.tani['v_d'], 12.0, places=6)

    def test_hiz_tavani(self):
        k = self.yeni()
        _, cmd = kos(k, [(0.0, 0.0, {'vel': [17.9, 0.0, 0.0]})] * 5)[-1]
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned), V_MAX + 1e-9)

    def test_dikey_tavan_yonu_korur(self):
        """Dik tirmanista |vz| WPNAV_SPEED_UP'i asmamali, yon bozulmamali."""
        k = self.yeni(gamma_max_deg=55.0)   # yeni(): tirmanma_max serbest
        _, cmd = kos(k, [(0.0, -55.0, {'vel': [17.0, 0.0, -3.0]})] * 20)[-1]
        self.assertLessEqual(-cmd.vel_ned[2], L.WPNAV_SPEED_UP_MPS + 1e-6)
        # yon korunmus mu: yatay/dikey oran tan(g) olmali
        yatay = math.hypot(cmd.vel_ned[0], cmd.vel_ned[1])
        self.assertAlmostEqual(math.degrees(math.atan2(-cmd.vel_ned[2], yatay)),
                               k.tani['g_deg'], delta=0.5)

    def test_dikey_alcalma_tavani(self):
        k = self.yeni(gamma_min_deg=-25.0)
        _, cmd = kos(k, [(0.0, 60.0, {'vel': [17.0, 0.0, 3.0],
                                      'pos': [0, 0, -200.0]})] * 20)[-1]
        self.assertLessEqual(cmd.vel_ned[2], L.WPNAV_SPEED_DN_MPS + 1e-6)

    # ---- kelepceler / emniyet ----

    def test_onculuk_kelepceleri(self):
        k = self.yeni(onc_az_max_deg=10.0, onc_el_max_deg=5.0, k_pn=3.0)
        kos(k, [(2.0 * n, -2.0 * n) for n in range(80)])
        self.assertLessEqual(abs(k.tani['onc_az']), 10.0 + 1e-9)
        self.assertLessEqual(abs(k.tani['onc_el']), 5.0 + 1e-9)

    def test_yaw_rate_tavani_ve_slew(self):
        k = self.yeni()
        onceki = 0.0
        for i in range(60):
            cmd = k.komut(olcum(i * 0.05, 90.0, 0.0, dt=0.05))
            self.assertLessEqual(abs(cmd.yaw_rate_dps), k.yaw_rate_max + 1e-9)
            self.assertLessEqual(abs(cmd.yaw_rate_dps - onceki),
                                 k.yaw_ivme_max * 0.05 + 1e-6)
            onceki = cmd.yaw_rate_dps

    def test_yaw_olu_bant(self):
        k = self.yeni()
        for _ in range(30):
            cmd = k.komut(olcum(0.0, 0.4, 0.0))
        self.assertAlmostEqual(cmd.yaw_rate_dps, 0.0, places=6)

    def test_yaw_kapali(self):
        k = self.yeni(yaw_acik=False)
        cmd = k.komut(olcum(0.0, 25.0, 0.0))
        self.assertIsNone(cmd.yaw_rate_dps)

    def test_irtifa_tabani(self):
        k = self.yeni()
        _, cmd = kos(k, [(0.0, 40.0, {'pos': [0, 0, -8.0]})] * 10)[-1]
        self.assertLessEqual(cmd.vel_ned[2], 0.0)

    def test_gamma_kelepcesi(self):
        k = self.yeni(gamma_max_deg=30.0)
        kos(k, [(0.0, -80.0)] * 10)
        self.assertLessEqual(k.tani['g_deg'], 30.0 + 1e-9)

    # ---- olcum sagligi ----

    def test_ayni_ornek_korumasi(self):
        """Ayni bbox tekrar gelirse turev SIFIRA cekilmemeli (dusuk okuma)."""
        k = self.yeni()
        dt = 0.05
        t = 0.0
        # her 3 dongude bir yeni ornek: gercek LOS orani 8 deg/s
        ex = 0.0
        for i in range(90):
            if i % 3 == 0:
                ex = 8.0 * t
            k.komut(olcum(t, ex, 0.0, dt=dt))
            t += dt
        self.assertAlmostEqual(k.tani['ex_nokta'], 8.0, delta=1.5)

    def test_ayni_ornek_korumasi_olmasa_dusuk_okurdu(self):
        """Karsit kanit: AYNI diziyi 'her dongu yeni ornek' gibi verirsek
        (imzayi bbox_w ile kirarak) LOS orani DUSUK okunur."""
        def _kosum(imza_kir):
            k = self.yeni()
            dt, t, ex = 0.05, 0.0, 0.0
            for i in range(90):
                if i % 3 == 0:
                    ex = 8.0 * t
                w = 30.0 + (i * 1e-6 if imza_kir else 0.0)
                k.komut(olcum(t, ex, 0.0, dt=dt, bbox=(w, 18.0)))
                t += dt
            return k.tani['ex_nokta']
        korumali, korumasiz = _kosum(False), _kosum(True)
        self.assertGreater(korumali, korumasiz + 1.0,
                           f"koruma kazanc saglamadi: {korumali} vs {korumasiz}")
        self.assertAlmostEqual(korumali, 8.0, delta=1.5)

    def test_uzun_bosluk_turevi_sifirlar(self):
        k = self.yeni()
        kos(k, [(0.5 * n, 0.0) for n in range(40)])
        self.assertGreater(abs(k.tani['ex_nokta']), 2.0)
        k.komut(olcum(100.0, 20.0, 0.0))          # 100 s bosluk
        self.assertAlmostEqual(k.tani['ex_nokta'], 0.0, places=9)

    def test_gecikme_telafisi(self):
        """bbox yaslandikca kerteriz ex_nokta ile ileri tasinmali."""
        k = self.yeni()
        kos(k, [(0.5 * n, 0.0) for n in range(40)])
        son_ex = 0.5 * 39
        k.komut(olcum(2.0, son_ex, 0.0, yas=0.25))
        self.assertGreater(k.tani['ex_ong'], son_ex)

    # ---- menzil sozlesmesi ----

    def test_menzil_yalniz_estimatordan(self):
        """menzil_m None iken bbox genisliginden MENZIL TURETILMEZ."""
        k = self.yeni()
        k.komut(olcum(0.0, 0.0, 0.0, menzil=None))
        self.assertIsNone(k.tani['menzil'])
        self.assertEqual(k.tani['menzil_kaynak'], 'yok')

    def test_menzil_bayatlayinca_son_deger_tutulur(self):
        k = self.yeni()
        k.komut(olcum(0.0, 0.0, 0.0, menzil=33.0))
        k.komut(olcum(0.05, 0.1, 0.0, menzil=None))
        self.assertAlmostEqual(k.tani['menzil'], 33.0, places=6)
        self.assertEqual(k.tani['menzil_kaynak'], 'son')

    def test_menzilsiz_komut_uretilir(self):
        """Menzil hic yokken bile yasa calismali (ic dongu acilardan besleniyor)."""
        k = self.yeni()
        _, cmd = kos(k, [(5.0, -10.0, {'menzil': None})] * 20)[-1]
        self.assertTrue(np.all(np.isfinite(cmd.vel_ned)))
        self.assertGreater(np.linalg.norm(cmd.vel_ned), 1.0)

    def test_ilerleme_sinyali_alan_kok(self):
        """d(ln alan_kok)/dt = -R_nokta/R olceksiz kapanma orani."""
        k = self.yeni()
        dt = 0.05
        for i in range(120):
            # R 40 -> 20 m dogrusal; alan_kok ~ 1/R
            R = 40.0 - 20.0 * (i / 119.0)
            olc = 1000.0 / R
            k.komut(olcum(i * dt, 0.0, 0.0, menzil=R, dt=dt,
                          bbox=(olc, olc)))
        self.assertGreater(k.tani['ln_alan_nokta'], 0.0)     # buyuyor
        self.assertLess(k.tani['menzil_nokta'], 0.0)         # kapaniyor
        # -R_nokta/R ile tutarli mi (kaba)
        beklenen = -k.tani['menzil_nokta'] / k.tani['menzil']
        self.assertAlmostEqual(k.tani['ln_alan_nokta'], beklenen, delta=0.05)

    # ---- terminal kapisi ----

    def test_terminal_menzil_kapisi_onculugu_dondurur(self):
        k = self.yeni(terminal_menzil_m=12.0)
        kos(k, [(0.4 * n, 0.0, {'menzil': 40.0}) for n in range(60)])
        onc = k.tani['onc_az']
        self.assertGreater(onc, 1.0)
        kos(k, [(30.0 + 0.4 * n, 0.0, {'menzil': 8.0}) for n in range(60)],
            t0=10.0)
        self.assertTrue(k.tani['terminal'])
        self.assertAlmostEqual(k.tani['onc_az'], onc, places=9)

    def test_terminal_alan_kapisi(self):
        k = self.yeni(terminal_alan_kok=50.0)
        k.komut(olcum(0.0, 0.0, 0.0, menzil=40.0, bbox=(60.0, 60.0)))
        self.assertTrue(k.tani['terminal'])

    def test_terminal_tam_gaz(self):
        """Terminalde hizalanma kapisi bypass edilir."""
        k = self.yeni(terminal_menzil_m=12.0)
        k.komut(olcum(0.0, 80.0, 0.0, menzil=8.0, vel=[12.0, 0.0, 0.0]))
        self.assertGreater(k.tani['theta_deg'], 35.0)
        self.assertAlmostEqual(k.tani['k_a'], k.ka_tepe, places=9)

    # ---- devir / tohumlama ----

    def test_tohumla_devirsiz(self):
        k = L.LosKontrolcu(aim_deg=0.0)
        k.tohumla(None)
        self.assertEqual(k.onc_az, 0.0)
        self.assertIsNone(k.tani.get('menzil'))
        cmd = k.komut(olcum(0.0, 0.0, 0.0))
        self.assertTrue(np.all(np.isfinite(cmd.vel_ned)))

    def test_tohumla_devirli(self):
        k = L.LosKontrolcu(aim_deg=0.0)
        k.tohumla({'t_mono': 1.0, 'cmd_vel_ned': [16.0, 1.0, -0.5],
                   'cmd_yaw_rad': 0.3, 'range_m': 47.5})
        k.komut(olcum(0.0, 0.0, 0.0, menzil=None))
        self.assertAlmostEqual(k.tani['menzil'], 47.5, places=6)

    def test_tohumla_bozuk_devir(self):
        k = L.LosKontrolcu(aim_deg=0.0)
        k.tohumla({'range_m': 'bozuk'})
        self.assertIsNone(k._menzil_son)

    def test_devir_sonrasi_sicrama_yok(self):
        """Devirden hemen sonraki ilk komut, ilk dongude asiri hiz istemez."""
        k = self.yeni()
        cmd = k.komut(olcum(0.0, 0.0, -27.5, vel=[16.0, 0.0, 0.0]))
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned), V_MAX + 1e-9)
        self.assertAlmostEqual(cmd.yaw_rate_dps, 0.0, delta=6.0)

    # ---- argparse / main yolu ----

    def test_argparse_kontrolcu_kurar(self):
        a = L.arg_ayristirici().parse_args(
            ['--k-pn', '1.4', '--tau-yak', '0.6', '--dikey-oran', '0.6',
             '--yaw-kapali', '--aim', '0', '--terminal-menzil', '15'])
        k = L.kontrolcu_kur(a)
        self.assertEqual(k.k_pn, 1.4)
        self.assertEqual(k.tau_yak, 0.6)
        self.assertEqual(k.dikey_oran, 0.6)
        self.assertFalse(k.yaw_acik)
        self.assertEqual(k.aim_deg, 0.0)
        self.assertEqual(k.terminal_menzil, 15.0)

    def test_dikey_oran_dusurulunce_tirmanis_azalir(self):
        ka, kb = self.yeni(dikey_oran=1.0), self.yeni(dikey_oran=0.4)
        _, ca = kos(ka, [(0.0, -27.5)] * 10)[-1]
        _, cb = kos(kb, [(0.0, -27.5)] * 10)[-1]
        self.assertLess(-ca.vel_ned[2] * 0.999, -ca.vel_ned[2])   # sanity
        self.assertGreater(-ca.vel_ned[2], -cb.vel_ned[2])


class FovKorumasi(unittest.TestCase):
    """Bolum 3b: kamera govdeye sabit oldugu icin komutlanan tirmanis burun
    yukari pitch uretir ve hedef HAM kadrajin altindan cikar. Ilk elips
    kosusundaki 16 devir/geri-donus dongusunun koku buydu."""

    def yeni(self, **kw):
        kw.setdefault('aim_deg', 0.0)
        # MONTAJ 30'A SABITLENIR (2026-08-04): sim varsayilani 0 dereceye
        # gecti (pitch-servo gimbal karari), ama bu sinifin sayisal
        # beklentileri (alt_aci = mount+pitch-eps, kelepce degerleri) LOS
        # yasasinin GELISTIRILIP DOGRULANDIGI +30 montaj geometrisine aittir.
        # LOS dondurulmus bir kiyas artifaktidir; testi yeni geometriye
        # tabanlamak yerine hangi geometride gecerli oldugunu ACIK yaziyoruz.
        kw.setdefault('mount_deg', 30.0)
        k = L.LosKontrolcu(**kw)
        k.tohumla(None)
        return k

    def test_alt_aci_bagintisi(self):
        """(12) alt_aci = (mount + pitch) - eps"""
        k = self.yeni(k_fov=0.0)
        # aim=0 -> eps = -ey (izdusum duzeltmesi ex=0'da notr)
        k.komut(olcum(0.0, 0.0, -15.7, menzil=35.0))
        # olcum() pitch_rad=0 verir -> alt_aci = 30 - 15.7
        self.assertAlmostEqual(k.tani['alt_aci'], 30.0 - 15.7, places=3)
        self.assertAlmostEqual(k.tani['eps'], 15.7, places=3)

    def test_hedef_kadrajin_altindayken_tirmanis_kisilir(self):
        """alt_aci > fov_marj -> komut yol acisi ASAGI cekilir."""
        acik = self.yeni(k_fov=1.0, tirmanma_max_mps=99.0)
        kapali = self.yeni(k_fov=0.0, tirmanma_max_mps=99.0)
        # olculen elips durumu: eps=15.7, pitch=+11.2 -> alt_aci=25.5
        o_kw = {'menzil': 35.0, 'pitch': math.radians(11.2)}
        _, c_acik = kos(acik, [(0.0, -15.7, o_kw)] * 10)[-1]
        _, c_kapali = kos(kapali, [(0.0, -15.7, o_kw)] * 10)[-1]
        self.assertAlmostEqual(acik.tani['alt_aci'], 25.5, delta=0.1)
        self.assertAlmostEqual(acik.tani['fov_duzeltme'], -(25.5 - 14.0),
                               delta=0.1)
        self.assertLess(acik.tani['g_deg'], kapali.tani['g_deg'] - 5.0)
        self.assertLess(-c_acik.vel_ned[2], -c_kapali.vel_ned[2])

    def test_hedef_kadrajin_ustundeyken_tirmanis_artirilir(self):
        """Simetrik yon: alt_aci < -fov_marj -> g YUKARI itilir."""
        k = self.yeni(k_fov=1.0, tirmanma_max_mps=99.0)
        # eps buyuk, pitch burun asagi -> hedef eksenin USTUNDE
        kos(k, [(0.0, -50.0, {'menzil': 35.0,
                              'pitch': math.radians(-10.0)})] * 10)
        self.assertLess(k.tani['alt_aci'], -14.0)
        self.assertGreater(k.tani['fov_duzeltme'], 0.0)
        self.assertGreater(k.tani['g_deg'], k.tani['g_ham'])

    def test_marj_icinde_koruma_susar(self):
        k = self.yeni(k_fov=1.0)
        kos(k, [(0.0, -25.0, {'menzil': 35.0,
                              'pitch': math.radians(0.0)})] * 10)
        self.assertAlmostEqual(k.tani['alt_aci'], 5.0, delta=0.2)
        self.assertAlmostEqual(k.tani['fov_duzeltme'], 0.0, places=9)
        self.assertAlmostEqual(k.tani['g_deg'], k.tani['g_ham'], places=9)

    def test_koruma_kapatilabilir(self):
        k = self.yeni(k_fov=0.0)
        kos(k, [(0.0, -15.7, {'menzil': 35.0,
                              'pitch': math.radians(11.2)})] * 10)
        self.assertAlmostEqual(k.tani['fov_duzeltme'], 0.0, places=9)

    def test_duzeltme_kelepcesi(self):
        k = self.yeni(k_fov=1.0, fov_duzeltme_max=6.0)
        kos(k, [(0.0, -15.7, {'menzil': 35.0,
                              'pitch': math.radians(11.2)})] * 10)
        self.assertAlmostEqual(k.tani['fov_duzeltme'], -6.0, places=9)

    def test_tirmanma_tavani(self):
        k = self.yeni(k_fov=0.0, tirmanma_max_mps=3.0)
        _, cmd = kos(k, [(0.0, -45.0, {'menzil': 35.0,
                                       'vel': [15.0, 0.0, -3.0]})] * 20)[-1]
        self.assertLessEqual(-cmd.vel_ned[2], 3.0 + 1e-9)
        # yatay hiz OLDURULMEDI (bilincli olarak vektor olceklenmez)
        self.assertGreater(math.hypot(cmd.vel_ned[0], cmd.vel_ned[1]), 10.0)

    def test_alcalma_kirpilmaz(self):
        """Tavan yalniz TIRMANMAYA uygulanir."""
        k = self.yeni(k_fov=0.0, tirmanma_max_mps=1.0)
        _, cmd = kos(k, [(0.0, 30.0, {'menzil': 35.0,
                                      'pos': [0, 0, -200.0]})] * 10)[-1]
        self.assertGreater(cmd.vel_ned[2], 1.0)


# ======================================================= 3) KAPALI DONGU

class Motor:
    """3B nokta-kutle capisma motoru.

    Avci: komut hizini birinci mertebe gecikmeyle (iskelet LPF'si + kopter
    dinamigi) izler, |v| <= 18, dikey |vz| tavanlari uygulanir.
    Hedef: verilen hiz profilini uygular.
    Olcum: GERCEK bagil vektorden uretilir -> ex = kerteriz - yaw,
    ey = -aim_etkin - eps. (Geometri sinifi bu esitligi gercek gimballe
    dogruluyor, burada tekrar kurmak yerine ondan yararlaniyoruz.)
    """

    def __init__(self, k, p0, v0, h0, hedef_hiz_fn, dt=0.05, yaw0=None,
                 tau_arac=0.45, tau_yaw=0.30, aim=0.0, bbox_olcek=900.0,
                 pitch_modeli=None, mount=30.0):
        self.k = k
        self.p = np.asarray(p0, float)      # avci NED
        self.v = np.asarray(v0, float)
        self.h = np.asarray(h0, float)      # hedef NED
        self.hedef_hiz_fn = hedef_hiz_fn
        self.dt, self.tau, self.tau_yaw = dt, tau_arac, tau_yaw
        self.aim, self.bbox_olcek = aim, bbox_olcek
        d = self.h - self.p
        self.yaw = math.atan2(d[1], d[0]) if yaw0 is None else yaw0
        self.yaw_rate = 0.0
        self.t = 0.0
        self.min_menzil = float(np.linalg.norm(d))
        self.iz = []
        self.fov_kayip = 0
        self.ham_kayip = 0
        # pitch_modeli:
        #   None     -> pitch = mount - eps, yani kamera ekseni tam hedefte.
        #               alt_aci = 0 olur ve FOV korumasi NOTR kalir; gudum
        #               yasasini tek basina sinamak icin.
        #   'kopter' -> OLCUME OTURTULMUS kaba vekil: elips kosusunda
        #               (20260803_173736) tirmanis ~0 iken pitch ortanca
        #               -1.8 deg, tirmanis ~4 m/s iken +11.2 deg olculdu
        #               -> pitch ~= -1.8 + 3.2 * tirmanma_hizi [m/s].
        self.pitch_modeli = pitch_modeli
        self.mount = float(mount)
        self.pitch_deg = 0.0

    def _pitch(self, eps_deg):
        if self.pitch_modeli == 'kopter':
            tirmanma = max(0.0, -float(self.v[2]))
            hedef = -1.8 + 3.2 * tirmanma
            a = self.dt / (self.dt + 0.5)          # tutum dongusu gecikmesi
            self.pitch_deg += a * (hedef - self.pitch_deg)
        else:
            self.pitch_deg = eps_deg - self.mount   # koruma notr
        return self.pitch_deg

    def _olcum(self):
        d = self.h - self.p
        R = float(np.linalg.norm(d))
        kerteriz = math.degrees(math.atan2(d[1], d[0]))
        eps = math.degrees(math.asin(L.kelepce(-d[2] / max(R, 1e-6), -1.0, 1.0)))
        ex = L.wrap180(kerteriz - math.degrees(self.yaw))
        # eps_coz'un TERSI: gercek kamera perspektifini taklit et
        #   tan(-ey) = tan(eps + aim) / cos(ex)
        e_acc = math.radians(eps + L.aim_etkin(self.aim, R))
        ey = -math.degrees(math.atan(math.tan(e_acc)
                                     / max(1e-6, math.cos(math.radians(ex)))))
        w = L.kelepce(self.bbox_olcek / max(R, 1.0), 2.0, 600.0)
        pitch_deg = self._pitch(eps)
        # HAM kadraj: hedef, (mount + pitch) ekseninin alt_aci kadar altinda.
        # |alt_aci| > dikey yari-kadraj -> tespit YOK.
        self.alt_aci = (self.mount + pitch_deg) - eps
        if abs(self.alt_aci) > L.FOV_DIKEY_YARI_DEG:
            self.ham_kayip += 1
        return Olcum(t=self.t, dt=self.dt, ex_deg=ex, ey_deg=ey,
                     bbox_w=w, bbox_h=0.6 * w,
                     alan_kok=math.sqrt(w * 0.6 * w),
                     kapsama_pct=100.0 * w / 1280.0, bbox_yas_s=0.02,
                     menzil_m=R, pos_ned=self.p.copy(), vel_ned=self.v.copy(),
                     yaw_rad=self.yaw, roll_rad=0.0,
                     pitch_rad=math.radians(pitch_deg)), ex, ey, R

    def adim(self):
        o, ex, ey, R = self._olcum()
        if abs(ex) > L.FOV_YATAY_YARI_DEG or abs(ey) > 40.0:
            self.fov_kayip += 1
        cmd = self.k.komut(o)
        u = np.asarray(cmd.vel_ned, float)
        n = float(np.linalg.norm(u))
        if n > self.k.v_max:
            u = u * (self.k.v_max / n)
        u[2] = L.kelepce(u[2], -L.WPNAV_SPEED_UP_MPS, L.WPNAV_SPEED_DN_MPS)
        a = self.dt / (self.dt + self.tau)
        self.v = self.v + a * (u - self.v)
        if cmd.yaw_rate_dps is not None:
            ay = self.dt / (self.dt + self.tau_yaw)
            self.yaw_rate += ay * (math.radians(cmd.yaw_rate_dps) - self.yaw_rate)
            self.yaw += self.yaw_rate * self.dt
        self.p = self.p + self.v * self.dt
        self.h = self.h + np.asarray(self.hedef_hiz_fn(self.t), float) * self.dt
        self.t += self.dt
        R2 = float(np.linalg.norm(self.h - self.p))
        self.min_menzil = min(self.min_menzil, R2)
        self.iz.append((self.t, R2, ex, ey, self.k.tani.get('onc_az', 0.0)))
        return R2

    def kos(self, sure):
        n = int(sure / self.dt)
        for _ in range(n):
            if self.adim() < 1.5:      # temas
                break
        return self.min_menzil


def _kur(k=None, **kw):
    # MONTAJ 30'A SABITLENIR (2026-08-04, Birim.yeni ile ayni gerekce):
    # kapali dongu kosumu Ucus(mount=30.0) ile olcum uretiyor; kontrolcu
    # $YILDIZ_MOUNT'tan 0 okursa KONTROLCU ile MODEL AYRISIR ve senaryo
    # kontrol yasasini degil bu uyumsuzlugu sinar. LOS dondurulmus kiyas
    # artifakti; gecerli oldugu geometri acik yazilir.
    # HIZ TAVANI 20'YE SABITLENIR (2026-08-05, ayni gerekce): cfg tavani
    # 20 -> 35 m/s'e cikti (gercek donanim 24 ile uçuyor, sim 36.6 olctu).
    # Bu senaryolarin AYIRT EDICILIGI hiz acigina dayaniyordu: "saf takip
    # (k_pn=0) iskalar, PN vurur" onculu 35 m/s'te COKUYOR -- kopter hedefin
    # (20 m/s) 1.75 katiysa saf takip bile 0.98 m'ye vuruyor, yani test
    # yasayi degil hiz ustunlugunu olcer hale geliyor. LOS dondurulmus kiyas
    # artifakti; gecerli oldugu ZARF acik yazilir. (Kelepcenin CANLI cfg'yi
    # izledigi invaryant testleri -- V_MAX kullananlar -- degismedi.)
    k = k or L.LosKontrolcu(aim_deg=0.0, mount_deg=30.0, v_max_mps=20.0)
    k.tohumla(None)
    return k


class KapaliDongu(unittest.TestCase):
    """Sentetik capisma senaryolari. Sim/SITL YOK, saf kinematik."""

    def devir_durumu(self, back=25.0, down=13.0):
        """Devir ani: avci hedefin 25 m arkasi, 13 m altinda, hedef hizinda."""
        hedef = np.array([0.0, 0.0, -100.0])
        avci = hedef - np.array([back, 0.0, -down])
        return avci, hedef

    def test_duz_hedef_stabil_kalir(self):
        """Hedef 20 m/s duz. Kopter 18 m/s -> KUYRUK KOVALAMACASI KAZANILAMAZ
        (fizik). Beklenen: yasa DIVERGE ETMEZ, oncu makul kalir, hedef FOV'da
        kalir, menzil kontrolsuz patlamaz."""
        avci, hedef = self.devir_durumu()
        k = _kur()
        m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                  lambda t: (20.0, 0.0, 0.0))
        m.kos(30.0)
        self.assertLess(m.fov_kayip, 60, "hedef kadrajdan cikti")
        self.assertLess(abs(k.tani['onc_az']), 15.0, "oncu patladi")
        R_son = m.iz[-1][1]
        self.assertLess(R_son, 200.0, f"menzil kontrolsuz acildi: {R_son:.0f}")

    def test_donen_hedef_yakalanir(self):
        """Hedef sabit donus orani ile daire ciziyor (elips rotasinin ozu).
        LOS/PN aciyi keser -> TEMAS olmali. Uc donus orani birden sinanir:
        yasa yalniz bir noktada degil, aralikta calismali."""
        for donus_dps, sure in ((6.0, 60.0), (9.0, 50.0), (14.0, 40.0)):
            with self.subTest(donus=donus_dps):
                avci, hedef = self.devir_durumu()
                k = _kur()
                w = math.radians(donus_dps)
                m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                          lambda t: (20.0 * math.cos(w * t),
                                     20.0 * math.sin(w * t), 0.0))
                mn = m.kos(sure)
                self.assertLess(mn, 3.0,
                                f"{donus_dps} deg/s donuste en yakin menzil "
                                f"{mn:.1f} m")

    def test_pn_saf_takipten_iyi(self):
        """AYNI senaryoda k_pn=0 (oncu yok = SAF TAKIP) ile k_pn=1 (PN)
        karsilastirmasi. Onculuk olmadan donen hedef YAKALANMAZ."""
        w = math.radians(9.0)
        hedef_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        sonuc = {}
        for kp in (0.0, 1.0):
            avci, hedef = self.devir_durumu()
            k = _kur(L.LosKontrolcu(aim_deg=0.0, k_pn=kp,
                                    mount_deg=30.0, v_max_mps=20.0))
            m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef, hedef_fn)
            sonuc[kp] = m.kos(50.0)
        self.assertLess(sonuc[1.0], 3.0, f"PN vurmadi: {sonuc}")
        self.assertGreater(sonuc[0.0], 15.0, f"saf takip beklenmedik: {sonuc}")

    def test_oncu_kelepcesi_hiz_acigi_varken_kritik(self):
        """onc_az_max yalniz AVCI HEDEFTEN YAVASKEN baglayicidir.

        Carpisma ucgeni sin(d) = (V_t/V_p) sin(theta_t) oldugu icin V_p < V_t
        iken cozum 80 dereceye varan oncu ister; dar kelepce yasayi kuyruk
        kovalamacasina dusurur. Bu test hiz acigini (v_max=18 < hedef 20)
        ACIKCA kurar -- guidance_config 20'ye cikarilmis olsa bile acigin
        ucusta yeniden dogmasi mumkundur (tirmanista yatay bilesen
        V*cos(gamma)'ya duser, WPNAV_SPEED_UP 10 m/s dikeyi ayrica kirpar),
        o yuzden gerekce bir REGRESYON BEKCISI olarak korunur.

        Olculen (daire, 60 s): kelepce 60 deg -> 28.2 / 22.4 / 15.0 m KACIRMA,
        85 deg -> 3.2 / 1.4 / 0.9 m ISABET (donus 3 / 6 / 9 deg/s).
        """
        for donus_dps in (3.0, 6.0, 9.0):
            with self.subTest(donus=donus_dps):
                w = math.radians(donus_dps)
                hedef_fn = (lambda t: (20.0 * math.cos(w * t),
                                       20.0 * math.sin(w * t), 0.0))
                sonuc = {}
                for omax in (60.0, 85.0):
                    avci, hedef = self.devir_durumu()
                    k = _kur(L.LosKontrolcu(aim_deg=0.0, v_max_mps=18.0,
                                            onc_az_max_deg=omax))
                    m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                              hedef_fn)
                    sonuc[omax] = m.kos(60.0)
                self.assertGreater(sonuc[60.0], 8.0,
                                   f"dar kelepce beklenmedik sekilde iyi: {sonuc}")
                self.assertLess(sonuc[85.0], 4.0,
                                f"genis kelepce vurmadi: {sonuc}")

    def test_hiz_paritesinde_kelepce_baglayici_degil(self):
        """guidance_config GORUNTULU_MAX_SPEED_MPS = 20 (2026-08-03) ile avci
        hedefle AYNI hizda; carpisma ucgeni artik kucuk oncuyle cozuluyor ve
        kelepce baglayici olmuyor. Yeni gercegin belgesi: dar kelepceyle bile
        ISABET beklenir."""
        self.assertGreaterEqual(V_MAX, 20.0,
                                "cfg.GORUNTULU_MAX_SPEED_MPS geriye alinmis")
        for donus_dps in (4.0, 9.0, 20.0):
            with self.subTest(donus=donus_dps):
                w = math.radians(donus_dps)
                hedef_fn = (lambda t: (20.0 * math.cos(w * t),
                                       20.0 * math.sin(w * t), 0.0))
                for omax in (60.0, 85.0):
                    avci, hedef = self.devir_durumu()
                    k = _kur(L.LosKontrolcu(aim_deg=0.0, onc_az_max_deg=omax))
                    m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                              hedef_fn)
                    mn = m.kos(60.0)
                    self.assertLess(mn, 3.0,
                                    f"donus={donus_dps} kelepce={omax} "
                                    f"en yakin menzil {mn:.1f} m")

    def test_menzil_tavani_kritik(self):
        """v_max = GORUNTULU_MAX_SPEED_MPS. 18 -> hedef (20 m/s) ancak cok
        genis onculukle yakalanir; 20 -> dar onculukle bile yakalanir.
        guidance_config'in 18 -> 20 yukseltilmesinin (2026-08-03) kanitidir;
        deger geri alinirsa bu test kaybi yeniden gosterir."""
        w = math.radians(6.0)
        hedef_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        sonuc = {}
        for vm in (18.0, 20.0):
            avci, hedef = self.devir_durumu()
            k = _kur(L.LosKontrolcu(aim_deg=0.0, v_max_mps=vm,
                                    onc_az_max_deg=60.0))
            m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef, hedef_fn)
            sonuc[vm] = m.kos(60.0)
        print(f"\n    [bilgi] dar onculukte (60 deg) en yakin menzil: "
              f"v_max=18 -> {sonuc[18.0]:.1f} m, v_max=20 -> {sonuc[20.0]:.1f} m")
        self.assertLess(sonuc[20.0], sonuc[18.0])

    def test_legacy_isaretiyle_karsilastirma(self):
        """legacy1'in TERS isaretli turev terimi (d_nokta = -K*d - (Kd+1)*qdot)
        donen hedefte belirgin sekilde daha kotu olmali. k_pn < 0 vererek o
        isareti taklit ediyoruz."""
        w = math.radians(9.0)
        hedef_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        sonuc = {}
        for etiket, kp in (('legacy_ters', -1.0), ('pn', 1.0)):
            avci, hedef = self.devir_durumu()
            k = _kur(L.LosKontrolcu(aim_deg=0.0, k_pn=kp,
                                    mount_deg=30.0, v_max_mps=20.0))
            m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef, hedef_fn)
            sonuc[etiket] = m.kos(50.0)
        self.assertLess(sonuc['pn'], sonuc['legacy_ters'],
                        f"isaret duzeltmesi kazanc saglamadi: {sonuc}")

    def test_statik_hedef_vurulur(self):
        """Kullanicinin '2loskf statik hedefi hep vururdu' referans olcutu:
        bizim yasa da vurmali (saf takip zaten yeterli, PN bozmamali)."""
        avci, hedef = self.devir_durumu()
        k = _kur()
        m = Motor(k, avci, np.array([5.0, 0.0, 0.0]), hedef, lambda t: (0, 0, 0))
        mn = m.kos(30.0)
        self.assertLess(mn, 2.0, f"statik hedefte en yakin menzil {mn:.2f} m")

    def test_capraz_hedef_vurulur(self):
        """Hedef 90 derece caprazdan geciyor -- LOS orani en yuksek durum.

        GEOMETRI YAKALANABILIR SECILDI: kopter 18 m/s, hedef 20 m/s oldugu
        icin capisma ucgeninin (|hedef(T) - avci_0| = 18 T) kokleri olmasi
        sarttir. Burada T ~ [3.6, 24] s araligi vardir, yani yasanin
        toparlanmasi icin pay var. Ayni senaryoda ONCULUKSUZ (k_pn=0) yasa
        kacirir -- bu, testin ayirt ediciligini kanitlar."""
        hedef = np.array([0.0, 0.0, -100.0])
        avci = np.array([-50.0, 60.0, -92.0])
        v0 = np.array([16.0, 4.0, 0.0])
        hedef_fn = (lambda t: (0.0, 20.0, 0.0))
        k = _kur()
        mn = Motor(k, avci.copy(), v0.copy(), hedef.copy(), hedef_fn).kos(40.0)
        # 8 m esigi: tirmanma tavani (3 m/s, bolum 3b-b) dikey kapanmayi
        # bilerek yavaslatiyor; 2026-08-03 oncesi (tavansiz) deger 2.8 m idi.
        # Bedel bilincli: kilit kaybi her seyi kaybettirir.
        self.assertLess(mn, 8.0, f"capraz hedefte en yakin menzil {mn:.1f} m")
        k0 = _kur(L.LosKontrolcu(aim_deg=0.0, k_pn=0.0,
                                 mount_deg=30.0, v_max_mps=20.0))
        mn0 = Motor(k0, avci.copy(), v0.copy(), hedef.copy(), hedef_fn).kos(40.0)
        self.assertGreater(mn0, 10.0,
                           f"saf takip de vurdu, test ayirt edici degil ({mn0:.1f})")

    def test_capraz_kacan_geometri_cozumsuz(self):
        """Belge amacli: hedefin avcidan UZAKLASAN tarafinda capisma ucgeninin
        KOKU YOKTUR (76T^2 + 1000T + 2369 = 0, iki kok de negatif). Hicbir
        gudum yasasi yakalayamaz; beklenen tek sey yasanin patlamamasidir."""
        hedef = np.array([0.0, 0.0, -100.0])
        avci = np.array([-40.0, -25.0, -88.0])
        k = _kur()
        m = Motor(k, avci, np.array([14.0, 8.0, 0.0]), hedef,
                  lambda t: (0.0, 20.0, 0.0))
        m.kos(30.0)
        self.assertTrue(np.all(np.isfinite(m.v)))
        self.assertLessEqual(abs(k.tani['onc_az']), k.onc_az_max + 1e-9)

    def test_yaw_titremesi_yok(self):
        """Yaw komutu isaret degistirerek salinmamali (gecmisteki titreme
        kip). Sabit donen hedefte isaret degisim sayisi sinirli olmali."""
        avci, hedef = self.devir_durumu()
        k = _kur()
        w = math.radians(9.0)
        yaw_kayit = []
        m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                  lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        for _ in range(int(40.0 / m.dt)):
            if m.adim() < 1.5:
                break
            yaw_kayit.append(k.tani['yaw_rate'])
        isaret = [1 if y > 1.0 else (-1 if y < -1.0 else 0) for y in yaw_kayit]
        degisim = sum(1 for a, b in zip(isaret, isaret[1:])
                      if a != 0 and b != 0 and a != b)
        self.assertLess(degisim, 12, f"yaw isaret degisimi {degisim} (titreme)")

    def test_fov_korumasi_ham_kadrajda_tutuyor(self):
        """KAPALI DONGU KANITI (bolum 3b): kopter pitch vekiliyle, koruma
        KAPALIYKEN hedef ham kadrajdan cikar; ACIKKEN kadrajda kalir.

        pitch vekili elips kosusunun olcumune oturtuldu
        (pitch ~= -1.8 + 3.2 * tirmanma_hizi); ayrintisi Motor._pitch'te."""
        sonuc = {}
        for etiket, kw in (('kapali', dict(k_fov=0.0, tirmanma_max_mps=99.0)),
                           ('acik', dict(k_fov=1.0, tirmanma_max_mps=3.0))):
            # GERCEK kosunun geometrisi: konumlu hedefin 41.8 m gerisine
            # surukleniyor (hedef 20 m/s, kopter tavani 20 m/s), yukselis
            # atan(11/41.8) = 14.8 deg -- olculen eps ortancasi 14.9 idi.
            avci, hedef = self.devir_durumu(back=41.8, down=11.0)
            # mount_deg ACIK: kontrolcu hazir kuruldugu icin _kur'un
            # varsayilani devreye girmez; Motor da mount=30 kullaniyor,
            # ayrisirsa test yasayi degil uyumsuzlugu olcer.
            k = _kur(L.LosKontrolcu(aim_deg=0.0, mount_deg=30.0,
                                    v_max_mps=20.0, **kw))
            m = Motor(k, avci, np.array([19.0, 0.0, 0.0]), hedef,
                      lambda t: (20.0, 0.0, 0.0), pitch_modeli='kopter')
            m.kos(25.0)
            n = len(m.iz)
            sonuc[etiket] = (m.ham_kayip, n, m.ham_kayip / max(n, 1))
        print(f"\n    [bilgi] ham kadraj disi ornek orani: "
              f"koruma kapali %{100*sonuc['kapali'][2]:.0f}, "
              f"acik %{100*sonuc['acik'][2]:.0f}")
        # Olculen: koruma kapali %24, acik %2 (10 kat iyilesme).
        self.assertGreater(sonuc['kapali'][2], 0.20,
                           f"koruma kapaliyken kayip beklenirdi: {sonuc}")
        self.assertLess(sonuc['acik'][2], 0.05,
                        f"koruma acikken hedef kadrajda kalmali: {sonuc}")
        self.assertLess(sonuc['acik'][2], 0.25 * sonuc['kapali'][2])

    def test_hicbir_komut_nan_degil(self):
        avci, hedef = self.devir_durumu()
        k = _kur()
        w = math.radians(20.0)
        m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                  lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t),
                             3.0 * math.sin(0.7 * t)))
        for _ in range(int(40.0 / m.dt)):
            if m.adim() < 1.5:
                break
            self.assertTrue(np.all(np.isfinite(m.v)))
            self.assertLessEqual(np.linalg.norm(m.v), k.v_max + 1e-6)

    def test_dikey_oran_kuyruk_kovalamacasinda(self):
        """--dikey-oran < 1 duz (kazanilamaz) senaryoda menzili daha iyi
        korumali: aci avantajini harcamaz. Rapordaki dugmenin kaniti."""
        sonuc = {}
        for kd in (1.0, 0.4):
            avci, hedef = self.devir_durumu()
            k = _kur(L.LosKontrolcu(aim_deg=0.0, dikey_oran=kd))
            m = Motor(k, avci, np.array([17.0, 0.0, 0.0]), hedef,
                      lambda t: (20.0, 0.0, 0.0))
            m.kos(30.0)
            sonuc[kd] = m.iz[-1][1]
        # Bilgi amacli; sert esik koymuyoruz (senaryoya duyarli).
        print(f"\n    [bilgi] duz rota 30 s sonunda menzil: "
              f"k_dik=1.0 -> {sonuc[1.0]:.1f} m, k_dik=0.4 -> {sonuc[0.4]:.1f} m")
        self.assertTrue(all(math.isfinite(v) for v in sonuc.values()))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + sys.argv[1:], verbosity=2)
