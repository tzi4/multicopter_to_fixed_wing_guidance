#!/usr/bin/env python3
"""terminal_los_gudum icin simsiz birim ve kapali-dongu testleri."""

from __future__ import annotations

import math
import unittest

import numpy as np

from goruntulu_temel import Olcum
from terminal_los_gudum import TerminalLosKontrolcu, _aci_deg


DT = 0.05


def olcum(t, ex=0.0, ey=0.0, menzil=25.0, vel=(20.0, 0.0, 0.0),
          yaw=0.0, pos=(0.0, 0.0, -60.0), vibe=None, tilt=0.0,
          bbox=(50.0, 20.0)):
    bw, bh = bbox
    return Olcum(
        t=t, dt=DT, ex_deg=ex, ey_deg=ey, bbox_w=bw, bbox_h=bh,
        alan_kok=math.sqrt(bw*bh), kapsama_pct=100.0*bw/1280.0,
        bbox_yas_s=0.02,
        menzil_m=menzil, pos_ned=np.asarray(pos, float),
        vel_ned=np.asarray(vel, float), yaw_rad=yaw, roll_rad=0.0,
        pitch_rad=0.0, t_capture=t, vibe_max=vibe, tilt_deg=tilt)


class Birim(unittest.TestCase):
    def yeni(self, **kw):
        k = TerminalLosKontrolcu(**kw)
        k.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        return k

    def test_erisilebilirlik_delta_v_ve_yon_konisi(self):
        k = self.yeni()
        v = np.array([20.0, 0.0, 0.0])
        for ham in ([-35, 0, 0], [0, 35, 0], [0, -35, 10], [80, 80, -20]):
            u = k._erisilebilir(v, np.asarray(ham, float))
            self.assertLessEqual(np.linalg.norm(u-v),
                                 k.a_yatay_max*k.komut_ufku+1e-9)
            self.assertLessEqual(_aci_deg(u[:2], v[:2]),
                                 k.komut_aci_max+1e-9)
            self.assertGreaterEqual(float(np.dot(u, v)), 0.0)
            self.assertGreaterEqual(u[2], -k.tirmanma_hiz_max-1e-9)
            self.assertLessEqual(u[2], k.alcalma_hiz_max+1e-9)

    def test_yerles_sonra_vur(self):
        k = self.yeni()
        for i in range(18):
            cmd = k.komut(olcum(i*DT, menzil=25.0-0.1*i))
        self.assertEqual(k.faz, "VUR")
        self.assertGreater(k.tani["a_ileri"], 2.0)
        self.assertGreater(cmd.vel_ned[0], 20.0)

    def test_sert_los_oraninda_gaz_kesip_yerlesir(self):
        k = self.yeni()
        for i in range(18):
            k.komut(olcum(i*DT, menzil=25.0-0.1*i))
        self.assertEqual(k.faz, "VUR")
        for j in range(1, 10):
            k.komut(olcum((18+j)*DT, ex=2.0*j, menzil=23.1))
        self.assertEqual(k.faz, "YERLES")
        self.assertEqual(k.tani["a_ileri"], 0.0)

    def test_terminalde_ters_komut_yok(self):
        k = self.yeni()
        cmd = k.komut(olcum(0.0, ex=28.0, menzil=2.8,
                            vel=(26.0, 2.0, 0.0)))
        self.assertEqual(k.faz, "DON")
        self.assertGreater(float(np.dot(cmd.vel_ned, np.array([26., 2., 0.]))),
                           0.0)
        self.assertLessEqual(k.tani["cmd_real_aci"], 45.0+1e-9)

    def test_dikey_kanal_isareti_ve_terminal_sonum(self):
        k = self.yeni()
        # Hedef yukarida: ey negatif -> eps pozitif -> NED vz azalir.
        for i in range(20):
            cmd = k.komut(olcum(i*DT, ey=-8.0, menzil=20.0))
        self.assertLess(cmd.vel_ned[2], 0.0)
        # Dikey terminal kapisinda ivme jerk siniriyla sifira doner.
        onceki = abs(k.tani["a_z"])
        for j in range(20):
            k.komut(olcum((20+j)*DT, ey=-8.0, menzil=7.0))
        self.assertLess(abs(k.tani["a_z"]), onceki)

    def test_canli_tilt_gudum_geometrisini_ofsetlemez(self):
        # Stabilize ey zaten dunya LOS'udur. Ayni ey, farkli kamera tilt'i
        # ayni komutu vermeli; tilt yalniz kadraj/tani referansidir.
        a, b = self.yeni(), self.yeni()
        ca = a.komut(olcum(0.0, ex=3.0, ey=-5.0, tilt=-20.0))
        cb = b.komut(olcum(0.0, ex=3.0, ey=-5.0, tilt=+20.0))
        np.testing.assert_allclose(ca.vel_ned, cb.vel_ned, atol=1e-12)

    def test_yaw_kapatilsa_da_yanal_los_komutu_kalir(self):
        k = self.yeni(yaw_komutu_ver=False)
        cmd = k.komut(olcum(0.0, ex=12.0, menzil=20.0))
        self.assertIsNone(cmd.yaw_rate_dps)
        self.assertNotEqual(cmd.vel_ned[1], 0.0)

    def test_hibrit_gecis_latchlidir(self):
        from hibrit_gudum import HibritKontrolcu
        h = HibritKontrolcu(gecis_menzil_m=18.0)
        h.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        cmd = h.komut(olcum(0.0, menzil=17.9))
        self.assertEqual(h.faz, "LOS")
        self.assertEqual(cmd.olay, "hibrit_los_gecisi")
        h.komut(olcum(DT, menzil=22.0))
        self.assertEqual(h.faz, "LOS")

    def test_hibrit_mpc_cikisi_da_erisilebilir(self):
        from hibrit_gudum import HibritKontrolcu
        h = HibritKontrolcu(gecis_menzil_m=18.0)
        h.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 30.0})
        o = olcum(0.0, ex=18.0, ey=-8.0, menzil=30.0,
                  vel=(20.0, 0.0, 0.0))
        cmd = h.komut(o)
        self.assertEqual(h.faz, "MPC")
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned-o.vel_ned),
                             h.terminal.a_yatay_max*h.terminal.komut_ufku+1e-9)
        self.assertLessEqual(_aci_deg(cmd.vel_ned[:2], o.vel_ned[:2]), 45.0)
        self.assertGreaterEqual(cmd.vel_ned[2], -h.terminal.tirmanma_hiz_max)

    def test_hibrit_varsayilan_menzil_gecisini_birebir_korur(self):
        from hibrit_gudum import HibritKontrolcu
        h = HibritKontrolcu(gecis_menzil_m=18.0)
        h.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 30.0})
        # Gorsel kapi fazlasiyla hazir olsa da varsayilan kaynak MENZIL.
        for i in range(10):
            h.komut(olcum(i*DT, menzil=25.0, bbox=(80.0, 32.0)))
        self.assertEqual(h.faz, "MPC")
        h.komut(olcum(10*DT, menzil=17.9, bbox=(20.0, 8.0)))
        self.assertEqual(h.faz, "LOS")

    def test_hibrit_gorsel_gecis_menzilden_bagimsizdir(self):
        from hibrit_gudum import HibritKontrolcu
        h = HibritKontrolcu(gecis_kaynagi="gorsel", gorsel_dwell_s=0.30)
        h.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 40.0})
        # 70x23 -> normalize karekok alan ~%4.18; merkez kapisinda.
        for i in range(5):
            h.komut(olcum(i*DT, ex=1.0, ey=-10.0, menzil=40.0,
                          bbox=(70.0, 23.0)))
        self.assertEqual(h.faz, "MPC")
        cmd = h.komut(olcum(5*DT, ex=1.0, ey=-10.0, menzil=40.0,
                            bbox=(70.0, 23.0)))
        self.assertEqual(h.faz, "LOS")
        self.assertIn("kaynak=gorsel", cmd.olay_detay)

    def test_hibrit_gorsel_gecis_kucuk_veya_kenarda_reddedilir(self):
        from hibrit_gudum import HibritKontrolcu
        h = HibritKontrolcu(gecis_kaynagi="gorsel")
        h.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 10.0})
        for i in range(10):
            h.komut(olcum(i*DT, ex=9.0, ey=0.0, menzil=10.0,
                          bbox=(70.0, 23.0)))
        self.assertEqual(h.faz, "MPC")
        for i in range(10, 20):
            h.komut(olcum(i*DT, ex=0.0, ey=0.0, menzil=10.0,
                          bbox=(30.0, 10.0)))
        self.assertEqual(h.faz, "MPC")

    def test_acilan_menzilde_vur_yok_ve_iska_birakilir(self):
        k = self.yeni()
        t = 0.0
        # Once kapanip ISKA kapisini arm et.
        for r in (25, 23, 20, 17, 14, 10, 7):
            cmd = k.komut(olcum(t, menzil=r)); t += DT
        # Sonra belirgin ve surekli acil.
        for r in (9, 12, 16, 18, 20):
            for _ in range(4):
                cmd = k.komut(olcum(t, menzil=r)); t += DT
        self.assertNotEqual(k.faz, "VUR")
        self.assertTrue(cmd.birak)
        self.assertIn("gecis sonrasi", cmd.birak_sebep)


class KapaliDongu(unittest.TestCase):
    def kos(self, hedef_donus_dps):
        k = TerminalLosKontrolcu()
        k.tohumla({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        p = np.array([0.0, 0.0, -60.0])
        v = np.array([20.0, 0.0, 0.0])
        h = np.array([25.0, 0.0, -60.0])
        yaw = 0.0
        en_yakin = 1e9
        en_buyuk_aci = 0.0
        for i in range(500):
            t = i*DT
            rota = math.radians(hedef_donus_dps)*max(t-1.0, 0.0)
            vh = np.array([20.0*math.cos(rota), 20.0*math.sin(rota), 0.0])
            rel = h-p
            r = float(np.linalg.norm(rel))
            en_yakin = min(en_yakin, r)
            kerteriz = math.atan2(rel[1], rel[0])
            ex = (math.degrees(kerteriz-yaw)+180.0) % 360.0-180.0
            eps = math.degrees(math.atan2(-rel[2],
                                          max(np.linalg.norm(rel[:2]), 1e-6)))
            cmd = k.komut(olcum(t, ex=ex, ey=-eps, menzil=r, vel=v,
                                yaw=yaw, pos=p))
            en_buyuk_aci = max(en_buyuk_aci, _aci_deg(cmd.vel_ned, v))
            # Basit ArduPilot vekili: 0.45 s hiz cevabi, 5 m/s2 yatay tavan.
            acc = (cmd.vel_ned-v)/0.45
            na = float(np.linalg.norm(acc[:2]))
            if na > 5.0:
                acc[:2] *= 5.0/na
            acc[2] = np.clip(acc[2], -5.0, 5.0)
            v += acc*DT
            p += v*DT
            h += vh*DT
            yaw += math.radians(cmd.yaw_rate_dps or 0.0)*DT
        return en_yakin, en_buyuk_aci

    def test_duz_kuyrukta_carpisma(self):
        en_yakin, aci = self.kos(0.0)
        self.assertLess(en_yakin, 1.0)
        self.assertLessEqual(aci, 45.0+1e-9)

    def test_elips_benzeri_sert_donuste_carpisma_konisi(self):
        en_yakin, aci = self.kos(8.0)
        self.assertLess(en_yakin, 2.0)
        self.assertLessEqual(aci, 45.0+1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
