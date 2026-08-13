#!/usr/bin/env python3
"""
takip_test.py - takip_gudum.py (ArduPilot FOLLOW yasasi) CEVRIMDISI TESTLERI
==============================================================================
Sim GEREKTIRMEZ. Iki katman:

  BIRIM     : ArduPilot matematiginin portu (sqrt_controller), LOS/kestirim
              geometrisi, hiz yasasinin kelepceleri, ofset erimesi, yaw.
  KAPALI    : mpc_test.Benzetim ile kapali dongu -- AYNI motor, AYNI iskelet
  DONGU       taklidi (LPF 0.35, hiz kelepcesi, ivme sinirli otopilot, gercek
              sanal gimbal). Yani MPC ile olculen sayilar dogrudan kiyaslanabilir.

Kosum:  cd guidance_allstar && python3 takip_test.py [--hizli]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

_BURASI = str(Path(__file__).resolve().parent)
if _BURASI not in sys.path:
    sys.path.insert(0, _BURASI)

from goruntulu_temel import Olcum                              # noqa: E402
from takip_gudum import (TakipAyar, TakipKontrolcu, los_ucayak,  # noqa: E402
                         sqrt_controller, sqrt_controller_2d)
import mpc_test                                                # noqa: E402
from mpc_test import Benzetim, ISKELET_HIZ_TAVANI              # noqa: E402

GECTI = []
KALDI = []


def _rapor(ad, tamam, ek=""):
    (GECTI if tamam else KALDI).append(ad)
    print(f"  [{'OK ' if tamam else 'HATA'}] {ad}{(' -- ' + ek) if ek else ''}")
    return tamam


def _olcum(ex=0.0, ey=0.0, r=40.0, t=0.0, dt=0.05, pos=(0.0, 0.0, -60.0),
           vel=(20.0, 0.0, 0.0), yaw=0.0, yas=0.0, vibe=None, alan=40.0):
    return Olcum(t=t, dt=dt, ex_deg=ex, ey_deg=ey, bbox_w=alan, bbox_h=alan,
                 alan_kok=alan, kapsama_pct=None, bbox_yas_s=yas, menzil_m=r,
                 pos_ned=np.array(pos, dtype=float),
                 vel_ned=np.array(vel, dtype=float),
                 yaw_rad=yaw, roll_rad=0.0, pitch_rad=0.0, vibe_max=vibe)


# ================================================ 0. ORTAK MOTOR MUHURU

def test_motor_muhru():
    """ORTAK TEST TESISI (mpc_test.Benzetim) DEGISTI Mİ?

    NEDEN VAR: bu dosyanin butun kapali dongu sayilari mpc_test.Benzetim
    motoruyla uretiliyor. O motor MPC'ye AIT DEGIL -- nokta-kutle avci +
    sanal gimbal + iskelet taklidi, yani iki yasanin da ORTAK zemini; yalnizca
    tarihsel olarak MPC'nin dosyasinda oturuyor. Baska bir ajan MPC uzerinde
    calisirken o sabitleri degistirirse (TO_TEST madde 10 tam olarak bunu
    oneriyor: eyleyici modeli tau) buradaki butun referans sayilar SESSIZCE
    kayar ve regresyon testleri yalan soyler.

    Bu test o sessizligi bozar: motor fizigi degisirse BURADA patlar ve
    ARDUPILOT_TAKIP.md'deki tablolarin yeniden olculmesi gerektigini soyler.
    BASARISIZLIK = 'motor degisti, sayilari tazele' demektir; hata DEGIL."""
    print("\n0. Ortak motor muhru (mpc_test.Benzetim fizigi)")
    beklenen = {
        'ISKELET_HIZ_TAVANI': (ISKELET_HIZ_TAVANI, 35.0),
        'ISKELET_TIRMANMA_MPS': (mpc_test.ISKELET_TIRMANMA_MPS, 10.0),
        'ISKELET_ALCALMA_MPS': (mpc_test.ISKELET_ALCALMA_MPS, 5.0),
        'KAYIP_BITIS_S': (mpc_test.KAYIP_BITIS_S, 1.5),
        'Benzetim.IVME_YATAY': (Benzetim.IVME_YATAY, 5.0),
        'Benzetim.IVME_DIKEY': (Benzetim.IVME_DIKEY, 5.0),
        'Benzetim.YAW_SLEW_DPS2': (Benzetim.YAW_SLEW_DPS2, 120.0),
        'Benzetim.YAW_LPF_TAU': (Benzetim.YAW_LPF_TAU, 0.15),
        'Benzetim.PITCH_TIRMANMA': (Benzetim.PITCH_TIRMANMA, 2.6),
    }
    kaymis = [f"{ad}: {olcu} != {bek}" for ad, (olcu, bek) in beklenen.items()
              if abs(float(olcu) - bek) > 1e-9]
    tamam = _rapor("0a motor sabitleri muhurlu", not kaymis,
                   '; '.join(kaymis) if kaymis else
                   f"{len(beklenen)} sabit degismemis")
    # gimbal_kamera VARSAYILANI: gimbal dalinda True. Degisirse butun
    # ARDUPILOT_TAKIP.md 6e tablosu (yasa vs fizik 2x2) yeniden okunmali.
    import inspect
    gk = inspect.signature(Benzetim.__init__).parameters['gimbal_kamera'].default
    tamam &= _rapor("0b gimbal_kamera varsayilani True (gimbal dali)",
                    gk is True, f"varsayilan={gk}")
    return tamam


# ======================================================= 1. ArduPilot matematigi

def test_sqrt_controller():
    """AP_Math/control.cpp portunun uc kolu da dogru mu."""
    print("\n1. sqrt_controller (AP_Math/control.cpp portu)")
    tamam = True

    # (a) ikinci mertebe tavan yoksa saf P
    tamam &= _rapor("1a tavan=0 -> saf P",
                    abs(sqrt_controller(10.0, 0.5, 0.0, 0.1) - 5.0) < 1e-9)

    # (b) p=0 -> saf sqrt(2*a*x)
    beklenen = math.sqrt(2.0 * 2.5 * 20.0)
    v = sqrt_controller(20.0, 0.0, 2.5, 0.0)
    tamam &= _rapor("1b p=0 -> sqrt(2*a*x)", abs(v - beklenen) < 1e-9,
                    f"{v:.3f} m/s")

    # (c) hibrit: dogrusal bolge |x| < a/p^2, disinda sqrt kolu
    p, a = 1.0, 2.5
    dogrusal = a / (p * p)                      # 2.5 m
    ic = sqrt_controller(1.0, p, a, 0.0)
    dis = sqrt_controller(25.0, p, a, 0.0)
    dis_bek = math.sqrt(2.0 * a * (25.0 - dogrusal / 2.0))
    tamam &= _rapor("1c hibrit kollar",
                    abs(ic - 1.0) < 1e-9 and abs(dis - dis_bek) < 1e-9,
                    f"ic={ic:.2f} dis={dis:.2f} (dogrusal sinir {dogrusal:.1f} m)")

    # (d) isaret bakisimi
    tamam &= _rapor("1d tek fonksiyon (tersi isaretle)",
                    abs(sqrt_controller(-25.0, p, a, 0.0) + dis) < 1e-9)

    # (e) dt kelepcesi: bir adimda hatayi asma
    v = sqrt_controller(0.1, 10.0, 0.0, 0.05)   # P cikisi 1.0, sinir 0.1/0.05=2
    v2 = sqrt_controller(0.1, 100.0, 0.0, 0.05)  # P cikisi 10 -> 2'ye kirpilir
    tamam &= _rapor("1e dt kelepcesi", abs(v - 1.0) < 1e-9 and abs(v2 - 2.0) < 1e-9,
                    f"{v:.2f} / {v2:.2f} m/s")

    # (f) 2D formu yonu korur
    e = np.array([3.0, 4.0])
    v2d = sqrt_controller_2d(e, 1.0, 2.5, 0.0)
    yon_hata = abs(float(np.cross(e / 5.0, v2d)))
    tamam &= _rapor("1f 2D yon korunumu", yon_hata < 1e-9,
                    f"|v|={np.linalg.norm(v2d):.2f}")
    return tamam


# ======================================================= 2. Goru -> hedef konumu

def test_hedef_kestirimi():
    """ex/ey/menzil -> hedef NED konumu; AP_Follow'un yerine gecen adim."""
    print("\n2. Hedef kestirimi (goru + menzil -> NED)")
    tamam = True
    k = TakipKontrolcu(TakipAyar())

    # (a) burun kuzeye, hedef tam onde ve es irtifada
    o = _olcum(ex=0.0, ey=0.0, r=50.0, yaw=0.0, pos=(0.0, 0.0, -60.0))
    hedef, u = k._hedef_kestir(o, 50.0, 0.0, 0.0)
    tamam &= _rapor("2a tam onde", np.allclose(hedef, [50.0, 0.0, -60.0], atol=1e-6),
                    f"{np.round(hedef, 2).tolist()}")

    # (b) ex=+30 -> hedef SAGDA (dogu, yaw=0'da)
    hedef, u = k._hedef_kestir(o, 50.0, 30.0, 0.0)
    tamam &= _rapor("2b ex>0 -> saga",
                    hedef[1] > 24.9 and abs(hedef[0] - 43.3) < 0.1,
                    f"{np.round(hedef, 2).tolist()}")

    # (c) eps=+20 (hedef YUKARIDA) -> NED z KUCULUR (irtifa artar)
    hedef, u = k._hedef_kestir(o, 50.0, 0.0, 20.0)
    tamam &= _rapor("2c eps>0 -> hedef yukarida",
                    abs(hedef[2] - (-60.0 - 50.0 * math.sin(math.radians(20)))) < 1e-6,
                    f"z={hedef[2]:.2f} (irtifa {-hedef[2]:.1f} m)")

    # (d) yaw ile donme: burun doguya bakiyorsa ex=0 hedefi DOGUYA koyar
    o2 = _olcum(yaw=math.radians(90.0))
    hedef, u = k._hedef_kestir(o2, 50.0, 0.0, 0.0)
    tamam &= _rapor("2d yaw cevrimi", np.allclose(hedef, [0.0, 50.0, -60.0], atol=1e-6),
                    f"{np.round(hedef, 2).tolist()}")

    # (e) TERS COZUM: rastgele hedeflerden uretilen ex/eps geri kestirimde
    #     ayni noktayi vermeli (gimbal zinciri disinda saf geometri).
    rng = np.random.default_rng(7)
    en_kotu = 0.0
    for _ in range(200):
        pos = rng.normal(0.0, 100.0, 3)
        yaw = rng.uniform(-math.pi, math.pi)
        d = rng.normal(0.0, 40.0, 3)
        r = float(np.linalg.norm(d))
        if r < 1.0:
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        dh = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        ex = math.degrees(math.atan2(dh[1], dh[0]))
        eps = math.degrees(math.atan2(-dh[2], math.hypot(dh[0], dh[1])))
        o3 = _olcum(pos=tuple(pos), yaw=yaw)
        hedef, _ = k._hedef_kestir(o3, r, ex, eps)
        en_kotu = max(en_kotu, float(np.linalg.norm(hedef - (pos + d))))
    tamam &= _rapor("2e ters cozum (200 rastgele)", en_kotu < 1e-9,
                    f"en buyuk hata {en_kotu:.2e} m")
    return tamam


# ======================================================= 3. Hiz yasasi kelepceleri

def test_hiz_yasasi():
    """ModeFollow::run() sirasi: P -> yatay olcek -> dikey kelepce -> fren."""
    print("\n3. Hiz yasasi (mode_follow.cpp sirasi)")
    tamam = True

    # (a) klasik kol saf P ('p' hiz kaynagi ile -- mode_follow'un kendisi)
    k = TakipKontrolcu(TakipAyar(kp=0.5, fren='kapali', hiz_kaynagi='p'))
    v, kel, _ = k._hiz_yasasi(np.array([20.0, 0.0, 0.0]), 20.0, 0.05)
    tamam &= _rapor("3a klasik = kp*hata", abs(v[0] - 10.0) < 1e-9, f"{v[0]:.2f} m/s")

    # (b) yatay tavan: YON KORUNUR
    k = TakipKontrolcu(TakipAyar(kp=1.0, hiz_tavani_mps=35.0, fren='kapali',
                                 hiz_kaynagi='p'))
    v, kel, _ = k._hiz_yasasi(np.array([60.0, 60.0, 0.0]), 84.85, 0.05)
    yatay = math.hypot(v[0], v[1])
    tamam &= _rapor("3b yatay tavan + yon korunumu",
                    abs(yatay - 35.0) < 1e-6 and abs(v[0] - v[1]) < 1e-9 and kel,
                    f"|v_xy|={yatay:.2f}")

    # (c) dikey tavanlar asimetrik (WPNAV_SPEED_UP 10 / _DN 5)
    v, _, _ = k._hiz_yasasi(np.array([0.0, 0.0, -50.0]), 0.0, 0.05)
    v2, _, _ = k._hiz_yasasi(np.array([0.0, 0.0, 50.0]), 0.0, 0.05)
    tamam &= _rapor("3c dikey tavan 10 yukari / 5 asagi",
                    abs(v[2] + 10.0) < 1e-9 and abs(v2[2] - 5.0) < 1e-9,
                    f"{v[2]:.1f} / {v2[2]:.1f} m/s")

    # (d) FREN KAPALI vs ACIK: 25 m'de tavan = sqrt(2*2.5*(25-1.25)) = 10.90
    k_kapali = TakipKontrolcu(TakipAyar(kp=1.0, fren='kapali', hiz_kaynagi='p'))
    k_acik = TakipKontrolcu(TakipAyar(kp=1.0, fren='ap', hiz_kaynagi='p'))
    k_acik._son_hata_z = 0.0
    hata = np.array([25.0, 0.0, 0.0])
    v_k, _, _ = k_kapali._hiz_yasasi(hata, 25.0, 0.05)
    v_a, _, ft = k_acik._hiz_yasasi(hata, 25.0, 0.05)
    beklenen = math.sqrt(2.0 * (0.5 * 5.0) * (25.0 - (0.5 * 5.0) / 2.0))
    tamam &= _rapor("3d fren: 25 m'de 10.90 m/s tavan (ACIK), kapalida 25",
                    abs(v_k[0] - 25.0) < 1e-6 and abs(v_a[0] - beklenen) < 1e-6,
                    f"kapali={v_k[0]:.2f} acik={v_a[0]:.2f} (tavan {ft:.2f})")

    # (e) FRENIN GEREKCESI: acikken hedef hizina (21 m/s) yetisilemez
    tamam &= _rapor("3e fren aciksa 21 m/s hedef yakalanamaz (KAPALI olmali)",
                    v_a[0] < 21.05 < v_k[0],
                    f"fren tavani {v_a[0]:.1f} < hedef 21.05 < serbest {v_k[0]:.1f}")

    # (f) 'menzil' kolu: uzakta fren ACIK, yakinda KAPALI
    k_m = TakipKontrolcu(TakipAyar(kp=1.0, fren='menzil', fren_menzil_m=45.0,
                                   hiz_kaynagi='p'))
    k_m._son_hata_z = 0.0
    v_uzak, _, _ = k_m._hiz_yasasi(np.array([60.0, 0.0, 0.0]), 60.0, 0.05)
    v_yakin, _, _ = k_m._hiz_yasasi(np.array([25.0, 0.0, 0.0]), 25.0, 0.05)
    tamam &= _rapor("3f 'menzil' kolu (45 m esigi)",
                    v_uzak[0] < 20.0 and abs(v_yakin[0] - 25.0) < 1e-6,
                    f"60 m -> {v_uzak[0]:.1f}, 25 m -> {v_yakin[0]:.1f} m/s")

    # (g) poscon kolu: dogrusal bolge disinda klasikten YAVAS
    k_p = TakipKontrolcu(TakipAyar(yasa='poscon', fren='kapali', hiz_kaynagi='p'))
    v_p, _, _ = k_p._hiz_yasasi(np.array([25.0, 0.0, 0.0]), 25.0, 0.05)
    tamam &= _rapor("3g poscon (Copter>=4.5) sqrt kolu",
                    5.0 < v_p[0] < 20.0 and v_p[0] < v_k[0],
                    f"poscon={v_p[0]:.2f} vs klasik={v_k[0]:.2f} m/s")

    # (h) hiz_kaynagi='tavan' (VARSAYILAN): buyukluk menzilden BAGIMSIZ,
    #     yon degismez -- plane_follow'un yon/hiz ayrisimi.
    k_t = TakipKontrolcu(TakipAyar(fren='kapali'))       # varsayilan tavan
    v_uzak, _, _ = k_t._hiz_yasasi(np.array([60.0, 0.0, 0.0]), 60.0, 0.05)
    v_yakin, _, _ = k_t._hiz_yasasi(np.array([5.0, 0.0, 0.0]), 5.0, 0.05)
    v_egik, _, _ = k_t._hiz_yasasi(np.array([10.0, 10.0, 0.0]), 14.14, 0.05)
    tamam &= _rapor("3h 'tavan': hiz menzilden bagimsiz, yon korunur",
                    abs(v_uzak[0] - 35.0) < 1e-6 and abs(v_yakin[0] - 35.0) < 1e-6
                    and abs(v_egik[0] - v_egik[1]) < 1e-9,
                    f"60 m -> {v_uzak[0]:.1f}, 5 m -> {v_yakin[0]:.1f} m/s")

    # (i) ivme sekillendirme: komut BUYUKLUGUNUN artisini kirpar
    k_s = TakipKontrolcu(TakipAyar(fren='kapali', ivme_sekillendirme_mps2=2.0))
    k_s._son_v_ned = np.array([17.0, 0.0, 0.0])
    v_s, _, _ = k_s._hiz_yasasi(np.array([60.0, 0.0, 0.0]), 60.0, 0.05)
    tamam &= _rapor("3i ivme sekillendirme (2 m/s^2, dt=0.05 -> +0.1 m/s)",
                    abs(float(np.linalg.norm(v_s)) - 17.1) < 1e-6,
                    f"|v|={float(np.linalg.norm(v_s)):.2f} m/s")
    return tamam


# ======================================================= 4. kp gerekcesi (sapma 2)

def test_kp_gerekcesi():
    """SAF mode_follow (|v| = kp*hata) HAREKETLI hedefe neden carpamaz.

    Analitik: hedef hizi ileri beslemesi YASAK oldugu icin P terimi hedefin
    TUM hizini tek basina saglamak zorunda. Denge d* = v_hedef / kp; o
    mesafede komut tam hedef hizina esit olur ve menzil DONAR. Test hem
    aritmetigi hem kapali donguyu gosterir."""
    print("\n4. 'p' hiz kaynagi (saf mode_follow) -- denge mesafesi")
    tamam = True
    sonuc = {}
    for kp in (0.1, 0.35, 1.0, 3.0):
        d_denge = 21.05 / kp
        doyum_menzili = ISKELET_HIZ_TAVANI / kp      # tavana degme mesafesi
        sonuc[kp] = (d_denge, doyum_menzili)
        print(f"      kp={kp:<5.2f} denge mesafesi={d_denge:7.1f} m   "
              f"tavana doyum={doyum_menzili:6.1f} m")
    tamam &= _rapor("4a AP varsayilani kp=0.1 -> 210 m dengede kalir",
                    sonuc[0.1][0] > 200.0,
                    "angajman zarfi (devir <=60 m) tamamen disinda")

    # KAPALI DONGU: teori d*=21.05 m diyor, motor ne diyor?
    s = senaryo_kos(rota="duz", devir="kuyruk", hedef_hiz_mps=21.05, tohum=3,
                    sure=30.0, ayar=TakipAyar(hiz_kaynagi='p', kp=1.0))
    print(f"      kapali dongu (kp=1.0, 'p'): min menzil {s['min_r']:.2f} m "
          f"(teori {sonuc[1.0][0]:.1f} m), kapanma {s['kapanma_ort']:+.1f} m/s")
    tamam &= _rapor("4b kapali dongu teoriyi dogruluyor (denge ~21 m)",
                    abs(s['min_r'] - sonuc[1.0][0]) < 6.0,
                    f"olculen {s['min_r']:.2f} m vs teori {sonuc[1.0][0]:.1f} m")

    # VARSAYILAN kol ('tavan') ayni senaryoda duvarı kirmali
    s2 = senaryo_kos(rota="duz", devir="kuyruk", hedef_hiz_mps=21.05, tohum=3,
                     sure=30.0, ayar=TakipAyar())
    print(f"      kapali dongu ('tavan'): min menzil {s2['min_r']:.2f} m, "
          f"bitis={s2['bitis']}, kapanma {s2['kapanma_ort']:+.1f} m/s")
    tamam &= _rapor("4c 'tavan' kolu denge duvarini kiriyor",
                    s2['min_r'] < 0.5 * s['min_r'],
                    f"{s['min_r']:.1f} m -> {s2['min_r']:.2f} m")
    return tamam


# ======================================================= 5. Ofset ve yaw

def test_ofset_ve_yaw():
    print("\n5. Ofset erimesi (FOLL_OFS) ve yaw (FOLL_YAW_BEHAVE=0)")
    tamam = True
    k = TakipKontrolcu(TakipAyar(ofs_geri_m=25.0, ofs_asagi_m=6.0))
    u = np.array([1.0, 0.0, 0.0])
    ofs_uzak, olcek_uzak = k._ofset(u, 60.0)
    ofs_orta, olcek_orta = k._ofset(u, 35.0)
    ofs_yakin, olcek_yakin = k._ofset(u, 20.0)
    tamam &= _rapor("5a uzakta tam ofset (geri 25, asagi 6)",
                    abs(ofs_uzak[0] + 25.0) < 1e-9 and abs(ofs_uzak[2] - 6.0) < 1e-9,
                    f"olcek={olcek_uzak:.2f}")
    tamam &= _rapor("5b terminal/erime arasinda dogrusal",
                    0.2 < olcek_orta < 0.8, f"35 m -> olcek {olcek_orta:.2f}")
    tamam &= _rapor("5c erime menzilinde tam sifir",
                    olcek_yakin == 0.0 and np.allclose(ofs_yakin, 0.0),
                    "carpma ofsetsiz yapilir")

    k0 = TakipKontrolcu(TakipAyar())
    tamam &= _rapor("5d ofset varsayilani SIFIR (carpma)",
                    np.allclose(k0._ofset(u, 60.0)[0], 0.0))

    # yaw: ATC_ANG_YAW_P * ex, tavan ATC_SLEW_YAW
    tamam &= _rapor("5e yaw = 4.5 * ex", abs(k0._yaw(4.0) - 18.0) < 1e-9)
    tamam &= _rapor("5f yaw tavani 60 dps", abs(k0._yaw(40.0) - 60.0) < 1e-9)
    tamam &= _rapor("5g ex<0 -> sola", k0._yaw(-4.0) < 0.0)
    k_ablasyon = TakipKontrolcu(TakipAyar(yaw_komutu_ver=False))
    tamam &= _rapor("5h --no-yaw ablasyonu", k_ablasyon._yaw(10.0) is None)
    return tamam


# ======================================================= 6. Durum makinesi

def test_durum_makinesi():
    """ISKA/VURUS: MPC ile AYNI esiklerle calismali (ortak olcut)."""
    print("\n6. Durum makinesi (ISKA / VURUS / temas)")
    tamam = True

    # (a) faz gecisleri
    k = TakipKontrolcu(TakipAyar())
    for i, r in enumerate([80.0, 50.0, 44.0, 30.0, 21.0, 8.0]):
        k._durum_makinesi(_olcum(r=r, t=i * 0.05), r, 10.0, 0.05)
    tamam &= _rapor("6a KAPANMA -> TERMINAL -> VURUS",
                    k.durum == 'VURUS' and k.vurus_karisim > 0.99,
                    f"durum={k.durum} karisim={k.vurus_karisim:.2f}")

    # (b) ISKA: menzil aciliyor
    k = TakipKontrolcu(TakipAyar())
    t = 0.0
    for r in list(np.linspace(60.0, 10.0, 40)) + list(np.linspace(10.0, 50.0, 40)):
        t += 0.05
        k._durum_makinesi(_olcum(r=r, t=t), r, -5.0, 0.05)
    tamam &= _rapor("6b ISKA (menzil aciliyor)", k.durum == 'ISKA',
                    k.iska_sebep)

    # (c) ISKA: zaman asimi (hic kapanmayan angajman)
    k = TakipKontrolcu(TakipAyar())
    t = 0.0
    for _ in range(300):
        t += 0.05
        k._durum_makinesi(_olcum(r=70.0, t=t), 70.0, 0.0, 0.05)
        if k.durum == 'ISKA':
            break
    tamam &= _rapor("6c ISKA (zaman asimi 8 s)",
                    k.durum == 'ISKA' and 'zaman asimi' in k.iska_sebep,
                    f"t={t:.1f} s")

    # (d) --no-iska ablasyonu
    k = TakipKontrolcu(TakipAyar(iska_modu=False))
    t = 0.0
    for r in list(np.linspace(60.0, 10.0, 40)) + list(np.linspace(10.0, 90.0, 60)):
        t += 0.05
        k._durum_makinesi(_olcum(r=r, t=t), r, -5.0, 0.05)
    tamam &= _rapor("6d --no-iska: ISKA ilan edilmez", k.durum != 'ISKA',
                    f"durum={k.durum}")

    # (e) TEMAS tespiti: vibe>15 VE olculen menzil<3
    k = TakipKontrolcu(TakipAyar())
    yok = k._vurus_basarili_kontrol(_olcum(r=2.0, vibe=8.0), 2.0)
    yok2 = k._vurus_basarili_kontrol(_olcum(r=20.0, vibe=30.0), 20.0)
    var = k._vurus_basarili_kontrol(_olcum(r=1.2, vibe=22.0), 1.2)
    tekrar = k._vurus_basarili_kontrol(_olcum(r=1.0, vibe=25.0), 1.0)
    tamam &= _rapor("6e temas tespiti (vibe + menzil, latch)",
                    yok is None and yok2 is None and var is not None
                    and tekrar is None and k.vuruldu,
                    var[1] if var else '')
    return tamam


# ============================================ 6b. MENZIL BAGIMSIZLIGI (KANIT)

def test_menzil_bagimsizligi():
    """VARSAYILAN AYARDA KOMUT MENZILDEN BAGIMSIZDIR -- cebirsel kanit + test.

    Kullanicinin kurali: menzil YERDEN TESPITTEN geliyor, ona GUVENMIYORUZ;
    goruntulu gudumun amaci zaten yerden tespitin hatasini atip hedefi kendi
    gozuyle gormek. Bu testin sordugu soru: yasa menzile ne kadar YASLANIYOR?

    CEBIR (varsayilan: ofs=0, hiz_kaynagi='tavan', fren kapali):
        hata   = r * u_los + 0
        v      = kp * hata            = kp*r * u_los
        'tavan': v = v * (V/|v|)      = V * u_los      <-- r TAM SADELESIR
        yatay tavan: |v_xy| = V*cos(eps) <= V          <-- hic baglamaz
        dikey kelepce: v_z = clip(-V*sin(eps), ...)    <-- yalniz eps'e bagli
    Yani komut (ex, ey, yaw, aim)'in fonksiyonudur; MENZIL HIC GIRMEZ.
    Menzil YALNIZCA ISKA/faz durum makinesine girer -- yani "ne zaman
    vazgecilecegine" karar veren HAKEMDIR, kontrolcu girdisi DEGIL.

    Bu test o iddiayi sayiyla dogrular ve 'p' kolunda BOZULDUGUNU gosterir
    (orada |v| = kp*r, yani menzil dogrudan gaz pedalidir)."""
    print("\n6b. Menzil bagimsizligi (yerden tespite guvenmeme)")
    tamam = True

    def _komut_vec(ayar, r_olc, **kw):
        k = TakipKontrolcu(ayar)
        k.tohumla({'cmd_vel_ned': [20.0, 0.0, 0.0]})
        # sekillendirme ilk dongude buyuklugu kirpar; YON'u etkilemez.
        return np.asarray(k.komut(_olcum(r=r_olc, **kw)).vel_ned, float)

    # (a) VARSAYILAN: menzil 6 kat degissin, komut BIREBIR ayni kalsin
    A = TakipAyar()
    v1 = _komut_vec(A, 10.0, ex=4.0, ey=-6.0)
    v2 = _komut_vec(A, 60.0, ex=4.0, ey=-6.0)
    v3 = _komut_vec(A, 200.0, ex=4.0, ey=-6.0)
    tamam &= _rapor("6b-a varsayilan: komut menzilden BAGIMSIZ",
                    np.allclose(v1, v2, atol=1e-12)
                    and np.allclose(v1, v3, atol=1e-12),
                    f"10 m: {np.round(v1, 4).tolist()} == 200 m: "
                    f"{np.round(v3, 4).tolist()}")

    # (b) menzil HIC YOKKEN (olcum None) de ayni komut
    k = TakipKontrolcu(TakipAyar())
    k.tohumla({'cmd_vel_ned': [20.0, 0.0, 0.0]})
    o = _olcum(r=None, ex=4.0, ey=-6.0)
    v_yok = np.asarray(k.komut(o).vel_ned, float)
    tamam &= _rapor("6b-b menzil OLCUMU YOKKEN de ayni komut",
                    np.allclose(v1, v_yok, atol=1e-12),
                    f"{np.round(v_yok, 4).tolist()}")

    # (c) 'p' kolunda BOZULUR: orada menzil dogrudan gaz pedalidir
    P = TakipAyar(hiz_kaynagi='p')
    p1 = _komut_vec(P, 10.0, ex=4.0, ey=-6.0)
    p2 = _komut_vec(P, 60.0, ex=4.0, ey=-6.0)
    tamam &= _rapor("6b-c 'p' kolu menzile BAGLI (karsit kanit)",
                    not np.allclose(p1, p2, atol=1e-6),
                    f"|v| {float(np.linalg.norm(p1)):.2f} -> "
                    f"{float(np.linalg.norm(p2)):.2f} m/s")

    # (d) ofset ACIKKEN bagimsizlik BOZULUR (ofset menzille olcekleniyor)
    O = TakipAyar(ofs_geri_m=25.0, ofs_asagi_m=6.0)
    o1 = _komut_vec(O, 20.0, ex=4.0, ey=-6.0)
    o2 = _komut_vec(O, 60.0, ex=4.0, ey=-6.0)
    tamam &= _rapor("6b-d ofset ACIK -> menzile baglanir (bu yuzden 0)",
                    not np.allclose(o1, o2, atol=1e-6),
                    "carpma ayarinda ofset 0 oldugu icin bedel yok")

    # (e) MENZIL NEREYE GIRIYOR: yalniz durum makinesi. Ayni olcum akisini
    #     iki farkli menzille surersek komutlar ayni, DURUMLAR farkli olmali.
    def _durumlar(r_carpani):
        k = TakipKontrolcu(TakipAyar())
        k.tohumla({'cmd_vel_ned': [20.0, 0.0, 0.0]})
        durumlar, komutlar = [], []
        for i in range(60):
            r_gercek = 60.0 - i * 0.8
            c = k.komut(_olcum(r=r_gercek * r_carpani, t=i * 0.05,
                               ex=2.0, ey=-4.0))
            durumlar.append(k.durum)
            komutlar.append(np.asarray(c.vel_ned, float).copy())
        return durumlar, np.array(komutlar)

    d1, c1 = _durumlar(1.0)
    d2, c2 = _durumlar(0.5)       # menzil YARISI kadar raporlanıyor
    tamam &= _rapor("6b-e menzil bozulunca KOMUT ayni, DURUM degisir",
                    np.allclose(c1, c2, atol=1e-12) and d1 != d2,
                    f"faz gecisi {d1.count('VURUS')} -> {d2.count('VURUS')} dongu")
    return tamam


# ======================================================= 7. Komut yolu butunu

def test_komut_yolu():
    """komut() ucu uca: olcumden Komut'a, isaretler dogru mu."""
    print("\n7. Komut yolu (ucu uca)")
    tamam = True
    k = TakipKontrolcu(TakipAyar())
    k.tohumla({'cmd_vel_ned': [17.0, 0.0, 0.0]})

    # (a) hedef tam onde, es irtifa -> komut saf kuzey; BUYUKLUK ilk dongude
    #     sekillendirmeyle sinirli (tohum 17.0 + 3.0*dt) -- basamak YOK.
    c = k.komut(_olcum(ex=0.0, ey=0.0, r=40.0, t=0.0, dt=0.05))
    tamam &= _rapor("7a tam onde -> saf kuzey, ilk dongude basamak YOK",
                    c.vel_ned[0] > 0.0 and abs(c.vel_ned[1]) < 1e-6
                    and abs(c.vel_ned[2]) < 1e-6
                    and abs(c.vel_ned[0] - 17.15) < 1e-6,
                    f"{np.round(c.vel_ned, 2).tolist()}")
    # 130 adim = 6.5 s: ISKA zaman asimindan (8 s) ONCE, sekillendirme
    # tavana ulasmis olur (17 + 3*6.5 = 36.5 > 35).
    for i in range(1, 130):
        c = k.komut(_olcum(ex=0.0, ey=0.0, r=40.0, t=i * 0.05, dt=0.05))
    tamam &= _rapor("7a2 sekillendirme bitince tavan (35 m/s)",
                    abs(c.vel_ned[0] - 35.0) < 1e-6, f"{c.vel_ned[0]:.2f} m/s")

    # (b) hedef YUKARIDA (ey<0) -> TIRMANMA komutu (vz<0)
    k.sifirla()
    c = k.komut(_olcum(ex=0.0, ey=-12.0, r=30.0, t=0.0))
    tamam &= _rapor("7b hedef ustte -> tirmanma", c.vel_ned[2] < -1.0,
                    f"vz={c.vel_ned[2]:.2f} m/s")

    # (c) hedef ASAGIDA (ey>0) -> alcalma, ama 5 m/s ile sinirli
    k.sifirla()
    c = k.komut(_olcum(ex=0.0, ey=+30.0, r=30.0, t=0.0))
    tamam &= _rapor("7c hedef altta -> alcalma (<=5 m/s)",
                    0.0 < c.vel_ned[2] <= 5.0 + 1e-9, f"vz={c.vel_ned[2]:.2f}")

    # (d) yaw komutu ex ile ayni isarette
    k.sifirla()
    c = k.komut(_olcum(ex=8.0, r=40.0))
    tamam &= _rapor("7d yaw komutu (+ex -> +yaw)",
                    c.yaw_rate_dps is not None and c.yaw_rate_dps > 0,
                    f"{c.yaw_rate_dps:.1f} dps")

    # (e) ISKA'da birak bayragi + frenli suzulme
    k = TakipKontrolcu(TakipAyar())
    k.tohumla(None)
    t = 0.0
    son = None
    for r in list(np.linspace(60.0, 10.0, 40)) + list(np.linspace(10.0, 60.0, 60)):
        t += 0.05
        son = k.komut(_olcum(r=r, t=t, vel=(30.0, 0.0, 0.0), alan=(40.0 - r * 0.4)))
        if son.birak:
            break
    tamam &= _rapor("7e ISKA -> birak + suzulme (sifir DEGIL)",
                    son is not None and son.birak
                    and float(np.linalg.norm(son.vel_ned)) > 10.0,
                    f"|v|={float(np.linalg.norm(son.vel_ned)):.1f} m/s "
                    f"sebep={son.birak_sebep[:40]}")

    # (f) TEMAS olayi Komut'a takiliyor mu
    k = TakipKontrolcu(TakipAyar())
    k.tohumla(None)
    k.komut(_olcum(r=30.0, t=0.0))
    c = k.komut(_olcum(r=1.5, t=0.05, vibe=25.0))
    tamam &= _rapor("7f vurus_basarili olayi Komut'ta",
                    c.olay == 'vurus_basarili', c.olay_detay[:50])
    return tamam


# ======================================================= 8. Kapali dongu

def senaryo_kos(rota="duz", devir="kuyruk", ayar=None, sure=25.0, tohum=3,
                loop_hz=20.0, hedef_hiz_mps=20.0, jitter=True,
                carpisma_m=2.0, iska_bitir=True, hedef_irtifa_m=60.0,
                kayip_bitir_s=1.5, menzil_bozma=None, gimbal_kamera=None):
    """mpc_test.senaryo_kos'un TakipKontrolcu surumu (ayni motor, ayni cikti).

    Motor mpc_test.Benzetim: nokta-kutle avci, GERCEK sanal gimbal, iskeletin
    LPF/kelepce/ivme zinciri. Yani buradaki sayilar MPC'nin ayni fonksiyonla
    uretilmis sayilariyla dogrudan kiyaslanabilir."""
    # gimbal_kamera=None -> motorun kendi varsayilani (gimbal dalinda True).
    # False vererek ESKI govdeye-sabit fizik geri gelir; yasa degisimini
    # fizik degisiminden AYIRMAK icin sart (bkz. ARDUPILOT_TAKIP.md 6e).
    ek = {} if gimbal_kamera is None else {'gimbal_kamera': bool(gimbal_kamera)}
    b = Benzetim(rota=rota, devir=devir, tohum=tohum, loop_hz=loop_hz,
                 hedef_irtifa_m=hedef_irtifa_m, hedef_hiz_mps=hedef_hiz_mps, **ek)
    k = TakipKontrolcu(ayar or TakipAyar())
    k.tohumla({'cmd_vel_ned': b.v.copy().tolist()})
    rng = np.random.default_rng(tohum + 100)

    t = 0.0
    min_r = 1e9
    r0 = None
    kayip = 0
    adet = 0
    kayip_ardisik = 0.0
    min_r_taze = 1e9         # HALA GORURKEN ulasilan en yakin menzil
    v_son = b.v.copy()
    son_istek = None
    # BOZUCUYA AYRI RNG: ana rng dongu jitter'ini (dt) uretiyor; bozucu ondan
    # cekerse jitter dizisi kayar ve YORUNGE degisir -- yani "gurultu menzili
    # bozdu" diye okunan sey aslinda farkli bir dt dizisidir. Bu tuzaga bir
    # kez dusuldu (gurultu kolu 1.35 yerine 1.60 gosterdi); ayri tohum
    # kollari BIREBIR karsilastirilabilir yapar.
    bozucu_rng = np.random.default_rng(tohum + 900)
    cmd_hizlar = []
    kapanmalar = []
    iska_t = None
    iska_sebep = ''
    bitis = 'sure_doldu'
    while t < sure:
        dt = b.dt_nom
        if jitter:
            dt = b.dt_nom * (1.0 + 0.25 * rng.random())
            if rng.random() < 0.04:
                dt = b.dt_nom * (2.0 + 1.0 * rng.random())
        o, gorunur, r = b.olcum(dt)
        if menzil_bozma is not None:
            # MENZIL BOZMA: yalniz OLCUMU kirletir; b.olcum'un dondurdugu
            # GERCEK r (metrikler icin) dokunulmaz kalir. Yerden tespitin
            # hatasini taklit etmenin dogru yeri burasi -- ortak motora
            # (mpc_test.Benzetim) hic dokunulmuyor.
            o.menzil_m = menzil_bozma(o.menzil_m, t, bozucu_rng)
        if r0 is None:
            r0 = r
        min_r = min(min_r, r)
        adet += 1
        if gorunur:
            kayip_ardisik = 0.0
            min_r_taze = min(min_r_taze, r)
            c = k.komut(o)
            istek = np.asarray(c.vel_ned, dtype=float)
            yaw_cmd = c.yaw_rate_dps
            son_istek = istek.copy()
            cmd_hizlar.append(float(np.linalg.norm(istek)))
            if getattr(c, 'birak', False) and iska_t is None:
                iska_t, iska_sebep = t, c.birak_sebep
                if iska_bitir:
                    bitis = 'iska'
                    break
        else:
            kayip += 1
            kayip_ardisik += dt
            # ISKELETIN kayip davranisi (goruntulu_temel.calistir):
            #   yas <= 0.7 s        -> kontrolcu zaten cagrilir (yukarisi)
            #   0.7 < yas <= 1.7 s  -> SON komut TUTULUR (yaw haric)
            #   yas > 1.7 s         -> SUZULME: komut = OLCULEN hiz (ivme 0)
            if kayip_ardisik <= 1.0 and son_istek is not None:
                istek = son_istek
            else:
                istek = b.v.copy()
            yaw_cmd = None
        # kapanma hizi (ANALIZ): LOS boyunca bagil hiz
        d = b.q - b.p
        n = float(np.linalg.norm(d))
        if n > 1e-6:
            kapanmalar.append(float((b.v - np.array(
                [b.hedef_hiz * math.cos(b.hedef_yon),
                 b.hedef_hiz * math.sin(b.hedef_yon), 0.0])) @ (d / n)))
        b.ilerlet(istek, yaw_cmd, dt)
        t += dt
        v_son = b.v.copy()
        if carpisma_m > 0.0 and r <= carpisma_m:
            bitis = 'carpisma'
            break
        if b.irtifa < 5.0:
            bitis = 'yer_temasi'
            break
        if kayip_ardisik > kayip_bitir_s:
            # mpc_test ile AYNI kural (KAYIP_BITIS_S): bu kadar kadrajsiz
            # kalinca karar verici yetkiyi konumluya geri verir, yani
            # GORUNTULU SEGMENT BITER. Kosuyu surdurmek "gudum kalitesi"
            # diye korlemesine suzulmeyi olcmek olurdu.
            bitis = 'kadraj_kaybi'
            break
    return {
        'rota': rota, 'devir': devir, 'tohum': tohum, 'sure': t,
        'r0': r0, 'min_r': min_r, 'min_r_taze': min_r_taze, 'bitis': bitis,
        'kayip_pct': 100.0 * kayip / max(adet, 1),
        'cmd_p95': float(np.percentile(cmd_hizlar, 95)) if cmd_hizlar else 0.0,
        'cmd_max': float(np.max(cmd_hizlar)) if cmd_hizlar else 0.0,
        'kapanma_ort': float(np.mean(kapanmalar)) if kapanmalar else 0.0,
        'iska_t': iska_t, 'iska_sebep': iska_sebep,
        'min_irtifa': b.min_irtifa, 'durum': k.durum, 'vuruldu': k.vuruldu,
        'hiz_son': float(np.linalg.norm(v_son)),
    }


def test_kapali_dongu(hizli=False):
    """Yedi senaryo. Esikler OLCULEN degerlerin ~%30 ustunde: bunlar hedef
    degil REGRESYON BEKCISIDIR (ayni motorla MPC'nin sayilari icin bkz.
    ARDUPILOT_TAKIP.md kiyas tablosu).

    'kadraj_kaybi' bitisi BASARISIZLIK DEGIL, olculen bir sonuctur: 1.5 s
    kadrajsiz kalinca karar verici yetkiyi konumluya geri verir (mpc_test
    ile ayni kural)."""
    print("\n8. Kapali dongu (mpc_test.Benzetim motoru, 1.5 s kadraj kurali)")
    tamam = True
    tohumlar = (3, 11) if hizli else (3, 11, 29, 47)
    senaryolar = [
        ("statik", dict(rota="duz", hedef_hiz_mps=0.0, devir="kuyruk"), 3.0),
        ("duz/kuyruk", dict(rota="duz", devir="kuyruk"), 3.0),
        ("duz/capraz", dict(rota="duz", devir="capraz"), 6.0),
        ("elips/kuyruk", dict(rota="elips", devir="kuyruk"), 4.0),
        ("elips/capraz", dict(rota="elips", devir="capraz"), 9.0),
        ("wanderer/kuyruk", dict(rota="wanderer", devir="kuyruk"), 9.0),
        ("wanderer/capraz", dict(rota="wanderer", devir="capraz"), 24.0),
    ]
    for ad, kw, esik in senaryolar:
        kw.setdefault('hedef_hiz_mps', 21.05)
        minler, bitisler, kayiplar = [], [], []
        for tohum in tohumlar:
            s = senaryo_kos(ayar=TakipAyar(), tohum=tohum, sure=30.0, **kw)
            minler.append(s['min_r'])
            bitisler.append(s['bitis'])
            kayiplar.append(s['kayip_pct'])
        ortanca = float(np.median(minler))
        print(f"      {ad:16s} min ortanca={ortanca:6.2f} m  "
              f"kayip={float(np.mean(kayiplar)):4.1f}%  "
              f"bitis={','.join(b[:4] for b in bitisler)}")
        tamam &= _rapor(f"8-{ad}: min menzil ortancasi < {esik:.0f} m",
                        ortanca < esik, f"{ortanca:.2f} m")
    return tamam


def _bozucular():
    """Yerden tespitin gercek hayattaki bozulma bicimleri.

    Hepsi YALNIZ Olcum.menzil_m'i kirletir; gorus (ex/ey) TEMIZ kalir --
    zaten kullanicinin guvendigi kanal o."""
    durum = {}

    def donmus(r, t, rng):
        if r is None:
            return None
        durum.setdefault('ilk', r)
        return durum['ilk']

    return [
        ('temiz            ', lambda r, t, g: r),
        ('yanlilik x0.5    ', lambda r, t, g: None if r is None else 0.5 * r),
        ('yanlilik x2.0    ', lambda r, t, g: None if r is None else 2.0 * r),
        ('yanlilik +20 m   ', lambda r, t, g: None if r is None else r + 20.0),
        ('gurultu %30      ', lambda r, t, g: None if r is None
         else max(0.5, r * (1.0 + 0.30 * g.normal()))),
        ('donmus (ilk deger)', donmus),
        ('kopuk %50        ', lambda r, t, g: None if (r is None or g.random() < 0.5)
         else r),
        ('MENZIL YOK       ', lambda r, t, g: None),
    ]


def test_menzil_bozulmasi(hizli=False):
    """KAPALI DONGUDE menzili kirlet: yasa ne kadar bozuluyor?

    6b cebirsel olarak komutun menzilden bagimsiz oldugunu gosterdi. Burada
    sorulan soru pratik: yerden tespit YANILIRSA (yanlilik/gurultu/kopukluk/
    hic yok) ANGAJMANIN SONUCU ne kadar degisir? Beklenti: guduM aynen
    ayni ucar, degisen tek sey ISKA HAKEMININ ne zaman dudugu caldigi."""
    print("\n11. Menzil bozulmasi altinda dayaniklilik (yerden tespit hatasi)")
    tamam = True
    tohumlar = (3,) if hizli else (3, 11, 29)
    # SON SENARYO BILINCLI: elips/capraz ISKA ile bitiyor (bkz. test 8).
    # Ilk ucu komut yolunu, sonuncusu HAKEMI sinar -- menzil yalnizca orada
    # is goruyor, dolayisiyla bozulmanin gorunecegi tek yer orasi.
    senaryolar = [("duz/kuyruk", dict(rota="duz", devir="kuyruk")),
                  ("elips/kuyruk", dict(rota="elips", devir="kuyruk")),
                  ("wanderer/kuyruk", dict(rota="wanderer", devir="kuyruk")),
                  ("elips/capraz(ISKA)", dict(rota="elips", devir="capraz"))]
    taban = {}
    sonuc = {}
    for ad, bozucu in _bozucular():
        satir = []
        for sad, kw in senaryolar:
            minler, bitisler = [], []
            for tohum in tohumlar:
                s = senaryo_kos(ayar=TakipAyar(), tohum=tohum, sure=30.0,
                                hedef_hiz_mps=21.05, menzil_bozma=bozucu, **kw)
                minler.append(s['min_r'])
                bitisler.append(s['bitis'][:4])
            satir.append((sad, float(np.median(minler)), bitisler))
        sonuc[ad.strip()] = satir
        if ad.startswith('temiz'):
            taban = {s: m for s, m, _ in satir}
        print(f"      {ad}  " + "  ".join(
            f"{s}={m:6.2f}[{','.join(b)}]" for s, m, b in satir))

    # OLCUT: bozuk kollarin min menzili temiz kolun 2 katini asmamali.
    for ad, satir in sonuc.items():
        if ad.startswith('temiz'):
            continue
        kotu = [f"{s} {m:.1f} vs {taban[s]:.1f}"
                for s, m, _ in satir if m > max(2.0 * taban[s], taban[s] + 3.0)]
        tamam &= _rapor(f"11-{ad}: temize gore bozulma sinirli",
                        not kotu, '; '.join(kotu) if kotu else 'hepsi bandda')
    return tamam


def test_hakem_menzil_duyarliligi():
    """MENZIL NEREDE HALA IS GORUYOR: ISKA hakeminin menzil kollari.

    Test 11 komut yolunun menzilden bagimsiz oldugunu gosterdi ama o
    senaryolarda ISKA hep ZAMAN ASIMIYLA atesledi -- yani menzile bagli
    kollar (menzil aciliyor / mutlak menzil) hic denenmedi. Burada o kollar
    DOGRUDAN sinaniyor: sentetik bir 'kapan sonra acil' menzil profili
    hakeme verilir ve her bozulma altinda ISKA'nin NE ZAMAN caldigina bakilir.

    OLCULEN (tahminim yanlisti, sayi hakli): hakem HER IKI yanlilik
    turunden de kayiyor --
        x0.5 -> 70.6 m | temiz 18.7 m | +20 m -> 40.7 m | x2.0 -> 25.7 m
    Sebep: kural saf FARK kurali degil; fark testleri (r > en_iyi + 30)
    MUTLAK kapilarla (iska_gecis_arm_m=12, iska_arm_m=45, iska_mutlak_m=120)
    ic ice. Toplamsal yanlilik bu kapilari kaydirinca hangi KOLUN atesledigi
    degisiyor (gecis kolu -> normal kol), yani sonuc da degisiyor.

    Sonuc: komut yolu menzilden tamamen bagimsiz (test 11), ama HAKEM degil.
    Hakemi de kurtarmanin yolu mutlak kapilari atip ORAN tabanina gecmek --
    sqrt(alan) ~ 1/r oldugu icin bbox alani bunu KALIBRASYONSUZ verir
    (bkz. iska_kaynak='alan')."""
    print("\n12. Hakem (ISKA) menzil duyarliligi -- kalan tek bagimlilik")
    tamam = True

    def _ates_ani(carpan=1.0, ekleme=0.0):
        """r: 60 -> 10 -> 80 profili. ISKA'nin atesledigi GERCEK menzili doner."""
        k = TakipKontrolcu(TakipAyar(iska_zaman_asimi_s=1e9))  # zaman kolu KAPALI
        t = 0.0
        profil = list(np.linspace(60.0, 10.0, 60)) + list(np.linspace(10.0, 80.0, 90))
        for r_ger in profil:
            t += 0.05
            r_bildirilen = r_ger * carpan + ekleme
            k._durum_makinesi(_olcum(r=r_bildirilen, t=t), r_bildirilen, -5.0, 0.05)
            if k.durum == 'ISKA':
                return r_ger, k.iska_sebep
        return None, ''

    taban, sebep = _ates_ani()
    tamam &= _rapor("12a temiz: 'menzil aciliyor' kolu atesliyor",
                    taban is not None and 'aciliyor' in sebep,
                    f"gercek {taban:.1f} m'de -- {sebep[:44]}")

    r_ek, _ = _ates_ani(ekleme=20.0)
    r_x2, _ = _ates_ani(carpan=2.0)
    r_x05, _ = _ates_ani(carpan=0.5)
    hepsi = [x for x in (taban, r_ek, r_x2, r_x05) if x is not None]
    yayilma = (max(hepsi) - min(hepsi)) if len(hepsi) == 4 else float('inf')
    tamam &= _rapor("12b hakem menzil yanliligindan KAYIYOR (olculen acik)",
                    yayilma > 5.0,
                    f"x0.5 {r_x05:.1f} | temiz {taban:.1f} | +20m {r_ek:.1f} | "
                    f"x2.0 {r_x2:.1f} m  (yayilma {yayilma:.1f} m)")

    # GORSEL HAKEM ayni profilde: bbox alani menzille tutarli uretilir
    # (s = sqrt(alan) ~ 1/r), menzil ise BOZULUR. Oran tabanli oldugu icin
    # hakem bozulmayi gormemeli.
    def _ates_ani_alan(carpan=1.0, ekleme=0.0):
        k = TakipKontrolcu(TakipAyar(iska_kaynak='alan',
                                     iska_zaman_asimi_s=1e9))
        t = 0.0
        profil = list(np.linspace(60.0, 10.0, 60)) + list(np.linspace(10.0, 80.0, 90))
        for r_ger in profil:
            t += 0.05
            kenar = 1000.0 / r_ger          # bbox kenari ~ 1/r (gercek geometri)
            o = _olcum(r=r_ger * carpan + ekleme, t=t, alan=kenar)
            alan, alan_hizi = k._alan_guncelle(o, 0.05)
            k._durum_makinesi(o, r_ger * carpan + ekleme, alan_hizi, 0.05)
            if k.durum == 'ISKA':
                return r_ger, k.iska_sebep
        return None, ''

    a_taban, a_sebep = _ates_ani_alan()
    a_ek, _ = _ates_ani_alan(ekleme=20.0)
    a_x2, _ = _ates_ani_alan(carpan=2.0)
    a_x05, _ = _ates_ani_alan(carpan=0.5)
    a_hepsi = [x for x in (a_taban, a_ek, a_x2, a_x05) if x is not None]
    a_yayilma = (max(a_hepsi) - min(a_hepsi)) if len(a_hepsi) == 4 else float('inf')
    tamam &= _rapor("12c GORSEL hakem (iska_kaynak='alan') menzilden BAGIMSIZ",
                    a_yayilma < 1e-9,
                    f"dort kolda da {a_taban:.1f} m -- {a_sebep[:40]}"
                    if a_taban else "ateslemedi")
    tamam &= _rapor("12d gorsel hakem makul menzilde atesliyor",
                    a_taban is not None and 12.0 < a_taban < 45.0,
                    f"{a_taban:.1f} m (menzil hakemi temizde {taban:.1f} m)"
                    if a_taban else '')
    return tamam


def test_gorsel_hakem(hizli=False):
    """GORSEL HAKEM: menzili tamamen devreden cikarinca ne oluyor?

    Test 11: komut yolu menzilden bagimsiz. Test 12: hakem DEGIL.
    Burada iki soru:
      (a) BEDEL: hakemi menzilden alana cevirmek angajman sonucunu bozuyor mu?
      (b) KAZANC: 'alan' hakemiyle sistemin TAMAMI menzilsiz calisiyor mu?
          (yani menzil kablosunu kesip atsak arac ayni mi ucar?)"""
    print("\n13. Gorsel hakem (iska_kaynak='alan') -- menzilsiz sistem")
    tamam = True
    tohumlar = (3,) if hizli else (3, 11, 29)
    sen = [("duz/kuyruk", dict(rota="duz", devir="kuyruk")),
           ("duz/capraz", dict(rota="duz", devir="capraz")),
           ("elips/kuyruk", dict(rota="elips", devir="kuyruk")),
           ("elips/capraz", dict(rota="elips", devir="capraz")),
           ("wand/kuyruk", dict(rota="wanderer", devir="kuyruk")),
           ("wand/capraz", dict(rota="wanderer", devir="capraz"))]

    def _kos(ayar, kw, bozucu=None):
        m = []
        for tohum in tohumlar:
            s = senaryo_kos(ayar=ayar, tohum=tohum, sure=30.0,
                            hedef_hiz_mps=21.05, menzil_bozma=bozucu, **kw)
            m.append(s['min_r'])
        return float(np.median(m))

    # (a) BEDEL: menzil hakemi vs gorsel hakem, TEMIZ menzille.
    # GIMBAL DALI: kilit TEK YONLU yapildi. Gimbal fiziginde gorunen-boy
    # tepesi farkli zamanlanip gorsel hakem wand/capraz'da menzil hakeminden
    # DAHA IYI CPA verdi (11.5 vs 13.2 m); |fark| olcutu bu iyilesmeyi
    # "bedel" sanip dusuyordu. Guvenlik niyeti ayni: gorsel hakem CPA'yi
    # KOTULESTIRMEMELI.
    bedel_max = 0.0
    for ad, kw in sen:
        a1 = _kos(TakipAyar(), kw)
        a2 = _kos(TakipAyar(iska_kaynak='alan'), kw)
        bedel_max = max(bedel_max, a2 - a1)
        print(f"      {ad:13s} menzil hakemi={a1:6.2f}  gorsel={a2:6.2f}")
    tamam &= _rapor("13a gorsel hakemin BEDELI yok (CPA kotulesmiyor)",
                    bedel_max < 0.05,
                    f"en buyuk kotulesme {bedel_max:+.3f} m")

    # (b) KAZANC: gorsel hakem + MENZIL HIC YOK -> hala ayni
    yok = lambda r, t, g: None                                   # noqa: E731
    fark_max = 0.0
    for ad, kw in sen:
        a1 = _kos(TakipAyar(iska_kaynak='alan'), kw)
        a2 = _kos(TakipAyar(iska_kaynak='alan'), kw, bozucu=yok)
        fark_max = max(fark_max, abs(a1 - a2))
    tamam &= _rapor("13b gorsel hakem + MENZIL YOK: sistem AYNEN ucuyor",
                    fark_max < 1e-9,
                    f"6 senaryo x {len(tohumlar)} tohum, en buyuk fark "
                    f"{fark_max:.2e} m")

    # (c) Kalite kapisi: kucuk bbox hakeme GIRMEMELI
    k = TakipKontrolcu(TakipAyar(iska_kaynak='alan'))
    for i in range(30):                       # 5 px bbox = kapinin altinda
        o = _olcum(r=200.0, t=i * 0.05, alan=5.0)
        alan, ah = k._alan_guncelle(o, 0.05)
        k._durum_makinesi(o, 200.0, ah, 0.05)
    tamam &= _rapor("13c kalite kapisi: 5 px bbox tepeyi kirletmiyor",
                    k.s_tepe == 0.0 and k.s_lpf is None,
                    f"s_tepe={k.s_tepe:.1f} kucuk_sayac={k._alan_kucuk_sayac}")

    # (d) 'cok kucuk' kolu = iska_mutlak_m'nin menzilsiz karsiligi
    for i in range(30, 60):
        o = _olcum(r=200.0, t=i * 0.05, alan=5.0)
        alan, ah = k._alan_guncelle(o, 0.05)
        k._durum_makinesi(o, 200.0, ah, 0.05)
    tamam &= _rapor("13d israrla kucuk hedef -> ISKA (menzil okumadan)",
                    k.durum == 'ISKA' and 'cok kucuk' in k.iska_sebep,
                    k.iska_sebep)
    return tamam


def test_ilerleme_saati(hizli=False):
    """ZAMAN ASIMI 'ilerleme' saatine baglaninca ne oluyor?

    Cevrimdisi izde olculen kusur: elips/capraz'da menzil 45 -> 19 m'ye
    inmis, kapanma her ceyrekte artmis (+7.7 m/s) ve tam o anda 8 s DUVAR
    SAATI dolup ISKA ilan edilmis -- kazanan angajman kesilmis.

    'ilerleme' kolunda saat yalniz hedefin gorunen alani BUYUMEZKEN isler.
    Iki sey birden dogrulanmali:
      (a) KAZANC: kapanirken kesilme bitmeli,
      (b) EMNIYET: umutsuz angajman HALA sonlanmali (yoksa arac sonsuza
          kadar bosa kovalar) -- mutlak tavan bunu garanti eder."""
    print("\n14. Ilerleme saati (zaman asimi kapanirken calmasin)")
    tamam = True
    tohumlar = (3,) if hizli else (3, 11, 29)
    sen = [("duz/kuyruk", dict(rota="duz", devir="kuyruk")),
           ("elips/kuyruk", dict(rota="elips", devir="kuyruk")),
           ("elips/capraz", dict(rota="elips", devir="capraz")),
           ("wand/capraz", dict(rota="wanderer", devir="capraz"))]

    def _kos(ayar, kw, **ek):
        m = []
        for tohum in tohumlar:
            s = senaryo_kos(ayar=ayar, tohum=tohum, sure=40.0,
                            hedef_hiz_mps=ek.pop('hedef_hiz_mps', 21.05),
                            **kw, **ek)
            m.append(s)
        return m

    kazanc = {}
    for ad, kw in sen:
        d = float(np.median([s['min_r'] for s in _kos(
            TakipAyar(iska_zaman_kaynak='duz'), kw)]))
        i = float(np.median([s['min_r'] for s in _kos(
            TakipAyar(iska_zaman_kaynak='ilerleme'), kw)]))
        kazanc[ad] = (d, i)
        print(f"      {ad:13s} duz saat={d:6.2f}  ilerleme={i:6.2f}  "
              f"{'KAZANC' if i < d - 0.5 else ('esit' if abs(i - d) < 0.5 else 'KAYIP')}")
    tamam &= _rapor("14a hicbir senaryoda GERILEME yok",
                    all(i <= d + 0.5 for d, i in kazanc.values()),
                    "en kotu fark "
                    f"{max(i - d for d, i in kazanc.values()):+.2f} m")
    tamam &= _rapor("14b elips/capraz'da buyuk kazanc",
                    kazanc["elips/capraz"][1] < 0.5 * kazanc["elips/capraz"][0],
                    f"{kazanc['elips/capraz'][0]:.2f} -> "
                    f"{kazanc['elips/capraz'][1]:.2f} m")

    # (c) EMNIYET: hedef bizden HIZLI -> hic kapanamayiz. Saat ilerlemeyi
    #     goremeyecegi icin normal isler ve ISKA ilan edilmeli.
    k = TakipKontrolcu(TakipAyar(iska_zaman_kaynak='ilerleme'))
    k.tohumla(None)
    t = 0.0
    for i in range(400):                      # 20 s, menzil hep aciliyor
        t += 0.05
        r = 40.0 + i * 0.05
        o = _olcum(r=r, t=t, alan=max(1.0, 1000.0 / r))
        alan, ah = k._alan_guncelle(o, 0.05)
        k._durum_makinesi(o, r, ah, 0.05)
        if k.durum == 'ISKA':
            break
    tamam &= _rapor("14c kapanmayan angajman HALA sonlaniyor",
                    k.durum == 'ISKA' and t < 12.0,
                    f"t={t:.1f} s -- {k.iska_sebep[:44]}")

    # (d) EMNIYET-2: alan SURTUNMESIZ buyuyor ama hic carpmiyoruz
    #     (patolojik) -> mutlak tavan devreye girmeli.
    k = TakipKontrolcu(TakipAyar(iska_zaman_kaynak='ilerleme',
                                 iska_mutlak_sure_s=12.0))
    k.tohumla(None)
    t = 0.0
    for i in range(600):
        t += 0.05
        # USSEL yaklasma: r = 60*exp(-t/30). Bagil alan buyumesi HER AN
        # 2/30 = 0.067 1/s (esik 0.05'in ustunde) ama menzil sonlu surede
        # SIFIRA INMEZ -> ilerleme saati asla dolmaz. Tek durduran mutlak
        # tavandir; testin sinadigi tam olarak budur.
        r = 60.0 * math.exp(-t / 30.0)
        o = _olcum(r=r, t=t, alan=1000.0 / r)
        alan, ah = k._alan_guncelle(o, 0.05)
        k._durum_makinesi(o, r, ah, 0.05)
        if k.durum == 'ISKA':
            break
    tamam &= _rapor("14d mutlak tavan (emniyet supabi) calisiyor",
                    k.durum == 'ISKA' and t < 16.0,
                    f"t={t:.1f} s -- {k.iska_sebep[:44]}")
    return tamam


def test_ablasyonlar(hizli=False):
    """Yasanin knob'lari kapali donguda ne yapiyor (A/B, ayni tohum)."""
    print("\n9. Ablasyonlar (duz rota, hedef 21.05 m/s, tohum 3)")
    tamam = True
    VARS = "VARSAYILAN (tavan, klasik, fren kapali)"
    kollar = [
        (VARS, TakipAyar()),
        ("saf mode_follow ('p', kp 1.0)", TakipAyar(hiz_kaynagi='p')),
        ("saf mode_follow + AP kp=0.1", TakipAyar(hiz_kaynagi='p', kp=0.1)),
        ("fren ACIK (AP 'yaninda dur')", TakipAyar(fren='ap')),
        ("poscon yon (Copter>=4.5)", TakipAyar(yasa='poscon')),
        ("standoff ofset 25/6", TakipAyar(ofs_geri_m=25.0, ofs_asagi_m=6.0)),
        ("yaw ablasyonu (--no-yaw)", TakipAyar(yaw_komutu_ver=False)),
        ("ivme sekillendirme KAPALI", TakipAyar(ivme_sekillendirme_mps2=0.0)),
    ]
    sonuc = {}
    for ad, ayar in kollar:
        s = senaryo_kos(rota="duz", devir="kuyruk", ayar=ayar, tohum=3,
                        sure=30.0, hedef_hiz_mps=21.05)
        sonuc[ad] = s
        print(f"      {ad:42s} min={s['min_r']:6.2f} m  bitis={s['bitis']:10s}"
              f"  kapanma={s['kapanma_ort']:+5.1f}  cmd_p95={s['cmd_p95']:5.1f}"
              f"  kayip={s['kayip_pct']:4.1f}%")
    tamam &= _rapor("9a saf 'p' kolu kapatamiyor (denge mesafesi)",
                    sonuc["saf mode_follow ('p', kp 1.0)"]['min_r']
                    > 4.0 * sonuc[VARS]['min_r'],
                    "hiz_kaynagi karari kapali donguda")
    tamam &= _rapor("9b AP kp=0.1 hic kapatmiyor",
                    sonuc["saf mode_follow + AP kp=0.1"]['min_r'] > 25.0)
    tamam &= _rapor("9c fren ACIK kapanmayi olduruyor",
                    sonuc["fren ACIK (AP 'yaninda dur')"]['min_r']
                    > sonuc[VARS]['min_r'],
                    "sapma (3)'un kapali dongu kaniti")
    # GIMBAL DALI: sekillendirmenin kadraj gerekcesi oldu (ivme -> pitch ->
    # SABIT kamera savrulur zinciri koptu; olculen: iki kolda da kayip %0).
    # Kilit, gimbal kazancini koruyan MUTLAK sagliga cevrildi: sekillendirme
    # olsun olmasin kadraj korunmali ve kapanma surmeli. Bozulursa ya tesis
    # gimbal modeli ya kisit katmani gerilemistir.
    tamam &= _rapor("9d gimbal fiziginde kadraj sekillendirmesiz de korunuyor",
                    sonuc["ivme sekillendirme KAPALI"]['kayip_pct'] < 5.0
                    and sonuc[VARS]['kayip_pct'] < 5.0
                    and sonuc["ivme sekillendirme KAPALI"]['min_r'] < 3.0,
                    f"kayip %{sonuc[VARS]['kayip_pct']:.1f} / "
                    f"%{sonuc['ivme sekillendirme KAPALI']['kayip_pct']:.1f}, "
                    f"min {sonuc['ivme sekillendirme KAPALI']['min_r']:.2f} m")
    return tamam


def test_sure():
    """Dongu maliyeti: bu yasa MPC'nin cozucusune gore ihmal edilebilir olmali."""
    print("\n10. Dongu maliyeti")
    k = TakipKontrolcu(TakipAyar())
    k.tohumla(None)
    o = _olcum(ex=3.0, ey=-8.0, r=35.0)
    for i in range(50):                      # isinma
        o.t = i * 0.05
        k.komut(o)
    t0 = time.perf_counter()
    N = 2000
    for i in range(N):
        o.t = 10.0 + i * 0.05
        k.komut(o)
    us = (time.perf_counter() - t0) / N * 1e6
    return _rapor("10a komut() < 200 us", us < 200.0, f"{us:.1f} us/dongu "
                  f"(MPC cozucusu p95 ~13000 us)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--hizli', action='store_true',
                   help='kapali dongu testlerinde tek tohum')
    args = p.parse_args()
    print("=" * 74)
    print("takip_gudum.py -- ArduPilot FOLLOW yasasi testleri")
    print("=" * 74)
    test_motor_muhru()
    test_sqrt_controller()
    test_hedef_kestirimi()
    test_hiz_yasasi()
    test_kp_gerekcesi()
    test_ofset_ve_yaw()
    test_durum_makinesi()
    test_menzil_bagimsizligi()
    test_komut_yolu()
    test_kapali_dongu(args.hizli)
    test_menzil_bozulmasi(args.hizli)
    test_hakem_menzil_duyarliligi()
    test_gorsel_hakem(args.hizli)
    test_ilerleme_saati(args.hizli)
    test_ablasyonlar(args.hizli)
    test_sure()
    print("\n" + "=" * 74)
    print(f"SONUC: {len(GECTI)}/{len(GECTI) + len(KALDI)} gecti")
    if KALDI:
        for ad in KALDI:
            print(f"  KALDI: {ad}")
        sys.exit(1)
    print("TUM TESTLER GECTI")


if __name__ == '__main__':
    main()
