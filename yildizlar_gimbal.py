#!/usr/bin/env python3
"""
yildizlar_gimbal.py - govde salinimini matematiksel olarak temizleyen kamera katmani
================================================================================
Mimari bumblebee/teva.py'den BIREBIR tasindi (stabilize_pixel, satir 841-864);
degisen tek sey kamera parametreleri ve montaj acisi. Sebep: fiziksel gimbal
yok, kamera govdeye SABIT. Kopter/ucak yattikca hedefin kadrajdaki yeri
gercekte hic hareket etmese bile kayiyor; gudum bu kaymayi hedef hareketi
sanip pesine dusuyordu.

ZINCIR (teva.py ile ayni sira - SIRA ONEMLI):
    r_cam   = K^-1 @ [px, py, 1]           ham piksel -> kamera isini
    r_body  = R_mount @ (R_c_b @ r_cam)    FIZIKSEL montaj acisi (+30 yukari)
                                           -> de-rotasyondan ONCE, cunku kamera
                                              govdeyle BIRLIKTE yatar
    r_stab  = R_stab @ r_body              govde roll/pitch'i cikar (yaw=0)
    r_virt  = R_aim @ r_stab               AIM ofseti -> de-rotasyondan SONRA,
                                           cunku "hedef ufka gore su kadar
                                           yukarida dursun" demek; ufka bagli.
                                           Eskiden (a) ile ayni yerde
                                           uygulandigi icin ucak yattikca
                                           delta*sin(roll) kadar SAHTE yatay
                                           hata uretiyordu.
    piksel  = K @ (R_c_b^T @ r_virt)

IKI AYRI BUYUKLUK (teva.py'nin en kritik ayrimi):
  mount_phys_pitch_deg : GERCEK fiziksel montaj acisi. Bu ortamda +30 derece
      (models/suru_drone_*/model.sdf, sensor pose pitch; olcumle dogrulandi:
      hedef +26.26 derece yukseliste iken bbox y=534 -> eksen +29.4 derece).
      teva.py'de sim icin 0.0 idi cunku orada kamera govdeye TAM PARALELDI.
  aim_pitch_deg : "hedef kadrajda nerede dursun" DC ofseti. Sanal gimbal AC
      (govde salinimi) gurultusunu temizler; bu DC bileseni elle verilir.
      BAGINTI (teva.py:709-714 ile ayni):
          sanal kadraj merkezi ufka gore  = -aim
      yani hedefi ufka gore eps derecede tutmak icin  aim = -eps.
      Konumlu gudumun standoff geometrisi eps'i belirler (back/down), o
      yuzden bu deger PLANA DEGIL yaklasma geometrisine baglidir.

AIM MENZIL SONUMLEMESI (teva.py:815-822): aim yakinken TAM uygulanir, uzakta
dogrusal olarak sifire iner. Uzakta tam aim uygulamak, kucuk aci farkinin
buyuk irtifa farki istemesine ve komutun doygunlasmasina yol aciyordu.

AIM YALNIZ DIKEY KANALIN ISI (2026-08-02'de olculdu ve duzeltildi): R_aim bir
Ry donusudur, yani yatay aciyi da dondurur. Kucuk yan aci (psi) limitinde
sanal kadrajda okunan yatay aci, gercek kerterizin
      kazanc = cos(eps) / cos(eps + aim)
kadar SIKISTIRILMIS halidir (eps = hedefin ufka gore yukselisi). aim=0'da
kazanc 1.000; nominal calisma noktasinda (hedef sanal merkezde, aim = -eps)
minimuma, cos(eps)'e iner. Canli veride (run/kanit/gimbal4.csv, aim=-27.47,
eps~24) olculen kazanc 0.909 idi: gudum kerterizi %8.8 DUSUK okuyordu, yani
hedefe donus surekli eksik komutlaniyordu. Aim'in isi dikey DC ofsetidir;
yatay kanalda hicbir isi yoktur. Bu yuzden aci_hatasi() yatay bileseni AIM
UYGULANMADAN ONCEKI stabilize isindan, dikey bileseni aim sonrasi isindan
alir. Dikey kanal degismedi; aim=0'da iki yol sayisal olarak ozdestir.
"""

import argparse
import math

import numpy as np

# Kamera -> govde eksen donusumu (teva.py:121). Kamera: x sag, y asagi,
# z ileri (OpenCV). Govde: x ileri, y sag, z asagi (NED).
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T


def compute_R_b_e(roll, pitch, yaw):
    """Govde -> yer (NED) yonelim matrisi (teva.py:126)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _ry(deg):
    """Y (pitch) ekseni etrafinda donme. POZITIF = isini YUKARI cevirir."""
    r = math.radians(deg)
    return np.array([[math.cos(r), 0.0, math.sin(r)],
                     [0.0,         1.0, 0.0],
                     [-math.sin(r), 0.0, math.cos(r)]])


def eklem_acisi(eps_cam_deg, pitch_rad, roll_rad):
    """FIZIKSEL GIMBAL (gimbal dali): kameranin dunya elevasyonundan eklem
    acisini cikarir.

    Stabilize gimbal plugin'i kamera eksenini dunya elevasyonu eps'te tutar;
    olcum zincirinin ihtiyaci ise kameranin GOVDEYE gore acisidir (eski
    'mount' kavraminin canli karsiligi). Kamera ekseni govdede _ry(q) ile
    donmus isin oldugundan dunya elevasyonu:

        eps = asin( sin(pitch)*cos(q) + cos(pitch)*cos(roll)*sin(q) )

    Ters cozum (A = sin(pitch), B = cos(pitch)*cos(roll)):
        q = asin( sin(eps) / sqrt(A^2+B^2) ) - atan2(A, B)

    eps_cam_deg: kameranin dunya elevasyonu [deg] (gimbal_tilt_status ya da
    yavas DC rejimde komutlanan deger). Donen deger: q [deg], pozitif=yukari.
    Roll=0'da q = eps - pitch'e sadelesir.
    """
    A = math.sin(pitch_rad)
    B = math.cos(pitch_rad) * math.cos(roll_rad)
    R = math.hypot(A, B)
    s = math.sin(math.radians(eps_cam_deg)) / max(R, 1e-9)
    s = max(-1.0, min(1.0, s))
    return math.degrees(math.asin(s) - math.atan2(A, B))


class SanalGimbal:
    """Ham bbox pikselini govde salinimindan arindirilmis piksele cevirir."""

    def __init__(self, width=1280, height=720, hfov_rad=1.1519,
                 mount_phys_pitch_deg=0.0, aim_pitch_deg=0.0,
                 aim_tam_menzil_m=120.0, aim_sifir_menzil_m=250.0,
                 cx=None, cy=None):
        self.width, self.height = width, height
        self.hfov_rad = hfov_rad
        # Raspberry Pi AI Camera (IMX500) 1280x720: hfov 66 derece
        # -> fx = fy = (1280/2)/tan(33 deg) = 985.5
        self.fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = self.fx                      # kare piksel
        self.cx = width / 2.0 if cx is None else cx
        self.cy = height / 2.0 if cy is None else cy

        self.mount_phys_pitch_deg = float(mount_phys_pitch_deg)
        self.aim_pitch_deg = float(aim_pitch_deg)
        self.aim_tam_menzil_m = float(aim_tam_menzil_m)
        self.aim_sifir_menzil_m = float(aim_sifir_menzil_m)

        self.K = np.array([[self.fx, 0.0, self.cx],
                           [0.0, self.fy, self.cy],
                           [0.0, 0.0, 1.0]])
        self.K_inv = np.linalg.inv(self.K)
        self.R_mount_phys = _ry(self.mount_phys_pitch_deg)
        self._son_aim_deg = None
        self.R_aim = _ry(self.aim_pitch_deg)

        self.vfov_rad = 2 * math.atan(math.tan(hfov_rad / 2) * height / width)

    # ---------------------------------------------------------------- aim

    def aim_etkin_deg(self, menzil_m):
        """Menzil-farkinda aim (teva.py:815-822).

        menzil None ise tam aim uygulanir (menzil bilgisi yoksa sonumleyecek
        bir sey de yok).
        """
        if menzil_m is None:
            return self.aim_pitch_deg
        r = float(menzil_m)
        if r <= self.aim_tam_menzil_m:
            k = 1.0
        elif r >= self.aim_sifir_menzil_m:
            k = 0.0
        else:
            k = ((self.aim_sifir_menzil_m - r)
                 / (self.aim_sifir_menzil_m - self.aim_tam_menzil_m))
        return self.aim_pitch_deg * k

    # -------------------------------------------------------------- ileri

    def _R_kamera_govde(self, eklem_deg):
        """Kamera->govde donusu: fiziksel gimbal varsa CANLI eklem acisi
        (eklem_deg, pozitif=yukari), yoksa sabit montaj. Sabit montaj yolu
        eski davranisla BIREBIR ayni (geri uyum)."""
        if eklem_deg is None:
            return self.R_mount_phys
        return _ry(float(eklem_deg))

    def piksel_uret(self, yukselis_deg, yan_deg, roll_rad, pitch_rad,
                    eklem_deg=None):
        """TERS yon: ufka gore (yukselis, yan) yonundeki hedef HAM kadrajda
        nereye duser? Yalniz dogrulama/test icin; ucus yolunda kullanilmaz.
        """
        eps, psi = math.radians(yukselis_deg), math.radians(yan_deg)
        r_stab = np.array([math.cos(eps) * math.cos(psi),
                           math.cos(eps) * math.sin(psi),
                           -math.sin(eps)])
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0)
        r_body = R_stab.T @ r_stab
        r_cam_body = self._R_kamera_govde(eklem_deg).T @ r_body
        r_cam = R_c_b_T @ r_cam_body
        if r_cam[2] <= 1e-9:
            return None                     # kameranin ARKASINDA
        p = self.K @ r_cam
        return p[0] / p[2], p[1] / p[2]

    def kadrajda_mi(self, px, py):
        return px is not None and 0 <= px < self.width and 0 <= py < self.height

    # ------------------------------------------------------------ stabil

    def stabilize(self, px, py, roll_rad, pitch_rad, menzil_m=None,
                  eklem_deg=None):
        """Ham piksel -> sanal (ufka hizali) piksel. teva.py:841-864.

        eklem_deg verilirse (fiziksel gimbal) sabit montaj yerine canli eklem
        acisi kullanilir; R_stab AYNEN kalir cunku R_stab @ _ry(eklem) tam
        olarak kameranin gercek dunya yonelimidir (roll de-rotasyonu dahil).
        """
        r_cam = self.K_inv @ np.array([px, py, 1.0])
        r_body = self._R_kamera_govde(eklem_deg) @ (R_c_b @ r_cam)  # de-rot ONCE
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0)

        aim_e = self.aim_etkin_deg(menzil_m)
        if aim_e != self._son_aim_deg:
            self._son_aim_deg = aim_e
            self.R_aim = _ry(aim_e)
        r_virt_body = self.R_aim @ (R_stab @ r_body)     # aim: de-rot SONRA
        r_virt_cam = R_c_b_T @ r_virt_body
        p = self.K @ r_virt_cam
        if p[2] == 0:
            return px, py
        return p[0] / p[2], p[1] / p[2]

    def aci_hatasi(self, px, py, roll_rad, pitch_rad, menzil_m=None,
                   eklem_deg=None):
        """Sanal kadrajda merkezden sapma (derece). Gudumun kullanacagi buyukluk.

        DIKEY aim SONRASI, YATAY aim ONCESI isindan okunur - gerekce modul
        docstring'inde (aim bir Ry donusu oldugu icin yatay aciyi
        cos(eps)/cos(eps+aim) kadar sikistirir; olculen kayip %8.8).
        """
        # DIKEY: aim uygulanmis isindan (DC ofset kasitlidir).
        sx, sy = self.stabilize(px, py, roll_rad, pitch_rad, menzil_m,
                                eklem_deg=eklem_deg)
        ey = math.degrees(math.atan((sy - self.cy) / self.fy))
        # YATAY: aim ONCESI isindan. Zincirin ilk iki adimi stabilize() ile
        # birebir ayni; yalniz R_aim uygulanmaz. Kerteriz dogrudan govde-yatay
        # isinin bilesenlerinden okunur (atan2: kadraj disi/genis acida da
        # dogru cehreyi verir).
        r_cam = self.K_inv @ np.array([px, py, 1.0])
        r_body = self._R_kamera_govde(eklem_deg) @ (R_c_b @ r_cam)
        r_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0) @ r_body
        ex = math.degrees(math.atan2(r_stab[1], r_stab[0]))
        return ex, ey

    def menzil_tahmin(self, bbox_w_px, hedef_genislik_m=1.6):
        """bbox genisliginden kaba menzil (m). Sadece AIM SONUMLEMESI icin.

        menzil = hedef_genisligi * fx / bbox_genisligi
        Hedefin gorunen genisligi olcumle 1.6 m alindi (120 m'de 7 px @
        56 px/derece -> 1.57 m). Kaba olmasi yeterli: sonumleme 120-250 m
        arasinda dogrusal bir rampa, bir kac metre hata rampada kayar sadece.
        Gudum menzili buradan ALMAZ; o telemetriden gelir.
        """
        if bbox_w_px is None or bbox_w_px < 1:
            return None
        return hedef_genislik_m * self.fx / float(bbox_w_px)

    def ozet(self):
        return (f"SanalGimbal {self.width}x{self.height} "
                f"hfov={math.degrees(self.hfov_rad):.1f} "
                f"vfov={math.degrees(self.vfov_rad):.1f} deg | "
                f"fx=fy={self.fx:.1f} cx={self.cx:.0f} cy={self.cy:.0f} | "
                f"mount_phys={self.mount_phys_pitch_deg:+.2f} "
                f"aim={self.aim_pitch_deg:+.2f} "
                f"(tam<{self.aim_tam_menzil_m:.0f}m, sifir>{self.aim_sifir_menzil_m:.0f}m)")


def analitik_aim(back_m, down_m):
    """(a) sikki: aim'i KENDI komut geometrinden hesapla, olcum gerekmez.

    Standoff takibinde hedefin ufka gore yukselisi tamamen senin verdigin
    slot geometrisidir:  eps = atan(down/back).  Sanal kadraj merkezi -aim
    oldugundan  aim = -eps.
    Olcumle dogrulandi (back=25, down=13 -> -27.47): elips -29.25,
    duz -27.03. (wanderer -10.52 idi; o plan irtifa degistirdigi icin
    slot geometrisi korunmuyor - bu yuzden trim gerekiyor.)
    """
    return -math.degrees(math.atan(float(down_m) / max(1e-6, float(back_m))))


class AimTrim:
    """aim'i analitik baslangictan KUCUK bir bantta YAVASCA duzeltir.

    NICIN VAR: analitik deger (a) senin komut geometrini bilir ama ucagin
    GERCEK montaj hatasini (teva.py'de olculen ~-1 derece kamera-otopilot
    acisi), hucum acisini ve slot takip hatasini bilmez. Bunlar DC
    buyukluklerdir; sanal gimbal AC'yi (govde salinimini) temizler, DC'yi
    bilerek elde birakir.

    NICIN YAVAS VE KELEPCELI: hizli bir dongu kurmak sanal gimbali
    SADELESTIRIR - aim hedefi kovalamaya baslarsa "sanal kadraj merkezi"
    artik sabit bir referans olmaktan cikar, DC/AC ayrimi coker ve elde
    kalan sey gizli bir takip dongusudur. Bunu engelleyen dort koruma:

      1. KELEPCE   : aim yalnizca analitik degerin +-kelepce_deg icinde
                     gezebilir. Montaj/AOA hatasini yutar, hedefi kovalayamaz.
      2. HIZ SINIRI: max_hiz_dps derece/saniye. Kelepceyi bastan basa
                     gecmesi dakikalar surer -> hedef dinamigiyle karisamaz.
      3. MENZIL KAPISI: yalnizca aim'in TAM uygulandigi yakin bantta guncelle.
                     Uzakta aim zaten sonumlendigi icin ey'in anlami yok.
      4. AYKIRI DEGER REDDI: |ey| buyukse (hedef kadraj kenarinda, muhtemelen
                     kismi tespit) ornek atilir.
    """

    def __init__(self, baslangic_deg, kelepce_deg=6.0, tau_s=25.0,
                 max_hiz_dps=0.15, menzil_max_m=150.0, ey_max_deg=12.0,
                 min_ornek=30):
        self.baslangic = float(baslangic_deg)
        self.aim = float(baslangic_deg)
        self.kelepce = float(kelepce_deg)
        self.tau = float(tau_s)
        self.max_hiz = float(max_hiz_dps)
        self.menzil_max = float(menzil_max_m)
        self.ey_max = float(ey_max_deg)
        self.min_ornek = int(min_ornek)
        self.ornek = 0
        self.kelepcede = False

    def guncelle(self, ey_deg, menzil_m, dt):
        """ey_deg: sanal kadrajdaki DIKEY aci hatasi. Yeni aim'i dondurur.

        ey<0 = hedef merkezin USTUNDE. Merkeze indirmek icin sanal merkezi
        yukari almak, yani aim'i AZALTMAK gerekir (merkez = -aim).
        """
        if ey_deg is None or dt <= 0:
            return self.aim
        if abs(ey_deg) > self.ey_max:
            return self.aim                      # aykiri: kadraj kenari
        if menzil_m is not None and menzil_m > self.menzil_max:
            return self.aim                      # uzak: aim zaten sonumlu
        self.ornek += 1
        if self.ornek < self.min_ornek:
            return self.aim                      # once yeterli kanit birikssin

        # Birinci mertebe: aim, ey'i sifira goturecek yonde tau ile suzulur.
        istenen = self.aim + ey_deg
        adim = (istenen - self.aim) * (dt / max(self.tau, 1e-3))
        sinir = self.max_hiz * dt
        adim = max(-sinir, min(sinir, adim))
        yeni = self.aim + adim
        alt, ust = self.baslangic - self.kelepce, self.baslangic + self.kelepce
        self.kelepcede = not (alt < yeni < ust)
        self.aim = max(alt, min(ust, yeni))
        return self.aim

    def ozet(self):
        return (f"AimTrim: baslangic={self.baslangic:+.2f} "
                f"suanki={self.aim:+.2f} (kelepce +-{self.kelepce:.1f}, "
                f"tau={self.tau:.0f}s, max {self.max_hiz:.2f} deg/s, "
                f"menzil<{self.menzil_max:.0f}m)"
                f"{'  [KELEPCEDE]' if self.kelepcede else ''}")


# ====================================================================== test

def statik_test(g, verbose=True):
    """Sanal gimbalin ASIL VAADINI dogrular:

    hedef ufka gore SABIT dururken govde yattikca HAM piksel kayar, ama
    SANAL piksel kaymamalidir. Ileri model (piksel_uret) ile geri model
    (stabilize) birbirinin tersi oldugundan bu kapali bir dogrulamadir.
    """
    print(g.ozet())
    print()
    print("--- STATIK TEST: hedef sabit, govde saliniyor ---")
    basarili = True
    for yuk, yan in ((30.0, 0.0), (25.0, +8.0), (35.0, -8.0), (30.0, +15.0)):
        ham, sanal = [], []
        for roll_d in (-30, -15, 0, 15, 30):
            for pitch_d in (-20, -10, 0, 10, 20):
                p = g.piksel_uret(yuk, yan, math.radians(roll_d), math.radians(pitch_d))
                if p is None or not g.kadrajda_mi(*p):
                    continue
                ham.append(p)
                sanal.append(g.stabilize(p[0], p[1], math.radians(roll_d),
                                         math.radians(pitch_d), menzil_m=None))
        if len(sanal) < 4:
            print(f"  yukselis={yuk:+5.1f} yan={yan:+5.1f}: yeterli ornek yok "
                  f"({len(sanal)}) - kadraj disi")
            continue
        ham = np.array(ham); sanal = np.array(sanal)
        ham_yayilim = float(np.hypot(*(ham.max(axis=0) - ham.min(axis=0))))
        sanal_yayilim = float(np.hypot(*(sanal.max(axis=0) - sanal.min(axis=0))))
        # BEKLENEN SANAL KONUM: sanal kadrajin merkezi ufka gore -aim'dedir,
        # montaj acisinda DEGIL (montaj yalnizca hangi hedefin fiziksel olarak
        # kadraja girdigini belirler). Yani sanal goruntu, montaji -aim olan
        # ve hic yatmayan bir kameranin ham goruntusudur.
        bekl = SanalGimbal(g.width, g.height, g.hfov_rad,
                           mount_phys_pitch_deg=-g.aim_pitch_deg
                           ).piksel_uret(yuk, yan, 0.0, 0.0)
        sapma = float(np.hypot(*(sanal.mean(axis=0) - np.array(bekl))))
        ok = sanal_yayilim < 0.5 and sapma < 0.5
        basarili &= ok
        print(f"  yukselis={yuk:+5.1f} yan={yan:+5.1f} ({len(sanal):2d} tutum): "
              f"HAM yayilim {ham_yayilim:7.1f} px -> SANAL {sanal_yayilim:5.2f} px, "
              f"beklenenden sapma {sapma:.2f} px  {'OK' if ok else 'HATA'}")
    print()
    print("--- FIZIKSEL GIMBAL: govde salinirken EKLEM telafi ediyor ---")
    # gimbal dali (2026-08-05): stabilize plugin kamera eksenini dunya
    # elevasyonu eps_cmd'de tutar -> eklem q = eklem_acisi(eps_cmd, pitch,
    # roll). Zincire sabit mount yerine canli q verilince sanal piksel yine
    # kimildamamali. Ayrica q, roll=0'da (eps - pitch)'e sadelesmeli.
    g_fiz = SanalGimbal(g.width, g.height, g.hfov_rad,
                        mount_phys_pitch_deg=0.0, aim_pitch_deg=0.0)
    eps_cmd = 10.0
    for yuk, yan in ((10.0, 0.0), (5.0, +8.0), (15.0, -8.0)):
        sanal = []
        for roll_d in (-30, -15, 0, 15, 30):
            for pitch_d in (-20, -10, 0, 10, 20):
                q = eklem_acisi(eps_cmd, math.radians(pitch_d),
                                math.radians(roll_d))
                p = g_fiz.piksel_uret(yuk, yan, math.radians(roll_d),
                                      math.radians(pitch_d), eklem_deg=q)
                if p is None or not g_fiz.kadrajda_mi(*p):
                    continue
                sanal.append(g_fiz.stabilize(p[0], p[1], math.radians(roll_d),
                                             math.radians(pitch_d),
                                             menzil_m=None, eklem_deg=q))
        sanal = np.array(sanal)
        bekl = SanalGimbal(g.width, g.height, g.hfov_rad,
                           mount_phys_pitch_deg=0.0).piksel_uret(yuk, yan, 0.0, 0.0)
        yayilim = float(np.hypot(*(sanal.max(axis=0) - sanal.min(axis=0))))
        sapma = float(np.hypot(*(sanal.mean(axis=0) - np.array(bekl))))
        ok = yayilim < 0.5 and sapma < 0.5
        basarili &= ok
        print(f"  yukselis={yuk:+5.1f} yan={yan:+5.1f} ({len(sanal):2d} tutum, "
              f"eps_cmd={eps_cmd:+.1f}): SANAL yayilim {yayilim:5.2f} px, "
              f"sapma {sapma:.2f} px  {'OK' if ok else 'HATA'}")
    q0 = eklem_acisi(eps_cmd, math.radians(7.0), 0.0)
    sadelesme_ok = abs(q0 - (eps_cmd - 7.0)) < 1e-9
    basarili &= sadelesme_ok
    print(f"  roll=0 sadelesmesi: q({eps_cmd:.0f}, pitch=7) = {q0:.6f} "
          f"(beklenen {eps_cmd-7.0:.1f})  {'OK' if sadelesme_ok else 'HATA'}")

    print()
    print("--- AIM OFSETI: sanal kadraj merkezi ufka gore nerede? ---")
    for aim in (0.0, -10.0, -25.0, -30.0):
        g2 = SanalGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                         aim_pitch_deg=aim)
        # merkeze dusen yukselisi ara
        en_iyi, en_kucuk = None, 1e9
        for eps10 in range(-600, 601):
            eps = eps10 / 10.0
            p = g2.piksel_uret(eps, 0.0, 0.0, 0.0)
            if p is None:
                continue
            s = g2.stabilize(p[0], p[1], 0.0, 0.0, menzil_m=None)
            d = abs(s[1] - g2.cy)
            if d < en_kucuk:
                en_kucuk, en_iyi = d, eps
        print(f"  aim={aim:+6.1f} -> sanal merkez yukselisi {en_iyi:+6.1f} deg "
              f"(baginti: -aim = {-aim:+6.1f})")
    print()
    print("--- AIM YATAY KAZANC: aim yatay kanala SIZMAMALI ---")
    # R_aim bir Ry donusudur; eskiden yatay aci da aim SONRASI kadrajdan
    # okunuyordu ve kerteriz cos(eps)/cos(eps+aim) kadar SIKISIYORDU. Canli
    # veride (gimbal4.csv: aim=-27.47, eps~24) bu kayip 0.909 olculdu -
    # gudum kerterizi %8.8 dusuk goruyordu. Duzeltmeden sonra kazanc, aim ne
    # olursa olsun ve govde nasil yatarsa yatsin 1.000 olmalidir.
    yan_ref = 3.0
    kazanc_ok = True
    for aim, eps in ((0.0, 30.0), (-10.0, 30.0), (-27.47, 24.0),
                     (-27.47, 30.0), (-30.0, 35.0), (-20.0, 20.0)):
        g4 = SanalGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                         aim_pitch_deg=aim)
        oranlar = []
        for roll_d in (-20, 0, 20):
            for pitch_d in (-10, 0, 10):
                p = g4.piksel_uret(eps, yan_ref, math.radians(roll_d),
                                   math.radians(pitch_d))
                if p is None or not g4.kadrajda_mi(*p):
                    continue
                ex, _ = g4.aci_hatasi(p[0], p[1], math.radians(roll_d),
                                      math.radians(pitch_d), menzil_m=None)
                oranlar.append(ex / yan_ref)
        if not oranlar:
            print(f"  aim={aim:+6.2f} eps={eps:+5.1f}: kadraj disi - atlandi")
            continue
        en_kotu = max(oranlar, key=lambda k: abs(k - 1.0))
        # Duzeltme ONCESI kodun verecegi kazanc (kucuk psi limiti):
        eski = math.cos(math.radians(eps)) / math.cos(math.radians(eps + aim))
        ok = abs(en_kotu - 1.0) < 0.01
        kazanc_ok &= ok
        print(f"  aim={aim:+6.2f} eps={eps:+5.1f} ({len(oranlar)} tutum): "
              f"kazanc {en_kotu:.4f} (eski kod {eski:.4f})  "
              f"{'OK' if ok else 'HATA'}")
    print(f"  yatay kazanc 1.000+-0.01 mi: {'EVET' if kazanc_ok else 'HAYIR'}")
    print()
    print("--- MENZIL SONUMLEMESI ---")
    g3 = SanalGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                     aim_pitch_deg=-28.0)
    for m in (50, 120, 185, 250, 400):
        print(f"  menzil {m:4d} m -> etkin aim {g3.aim_etkin_deg(m):+6.2f} deg")
    print()
    print("--- AIM TRIM: kelepce ve hiz sinirinin ISLEDIGI ---")
    # Kotu senaryo: hedef surekli 10 derece ustte gorunuyor (buyuk DC hata).
    # Trim onu yutmali AMA kelepceyi asmamali ve yavas gitmeli.
    t = AimTrim(baslangic_deg=-27.0, kelepce_deg=6.0, tau_s=25.0,
                max_hiz_dps=0.15, min_ornek=0)
    dt = 1.0 / 30
    for saniye in (0, 10, 30, 60, 120, 300):
        hedef_sn = saniye
        while getattr(t, '_t', 0) < hedef_sn:
            t.guncelle(-10.0, 80.0, dt)
            t._t = getattr(t, '_t', 0) + dt
        print(f"  t={saniye:4d}s  aim={t.aim:+7.3f}"
              f"{'  [KELEPCEDE]' if t.kelepcede else ''}")
    kelepce_ok = abs(t.aim - (-27.0)) <= 6.0 + 1e-6
    print(f"  kelepce korundu mu: {'EVET' if kelepce_ok else 'HAYIR'} "
          f"(|aim - baslangic| = {abs(t.aim + 27.0):.3f} <= 6.0)")

    print()
    print("--- AIM TRIM: kapilar ---")
    t2 = AimTrim(-27.0, min_ornek=0)
    a0 = t2.aim
    t2.guncelle(-30.0, 80.0, 1.0);  print(f"  aykiri deger (|ey|=30>12) -> degisim {t2.aim-a0:+.4f} (0 olmali)")
    t2.guncelle(-5.0, 400.0, 1.0);  print(f"  uzak menzil (400>150 m)   -> degisim {t2.aim-a0:+.4f} (0 olmali)")
    t2.guncelle(-5.0, 80.0, 1.0);   print(f"  gecerli ornek             -> degisim {t2.aim-a0:+.4f} (!=0 olmali)")
    kapi_ok = abs(t2.aim - a0) > 0

    print()
    print("--- ANALITIK BASLANGIC ---")
    for b, d in ((25, 13), (25, 9), (40, 20), (30, 5)):
        print(f"  back={b:3d} down={d:3d} -> aim = {analitik_aim(b, d):+7.2f} deg")

    print()
    print("--- MENZIL TAHMINI (bbox genisliginden) ---")
    for w in (300, 150, 60, 20):
        print(f"  bbox {w:4d} px -> ~{g.menzil_tahmin(w):6.1f} m")

    basarili = basarili and kelepce_ok and kapi_ok and kazanc_ok
    print()
    print("SONUC:", "TUM STATIK TESTLER GECTI" if basarili else "TEST BASARISIZ")
    return basarili


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--test', action='store_true', help='statik dogrulama kos')
    p.add_argument('--mount', type=float, default=30.0)
    p.add_argument('--aim', type=float, default=0.0)
    p.add_argument('--hfov', type=float, default=1.1519)
    a = p.parse_args()
    g = SanalGimbal(hfov_rad=a.hfov, mount_phys_pitch_deg=a.mount,
                    aim_pitch_deg=a.aim)
    if a.test:
        raise SystemExit(0 if statik_test(g) else 1)
    print(g.ozet())


if __name__ == '__main__':
    main()
