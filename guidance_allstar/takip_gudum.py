#!/usr/bin/env python3
"""
takip_gudum.py - ARDUPILOT "FOLLOW" YASASI ile goruntulu gudum
==============================================================================
MPC'ye ALTERNATIF kol. Ayni iskelet (goruntulu_temel.GoruntuluDongu), ayni
sozlesme (ex/ey sanal gimbalden, hedeften YALNIZ menzil, komut = NED hizi),
ayni ISKA/VURUS durum makinesi -- DEGISEN TEK SEY GUDUM YASASI: optimizasyon
yerine ArduPilot'un kendi takip yasasi.

--------------------------------------------------------------------- KOKEN
Yasa, ArduPilot deposundaki iki dosyadan BIREBIR turetildi (yerel kopya
$ARDUPILOT_DIR, 2025-07-13):

  ArduCopter/mode_follow.cpp        FOLLOW modunun hiz yasasi
  libraries/AP_Follow/AP_Follow.cpp hedef kestirimi + ofset (FOLL_* param)
  libraries/AC_Avoidance/AC_Avoid.cpp::limit_velocity_2D / get_max_speed
  libraries/AP_Math/control.cpp::sqrt_controller

Copter-4.4 surumundeki ModeFollow::run() cekirdegi (bu dosyada "klasik" yasa):

    desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P
    |v_xy| <= WPNAV_SPEED ;  v_z in [-WPNAV_SPEED_DN, WPNAV_SPEED_UP]
    limit_velocity_2D(PSC_POSXY_P, WPNAV_ACCEL/2, ...)   # hedefe yaklasirken yavasla
    v_z    <= get_max_speed(PSC_POSZ_P, WPNAV_ACCEL_Z/2, |dz|)
    yaw    = hedefin kerterizi                            # FOLL_YAW_BEHAVE=0

Copter-4.5+ ayni isi pos_control->input_pos_vel_accel_NE ile yapar; oradaki
ic cevrim AC_P_2D::update_all = sqrt_controller(pos_hata, PSC_POSXY_P,
ivme_tavani, dt)'dir. Bu dosyada o kol "poscon" adiyla duruyor (--yasa poscon).

--------------------------------------------------- DORT ZORUNLU UYARLAMA
ArduPilot FOLLOW modu "hedefin yaninda dur"maya calisir; biz CARPMAYA
calisiyoruz ve hedef telemetrisinin cogunu kullanamiyoruz. Uc yerde ayrildik,
ucu de BILINCLI ve tek satirda geri alinabilir:

(1) HEDEF NEREDE?  AP_Follow hedefi MAVLink GLOBAL_POSITION_INT'ten alir.
    Bizde o YASAK (proje kurali: hedef telemetrisinden yalniz MENZIL). Yerine
    GORUNTU + MENZIL:
        eps = -(ey + aim)                 hedefin ufka gore yukselisi [deg]
        u_los = Rz(yaw) . los_ucayak(ex, eps)[0]
        hedef_pos = kendi_pos + menzil * u_los
    Kullanicinin sordugu "vision:get_estimated_target()" bindingi ArduPilot'ta
    YOK; companion tarafinda uretilen sey tam olarak bu uc satirdir.

(2) HIZ ILERI BESLEMESI YOK.  Klasik yasanin ilk terimi vel_of_target'tir ve
    hareketli hedefte isi ASIL O YAPAR. Bizde yasak -> geriye saf P kaliyor.
    Sonuc aritmetik: saf P'de kalici takip mesafesi d* = v_hedef / kp. Hedef
    21 m/s ve ArduPilot varsayilani FOLL_POS_P=0.1 iken d* = 210 m, yani
    angajman zarfinin (devir <=60 m) tamamen disinda -- vanilya ayar bu
    gorevde MATEMATIKSEL OLARAK yakalayamaz. Bu yuzden varsayilan kp=1.0
    (ArduCopter PSC_POSXY_P varsayilaniyla ayni sayi): 35 m hatada komut
    zaten hiz tavanina doyar, yani angajman boyunca "hedefe dogru tam gaz".
    kp ALTINDAKI tek gerekce budur; --kp ile degistirilebilir.

(3) YAKLASMA FRENI VARSAYILAN KAPALI.  limit_velocity_2D hedefe yaklasirken
    hizi sqrt(2*a*d) ile kirpar; 25 m'de (a=2.5 m/s^2) tavan 10.9 m/s olur --
    21 m/s'lik hedefe YETISEMEZ. Fren "yaninda dur"un ta kendisidir, bizim
    isimiz degil. --fren ap ile ACILIR (istasyon tutma / guvenli mesafe
    davranisi isteyen icin), --fren menzil ile yalniz uzakta acilir.

(4) HIZ BUYUKLUGU YASADAN DEGIL SEYIR TAVANINDAN.  (2)'nin kacinilmaz
    sonucu: |v| = kp*hata olan her saf P yasasi hareketli hedefe
    carpamaz -- denge mesafesi d* = v_hedef/kp'de menzil DONAR (kapali
    donguda olculdu: 22.75 m, teori 21.05). Cozum plane_follow.lua'nin
    kendi mimarisidir: orada da YON (GUIDED_CHANGE_HEADING) ile HIZ
    (GUIDED_CHANGE_SPEED) AYRI kanallardir. Bizde yon FOLLOW yasasindan,
    hiz seyir tavanindan gelir (hiz_kaynagi='tavan'). Saf mode_follow kolu
    --hiz-kaynagi p ile durur ve kiyas icin olculur.

Bunlarin disinda kod ArduPilot'un yaptigini yapar; ozellikle FOV/kadraj
maliyeti, PN terimi, hedef manevrasi kestirimi (BozucuKestirici) gibi MPC'ye
ozgu hicbir sey YOKTUR. Amac zaten "150 satirlik ArduPilot yasasi nereye
kadar yeter" sorusunu olcmek.

--------------------------------------- KADRAJI KIM KORUYOR (kritik fark)
Bu yasada hedefin kadrajda kalmasini saglayan HICBIR terim yoktur. Olculen
sonuc: devir aninda komut 17 -> 35 m/s'e BASAMAK yapiyor, ivme burnu 15.7 deg
asagi egiyor, govdeye sabit kamera onunla iniyor ve hedef UST kenardan
cikiyor (kapali donguda 18 kare kor). Tek savunma kinematik sekillendirmedir
(ivme_sekillendirme_mps2, varsayilan 3.0 -- olcum tablosu asagida) ve o da
ArduPilot'un kendi araci. ArduPilot'un ASIL cevabi ise gudumde degil
donanimdadir: FOLLOW modu girerken gimbali hedefe kilitler
(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER, mode_follow.cpp init()).

--------------------------------------------------------- NEDEN OFSET 0
FOLL_OFS_* hedefin YANINDA duracak noktayi verir; carpma senaryosunda dogal
degeri sifirdir ve varsayilan odur. Sifir ofsetin bedava yan faydasi: komut
DOGRUDAN hedefe baktigi icin avci hedefin irtifasina tirmanir, eps -> 0 gider
ve sabit 0 derece kamerada hedef kadrajin ustunden cikmaz. MPC'de bu ayri bir
mekanizmaydi (vurus_hiza_*, tur-3); burada yasanin kendisinden geliyor.
Standoff'lu takip isteyen --ofs-geri/--ofs-asagi verir; ofs, menzil
ofs_erime_menzil_m'e inerken dogrusal olarak sifira erir (yoksa fren kapaliyken
avci ofset noktasinin etrafinda salinir).

------------------------------------------------------------- KULLANIM
    cd guidance_allstar && python3 takip_gudum.py
    SURE=360 GORUNTULU="takip_gudum.py" PLAN=missions/hedef_sonsuz.plan \\
        tools/senaryo.sh                     # tam deneme (METOT=takip)
    python3 takip_test.py                    # cevrimdisi testler (sim gerekmez)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

_BURASI = str(Path(__file__).resolve().parent)
if _BURASI not in sys.path:                       # dogrudan calistirmada da
    sys.path.insert(0, _BURASI)                   # kardes modulleri bul

from goruntulu_temel import (GoruntuluKontrolcu, Komut, Olcum,   # noqa: E402
                             govde_ileri_ned)

KDEG = 180.0 / math.pi


# =========================================================== ORTAM KAPILARI
# mpc_gudum.py'deki ikizlerinin BIREBIR kopyasi. Kopyalandi (import edilmedi)
# cunku bu modul MPC'den BAGIMSIZ ucabilmeli: mpc_gudum'u import etmek
# cozucuyu ve 3000 satirlik ayar blogunu da yukler, ve iki kol birbirinin
# ayarini kazara paylasirsa A/B karsilastirmasi anlamsizlasir.

def cevre_mount_deg(varsayilan: float = 0.0) -> float:
    """Kamera ekseninin ufka gore yukselisi TEK KAYNAKTAN (standoff_geom.sh).

    GIMBAL DALI: fiziksel stabilize gimbal komutu $YILDIZ_TILT oncelikli;
    dondurulmus govdeye-sabit kosular icin $YILDIZ_MOUNT yedek."""
    for anahtar in ('YILDIZ_TILT', 'YILDIZ_MOUNT'):
        deger = os.environ.get(anahtar)
        if deger is not None:
            try:
                return float(deger)
            except (TypeError, ValueError):
                pass
    return float(varsayilan)


def cevre_hiz_tavani(varsayilan: float = 35.0) -> float:
    """Hiz tavani TEK KAYNAKTAN: guidance_config.GORUNTULU_MAX_SPEED_MPS.

    Iskelet (goruntulu_temel) komutu ZATEN bu sayiyla kelepceler; yasanin
    kendi tavani daha dusuk olursa kelepceye hic degilmez ve darbogaz
    sessizce gudumun kendi ayarina kayar (mpc_sonsuz_20260805_022808'de
    tam olarak bu oldu)."""
    try:
        import guidance_config as _cfg
        return float(getattr(_cfg, 'GORUNTULU_MAX_SPEED_MPS', varsayilan))
    except Exception:                       # cevrimdisi/kismi kurulum
        return float(varsayilan)


# ============================================================ ARDUPILOT MATH

def sqrt_controller(hata: float, p: float, ikinci_mertebe_tavan: float,
                    dt: float) -> float:
    """AP_Math/control.cpp::sqrt_controller BIREBIR portu.

    Setpoint yakininda P kontrolcu, uzakta sqrt(2*a*dx): yani "bu hizla
    gidersem ivme tavanimla tam noktada durabilir miyim" egrisi. ArduPilot'un
    tum konum kontrolculerinin (AC_P_2D, AC_P_1D, AC_Avoid) cekirdegi budur.
    """
    if ikinci_mertebe_tavan <= 0.0:
        duzeltme = hata * p
    elif p == 0.0:
        if hata > 0.0:
            duzeltme = math.sqrt(2.0 * ikinci_mertebe_tavan * hata)
        elif hata < 0.0:
            duzeltme = -math.sqrt(2.0 * ikinci_mertebe_tavan * (-hata))
        else:
            duzeltme = 0.0
    else:
        dogrusal_mesafe = ikinci_mertebe_tavan / (p * p)
        if hata > dogrusal_mesafe:
            duzeltme = math.sqrt(2.0 * ikinci_mertebe_tavan
                                 * (hata - dogrusal_mesafe / 2.0))
        elif hata < -dogrusal_mesafe:
            duzeltme = -math.sqrt(2.0 * ikinci_mertebe_tavan
                                  * (-hata - dogrusal_mesafe / 2.0))
        else:
            duzeltme = hata * p
    if dt > 0.0:                       # son adimda hatayi asma
        return float(np.clip(duzeltme, -abs(hata) / dt, abs(hata) / dt))
    return float(duzeltme)


def sqrt_controller_2d(hata_xy, p: float, ikinci_mertebe_tavan: float,
                       dt: float):
    """Vektor formu (AP_Math/control.cpp): yon korunur, buyukluk sekillenir."""
    hata_xy = np.asarray(hata_xy, dtype=float)
    boy = float(np.linalg.norm(hata_xy))
    if boy <= 0.0:
        return np.zeros(2)
    return hata_xy * (sqrt_controller(boy, p, ikinci_mertebe_tavan, dt) / boy)


def los_ucayak(ex_deg, eps_deg):
    """Burun (heading) cercevesinde LOS birim vektoru (mpc_gudum ile AYNI).

    Cerceve: x ileri (burun, YATAY), y sag, z asagi -- yalniz yaw uygulanmis
    NED. ex: hedefin buruna gore yatay kerterizi (+sag), eps: hedefin ufka
    gore yukselisi (+yukari). Yalnizca ilk ayak (LOS boyunca birim vektor)
    gerekiyor; MPC'nin e2/e3 ayaklari bu yasada kullanilmaz cunku komut
    LOS bileselerine ayrilmadan dogrudan NED'de kuruluyor."""
    ex = math.radians(ex_deg)
    eps = math.radians(eps_deg)
    ce, se = math.cos(eps), math.sin(eps)
    return np.array([ce * math.cos(ex), ce * math.sin(ex), -se])


# ==================================================================== AYARLAR

@dataclass
class TakipAyar:
    """ArduPilot parametre adlariyla eslesen ayarlar.

    Her alanin yaninda ArduPilot karsiligi ve VARSA sapmanin gerekcesi var.
    Sapma gerekcesi olmayan her sayi ArduPilot varsayilanidir."""

    # ---------------------------------------------------- AP_Follow (FOLL_*)
    kp: float = 1.0
    # FOLL_POS_P. AP varsayilani 0.1; burada 1.0 -- bkz. dosya basligi (2).
    # 1.0 ayni zamanda PSC_POSXY_P varsayilanidir, yani "poscon" kolunda
    # zaten kullanilan sayi; iki kolun tek knob'la kiyaslanabilmesi icin ayni.
    ofs_geri_m: float = 0.0             # FOLL_OFS_X isaret cevrilmis (bkz.
    ofs_asagi_m: float = 0.0            # dosya basligi "NEDEN OFSET 0")
    ofs_erime_menzil_m: float = 25.0
    # Ofsetin sifira erimeye BASLADIGI menzil degil, TAM eridigi menzil:
    # olcek = clip((r - erime) / (terminal - erime), 0, 1). Ofset sifirsa
    # tamamen etkisiz (carpma varsayilaninda hicbir sey yapmaz).
    yaw_p: float = 4.5                  # ATC_ANG_YAW_P (ArduCopter varsayilani)
    yaw_hiz_tavani_dps: float = 60.0    # ATC_SLEW_YAW 6000 cdeg/s
    yaw_komutu_ver: bool = True         # ablasyon: yaw'i otopilota birak

    # ----------------------------------------- pos_control limitleri (WPNAV_*)
    hiz_tavani_mps: float = field(default_factory=cevre_hiz_tavani)
    tirmanma_tavani_mps: float = 10.0   # WPNAV_SPEED_UP 1000 cm/s
    alcalma_tavani_mps: float = 5.0     # WPNAV_SPEED_DN 500 cm/s
    ivme_yatay_mps2: float = 5.0        # WPNAV_ACCEL 500 cm/s^2
    ivme_dikey_mps2: float = 5.0        # WPNAV_ACCEL_Z 500 cm/s^2
    psc_pos_p: float = 1.0              # PSC_POSXY_P
    psc_pos_z_p: float = 1.0            # PSC_POSZ_P

    # ------------------------------------------------------------------ yasa
    yasa: str = 'klasik'                # 'klasik' (Copter<=4.4) | 'poscon' (>=4.5)
    fren: str = 'kapali'                # 'kapali' | 'ap' | 'menzil'
    fren_menzil_m: float = 45.0         # 'menzil' kolunda frenin ACIK oldugu
                                        # menzil ustu (altinda kapanir)
    hiz_kaynagi: str = 'tavan'          # 'tavan' | 'p'
    # ------------------------------------------------------------------------
    # HIZ NEREDEN GELIYOR -- BU BRANCH'IN EN ONEMLI TEK KARARI.
    #
    # 'p'    : mode_follow.cpp'nin kendisi. Hiz BUYUKLUGU de yasadan cikar:
    #          |v| = kp * |hata|. OLCULEN SONUC (takip_test 4 ve 8): hedef
    #          hizi ileri beslemesi olmadan bu yasa HAREKETLI hedefe
    #          MATEMATIKSEL OLARAK carpamaz -- kalici denge mesafesi
    #          d* = v_hedef / kp (21.05 m/s hedef + kp=1.0 -> 21 m). Kapali
    #          donguda olculdu: min menzil 22.75 m, kapanma +0.8 m/s.
    #          Denge mesafesini kucultmenin tek yolu kp'yi buyutmek, o da
    #          kerteriz gurultusunu ayni oranda buyutur (v = kp*r).
    #
    # 'tavan': plane_follow.lua MIMARISININ koptere tasinmasi. Orada da yon
    #          ile hiz AYRI kanallardir: GUIDED_CHANGE_HEADING yonu verir,
    #          GUIDED_CHANGE_SPEED hizi verir (ve hiz hedefin airspeed'i
    #          etrafinda bir PID'den gelir, mesafeyle olceklenmez). Bizde
    #          yon FOLLOW yasasindan (LOS + ofset), hiz ise seyir tavanindan
    #          gelir. Hedef hizi hakkinda hicbir sey bilmeye gerek yoktur:
    #          "hedefe dogru, elimden geldigince hizli".
    #          VARSAYILAN BUDUR cunku gorev CARPMAK; 'p' kolu kiyas icin
    #          duruyor ve --hiz-kaynagi p ile acilir.
    ivme_sekillendirme_mps2: float = 3.0
    # Komut BUYUKLUGUNUN ARTIS hizi siniri [m/s^2]; 0 = kapali.
    # ArduPilot karsiligi: pos_control'un kinematik sekillendirmesi
    # (AP_Math/control.cpp::shape_vel_accel, WPNAV_ACCEL/WPNAV_JERK);
    # mode_follow >= 4.5 hedefi zaten oradan gecirir, yani bu satir
    # "Copter 4.4'un atladigi adimi geri koymak"tir. Yalniz ARTIS kirpilir;
    # azalis serbesttir (fren kanali ayri, bkz. fren).
    #
    # VARSAYILAN 3.0 -- OLCULDU (takip_test, 6 senaryo x 4 tohum, 21.05 m/s
    # hedef; min menzil ortancasi / kadraj kaybi):
    #     a=0.0   1.78 / 15.36 / 7.96 / 28.86 / 14.96 / 30.00   kayip %50-72
    #     a=2.0   5.37 / 18.65 / 5.48 / 18.02 /  8.60 / 23.42   kayip %0-8
    #     a=3.0   1.35 /  1.37 / 1.88 / 21.48 /  5.20 / 17.78   kayip %0-23
    #     a=5.0   1.68 /  2.83 / 2.50 / 37.11 /  1.74 / 39.81   kayip %7-41
    # Yani bu yasada sekillendirme kapanmanin DUSMANI degil, SARTIDIR.
    #
    # MPC'DE TERSI OLCULMUSTU (DEVAM.md tur-2: ileri_ivme_tavani 2 m/s^2 ->
    # min menzil 1.93 -> 11.17 m, "olculerek elendi"). CELISKI DEGIL:
    # MPC kadraji ZATEN maliyetinde koruyordu, orada sekillendirme yalniz
    # kapanma bedeli getiriyordu. Bu yasada kadraji koruyan hicbir terim
    # YOK; devir anindaki 17 -> 35 m/s BASAMAGI burnu 15.7 deg asagi egiyor,
    # sabit kamera onunla iniyor ve hedef UST kenardan cikiyor (kapali dongu
    # izinde 18 kare kor). Sekillendirmenin kazandirdigi kadraj, kaybettirdigi
    # kapanmadan buyuk.

    # ------------------------------------------------------- kamera geometrisi
    mount_pitch_deg: float = field(default_factory=cevre_mount_deg)
    aim_deg: float = 0.0                # senaryo.sh AIM=0 sabitliyor
    # eps = -(ey + aim): sanal gimbal ey'yi kamera EKSENINE gore uretir,
    # eksen de (mount + aim) kadar egiktir. mount 0 + aim 0'da eps = -ey.
    # mount_pitch_deg yalniz LOG ve kadraj pay hesabinda kullanilir: bu yasa
    # kadraj kisiti KURMAZ (ArduPilot'ta oyle bir sey yok).

    # ------------------------------------------------------- menzil suzgeci
    # mpc_gudum._menzil ile AYNI: olcum seyrek/gurultulu geldigi icin ic
    # durum modelle ilerletilir ve olcume kismi kazancla cekilir.
    menzil_olcum_kazanci: float = 0.35
    menzil_yoksa_m: float = 55.0
    menzil_taban_m: float = 6.0
    menzil_hizi_tau_s: float = 0.30
    alan_hizi_tau_s: float = 0.35
    bayat_kisit_s: float = 0.30

    # ------------------------------------------------- VURUS / ISKA (ORTAK)
    # BU BLOK MPC ILE BIREBIR AYNI SAYILARDIR. Bilincli: sonlandirma ve
    # temas tespiti GUDUM YASASI DEGILDIR; iki kol ayni olcutle olculmezse
    # karsilastirma kirlenir. Gerekceler mpc_gudum.MpcAyar'daki iska_*/
    # vurus_* bloklarinda; burada TEKRARLANMAZ, degistirilmez.
    vurus_modu: bool = True
    vurus_menzil_m: float = 22.0
    vurus_tam_menzil_m: float = 8.0
    terminal_menzil_m: float = 45.0
    vurus_basari_tespiti: bool = True
    vurus_basari_vibe: float = 15.0
    vurus_basari_menzil_m: float = 3.0
    iska_modu: bool = True
    iska_kaynak: str = 'menzil'         # 'menzil' | 'alan'
    # ------------------------------------------------------------------------
    # HAKEM NEREDEN BESLENIYOR -- kullanicinin "menzile guvenmiyorum" kuralinin
    # son adimi.
    #
    # OLCULEN DURUM (takip_test 11 ve 12):
    #   * KOMUT YOLU menzilden ZATEN tamamen bagimsiz. Varsayilan ayarda
    #     (ofs=0, hiz_kaynagi='tavan') cebirsel olarak sadelesir:
    #         v = kp*(r*u_los) -> tavana olceklenince -> V*u_los
    #     Kapali donguda 8 bozma kolu (x0.5, x2, +20 m, %30 gurultu, donmus,
    #     %50 kopuk, MENZIL HIC YOK) BIREBIR AYNI sonucu verdi.
    #   * Geriye kalan tek tuketici ISKA HAKEMI ve O BOZULUYOR: ayni sentetik
    #     profilde ates anini x0.5 -> 70.6 m, temiz -> 18.7 m, +20 m -> 40.7 m,
    #     x2.0 -> 25.7 m'ye kaydiriyor. Sebep: fark kurallari (r > en_iyi+30)
    #     MUTLAK kapilarla (12 / 45 / 120 m) ic ice.
    #
    # 'alan' KOLU bu bagimliligi da kaldirir. Fizik: hedef sabit boyutlu
    # oldugu icin s = sqrt(bbox alani) ~ C/r. C bilinmese bile ORAN bilinir:
    #         r / r_en_iyi  ==  s_tepe / s
    # Yani "menzil en iyinin 2 katina acildi" testi "gorunen boy tepenin
    # yarisina dustu"ye BIREBIR cevrilir ve C SADELESIR -- kalibrasyon
    # GEREKMEZ. Mutlak kapilar da gerekmez: "gectik mi" sorusu zaten
    # alan_hizi'nin isaret degistirmesiyle (buyuyordu, kuculuyor) yanitlanir.
    #
    # BEDELI (durust olmak lazim): bbox BAYATSA alan donar, hakem KOR kalir.
    # Menzil kestirimi ise gorusten bagimsiz akmaya devam ederdi. Kor fazda
    # geriye tek koruma ZAMAN ASIMI kalir. Bu yuzden varsayilan hala
    # 'menzil' (sim kiyaslari kirlenmesin); 'alan' --iska-kaynak alan ile.
    #
    # NEDEN YALNIZ ORAN, ASLA MUTLAK MENZIL (log madenciligiyle olculdu,
    # 14.816 kare / 12 kosu, 2026-08-05):
    #   * C = medyan(sqrt(alan)*gercek_menzil) UCAK grubunda 9 kosu boyunca
    #     685-730 arasinda, yani +-%3.3 -- kalibrasyon KOSULAR ARASI saglam.
    #   * AMA tek karede dagilim genis: r_gorsel = C/sqrt(alan) icin 0-20 m
    #     bandinda mutlak hata medyani 2.74 m, p90 21.4 m; ayni bantta
    #     menzil kestirimi 1.07 / 2.21 m. Kuyrukta 9.7x KOTU.
    #   * Model yalnizca ~9-50 m arasinda gecerli: min(w,h) <= 8 px altinda
    #     C 128-407'ye cokuyor, min(w,h) > 80 px (~7 m, yani VURUS ANI)
    #     ustunde de bozuluyor (0-5 m bandinda C 698 yerine 133.7).
    #   * Hata beyaz gurultu DEGIL epizodik (>%30 hata epizotlarinin p90
    #     suresi 0.65 s), o yuzden filtreleme kurtarmiyor.
    # SONUC: alan MUTLAK MENZIL KAYNAGI OLAMAZ. Ama hakemin sordugu soru
    # mutlak degil ORANSAL ("tepeye gore yariya dustu mu") ve o soru C'yi
    # sadelestirir. Yine de yukaridaki gecerlilik penceresi yuzunden
    # KALITE KAPISI sart (iska_alan_piksel_taban) ve karar DEBOUNCE'lidir.
    iska_acilma_orani: float = 2.0
    # 'alan' kolunda "menzil aciliyor": s < s_tepe / oran. 2.0 = "gorunen boy
    # yariya dustu" = "menzil iki katina cikti". Menzil kolundaki +30 m'lik
    # toplamsal esigin tipik en_iyi degerlerindeki (10-20 m) karsiligi 2.5-4x,
    # 45 m'deki karsiligi 1.67x idi -- yani eski kural menzile gore TUTARSIZ
    # sertlikteydi; oran kurali her olcekte ayni.
    iska_gecis_acilma_orani: float = 1.6    # gecis onaylandiktan sonra (dar)
    iska_alan_piksel_taban: float = 9.0
    # KALITE KAPISI [px]: min(bbox_w, bbox_h) bunun altindaysa alan olcumu
    # hakeme GIRMEZ. Sayi olculdu: 0-4 px'te bagil hata medyani %267,
    # 5-6 px'te %844, 7-8 px'te %86, 9-12 px'te %10, 13-20 px'te %6.
    # Esdeger kapsama esigi ~%1.0.
    iska_alan_tau_s: float = 0.35       # sqrt(alan) LPF'i (epizodik sicrama)
    iska_alan_onay_dongu: int = 6       # oran esigi ust uste kac dongu
    iska_alan_kucuk_dongu: int = 25
    # Kalite kapisinin ALTINDA ust uste bu kadar dongu kalinirsa "hedef cok
    # uzak/kucuk" ilan edilir. iska_mutlak_m'nin (120 m) gorusel karsiligi:
    # sim'de 402 m'de yapilan sahte devri yakalayan kol oydu, 'alan' kolunda
    # onun yerini bu tutar -- ve MENZIL OKUMADAN.
    iska_arm_m: float = 45.0
    iska_acilma_m: float = 30.0
    iska_gecis_arm_m: float = 12.0
    iska_gecis_acilma_m: float = 8.0
    iska_mutlak_m: float = 120.0
    iska_zaman_asimi_s: float = 8.0
    iska_zaman_kaynak: str = 'ilerleme'   # 'ilerleme' | 'duz'
    # ------------------------------------------------------------------------
    # ZAMAN ASIMI NEYI SAYIYOR -- olculen en pahali kusur burada.
    #
    # 'duz' (varsayilan, MPC ile ayni): devirden beri gecen DUVAR SAATI.
    # Kusuru cevrimdisi izde yakalandi (elips/capraz, tohum 3):
    #     t=0.0  r=45.0                 kapanma  -3.0 m/s
    #     t=6.6  r=31.4                 kapanma  +5.9
    #     t=7.8  r=23.2                 kapanma  +7.4
    #     t=8.0  ZAMAN ASIMI -> ISKA    kapanma  +7.7  (VE HIZLANIYOR)
    # Yani menzil 45 -> 19 m'ye inmis, kapanma her cegrekte artmis ve hakem
    # tam KAZANIRKEN dudugu calmis. Ayni desen asili hedef sim kosusunda da
    # var: 9 angajmanin 9'u zaman asimiyla kesildi, en iyi menziller
    # 20.8/19.2/14.3/13.4/8.8/5.6/4.6 m -- hepsi kapanirken.
    #
    # 'ilerleme': saat yalniz ILERLEME YOKKEN isler. "Ilerleme" tanimi
    # GORSELDIR ve olcek-bagimsizdir: hedefin gorunen alani BUYUYORSA
    # yaklasiyoruz demektir. Bagil buyume (dA/dt)/A kullanilir cunku
    # A ~ 1/r^2 oldugu icin (dA/dt)/A = 2*kapanma/r [1/s] -- yani menzilden
    # BAGIMSIZ bir "yaklasiyor muyum" olcusu. Mutlak px^2/s esigi olsaydi
    # uzakta hic tetiklenmez, yakinda hep tetiklenirdi.
    #
    # Sizinti yok: sayac SIFIRLANMAZ, geri SARILIR (sizintili integratör) --
    # gurultulu alan_hizi'nin isaret cirpmasi saati sonsuza kadar
    # durduramasin diye. Ustune mutlak tavan (iska_mutlak_sure_s) konur.
    iska_ilerleme_esigi_1s: float = 0.05
    # Bagil alan buyume esigi [1/s]. 0.05 = "2*kapanma/r > 0.05", yani 40 m'de
    # 1 m/s, 20 m'de 0.5 m/s kapanma. Gurultu bandinin ustunde, anlamli
    # kapanmanin altinda.
    iska_mutlak_sure_s: float = 25.0
    # Angajmanin MUTLAK tavani [s]: ilerleme surse bile bu asilirsa ISKA.
    # Sonsuz angajman olmasin diye emniyet supabi.
    iska_baslangic_koruma_s: float = 1.0
    gecis_menzil_hizi_esigi_mps: float = 3.0
    gecis_kapanma_esigi_mps: float = 10.0
    gecis_onay_dongu: int = 4
    gecis_alan_onay_dongu: int = 6
    iska_suzulme_hiz_mps: float = 12.0
    iska_suzulme_ivme_mps2: float = 3.0
    iska_redis_anahtar: str = ''

    def __post_init__(self):
        if self.yasa not in ('klasik', 'poscon'):
            raise ValueError(f"yasa 'klasik' | 'poscon' olmali: {self.yasa!r}")
        if self.fren not in ('kapali', 'ap', 'menzil'):
            raise ValueError(f"fren 'kapali'|'ap'|'menzil' olmali: {self.fren!r}")
        if self.hiz_kaynagi not in ('tavan', 'p'):
            raise ValueError(f"hiz_kaynagi 'tavan'|'p' olmali: {self.hiz_kaynagi!r}")
        if self.iska_kaynak not in ('menzil', 'alan'):
            raise ValueError(f"iska_kaynak 'menzil'|'alan' olmali: "
                             f"{self.iska_kaynak!r}")
        if self.iska_zaman_kaynak not in ('duz', 'ilerleme'):
            raise ValueError(f"iska_zaman_kaynak 'duz'|'ilerleme' olmali: "
                             f"{self.iska_zaman_kaynak!r}")


# ================================================================ KONTROLCU

class TakipKontrolcu(GoruntuluKontrolcu):
    """goruntulu_temel sozlesmesine uyan ArduPilot FOLLOW yasasi."""

    ad = "takip"

    def __init__(self, ayar: TakipAyar = None, tani_log=None):
        self.a = ayar or TakipAyar()
        self.tani_log_yolu = tani_log
        self._tani_f = None
        self._tani = None
        self._redis = None
        self.sifirla()

    # ------------------------------------------------------------- durum

    def sifirla(self):
        self.r_ic = None
        self.alan = None
        self.alan_hizi = 0.0
        self.sayac = 0
        self.v_ned_tohum = None
        self._son_v_ned = None
        self._son_hata_z = 0.0           # dikey fren icin (komut()'ta tazelenir)
        self.yaw_uygulanan = 0.0
        # --- durum makinesi (mpc_gudum ile ayni anlamda) ---
        self.durum = 'KAPANMA'           # KAPANMA | TERMINAL | VURUS | ISKA
        self.vurus_karisim = 0.0
        self.vuruldu = False
        self.vurus_vibe = 0.0
        self.vurus_menzil = 0.0
        self._bekleyen_olay = None
        self.en_iyi_menzil = float('inf')
        self.menzil_hizi = 0.0
        self.gecildi = False
        self.iska_sebep = ''
        self._r_onceki = None
        self._gecis_sayac = 0
        self._gecis_alan_sayac = 0
        self._kapanma_tepe = 0.0
        self._yetki_t0 = None
        # --- gorsel hakem durumu (iska_kaynak='alan') ---
        self.s_lpf = None                # sqrt(alan) LPF [px]
        self.s_tepe = 0.0                # gorulen en buyuk (en yakin) deger
        self._alan_buyudu = False        # bir kez bile buyudu mu (gecis icin)
        self._alan_acilma_sayac = 0
        self._alan_kucuk_sayac = 0
        self._ilerlemeyen_s = 0.0     # 'ilerleme' zaman asimi saati

    def tohumla(self, devir):
        """Devir aninda bir kez. ArduPilot FOLLOW modunun init()'i ile ayni
        rol: pos_control durumunu aracin O ANKI hizindan baslatmak. Bizde
        iskeletin LPF'si zaten devir hiziyla tohumlaniyor, burada yalniz
        kayit tutulur (tani logunda 'devir hizi' gorunsun)."""
        self.sifirla()
        if devir and 'cmd_vel_ned' in devir:
            self.v_ned_tohum = np.asarray(devir['cmd_vel_ned'], dtype=float)
            # ILK KOMUTUN DEMIRI: ivme sekillendirmesi ve ISKA suzulmesi
            # "onceki komut" ister. Tohumlanmazsa ilk dongu sekillendirmeyi
            # ATLAR -- ve devir anindaki 18 -> 35 m/s basamagi tam olarak o
            # ilk dongude olur, yani sekillendirme hicbir sey yapmazdi.
            self._son_v_ned = self.v_ned_tohum.copy()
        print(f"[takip] tohumlandi, devir hizi="
              f"{None if self.v_ned_tohum is None else np.round(self.v_ned_tohum, 2).tolist()}")

    # -------------------------------------------------------- yardimcilar

    def _menzil(self, olcum, kapanma_mps, dt):
        """Ic menzil durumu (mpc_gudum._menzil ile ayni yontem).

        SADECE Olcum.menzil_m kullanilir. kapanma_mps: LOS boyunca kendi
        hizimizin izdusumu (+ = menzil kisaliyor) -- model ilerletme terimi."""
        if self.r_ic is None:
            self.r_ic = (float(olcum.menzil_m) if olcum.menzil_m is not None
                         else self.a.menzil_yoksa_m)
            return self.r_ic
        self.r_ic += dt * (-kapanma_mps)
        if olcum.menzil_m is not None:
            self.r_ic += self.a.menzil_olcum_kazanci * (
                float(olcum.menzil_m) - self.r_ic)
        self.r_ic = float(max(self.a.menzil_taban_m * 0.5, self.r_ic))
        return self.r_ic

    def _alan_guncelle(self, olcum, dt):
        """bbox alani (px^2) ve buyume hizi -- gecis taniklarindan biri."""
        if olcum.bbox_w is None or olcum.bbox_h is None:
            return self.alan, self.alan_hizi
        A = float(olcum.bbox_w) * float(olcum.bbox_h)
        if self.alan is None:
            self.alan, self.alan_hizi = A, 0.0
            return self.alan, self.alan_hizi
        if 0.01 < dt < 0.35:
            ham = (A - self.alan) / dt
            k = dt / (dt + self.a.alan_hizi_tau_s)
            self.alan_hizi += k * (ham - self.alan_hizi)
        self.alan = A
        return self.alan, self.alan_hizi

    def _vurus_basarili_kontrol(self, olcum, r):
        """FIZIKSEL TEMAS: kendi vibrasyonumuz + OLCULEN menzil.

        mpc_gudum._vurus_basarili_kontrol ile birebir ayni esikler ve ayni
        latch mantigi -- iki kolun vurus sayisi ayni tanimla sayilmali."""
        a = self.a
        if (not a.vurus_basari_tespiti or self.vuruldu
                or olcum.vibe_max is None):
            return None
        r_olc = float(olcum.menzil_m) if olcum.menzil_m is not None else float(r)
        if (float(olcum.vibe_max) > a.vurus_basari_vibe
                and r_olc < a.vurus_basari_menzil_m):
            self.vuruldu = True
            self.vurus_vibe = float(olcum.vibe_max)
            self.vurus_menzil = r_olc
            detay = (f"vibe={self.vurus_vibe:.1f} (esik "
                     f"{a.vurus_basari_vibe:.0f}) menzil={r_olc:.2f} m "
                     f"durum={self.durum} vurus={self.vurus_karisim:.2f}")
            print(f"[takip] VURUS_BASARILI: {detay}")
            return ('vurus_basarili', detay)
        return None

    # ----------------------------------------------- ISKA durum makinesi

    def _durum_makinesi(self, olcum, r, alan_hizi, dt):
        """KAPANMA -> TERMINAL -> VURUS, ve ISKA sonlandirmasi.

        mpc_gudum._durum_makinesi'nin BIREBIR karsiligi (ayni tanikler, ayni
        esikler). Girdileri: menzil (izinli tek hedef olcusu), bbox alan hizi
        ve kendi saatimiz. Hedef hizi TURETILMEZ."""
        a = self.a
        if self._yetki_t0 is None:
            self._yetki_t0 = olcum.t
        toplam = float(olcum.t - self._yetki_t0)
        gecen = self._zaman_asimi_saati(olcum, alan_hizi, dt, toplam)

        if self._r_onceki is not None:
            ham = float(np.clip((r - self._r_onceki) / dt, -60.0, 60.0))
            k = dt / (dt + max(a.menzil_hizi_tau_s, 1e-6))
            self.menzil_hizi += k * (ham - self.menzil_hizi)
        self._r_onceki = float(r)
        if r < self.en_iyi_menzil:
            self.en_iyi_menzil = float(r)

        if self.durum != 'ISKA':
            if self.en_iyi_menzil <= a.terminal_menzil_m:
                self.durum = 'TERMINAL'
            if a.vurus_modu and self.en_iyi_menzil <= a.vurus_menzil_m:
                self.durum = 'VURUS'
        if self.durum == 'VURUS':
            genislik = max(a.vurus_menzil_m - a.vurus_tam_menzil_m, 1e-6)
            self.vurus_karisim = float(np.clip(
                (a.vurus_menzil_m - r) / genislik, 0.0, 1.0))
        else:
            self.vurus_karisim = 0.0

        if not a.iska_modu or self.durum == 'ISKA':
            return
        if a.iska_kaynak == 'alan':
            return self._hakem_alan(olcum, alan_hizi, dt, gecen)

        if r <= a.iska_gecis_arm_m:
            self._kapanma_tepe = min(self._kapanma_tepe, self.menzil_hizi)
        taze = olcum.bbox_yas_s <= a.bayat_kisit_s
        self._gecis_sayac = (self._gecis_sayac + 1
                             if self.menzil_hizi > a.gecis_menzil_hizi_esigi_mps
                             else 0)
        self._gecis_alan_sayac = (self._gecis_alan_sayac + 1
                                  if (taze and alan_hizi < 0.0) else 0)
        if (self.en_iyi_menzil <= a.iska_gecis_arm_m
                and self._kapanma_tepe <= -a.gecis_kapanma_esigi_mps
                and (self._gecis_sayac >= a.gecis_onay_dongu
                     or self._gecis_alan_sayac >= a.gecis_alan_onay_dongu)):
            self.gecildi = True

        if gecen < a.iska_baslangic_koruma_s:
            return
        sebep = ''
        if self.gecildi and r > self.en_iyi_menzil + a.iska_gecis_acilma_m:
            sebep = (f"gecis onayli, menzil aciliyor ({r:.0f} m > en iyi "
                     f"{self.en_iyi_menzil:.0f} + {a.iska_gecis_acilma_m:.0f})")
        elif (self.en_iyi_menzil <= a.iska_arm_m
                and r > self.en_iyi_menzil + a.iska_acilma_m):
            sebep = (f"menzil aciliyor ({r:.0f} m > en iyi "
                     f"{self.en_iyi_menzil:.0f} + {a.iska_acilma_m:.0f})")
        elif r > a.iska_mutlak_m:
            sebep = f"mutlak menzil ({r:.0f} m > {a.iska_mutlak_m:.0f})"
        elif gecen > a.iska_zaman_asimi_s:
            sebep = (f"zaman asimi ({gecen:.1f} s > {a.iska_zaman_asimi_s:.0f}), "
                     f"en iyi menzil {self.en_iyi_menzil:.1f} m")
        if sebep:
            self.durum = 'ISKA'
            self.iska_sebep = sebep
            print(f"[takip] ISKA: {sebep} -> yetki birakiliyor")
            self._iska_yayinla()

    def _zaman_asimi_saati(self, olcum, alan_hizi, dt, toplam):
        """Zaman asimi saatini dondurur (bkz. TakipAyar.iska_zaman_kaynak).

        'duz'      : duvar saati -- devirden beri gecen sure.
        'ilerleme' : YALNIZ ilerleme yokken isleyen sizintili saat.
                     Ilerleme olcusu GORSELDIR: bagil alan buyumesi
                     (dA/dt)/A > esik. A ~ 1/r^2 oldugu icin bu buyukluk
                     2*kapanma/r'ye esittir, yani menzil OKUNMADAN
                     "yaklasiyor muyum" sorusunu yanitlar.
        Her iki kolda da mutlak tavan asilirsa saat tavana kilitlenir."""
        a = self.a
        if a.iska_zaman_kaynak == 'duz':
            return toplam
        taze = olcum.bbox_yas_s <= a.bayat_kisit_s
        bagil = 0.0
        if taze and self.alan is not None and self.alan > 1e-9:
            bagil = float(alan_hizi) / float(self.alan)
        if bagil > a.iska_ilerleme_esigi_1s:
            # ILERLIYORUZ: saati geri sar (sifirlama YOK -- alan_hizi
            # isaret cirparsa saat sonsuza kadar durmasin).
            self._ilerlemeyen_s = max(0.0, self._ilerlemeyen_s - dt)
        else:
            self._ilerlemeyen_s += dt
        if toplam > a.iska_mutlak_sure_s:
            # EMNIYET SUPABI: ilerleme surse bile angajman sonsuz olamaz.
            return max(self._ilerlemeyen_s, a.iska_zaman_asimi_s + 1e-6)
        return self._ilerlemeyen_s

    def _hakem_alan(self, olcum, alan_hizi, dt, gecen):
        """GORSEL HAKEM: ISKA karari YALNIZ bbox'tan, menzil OKUNMADAN.

        Uc kural, ucu de OLCEK-BAGIMSIZ (C sadelesir):
          1. ACILMA : s = sqrt(alan) tepesinin 1/oran'ina dustu
                      (s_tepe/s == r/r_en_iyi oldugu icin bu birebir
                      "menzil en iyinin oran katina cikti" demektir)
          2. GECIS  : alan buyuyordu, simdi kuculuyor -> gectik
                      (menzil kolundaki 'kapanma tepesi' sartinin gorsel ikizi)
          3. COK KUCUK: kalite kapisinin altinda israrla kaliyoruz ->
                      hedef cok uzak (iska_mutlak_m'nin menzilsiz karsiligi)
        Zaman asimi kolu ortak; o zaten menzil okumuyor.

        KALITE KAPISI sart: olculen gecerlilik penceresi min(w,h) >= 9 px
        (altinda bagil hata %86-844). Kapinin altindaki kareler tepeyi de
        GUNCELLEMEZ -- yoksa tek bir sismis bbox ulasilamaz bir tepe koyar
        ve hakem hemen 'aciliyor' der."""
        a = self.a
        kenar = None
        if olcum.bbox_w is not None and olcum.bbox_h is not None:
            kenar = min(float(olcum.bbox_w), float(olcum.bbox_h))
        taze = olcum.bbox_yas_s <= a.bayat_kisit_s
        kaliteli = (taze and kenar is not None
                    and kenar >= a.iska_alan_piksel_taban
                    and self.alan is not None and self.alan > 0.0)

        if kaliteli:
            self._alan_kucuk_sayac = 0
            s = math.sqrt(float(self.alan))
            if self.s_lpf is None:
                self.s_lpf = s
            else:
                k = dt / (dt + max(a.iska_alan_tau_s, 1e-6))
                self.s_lpf += k * (s - self.s_lpf)
            self.s_tepe = max(self.s_tepe, self.s_lpf)
            if alan_hizi > 0.0:
                self._alan_buyudu = True
            # (2) GECIS: buyuduyduk, simdi kuculuyor
            self._gecis_alan_sayac = (self._gecis_alan_sayac + 1
                                      if (self._alan_buyudu and alan_hizi < 0.0)
                                      else 0)
            if self._gecis_alan_sayac >= a.gecis_alan_onay_dongu:
                self.gecildi = True
            # (1) ACILMA
            oran = (a.iska_gecis_acilma_orani if self.gecildi
                    else a.iska_acilma_orani)
            acildi = (self.s_tepe > 0.0 and self.s_lpf * oran < self.s_tepe)
            self._alan_acilma_sayac = (self._alan_acilma_sayac + 1
                                       if acildi else 0)
        elif taze or olcum.bbox_yas_s > a.bayat_kisit_s:
            # Kalite kapisinin altinda YA DA bbox bayat: tepe/oran dondurulur
            # (kor fazda hakem karar VERMEZ), yalniz 'cok kucuk' sayaci isler.
            if kenar is not None and kenar < a.iska_alan_piksel_taban:
                self._alan_kucuk_sayac += 1
            self._alan_acilma_sayac = 0

        if gecen < a.iska_baslangic_koruma_s:
            return
        sebep = ''
        if self._alan_acilma_sayac >= a.iska_alan_onay_dongu:
            oran = (a.iska_gecis_acilma_orani if self.gecildi
                    else a.iska_acilma_orani)
            sebep = (f"gorunen boy tepenin 1/{oran:.1f}'ine dustu "
                     f"(s {self.s_lpf:.1f} < tepe {self.s_tepe:.1f}/{oran:.1f})"
                     + (", gecis onayli" if self.gecildi else ""))
        elif self._alan_kucuk_sayac >= a.iska_alan_kucuk_dongu:
            sebep = (f"hedef cok kucuk ({kenar:.1f} px < "
                     f"{a.iska_alan_piksel_taban:.0f}) {self._alan_kucuk_sayac} dongu")
        elif gecen > a.iska_zaman_asimi_s:
            sebep = (f"zaman asimi ({gecen:.1f} s > {a.iska_zaman_asimi_s:.0f}), "
                     f"gorunen tepe {self.s_tepe:.1f} px")
        if sebep:
            self.durum = 'ISKA'
            self.iska_sebep = sebep
            print(f"[takip] ISKA (gorsel hakem): {sebep} -> yetki birakiliyor")
            self._iska_yayinla()

    def _iska_yayinla(self):
        """ISTEGE BAGLI Redis kopru yayini (mpc_gudum ile ayni sozlesme)."""
        anahtar = self.a.iska_redis_anahtar
        if not anahtar:
            return
        try:
            import json
            import redis
            if getattr(self, '_redis', None) is None:
                self._redis = redis.Redis(host='localhost', port=6379, db=0)
            self._redis.set(anahtar, json.dumps({
                't_mono': time.monotonic(), 'sebep': self.iska_sebep,
                'en_iyi_menzil': round(float(self.en_iyi_menzil), 2)}))
        except Exception as exc:
            print(f"[takip] ISKA redis yayini basarisiz: {exc}")

    # ------------------------------------------------------- FOLLOW yasasi

    def _hedef_kestir(self, olcum, r, ex, eps):
        """(1) AP_Follow'un yerine gecen GORU: hedefin NED konumu.

        AP_Follow bunu GLOBAL_POSITION_INT'ten alir ve hedefin HIZIYLA
        ekstrapole eder (get_target_pos_vel_accel_NED_m). Bizde iki fark:
          * konum goruntuden + menzilden kuruluyor,
          * EKSTRAPOLASYON YOK (hedef hizi yasak). Yani kestirim her zaman
            'simdi' anina aittir; bbox gecikmesi kadar geridedir.
        Doner: (hedef_pos_ned, u_los_ned). pos_ned yoksa hedef konumu
        goreli kalir (pos = 0 kabul) -- yasa zaten farki kullanir."""
        yaw = olcum.yaw_rad if olcum.yaw_rad is not None else 0.0
        l_h = los_ucayak(ex, eps)                    # heading cercevesi
        u_los = govde_ileri_ned(yaw, l_h[0], l_h[1], l_h[2])
        pos = (np.asarray(olcum.pos_ned, dtype=float)
               if olcum.pos_ned is not None else np.zeros(3))
        return pos + r * u_los, u_los

    def _ofset(self, u_los, r):
        """FOLL_OFS_* karsiligi, LOS cercevesinde.

        AP_Follow ofseti ya NED'de ya da hedefin BURNUNA gore (FRD) uygular;
        FRD hedefin heading'ini ister -> YASAK. Bizim cerceve LOS: 'geri'
        hedeften bize dogru yatay LOS boyunca, 'asagi' NED z. Boylece
        hedefin yonelimi hakkinda hicbir sey varsaymayiz.

        ERIME: ofset menzil ile dogrusal olarak sifira iner. Gerekce basit --
        fren kapaliyken sabit ofsetli bir hedef noktasi tutulamaz (P yasasi
        ofset noktasinin etrafinda salinir); carpma senaryosunda ofsetin
        gorevi zaten yalnizca UZAK fazda kadraji beslemektir."""
        a = self.a
        if a.ofs_geri_m == 0.0 and a.ofs_asagi_m == 0.0:
            return np.zeros(3), 1.0
        genislik = max(a.terminal_menzil_m - a.ofs_erime_menzil_m, 1e-6)
        olcek = float(np.clip((r - a.ofs_erime_menzil_m) / genislik, 0.0, 1.0))
        yatay = np.array([u_los[0], u_los[1], 0.0])
        n = float(np.linalg.norm(yatay))
        if n > 1e-6:
            yatay = yatay / n
        ofs = olcek * (-a.ofs_geri_m * yatay + np.array([0.0, 0.0, a.ofs_asagi_m]))
        return ofs, olcek

    def _hiz_yasasi(self, hata_ned, r_yatay, dt):
        """ModeFollow::run() hiz yasasi. hata_ned = (hedef+ofs) - kendi konum.

        SIRA ARDUPILOT'TAKI SIRADIR ve onemlidir: once P (ya da sqrt), sonra
        yatay olcekleme, sonra dikey kelepce, en son yaklasma freni. Fren en
        sonda oldugu icin yatay olceklemeden GECMIS bir vektoru kirpar --
        ArduPilot'ta da oyle (mode_follow.cpp: limit_velocity_2D cagrisi
        scale/constrain'den SONRA)."""
        a = self.a
        v = np.zeros(3)
        if a.yasa == 'klasik':
            # Copter <= 4.4: saf P (+ hedef hizi FF -- bizde yok).
            v = a.kp * np.asarray(hata_ned, dtype=float)
        else:
            # Copter >= 4.5: AC_P_2D / AC_P_1D = sqrt_controller.
            v[:2] = sqrt_controller_2d(hata_ned[:2], a.psc_pos_p,
                                       a.ivme_yatay_mps2, dt)
            v[2] = sqrt_controller(float(hata_ned[2]), a.psc_pos_z_p,
                                   a.ivme_dikey_mps2, dt)

        # --- HIZ KAYNAGI (bkz. TakipAyar.hiz_kaynagi) ---
        # Yasa yalniz YONU belirler, buyuklugu seyir tavani verir. YON
        # kaynagi bilincli olarak yine yasanin cikisidir (P/poscon vektoru),
        # hata vektorunun kendisi DEGIL: ofset ve kelepceler yonu de
        # sekillendiriyor, o sekillendirmeyi atlamak istemiyoruz.
        if a.hiz_kaynagi == 'tavan':
            n = float(np.linalg.norm(v))
            if n > 1e-6:
                v = v * (a.hiz_tavani_mps / n)

        # --- yatay hiz tavani (WPNAV_SPEED): YON KORUNUR, olceklenir ---
        yatay_hiz = float(math.hypot(v[0], v[1]))
        kelepce_yatay = yatay_hiz > a.hiz_tavani_mps
        if kelepce_yatay and yatay_hiz > 1e-9:
            v[0] *= a.hiz_tavani_mps / yatay_hiz
            v[1] *= a.hiz_tavani_mps / yatay_hiz

        # --- dikey tavanlar (WPNAV_SPEED_UP / _DN); NED'de z asagi + ---
        v[2] = float(np.clip(v[2], -a.tirmanma_tavani_mps, a.alcalma_tavani_mps))

        # --- YAKLASMA FRENI (AC_Avoid::limit_velocity_2D) ---
        fren_acik = (a.fren == 'ap'
                     or (a.fren == 'menzil' and r_yatay > a.fren_menzil_m))
        fren_tavani = float('inf')
        if fren_acik:
            # DIKKAT (ArduPilot'un kendi tuhafligi, bilerek korundu):
            # limit_direction hedefe dogru olan yon DEGIL, ISTENEN HIZIN
            # yonudur (mode_follow.cpp dir_to_target_xy'yi desired_velocity'den
            # kurar). Mesafe ise hedefe olan yatay mesafedir.
            yatay_v = np.array([v[0], v[1]])
            n = float(np.linalg.norm(yatay_v))
            if n > 1e-9:
                yon = yatay_v / n
                fren_tavani = sqrt_controller(r_yatay, a.psc_pos_p,
                                              0.5 * a.ivme_yatay_mps2, dt)
                hiz_yon = float(yatay_v @ yon)
                if hiz_yon > fren_tavani:
                    yatay_v = yatay_v + yon * (fren_tavani - hiz_yon)
                    v[0], v[1] = float(yatay_v[0]), float(yatay_v[1])
            # dikey fren: |v_z| <= get_max_speed(PSC_POSZ_P, accel_z/2, |dz|)
            dz = abs(float(self._son_hata_z))
            vz_tavan = sqrt_controller(dz, a.psc_pos_z_p,
                                       0.5 * a.ivme_dikey_mps2, dt)
            v[2] = float(np.clip(v[2], -vz_tavan, vz_tavan))

        # --- ISTEGE BAGLI kinematik sekillendirme (pos_control/shape_vel_accel)
        if a.ivme_sekillendirme_mps2 > 0.0 and self._son_v_ned is not None:
            n_yeni = float(np.linalg.norm(v))
            n_eski = float(np.linalg.norm(self._son_v_ned))
            tavan = n_eski + a.ivme_sekillendirme_mps2 * dt
            if n_yeni > tavan > 0.0:
                v = v * (tavan / n_yeni)      # YON korunur, ARTIS kirpilir
        return v, kelepce_yatay, fren_tavani

    def _yaw(self, ex):
        """FOLL_YAW_BEHAVE = 0 (FACE_LEAD_VEHICLE).

        ArduCopter yaw ACISI komutlar (auto_yaw.set_yaw_angle_rate) ve aci
        hatasini ATC_ANG_YAW_P ile hiza cevirir. Iskelet bize yalniz yaw HIZI
        kanali veriyor, o yuzden ayni cevrimi biz yapiyoruz:
            yaw_rate = ATC_ANG_YAW_P * (hedef kerterizi - burun)
        ve kerteriz hatasi ZATEN elimizde: kameranin ex'i. Iskelet ustune
        120 deg/s^2 slew + 0.15 s LPF uygular (ortak hijyen)."""
        a = self.a
        if not a.yaw_komutu_ver:
            return None
        return float(np.clip(a.yaw_p * ex,
                             -a.yaw_hiz_tavani_dps, a.yaw_hiz_tavani_dps))

    # -------------------------------------------------------- ISKA komutu

    def _iska_komut(self, olcum, ex, ey, eps, r, alan, alan_hizi, dt):
        """ISKA: FRENLI SUZULME + 'birak' bayragi (mpc_gudum ile ayni tasarim).

        Sifir komut ETMEYIZ: sifir "komut vermemek" degil TAM FREN'dir.
        Yonu koruyup buyuklugu rampa ile indiririz; donus yaricapi 35 m/s'de
        245 m, 12 m/s'de 29 m -- konumlunun yeniden konumlanmasi bunu ister."""
        a = self.a
        v = (np.asarray(olcum.vel_ned, dtype=float).copy()
             if olcum.vel_ned is not None
             else (self._son_v_ned.copy() if self._son_v_ned is not None
                   else np.zeros(3)))
        hiz = float(np.linalg.norm(v))
        if hiz > 1e-6:
            hedef_hiz = max(a.iska_suzulme_hiz_mps,
                            hiz - a.iska_suzulme_ivme_mps2 * dt)
            v = v * (min(hedef_hiz, hiz) / hiz)
        self._son_v_ned = v.copy()
        self.yaw_uygulanan = 0.0
        self._tani_yaz(olcum, ex, ey, eps, r, v, alan, alan_hizi,
                       {'kelepce_yatay': 0, 'fren_tavani': float('nan'),
                        'ofs_olcek': 0.0, 'hata': np.zeros(3),
                        'v_ham': np.zeros(3)})
        self.sayac += 1
        k = Komut(vel_ned=v, yaw_rate_dps=None)
        k.birak = True
        k.birak_sebep = self.iska_sebep
        return k

    # ------------------------------------------------------------- komut

    def komut(self, olcum: Olcum) -> Komut:
        a = self.a
        dt = float(np.clip(olcum.dt, 0.02, 0.30))
        ex = float(olcum.ex_deg)
        ey = float(olcum.ey_deg)
        eps = -(ey + a.aim_deg)              # hedefin ufka gore yukselisi

        # --- LOS ve menzil ---
        yaw = olcum.yaw_rad if olcum.yaw_rad is not None else 0.0
        l_h = los_ucayak(ex, eps)
        u_los = govde_ileri_ned(yaw, l_h[0], l_h[1], l_h[2])
        # menzil suzgecinin model terimi icin LOS boyunca kendi hizimiz
        # (+ = kapaniyor). OLCULEN hiz kullanilir, komut edilen degil.
        kapanma = (float(np.asarray(olcum.vel_ned, dtype=float) @ u_los)
                   if olcum.vel_ned is not None else 0.0)
        r = self._menzil(olcum, kapanma, dt)
        alan, alan_hizi = self._alan_guncelle(olcum, dt)

        # --- durum makinesi ve temas tespiti (komut yolundan ONCE) ---
        self._durum_makinesi(olcum, r, alan_hizi, dt)
        self._bekleyen_olay = self._vurus_basarili_kontrol(olcum, r)
        if self.durum == 'ISKA':
            return self._olay_tak(self._iska_komut(
                olcum, ex, ey, eps, r, alan, alan_hizi, dt))

        # --- (1) hedef kestirimi, (2) ofset, (3) hiz yasasi ---
        pos = (np.asarray(olcum.pos_ned, dtype=float)
               if olcum.pos_ned is not None else np.zeros(3))
        hedef_pos = pos + r * u_los
        ofs, ofs_olcek = self._ofset(u_los, r)
        hata = (hedef_pos + ofs) - pos          # = r*u_los + ofs
        self._son_hata_z = float(hata[2])       # dikey fren icin
        r_yatay = float(math.hypot(hata[0], hata[1]))
        v_ned, kelepce_yatay, fren_tavani = self._hiz_yasasi(hata, r_yatay, dt)
        yaw_rate = self._yaw(ex)
        self.yaw_uygulanan = 0.0 if yaw_rate is None else float(yaw_rate)

        self._tani_yaz(olcum, ex, ey, eps, r, v_ned, alan, alan_hizi,
                       {'kelepce_yatay': int(kelepce_yatay),
                        'fren_tavani': fren_tavani, 'ofs_olcek': ofs_olcek,
                        'hata': hata, 'v_ham': a.kp * hata})
        self.sayac += 1
        self._son_v_ned = v_ned.copy()
        k = Komut(vel_ned=v_ned, yaw_rate_dps=yaw_rate)
        k.birak = False
        k.birak_sebep = ''
        return self._olay_tak(k)

    def _olay_tak(self, k):
        olay = getattr(self, '_bekleyen_olay', None)
        if olay:
            k.olay, k.olay_detay = olay
            self._bekleyen_olay = None
        return k

    # ------------------------------------------------------------ tani log

    TANI_KOLONLARI = [
        't', 't_unix', 'dt', 'durum', 'vurus',
        'ex', 'ey', 'eps', 'menzil', 'menzil_olc', 'menzil_hizi', 'en_iyi',
        'hata_n', 'hata_e', 'hata_d', 'hata_yatay', 'ofs_olcek',
        'v_ham_n', 'v_ham_e', 'v_ham_d',
        'cmd_n', 'cmd_e', 'cmd_d', 'cmd_hiz', 'cmd_yatay',
        'kelepce_yatay', 'fren_tavani', 'yaw_cmd_dps',
        'alan', 'alan_hizi', 's_lpf', 's_tepe', 'bbox_yas', 'vibe', 'vuruldu',
        'pitch_deg', 'yaw_deg', 'irtifa_m',
    ]

    def _tani_yaz(self, olcum, ex, ey, eps, r, v_ned, alan, alan_hizi, ek):
        """takip_tani_*.csv: yasanin HER ADIMI okunabilir olsun.

        Kolon secimi bilincli olarak MPC'nin tani kolonlariyla ORTUSUR
        (durum, vurus, menzil, menzil_hizi, en_iyi, cmd_*, vibe, vuruldu):
        iki kolun kosulari ayni arac (tools/kosu_anlat.py, karsilastir.py)
        ile okunabilsin. Yasaya OZGU kolonlar: hata_*, ofs_olcek, v_ham_*
        (kelepcelenmemis P cikisi), fren_tavani."""
        if self.tani_log_yolu is None:
            return
        if self._tani_f is None:
            os.makedirs(os.path.dirname(self.tani_log_yolu), exist_ok=True)
            self._tani_f = open(self.tani_log_yolu, 'w', newline='')
            self._tani = csv.writer(self._tani_f)
            self._tani.writerow(self.TANI_KOLONLARI)
        hata = np.asarray(ek.get('hata', np.zeros(3)), dtype=float)
        v_ham = np.asarray(ek.get('v_ham', np.zeros(3)), dtype=float)
        ft = ek.get('fren_tavani', float('nan'))
        self._tani.writerow([
            f"{olcum.t:.4f}", f"{time.time():.3f}", f"{olcum.dt:.4f}",
            self.durum, f"{self.vurus_karisim:.3f}",
            f"{ex:.4f}", f"{ey:.4f}", f"{eps:.4f}", f"{r:.2f}",
            '' if olcum.menzil_m is None else f"{float(olcum.menzil_m):.2f}",
            f"{self.menzil_hizi:.2f}",
            ('' if not math.isfinite(self.en_iyi_menzil)
             else f"{self.en_iyi_menzil:.2f}"),
            f"{hata[0]:.2f}", f"{hata[1]:.2f}", f"{hata[2]:.2f}",
            f"{math.hypot(hata[0], hata[1]):.2f}",
            f"{float(ek.get('ofs_olcek', 0.0)):.3f}",
            f"{v_ham[0]:.2f}", f"{v_ham[1]:.2f}", f"{v_ham[2]:.2f}",
            f"{v_ned[0]:.3f}", f"{v_ned[1]:.3f}", f"{v_ned[2]:.3f}",
            f"{float(np.linalg.norm(v_ned)):.3f}",
            f"{math.hypot(v_ned[0], v_ned[1]):.3f}",
            int(ek.get('kelepce_yatay', 0)),
            '' if not math.isfinite(ft) else f"{ft:.2f}",
            f"{self.yaw_uygulanan:.2f}",
            '' if alan is None else f"{alan:.0f}", f"{alan_hizi:.1f}",
            '' if self.s_lpf is None else f"{self.s_lpf:.1f}",
            f"{self.s_tepe:.1f}",
            f"{olcum.bbox_yas_s:.3f}" if math.isfinite(olcum.bbox_yas_s) else '',
            '' if olcum.vibe_max is None else f"{olcum.vibe_max:.1f}",
            1 if self.vuruldu else 0,
            '' if olcum.pitch_rad is None else f"{math.degrees(olcum.pitch_rad):.2f}",
            '' if olcum.yaw_rad is None else f"{math.degrees(olcum.yaw_rad):.2f}",
            '' if olcum.pos_ned is None else f"{-float(olcum.pos_ned[2]):.2f}",
        ])
        if self.sayac % 20 == 0:
            self._tani_f.flush()


# =============================================================== main

def main():
    p = argparse.ArgumentParser(
        description="ArduPilot FOLLOW yasasi ile goruntulu gudum")
    p.add_argument('--sure', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--yasa', choices=('klasik', 'poscon'), default=None,
                   help="klasik = Copter<=4.4 saf P; poscon = Copter>=4.5 "
                        "sqrt_controller (varsayilan klasik)")
    p.add_argument('--kp', type=float, default=None,
                   help='FOLL_POS_P karsiligi (varsayilan 1.0; AP 0.1)')
    p.add_argument('--fren', choices=('kapali', 'ap', 'menzil'), default=None,
                   help='yaklasma freni (AC_Avoid::limit_velocity_2D); '
                        'varsayilan kapali -- carpma icin')
    p.add_argument('--hiz-kaynagi', choices=('tavan', 'p'), default=None,
                   help="tavan = plane_follow mimarisi (yon yasadan, hiz "
                        "seyir tavanindan; VARSAYILAN); p = saf mode_follow "
                        "(|v| = kp*hata -- hareketli hedefe carpamaz)")
    p.add_argument('--ivme-sekil', type=float, default=None,
                   help='komut buyuklugunun artis siniri [m/s^2]; 0=kapali')
    p.add_argument('--ofs-geri', type=float, default=None,
                   help='FOLL_OFS geri [m], LOS cercevesi (varsayilan 0)')
    p.add_argument('--ofs-asagi', type=float, default=None,
                   help='FOLL_OFS asagi [m] (varsayilan 0)')
    p.add_argument('--yaw-p', type=float, default=None,
                   help='ATC_ANG_YAW_P karsiligi (varsayilan 4.5)')
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw KOMUTLAMA (ablasyon: otopilotta kalir)')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--mount', type=float, default=None)
    p.add_argument('--hiz-tavani', type=float, default=None)
    p.add_argument('--no-vurus', action='store_true')
    p.add_argument('--no-iska', action='store_true')
    p.add_argument('--iska-kaynak', choices=('menzil', 'alan'), default=None,
                   help="ISKA hakemi neyi okusun: 'menzil' (varsayilan, MPC "
                        "ile ayni olcut) | 'alan' = bbox alani orani, MENZIL "
                        "HIC OKUNMAZ -- sistemin tamami menzilsiz calisir")
    p.add_argument('--iska-zaman-kaynak', choices=('duz', 'ilerleme'),
                   default=None,
                   help="zaman asimi neyi saysin: 'duz' (duvar saati, "
                        "varsayilan) | 'ilerleme' = saat yalniz hedefin "
                        "gorunen alani BUYUMEZKEN isler (menzil okunmaz)")
    p.add_argument('--iska-zaman-asimi', type=float, default=None,
                   help='ISKA zaman asimi [s] (varsayilan 8, MPC ile ayni). '
                        'ASILI HEDEFTE 15 GEREKIR: devir HAREKETSIZ araca '
                        'yapiliyor (tohum hizi 0) ve komut 3 m/s^2 ile '
                        'rampalandigi icin 20 m 8 s\'ye sigmiyor -- olculdu, '
                        'angajmanlar 5.6/4.6 m\'de kesildi. MPC\'nin asili '
                        'hedef rekoru (0.47 m) da 15 s esikle alinmisti.')
    p.add_argument('--iska-redis-anahtar', default=None)
    p.add_argument('--tani-log', default=None)
    args = p.parse_args()

    ayar = TakipAyar()
    if args.yasa is not None:
        ayar.yasa = args.yasa
    if args.kp is not None:
        ayar.kp = args.kp
    if args.fren is not None:
        ayar.fren = args.fren
    if args.hiz_kaynagi is not None:
        ayar.hiz_kaynagi = args.hiz_kaynagi
    if args.ivme_sekil is not None:
        ayar.ivme_sekillendirme_mps2 = args.ivme_sekil
    if args.ofs_geri is not None:
        ayar.ofs_geri_m = args.ofs_geri
    if args.ofs_asagi is not None:
        ayar.ofs_asagi_m = args.ofs_asagi
    if args.yaw_p is not None:
        ayar.yaw_p = args.yaw_p
    if args.no_yaw:
        ayar.yaw_komutu_ver = False
    if args.aim is not None:
        ayar.aim_deg = args.aim
    if args.mount is not None:
        ayar.mount_pitch_deg = args.mount
    if args.hiz_tavani is not None:
        ayar.hiz_tavani_mps = args.hiz_tavani
    if args.no_vurus:
        ayar.vurus_modu = False
    if args.no_iska:
        ayar.iska_modu = False
    if args.iska_kaynak is not None:
        ayar.iska_kaynak = args.iska_kaynak
    if args.iska_zaman_kaynak is not None:
        ayar.iska_zaman_kaynak = args.iska_zaman_kaynak
    if args.iska_zaman_asimi is not None:
        ayar.iska_zaman_asimi_s = args.iska_zaman_asimi
    if args.iska_redis_anahtar is not None:
        ayar.iska_redis_anahtar = args.iska_redis_anahtar
    ayar.__post_init__()                 # elle ezilen alanlari dogrula

    print(f"[takip] ArduPilot FOLLOW yasasi: yasa={ayar.yasa} kp={ayar.kp:.2f} "
          f"hiz_kaynagi={ayar.hiz_kaynagi} fren={ayar.fren} "
          f"ofs=(geri {ayar.ofs_geri_m:.1f} / asagi "
          f"{ayar.ofs_asagi_m:.1f} m, erime {ayar.ofs_erime_menzil_m:.0f} m)"
          + (f" ivme_sekil={ayar.ivme_sekillendirme_mps2:.1f} m/s^2"
             if ayar.ivme_sekillendirme_mps2 > 0 else ""))
    print(f"[takip] hiz tavani={ayar.hiz_tavani_mps:.1f} m/s "
          f"(guidance_config ile ayni kaynak), tirmanma/alcalma="
          f"{ayar.tirmanma_tavani_mps:.0f}/{ayar.alcalma_tavani_mps:.0f} m/s, "
          f"yaw_p={ayar.yaw_p:.1f} tavan={ayar.yaw_hiz_tavani_dps:.0f} dps")
    print(f"[takip] iska modu={'ACIK' if ayar.iska_modu else 'KAPALI'}, "
          f"vurus fazi={'ACIK' if ayar.vurus_modu else 'KAPALI'} "
          f"({ayar.vurus_menzil_m:.0f} m -> {ayar.vurus_tam_menzil_m:.0f} m), "
          f"montaj={ayar.mount_pitch_deg:+.1f} aim={ayar.aim_deg:+.1f}")

    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    tani = args.tani_log or str(Path(__file__).resolve().parent / 'logs'
                                / f"takip_tani_{damga}.csv")
    from goruntulu_temel import GoruntuluDongu
    GoruntuluDongu(TakipKontrolcu(ayar, tani_log=tani),
                   loop_hz=args.loop_hz).calistir(args.sure)


if __name__ == '__main__':
    main()
