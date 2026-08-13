#!/usr/bin/env python3
"""
pid_gudum.py - GORUNTULU gudum, PID yontemi (LOS / PID / MPC yarisinin PID kolu)
================================================================================

*** DONDURULMUS KOL (gimbal dali, 2026-08-05) ***
Bu kol GOVDEYE-SABIT KAMERA varsayimiyla yazildi ve FIZIKSEL GIMBAL dalinda
CALISMAZ: "kamera ekseni = montaj + govde pitch" varsayimi GECERSIZ. Kamera
artik kendini stabilize eden tek eksen (tilt) gimbalde; govde pitch'i
goruntuye yansimiyor (olculdu: govde +-35 deg iken kamera max 0.65 deg) ve
dikey eksen ARTIK KOMUT EDILEBILIR (YILDIZ_TILT = atan(down/back)).
Buradaki dikey kanal, FOV bantlari ve pitch temelli teshisler yeniden
turetilmeden kullanilmamalidir. Rehabilitasyon icin: NOTLAR_GIMBAL.md

goruntulu_temel.GoruntuluKontrolcu tabanina oturur; sozlesme orada tanimli.
Girdi sanal gimbalden gelen acisal hatalar (ex_deg, ey_deg), cikti NED
CIZGISEL HIZ + yaw_rate. Attitude komutlanmaz.

TEMEL FIKIR
-----------
Komut vektoru GOVDE ekseninde degil, OLCULEN LOS EKSENINDE kurulur:

    ileri ekseni  =  yaw + ex_deg  azimutu

Yani "ileri" her zaman hedefin OLCULEN yatay kerterizine bakar; yaw dongusunun
ne kadar geride kaldigi komut yonunu bozmaz. Gerekce: yaw_rate hem otopilotta
hem bizde kelepceli (+-60 deg/s) ve gecmiste sabit-dt varsayimiyla birlesince
titremenin/dev dairenin kaynagi olmustu. Yaw'i gudumden AYIRIP yalniz FOV
gorevine (kamerayi hedefte tut) birakinca gudum yaw performansindan bagimsizlasir.
--govde-ileri bayragi eski (burun eksenli) davranisa dondurur; makale icin
temiz bir A/B dugmesidir.

Uc kanal + bir FOV dongusu:

  1) YANAL  (LOS eksenine dik saga hiz)      <- ex_deg PID
  2) DIKEY  (NED asagi hiz)                  <- ey_deg PID
  3) ILERI  (LOS azimutu boyunca)            <- hiz butcesinden ARTAN + menzil/
                                                alan_kok temelli "commit" rampasi
  4) YAW    (yaw_rate_dps)                   <- ex_deg PD+I, YALNIZ kadraj icin

MENZIL ILE KAZANC ZAMANLAMASI (mirasin en degerli dersi)
--------------------------------------------------------
bumblebee/teva.py'de olculdu: dongu hatayi ACI olcup komutu METRE veriyorsa
sabit "m/derece" kazanci yanlistir, cunku 1 derecenin metre karsiligi menzille
buyur. Orada duzeltme  m_per_deg = R*tan(1 deg)  idi ve olculen sonuc:
ortalama dikey hata 1.01 -> 0.73 m, doygunluk %19.4 -> %7.4.

Bizde ayni fikir bir adim ileri tasindi: aci once menzille METREYE cevrilir,
sonra bir ZAMAN SABITIYLE m/s'ye:

    yanal_ofset_m = R * tan(ex)          # hedefin LOS ekseninden yanal sapmasi
    v_yanal_P     = yanal_ofset_m / tau_yanal
    v_yanal_D     = k_pn * R * lambda_dot # (rad/s) -> m/s, PN'in hiz bicimi

D terimi kasten "PN benzeri"dir: R*lambda_dot hedefin LOS'a dik gorunur
hizidir; onu kadar yanal hiz vermek LOS acisini DONDURMEMEK demektir, yani
carpisma rotasi. k_pn ~ 1-2 secilir (N'e karsilik gelir). Kose kesmeyi
(elips/wanderer rotalarinda yakalamayi) saglayan terim BUDUR; saf takip (P)
tek basina hep kuyruk kovalamacasi verir ve 18 m/s ile 20 m/s'lik hedefe
yetismez.

lambda_dot ATALET LOS hizidir, ex'in ham turevi DEGILDIR (ex burna goredir ve
yaw dongusu onu sifirlar). Turetimi ve gecikme eslemesi kodda; kuru kosuda
olculen fark buyuk (80 s elips, son menzil 317 m -> 36 m).

HIZ BUTCESI (ileri ekseni neden "artan" olarak hesaplanir)
----------------------------------------------------------
Iskelet |v| > V_MAX olursa vektoru TEK PARCA olcekler; bu yon bozmaz ama
ileri gazi ongorulemez sekilde calar ve anti-windup'i tanimsizlastirir. Bu
yuzden butce burada ACIKCA paylastirilir:
    manevra = hypot(v_yanal, v_dikey)   -> once manevra_max ile kelepce
    ileri   = min(ileri_istegi, sqrt(V_MAX^2 - manevra^2))
Boylece |v| hicbir zaman tavani asmaz, iskeletin kelepcesi hic devreye girmez
ve merkezleme kanallari her zaman butcelerini alir.

"Cesur ileri gaz": ileri_istegi HER ZAMAN tavandir (V_MAX, guidance_config
GORUNTULU_MAX_SPEED_MPS). SIM OLCUMU (kosu 2): taban tavanin altinda
tutulursa hedef bizden hizli oldugu icin menzil kalici olarak aciliyor.
Yavaslatma YOKTUR -- istenen sey carpismadir; merkezlemenin payi zaten
manevra butcesinden dusuluyor.

OLCUM KALITESI (kucuk/gurultulu bbox ve tespit kesintileri)
-----------------------------------------------------------
Duz rotada olculdu: hedef kuyruk goruntusunde bbox ~8 px, tespit %2.7. Boyle
bir olcumde D (PN) terimi TEHLIKELIDIR, cunku gurultu kazanci menzille
BUYUR: v_yanal_D = k_pn * R * lambda_dot. 60 m'de 0.2 deg'lik bir merkez
titremesi ~4 m/s sahte yanal hiz uretebilir -- tam da hedefin en zor
gorundugu yerde. P terimi ise aynı gurultuden neredeyse etkilenmez
(R*tan(0.2 deg)/tau = 0.14 m/s), yani sorun tamamen TUREV kanalindadir.

Bu yuzden bir OLCUM KALITESI skaleri q in [0,1] uretilir:
    q_alan : alan_kok kucukse dusuk (6 px -> 0, 20 px -> 1)
    q_yas  : bbox_yas_s buyukse dusuk (0.15 s -> 1, 0.45 s -> taban)
    q = q_alan * q_yas
ve su uc yerde kullanilir:
  1. D terimleri q ile OLCEKLENIR (+ mutlak pn_max kelepcesi).
  2. Turev suzgecinin zaman sabiti (1 + carpan*(1-q)) ile BUYUTULUR --
     SNR dustukce daha cok ortalama. Lag bedeli kabul edilebilir: dusuk
     kalite zaten uzak menzil demek, orada terminal hassasiyet gerekmiyor.
  3. q esigin altindaysa entegratorler DONDURULUR (bayat/gurultulu olcum
     kalici yanlilik biriktirmesin).
P ve ileri kanal q'dan ETKILENMEZ: takip ve kapanma her kalitede surmeli,
yoksa hedefi zaten goremedigimiz anda bir de geride kaliriz. Yaw D'si de
etkilenmez (o damping'dir; zayiflatmak dongunun sonumunu bozar).

DIKEY KANAL: ey NEDIR VE SETPOINT NICIN MENZILLE KAYAR
------------------------------------------------------
tools/senaryo.sh AIM=0 sabitler. aim=0 iken sanal kadraj UFKA hizalidir ve
(zincir turetildi, yildizlar_gimbal.stabilize):

        ey_deg = -(hedefin ufka gore YUKSELIS acisi)

Yani ey < 0 = hedef BIZDEN YUKARIDA, ey = 0 = ES IRTIFA. Duman testi bunu
dogruladi: devir aninda ey tipik olarak -15..-30 deg (standoff geometrisi
back=25/down=13 -> yukselis 27.5 deg). ey'i sifira surmek "es irtifaya
tirman" demektir; carpisma icin GEREKLIDIR ama HEMEN yapilamaz:

  KAMERA KADRAJI FIZIKSEL SINIR. Kamera govdeye +30 deg YUKARI sabit monte
  (models/suru_drone_*/model.sdf). fx=fy=985.5, yari dikey kadraj
  atan(360/985.5) = 20.1 deg. Yani gorulebilir yukselis bandi
        [ (30 + govde_pitch) - 20 ,  (30 + govde_pitch) + 20 ].
  Duragan takipte pitch ~-2.5 -> band [7.5, 47.5] deg. Es irtifa (yukselis
  0) bu bandin DISINDADIR; ancak arac tam gazda burun asagi yattikca
  (pitch -15..-20) band [-5, 35]'e kayar ve es irtifa gorulur hale gelir.
  Yani "hemen es irtifaya tirman" komutu hedefi kadrajin ALT KENARINDAN
  disari atma riskidir.

Bu yuzden dikey SETPOINT MENZILLE ZAMANLANIR (--ey-hedef-uzak/-yakin):
    menzil >= 55 m  ->  ey_hedef = -18 deg   (hedefi yukarida tut, kadraj
                                              guvenli, yavas tirman)
    menzil <= 12 m  ->  ey_hedef =   0 deg   (es irtifa = carpisma)
    arasi           ->  dogrusal
12 m'de kalan 0 derecelik hedef, 12 m'de 0 m dikey kacirma demektir; 25 m'de
setpoint -5.4 deg olur ki bu 25 m'de 2.4 m yukari, hala kadrajin ortasinda.
Boylece arac uzaktan yavas, yakinda kararli sekilde hedefin irtifasina cikar.

TEHLIKE: menzil ACILIYORSA (hedef kaciyor) ve setpoint es irtifadaysa dikey
kanal "alcal" diyebilir. Mutlak irtifa tabani vardir (--min-irtifa,
varsayilan 15 m): altina inildiginde alcalma yasak + dikey entegrator
dondurulur + yumusak tirmanis komutlanir.

NOT (aim != 0 kullanilirsa): ey'in tanimi "aim'e gore" kayar ve ey_hedef
uzak degeri sifira cekilmelidir (aim zaten DC ofseti tasir). Kod bu konuda
tarafsizdir; degistirilecek tek sey iki setpoint sayisidir.

ANTI-WINDUP (dort katman)
-------------------------
  1. KAPI      : |hata| > int_kapi_deg ise entegre EDILMEZ (hedef kadraj
                 kenarinda / kismi tespit -- teva.py'nin aykiri deger reddi).
  2. DOYGUNLUK : ilgili kanal kelepcede ise entegrator tau_aw ile sonumlenir
                 (teva.py'de integral *= 0.9; burada dt-normalize edildi).
  3. SIZINTI   : her zaman yavas sizinti (tau_sizinti). Devir tohumunu da
                 zamanla yikar, yani tohum kalici bir yanlilik birakmaz.
  4. YENIDEN KILIT: iki komut() cagrisi arasinda bosluk varsa (bbox kaybi --
                 iskelet o dongulerde bizi CAGIRMAZ) entegratorler yariya
                 iner, turev suzgecleri sifirlanir; bosluk uzunsa tam sifir.

DEVIR TOHUMLAMASI (kalinti temelli, entegratordan AYRI)
-------------------------------------------------------
Klasik "integral tohumlama" burada YANLIS olurdu: devir aninda ex ~ 0 ama
ey ~ -20 deg'dir (yukaridaki bolum), yani dikey P terimi devir aninda ZATEN
buyuktur. Tohumu entegratore koymak onun UZERINE binerdi.

Bunun yerine ayri, kendi zaman sabitiyle sonen bir TOHUM terimi vardir:
  * tohumla(devir): cmd_vel_ned, cmd_yaw_rad ile govde eksenine cevrilir ve
    saklanir (ileri bileseni dogrudan ileri-hiz durumunu tohumlar).
  * ILK komut() cagrisinda, P+I+D hesaplandiktan SONRA
        tohum = devir_bileseni - (P + I + D)
    olarak kurulur. Yani ilk komut TAM OLARAK konumlunun son komutudur --
    hata ne olursa olsun sicrama yoktur.
  * Sonraki her adimda tohum tau_tohum (2 s) ile sifira soner; PID devrali.
Entegratorler sifirdan baslar, yani tohum kalici bir yanlilik birakmaz ve
anti-windup mantigi kirilmaz.

KULLANIM
--------
    python3 pid_gudum.py                    # varsayilan kazanclarla
    python3 pid_gudum.py --kp-yaw 2.0 --tau-yanal 2.0 --sure 300
    python3 pid_gudum.py --liste            # kazanclari yazdir ve cik
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import asdict, dataclass, field

import numpy as np

import guidance_config as cfg
from goruntulu_temel import (Komut, GoruntuluDongu, GoruntuluKontrolcu,
                             govde_ileri_ned)


# --------------------------------------------------------------------- ayarlar

@dataclass
class PidAyar:
    """Tum kazanc/sinirlar tek yerde. Her degerin yanindaki not GEREKCESIDIR."""

    # --- YANAL kanal (ex_deg -> LOS eksenine dik saga hiz) ---
    tau_yanal_s: float = 1.5
    # "Hedefin yanal ofsetini kac saniyede kapat". R*tan(ex) metre cinsinden
    # ofset; onu tau'ya bolmek m/s verir.
    # NICIN 1.5 s VE DAHA KISA DEGIL (kapali dongu analizi): menzil olcekleme
    # sayesinde kapali dongu MENZILDEN BAGIMSIZ birinci mertebedir --
    #     d(e)/dt = -v/R = -e/tau.
    # Yani tau dogrudan kapali dongu zaman sabitidir. Dongudeki OLU ZAMAN
    # butcesi: bbox/kamera ~0.15 s + iskelet komut LPF'si 0.35 s + otopilotun
    # hiz dongusu ~0.4 s = ~0.9 s. Birinci mertebe + T olu zaman icin kaba
    # kural tau >~ 2T; 1.5 s tam bu sinirdadir. Daha kisa tau gecmisteki
    # titreme/dev daire kok nedenini geri getirir. Kose kesme icin gereken
    # ONCULUK tau'yu kisaltmakla degil, D (PN) terimiyle saglanir.
    k_pn_yatay: float = 1.8
    # PN'in hiz bicimindeki seyrusefer sabiti. v_yanal = k_pn * R * lambda_dot
    # LOS acisini DONDURMEYEN hizdir; k_pn>1 kose kesmeye izin verir.
    # Klasik PN'de N=3-5 ivme icindir; hiz biciminde 1 zaten "acisal hizi
    # tamamen yut" demektir.
    # 1.8 SECIMI (cevrimdisi kuru kosu taramasi, 60-80 s elips 8 deg/s donus,
    # kotumser model: arac hiz gecikmesi 0.8 s + kamera 0.30 s):
    #   k_pn  1.2 -> son menzil 43 m, |ex| tepe 4.2 deg
    #   k_pn  1.8 -> son menzil 36 m, |ex| tepe 5.9 deg
    #   k_pn  2.5 -> son menzil 30 m, |ex| tepe 6.8 deg
    #   k_pn  3.5 -> son menzil 30 m, |ex| tepe 7.9 deg  (doyuyor, kazanc yok)
    # Hicbirinde iraksama/limit cevrimi yok. 1.8, kapanmanin buyuk kismini
    # alip kadraj payini (yatay yari kadraj 33 deg) genis birakir. SIM FAZININ
    # BIRINCI AYAR DUGMESI BUDUR: yakalanamiyorsa once 2.5'e cikar.
    ki_yanal: float = 0.06          # (m/s) / (deg*s)
    # Surekli yan ruzgar / hedefin surekli donusu gibi DC yanliliklari kapatir.
    # Kucuk: entegrator bu problemde asil olarak DEVIR TOHUMU tasiyicisidir.
    yanal_max: float = 10.0         # m/s
    # 18 m/s butcenin en fazla bu kadari saf yanal olabilir; kalani ileri kalir.

    # --- DIKEY kanal (ey_deg -> NED asagi hiz) ---
    tau_dikey_s: float = 1.6
    # Yataydan biraz yavas: kopterin dikey ivme yetkisi (WPNAV_ACCEL_Z 250
    # cm/s^2) yatay yetkisinden dusuk; ayrica dikey eksende hata KADRAJIN
    # ALT KENARINA dogru surulur (kamera +30 monteli), yani agresiflik
    # burada dogrudan tespit kaybina cevrilir.
    k_pn_dikey: float = 1.0
    ki_dikey: float = 0.05          # (m/s) / (deg*s)
    tirmanis_max: float = 8.0       # m/s (negatif asagi = tirmanis)
    alcalma_max: float = 4.5        # m/s (pozitif asagi = alcalma)
    # ASIMETRIK ve otopilot limitleriyle hizali: guidance_config
    # GUIDED_STARTUP_PARAM_ASSERTS -> WPNAV_SPEED_UP 1000 cm/s (10 m/s),
    # WPNAV_SPEED_DN 500 cm/s (5 m/s). Otopilotun servis edemeyecegi bir hiz
    # komutlamak dongunun o kanalini fiilen ACIK CEVRIM yapar; tavanin biraz
    # altinda kalmak bunu onler. Alcalmanin daha dar olmasi ayrica emniyet:
    # kacan hedefte dikey kanal asagi doygunlasma egilimindedir (bkz. irtifa
    # tabani).

    # --- MANEVRA / ILERI butcesi ---
    manevra_max: float = 12.0       # m/s, hypot(yanal, dikey) tavani
    # 18 m/s butcede 12 m/s manevraya izin verilirse ileri eksene en kotu
    # ihtimalle sqrt(18^2-12^2) = 13.4 m/s kalir. Yani en sert merkezleme
    # aninda bile kapanma kabiliyetinin ~%75'i korunur.
    v_ileri_taban: float = float(getattr(cfg, "GORUNTULU_MAX_SPEED_MPS", 18.0))
    # m/s -- TAVANLA AYNI KAYNAKTAN okunur (bilerek; ayrisamazlar).
    # SIM OLCUMU (kosu 2, pid_elips_20260803_170119): taban 16 m/s iken devir
    # sonrasi komut buyuklugu 16.1 m/s'de kaldi; hedef 20 m/s oldugu icin
    # menzil 12 saniyede 29 m -> 95 m acildi ve bbox 52 px -> 7 px'e dusup
    # yetki 'konumlu'ya geri dondu. Ders: BU PROBLEMDE HIZ BUTCESINI GONULLU
    # OLARAK KISMANIN HICBIR KARSILIGI YOK. Hedef bizden hizli; tavanin
    # altindaki her m/s kalici kapanma kaybidir. Merkezlemenin ihtiyaci olan
    # pay zaten manevra butcesinden (asagida) dusuluyor, ileri kanalin ayrica
    # frenlemesine gerek yok.
    # Taban == tavan oldugu icin asagidaki "commit" rampasi VARSAYILANDA
    # ETKISIZDIR; parametre, uzakta bilerek yavaslamak istenen bir deney
    # icin (or. kadraj kararliligi calismasi) korunmustur.
    v_ileri_tavan: float = float(getattr(cfg, "GORUNTULU_MAX_SPEED_MPS", 18.0))
    ileri_ivme_mps2: float = 6.0
    # Ileri hiz durumunun egim siniri. Devir tohumundan (konumlunun hizi) tam
    # gaza gecisi ~0.5 s'ye yayar; komut LPF'si (tau 0.35) ile birlikte
    # aracin ic konum denetleyicisine basamak girmez.

    # "Commit" rampasi: yaklastikca ileri istek taban -> tavan.
    menzil_uzak_m: float = 60.0     # bu menzilde s=0 (taban gaz)
    menzil_yakin_m: float = 20.0    # bu menzilde s=1 (tam gaz)
    # 60 m devir kapisiyla AYNI (bbox_to_redis karar verici: ~1.5 s kadraj +
    # kapsama >= %2 + estimator menzili <= 60 m). Yani devir aninda s=0'dan
    # baslariz; rampa dogal olarak devirle hizalanir.
    alan_uzak_px: float = 12.0      # alan_kok bu ise s=0 (menzil ~60 m)
    alan_yakin_px: float = 34.0     # alan_kok bu ise s=1 (menzil ~20 m)
    # SIM KALIBRASYONU (kosu 2, pid_elips_20260803_170119, 799 esli ornek):
    #   menzil bandi -> alan_kok medyan
    #     80-200 m :  6      50- 80 m :  9      35- 50 m : 18
    #     25- 35 m : 26      15- 25 m : 36
    # Bu esikleri menzil rampasiyla hizaladim: menzil_uzak=60 m -> alan~12,
    # menzil_yakin=20 m -> alan~34. Ilk provizyonel degerler (8/25) menzil
    # 40-50 m'de bile s_alan=1 veriyordu; olcum bunu 18'e cekti.
    # DIKKAT (hala gecerli): bbox GENISLIGI menzil vekili DEGILDIR -- gorunus
    # acisina bagli. Olculdu: virajda 204 m'de 31 px, standoff 26 m'de arkadan
    # medyan 34 px. alan_kok burada YALNIZ IKINCIL kanittir; birincil kaynak
    # estimator menzilidir (devir kapisi <= 60 m tazelik garantiler). Ayrica
    # taban==tavan oldugu icin commit rampasi varsayilanda ETKISIZ; bu esikler
    # yalniz tani/log ve v_ileri_taban elle dusuruldugundeki deneyler icin
    # anlamlidir.

    # --- YAW / FOV dongusu (ex_deg -> yaw_rate_dps) ---
    kp_yaw: float = 2.5             # (deg/s) / deg  -> 1/s
    # Kapali dongu zaman sabiti ~1/2.5 = 0.4 s. Yaw dongusunun plant'i saf
    # entegrator + ~0.3 s otopilot gecikmesidir; Kp*T = 0.75 < 1 oldugu icin
    # faz payi makul. Daha yuksek Kp gecmisteki yaw titremesini geri getirir.
    ki_yaw: float = 0.5             # (deg/s) / (deg*s)
    # Surekli donen hedefte (elips) P tek basina lambda_dot/Kp kadar KALICI
    # kerteriz hatasi birakir: 20 deg/s LOS hizinda 8 derece. I bunu birkac
    # saniyede kapatir; katkisi yaw_i_katki_max ile sinirlidir.
    kd_yaw: float = 0.12            # (deg/s) / (deg/s) -> boyutsuz
    yaw_rate_max: float = 60.0      # deg/s
    yaw_i_katki_max: float = 18.0   # deg/s, I teriminin tek basina tavani

    # --- ortak PID sertlestirmeleri ---
    olu_bant_deg: float = 0.25
    # teva.py olcumu: gurultu tabani 25 m altinda 0.019 deg, 50-100 m'de
    # 0.164 deg. Bizim calisma bandimiz 20-60 m; 0.25 deg gurultunun birkac
    # kati ama 30 m'de yalnizca 0.13 m -- carpisma hassasiyetini bozmaz.
    # P ve I'ya uygulanir, D'ye UYGULANMAZ (D'de olu bant faz kaybidir).
    turev_tau_s: float = 0.15
    # Turev EMA zaman sabiti. teva.py alpha=0.05 @10 Hz (~1.9 s) kullaniyordu;
    # orada Kd cok kucuktu. Bizde D terimi KOSE KESME terimidir, 1.9 s gecikme
    # onu ise yaramaz hale getirir. 0.15 s ~ 30 Hz bbox akisinda ~4-5 kare
    # ortalamasi: piksel gurultusunu yutar, LOS dinamigini yutmaz.
    int_kapi_deg: float = 10.0
    # |hata| bunun ustundeyken entegre etme. Kadraj kenari = muhtemelen kismi
    # tespit; ayrica bu bolgede P zaten doygun, I yalniz sisebilir. 10 deg
    # ozellikle DIKEY kanal icin secildi: devir aninda ey ~ -20 deg gelir ve
    # bu buyuk aciligin entegratore yedirilmemesi gerekir (menzille zamanlanan
    # setpoint hatayi ~-9 dereceye indirir, tam gaz tirmanista yeniden 10'u
    # asarsa entegrator yine kendiliginden kapanir).
    int_katki_max: float = 3.0      # m/s, yanal/dikey I teriminin tavani
    lambda_dot_max: float = 90.0    # deg/s, atalet LOS hizi kelepcesi

    # --- OLCUM KALITESI (kucuk/gurultulu bbox, tespit kesintisi) ---
    # Gerekce ve olcum modul docstring'inde ("OLCUM KALITESI" bolumu).
    alan_kalite_alt_px: float = 6.0    # alan_kok bu ve altinda q_alan = 0
    alan_kalite_ust_px: float = 20.0   # bu ve ustunde q_alan = 1
    # KALIBRASYON (kosu 2, 799 esli ornek): alan_kok 6 -> 80-200 m,
    # 9 -> 50-80 m, 18 -> 35-50 m, 26 -> 25-35 m, 36 -> 15-25 m.
    # 6 px = "hedef neredeyse nokta" (duz rotanin kuyruk goruntusu buraya
    # dusuyor). UST DEGER 20 SECIMI cevrimdisi gurultu taramasiyla yapildi
    # (sigma=0.10 deg merkez titremesi, alan_kok=14, tespit orani %80):
    #     ust=14 -> yanal komut jitteri 1.261, tepe 10.0 (yanal_max'a DOYDU)
    #     ust=20 -> jitter 0.477, tepe 7.11      <-- secilen
    #     ust=24 -> jitter 0.354, tepe 4.87
    # ve UCUNDE DE kose kesme AYNI kaldi (elips kuru kosusu son menzil 35.8 m
    # degismedi), yani gurultu bastirma bedava geldi. 20'de kalindi cunku
    # gercek kalibrasyonla terminal bandda (25-35 m, alan_kok~26) q=1 olur:
    # carpismanin olacagi yerde PN yetkisi TAM kalir. 35-50 m'de q=0.86,
    # 50-80 m'de q=0.21 -- gurultunun bastigi yerde dogal olarak kisilir.
    yas_kalite_ust_s: float = 0.15     # bbox_yas bu ve altinda q_yas = 1
    yas_kalite_alt_s: float = 0.45     # bu ve ustunde q_yas = taban
    yas_kalite_taban: float = 0.15     # tam sifir DEGIL: biraz onculuk kalsin
    # Iskelet bbox_bayat_s = 0.7 s'ye kadar bizi cagirir; 0.45 s onun altinda
    # secildi ki "cagriliyoruz ama olcum eski" bolgesi kademeli sonsun.
    kalite_turev_carpani: float = 2.0  # q=0'da tau -> tau*(1+2) = 3 kat
    kalite_int_esigi: float = 0.5      # q bunun altinda entegre etme
    pn_max_mps: float = 8.0            # D teriminin MUTLAK kelepcesi [m/s]
    # q olcekleme oransaldir; pn_max ise "ne olursa olsun" tavani. Manevra
    # butcesinin (12 m/s) ucte ikisi: tek basina D terimi butceyi yiyemesin.
    # DUSURULMEDI (bilerek): ayni taramada pn_max 8 -> 6 -> 5 kose kesmeyi
    # bozdu (elips son menzil 35.8 -> 42.7 -> 46.8 m) ama alan_ust=20 iken
    # gurultuye HIC katki vermedi. Gurultuyu q sonduruyor, kelepce degil.
    kamera_gecikme_s: float = 0.12
    # ex'in kamera boru hattindaki gecikmesi; yaw ayni kadar geciktirilerek
    # lambda = ex + yaw toplaminin fazi tutturulur. OLCULEBILIR: bbox_to_redis
    # 'tracker_bbox_stab' paketinde t_capture var; Olcum bunu tasimadigi icin
    # simdilik nominal deger. (Onerilecek iskelet degisikligi: Olcum.olcum_yasi)
    aw_tau_s: float = 0.5
    # Doygunlukta entegrator sonumleme zaman sabiti (dt-normalize; teva'nin
    # kare basina *0.9 kuralinin sabit-dt'den arindirilmis hali).
    sizinti_tau_s: float = 8.0
    # Her adimda uygulanan yavas sizinti (sizintili entegrator). Uzun
    # kosularda I'nin sessizce kalici olmasini onler.
    tohum_tau_s: float = 2.0
    # Devir tohumunun sonumleme zaman sabiti. 2 s: iskeletin komut LPF'si
    # (0.35 s) bittikten sonra PID'in tamamen devralmasi ~3 tau = 6 s surer;
    # terminal angajman 5-15 s oldugu icin bu gecisin "ilk ceyregi" demektir.

    # --- menzil zamanlamasi ---
    menzil_varsayilan_m: float = 40.0
    # Estimator menzili yoksa (bayat/ilk anlar) kullanilan nominal. Devir
    # kapisi menzili <= 60 m'de acilir ve standoff 25 m'dir; 40 m bandin
    # ortasidir.
    menzil_alt_m: float = 12.0      # kazanc zamanlamasi icin kelepce
    menzil_ust_m: float = 80.0
    # Kelepce SART: kazanclar R ile DOGRU orantili; bozuk/kacik bir menzil
    # (estimator sicramasi) dogrudan kazanc sicramasi olurdu.

    # --- emniyet ---
    min_irtifa_m: float = 15.0
    # Home'a gore irtifa tabani. Altinda alcalma yasak + dikey I dondurulur;
    # ihlal buyudukce yumusak tirmanis komutlanir. Gerekce modul docstring'inde
    # (menzil aciliyorken sabit LOS acisi = yere dogru inis).
    irtifa_geri_itme: float = 0.5   # (m/s) / m ihlal

    # --- DIKEY SETPOINT: KAMERA EKSENINE gore (pitch-farkindali) ---
    kamera_montaj_deg: float = field(
        default_factory=lambda: float(os.environ.get('YILDIZ_MOUNT', 0.0)))
    # $YILDIZ_MOUNT (scripts/standoff_geom.sh export eder; models/
    # suru_drone_*/model.sdf ile AYNI olmali), fallback 0.0.
    # 2026-08-04: sabit 30.0 idi; sim montaji 0 dereceye alininca
    # (pitch-servo gimbal karari) bu sabit dikey setpoint'i 30 derece
    # kaydiriyordu. UYARI: _ey_hedefi'ndeki [5,45] eksen kelepcesi
    # mount 0'da hala tabanina (5 deg) oturuyor -- PID kodu bu turda
    # DONDURULMUS durumda, bkz. MPC ajani raporu.
    kadraj_agirlik_uzak: float = 1.0    # uzakta hedefi TAM kamera ekseninde tut
    kadraj_agirlik_yakin: float = 0.55  # yakinda ekseni birak, dikey kacirmayi kis
    ey_hedef_uzak_m: float = 55.0
    ey_hedef_yakin_m: float = 12.0
    max_derinlik_m: float = 25.0        # hedefin bu kadar altina inmeyi asma
    # NICIN EKSENE GORE, UFKA GORE DEGIL (yaw-kilit kosusunda yakalandi):
    # Onceki surumde setpoint UFKA gore sabit bir aciydi (-18 deg). Kadrajda
    # kalmayi belirleyen sey ise hedefin KAMERA EKSENINE gore acisidir ve
    # eksen govde pitch'iyle kayar:
    #       eksen_yukselisi = montaj(+30) + govde_pitch
    # Nominal takipte pitch ~-2.5 -> eksen 27.5 idi ve -18 deg setpoint 9.5
    # derece paylaydi. Ama duz rotada konumlu standoff'u TUTAMIYOR (hedef ve
    # kopter ikisi de 20 m/s): 25 m yerine ortanca 41.8 m geride surukleniyor,
    # dikey ofset 13 m sabit kaldigi icin yukselis 27.5 -> 17.3 dereceye
    # yatiyor, pitch ise +6.8'e cikip ekseni +36.8'e tasiyor. Sonuc: hedef
    # eksenin 19.5 derece ALTINDA, +-20.1 derecelik dikey kadrajin tam
    # kenarinda. Eski setpoint (-12.5 @41.8 m) bu durumda TIRMANIS komutluyor;
    # tirmanmak yukselisi daha da dusurur, yani hedefi kadrajin ALT
    # KENARINDAN DISARI iter -- tam ters yon.
    # Yeni setpoint: ey_hedef = -(montaj + pitch) * agirlik. Ayni geometride
    # -31.7 deg cikar, yani ALCALMA komutlanir; alcalmak derinligi artirir,
    # yukselisi buyutur ve hedefi kadraj merkezine geri getirir.
    # AGIRLIK NICIN 1 -> 0.55 RAMPASI: dikey kacirma = R*tan(yukselis)
    # oldugu icin menzil kapandikca ayni aci ZATEN kucuk kacirma demektir
    # (R=5 m'de 27 derece yalniz 2.5 m). Yani es irtifaya zorlamak (eski
    # yakin-uc setpoint'i 0) gereksiz; onun yerine yakinda ekseni bir miktar
    # birakip kacirmayi kisiyoruz ama TESPITI KAYBETMIYORUZ -- kadrajda
    # kalmak bu problemde asil kisitti (duz rotada tespit %2.7 idi).

    yeniden_kilit_s: float = 0.6
    # komut() cagrilari arasindaki bosluk bunu asarsa yeniden kilit sayilir.
    tam_sifir_s: float = 2.0        # bu kadar bosluktan sonra tam sifirla

    # --- yapisal secenekler ---
    govde_ileri: bool = False
    # False (varsayilan): ileri ekseni yaw+ex (OLCULEN LOS azimutu).
    # True: eski davranis, ileri ekseni burun. Makale icin ablasyon dugmesi.


# ------------------------------------------------------------- yardimci parcalar

def _kelepce(x, alt, ust):
    return alt if x < alt else (ust if x > ust else x)


def _olu_bant(e, bant):
    """Olu bandi CIKARARAK uygular (siçrama yaratmaz): |e|<bant -> 0."""
    if e is None:
        return 0.0
    if abs(e) <= bant:
        return 0.0
    return e - math.copysign(bant, e)


class TurevSuzgeci:
    """Olculen dt ile calisan, ZOH tekrarlarina karsi korumali turev suzgeci.

    NEDEN OZEL: bbox akisi 15-30 Hz, dongu 20 Hz. Ayni bbox iki kez okundugunda
    ham turev SIFIR, ardindan gelen yeni karede ISE COK BUYUK cikar; bu tarak
    (comb) D terimini gurultuye cevirir. Cozum: yalnizca DEGER DEGISTIGINDE
    ve o degisimin GERCEK gecen suresiyle turev al.

    Ayrica ilk ornekte turev 0 doner (teva.py dersi: ilk karede 1127-2817
    deg/s'lik sahte turevler olculmustu).
    """

    def __init__(self, tau_s, max_bekleme_s=0.35, min_dt_s=0.02):
        self.tau = float(tau_s)
        self.max_bekleme = float(max_bekleme_s)
        self.min_dt = float(min_dt_s)
        self.sifirla()

    def sifirla(self):
        self._onceki_deger = None
        self._onceki_t = None
        self._suzulmus = 0.0

    def guncelle(self, deger, t, dt, tau=None):
        """deger: ham hata [deg], t: monotonic [s], dt: OLCULEN dongu adimi.

        tau: bu adim icin zaman sabiti ezmesi. Olcum kalitesi dustukce
        (kucuk bbox / bayat tespit) disaridan BUYUTULUR: sinyal-gurultu orani
        kotulestiginde daha cok ortalama almak gerekir. Lag bedeli kabul
        edilebilir, cunku dusuk kalite zaten uzak menzil demektir ve orada
        terminal hassasiyete ihtiyac yoktur."""
        if deger is None:
            return self._suzulmus
        if self._onceki_deger is None:
            self._onceki_deger, self._onceki_t = float(deger), float(t)
            return 0.0
        gecen = float(t) - self._onceki_t
        if deger == self._onceki_deger and gecen < self.max_bekleme:
            # Ayni bbox tekrar okundu: turevi TAZELEME, elde tut.
            return self._suzulmus
        ham = (float(deger) - self._onceki_deger) / max(gecen, self.min_dt)
        self._onceki_deger, self._onceki_t = float(deger), float(t)
        tau_e = self.tau if tau is None else float(tau)
        a = dt / (dt + tau_e) if tau_e > 1e-6 else 1.0
        self._suzulmus += a * (ham - self._suzulmus)
        return self._suzulmus


class Entegrator:
    """Kapili + doygunluk sonumlemeli + sizintili entegrator."""

    def __init__(self, ki, katki_max, kapi_deg, aw_tau_s, sizinti_tau_s):
        self.ki = float(ki)
        self.katki_max = float(katki_max)
        self.kapi = float(kapi_deg)
        self.aw_tau = float(aw_tau_s)
        self.sizinti_tau = float(sizinti_tau_s)
        self.deger = 0.0            # birikmis hata [deg*s]

    @property
    def _deger_max(self):
        return (self.katki_max / self.ki) if self.ki > 1e-9 else 0.0

    def sifirla(self):
        self.deger = 0.0

    def zayiflat(self, oran):
        self.deger *= float(oran)

    def adim(self, hata_deg, dt, doygun, dondur=False):
        """Entegratoru ilerlet ve KATKI [m/s] dondur."""
        # 3. katman: her zaman sizinti (devir tohumunu da yikar).
        self.deger *= max(0.0, 1.0 - dt / max(self.sizinti_tau, 1e-6))
        if doygun:
            # 2. katman: kanal kelepcede -> hizli sonumleme.
            self.deger *= max(0.0, 1.0 - dt / max(self.aw_tau, 1e-6))
        elif not dondur and hata_deg is not None and abs(hata_deg) <= self.kapi:
            # 1. katman: kapi. Kadraj kenarinda / dondurulmusken birikme yok.
            self.deger += hata_deg * dt
        self.deger = _kelepce(self.deger, -self._deger_max, self._deger_max)
        return self.ki * self.deger


# ------------------------------------------------------------------ kontrolcu

class PidKontrolcu(GoruntuluKontrolcu):
    """PID gorsel servo: ex/ey -> LOS cercevesinde hiz + yaw_rate."""

    ad = "pid"

    def __init__(self, ayar: PidAyar | None = None):
        self.a = ayar or PidAyar()
        a = self.a
        self.i_yanal = Entegrator(a.ki_yanal, a.int_katki_max, a.int_kapi_deg,
                                  a.aw_tau_s, a.sizinti_tau_s)
        self.i_dikey = Entegrator(a.ki_dikey, a.int_katki_max, a.int_kapi_deg,
                                  a.aw_tau_s, a.sizinti_tau_s)
        # Yaw entegratorunun "katkisi" deg/s cinsinden; ayni sinif kullanilir.
        self.i_yaw = Entegrator(a.ki_yaw, a.yaw_i_katki_max, a.int_kapi_deg,
                                a.aw_tau_s, a.sizinti_tau_s)
        self.d_ex = TurevSuzgeci(a.turev_tau_s)
        self.d_ey = TurevSuzgeci(a.turev_tau_s)
        self.d_lambda = TurevSuzgeci(a.turev_tau_s)  # ATALET LOS kerterizi
        self._yaw_acik = None                      # sarilmasi cozulmus yaw [deg]
        self._yaw_tampon = []                      # [(t, yaw_acik)] gecikme icin
        self.v_ileri = a.v_ileri_taban       # ileri hiz durumu (egim sinirli)
        self.tohum_sag = 0.0                 # devir tohumu (PID'den AYRI)
        self.tohum_asagi = 0.0
        self._tohum_istegi = None            # ilk komut()'ta kalintiya cevrilir
        self._son_t = None
        self._son_yaw = None
        self._son_menzil = a.menzil_varsayilan_m
        self._doygun_yanal = False
        self._doygun_dikey = False
        self._doygun_yaw = False
        self.tani = {}                       # son dongunun ic terimleri (log/test)

    # -- devir ------------------------------------------------------------

    def tohumla(self, devir):
        """Konumlunun son komutunu govde eksenine cevirip TOHUM olarak sakla.

        Tohum entegratore YAZILMAZ: devir aninda dikey hata zaten buyuktur
        (ey ~ -20 deg), yani P terimi sifir degildir ve tohumu entegratore
        koymak onun uzerine binerdi. Tohum ilk komut() cagrisinda
        "devir_komutu - (P+I+D)" KALINTISI olarak kurulur ve tau_tohum ile
        soner (gerekce modul docstring'inde)."""
        self.i_yanal.sifirla()
        self.i_dikey.sifirla()
        self.i_yaw.sifirla()
        self.d_ex.sifirla()
        self.d_ey.sifirla()
        self.d_lambda.sifirla()
        self._yaw_acik = None
        self._yaw_tampon.clear()
        self._son_t = None
        self.v_ileri = self.a.v_ileri_taban
        self.tohum_sag = 0.0
        self.tohum_asagi = 0.0
        self._tohum_istegi = None
        if not devir:
            return
        try:
            v = np.asarray(devir.get('cmd_vel_ned'), dtype=float).reshape(3)
        except Exception:
            v = None
        yaw = devir.get('cmd_yaw_rad')
        if v is not None and yaw is not None:
            c, s = math.cos(float(yaw)), math.sin(float(yaw))
            ileri = c * v[0] + s * v[1]
            sag = -s * v[0] + c * v[1]
            asagi = float(v[2])
            # Ileri: durum olarak tohumla (egim siniri buradan rampalar).
            self.v_ileri = _kelepce(ileri, 0.0, self.a.v_ileri_tavan)
            self._tohum_istegi = (sag, asagi)
            self._son_yaw = float(yaw)
        r = devir.get('range_m')
        try:
            if r is not None and math.isfinite(float(r)) and float(r) > 0.0:
                self._son_menzil = float(r)
        except (TypeError, ValueError):
            pass

    # -- ic hesap (test edilebilir) ---------------------------------------

    def _menzil_zamanlamasi(self, olcum):
        """Kazanc zamanlamasinda kullanilacak menzil [m] (kelepceli)."""
        m = olcum.menzil_m
        if m is not None and math.isfinite(m) and m > 0.0:
            self._son_menzil = float(m)
        return _kelepce(self._son_menzil, self.a.menzil_alt_m,
                        self.a.menzil_ust_m)

    def _commit(self, olcum):
        """Ileri gaz rampasi s in [0,1]: yaklastikca tam gaz.

        BIRINCIL kaynak estimator menzili (devir kapisi <= 60 m tazelik
        garantiler). alan_kok IKINCIL: bbox boyutu gorunus acisina bagli
        oldugu icin menzil vekili olarak KULLANILMAZ, yalnizca "cok yakin"
        kanitini menzil bayatladiginda da tasir."""
        a = self.a
        s_menzil = 0.0
        m = olcum.menzil_m
        if m is not None and math.isfinite(m) and m > 0.0:
            genislik = max(1e-6, a.menzil_uzak_m - a.menzil_yakin_m)
            s_menzil = _kelepce((a.menzil_uzak_m - m) / genislik, 0.0, 1.0)
        s_alan = 0.0
        ak = olcum.alan_kok
        if ak is not None and math.isfinite(ak) and ak > 0.0:
            genislik = max(1e-6, a.alan_yakin_px - a.alan_uzak_px)
            s_alan = _kelepce((ak - a.alan_uzak_px) / genislik, 0.0, 1.0)
        # max: iki bagimsiz kanit; biri yoksa (None) digeri tasir.
        return max(s_menzil, s_alan)

    def _kalite(self, olcum):
        """Olcum kalitesi q in [0,1] (kucuk bbox / bayat tespit -> dusuk).

        Yalniz TUREV (PN) kanalini ve entegrator kapisini etkiler; P ve ileri
        kanal her kalitede tam calisir. Gerekce modul docstring'inde."""
        a = self.a
        ak = olcum.alan_kok
        if ak is None or not math.isfinite(ak):
            q_alan = 1.0        # bu alan yalniz 'taze' cagrilarda dolu gelir
        else:
            genislik = max(1e-6, a.alan_kalite_ust_px - a.alan_kalite_alt_px)
            q_alan = _kelepce((ak - a.alan_kalite_alt_px) / genislik, 0.0, 1.0)
        yas = olcum.bbox_yas_s
        if yas is None or not math.isfinite(yas):
            q_yas = a.yas_kalite_taban
        else:
            genislik = max(1e-6, a.yas_kalite_alt_s - a.yas_kalite_ust_s)
            taze = _kelepce((a.yas_kalite_alt_s - yas) / genislik, 0.0, 1.0)
            q_yas = a.yas_kalite_taban + (1.0 - a.yas_kalite_taban) * taze
        return q_alan * q_yas

    def _ey_hedefi(self, r, pitch_deg):
        """Menzille zamanlanan DIKEY setpoint [deg], KAMERA EKSENINE gore.

        eksen_yukselisi = montaj + govde_pitch; setpoint bunun bir kesridir.
        Uzakta tam eksen (maksimum kadraj payi), yakinda kesir kucultulup
        dikey kacirma kisilir. Gerekce PidAyar'da."""
        a = self.a
        genislik = max(1e-6, a.ey_hedef_uzak_m - a.ey_hedef_yakin_m)
        u = _kelepce((r - a.ey_hedef_yakin_m) / genislik, 0.0, 1.0)
        agirlik = a.kadraj_agirlik_yakin + (a.kadraj_agirlik_uzak
                                            - a.kadraj_agirlik_yakin) * u
        # Eksen makul bir bantta tutulur: bozuk/asiri pitch setpoint'i
        # ucurmasin (govde pitch'i gecici olarak 40 dereceyi bulabilir).
        eksen = _kelepce(a.kamera_montaj_deg + pitch_deg, 5.0, 45.0)
        return -eksen * agirlik

    def govde_istegi(self, olcum):
        """LOS cercevesinde (ileri, sag, asagi) + yaw_rate uret.

        Hiz komutunun TAMAMI burada olusur; komut() yalnizca NED'e cevirir.
        Ayri durmasinin sebebi test edilebilirlik: birim testleri bu sozlukten
        isaret/yon dogrular, MAVLink veya Redis'e ihtiyac duymaz."""
        a = self.a
        dt = max(float(olcum.dt), 1e-3)

        # --- yeniden kilit (bbox kaybi): iskelet kayip dongulerde bizi
        # cagirmaz, bu yuzden bosluk BURADAN olculur (4. anti-windup katmani).
        if self._son_t is not None:
            bosluk = float(olcum.t) - self._son_t
            if bosluk > a.tam_sifir_s:
                self.i_yanal.sifirla()
                self.i_dikey.sifirla()
                self.i_yaw.sifirla()
                self.d_ex.sifirla()
                self.d_ey.sifirla()
                self.d_lambda.sifirla()
                self._yaw_acik = None
                self._yaw_tampon.clear()
                self.tohum_sag = self.tohum_asagi = 0.0
            elif bosluk > a.yeniden_kilit_s:
                self.i_yanal.zayiflat(0.5)
                self.i_dikey.zayiflat(0.5)
                self.i_yaw.zayiflat(0.5)
                self.d_ex.sifirla()
                self.d_ey.sifirla()
                self.d_lambda.sifirla()
                self._yaw_acik = None
                self._yaw_tampon.clear()
        self._son_t = float(olcum.t)

        ex = 0.0 if olcum.ex_deg is None else float(olcum.ex_deg)
        ey_ham = 0.0 if olcum.ey_deg is None else float(olcum.ey_deg)
        r = self._menzil_zamanlamasi(olcum)
        pitch_deg = (0.0 if olcum.pitch_rad is None
                     else math.degrees(float(olcum.pitch_rad)))
        ey_hedef = self._ey_hedefi(r, pitch_deg)
        ey = ey_ham - ey_hedef

        # --- OLCUM KALITESI: kucuk/bayat bbox -> turev kanalini kis ---
        q = self._kalite(olcum)
        # SNR dustukce daha cok ortalama (lag bedeli bilerek kabul edilir).
        turev_tau = a.turev_tau_s * (1.0 + a.kalite_turev_carpani * (1.0 - q))

        # --- turevler (olculen dt, ZOH korumali) ---
        ex_dot = self.d_ex.guncelle(olcum.ex_deg, olcum.t, dt, turev_tau)
        ey_dot = self.d_ey.guncelle(olcum.ey_deg, olcum.t, dt, turev_tau)

        # ATALET LOS ACISAL HIZI (kuru kosuda yakalanan HATA, 'elips' rota):
        #   ex = lambda - yaw    (lambda = ATALET kerterizi, ex = BURNA gore)
        #   => d(ex)/dt = lambda_dot - yaw_dot
        # Yaw dongusu ex'i sifirda tuttugu icin STABIL DONUSTE ex_dot ~ 0 olur
        # ve PN terimi OLUR. Kose kesmeyi saglayan buyukluk ATALET LOS hizidir.
        # Kuru kosu (80 s, elips, 8 deg/s donus) son menzil: ex_dot ile 317 m
        # yerine 36 m -- terimin canlanmasi tek basina belirleyici.
        #
        # GECIKME ESLEMESI: ex kamera boru hattindan ~0.1-0.2 s GECIKMELI
        # gelir, yaw ise ATTITUDE'dan tazedir. Ikisi ham haliyle toplanirsa
        # her yaw gecicisinde SAHTE bir lambda_dot dogar ve o sahte deger
        # (k_pn * R ile carpildigi icin) yanal kanalda buyuk gorunur -- yani
        # yaw dongusu ile yanal kanal arasinda yapay bir geri besleme kurulur.
        # Bu yuzden yaw AYNI KADAR GECIKTIRILIR ve lambda = ex + yaw_gecikmeli
        # TEK bir sinyal olarak turevlenir (tek suzgec, tutarli faz).
        # DIKEY kanalda duzeltme GEREKMEZ: ey sanal gimbalde ufka hizali
        # okundugu icin ey_dot ZATEN atalet LOS yukselis hizidir.
        yaw_gec = None
        if olcum.yaw_rad is not None:
            ham = math.degrees(float(olcum.yaw_rad))
            if self._yaw_acik is None:
                self._yaw_acik = ham
            else:
                self._yaw_acik += ((ham - self._yaw_acik + 180.0) % 360.0) - 180.0
            self._yaw_tampon.append((float(olcum.t), self._yaw_acik))
            hedef_t = float(olcum.t) - a.kamera_gecikme_s
            while len(self._yaw_tampon) > 1 and self._yaw_tampon[1][0] <= hedef_t:
                self._yaw_tampon.pop(0)
            if len(self._yaw_tampon) > 200:        # tampon sinirsiz buyumesin
                del self._yaw_tampon[:-200]
            yaw_gec = self._yaw_tampon[0][1]
        if yaw_gec is None:
            lambda_dot_ham = ex_dot                # yaw yok: elde olan bu
        else:
            lambda_dot_ham = self.d_lambda.guncelle(ex + yaw_gec, olcum.t,
                                                    dt, turev_tau)
        # KELEPCE: lambda_dot yanal komuta k_pn*R ile carpilarak girer, yani
        # bir ATTITUDE sicramasi / sarilma hatasi aniden dev bir yanal hiz
        # isteyebilir. Fiziksel ust sinir: kendi yaw_rate tavanimiz 60 deg/s,
        # hedefin yakin menzilde yaratabilecegi LOS hizi ~40/12 rad/s. 90 deg/s
        # bunlarin ustunde ama gurultu sicramalarinin cok altindadir.
        lambda_dot = _kelepce(lambda_dot_ham,
                              -a.lambda_dot_max, a.lambda_dot_max)  # deg/s

        # --- YANAL kanal ---
        ex_db = _olu_bant(ex, a.olu_bant_deg)
        # Aciyi menzille METREYE cevir (teva.py m_per_deg dersi), sonra zaman
        # sabitiyle m/s'ye. tan(): buyuk acilarda da dogru ofseti verir.
        yanal_p = r * math.tan(math.radians(ex_db)) / max(a.tau_yanal_s, 1e-3)
        # PN'in hiz bicimi: R * lambda_dot [m/s]. lambda_dot ATALET LOS hizidir
        # (ex_dot + yaw_dot) -- yukaridaki gerekce.
        yanal_d = _kelepce(q * a.k_pn_yatay * r * math.radians(lambda_dot),
                           -a.pn_max_mps, a.pn_max_mps)
        dusuk_kalite = q < a.kalite_int_esigi
        yanal_i = self.i_yanal.adim(ex_db, dt, self._doygun_yanal,
                                    dondur=dusuk_kalite)

        # --- DIKEY kanal ---
        ey_db = _olu_bant(ey, a.olu_bant_deg)
        # ALCALMA YETKISI (eski "ters yon kilidi"nin yerine gecti).
        # Eski kilit "hedef yukaridayken asla alcalma" diyordu; kuru kosudaki
        # kacak dalisi engelliyordu ama yaw-kilit kosusu gosterdi ki duz
        # rotada DOGRU hamle tam da ALCALMAKTIR (surukleme yukselisi
        # yatirinca hedef kadrajin alt kenarina kaciyor; alcalmak onu merkeze
        # geri getiriyor). Setpoint artik kamera eksenine bagli oldugu icin
        # alcalma kendiliginden SINIRLIDIR -- yukselis setpoint'e ulasinca
        # hata kapanir. Geriye yalnizca iki mutlak emniyet kaliyor:
        #   1. irtifa tabani (min_irtifa_m)
        #   2. hedefin altindaki DERINLIK tavani: derinlik = R*sin(-ey).
        #      Menzil surekli aciliyorsa sabit aciyi korumak derinligi
        #      R ile buyuturdu; bu tavan onu keser. (R ve kamera acisi
        #      disinda bir sey kullanmaz -- menzil kurali korunur.)
        irtifa = None
        if olcum.pos_ned is not None:
            irtifa = -float(olcum.pos_ned[2])       # NED z asagi pozitif
        taban_ihlali = (irtifa is not None and irtifa < a.min_irtifa_m)
        # HAM menzille (self._son_menzil), kazanc zamanlamasinin kelepceli
        # r'siyle DEGIL: r [12,80]'e kirpildigi icin uzak menzilde derinligi
        # OLDUGUNDAN KUCUK gosteriyordu ve tavan gec devreye giriyordu
        # (kuru kosuda olculdu: duz rotada 25 m yerine 35 m derinlige inildi).
        derinlik_m = self._son_menzil * math.sin(math.radians(max(0.0, -ey_ham)))
        derinlik_asildi = derinlik_m > a.max_derinlik_m
        # ey_db > 0 = ALCALMA istegi. Iki mutlak emniyetten biri devredeyse
        # istek sifirlanir (entegratore de girmez -> sismez).
        if ey_db > 0.0 and (taban_ihlali or derinlik_asildi):
            ey_db = 0.0
        dikey_p = r * math.tan(math.radians(ey_db)) / max(a.tau_dikey_s, 1e-3)
        dikey_d = _kelepce(q * a.k_pn_dikey * r * math.radians(ey_dot),
                           -a.pn_max_mps, a.pn_max_mps)
        # derinlik/taban ihlalinde entegratoru DONDURMAK YETMEZ: onceden
        # birikmis alcalma yetkisi uzun bir kuyruk birakiyor (olculdu: tavan
        # 25 m'de tetiklenmesine ragmen 35.9 m'ye inildi). Doygunluk yolundan
        # AKTIF sonumleme (aw_tau) ile bosaltilir.
        dikey_i = self.i_dikey.adim(
            ey_db, dt, self._doygun_dikey or derinlik_asildi or taban_ihlali,
            dondur=taban_ihlali or dusuk_kalite or derinlik_asildi)

        # --- DEVIR TOHUMU: PID'den ayri, kalinti temelli, kendi tau'suyla ---
        if self._tohum_istegi is not None:
            # Ilk komut: tohum = konumlunun son komutu - PID'in su anki cikisi
            # => ILK komut tam olarak konumlunun son komutu olur (sicramasiz).
            hedef_sag, hedef_asagi = self._tohum_istegi
            self._tohum_istegi = None
            self.tohum_sag = _kelepce(hedef_sag - (yanal_p + yanal_d + yanal_i),
                                      -a.manevra_max, a.manevra_max)
            self.tohum_asagi = _kelepce(hedef_asagi - (dikey_p + dikey_d + dikey_i),
                                        -a.manevra_max, a.manevra_max)
        else:
            son = max(0.0, 1.0 - dt / max(a.tohum_tau_s, 1e-6))
            self.tohum_sag *= son
            self.tohum_asagi *= son

        v_sag_ham = yanal_p + yanal_d + yanal_i + self.tohum_sag
        v_sag = _kelepce(v_sag_ham, -a.yanal_max, a.yanal_max)
        v_asagi_ham = dikey_p + dikey_d + dikey_i + self.tohum_asagi
        v_asagi = _kelepce(v_asagi_ham, -a.tirmanis_max, a.alcalma_max)
        if derinlik_asildi:
            # Derinlik tavani: giris kapisi tek basina yeterli degil, cikis da
            # kelepcelenir -- tohum/entegrator/D artigi alcaltmaya devam etmesin.
            v_asagi = min(v_asagi, 0.0)
        if taban_ihlali:
            # Yumusak geri itme: ihlal buyudukce zorunlu tirmanis.
            geri = -(a.min_irtifa_m - irtifa) * a.irtifa_geri_itme
            v_asagi = min(v_asagi, geri)

        # --- manevra butcesi ---
        manevra = math.hypot(v_sag, v_asagi)
        butce_doygun = manevra > a.manevra_max
        if butce_doygun and manevra > 1e-9:
            olcek = a.manevra_max / manevra
            v_sag *= olcek
            v_asagi *= olcek
            manevra = a.manevra_max
        # Bir sonraki adimin anti-windup kapisi (koşullu entegrasyon, 1 adim
        # gecikmeli -- standart yontem).
        self._doygun_yanal = butce_doygun or abs(v_sag_ham) > a.yanal_max
        self._doygun_dikey = (butce_doygun
                              or v_asagi_ham > a.alcalma_max
                              or v_asagi_ham < -a.tirmanis_max)

        # --- ILERI kanal: butcede ARTAN + commit rampasi ---
        s = self._commit(olcum)
        ileri_istek = a.v_ileri_taban + (a.v_ileri_tavan - a.v_ileri_taban) * s
        kalan = math.sqrt(max(0.0, a.v_ileri_tavan ** 2 - manevra ** 2))
        ileri_hedef = min(ileri_istek, kalan)
        # Egim siniri: devir tohumundan (veya onceki degerden) rampala.
        adim_max = a.ileri_ivme_mps2 * dt
        self.v_ileri += _kelepce(ileri_hedef - self.v_ileri, -adim_max, adim_max)
        self.v_ileri = _kelepce(self.v_ileri, 0.0, kalan)

        # --- YAW / FOV dongusu (gudumden BAGIMSIZ) ---
        yaw_p = a.kp_yaw * ex_db
        yaw_d = a.kd_yaw * ex_dot
        yaw_i = self.i_yaw.adim(ex_db, dt, self._doygun_yaw,
                                dondur=dusuk_kalite)
        yaw_ham = yaw_p + yaw_d + yaw_i
        yaw_rate = _kelepce(yaw_ham, -a.yaw_rate_max, a.yaw_rate_max)
        self._doygun_yaw = abs(yaw_ham) > a.yaw_rate_max

        self.tani = {
            'r': r, 's_commit': s, 'ex': ex, 'ey': ey,
            'ey_ham': ey_ham, 'ey_hedef': ey_hedef,
            'tohum_sag': self.tohum_sag, 'tohum_asagi': self.tohum_asagi,
            'ex_dot': ex_dot, 'ey_dot': ey_dot,
            'lambda_dot': lambda_dot, 'q': q, 'turev_tau': turev_tau,
            'yanal_p': yanal_p, 'yanal_d': yanal_d, 'yanal_i': yanal_i,
            'dikey_p': dikey_p, 'dikey_d': dikey_d, 'dikey_i': dikey_i,
            'yaw_p': yaw_p, 'yaw_d': yaw_d, 'yaw_i': yaw_i,
            'manevra': manevra, 'kalan': kalan, 'irtifa': irtifa,
            'taban_ihlali': taban_ihlali, 'derinlik_m': derinlik_m,
            'derinlik_asildi': derinlik_asildi, 'pitch_deg': pitch_deg,
        }
        return {'ileri': self.v_ileri, 'sag': v_sag, 'asagi': v_asagi,
                'yaw_rate_dps': yaw_rate, 'ex_deg': ex}

    # -- sozlesme ---------------------------------------------------------

    def komut(self, olcum):
        istek = self.govde_istegi(olcum)
        yaw = olcum.yaw_rad
        if yaw is None:
            yaw = self._son_yaw
        if yaw is None:
            # Attitude hic gelmediyse NED'e cevrilemez. Sifir komutla; iskelet
            # LPF'si son hizdan yumusakca sonumler, karar verici zaten kisa
            # surede 'konumlu'ya doner.
            return Komut(vel_ned=np.zeros(3), yaw_rate_dps=None)
        self._son_yaw = float(yaw)
        # Ileri ekseni: OLCULEN LOS azimutu (yaw + ex). --govde-ileri ile burun.
        eksen = float(yaw) if self.a.govde_ileri else \
            float(yaw) + math.radians(istek['ex_deg'])
        v = govde_ileri_ned(eksen, istek['ileri'], istek['sag'], istek['asagi'])
        return Komut(vel_ned=v, yaw_rate_dps=istek['yaw_rate_dps'])


# ----------------------------------------------------------------------- CLI

def ayar_argparse(p: argparse.ArgumentParser):
    """PidAyar alanlarini --kebab-case bayraklarina acar."""
    var = PidAyar()
    for ad, deger in asdict(var).items():
        bayrak = '--' + ad.replace('_', '-')
        if isinstance(deger, bool):
            p.add_argument(bayrak, action='store_true', default=deger)
        else:
            p.add_argument(bayrak, type=type(deger), default=deger)
    return p


def ayar_uret(args) -> PidAyar:
    alanlar = asdict(PidAyar())
    return PidAyar(**{ad: getattr(args, ad) for ad in alanlar})


def main():
    p = argparse.ArgumentParser(
        description="PID goruntulu gudum (goruntulu_temel iskeleti uzerinde)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--sure', type=float, default=None,
                   help='saniye sonra dur (None = SIGINT bekle)')
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--tau-lpf', type=float, default=0.35,
                   help='iskelet komut LPF zaman sabiti [s]')
    p.add_argument('--log', default=None, help='CSV log yolu')
    p.add_argument('--liste', action='store_true',
                   help='kazanclari yazdir ve cik (sim baslatmaz)')
    ayar_argparse(p)
    args = p.parse_args()

    ayar = ayar_uret(args)
    if args.liste:
        print("PID ayarlari:")
        for ad, deger in asdict(ayar).items():
            print(f"  {ad:24s} = {deger}")
        return

    GoruntuluDongu(PidKontrolcu(ayar), loop_hz=args.loop_hz,
                   tau_s=args.tau_lpf, log_yolu=args.log).calistir(args.sure)


if __name__ == '__main__':
    main()
