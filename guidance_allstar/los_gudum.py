#!/usr/bin/env python3
"""
los_gudum.py - LOS (Line-of-Sight) oranli gudum, GORUNTULU faz
==============================================================

*** DONDURULMUS KOL (gimbal dali, 2026-08-05) ***
Bu kol GOVDEYE-SABIT KAMERA varsayimiyla yazildi ve FIZIKSEL GIMBAL dalinda
CALISMAZ: "kamera ekseni = montaj + govde pitch" varsayimi GECERSIZ. Kamera
artik kendini stabilize eden tek eksen (tilt) gimbalde; govde pitch'i
goruntuye yansimiyor (olculdu: govde +-35 deg iken kamera max 0.65 deg) ve
dikey eksen ARTIK KOMUT EDILEBILIR (YILDIZ_TILT = atan(down/back)).
Buradaki dikey kanal, FOV bantlari ve pitch temelli teshisler yeniden
turetilmeden kullanilmamalidir. Rehabilitasyon icin: NOTLAR_GIMBAL.md

goruntulu_temel.GoruntuluKontrolcu sozlesmesini uygular. Girdi sanal
gimbalden gelen acisal hata (ex_deg, ey_deg) + estimator menzili; cikti NED
CIZGISEL HIZ (+ yaw_rate). ATTITUDE KOMUTLANMAZ.


0. KAYNAK KODLARLA ILISKI (yildizlar25/legacy1.py, 2LOSKF*.py)
--------------------------------------------------------------
Yasanin ISKELETI legacy1.py'den DEVSIRILDI; birebir port DEGIL:

  ALINAN (aynen korunan fikirler)
    * Komut yonu bir ACI DURUMU'dur (sigma_y, sigma_z) ve her adimda
      integre edilir; hiz komutu v_d * n_d(sigma) seklinde uretilir.
      legacy1: sigma_cmd += dt * (K * ang_diff(q, sigma) - Kd * qdot)
    * Hiz buyuklugu ARTIMLIDIR: v_d = |v_simdi| + k_a  (2LOSKF2, satir 305).
      "Su an gittiginden k_a kadar daha hizli git" -- aracin kendi ivme
      sinirlarina saygi duyan, kapali dongulu bir rampa.
    * k_a hizalanmaya bagli KOSINUS CANI ile kapanir (2LOSKF2
      calculate_ka_from_ey): hizali degilken hizlanma, once yonel.
    * yaw yasasi:  yaw_rate = kp * e_x + kd * de_x/dt , kp=0.71, kd=0.917
      (legacy1 satir 328 / 2LOSKF2 satir 310) -- kullanicinin elinde iyi
      calistigi bilinen TEK tune ikilisi, birim uyumlu sekilde tasindi.

  DEGISTIRILEN (ve NEDEN)
    (a) TUREV TERIMININ ISARETI. legacy1'de sigma_dot = K*(q - sigma)
        - Kd*qdot. Onculuk (lead) acisi d = sigma - q icin bu
            d_nokta = -K*d - (Kd + 1) * qdot
        verir; yani LOS saga donerken komut SOLA kayar (GERI onculuk).
        Statik hedefte qdot ~ 0 oldugu icin bu terim hic uyanmaz -- "2loskf
        statik hedefi hep vururdu" gozlemi tam olarak bunun sonucudur.
        Capraz giden hedefte ise yasa gecikmeli takibe dusuyor. Dogru
        oransal seyrusefer (PN) isareti ARTIDIR:
            d_nokta = -d/tau_onc + (N - 1) * qdot                     (A)
        (A) ile hiz vektorunun ataletsel donus hizi N * qdot olur; bu
        klasik PN'in hiz-komutlu aractaki tam karsiligidir (bkz. 2).
    (b) DE-ROTASYON. legacy1 ham pikselden Rbe (roll/pitch/yaw) ile LOS
        birim vektoru kuruyor; govde salindikca hata sinyali saliniyor.
        Bizde sanal gimbal (yildizlar_gimbal) roll/pitch'i zaten temizliyor,
        ex/ey ufka hizali cerceveden geliyor.
    (c) KENDI YAW ORANIMIZIN CIKARILMASI. ex GOVDEYE goredir; ataletsel LOS
        orani chi_nokta = ex_nokta + yaw_nokta'dir (bkz. 1). legacy1 LOS'u
        dogrudan Earth frame'de kurdugu icin bu terimi ortuk aliyordu;
        biz ex ile calistigimiz icin ACIKCA eklemek zorundayiz. Eksik
        kalirsa yaw kontrolcusu kendi donusunu "hedef kaciyor" sanip PN'e
        sahte oran besler (kendi kendini besleyen donme).
    (d) DERINLIK. legacy1/2LOSKF menzili bbox genisliginden aliyordu
        (c_ptz = REAL_TARGET_WIDTH * F_OC / w). BU ORTAMDA GECERSIZ: duman
        testi bbox genisliginin aspect'e bagli oldugunu gosterdi (virajda
        204 m'de 31 px, standoff 26 m'de 34 px). Menzil YALNIZ estimatordan
        alinir; ilerleme sinyali olarak alan_kok ile birlikte kullanilir.
    (e) SINIRLAR. MAX_YAW_RATE 360 deg/s -> 60 deg/s, ustune ivme (slew)
        sinirli. Gorev brifingi ve yildizlar_gudum.sh'teki titreme notlari.

  KULLANILMAYAN: kaynak kodlardaki Kalman/EKF importlari (kullanici notu:
  o gorev hic yapilmamis, olu import).

  old_los_codes/*.py GECERSIZ REFERANSTIR (kullanici, 2026-08-03); oradan
  hicbir sey alinmadi.


1. FRAMELER VE OLCUM ANLAMLARI
------------------------------
H cercevesi ("govde-ufuk"): x = govde burnunun YATAY izdusumu (yaw), y = sag,
z = asagi. goruntulu_temel.govde_ileri_ned(yaw, ileri, sag, asagi) bu
cerceveyi NED'e cevirir.

  ex_deg : LOS'un GOVDE burnuna gore yatay kerterizi (+ sagda).
           yildizlar_gimbal.aci_hatasi() bunu atan2(r_stab[1], r_stab[0]) ile
           uretir; r_stab yaw=0 ile de-rotasyonludur, yani ex H cercevesindeki
           azimuttur.
  ey_deg : SANAL kadraj merkezine gore dikey sapma (+ asagida). Sanal merkez
           ufka gore -aim yukseliste durur, dolayisiyla

               eps = -aim_etkin - ey_deg                              (1)

           eps = hedefin UFKA gore yukselisi [deg]. tools/senaryo.sh
           YILDIZ_AIM=0 sabitler (2026-08-03) -> aim_etkin = 0 ve
           eps = -ey; "ey -> 0" DOGRUDAN es-irtifa carpisma geometrisidir.

           IZDUSUM DUZELTMESI: ey bir PERSPEKTIF olcumudur, kadraj merkezinden
           yatayda uzaklastikca dikey aci SISER. Tam bagintii (los_test.py
           gercek gimballe dogruluyor):

               eps = atan( -tan(ey) * cos(ex) ) - aim_etkin              (1)

           Duzeltilmemis hali ex=20 deg, eps=12 deg'de %6 hata veriyordu
           (12.75 okunuyordu). aim=0 iken (1) TAMDIR; aim != 0 iken ex, aim
           donmus cercevedeki azimutun yerine kullanilir (ikinci mertebe
           yaklasim).

Ataletsel LOS acilari (legacy1'in q_z / q_y'sinin karsiligi):

    q_az = yaw + ex        (azimut, NED)          <-> legacy1 q_z
    q_el = eps             (yukselis, ufka gore)  <-> legacy1 -q_y

Turevleri (OLCULEN dt ile, suzgecli):

    q_az_nokta = ex_nokta + yaw_nokta                                 (2)
    q_el_nokta = d(eps)/dt                                            (3)

(3) DOGRUDAN eps'in turevidir (ey'in degil): boylece hem (1)'deki izdusum
duzeltmesi hem de aim rampasinin degisimi turevin icine kendiliginden girer.


2. KONTROL YASASI -- MENZIL OLCEKLI ONCU (LEAD) ACISI
-----------------------------------------------------
Komut yonu, LOS'a gore ONCU ACI durumlariyla tanimlanir (legacy1'in sigma
durumuyla ayni yapi):

    sigma_az = q_az + d_az        sigma_el = q_el + d_el

CARPISMA UCGENININ TAM ARTIGI. Avcinin LOS'a dik hiz bileseni V*sin(d)'dir.
Bagil enlemesine hiz ise dogrudan olculur:

    R * q_nokta  =  (hedefin LOS'a dik hizi)  -  V * sin(d)               (4)

Bu ozdeslik hedefin hizini KESTIRMEZ; yalnizca menzil (estimator, izinli) ve
acisal LOS oranindan (kamera) okunan bir ARTIKTIR. LOS oranini sifirlayacak
oncu acisi buradan CEBIRSEL olarak cikar:

    sin(d_gerek) = sin(d) + k_pn * R * q_nokta / V                        (5)
    d_gerek      = asin( kelepce(sin(d_gerek), +-sin(d_max)) )

k_pn = 1.0 "bir sonraki anda LOS oranini tam sifirla" demektir (dead-beat);
>1 fazla surer, <1 az.

ONCU DINAMIGI (birinci mertebe yaklasim + yikama):

    d_nokta = (d_gerek - d) / tau_yak  -  d / tau_onc                     (6)

tau_yak arac tepkisiyle (komut LPF 0.35 s + kopter dinamigi ~0.5 s) ayni
mertebede secilir; daha hizli secilirse kontrolcu, aracin henuz uygulamadigi
onculugu "yetersiz" sanip sismeye baslar (windup).

NEDEN BU BICIM, KLASIK d_nokta = (N-1)*q_nokta DEGIL: kucuk aci limitinde
ikisi ozdestir ve ETKIN navigasyon orani

    N_etkin = 1 + R / (V * tau_yak * cos d)                               (7)

cikar -- yani (6) MENZILE GORE ZAMANLANMIS bir PN'dir. Onemi sudur: R -> 0
iken q_nokta 1/R ile patlar; SABIT N'li klasik PN bu anda onculugu kelepceye
dayandirip araci LOS'a neredeyse DIK ucurur (olculdu: capraz senaryoda oncu
1 s icinde 60 dereceye doyuyor, menzil 21 m'de donup aciliyordu). (5)-(6)'da
R carpani ayni anda kuculdugu icin oncu talebi kendiliginde sonumlenir.
N_etkin tipik degerleri: R=60 m, V=18, tau_yak=0.8 -> 5.2;  R=25 m -> 2.7.

R YOKSA: menzil hic bilinmiyorsa (5) menzil_varsayilan_m ile kosulur ve
tani['menzil_kaynak'] = 'yok' olarak loglanir; yasa yine calisir ama olcegi
kabadir.

YIKAMA (-d/tau_onc) legacy1'in K*(q - sigma) teriminin ta kendisidir
(K = 1/tau_onc): LOS orani sifira inince yasayi saf takibe (pure pursuit)
geri dusurur ve gurultunun integratorde rastgele yuruyusle sismesini onler.
Bir SIZINTI oldugu icin capisma suresinden COK BUYUK secilir (varsayilan
20 s; terminal faz ~5-15 s).

(5)-(6) HER DONGUDE cozulur (yeni bbox ornegi gelmemis olsa da): q_nokta
zaten SURELI bir hiz kestirimidir, sifir-mertebe tutucu ile dongu adimi kadar
integre etmek dogrudur. Ornek gelmedigi surece dondurulmesi gereken tek sey
TUREV SUZGECIDIR (bolum 6).

ONCU KELEPCESI VE FOV: oncu aci HIZ VEKTORUNU dondurur, KAMERAYI DEGIL.
Kopterde yaw hiz vektorunden bagimsiz suruldugu icin buyuk oncu hedefi
kadrajdan CIKARMAZ. Kelepce (onc_az_max, varsayilan 85 deg) yalniz "kapanma
hizini tumden feda etme" sinirdir. GENIS OLMASI SART: hedef bizden HIZLI
(20 vs 18 m/s) oldugu icin carpisma ucgeni sin(d) = (V_t/V_p) sin(theta_t)
ile 80 dereceye varan oncu isteyebilir; los_test kapali dongu taramasinda
(daire 6/9/14 deg/s) 60 deg kelepce 22.4/15.1/9.8 m KACIRMA, 85 deg kelepce
1.3/1.5/1.2 m ISABET verdi. Doygunlukta park etme riskini yikama (tau_onc)
ve terminal dondurma keser.

Komut yonu ve hiz (legacy1 n_d / V_d ile ayni bicim, H cercevesinde):

    c = ex + d_az                    (govdeye gore azimut, deg)       (6)
    g = k_dik * eps + d_el           (ufka gore yol acisi, deg)       (7)
    n_d = [cos g cos c, cos g sin c, -sin g]                          (8)
    v_d = kelepce(|v_simdi| + k_a, v_min, v_max)                      (9)
    v_H = v_d * n_d

k_dik (--dikey-oran) varsayilan 1.0'dir; 1.0 iken (7) saf LOS'tur (bkz.
"DIKEY ORAN").

MENZIL NEREDE KULLANILIR: (5)'te oncu acisinin OLCEGI olarak (izin verilen ve
brifingde acikca onerilen kullanim: "menzili kapanis hesabinda kullanabilirsin"),
(1)'deki aim rampasini cozmek, TERMINAL kapisi ve R_nokta tanilamasi icin.
Hedef HIZI / YONU / IVMESI hicbir yerde turetilmez -- (4) bir OLCUM
ozdesligidir, kestirim degildir.

ILERLEME SINYALI: menzil_m ve alan_kok BIRLIKTE. alan_kok = sqrt(w*h) sabit
aspect'te 1/R ile orantilidir, yani d(ln alan_kok)/dt = -R_nokta/R olceksiz
kapanma oranidir. Aspect degistigi icin (duman testi) alan_kok TEK BASINA
menzil vekili DEGILDIR; menzille birlikte terminal kapisini ve amac
fonksiyonunu besler.


3. HIZ YASASI (2LOSKF2'den)
---------------------------
    k_a = KA_TEPE * cos( min(theta/THETA_ESIK, 1) * pi/2 )           (10)
    v_d = kelepce(|v_simdi| + k_a, v_min, v_max)

theta = MEVCUT hiz vektoru ile komut yonu n_d arasindaki aci. Kaynak kod
theta yerine |e_y| (dikey piksel hatasi) kullaniyordu; BURADA DEGISTIRILDI
cunku bizim devir geometrimizde hedef ~27 derece YUKARIDA baslar (standoff
back=25 / down=13) ve |e_y| esigi tum terminal boyunca kapali kalir, arac hic
hizlanmazdi. Kopterde kamera ile hiz vektoru zaten AYRIK oldugu icin "hizali
miyim" sorusunun fiziksel karsiligi theta'dir.

Anlami: yonelmen gereken yon mevcut gidisinden cok farkliysa GAZ VERME, once
don (donus yaricapi V^2/a ile buyudugu icin yavas olmak yonu daha cabuk
duzeltir); hizalandikca tam gaza cik.

|v| tavani GORUNTULU_MAX_SPEED_MPS (iskelet de kelepceler). 2026-08-03'te
18 -> 20 m/s yukseltildi (params/swarm_copter.parm WPNAV_SPEED 2000 cm/s =
20 m/s zaten izin veriyordu). BU SAYI YASANIN EN BUYUK KISITIYDI: hedef
20 m/s seyrettigi icin 18'de kalindiginda donen hedef ancak COK genis
onculukle (85 deg) yakalaniyordu; 20'de dar onculukle bile (60 deg)
yakalaniyor (los_test.test_menzil_tavani_kritik: 22.4 m -> 1.5 m; duz rotada
30 s sonunda menzil 88.7 m -> 28.7 m). HIZ ACIGI UCUSTA YENIDEN DOGABILIR:
tirmanista yatay bilesen V*cos(gamma)'ya duser ve WPNAV_SPEED_UP 10 m/s
dikeyi ayrica kirpar; bu yuzden genis oncu kelepcesi korunur.
Dikey tavan params/swarm_copter.parm: WPNAV_SPEED_UP 10, WPNAV_SPEED_DN 5.
|v_z| tavani asarsa once TUM vektor olceklenir (YON KORUNUR -- gudum
acisindan onemli olan budur), taban hiza inildiginde yalniz v_z kirpilir.

DIKEY ORAN (--dikey-oran, k_dik): hedef ucak 20 m/s seyirde; kopter tavani
da 20 m/s ama TIRMANIRKEN yatay bilesen V*cos(gamma)'ya duser, yani dikey
kapanma sirasinda fiilen hedeften yavas kalinir. Kuyruk kovalamacada menzil
kapanma hizi
    R_nokta = V_t * cos(theta_t) - V_p        (theta_t = LOS ile hedef hizi arasi)
oldugu icin LOS'un hedef hiz vektorunden SAPIK olmasi kapanmayi iyilestirir.
Devir aninda hedef ~27 derece YUKARIDADIR; saf LOS ile hemen tirmanmak bu aci
avantajini ~1.5 s'de harcar ve geriye kazanilamaz duz kuyruk kovalamacasi
kalir. k_dik < 1 dikey kapanmayi geciktirip aci avantajini korur. VARSAYILAN
1.0 (saf LOS) birakildi ki makalenin temel yasasi bozulmasin; k_dik bir DENEY
DUGMESIDIR. NOT: theta_t ne olculur ne kestirilir; yukaridaki bagintii
yalnizca dugmenin GEREKCESIDIR.


3b. HAM KADRAJ (FOV) KORUMASI -- 2026-08-03 ELIPS KOSUSUNUN KOK NEDENI
----------------------------------------------------------------------
Ilk sim kosusunda (run/denemeler/los_elips_20260803_173736) yetki 16 KEZ
alinip geri verildi. CSV analizi:

  * Her devirden ORTALAMA 1.2-1.4 s sonra tespit oluyor; kalan ~2.7 s
    karar vericinin dwell'i (komut yok, iskelet sifira sonumleniyor).
    Toplam 41.9 s tazelik / 293 s pencere.
  * Kayiptan onceki 1 s'de ortalama cmd_vz = -2.34 m/s (TIRMANIS).
  * bbox.log tutumu: konumlu fazda pitch ortanca -1.8 deg, GORUNTULU fazda
    ortanca +11.2, p95 +31.3 deg (BURUN YUKARI).

GEOMETRI: kamera GOVDEYE SABIT +30 deg monte. Ham kadrajin gordugu eksen
ufka gore (mount + pitch) yukseliste. Hedefin yukselisi eps ise hedef,
eksenin

    alt_aci = (mount + pitch) - eps                                  (12)

kadar ALTINDADIR ve dikey yari-kadraj yalnizca 20.07 deg'dir. Olculen
degerlerle: konumluda 30 - 1.8 - 11.1 = 17.1 deg (ZAR ZOR ICERIDE),
goruntuluda 30 + 11.2 - 15.7 = 25.5 deg (KADRAJ DISI).

Yani hedefi kaybettiren sey ne bbox kalitesi ne de yatay gudum; KENDI
TIRMANIS KOMUTUMUZUN URETTIGI BURUN-YUKARI PITCH'tir. Kopter tirmanmak
(ve tirmanirken yatay hizi korumak) icin itki vektorunu geri yatirir;
kamera onunla birlikte yukari bakar ve zaten eksenin 16-19 derece altinda
duran hedef alttan kadrajdan cikar. Sonra dwell boyunca iskelet komutu
sifira sonumler -> kopter frenler -> pitch DAHA da yukari gider -> kisir
dongu.

COZUM (iki katmanli):
  (a) FOV KORUMASI: pitch OLCULUR (Olcum.pitch_rad), (12) hesaplanir ve
      komut yol acisi g, hedefi kadrajda tutacak kadar ASAGI cekilir:
          fazla = |alt_aci| - fov_marj_deg
          alt_aci > 0 ise  g -= k_fov * fazla   (hedef altta -> az tirman)
          alt_aci < 0 ise  g += k_fov * fazla   (hedef ustte -> daha tirman)
      Bu KAPALI DONGUDUR: g azalinca tirmanis azalir, pitch duser, alt_aci
      kuculur, koruma kendini geri ceker. fov_marj_deg = 14 secildi:
      20.07 deg dikey yari-kadrajin ~%70'i, kalan pay pitch salinimi
      (5-95 arasi 33 deg olculdu) ve bbox merkez gurultusu icin.
  (b) TIRMANMA TAVANI (tirmanma_max_mps): komutlanan YUKARI hiz sert
      kirpilir. DIKKAT -- bu, "dikey tavanda YONU koru" kuralinin bilincli
      ISTISNASIDIR: yonu korumak icin tum vektoru olceklemek yatay hizi
      da oldururdu, oysa hedef 20 m/s gidiyor ve yatay hiz butcemiz zaten
      dar. Sensor kisitini (hedefi gorebilmek) gudum optimalliginin
      UZERINE koyuyoruz: kilit kaybi her seyi kaybettirir.

DIKEY KAPANMAYI GECIKTIRMEK PAHALI DEGILDIR: carpisma icin eps'in sifira
inmesi SART DEGIL; sabit kerteriz + kapanan menzil yeterlidir (bkz. bolum
2). Tirmanisi yavaslatmak yalnizca temas acisini degistirir.


4. TERMINAL KAPISI
------------------
menzil <= terminal_menzil_m  YA DA  alan_kok >= terminal_alan_kok iken:
  * onculuk integratorleri DONDURULUR (d_az, d_el sabit kalir),
  * k_a tam degerine zorlanir (hizalanma kapisi bypass) -> tam gaz.
Gerekce: R -> 0'da q_nokta 1/R ile patlar; kalan ucus suresi (t_go) arac
gecikmesinden (LPF 0.35 s + kopter dinamigi) kucuk oldugu icin bu asamada
uretilen duzeltme ZATEN UYGULANAMAZ, yalnizca son anda savrulma (ve isabetsiz
gecis) uretir. Klasik PN uygulamalarinda da terminal dondurma standarttir.


5. YAW (FOV) KONTROLCUSU
------------------------
FOV 66 deg yatay (+-33), kamera gimbalsiz -> hedefi yatayda kadrajda tutmak
gudumun isidir. legacy1/2LOSKF2 yasasi aynen (birim uyumlu) tasindi:

    yaw_rate = kp * ex + kd * ex_nokta                                (11)
    kp = 0.71 [1/s], kd = 0.917 [s]

ex_nokta = q_az_nokta - yaw_nokta oldugundan (11) ortuk olarak hem LOS ileri
beslemesi (kd * q_az_nokta) hem de YAW ORAN SONUMLEMESI (-kd * yaw_nokta)
icerir; kaynak yasanin gucu buradadir.

Kaynak koddaki MAX_YAW_RATE = 360 deg/s BURADA 60 deg/s'ye indirildi ve
ustune ivme (slew) siniri kondu: yildizlar_gudum.sh ve senaryo.sh notlari
yavas dongu + buyuk yaw basamaklarinin titremeye yol actigini kayit altina
aliyor. Ayrica olu bant (bbox merkez gurultusu ~+-0.15 deg) sifir civari
limit-cevrimini keser.

Yaw komutu HIZ VEKTORUNU DEGISTIRMEZ (komut NED'dedir); yalnizca kamerayi
yonlendirir, yani yaw kanalinin agresifligi yorunge kararliligini degil
GORUNTU kalitesini etkiler.


6. OLCUM SAGLIGI
----------------
  * AYNI ORNEK KORUMASI: dongu 20 Hz, kamera farkli hizda. Ayni bbox iki
    dongude gorulurse (ex,ey,w,h ozdes) turev (ex - ex_onceki)/dt = 0 cikar
    ve LOS orani SISTEMATIK OLARAK DUSUK okunur. Yeni ornek gelene kadar
    TUREV SUZGECI dondurulur ve dt biriktirilir. (Oncu acisi (6) buna ragmen
    her dongude cozulur -- bkz. bolum 2.) Olculdu: koruma olmadan 8 deg/s
    gercek LOS orani 6.3 deg/s okunuyordu.
  * GECIKME TELAFISI: ex/ey yakalanma anina aittir; simdiki kerteriz
    ex + ex_nokta * gecikme ile ongorulur (gecikme = bbox_yas_s + sabit ek).
  * BOSLUK SIFIRLAMA: iki gecerli olcum arasi > sifirla_bosluk_s ise turev
    durumlari sifirlanir (bayat turev = sahte buyuk LOS orani).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

from goruntulu_temel import (Komut, GoruntuluDongu, GoruntuluKontrolcu,
                             govde_ileri_ned)

# yildizlar_gimbal depo kokundedir; senaryo.sh 'cd guidance_allstar' yaptigi
# icin ust dizin sys.path'te olmayabilir. SALT OKUNUR import (yalniz testler
# ayna fonksiyonlari dogrulasin diye erisilebilir tutulur).
_KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _KOK not in sys.path:
    sys.path.append(_KOK)


# ------------------------------------------------------------------ sabitler
#
# KAMERA (models/suru_drone_1/model.sdf + yildizlar_gimbal.py:99-101)
KAMERA_GENISLIK_PX = 1280.0
KAMERA_YUKSEKLIK_PX = 720.0
KAMERA_HFOV_DEG = 66.0                  # -> fx = 640/tan(33) = 985.5 px
FX_PX = (KAMERA_GENISLIK_PX / 2.0) / math.tan(math.radians(KAMERA_HFOV_DEG / 2.0))
FOV_YATAY_YARI_DEG = KAMERA_HFOV_DEG / 2.0                       # +-33.0
FOV_DIKEY_YARI_DEG = math.degrees(math.atan(
    math.tan(math.radians(KAMERA_HFOV_DEG / 2.0))
    * KAMERA_YUKSEKLIK_PX / KAMERA_GENISLIK_PX))                 # +-20.07

# Aim menzil sonumlemesi (yildizlar_gimbal.aim_etkin_deg ile BIREBIR ayni
# rampa; los_test.py ikisinin sayisal esitligini dogrular). senaryo.sh
# YILDIZ_AIM=0 verdigi icin varsayilan calismada bu rampa NO-OP'tur.
AIM_TAM_MENZIL_M = 120.0
AIM_SIFIR_MENZIL_M = 250.0

# Arac tavanlari (params/swarm_copter.parm satir 30-40)
WPNAV_SPEED_UP_MPS = 10.0
WPNAV_SPEED_DN_MPS = 5.0


def wrap180(a_deg: float) -> float:
    """Aciyi (-180, 180] araligina indirger."""
    return (float(a_deg) + 180.0) % 360.0 - 180.0


def kelepce(x, alt, ust):
    return alt if x < alt else (ust if x > ust else x)


def analitik_aim(back_m: float, down_m: float) -> float:
    """aim = -atan(down/back). yildizlar_gimbal.analitik_aim'in AYNASI.

    Ayna tutulur (import edilmez) ki depo kokune erisilemeyen bir ortamda
    (birim test, farkli cwd) da calissin. los_test.py iki uygulamanin sayisal
    ozdesligini dogrular. senaryo.sh AIM=0 sabitledigi icin bu fonksiyon
    yalniz elle --back/--down deneyi yapilirsa devreye girer.
    """
    return -math.degrees(math.atan(float(down_m) / max(1e-6, float(back_m))))


def aim_etkin(aim_deg: float, menzil_m):
    """Menzil-farkinda aim. yildizlar_gimbal.SanalGimbal.aim_etkin_deg AYNASI.

    menzil None -> tam aim (sonumlenecek bilgi yok).
    """
    if menzil_m is None:
        return float(aim_deg)
    r = float(menzil_m)
    if r <= AIM_TAM_MENZIL_M:
        k = 1.0
    elif r >= AIM_SIFIR_MENZIL_M:
        k = 0.0
    else:
        k = ((AIM_SIFIR_MENZIL_M - r)
             / (AIM_SIFIR_MENZIL_M - AIM_TAM_MENZIL_M))
    return float(aim_deg) * k


def eps_coz(ex_deg, ey_deg, aim_etkin_deg):
    """(1) Hedefin UFKA gore yukselisi [deg], izdusum duzeltmeli.

        eps = atan( -tan(ey) * cos(ex) ) - aim_etkin

    NEDEN cos(ex): ey kadraj merkezinden okunan bir PERSPEKTIF acisidir;
    yildizlar_gimbal zincirinde  tan(-ey) = tan(eps + aim) / cos(psi)  cikar
    (psi = sanal kadrajdaki yatay aci, aim=0 iken ex'in kendisi). Duzeltme
    yapilmazsa hedef kadrajin kenarindayken yukselis SISIK okunur: ex=20 deg,
    eps=12 deg'de 12.75 deg (%6). los_test.Geometri bu bagintii GERCEK
    SanalGimbal ile dogrular.
    """
    ey_r = math.radians(float(ey_deg))
    # tan(+-90) tasmasina karsi ey'i sinirla (kadraj disi bbox gelmez ama
    # bayat/bozuk veri gelebilir).
    ey_r = kelepce(ey_r, -math.radians(85.0), math.radians(85.0))
    ex_r = kelepce(math.radians(float(ex_deg)), -math.radians(89.0),
                   math.radians(89.0))
    return math.degrees(math.atan(-math.tan(ey_r) * math.cos(ex_r))) \
        - float(aim_etkin_deg)


class TurevSuzgec:
    """Birinci mertebe suzgecli sayisal turev.

    Ham turev (x - x_onceki)/dt bbox merkez titremesini (+-3 px = +-0.15 deg)
    dt=0.04 s'te +-3.8 deg/s'lik gurultuye cevirir; tau ile bu ~1/3'e iner.
    tau BUYUDUKCE gurultu duser ama LOS orani gecikir -> onculuk gec kalir.
    (Kaynak kodlarda ayni is beta_qdot=0.2 sabit-agirlikli LPF ile yapiliyordu;
    burada dt-farkinda yazildi cunku dongu adimi degisken -- sabit agirlik
    dongu yavaslayinca farkli bir kesim frekansi demektir, bu depoda daha once
    kok neden olarak isaretlenmis bir hata sinifi.)
    """

    def __init__(self, tau_s):
        self.tau = float(tau_s)
        self.x_onceki = None
        self.d = 0.0

    def sifirla(self):
        self.x_onceki = None
        self.d = 0.0

    def guncelle(self, x, dt):
        if x is None or dt <= 1e-6:
            return self.d
        if self.x_onceki is None:
            self.x_onceki = float(x)
            return self.d
        ham = (float(x) - self.x_onceki) / dt
        self.x_onceki = float(x)
        a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
        self.d += a * (ham - self.d)
        return self.d


# -------------------------------------------------------------- kontrolcu

class LosKontrolcu(GoruntuluKontrolcu):
    """LOS oranli (PN) goruntulu gudum kontrolcusu.

    Tune parametrelerinin tamami __init__ argumanidir; main() bunlari argparse
    ile disari acar (or. `python3 los_gudum.py --n 4.0 --dikey-oran 0.6`).
    """

    ad = "los"

    def __init__(self,
                 # --- PN cekirdegi (denklem 5-6) ---
                 k_pn=1.0,              # (5) LOS oran kazanci. 1.0 = dead-beat
                                        # ("bir sonraki anda LOS oranini tam
                                        # sifirla"). >1 fazla surer (asim),
                                        # <1 az. Etkin klasik N icin (7).
                 tau_yak_s=0.8,         # (6) onculuge yaklasim zaman sabiti.
                                        # Arac tepkisiyle (komut LPF 0.35 s +
                                        # kopter dinamigi ~0.5 s) ayni mertebe
                                        # olmali; kucuk secilirse kontrolcu
                                        # aracin uygulamadigi onculugu
                                        # "yetersiz" sanip siser (windup).
                 tau_onc_s=20.0,        # oncu yikama sabiti (legacy1'in
                                        # K = 1/tau'su). Capisma suresinden
                                        # (~5-15 s) COK buyuk olmali, yoksa
                                        # sizinti LOS oranini sifira indirmez
                                        # (bkz. bolum 2).
                 onc_az_max_deg=85.0,   # yatay oncu kelepcesi. FOV ile ILGISI
                                        # YOK (oncu HIZ VEKTORUNU dondurur,
                                        # kamerayi degil; yaw ayri surulur).
                                        # GENIS OLMALI: carpisma ucgeni
                                        # sin(d) = (V_t/V_p) sin(theta_t)
                                        # oldugundan V_p < V_t iken 80 dereceye
                                        # varan oncu ister. los_test kapali
                                        # dongu taramasi, v_max=18 (hiz acigi),
                                        # daire 6/9/14 deg/s:
                                        #   60 deg -> 22.4/15.1/9.8 m KACIRMA
                                        #   75 deg -> 11.4/ 6.4/3.1 m
                                        #   85 deg ->  1.3/ 1.5/1.2 m ISABET
                                        # v_max=20 paritesinde kelepce artik
                                        # baglayici degil (60 deg de vuruyor),
                                        # ama tirmanista hiz acigi geri dondugu
                                        # icin genis kelepce korunur.
                                        # Doygunlukta park etme riskini yikama
                                        # (tau_onc) ve terminal dondurma keser.
                 onc_el_max_deg=25.0,   # dikey oncu: kopterin dikey hiz
                                        # yetkisi zaten 10/5 m/s ile sinirli,
                                        # buyuk dikey oncu uygulanamaz
                                        # (35 deg denendi, fark <0.3 m).
                 # --- olcum isleme (bolum 6) ---
                 tau_turev_s=0.20,      # LOS orani suzgeci (bkz. TurevSuzgec)
                 tau_yaw_nokta_s=0.20,  # yaw orani suzgeci: (2)'de ex_nokta ile
                                        # TOPLANDIGI icin tau_turev ile AYNI
                                        # olmali. Farkli olursa iki sinyal
                                        # farkli gecikir ve toplamlari kendi
                                        # donusumuzu tam goturemez -- olculdu:
                                        # 0.15 vs 0.20'de 20 deg/s yaw altinda
                                        # 3.2 deg sahte oncu kaliyordu.
                 gecikme_ek_s=0.05,     # bbox_yas_s ustune sabit boru hatti
                                        # gecikmesi (ros -> redis -> dongu)
                 sifirla_bosluk_s=0.60, # bu kadar olcumsuz kalinirsa turevler
                                        # sifirlanir (bayat turev sahte oran)
                 # --- hiz yasasi (denklem 10) ---
                 v_max_mps=None,        # None -> cfg.GORUNTULU_MAX_SPEED_MPS
                 v_min_mps=4.0,         # |v_simdi| hatali okunursa komut
                                        # cokmesin diye taban
                 ka_tepe_mps=2.0,       # 2LOSKF2 KA_PEAK = 2.0 (aynen)
                 theta_esik_deg=35.0,   # kosinus cani genisligi. 2LOSKF2'de
                                        # 300 px = 17.8 deg idi ama olcut
                                        # |e_y|'di; burada olcut theta oldugu
                                        # icin (bkz. bolum 3) daha genis.
                 dikey_oran=1.0,        # k_dik: 1.0 = saf LOS
                 # --- ham kadraj (FOV) korumasi, bolum 3b ---
                 mount_deg=None,        # None -> $YILDIZ_MOUNT (0.0)
                 fov_marj_deg=14.0,     # (12)'de izin verilen |alt_aci|.
                                        # Dikey yari-kadraj 20.07 deg; kalan
                                        # 6 deg pay pitch salinimi ve bbox
                                        # gurultusu icin.
                 k_fov=1.0,             # koruma kazanci (deg cikti / deg hata)
                 fov_duzeltme_max=30.0, # korumanin g'yi cekebilecegi en fazla
                 tirmanma_max_mps=3.0,  # komutlanan YUKARI hiz tavani.
                                        # Olcum: kayip aninda ortalama
                                        # cmd_vz = -2.34, p5 = -8.66 m/s idi.
                 gamma_min_deg=-25.0,   # yol acisi kelepceleri: kopter dik
                 gamma_max_deg=55.0,    # tirmanabilir ama 55 deg ustunde yatay
                                        # kapanma tumden durur
                 # --- terminal kapisi (bolum 4) ---
                 terminal_menzil_m=12.0,
                 terminal_alan_kok=55.0,  # SIM ILE KALIBRE EDILECEK. 26 m'de
                                          # bbox ~34 px genis olculdu; alan_kok
                                          # aspect'e bagli oldugu icin bu esik
                                          # ilk kosuda loglardan duzeltilmeli.
                 # --- yaw (FOV) kontrolcusu (denklem 11) ---
                 yaw_acik=True,
                 kp_yaw=0.71,           # legacy1/2LOSKF2 tuned degeri
                 kd_yaw=0.917,          # legacy1/2LOSKF2 tuned degeri
                 yaw_olu_bant_deg=0.8,  # bbox merkez gurultusu ~+-0.15 deg;
                                        # 0.8 deg olu bant limit-cevrimini keser
                 yaw_rate_max_dps=60.0, # kaynak koddaki 360'tan indirildi
                 yaw_ivme_max_dps2=120.0,  # basamak yerine rampa -> kamera
                                           # sarsintisi/blur azalir
                 tau_yaw_cikis_s=0.10,
                 # --- geometri / emniyet ---
                 aim_deg=None,          # None -> $YILDIZ_AIM (senaryo.sh 0)
                 back_m=None, down_m=None,   # yalniz aim verilmediginde ve
                                             # YILDIZ_AIM da yoksa turetim icin
                 min_irtifa_m=12.0,     # bu irtifanin altinda alcalma sifirlanir
                 menzil_min_m=3.0, menzil_max_m=400.0,
                 menzil_varsayilan_m=40.0):   # (5) icin menzil hic yoksa
                                              # kullanilan kaba olcek; devir
                                              # kapisi <=60 m oldugu icin
                                              # tipik devir menzilinin ortasi
        import guidance_config as cfg   # yerel import: test ortami hafif kalsin
        self.v_max = float(v_max_mps if v_max_mps is not None
                           else getattr(cfg, "GORUNTULU_MAX_SPEED_MPS", 18.0))
        self.v_min = float(v_min_mps)

        self.k_pn = float(k_pn)
        self.tau_yak = float(tau_yak_s)
        self.tau_onc = float(tau_onc_s)
        self.onc_az_max = float(onc_az_max_deg)
        self.onc_el_max = float(onc_el_max_deg)

        self.tau_turev = float(tau_turev_s)
        self.gecikme_ek = float(gecikme_ek_s)
        self.sifirla_bosluk = float(sifirla_bosluk_s)

        self.ka_tepe = float(ka_tepe_mps)
        self.theta_esik = float(theta_esik_deg)
        self.dikey_oran = float(dikey_oran)
        # VARSAYILAN 30.0 -> 0.0 (2026-08-04 montaj gecisi): sim montaji
        # 0 dereceye alindi (pitch-servo gimbal karari; sim'de kopter
        # neredeyse hic egilmedigi icin 0 sabit kamera ~ ideal gimbal).
        # $YILDIZ_MOUNT hala tek kaynak (scripts/standoff_geom.sh).
        self.mount_deg = float(mount_deg if mount_deg is not None
                               else os.environ.get('YILDIZ_MOUNT', 0.0))
        self.fov_marj = float(fov_marj_deg)
        self.k_fov = float(k_fov)
        self.fov_duzeltme_max = float(fov_duzeltme_max)
        self.tirmanma_max = float(tirmanma_max_mps)
        self.gamma_min = float(gamma_min_deg)
        self.gamma_max = float(gamma_max_deg)

        self.terminal_menzil = float(terminal_menzil_m)
        self.terminal_alan_kok = float(terminal_alan_kok)

        self.yaw_acik = bool(yaw_acik)
        self.kp_yaw = float(kp_yaw)
        self.kd_yaw = float(kd_yaw)
        self.yaw_olu_bant = float(yaw_olu_bant_deg)
        self.yaw_rate_max = float(yaw_rate_max_dps)
        self.yaw_ivme_max = float(yaw_ivme_max_dps2)
        self.tau_yaw_cikis = float(tau_yaw_cikis_s)

        self.min_irtifa = float(min_irtifa_m)
        self.menzil_min = float(menzil_min_m)
        self.menzil_max = float(menzil_max_m)
        self.menzil_varsayilan = float(menzil_varsayilan_m)

        # AIM oncelik sirasi:  --aim  >  $YILDIZ_AIM  >  analitik(back,down)
        # >  0.0. tools/senaryo.sh satir 84 `YILDIZ_AIM="${AIM:-0}"` export
        # eder, yani NORMAL kosuda aim = 0 ve (1) eps = -ey'e indirgenir.
        # SIFIR VARSAYILANI BILINCLIDIR: env yoksa analitik -27.5'e dusmek,
        # senaryo.sh'in sabitledigi 0 ile catisir ve eps'i 27 derece kaydirir.
        # Analitik turetim yalniz --back/--down ACIKCA verilirse kullanilir.
        self.back_m = float(back_m) if back_m is not None else None
        self.down_m = float(down_m) if down_m is not None else None
        if aim_deg is not None:
            self.aim_deg, self.aim_kaynak = float(aim_deg), 'arg'
        elif os.environ.get('YILDIZ_AIM') not in (None, ''):
            self.aim_deg, self.aim_kaynak = float(os.environ['YILDIZ_AIM']), 'env'
        elif self.back_m is not None and self.down_m is not None:
            self.aim_deg = analitik_aim(self.back_m, self.down_m)
            self.aim_kaynak = 'analitik'
        else:
            self.aim_deg, self.aim_kaynak = 0.0, 'varsayilan'

        # --- durumlar ---
        self.d_ex = TurevSuzgec(self.tau_turev)
        self.d_eps = TurevSuzgec(self.tau_turev)     # (3): DOGRUDAN eps turevi
        self.d_yaw = TurevSuzgec(float(tau_yaw_nokta_s))
        self.d_menzil = TurevSuzgec(0.50)        # yalniz tanilama/log
        self.d_ln_alan = TurevSuzgec(0.50)       # olceksiz kapanma orani
        self.onc_az = 0.0
        self.onc_el = 0.0
        self.yaw_rate_cikis = 0.0
        self.terminal = False
        self._imza = None            # son bbox ornegi (ayni ornek korumasi)
        self._t_son_yeni = None      # son YENI ornegin zamani
        self._t_onceki = None        # son komut() cagrisi
        self._menzil_son = None
        self.tani = {}               # test / hata ayiklama penceresi

    # -------------------------------------------------------------- devir

    def tohumla(self, devir):
        """Devir ani. Turev/onculuk durumlari SIFIRDAN baslar: konumlu faz
        standoff'ta dengede oldugu icin LOS orani ~0'dir, sifir onculuk dogru
        baslangictir. Hiz surekliligini iskeletin LPF'si zaten devir hiziyla
        tohumluyor (goruntulu_temel.py:388-393), burada tekrarlanmaz.
        """
        self.d_ex.sifirla()
        self.d_eps.sifirla()
        self.d_yaw.sifirla()
        self.d_menzil.sifirla()
        self.d_ln_alan.sifirla()
        self.onc_az = 0.0
        self.onc_el = 0.0
        self.yaw_rate_cikis = 0.0
        self.terminal = False
        self._imza = None
        self._t_son_yeni = None
        self._t_onceki = None
        self._menzil_son = None
        # Konumlu fazin son menzili varsa menzil zincirine tohum olsun
        # (devir artik estimator menzili <= 60 m kapisiyla oluyor).
        if devir and devir.get('range_m'):
            try:
                self._menzil_son = kelepce(float(devir['range_m']),
                                           self.menzil_min, self.menzil_max)
            except (TypeError, ValueError):
                self._menzil_son = None

    # -------------------------------------------------------------- menzil

    def _menzil_coz(self, o):
        """Menzil YALNIZ estimatordan; yoksa son bilinen deger tutulur.

        bbox genisliginden menzil TURETILMEZ: duman testi (2026-08-03) bbox
        genisliginin aspect'e bagli oldugunu gosterdi (virajda 204 m'de 31 px,
        standoff 26 m'de arkadan ortanca 34 px). Kaynak kodlardaki
        c_ptz = REAL_TARGET_WIDTH * F_OC / w bu ortamda GECERSIZDIR.
        """
        if o.menzil_m is not None and math.isfinite(o.menzil_m):
            r = kelepce(float(o.menzil_m), self.menzil_min, self.menzil_max)
            self._menzil_son = r
            return r, 'estimator'
        if self._menzil_son is not None:
            return self._menzil_son, 'son'
        return None, 'yok'

    # ---------------------------------------------------------- oncu acisi

    def _onc_guncelle(self, onc_deg, q_nokta_dps, R_m, v_ref, onc_max, dt):
        """(5)-(6): carpisma ucgeninin artigindan oncu acisini gunceller.

        Doner: (yeni_onc_deg, N_etkin). N_etkin yalniz LOG/makale icindir --
        (7) ile hesaplanan, bu dongudeki KLASIK PN esdegeri.
        """
        sd = math.sin(math.radians(onc_deg))
        artik = self.k_pn * R_m * math.radians(q_nokta_dps) / max(v_ref, 1e-3)
        s_max = math.sin(math.radians(onc_max))
        gerek = math.degrees(math.asin(kelepce(sd + artik, -s_max, s_max)))
        onc_yeni = onc_deg + ((gerek - onc_deg) / self.tau_yak
                              - onc_deg / self.tau_onc) * dt
        onc_yeni = kelepce(onc_yeni, -onc_max, onc_max)
        cd = max(math.cos(math.radians(onc_deg)), 0.2)
        n_etkin = 1.0 + R_m / (max(v_ref, 1e-3) * self.tau_yak * cd)
        return onc_yeni, n_etkin

    # --------------------------------------------------------------- komut

    def komut(self, o) -> Komut:
        dt_dongu = max(1e-3, float(o.dt))

        # --- 0. kendi yaw oranimiz (her cagride, bbox tazeliginden bagimsiz) ---
        yaw_deg = None if o.yaw_rad is None else math.degrees(o.yaw_rad)
        yaw_nokta = 0.0
        if yaw_deg is not None:
            # Sarma: turev suzgecine SUREKLI bir sinyal ver; aci -180/+180'de
            # atlarsa turev 360/dt gibi sahte devasa bir deger uretir.
            if self.d_yaw.x_onceki is not None:
                yaw_deg = self.d_yaw.x_onceki + wrap180(yaw_deg
                                                        - self.d_yaw.x_onceki)
            yaw_nokta = self.d_yaw.guncelle(yaw_deg, dt_dongu)

        # Bu cagride yeni bbox ornegi var mi? (ayni ornek korumasi, bolum 6)
        imza = (o.ex_deg, o.ey_deg, o.bbox_w, o.bbox_h)
        yeni_ornek = (imza != self._imza)
        self._imza = imza

        t = float(o.t)
        dt_olcum = dt_dongu if self._t_son_yeni is None else (t - self._t_son_yeni)
        if self._t_onceki is not None and (t - self._t_onceki) > self.sifirla_bosluk:
            # Uzun bosluktan sonra bayat turevle devam etmek yerine sifirla.
            self.d_ex.sifirla()
            self.d_eps.sifirla()
            self.d_menzil.sifirla()
            self.d_ln_alan.sifirla()
            dt_olcum = dt_dongu
        self._t_onceki = t
        dt_olcum = kelepce(dt_olcum, 1e-3, 0.5)

        # --- 1. menzil, aim, ilerleme sinyali ---
        menzil, menzil_kaynak = self._menzil_coz(o)
        aim_e = aim_etkin(self.aim_deg, menzil)

        # --- 2. acisal hatalar ve LOS oranlari (1)-(3) ---
        ex = 0.0 if o.ex_deg is None else float(o.ex_deg)
        ey = 0.0 if o.ey_deg is None else float(o.ey_deg)
        eps = kelepce(eps_coz(ex, ey, aim_e), -80.0, 85.0)      # (1)

        if yeni_ornek:
            ex_nokta = self.d_ex.guncelle(ex, dt_olcum)
            q_el_nokta = self.d_eps.guncelle(eps, dt_olcum)
            if menzil is not None:
                self.d_menzil.guncelle(menzil, dt_olcum)
            if o.alan_kok is not None and o.alan_kok > 1e-3:
                self.d_ln_alan.guncelle(math.log(float(o.alan_kok)), dt_olcum)
            self._t_son_yeni = t
        else:
            ex_nokta, q_el_nokta = self.d_ex.d, self.d_eps.d

        q_az_nokta = ex_nokta + yaw_nokta           # (2) [deg/s]

        # --- 3. gecikme telafisi (olcum yakalanma anina ait) ---
        gecikme = kelepce((o.bbox_yas_s if math.isfinite(o.bbox_yas_s) else 0.0)
                          + self.gecikme_ek, 0.0, 0.4)
        ex_ong = ex + kelepce(ex_nokta * gecikme, -8.0, 8.0)
        eps_ong = eps + kelepce(q_el_nokta * gecikme, -8.0, 8.0)

        # --- 3b. kendi hizimiz (onculuk olcegi ve hizalanma kapisi icin) ---
        v_simdi = 0.0
        hiz_h_birim = None
        if o.vel_ned is not None:
            hiz = np.asarray(o.vel_ned, dtype=float).reshape(3)
            v_simdi = float(np.linalg.norm(hiz))
            if v_simdi > 1.0 and o.yaw_rad is not None:
                # mevcut hizi H cercevesine cevir (yalniz yaw dondurmesi)
                cy_, sy_ = math.cos(o.yaw_rad), math.sin(o.yaw_rad)
                hiz_h_birim = np.array(
                    [cy_ * hiz[0] + sy_ * hiz[1],
                     -sy_ * hiz[0] + cy_ * hiz[1], hiz[2]]) / v_simdi

        # --- 4. terminal kapisi (bolum 4) ---
        terminal = False
        if menzil is not None and menzil <= self.terminal_menzil:
            terminal = True
        if o.alan_kok is not None and o.alan_kok >= self.terminal_alan_kok:
            terminal = True
        self.terminal = terminal

        # --- 5. MENZIL OLCEKLI ONCU ACISI (5)-(6) ---
        # HER DONGUDE cozulur (bkz. bolum 2): q_nokta bir hiz kestirimidir,
        # sifir-mertebe tutucu ile dt_dongu kadar integre etmek dogrudur.
        # Ornek beklerken dondurulan tek sey turev suzgecidir.
        v_ref = kelepce(v_simdi if v_simdi > 2.0 else self.v_max,
                        5.0, self.v_max)
        R_olcek = menzil if menzil is not None else self.menzil_varsayilan
        n_etkin_az = n_etkin_el = 0.0
        if not terminal:
            self.onc_az, n_etkin_az = self._onc_guncelle(
                self.onc_az, q_az_nokta, R_olcek, v_ref,
                self.onc_az_max, dt_dongu)
            self.onc_el, n_etkin_el = self._onc_guncelle(
                self.onc_el, q_el_nokta, R_olcek, v_ref,
                self.onc_el_max, dt_dongu)

        # --- 6. komut yonu (6)-(8) ---
        c_deg = wrap180(ex_ong + self.onc_az)
        g_ham = self.dikey_oran * eps_ong + self.onc_el

        # HAM KADRAJ (FOV) KORUMASI -- bolum 3b. Kamera govdeye sabit oldugu
        # icin komut yol acisi -> tirmanis -> burun yukari pitch -> hedef
        # kadrajin ALTINDAN cikiyor. pitch OLCULUP geri beslenir.
        alt_aci = fov_duzeltme = 0.0
        if o.pitch_rad is not None:
            alt_aci = (self.mount_deg + math.degrees(o.pitch_rad)) - eps
            if alt_aci > self.fov_marj:
                fov_duzeltme = -self.k_fov * (alt_aci - self.fov_marj)
            elif alt_aci < -self.fov_marj:
                fov_duzeltme = self.k_fov * (-self.fov_marj - alt_aci)
            fov_duzeltme = kelepce(fov_duzeltme, -self.fov_duzeltme_max,
                                   self.fov_duzeltme_max)
        g_deg = kelepce(g_ham + fov_duzeltme, self.gamma_min, self.gamma_max)

        cg, sg = math.cos(math.radians(g_deg)), math.sin(math.radians(g_deg))
        cc, sc = math.cos(math.radians(c_deg)), math.sin(math.radians(c_deg))
        n_d = np.array([cg * cc, cg * sc, -sg])

        # --- 7. hiz buyuklugu (9)-(10) ---
        theta_deg = 0.0
        if hiz_h_birim is not None:
            theta_deg = math.degrees(math.acos(
                kelepce(float(np.dot(hiz_h_birim, n_d)), -1.0, 1.0)))
        if terminal:
            k_a = self.ka_tepe                      # tam gaz, kapi bypass
        else:
            oran = min(abs(theta_deg) / max(1e-6, self.theta_esik), 1.0)
            k_a = self.ka_tepe * math.cos(oran * math.pi / 2.0)
        v_d = kelepce(v_simdi + k_a, self.v_min, self.v_max)

        v_h = n_d * v_d

        # --- 8a. TIRMANMA TAVANI (bolum 3b-b): yalniz v_z kirpilir, vektor
        # OLCEKLENMEZ. Yonu korumak icin olceklemek yatay hizi da oldururdu;
        # hedef 20 m/s gidiyor ve yatay butce zaten dar. Sensor kisiti
        # (hedefi gorebilmek) gudum optimalliginin UZERINDEDIR.
        if v_h[2] < -self.tirmanma_max:
            v_h[2] = -self.tirmanma_max

        # --- 8b. dikey tavan: once YONU koruyarak olcekle, sonra kirp ---
        vz = v_h[2]
        tavan = WPNAV_SPEED_UP_MPS if vz < 0 else WPNAV_SPEED_DN_MPS
        if abs(vz) > tavan:
            olcek = tavan / abs(vz)
            if v_d * olcek >= self.v_min:
                v_h = v_h * olcek                    # yon korunur
            else:
                v_h = v_h * (self.v_min / v_d)
                v_h[2] = kelepce(v_h[2], -WPNAV_SPEED_UP_MPS, WPNAV_SPEED_DN_MPS)

        # --- 9. NED'e cevir + emniyet ---
        yaw_r = 0.0 if o.yaw_rad is None else float(o.yaw_rad)
        v_ned = govde_ileri_ned(yaw_r, v_h[0], v_h[1], v_h[2])

        if o.pos_ned is not None:
            irtifa = -float(o.pos_ned[2])       # pos_ned[2] asagi pozitif
            if irtifa < self.min_irtifa and v_ned[2] > 0.0:
                v_ned[2] = 0.0
        n = float(np.linalg.norm(v_ned))
        if n > self.v_max:
            v_ned = v_ned * (self.v_max / n)

        # --- 10. yaw (FOV) kontrolcusu (11) ---
        yaw_rate = None
        if self.yaw_acik:
            e = ex_ong
            e_dz = 0.0 if abs(e) <= self.yaw_olu_bant else \
                e - math.copysign(self.yaw_olu_bant, e)
            istek = self.kp_yaw * e_dz + self.kd_yaw * ex_nokta
            istek = kelepce(istek, -self.yaw_rate_max, self.yaw_rate_max)
            d_max = self.yaw_ivme_max * dt_dongu          # ivme (slew) siniri
            istek = kelepce(istek, self.yaw_rate_cikis - d_max,
                            self.yaw_rate_cikis + d_max)
            a = (dt_dongu / (dt_dongu + self.tau_yaw_cikis)
                 if self.tau_yaw_cikis > 1e-6 else 1.0)
            self.yaw_rate_cikis += a * (istek - self.yaw_rate_cikis)
            yaw_rate = self.yaw_rate_cikis

        # --- 11. tanilama penceresi ---
        self.tani = {
            'yeni_ornek': yeni_ornek, 'dt_olcum': dt_olcum,
            'menzil': menzil, 'menzil_kaynak': menzil_kaynak,
            'menzil_nokta': self.d_menzil.d,
            'alan_kok': o.alan_kok, 'ln_alan_nokta': self.d_ln_alan.d,
            'aim_etkin': aim_e, 'eps': eps, 'eps_ong': eps_ong,
            'ex_ong': ex_ong, 'ex_nokta': ex_nokta,
            'yaw_nokta': yaw_nokta,
            'q_az_nokta': q_az_nokta, 'q_el_nokta': q_el_nokta,
            'onc_az': self.onc_az, 'onc_el': self.onc_el,
            'n_etkin_az': n_etkin_az, 'n_etkin_el': n_etkin_el,
            'r_olcek': R_olcek, 'v_ref': v_ref,
            'sigma_az': wrap180(math.degrees(yaw_r) + ex_ong + self.onc_az),
            'sigma_el': g_deg,
            'c_deg': c_deg, 'g_deg': g_deg, 'n_d': n_d.copy(),
            'g_ham': g_ham, 'alt_aci': alt_aci,
            'fov_duzeltme': fov_duzeltme,
            'theta_deg': theta_deg, 'k_a': k_a, 'v_d': v_d,
            'v_simdi': v_simdi, 'terminal': terminal,
            'v_h': v_h.copy(), 'yaw_rate': yaw_rate,
        }
        return Komut(vel_ned=v_ned, yaw_rate_dps=yaw_rate)


# ----------------------------------------------------------------- main

def arg_ayristirici():
    p = argparse.ArgumentParser(
        description="LOS (PN) goruntulu gudum -- goruntulu_temel iskeleti")
    g = p.add_argument_group('PN cekirdegi')
    g.add_argument('--k-pn', type=float, default=1.0,
                   help='LOS oran kazanci; 1.0 = dead-beat (denklem 5)')
    g.add_argument('--tau-yak', type=float, default=0.8,
                   help='onculuge yaklasim zaman sabiti [s] (denklem 6)')
    g.add_argument('--tau-onc', type=float, default=20.0,
                   help='oncu aci yikama sabiti [s] (legacy1 K = 1/tau)')
    g.add_argument('--onc-az-max', type=float, default=35.0)
    g.add_argument('--onc-el-max', type=float, default=20.0)

    g = p.add_argument_group('olcum isleme')
    g.add_argument('--tau-turev', type=float, default=0.20)
    g.add_argument('--tau-yaw-nokta', type=float, default=0.20)
    g.add_argument('--gecikme-ek', type=float, default=0.05)
    g.add_argument('--sifirla-bosluk', type=float, default=0.60)

    g = p.add_argument_group('hiz yasasi')
    g.add_argument('--v-max', type=float, default=None,
                   help='varsayilan cfg.GORUNTULU_MAX_SPEED_MPS (su an 20). '
                        'DIKKAT: '
                        'iskelet (GoruntuluDongu) da cfg degeriyle kelepceler, '
                        'yani buradan 18 uzeri vermek TEK BASINA ETKISIZDIR; '
                        'guidance_config.GORUNTULU_MAX_SPEED_MPS de artmali.')
    g.add_argument('--v-min', type=float, default=4.0)
    g.add_argument('--ka-tepe', type=float, default=2.0,
                   help='2LOSKF2 KA_PEAK')
    g.add_argument('--theta-esik', type=float, default=35.0,
                   help='hizlanma kosinus cani genisligi [deg]')
    g.add_argument('--dikey-oran', type=float, default=1.0,
                   help='k_dik: 1.0 saf LOS; <1 aci avantajini korur')
    g.add_argument('--mount', type=float, default=None,
                   help='kamera montaj acisi [deg]; varsayilan $YILDIZ_MOUNT (0)')
    g.add_argument('--fov-marj', type=float, default=14.0,
                   help='ham kadrajda izin verilen |eksen-hedef| acisi [deg]')
    g.add_argument('--k-fov', type=float, default=1.0,
                   help='FOV koruma kazanci; 0 = koruma kapali')
    g.add_argument('--tirmanma-max', type=float, default=3.0,
                   help='komutlanan YUKARI hiz tavani [m/s]')
    g.add_argument('--gamma-min', type=float, default=-25.0)
    g.add_argument('--gamma-max', type=float, default=55.0)

    g = p.add_argument_group('terminal kapisi')
    g.add_argument('--terminal-menzil', type=float, default=12.0)
    g.add_argument('--terminal-alan-kok', type=float, default=55.0)

    g = p.add_argument_group('yaw (FOV) kontrolcusu')
    g.add_argument('--yaw-kapali', action='store_true',
                   help='yaw_rate gonderme (otopilotta birak) -- A/B icin')
    g.add_argument('--kp-yaw', type=float, default=0.71)
    g.add_argument('--kd-yaw', type=float, default=0.917)
    g.add_argument('--yaw-olu-bant', type=float, default=0.8)
    g.add_argument('--yaw-rate-max', type=float, default=60.0)
    g.add_argument('--yaw-ivme-max', type=float, default=120.0)

    g = p.add_argument_group('geometri / emniyet')
    g.add_argument('--aim', type=float, default=None,
                   help='aim [deg]; verilmezse $YILDIZ_AIM (senaryo.sh: 0)')
    g.add_argument('--back', type=float, default=None,
                   help='aim analitik turetimi icin standoff back [m]')
    g.add_argument('--down', type=float, default=None,
                   help='aim analitik turetimi icin standoff down [m]')
    g.add_argument('--min-irtifa', type=float, default=12.0)

    g = p.add_argument_group('dongu')
    g.add_argument('--loop-hz', type=float, default=20.0)
    g.add_argument('--tau', type=float, default=0.35, help='komut LPF [s]')
    g.add_argument('--sure', type=float, default=None)
    g.add_argument('--log', type=str, default=None)
    return p


def kontrolcu_kur(a):
    return LosKontrolcu(
        k_pn=a.k_pn, tau_yak_s=a.tau_yak, tau_onc_s=a.tau_onc,
        onc_az_max_deg=a.onc_az_max, onc_el_max_deg=a.onc_el_max,
        tau_turev_s=a.tau_turev, tau_yaw_nokta_s=a.tau_yaw_nokta,
        gecikme_ek_s=a.gecikme_ek, sifirla_bosluk_s=a.sifirla_bosluk,
        v_max_mps=a.v_max, v_min_mps=a.v_min,
        ka_tepe_mps=a.ka_tepe, theta_esik_deg=a.theta_esik,
        dikey_oran=a.dikey_oran, mount_deg=a.mount,
        fov_marj_deg=a.fov_marj, k_fov=a.k_fov,
        tirmanma_max_mps=a.tirmanma_max,
        gamma_min_deg=a.gamma_min, gamma_max_deg=a.gamma_max,
        terminal_menzil_m=a.terminal_menzil,
        terminal_alan_kok=a.terminal_alan_kok,
        yaw_acik=(not a.yaw_kapali), kp_yaw=a.kp_yaw, kd_yaw=a.kd_yaw,
        yaw_olu_bant_deg=a.yaw_olu_bant, yaw_rate_max_dps=a.yaw_rate_max,
        yaw_ivme_max_dps2=a.yaw_ivme_max,
        aim_deg=a.aim, back_m=a.back, down_m=a.down,
        min_irtifa_m=a.min_irtifa)


def main():
    a = arg_ayristirici().parse_args()
    k = kontrolcu_kur(a)
    print(f"[los] k_pn={k.k_pn} tau_yak={k.tau_yak}s tau_onc={k.tau_onc}s "
          f"aim={k.aim_deg:+.2f} deg ({k.aim_kaynak}) "
          f"v=[{k.v_min:.1f}..{k.v_max:.1f}] m/s ka_tepe={k.ka_tepe:.1f} "
          f"yaw={'acik' if k.yaw_acik else 'kapali'} "
          f"(kp={k.kp_yaw} kd={k.kd_yaw}) k_dik={k.dikey_oran:.2f} "
          f"fov[mount={k.mount_deg:.0f} marj={k.fov_marj:.0f} k={k.k_fov:.2f}] "
          f"tirmanma_max={k.tirmanma_max:.1f}", flush=True)
    GoruntuluDongu(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_yolu=a.log).calistir(a.sure)


if __name__ == '__main__':
    main()
