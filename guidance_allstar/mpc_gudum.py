#!/usr/bin/env python3
"""
mpc_gudum.py - GORUNTULU gudum: Model Ongorulu Kontrol (MPC) yontemi
====================================================================
goruntulu_temel.GoruntuluKontrolcu sozlesmesine uyar. Karar degiskeni
DOGRUDAN gelecek 3B hiz komutu dizisidir; bu yuzden MPC bu platformda
dogal oturur (sabit kanatta 3B hiz komutlanamadigi icin ayni formulasyon
o zaman uygulanamamisti).

------------------------------------------------------------------ MODEL
Sunumdaki (old_los_codes/"Goruntulu gudum ve yerden tespit.pdf", s.15-18)
sanal-koordinat MPC modeli:

    x = [u, v, Z]^T        u,v: hedefin piksel koordinatlari, Z: menzil
    u_input = [Vx,virt, Vy,virt, Vz,virt]^T
    u_{k+1} = u_k - (dt/Z_k) Vy,k
    v_{k+1} = v_k - (dt/Z_k) Vz,k
    Z_{k+1} = Z_k - dt Vx,k
    J = sum q1 u^2 + q2 v^2 + q3 (Z - Zref)^2 + r1 (Vx - Vprev)^2
    kisitlar: |u|<=umax, |v|<=vmax (FOV), Vmin<=Vx<=Vmax, V_NED = Rz(psi) V*

BURADAKI MODEL BUNUN AYNISIDIR, dort farkla (hepsi ortamin gercegi):
  (1) PIKSEL YERINE DERECE. Sanal gimbal zaten ex_deg/ey_deg veriyor;
      piksel modeli odak uzakligina bolununce birebir bu olur
      (kucuk aci: u_px = fx * tan(ex) ~ fx*ex_rad). Boylece 1280x720'ye
      bagimlilik kalkar, kazanclar kameradan bagimsizlasir.
  (2) BOZUCU TERIMI d. Sunumun modeli hedefi DURAGAN varsayar (u_dot
      yalniz kendi hizimiza bagli). Hedef 20 m/s ucuyor ve hedef
      hizini telemetriden turetmek YASAK (goruntulu_temel sozlesmesi).
      Bunun yerine d = (olculen aci turevi) - (modelin ongordugu aci
      turevi) olarak GORUNTUDEN kestirilir. d fiziksel olarak
      "hedefin dik hiz bileseni / menzil"dir; yani MPC'nin ihtiyaci
      olan tek hedef bilgisi bbox'tan gelir, telemetriden DEGIL.
  (3) YAW HIZI 4. GIRDI. Kamera govdeye sabit ve yatay yari-FOV 33 deg;
      hedefi kadrajda tutmak icin burun cevirmek gerekir. Yaw hizi
      modelin icinde oldugu icin AYRI BIR FOV KONTROLCUSU YOKTUR --
      MPC ayni maliyet fonksiyonunda hem gudumu hem kadraji cozer.
  (4) ODUL LINEER BBOX ALANI. Sunumdaki q3 (Z - Zref)^2 terimi
      menzili KAREYLE cezalandirir, yani kapanma tesviki UZAKTA guclu
      YAKINDA zayiftir -- carpismanin tam tersi. Burada odul dogrudan
      bbox ALANI (w*h, px^2) ve onun BIRINCI TUREVIDIR. Alan ~ K/r^2
      oldugu icin tesvik 1/r^3 ile buyur (40 m'de 1 birim, 20 m'de 8
      birim): terminal agresiflik bilincli olarak artar. Ikisi de
      nominal menzil yorungesi etrafinda dogrusallastirildigi icin
      maliyete YALNIZ LINEER terim girer -- Hessian ve cozum suresi
      degismez.
  (5) FOV SERT KISIT. Goruntulu gudumde hedefin kadrajdan cikmasi
      kosuyu BITIRIR, dolayisiyla kadraj bir "tercih" degil kisittir.
      Anahtar gozlem: bir adim sonraki kadraj degiskenleri girdide
      AFFINE'dir, yani dusey kadraj kisiti mevcut DIKEY HIZ
      DILIMININ, yatay kadraj kisiti ise YAW KUTUSUNUN daraltilmasina
      indirgenir -- ikisi de izdusumun zaten TAM cozdugu kume tipleri.
      Sert kisit boylece ek degisken ya da ek iterasyon MALIYETI
      OLMADAN gelir (bkz. _cbf_sinirlari). Ayrik CBF formunda
      yazildigi icin, devir aninda hedef zaten alt kenardayken bile
      kume BOSALMAZ: kisit "iyilestir" der, "aninda sagla" demez.
  (6) EYLEYICI GECIKMESI + IVME SINIRI. Zincir: iskelet LPF'si
      (tau=0.35) -> |v|<=18 kelepce -> otopilotun hiz dongusu, ki o
      WPNAV_ACCEL = 5 m/s^2 ile IVME SINIRLIDIR. Yani hiz komutuna
      tepki ani degil, RAMPA'dir: yanal hizi 15 m/s degistirmek ~3 s
      surer. Modelde 1. mertebe tau = 1.0 s ile temsil edilir. Bunu
      modele koymamak asma (overshoot) ve titreme uretir; ayrica
      komut ivmesi kopterin YATMA ACISIDIR (atan(a/g)) ve yatma
      kamerayi cevirdigi icin dogrudan FOV meselesidir -- bu yuzden
      maliyette ayrica ivme (|u - w|) cezasi vardir.

Kritik ayrim (SAF TAKIP vs CARPISMA ROTASI): maliyette hem
"ex -> 0" (kadraj) hem "ATALETSEL LOS hizi -> 0" (paralel seyir) var.
Ataletsel LOS hizi, gorusteki aci hizi + yaw hizi toplamidir ve model
uzerinden SADELESIR:
        sigma_az = ex_dot + yaw_rate = -c_az * w2 + d_ex
yani ataletsel LOS hizini yalniz YANAL HIZ (w2) sifirlar, burun cevirmek
sifirlamaz. Maliyete bu terimi koymak MPC'yi otomatik olarak orantili
seyre (PN / carpisma rotasi) surer: yaw kadraji, yanal hiz gudumu yapar.
Yalniz "ex -> 0" konsaydi, optimizasyon en ucuz yol olan yaw'i secer ve
kuyruk takibine (18 m/s ile 20 m/s hedefi asla yakalayamayan) duserdik.

------------------------------------------------------------- GEOMETRI
senaryo.sh AIM=0 sabitler; o durumda sanal kadraj merkezi UFKA hizalidir:
        ey_deg = -(hedefin ufka gore yukselisi)      [aim=0]
        genel:  yukselis eps_deg = -(ey_deg + aim_deg)
Montaj acisi $YILDIZ_MOUNT'tan gelir (bkz. cevre_mount_deg); dikey
yari-FOV ~20.07 deg. Hedefin kadrajda kalmasi icin yukselisinin
(mount + govde_pitch) civarinda olmasi gerekir; ey ekseninde bandin
MERKEZI govde pitch'iyle KAYAR:
        ey_ref = -(mount_deg + pitch_deg)
Bu yuzden "ey -> 0" (es-irtifa) bir KADRAJ hedefi DEGILDIR; kadraj
hedefi ey -> ey_ref'tir ve FOV bandi bu merkez etrafindadir. Carpisma
icin gereken sey yukselisin SIFIRLANMASI degil, yukselis HIZININ
sifirlanmasidir (sabit LOS + kapanan menzil = carpisma).

MONTAJ 0 GECISI (2026-08-04 kullanici karari). Gercek donanimda
pitch-servo GIMBAL kullanilacak: olculdu ki ruzgar dahil SABIT
kameranin gereken montaj araligi 36.3 deg yayiliyor ve dikey yari-FOV
20.07 buna yetmiyor. Sim'de kopter neredeyse hic egilmedigi icin
0 deg sabit kamera ~ ideal gimbal emulasyonudur. Yeni geometri:
        mount 0, standoff back 25 / down 6  ->  LOS yukselisi
        eps0 = atan(6/25) = 13.50 deg
        beta_standoff = mount + pitch - eps = 0 - 2.5 - 13.5 = -16.0
        beta_carpma   = mount + pitch - 0   ~ -2.5 .. +25 (fren)
IKI ONEMLI ISARET DEGISIKLIGI:
  (1) Hedef artik eksenin ALTINDA degil USTUNDE basliyor (beta<0).
      +30 montajda kadraj cost'u hedefi merkeze almak icin ALCALMAK
      istiyordu (tur-2 tabana dalma regresyonu); mount 0'da ayni cost
      TIRMANMAK ister -- yani dikey kanal artik yerden UZAKLASARAK
      merkezliyor. Hedef-alti derinlik tavani bu yuzden nadiren
      baglayici; yine de emniyet artigi olarak duruyor.
  (2) Kadraj kaybi riski ALT kenardan UST kenara kaydi: yaklasma
      fazinda hedef ustte (beta -16, kenar -20.07), terminal fazda
      fren burnu kaldirinca alta kayiyor. Bantlar buna gore
      yeniden olculdu (fov_ust_bant / fov_alt_bant).

------------------------------------------------------------- COZUCU
Yogunlastirilmis (condensed) LTV-QP + FISTA (hizlandirilmis izdusumlu
gradyan). Neden cvxpy/OSQP degil: hedef donanim Raspberry Pi 5, dongu
20 Hz, butce ~15 ms; ayrica |v|<=hiz_tavani kisiti bir KURE'dir, QP degil
SOCP ister. Hareket bloklamasiyla (move blocking) karar degiskeni
7 BLOK x 4 GIRDI = 28 sayiya iner (bloklar=(1,1,2,2,3,4,7), toplam
n_adim=20; ilk iki adim tek tek serbest, son blok 7 adim sabit);
Hessian 28x28 olur.
SURE (2026-08-05 GERCEK KOSU olcumu, mpc_tani sure_ms):
  p50 7.3 ms | p95 13.4 ms | max 16.6 ms -- butce 13 ms, ASILIYOR (%13-16).
  Iterasyon TAVANINA carpma %46-70: yani cozum cogu dongude yakinsadigi
  icin degil BUTCE BITTIGI icin duruyor (sicak-baslatmaya YANLI ara nokta).
  (Eski "1.7 ms / p95 2.3 ms" iddiasi 18 m/s tavanli ve daha kucuk
  ufuklu bir yapilandirmadan kalmaydi; TO_TEST madde 7.)
Tavan yine de DETERMINISTIK (iterasyon tavani + duvar saati korumasi).
EKSIK METRIK: durma olcutu |dZ| < tolerans_mps'tir, yani "adim kucuvldu"
der, "optimumdayim" DEMEZ. Optimallik sertifikasi (sabit-nokta artigi)
loglanmiyor -- TO_TEST madde 7.
Girdi kisiti -- hiz kuresi {|v|<=hiz_tavani} KESISIM dusey hiz
dilimi CARPIM yaw kutusu -- KAPALI FORM Oklid
izdusumuyle TAM saglanir (kure-dilim kesisimine izdusum formulu
_izdusum_kure_dilim'de; scipy SLSQP ile 1e-5 m/s'e dogrulandi).
FOV kisiti IKI KATMANLIDIR: (a) SERT katman -- girdi kutusuna/dilimine
CBF ile yazilir ve izdusumle KESIN saglanir; (b) PLAN katmani -- ufuk
boyunca l1 TAM CEZA (Huber ile yumusatilmis), yatay kanalda etkin.
Hicbir katman "infeasible" donduremez: sert katman en iyi cabaya
duser, ceza katmani her zaman bir cevap verir.
Sayisal notlar (hepsi olculdu, mpc_test.py):
  * Adim boyu L, FOV cezasinin Hessian'ini SADECE ihlale yakin
    satirlardan kurar; hepsini katmak lambda_max'i ~145 kat sisirip
    FISTA'yi kullanilamaz yapiyordu.
  * lambda_prox ile proksimal duzenlileme kosul sayisini ~1.3e3'ten
    ~2.7e2'ye indirir ve plani dongu basina sinirlar (real-time
    iteration MPC).
  * Sicak baslatma: cozum bir blok kaydirilir; DEVIR aninda konumlunun
    son hiz komutu tohumdur. Soguk cozum ilk 2 dongude genis butceyle
    (25 ms) kosar, sonra 3-5 dongude tam optimuma oturur.
#BURADA KALDIM
------------------------------------------------------ VURUS FAZI
(2026-08-05) Kullanici istegi: "hedefi gordugu andan itibaren
birakmadan ustune hizlanan ve CARPAN" gudum. Iki kok neden olculdu:
  (1) HIZ PARITESI. MpcAyar tavani 18 m/s'te kalmisti, iskelet 35'e
      cikmisti; hedef 21.05 m/s -> saf kuyrukta kapanma -3 m/s ve
      7/7 iska. Tavan artik guidance_config'ten TURETILIYOR
      (cevre_hiz_tavani), yani bir daha ayrisamaz.
  (2) YAKIN MENZILDE MALIYET HALA "TAKIP" MALIYETIYDI. Kadraj
      korumasi (sert FOV kisiti + fren daraltmasi) uzun sureli takip
      icin dogru, son bir saniyede yanlis. VURUS fazi menzille
      SUREKLI bir karisim katsayisiyla (22 m'de 0, 8 m'de 1) FOV
      bantlarini fiziksel kenara acar, fren/hizlanma daraltmasini
      sonumler, kadraj ve alan odulu agirliklarini buyutur, ivme
      (yatma) cezasini kisar. Ayrintili gerekce MpcAyar vurus_*
      blogunda.
Kamera 0 deg SABIT oldugu icin kadrajdan cikan hedef bizi KOR
birakir; bu yuzden faz kadraji BIRAKMAZ, yalnizca merkezleme
talebini kenar talebine cevirir. Son <1 s'de bbox bayatlarsa cozucu
kosmaz ve son komut tekrarlanir (kor suzulme, bkz. _kor_komut).

-------------------------------------------------------- ISKA MODU
MPC'nin USTUNDE, maliyet fonksiyonunun DISINDA bir durum makinesi:
        KAPANMA -> TERMINAL -> VURUS -> ISKA -> (yetkiyi birak)
Neden maliyette degil: menzil acilirken bile hedefe dogru komut
uretmek maliyet acisindan DOGRUDUR (LOS hizi, alan odulu, kadraj --
hepsi hedefi gostermeye devam eder). "Gectim, birakmaliyim" bir
optimum degil bir SONLANDIRMA kararidir; optimizasyon onu bulamaz.
Gecis (pass) tespiti HEDEF TELEMETRISINE DOKUNMAZ: menzilin kendi
turevi (r_ic, LPF) VEYA bbox alaninin buyume hizinin isareti.
Ayrintili gerekce ve olcumler MpcAyar'in iska_* blogunda.
Referans tasarim: formation_KILLER._attacker_intercept_thread
(gercek donanimda ayarli ARM/OPEN/TERMINAL/TIMEOUT ikilileri).

KULLANIM
    python3 mpc_gudum.py                  # canli (yetki bekler)
    python3 mpc_gudum.py --no-iska        # ablasyon: eski davranis
    python3 mpc_test.py                   # cevrimdisi dogrulama + sure
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
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

KDEG = 180.0 / math.pi          # rad/s -> deg/s ve 1/m -> deg/(m/s)/m


def cevre_mount_deg(varsayilan: float = 0.0) -> float:
    """Kamera EKSENININ ufka gore yukselisi TEK KAYNAKTAN.

    GIMBAL DALI (2026-08-05): sim'de artik GERCEK, kendini stabilize eden
    tek eksenli gimbal var (uctusta olculdu: govde +-35 deg savrulurken
    kamera dunya pitch'i 0.65 deg icinde). Kamera ekseni = KOMUTLANAN tilt,
    o da standoff geometrisinden turetilir: $YILDIZ_TILT = atan(down/back)
    (scripts/standoff_geom.sh export eder). ONCELIK SIRASI:
      1. $YILDIZ_TILT  (fiziksel gimbal komutu -- yeni tek kaynak)
      2. $YILDIZ_MOUNT (dondurulmus govdeye-sabit kollar / eski kosular)
      3. varsayilan 0.0
    Kodda sabit yazmama dersi ayni: 2026-08-04'te montaj 30 -> 0
    gecisinde bayat sabit tam kadraj kaybina yol acmisti.
    """
    for anahtar in ('YILDIZ_TILT', 'YILDIZ_MOUNT'):
        deger = os.environ.get(anahtar)
        if deger is not None:
            try:
                return float(deger)
            except (TypeError, ValueError):
                pass
    return float(varsayilan)


def _cevre_sayi(anahtar: str, varsayilan: float) -> float:
    """Ortamdan sayi oku; bozuksa/yoksa varsayilana dus."""
    try:
        deger = os.environ.get(anahtar)
        return varsayilan if deger is None else float(deger)
    except (TypeError, ValueError):
        return varsayilan


def _bos_nan(x, bicim: str = '.3f') -> str:
    """NaN/None -> bos hucre, aksi halde bicimli sayi.

    NEDEN: tani CSV'sinde "mekanizma KAPALI" ile "deger 0" ayni kolonda
    okunuyorsa olcum yanlis cikar (0 s'lik bir tau ya da t_go fiziksel
    olarak imkansizdir, ama 0 yazilirsa ortalamayi bozar).
    """
    try:
        v = float(x)
    except (TypeError, ValueError):
        return ''
    return '' if not math.isfinite(v) else format(v, bicim)


def cevre_q_alan_carpani(varsayilan: float = 1.0) -> float:
    """YILDIZ_Q_ALAN_CARPANI -- bbox ALAN odulunun carpani (A/B dugmesi).

    TO_TEST madde 1: kapanma bir AMAC degil PRIM olarak kodlanmis; odulu
    buyutmek 22-45 m "yan yana ucma" tikanmasini kiriyor. Odul maliyete
    YALNIZ LINEER girdigi icin (bkz. _alan_odulu) Hessian, kosul sayisi ve
    cozucu suresi DEGISMEZ -- bedavaya gelen bir dugme.

    OFFLINE OLCUM (n=32/grup, gimbal fiziginde, capraz+yanal x duz+elips):
        carpan 1 (varsayilan): tikanan kapanma %25, zaman asimi %75
        carpan 4            : tikanan kapanma %75, zaman asimi %41
        carpan 4 + ufuk ref : tikanan kapanma %100, zaman asimi %0
      KUYRUK geometrisi hicbir kolda BOZULMADI (%67 sabit).
    VARSAYILAN 1.0 = eski davranis; A/B icin YILDIZ_Q_ALAN_CARPANI=4.
    """
    return _cevre_sayi('YILDIZ_Q_ALAN_CARPANI', varsayilan)


def cevre_ufuk_menzil_ref(varsayilan: float = 0.0) -> float:
    """YILDIZ_UFUK_MENZIL_REF -- MALIYET ufkunu menzille olcekler (0 = KAPALI).

    TO_TEST madde 3. KISIT ufku zaten menzille olcekleniyor (_cbf_sinirlari,
    T = cbf_ongoru_s * r / cbf_menzil_ref_m) ama MALIYET ufku SABIT 2.4 s
    kaliyordu -- olculdu: t_go ortancasi 0.61 s, yani VURUS'ta ufkun ~%73'u
    CARPISMA SONRASINDA. Asimetri buradaydi.

    NEDEN t_go DEGIL MENZIL: t_go ile olcekleme zaten denenmis ve capraz
    geometride ETKISIZ cikmis (kapanma ~0 -> t_go sonsuz -> olcekleme hic
    devreye girmiyor; bkz. _cbf_sinirlari icindeki not).

    Deger verilirse her adim suresi  adim_s * clip(r/ref, taban, 1)  olur.
    Olculen calisan bant: ref 45-60 m. 35 ve 20 ETKISIZ.
    """
    return _cevre_sayi('YILDIZ_UFUK_MENZIL_REF', varsayilan)


def cevre_dikey_terminal(varsayilan: float = 0.0) -> float:
    """YILDIZ_DIKEY_TERMINAL -- IRTIFA-AGNOSTIK terminal dikey hizalama.

    0 = KAPALI (VARSAYILAN, bugunku davranis birebir korunur), 1 = ACIK.

    NEDEN VAR (2026-08-07 olcumu, duz rota, uc dikey standoff):
        DOWN +4 (avci altta) : CPA 2.53 m, CPA'da DIKEY artik +0.92 m
        DOWN  0 (ayni irtifa): CPA 1.15 m, dikey artik -0.56 m
        DOWN -3 (avci ustte) : CPA 1.42 m, dikey artik +0.64 m
    DOWN +4 kosusunda dikey ayrimin menzil profili (|dz| ortancasi):
        45-35 m: 3.41   35-25 m: 3.58   25-15 m: 3.98   15-8 m: 2.34
    Yani standoff'un dikey bileseni 15 m'ye kadar HIC KAPANMIYOR; kapanma
    son 15 m'ye, yani ~1 saniyeye kaliyor.

    KOK NEDEN: maliyette dikey standoff'u KAPATAN tek terim q_ey idi ve o
    terim beta = ey - ey_ref uzerinde tanimli. FAZ C ile (gimbal tilt'i
    hedefin yukselisini IZLIYOR) ey_ref CANLI eksene baglandi
    (_kadraj_sabiti), yani ey_ref -> -eps. Boylece beta ~ 0 olur: kamera
    hedefi mukemmel cerceveler ve MALIYET DIKEY HATAYI ARTIK GORMEZ.
    Geriye kalan tek dikey terim, dikey LOS hizini sigma_el -> 0 suren
    PN terimidir; onun anlami tam olarak "PARALEL SEYIR = dikey standoff'u
    KORU"dur (bkz. _maliyet_satirlari). Olcum bunu dogruluyor: 25-35 m
    bandinda |dz| sabit ~3.5-4.0 m. Gimbal kadraj sorununu cozerken dikey
    kapanmayi suren sinyali de goturdu.

    MEKANIZMA (bu dugme): eski VURUS dikey hizalama biasinin (vurus_hiza_*,
    emekli) IRTIFA-AGNOSTIK ve ERKEN surumu. Dikey LOS hizi referansina
        sigma_el_ref = s * clip(olu_bant(eps) / tau, -tavan, +tavan)
    biasi konur; eps_dot = -sigma_el oldugu icin bu, eps'i (ve dolayisiyla
    dz = r*sin(eps)'i) SIFIRA suren bir taleptir. Eskisinden UC farki var:
      (1) IKI YANLI: eps<0 (avci ustte) iken de calisir -> irtifa-agnostik.
      (2) VURUS karisimina (22 m) degil TERMINAL MENZILE (45 m) baglidir:
          hizalama son saniyeye degil, ~3 s'lik pencereye yayilir.
      (3) bbox BAYATSA devre disi: eps donmus bir olcumden gelirse
          kendi kendini besleyen dikey talep olusur (bkz. vurus_kor_suzulme).
    EMNIYET: yalniz MALIYET REFERANSINA dokunur. Dikey hiz kutusu
    (tirmanma 9 / alcalma 4.5 m/s), irtifa tabani CBF'i ve derinlik tavani
    coz() icinde girdi kutusu olarak AYNEN durur; referans onlari DELEMEZ.
    """
    return _cevre_sayi('YILDIZ_DIKEY_TERMINAL', varsayilan)


def cevre_dikey_hata(varsayilan: float = 1.0) -> float:
    # VARSAYILAN ACIK (2026-08-10 kullanici karari): kanitli duzeltmeler
    # bayrak istemez -- ciplak `python3 mpc_gudum.py` sampiyon kurulumla
    # ucar. Eski tasarimi dogrulayan mpc_test beklentileri ayni gun
    # guncellendi (eski-davranis testleri bayragi acikca 0'a sabitler).
    #   KAPAMA (eski davranis): YILDIZ_DIKEY_HATA=0 YILDIZ_DIKEY_TGO=0
    """YILDIZ_DIKEY_HATA -- maliyette DOGRUDAN dikey hata (P) terimi.

    1 = ACIK (VARSAYILAN, 2026-08-10'dan beri), 0 = KAPALI (eski davranis).
    YILDIZ_DIKEY_TERMINAL'den BAGIMSIZ; ikisi birlikte de acilabilir.

    DEGER BIR CARPANDIR (q_dikey_hata uzerinde): 1 = nominal agirlik,
    2 = iki kat, 0.5 = yarim. NEDEN AYRI BIR ENV DEGIL: agirlik bu
    mekanizmanin TEK ayar dugmesi ve ilk sim olcumu (asagida) tam olarak
    agirligin YETERSIZ oldugunu gosterdi -- taramayi tek degiskenle
    yapabilmek icin dugmenin kendisi carpan olarak okunur.

    NEDEN VAR (KOK NEDENIN IKINCI YARISI). cevre_dikey_terminal()
    dokumante ediyor ki FAZ C sonrasi (gimbal tilt hedefin yukselisini
    IZLIYOR) beta = ey - ey_ref ~ 0 oldugu icin maliyet dikey hatayi
    ARTIK GORMUYOR. Onceki tur bu bosluga bir TUREV terimi koydu:
    dikey LOS HIZI referansina bias (sigma_el_ref). Olculdu (2026-08-07,
    duz+elips, DOWN +4):
        mekanizma calisiyor  : hiza_ref aktif kare %65-83,
                               terminal eps 7.0 -> 4.5 deg,
                               20-8 m bandinda |dz| 2.59 -> 1.98 m
        AMA CPA'daki dikey artik DUSMEDI (havuz ortancasi 0.99 -> 1.09)
    Klasik "P'siz D" imzasi: hata KAPANMAYA BASLIYOR ama TAM KAPANMIYOR.
    Turev terimi yalnizca hatanin DEGISIMINI cezalandirir; hatanin
    KENDISI icin maliyette hicbir satir kalmamistir.

    MEKANIZMA (bu dugme): maliyete, ey durumu uzerinde DOGRUDAN bir hata
    satiri eklenir. Geometri kesin (bkz. komut(): eps = -(ey + aim)):
        eps -> eps_hedef   <=>   ey -> ey_hedef = -(eps_hedef + aim)
    yani hedef "hatti" (eps = 0 duzlemi) MPC durumunda SABIT bir
    referanstir; ayri bir kestirim gerekmez.
      * REFERANS AYRISIMI (FAZ C KAZANIMI BOZULMAZ): kadraj terimi
        (q_ey) ve FOV/CBF kisitlari ey_ref = CANLI gimbal ekseninde
        KALIR -- kamera hedefi merkezlemeye devam eder. YALNIZCA bu yeni
        satir "hedef hatti" referansini kullanir. Iki referans ayni
        durum uzerinde ama FARKLI islerde: biri GORMEK, oteki VURMAK.
      * TERMINALDE SIFIRA RAMPA: agirlik menzille 45 -> 25 m arasi
        0'dan tam degere cikar. Uzakta terim YOK, yani standoff
        (DOWN parametresi) AYNEN korunur ve yaklasma profili degismez;
        terminalde hedef eps = 0, yani dikey standoff eritilir.
      * IKI YANLI: eps>0 (hedef ustumuzde) ve eps<0 (altimizda) simetrik
        -> DOWN -4..+4 hepsinde calisir, irtifa-agnostik.
      * OLU BANT: |eps| <= rahat iken satirin AGIRLIGI SIFIR (referans
        kaydirmasi degil). Boylece kucuk |eps|'te cozum KAPALI kol ile
        BIT-AYNI olur ve bbox gurultusu (+-1-2 px) dikey komuta cevrilmez.
      * BAYAT BBOX KAPISI: eps donmus bir olcumden geliyorsa terim kendi
        kendini besleyen dikey talep uretir (ayni tuzak vurus_kor_suzulme
        ve dikey_terminal'de olculdu) -> bayatta rampa 0.
    NEDEN eps (aci) UZERINDE, dz (metre) UZERINDE DEGIL: dz = r*sin(eps)
    durumda DOGRUSAL DEGIL (r de bir durum); aci satiri tek sutunlu ve
    Hessian'i degistirmiyor, yani cozum suresi AYNI kaliyor. Ustelik
    sabit acisal olu bant, menzil kapandikca METRIK toleransi kendiliginden
    daraltir: 1 deg = 25 m'de 0.44 m, 10 m'de 0.17 m -- kabul olcutumuz
    (CPA'da |dikey| < 0.4 m) tam da bu davranisi istiyor.
    EMNIYET: yalniz MALIYETE dokunur. Dikey hiz kutusu (tirmanma 9 /
    alcalma 4.5 m/s), irtifa tabani CBF'i ve derinlik tavani coz() icinde
    girdi kutusu olarak AYNEN durur; maliyet onlari DELEMEZ.
    """
    return _cevre_sayi('YILDIZ_DIKEY_HATA', varsayilan)


def cevre_dikey_tgo(varsayilan: float = 2.0) -> float:
    # YILDIZ_DIKEY_HATA ile BIRLIKTE acilir (kampanyada ikisi birlikte
    # olculdu); VARSAYILAN 2.0 (2026-08-10) -- sampiyon kurulum carpan=2
    # ile uctu ve dogrulandi, bkz. cevre_dikey_hata() basindaki karar.
    # Bu kol ASMAYI keser: sabit tau'da e_dot hic sifirlanmiyordu (olculen
    # -4.5..-5.1 m/s dikey hiz @CPA); t_go/k ile hem e hem e_dot sifira gider.
    """YILDIZ_DIKEY_TGO -- dikey hiz referansini KALAN ZAMANLA sekillendirir.

    VARSAYILAN 2.0 (ACIK, 2026-08-10'dan beri); 0 = KAPALI (eski davranis).
    DEGER BIR CARPANDIR (dikey_tgo_k uzerinde): 1 = nominal k, 1.2 = %20
    daha sert sonumleme. Tek ayar dugmesi k oldugu icin (tau_min bir
    emniyet tabanidir, ayar degil) duyarlilik taramasi TEK env ile
    yapilabilsin diye dugmenin kendisi carpandir -- YILDIZ_DIKEY_HATA
    ile ayni desen.

    NEDEN VAR (KOK NEDENIN UCUNCU VE SON PARCASI). Onceki iki tur
    olculdu (bkz. cevre_dikey_terminal / cevre_dikey_hata):
      * D tek basina (DIKEY_TERMINAL): hata kapanmaya BASLIYOR, TAM
        kapanmiyor -- "P'siz D".
      * P tek basina (DIKEY_HATA=1.5): hata ILK KEZ monoton kapaniyor
        (elips DOWN+4, |dz| ortancasi 25-35 m 3.83 -> 8-15 m 0.92 ->
        0-8 m 0.67; kapali kolda 3.29/2.07/1.51) AMA CPA artigi
        DUSMUYOR cunku SIFIRA BUYUK HIZLA VARIYOR: sifir gecisi
        CPA'dan 0.20 s once, dikey hiz @CPA -4.5..-5.1 m/s -> 0.9-1.2 m
        ASMA. Isaret de bunu soyluyor: kapali kol +2.02 (yetisemiyor),
        P kolu -1.10/-1.22 (gecip gidiyor).
      * P+D birlikte: COZMUYOR, cunku D'nin zaman sabiti
        dikey_terminal_tau_s = 1.5 s SABIT ve r < 15 m'deki t_go'dan
        (< 1 s) UZUN. Yani terminalde D "yavasla" der, P "kapat" der;
        sonuc ERKEN FREN -- elipste sifir gecisi HIC olmuyor, 0-8 m
        bandinda |dz| 2.20'de kaliyor.
    Ortak kok neden TEK: referans "hatayi sifirla" diyor, "hatayi VE
    HIZINI BIRLIKTE sifirla" demiyor. Sabit tau bir ZAMAN OLCEGI
    dayatir; carpisma probleminde tek anlamli zaman olcegi KALAN
    ZAMANDIR.

    MEKANIZMA: D kolunun zaman sabiti SABIT tau yerine t_go'ya baglanir
        tau_eff = clip(t_go / k, tau_min, tau_max)
    yani (kirpilmadigi bantta) referans
        sigma_el_ref = dikey_s * k * eps_fazla / t_go
    olur. Bu bir ZEM/PN yasasidir: "kalan zamana gore sonumle".

    NEDEN BU ASMAYI SIFIRLAR (kapali form, gerekcenin ozu). eps_dot =
    -sigma_el ve (yaklasik) sabit kapanmada t_go = T - t olsun:
        de/dt = -k e / (T - t)   =>   e(t) = e0 * ((T-t)/T)^k
    yani k > 1 icin HEM e HEM de e_dot = -k e0 (T-t)^(k-1) / T^k
    carpismada SIFIRA gider. Sabit tau'da ise e(t) = e0 exp(-t/tau):
    e kucuk olur ama e_dot ASLA sifirlanmaz -- olculen -4.5..-5.1 m/s
    dikey hiz @CPA tam olarak budur. Fiziksel dikey artik daha da iyi
    davranir: dz = r sin(eps) ve r ~ r0 (T-t)/T oldugundan
        dz ~ dz0 * ((T-t)/T)^(k+1),   d(dz)/dt ~ (k+1) dz / (T-t) -> 0.
    Yani k > 1 secmek "kritik sonum benzeri" terminal profildir; k
    buyudukce kapanma GEC ve SERT olur (hata uzun sure durur, sonra
    hizla erir), k -> 1'de asma geri gelir. Secilen k icin ve olculen
    diz noktasi icin bkz. MpcAyar.dikey_tgo_k.

    IKINCI VE ZORUNLU PARCA -- TAVAN. Yasa ancak talebi KIRPILMADIGI
    surece is gorur. Eski (sabit tau) kolun tavani SABIT bir acisal
    hizdir (8 dps) ve bu, menzille birlikte anlam degistirdigi icin
    yasayi tam da terminalde boguyordu: kirpilmis talep SABIT eps_dot
    demektir, sabit eps_dot ise ASMA'nin kendisidir. TGO kolunda tavan
    FIZIKSEL GIRDI KUTUSUDUR (tirmanma 9 / alcalma 4.5 m/s), yone gore
    secilir ve menzille tutarli sekilde aciya cevrilir. Olculdu: bu
    duzeltme olmadan k'yi buyutmek eps >= 8 deg'de HICBIR SEY
    degistirmiyordu (dikey hiz @CPA k=2.5 ile k=8 arasinda -3.35 m/s'de
    sabit). Bkz. _maliyet_satirlari icindeki tavan bloku.

    NEDEN P (DIKEY_HATA) ILE BIRLIKTE. Bu kol yalniz HIZ referansini
    sekillendirir; hatanin KENDISI icin maliyette hala satir yoktur
    (o satir P kolununkidir). P olmadan yasa yine "P'siz D"dir --
    sadece daha akilli bir D. Ana kol bu yuzden P + TGO'dur; TGO tek
    basina da acilabilir (ablasyon icin).

    t_go KAYNAGI (YENI KESTIRIM YOK): kontrolcunun ZATEN hesapladigi
    self.menzil_hizi (= d(r_ic)/dt, LPF tau 0.30 s; gecis kapanma-hizi
    sartinda da bu kullaniliyor) ve ic menzil r. t_go = r / (-menzil_hizi).
    GECERSIZ/BAYAT HALDE GUVENLI TARAFA DUSER: kapanma
    dikey_tgo_kapanma_min_mps'in altindaysa (acilma, capraz geometri,
    ilk 0.3 s'lik LPF isinmasi) t_go YOK sayilir ve tau_eff = tau_max
    alinir -- yani en YUMUSAK talep, eski sabit-tau davranisi. Agresif
    tarafa DUSULMEZ.
    EMNIYET: yalniz MALIYET REFERANSINA dokunur, dikey_terminal ile
    ayni satirdir. Dikey hiz kutusu (tirmanma 9 / alcalma 4.5 m/s),
    irtifa tabani CBF'i ve derinlik tavani coz() icinde girdi kutusu
    olarak AYNEN durur; referans onlari DELEMEZ. Bayat bbox kapisi da
    ayni (dikey_s uzerinden). Hessian BUYUMEZ: yeni satir yok, var
    olan satirin REFERANSI degisiyor -- cozucu suresi ayni.
    """
    return _cevre_sayi('YILDIZ_DIKEY_TGO', varsayilan)


def cevre_cozucu_bol(varsayilan: float = 0.0) -> float:
    """YILDIZ_COZUCU_BOL -- FISTA'ya BOL butce ver (SIM OLCUM KALITESI icin).

    0 = KAPALI (VARSAYILAN, bugunku degerler birebir: iterasyon_tavani 26,
    sure_butcesi_ms 13). 1 = ACIK -> 40 iterasyon, 18 ms.

    *** BU BIR PERFORMANS DUGMESI DEGIL, BIR OLCUM ARACI. GERCEK DONANIM
    (Pi 5) ICIN AYRI KARAR VERILIR -- bkz. asagidaki UYARI. ***

    NEDEN VAR (olculen kok neden). Tur-5 A/B tablosunda tani CSV'sindeki
    'butce_kesti' orani HER KOLDA (baseline dahil) %52-89 cikti. Yani
    FISTA karelerin cogunda tolerans_mps'e (2 cm/s) YAKINSAMADAN, ya
    iterasyon tavanina (26) ya da sure butcesine (13 ms) carparak
    donuyor. Bunun anlami: her karede verilen komut, o karenin QP'sinin
    OPTIMUMU DEGIL, optimuma giden yolda RASTGELE BIR NOKTA -- ve o
    nokta karenin ne kadar CPU aldigina (isletim sistemi zamanlamasi,
    Gazebo yuku, log yazimi) bagli. Dolayisiyla olculen HER A/B farkinin
    ustune, deneyle ILGISIZ, kosudan kosuya degisen bir gurultu biniyor.
    Bu gurultu somut olarak goruldu: elips DOWN+4'te k=2.0 kolunun AYNI
    konfigurasyonla iki kosusu 15-8 m bandinda |dz| 0.19 ve 2.78 m verdi
    -- yani KONFIGURASYON ICI yayilim, olcmeye calistigimiz HUCRELER
    ARASI farktan buyuk. n=1-2 ile k secilemeyisinin sebebi budur.

    MEKANIZMA: tavan 26 -> 40 (%54 daha fazla iterasyon), butce 13 -> 18
    ms. Baska HICBIR SEY degismez -- ayni maliyet, ayni kisitlar, ayni
    tolerans. Tek beklenen etki: cozum optimuma DAHA COK yaklasir ve
    "ne kadar yaklasti" sorusu makine yuku yerine PROBLEMIN KENDISINE
    baglanir (tolerans belirler, saat degil).

    HANGISI BAGLIYORDU -- OLCULDU (offline kapali dongu, ANA KOL P+TGO
    k=2, 24 senaryo = 4 rota x 3 devir x 2 tohum, N~2590 coz() cagrisi):
        NORMAL 26/13 : butce_kesti %64.5 | iterasyonun %67.5'i TAVANDA
                       iterasyon ort 22.9 p50 26 | sure p50 5.4 p95 7.8
                       p99 13.1 ms
        BOL    40/18 : butce_kesti %49.5 | iterasyonun %51.9'u TAVANDA
                       iterasyon ort 30.8 p50 40 | sure p50 6.1 p95 10.3
                       p99 13.1 ms
    Iki sey acikca gorunuyor:
      (1) BAGLAYAN SEY SURE BUTCESI DEGIL, ITERASYON TAVANIYDI. Normal
          kolda sure p95 = 7.8 ms iken butce 13 ms; yani kesilmelerin
          neredeyse tamami "26 iterasyon bitti" diye oluyordu. 13 -> 18
          ms tek basina hemen hicbir sey degistirmezdi; ISI YAPAN 26 -> 40.
      (2) YINE DE YAKINSAMIYOR: %49.5 hala yuksek. Yani tolerans_mps
          (2 cm/s) bu problem icin 40 iterasyonda da ulasilmiyor --
          sorun sadece butce degil, FISTA'nin bu olceklemedeki yakinsama
          HIZI. Butce buyutmek gurultuyu YARIYA INDIRIYOR, SIFIRLAMIYOR.
    Bedel: cozucu suresi p50 +0.7 ms, p95 +2.5 ms. p99 AYNI (13.1 ms),
    cunku p99'u belirleyen soguk baslangic cozumleri zaten genis butceli.

    DAVRANIS DEGISIYOR MU? EVET -- BU SAF BIR "OLCUM" DUGMESI DEGIL.
    Offline'da BOL kolu DAHA YAKIN kapatiyor (min menzil ortancasi 24
    senaryoda 7.84 -> 7.16 m; mpc_test 5-kapali-dongu ortancasi
    6.41 m). Beklenen yon: cozucu maliyetin ISTEDIGI seyi daha iyi
    gerceklestiriyor. Bedeli de olculdu: test_montaj_sifir'da DOGRU
    montajda kadraj kaybi %0.0 -> %9.2 (min menzil 11.9 -> 7.0 m), yani
    daha sert kapanma hedefi FOV kenarinda daha cok gezdiriyor. O test
    "yanlis montaj DOGRUNUN 3 KATI kadraj kaybettirmeli" diye bir ORAN
    olcutu kullandigi icin BOL=1 ile KALIYOR (%20.8 vs 3 x %9.2 = %27.6)
    -- ayrim MUTLAK olarak duruyor (yanlis montaj her iki eksende de
    kotu: %20.8 vs %9.2 kayip, 4.9 vs 7.0 m min menzil) ama oran
    olcutunun paydasi buyudugu icin esik tutmuyor. VARSAYILAN KAPALI
    oldugu icin depo kapisi (mpc_test 86/86) etkilenmez; BOL=1 ile
    kosulursa 85/86 beklenir ve sebebi BUDUR.

    *** SIM KAPALI DONGU HUKMU (2026-08-07, tur-5): DUGME MEKANIK OLARAK
    CALISIYOR AMA GURULTUYU KIRMIYOR -- VARSAYILAN KAPALI KALIR. ***
    Kosuldu: elips DOWN+4 ve duz DOWN+4, ANA KOL, SURE=300.
      MEKANIK (amaclanan etki, GERCEKLESTI):
        butce_kesti          %52-78 (n=4)   -> %43-54 (n=10)
        iterasyon tavaninda  %48-69         -> %29-41
        komut() p95 [ms]     13.2-13.4      -> 18.3-19.1
        dongu dt p50 [s]     0.050          -> 0.050   (20 Hz KORUNDU)
      Not: NORMAL kolda komut() p95 tam 13.2-13.4 ms, yani sim'de sure
      butcesi de BAGLIYORDU (offline'da baglamiyordu, cunku sim'de
      Gazebo yuku var). Iki mekanizma da hedeflendigi gibi gevsedi.
      OLCUM KALITESI (ASIL AMAC, GERCEKLESMEDI):
        * Kosu-ici yayilim DARALMADI, karisik oynadi: 15-8 m bandinda
          |dz| k=2 elips [0.19 .. 2.78] -> [0.88 .. 2.18] (daraldi) AMA
          CPA dikey artik AYNI hucrede [0.15 .. 0.50] -> [0.09 .. 1.59]
          (genisledi).
        * DAHA KOTUSU, ISARETI OLAN BIR KAYMA VAR: elips DOWN+4'te KOL
          FARK ETMEKSIZIN (k=2 ve k=3 birlikte) CPA dikey artik
          NORMAL n=4 : 0.15 0.37 0.50 0.64   (hepsi <= 0.64)
          BOL    n=6 : 0.09 0.13 1.39 1.50 1.59 1.74  (ortanca 1.45)
          Yani BOL=1, KABUL OLCUTUMUZU (CPA'da |dikey|) KOTULESTIRIYOR.
      YORUM (offline montaj bulgusuyla AYNI KOK): daha iyi yakinsayan
      cozucu MALIYETIN ISTEDIGINI daha sadik gerceklestiriyor -- ve
      maliyet, terminal dikey artigi TEK BASINA istemiyor; PN/kadraj
      terimleriyle bir uzlasma istiyor. Butceyi buyutmek o uzlasmaya
      daha hizli goturuyor, dolayisiyla "yakinsamamislik" bir GURULTU
      degil, kismen bizim LEHIMIZE calisan bir sapmaymis.
      HUKUM: gurultunun kaynagi cozucu butcesi DEGIL. butce_kesti'yi
      %62'den %46'ya indirmek kosu-ici yayilimi kapatmadi; demek ki
      yayilimin surucusu baska (hedefin rota fazi / devir geometrisi,
      bbox gurultusu, angajman sayisi). Dugme KAPALI kalir; A/B'ler
      NORMAL butcede kosulur ve n buyutulerek karar verilir.
      TEK MESRU KULLANIMI: "bu fark cozucu butcesinden mi geliyor?"
      sorusunu bir daha sormak gerekirse ablasyon anahtari olarak.

    NEDEN 18 ms SIM'DE GUVENLI: sim makinesinde gudum dongu butcesi
    50 ms (20 Hz; olculen Hz kosularda 19.8-19.9, yani dongu zaten
    doluyu doldurmuyor) ve olculen cozucu suresi p95 ~13-15 ms. 18 ms
    tavan, dongunun %36'si demektir; kalan 32 ms MAVLink + log + bbox
    icin fazlasiyla yeter. Yani sim'de bu dugme dongu hizini DUSURMEZ,
    yalnizca kesilme olayini nadirlestirmeyi hedefler.

    *** UYARI -- GERCEK DONANIM (Pi 5) BU DEGERI OTOMATIK ALMAZ. ***
    Ucus bilgisayarinda dongu butcesi ayni 50 ms olsa bile CPU cok daha
    yavas ve PAYLASILMIS (bbox cikarimi ayni kart uzerinde). Orada 18 ms
    tavan, kesilmeyi azaltmak yerine DONGUYU KACIRTABILIR -- ki dongu
    kacirmak, yakinsamamis bir komuttan cok daha kotudur (bkz.
    memory: "Guduem dongusu dt kok nedeni" -- 2 Hz'e dusen dongu dev
    daireler uretmisti). Donanim karari AYRI olculmelidir: Pi 5'te
    cozucu suresi p50/p95 ve dongu dt dagilimi olculmeden bu dugme
    ACILMAZ. Varsayilan bu yuzden KAPALI kalir ve dugme sim kosularinda
    ACIKCA env ile verilir.
    """
    return _cevre_sayi('YILDIZ_COZUCU_BOL', varsayilan)


def cevre_apn(varsayilan: float = 0.0) -> float:
    """YILDIZ_APN -- HEDEF YANAL IVMESINI kestir ve UFKA YAY (APN).

    0 = KAPALI (varsayilan, eski davranis BIT-AYNI). >0 = ACIK; deger
    ayni zamanda a_dik katkisinin carpanidir (1.0 = tam APN, 0.5 =
    yarim -- ablasyon icin).

    KOK NEDEN (olculdu, elips hedef, tur-8 havuzu). Bozucu kestirici
    TEK skaler d_ex uretiyor; fiziksel anlami KDEG*v_dik/r, yani
    "hedefin LOS'a dik HIZI". Ufka yayilirken v_dik SABIT varsayiliyor:
        d_ex_k = d_ex0 * r0/rbar_k  ==  KDEG*v_dik0/rbar_k
    Bu, DUZ ucan hedef icin dogru ve orada yasa saf PN gibi calisiyor
    (CPA 0.11-0.92 m, temas menzili). Hedef DONERKEN yanlis: v_dik
    sabit degil, |a_dik| ~ v^2/R kadar degisiyor. Olculen iska
    vektorunun %92-99'u "donusun disi" yonunde -- yani plan hedefi
    hep bir onceki teget uzerinde ariyor. CPA donuste 1.9-2.1 m.

    DUZELTME (klasik APN'in bu koordinatlardaki karsiligi): ikinci bir
    turev kestir (a_dik = d(v_dik)/dt) ve ufukta LINEER yay:
        d_ex_k = (v_dik0 + a_dik*t_k) * KDEG / rbar_k
    APN, sabit hedef ivmesi altinda PN'in optimal genellemesidir;
    burada yalnizca SERBEST CEVABA (Xf) girer -- Gam'a ve Hessian'a
    DOKUNMAZ, yani cozucu maliyeti ve suresi degismez.

    NICIN EMNIYETLI:
      * |a_dik| <= apn_a_tavani_mps2 (6 m/s^2) fiziksel kelepce.
      * Turev gurultulu; AYRI ve daha yavas LPF (apn_tau_s).
      * OLU BANT (apn_olu_bant_mps2): gurultu tabaninin altindaki
        kestirim SIFIRLANIR -> DUZ hedefte davranis degismez.
      * Bozucu guven rampasi (BozucuKestirici.guven) ile carpilir.
      * r < bozucu_dondur_menzil_m iken d_ex ile AYNI sekilde DONAR.
    """
    return _cevre_sayi('YILDIZ_APN', varsayilan)


def cevre_eyleyici(varsayilan: float = 1.0) -> float:
    """YILDIZ_EYLEYICI -- DOYUMLU (ivme sinirli) eyleyici modeli.

    >0 = ACIK (VARSAYILAN 1.0, 2026-08-10'dan beri). 0 = KAPALI (eski
    davranis BIT-AYNI). DIKKAT: tau_lin/a_max sabitleri iris'te OLCULDU;
    yeni aracta (hummingbird vb.) adim-cevap olcumu tekrarlanip
    YILDIZ_TAU_LIN / YILDIZ_A_MAX guncellenmeli.

    KOK NEDEN (2026-08-08 tur-2'de OLCULDU, uc bagimsiz kosuda ayni).
    MPC'nin eyleyici modeli tek satir:
        w <- w + al*(U - w),   al = h/(h + hiz_gecikme_tau_s),  tau = 1.00 s
    Bu BIRINCI MERTEBE ve IVMESI SINIRSIZDIR: hata ne kadar buyukse ivme o
    kadar buyuk istenir (a = |e|/tau; |e| = 15 m/s icin 15 m/s^2 talep).
    Gercek arac IVME SINIRLI. Olcum (e = cmd_v - vel, yatay;
    a_par = (dv/dt).e_birim; kosular taban8 / tyaw1 / tyawacc):
        PLATO (|e|>8 m/s, a_par p90) : 4.07 / 3.81 / 3.96 m/s^2
        dogrusal tau (|e|<4 m/s)     : 2.33 / 1.56 / 1.74 s
        tau_etkin (|e|=10-20 m/s)    : 5.47 / 6.75 / 6.58 s   <-- EN SIK BANT
    Yani model, calisilan bantta gercegin ~1/6'si kadar bir zaman sabiti
    varsayiyor: MPC ULASILAMAZ komut planliyor, sonra her dongu yeniden
    planlayip ayni ulasilamaz komutu tekrar veriyor.
    NOT: WPNAV_ACCEL 250->500 restorasyonu PLATOYU DEGISTIRMEDI (GUIDED hiz
    setpoint yolu PSC'den gecer, WPNAV_* waypoint kontrolcusunun parametresi);
    yani bu bir parametre isi degil, MODEL isi.

    DUZELTME (doyumun ARDISIK DOGRUSALLASTIRMASI). Gercek davranis
        w_dot = clip((u - w)/tau_lin, +-a_max)
    LTV cercevesine ADIM-BASI ETKIN TAU olarak oturur:
        tau_eff_k = max(tau_lin, |u_nom_k - w_nom_k| / a_max)
        al_k      = h_k / (h_k + tau_eff_k)
    Hata kucukken tau_lin (dogrusal bolge), buyukken |e|/a_max (doyum
    bolgesi) -- ikinci halde al*|e|/h = a_max, yani tam ivme tavani.
    Katsayilar ZATEN adim-basi (LTV) oldugu icin QP'nin YAPISI korunur;
    Hessian'in DEGERLERI degisir (kacinilmaz, cozucu suresi olculmeli).

    HER IKI EKSEN. Ilk surumde YALNIZ YATAY uygulanmis, dikey eski
    1.00 s'de birakilmisti ("ayrica olculmedi, muhafazakar"). BU BIR
    ARIZA URETTI: dikey yapay olarak UCUZ kaldi ve cozucu talebi oraya
    kaydirdi (|u3| p90 2.2 kat, ALTITUDE ABORT/angajman 0.36 -> 0.69).
    Dikey de olculdu (tau_lin_z 2.16 s, plato 5.25 m/s^2 -- yani dikey
    yataydan DAHA YAVAS) ve simetrik sekilde uygulaniyor. _cbf_sinirlari
    dusey dilimi de AYNI kazanci gorur (tau_v argumani), yoksa plan
    "yavasladim" derken kisit "hizliyim" varsayardi.

    Ayarlar env ile ezilebilir: YILDIZ_TAU_LIN (varsayilan 1.7 s),
    YILDIZ_A_MAX (varsayilan 4.0 m/s^2).
    """
    return _cevre_sayi('YILDIZ_EYLEYICI', varsayilan)


def cevre_ilerleme_saat(varsayilan: float = 0.0) -> float:
    """YILDIZ_ILERLEME_SAAT -- ISKA zaman asimini ILERLEMEYE bagla.

    0 = KAPALI (varsayilan, eski davranis BIT-AYNI). >0 = ACIK.

    KOK NEDEN (2026-08-08, olculdu). Zaman asimi DUVAR SAATIDIR:
        gecen = t - yetki_t0 > iska_zaman_asimi_s (8 s)  ->  ISKA
    Yani kapisi "ne kadar zamandir yetkideyim", oysa sormak istedigimiz
    "ILERLIYOR MUYUM". Iki olcum:
      * Tur-1 havuzu (taban8 + apn1 + apn1t, 34 angajman): 15 zaman
        asiminin 4'u (%27) arac HALA KAPANIRKEN kesildi -- son 2 s
        menzil egimleri -4.11 / -3.59 / -2.11 / -1.34 m/s, menziller
        8.8 / 21.6 / 13.6 / 25.6 m. Biri r=8.8 m'de -4.1 m/s ile
        kesilmis: bir saniye daha verilse temas menzilindeydi.
      * G kosusu (eyl1): IKI iskanin IKISI de zaman asimi ve ikisi de
        14-17 m'de KAPANIRKEN kesildi.
    Yani kapi, tam da ise yarayan angajmanlari da kesiyor.

    DUZELTME. Duvar saati yerine DURGUNLUK SAATI tut:
        ilerliyorsak : saat <- max(0, saat + dt*(1 - ilerleme_kazanci))
        durgunsak    : saat <- saat + dt
        ISKA         : saat > iska_zaman_asimi_s   (taban 8 s AYNEN KALIR)
    kazanc 1.5 ile ilerleyen angajmanda saat 0.5 s/s GERI sarar, yani
    "her ilerleme ani saati KISMEN tazeler" -- ama tazeleme ORANLIDIR,
    kare hizindan (20 Hz) bagimsizdir. Durgunluk tespiti degismedi:
    ilerleme yoksa saat aynen 1 s/s ilerler ve 8 s'de ates eder.

    ILERLEME = EN IYI MENZILIN (best-so-far) iyilesme hizi:
        (en_iyi_eski - en_iyi_simdi) / pencere > ilerleme_kapanma_esigi_mps
    Tasarim notundaki iki tanik ("kapanma hizi > esik" VEYA "son N s'de
    yeni minimum") TEK bir olcude birlesti, cunku ikisini ayri tutan
    her versiyon salinimla kandirildi -- gerekce ve iki olcum icin bkz.
    MpcAyar.ilerleme_kapanma_esigi_mps. best-so-far izi MONOTONDUR,
    yani olcu salinima YAPISI GEREGI bagisiktir ve "gercekten
    kapaniyorsan best-so-far iner" oldugu icin her iki tanigi da
    kapsar.

    MUTLAK TAVAN (sonsuz dongu YOK): gecen > ilerleme_tavan_s (22 s)
    ise ilerleme olsa bile ISKA. 22 s, 35 m/s'de ~770 m yol demektir;
    bunun otesi zaten yeniden konumlanma isidir.

    DOKUNULMAYANLAR: "menzil aciliyor" (45/30 ve gecis-onayli 30/8) ve
    "mutlak menzil" kapilari AYNEN kalir -- onlar saglikli ve zaten
    ilerlemenin DURDUGUNU degil TERSINE DONDUGUNU olcuyorlar.
    """
    return _cevre_sayi('YILDIZ_ILERLEME_SAAT', varsayilan)


def cevre_kor_pn(varsayilan: float = 1.0) -> float:
    """YILDIZ_KOR_PN -- KOR SUZULMEDE KOMUTU DEGIL *YASAYI* SURDUR.

    >0 = ACIK (VARSAYILAN 1.0, 2026-08-10'dan beri). 0 = KAPALI (eski
    davranis BIT-AYNI).

    KOK NEDEN (TO_TEST madde 4 + 2026-08-09 miss-vektoru olcumu).
    Kacan yarim metre YATAY eksende: kampanyanin 62 en-yakin gecisinde
    yatay p50 1.04 m vs |dikey| p50 0.73 m; <1.0 m alt kumesinde bile
    yatay 0.51 vs dikey 0.35. Terminalde ise kor kaliyoruz: son 5 m'de
    dongulerin ~%78'i bbox'siz ve O KORLUKTE KOMUT DONUYOR.
    Iki ayri korluk var:
      (1) ISKELET  (goruntulu_temel: bbox_yas > 0.7 s) -> MPC HIC cagrilmaz;
      (2) KONTROLCU (bayat_kisit_s = 0.30 s) -> cozucu kosar ama sert FOV
          kisiti birakilir; VURUS + r <= vurus_kor_menzil_m (8 m) kolunda
          ise _kor_komut SON KOMUTU AYNEN TEKRARLAR (cozucu HIC kosmaz).
    Sorun (2)'nin son adiminda: menzil kapanirken geometri degisiyor
    (c2 = KDEG/r buyuyor) ama yanal komut SABIT kaliyor. Yani "hedefi
    goremiyorum" ile "geometriyi bilmiyorum" karistiriliyor -- oysa
    MENZIL kanali bbox'tan BAGIMSIZ ve taze.

    BU KOL: bbox bayatken ex/ey'yi ICERIDEN OLU HESAPLA ilerlet ve
    cozucuyu KOSMAYA DEVAM ETTIR (komut tekrari yerine). Ilerletme,
    _nominal_yorunge'deki AYNI denklemlerdir:
        ex <- ex + dt*(-c2*w2 - yaw_hizi + d_ex),   c2 = KDEG/r
        ey <- ey + dt*(-c3*w3 + d_ey),              c3 = KDEG/r
    r, _menzil()'den gelir (model-ilerletmeli + menzil olcumu; bbox'a
    bagli DEGIL). Boylece kapanirken ayni yanal hiz daha buyuk aci hizi
    uretir ve yasa geometriyi izlemeye devam eder.

    *** YANLIS-YON RISKI VE RAYLAR (bu kolun tamami emniyet icindir) ***
    Son kestirim hataliysa olu hesap onu BUYUTUR. Bes ray:
      R1 SURE SINIRI  kor_pn_azami_s (1.0 s, YILDIZ_KOR_PN_AZAMI_S). Bu
         sure asilinca ilerletme BIRAKILIR ve bugunku donmus-komut
         davranisina DUSULUR. t_go terminalde zaten < 1.6 s.
      R2 d_ex SONUMLEMESI  korluk boyunca ilerletmede kullanilan bozucu
         exp(-t_kor / kor_pn_tau_s) ile SIFIRA soner (0.7 s,
         YILDIZ_KOR_PN_TAU). Gerekce: donmus d_ex hedefin O ANKI
         manevrasidir; 1 s sonra gecerliligi yoktur. Sifir = "hedef duz
         gidiyor varsay" = en az bilgi, en az zarar.
      R3 |ex| KELEPCESI  ilerletilen |ex|, son GERCEK olcumun |ex|'ini
         kor_pn_ex_pay_deg'den (5.0) fazla asamaz. Olu hesap hedefi
         kadraj disina "hayali" surukleyip agresif manevra tetiklemesin.
      R4 MENZIL TAZELIK KAPISI  olcum.menzil_m None ise ilerletme YOK --
         iki bagimsiz kor kaynagi (bbox + menzil) birlesmesin.
      R5 YAW  bugunku "son yaw'i surdur" davranisi (TUT_YAW, kanitli)
         korunur; bu kol yaw yasasina DOKUNMAZ.
    """
    return _cevre_sayi('YILDIZ_KOR_PN', varsayilan)


def cevre_hiz_tavani(varsayilan: float = 35.0) -> float:
    """Hiz kelepcesi TEK KAYNAKTAN: guidance_config.GORUNTULU_MAX_SPEED_MPS.

    NEDEN TEK KAYNAK: iskelet (goruntulu_temel.GoruntuluDongu) komutu ZATEN
    bu sayiyla kelepceliyor. MPC kendi tavanini daha DUSUK tutarsa, cozucu
    ulasabilecegi bir hizi hic istemez ve kelepceye HIC DEGMEZ -- olculdu:
    hedef_sonsuz kosusunda (mpc_sonsuz_20260805_022808) MpcAyar tavani 18,
    iskelet tavani 35 idi; kelepceye degme orani %0, hedef 21.05 m/s,
    kapanma -3 m/s, 7/7 ISKA "menzil aciliyor". Yani saf kuyruk takibinde
    yakalamanin onundeki TEK engel bu tek satirdi.

    Tavan bir KELEPCEDIR, TALEP DEGIL: maliyet ne isterse onu kullanir
    (bkz. guidance_config'teki ayni not). Yukseltmek agresiflik dayatmaz,
    kisiti kaldirir; agresifligi q_ivme ve VURUS fazi belirler.
    """
    try:
        import guidance_config as _cfg
        return float(getattr(_cfg, 'GORUNTULU_MAX_SPEED_MPS', varsayilan))
    except Exception:                       # cevrimdisi/kismi kurulum
        return float(varsayilan)


# ============================================================ AYARLAR

@dataclass
class MpcAyar:
    """Tum tunable'lar tek yerde; her birinin yaninda GEREKCE var."""

    # ---- ufuk / ayriklastirma -------------------------------------
    n_adim: int = 20
    # 20 x 0.18 = 3.6 s (UFUK AYNI, adim sayisi azaltildi). Gerekce: eyleyici IVME SINIRLI (5 m/s^2), yani
    # yanal hizi 15 m/s degistirmek 3 s aliyor. Ufuk bundan kisa olursa
    # MPC "sonra duzeltirim" diye planlar ve kacirir. Devir senaryosu
    # 25-60 m menzil, t_go tipik 3-10 s -> 3.6 s ufuk terminal fazin
    # buyuk kismini gorur. Daha uzunu bilgi katmiyor (bozucu d ufuk
    # boyunca sabit varsayiliyor), yalniz cozum suresini buyutuyor.
    adim_s: float = 0.12
    # SABIT ongoru adimi; dongu dt'sinden AYRIDIR. Ilk adim OLCULEN
    # dt'yi kullanir (komut o kadar sure basili kalacak), gerisi
    # nominal. Dongu 5 Hz'e duserse ufuk uzayip sacmalamaz.
    # 0.18 -> 0.12 (2026-08-05, 35 m/s turu). IKI GEREKCE, ikisi de
    # dogrudan hiz tavaninin fonksiyonu:
    #  (1) UFUK t_go'YU ASMAMALI. 20 x 0.18 = 3.6 s ufuk 18 m/s'de
    #      65 m yol demekti ve devir zarfi (<=60 m) ile ayni
    #      mertebedeydi. 35 m/s'de ayni ufuk 126 m -- yani MPC
    #      angajmanin BITTIGI noktadan cok otesini planliyor ve
    #      "sonra duzeltirim" diye bugunun komutunu yumusatiyor.
    #      20 x 0.12 = 2.4 s -> 84 m, zarfin biraz ustu (t_go tavani).
    #  (2) DOGRUSALLASTIRMA HATASI. LTV katsayilari c = KDEG/r adim
    #      basi DONDURULUR; 35 m/s'de 0.18 s'lik bir adim 6.3 m menzil
    #      degistirir, yani menzil TABANI (6 m) mertebesinde -- yakin
    #      menzilde c iki katina cikarken katsayi sabit varsayiliyordu.
    #      0.12 s'de adim basi degisim 4.2 m'ye iner.
    # Karar degiskeni sayisi (bloklar) DEGISMEDI -> cozucu suresi ayni.
    bloklar: tuple = (1, 1, 2, 2, 3, 4, 7)
    # Hareket bloklamasi (toplam = n_adim). Ilk iki adim serbest (o an
    # uygulanacak komut hassas olmali), sonrakiler kabalasir. 32 karar
    # degiskeni -> Pi 5'te milisaniyeler.

    # ---- kisitlar --------------------------------------------------
    hiz_tavani_mps: float = field(default_factory=cevre_hiz_tavani)
    # GORUNTULU_MAX_SPEED_MPS ile AYNI KAYNAK (bkz. cevre_hiz_tavani).
    # 18.0 SABIT YAZILMISTI ve iskelet 35'e cikarildiginda geride kaldi;
    # olculen bedeli: saf kuyruk takibinde kapanma -3 m/s, 7/7 iska.
    # ARTIK TURETILIYOR -- iki tavan bir daha ayrisamaz.
    # DIKKAT (35 m/s'in DIKEY sinirlari): yatay tavan buyudu ama
    # WPNAV_SPEED_UP/_DN buyumedi. LOS 13.5 deg yukselisteyken 35 m/s
    # kapanma 35*sin(13.5) = 8.2 m/s TIRMANMA ister; tirmanma tavani
    # 9.0. Yani ~15 deg'den dik LOS'larda kapanma hizini artik yatay
    # tavan degil DIKEY tavan sinirlar. Standoff down 4-6 m icin
    # (eps 9-13.5 deg) pay var; daha dik geometri gorulurse
    # tirmanma_tavani WPNAV_SPEED_UP ile birlikte artirilmalidir.
    # DEVIR (HANDOVER) IVME RAMPASI -- DENENDI, SIM'DE ELENDI, GERI ALINDI.
    # (2026-08-05, tur-2 sim kosusu.) Devirde q_ivme'yi 3x agirlastirip
    # tau=2 s ile sonumleyen bir rampa eklenmisti; niyet devir
    # transiyentindeki burun-asagi dalisi (ilk 3 s pitch min -20.5 deg)
    # yumusatmakti. SIM OLCTU: mekanik calisti (ivme_carp devirde 3.0,
    # ~2 s'de 1.0) AMA (1) devir transiyentini DUZELTMEDI (taze-degil
    # dongu 261 -> 269, pitch min hala -34 deg) ve (2) KAPANMAYI YARIYA
    # DUSURDU (menzil_hizi ort -3.11 -> -1.40 m/s, guclu kapanma
    # <= -10 m/s orani %7.0 -> %2.9). Net zararli -> GERI ALINDI.
    # Devir transiyentinin dogru cozumu ivme cezasi degil, terminal
    # DIKEY HIZALAMA (vurus_hiza_*) cikti (bkz. _maliyet_satirlari dikey
    # referans biasi): duz kamera = az govde hareketi = az dalis.
    #
    # HIZ-ARTIS KELEPCESI -- ayni turda DENENDI, OLCUMLE ELENDI.
    # |u_k| <= min(tavan, max(|w_k|, taban) + a_ileri*tau) bicimli sert
    # bir kure-yaricapi kelepcesi; kapali form korunuyordu (blok basi
    # yaricap, ek maliyet yok). Pitch'i DUZELTMEDI, kapanmayi oldurdu:
    #     a_ileri tavani   pitch hizi   pitch min   min menzil
    #        kapali           3.81        -24.9       1.93
    #        2.0 m/s^2        4.43        -22.8      11.17
    # Sebep: ArduPilot hiz dongusu setpoint farkini ~0.25 s'lik zaman
    # sabitiyle kapatir; 1.25 m/s'lik bir fark bile ivmeyi 5 m/s^2'ye
    # DOYURUR. Komutun BUYUKLUGUNU kelepcelemek bu farki kucultmez;
    # kucultan sey (u - w) uzerindeki CEZADIR. Mekanizma kodda kaldi
    # (blok-basi v_tav dizisi) ama VARSAYILAN KAPALI.
    ileri_ivme_tavani_mps2: float = 0.0
    hiz_artis_taban_mps: float = 6.0
    tirmanma_tavani_mps: float = 9.0    # WPNAV_SPEED_UP=10 m/s, %10 pay
    alcalma_tavani_mps: float = 4.5     # WPNAV_SPEED_DN=5 m/s, %10 pay
    yaw_hiz_tavani_dps: float = 90.0
    # 50 deg/s IDI ve wanderer kosusunda DOYDU: mpc_tani_20260803_182755
    # loglarinda donguletin %9.4'unde |yaw|=50 tavana yapisik, |ex|>33
    # (yatay FOV disi) %3.9. Elips'te doyum %0.3, |ex|>33 %0.3. Yani
    # wanderer bozulmasinin yarisi dogrudan YAW DOYUMU. ATC_SLEW_YAW
    # 180 deg/s'e izin veriyor; 90 deg/s ile 33 deg'lik yari-FOV
    # 0.37 s'de taraniyor, govde salinimi hala makul.
    ex_siniri_deg: float = 26.0
    # Yatay yari-FOV 33 deg; 7 deg pay govde yalpasi + tespit kenar
    # kaybi icin. Bu artik SERT sinirdir (CBF ile yaw kutusuna
    # cevrilir), yumusak ceza degil.
    fov_alt_bant_deg: float = 14.0      # eksenin ALTINDA izin verilen
    fov_taban_bant_deg: float = 4.0     # fren daraltmasinin alt siniri
    fov_ust_bant_deg: float = 17.5      # eksenin USTUNDE izin verilen
    fov_ust_taban_bant_deg: float = 6.0
    # UST BANDIN daraltma tabani (fov_taban_bant_deg'in ust kenardaki
    # esi). 2026-08-05, 35 m/s turu: alt bant FRENLE daraliyordu
    # (fren -> burun yukari -> hedef alt kenara), ust bant ise SABITTI.
    # 18 m/s'de bu bir eksiklik degildi cunku ileri IVMELENME kucuktu;
    # 35 m/s tavaninda angajmanin ilk saniyeleri SUREKLI ileri
    # ivmelenmedir ve her 1 m/s^2 ileri ivme burnu KDEG/g = 5.84 deg
    # ASAGI egerek sabit kamerayi asagi cevirir -- mount 0'da hedef
    # zaten eksenin USTUNDE (beta ~ -16) oldugu icin bu dogrudan UST
    # kenara iter. Simetrik daraltma bu yuzden eklendi (bkz.
    # _cbf_sinirlari'ndaki 'hizlanma' terimi).
    # Dikey yari-FOV ~20.07 deg (atan(tan(33)*720/1280)); iki bant da
    # bu KENARIN icinde pay birakmali.
    # TARIHCE (+30 MONTAJ): kayiplar HEP ALTTAN geliyordu (tirmanis ->
    # burun-yukari -> sabit kamera yukari bakar -> zaten altta olan
    # hedef busbutun cikar), o yuzden bant asimetrik ve ALT dardi
    # (11/16).
    # MONTAJ 0 (2026-08-04): isaret TERSINE dondu. Standoff'ta
    # beta = mount + pitch - eps = 0 - 2.5 - 13.5 = -16.0, yani hedef
    # eksenin 16 deg USTUNDE ve UST kenara yalnizca 4 deg var; alt
    # kenara ise 36 deg.
    # OLCUM ONCE: bantlar 6 senaryoluk kapali dongu taramasinda
    # BASKIN DEGISKEN CIKMADI -- ust bant 15/16/17.5/19 icin kadraj
    # kaybi %1.74-2.03, alt bant 11/14/17 icin fark yok (alt kenar
    # bu geometride zaten hic baglayici degil). Yani asagidaki secim
    # bir optimum degil, GEOMETRIK PAY mantigidir:
    #   * ust bant 16.0 -> 17.5: 16.0 devir aninin TAM uzerine
    #     dusuyor, yani kisit daha t=0'da baglayici oluyordu (kotu
    #     degil ama gereksiz). 17.5 devir noktasina 1.5 deg,
    #     fiziksel kenara 2.6 deg pay birakir.
    #   * alt bant 11.0 -> 14.0: alt kenar artik BOL; dar tutmanin
    #     tek etkisi terminal frende gereksiz tirmanma talebi olurdu.
    #     Bant yalniz endgame'de (eps -> 0, fren burnu kaldirir)
    #     baglayici olur.
    # ASIL LEVYE fov_tirmanma_talep_tavani cikti (bkz. asagisi):
    # 3.0 -> 5.0 kadraj kaybini %2.37'den %1.75'e indirdi.
    mount_pitch_deg: float = field(default_factory=cevre_mount_deg)
    # $YILDIZ_MOUNT (standoff_geom.sh export eder), fallback 0.0.
    # ARTIK SABIT YAZILMIYOR: sim montaji 30 -> 0'a gecirildiginde bu
    # tek satir kadraj kaybinin tamamini aciklamisti. --mount ile
    # ezilir (ablasyon / gercek donanim gimbal aci komutu).
    aim_deg: float = 0.0                # senaryo.sh AIM=0 sabitliyor

    pitch_baglasimi: bool = False
    # GOVDE PITCH BAGLASIMI (gimbal anahtari).
    # ACIK  : kamera ekseni govdeye SABIT -> eksen = mount + govde
    #         pitch. Tirmanma (3.2 deg/(m/s)) ve fren (5.84 deg/(m/s^2))
    #         ekseni kaydirir; MODEL bunu ongorur (bkz.
    #         pitch_tirmanma_kats ve _cbf_sinirlari'ndaki fren
    #         daraltmasi).
    # KAPALI: kamera ekseni GOVDEDEN BAGIMSIZ stabilize (pitch-servo
    #         gimbal). ey_ref = -(mount + aim), tirmanma/fren terimleri
    #         DUSER; dikey kisitin girdi duyarliligi geriye kalan SAF
    #         GEOMETRIK kanaldan (alcalirsan hedef gorunur yukselir)
    #         gelir -- bu yuzden anahtar kapatilinca kisit KORLESMEZ.
    # VARSAYILAN KAPALI (gimbal dali, 2026-08-05): sim'de artik GERCEK
    # stabilize gimbal var -- ucusta olculdu: govde +-35 deg savrulurken
    # kamera dunya pitch'i maks 0.65 deg. Govde pitch'i goruntuye
    # YANSIMIYOR; baglasimi acik tutmak modele VAR OLMAYAN bir kaymayi
    # ongortur (cift telafi). Eski govdeye-sabit davranis icin
    # --pitch-baglasimi ile acilir (mpc_test 5g iki yonu de olcer).

    # --- DIKEY: HEDEF-ALTI DERINLIK TAVANI (2026-08-04 tur-2) ---
    standoff_derinlik_m: float = 6.0
    derinlik_tavani_m: float = 15.0
    derinlik_yaklasma_s: float = 2.0
    # KOK SORUN (test pilotu tur-2): dikey kanal tabana yasliyordu.
    # Sebep: kadraj cost'unun referansi KAMERA EKSENININ MERKEZI
    # (ey_ref = -(mount+pitch)). +30 montajda hedef ekseninin ALTINDA
    # oturdugu icin (ozet.txt: -9.8 deg), hedefi merkeze getirmenin
    # tek yolu ALCALIP gorunen yukselisini artirmak. Uzun menzilde bu
    # geometrik olarak imkansiz (r=190 m'de hedefi eksene almak icin
    # 95 m alcalmak gerekir) -> kontrolcu tabana kadar dalar.
    # COZUM (TEK YANLI): kadraj cost'unu DEGISTIRMEDIM (merkeze cekme
    # yakalanan/kapanan kosularda dogru davraniyor; onu bozan bir
    # referans kaydirmasi elips'te MAKSIMUM alcalmayi kotulestirdi).
    # Bunun yerine ALCALMAYA bir tavan koyuyorum: hedefin altinda en
    # fazla derinlik_tavani_m kadar in. Hedef-alti derinlik =
    # r*sin(eps); bu tavan dikey hiz DILIMININ ust sinirina yazilir
    # (irtifa tabaniyla ayni makine, izdusum kapali formda kalir).
    # TEK YANLIDIR: yalniz alcalmayi keser, tirmanmayi ASLA zorlamaz
    # -> kapanan kosulari bozamaz. Menzil kapandikca r*sin(eps)
    # kuculur, tavan acilir, terminal carpisma serbest kalir.
    # MONTAJ 0 GUNCELLEMESI: tasarim standoff derinligi down = 6 m
    # (back 25 / down 6), yani 13 -> 6. Tavan ayni "tasarim + 7 m pay"
    # mantigiyla 20 -> 15 (13 degil): devir kapisi ayni LOS acisinda
    # 60 m menzile kadar acik ve 55 m'de derinlik 55*sin(13.5) = 12.8 m
    # oluyor; 13'luk bir tavan devrin HEMEN ardindan alcalmayi
    # kilitlerdi. 15 m devir geometrisini gecirir, 2x asiri-dalmayi
    # (>= 12 m tasarim ustu) keser.
    # ONEMLI (mount 0): bu tavan artik NADIREN baglayicidir. mount 0'da
    # hedef eksenin USTUNDE oldugu icin kadraj cost'u TIRMANMA ister,
    # yani tur-2'deki "merkezlemek icin dal" mekanizmasi kokten
    # yoktur. Tavan bilincli olarak EMNIYET ARTIGI olarak birakildi
    # (gimbal aim'i degisirse ya da aim_deg elle verilirse mekanizma
    # geri gelebilir).
    dikey_derinlik_tavani: bool = True

    pitch_tirmanma_kats: float = 3.2
    pitch_kats_tavan_dps: float = 25.0
    # TIRMANMA -> PITCH ESLEMESI. LOS ajaninin olcumu:
    #     pitch_deg ~ -1.8 + 3.2 * tirmanma_hizi [m/s]
    # Bu, +30 montajda kadraj kaybinin kok mekanizmasiydi: 5 m/s
    # tirmanma burnu +16 deg kaldirir, kamera ekseni (mount+pitch)
    # 46 deg'e cikardi ve zaten 22 deg yukseliste olan hedef dikey
    # FOV'un (yari 20 deg) ALT kenarindan cikardi. MOUNT 0'da isaret
    # tersine doner: hedef eksenin USTUNDE oldugu icin (beta<0)
    # tirmanmak hedefi merkeze YAKLASTIRIR, tehlike terminal fazda
    # (beta ~ 0 iken fren burnu kaldirinca) ALT kenara kayar.
    # Terim her iki halde de ayni isareti tasir, yalniz hangi kenarin
    # baglayici oldugu degisir. MODEL BUNU ONGORMEK ZORUNDA: FOV kisiti
    #     beta_k = ey_k - kats * vz_k + C
    # olarak kurulur (vz NED, negatif=tirmanma), yani "tirmanirsan alt
    # payini kendin yersin" kisitin ICINDE. C olculen pitch'e
    # demirlenir (mutlak fite degil), boylece fit hatasi DC olarak
    # kalir. pitch_kats_tavan: eslemenin dogrusal kabul edildigi ust
    # sinir (bunun otesinde pitch doyar).
    # GIMBAL NOTU: bu katsayi pitch_baglasimi=False iken SIFIRLANIR
    # (gimbal ekseni govdeden ayirir). O durumda dikey kisit
    # korlesmesin diye girdi duyarliligina saf geometrik kanal
    # (kats_geom = T*c3/cos(eps), bkz. _cbf_sinirlari) her iki modda
    # da eklenir -- mount 0'da bu terim zaten kats'in ~%40'i
    # buyuklugundedir, yani anahtar kapandiginda kisit zayiflar ama
    # KAYBOLMAZ.

    # ---- maliyet agirliklari (normalize; birimsiz) -----------------
    # Her terim once bir OLCEGE bolunur, sonra agirlikla carpilir.
    olcek_ex_deg: float = 10.0
    olcek_ey_deg: float = 10.0
    olcek_menzil_m: float = 50.0
    olcek_los_hiz_dps: float = 10.0
    olcek_hiz_mps: float = 18.0
    # DIKKAT: bu HIZ TAVANI DEGIL, sabit bir NORMALIZASYON olcegidir ve
    # tavan 18 -> 35 turunda BILINCLI OLARAK 18'de birakildi. Onunla
    # bolunen uc terim (r_hiz seviye cezasi, r_delta_hiz fark cezasi,
    # lambda_prox) MUTLAK eyleyici puruzsuzlugunu ifade eder: eyleyici
    # ivme siniri (WPNAV_ACCEL 5 m/s^2) hiz tavanindan BAGIMSIZDIR,
    # yani "blok basina kac m/s degisebilirim" sorusunun cevabi tavanla
    # buyumez. Olcegi 35'e tasimak bu uc agirligi (18/35)^2 = 0.26 ile
    # carpar; lambda_prox'un kosul sayisini 1.3e3 -> 2.7e2 indiren
    # etkisi de ayni oranda zayiflar (cozucu suresi buyur). Olcek
    # birim tasiyicidir, kelepce degil.
    olcek_yaw_dps: float = 50.0

    q_los_hiz: float = 4.0
    # ATALETSEL LOS hizi -> 0. Carpisma rotasini kuran ANA terim (PN).
    # En buyuk agirlik burada olmali; kadraj terimleri onu ezmemeli.
    q_ex: float = 0.5      # kadraj (yatay) -- esasen yaw ile odenir
    q_ey: float = 0.60     # kadraj (dikey) -- u3 ile odenir, LOS hiziyla
                           # yarisir; 18 m/s'de LOS'un ~1/10'u yetiyordu.
    # 0.35 -> 0.60 (2026-08-05, 35 m/s turu). GEREKCE OLCULDU: yuksek
    # tavanda kadraj kayiplarinin BASKIN kanali DIKEY oldu. Saf kuyruk
    # takibinde kayip aninda |ex| max 1.1 deg (yatayda sorun YOK) --
    # yani hedef alt/ust kenardan cikiyor. Mekanizma geometrik:
    # standoff hedefin ALTINDA (down 4-6 m) ve eps = asin(down/r)
    # menzil kapandikca buyur; 35 m/s'de kapanma 2-5 kat hizli oldugu
    # icin dikey kanalin duzeltme suresi ayni oranda kisaldi.
    # Kapali dongu paneli (12 senaryo): q_ey 0.35 -> 0.60 kadraj kaybi
    # %26.7 -> %23.4, min menzil ORTANCASI 9.63 -> 8.51 m. Yani bu
    # ayar KADRAJI ve YAKALAMAYI AYNI ANDA iyilestirdi -- alisildik
    # takas burada yok, cunku kaybedilen kadraj zaten yakalamayi
    # kesiyordu. 0.9 da olculdu (kayip %21.3, ortanca 8.23) ama
    # ortalama min menzil kotulesiyor (11.90 -> 12.41): dikey kanal
    # LOS hizi terimini ezmeye basliyor. 0.60 iki olcutu birden
    # iyilestiren en buyuk deger.

    # ---- ODUL: LINEER BBOX ALANI (karekok DEGIL) -------------------
    q_alan: float = 3.0
    q_alan_hizi: float = 1.5
    # A/B DUGMESI (TO_TEST madde 1): YILDIZ_Q_ALAN_CARPANI ikisini birden
    # olcekler. Varsayilan 1.0 -> davranis DEGISMEZ. Gerekce/olcum:
    # cevre_q_alan_carpani docstring'i.
    q_alan_carpani: float = field(default_factory=cevre_q_alan_carpani)
    # A/B DUGMESI (TO_TEST madde 3): maliyet ufkunu menzille olcekle.
    # 0.0 = KAPALI (varsayilan, eski davranis). Bkz. cevre_ufuk_menzil_ref.
    ufuk_menzil_ref_m: float = field(default_factory=cevre_ufuk_menzil_ref)
    ufuk_adim_taban_s: float = 0.05     # olcekleme sonrasi adim alt siniri
    q_menzil: float = 0.25
    # Odul artik dogrudan bbox ALANI (w*h, px^2) ve onun BIRINCI
    # TUREVI (buyume hizi = yaklasma hizinin vekili). Alan ~ K/r^2
    # oldugu icin:
    #     a_k = A(rbar_k)/A_0 = (r_0/rbar_k)^2         (bagil alan)
    #     da_k/dt = 2 a_k w1_k / rbar_k                (buyume hizi)
    # Ikisi de rbar etrafinda dogrusallastirilir -> maliyete YALNIZ
    # LINEER terim girer, Hessian degismez, cozum suresi artmaz.
    # NEDEN KAREKOK/MENZIL-KARESI DEGIL: q_menzil*(r/50)^2 teriminin
    # kapanma tesviki r ile ORANTILIDIR, yani UZAKTA guclu yakinda
    # zayif -- carpismanin tam tersi. Lineer alanin tesviki 1/r^3 ile
    # buyur: 40 m'de 1 birimse 20 m'de 8 birim. Terminal agresiflik
    # bilincli olarak artar; onu dengeleyen sey artik SERT FOV
    # kisitidir (bkz. cbf_gamma). q_menzil kucuk bir kalintidir:
    # tek isi Hessian'a menzil kanalinda egrilik vermek (sayisal
    # kosullama), gudum tesviki artik alandan gelir.
    p_carpani: float = 3.0  # terminal adimda tum durum agirliklari x3

    r_hiz: float = 0.02     # girdi seviyesi cezasi (yalniz yanal/dikey ve
                            # yaw'a; ILERI kapanmaya ceza YOK, hizli
                            # kapanmak amacimiz)
    r_yaw: float = 0.06

    q_ivme: float = 0.80
    olcek_ivme_mps: float = 6.0
    # IVME (yatma acisi) CEZASI -- bu ortamda hayati. Komut ile o anki
    # hizin farki (u - w) dogrudan istenen ivmedir; otopilot bunu
    # WPNAV_ACCEL=5 m/s^2 ile karsilar ve kopter atan(a/g) kadar YATAR.
    # Sert frenleme burnu 25 deg YUKARI kaldirir; +30 montajda kamera
    # ekseni 55 dereceye cikip hedefi dikey FOV'un (yari 20 deg)
    # DISINDA birakiyordu. MOUNT 0'da ayni 25 derece bu kez hedefi ALT
    # kenara iter (eksen 25 dereceye cikar, hedef ~0 yukseliste kalir)
    # -- yani ceza aynen gerekli, kaybedilen kenar degisti.
    # Benzetimde olculdu: bu ceza yokken hedef ilk saniyede
    # kayboluyordu. Ikinci faydasi sayisal: u1 (kapanma)
    # kanalinin Hessian'da hic egriligi yoktu, bu terim onu verdigi
    # icin kosul sayisi ~5 kat duser.
    # AYAR NOTU: bu, "ne kadar agresif manevra" ile "kadraji koru"
    # arasindaki ANA dugmedir. 0.45 -> daha agresif, dar virajda 1-2 m
    # daha yakin ama gecici kadraj kayiplari artiyor; 1.5 -> kadraj
    # cok saglam ama kuyruk takibinde yakinsama yavasliyor. Sim
    # loglarindaki (pitch, tespit surekliligi) ikilisine bakarak
    # yeniden ayarlanmali.
    r_delta_hiz: float = 0.55
    r_delta_yaw: float = 10.0
    r_delta_yaw_serbest: float = 1.0
    yaw_serbest_vperp_alt: float = 7.0
    yaw_serbest_vperp_ust: float = 18.0
    yaw_agirlik_tau_s: float = 0.6
    # Komut degisim cezasi (sunumdaki r1). Yuksek tutuldu: iskeletin
    # LPF'si zaten yumusatiyor ama MPC'nin cozumu adimdan adima
    # ziplamamali, yoksa LPF surekli pesinden kosar (faz kaybi).
    # r_delta_yaw 0.30 -> 10.0 (2026-08-04 tur-3 chatter analizi):
    # etkin agirlik 0.30/50^2 = 1.2e-4 (deg/s)^-2 idi; ex takip terimi
    # 5e-3 deg^-2 ve yaw duyarliligi ~0.5 oldugu icin hareket
    # bastirma PRATIKTE YOKTU (3-4 kat buyukluk kucuk). Tarama
    # (0.3/3/10/30) ile 10 secildi: |dYaw| adim rms 2.6-3.7 -> 1.4
    # dps (yaklasik 2.5x), yakalama degismedi. 30'da terminal
    # cevikligi bozuluyor.
    #
    # TUR-4 REGRESYONU VE COZUMU (2026-08-04): SABIT 10.0 chatter'i
    # hedefin cok altina indirdi (0.44-0.91 dps, hedef 2.5) AMA TEPE
    # YAW YETKISINI oldurdu: |yaw| maks 90 -> 53-61 dps, >80 dps
    # orani %1 -> %0. Wanderer'in keskin manevralarinda gereken kisa
    # sureli yuksek yaw ataklari uretilemedi -> |ex| p90 14.3 -> 29,
    # alan -%65, tespit %94 -> %79. Elips (ongorulebilir rota) ayni
    # sonumlemeden ETKILENMEDI, hatta iyilesti -- yani sorun sabit
    # bir katsayinin iki farkli rejime ayni cevabi vermesiydi.
    #
    # COZUM: KAZANC PROGRAMLAMA -- ama DOGRU DEGISKEN UZERINDE.
    #
    # ILK DENEME (basarisiz, olcumle elendi): olcut max(|ex|, |d_ex|)
    # idi. Iki ayri kusuru vardi ve ikisi de kapali dongude olculdu
    # (8 senaryo paneli: keskin/viraj/wanderer/elips/duz):
    #  (1) |ex| KONTROL EDILEN degiskendir. Onun uzerinden yetki acmak
    #      POZITIF GERI BESLEME kurar: hata buyur -> yetki acilir ->
    #      yaw asar -> hata ters isaretle buyur -> yetki yine acilir.
    #      Kuyruk takibinde limit cevrimi urettigi olculdu: HAM |dYaw|
    #      adim rms 2.48 (sabit 10) -> 8.44 dps, |ex| p90 13.0 -> 31.6,
    #      hatta bir tohumda KADRAJ KAYBI.
    #  (2) |d_ex| ACI hizidir, yani d = KDEG*v_dik/r. Ayni hedef
    #      hareketi YAKIN menzilde buyuk d uretir; olcut menzille
    #      kendiliginden sertlesir ve yetkiyi tam da chatter'in dogdugu
    #      yerde (r<20 m, tur-3 olcumu: 0.67 -> 7.37 dps) acar.
    #
    # YURURLUKTEKI COZUM: olcut HEDEFIN LOS'A DIK HIZI
    #       v_dik = |d_ex| * r / KDEG   [m/s]
    # yani bozucunun MENZILDEN ARINDIRILMIS hali. Ozellikleri:
    #  * DISSALDIR: d_ex kestirimi kendi yaw hizimizi cikararak
    #    kurulur (BozucuKestirici), yani "hedef manevra ediyor mu"
    #    sorusunun cevabidir; bizim komutumuzun fonksiyonu degildir ->
    #    (1)'deki geri besleme yok.
    #  * MENZILDEN BAGIMSIZ: 40 m'de de 15 m'de de ayni hedef hareketi
    #    ayni sayiyi verir -> (2)'deki yakin-menzil sertlesmesi yok.
    #  * FIZIKSEL: 20 m/s'lik hedefin keskin virajinda v_dik 15-20 m/s
    #    olur, duz bacakta ~0-5 m/s.
    #       v_dik <= 7 m/s  -> r_delta_yaw          (10, sakin)
    #       v_dik >= 18 m/s -> r_delta_yaw_serbest  (1, cevik)
    # Uzerine AGIRLIK LPF'si (yaw_agirlik_tau_s = 0.6 s): kazancin
    # kendisi dongu basina ziplarsa cozum de ziplar. Tarama (tau
    # 0.4/0.6/0.9 x esik 3-12/5-15/7-18) 7-18 + 0.6 s'i secti.
    #
    # OLCULEN SONUC (8 senaryo, iki tohum; sabit 10.0'a gore):
    #   |ex| p90 ortalamasi 18.65 -> 17.60 deg
    #   HAM chatter: keskin rotalarda 2.48 -> 2.48, gercek rotalarda
    #     (elips/wanderer/duz) 1.21 -> 1.61 dps  (hedef < 2.5)
    #   araca ULASAN chatter 1.79 -> 2.17 dps    (hedef < 3.0)
    #   toplam kadraj kaybi 156 -> 153 dongu
    #   dar virajda min menzil 4.3/4.2 -> 3.8/4.3 m
    # Yani manevra yetkisi geri geldi, chatter marji korundu.

    # ---- FOV: SERT KISIT (iki katmanli) ----------------------------
    cbf_ongoru_s: float = 1.0
    cbf_menzil_ref_m: float = 45.0
    cbf_ongoru_min_s: float = 0.35
    # cbf_menzil_ref_m: ongoru ufku MENZILLE ORANTILI olceklenir
    #   (T = cbf_ongoru_s * r / ref). Sebep: kutu merkezinin girdiye
    #   duyarliligi c2*T ile gider ve c2 = KDEG/r; sabit T'de
    #   duyarlilik yakin menzilde patliyor. Olculdu (viraj/yanal,
    #   |dYaw| adim rms): 35-60 m'de 0.67 dps iken 12-20 m'de 7.37.
    #   T ~ r secimi c2*T'yi sabitler.
    cbf_gamma: float = 0.50
    # KATMAN 1 (SERT): kadraj kisiti, UYGULANAN girdi uzerinde kesin
    # saglanir. Anahtar gozlem: bir adim sonraki kadraj degiskeni
    # girdide AFFINE'dir --
    #   beta_{k+1} = B0 - kats*al*(a_dik . u)   (dusey)
    #   ex_{k+1}   = E0 - h*omega               (yatay)
    # yani dusey kisit mevcut DIKEY HIZ DILIMININ daraltilmasi, yatay
    # kisit YAW KUTUSUNUN daraltilmasidir. Ikisi de izdusumun zaten
    # TAM cozdugu kume tipleri -> sert kisit BEDAVA gelir, ne ek
    # degisken ne ek iterasyon.
    # Kisit ayrik-zaman kontrol bariyeri (CBF) formunda yazilir:
    #   h = sinir - beta >= 0,  h_{k+1} >= (1-gamma) h_k
    # Bunun sebebi: devir aninda hedef ZATEN alt kenara yakin geliyor
    # (konumlu hedefin ~42 m gerisine surukleniyor). Duz "beta<=sinir"
    # yazarsak kume BOS olur ve cozucu coker. CBF formunda ihlal
    # varken kisit "iyilestir" der, "aninda sagla" demez -- yani
    # ucusta ASLA infeasible olmaz. gamma=0.30: her adimda ihlalin
    # %30'u kapatilir (20 Hz'de zaman sabiti ~0.17 s).
    fov_sert: bool = True               # ablasyon icin kapatilabilir

    # --- SERT KISITIN EMNIYET SUBAPLARI (2026-08-04 cakilma dersi) ---
    fov_alcalma_talep_tavani_mps: float = 0.4
    fov_tirmanma_talep_tavani_mps: float = 9.0
    # TIRMANMA TALEP TAVANI 5.0 -> 9.0 (2026-08-05, 35 m/s turu).
    # BU TAVAN KAPANMA HIZIYLA OLCEKLENIR, cunku isi geometriktir:
    # standoff hedefin ALTINDADIR (down 4-6 m) ve menzil kapandikca
    # goruntulen yukselis eps = asin(down/r) BUYUR -- 30 m'de 11.5 deg,
    # 12 m'de 30 deg, yani dikey yari-FOV'un (20.07) DISI. Kadraji
    # korumanin tek yolu derinligi menzille ORANTILI kapatmaktir:
    #     tirmanma ~ kapanma_hizi * (down / r)
    # Kapanma 15 m/s ve down/r ~ 0.33 iken bu 5 m/s'i ASAR. Olculdu
    # (kapali dongu paneli, 12 senaryo, tavan 35 m/s): tavan 5.0'da
    # SAF KUYRUK takibi min menzil 11.68 m'de KADRAJI KAYBEDIYOR
    # (|ex| max 1.1 deg -- yani kayip tamamen DIKEY); 9.0'da ayni
    # senaryo 2 m'ye inip CARPISMA ile bitiyor. Panel geneli:
    # kayip %26.9 -> %26.7, ort min menzil 13.2 -> 12.4 m.
    # 9.0 = tirmanma_tavani_mps, yani kisit artik aracin TUM fiziksel
    # tirmanma yetkisini isteyebilir. Tirmanma ALCALMA gibi tehlikeli
    # degildir (yerden UZAKLASIR) -- asimetri bu yuzden bilinclidir:
    # alcalma talebi 0.4, tirmanma talebi 9.0.
    # TARIHCE (3.0 -> 5.0, 2026-08-04 montaj 0 olcumu):
    # Mount 0'da kadraji kurtaran yon TIRMANMAKTIR (hedef eksenin
    # USTUNDE): sabit derinlikte menzil kapandikca eps buyur, yani
    # kadraji korumak icin derinligi menzille ORANTILI kapatmak
    # gerekir -- kapanma 10 m/s ve d/r ~ 0.23 iken bu tek basina
    # 2.3 m/s tirmanma demek, uzerine duzeltme payi. 3.0 tavani
    # cebir taramasinda vakalarin %18'inde baglayiciydi (kisit
    # "daha cok tirman" diyemiyordu); 5.0'da %6.5. Kapali dongude
    # olculdu (6 senaryo): kadraj kaybi %2.37 -> %1.75, min menzil
    # ve yer emniyeti degismedi. Tirmanma ALCALMA gibi tehlikeli
    # degil (yerden UZAKLASIR), o yuzden asimetri bilinclidir:
    # alcalma talebi 0.4, tirmanma talebi 5.0.
    # KADRAJ KISITININ ISI TIRMANMAYI YASAKLAMAKTIR, ALCALMA
    # EMRETMEK DEGIL. Kayip mekanizmasi "tirmanma -> burun yukari ->
    # sabit kamera yukari bakar -> hedef alttan cikar" oldugu icin
    # dogru mudahale tirmanmayi kesmektir. Alcalmak da geometrik
    # olarak beta'yi kucultur (hedef bize gore yukselir) ama bu
    # TEHLIKELI bir kaldiractir: benzetimde olculdu, kisit serbest
    # birakildiginda arac kadraji kurtarmak icin 22 m alcaldi ve
    # yere ucti (fov_sert=False'ta hic alcalmiyor). Bu yuzden kisitin
    # talep edebilecegi alcalma 0.4 m/s ile sinirli: pratikte
    # "tirmanma <= 0" demek. Yetmiyorsa zaten kurtarilamiyordur ve
    # zaman asimi kisiti tamamen birakir.
    irtifa_taban_m: float = 25.0
    irtifa_yaklasma_s: float = 3.0
    # KENDI IRTIFA TABANIM. goruntulu_temel'de yontemden bagimsiz bir
    # taban (15 m) var ama kontrolcu ona YASLANMAMALI: o son
    # savunmadir, guduma girmez. Buradaki taban dikey hiz DILIMININ
    # ust sinirina yazilir:
    #     vz <= (irtifa - taban) / yaklasma_s
    # yani tabana yaklasirken izin verilen alcalma dogrusal olarak
    # sifire iner. Normali a_dik'e paralel oldugu icin MEVCUT dilimle
    # ayni kumeye girer -> izdusum kapali formda kalir, bedava.
    # NEDEN VAR: 2026-08-04 cakilmasinda kadraj kisiti araci yere
    # indirdi. Kok neden duzeltildi ama dikey kanalda "neden olursa
    # olsun" calisan bir taban olmasi sart -- benzetimde bu taban
    # olmadan alcalma bir kez baslayinca kendini besliyor.
    kisit_alcalma_esigi_mps: float = 0.25
    bos_kume_tavan_dongu: int = 40
    bos_geri_esik_dongu: int = 15
    # Kisit ~2 s (20 Hz'de 40 dongu) boyunca KARSILANAMAZSA ya da
    # kesintisiz alcalma dayatirsa (bkz. kisit_alcalma_esigi) kadraj
    # kisiti tamamen BIRAKILIR. Gerekce: o noktada hedef zaten
    # kurtarilamiyor; kadraj kaybini kabul etmek, kadraji kovalayarak
    # yere ucmaktan iyidir. Karar verici 1.5 s dwell sonrasi
    # 'konumlu'ya doner ve yeniden konumlandirir.
    # bos_geri_esik: birakilmis kisit, sayac bu esigin altina
    # sonumlenince YENIDEN DEVREYE girer (histerezis). 40/15 bandi
    # ~1.25 s birakma demektir; sinirsiz latch (eski hata) degil.
    bayat_kisit_s: float = 0.30
    # bbox bu yastan eskiyse kadraj kisiti DARALTMA YAPMAZ.
    # KACAK DONGU: bbox bayatken ey DONAR; beta'yi degistiren tek
    # terim -kats*vz kalir, yani kisit kendi urettigi dikey hizi
    # olcup daha da sertlesir. Olculdu (elips blok 2): ey -14.57'de
    # donmusken beta 1.4 s'de 4.96 -> 37.42. Iskelet kisa boslukta
    # son komutu 1 s TUTTUGU icin bu pencerede uretilen asiri komut
    # bir saniye boyunca uygulanir -- kapi bu yuzden sart.

    rho_fov: float = 8.0
    rho_fov_dikey: float = 0.0
    fov_l1_delta_deg: float = 1.5
    # KATMAN 2 (PLAN): ufuk boyunca l1 TAM CEZA (exact penalty).
    # Kuadratik ceza kisit ihlalini asla tam sifirlamaz (gradyani
    # ihlalle birlikte sonuyor); l1'in gradyani sabit rho oldugu icin
    # rho yeterince buyukse cozum SERT kisitli cozume matematiksel
    # olarak esittir, sagalanamadigindaysa cozucu yine coker degil,
    # en az ihlali secer. l1 turevlenemez oldugundan Huber ile
    # yumusatilir (|ihlal| < delta bolgesinde kuadratik): birinci
    # mertebe yontem gecerli kalir, delta=0.75 deg cozunurluk zaten
    # bbox gurultusunun (0.08 deg) cok ustunde.
    # rho SECIMI: tam ceza teoremi rho > ||lambda*||_inf ister.
    # Ana maliyetin bir dereceye gore gradyani ~0.4 mertebesinde
    # (q/olcek^2 * deger), yani rho=8 yaklasik 20 kat paydir --
    # kisit saglanabilir oldugunda cozum SERT kisitli cozumle ayni.
    # Daha buyuk rho, Huber egriligini (rho/delta) ve dolayisiyla
    # FISTA adim boyunu gereksiz cezalandiriyor (rho=30/delta=0.75
    # denendi: L 3e4'e cikti, cozucu felc oldu).
    # rho_fov_dikey = 0 (VARSAYILAN, olcumle secildi): dikey kadraj
    # kisitini ZATEN sert CBF dilimi tam olarak sagliyor; ustune l1
    # cezasi koymak hicbir sey kazandirmiyor ama cok pahali. Cunku
    # beta satiri girdiye COK duyarli (-kats*a_dik.w terimi girdiye
    # dogrudan al kazanciyla baglaniyor), Ga^T Ga buyuk cikiyor ve
    # L 14 -> 1.4e3'e firliyor; FISTA 30 iterasyonda hicbir yere
    # gidemiyor (olculdu: u1 = 9.6 kalirken sert-kisit-tek-basina
    # cozumu u1 = 16.1). Yatay kanalda ise ex satiri girdiye yalnizca
    # integrasyon uzerinden bagli, Ga kucuk, ceza bedava; ayrica yaw
    # yetkisi DOYABILDIGI icin (wanderer'da %9.4 doyum olculdu) orada
    # ufuk boyu ceza gercekten is goruyor.
    fov_uyanik_pay_deg: float = 4.0
    # L hesabinda "kink'e yakin" sayilan pay. Huber'in DOGRUSAL
    # bolgesinde egrilik SIFIRDIR; oradaki satirlari L'ye katmak
    # adimi bosuna kucultur. Bu yuzden pay dar tutulur ve iraksama
    # riski uyarlanabilir yeniden baslatmaya birakilir.
    # Adim boyu (L) hesabinda "az sonra aktif olabilir" sayilan pay.
    # Ceza Hessian'i lambda_max'i tek basina ~145 kat sisiriyor; onu
    # HER ZAMAN L'ye katmak, ceza pasifken (zamanin cogunda) adimi
    # gereksiz kucultuyor. Bu yuzden L yalnizca AKTIF/yakin satirlarla
    # kurulur.

    lambda_prox: float = 1.0
    # Proksimal duzenlileme: ||u - u_warm||^2 cezasi. IKI isi var:
    # (1) Hessian'in kucuk ozdegerini yukselterek kosul sayisini
    #     ~1.3e3'ten ~2.7e2'ye dusurur -> FISTA 2.3 kat daha az
    #     iterasyonla ayni dogruluga varir (olculdu).
    # (2) Planin dongu basina ne kadar degisebilecegini sinirlar;
    #     linearizasyon hatasina ve olcum gurultusune karsi
    #     "gercek zamanli iterasyon" (real-time iteration) MPC'nin
    #     standart korumasi budur.

    # ---- bozucu (hedef hareketi) kestirimi -------------------------
    bozucu_guven_s: float = 0.8
    # BOZUCU GUVENI. Devir aninda d = 0'dir; ama hedef 20 m/s ucuyor ve
    # d fiziksel olarak "hedefin dik hizi / menzil" (tipik 15-25 deg/s).
    # d=0 varsayimiyla MPC hedefi DURAGAN saniyor ve yanlis (sert fren)
    # manevra kuruyor -- benzetimde olculdu: ilk saniyede kadraj
    # kaybi. Cozum: (a) ilk orneklerde LPF kazanci 1/n (kosan ortalama)
    # olarak alinir, boylece d birkac dongude oturur; (b) LOS-hizi
    # (PN) agirligi bu sure boyunca guven ile carpilir. Guven dusukken
    # baskin terim menzil olur -- yani konumlu gudumun yaptigi sey,
    # guvenli varsayilan.
    bozucu_tau_s: float = 0.45
    bozucu_kutu_tau_s: float = 1.2
    bozucu_mutlak_tavan_dps: float = 60.0
    bozucu_dondur_menzil_m: float = 15.0
    # bozucu_kutu_tau_s: SERT KISITIN kutu merkezini besleyen AYRI,
    #   yavas kopyanin zaman sabiti. Maliyet referansi 0.45 s'de kalir
    #   (gudum tepkisi), kisit siniri 1.2 s ile yumusar. Tur-3'te ayni
    #   gurultu ikisine birden giriyordu ve yaw chatter'inin (4 Hz,
    #   +-16 dps) dogrudan kaynagiydi.
    # bozucu_mutlak_tavan_dps: fizik kelepcesine EK mutlak tavan.
    # bozucu_dondur_menzil_m: bu menzilin altinda bozucu DONDURULUR.
    # d = olculen aci hizi - model aci hizi. Ham hali cok gurultulu
    # (bbox merkezi +-1-2 px titriyor); 0.45 s LPF hedef manevrasinin
    # bandini (~1 Hz altinda) gecirir, piksel gurultusunu keser.
    hedef_hiz_tavani_mps: float = 40.0
    # d kelepcesi fizik uzerinden: |d| <= KDEG * v_hedef_max / menzil.
    # 40 m/s makul hedefin (20 m/s) iki kati -> emniyetli ust sinir.

    # ---- APN: hedef yanal IVMESI (env dugmeli) ---------------------
    # Gerekce, kok neden ve olcum: bkz. cevre_apn().
    apn: bool = field(default_factory=lambda: cevre_apn() > 0.0)
    apn_carpani: float = field(
        default_factory=lambda: max(cevre_apn(), 0.0) or 1.0)
    apn_tau_s: float = 0.6
    # a_dik = d(v_dik)/dt TUREVdir: d_ex'in kendi LPF'i (0.45 s) uzerine
    # ikinci bir turev almak gurultuyu ~1/dt kati buyutur. 0.6 s AYRI ve
    # daha yavas bir LPF sart. 1/n hizli baslangic BILINCLI OLARAK YOK
    # (d_ex'ten farki budur): ilk turev ornegi tek basina kestirimi
    # kelepceye (+-6) surerdi. Sifirdan rampa + guven carpani = guvenli
    # taraf; ~2 s'de oturur, angajmanlar 20-60 s.
    apn_a_tavani_mps2: float = 6.0
    # Hedef sabit kanatli; 20 m/s'de 6 m/s^2 yanal ivme R ~ 67 m donus
    # yaricapi demek (~0.6 g). Elips hedefin gercek talebi bunun altinda;
    # tavan gurultu patlamalarini kesmek icin var, yasayi kisitlamak icin
    # degil.
    apn_olu_bant_mps2: float = 0.5
    # CIKARMALI (yumusak) olu bant: a_etkin = sign(a)*max(|a|-db, 0).
    # SERT olu bant esikte sicrama uretir ve o sicrama dogrudan ufka
    # yayilir. Cikarmali hali esikte SUREKLIdir. Amac: DUZ bacakta
    # (gurultu tabani ~0.3-0.5 m/s^2 olculdu) katki TAM SIFIR kalsin.
    menzil_bozucu_kaynak: str = "kapali"
    # "kapali" | "menzil". Menzil turevinden kapanma hizi cikarmak
    # hedefin RADYAL hizini turetmek demektir; goruntulu_temel
    # sozlesmesi hedef telemetrisinden YALNIZ menzile izin veriyor.
    # Varsayilan KAPALI. Acilirsa d_r kestirilir (menzil terimi biraz
    # daha dogru olur, davranis degismez cunku zaten tam gaz kapaniyoruz).

    # ---- menzil suzgeci --------------------------------------------
    alan_hizi_tau_s: float = 0.35
    # bbox alani buyume hizi LPF'si. Alan w*h oldugu icin gurultusu
    # karekokunkinin ~2 katidir; 0.35 s yaklasma isaretini gecirir.
    menzil_olcum_kazanci: float = 0.35
    # Estimator menzili 10 Hz ve gurultulu; ic durum model ile
    # ilerletilip (r -= w1*dt) olcume dogru bu kazancla cekilir.
    # Boylece bbox var ama menzil bayatsa gudum kor kalmaz.
    menzil_yoksa_m: float = 55.0
    # Hic menzil gelmediyse: devir kapisi menzili <=60 m'de aciyor,
    # 55 m makul bir baslangictir (yanlissa suzgec 1-2 s'de duzeltir).
    menzil_taban_m: float = 6.0
    # c = KDEG/r katsayisi r->0'da patlar; 6 m'de zaten carpistik.

    # ---- eyleyici modeli -------------------------------------------
    hiz_gecikme_tau_s: float = 1.00
    # Komut -> gercek hiz gecikmesi. ZINCIR: iskelet LPF'si (tau=0.35)
    # -> |v|<=18 kelepce -> otopilotun hiz dongusu, ki o WPNAV_ACCEL =
    # 5 m/s^2 ile IVME SINIRLIDIR (params/swarm_copter.parm). Yani
    # tepki 1. mertebe degil RAMPA'dir: 10 m/s'lik bir hiz degisimi
    # 2 s surer. 1. mertebe esdegeri tau ~ 0.63*dV/A; dV~8 m/s tipik
    # oldugundan tau ~ 1.0 s. Bu sayi ufku da belirliyor: MPC yanal
    # hizi ANCAK 3 s'de degistirebildigini bilmeli, yoksa "son anda
    # duzeltirim" diye planlayip kaciriyor. Model tau'yu KUCUK
    # tutmak (0.6) benzetimde asma ve kadraj kaybi uretti.
    #
    # *** 2026-08-08 OLCUMU: BU SAYI CALISILAN BANTTA ~6 KAT KUCUK. ***
    # Yukaridaki "dV~8 m/s" varsayimi tutmuyor; olculen |e| dagiliminin
    # tepesi 10-20 m/s ve orada tau_etkin 5.5-6.8 s. Ustelik zincir
    # 1. mertebe DEGIL, IVME SINIRLI (plato ~4 m/s^2) -- yani tek bir
    # tau ile temsil edilemez. Bu alan DEGISTIRILMEDI (kapali kol
    # bit-ayni kalsin); duzeltme ayri env dugmesi arkasinda:
    # bkz. cevre_eyleyici() / eyleyici* alanlari asagida.

    # ---- DOYUMLU eyleyici (env dugmeli, varsayilan ACIK) ------------
    # Gerekce, kok neden ve olcum: bkz. cevre_eyleyici().
    eyleyici: bool = field(default_factory=lambda: cevre_eyleyici() > 0.0)
    eyleyici_tau_lin_s: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_TAU_LIN', 1.7))
    # Dogrusal bolge zaman sabiti. Olculen: 2.33 / 1.56 / 1.74 s
    # (|e| < 4 m/s bandi, uc kosu) -> 1.7 ortalamaya yakin.
    eyleyici_a_max_mps2: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_A_MAX', 4.0))
    eyleyici_tau_lin_z_s: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_TAU_LIN_Z', 1.0))
    # DIKEY dogrusal bolge zaman sabiti. ILK SURUMDE YOKTU: dikey,
    # modelin eski 1.00 s'sinde birakilmisti ("ayrica olculmedi,
    # muhafazakar" demistim). OLCTUM (yatayla AYNI yontem, EYL KAPALI
    # 6 kosu, e = cmd_vz - vel_z, a_par = (dvz/dt)*sign(e), |e|<3 m/s
    # bolgesinde orijinden gecen dogru):
    #     egim 0.294-0.575 1/s  ->  tau_lin_z = 1.74 / 1.77 / 1.86 /
    #     2.41 / 2.54 / 3.40 s,  ORTALAMA 2.16 s
    # YANI DIKEY, YATAYDAN DAHA YAVAS (2.16 vs 1.70) -- model ise onu
    # 1.00 s ile YATAYDAN HIZLI sanıyordu. Yatay gercekci sekilde
    # yavaslatilinca cozucu talebi bu YAPAY UCUZ kanala kaydirdi:
    #     |u3| p90 6.77 -> 14.74 m/s, u3>u1 %7.5 -> %19.8,
    #     |cmd_vz| p90 6.38 -> 9.00 (tavanda), abort/angajman 0.36 -> 0.69
    #
    # *** VARSAYILAN 1.0'A GERI ALINDI (2026-08-08 gece-2, eyl2 kosusu). ***
    # Olculen 2.2'yi UYGULAMAK ISE YARAMADI, TERSINE COTULESTIRDI:
    #     ALTITUDE ABORT / angajman  0.54-0.92 (dikey eski) -> 1.11 (dikey yeni)
    #     |cmd_vz| TAVANDA kare orani      %5.4 -> %20.3  (4 kat)
    #     |cmd_vz| p90                     8.53 -> 8.99   (tavan 9)
    # Yani dikey kanali "pahalilastirmak" dikey talebi AZALTMADI. CBF de suclu
    # degil: zorla-alcalma dayattigi kare orani %7.6 -> %2.2'ye DUSTU (kutu
    # acildi) ama komut yine tavana yapisti. Demek ki dikey talebi kuran sey
    # eyleyici modeli DEGIL, maliyet/geometri tarafi (dikey standoff + eps
    # kovalamasi) -- o AYRI BIR TURUN isi.
    # OLCULEN DEGERLER SILINMEDI: tau_lin_z = 2.16 s (1.74/1.77/1.86/2.41/
    # 2.54/3.40, EYL-KAPALI 6 kosu) ve plato 5.25 m/s^2 (= WPNAV_ACCEL_Z 500)
    # DOGRU sayilardir; dikey maliyet/geometri duzeltildiginde
    # YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5 ile ACILIR.
    # VARSAYILAN 1.0 + a_max_z = sonsuz  ==  dikey doyum YOK, yani
    # YILDIZ_EYLEYICI=1 tam olarak KANITLI YATAY-YALNIZ koldur.
    eyleyici_a_max_z_mps2: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_A_MAX_Z', float('inf')))
    # DIKEY ivme tavani. ILK SURUMDE YOKTU ve bu OLCULEBILIR BIR ARIZA
    # URETTI (2026-08-08 gece-2): doyum yalniz yataya uygulaninca cozucu
    # ayni kadraj/kapanma talebini IYIMSER KALAN dikey kanaldan karsiladi
    # -- "su ucuz kanaldan akar". Olculdu (11 kosu, EYL kapali n=6 vs
    # acik n=5):
    #     |u3| p90        6.77 -> 14.74 m/s   (2.2 kat)
    #     u3 > u1 orani   %7.5 -> %19.8       (2.6 kat)
    #     |cmd_vz| p90    6.38 ->  9.00 m/s   (ACIK kollarin HEPSI tavanda)
    #     ALTITUDE ABORT / angajman  0.36 -> 0.69
    # VARSAYILAN sonsuz = DIKEY DOYUM KAPALI (bkz. eyleyici_tau_lin_z_s
    # icindeki eyl2 negatif sonucu). Olculen deger 5.0'dir ve env ile acilir.
    # 5.0 m/s^2 nereden: EYL KAPALI kollarin GERCEKLESEN dikey ivmesi
    # (|az| p90 ort 4.57, buyuk-hata platosu ort 5.25) ve WPNAV_ACCEL_Z
    # = 500 cm/s^2 = 5.0 m/s^2 ile TAM ORTUSUYOR. Yani yataydan farkli
    # olarak dikeyde WPNAV_ACCEL_Z GERCEKTEN baglayici.
    # NOT: dikeyin DOGRUSAL bolge tabani hiz_gecikme_tau_s (1.00 s)
    # olarak BIRAKILDI -- ayrica olculmedi, bu degisiklik yalnizca
    # EKSIK OLAN IVME SINIRINI ekler, dogrusal bolgeyi yeniden ayarlamaz.
    # Yatay ivme platosu. Olculen: 4.07 / 3.81 / 3.96 m/s^2 (|e| > 8 m/s,
    # a_par p90). WPNAV_ACCEL 250 <-> 500 degisimi bu platoyu
    # DEGISTIRMEDI, yani sinir WPNAV_ACCEL degil.
    pitch_lpf_tau_s: float = 0.30
    pitch_alt_deg: float = -35.0
    pitch_ust_deg: float = 35.0
    # ey_ref = -(mount + pitch): kamera ekseni pitch'le GERCEKTEN
    # kayar, bandin pitch'i izlemesi FIZIKTIR.
    # TARIHCE (onemli): once 1.2 s LPF + [-20,+8] kelepce vardi.
    # Sebep, anlik pitch'i izlemenin POZITIF GERI BESLEME uretmesiydi
    # (fren -> burun yukari -> ey_ref asagi -> daha sert manevra ->
    # daha cok burun yukari). O kelepce ARTIK ZARARLI: sert CBF
    # kisiti tirmanma->pitch etkisini MODELIN ICINDE tasidigi icin
    # geri besleme dogru sekilde kapaniyor, kelepce ise yalnizca
    # gercek pitch +8'i astiginda beta'yi KORLESTIRIYOR -- benzetimde
    # olculdu: beta p95 = 12 "guvenli" gorunurken hedef HAM kadrajin
    # alt kenarindan cikiyordu. Kelepce +-35'e acildi (fiziksel yatma
    # siniri), LPF 0.30 s'ye indi (kadraj ANLIK pitch'e bakar, gecmise
    # degil).

    # ---- cozucu ----------------------------------------------------
    # BOL BUTCE DUGMESI (YILDIZ_COZUCU_BOL=1): iterasyon_tavani 26 -> 40,
    # sure_butcesi_ms 13 -> 18. Gerekce ve DONANIM UYARISI icin bkz.
    # cevre_cozucu_bol(). Ozet: butce_kesti orani her kolda %52-89 ve bu,
    # A/B farklarinin ustune binen bir OLCUM gurultusu. Dugme SIM olcum
    # kalitesi icindir; Pi 5 karari AYRIDIR (orada dongu kacirma riski).
    iterasyon_tavani: int = field(
        default_factory=lambda: 40 if cevre_cozucu_bol() > 0.0 else 26)
    iterasyon_tabani: int = 5
    ilk_iterasyon_tavani: int = 400
    ilk_butce_ms: float = 25.0
    ilk_cozum_sayisi: int = 2
    devir_prox_tohum: bool = True
    # DEVIR PROKSIMAL DEMIRI. coz() fark cezasini ||u - u_onceki||^2
    # olarak kurar; u_onceki ILK dongude YOKTUR ve eskiden SIFIR
    # birakiliyordu. Yani ilk QP'ye "onceki komut duruyordu" diye
    # yalan soyleniyor ve r_delta_hiz agirligi ilk komutu DURMAYA
    # cekiyordu -- oysa arac o anda 17 m/s ile ucuyor. U_warm ZATEN
    # devir hiziyla tohumlanmisti (bkz. _warm_start); eksik olan
    # proksimal DEMIRDI. Olculdu (offline devir, 42 kosu x 5 devir
    # yaw hizi): ilk komut ile konumlunun son komutu arasindaki fark
    # 4.36 -> 4.17 m/s, ilk 1 s ortalamasi 3.38 -> 2.50 m/s. Etki
    # kucuk ama BEDELI SIFIR HESAP ve isareti her kosuda ayni.
    devir_yaw_tohum: bool = True
    # Devirde yaw hizi LPF'sinin ILK ORNEK kazanci (k=1). Bozucunun
    # ILK artigini kirleten tek kalem budur (bkz. _yaw_hizi_olc);
    # asil devir kazanci buradan gelir. Ikisi de ablasyon icin ayri
    # anahtar; varsayilan ACIK.
    # SOGUK BASLANGIC: devir aninda onceki cozum yoktur, warm start
    # yalniz konumlunun son hizidir. Olculdu: sicak baslatilmis cozum
    # 3-5 DONGUDE (0.15-0.25 s) tam optimuma oturuyor, ama ilk dongude
    # hala uzak. Ilk 2 cozume genis butce vermek bu gecikmeyi kaldirir;
    # bir kereye mahsus 25 ms, t_go 3-10 s yaninda onemsiz.
    tolerans_mps: float = 0.02
    # Durma olcutu FIZIKSEL: ardisik iterasyonlar arasi girdi degisimi
    # 2 cm/s'in altina inince durulur (yaw hizi ayni olcege cekilir).
    # Bunun altini kovalamak anlamsiz -- otopilotun hiz dongusu
    # zaten bu cozunurlukte degil.
    sure_butcesi_ms: float = field(
        default_factory=lambda: 18.0 if cevre_cozucu_bol() > 0.0 else 13.0)
    # DUVAR SAATI KORUMASI. 6.0 IDI ve tur-3'te donguletin %38-47'sinde
    # ASILIYORDU -- yani cozucu duzenli olarak YARIDA kesiliyordu.
    # Ustelik kontrol 8 iterasyonda bir yapildigi icin iterasyon
    # sayisi 8/16/24'te KUMELENIYOR, ardisik donguler FARKLI
    # noktalarda kesiliyor ve komut bundan ziplama aliyordu: 1-5 Hz
    # bandindaki yaw varyansinin yalnizca %12-29'u ex+d_ex ile
    # aciklaniyordu, gerisi COZUCU kaynakli. Dongu 50 ms, p95 zaten
    # 9-10 ms; butce 13 ms'e cikarilinca kesme NADIR olay olur ve
    # iterasyon sayisini yakinsama toleransi belirler (tutarli).
    # Kontrol artik HER iterasyonda yapilir (kumelenme yok).
    sqp_gecis: int = 1
    # LTV katsayilari (c=KDEG/r) ongorulen menzil yorungesi etrafinda
    # dondurulur. 1 gecis warm-start yorungesiyle yeterli; 2 daha
    # dogru ama ~2x sure. Test dosyasi ikisini de olcuyor.

    # ---- yaw politikasi --------------------------------------------
    yaw_komutu_ver: bool = True
    # False ise yaw otopilotta kalir ve MPC yalnizca hiz kumandalar
    # (ablasyon deneyi icin: "ayri FOV kontrolcusu gerekli mi?").

    # ---- VURUS FAZI (2026-08-05) -----------------------------------
    # ISTEK (kullanici, 2026-08-05): "hedefi gordugu andan itibaren
    # birakmadan ustune hizlanan ve CARPAN" gudum. Olculen kusur iki
    # parcaliydi: (1) hiz paritesi (yukarida, hiz_tavani_mps),
    # (2) YAKIN MENZILDE MALIYETIN HALA "TAKIP" MALIYETI OLMASI.
    #
    # (2)'nin mekanigi: maliyetin yakin menzildeki baskin kalemleri
    # kadraj korumasidir -- SERT FOV kisiti dikey hiz dilimini ve yaw
    # kutusunu daraltir, fren daraltmasi alt bandi 4 dereceye kadar
    # cokertir ve kisit "tirman" der. Bunlarin hepsi UZUN SURELI TAKIP
    # icin dogrudur (kadraj kaybi kosuyu bitirir) ama son bir saniyede
    # YANLIS: orada hedefi kadrajda tutmanin degeri, hedefe CARPMANIN
    # degerinden kucuktur. Olcum: asili (durgun) hedefte min menzil
    # 0.47 m, yani terminal hassasiyet zaten var; kaybedilen sey
    # hareketli hedefte terminal FAZIN KENDISI.
    #
    # COZUM: menzille birlikte SUREKLI (kademeli) bir karisim kats.
    #     s = clip((vurus_menzil - r) / (vurus_menzil - vurus_tam), 0, 1)
    # s = 0 -> bugunku maliyet aynen; s = 1 -> saf yakalama. s'nin
    # yaptigi uc sey:
    #   * FOV bantlarini FIZIKSEL KENARA acar (14/17.5 -> 19.0; dikey
    #     yari-FOV 20.07, yani 1 derece pay). Kadraj hala kisitlidir --
    #     kamera 0 deg SABIT montaj oldugu icin kadrajdan cikan hedef
    #     bizi KOR birakir -- ama kisit artik "merkezle" degil "kenara
    #     degme" der. Yatay sinir 26 -> 31 (fiziksel 33).
    #   * fren/hizlanma kaynakli bant daraltmasini (1-s) ile sonumler:
    #     terminal manevranin bedelini kadraj bandindan almaz.
    #   * agirliklari cevirir: kadraj (q_ex,q_ey) YUKARI -- bbox
    #     merkezine kilitlenmek carpismanin ta kendisidir (ex=0,
    #     ey=ey_ref kadrajin MERKEZIDIR, bkz. _kadraj_sabiti) --,
    #     odul (q_alan, q_alan_hizi) YUKARI, ivme/yatma cezasi (q_ivme)
    #     ASAGI (agresiflik dugmesi; 27 deg yatma < 1 s icin kabul).
    vurus_modu: bool = True
    vurus_menzil_m: float = 22.0
    # VURUS FAZINA GIRIS MENZILI. Uc olculmus sayidan tureti:
    #   * DEVIR menzili 26.8-33.5 m (17 yetki segmenti) -> 22 m
    #     BUNUN ALTINDA: faz devir aninda ASLA acilmaz, yani cozucunun
    #     soguk oldugu ilk saniyede agresif maliyet devreye girmez.
    #   * gecis (pass) kolu 12 m'de silahlanir -> 22 > 12: vurus fazi
    #     gecis tespitinden ONCE baslar (dogru sira).
    #   * t_go: 22 m'de kuyruk takibinde kapanma 35-21 = 14 m/s ->
    #     1.6 s; kafa kafaya 56 m/s -> 0.4 s. Yani faz "son 0.4-1.6 s".
    vurus_tam_menzil_m: float = 8.0
    # s = 1 (TAM birakma) menzili. 8 m, gecis kolunun acilma esigiyle
    # (iska_gecis_acilma_m) ayni sayi ve ucak govdesinin (~6.4 m kanat
    # acikligi) hemen ustu: bu menzilin icinde kadraj kisitinin
    # soyleyebilecegi hicbir sey vurusu iyilestiremez.
    vurus_bant_deg: float = 19.0        # dikey yari-FOV 20.07 -> 1 deg pay
    vurus_ex_siniri_deg: float = 31.0   # yatay yari-FOV 33 -> 2 deg pay
    vurus_ex_carpani: float = 1.0       # q_ex carpani (s=1'de)
    vurus_ey_carpani: float = 1.0       # q_ey carpani (s=1'de)
    vurus_los_carpani: float = 1.0      # q_los_hiz carpani (s=1'de)
    # KADRAJ/PN CARPANLARI NEDEN 1.0 -- yani VURUS bu agirliklari
    # DEGISTIRMIYOR (2026-08-05, olcumle secildi):
    #
    # Kullanici istegi "bbox merkezine kilitlen (ex,ey -> 0)" idi.
    # ONEMLI: BU ZATEN BOYLE. Maliyetin kadraj referanslari ex -> 0 ve
    # ey -> ey_ref'tir ve (ex=0, ey=ey_ref) TANIM GEREGI bbox'in
    # kadraj MERKEZIDIR (bkz. _kadraj_sabiti). Yani vurus fazinda
    # degistirilecek olan REFERANS degil, kadrajin dayattigi FRENDIR.
    #
    # Agirliklari BUYUTMEK denendi ve OLCULDU (kapali dongu, 6
    # senaryo x 5 tohum):
    #   * q_ex x3  -> keskin/capraz medyan min menzil 12.87 -> 12.84,
    #     viraj/capraz'da ise tek tohumda 2.34 -> 9.48 m'lik bozulma.
    #     Sebep teorik olarak da belli: "ex -> 0" SAF TAKIP'tir
    #     (bkz. dosya basligi, "SAF TAKIP vs CARPISMA ROTASI");
    #     agirligini buyutmek optimizasyonu kuyruk takibine iter,
    #     oysa carpismayi kuran terim ATALETSEL LOS HIZI'dir.
    #   * q_ey x3  -> u3 (LOS'a DIK dusey hiz) talebini buyutur ve o
    #     hiz dogrudan u1'in (kapanma) yerine gecer; yakin menzilde
    #     c3 = KDEG/r patladigi icin kucuk bir ey hatasi bile buyuk
    #     u3 uretir -> vurus fazi KENDINI FRENLER (olculdu: s=1'de
    #     u1 27.33 -> 25.15 m/s, yani kapanma AZALIYOR).
    #   * q_los_hiz x2 -> panel ortalamasi 6.75 -> 8.14 m (kotu).
    # Uc knob da AYARLANABILIR birakildi (ablasyon ve gercek donanim
    # icin) ama varsayilan 1.0'dir: VURUS fazinin isi agirlik
    # degistirmek degil, KISITI ve IVME CEZASINI cozmektir.
    # Dikey kadraj zaten IKI mekanizmayla korunuyor: nominal q_ey
    # (0.35 -> 0.60'a cikarildi) ve SERT bant (VURUS'ta 19 deg).
    vurus_ivme_carpani: float = 1.0     # q_ivme carpani (s=1'de)
    # 0.35 -> 1.0 (2026-08-05, SIM TURU SONRASI, olcumle).
    # Ilk tasarimda VURUS fazinda ivme (yatma) cezasi 0.35x'e
    # KISILIYORDU: "agresiflik dugmesi". Sim gosterdi ki bu dugmeye
    # GEREK YOK ve BEDELI BUYUK:
    #  * GEREK YOK: hiz paritesi tek basina yetti -- u1 max 35.00,
    #    cmd_hiz p95 34.78, kapanma menzil_hizi min -18.4 m/s. Yani
    #    tavana zaten deginiyor; ekstra ivme yetkisi bos duruyordu.
    #  * BEDELI: ivme = YATMA ACISI (pitch = -atan(a/g), 5.84 deg per
    #    m/s^2) ve kamera 0 deg SABIT. Serbest birakilan ivme dogrudan
    #    kamera eksenini savuruyor. Sim: |pitch hizi| ortancasi
    #    3.6 -> 13.3 deg/s (kadrajda ~952 px/s ufuk kaymasi = gorsel
    #    titreme), kadraj kaybi %8.6 -> %48.3, kayiplarin %18.5'i UST
    #    kenardan (alt %6.0), bunun %10.9'u FIZIKSEL FOV DISINDA.
    # OFFLINE A/B (duz rota, hedef 21.05 m/s, 3 devir x 2 tohum):
    #     carpan   pitch hizi   ust kenar %   min menzil   cmd p95
    #       0.35      3.81         12.0          1.93        35.0
    #       0.70      2.74          9.6          2.03        35.0
    #       1.00      2.30          9.6          1.90        35.0
    # Yani kisitlamayi birakmak pitch hizini %40 dusuruyor, ust kenar
    # kaybini %20 azaltiyor ve KAPANMAYA HIC DOKUNMUYOR. Bedelsiz.
    # NOT: 1.0'in OTESI (VURUS'ta cezayi ARTIRMAK) da denendi; bkz.
    # asagidaki olcum tablosu -- kapanmayi bozdugu icin alinmadi.
    vurus_odul_carpani: float = 2.0     # q_alan, q_alan_hizi carpani

    # ---- VURUS TERMINAL DIKEY HIZALAMA (2026-08-05, tur-3) ----------
    # EMEKLI (gimbal dali, 2026-08-05, kullanici onayi): mekanizmanin
    # varlik nedeni "eps buyuyunce hedef UST kenardan cikar, o yuzden
    # hedef hattina tirman" idi. Fiziksel stabilize gimbal + Faz C
    # (tilt'in eps'i takibi) ayni sorunu KAMERAYI dondurerek cozuyor;
    # tirmanma bias'i ise kapanmayi yiyordu (offline: u1 28.29 -> 27.59).
    # Kod ve birim testleri (acik parametreyle) duruyor; ablasyon icin
    # vurus_hiza_kapatma=True verilebilir.
    vurus_hiza_kapatma: bool = False
    vurus_hiza_rahat_deg: float = 5.0
    vurus_hiza_tau_s: float = 1.5
    vurus_hiza_tavan_dps: float = 8.0
    # RAHAT BANDI 10 -> 5 (2026-08-05, tur-3 SIM olcumu).
    # Tur-3'te mekanizma KAPALI DONGUDE DOGRULANDI: hiza_ref 39 karede
    # aktif, hepsi pozitif (tek-yanlilik tuttu), CPA'da DIKEY AYRIM
    # 0.81 m -> 0.05 m (standoff gercekten eridi), CPA 0.84 -> 0.62 m
    # (gercek temas: vibe 2.0 -> 25.5). AMA ust-kenar fiziksel-disi
    # kaybi %16.24 -> %16.26, yani HIC DUSMEDI. Sebep olculdu:
    # mekanizma terminal karelerin YALNIZ %8.1'inde aciliyor, cunku
    # VURUS'ta eps ortancasi 4.6 deg, p95 13.6 deg -- 10 deg'lik
    # deadzone karelerin ~%90'ini disarida birakiyor.
    # 5 deg, olculen dagilimin ORTANCASININ hemen ustudur: aktif
    # pencere ~%8 -> ~%45. Tur-3'un iki emniyet olcumu bu daraltmayi
    # destekliyor: (1) ALT KENAR BOZULMADI (39 aktif karenin 0'i alt
    # tasmayla izlenmedi -- tek-yanlilik + deadzone isini yapti),
    # (2) KAPANMA BEDELI YOK (aktif karelerde u1 20.31 vs pasif 17.92).
    #
    # TAVAN VE TAU NEDEN DEGISMEDI (offline cozucu taramasi, eps
    # dagiliminin GERCEK bandinda: 4.6 / 6 / 7.5 / 9 / 11 / 13.6 deg):
    #     kol                aktif/6   u1 kaybi ort/max   tirmanma ort
    #     r10,t8,tau1.5 (tur-3)   2      0.36 / 2.14          0.13
    #     r5, t8,tau1.5           5      0.85 / 3.98          0.60
    #     r5, t8,tau2.0           5      0.63 / 3.52          0.48
    #     r5,t10,tau2.0           5      0.63 / 3.52          0.48
    #     r5, t6,tau2.0           5      0.63 / 3.52          0.48
    # TAVAN bu bantta BAGLAYICI DEGIL: ref = (eps-5)/tau en fazla
    # (13.6-5)/1.5 = 5.7 dps, yani 6/8/10 tavanlari ayni cozumu verir
    # (tablodaki uc tau=2.0 satiri BIREBIR ayni). Degistirmek bos
    # degisiklik olurdu. TAU 1.5 -> 2.0 u1 bedelini %26 azaltip
    # tirmanmayi %20 kisiyor; SIM tur-3'te kapanma bedeli ZATEN SIFIR
    # olculdugu icin (u1 aktif 20.31 > pasif 17.92) o takasa ihtiyac
    # yok ve tirmanma yetkisi asil aradigimiz sey. Ustelik tek turda
    # tek knob degistirmek bir sonraki sim kosusunu yorumlanabilir
    # kilar: ust-kenar orani duserse sebebi TEK sayidir.
    # ASIL FIKIR (kullanici, tur-3): ust-kenar kaybinin KOK NEDENI
    # STANDOFF DIKEY OFSETI. Avci hedefin ~4-6 m ALTINDA ucuyor
    # (standoff down); kamera hedefi eksenin USTUNDE gorur (beta<0) ve
    # menzil kapandikca gorunen yukselis eps = asin(down/r) BUYUR --
    # 12 m'de 19.5 deg, 8 m'de 30 deg, dikey yari-FOV 20.07'yi ASAR.
    # Bant acmak (VURUS'un mevcut yaptigi) bu kismi KURTARAMAZ; sim'de
    # ust-kenar kaybinin %10.9'u FIZIKSEL FOV DISINDAYDI.
    #
    # COZUM (SAF GUDUM, standoff_geom / SDF'e DOKUNMADAN): VURUS
    # fazinda, hedef gorunen yukselisi RAHAT bandi asinca avciyi
    # hedef HATTINA dogru TIRMANDIR. Boylece terminalde dikey standoff
    # erir, kamera duzlesir, hedef ust kenardan cikmaz; duz kamera
    # gorsel titremeyi de dusurur (govde hareketi = kamera hareketi).
    #
    # MEKANIZMA: dikey ATALETSEL LOS HIZI referansina BIAS. Nominalde
    # o terim sigma_el (dikey LOS hizi) -> 0 surer (paralel seyir =
    # standoff korunur). VURUS'ta referans:
    #     sigma_el_ref = vurus * clip((eps - rahat) / tau, 0, tavan)
    # eps>rahat iken POZITIF sigma_el istemek eps'i kapatir
    # (eps_dot = -sigma_el). SAF GEOMETRI: yalnizca eps = f(ey, aim)
    # kullanir (ey goruntuden), hedef telemetrisine DOKUNMAZ.
    #
    # NEDEN DEADZONE (rahat_deg): terminalde tirmanmak, |v|<=tavan
    # kuresinden dikey bilesen ISTER, yani ILERI KAPANMAYI yer.
    # Olculdu (tek atis, eps=13 deg, deadzone yokken): u1 35 -> 30 m/s.
    # Bu bedeli YALNIZ hedef gercekten kenara yaklasinca odemek icin
    # bias eps <= rahat (10 deg, dikey yari-FOV'un yarisi) iken SIFIR;
    # ancak hedef 10 deg'in otesine cikinca (kenara 10 deg kala)
    # kademeli devreye girer. rahat < 8 deg'de kapanma bedeli
    # yayginlasir, > 14 deg'de hedef kenardan cikmadan mudahale
    # yetismez; 10 ikisinin ortasi.
    # tau 1.5 s, tavan 8 dps: eps=19 (kenar) iken (19-10)/1.5 = 6 dps
    # dikey LOS hizi ister -- olcek 10 dps'nin altinda, dikey kanali
    # doyurmaz. eps=22'de tavana (8) oturur.
    # DIKKAT (tek yanli): yalniz eps>rahat'ta (hedef eksenin USTUNDE
    # ve kenara yakin) tirmanma ister; alcalma ASLA zorlanmaz.
    #
    # OFFLINE/SIM GAP (ONEMLI): offline motor terminal fazina eps < 0
    # ile giriyor (hedef ZATEN duzlesmis: -1.6..-5.0 deg olculdu),
    # cunku benzetimin dikey dinamigi sim'inkiyle AYNI DEGIL -- sim'de
    # avci standoff'ta ~4 m altta KALIYOR (ust-kenar kaybi oradan),
    # offline'da terminale kadar hizaya tirmaniyor. Yani bu mekanizma
    # OFFLINE KAPALI DONGUDE OLCULEMEZ (tum senaryolarda bias 0 kalir,
    # regresyon riski de yok). Cozucu SEVIYESINDE deterministik olarak
    # test edilir (test_vurus_dikey_hizalama); KAPALI DONGU dogrulamasi
    # SIM'e aittir. Bu, tur-2'de kullanicinin isaret ettigi
    # offline->sim ayrisimasinin ta kendisidir.
    # ---- IRTIFA-AGNOSTIK TERMINAL DIKEY HIZALAMA (2026-08-07) -------
    # A/B DUGMESI: YILDIZ_DIKEY_TERMINAL=1. VARSAYILAN KAPALI.
    # Kok neden, olcum ve emniyet gerekcesi: bkz. cevre_dikey_terminal().
    # Kisaca: FAZ C (tilt eps'i izliyor) beta'yi ~0'a cektigi icin maliyet
    # dikey standoff'u ARTIK GORMUYOR; PN terimi de "paralel seyir" yani
    # "standoff'u KORU" demek. Bu dugme dikey LOS hizi referansina, eps'i
    # kapatan IKI YANLI bir bias koyar.
    dikey_terminal: bool = field(
        default_factory=lambda: cevre_dikey_terminal() > 0.0)
    # MENZIL RAMPASI. 45 m = terminal_menzil_m (durum makinesinin TERMINAL
    # esigi) ve iska_arm_m ile ayni sayi -- yeni bir faz tanimi UYDURMUYORUZ,
    # var olan terminal tanimini kullaniyoruz. 25 m'de bias TAM guclenir.
    # NEDEN 45 -> 25 arasi: kuyruk takibinde kapanma ~14 m/s, yani bu bant
    # ~1.4 s surer ve 25 m'den CPA'ya ~1.8 s daha vardir. Toplam ~3 s'lik
    # pencere, 4 m'lik dikey ofsete ~1.3 m/s tirmanma demektir -- tirmanma
    # tavaninin (9 m/s) yedide biri, yani ILERI KAPANMADAN calinan hiz
    # ihmal edilebilir. Eski (emekli) mekanizma VURUS'a bagliydi: 22 m'de
    # baslar, 8 m'de doyar -> ~1 s. "Son saniye" tam olarak oydu.
    dikey_terminal_menzil_m: float = 45.0
    dikey_terminal_tam_m: float = 25.0
    # OLU BANT (iki yanli). Eski mekanizmada 5 deg idi ve gerekcesi
    # "kadraj kenari"ydi (hedef ust kenardan cikmasin). Burada amac kadraj
    # DEGIL CARPMA GEOMETRISI, o yuzden esik cok daha dar: 2 deg. Anlami
    # dz = r*sin(2 deg) -- 45 m'de 1.6 m, 25 m'de 0.9 m, 10 m'de 0.35 m.
    # Yani "2 deg olu bant" pratikte "CPA'da 0.3-0.4 m dikey artiga izin
    # var" demektir; kabul olcutumuz (|vert| < 0.4 m) ile ayni mertebede.
    # Daha darina inmek eps olcum gurultusunu (bbox +-1-2 px) dikey hiz
    # komutuna cevirir -- 1 deg = 25 m'de 0.44 m, olcum belirsizliginin ta
    # kendisi.
    dikey_terminal_rahat_deg: float = 2.0
    # TAU: "eps'i kac saniyede kapat". eps_dot = -sigma_el oldugu icin
    # sigma_el_ref = eps/tau birinci mertebeden bir kapanmadir. 1.5 s,
    # 25 m'deki t_go (~1.8 s) ile ayni mertebe -- daha hizli istemek
    # (tau<1) ileri kapanmayi yer, daha yavasi (tau>2.5) CPA'ya yetismez.
    # Emekli mekanizmanin tau'su da 1.5 idi ve offline taramada bu bantta
    # kapanma bedeli olculmemisti.
    dikey_terminal_tau_s: float = 1.5
    # TAVAN: dikey LOS hizi talebi ust siniri. Gerekli dikey hiz
    # w3 = ref * r / 57.3 oldugu icin 8 dps, 45 m'de 6.3 m/s, 25 m'de
    # 3.5 m/s eder. Alcalma tarafinda fiziksel tavan zaten 4.5 m/s ve
    # girdi kutusu onu kirpar; tirmanma tarafinda 9 m/s. Yani tavan
    # FIZIKSEL kutunun ONUNDE degil, gurultuye karsi bir emniyet supabi.
    dikey_terminal_tavan_dps: float = 8.0

    # ---- DOGRUDAN DIKEY HATA (P) TERIMI (2026-08-07) ----------------
    # A/B DUGMESI: YILDIZ_DIKEY_HATA. VARSAYILAN ACIK (1.0).
    # Kok neden, geometri ve emniyet gerekcesi: bkz. cevre_dikey_hata().
    # Kisaca: yukaridaki dikey_terminal bir TUREV (D) terimidir; hatanin
    # KENDISI icin maliyette satir yok. Bu dugme ey durumu uzerine,
    # "hedef hatti" (eps = 0) referansli DOGRUDAN bir hata satiri ekler.
    # Ikisi BAGIMSIZ acilir; birlikte acilinca P+D olur.
    dikey_hata: bool = field(
        default_factory=lambda: cevre_dikey_hata() > 0.0)
    # ENV CARPANI (YILDIZ_DIKEY_HATA degeri). 1 = nominal q_dikey_hata.
    dikey_hata_carpani: float = field(
        default_factory=lambda: max(cevre_dikey_hata(), 0.0) or 1.0)
    # AGIRLIK. ILK DENEME q_ey ile ayni mertebeydi (0.60) ve SIM'DE
    # OLCULDU KI YETERSIZ: elips/DOWN+4 kosusunda terim karelerin
    # %91.5'inde ACIKTI ama CPA'da dikey artik 2.02 -> 1.78 m, yani
    # neredeyse hic degismedi (dhP_elips_20260807_045603).
    # NEDEN: bu terim, ayni durum uzerinde PN terimiyle (q_los_hiz=4.0)
    # YARISIYOR ve PN'in anlami tam olarak "dikey standoff'u KORU"dur.
    # Cozucu ufkunda ey_k ~ ey_0 - c3*w3*t_k oldugundan iki terimin
    # gradyanlarini esitlemek dengedeki dikey hizi verir:
    #     w3 ~ q_dikey_hata * e_0 * SUM(t_k) / (q_los_hiz * c3 * N)
    # (olcekler esit: olcek_ey_deg = olcek_los_hiz_dps = 10, sadelesir).
    # GEREKEN dikey hiz iki parcalidir ve ikisi de r kapandikca buyur:
    #   (1) eps'i SABIT tutmak bile dz/r * kapanma kadar tirmanma ister
    #       -- dz 4 m, r 25 m, kapanma 14 m/s -> 2.24 m/s,
    #   (2) dz'yi CPA'ya kadar ERITMEK dz/t_go kadar daha ister
    #       -- 4 m / 1.8 s -> 2.2 m/s.
    # Yani terminalde ~2.2-2.8 m/s mertebesi gerekiyor. Cozucu taramasi
    # (eps=+8 deg, r=25 m): q=0.6 -> u3 1.63 m/s, q=1.5 -> 2.46 m/s,
    # q=2.0 -> 2.81, q=4.0 -> 3.97 (q>4'te doyuyor).
    # 1.5 SECILDI cunku KAPALI DONGUDE ISE YARADIGI OLCULEN kol olan
    # dikey_terminal (TUREV) ayni noktada 2.22 m/s uretiyor: yani P
    # terimine, calistigi bilinen mekanizmayla AYNI YETKI veriliyor --
    # yeni bir agresiflik seviyesi UYDURULMUYOR.
    # Rampayla ve env carpaniyla olceklenir: 45 m'de 0, 25 m'de tam.
    q_dikey_hata: float = 1.5
    # MENZIL RAMPASI: dikey_terminal ile AYNI sayilar (45 -> 25 m).
    # NEDEN AYNI: 45 m durum makinesinin TERMINAL esigi ve iska_arm_m ile
    # ayni sayidir; yeni bir faz tanimi UYDURMUYORUZ. 25 m'de terim tam
    # guclenir ve CPA'ya ~1.8 s kalir. Ayrica iki dugme birlikte
    # acildiginda (P+D) ikisinin de ayni pencerede calismasi, kolun
    # yorumunu tek degiskene indirger.
    dikey_hata_menzil_m: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_DIKEY_RAMPA_BAS', 45.0))
    dikey_hata_tam_m: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_DIKEY_RAMPA_SON', 25.0))
    # *** RAMPA PENCERESI ARTIK ENV-AYARLI (2026-08-09, dz analizi). ***
    # VARSAYILANLAR DEGISMEDI (45 -> 25), yani env verilmezse BIT-AYNI.
    # NEDEN AYARLANABILIR OLDU -- dz<0 kok neden analizi (ayri ajan):
    #   (1) DIKEY KANAL OLU ZAMANI ~1.5 s: olculen tau_lin_z 2.16 s, model
    #       1.0 saniyeyi varsayiyor; CPA oncesi talebin ICRA orani %11.
    #   (2) eps bir ACIDIR: dz sabit kalsa bile r kuculdukce eps buyur,
    #       yani talep DOGAL OLARAK SON SANIYEDE dogar ve tavana vurur.
    #       45 -> 25 m penceresi bu yuzden COK GEC: R=30 m'de CPA'ya
    #       4-6.7 s varken rampa daha yeni basliyor, R=15 m'de CPA'ya
    #       0.4-1.4 s kaliyor -- OLU ZAMANDAN KISA. Talep dogdugunda
    #       icra edecek zaman kalmiyor.
    # DEGISEN SEY GENLIK DEGIL *ZAMANLAMA*: rampayi one almak (or.
    # 80 -> 45 m) talebi olu zamandan ONCE dogurur, boylece ayni genlik
    # yumusak yayilarak ICRA EDILEBILIR.
    # dv1 paketinde YILDIZ_DIKEY_RAMPA_BAS=80 YILDIZ_DIKEY_RAMPA_SON=45
    # ile birlikte YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5 verilir: dogru
    # tau, erken dogan talebi yumusak yaymak icin GEREKLIDIR (eyl2'de
    # tau TEK BASINA basarisizdi cunku talep hala gec doguyordu).
    # OLU BANT (iki yanli, AGIRLIGI sifirlar -- referansi kaydirmaz).
    # 1.0 deg secildi: dz = r*sin(1 deg) demek, 25 m'de 0.44 m, 15 m'de
    # 0.26 m, 10 m'de 0.17 m. Kabul olcutu CPA'da |dikey| < 0.4 m
    # oldugu icin bu bant CPA yakininda olcutun ALTINDA kalir, uzakta ise
    # bbox gurultusunu (+-1-2 px ~ 0.1-0.2 deg) rahatca yutar.
    # dikey_terminal'in olu bandi (2.0 deg) daha genis cunku o TUREV
    # terimi hiz komutuna dogrudan cevriliyor; buradaki P terimi cozucu
    # icinde diger maliyetlerle yarisiyor, o yuzden daha dar tutulabilir.
    dikey_hata_rahat_deg: float = 1.0

    # ---- t_go SEKILLI DIKEY HIZ REFERANSI (2026-08-07, tur-3) -------
    # A/B DUGMESI: YILDIZ_DIKEY_TGO. VARSAYILAN ACIK (2.0).
    # Kok neden, kapali-form gerekce ve emniyet: bkz. cevre_dikey_tgo().
    # Kisaca: D kolunun (dikey_terminal) SABIT tau'su terminalde t_go'dan
    # uzun kaliyor; bu dugme tau'yu KALAN ZAMANA baglar:
    #     tau_eff = clip(t_go / k, tau_min, tau_max)
    # ve boylece hem hatayi hem HIZINI birlikte sifira goturur.
    # DIKEY_TERMINAL'den BAGIMSIZ ACILIR: bu dugme tek basina da ayni
    # hiz-referansi satirini devreye sokar (ana kol P + TGO'dur).
    dikey_tgo: bool = field(
        default_factory=lambda: cevre_dikey_tgo() > 0.0)
    # ENV CARPANI (YILDIZ_DIKEY_TGO degeri). 1 = nominal k.
    dikey_tgo_carpani: float = field(
        default_factory=lambda: max(cevre_dikey_tgo(), 0.0) or 1.0)
    # k -- "kalan zamanin kacta birinde hatayi kapat". Kapali formda
    # e(t) = e0 * (t_go/T)^k oldugu icin k'nin anlami dogrudan terminal
    # profilin USTELIDIR:
    #   k = 1   : e_dot SABIT kalir -> ASMA (bugunku hatanin ta kendisi)
    #   k = 2   : e_dot ~ t_go dogrusal sifirlanir
    #   k = 3   : e_dot ~ t_go^2, dikey artik dz ~ t_go^4
    #   k >> 3  : hata uzun sure DURUR, son anda erimek zorunda kalir;
    #             kazanc doyar ama profil "GEC VE SERT" olur.
    #
    # OFFLINE (cozucu-dongusu taramasi, P + TGO, DOWN=+4, eps0=+5.1 deg,
    # hedef 21 m/s DUZ ve SEVIYELI) DIKEY HIZ @CPA'yi verdi:
    #   k=2.0 -3.18 | k=2.5 -0.99 | k=2.6 -0.71 | k=2.8 -0.56
    #   k=3.0 -0.52 | k=3.5 -0.45 | k=4.0 -0.40 | k=6.0 -0.32  [m/s]
    # (referans: P tek basina -5.35, kapali kol -5.76 m/s.) Offline DIZ
    # 2.5-2.8 arasinda; offline'a bakarak k=3.0 secilmisti.
    #
    # *** KAPALI DONGU BUNU CURUTTU -- k=2.0 SECILDI. ***
    # SIM A/B (SURE=300, elips DOWN+4), CPA'daki dikey artik [m]:
    #   kapali kol : 2.02 / 1.03      P tek basina : 1.10
    #   k=3.0      : 0.58 / 0.64      k=2.0        : 0.41 / 0.15
    # ve k=2.0 UC ROTADA DA (elips DOWN+4, duz DOWN+4, elips DOWN=0)
    # GERCEK FIZIKSEL TEMAS uretti (vurus_basarili, vibe 234/40/24;
    # butun kampanyada baska hicbir kol temas uretmedi).
    # NEDEN OFFLINE YANILDI (kayit icin onemli): offline benzetimin
    # basarisizlik kipi ASMA'ydi, orada buyuk k odullenir. GERCEK
    # kosuda -- ozellikle hedef DONERKEN -- basarisizlik kipi TERSI:
    # yasa GEC kapatiyor. k=3.0 kolunun imzasi tam da bu: artik ISARETI
    # + (yetisemiyor), |dz| 15-25 m bandinda 2.77 m'de TUTULUYOR ve son
    # 1.3 s'de dalisa geciyor. k=2.0 kapatmayi ERKENE ceker: ayni bantta
    # 8-15 m |dz| 2.95 -> 0.08 m. Yani k, "asma" ile "gec kalma"
    # arasindaki takastir ve GERCEK geometride optimum DAHA KUCUKTUR.
    # (Ayni offline->sim ayrisimasi tur-2'de de yasandi; kural: offline
    # tarama YON verir, KARARI kapali dongu verir.)
    # NOT: env CARPANDIR; bu satir degistigi icin ESKI kosu etiketleri
    # kayar -- k=3.0 kolunu tekrar uretmek icin YILDIZ_DIKEY_TGO=1.5.
    # Kapanma bedeli OLCULDU ve YOK (offline CPA'ya varis her kolda
    # 3.15-3.20 s; sim'de yatay CPA elipste 0.96/1.84 -> 0.24/0.21 m,
    # yani IYILESTI).
    #
    # *** TUR-5 KAPALI DONGU HUKMU: k=2.0 ONAYLANDI (n=6 vs n=6). ***
    # Yukaridaki secim n=1-2 ile yapilmisti ve KARAR VERILEBILIR DEGILDI:
    # ayni konfigurasyonun iki kosusu 15-8 m bandinda 0.19 ve 2.78 m
    # vermisti, yani KOSU-ICI yayilim HUCRELER-ARASI farktan buyuktu.
    # Tur-5'te elips DOWN+4 hucresi her kolda n=6'ya cikarildi (3 normal
    # cozucu butcesi + 3 YILDIZ_COZUCU_BOL=1; iki kol da ayni sekilde
    # dengelendigi icin butce bir ORTAK ETKEN, kollari ayirmiyor).
    #   metrik [ortanca (min..max)]        k=2.0            k=3.0
    #   bant |dz| 25-15 m              1.88 (0.82-2.98)  2.22 (2.00-2.50)
    #   bant |dz| 15-8 m               1.23 (0.19-2.78)  1.81 (0.61-2.78)
    #   bant |dz| 8-0 m                1.21 (0.50-1.39)  1.17 (0.57-1.79)
    #   CPA dikey |artik|              0.35 (0.09-1.59)  0.51 (0.03-1.74)
    #   yakin-CPA |dikey| ortancasi    0.58 (0.09-1.43)  0.86 (0.37-1.89)
    #   CPA yatay                      0.73 (0.37-2.04)  1.36 (0.25-2.04)
    #   ISKA                           1.5  (1-3)        3.0  (1-5)
    #   FIZIKSEL TEMAS                 2/6               0/6
    # k=2 alti metrikte ustun, k=3 yalniz 8-0 m bandinda ve o da
    # ihmal edilebilir farkla (1.17 vs 1.21). Ikinci hucre (duz DOWN+4,
    # n=2+2, BOL) ayni yone bakiyor: 8-0 bandi 0.95 vs 1.54, CPA yatay
    # 1.68 vs 2.81, ISKA 4 vs 7 -- hepsi k=2 lehine.
    # DURUSTLUK NOTU: hicbir DIKEY metrikte aralikler AYRIK DEGIL; karar
    # ORTANCALARIN tutarli yonune + temas sayisina dayaniyor. Kampanyanin
    # butun fiziksel temaslari (simdi 4 adet) k=2 kolundan geldi, k=3
    # hicbir kosuda vibe > 4 uretmedi.
    dikey_tgo_k: float = 2.0
    # TAU TABANI. t_go -> 0'da tau_eff -> 0 ve talep patlar; taban bunu
    # keser. 0.30 s secildi cunku (a) eyleyici zaman sabiti 1.0 s,
    # dolayisiyla 0.3 s'den kisa bir dikey hiz talebi zaten TAKIP
    # EDILEMEZ -- daha kucugu yalnizca gurultu uretir; (b) t_go = 0.60 s
    # (k=2.0) altinda devreye girer, ki orasi r < 9 m demektir ve orada
    # kapali formda hata zaten ~(t_go/T)^2 ile ihmal edilebilir
    # kucuklukte olmalidir. Yani taban NORMAL kosuda HIC baglayici
    # olmamali; baglayici olursa yasa is gormemis demektir (tanidir).
    # DUYARLILIK OLCULDU (offline, k=3.0, DOWN=+4): tau_min
    # 0.15/0.30/0.45/0.60 -> dikey hiz @CPA -0.42/-0.52/-0.61/-0.71 m/s.
    # Yani 0.30 civari DUZ bir bolgedir, kritik bir ayar degildir.
    # Sim'de de baglayici DEGIL: kosularda tau tabanina yapisan kare
    # orani %2-12 (k=2.0 kolunda %4-11), yani yasa neredeyse hep
    # t_go'nun kendisiyle calisiyor.
    dikey_tgo_tau_min_s: float = 0.30
    # TAU TAVANI = dikey_terminal_tau_s (1.5 s) ile AYNI SAYI. Boylece
    # bu kol UZAKTA (ya da t_go gecersizken) eski sabit-tau kolundan
    # daha AGRESIF OLAMAZ; yalnizca terminale dogru sertlesir. Yeni bir
    # agresiflik seviyesi uydurulmuyor.
    dikey_tgo_tau_max_s: float = 1.50
    # t_go GECERLILIK KAPISI. menzil_hizi LPF'lidir (tau 0.30 s) ve
    # capraz geometride kapanma ~0'a duser -> t_go patlar. 1.0 m/s
    # altindaki kapanmada t_go YOK sayilir ve tau_max'a (en yumusak)
    # dusulur. NEDEN 1.0: 45 m'de 1 m/s kapanma t_go = 45 s demektir,
    # zaten tavani (dikey_tgo_tavan_s) asar; esik pratikte "kapaniyor
    # muyuz" sorusunun isaret testidir, bir ayar degil.
    dikey_tgo_kapanma_min_mps: float = 1.0
    # t_go TAVANI. Yasa yalnizca 45 m icinde (dikey_s rampasi) calisir
    # ve orada gercekci t_go 3-4 s'dir. 6.0 s ustu degerler LPF
    # isinmasi/gecici acilma demektir; kirpmak tau_eff'i tau_max'a
    # oturtur, yani yine en yumusak dala duser.
    dikey_tgo_tavan_s: float = 6.0

    # ---- VURUS BASARISI TESPITI (2026-08-05, tur-4) -----------------
    vurus_basari_tespiti: bool = True
    vurus_basari_vibe: float = 15.0
    vurus_basari_menzil_m: float = 3.0
    # NEDEN VAR (OLCUM SORUNU, kullanici istegi): kullanici TEKRARLANABILIR
    # carpma istiyor ama her BASARILI vurus kosuyu BITIRIYOR -- arac hedefe
    # carpip takla atiyor ve dusuyor (tur-3: menzil 1.04 m'de vibe 25.5,
    # ardindan roll -106 deg / pitch -51.8 deg). Yani bir kosudan bir
    # ornek cikiyor ve n=5'te kaliyor; istatistiksel guc yok. Ustelik
    # "vurduk mu" sorusu simdiye kadar CPA TAHMININDEN cikariliyordu.
    #
    # TESPIT: vibe > vurus_basari_vibe VE menzil < vurus_basari_menzil_m.
    # Esikler tur-3 kosusunun OLCULEN degerlerinden:
    #   * GERCEK TEMAS  : 2.32 m'de vibe 2.0 -> 17.4, 1.04 m'de 25.5
    #   * TEMASSIZ GECIS: tur-2'de 0.85 m'de vibe yalnizca 3.3
    # 15.0 iki kumenin arasinda, temassiz gecise 4.5x pay birakir.
    # MENZIL KAPISI SART: LOG_SOZLUGU'ne gore YER TEMASI vibe'i 150-345'e
    # firlatir; r < 3 m sarti onu eler (hedefin 3 m yakininda yere
    # carpmak, hedefe carpmakla ayni sey degildir ama pratikte
    # ayirt edilemez -- yine de irtifa zaten 25 m tabaniyla korunuyor).
    #
    # KURAL NOTU: vibe BIZIM telemetrimizdir (VIBRATION mesaji), hedefin
    # DEGIL. "Hedeften yalniz menzil" kurali ihlal edilmiyor; zaten
    # tespitin ikinci girdisi de menzilin ta kendisi.
    # LATCH'li: angajman basina bir kez ilan edilir (sifirla() temizler),
    # yoksa temas suresince her dongu olay yazilirdi.
    # ---- KOR SUZULMEDE PN SURDURME (env dugmeli, varsayilan ACIK) ----
    # Kok neden, mekanizma ve R1-R5 raylari: bkz. cevre_kor_pn().
    kor_pn: bool = field(default_factory=lambda: cevre_kor_pn() > 0.0)
    kor_pn_azami_s: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_KOR_PN_AZAMI_S', 1.0))
    kor_pn_tau_s: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_KOR_PN_TAU', 0.7))
    kor_pn_ex_pay_deg: float = 5.0
    kor_pn_ex_mutlak_deg: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_KOR_PN_EX_MUTLAK', 26.0))
    # R3' MUTLAK KADRAJ KELEPCESI (2026-08-09, kpn1b+kpn2 olcumu).
    # R3 tek basina YETMEDI: kelepce tabani "son GERCEK |ex|" idi ve
    # DONUSTE o zaten 30+ deg oluyor, yani taban kayiyordu. Olculdu --
    # kor karelerde |ex| p90: kpn1b 22.9, kpn2 38.6 deg. Kadraj YARI-ACISI
    # 20.07 deg. Yani olu hesap hedefi kadrajin CO K DISINA "hayali"
    # surukluyor ve yasa o hayaleti kovaliyordu; donus fazi bu yuzden
    # bozuldu (kapanma +3.87 -> -7.93/-2.89, taze %74 -> %46/%56).
    # DUZELTME: |ex| <= min(|ex0| + pay, kor_pn_ex_mutlak_deg).
    #
    # *** VARSAYILAN 20 -> 26 (2026-08-09, ucus kaniti). *** 20, kacagi
    # kesiyordu ama MESRU terminal tasmalarini da kirpiyordu. 26 ile
    # olculen (angajman havuzu, ORTAK KATMAN kapanma0>=3):
    #     tavan 20 (3 kosu, n=21): CPA3B p50 5.99 | YATAY p50 4.66 | <3m 3 (%14)
    #     tavan 26 (2 kosu, n=10): CPA3B p50 3.50 | YATAY p50 2.77 | <3m 4 (%40)
    #   -> medyan CPA -%42, yatay bilesen -%41, <3m orani 3 KATI.
    #   DONUS bandi korundu (kapanma > 0, acilan < %50) ve kor |ex| MAX
    #   tam 26.0'da civili kaldi (kacak YOK).
    # ACIK KALAN: sub-metre kuyrugu (0.69-0.92 m gecisler) GERI GELMEDI --
    # en iyi gecis 1.08 -> 1.22. Kelepce medyani duzeltti ama kuyrugu
    # aciklamiyor; ayri soru olarak TO_TEST'te.
    # GEREKCE: kadraj disinda hedefin NEREDE oldugu BILINMIYOR; olu
    # hesabin oraya "emin" gitmesi bilgi degil uydurmadir. Kenarda
    # DURSUN, disari cikmasin.
    kor_pn_menzil_m: float = field(
        default_factory=lambda: _cevre_sayi('YILDIZ_KOR_PN_MENZIL', 12.0))
    # MENZIL KAPISI: r_ic bu degerin USTUNDEYSE ilerletme YAPILMAZ (eski
    # donmus-komut davranisi). Veri: kolun KAZANCI terminalde olusuyor
    # (kor_pn=1 karelerinde r_ic p50 kpn1b 3.4 m, kpn2 8.9 m; en yakin
    # CPA 1.44 -> 0.84/0.75 m), KAYBI ise uzak menzildeki kor karelerden
    # geliyor (donus fazi). 12 m, VURUS menzilinin (8 m) biraz ustu --
    # terminal penceresini tam kapsar, uzak menzili disarida birakir.

    vurus_kor_suzulme: bool = True
    vurus_kor_menzil_m: float = 8.0
    # KOR SUZULME (coast). VURUS fazindayken, menzil vurus_kor_menzil_m
    # ICINDEYKEN ve bbox BAYATLAMISSA cozucu KOSMAZ; son komut aynen
    # tekrarlanir. Gerekce: bayat bbox'ta ey DONAR ve kadraj kisiti
    # kendi urettigi dikey hizi olcup daha da sertlesir (olculdu,
    # elips blok 2: ey -14.57'de donmusken beta 1.4 s'de 4.96 ->
    # 37.42). 8 m'nin icinde t_go < 0.5 s'dir: plan zaten kurulmustur,
    # yeni bir manevranin duzeltebilecegi bir sey yoktur ve kamera
    # 0 deg SABIT oldugu icin zaten kor kaldik.
    #
    # MENZIL KAPISI OLCUMLE EKLENDI (2026-08-05). Ilk surumde kapi
    # YALNIZ bayatliktı (tum VURUS fazi boyunca). Kapali dongude
    # OLCULDU ki bu YAKALAMAYI BOZUYOR:
    #     viraj/capraz  min 2.47 -> 9.51 m,  keskin/capraz 3.81 -> 12.97
    # Sebep: iskelet bbox'i 0.70 s'ye kadar TAZE sayar ve kontrolcuyu
    # cagirmaya devam eder; bizim bayat_kisit_s kapimiz 0.30 s. Yani
    # 0.30-0.70 s araligindaki HER bosluk suzulmeye ceviriliyordu --
    # 35 m/s'de 14 m'lik komutsuz ucus, hem de geometrinin en hizli
    # degistigi anda. O aralikta DOGRU cevap zaten vardi: sert kisiti
    # birak (bayat_kisit_s kapisi), cozmeye DEVAM et. Kor suzulme
    # yalnizca duzeltmenin fiziksel olarak anlamsiz oldugu son 8 m'de
    # devrededir. 0.70 s'yi asan gercek kayipta zaten iskelet son
    # komutu 1 s tutar (bosluk_tut_s) -- bu, kor suzulmenin dogal
    # devamidir. ISKA durum makinesi (menzil taniki bbox'tan BAGIMSIZ,
    # estimator'dan gelir) suzulmeyi her halukarda sonlandirir.

    # ---- ISKA DURUM MAKINESI (2026-08-05) --------------------------
    # NEDEN MALIYET FONKSIYONUNDA DEGIL: MPC "gectim, birakmaliyim"
    # durumunu optimizasyonla COZEMEZ. Menzil acilirken bile hedefe
    # dogru komut uretmek maliyet acisindan DOGRUDUR (LOS hizi -> 0,
    # alan odulu, kadraj -- hepsi hala hedefi gostermeye devam eder).
    # Yani "birak" bir optimum degil, bir SONLANDIRMA KARARIDIR; MPC'nin
    # USTUNDE bir durum makinesi olarak kurulur:
    #     KAPANMA -> TERMINAL -> ISKA -> (yetkiyi birak)
    #
    # OLCULEN SORUN (statik hedef, mpc_tani_20260804_220521 /
    # _223143 / _223930, 17 yetki segmenti):
    #   * Her segmentte ilk terminal gecis t+4..5 s'de oluyor
    #     (CPA 0.9-17 m), sonra kontrolcu KOMUT VERMEYE DEVAM EDIYOR.
    #   * Ardindan menzil 220 m'ye kadar ACILIP geri kapaniyor:
    #     220521 seg0 profili 27 -> 15 (t+4) -> 220 (t+21) -> 3 (t+35).
    #     Yani MPC yeniden angaje OLUYOR ama 30 s suren, 110 m
    #     yaricapli DEV BIR DAIRE ile.
    #   * FIZIK: 18 m/s ve WPNAV_ACCEL 5 m/s^2 ile en kucuk donus
    #     yaricapi v^2/a = 65 m; U donusu 130 m yer degistirme, yani
    #     ~11 s YALNIZ DONUS. Hiz tavani 35 m/s'e cikinca yaricap
    #     245 m'ye firlar -- yani gecis sonrasi kendi basina yeniden
    #     angajman HIZLA IMKANSIZLASIR. Ucuz olan yol: yetkiyi birak,
    #     konumlu gudum (tam telemetriyle) standoff'a yeniden
    #     konumlansin ve yeniden devretsin.
    # Bu makinenin KAZANCI (ayni 17 segmentte cevrimdisi tekrar
    # oynatildi): bosa ucus 4.4-64.4 s kisaliyor, ortalama ~28 s.
    iska_modu: bool = True
    terminal_menzil_m: float = 45.0
    # Bu menzilin ICINE girildiyse durum TERMINAL olur (referans:
    # formation_KILLER.ATTACK_TERMINAL_M = 45, gercek ucusta ayarli).
    # Bizde devir kapisi 60 m (bbox_to_redis.GECIS_MENZIL_M) oldugu
    # icin 45 m gercek bir KAPANMA fazi birakir: 45-60 m'de devralinan
    # angajmanlar once KAPANMA'da olur. TERMINAL burada YALNIZ bir
    # durum etiketidir -- kontrol yasasini DEGISTIRMEZ (MPC'nin lineer
    # alan odulu terminal agresifligi zaten kendisi uretiyor).
    iska_arm_m: float = 45.0
    iska_acilma_m: float = 30.0
    # KLASIK IKILI (referans: ATTACK_ABORT_ARM_M=60 / OPEN_M=30).
    # ARM 60 -> 45 DUSURULDU cunku KILLER'da saldirgan 120-150 m'de
    # angaje oluyor ve "60 m" gercekten yaklastigin kanitiydi; BIZDE
    # yetkinin kendisi zaten <=60 m'de devrediliyor, yani 60'lik bir
    # ARM her devirde t=0'da dolardi ve "yaklastik" bilgisi tasimazdi.
    # 45 m, olculen devir menzil bandinin (26.8-33.5 m) ustunde ama
    # devir kapisinin (60) altinda: kural ancak GERCEKTEN kapandiktan
    # sonra silahlanir.
    # OPEN 30 aynen korundu: 25-30 m/s kapanma hizinda ~1 s, menzil
    # olcum gurultusunun (sigma ~1.2 m sim) 25 katindan buyuk.
    iska_gecis_arm_m: float = 12.0
    iska_gecis_acilma_m: float = 8.0
    # GECIS (pass) KOLU -- klasik ikilinin HIZLI kisayolu. Gecis
    # onaylandiysa 30 m acilma beklemek gereksiz: 8 m yeter.
    # gecis_arm 20 -> 12 m (2026-08-05, 35 m/s turu). ESKI GEREKCE:
    # gercek terminal geciste CPA 0.92 / 2.19 / 7.27 / 8.76 / 9.5 /
    # 11.0 m, devir menzili 26.8-33.5 m; 20 m ikisini ayiriyordu.
    # YENI GEREKCE: ayirmasi gereken UCUNCU kume var -- ORTA SAFHA
    # SALINIMI (olculen vaka: 18.1 m'de -12.4 m/s ile frenleyip
    # doner, 42 m'ye acilir, sonra GERI DONUP 0.43 m'de vurur). Bu
    # kume 18 m/s tavaninda KAPANMA HIZIYLA ayriliyordu
    # (gecis_kapanma_esigi 15 m/s: gercek gecisler -17.7..-29.0,
    # salinim -12.4). 35 m/s tavaninda o ayrim COKUYOR:
    #   * saf kuyruk takibinde gercek terminal gecis kapanma hizi
    #     35 - 21.05 = 13.9 m/s -- yani ESKI ESIGIN ALTINDA. Olculdu:
    #     gecis_kapanma_esigi=15 kuyruk takibinde 0/16 ates.
    #   * ayni tavanda salinim kolunun fren hizi da buyur.
    # Yani hiz ekseni artik AYIRMIYOR. GEOMETRI AYIRIYOR: gercek
    # gecislerin hepsi CPA <= 11 m, salinim vakasinin CPA'si 18.1 m.
    # Arm cemberi 12 m bu iki kumeyi HIZ TAVANINDAN BAGIMSIZ ayirir
    # (CPA bir uzunluktur, tavanla olceklenmez) ve devir menzilinin
    # (26.8+) hala cok altindadir. Ayrim testi mpc_test 5k(c)'de
    # taramayla korunuyor.
    # gecis_acilma 8 m: onaylanmis CPA'dan sonra menzil kapanma
    # hiziyla (14-56 m/s) acilir, 8 m ~ 0.14-0.57 s. Menzil
    # gurultusunun (sigma ~1.2 m) ~6.7 sigmasi, yani tek ornek
    # sicramasi tetikleyemez.
    iska_mutlak_m: float = 120.0
    # MUTLAK IPTAL (referans ATTACK_ABORT_MAX_M = 250). 120 = devir
    # kapisi 60 m'nin iki kati; goruntulu gudumun doktrin zarfinin
    # disi. Olculdu: iskalanan segmentler 77-243 m'ye kadar yetkiyi
    # tutuyordu. Bu bir ARTIK KORUMADIR -- 45/30 kolu once atesler.
    # BASIT GECIS KURALI ACILIRSA (YILDIZ_GECIS_BASIT=1) BUNU 300
    # YAP: o kural menzil kapisini kaldirdigi icin devir 150+ m'de
    # gelebiliyor ve 120 siniri devri ~1 s'de iptal edip bbox ile
    # ping-pong uretiyor (olculdu: cevrim ~1.2 s, dakikada ~50 devir).
    # 300 = tespit ufku (~200 m) ustune pay.
    iska_zaman_asimi_s: float = 8.0
    # ZAMAN ASIMI (referans ATTACK_TIMEOUT_S = 14). Tek isi
    # "kapanmiyor ama acilmiyor da" (es-hizli kuyruk takibi)
    # kilitlenmesini kirmak: 45/30 kolu orada ASLA atesleyemez.
    # 15 -> 8 s (2026-08-05, 35 m/s turu). Esik bir HIZ olcusudur:
    # devir zarfi 60 m ve saf kuyrukta ULASILABILIR kapanma artik
    # 35 - 21.05 = 13.9 m/s, yani 60 m 4.3 s'de kapanir. 8 s bunun
    # ~2 kati -- gecikmeler, viraj ve oturma icin pay birakir ama
    # "kapanmiyor" durumunu 15 s yerine 8 s'de teshis eder.
    # NEDEN KISALTMAK ONEMLI: 35 m/s'de en kucuk donus yaricapi
    # v^2/a = 245 m (18 m/s'de 65 m). Bosa gecen her saniye 35 m
    # yol ve yeniden konumlanmasi cok daha pahali bir geometri
    # demektir; zaman asimi artik hiz tavaniyla TERS olceklenir.
    # ---- ILERLEME-TABANLI ZAMAN ASIMI (env dugmeli, varsayilan KAPALI)
    # Kok neden, olcum ve tasarim: bkz. cevre_ilerleme_saat().
    ilerleme_saat: bool = field(
        default_factory=lambda: cevre_ilerleme_saat() > 0.0)
    ilerleme_kapanma_esigi_mps: float = 1.0
    # ILERLEME OLCUSU: EN IYI MENZILIN (best-so-far) iyilesme HIZI
    #     (en_iyi_eski - en_iyi_simdi) / pencere > esik
    # bu esigi asiyorsa ILERLIYORUZ.
    #
    # NICIN best-so-far, NICIN anlik/pencereli HAM menzil DEGIL
    # (tasarim sirasinda IKI kez olculdu, ikisi de kandirildi):
    #   * ANLIK menzil_hizi (LPF tau 0.30 s): 1-2 Hz salinimi gecirir.
    #     Sentetik kontrol r(t) = 25 + 2*sin(2t) -- menzil DURAGAN,
    #     yalnizca salinan -- karelerin ~yarisinda "kapanma > 1.0"
    #     verdi; saat 0.5 s/s geri + 1.0 s/s ileri = net ~+0.25 s/s,
    #     8 s'lik durgunluk 32 s'de dolacakti (22 s mutlak tavana
    #     carpti). Tam da kirmak istedigimiz "kapanmiyor ama acilmiyor
    #     da" kilidini SERBEST BIRAKIYORDU.
    #   * PENCERELI HAM menzil (r_eski - r_simdi)/sure: ayni sinusle
    #     YINE kandirildi (22.1 s). Periyodu pencereyle kiyaslanabilir
    #     her salinim, sonlu bir pencereyi kandirir -- pencere
    #     buyutmek de gecikme demek.
    # best-so-far izi MONOTON AZALANDIR: salinim ona yeni minimum
    # yaptirmaz, dolayisiyla olcu YAPISI GEREGI salinima bagisiktir.
    # Bu tek olcu, tasarim notundaki iki tanigi (surekli kapanma ve
    # kesikli/yeni-minimum) ZATEN kapsar: gercekten kapaniyorsan
    # best-so-far surekli iniyordur.
    # 1.0 m/s esigi: gurultu tabaninin belirgin ustunde ama es-hizli
    # kuyruk takibi kilidini (kazanim ~0) serbest birakmaz.
    ilerleme_pencere_s: float = 2.0
    # Kazanim hizinin olculdugu pencere. Kesikli kapanmada (viraj,
    # kadraj boslugu) tek tek kareler durgun gorunse de 2 s'lik
    # kazanim esigi asar.
    ilerleme_kazanci: float = 1.5
    # Ilerlerken saat (1 - 1.5) = -0.5 s/s ile GERI sarar (kismi
    # tazeleme), durgunken +1 s/s. 1.0 = yalnizca dondurur, >1 = geri
    # kazandirir. Oransal oldugu icin kare hizindan bagimsizdir.
    ilerleme_tavan_s: float = 22.0
    # MUTLAK TAVAN: ilerleme olsa bile bu sureden sonra ISKA. Sonsuz
    # dongu emniyeti.
    iska_baslangic_koruma_s: float = 1.0
    # Devirden sonraki ilk 1 s'de ISKA ILAN EDILMEZ. Devirde cozucu
    # soguk (ilk 2 cozum genis butceli), LPF tohumlaniyor ve menzil
    # suzgeci oturuyor; bu pencerede uretilen gecici acilma bir iska
    # degildir.
    menzil_hizi_tau_s: float = 0.30
    gecis_menzil_hizi_esigi_mps: float = 3.0
    gecis_kapanma_esigi_mps: float = 10.0
    gecis_onay_dongu: int = 4
    gecis_alan_onay_dongu: int = 6
    # GECIS TESPITI -- IKI BAGIMSIZ TANIK, HEDEF TELEMETRISI YOK.
    # KILLER analitik menzil turevi kullaniyor:
    #     d(rng)/dt = birim_kerteriz . (hedef_hizi - kendi_hizimiz)
    # ama bizde HEDEF HIZI YASAK (goruntulu_temel sozlesmesi: hedeften
    # yalniz MENZIL). Karsiligimiz iki isarettir:
    #   (1) menzil_hizi = d(r_ic)/dt, LPF tau 0.30 s. HAM menzil
    #       olcumunun turevi KULLANILAMAZ: r_olcum ardisik orneklerde
    #       +-1.5 m ziplar, dt=0.05 s'de bu +-30 m/s gurultu demektir.
    #       r_ic zaten model-ilerletmeli + kazanc 0.35 suzgecidir
    #       (bkz. _menzil) ve turevi temizdir. OLCULEN GECIS IMZASI
    #       cok guclu: CPA'da dr/dt 2 dongude -27 m/s'den +23 m/s'ye
    #       doner (50 m/s'lik sicrama), yani 3 m/s esigi ~17 kat pay.
    #   (2) alan_hizi = d(bbox alani)/dt (zaten mpc_tani'de var,
    #       bbox'tan gelir, telemetriden DEGIL). Isaret degistirmesi
    #       "hedef uzaklasiyor" demektir.
    # NEDEN "VE" DEGIL "VEYA": olculdu ki ikisi de tek basina
    # GUVENILMEZ ama farkli sekilde bozulur --
    #   * alan_hizi CPA'da DOYUYOR: 220521 seg0'da bbox tum kadraji
    #     kapliyor (alan 34440 px^2 tavan), buyume hizi LPF'si
    #     +43000'e ciktigi icin isaret degistirmesi 2.5 s GECIKIYOR.
    #   * menzil kanali estimator 10 Hz oldugu icin nadiren bayatlar;
    #     ayrica CPA'da tespit sik sik kayboluyor (223143 seg4:
    #     bbox_yas 0.64 s, alan DONMUS) -- orada alan tamamen kor,
    #     menzil calisiyor.
    # Yani biri gurultuluyken/kor iken digeri dogrular: her iki tanik
    # da TEK BASINA yeterlidir, ama her biri kendi ONAY SAYACINI
    # doldurmak zorundadir (4 dongu ~ 0.2 s menzil, 6 dongu ~ 0.3 s
    # alan; alan daha gurultulu oldugu icin daha uzun).
    #
    # gecis_kapanma_esigi_mps = 15: UCUNCU VE ZORUNLU SART -- gecis
    # ilan edilebilmesi icin menzil hizinin, gecis_arm cemberinin
    # ICINDE en az bu kadar NEGATIF (kapanan) olmus olmasi gerekir.
    # NEDEN: menzilin isaret degistirmesi tek basina "gectim" DEMEK
    # DEGILDIR -- MPC orta safhada genis bir salinim yapip 18 m'de
    # yavaslayarak donebilir ve menzil yine acilir. Ikisi FIZIKSEL
    # OLARAK AYRILIR: terminal gecis TAM HIZDA bir delip-gecmedir.
    # OLCUM (17 gercek yetki segmenti + cevrimdisi motor; r<=20 m
    # icinde gorulen en negatif menzil hizi):
    #     gercek terminal gecis  : -17.7 -19.7 -19.9 -20.8 -23.9
    #                              -27.6 -28.7 -29.0 m/s
    #     yavas yaklasma / fren  :  -6.7  -7.8  -8.2  -8.8 -11.9 m/s
    #     motorun salinim vakasi : -12.4 m/s (CPA 18.1 m, ardindan
    #                              42 m'ye acilip GERI DONUP 0.43 m'de
    #                              vurdu -- yani ISKA ilan etmek
    #                              GERCEK bir hatadir)
    # 15.0 iki kumenin arasindaydi, her iki yona 2.6 m/s pay birakiyordu.
    #
    # 15.0 -> 10.0 (2026-08-05, 35 m/s turu). ESKI VARSAYIM YANLIS
    # CIKTI: "tavan 35'e cikarsa gercek gecislerin kapanma hizi da
    # buyur" ancak KAFA KAFAYA geometride dogrudur. Saf KUYRUK
    # takibinde kapanma hizi hedef hiziyla SINIRLIDIR:
    #     kapanma = v_bizim - v_hedef = 35 - 21.05 = 13.9 m/s
    # yani 15 m/s esigi kuyrukta MATEMATIKSEL OLARAK ulasilamaz.
    # Olculdu: 0/16 ates (elips 0/9, hedef_sonsuz 0/7) -- yani gecis
    # kolu bu rejimde HIC calismadi. 10.0, 13.9'un altinda 3.9 m/s
    # pay birakir; hedef 25 m/s'e cikarsa (kapanma 10.0) esik yine
    # tam sinira gelir, o yuzden bu sayi HEDEF HIZINA baglidir ve
    # hedef hizi degisirse yeniden olculmelidir.
    # AYIRICI GOREVI GEOMETRIYE DEVREDILDI: yanlis ilanlari artik
    # kapanma hizi degil iska_gecis_arm_m = 12 m ceberi eler (orta
    # safha salinimi CPA 18.1 m > 12). Kapanma sarti KALDIRILMADI,
    # ikinci savunma olarak duruyor: 10 m/s, olculen yavas yaklasma /
    # fren kumesinin (-6.7..-8.8 m/s) ustunde.
    iska_suzulme_hiz_mps: float = 12.0
    iska_suzulme_ivme_mps2: float = 3.0
    # ISKA SUZULMESI ARTIK FRENLI (2026-08-05, 35 m/s turu).
    # ESKI DAVRANIS: ISKA'da komut = OLCULEN hiz (ivme sifir, duz
    # ucus). 18 m/s'de dogruydu. 35 m/s'de degil: yetkiyi 35 m/s ile
    # devretmek, konumlu guduma en kucuk donus yaricapi v^2/a = 245 m
    # olan bir arac birakmak demektir -- yeniden konumlanma 490 m'lik
    # bir U donusu ister. 12 m/s'de ayni yaricap 29 m'ye iner (8 m/s'de
    # 13 m); yani birakma ANINDAKI HIZ, yeniden angajman suresini
    # dogrudan belirler.
    # NEDEN SIFIR DEGIL, NEDEN RAMPA: sifir yazmak "komut vermemek"
    # degil TAM FREN komutudur (2026-08-04 dersi; fren -> burun yukari
    # -> sabit kamera yukari bakar). Bunun yerine komut, o anki hiz
    # YONUNDE tutulur ve BUYUKLUGU 3.0 m/s^2 ile (WPNAV_ACCEL 5'in
    # %60'i, yani yatma atan(3/9.81) = 17 deg) 12 m/s'e indirilir.
    # Yon KORUNUR: donus konumlu gudumun isidir, biz yalnizca ona
    # donebilecegi bir hiz birakiriz.
    iska_redis_anahtar: str = ''
    # ISTEGE BAGLI dogrudan Redis yayini (varsayilan KAPALI).
    # Asil mekanizma Komut nesnesine konan 'birak' bayragidir (bkz.
    # _iska_komut); ORTAK DOSYA degistirilmeden once bu anahtar bir
    # kopru olarak kullanilabilir. 'komut_yetkisi'ne YAZILMAZ:
    # bbox_to_redis her karede o anahtari kendi moduyla EZIYOR
    # (bbox_to_redis.py:472), yani kontrolcunun yazdigi deger 33 ms
    # icinde silinir.

    def __post_init__(self):
        if sum(self.bloklar) != self.n_adim:
            raise ValueError(f"bloklar toplami {sum(self.bloklar)} != "
                             f"n_adim {self.n_adim}")
        if not self.yaw_komutu_ver:
            # Ablasyon tutarli olsun: yaw komutlanmayacaksa MODEL de
            # yaw yetkisi OLMADIGINI bilmeli, yoksa plani var olmayan
            # bir kontrol uzerine kurar.
            self.yaw_hiz_tavani_dps = 0.0


# ====================================================== YARDIMCI PARCA

def los_ucayak(ex_deg, eps_deg):
    """Burun (heading) cercevesinde LOS ucayagi: (l, e2, e3).

    Cerceve: x ileri (burun, YATAY), y sag, z asagi -- yani yalniz yaw
    uygulanmis NED. ex: hedefin buruna gore yatay kerterizi (+sag),
    eps: hedefin ufka gore yukselisi (+yukari).
        l  : LOS boyunca (u1 pozitif = MENZIL KAPANIR)
        e2 : LOS'a dik, yatay, saga dogru (u2 pozitif = ex AZALIR)
        e3 : LOS'a dik, dusey duzlemde ASAGI (u3 pozitif = biz alcaliriz,
             hedef bize gore YUKSELIR: eps ARTAR, ey = -eps AZALIR)
    Uc vektor ortonormaldir (mpc_test.py dogruluyor).
    """
    ex = math.radians(ex_deg)
    eps = math.radians(eps_deg)
    ce, se = math.cos(eps), math.sin(eps)
    cx, sx = math.cos(ex), math.sin(ex)
    l = np.array([ce * cx, ce * sx, -se])
    e2 = np.array([-sx, cx, 0.0])
    e3 = np.array([se * cx, se * sx, ce])
    return l, e2, e3


def _izdusum_kure_dilim(U, v_tav, a_dik, vz_alt, vz_ust, yaw_alt, yaw_ust):
    """Girdi kisit kumesine TAM Oklid izdusumu (blok blok, vektorel).

    v_tav / vz_alt / vz_ust / yaw_tav BLOK BASINA dizidir; onkosullama
    (precondition) her blogu farkli olcekledigi icin sinirlar da blok
    basina degisir. Onkosullama hiz uclusunu AYNI carpanla olcekler,
    boylece kure kure kalir (elipsoid olmaz) ve izdusum kapali formda
    kalir -- onkosullamayi bu yuzden blok-uniform sectik.

    Kume (her blok icin): {|v|<=v_tav} kesisim {vz_alt <= a.v <= vz_ust}
    carpim {yaw_alt<=yaw<=yaw_ust}.  a = LOS ucayaginin NED-z satiri,
    |a|=1 oldugu icin dilim izdusumu basit oteleme. Dusey dilim ve yaw
    kutusu SERT FOV kisitiyla daraltilmis halde gelir (bkz.
    MpcCozucu._cbf_sinirlari): kadraj kisiti boylece cezaya degil
    IZDUSUME emanet edilir, yani kesin saglanir.

    Kure+dilim kesisimine izdusumun KAPALI FORMU:
      1) p = kureye izdusum; dilimde ise cevap odur (C1'e izdusum
         C1 kapsayan kumede kaldigi icin gecerlidir).
      2) q = dilime izdusum; kurede ise cevap odur.
      3) degilse cevap iki sinirin kesisim cemberindedir:
         x = c*a + sqrt(R^2-c^2) * birim(y - (a.y) a)
    """
    Ur = U.reshape(-1, 4).copy()
    v = Ur[:, :3]
    yaw = Ur[:, 3]

    # 1) kure
    n = np.linalg.norm(v, axis=1)
    olcek = np.where(n > v_tav, v_tav / np.maximum(n, 1e-12), 1.0)
    olcek = np.minimum(olcek, 1.0)
    p = v * olcek[:, None]
    s_p = p @ a_dik
    tamam = (s_p >= vz_alt - 1e-9) & (s_p <= vz_ust + 1e-9)

    if not np.all(tamam):
        # ihlal eden bloklar icin 2) ve 3)
        s_y = v @ a_dik
        c = np.clip(s_y, vz_alt, vz_ust)
        q = v - (s_y - c)[:, None] * a_dik[None, :]
        n_q = np.linalg.norm(q, axis=1)
        kurede = n_q <= v_tav + 1e-9
        # 3) cember cozumu
        y_dik = v - s_y[:, None] * a_dik[None, :]
        n_dik = np.linalg.norm(y_dik, axis=1)
        guvenli = np.maximum(n_dik, 1e-12)
        yaricap = np.sqrt(np.maximum(v_tav ** 2 - c ** 2, 0.0))
        cember = (c[:, None] * a_dik[None, :]
                  + (yaricap / guvenli)[:, None] * y_dik)
        secim = np.where(kurede[:, None], q, cember)
        p = np.where(tamam[:, None], p, secim)

    Ur[:, :3] = p
    Ur[:, 3] = np.clip(yaw, yaw_alt, yaw_ust)
    return Ur.reshape(-1)


class BozucuKestirici:
    """d = (olculen aci hizi) - (modelin kendi hizimizdan ongordugu).

    Fiziksel anlami: hedefin LOS'a dik hiz bileseni / menzil. Yani
    MPC'nin ihtiyac duydugu TEK hedef bilgisi -- ve bu bilgi bbox'tan
    (goruntuden) geliyor, hedef telemetrisinden DEGIL. goruntulu_temel
    sozlesmesi bunu serbest birakiyor.
    """

    def __init__(self, ayar: MpcAyar):
        self.a = ayar
        self.sifirla()

    def sifirla(self):
        self.d_ex = 0.0
        self.d_ey = 0.0
        self.d_r = 0.0
        # KUTU MERKEZI icin AYRI, DAHA YAVAS kopya (bkz. guncelle).
        self.d_ex_kutu = 0.0
        self.d_ey_kutu = 0.0
        self.guven = 0.0
        self._t_toplam = 0.0
        self._n = 0
        self._ex_onceki = None
        self._ey_onceki = None
        self._r_onceki = None
        # APN (bkz. cevre_apn): hedefin LOS'a DIK hizi ve onun turevi.
        # v_dik = d_ex * r / KDEG  [m/s]   (d_ex'in fiziksel tersi)
        # a_dik = d(v_dik)/dt      [m/s^2] (AYRI, daha yavas LPF)
        self.v_dik = 0.0
        self.a_dik = 0.0
        self._v_dik_onceki = None

    def guncelle(self, ex, ey, r_olcum, dt, c2, c3, w1, w2, w3, yaw_dps):
        """dt OLCULEN adimdir. Turev penceresi bozulduysa (bbox bosluk,
        dt sicramasi) guncelleme ATLANIR, eski d dondurulur."""
        # YAKIN MENZIL KORUMASI (2026-08-04 tur-3): kelepce fizik
        # uzerinden KDEG*v_max/r idi; r=6 m'de 382 dps eder. Devirden
        # hemen sonra (r=4-9 m) d_ex olculdu: -136...+160 dps salindi
        # ve yaw komutu +-90 raylari arasinda ISARET ATLADI. Yakin
        # menzilde aci turevleri gercekten devasa ve gurultulu; orada
        # bozucu kestirimi ANLAMSIZDIR. Cozum: r < dondur_menzil ise
        # guncellemeyi DONDUR (son saglikli degeri tut). Terminal
        # fazda plan zaten kurulmus durumda, t_go < 1 s.
        yakin = (r_olcum is not None
                 and r_olcum < self.a.bozucu_dondur_menzil_m)
        gecerli = (self._ex_onceki is not None and 0.01 < dt < 0.35
                   and not yakin)
        if gecerli:
            ex_hiz = (ex - self._ex_onceki) / dt
            ey_hiz = (ey - self._ey_onceki) / dt
            # model: ex_dot = -c2*w2 - yaw ; ey_dot = -c3*w3
            ham_ex = ex_hiz - (-c2 * w2 - yaw_dps)
            ham_ey = ey_hiz - (-c3 * w3)
            self._n += 1
            self._t_toplam += dt
            # HIZLI BASLANGIC: ilk orneklerde 1/n (kosan ortalama),
            # sonra normal LPF. d bir kac dongude oturur.
            k = max(1.0 / self._n, dt / (dt + self.a.bozucu_tau_s))
            self.guven = 1.0 - math.exp(
                -self._t_toplam / max(1e-3, self.a.bozucu_guven_s))
            self.d_ex += k * (ham_ex - self.d_ex)
            self.d_ey += k * (ham_ey - self.d_ey)
            # AYRI YAVAS KOPYA -> SERT KISITIN KUTU MERKEZI.
            # Tur-3 chatter kok nedeni: ayni gurultulu d hem maliyete
            # hem de CBF kutusunun MERKEZINE giriyordu (cift sayim).
            # Kutu genisligi 26 deg sabit ama merkezi her dongu
            # yeniden hesaplandigi icin zipliyordu: yaw_alt_cbf adim
            # farki medyan 0.30 iken rms 3.95, p95 6.2 dps (agir
            # kuyruk) -> yaw komutu 4 Hz'de +-16 dps chatter.
            # Maliyet referansi HIZLI kalmali (gudume tepki), ama
            # KISIT SINIRI yavas ve kararli olmali.
            k_kutu = max(1.0 / self._n,
                         dt / (dt + self.a.bozucu_kutu_tau_s))
            self.d_ex_kutu += k_kutu * (ham_ex - self.d_ex_kutu)
            self.d_ey_kutu += k_kutu * (ham_ey - self.d_ey_kutu)
            if (self.a.menzil_bozucu_kaynak == "menzil"
                    and r_olcum is not None and self._r_onceki is not None):
                # model: r_dot = -w1 + d_r  ->  d_r = r_dot_olcum + w1
                ham_r = (r_olcum - self._r_onceki) / dt + w1
                self.d_r += k * (ham_r - self.d_r)
        self._ex_onceki, self._ey_onceki = ex, ey
        if r_olcum is not None:
            self._r_onceki = r_olcum

        # fizik kelepcesi: |d| <= KDEG * v_hedef_max / menzil
        # ARTI MUTLAK TAVAN: fizik kelepcesi tek basina yakin menzilde
        # (r=6 m) 382 dps'e izin veriyordu -- yaw tavani 90 dps olan
        # bir aracta bu, kisitin gurultuyle doygunluga surulmesi
        # demek. Mutlak tavan yaw yetkisinin biraz ustunde tutulur.
        r_g = max(self.a.menzil_taban_m,
                  r_olcum if r_olcum else self.a.menzil_yoksa_m)
        tavan = min(KDEG * self.a.hedef_hiz_tavani_mps / r_g,
                    self.a.bozucu_mutlak_tavan_dps)
        self.d_ex = float(np.clip(self.d_ex, -tavan, tavan))
        self.d_ey = float(np.clip(self.d_ey, -tavan, tavan))
        self.d_ex_kutu = float(np.clip(self.d_ex_kutu, -tavan, tavan))
        self.d_ey_kutu = float(np.clip(self.d_ey_kutu, -tavan, tavan))
        self.d_r = float(np.clip(self.d_r, -self.a.hedef_hiz_tavani_mps,
                                 self.a.hedef_hiz_tavani_mps))

        # ---- APN: hedefin dik HIZI ve IVMESI (bkz. cevre_apn) --------
        # v_dik KELEPCELENMIS d_ex uzerinden turetilir; boylece d_ex ile
        # v_dik ayni bilginin iki yuzu kalir ve kelepce iki yerde
        # ayrismaz. Fiziksel tanim: d_ex = KDEG * v_dik / r.
        self.v_dik = self.d_ex * r_g / KDEG
        if gecerli:
            if self._v_dik_onceki is not None:
                ham_a = (self.v_dik - self._v_dik_onceki) / dt
                # AYRI ve DAHA YAVAS LPF. 1/n hizli baslangic BILINCLI
                # OLARAK YOK (d_ex kolundan tek farki): turevin ilk
                # ornegi tek basina kestirimi kelepceye surerdi.
                k_a = dt / (dt + self.a.apn_tau_s)
                self.a_dik += k_a * (ham_a - self.a_dik)
                tav_a = self.a.apn_a_tavani_mps2
                self.a_dik = float(np.clip(self.a_dik, -tav_a, tav_a))
            self._v_dik_onceki = self.v_dik
        else:
            # SUREKLILIK KOPTU (r < dondur_menzil ya da dt/bbox boslugu).
            # a_dik SON SAGLIKLI degerinde tutulur -- d_ex ile AYNI
            # dondurma kurali. Onceki ornek ATILIR: dondan cikisin ilk
            # karesinde saniyeler boyunca birikmis bir v_dik farkini
            # tek bir dt'ye bolmek tavan degerinde SAHTE bir ivme
            # sicramasi uretirdi.
            self._v_dik_onceki = None
        return self.d_ex, self.d_ey, self.d_r

    def apn_a_etkin(self) -> float:
        """Yasaya VERILECEK hedef yanal ivmesi [m/s^2]. Kol kapaliysa 0.

        Iki kapi (bkz. cevre_apn):
          (1) CIKARMALI OLU BANT: a_etkin = sign(a)*max(|a|-db, 0).
              DUZ bacakta olculen gurultu tabani ~0.3-0.5 m/s^2; olu
              bant onu TAM SIFIRA indirir, yani duz hedefte yasa
              bugunkunun AYNISI kalir. Cikarmali (sert degil) secildi:
              sert esik gecis aninda bir sicrama uretir ve o sicrama
              dogrudan ufka yayilirdi.
          (2) GUVEN RAMPASI: d_ex'in PN agirligini carpan ayni rampa
              (BozucuKestirici.guven, tau = bozucu_guven_s). Devir
              aninda kestirim daha oturmamistir; a_dik oradan gelen en
              gurultulu terimdir, o yuzden ayni rampanin altinda kalir.
        """
        if not self.a.apn:
            return 0.0
        db = self.a.apn_olu_bant_mps2
        buyukluk = max(abs(self.a_dik) - db, 0.0)
        return (math.copysign(buyukluk, self.a_dik)
                * max(self.guven, 0.0) * self.a.apn_carpani)


# ============================================================= COZUCU

class MpcCozucu:
    """Yogunlastirilmis LTV-QP + FISTA. Canli koddan BAGIMSIZ kullanilir
    (mpc_test.py dogrudan bunu cagirir)."""

    NX = 6      # [ex, ey, r, w1, w2, w3]
    NU = 4      # [u1, u2, u3, yaw_dps]

    def __init__(self, ayar: MpcAyar = None):
        self.a = ayar or MpcAyar()
        a = self.a
        N, nb = a.n_adim, len(a.bloklar)
        self.N, self.nb = N, nb
        self.nu_top = self.NU * nb
        self.nx_top = self.NX * N

        # adim -> blok esleme
        self.blok_of = np.empty(N, dtype=int)
        i = 0
        for b, uzunluk in enumerate(a.bloklar):
            self.blok_of[i:i + uzunluk] = b
            i += uzunluk

        # --- maliyet satir/sutun indeksleri (SABIT; her dongude yalniz
        #     degerler yazilir -> tahsis yok, hizli) ---
        k = np.arange(N)
        self.w_sut = np.concatenate([
            self.NX * k + 0,    # ex        (seviye)
            self.NX * k + 1,    # ey        (seviye)
            self.NX * k + 2,    # r         (seviye)
            self.NX * k + 4,    # w2        (ataletsel LOS hizi, yatay)
            self.NX * k + 5,    # w3        (ataletsel LOS hizi, dikey)
            self.NX * k + 1,    # ey        (DIKEY HATA: hedef hatti ref)
        ])
        # 6. satir bloku: DOGRUDAN DIKEY HATA (env dugmeli, varsayilan
        # KAPALI -> agirligi 0, yani M ve b'ye sifir satir olarak girer;
        # cikti BIT-AYNI kalir). Ayri bir blok olmasinin nedeni referans
        # AYRISIMI: 2. blok (q_ey) kadraj icin CANLI gimbal eksenini,
        # bu blok carpma icin "hedef hatti"ni (eps=0) referans alir.
        # Bkz. cevre_dikey_hata().
        self.n_satir = 6 * N
        self.w_deg = np.zeros(self.n_satir)
        self.w_ref = np.zeros(self.n_satir)

        # FOV satirlari. YATAY satir = ex_k (tek sutun). DIKEY satir =
        # beta_k = ey_k - kats*(a_dik . w_k) + C, yani DORT sutun
        # (ey, w1, w2, w3); a_dik her dongude degistigi icin Gf
        # gather'lardan kurulur (dense carpim yok).
        self.sut_ex = self.NX * k + 0
        self.sut_ey = self.NX * k + 1
        self.sut_r = self.NX * k + 2
        self.sut_w = np.stack([self.NX * k + 3, self.NX * k + 4,
                               self.NX * k + 5])          # (3, N)
        self.fov_alt = np.zeros(2 * N)
        self.fov_ust = np.zeros(2 * N)
        self.fov_rho = np.concatenate([np.full(N, a.rho_fov),
                                       np.full(N, a.rho_fov_dikey)])
        self.fov_etkin = self.fov_rho > 0.0

        # --- girdi seviyesi ve fark cezalari (SABIT) ---
        r_lvl = np.array([0.0,                                  # u1: ceza YOK
                          a.r_hiz / a.olcek_hiz_mps ** 2,
                          a.r_hiz / a.olcek_hiz_mps ** 2,
                          a.r_yaw / a.olcek_yaw_dps ** 2])
        self.R_kos = np.tile(r_lvl, nb)
        sdu = np.array([a.r_delta_hiz / a.olcek_hiz_mps ** 2,
                        a.r_delta_hiz / a.olcek_hiz_mps ** 2,
                        a.r_delta_hiz / a.olcek_hiz_mps ** 2,
                        a.r_delta_yaw / a.olcek_yaw_dps ** 2])
        self.sdu = sdu
        D = np.zeros((self.nu_top, self.nu_top))
        for b in range(nb):
            s = self.NU * b
            D[s:s + self.NU, s:s + self.NU] = np.eye(self.NU)
            if b > 0:
                D[s:s + self.NU, s - self.NU:s] = -np.eye(self.NU)
        self.D = D
        Sd = np.tile(sdu, nb)
        self.DtSD = D.T @ (Sd[:, None] * D)
        self.Sd = Sd
        # YAW fark cezasi artik DONGU BASINA DEGISIYOR (gain
        # scheduling, bkz. yaw_delta_agirlik). Her dongude 28x28
        # matrisi yeniden kurmamak icin ceza HIZ ve YAW parcalarina
        # ayristirilir; kosum aninda yalnizca skaler bir carpma-toplama
        # kalir (ihmal edilebilir maliyet).
        sdu_v = sdu.copy(); sdu_v[3] = 0.0          # yalniz hiz
        sdu_y = np.array([0.0, 0.0, 0.0, 1.0])      # birim yaw
        self.Sd_v = np.tile(sdu_v, nb)
        self.Sd_y = np.tile(sdu_y, nb)
        self.DtSD_v = D.T @ (self.Sd_v[:, None] * D)
        self.DtSD_y = D.T @ (self.Sd_y[:, None] * D)
        self.prox_kos = np.tile(
            np.array([a.lambda_prox / a.olcek_hiz_mps ** 2] * 3
                     + [a.lambda_prox / a.olcek_yaw_dps ** 2]), nb)
        self.ivme_kos = np.tile(
            np.array([a.q_ivme / a.olcek_ivme_mps ** 2] * 3 + [0.0]), nb)

        # durma olcutu icin birim esitleme (yaw deg/s -> m/s esdegeri)
        self.tol_agirlik = np.tile(
            np.array([1.0, 1.0, 1.0, a.olcek_hiz_mps / a.olcek_yaw_dps]), nb)

        # calisma tamponlari
        self.Gam = np.zeros((self.nx_top, self.nu_top))
        self.Xf = np.zeros(self.nx_top)
        self.son_sure_ms = 0.0
        self.son_iterasyon = 0
        # BUTCE KESILMESI (2026-08-07, gercek ucus loglamasi): cozucu
        # yakinsayarak mi durdu, yoksa iterasyon tavanina / sure butcesine
        # mi carpti? Ikisi de ayni 'iter' ve 'sure_ms' sayilarini
        # uretebiliyor, yani bugune kadar Pi'de SESSIZ bir bozulmaydi:
        # yakinsamamis bir QP cozumu "calisiyor gibi" gorunen ama
        # optimal olmayan komut verir. Bayrak bunu gorunur yapar.
        self.son_butce_kesti = 0
        self.son_maliyet = 0.0
        self.son_beta = 0.0
        self.son_yaw_delta = 0.0
        self.son_bant_alt = self.a.fov_alt_bant_deg
        self.son_bant_ust = self.a.fov_ust_bant_deg
        self.son_vurus = 0.0
        self.son_v_tavan = self.a.hiz_tavani_mps
        self.son_ivme_carpani = 1.0
        self.son_hiza_ref = 0.0
        # t_go sekilli tau tanisi (bkz. cevre_dikey_tgo). KAPALI kolda
        # NaN kalir -- CSV'de "kol kapali" ile "t_go gecersiz" ayrilsin.
        self.son_tgo = float('nan')
        self.son_dikey_tau = float('nan')
        # APN: yasada KULLANILAN hedef yanal ivmesi [m/s^2] (olu bant ve
        # guven carpani UYGULANMIS hali). Kol kapaliyken tam 0. Bkz.
        # cevre_apn; tani CSV kolonu 'apn_a'.
        self.son_apn_a = 0.0
        # DOYUMLU eyleyici (bkz. cevre_eyleyici): _nominal_yorunge her
        # cagrida yazar. None = kol KAPALI -> eski sabit tau yolu.
        self._al_h = None
        self._al_v = None
        self._tau_v_blok = None
        self.son_tau_eff = float('nan')
        self.son_tau_eff_z = float('nan')
        self.son_dikey_hata = 0.0
        self.son_fov_serbest = 0
        self.bos_sayac = 0
        self._birakildi = False
        self._son_cbf = (0.0, 0.0, 0.0, 0.0)
        self.cozum_sayaci = 0
        # heading ILERI ekseninin LOS ucayagindaki bilesenleri; coz()
        # her cagrida gunceller, burada guvenli varsayilan durur.
        self._ileri_eks = np.array([1.0, 0.0, 0.0])

    @property
    def kats(self):
        """Tirmanma -> pitch -> KAMERA EKSENI baglasimi [deg/(m/s)].

        pitch_baglasimi kapaliyken (gimbal) SIFIRDIR: eksen govdeden
        bagimsiz stabilize oldugu icin tirmanmak kadraji kaydirmaz.
        Ozellik (property) olarak duruyor ki ayar nesnesi kurulumdan
        SONRA degistirildiginde bayat bir kopya kalmasin."""
        return self.a.pitch_tirmanma_kats if self.a.pitch_baglasimi else 0.0

    # ------------------------------------------------------ ic parcalar

    def _adim_sureleri(self, dt0):
        h = np.full(self.N, self.a.adim_s)
        h[0] = float(np.clip(dt0, 0.02, 0.30))
        # TO_TEST madde 3: maliyet ufkunu MENZILLE olcekle (0 = kapali).
        # r, coz() icinde x0[2]'den saklanir. Ilk adim OLCULEN dt oldugu
        # icin olceklenmez -- o adim gercekten uygulanacak komuttur.
        ref = self.a.ufuk_menzil_ref_m
        if ref > 0.0:
            r = getattr(self, '_son_menzil', None)
            if r is not None:
                taban = self.a.ufuk_adim_taban_s / max(self.a.adim_s, 1e-6)
                h[1:] = self.a.adim_s * float(np.clip(r / ref, taban, 1.0))
        return h

    def _apn_d_ex(self, d_temel, rg, t_k, apn_a):
        """APN: v_dik SABIT yayilimina hedef IVME terimini ekler.

        Temel (bugunku) yayilim  d_ex_k = d_ex0 * r0/rbar_k, ki bu tam
        olarak KDEG*v_dik0/rbar_k demektir (v_dik SABIT varsayimi).
        APN bunu birinci mertebeden acar:
            d_ex_k = KDEG*(v_dik0 + a_dik*t_k) / rbar_k
                   = d_temel_k  +  KDEG*a_dik*t_k / rbar_k

        (Isim notu: apn_a = hedefin YANAL IVMESI [m/s^2]. Bu dosyada
        _cbf_sinirlari'ndaki 'a_dik' AYRI bir seydir -- orada asagi
        yon BIRIM VEKTORU. Karismasin diye burada apn_a denildi.)

        *** BU FONKSIYON YALNIZCA apn_a != 0 IKEN CAGRILIR. *** Kapali
        kolda cagiran aritmetigine HIC girmez -> davranis BIT-AYNI.

        NOT: yalnizca SERBEST CEVABA girer. Gam (girdi->durum) ve
        dolayisiyla Hessian bu terimden ETKILENMEZ -- cozucu is yuku
        ve suresi ayni kalir.

        Kelepce: TOPLAM |d_ex_k| fizik tavaninin altinda tutulur (ayni
        formul BozucuKestirici'de k=0 icin uygulaniyor). Ufkun sonunda
        (t_k ~ 2.3 s) 6 m/s^2 lineer yayilim 13.7 m/s'lik bir v_dik
        degisimi demek; tavan bu dogrusal ekstrapolasyonun fiziksel
        olmayan kuyrugunu keser.
        """
        a = self.a
        d = d_temel + (KDEG * apn_a) * t_k / rg
        tavan = np.minimum(KDEG * a.hedef_hiz_tavani_mps / rg,
                           a.bozucu_mutlak_tavan_dps)
        return np.clip(d, -tavan, tavan)

    def _nominal_yorunge(self, x0, U, h, d_ex, d_ey, d_r, cos_eps, r0,
                         apn_a=0.0):
        """Warm-start girdisiyle TAM nominal durum yorungesi.

        Uc ise yarar:
          (1) LTV katsayilari c = KDEG/r bunun etrafinda dondurulur
              (SQP dogrusallastirmasi). Menzil denklemi girdide zaten
              dogrusal oldugu icin rbar YAKLASIK DEGIL, tamdir.
          (2) Ivme cezasinin merkezi wbar (blok basi nominal hiz).
          (3) SERT FOV kisitinin (CBF) blok basina sinirlarini kurmak
              icin blok basindaki nominal ex/ey/w gerekir.

        apn_a: hedefin LOS'a dik IVMESI [m/s^2], 0 = KAPALI (varsayilan,
        eski davranis BIT-AYNI). Bkz. cevre_apn / _apn_d_ex.
        """
        Ur = U.reshape(self.nb, self.NU)
        w = np.array(x0[3:6], dtype=float)
        ex, ey, r = float(x0[0]), float(x0[1]), float(x0[2])
        sure_blok = 0.0
        N, nb, NU = self.N, self.nb, self.NU
        rbar = np.empty(N)
        wbar = np.zeros(self.nu_top)
        # blok basi durum (CBF icin):
        #   0..4 ex, ey, w1, w2, w3 | 5..7 c2, c3, h | 8 ileri ivme
        bb = np.zeros((nb, 9))
        ileri = self._ileri_eks           # heading-x'in LOS ucayagindaki
        w_blok_bas = None                 # bilesenleri
        gorulen = -1
        tau = self.a.hiz_gecikme_tau_s
        taban = self.a.menzil_taban_m
        t_k = 0.0                         # ADIM BASINDAKI ufuk zamani (APN)
        # DOYUMLU EYLEYICI (bkz. cevre_eyleyici). ADIM-BASI etkin tau
        # burada, NOMINAL yorunge uzerinde hesaplanir ve _yorunge_
        # matrisleri'ne AYNEN aktarilir (self._al_h) -- iki fonksiyon
        # AYNI dogrusallastirma noktasini kullanmak ZORUNDA, yoksa Xf ile
        # Gam ayrisir ve QP tutarsiz olur.
        doyum = self.a.eyleyici
        tau_lin = self.a.eyleyici_tau_lin_s
        a_max = max(self.a.eyleyici_a_max_mps2, 1e-3)
        a_max_z = max(self.a.eyleyici_a_max_z_mps2, 1e-3)
        tau_lin_z = self.a.eyleyici_tau_lin_z_s
        al_h = np.empty(N) if doyum else None
        al_v = np.empty(N) if doyum else None
        tau_v_blok = (np.full(nb, self.a.eyleyici_tau_lin_z_s)
                      if doyum else None)
        for k in range(N):
            rbar[k] = r
            hk = h[k]
            b = self.blok_of[k]
            rg = max(r, taban)
            c2 = KDEG / (rg * cos_eps)
            c3 = KDEG / rg
            olcek = r0 / rg
            # APN: v_dik sabit yerine v_dik0 + a_dik*t_k (bkz. _apn_d_ex).
            # apn_a = 0 iken bu dal HIC calismaz -> kapali kol BIT-AYNI.
            d_ex_k = (d_ex * olcek if apn_a == 0.0 else
                      float(self._apn_d_ex(d_ex * olcek, rg, t_k, apn_a)))
            if b != gorulen:
                if gorulen >= 0 and w_blok_bas is not None:
                    # onceki blogun GERCEKLESEN ortalama ileri ivmesi
                    bb[gorulen, 8] = ((w - w_blok_bas) @ ileri) / max(
                        sure_blok, 1e-3)
                wbar[NU * b:NU * b + 3] = w
                bb[b, :8] = (ex, ey, w[0], w[1], w[2], c2, c3, hk)
                w_blok_bas = w.copy()
                sure_blok = 0.0
                gorulen = b
            sure_blok += hk
            ex = ex + hk * (-c2 * w[1] + d_ex_k)
            ey = ey + hk * (-c3 * w[2] + d_ey * olcek)
            r = r + hk * (-w[0] + d_r)
            al = hk / (hk + tau) if tau > 1e-6 else 1.0
            if doyum:
                # YATAY hata buyuklugu (u1 = LOS boyu, u2 = dik yatay).
                # tau_eff = max(tau_lin, |e_yatay|/a_max): doyum bolgesinde
                # al*|e|/h = a_max, yani TAM ivme tavani.
                eh = math.hypot(Ur[b, 0] - w[0], Ur[b, 1] - w[1])
                alh = hk / (hk + max(tau_lin, eh / a_max))
                al_h[k] = alh
                w0 = w[0] + alh * (Ur[b, 0] - w[0])
                w1n = w[1] + alh * (Ur[b, 1] - w[1])
                # DIKEY (u3): dogrusal taban ESKI tau, uzerine IVME
                # SINIRI. Ilk surumde sinir YOKTU ve talep bu kanala
                # kaciyordu -- bkz. eyleyici_a_max_z_mps2.
                ev = abs(Ur[b, 2] - w[2])
                tau_vk = max(tau_lin_z, ev / a_max_z)
                alv = hk / (hk + tau_vk)
                al_v[k] = alv
                if k == 0 or self.blok_of[k - 1] != b:
                    tau_v_blok[b] = tau_vk
                w2n = w[2] + alv * (Ur[b, 2] - w[2])
                w = np.array([w0, w1n, w2n])
            else:
                w = w + al * (Ur[b, :3] - w)
            t_k += hk
        if gorulen >= 0 and w_blok_bas is not None:
            bb[gorulen, 8] = ((w - w_blok_bas) @ ileri) / max(sure_blok, 1e-3)
        # DONUS ARITESI DEGISMEDI (mpc_test.py konumsal olarak 3 deger
        # aciyor); adim-basi al YAN KANALDAN tasinir.
        self._al_h = al_h
        self._al_v = al_v
        self._tau_v_blok = tau_v_blok
        self.son_tau_eff = (float('nan') if al_h is None else
                            float(h[0] * (1.0 - al_h[0]) / max(al_h[0], 1e-9)))
        self.son_tau_eff_z = (float('nan') if al_v is None else
                              float(h[0] * (1.0 - al_v[0]) / max(al_v[0], 1e-9)))
        return np.maximum(rbar, taban), wbar, bb

    def _cbf_sinirlari(self, bb, a_dik, beta_c, d_ex, d_ey, r0, rbar,
                       vurus=0.0, tau_v=None):
        """SERT FOV kisitini GIRDI KUTUSUNA/DILIMINE cevirir.

        Bir adim sonraki kadraj degiskenleri girdide AFFINE:
            beta_{k+1} = B0 - kats*al*(a_dik . u)     -> dusey DILIM
            ex_{k+1}   = E0 - h*omega                 -> yaw KUTUSU
        Ayrik CBF: h = sinir - beta >= 0 icin h_{k+1} >= (1-g) h_k,
        yani  beta_{k+1} <= g*sinir + (1-g)*beta_k. Ihlal varken
        "iyilestir" der, "aninda sagla" demez -> ASLA infeasible degil.

        Kume yine de bos cikarsa (fiziksel hiz/yaw tavani yetmiyorsa)
        EN IYI CABA secilir: ALT kenar onceliklidir, cunku olculen
        kayiplarin tamami alttan oldu.
        """
        a = self.a
        nb = self.nb
        tau = a.hiz_gecikme_tau_s
        ex0, ey0 = bb[:, 0], bb[:, 1]
        w1, w2, w3 = bb[:, 2], bb[:, 3], bb[:, 4]
        c2, c3, hk = bb[:, 5], bb[:, 6], bb[:, 7]
        olcek = r0 / np.maximum(rbar[0], a.menzil_taban_m)
        # ONGORU UFKU: kisit TEK dongu adimi (0.05 s) uzerinden
        # yazilamaz. Eyleyici tau=1.0 s oldugundan bir adimda girdinin
        # hiza etkisi al = h/(h+tau) = 0.048'dir; kisiti bu kazancla
        # bolmek 1 derecelik model hatasini 6.6 m/s'lik dikey hiz
        # komutuna cevirir -- olculdu: kisit bang-bang'e donuyor,
        # kopter dikeyde savruluyor ve kadraj sert kisit ACIKKEN daha
        # cok kaybediliyordu. Ufuk eyleyici zaman sabitiyle ayni
        # mertebede secilir (0.8 s -> al_T = 0.44), boylece kisitin
        # girdi duyarliligi fiziksel olarak anlamli olur.
        # ONGORU UFKUNU t_go ILE OLCEKLE (2026-08-04 tur-3 chatter):
        # Yakin menzilde iki sey birden bozuluyor: (1) c = KDEG/r
        # patliyor, (2) sabit 1.0 s ufuk CARPISMA ANINI GECIYOR, yani
        # kisit artik var olmayacak bir gelecegi tahmin ediyor. Ikisi
        # birlikte kutu merkezini savuruyordu: |dYaw| rms menzil
        # bandina gore 35-60 m'de 0.67 iken 12-20 m'de 7.37 dps
        # olculdu. Ufku t_go'nun bir kesriyle sinirlamak fizigin
        # soyledigi seydir: carpismadan oteye planlama.
        # Kutu merkezinin girdiye DUYARLILIGI c2*T ile olcekleniyor ve
        # c2 = KDEG/r. Sabit T ile duyarlilik 1/r gibi patliyor. T'yi
        # MENZILLE ORANTILI secmek c2*T carpimini sabit tutar, yani
        # kutu merkezinin savrulmasi menzilden BAGIMSIZ olur.
        # (Once t_go ile olceklemeyi denedim: capraz geometride
        # kapanma hizi w1 kucuk oldugu icin t_go buyuk cikiyor ve
        # olcekleme HIC devreye girmiyordu -- olculdu, etkisiz.)
        T = float(np.clip(a.cbf_ongoru_s * rbar[0] / a.cbf_menzil_ref_m,
                          a.cbf_ongoru_min_s, a.cbf_ongoru_s))
        # DIKEY DOYUM TUTARLILIGI (2026-08-08 gece-2): asagidaki `al`
        # bu fonksiyondaki TEK eyleyici kazancidir ve DUSEY dilime aittir.
        # Dikey kanala ivme siniri eklendiginde (bkz.
        # MpcAyar.eyleyici_a_max_z_mps2) kisit da AYNI kazanci gormeli,
        # yoksa plan "yavasladim" derken kisit "hizliyim" varsayar ve
        # dilim gercekte ulasilamayan bir vz talep eder. tau_v BLOK BASI
        # dikey tau_eff dizisidir; None = kol kapali -> eski sabit tau,
        # yani BIT-AYNI.
        if tau_v is None:
            al = T / (T + tau) if tau > 1e-6 else 1.0     # BIT-AYNI dal
        else:
            al = T / (T + np.maximum(tau_v, 1e-6))        # blok basi dizi

        # --- DUSEY: beta = ey - kats*vz + C ---
        kats = self.kats
        vz_simdi = w1 * a_dik[0] + w2 * a_dik[1] + w3 * a_dik[2]
        beta_simdi = ey0 - kats * vz_simdi + beta_c
        # IKI KANAL: (1) PITCH baglasimi (kats; gimbalde SIFIR),
        # (2) SAF GEOMETRI -- alcalirsan hedefin gorunen yukselisi
        #     artar, yani ey duser. Kanal kazanci:
        #         w3 = (vz + w1*sin eps)/cos eps  ->  dw3/dvz = 1/cos eps
        #         d(ey1)/ds = -T*c3*al/cos eps = -T*c2*al
        #     (c2 = c3/cos eps zaten blok basi tabloda var.)
        # Geometrik kanal ONCEDEN IHMAL EDILIYORDU; +30 montajda pitch
        # kanali (3.2) baskin oldugu icin fark etmiyordu, ama gimbalde
        # (kats=0) tek kanal ODUR. Buyuklugu: T menzille olcekli
        # secildigi icin T*c2 ~ 1.3 deg/(m/s) sabittir, yani pitch
        # kanalinin ~%40'i.
        kats_geom = T * c2                        # = T*c3/cos_eps
        ey1 = ey0 + T * (-c3 * w3 + d_ey * olcek)
        B0 = (ey1 + kats_geom * al * vz_simdi
              - kats * (1.0 - al) * vz_simdi + beta_c)
        katsayi = (kats + kats_geom) * al         # beta1 = B0 - katsayi*s
        katsayi = np.maximum(katsayi, 1e-6)
        # FREN -> BURUN YUKARI: kopter yatarak ivmelenir, pitch ~
        # -atan(a_ileri/g) yani her 1 m/s^2 FREN burnu KDEG/g = 5.84 deg
        # kaldirir. Bu, tirmanma teriminden (3.2 deg per m/s) cok daha
        # buyuk ve olculen kadraj kayiplarinin asil suclusu. Kisitin
        # normali a_dik'e PARALEL OLMADIGI icin dilime yazilamaz;
        # bunun yerine PLANLANAN frenin yiyecegi pay BANTTAN DUSULUR
        # (konservatif daraltma, izdusum kapali formda kalir).
        # DIKKAT (2026-08-04 ikinci cakilma dersi): burada MUTLAK
        # planlanan fren kullanilamaz. beta_c zaten OLCULEN pitch'i
        # tasiyor ve o pitch mevcut freni ICERIYOR; mutlak freni bir
        # daha dusmek CIFT SAYIM olur. Sonucu olcumle gorduk: MPC
        # planinda daima bir miktar yavaslama oldugu icin bant surekli
        # tabanina (4 deg) cokuyor, beta ~9-12 kalici "ihlal" sayiliyor
        # ve kisit KESINTISIZ alcalma talep ediyordu -- 2 m/s'lik
        # tavanla bile 15 s'de 30 m irtifa demek, yani yer.
        # Dogrusu, tirmanma terimiyle ayni mantik: SIMDIKI duruma gore
        # DEGISIM. Blok 0'da daralma sifirdir (pitch'i zaten olctuk).
        # GIMBAL: pitch_baglasimi kapaliyken fren burnu kaldirsa da
        # KAMERA EKSENI KIMILDAMAZ -> daraltma yapilmaz.
        # VURUS FAZI: bantlar FIZIKSEL KENARA acilir ve fren/hizlanma
        # daraltmasi sonumlenir (bkz. MpcAyar vurus_* bloku). s=0'da
        # asagisi birebir eski davranistir.
        s_v = float(np.clip(vurus, 0.0, 1.0))
        bant_alt_nom = (a.fov_alt_bant_deg
                        + s_v * (a.vurus_bant_deg - a.fov_alt_bant_deg))
        bant_ust_nom = (a.fov_ust_bant_deg
                        + s_v * (a.vurus_bant_deg - a.fov_ust_bant_deg))
        if a.pitch_baglasimi:
            # FREN -> burun YUKARI -> hedef ALT kenara (eski terim).
            fren = np.maximum(0.0, -(bb[:, 8] - bb[0, 8]))
            # HIZLANMA -> burun ASAGI -> hedef UST kenara. 35 m/s
            # turunda eklendi: mount 0'da hedef zaten eksenin USTUNDE
            # ve angajmanin ilk saniyeleri surekli ileri ivmelenmedir.
            hizlanma = np.maximum(0.0, bb[:, 8] - bb[0, 8])
            bant_alt = np.maximum(
                bant_alt_nom - (1.0 - s_v) * (KDEG / 9.80665) * fren,
                a.fov_taban_bant_deg)
            bant_ust = np.maximum(
                bant_ust_nom - (1.0 - s_v) * (KDEG / 9.80665) * hizlanma,
                a.fov_ust_taban_bant_deg)
        else:
            bant_alt = np.full(nb, bant_alt_nom)
            bant_ust = np.full(nb, bant_ust_nom)
        g = a.cbf_gamma
        hedef_alt = g * bant_alt + (1 - g) * beta_simdi
        hedef_ust = -g * bant_ust + (1 - g) * beta_simdi
        # beta1 <= hedef_alt  ->  s >= (B0 - hedef_alt)/katsayi
        s_lo = (B0 - hedef_alt) / katsayi
        # beta1 >= hedef_ust  ->  s <= (B0 - hedef_ust)/katsayi
        s_hi = (B0 - hedef_ust) / katsayi
        # === EMNIYET 1: KISITIN TALEP EDEBILECEGI ALCALMA SINIRLIDIR ===
        # 2026-08-04 CAKILMA REGRESYONU: burada s_lo dogrudan fiziksel
        # kutuya kirpiliyordu. Kume bosaldiginda s_lo BUYUK POZITIF
        # oluyor, kirpma her seferinde +alcalma_tavani'na (+4.5 m/s)
        # dayaniyor ve kutu [4.5, 4.5]'e COKUYORDU -> dikey komut
        # AZAMI ALCALMAYA CIVILENDI. Uc rotada da kopter hedefe degil
        # YERE carpti (elips: pos_z -45.8 -> +2.7, hedef 128 m otede).
        # Artik kisit en fazla fov_alcalma_talep_tavani kadar alcalma
        # ISTEYEBILIR; bunun otesi zaten kurtarilamaz bir geometridir
        # ve asagidaki zaman asimi devreye girer.
        s_lo_ham = s_lo
        s_lo = np.minimum(s_lo, a.fov_alcalma_talep_tavani_mps)
        s_hi = np.maximum(s_hi, -a.fov_tirmanma_talep_tavani_mps)
        vz_alt = np.maximum(-a.tirmanma_tavani_mps, s_lo)
        vz_ust = np.minimum(a.alcalma_tavani_mps, s_hi)
        # === EMNIYET 2: BOS KUME -> ORTA NOKTA (civileme YOK) ===
        # Chebyshev ortasi iki tek-yanli ihlali DENGELER; eskisi gibi
        # tek kenara civilemez.
        bos = vz_alt > vz_ust
        if np.any(bos):
            ort = np.clip(0.5 * (vz_alt + vz_ust),
                          -a.tirmanma_tavani_mps, a.alcalma_tavani_mps)
            vz_alt = np.where(bos, ort, vz_alt)
            vz_ust = np.where(bos, ort, vz_ust)
        # ZAMAN ASIMI GIRDISI. Iki durum sayilir:
        #  (a) kume bos / talep tavani asildi (kisit karsilanamiyor),
        #  (b) kisit SEVIYE UCUSU bile yasakliyor (alt sinir > 0), yani
        #      kesintisiz alcalma dayatiyor. (b) sarti kritik: talep
        #      tavanina TAM oturan bir kisit "karsilanabilir" gorunur
        #      ama saatlerce alcalma emreder ve araci yere indirir.
        #      Kadraj icin 2 s kesintisiz alcalma gerekiyorsa kadraj
        #      zaten kurtarilamiyordur.
        doygun = bool(bos[0] or s_lo_ham[0] > a.fov_alcalma_talep_tavani_mps
                      or vz_alt[0] > a.kisit_alcalma_esigi_mps)

        # --- YATAY: ex1 = E0 - h*omega ---
        E0 = ex0 + T * (-c2 * w2 + d_ex * olcek)
        lim = a.ex_siniri_deg + s_v * (a.vurus_ex_siniri_deg - a.ex_siniri_deg)
        hedef_u = g * lim + (1 - g) * ex0
        hedef_l = -g * lim + (1 - g) * ex0
        w_lo = (E0 - hedef_u) / T
        w_hi = (E0 - hedef_l) / T
        yaw_alt = np.maximum(-a.yaw_hiz_tavani_dps, w_lo)
        yaw_ust = np.minimum(a.yaw_hiz_tavani_dps, w_hi)
        bos = yaw_alt > yaw_ust
        if np.any(bos):
            ort = np.clip(0.5 * (w_lo + w_hi), -a.yaw_hiz_tavani_dps,
                          a.yaw_hiz_tavani_dps)
            yaw_alt = np.where(bos, ort, yaw_alt)
            yaw_ust = np.where(bos, ort, yaw_ust)
        return (vz_alt, vz_ust, yaw_alt, yaw_ust, beta_simdi,
                float(bant_alt[0]), float(bant_ust[0]), doygun)

    def _yorunge_matrisleri(self, x0, h, rbar, cos_eps, d_ex0, d_ey0, d_r,
                            r0, apn_a=0.0, al_h=None, al_v=None):
        """Xf (serbest cevap) ve Gam (girdi -> durum) yogunlastirilmasi.

        A matrisi seyrek ve yapisal; 6x6 carpim yerine 4 vektor islemi
        kullaniyoruz (Pi 5'te fark eder).

        apn_a: hedefin LOS'a dik IVMESI [m/s^2], 0 = KAPALI (varsayilan,
        eski davranis BIT-AYNI). YALNIZ Xf'e (serbest cevaba) girer;
        Gam bozucudan zaten bagimsizdir, dolayisiyla Hessian ve cozucu
        maliyeti degismez. Bkz. cevre_apn / _apn_d_ex.

        al_h: DOYUMLU eyleyicinin ADIM-BASI yatay kazanci (uzunluk N) ya
        da None = KAPALI (varsayilan, eski davranis BIT-AYNI). Diziyi
        _nominal_yorunge uretir; iki fonksiyon AYNI dogrusallastirma
        noktasini paylasmak zorundadir. YATAY kanallara (w1,w2 / u1,u2)
        uygulanir. al_v ayni seyin DIKEY kanal (w3/u3) karsiligidir;
        None = dikeyde eski sabit tau. Bkz. cevre_eyleyici() ve
        MpcAyar.eyleyici_a_max_z_mps2."""
        N, NU, NX = self.N, self.NU, self.NX
        tau = self.a.hiz_gecikme_tau_s
        Gam = self.Gam
        Gam.fill(0.0)
        Xf = self.Xf
        f = np.array(x0, dtype=float)
        G = np.zeros((NX, self.nu_top))
        c2 = KDEG / (rbar * max(cos_eps, 0.2))
        c3 = KDEG / rbar
        # bozucu menzille olceklenir: d = KDEG * v_dik / r
        olcek_d = r0 / rbar
        d_ex = d_ex0 * olcek_d
        d_ey = d_ey0 * olcek_d
        if apn_a != 0.0:
            # ADIM BASINDAKI ufuk zamani (t_0 = 0). Euler ileri
            # integrasyonda k. adimda kullanilan bozucu, o adimin
            # BASINDAKI degeridir -- rbar[k] ile ayni konvansiyon.
            t_adim = np.concatenate(([0.0], np.cumsum(h)[:-1]))
            d_ex = self._apn_d_ex(d_ex, rbar, t_adim, apn_a)

        for k in range(N):
            hk = h[k]
            al = hk / (hk + tau) if tau > 1e-6 else 1.0
            sut = NU * self.blok_of[k]

            # --- G (durum -> girdi hassasiyeti) ---
            G[0] -= (hk * c2[k]) * G[4]
            G[1] -= (hk * c3[k]) * G[5]
            G[2] -= hk * G[3]
            if al_h is None:
                G[3:6] *= (1.0 - al)
                G[0, sut + 3] -= hk      # yaw hizi ex'i dogrudan dusurur
                G[3, sut + 0] += al
                G[4, sut + 1] += al
                G[5, sut + 2] += al
            else:
                alh = al_h[k]            # YATAY: doyumlu kazanc
                alv = al if al_v is None else al_v[k]   # DIKEY
                G[3] *= (1.0 - alh)
                G[4] *= (1.0 - alh)
                G[5] *= (1.0 - alv)
                G[0, sut + 3] -= hk
                G[3, sut + 0] += alh
                G[4, sut + 1] += alh
                G[5, sut + 2] += alv

            # --- f (serbest cevap) ---
            f[0] += hk * (-c2[k] * f[4] + d_ex[k])
            f[1] += hk * (-c3[k] * f[5] + d_ey[k])
            f[2] += hk * (-f[3] + d_r)
            if al_h is None:
                f[3:6] *= (1.0 - al)
            else:
                f[3] *= (1.0 - al_h[k])
                f[4] *= (1.0 - al_h[k])
                f[5] *= (1.0 - (al if al_v is None else al_v[k]))

            Gam[NX * k:NX * (k + 1), :] = G
            Xf[NX * k:NX * (k + 1)] = f

        return Xf, Gam, c2, c3, d_ex, d_ey

    def _maliyet_satirlari(self, c2, c3, d_ex, d_ey, ey_ref, guven=1.0,
                           vurus=0.0, eps_deg=0.0, dikey_s=0.0,
                           dikey_hata_s=0.0, dikey_tau_s=None):
        a = self.a
        N = self.N
        carp = np.ones(N)
        carp[-1] = a.p_carpani
        s = np.sqrt(carp)
        # VURUS: kadraj agirliklari YUKARI (bbox merkezine kilitlenme),
        # FOV satirlarinin sinirlari fiziksel kenara acilir.
        s_v = float(np.clip(vurus, 0.0, 1.0))
        carp_ex = 1.0 + s_v * (a.vurus_ex_carpani - 1.0)
        carp_ey = 1.0 + s_v * (a.vurus_ey_carpani - 1.0)
        wq_ex = math.sqrt(a.q_ex * carp_ex) / a.olcek_ex_deg * s
        wq_ey = math.sqrt(a.q_ey * carp_ey) / a.olcek_ey_deg * s
        wq_r = math.sqrt(a.q_menzil) / a.olcek_menzil_m * s
        # PN terimi bozucu kestirimine dayanir; d guvenilmezken bu
        # agirlik kisilir (bkz. MpcAyar.bozucu_guven_s).
        carp_los = 1.0 + s_v * (a.vurus_los_carpani - 1.0)
        wq_s = math.sqrt(a.q_los_hiz * carp_los * max(guven, 0.0)) \
            / a.olcek_los_hiz_dps * s

        self.w_deg[0 * N:1 * N] = wq_ex
        self.w_deg[1 * N:2 * N] = wq_ey
        self.w_deg[2 * N:3 * N] = wq_r
        self.w_deg[3 * N:4 * N] = -wq_s * c2
        self.w_deg[4 * N:5 * N] = -wq_s * c3

        self.w_ref[0 * N:1 * N] = 0.0
        self.w_ref[1 * N:2 * N] = wq_ey * ey_ref
        self.w_ref[2 * N:3 * N] = 0.0          # Zref = 0 -> carpisma
        # artik = -wq_s*c2*w2 + wq_s*d_ex  =>  ref = -wq_s*d_ex
        self.w_ref[3 * N:4 * N] = -wq_s * d_ex
        # DIKEY LOS HIZI REFERANSI: nominalde sigma_el -> 0 (paralel
        # seyir = dikey standoff korunur). VURUS'ta hatta tirmanma
        # biasi eklenir (bkz. MpcAyar.vurus_hiza_*): sigma_el ->
        # sigma_el_ref = vurus * clip(eps/tau, tavan), eps>0 iken.
        # Referans w_ref = -wq_s*(d_ey - sigma_el_ref) olur; cunku
        # terim (-c3*w3 + d_ey) -> sigma_el_ref surer.
        sigma_el_ref = 0.0
        if a.vurus_hiza_kapatma and s_v > 0.0 and eps_deg > a.vurus_hiza_rahat_deg:
            sigma_el_ref = s_v * min(
                (eps_deg - a.vurus_hiza_rahat_deg) / max(a.vurus_hiza_tau_s, 1e-3),
                a.vurus_hiza_tavan_dps)
        # IRTIFA-AGNOSTIK TERMINAL DIKEY HIZALAMA (env dugmeli, varsayilan
        # KAPALI; bkz. cevre_dikey_terminal ve MpcAyar.dikey_terminal).
        # dikey_s menzil rampasidir (coz() hesaplar; bbox bayatsa 0).
        # IKI YANLI: eps>0 (hedef ustumuzde) -> POZITIF sigma_el iste
        # (tirman), eps<0 (hedef altimizda) -> NEGATIF iste (alcal).
        # eps_dot = -sigma_el oldugu icin her iki kolda da |eps| kuculur.
        # t_go SEKILLI TAU (env dugmeli, varsayilan ACIK; bkz.
        # cevre_dikey_tgo ve MpcAyar.dikey_tgo). dikey_tau_s coz()
        # tarafindan hesaplanir: None ise ESKI sabit tau kullanilir,
        # yani TGO kapaliyken bu satir BIT-AYNI kalir.
        # KOL SECIMI: satir dikey_terminal VEYA dikey_tgo acikken
        # kurulur -- ana kol (P + TGO) dikey_terminal'i ACMADAN da bu
        # hiz referansini kullanabilsin diye.
        if (a.dikey_terminal or a.dikey_tgo) and dikey_s > 0.0:
            olu = a.dikey_terminal_rahat_deg
            if eps_deg > olu:
                fazla = eps_deg - olu
            elif eps_deg < -olu:
                fazla = eps_deg + olu
            else:
                fazla = 0.0
            tau_d = (a.dikey_terminal_tau_s if dikey_tau_s is None
                     else float(dikey_tau_s))
            # TAVAN. Eski (sabit tau) kolda SABIT bir acisal tavandir
            # (8 dps). TGO kolunda MENZILE TUTARLI bir HIZ tavanidir ve
            # bu bir duzeltmedir, gevsetme degil:
            #   sigma_el [dps] -> dikey hiz = r * rad(sigma) = sigma / c3
            # yani SABIT acisal tavan, menzille birlikte anlamini
            # degistirir: 8 dps 45 m'de 6.3 m/s (ALCALMA kutusunun 4.5
            # tavanini ASAR, yani referans teslim edilemeyecek bir sey
            # ister) ama 15 m'de yalnizca 2.1 m/s (tam da t_go yasasinin
            # calismasi gereken yerde ONU BOGAR). Olculdu (cozucu-dongu
            # taramasi): eps0 >= 8 deg'de sabit tavan yuzunden talep
            # 25 m civarinda kirpiliyor, kirpilan talep SABIT bir
            # eps_dot demektir ve sabit eps_dot ASMA'nin ta kendisidir
            # -- k=3.0'da bile dikey hiz @CPA -5.4 m/s kaliyordu.
            # TGO kolunda tavan dogrudan GIRDI KUTUSUNUN kendisidir
            # (tirmanma 9 / alcalma 4.5 m/s), yone gore secilir.
            # Boylece referans kutunun teslim edebileceginden FAZLASINI
            # asla istemez -- 45 m'de eski tavandan DAHA SIKI, 25 m'nin
            # icinde daha genis, her yerde FIZIKSEL.
            if dikey_tau_s is None:
                tavan_d = a.dikey_terminal_tavan_dps
            else:
                v_tav = (a.tirmanma_tavani_mps if fazla > 0.0
                         else a.alcalma_tavani_mps)
                tavan_d = float(c3[0]) * v_tav
            ref_d = float(np.clip(
                dikey_s * fazla / max(tau_d, 1e-3), -tavan_d, tavan_d))
            # Emekli VURUS biasi ile birlikte acilirsa (ablasyon) BUYUK
            # OLAN kazanir; ikisi toplanip iki kat talep uretmesin.
            if abs(ref_d) > abs(sigma_el_ref):
                sigma_el_ref = ref_d
        self.son_hiza_ref = float(sigma_el_ref)
        self.w_ref[4 * N:5 * N] = -wq_s * (d_ey - sigma_el_ref)

        # DOGRUDAN DIKEY HATA (P) SATIRI -- env dugmeli, varsayilan ACIK
        # (bkz. cevre_dikey_hata ve MpcAyar.dikey_hata).
        # GEOMETRI: komut() icinde eps = -(ey + aim) TANIMDIR, yani
        # "hedef hatti" (eps = eps_hed) MPC durumunda SABIT bir ey
        # referansidir: ey_hed = -(eps_hed + aim). Ek kestirim YOK.
        # OLU BANT: |eps| <= rahat iken AGIRLIK sifir -> satir M/b'ye
        # sifir olarak girer, cozum KAPALI kol ile bit-ayni. Bant disinda
        # hedef, bandin KENARIDIR (eps_hed = +-rahat); boylece esikte
        # artik ~0'dan baslar ve talep SUREKLIDIR (sicrama yok).
        # VURUS karisimiyla OLCEKLENMEZ: amac hizalamayi son saniyeye
        # degil menzil rampasinin verdigi ~3 s'lik pencereye yaymak.
        olu = a.dikey_hata_rahat_deg
        acik = (a.dikey_hata and dikey_hata_s > 0.0
                and abs(eps_deg) > olu)
        if acik:
            eps_hed = olu if eps_deg > 0.0 else -olu
            ey_hed = -(eps_hed + a.aim_deg)
            wq_dik = (math.sqrt(a.q_dikey_hata * a.dikey_hata_carpani
                                * dikey_hata_s) / a.olcek_ey_deg) * s
            self.w_deg[5 * N:6 * N] = wq_dik
            self.w_ref[5 * N:6 * N] = wq_dik * ey_hed
            # Tani: cozucunun kapatmasini istedigi dikey aci (deg).
            # Isareti eps ile ayni -> "kac derece yukari/asagi".
            self.son_dikey_hata = float(eps_deg - eps_hed)
        else:
            self.w_deg[5 * N:6 * N] = 0.0
            self.w_ref[5 * N:6 * N] = 0.0
            self.son_dikey_hata = 0.0

        ex_lim = a.ex_siniri_deg + s_v * (a.vurus_ex_siniri_deg
                                          - a.ex_siniri_deg)
        self.fov_alt[:N] = -ex_lim
        self.fov_ust[:N] = ex_lim
        # dikey satirlar artik beta = ey - kats*vz + C uzerinde
        self.fov_alt[N:] = -(a.fov_ust_bant_deg
                             + s_v * (a.vurus_bant_deg - a.fov_ust_bant_deg))
        self.fov_ust[N:] = (a.fov_alt_bant_deg
                            + s_v * (a.vurus_bant_deg - a.fov_alt_bant_deg))

    def _alan_odulu(self, rbar, r0, vurus=0.0):
        """LINEER bbox alani odulunun DOGRUSALLASTIRILMIS gradyani.

        A ~ K/r^2 oldugundan bagil alan a_k = (r0/rbar_k)^2 ve buyume
        hizi da_k/dt = 2 a_k w1_k / rbar_k. Ikisi de rbar etrafinda
        dogrusallastirilir; maliyete YALNIZ LINEER terim girer, yani
        Hessian ve cozum suresi degismez.

        Dondurulen: (c_r, c_w1) -- yiginlanmis durum uzerindeki lineer
        maliyet katsayilari (J += c_r . r + c_w1 . w1).
        """
        a = self.a
        N = self.N
        rg = np.maximum(rbar, a.menzil_taban_m)
        bagil = (r0 / rg) ** 2                    # A_k / A_0
        carp = np.ones(N)
        carp[-1] = a.p_carpani
        # VURUS: odul (kapanma tesviki) buyutulur.
        odul_carp = 1.0 + float(np.clip(vurus, 0.0, 1.0)) * (
            a.vurus_odul_carpani - 1.0)
        # A/B dugmesi (TO_TEST madde 1): varsayilan 1.0 -> no-op.
        odul_carp *= a.q_alan_carpani
        # d(-q*a_k)/d r_k = +2 q a_k / rbar_k
        c_r = (2.0 * a.q_alan * odul_carp / N) * bagil / rg * carp
        # d(-q*da_k/dt)/d w1_k = -2 q a_k / rbar_k   (1 s zaman olcegi)
        c_w1 = (-2.0 * a.q_alan_hizi * odul_carp / N) * bagil / rg * carp
        return c_r, c_w1

    # ------------------------------------------------------------ coz

    def coz(self, x0, d_ex, d_ey, d_r, eps_deg, ey_ref, beta_c,
            dt0, U_warm=None, u_onceki=None, guven=1.0, bbox_yas_s=0.0,
            irtifa_m=None, d_ex_kutu=None, d_ey_kutu=None,
            yaw_delta_agirlik=None, vurus=0.0, ivme_carpani=None,
            t_go_s=None, apn_a=0.0):
        """Bir MPC adimi cozer.

        x0        : [ex_deg, ey_deg, r_m, w1, w2, w3]  (w: LOS ucayaginda
                    OLCULEN kendi hizimiz)
        d_*       : bozucu kestirimleri (deg/s, deg/s, m/s)
        eps_deg   : hedefin ufka gore yukselisi (LOS ucayagi icin)
        ey_ref    : kadraj merkezi (= -(mount+pitch+aim))
        beta_c    : kadraj degiskeni sabiti; beta_k = ey_k - kats*vz_k
                    + beta_c  (bkz. MpcKontrolcu._kadraj_sabiti)
        bbox_yas_s: son gecerli tespitten beri gecen sure. bayat_kisit_s
                    asilirsa SERT kadraj kisiti BIRAKILIR (ey donmus
                    olacagi icin kisit kendi kendini besler)
        vurus     : VURUS fazi karisim katsayisi [0..1] (bkz. MpcAyar
                    vurus_* bloku). 0 = bugunku maliyet aynen,
                    1 = saf yakalama (bantlar fiziksel kenarda, kadraj
                    ve odul agirliklari yukari, ivme cezasi asagi)
        t_go_s    : KALAN ZAMAN kestirimi [s] ya da None. Kontrolcunun
                    ZATEN hesapladigi menzil_hizi'ndan gelir (bkz.
                    cevre_dikey_tgo); yalnizca dikey_tgo kolunda
                    kullanilir. None = gecersiz -> en yumusak dal.
        apn_a     : hedefin LOS'a DIK IVMESI [m/s^2] ya da 0 (KAPALI,
                    varsayilan -- eski davranis BIT-AYNI). Kontrolcu
                    bunu BozucuKestirici.apn_a_etkin()'den alir (olu
                    bant + guven carpani + kelepce ORADA uygulanmis
                    olur). Bkz. cevre_apn / _apn_d_ex.
        dt0       : ilk adimin OLCULEN suresi
        Dondurur  : (U (nb x NU), bilgi dict)
        """
        t_bas = time.perf_counter()
        a = self.a
        nu_top = self.nu_top
        # _adim_sureleri menzil olceklemesi icin (TO_TEST madde 3) su anki
        # menzili buradan alir; x0[2] = r_m.
        self._son_menzil = float(np.asarray(x0, dtype=float)[2])
        self.cozum_sayaci += 1
        # _fista cagrilmayan koldan (analitik/degrade cozum) eski deger
        # tasinmasin: her cozumde sifirlanir, _fista kendi sonucunu yazar.
        self.son_butce_kesti = 0
        soguk = self.cozum_sayaci <= a.ilk_cozum_sayisi
        it_tavan = a.ilk_iterasyon_tavani if soguk else a.iterasyon_tavani
        butce_s = (a.ilk_butce_ms if soguk else a.sure_butcesi_ms) / 1000.0

        if U_warm is None:
            U = np.zeros(nu_top)
        else:
            U = np.array(U_warm, dtype=float).reshape(-1)
        if u_onceki is None:
            u_onceki = U[:self.NU].copy()
        u_onceki = np.asarray(u_onceki, dtype=float).reshape(self.NU)

        # YAW fark cezasi kazanc programlamasi (bkz. MpcAyar).
        w_yaw = (a.r_delta_yaw if yaw_delta_agirlik is None
                 else float(yaw_delta_agirlik)) / a.olcek_yaw_dps ** 2
        DtSD = self.DtSD_v + w_yaw * self.DtSD_y
        Sd = self.Sd_v + w_yaw * self.Sd_y
        self.son_yaw_delta = w_yaw * a.olcek_yaw_dps ** 2

        h = self._adim_sureleri(dt0)
        r0 = float(x0[2])
        cos_eps = math.cos(math.radians(eps_deg))
        # Dusey hiz kisiti icin: vz_ned = v . [0,0,1]; v LOS ucayaginda
        # verildigi icin katsayilar ucayagin z bilesenleridir. Yaw
        # donusu z'yi degistirmedigi icin heading cercevesi = NED z.
        l_v, e2_v, e3_v = los_ucayak(float(x0[0]), eps_deg)
        a_dik = np.array([l_v[2], e2_v[2], e3_v[2]])
        # heading ILERI ekseninin LOS ucayagindaki bilesenleri
        self._ileri_eks = np.array([l_v[0], e2_v[0], e3_v[0]])
        n_a = float(np.linalg.norm(a_dik))
        a_dik = a_dik / n_a if n_a > 1e-9 else np.array([0.0, 0.0, 1.0])

        # fark cezasi offseti: ilk blokta u_onceki (agirliksiz; agirlik
        # Sd ile ayrica uygulaniyor)
        o_vec = np.zeros(nu_top)
        o_vec[:self.NU] = u_onceki

        N = self.N
        s_v = float(np.clip(vurus, 0.0, 1.0)) if a.vurus_modu else 0.0
        self.son_vurus = s_v
        # IVME (yatma) CEZASI CARPANI. Varsayilan VURUS karisimindan
        # gelir (vurus_ivme_carpani; varsayilan 1.0 -> no-op). ivme_kos
        # __init__'te bir kez kuruldugu icin burada yalnizca skaler bir
        # carpma kalir (ek tahsis yok). ivme_carpani disaridan da
        # verilebilir (ablasyon); DEVIR RAMPASI tur-2 sim'inde elenip
        # geri alindi (bkz. MpcAyar devir_ivme yorumu).
        if ivme_carpani is None:
            ivme_carpani = 1.0 + s_v * (a.vurus_ivme_carpani - 1.0)
        self.son_ivme_carpani = float(ivme_carpani)
        ivme_kos = self.ivme_kos * float(ivme_carpani)
        # DIKEY TERMINAL RAMPASI (env dugmeli). Menzille kurulur, VURUS
        # karisimiyla DEGIL: amaç hizalamayi son saniyeden ~3 s'lik
        # pencereye yaymak. BAYAT BBOX KAPISI: eps = -(ey + aim) ve ey
        # donmus bir olcumden geliyorsa dikey talep kendi kendini besler
        # (ayni tuzak vurus_kor_suzulme'de olculdu), o yuzden bayatta 0.
        # KOL SECIMI: rampa dikey_terminal VEYA dikey_tgo acikken kurulur;
        # ana kol (P + TGO) eski D dugmesini ACMADAN calissin diye.
        dikey_s = 0.0
        if (a.dikey_terminal or a.dikey_tgo) and bbox_yas_s <= a.bayat_kisit_s:
            genis = max(a.dikey_terminal_menzil_m - a.dikey_terminal_tam_m,
                        1e-6)
            dikey_s = float(np.clip(
                (a.dikey_terminal_menzil_m - r0) / genis, 0.0, 1.0))
        # t_go SEKILLI TAU (env dugmeli; bkz. cevre_dikey_tgo).
        # tau_eff = clip(t_go / k, tau_min, tau_max). KAPALIYKEN None
        # doner -> _maliyet_satirlari eski SABIT tau'yu kullanir, yani
        # kapali kol BIT-AYNI.
        # GUVENLI TARAF: t_go verilmemis/gecersiz/negatif/NaN ise ya da
        # tavani asiyorsa tavan degeri kullanilir -> tau_eff tau_max'a
        # oturur, yani EN YUMUSAK talep. Agresif tarafa asla dusulmez.
        dikey_tau = None
        t_go_tani = float('nan')
        if a.dikey_tgo:
            k_eff = max(a.dikey_tgo_k * a.dikey_tgo_carpani, 1e-3)
            tg = a.dikey_tgo_tavan_s
            if t_go_s is not None:
                tg_f = float(t_go_s)
                if math.isfinite(tg_f) and tg_f > 0.0:
                    tg = min(tg_f, a.dikey_tgo_tavan_s)
            t_go_tani = tg
            dikey_tau = float(np.clip(tg / k_eff, a.dikey_tgo_tau_min_s,
                                      a.dikey_tgo_tau_max_s))
        self.son_tgo = t_go_tani
        self.son_dikey_tau = (float('nan') if dikey_tau is None
                              else float(dikey_tau))
        # DOGRUDAN DIKEY HATA (P) RAMPASI -- ayri env dugmesi, ayni
        # gerekce (bkz. cevre_dikey_hata). AYRI degisken tutuluyor cunku
        # iki dugme BAGIMSIZ acilabiliyor ve rampalarinin parametreleri
        # de ayri ayri ayarlanabilsin. BAYAT BBOX KAPISI ayni: eps donmus
        # bir olcumden geliyorsa terim kendi kendini besler.
        dikey_hata_s = 0.0
        if a.dikey_hata and bbox_yas_s <= a.bayat_kisit_s:
            genis_h = max(a.dikey_hata_menzil_m - a.dikey_hata_tam_m, 1e-6)
            dikey_hata_s = float(np.clip(
                (a.dikey_hata_menzil_m - r0) / genis_h, 0.0, 1.0))
        # APN (bkz. cevre_apn): kol KAPALIYSA apn_e tam 0'dir ve asagidaki
        # iki fonksiyonda ilgili dal HIC calismaz -> BIT-AYNI.
        apn_e = float(apn_a) if a.apn else 0.0
        if not math.isfinite(apn_e):
            apn_e = 0.0
        self.son_apn_a = apn_e
        gecis = max(1, int(a.sqp_gecis))
        for _ in range(gecis):
            rbar, wbar, bb = self._nominal_yorunge(
                x0, U, h, d_ex, d_ey, d_r, cos_eps, r0, apn_a=apn_e)
            # DOYUMLU EYLEYICI: adim-basi yatay kazanci NOMINAL yorungeden
            # AYNEN al (ayni dogrusallastirma noktasi sart). Kol kapaliysa
            # _nominal_yorunge None birakir -> eski yol BIT-AYNI.
            Xf, Gam, c2, c3, dex_v, dey_v = self._yorunge_matrisleri(
                x0, h, rbar, cos_eps, d_ex, d_ey, d_r, r0, apn_a=apn_e,
                al_h=self._al_h, al_v=self._al_v)
            self._maliyet_satirlari(c2, c3, dex_v, dey_v, ey_ref, guven,
                                    vurus=s_v, eps_deg=eps_deg,
                                    dikey_s=dikey_s,
                                    dikey_hata_s=dikey_hata_s,
                                    dikey_tau_s=dikey_tau)

            # M = W Gam (W satir basina TEK sifirdisi -> topla-carp)
            M = self.w_deg[:, None] * Gam[self.w_sut, :]
            b = self.w_deg * Xf[self.w_sut] - self.w_ref

            H = 2.0 * (M.T @ M + DtSD)
            H[np.diag_indices(nu_top)] += 2.0 * (self.R_kos + self.prox_kos
                                                 + ivme_kos)
            f = 2.0 * (M.T @ b - self.D.T @ (Sd * o_vec)
                       - self.prox_kos * U - ivme_kos * wbar)
            # LINEER ALAN ODULU: yalniz gradyana girer (Hessian yok)
            c_r, c_w1 = self._alan_odulu(rbar, r0, vurus=s_v)
            f += Gam[self.sut_r, :].T @ c_r
            f += Gam[self.sut_w[0], :].T @ c_w1

            # --- FOV satirlari (PLAN katmani, l1 tam ceza) ---
            # yatay: ex_k ; dikey: beta_k = ey_k - kats*(a_dik.w_k) + C
            kats = self.kats
            Gb = Gam[self.sut_ey, :] - kats * (
                a_dik[0] * Gam[self.sut_w[0], :]
                + a_dik[1] * Gam[self.sut_w[1], :]
                + a_dik[2] * Gam[self.sut_w[2], :])
            Xb = Xf[self.sut_ey] - kats * (
                a_dik[0] * Xf[self.sut_w[0]] + a_dik[1] * Xf[self.sut_w[1]]
                + a_dik[2] * Xf[self.sut_w[2]]) + beta_c
            Gf = np.vstack([Gam[self.sut_ex, :], Gb])
            Xf_f = np.concatenate([Xf[self.sut_ex], Xb])

            # --- ONKOSULLAMA (Jacobi, blok-uniform) -------------------
            # Hiz uclusu icin ORTAK carpan kullanmak zorundayiz (kure
            # kure kalsin, elipsoid olmasin -> izdusum kapali formda
            # kalsin); yaw ayri olceklenir. Asil kazanci birimleri
            # (m/s ve deg/s) esitlemek ve L'yi olcekten bagimsiz
            # kilmaktir.
            kv = np.diag(H).reshape(self.nb, self.NU)
            pv = 1.0 / np.sqrt(np.maximum(kv[:, :3].mean(axis=1), 1e-9))
            py = 1.0 / np.sqrt(np.maximum(kv[:, 3], 1e-9))
            P = np.empty(nu_top)
            P.reshape(self.nb, self.NU)[:, :3] = pv[:, None]
            P.reshape(self.nb, self.NU)[:, 3] = py

            Hqs = (P[:, None] * H) * P[None, :]
            fs = P * f
            Gfs = Gf * P[None, :]

            # --- ADIM BOYU: aktif-kume farkindaligi ---
            # FOV cezasinin Hessian'i tek basina lambda_max'i ~145 kat
            # sisiriyor (olculdu). L'ye YALNIZ ihlale yakin satirlarin
            # katkisi konur; boylece ceza pasifken adim tam boy olur.
            z0 = U / P
            Ltop = Hqs
            if np.any(self.fov_etkin):
                # Huber'in EGRILIGI YALNIZ KINK CIVARINDA vardir:
                # |ihlal| > delta bolgesi DOGRUSALDIR (egrilik 0), ihlal
                # yokken de terim pasiftir. "Sinira yakin" diye genis bir
                # kume almak L'yi 1.6e3'e cikariyordu ve FISTA 30
                # iterasyonda hicbir yere gidemiyordu (olculdu: u1 2.9
                # kalirken gercek optimum 14.2). Bu yuzden olcut KINK'e
                # UZAKLIKTIR.
                y_w = Xf_f + Gfs @ z0
                pay = a.fov_uyanik_pay_deg
                uyanik = self.fov_etkin & (
                    (np.abs(y_w - self.fov_ust) < pay)
                    | (np.abs(y_w - self.fov_alt) < pay))
                if np.any(uyanik):
                    olc = np.sqrt(self.fov_rho[uyanik] / a.fov_l1_delta_deg)
                    Ga = olc[:, None] * Gfs[uyanik, :]
                    Ltop = Hqs + (Ga.T @ Ga)
            L = 1.4 * max(float(np.linalg.eigvalsh(Ltop)[-1]), 1e-9)

            # --- SERT FOV -> girdi kutusu/dilimi (CBF) ---
            # UC KAPI: (1) bbox bayatsa daraltma yok (kacak dongu),
            # (2) kisit ~0.5 s karsilanamadiysa birak (cakilma dersi),
            # (3) ablasyon anahtari.
            bayat = bbox_yas_s > a.bayat_kisit_s
            self.son_fov_serbest = 0
            if bayat:
                self.son_fov_serbest = 1
            elif self._birakildi:
                self.son_fov_serbest = 2
            if a.fov_sert and not self.son_fov_serbest:
                (vz_a, vz_u, yaw_a, yaw_u, beta_simdi, self.son_bant_alt,
                 self.son_bant_ust, doygun) = self._cbf_sinirlari(
                    bb, a_dik, beta_c,
                    d_ex if d_ex_kutu is None else d_ex_kutu,
                    d_ey if d_ey_kutu is None else d_ey_kutu, r0, rbar,
                    vurus=s_v, tau_v=self._tau_v_blok)
                self.son_beta = float(beta_simdi[0])
                # HISTEREZIS (yukari): karsilanamayan dongulerde birikir,
                # karsilanabilir olunca temizlenir.
                if doygun:
                    self.bos_sayac += 1
                    if self.bos_sayac > a.bos_kume_tavan_dongu:
                        self._birakildi = True
                else:
                    self.bos_sayac = 0
            else:
                # KISIT BIRAKILDI: yalniz FIZIKSEL tavanlar. Kadraj
                # kaybi kabul edilir -- karar verici 1.5 s dwell
                # sonrasi 'konumlu'ya doner ve yeniden konumlandirir.
                vz_a = np.full(self.nb, -a.tirmanma_tavani_mps)
                vz_u = np.full(self.nb, a.alcalma_tavani_mps)
                yaw_a = np.full(self.nb, -a.yaw_hiz_tavani_dps)
                yaw_u = np.full(self.nb, a.yaw_hiz_tavani_dps)
                self.son_bant_alt = a.fov_alt_bant_deg
                self.son_bant_ust = a.fov_ust_bant_deg
                self.son_beta = float(x0[1] + beta_c
                                      - self.kats
                                      * float(a_dik @ x0[3:6]))
                # LATCH HATASI DUZELTMESI (2026-08-04, tur-3 analizi):
                # Burada sayac ESKIDEN hic sonumlenmiyordu; bir kez
                # tavani asinca (41) sonsuza dek orada kaliyor ve sert
                # FOV kisiti kosunun %59-75'inde KALICI KAPALI oluyordu
                # (yalniz bbox bayatlamasi sifirliyordu). Yani
                # kullanicinin istedigi sert kisit fiilen devre disiydi.
                # Artik birakilmis durumda sayac her dongu SONUMLENIR ve
                # geri_esik'in altina inince kisit YENIDEN DEVREYE
                # GIRER. Histerezis bandi (tavan -> geri_esik) rejimler
                # arasinda ani gidip gelmeyi engeller.
                if self.son_fov_serbest == 2:
                    self.bos_sayac = max(0, self.bos_sayac - 1)
                    if self.bos_sayac <= a.bos_geri_esik_dongu:
                        self._birakildi = False
                else:
                    self.bos_sayac = 0      # bayat/ablasyon sayaci sifirlar
            # HEDEF-ALTI DERINLIK TAVANI: +30 montajda kadraj cost'u
            # eksen-alti hedefi merkeze cekmek icin tabana kadar
            # daliyordu (mount 0'da bu mekanizma YOK, tavan emniyet
            # artigi olarak duruyor -- bkz. MpcAyar.derinlik_tavani_m).
            # Keser: hedefin altinda en fazla derinlik_tavani_m.
            # Hedef-alti derinlik = r*sin(eps) (eps>0: hedef yukarida).
            # TEK YANLI: yalniz alcalmayi (vz_u) keser.
            if a.dikey_derinlik_tavani and eps_deg > 0.0:
                derinlik = r0 * math.sin(math.radians(eps_deg))
                vz_tavan_d = np.clip(
                    (a.derinlik_tavani_m - derinlik) / a.derinlik_yaklasma_s,
                    0.0, a.alcalma_tavani_mps)
                vz_u = np.minimum(vz_u, vz_tavan_d)
                vz_a = np.minimum(vz_a, vz_u)
            # IRTIFA TABANI: her kosulda (kisit birakilmis olsa bile)
            if irtifa_m is not None:
                vz_tavan = np.clip(
                    (float(irtifa_m) - a.irtifa_taban_m) / a.irtifa_yaklasma_s,
                    0.0, a.alcalma_tavani_mps)
                vz_u = np.minimum(vz_u, vz_tavan)
                vz_a = np.minimum(vz_a, vz_u)
            self._son_cbf = (float(vz_a[0]), float(vz_u[0]),
                             float(yaw_a[0]), float(yaw_u[0]))
            # HIZLANMA RAMPASI: kure yaricapi blok basina
            #   min(tavan, max(|w_k|, taban) + a_ileri * tau)
            # (bkz. MpcAyar.ileri_ivme_tavani_mps2). tau eyleyici
            # gecikmesidir: komut-hiz farki (u - w) tau uzerinden
            # ivmeye donusur, yani a*tau tam olarak "bir zaman
            # sabitinde ne kadar hizlanabilirim"dir.
            if a.ileri_ivme_tavani_mps2 > 0.0:
                w_blok = np.linalg.norm(
                    wbar.reshape(self.nb, self.NU)[:, :3], axis=1)
                v_tav_blok = np.minimum(
                    a.hiz_tavani_mps,
                    np.maximum(w_blok, a.hiz_artis_taban_mps)
                    + a.ileri_ivme_tavani_mps2 * max(a.hiz_gecikme_tau_s, 1e-3))
            else:
                v_tav_blok = np.full(self.nb, a.hiz_tavani_mps)
            self.son_v_tavan = float(v_tav_blok[0])
            sinir = {
                "v": v_tav_blok / pv,
                "vz_alt": vz_a / pv,
                "vz_ust": vz_u / pv,
                "yaw_alt": yaw_a / py,
                "yaw_ust": yaw_u / py,
            }
            z = self._fista(z0, Hqs, fs, Gfs, Xf_f, L, a_dik, t_bas,
                            sinir, P, it_tavan, butce_s)
            U = P * z
            self.son_maliyet = float(0.5 * z @ (Hqs @ z) + fs @ z
                                     + b @ b)

        Ur = U.reshape(self.nb, self.NU)
        self.son_sure_ms = (time.perf_counter() - t_bas) * 1000.0
        bilgi = {
            "sure_ms": self.son_sure_ms,
            "iterasyon": self.son_iterasyon,
            "butce_kesti": self.son_butce_kesti,
            "maliyet": self.son_maliyet,
            "rbar_son": float(rbar[-1]),
            "L": L,
            "beta": self.son_beta,
            "yaw_delta": self.son_yaw_delta,
            "bant_alt": self.son_bant_alt,
            "bant_ust": self.son_bant_ust,
            "vurus": self.son_vurus,
            "v_tavan": self.son_v_tavan,
            "ivme_carpani": self.son_ivme_carpani,
            "hiza_ref": self.son_hiza_ref,
            "tgo": self.son_tgo,
            "dikey_tau": self.son_dikey_tau,
            "dikey_hata": self.son_dikey_hata,
            "apn_a": self.son_apn_a,
            "tau_eff": self.son_tau_eff,
            "tau_eff_z": self.son_tau_eff_z,
            "fov_serbest": self.son_fov_serbest,
            "bos_sayac": self.bos_sayac,
            "cbf": self._son_cbf,
        }
        return Ur, bilgi

    def _fista(self, z0, H, f, Gf, Xf_f, L, a_dik, t_bas, sinir, P,
               it_tavan, butce_s):
        """Hizlandirilmis izdusumlu gradyan (FISTA) + uyarlanabilir
        yeniden baslatma. Onkosullanmis (z) uzayda calisir; durma
        olcutu ise FIZIKSEL (U = P z) uzayda okunur."""
        a = self.a
        alt_w, ust_w = self.fov_alt, self.fov_ust
        rho_v = self.fov_rho
        delta = max(a.fov_l1_delta_deg, 1e-6)
        tol_ag = self.tol_agirlik * P
        sv = sinir["v"]
        s_alt, s_ust = sinir["vz_alt"], sinir["vz_ust"]
        y_alt, y_ust = sinir["yaw_alt"], sinir["yaw_ust"]

        GfT = Gf.T
        adim_boy = 1.0 / L

        Y = z0.copy()
        Zo = z0.copy()
        t = 1.0
        it = 0
        yakinsadi = False       # LOG-ONLY: durma olcutuyle mi durduk?
        for it in range(1, it_tavan + 1):
            y = Xf_f + Gf @ Y
            ihlal = np.clip(y - ust_w, 0.0, None)
            ihlal += np.clip(y - alt_w, None, 0.0)
            g = H @ Y + f
            # HUBER-l1 TAM CEZA gradyani: |ihlal| > delta bolgesinde
            # SABIT rho (l1), altinda dogrusal (kuadratik ceza) ->
            # turevlenebilir kalir, birinci mertebe yontem gecerli.
            np.clip(ihlal, -delta, delta, out=ihlal)
            ihlal *= rho_v
            g += (1.0 / delta) * (GfT @ ihlal)
            Zn = _izdusum_kure_dilim(Y - adim_boy * g, sv, a_dik, s_alt,
                                     s_ust, y_alt, y_ust)
            fark = Zn - Zo
            # uyarlanabilir yeniden baslatma (gradyan olcutu): momentum
            # ters yone calisiyorsa t sifirlanir -- sabit momentumda
            # gorulen salinimi keser, adim boyu iyimser secildiginde de
            # iraksamaya karsi emniyettir
            if float(np.dot(Y - Zn, fark)) > 0.0:
                t = 1.0
                Y = Zn.copy()
            else:
                tn = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
                Y = Zn + ((t - 1.0) / tn) * fark
                t = tn
            adim = float(np.max(np.abs(fark) * tol_ag))
            Zo = Zn
            if it >= a.iterasyon_tabani and adim < a.tolerans_mps:
                yakinsadi = True
                break
            if (time.perf_counter() - t_bas) > butce_s:
                break
        self.son_iterasyon = it
        # LOG-ONLY BAYRAK: yakinsamadan ciktiysak ya sure butcesi ya da
        # iterasyon tavani kesmistir; ikisi de "bu dongude optimal olmayan
        # bir komut verdik" demektir. Kontrol yolu bu degeri OKUMAZ.
        self.son_butce_kesti = 0 if yakinsadi else 1
        return Zo


# ========================================================= KONTROLCU

class MpcKontrolcu(GoruntuluKontrolcu):
    """goruntulu_temel sozlesmesine uyan MPC yontemi."""

    ad = "mpc"

    def __init__(self, ayar: MpcAyar = None, tani_log=None):
        self.a = ayar or MpcAyar()
        self.cozucu = MpcCozucu(self.a)
        self.bozucu = BozucuKestirici(self.a)
        self.tani_log_yolu = tani_log
        self._tani_f = None
        self._tani = None
        self._redis = None          # ISKA kopru yayini (istege bagli);
                                    # sifirla()'da DEGIL: baglanti
                                    # devirler arasinda korunur
        self.sifirla()

    # ------------------------------------------------------- durum

    def sifirla(self):
        self.r_ic = None                     # ic menzil durumu
        # KOR PN (bkz. cevre_kor_pn): olu hesapla ilerletilen kadraj
        # durumu ve korlugun ne kadar surdugu.
        self._kor_ex = None                  # ilerletilen ex [deg]
        self._kor_ey = None                  # ilerletilen ey [deg]
        self._kor_t = 0.0                    # kesintisiz korluk suresi [s]
        self._kor_ex0 = None                 # son GERCEK olcumun |ex|'i (R3)
        self.kor_pn_kosdu = 0                # tani: bu dongu ilerletildi mi
        self.U = None                        # son cozum (nb x NU)
        # None = "henuz komut vermedik". Ilk komut()ta devir hiziyla
        # tohumlanir (bkz. MpcAyar.devir_prox_tohum); LOS ucayagi ilk
        # olcumden once bilinmedigi icin tohumlama BURADA yapilamaz.
        self.u_onceki = None
        self.v_ned_tohum = None
        self.pitch_lpf = None
        self.alan = None
        self.alan_hizi = 0.0
        self.yaw_uygulanan = 0.0
        self._yaw_onceki = None
        self.yaw_hiz_lpf = 0.0
        self._yaw_n = 0                  # yaw hizi LPF ornek sayaci
        self.yaw_agirlik_lpf = None      # yaw fark cezasi kazanc LPF'si
        self.bozucu.sifirla()
        self.sayac = 0
        # --- ISKA DURUM MAKINESI (bkz. MpcAyar iska_* bloku) ---
        # Hepsi BURADA sifirlanir: tohumla() sifirla()'yi cagirdigi icin
        # her yeni devir TAZE baslar (en_iyi_menzil ve gecis bayraklari
        # onceki angajmandan TASINMAZ -- tasinsalardi ikinci devir daha
        # dogar dogmaz "menzil aciliyor" diye kendini iptal ederdi).
        self.durum = 'KAPANMA'           # KAPANMA | TERMINAL | VURUS | ISKA
        self.vurus_karisim = 0.0         # VURUS fazi karisimi [0..1]
        self.vuruldu = False             # VURUS_BASARILI ilan edildi mi (latch)
        self.vurus_vibe = 0.0            # ilan anindaki vibe
        self.vurus_menzil = 0.0          # ilan anindaki menzil [m]
        self._bekleyen_olay = None       # Komut'a takilacak ayrik olay
        self.kor_dongu = 0               # VURUS'ta kor suzulen dongu sayisi
        self.en_iyi_menzil = float('inf')
        self.menzil_hizi = 0.0           # d(r_ic)/dt, LPF [m/s]
        self.gecildi = False             # gecis (pass) onaylandi mi
        self.iska_sebep = ''
        self._r_onceki = None
        self._gecis_sayac = 0
        self._gecis_alan_sayac = 0
        self._kapanma_tepe = 0.0         # gecis cemberi icindeki en
                                         # negatif menzil hizi [m/s]
        self._yetki_t0 = None
        # ILERLEME SAATI (bkz. cevre_ilerleme_saat). Angajman basi
        # sifirlanir: onceki devrin durgunlugu tasinmaz.
        self._durgunluk_saat = 0.0       # ISKA'yi atesleyen saat [s]
        self._menzil_izi = deque()       # (t, en_iyi) penceresi
        # deque: popleft O(1). Liste + pop(0) durum makinesini
        # 21 us/donguye cikarmisti (kilit 20 us).
        self._son_v_ned = None           # ISKA suzulmesi icin son komut

    def tohumla(self, devir):
        """Devir aninda bir kez. Konumlunun son hiz komutu WARM-START'tir:
        ilk cozum sifirdan degil, aracin O ANKI hareketinden baslar."""
        self.sifirla()
        if devir and 'cmd_vel_ned' in devir:
            self.v_ned_tohum = np.asarray(devir['cmd_vel_ned'], dtype=float)
        print(f"[mpc] tohumlandi, devir hizi="
              f"{None if self.v_ned_tohum is None else np.round(self.v_ned_tohum, 2).tolist()}")

    # ------------------------------------------------- yardimcilar

    def _menzil(self, olcum, w1, dt):
        """Ic menzil durumu: model ile ilerlet, olcum varsa ona cek.

        SADECE Olcum.menzil_m kullanilir (bbox genisligi menzil vekili
        DEGIL: virajda 204 m'de 31 px, arkadan 26 m'de 34 px olctuk)."""
        if self.r_ic is None:
            self.r_ic = (float(olcum.menzil_m) if olcum.menzil_m is not None
                         else self.a.menzil_yoksa_m)
            return self.r_ic
        self.r_ic += dt * (-w1)              # model ilerletme
        if olcum.menzil_m is not None:
            self.r_ic += self.a.menzil_olcum_kazanci * (
                float(olcum.menzil_m) - self.r_ic)
        self.r_ic = float(max(self.a.menzil_taban_m * 0.5, self.r_ic))
        return self.r_ic

    def _kadraj_sabiti(self, olcum, vz_simdi):
        """Kadraj degiskeni beta = ey - kats*vz + C icin C sabiti.

        Kamera ekseninin ufka gore yukselisi = mount + govde_pitch.
        beta hedefin bu EKSENE gore dikey sapmasidir (+ = eksenin
        ALTINDA). Tanim geregi:
            beta = ey - ey_ref,  ey_ref = -(mount + pitch + aim)
        Tirmanma pitch'i yukari ittigi icin pitch, ufuk boyunca
            pitch_k = pitch_olculen - kats*(vz_k - vz_0)
        ile ONGORULUR. Bunu beta'ya koyunca:
            beta_k = ey_k - kats*vz_k + C,
            C = mount + aim + pitch_olculen + kats*vz_0
        DIKKAT: C, MUTLAK bir pitch fitine degil OLCULEN pitch'e
        demirlenir; boylece k=0'da beta tam olarak olculen sapmadir ve
        fit hatasi yalnizca ufuk boyunca DEGISIM terimini etkiler.
        Pitch LPF'lenir (ham pitch salindigi icin bant titremesin).

        GIMBAL (pitch_baglasimi=False): eksen govdeden BAGIMSIZ, yani
        ey_ref = -(mount + aim) ve tirmanma terimi duser. pitch_lpf
        yine de guncellenir -- tani logunda okunuyor ve anahtar
        acilip kapandiginda gecmis bozulmasin."""
        a = self.a
        pitch_deg = (math.degrees(olcum.pitch_rad)
                     if olcum.pitch_rad is not None else 0.0)
        pitch_deg = float(np.clip(pitch_deg, a.pitch_alt_deg, a.pitch_ust_deg))
        if self.pitch_lpf is None:
            self.pitch_lpf = pitch_deg
        else:
            dt = float(np.clip(olcum.dt, 0.02, 0.30))
            k = dt / (dt + a.pitch_lpf_tau_s)
            self.pitch_lpf += k * (pitch_deg - self.pitch_lpf)
        eksen_pitch = self.pitch_lpf if a.pitch_baglasimi else 0.0
        kats = a.pitch_tirmanma_kats if a.pitch_baglasimi else 0.0
        # FAZ C (gimbal dali): tilt artik DINAMIK -- bbox hedefin yukselisini
        # izliyor ve o karede kullandigi gercek elevasyonu tracker_bbox_stab[7]
        # ile yayinliyor (Olcum.tilt_deg). ey_ref statik YILDIZ_TILT yerine
        # CANLI degerden kurulur; alan yoksa (eski kayit / tilt kapali)
        # statik mount_pitch_deg'e duser.
        tilt_canli = getattr(olcum, 'tilt_deg', None)
        eksen_taban = (float(tilt_canli) if tilt_canli is not None
                       else a.mount_pitch_deg)
        ey_ref = -(eksen_taban + eksen_pitch + a.aim_deg)
        beta_c = -ey_ref + kats * vz_simdi
        return ey_ref, beta_c


    def _vurus_basarili_kontrol(self, olcum, r):
        """FIZIKSEL TEMAS tespiti: KENDI vibrasyonumuz + menzil.

        Bkz. MpcAyar.vurus_basari_* -- esikler tur-3 sim kosusundan
        olculdu (gercek temas 17.4-25.5 vs temassiz gecis 3.3).
        LATCH'li: angajman basina bir kez. Dondurulen (olay, detay)
        ciftini komut() Komut'a koyar; iskelet _olay.csv'ye yazar ve
        goruntulu.log'a basar.

        Hedef telemetrisi KULLANILMAZ: vibe bizim VIBRATION mesajimiz,
        menzil zaten izinli tek hedef buyuklugu.
        """
        a = self.a
        if (not a.vurus_basari_tespiti or self.vuruldu
                or olcum.vibe_max is None):
            return None
        # MENZIL: OLCULEN deger kullanilir, ic suzgec durumu (r_ic) DEGIL.
        # TUZAK (2026-08-05'te yakalandi): _menzil() ic durumu
        # max(menzil_taban_m*0.5, ...) = 3.0 m ile TABANLANIR (c = KDEG/r
        # katsayisi r->0'da patladigi icin sayisal koruma). Yani r_ic
        # ASLA 3.0 m'nin altina inmez ve "r < 3 m" kapisi r_ic ile ASLA
        # acilmaz -- tespit sessizce hic atesletmezdi. Temas FIZIKSEL bir
        # olaydir; olculen menzille yargilanir, cozucunun sayisal
        # tabanlanmis kopyasiyla degil.
        r_olc = float(olcum.menzil_m) if olcum.menzil_m is not None else float(r)
        if (float(olcum.vibe_max) > a.vurus_basari_vibe
                and r_olc < a.vurus_basari_menzil_m):
            self.vuruldu = True
            self.vurus_vibe = float(olcum.vibe_max)
            self.vurus_menzil = r_olc
            detay = (f"vibe={self.vurus_vibe:.1f} (esik "
                     f"{a.vurus_basari_vibe:.0f}) menzil={r_olc:.2f} m "
                     f"durum={self.durum} vurus={self.vurus_karisim:.2f}")
            print(f"[mpc] VURUS_BASARILI: {detay}")
            return ('vurus_basarili', detay)
        return None

    def _yaw_delta_agirlik(self, d_ex, r, dt):
        """Yaw fark cezasinin KAZANC PROGRAMLAMASI (bkz. MpcAyar).

        Olcut HEDEFIN LOS'A DIK HIZIDIR: v_dik = |d_ex| * r / KDEG.
        Bozucu kestirimi kendi yaw hizimizi cikardigi icin bu sayi
        DISSALDIR (hedef manevrasi), yani kazanc programlamasi kontrol
        dongusune geri beslenmez; menzile bolundugu icin de yakin
        menzilde kendiliginden sertlesmez. Ikisi de olcumle elenen
        onceki olcutlerin (|ex| ve ham |d_ex|) kusurlariydi.

        Agirlik ayrica LPF'lenir: kazancin kendisi dongu basina
        ziplarsa QP'nin cozumu de ziplar (chatter'in ta kendisi).
        """
        a = self.a
        v_dik = abs(float(d_ex)) * max(float(r), a.menzil_taban_m) / KDEG
        s = float(np.clip(
            (v_dik - a.yaw_serbest_vperp_alt)
            / max(a.yaw_serbest_vperp_ust - a.yaw_serbest_vperp_alt, 1e-6),
            0.0, 1.0))
        ham = a.r_delta_yaw + s * (a.r_delta_yaw_serbest - a.r_delta_yaw)
        if self.yaw_agirlik_lpf is None:
            self.yaw_agirlik_lpf = ham
        else:
            k = dt / (dt + max(a.yaw_agirlik_tau_s, 1e-6))
            self.yaw_agirlik_lpf += k * (ham - self.yaw_agirlik_lpf)
        return self.yaw_agirlik_lpf

    def _alan_guncelle(self, olcum, dt):
        """LINEER bbox alani (w*h, px^2) ve buyume hizi (px^2/s).

        Odul tanimi karekok DEGIL alanin kendisidir; alan ~ K/r^2
        oldugu icin yakin menzil kareyle odullendirilir. Buyume hizi
        yaklasma hizinin GORUNTU tarafindaki vekilidir (telemetriye
        dokunmaz)."""
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

    def _t_go(self):
        """KALAN ZAMAN [s] ya da None -- YENI KESTIRIM DEGIL.

        Girdileri, kontrolcunun ZATEN tuttugu iki sayidir:
          * self._r_onceki'nin guncellendigi ic menzil r (izinli tek
            hedef olcusu),
          * self.menzil_hizi = d(r_ic)/dt, LPF tau 0.30 s -- ayni sayi
            gecis kapanma-hizi sartinda da kullaniliyor
            (bkz. _durum_makinesi ve gecis_kapanma_esigi_mps).
        Yani "hedeften yalniz menzil" kurali ihlal edilmiyor ve yeni bir
        gozlemci/kestirici EKLENMIYOR.

        NEDEN None DONEBILIR: menzil_hizi negatif = kapaniyoruz. Kapanma
        esigin altindaysa (acilma, capraz geometri, LPF isinmasi) t_go
        ya sonsuz ya anlamsizdir. None doner ve coz() en YUMUSAK dala
        (tau_max) duser -- gecersiz veriden AGRESIF talep uretilmez.
        _durum_makinesi coz()'den ONCE cagrildigi icin menzil_hizi
        tazedir.
        """
        a = self.a
        kapanma = -float(self.menzil_hizi)
        if (self._r_onceki is None
                or kapanma < a.dikey_tgo_kapanma_min_mps):
            return None
        return float(self._r_onceki) / kapanma

    # ------------------------------------------- ISKA durum makinesi

    def _durum_makinesi(self, olcum, r, alan_hizi, dt):
        """KAPANMA -> TERMINAL -> ISKA. MPC'nin USTUNDE, maliyetin DISINDA.

        Neden maliyette degil: bkz. MpcAyar iska_* bloku -- "gectim,
        birakmaliyim" bir optimum degil bir SONLANDIRMA kararidir.

        Maliyeti: ~20 kayan nokta islemi + iki sayac. Cozucu butcesi
        (p95 ~8.6 ms / tavan 13 ms) yaninda olculemez; ISKA ilan
        edildikten sonra ise cozucu HIC KOSMADIGI icin dongu maliyeti
        DUSER.

        Bu fonksiyon HEDEF TELEMETRISINE DOKUNMAZ: girdileri r (menzil,
        izinli tek hedef olcusu), alan_hizi (bbox'tan) ve kendi
        saatimizdir. Hedef hizi TURETILMEZ.
        """
        a = self.a
        if self._yetki_t0 is None:
            self._yetki_t0 = olcum.t
        gecen = float(olcum.t - self._yetki_t0)

        # --- menzil hizi: r_ic turevi + LPF (ham r_olcum turevi
        #     kullanilamaz, +-30 m/s gurultu verir; bkz. MpcAyar) ---
        if self._r_onceki is not None:
            ham = float(np.clip((r - self._r_onceki) / dt, -60.0, 60.0))
            k = dt / (dt + max(a.menzil_hizi_tau_s, 1e-6))
            self.menzil_hizi += k * (ham - self.menzil_hizi)
        self._r_onceki = float(r)
        if r < self.en_iyi_menzil:
            self.en_iyi_menzil = float(r)

        # --- ILERLEME SAATI (bkz. cevre_ilerleme_saat) ---
        # TAMAMI BAYRAGIN ARKASINDA. Once bayraktan BAGIMSIZ yazilmisti
        # ("taban kosularinda ardil okunsun" diye) ama mpc_test'in
        # "durum makinesi UCUZ (< 20 us/dongu)" kilidini KIRDI: 21.0 us
        # olculdu. Kapali kolun tek bir mikrosaniyesi bile bir tani
        # kolaylıgı icin harcanmaz -- ustelik ardil analiz zaten
        # mumkun: r ve t goruntulu CSV'de var, "kol acik olsaydi"
        # sorusu offline yeniden hesaplanabilir.
        # ILERLEME OLCUSU = EN IYI MENZILIN (best-so-far) IYILESME HIZI.
        # Iz MONOTON AZALAN bir sinyaldir, bu yuzden SALINIMLA
        # KANDIRILAMAZ -- olcumu icin bkz. ilerleme_kapanma_esigi_mps.
        if a.ilerleme_saat:
            iz = self._menzil_izi
            iz.append((float(olcum.t), float(self.en_iyi_menzil)))
            while len(iz) > 2 and olcum.t - iz[0][0] > a.ilerleme_pencere_s:
                iz.popleft()
            t_eski, en_iyi_eski = iz[0]
            sure = olcum.t - t_eski
            kazanim = ((en_iyi_eski - self.en_iyi_menzil) / sure
                       if sure >= 0.5 * a.ilerleme_pencere_s else 0.0)
            ilerliyor = kazanim > a.ilerleme_kapanma_esigi_mps
            self._durgunluk_saat = max(
                0.0, self._durgunluk_saat
                + dt * ((1.0 - a.ilerleme_kazanci) if ilerliyor else 1.0))

        # --- FAZ (KAPANMA -> TERMINAL -> VURUS) ---
        # ISKA'dan BAGIMSIZ: bunlar GUDUM fazlaridir, sonlandirma
        # karari degil. Eskiden faz atamasi iska_modu kapisinin
        # ARKASINDAYDI; --no-iska ablasyonunda faz hic degismezdi,
        # yani ablasyon iki seyi birden kapatiyordu.
        if self.durum != 'ISKA':
            if self.en_iyi_menzil <= a.terminal_menzil_m:
                self.durum = 'TERMINAL'
            if a.vurus_modu and self.en_iyi_menzil <= a.vurus_menzil_m:
                self.durum = 'VURUS'
        # VURUS KARISIMI: faz LATCH'li ama karisim ANLIK menzille
        # surer. Iskalayip acilirsak karisim kendiliginden 0'a doner
        # (maliyet nominale geri gelir), faz etiketi ise angajman
        # boyunca kalir -- log okunabilirligi icin.
        if self.durum == 'VURUS':
            genislik = max(a.vurus_menzil_m - a.vurus_tam_menzil_m, 1e-6)
            self.vurus_karisim = float(np.clip(
                (a.vurus_menzil_m - r) / genislik, 0.0, 1.0))
        else:
            self.vurus_karisim = 0.0

        if not a.iska_modu or self.durum == 'ISKA':
            return                       # ISKA LATCH'lidir: devre kadar

        # --- GECIS (pass) tespiti: iki bagimsiz tanik, "VEYA";
        #     uzerine ZORUNLU kapanma-hizi sarti (delip-gecme kaniti) ---
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

        # --- ISKA ilani ---
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
        elif not a.ilerleme_saat and gecen > a.iska_zaman_asimi_s:
            sebep = (f"zaman asimi ({gecen:.1f} s > {a.iska_zaman_asimi_s:.0f}), "
                     f"en iyi menzil {self.en_iyi_menzil:.1f} m")
        elif a.ilerleme_saat and self._durgunluk_saat > a.iska_zaman_asimi_s:
            # ILERLEME SAATI kolu: kapi artik "kac saniyedir yetkideyim"
            # degil "kac saniyedir ILERLEMIYORUM". Sebep dizesine ikisi
            # de yazilir ki logdan kolun hangisi oldugu ve kazanci ne
            # kadar kullandigi TEK SATIRDA okunabilsin.
            # ONEK "zaman asimi/" BILINCLI: ISKA sebepleri her yerde
            # (mpc_test, metrik betikleri, kosu ozetleri, senin A/B
            # tablolarin) bu TOKEN'la siniflaniyor. Yalniz "durgunluk"
            # yazmak ayni ailedeki iskalari sessizce "diger" kovasina
            # dusururdu -- kollar arasi kiyas KIRILIRDI. Onek korunur,
            # yeni bilgi parantezde eklenir.
            sebep = (f"zaman asimi/durgunluk ({self._durgunluk_saat:.1f} s > "
                     f"{a.iska_zaman_asimi_s:.0f}; angajman {gecen:.1f} s), "
                     f"en iyi menzil {self.en_iyi_menzil:.1f} m")
        elif a.ilerleme_saat and gecen > a.ilerleme_tavan_s:
            sebep = (f"zaman asimi/ilerleme tavani ({gecen:.1f} s > "
                     f"{a.ilerleme_tavan_s:.0f}), en iyi menzil "
                     f"{self.en_iyi_menzil:.1f} m")
        if sebep:
            self.durum = 'ISKA'
            self.iska_sebep = sebep
            print(f"[mpc] ISKA: {sebep} -> yetki birakiliyor")
            self._iska_yayinla()

    def _iska_yayinla(self):
        """ISTEGE BAGLI Redis kopru yayini (iska_redis_anahtar bossa yok).

        'komut_yetkisi'ne BILINCLI OLARAK yazilmaz: bbox_to_redis her
        karede o anahtari kendi moduyla eziyor (bbox_to_redis.py:472),
        yani kontrolcunun yazdigi deger ~33 ms icinde silinirdi.
        'manuel_durdur' de kullanilmaz: o bir OPERATOR kill-switch'idir,
        MANDALLIDIR (temizlenmezse karar verici bir daha ASLA
        'goruntulu'ya donmez) ve paylasimlidir."""
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
        except Exception as exc:                     # yayin ASLA ucusu bozmaz
            print(f"[mpc] ISKA redis yayini basarisiz: {exc}")

    def _iska_komut(self, olcum, ex, ey, eps, r, d_ex, d_ey, ey_ref,
                    alan, alan_hizi, dt):
        """ISKA'da komut: FRENLI SUZULME (coast) + 'birak' bayragi.

        NEDEN SIFIR DEGIL: sifir "komut vermemek" degil, tam gazdan
        TAM FREN komutudur (goruntulu_temel'de ayni ders 2026-08-04'te
        ogrenildi). NEDEN GUDUM DEGIL: olculen kusurun ta kendisi bu --
        hedefi gectikten sonra 9-12 m/s ile hala hedefe dogru komut
        uretmek. Yaw KOMUTLANMAZ (yaw_rate=None): kor donus, karar
        vericinin yeniden konumlandirmasini zorlastirir.

        FREN (2026-08-05, 35 m/s turu): suzulme artik SABIT HIZDA
        degil, YONU KORUYARAK iska_suzulme_hiz_mps'e RAMPA ile inen bir
        komuttur (bkz. MpcAyar.iska_suzulme_*). Sebep geometrik: en
        kucuk donus yaricapi v^2/a, yani 35 m/s'de 245 m, 12 m/s'de
        29 m. Yetkiyi hangi hizla biraktigimiz, konumlu gudumun
        yeniden konumlanma suresini dogrudan belirler.

        Cozucu KOSMAZ -- ISKA dongusu normalden UCUZDUR."""
        a = self.a
        v = (np.asarray(olcum.vel_ned, dtype=float).copy()
             if olcum.vel_ned is not None
             else (self._son_v_ned.copy() if self._son_v_ned is not None
                   else np.zeros(3)))
        # HIZ BUYUKLUGUNU rampa ile indir, YONU koru.
        hiz = float(np.linalg.norm(v))
        if hiz > 1e-6:
            hedef_hiz = max(a.iska_suzulme_hiz_mps,
                            hiz - a.iska_suzulme_ivme_mps2 * dt)
            v = v * (min(hedef_hiz, hiz) / hiz)
        self._son_v_ned = v.copy()
        self.yaw_uygulanan = 0.0
        self._tani_yaz(olcum, ex, ey, eps, r, d_ex, d_ey,
                       np.zeros(MpcCozucu.NU), v, ey_ref, alan, alan_hizi,
                       {'sure_ms': 0.0, 'iterasyon': 0, 'maliyet': 0.0})
        self.sayac += 1
        k = Komut(vel_ned=v, yaw_rate_dps=None)
        # ORTAK DOSYA SOZLESMESI ONERISI: goruntulu_temel.Komut'a
        # 'birak: bool = False' ve 'birak_sebep: str = ""' alanlari
        # eklenmeli; GoruntuluDongu birak=True gorunce aktif=False
        # yapip karar vericiye haber vermeli. Alan EKLENENE KADAR
        # oznitelik olarak konur (Komut bir dataclass, __slots__ yok):
        # iskelet gormezden gelir, davranis bozulmaz, testler okur.
        k.birak = True
        k.birak_sebep = self.iska_sebep
        return k

    def _kor_komut(self, olcum, ex, ey, eps, r, d_ex, d_ey, ey_ref,
                   alan, alan_hizi):
        """VURUS fazinda bbox BAYATSA: son komutu AYNEN tekrarla (coast).

        Uzak menzilde bayat bbox'a dogru cevap SERT KISITI BIRAKMAKTIR
        (bayat_kisit_s kapisi): ey donmusken kisit kendi urettigi dikey
        hizi olcup daha da sertlesir. VURUS'ta ise dogru cevap PLANI
        TAMAMLAMAKTIR -- t_go < 1.6 s, hedef son gorulen LOS uzerinde,
        kamera 0 deg SABIT oldugu icin zaten kor kaldik ve o korlugu
        yeni bir manevrayla derinlestirmenin faydasi yok. Cozucu
        KOSMAZ; komut, iskeletin LPF'sinden gecerek yumusakca devam
        eder. Bu suzulmeyi ISKA durum makinesi sonlandirir (menzil
        taniki bbox'tan BAGIMSIZ, estimator'dan gelir).

        Yaw KOMUTLANIR (son deger): burun donusu durursa hedefin son
        kerterizini kaybederiz; kadraj disi olsa bile burun dogru yone
        bakmaya devam etmeli."""
        v = (self._son_v_ned.copy() if self._son_v_ned is not None
             else (np.asarray(olcum.vel_ned, dtype=float).copy()
                   if olcum.vel_ned is not None else np.zeros(3)))
        u0 = (self.u_onceki.copy() if self.u_onceki is not None
              else np.zeros(MpcCozucu.NU))
        self.kor_dongu += 1
        self._tani_yaz(olcum, ex, ey, eps, r, d_ex, d_ey, u0, v, ey_ref,
                       alan, alan_hizi,
                       {'sure_ms': 0.0, 'iterasyon': 0, 'maliyet': 0.0,
                        'vurus': self.vurus_karisim})
        self.sayac += 1
        k = Komut(vel_ned=v,
                  yaw_rate_dps=(float(self.yaw_uygulanan)
                                if self.a.yaw_komutu_ver else None))
        k.birak = False
        k.birak_sebep = ''
        return k

    def _warm_start(self, v_heading, l, e2, e3):
        """Onceki cozumu bir blok kaydir; yoksa devir hizindan uret.

        v_heading: konumlunun devirdeki son hiz komutu, HEADING
        cercevesine (Rz(-yaw)) cevrilmis halde. Boylece ilk cozum
        sifirdan degil aracin o anki hareketinden baslar -- iskeletin
        LPF tohumlamasiyla ayni mantik, sicrama olmaz."""
        nb, NU = self.cozucu.nb, MpcCozucu.NU
        if self.U is not None:
            U = np.vstack([self.U[1:], self.U[-1:]])
            return U.reshape(-1)
        if v_heading is None:
            return np.zeros(nb * NU)
        u0 = np.array([float(v_heading @ l), float(v_heading @ e2),
                       float(v_heading @ e3), 0.0])
        return np.tile(u0, nb)

    def _yaw_hizi_olc(self, yaw_rad, dt):
        """Yaw hizini KENDI tutumumuzdan olc (komut edileni degil).

        Bozucu kestirimi ex_dot'tan yaw etkisini cikaracak; komut
        edilen yaw hizi otopilotta gecikmeyle olustugu icin komutu
        kullanmak bozucuya sahte bir bilesen sizdirir.

        DEVIR SOGUKLUGU (2026-08-04, olculdu): LPF sifirdan basliyordu
        ve BOZUCUNUN ILK ARTIGI tam da bu LPF'nin ilk orneginde
        hesaplaniyor. Devirde arac genellikle DONUYOR (konumlu gudum
        hedefe yaw kumandaliyor); sifirdan baslayan LPF ilk ornekte
        gercek yaw hizinin ancak k=dt/(dt+0.15)~0.25'ini goruyor, yani
        yaw hizinin %75'i bozucudan CIKARILAMIYOR ve dogrudan d_ex'e
        SAHTE bir bilesen olarak giriyor. Ustelik bozucu ilk ornekte
        1/n=1 kazanciyla o degeri OLDUGU GIBI benimsiyor. Cozum:
        YALNIZ ILK ORNEKTE k=1 (kosan ortalamanin ilk adimi), yani
        yaw hizi bozucunun ilk artigiyla AYNI PENCEREDE olculdugu
        gibi girer. Sonraki ornekler normal LPF'de kalir -- KASITLI:
        max(1/n, ...) ile 2. ve 3. ornegi de hizlandirmak kestirimi
        gurultuye acar ve 25 s'lik kapali dongu olcutlerini (test 5i)
        kaotik olarak oynatir. Kazanc yalnizca artigin KIRLENDIGI tek
        ornekte degisir."""
        if yaw_rad is None:
            return self.yaw_uygulanan
        if self._yaw_onceki is None or not (0.01 < dt < 0.35):
            self._yaw_onceki = yaw_rad
            return self.yaw_hiz_lpf
        fark = math.degrees(
            (yaw_rad - self._yaw_onceki + math.pi) % (2 * math.pi) - math.pi)
        self._yaw_onceki = yaw_rad
        ham = fark / dt
        self._yaw_n += 1
        k = dt / (dt + 0.15)          # kisa LPF: yaw hizi zaten hizli
        if self.a.devir_yaw_tohum and self._yaw_n == 1:
            k = 1.0
        self.yaw_hiz_lpf += k * (float(np.clip(ham, -200.0, 200.0))
                                 - self.yaw_hiz_lpf)
        return self.yaw_hiz_lpf

    # ---------------------------------------------------------- komut

    def komut(self, olcum: Olcum) -> Komut:
        a = self.a
        dt = float(np.clip(olcum.dt, 0.02, 0.30))
        ex = float(olcum.ex_deg)
        ey = float(olcum.ey_deg)
        # --- KOR PN: bbox BAYATSA kadraji OLU HESAPLA ILERLET -----------
        # (bkz. cevre_kor_pn; R1-R5 raylari orada gerekcelendirildi)
        # KAPALI kolda bu blok HIC calismaz -> davranis BIT-AYNI.
        self.kor_pn_kosdu = 0
        bayat = olcum.bbox_yas_s > a.bayat_kisit_s
        if a.kor_pn and bayat and olcum.menzil_m is not None:   # R4
            self._kor_t += dt
            # MENZIL KAPISI: kazanc TERMINALDE olusuyor, kayip uzak
            # menzildeki kor karelerden geliyor (bkz. kor_pn_menzil_m).
            r_kapi = (self.r_ic if self.r_ic is not None
                      else float(olcum.menzil_m))
            if (self._kor_t <= a.kor_pn_azami_s                 # R1
                    and r_kapi <= a.kor_pn_menzil_m             # MENZIL KAPISI
                    and self._kor_ex is not None):
                sonum = math.exp(-self._kor_t / max(a.kor_pn_tau_s, 1e-3))
                r_i = max(self.r_ic if self.r_ic is not None else
                          float(olcum.menzil_m), a.menzil_taban_m)
                eps_i = -(self._kor_ey + a.aim_deg)
                c2_i = KDEG / (r_i * max(math.cos(math.radians(eps_i)), 0.2))
                c3_i = KDEG / r_i
                w_i = (self.u_onceki[:3] if self.u_onceki is not None
                       else np.zeros(3))
                yaw_i = float(self.yaw_uygulanan)
                self._kor_ex += dt * (-c2_i * w_i[1] - yaw_i
                                      + self.bozucu.d_ex * sonum)
                self._kor_ey += dt * (-c3_i * w_i[2]
                                      + self.bozucu.d_ey * sonum)
                # R3 + R3': kelepce hem son GERCEK olcume gore
                # (pay) hem de MUTLAK kadraj yari-acisina gore.
                tav = a.kor_pn_ex_mutlak_deg
                if self._kor_ex0 is not None:
                    tav = min(self._kor_ex0 + a.kor_pn_ex_pay_deg, tav)
                self._kor_ex = float(np.clip(self._kor_ex, -tav, tav))
                ex, ey = self._kor_ex, self._kor_ey
                self.kor_pn_kosdu = 1
        elif not bayat:
            # TAZE olcum: olu hesabi olcumle YENIDEN TOHUMLA.
            self._kor_t = 0.0
            self._kor_ex, self._kor_ey = ex, ey
            self._kor_ex0 = abs(ex)
        eps = -(ey + a.aim_deg)              # hedefin ufka gore yukselisi

        yaw = olcum.yaw_rad if olcum.yaw_rad is not None else 0.0
        l, e2, e3 = los_ucayak(ex, eps)
        c, s = math.cos(yaw), math.sin(yaw)

        def _ned_to_heading(v):
            v = np.asarray(v, dtype=float)
            return np.array([c * v[0] + s * v[1],
                             -s * v[0] + c * v[1], v[2]])

        # --- kendi hizimiz LOS ucayaginda (OLCULEN, komut edilen degil:
        #     eyleyici gecikmesi boylece kestirime sizmaz) ---
        if olcum.vel_ned is not None:
            v_h = _ned_to_heading(olcum.vel_ned)
            w = np.array([float(v_h @ l), float(v_h @ e2), float(v_h @ e3)])
        elif self.u_onceki is not None:
            w = self.u_onceki[:3].copy()
        else:
            w = np.zeros(3)

        r = self._menzil(olcum, w[0], dt)
        r_g = max(r, a.menzil_taban_m)
        c2 = KDEG / (r_g * max(math.cos(math.radians(eps)), 0.2))
        c3 = KDEG / r_g
        yaw_hizi = self._yaw_hizi_olc(olcum.yaw_rad, dt)

        d_ex, d_ey, d_r = self.bozucu.guncelle(
            ex, ey, olcum.menzil_m, dt, c2, c3, w[0], w[1], w[2], yaw_hizi)
        if a.menzil_bozucu_kaynak != "menzil":
            d_r = 0.0

        vz_simdi = float(l[2] * w[0] + e2[2] * w[1] + e3[2] * w[2])
        ey_ref, beta_c = self._kadraj_sabiti(olcum, vz_simdi)
        alan, alan_hizi = self._alan_guncelle(olcum, dt)

        # --- ISKA DURUM MAKINESI (cozucuden ONCE) ---
        # Buraya kadarki her sey OLCUM guncellemesidir (birkac
        # mikrosaniye); pahali olan tek adim cozucudur ve ISKA'da
        # atlanir. Yeri bilincli: durum makinesi cozucunun cevabina
        # BAKMAZ, cunku "gectim" sorusunun cevabi optimizasyonda
        # degil geometridedir.
        self._durum_makinesi(olcum, r, alan_hizi, dt)
        # FIZIKSEL TEMAS: durum makinesinden SONRA (detay satirinda durum
        # ve vurus karisimi guncel olsun), komut yolundan ONCE -- olay
        # HANGI koldan donersek donelim Komut'a takilir (bkz. _olay_tak).
        self._bekleyen_olay = self._vurus_basarili_kontrol(olcum, r)
        if self.durum == 'ISKA':
            return self._olay_tak(self._iska_komut(
                olcum, ex, ey, eps, r, d_ex, d_ey, ey_ref, alan,
                alan_hizi, dt))
        # VURUS + BAYAT BBOX -> KOR SUZULME (bkz. _kor_komut)
        # KOR PN ACIKKEN bu kol ATLANIR: asil kazanim tam burasi --
        # terminalde (VURUS, r <= 8 m) komut tekrari yerine ILERLETILMIS
        # kadrajla cozucuyu kosturmak. R1 suresi dolunca kor_pn_kosdu 0
        # olur ve kol yine devreye girer (guvenli geri dusus).
        if (self.durum == 'VURUS' and a.vurus_kor_suzulme
                and not (a.kor_pn and self.kor_pn_kosdu)
                and r <= a.vurus_kor_menzil_m
                and olcum.bbox_yas_s > a.bayat_kisit_s
                and self._son_v_ned is not None):
            return self._olay_tak(self._kor_komut(
                olcum, ex, ey, eps, r, d_ex, d_ey, ey_ref, alan, alan_hizi))

        x0 = np.array([ex, ey, r, w[0], w[1], w[2]])
        v_tohum = (None if self.v_ned_tohum is None
                   else _ned_to_heading(self.v_ned_tohum))
        U_warm = self._warm_start(v_tohum, l, e2, e3)
        if self.u_onceki is None:
            # ILK DONGU: fark cezasinin demiri. Sifir demek "arac
            # duruyordu" demektir ve ilk komutu frene ceker; dogrusu
            # konumlunun son komutudur (U_warm'in ilk blogu tam olarak
            # o). Yaw kanali 0 kalir: konumlu yaw HIZI komutlamiyor.
            self.u_onceki = np.zeros(MpcCozucu.NU)
            if a.devir_prox_tohum:
                self.u_onceki[:3] = np.asarray(U_warm[:3], dtype=float)
        U, bilgi = self.cozucu.coz(
            x0, d_ex, d_ey, d_r, eps, ey_ref, beta_c,
            dt0=dt, U_warm=U_warm, u_onceki=self.u_onceki,
            guven=self.bozucu.guven, bbox_yas_s=olcum.bbox_yas_s,
            irtifa_m=(None if olcum.pos_ned is None
                      else -float(olcum.pos_ned[2])),
            d_ex_kutu=self.bozucu.d_ex_kutu,
            d_ey_kutu=self.bozucu.d_ey_kutu,
            yaw_delta_agirlik=self._yaw_delta_agirlik(d_ex, r, dt),
            vurus=self.vurus_karisim,
            t_go_s=self._t_go(),
            # APN (env dugmeli, varsayilan KAPALI -> tam 0). Olu bant,
            # guven carpani ve kelepce KESTIRICIDE uygulanmis olur.
            apn_a=self.bozucu.apn_a_etkin())
        self.U = U
        u0 = U[0]
        self.u_onceki = u0.copy()

        # --- LOS ucayagi -> heading cercevesi -> NED ---
        v_h = u0[0] * l + u0[1] * e2 + u0[2] * e3
        v_ned_cmd = govde_ileri_ned(yaw, v_h[0], v_h[1], v_h[2])
        yaw_rate = float(u0[3]) if a.yaw_komutu_ver else None
        self.yaw_uygulanan = float(u0[3]) if a.yaw_komutu_ver else 0.0

        self._tani_yaz(olcum, ex, ey, eps, r, d_ex, d_ey, u0, v_ned_cmd,
                       ey_ref, alan, alan_hizi, bilgi)
        self.sayac += 1
        self._son_v_ned = v_ned_cmd.copy()
        k = Komut(vel_ned=v_ned_cmd, yaw_rate_dps=yaw_rate)
        k.birak = False                  # bkz. _iska_komut sozlesme notu
        k.birak_sebep = ''
        return self._olay_tak(k)

    def _olay_tak(self, k):
        """Bekleyen ayrik olayi Komut'a takar (iskelet _olay.csv'ye yazar).

        TEK NOKTA: olay uretimi komut yolundan BAGIMSIZ (temas her uc
        kolda da -- normal / ISKA / kor suzulme -- olabilir), bu yuzden
        uretim komut() basinda bir kez yapilir, tasima burada."""
        olay = getattr(self, '_bekleyen_olay', None)
        if olay:
            k.olay, k.olay_detay = olay
            self._bekleyen_olay = None
        self._komut_bas(k)
        return k

    def _komut_bas(self, k):
        """Talep edilen komutu terminale basar (varsayilan 1 Hz).

        _tani CSV'si zaten HER adimi yazar; bu satir canli izleme
        icindir. Yeri bilincli olarak _olay_tak: uc komut kolu da
        (normal / ISKA / kor suzulme) buradan gecer, yani basilan deger
        araca giden komutun ta kendisidir. Throttle'in nedeni dongu
        hizi: 2 Hz'e dusen dongu gecmisi var (bkz. LOG_SOZLUGU), her
        tick'te stdout'a yazmak o yaraya tuz basmasin.
        $YILDIZ_KOMUT_BASKI_S periyodu ayarlar; 0 ya da negatif kapatir."""
        periyot = getattr(self, '_komut_baski_periyot', None)
        if periyot is None:
            try:
                periyot = float(os.environ.get('YILDIZ_KOMUT_BASKI_S',
                                               '1.0'))
            except ValueError:
                periyot = 1.0
            self._komut_baski_periyot = periyot
        if periyot <= 0:
            return
        simdi = time.monotonic()
        if simdi - getattr(self, '_komut_baski_t', 0.0) < periyot:
            return
        self._komut_baski_t = simdi
        v = np.asarray(k.vel_ned, dtype=float)
        yr = '  --' if k.yaw_rate_dps is None else f"{k.yaw_rate_dps:+6.1f}"
        ek = f" BIRAK({k.birak_sebep})" if k.birak else ''
        print(f"[mpc] komut vN={v[0]:+6.2f} vE={v[1]:+6.2f} "
              f"vD={v[2]:+6.2f} m/s |v|={float(np.linalg.norm(v)):5.2f} "
              f"yaw_hizi={yr} dps durum={self.durum}{ek}", flush=True)

    # ------------------------------------------------------ tani log

    def _tani_yaz(self, olcum, ex, ey, eps, r, d_ex, d_ey, u0, v_ned,
                  ey_ref, alan, alan_hizi, bilgi):
        """ODUL KOLONLARI LINEER ALANA HIZALI (2026-08-04 revizyonu):
        'alan' = w*h [px^2] (karekok DEGIL), 'alan_hizi' = dA/dt
        [px^2/s]. 'beta' = hedefin KAMERA EKSENINE gore dikey sapmasi
        (+ = eksenin altinda) ve 'beta_sinir' o anki sert alt sinir --
        kadraj payinin canli olcusu, kayip analizinin ana kolonu.

        MONTAJ 0 OKUMA NOTU: beta artik tipik olarak NEGATIFTIR
        (hedef eksenin USTUNDE: standoff'ta ~ -16 deg, carpmada ~ 0).
        Kadraj payi bu yuzden 'beta_sinir'a degil, -fov_ust_bant'a
        (-17.5) ve fiziksel kenara (-20.07) gore okunmalidir; alt
        sinir yalniz terminal frende baglayici olur.
        'yaw_ceza': yaw fark cezasinin O ANKI programlanmis degeri
        (10 = sakin/sonumlu, 1 = cevik). Hedef manevrasinin dik hiz
        bileseninden turetilir; sim'de "yetki acildi mi" sorusu bu
        kolondan okunur.

        ISKA KOLONLARI (sonda, 2026-08-05):
          'durum'         : KAPANMA | TERMINAL | ISKA
          'en_iyi_menzil' : bu angajmanda ulasilan en kucuk r [m]
          'menzil_hizi'   : d(r_ic)/dt suzulmus [m/s]; NEGATIF =
                            kapaniyoruz, POZITIF = aciliyor. Terminal
                            gecis bu kolonun isaret degistirdigi andir.
        Iska teshisi tek satirda: durum ISKA olan ilk satiri bul,
        'menzil_hizi' ve 'en_iyi_menzil' o anki degerleriyle sebebi
        (goruntulu.log'daki '[mpc] ISKA:' satiri) dogrular."""
        if self.tani_log_yolu is None:
            return
        if self._tani is None:
            os.makedirs(os.path.dirname(self.tani_log_yolu), exist_ok=True)
            self._tani_f = open(self.tani_log_yolu, 'w', newline='')
            self._tani = csv.writer(self._tani_f)
            self._tani.writerow(
                ['t', 'dt', 'bbox_yas', 'ex', 'ey', 'eps', 'ey_ref',
                 'beta', 'beta_sinir', 'derinlik', 'fov_serbest',
                 'bos_sayac', 'r_ic', 'r_olcum', 'alan', 'alan_hizi',
                 'd_ex', 'd_ey', 'u1', 'u2', 'u3', 'yaw_dps', 'yaw_ceza',
                 'los_hiz_az', 'los_hiz_el', 'vx', 'vy', 'vz', 'pitch_lpf',
                 'vz_alt_cbf', 'vz_ust_cbf', 'yaw_alt_cbf', 'yaw_ust_cbf',
                 'sure_ms', 'iter', 'maliyet',
                 # ISKA DURUM MAKINESI -- kolonlar bilincli olarak SONA
                 # eklendi: mevcut kolonlarin INDISI kaymasin (test
                 # pilotunun awk/cut tek satirlari kirilmasin).
                 'durum', 'en_iyi_menzil', 'menzil_hizi',
                 # mutlak saat koprusu (video/bbox hizalamasi); en sonda
                 # ayni indis-koruma gerekcesiyle.
                 't_unix',
                 # VURUS FAZI (2026-08-05) -- yine SONA, indis-koruma.
                 #  'vurus'    : karisim katsayisi [0..1]. 0 = nominal
                 #               maliyet, 1 = saf yakalama. 'durum'
                 #               VURUS iken 22 m'de 0, 8 m'de 1.
                 #  'bant_ust' : o anki UST kadraj bandi [deg]. VURUS'ta
                 #               17.5 -> 19.0'a acilir; ileri ivmelenme
                 #               varken daralir (yeni hizlanma terimi).
                 'vurus', 'bant_ust',
                 #  'ivme_carp': q_ivme'ye o dongude uygulanan carpan
                 #               (VURUS ivme carpani; varsayilan 1.0).
                 #  'hiza_ref' : VURUS terminal dikey hizalama biasi
                 #               [dps]. eps > 10 deg iken pozitif =
                 #               "hatta tirman" (standoff eritme). 0 =
                 #               deadzone icinde ya da VURUS disi.
                 'ivme_carp', 'hiza_ref',
                 #  'vuruldu'  : FIZIKSEL TEMAS latch'i (0/1).
                 #               vibe > 15 VE menzil < 3 m gorulunce
                 #               1'e gecer ve angajman boyunca kalir.
                 #               Kosu ozetinde vurus SAYISI bu
                 #               kolonun 0->1 gecis sayisidir.
                 #  'vibe'     : KENDI vibrasyonumuz (en buyuk eksen).
                 'vuruldu', 'vibe',
                 # 'butce_kesti' (2026-08-07): 1 = FISTA yakinsamadan
                 # cikti, yani iterasyon tavanina (iterasyon_tavani) ya
                 # da sure butcesine (sure_butcesi_ms) carpti. 'iter' ve
                 # 'sure_ms' bunu TEK BASINA soylemez: tavana degen bir
                 # cozum de yakinsayan bir cozum de ayni sayilari
                 # basabilir. Raspberry Pi 5'te CPU sikisirsa cozucu
                 # SESSIZCE optimal olmayan komut vermeye baslar; bu
                 # kolon o bozulmanin tek gorunur imzasidir. 0 = durma
                 # olcutu saglandi (saglikli). Yine SONA eklendi
                 # (indis-koruma).
                 'butce_kesti',
                 # 'dikey_hata' (2026-08-07): DOGRUDAN dikey hata (P)
                 # teriminin o karede maliyete koydugu aci artigi (deg,
                 # isaret eps ile ayni). 0 = terim kapali, olu bant ici,
                 # menzil rampasi disi ya da bbox bayat. 'hiza_ref'
                 # (TUREV kolu) ile YAN YANA okunur: P mi D mi calisiyor
                 # sorusunun tek gorunur cevabi. SONA eklendi
                 # (indis-koruma).
                 'dikey_hata',
                 # 'tgo' / 'dikey_tau' (2026-08-07, tur-3): t_go sekilli
                 # dikey hiz referansinin tanisi (bkz. cevre_dikey_tgo).
                 # 'tgo'       : yasada KULLANILAN kalan zaman [s]
                 #               (tavan/gecersizlik uygulanmis hali).
                 # 'dikey_tau' : tau_eff = clip(tgo/k, tau_min, tau_max).
                 # Ikisi de BOS = kol kapali. dikey_tau tau_min'e
                 # (0.30) YAPISMISSA yasa doymus demektir -- asma
                 # teshisinin ilk bakilacak kolonu. SONA eklendi
                 # (indis-koruma).
                 'tgo', 'dikey_tau',
                 # APN TANISI (2026-08-08) -- yine SONA, indis-koruma.
                 # Bkz. cevre_apn.
                 #  'v_dik' : kestirilen hedef DIK hizi [m/s]
                 #            (= d_ex * r / KDEG; kol KAPALIYKEN de
                 #            yazilir -- APN'in ne kazandiracagini
                 #            olcmek icin taban kosusunda da gerekli).
                 #  'a_dik' : v_dik'in suzulmus turevi [m/s^2], HAM
                 #            kestirim (olu bant/guven UYGULANMAMIS).
                 #  'apn_a' : yasada GERCEKTEN kullanilan deger; olu
                 #            bant + guven + carpan uygulanmis hali.
                 #            Kol kapali ya da olu bant icinde ise 0.
                 # Donus/duz faz ayrimi 'a_dik' p50/p90 ile bu uc
                 # kolondan okunur: a_dik buyuk ama apn_a 0 ise kapiyi
                 # gurultu degil OLU BANT kesmistir.
                 'v_dik', 'a_dik', 'apn_a',
                 # DOYUMLU EYLEYICI TANISI (2026-08-08) -- SONA,
                 # indis-koruma. Bkz. cevre_eyleyici.
                 #  'tau_eff' : ILK adimin YATAY etkin zaman sabiti [s]
                 #              = max(tau_lin, |e_yatay|/a_max). BOS =
                 #              kol KAPALI (eski sabit 1.00 s yol).
                 #              tau_lin'e (1.7) YAPISMISSA dogrusal
                 #              bolgedeyiz; buyudukce plan DOYUMDA
                 #              demektir -- ulasilamaz komut uretmek
                 #              yerine modelin manevranin zaman
                 #              aldigini bildigi durum.
                 #  'tau_eff_z': AYNISININ DIKEY karsiligi
                 #              = max(tau_lin_z, |e_v|/a_max_z).
                 #              Ilk surumde dikeyde sinir YOKTU ve talep
                 #              bu kanala kaciyordu (|u3| p90 2.2 kat,
                 #              ALTITUDE ABORT 1.9 kat) -- bkz.
                 #              MpcAyar.eyleyici_a_max_z_mps2.
                 #  'u_doyum' : |u_yatay| hiz tavaninin %99'una degdi
                 #              mi (0/1). Beklenen etki: kol ACIKKEN bu
                 #              oran DUSMELI (ulasilamaz u1 sicramalari
                 #              azalir).
                 'tau_eff', 'tau_eff_z', 'u_doyum', 'kor_pn',
                 # ILERLEME SAATI TANISI (2026-08-08) -- SONA,
                 # indis-koruma. Bkz. cevre_ilerleme_saat.
                 #  'kor_pn'    : bu dongude kadraj OLU HESAPLA
                 #               ILERLETILDI mi (0/1). Kol kapaliyken
                 #               hep 0. "Kor karelerde komut guncelleme
                 #               orani" = bayat kareler icinde bu
                 #               kolonun 1 oldugu oran; kol calisiyorsa
                 #               o karelerde cozucu de kosmustur
                 #               (sure_ms > 0 ile capraz dogrulanir).
                 #  'durgunluk' : ISKA'yi atesleyen DURGUNLUK saati [s].
                 #               BOS = kol KAPALI (o zaman karar duvar
                 #               saatinden verilir ve saat hic
                 #               hesaplanmaz -- kapali kolda tek
                 #               mikrosaniye harcanmaz, bkz.
                 #               _durum_makinesi'ndeki not).
                 #               iska_zaman_asimi_s'i (8) asinca ISKA
                 #               gelir. Ilerledikce GERI sarar.
                 'durgunluk'])
        r_g = max(r, self.a.menzil_taban_m)
        los_az = -KDEG / r_g * u0[1] + d_ex
        los_el = -KDEG / r_g * u0[2] + d_ey
        s = bilgi.get('cbf', (float('nan'),) * 4)
        self._tani.writerow([
            f"{olcum.t:.4f}", f"{olcum.dt:.4f}", f"{olcum.bbox_yas_s:.3f}",
            f"{ex:.3f}", f"{ey:.3f}", f"{eps:.3f}", f"{ey_ref:.3f}",
            f"{bilgi.get('beta', float('nan')):.3f}",
            # YURURLUKTEKI bant (fren daraltmasi UYGULANMIS hali).
            # Onceden sabit fov_alt_bant_deg yaziliyordu; test pilotu
            # cakilma teshisi icin bu kolona muhtacti (LOG KORLUGU).
            f"{bilgi.get('bant_alt', float('nan')):.2f}",
            f"{r * math.sin(math.radians(eps)) if eps > 0 else 0.0:.1f}",
            f"{bilgi.get('fov_serbest', 0)}", f"{bilgi.get('bos_sayac', 0)}",
            f"{r:.2f}",
            '' if olcum.menzil_m is None else f"{olcum.menzil_m:.2f}",
            '' if alan is None else f"{alan:.1f}",
            f"{alan_hizi:.2f}",
            f"{d_ex:.3f}", f"{d_ey:.3f}",
            f"{u0[0]:.3f}", f"{u0[1]:.3f}", f"{u0[2]:.3f}", f"{u0[3]:.2f}",
            f"{bilgi.get('yaw_delta', float('nan')):.2f}",
            f"{los_az:.3f}", f"{los_el:.3f}",
            f"{v_ned[0]:.3f}", f"{v_ned[1]:.3f}", f"{v_ned[2]:.3f}",
            f"{self.pitch_lpf:.2f}",
            f"{s[0]:.2f}", f"{s[1]:.2f}", f"{s[2]:.1f}", f"{s[3]:.1f}",
            f"{bilgi['sure_ms']:.3f}", bilgi['iterasyon'],
            f"{bilgi['maliyet']:.4f}",
            self.durum,
            ('' if not math.isfinite(self.en_iyi_menzil)
             else f"{self.en_iyi_menzil:.2f}"),
            f"{self.menzil_hizi:.2f}",
            f"{time.time():.3f}",
            f"{self.vurus_karisim:.3f}",
            f"{bilgi.get('bant_ust', float('nan')):.2f}",
            f"{bilgi.get('ivme_carpani', float('nan')):.3f}",
            f"{bilgi.get('hiza_ref', float('nan')):.3f}",
            1 if self.vuruldu else 0,
            '' if olcum.vibe_max is None else f"{olcum.vibe_max:.1f}",
            int(bilgi.get('butce_kesti', 0)),
            f"{bilgi.get('dikey_hata', float('nan')):.3f}",
            # NaN = kol kapali; bos string yaz ki CSV'de "0" ile
            # karismasin (0 gercek bir tau/t_go degeri olamaz).
            _bos_nan(bilgi.get('tgo', float('nan'))),
            _bos_nan(bilgi.get('dikey_tau', float('nan'))),
            f"{self.bozucu.v_dik:.3f}", f"{self.bozucu.a_dik:.3f}",
            f"{bilgi.get('apn_a', 0.0):.3f}",
            _bos_nan(bilgi.get('tau_eff', float('nan')), '.3f'),
            _bos_nan(bilgi.get('tau_eff_z', float('nan')), '.3f'),
            (1 if (math.hypot(u0[0], u0[1])
                   >= 0.99 * bilgi.get('v_tavan', self.a.hiz_tavani_mps))
             else 0),
            int(self.kor_pn_kosdu),
            (f"{self._durgunluk_saat:.2f}" if self.a.ilerleme_saat
             else "")])
        if self.sayac % 20 == 0:
            self._tani_f.flush()


# =============================================================== main

def main():
    p = argparse.ArgumentParser(description="MPC goruntulu gudum")
    p.add_argument('--sure', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--ufuk', type=int, default=None,
                   help='ongoru adim sayisi (varsayilan 24)')
    p.add_argument('--adim-s', type=float, default=None)
    p.add_argument('--sqp', type=int, default=None, help='SQP gecis sayisi')
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw KOMUTLAMA (ablasyon: yaw otopilotta kalir, '
                        '"ayri FOV kontrolcusu gerekli mi" deneyi)')
    p.add_argument('--aim', type=float, default=None,
                   help='sanal gimbal aim ofseti (senaryo.sh AIM=0 verir)')
    p.add_argument('--mount', type=float, default=None,
                   help='kamera montaj acisi [deg, YUKARI +]; verilmezse '
                        '$YILDIZ_MOUNT (standoff_geom.sh), o da yoksa 0')
    p.add_argument('--pitch-baglasimi', dest='pitch_baglasimi',
                   action='store_true', default=None,
                   help='kamera ekseni GOVDEYE SABIT (varsayilan): tirmanma '
                        've fren pitch terimleri modelde')
    p.add_argument('--no-pitch-baglasimi', dest='pitch_baglasimi',
                   action='store_false',
                   help='kamera ekseni GIMBALLE stabilize: pitch terimleri '
                        'dusurulur (gercek pitch-servo gimbal donanimi)')
    p.add_argument('--q-ivme', type=float, default=None,
                   help='ivme/yatma cezasi (agresiflik <-> kadraj korunumu)')
    p.add_argument('--hiz-tavani', type=float, default=None,
                   help='hiz kelepcesi [m/s]; verilmezse '
                        'guidance_config.GORUNTULU_MAX_SPEED_MPS')
    p.add_argument('--no-vurus', action='store_true',
                   help='VURUS fazini KAPAT (ablasyon: yakin menzilde '
                        'maliyet nominal takip maliyeti olarak kalir)')
    p.add_argument('--no-iska', action='store_true',
                   help='ISKA durum makinesini KAPAT (ablasyon: eski '
                        'davranis -- gecis sonrasi komut vermeye devam)')
    p.add_argument('--iska-redis-anahtar', default=None,
                   help='ISKA ilaninda yazilacak Redis anahtari (kopru; '
                        'varsayilan KAPALI, asil yol Komut.birak bayragi)')
    p.add_argument('--tani-log', default=None)
    args = p.parse_args()

    ayar = MpcAyar()
    if args.ufuk is not None:
        ayar = MpcAyar(n_adim=args.ufuk,
                       bloklar=_blok_uret(args.ufuk))
    if args.adim_s is not None:
        ayar.adim_s = args.adim_s
    if args.sqp is not None:
        ayar.sqp_gecis = args.sqp
    if args.no_yaw:
        ayar.yaw_komutu_ver = False
    if args.aim is not None:
        ayar.aim_deg = args.aim
    if args.mount is not None:
        ayar.mount_pitch_deg = args.mount
    if args.pitch_baglasimi is not None:
        ayar.pitch_baglasimi = args.pitch_baglasimi
    if args.q_ivme is not None:
        ayar.q_ivme = args.q_ivme
    if args.hiz_tavani is not None:
        ayar.hiz_tavani_mps = args.hiz_tavani
    if args.no_vurus:
        ayar.vurus_modu = False
    if args.no_iska:
        ayar.iska_modu = False
    if args.iska_redis_anahtar is not None:
        ayar.iska_redis_anahtar = args.iska_redis_anahtar
    print(f"[mpc] iska modu="
          f"{'ACIK' if ayar.iska_modu else 'KAPALI'} "
          f"(terminal {ayar.terminal_menzil_m:.0f} m, arm "
          f"{ayar.iska_arm_m:.0f}/{ayar.iska_gecis_arm_m:.0f}, acilma "
          f"{ayar.iska_acilma_m:.0f}/{ayar.iska_gecis_acilma_m:.0f}, "
          f"zaman asimi {ayar.iska_zaman_asimi_s:.0f} s)")
    print(f"[mpc] hiz tavani={ayar.hiz_tavani_mps:.1f} m/s "
          f"(guidance_config.GORUNTULU_MAX_SPEED_MPS ile ayni kaynak), "
          f"vurus fazi={'ACIK' if ayar.vurus_modu else 'KAPALI'} "
          f"({ayar.vurus_menzil_m:.0f} m -> {ayar.vurus_tam_menzil_m:.0f} m), "
          f"ufuk={ayar.n_adim * ayar.adim_s:.2f} s")
    print(f"[mpc] montaj={ayar.mount_pitch_deg:+.2f} deg "
          f"(YILDIZ_MOUNT={os.environ.get('YILDIZ_MOUNT', '<yok>')}), "
          f"aim={ayar.aim_deg:+.2f}, pitch_baglasimi="
          f"{'ACIK' if ayar.pitch_baglasimi else 'KAPALI (gimbal)'}, "
          f"dikey bant -{ayar.fov_ust_bant_deg:.1f}..+"
          f"{ayar.fov_alt_bant_deg:.1f} deg")
    # A/B dugmesi kosunun BASINDA loga yazilsin: hangi kolun kostugu
    # sonradan CSV'den degil, ilk satirdan okunabilsin.
    print(f"[mpc] terminal dikey hizalama="
          f"{'ACIK' if ayar.dikey_terminal else 'KAPALI'} "
          f"(YILDIZ_DIKEY_TERMINAL, {ayar.dikey_terminal_menzil_m:.0f} m -> "
          f"{ayar.dikey_terminal_tam_m:.0f} m, olu bant "
          f"+-{ayar.dikey_terminal_rahat_deg:.1f} deg, tau "
          f"{ayar.dikey_terminal_tau_s:.1f} s)")
    print(f"[mpc] dogrudan dikey hata (P)="
          f"{'ACIK' if ayar.dikey_hata else 'KAPALI'} "
          f"(YILDIZ_DIKEY_HATA, q={ayar.q_dikey_hata:.2f}"
          f"x{ayar.dikey_hata_carpani:.2f}, "
          f"{ayar.dikey_hata_menzil_m:.0f} m [YILDIZ_DIKEY_RAMPA_BAS] -> "
          f"{ayar.dikey_hata_tam_m:.0f} m [YILDIZ_DIKEY_RAMPA_SON], olu bant "
          f"+-{ayar.dikey_hata_rahat_deg:.1f} deg)")
    print(f"[mpc] t_go sekilli dikey tau="
          f"{'ACIK' if ayar.dikey_tgo else 'KAPALI'} "
          f"(YILDIZ_DIKEY_TGO, k={ayar.dikey_tgo_k:.2f}"
          f"x{ayar.dikey_tgo_carpani:.2f}"
          f"={ayar.dikey_tgo_k * ayar.dikey_tgo_carpani:.2f}, tau "
          f"{ayar.dikey_tgo_tau_min_s:.2f}..{ayar.dikey_tgo_tau_max_s:.2f} s, "
          f"kapanma kapisi {ayar.dikey_tgo_kapanma_min_mps:.1f} m/s)")
    # HANGI KOMBINASYON KOSUYOR: uc dugme bagimsiz acildigi icin kolun
    # adi tek satirda okunabilsin (A/B tablosunu CSV'den degil logun
    # ilk satirlarindan etiketliyoruz).
    kol = '+'.join([ad for ad, acik in (
        ('D(TERMINAL)', ayar.dikey_terminal), ('P(HATA)', ayar.dikey_hata),
        ('TGO', ayar.dikey_tgo)) if acik]) or 'YOK (temel kol)'
    print(f"[mpc] DIKEY KANAL KOLU = {kol}")
    # APN ve DOYUMLU EYLEYICI de loga yazilsin. LOG KORLUGU DUZELTMESI
    # (2026-08-08): bu iki dugme eklendiginde banner satiri yazilmamisti;
    # kosu klasorunden hangi kolun uctugu ANCAK mpc_tani kolonlarina
    # (apn_a / tau_eff) bakarak anlasilabiliyordu. Yukaridaki dikey
    # dugmelerle ayni disiplin: kol adi logun ilk satirlarinda dursun.
    print(f"[mpc] APN (hedef yanal ivmesi)="
          f"{'ACIK' if ayar.apn else 'KAPALI'} "
          f"(YILDIZ_APN, carpan {ayar.apn_carpani:.2f}, tau "
          f"{ayar.apn_tau_s:.2f} s, olu bant "
          f"{ayar.apn_olu_bant_mps2:.2f}, tavan "
          f"{ayar.apn_a_tavani_mps2:.1f} m/s2)")
    print(f"[mpc] DOYUMLU EYLEYICI="
          f"{'ACIK' if ayar.eyleyici else 'KAPALI'} "
          f"(YILDIZ_EYLEYICI, tau_lin {ayar.eyleyici_tau_lin_s:.2f} s "
          f"[YILDIZ_TAU_LIN], a_max {ayar.eyleyici_a_max_mps2:.2f} m/s2 "
          f"[YILDIZ_A_MAX]; DIKEY a_max {ayar.eyleyici_a_max_z_mps2:.2f} "
          f"m/s2 [YILDIZ_A_MAX_Z], dikey tau_lin "
          f"{ayar.eyleyici_tau_lin_z_s:.2f} s [YILDIZ_TAU_LIN_Z])")
    print(f"[mpc] KOR PN (kor suzulmede yasayi surdur)="
          f"{'ACIK' if ayar.kor_pn else 'KAPALI'} "
          f"(YILDIZ_KOR_PN, azami {ayar.kor_pn_azami_s:.1f} s "
          f"[YILDIZ_KOR_PN_AZAMI_S], d sonum tau {ayar.kor_pn_tau_s:.2f} s "
          f"[YILDIZ_KOR_PN_TAU], |ex| payi {ayar.kor_pn_ex_pay_deg:.1f} deg, "
          f"|ex| MUTLAK {ayar.kor_pn_ex_mutlak_deg:.1f} deg "
          f"[YILDIZ_KOR_PN_EX_MUTLAK], menzil kapisi "
          f"{ayar.kor_pn_menzil_m:.1f} m [YILDIZ_KOR_PN_MENZIL])")
    print(f"[mpc] ILERLEME SAATI="
          f"{'ACIK' if ayar.ilerleme_saat else 'KAPALI'} "
          f"(YILDIZ_ILERLEME_SAAT, taban {ayar.iska_zaman_asimi_s:.0f} s, "
          f"kapanma esigi {ayar.ilerleme_kapanma_esigi_mps:.1f} m/s, "
          f"yeni-min penceresi {ayar.ilerleme_pencere_s:.1f} s, "
          f"kazanc {ayar.ilerleme_kazanci:.2f}, "
          f"mutlak tavan {ayar.ilerleme_tavan_s:.0f} s)")
    # COZUCU BUTCESI kosunun basinda loga yazilsin: butce_kesti oranini
    # sonradan yorumlarken hangi tavanla kosuldugu bilinsin.
    print(f"[mpc] cozucu butcesi="
          f"{'BOL' if ayar.iterasyon_tavani > 26 else 'NORMAL'} "
          f"(YILDIZ_COZUCU_BOL, iterasyon tavani {ayar.iterasyon_tavani}, "
          f"sure {ayar.sure_butcesi_ms:.0f} ms; SIM olcum kalitesi icin, "
          f"donanim karari AYRI)")

    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    tani = args.tani_log or str(Path(__file__).resolve().parent / 'logs'
                                / f"mpc_tani_{damga}.csv")
    from goruntulu_temel import GoruntuluDongu
    GoruntuluDongu(MpcKontrolcu(ayar, tani_log=tani),
                   loop_hz=args.loop_hz).calistir(args.sure)


def _blok_uret(n):
    """n adim icin makul artan blok deseni (toplam = n)."""
    desen = []
    kalan = n
    uzunluk = 1
    while kalan > 0:
        u = min(uzunluk, kalan)
        desen.append(u)
        kalan -= u
        if len(desen) % 2 == 0:
            uzunluk += 1
    return tuple(desen)


if __name__ == '__main__':
    main()
