#!/usr/bin/env python3
"""
gimbal_kanit.py - gimbal zincirinin CANLI UCUSTA calistigini sayiyla gosterir
============================================================================
GIMBAL DALI (2026-08-05): OLCUT DEGISTI. Kamera artik govdeye sabit degil,
kendini stabilize eden FIZIKSEL tek eksen (tilt) gimbalde. Eski olcut
(corr(stab_ey, pitch) < 0.5 * corr(ham_ey, pitch)) artik BOS bir sinav:
gövde pitch'i goruntuye HIC yansimadigi icin ham_ey'de temizlenecek pitch
zaten YOKTUR - payda sifira gider, olcut anlamsizlasir.

Yerine IKI KATMANLI kanit:

  (a) FIZIKSEL KATMAN - gimbalin kendisi:
      |corr(ham_ey, pitch)| < 0.30
      HAM (de-rotasyon oncesi) dikey hata govde pitch'inden BAGIMSIZ olmali.
      Bu, mekanik stabilizasyonun gercekten calistigini gosterir; yazilim
      hic devreye girmeden once olculur. Eski dunyada bu korelasyon YUKSEK
      cikardi (hata hedeften degil ucagin yatmasindan gelirdi).

  (b) YAZILIM KATMANI - de-rotasyonun kalan isi:
      Tilt gimbal roll'u COZMEZ; kamera govdeyle birlikte yatar. Yatan
      kadrajda yatayda ex kadar acik olan hedef dikeyde ~ ex*sin(roll) kadar
      kayar. Olcut: |corr(stab_ey, ex*sin(roll))| < |corr(ham_ey, ex*sin(roll))|
      (regresor HER IKI KATMANDA AYNI: ham_ex*sin(roll)).
      Ikisi de zayifsa (< 0.30) hukum "roll etkisi zaten kucuk" olur -
      de-rotasyonun temizleyecek bir sey bulamamasi BASARISIZLIK DEGILDIR.

Ek olarak tilt zincirinin SAGLIGI raporlanir (olcut degil, teshis):
  - medyan(tilt_status - tilt_cmd) : servo ofseti (olu bant ~0.17 deg)
  - p95(tilt_yas_ms)               : durum topic'inin bayatligi
  - maks |eklem_deg - eklem_acisi(tilt_status, pitch, roll)| : olcum
    zincirinin kendi ic tutarliligi (bbox_to_redis ile bu arac ayni
    kapali formu kullaniyor mu)

VERI YETERSIZLIGI: korelasyon hukmu ancak UYARIM varsa verilebilir. Govde
yerde durgunken (std(pitch) < 1 deg) korelasyon gurultunun gurultuye
oranidir. Bu durumda arac "VERI YETERSIZ" der ve 2 ile ciakr - GECTI de
demez, KALDI da.

CIKIS KODU: 0 = GECTI, 1 = KALDI, 2 = KARARSIZ (yetersiz uyarim).

TARIHSEL MOD: tilt kolonlari (tilt_cmd_deg/tilt_status_deg/eklem_deg) olmayan
eski CSV'lerle cagrilirsa eski olcut (sanal gimbal, govdeye sabit kamera)
kosulur.

YATAY EKSENDE TUTUM KORELASYONU OLCUT DEGILDIR (2026-08-02'de anlasildi).
Bank-to-turn gudumde roll'un KENDISI kerteriz hatasiyla komutlanir; stabilize
yatay hata da (dogru calistiginda) gercek kerterizi olcer. Yani stab_ex ile
roll arasindaki bag ENDOJENDIR - kapali dongunun kendi eslesmesidir, gimbal
artigi degil. Olcum: corr(stab_ex, -gercek_yan) = 0.992 (Gazebo yer gercegi,
6391 kare). Yatay kanal ancak YER GERCEGIYLE dogrulanabilir: --gercek-yon.
"""

import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:                                    # kapali form TEK KAYNAK: yildizlar_gimbal
    from yildizlar_gimbal import eklem_acisi       # noqa: E402
except Exception:                       # numpy yoksa: ayni formulun yerel kopyasi
    def eklem_acisi(eps_cam_deg, pitch_rad, roll_rad):
        A = math.sin(pitch_rad)
        B = math.cos(pitch_rad) * math.cos(roll_rad)
        R = math.hypot(A, B)
        s = max(-1.0, min(1.0, math.sin(math.radians(eps_cam_deg)) / max(R, 1e-9)))
        return math.degrees(math.asin(s) - math.atan2(A, B))

# Uyarim esikleri: bunlarin altinda korelasyon hukmu verilmez.
UYARIM_PITCH_DEG = 1.0        # std(pitch); altinda "govde savrulmadi"
UYARIM_ROLL_REG = 0.05        # std(ham_ex*sin(roll)) [deg]; altinda roll etkisi yok
ZAYIF = 0.30                  # korelasyon "zayif" esigi (her iki katman)


def oku(yol):
    with open(yol) as f:
        return list(csv.DictReader(f))


def sayi(satirlar, ad):
    out = []
    for s in satirlar:
        try:
            out.append(float(s[ad]))
        except (ValueError, KeyError, TypeError):
            out.append(float('nan'))
    return out


def temiz(*diziler):
    n = len(diziler[0])
    tut = [i for i in range(n)
           if all(not math.isnan(d[i]) for d in diziler)]
    return [[d[i] for i in tut] for d in diziler]


def std(v):
    if len(v) < 2:
        return float('nan')
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def zaman_sutunu(satirlar):
    """Kare zamani: yeni loglarda 't_kare', en eskilerde 't'."""
    return 't_kare' if 't_kare' in satirlar[0] else 't'


def medyan(v):
    if not v:
        return float('nan')
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def yuzdelik(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def korelasyon(a, b):
    if len(a) < 3:
        return float('nan')
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa == 0 or sb == 0:
        return float('nan')
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def egim(x, y):
    """y ~ a + k*x en kucuk kareler; k dondurur."""
    n = len(x)
    if n < 3:
        return float('nan')
    mx, my = sum(x) / n, sum(y) / n
    sxy = sum((p - mx) * (q - my) for p, q in zip(x, y))
    sxx = sum((p - mx) ** 2 for p in x)
    return sxy / sxx if sxx else float('nan')


def interp(ts, vs, t):
    """ts uzerinde dogrusal ara deger (ts artan olmali)."""
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    t0, t1 = ts[i - 1], ts[i]
    if t1 == t0:
        return vs[i]
    w = (t - t0) / (t1 - t0)
    return vs[i - 1] + w * (vs[i] - vs[i - 1])


def yer_gercegi_dogrula(gt_yol, t_kare, sx, roll):
    """Yatay kanali Gazebo yer gercegiyle dogrular.

    DIKKAT: gercek_yon.csv'nin pitch_deg kolonu KULLANILMAZ - uretici script
    Gazebo ENU tutumunu NED'e cevirirken pitch'in isaretini ters birakmis
    (olcum: corr(pitch_gt, pitch_gimbal) = -0.988, roll icin +0.994). Burada
    yalnizca konumdan turetilen gercek_yan_deg kolonu okunur; o kolon
    kuaterniyondan degil iki aracin konum farkindan geldigi icin saglamdir.

    Isaret sozlesmesi: gimbalin ex'i saga pozitif, gercek_yan_deg ufuk
    cercevesinde saga pozitif olcer ama zit yonlu tanimlidir -> -gercek_yan.
    """
    gt = oku(gt_yol)
    tg = sayi(gt, 't')
    yan = sayi(gt, 'gercek_yan_deg')
    tg, yan = temiz(tg, yan)
    if len(tg) < 10:
        return None
    ikili = sorted(zip(tg, yan))
    tg = [p[0] for p in ikili]
    yan = [p[1] for p in ikili]

    # ZAMAN ESLEME: en iyi sabit kaymayi +-500 ms icinde korelasyonla bul.
    # Iki kayit ayri sureclerden gelir; ortak saat monotonic olsa da yazma
    # anlari arasinda sabit bir kayma kalabilir.
    en_iyi = (None, -2.0)
    for adim in range(-50, 51):
        kayma = adim / 100.0
        a, b = [], []
        for t, s in zip(t_kare, sx):
            th = t + kayma
            if not (tg[0] <= th <= tg[-1]):
                continue
            a.append(s)
            b.append(-interp(tg, yan, th))
        if len(a) < 50:
            continue
        r = korelasyon(a, b)
        if not math.isnan(r) and abs(r) > en_iyi[1]:
            en_iyi = (kayma, abs(r))
    kayma = en_iyi[0]
    if kayma is None:
        return None

    ex, ger, rl = [], [], []
    for t, s, r_ in zip(t_kare, sx, roll):
        th = t + kayma
        if not (tg[0] <= th <= tg[-1]):
            continue
        ex.append(s)
        ger.append(-interp(tg, yan, th))
        rl.append(r_)
    if len(ex) < 50:
        return None

    # KERTERIZ KAZANCI: olculen ex, gercek kerterizin kac katı? 1.000 olmali.
    # (Duzeltme oncesi aim'in Ry donusu yuzunden 0.910 olculuyordu.)
    k = egim(ger, ex)
    kalinti = [e - k * g for e, g in zip(ex, ger)]
    sr = [math.sin(math.radians(r_)) for r_ in rl]
    egim_roll = egim(sr, kalinti)
    return {'n': len(ex), 'kayma_ms': kayma * 1000.0, 'corr': en_iyi[1],
            'kazanc': k, 'egim': egim_roll, 'std': std(kalinti)}


def sacilim_yaz(hx, hy, sx, sy):
    print("  SACILIM (std, derece)             ham -> stabilize   [bilgi]")
    ky = std(hy) / std(sy) if std(sy) else float('inf')
    kx = std(hx) / std(sx) if std(sx) else float('inf')
    print(f"     dikey  hata : {std(hy):6.2f} -> {std(sy):6.2f}   ({ky:4.1f}x daha dar)")
    print(f"     yatay  hata : {std(hx):6.2f} -> {std(sx):6.2f}   ({kx:4.1f}x daha dar)")


def yatay_yer_gercegi(gercek_yon, tk, sx, roll):
    """--gercek-yon blogu. (gecti_mi, karar_verildi_mi) doner."""
    print()
    print("  YER GERCEGI (Gazebo) ile YATAY KANAL")
    d = yer_gercegi_dogrula(gercek_yon, tk, sx, roll)
    if d is None:
        print("     eslesen ornek yok - zaman pencereleri ortusmuyor")
        return False, True
    print(f"     eslesen ornek: {d['n']}  "
          f"(zaman kaymasi {d['kayma_ms']:+.0f} ms, |corr| {d['corr']:.4f})")
    print(f"     kerteriz kazanci      : {d['kazanc']:.4f}  "
          f"(1.000 olmali; aim duzeltmesi oncesi 0.910 olculmustu)")
    print(f"     kalinti sin(roll) egimi: {d['egim']:+.3f} deg  (|.|<1.0 olmali)")
    print(f"     kalinti sacilimi       : {d['std']:.3f} deg  (<1.0 olmali)")
    yatay_ok = abs(d['egim']) < 1.0 and d['std'] < 1.0
    print("     ->", "YATAY KANAL YER GERCEGIYLE DOGRULANDI" if yatay_ok
          else "YATAY KANALDA ACIKLANMAMIS ARTIK VAR")
    return yatay_ok, True


# --------------------------------------------------------------- GIMBAL MOD

def olcut_gimbal(satirlar, gercek_yon):
    tk = sayi(satirlar, zaman_sutunu(satirlar))
    roll = sayi(satirlar, 'roll_deg')
    pitch = sayi(satirlar, 'pitch_deg')
    hx = sayi(satirlar, 'ham_ex_deg')
    hy = sayi(satirlar, 'ham_ey_deg')
    sx = sayi(satirlar, 'stab_ex_deg')
    sy = sayi(satirlar, 'stab_ey_deg')
    tk, roll, pitch, hx, hy, sx, sy = temiz(tk, roll, pitch, hx, hy, sx, sy)
    n = len(roll)
    if n < 50:
        print(f"VERI YETERSIZ: yalniz {n} tam satir")
        return 2

    print(f"=== FIZIKSEL TILT GIMBAL CANLI KANITI ({n} tespit karesi) ===")
    print(f"  govde tutumu: roll std {std(roll):5.2f} deg   "
          f"pitch std {std(pitch):5.2f} deg")
    print()
    sacilim_yaz(hx, hy, sx, sy)
    print()

    # ---- (a) FIZIKSEL KATMAN -------------------------------------------
    print("  (a) FIZIKSEL KATMAN: gimbal govde pitch'ini ayiriyor mu?")
    r_hy_p = abs(korelasyon(hy, pitch))
    r_sy_p = abs(korelasyon(sy, pitch))
    print(f"      |corr(ham_ey , pitch)|  : {r_hy_p:5.3f}   (olcut: < {ZAYIF:.2f})")
    print(f"      |corr(stab_ey, pitch)|  : {r_sy_p:5.3f}   [bilgi]")
    pitch_uyarim = std(pitch)
    if math.isnan(r_hy_p) or pitch_uyarim < UYARIM_PITCH_DEG:
        fiz = None
        print(f"      -> VERI YETERSIZ: pitch uyarimi {pitch_uyarim:.2f} deg "
              f"(< {UYARIM_PITCH_DEG:.1f} deg). Govde savrulmadan bu korelasyon")
        print(f"         gurultunun gurultuye oranidir; hukum verilemez.")
    else:
        fiz = r_hy_p < ZAYIF
        print("      ->", "GIMBAL PITCH'I AYIRIYOR" if fiz
              else "*** HAM HATA HALA PITCH'E BAGLI - gimbal stabilize etmiyor ***")
    print()

    # ---- (b) YAZILIM KATMANI -------------------------------------------
    # Tilt gimbal roll'u cozmez: yatan kadrajda yatayda ex acik olan hedef
    # dikeyde ~ ex*sin(roll) kayar. Regresor HER IKI KATMANDA AYNI olmali,
    # yoksa "hangi sinyalin ne kadari temizlendi" karsilastirmasi anlamsizdir.
    reg = [x * math.sin(math.radians(r)) for x, r in zip(hx, roll)]
    print("  (b) YAZILIM KATMANI: de-rotasyon roll sizintisini temizliyor mu?")
    print(f"      regresor u = ham_ex*sin(roll):  std {std(reg):.4f} deg")
    r_hy_u = abs(korelasyon(hy, reg))
    r_sy_u = abs(korelasyon(sy, reg))
    print(f"      |corr(ham_ey , u)|      : {r_hy_u:5.3f}")
    print(f"      |corr(stab_ey, u)|      : {r_sy_u:5.3f}   (olcut: ham'dan KUCUK)")
    if math.isnan(r_hy_u) or math.isnan(r_sy_u) or std(reg) < UYARIM_ROLL_REG:
        yaz = None
        print(f"      -> VERI YETERSIZ: roll sizinti regresoru neredeyse sabit "
              f"(std < {UYARIM_ROLL_REG:.2f} deg).")
        print(f"         Temizlenecek bir sey uretilmemis; hukum verilemez.")
    elif r_hy_u < ZAYIF and r_sy_u < ZAYIF:
        yaz = True
        print(f"      -> ROLL ETKISI ZATEN KUCUK (ikisi de < {ZAYIF:.2f}); "
              f"de-rotasyon zarar da vermiyor")
    else:
        yaz = r_sy_u < r_hy_u
        print("      ->", "DE-ROTASYON ROLL SIZINTISINI AZALTIYOR" if yaz
              else "*** DE-ROTASYON ROLL SIZINTISINI AZALTMIYOR ***")
    print()

    # ---- TILT ZINCIRI SAGLIGI (teshis) ---------------------------------
    tilt_saglik(satirlar)

    # ---- HUKUM ----------------------------------------------------------
    print()
    kararlar = [fiz, yaz]
    if any(k is False for k in kararlar):
        print("SONUC: KALDI - yukaridaki *** satirlarina bakin")
        kod = 1
    elif any(k is None for k in kararlar):
        print("SONUC: KARARSIZ - yeterli tutum uyarimi olan bir kosuyla tekrarlayin")
        print("       (yerde durgun ya da cok duz ucus kaydi hukum vermeye yetmez)")
        kod = 2
    else:
        print("SONUC: GIMBAL ZINCIRI CALISIYOR (fiziksel + yazilim katmani)")
        kod = 0

    if gercek_yon:
        yatay_ok, _ = yatay_yer_gercegi(gercek_yon, tk, sx, roll)
        if not yatay_ok:
            kod = 1
    return kod


def tilt_saglik(satirlar):
    """Tilt zincirinin teshis raporu (OLCUT DEGIL - sayilar burada durur)."""
    print("  TILT ZINCIRI SAGLIGI [teshis, olcut degil]")
    cmd = sayi(satirlar, 'tilt_cmd_deg')
    dur = sayi(satirlar, 'tilt_status_deg')
    yas = sayi(satirlar, 'tilt_yas_ms')
    ekl = sayi(satirlar, 'eklem_deg')
    roll = sayi(satirlar, 'roll_deg')
    pitch = sayi(satirlar, 'pitch_deg')

    c, d = temiz(cmd, dur)
    if c:
        fark = [b - a for a, b in zip(c, d)]
        print(f"      medyan(tilt_status - tilt_cmd) : {medyan(fark):+.3f} deg  "
              f"(olu bant ~0.17 deg; buyuk fark = servo doymus/yuklu)")
    else:
        print("      tilt_cmd/tilt_status ortak satiri yok")

    (y,) = temiz(yas)
    if y:
        print(f"      tilt_yas_ms  p95 {yuzdelik(y, 0.95):6.0f} ms   "
              f"medyan {medyan(y):5.0f}   maks {max(y):6.0f}   "
              f"(>1500 ms = komut degerine dusuluyor)")
    else:
        print("      tilt_yas_ms kolonu bos")

    e, dd, pp, rr = temiz(ekl, dur, pitch, roll)
    if e:
        hata = [abs(a - eklem_acisi(b, math.radians(p), math.radians(r)))
                for a, b, p, r in zip(e, dd, pp, rr)]
        basit = [abs(a - (b - p)) for a, b, p in zip(e, dd, pp)]
        print(f"      |eklem_deg - eklem_acisi(status,pitch,roll)| maks "
              f"{max(hata):.4f} deg  (zincir ic tutarliligi, <0.01 olmali)")
        print(f"      |eklem_deg - (status - pitch)| maks {max(basit):.4f} deg  "
              f"(roll=0 sadelestirmesinin gecerlilik payi)")
    else:
        print("      eklem_deg kolonu bos - tilt modu kapali kosulmus olabilir")


# ------------------------------------------------------------ TARIHSEL MOD

def olcut_tarihsel(satirlar, gercek_yon):
    """ESKI OLCUT: govdeye SABIT kamera + SANAL gimbal dunyasi.

    Ham dikey hata govde pitch'iyle guclu korelasyon gosterirdi (hata
    hedeften degil ucagin yatmasindan gelirdi); sanal gimbal bu bagi
    kesmeliydi. Fiziksel gimbalde ham_ey'de pitch ZATEN olmadigi icin bu
    olcut bos bir sinav - yalniz eski kayitlar icin ayakta.
    """
    tk = sayi(satirlar, zaman_sutunu(satirlar))
    roll = sayi(satirlar, 'roll_deg')
    pitch = sayi(satirlar, 'pitch_deg')
    hx = sayi(satirlar, 'ham_ex_deg')
    hy = sayi(satirlar, 'ham_ey_deg')
    sx = sayi(satirlar, 'stab_ex_deg')
    sy = sayi(satirlar, 'stab_ey_deg')
    aim = sayi(satirlar, 'aim_deg')
    tk, roll, pitch, hx, hy, sx, sy, aim = temiz(tk, roll, pitch, hx, hy,
                                                 sx, sy, aim)
    n = len(roll)
    if n < 50:
        print(f"VERI YETERSIZ: yalniz {n} tam satir")
        return 2

    print(f"=== TARIHSEL MOD: SANAL GIMBAL CANLI KANITI ({n} tespit karesi) ===")
    print("  (CSV'de tilt kolonlari yok -> govdeye sabit kamera varsayimi)")
    print(f"  govde tutumu: roll std {std(roll):5.2f} deg   "
          f"pitch std {std(pitch):5.2f} deg")
    print()
    sacilim_yaz(hx, hy, sx, sy)
    print()
    print("  TUTUMLA KORELASYON (eski asil kanit)  ham -> stabilize")
    r_hy = abs(korelasyon(hy, pitch)); r_sy = abs(korelasyon(sy, pitch))
    r_hx = abs(korelasyon(hx, roll));  r_sx = abs(korelasyon(sx, roll))
    print(f"     |corr(dikey hata , pitch)| : {r_hy:5.3f} -> {r_sy:5.3f}")
    print(f"     |corr(yatay hata , roll )| : {r_hx:5.3f} -> {r_sx:5.3f}")
    print()
    if aim:
        print(f"  aim: basta {aim[0]:+.2f} -> sonda {aim[-1]:+.2f} deg "
              f"(trim {aim[-1] - aim[0]:+.2f} deg oynatti)")
        print()
    dikey_ok = (r_sy < 0.5 * r_hy and std(sy) < std(hy))
    print("SONUC:", "DIKEY EKSENDE GIMBAL CALISIYOR" if dikey_ok
          else "DIKEY EKSENDE BEKLENEN IYILESME YOK")
    print(f"  (yatay roll korelasyonu {r_hx:.3f} -> {r_sx:.3f} bilgi amaclidir: "
          f"bank-to-turn'de endojendir, olcut degildir)")
    kod = 0 if dikey_ok else 1
    if gercek_yon:
        yatay_ok, _ = yatay_yer_gercegi(gercek_yon, tk, sx, roll)
        if not yatay_ok:
            kod = 1
    return kod


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv')
    p.add_argument('--gercek-yon', metavar='CSV',
                   help='Gazebo yer gercegi (gercek_yon.csv). Verilirse yatay '
                        'kanal da olcute girer.')
    p.add_argument('--tarihsel', action='store_true',
                   help='tilt kolonlari olsa bile ESKI olcutu kosur')
    a = p.parse_args()
    satirlar = oku(a.csv)
    if len(satirlar) < 50:
        raise SystemExit(f"yetersiz ornek: {len(satirlar)}")

    tilt_var = all(k in satirlar[0] for k in
                   ('tilt_cmd_deg', 'tilt_status_deg', 'eklem_deg'))
    if tilt_var and not a.tarihsel:
        return olcut_gimbal(satirlar, a.gercek_yon)
    return olcut_tarihsel(satirlar, a.gercek_yon)


if __name__ == '__main__':
    sys.exit(main())
