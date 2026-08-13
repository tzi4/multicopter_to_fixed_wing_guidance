#!/usr/bin/env python3
"""
gimbal_zaman_kalibre.py - kamera boru hatti gecikmesini OLCER
=============================================================
Sorun: kare ile tutum ornegi ayni anda uretilmiyor. Kamera bir kareyi
YAKALADIGI an ile o karenin bize ULASTIGI an arasinda sabit bir gecikme var
(sensor okuma + ROS/driver + aktarim). Tutum ise cok daha kisa bir yoldan
geliyor. Bu fark de-rotasyona dogrudan hata olarak giriyor: 30 derece/s
govde hizinda 30 ms kayma ~1 derece demek.

REFERANS SINYALI DEGISTI (gimbal dali, 2026-08-05)
--------------------------------------------------
Eski olcut: dogru gecikmede |corr(stab_ey, pitch)| MINIMUM olur. Kamera
fiziksel tilt gimbalde stabilize oldugundan govde pitch'i goruntuye ARTIK
YANSIMIYOR - yani minimize edilecek pitch sinyali kalmadi; egri her gecikmede
duz cikar ve arac rastgele bir minimum secer.

Yeni olcut: TILT GIMBAL ROLL'U COZMEZ. Kamera govdeyle birlikte yatar, roll
goruntuye TAM olarak yansir ve onu YALNIZCA yazilim de-rotasyonu temizler.
Yatan kadrajda yatayda ex kadar acik duran hedef dikeyde ~ ex*sin(roll) kadar
kayar. Dolayisiyla:

    dogru gecikmede  |corr(stab_ey, ham_ex*sin(roll(t-gecikme)))|  MINIMUM

Yanlis gecikmede de-rotasyon yanlis anin roll'unu kullanir, sizintinin bir
kismi stab_ey'de kalir ve korelasyon buyur. Regresordeki ham_ex piksel
geometrisinden gelir (bbox_cx), tutumdan BAGIMSIZDIR; gecikmeye duyarli olan
yalnizca roll'dur - aranan sey de tam odur.

NEDEN BU, "tilt_status ile ham_my hizalamasi" DEGIL: o yontem gimbal durum
topic'inin kareye gore gecikmesini olcer (kamera boru hattinin degil), hedefin
dikeyde hareket etmesini sart kosar ve araca yeni bir cikarim zinciri sokar.
Buradaki secim mevcut mimariye EN AZ INVAZIV olanidir: tarama dongusu,
SanalGimbal yeniden hesabi ve raporlama AYNEN kaldi, yalnizca amac fonksiyonu
degisti. Ek olarak, CSV'de tilt_status_deg varsa her adayda eklem acisi
KAYDIRILMIS tutumla yeniden turetilir (eklem_acisi kapali formu) - ucus
yolundaki zincirin aynisi.

Simulasyon saatine bagli DEGIL: iki log da time.monotonic() ile
damgalanmistir, gercek ucusta da ayni sekilde uretilir.

Kullanim:
    tools/gimbal_zaman_kalibre.py run/kanit/gimbal.csv run/kanit/tutum.csv
    tools/gimbal_zaman_kalibre.py gimbal.csv        # tutum ayni CSV'den
                                                   # (ARTIK gecikme olculur)
"""

import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yildizlar_gimbal import SanalGimbal, eklem_acisi        # noqa: E402

UYARIM_ROLL_DEG = 1.0      # std(roll); altinda gecikme cikarilamaz
UYARIM_REG = 0.05          # std(ham_ex*sin(roll)) [deg]; altinda referans yok
AYIRT_EDICILIK = 0.02      # egrinin tepe-dip farki; altinda minimum anlamsiz


def oku(yol):
    with open(yol) as f:
        return list(csv.DictReader(f))


def korelasyon(a, b):
    n = len(a)
    if n < 3:
        return float('nan')
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa == 0 or sb == 0:
        return float('nan')
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def std(v):
    n = len(v)
    if n < 2:
        return float('nan')
    m = sum(v) / n
    return math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))


class TutumSerisi:
    """Zaman damgali roll/pitch serisi; istenen ana dogrusal interpolasyon."""

    def __init__(self, satirlar):
        # Zaman sutunu: ayri tutum logunda 't_yerel', ayni CSV'den okunurken
        # 't_kare' (en eski loglarda 't').
        ad = None
        for aday in ('t_yerel', 't_kare', 't'):
            if satirlar and aday in satirlar[0]:
                ad = aday
                break
        self.zaman_ad = ad
        self.t, self.roll, self.pitch = [], [], []
        if ad is None:
            return
        for s in satirlar:
            try:
                self.t.append(float(s[ad]))
                self.roll.append(math.radians(float(s['roll_deg'])))
                self.pitch.append(math.radians(float(s['pitch_deg'])))
            except (ValueError, KeyError, TypeError):
                continue

    @staticmethod
    def _karistir(a, b, k):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        return a + d * k

    def at(self, th):
        if not self.t or th <= self.t[0]:
            return (self.roll[0], self.pitch[0]) if self.t else None
        if th >= self.t[-1]:
            return self.roll[-1], self.pitch[-1]
        i = bisect.bisect_left(self.t, th)
        t0, t1 = self.t[i - 1], self.t[i]
        k = 0.0 if t1 == t0 else (th - t0) / (t1 - t0)
        return (self._karistir(self.roll[i - 1], self.roll[i], k),
                self._karistir(self.pitch[i - 1], self.pitch[i], k))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('gimbal_csv')
    p.add_argument('tutum_csv', nargs='?',
                   help='ayri tutum logu; verilmezse tutum ayni CSV\'den '
                        'okunur (o zaman ARTIK gecikme olculur)')
    p.add_argument('--mount', type=float, default=0.0,
                   help='sabit montaj acisi [deg]. GIMBAL DALINDA 0: kamera '
                        'gimbal tilt link\'inde, govdeye gore acisi CANLI '
                        'eklem acisidir (tilt_status_deg kolonundan turetilir).')
    p.add_argument('--tara', default='-40,200,5', metavar='BAS,BIT,ADIM',
                   help='gecikme taramasi (ms)')
    a = p.parse_args()

    kareler = oku(a.gimbal_csv)
    ayni_dosya = a.tutum_csv is None
    seri = TutumSerisi(kareler if ayni_dosya else oku(a.tutum_csv))
    if len(kareler) < 100 or len(seri.t) < 100:
        raise SystemExit(f"yetersiz veri: {len(kareler)} kare, {len(seri.t)} tutum")
    if ayni_dosya:
        print("NOT: ayri tutum logu verilmedi; tutum kare CSV'sinden okunuyor.")
        print("     O kolonlar bbox_to_redis'in UYGULADIGI gecikmeyle zaten")
        print("     interpole edilmis -> bulunan sayi ARTIK (kalan) gecikmedir.")

    # Sutun adi eski loglarda 't', yenilerde 't_kare'.
    zaman_ad = 't_kare' if 't_kare' in kareler[0] else 't'
    tilt_var = 'tilt_status_deg' in kareler[0]
    ornekler = []
    for s in kareler:
        try:
            t = float(s[zaman_ad])
            cx, cy = float(s['bbox_cx']), float(s['bbox_cy'])
            aim_e = float(s['aim_etkin_deg'])
        except (ValueError, KeyError, TypeError):
            continue
        eps = None
        if tilt_var:
            try:
                eps = float(s['tilt_status_deg'])
            except (ValueError, TypeError):
                eps = None
        ornekler.append((t, cx, cy, aim_e, eps))
    if len(ornekler) < 100:
        raise SystemExit(f"yetersiz kullanilabilir kare: {len(ornekler)}")

    g = SanalGimbal(mount_phys_pitch_deg=a.mount)
    # REFERANS: hedefin kadrajdaki YATAY acikligi, pikselden dogrudan
    # (tutumdan bagimsiz -> gecikmeye duyarsiz). Roll sizintisi bunun
    # sin(roll) ile carpimidir.
    ham_ex = [math.degrees(math.atan((cx - g.cx) / g.fx))
              for _, cx, _, _, _ in ornekler]

    bas, bit, adim = (float(x) for x in a.tara.split(','))
    print(f"kare: {len(ornekler)}   tutum ornegi: {len(seri.t)}   "
          f"tutum araligi: {seri.t[-1]-seri.t[0]:.0f} s   "
          f"tilt kolonu: {'VAR' if tilt_var else 'yok'}")
    print(f"tarama: {bas:.0f} .. {bit:.0f} ms, adim {adim:.0f} ms")
    print()
    print("  gecikme   |corr(stab_ey, ex*sin(roll))|   std(stab_ey)  |corr(.,pitch)|")

    en_iyi = None
    sonuc = []
    reg_std_max = 0.0
    roll_std_max = 0.0
    ms = bas
    while ms <= bit + 1e-9:
        ey_l, reg_l, pitch_l, roll_l = [], [], [], []
        for (t_kare, cx, cy, aim_e, eps), ex in zip(ornekler, ham_ex):
            tut = seri.at(t_kare - ms / 1000.0)
            if tut is None:
                continue
            roll, pitch = tut
            eklem = None if eps is None else eklem_acisi(eps, pitch, roll)
            g.aim_pitch_deg = aim_e
            g._son_aim_deg = None
            _, ey = g.aci_hatasi(cx, cy, roll, pitch, None, eklem_deg=eklem)
            ey_l.append(ey)
            reg_l.append(ex * math.sin(roll))
            pitch_l.append(math.degrees(pitch))
            roll_l.append(math.degrees(roll))
        r = abs(korelasyon(ey_l, reg_l))
        rp = abs(korelasyon(ey_l, pitch_l))
        sd = std(ey_l)
        reg_std_max = max(reg_std_max, std(reg_l))
        roll_std_max = max(roll_std_max, std(roll_l))
        if not math.isnan(r):
            sonuc.append((ms, r, sd, rp))
            if en_iyi is None or r < en_iyi[1]:
                en_iyi = (ms, r, sd, rp)
        else:
            sonuc.append((ms, float('nan'), sd, rp))
        ms += adim

    for ms, r, sd, rp in sonuc:
        isaret = '  <<< EN IYI' if en_iyi and ms == en_iyi[0] else ''
        cubuk = '' if math.isnan(r) else '#' * int(r * 50)
        rs = '  nan' if math.isnan(r) else f"{r:5.3f}"
        print(f"  {ms:6.0f} ms   {rs}  {cubuk:<25s} {sd:6.2f}  "
              f"{rp:5.3f}{isaret}")

    print()
    print(f"roll uyarimi: std {roll_std_max:.2f} deg   "
          f"referans u = ex*sin(roll): std {reg_std_max:.4f} deg")

    # --- HUKUM: once verinin bu soruyu yanitlayip yanitlamadigina bak -------
    if en_iyi is None or roll_std_max < UYARIM_ROLL_DEG or reg_std_max < UYARIM_REG:
        print()
        print("VERI YETERSIZ: govde roll'u savrulmamis (ya da hedef kadrajin")
        print("  tam ortasinda durmus), yani de-rotasyonun temizleyecegi bir")
        print("  sizinti URETILMEMIS. Gecikme bu kayittan cikarilamaz.")
        print(f"  Gerekli: std(roll) > {UYARIM_ROLL_DEG:.1f} deg VE "
              f"std(ex*sin(roll)) > {UYARIM_REG:.2f} deg")
        print("  Cozum: manevrali bir kosunun (elips/sonsuz plani) kaydiyla "
              "tekrarlayin.")
        return 2

    gecerli = [r for _, r, _, _ in sonuc if not math.isnan(r)]
    ayirt = max(gecerli) - min(gecerli)
    sifir = [r for m, r, _, _ in sonuc if m == 0 and not math.isnan(r)]
    if sifir:
        print(f"0 ms (duzeltmesiz) : |corr| = {sifir[0]:.3f}")
    print(f"EN IYI GECIKME     : {en_iyi[0]:.0f} ms  ->  |corr| = {en_iyi[1]:.3f}  "
          f"std = {en_iyi[2]:.2f} deg")
    print(f"egri ayirt ediciligi: tepe-dip {ayirt:.3f}")
    if ayirt < AYIRT_EDICILIK:
        print("  *** EGRI DUZ: minimum gurultu icinde, bu sayiya GUVENMEYIN ***")
        print("  (daha uzun / daha manevrali bir kayit gerekir)")
        return 2
    if en_iyi[0] in (bas, bit):
        print("  *** MINIMUM TARAMA UCUNDA: --tara araligini genisletin ***")

    print()
    print(f"Kullanim: bbox_to_redis.py --kamera-gecikme-ms {en_iyi[0]:.0f}")
    print(f"veya     YILDIZ_KAMERA_GECIKME_MS={en_iyi[0]:.0f} ./yildizlar_gudum.sh")
    if ayni_dosya:
        print("(ARTIK gecikme: mevcut ayara EKLENIR, yerine gecmez)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
