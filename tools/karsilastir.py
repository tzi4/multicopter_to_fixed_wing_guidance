#!/usr/bin/env python3
"""karsilastir.py - GORUNTULU gudum metotlarini ayni olcutle yan yana koyar.

NICIN AYRI BIR ARAC: deneme_ozeti.py KONUMLU fazi ozetler ("yaklasti mi,
kadraja girdi mi"). Burada sorulan soru baska: DEVIRDEN SONRA hangi metot
hedefi daha iyi merkezde tutup daha hizli buyuttu? Uc metot (LOS/PID/MPC)
kendi kodunu yazdi, kendi tune'unu yapti; karsilastirma tek elden ve AYNI
tanimlarla yapilmali ki secim koda degil sonuca dayansin.

ODUL TANIMI (kullanicinin koydugu amac, 2026-08-04: LINEER, karekok yok):
  1. BIRINCIL: bbox ALANI (w*h, px^2) surekli buyusun -> collision. Ikinci
     odul alanin BUYUME HIZI (birinci turev ~ yaklasma hizinin vekili).
     CSV'de alan_kok saklanir (gecmis kosularla uyum); burada alan_kok^2
     olarak lineere cevrilir.
  2. Hedef merkezde kalsin: |ex|,|ey| RMS (sanal kadrajda, derece).
  3. IKINCIL: ne kadar hizli. Olcut: devirden en yakin menzile gecen sure.
  Carpisma delili: min menzil + titresim (vibe) sicramasi.

KULLANIM:
    python3 tools/karsilastir.py                     # tum goruntulu koslar
    python3 tools/karsilastir.py --metot pid los     # yalniz belirtilenler
    python3 tools/karsilastir.py --csv rapor.csv     # makine okunur cikti
"""

import argparse
import csv
import math
import re
from pathlib import Path

KOK = Path(__file__).resolve().parent.parent
LOG_DIZIN = KOK / 'guidance_allstar' / 'logs'
# goruntulu_<metot>_<damga>.csv
AD = re.compile(r'goruntulu_([a-z0-9]+)_(\d{8}_\d{6})\.csv$')


def _f(satir, anahtar):
    ham = satir.get(anahtar, '')
    if ham in ('', None):
        return None
    try:
        return float(ham)
    except ValueError:
        return None


def _rms(degerler):
    gecerli = [d for d in degerler if d is not None]
    if not gecerli:
        return None
    return math.sqrt(sum(d * d for d in gecerli) / len(gecerli))


def kosu_analiz(yol):
    """Bir goruntulu CSV'sini oku, odul olcutlerini cikar."""
    with open(yol) as f:
        satirlar = list(csv.DictReader(f))
    if not satirlar:
        return None

    m = AD.search(yol.name)
    metot, damga = (m.group(1), m.group(2)) if m else ('?', '?')

    # Devir anlari: t'de 1 s'den buyuk bosluk = yetki yeniden devralindi.
    # (Dongu yalniz yetkiliyken yazar; bosluk 'konumluya donduk' demektir.)
    devirler = []
    onceki_t = None
    for s in satirlar:
        t = _f(s, 't')
        if t is None:
            continue
        if onceki_t is None or t - onceki_t > 1.0:
            devirler.append(s)
        onceki_t = t

    alan = [_f(s, 'alan_kok') for s in satirlar]
    alan_gecerli = [a for a in alan if a is not None]

    # LINEER alan buyume hizi [px^2/s]: ardisik gecerli tespit ciftlerinden,
    # tespit boslugu 1 s'yi asan ciftler atilir (turev anlamsizlasir).
    buyume = []
    onceki_alan = onceki_alan_t = None
    for s in satirlar:
        a, t = _f(s, 'alan_kok'), _f(s, 't')
        if a is None or t is None:
            continue
        if onceki_alan is not None and 0 < t - onceki_alan_t <= 1.0:
            buyume.append((a * a - onceki_alan * onceki_alan)
                          / (t - onceki_alan_t))
        onceki_alan, onceki_alan_t = a, t
    buyume.sort()
    menzil = [_f(s, 'menzil_m') for s in satirlar]
    menzil_gecerli = [r for r in menzil if r is not None]
    vibe = [_f(s, 'vibe_max') for s in satirlar]
    vibe_gecerli = [v for v in vibe if v is not None]
    # Vibe tepesinin BAGLAMI: sicrama hedefin dibindeyse carpisma delili,
    # hedef uzakta + irtifa ~0 ise YER TEMASIDIR (2026-08-04'te uc kosu
    # boyle yanlis okunuyordu). Tepe aninin menzili ve irtifasi kaydedilir.
    vibe_menzil = vibe_pos_z = None
    if vibe_gecerli:
        tepe = max(vibe_gecerli)
        for s in satirlar:
            if _f(s, 'vibe_max') == tepe:
                vibe_menzil = _f(s, 'menzil_m')
                vibe_pos_z = _f(s, 'pos_z')
                break

    # Tespit VARKEN merkezleme hatasi (bbox yokken hata tanimsiz).
    ex = [_f(s, 'ex_deg') for s in satirlar]
    ey = [_f(s, 'ey_deg') for s in satirlar]

    # Hiz: ilk devirden en yakin menzile gecen sure.
    sure_en_yakin = None
    if menzil_gecerli and devirler:
        t0 = _f(devirler[0], 't')
        en_iyi_t, en_iyi_r = None, None
        for s in satirlar:
            r, t = _f(s, 'menzil_m'), _f(s, 't')
            if r is None or t is None or t < (t0 or 0):
                continue
            if en_iyi_r is None or r < en_iyi_r:
                en_iyi_r, en_iyi_t = r, t
        if en_iyi_t is not None and t0 is not None:
            sure_en_yakin = en_iyi_t - t0

    # Devir sicramasi: devirden sonraki ilk 2 s'de komut buyuklugu adimi.
    sicrama = None
    if devirler:
        t0 = _f(devirler[0], 't')
        buyukluk, onceki = [], None
        for s in satirlar:
            t = _f(s, 't')
            if t is None or t0 is None or not (t0 <= t <= t0 + 2.0):
                continue
            v = [_f(s, k) for k in ('cmd_vx', 'cmd_vy', 'cmd_vz')]
            if any(x is None for x in v):
                continue
            n = math.sqrt(sum(x * x for x in v))
            if onceki is not None:
                buyukluk.append(abs(n - onceki))
            onceki = n
        if buyukluk:
            sicrama = max(buyukluk)

    dt = [_f(s, 'dt') for s in satirlar]
    dt_gecerli = sorted(d for d in dt if d)

    # VURUS HUKMU (2026-08-04): "carptik mi" sorusunun tek bakista cevabi.
    # DIKKAT -- vibe TEK BASINA DELIL DEGIL: Gazebo iki SITL araci arasinda
    # TEMAS MODELLEMIYOR, yani gercek bir carpmada bile titresim sicramiyor
    # (olculdu: 0.92 m'lik gecişte vibe 0.9). Buna karsilik YERE carpmada vibe
    # 150-345'e firliyor. Yani vibe'in isi carpmayi degil YER TEMASINI ayirt
    # etmek. Hukum: menzil esigi + irtifanin yerde OLMADIGININ dogrulanmasi.
    yerde = (vibe_pos_z is not None and vibe_pos_z > -5.0)
    if menzil_gecerli:
        mn = min(menzil_gecerli)
        if yerde and (vibe_max_deg := max(vibe_gecerli) if vibe_gecerli else 0) > 50:
            hukum = 'YER'          # yer temasi -- carpma DEGIL
        elif mn <= 3.0:
            hukum = 'VURUS'
        elif mn <= 8.0:
            hukum = 'YAKIN'
        else:
            hukum = 'ISKA'
    else:
        hukum = '-'

    return {
        'metot': metot,
        'damga': damga,
        'hukum': hukum,
        'dosya': yol.name,
        'ornek': len(satirlar),
        'devir_sayisi': len(devirler),
        'devir_menzili': _f(devirler[0], 'menzil_m') if devirler else None,
        'alan_max_px2': (max(alan_gecerli) ** 2) if alan_gecerli else None,
        'alan_hiz_p90': (buyume[int(len(buyume) * 0.9)] if buyume else None),
        'menzil_min': min(menzil_gecerli) if menzil_gecerli else None,
        'ex_rms': _rms(ex),
        'ey_rms': _rms(ey),
        'ex_tepe': max((abs(x) for x in ex if x is not None), default=None),
        'sure_en_yakin_s': sure_en_yakin,
        'devir_sicramasi': sicrama,
        'vibe_max': max(vibe_gecerli) if vibe_gecerli else None,
        'vibe_menzil_m': vibe_menzil,
        'vibe_pos_z': vibe_pos_z,
        'dongu_hz': (1.0 / dt_gecerli[len(dt_gecerli) // 2]) if dt_gecerli else None,
        'tespit_orani': (sum(1 for a in alan if a is not None) / len(satirlar) * 100),
    }


def _s(deger, bicim='{:.1f}', bos='  -  '):
    return bos if deger is None else bicim.format(deger)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--metot', nargs='*', help='yalniz bu metotlar (los pid mpc)')
    p.add_argument('--csv', help='sonucu bu dosyaya da yaz')
    p.add_argument('--min-ornek', type=int, default=20,
                   help='bu kadar satirdan kisa kosular atlanir (varsayilan 20)')
    a = p.parse_args()

    kosular = []
    for yol in sorted(LOG_DIZIN.glob('goruntulu_*.csv')):
        sonuc = kosu_analiz(yol)
        if sonuc is None or sonuc['ornek'] < a.min_ornek:
            continue
        if a.metot and sonuc['metot'] not in a.metot:
            continue
        kosular.append(sonuc)

    if not kosular:
        print("goruntulu kosu bulunamadi "
              f"({LOG_DIZIN}/goruntulu_*.csv)")
        return

    print("=" * 108)
    print("GORUNTULU GUDUM KARSILASTIRMASI  "
          "(birincil odul: alan_kok buyumesi + merkezleme; ikincil: sure)")
    print("=" * 108)
    basliklar = (f"{'metot':7} {'damga':16} {'hukum':6} {'devir':5} {'devir_m':8} "
                 f"{'min_m':7} {'alan_px2':9} {'a_hiz90':8} "
                 f"{'ex_rms':7} {'ey_rms':7} "
                 f"{'sure_s':7} {'sicrama':8} {'vibe':6} {'Hz':5} {'tespit%':7}")
    print(basliklar)
    print("-" * 124)
    for k in sorted(kosular, key=lambda x: (x['metot'], x['damga'])):
        print(f"{k['metot']:7} {k['damga']:16} {k['hukum']:6} {k['devir_sayisi']:5d} "
              f"{_s(k['devir_menzili'], '{:8.1f}'):8} "
              f"{_s(k['menzil_min'], '{:7.1f}'):7} "
              f"{_s(k['alan_max_px2'], '{:9.0f}'):9} "
              f"{_s(k['alan_hiz_p90'], '{:8.0f}'):8} "
              f"{_s(k['ex_rms'], '{:7.2f}'):7} {_s(k['ey_rms'], '{:7.2f}'):7} "
              f"{_s(k['sure_en_yakin_s'], '{:7.1f}'):7} "
              f"{_s(k['devir_sicramasi'], '{:8.2f}'):8} "
              f"{_s(k['vibe_max'], '{:6.1f}'):6} "
              f"{_s(k['dongu_hz'], '{:5.1f}'):5} "
              f"{k['tespit_orani']:7.1f}")

    print()
    print("SUTUNLAR: devir=yetki devralma sayisi | devir_m=ilk devirdeki menzil")
    print("  hukum: VURUS(<=3 m) | YAKIN(<=8) | ISKA | YER(=yer temasi, carpma DEGIL)")
    print("    DIKKAT: Gazebo iki SITL araci arasinda TEMAS MODELLEMIYOR -- gercek")
    print("    carpmada vibe SICRAMAZ (0.92 m'lik geciste vibe 0.9). Vibe'in isi")
    print("    YER temasini ayirt etmek (orada 150-345'e firliyor).")
    print("  min_m=goruldugu en yakin menzil (eski not: vibe sicramasi;")
    print("        DIKKAT: vibe tepesi hedef uzakta + pos_z~0 iken YER TEMASIDIR,")
    print("        CSV'deki vibe_menzil_m/vibe_pos_z kolonlarindan ayirt edilir)")
    print("  alan_px2=bbox alani tepe [px^2, LINEER] (BIRINCIL ODUL, buyuk=iyi)")
    print("  a_hiz90=alan buyume hizinin p90'i [px^2/s] (IKINCIL ODUL: yaklasma hizi)")
    print("  ex/ey_rms=merkezleme hatasi (kucuk=iyi) | sure_s=devirden en yakina")
    print("  sicrama=devirde komut buyuklugu adimi (kucuk=yumusak gecis)")

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(kosular[0].keys()))
            w.writeheader()
            w.writerows(kosular)
        print(f"\nCSV yazildi: {a.csv}")


if __name__ == '__main__':
    main()
