#!/usr/bin/env python3
"""Bir denemenin guidance.log + bbox.log ciktisini tek sayfada ozetler.

Amac: her denemeden sonra "yaklasti mi, kadraja girdi mi" sorusunun cevabinin
goz karariyla degil SAYIYLA verilmesi.
"""

import math
import os
import re
import sys
from pathlib import Path

ANSI = re.compile(r'\x1b\[[0-9;]*m')
RANGE = re.compile(r'range_target=\s*([0-9.]+)m')
SLOT = re.compile(r'range_slot=\s*([0-9.]+)m')
BBOX = re.compile(r'bbox=\((\d+),(\d+),(\d+),(\d+)\)\s+cov=([0-9.]+)%')
OZET = re.compile(r'kare=(\d+)\s+fps=([0-9.]+)\s+tespit_orani=%([0-9.]+)')


def oku(path):
    if not path.exists():
        return []
    return ANSI.sub('', path.read_text(errors='replace')).splitlines()



def _montaj_oku():
    """DONDURULMUS YOL. Kamera STATIK montaj acisi [deg, yukari +].

    GIMBAL DALI (2026-08-05): kamera fiziksel tilt gimbalinde, model.sdf'teki
    statik montaj artik HEP 0. Bu okuyucu yalnizca --no-tilt (govdeye-sabit)
    kosulari icin, son care olarak duruyor. Canli eksen icin _tilt_oku().
    """
    v = os.environ.get('YILDIZ_MOUNT')
    if v:
        try:
            return float(v)
        except ValueError:
            pass
    sdf = Path(__file__).resolve().parent.parent / "models" / "suru_drone_1" / "model.sdf"
    m = re.search(r'<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+(-?[\d.]+)',
                  sdf.read_text()) if sdf.exists() else None
    return -math.degrees(float(m.group(1))) if m else 0.0


_TILT_DENEME_DIR = None   # main() doldurur: deneme klasoru (bbox.log kaynagi)


def _tilt_oku(kok=None):
    """Kamera ekseni = GIMBAL TILT'i [deg, dunya elevasyonu, yukari +].

    FAZ C (2026-08-06): tilt artik DINAMIK (bbox hedefin yukselisini izler),
    o yuzden ONCE kosunun OLCULEN ortancasi (gudum CSV'sindeki tilt_deg /
    goruntulu CSV'deki tilt_status_deg) denenir; env degerleri yalniz
    baslangic/yeniden-edinim setpoint'idir ve yedege dustu:
      olculen ortanca -> $YILDIZ_TILT -> atan($YILDIZ_DOWN/$YILDIZ_BACK).
    """
    olculen = _tilt_olculen_ortanca(kok, deneme_dir=_TILT_DENEME_DIR) \
        if kok is not None else None
    if olculen is not None:
        return olculen, 'olculen ortanca (dinamik tilt)'
    v = os.environ.get('YILDIZ_TILT')
    if v:
        try:
            return float(v), 'YILDIZ_TILT'
        except ValueError:
            pass
    d, b = os.environ.get('YILDIZ_DOWN'), os.environ.get('YILDIZ_BACK')
    if d and b:
        try:
            return math.degrees(math.atan2(float(d), max(float(b), 1e-6))), \
                'atan(down/back)'
        except ValueError:
            pass
    return None, None


def _tilt_olculen_ortanca(kok, deneme_dir=None):
    """Kosunun OLCULEN tilt ortancasi [deg] ya da None.

    Kaynak sirasi:
      1. deneme_dir/bbox.log OZET satirlarindaki 'tilt=+cmd/+statusdeg'
         (her kosuda VAR; Faz C'de status her karede degisir)
      2. goruntulu_*.csv 'tilt_deg' kolonu (gudum kosarken)
      3. gimbal-log CSV'lerindeki 'tilt_status_deg'
    """
    import csv
    import glob
    import re
    if deneme_dir is not None:
        try:
            m = re.findall(r'tilt=[+-][\d.]+/([+-][\d.]+)deg',
                           (Path(deneme_dir) / 'bbox.log').read_text(
                               errors='replace'))
            vals = sorted(float(x) for x in m)
            if vals:
                return vals[len(vals) // 2]
        except OSError:
            pass
    for desen, kolon in (('goruntulu_*.csv', 'tilt_deg'),
                         ('*.csv', 'tilt_status_deg')):
        for yol in sorted(glob.glob(str(kok / desen)),
                          key=os.path.getmtime, reverse=True):
            try:
                vals = []
                for r in csv.DictReader(open(yol)):
                    try:
                        vals.append(float(r[kolon]))
                    except (TypeError, ValueError, KeyError):
                        continue
                if vals:
                    vals.sort()
                    return vals[len(vals) // 2]
            except OSError:
                continue
    return None


def gimbal_analizi():
    """En son gudum CSV'sinden GEOMETRI raporu.

    Hedefin kadrajda kalip kalmadigini belirleyen tek karsilastirma:
        kamera ekseni  = GIMBAL TILT'i (dunya elevasyonu, + = yukari)
        hedefin yeri   = avciya gore yukselis acisi
    Ikisinin FARKI kadrajin dikey yari-acisini (720 px / 40.13 deg -> +-20.07)
    asarsa hedef fiziksel olarak kadrajin disindadir.

    GIMBAL DALI (2026-08-05): eksen ARTIK "montaj + govde pitch" DEGIL. Kamera
    kendini stabilize eden fiziksel tek eksen tilt gimbalinde; govde pitch'i
    +-35 deg savrulurken kamera dunya pitch'i max 0.65 deg olculdu, yani pitch
    esitlige girmiyor. Eski montaj+pitch yolu yalnizca --no-tilt (dondurulmus
    govdeye-sabit) kosulari icin, tilt bilinmiyorsa yedek olarak calisir.
    """
    import csv
    import glob
    import math
    import os
    kok = Path(__file__).resolve().parent.parent / 'guidance_allstar' / 'logs'
    # KONUMLU logu ADIYLA sec. Eskiden 'en yeni *.csv' aliniyordu; logs/
    # altinda artik goruntulu_*/mpc_tani_*/..._olay.csv de var ve bunlarda
    # range_m/meas_z/pitch_deg ucusu birlikte bulunmadigi icin blok SESSIZCE
    # bos donuyordu (2026-08-05).
    dosyalar = sorted(glob.glob(str(kok / 'guided_follow_*.csv')),
                      key=os.path.getmtime)
    if not dosyalar:
        return
    rows = list(csv.DictReader(open(dosyalar[-1])))

    def fl(r, k):
        try:
            return float(r[k])
        except (TypeError, ValueError, KeyError):
            return None

    yakin = [r for r in rows if (fl(r, 'range_m') or 1e9) < 60]
    if not yakin:
        return
    # SABIT YAZILMAZ (2026-08-04): eksen kosudan kosuya degisir, sabit yazmak
    # "kamera ekseni +30, hedef-eksen -17.4" gibi YANLIS geometri basiyordu.
    # GIMBAL DALI (2026-08-05): eksenin tek kaynagi artik TILT.
    TILT_DEG, TILT_KAYNAK = _tilt_oku(kok)
    MONTAJ_DEG = _montaj_oku()
    YARI_FOV_DEG = 20.07       # AI Camera 720p: 40.13 deg dikey FOV / 2

    pitch = sorted(x for r in yakin if (x := fl(r, 'pitch_deg')) is not None)
    yuk = []
    for r in yakin:
        rng, pz, mz = fl(r, 'range_m'), fl(r, 'pursuer_z'), fl(r, 'meas_z')
        if None in (rng, pz, mz) or rng < 3:
            continue
        dz = mz - pz                                   # NED: z asagi pozitif
        yatay = math.sqrt(max(1e-6, rng * rng - dz * dz))
        yuk.append(math.degrees(math.atan2(-dz, yatay)))
    yuk.sort()
    if not (pitch and yuk):
        return

    def p(v, q):
        return v[min(len(v) - 1, int(q * len(v)))]

    print()
    print("--- GEOMETRI (menzil < 60 m, gudum CSV'sinden) ---")
    print(f"  govde pitch (deg)   : %5 {p(pitch,.05):+.1f} ortanca "
          f"{p(pitch,.5):+.1f} %95 {p(pitch,.95):+.1f}")
    print(f"  hedefin yukselisi   : %5 {p(yuk,.05):+.1f} ortanca "
          f"{p(yuk,.5):+.1f} %95 {p(yuk,.95):+.1f}")
    if TILT_DEG is not None:
        eksen = TILT_DEG
        eksen_metni = f"{eksen:+.1f} deg  (gimbal tilt, kaynak: {TILT_KAYNAK})"
    else:
        # Tilt bilinmiyor: DONDURULMUS govdeye-sabit yol (--no-tilt kosulari).
        eksen = MONTAJ_DEG + p(pitch, .5)
        eksen_metni = (f"{MONTAJ_DEG:+.0f} (montaj) {p(pitch,.5):+.1f} (pitch) "
                       f"= {eksen:+.1f} deg  [TILT BILINMIYOR -> eski "
                       f"govdeye-sabit varsayim; YILDIZ_TILT verin]")
    fark = p(yuk, .5) - eksen
    print(f"  kamera ekseni       : {eksen_metni}")
    print(f"  hedef - eksen       : {fark:+.1f} deg  "
          f"(kadraj yari-acisi +-{YARI_FOV_DEG:.2f} deg)")
    if abs(fark) > YARI_FOV_DEG:
        print(f"  >>> hedef ORTALAMADA kadrajin disinda. Dikey geometriyi duzelt:")
        print(f"      (a) TILT'i {p(yuk,.5):+.1f} dereceye cek (hedefin ortanca "
              f"yukselisi); tilt = atan(down/back) oldugu icin bu, standoff "
              f"ikilisini degistirmek demektir")
        print(f"      (b) mevcut tilt {eksen:+.1f} deg korunacaksa standoff'u ona "
              f"uydur: back=B icin down = {math.tan(math.radians(eksen)):.2f}*B")
        print(f"      (kaynak: scripts/standoff_geom.sh -- YILDIZ_DOWN/BACK/TILT)")
    # Pitch salinimi: ESKI govdeye-sabit kamerada dogrudan kadraj salinimiydi.
    # Gimbal dalinda kadraji BOZMAZ (kamera dunya pitch'i max 0.65 deg olculdu),
    # yalnizca aracin ne kadar savruldugunu anlatir -- teshis icin duruyor.
    salinim = p(pitch, .95) - p(pitch, .05)
    not_ = ("(gimballi: kadraja yansimaz, yalniz arac tutumu)"
            if TILT_DEG is not None else "(govdeye-sabit: kadraj salinimi)")
    print(f"  pitch salinimi (5-95): {salinim:.1f} deg "
          f"{'>> ' if salinim > 2 * YARI_FOV_DEG else '<= '}"
          f"kadraj yuksekligi {2 * YARI_FOV_DEG:.1f} deg {not_}")


def main():
    if len(sys.argv) != 2:
        raise SystemExit("kullanim: deneme_ozeti.py <deneme_klasoru>")
    d = Path(sys.argv[1])
    global _TILT_DENEME_DIR
    _TILT_DENEME_DIR = d

    g = oku(d / 'guidance.log')
    menziller = [float(m.group(1)) for line in g for m in [RANGE.search(line)] if m]
    slotlar = [float(m.group(1)) for line in g for m in [SLOT.search(line)] if m]
    olaylar = [l.strip() for l in g
               if any(k in l for k in ('MISSION FAILSAFE', 'RECOVERY', 'crash',
                                       'KILL MODE', 'abort', 'ABORT'))]

    b = oku(d / 'bbox.log')
    kutular = [(int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)),
                float(m.group(5))) for line in b for m in [BBOX.search(line)] if m]
    ozetler = [(int(m.group(1)), float(m.group(2)), float(m.group(3)))
               for line in b for m in [OZET.search(line)] if m]

    print(f"=== DENEME: {d.name} ===")
    print()
    print("--- KONUMLU GUDUM ---")
    if menziller:
        print(f"  dongu ornegi        : {len(menziller)}")
        print(f"  hedefe mesafe       : basta {menziller[0]:.0f} m -> "
              f"sonda {menziller[-1]:.0f} m  (EN YAKIN {min(menziller):.0f} m)")
        if slotlar:
            print(f"  standoff noktasina  : EN YAKIN {min(slotlar):.0f} m")
        # Mesafenin hangi bantlarda ne kadar zaman gectigi: "yaklasti mi"
        # sorusunun tek sayilik cevabi yaniltici olabiliyor (bir kez yaklasip
        # sonra acilmak ile bantta kalmak ayni "en yakin" degerini verir).
        bantlar = [(0, 50), (50, 100), (100, 200), (200, 500), (500, 10**9)]
        print("  mesafe bandi dagilimi:")
        for lo, hi in bantlar:
            n = sum(1 for r in menziller if lo <= r < hi)
            if n:
                ust = '+' if hi > 10**8 else str(hi)
                print(f"      {lo:4d}-{ust:>5s} m : {n:5d} ornek (%{100*n/len(menziller):.1f})")
    else:
        print("  (gudum dongusu hic ornek uretmedi)")
    if olaylar:
        print("  olaylar:")
        for o in dict.fromkeys(olaylar):
            print(f"      {o}")

    print()
    print("--- KAMERA / BBOX ---")
    if ozetler:
        son_kare, _, son_oran = ozetler[-1]
        fps = sum(o[1] for o in ozetler) / len(ozetler)
        print(f"  islenen kare        : {son_kare} (ort {fps:.1f} fps)")
        print(f"  kumulatif tespit    : %{son_oran:.1f}")
    if kutular:
        genislikler = sorted(k[2] for k in kutular)
        kapsamalar = sorted(k[4] for k in kutular)
        n = len(kutular)
        print(f"  TESPIT SAYISI       : {n}")
        print(f"  bbox genisligi (px) : min {genislikler[0]} "
              f"ortanca {genislikler[n // 2]} max {genislikler[-1]}")
        print(f"  yatay kapsama (%)   : min {kapsamalar[0]:.2f} "
              f"ortanca {kapsamalar[n // 2]:.2f} max {kapsamalar[-1]:.2f}")
        # Kadrajdaki yer: yaw kilidi hedefi ortada tutabiliyor mu?
        merkezler_x = sorted(k[0] + k[2] / 2 for k in kutular)
        merkezler_y = sorted(k[1] + k[3] / 2 for k in kutular)
        print(f"  kadraj merkezi x    : ortanca {merkezler_x[n // 2]:.0f} "
              f"(kadraj ortasi 640)")
        print(f"  kadraj merkezi y    : ortanca {merkezler_y[n // 2]:.0f} "
              f"(kadraj ortasi 360)")
    else:
        print("  TESPIT SAYISI       : 0  (hedef hic kadraja girmedi)")

    gimbal_analizi()


if __name__ == '__main__':
    main()
