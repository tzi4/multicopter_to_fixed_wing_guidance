#!/usr/bin/env python3
"""montaj_ayarla.py - TARIHSEL: gimbal dalinda tools/tilt_ayarla.py kullan.

*** TARIHSEL ARAC -- YAZMA YOLU KAPALI (gimbal dali, 2026-08-05) ***
Bu arac kameranin GOVDEYE SABIT oldugu dunyaya aittir. Gimbal dalinda kamera
kendini stabilize eden tek eksen (tilt) gimbalde: montaj acisi diye bir ayar
dugmesi KALMADI, kamera acisi standoff geometrisinden turetilir
(YILDIZ_TILT = atan(DOWN/BACK)) ve calisma aninda komutlanir.

Ozellikle TEHLIKELI olan: bu aracin SDF 'cam' sensor pozuna yazmasi. Kamera
artik gimbal tilt link'ine kaynakli; oraya bir aci yazmak gimbalin komutladigi
dunya elevasyonunun USTUNE SESSIZ bir ofset bindirir (gimbal kendini eps'e
stabilize eder, gercek eksen eps+ofset olur, olcum zinciri farki goremez).
Bu yuzden --uygula (ve --geri-al) yollari DEVRE DISI; --goster calisir.

  YENI ARAC:  python3 tools/tilt_ayarla.py --goster
              python3 tools/tilt_ayarla.py --down 6 --back 25 [--uygula]

Asagisi eski dunyanin belgesidir (govdeye sabit kamera kollari icin gecerli):

NICIN ARAC GEREKIYORDU: montaj acisi tek bir ayar dugmesi DEGIL. Uc yerde birden
gecer ve biri otekinden ayrisirsa sessizce yanlis calisir:

  1. FIZIKSEL KAYNAK  models/suru_drone_*/model.sdf, "cam" sensorunun <pose>
     icindeki pitch (radyan, YUKARI bakis NEGATIF: -0.5235988 = +30 deg).
     Simulasyonun gercek kamera acisi budur.
  2. SANAL GIMBAL     yildizlar_gimbal.SanalGimbal(mount_phys_pitch_deg=...)
     -- yildizlar_gudum.sh bunu YILDIZ_MOUNT ile gecirir. SDF ile BIREBIR
     ayni olmali; ayrisirsa de-rotasyon (govde salinimi temizleme) yanlis olur.
  3. STANDOFF         scripts/standoff_geom.sh: DOWN = BACK*tan(MOUNT+TRIM).
     Hedefin kamera EKSENINDE durmasini bu saglar (down=3 denenen kosuda
     hedef eksenden 22 deg kacip tespit %5'e dusmustu).

TEMEL KURAL (2026-08-04, IRL suruklenme analizi + statik hedef testi):
    kamera ekseni = montaj + govde pitch
    montaj, CARPMA ANINDAKI hedef LOS yukselisine esitlenmeli.
  * Gercek donanimda govde terminal dash'te dik burun-asagi (18 m/s'de ~-34
    deg; tan(-pitch)=0.00212*V_hava^2) -> arkadan es-irtifa carpmada
    montaj = terminal pitch buyuklugu.
  * SIMULASYONDA kopterin suruklemesi yok denecek kadar az (20-25 m/s'de
    olculen pitch -1.1 deg) -> sim'de montaj ~= standoff LOS yukselisi.
  Bu yuzden sim montaji gercek donanim montajini DOGRULAMAZ; ikisi ayri karar.

KULLANIM:
    python3 tools/montaj_ayarla.py --goster                 # su anki durum
    python3 tools/montaj_ayarla.py --mount 14 --back 40     # DOWN turetilir
    python3 tools/montaj_ayarla.py --mount 14 --back 40 --uygula
    python3 tools/montaj_ayarla.py --geri-al                # git checkout ile

--uygula verilmezse HICBIR SEY YAZILMAZ, yalnizca ne olacagi gosterilir.
Uygulandiktan sonra yildizlar_gimbal statik testi otomatik kosulur.
"""

import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

KOK = Path(__file__).resolve().parent.parent
SDF_DESEN = 'models/suru_drone_*/model.sdf'
GIMBAL = KOK / 'yildizlar_gimbal.py'
STANDOFF = KOK / 'scripts/standoff_geom.sh'
# DIKKAT: ayni <pose> desenini 'cam_link' GORSELI de tasiyor (1.5707 = silindir
# govdesini yatirmak icin, kamerayla ilgisi YOK). Bu yuzden desen <sensor
# name="cam"> etiketine DEMIRLENIR; yalniz onu izleyen ilk pose degistirilir.
# (Bu hatayi aracin kuru kosusu yakaladi: ilk surum -89.99 deg okuyordu.)
POSE = re.compile(r'(<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+)'
                  r'(-?[\d.]+)(\s+[-\d.]+\s*</pose>)')


def sdf_dosyalari():
    return sorted(KOK.glob(SDF_DESEN))


def sdf_oku(yol):
    m = POSE.search(yol.read_text())
    if not m:
        return None
    return -math.degrees(float(m.group(2)))       # pose NEGATIF = yukari bakis


def mevcut_durum():
    d = {}
    for yol in sdf_dosyalari():
        d[yol.relative_to(KOK)] = sdf_oku(yol)
    m = re.search(r'mount_phys_pitch_deg=([\d.]+)', GIMBAL.read_text())
    d['yildizlar_gimbal.py (varsayilan)'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_MOUNT="\$\{YILDIZ_MOUNT:-([\d.]+)\}"', STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_MOUNT'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_BACK="\$\{YILDIZ_BACK:-([\d.]+)\}"', STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_BACK'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_PITCH_TRIM="\$\{YILDIZ_PITCH_TRIM:-(-?[\d.]+)\}"',
                  STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_PITCH_TRIM'] = float(m.group(1)) if m else None
    return d


def yaz(mount, back, trim, down=None):
    """down verilirse standoff_geom.sh'e KALICI yazilir.

    HATA (2026-08-04, gelistirici ajani yakaladi): eski surum --down'u yalniz
    ekrana basiyor, dosyaya yazmiyordu. Sonuc: montaj 0'a cekilince
    standoff_geom.sh turetimi DOWN=-1 veriyor ve DOWN= env'i verilmeyen her
    kosuda kopter hedefin USTUNE konumlaniyordu.
    """
    if down is None:
        down = round(back * math.tan(math.radians(mount + trim)))
    rad = -math.radians(mount)
    for yol in sdf_dosyalari():
        metin = yol.read_text()
        yeni, n = POSE.subn(lambda m: f"{m.group(1)}{rad:.7f}{m.group(3)}", metin)
        if n != 1:
            raise SystemExit(f"HATA: {yol} icinde 'cam' pose satiri {n} kez eslesti")
        yol.write_text(yeni)
    g = GIMBAL.read_text()
    GIMBAL.write_text(re.sub(r'mount_phys_pitch_deg=[\d.]+',
                             f'mount_phys_pitch_deg={float(mount)}', g, count=1))
    s = STANDOFF.read_text()
    s = re.sub(r'(YILDIZ_MOUNT="\$\{YILDIZ_MOUNT:-)[\d.]+(\}")',
               rf'\g<1>{mount:g}\g<2>', s, count=1)
    s = re.sub(r'(YILDIZ_BACK="\$\{YILDIZ_BACK:-)[\d.]+(\}")',
               rf'\g<1>{back:g}\g<2>', s, count=1)
    # Tasarim DOWN'u turetim-basarisiz dalina KALICI yaz (yukaridaki docstring).
    s = re.sub(r"(YILDIZ_DOWN_TASARIM=)[\d.]+", rf"\g<1>{down:g}", s, count=1)
    STANDOFF.write_text(s)
    return down


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--goster', action='store_true', help='su anki durumu yaz, cik')
    p.add_argument('--mount', type=float, help='yeni montaj acisi [deg, YUKARI +]')
    p.add_argument('--back', type=float, help='standoff geri mesafesi [m]')
    p.add_argument('--trim', type=float, default=None,
                   help='tipik govde pitch [deg]; verilmezse mevcut korunur')
    p.add_argument('--down', type=float, default=None,
                   help='standoff dikey ofseti [m] -- ELLE ver. GIMBALLI kurulumda '
                        'zorunlu: gimbal kamera acisini govdeden ayirdigi icin '
                        '"down = back*tan(mount+trim)" turetimi GECERSIZDIR '
                        '(mount=0 iken turetim negatif down verir, yani kopteri '
                        'hedefin USTUNE koyar).')
    p.add_argument('--uygula', action='store_true', help='GERCEKTEN yaz')
    p.add_argument('--geri-al', action='store_true',
                   help='git checkout ile tum montaj dosyalarini geri al')
    a = p.parse_args()

    # TARIHSEL KILIT (gimbal dali): yazma yolu tamamen kapali. Modul
    # docstring'indeki gerekce: SDF pozuna yazmak stabilize gimbalin uzerine
    # sessiz bir ofset bindirir.
    if a.uygula or a.geri_al:
        print(__doc__.split('Asagisi eski dunyanin')[0].strip())
        print("\n*** ISLEM YAPILMADI. tools/tilt_ayarla.py kullanin. ***")
        return

    if a.geri_al:
        hedefler = [str(y.relative_to(KOK)) for y in sdf_dosyalari()]
        hedefler += ['yildizlar_gimbal.py', 'scripts/standoff_geom.sh']
        subprocess.run(['git', 'checkout', '--'] + hedefler, cwd=KOK, check=True)
        print("geri alindi:", ', '.join(hedefler))
        return

    durum = mevcut_durum()
    print("--- MEVCUT DURUM ---")
    for k, v in durum.items():
        print(f"  {str(k):42s} {v}")
    sdf_degerleri = {v for k, v in durum.items() if str(k).startswith('models/')}
    if len(sdf_degerleri) > 1:
        print("  *** UYARI: SDF dosyalari AYRISMIS ***")
    if a.goster or a.mount is None:
        if a.mount is None and not a.goster:
            print("\n--mount verilmedi; yalniz durum gosterildi.")
        return

    back = a.back if a.back is not None else durum['standoff_geom.sh YILDIZ_BACK']
    trim = a.trim if a.trim is not None else durum['standoff_geom.sh YILDIZ_PITCH_TRIM']
    if a.down is not None:
        down = a.down
        turetim = "ELLE (gimballi kurulum: turetim gecersiz)"
    else:
        down = round(back * math.tan(math.radians(a.mount + trim)))
        turetim = "turetildi: back*tan(mount+trim)"
        if down <= 0:
            print(f"\n*** UYARI: turetilen down={down} <= 0, yani kopter hedefin "
                  f"USTUNDE ucar. Gimballi kurulumda --down'u ELLE verin. ***")
    los = math.degrees(math.atan(down / back)) if back else 0.0
    vh = math.degrees(2 * math.atan(math.tan(math.radians(66) / 2) * 720 / 1280)) / 2

    print("\n--- YENI GEOMETRI ---")
    print(f"  montaj                {a.mount:+.2f} deg (SDF pose {-math.radians(a.mount):+.7f} rad)")
    print(f"  back / down           {back:g} m / {down:g} m  (trim {trim:+g}) [{turetim}]")
    print(f"  standoff LOS yukselis {los:+.2f} deg")
    print(f"  dikey yari-FOV        {vh:.2f} deg")
    print(f"  standoff'ta hedefin eksenden sapmasi  {los - (a.mount + trim):+.2f} deg")
    print(f"  ARKADAN CARPMADA (LOS->0) sapma       {-(a.mount + trim):+.2f} deg  "
          f"-> {'KADRAJ DISI' if abs(a.mount + trim) > vh else 'kadraj icinde'}")

    if not a.uygula:
        print("\n(kuru kosu -- yazmak icin --uygula ekleyin)")
        return

    down = yaz(a.mount, back, trim, a.down)
    print(f"\nYAZILDI. Dogrulama kosuluyor...")
    r = subprocess.run([sys.executable, str(GIMBAL), '--test'],
                       cwd=KOK, capture_output=True, text=True)
    print(r.stdout.strip().splitlines()[-1] if r.stdout else r.stderr[-400:])
    if r.returncode != 0:
        print("*** STATIK TEST BASARISIZ -- 'python3 tools/montaj_ayarla.py "
              "--geri-al' ile geri alin ***")
        raise SystemExit(1)
    print(f"\nSonraki adim: A/B kosusu\n"
          f"  SURE=360 GORUNTULU=\"mpc_gudum.py\" PLAN=missions/hedef_elips.plan "
          f"tools/senaryo.sh\n"
          f"Kriter: statik test GECTI (yukarida) + tespit orani korunmali + "
          f"<15 m bandinda tespit %0'dan yukselmeli.")


if __name__ == '__main__':
    main()
