#!/usr/bin/env python3
"""tilt_ayarla.py - GIMBAL DALI: standoff gorev geometrisini (down/back) ayarlar.

NICIN YENI ARAC (eski tools/montaj_ayarla.py'nin yerine):
Kamera artik govdeye SABIT DEGIL; kendini stabilize eden tek eksen (tilt)
gimbalde. Bagimlilik yonu TERSINE DONDU:

  ESKI DUNYA:  montaj acisi (SDF pose) sabit -> standoff DOWN ona uymak zorunda
               (down = back*tan(mount+trim))
  YENI DUNYA:  DOWN/BACK serbest GOREV TASARIMI dugmesi -> kamera acisi ondan
               turetilir:   YILDIZ_TILT = atan(DOWN / BACK)   [dunya elevasyonu]

Bu yuzden bu arac YALNIZ TEK DOSYAYA yazar: scripts/standoff_geom.sh.

*** SDF SENSOR POZUNA HICBIR ARAC YAZMAZ ***
models/suru_drone_*/model.sdf icindeki "cam" sensorunun pose pitch'i 0 KALMALI.
Kamera artik gimbal tilt link'ine kaynakli; oraya bir aci yazmak gimbalin
komutladigi dunya elevasyonunun USTUNE SESSIZ bir ofset bindirir (gimbal
kendini eps'e stabilize eder, gercek eksen eps+ofset olur; olcum zinciri
farki goremez). Eski aracin SDF ve yildizlar_gimbal yazma yollari bu yuzden
SILINDI; bu arac onlari yalnizca DOGRULAR (0 degilse UYARI verir).

KULLANIM:
    python3 tools/tilt_ayarla.py --goster                  # mevcut durum + rapor
    python3 tools/tilt_ayarla.py --down 6 --back 25        # kuru kosu (YAZMAZ)
    python3 tools/tilt_ayarla.py --down 6 --back 25 --uygula
    python3 tools/tilt_ayarla.py --geri-al                 # yedekten geri yukle

--uygula verilmezse HICBIR SEY YAZILMAZ.
"""

import argparse
import math
import os
import re
import subprocess
import sys
from pathlib import Path

KOK = Path(__file__).resolve().parent.parent
SDF_DESEN = 'models/suru_drone_*/model.sdf'
STANDOFF = KOK / 'scripts/standoff_geom.sh'
YEDEK = KOK / 'scripts/standoff_geom.sh.yedek'

# Ayni <pose> desenini 'cam_link' GORSELI de tasir (1.5707 = silindiri yatirmak
# icin, kamerayla ilgisi YOK). Bu yuzden desen <sensor name="cam"> etiketine
# DEMIRLENIR. (Eski aractan aynen korundu; burada YALNIZ OKUMA icin.)
POSE = re.compile(r'(<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+)'
                  r'(-?[\d.]+)(\s+[-\d.]+\s*</pose>)')

# Kamera: 1280x720, hfov 66 deg (IMX500). Dikey YARI-FOV:
HFOV_RAD = 1.1519
GEN, YUK = 1280.0, 720.0
YARI_VFOV = math.degrees(math.atan(math.tan(HFOV_RAD / 2.0) * YUK / GEN))


def sdf_dosyalari():
    return sorted(KOK.glob(SDF_DESEN))


def sdf_oku(yol):
    """SDF 'cam' sensorunun pose pitch'i [deg, YUKARI +]. Bulunamazsa None."""
    m = POSE.search(yol.read_text())
    if not m:
        return None
    return -math.degrees(float(m.group(2)))       # pose NEGATIF = yukari bakis


def standoff_degerleri():
    """standoff_geom.sh'i KAYNAKLAYIP turetilmis degerleri okur.

    Dosyadaki varsayilanlari regex ile ayiklamak yerine kabugu kosturuyoruz:
    YILDIZ_TILT turetimi (ve eski-turetim dali) orada yasiyor, tek kaynak o.
    Disaridan gelen YILDIZ_* ezmeleri temizlenir, yoksa 'mevcut durum'
    dosyanin degil kabugun durumu olur.
    """
    env = {k: v for k, v in os.environ.items() if not k.startswith('YILDIZ_')}
    r = subprocess.run(
        ['bash', '-c',
         f'source "{STANDOFF}"; '
         'echo "$YILDIZ_BACK|$YILDIZ_DOWN|$YILDIZ_TILT|$YILDIZ_MOUNT|$YILDIZ_PITCH_TRIM"'],
        cwd=KOK, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise SystemExit(f"standoff_geom.sh kaynaklanamadi:\n{r.stderr}")
    satir = [s for s in r.stdout.strip().splitlines() if '|' in s][-1]
    back, down, tilt, mount, trim = satir.split('|')
    m = re.search(r'YILDIZ_DOWN_TASARIM=([\d.]+)', STANDOFF.read_text())
    return {'back': float(back), 'down': float(down), 'tilt': float(tilt),
            'mount': float(mount), 'trim': float(trim),
            'down_tasarim': float(m.group(1)) if m else None}


def durum_yaz():
    d = standoff_degerleri()
    print("--- MEVCUT DURUM (scripts/standoff_geom.sh) ---")
    print(f"  YILDIZ_BACK               {d['back']:g} m")
    print(f"  YILDIZ_DOWN_TASARIM       {d['down_tasarim']:g} m")
    print(f"  YILDIZ_DOWN (etkin)       {d['down']:g} m")
    print(f"  YILDIZ_TILT (turetilen)   {d['tilt']:+.2f} deg   "
          f"= atan(down/back)")
    print(f"  YILDIZ_MOUNT / PITCH_TRIM {d['mount']:g} / {d['trim']:g} deg   "
          f"(gimbal dalinda dikey kanalda ISLEVSIZ)")
    print()
    print("--- SDF SENSOR POZU (yazilmamali, 0 olmali) ---")
    kirli = []
    for yol in sdf_dosyalari():
        v = sdf_oku(yol)
        ad = str(yol.relative_to(KOK))
        if v is None:
            print(f"  {ad:34s} 'cam' sensoru bulunamadi (?)")
            kirli.append(ad)
        else:
            print(f"  {ad:34s} {v:+.4f} deg" + ('' if abs(v) < 1e-3 else '   <<< SIFIR DEGIL'))
            if abs(v) >= 1e-3:
                kirli.append(ad)
    if kirli:
        print()
        print("  *** UYARI: SDF 'cam' pozu sifir degil. Kamera gimbal tilt")
        print("  *** link'ine kaynakli; buradaki aci gimbalin dunya elevasyonu")
        print("  *** komutunun USTUNE sessiz bir ofset bindirir. Sifirlayin:")
        print("  ***   git checkout -- " + ' '.join(kirli))
    return d, kirli


def rapor(down, back):
    """Terminal yaklasmada dikey kadraj butcesi.

    Standoff'ta kopter hedefin BACK kadar arkasinda, DOWN kadar altindadir.
    Yaklasirken yatay ayrim kapanir ama DOWN korunur -> hedefin gorus acisi
    yukselisi eps(r) = asin(DOWN / r) HIZLA BUYUR. Kamera tilt'i standoff
    tasarim degerinde (atan(DOWN/BACK)) DONDURULURSA hedef bir menzilden sonra
    kadrajin ustunden tasar. Faz C'nin (terminal tilt komutu) devralmasi
    gereken menzil budur.
    """
    tilt = math.degrees(math.atan2(down, max(back, 1e-9)))
    print(f"--- GEOMETRI (down {down:g} m / back {back:g} m) ---")
    print(f"  YILDIZ_TILT = atan(down/back)   {tilt:+.2f} deg")
    print(f"  dikey YARI-FOV                  {YARI_VFOV:.2f} deg "
          f"(hfov {math.degrees(HFOV_RAD):.0f} deg, {GEN:.0f}x{YUK:.0f})")
    print()
    print("--- TERMINAL KADRAJ BUTCESI ---")
    s = math.sin(math.radians(YARI_VFOV))
    r_eksen = down / s if s > 0 else float('inf')
    print(f"  eps(r) = asin(down/r)  --  eps > {YARI_VFOV:.2f} deg olan menzil: "
          f"r < {r_eksen:.1f} m")
    print(f"    (tilt 0'da dondurulursa kadrajdan cikis menzili)")
    s2 = math.sin(math.radians(min(89.9, tilt + YARI_VFOV)))
    r_tilt = down / s2 if s2 > 0 else float('inf')
    print(f"  tilt {tilt:+.2f} deg'de dondurulursa (eps - tilt > {YARI_VFOV:.2f}): "
          f"r < {r_tilt:.1f} m")
    print(f"  ==> FAZ C (terminal tilt komutu) EN GEC {r_tilt:.1f} m'de devralmali;")
    print(f"      guvenli pay icin ~{r_tilt * 1.5:.0f} m onerilir.")
    print()
    print("  menzil    eps=asin(down/r)   tilt-eps    kadraj (tilt donuk)")
    for r in range(10, 101, 10):
        if r <= down:
            print(f"  {r:5d} m   {'--':>10s}         {'--':>8s}    hedef TAM ALTTA")
            continue
        eps = math.degrees(math.asin(min(1.0, down / r)))
        fark = tilt - eps
        durum = 'icinde' if abs(fark) <= YARI_VFOV else 'DISINDA'
        print(f"  {r:5d} m   {eps:10.2f} deg   {fark:+8.2f}    {durum}")
    print()
    print("  NOT: 'tilt-eps' hedefin optik eksenden dikey sapmasidir; |fark| >")
    print(f"  {YARI_VFOV:.2f} deg olunca hedef FIZIKSEL olarak kadraj disidir "
          f"(yazilim gimbali kurtaramaz).")


def yaz(down, back):
    """YALNIZ scripts/standoff_geom.sh. SDF ve yildizlar_gimbal'a DOKUNULMAZ."""
    metin = STANDOFF.read_text()
    YEDEK.write_text(metin)
    yeni, n1 = re.subn(r'(YILDIZ_DOWN_TASARIM=)[\d.]+', rf'\g<1>{down:g}',
                       metin, count=1)
    if n1 != 1:
        raise SystemExit("HATA: YILDIZ_DOWN_TASARIM satiri bulunamadi")
    yeni, n2 = re.subn(r'(YILDIZ_BACK="\$\{YILDIZ_BACK:-)[\d.]+(\}")',
                       rf'\g<1>{back:g}\g<2>', yeni, count=1)
    if n2 != 1:
        raise SystemExit("HATA: YILDIZ_BACK varsayilan satiri bulunamadi")
    STANDOFF.write_text(yeni)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--goster', action='store_true',
                   help='mevcut durumu + raporu yaz, cik')
    p.add_argument('--down', type=float, help='standoff dikey ofseti [m]')
    p.add_argument('--back', type=float, help='standoff geri mesafesi [m]')
    p.add_argument('--uygula', action='store_true',
                   help='GERCEKTEN yaz (yalniz scripts/standoff_geom.sh)')
    p.add_argument('--geri-al', action='store_true',
                   help='yedekten (yoksa git checkout ile) geri yukle')
    a = p.parse_args()

    if a.geri_al:
        if YEDEK.exists():
            STANDOFF.write_text(YEDEK.read_text())
            YEDEK.unlink()
            print(f"geri alindi (yedek): {STANDOFF.relative_to(KOK)}")
        else:
            subprocess.run(['git', 'checkout', '--', 'scripts/standoff_geom.sh'],
                           cwd=KOK, check=True)
            print("geri alindi (git checkout): scripts/standoff_geom.sh")
        return 0

    d, kirli = durum_yaz()
    print()

    if a.down is None and a.back is None:
        rapor(d['down'], d['back'])
        if not a.goster:
            print("\n(--down/--back verilmedi; yalniz durum gosterildi)")
        return 1 if kirli else 0

    down = a.down if a.down is not None else d['down']
    back = a.back if a.back is not None else d['back']
    if down <= 0 or back <= 0:
        raise SystemExit("HATA: down ve back POZITIF olmali (kopter hedefin "
                         "ARKASINDA ve ALTINDA durur)")
    rapor(down, back)

    if not a.uygula:
        print("\n(kuru kosu -- yazmak icin --uygula ekleyin)")
        return 0

    yaz(down, back)
    print(f"\nYAZILDI: scripts/standoff_geom.sh "
          f"(yedek: {YEDEK.relative_to(KOK)}; geri almak icin --geri-al)")
    yeni = standoff_degerleri()
    print(f"  dogrulama: BACK {yeni['back']:g}  DOWN {yeni['down']:g}  "
          f"TILT {yeni['tilt']:+.2f} deg")
    if abs(yeni['down'] - down) > 1e-6 or abs(yeni['back'] - back) > 1e-6:
        print("*** UYARI: kaynaklanan degerler yazilanla uyusmuyor "
              "(YILDIZ_* env ezmesi ya da eski turetim dali acik?) ***")
        return 1
    print("\nSonraki adim: A/B kosusu (SDF'ye dokunulmadi, yeniden derleme yok)\n"
          "  SURE=360 PLAN=missions/hedef_elips.plan tools/senaryo.sh\n"
          "Kriter: tespit orani korunmali; terminal bantta (yukaridaki Faz C\n"
          "menzilinin altinda) kadraj kaybi artmamali.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
