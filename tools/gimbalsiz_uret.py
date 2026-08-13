#!/usr/bin/env python3
"""Secilen kopter modelinin GIMBALSIZ varyantini uretir.

NEDEN VAR: gimbal dali kamerayi gimbal_small_2d'nin tilt eklemine bagladi.
Gimbal bir arizanin (ornegin ARM engelleyen bir fizik kararsizligi) suphelisi
oldugunda ya da eski davranisla A/B gerektiginde, GIMBALSIZ bir kola
donebilmek lazim. Bu betik ASIL SDF'LERE DOKUNMAZ: ayri bir agac uretir.

NASIL SECILIR: yildizlar_gudum.sh --gimbalsiz  (ya da YILDIZ_GIMBAL=0)
Launcher Iris icin models_sabit'i, Hummingbird icin
models_hummingbird_sabit'i, RoboFly icin models_robofly_sabit'i
GAZEBO_MODEL_PATH'in basina koyar. Dunya dosyasi DEGISMEZ.

Uc donusum:
  1) <include> model://gimbal_small_2d  blogu silinir
  2) <joint name="gimbal_mount"> silinir
  3) camera_mount'un ebeveyni gimbal_1::tilt_link -> secilen govdenin
     base_link'i olur
     (yani kamera yine govdeye SABIT, gimbal oncesi davranis)

KULLANIM:
    python3 tools/gimbalsiz_uret.py                         # Iris
    python3 tools/gimbalsiz_uret.py --arac hummingbird      # Hummingbird
    python3 tools/gimbalsiz_uret.py --arac robofly          # RoboFly
    python3 tools/gimbalsiz_uret.py --arac iris --kontrol   # yazmadan kontrol
"""

import argparse
import os
import re
import shutil
import sys

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
def donustur(ham: str, kaynak_adi: str, govde_linki: str) -> str:
    baslik = (f'<!-- OTOMATIK URETILDI: {kaynak_adi}/suru_drone_N/model.sdf\'in '
              'GIMBALSIZ\n'
              '     (govdeye sabit kamera) varyanti. Elle duzenleme; kaynak '
              'degisince\n'
              '     tools/gimbalsiz_uret.py ile yeniden uretilir. -->\n')
    ham = re.sub(
        r'\n\s*<include>\s*\n\s*<uri>model://gimbal_small_2d</uri>.*?</include>\n',
        '\n', ham, flags=re.S)
    ham = re.sub(r'\n\s*<joint name="gimbal_mount".*?</joint>\n',
                 '\n', ham, flags=re.S)
    ham = ham.replace('<parent>gimbal_1::tilt_link</parent>',
                      f'<parent>{govde_linki}</parent>')
    # XML bildirimi dosyanin mutlak ilk satiri olmak zorundadir. Uretim
    # bilgisini bildirimin ardina koy; aksi halde Gazebo tolere etse bile
    # standart XML dogrulayicilari dosyayi reddeder.
    if ham.startswith('<?xml'):
        ilk_satir_sonu = ham.find('\n') + 1
        return ham[:ilk_satir_sonu] + baslik + ham[ilk_satir_sonu:]
    return baslik + ham


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--arac', choices=('iris', 'hummingbird', 'robofly'), default='iris',
                   help='govde modeli (varsayilan: iris)')
    p.add_argument('--kontrol', action='store_true',
                   help='yalniz guncellik kontrolu yap, YAZMA')
    a = p.parse_args()

    if a.arac == 'hummingbird':
        kaynak_dizin = os.path.join(KOK, 'models_hummingbird')
        hedef_dizin = os.path.join(KOK, 'models_hummingbird_sabit')
        govde_linki = 'hummingbird::hummingbird/base_link'
        kaynak_adi = 'models_hummingbird'
    elif a.arac == 'robofly':
        kaynak_dizin = os.path.join(KOK, 'models_robofly')
        hedef_dizin = os.path.join(KOK, 'models_robofly_sabit')
        govde_linki = 'robofly::base_link'
        kaynak_adi = 'models_robofly'
    else:
        kaynak_dizin = os.path.join(KOK, 'models')
        hedef_dizin = os.path.join(KOK, 'models_sabit')
        govde_linki = 'iris::base_link'
        kaynak_adi = 'models'

    bayat = []
    for ad in sorted(os.listdir(kaynak_dizin)):
        kaynak = os.path.join(kaynak_dizin, ad, 'model.sdf')
        if not ad.startswith('suru_drone_') or not os.path.isfile(kaynak):
            continue
        hedef_dir = os.path.join(hedef_dizin, ad)
        hedef = os.path.join(hedef_dir, 'model.sdf')
        yeni = donustur(open(kaynak).read(), kaynak_adi, govde_linki)
        eski = open(hedef).read() if os.path.isfile(hedef) else None
        if eski == yeni:
            print(f'  guncel  : {ad}')
            continue
        bayat.append(ad)
        if a.kontrol:
            print(f'  BAYAT   : {ad}')
            continue
        os.makedirs(hedef_dir, exist_ok=True)
        cfg = os.path.join(kaynak_dizin, ad, 'model.config')
        if os.path.isfile(cfg):
            shutil.copy(cfg, hedef_dir)
        open(hedef, 'w').write(yeni)
        # Uretilen dosyada YAPISAL gimbal atfi kalmamali (yorumlar serbest).
        for desen in ('<uri>model://gimbal_small_2d',
                      '<joint name="gimbal_mount"',
                      '<parent>gimbal_1::'):
            if desen in yeni:
                raise SystemExit(f'HATA: {ad} icinde hala {desen!r} var')
        print(f'  URETILDI: {ad}')

    if a.kontrol and bayat:
        print(f'\n{len(bayat)} model BAYAT -> python3 tools/gimbalsiz_uret.py')
        return 1
    print(f'\n{os.path.basename(hedef_dizin)}/ hazir. '
          'Kullanim: ./yildizlar_gudum.sh --gimbalsiz')
    return 0


if __name__ == '__main__':
    sys.exit(main())
