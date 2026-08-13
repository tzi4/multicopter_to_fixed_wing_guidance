#!/usr/bin/env python3
"""
gorev_baslat.py - TUM ARACLARI GOREVE SOKAN TEK KOMUT
=====================================================
bumblebee/formation.py'nin bu ortamdaki karsiligi. Fark: bumblebee'de iki
sabit kanat vardi ve ikisi de AUTO'ya aliniyordu; burada hedef sabit kanat
AUTO plan ucar, avci kopter(ler) GUIDED'da havalanip gudum koduna hazir
bekler.

YAPTIGI SIRAYLA:
  1. Butun araclara baglanir ve SysID'leri DOGRULAR
     (14550'de hepsi bir arada oldugu icin yanlis porta baglanmak sessizce
      baska araca komut gondermek demek).
  2. Hedef ucaga plani yukler ve GERI OKUYARAK dogrular (scripts/load_plan.py).
  3. Hedefi ARM edip AUTO'ya alir, tirmandigini dogrular.
  4. Kopterleri GUIDED'da ARM edip --drone-alt metreye cikarir.
  5. Istenirse hedefe seyir hizi komutu gonderir.
  6. Kapanista durum tablosu basar.

Her adim DOGRULANIR; biri tutmazsa hata verip durur (sessizce yarim kalmis
bir kurulumla gudum testine girmek en pahali hata).

KULLANIM:
  tools/gorev_baslat.py                          # varsayilan: elips plan, 1 kopter, 60 m
  tools/gorev_baslat.py --drones 3 --drone-alt 80
  tools/gorev_baslat.py --plan missions/hedef_tur.plan --hedef-hiz 16
  tools/gorev_baslat.py --sadece-hedef           # kopterlere dokunma
  tools/gorev_baslat.py --sadece-drone           # hedefe dokunma
"""

import argparse
import math
import os
import subprocess
import sys
import time

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(KOK, 'tools'))

from pymavlink import mavutil                                   # noqa: E402
from suru_komut import (COPTER_GUIDED, PLANE_AUTO, TARGET_PORT,  # noqa: E402
                        TARGET_SYSID, arm_et, baglan, drone_baglan, hiz_ayarla,
                        komut, konum_al, mod_ayarla, mod_bekle, prearm_bekle)


def adim(n, metin):
    print(f"\n[{n}] {metin}", flush=True)


def plan_yukle(plan, port, sysid):
    """scripts/load_plan.py: MISSION_ITEM_INT ile yukler, GERI OKUYUP dogrular."""
    yol = os.path.join(KOK, 'scripts', 'load_plan.py')
    sonuc = subprocess.run([sys.executable, yol, '--plan', plan,
                            '--ports', f'{port}:{sysid}'],
                           capture_output=True, text=True)
    print(sonuc.stdout.strip() or sonuc.stderr.strip(), flush=True)
    return sonuc.returncode == 0


def hedefi_baslat(args):
    adim(1, f'Hedef ucak (SysID {TARGET_SYSID}) plani yukleniyor: '
            f'{os.path.relpath(args.plan, KOK)}')
    # Plan yuklemesi 14602'yi kullanir (14601 elle komut araclarina ait);
    # ayni UDP portunu iki surec baglayamaz.
    if not plan_yukle(args.plan, 14602, TARGET_SYSID):
        raise SystemExit('plan yuklenemedi')

    adim(2, 'Hedef ucak baglaniyor, GPS/EKF bekleniyor')
    hedef = baglan(TARGET_PORT, TARGET_SYSID)
    if not prearm_bekle(hedef):
        raise SystemExit('hedef ucak GPS fix alamadi')

    adim(3, 'Hedef ucak AUTO moduna aliniyor ve ARM ediliyor')
    mod_ayarla(hedef, PLANE_AUTO)
    if not mod_bekle(hedef, PLANE_AUTO):
        raise SystemExit('hedef AUTO moduna gecmedi')
    if not arm_et(hedef):
        raise SystemExit('hedef ARM edilemedi')
    # Bazi ArduPlane surumlerinde AUTO'da ARM tek basina kalkisi tetiklemiyor.
    komut(hedef, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)

    adim(4, f'Hedef ucagin {args.hedef_alt:.0f} m\'ye tirmanmasi bekleniyor')
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        k = konum_al(hedef)
        if k is None:
            continue
        print(f'    irtifa={k[2]:6.1f} m', flush=True)
        if k[2] >= args.hedef_alt:
            break
        time.sleep(2)
    else:
        raise SystemExit(f'hedef {args.hedef_alt} m\'ye {args.timeout}s icinde cikmadi')

    if args.hedef_hiz > 0:
        # ArduPlane'de bu komutun ETKI ETMESI icin TECS_SYNAIRSPEED 1 sart
        # (params/hedef_ucak.parm icinde olcumuyle birlikte yaziyor).
        sonuc = komut(hedef, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                      0, args.hedef_hiz, -1, 0)
        print(f'    seyir hizi {args.hedef_hiz} m/s -> ACK={sonuc}', flush=True)
    hedef.close()
    print('    hedef ucak gorevde.', flush=True)


def kopterleri_baslat(args):
    for i in range(1, args.drones + 1):
        adim(5 + i, f'drone_{i} (SysID {i}) GUIDED\'da kaldiriliyor '
                    f'-> {args.drone_alt:.0f} m')
        d = drone_baglan(i)
        if not prearm_bekle(d):
            raise SystemExit(f'drone_{i} GPS fix alamadi')
        mod_ayarla(d, COPTER_GUIDED)
        if not mod_bekle(d, COPTER_GUIDED):
            raise SystemExit(f'drone_{i} GUIDED moduna gecmedi')
        if not arm_et(d):
            raise SystemExit(f'drone_{i} ARM edilemedi')
        komut(d, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
              args.drone_alt)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            k = konum_al(d)
            if k is None:
                continue
            if k[2] >= args.drone_alt * 0.95:
                print(f'    drone_{i} {k[2]:.1f} m\'de hazir.', flush=True)
                break
            time.sleep(1)
        else:
            raise SystemExit(f'drone_{i} {args.drone_alt} m\'ye cikmadi')
        if args.drone_hiz > 0:
            # GUIDED hiz tavani: WPNAV_SPEED parametre ust siniri 20 m/s;
            # DO_CHANGE_SPEED bu tavani kirpmadan gecer (suru_komut.hiz_ayarla).
            print(f'    hiz tavani {args.drone_hiz} m/s -> '
                  f'ACK={hiz_ayarla(d, args.drone_hiz)}', flush=True)
        d.close()


def durum_tablosu(args):
    print('\n' + '=' * 68)
    print(f'{"arac":12s} {"port":>6s} {"sysid":>6s} {"mod":>5s} {"arm":>4s} '
          f'{"irtifa":>8s}  konum')
    print('-' * 68)
    hedefler = [(14551 + 10 * i, i + 1, f'drone_{i+1}') for i in range(args.drones)]
    hedefler.append((TARGET_PORT, TARGET_SYSID, 'hedef_ucak'))
    for port, sysid, ad in hedefler:
        try:
            m = baglan(port, sysid, timeout=8)
        except SystemExit:
            print(f'{ad:12s} {port:6d} {sysid:6d}  BAGLANTI YOK')
            continue
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        k = konum_al(m)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        print(f'{ad:12s} {port:6d} {sysid:6d} {hb.custom_mode if hb else -1:5d} '
              f'{"E" if armed else "H":>4s} {k[2] if k else 0:7.1f}m  '
              f'{k[0]:.6f},{k[1]:.6f}' if k else '')
        m.close()
    print('=' * 68)


def standoff_geometrisi():
    """Ipucu satirinda basilacak standoff ikilisini ortamdan turetir.

    TEK KAYNAK scripts/standoff_geom.sh'tir (yildizlar_gudum.sh ve
    tools/senaryo.sh onu kaynaklar); ipucu da artik formul kopyalamak
    yerine onu KAYNAKLAYIP okur. Kopya surumun YILDIZ_MOUNT varsayilani
    30'da kalmisti ve ortam 0 dereceye gectikten sonra --down 13
    oneriyordu (dogrusu tasarim degeri 4; mount~0'da turetim gecersiz).
    """
    try:
        cikti = subprocess.check_output(
            ['bash', '-c',
             f'source "{KOK}/scripts/standoff_geom.sh" >/dev/null 2>&1; '
             'echo "$YILDIZ_BACK $YILDIZ_DOWN $YILDIZ_MOUNT $YILDIZ_PITCH_TRIM"'],
            text=True)
        back, down, mount, trim = (float(x) for x in cikti.split())
    except (OSError, ValueError, subprocess.CalledProcessError):
        # standoff_geom.sh okunamazsa gorev raporu ipucu yuzunden
        # patlamasin: mevcut 0-derece kurulumun bilinen ikilisine dus.
        back, down, mount, trim = 25.0, 4.0, 0.0, -2.5
    return back, down, mount, trim


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', default=os.path.join(KOK, 'missions', 'hedef_elips.plan'))
    p.add_argument('--drones', type=int, default=1, help='kac kopter kaldirilacak (0..5)')
    p.add_argument('--drone-alt', type=float, default=60)
    p.add_argument('--drone-hiz', type=float, default=0,
                   help='kopter GUIDED hiz tavani (m/s); 0 = dokunma')
    p.add_argument('--hedef-alt', type=float, default=50,
                   help='hedefin dogrulanacak tirmanis irtifasi (m)')
    p.add_argument('--hedef-hiz', type=float, default=0,
                   help='hedef seyir hizi (m/s); 0 = plandaki deger')
    p.add_argument('--timeout', type=float, default=240)
    p.add_argument('--sadece-hedef', action='store_true')
    p.add_argument('--sadece-drone', action='store_true')
    args = p.parse_args()

    if not 0 <= args.drones <= 5:
        raise SystemExit('--drones 0..5 arasinda olmali')
    if not os.path.isfile(args.plan):
        raise SystemExit(f'plan bulunamadi: {args.plan}')

    t0 = time.time()
    if not args.sadece_drone:
        hedefi_baslat(args)
    if not args.sadece_hedef and args.drones > 0:
        kopterleri_baslat(args)
    durum_tablosu(args)
    print(f'\nGorev basladi ({time.time() - t0:.0f} s).')
    back, down, mount, trim = standoff_geometrisi()
    # IKI SUREC BIRDEN yazilir: 2026-08-05'te ucuncu kez goruntulu gudum
    # baslatilmadan kosuldu ve "MPC titriyor / hedefi takip etmiyor" diye
    # goruldu -- oysa MPC hic kosmuyordu. Ipucu tek surec gosterdigi surece
    # bu hata tekrarlaniyor.
    print('\n' + '=' * 68)
    print('SIRADA -- IKI AYRI TERMINALDE (ikisi de gerekli):')
    print('=' * 68)
    print('  [1] KONUMLU (yaklasim):')
    print('      cd guidance_allstar && python3 simple_guided_follow.py \\')
    print(f'          --no-kill-mode --yaw-lock --back {back:g} --down {down:g}')
    print('      # ikinci yaklasim secenegi: --yaklasim kesisme')
    print()
    print('  [2] GORUNTULU (devralan) -- BUNU UNUTMA:')
    print('      cd guidance_allstar && python3 mpc_gudum.py')
    print('      # ya da diger kol: python3 takip_gudum.py')
    print()
    print('  [2] BASLATILMAZSA goruntuluye GECILMEZ: olu-adam anahtari')
    print('  ("goruntulu_hayatta") gecisi engeller ve bbox.log su satiri basar:')
    print('      [KARAR] goruntulu kontrolcu YOK ... baslatmayi unuttun mu?')
    print('=' * 68)
    print(f'  (standoff kaynagi scripts/standoff_geom.sh: mount={mount:g} '
          f'trim={trim:g}; YILDIZ_BACK/YILDIZ_MOUNT/YILDIZ_PITCH_TRIM/'
          'YILDIZ_DOWN ile degisir)')


if __name__ == '__main__':
    main()
