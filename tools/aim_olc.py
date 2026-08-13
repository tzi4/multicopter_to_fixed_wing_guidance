#!/usr/bin/env python3
"""
aim_olc.py - dikey nisan ofsetini (AIM / TILT TRIM) olcumden bulur
==================================================================
Baginti (yildizlar_gimbal.py'de ispatlandi):  sanal kadraj merkezi = -aim
'tracker_bbox_stab' kanali aim=0 ile kosarken hedefin ufka gore acisini
verir: yayinlanan ey = -(hedefin yukselisi). Hedefi TAM MERKEZE oturtmak
icin aim = -yukselis = ey. Yani olculen ey'in ORTANCASI, dikey nisanin
duzeltilmesi gereken miktaridir.

[GIMBAL DALI 2026-08-05] Bu sayinin ANLAMI degisti. Eskiden kamera govdeye
sabitti ve AIM yalnizca YAZILIM bir ofsetti: sanal kadrajin merkezini
kaydiriyor, fiziksel FOV'u kaydirmiyordu. Artik kamera kendini stabilize
eden fiziksel tek eksen tilt gimbalinde ve dikey eksen KOMUT EDILEBILIR
(scripts/standoff_geom.sh -> YILDIZ_TILT = atan(down/back)). Yani burada
olculen ortanca ey aslinda bir TILT TRIM'dir:

    YILDIZ_TILT_yeni = YILDIZ_TILT_eski + ortanca(ey)

ve bu duzeltme fiziksel FOV'u gercekten dondurur -- hedefi kadraja GERI
GETIRIR. Ayni sayiyi yazilim AIM'i olarak da uygulayabilirsiniz, ama o
yalniz sanal merkezi kaydirir; sistematik bir dikey kayma varsa dogru yer
TILT'tir. (Ozetle: bu arac artik "sanal gimbal kalibrasyonu" degil,
"dikey nisan trim olcumu".)

Kullanim (bbox_to_redis --aim 0 ile kosarken):
    tools/aim_olc.py --sure 300 --etiket elips
"""

import argparse
import json
import statistics
import time

import redis


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--sure', type=float, default=300)
    p.add_argument('--etiket', default='')
    p.add_argument('--kanal', default='tracker_bbox_stab')
    a = p.parse_args()

    r = redis.Redis(host='localhost', port=6379, db=0)
    ps = r.pubsub()
    ps.subscribe(a.kanal)
    ex_l, ey_l = [], []
    t0 = time.time()
    son = t0
    while time.time() - t0 < a.sure:
        m = ps.get_message(timeout=1.0)
        if not m or m['type'] != 'message':
            continue
        try:
            d = json.loads(m['data'].decode())
        except Exception:
            continue
        ex_l.append(float(d[4]))
        ey_l.append(float(d[5]))
        if time.time() - son > 30:
            son = time.time()
            print(f"  n={len(ey_l)} ey ortanca={statistics.median(ey_l):+.2f}", flush=True)

    print()
    if len(ey_l) < 20:
        print(f"[{a.etiket}] YETERSIZ ORNEK ({len(ey_l)})")
        return
    ey_l.sort(); ex_l.sort()
    n = len(ey_l)
    def q(v, p): return v[min(n - 1, int(p * n))]
    print(f"=== DIKEY NISAN (AIM / TILT TRIM) OLCUMU [{a.etiket}] n={n} ===")
    print(f"  ey (dikey aci hatasi, deg): %10 {q(ey_l,.10):+6.2f}  "
          f"ortanca {q(ey_l,.5):+6.2f}  %90 {q(ey_l,.90):+6.2f}")
    print(f"  ex (yatay, deg)           : %10 {q(ex_l,.10):+6.2f}  "
          f"ortanca {q(ex_l,.5):+6.2f}  %90 {q(ex_l,.90):+6.2f}")
    print(f"  >>> BU PLAN ICIN DIKEY TRIM = {q(ey_l,.5):+.2f} derece")
    print(f"      (hedefin ufka gore yukselisi = {-q(ey_l,.5):+.2f} derece)")
    print(f"      TILT'e uygula (fiziksel gimbal, FOV'u gercekten dondurur):")
    print(f"        YILDIZ_TILT_yeni = YILDIZ_TILT_eski {q(ey_l,.5):+.2f}")
    print(f"      ya da yazilim ofseti olarak: bbox_to_redis --aim "
          f"{q(ey_l,.5):+.2f} (yalniz sanal merkezi kaydirir)")


if __name__ == '__main__':
    main()
