#!/usr/bin/env python3
"""
measure_aim.py: estimate vertical aiming offset, AIM / TILT TRIM, from measurements.
================================================================== The relationship verified in
yildizlar_gimbal.py is: virtual-frame center = -aim. With aim=0, tracker_bbox_stab reports target
elevation relative to the horizon as ey = -(target elevation). Centering the target requires aim =
-elevation = ey. The median measured ey is therefore the required vertical aiming correction.

[GIMBAL BRANCH 2026-08-05] The MEANING of this number has changed. In the past, the camera was fixed
to the body and AIM was just a SOFTWARE offset: it shifted the center of the virtual frame, not the
physical FOV. Now the camera is on a self-stabilizing physical single-axis tilt gimbal and the
vertical axis is COMMANDABLE (scripts/standoff_geom.sh -> YILDIZ_TILT = atan(down/back)). So the
median ey measured here is actually a TILT TRIM:

    YILDIZ_TILT_new = YILDIZ_TILT_previous + median(ey)

and this fix actually rotates the physical FOV -- BRINGS the target BACK into the frame. You can
also apply the same number as software AIM, but it only shifts the virtual center; If there is a
systematic vertical shift, the right place is TILT. (In summary: this tool is no longer "virtual
gimbal calibration", but "vertical apr trim measurement".)

Usage (when running with bbox_to_redis --aim 0): tools/measure_aim.py --duration-value 300 --label-value ellipse
"""

import argparse
import json
import statistics
import time

import redis


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration-value', type=float, default=300)
    p.add_argument('--label-value', default='')
    p.add_argument('--channel-value', default='tracker_bbox_stab')
    a = p.parse_args()

    r = redis.Redis(host='localhost', port=6379, db=0)
    ps = r.pubsub()
    ps.subscribe(a.channel_value)
    ex_l, ey_l = [], []
    t0 = time.time()
    last_value = t0
    while time.time() - t0 < a.duration_value:
        m = ps.get_message(timeout=1.0)
        if not m or m['type'] != 'message':
            continue
        try:
            d = json.loads(m['data'].decode())
        except Exception:
            continue
        ex_l.append(float(d[4]))
        ey_l.append(float(d[5]))
        if time.time() - last_value > 30:
            last_value = time.time()
            print(f"  n= {len(ey_l)} ey median= {statistics.median(ey_l):+.2f}", flush=True)

    print()
    if len(ey_l) < 20:
        print(f"[{a.label_value}] INSUFFICIENT SAMPLE ({len(ey_l)})")
        return
    ey_l.sort(); ex_l.sort()
    n = len(ey_l)
    def q(v, p): return v[min(n - 1, int(p * n))]
    print(f"=== VERTICAL AIM (AIM / TILT TRIM) MEASUREMENT [{a.label_value}] n={n} ===")
    print(f"  ey (vertical angle error, deg): %10 {q(ey_l,.10):+6.2f}  "
          f"median {q(ey_l,.5):+6.2f} % 90 {q(ey_l,.90):+6.2f}")
    print(f"  ex (horizontal, deg) : %10 {q(ex_l,.10):+6.2f}  "
          f"median {q(ex_l,.5):+6.2f} % 90 {q(ex_l,.90):+6.2f}")
    print(f"  >>> VERTICAL TRIM FOR THIS PLAN = {q(ey_l,.5):+.2f} degrees")
    print(f"      (target elevation relative to the horizon = {-q(ey_l,.5):+.2f} degrees)")
    print(f"      Apply to TILT (physical gimbal actually rotates FOV):")
    print(f"        YILDIZ_TILT_new = YILDIZ_TILT_previous {q(ey_l,.5):+.2f}")
    print(f"      or as software offset: bbox_to_redis --aim "
          f"{q(ey_l,.5):+.2f} (shifts virtual center only)")


if __name__ == '__main__':
    main()
