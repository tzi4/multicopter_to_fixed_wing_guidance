#!/usr/bin/env python3
"""
calibrate_gimbal_timing.py - MEASURS camera pipeline latency
============================================================ Problem: frame and attitude sample are
not produced at the same time. There is a constant delay between the moment the camera CAPTURES a
frame and the moment that frame REACHES us (sensor read + ROS/driver + transfer). Attitude comes in
a much shorter way. This difference enters the de-rotation directly as error: 30 ms slip at body
speed 30 degrees/s means ~1 degrees.

REFERENCE SIGNAL CHANGED (gimbal branch, 2026-08-05)
-------------------------------------------------- Old benchmark: at correct delay |corr(stab_ey,
pitch)| It would be MINIMUM. Since the camera is stabilized in the physical tilt gimbal, the body NO
LONGER reflects pitch in the image - meaning there is no pitch signal left to minimize; The curve
becomes flat at each delay and the tool randomly chooses a minimum.

New criteria: TILT DOES NOT DISSOLV GIMBAL ROLL. The camera lies with the body, roll is FULLY
reflected in the image and ONLY software de-rotation clears it. In the lying frame, the target that
is open by ex horizontally moves by ~ ex*sin(roll) vertically. Therefore:

    at correct delay |corr(stab_ey, raw_ex*sin(roll(t-delay)))| MINIMUM

At the wrong delay, de-rotation uses roll of the wrong moment, some of the leakage remains in
stab_ey and the correlation grows. raw_ex in the regressor comes from pixel geometry (bbox_cx),
which is INDEPENDENT of attitude; Only the roll is latency sensitive - which is exactly what is
being sought after.

Rationale for this objective: aligning tilt_status with raw_my measures the gimbal-state topic's
delay relative to the frame instead of the camera pipeline delay. It also requires vertical target
motion and adds a new inference chain. The selected objective retains the scan loop, VirtualGimbal
recomputation, and reporting. When the CSV includes tilt_status_deg, each candidate recomputes joint
angle from the shifted attitude using the closed-form joint_angle expression, matching the flight
pipeline.

It is NOT dependent on the simulation clock: both logs are stamped with time.monotonic(), they are
generated the same way in real flight.

Usage: tools/calibrate_gimbal_timing.py run/proof/gimbal.csv run/proof/attitude_value.csv
tools/calibrate_gimbal_timing.py gimbal.csv # attitude same as CSV # (PLUS delay is measured)
"""

import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yildizlar_gimbal import VirtualGimbal, joint_angle        # noqa: E402

EXCITATION_ROLL_DEG = 1.0      # std(roll); Delay cannot be removed below
EXCITATION_REG = 0.05          # std(raw_ex*sin(roll)) [deg]; no reference below
DISTINGUISH_CAPABILITY = 0.02      # the peak-bottom difference of the curve; below minimum meaningless


def read_value(path_value):
    with open(path_value) as f:
        return list(csv.DictReader(f))


def correlation(a, b):
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


class AttitudeSeries:
    """roll/pitch series with time stamp; desired master linear interpolation."""

    def __init__(self, rows_value):
        # Time smash: 't_local' in separate attitude log, 't_frame' when reading from the same CSV ('t' in
        # oldest logs).
        label_item = None
        for candidate_value in ('t_local', 't_frame', 't'):
            if rows_value and candidate_value in rows_value[0]:
                label_item = candidate_value
                break
        self.time_label = label_item
        self.t, self.roll, self.pitch = [], [], []
        if label_item is None:
            return
        for s in rows_value:
            try:
                self.t.append(float(s[label_item]))
                self.roll.append(math.radians(float(s['roll_deg'])))
                self.pitch.append(math.radians(float(s['pitch_deg'])))
            except (ValueError, KeyError, TypeError):
                continue

    @staticmethod
    def _blend(a, b, k):
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
        return (self._blend(self.roll[i - 1], self.roll[i], k),
                self._blend(self.pitch[i - 1], self.pitch[i], k))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('gimbal_csv')
    p.add_argument('attitude_csv', nargs='?',
                   help='separate attitude log; If not given, the attitude is the same as CSV\' '
                        'is read (then the delay is measured PLUS)')
    p.add_argument('--mount', type=float, default=0.0,
                   help='fixed mounting angle [deg]. 0 ON GIMBAL BRANCH: camera '
                        'gimbal tilt link \' in, angle relative to the body LIVE '
                        'is the joint angle (derived from column tilt_status_deg).')
    p.add_argument('--scan-value', default='-40,200,5', metavar='START,BIT,STEP',
                   help='delay scan (ms)')
    a = p.parse_args()

    frames_value = read_value(a.gimbal_csv)
    same_file = a.attitude_csv is None
    series = AttitudeSeries(frames_value if same_file else read_value(a.attitude_csv))
    if len(frames_value) < 100 or len(series.t) < 100:
        raise SystemExit(f"insufficient data: {len(frames_value)} frame, {len(series.t)} attitude")
    if same_file:
        print("NOTE: no separate attitude log provided; attitude is read from square CSV.")
        print("     Those columns are already with the delay IMPLEMENTED by bbox_to_redis")
        print("     interpolated -> the number found PLUS the (remaining) delay.")

    # The name of the column is 't' in the old logs and 't_frame' in the new ones.
    time_label = 't_frame' if 't_frame' in frames_value[0] else 't'
    tilt_var = 'tilt_status_deg' in frames_value[0]
    samples_item = []
    for s in frames_value:
        try:
            t = float(s[time_label])
            cx, cy = float(s['bbox_cx']), float(s['bbox_cy'])
            aim_e = float(s['aim_active_deg'])
        except (ValueError, KeyError, TypeError):
            continue
        eps = None
        if tilt_var:
            try:
                eps = float(s['tilt_status_deg'])
            except (ValueError, TypeError):
                eps = None
        samples_item.append((t, cx, cy, aim_e, eps))
    if len(samples_item) < 100:
        raise SystemExit(f"insufficient available frames: {len(samples_item)}")

    g = VirtualGimbal(mount_phys_pitch_deg=a.mount)
    # REFERENCE: HORIZONTAL aperture of the target in the frame, directly from the pixel (attitude
    # independent -> delay insensitive). Roll leakage is this multiplied by sin(roll).
    raw_ex = [math.degrees(math.atan((cx - g.cx) / g.fx))
              for _, cx, _, _, _ in samples_item]

    start_value, bit, step_value = (float(x) for x in a.scan_value.split(','))
    print(f"square: {len(samples_item)} attitude example: {len(series.t)}   "
          f"attitude range: {series.t[-1]-series.t[0]:.0f} s   "
          f"tilt column: {'PRESENT' if tilt_var else 'absent'}")
    print(f"scan: {start_value:.0f} .. {bit:.0f} ms , step {step_value:.0f} ms")
    print()
    print("  delay |corr(stab_ey, ex*sin(roll))| std(stab_ey) |corr(.,pitch)|")

    best_candidate = None
    result_value = []
    reg_std_max = 0.0
    roll_std_max = 0.0
    ms = start_value
    while ms <= bit + 1e-9:
        ey_l, reg_l, pitch_l, roll_l = [], [], [], []
        for (t_frame, cx, cy, aim_e, eps), ex in zip(samples_item, raw_ex):
            hold_value = series.at(t_frame - ms / 1000.0)
            if hold_value is None:
                continue
            roll, pitch = hold_value
            joint = None if eps is None else joint_angle(eps, pitch, roll)
            g.aim_pitch_deg = aim_e
            g._last_aim_deg = None
            _, ey = g.angle_error_value(cx, cy, roll, pitch, None, joint_deg=joint)
            ey_l.append(ey)
            reg_l.append(ex * math.sin(roll))
            pitch_l.append(math.degrees(pitch))
            roll_l.append(math.degrees(roll))
        r = abs(correlation(ey_l, reg_l))
        rp = abs(correlation(ey_l, pitch_l))
        sd = std(ey_l)
        reg_std_max = max(reg_std_max, std(reg_l))
        roll_std_max = max(roll_std_max, std(roll_l))
        if not math.isnan(r):
            result_value.append((ms, r, sd, rp))
            if best_candidate is None or r < best_candidate[1]:
                best_candidate = (ms, r, sd, rp)
        else:
            result_value.append((ms, float('nan'), sd, rp))
        ms += step_value

    for ms, r, sd, rp in result_value:
        sign_value = '  <<< EN GOOD' if best_candidate and ms == best_candidate[0] else ''
        bar_item = '' if math.isnan(r) else '#' * int(r * 50)
        rs = '  nan' if math.isnan(r) else f"{r:5.3f}"
        print(f"  {ms:6.0f} ms   {rs}  {bar_item:<25s} {sd:6.2f}  "
              f"{rp:5.3f}{sign_value}")

    print()
    print(f"roll uyarimi: std {roll_std_max:.2f} deg   "
          f"referans u = ex*sin(roll): std {reg_std_max:.4f} deg")

    # --- JUDGMENT: first see if the data answers this question -------
    if best_candidate is None or roll_std_max < EXCITATION_ROLL_DEG or reg_std_max < EXCITATION_REG:
        print()
        print("INSUFFICIENT DATA: the body is not swung roll (or the target")
        print("  stopped right in the middle), so there is something that de-rotation will clear.")
        print("  leak NOT produced. The delay cannot be removed from this record.")
        print(f"  Required: std(roll) > {EXCITATION_ROLL_DEG:.1f} deg AND "
              f"std(ex*sin(roll)) > {EXCITATION_REG:.2f} deg")
        print("  Solution: with registration of a maneuvering run (ellipse/infinity plan) "
              "tekrarlayin.")
        return 2

    valid_value = [r for _, r, _, _ in result_value if not math.isnan(r)]
    distinguish = max(valid_value) - min(valid_value)
    zero = [r for m, r, _, _ in result_value if m == 0 and not math.isnan(r)]
    if zero:
        print(f"0 ms (uncorrected) : |corr| ={zero[0]:.3f}")
    print(f"BEST DELAY: {best_candidate[0]:.0f} ms -> |corr| ={best_candidate[1]:.3f}  "
          f"std = {best_candidate[2]:.2f} deg")
    print(f"curve discrimination: peak-trough {distinguish:.3f}")
    if distinguish < DISTINGUISH_CAPABILITY:
        print("  *** CURVE STRAIGHT: within minimal noise, DO NOT RELY on this number ***")
        print("  (a longer/more maneuverable recording is required)")
        return 2
    if best_candidate[0] in (start_value, bit):
        print("  *** MINIMUM SCAN UCUNDA: --scan-value araligini genisletin ***")

    print()
    print(f"Usage: bbox_to_redis.py --camera-latency-ms {best_candidate[0]:.0f}")
    print(f"or YILDIZ_CAMERA_LATENCY_MS={best_candidate[0]:.0f} ./yildizlar_guidance.sh")
    if same_file:
        print("(RESIDUAL delay: ADDED to the existing setting, not replaced)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
