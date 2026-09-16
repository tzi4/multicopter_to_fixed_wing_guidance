#!/usr/bin/env python3
"""
gimbal_evidence.py - indicates by number that the gimbal chain is working in LIVE FLIGHT
============================================================== GIMBAL BRANCH (2026-08-05): METER
CHANGED. The camera is no longer fixed to the body, but on a PHYSICAL single-axis (tilt) gimbal that
stabilizes itself. The old criterion (corr(stab_ey, pitch) < 0.5 * corr(raw_ey, pitch)) is now a
NULL test: body pitch will be cleared at raw_ey as it is NEVER reflected in the image pitch does NOT
exist anyway - the denominator goes to zero, the criterion becomes meaningless.

TWO-LAYER evidence instead:

  (a) PHYSICAL LAYER - the gimbal itself:
      |corr(raw_ey, pitch)| < 0.30
      RAW (pre-de-rotation) vertical error must be INDEPENDENT from body pitch. This shows that
      mechanical stabilization is actually working; measured before the software ever comes into
      play. In the old world, this correlation was HIGH (the error came from the plane's bank, not
      the target).

  (b) SOFTWARE LAYER - residual work of de-rotation: Tilt gimbal DOES NOT decode roll ; The camera
  lies with the body. In a lying frame, the target that is open by ex horizontally is shifted by ~
  ex * sin ( roll ) vertically. Criteria:
      |corr(stab_ey, ex*sin(roll))| < |corr(raw_ey, ex*sin(roll))| (regressor SAME IN BOTH LAYERS:
      raw_ex*sin(roll)). If both are weak (< 0.30) the verdict is "The effect of roll is already
      small" - it is NOT a FAILURE that the de-rotation cannot find anything to clear.

Additionally the HEALTH of the tilt chain is reported (diagnostic, not a criterion): -
median(tilt_status - tilt_cmd) : servo offset (deadband ~0.17 deg) - p95(tilt_age_ms) : staleness of
status topic - max |joint_deg - joint_angle(tilt_status, pitch, roll)| : internal consistency of the
measurement chain (does this tool use the same closed form as bbox_to_redis)

INSUFFICIENCY OF DATA: correlation judgment can only be made if there is WARNING. The correlation is
the ratio of noise to noise when the body is at rest on the ground (std(pitch) < 1 deg). In this
case, the vehicle says "INSUFFICIENT DATA" and chiakr with 2 - It does not say PASS or FAIL.

EXIT CODE: 0 = PASS, 1 = FAIL, 2 = UNDECISION (insufficient stimulation).

HISTORICAL MODE: if called with older CSVs without tilt columns
(tilt_cmd_deg/tilt_status_deg/joint_deg), the old benchmark (virtual gimbal, body-fixed camera) will
be applied.

ATTITUDE CORRELATION ON THE HORIZONTAL AXIS IS NOT THE CRITERIA (understood in 2026-08-02). In
bank-to-turn guidance, roll ITSELF is commanded with the bearing error; The stabilized horizontal
error also measures the true bearing (when working correctly). So the coupling between stab_ex and
roll is ENDOGENous - a coupling of the closed loop itself, not gimbal residue. Measurement:
corr(stab_ex, -ground_truth_lateral) = 0.992 (Gazebo ground truth, 6391 square). The horizontal
channel can only be verified by GROUND TRUTH: --ground-truth-orientation.
"""

import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:                                    # closed form SINGLE SOURCE: yildizlar_gimbal
    from yildizlar_gimbal import joint_angle       # noqa: E402
except Exception:                       # If numpy is not present: local copy of the same formula
    def joint_angle(eps_cam_deg, pitch_rad, roll_rad):
        A = math.sin(pitch_rad)
        B = math.cos(pitch_rad) * math.cos(roll_rad)
        R = math.hypot(A, B)
        s = max(-1.0, min(1.0, math.sin(math.radians(eps_cam_deg)) / max(R, 1e-9)))
        return math.degrees(math.asin(s) - math.atan2(A, B))

# Stimulation thresholds: below these no correlation judgment is made.
EXCITATION_PITCH_DEG = 1.0        # std(pitch); under "the body did not skid"
EXCITATION_ROLL_REG = 0.05        # std(raw_ex*sin(roll)) [deg]; roll no effect under
WEAK = 0.30                  # correlation "weak" threshold (both layers)


def read_value(path_value):
    with open(path_value) as f:
        return list(csv.DictReader(f))


def count_value2(rows_value, label_item):
    out = []
    for s in rows_value:
        try:
            out.append(float(s[label_item]))
        except (ValueError, KeyError, TypeError):
            out.append(float('nan'))
    return out


def clean_value(*arrays):
    n = len(arrays[0])
    hold_value = [i for i in range(n)
           if all(not math.isnan(d[i]) for d in arrays)]
    return [[d[i] for i in hold_value] for d in arrays]


def std(v):
    if len(v) < 2:
        return float('nan')
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def time_column(rows_value):
    """Frame time: 't_frame' in new logs, 't' in oldest ones."""
    return 't_frame' if 't_frame' in rows_value[0] else 't'


def median_item(v):
    if not v:
        return float('nan')
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def percentile_item(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def correlation(a, b):
    if len(a) < 3:
        return float('nan')
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa == 0 or sb == 0:
        return float('nan')
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def inclination(x, y):
    """y ~ a + k*x least squares; returns k."""
    n = len(x)
    if n < 3:
        return float('nan')
    mx, my = sum(x) / n, sum(y) / n
    sxy = sum((p - mx) * (q - my) for p, q in zip(x, y))
    sxx = sum((p - mx) ** 2 for p in x)
    return sxy / sxx if sxx else float('nan')


def interp(ts, vs, t):
    """Linear intermediate value over ts (ts must be increasing)."""
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    t0, t1 = ts[i - 1], ts[i]
    if t1 == t0:
        return vs[i]
    w = (t - t0) / (t1 - t0)
    return vs[i - 1] + w * (vs[i] - vs[i - 1])


def ground_truth_validate(gt_path, t_frame, sx, roll):
    """Verifies the horizontal channel with ground truth Gazebo.

    WARNING: the pitch_deg column in ground_truth_orientation.csv is not used. Its producer left the pitch sign
    inverted when converting Gazebo ENU attitude to NED. Measured corr(pitch_gt, pitch_gimbal) =
    -0.988, compared with +0.994 for roll. Only the position-derived ground_truth_lateral_deg column is read
    here. It is reliable because it comes from the difference between the two vehicle positions
    rather than a quaternion.

    Pointing convention: the gimbal measures ex positive to the right, ground_truth_lateral_deg measures
    positive to the right in the horizon frame but is defined in the opposite direction -> -
    ground_truth_lateral .
    """
    gt = read_value(gt_path)
    tg = count_value2(gt, 't')
    lateral = count_value2(gt, 'ground_truth_lateral_deg')
    tg, lateral = clean_value(tg, lateral)
    if len(tg) < 10:
        return None
    pair = sorted(zip(tg, lateral))
    tg = [p[0] for p in pair]
    lateral = [p[1] for p in pair]

    # TIME MATCHING: find the best constant offset with correlation in +-500 ms. The two records come from
    # separate processes; Even though the shared clock is monotonic, a constant shift may remain between
    # moments of writing.
    best_candidate = (None, -2.0)
    for step_value in range(-50, 51):
        slip = step_value / 100.0
        a, b = [], []
        for t, s in zip(t_frame, sx):
            th = t + slip
            if not (tg[0] <= th <= tg[-1]):
                continue
            a.append(s)
            b.append(-interp(tg, lateral, th))
        if len(a) < 50:
            continue
        r = correlation(a, b)
        if not math.isnan(r) and abs(r) > best_candidate[1]:
            best_candidate = (slip, abs(r))
    slip = best_candidate[0]
    if slip is None:
        return None

    ex, reference_angles, rl = [], [], []
    for t, s, r_ in zip(t_frame, sx, roll):
        th = t + slip
        if not (tg[0] <= th <= tg[-1]):
            continue
        ex.append(s)
        reference_angles.append(-interp(tg, lateral, th))
        rl.append(r_)
    if len(ex) < 50:
        return None

    # BEARING GAIN: how many times the measured ex is the actual bearing? It should be 1.000. (Before the
    # correction, 0.910 was measured due to the Ry spin of the aim.)
    k = inclination(reference_angles, ex)
    residual_item = [e - k * g for e, g in zip(ex, reference_angles)]
    sr = [math.sin(math.radians(r_)) for r_ in rl]
    inclination_roll = inclination(sr, residual_item)
    return {'n': len(ex), 'slip_ms': slip * 1000.0, 'corr': best_candidate[1],
            'gain': k, 'inclination': inclination_roll, 'std': std(residual_item)}


def scatter_write(hx, hy, sx, sy):
    print("  SCATTER (std, degree) raw -> stabilized [info]")
    ky = std(hy) / std(sy) if std(sy) else float('inf')
    kx = std(hx) / std(sx) if std(sx) else float('inf')
    print(f"     vertical error: {std(hy):6.2f} -> {std(sy):6.2f} ({ky:4.1f}x narrower)")
    print(f"     horizontal error: {std(hx):6.2f} -> {std(sx):6.2f} ({kx:4.1f}x narrower)")


def horizontal_ground_truth(ground_truth_orientation, tk, sx, roll):
    """--ground-truth-orientation block. (passed_mi, karar_verildi_mi) returns."""
    print()
    print("  HORIZONTAL CHANNEL with GROUND TRUTH (Gazebo)")
    d = ground_truth_validate(ground_truth_orientation, tk, sx, roll)
    if d is None:
        print("     no matching samples - time windows do not overlap")
        return False, True
    print(f"     matching sample: {d['n']}  "
          f"(timeshift {d['slip_ms']:+.0f} ms, |corr| {d['corr']:.4f})")
    print(f"     bearing gain: {d['gain']:.4f}  "
          f"(Must be 1.000; 0.910 was measured before current correction)")
    print(f"     residual sin(roll) slope: {d['inclination']:+.3f} deg (must be |.|<1.0)")
    print(f"     residue scatter: {d['std']:.3f} deg (must be <1.0)")
    horizontal_ok = abs(d['inclination']) < 1.0 and d['std'] < 1.0
    print("     ->", "HORIZONTAL CHANNEL VERIFIED WITH GROUND TRUTH" if horizontal_ok
          else "THERE IS NOW UNDESCRIBED IN THE HORIZONTAL CHANNEL")
    return horizontal_ok, True


# --------------------------------------------------------------- GIMBAL MODE

def criterion_gimbal(rows_value, ground_truth_orientation):
    tk = count_value2(rows_value, time_column(rows_value))
    roll = count_value2(rows_value, 'roll_deg')
    pitch = count_value2(rows_value, 'pitch_deg')
    hx = count_value2(rows_value, 'raw_ex_deg')
    hy = count_value2(rows_value, 'raw_ey_deg')
    sx = count_value2(rows_value, 'stab_ex_deg')
    sy = count_value2(rows_value, 'stab_ey_deg')
    tk, roll, pitch, hx, hy, sx, sy = clean_value(tk, roll, pitch, hx, hy, sx, sy)
    n = len(roll)
    if n < 50:
        print(f"INSUFFICIENT DATA: {n} full line only")
        return 2

    print(f"=== PHYSICAL TILT GIMBAL LIVE PROOF ({n} detection frame) ===")
    print(f"  body attitude: roll std {std(roll):5.2f} deg   "
          f"pitch std {std(pitch):5.2f} deg")
    print()
    scatter_write(hx, hy, sx, sy)
    print()

    # ---- (a) PHYSICAL LAYER -------------------------------------------
    print("  (a) PHYSICAL LAYER: does the gimbal separate the body pitch?")
    r_hy_p = abs(correlation(hy, pitch))
    r_sy_p = abs(correlation(sy, pitch))
    print(f"      |corr( raw_ey , pitch )| : {r_hy_p:5.3f} (measured: < {WEAK:.2f} )")
    print(f"      |corr( stab_ey , pitch )| : {r_sy_p:5.3f} [information]")
    pitch_excitation = std(pitch)
    if math.isnan(r_hy_p) or pitch_excitation < EXCITATION_PITCH_DEG:
        fiz = None
        print(f"      -> DATA YETERSIZ: pitch uyarimi {pitch_excitation:.2f} deg "
              f"(< {EXCITATION_PITCH_DEG:.1f} deg). This correlation without hull skidding")
        print(f"         is the ratio of noise to noise; judgment cannot be given.")
    else:
        fiz = r_hy_p < WEAK
        print("      ->", "GIMBAL PITCH'I AYIRIYOR" if fiz
              else "*** RAW ERROR STILL DUE TO PITCH - gimbal not stabilizing ***")
    print()

    # ---- (b) SOFTWARE LAYER ------------------------------------------- Tilt gimbal does not resolve
    # roll: in the lying frame, the target that is turned on horizontally ex shifts vertically ~
    # ex*sin(roll). The regressor must be the SAME IN BOTH LAYERS, otherwise the comparison of "how much
    # of which signal was cleaned" is meaningless.
    reg = [x * math.sin(math.radians(r)) for x, r in zip(hx, roll)]
    print("  (b) YAZILIM KATMANI: de-rotasyon roll sizintisini temizliyor mu?")
    print(f"      regresor u = raw_ex*sin(roll):  std {std(reg):.4f} deg")
    r_hy_u = abs(correlation(hy, reg))
    r_sy_u = abs(correlation(sy, reg))
    print(f"      |corr(raw_ey , u)|      : {r_hy_u:5.3f}")
    print(f"      |corr(stab_ey, u)| : {r_sy_u:5.3f} (criteria: LESS THAN RAW)")
    if math.isnan(r_hy_u) or math.isnan(r_sy_u) or std(reg) < EXCITATION_ROLL_REG:
        write_value = None
        print(f"      -> INSUFFICIENT DATA: roll leak regressor almost fixed "
              f"(std < {EXCITATION_ROLL_REG:.2f} deg).")
        print(f"         Nothing was produced to be cleaned; judgment cannot be given.")
    elif r_hy_u < WEAK and r_sy_u < WEAK:
        write_value = True
        print(f"      -> ROLL EFFECT IS ALREADY SMALL (both < {WEAK:.2f}); "
              f"De-rotation doesn't cause any harm either.")
    else:
        write_value = r_sy_u < r_hy_u
        print("      ->", "DE-ROTASYON ROLL SIZINTISINI AZALTIYOR" if write_value
              else "*** DE-ROTASYON ROLL SIZINTISINI AZALTMIYOR ***")
    print()

    # ---- TILT ZINCIRI SAGLIGI (teshis) ---------------------------------
    tilt_health(rows_value)

    # ---- VERDICT ----------------------------------------------------------
    print()
    decisions = [fiz, write_value]
    if any(k is False for k in decisions):
        print("RESULT: FAIL - see *** lines above")
        code_value = 1
    elif any(k is None for k in decisions):
        print("RESULT: UNDETERMINED - repeat with a run with adequate attitude stimulation")
        print("       (recording of steady or very flat flight on the ground is not sufficient to make a judgement)")
        code_value = 2
    else:
        print("RESULT: GIMBAL ZINCIRI RUNNING (fiziksel + yazilim katmani)")
        code_value = 0

    if ground_truth_orientation:
        horizontal_ok, _ = horizontal_ground_truth(ground_truth_orientation, tk, sx, roll)
        if not horizontal_ok:
            code_value = 1
    return code_value


def tilt_health(rows_value):
    """Diagnostic report of pinball chain (NOT METER - numbers stop here)."""
    print("  TILT CHAIN ​​HEALTH [diagnostic, not criterion]")
    cmd = count_value2(rows_value, 'tilt_cmd_deg')
    stop_value = count_value2(rows_value, 'tilt_status_deg')
    age_value = count_value2(rows_value, 'tilt_age_ms')
    ekl = count_value2(rows_value, 'joint_deg')
    roll = count_value2(rows_value, 'roll_deg')
    pitch = count_value2(rows_value, 'pitch_deg')

    c, d = clean_value(cmd, stop_value)
    if c:
        difference = [b - a for a, b in zip(c, d)]
        print(f"      median_item(tilt_status - tilt_cmd) : {median_item(difference):+.3f} deg  "
              f"(deadband ~0.17 deg; big difference = servo saturated/loaded)")
    else:
        print("      tilt_cmd/tilt_status has no common line")

    (y,) = clean_value(age_value)
    if y:
        print(f"      tilt_age_ms  p95 {percentile_item(y, 0.95):6.0f} ms   "
              f"median_item {median_item(y):5.0f}   maks {max(y):6.0f}   "
              f"(>1500 ms = decreasing command value)")
    else:
        print("      tilt_age_ms column is empty")

    e, dd, pp, rr = clean_value(ekl, stop_value, pitch, roll)
    if e:
        error_value = [abs(a - joint_angle(b, math.radians(p), math.radians(r)))
                for a, b, p, r in zip(e, dd, pp, rr)]
        simple = [abs(a - (b - p)) for a, b, p in zip(e, dd, pp)]
        print(f"      |joint_deg - joint_angle(status,pitch,roll)| max "
              f"{max(error_value):.4f} deg (chain internal consistency must be <0.01)")
        print(f"      |joint_deg - (status - pitch)| maks {max(simple):.4f} deg  "
              f"(roll = Validity of simplification 0)")
    else:
        print("      joint_deg column is empty - tilt mode may be turned off")


# ------------------------------------------------------------ HISTORICAL MODE

def criterion_historical(rows_value, ground_truth_orientation):
    """OLD METER: body-FIXED camera + VIRTUAL gimbal world.

    The raw vertical error would correlate strongly with the fuselage pitch (the error would come
    from the aircraft bank, not the target); The virtual gimbal should have broken this connection.
    This benchmark is a futile test since the physical gimbal ALREADY has no raw_ey with pitch -
    standing only for old records.
    """
    tk = count_value2(rows_value, time_column(rows_value))
    roll = count_value2(rows_value, 'roll_deg')
    pitch = count_value2(rows_value, 'pitch_deg')
    hx = count_value2(rows_value, 'raw_ex_deg')
    hy = count_value2(rows_value, 'raw_ey_deg')
    sx = count_value2(rows_value, 'stab_ex_deg')
    sy = count_value2(rows_value, 'stab_ey_deg')
    aim = count_value2(rows_value, 'aim_deg')
    tk, roll, pitch, hx, hy, sx, sy, aim = clean_value(tk, roll, pitch, hx, hy,
                                                 sx, sy, aim)
    n = len(roll)
    if n < 50:
        print(f"INSUFFICIENT DATA: {n} full line only")
        return 2

    print(f"=== HISTORICAL MODE: VIRTUAL GIMBAL LIVE PROOF ({n} detection frame) ===")
    print("  (No tilt columns on CSV -> assumption of body-fixed camera)")
    print(f"  body attitude: roll std {std(roll):5.2f} deg   "
          f"pitch std {std(pitch):5.2f} deg")
    print()
    scatter_write(hx, hy, sx, sy)
    print()
    print("  CORRELATION WITH ATTITUDE (old original evidence) raw -> stabilized")
    r_hy = abs(correlation(hy, pitch)); r_sy = abs(correlation(sy, pitch))
    r_hx = abs(correlation(hx, roll));  r_sx = abs(correlation(sx, roll))
    print(f"     |corr(vertical error , pitch)| : {r_hy:5.3f} -> {r_sy:5.3f}")
    print(f"     |corr(horizontal error , roll )| : {r_hx:5.3f} -> {r_sx:5.3f}")
    print()
    if aim:
        print(f"  aim: initial {aim[0]:+.2f} -> final {aim[-1]:+.2f} deg "
              f"(trim {aim[-1] - aim[0]:+.2f} deg oynatti)")
        print()
    vertical_ok = (r_sy < 0.5 * r_hy and std(sy) < std(hy))
    print("RESULT:", "GIMBAL WORKS ON THE VERTICAL AXIS" if vertical_ok
          else "NO EXPECTED IMPROVEMENT IN THE VERTICAL AXIS")
    print(f"  (horizontal roll correlation {r_hx:.3f} -> {r_sx:.3f} is for informational purposes only: "
          f"endogenous in bank-to-turn, not a criterion)")
    code_value = 0 if vertical_ok else 1
    if ground_truth_orientation:
        horizontal_ok, _ = horizontal_ground_truth(ground_truth_orientation, tk, sx, roll)
        if not horizontal_ok:
            code_value = 1
    return code_value


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv')
    p.add_argument('--ground-truth-orientation', metavar='CSV',
                   help='Gazebo ground truth (ground_truth_orientation.csv). If given horizontally '
                        'The channel also falls into the criteria.')
    p.add_argument('--historical', action='store_true',
                   help='Runs OLD criteria even with tilt columns')
    a = p.parse_args()
    rows_value = read_value(a.csv)
    if len(rows_value) < 50:
        raise SystemExit(f"insufficient sample: {len(rows_value)}")

    tilt_var = all(k in rows_value[0] for k in
                   ('tilt_cmd_deg', 'tilt_status_deg', 'joint_deg'))
    if tilt_var and not a.historical:
        return criterion_gimbal(rows_value, a.ground_truth_orientation)
    return criterion_historical(rows_value, a.ground_truth_orientation)


if __name__ == '__main__':
    sys.exit(main())
