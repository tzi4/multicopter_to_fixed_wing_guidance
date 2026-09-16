#!/usr/bin/env python3
"""compare_results.py - juxtaposes VISUAL guidance methods with the same criterion.

WHY A SEPARATE TOOL: trial_summary.py POSITIONED phase summaries ("did he get close, did he enter the
frame"). The question asked here is different: AFTER the handoff, which method kept the target
better centered and made it bigger faster? Three methods (LOS/PID/MPC) wrote their own code, made
their own tune; The comparison should be made from one source and with the SAME definitions so that
the choice is based on the result, not the code.

PRIZE DESCRIPTION (user-imposed goal, 2026-08-04: LINEAR, no square roots): 1. PRIMARY: bbox FIELD
(w*h, px^2) keep growing -> collision. Second awardee's GROWTH RATE (first derivative ~ proxy for
approach speed). area_root is stored in CSV (compatibility with past runs); where it is converted to
linear as area_root^2. 2. Keep the target at the center: |ex|,|ey| RMS (in virtual frame, degree). 3.
SECONDARY: how fast. Criterion: time to closest range after handoff. Crash evidence: min range +
vibration jump.

USAGE:
    python3 tools/compare_results.py                     # all video runs
    python3 tools/compare_results.py --method-value pid los     # only those mentioned
    python3 tools/compare_results.py --csv report.csv     # machine readable output
"""

import argparse
import csv
import math
import re
from pathlib import Path

ROOT_VALUE = Path(__file__).resolve().parent.parent
LOG_DIRECTORY = ROOT_VALUE / 'guidance_allstar' / 'logs'
# display_<method>_<stamp>.csv
LABEL = re.compile(r'visual_([a-z0-9]+)_(\d{8}_\d{6})\.csv$')


def _f(row_value, key_value):
    raw_value = row_value.get(key_value, '')
    if raw_value in ('', None):
        return None
    try:
        return float(raw_value)
    except ValueError:
        return None


def _rms(values_item):
    valid_value = [d for d in values_item if d is not None]
    if not valid_value:
        return None
    return math.sqrt(sum(d * d for d in valid_value) / len(valid_value))


def run_analysis(path_value):
    """Read a video CSV, extract the reward criteria."""
    with open(path_value) as f:
        rows_value = list(csv.DictReader(f))
    if not rows_value:
        return None

    m = LABEL.search(path_value.name)
    method_value, stamp_value = (m.group(1), m.group(2)) if m else ('?', '?')

    # handoff moments: gap at t greater than 1 s = authority reassigned. (The loop writes only when in
    # authority; space means 'we returned to position'.)
    handoffs = []
    previous_t = None
    for s in rows_value:
        t = _f(s, 't')
        if t is None:
            continue
        if previous_t is None or t - previous_t > 1.0:
            handoffs.append(s)
        previous_t = t

    area_value = [_f(s, 'area_root') for s in rows_value]
    area_valid = [a for a in area_value if a is not None]

    # LINEAR field growth rate [px^2/s]: from consecutive valid detection pairs, those whose detection gap
    # exceeds 1 s are discarded (the derivative becomes meaningless).
    growth = []
    previous_area = previous_area_t = None
    for s in rows_value:
        a, t = _f(s, 'area_root'), _f(s, 't')
        if a is None or t is None:
            continue
        if previous_area is not None and 0 < t - previous_area_t <= 1.0:
            growth.append((a * a - previous_area * previous_area)
                          / (t - previous_area_t))
        previous_area, previous_area_t = a, t
    growth.sort()
    range_value = [_f(s, 'range_m_value') for s in rows_value]
    range_valid = [r for r in range_value if r is not None]
    vibe = [_f(s, 'vibe_max') for s in rows_value]
    vibe_valid = [v for v in vibe if v is not None]
    # CONTEXT of the Vibe top: if the bounce is at the bottom of the target it is evidence of impact, if
    # the target is away + altitude ~0 it is GROUND CONTACT (this is how three runs were misread on
    # 2026-08-04). The range and altitude of the peak moment are recorded.
    vibe_range = vibe_pos_z = None
    if vibe_valid:
        peak = max(vibe_valid)
        for s in rows_value:
            if _f(s, 'vibe_max') == peak:
                vibe_range = _f(s, 'range_m_value')
                vibe_pos_z = _f(s, 'pos_z')
                break

    # Centering error WITH detection (error undefined without bbox).
    ex = [_f(s, 'ex_deg') for s in rows_value]
    ey = [_f(s, 'ey_deg') for s in rows_value]

    # Speed: time to closest range after first handoff.
    duration_en_near = None
    if range_valid and handoffs:
        t0 = _f(handoffs[0], 't')
        best_t, best_r = None, None
        for s in rows_value:
            r, t = _f(s, 'range_m_value'), _f(s, 't')
            if r is None or t is None or t < (t0 or 0):
                continue
            if best_r is None or r < best_r:
                best_r, best_t = r, t
        if best_t is not None and t0 is not None:
            duration_en_near = best_t - t0

    # handoff jump: instruction size step at the first 2 s after handoff.
    jump_value = None
    if handoffs:
        t0 = _f(handoffs[0], 't')
        magnitude, previous_value = [], None
        for s in rows_value:
            t = _f(s, 't')
            if t is None or t0 is None or not (t0 <= t <= t0 + 2.0):
                continue
            v = [_f(s, k) for k in ('cmd_vx', 'cmd_vy', 'cmd_vz')]
            if any(x is None for x in v):
                continue
            n = math.sqrt(sum(x * x for x in v))
            if previous_value is not None:
                magnitude.append(abs(n - previous_value))
            previous_value = n
        if magnitude:
            jump_value = max(magnitude)

    dt = [_f(s, 'dt') for s in rows_value]
    dt_valid = sorted(d for d in dt if d)

    # HIT PROVISION ( 2026 - 08 - 04 ): The answer to the question "did we hit" at a glance. CAUTION --
    # vibe is NOT EVIDENCE BY ITSELF: Gazebo DOES NOT MODEL CONTACT between two SITL vehicles, meaning
    # that even in an actual impact, vibration does not bounce (measured: vibe 0.92 per m passage 0.9 ).
    # On the other hand, when hitting the GROUND, the vibe jumps to 150 - 345. So vibe's job is to
    # distinguish GROUND CONTACT, not impact. Verdict: range threshold + verification that altitude is NOT
    # on the ground.
    on_ground = (vibe_pos_z is not None and vibe_pos_z > -5.0)
    if range_valid:
        mn = min(range_valid)
        if on_ground and (vibe_max_deg := max(vibe_valid) if vibe_valid else 0) > 50:
            verdict_value = 'GROUND'          # ground contact -- NOT impact
        elif mn <= 3.0:
            verdict_value = 'IMPACT'
        elif mn <= 8.0:
            verdict_value = 'NEAR'
        else:
            verdict_value = 'MISS'
    else:
        verdict_value = '-'

    return {
        'method_value': method_value,
        'stamp_value': stamp_value,
        'verdict_value': verdict_value,
        'file_value': path_value.name,
        'sample_value': len(rows_value),
        'handoff_count': len(handoffs),
        'handoff_range': _f(handoffs[0], 'range_m_value') if handoffs else None,
        'area_max_px2': (max(area_valid) ** 2) if area_valid else None,
        'area_speed_p90': (growth[int(len(growth) * 0.9)] if growth else None),
        'range_min': min(range_valid) if range_valid else None,
        'ex_rms': _rms(ex),
        'ey_rms': _rms(ey),
        'ex_peak': max((abs(x) for x in ex if x is not None), default=None),
        'duration_en_near_s': duration_en_near,
        'handoff_sicramasi': jump_value,
        'vibe_max': max(vibe_valid) if vibe_valid else None,
        'vibe_range_m': vibe_range,
        'vibe_pos_z': vibe_pos_z,
        'loop_hz': (1.0 / dt_valid[len(dt_valid) // 2]) if dt_valid else None,
        'detection_ratio': (sum(1 for a in area_value if a is not None) / len(rows_value) * 100),
    }


def _s(value_value, format_value='{:.1f}', empty_value='  -  '):
    return empty_value if value_value is None else format_value.format(value_value)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--method-value', nargs='*', help='only these methods (los pid mpc)')
    p.add_argument('--csv', help='write the result to this file as well')
    p.add_argument('--min-sample', type=int, default=20,
                   help='runs shorter than that many lines are skipped (default 20)')
    a = p.parse_args()

    runs = []
    for path_value in sorted(LOG_DIRECTORY.glob('visual_*.csv')):
        result_value = run_analysis(path_value)
        if result_value is None or result_value['sample_value'] < a.min_sample:
            continue
        if a.method_value and result_value['method_value'] not in a.method_value:
            continue
        runs.append(result_value)

    if not runs:
        print("video run not found "
              f"({LOG_DIRECTORY}/visual_*.csv)")
        return

    print("=" * 108)
    print("VIDEO GUIDANCE COMPARISON  "
          "(primary reward: area_root growth + centering; secondary: duration)")
    print("=" * 108)
    headers_value = (f"{'method_value':7} {'stamp_value':16} {'verdict_value':6} {'handoff':5} {'handoff_m':8} "
                 f"{'min_m':7} {'area_px2':9} {'a_speed90':8} "
                 f"{'ex_rms':7} {'ey_rms':7} "
                 f"{'duration_s_value':7} {'jump_value':8} {'vibe':6} {'Hz':5} {'detection%':7}")
    print(headers_value)
    print("-" * 124)
    for k in sorted(runs, key=lambda x: (x['method_value'], x['stamp_value'])):
        print(f"{k['method_value']:7} {k['stamp_value']:16} {k['verdict_value']:6} {k['handoff_count']:5d} "
              f"{_s(k['handoff_range'], '{:8.1f}'):8} "
              f"{_s(k['range_min'], '{:7.1f}'):7} "
              f"{_s(k['area_max_px2'], '{:9.0f}'):9} "
              f"{_s(k['area_speed_p90'], '{:8.0f}'):8} "
              f"{_s(k['ex_rms'], '{:7.2f}'):7} {_s(k['ey_rms'], '{:7.2f}'):7} "
              f"{_s(k['duration_en_near_s'], '{:7.1f}'):7} "
              f"{_s(k['handoff_sicramasi'], '{:8.2f}'):8} "
              f"{_s(k['vibe_max'], '{:6.1f}'):6} "
              f"{_s(k['loop_hz'], '{:5.1f}'):5} "
              f"{k['detection_ratio']:7.1f}")

    print()
    print("COLUMNS: handoff=number of delegations | handoff_m=range at first revolution")
    print("  verdict: STRIKE(<=3 m) | NEAR(<=8) | MISS | GROUND(=ground contact, NOT impact)")
    print("    CAUTION: Gazebo DOES NOT MODEL CONTACT between two SITL vehicles -- real")
    print("    vibe does NOT spike on contact (vibe 0.9 during a 0.92 m pass). Its role is")
    print("    Distinguishing the GROUND theme (there it jumps to 150 - 345).")
    print("  min_m=closest range seen (old note: vibe bounce;")
    print("        ATTENTION: vibe peak is GROUND CONTACT when target is away + pos_z~0,")
    print("        distinguished from vibe_range_m/vibe_pos_z columns in CSV)")
    print("  area_px2=bbox area peak [px^2, LINEAR] (PRIMARY PRIZE, large=good)")
    print("  a_speed90=p90 of area growth rate [px^2/s] (SECONDARY REWARD: approach speed)")
    print("  ex/ey_rms=centering error (small=good) | duration_s_value=closest after handoff")
    print("  bounce=command size step at handoff (small=soft transition)")

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(runs[0].keys()))
            w.writeheader()
            w.writerows(runs)
        print(f"\n CSV written: {a.csv}")


if __name__ == '__main__':
    main()
