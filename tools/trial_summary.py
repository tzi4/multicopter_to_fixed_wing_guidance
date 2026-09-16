#!/usr/bin/env python3
"""Summarizes the guidance.log + bbox.log output of an experiment on a single page.

Purpose: After each trial, the answer to the question "did it get close, did it enter the frame?" is
given in NUMBER, not by eye decision.
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
SUMMARY = re.compile(r'frame=(\d+)\s+fps=([0-9.]+)\s+detection_ratio=%([0-9.]+)')


def read_value(path):
    if not path.exists():
        return []
    return ANSI.sub('', path.read_text(errors='replace')).splitlines()



def _mounting_read():
    """FROZEN ROAD. Camera STATIC mounting angle [deg, up +].

    GIMBAL BRANCH (2026-08-05): on the camera physical tilt gimbal, the static mount on model.sdf is
    now ALL 0. This reader is for --no-tilt (body-fixed) runs only, as a last resort. _tilt_read()
    for live axis.
    """
    v = os.environ.get('YILDIZ_MOUNT')
    if v:
        try:
            return float(v)
        except ValueError:
            pass
    sdf = Path(__file__).resolve().parent.parent / "models" / "swarm_drone_1" / "model.sdf"
    m = re.search(r'<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+(-?[\d.]+)',
                  sdf.read_text()) if sdf.exists() else None
    return -math.degrees(float(m.group(1))) if m else 0.0


_TILT_TRIAL_DIR = None   # main() populates: test folder (source bbox.log)


def _tilt_read(root_value=None):
    """Camera axis = GIMBAL TILT [deg, world elevation, up +].

    PHASE O (2026-08-06): tilt is now DYNAMIC (bbox tracks target ascension), so BEFORE the MEASURED
    median of the run (tilt_deg on guidance CSV / display CSV tilt_status_deg) is tried; env values
    ​​are just the initial/re-acquisition setpoint and fallback: measured median -> $YILDIZ_TILT ->
    atan($YILDIZ_DOWN/$YILDIZ_BACK).
    """
    measured = _tilt_measured_median(root_value, trial_dir=_TILT_TRIAL_DIR) \
        if root_value is not None else None
    if measured is not None:
        return measured, 'measured median (dynamic tilt)'
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


def _tilt_measured_median(root_value, trial_dir=None):
    """MEASURED tilt median of the run [deg] or None.

    Source order: 1. trial_dir/bbox.log 'tilt=+cmd/+statuschange' in SUMMARY lines (YES in every
    run; status changes in every frame in Phase O) 2. displayed_*.csv 'tilt_deg' column (while
    steering) 3. 'tilt_status_deg' in gimbal-log CSVs
    """
    import csv
    import glob
    import re
    if trial_dir is not None:
        try:
            m = re.findall(r'tilt=[+-][\d.]+/([+-][\d.]+)deg',
                           (Path(trial_dir) / 'bbox.log').read_text(
                               errors='replace'))
            vals = sorted(float(x) for x in m)
            if vals:
                return vals[len(vals) // 2]
        except OSError:
            pass
    for pattern_value, column_item in (('visual_*.csv', 'tilt_deg'),
                         ('*.csv', 'tilt_status_deg')):
        for path_value in sorted(glob.glob(str(root_value / pattern_value)),
                          key=os.path.getmtime, reverse=True):
            try:
                vals = []
                for r in csv.DictReader(open(path_value)):
                    try:
                        vals.append(float(r[column_item]))
                    except (TypeError, ValueError, KeyError):
                        continue
                if vals:
                    vals.sort()
                    return vals[len(vals) // 2]
            except OSError:
                continue
    return None


def gimbal_analysis():
    """GEOMETRY report from my latest homing CSV.

    The only comparison that determines whether the target remains in the frame is: camera axis =
    GIMBAL TILT (earth elevation, + = up) location of the target = elevation angle relative to the
    hunter DIFFERENCE OF THE TWO vertical half-angle of the frame (720 px / 40.13 deg -> +-20.07) If
    it exceeds it, the target is physically out of frame.

    GIMBAL BRANCH (2026-08-05): axis IS NO LONGER "mount + body pitch". The camera is in a
    self-stabilizing physical single-axis tilt gimbal; While the body was swinging pitch +-35 deg,
    the camera world pitch was measured at max 0.65 deg, so pitch does not enter the equation. The
    old mount+pitch path only works for --no-tilt (frozen body-fixed) runs as a fallback if the tilt
    is unknown.
    """
    import csv
    import glob
    import math
    import os
    root_value = Path(__file__).resolve().parent.parent / 'guidance_allstar' / 'logs'
    # Select the LOCATION LOCATION BY NAME. In the past, 'latest *.csv' was retrieved; Under logs/, there
    # is now display_*/mpc_diagnosis_*/..._event.csv, and since the range_m/meas_z/pitch_deg flight was not
    # found together, the block was SILENTLY returning empty (2026-08-05).
    files_item = sorted(glob.glob(str(root_value / 'guided_follow_*.csv')),
                      key=os.path.getmtime)
    if not files_item:
        return
    rows = list(csv.DictReader(open(files_item[-1])))

    def fl(r, k):
        try:
            return float(r[k])
        except (TypeError, ValueError, KeyError):
            return None

    near_value = [r for r in rows if (fl(r, 'range_m') or 1e9) < 60]
    if not near_value:
        return
    # FIXED WON'T WRITE (2026-08-04): axis changes from run to run, fixed typing was printing WRONG
    # geometry like "camera-axis +30, target-axis -17.4". GIMBAL BRANCH (2026-08-05): the sole source of
    # the axis is now TILT.
    TILT_DEG, TILT_SOURCE = _tilt_read(root_value)
    MOUNTING_DEG = _mounting_read()
    HALF_FOV_DEG = 20.07       # AI Camera 720p: 40.13 deg vertical FOV / 2

    pitch = sorted(x for r in near_value if (x := fl(r, 'pitch_deg')) is not None)
    elevation_value = []
    for r in near_value:
        rng, pz, mz = fl(r, 'range_m'), fl(r, 'pursuer_z'), fl(r, 'meas_z')
        if None in (rng, pz, mz) or rng < 3:
            continue
        dz = mz - pz                                   # NED: z down positive
        horizontal = math.sqrt(max(1e-6, rng * rng - dz * dz))
        elevation_value.append(math.degrees(math.atan2(-dz, horizontal)))
    elevation_value.sort()
    if not (pitch and elevation_value):
        return

    def p(v, q):
        return v[min(len(v) - 1, int(q * len(v)))]

    print()
    print("--- GEOMETRY (range < 60 m, from homing CSV) ---")
    print(f"  body pitch (deg) : %5 {p(pitch,.05):+.1f} median "
          f"{p(pitch,.5):+.1f} %95 {p(pitch,.95):+.1f}")
    print(f"  target rise : %5 {p(elevation_value,.05):+.1f} median "
          f"{p(elevation_value,.5):+.1f} %95 {p(elevation_value,.95):+.1f}")
    if TILT_DEG is not None:
        axis_value = TILT_DEG
        axis_text = f"{axis_value:+.1f} deg (gimbal tilt, source: {TILT_SOURCE} )"
    else:
        # Tilt unknown: FROZEN body-fixed path (conditions --no-tilt).
        axis_value = MOUNTING_DEG + p(pitch, .5)
        axis_text = (f"{MOUNTING_DEG:+.0f} (assembly) {p(pitch,.5):+.1f} ( pitch ) "
                       f"={axis_value:+.1f} deg [TILT UNKNOWN -> old "
                       f"body-fixed assumption;  give YILDIZ_TILT]")
    difference = p(elevation_value, .5) - axis_value
    print(f"  camera axis: {axis_text}")
    print(f"  target - axis : {difference:+.1f} deg  "
          f"(framing half-angle +-{HALF_FOV_DEG:.2f} deg)")
    if abs(difference) > HALF_FOV_DEG:
        print(f"  >>> The target is outside the image on average. Correct the vertical geometry:")
        print(f"      (a) Move TILT to {p(elevation_value,.5):+.1f} degrees (median "
              f"elevation); Since tilt = atan(down/back) this is standoff "
              f"means changing the pair")
        print(f"      (b) if the current tilt {axis_value:+.1f} deg is to be maintained, add standoff to it. "
              f"fit: back=B for down = {math.tan(math.radians(axis_value)):.2f}*B")
        print(f"      (source: scripts/standoff_geom.sh -- YILDIZ_DOWN /BACK/TILT)")
    # Pitch oscillation: In the OLD body-mounted camera, it was direct framing release. It does NOT distort
    # the frame on the gimbal branch (the camera measured world pitch to max 0.65 deg), it just tells how
    # much the vehicle is drifting -- it stops for diagnostics.
    oscillation = p(pitch, .95) - p(pitch, .05)
    not_ = ("(with gimbal: not reflected in the frame, only vehicle attitude)"
            if TILT_DEG is not None else "(body-fixed: framing swing)")
    print(f"  pitch oscillation (5-95): {oscillation:.1f} deg "
          f"{'>> ' if oscillation > 2 * HALF_FOV_DEG else '<= '}"
          f"framing height {2 * HALF_FOV_DEG:.1f} deg {not_}")


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: trial_summary.py <trial_klasoru>")
    d = Path(sys.argv[1])
    global _TILT_TRIAL_DIR
    _TILT_TRIAL_DIR = d

    g = read_value(d / 'guidance.log')
    ranges_item = [float(m.group(1)) for line in g for m in [RANGE.search(line)] if m]
    slots_item = [float(m.group(1)) for line in g for m in [SLOT.search(line)] if m]
    events_value = [l.strip() for l in g
               if any(k in l for k in ('MISSION FAILSAFE', 'RECOVERY', 'crash',
                                       'KILL MODE', 'abort', 'ABORT'))]

    b = read_value(d / 'bbox.log')
    boxes_value = [(int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)),
                float(m.group(5))) for line in b for m in [BBOX.search(line)] if m]
    summaries = [(int(m.group(1)), float(m.group(2)), float(m.group(3)))
               for line in b for m in [SUMMARY.search(line)] if m]

    print(f"=== TRIAL: {d.name} ===")
    print()
    print("--- POSITIONED GUIDANCE ---")
    if ranges_item:
        print(f"  loop example: {len(ranges_item)}")
        print(f"  Distance to target: {ranges_item[0]:.0f} m at first -> "
              f"final {ranges_item[-1]:.0f} m (NEAREST {min(ranges_item):.0f} m)")
        if slots_item:
            print(f"  standoff noktasina  : EN NEAR {min(slots_item):.0f} m")
        # How much time the distance has passed in which bands: The single-number answer to the question "is
        # it close?" can be misleading (getting close once and then opening up and staying in the band gives
        # the same "closest" value).
        bands_item = [(0, 50), (50, 100), (100, 200), (200, 500), (500, 10**9)]
        print("  distance band distribution:")
        for lo, hi in bands_item:
            n = sum(1 for r in ranges_item if lo <= r < hi)
            if n:
                upper_value = '+' if hi > 10**8 else str(hi)
                print(f"      {lo:4d}-{upper_value:>5s} m : {n:5d} example (%{100*n/len(ranges_item):.1f})")
    else:
        print("  (guidance loop produced no samples)")
    if events_value:
        print("  events:")
        for o in dict.fromkeys(events_value):
            print(f"      {o}")

    print()
    print("--- CAMERA / BBOX ---")
    if summaries:
        last_frame, _, last_ratio = summaries[-1]
        fps = sum(o[1] for o in summaries) / len(summaries)
        print(f"  processed frame : {last_frame} (avg {fps:.1f} fps)")
        print(f"  cumulative detection: {last_ratio:.1f}")
    if boxes_value:
        widths_item = sorted(k[2] for k in boxes_value)
        coverages = sorted(k[4] for k in boxes_value)
        n = len(boxes_value)
        print(f"  DETECTION COUNT       : {n}")
        print(f"  bbox genisligi (px) : min {widths_item[0]} "
              f"median {widths_item[n // 2]} max {widths_item[-1]}")
        print(f"  horizontal coverage (%) : min {coverages[0]:.2f} "
              f"median {coverages[n // 2]:.2f} max {coverages[-1]:.2f}")
        # Location in the frame: Can the yaw lock hold the target in the center?
        centers_x = sorted(k[0] + k[2] / 2 for k in boxes_value)
        centers_y = sorted(k[1] + k[3] / 2 for k in boxes_value)
        print(f"  framing center x : median {centers_x[n // 2]:.0f} "
              f"(center of frame 640 )")
        print(f"  framing center y : median {centers_y[n // 2]:.0f} "
              f"(center of frame 360 )")
    else:
        print("  DETECTION NUMBER: 0 (target never entered the frame)")

    gimbal_analysis()


if __name__ == '__main__':
    main()
