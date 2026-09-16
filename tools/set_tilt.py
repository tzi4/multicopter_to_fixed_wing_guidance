#!/usr/bin/env python3
"""set_tilt.py - GIMBAL BRANCH: Sets the standoff mission geometry (down/back).

WHY NEW TOOL (replacing old tools/set_mounting.py): The camera is no longer FIXED to the body; in a
self-stabilizing single axis (tilt) gimbal. Dependency direction REVERSED:

  OLD WORLD: mounting angle (SDF pose) fixed -> standoff DOWN has to comply with it
               (down = back*tan(mount+trim))
  NEW WORLD: DOWN/BACK free MISSION DESIGN button -> camera angle is derived from it: YILDIZ_TILT =
  atan(DOWN / BACK) [world elevation]

That's why this tool writes to ONE FILE ONLY: scripts/standoff_geom.sh.

*** NO VEHICLE WILL WRITE TO THE SENSOR POS SDF *** The pose pitch of the "glass" sensor in
models/swarm_drone_*/model.sdf SHOULD REMAIN 0. The camera is now welded to the gimbal tilt link;
typing an angle there superimposes a QUIET offset on top of the world elevation commanded by the
gimbal (the gimbal stabilizes itself to eps, the actual axis becomes eps+offset; the measurement
chain cannot tell the difference). That's why the SDF and yildizlar_gimbal writing paths of the old
vehicle have been DELETED; this tool only VERIFIES them (WARNING if it is not 0).

USAGE:
    python3 tools/set_tilt.py --display-value                  # current status + report
    python3 tools/set_tilt.py --down 6 --back 25        # dry run (NOT WRITING)
    python3 tools/set_tilt.py --down 6 --back 25 --apply-value
    python3 tools/set_tilt.py --restore-backup                 # restore from backup

Nothing is written unless --apply-value is supplied.
"""

import argparse
import math
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT_VALUE = Path(__file__).resolve().parent.parent
SDF_PATTERN = 'models/swarm_drone_*/model.sdf'
STANDOFF = ROOT_VALUE / 'scripts/standoff_geom.sh'
FALLBACK = ROOT_VALUE / 'scripts/standoff_geom.sh.fallback_value'

# IMAGE 'cam_link' also carries the same <pose> pattern (1.5707 = for tilting the cylinder, NOTHING to
# do with the camera). That's why the pattern is Anchored to the <sensor name="cam"> tag. (Preserved
# verbatim from old vehicle; here for READING ONLY.)
POSE = re.compile(r'(<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+)'
                  r'(-?[\d.]+)(\s+[-\d.]+\s*</pose>)')

# Camera: 1280x720, hfov 66 deg (IMX500). Vertical SEMI-FOV:
HFOV_RAD = 1.1519
GEN, ELEVATION = 1280.0, 720.0
HALF_VFOV = math.degrees(math.atan(math.tan(HFOV_RAD / 2.0) * ELEVATION / GEN))


def sdf_files():
    return sorted(ROOT_VALUE.glob(SDF_PATTERN))


def sdf_read(path_value):
    """Pose pitch of sensor 'glass' SDF [deg, UP +]. If not found None."""
    m = POSE.search(path_value.read_text())
    if not m:
        return None
    return -math.degrees(float(m.group(2)))       # pose NEGATIVE = look up


def standoff_values():
    """standoff_geom SOURCES .sh and reads the derived values.

    Instead of regexing out the defaults in the file, we run the shell: The YILDIZ_TILT derivation
    (and legacy-derived branch) lives there, it's the only source. External STAR_* crushes are
    cleared, otherwise the 'current state' will be the state of the shell, not the file.
    """
    env = {k: v for k, v in os.environ.items() if not k.startswith('YILDIZ_')}
    r = subprocess.run(
        ['bash', '-c',
         f'source "{STANDOFF}"; '
         'echo "$YILDIZ_BACK|$YILDIZ_DOWN|$YILDIZ_TILT|$YILDIZ_MOUNT|$YILDIZ_PITCH_TRIM"'],
        cwd=ROOT_VALUE, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise SystemExit(f"could not source standoff_geom.sh:\n{r.stderr}")
    row_value = [s for s in r.stdout.strip().splitlines() if '|' in s][-1]
    back, down, tilt, mount, trim = row_value.split('|')
    m = re.search(r'YILDIZ_DESIGN_DOWN=([\d.]+)', STANDOFF.read_text())
    return {'back': float(back), 'down': float(down), 'tilt': float(tilt),
            'mount': float(mount), 'trim': float(trim),
            'design_down': float(m.group(1)) if m else None}


def state_write():
    d = standoff_values()
    print("--- CURRENT STATUS (scripts/standoff_geom.sh) ---")
    print(f"  YILDIZ_BACK               {d['back']:g} m")
    print(f"  YILDIZ_DESIGN_DOWN       {d['design_down']:g} m")
    print(f"  YILDIZ_DOWN (active) {d['down']:g} m")
    print(f"  YILDIZ_TILT (derived) {d['tilt']:+.2f} deg   "
          f"= atan(down/back)")
    print(f"  YILDIZ_MOUNT / PITCH_TRIM {d['mount']:g} / {d['trim']:g} deg   "
          f"(NO FUNCTIONAL in vertical channel on gimbal branch)")
    print()
    print("--- SDF SENSOR POS (should not be written, it should be 0) ---")
    dirty = []
    for path_value in sdf_files():
        v = sdf_read(path_value)
        label_item = str(path_value.relative_to(ROOT_VALUE))
        if v is None:
            print(f"  {label_item:34s} 'cam' sensor not found (?)")
            dirty.append(label_item)
        else:
            print(f"  {label_item:34s} {v:+.4f} deg" + ('' if abs(v) < 1e-3 else '   <<< NOT ZERO'))
            if abs(v) >= 1e-3:
                dirty.append(label_item)
    if dirty:
        print()
        print("  *** WARNING: SDF 'glass' exposure is not zero. camera gimbal tilt")
        print("  Sourced from *** link; The angle here is the world elevation of the gimbal")
        print("  Superimposes a silent offset ON TOP of the *** command. Reset:")
        print("  ***   git checkout -- " + ' '.join(dirty))
    return d, dirty


def report_value(down, back):
    """Vertical framing budget on terminal approach.

    In Standoff, the hopper is BACK behind the target and DOWN below it. When approaching,
    horizontal separation closes but DOWN is maintained -> target's line of sight rise eps(r) =
    hang(DOWN / r) GROWS RAPIDLY. If the camera tilt is FREEZED at the design value standoff
    (atan(DOWN/BACK)), the target extends over the top of the frame after a range. This is the range
    that Phase C (terminal tilt command) must take over.
    """
    tilt = math.degrees(math.atan2(down, max(back, 1e-9)))
    print(f"--- GEOMETRY (down {down:g} m / back {back:g} m) ---")
    print(f"  YILDIZ_TILT = atan(down/back)   {tilt:+.2f} deg")
    print(f"  vertical SEMI-FOV {HALF_VFOV:.2f} deg "
          f"(hfov {math.degrees(HFOV_RAD):.0f} deg, {GEN:.0f}x{ELEVATION:.0f})")
    print()
    print("--- TERMINAL FRAMING BUDGET ---")
    s = math.sin(math.radians(HALF_VFOV))
    r_axis = down / s if s > 0 else float('inf')
    print(f"  Range where eps(r) = hang(down/r) -- eps > {HALF_VFOV:.2f} deg: "
          f"r < {r_axis:.1f} m")
    print(f"    (out-of-frame range if tilt is frozen at 0)")
    s2 = math.sin(math.radians(min(89.9, tilt + HALF_VFOV)))
    r_tilt = down / s2 if s2 > 0 else float('inf')
    print(f"  tilt {tilt:+.2f} deg'de dondurulursa (eps - tilt > {HALF_VFOV:.2f}): "
          f"r < {r_tilt:.1f} m")
    print(f"  ==> PHASE O (terminal tilt command) must take over at {r_tilt:.1f} m AT THE LATEST;")
    print(f"      ~{r_tilt * 1.5:.0f} m is recommended for safe stake.")
    print()
    print("  range eps=hang(down/r) tilt-eps framing (tilt facing)")
    for r in range(10, 101, 10):
        if r <= down:
            print(f"  {r:5d} m {'--':>10s} {'--':>8s} target JUST BELOW")
            continue
        eps = math.degrees(math.asin(min(1.0, down / r)))
        difference = tilt - eps
        state_value = 'within' if abs(difference) <= HALF_VFOV else 'OUTSIDE'
        print(f"  {r:5d} m {eps:10.2f} deg {difference:+8.2f} {state_value}")
    print()
    print("  NOTE: 'tilt-eps' is the vertical deviation of the target from the optical axis; |difference| >")
    print(f"  When {HALF_VFOV:.2f} becomes deg, the target is PHYSICALLY out of frame "
          f"(yazilim gimbali kurtaramaz).")


def write_value(down, back):
    """ALONE scripts/standoff_geom.sh. SDF and yildizlar_gimbal CANNOT be touched."""
    text_value = STANDOFF.read_text()
    FALLBACK.write_text(text_value)
    new_value, n1 = re.subn(r'(YILDIZ_DESIGN_DOWN=)[\d.]+', rf'\g<1>{down:g}',
                       text_value, count=1)
    if n1 != 1:
        raise SystemExit("ERROR: line YILDIZ_DESIGN_DOWN not found")
    new_value, n2 = re.subn(r'(YILDIZ_BACK="\$\{YILDIZ_BACK:-)[\d.]+(\}")',
                       rf'\g<1>{back:g}\g<2>', new_value, count=1)
    if n2 != 1:
        raise SystemExit("ERROR: YILDIZ_BACK default line not found")
    STANDOFF.write_text(new_value)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--display-value', action='store_true',
                   help='write current status + report, exit')
    p.add_argument('--down', type=float, help='standoff vertical offset [m]')
    p.add_argument('--back', type=float, help='standoff back distance [m]')
    p.add_argument('--apply-value', action='store_true',
                   help='ACTUALLY write (scripts/standoff_geom.sh only)')
    p.add_argument('--restore-backup', action='store_true',
                   help='Restore from backup (or with git checkout)')
    a = p.parse_args()

    if a.restore_backup:
        if FALLBACK.exists():
            STANDOFF.write_text(FALLBACK.read_text())
            FALLBACK.unlink()
            print(f"rolled back (backup): {STANDOFF.relative_to(ROOT_VALUE)}")
        else:
            subprocess.run(['git', 'checkout', '--', 'scripts/standoff_geom.sh'],
                           cwd=ROOT_VALUE, check=True)
            print("reverted (git checkout): scripts/standoff_geom.sh")
        return 0

    d, dirty = state_write()
    print()

    if a.down is None and a.back is None:
        report_value(d['down'], d['back'])
        if not a.display_value:
            print("\n(--down/--back not given; only status shown)")
        return 1 if dirty else 0

    down = a.down if a.down is not None else d['down']
    back = a.back if a.back is not None else d['back']
    if down <= 0 or back <= 0:
        raise SystemExit("ERROR: down and back must be POSITIVE (copter "
                         "(Sits BEHIND and BELOW)")
    report_value(down, back)

    if not a.apply_value:
        print("\n(dry running --add --apply-value to write)")
        return 0

    write_value(down, back)
    print(f"\n WRITTEN: scripts/ standoff_geom .sh "
          f"(backup: {FALLBACK.relative_to(ROOT_VALUE)}; for recovery --restore-backup)")
    new_value = standoff_values()
    print(f"  verification: BACK {new_value['back']:g} DOWN {new_value['down']:g}  "
          f"TILT {new_value['tilt']:+.2f} deg")
    if abs(new_value['down'] - down) > 1e-6 or abs(new_value['back'] - back) > 1e-6:
        print("*** WARNING: sourced values do not match the values written "
              "(YILDIZ_* environment override or legacy configuration active?) ***")
        return 1
    print("\nNext step: A/B run (SDF untouched, no recompilation)\n"
          "  DURATION=360 PLAN=missions/target_ellipse.plan tools/scenario.sh\n"
          "Criterion: detection rate must be maintained; in the terminal band (Phase C above \n"
          "below range) the framing loss should not increase.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
