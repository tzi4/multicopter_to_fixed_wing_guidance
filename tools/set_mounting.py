#!/usr/bin/env python3
"""set_mounting.py - HISTORICAL: use tools/set_tilt.py in gimbal branch.

*** HISTORICAL VEHICLE -- WRITING PATH OFF (gimbal branch, 2026-08-05) *** This vehicle belongs to
the world where the camera is FIXED TO THE BODY. In the gimbal branch, the camera is a
self-stabilizing single axis (tilt) gimbal: there is NO adjustment button for the mounting angle,
the camera angle is derived from the standoff geometry (YILDIZ_TILT = atan(DOWN/BACK)) and is
commanded at runtime.

Particularly DANGEROUS: this vehicle writes to the 'glass' sensor pose SDF. The camera is now welded
to the gimbal tilt link; typing an angle there superimposes a QUIET offset on top of the world
elevation commanded by the gimbal (the gimbal stabilizes itself to eps, the actual axis becomes
eps+offset, the measurement chain cannot see the difference). So paths --apply-value (and --restore-backup) are
DISABLED; --display-value works.

  NEW TOOL: python3 tools/set_tilt.py --display-value
              python3 tools/set_tilt.py --down 6 --back 25 [--apply-value]

Below is the old world document (applies to body-mounted camera arms):

WHY THE TOOL WAS NEEDED: the mounting angle is NOT a single adjustment knob. If it occurs in three
places and one is separated from the other, it silently works incorrectly:

  1. PHYSICAL SOURCE models/swarm_drone_*/model.sdf, pitch in <pose> of the "glass" sensor (radians,
  UPWARD NEGATIVE: -0.5235988 = +30 deg). This is the real camera angle of the simulation. 2.
  VIRTUAL GIMBAL yildizlar_gimbal.VirtualGimbal(mount_phys_pitch_deg=...) -- yildizlar_guidance.sh passes
  this with YILDIZ_MOUNT. It should be EXACTLY the same as SDF; If it separates, de-rotation (body
  swing cleaning) will be incorrect. 3. STANDOFF scripts/standoff_geom.sh: DOWN =
  BACK*tan(MOUNT+TRIM). This ensures that the target stays on the camera AXIS (down=3 in the
  attempted run, the target escaped from the axis 22 deg and the detection dropped to %5).

RULE OF FUNDAMENTAL ( 2026 - 08 - 04 , IRL drift analysis + static target test): camera axis = mount
+ body pitch mount should be equal to the target LOS elevation AT THE TIME OF IMPACT. * In real
equipment body upright nose- down in terminal dash (~- 34 deg on 18 m/s ; tan(- pitch )= 0.00212 *
V_hava ^ 2 ) -> from the rear co-altitude impact mount = terminal pitch size. * In SIMULATION,
copter drag is negligible (measured pitch - 1.1 deg at 20 - 25 m/s ), so simulated mounting angle
approximately equals standoff LOS elevation. The simulated mounting angle therefore does not
validate the hardware mounting angle. They are separate decisions.

USAGE:
    python3 tools/set_mounting.py --display-value                 # current situation
    python3 tools/set_mounting.py --mount 14 --back 40     # derive DOWN
    python3 tools/set_mounting.py --mount 14 --back 40 --apply-value
    python3 tools/set_mounting.py --restore-backup                # with git checkout

If --apply-value is not given, NOTHING IS WRITTEN, only what will happen is shown. Once applied, the
yildizlar_gimbal static test is run automatically.
"""

import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

ROOT_VALUE = Path(__file__).resolve().parent.parent
SDF_PATTERN = 'models/swarm_drone_*/model.sdf'
GIMBAL = ROOT_VALUE / 'yildizlar_gimbal.py'
STANDOFF = ROOT_VALUE / 'scripts/standoff_geom.sh'
# ATTENTION: IMAGE 'cam_link' also carries the same <pose> pattern (1.5707 = for tilting the cylinder
# body, NOTHING to do with the camera). That's why the pattern is Anchored to the <sensor name="cam">
# tag; Only the first pose following it is changed. (Dry running the tool caught this bug: the first
# version read -89.99 deg.)
POSE = re.compile(r'(<sensor\s+name="cam"\s+type="camera"\s*>\s*<pose>'
                  r'\s*[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+)'
                  r'(-?[\d.]+)(\s+[-\d.]+\s*</pose>)')


def sdf_files():
    return sorted(ROOT_VALUE.glob(SDF_PATTERN))


def sdf_read(path_value):
    m = POSE.search(path_value.read_text())
    if not m:
        return None
    return -math.degrees(float(m.group(2)))       # pose NEGATIVE = look up


def current_state():
    d = {}
    for path_value in sdf_files():
        d[path_value.relative_to(ROOT_VALUE)] = sdf_read(path_value)
    m = re.search(r'mount_phys_pitch_deg=([\d.]+)', GIMBAL.read_text())
    d['yildizlar_gimbal .py (default)'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_MOUNT="\$\{YILDIZ_MOUNT:-([\d.]+)\}"', STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_MOUNT'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_BACK="\$\{YILDIZ_BACK:-([\d.]+)\}"', STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_BACK'] = float(m.group(1)) if m else None
    m = re.search(r'YILDIZ_PITCH_TRIM="\$\{YILDIZ_PITCH_TRIM:-(-?[\d.]+)\}"',
                  STANDOFF.read_text())
    d['standoff_geom.sh YILDIZ_PITCH_TRIM'] = float(m.group(1)) if m else None
    return d


def write_value(mount, back, trim, down=None):
    """If down is given, PERMANENT is written to standoff_geom.sh.

    BUG (2026-08-04, caught by developer agent): old version was printing --down only to the screen,
    not writing to file. Result: when the assembly was moved to 0, the standoff_geom.sh derivation
    gave DOWN=-1 and in every run where DOWN= env was not given, the hopper was positioned ABOVE the
    target.
    """
    if down is None:
        down = round(back * math.tan(math.radians(mount + trim)))
    rad = -math.radians(mount)
    for path_value in sdf_files():
        text_value = path_value.read_text()
        new_value, n = POSE.subn(lambda m: f"{m.group(1)}{rad:.7f}{m.group(3)}", text_value)
        if n != 1:
            raise SystemExit(f"ERROR: pose line 'cam' in {path_value} matched times {n}")
        path_value.write_text(new_value)
    g = GIMBAL.read_text()
    GIMBAL.write_text(re.sub(r'mount_phys_pitch_deg=[\d.]+',
                             f'mount_phys_pitch_deg={float(mount)}', g, count=1))
    s = STANDOFF.read_text()
    s = re.sub(r'(YILDIZ_MOUNT="\$\{YILDIZ_MOUNT:-)[\d.]+(\}")',
               rf'\g<1>{mount:g}\g<2>', s, count=1)
    s = re.sub(r'(YILDIZ_BACK="\$\{YILDIZ_BACK:-)[\d.]+(\}")',
               rf'\g<1>{back:g}\g<2>', s, count=1)
    # Write the design DOWN to the derivation-fail branch PERMANENT (associatestring above).
    s = re.sub(r"(YILDIZ_DESIGN_DOWN=)[\d.]+", rf"\g<1>{down:g}", s, count=1)
    STANDOFF.write_text(s)
    return down


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--display-value', action='store_true', help='write current status, exit')
    p.add_argument('--mount', type=float, help='new mounting angle [deg, UP +]')
    p.add_argument('--back', type=float, help='standoff back distance [m]')
    p.add_argument('--trim', type=float, default=None,
                   help='typical body pitch [deg]; If not given, existing data is preserved')
    p.add_argument('--down', type=float, default=None,
                   help='standoff vertical offset [m] -- MANUAL ver. In GIMBALL setup '
                        'mandatory: because the gimbal separates the camera angle from the body '
                        'The derivation "down = back*tan(mount+trim)" is INVALID '
                        '(When mount=0, the derivation gives negative down, i.e. '
                        'puts it ON TOP of the target).')
    p.add_argument('--apply-value', action='store_true', help='REALLY write')
    p.add_argument('--restore-backup', action='store_true',
                   help='restore all mounting files with git checkout')
    a = p.parse_args()

    # HISTORICAL LOCK (gimbal branch): write path is completely closed. Justification in the module
    # docstring: Writing to pose SDF imposes a silent offset on the stabilized gimbal.
    if a.apply_value or a.restore_backup:
        print(__doc__.split('Down in the old world')[0].strip())
        print("\n*** NO ACTION TAKEN. Use tools/set_tilt.py. ***")
        return

    if a.restore_backup:
        targets_value = [str(y.relative_to(ROOT_VALUE)) for y in sdf_files()]
        targets_value += ['yildizlar_gimbal.py', 'scripts/standoff_geom.sh']
        subprocess.run(['git', 'checkout', '--'] + targets_value, cwd=ROOT_VALUE, check=True)
        print("withdrawn:", ', '.join(targets_value))
        return

    state_value = current_state()
    print("--- THE CURRENT SITUATION ---")
    for k, v in state_value.items():
        print(f"  {str(k):42s} {v}")
    sdf_values = {v for k, v in state_value.items() if str(k).startswith('models/')}
    if len(sdf_values) > 1:
        print("  *** WARNING: SDF files ARE DECOMPOSITED ***")
    if a.display_value or a.mount is None:
        if a.mount is None and not a.display_value:
            print("\n--mount not issued; only the situation was shown.")
        return

    back = a.back if a.back is not None else state_value['standoff_geom.sh YILDIZ_BACK']
    trim = a.trim if a.trim is not None else state_value['standoff_geom.sh YILDIZ_PITCH_TRIM']
    if a.down is not None:
        down = a.down
        derivation = "MANUAL (installation with gimbal: derivation invalid)"
    else:
        down = round(back * math.tan(math.radians(a.mount + trim)))
        derivation = "derived: back*tan(mount+trim)"
        if down <= 0:
            print(f"\n*** WARNING: derived down={down} <= 0, i.e. hopper target "
                  f"flies ABOVE it. Set --down explicitly for a gimbal installation. ***")
    los = math.degrees(math.atan(down / back)) if back else 0.0
    vh = math.degrees(2 * math.atan(math.tan(math.radians(66) / 2) * 720 / 1280)) / 2

    print("\n--- NEW GEOMETRY ---")
    print(f"  assembly {a.mount:+.2f} deg ( SDF pose {-math.radians(a.mount):+.7f} rad )")
    print(f"  back / down           {back:g} m / {down:g} m  (trim {trim:+g}) [{derivation}]")
    print(f"  standoff LOS elevation {los:+.2f} deg")
    print(f"  vertical semi-FOV {vh:.2f} deg")
    print(f"  Target deviation from axis in standoff {los - (a.mount + trim):+.2f} deg")
    print(f"  Deviation IN REAR IMPACT ( LOS -> 0 ) {-(a.mount + trim):+.2f} deg  "
          f"-> {'FRAMING OUTSIDE' if abs(a.mount + trim) > vh else 'inside the image'}")

    if not a.apply_value:
        print("\n(dry running --add --apply-value to write)")
        return

    down = write_value(a.mount, back, trim, a.down)
    print(f"\nWRITTEN. Validation is running...")
    r = subprocess.run([sys.executable, str(GIMBAL), '--test'],
                       cwd=ROOT_VALUE, capture_output=True, text=True)
    print(r.stdout.strip().splitlines()[-1] if r.stdout else r.stderr[-400:])
    if r.returncode != 0:
        print("*** STATIC TEST FAILED -- 'python3 tools/set_mounting.py "
              "Get it back with '--restore-backup' ***")
        raise SystemExit(1)
    print(f"\n Next step: A/B run \n"
          f"  DURATION=360 VIEW=\"mpc_guidance.py\" PLAN=missions/target_ellipse.plan "
          f"tools/scenario.sh\n"
          f"Criteria: static test PASS (above) + detection rate must be maintained + "
          f"< 15 detection in the m band should increase from 0.")


if __name__ == '__main__':
    main()
