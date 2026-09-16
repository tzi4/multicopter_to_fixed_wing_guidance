#!/usr/bin/env python3
"""Is the CAMERA TILT COMMANDED BY THE STACK the same as the TILT ASSUMED BY THE SCENARIO?

WHY THERE IS ( 2026 - 08 - 09 , third "silent decomposition" lesson): Standoff vertical geometry
goes to THREE SEPARATE CONSUMERS and they all MUST believe the same number: (a) bbox_to_redis --
gives the physical gimbale tilt COMMAND. This is read ON STACK OPEN ( yildizlar_guidance .sh: 425 -
431 ). (b) simple_guided_follow --back / --down -- installs standoff with position. At the beginning
of the RACE, it comes from scenario.sh. (c) mpc_guidance . environment_mount_deg ()
      (:214-235) -- sets the framing reference ey_ref; PRIORITY goes to $YILDIZ_TILT. (b) and (c)
      come from scenario.sh's environment, while (a) comes from the STACK's environment. Since the
      two are installed at separate times and with separate env, IT CAN BE SILENTLY DETACHED: *
      stack goes up with YILDIZ_DOWN=0 (camera looks at 0.00 deg), * script runs without giving DOWN
      -> standoff_geom returns to DESIGN value (4), positioned 4 m establishes standoff from the
      bottom and MPC establishes "camera axis +9.09". Result: when the camera is looking at 0.00,
      the law assumes it is looking at 9.09. There is no error message. (And vice versa: stack 4,
      script 0.) Same class: tools/plan_alignment.py (which ROUTE is installed) -- which TILT command to
      do this.

HOW TO READ: bbox_to_redis achieve tilt on startup: "tilt (geometry standoff): back=25 down=4 ->
+9.09 deg" (derived) "tilt manually: +0.00 deg" (with --tilt) "PHYSICAL GIMBAL: iris-1 <- tilt +0.00
deg" (committed) Gets the LAST committed value in the log and compares it to the expected tilt.

OUTPUT: 0 compatible | 1 INCOMPATIBLE | 2 decision could not be made (log/value could not be read).
"""
import argparse, os, re, sys
from pathlib import Path

DEFAULT_LOG = str(Path(__file__).resolve().parents[1] / 'logs' / 'bbox.log')


def stack_tilt(path_value):
    """COMMIT tilt command of the stack (last valid record) from bbox.log."""
    if not os.path.exists(path_value):
        return None, 'bbox.log missing'
    text_value = open(path_value, errors='replace').read()
    # Priority: "PHYSICAL GIMBAL: <model> <- tilt +X deg" (committed value)
    m = re.findall(r'PHYSICAL GIMBAL: \S+ <- tilt ([+-]?[0-9.]+) deg', text_value)
    if m:
        return float(m[-1]), 'PHYSICAL GIMBAL line'
    m = re.findall(r'tilt manually set: ([+-]?[0-9.]+) deg', text_value)
    if m:
        return float(m[-1]), 'manually set tilt line'
    m = re.findall(r'tilt \(standoff geometry\):.*?-> ([+-]?[0-9.]+) deg', text_value)
    if m:
        return float(m[-1]), 'standoff geometry line'
    return None, 'Tilt line not found in bbox.log'


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--expected-value', type=float, required=True,
                   help='tilt expected by the scenario [deg] (= $YILDIZ_TILT)')
    p.add_argument('--log', default=DEFAULT_LOG)
    p.add_argument('--tolerance', type=float, default=0.5, help='[deg]')
    a = p.parse_args()

    t, source_value = stack_tilt(a.log)
    if t is None:
        print(f"tilt_alignment: could not read stack tilt ({source_value})", file=sys.stderr)
        return 2
    difference = abs(t - a.expected_value)
    print(f"tilt_alignment: STACK (bbox) tilt = {t:+.2f} deg  [{source_value}]")
    print(f"tilt_alignment : SCENARIO expected = {a.expected_value:+.2f} deg ( YILDIZ_TILT )")
    if difference <= a.tolerance:
        print(f"tilt_alignment : COMPATIBLE (difference {difference:.2f} deg )")
        return 0
    print(f"tilt_alignment : *** INCOMPATIBLE -- difference {difference:.2f} deg ***", file=sys.stderr)
    print("tilt_alignment: The camera points at one angle while the scenario assumes another; "
          "the image reference (ey_ref) and vertical standoff are inconsistent.",
          file=sys.stderr)
    print("tilt_alignment: solution: use the same DOWN value for the scenario and stack "
          "(e.g. DOWN=0), or restart the stack with the desired DOWN.",
          file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
