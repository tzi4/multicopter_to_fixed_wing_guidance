#!/usr/bin/env python3
"""
launch_mission.py - THE ONLY COMMAND THAT PUT ALL VEHICLES INTO MISSION
=========================================================== This is the equivalent of
bumblebee/formation.py. Difference: bumblebee had two fixed wings and both were put into AUTO; Here
the target flies in a fixed wing AUTO plan, the fighter helicopter(s) take off at GUIDED and wait
ready for the guidance code.

IN ORDER BY: 1. It connects to all vehicles and VERIFIES SysIDs (since 14550 has them all together,
connecting to the wrong port means silently sending a command to another vehicle). 2. Uploads the
plan to the target plane and verifies it by READING BACK (scripts/load_plan.py). 3. Arms the target
and puts it in AUTO, confirming that it has climbed. 4. Arms the copters at GUIDED and increases
them to --drone-alt meters. 5. If desired, it sends a cruise speed command to the target. 6. Success
table on closing.

Every step is VERIFIED; If one of them doesn't work, it gives an error and stops (silently entering
the guidance test with an incomplete installation is the most expensive mistake).

USE: tools/launch_mission.py # default: ellipse plan, 1 copter, 60 m tools/launch_mission.py --drones 3
--drone-alt 80 tools/launch_mission.py --plan missions/target_circuit.plan --target-speed 16
tools/launch_mission.py --only-target # touch the copters tools/launch_mission.py --only-drone #
touch the target
"""

import argparse
import math
import os
import subprocess
import sys
import time

ROOT_VALUE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT_VALUE, 'tools'))

from pymavlink import mavutil                                   # noqa: E402
from swarm_command import (COPTER_GUIDED, PLANE_AUTO, TARGET_PORT,  # noqa: E402
                        TARGET_SYSID, arm_et, connect_value, drone_connect, speed_set,
                        command_value, get_position, mode_set, mode_wait, prearm_wait)


def step_value(n, text_value):
    print(f"\n[{n}] {text_value}", flush=True)


def plan_load(plan, port, sysid):
    """scripts/load_plan.py: Loads with MISSION_ITEM_INT, READS BACK and verifies."""
    path_value = os.path.join(ROOT_VALUE, 'scripts', 'load_plan.py')
    result_value = subprocess.run([sys.executable, path_value, '--plan', plan,
                            '--ports', f'{port}:{sysid}'],
                           capture_output=True, text=True)
    print(result_value.stdout.strip() or result_value.stderr.strip(), flush=True)
    return result_value.returncode == 0


def target_launch(args):
    step_value(1, f'Loading target aircraft (SysID {TARGET_SYSID}) plan: '
            f'{os.path.relpath(args.plan, ROOT_VALUE)}')
    # Plan loading uses 14602 (14601 belongs to manual command tools); No two processes can connect the
    # same UDP port.
    if not plan_load(args.plan, 14602, TARGET_SYSID):
        raise SystemExit('plan yuklenemedi')

    step_value(2, 'Target aircraft connecting, waiting for GPS/EKF')
    target_value = connect_value(TARGET_PORT, TARGET_SYSID)
    if not prearm_wait(target_value):
        raise SystemExit('target aircraft GPS could not fix')

    step_value(3, 'The target aircraft is put into AUTO mode and ARMed.')
    mode_set(target_value, PLANE_AUTO)
    if not mode_wait(target_value, PLANE_AUTO):
        raise SystemExit('target did not switch to AUTO mode')
    if not arm_et(target_value):
        raise SystemExit('target could not be ARMed')
    # In some ArduPlane versions, ARM alone does not trigger take-off in AUTO.
    command_value(target_value, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)

    step_value(4, f'Target aircraft expected to climb to {args.target_alt_value:.0f} m\'')
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        k = get_position(target_value)
        if k is None:
            continue
        print(f'    altitude={k[2]:6.1f} m', flush=True)
        if k[2] >= args.target_alt_value:
            break
        time.sleep(2)
    else:
        raise SystemExit(f'target did not appear in {args.target_alt_value} m\'ye {args.timeout}s')

    if args.target_speed > 0:
        # In order for this command to take effect in ArduPlane, TECS_SYNAIRSPEED 1 is required (it is written
        # with its measurement in params/target_plane.parm).
        result_value = command_value(target_value, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                      0, args.target_speed, -1, 0)
        print(f'    cruise speed {args.target_speed} m/s -> ACK={result_value}', flush=True)
    target_value.close()
    print('    The target aircraft is on duty.', flush=True)


def copters_launch(args):
    for i in range(1, args.drones + 1):
        step_value(5 + i, f'drone_{i} (SysID {i}) GUIDED\' is also being removed '
                    f'-> {args.drone_alt:.0f} m')
        d = drone_connect(i)
        if not prearm_wait(d):
            raise SystemExit(f'drone_{i} GPS fix alamadi')
        mode_set(d, COPTER_GUIDED)
        if not mode_wait(d, COPTER_GUIDED):
            raise SystemExit(f'drone_{i} GUIDED moduna gecmedi')
        if not arm_et(d):
            raise SystemExit(f'drone_{i} ARM edilemedi')
        command_value(d, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
              args.drone_alt)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            k = get_position(d)
            if k is None:
                continue
            if k[2] >= args.drone_alt * 0.95:
                print(f'    drone_ {i} {k[2]:.1f} m \' is also available.', flush=True)
                break
            time.sleep(1)
        else:
            raise SystemExit(f'drone_{i} {args.drone_alt} m\'ye cikmadi')
        if args.drone_speed > 0:
            # GUIDED speed ceiling: WPNAV_SPEED parameter upper limit 20 m/s; DO_CHANGE_SPEED passes this ceiling
            # without clipping (swarm_command.speed_set).
            print(f'    speed ceiling {args.drone_speed} m/s -> '
                  f'ACK={speed_set(d, args.drone_speed)}', flush=True)
        d.close()


def state_table(args):
    print('\n' + '=' * 68)
    print(f'{"vehicle_value":12s} {"port":>6s} {"sysid":>6s} {"mode_value":>5s} {"arm":>4s} '
          f'{"altitude_value":>8s} location')
    print('-' * 68)
    targets_value = [(14551 + 10 * i, i + 1, f'drone_{i+1}') for i in range(args.drones)]
    targets_value.append((TARGET_PORT, TARGET_SYSID, 'target_plane'))
    for port, sysid, label_item in targets_value:
        try:
            m = connect_value(port, sysid, timeout=8)
        except SystemExit:
            print(f'{label_item:12s} {port:6d} {sysid:6d} NO CONNECTION')
            continue
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        k = get_position(m)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        print(f'{label_item:12s} {port:6d} {sysid:6d} {hb.custom_mode if hb else -1:5d} '
              f'{"E" if armed else "H":>4s} {k[2] if k else 0:7.1f}m  '
              f'{k[0]:.6f},{k[1]:.6f}' if k else '')
        m.close()
    print('=' * 68)


def standoff_geometry():
    """Derive the standoff pair from the environment for the hint line.

    THE ONLY SOURCE is scripts/standoff_geom.sh (yildizlar_guidance.sh and tools/scenario.sh sources
    it); The hint now reads the formula by REFERENCED instead of copying it. The clone version
    YILDIZ_MOUNT defaulted to 30, and after the environment was upgraded to 0, it suggested --down
    13 (correctly the design value is 4; derivation is invalid in mount~0).
    """
    try:
        output = subprocess.check_output(
            ['bash', '-c',
             f'source "{ROOT_VALUE}/scripts/standoff_geom.sh" >/dev/null 2>&1; '
             'echo "$YILDIZ_BACK $YILDIZ_DOWN $YILDIZ_MOUNT $YILDIZ_PITCH_TRIM"'],
            text=True)
        back, down, mount, trim = (float(x) for x in output.split())
    except (OSError, ValueError, subprocess.CalledProcessError):
        # If standoff_geom.sh cannot be read, don't blow up because of the mission report hint: fall into the
        # known binary of the current 0-degree setup.
        back, down, mount, trim = 25.0, 4.0, 0.0, -2.5
    return back, down, mount, trim


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', default=os.path.join(ROOT_VALUE, 'missions', 'target_ellipse.plan'))
    p.add_argument('--drones', type=int, default=1, help='how many copters will be removed (0..5)')
    p.add_argument('--drone-alt', type=float, default=60)
    p.add_argument('--drone-speed', type=float, default=0,
                   help='hopper GUIDED speed ceiling (m/s); 0 = touch')
    p.add_argument('--target-alt-value', type=float, default=50,
                   help='climb altitude of the target to be verified (m)')
    p.add_argument('--target-speed', type=float, default=0,
                   help='target cruise speed (m/s); 0 = value in plan')
    p.add_argument('--timeout', type=float, default=240)
    p.add_argument('--only-target', action='store_true')
    p.add_argument('--only-drone', action='store_true')
    args = p.parse_args()

    if not 0 <= args.drones <= 5:
        raise SystemExit('Must be between --drones 0..5')
    if not os.path.isfile(args.plan):
        raise SystemExit(f'plan not found: {args.plan}')

    t0 = time.time()
    if not args.only_drone:
        target_launch(args)
    if not args.only_target and args.drones > 0:
        copters_launch(args)
    state_table(args)
    print(f'\n Task started ( {time.time() - t0:.0f} s).')
    back, down, mount, trim = standoff_geometry()
    # TWO PROCESSES ARE WRITTEN: 2026-08-05 was run a third time without starting visual guidance and was
    # seen as "MPC is shaking / not following the target" -- whereas MPC was not running at all. This
    # error is repeated as long as the clue points to a single process.
    print('\n' + '=' * 68)
    print('NEXT -- ON TWO SEPARATE TERMINALS (both required):')
    print('=' * 68)
    print('  [ 1 ] POSITION_BASED (approximation):')
    print('      cd guidance_allstar && python3 simple_guided_follow.py \\')
    print(f'          --no-kill-mode --yaw-lock --back {back:g} --down {down:g}')
    print('      # second approximation option: --approximation intersection')
    print()
    print('  [2] DISPLAY (inherit) -- REMEMBER THIS:')
    print('      cd guidance_allstar && python3 mpc_guidance.py')
    print('      # or other branch: python3 tracking_guidance.py')
    print()
    print('  [ 2 ] CANNOT GO TO VIEW IF NOT START: dead-man switch')
    print('  ("visual_alive") blocks the migration and succeeds this line in bbox.log:')
    print('      [DECISION] NO video controller... forgot to initialize?')
    print('=' * 68)
    print(f'  ( standoff source scripts/ standoff_geom .sh: mount = {mount:g} '
          f'trim={trim:g}; YILDIZ_BACK/YILDIZ_MOUNT/YILDIZ_PITCH_TRIM/'
          'varies with YILDIZ_DOWN)')


if __name__ == '__main__':
    main()
