#!/usr/bin/env python3
"""
configure_gimbal_params.py -- SETUP/VERIFY ArduPilot parameters for servo gimbal
=========================================================== WHY THIS TOOL: The gimbal command
(MAV_CMD_DO_MOUNT_CONTROL / GIMBAL_MANAGER) returns the servo only if the mount SERVO driver is
defined on autopilot. That definition consists of a handful of parameters, and when entered
MANUALLY, two errors occur most frequently: (1) servo is soldered to another channel, but
SERVOx_FUNCTION is in another channel, (2) MNT1_TYPE is not REBOOTED after changing (the parameter
is written but the driver is not loaded; it appears "the command is going, the servo does not move"
from the outside). This tool shuts down both: writes, READS BACK, verifies, and tells you whether to
reboot.

WHY DEFAULT 'DRY': typing parameters permanently changes the tool. First, it SHOWS what to write
with --channel-value; you actually need --apply-value to write it (same discipline as tools/set_tilt.py in the
repo).

USAGE: # see what to write (does not write anything) tools/ configure_gimbal_params .py
--channel-value 5

    # actually write + read back and verify tools/configure_gimbal_params.py --channel-value 5 --apply-value

    # read current values ​​only (diagnostics) tools/configure_gimbal_params.py --channel-value 5 --read-value

    # real hardware (serial port) tools/configure_gimbal_params.py --connection-value /dev/ttyAMA0 --baud 57600
    --channel-value 5 --apply-value

AFTER: tools/mavlink_tilt.py --channel-value 5 (give a command, see if the servo moves)
tools/gimbal_bench_tracking.py (tilt the body, is the camera stable)
"""

import argparse
import sys
import time

from pymavlink import mavutil

# --- PARAMETER SET ------------------------------------------------ MNT1_TYPE=1 (Servo): Connects
# mount1 to the servo driver. ArduPilot reads this parameter at BOOST -> if it changes, REBOOT
# REQUIREMENT. SERVOx_FUNCTION=7 (Mount1Pitch): single axis (pitch) gimbal is our case. MNT1_MODE=2
# (MAVLINK_TARGETING): receives the angle command from MAVLink (from Pi). If it is RC_TARGETING (3),
# our commands will be ignored and the gimbal will listen to RC. MNT1_PITCH_MIN/MAX: software clamp.
# It is kept narrower than the ACTUAL range of the physical gimbal so that the servo does not reach
# the mechanical limit. Defaults were chosen compatible with TiltTracking's software clamp
# (tools/gz_gimbal.py: -30..+60); If your gimbal is narrower, go with --pitch-min/--pitch-max.
MNT_TYPE_SERVO = 1
SERVO_FUNC_MOUNT1_PITCH = 7
MNT_MODE_MAVLINK = 2


def param_set(channel_value, pitch_min, pitch_max):
    """(name, value, type, reboot_ister) list -- ORDER of spelling does not matter."""
    return [
        ('MNT1_TYPE', MNT_TYPE_SERVO,
         mavutil.mavlink.MAV_PARAM_TYPE_INT8, True),
        (f'SERVO{channel_value}_FUNCTION', SERVO_FUNC_MOUNT1_PITCH,
         mavutil.mavlink.MAV_PARAM_TYPE_INT16, True),
        ('MNT1_MODE', MNT_MODE_MAVLINK,
         mavutil.mavlink.MAV_PARAM_TYPE_INT8, False),
        ('MNT1_PITCH_MIN', pitch_min,
         mavutil.mavlink.MAV_PARAM_TYPE_REAL32, False),
        ('MNT1_PITCH_MAX', pitch_max,
         mavutil.mavlink.MAV_PARAM_TYPE_REAL32, False),
    ]


def read_value(mav, label_item, time_timeout=3.0):
    """Read single parameter; otherwise None. (The name not in the firmware remains silent.)"""
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component, label_item.encode(), -1)
    finish = time.time() + time_timeout
    while time.time() < finish:
        m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == label_item:
            return m.param_value
    return None


def write_and_validate(mav, label_item, value_value, type_value, trial=3):
    """Write -> read back -> compare. Autopilot does not give ACK, verification is REQUIRED."""
    for i in range(trial):
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               label_item.encode(), float(value_value), type_value)
        time.sleep(0.3)
        read_value2 = read_value(mav, label_item)
        if read_value2 is not None and abs(read_value2 - float(value_value)) < 1e-4:
            return True, read_value2
        if i < trial - 1:
            time.sleep(0.5)
    return False, read_value2


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--connection-value', default='tcp:127.0.0.1:5760',
                   help='SITL: tcp:127.0.0.1:5760 | hardware: /dev/ttyAMA0')
    p.add_argument('--baud', type=int, default=57600,
                   help='Used only in serial connection')
    p.add_argument('--channel-value', type=int, default=5,
                   help='mount output to which pitch servo is SOLDERED '
                        '(SERVOx_FUNCTION=7 is written here)')
    p.add_argument('--pitch-min', type=float, default=-30.0,
                   help='MNT1_PITCH_MIN [deg] (down view limit)')
    p.add_argument('--pitch-max', type=float, default=60.0,
                   help='MNT1_PITCH_MAX [deg] (up view limit)')
    p.add_argument('--apply-value', action='store_true',
                   help='ACTUALLY write (or it just shows what to write)')
    p.add_argument('--read-value', action='store_true',
                   help='just read and print current values ​​(diagnostics)')
    a = p.parse_args()

    set_value = param_set(a.channel_value, a.pitch_min, a.pitch_max)

    if not (a.apply_value or a.read_value):
        print("DRY RUNNING -- nothing written. Add --apply-value to write.\n")
        print(f"  What to write for channel {a.channel_value}:")
        for label_item, value_value, _, reboot in set_value:
            print(f"    {label_item:18s} = {value_value:g}" + ("   [REBOOT required]" if reboot else ""))
        print("\n WARNING: Whichever output the servo is soldered to should be --channel-value.")
        return 0

    print(f"baglaniliyor: {a.connection_value}")
    if a.connection_value.startswith('/dev/') or a.connection_value.startswith('COM'):
        mav = mavutil.mavlink_connection(a.connection_value, baud=a.baud, source_system=254)
    else:
        mav = mavutil.mavlink_connection(a.connection_value, source_system=254)
    mav.wait_heartbeat(timeout=15)
    if mav.target_system == 0:
        print("ERROR: No HEARTBEAT -- is the link/baud correct?", file=sys.stderr)
        return 1
    print(f"baglandi: sysid={mav.target_system} comp={mav.target_component}\n")

    if a.read_value:
        print("CURRENT VALUES:")
        for label_item, expected_value, _, _ in set_value:
            v = read_value(mav, label_item)
            if v is None:
                state_value = "NONE (undefined in firmware?)"
            elif abs(v - float(expected_value)) < 1e-4:
                state_value = "expected value"
            else:
                state_value = f"DIFFERENT (expected {expected_value:g} )"
            print(f"  {label_item:18s} = {'--' if v is None else f'{v:g}':>8s} {state_value}")
        return 0

    reboot_needed = False
    error_value = False
    print("WRITTEN (each verified by reading back):")
    for label_item, value_value, type_value, reboot in set_value:
        previous_value = read_value(mav, label_item)
        complete, read_value2 = write_and_validate(mav, label_item, value_value, type_value)
        if complete:
            changed_value = previous_value is None or abs(previous_value - float(value_value)) > 1e-4
            print(f"  {label_item:18s} = {value_value:g} OK" +
                  (f"   (previously {previous_value:g} )" if changed_value and previous_value is not None else ""))
            if reboot and changed_value:
                reboot_needed = True
        else:
            error_value = True
            print(f"  {label_item:18s} = {value_value:g} FAILED "
                  f"(read: {'absent' if read_value2 is None else f'{read_value2:g}'})")

    print()
    if error_value:
        print("SOME PARAMETERS COULD NOT BE WRITTEN. Common reasons: this is the parameter name")
        print("is missing in the firmware (version difference), or the vehicle is ARMED (some params")
        print("armed iken kilitli).")
        return 1

    if reboot_needed:
        print("*** REBOOT REQUIREMENT *** MNT1_TYPE / SERVOx_FUNCTION is read at startup.")
        print("Restart autopilot, then verify:")
        print(f"    tools/configure_gimbal_params.py --connection-value {a.connection_value} "
              f"--channel-value {a.channel_value} --read-value")
    else:
        print("No reboot required (type and function were already correct).")

    print("\nNEXT STEP -- does the servo physically move:")
    print(f"    tools/mavlink_tilt.py --connection-value {a.connection_value} --channel-value {a.channel_value}")
    print("Then body tilt test: tools/gimbal_bench_tracking.py "
          "(see hardware/GIMBAL_TRACKING_TEST.md)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
