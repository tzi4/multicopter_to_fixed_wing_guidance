#!/usr/bin/env python3
"""
swarm_command.py - manual command tool for swarm + target aircraft
========================================== Port contract is the same as yildizlar_guidance.sh and
companion_scripts/config.py: drone N (1..5) -> udp:127.0.0.1:{14551 + 10*(N-1)} SysID N target
aircraft -> udp:127.0.0.1:14601 SysID 6

Subcommands: status summarizes mode, arm state, position, and altitude for all vehicles.
target-takeoff arms the target aircraft and selects AUTO with its plan already loaded. drone-takeoff
--id N arms the copter in GUIDED and climbs to --alt meters. tracking --id N holds the copter behind
the target and points its nose toward the target.

NOTE (WSL2): ONLY ONE process can connect each UDP port. While this tool is running, another tool
listening to the same port (e.g. ground_station.py) should not be run.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

DRONE_PORT_BASE = 14551
TARGET_PORT = 14601
TARGET_SYSID = 6

# ArduCopter / ArduPlane custom mode numaralari
COPTER_GUIDED = 4
COPTER_LOITER = 5
PLANE_AUTO = 10
PLANE_TAKEOFF = 13

EARTH_R = 6378137.0


def connect_value(port, sysid, timeout=30):
    """Connect to the specified port and wait for a heartbeat from the expected SysID.

    SysID verification is required: Since all vehicles are together in 14550, it is possible to
    silently send commands to another vehicle when connected to the wrong port.
    """
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}',
                                        source_system=255, source_component=190)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and msg.get_srcSystem() == sysid:
            master.target_system = sysid
            master.target_component = msg.get_srcComponent()
            flow_rate_request(master)
            # Discard the accumulation at the time of connection: it was measured, in the first readings, 2-5 s
            # stale data was coming, when the buffer is empty, the delay decreases to 0.02 s.
            last_cleanup = time.monotonic() + 1.0
            while time.monotonic() < last_cleanup:
                if master.recv_match(blocking=False) is None:
                    break
            return master
    master.close()
    raise SystemExit(f"port {port}: SysID {sysid} heartbeat alinamadi ({timeout}s)")


def drone_connect(drone_id, timeout=30):
    if not 1 <= drone_id <= 5:
        raise SystemExit("drone id should be 1..5")
    return connect_value(DRONE_PORT_BASE + 10 * (drone_id - 1), drone_id, timeout)


def command_value(master, command, *params, wait_value=True):
    """Sends COMMAND_LONG; If wait=True, it reads and returns the ACK."""
    params = list(params) + [0] * (7 - len(params))
    master.mav.command_long_send(master.target_system, master.target_component,
                                 command, 0, *params[:7])
    if not wait_value:
        return None
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack is not None and ack.command == command:
            return ack.result
    return None


def mode_set(master, custom_mode):
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             custom_mode)


def mode_wait(master, custom_mode, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and msg.get_srcSystem() == master.target_system:
            if msg.custom_mode == custom_mode:
                return True
            mode_set(master, custom_mode)
    return False


def flow_rate_request(master, hz=10.0):
    """GLOBAL_POSITION_INT + increase ATTITUDE flow rate (SET_MESSAGE_INTERVAL).

    By default MAVProxy comes GLOBAL_POSITION_INT ~1.5 Hz; 20 m/s means 13 m between two samples in
    a target. Insufficient for guidance cycles.
    """
    interval_us_value = int(1e6 / hz)
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                   mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
        command_value(master, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
              msg_id, interval_us_value, wait_value=False)


def get_position(master, timeout=5):
    """(lat, lon, rel_alt_m, hdg_deg) - From GLOBAL_POSITION_INT, THE FRESHEST example.

    ATTENTION - there was a bug here and it was measured (2026-08-01): a single call to recv_match()
    returns the OLDEST message in the buffer UDP. When a reading was made per cycle, the buffer
    filled up and the read data became stale: Waiting for 6 s, a single read -> 9 message
    accumulated in the buffer, the FIRST read was ~6 s old. 20 m/s 120 m position error at target.
    Solution: empty the buffer TO THE END, use the last sample.
    (guidance_allstar/simple_guided_follow.py bundan etkilenmiyordu; o
    It uses a constantly reading thread - mavlink_utils.MavStateReader.)
    """
    last_value = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT',
                                blocking=(last_value is None), timeout=1)
        if msg is None:
            if last_value is not None:
                break
            continue
        if msg.get_srcSystem() == master.target_system:
            last_value = msg
    if last_value is None:
        return None
    return (last_value.lat / 1e7, last_value.lon / 1e7, last_value.relative_alt / 1000.0,
            last_value.hdg / 100.0)


def prearm_wait(master, timeout=120):
    """Waits until EKF/GPS is ready (SYS_STATUS + GPS_FIX)."""
    deadline = time.monotonic() + timeout
    last_value = ''
    while time.monotonic() < deadline:
        msg = master.recv_match(type=['GPS_RAW_INT', 'STATUSTEXT'], blocking=True,
                                timeout=1)
        if msg is None:
            continue
        if msg.get_srcSystem() != master.target_system:
            continue
        if msg.get_type() == 'STATUSTEXT':
            text_value = msg.text.strip()
            if text_value != last_value:
                print(f"  [FCU] {text_value}", flush=True)
                last_value = text_value
        elif msg.fix_type >= 3 and msg.satellites_visible >= 6:
            return True
    return False


def arm_et(master, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result_value = command_value(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if result_value == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if hb is not None and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return True
            return True
        print(f"  ARM rejected (result = {result_value}), trying again...", flush=True)
        time.sleep(2)
    return False


def speed_set(master, speed_ms):
    """GUIDED raise horizontal speed cap AT RUN (DO_CHANGE_SPEED).

    REASON: The upper limit of the WPNAV_SPEED parameter is 2000 cm/s = 20 m/s, i.e. 20 m/s CANNOT
    be exceeded from the parameter file. But in GUIDED MAV_CMD_DO_CHANGE_SPEED (type 1 = ground
    speed) directly calls ModeGuided::set_speed_xy_cms() ->
    AC_PosControl::set_max_speed_accel_NE_cm() and in this path there is NO TRIM to WPNAV_SPEED
    (ArduCopter/GCS_MAVLink_Copter.cpp:690, mode_guided.cpp:289).

    CAUTION: the value is reset EVERY time GUIDED is entered (pva_control_start writes WPNAV_SPEED
    again). It should resend this AFTER the homing code changes to GUIDED.
    """
    return command_value(master, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                 1,               # param1: 1 = ground speed
                 speed_ms,          # param2: m/s
                 -1, 0)           # param3: gas (- 1 = replacement)


def go_to_point(master, lat, lon, rel_alt):
    """Absolute position target on GUIDED (only position bits on)."""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                      # just use x,y,z
        int(lat * 1e7), int(lon * 1e7), rel_alt,
        0, 0, 0, 0, 0, 0, 0, 0)


def yaw_set(master, heading_deg, speed_dps=90):
    """CONDITION_YAW: absolute direction (param4= 0), shortest path (param3= 0).

    The initial speed_dps value 30 was insufficient for close passes. A target moving at 20 m/s at 120
    m has bearing rate approximately 9.5 deg/s, but the copter lagged while turning between commands
    at 30 deg/s. The target remained on the right edge: all 17 detections had x>500/640. A 90 deg/s
    turn rate tracked the bearing.
    """
    command_value(master, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
          heading_deg % 360.0, speed_dps, 0, 0, wait_value=False)


def distance_direction(lat1, lon1, lat2, lon2):
    """(distance_m, bearing_deg) - straight approach is sufficient for small distances."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    north_value = dlat * EARTH_R
    east_value = dlon * EARTH_R * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(north_value, east_value), math.degrees(math.atan2(east_value, north_value)) % 360


def shift_value(lat, lon, north_m, east_m):
    lat2 = lat + math.degrees(north_m / EARTH_R)
    lon2 = lon + math.degrees(east_m / (EARTH_R * math.cos(math.radians(lat))))
    return lat2, lon2


# ------------------------------------------------------------------ commands

def cmd_state(args):
    targets_value = [(DRONE_PORT_BASE + 10 * i, i + 1, f"drone_{i + 1}")
                for i in range(args.drones)]
    targets_value.append((TARGET_PORT, TARGET_SYSID, "target_plane"))
    for port, sysid, label_item in targets_value:
        try:
            master = connect_value(port, sysid, timeout=8)
        except SystemExit as exc:
            print(f"{label_item:12s} port {port}: {exc}")
            continue
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        position_value2 = get_position(master)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        mode_value = hb.custom_mode if hb else -1
        if position_value2:
            print(f"{label_item:12s} port={port} sysid={sysid} mod={mode_value} "
                  f"arm={'E' if armed else 'H'} "
                  f"lat={position_value2[0]:.6f} lon={position_value2[1]:.6f} alt={position_value2[2]:.1f}m "
                  f"hdg={position_value2[3]:.0f}")
        else:
            print(f"{label_item:12s} port={port} sysid={sysid} mod={mode_value} "
                  f"arm={'E' if armed else 'H'} (no position)")
        master.close()


def cmd_target_takeoff(args):
    master = connect_value(TARGET_PORT, TARGET_SYSID)
    print("Target aircraft connected. GPS/EKF expected...", flush=True)
    if not prearm_wait(master):
        raise SystemExit("target aircraft GPS could not fix")
    print("AUTO moduna aliniyor...", flush=True)
    mode_set(master, PLANE_AUTO)
    if not mode_wait(master, PLANE_AUTO):
        raise SystemExit("target aircraft did not switch to AUTO mode")
    print("ARM ediliyor...", flush=True)
    if not arm_et(master):
        raise SystemExit("target aircraft could not be ARMed")
    # In AUTO, after ARM, ArduPlane starts the take-up item itself; some versions require MISSION_START
    # for the first trigger.
    command_value(master, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    print("The departure is being watched...", flush=True)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        position_value2 = get_position(master)
        if position_value2 is None:
            continue
        print(f"  altitude={position_value2[2]:6.1f} m lat={position_value2[0]:.6f} lon={position_value2[1]:.6f}",
              flush=True)
        if position_value2[2] >= args.alt:
            print(f"Target aircraft is at {position_value2[2]:.1f} m, mission continues.")
            master.close()
            return
        time.sleep(2)
    master.close()
    raise SystemExit(f"target aircraft {args.alt} did not arrive at m within {args.timeout}s")


def cmd_drone_takeoff(args):
    master = drone_connect(args.id)
    print(f"drone_{args.id} connected. Waiting for GPS/EKF...", flush=True)
    if not prearm_wait(master):
        raise SystemExit("GPS fix alinamadi")
    print("GUIDED moduna aliniyor...", flush=True)
    mode_set(master, COPTER_GUIDED)
    if not mode_wait(master, COPTER_GUIDED):
        raise SystemExit("GUIDED moduna gecmedi")
    print("ARM ediliyor...", flush=True)
    if not arm_et(master):
        raise SystemExit("ARM edilemedi")
    print(f"{args.alt} departure to m...", flush=True)
    command_value(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        position_value2 = get_position(master)
        if position_value2 is None:
            continue
        print(f"  altitude={position_value2[2]:6.1f} m", flush=True)
        if position_value2[2] >= args.alt * 0.95:
            print(f"drone_ {args.id} {position_value2[2]:.1f} Ready at m.")
            master.close()
            return
        time.sleep(1)
    master.close()
    raise SystemExit(f"drone {args.alt} did not take off to m in {args.timeout}s")


def cmd_tracking(args):
    """Keeps the hopper BEHIND the target and points its nose towards the target.

    The goal is NOT to hit: to get the target in the frame and feed the bbox stream. Therefore, the
    command point is a point shifted --backward meters behind the target and --vertical meters above/below;
    The hopper is not given the target ITSELF.
    """
    target_value = connect_value(TARGET_PORT, TARGET_SYSID)
    drone = drone_connect(args.id)
    print(f"tracking started: drone_{args.id} <- target aircraft "
          f"(back={args.backward} m, vertical={args.vertical:+.0f} m)", flush=True)

    last_yaw = 0.0
    deadline = time.monotonic() + args.duration_value
    while time.monotonic() < deadline:
        h = get_position(target_value, timeout=3)
        d = get_position(drone, timeout=3)
        if h is None or d is None:
            print("  waiting for telemetry...", flush=True)
            continue

        h_lat, h_lon, h_alt, h_hdg = h
        d_lat, d_lon, d_alt, _ = d

        # Meters 'back' AGAINST the target's direction of travel: following behind.
        rad = math.radians(h_hdg)
        git_lat, git_lon = shift_value(h_lat, h_lon,
                                  -args.backward * math.cos(rad),
                                  -args.backward * math.sin(rad))
        git_alt = max(5.0, h_alt + args.vertical)
        go_to_point(drone, git_lat, git_lon, git_alt)

        distance, direction = distance_direction(d_lat, d_lon, h_lat, h_lon)
        # When the yaw command deviates by more than 3 degrees, not every frame: sending CONDITION_YAW
        # continuously resets the rotation of the helicopter and makes it vibrate.
        if abs((direction - last_yaw + 180) % 360 - 180) > 3:
            yaw_set(drone, direction)
            last_yaw = direction
        print(f"  target sub={h_alt:5.1f} hdg={h_hdg:3.0f} | drone alt={d_alt:5.1f} "
              f"| distance= {distance:7.1f} m direction= {direction:3.0f}", flush=True)
        time.sleep(args.period_value)

    target_value.close()
    drone.close()
    print("tracking period has expired.")


def cmd_ambush(args):
    """He parks the copter at a point ON the ROUTE of the target and constantly points its nose towards the
target.

    WHY NOT PURSUE AMBUSH: roof of the helicopter WPNAV_SPEED 18 m/s, target aircraft 20 m/s
    cruising -> chasing copter cannot close the distance AT ALL (measurement: in pursuit mode the
    distance remained in the m band 450-620). In ambush, the copter waits on the target's route,
    that is, GEOMETRY, not SPEED, provides closure.

    [GIMBAL BRANCH 2026-08-05 - CORRECTION OF REASON] A second reason was written here in the past:
    "at full throttle, the hopper leans forward ~20 degrees, since the camera is fixed and looking
    forward, the target escapes to the upper edge of the vertical frame". This is NOW VALID: the
    camera was measured on a self-stabilizing physical tilt gimbal, the camera world pitch at max
    0.65 deg while the body was swinging +-35 deg (GIMBAL_NOTES.md). So the ambush no longer has
    the VERTICAL framing advantage. Standing grounds: (a) speed difference (the main reason), (b) no
    yaw drift in the suspended copter -- horizontal framing is still connected to the airframe yaw
    because the gimbal is SINGLE AXIS dead, (c) roll is zero (single axis gimbal does not compensate
    roll, it is reflected in the image).

    Location is given in meters north/east relative to home; home = vehicles origin (world origin
    Gazebo).
    """
    drone = drone_connect(args.id)
    target_value = connect_value(TARGET_PORT, TARGET_SYSID)

    initial = get_position(drone, timeout=10)
    if initial is None:
        raise SystemExit("drone konumu okunamadi")
    # Home: not the starting point, no back calculation from the current position; --north-value /--east-value is
    # relative to HOME, so we want home from FCU.
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                0, 0, 0, 0, 0, 0, 0, 0)
    home_msg = None
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        home_msg = drone.recv_match(type='HOME_POSITION', blocking=True, timeout=2)
        if home_msg is not None and home_msg.get_srcSystem() == drone.target_system:
            break
    if home_msg is None:
        raise SystemExit("HOME_POSITION alinamadi")
    h_lat, h_lon = home_msg.latitude / 1e7, home_msg.longitude / 1e7

    ambush_lat, ambush_lon = shift_value(h_lat, h_lon, args.north_value, args.east_value)
    print(f"pusu noktasi: lat={ambush_lat:.6f} lon={ambush_lon:.6f} alt={args.alt} m "
          f"({args.north_value:+.0f} m north, {args.east_value:+.0f} m east from home)", flush=True)

    go_to_point(drone, ambush_lat, ambush_lon, args.alt)
    arrival_deadline = time.monotonic() + args.arrival_timeout
    while time.monotonic() < arrival_deadline:
        d = get_position(drone, timeout=3)
        if d is None:
            continue
        remaining_value, _ = distance_direction(d[0], d[1], ambush_lat, ambush_lon)
        print(f"  {remaining_value:6.1f} m to ambush point, altitude={d[2]:5.1f} m", flush=True)
        if remaining_value < 15 and abs(d[2] - args.alt) < 5:
            break
        time.sleep(2)
    else:
        print("WARNING: waiting point not reached; proceeding to the waiting state.")

    print("in ambush: nose turned to target, position command NO LONGER SENT "
          "(resets new position target CONDITION_YAW).", flush=True)
    last_yaw = None
    nearest_distance = float('inf')
    deadline = time.monotonic() + args.duration_value
    while time.monotonic() < deadline:
        h = get_position(target_value, timeout=3)
        d = get_position(drone, timeout=3)
        if h is None or d is None:
            continue
        distance, direction = distance_direction(d[0], d[1], h[0], h[1])
        nearest_distance = min(nearest_distance, distance)
        if last_yaw is None or abs((direction - last_yaw + 180) % 360 - 180) > 1.5:
            yaw_set(drone, direction)
            last_yaw = direction
        print(f"  target distance={distance:7.1f} m direction={direction:3.0f} "
              f"target_alt_value={h[2]:5.1f} | nearest_distance={nearest_distance:7.1f} m", flush=True)
        time.sleep(args.period_value)

    print(f"The ambush is over. Closest passage: {nearest_distance:.1f} m")
    drone.close()
    target_value.close()


def cmd_speed_test(args):
    """It measures the horizontal speed and altitude hold that the copter ACTUALLY achieves.

    Parameter ceiling and PHYSICAL ceiling are not the same thing: writing WPNAV_SPEED 2000 does not
    guarantee 20 m/s, the thrust/weight ratio of the copter and ANGLE_MAX are achieved as much as
    they allow. This command goes to a remote point and reports ground speed and altitude deviation
    from GLOBAL_POSITION_INT.
    """
    drone = drone_connect(args.id)
    d = get_position(drone, timeout=10)
    if d is None:
        raise SystemExit("drone konumu okunamadi")
    if d[2] < 5:
        raise SystemExit("take off first with 'drone-takeoff'")

    start_alt = d[2]
    git_lat, git_lon = shift_value(d[0], d[1], args.distance, 0)
    if args.speed_value > 0:
        result_value = speed_set(drone, args.speed_value)
        print(f"DO_CHANGE_SPEED {args.speed_value} m/s -> result={result_value} "
              f"({'accepted' if result_value == 0 else 'RED'})", flush=True)
    print(f"{args.distance:.0f} m full throttle running north, altitude {start_alt:.1f} m",
          flush=True)
    go_to_point(drone, git_lat, git_lon, start_alt)

    max_speed = 0.0
    minimum_altitude = start_alt
    max_bank = 0.0
    sample_value = 0
    deadline = time.monotonic() + args.duration_value
    while time.monotonic() < deadline:
        msg = drone.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'],
                               blocking=True, timeout=2)
        if msg is None or msg.get_srcSystem() != drone.target_system:
            continue
        if msg.get_type() == 'ATTITUDE':
            bank = math.degrees(math.hypot(msg.roll, msg.pitch))
            max_bank = max(max_bank, bank)
            continue
        speed_value = math.hypot(msg.vx, msg.vy) / 100.0
        alt = msg.relative_alt / 1000.0
        max_speed = max(max_speed, speed_value)
        minimum_altitude = min(minimum_altitude, alt)
        sample_value += 1
        if sample_value % 5 == 0:
            print(f"  speed={speed_value:5.2f} m/s altitude={alt:6.2f} m  "
                  f"bank={max_bank:4.1f} deg", flush=True)

    print(f"\nRESULT: top ground speed = {max_speed:.2f} m/s")
    print(f"       en cok bank        = {max_bank:.1f} derece")
    print(f"       altitude {start_alt:.1f} -> lowest {minimum_altitude:.1f} m "
          f"(lost {start_alt - minimum_altitude:.1f} m)")
    drone.close()


def cmd_speed_lock(args):
    """GUIDED Side process that periodically re-imposes the speed cap.

    WHY SEPARATE PROCESS: guidance_allstar/simple_guided_follow.py does not send any DO_CHANGE_SPEED
    (there is not a single place in the repo where it occurs), so the hunter remains in the
    WPNAV_SPEED ceiling = 20 m/s. Since the target plane 20 m/s is cruising, the distance does not
    close AT ALL on the straight leg; The shutdown only comes from taking a nap while the plane is
    turning. Measured: same iris holding 38 m/s without losing altitude (tools/swarm_command.py
    speed-test).

    WHY PERIODIC: each time the value is entered in GUIDED it is reset to WPNAV_SPEED; When the
    miss-recovery machine drops to BRAKE and returns to GUIDED, the ceiling also falls back. That's
    why command --period-value is refreshed every second.

    The guidance code itself CANNOT be touched: it's just a COMMAND_LONG , it doesn't race via
    setpoint.
    """
    drone = drone_connect(args.id)
    print(f"speed lock: drone_{args.id} -> {args.speed_value} m/s every {args.period_value} s "
          f"refreshing (exit with Ctrl+C)", flush=True)
    last_result = None
    deadline = time.monotonic() + args.duration_value
    while time.monotonic() < deadline:
        result_value = speed_set(drone, args.speed_value)
        if result_value != last_result:
            print(f"  DO_CHANGE_SPEED result= {result_value} "
                  f"({'accepted' if result_value == 0 else 'RED/yanit absent'})", flush=True)
            last_result = result_value
        time.sleep(args.period_value)
    drone.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command_value', required=True)

    p = sub.add_parser('state_value', help='Summary of all tools')
    p.add_argument('--drones', type=int, default=5)
    p.set_defaults(func=cmd_state)

    p = sub.add_parser('target-takeoff', help='remove target aircraft with AUTO')
    p.add_argument('--alt', type=float, default=40, help='altitude to be verified (m)')
    p.add_argument('--timeout', type=float, default=180)
    p.set_defaults(func=cmd_target_takeoff)

    p = sub.add_parser('drone-takeoff', help='lift the hopper with GUIDED')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--alt', type=float, default=50)
    p.add_argument('--timeout', type=float, default=120)
    p.set_defaults(func=cmd_drone_takeoff)

    p = sub.add_parser('tracking_value', help='keep the hopper behind the target')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--backward', type=float, default=120, help='Distance to stay behind the target (m)')
    p.add_argument('--vertical', type=float, default=-10, help='vertical offset relative to target (m)')
    p.add_argument('--duration-value', type=float, default=300)
    p.add_argument('--period-value', type=float, default=1.0)
    p.set_defaults(func=cmd_tracking)

    p = sub.add_parser('ambush', help='park the helicopter on the target\'s route')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--north-value', type=float, default=500, help='North (m) relative to home\'')
    p.add_argument('--east-value', type=float, default=120, help='East (m) from home\'')
    p.add_argument('--alt', type=float, default=62, help='ambush altitude (m)')
    p.add_argument('--duration-value', type=float, default=300)
    p.add_argument('--period-value', type=float, default=0.5)
    p.add_argument('--arrival-timeout', type=float, default=180)
    p.set_defaults(func=cmd_ambush)

    p = sub.add_parser('speed-test', help='measure the actual speed achieved')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--distance', type=float, default=1500, help='running distance (m)')
    p.add_argument('--speed-value', type=float, default=0,
                   help='Desired ground speed with DO_CHANGE_SPEED (m/s); 0 = do not send')
    p.add_argument('--duration-value', type=float, default=90)
    p.set_defaults(func=cmd_speed_test)

    p = sub.add_parser('speed-lock', help='GUIDED constantly impose speed cap')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--speed-value', type=float, default=35, help='desired ground speed (m/s)')
    p.add_argument('--period-value', type=float, default=2.0)
    p.add_argument('--duration-value', type=float, default=100000)
    p.set_defaults(func=cmd_speed_lock)

    args = parser.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        print("\nkesildi.", file=sys.stderr)


if __name__ == '__main__':
    main()
