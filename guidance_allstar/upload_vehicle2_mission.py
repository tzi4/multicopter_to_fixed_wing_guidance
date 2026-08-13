#!/usr/bin/env python3
"""Upload a QGroundControl .plan or ArduPilot .waypoints mission to vehicle 2."""

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil, mavwp


GLOBAL_INT_FRAME = {
    mavutil.mavlink.MAV_FRAME_GLOBAL: mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
}

DEFAULT_MISSION = Path(__file__).resolve().parents[1] / "missions" / "hedef_elips.plan"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Upload a QGroundControl .plan or ArduPilot .waypoints mission to the target vehicle"
    )
    parser.add_argument("--connect", default="udpin:localhost:14600")
    parser.add_argument("--mission", default=str(DEFAULT_MISSION))
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--mode", default="", help="Optional mode to set after upload, e.g. AUTO")
    parser.add_argument("--set-current", type=int, default=0)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--no-verify", action="store_true", help="Skip post-upload mission count verification")
    return parser.parse_args()


def wait_for_heartbeat(master, timeout):
    print(f"[mission_upload] Waiting for heartbeat on {getattr(master, 'address', 'vehicle')}")
    msg = master.wait_heartbeat(timeout=timeout)
    if msg is None:
        raise TimeoutError("Timed out waiting for vehicle heartbeat")

    print(
        f"[mission_upload] Heartbeat sys={master.target_system} "
        f"comp={master.target_component}"
    )


def qgc_plan_simple_item_to_mavlink(item, seq):
    if item.get("type") != "SimpleItem":
        raise ValueError(
            "Only QGroundControl SimpleItem missions are supported. "
            f"Unsupported item type at sequence {seq}: {item.get('type')!r}"
        )

    params = [0.0 if value is None else value for value in item.get("params", [])]
    if len(params) > 7:
        raise ValueError(f"QGC .plan item {seq} has too many params: {len(params)}")
    params += [0.0] * (7 - len(params))

    return mavutil.mavlink.MAVLink_mission_item_message(
        0,
        0,
        int(seq),
        int(item.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
        int(item["command"]),
        1 if seq == 0 else 0,
        1 if item.get("autoContinue", True) else 0,
        float(params[0]),
        float(params[1]),
        float(params[2]),
        float(params[3]),
        float(params[4]),
        float(params[5]),
        float(params[6]),
    )


def load_qgc_plan(path):
    with path.open() as f:
        plan = json.load(f)

    if plan.get("fileType") != "Plan":
        raise ValueError(f"Not a QGroundControl Plan file: {path}")

    mission = plan.get("mission")
    if not isinstance(mission, dict):
        raise ValueError(f"QGC .plan has no mission object: {path}")

    items = mission.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError(f"QGC .plan has no mission items: {path}")

    loader = mavwp.MAVWPLoader()
    for seq, item in enumerate(items):
        loader.add(qgc_plan_simple_item_to_mavlink(item, seq))

    return loader


def load_mission(path):
    mission_path = Path(path).expanduser()
    if not mission_path.exists():
        raise FileNotFoundError(f"Mission file does not exist: {mission_path}")

    if mission_path.suffix.lower() == ".plan":
        loader = load_qgc_plan(mission_path)
    else:
        loader = mavwp.MAVWPLoader()
        loader.load(str(mission_path))

    count = loader.count()
    if count <= 0:
        raise RuntimeError(f"Mission file has no waypoints: {mission_path}")

    print(f"[mission_upload] Loaded {count} mission items from {mission_path}")
    return loader, mission_path


def drain_mission_messages(master, duration=0.5):
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        master.recv_match(
            type=["MISSION_ACK", "MISSION_REQUEST", "MISSION_REQUEST_INT"],
            blocking=False,
        )


def scaled_global_coord(value):
    if abs(value) <= 180.0:
        return int(round(value * 1e7))
    return int(round(value))


def mission_item_for_request(master, item, request_int):
    if not request_int:
        item.target_system = master.target_system
        item.target_component = master.target_component
        return item

    frame = GLOBAL_INT_FRAME.get(item.frame, item.frame)
    is_global_frame = item.frame in GLOBAL_INT_FRAME or frame in GLOBAL_INT_FRAME.values()
    x = scaled_global_coord(item.x) if is_global_frame else int(round(item.x))
    y = scaled_global_coord(item.y) if is_global_frame else int(round(item.y))
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,
        master.target_component,
        item.seq,
        frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        x,
        y,
        item.z,
    )


def upload_mission(master, loader, timeout):
    count = loader.count()
    deadline = time.monotonic() + timeout

    print("[mission_upload] Clearing existing mission")
    master.waypoint_clear_all_send()
    drain_mission_messages(master)

    print(f"[mission_upload] Sending mission count: {count}")
    master.waypoint_count_send(count)

    sent = set()
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = int(msg.seq)
            if seq < 0 or seq >= count:
                raise RuntimeError(f"Vehicle requested invalid mission seq {seq}")

            item = mission_item_for_request(
                master,
                loader.wp(seq),
                request_int=(msg_type == "MISSION_REQUEST_INT"),
            )
            master.mav.send(item)
            sent.add(seq)
            print(f"[mission_upload] Sent item {seq + 1}/{count}")
            continue

        if msg_type == "MISSION_ACK":
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                if len(sent) < count:
                    print("[mission_upload] Ignoring early MISSION_ACK before all items were sent")
                    continue
                missing = sorted(set(range(count)) - sent)
                if missing:
                    print(f"[mission_upload] Accepted; vehicle did not request items {missing}")
                print("[mission_upload] Mission upload accepted")
                return

            raise RuntimeError(f"Mission upload rejected with MAV_MISSION_RESULT={msg.type}")

    raise TimeoutError("Timed out during mission upload")


def verify_mission_count(master, expected_count, timeout=10.0):
    print("[mission_upload] Verifying uploaded mission count")
    master.mav.mission_request_list_send(master.target_system, master.target_component)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=1.0)
        if msg is None:
            continue

        onboard_count = int(msg.count)
        print(f"[mission_upload] Vehicle reports {onboard_count} mission items")
        if onboard_count != expected_count:
            raise RuntimeError(
                f"Mission count mismatch: uploaded {expected_count}, vehicle has {onboard_count}"
            )
        return

    raise TimeoutError("Timed out waiting for mission count verification")


def set_current_mission_item(master, seq):
    print(f"[mission_upload] Setting current mission item to {seq}")
    master.mav.mission_set_current_send(
        master.target_system,
        master.target_component,
        int(seq),
    )


def set_mode(master, mode_name):
    modes = master.mode_mapping()
    if mode_name not in modes:
        raise RuntimeError(f"Mode {mode_name!r} is not available on this vehicle")

    print(f"[mission_upload] Setting mode {mode_name}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes[mode_name],
    )


def arm_vehicle(master, timeout=10.0):
    print("[mission_upload] Arming vehicle")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[mission_upload] Arm accepted")
            return
        raise RuntimeError(f"Arm rejected with MAV_RESULT={msg.result}")

    print("[mission_upload] Arm command sent; no COMMAND_ACK received before timeout")


def main():
    args = parse_args()
    loader, _ = load_mission(args.mission)

    master = mavutil.mavlink_connection(args.connect)
    wait_for_heartbeat(master, args.timeout)

    upload_mission(master, loader, args.timeout)

    if not args.no_verify:
        verify_mission_count(master, loader.count())

    set_current_mission_item(master, args.set_current)

    if args.mode:
        set_mode(master, args.mode)

    if args.arm:
        arm_vehicle(master)

    print("[mission_upload] Done")


if __name__ == "__main__":
    main()
