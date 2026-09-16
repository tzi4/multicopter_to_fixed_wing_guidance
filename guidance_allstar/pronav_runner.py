#!/usr/bin/env python3
"""
pronav_runner.py  –  Proportional Navigation Intercept Runner

Connects to:
  • Pursuer (ArduCopter)  – pymavlink read on 14552, Dronekit send on 14551
  • Target  (ArduPlane)   – pymavlink read on 14600

Runs the PN guidance loop and forwards velocity commands to the pursuer.
"""

import math
import signal
import sys
import time

import guidance_config as cfg
import numpy as np
import vector_math as vecm
from guidance_gui import GuidanceGUI, gui_tick, push_snapshot
from pronavwndr2 import GuidanceLoop
from velocity_control import AttitudeController

# ─── Configuration (from guidance_config.py) ──────────────────
PURSUER_READ_PORT = cfg.PURSUER_CONN_STR
TARGET_READ_PORT = cfg.TARGET_CONN_STR
PURSUER_SEND_PORT = cfg.VEHICLE_CONN_STR
SEND_RATE_HZ = cfg.SEND_RATE_HZ
# ──────────────────────────────────────────────────────────────


def main():
    # --- Graceful shutdown ---
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n[runner] Ctrl+C received — shutting down ...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # --- 1. Initialise guidance (connects to both aircraft via pymavlink) ---
    guidance = GuidanceLoop(
        pursuer_conn_str=PURSUER_READ_PORT,
        target_conn_str=TARGET_READ_PORT,
    )

    # --- 2. Initialise attitude sender (connects to pursuer via Dronekit) ---
    att_ctrl = AttitudeController(
        connection_string=PURSUER_SEND_PORT,
        send_rate_hz=SEND_RATE_HZ,
    )
    sender_thread = att_ctrl.start_sender_thread()

    # --- 2b. Initialise GUI (non‑blocking) ---
    print("[runner] Starting guidance GUI ...")
    gui = GuidanceGUI(param_module="guidance_config", refresh_hz=10)
    gui.start()

    # --- 3. Main guidance loop ---
    dt = guidance.dt
    print(f"\n[runner] ═══ Guidance loop STARTED at {guidance.LOOP_HZ} Hz ═══\n")

    # Maintain yaw state for integration
    yaw_cmd = 0.0
    first_pass = True

    while running:
        t_start = time.monotonic()

        vx_prev, vy_prev, vz_prev = att_ctrl.get_velocity_own()

        result = guidance.step(pursuer_vel_fallback=(vx_prev, vy_prev, vz_prev))

        if result is None:
            print("[runner] Waiting for MAVLink data from both aircraft ...")
            time.sleep(0.5)
            continue

        roll_cmd, pitch_cmd, yaw_cmd, yaw_rate, thrust_req, range_m = result

        # 4. Forward to attitude controller
        att_ctrl.set_command(roll_cmd, pitch_cmd, yaw_cmd, yaw_rate, thrust_req)
        att_ctrl.set_range(range_m)

        # 4b. Push telemetry to GUI
        p_pos = guidance._pursuer_reader.get()[0]
        t_pos = guidance._target_reader.get()[0]
        if p_pos is not None and t_pos is not None:
            push_snapshot(
                pursuer_pos=np.array(p_pos),
                target_pos=np.array(t_pos),
            )
        gui_tick()

        # Rate-limit to guidance frequency
        elapsed = time.monotonic() - t_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- Cleanup ---
    gui.stop()
    att_ctrl.stop()
    sender_thread.join(timeout=1.0)
    print("[runner] Done.")


if __name__ == "__main__":
    main()
