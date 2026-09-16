#!/usr/bin/env python3

import math
import signal
import sys
import threading
import time

import guidance_config as cfg
import matplotlib.pyplot as plt
import mavlink_utils
import numpy as np
import vector_math as vecm
from filterwndr import (
    HeadingTurnRateEstimator,
    IMMLowPassFilter,
    OOSM_IMM_Tracker,
    aggregate_mode_probabilities,
    predict_n_steps_ahead,
    setup_imm_filter,
    update_ct_filter_dynamics,
)
from guidance_gui import GuidanceGUI, gui_tick, push_snapshot
from pymavlink import mavutil
from telemetry_logger import TelemetryLogger
from velocity_control import AttitudeController

GRAVITY_MPS2 = 9.81


def clamp_norm(v, max_norm):
    n = np.linalg.norm(v)
    if n > max_norm and n > 1e-6:
        return v * (max_norm / n)
    return v


def safe_unit(v, fallback):
    n = np.linalg.norm(v)
    if n < 1e-6:
        return fallback.copy()
    return v / n


def yaw_to_los(target_pos, pursuer_pos, fallback_yaw=0.0):
    rel = np.asarray(target_pos, dtype=float).reshape(3) - np.asarray(
        pursuer_pos, dtype=float
    ).reshape(3)
    if rel[0] * rel[0] + rel[1] * rel[1] < 1e-6:
        return fallback_yaw
    return math.atan2(rel[1], rel[0])


def allocate_quad_thrust(ax, ay, az):
    """
    Keep the requested acceleration inside a non-inverted quadcopter envelope.

    NED acceleration is converted to specific thrust f = [ax, ay, az - g].
    A normal quad cannot produce positive fz without inverting, and yaw authority
    collapses when commanded lift goes near zero, so preserve a configurable
    upward lift component and scale XY into the remaining thrust/tilt budget.
    """
    fx = float(ax)
    fy = float(ay)
    fz = float(az) - GRAVITY_MPS2

    max_thrust = max(float(getattr(cfg, "MAX_THRUST", 15.0)), 1e-3)
    max_tilt_deg = float(getattr(cfg, "MAX_TILT_DEG", 45.0))
    max_tilt_rad = math.radians(max(0.0, min(89.0, max_tilt_deg)))
    min_lift = max(0.0, float(getattr(cfg, "LAG_PURSUIT_MIN_LIFT", 0.0)))
    min_lift = min(min_lift, max_thrust)

    raw_thrust_req = math.sqrt(fx * fx + fy * fy + fz * fz)

    if min_lift > 0.0:
        fz = min(fz, -min_lift)
    else:
        fz = min(fz, 0.0)
    fz = max(fz, -max_thrust)

    xy_by_total_thrust = math.sqrt(max(0.0, max_thrust * max_thrust - fz * fz))
    xy_by_tilt = max(0.0, -fz * math.tan(max_tilt_rad))
    xy_limit = min(xy_by_total_thrust, xy_by_tilt)
    xy_req = math.sqrt(fx * fx + fy * fy)
    xy_scale = 1.0

    if xy_req > xy_limit and xy_req > 1e-6:
        xy_scale = xy_limit / xy_req
        fx *= xy_scale
        fy *= xy_scale

    thrust_req = math.sqrt(fx * fx + fy * fy + fz * fz)
    diagnostics = {
        "raw_thrust_req": raw_thrust_req,
        "thrust_req": thrust_req,
        "xy_scale": xy_scale,
        "xy_limit": xy_limit,
        "lift": -fz,
    }
    return fx, fy, fz + GRAVITY_MPS2, diagnostics


class MediumRangeFollower:
    def __init__(self):
        # Formation slot
        self.d_back = 25.0
        self.d_side = 0.0
        self.d_down = 0.0

        # Prediction
        self.tau_pred = 0.30

        # Controller gains, units roughly 1/s
        self.k_along = 0.45
        self.k_cross = 0.75
        self.k_cross_overshoot = 0.9
        self.k_down = 0.45

        # Limits
        self.v_max = float(getattr(cfg, "LAG_PURSUIT_MAX_SPEED", 18.0))
        self.a_max = 5.0
        self.vz_max_up = 3.0  # upward speed limit, NED negative
        self.vz_max_down = 2.0  # downward speed limit, NED positive

        # Overshoot prediction
        self.lookahead = 1.0

        # Turn handling
        self.omega_deadband = 0.015

        # Internal memory
        self.prev_v_cmd = np.zeros(3)
        self.prev_h = np.array([1.0, 0.0, 0.0])

    def update(self, imm_x, dt, p_drone, v_drone):
        # 1. Target state (already predicted via predict_n_steps_ahead outside or inside)
        x_t = imm_x
        p_t = np.asarray(x_t[0:3], dtype=float).reshape(3)
        v_t = np.asarray(x_t[3:6], dtype=float).reshape(3)

        # Optional CT turn rate from your state
        omega_t = float(x_t[9]) if len(x_t) > 9 else 0.0
        if abs(omega_t) < self.omega_deadband:
            omega_t = 0.0

        # 2. Build target-relative horizontal frame
        v_t_horiz = np.array([v_t[0], v_t[1], 0.0])
        h = safe_unit(v_t_horiz, self.prev_h)
        self.prev_h = h.copy()

        # Right vector in local NED horizontal plane
        r = np.array([-h[1], h[0], 0.0])
        down = np.array([0.0, 0.0, 1.0])

        # 3. Optional turn widening
        d_back_eff = self.d_back
        d_side_eff = self.d_side

        if omega_t != 0.0:
            outside_sign = np.sign(omega_t)
            d_side_eff += outside_sign * 5.0
            d_back_eff += min(15.0, abs(omega_t) * 120.0)

        # 4. Desired slot position
        p_slot = p_t - d_back_eff * h + d_side_eff * r + self.d_down * down

        # 5. Feedforward slot velocity
        v_slot = v_t - omega_t * (d_back_eff * r + d_side_eff * h)

        # 6. Error in target-relative frame
        e = p_slot - p_drone
        e_along = float(np.dot(e, h))
        e_cross = float(np.dot(e, r))
        e_down = float(e[2])

        # 7. Overshoot prediction
        v_rel_slot = v_slot - v_drone
        e_along_dot = float(np.dot(v_rel_slot, h))
        e_along_projected = e_along + e_along_dot * self.lookahead

        overshoot = (e_along < 0.0) or (e_along_projected < 0.0)

        # 8. Velocity command
        if not overshoot:
            v_cmd = (
                v_slot
                + self.k_along * e_along * h
                + self.k_cross * e_cross * r
                + self.k_down * e_down * down
            )
            mode = "slot_capture"
        else:
            v_cmd = (
                v_slot
                + self.k_cross_overshoot * e_cross * r
                + self.k_down * e_down * down
            )
            mode = "overshoot_guard"

        # 9. Speed limit
        v_cmd = clamp_norm(v_cmd, self.v_max)

        # 10. Vertical speed limit
        v_cmd[2] = np.clip(v_cmd[2], -self.vz_max_up, self.vz_max_down)

        # 11. Acceleration / slew limit
        a_cmd = (v_cmd - self.prev_v_cmd) / dt
        a_cmd = clamp_norm(a_cmd, self.a_max)

        v_cmd = self.prev_v_cmd + a_cmd * dt
        self.prev_v_cmd = v_cmd.copy()

        # 12. Yaw command: look at target
        rel_to_target = p_t - p_drone
        yaw_cmd = math.atan2(rel_to_target[1], rel_to_target[0])

        diagnostics = {
            "mode": mode,
            "p_slot": p_slot,
            "v_slot": v_slot,
            "e_along": e_along,
            "e_cross": e_cross,
            "e_down": e_down,
            "e_along_projected": e_along_projected,
            "omega_t": omega_t,
            "d_back_eff": d_back_eff,
            "d_side_eff": d_side_eff,
        }

        return v_cmd, a_cmd, yaw_cmd, diagnostics


class PIDController:
    def __init__(self, kp, ki, kd, int_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.int_limit = int_limit

    def update(self, error, derivative, dt):
        self.integral += error * dt
        self.integral = max(-self.int_limit, min(self.int_limit, self.integral))
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def main():
    running = True

    # Collect data for plotting
    time_log = []
    ex_log = []
    ey_log = []
    ez_log = []

    def signal_handler(sig, frame):
        nonlocal running
        print("\n[PID Runner] Ctrl+C received. Stopping and plotting...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    print("[PID Runner] Connecting to telemetry (Pursuer & Target)...")
    pursuer_conn = mavutil.mavlink_connection(cfg.PURSUER_CONN_STR)
    target_conn = mavutil.mavlink_connection(cfg.TARGET_CONN_STR)

    # Get home position
    print("[PID Runner] Waiting for HOME_POSITION...")
    msg = pursuer_conn.recv_match(type="HOME_POSITION", blocking=True, timeout=5.0)
    if msg:
        home_lat = msg.latitude / 1e7
        home_lon = msg.longitude / 1e7
        home_alt = msg.altitude / 1000.0
    else:
        print("[PID Runner] HOME_POSITION timeout, using GLOBAL_POSITION_INT...")
        gp = pursuer_conn.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0
        )
        if gp:
            home_lat = gp.lat / 1e7
            home_lon = gp.lon / 1e7
            home_alt = gp.alt / 1000.0
        else:
            raise RuntimeError("Cannot determine pursuer home position")

    print(
        f"[PID Runner] Home: lat={home_lat:.7f} lon={home_lon:.7f} alt={home_alt:.1f}"
    )

    # Setup readers
    pursuer_reader = mavlink_utils.MavStateReader(
        pursuer_conn,
        ["LOCAL_POSITION_NED", "RC_CHANNELS"],
        mavlink_utils.parse_local_ned,
    )

    target_reader = mavlink_utils.MavStateReader(
        target_conn,
        ["GLOBAL_POSITION_INT", "SCALED_IMU", "ATTITUDE"],
        lambda m: mavlink_utils.parse_global_int(m, home_lat, home_lon, home_alt),
    )

    pursuer_reader.start()
    target_reader.start()

    print("[PID Runner] Waiting for first state from both aircraft...")
    while True:
        p_state, _ = pursuer_reader.get()
        t_state, _ = target_reader.get()
        if p_state is not None and t_state is not None:
            break
        time.sleep(0.1)

    print("[PID Runner] Connecting to vehicle attitude control...")
    att_ctrl = AttitudeController(
        connection_string=cfg.VEHICLE_CONN_STR, send_rate_hz=cfg.SEND_RATE_HZ
    )

    # Create thread for sending attitude commands
    sender_thread = threading.Thread(target=att_ctrl.sender_loop, daemon=True)
    sender_thread.start()

    logger = TelemetryLogger("lag_pursuit_pid.log")

    # PID setup for backup/far modes
    pid_x = PIDController(cfg.LAG_PID_XY_KP, cfg.LAG_PID_XY_KI, cfg.LAG_PID_XY_KD)
    pid_y = PIDController(cfg.LAG_PID_XY_KP, cfg.LAG_PID_XY_KI, cfg.LAG_PID_XY_KD)
    pid_z = PIDController(cfg.LAG_PID_Z_KP, cfg.LAG_PID_Z_KI, cfg.LAG_PID_Z_KD)

    FOLLOW_DIST = cfg.LAG_PURSUIT_DIST

    # Medium Range Follower
    medium_follower = MediumRangeFollower()
    # Apply standard lag distance dynamically
    medium_follower.d_back = FOLLOW_DIST

    # State machine ranges
    R_medium_enter = 85.0
    R_close_exit = 15.0

    dt = 1.0 / cfg.LOOP_HZ

    # Setup IMM Estimator for clean target state derivatives
    imm = setup_imm_filter(dt)
    oosm_tracker = OOSM_IMM_Tracker(
        imm, dt, max_history_size=50
    )  # Maintain ~1.0s history at 50Hz
    lpf = IMMLowPassFilter()
    turn_rate_estimator = HeadingTurnRateEstimator()
    first_imm_update = True
    last_fast_turn_onset = False
    z_update_freeze_remaining = 0

    t_start = time.monotonic()

    start_time_log = time.time()

    print(f"[PID Runner] Loop started at {cfg.LOOP_HZ} Hz.")

    # --- Start GUI ---
    print("[PID Runner] Starting guidance GUI ...")
    gui = GuidanceGUI(param_module="guidance_config", refresh_hz=10)
    gui.start()

    try:
        while running:
            t_loop = time.monotonic()

            p_pos, p_vel = pursuer_reader.get()
            t_pos, t_vel = target_reader.get()

            if p_pos is None or t_pos is None:
                time.sleep(0.01)
                continue

            px, py, pz = p_pos
            pvx, pvy, pvz = p_vel
            tx, ty, tz = t_pos
            _, _, _ = t_vel  # We will overwrite the raw velocity with the IMM estimate

            # Predict and Update the IMM filter with the raw target position
            z_meas = np.array([tx, ty, tz])
            turn_rate_estimator.update(z_meas, t_loop)
            if first_imm_update:
                for f in imm.filters:
                    f.x[0:3] = [tx, ty, tz]
                imm.x = imm.filters[0].x.copy()
                oosm_tracker._record_state()
                first_imm_update = False
            else:
                was_fast_turn_onset = last_fast_turn_onset
                last_fast_turn_onset = turn_rate_estimator.fast_onset_strength > 0.0
                if last_fast_turn_onset and not was_fast_turn_onset:
                    z_update_freeze_remaining = max(
                        z_update_freeze_remaining,
                        max(0, int(getattr(cfg, "Z_UPDATE_FREEZE_PACKETS", 2))),
                    )

                # E.g. Simulate a 100ms delayed image processing measurement by sending an older timestamp
                simulated_camera_time = t_loop - 0.100

                # Advance internal clock by 1 tick and process the OOSM math
                oosm_tracker.step_forward()
                freeze_z_update = z_update_freeze_remaining > 0
                oosm_tracker.process_delayed_measurement(
                    z_meas,
                    simulated_camera_time,
                    freeze_z_update=freeze_z_update,
                )
                if freeze_z_update:
                    z_update_freeze_remaining -= 1

            # Execute 250ms prediction for the intercept vector mapping
            pred_steps = int(0.250 / dt)
            predicted_x = predict_n_steps_ahead(imm, dt, pred_steps)
            predicted_x = lpf.filter(predicted_x, dt)

            tx, ty, tz = predicted_x[0:3].flatten()
            tvx, tvy, tvz = predicted_x[3:6].flatten()
            p_drone = np.array([px, py, pz])
            v_drone = np.array([pvx, pvy, pvz])

            range_to_target = np.linalg.norm(np.array([tx, ty, tz]) - p_drone)

            if range_to_target > R_medium_enter:
                state = "far_chase"
            elif R_close_exit < range_to_target <= R_medium_enter:
                state = "medium_slot_follow"
            else:
                state = "close_abort_or_other_rule_defined_behavior"

            ax, ay, az = 0.0, 0.0, 0.0

            if state == "medium_slot_follow":
                v_cmd, a_cmd, yaw_cmd, diag = medium_follower.update(
                    predicted_x, dt, p_drone, v_drone
                )
                ax, ay, az = a_cmd[0], a_cmd[1], a_cmd[2]

                # For plotting legacy compatibility
                ex, ey, ez = diag["e_cross"], diag["e_along"], diag["e_down"]

                if logger.step_count % 5 == 0:
                    print(
                        f"State={state} "
                        f"mode={diag['mode']} "
                        f"e_along={diag['e_along']:.2f} "
                        f"e_cross={diag['e_cross']:.2f} "
                        f"e_proj={diag['e_along_projected']:.2f} "
                        f"omega={diag['omega_t']:.3f} "
                        f"v_cmd={np.linalg.norm(v_cmd):.2f}"
                    )
            else:
                # 1. Fallback to basic PID for far_chase or close_abort
                t_speed = math.sqrt(tvx**2 + tvy**2 + tvz**2)
                if t_speed > 0.5:
                    dir_x = tvx / t_speed
                    dir_y = tvy / t_speed
                    dir_z = tvz / t_speed
                else:
                    rel_d, rx, ry, rz = vecm.calculate_distance_vector(
                        tx, ty, tz, px, py, pz
                    )
                    if rel_d > 0.1:
                        dir_x = rx / rel_d
                        dir_y = ry / rel_d
                        dir_z = rz / rel_d
                    else:
                        dir_x, dir_y, dir_z = 1.0, 0.0, 0.0

                sp_x = tx - dir_x * FOLLOW_DIST
                sp_y = ty - dir_y * FOLLOW_DIST
                sp_z = tz + cfg.Z_STEADY_ERROR

                ex = sp_x - px
                ey = sp_y - py
                ez = sp_z - pz

                dex = tvx - pvx
                dey = tvy - pvy
                dez = tvz - pvz

                ax = pid_x.update(ex, dex, dt)
                ay = pid_y.update(ey, dey, dt)
                az = pid_z.update(ez, dez, dt)

                if logger.step_count % 5 == 0:
                    print(f"State={state}")

            # Log errors
            curr_t = time.time() - start_time_log
            time_log.append(curr_t)
            ex_log.append(ex)
            ey_log.append(ey)
            ez_log.append(ez)

            ax, ay, az, thrust_diag = allocate_quad_thrust(ax, ay, az)

            # 3. Always face the target completely (Yaw)
            if state != "medium_slot_follow":
                _, rx, ry, _ = vecm.calculate_distance_vector(tx, ty, tz, px, py, pz)
                yaw_cmd = math.atan2(ry, rx)

            yaw_lock_enabled = bool(getattr(cfg, "YAW_LOCK_ENABLED", False))
            if yaw_lock_enabled:
                yaw_cmd = yaw_to_los([tx, ty, tz], p_drone, yaw_cmd)

            # 4. Convert accel to euler
            roll_cmd, pitch_cmd, yaw_rate, thrust_req = vecm.accel_to_euler_ef(
                ax, ay, az, yaw_cmd, pvx, pvy
            )
            if yaw_lock_enabled:
                yaw_rate = 0.0

            # 5. Send command
            att_ctrl.set_command(roll_cmd, pitch_cmd, yaw_cmd, yaw_rate, thrust_req)

            dist_to_target = math.sqrt((tx - px) ** 2 + (ty - py) ** 2 + (tz - pz) ** 2)
            att_ctrl.set_range(dist_to_target)

            # Log
            logger.log_step(
                {
                    "timestamp": time.time(),
                    "mode": "PID_LAG",
                    "range": dist_to_target,
                    "px": px,
                    "py": py,
                    "pz": pz,
                    "tx": tx,
                    "ty": ty,
                    "tz": tz,
                    "pvx": pvx,
                    "pvy": pvy,
                    "pvz": pvz,
                    "tvx": tvx,
                    "tvy": tvy,
                    "tvz": tvz,
                    "cmd_acx": ax,
                    "cmd_acy": ay,
                    "cmd_acz": az,
                    "cmd_roll": roll_cmd,
                    "cmd_pitch": pitch_cmd,
                    "cmd_yaw": yaw_cmd,
                    "cmd_yaw_rate": yaw_rate,
                    "raw_thrust_req": thrust_diag["raw_thrust_req"],
                    "thrust_req": thrust_diag["thrust_req"],
                    "thrust_xy_scale": thrust_diag["xy_scale"],
                    "thrust_lift": thrust_diag["lift"],
                }
            )

            elapsed = time.monotonic() - t_loop
            if elapsed < dt:
                time.sleep(dt - elapsed)

            # Push telemetry to GUI
            push_snapshot(
                pursuer_pos=np.array([px, py, pz]),
                target_pos=np.array([tx, ty, tz]),
                mode_probs=aggregate_mode_probabilities(imm),
            )
            gui_tick()

    except KeyboardInterrupt:
        pass
    finally:
        print("[PID Runner] Cleaning up...")
        running = False
        att_ctrl.stop()
        logger.close()

        # Plotting
        print("[PID Runner] Plotting errors...")
        plt.figure(figsize=(10, 6))
        plt.plot(time_log, ex_log, label="X Error")
        plt.plot(time_log, ey_log, label="Y Error")
        plt.plot(time_log, ez_log, label="Z Error")
        plt.title("PID Error over Time (Lag pursuit setpoint)")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.legend()
        plt.grid(True)
        plt.savefig("pid_lag_pursuit_errors.png")
        plt.show()


if __name__ == "__main__":
    main()
