import math
import time
import numpy as np
import threading
from pymavlink import mavutil
import guidance_config as cfg

import vector_math
import mavlink_utils
from filterwndr import setup_imm_filter
from velocity_control import AttitudeController
from telemetry_logger import TelemetryLogger

def run_frpn():
    print("[frpn] Starting FRPN runner...")
    
    # ----------------------------------------------------
    # 1. Setup Connections & Readers
    # ----------------------------------------------------
    print("[frpn] Connecting to Pursuer (ArduCopter)...")
    pursuer_conn_str = cfg.PURSUER_CONN_STR if hasattr(cfg, 'PURSUER_CONN_STR') else 'udpin:localhost:14552'
    pursuer_conn = mavutil.mavlink_connection(pursuer_conn_str)
    pursuer_conn.wait_heartbeat()
    print("[frpn] Pursuer heartbeat received.")
    
    pursuer_reader = mavlink_utils.MavStateReader(
        pursuer_conn,
        ['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'],
        lambda msg: mavlink_utils.parse_local_ned(msg) if msg.get_type() == 'LOCAL_POSITION_NED' else mavlink_utils.parse_global_int(msg)
    )
    pursuer_reader.start()

    print("[frpn] Connecting to Target (ArduPlane)...")
    target_conn_str = cfg.TARGET_CONN_STR if hasattr(cfg, 'TARGET_CONN_STR') else 'udpin:localhost:14600'
    target_conn = mavutil.mavlink_connection(target_conn_str)
    target_conn.wait_heartbeat()
    print("[frpn] Target heartbeat received.")
    
    target_reader = mavlink_utils.MavStateReader(
        target_conn,
        ['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'],
        lambda msg: mavlink_utils.parse_local_ned(msg) if msg.get_type() == 'LOCAL_POSITION_NED' else mavlink_utils.parse_global_int(msg, 0.0, 0.0, 0.0)
    )
    target_reader.start()

    # Attitude Controller (Dronekit on 14551)
    veh_conn_str = cfg.VEHICLE_CONN_STR if hasattr(cfg, 'VEHICLE_CONN_STR') else '127.0.0.1:14551'
    att_ctrl = AttitudeController(connection_string=veh_conn_str, send_rate_hz=getattr(cfg, 'LOOP_HZ', 50))
    att_ctrl.start_sender_thread()

    # IMM Filter
    RUN_HZ = getattr(cfg, 'LOOP_HZ', 50)
    dt = 1.0 / RUN_HZ
    imm = setup_imm_filter(dt)
    
    # State tracking
    current_yaw_cmd = None
    recovery_end_time = 0.0
    a_cmd_filtered_prev = np.array([0.0, 0.0, 0.0])
    a_cmd_filtered = np.array([0.0, 0.0, 0.0])
    transition_elapsed = 0.0
    t_desired_prev = 0.0
    last_target_stamp = 0.0
    
    is_miss_recovery = False
    is_terminal_frozen = False
    last_ax_cmd, last_ay_cmd, last_az_cmd = 0.0, 0.0, 0.0
    frozen_ax_cmd, frozen_ay_cmd, frozen_az_cmd = 0.0, 0.0, 0.0
    
    # Initialize logger
    logger = TelemetryLogger()
    
    print("[frpn] Entering main loop...")
    
    imm_start_time = time.monotonic()
    print("[frpn] Spinning up IMM estimator for 5 seconds...")
    while True:
        t_start = time.monotonic()
        
        #  -- Step 1: Pull pursuer pos and vel from ardupilot ---
        p_pos, p_vel = pursuer_reader.get()
        
        # --- Step 2: Pull target pos, vel, and accel ---
        t_meas_pos, _, t_stamp = target_reader.get_with_stamp()
        
        if p_pos is None or p_vel is None or t_meas_pos is None:
            time.sleep(0.01)
            continue
            
        from filterwndr import update_ct_filter_dynamics
        update_ct_filter_dynamics(imm, dt)
        
        imm.predict()

        # Update IMM only if we got a new target telemetry packet
        if t_stamp > last_target_stamp:
            z_meas = np.array([t_meas_pos[0], t_meas_pos[1], t_meas_pos[2]])
            imm.update(z_meas)
            last_target_stamp = t_stamp
        
        # Allow 5 seconds for the IMM filter to converge before generating commands
        if (time.monotonic() - imm_start_time) < 5.0:
            time.sleep(max(0, dt - (time.monotonic() - t_start)))
            continue
            
        prediction_steps = getattr(cfg, 'FRPN_PREDICTION_STEPS', 1)
        if prediction_steps > 1:
            from filterwndr import predict_n_steps_ahead
            predicted_state = predict_n_steps_ahead(imm, dt, steps=prediction_steps)
            target_pos = predicted_state[0:3].flatten()
            target_vel = predicted_state[3:6].flatten()
            target_accel = predicted_state[6:9].flatten()
        else:
            target_pos = imm.x[0:3].flatten()
            target_vel = imm.x[3:6].flatten()
            target_accel = imm.x[6:9].flatten()
        
        # --- Step 3: Run guidance law to compute desired accel ---
        relpos = target_pos - p_pos
        relvel = target_vel - p_vel
        range_m = np.linalg.norm(relpos)
        
        # Closing velocity (Vc) and pursuer speed
        closing_vel = -np.dot(relpos, relvel) / range_m if range_m > 1e-7 else 0.0
        pv_mag = np.linalg.norm(p_vel)

        # Calculate Zero Effort Miss (ZEM)
        relpos_cross_relvel = np.cross(relpos, relvel)
        omega_mag = np.linalg.norm(relpos_cross_relvel) / (range_m**2 + 1e-6)
        omega_mag_clamped = min(omega_mag, getattr(cfg, 'MAX_OMEGA', 2.0))
        zem = (range_m**2 * omega_mag_clamped) / max(closing_vel, 0.1)

        miss_range = getattr(cfg, 'MISS_DETECT_RANGE', 10.0)
        zem_limit = getattr(cfg, 'ZEM_LIMIT', 1.0)
        
        # --- State Machine Update ---
        if not is_miss_recovery:
            if range_m <= miss_range and closing_vel > 0.0:
                if not is_terminal_frozen:
                    if zem < zem_limit:
                        is_terminal_frozen = True
                        frozen_ax_cmd, frozen_ay_cmd, frozen_az_cmd = last_ax_cmd, last_ay_cmd, last_az_cmd
                        print(f"[ZEM FSM] Terminal approach! ZEM={zem:.2f}m < {zem_limit}m. Freezing guidance.")
                    else:
                        is_miss_recovery = True
                        is_terminal_frozen = False
                        print(f"[ZEM FSM] Miss predicted! ZEM={zem:.2f}m > {zem_limit}m. Swinging into Lag Pursuit.")
            elif closing_vel < -1.0: 
                # Target zipped past us! We are now opening range.
                is_miss_recovery = True
                is_terminal_frozen = False
                print(f"[ZEM FSM] Target passed! Closing Vel: {closing_vel:.2f} m/s. Initiating Turnaround.")
            elif range_m > miss_range:
                is_terminal_frozen = False
                
        # Recovery exit condition: switch back once we've opened some distance and are closing again
        if is_miss_recovery and range_m > miss_range * 1.5 and closing_vel > 0.0:
            is_miss_recovery = False
            print("[ZEM FSM] Re-engaging FRPN mid-course!")

        # --- Active Mode Selection ---
        if is_miss_recovery:
            active_mode = "PURSUIT"
        elif is_terminal_frozen:
            active_mode = "TERMINAL_FREEZE"
        else:
            use_frpn = closing_vel > getattr(cfg, 'APN_ENGAGE_VC_MIN', 5.0) and pv_mag > getattr(cfg, 'APN_ENGAGE_SPEED_MIN', 20.0)
            active_mode = "FRPN" if use_frpn else "PURSUIT"

        if active_mode == "TERMINAL_FREEZE":
            ax_cmd, ay_cmd, az_cmd = frozen_ax_cmd, frozen_ay_cmd, frozen_az_cmd

        elif active_mode == "FRPN":
            tgo = range_m / (np.linalg.norm(relvel) + 1e-6)

            navgain = cfg.NAV_GAIN
            weight = cfg.WEIGHTING_GAIN

            ax_png = navgain * (((1-weight) * (relpos[0] + relvel[0] * tgo) + weight * relpos[0]) / (tgo*tgo))
            ay_png = navgain * (((1-weight) * (relpos[1] + relvel[1] * tgo) + weight * relpos[1]) / (tgo*tgo))
            
            if getattr(cfg, 'Z_AXIS_PN', True):
                az_png = navgain * (((1-weight) * (relpos[2] + relvel[2] * tgo) + weight * relpos[2]) / (tgo*tgo))
            else:
                pd_kp = getattr(cfg, 'PD_KP', 2.0)
                pd_kd = getattr(cfg, 'PD_KD', 1.5)
                az_png = pd_kp * relpos[2] + pd_kd * relvel[2]
                
            # 2. Dynamic Saturated PNG combined with Pursuit Blending
            t_max_accel = getattr(cfg, 'MAX_THRUST', 15.0)
            g_val = 9.81
            a_lat_max = math.sqrt(max(0, t_max_accel**2 - g_val**2))
            
            # We evaluate lateral capability primarily, since Z mostly opposes gravity
            a_cmd_lat_mag = math.sqrt(ax_png**2 + ay_png**2)
            rho = min(a_cmd_lat_mag / a_lat_max, 1.0) if a_lat_max > 1e-3 else 1.0

            # 1. Address the first-order lag trap for Quadcopters (The Blind-Zone Fix)
            # Instead of decaying NavGain to 0 (which flattens the copter into an airbrake), 
            # we force rho -> 1.0 right before impact. This cleanly locks the drone into Pure Pursuit 
            # along the LOS vector, maintaining speed and turning it into a stable kinetic spear.
            tau = getattr(cfg, 'ATTITUDE_TAU', 0.2) # ~200ms time constant for standard quad
            if tgo < 4.0 * tau:
                override_factor = 1.0 - max(0.0, tgo / (4.0 * tau))
                rho = max(rho, override_factor) 
                
            # Blending function f(rho), utilizing x^3 for smooth ease-in at higher rho
            f_rho = rho ** 3
            
            # Pure pursuit acceleration command for blending (NO DECEL_RANGE BRAKING FOR KINETIC IMPACT)
            if range_m > 1e-7:
                los_x, los_y, los_z = relpos / range_m
            else:
                los_x, los_y, los_z = 0.0, 0.0, 0.0
                
            pursuit_speed = getattr(cfg, 'SPEED_MAX', 40.0) 
            current_speed_along_los = (p_vel[0] * los_x) + (p_vel[1] * los_y) + (p_vel[2] * los_z)
            speed_err = pursuit_speed - current_speed_along_los
            accel_mag = speed_err * getattr(cfg, 'PURSUER_KP', 3.0)
            
            ax_pp = los_x * accel_mag
            ay_pp = los_y * accel_mag
            
            if getattr(cfg, 'Z_AXIS_PN', True):
                az_pp = los_z * accel_mag
                az_cmd = (1.0 - f_rho) * az_png + f_rho * az_pp
            else:
                # Isolate Z axis tracking from horizontal pursuit blending
                az_cmd = az_png
                
            # Apply dynamic blending
            ax_cmd = (1.0 - f_rho) * ax_png + f_rho * ax_pp
            ay_cmd = (1.0 - f_rho) * ay_png + f_rho * ay_pp
            
        else: # PURSUIT mode
            # Quadcopter Pure Pursuit: directly accelerate along Line-Of-Sight (LOS)
            if range_m > 1e-7:
                los_x, los_y, los_z = relpos / range_m
            else:
                los_x, los_y, los_z = 0.0, 0.0, 0.0
                
            pursuit_speed = getattr(cfg, 'SPEED_MAX', 40.0)
            
            # Calculate speed error along LOS
            current_speed_along_los = (p_vel[0] * los_x) + (p_vel[1] * los_y) + (p_vel[2] * los_z)
            speed_err = pursuit_speed - current_speed_along_los
            
            # Proportional speed controller commands thrust cleanly backwards or forwards
            accel_mag = speed_err * getattr(cfg, 'PURSUER_KP', 3.0)
            
            ax_cmd = los_x * accel_mag
            ay_cmd = los_y * accel_mag
            
            if getattr(cfg, 'Z_AXIS_PN', True):
                az_cmd = los_z * accel_mag
            else:
                pd_kp = getattr(cfg, 'PD_KP', 2.0)
                pd_kd = getattr(cfg, 'PD_KD', 1.5)
                az_cmd = pd_kp * relpos[2] + pd_kd * relvel[2]

        # --- Infinite Jerk / Smoothing ---
        # Smooth the raw guidance commands first before any emergency overrides
        v_mag = np.linalg.norm(p_vel)
        if v_mag > 0.1 and range_m > 0.1:
            los_vec = relpos / range_m
            v_dir = p_vel / v_mag
            dot_val = np.clip(np.dot(v_dir, los_vec), -1.0, 1.0)
            angle_error = math.acos(dot_val)
        else:
            angle_error = 0.0
            
        rate_limit = math.radians(getattr(cfg, 'MAX_TURN_DEG', 80.0))
        if rate_limit < 1e-3: rate_limit = 1.0
        
        t_min = getattr(cfg, 'TRANSITION_T_MIN', 0.3)
        t_max = getattr(cfg, 'TRANSITION_T_MAX', 1.5)
        k_margin = getattr(cfg, 'TRANSITION_K_MARGIN', 1.2)
        
        t_desired = np.clip((angle_error / rate_limit) * k_margin, t_min, t_max)
        
        cmd_raw_vec = np.array([ax_cmd, ay_cmd, az_cmd])
        cmd_diff = np.linalg.norm(cmd_raw_vec - a_cmd_filtered)
        
        # FIX: The transition_elapsed logic was causing the filter to get "stuck" at alpha=0
        # If transition_elapsed is reset to 0, alpha becomes 0.
        # If alpha is 0, a_cmd_filtered = a_cmd_filtered_prev.
        # If the command suddenly changes, alpha resets to 0, locking the output to the old value,
        # until the timer slowly ticks up. But if it keeps changing slightly, it keeps resetting to 0!
        
        # We only reset the filter if the command diff is massive, AND the previous transition finished.
        # BUT, if we reset transition_elapsed to 0, alpha=0, so it freezes at the previous value!
        # This is actually correct for a smooth transition from the PREVIOUS value.
        # The real bug: transition_elapsed must advance.
        
        # Let's use a simpler standard EMA if not transitioning a huge jump to prevent zero-locking.
        if cmd_diff > getattr(cfg, 'JERK_RESET_THRESH', 2.0):
            # NEW: Initialize filtered prev to the current filtered output before resetting for smooth start
            a_cmd_filtered_prev = a_cmd_filtered
            transition_elapsed = 0.0
            t_desired_prev = t_desired
            
        if transition_elapsed < t_desired_prev and t_desired_prev > 1e-4:
            transition_elapsed += dt
            s = np.clip(transition_elapsed / t_desired_prev, 0.0, 1.0)
            alpha_trans = 10*(s**3) - 15*(s**4) + 6*(s**5)
            a_cmd_filtered = alpha_trans * cmd_raw_vec + (1.0 - alpha_trans) * a_cmd_filtered_prev
            # Do NOT update a_cmd_filtered_prev here to the current blended value, 
            # otherwise it exponentially decays incorrectly during the sweep.
            # a_cmd_filtered_prev should remain the "anchor" start point for the duration of the transition.
        else:
            # Not in a major transition, just use light EMA smoothing
            alpha_ema = getattr(cfg, 'CMD_SMOOTHING_ALPHA', 0.8) # Default high for reactivity
            if alpha_ema > 1.0: alpha_ema = 1.0
            if alpha_ema < 0.0: alpha_ema = 0.0
            a_cmd_filtered = alpha_ema * cmd_raw_vec + (1.0 - alpha_ema) * a_cmd_filtered_prev
            a_cmd_filtered_prev = a_cmd_filtered # Update anchor for EMA
            
        ax_cmd, ay_cmd, az_cmd = a_cmd_filtered[0], a_cmd_filtered[1], a_cmd_filtered[2]

        # --- Altitude protection (linear proportional) ---
        # Intervene after smoothing so emergency pull-ups are instant
        try:
            min_alt = cfg.MIN_ALT_M
            kp_alt = cfg.KP_ALT
        except AttributeError:
            min_alt = 15.0
            kp_alt = 3.0
            
        alt_m = -p_pos[2]
        alt_error = min_alt - alt_m  # positive means we are too low
        if alt_error > 0:
            # NED: negative Z is UP. Linearly proportional to alt_error.
            # If we are below min altitude and the current command is not pushing up enough, override.
            az_cmd = min(az_cmd, -kp_alt * alt_error) # Command more negative (UP) acceleration

        # --- Output clamping (Dynamic) / Thrust Budgeting ---
        # Final gatekeeper: enforce physical limits on the requested commands
        g_val = 9.81
        
        # Get the absolute MAX total thrust the drone can physically produce from config. 
        # (e.g., 12.0 = 12.0 m/s^2, meaning 2.19 m/s^2 leftover for XY maneuvers after 9.81 of gravity).
        t_max = getattr(cfg, 'MAX_THRUST', 15.0)

        # 1. calculate needed thrust to stay airborne: a_z_cmd - g_val
        thrust_z = az_cmd - g_val
        thrust_z = min(thrust_z, 0.0)
        
        # 2. check if T_z exceeds T_max -> limit it (prioritize Z axis entirely)
        if abs(thrust_z) > t_max:
            thrust_z = -t_max if thrust_z < 0 else t_max
            # Re-calculate az_cmd from the limited thrust_z
            az_cmd = thrust_z + g_val
            
        # 3. calculate thrust budget leftover for XY maneuvers
        t_xy_thrust_limit = math.sqrt(max(0, t_max**2 - thrust_z**2))
        
        # 4. calculate a geometric limit bounds on XY to prevent max tilt angle violations.
        # If we command excessive XY, the attitude controller will clamp the tilt angle, 
        # which fundamentally ruins the Z-component thrust if total thrust is maxed out.
        max_tilt_rad = math.radians(getattr(cfg, 'MAX_TILT_DEG', 60.0))
        t_xy_tilt_limit = abs(thrust_z) * math.tan(max_tilt_rad)

        # 5. True XY budget is the strictest of the physical limits
        t_xy_max = min(t_xy_thrust_limit, t_xy_tilt_limit)
        
        # 6. T_xy_req
        t_xy_req = math.sqrt(ax_cmd**2 + ay_cmd**2)
        
        # 7. if T_xy_req > t_xy_max then scale only x,y components of the accel cmd
        if t_xy_req > t_xy_max and t_xy_req > 1e-6:
            scale = t_xy_max / t_xy_req
            ax_cmd *= scale
            ay_cmd *= scale

        # --- Step 4: Send desired accel to velocity_control.py ---
        # Fixed-wing logic fails for quadcopters because pitching to brake causes the 
        # velocity vector to rapidly cross zero and flip 180, causing terrifying yaw spins.
        # 1. In standard FRPN Chase (flying forward), it's safe to face the velocity vector.
        # 2. In Terminal Freeze or Pursuit (Miss Recovery/Braking), ALWAYS definitively face the target (LOS).
        if active_mode == "FRPN" and (p_vel[0]**2 + p_vel[1]**2) > 1.0:
            target_yaw = math.atan2(p_vel[1], p_vel[0])
        else:
            target_yaw = math.atan2(relpos[1], relpos[0])
            
        if current_yaw_cmd is None:
            current_yaw_cmd = target_yaw
        else:
            # Ensure shortest path for wrapping
            yaw_err = target_yaw - current_yaw_cmd
            while yaw_err > math.pi: yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi
            current_yaw_cmd += yaw_err
                
        roll_cmd, pitch_cmd, yaw_rate, thrust_req = vector_math.accel_to_euler_ef(
            ax_cmd, ay_cmd, az_cmd, current_yaw_cmd, p_vel[0], p_vel[1]
        )
        
        # We no longer integrate yaw_rate into current_yaw_cmd because 
        # the yaw target is explicitly locked to the velocity vector above.
            
        att_ctrl.set_command(roll_cmd, pitch_cmd, current_yaw_cmd, yaw_rate, thrust_req)
        
        # Store last commands for ZEM terminal freeze logic
        last_ax_cmd, last_ay_cmd, last_az_cmd = ax_cmd, ay_cmd, az_cmd
        
        # Logging
        range_m = np.linalg.norm(relpos)
        closing_vel = -np.dot(relpos, relvel) / range_m if range_m > 0 else 0.0
        
        logger.log_step({
            "timestamp": time.time(),
            "mode": active_mode,
            "px": p_pos[0], "py": p_pos[1], "pz": p_pos[2],
            "pvx": p_vel[0], "pvy": p_vel[1], "pvz": p_vel[2],
            "tx": target_pos[0], "ty": target_pos[1], "tz": target_pos[2],
            "tvx": target_vel[0], "tvy": target_vel[1], "tvz": target_vel[2],
            "tax": target_accel[0], "tay": target_accel[1], "taz": target_accel[2],
            "range": range_m,
            "closing_velocity": closing_vel,
            "cmd_acx": ax_cmd, "cmd_acy": ay_cmd, "cmd_acz": az_cmd,
            "cmd_roll": roll_cmd, "cmd_pitch": pitch_cmd, "cmd_yaw_rate": yaw_rate
        })
        
        # Sleep for remainder of dt
        elapsed = time.monotonic() - t_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

if __name__ == "__main__":
    run_frpn()
