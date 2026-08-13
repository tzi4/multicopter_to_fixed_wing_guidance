import math
import time
import threading
from pymavlink import mavutil
import guidance_config as cfg

import vector_math
import mavlink_utils

hit_req_range = cfg.HIT_REQ_RANGE

# ============================================================
#  Core Guidance Function
# ============================================================

def _compute_los_rate(rx, ry, rz, dist, vrel_x, vrel_y, vrel_z, los_rate_method, prev_los, los_x, los_y, los_z, dt):
    """Calculate the Line-Of-Sight (LOS) angular rate vector (omega)."""
    if dist <= hit_req_range:
        return 0.0, 0.0, 0.0
        
    if los_rate_method == "diff" and prev_los is not None:
        omega_x = (los_x - prev_los[0]) / dt
        omega_y = (los_y - prev_los[1]) / dt
        omega_z = (los_z - prev_los[2]) / dt
    else:
        cx, cy, cz = vector_math.vector_cross_product(rx, ry, rz, vrel_x, vrel_y, vrel_z)
        omega_x = cx / (dist**2)
        omega_y = cy / (dist**2)
        omega_z = cz / (dist**2)

    omega_mag = math.sqrt(omega_x**2 + omega_y**2 + omega_z**2)
    if omega_mag > cfg.MAX_OMEGA:
        s = cfg.MAX_OMEGA / omega_mag
        omega_x *= s
        omega_y *= s
        omega_z *= s
        
    return omega_x, omega_y, omega_z

def _compute_pd_acceleration(rx, ry, rz, vrel_raw_x, vrel_raw_y, vrel_raw_z):
    """Calculate 3D Proportional-Derivative acceleration for terminal homing."""
    G_NED_Z = -9.81
    acx = cfg.PD_KP * rx + cfg.PD_KD * vrel_raw_x
    acy = cfg.PD_KP * ry + cfg.PD_KD * vrel_raw_y
    acz = cfg.PD_KP * rz + cfg.PD_KD * vrel_raw_z + G_NED_Z
    return acx, acy, acz

def _compute_pn_acceleration(guidance_mode, navigation_constant, closing_velocity, omega_x, omega_y, omega_z, los_x, los_y, los_z, tax, tay, pvx, pvy, pvz):
    """Calculate Proportional Navigation acceleration commands."""
    if guidance_mode in ("TPN", "APN"):
        cx, cy, cz = vector_math.vector_cross_product(omega_x, omega_y, omega_z, los_x, los_y, los_z)
        scale = navigation_constant * closing_velocity
        acx = scale * cx
        acy = scale * cy
        acz = scale * cz if cfg.Z_AXIS_PN else 0.0
        if guidance_mode == "APN":
            acx += navigation_constant * tax / 2.0
            acy += navigation_constant * tay / 2.0
    else:  # PPN
        acx, acy, acz_raw = vector_math.vector_cross_product(omega_x, omega_y, omega_z, pvx, pvy, pvz)
        acx *= navigation_constant
        acy *= navigation_constant
        acz = acz_raw * navigation_constant if cfg.Z_AXIS_PN else 0.0
    return acx, acy, acz


def _apply_tail_chase_heuristic(closing_velocity, guidance_mode, acx, acy, pvx, pvy, pvz, rx, ry, rz):
    if closing_velocity >= 0 or guidance_mode in ("APN", "PD", "PURSUIT"):
        return acx, acy
        
    nx, ny, nz = vector_math.vector_cross_product(pvx, pvy, pvz, rx, ry, rz)
    nm = math.sqrt(nx**2 + ny**2 + nz**2)
    if nm > hit_req_range:
        nx, ny, nz = nx / nm, ny / nm, nz / nm
    else:
        nx, ny, nz = vector_math.vector_cross_product(pvx, pvy, pvz, 0, 0, 1)
        nm = math.sqrt(nx**2 + ny**2 + nz**2)
        if nm < hit_req_range:
            nx, ny, nz = vector_math.vector_cross_product(pvx, pvy, pvz, 0, 1, 0)
            nm = math.sqrt(nx**2 + ny**2 + nz**2)
        if nm > hit_req_range:
            nx, ny, nz = nx / nm, ny / nm, nz / nm
            
    heuristic = 0.5
    dx, dy, _ = vector_math.vector_cross_product(nx, ny, nz, pvx, pvy, pvz)
    return dx * heuristic, dy * heuristic

def compute_guidance_command(
    pursuer_pos,        # (x, y, z)  NED metres
    pursuer_vel,        # (vx, vy, vz) NED m/s
    target_pos,         # (x, y, z)  NED metres
    target_vel,         # (vx, vy, vz) NED m/s
    dt,                 # time-step  [s]
    max_turn_deg,       # max turn-rate limit  [deg/s]
    speed_min,          # stall speed  [m/s]
    speed_max,          # max speed    [m/s]
    navigation_constant,# N (typically 3–5)
    guidance_mode="PPN",
    target_accel=(0.0, 0.0, 0.0),
    los_rate_method="cross",
    prev_los=None,
    transition_multiplier=1.0,
    telemetry_dict=None,
):
    """
    Pure Proportional Navigation guidance step.

    Parameters
    ----------
    pursuer_pos, pursuer_vel : tuple (x,y,z)
        Pursuer position and velocity in NED frame.
    target_pos, target_vel : tuple (x,y,z)
        Target position and velocity in NED frame.
    dt : float
        Time-step in seconds.
    max_turn_deg : float
        Maximum turn-rate the pursuer can sustain [deg/s].
    speed_min, speed_max : float
        Pursuer airspeed limits [m/s].
    navigation_constant : float
        PN gain N.
    guidance_mode : str
        "APN", "TPN", "PPN", or "PPN_modified".
    target_accel : tuple (ax,ay,az)
        Target acceleration (used by APN and PPN_modified).

    Returns
    -------
    (ax_cmd, ay_cmd, az_cmd) : tuple of float
        Commanded acceleration [m/s^2].
    """
    px, py, pz = pursuer_pos
    pvx, pvy, pvz = pursuer_vel
    
    tx, ty, tz = target_pos
    tvx, tvy, tvz = target_vel
    tax, tay, taz = target_accel

    # --- 1. Relative geometry ---
    rx = tx - px
    ry = ty - py
    rz = tz - pz
    dist = math.sqrt(rx**2 + ry**2 + rz**2)

    vrel_x = tvx - pvx
    vrel_y = tvy - pvy
    vrel_z = tvz - pvz

    # --- 2. LOS unit vector ---
    if dist > hit_req_range:
        los_x = rx / dist
        los_y = ry / dist
        los_z = rz / dist
    else:
        los_x = los_y = los_z = 0.0
    cur_los = (los_x, los_y, los_z)

    # --- 2b. LOS rate vector ---
    if guidance_mode == "PPN_modified":
        vrel_x += tax * dt
        vrel_y += tay * dt
        vrel_z += taz * dt

    omega_x, omega_y, omega_z = _compute_los_rate(
        rx, ry, rz, dist, vrel_x, vrel_y, vrel_z, 
        los_rate_method, prev_los, los_x, los_y, los_z, dt
    )

    # --- 3. Closing velocity ---
    vrel_raw_x = tvx - pvx
    vrel_raw_y = tvy - pvy
    vrel_raw_z = tvz - pvz
    range_rate = (vector_math.vector_dot_product(rx, ry, rz,
                                     vrel_raw_x, vrel_raw_y, vrel_raw_z)
                  / dist) if dist > hit_req_range else 0.0
    closing_velocity = -range_rate

    # --- 3b. Logarithmic PN gain decay based on time-to-go ---
    n_eff = navigation_constant
    t_go = None
    if cfg.PN_GAIN_DECAY_ENABLED and dist > hit_req_range:
        if closing_velocity > 0:
            t_go = dist / closing_velocity
        else:
            # Flying away (post-miss): use pursuer speed as proxy
            pv_mag = math.sqrt(pvx**2 + pvy**2 + pvz**2)
            t_go = dist / max(pv_mag, 1.0)
        k = cfg.PN_GAIN_DECAY_K
        t_ref = cfg.PN_GAIN_DECAY_T_REF
        decay = min(1.0, math.log(1.0 + k * t_go) / math.log(1.0 + k * t_ref))
        n_eff = navigation_constant * decay

    if guidance_mode == "PD":
        acx, acy, acz = _compute_pd_acceleration(rx, ry, rz, vrel_raw_x, vrel_raw_y, vrel_raw_z)

        # Note: acz is used directly here, but later code expects PN to be horizontal only
        # We'll handle this in the Z-axis processing section.
    elif guidance_mode in ("TPN", "APN", "PPN", "PPN_modified"):
        acx, acy, acz = _compute_pn_acceleration(
            guidance_mode, n_eff, closing_velocity, 
            omega_x, omega_y, omega_z, los_x, los_y, los_z, 
            tax, tay, pvx, pvy, pvz
        )
    elif guidance_mode == "PURSUIT":
        # Pure pursuit generating acceleration along the LOS vector.
        # We want to accelerate towards a target pursuit speed.
        pursuit_speed = speed_max * min(dist / cfg.DECEL_RANGE, 1.0)
        pursuit_speed = max(pursuit_speed, speed_min)
        
        # Calculate speed error along LOS
        current_speed_along_los = (pvx * los_x) + (pvy * los_y) + (pvz * los_z)
        speed_err = pursuit_speed - current_speed_along_los
        
        # Proportional speed controller
        accel_mag = speed_err * cfg.PURSUER_KP
        
        acx = los_x * accel_mag
        acy = los_y * accel_mag
        acz = (los_z * accel_mag) if cfg.Z_AXIS_PN else 0.0
        
    else:
        # Fallback (should not happen)
        acx, acy, acz = 0.0, 0.0, 0.0

    # Apply sigmoid transition to smooth in the PN acceleration
    if guidance_mode in ("TPN", "APN", "PPN", "PPN_modified"):
        acx *= transition_multiplier
        acy *= transition_multiplier

    # --- 4b. Tail-chase heuristic (Vc < 0 → target moving away) ---
    acx, acy = _apply_tail_chase_heuristic(
        closing_velocity, guidance_mode, acx, acy, pvx, pvy, pvz, rx, ry, rz
    )

    # --- Turn-rate limiter (horizontal) ---
    acmag = math.sqrt(acx**2 + acy**2)
    pvmag = math.sqrt(pvx**2 + pvy**2)
    if pvmag > 1e-3:
        max_turn_accel = pvmag * math.radians(max_turn_deg)
        if acmag > max_turn_accel:
            s_turn = max_turn_accel / acmag
            acx *= s_turn; acy *= s_turn
    
    # --- PN acceleration clamp (horizontal) ---
    acmag = math.sqrt(acx**2 + acy**2)
    if acmag > cfg.MAX_PN_ACCEL:
        s_clamp = cfg.MAX_PN_ACCEL / acmag
        acx *= s_clamp; acy *= s_clamp
    
    # --- Z axis processing (already computed as acz) ---
    # We allow the Z acceleration to pass through, but we could clamp it too 
    # if it's extreme. For now, we trust the PN/Pursuit Z calculation.
    if abs(acz) > cfg.MAX_PN_ACCEL:
        acz = math.copysign(cfg.MAX_PN_ACCEL, acz)
    if telemetry_dict is not None:
        telemetry_dict["cmd_acx"] = acx
        telemetry_dict["cmd_acy"] = acy
        telemetry_dict["cmd_acz"] = acz
        telemetry_dict["omega_x"] = omega_x
        telemetry_dict["omega_y"] = omega_y
        telemetry_dict["omega_z"] = omega_z
        telemetry_dict["closing_velocity"] = closing_velocity
        telemetry_dict["n_eff"] = n_eff
        telemetry_dict["t_go"] = t_go if t_go is not None else -1.0

    # --- Speed clamping ---
    """
    vcmag = math.sqrt(vcx**2 + vcy**2 + vcz**2)
    if vcmag > hit_req_range:
        if vcmag > speed_max:
            s = speed_max / vcmag
            vcx *= s; vcy *= s; vcz *= s
        elif vcmag < speed_min:
            s = speed_min / vcmag
            vcx *= s; vcy *= s; vcz *= s
    """
    return (acx, acy, acz, cur_los)
# ============================================================
#  Guidance Loop (importable class)
# ============================================================

class GuidanceLoop:
    """
    Encapsulates the PN guidance computation.
    Connects to both pursuer and target via pymavlink, computes
    guidance commands, and returns them for an external sender.

    uses background threads to continuously read MAVLink state so
    the guidance step() is never blocked by recv_match().
    """

    # --- Configuration (loaded from guidance_config.py) ---
    LOOP_HZ          = cfg.LOOP_HZ
    NAV_CONSTANT     = cfg.NAV_CONSTANT
    MAX_TURN_DEG     = cfg.MAX_TURN_DEG
    SPEED_MIN        = cfg.SPEED_MIN
    SPEED_MAX        = cfg.SPEED_MAX
    STARTUP_KICK     = cfg.STARTUP_KICK
    COMMAND_DT       = cfg.COMMAND_DT
    MAX_ACCEL        = cfg.MAX_ACCEL
    GUIDANCE_MODE    = cfg.GUIDANCE_MODE
    ACCEL_SOURCE     = cfg.ACCEL_SOURCE
    LOS_RATE_METHOD  = cfg.LOS_RATE_METHOD
    RC_SWITCH_CHANNEL = cfg.RC_SWITCH_CHANNEL
    DECEL_RANGE      = cfg.DECEL_RANGE
    PN_ENGAGE_RANGE  = cfg.PN_ENGAGE_RANGE
    MIN_ALT_M        = cfg.MIN_ALT_M
    USE_PROPAGATED_VEL_POS = cfg.USE_PROPAGATED_VEL_POS

    def __init__(self,
                 pursuer_conn_str='udpin:localhost:14552',
                 target_conn_str='udpin:localhost:14600'):
        self.dt = 1.0 / self.LOOP_HZ
        self.prev_target_vel = None
        self._prev_los = None
        self._target_vel_stamp = 0.0               # time of last velocity change
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self._apn_start_time = None
        self.transition_multiplier = 0.0
        self._last_mode_print = ""   # track mode for change logging
        self._prev_cmd = (0.0, 0.0, 0.0)  # previous commanded velocity for rate limiting
        self._smooth_cmd = None  # EMA-smoothed velocity command (initialized on first step)

        from telemetry_logger import TelemetryLogger
        self.logger = TelemetryLogger()

        from numeric_differentiation import VelocityDifferentiator
        self.target_differentiator = VelocityDifferentiator()

        # Miss detection state — angular velocity of LOS
        self._los_angle_differentiator = VelocityDifferentiator()
        self._miss_detected = False
        self._miss_start_time = None
        self._min_range_seen = float('inf')
        self._prev_range = float('inf')

        # --- Connect to pursuer (read-only) ---
        print(f"[pronav] Connecting to PURSUER on {pursuer_conn_str} ...")
        self.pursuer_conn = mavutil.mavlink_connection(pursuer_conn_str)
        self.pursuer_conn.wait_heartbeat()
        print(f"[pronav] Pursuer heartbeat  (sys {self.pursuer_conn.target_system} "
              f"comp {self.pursuer_conn.target_component})")

        # --- Get pursuer GPS home (used as NED origin for target) ---
        self._fetch_pursuer_home()

        # --- Connect to target (read-only) ---
        print(f"[pronav] Connecting to TARGET on {target_conn_str} ...")
        self.target_conn = mavutil.mavlink_connection(target_conn_str)
        self.target_conn.wait_heartbeat()
        print(f"[pronav] Target heartbeat   (sys {self.target_conn.target_system} "
              f"comp {self.target_conn.target_component})")

        # Request ALL data streams from target (SCALED_IMU + ATTITUDE not sent by default)
        self.target_conn.mav.request_data_stream_send(
            self.target_conn.target_system,
            self.target_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1,  # 10 Hz, start
        )
        print("[pronav] Requested ALL data streams from target")

        # --- Start background readers ---
        # Pursuer reader listens for LOCAL_POSITION_NED (primary) + RC_CHANNELS
        self._pursuer_reader = mavlink_utils.MavStateReader(
            self.pursuer_conn,
            ['LOCAL_POSITION_NED', 'RC_CHANNELS'],
            mavlink_utils.parse_local_ned,
        )

        # Capture home coords for the target parser closure
        h_lat, h_lon, h_alt = self.home_lat, self.home_lon, self.home_alt
        self._target_reader = mavlink_utils.MavStateReader(
            self.target_conn,
            ['GLOBAL_POSITION_INT', 'SCALED_IMU', 'ATTITUDE'],
            lambda msg: mavlink_utils.parse_global_int(msg, h_lat, h_lon, h_alt),
        )

        self._pursuer_reader.start()
        self._target_reader.start()

        # Wait until we have at least one reading from each
        print("[pronav] Waiting for first state from both aircraft ...")
        while True:
            p, _ = self._pursuer_reader.get()
            t, _ = self._target_reader.get()
            if p is not None and t is not None:
                break
            time.sleep(0.1)

        print(f"[pronav] Guidance ready — {self.LOOP_HZ} Hz, "
              f"mode={self.GUIDANCE_MODE}, N={self.NAV_CONSTANT}")

    def _check_miss_status(self, range_m, rx, ry, rz, pursuer_vel):
        """Detect a miss by monitoring the angular velocity of the LOS vector.
        
        When the pursuer flies past the target, the LOS direction rotates
        rapidly.  We numerically differentiate the azimuth and elevation
        angles of the range vector; if the resulting angular rate exceeds
        MISS_ANGULAR_RATE_THRESHOLD the flyby is declared.
        """
        self._min_range_seen = min(self._min_range_seen, range_m)

        # --- Compute spherical angles of the range vector ---
        range_2d = math.sqrt(rx**2 + ry**2)
        azimuth   = math.atan2(ry, rx)                      # radians, horizontal plane
        elevation = math.atan2(-rz, range_2d) if range_2d > 1e-6 else 0.0  # radians, positive = up

        # Feed into the differentiator (vz slot unused, pass 0)
        now = time.monotonic()
        self._los_angle_differentiator.update(now, azimuth, elevation, 0.0)

        # Get angular rates via the configured differentiation method
        daz, del_, _ = self._los_angle_differentiator.get_acceleration(cfg.NUMERIC_DIFF_METHOD)
        angular_rate = math.sqrt(daz**2 + del_**2)  # rad/s

        # --- Trigger miss if angular rate exceeds threshold ---
        if (not self._miss_detected
                and angular_rate > cfg.MISS_ANGULAR_RATE_THRESHOLD
                and self._min_range_seen < cfg.MISS_DETECT_RANGE):
            self._miss_detected = True
            self._miss_start_time = time.monotonic()
            print(f"[pronav] MISS DETECTED — LOS angular rate {angular_rate:.2f} rad/s "
                  f"exceeds threshold {cfg.MISS_ANGULAR_RATE_THRESHOLD:.2f}, "
                  f"closest approach was {self._min_range_seen:.1f} m. Re-engaging ...")

        # --- Clear miss state once the pursuer is facing the target ---
        if self._miss_detected:
            pvx, pvy, pvz = pursuer_vel
            vel_mag = math.sqrt(pvx**2 + pvy**2 + pvz**2)
            if vel_mag > 0.5 and range_m > 0.1:
                cos_angle = (pvx*rx + pvy*ry + pvz*rz) / (vel_mag * range_m)
                cos_angle = max(-1.0, min(1.0, cos_angle))
                facing_angle_deg = math.degrees(math.acos(cos_angle))
                if facing_angle_deg < cfg.REENGAGE_FACING_DEG:
                    print(f"[pronav] Re-engaged — facing target ({facing_angle_deg:.1f}° off LOS)")
                    self._miss_detected = False
                    self._miss_start_time = None
                    self._min_range_seen = range_m

    def _compute_reengagement_command(self, range_m, rx, ry, rz):
        """Handle braking and turning back towards target after a miss.
        
        Uses a sigmoid ramp to smoothly transition the pursuit speed
        from near-zero up to SPEED_MAX over ~2 seconds, preventing
        the violent velocity discontinuity that caused crashes.
        """
        pvx, pvy, pvz = self._prev_cmd
        pv_mag = math.sqrt(pvx**2 + pvy**2 + pvz**2)
        """
        if pv_mag > cfg.SAFE_TURN_SPEED:
            # Phase 1: Brake along current heading until speed is safe for turning.
            smooth_decel = self.MAX_ACCEL * 0.5
            brake_speed = max(pv_mag - smooth_decel * self.dt, cfg.SAFE_TURN_SPEED)

            if pv_mag > 0.1:
                vx_cmd = (pvx / pv_mag) * brake_speed
                vy_cmd = (pvy / pv_mag) * brake_speed
                vz_cmd = (pvz / pv_mag) * brake_speed
            else:
                vx_cmd, vy_cmd, vz_cmd = 0.0, 0.0, 0.0

            return self._rate_limit_cmd(vx_cmd, vy_cmd, vz_cmd, range_m, reengaging=False)
        else:
        """
        # Phase 2: Turn toward target with sigmoid-ramped speed.
        # Sigmoid: 0→1 over RAMP_TIME_S, centered at RAMP_TIME_S/2.
        t_elapsed = time.monotonic() - self._miss_start_time if self._miss_start_time else 0.0
        if t_elapsed >= cfg.REENGAGE_RAMP_TIME_S:
            ramp = 1.0
        else:
            x = (t_elapsed / cfg.REENGAGE_RAMP_TIME_S) * 10.0 - 5.0
            ramp = 1.0 / (1.0 + math.exp(-x))

        pursuit_speed = self.SPEED_MIN + (self.SPEED_MAX - self.SPEED_MIN) * ramp
        #pursuit_speed = min(pursuit_speed, self.SPEED_MAX * min(range_m / max(self.DECEL_RANGE, 1.0), 1.0))
        pursuit_speed = max(pursuit_speed, self.SPEED_MIN)

        vx_cmd = (rx / range_m) * pursuit_speed
        vy_cmd = (ry / range_m) * pursuit_speed
        vz_cmd = (rz / range_m) * pursuit_speed
        return self._rate_limit_cmd(vx_cmd, vy_cmd, vz_cmd, range_m, reengaging=True)

    def _get_target_acceleration(self, target_vel):
        """Estimate or fetch the target's acceleration based on the configured source."""
        if self.GUIDANCE_MODE not in ("APN", "PPN_modified"):
            return (0.0, 0.0, 0.0)

        if self.ACCEL_SOURCE == "mavlink":
            accel = self._target_reader.get_accel_ned()
            return accel if accel is not None else (0.0, 0.0, 0.0)

        # Estimate via numerical differentiation
        if self.prev_target_vel is None or target_vel != self.prev_target_vel:
            now = time.monotonic()
            self.target_differentiator.update(now, target_vel[0], target_vel[1], target_vel[2])
            self._target_accel_est = self.target_differentiator.get_acceleration(cfg.NUMERIC_DIFF_METHOD)
            self.prev_target_vel = target_vel

        return self._target_accel_est

    def _determine_active_mode(self, rx, ry, rz, range_m, pursuer_vel, target_vel):
        """Calculate closing velocity and pursuer speed to determine guidance mode."""
        vrel_raw_x = target_vel[0] - pursuer_vel[0]
        vrel_raw_y = target_vel[1] - pursuer_vel[1]
        vrel_raw_z = target_vel[2] - pursuer_vel[2]
        
        range_rate = ((rx*vrel_raw_x + ry*vrel_raw_y + rz*vrel_raw_z) / range_m) if range_m > 1e-7 else 0.0
        closing_velocity = -range_rate
        pv_mag = math.sqrt(pursuer_vel[0]**2 + pursuer_vel[1]**2 + pursuer_vel[2]**2)

        # Mode Selection
        use_apn = closing_velocity > cfg.APN_ENGAGE_VC_MIN and pv_mag > cfg.APN_ENGAGE_SPEED_MIN
        use_pd = range_m <= cfg.PD_ENGAGE_RANGE
        active_mode = "PD" if use_pd else ("APN" if use_apn else "PURSUIT")
        return active_mode, closing_velocity

    def _fetch_pursuer_home(self):
        """Wait for HOME_POSITION from the pursuer to set the NED origin."""
        print("[pronav] Requesting pursuer HOME_POSITION ...")
        self.pursuer_conn.mav.command_long_send(
            self.pursuer_conn.target_system,
            self.pursuer_conn.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        msg = self.pursuer_conn.recv_match(type='HOME_POSITION', blocking=True,
                                           timeout=10.0)
        if msg:
            self.home_lat = msg.latitude / 1e7
            self.home_lon = msg.longitude / 1e7
            self.home_alt = msg.altitude / 1000.0
            print(f"[pronav] Pursuer home: lat={self.home_lat:.7f}  "
                  f"lon={self.home_lon:.7f}  alt={self.home_alt:.1f} m MSL")
        else:
            print("[pronav] HOME_POSITION timeout, using first GLOBAL_POSITION_INT ...")
            gp = self.pursuer_conn.recv_match(type='GLOBAL_POSITION_INT',
                                              blocking=True, timeout=5.0)
            if gp:
                self.home_lat = gp.lat / 1e7
                self.home_lon = gp.lon / 1e7
                self.home_alt = gp.alt / 1000.0
                print(f"[pronav] Using pursuer pos as home: "
                      f"lat={self.home_lat:.7f}  lon={self.home_lon:.7f}  "
                      f"alt={self.home_alt:.1f} m MSL")
            else:
                raise RuntimeError("Cannot determine pursuer home position")

    def _log_and_return(self, ax_cmd, ay_cmd, az_cmd, range_m, mode,
                        pursuer_pos, pursuer_vel, target_pos, target_vel):
        """Log telemetry and return the acceleration command tuple."""
        self.logger.log_step({
            "timestamp": time.time(),
            "mode": mode,
            "px": pursuer_pos[0], "py": pursuer_pos[1], "pz": pursuer_pos[2],
            "pvx": pursuer_vel[0], "pvy": pursuer_vel[1], "pvz": pursuer_vel[2],
            "tx": target_pos[0], "ty": target_pos[1], "tz": target_pos[2],
            "tvx": target_vel[0], "tvy": target_vel[1], "tvz": target_vel[2],
            "range": range_m,
            "cmd_acx": ax_cmd, "cmd_acy": ay_cmd, "cmd_acz": az_cmd,
            "tax": 0.0, "tay": 0.0, "taz": 0.0,
        })
        return (ax_cmd, ay_cmd, az_cmd, range_m)

    def step(self, pursuer_vel_fallback=None):
        """
        Runs one iteration of the guidance loop.
        
        Parameters
        ----------
        pursuer_vel_fallback : tuple
            If provided, uses this velocity (from Dronekit) instead of 
            the one from the raw MAVLink stream if MAVLink is unreliable.
            
        Returns
        -------
        roll_cmd, pitch_cmd, yaw_cmd, yaw_rate, thrust_req, range_m 
            (Earth-Frame attitude targets and distance)
        or None if no data is available.
        """
        #self._loop_count += 1
        
        # --- 1. Read cached states (never blocks) ---
        pursuer_pos, pursuer_vel = self._pursuer_reader.get()
        target_pos,  target_vel  = self._target_reader.get()

        if pursuer_vel_fallback is not None and pursuer_vel is not None:
             # Use dronekit velocity if provided
             pursuer_vel = pursuer_vel_fallback

        if pursuer_pos is None or target_pos is None:
            return None

        # --- 1b. Predict target state for this tick ---
        target_accel = self._get_target_acceleration(target_vel)
        if self.USE_PROPAGATED_VEL_POS:
            t_dt = self.dt
            # 1. Update target position (p = p + v*dt + 0.5*a*dt^2)
            target_pos = (
                target_pos[0] + target_vel[0]*t_dt + 0.5*target_accel[0]*t_dt**2,
                target_pos[1] + target_vel[1]*t_dt + 0.5*target_accel[1]*t_dt**2,
                target_pos[2] + target_vel[2]*t_dt + 0.5*target_accel[2]*t_dt**2
            )
            # 2. Update target velocity (v = v + a*dt)
            target_vel = (
                target_vel[0] + target_accel[0]*t_dt,
                target_vel[1] + target_accel[1]*t_dt,
                target_vel[2] + target_accel[2]*t_dt
            )

        # --- 2. Range + direction vector ---
        range_m, rx, ry, rz = vector_math.calculate_distance_vector(
            target_pos[0], target_pos[1], target_pos[2],
            pursuer_pos[0], pursuer_pos[1], pursuer_pos[2],
        )

        # --- 2b. Miss detection (disabled for APN — it self-corrects) ---
        self._check_miss_status(range_m, rx, ry, rz, pursuer_vel)
        self._prev_range = range_m

        # --- 2c. Re-engagement pursuit (overrides after a miss) ---
        # [COMMENTED OUT — velocity commands replaced by acceleration output]
        # if self._miss_detected and range_m > 1.0:
        #     result = self._compute_reengagement_command(range_m, rx, ry, rz)
        #     return self._log_and_return(result[0], result[1], result[2], range_m,
        #                                "REENGAGE", pursuer_pos, pursuer_vel, target_pos, target_vel)

        # --- 3. Startup kick (bootstrap PPN) ---
        # [COMMENTED OUT — velocity commands replaced by acceleration output]
        # pursuer_speed = math.sqrt(
        #     pursuer_vel[0]**2 + pursuer_vel[1]**2 + pursuer_vel[2]**2
        # )
        # if pursuer_speed < 0.7 * self.STARTUP_KICK and range_m > 1.0:
        #     vx_cmd = (rx / range_m) * self.STARTUP_KICK
        #     vy_cmd = (ry / range_m) * self.STARTUP_KICK
        #     vz_cmd = (rz / range_m) * self.STARTUP_KICK
        #     vx_cmd, vy_cmd, vz_cmd, range_m = self._rate_limit_cmd(vx_cmd, vy_cmd, vz_cmd, range_m)
        #     return self._log_and_return(vx_cmd, vy_cmd, vz_cmd, range_m,
        #                                "KICK", pursuer_pos, pursuer_vel, target_pos, target_vel)

        # --- 4. RC switch: select guidance mode ---
        # [COMMENTED OUT — velocity commands replaced by acceleration output]
        # rc_pwm = self._pursuer_reader.get_rc(self.RC_SWITCH_CHANNEL)
        # use_pursuit = rc_pwm > 1500
        #
        # if use_pursuit and range_m > 1.0:
        #     if "PURSUIT (RC)" != self._last_mode_print:
        #         print(f"[pronav] Mode → PURSUIT (RC Override)  (CH{self.RC_SWITCH_CHANNEL} = {rc_pwm})")
        #         self._last_mode_print = "PURSUIT (RC)"
        #     pursuit_speed = self.SPEED_MAX * min(range_m / self.DECEL_RANGE, 1.0)
        #     pursuit_speed = max(pursuit_speed, self.SPEED_MIN)
        #     vx_cmd = (rx / range_m) * pursuit_speed
        #     vy_cmd = (ry / range_m) * pursuit_speed
        #     vz_cmd = (rz / range_m) * pursuit_speed
        #     vx_cmd, vy_cmd, vz_cmd, range_m = self._rate_limit_cmd(vx_cmd, vy_cmd, vz_cmd, range_m)
        #     return self._log_and_return(vx_cmd, vy_cmd, vz_cmd, range_m,
        #                                "PURSUIT(RC)", pursuer_pos, pursuer_vel, target_pos, target_vel)

        # --- 4. Kinematic-based guidance selection ---
        active_mode, closing_velocity = self._determine_active_mode(
            rx, ry, rz, range_m, pursuer_vel, target_vel
        )
        
        if active_mode != self._last_mode_print:
            print(f"[pronav] Mode → {active_mode}")
            self._last_mode_print = active_mode

        if active_mode == "APN" or active_mode == "PD":
            # Start timer if we just transitioned into APN (or PD, though PD uses full multiplier)
            if self._apn_start_time is None:
                self._apn_start_time = time.monotonic()
            
            t_elapsed = time.monotonic() - self._apn_start_time
            if t_elapsed >= cfg.APN_TRANSITION_TIME_S:
                self.transition_multiplier = 1.0
            else:
                x = (t_elapsed / cfg.APN_TRANSITION_TIME_S) * 10.0 - 5.0
                self.transition_multiplier = 1.0 / (1.0 + math.exp(-x))
        else:
            self._apn_start_time = None
            self.transition_multiplier = 0.0

        # Logging dictionary
        telemetry_dict = {
            "timestamp": time.time(),
            "mode": active_mode,
            "px": pursuer_pos[0], "py": pursuer_pos[1], "pz": pursuer_pos[2],
            "pvx": pursuer_vel[0], "pvy": pursuer_vel[1], "pvz": pursuer_vel[2],
            "tx": target_pos[0], "ty": target_pos[1], "tz": target_pos[2],
            "tvx": target_vel[0], "tvy": target_vel[1], "tvz": target_vel[2],
            "range": range_m
        }

        # --- 5. Target acceleration ---
        telemetry_dict["tax"] = target_accel[0]
        telemetry_dict["tay"] = target_accel[1]
        telemetry_dict["taz"] = target_accel[2]

        # Determine the actual mathematical guidance mode to run
        # active_mode from _determine_active_mode is either "PD", "APN" (meaning any PN), or "PURSUIT".
        if active_mode == "APN":
            actual_guidance_mode = self.GUIDANCE_MODE
        else:
            actual_guidance_mode = active_mode

        # --- 6. Compute PN guidance command (returns acceleration) ---
        ax_cmd, ay_cmd, az_cmd, cur_los = compute_guidance_command(
            pursuer_pos, pursuer_vel,
            target_pos,  target_vel,
            self.COMMAND_DT,
            self.MAX_TURN_DEG,
            self.SPEED_MIN,
            self.SPEED_MAX,
            self.NAV_CONSTANT,
            guidance_mode=actual_guidance_mode,
            target_accel=target_accel,
            los_rate_method=self.LOS_RATE_METHOD,
            prev_los=self._prev_los,
            transition_multiplier=self.transition_multiplier,
            telemetry_dict=telemetry_dict,
        )
        self._prev_los = cur_los

        # --- 6.5 Altitude protection (linear proportional) ---
        # Intervene when below MIN_ALT_M.
        alt_m = -pursuer_pos[2]
        alt_error = self.MIN_ALT_M - alt_m  # positive means we are too low
        if alt_error > 0:
            # NED: negative Z is UP. Linearly proportional to alt_error.
            az_cmd -= cfg.KP_ALT * alt_error

        # Output the required attitude instead of just raw acceleration.
        # Initialize yaw on the first pass
        if getattr(self, '_current_yaw_cmd', None) is None:
            self._current_yaw_cmd = math.atan2(pursuer_vel[1], pursuer_vel[0]) if (pursuer_vel[0]**2 + pursuer_vel[1]**2) > 0.1 else 0.0

        # Convert acceleration to Earth-Frame attitude
        import vector_math as vecm
        roll_cmd, pitch_cmd, yaw_rate, thrust_req = vecm.accel_to_euler_ef(
            ax_cmd, ay_cmd, az_cmd, self._current_yaw_cmd, pursuer_vel[0], pursuer_vel[1]
        )
        
        # Integrate yaw rate
        self._current_yaw_cmd += yaw_rate * self.dt
        # wrap yaw
        while self._current_yaw_cmd > math.pi:  self._current_yaw_cmd -= 2*math.pi
        while self._current_yaw_cmd < -math.pi: self._current_yaw_cmd += 2*math.pi

        # --- Log commands ---
        telemetry_dict["cmd_acx"] = ax_cmd
        telemetry_dict["cmd_acy"] = ay_cmd
        telemetry_dict["cmd_acz"] = az_cmd
        telemetry_dict["cmd_roll"] = roll_cmd
        telemetry_dict["cmd_pitch"] = pitch_cmd
        telemetry_dict["cmd_yaw_rate"] = yaw_rate

        self.logger.log_step(telemetry_dict)

        return (roll_cmd, pitch_cmd, self._current_yaw_cmd, yaw_rate, thrust_req, range_m)

# ============================================================
#  Standalone test (prints commands without sending)
# ============================================================

if __name__ == "__main__":
    guidance = GuidanceLoop()
    dt = guidance.dt
    print(f"[pronav] Running standalone test at {guidance.LOOP_HZ} Hz ...")

    while True:
        t_start = time.monotonic()

        result = guidance.step()
        if result is None:
            print("[pronav] Waiting for MAVLink data ...")
            time.sleep(0.5)
            continue

        vx, vy, vz, rng = result
        vmag = math.sqrt(vx**2 + vy**2 + vz**2)
        print(f"[pronav] Vcmd=({vx:+7.2f}, {vy:+7.2f}, {vz:+7.2f}) "
              f"|V|={vmag:6.2f} m/s   Range={rng:7.1f} m")

        elapsed = time.monotonic() - t_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
