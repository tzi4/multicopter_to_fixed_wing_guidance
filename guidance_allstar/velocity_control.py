import time
import math
import threading
from mavlinkHandler import MAVLinkHandlerDronekit as MAVLinkHandler
from pymavlink import mavutil
import guidance_config as cfg


class AttitudeController:
    """
    Continuously sends SET_ATTITUDE_TARGET commands to the pursuer drone
    via Dronekit at a fixed rate. Accepts Earth-Frame roll, pitch, yaw, 
    yaw rate, and thrust programmatically through set_command().
    """

    def __init__(self, connection_string='127.0.0.1:14551', send_rate_hz=10):
        print(f"[vel_ctrl] Connecting to vehicle on: {connection_string}")
        try:
            self.mavlink_handler = MAVLinkHandler(connection_string, _wait_ready=True)
            self.vehicle = self.mavlink_handler.master
            print(f"[vel_ctrl] Vehicle connected! Mode: {self.vehicle.mode.name}, Armed: {self.vehicle.armed}")
        except Exception as e:
            print(f"[vel_ctrl] Failed to connect to vehicle: {e}")
            raise

        self.send_rate_hz = send_rate_hz
        self.running = True
        self.boot_time = time.time()
        self.lock = threading.Lock()

        # Attitude command state
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.cmd_yaw_rate = 0.0
        self.cmd_thrust = 0.5  # Hover thrust default

        # EMA initialization
        self._first_command = True

        # Telemetry state
        self.range_m = 0.0
        self.cmd_accel_mag = 0.0

    # ----------------------------------------------------------
    #  Public API  (thread-safe)
    # ----------------------------------------------------------

    def set_command(self, roll, pitch, yaw, yaw_rate, thrust_req):
        """Update the attitude command being sent."""
        with self.lock:
            yaw_lock_enabled = bool(getattr(cfg, 'YAW_LOCK_ENABLED', False))
            # Apply Exponential Moving Average (EMA) smoothing to continuous values
            # to prevent sudden mechanical jerks (e.g. from 0 to 80 deg instantly)
            alpha = cfg.CMD_SMOOTHING_ALPHA
            if self._first_command:
                self.cmd_roll = roll
                self.cmd_pitch = pitch
                self.cmd_yaw_rate = 0.0 if yaw_lock_enabled else yaw_rate
                self.cmd_accel_mag = thrust_req
                self._first_command = False
            else:
                self.cmd_roll += alpha * (roll - self.cmd_roll)
                self.cmd_pitch += alpha * (pitch - self.cmd_pitch)
                if yaw_lock_enabled:
                    self.cmd_yaw_rate = 0.0
                else:
                    self.cmd_yaw_rate += alpha * (yaw_rate - self.cmd_yaw_rate)
                self.cmd_accel_mag += alpha * (thrust_req - self.cmd_accel_mag)

            # Clamp angles to a safe maximum to prevent flip-overs
            max_tilt = math.radians(cfg.MAX_TILT_DEG)
            self.cmd_roll = max(-max_tilt, min(max_tilt, self.cmd_roll))
            self.cmd_pitch = max(-max_tilt, min(max_tilt, self.cmd_pitch))
            
            # Yaw is not smoothed directly here because it wraps at pi/-pi
            # and is just a reference heading, not directly actuated in our mode.
            self.cmd_yaw = yaw
            
            # Simple thrust normalization 
            # Map requested acceleration directly against the configured max physical thrust
            max_thrust = getattr(cfg, 'MAX_THRUST', 15.0)
            norm_thrust = self.cmd_accel_mag / max_thrust if max_thrust > 1e-3 else 0.0
            self.cmd_thrust = max(0.0, min(1.0, norm_thrust))

    def set_range(self, range_m):
        """Update the displayed range to target (m)."""
        with self.lock:
            self.range_m = range_m

    def stop(self):
        """Signal the sender loop to stop."""
        self.running = False

    # ----------------------------------------------------------
    #  MAVLink sender
    # ----------------------------------------------------------
    def get_velocity_own(self):
        """Get current local NED velocity from Dronekit state."""
        vel = self.vehicle.velocity
        if vel is not None and len(vel) == 3:
            return float(vel[0]), float(vel[1]), float(vel[2])
        return 0.0, 0.0, 0.0

    def _euler_to_quaternion(self, r, p, y):
        """Convert Euler angles (roll, pitch, yaw) to a quaternion [w, x, y, z]."""
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        q = [
            cr*cp*cy + sr*sp*sy,  # w
            sr*cp*cy - cr*sp*sy,  # x
            cr*sp*cy + sr*cp*sy,  # y
            cr*cp*sy - sr*sp*cy   # z
        ]
        return q

    def _send_attitude_target(self, roll, pitch, yaw, yaw_rate, thrust):
        """Low-level: encode and send a SET_ATTITUDE_TARGET."""
        q = self._euler_to_quaternion(roll, pitch, yaw)

        # Typemask:
        # ArduCopter rejects messages that partially ignore body rates.
        # We must either ignore all rates (0b00000111) or un-ignore all rates (0b00000000).
        # In yaw-lock mode the quaternion yaw is the authority, so body rates are ignored.
        if bool(getattr(cfg, 'YAW_LOCK_ENABLED', False)):
            type_mask = (
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
                | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
                | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            )
            yaw_rate = 0.0
        else:
            # We provide yaw_rate as feedforward, and set roll/pitch rate to 0.
            type_mask = 0b00000000

        # Use DroneKit's message factory
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            int(1e3 * (time.time() - self.boot_time)), # time_boot_ms
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            type_mask,
            q, # attitude quaternion
            0, # body roll rate
            0, # body pitch rate
            yaw_rate, # body yaw rate
            thrust # thrust
        )
        self.vehicle.send_mavlink(msg)

    # ----------------------------------------------------------
    #  Sender loop  (runs in a background thread)
    # ----------------------------------------------------------

    def sender_loop(self):
        """
        Background loop that sends the latest attitude command at
        send_rate_hz.
        """
        period = 1.0 / self.send_rate_hz

        while self.running:
            # Mode enforcement: SET_ATTITUDE_TARGET requires GUIDED mode
            if self.vehicle.mode.name not in ["GUIDED", "GUIDED_NOGPS"]:
                print(f"[att_ctrl] Switching from {self.vehicle.mode.name} to GUIDED...")
                from dronekit import VehicleMode
                self.vehicle.mode = VehicleMode("GUIDED")

            with self.lock:
                roll = self.cmd_roll
                pitch = self.cmd_pitch
                yaw = self.cmd_yaw
                yaw_rate = self.cmd_yaw_rate
                thrust = self.cmd_thrust

            self._send_attitude_target(roll, pitch, yaw, yaw_rate, thrust)

            time.sleep(period)

        # Zero-out attitude on exit (level hover)
        self._send_attitude_target(0.0, 0.0, self.cmd_yaw, 0.0, 0.5)
        print("[att_ctrl] Sender loop stopped. Leveled out.")

    def start_sender_thread(self):
        """Convenience: start sender_loop in a daemon thread and return it."""
        t = threading.Thread(target=self.sender_loop, daemon=True)
        t.start()
        return t
