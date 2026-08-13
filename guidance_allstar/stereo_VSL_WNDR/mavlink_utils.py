import math
import threading
import time

import vector_math


def _message_stamp_seconds(msg):
    time_boot_ms = getattr(msg, "time_boot_ms", None)
    if time_boot_ms is not None:
        try:
            stamp = float(time_boot_ms) / 1000.0
            if stamp > 0.0:
                return stamp
        except (TypeError, ValueError):
            pass
    return time.monotonic()


class MavStateReader(threading.Thread):
    """
    Background thread that continuously reads MAVLink messages and
    caches the latest position + velocity. Consumers call get() to
    obtain the most recent snapshot without blocking.
    """

    def __init__(self, mavconn, msg_type, parse_fn):
        """
        Parameters
        ----------
        mavconn   : pymavlink connection
        msg_type  : str or list of str, e.g. 'LOCAL_POSITION_NED'
        parse_fn  : callable(msg) -> (pos_tuple, vel_tuple)
                    Used only for the primary (first) message type.
        """
        super().__init__(daemon=True)
        self.mavconn = mavconn
        if isinstance(msg_type, str):
            self.msg_types = [msg_type]
        else:
            self.msg_types = list(msg_type)
        self.parse_fn = parse_fn
        self.lock = threading.Lock()
        self._pos = None
        self._vel = None
        self._stamp = 0.0
        self._wall_stamp = 0.0
        self._extras = {}

    def run(self):
        while True:
            # One malformed packet or transport hiccup must not kill this
            # thread: consumers cache-read via get() and would silently fly on
            # frozen data forever (2026-07-24 review). Log, back off, retry.
            try:
                self._run_once()
            except Exception as exc:
                print(
                    f"[mav_reader] {self.msg_types[0]} reader error "
                    f"(thread kept alive): {exc}"
                )
                time.sleep(0.2)

    def _run_once(self):
        while True:
            msg = self.mavconn.recv_match(
                type=self.msg_types, blocking=True, timeout=2.0
            )
            if msg is not None:
                mtype = msg.get_type()
                if mtype == self.msg_types[0]:
                    # Primary message -> update pos/vel using the vehicle's
                    # own message time when available. Wall time remains
                    # available for stale-data checks in callers.
                    pos, vel = self.parse_fn(msg)
                    with self.lock:
                        self._pos = pos
                        self._vel = vel
                        self._stamp = _message_stamp_seconds(msg)
                        self._wall_stamp = time.monotonic()
                elif mtype == "RC_CHANNELS":
                    rc = {}
                    for ch in range(1, 19):
                        rc[ch] = getattr(msg, f"chan{ch}_raw", 0)
                    with self.lock:
                        self._extras["rc"] = rc
                elif mtype == "SCALED_IMU":
                    with self.lock:
                        if "imu" not in self._extras:
                            print(
                                f"[pronav] First SCALED_IMU received: xacc={msg.xacc} yacc={msg.yacc} zacc={msg.zacc}"
                            )
                        self._extras["imu"] = (msg.xacc, msg.yacc, msg.zacc)
                elif mtype == "ATTITUDE":
                    with self.lock:
                        if "att" not in self._extras:
                            print(
                                f"[pronav] First ATTITUDE received: roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}"
                            )
                        self._extras["att"] = (msg.roll, msg.pitch, msg.yaw)
                        # Body angular rates [rad/s]. yawspeed is the recovery
                        # machine's "is it still spinning?" signal.
                        self._extras["att_rate"] = (
                            getattr(msg, "rollspeed", 0.0),
                            getattr(msg, "pitchspeed", 0.0),
                            getattr(msg, "yawspeed", 0.0),
                        )
                        self._extras["att_wall"] = time.monotonic()

    def get(self):
        with self.lock:
            return self._pos, self._vel

    def get_with_stamp(self):
        with self.lock:
            return self._pos, self._vel, self._stamp

    def get_with_times(self):
        with self.lock:
            return self._pos, self._vel, self._stamp, self._wall_stamp

    def get_rc(self, channel=7):
        with self.lock:
            rc = self._extras.get("rc")
            if rc is None:
                return 0
            return rc.get(channel, 0)

    def get_yaw_rate(self, max_age_s=1.0):
        """Latest body yaw rate [rad/s], or None if never seen / stale.

        Returns None rather than 0.0 when the data is missing or old, so a
        caller gating on "is it spinning?" can tell "not spinning" apart from
        "no attitude telemetry" and fail open instead of holding forever.
        """
        with self.lock:
            rate = self._extras.get("att_rate")
            wall = self._extras.get("att_wall", 0.0)
        if rate is None:
            return None
        if max_age_s > 0.0 and (time.monotonic() - wall) > max_age_s:
            return None
        return float(rate[2])

    def get_accel_ned(self):
        with self.lock:
            imu = self._extras.get("imu")
            att = self._extras.get("att")
        if imu is None or att is None:
            return None
        G = 9.80665
        fx = imu[0] * G / 1000.0
        fy = imu[1] * G / 1000.0
        fz = imu[2] * G / 1000.0
        roll, pitch, yaw = att
        nx, ny, nz = vector_math.body_to_ned(roll, pitch, yaw, fx, fy, fz)
        return (nx, ny, nz + G)


class RcSwitchReader(threading.Thread):
    """
    Background thread that reads RC_CHANNELS and caches the PWM value.
    """

    def __init__(self, mavconn, channel=7):
        super().__init__(daemon=True)
        self.mavconn = mavconn
        self.channel = channel
        self.lock = threading.Lock()
        self._pwm = 0
        self._active = False

    def run(self):
        while True:
            msg = self.mavconn.recv_match(
                type="RC_CHANNELS", blocking=True, timeout=2.0
            )
            if msg is not None:
                attr = f"chan{self.channel}_raw"
                pwm = getattr(msg, attr, 0)
                with self.lock:
                    self._pwm = pwm
                    self._active = True

    def is_pursuit(self):
        with self.lock:
            return self._pwm > 1500

    def is_active(self):
        with self.lock:
            return self._active


# Parsers for the MavStateReader
def parse_local_ned(msg):
    return (msg.x, msg.y, msg.z), (msg.vx, msg.vy, msg.vz)


def parse_global_int(msg, home_lat, home_lon, home_alt, earth_radius=6378137.0):
    t_lat = msg.lat / 1e7
    t_lon = msg.lon / 1e7
    t_alt = msg.alt / 1000.0
    pos = vector_math.global_to_ned(
        t_lat, t_lon, t_alt, home_lat, home_lon, home_alt, earth_radius
    )
    vel = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
    return pos, vel
