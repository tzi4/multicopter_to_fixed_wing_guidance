import math
import threading
import time

import vector_math

# HEARTBEAT'in custom_mode sayisini insan-okunur mod adina cevirmek icin
# (ArduPilot Copter: 4 = GUIDED, 5 = LOITER, ...). pymavlink yoksa (saf
# birim testi) sessizce ham sayiya duseriz -- bu modul MAVLink'siz de
# import edilebilmeli.
try:
    from pymavlink import mavutil as _mavutil
except Exception:                                    # pragma: no cover
    _mavutil = None


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
                elif mtype == "VIBRATION":
                    with self.lock:
                        if "vibe" not in self._extras:
                            print(
                                f"[mav_reader] First VIBRATION received: "
                                f"x={msg.vibration_x:.1f} y={msg.vibration_y:.1f} "
                                f"z={msg.vibration_z:.1f}"
                            )
                        self._extras["vibe"] = (
                            float(msg.vibration_x),
                            float(msg.vibration_y),
                            float(msg.vibration_z),
                        )
                        self._extras["vibe_wall"] = time.monotonic()
                elif mtype == "HEARTBEAT":
                    # OTOPILOT MOD + BAGLANTI SAGLIGI (2026-08-07, gercek
                    # ucus loglamasi). Iki soruyu birden cevaplar:
                    #  (a) "kaza aninda arac hangi moddaydi?" -- GUIDED'dan
                    #      dusmus (LAND/RTL/STABILIZE) bir arac bizim
                    #      setpointlerimizi ZATEN dinlemiyordur; hicbir
                    #      gudum kolonu bunu gostermez.
                    #  (b) "telemetri hala akiyor mu?" -- heartbeat yasi
                    #      buyuyorsa okudugumuz her sey DONMUS veridir
                    #      (get() cache'ten dondugu icin sessizce eski
                    #      degerle ucar; 2026-07-24 review'un korkusu).
                    # Gimbal/yer istasyonu heartbeat'leri sayilmasin diye
                    # yalniz otopilotun kendi bileseni (AUTOPILOT1) kabul
                    # edilir.
                    if getattr(msg, "autopilot", 0) == 0:      # INVALID = GCS
                        continue
                    try:
                        ad = (_mavutil.mode_string_v10(msg)
                              if _mavutil is not None else '')
                    except Exception:
                        ad = ''
                    with self.lock:
                        if "hb" not in self._extras:
                            print(f"[mav_reader] First HEARTBEAT received: "
                                  f"mod={ad or msg.custom_mode}")
                        self._extras["hb"] = (ad, int(msg.custom_mode),
                                              int(msg.base_mode))
                        self._extras["hb_wall"] = time.monotonic()

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

    def get_attitude(self, max_age_s=1.0):
        """Latest (roll, pitch, yaw) [rad], or None if never seen / stale.

        Used to measure the crab angle (heading vs course) and total tilt at
        CT-activation, to confirm the saturation/yaw-shed mechanism.
        """
        with self.lock:
            att = self._extras.get("att")
            wall = self._extras.get("att_wall", 0.0)
        if att is None:
            return None
        if max_age_s > 0.0 and (time.monotonic() - wall) > max_age_s:
            return None
        return (float(att[0]), float(att[1]), float(att[2]))

    def get_vibration(self, max_age_s=2.0):
        """Latest (vibe_x, vibe_y, vibe_z) [m/s^2], or None if never seen/stale.

        ArduPilot VIBRATION levels: <30 nominal, 30-60 marginal, >60 bad. A
        physical impact (target strike or ground crash) spikes these hard, which
        the impact detector uses to score a hit/crash. Returns None (not zeros)
        when absent so the detector can fall back to kinematics.
        """
        with self.lock:
            vibe = self._extras.get("vibe")
            wall = self._extras.get("vibe_wall", 0.0)
        if vibe is None:
            return None
        if max_age_s > 0.0 and (time.monotonic() - wall) > max_age_s:
            return None
        return (float(vibe[0]), float(vibe[1]), float(vibe[2]))

    def get_heartbeat(self):
        """Son HEARTBEAT: (mod_adi, custom_mode, yas_s) ya da None.

        DIGERLERINDEN FARKI: BAYATLIK KONTROLU YOK, cunku aranan sey
        zaten bayatligin KENDISIDIR. yas_s buyuyorsa otopilotla baglanti
        kopmustur ve o an okunan tum pos/vel/att degerleri donmustur.
        Yalniz LOG icin; kontrol yolu bu metodu cagirmaz.
        """
        with self.lock:
            hb = self._extras.get("hb")
            wall = self._extras.get("hb_wall", 0.0)
        if hb is None:
            return None
        return (hb[0], hb[1], time.monotonic() - wall)

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


def parse_global_int(
    msg, home_lat, home_lon, home_alt, earth_radius=6378137.0,
    use_relative_alt=True,
):
    """GLOBAL_POSITION_INT -> (pos_ned, vel_ned) in the pursuer's local frame.

    use_relative_alt=True (default) takes `relative_alt`, the target's height
    above ITS OWN home, and references it to zero. The pursuer's
    LOCAL_POSITION_NED z is height above ITS home, so with both aircraft
    launched from the same field the two are directly comparable -- and immune
    to the two vehicles disagreeing about that field's AMSL elevation.

    That disagreement is not hypothetical: on 2026-07-31 the two aircraft
    differed by 36.80 m on the same field (115.02 vs 78.22 m AMSL, each from its
    own GPS fix). Using `alt` (AMSL) minus the pursuer's home fed that straight
    into the vertical command -- the drone flew 36.8 m below the target while
    every diagnostic looked clean, because the estimator was tracking the
    reported position perfectly and both vehicles displayed the SAME AMSL.

    use_relative_alt=False restores the AMSL path, which is correct only when
    the two vehicles launch from different elevations AND their home altitudes
    are trustworthy.
    """
    t_lat = msg.lat / 1e7
    t_lon = msg.lon / 1e7
    if use_relative_alt:
        t_alt = msg.relative_alt / 1000.0
        ref_alt = 0.0
    else:
        t_alt = msg.alt / 1000.0
        ref_alt = home_alt
    pos = vector_math.global_to_ned(
        t_lat, t_lon, t_alt, home_lat, home_lon, ref_alt, earth_radius
    )
    vel = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
    return pos, vel
