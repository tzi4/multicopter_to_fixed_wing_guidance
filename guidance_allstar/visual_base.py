#!/usr/bin/env python3
"""
visual_base.py - IMAGE guidance common framework (based on LOS / PID / MPC)
============================================================= Position guidance
(simple_guided_follow.py) frames the target; The bbox_to_redis.py decision maker 'displays' the key
Redis 'command_authority' when the target ~1.5 s remains in the frame. From then on, the VEHICLE is in
the hands of the code built on THIS SKELETON: the situated process stops sending the setpoint (the
authority gate is there) but does NOT continue to die -- it maintains steering if the decision maker
returns to 'located'.

CONTRACT (all three methods obey): * Error signal comes from VIRTUAL GIMBAL: bbox_to_redis.py
achieve [sx, sy, w, h, ex_deg, ey_deg, t_capture] to channel 'tracker_bbox_stab'. ex/ey is the
angular error free of body oscillation (yildizlar_gimbal.angle_error_value). Reading raw pixels is not
PROHIBITED, but it is unnecessary; The gimbal layer is ready below. * RANGE ONLY FROM ESTIMATOR: we
rely on 3D location telemetry of the target az; The ONLY quantity allowed to be derived from it is
range (length of vector LOS). Class RangeEstimator sets filterwndr IMM and outputs ONLY range()
OUT. Target speed, direction, acceleration, etc. NOT DERIVED, NOT USED (user rule, 2026-08-03). Lost
assist may come LATER; not yet. * Command is SPEED: SET_POSITION_TARGET_LOCAL_NED, vx,vy,vz only
(+optional yaw_rate; FOV drives controller yaw with it). Attitude is NOT COMMANDED. * THE TRANSITION
IS SOFT: in each positioned loop, it writes its last command to the 'handoff_state' key. Thanks to
the controller's seed() call at the time of handoff and the rapid seeding of the command LPF, the
vehicle does not see any jumps.
    ("integral tohumlama").
  * DT MEASUREMENT: loop dt is measured from wall clock, nominally unreliable (loop falling to 2 Hz
  + fixed dt assumption was giant circle/flicker smell).

USAGE (method code example -- los_guidance.py / pid_guidance.py / mpc_guidance.py):

    from visual_base import (VisualController, VisualLoop, Command)

    class LosController(VisualController):
        ad = "los"
        def seed(self, handoff): # handoff: dict|None ('handoff_state') self.v_int =
        np.array(handoff["cmd_vel_ned"]) if handoff else np.zeros(3) def command(self, o): # o:
        Measurement ... # o.ex_deg, o.ey_deg, o.range_m_value, ... return Command(vel_ned=v,
        yaw_rate_dps=r)

    VisualLoop ( LosController ()).run()

OBJECTIVE FUNCTION (logged for experimental comparison): keep the target in the center (|ex|,|ey|
small) AND let sqrt(bbox area) grow (approach to collision). Log columns become input to the
trial_summary/comparison tools.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import redis
from pymavlink import mavutil

import guidance_config as cfg
import mavlink_utils
from filterwndr import clamp_filter_dt, predict_imm_over_dt, setup_imm_filter
from numeric_differentiation import VelocityDifferentiator

# SET_POSITION_TARGET_LOCAL_NED type masks: ignore position+acceleration+yaw, use velocity (yaw_rate
# optional).
_MASK_SPEED = (1 + 2 + 4) + (64 + 128 + 256) + 1024 + 2048          # only vx,vy,vz
_MASK_SPEED_YAWRATE = (1 + 2 + 4) + (64 + 128 + 256) + 1024         # + yaw_rate

# DEAD-MAN SWITCH: display controller writes 'visual_alive' every cycle, with TTL. If the process
# dies/hangs, the key TTL will eventually drop and bbox_to_redis will NOT delegate (or roll back)
# authorization to the display. TTL was selected significantly larger than the slowest acceptable
# cycle (2 Hz): 20 In normal running of Hz the key is constantly refreshed, in real death 2 drops in
# s.
ALIVE_TTL_S = 2

# --- FRAMING GEOMETRY (for LOG only; uses ex/ey of guidance virtual gimbal). Raspberry Pi AI Camera
# (IMX500) 1280x720, hfov 66 deg -> fx = (1280/2)/tan(33 deg) = 985.5. Same as
# bbox_to_redis.YildizlarGimbal.
_FRAMING_W = float(getattr(cfg, "CAMERA_WIDTH_PX", 1280))
_FRAMING_H = float(getattr(cfg, "CAMERA_HEIGHT_PX", 720))
_FRAMING_HFOV_RAD = float(getattr(cfg, "CAMERA_HFOV_RAD", 1.1519))
_FRAMING_FX = (_FRAMING_W / 2.0) / math.tan(_FRAMING_HFOV_RAD / 2.0)

# Encounter type thresholds [deg]. angle definition in _encounter_geometry.
_TYPE_HEAD_TOWARD_HEAD_DEG = 60.0
_TYPE_TAIL_DEG = 120.0

# --- YAW SUSTAIN IN SHORT GAP (env: YILDIZ_HOLD_YAW, DEFAULT ON since 2026-08-10 -- to off
# YILDIZ_HOLD_YAW=0) ----------
#
# TODAY'S BEHAVIOR AND ITS MEASURED COST. On the following 'hold' lever (bbox is briefly stale, the
# last valid speed command is held) yaw_rate is CONSCIOUSLY releasing None; The reason was "blind
# turning will cause the target to lose horizontally as well". Ellipse measured at target: NO yaw
# command goes to the vehicle in 38-42% of the target's RETURN frames. The result is a POSITIVE
# FEEDBACK: target moves out of frame in the direction of the bend -> detection drops -> more 'hold'
# frames -> more yaw frames. This is harmless on the straight leg (the target remains in the frame),
# on the return it ends the engagement.
#
# CORRECTION AND MEETING HISTORICAL CONCERN. "Freezing" (holding constant) the yaw is actually
# dangerous: if bbox never comes on again, the vehicle will continue to rotate at constant speed. So
# the sustain is (a) USSELF LIMITED -- scaled down by exp(-dt/tau) at each frame, (b) DURATION LIMITED
# -- dropped completely after a maximum of HOLD_YAW_MAXIMUM_S (None). So in the worst case the total
# additional yaw angle to the vehicle is |yaw0| * tau * (1 - exp(-max/tau)) ~ 0.78*|yaw0| is about
# degree-second; 30 ~23 deg for a dps command. Not a blind RETURN, but a blind EXPRESSION. The 'coast'
# (long loss) arm CANNOT be touched.
HOLD_YAW_TAU_S = 1.0        # damping time constant
HOLD_YAW_MAXIMUM_S = 1.5      # After this period yaw is completely dropped


def _environment_flag(key_value: str, default_value2: float = 0.0) -> float:
    """Read numbers from the environment; If it's broken/not there, go back to default."""
    try:
        value_value = os.environ.get(key_value)
        return default_value2 if value_value is None else float(value_value)
    except (TypeError, ValueError):
        return default_value2


# ---------------------------------------------------------------------- data

@dataclass
class Measurement:
    """The full suite of measurements delivered to the controller each cycle."""
    t: float                    # time.monotonic()
    dt: float                   # MEASURED cycle step [s]
    # --- virtual gimbal ( tracker_bbox_stab ) ---
    ex_deg: float | None        # horizontal angular error (+: target is on the right)
    ey_deg: float | None        # vertical angular error (+: target below)
    bbox_w: float | None        # raw bbox width [ px ]
    bbox_h: float | None
    area_root: float | None      # sqrt (w*h) -- the size we are trying to increase
    coverage_pct: float | None   # horizontal coverage [%]
    bbox_age_s: float           # time since last valid detection
    # --- estimator (range ONLY) ---
    range_m_value: float | None
    # --- our own situation (LOCAL_POSITION_NED / ATTITUDE) ---
    pos_ned: np.ndarray | None
    vel_ned: np.ndarray | None
    yaw_rad: float | None
    roll_rad: float | None
    pitch_rad: float | None
    # --- optional (by default; do not break old constructors) ---
    t_capture: float | None = None
    # CAPTURE moment of the frame (ROS time, tracker_bbox_stab[6]). Don't confuse it with another watch;
    # only the DIFFERENCE of successive measurements is significant (for delay-sensitive derivatives such
    # as lambda-point phase matching).
    px_virtual_x: float | None = None     # virtual (stabilized) pixel [ px ]
    px_virtual_y: float | None = None
    px_raw_cx: float | None = None      # RAW bbox center [ px ] (framing 1280x720)
    px_raw_cy: float | None = None
    acc_ned: np.ndarray | None = None   # OWN acceleration [m/s^2, NED] (derivative+LPF)
    tilt_deg: float | None = None
    # PHASE O (gimbal branch): the camera's elevation of the WORLD in that frame (tracker_bbox_stab[7],
    # broadcasts bbox_to_redis). Since the tilt is now dynamic, the homing should set ey_ref FROM THIS
    # instead of the static YILDIZ_TILT (mpc_guidance._framing_constant does so; None drops to static). Note:
    # px_* and acc_ned added for LOG; controllers DO NOT have to use them (contract still ex/ey + range).
    # Raw pixels are not prohibited. --- ROTATE RECTANGULAR (Emits when bbox_to_redis YILDIZ_MINRECT=1)
    # --- tracker_bbox_stab[8..10]. When the flag is off, the fields remain NONE -> all None (same
    # optional-field pattern as tilt_deg). rot_w_px : LONG side of the turned rectangle [px] (>= rot_h_px)
    # rot_h_px : SHORT side [px] rot_angle_deg : angle of the long axis relative to the screen horizontal,
    # COUNTER-clockwise positive, [-90,+90) AXIS angle (no direction information, mode 180). The target
    # banki is derived from this and our OWN roll -- so unlike the AABB aspect, the camera roll is
    # decoupled at the source and the pointing (left/right rotation) is preserved. The derivation is left
    # to the consumer; here only MOVABLE (Measurement contract: measured, uninterpreted quantity).
    rot_w_px: float | None = None
    rot_h_px: float | None = None
    rot_angle_deg: float | None = None
    vibe_max: float | None = None       # The biggest axis of our OWN VIBRATION
    # 2026-08-05: the skeleton was ALREADY reading and logging this number (column vibe_max) but was NOT
    # giving it to the controller. Opened to the controller because that's the only internal indication of
    # PHYSICAL CONTACT (type-3 sim: 2.32 vibe at m 2.0 -> 17.4, 1.04 at m 25.5; tour we don't touch-2 at
    # pass 0.85 only 3.3 in m). NOT A RULE VIOLATION: this is OUR telemetry, not the target's -- still
    # only range is used from the target.


@dataclass
class Command:
    """The command returned by the controller. vel_ned is in frame NED [m/s]."""
    vel_ned: np.ndarray
    yaw_rate_dps: float | None = None    # None = yaw remains on autopilot
    release_value: bool = False                  # True = release authority (MISS)
    release_reason: str = ''                # human-readable reason if drop=True
    event_value: str = ''                       # DISCRETED event declared by the controller
    event_detail: str = ''
    # 2026-08-05: so that the controller can feed the event log (_event.csv). First consumer IMPACT_SUCCESSFUL
    # (see mpc_guidance._hit_successful_check): so that the hit count could be read DIRECTLY from the run
    # summary -- it used to be subtracted from the CPA prediction, and the sample count remained at 5
    # since each successful hit ended the run (the car rolled over and crashed). Empty string = no event;
    # skeleton only writes FULL ones.


class VisualController:
    """The base class of method classes."""
    label_item = "base"

    def seed_value2(self, handoff):
        """Invoked once at handoff instant; handoff dict ' handoff_state ' or None (if positioned could not
write at all). Integrators are seeded here."""

    def command_value(self, measurement: Measurement) -> Command:
        raise NotImplementedError


def body_forward_ned(yaw_rad, forward, right_value, down_value):
    """Converts body-plane (forward, right, down) speed to NED (yaw only)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([c * forward - s * right_value, s * forward + c * right_value, down_value])


# ------------------------------------------------- LOG-ONLY analysis assistant
#
# WARNING -- CONTROL PATH DOES NOT CALL THIS SECTION. User rule (2026-08-03): GUIDANCE from target's
# 3D telemetry can only use RANGE. The following function uses the position/velocity/acceleration of
# the target; Its output is written ONLY to CSV, not given to any controller. The 'ref_' prefix in
# column names is a visible sign of this distinction: if a column begins with 'ref_' that number is
# NOT GUIDED, it is for analysis only. If you see a controller reading ref_* it is a RULES VIOLATION.


def _encounter_geometry(pos, vel, target_pos_value, target_vel_value):
    """Encounter geometry from hunter/target situation (FOR LOG/ANALYSIS).

    Signal conventions: bearing_deg : compass bearing from us to the target, 0 =north, +east, [ 0 ,
    360 ) elevation_deg : angle of elevation of the target relative to us, + = target ABOVE
    approximation_deg : with the DIRECTION OF THE target The angle between the 'target to us' vector.  0
    deg = target coming straight at us (HEAD TO HEAD), 180 deg = target moving away from us, we are
    behind it
                       (TAIL TAKIBI). 90 deg civari = CROSSING.
      closure_mps : range closing speed, + = distance SHORTER tgo_s : range / closing (only when
      closing) cpa_m / cpa_s : closest passing distance and time remaining assuming constant speed.
      If cpa_s < 0, the closest pass is in the PAST (clamped to 0, cpa_m is synchronized to the
      current range). Missing data fields return None; dictionary keys are always the same.
    """
    empty_value = {'range_m_value': None, 'bearing_deg': None, 'elevation_deg': None,
           'approximation_deg': None, 'type_value': '', 'closure_mps': None,
           'tgo_s': None, 'cpa_m': None, 'cpa_s': None}
    if pos is None or target_pos_value is None:
        return empty_value
    r = np.asarray(target_pos_value, float).reshape(3) - np.asarray(pos, float).reshape(3)
    range_value = float(np.linalg.norm(r))
    g = dict(empty_value)
    g['range_m_value'] = range_value
    horizontal = math.hypot(float(r[0]), float(r[1]))
    g['bearing_deg'] = math.degrees(math.atan2(float(r[1]), float(r[0]))) % 360.0
    g['elevation_deg'] = math.degrees(math.atan2(-float(r[2]), max(horizontal, 1e-6)))
    if vel is None or target_vel_value is None or range_value < 1e-6:
        return g

    v_pursuer = np.asarray(vel, float).reshape(3)
    v_target = np.asarray(target_vel_value, float).reshape(3)
    u_los = r / range_value
    v_relative = v_target - v_pursuer                      # speed of the target relative to us
    g['closure_mps'] = -float(np.dot(v_relative, u_los))
    if g['closure_mps'] > 0.1:
        g['tgo_s'] = range_value / g['closure_mps']

    # Closest pass (CPA): linear extrapolation with constant relative velocity.
    n2 = float(np.dot(v_relative, v_relative))
    if n2 > 1e-6:
        t_cpa = -float(np.dot(r, v_relative)) / n2
        if t_cpa < 0.0:                              # the transition is behind
            g['cpa_s'], g['cpa_m'] = 0.0, range_value
        else:
            g['cpa_s'] = t_cpa
            g['cpa_m'] = float(np.linalg.norm(r + v_relative * t_cpa))

    # Encounter type: angle between the target's direction of travel and the 'target to us' direction.
    h_speed = float(np.linalg.norm(v_target))
    if h_speed < 1.0:
        g['type_value'] = 'stationary'                          # the target remains in practice
        return g
    cosine = float(np.dot(v_target / h_speed, -u_los))
    g['approximation_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    if g['approximation_deg'] < _TYPE_HEAD_TOWARD_HEAD_DEG:
        g['type_value'] = 'head_toward_head'
    elif g['approximation_deg'] > _TYPE_TAIL_DEG:
        g['type_value'] = 'tail'
    else:
        g['type_value'] = 'crossing'
    return g


# ---------------------------------------------------------------------------------- reader

class BboxReader(threading.Thread):
    """Redis 'tracker_bbox_stab' + 'tracker_bbox' aboneligi (ayri thread).

    The stab channel comes when the gimbal is on; The raw channel comes with every detection. The
    coverage is taken from the raw channel (not in stab)."""

    def __init__(self, r):
        super().__init__(daemon=True)
        self.pubsub = r.pubsub(ignore_subscribe_messages=True)
        self.pubsub.subscribe('tracker_bbox_stab', 'tracker_bbox')
        self.lock = threading.Lock()
        self._stab = None           # (sx, sy, w, h, ex, ey, t_capture)
        self._stab_wall = 0.0
        self._coverage = None
        self._coverage_wall = 0.0
        # RAW channel (tracker_bbox): [x, y, w, h, coverage_%, valid, t_capture] x,y is the UPPER LEFT corner.
        # Center = x + w/2, y + h/2. Only saved for LOG: the raw answer to the question "where in the frame
        # was the target" BEFORE the virtual gimbal corrected it.
        self._raw = None
        self._raw_wall = 0.0

    def run(self):
        for message_value in self.pubsub.listen():
            try:
                data_value = json.loads(message_value['data'])
                channel_value = (message_value['channel'].decode()
                         if isinstance(message_value['channel'], bytes)
                         else message_value['channel'])
                now_value = time.monotonic()
                with self.lock:
                    if channel_value == 'tracker_bbox_stab':
                        self._stab = data_value
                        self._stab_wall = now_value
                    else:
                        # [x, y, w, h, coverage_%, valid, t_capture]
                        if len(data_value) >= 6 and data_value[5]:
                            self._coverage = float(data_value[4])
                            self._coverage_wall = now_value
                            self._raw = data_value
                            self._raw_wall = now_value
            except Exception:
                continue

    def last_value(self):
        """Returns (stab_listesi|None, bbox_age_s, coverage|None)."""
        with self.lock:
            stab, wall = self._stab, self._stab_wall
            coverage_value = self._coverage
        age_value = (time.monotonic() - wall) if stab is not None else float('inf')
        return stab, age_value, coverage_value

    def raw_value(self):
        """(raw_listesi|None, raw_age_s) -- Raw bbox channel for LOG ONLY."""
        with self.lock:
            data_value, wall = self._raw, self._raw_wall
        age_value = (time.monotonic() - wall) if data_value is not None else float('inf')
        return data_value, age_value


class RangeEstimator(threading.Thread):
    """Target telemetry -> filterwndr IMM -> range ONLY.

    User rule (2026-08-03): only RANGE is derived from the target's 3D position data. The only
    drop-out metric of this class is range(); The imm object cannot be touched from the outside. The
    castor/freeze machine (in the simple_guided_follow) is NOT included here -- location tracking is
    sufficient for range, that machine thins out the speed/spin estimation.
    """

    def __init__(self, target_conn_str, home_lat, home_lon, home_alt, hz=10.0):
        super().__init__(daemon=True)
        self.hz = hz
        self._imm = setup_imm_filter(1.0 / hz)
        self._initial = True
        self._last_stamp = None
        self._last_update_wall = 0.0
        self.lock = threading.Lock()
        conn = mavutil.mavlink_connection(target_conn_str, source_system=252)
        conn.wait_heartbeat(timeout=30)
        # Here's the target flow (same message, same rate as positioned).
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            int(1e6 / hz), 0, 0, 0, 0, 0)
        use_rel = bool(getattr(cfg, "TARGET_ALT_USE_RELATIVE", True))
        self._reader = mavlink_utils.MavStateReader(
            conn, "GLOBAL_POSITION_INT",
            lambda msg: mavlink_utils.parse_global_int(
                msg, home_lat, home_lon, home_alt, use_relative_alt=use_rel))
        self._reader.start()

    def run(self):
        period_value = 1.0 / self.hz
        while True:
            time.sleep(period_value)
            pos, _, stamp = self._reader.get_with_stamp()
            if pos is None or stamp <= 0.0:
                continue
            if self._last_stamp is not None and stamp <= self._last_stamp:
                continue
            z = np.asarray(pos, dtype=float).reshape(3)
            with self.lock:
                if self._initial:
                    for filt in self._imm.filters:
                        filt.x[0:3] = z
                    self._imm.x = self._imm.filters[0].x.copy()
                    self._initial = False
                else:
                    dt = clamp_filter_dt(stamp - self._last_stamp)
                    predict_imm_over_dt(self._imm, dt)
                    self._imm.update(z)
                self._last_stamp = stamp
                self._last_update_wall = time.monotonic()

    def range_value(self, own_pos_ned, stale_s=2.0):
        """|target_estimation - own location| [m]; None if data is missing/stale."""
        if own_pos_ned is None:
            return None
        with self.lock:
            if self._initial:
                return None
            if time.monotonic() - self._last_update_wall > stale_s:
                return None
            est = np.asarray(self._imm.x[0:3], dtype=float).reshape(3)
        difference = est - np.asarray(own_pos_ned, dtype=float).reshape(3)
        return float(np.linalg.norm(difference))

    # ------------------------------------------------------------------ LOG-ONLY. THE CONTROL PATH DOES
    # NOT CALL THIS METHOD.
    #
    # User rule (2026-08-03): guidance can only use RANGE from target's telemetry -> range() method. But
    # ANALYSIS cannot be done without knowing the target's position, speed and acceleration: In 2026-08-04
    # the user said "the target was coming towards me, MPC seems to be running away" and to verify this
    # the angle between the target's direction of travel and the bearing was calculated MANUAL -- because
    # it was not in any logs. This method closes that gap; its output goes only to CSV columns prefixed
    # with 'ref_'. ------------------------------------------------------------------
    def ref_target_state(self, stale_s=2.0):
        """target state FOR LOG; It is NOT given to the controller.

        Returns: {'pos': ndarray|None (measured NED), 'vel': ndarray|None (measured NED [m/s]), 'acc':
        ndarray|None (IMM kestirimi [m/s^2]), 'turn_dps': float|None (IMM turn hizi), 'est_pos':
        ndarray|None (IMM position_value2 kestirimi)} If no/stale data all None.
        """
        empty_value = {'pos': None, 'vel': None, 'acc': None,
               'turn_dps': None, 'est_pos': None}
        try:
            measurement_pos, measurement_vel = self._reader.get()
        except Exception:
            return empty_value
        with self.lock:
            ready = not self._initial
            stale_value = (time.monotonic() - self._last_update_wall) > stale_s
            x = np.asarray(self._imm.x, dtype=float).copy() if ready else None
        if measurement_pos is None or not ready or stale_value:
            return empty_value
        return {
            'pos': np.asarray(measurement_pos, dtype=float).reshape(3),
            'vel': (np.asarray(measurement_vel, dtype=float).reshape(3)
                    if measurement_vel is not None else None),
            # IMM status 10D: [x,y,z, vx,vy,vz, ax,ay,az, omega]
            'acc': x[6:9].copy(),
            'turn_dps': math.degrees(float(x[9])),
            'est_pos': x[0:3].copy(),
        }


# ------------------------------------------------------------------ Commander

class SpeedCommander:
    """Hunter connection: status reader + speed setpoint sender."""

    def __init__(self, pursuer_conn_str):
        self.conn = mavutil.mavlink_connection(pursuer_conn_str,
                                               source_system=251)
        self.conn.wait_heartbeat(timeout=30)
        self.boot = time.monotonic()
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        # HEARTBEAT (2026-08-07): LOG ONLY. The autopilot's flight mode and link health (heartbeat age) were
        # not available on any CSV; In actual flight, the "vehicle crashed from GUIDED" or "telemetry
        # interrupted" scenarios were indistinguishable from post-crash logs. Flow is not desired -- already
        # achieve autopilot heartbeat 1 Hz.
        self.reader = mavlink_utils.MavStateReader(
            self.conn,
            ["LOCAL_POSITION_NED", "ATTITUDE", "VIBRATION", "HEARTBEAT"],
            mavlink_utils.parse_local_ned)
        self.reader.start()

    def speed_send(self, vel_ned, yaw_rate_rad=None):
        mask = _MASK_SPEED if yaw_rate_rad is None else _MASK_SPEED_YAWRATE
        self.conn.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self.boot) * 1000.0) & 0xFFFFFFFF,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            0.0, 0.0, 0.0,
            float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2]),
            0.0, 0.0, 0.0,
            0.0, float(yaw_rate_rad or 0.0))


# ---------------------------------------------------------------------- event

class EventTracker:
    """Generate discrete events from continuous measurements to record what happened.

    WHY: 2644 is a line CSV tells "what numbers were" but not "what happened". The moment of
    handoff, loss of bbox, activation of constraints, range thresholds, closest pass and miss were
    distributed in separate text logs. Here it's all in ONE place, timestamped and machine readable.

    Usage: watch(...) is called in each loop, returning a list of events occurring in that loop
    (mostly empty). It is edge triggered: if a constraint 100 loop remains open, it generates an
    'on' event once, a 'closed' event once.
    """

    RANGE_THRESHOLDS = (100.0, 50.0, 30.0, 20.0, 10.0, 5.0, 3.0)

    def __init__(self):
        self._state = None            # previous detection status
        self._flag = {}             # constraint name -> is it on?
        self._crossed_threshold = set()
        self._en_near = float('inf')
        self._was_closing = None
        self._miss_notified = False

    def _edge(self, events_value, label_item, enabled_value, detail=''):
        if bool(enabled_value) != bool(self._flag.get(label_item, False)):
            self._flag[label_item] = bool(enabled_value)
            events_value.append((f"{label_item}_{'enabled_value' if enabled_value else 'disabled'}", detail))

    def track_value(self, state_value, range_value, geo, clamps):
        """Returns: [(event_name, detail_text), ...] -- mostly empty list."""
        events_value = []

        # 1) Detection state transitions: fresh <-> hold (short gap) <-> coast (lost)
        if state_value != self._state:
            if self._state is not None:
                # fresh=bbox present | hold=brief gap, retain last command | coast=long loss, decay toward measured velocity
                # (coast)
                events_value.append((f"detection_{self._state}_to_{state_value}",
                                f"tip={geo.get('type_value', '')}"))
            self._state = state_value

        # 2 ) Restraints/clamps (edge ​​triggered)
        for label_item, enabled_value in clamps.items():
            self._edge(events_value, label_item, enabled_value, f"range={_ms(range_value)}")

        # 3 ) Range thresholds: first pass reported once
        if range_value is not None:
            for threshold_value in self.RANGE_THRESHOLDS:
                if range_value <= threshold_value and threshold_value not in self._crossed_threshold:
                    self._crossed_threshold.add(threshold_value)
                    events_value.append((f"range_{threshold_value:.0f}m",
                                    f"tip={geo.get('type_value', '')} "
                                    f"kapanma={_ms(geo.get('closure_mps'))}"))
            self._en_near = min(self._en_near, range_value)

        # 4 ) Closest transition: when closing changes sign (closing->opening)
        kap = geo.get('closure_mps')
        if kap is not None:
            now_shutting_down = kap > 0.0
            if (self._was_closing is True and not now_shutting_down
                    and range_value is not None and range_value < 200.0):
                events_value.append(('en_near_transition',
                                f"range={_ms(range_value)} type={geo.get('type_value','')} "
                                f"approximation_deg={_ms(geo.get('approximation_deg'))}"))
                self._miss_notified = False
            self._was_closing = now_shutting_down

        # 5) Miss: range doubled after closest pass
        if (range_value is not None and math.isfinite(self._en_near)
                and not self._miss_notified
                and self._en_near > 3.0
                and range_value > max(2.0 * self._en_near, self._en_near + 15.0)):
            self._miss_notified = True
            events_value.append(('miss_value', f"nearest_distance={self._en_near:.1f}m "
                                    f"current={range_value:.1f}m"))
        return events_value


def _ms(v, format_value='{:.1f}'):
    """None-safe short number formatter (for event detail texts)."""
    return '-' if v is None else format_value.format(v)


def _stab_ops(stab, i, fresh_value):
    """OPTIONAL i of tracker_bbox_stab. convert field to float or None.

    Depending on the publisher version the payload may be short (legacy bbox_to_redis or
    YILDIZ_MINRECT closed) -- then there is no space. Common version of handwritten pattern for
    tilt_deg."""
    if not fresh_value or stab is None or len(stab) <= i or stab[i] is None:
        return None
    try:
        return float(stab[i])
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------- dong

# CSV column layout. SINGLE SOURCE: both the title line and LOG_DICTIONARY.md reference this. WHEN ADDING
# a new column, add it to the end (older vehicles use DictReader, the name is important, not the
# order; adding it to the end keeps the diff readable though).
LOG_COLUMNS = [
    # --- time ---
    't', 't_mono', 't_unix', 'dt', 'authority', 'state_value',
    # --- image plane ---
    'ex_deg', 'ey_deg', 'bbox_w', 'bbox_h', 'area_root', 'area_px2',
    'coverage_pct', 'bbox_age_s', 't_capture',
    'px_virtual_x', 'px_virtual_y', 'px_raw_cx', 'px_raw_cy',
    'framing_edge_px', 'framing_edge_deg', 'raw_age_s', 'tilt_deg',
    # --- Range used by guidance (PERMITTED single target size) ---
    'range_m_value',
    # --- command ---
    'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_speed_mps', 'cmd_yaw_rate_dps',
    'clamp_speed', 'clamp_altitude', 'clamp_yaw_slew',
    # --- our own situation ---
    'pos_x', 'pos_y', 'pos_z', 'altitude_m',
    'vel_x', 'vel_y', 'vel_z', 'speed_mps',
    'acc_x_mps2', 'acc_y_mps2', 'acc_z_mps2',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'route_deg', 'vibe_max',
    # --- TARGET CONDITION: ANALYSIS ONLY (NOT guided, the 'ref_' prefix is ​​its visible sign) ---
    'ref_target_x', 'ref_target_y', 'ref_target_z',
    'ref_target_vx', 'ref_target_vy', 'ref_target_vz',
    'ref_target_speed_mps', 'ref_target_route_deg',
    'ref_target_ax_mps2', 'ref_target_ay_mps2', 'ref_target_az_mps2',
    'ref_target_turn_dps',
    # --- COMPARISON GEOMETRY: analysis only ---
    'ref_range_ground_truth_m', 'ref_bearing_deg', 'ref_elevation_deg',
    'ref_approximation_angle_deg', 'ref_encounter_type',
    'ref_closure_rate_mps', 'ref_tgo_s', 'ref_cpa_m', 'ref_cpa_s',
    # --- events (event names separated by '|' if not empty) ---
    'event_value',
    # --- SYSTEM HEALTH (2026-08-07, actual flight logging) ------------- All LOG-ONLY; none of them enter
    # into the control decision. They answer the first three questions asked after the accident: was the
    # loop in real time, was the process alive, was the autopilot listening to us.
    'loop_hz_mean', 'dt_excess', 'alive_ttl', 'ap_mode', 'hb_age_s',
    # --- TURNED RECTANGLE (Added to the end, index-protection) ------------------ bbox_to_redis is filled
    # when YILDIZ_MINRECT=1, NULL otherwise. rot_angle_deg display-CCW axis angle [-90,+90); target bank
    # CANDIDATE derives from this + roll_deg: bank ~ fold(roll_deg - rot_angle_deg). CAUTION: the strength
    # of this correlation measured OFFLINE is weak (see note on bbox_to_redis) -- the column is there for
    # ANALYSIS, it must be verified before making a control decision.
    'rot_w_px', 'rot_h_px', 'rot_angle_deg',
]


class VisualLoop:
    """It waits for authorization, seeds the handover, runs the controller with the MEASURED dt.

    There are two common protections on the command path (independent of method): * speed clamp: |v|
    <= VISUAL_MAX_SPEED_MPS (default 18, same as ceiling WPNAV_SPEED). * instruction LPF: tau_s
    first order. At the time of handoff, the LPF state is SEEDED by the last command of the
    positioner -> transition is non-jumping. If bbox disappears (age > bbox_stale_s) the command
    fades from its last value to zero; The decision maker returns to 'position' after dwell.
    """

    def __init__(self, controller, loop_hz=20.0, tau_s=0.35, bbox_stale_s=0.7,
                 gap_hold_s=1.0, altitude_floor_m=15.0,
                 yaw_tau_s=0.15, yaw_slew_dps2=120.0,
                 log_path=None, pursuer=None, target=None):
        self.k = controller
        self.loop_dt = 1.0 / float(loop_hz)
        self.tau = float(tau_s)
        self.bbox_stale_s = float(bbox_stale_s)
        self.gap_hold_s = float(gap_hold_s)
        self.altitude_floor_m = float(altitude_floor_m)
        self.yaw_tau_s = float(yaw_tau_s)
        self.yaw_slew_dps2 = float(yaw_slew_dps2)
        self.speed_ceiling = float(getattr(cfg, "VISUAL_MAX_SPEED_MPS", 18.0))
        # ATTENTION: locator's ports are NOT (14652/14603) -- two processes cannot connect the same udpin
        # port. Dedicated outputs for video are used.
        self.pursuer = pursuer or getattr(cfg, "VISUAL_PURSUER_CONN_STR",
                                          "udpin:127.0.0.1:14654")
        self.target = target or getattr(cfg, "VISUAL_TARGET_CONN_STR",
                                        "udpin:127.0.0.1:14604")
        stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = log_path or str(
            Path(__file__).resolve().parent / 'logs'
            / f"visual_{self.k.label_item}_{stamp_value}.csv")
        self._running_value = True
        self.r = None                 # _connect() doldurur
        self._ttl_counter = 0           # dead-man TTL sampling counter
        self._last_ttl = -1            # with last example(nen) TTL [s]

    # -- setup ----------------------------------------------------------

    def _connect(self):
        print(f"[image:{self.k.label_item}] Connecting to Redis...")
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.r.ping()
        self.bbox = BboxReader(self.r)
        self.bbox.start()
        print(f"[image:{self.k.label_item}] hunter link: {self.pursuer}")
        self.commander = SpeedCommander(self.pursuer)
        # Get home from hunter (for target global->NED conversion; same as positioned). When the stack is fast
        # restarted, the first request can get into a race and go unanswered (two runs like this happened in
        # 2026-08-04); Try again, don't give up in one try.
        msg = None
        for trial in range(1, 4):
            self.commander.conn.mav.command_long_send(
                self.commander.conn.target_system,
                self.commander.conn.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
            msg = self.commander.conn.recv_match(type="HOME_POSITION",
                                               blocking=True, timeout=10.0)
            if msg is not None:
                break
            print(f"[display:{self.k.label_item}] HOME_POSITION missed "
                  f"(trial {trial}/3)")
        if msg is None:
            raise SystemExit("HOME_POSITION could not be received (3 trial)")
        home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
        print(f"[display:{self.k.label_item}] home={home[0]:.7f},{home[1]:.7f} "
              f"sub={home[2]:.1f} target: {self.target}")
        self.range_provider = RangeEstimator(self.target, *home)
        self.range_provider.start()

    def _handoff_read(self):
        try:
            raw_value = self.r.get('handoff_state')
            return json.loads(raw_value) if raw_value else None
        except Exception:
            return None

    def _authority(self):
        try:
            v = self.r.get('command_authority')
            return v.decode('utf-8', 'replace') if v else 'position_based'
        except Exception:
            return 'position_based'

    def _transition_reason(self):
        """What rule triggered the transfer? bbox_to_redis._update_decision writes ('simple(5 sequential
frame)' / 'old(38/45 frame, ...)'). If the key is not present (older bbox_to_redis version) it
returns empty -- the event detail remains as before.
        """
        try:
            v = self.r.get('transition_reason')
            return v.decode('utf-8', 'replace') if v else ''
        except Exception:
            return ''

    def _alive_notify(self):
        """DEAD-MAN KEY: 'I am here and I can give commands'.

        CAUSE (2026-08-05 fault): When bbox_to_redis changed the authorization to 'video', it never
        asked whether the video controller was WORKING or not. When the controller is delegated
        without initialization (or hung in _connect()), the positional setpoint stops sending and NO
        ONE commands the vehicle: Range increased from 40 m to 124 m in a window of 5.9 s, yaw
        flutter 6.6x plus (measured, see guided_follow_20260805_145608.csv). The symptom looks like
        "MPC is shaking and not following the target at all" but MPC was NOT running at all.

        The key is not INTERMEDIATE: it is written with a TTL, so if the process dies the key
        disappears on its own and the decision maker does NOT switch to visual guidance / goes back. If
        Redis is not present, it is silently skipped (legacy behavior)."""
        if self.r is None:
            return
        try:
            self.r.set('visual_alive',
                       json.dumps({'label_item': self.k.label_item, 't_mono': time.monotonic()}),
                       ex=ALIVE_TTL_S)
        except Exception:
            pass

    def _alive_ttl(self):
        """LOG-ONLY: REMAINING TTL [s] of key 'visual_alive'.

        The dead-man switch acts silently. If the process hangs, the key expires, the decision maker
        reclaims authority, and the log may show only that rows stopped. This column verifies key
        refresh throughout the run. Normal operation gives 2, ALIVE_TTL_S. A value of 1 indicates
        the loop stalled for approximately 1 s or longer. A value of -1 means the key is absent or
        Redis was missed.

        COST: each cycle is a separate Redis round trip expensive ( 20 is a disproportionate slice
        of the cycle in Hz ).  10 is SAMPLED once in the loop, repeating the last value in between
        -- the column is still filled in each row.
        """
        self._ttl_counter += 1
        if self._ttl_counter % 10 == 1:
            try:
                v = self.r.ttl('visual_alive')
                # Redis-py: -2 no key, -1 no TTL. They both mean "not healthy"; combined into a single -1.
                self._last_ttl = int(v) if v is not None and v >= 0 else -1
            except Exception:
                self._last_ttl = -1
        return self._last_ttl

    # -- main dong -----------------------------------------------------------

    def run_value(self, duration_s_value=None):
        def _signal_value(signum, frame):
            self._running_value = False
        signal.signal(signal.SIGINT, _signal_value)
        signal.signal(signal.SIGTERM, _signal_value)

        self._connect()
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        log_f = open(self.log_path, 'w', newline='')
        log = csv.writer(log_f)
        log.writerow(LOG_COLUMNS)
        # EVENT LOG: discrete, sparse, human readable. The 'event' column of the main CSV carries the same
        # information but remains here with the detail text.
        event_path = self.log_path.replace('.csv', '_event.csv')
        if event_path == self.log_path:
            event_path = self.log_path + '.event_value.csv'
        event_f = open(event_path, 'w', newline='')
        event_w = csv.writer(event_f)
        event_w.writerow(['t', 't_unix', 'event_value', 'range_m_value', 'detail'])
        tracker_value = EventTracker()
        derivative_value = VelocityDifferentiator(max_history=5)
        acc_lpf = np.zeros(3)       # LPF which cuts the noise of acceleration derivative
        ACC_TAU = 0.20              # [s] -- noisy in derivative 20 Hz
        print(f"[image:{self.k.label_item}] log:{self.log_path}")
        print(f"[image:{self.k.label_item}] event log:{event_path}")
        print(f"[image:{self.k.label_item}] waiting for authorization "
              f"(Redis 'command_authority' == 'display')...")

        def _event_write(t_m, t_u, events_value, range_value):
            for label_item, detail in events_value:
                event_w.writerow([f"{t_m:.4f}", f"{t_u:.3f}", label_item,
                                 '' if range_value is None else f"{range_value:.2f}",
                                 detail])
            if events_value:
                event_f.flush()      # events are sparse: write to disk instantly

        _headerless_warning = [False]  # Column count warning, press before
        start_value3 = time.monotonic()
        active_value = False
        lpf_vel = np.zeros(3)
        previous_t = None
        last_request = None            # short spacer (gap_hold_s)
        # SHORT CLEAR YAW SUSTAIN (see note above HOLD_YAW_TAU_S). When the flag is OFF, these two variables
        # are written but NEVER read.
        hold_yaw_enabled = _environment_flag('YILDIZ_HOLD_YAW', 1.0) > 0.0
        last_yaw_cmd_value = None          # last yaw_rate actually commanded
        last_yaw_t = None            # monotonic time of that instruction
        lpf_yaw = 0.0               # yaw command smoothing status
        release_waiting = False      # Pseudo-handoff lock after MISS (below)
        # --- REAL-TIME HEALTH (2026-08-07) -- only measurement/alert does NOT touch the control path. The
        # pattern was copied from simple_guided_follow.py (where the loop falling into 2 Hz + constant dt
        # assumption was the root cause of the giant circles and flickering; the imaged skeleton had NEVER the
        # same measurement). Window ~2 s: 40 example in 20 Hz.
        dt_window = deque(maxlen=max(5, int(round(2.0 / self.loop_dt))))
        slow_streak = 0            # number of consecutive "dt large" cycles
        last_slow_warning = 0.0       # warning spam clamp [monotonic]
        previous_ap_mode = None        # autopilot mode change edge trigger
        row_counter = 0            # main CSV periodic flush counter
        while self._running_value:
            now_value = time.monotonic()
            if duration_s_value is not None and now_value - start_value3 > duration_s_value:
                break

            # DEAD-MAN: Declare us alive BEFORE ASKING for authority. It is also written while waiting for
            # authorization -- only then can the decision maker say "there is someone I can delegate it to."
            self._alive_notify()
            authorized = (self._authority() == 'visual')
            if not active_value:
                if release_waiting:
                    # We are just after the ISS: 'visual_release' is written but 1-37 ms intervenes (measured) until
                    # bbox_to_redis makes 'command_authority' 'imaged' -> 'located'. Meanwhile the authority was still
                    # 'displayed' and since active=False the following handoff INSTANTANEOUS block was doing a FAKE reinheritance
                    # (17 handoff_received event against 9 real handover in sim run). Don't take over until you see that
                    # authority has ACTUALLY gone off-screen; this flag does not block the next ACT handover (located ->
                    # displayed) because it is cleared as soon as the authorized False becomes available.
                    if authorized:
                        time.sleep(0.02)
                        continue
                    release_waiting = False
                if not authorized:
                    time.sleep(0.1)
                    continue
                # --- handoff MOMENT ---
                handoff = self._handoff_read()
                self.k.seed_value2(handoff)
                if handoff and 'cmd_vel_ned' in handoff:
                    lpf_vel = np.asarray(handoff['cmd_vel_ned'], dtype=float)
                else:
                    _, v = self.commander.reader.get()
                    lpf_vel = (np.asarray(v, dtype=float)
                               if v is not None else np.zeros(3))
                previous_t = None
                last_request = None      # Do not carry over command from previous engagement
                last_yaw_cmd_value = None    # same reason: yaw should not be moved either
                last_yaw_t = None
                lpf_yaw = 0.0
                active_value = True
                p_pos, _ = self.commander.reader.get()
                reason_value = self._transition_reason()
                _event_write(now_value, time.time(), [(
                    'handoff_received',
                    f"seed_speed={np.round(lpf_vel, 2).tolist()} "
                    f"handoff_state={'var' if handoff else 'NONE'}"
                    + (f" kural={reason_value}" if reason_value else ''))],
                    self.range_provider.range_value(p_pos))
                print(f"[image:{self.k.label_item}] >>> AUTHORITY HAS BEEN TAKEN over, "
                      f"seed speed={np.round(lpf_vel, 2).tolist()}"
                      f"{' (handoff_state absent, own_value hizimiz)' if not handoff else ''}"
                      f"{(' kural=' + reason_value) if reason_value else ''}")
                continue

            if not authorized:
                active_value = False
                p_pos, _ = self.commander.reader.get()
                _event_write(now_value, time.time(),
                          [('authority_to_position_returned', 'command interrupted')],
                          self.range_provider.range_value(p_pos))
                print(f"[image:{self.k.label_item}] authority returned to 'located'; "
                      f"command interrupted, waiting again")
                continue

            # --- measurement package --- raw_dt: wall clock step WITHOUT CLAMP. The dt given to the controller
            # caps at 0.5 s below (lest a long hang-up throw off the integrators) -- but the health meter is there
            # to see delays EXCEEDING that ceiling, so it uses the raw value.
            raw_dt = (self.loop_dt if previous_t is None
                      else max(now_value - previous_t, 1e-6))
            dt = (self.loop_dt if previous_t is None
                  else min(max(now_value - previous_t, 0.5 * self.loop_dt), 0.5))
            previous_t = now_value
            dt_window.append(raw_dt)
            dt_excess = 1 if raw_dt > 1.5 * self.loop_dt else 0
            loop_hz_mean = (len(dt_window)
                            / max(float(sum(dt_window)), 1e-6))
            # SLOW LOOP WARNING (streak logic, no spam): a single delay (GC, disk) is normal; 10 CONSEQUENTIAL
            # loop delay will choke the system and the results of that run will be questionable. stderr is pressed
            # -- stdout is the command stream, so the warning doesn't get lost there.
            slow_streak = slow_streak + 1 if dt_excess else 0
            if slow_streak >= 10 and now_value - last_slow_warning > 5.0:
                last_slow_warning = now_value
                print(f"[display:{self.k.label_item}] *** LOOP SLOW: measured "
                      f"{loop_hz_mean:.2f} Hz (requested {1.0/self.loop_dt:.1f} "
                      f"Hz, dt {1000.0*float(np.mean(dt_window)):.0f} ms) -- "
                      f"{slow_streak} sequential loop. Reduce CPU load ***",
                      file=sys.stderr, flush=True)
            t_unix = time.time()          # common ABSOLUTE time (log alignment)
            stab, bbox_age, coverage_value = self.bbox.last_value()
            raw_bbox, raw_age = self.bbox.raw_value()
            pos, vel = self.commander.reader.get()
            att = self.commander.reader.get_attitude()
            vibe = self.commander.reader.get_vibration()
            range_value = self.range_provider.range_value(pos)
            fresh_value = stab is not None and bbox_age <= self.bbox_stale_s
            # Our OWN acceleration: Numerical derivative of velocity LOCAL_POSITION_NED + LPF. (The target's
            # acceleration comes from IMM below; both are LOG only.)
            if vel is not None:
                derivative_value.update(now_value, float(vel[0]), float(vel[1]), float(vel[2]))
                raw_acc = np.asarray(derivative_value.get_acceleration('backwards'), float)
                aa = dt / (dt + ACC_TAU) if ACC_TAU > 1e-6 else 1.0
                acc_lpf = acc_lpf + aa * (raw_acc - acc_lpf)
            o = Measurement(
                t=now_value, dt=dt,
                ex_deg=float(stab[4]) if fresh_value else None,
                ey_deg=float(stab[5]) if fresh_value else None,
                bbox_w=float(stab[2]) if fresh_value else None,
                bbox_h=float(stab[3]) if fresh_value else None,
                area_root=(math.sqrt(float(stab[2]) * float(stab[3]))
                          if fresh_value else None),
                coverage_pct=coverage_value if fresh_value else None,
                bbox_age_s=bbox_age,
                t_capture=(float(stab[6]) if fresh_value and len(stab) > 6 else None),
                tilt_deg=(float(stab[7]) if fresh_value and len(stab) > 7
                          and stab[7] is not None else None),
                range_m_value=range_value,
                pos_ned=np.asarray(pos, dtype=float) if pos is not None else None,
                vel_ned=np.asarray(vel, dtype=float) if vel is not None else None,
                yaw_rad=att[2] if att is not None else None,
                roll_rad=att[0] if att is not None else None,
                pitch_rad=att[1] if att is not None else None,
                px_virtual_x=float(stab[0]) if fresh_value else None,
                px_virtual_y=float(stab[1]) if fresh_value else None,
                px_raw_cx=(float(raw_bbox[0]) + float(raw_bbox[2]) / 2.0
                           if raw_bbox is not None and raw_age <= self.bbox_stale_s
                           else None),
                px_raw_cy=(float(raw_bbox[1]) + float(raw_bbox[3]) / 2.0
                           if raw_bbox is not None and raw_age <= self.bbox_stale_s
                           else None),
                acc_ned=acc_lpf.copy() if vel is not None else None,
                # TURNED RECTANGULAR passthrough (same pattern as tilt_deg): fields are ADDED to the payload when
                # YILDIZ_MINRECT=1; Otherwise len(stab) <= 8 and all remains None.
                rot_w_px=_stab_ops(stab, 8, fresh_value),
                rot_h_px=_stab_ops(stab, 9, fresh_value),
                rot_angle_deg=_stab_ops(stab, 10, fresh_value),
                vibe_max=None if vibe is None else float(max(vibe)),
            )

            # --- controller ---
            yaw_rate_dps = None
            state_value = 'fresh_value'          # LOG: which branch we went on (set below)
            k_events = []         # discrete events declared by the controller
            if fresh_value:
                cmd = self.k.command_value(o)
                if getattr(cmd, 'event_value', ''):
                    k_events.append((cmd.event_value,
                                       getattr(cmd, 'event_detail', '')))
                    print(f"[image:{self.k.label_item}] EVENT: {cmd.event_value} "
                          f"{getattr(cmd, 'event_detail', '')}")
                if getattr(cmd, 'release_value', False):
                    # WAT: the controller (state machine in mpc_guidance) voluntarily relinquishes authority. The key
                    # 'visual_release' is read by bbox_to_redis._make_decision (see next to the 'manual_stop' block
                    # there). 'command_authority' is not written DIRECTLY: bbox_to_redis crushes it with its own mod every
                    # frame (~33 ms), so the value we wrote would be instantly deleted. 'manual_stop' is also unused: it
                    # is an OPERATOR kill-switch, latched (if not cleared it will never go back to 'display') and shared.
                    self.r.set('visual_release', json.dumps({
                        't_mono': now_value, 'reason_value': getattr(cmd, 'release_reason', ''),
                        'method_value2': self.k.label_item}))
                    self.commander.speed_send(np.asarray(cmd.vel_ned, float), None)  # COAST
                    active_value = False
                    release_waiting = True   # see guard before handoff INSTANTANEOUS
                    _event_write(now_value, time.time(),
                              [('MISS:birak', getattr(cmd, 'release_reason', ''))],
                              range_value)
                    print(f"[image:{self.k.label_item}] <<< MISS: authorization is released, "
                          f"reason = {getattr(cmd, 'release_reason', '')!r}")
                    continue
                request_value = np.asarray(cmd.vel_ned, dtype=float).reshape(3)
                yaw_rate_dps = cmd.yaw_rate_dps
                last_request = request_value.copy()
                if yaw_rate_dps is not None:
                    last_yaw_cmd_value = float(yaw_rate_dps)
                    last_yaw_t = now_value
            elif last_request is not None and bbox_age <= (self.bbox_stale_s
                                                        + self.gap_hold_s):
                # SHORT SPACE: KEEP last valid command (except yaw_rate). The old behavior (immediate damping to zero)
                # set up the vicious circle measured in the los_ellipse run: brake -> nose up pitch -> fixed camera
                # looks up -> the target already at the bottom edge comes out completely -> the loss becomes
                # permanent. Reference behavior 2LOSKF2._apply_last_command (1 s grace). yaw_rate BY DEFAULT is not
                # kept: blind turning also makes the target lose horizontally. When YILDIZ_HOLD_YAW=1, the last yaw
                # command is continued WITH SUSPENSION and TIME LIMITED -- see rationale and measurement. Block above
                # HOLD_YAW_TAU_S.
                request_value = last_request
                state_value = 'hold_value'
                if (hold_yaw_enabled and last_yaw_cmd_value is not None
                        and last_yaw_t is not None):
                    elapsed_item = now_value - last_yaw_t
                    if elapsed_item <= HOLD_YAW_MAXIMUM_S:
                        # Calculate from Elapsed TIME instead of square-by-square multiplication: even if dt fluctuates (cycle
                        # slows down), damping follows the same physical curve.
                        yaw_rate_dps = last_yaw_cmd_value * math.exp(
                            -elapsed_item / HOLD_YAW_TAU_S)
                # 'state' CONSCIOUSLY remains 'hold': phase statistics of base runs (fresh/hold/free %) are
                # comparable. How many frames the hold runs on is already read exactly from the condition
                # "state=='hold' AND cmd_yaw_rate_dps is not empty".
            else:
                # LONG loss: NOT ZERO, FLOATING (coast).
                #
                # OLD BEHAVIOR AND WHY IT WAS WRONG (2026-08-04, the root of the handoff shake the user saw in the
                # video): zero was written here. But zero is NOT "no command", it is the BRAKE command from 18 m/s to
                # full stop. Measured (mpc_20260804_160604): After handoff 1.2 s the target disappears, 1 s the holder
                # ends and the command 18.0 -> 13.8 -> 8.1 -> 4.7 -> 2.8 -> 0.19 crashes to m/s. Since the decision
                # maker dwell (2 s) has not been filled, we still have the authority; In other words, the vehicle has
                # a space that we call "stop". Moreover, this feeds on itself: brake -> nose up pitch -> fixed camera
                # looks up -> the target already on the edge disappears completely -> the loss becomes permanent
                # (agent LOS found the same chain independently).
                #
                # TRUTH: the acceleration command should be ZERO, not the speed command. Absorbing measured alignment
                # physically means "straight three, do nothing"; The geometry is not disturbed and the vehicle remains
                # stable until positioned authority takes over.
                if o.vel_ned is not None:
                    request_value = o.vel_ned.copy()
                elif last_request is not None:
                    request_value = last_request
                else:
                    request_value = np.zeros(3)
                state_value = 'coast'

            # joint guards: LPF + speed clamp
            a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
            lpf_vel = lpf_vel + a * (request_value - lpf_vel)
            n = float(np.linalg.norm(lpf_vel))
            clamp_speed = n > self.speed_ceiling          # LOG: did it touch the ceiling?
            v_cmd = lpf_vel * (self.speed_ceiling / n) if clamp_speed else lpf_vel

            # ABSOLUTE ALTITUDE FLOOR (2026-08-04 crash lesson): in all three routes, an error in the controller
            # layer commanded a continuous descent and drove the copter into the ground (while the target was 128+
            # m away). Method-independent last defense: no descent command is transmitted below the altitude floor. In NED
            # NED z increases downward and z = -altitude. Below the altitude floor,
            # pos[2] > -altitude_floor_m. Block only downward v_cmd[2] > 0.
            # The clamp leaves the LPF state and horizontal commands unchanged.
            clamp_altitude = (pos is not None
                              and float(pos[2]) > -self.altitude_floor_m
                              and v_cmd[2] > 0.0)
            if clamp_altitude:
                v_cmd = v_cmd.copy()
                v_cmd[2] = 0.0

            # YAW COMMAND SMOOTH (2026-08-04 jitter analysis): yaw_rate was coming RAW this far -- LPF was only
            # applied to the velocity channels. Measured: +-16 dps fluttering on yaw command 4 Hz with hard FOV
            # constraint of MPC on (step difference rms 4.3-6.2 dps, LOS 0.6-0.8 / PID 1.3), lag-1 autocorrelation
            # of increments ~0 i.e. random walk driven by white noise. The velocity channels are clear on the same
            # run (0.42-0.59) because they were passing through LPF. Two-stage protection: first the acceleration
            # (slew) clamp -- only cuts off the leaping steps, leaving legitimate fast turning untouched; then
            # light LPF. Both are INDEPENDENT from the method, common hygiene of the command path.
            clamp_yaw = False
            if yaw_rate_dps is not None:
                limit_value = self.yaw_slew_dps2 * dt
                outlier_value = float(np.clip(yaw_rate_dps,
                                       lpf_yaw - limit_value, lpf_yaw + limit_value))
                clamp_yaw = abs(outlier_value - yaw_rate_dps) > 1e-6   # LOG
                yaw_rate_dps = outlier_value
                ay = dt / (dt + self.yaw_tau_s) if self.yaw_tau_s > 1e-6 else 1.0
                lpf_yaw += ay * (yaw_rate_dps - lpf_yaw)
                yaw_cmd_dps = lpf_yaw
            else:
                lpf_yaw = 0.0          # yaw on autopilot: refresh status
                yaw_cmd_dps = None

            self.commander.speed_send(
                v_cmd,
                None if yaw_cmd_dps is None else math.radians(yaw_cmd_dps))

            # ================= LOG (control path ENDED, command sent) ==== From here on down DOES NOT AFFECT THE
            # BEHAVIOR OF THE VEHICLE. In particular, the ref_target_state() call is made here AFTER the command,
            # so that the answer to the question "has the target telemetry leaked into my guidance" can be read
            # even from the code itself: it cannot, the command is already gone.
            ref = self.range_provider.ref_target_state()
            geo = _encounter_geometry(pos, vel, ref['pos'], ref['vel'])
            h_vel = ref['vel']
            h_speed = None if h_vel is None else float(np.linalg.norm(h_vel))
            h_route = (None if h_vel is None or h_speed < 0.5 else
                      math.degrees(math.atan2(float(h_vel[1]),
                                              float(h_vel[0]))) % 360.0)
            own_speed = None if vel is None else float(np.linalg.norm(vel))
            own_route = (None if vel is None or own_speed < 0.5 else
                        math.degrees(math.atan2(float(vel[1]),
                                                float(vel[0]))) % 360.0)
            # Distance to frame edge: RAW center is used -- it tells you how long until the subject physically
            # leaves the frame (virtual pixels may extend out of frame, it's a calculation).
            edge_px = edge_deg = None
            if o.px_raw_cx is not None:
                edge_px = min(o.px_raw_cx, _FRAMING_W - o.px_raw_cx,
                               o.px_raw_cy, _FRAMING_H - o.px_raw_cy)
                edge_deg = math.degrees(math.atan(edge_px / _FRAMING_FX))

            # --- SYSTEM HEALTH (LOG-ONLY, command has already been sent) --- Autopilot mode and heartbeat age:
            # the only evidence of the question "was the vehicle LISTENING to our setpoints?" If a mode other than
            # GUIDED (LAND/RTL/STABILIZED/failsafe) is displayed, all guidance columns are incorrect -- the
            # command is sent but not transferred to the vehicle.
            hb = self.commander.reader.get_heartbeat()
            ap_mode = '' if hb is None else (hb[0] or str(hb[1]))
            hb_age_s = None if hb is None else hb[2]
            alive_ttl = self._alive_ttl()

            clamps = {'clamp_speed': clamp_speed,
                          'clamp_altitude': clamp_altitude,
                          'clamp_yaw_slew': clamp_yaw}
            events_value = tracker_value.track_value(state_value, range_value, geo, clamps)
            events_value = k_events + events_value      # controller events are also written
            # MODE CHANGE: edge triggered, most valuable line in the event log. The first seen mode is also
            # written (formerly None) -- fixes what mode the vehicle was in at the time of handoff.
            if ap_mode and ap_mode != previous_ap_mode:
                events_value = events_value + [('ap_mode_changed',
                                      f"{previous_ap_mode or '-'} -> {ap_mode}")]
                print(f"[display:{self.k.label_item}] AUTOPILOT MODE: "
                      f"{previous_ap_mode or '-'} -> {ap_mode}")
                previous_ap_mode = ap_mode
            _event_write(now_value, t_unix, events_value, range_value)

            def _n(v, b='{:.2f}'):
                return '' if v is None else b.format(v)

            row_value = [
                f"{now_value:.4f}", f"{now_value:.4f}", f"{t_unix:.3f}",
                f"{dt:.4f}", 'visual', state_value,
                _n(o.ex_deg, '{:.4f}'), _n(o.ey_deg, '{:.4f}'),
                _n(o.bbox_w, '{:.0f}'), _n(o.bbox_h, '{:.0f}'),
                _n(o.area_root), _n(None if o.bbox_w is None
                                   else o.bbox_w * o.bbox_h, '{:.0f}'),
                _n(o.coverage_pct, '{:.3f}'),
                f"{bbox_age:.3f}" if math.isfinite(bbox_age) else '',
                _n(o.t_capture, '{:.4f}'),
                _n(o.px_virtual_x, '{:.1f}'), _n(o.px_virtual_y, '{:.1f}'),
                _n(o.px_raw_cx, '{:.1f}'), _n(o.px_raw_cy, '{:.1f}'),
                _n(edge_px, '{:.1f}'), _n(edge_deg, '{:.2f}'),
                f"{raw_age:.3f}" if math.isfinite(raw_age) else '',
                _n(o.tilt_deg, '{:.3f}'),
                _n(range_value),
                f"{v_cmd[0]:.3f}", f"{v_cmd[1]:.3f}", f"{v_cmd[2]:.3f}",
                f"{float(np.linalg.norm(v_cmd)):.3f}",
                _n(yaw_cmd_dps), int(clamp_speed), int(clamp_altitude),
                int(clamp_yaw),
                *(('', '', '', '') if pos is None else
                  (f"{pos[0]:.2f}", f"{pos[1]:.2f}", f"{pos[2]:.2f}",
                   f"{-float(pos[2]):.2f}")),
                *(('', '', '', '') if vel is None else
                  (f"{vel[0]:.2f}", f"{vel[1]:.2f}", f"{vel[2]:.2f}",
                   f"{own_speed:.2f}")),
                *(('', '', '') if o.acc_ned is None else
                  (f"{o.acc_ned[0]:.2f}", f"{o.acc_ned[1]:.2f}",
                   f"{o.acc_ned[2]:.2f}")),
                *(('', '', '') if att is None else
                  (f"{math.degrees(att[0]):.2f}", f"{math.degrees(att[1]):.2f}",
                   f"{math.degrees(att[2]):.2f}")),
                _n(own_route, '{:.1f}'),
                '' if vibe is None else f"{max(vibe):.1f}",
                # --- ref_* : ANALYSIS ONLY, not guided ---
                *(('', '', '') if ref['pos'] is None else
                  (f"{ref['pos'][0]:.2f}", f"{ref['pos'][1]:.2f}",
                   f"{ref['pos'][2]:.2f}")),
                *(('', '', '') if h_vel is None else
                  (f"{h_vel[0]:.2f}", f"{h_vel[1]:.2f}", f"{h_vel[2]:.2f}")),
                _n(h_speed), _n(h_route, '{:.1f}'),
                *(('', '', '') if ref['acc'] is None else
                  (f"{ref['acc'][0]:.2f}", f"{ref['acc'][1]:.2f}",
                   f"{ref['acc'][2]:.2f}")),
                _n(ref['turn_dps'], '{:.2f}'),
                _n(geo['range_m_value']), _n(geo['bearing_deg'], '{:.1f}'),
                _n(geo['elevation_deg'], '{:.2f}'),
                _n(geo['approximation_deg'], '{:.1f}'), geo['type_value'],
                _n(geo['closure_mps']), _n(geo['tgo_s']),
                _n(geo['cpa_m']), _n(geo['cpa_s']),
                '|'.join(label_item for label_item, _ in events_value),
                # --- system health (added SONA, index-protection) ---
                f"{loop_hz_mean:.2f}", dt_excess, alive_ttl, ap_mode,
                _n(hb_age_s, '{:.2f}'),
                # --- opaque rectangle (added SONA, index-preservation) ---
                _n(o.rot_w_px, '{:.1f}'), _n(o.rot_h_px, '{:.1f}'),
                _n(o.rot_angle_deg, '{:.2f}'),
            ]
            if len(row_value) != len(LOG_COLUMNS) and not _headerless_warning[0]:
                _headerless_warning[0] = True
                print(f"[display:{self.k.label_item}] LOG WARNING: on line "
                      f"There is a field {len(row_value)}, {len(LOG_COLUMNS)} in the title. "
                      f"Columns can be SHIFTED!")
            log.writerow(row_value)
            # PERIODIC FLUSH (2026-08-07): master CSV used to be flushed ONLY at shutdown -- so on crash/SIGKILL
            # the last ~2 s (8 KB block buffer) were never written to disk and crashed The lines of the memory
            # were exactly the missing part. Same pattern as in mpc_diagnostic: 20 one per row, 20 Hz ~1 Hz.
            row_counter += 1
            if row_counter % 20 == 0:
                log_f.flush()

            remaining_value = self.loop_dt - (time.monotonic() - now_value)
            if remaining_value > 0:
                time.sleep(remaining_value)

        log_f.close()
        event_f.close()
        print(f"[image:{self.k.label_item}] loop finished, log closed")


# ----------------------------------------------------------------- dry test

class _HolderController(VisualController):
    """The simplest controller to test integration glue: maintains the same speed after handoff. It does
NOT guide; It's just to prove that the handoff mechanic + command path works."""
    label_item = "holder_value"

    def __init__(self):
        self.v = np.zeros(3)

    def seed_value2(self, handoff):
        if handoff and 'cmd_vel_ned' in handoff:
            self.v = np.asarray(handoff['cmd_vel_ned'], dtype=float)

    def command_value(self, measurement):
        return Command(vel_ned=self.v.copy())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--holder-value', action='store_true',
                   help='run with gripper dry-test controller')
    p.add_argument('--duration-value', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    a = p.parse_args()
    if not a.holder_value:
        raise SystemExit("This module is the skeleton; import from method code "
                         "or run a dry test with --holder-value.")
    VisualLoop(_HolderController(), loop_hz=a.loop_hz).run_value(a.duration_value)


if __name__ == '__main__':
    main()
