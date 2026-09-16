#!/usr/bin/env python3
"""
single_node_guidance.py - Independent visual-guidance node running in a single process
=========================================== WHY EXISTS (architectural objection, 2026-08-07 team
request) -------------------------------------------------- DECISION DETECTION PROCESS GIVES IN
TODAY'S CHAIN:

    bbox_to_redis.py --('command_authority' = displayed/located)--> command
    visual_base.VisualLoop -> WAITS for authorization, runs MPC when it arrives

In other words, the seeing eye answers the question "Should I engage?", not the hand holding the
gun. This is his teammate's objection: THE GUIDANCE NODE SHOULD MAKE ITS OWN DECISION. This file
performs the SAME FUNCTION in a single process and by its OWN decision:

    bbox_to_redis.py --(broadcast 'tracker_bbox_stab', MEASUREMENT ONLY)--> THIS NODE THIS NODE:
    evaluate gates -> ENGAGE -> LOS/PN -> speed command -> RELEASE

'command_authority' CANNOT BE READ AT ALL. The decision is at the door of engagement below.

CURRENT GUIDANCE LAW AND HARDWARE CONFIGURATION (2026-08-11) ----------------------------- * The default visual-guidance
law is terminal_los_guidance.TerminalLosController. MPC is loaded with ``--guidance-value mpc`` for
comparison/return only; The default installation does not depend on optimization libraries. *
Hardware gate ``--large-frame 5 --area-pct 3``: engages when five consecutive FRESH bbox cross the
area threshold. Centering and target telemetry are not required by the engagement gate. The YOLO confidence filter is applied
BEFORE detection on the camera bridge; Therefore, only frames accepted by the detector enter the
counter. * Position guidance continues to establish the slot behind the target. LOS/PN works only
after visual authorization is obtained.

PROTECTED PARTS (on purpose): measurement contract, safety and release layer
-------------------------------------------------------------- * Same as the ENGAGEMENT RULE:
bbox_to_redis's THREE DOORS rule (window + area + range), which was confirmed today, has been MOVED
here. Copied because that logic is embedded in the internal state of the SwarmRedisDetector class
(ROS frame loop, decision_window); It does not have a surface that can be imported. Thresholds are
read from the same env variables so that the two sides do not separate. * Error signal comes from
virtual gimbal: Redis 'tracker_bbox_stab' [sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps].
'tracker_bbox' with raw element 7 is the SPARE path (if there is no stabilized channel; see
_raw_stab). * TARGET TELEMETRY PROVIDES RANGE ONLY: the only quantity derived from the target's telemetry is range
(user rule 2026-08-03). Two sources can be selected: estimator -> visual_base.RangeEstimator
(IMM, target's GLOBAL_POSITION_INT, own MAVLink link 14604) Redis -> 'handoff_state'.range_m (if
feeding ground station/located) Range is interrupted, the LAST VALID VALUE IS FROZEN and warned
explicitly -- in LOS/PN, PN uses the gain, t_go uses the range for the vertical channel and miss detection.
Therefore, although visual engagement is telemetry-free, the current terminal law is not yet completely
telemetry-free. * Command is SPEED: SET_POSITION_TARGET_LOCAL_NED, vx,vy,vz only (+yaw_rate). Safety
layer (speed clamp, altitude floor, command LPF, yaw slew+
    LPF) MOVED from visual_base.VisualLoop.run(). It was copied because that layer is
    embedded in the body of a single giant method, not a separate function. The justification for
    each piece was preserved along with it, so that if the two files were separated, the WHY would
    not be lost.

ENGAGEMENT STATE MACHINE (main difference: decision here)
------------------------------------------------- WAIT --(engagement door opened)--> ENGAGED --(lost
ladder | detection rate collapsed | HIT)--> DROP -> WAIT

  DEFAULT RULE -- THREE DOORS (SAME AS bbox_to_redis): 1) DETERMINATION: >= 20 of the last 25 frame
  must be valid detection (%80). Thresholds are from the same env as bbox_to_redis:
  YILDIZ_WINDOW_FRAME / YILDIZ_WINDOW_RATIO. 2) AREA: The area of ​​bbox must be larger than the
  rectangle of the frame (%2 x %2)
        (calibration: bbox_area ~ 4.65e5/r^2 -> %2x%2 ~ 35 m).
     3) RANGE: range <= 60 m. If the range is unavailable/stale this port will be SKIPPED (same
     behavior as in bbox_to_redis).

  WINDOW LENGTH IS MEASURED IN FRAMES (2026-08-07 user decision): 25 frame 30 in fps 0.83 s, ~20 in
  Hz In hardware YOLO it is 1.25. (The old 45 frame was 1.5 s and 2.25 s, respectively -- the same
  code was engaged half a second late in hardware; the 25 frame keeps both sides in the reasonable
  band.) BOTH speeds are written on each line so that no guesswork is required to see this
  difference in the log: detection_fps : BROADCAST rate of bbox_to_redis (discrete counter measures the
  thread) observation_fps : individual frame rate that the loop SEEs = min(loop_hz, detection_fps) The
  window operates on observed frames -- the homing can only react to the frame it sees. So in a
  cycle of 20 Hz, 25 frames are 1.25 s, even if the camera is 30 fps. --loop-hz 30 (or
  --window-frame) if you want it shorter. Anyone can also install the window in DURATION:
  --window-s-value (NOT the default).

  SIMPLE RULE (option, DEFAULT OFF): --simple-frame If N is given, three gates are skipped, "N valid
  frames in a row" is enough. Same logic as bbox_to_redis in YILDIZ_TRANSITION_SIMPLE; For those who want
  predictable behavior on the field. Env equivalent: YILDIZ_SINGLE_NODE_FRAME.

  LOSS LADDER (same thresholds as visual_base , same reasons): age <= 0.7 s -> ' fresh_value ', MPC
  runs 0.7 < age <= 1.7 s -> 'hold', last valid command HOLD age > 1.7 s -> 'without', coast along the measured
  velocity direction (NOT zero: zero is not "no command" but full brake command) age > release_s
      (2.5 s) -> RELEASE Second way to release (fallback rule of bbox_to_redis): RELEASE if the
      current number of frames in the window drops below 3 and 2 s dwell is full. The loss ladder
      captures CONSTANT loss, this path captures Flickering detection. There is COOL DOWN after DROP
      (default 3 s): bbox reappears as soon as engaged produces ping-pong (measured in
      bbox_to_redis: cycle ~1.2 s, ~50 handoff -> HANDOFF_COOLDOWN_S per minute).

OUTPUT (protocol formation_KILLER) ------------------------------------------ Redis 'single_node_state'
: JSON on EVERY state change + periodic refresh {'state_value','reason_value','t_mono','t_unix','range_m_value',...}
Redis 'single_node_alive': Heartbeat with TTL (dead-man switch) MAVLink STATUSTEXT : "DISP: I'm in
command" / "DISP: I let go, I'm listening" (closes with --no-statustext) The authority channel is
Redis; STATUSTEXT is echo for human/GCS (formation_KILLER already logs STATUSTEXT, see
_handle_statustext).

USAGE
--------
    python3 hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3
    python3 hardware/single_node_guidance.py --guidance-value mpc            # A/B only
    python3 hardware/single_node_guidance.py --dry-run --mock-mavlink   # table test

Detailed description: hardware/README.md
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
from datetime import datetime
from pathlib import Path

import numpy as np
import redis

# --- module path inside the repo ------------------------------------------------- This file is under
# hardware/ but the guidance code is under guidance_allstar/ and that package itself uses PLAIN import
# ("import guidance_config exceed cfg"). That's why we import it not as a package, but by adding the
# directory to sys.path; Otherwise, visual_base's own imports will be broken.
_ROOT = Path(__file__).resolve().parent.parent
_GA = _ROOT / 'guidance_allstar'
if str(_GA) not in sys.path:
    sys.path.insert(0, str(_GA))

import guidance_config as cfg                                   # noqa: E402
from visual_base import (                                   # noqa: E402
    ALIVE_TTL_S,
    BboxReader,
    SpeedCommander,
    LOG_COLUMNS,
    RangeEstimator,
    Measurement,
    EventTracker,
    _FRAMING_FX,
    _FRAMING_H,
    _FRAMING_W,
    _encounter_geometry,
)
from terminal_los_guidance import TerminalLosController             # noqa: E402
from numeric_differentiation import VelocityDifferentiator      # noqa: E402

# UNDERLINE NAMES: _FRAME_* and _encounter_geometry are marked "in-module" on the visual_base, but
# both are LOG-ONLY auxiliary. We import rather than copy -- if the frame geometry and encounter
# analysis are SEPARATE in two files, the logs become incomparable.

# ---------------------------------------------------------------- constants

# Engagement states (Redis 'single_node_state' broadcasts them verbatim).
WAIT = 'WAIT'
ENGAGED = 'ENGAGED'

# Redis keys. INTENTIONALLY DOES NOT REUSE 'visual_alive' on purpose: bbox_to_redis is looking
# for that key at the dead-man's door, and it is not this node's job to pretend there is a process
# there (we don't ask for authority, we GET authority).
KEY_STATE = 'single_node_state'
KEY_ALIVE = 'single_node_alive'
KEY_HANDOFF = 'handoff_state'          # READ only (seed + Redis range)
KEY_AUTHORITY = 'command_authority'         # only written as OPTIONAL, NOT READ

# When the range is interrupted: the last value is frozen, after which a loud warning is given. The
# alert is periodic (no spam).
RANGE_FREEZE_WARNING_S = 1.0
RANGE_WARNING_PERIOD_S = 5.0

# STATUSTEXT texts -- PROTOCOL. If you change it, also change it on the formation_KILLER side (the
# table in README has the same texts).
ST_ENGAGED = "DISPLAY: I have the command"
ST_RELEASE = "VIDEO: I let go, I'm listening"

# Single node-SPECIFIC columns to the master CSV. Appended AT THE END of LOG_COLUMNS: older vehicles
# use DictReader (the name matters, not the order), and the append keeps the diff readable -- same
# rule as for visual_base.
EK_COLUMNS = [
    'single_node_state',     # WAIT | ENGAGED
    'kural',              # three_gate_frame | three_gate_duration | simple
    'detection_fps',         # Publication rate, log-only: the Hz produced by bbox_to_redis . Use a separate thread to distinguish it from the loop's observed rate, kapi.fps.
                          # YOLO on hardware ~20). Measured by FrameCounter, LOG ONLY.
    'observation_fps',         # individual frame rate SEEN by the loop = min(loop_hz,
                          # detection_fps ). THIS is how the engagement window works.
    'window_sample',      # number of samples in stability window (<= 25)
    'valid_frame',       # valid number of samples in window (threshold 20)
    'valid_ratio',       # current sample rate in window [0..1]
    'gate_area',          # 1 = field door open (bbox is large enough)
    'gate_range',        # 1 = range gate open (or bypassed)
    'consecutive_frame',       # simple rule counter
    'consecutive_large_frame', # short LOS gate: successive field-valid bbox
    'loss_s',            # uninterrupted bbox loss time [s]
    'range_source',      # estimator | redis | fake
    'range_fresh',        # 1 = measured, 0 = FROZEN (last valid value)
    'range_age_s',       # time since last valid range
    'dry',               # 1 = --dry-run (command did NOT go to MAVLink)
]


# ------------------------------------------------------------------------------- range source
#
# All three sources give the same surface: range(pos_ned) -> float|None ref_target_state() -> dict
# (LOG-ONLY) So the selection of --range-source remains ONE line in the loop code.


class EstimatorRange:
    """visual_base.RangeEstimator wrapper (DEFAULT source).

    WE CANNOT COPY the estimator: the IMM setup, the staleness threshold, and the "outside range
    only" contract are all there in one place."""

    label_item = 'estimator'

    def __init__(self, target_conn_str, home):
        self._k = RangeEstimator(target_conn_str, *home)
        self._k.start()

    def range_value(self, pos_ned):
        return self._k.range_value(pos_ned)

    def ref_target_state(self):
        return self._k.ref_target_state()


class RedisRange:
    """'handoff_state'.range_m -- if positioned process / feeding ground station.

    WHY THERE IS: the default is the estimator that opens its own MAVLink connection (14604) so that
    this node can fly alone; but if a process is running that ALREADY calculates the same range
    (simple_guided_follow writes range_m to 'handoff_state' every cycle), running a second IMM is a
    waste of CPU and a second source of estimation error. NOT A VIOLATION: the only field read is
    range_m.
    """

    label_item = 'redis'
    STALE_S = 3.0          # Same threshold as bbox_to_redis.HANDOFF_STALE_S

    def __init__(self, r):
        self.r = r

    def range_value(self, pos_ned):
        try:
            raw_value = self.r.get(KEY_HANDOFF)
            if not raw_value:
                return None
            d = json.loads(raw_value)
            range_value = d.get('range_m')
            t = d.get('t_mono')
            if range_value is None:
                return None
            # t_mono is the monotonic clock of the WRITTEN PROCESS; The difference is significant since it's the
            # same clock on the same machine (CLOCK_MONOTONIC system-wide on Linux). On different machine (Pi <->
            # place) this check will fail -> use estimator in that setup.
            if t is not None and (time.monotonic() - float(t)) > self.STALE_S:
                return None
            return float(range_value)
        except Exception:
            return None

    def ref_target_state(self):
        # We do not have access to the target's position/velocity/acceleration -> ref_* columns are empty.
        return {'pos': None, 'vel': None, 'acc': None,
                'turn_dps': None, 'est_pos': None}


class MockRange:
    """DESK TEST: produces a dummy range that closes (60 m -> 5 m, 5 m/s)."""

    label_item = 'mock'

    def __init__(self, start_m=60.0, closure_mps=5.0):
        self.t0 = time.monotonic()
        self.r0 = float(start_m)
        self.v = float(closure_mps)

    def range_value(self, pos_ned):
        return max(5.0, self.r0 - self.v * (time.monotonic() - self.t0))

    def ref_target_state(self):
        return {'pos': None, 'vel': None, 'acc': None,
                'turn_dps': None, 'est_pos': None}


class FrameCounter(threading.Thread):
    """It COUNTS arrivals of 'tracker_bbox_stab' -- its only job is to measure the BROADCAST speed.

    WHY SEPARATE THREAD (measured in dry test): if the camera is outputting 30 fps while the
    guidance loop 20 Hz is running, the number of "separate frames" counted through the loop
    increases at most to the LOOP speed (measured value was 19.9). So the loop cannot IN PRINCIPLE
    see the actual speed of the broadcast. This thread measures the actual speed as it sees every
    message.

    The number is for LOG ONLY (column detection_fps): "sim 30 fps / hardware ~20 Hz YOLO" so that the
    difference can be read from the logs without guessing. The engagement window operates NOT by
    this number, but by the frames the loop SEEs -- steering can only react to the frame it sees."""

    def __init__(self, r, channel_value='tracker_bbox_stab'):
        super().__init__(daemon=True)
        self.pubsub = r.pubsub(ignore_subscribe_messages=True)
        self.pubsub.subscribe(channel_value)
        self.lock_value = threading.Lock()
        self._t = deque(maxlen=120)

    def run(self):
        for _ in self.pubsub.listen():
            with self.lock_value:
                self._t.append(time.monotonic())

    def fps(self):
        now_value = time.monotonic()
        with self.lock_value:
            while self._t and now_value - self._t[0] > 3.0:
                self._t.popleft()
            n = len(self._t)
            if n < 2:
                return 0.0
            return (n - 1) / max(self._t[-1] - self._t[0], 1e-6)


class RangeHolder:
    """Kaynagin ustune DONDURMA + WARNING katmani.

    WHY (user request 2026-08-07): passing None when range is cut drops MPC to assumption
    MpcConfig.range_if_absent_m (55 m). Assuming 55 m while 20 closes at m reduces the c = KVVA/r
    coefficients by ~3 times, that is, MPC produces the soft command "I have time". Freezing the
    last TRUE measurement is a wrong but CONSISTENT assumption and can be distinguished later from
    the status columns (range_fresh / range_age_s)."""

    def __init__(self, source_value, write_value=print):
        self.source_value = source_value
        self._last = None
        self._last_t_value = 0.0
        self._last_warning = 0.0
        self._write = write_value
        self.fresh_value = False
        self.age_s_value = float('inf')

    def range_value(self, pos_ned):
        m = self.source_value.range_value(pos_ned)
        now_value = time.monotonic()
        if m is not None and math.isfinite(m):
            self._last, self._last_t_value = float(m), now_value
            self.fresh_value, self.age_s_value = True, 0.0
            return self._last
        self.fresh_value = False
        if self._last is None:
            self.age_s_value = float('inf')
            self._warn(now_value, "RANGE NEVER CAME -- MPC range_if_absent_m "
                              "(default is 55 m) constant!")
            return None
        self.age_s_value = now_value - self._last_t_value
        if self.age_s_value > RANGE_FREEZE_WARNING_S:
            self._warn(now_value, f"RANGE CUT ({self.age_s_value:.1f} s) -- end "
                              f"current value {self._last:.1f} m FROZEN")
        return self._last

    def _warn(self, now_value, text_value):
        if now_value - self._last_warning < RANGE_WARNING_PERIOD_S:
            return
        self._last_warning = now_value
        self._write(f"[tekdugum] *** {text_value} ***")


# ------------------------------------------------------------------------------- engagement door

class EngagementGate:
    """“Should I engage?” SINGLE answer place for the question.

    SOURCE: bbox_to_redis . SwarmRedisDetector decision maker (_evaluate_frame / _make_decision).
    MOVED from there because that logic is embedded in the ROS frame loop and class internal state
    -- it has no importable surface. Thresholds are read from the SAME env variables (
    YILDIZ_WINDOW_FRAME / _RATIO / YILDIZ_TRANSITION_AREA_PCT / YILDIZ_TRANSITION_RANGE ) so that both sides
    behave the same with the same setting.

    DEFAULT: Three doors with SQUARE windows -- valid at square 25 >= 20 (%80). NOTE (2026-08-07
    user decision): window is in SQUARES, not duration; 25 frame 30 is 0.83 s at fps, 20 is Hz at
    YOLO is 1.25 s. The old 45 frame was saying 1.5 s on the shim / 2.25 s on the hardware --
    handoff was delayed by half a second before the code changed. 25 square keeps both sides in
    reasonable band. The measured detection rate (fps) is in any case LOGGED (column detection_fps) so
    that no guesswork is required to see this difference from the log.

    OPTION: With --window-s-value the window becomes DURATION based (then it is not the number of frames
    but the rate of samples in the last X seconds). NOT default; It stands ready in case the fps
    changes too much on the hardware.

    HOW DO WE SET UP THE FRAME WINDOW (important difference): bbox_to_redis sees every CAMERA frame
    and writes False to the window for invalid ones. We only see VALID detections (bbox_to_redis
    only broadcasts them to Redis). Therefore, missed frames are reconstructed from the MEASURED
    frame period (1/fps): if more than one period has passed since the last frame, False is written
    to the window. The result has the same meaning as bbox_to_redis's window; The quantization
    difference is one frame period.
    """

    # Same as bbox_to_redis.REVERT_THRESHOLD: If 3 remains valid frame in window of 25 frames (+2 s
    # dwell), detection is considered LOST.
    RELEASE_FRAME = 3
    DEFAULT_FPS = 30.0        # frame period used until fps is measured

    def __init__(self, window_frame=None, valid_ratio=None, window_s_value=None,
                 area_pct=None, range_threshold_m=None, simple_frame=None,
                 large_frame=None, release_dwell_s=2.0):
        # Thresholds are from the SAME env as bbox_to_redis ; Once scenario.sh is set, both sides see the same
        # number.
        self.window_frame = max(3, int(float(
            window_frame if window_frame is not None
            else os.environ.get('YILDIZ_WINDOW_FRAME', '25'))))
        self.valid_ratio = float(
            valid_ratio if valid_ratio is not None
            else os.environ.get('YILDIZ_WINDOW_RATIO', '0.8'))
        self.valid_threshold = max(2, int(round(self.window_frame
                                             * self.valid_ratio)))
        self.window_s_value = None if window_s_value is None else float(window_s_value)
        self.area_pct = float(area_pct if area_pct is not None
                              else os.environ.get('YILDIZ_TRANSITION_AREA_PCT', '2'))
        self.range_threshold_m = float(
            range_threshold_m if range_threshold_m is not None
            else os.environ.get('YILDIZ_TRANSITION_RANGE', '60'))
        self.simple_frame = None if not simple_frame else max(1, int(simple_frame))
        self.large_frame = None if not large_frame else max(1, int(large_frame))
        self.release_dwell_s = float(release_dwell_s)
        # FAIL threshold (hysteresis): when engaged, the validity criterion is coverage, not AREA, and its
        # threshold is very low -- bbox_to_redis.MIN_COVERAGE_HOLD. The rationale is there: the rollback
        # should manage the detection LOSS, not the size.
        self.stay_coverage_pct = float(os.environ.get('YILDIZ_COV_KAL', '0.3'))
        self._window = deque(maxlen=(None if self.window_s_value is not None
                                      else self.window_frame))  # (t, valid)
        self._frame_t = deque(maxlen=90)  # SEPARATE frame arrivals (fps measurement)
        self._last_slot_t = None          # moment of the last written window slot
        self._last_signature = None
        self.consecutive_frame = 0
        self.consecutive_large_frame = 0
        self._release_dwell_t0 = None
        # Last evaluation results (LOG columns write these). self.fps = individual frame rate that the loop
        # SEEs (column 'observation_fps'), NOT the broadcast rate: if the loop is 20 Hz, at most 20 individual
        # frames can be seen from the broadcast of 30 fps. FrameCounter measures broadcast speed.
        self.fps = 0.0
        self.ratio_value = 0.0
        self.sample_value = 0
        self.valid_count_value = 0
        self.gate_area = False
        self.gate_range = False

    @property
    def mode_value(self):
        if self.large_frame:
            return 'large_frame'
        if self.simple_frame:
            return 'simple'
        return 'three_gate_duration' if self.window_s_value is not None else 'three_gate_frame'

    # -- measurement input -----------------------------------------------------

    def area_threshold_px2(self):
        """The area of ​​rectangle (p% x p%) is the same as [px^2] -- bbox_to_redis."""
        p = self.area_pct / 100.0
        return (p * _FRAMING_W) * (p * _FRAMING_H)

    def frame_period(self):
        """OBSERVED frame period [s]; default if not yet measured.

        Window slots progress with this period. The observation rate is used, NOT the broadcast rate
        (detection_fps); Otherwise, the frames that the loop cannot see are considered 'missed' and the
        window will never be filled."""
        return 1.0 / (self.fps if self.fps > 1e-6 else self.DEFAULT_FPS)

    def frame_stale_s(self):
        """Upper time [s] after which a detection is considered a "new frame".

        Derived from MEASURED fps (~2.5 frame period). A fixed threshold (e.g. 0.15 s) would count
        30 as accurate at fps, 10 at Hz as stale at a YOLO. Clamp: most az 0.10 s, most 0.35 s."""
        return float(min(0.35, max(0.10, 2.5 * self.frame_period())))

    def sample_value2(self, t, signature_value, bbox_age_s, coverage_pct, area_px2, range_value,
                range_fresh, engaged):
        """Called once per LOOP. Updates the window and counters."""
        # 1) SEPARATE frame arrivals -> fps. Signature (t_capture + pixels/size) is used to avoid counting the
        # same frame twice: camera 30 Hz, loop 20 Hz -- unsigned count would equalize fps to loop rate.
        new_frame = signature_value is not None and signature_value != self._last_signature
        if new_frame:
            self._last_signature = signature_value
            self._frame_t.append(t)
        while self._frame_t and t - self._frame_t[0] > 3.0:
            self._frame_t.popleft()
        if len(self._frame_t) >= 2:
            self.fps = (len(self._frame_t) - 1) / max(
                self._frame_t[-1] - self._frame_t[0], 1e-6)
        else:
            self.fps = 0.0

        # 2) Is this frame 'valid'? (bbox_to_redis._evaluate_frame) Criteria AREA gate in WAIT, low coverage
        # threshold in ENGAGED.
        frame_fresh = bbox_age_s <= self.frame_stale_s()
        self.gate_area = bool(frame_fresh and area_px2 is not None
                              and area_px2 >= self.area_threshold_px2())
        if engaged:
            valid_value = bool(frame_fresh and (coverage_pct is None
                                          or coverage_pct >= self.stay_coverage_pct))
        else:
            valid_value = self.gate_area

        # 3) PENCEREYI ADVANCE.
        if self.window_s_value is not None:
            # DURATION MODE (option): one sample per cycle, older samples drop over time.
            self._window.append((t, valid_value))
            while self._window and t - self._window[0][0] > self.window_s_value:
                self._window.popleft()
        else:
            # SQUARE MODE (DEFAULT): advances slot by slot. Missed frames are reconstructed from the measured
            # period (see class docstring).
            period_value = self.frame_period()
            if self._last_slot_t is None:
                if new_frame:
                    self._window.append((t, valid_value))
                    self._last_slot_t = t
            else:
                # Missed slots: written as invalid frames. The beginning of the loop is filled to the maximum size of
                # the window (so that it does not enter an infinite loop after a long pause).
                populate = 0
                while (t - self._last_slot_t) > 1.5 * period_value and \
                        populate < self.window_frame:
                    self._last_slot_t += period_value
                    self._window.append((self._last_slot_t, False))
                    populate += 1
                if populate >= self.window_frame:
                    self._last_slot_t = t - period_value
                if new_frame:
                    self._window.append((t, valid_value))
                    self._last_slot_t = t
        self.sample_value = len(self._window)
        self.valid_count_value = sum(1 for _, g in self._window if g)
        self.ratio_value = (self.valid_count_value / self.sample_value) if self.sample_value else 0.0

        # 4) SIMPLE RULE counter: consecutive VALID DETECT frame (area/range gate NOT SEARCHED -- same as
        # bbox_to_redis.consecutive_valid).
        if not frame_fresh:
            self.consecutive_frame = 0
            self.consecutive_large_frame = 0
        elif new_frame:
            self.consecutive_frame += 1
            if self.gate_area:
                self.consecutive_large_frame += 1
            else:
                self.consecutive_large_frame = 0

        # 5) RANGE GATE. If the range is absent/stale, it is SKIPPED (bbox_to_redis: "If handoff_state is
        # absent/stale, the door is skipped -- legacy behavior"). FROZEN range is also considered "stale":
        # making the door decision with an old measurement is no more dangerous than not asking for the door
        # at all -- but at least it shows up in the log (range_fresh=0).
        self.gate_range = (range_value is None or not range_fresh
                            or range_value <= self.range_threshold_m)

    # -- kararlar ---------------------------------------------------------

    def window_full(self):
        """bbox_to_redis: 'len(decision_window) < WINDOW_SIZE -> no decision'."""
        if self.window_s_value is None:
            return self.sample_value >= self.window_frame
        # DURATION MODE: both time spread and minimum number of samples required -- if the cycle drops to 2
        # Hz, 3 would be sampled in the 1.5 window of s and 80% would be considered 'filled' with three
        # samples.
        if self.sample_value < 5:
            return False
        return (self._window[-1][0] - self._window[0][0]) >= 0.95 * self.window_s_value

    def engaged_required(self):
        """(yes_mi, reason_text)."""
        if self.large_frame is not None:
            if self.consecutive_large_frame >= self.large_frame:
                return True, (f"large_frame({self.consecutive_large_frame} consecutive, "
                              f"area % {self.area_pct:g} )")
            return False, ''
        if self.simple_frame is not None:
            if self.consecutive_frame >= self.simple_frame:
                return True, f"simple ({self.consecutive_frame} sequential frame)"
            return False, ''
        if not self.window_full():
            return False, ''
        if self.window_s_value is None:
            if self.valid_count_value < self.valid_threshold:
                return False, ''
            window_text = (f"{self.valid_count_value}/{self.window_frame} square")
        else:
            if self.ratio_value < self.valid_ratio:
                return False, ''
            window_text = (f"rate {self.ratio_value:.2f} @ {self.window_s_value:.1f} s")
        if not self.gate_range:
            return False, ''
        return True, (f"three_gate ({window_text}, area {self.area_pct:g}, "
                      f"range <= {self.range_threshold_m:.0f} m, "
                      f"fps={self.fps:.1f})")

    def should_release(self, t):
        """Detect LOSS (window + dwell) -> ( yes_mi , reason).

        Return rule of bbox_to_redis: valid_count <= REVERT_THRESHOLD (3) and 2 s dwell. The Loss
        LADDER captures persistent loss; this path captures FLICKER detection (2.5 s never disappear
        without interruption but only appears one frame per second)."""
        if self.window_s_value is None:
            disconnected = self.window_full() and self.valid_count_value <= self.RELEASE_FRAME
            text_value = f"{self.valid_count_value}/{self.window_frame} square"
        else:
            disconnected = self.window_full() and self.ratio_value <= (
                float(self.RELEASE_FRAME) / self.window_frame)
            text_value = f"rate {self.ratio_value:.2f}"
        if not disconnected:
            self._release_dwell_t0 = None
            return False, ''
        if self._release_dwell_t0 is None:
            self._release_dwell_t0 = t
            return False, ''
        if t - self._release_dwell_t0 >= self.release_dwell_s:
            self._release_dwell_t0 = None
            return True, f"detection loss ( {text_value} + {self.release_dwell_s:.0f} s dwell)"
        return False, ''

    def reset_value(self):
        """Clear window on status change.

        WHY: validity criterion changes in ENGAGED (domain -> coverage). Making a new decision with
        a window filled with old criteria would be wrong for the same reason that bbox_to_redis does
        decision_window.clear() at handoff."""
        self._window.clear()
        self._last_slot_t = None
        self.consecutive_frame = 0
        self.consecutive_large_frame = 0
        self._release_dwell_t0 = None
        self.ratio_value = 0.0
        self.sample_value = 0
        self.valid_count_value = 0


# --------------------------------------------------------------- fake MAVLink tip
#
# DESK TEST (--mock-mavlink): let the entire chain run WITHOUT vehicle, SITL and network. SAME
# surface as SpeedCommander; The loop code doesn't know which one it's running with.


class _MockReader:
    def __init__(self):
        self.t0 = time.monotonic()

    def get(self):
        # 30 is at an altitude of m, 12 m/s is an imitation of a vehicle heading north.
        t = time.monotonic() - self.t0
        return (np.array([12.0 * t, 0.0, -30.0]), np.array([12.0, 0.0, 0.0]))

    def get_attitude(self):
        return (0.0, math.radians(-3.0), 0.0)      # roll, pitch, yaw [rad]

    def get_vibration(self):
        return (1.0, 1.0, 1.0)

    def get_heartbeat(self):
        return ('GUIDED', 4, 0.1)                  # (mod, mode_no, age_s_value)


class MockCommander:
    """Dry-run substitute for SpeedCommander: sends no MAVLink commands and counts calls."""

    def __init__(self):
        self.conn = None
        self.reader = _MockReader()
        self.sent_value = 0

    def speed_send(self, vel_ned, yaw_rate_rad=None):
        self.sent_value += 1


# ------------------------------------------------------------------- yardimci

def _raw_stab(raw_bbox):
    """RAW 'tracker_bbox' [x,y,w,h,coverage,current,t_capture] -> stab format.

    BACKUP ROAD. The stabilized channel (tracker_bbox_stab) is the output of the virtual gimbal; If
    it is not coming (gimbal layer is off / old bbox_to_redis), the angular error is DERIVED from
    the raw pixel center so that the guidance is not completely blind:
        ex = atan((cx - W/2) / fx),  ey = atan((cy - H/2) / fx)
    WARNING: this value is NOT free from BODY Oscillation (roll/pitch leaks directly to ex/ey). Its
    only acceptable use is if it is better than "no data"; If it is seen while running, it means
    there is a malfunction in the gimbal chain."""
    if raw_bbox is None or len(raw_bbox) < 4:
        return None
    x, y, w, h = (float(raw_bbox[0]), float(raw_bbox[1]),
                  float(raw_bbox[2]), float(raw_bbox[3]))
    cx, cy = x + w / 2.0, y + h / 2.0
    ex = math.degrees(math.atan((cx - _FRAMING_W / 2.0) / _FRAMING_FX))
    ey = math.degrees(math.atan((cy - _FRAMING_H / 2.0) / _FRAMING_FX))
    t_cap = float(raw_bbox[6]) if len(raw_bbox) > 6 else None
    return [cx, cy, w, h, ex, ey, t_cap, None]


def _n(v, b='{:.2f}'):
    """None-safe number formatter (same as in visual_base)."""
    return '' if v is None else b.format(v)


def _mavutil():
    """Imports pymavlink.mavutil LATE.

    WHY: desk test (--mock-mavlink + fake Redis) so that it can run on a machine without pymavlink
    installed."""
    from pymavlink import mavutil
    return mavutil


# ------------------------------------------------------------------------------- main dong

class SingleNodeGuidance:
    """Self-determined, single-process video guidance loop."""

    label_item = 'single_node'

    def __init__(self, controller, gate_value=None, loop_hz=20.0,
                 range_source='estimator', statustext=True, dry=False,
                 mock_mavlink=False, authority_write=True, tau_s=0.35,
                 bbox_stale_s=0.7, gap_hold_s=1.0, release_s=2.5,
                 cooldown_s=3.0, altitude_floor_m=15.0, yaw_tau_s=0.15,
                 yaw_slew_dps2=120.0, log_path=None, pursuer=None, target=None):
        self.k = controller
        self.gate_value = gate_value or EngagementGate()
        self.loop_dt = 1.0 / float(loop_hz)
        self.range_source_name = range_source
        self.statustext = bool(statustext)
        self.dry = bool(dry)
        self.mock_mavlink = bool(mock_mavlink)
        # WRITE AUTHORITY: this node is NOT waiting for authorization, but the position-guidance process
        # (simple_guided_follow) is still looking at port 'command_authority'. To prevent two processes from
        # writing commands to the same tool, we write 'display' when engaging and 'located' when releasing.
        # This is NOT a REQUEST FOR PERMISSION, it is an ADVERTISEMENT: we write and continue, we do not wait
        # for an answer. If the position-guidance process does not run at all, disable the write with --no-authority-write.
        self.authority_write = bool(authority_write)
        self.tau = float(tau_s)
        self.bbox_stale_s = float(bbox_stale_s)
        self.gap_hold_s = float(gap_hold_s)
        self.release_s = float(release_s)
        self.cooldown_s = float(cooldown_s)
        self.altitude_floor_m = float(altitude_floor_m)
        self.yaw_tau_s = float(yaw_tau_s)
        self.yaw_slew_dps2 = float(yaw_slew_dps2)
        self.speed_ceiling = float(getattr(cfg, 'VISUAL_MAX_SPEED_MPS', 18.0))
        # ATTENTION: ports of locator are NOT (14652/14603) -- two processes cannot connect the same udpin
        # port (same warning on visual_base).
        self.pursuer = pursuer or getattr(cfg, 'VISUAL_PURSUER_CONN_STR',
                                          'udpin:127.0.0.1:14654')
        self.target = target or getattr(cfg, 'VISUAL_TARGET_CONN_STR',
                                        'udpin:127.0.0.1:14604')
        stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = log_path or str(
            Path(__file__).resolve().parent / 'logs'
            / f"single_node_{self.k.label_item}_{stamp_value}.csv")
        self._running_value = True
        self.r = None
        self.state_value = WAIT
        self._state_reason = 'start_value3'
        self._last_state_broadcast = 0.0
        self._broadcast_fps = 0.0        # FrameCounter measurement (LOG + status posting)

    # -- setup ----------------------------------------------------------

    def _connect(self):
        print("[tekdugum] Redis'e baglaniliyor...")
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.r.ping()
        self.bbox = BboxReader(self.r)
        self.bbox.start()
        self.counter_value = FrameCounter(self.r)     # LOG-ONLY broadcast rate measurement
        self.counter_value.start()

        if self.mock_mavlink:
            # Table test: no tools. Dry mode is MANDATORY (we don't have any three to send anyway) -- we write it
            # clearly so it won't be mistaken for "real run".
            self.dry = True
            self.commander = MockCommander()
            print("[single-node] *** FAKE MAVLINK: NO vehicle link, "
                  "commands are just logging ***")
            self.range_provider = RangeHolder(MockRange())
            return

        print(f"[tekdugum] hunter connection: {self.pursuer}")
        self.commander = SpeedCommander(self.pursuer)

        if self.range_source_name == 'redis':
            # NO need for Home: range comes as a ready number.
            self.range_provider = RangeHolder(RedisRange(self.r))
            print("[onenode] range source: Redis 'handoff_state'.range_m")
            return

        # Get Home FROM HUNTER (to change the target's global position to NED). Three tries: when the stack is
        # fast restarted the first request may race and go unanswered (measured on visual_base,
        # 2026-08-04).
        msg = None
        for trial in range(1, 4):
            self.commander.conn.mav.command_long_send(
                self.commander.conn.target_system,
                self.commander.conn.target_component,
                _mavutil().mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
            msg = self.commander.conn.recv_match(type='HOME_POSITION',
                                               blocking=True, timeout=10.0)
            if msg is not None:
                break
            print(f"[single knot] HOME_POSITION missed (trial {trial}/3)")
        if msg is None:
            raise SystemExit("HOME_POSITION could not be received (3 trial)")
        home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
        print(f"[tekdugum] home={home[0]:.7f},{home[1]:.7f} "
              f"sub={home[2]:.1f} target: {self.target}")
        self.range_provider = RangeHolder(EstimatorRange(self.target, home))

    # -- Redis ilanlari ---------------------------------------------------

    def _alive_notify(self):
        """DEAD-ADAM ANAHTARI (TTL'li kalp atisi).

        WHY: In 2026-08-05, while the video controller remained suspended, authority was delegated
        to it and NO ONE commanded the vehicle for 5.9 s (range 40 -> 124 m). This node does not ask
        for authority, but the same question should be able to be asked externally: 'is my node
        alive?'. The key is written with TTL -> it disappears automatically if the process dies. CSV
        OTHERWISE THE PROCESS NEVER STARTED."""
        if self.r is None:
            return
        try:
            self.r.set(KEY_ALIVE, json.dumps({
                'label_item': self.label_item, 'method_value2': self.k.label_item, 'pid': os.getpid(),
                't_mono': time.monotonic(), 't_unix': time.time(),
                'state_value': self.state_value}), ex=ALIVE_TTL_S)
        except Exception:
            pass

    def _state_publish(self, range_value=None, force_value=False):
        """'single_node_state': SOLE AUTHORITY source of engagement status.

        TTL NONE (intentionally): a late connected reader (formation_KILLER panel, ground station)
        should INSTANTLY see the latest status. Staleness is distinguished by t_mono/t_unix; The
        answer to the question 'Is the process alive?' is in a separate key.
        (single_node_alive, TTL'li)."""
        if self.r is None:
            return
        now_value = time.monotonic()
        if not force_value and now_value - self._last_state_broadcast < 0.5:
            return
        self._last_state_broadcast = now_value
        try:
            self.r.set(KEY_STATE, json.dumps({
                'state_value': self.state_value,
                'reason_value': self._state_reason,
                'method_value2': self.k.label_item,
                'kural': self.gate_value.mode_value,
                'pid': os.getpid(),
                't_mono': now_value,
                't_unix': time.time(),
                'range_m_value': None if range_value is None else float(range_value),
                'detection_fps': round(self._broadcast_fps, 2),
                'observation_fps': round(self.gate_value.fps, 2),
                'valid_ratio': round(self.gate_value.ratio_value, 3),
                'dry': bool(self.dry),
            }))
        except Exception:
            pass

    def _authority_announce(self, value_value):
        """'command_authority' -> ADVERTISEMENT (see __init__ authority_write note).

        WARNING: The bbox_to_redis decision maker also writes this key (only in mode CHANGE). If two
        run at the same time, the last one to write wins. In a single node installation,
        bbox_to_redis should remain in the DETECT ONLY role."""
        if not self.authority_write or self.r is None:
            return
        try:
            self.r.set(KEY_AUTHORITY, value_value)
        except Exception:
            pass

    def _statustext(self, text_value):
        """MAVLink STATUSTEXT notification (logged by formation_KILLER).

        Not shipped in dry mode and --no-statustext; The screen is pressed again -- 'what would I
        announce' should be visible in the table test."""
        print(f"[tekdugum] >>> {text_value}")
        if not self.statustext or self.dry or self.commander.conn is None:
            return
        try:
            self.commander.conn.mav.statustext_send(
                _mavutil().mavlink.MAV_SEVERITY_INFO,
                text_value.encode('ascii', 'replace')[:50])
        except Exception as exc:
            print(f"[tekdugum] STATUSTEXT gonderilemedi: {exc}")

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
        columns_value = list(LOG_COLUMNS) + EK_COLUMNS
        log.writerow(columns_value)
        event_path = self.log_path.replace('.csv', '_event.csv')
        if event_path == self.log_path:
            event_path = self.log_path + '.event_value.csv'
        event_f = open(event_path, 'w', newline='')
        event_w = csv.writer(event_f)
        event_w.writerow(['t', 't_unix', 'event_value', 'range_m_value', 'detail'])
        tracker_value = EventTracker()
        derivative_value = VelocityDifferentiator(max_history=5)
        acc_lpf = np.zeros(3)
        ACC_TAU = 0.20
        print(f"[tekdugum] log: {self.log_path}")
        print(f"[tekdugum] event log: {event_path}")
        if self.gate_value.large_frame:
            print(f"[single knot] MY OWN DECISION -- LARGE_FRAME: repeatedly "
                  f"{self.gate_value.large_frame} detect + area "
                  f"%{self.gate_value.area_pct:g} (center/range not searched)")
        elif self.gate_value.simple_frame:
            print(f"[single knot] MY OWN DECISION -- SIMPLE rule: consecutively "
                  f"{self.gate_value.simple_frame} current frame "
                  f"(area/range/window doors SKIPPED)")
        else:
            window_value = (f"{self.gate_value.valid_threshold}/{self.gate_value.window_frame} square"
                       if self.gate_value.window_s_value is None else
                       f"%{100 * self.gate_value.valid_ratio:.0f} @ "
                       f"{self.gate_value.window_s_value:.1f} s")
            print(f"[teknode] MY OWN DECISION -- THREE DOORS: {window_value} + area %"
                  f"{self.gate_value.area_pct:g} + range <= "
                  f"{self.gate_value.range_threshold_m:.0f} m "
                  f"('command_authority' NOT EXPECTED)")
        if self.dry:
            print("[tekdugum] *** DRY RUN ( --dry-run ): commands to MAVLink "
                  "IT'S NOT GOING, IT'S LONGING ONLY ***")

        def _event_write(t_m, t_u, events_value, range_value):
            for label_item, detail in events_value:
                event_w.writerow([f"{t_m:.4f}", f"{t_u:.3f}", label_item,
                                 '' if range_value is None else f"{range_value:.2f}",
                                 detail])
            if events_value:
                event_f.flush()

        start_value3 = time.monotonic()
        lpf_vel = np.zeros(3)
        lpf_yaw = 0.0
        previous_t = None
        last_request = None
        last_release_t = -1e9
        loss_t0 = None              # start of uninterrupted detection loss
        dt_window = deque(maxlen=max(5, int(round(2.0 / self.loop_dt))))
        slow_streak = 0
        last_slow_warning = 0.0
        previous_ap_mode = None
        row_counter = 0
        self._state_publish(force_value=True)

        while self._running_value:
            now_value = time.monotonic()
            if duration_s_value is not None and now_value - start_value3 > duration_s_value:
                break
            self._alive_notify()

            # ---measured dt (nominal UNRELIABLE; loop falling into 2 Hz + constant dt assumption was root of
            # giant circle/dither) ---
            raw_dt = (self.loop_dt if previous_t is None
                      else max(now_value - previous_t, 1e-6))
            dt = (self.loop_dt if previous_t is None
                  else min(max(now_value - previous_t, 0.5 * self.loop_dt), 0.5))
            previous_t = now_value
            dt_window.append(raw_dt)
            dt_excess = 1 if raw_dt > 1.5 * self.loop_dt else 0
            loop_hz_mean = len(dt_window) / max(float(sum(dt_window)), 1e-6)
            slow_streak = slow_streak + 1 if dt_excess else 0
            if slow_streak >= 10 and now_value - last_slow_warning > 5.0:
                last_slow_warning = now_value
                print(f"[singleknot] *** LOOP SLOW: measured "
                      f"{loop_hz_mean:.2f} Hz (requested "
                      f"{1.0 / self.loop_dt:.1f} Hz) -- {slow_streak} "
                      f"consecutive loop. Reduce CPU load ***",
                      file=sys.stderr, flush=True)

            t_unix = time.time()
            stab, bbox_age, coverage_value = self.bbox.last_value()
            raw_bbox, raw_age = self.bbox.raw_value()
            # BACKUP PATH: if the stabilized channel is missing/stale but the raw channel is fresh, derive angular
            # error from the raw pixel (see _raw_stab warning).
            if (stab is None or bbox_age > self.bbox_stale_s) and \
                    raw_bbox is not None and raw_age <= self.bbox_stale_s:
                fallback_value = _raw_stab(raw_bbox)
                if fallback_value is not None:
                    stab, bbox_age = fallback_value, raw_age
            pos, vel = self.commander.reader.get()
            att = self.commander.reader.get_attitude()
            vibe = self.commander.reader.get_vibration()
            range_value = self.range_provider.range_value(pos)
            fresh_value = stab is not None and bbox_age <= self.bbox_stale_s

            # Frame signature: Prevents the SAME frame from being counted twice (camera ~30 Hz, loop 20 Hz). If
            # there is t_capture, that is enough; otherwise the pixel + size quartet is practically unique.
            signature_value = None
            if stab is not None:
                signature_value = (stab[6] if len(stab) > 6 else None,
                        stab[0], stab[1], stab[2], stab[3])
            area_px2 = (float(stab[2]) * float(stab[3])) if stab is not None else None
            if coverage_value is None and stab is not None:
                # Coverage comes from the RAW channel; otherwise derive from width bbox (w/framing_genisligi * 100) --
                # same definition.
                coverage_gate = 100.0 * float(stab[2]) / _FRAMING_W
            else:
                coverage_gate = coverage_value
            self.gate_value.sample_value2(now_value, signature_value, bbox_age, coverage_gate, area_px2,
                              range_value, self.range_provider.fresh_value,
                              engaged=(self.state_value == ENGAGED))

            # --- lost watch ---
            if fresh_value:
                loss_t0 = None
            elif loss_t0 is None:
                loss_t0 = now_value
            loss_s = 0.0 if loss_t0 is None else now_value - loss_t0

            # --- own acceleration (LOG-ONLY) ---
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
                t_capture=(float(stab[6]) if fresh_value and len(stab) > 6
                           and stab[6] is not None else None),
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
                           if raw_bbox is not None
                           and raw_age <= self.bbox_stale_s else None),
                px_raw_cy=(float(raw_bbox[1]) + float(raw_bbox[3]) / 2.0
                           if raw_bbox is not None
                           and raw_age <= self.bbox_stale_s else None),
                acc_ned=acc_lpf.copy() if vel is not None else None,
                vibe_max=None if vibe is None else float(max(vibe)),
            )

            events_value = []
            state_column = 'wait_value'
            yaw_cmd_dps = None
            v_cmd = np.zeros(3)
            clamp_speed = clamp_altitude = clamp_yaw = False

            # ================= ENGAGEMENT STATE MACHINE ==================
            if self.state_value == WAIT:
                cooldown_remaining = self.cooldown_s - (now_value - last_release_t)
                yes, reason_value = self.gate_value.engaged_required()
                if yes and cooldown_remaining <= 0.0:
                    # --- ENGAGED OL: karari OWN verdik ---
                    handoff = self._handoff_read()
                    self.k.seed_value2(handoff)
                    # SEEDING of command LPF: to avoid splashing at the time of engagement (integral seeding). The last
                    # command of the positioner is that, if there is one, or our OWN measured speed -- NOT zero, zero is
                    # the full brake command.
                    if handoff and 'cmd_vel_ned' in handoff:
                        lpf_vel = np.asarray(handoff['cmd_vel_ned'], dtype=float)
                    elif vel is not None:
                        lpf_vel = np.asarray(vel, dtype=float)
                    else:
                        lpf_vel = np.zeros(3)
                    lpf_yaw = 0.0
                    last_request = None
                    previous_t = None
                    self.state_value = ENGAGED
                    self._state_reason = reason_value
                    self.gate_value.reset_value()      # size changed: refresh window
                    self._authority_announce('visual')
                    self._state_publish(range_value, force_value=True)
                    self._statustext(ST_ENGAGED)
                    events_value.append(('ENGAGED', f"{reason_value} range={_n(range_value)} "
                                              f"seed_speed="
                                              f"{np.round(lpf_vel, 2).tolist()}"))

            if self.state_value == ENGAGED:
                release_reason = ''
                if fresh_value:
                    cmd = self.k.command_value(o)
                    if getattr(cmd, 'event_value', ''):
                        events_value.append((cmd.event_value, getattr(cmd, 'event_detail', '')))
                        print(f"[tekdugum] EVENT: {cmd.event_value} "
                              f"{getattr(cmd, 'event_detail', '')}")
                    request_value = np.asarray(cmd.vel_ned, dtype=float).reshape(3)
                    yaw_cmd_dps = cmd.yaw_rate_dps
                    last_request = request_value.copy()
                    state_column = 'fresh_value'
                    if getattr(cmd, 'release_value', False):
                        # WATCH: MPC's own state machine said "pass/miss". WE leave the authority -- NOT typing in the key
                        # 'visual_release' and WAITING FOR A decision maker to APPROVE (the architect is the difference).
                        state_column = 'coast'
                        release_reason = f"MISS: {getattr(cmd, 'release_reason', '')}"
                elif last_request is not None and \
                        bbox_age <= (self.bbox_stale_s + self.gap_hold_s):
                    # SHORT SPACE: KEEP last valid command (except yaw_rate -- blind rotation also makes the target lose
                    # horizontally).
                    request_value = last_request
                    state_column = 'hold_value'
                else:
                    # LONG loss: NOT ZERO, FLOATING. Zero is not "no command", it is the BRAKE command from 18-35 m/s to
                    # full stop; brake -> nose up pitch -> camera looks up -> already the target on the edge comes out
                    # completely (measurement 2026-08-04).
                    request_value = (o.vel_ned.copy() if o.vel_ned is not None
                             else (last_request if last_request is not None
                                   else np.zeros(3)))
                    state_column = 'coast'
                    if loss_s > self.release_s:
                        release_reason = (f"bbox loss {loss_s:.1f} s > "
                                       f"{self.release_s:.1f} s")
                if not release_reason:
                    # Second way to quit: shaky detection (window ratio too high).
                    kop, kop_reason = self.gate_value.should_release(now_value)
                    if kop:
                        release_reason = kop_reason

                # ---- COMMON SAFETY LAYER (moved from visual_base ---- 1) command LPF
                a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
                lpf_vel = lpf_vel + a * (request_value - lpf_vel)
                # 2) speed clamp
                n = float(np.linalg.norm(lpf_vel))
                clamp_speed = n > self.speed_ceiling
                v_cmd = (lpf_vel * (self.speed_ceiling / n) if clamp_speed
                         else lpf_vel.copy())
                # 3) ABSOLUTE ALTITUDE BASE (2026-08-04 crash lesson): method independent last defense. In NED, no
                # Descent command is transmitted when z = -altitude, base: z > -altitude_floor_m.
                clamp_altitude = (pos is not None
                                  and float(pos[2]) > -self.altitude_floor_m
                                  and v_cmd[2] > 0.0)
                if clamp_altitude:
                    v_cmd = v_cmd.copy()
                    v_cmd[2] = 0.0
                # 4) yaw: first slew (acceleration) clamp, then light LPF. Measured (2026-08-04): +-16 dps was
                # fluttering in raw yaw_rate 4 Hz because LPF was on speed channels only.
                if yaw_cmd_dps is not None:
                    limit_value = self.yaw_slew_dps2 * dt
                    outlier_value = float(np.clip(yaw_cmd_dps,
                                           lpf_yaw - limit_value, lpf_yaw + limit_value))
                    clamp_yaw = abs(outlier_value - yaw_cmd_dps) > 1e-6
                    ay = dt / (dt + self.yaw_tau_s) if self.yaw_tau_s > 1e-6 else 1.0
                    lpf_yaw += ay * (outlier_value - lpf_yaw)
                    yaw_cmd_dps = lpf_yaw
                else:
                    lpf_yaw = 0.0
                    yaw_cmd_dps = None

                # ---- COMMAND ---- DRY RUN: It is NOT written to MAVLink, but the command is FULLY calculated and
                # logged -- in the table test, the answer to the question "what would I send" stops at CSV.
                if not self.dry:
                    self.commander.speed_send(
                        v_cmd,
                        None if yaw_cmd_dps is None
                        else math.radians(yaw_cmd_dps))

                # RELEASE AT THE LAST: after the last (float) command has been sent. If we had let go first, the
                # vehicle would have been left without commands in that cycle.
                if release_reason:
                    self._release(release_reason, range_value, events_value)
                    last_release_t = now_value

            # ================= LOG (command path END) ==================== ref_* generation AFTER THE COMMAND:
            # The answer to the question "has the target telemetry leaked to my command" can be read even from the
            # code sequence.
            ref = self.range_provider.source_value.ref_target_state()
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
            edge_px = edge_deg = None
            if o.px_raw_cx is not None:
                edge_px = min(o.px_raw_cx, _FRAMING_W - o.px_raw_cx,
                               o.px_raw_cy, _FRAMING_H - o.px_raw_cy)
                edge_deg = math.degrees(math.atan(edge_px / _FRAMING_FX))
            hb = self.commander.reader.get_heartbeat()
            ap_mode = '' if hb is None else (hb[0] or str(hb[1]))
            hb_age_s = None if hb is None else hb[2]
            # Publication rate, log-only: the Hz produced by bbox_to_redis . Use a separate thread to distinguish
            # it from the loop's observed rate, kapi.fps.
            broadcast_fps = self._broadcast_fps = self.counter_value.fps()

            clamps = {'clamp_speed': clamp_speed,
                          'clamp_altitude': clamp_altitude,
                          'clamp_yaw_slew': clamp_yaw}
            events_value = events_value + tracker_value.track_value(state_column, range_value, geo,
                                              clamps)
            if ap_mode and ap_mode != previous_ap_mode:
                events_value.append(('ap_mode_changed',
                                f"{previous_ap_mode or '-'} -> {ap_mode}"))
                print(f"[tekdugum] OTOPILOT MODE: "
                      f"{previous_ap_mode or '-'} -> {ap_mode}")
                previous_ap_mode = ap_mode
            _event_write(now_value, t_unix, events_value, range_value)
            self._state_publish(range_value)

            # The row is constructed from the DICTIONARY, not from the index: the column list is IMPORTED from
            # visual_base, and if a column is added there it will not slide silently here -- the missing name
            # will remain blank.
            s = {
                't': f"{now_value:.4f}", 't_mono': f"{now_value:.4f}",
                't_unix': f"{t_unix:.3f}", 'dt': f"{dt:.4f}",
                'authority': 'visual' if self.state_value == ENGAGED else 'position_based',
                'state_value': state_column,
                'ex_deg': _n(o.ex_deg, '{:.4f}'), 'ey_deg': _n(o.ey_deg, '{:.4f}'),
                'bbox_w': _n(o.bbox_w, '{:.0f}'), 'bbox_h': _n(o.bbox_h, '{:.0f}'),
                'area_root': _n(o.area_root),
                'area_px2': _n(None if o.bbox_w is None
                               else o.bbox_w * o.bbox_h, '{:.0f}'),
                'coverage_pct': _n(o.coverage_pct, '{:.3f}'),
                'bbox_age_s': (f"{bbox_age:.3f}" if math.isfinite(bbox_age)
                               else ''),
                't_capture': _n(o.t_capture, '{:.4f}'),
                'px_virtual_x': _n(o.px_virtual_x, '{:.1f}'),
                'px_virtual_y': _n(o.px_virtual_y, '{:.1f}'),
                'px_raw_cx': _n(o.px_raw_cx, '{:.1f}'),
                'px_raw_cy': _n(o.px_raw_cy, '{:.1f}'),
                'framing_edge_px': _n(edge_px, '{:.1f}'),
                'framing_edge_deg': _n(edge_deg, '{:.2f}'),
                'raw_age_s': f"{raw_age:.3f}" if math.isfinite(raw_age) else '',
                'tilt_deg': _n(o.tilt_deg, '{:.3f}'),
                'range_m_value': _n(range_value),
                'cmd_vx': f"{v_cmd[0]:.3f}", 'cmd_vy': f"{v_cmd[1]:.3f}",
                'cmd_vz': f"{v_cmd[2]:.3f}",
                'cmd_speed_mps': f"{float(np.linalg.norm(v_cmd)):.3f}",
                'cmd_yaw_rate_dps': _n(yaw_cmd_dps),
                'clamp_speed': int(clamp_speed),
                'clamp_altitude': int(clamp_altitude),
                'clamp_yaw_slew': int(clamp_yaw),
                'roll_deg': _n(None if att is None else math.degrees(att[0])),
                'pitch_deg': _n(None if att is None else math.degrees(att[1])),
                'yaw_deg': _n(None if att is None else math.degrees(att[2])),
                'route_deg': _n(own_route, '{:.1f}'),
                'vibe_max': '' if vibe is None else f"{max(vibe):.1f}",
                'ref_target_speed_mps': _n(h_speed),
                'ref_target_route_deg': _n(h_route, '{:.1f}'),
                'ref_target_turn_dps': _n(ref['turn_dps'], '{:.2f}'),
                'ref_range_ground_truth_m': _n(geo['range_m_value']),
                'ref_bearing_deg': _n(geo['bearing_deg'], '{:.1f}'),
                'ref_elevation_deg': _n(geo['elevation_deg'], '{:.2f}'),
                'ref_approximation_angle_deg': _n(geo['approximation_deg'], '{:.1f}'),
                'ref_encounter_type': geo['type_value'],
                'ref_closure_rate_mps': _n(geo['closure_mps']),
                'ref_tgo_s': _n(geo['tgo_s']),
                'ref_cpa_m': _n(geo['cpa_m']), 'ref_cpa_s': _n(geo['cpa_s']),
                'event_value': '|'.join(label_item for label_item, _ in events_value),
                'loop_hz_mean': f"{loop_hz_mean:.2f}", 'dt_excess': dt_excess,
                # 'alive_ttl': heartbeat TTL. visual_base would ask and sample Redis every 10 cycle; Here we
                # write the key EVERY cycle, so asking the constant TTL again does not add information -- we report
                # the value we wrote.
                'alive_ttl': ALIVE_TTL_S,
                'ap_mode': ap_mode, 'hb_age_s': _n(hb_age_s, '{:.2f}'),
                # --- single button specific ---
                'single_node_state': self.state_value,
                'kural': self.gate_value.mode_value,
                'detection_fps': f"{broadcast_fps:.2f}",
                'observation_fps': f"{self.gate_value.fps:.2f}",
                'window_sample': self.gate_value.sample_value,
                'valid_frame': self.gate_value.valid_count_value,
                'valid_ratio': f"{self.gate_value.ratio_value:.3f}",
                'gate_area': int(self.gate_value.gate_area),
                'gate_range': int(self.gate_value.gate_range),
                'consecutive_frame': self.gate_value.consecutive_frame,
                'consecutive_large_frame': self.gate_value.consecutive_large_frame,
                'loss_s': f"{loss_s:.2f}",
                'range_source': self.range_provider.source_value.label_item,
                'range_fresh': int(self.range_provider.fresh_value),
                'range_age_s': ('' if not math.isfinite(self.range_provider.age_s_value)
                                 else f"{self.range_provider.age_s_value:.2f}"),
                'dry': int(self.dry),
            }
            if pos is not None:
                s.update({'pos_x': f"{pos[0]:.2f}", 'pos_y': f"{pos[1]:.2f}",
                          'pos_z': f"{pos[2]:.2f}",
                          'altitude_m': f"{-float(pos[2]):.2f}"})
            if vel is not None:
                s.update({'vel_x': f"{vel[0]:.2f}", 'vel_y': f"{vel[1]:.2f}",
                          'vel_z': f"{vel[2]:.2f}", 'speed_mps': f"{own_speed:.2f}"})
            if o.acc_ned is not None:
                s.update({'acc_x_mps2': f"{o.acc_ned[0]:.2f}",
                          'acc_y_mps2': f"{o.acc_ned[1]:.2f}",
                          'acc_z_mps2': f"{o.acc_ned[2]:.2f}"})
            if ref['pos'] is not None:
                s.update({'ref_target_x': f"{ref['pos'][0]:.2f}",
                          'ref_target_y': f"{ref['pos'][1]:.2f}",
                          'ref_target_z': f"{ref['pos'][2]:.2f}"})
            if h_vel is not None:
                s.update({'ref_target_vx': f"{h_vel[0]:.2f}",
                          'ref_target_vy': f"{h_vel[1]:.2f}",
                          'ref_target_vz': f"{h_vel[2]:.2f}"})
            if ref['acc'] is not None:
                s.update({'ref_target_ax_mps2': f"{ref['acc'][0]:.2f}",
                          'ref_target_ay_mps2': f"{ref['acc'][1]:.2f}",
                          'ref_target_az_mps2': f"{ref['acc'][2]:.2f}"})
            log.writerow([s.get(label_item, '') for label_item in columns_value])
            # PERIODIC FLUSH: in a crash/SIGKILL, the last seconds are not lost before being written to disk ( 20
            # once per line ~= 1 Hz ).
            row_counter += 1
            if row_counter % 20 == 0:
                log_f.flush()

            remaining_value = self.loop_dt - (time.monotonic() - now_value)
            if remaining_value > 0:
                time.sleep(remaining_value)

        # --- closing: we leave it like that (no dying quietly) ---
        if self.state_value == ENGAGED:
            self._release('process is closing', None, [])
            self._alive_notify()      # keep 'status' up to date on heartbeat
        log_f.close()
        event_f.close()
        print(f"[tekdugum] loop is over, log is closed: {self.log_path}")

    # -- state transition -----------------------------------------------------

    def _release(self, reason_value, range_value, events_value):
        """ENGAGED -> WAIT. It does NOT interrupt the command: the last glide command has already been sent;
There is only announcement and status reset here."""
        self.state_value = WAIT
        self._state_reason = reason_value
        self.gate_value.reset_value()
        self._authority_announce('position_based')
        self._state_publish(range_value, force_value=True)
        self._statustext(ST_RELEASE)
        events_value.append(('RELEASE', reason_value))
        print(f"[tekdugum] <<< RELEASE: {reason_value}")

    def _handoff_read(self):
        """'handoff_state' (last command of positioner) -- warm-start seed, IF EXISTS.

        Otherwise, no problem: the seed becomes our own measured speed. This does NOT depend on the
        existence of a node-located process."""
        try:
            raw_value = self.r.get(KEY_HANDOFF)
            return json.loads(raw_value) if raw_value else None
        except Exception:
            return None


# ------------------------------------------------------------------ CLI

def main(argv=None):
    p = argparse.ArgumentParser(
        description="Single-node, self-determining video guidance")
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--duration-value', type=float, default=None,
                   help='running time [s] (default: unlimited)')
    p.add_argument('--range-source', choices=['estimator', 'redis'],
                   default=os.environ.get('YILDIZ_SINGLE_NODE_RANGE', 'estimator'),
                   help="estimator: own IMM (default) | "
                        "redis: 'handoff_state'.range_m")
    # --- engagement door ---
    p.add_argument('--window-frame', type=int, default=None,
                   help='stability window [FRAME] (default '
                        '$YILDIZ_WINDOW_FRAME or 25; 30 fps -> 0.83 s, '
                        '20 Hz -> 1.25 s)')
    p.add_argument('--valid-ratio', type=float, default=None,
                   help='minimum valid frame rate in window (default '
                        '$YILDIZ_WINDOW_RATIO or 0.8 -> 20/25)')
    p.add_argument('--window-s-value', type=float, default=None,
                   help='OPTIONAL: display the window in DURATION instead of SQUARE '
                        'establish [s]. If not given, the square window is used.')
    p.add_argument('--simple-frame', type=int,
                   default=(int(os.environ['YILDIZ_SINGLE_NODE_FRAME'])
                            if os.environ.get('YILDIZ_SINGLE_NODE_FRAME') else None),
                   help='SIMPLE rule: N consecutive valid frames is enough '
                        '(three doors are SKIPPED). If not given, three door rule.')
    p.add_argument('--large-frame', type=int, default=None,
                   help='Short gate for LOS: consecutive N fresh frames AT THE SAME TIME '
                        'Let it pass the threshold of --area-pct; center and range are not sought')
    p.add_argument('--area-pct', type=float, default=None,
                   help='area gate: rectangle of the frame (p %% x p %% ) '
                        '(default $YILDIZ_TRANSITION_AREA_PCT or 2)')
    p.add_argument('--transition-range', type=float, default=None,
                   help='range gate [m] (default $YILDIZ_TRANSITION_RANGE '
                        'or 60); If there is no range/stale, the door is skipped')
    p.add_argument('--release-s', type=float, default=2.5,
                   help='RELEASE after uninterrupted loss of bbox exceeds this period')
    p.add_argument('--cooldown-s', type=float, default=3.0,
                   help='Minimum wait for re-engagement after QUIT '
                        '(ping-pong engeli)')
    # --- posting / running style ---
    p.add_argument('--no-statustext', dest='statustext', action='store_false',
                   help='MAVLink Turn off STATUSTEXT announcements')
    p.add_argument('--no-authority-write', dest='authority_write', action='store_false',
                   help="Close posting 'command_authority' (if there is no positioned process)")
    p.add_argument('--dry-run', dest='dry', action='store_true',
                   help="Run the entire chain WITHOUT WRITTING to MAVLink (commands "
                        'logged only) -- pre-hardware table testing')
    p.add_argument('--mock-mavlink', action='store_true',
                   help='run WITHOUT vehicle/SITL: dummy state + dummy range '
                        '(--dry-run is enabled automatically)')
    p.add_argument('--log', default=None, help='main CSV path')
    p.add_argument('--diagnostic-log', default=None, help='MPC diagnostic CSV path')
    # --- homing law ---
    p.add_argument('--guidance-value', choices=['los', 'mpc'], default='los',
                   help='display single law (default: los)')
    # --- MPC relays (--guidance-value mpc only) ---
    p.add_argument('--horizon', type=int, default=None)
    p.add_argument('--step-s', type=float, default=None)
    p.add_argument('--mount', type=float, default=None,
                   help='camera mounting angle [deg, UP +]')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw COMMAND (yaw remains on autopilot)')
    p.add_argument('--speed-ceiling', type=float, default=None)
    p.add_argument('--no-miss', action='store_true',
                   help='MPC Turn off ISA state machine')
    # --- LOS/PN settings (--guidance-value los only) ---
    p.add_argument('--n-pn', type=float, default=4.0)
    p.add_argument('--strike-acceleration', type=float, default=4.0)
    p.add_argument('--command-horizon', type=float, default=0.70)
    p.add_argument('--terminal-range', type=float, default=3.0)
    p.add_argument('--terminal-tgo', type=float, default=0.25)
    a = p.parse_args(argv)

    if a.simple_frame and a.large_frame:
        p.error('--simple-frame and --large-frame cannot be used together')

    stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
    diagnostic = a.diagnostic_log or str(Path(__file__).resolve().parent / 'logs'
                             / f"single_node_mpc_diagnostic_{stamp_value}.csv")
    if a.guidance_value == 'los':
        controller = TerminalLosController(
            n_pn=a.n_pn, strike_acceleration_mps2=a.strike_acceleration,
            command_horizon_s=a.command_horizon,
            terminal_range_m=a.terminal_range,
            terminal_tgo_s=a.terminal_tgo,
            yaw_command_provide=not a.no_yaw)
        print(f"[single node] homing law: terminal LOS/PN (single law) | "
              f"N={a.n_pn:g} strike_acceleration={a.strike_acceleration:g} "
              f"horizon={a.command_horizon:.2f}s "
              f"yaw={'otopilotta' if a.no_yaw else 'enabled_value'}")
    else:
        # MPC is only loaded when explicitly requested. The default LOS installation does not have to move
        # optimization dependencies to hardware.
        from mpc_guidance import MpcConfig, MpcController, _block_generate
        config_value = MpcConfig()
        if a.horizon is not None:
            config_value = MpcConfig(n_step=a.horizon, blocks=_block_generate(a.horizon))
        if a.step_s is not None:
            config_value.step_s = a.step_s
        if a.mount is not None:
            config_value.mount_pitch_deg = a.mount
        if a.aim is not None:
            config_value.aim_deg = a.aim
        if a.no_yaw:
            config_value.yaw_command_provide = False
        if a.speed_ceiling is not None:
            config_value.speed_ceiling_mps = a.speed_ceiling
        if a.no_miss:
            config_value.miss_mode = False
        controller = MpcController(config_value, diagnostic_log=diagnostic)
        print(f"[single knot] guidance law: mpc_guidance.MpcController (IMPORT, "
              f"NOT copy) | horizon={config_value.n_step * config_value.step_s:.2f} s | "
              f"speed cap={config_value.speed_ceiling_mps:.1f} m/s")
    gate_value = EngagementGate(window_frame=a.window_frame,
                          valid_ratio=a.valid_ratio, window_s_value=a.window_s_value,
                          area_pct=a.area_pct, range_threshold_m=a.transition_range,
                          simple_frame=a.simple_frame, large_frame=a.large_frame)
    loop = SingleNodeGuidance(
        controller, gate_value=gate_value,
        loop_hz=a.loop_hz, range_source=a.range_source,
        statustext=a.statustext, dry=a.dry or a.mock_mavlink,
        mock_mavlink=a.mock_mavlink, authority_write=a.authority_write,
        release_s=a.release_s, cooldown_s=a.cooldown_s, log_path=a.log)
    loop.run_value(a.duration_value)
    # The loop object RETURNS: this is how the dry test harness (table test harness) verifies that the
    # command was actually sent (commander.sent == 0).
    return loop


if __name__ == '__main__':
    main()
