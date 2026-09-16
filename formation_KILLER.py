import os
import re
import sys
import time
import threading
import math
import json
import queue
import logging
import subprocess
import termios
import tty
import select
from logging.handlers import RotatingFileHandler
from enum import Enum
from pymavlink import mavutil
import redis
import cv2
import numpy as np
import config

# guidance_config is read only for LIVE altitude offset (ALT_OFFSET_M). If not found, attack guidance
# operates without offset (0.0 m); The process NEVER crashes.
try:
    import guidance_config as _guidance_cfg
    _GUIDANCE_CFG_IMPORT_ERR = None
except Exception as _exc:  # pragma: no cover - guidance_config
    _guidance_cfg = None
    _GUIDANCE_CFG_IMPORT_ERR = _exc  # logger does not exist yet; _make_alt_offset warns


# ATTENTION: apply_global_speed_limit() WRITES WPNAV_SPEED from these constants to the VEHICLE (before
# takeoff, at every run). So WPNAV_SPEED entered manually from Mission Planner will be SILENTLY
# CRUSHED on the next run. This is why on flight 2026-07-30 it was recorded as WPNAV_SPEED=1700: it
# was NAV_SPEED=17.0. Keep these constants the SAME as the value entered into the tool, or the code
# will undo the pilot's setting.
MAX_SPEED = 25.0   # DO_CHANGE_SPEED ceiling in _set_speed() (do not clip 24)
NAV_SPEED = 24.0   # -> WPNAV_SPEED = 2400 cm/s; Same as the value entered into the vehicle
NAV_WPNAV_ACCEL_CMS = 300.0 # sqrt(WPNAV_ACCEL * distance) i.e. 4 m/s^2 reaches 20m/s alignment in 50 meter

RTL_SPEED = 15
DRONE_IDS            = [1, 3, 4, 5] #[1, 2, 3, 4, 5]   [2, 3, 4, 5, 6]
LEADER_ID            = 3

ENABLE_KILL_STRATEGY = True
SUCCESSION_ORDER = [3, 1, 4, 5] # [3, 2, 5, 1, 4]  [4, 2, 5, 3, 6]

DIAMOND_TOP_GAP_M       = 20.0
DIAMOND_BOTTOM_GAP_M    = 20.0
DIAMOND_LATERAL_STAGGER = 3.0   # right/left vertical scroll
DIAMOND_FORE_AFT_STAGGER= 3.0   # top/bottom forward-backward scrolling

# Mandatory sequence: COAST < POS_LOITER < HB_TIMEOUT
LEADER_COAST_S       = 1.5
LEADER_POS_LOITER_S  = 6.0
LEADER_HB_TIMEOUT_S  = 10.0

assert LEADER_COAST_S < LEADER_POS_LOITER_S < LEADER_HB_TIMEOUT_S, \
    "Thresholds must be in ascending order: COAST < POS_LOITER < HB_TIMEOUT"


MSG_RATE_HZ = {
    "RAW_IMU":      50.0,   # id 27 — primary impact detector (acceleration jump)
    "VIBRATION":    10.0,   # id 241 — confirmatory vibration bounce
    "SYS_STATUS":    5.0,   # id 1 — sensor health + battery
    "SERVO_OUTPUT_RAW": 10.0, # id 36 — engine output saturation (damage self-check)
}
IMPACT_ACCEL_G      = 3.5    # |acceleration| for coup flag threshold [g]
IMPACT_MIN_MS       = 50.0   # should stay above the threshold this long [ms]

ENABLE_STATUSTEXT_LOGGING = True
STATUSTEXT_DEDUP_S = 3.0    # Logging the same text again from the same drone during this period

ATTACKER_PRIORITY   = [5, 1, 4, 3]  # He's obsessed with the war ;) [3, 2, 5, 6]

ATTACK_SPEED_MPS  = 24.0 # to send mavlink do_change_speed command
CRUISE_SPEED_MPS  = 24.0   # Speed ​​to be restored when returning to formation

COMMIT_RANGE_M = 140.0
# --- Attack cancellation/release policy ------------------------------
ATTACK_TIMEOUT_S = 14.0
ATTACK_ABORT_ARM_M = 60.0
ATTACK_ABORT_OPEN_M = 30.0
# CAUTION: ATTACK_ABORT_MAX_M 50 SHOULD NOT BE DONE! The attacker is assigned to a range of ~120-150
# m; 50 Absolute limit of m is canceled at the FIRST TICK. The real "miss" signal is when the range
# turns ON again AFTER approach — this is captured by the ATTACK_ABORT_ARM_M + ATTACK_ABORT_OPEN_M
# duo.
ATTACK_ABORT_MAX_M  = 250.0  # absolute: cancel if target completely escaped [m]
ATTACK_COOLDOWN_S   = 10.0   # reselecting this drone after failed attempt[s]
ATTACK_TERMINAL_M   = 45.0   # switch to short-sighted pure pursuit within this range [m]
ATTACK_MAX_TGT_SPD  = 35.0   # reject target speed estimates above this value [m/s]
ATTACK_VEL_LPF      = 0.35   # low pass filter coefficient for target speed estimation

ATTACK_WPNAV_SPEED_CMS = 2400.0
ATTACK_WPNAV_ACCEL_CMS = 400.0
ATTACK_OVERSHOOT_M = 50.0


ENABLE_ATTACK_EGRESS   = False    # False → old behavior (straight beeline, no deviation)
EGRESS_CLEAR_RADIUS_M  = 4.0    # If there are neighbors within this radius, April is deflected [m]
EGRESS_DEFLECT_GAIN    = 1.0     # outward/upward deviation severity (larger = sharper turn)
EGRESS_CEIL_PAD_M      = 5.0     # stay this far below the ceiling (max_alt) [m]
ENABLE_TARGET_SIDE_SELECTION = True  # F: select attacker from target side drone

# --- Terminal (KILL) phase: speed ram-through — speed WITHOUT TOUCHING WPNAV_ACCEL --- While in
# ATTACK_TERMINAL_M, the PURE SPEED vector is commanded instead of the position target; ArduCopter
# Because the S-curve does not brake to zero at the target, the drone passes THROUGH the target at
# full speed (higher impact speed, faster 'kill').
ENABLE_VELOCITY_TERMINAL = True  # False → position target also in terminal (old behavior)

ENABLE_ATTACK_YAW_LOCK    = False
ATTACK_YAW_UNLOCK_SPEED_MPS  = 8.0
ATTACK_YAW_LOCK_TIMEOUT_S    = 15.0
ATTACK_YAW_RECHASE_ANGLE_DEG = 60.0
ATTACK_YAW_RECHASE_CONFIRM_S = 0.5
# Substitute value to be written when unlocking if WP_YAW_BEHAVIOR cannot be read from the vehicle
# (ArduCopter factory default = 2 / "look ahead").
DEFAULT_WP_YAW_BEHAVIOR      = 2.0

ATTACKER_SLOT_KEY   = "attacker_slot_ned"
ATTACKER_STATE_KEY  = "attacker_state_ned"
LEADER_STATE_KEY    = "leader_state_ned"    # status published for the leader's own guidance


POSITION_ONLY_MASK = 3576
# SET_POSITION_TARGET type_mask — SPEED only (position/acceleration/yaw ignored). bit0..2 (position) +
# bit6..8 (acceleration) + bit10,11 (yaw/yaw_rate) = 7+448+3072 = 3527.
VELOCITY_ONLY_MASK = 3527

# --- Yaw LOCK masks --- A constant head angle is COMMANDED when yaw lock is engaged; for this we
# DROPS the yaw bit (bit10 = 1024) from the mask (bit clear = "use this space"). yaw_rate (bit11 =
# 2048) continues to be ignored — we give the absolute angle, not the rotation speed.
POSITION_YAW_MASK = POSITION_ONLY_MASK - 1024   # 2552 : position + fixed yaw
VELOCITY_YAW_MASK = VELOCITY_ONLY_MASK - 1024   # 2503: speed + constant yaw


def diamond_body_slots(n_followers, offset, top_gap, bottom_gap,
                       lateral_stagger, fore_aft_stagger):
    top    = (-fore_aft_stagger, 0.0,     -top_gap)          # behind + above
    right  = (0.0,               offset,  -lateral_stagger)  # on the right + slightly above
    left   = (0.0,              -offset,   lateral_stagger)  # left + slightly below
    bottom = (fore_aft_stagger,  0.0,      bottom_gap)       # ahead + below
    ladder = [top, right, left, bottom]
    if n_followers >= 4:
        return ladder[:4]
    if n_followers == 3:
        return [top, right, left]
    if n_followers == 2:
        return [(-fore_aft_stagger,  offset * 0.5, -top_gap),
                ( fore_aft_stagger, -offset * 0.5,  bottom_gap)]
    if n_followers == 1:
        return [(-fore_aft_stagger, 0.0, -top_gap)]
    return []


def solve_intercept_time(rel_pos, tgt_vel, atk_speed):
    """
        |rel_pos + tgt_vel * t| = atk_speed * t

        (|Vt|^2 - Sa^2) t^2 + 2 (R . Vt) t + |R|^2 = 0
    """
    vt2 = tgt_vel[0]**2 + tgt_vel[1]**2 + tgt_vel[2]**2
    a = vt2 - atk_speed * atk_speed
    b = 2.0 * (rel_pos[0]*tgt_vel[0] + rel_pos[1]*tgt_vel[1] + rel_pos[2]*tgt_vel[2])
    c = rel_pos[0]**2 + rel_pos[1]**2 + rel_pos[2]**2

    if abs(a) < 1e-6:
        # The equation comes down to first order: target speed == attacker speed.
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t > 1e-3 else None

    disc = b*b - 4.0*a*c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    roots = [(-b + sq) / (2.0*a), (-b - sq) / (2.0*a)]
    positive = [t for t in roots if t > 1e-3]
    return min(positive) if positive else None


def next_leader(succession_order, alive_ids, current_leader, skip_ids=None):
    skip = set(skip_ids or set())
    for did in succession_order:
        if did == current_leader:
            continue
        if did in skip:
            continue
        if did in alive_ids:
            return did
    return None



# Loglama
def setup_logging(log_dir="logs", log_file="swarm.log", level=logging.INFO):
    os.makedirs(log_dir, exist_ok=True)
    logger = logging.getLogger("Swarm")
    logger.setLevel(level)
    logger.propagate = False

    if logger.handlers:
        for h in list(logger.handlers):
            logger.removeHandler(h)

    fmt = logging.Formatter(
        fmt="%(asctime)s | %(levelname)s | %(threadName)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    fh = RotatingFileHandler(
        os.path.join(log_dir, log_file),
        maxBytes=5_000_000,
        backupCount=3,
        encoding="utf-8",
    )
    fh.setLevel(level)
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    return logger


logger = setup_logging()

_EVENTS_LOG_DIR  = "logs"
_EVENTS_LOG_PATH = os.path.join(_EVENTS_LOG_DIR, "events.jsonl")
_events_lock     = threading.Lock()


def log_event(event_type: str, **fields):
    """
    Adds a single-line record JSON to the file logs/events.jsonl. Orn:
    log_event("slot_reassignment", follower_id=3, previous_slot=1, new_slot=2)
    """
    record = {"ts": time.time(), "event": event_type}
    record.update(fields)
    try:
        os.makedirs(_EVENTS_LOG_DIR, exist_ok=True)
        with _events_lock:
            with open(_EVENTS_LOG_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception:
        logger.exception("Error writing event log (JSONL).")


# ======================== LIVE altitude offset (guidance_config.ALT_OFFSET_M)
# =========================
class LiveAltOffset:
    """NUMERIC parameter reader that can be changed DURING flight.

    Reads the value from the line ``<name> = <number>`` in guidance_config.py; ``name`` defaults to
    ``ALT_OFFSET_M`` (command altitude offset: POSITIVE = HIGHER three; calling party subtracts this
    from commanded NED z, z in NED down-positive), but can be used for any numeric parameter -- a
    second example in this file reads FORMATION_OFFSET_M live. NOTE: Sibling copies in
    simple_guided_follow*.py only carry version ALT_OFFSET_M; this copy is their SUPERSET (with
    default arguments the behavior is exactly the same).

    NO reboot: every ``poll_s`` every second it looks at the mtime of the file and only if the file
    has changed re-parses the relevant line with a single regex and writes it back to the module.

    NOT USING ``importlib.reload()`` on purpose: reload would (a) overwhelm any other parameters the
    GUI modifies in memory, and (b) could run a half-saved file at that exact moment — all while the
    vehicle was flying with those numbers. A single-line regex cannot do either of these: a line
    that is corrupt, missing, or captured at the time of saving will only preserve the PREVIOUS
    value.

    Regex looks for a ACTUAL line break (with optional comment) following the number; so a write
    that is captured at the time of saving and clipped to a VALID number -- reading "ALT_OFFSET_M =
    25.0" as "ALT_OFFSET_M = 2" -- is rejected instead of being flown as 2.

    A file whose mtime has changed but cannot be parsed is retried with ``_MAX_REPARSE_TRIES``
    polling (half-write completed in milliseconds), then chased with ``on_error``: due to a config
    whose line is commented out, the file cannot be re-read indefinitely in the control loop.

    An out-of-range value (wrong key) is truncated, not applied: by default +-``max_abs_m``, or
    ``min_m``/``max_m`` If given, within that range (for non-symmetrical quantities such as offset:
    if the formation offset decreases to 0, the drones will overlap). ``slew_mps`` makes a live
    change a ramp instead of a step.

    KNOWN LIMIT: detection is with mtime; A config (cp -p, rsync -t, tar -x) will NOT be seen
    restored with the timestamp preserved. Put `touch` in the file.
    """

    _MAX_REPARSE_TRIES = 3

    @staticmethod
    def _build_pattern(name):
        return re.compile(
            r"^[ \t]*" + re.escape(name) + r"[ \t]*=[ \t]*"
            r"([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)"
            r"[ \t]*(?:#[^\n]*)?\n",
            re.MULTILINE,
        )

    def __init__(self, module=None, path=None, poll_s=0.5, max_abs_m=30.0,
                 slew_mps=0.0, default=0.0, on_change=None, on_error=None,
                 name="ALT_OFFSET_M", min_m=None, max_m=None):
        self._name = str(name)
        self._pattern = self._build_pattern(self._name)
        self._min_m = None if min_m is None else float(min_m)
        self._max_m = None if max_m is None else float(max_m)
        self._module = module
        if path is None and module is not None:
            path = getattr(module, "__file__", None)
        self._path = str(path) if path else None
        self._poll_s = max(0.0, float(poll_s))
        self._max_abs = abs(float(max_abs_m))
        self._slew = max(0.0, float(slew_mps))
        self._on_change = on_change
        self._on_error = on_error
        self._reparse_tries = 0
        self._value = self._clamp(self._read_module(default))
        self._applied = self._value
        # If the module exists, we already have its value; Only the NEXT file changes are of interest. If
        # there is no module (import failed, file-only backup) we have nothing; Leave mtime blank so that the
        # FIRST poll actually parses the file — otherwise the offset would silently remain at 0.0 until
        # someone resaved the config.
        self._mtime = self._stat() if module is not None else None
        self._next_poll = 0.0
        self._last_t = None

    def _clamp(self, v):
        lo = -self._max_abs if self._min_m is None else self._min_m
        hi = self._max_abs if self._max_m is None else self._max_m
        try:
            v = float(v)
        except (TypeError, ValueError):
            # Bad/missing value: in asymmetric range 0.0 MAY NOT be valid (formation offset 0 = drones on top of
            # each other) so drop to base.
            return 0.0 if self._min_m is None else lo
        if v != v:  # NaN
            return 0.0 if self._min_m is None else lo
        return max(lo, min(hi, v))

    def _read_module(self, default):
        if self._module is None:
            return default
        return getattr(self._module, self._name, default)

    def _stat(self):
        if not self._path:
            return None
        try:
            return os.path.getmtime(self._path)
        except OSError:
            return None

    def _poll(self, now):
        if now < self._next_poll:
            return
        self._next_poll = now + self._poll_s
        mtime = self._stat()
        if mtime is not None and mtime != self._mtime:
            parsed = None
            try:
                with open(self._path, "r", encoding="utf-8", errors="replace") as fh:
                    text = fh.read()
            except OSError:
                text = None
            if text is not None:
                match = self._pattern.search(text)
                if match is not None:
                    try:
                        parsed = self._clamp(float(match.group(1)))
                    except ValueError:
                        parsed = None
            if parsed is not None:
                # mtime is processed ONLY after successful parsing. If we captured the file at the time it was saved,
                # mtime remains the same and we read it again at the next poll; otherwise on a mount point with coarse
                # mtime resolution (NFS, some FUSE/shared folders) the completed write could carry the same mtime and
                # the edit would be swallowed without warning.
                self._mtime = mtime
                self._reparse_tries = 0
                if self._module is not None:
                    setattr(self._module, self._name, parsed)
                else:
                    self._file_value = parsed
            else:
                # Could not read/parse. Retry a few polls (half writes complete in milliseconds), then accept this
                # mtime and stop reading: otherwise a config with line ALT_OFFSET_M commented out would have the file
                # read twice per second ad infinitum -- plus a syscall that blocks every read on network mount points,
                # which is the raison d'être of this retry.
                self._reparse_tries += 1
                if self._reparse_tries >= self._MAX_REPARSE_TRIES:
                    self._mtime = mtime
                    self._reparse_tries = 0
                    if self._on_error is not None:
                        try:
                            self._on_error(self._path, self._value)
                        except Exception:
                            pass
        if self._module is not None:
            new = self._clamp(self._read_module(self._value))
        else:
            new = self._clamp(getattr(self, "_file_value", self._value))
        if new != self._value:
            old, self._value = self._value, new
            if self._on_change is not None:
                try:
                    self._on_change(old, new)
                except Exception:
                    pass

    def target(self):
        """Target offset BEFORE the ramp is applied [m]."""
        return self._value

    def reset(self):
        """Fixes the ramp at the current value (when starting a new run/attack)."""
        self._applied = self._value
        self._last_t = None

    def value(self, now=None):
        """Offset to apply now [m]: trimmed, fresh, ramped if necessary."""
        now = time.monotonic() if now is None else float(now)
        self._poll(now)
        if self._slew <= 0.0:
            self._applied = self._value
            self._last_t = now
            return self._applied
        if self._last_t is None:
            self._last_t = now
            self._applied = self._value
            return self._applied
        dt = min(max(now - self._last_t, 0.0), 1.0)
        self._last_t = now
        delta = self._value - self._applied
        step = self._slew * dt
        if abs(delta) <= step:
            self._applied = self._value
        else:
            self._applied += step if delta > 0.0 else -step
        return self._applied


def _guidance_cfg_path(tag):
    """Path of guidance_config.py on disk (backup if the module could not be imported)."""
    if _guidance_cfg is not None:
        return None  # LiveAltOffset gets the path from the module's __file__
    candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "guidance_config.py")
    path = candidate if os.path.exists(candidate) else None
    # SILENCE: silently dropping the live parameter to default is not noticeable in flight; The only
    # symptom is the value in the log.
    logger.error(
        f"[{tag}] guidance_config IMPORT EDILEMEDI ({_GUIDANCE_CFG_IMPORT_ERR!r}). "
        + (f"Using file backup: {path}" if path else
           "File not found either -> live parameter will remain at default! "
           "Put guidance_config.py next to this script.")
    )
    return path


def _make_alt_offset():
    """Installs a single live altitude offset reader for attack guidance."""
    path = _guidance_cfg_path("AltOffset")
    return LiveAltOffset(
        module=_guidance_cfg,
        path=path,
        poll_s=float(getattr(_guidance_cfg, "ALT_OFFSET_POLL_S", 0.5)),
        max_abs_m=float(getattr(_guidance_cfg, "ALT_OFFSET_MAX_ABS_M", 30.0)),
        slew_mps=float(getattr(_guidance_cfg, "ALT_OFFSET_SLEW_MPS", 2.0)),
        default=0.0,
        on_change=lambda old, new: logger.warning(
            f"[AltOffset] ALT_OFFSET_M {old:+.2f} -> {new:+.2f} m "
            f"(live; attack April altitude shifted)"
        ),
        on_error=lambda path, keep: logger.error(
            f"[AltOffset] {path} has changed but is available "
            f"Row 'ALT_OFFSET_M = <number>' not found -- {keep:+.2f} with m "
            f"continues to fly. Correct the line and save again."
        ),
    )


# Command altitude offset of attack homing. simple_guided_follow reads its offset from the SAME file;
# The slot coming from leader_slot_ned is not added AGAIN here (to avoid double counting) — see fig.
# _leader_slot_reader_thread.
ALT_OFFSET = _make_alt_offset()


def _make_formation_offset():
    """Live reader for formation offset (guidance_config.FORMATION_OFFSET_M).

    During the flight, it is enough to change the line FORMATION_OFFSET_M in guidance_config.py and
    save: ~0.5 Within s the formation manager uses the new range. The code is not restarted, the
    formation is not re-established, and slot assignments are not reset.

    Clipping is NOT SYMMETRIC: it is a distance, not an offset. A value that goes down to 0 (or
    negative) puts all followers on the leader -- so the base is FORMATION_OFFSET_MIN_M, and a
    bad/NaN value falls on the BASE, not 0.0. The change is ramped by FORMATION_OFFSET_SLEW_MPS: 10
    -> 25 An adjustment of m would otherwise shift each follower's target by 15 m in one frame.
    """
    path = _guidance_cfg_path("FormOffset")
    lo = float(getattr(_guidance_cfg, "FORMATION_OFFSET_MIN_M", 3.0))
    hi = float(getattr(_guidance_cfg, "FORMATION_OFFSET_MAX_M", 60.0))
    return LiveAltOffset(
        module=_guidance_cfg,
        path=path,
        name="FORMATION_OFFSET_M",
        min_m=lo,
        max_m=hi,
        poll_s=float(getattr(_guidance_cfg, "FORMATION_OFFSET_POLL_S",
                             getattr(_guidance_cfg, "ALT_OFFSET_POLL_S", 0.5))),
        slew_mps=float(getattr(_guidance_cfg, "FORMATION_OFFSET_SLEW_MPS", 1.5)),
        default=float(getattr(_guidance_cfg, "FORMATION_OFFSET_M", 10.0)),
        on_change=lambda old, new: logger.warning(
            f"[FormOffset] FORMATION_OFFSET_M {old:.2f} -> {new:.2f} m "
            f"(live; pattern range will shift with {float(getattr(_guidance_cfg, 'FORMATION_OFFSET_SLEW_MPS', 1.5)):.1f} m/s)"
        ),
        on_error=lambda path, keep: logger.error(
            f"[FormOffset] {path} has changed but is available "
            f"Line 'FORMATION_OFFSET_M = <number>' not found -- {keep:.2f} m "
            f"continues to fly. Correct the line and save again."
        ),
    )


# Formation range (live). The 'offset' argument given to start_formation_following is just the INITIAL
# value; _formation_manager_thread reads from here on every tick.
FORMATION_OFFSET_LIVE = _make_formation_offset()


# =========================
# Formasyon Tipleri
# =========================
class Formation(Enum):
    LINE              = "LINE"
    HORIZONTAL_LINE   = "HORIZONTAL_LINE"
    V_SHAPE           = "V_SHAPE"
    DIAMOND           = "DIAMOND"


# =========================
# SwarmController
# =========================
class SwarmController:
    """
    Unified swarm controller.

    Architecture: - A single packet-outpatcher thread processes all MAVLink message types. - Mod
    tracking has been added to outerpatcher. - Formation logic with yaw-rotation and lookahead. -
    follow_enable Event + pause_formation_following . - parallel_launch. - simple_guided_follow
    after the formation is established. sh is started as subprocess. - End of mission procedure RTL.
    - Logging anywhere.

    Security features: - Heartbeat watchdog: RTL will be triggered if there is no heartbeat from any
    drone within 3s. - Keyboard listener: 'r' key sends RTL to all drones, 'l' key sends LAND to all
    drones. - Leader position guard: if the leader position is not updated for 2s, followers will be
    LOITERed.
    """

    def __init__(
        self,
        leader_id: int,
        connection_port: int,
        drone_ids: list,
        symmetric_horizontal: bool = False,
        ):
        self.drone_ids            = drone_ids
        self.leader_id            = leader_id
        self.symmetric_horizontal = symmetric_horizontal

        self.connection_string = f'udp:127.0.0.1:{connection_port}'
        logger.info(f"Establishing connection: {self.connection_string} | IDs={self.drone_ids}")
        self.master = mavutil.mavlink_connection(self.connection_string, source_system=255)

        # --- Shared status ---
        self.drone_positions  = {}   # sysid -> (x, y, z)  NED
        self.drone_headings   = {}   # sysid -> yaw radyan
        self.drone_velocities = {}   # sysid -> (vx, vy, vz)
        self.drone_modes      = {}   # sysid -> mod string
        self.takeoff_positions= {}   # sysid -> (n, e, d)

        self.lock         = threading.Lock()
        self.stop_threads = threading.Event()
        self.threads      = []

        self.follow_enable = threading.Event()
        self.follow_enable.clear()

        self._slot_assignments = {} # It is used to direct to the nearest slot during formation transitions.
        self._slot_reassign_threshold_m = 5.0  # default is updated at start_formation_following
        # Last calculated pattern targets {fid: (n,e,d)} — broadcasts the status broadcaster so that
        # viz_dashboard can draw the slot target lines.
        self._last_formation_targets = {}

        self._current_formation = None
        self._current_offset    = None

        # Flag for leader position keeper — Followers are put into LOITER when False
        self.leader_pos_ok = threading.Event()
        self.leader_pos_ok.set()   # considered healthy to begin with

        self._hb_queues   = {did: queue.Queue()          for did in self.drone_ids}
        self._gpos_queues = {did: queue.Queue(maxsize=8) for did in self.drone_ids}

        self.drone_armed = {did: False for did in self.drone_ids}
        # Queue to drone press for COMMAND_ACK answers — verifies arm/command result
        self._command_ack_queues = {did: queue.Queue(maxsize=8) for did in self.drone_ids}

        self._nav_queues  = {did: queue.Queue(maxsize=8) for did in self.drone_ids}
        # Queue to drone press for MISSION_ITEM_REACHED — used in mission restart logic
        self._mission_reached_queues = {did: queue.Queue(maxsize=16) for did in self.drone_ids}
        # Tail for drone press for MISSION_CURRENT — follows current WP index
        self._mission_current_queues = {did: queue.Queue(maxsize=4) for did in self.drone_ids}
        # Queue for MISSION_COUNT answers — avoids using recv_match outside of externalpatcher
        self._mission_count_queue = queue.Queue(maxsize=4)
        # Queue for MISSION_ITEM/MISSION_ITEM_INT replies — used to get the last WP coordinates
        self._mission_item_queue = queue.Queue(maxsize=4)

        # simple_guided_follow subprocess handle to terminate during shutdown
        self._follow_proc = None
        # Cached (lat, lon, alt) origin NED set by fetch_and_publish_ned_origin
        self._ned_origin = None

        # It's time for the last heartbeat of the drone press for the heartbeat watchdog
        self._last_heartbeat_time = {did: time.monotonic() for did in self.drone_ids}
        # Last update time of leader position (security feature 3)
        self._last_leader_pos_time = time.monotonic()
        self._alive_ids = set(self.drone_ids)
        self._grounded_ids = set()
        self._on_leader_lost = None
        self._election_lock = threading.Lock()
        self._election_in_progress = False

        # --- NEW: high speed telemetry queues (impact/damage analysis) --
        self._imu_queues    = {did: queue.Queue(maxsize=64) for did in self.drone_ids}
        self._vib_queues    = {did: queue.Queue(maxsize=16) for did in self.drone_ids}
        self._sys_queues    = {did: queue.Queue(maxsize=8)  for did in self.drone_ids}
        self._servo_queues  = {did: queue.Queue(maxsize=16) for did in self.drone_ids}

        # --- Parameter reading auxiliary queue (general purpose for get_param) --- Queue to drone press for
        # PARAM_VALUE responses — get_param waits. NOTE: The speed ceiling can NO LONGER be read from the
        # vehicle; derived from constants (NAV_SPEED / MAX_SPEED) (see apply_global_speed_limit). This queue
        # is kept only for the general purpose get_param helper method.
        self._param_value_queues = {did: queue.Queue(maxsize=32) for did in self.drone_ids}
        # Last calculated |acceleration| (g) and permanent coup flag.
        self._last_accel_g  = {did: 1.0   for did in self.drone_ids}
        self._impact_flag   = {did: False for did in self.drone_ids}
        # Latest drone press SYS_STATUS health bit mask + battery.
        self._sys_health    = {did: None  for did in self.drone_ids}
        # STATUSTEXT noise clipping: sysid -> (last_text, monotonic_time)
        self._last_statustext = {}
        # The last target seen by _kill_check for attacker selection (F) is location NED. The intersection
        # thread also updates on each tick (reads lock monitor yaw).
        self._last_target_ned = None
        # Offensive altitude ceiling (from config/controller.json; set at main).
        self._attacker_max_alt = config.MAX_ALT_M

        # --- NEW: attacker state machine -------------------------------- Which follower is currently
        # assigned to the attack. None = none.
        self._committed_attacker = None
        # Identifier of the (old) second guidance process — no longer used.
        self._attacker_proc = None
        # Drones that must be skipped during leader selection (mid-attack).
        self._attack_skip = set()
        # drone_id -> should not be assigned to attack again by this time (monotonic).
        self._attack_cooldown = {}

        # --- yaw lock status after pass ------------------------------- drone_id -> {"yaw": locked heading angle
        # [rad], "t0": lock timestamp [monotonic]} The lock continues to live AFTER the attacker is released into
        # the formation; so it is managed by a watchdog thread separate from the intersection thread (see
        # _yaw_lock_watchdog).
        self._yaw_locks = {}
        self._yaw_lock_mu = threading.Lock()
        # drone_id -> WP_YAW_BEHAVIOR read BEFORE entering the attack (restore).
        self._wp_yaw_behavior_saved = {}

        self._shutting_down = threading.Event()

        # Start single external patcher thread (MINE architecture)
        dispatch_thread = threading.Thread(
            target=self._packet_dispatcher_thread,
            name="PacketDispatcher",
            daemon=True,
        )
        dispatch_thread.start()
        self.threads.append(dispatch_thread)
        logger.info("Package externalpatcher thread started.")

        self.wait_for_all_heartbeats()

    # ------------------------------------------------------------------ Package externalpatcher - single
    # thread, all message types Mod tracking is also added here
    # ------------------------------------------------------------------
    def _packet_dispatcher_thread(self):
        while not self.stop_threads.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue

            src   = msg.get_srcSystem()
            mtype = msg.get_type()

            # --- HEARTBEAT: queues + mod tracking + watchdog timestamp ---
            if mtype == 'HEARTBEAT' and src in self._hb_queues:
                q = self._hb_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

                # Update last heartbeat time for watchdog
                self._last_heartbeat_time[src] = time.monotonic()

                # Update arm status from HEARTBEAT (for arm verification)
                self.drone_armed[src] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                # Mod tracking — integrated here to avoid a separate thread
                try:
                    mode_name = mavutil.mode_string_v10(msg)
                    with self.lock:
                        if self.drone_modes.get(src) != mode_name:
                            logger.info(f"Mode update -> drone={src}, mode={mode_name}")
                            log_event("mode_change", drone_id=src, mode=mode_name)
                            self.drone_modes[src] = mode_name
                except Exception:
                    pass

            # --- GLOBAL_POSITION_INT: altitude hold + bearing + SHARED NED position/speed ---
            elif mtype == 'GLOBAL_POSITION_INT' and src in self._gpos_queues:
                q = self._gpos_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

                with self.lock:
                    if msg.hdg != 65535:
                        self.drone_headings[src] = math.radians(msg.hdg / 100.0)

                if self._ned_origin is not None:
                    n, e, d = latlon_to_ned(
                        msg.lat / 1e7,
                        msg.lon / 1e7,
                        msg.alt / 1000.0,   # AMSL metre
                        *self._ned_origin,
                    )
                    with self.lock:
                        self.drone_positions[src]  = (n, e, d)
                        # cm/s -> m/s on vx/vy/vz NED axes (axes are common, only the origin is different so no need to
                        # re-reference speeds).
                        self.drone_velocities[src] = (
                            msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
                        )

                    # Update timestamp when leader position is updated (security feature 3)
                    if src == self.leader_id:
                        self._last_leader_pos_time = time.monotonic()

            # FIX: LOCAL_POSITION_NED reception is disabled. Each vehicle reports this message according to its
            # OWN EKF origin; mixing these frames between drones sent followers in the wrong direction.
            # Position/velocity now comes from GLOBAL_POSITION_INT above, at the common origin frame. elif mtype
            # == 'LOCAL_POSITION_NED' and src in self.drone_ids: with self.lock:
            #         self.drone_positions[src]  = (msg.x, msg.y, msg.z)
            #         self.drone_velocities[src] = (msg.vx, msg.vy, msg.vz)
            #     if src == self.leader_id:
            #         self._last_leader_pos_time = time.monotonic()

            # --- NAV_CONTROLLER_OUTPUT: waypoint mesafesi ---
            elif mtype == 'NAV_CONTROLLER_OUTPUT' and src in self._nav_queues:
                q = self._nav_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_ITEM_REACHED: for task restart detection ---
            elif mtype == 'MISSION_ITEM_REACHED' and src in self._mission_reached_queues:
                q = self._mission_reached_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_CURRENT: follows the active waypoint index ---
            elif mtype == 'MISSION_CURRENT' and src in self._mission_current_queues:
                q = self._mission_current_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- COMMAND_ACK: to verify arm/command result ---
            elif mtype == 'COMMAND_ACK' and src in self._command_ack_queues:
                q = self._command_ack_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_COUNT: mission_request_list_send'e cevap ---
            elif mtype == 'MISSION_COUNT':
                q = self._mission_count_queue
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_ITEM_INT: mission_request_int_send'e cevap ---
            elif mtype == 'MISSION_ITEM_INT':
                q = self._mission_item_queue
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- NEW: RAW_IMU — primary impact detector (acceleration jump) ----
            elif mtype == 'RAW_IMU' and src in self._imu_queues:
                # ArduPilot has xacc/yacc/zacc values ​​for RAW_IMU in mg (milli-g).
                try:
                    ax = msg.xacc / 1000.0
                    ay = msg.yacc / 1000.0
                    az = msg.zacc / 1000.0
                    mag_g = math.sqrt(ax*ax + ay*ay + az*az)
                    self._last_accel_g[src] = mag_g
                except Exception:
                    pass
                q = self._imu_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- NEW: VIBRATION — vibration signal confirming impact --------
            elif mtype == 'VIBRATION' and src in self._vib_queues:
                q = self._vib_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- NEW: SYS_STATUS — sensor health + battery ------------------
            elif mtype == 'SYS_STATUS' and src in self._sys_queues:
                try:
                    self._sys_health[src] = {
                        "health": int(msg.onboard_control_sensors_health),
                        "present": int(msg.onboard_control_sensors_present),
                        "enabled": int(msg.onboard_control_sensors_enabled),
                        "batt_v": (msg.voltage_battery / 1000.0
                                   if msg.voltage_battery not in (0, 65535) else None),
                        "batt_a": (msg.current_battery / 100.0
                                   if msg.current_battery not in (0, -1) else None),
                    }
                except Exception:
                    pass
                q = self._sys_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- NEW: SERVO_OUTPUT_RAW — engine saturation (damage check) -------
            elif mtype == 'SERVO_OUTPUT_RAW' and src in self._servo_queues:
                q = self._servo_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- NEW: PARAM_VALUE — get_param response (WPNAV_SPEED/ACCEL) -----
            elif mtype == 'PARAM_VALUE' and src in self._param_value_queues:
                q = self._param_value_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- STATUSTEXT — autonomous mode change / failsafe catch REASON --- Switched on and off with
            # ENABLE_STATUSTEXT_LOGGING. Logs only; It does not touch the flight logic.
            elif (mtype == 'STATUSTEXT' and ENABLE_STATUSTEXT_LOGGING
                  and src in self._hb_queues):
                self._handle_statustext(src, msg)

    def _handle_statustext(self, src, msg):
        """
        Writes STATUSTEXT from the vehicle to swarm.log and events.jsonl (panel event stream). Makes
        visible the reason (GCS/EKF/battery failsafe etc.) when a drone crashes into LAND/RTL by
        itself.

        It is purely ADDITIONAL — it does not delete/change any state. It can be safely closed with
        the flag ENABLE_STATUSTEXT_LOGGING at the top level.
        """
        try:
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode("utf-8", "replace")
            text = (text or "").strip()
            if not text:
                return

            severity = int(getattr(msg, "severity", 6))

            # Noise clipping: repeating the same text from the same drone in a short time.
            now = time.monotonic()
            prev = self._last_statustext.get(src)
            if prev and prev[0] == text and (now - prev[1]) < STATUSTEXT_DEDUP_S:
                return
            self._last_statustext[src] = (text, now)

            # MAVLink severity: 0=emergency … 3=error, 4=warning, 5..7=notice/info/debug
            if severity <= 3:
                logger.error(f"[STATUSTEXT] d{src} sev={severity}: {text}")
            elif severity == 4:
                logger.warning(f"[STATUSTEXT] d{src} sev={severity}: {text}")
            else:
                logger.info(f"[STATUSTEXT] d{src} sev={severity}: {text}")

            log_event("statustext", drone_id=src, severity=severity, text=text)
        except Exception:
            logger.exception("Error processing [STATUSETEXT].")

    # ------------------------------------------------------------------ Security feature 1 — Heartbeat
    # watchdog thread Monitors the heartbeat of each drone. If there is no heartbeat from any drone within
    # 3 seconds, the RTL command is sent to that drone.
    # ------------------------------------------------------------------
    def _heartbeat_watchdog_thread(self, timeout_s: float = 3.0):
        rtl_sent     = set()
        _logged_lost = set()
        logger.info(f"Heartbeat watchdog started (timeout={timeout_s}s)")

        while not self.stop_threads.is_set():
            now = time.monotonic()
            with self.lock:
                alive_now = set()
            for did in self.drone_ids:
                # Drones that are decommissioned on takeoff: will NEVER be put back into the 'alive' cluster, even if
                # they send a heartbeat.
                if did in self._grounded_ids:
                    self._alive_ids.discard(did)
                    continue
                last = self._last_heartbeat_time.get(did, now)
                age  = now - last
                if age > timeout_s:
                    if did not in _logged_lost:
                        logger.warning(
                            f"[Watchdog] Drone {did} heartbeat was cut off before {age:.1f}s!"
                        )
                        log_event("heartbeat_lost", drone_id=did, age_s=round(age, 2))
                        _logged_lost.add(did)
                    if did not in rtl_sent:
                        self.set_mode(did, "RTL")      # as much as possible, once
                        rtl_sent.add(did)
                    # Removes this drone from the current 'alive' cluster.
                    self._alive_ids.discard(did)
                else:
                    alive_now.add(did)
                    self._alive_ids.add(did)
                    if did in _logged_lost:
                        logger.info(f"[Watchdog] Drone {did} heartbeat is back.")
                        log_event("heartbeat_recovered", drone_id=did)
                        _logged_lost.discard(did)
                    rtl_sent.discard(did)


            # Selection uses the SAME threshold (timeout_s) as dead-marking. Thus, in this tick the leader is
            # already removed from _alive_ids -> no early failover.
            ldr = self.leader_id
            ldr_age = now - self._last_heartbeat_time.get(ldr, now)
            if ldr_age > timeout_s and self._on_leader_lost is not None:
                self._on_leader_lost(ldr, set(self._alive_ids))

            time.sleep(0.5)

    def start_heartbeat_watchdog(self, timeout_s: float = 3.0):
        """Starts the heartbeat watchdog thread. It should be called after parallel_launch."""
        # Reset timestamps before starting the watchdog so that the start time does not trigger false alerts.
        now = time.monotonic()
        for did in self.drone_ids:
            self._last_heartbeat_time[did] = now

        t = threading.Thread(
            target=self._heartbeat_watchdog_thread,
            args=(timeout_s,),
            name="HBWatchdog",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Heartbeat watchdog thread started.")

    # ---------------------------------------------------------------------------------- Security feature
    # 2 — Keyboard emergency listener 'r' → to all drones RTL 'l' → to all drones LAND 'q' → stop program
    # (handles existing finally block RTL) Terminal runs in raw mode; reads tus without blocking.
    # ------------------------------------------------------------------
    def _keyboard_listener_thread(self):
        logger.info(
            "[Keyboard] Emergency listener started. "
            "r=RTL, l=LAND, q=Output"
        )
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self.stop_threads.is_set():
                # Check if a character exists without blocking
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1).lower()
                    if ch == 'r':
                        logger.warning("[Keyboard] 'r' detected — sending RTL to all drones!")
                        log_event("emergency_command", command="RTL", source="keyboard")
                        for did in self.drone_ids:
                            self.set_mode(did, "RTL")
                    elif ch == 'l':
                        logger.warning("[Keyboard] 'l' detected — sending LAND to all drones!")
                        log_event("emergency_command", command="LAND", source="keyboard")
                        for did in self.drone_ids:
                            self.set_mode(did, "LAND")
                    elif ch == 'q':
                        logger.warning("[Keyboard] 'q' detected — program terminating!")
                        self.stop_threads.set()
                        break
        except Exception:
            logger.exception("Keyboard listener error.")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def start_keyboard_listener(self):
        """Starts the keyboard emergency listener thread."""
        t = threading.Thread(
            target=self._keyboard_listener_thread,
            name="KeyboardListener",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Keyboard listener initialized (r=RTL, l=LAND, q=Exit).")

    # --------------------------------------------------------------------------- Safety feature 3 —
    # Leader position watchdog thread If the leader drone's position LOCAL_POSITION_NED is not updated for
    # more than 2 seconds, all followers are put into LOITER mode and formation commands are paused. When
    # the position returns, the formation continues.
    # ------------------------------------------------------------------
    def _leader_position_watchdog_thread(self, stale_timeout_s: float = 2.0):
        loiter_active = False
        last_warn     = 0.0
        logger.info(f"Leader position watchdog started (timeout={stale_timeout_s}s)")

        while not self.stop_threads.is_set():
            now = time.monotonic()
            age = now - self._last_leader_pos_time
            stale = age > stale_timeout_s

            if stale and not loiter_active:
                logger.warning(
                    f"[LeaderKeeper] Leader position {age:.1f}s has not been updated! "
                    f"Followers are taken to LOITER."
                )
                log_event("leader_position_lost", age_s=round(age, 2))
                self.follow_enable.clear()   # stop formation commands
                self.leader_pos_ok.clear()
                for did in self.drone_ids:
                    if did != self.leader_id and did not in self._grounded_ids:
                        self.set_mode(did, "LOITER")
                loiter_active = True

            elif stale and loiter_active:
                if now - last_warn > 5.0:
                    logger.warning(
                        f"[LeaderWatchman] Leader position is still stale ({age:.1f}s). "
                        f"Followers are waiting at LOITER."
                    )
                    last_warn = now

            elif not stale and loiter_active:
                logger.info(
                    "[LeaderWatchman] Leader position is back — "
                    "followers are moved to GUIDED and the formation continues."
                )
                log_event("leader_position_recovered")
                for did in self.drone_ids:
                    if did != self.leader_id and did not in self._grounded_ids:
                        self.set_mode(did, "GUIDED")
                self.leader_pos_ok.set()
                self.follow_enable.set()   # restart formation commands
                loiter_active = False

            time.sleep(0.5)

    def start_leader_position_watchdog(self, stale_timeout_s: float = 2.0):
        """The leader starts the position watchdog thread. It should be called after parallel_launch."""
        self._last_leader_pos_time = time.monotonic()
        t = threading.Thread(
            target=self._leader_position_watchdog_thread,
            args=(stale_timeout_s,),
            name="LeaderPosWatchdog",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("The leader position watchdog thread has been started.")

    # ------------------------------------------------------------------ Status broadcaster for live
    # formation visualization panel Broadcasts swarm status (leader + follower NED positions, modes,
    # assigned slots) periodically to Redis; viz_dashboard.py reads this and draws it live in the browser.
    # Also saves as "state_snapshot" to events.jsonl at lower speed (for post-flight replay/analysis).
    # ------------------------------------------------------------------
    def _state_publisher_thread(self, redis_client, rate_hz: float = 5.0, jsonl_rate_hz: float = 1.0):
        REDIS_KEY = "swarm_state"
        dt        = 1.0 / max(rate_hz, 0.1)
        jsonl_dt  = 1.0 / max(jsonl_rate_hz, 0.01)
        last_jsonl = 0.0
        logger.info(
            f"Status publisher initialized (live={rate_hz:.1f}Hz, JSONL={jsonl_rate_hz:.1f}Hz)."
        )

        while not self.stop_threads.is_set():
            try:
                with self.lock:
                    positions  = dict(self.drone_positions)
                    headings   = dict(self.drone_headings)
                    modes      = dict(self.drone_modes)
                    velocities = dict(self.drone_velocities)
                slot_assignments  = dict(self._slot_assignments)
                formation_targets = dict(self._last_formation_targets)
                now_mono = time.monotonic()

                def _drone_snapshot(did):
                    sysh = self._sys_health.get(did) or {}
                    return {
                        "position": positions.get(did),
                        "heading": headings.get(did),
                        "mode": modes.get(did),
                        "slot": slot_assignments.get(did),
                        "is_leader": did == self.leader_id,
                        # --- Enhanced telemetry (for viz_dashboard) ---
                        "velocity": velocities.get(did),
                        "armed": self.drone_armed.get(did, False),
                        "hb_age_s": round(
                            now_mono - self._last_heartbeat_time.get(did, now_mono), 2
                        ),
                        "accel_g": round(self._last_accel_g.get(did, 1.0), 2),
                        "impact": bool(self._impact_flag.get(did, False)),
                        "grounded": did in self._grounded_ids,
                        "alive": did in self._alive_ids,
                        "attacker": did == self._committed_attacker,
                        "batt_v": sysh.get("batt_v"),
                        "batt_a": sysh.get("batt_a"),
                        # The slot target (n, e, d) last calculated by the formation manager for this drone — the panel draws
                        # a dashed target line.
                        "target": formation_targets.get(did),
                    }

                snapshot = {
                    "ts": time.time(),
                    "leader_id": self.leader_id,
                    "formation": self._current_formation,
                    "offset": self._current_offset,
                    "follow_enabled": self.follow_enable.is_set(),
                    "leader_pos_ok": self.leader_pos_ok.is_set(),
                    "attacker_id": self._committed_attacker,
                    "drones": {str(did): _drone_snapshot(did) for did in self.drone_ids},
                }

                try:
                    redis_client.set(REDIS_KEY, json.dumps(snapshot))
                except Exception:
                    logger.exception("Error publishing status to Redis.")

                now = time.monotonic()
                if now - last_jsonl > jsonl_dt:
                    log_event("state_snapshot", **snapshot)
                    last_jsonl = now

            except Exception:
                logger.exception("State publisher thread error.")

            time.sleep(dt)

    def start_state_publisher(self, redis_client, rate_hz: float = 5.0, jsonl_rate_hz: float = 1.0):
        """
        Starts the status publisher thread for the live pattern visualization panel
        (viz_dashboard.py). It doesn't matter when it is called — it can also be called before
        takeoff, if there is no position yet, the fields will be None.
        """
        t = threading.Thread(
            target=self._state_publisher_thread,
            args=(redis_client,),
            kwargs={"rate_hz": rate_hz, "jsonl_rate_hz": jsonl_rate_hz},
            name="StatePublisher",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("The status publisher thread has been started.")

    # ------------------------------------------------------------------ Heartbeat / connection helpers
    # ------------------------------------------------------------------
    def wait_for_all_heartbeats(self, timeout: float = 30.0):
        logger.info(f"Waiting for heartbeats: {self.drone_ids}")
        unseen   = set(self.drone_ids)
        deadline = time.time() + timeout

        while unseen and time.time() < deadline:
            for did in list(unseen):
                try:
                    self._hb_queues[did].get(timeout=0.2)
                    unseen.discard(did)
                    logger.info(f"Heartbeat received -> {did}, remaining={unseen or 'none'}")
                except queue.Empty:
                    pass

        if unseen:
            raise ConnectionError(f"Could not connect to all drones. Missing: {unseen}")
        logger.info("Heartbeat was received from all drones. The system is ready.")

    # ------------------------------------------------------------------ Mod helpers
    # ------------------------------------------------------------------
    def is_in_guided_mode(self, target_id: int) -> bool:
        with self.lock:
            current_mode = self.drone_modes.get(target_id, "")
        logger.debug(f"is_in_guided_mode drone={target_id} mode={current_mode}")
        return current_mode == "GUIDED"

    def _wait_for_guided(self, drone_id: int, timeout: float = 15.0) -> bool:
        """Waits until Heartbeat displays custom_mode == 4 (GUIDED).  """
        deadline = time.time() + timeout
        q        = self._hb_queues[drone_id]
        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                if msg.custom_mode == 4:
                    return True
                self.set_mode(drone_id, "GUIDED")
            except queue.Empty:
                self.set_mode(drone_id, "GUIDED")
        return False

    # ------------------------------------------------------------------ Task assistants for the AUTO loop
    # ------------------------------------------------------------------
    def _restart_mission(self, target_id: int, seq: int = 2):
        """
        Restarts the AUTO duty cycle by sending MAV_CMD_DO_SET_MISSION_CURRENT (seq=seq).
        """
        logger.info(f"[MissionRestart] Mission restarting from seq={seq} (drone={target_id})")
        log_event("mission_restart", drone_id=target_id, seq=seq)
        self.master.mav.command_long_send(
            target_id, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            0,
            float(seq),   # param1: sequence number to skip
            0, 0, 0, 0, 0, 0,
        )

    def _get_mission_count(self, target_id: int, timeout: float = 5.0) -> int:
        """
        Requests MISSION_COUNT from target_id and returns the task item number. Returns -1 in case
        of timeout.
        """
        while True:
            try: self._mission_count_queue.get_nowait()
            except queue.Empty: break

        self.master.mav.mission_request_list_send(target_id, 0)
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = self._mission_count_queue.get(
                    timeout=min(1.0, deadline - time.time())
                )
                if msg.get_srcSystem() == target_id:
                    logger.info(f"[MissionRestart] Mission count={msg.count} (drone={target_id})")
                    return int(msg.count)
                try: self._mission_count_queue.put_nowait(msg)
                except queue.Full: pass
            except queue.Empty:
                self.master.mav.mission_request_list_send(target_id, 0)

        logger.warning(f"[MissionRestart] MISSION_COUNT timeout (drone={target_id})")
        return -1

    def _get_last_waypoint_global(self, target_id: int, last_wp_index: int, timeout: float = 5.0):
        """
        It requests MISSION_ITEM_INT for index (last_wp_index - 1) and returns (lat, lon, alt)
        values ​​as float. Returns None in case of timeout.
        """
        item_seq = last_wp_index - 1
        while True:
            try: self._mission_item_queue.get_nowait()
            except queue.Empty: break

        self.master.mav.mission_request_int_send(target_id, 0, item_seq)
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = self._mission_item_queue.get(timeout=min(1.0, deadline - time.time()))
                if msg.get_srcSystem() == target_id and msg.seq == item_seq:
                    lat = msg.x / 1e7
                    lon = msg.y / 1e7
                    alt = msg.z
                    logger.info(
                        f"[MissionRestart] Last WP coordinate (seq={item_seq}): "
                        f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}"
                    )
                    return lat, lon, alt
                try: self._mission_item_queue.put_nowait(msg)
                except queue.Full: pass
            except queue.Empty:
                self.master.mav.mission_request_int_send(target_id, 0, item_seq)

        logger.warning(f"[MissionRestart] MISSION_ITEM_INT timeout (drone={target_id}, seq={item_seq})")
        return None

    # ------------------------------------------------------------------ MAVLink command assistants
    # ------------------------------------------------------------------
    def set_mode(self, target_system_id: int, mode_name: str):
        COPTER_MODES = {
            "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
            "GUIDED": 4,    "LOITER": 5, "RTL": 6,    "CIRCLE": 7,
            "LAND": 9,      "POSHOLD": 16, "BRAKE": 17,
        }
        mode_id = COPTER_MODES.get(mode_name.upper())
        if mode_id is None:
            logger.warning(f"Bilinmeyen mode: '{mode_name}'")
            return
        logger.info(f"Mode set -> tgt={target_system_id}, mode={mode_name}, id={mode_id}")

        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )
        time.sleep(0.05)

    def arm(self, target_system_id: int, timeout: float = 15.0,
            retry_interval: float = 2.0) -> bool:
        """
        He arms the drone and verifies that it is indeed armed.

        param2 = 0 → prearm safety checks (GPS/EKF/compass/battery) APPLIED (NO force-arm). If the
        checks do not pass, the drone will not arm.

        Verification: * Primary: SAFETY_ARMED flag on HEARTBEAT (constant income even on lost link,
        most reliable confirmation). * Secondary: COMMAND_ACK — used to log the reason for the
        rejection.

        Returns True on success, False on timeout/rejection.
        """
        ack_q = self._command_ack_queues.get(target_system_id)
        # Clear old ACKs so we don't read the response to previous commands
        if ack_q is not None:
            while True:
                try: ack_q.get_nowait()
                except queue.Empty: break

        logger.info(f"Arm -> {target_system_id} (prearm controls active)")
        deadline    = time.time() + timeout
        last_send   = 0.0
        denied_seen = False

        while time.time() < deadline:
            now = time.time()
            # Resend the command periodically (resistance to packet loss and subsequent recovery of the prearm —
            # e.g. GPS fix —).
            if now - last_send >= retry_interval:
                self.master.mav.command_long_send(
                    target_system_id, 0,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0,   # param1=1 (arm), param2=0 (NO force)
                )
                last_send = now

            # 1) Primary approval: heartbeat armed flag
            if self.drone_armed.get(target_system_id):
                logger.info(f"Drone {target_system_id}: ARMED approved (heartbeat).")
                log_event("arm_confirmed", drone_id=target_system_id)
                return True

            # 2 ) COMMAND_ACK — log result/cause
            if ack_q is not None:
                try:
                    ack = ack_q.get(timeout=0.5)
                    res = ack.result
                    if res == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        logger.info(f"Drone {target_system_id}: arm command accepted (ACK) — "
                                    f"Heartbeat approval awaited.")
                    elif res in (mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                                 mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                        logger.warning(f"Drone {target_system_id}: arm temporarily rejected "
                                       f"(result= {res} ) — will try again.")
                    else:
                        if not denied_seen:
                            logger.error(f"Drone {target_system_id}: ARM REJECTED (result={res}) "
                                         f"— prearm control may fail (GPS/EKF/battery).")
                            log_event("arm_denied", drone_id=target_system_id, result=int(res))
                            denied_seen = True
                except queue.Empty:
                    pass
            else:
                time.sleep(0.2)

        logger.error(f"Drone {target_system_id}: Failed to verify ARM in {timeout:.0f}s.")
        log_event("arm_timeout", drone_id=target_system_id)
        return False

    def takeoff(self, target_system_id: int, altitude: float):
        logger.info(f"Takeoff -> {target_system_id} alt={altitude}")
        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,
        )
        time.sleep(0.05)

    def land(self, target_system_id: int):
        logger.info(f"LAND -> {target_system_id}")
        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.05)

    def goto_global_int(self, target_system_id: int, lat: float, lon: float, alt: float):
        logger.info(f"GOTO_GLOBAL_INT -> tgt={target_system_id} lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}")
        self.master.mav.set_position_target_global_int_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def goto_local_ned(self, target_system_id: int, north: float, east: float, down: float):
        logger.debug(f"GOTO_LOCAL_NED -> tgt={target_system_id} NED=({north:.2f},{east:.2f},{down:.2f})")
        self.master.mav.set_position_target_local_ned_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            north, east, down,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    # FIX: target point is in COMMON NED frame. MAV_FRAME_LOCAL_NED is interpreted by each vehicle
    # according to its OWN EKF origin; so it was shifting until the same (n,e,d) means a different
    # physical point on every drone -> followers flew toward slots (own_orijini - lider_orijini), that
    # is, in the wrong direction. Here, the target in the common frame is converted back to
    # lat/lon/alt(AMSL) and sent as MAV_FRAME_GLOBAL_INT and SET_POSITION_TARGET_GLOBAL_INT; This is
    # independent of origin: each vehicle flies to the same physical point.
    def goto_shared_ned(self, target_system_id: int, north: float, east: float, down: float,
                        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                        ax: float = 0.0, ay: float = 0.0, az: float = 0.0,
                        yaw: float = 0.0, yaw_rate: float = 0.0,
                        type_mask: int = POSITION_ONLY_MASK):
        """
        Sends a target in the common NED frame to the vehicle. To make the location independent of
        the origin, it is converted to lat/lon/alt(AMSL) and sent with GLOBAL_INT.

        FEEDFORWARD: the velocity/acceleration/yaw fields and type_mask are POSITION-ALONE (mask ==
        POSITION_ONLY_MASK) by default, so the behavior of follower and attacker calls (passing
        n,e,d only) does NOT change. The leader slot reader passes vx/vy/vz + ax/ay/az + yaw +
        type_mask published by simple_guided_follow here; The --no-position-only mode thus produces
        a truly velocity/acceleration feedforward flight profile (otherwise the flag is
        non-functional: both modes fly attitude-alone).

        NOTE (frame): since the velocity/acceleration is on the NED (North/East/Down) axes and
        independent of the origin, the vx/vy/vz issued in the common frame can be passed directly to
        the GLOBAL_INT message — only the POSITION needs to be converted. type_mask bit meanings are
        the same as LOCAL_NED in _GLOBAL_INT. Publisher's 'coordinate_frame' field
        (MAV_FRAME_LOCAL_NED) is NOT used here; For global shipping, MAV_FRAME_GLOBAL_INT is fixed.
        """
        if self._ned_origin is None:
            logger.warning(
                f"goto_shared_ned: NED origin does not exist yet, command skipped (tgt={target_system_id})."
            )
            return
        lat, lon, alt = ned_to_latlon(north, east, down, *self._ned_origin)
        logger.debug(
            f"GOTO_SHARED_NED -> tgt={target_system_id} NED=({north:.2f},{east:.2f},{down:.2f}) "
            f"=> lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m AMSL mask={type_mask}"
        )
        self.master.mav.set_position_target_global_int_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,   # alt = AMSL (origin_alt - down)
            int(type_mask),                         # publisher's chosen mask (default position-alone)
            int(lat * 1e7), int(lon * 1e7), alt,
            float(vx), float(vy), float(vz),        # speed feedforward (NED m/s)
            float(ax), float(ay), float(az),        # acceleration feedforward ( NED m/s ^ 2 )
            float(yaw), float(yaw_rate),            # yaw [rad], yaw_rate [rad/s]
        )

    def _goto_with_yaw_lock(self, drone_id, north, east, down, lock_yaw=None):
        """
        goto_shared_ned wrapper: commands the rotated head angle together with POSITION_YAW_MASK if
        the yaw lock is present, otherwise nothing changes (position-alone, old behavior).

        If lock_yaw None is passed, the lock status is read here; so the caller does not need to
        query separately (like the formation loop).
        """
        if lock_yaw is None:
            lock_yaw = self._locked_yaw(drone_id)
        if lock_yaw is None:
            self.goto_shared_ned(drone_id, north, east, down)
        else:
            self.goto_shared_ned(drone_id, north, east, down,
                                 yaw=lock_yaw, yaw_rate=0.0,
                                 type_mask=POSITION_YAW_MASK)

    # ------------------------------------------------------------------ Waypoint arrival wait (with MINE
    # — FRIEND's guided-mode protection) Reads from _nav_queues to avoid stealing packets from Outpatcher.
    # ------------------------------------------------------------------
    def wait_for_waypoint_arrival(self, target_id, radius=3.0, timeout=60):
        logger.info(f"WP standby -> drone= {target_id} , radius= {radius} , timeout= {timeout}")
        start   = time.time()
        q       = self._nav_queues[target_id]
        while time.time() - start < timeout:
            if not self.is_in_guided_mode(target_id):
                logger.warning(f"While drone {target_id} was waiting for WP, GUIDED exited the mode. Cancel.")
                return False
            try:
                msg = q.get(timeout=1.0)
                d   = msg.wp_dist
                logger.info(f"[WP] drone= {target_id} distance= {d:.2f} m")
                if d <= radius:
                    logger.info(f"Drone {target_id} arrived at WP.")
                    return True
            except queue.Empty:
                pass
        logger.warning(f"Drone {target_id} WP timeout.")
        return False

    # ------------------------------------------------------------------ Altitude hold
    # ------------------------------------------------------------------
    def wait_for_takeoff(
        self,
        target_sysid: int,
        target_alt_m: float,
        threshold: float = 0.85,
        timeout: float   = 60.0,
    ) -> bool:
        needed   = target_alt_m * threshold
        deadline = time.time() + timeout
        q        = self._gpos_queues[target_sysid]
        logger.info(f"Drone {target_sysid}: climbing to {needed:.1f} m...")

        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                alt = msg.relative_alt / 1000.0
                logger.debug(f"Drone {target_sysid}: {alt:.1f} m")
                if alt >= needed:
                    logger.info(f"Drone {target_sysid}: airborne at {alt:.1f} m [OK]")
                    return True
            except queue.Empty:
                pass

        logger.warning(f"Drone {target_sysid}: did not reach altitude in time.")
        return False

    # ------------------------------------------------------------------ Parallel takeoff — drone uses
    # build_takeoff_altitudes for press altitude (MINE + FRIEND)
    # ------------------------------------------------------------------
    def parallel_launch(self, drone_ids: list, takeoff_altitudes: dict):
        """
        All drones take off at the same time. takeoff_altitudes: dict[drone_id -> altitude_m] (from
        build_takeoff_altitudes)
        """
        results = {}
        barrier = threading.Barrier(len(drone_ids))

        def launch_one(drone_id):
            alt = takeoff_altitudes[drone_id]

            # Request GUIDED
            self.set_mode(drone_id, "GUIDED")
            guided = self._wait_for_guided(drone_id, timeout=15)
            if not guided:
                logger.warning(f"Drone {drone_id}: GUIDED not confirmed, continuing.")

            # Arm — prearm controls active, result verified
            logger.info(f"Drone {drone_id}: arming...")
            if not self.arm(drone_id):
                logger.error(f"Drone {drone_id}: ARM failed — takeoff CANCELED.")
                results[drone_id] = False
                barrier.abort()   # release other waiting threads
                return

            # Departure
            logger.info(f"Drone {drone_id}: takeoff to {alt} m...")
            self.takeoff(drone_id, alt)

            # altitude hold
            ok = self.wait_for_takeoff(drone_id, alt, threshold=0.85, timeout=60)
            results[drone_id] = ok

            # Store takeoff NED position for optional RTB use
            with self.lock:
                pos = self.drone_positions.get(drone_id, (0, 0, 0))
                self.takeoff_positions[drone_id] = (pos[0], pos[1], -alt)

            logger.info(f"Drone {drone_id}: ready, waiting at barrier...")
            try:
                barrier.wait(timeout=90)
            except threading.BrokenBarrierError:
                logger.warning(f"Drone {drone_id}: barrier broken — continuing.")

        threads = []
        for drone_id in drone_ids:
            t = threading.Thread(target=launch_one, args=(drone_id,), daemon=True)
            t.start()
            threads.append(t)
            time.sleep(0.3)   # cascade mod commands

        for t in threads:
            t.join(timeout=120)

        failed    = [did for did in drone_ids if not results.get(did)]
        succeeded = [did for did in drone_ids if results.get(did)]

        if not failed:
            logger.info("All drones airborne [OK]")
            return

        logger.warning(
            f"Failed to take off (arm/altitude): {failed} — these drones are decommissioned "
            f"are removed (RTL), the swarm continues without them."
        )

        # User request: even if a follower OR leader fails to take off, the mission should continue without
        # that drone (same as RTL+continue in heartbeat loss). If the leader fails, a new leader is chosen
        # from the survivors according to SUCCESSION_ORDER; If there is no suitable leader it is truly
        # aborted.
        if self.leader_id in failed:
            new_ldr = next_leader(SUCCESSION_ORDER, set(succeeded), self.leader_id)
            if new_ldr is None:
                for did in failed:
                    self._ground_drone(did, reason="launch_failed")
                raise RuntimeError(
                    f"Leader {self.leader_id} and all backups fail takeoff — "
                    f"There is no drone to fly, the mission is cancelled."
                )
            logger.warning(
                f"[Launch] Leader {self.leader_id} fails to start -> new leader "
                f"{new_ldr} (deceased: {sorted(succeeded)})."
            )
            log_event("leader_reassigned_launch", old_leader=self.leader_id,
                      new_leader=new_ldr, alive=sorted(succeeded))
            self.leader_id = new_ldr

        # Remove all failed drones from the mission (RTL + permanent exclusion).
        for did in failed:
            self._ground_drone(did, reason="launch_failed")

    def _ground_drone(self, drone_id: int, reason: str = ""):
        """
        He REMOVES one drone from the mission: RTL and the rest of the swarm continues without him.
        It is the startup-error counterpart of the heartbeat loss behavior (RTL + continue).
        Grounded drone is no longer considered 'alive'; formation, leader selection, attacker
        selection and position guard bypass it.
        """
        logger.warning(f"[Ground] Drone {drone_id} is being decommissioned ({reason}) "
                       f"-> RTL, the swarm continues without him.")
        log_event("drone_grounded", drone_id=drone_id, reason=reason)
        self._grounded_ids.add(drone_id)
        self._alive_ids.discard(drone_id)
        self.set_mode(drone_id, "RTL")

    # ------------------------------------------------------------------ Formation calculation (MINE —
    # yaw-rotated + lookahead) ------------------------------------------------------------------
    def _slots_symmetric(self, n: int) -> list:
        """Symmetric slot generator for HORIZONTAL_LINE."""
        slots = []
        k = 1
        while len(slots) < n:
            if len(slots) < n: slots.append(-k)
            if len(slots) < n: slots.append(k)
            k += 1
        return slots

    def _compute_all_slots(
        self,
        leader_pos,
        leader_yaw,
        leader_id,
        formation,
        offset,
        vertical_offsets,
        ):
        """
        Calculates NED positions of all follower slots.

        Offsets are defined in the leading body frame (Forward, Right, Down) and then frozen into
        the world NED frame with just rotation yaw. The lookahead is also applied to the body frame
        — this maintains the symmetry of the formation on the leader's turns (the global NED
        lookahead was breaking the formation asymmetrically on the turns).
        """
        leader_vx, leader_vy, _ = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        lookahead_time = 2.5

        # Convert lookahead to body frame (reverse rotation yaw) This ensures the center point always remains
        # on the leader nose as the leader rotates
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        # Global NED speed → body frame speed (inverse of yaw = transpose yaw)
        vx_body =  leader_vx * cos_yaw + leader_vy * sin_yaw   # forward
        vy_body = -leader_vx * sin_yaw + leader_vy * cos_yaw   # Right

        # Lookahead is applied to the body frame
        fwd_lookahead = vx_body * lookahead_time
        rgt_lookahead = vy_body * lookahead_time

        followers = sorted([fid for fid in self.drone_ids if fid != leader_id])
        n         = len(followers)

        # Body frame slot definitions: (forward, right, down_delta) forward > 0 = in front of the leader,
        # right > 0 = to the right of the leader, down_delta > 0 = below the leader
        body_slots = []

        if formation == Formation.LINE:
            for i in range(n):
                slot = i - (n - 1) / 2.0
                # LINE: drones line up left and right around the leader (on the right axis)
                body_slots.append((-slot * offset, 0.0, 0.0))

        elif formation == Formation.HORIZONTAL_LINE:
            if self.symmetric_horizontal:
                sym_slots = self._slots_symmetric(n)
                for s in sym_slots:
                    body_slots.append((0.0, s * offset, 0.0))
            else:
                for i in range(n):
                    slot = i - (n - 1) / 2.0
                    body_slots.append((0.0, slot * offset, 0.0))

        elif formation == Formation.V_SHAPE:
            for i in range(n):
                v_slot = i + 1
                side   = -1 if v_slot % 2 == 0 else 1
                fwd    = -((v_slot // 2 + 1) * offset * 0.7)
                rgt    = ((v_slot // 2 + 1) * offset * 0.7) * side
                body_slots.append((fwd, rgt, 0.0))

        elif formation == Formation.DIAMOND:
            # Horizontal slots (right/left): body frame right axis Vertical slots (top/bottom): NED Z axis —
            # controlled by VERTICAL_OFFSETS
            positive_offsets = [v for v in vertical_offsets.values() if v > 0]
            negative_offsets = [v for v in vertical_offsets.values() if v < 0]
            top_gap    = max(positive_offsets) if positive_offsets else offset
            bottom_gap = abs(min(negative_offsets)) if negative_offsets else offset
            body_slots = [
                (0.0,  offset, 0.0),          # Right
                (0.0, -offset, 0.0),          # left
                (0.0,  0.0,   -top_gap),      # top (NED down negative = up)
                (0.0,  0.0,    bottom_gap),   # alt
            ]

        # Body frame → earth NED transformation (yaw only) Rotation matrix (yaw only):
        #   NED_north = fwd * cos(yaw) - rgt * sin(yaw)
        #   NED_east  = fwd * sin(yaw) + rgt * cos(yaw)
        #   NED_down = down_delta (vertical, not affected by yaw)
        result = []
        for (fwd, rgt, down_delta) in body_slots:
            # Merge lookahead with body offset, then rotate together
            total_fwd = fwd + fwd_lookahead
            total_rgt = rgt + rgt_lookahead

            ned_n = total_fwd * cos_yaw - total_rgt * sin_yaw
            ned_e = total_fwd * sin_yaw + total_rgt * cos_yaw
            ned_d = leader_pos[2] + down_delta   # vertical: global NED Z independent of yaw

            result.append((leader_pos[0] + ned_n, leader_pos[1] + ned_e, ned_d))

        return result, followers

    def _assign_slots(self, followers, slot_positions, vertical_offsets, leader_d):
        """
        It assigns each follower to the nearest free slot — by hysteresis. The current assignment is
        preserved (prevents slot swaps) if it is no closer than SLOT_REASSIGN_THRESHOLD_M from the
        new best slot. Returns: {follower_id: slot_index}
        """
        threshold  = self._slot_reassign_threshold_m
        n          = len(followers)
        assignment = {}   # follower_id -> slot_index (new assignment for this tick)
        used_slots = set()

        # 1. Calculate temporary best assignment (nearest free slot for each drone)
        best_for = {}   # follower_id -> (dist, slot_index)
        for fid in followers:
            with self.lock:
                pos = self.drone_positions.get(fid)
            if pos is None:
                best_for[fid] = (float('inf'), 0)
                continue
            dists = []
            for si, (sn, se, sd) in enumerate(slot_positions):
                dn = pos[0] - sn
                de = pos[1] - se
                dd = pos[2] - sd
                dists.append((math.sqrt(dn*dn + de*de + dd*dd), si))
            dists.sort()
            best_for[fid] = dists[0]   # (min_dist, best_slot_index)

        # 2. Hysteresis: if the current assignment is valid and the new best slot is not better enough, keep
        # the current assignment
        locked = {}   # follower_id -> slot_index (persistent assignments)
        for fid in followers:
            current_slot = self._slot_assignments.get(fid)
            if current_slot is not None and current_slot < n:
                sn, se, sd = slot_positions[current_slot]
                with self.lock:
                    pos = self.drone_positions.get(fid)
                if pos is not None:
                    dn   = pos[0] - sn
                    de   = pos[1] - se
                    dd = pos[2] - sd
                    curr_dist = math.sqrt(dn*dn + de*de + dd*dd)
                    best_dist, best_slot = best_for[fid]
                    # Change only if the new slot is closer to the threshold than the existing slot
                    if best_slot != current_slot and (curr_dist - best_dist) < threshold:
                        locked[fid] = current_slot   # not good enough, keep current assignment

        # 3. Apply locked assignments first
        for fid, si in locked.items():
            if si not in used_slots:
                assignment[fid] = si
                used_slots.add(si)

        # 4. Assign unlocked drones to the nearest free slot (greedy)
        unassigned = [fid for fid in followers if fid not in assignment]
        # Sort by distance: nearest drone shoots first
        unassigned.sort(key=lambda fid: best_for[fid][0])
        for fid in unassigned:
            best_dist, best_slot = best_for[fid]
            if best_slot not in used_slots:
                assignment[fid] = best_slot
                used_slots.add(best_slot)
            else:
                # Preferred slot taken — find nearest free slot
                with self.lock:
                    pos = self.drone_positions.get(fid)
                fallback = None
                fallback_dist = float('inf')
                for si, (sn, se, sd) in enumerate(slot_positions):
                    if si in used_slots:
                        continue
                    if pos is not None:
                        dn = pos[0] - sn
                        de = pos[1] - se
                        dd = pos[2] - sd
                        d  = math.sqrt(dn*dn + de*de + dd*dd)
                    else:
                        d = float('inf')
                    if d < fallback_dist:
                        fallback_dist = d
                        fallback = si
                if fallback is not None:
                    assignment[fid] = fallback
                    used_slots.add(fallback)

        # Cache assignment — log changed assignments as events
        for fid, si in assignment.items():
            prev = self._slot_assignments.get(fid)
            if prev != si:
                log_event(
                    "slot_reassignment",
                    follower_id=fid, previous_slot=prev, new_slot=si,
                )
        self._slot_assignments.update(assignment)
        return assignment

    def _assign_slots_diamond(self, followers, slot_positions, vertical_offsets):
        """
        Special slot assignment for DIAMOND formation. Slot 0: right, Slot 1: left (horizontal — low
        vertical offset) Slot 2: top, Slot 3: bottom (vertical — high vertical offset)

        Assignment logic: 1. Drones |vertical_offset| sort by size. 2. Largest 2 |offset| → vertical
        slots (2=top, 3=bottom). Determine which one goes above and which one goes below based on
        the drone's current Z position (higher up → upper slot). 3. Remaining 2 drone → horizontal
        slots (0=right, 1=left). Determine which one goes right and which one goes left based on the
        current horizontal position of the drone (with 2D distance to the slot).
        """
        assignment = {}

        # Sort drones from largest to smallest by |vertical_offset|
        sorted_by_voffset = sorted(
            followers,
            key=lambda fid: abs(vertical_offsets.get(fid, 0.0)),
            reverse=True,
        )

        vertical_drones   = sorted_by_voffset[:2]   # largest 2 vertical offset → top/bottom
        horizontal_drones = sorted_by_voffset[2:]   # remainder → right/left

        # --- Assign vertical drones to top (slot 2) and bottom (slot 3) --- Look at current Z position: more
        # negative Z (higher) → to top slot
        slot2_pos = slot_positions[2]   # upper slot NED position
        slot3_pos = slot_positions[3]   # alt slot NED pozisyonu

        d0, d1 = vertical_drones[0], vertical_drones[1]
        with self.lock:
            pos0 = self.drone_positions.get(d0)
            pos1 = self.drone_positions.get(d1)

        # If the position is unknown, use the sign of the vertical offset (+ = up = slot 2)
        if pos0 is None or pos1 is None:
            for fid in vertical_drones:
                v = vertical_offsets.get(fid, 0.0)
                assignment[fid] = 2 if v > 0 else 3
        else:
            # Compare the total distance to slot 2 and slot 3 for both drones and choose the best pair (minimum
            # total distance assignment)
            def dist3(pos, slot):
                return math.sqrt(sum((pos[i] - slot[i])**2 for i in range(3)))

            cost_0to2 = dist3(pos0, slot2_pos) + dist3(pos1, slot3_pos)
            cost_0to3 = dist3(pos0, slot3_pos) + dist3(pos1, slot2_pos)

            if cost_0to2 <= cost_0to3:
                assignment[d0] = 2
                assignment[d1] = 3
            else:
                assignment[d0] = 3
                assignment[d1] = 2

        # --- Assign horizontal drones to right (slot 0) and left (slot 1) ---
        if len(horizontal_drones) >= 2:
            h0, h1 = horizontal_drones[0], horizontal_drones[1]
            slot0_pos = slot_positions[0]
            slot1_pos = slot_positions[1]

            with self.lock:
                hpos0 = self.drone_positions.get(h0)
                hpos1 = self.drone_positions.get(h1)

            if hpos0 is None or hpos1 is None:
                assignment[h0] = 0
                assignment[h1] = 1
            else:
                def dist2(pos, slot):
                    return math.sqrt((pos[0]-slot[0])**2 + (pos[1]-slot[1])**2)

                cost_0to0 = dist2(hpos0, slot0_pos) + dist2(hpos1, slot1_pos)
                cost_0to1 = dist2(hpos0, slot1_pos) + dist2(hpos1, slot0_pos)

                if cost_0to0 <= cost_0to1:
                    assignment[h0] = 0
                    assignment[h1] = 1
                else:
                    assignment[h0] = 1
                    assignment[h1] = 0
        elif len(horizontal_drones) == 1:
            assignment[horizontal_drones[0]] = 0

        # Save to cache and log slot changes
        for fid, si in assignment.items():
            prev = self._slot_assignments.get(fid)
            if prev != si:
                log_event("slot_reassignment", follower_id=fid, previous_slot=prev, new_slot=si)
        self._slot_assignments.update(assignment)
        return assignment

    def calculate_formation_target(
        self,
        leader_pos,
        leader_yaw,
        follower_id,
        leader_id,
        formation,
        offset,
        vertical_offsets,
        min_alt_m: float = 10.0,
        ):
        """
        Calculates the target NED for a follower. Slot assignment is made according to the closest
        slot; Unnecessary slot changes are prevented with hysteresis.
        """
        slot_positions, followers = self._compute_all_slots(
        leader_pos, leader_yaw, leader_id, formation, offset, vertical_offsets
        )
        leader_d = leader_pos[2]
        if formation == Formation.DIAMOND:
            assignment = self._assign_slots_diamond(followers, slot_positions, vertical_offsets)
        else:
            assignment = self._assign_slots(followers, slot_positions, vertical_offsets, leader_d)

        si = assignment.get(follower_id)
        if si is None:
            target_d = leader_pos[2]
            sn, se = leader_pos[0], leader_pos[1]
        else:
            sn, se, _ = slot_positions[si]
            v_offset = vertical_offsets.get(follower_id, 0.0)
            target_d = leader_d - v_offset

        # --- Safety: follower altitude cannot fall below min_alt_m ---
        min_d = -abs(min_alt_m)          # NED z ceiling (at most not so negative side)
        safe_d = min(target_d, min_d)    # actual altitude (=-z) always >= min_alt_m
        if safe_d != target_d:
            logger.debug(
                f"[FollowSafety] f={follower_id}: target altitude {-target_d:.1f}m, "
                f"Upgrading to base {min_alt_m:.1f}m."
            )

        return (sn, se, safe_d)



    # ------------------------------------------------------------------ Follower thread (MINE logic +
    # FRIEND's follow_enable + guided protection)
    # ------------------------------------------------------------------
    def _formation_manager_thread(self, formation, offset, vertical_offsets, min_alt_m=10.0):
        """
        SINGLE formation manager (replaces threads per follower).

        Why single thread: in the old design, each follower was re-running the GLOBAL slot
        assignment in its own thread and changing self._slot_assignments without acquiring a lock.
        In 2 Hz , four competing threads could instantly assign two side drones to the same side.
        Here SINGLE thread calculates the target of all followers once from a single consistent
        snapshot at each tick.

        The leader can change: each tick reads self.leader_id; After the election, the formation
        fits around the new leader without the need to restart.

        Coastal: if the lead position becomes stale for a short time (< LEADER_LOITER_S), it is
        extrapolated at full speed instead of freezing — the final LOITER decision is the lead
        position keeper.

        Attacker awareness: the attacker assigned to the task (if any) is skipped here; Its guidance
        belongs to the in-process intersection thread.
        """
        logger.info(f"Formation manager started -> form={formation.name}")
        last_log = 0.0
        send_dt  = 0.1    # 10 Hz command speed (tighter tracking, more az delay)
        while not self.stop_threads.is_set():
            try:
                if self._shutting_down.is_set():
                    logger.info("[FORM] Closing detected — formation commands stopped.")
                    return
                if not self.follow_enable.is_set():
                    time.sleep(0.2)
                    continue

                leader_id = self.leader_id   # may change — re-read at each tick

                with self.lock:
                    leader_pos = self.drone_positions.get(leader_id)
                    leader_yaw = self.drone_headings.get(leader_id, 0.0)
                    leader_vel = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
                    last_pos_t = self._last_leader_pos_time

                # --- Coast: Extrapolate between 0.3..COAST with final speed --------- NO extrapolation between
                # COAST..POS_LOITER (last position kept); The position guard beyond POS_LOITER has already taken
                # LOITER.
                pos_age = time.monotonic() - last_pos_t
                if leader_pos and 0.3 < pos_age <= LEADER_COAST_S:
                    leader_pos = (
                        leader_pos[0] + leader_vel[0] * pos_age,
                        leader_pos[1] + leader_vel[1] * pos_age,
                        leader_pos[2] + leader_vel[2] * pos_age,
                    )

                if not leader_pos:
                    time.sleep(send_dt)
                    continue

                # Active followers = everyone except the leader, the attacker assigned to the mission, and grounded
                # drones at takeoff (the attacker's guidance belongs to a separate thread).
                followers = [fid for fid in self.drone_ids
                             if fid != leader_id
                             and fid != self._committed_attacker
                             and fid not in self._grounded_ids]

                # --- LIVE formation offset ---------------------------------- The 'offset' argument is just the initial
                # value; the actual value is read from guidance_config.FORMATION_OFFSET_M at each tick, so just save
                # the file in flight (no reboot). It's ramped: 10 -> 25 An edit of m doesn't shift each follower's
                # target by 15 m in one frame, but by FORMATION_OFFSET_SLEW_MPS per second -- or they'll all sidestep
                # at full throttle at the same time and the slot assignment (_assign_slots) may also reshuffle in the
                # same frame.
                offset_live = FORMATION_OFFSET_LIVE.value()
                self._current_offset = offset_live   # Appear in /state broadcast

                # Calculate all targets in a single, consistent pass.
                targets = self._compute_formation_targets(
                    leader_pos, leader_yaw, leader_id, followers,
                    formation, offset_live, vertical_offsets, min_alt_m,
                )

                for fid, tgt in targets.items():
                    if not self.is_in_guided_mode(fid):
                        continue
                    # LEFT in formation after the pass, but still fast, the attacker flies here also with the locked yaw:
                    # normally he would turn full throttle 180° since his slot is usually BEHIND.
                    self._goto_with_yaw_lock(fid, *tgt)

                now = time.time()
                if now - last_log > 2.0:
                    logger.info(
                        f"[FORM] leader={leader_id} "
                        f"n_follow={len(followers)} "
                        f"offset={offset_live:.1f}m "
                        f"lead={tuple(round(x,1) for x in leader_pos)} "
                        f"coast_age={pos_age:.1f}s"
                    )
                    last_log = now

                time.sleep(send_dt)

            except Exception:
                logger.exception("Formation manager thread error.")
                time.sleep(0.5)

    def _compute_formation_targets(self, leader_pos, leader_yaw, leader_id,
                                   followers, formation, offset, vertical_offsets,
                                   min_alt_m):
        """
        Calculates NED targets of all active followers in ONE consistent pass. For DIAMOND, it uses
        a number-tolerant body-slot table and a single nearest-slot assignment. Rotary: {takipci_id:
        (n, e, d)}.
        """
        n = len(followers)
        if n == 0:
            self._last_formation_targets = {}
            return {}

        if formation == Formation.DIAMOND:
            body_slots = diamond_body_slots(
                n, offset,
                DIAMOND_TOP_GAP_M, DIAMOND_BOTTOM_GAP_M,
                DIAMOND_LATERAL_STAGGER, DIAMOND_FORE_AFT_STAGGER,
            )
        else:
            # --- LINE / HORIZONTAL_LINE / V_SHAPE ----------------------------- In these formations, the vertical
            # separation is determined by the VERTICAL_OFFSETS dictionary.
            #
            # FIX (vertical offset was not applied): _compute_all_slots generates body slots for these three
            # formations with down_delta=0.0, meaning z of the slot is ALWAYS equal to the leader's altitude. The
            # vertical offset is NOT IN the slot — the original calculate_formation_target() applied it to the
            # drone press AFTER the slot assignment: sn, se, _ = slot_positions[si] # slot z IS DISCARDED
            #       v_offset  = vertical_offsets.get(follower_id, 0.0)
            #       target_d = leader_d - v_offset # drone's own offset In the single-thread rewrite, this step
            #       was dropped and the slot's z (== leader altitude) was used directly; after all, ALL followers
            #       were flying at the same altitude as the leader. Below the original behavior has been restored.
            #
            # Sign convention (NED, down positive): target_d = leader_d - v_offset v_offset > 0 -> ABOVE leader
            # v_offset < 0 -> BELOW leader (consistent with DIAMOND arm: there top_gap = max(positive offsets).)
            slot_positions, legacy_followers = self._compute_all_slots(
                leader_pos, leader_yaw, leader_id, formation, offset, vertical_offsets
            )
            assignment = self._assign_slots(
                legacy_followers, slot_positions, vertical_offsets, leader_pos[2]
            )
            leader_d = leader_pos[2]
            out = {}
            for fid in followers:
                si = assignment.get(fid)
                if si is None:
                    continue
                sn, se, _sd = slot_positions[si]   # slot z ATILIR (== leader_d)
                v_offset = vertical_offsets.get(fid, 0.0)
                target_d = leader_d - v_offset     # vertical offset to drone press
                safe_d   = min(target_d, -abs(min_alt_m))   # min altitude base
                if safe_d != target_d:
                    logger.debug(
                        f"[FollowSafety] f={fid}: target altitude {-target_d:.1f}m, "
                        f"Upgrading to base {min_alt_m:.1f}m."
                    )
                out[fid] = (sn, se, safe_d)
            self._last_formation_targets = out
            return out

        # --- DIAMOND: return body slots to world NED (yaw + flashforward) --
        leader_vx, leader_vy, _ = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        lookahead_time = 3.0
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        vx_body =  leader_vx * cos_yaw + leader_vy * sin_yaw
        vy_body = -leader_vx * sin_yaw + leader_vy * cos_yaw
        fwd_lookahead = vx_body * lookahead_time
        rgt_lookahead = vy_body * lookahead_time

        slot_positions = []
        for (fwd, rgt, down_delta) in body_slots:
            total_fwd = fwd + fwd_lookahead
            total_rgt = rgt + rgt_lookahead
            ned_n = total_fwd * cos_yaw - total_rgt * sin_yaw
            ned_e = total_fwd * sin_yaw + total_rgt * cos_yaw
            ned_d = leader_pos[2] + down_delta
            slot_positions.append((leader_pos[0] + ned_n,
                                   leader_pos[1] + ned_e, ned_d))

        # Single-nearest-slot assignment with hysteresis (uses existing _assign_slots; greedy + hysteresis and
        # lock-safe when called from SINGLE thread — which is now guaranteed).
        assignment = self._assign_slots(
            followers, slot_positions, vertical_offsets, leader_pos[2]
        )

        out = {}
        for fid in followers:
            si = assignment.get(fid)
            if si is None or si >= len(slot_positions):
                continue
            sn, se, sd = slot_positions[si]
            safe_d = min(sd, -abs(min_alt_m))   # never descend below minimum altitude
            out[fid] = (sn, se, safe_d)
        self._last_formation_targets = out
        return out

    def start_formation_following(self, leader_id, formation, offset, vertical_offsets,
                               slot_reassign_threshold_m: float = 5.0, follower_min_alt_m: float = 10.0):
        logger.info(
            f"Formation tracking starts -> leader={leader_id}, "
            f"formation={formation.name}, offset={offset}, "
            f"slot_threshold={slot_reassign_threshold_m:.1f}m"
        )
        log_event(
            "formation_start", leader_id=leader_id, formation=formation.name,
            offset=offset, slot_reassign_threshold_m=slot_reassign_threshold_m,
        )
        self._slot_assignments = {}
        self._slot_reassign_threshold_m = slot_reassign_threshold_m
        self._current_formation = formation.name
        self._current_offset    = offset
        self.follow_enable.set()
        # ONE admin thread (there used to be one thread per follower).
        t = threading.Thread(
            target=self._formation_manager_thread,
            args=(formation, offset, vertical_offsets),
            kwargs={"min_alt_m": follower_min_alt_m},
            name="FormationManager",
            daemon=True,
        )
        t.start()
        self.threads.append(t)

    def pause_formation_following(self):
        """Pauses all follower threads without stopping them."""
        logger.info("Formation following STOPPED.")
        log_event("formation_pause")
        self.follow_enable.clear()

    # ------------------------------------------------------------------ Leader slot controller thread
    # ------------------------------------------------------------------
    def _leader_slot_reader_thread(
        self,
        redis_client,
        leader_id:          int,
        stale_timeout:       float = 2.0,
        # Publishing simple_guided_follow 20 Hz; On 10 Hz every second setpoint was being thrown (logs
        # 2026-07-30). Match rate -> each slot goes to the vehicle.
        send_rate_hz:        float = 20.0,
        min_alt_m:           float = 10.0,
        loop_start_wp_index: int   = 2,
        loop_arrival_dist_m: float = 12.0,
        wp_speed_mps:        float = 10.0,
        ):
        import json
        REDIS_KEY   = "leader_slot_ned"
        sleep_dt    = 1.0 / max(send_rate_hz, 1.0)
        last_warn   = 0.0
        guided_mode = True   # track the mode most recently requested by this controller

        # NED Z negative-up: -min_alt_m is the ceiling value for Z (most negative = highest)
        min_z_ned = -abs(min_alt_m)

        logger.info(
            f"Leader slot controller started (leader={leader_id}, key={REDIS_KEY}, "
            f"min_alt={min_alt_m:.1f}m, loop_start_wp={loop_start_wp_index}, "
            f"arrival_dist={loop_arrival_dist_m:.1f}m)"
        )

        def drain_mission_current():
            """Returns the latest seq MISSION_CURRENT, None if there is nothing in the queue."""
            q = self._mission_current_queues[leader_id]
            last_seq = None
            while True:
                try:
                    msg = q.get_nowait()
                    last_seq = int(msg.seq)
                except queue.Empty:
                    break
            return last_seq

        # Reset mission-loop state whenever AUTO mode is entered or re-entered.
        mission_state = {
            "mission_count":     -1,
            "last_wp_index":     -1,
            "last_wp_ned":       None,
            "current_seq":       -1,
            "awaiting_arrival":  False,
            "parked_since":      None,   # The moment it started to stay stable on the last WP
        }

        def enter_auto():
            """Reflects the wait_for_endurance() setup: number of tasks + get last WP coordinates."""
            self.set_mode(leader_id, "AUTO")
            mission_state["mission_count"] = self._get_mission_count(leader_id)
            mission_state["parked_since"] = None
            if mission_state["mission_count"] <= 0:
                # In AUTO mode, if there is no mission, the vehicle does NOTHING (or flies an old, unknown mission).
                # This is the classic cause of the "went somewhere I don't know" situation — beep loudly.
                logger.error(
                    f"[LeaderSlot] CRITICAL: NO AUTO task on leader {leader_id} "
                    f"(mission_count={mission_state['mission_count']}). Scan route "
                    f"not loaded! The leader will wait in AUTO. with Mission Planner "
                    f"Put the scanning mission on ALL drones."
                )
            if mission_state["mission_count"] > 0:
                mission_state["last_wp_index"] = mission_state["mission_count"] - 1
                origin = self._ned_origin
                wp = self._get_last_waypoint_global(leader_id, mission_state["mission_count"])
                if wp is not None and origin is not None:
                    wp_lat, wp_lon, wp_alt = wp
                    mission_state["last_wp_ned"] = latlon_to_ned(
                        wp_lat, wp_lon, wp_alt, *origin
                    )
            mission_state["awaiting_arrival"] = False
            drain_mission_current()

        while not self.stop_threads.is_set():
            try:
                # CLOSE: send NO MORE commands to the leader if descent/closing has started. Otherwise the following
                # "revert to AUTO" logic will crush the RTL.
                if self._shutting_down.is_set():
                    logger.info(
                        f"[LeaderSlot] Closing detected — leader {leader_id} "
                        f"control is released (RTL/landing free)."
                    )
                    return

                # IF THE LEADER IS ATTACKING, RELEASE slot control. Otherwise, this thread sends the setpoint
                # leader_slot_ned at 20 Hz and the attack thread sends the intersection point at 10 Hz to the SAME
                # vehicle; the vehicle oscillates between the two commands and never hits the target. When the attack
                # is over (_release_attacker -> _committed_attacker = None), this cycle automatically continues where
                # it left off; Since guided_mode is pulled into False, GUIDED + default speed is re-established.
                if self._committed_attacker == leader_id:
                    if guided_mode:
                        logger.info(
                            f"[LeaderSlot] Lider {leader_id} SALDIRIDA — slot "
                            f"control is left to the attack thread."
                        )
                        guided_mode = False
                    time.sleep(sleep_dt)
                    continue

                raw = redis_client.get(REDIS_KEY)
                now = time.monotonic()

                if raw:
                    data = json.loads(raw)
                    # FIX: If 'ts' is missing/invalid the data is considered OLD. In the past, age=now was calculated with
                    # data.get("ts", 0.0); this could silently lead to misbehavior.
                    _ts = data.get("ts", None)
                    try:
                        age = float("inf") if _ts is None else (now - float(_ts))
                    except (TypeError, ValueError):
                        age = float("inf")

                    if age < stale_timeout:
                        # --- Fresh prediction: Guarantee GUIDED and send slot ---
                        if not guided_mode:
                            logger.info(
                                f"[LeaderSlot] simple_guided_follow data returned "
                                f"— moving leader {leader_id} to GUIDED mode."
                            )
                            self.set_mode(leader_id, "GUIDED")
                            # Problem 2: WP_SPEED (DO_CHANGE_SPEED) sent in AUTO is moved to GUIDED. Returning to GUIDED, the
                            # leader should fly at the vehicle's OWN ArduPilot default speed (with ceiling MAX_SPEED).
                            self._set_speed(leader_id, self._default_speed_mps(leader_id))
                            guided_mode = True
                            drain_mission_current()

                        raw_z     = float(data["z"])
                        clamped_z = min(raw_z, min_z_ned)
                        if clamped_z != raw_z:
                            logger.debug(
                                f"[LeaderSlot] Z floor applied: {raw_z:.2f} → {clamped_z:.2f} "
                                f"(min_alt={min_alt_m:.1f}m)"
                            )
                        # FIX: Redis slot (leader_slot_ned) is produced by simple_guided_follow in COMMON origin frame, but
                        # MAV_FRAME_LOCAL_NED (leader's own EKF frame). goto_shared_ned sends the equivalent global point
                        # instead.
                        #
                        # FEED-FORWARD: pass vx/vy/vz + ax/ay/az + yaw + type_mask fields EXACTLY as written by the publisher.
                        # So the --no-position-only mode actually produces a velocity/acceleration feedforward profile. For
                        # legacy/az domain payloads, defaults fall to POSITION_ONLY_MASK -> just like legacy location-only
                        # stream. (Position z has already been clipped to the min-altitude base with 'clamped_z'.)
                        self.goto_shared_ned(
                            leader_id,
                            data["x"], data["y"], clamped_z,
                            vx=float(data.get("vx", 0.0)),
                            vy=float(data.get("vy", 0.0)),
                            vz=float(data.get("vz", 0.0)),
                            ax=float(data.get("ax", 0.0)),
                            ay=float(data.get("ay", 0.0)),
                            az=float(data.get("az", 0.0)),
                            yaw=float(data.get("yaw", 0.0)),
                            yaw_rate=float(data.get("yaw_rate", 0.0)),
                            type_mask=int(data.get("type_mask", POSITION_ONLY_MASK)),
                        )

                    else:
                        # --- Stale data: guarantee AUTO and run loop logic ---
                        if guided_mode:
                            logger.warning(
                                f"[ LeaderSlot ] Data stale ( {age:.1f} s) "
                                f"— leader {leader_id} is put into AUTO mode."
                            )
                            guided_mode = False
                            enter_auto()
                        else:
                            # If the leader fell out of AUTO (e.g. mission ended, went to LOITER, or something else changed the
                            # mode) AUTO AGAIN. User request: when there is no target, the leader should fly his own route
                            # forever.
                            _m = self.drone_modes.get(leader_id, "")
                            if _m and _m != "AUTO":
                                logger.warning(
                                    f"[LeaderSlot] Leader {leader_id} is not in AUTO "
                                    f"(mode={_m}) — Restoring to AUTO."
                                )
                                enter_auto()

                        if now - last_warn > 5.0:
                            logger.warning(f"[LeaderSlot] Still waiting... (staleness={age:.1f}s)")
                            last_warn = now

                        self._run_mission_loop_step(
                            leader_id, drain_mission_current, mission_state,
                            loop_arrival_dist_m, loop_start_wp_index, wp_speed_mps
                        )

                else:
                    # --- Switch Redis does not have it yet: ensure AUTO and run loop logic ---
                    if guided_mode:
                        logger.warning(
                            f"[LeaderSlot] '{REDIS_KEY}' does not exist yet "
                            f"— leader {leader_id} is put into AUTO mode."
                        )
                        guided_mode = False
                        enter_auto()
                    else:
                        _m = self.drone_modes.get(leader_id, "")
                        if _m and _m != "AUTO":
                            logger.warning(
                                f"[LeaderSlot] Leader {leader_id} is not in AUTO "
                                f"(mode={_m}) — Restoring to AUTO."
                            )
                            enter_auto()

                    if now - last_warn > 5.0:
                        logger.warning(f"[LeaderSlot] Expecting key '{REDIS_KEY}' Redis...")
                        last_warn = now

                    self._run_mission_loop_step(
                        leader_id, drain_mission_current, mission_state,
                        loop_arrival_dist_m, loop_start_wp_index, wp_speed_mps
                    )

            except Exception:
                logger.exception("Leader slot controller error.")

            time.sleep(sleep_dt)

    def _set_speed(self, target_id: int, speed_mps: float, allow_over_max: bool = False):
        """
        Sets the horizontal flight speed (speed CEILING) of the drone by sending
        MAV_CMD_DO_CHANGE_SPEED. No answer expected — fire-and-forget.

        Global MAX_SPEED ceiling: if a positive speed is desired and allow_over_max is False, the
        value is limited to MAX_SPEED. Only attack/kill mode can exceed this ceiling by passing
        allow_over_max = True.
        """
        if speed_mps > 0 and not allow_over_max and speed_mps > MAX_SPEED:
            logger.debug(f"[ SpeedLimit ] Drone {target_id} : requested {speed_mps:.1f} m/s "
                         f"-> {MAX_SPEED:.1f} clipped to m/s ceiling.")
            speed_mps = MAX_SPEED
        self.master.mav.command_long_send(
            target_id, 0,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,                    # param1: speed type (1 = ground speed)
            float(speed_mps),     # param2: desired speed [m/s]
            -1,                   # param3: throttle (-1 = replacement)
            0, 0, 0, 0,
        )

    def set_param(self, target_id: int, name: str, value: float):
        """
        Sets the parameter of a drone (PARAM_SET). No answer expected.

        This is CRITICAL for the attacker: The peak achievable speed at ArduPilot while traveling to
        position target GUIDED is limited to v_peak = sqrt(WPNAV_ACCEL * distance). DO_CHANGE_SPEED
        only raises the roof; the main bottleneck is WPNAV_ACCEL.
        """
        try:
            self.master.mav.param_set_send(
                target_id, 1,
                name.encode("utf-8") if isinstance(name, str) else name,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            time.sleep(0.02)
        except Exception:
            logger.exception(f"set_param error (tgt={target_id}, {name}={value})")

    def get_param(self, target_id: int, name: str, timeout: float = 3.0):
        """
        Reads the parameter of a drone (PARAM_REQUEST_READ -> PARAM_VALUE). Returns the value as
        float; In case of timeout, None is returned.

        Since outerpatcher owns all packet reads, the response is retrieved from queue PARAM_VALUE
        (recv_match is NOT used directly).
        """
        q = self._param_value_queues.get(target_id)
        # Clear out the old PARAM_VALUEs so we don't get an answer from a previous read.
        if q is not None:
            while True:
                try: q.get_nowait()
                except queue.Empty: break

        pname = name.encode("utf-8") if isinstance(name, str) else name
        deadline  = time.time() + timeout
        last_send = 0.0
        while time.time() < deadline:
            now = time.time()
            # Here's the periodic replay (against packet loss).
            if now - last_send >= 0.5:
                self.master.mav.param_request_read_send(target_id, 0, pname, -1)
                last_send = now
            if q is None:
                time.sleep(0.1); continue
            try:
                msg = q.get(timeout=0.2)
            except queue.Empty:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode("utf-8", "ignore")
            got = got.rstrip("\x00")
            if got == name:
                return float(msg.param_value)
            # Response for another parameter — ignore, it will be requested again.

        logger.warning(f"[get_param] Drone {target_id}: Could not read '{name}' (timeout={timeout:.0f}s).")
        return None

    def _effective_cruise_mps(self) -> float:
        """Effective cruise speed [m/s] = min(NAV_SPEED, MAX_SPEED). FIXED based."""
        return min(NAV_SPEED, MAX_SPEED)

    def _default_speed_mps(self, drone_id: int) -> float:
        """
        Target WPNav speed for formation/cruise [m/s] = min(NAV_SPEED, MAX_SPEED).

        Derived from CONSTANTS (NOT read from the instrument); so it is not affected by a corrupted
        WPNAV_SPEED value written to the vehicle from a previous low-MAX run.
        """
        return self._effective_cruise_mps()

    def apply_global_speed_limit(self, drone_ids, param_timeout: float = 3.0):
        """
        adjusts the cruise speed and applies the global ceiling MAX_SPEED. Attack/kill mode is
        exempt from this limit (separate high profile).
        """
        cruise_mps = self._effective_cruise_mps()
        cruise_cms = cruise_mps * 100.0
        logger.info(
            f"[SpeedLimit] Cruising speed applied: WPNAV_SPEED={cruise_mps:.1f} m/s "
            f"(NAV_SPEED={NAV_SPEED:.1f}, MAX_SPEED={MAX_SPEED:.1f}), "
            f"WPNAV_ACCEL={NAV_WPNAV_ACCEL_CMS/100.0:.1f} m/s^2"
        )
        for did in drone_ids:
            self.set_param(did, "WPNAV_SPEED", cruise_cms)
            self.set_param(did, "WPNAV_ACCEL", NAV_WPNAV_ACCEL_CMS)
            log_event("speed_limit_applied", drone_id=did,
                      wpnav_speed_cms=cruise_cms,
                      wpnav_accel_cms=NAV_WPNAV_ACCEL_CMS,
                      nav_speed_mps=NAV_SPEED, max_speed_mps=MAX_SPEED)

    def _apply_attack_dynamics(self, drone_id: int):
        """The attacker switches the drone to a high speed/acceleration profile (MAX_SPEED exempt)."""
        self.set_param(drone_id, "WPNAV_SPEED", ATTACK_WPNAV_SPEED_CMS)
        self.set_param(drone_id, "WPNAV_ACCEL", ATTACK_WPNAV_ACCEL_CMS)
        # Attack/kill mode EXEMPT from global cap — allow_over_max=True.
        self._set_speed(drone_id, ATTACK_SPEED_MPS, allow_over_max=True)

    def _restore_default_dynamics(self, drone_id: int):
        """
        Problem 3 — reverts to normal cruise speed/acceleration profile as attacker returns to
        formation; so it flies at the SAME speed as the rest of the flock.

        These are the SAME fixed-base values ​​as Profile apply_global_speed_limit:
        WPNAV_SPEED = min(NAV_SPEED, MAX_SPEED), WPNAV_ACCEL = NAV_WPNAV_ACCEL_CMS.
        """
        cruise_mps = self._effective_cruise_mps()
        self.set_param(drone_id, "WPNAV_SPEED", cruise_mps * 100.0)
        self.set_param(drone_id, "WPNAV_ACCEL", NAV_WPNAV_ACCEL_CMS)
        # Set the operating-instant DO_CHANGE_SPEED roof to cruising speed.
        self._set_speed(drone_id, cruise_mps)

    def _run_mission_loop_step(
        self, leader_id, drain_current_fn, state, arrival_dist_m, start_wp_index, wp_speed_mps
        ):
        """
        An iteration of the proven endurance-type pattern; It is called at every tick when the
        leader is in AUTO and there is no fresh slot giveaway.
        """
        if wp_speed_mps > 0:
            if self.drone_modes.get(leader_id, "") == "RTL":
                self._set_speed(leader_id, RTL_SPEED)
            else:
                self._set_speed(leader_id, wp_speed_mps)

        last_wp_index = state.get("last_wp_index", -1)
        if last_wp_index < 0:
            return

        seq = drain_current_fn()
        if seq is not None:
            state["current_seq"] = seq
            if seq >= last_wp_index:
                state["awaiting_arrival"] = True

        if not state.get("awaiting_arrival"):
            return

        last_wp_ned = state.get("last_wp_ned")
        if last_wp_ned is None:
            logger.warning(
                f"[MissionRestart] Last WP coordinate unknown — "
                f"distance control is skipped, the cycle is started directly."
            )
            self._restart_mission(leader_id, seq=start_wp_index)
            state["awaiting_arrival"] = False
            return

        with self.lock:
            leader_pos = self.drone_positions.get(leader_id)

        if leader_pos is None:
            return

        dn = leader_pos[0] - last_wp_ned[0]
        de = leader_pos[1] - last_wp_ned[1]
        dist = math.sqrt(dn * dn + de * de)

        logger.debug(f"[ MissionRestart ] Distance to last WP: {dist:.1f} m")

        if dist < arrival_dist_m:
            logger.info(
                f"[MissionRestart] Last WP reached (distance={dist:.1f}m < {arrival_dist_m:.1f}m) "
                f"— mission restarting from WP {start_wp_index}."
            )
            self._restart_mission(leader_id, seq=start_wp_index)
            state["awaiting_arrival"] = False
            state["parked_since"] = None
            return

        # --- PARKING-SAFETY (new) ---------------------------------------- If the distance threshold is kept
        # too narrow (e.g. 1 m) the vehicle stops within 1-2 m of the last WP and `dist < arrival_dist_m` is
        # NEVER satisfied; the mission never restarts and the leader hangs there forever (this is exactly what
        # was seen in the log). This is safety: if the leader actually STOPPED (speed ~0) and this lasted
        # several seconds, restart the mission regardless of distance.
        with self.lock:
            vel = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        speed = math.sqrt(vel[0]**2 + vel[1]**2)
        now_m = time.monotonic()
        if speed < 1.0:
            if state.get("parked_since") is None:
                state["parked_since"] = now_m
            elif now_m - state["parked_since"] > 5.0:
                logger.warning(
                    f"[MissionRestart] Leader STOPPED near last WP "
                    f"(distance={dist:.1f}m, speed={speed:.1f}m/s, 5s+) — "
                    f"task restarting from WP {start_wp_index}."
                )
                self._restart_mission(leader_id, seq=start_wp_index)
                state["awaiting_arrival"] = False
                state["parked_since"] = None
        else:
            state["parked_since"] = None

    # ------------------------------------------------------------------ Start simple_guided_follow.sh in
    # a new gnome-terminal window ------------------------------------------------------------------
    def launch_guided_follow(
        self,
        script_path: str = "simple_guided_follow.sh",
        redis_host: str  = "localhost",
        redis_port: int  = 6379,
        min_alt_m:  float = 10.0,
        loop_start_wp_index: int   = 2,
        loop_arrival_dist_m: float = 12.0,
        wp_speed_mps: float = 10.0,
        ):
        # abs_path = os.path.abspath(script_path)
        # logger.info(f"Launching simple_guided_follow in new terminal: {abs_path}")
        # try:
        #     self._follow_proc = subprocess.Popen(
        #         [ "gnome-terminal", "--title=simple_guided_follow", "--", "bash", "-c", f"bash {abs_path};
        #         echo '[simple_guided_follow] Done. Enter to close.'; read", ] ) logger.info(f"gnome-terminal
        #         launched (pid={self._follow_proc.pid})") except Exception:
        #         logger.exception("simple_guided_follow gnome-terminal launch error.")

        try:
            import redis as redis_lib
            rc = redis_lib.Redis(host=redis_host, port=redis_port, db=0)
            rc.ping()
            t = threading.Thread(
                target=self._leader_slot_reader_thread,
                args=(rc, self.leader_id),
                kwargs={
                    "min_alt_m": min_alt_m,
                    "loop_start_wp_index": loop_start_wp_index,
                    "loop_arrival_dist_m": loop_arrival_dist_m,
                    "wp_speed_mps": wp_speed_mps,
                },
                name="LeaderSlotCtrl",
                daemon=True,
            )
            t.start()
            self.threads.append(t)
            logger.info(f"Leader slot controller thread started ({redis_host}:{redis_port})")
        except Exception:
            logger.exception("Redis connection failed — failed to initialize leader slot controller.")

    # ------------------------------------------------------------------ Receive origin NED from lead
    # drone and broadcast to Redis ------------------------------------------------------------------
    def fetch_and_publish_ned_origin(self, leader_id: int, redis_client, timeout: float = 10.0):
        """
        Reads origin NED (ground level GPS) from the lead drone's GLOBAL_POSITION_INT via the
        outerpatcher's existing _gpos_queues. Must be called BEFORE takeoff.
        """
        import json
        logger.info(f"Waiting for the NED origin (leader={leader_id}, gpos queue)...")

        q        = self._gpos_queues[leader_id]
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                rel_alt_m = msg.relative_alt / 1000.0
                if rel_alt_m > 2.0:
                    logger.warning(
                        f"[NedOrigin] Drone {leader_id} zaten havada "
                        f"(relative_alt={rel_alt_m:.1f}m) — continuing with existing GPS."
                    )
                home_lat = msg.lat / 1e7
                home_lon = msg.lon / 1e7
                home_alt = msg.alt / 1000.0

                payload = {"lat": home_lat, "lon": home_lon, "alt": home_alt}
                redis_client.set("ned_origin", json.dumps(payload))
                self._ned_origin = (home_lat, home_lon, home_alt)
                logger.info(
                    f"NED written to origin Redis: lat={home_lat:.7f} "
                    f"lon={home_lon:.7f} alt={home_alt:.1f}m "
                    f"(relative_alt={rel_alt_m:.1f}m)"
                )
                return home_lat, home_lon, home_alt

            except queue.Empty:
                pass

        raise RuntimeError(
            f"Failed to retrieve NED origin — from drone {leader_id} in {timeout}s "
            "GLOBAL_POSITION_INT did not arrive. Is Outpatcher working?"
        )


    def wait_for_gps_health(self, drone_id, min_fix=3, min_sats=10, timeout=60.0):
        """
        Waits until the drone reports a healthy GPS fix; It gives an error at timeout. Requires
        GPS_RAW_INT; If it does not flow, it requests it. min_fix=3 -> 3D lock. min_sats -> visible
        satellite threshold.
        """
        logger.info(f"[GPSGate] Drone {drone_id}: Waiting for GPS health "
                    f"(fix>={min_fix}, sats>={min_sats})...")
        self.request_message_interval(drone_id, 'GPS_RAW_INT', 2.0)
        deadline = time.time() + timeout
        last_log = 0.0
        while time.time() < deadline:
            # GPS_RAW_INT is not kept in a separate queue; It is read opportunistically directly from the master
            # with a short recv.
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1.0)
            if msg and msg.get_srcSystem() == drone_id:
                fix  = int(msg.fix_type)
                sats = int(msg.satellites_visible)
                if time.time() - last_log > 2.0:
                    logger.info(f"[GPSGate] Drone {drone_id}: fix={fix} sats={sats}")
                    last_log = time.time()
                if fix >= min_fix and sats >= min_sats:
                    logger.info(f"[GPSGate] Drone {drone_id}: GPS OK (fix={fix}, sats={sats}).")
                    log_event("gps_ok", drone_id=drone_id, fix=fix, sats=sats)
                    return True
        log_event("gps_gate_timeout", drone_id=drone_id)
        raise RuntimeError(
            f"[GPSGate] Drone {drone_id}: healthy GPS in {timeout:.0f}s "
            f"Failed to receive (fix>={min_fix}, sats>={min_sats}) — departure CANCELLED."
        )

    def gps_gate_all(self, drone_ids, min_fix=3, min_sats=10, timeout=60.0):
        """Checks the health of each drone GPS. If someone doesn't pass, the mission is cancelled."""
        for did in drone_ids:
            self.wait_for_gps_health(did, min_fix=min_fix, min_sats=min_sats, timeout=timeout)
        logger.info("[GPSGate] All drones are ready with healthy GPS fix.")


    _MSG_IDS = {
        'GPS_RAW_INT': 24, 'RAW_IMU': 27, 'SERVO_OUTPUT_RAW': 36,
        'SYS_STATUS': 1, 'VIBRATION': 241, 'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30, 'VFR_HUD': 74,
    }

    def request_message_interval(self, drone_id, msg_name, rate_hz):
        """Asks the drone to stream message msg_name at speed rate_hz (0 = close)."""
        msg_id = self._MSG_IDS.get(msg_name)
        if msg_id is None:
            logger.warning(f"request_message_interval : unknown message '{msg_name}'")
            return
        interval_us = -1 if rate_hz <= 0 else int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            drone_id, 0,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            float(msg_id), float(interval_us), 0, 0, 0, 0, 0,
        )
        time.sleep(0.02)

    def setup_high_rate_telemetry(self, drone_ids):
        """Requests pulse/health message streams on all drones."""
        logger.info("High-speed telemetry streams are desired...")
        for did in drone_ids:
            for msg_name, hz in MSG_RATE_HZ.items():
                self.request_message_interval(did, msg_name, hz)
            # Also ensure that the GLOBAL_POSITION_INT / ATTITUDE flows for the "Hit Detection Flight Record" in
            # the specification are >= 20 Hz.
            self.request_message_interval(did, 'GLOBAL_POSITION_INT', 25.0)
            self.request_message_interval(did, 'ATTITUDE', 25.0)
            self.request_message_interval(did, 'VFR_HUD', 20.0)
        logger.info("Telemetry stream requests sent (verify actual speed on bench).")


    def start_flight_csv_loggers(self, drone_ids, log_dir="logs/flight_csv", rate_hz=25.0):
        import csv as _csv
        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")

        def logger_thread(did):
            path = os.path.join(log_dir, f"drone{did}_{stamp}.csv")
            f = open(path, "w", newline="")
            w = _csv.writer(f)
            w.writerow(["wall_time", "lat", "lon", "alt_amsl", "rel_alt",
                        "vn", "ve", "vd", "accel_g", "roll", "pitch", "yaw",
                        "mode", "batt_v", "batt_a", "armed"])
            dt = 1.0 / max(rate_hz, 1.0)
            rows = 0
            logger.info(f"[FlightCSV] Drone {did} -> {path} @ {rate_hz:.0f}Hz")
            while not self.stop_threads.is_set():
                try:
                    with self.lock:
                        pos  = self.drone_positions.get(did)
                        vel  = self.drone_velocities.get(did, (0, 0, 0))
                        yaw  = self.drone_headings.get(did, 0.0)
                        mode = self.drone_modes.get(did, "")
                        armed = self.drone_armed.get(did, False)
                    # Get the latest attribute/global giveaway opportunistically from the queues.
                    lat = lon = alt = rel = None
                    try:
                        gq = self._gpos_queues[did]
                        gmsg = gq.queue[-1] if len(gq.queue) else None
                        if gmsg is not None:
                            lat = gmsg.lat / 1e7
                            lon = gmsg.lon / 1e7
                            alt = gmsg.alt / 1000.0
                            rel = gmsg.relative_alt / 1000.0
                    except Exception:
                        pass
                    sysh = self._sys_health.get(did) or {}
                    w.writerow([
                        f"{time.time():.3f}",
                        lat, lon, alt, rel,
                        vel[0], vel[1], vel[2],
                        f"{self._last_accel_g.get(did, 0.0):.3f}",
                        "", "", f"{yaw:.4f}",
                        mode, sysh.get("batt_v"), sysh.get("batt_a"),
                        int(bool(armed)),
                    ])
                    rows += 1
                    if rows % 50 == 0:
                        f.flush()
                except Exception:
                    logger.exception(f"[FlightCSV] Error writing Drone {did}.")
                time.sleep(dt)
            f.flush(); f.close()

        for did in drone_ids:
            t = threading.Thread(target=logger_thread, args=(did,),
                                 name=f"FlightCSV-{did}", daemon=True)
            t.start()
            self.threads.append(t)

    # =========================================================== NEW: leader selection handler (summons
    # heartbeat guard on leader loss)
    # ============================================================================
    def enable_leader_failover(self):
        """The leader records the recall so that the loss of the heartbeat triggers the selection."""
        self._on_leader_lost = self._elect_new_leader
        logger.info("Leader failover enabled (selection is made on heartbeat loss).")

    def _elect_new_leader(self, dead_leader, alive_ids):
        """
        SUCCESSION_ORDER makes the next drone that is alive and not in the middle of an attack the
        leader. The old leader (if his link returns) becomes a follower.
        """
        with self._election_lock:
            if self._election_in_progress:
                return
            if self.leader_id != dead_leader:
                return   # someone else has already been chosen
            new_ldr = next_leader(SUCCESSION_ORDER, alive_ids, dead_leader,
                                  skip_ids=self._attack_skip)
            if new_ldr is None:
                logger.error("[Election] No new suitable leaders! The swarm is taken to LOITER.")
                self.follow_enable.clear()
                for did in alive_ids:
                    self.set_mode(did, "LOITER")
                return
            self._election_in_progress = True

        logger.warning(f"[Election] Leader {dead_leader} disappeared -> new leader {new_ldr}")
        log_event("leader_elected", old_leader=dead_leader, new_leader=new_ldr,
                  alive=sorted(alive_ids))

        # The new leader must fly the scanning mission (pre-installed on ALL drones). switch to AUTO; Since
        # the formation manager reads self.leader_id at each tick, it automatically sits around the new
        # leader.
        self.leader_id = new_ldr
        self.set_mode(new_ldr, "AUTO")

        # Send as much RTL to the dead leader as you can (it will only arrive if his link returns).
        self.set_mode(dead_leader, "RTL")

        # Reset slot assignments for clean calculation of new geometry.
        self._slot_assignments = {}
        self.follow_enable.set()
        self._last_leader_pos_time = time.monotonic()

        with self._election_lock:
            self._election_in_progress = False

    # ========================================================== NEW: hit + damage self-check tracker
    # (true telemetry). Detects impact from timed acceleration spike, verifies with vibration, then
    # executes autonomous damage self-check. If it passes, he continues his duty; if it stays, IT WILL
    # LAND WHERE IT IS (dragging the damaged vehicle on site with RTL).
    # ===============================================================================
    def start_impact_monitor(self, drone_ids):
        def monitor(did):
            above_since = None
            logger.info(f"[Impact] Drone {did} izleniyor.")
            while not self.stop_threads.is_set():
                g = self._last_accel_g.get(did, 1.0)
                now = time.monotonic()
                if g >= IMPACT_ACCEL_G:
                    if above_since is None:
                        above_since = now
                    elif (now - above_since) * 1000.0 >= IMPACT_MIN_MS:
                        if not self._impact_flag[did]:
                            logger.warning(f"[Impact] Drone {did} : IMPACT detected "
                                           f"(|a|={g:.1f}g).")
                            log_event("impact_detected", drone_id=did, accel_g=round(g, 2))
                            self._impact_flag[did] = True
                            self._on_impact(did)
                else:
                    above_since = None
                time.sleep(0.02)   # 50 Hz check
        for did in drone_ids:
            t = threading.Thread(target=monitor, args=(did,),
                                 name=f"Impact-{did}", daemon=True)
            t.start()
            self.threads.append(t)

    def _on_impact(self, did):
        """Runs autonomous damage self-check in a separate thread."""
        t = threading.Thread(target=self._damage_self_check, args=(did,),
                             name=f"Damage-{did}", daemon=True)
        t.start()
        self.threads.append(t)

    def _damage_self_check(self, did, hold_s=3.0):
        """
        Completely autonomous, no human intervention. It waits a short time and evaluates whether
        the vehicle is still controllable with real telemetry: * SYS_STATUS sensor health bitmask
        (gyroscope/accelerometer/motor) * battery current reasonableness * SERVO_OUTPUT_RAW
        saturation (damaged propeller fixes the motor on top) If passed, it returns to duty. If he
        stays, he lands where he is.
        """
        logger.info(f"[Damage] Drone {did}: damage self-check ({hold_s:.0f}s)...")
        t0 = time.monotonic()
        sat_hits = 0
        samples = 0
        while time.monotonic() - t0 < hold_s and not self.stop_threads.is_set():
            # Engine saturation control
            try:
                sq = self._servo_queues[did]
                smsg = sq.queue[-1] if len(sq.queue) else None
                if smsg is not None:
                    outs = [getattr(smsg, f"servo{i}_raw", 0) for i in range(1, 5)]
                    if any(o >= 1980 for o in outs if o):   # pinned near max
                        sat_hits += 1
                    samples += 1
            except Exception:
                pass
            time.sleep(0.1)

        sysh = self._sys_health.get(did) or {}
        health = sysh.get("health")
        present = sysh.get("present")
        health_bad = False
        if health is not None and present:
            # If an existing+active sensor reports unhealthy, this is a red flag.
            unhealthy = present & ~health
            if unhealthy:
                health_bad = True

        motor_bad = samples > 0 and (sat_hits / samples) > 0.4

        if health_bad or motor_bad:
            logger.error(f"[Damage] Drone {did}: Considered DAMAGED "
                         f"(health_bad={health_bad}, motor_bad={motor_bad}) -> LAND.")
            log_event("damage_confirmed", drone_id=did,
                      health_bad=health_bad, motor_bad=motor_bad)
            self.set_mode(did, "LAND")
            self._alive_ids.discard(did)
            # If the damaged drone is an attacker, it abandons the mission and passes it on to the next attacker.
            if self._committed_attacker == did:
                self._release_attacker(did, reason="damaged")
                if ENABLE_KILL_STRATEGY:
                    self._commit_next_attacker()
        else:
            logger.info(f"[Damage] Drone {did}: SOLID — continue mission.")
            log_event("damage_cleared", drone_id=did)
            self._impact_flag[did] = False
            # He survived the blow: if he was the attacker, he returns to formation; If the target is still alive,
            # the attack logic may assign another attacker.
            if self._committed_attacker == did:
                self._release_attacker(did, reason="survived_hit")

    def publish_pursuer_state(self, redis_client, drone_id, key):
        """Broadcasts a drone's live NED status for the guidance process to read."""
        with self.lock:
            pos = self.drone_positions.get(drone_id)
            vel = self.drone_velocities.get(drone_id, (0, 0, 0))
        if pos is None:
            return
        payload = {"x": pos[0], "y": pos[1], "z": pos[2],
                   "vx": vel[0], "vy": vel[1], "vz": vel[2],
                   "ts": time.monotonic()}
        try:
            redis_client.set(key, json.dumps(payload))
        except Exception:
            pass

    def start_pursuer_state_publisher(self, redis_client, rate_hz=20.0):
        """
        Broadcasts the leader's live NED status; so that the leader's own guidance process
        (simple_guided_follow --leader-state-key leader_state_ned) can calculate the CPA
        intersection geometry.
        """
        def pub():
            # Broadcasts the live NED status of the LEADER. The attacker no longer needs a state published here —
            # the in-process intercept reads the account directly from self.drone_positions.
            dt = 1.0 / max(rate_hz, 1.0)
            while not self.stop_threads.is_set():
                self.publish_pursuer_state(redis_client, self.leader_id, LEADER_STATE_KEY)
                time.sleep(dt)
        t = threading.Thread(target=pub, name="PursuerStatePub", daemon=True)
        t.start()
        self.threads.append(t)

    def _pick_attacker(self, target_ned=None):
        """
        He chooses a suitable attacker.

        F (collision avoidance): If target_ned is given and ENABLE_TARGET_SIDE_SELECTION is on, it
        selects from among the available candidates the drone whose exit vector is BEST aligned with
        the target direction (i.e. already ON the target SIDE) — thus diverging outwards without
        passing through the swarm. If target_ned is not present or the feature is turned off, it
        uses the sequence ATTACKER_PRIORITY (the old behavior is preserved verbatim).
        """
        now = time.monotonic()

        def _eligible(did):
            # THE LEADER CAN ALSO ATTACK (2026-07-31). It used to be here if did == self.leader_id: return False;
            # In scenario ATTACKER_PRIORITY = [LEADER_ID] this was ALWAYS emptying the candidate list and freezing
            # the _pick_attacker None -> no attack would ever start. While the leader is attacking, slot control
            # is left in the _leader_slot_reader_thread (see _committed_attacker control there), so that two
            # threads do not send a setpoint to the same vehicle at the same time.
            if did in self._grounded_ids:
                return False
            if did not in self._alive_ids:
                return False
            if did == self._committed_attacker:
                return False
            if self._impact_flag.get(did):
                return False
            # Skip drones on cooldown after failed attempt.
            if self._attack_cooldown.get(did, 0.0) > now:
                return False
            return True

        candidates = [d for d in ATTACKER_PRIORITY if _eligible(d)]
        if not candidates:
            return None

        # Priority order (legacy behavior) — if the target is unknown or F is off.
        if target_ned is None or not ENABLE_TARGET_SIDE_SELECTION:
            return candidates[0]

        # Target-side selection: the candidate with the largest dot product of the unit vector from the swarm
        # center to the target and the unit vector from the center to the candidate drone.
        with self.lock:
            positions = {d: p for d, p in self.drone_positions.items() if p is not None}
        others = [p for d, p in positions.items() if d not in self._grounded_ids]
        if not others:
            return candidates[0]
        cn = sum(p[0] for p in others) / len(others)
        ce = sum(p[1] for p in others) / len(others)
        tdx, tdy = target_ned[0] - cn, target_ned[1] - ce
        tnorm = math.hypot(tdx, tdy)
        if tnorm < 1e-3:
            return candidates[0]
        tux, tuy = tdx / tnorm, tdy / tnorm

        best, best_score = None, -2.0
        for did in candidates:
            p = positions.get(did)
            if p is None:
                continue
            dx, dy = p[0] - cn, p[1] - ce
            dnorm = math.hypot(dx, dy)
            score = (dx * tux + dy * tuy) / dnorm if dnorm > 1e-3 else 0.0
            if score > best_score:
                best, best_score = did, score
        return best if best is not None else candidates[0]

    def commit_attacker(self, drone_id, script_path=None, redis_host="localhost",
                        redis_port=6379, min_alt_m=10.0):
        """
        Throws a follower into the attack — WITHOUT OPENING a SECOND homing window.

        Correction from previous design: now 2. a simple_guided_follow process is not started. The
        target track is already at Redis ('dogru_rakip_telemetry' is being updated by the single
        working controller/guidance chain). This method starts a lightweight in-process intersection
        thread: * reads the SAME target trace from Redis, * reads the live state of THIS attacker
        NED (self.drone_positions), * calculates the actual intersection with the collision
        triangle, * redirects the attacker directly to goto_shared_ned. No second terminal, no
        second IMM, no attacker_slot_ned round trip.
        """
        if self._committed_attacker is not None:
            logger.warning(f"[Attack] There is already an attacker ({self._committed_attacker}).")
            return
        logger.warning(f"[Attack] Drone {drone_id} was assigned to ATTACK (in-process intercept, "
                       f"max speed {ATTACK_SPEED_MPS:.0f} m/s).")
        log_event("attacker_committed", drone_id=drone_id)
        self._committed_attacker = drone_id
        self._attack_skip.add(drone_id)   # Leader selection should not promote an attacker
        self.set_mode(drone_id, "GUIDED")
        # Apply full attack speed/acceleration profile NOW. Departure from the swarm is now done at FULL SPEED
        # (steering around the swarm by deflecting the aim point out/up — see _egress_deflect); so there is no
        # need to delay the speed. (JUST DO_CHANGE_SPEED is NOT enough: peak speed is limited to
        # sqrt(WPNAV_ACCEL * distance) as S-curve braking at target; so _apply_attack_dynamics also increases
        # WPNAV_ACCEL/WPNAV_SPEED.)
        self._apply_attack_dynamics(drone_id)

        # After transition, the yaw lock writes WP_YAW_BEHAVIOR=0. Read its previous value during approach,
        # because a 3 s get_param wait at the handoff instant is unacceptable. Run the blocking get_param in a
        # separate thread to avoid delaying engagement.
        if ENABLE_ATTACK_YAW_LOCK:
            self._cache_wp_yaw_behavior(drone_id)

        rc = redis.Redis(host=redis_host, port=redis_port, db=0)
        t = threading.Thread(target=self._attacker_intercept_thread,
                             args=(rc, drone_id, min_alt_m),
                             name=f"AttackerIntercept-{drone_id}", daemon=True)
        t.start()
        self.threads.append(t)

    def _attacker_intercept_thread(self, redis_client, drone_id, min_alt_m,
                                   send_rate_hz=10.0, target_key="dogru_rakip_telemetry"):
        """
        In-process interception guidance for a single assigned attacker.
        """
        dt = 1.0 / max(send_rate_hz, 1.0)
        min_z = -abs(min_alt_m)
        origin = self._ned_origin
        if origin is None:
            logger.error("[Attack] NED no origin - cannot start intercept.")
            self._release_attacker(drone_id, reason="no_origin")
            return

        prev_tp       = None
        prev_meas_t   = None
        prev_ctrl_ts  = None
        tgt_vel       = (0.0, 0.0, 0.0)
        best_rng      = float("inf")   # The closest range reached in this attack
        miss_armed    = False          # When you get close enough it becomes True
        pass_armed    = False          # entered terminal range -> waiting for passage
        t_commit      = time.monotonic()
        last_log      = 0.0

        # Aggressive altitude ceiling (for aim clamp + egress deflection).
        max_alt_m = getattr(self, "_attacker_max_alt", config.MAX_ALT_M)

        # Fix the ramp of the live altitude offset at the beginning of this attack: the singleton reader is
        # not polled BETWEEN attacks, so if we started with the old _applied/_last_t values ​​the first
        # seconds of the new attack would be aimed at a stale altitude.
        ALT_OFFSET.reset()
        alt_offset_m = ALT_OFFSET.value()
        alt_clipped_since_log = False
        logger.info(f"[Attack] d{drone_id} altitude offset {alt_offset_m:+.2f} m "
                    f"(live: guidance_config.ALT_OFFSET_M)")

        while not self.stop_threads.is_set() and self._committed_attacker == drone_id:
            try:
                if self._shutting_down.is_set():
                    logger.info(f"[Attack] Closure detected — d{drone_id} attack aborted.")
                    self._release_attacker(drone_id, reason="shutdown")
                    return

                # ----exact timeout --------------------------------------
                elapsed = time.monotonic() - t_commit
                if elapsed > ATTACK_TIMEOUT_S:
                    logger.warning(f"[Attack] d{drone_id} in {ATTACK_TIMEOUT_S:.0f}s "
                                   f"missed -> returning to follow mode.")
                    self._release_attacker(drone_id, reason="timeout")
                    return

                raw = redis_client.get(target_key)
                if not raw:
                    time.sleep(dt); continue
                tj = json.loads(raw)
                if isinstance(tj, dict) and tj.get("valid") is False:
                    time.sleep(dt); continue
                if "lat" not in tj or "lon" not in tj:
                    time.sleep(dt); continue

                # ----change target to common NED --------------------------------
                t_alt_rel = float(tj.get("alt", 0.0))
                tn, te, td = latlon_to_ned(
                    float(tj["lat"]), float(tj["lon"]),
                    origin[2] + t_alt_rel, *origin)
                tp = (tn, te, td)
                now = time.monotonic()

                ctrl_ts = tj.get("ctrl_ts")
                is_new = False
                meas_dt = None
                if ctrl_ts is not None:
                    ctrl_ts = float(ctrl_ts)
                    if prev_ctrl_ts is None or ctrl_ts > prev_ctrl_ts:
                        is_new = True
                        if prev_ctrl_ts is not None:
                            meas_dt = ctrl_ts - prev_ctrl_ts
                else:
                    if prev_tp is None or (abs(tp[0]-prev_tp[0]) +
                                           abs(tp[1]-prev_tp[1]) +
                                           abs(tp[2]-prev_tp[2])) > 1e-6:
                        is_new = True
                        if prev_meas_t is not None:
                            meas_dt = now - prev_meas_t

                if is_new:
                    if prev_tp is not None and meas_dt and meas_dt > 1e-3:
                        raw_v = ((tp[0]-prev_tp[0])/meas_dt,
                                 (tp[1]-prev_tp[1])/meas_dt,
                                 (tp[2]-prev_tp[2])/meas_dt)
                        spd = math.sqrt(raw_v[0]**2 + raw_v[1]**2 + raw_v[2]**2)
                        if spd <= ATTACK_MAX_TGT_SPD:      # reject splashes
                            a_lpf = ATTACK_VEL_LPF
                            tgt_vel = (a_lpf*raw_v[0] + (1-a_lpf)*tgt_vel[0],
                                       a_lpf*raw_v[1] + (1-a_lpf)*tgt_vel[1],
                                       a_lpf*raw_v[2] + (1-a_lpf)*tgt_vel[2])
                    prev_tp      = tp
                    prev_meas_t  = now
                    prev_ctrl_ts = ctrl_ts if ctrl_ts is not None else prev_ctrl_ts

                # ---- attacker's own situation -------------------------------
                with self.lock:
                    ap = self.drone_positions.get(drone_id)
                    av = self.drone_velocities.get(drone_id, (0.0, 0.0, 0.0))
                if ap is None:
                    time.sleep(dt); continue

                rel = (tp[0]-ap[0], tp[1]-ap[1], tp[2]-ap[2])
                rng = math.sqrt(rel[0]**2 + rel[1]**2 + rel[2]**2)

                # Fresh target NED for _pick_attacker with yaw lock monitor.
                self._last_target_ned = tp

                # ---- PASS DETECTION + yaw lock ------------------------------ Range rate of change (range-rate)
                # ANALYTICAL is calculated: d(rng)/dt = unit_bearing · (target_speed - own_speed) We close when the
                # sign is negative, turning positive = nearest pass instant (drone cut the target plane). RANGE
                # DIFFERENCE has no delay compared to receiving and is cleaner against GPS noise — this is important
                # to be able to set the lock at the exact moment.
                if ENABLE_ATTACK_YAW_LOCK and rng > 1e-3:
                    if rng <= ATTACK_TERMINAL_M:
                        pass_armed = True
                    if pass_armed:
                        rr = ((rel[0]*(tgt_vel[0]-av[0]) +
                               rel[1]*(tgt_vel[1]-av[1]) +
                               rel[2]*(tgt_vel[2]-av[2])) / rng)
                        if rr > 0.0:
                            self._lock_yaw(drone_id,
                                           reason=f"target_pass_rng_{rng:.0f}m")
                            # Only if the lock is ACTUALLY set, set-once-drop your flag; If the head angle is not known at that
                            # moment, it is tried again in subsequent clicks.
                            if self._locked_yaw(drone_id) is not None:
                                pass_armed = False

                # ---- iptal kontrolleri --------------------------------------
                if rng > ATTACK_ABORT_MAX_M:
                    logger.warning(f"[Attack] d{drone_id} target is too far away "
                                   f"({rng:.0f}m > {ATTACK_ABORT_MAX_M:.0f}m) -> follow mode.")
                    self._release_attacker(drone_id, reason="target_ran_away")
                    return

                if rng < best_rng:
                    best_rng = rng
                if best_rng <= ATTACK_ABORT_ARM_M:
                    miss_armed = True
                if miss_armed and rng > best_rng + ATTACK_ABORT_OPEN_M:
                    logger.warning(f"[Attack] d{drone_id} SCA: range opening "
                                   f"( {rng:.0f} m > best {best_rng:.0f} m + "
                                   f"{ATTACK_ABORT_OPEN_M:.0f}m) -> follow mode.")
                    self._release_attacker(drone_id, reason="missed_range_opening")
                    return

                # ---- steering ---------------------------------------------------
                atk_spd = ATTACK_SPEED_MPS   # maximum speed is commanded throughout the attack

                if rng <= ATTACK_TERMINAL_M:
                    # Terminal phase: at very short range the intersection solution is poorly conditioned; Aim with short
                    # steady foresight just in front of the target.
                    ip = (tp[0] + tgt_vel[0]*0.3,
                          tp[1] + tgt_vel[1]*0.3,
                          tp[2] + tgt_vel[2]*0.3)
                    phase = "TERM"
                    t_int = rng / max(atk_spd, 1.0)
                else:
                    t_int = solve_intercept_time(rel, tgt_vel, atk_spd)
                    if t_int is None:
                        # There is no intersection solution (the target escapes us in this geometry). Aim directly at the
                        # target and try to cover it; If hopeless the miss detector/timeout will release.
                        ip = tp
                        phase = "PURSUE"
                        t_int = rng / max(atk_spd, 1.0)
                    else:
                        ip = (tp[0] + tgt_vel[0]*t_int,
                              tp[1] + tgt_vel[1]*t_int,
                              tp[2] + tgt_vel[2]*t_int)
                        phase = "INT"

                # --- Extend April point BEYOND intersection ---
                ex = ip[0] - ap[0]; ey = ip[1] - ap[1]; ez = ip[2] - ap[2]
                enorm = math.sqrt(ex*ex + ey*ey + ez*ez)
                if enorm > 1e-3:
                    ux, uy, uz = ex/enorm, ey/enorm, ez/enorm
                    aim = (ip[0] + ux*ATTACK_OVERSHOOT_M,
                           ip[1] + uy*ATTACK_OVERSHOOT_M,
                           ip[2] + uz*ATTACK_OVERSHOOT_M)
                else:
                    aim = ip

                # --- Leaving the swarm at FULL SPEED (egress) — anti-collision (#3) --- While in the swarm, deflect the
                # FAR aim point out+up; S-curve no braking as April stays away -> full speed maintained, orbit goes
                # AROUND the swarm. When clear, the deviation is reset.
                deflected = False
                if ENABLE_ATTACK_EGRESS:
                    aim, deflected = self._egress_deflect(drone_id, ap, aim)

                # Altitude envelope — Limit BOTH sides (NED z = -altitude): base: do not descend below min altitude
                # (legacy behavior), ceiling: do not ascend above max altitude (config/controller.json). LIVE altitude
                # offset (guidance_config.ALT_OFFSET_M): because the vehicle reads its altitude ~5 m HIGHER than it
                # actually is, it actually sits that LOWER at each commanded altitude -> without offset the attacker
                # passes 5 m BELOW the target. It is applied BEFORE envelope clippings so that the floor/ceiling
                # boundaries face the ACTUAL command. Editing and saving guidance_config.py during flight takes effect
                # within ~0.5 s; The change is ramped with ALT_OFFSET_SLEW_MPS.
                alt_offset_m = ALT_OFFSET.value(now)
                ceil_z = -(max_alt_m - EGRESS_CEIL_PAD_M)   # Can't be more negative
                aim_z_wanted = aim[2] - alt_offset_m  # NED z down-positive: minus = up
                aim_z = min(aim_z_wanted, min_z)   # base
                aim_z = max(aim_z, ceil_z)   # ceiling
                # ONLY CEILING clipping is reported: IF aim_z is GREATER than desired (large = low in NED) the ceiling
                # has swallowed the offset and the attacker again passes UNDER the target — he must not remain silent.
                # Base clipping pushes the tool UP than intended; this is the safe side and does not burn the flag
                # (otherwise the flag would constantly burn and lose its meaning while the target was flying under
                # min_alt).
                if alt_offset_m > 0.0 and (aim_z - aim_z_wanted) > 0.05:
                    alt_clipped_since_log = True

                # --- Command send ------------------------------------------ If yaw is locked, the rotated head angle
                # is EXPRESSLY sent on every command (mask uses field yaw); If there is no lock, yaw is ignored and
                # the behavior is exactly the same as before.
                lock_yaw = self._locked_yaw(drone_id)
                if self.is_in_guided_mode(drone_id):
                    if (ENABLE_VELOCITY_TERMINAL and phase == "TERM"
                            and not deflected):
                        # KILL / terminal phase: pure SPEED vector (ram-through). NO position target -> NO S-curve braking;
                        # the drone passes THROUGH the target at full speed (without touching the WPNAV_ACCEL).
                        dxr = aim[0] - ap[0]; dyr = aim[1] - ap[1]; dzr = aim_z - ap[2]
                        vnorm = math.sqrt(dxr*dxr + dyr*dyr + dzr*dzr)
                        if vnorm > 1e-3:
                            vs = ATTACK_SPEED_MPS / vnorm
                            self._send_velocity_ned(drone_id, dxr*vs, dyr*vs, dzr*vs,
                                                    yaw=lock_yaw)
                        else:
                            self._goto_with_yaw_lock(drone_id, aim[0], aim[1], aim_z,
                                                     lock_yaw)
                    else:
                        self._goto_with_yaw_lock(drone_id, aim[0], aim[1], aim_z,
                                                 lock_yaw)

                if now - last_log > 1.0:
                    tspd = math.sqrt(tgt_vel[0]**2 + tgt_vel[1]**2 + tgt_vel[2]**2)
                    aspd_act = math.sqrt(av[0]**2 + av[1]**2 + av[2]**2)
                    tag = ("VEL" if (ENABLE_VELOCITY_TERMINAL and phase == "TERM"
                                     and not deflected)
                           else ("DEF" if deflected else phase))
                    logger.info(f"[Attack] d{drone_id} {phase}/{tag} rng={rng:.0f}m "
                                f"alt_off={alt_offset_m:+.1f}m"
                                f"{'(ZARF KIRPTI)' if alt_clipped_since_log else ''} "
                                f"best={best_rng:.0f}m t_int={t_int:.1f}s "
                                f"tgt_spd={tspd:.1f} atk_spd={aspd_act:.1f}m/s "
                                f"t= {elapsed:.0f} s intersection=( {ip[0]:.0f} , {ip[1]:.0f} )")
                    last_log = now
                    alt_clipped_since_log = False

            except Exception:
                logger.exception("[Attack] intercept thread error.")
            time.sleep(dt)

    def _egress_deflect(self, drone_id, ap, aim):
        """
        Leaving the swarm at FULL SPEED (#3 — collision avoidance, NO loss of speed).

        Input: aim -> normal intercept april point (FAR; including overshoot). Rotary: (new_aim_value,
        deflected_bool) deflected=True if current is deflected out+up to go around the swarm
        (direction is frozen but APR remains FAR -> S-curve does not brake, full speed is
        maintained). If it is clean, the current returns as it is, False.

        Logic: if the horizontal distance to nearest neighbor is below EGRESS_CLEAR_RADIUS_M (i.e.
        we are still inside the swarm), blend the April direction OUT and UP from the swarm center,
        with a weight proportional to how 'inside' we are. As you move away, the weight drops to 0
        and the attacker naturally settles into the actual intersection. The altitude ceiling is
        maintained at the aim_z clamp on the calling side.
        """
        with self.lock:
            others = [p for d, p in self.drone_positions.items()
                      if d != drone_id and d not in self._grounded_ids and p is not None]
        if not others:
            return aim, False

        min_horiz = min(math.hypot(ap[0] - p[0], ap[1] - p[1]) for p in others)
        if min_horiz >= EGRESS_CLEAR_RADIUS_M:
            return aim, False   # clean horizontal -> no deviation, straight intersection.

        # How far inside are we? (0 = at the border, 1 = at the center) -> deviation weight.
        strength = (EGRESS_CLEAR_RADIUS_M - min_horiz) / EGRESS_CLEAR_RADIUS_M
        weight = max(0.0, min(1.0, strength)) * EGRESS_DEFLECT_GAIN

        # Unit direction to April.
        ax_, ay_, az_ = aim[0]-ap[0], aim[1]-ap[1], aim[2]-ap[2]
        adist = math.sqrt(ax_*ax_ + ay_*ay_ + az_*az_)
        if adist < 1e-3:
            return aim, False
        tux, tuy, tuz = ax_/adist, ay_/adist, az_/adist

        # Horizontal unit direction outward (from swarm center to attacker).
        cn = sum(p[0] for p in others) / len(others)
        ce = sum(p[1] for p in others) / len(others)
        ox, oy = ap[0] - cn, ap[1] - ce
        onorm = math.hypot(ox, oy)
        if onorm < 1e-3:
            # Attacker ~in the center (i.e. top/bottom slot): choose a direction perpendicular to the april.
            ox, oy = -tuy, tux
            onorm = math.hypot(ox, oy) or 1.0
        oux, ouy = ox / onorm, oy / onorm

        # Direction blend: april + weight*(outward + upward). (NED up = -z.)
        dx = tux + weight * oux
        dy = tuy + weight * ouy
        dz = tuz + weight * (-1.0)   # up component
        dn = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        dx, dy, dz = dx/dn, dy/dn, dz/dn

        # Maintain FAR aim: same distance, deflected direction.
        new_aim = (ap[0] + dx*adist, ap[1] + dy*adist, ap[2] + dz*adist)
        return new_aim, True

    # ------------------------------------------------------------------ YAW LOCK after transition
    # ------------------------------------------------------------------
    def _locked_yaw(self, drone_id):
        """The rotated head angle of this drone is [rad], None if there is no lock."""
        with self._yaw_lock_mu:
            st = self._yaw_locks.get(drone_id)
        return st["yaw"] if st else None

    def _cache_wp_yaw_behavior(self, drone_id):
        """
        Reads and stores BEFORE attack WP_YAW_BEHAVIOR in the background; This value is written back
        when the lock is released. Since get_param blocks, it runs on a separate thread — it does
        not delay commit_attacker.
        """
        if drone_id in self._wp_yaw_behavior_saved:
            return
        def _read():
            val = self.get_param(drone_id, "WP_YAW_BEHAVIOR")
            if val is None:
                logger.warning(f"[YawLock] d{drone_id}: Could not read WP_YAW_BEHAVIOR; "
                               f"default when unlocking "
                               f"{DEFAULT_WP_YAW_BEHAVIOR:.0f} will be written.")
                return
            self._wp_yaw_behavior_saved[drone_id] = float(val)
            logger.info(f"[YawLock] d{drone_id}: before attack "
                        f"WP_YAW_BEHAVIOR={val:.0f} stored.")
        t = threading.Thread(target=_read, name=f"YawBehaviorRead-{drone_id}",
                             daemon=True)
        t.start()
        self.threads.append(t)

    def _lock_yaw(self, drone_id, reason=""):
        """
        Returns the head angle at INSTANT value and starts the thread monitoring the lock.

        Two-layer: (1) WP_YAW_BEHAVIOR=0 -> turns off AUTOMATIC yaw of ArduCopter towards
        destination/route, (2) fixed yaw field in commands -> rotated angle is explicitly commanded
        (yaw_rate is ignored). Neither alone is sufficient: without (1) the auto will compete with
        our command yaw, without (2) the vehicle may drift to the final destination yaw.
        """
        if not ENABLE_ATTACK_YAW_LOCK:
            return
        # The head angle is read before GETTING the lock mutex: to avoid nesting two locks (it says
        # drone_headings under outerpatcher self.lock).
        with self.lock:
            hdg = self.drone_headings.get(drone_id)
        if hdg is None:
            logger.warning(f"[YawLock] d{drone_id}: head angle unknown, "
                           f"lock COULD NOT ESTABLISH ( {reason} ).")
            return
        with self._yaw_lock_mu:
            if drone_id in self._yaw_locks:
                return                      # zaten kilitli
            self._yaw_locks[drone_id] = {"yaw": float(hdg),
                                         "t0": time.monotonic()}
        # Close automatic yaw (only our fixed angle is valid throughout the lock).
        self.set_param(drone_id, "WP_YAW_BEHAVIOR", 0.0)
        logger.warning(f"[YawLock] d{drone_id} LOCKED ({reason}) — head angle "
                       f"{math.degrees(hdg):.0f}° frozen; dissolve: horizontal speed "
                       f"<= {ATTACK_YAW_UNLOCK_SPEED_MPS:.0f} m/s, recharge "
                       f"or {ATTACK_YAW_LOCK_TIMEOUT_S:.0f}s.")
        log_event("yaw_locked", drone_id=drone_id, reason=reason,
                  heading_deg=math.degrees(hdg))
        t = threading.Thread(target=self._yaw_lock_watchdog, args=(drone_id,),
                             name=f"YawLock-{drone_id}", daemon=True)
        t.start()
        self.threads.append(t)

    def _unlock_yaw(self, drone_id, reason=""):
        """Removes the lock and returns WP_YAW_BEHAVIOR to its BEFORE attack value."""
        with self._yaw_lock_mu:
            st = self._yaw_locks.pop(drone_id, None)
        if st is None:
            return
        prev = self._wp_yaw_behavior_saved.get(drone_id)
        if prev is None:
            prev = DEFAULT_WP_YAW_BEHAVIOR
            logger.warning(f"[YawLock] d{drone_id}: pre-attack WP_YAW_BEHAVIOR "
                           f"could not be read — default is writing {prev:.0f}.")
        self.set_param(drone_id, "WP_YAW_BEHAVIOR", float(prev))
        held = time.monotonic() - st["t0"]
        logger.warning(f"[YawLock] d{drone_id} FREE ({reason}) — {held:.1f}s remained locked.")
        log_event("yaw_unlocked", drone_id=drone_id, reason=reason, held_s=held)

    def _yaw_lock_watchdog(self, drone_id, rate_hz=10.0):
        """
        It waits for the condition that will unlock the lock. It is SEPARATE from the intersection
        thread: it continues to run after the attacker is released into formation (the really
        dangerous slowdown phase).

        Dissolution conditions (first to occur wins): 1) horizontal ground speed <=
        ATTACK_YAW_UNLOCK_SPEED_MPS, 2) turn completed while attack STILL alive: angle between own
        velocity vector and target bearing remained CONTINUOUSLY below ATTACK_YAW_RECHASE_ANGLE_DEG
        along ATTACK_YAW_RECHASE_CONFIRM_S, 3) ATTACK_YAW_LOCK_TIMEOUT_S is full (safety net).
        """
        dt = 1.0 / max(rate_hz, 1.0)
        t0 = time.monotonic()
        rechase_since = None       # The moment when the angle is first entered below the threshold
        while not self.stop_threads.is_set():
            with self._yaw_lock_mu:
                if drone_id not in self._yaw_locks:
                    return          # someone else solved it (closing etc.)
            if self._shutting_down.is_set():
                self._unlock_yaw(drone_id, reason="shutdown")
                return

            elapsed = time.monotonic() - t0
            if elapsed > ATTACK_YAW_LOCK_TIMEOUT_S:
                logger.warning(f"[YawLock] d{drone_id}: {ATTACK_YAW_LOCK_TIMEOUT_S:.0f}s "
                               f"no dissociation condition occurred in it -> forced release.")
                self._unlock_yaw(drone_id, reason="timeout")
                return

            with self.lock:
                pos = self.drone_positions.get(drone_id)
                vel = self.drone_velocities.get(drone_id, (0.0, 0.0, 0.0))
            tgt = self._last_target_ned

            # (1) Speed ​​condition — HORIZONTAL ground speed (climb/descent not counted).
            gnd_spd = math.hypot(vel[0], vel[1])
            if gnd_spd <= ATTACK_YAW_UNLOCK_SPEED_MPS:
                self._unlock_yaw(drone_id, reason=f"slowed_to_{gnd_spd:.1f}mps")
                return

            # (2) Recharge — ONLY valid if the attack is still on this drone. For a drone left in formation, this
            # condition is disabled; In that case, only speed (1) or timeout (3) unlocks.
            if (self._committed_attacker == drone_id
                    and pos is not None and tgt is not None):
                lx, ly, lz = tgt[0]-pos[0], tgt[1]-pos[1], tgt[2]-pos[2]
                lnorm = math.sqrt(lx*lx + ly*ly + lz*lz)
                vnorm = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
                if lnorm > 1e-3 and vnorm > 1e-3:
                    # The angle between the velocity vector and the bearing to the target.
                    cos_a = (lx*vel[0] + ly*vel[1] + lz*vel[2]) / (lnorm * vnorm)
                    cos_a = max(-1.0, min(1.0, cos_a))
                    angle_deg = math.degrees(math.acos(cos_a))
                    if angle_deg <= ATTACK_YAW_RECHASE_ANGLE_DEG:
                        now = time.monotonic()
                        if rechase_since is None:
                            rechase_since = now
                        elif now - rechase_since >= ATTACK_YAW_RECHASE_CONFIRM_S:
                            self._unlock_yaw(
                                drone_id,
                                reason=f"rechase_angle_{angle_deg:.0f}deg")
                            return
                    else:
                        rechase_since = None    # went out of threshold -> counter zero
            else:
                rechase_since = None

            time.sleep(dt)

    def _send_velocity_ned(self, target_id, vn, ve, vd, yaw=None):
        """
        Pure SPEED command (SET_POSITION_TARGET_LOCAL_NED, speed-only mask).

        NO position target -> ArduCopter The S-curve position planner is not enabled, i.e. there is
        no BRAKING to zero at the target. Used to move THROUGH the target at full speed during the
        terminal/kill phase (acceleration WITHOUT TOUCHING WPNAV_ACCEL).

        Since the NED velocity components are origin-independent, they are sent directly in the
        LOCAL_NED frame (position fields are ignored).

        If yaw is not None (post-switch yaw lock), the mask USES the yaw field and the given
        absolute head angle [rad] is commanded; yaw_rate is ignored again.
        """
        mask = VELOCITY_ONLY_MASK if yaw is None else VELOCITY_YAW_MASK
        self.master.mav.set_position_target_local_ned_send(
            0, target_id, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            0.0, 0.0, 0.0,              # location (ignored)
            float(vn), float(ve), float(vd),
            0.0, 0.0, 0.0,              # acceleration (ignored)
            float(yaw or 0.0), 0.0,     # yaw [rad] (valid if locked), yaw_rate ignored
        )

    def _release_attacker(self, drone_id, reason=""):
        """
        Returns the attacker to formation.

        ATTENTION: yaw lock cannot be INTENTIONALLY unlocked here. Release is often triggered
        immediately after the transition (drone is still ~ 22 m/s ) and the formation slot is
        BEHIND; Unlocking here creates exactly the full-throttle 180 ° rotation we want to avoid.
        The lock is released by _yaw_lock_watchdog: when the horizontal speed drops to 13 m/s (this
        condition occurs quickly after release) or at timeout 10 sec.
        """
        logger.info(f"[Attack] Drone {drone_id} is free from attack ({reason}) "
                    f"-> returning to follow mode.")
        log_event("attacker_released", drone_id=drone_id, reason=reason)
        if self._committed_attacker == drone_id:
            self._committed_attacker = None
        self._attack_skip.discard(drone_id)
        # Restore vehicle's OWN default speed/acceleration profile (with ceiling MAX_SPEED) for formation
        # flight — released attacker flies at the same speed as the swarm.
        self._restore_default_dynamics(drone_id)
        # Wait: reselecting this drone immediately after the failed attempt.
        if reason not in ("hit", "survived_hit"):
            self._attack_cooldown[drone_id] = time.monotonic() + ATTACK_COOLDOWN_S

    def _commit_next_attacker(self):
        """Selects and discards the next available attacker (re-attack cycle)."""
        # F: use last known target position for target-side selection if possible.
        nxt = self._pick_attacker(target_ned=self._last_target_ned)
        if nxt is None:
            logger.warning("[Attack] No other available attackers.")
            return
        self.commit_attacker(nxt, min_alt_m=self._attacker_min_alt)

    def sequential_landing(self, per_drone_gap_s=2.0, verify_s=6.0):
        """
        He gradually moves the swarm to RTL and VERIFIES that they have ALL actually moved to RTL.
        """
        logger.info("--- Sequential landing procedure ---")

        self._shutting_down.set()
        self.pause_formation_following()
        time.sleep(0.3)   # let threads finish their last tick

        order = [self.leader_id]
        if self._committed_attacker is not None and self._committed_attacker != self.leader_id:
            order.append(self._committed_attacker)
        order += [d for d in self.drone_ids
                  if d != self.leader_id and d != self._committed_attacker]

        for did in order:
            if did not in self._alive_ids:
                logger.info(f"[SeqLand] Drone {did} is skipped (not alive).")
                continue
            mode_now = self.drone_modes.get(did, "")
            if mode_now == "LAND":
                logger.info(f"[SeqLand] Drone {did} already LAND — skipped (damaged).")
                continue
            logger.info(f"[SeqLand] Drone {did} -> RTL")
            log_event("sequential_rtl", drone_id=did)
            self.set_mode(did, "RTL")
            time.sleep(per_drone_gap_s)

        # 3 ) VERIFICATION: If there is still something that does not pass to RTL, try again.
        deadline = time.time() + verify_s
        pending = set()
        while time.time() < deadline:
            pending = set()
            for did in self.drone_ids:
                if did not in self._alive_ids:
                    continue
                m = self.drone_modes.get(did, "")
                if m not in ("RTL", "LAND"):
                    pending.add(did)
            if not pending:
                logger.info("[SeqLand] VERIFIED: all vehicles are in RTL/LAND mode.")
                return
            for did in pending:
                logger.warning(
                    f"[SeqLand] Drone {did} still not in RTL "
                    f"(mode={self.drone_modes.get(did,'?')}) — Resending RTL."
                )
                self.set_mode(did, "RTL")
            time.sleep(1.0)

        if pending:
            logger.error(
                f"[SeqLand] WARNING: {sorted(pending)} tools in {verify_s:.0f}s "
                f"Did not move to RTL! Manual intervention from the remote control may be required."
            )

    def stop_all(self):
        """It stops all threads and closes the connection."""
        logger.info("All threads are stopped...")
        self._shutting_down.set()   # Threads sending commands should stop immediately

        # Release any remaining yaw locks so WP_YAW_BEHAVIOR does not remain 0 on the vehicle. Do this before
        # stop_threads, otherwise the monitoring threads exit without restoring the value.
        for did in list(self._yaw_locks.keys()):
            self._unlock_yaw(did, reason="stop_all")

        self.stop_threads.set()

        # simple_guided_follow kill subprocess if it is still running
        if self._follow_proc is not None and self._follow_proc.poll() is None:
            logger.info("Process simple_guided_follow is terminating...")
            self._follow_proc.terminate()
            try:
                self._follow_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._follow_proc.kill()
            logger.info("simple_guided_follow process stopped.")

        for t in self.threads:
            try:
                if t.is_alive():
                    t.join(timeout=3)
            except Exception:
                logger.exception("Thread join error.")
        try:
            self.master.close()
        except Exception:
            logger.exception("Error closing connection MAVLink.")
        logger.info("The connection has been closed.")


# ======================== Auxiliary functions =========================

def load_waypoints_from_file(filename="waypoints.json") -> list:
    """Loads waypoints from a JSON file.  """
    logger.info(f"Waypoints okunuyor: {filename}")
    try:
        with open(filename, 'r', encoding="utf-8") as f:
            waypoints = json.load(f)
        if not waypoints:
            logger.warning("Waypoint file is empty.")
            return []
        logger.info(f"{len(waypoints)} waypoints have been loaded.")
        return waypoints
    except FileNotFoundError:
        logger.error(f"'{filename}' not found, task will be skipped.")
        return []
    except json.JSONDecodeError:
        logger.error(f"'{filename}' is not a valid JSON.")
        return []
    except Exception:
        logger.exception("Unexpected error reading waypoints.")
        return []


def latlon_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    """
    Flat-world equirectangular approach. Returns (north, east, down) in meters relative to the
    origin.
    """
    R_EARTH = 111320.0
    north = (lat - origin_lat) * R_EARTH
    east  = (lon - origin_lon) * R_EARTH * math.cos(math.radians(origin_lat))
    down  = -(alt - origin_alt)
    return north, east, down

def ned_to_latlon(north, east, down, origin_lat, origin_lon, origin_alt):
    """
    Inverse of latlon_to_ned: returns the point (north, east, down) in the shared NED frame as (lat,
    lon, alt AMSL).
    """
    R_EARTH = 111320.0
    lat = origin_lat + north / R_EARTH
    lon = origin_lon + east / (R_EARTH * math.cos(math.radians(origin_lat)))
    alt = origin_alt - down
    return lat, lon, alt


def build_takeoff_altitudes(
    base_altitude: float,
    vertical_offsets: dict,
    drone_ids: list,
    ) -> dict:
    """
    Returns the takeoff altitude to the drone press. vertical_offset > 0 → the drone lifts higher
    than the leader.
    """
    return {
        did: base_altitude + float(vertical_offsets.get(did, 0.0))
        for did in drone_ids
    }

# =========================
# Main
# =========================
def main():
    # --- SETTINGS ---
    CONNECTION_PORT      = 14554
    TAKEOFF_ALTITUDE     = 40.0
    # LIVE: this is just the INITIAL value and is read from FORMATION_OFFSET_M in guidance_config.py. To
    # replace in-flight, edit and save line FORMATION_OFFSET_M in guidance_config.py, NOT THIS line -- the
    # formation manager ramps to the new range in ~0.5 s. (10.0 here only comes into play if
    # guidance_config is not found.)
    FORMATION_OFFSET     = FORMATION_OFFSET_LIVE.target()
    SELECTED_FORMATION   = Formation.DIAMOND
    SYMMETRIC_HORIZONTAL = True

    LEADER_MIN_ALT_M     = 30.0
    FOLLOWER_MIN_ALT_M   = 25.0
    SLOT_REASSIGN_THRESHOLD_M = 14.0 # Slot reassignment threshold — a drone will switch to a new slot if there is a slot this much (meters) closer than its current slot
    # Too small → constant swapping, too large → slow response

    LEADER_LOOP_START_WP = 2
    LEADER_LOOP_ARRIVAL_DIST_M = 2.0
    WP_SPEED = 8.0 # Flight speed of the leading drone in AUTO mode [m/s]. If set to 0, no speed command is sent.

    GUIDED_FOLLOW_SCRIPT = "simple_guided_follow.sh"

    # --- Security settings ---
    GPS_MIN_SATS             = 10
    GPS_GATE_TIMEOUT_S       = 40.0

    VERTICAL_OFFSETS = {
        3:  4.0,   # top observer -> 4 m ABOVE leader
        5:  2.0,   # flank attacker -> 2 m ABOVE the leader
        2: -2.0,   # flank attacker -> 2 m BELOW leader
        4: -4.0,   # sub observer -> BELOW leader 4 444444444444444
    }

    TAKEOFF_ALTITUDES = build_takeoff_altitudes(
        TAKEOFF_ALTITUDE, VERTICAL_OFFSETS, DRONE_IDS
    )
    logger.info(f"Takeoff alts per drone: {TAKEOFF_ALTITUDES}")

    REDIS_HOST       = config.REDIS_HOST   # single source: controller.json ( config.py )
    REDIS_PORT       = config.REDIS_PORT
    CAMERA_REDIS_KEY = 'leader_cam_frame'

    redis_client_img = redis.Redis(
        host=REDIS_HOST, port=REDIS_PORT, db=0,
        socket_timeout=0.1,
        socket_connect_timeout=2.0,
    )

    _latest_frame = [None]
    _camera_running = [True]

    def _camera_thread():
        error_count = 0
        last_error_log = 0.0
        while _camera_running[0]:
            try:
                frame_data = redis_client_img.get(CAMERA_REDIS_KEY)
                if frame_data:
                    nparr = np.frombuffer(frame_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        _latest_frame[0] = frame
                        error_count = 0
                    else:
                        logger.warning("OpenCV could not decode the image (bad byte).")
            except Exception as e:
                # Redis timeout or connection error — log every error occasionally rather than swallowing it silently.
                error_count += 1
                now = time.time()
                if now - last_error_log > 5.0:
                    logger.warning(f"Camera Redis error ({error_count}x): {e}")
                    last_error_log = now
            time.sleep(0.033)   # ~30 Hz receiving speed

    # Load waypoints from JSON
    WAYPOINT_LIST = load_waypoints_from_file("waypoints.json")

    cam_thread = threading.Thread(target=_camera_thread, name="CameraFetch", daemon=True)
    cam_thread.start()

    swarm = None
    try:
        swarm = SwarmController(
            LEADER_ID,
            CONNECTION_PORT,
            DRONE_IDS,
            symmetric_horizontal=SYMMETRIC_HORIZONTAL,
        )
        logger.info(f"SELECTED PATTERN: {SELECTED_FORMATION.value}")

        # Start status publisher for live pattern visualization panel (viz_dashboard.py)
        redis_state_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)

        _stale_keys = [
            "leader_slot_ned", "attacker_slot_ned", "attacker_state_ned",
            "leader_state_ned", "dogru_rakip_telemetry", "yanlis_rakip_telemetry",
            "target_status", "ned_origin",
        ]
        for _k in _stale_keys:
            try:
                if redis_state_client.delete(_k):
                    logger.warning(f"[ RedisFlush ] Stale key deleted: ' {_k} '")
            except Exception:
                logger.exception(f"[RedisFlush] Could not delete '{_k}'")
        logger.info("[RedisFlush] Cleared keys from previous run.")
        # Before testing, run: pkill -f simple_guided_follow

        swarm.start_state_publisher(redis_state_client, rate_hz=5.0, jsonl_rate_hz=1.0)
        swarm.setup_high_rate_telemetry(DRONE_IDS)
        swarm.apply_global_speed_limit(DRONE_IDS)
        swarm.gps_gate_all(DRONE_IDS, min_fix=3, min_sats=GPS_MIN_SATS, timeout=GPS_GATE_TIMEOUT_S)

        redis_origin = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)
        swarm.fetch_and_publish_ned_origin(LEADER_ID, redis_origin)

        # NEW: start drone press >=20 Hz flight CSV recording (Hit Detection recording).
        swarm.start_flight_csv_loggers(DRONE_IDS, rate_hz=25.0)

        # Parallel takeoff with drone press altitudes
        logger.info("Parallel takeoff begins...")
        swarm.parallel_launch(DRONE_IDS, TAKEOFF_ALTITUDES)

        swarm.start_heartbeat_watchdog(timeout_s=LEADER_HB_TIMEOUT_S)
        swarm.start_leader_position_watchdog(stale_timeout_s=LEADER_POS_LOITER_S)

        swarm.enable_leader_failover()
        swarm.start_impact_monitor(DRONE_IDS)

        redis_pursuer = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)
        swarm.start_pursuer_state_publisher(redis_pursuer, rate_hz=20.0)
        swarm._attacker_min_alt = FOLLOWER_MIN_ALT_M
        swarm._attacker_max_alt = config.MAX_ALT_M   # offensive altitude ceiling (controller.json)

        # swarm.start_keyboard_listener() logger.info( "Security systems active: " "Heartbeat watchdog, Leader
        # position watchdog, Keyboard listener (r=RTL, l=LAND, q=Exit)" )

        # Start formation tracking
        logger.info("--- Formation Tracking is Starting ---")
        swarm.start_formation_following(
            swarm.leader_id, SELECTED_FORMATION, FORMATION_OFFSET, VERTICAL_OFFSETS, slot_reassign_threshold_m=SLOT_REASSIGN_THRESHOLD_M, follower_min_alt_m=FOLLOWER_MIN_ALT_M,
        )
        logger.info("Follower drones follow the leader in 3D.")

        time.sleep(5)

        logger.info("---simple_guided_follow starting (lead drone switches to aircraft tracking) ---")
        swarm.launch_guided_follow(
            GUIDED_FOLLOW_SCRIPT,
            min_alt_m=LEADER_MIN_ALT_M,
            loop_start_wp_index=LEADER_LOOP_START_WP,
            loop_arrival_dist_m=LEADER_LOOP_ARRIVAL_DIST_M,
            wp_speed_mps=WP_SPEED,
        )
        logger.info("simple_guided_follow is running in the background.")


        SHOW_CAMERA = False   # make False to work without screen (without cv2 window)

        rc_target = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0,
                                decode_responses=True)

        if ENABLE_KILL_STRATEGY:
            logger.info("ENABLE_KILL_STRATEGY=True -> attack logic ACTIVE.")
        else:
            logger.info("ENABLE_KILL_STRATEGY=False -> takeoff/formation/landing only.")

        if SHOW_CAMERA:
            cv2.namedWindow("Leader Drone Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Leader Drone Camera", 640, 480)
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for video...", (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        last_no_frame_warn = 0.0

        def _kill_check():
            """A single iteration of autonomous attacker assignment control."""
            if not ENABLE_KILL_STRATEGY:
                return
            if swarm._committed_attacker is not None:
                return
            try:
                raw = rc_target.get("dogru_rakip_telemetry")
                if not raw:
                    return
                tj = json.loads(raw)
                if not (tj.get("valid", True) and "lat" in tj):
                    return
                origin = swarm._ned_origin
                with swarm.lock:
                    lp = swarm.drone_positions.get(swarm.leader_id)
                if origin and lp:
                    tn, te, td = latlon_to_ned(
                        tj["lat"], tj["lon"],
                        origin[2] + tj.get("alt", 0.0), *origin)
                    # F: store last known target NED (also uses reattack cycle).
                    swarm._last_target_ned = (tn, te, td)
                    rng = math.sqrt((lp[0]-tn)**2 + (lp[1]-te)**2)
                    if rng <= COMMIT_RANGE_M:
                        atk = swarm._pick_attacker(target_ned=(tn, te, td))
                        if atk is not None:
                            logger.warning(f"[Kill] At target range ({rng:.0f}m) "
                                           f"-> assigning attacker {atk}.")
                            swarm.commit_attacker(
                                atk, redis_host=REDIS_HOST,
                                redis_port=REDIS_PORT, min_alt_m=FOLLOWER_MIN_ALT_M)
            except Exception:
                logger.exception("[Kill] control loop error.")

        while not swarm.stop_threads.is_set():
            _kill_check()

            if SHOW_CAMERA:
                frame = _latest_frame[0]
                if frame is not None:
                    if frame.max() == 0:
                        logger.warning("The image is black!")
                    cv2.imshow("Leader Drone Camera", frame)
                else:
                    cv2.imshow("Leader Drone Camera", placeholder)
                    now = time.time()
                    if now - last_no_frame_warn > 5.0:
                        logger.warning(
                            f"There is no camera view yet (Redis key='{CAMERA_REDIS_KEY}'). "
                            f"Is leader_to_redis.py working and correct drone topic? "
                            f"abone mi? (Lider = drone {LEADER_ID})")
                        last_no_frame_warn = now
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    logger.info("The camera window is closed.")
                    break
            else:
                time.sleep(0.1)

    except (ConnectionError, KeyboardInterrupt):
        logger.exception("PROGRAM INTERRUPTED / CONNECTION ERROR.")
    except Exception:
        logger.exception("UNEXPECTED ERROR.")

    finally:
        _camera_running[0] = False

        try:
            import signal as _signal
            _signal.signal(_signal.SIGINT, _signal.SIG_IGN)
            logger.info("Closing has begun — additional Ctrl+C signals are ignored.")
        except Exception:
            pass

        if swarm:
            logger.info("--- End of Mission: Sequential Landing Procedure ---")

            try:
                swarm.sequential_landing(per_drone_gap_s=2.0, verify_s=6.0)
            except Exception:
                logger.exception("Error during sequential landing — RTL will still be attempted.")
                for _d in swarm.drone_ids:
                    try: swarm.set_mode(_d, "RTL")
                    except Exception: pass

            logger.info("Sequential landing commands were sent. Waiting for landing (25s)...")
            try:
                time.sleep(25)
            except Exception:
                pass
            swarm.stop_all()
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

if __name__ == "__main__":
    main()