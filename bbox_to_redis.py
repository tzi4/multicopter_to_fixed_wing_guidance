#!/usr/bin/env python3
"""
Drone camera -> Redis ' tracker_bbox ' color-detection bridge (+ mod decider)
============================================================== companion_scripts /
drone_cam_bbox_to_redis .py from the skeleton friend (topic, decider window, HUD). Moved
MEASUREMENT-BASED lessons from the bumblebee package onto it:

  1) The target color is PURPLE. Measurements in bumblebee (3 recordings, 900 frames, 1.99e8
  chromatic pixels, S>=70,V>=50): H 110-114 (sky dome): 54.4%; H 45-49 (grass): 26.1%; H 30-34
  (runway): 13.8%; H 0-4 (RED window): 1.9%, overlapping the background; H 140-160 (PURPLE window):
  0.0012%, approximately 1700x separation. When the aircraft banked, the horizon band
  (BGR~127,127,255) produced a false target across the frame width. The target color was changed to
  Gazebo/Purple in models/gazebo-plane6 while retaining the world detail. YILDIZ_TARGET_COLOR=red
  restores the red window.

  2) SHAPE DOOR (optional, default CLOSED). After km Gazebo ~2, it produces line artifacts bearing
  aircraft colors at altitude 1 px in the ground plane; After dilation, h~5 px contours and they can
  play the "largest contour" selection.

  3) Video recording is INDEPENDENT from external playback (also works in headless).

Broadcast format (JSON reads guidance codes with json.loads): [x, y, w, h, horizontal_coverage_%,
validity, t_capture] guidance codes under stars25/ reads data[:4]; excess is harmless.
"""

import argparse
import csv
import json
import math
import os
import signal
import threading
import time
from collections import deque

import cv2
import numpy as np
import redis

# --- OPENCV THREAD NUMBER (bumblebee tutorial, measurement 2026-07) --- OpenCV does the SAME work as
# the default 16 thread at ~4x CPU (1080p measurement: 7.7 ms @%411 CPU vs single thread 10.3 ms @%100
# CPU). 30 For the Hz camera, one thread is already more than enough, so the default is 1. It gets
# squashed with YILDIZ_CV_THREADS (0/negative = OpenCV default; setNumThreads(0) in OpenCV means
# "sequential" NOT "all cores", so -1).
_CV_THREADS = int(os.environ.get('YILDIZ_CV_THREADS', '1'))
cv2.setNumThreads(_CV_THREADS if _CV_THREADS > 0 else -1)
# --- ROS OPTIONAL (2026-08-07) ------------------------------------ WHY TRY/EXCEPT: ROS FEMALE
# parts
# of this file (AttitudeReader, HSV detection, virtual gimbal chain, decision maker) ACTUAL It is
# also
# required in HARDWARE, but there is NO ROS in Raspberry Pi 5. Hard import at the module level was
# preventing the file from even being IMPORTED; the hardware bridge (hardware/camera_bridge.py)
# would
# have to REWRITE those parts -- so two copies of the same logic, two separate realities. So that it
# can be imported instead of copied.
#
# Behavior is unchanged when ROS is available: all three modules connect as before and the same
# execution path runs. Without ROS, only the ROS-dependent paths are unavailable: the subscriber in
# SwarmRedisDetector.__init__ and main/rospy.spin. Both report the error below.
try:
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image
except ImportError as _ros_import_error:      # No ROS (Pi) -- non-ROS paths work
    rospy = None
    CvBridge = None
    Image = None

    class CvBridgeError(Exception):
        """Make 'except CvBridgeError' in image_callback resolve without ROS."""

    ROS_ERROR = _ros_import_error
else:
    ROS_ERROR = None


def ros_required(location):
    """Called at the entry of paths dependent on ROS: an error that tells what to do instead of silent
AttributeError."""
    if rospy is None:
        raise SystemExit(
            f"ERROR: {location} requires ROS but ROS could not be imported "
            f"({ROS_ERROR}).\n"
            "In real hardware (without ROS) instead "
            "Use 'python3 hardware/camera_bridge.py'.")

from yildizlar_gimbal import AimTrim, VirtualGimbal, analytical_aim, joint_angle
from tools.gz_gimbal import (TiltStateReader, TiltCommander, TiltTracking,
                             model_name_from_topic)


class AttitudeReader(threading.Thread):
    """MAVLink continuously reads ATTITUDE and writes it to the TIMESTAMPED buffer.

    WHY THREAD: calling single recv_match per frame accumulates in UDP buffer and STALE attitude is
    read (same error measured in tools/swarm_command.py get_position: 9 message accumulates in 6 s and 6 s
    old data was coming).

    WHY BUFFER + INTERPOLATION (2026-08-02): just keeping the "latest attitude" was not enough.
    Attitude 50 Hz, cameras 30 Hz and since the two streams come independently of each other, there
    is a shift between them 0-30 ms; At a body speed of 30 degrees/s, this means a de-rotation error
    of ~1 degrees. It was measured: the correlation of stabilized vertical error with pitch could
    only decrease from 0.873 to 0.290. Solution: store the last N samples with timestamp, move by
    linear interpolation to the key where the frame was CAPTURED.

    IT ALSO WORKS IN ACTUAL FLIGHT - NOT dependent on the simulation clock: * Each sample is stamped
    with time.monotonic() as soon as it ARRIVES. The same clock is used for frames; that is, the two
    streams meet at ONE COMMON hour. (On the Pi, both the camera and MAVLink are stamped on the same
    machine.) * Additionally, the FCU's own clock ( time_boot_ms ) is recorded and the MINIMUM of
    the ( t_local - t_fcu ) difference is kept. Minimum is the best estimate of the actual offset
    between two clocks as it corresponds to the delayed sample; Used to report link latency and
    switch to FCU clock if necessary. * The pipeline delay of the camera (capture -> reach us) is a
    fixed number and MEASURED by tools/ calibrate_gimbal_timing .py; implemented here as latency_s .
    """

    def __init__(self, port, hz=50.0, buffer=512, latency_s=0.0, mav=None):
        """If port: NUMBER, 'udpin:127.0.0.1:<port>' is installed (sim behavior, UNCHANGED). TEXT is used
directly as address pymavlink
        ('/dev/ttyACM0', 'tcp:127.0.0.1:5760', 'udpin:0.0.0.0:14601' ...).
        REASON: in real hardware the autopilot is behind the USB/serial or mavproxy splitter; The
        fixed pattern 'udpin:127.0.0.1:N' is not there.

        mav: A READY mavutil connection (default None = establishes its own connection as before).
        REASON: in real hardware, the same address cannot be opened TWICE -- 'udpin:' binds to the
        port (second boot gives an error), serial port gives bad frame on second boot. To SHARE the
        same link with the tilt command (tools/mavlink_tilt.MavlinkTiltCommander). The contract is the
        same as the docstring of mavlink_tilt: it is called BEFORE scripter.start() (it does its
        single recv there), then this thread runs as the single recv consumer."""
        super().__init__(daemon=True)
        self.port = port
        self.mav = mav
        self.hz = hz
        self.latency_s = float(latency_s)
        self.buf = deque(maxlen=buffer)      # (t_local, t_fcu_s, roll, pitch, yaw)
        self.lock = threading.Lock()
        self.roll = self.pitch = self.yaw = 0.0     # en son (geriye uyumluluk)
        self.counter = 0
        self.ready = False
        self.clock_offset = None              # min(t_local - t_fcu)
        self.last_gap_s = 0.0              # range used in interpolation
        self._log_f = self._log_w = None
        self._log_counter = 0                  # periodic flush counter

    def log_ac(self, path_value):
        self._log_f = open(path_value, 'w', newline='')
        self._log_w = csv.writer(self._log_f)
        self._log_w.writerow(['t_local', 't_fcu_s', 'roll_deg', 'pitch_deg', 'yaw_deg'])

    def log_close(self):
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
            self._log_w = None

    def run(self):
        from pymavlink import mavutil
        address = (self.port if isinstance(self.port, str)
                 else f'udpin:127.0.0.1:{self.port}')
        try:
            if self.mav is not None:
                self._m = self.mav          # SHARED connection (see __init__)
            else:
                self._m = mavutil.mavlink_connection(address, source_system=253)
                self._m.wait_heartbeat(timeout=30)
            self._m.mav.command_long_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                int(1e6 / self.hz), 0, 0, 0, 0, 0)
            print(f"Virtual gimbal telemetry: {address} connected "
                  f"({self.hz:.0f} Hz requested)", flush=True)
        except Exception as exc:
            print(f"WARNING: attitude connection could not be established ( {exc} ); "
                  f"virtual gimbal CIRCUIT OUTSIDE", flush=True)
            return
        while True:
            msg = self._m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if msg is None:
                continue
            t_local = time.monotonic()
            t_fcu = msg.time_boot_ms / 1000.0
            offset_value = t_local - t_fcu
            if self.clock_offset is None or offset_value < self.clock_offset:
                self.clock_offset = offset_value
            with self.lock:
                self.buf.append((t_local, t_fcu, msg.roll, msg.pitch, msg.yaw))
            self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            self.counter += 1
            self.ready = True
            if self._log_w is not None:
                self._log_w.writerow([f"{t_local:.6f}", f"{t_fcu:.3f}",
                                      f"{math.degrees(msg.roll):.4f}",
                                      f"{math.degrees(msg.pitch):.4f}",
                                      f"{math.degrees(msg.yaw):.4f}"])
                # PERIODIC FLUSH: do not lose the last buffer in a crash (same reason as gimbal CSV, see note
                # 2026-08-07).
                self._log_counter += 1
                if self._log_counter % 20 == 0:
                    self._log_f.flush()

    @staticmethod
    def _angle_blend(a, b, k):
        """Shortest path angle interpolation (roll passes helix at +-180)."""
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        return a + d * k

    def get_attitude(self, t_frame):
        """roll/pitch of the moment (t_frame - delay) when the frame was CAPTURED.

        Linear between two adjacent samples; if it stays outside the buffer it drops to the nearest
        sample (NO extrapolation - magnifies the noise).
        """
        t_target = t_frame - self.latency_s
        with self.lock:
            n = len(self.buf)
            if n == 0:
                return None
            if n == 1 or t_target <= self.buf[0][0]:
                self.last_gap_s = 0.0
                return self.buf[0][2], self.buf[0][3], t_target
            if t_target >= self.buf[-1][0]:
                self.last_gap_s = t_target - self.buf[-1][0]
                return self.buf[-1][2], self.buf[-1][3], t_target
            # trailing scan instead of binary search: the target is usually in the last few samples
            for i in range(n - 1, 0, -1):
                t0 = self.buf[i - 1][0]
                t1 = self.buf[i][0]
                if t0 <= t_target <= t1:
                    k = 0.0 if t1 == t0 else (t_target - t0) / (t1 - t0)
                    self.last_gap_s = t1 - t0
                    return (self._angle_blend(self.buf[i - 1][2], self.buf[i][2], k),
                            self._angle_blend(self.buf[i - 1][3], self.buf[i][3], k),
                            t_target)
            self.last_gap_s = 0.0
            return self.buf[-1][2], self.buf[-1][3], t_target


# --- "THE CODE IS CURRENTLY WORKING" LINE (bumblebee/bbox_to_redis.py:200 idea) --- So that when you
# watch the video later, the answer to the question "with which code was this recording taken" is
# written ABOVE the frame. Since CPU COST IS SIGNIFICANT: * scanning is done in a SEPARATE THREAD and
# KODU_SCAN_S, * the frame processing path only reads a literal string (accessing an attribute), *
# /proc/<pid>/cmdline is read - NO pgrep/subprocess fork. Measurement: the total additional cost of
# overlay + video recording was already 5%; this line remains within it.
CODE_SCAN_S = 5.0
CODE_PATTERNS = (
    ('simple_guided_follow', 'konumlu(allstar)'),
    ('launch_mission', 'launch_mission'),
    ('swarm_command', 'swarm_command'),
    ('camera_calibration', 'camera_kalib'),
    ('measure_aim', 'measure_aim'),
    ('scenario.sh', 'scenario'),
)


class RunningCode(threading.Thread):
    """Periodically detects running guidance/test scripts."""

    def __init__(self, period=CODE_SCAN_S):
        super().__init__(daemon=True)
        self.period = float(period)
        self.text = 'Code: bbox_to_redis'
        self._stop = threading.Event()

    def _scan(self):
        found = []
        try:
            pids = [d for d in os.listdir('/proc') if d.isdigit()]
        except OSError:
            return
        for pid in pids:
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().decode('utf-8', 'ignore')
            except OSError:
                continue
            for pattern_value, label_item in CODE_PATTERNS:
                if pattern_value in cmd and label_item not in found:
                    found.append(label_item)
        self.text = 'Code: bbox_to_redis' + (' + ' + ' + '.join(found) if found else '')

    def run(self):
        while not self._stop.is_set():
            self._scan()
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()


# --- TARGET COLOR ------------------------------------------------------------
TARGET_COLOR_WINDOWS = {
    # Gazebo /Purple = RGB ( 1 , 0 , 1 ) -> OpenCV HSV H= 150 . One window is enough.
    'purple': ((140, 160),),
    # Since red spirals at H=0, two windows are required (old behavior).
    'red': ((0, 10), (170, 180)),
}
TARGET_COLOR_SV = {
    'purple': (120, 60),   # (S_min, V_min) - H.264 kroma gurultusunu eler
    'red': (70, 50),       # old red thresholds EXACTLY
}
DEFAULT_TARGET_COLOR = 'purple'

MIN_CONTOUR_AREA = 18      # value of friend; 5 does not miss distant target at fps


def build_color_ranges(color, s_min=None, v_min=None):
    """Generates HSV window list from color name (lower, upper)."""
    color = (color or '').strip().lower() or DEFAULT_TARGET_COLOR
    if color not in TARGET_COLOR_WINDOWS:
        print(f"WARNING: unknown YILDIZ_TARGET_COLOR='{color}'; valid: "
              f"{', '.join(sorted(TARGET_COLOR_WINDOWS))}. "
              f"using '{DEFAULT_TARGET_COLOR}'.")
        color = DEFAULT_TARGET_COLOR
    default_s, default_v = TARGET_COLOR_SV[color]
    s_min = default_s if s_min is None else s_min
    v_min = default_v if v_min is None else v_min
    ranges = [(np.array([lo, s_min, v_min]), np.array([hi, 255, 255]))
              for lo, hi in TARGET_COLOR_WINDOWS[color]]
    return color, ranges


def _opt_int(name):
    raw = os.environ.get(name, '').strip()
    if not raw:
        return None
    try:
        return max(0, min(255, int(float(raw))))
    except ValueError:
        print(f"WARNING: {name} is invalid ({raw}), ignored.")
        return None


# --- ROTATED RECTANGLE (YILDIZ_MINRECT) ---------------------------------- FLAG: default OFF. When
# off, the detection path and the payload tracker_bbox_stab remain BIT-SAME (minAreaRect is not
# called, the payload remains with the 8 element).
MINRECT_ENABLED = (os.environ.get('YILDIZ_MINRECT', '0').strip().lower()
                in ('1', 'true', 'yes', 'on'))
# REASON: target fixed wing; BANKES WHILE TURNING. Axis-aligned bbox (AABB) mixes both our own roll
# and the target's bank to the same w/h ratio and is UNSIGNED (NOT distinguishable whether it is
# turning left or right; measured: aspect signal can only be read 13% of the time, R^2=0.32). If
# cv2.minAreaRect is applied to the SAME contour, the REAL w/h of the rotated rectangle is output
# (camera roll disappears at its source) and the SIGNED angle of the LONG AXIS is measured -> target
# bank -> omega = g*tan(bank)/v.
#
# angle DEFINITION (WE DO NOT RELY on the OpenCV convention; in version 4.5.1 the range [-90,0)
# changed to (0,90] and w/h ordering returned with it -- to avoid falling into this trap angle
# minAreaRect itself NOT derived from the field 'angle', but from the corner points boxPoints()): *
# rot_w_px = LONG side of the rectangle [px] (always >= rot_h_px) * rot_h_px = SHORT side [px] *
# rot_angle_deg = angle of the LONG AXIS relative to the image horizontal, COUNTER-Clockwise positive ON
# SCREEN (atan2(-dy, dx) is used because the image y-axis is facing down). It is an AXIS angle: the
# direction (which end is the head) is unknown, so the mode is defined 180 and folds into the range
# [-90, +90).
#
# CONTINUITY: the fold only wraps as the visible long axis approaches the VERTICAL (|angle| -> 90).
# When chasing, the long axis is the wing line and the apparent angle is ~ (target bank) -/+ (fighter
# roll); There will be NO wrapping unless both of them reach ~90 degrees. The consumer must still turn
# on the 180 degree jump between successive samples (example: d = ((a2-a1+90) % 180) - 90).
#
# OFFLINE VERIFICATION (2026-08-09, 3 ellipse run: b5k2a / b5k3a / b5k3b; same HSV chain from recorded
# videos + THOUSAND ATT roll of the target is real):
#   * ROLL DECOMPOSITION WORKS: corr(rot_angle, pursuer_roll) = +0.77..+0.80, i.e. the DOMINANT term of the
#   visible axis angle is clean with our own roll and bank ~ fold(roll - rot_angle) It is extracted as
#   follows. * BUT THE REMAINING SIGNAL IS WEAK: between bank estimate and ACTUAL target bank r =
#   +0.33..+0.43 (R^2 0.11..0.18), RMS ~9-11 deg; noise per frame ~4.5 deg RMS (actual bank varies
#   between frames 0.3 deg). example = g*tan(bank)/v correlation only +0.22..+0.35. * Even the SIGN
#   rotates according to the range band (r = -0.65 at 25-40 m). Reason: range on runs 8-30 m and eye
#   angle p50 18-55 deg -- so it's not pure tail chasing; this close perspective + the contribution of
#   the fuselage/tail silhouette overwhelms the wing line. THE FULL GEOMETRIC FORWARD MODEL (actual
#   target attitude + camera attitude -> expected screen angle) explains the measured angle with r =
#   0.47..0.82: the measurement is not bad, the "bank = roll - angle" APPROACH is invalid except for
#   tail-chasing. RULING: fields opened for LOG/ANALYSIS; NOT READY to connect to MPC. Missing: (1)
#   target rotating in two directions (missions/target_s.plan) -- sign separation could NEVER be tested
#   because the target always points to the right in the ellipse; (2) inversion or gate taking into
#   account the angle of the eye (eye < ~30 deg and height > ~60 rising to r 0.56 in px).
def _frozen_rectangle(contour):
    """Applies minAreaRect to the contour; (long_px, kisa_px, axis_angle_deg) returns.

    angle is in the range [-90, +90), CCW positive ON DISPLAY. Definition documented in the block
    above. In case of error (degenerate contour) None is returned.
    """
    try:
        box_value = cv2.boxPoints(cv2.minAreaRect(contour))
    except cv2.error:
        return None
    edge = [(float(box_value[(i + 1) % 4][0]) - float(box_value[i][0]),
              float(box_value[(i + 1) % 4][1]) - float(box_value[i][1]))
             for i in range(2)]          # two consecutive edges are already PERPENDICULAR
    boy = [math.hypot(dx, dy) for dx, dy in edge]
    i_long = 0 if boy[0] >= boy[1] else 1
    dx, dy = edge[i_long]
    if boy[i_long] <= 0.0:
        return None
    # y facing DOWN -> turn screen-CCW positive with -dy
    angle_value = math.degrees(math.atan2(-dy, dx))
    angle_value = ((angle_value + 90.0) % 180.0) - 90.0          # axis angle, [-90, +90)
    return boy[i_long], boy[1 - i_long], angle_value


def hsv_detection(cv_image, color_ranges, min_blob_h=0.0, min_blob_fill=0.0,
               min_area=MIN_CONTOUR_AREA, frozen_value=False):
    """Find the largest color contour; return (x, y, w, h) or None.

    INCREASED TO MODULE LEVEL (2026-08-07). REASON: hardware/camera_bridge.py (real hardware entry
    without ROS) also uses the same detection logic. If it had remained as SwarmRedisDetector's
    method, the __init__ of that class would NOT have been accessed on the Pi because the subscriber
    ROS had installed it, and would have had to be copied -- two copies of the same logic, two
    silently decomposing truths.

    The body was moved EXACTLY; shape gate thresholds are now parameters and their defaults are
    EXACTLY the same as the old behavior (0 = off).

    If rotate=True is given, the return value is the pair ((x, y, w, h), rot); triple (or None) of
    rot _rotated_rectangle() (long_px, kisa_px, angle_deg). DEFAULT False -> return value and
    calculation EXACTLY the same as before
    (minAreaRect HIC cagrilmaz).
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower, upper = color_ranges[0]
    mask = cv2.inRange(hsv, lower, upper)
    for lower, upper in color_ranges[1:]:
        mask = mask + cv2.inRange(hsv, lower, upper)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0.0
    best_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= min_area or area <= best_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        # Shape gate: ground noise is thin/tall and low occupancy.
        if min_blob_h > 0 and h < min_blob_h:
            continue
        if min_blob_fill > 0 and w * h > 0 and area / (w * h) < min_blob_fill:
            continue
        best, best_area = (x, y, w, h), area
        best_contour = contour
    if not frozen_value:
        return best
    # minAreaRect ONLY for winning contours (not in the loop) -- single call per frame at additional cost.
    return best, (None if best_contour is None
                  else _frozen_rectangle(best_contour))


class SwarmRedisDetector:
    # --- DECISION MAKER (friend's decider.py logic) --- Transition rule: "target ~1 s positioned when it
    # remains in the frame -> imaged". WINDOW 45 -> 25 SQUARE (2026-08-07 user decision). REASON: window
    # is in SQUARES, not DURATION. Now the camera will run 30 fps (45 frames = 1.5 s) but on real hardware
    # it will run YOLO ~20 Hz; the same 45 frame there is 2.25 s, so the handoff is DELAYED by half a
    # second before the code changes. The 25 frame keeps both sides within a reasonable band: 30 fps ->
    # 0.83 s, 20 Hz -> 1.25 s. The current frame rate REMAINS at %80 (20/25), so the stability metric is
    # the same. Set with env: YILDIZ_WINDOW_FRAME / YILDIZ_WINDOW_RATIO.
    WINDOW_SIZE = max(3, int(float(os.environ.get('YILDIZ_WINDOW_FRAME', '25'))))
    TRANSITION_THRESHOLD = max(          # positioned -> current frame for display
        2, int(round(WINDOW_SIZE
                     * float(os.environ.get('YILDIZ_WINDOW_RATIO', '0.8')))))
    REVERT_THRESHOLD = 3             # low threshold for display -> location
    REVERT_DWELL_SECONDS = 2
    # Coverage thresholds are in PERCENTAGE (horizontal_coverage = w/w_img*100). The old 0.85/0.7 values
    # ​​were written as RATIO; the door always remained open and the pass was triggered even when the
    # target was 2 km away (README "Known issue"). MEASUREMENT (holder_ellipse 2026-08-03): bbox width does
    # not scale RELIABLE with range -- 204 in m (bend, planform view) when measuring 31 px In standoff (26
    # m, from behind) median 34 / max 64 px remained (coverage max %5.0). So coverage alone CANNOT be a
    # range gate; %2 is the "target appears to be of significant size" filter, the actual range gate is
    # TRANSITION_RANGE_M below (the estimator range the location writes to handoff_state; by convention, only
    # the range from 3D data is used). Dwell threshold deliberately low: handles the reverse handover
    # DETECTION LOSS (REVERT_THRESHOLD+dwell).
    MIN_COVERAGE_TRANSITION = float(os.environ.get('YILDIZ_COV_TRANSITION', '2.0'))
    MIN_COVERAGE_HOLD = float(os.environ.get('YILDIZ_COV_KAL', '0.3'))
    # AREA MEASUREMENT -- DEFAULT (2026-08-05 user decision: "35 village per meter"). The transition
    # threshold is the area of ​​the rectangle of the frame (p% x p%); 0 = OFF (reverts to old
    # horizontal-coverage behavior). MEASURED range response: p=2 -> 35 m | p=3 -> 24 m | p=4 -> 18 m
    #                           p=5 -> 14 m | p=6 -> 12 m | p=8 ->  9 m
    # (calibration: bbox_area ~ 4.65e5/r^2, n=1945 actual running frame; range response measured
    # independent of encounter type) WHY AREA: horizontal coverage SINGLE axis meter, aspect ratio of
    # target (median 2.33 ) scrolls as it changes; The field sees both axes.
    TRANSITION_AREA_PCT = float(os.environ.get('YILDIZ_TRANSITION_AREA_PCT', '2'))
    TRANSITION_RANGE_M = float(os.environ.get('YILDIZ_TRANSITION_RANGE', '60'))
    # --- SIMPLE SWITCH RULE (2026-08-07 user decision, DEFAULT ON) --- "Transfer if detection occurs in N
    # consecutive frames." In real flight, the above three-door rule (area %2 + range 60 m + 45 valid in a
    # square window 36) can delay the rev A LOT; The desired behavior in the field should be simple and
    # predictable. Doors SKIPPED in simple mode: area door, range door, 36/45 window. NOT SKIPPED:
    # manual_stop, visual_release (ISW) and DEAD-MAN gate -- so again, only WORKING is delegated to a
    # video controller. The RETURN rules (REVERT_THRESHOLD + dwell, dead-man, visual_release) NEVER
    # CHANGE; simple rule only affects positioned -> display direction. DEFAULT 0 (evening 2026-08-07,
    # user decision): the handoff rule of the version that HIT LAST EVENING goes to the field first;
    # simple rule is ready in code but CLOSED, it will be verified first. YILDIZ_TRANSITION_SIMPLE=1 to open.
    # MUST BE OPENED TOGETHER WHEN OPENING (measured, otherwise ping-pong): YILDIZ_HANDOFF_COOLDOWN_S=3 and
    # mpc_guidance MpcConfig.miss_absolute_m=300 Reason: since the simple rule removes the range gate, handoff
    # can come at 150+ m; 120 m absolute WAT limit cancels cycle at ~1 s, bbox re-engages at 0.17 s ->
    # cycle ~1.2 s, ~50 handoff per minute.
    TRANSITION_SIMPLE = (os.environ.get('YILDIZ_TRANSITION_SIMPLE', '0').strip().lower()
                   in ('1', 'true', 'yes', 'on'))
    TRANSITION_FRAME = max(1, int(float(os.environ.get('YILDIZ_TRANSITION_FRAME', '5'))))
    # HARDWARE/LOS RULE: N consecutive frames must not ONLY be detected, but also passed through the field
    # gate (large enough bbox). Exact center, target telemetry and long 20/25 windows are not sought.
    # 0=off; When on, SIMPLE comes before the rule.
    TRANSITION_LARGE_FRAME = max(
        0, int(float(os.environ.get('YILDIZ_TRANSITION_LARGE_FRAME', '0'))))
    # handoff COOL DOWN: minimum wait for re-handoff after video -> location return. The simple rule is
    # that as the target remains visible, the counter fills TRANSITION_FRAME in 30 Hz in ~0.17 s; Immediate
    # delegation back to the ceded MPC with MISS produces ping-pong (measured: cycles ~1.2 s, ~50 handoff
    # per minute). DEFAULT 0 = last night's behavior; The simple rule is to open with
    # YILDIZ_HANDOFF_COOLDOWN_S=3.
    HANDOFF_COOLDOWN_S = float(os.environ.get('YILDIZ_HANDOFF_COOLDOWN_S', '0'))
    HANDOFF_STALE_S = 3.0              # If handoff_state is older than this, ignore range
    # DEAD-MAN: If the video controller's heartbeat is older than this, it is considered "none".
    # Compatible with visual_base.ALIVE_TTL_S (2s); TTL already lowers the key, this threshold is
    # only the second safety.
    ALIVE_STALE_S = 2.5

    def __init__(self, topic, display=True, record_path=None, record_fps=0.0,
                 gimbal=None, attitude_value=None, aim_trim=None, gimbal_log=None,
                 tilt_reader=None, tilt_commander=None, tilt_tracking=None):
        self.gimbal = gimbal
        self.attitude_value = attitude_value
        self.aim_trim = aim_trim
        # PHASE O: If tilt_tracking is given, the tilt target is moved to the measured elevation of the target in
        # each frame (returns to standoff angle on detection loss)
        self.tilt_tracking = tilt_tracking
        self._tracking_last_t = None
        # PHYSICAL GIMBAL (gimbal branch): If tilt_commander/reader is supplied, the camera is on the stabilized
        # physical gimbal; Instead of the fixed mount, the LIVE joint angle (joint_angle(eps, pitch, roll))
        # enters the chain.
        self.tilt_reader = tilt_reader
        self.tilt_commander = tilt_commander
        self._last_joint_deg = None
        # YILDIZ_MINRECT: opaque rectangle (long, short, angle) of the last frame or None. When the flag is
        # off, NEVER is written, it always remains None.
        self._last_rot = None
        self._last_frame_t = None
        self._log_f = self._log_w = None
        self._log_counter = 0          # periodic flush counter (20 per line)
        if gimbal_log:
            self._log_f = open(gimbal_log, 'w', newline='')
            self._log_w = csv.writer(self._log_f)
            self._log_w.writerow(['t_frame', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h',
                                  'roll_deg', 'pitch_deg', 'range_m_value',
                                  'raw_ex_deg', 'raw_ey_deg',
                                  'stab_ex_deg', 'stab_ey_deg',
                                  'aim_deg', 'aim_active_deg',
                                  'ros_stamp', 'interp_gap_ms', 'latency_ms',
                                  'tilt_cmd_deg', 'tilt_status_deg',
                                  'tilt_age_ms', 'joint_deg'])
            print(f"Gimbal logu: {gimbal_log}", flush=True)
        self.topic = topic
        self.display = display
        self.record_path = record_path
        self.record_fps = record_fps
        self.draw = display or (record_path is not None)

        print("Redis sunucusuna baglaniliyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.ping()
            self.r.set('command_authority', 'position_based')
            print("Redis connection successful. Broadcast channel: 'tracker_bbox'")
        except Exception as exc:
            raise SystemExit(f"Redis connection error: {exc}")

        self.target_color, self.color_ranges = build_color_ranges(
            os.environ.get('YILDIZ_TARGET_COLOR', DEFAULT_TARGET_COLOR),
            _opt_int('YILDIZ_HSV_SMIN'), _opt_int('YILDIZ_HSV_VMIN'))
        window_value = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                             for lo, hi in self.color_ranges)
        print(f"Target color: {self.target_color.upper()} -> {window_value}")

        # Shape gate: if not given, behavior is exactly as before.
        self.min_blob_h = float(os.environ.get('YILDIZ_MIN_BLOB_H', '0') or 0)
        self.min_blob_fill = float(os.environ.get('YILDIZ_MIN_BLOB_FILL', '0') or 0)
        if self.min_blob_h > 0 or self.min_blob_fill > 0:
            print(f"Shape door open: min_h={self.min_blob_h:.0f} px, "
                  f"min_doluluk={self.min_blob_fill:.2f}")

        # --- decision maker internal state ---
        self.current_mode = 'position_based'
        self.decision_window = deque(maxlen=self.WINDOW_SIZE)
        self.revert_pending_since = None
        # SIMPLE RULE counter: consecutive VALID DETECT frame (does not need to pass through area/range gate);
        # invalid single frame zeros.
        self.consecutive_valid = 0
        self.consecutive_large = 0
        self._transition_reason = ''       # rule that triggered the last mode change
        self._last_to_position_turn = 0.0  # monotonic; 0 = no return since startup
        if self.TRANSITION_LARGE_FRAME:
            print(f"Transition rule: LARGE_FRAME -- consecutive "
                  f"{self.TRANSITION_LARGE_FRAME} detect + area "
                  f"%{self.TRANSITION_AREA_PCT:g} (center/range not searched)")
        elif self.TRANSITION_SIMPLE:
            print(f"Transition rule: SIMPLE -- consecutive {self.TRANSITION_FRAME} valid "
                  f"detection frame (area/range/window doors SKIPPED). "
                  f"For old rule YILDIZ_TRANSITION_SIMPLE=0")
        else:
            print(f"Migration rule: OLD -- {self.TRANSITION_THRESHOLD}/"
                  f"{self.WINDOW_SIZE} square + area {self.TRANSITION_AREA_PCT:g} + "
                  f"range <= {self.TRANSITION_RANGE_M:.0f} m")

        # --- measurement ---
        self.frame_count = 0
        self.detect_count = 0
        self.first_frame_time = None
        # The frame size updates with the first frame; field threshold uses this.
        self._framing_w, self._framing_h = 1280.0, 720.0
        self.last_report_time = time.time()
        self.last_report_frames = 0

        self.video_writer = None
        self._record_buffer = []
        # The line of code is only meaningful if DRAWING; Only then does the thread start (the headless
        # broadcast path is not affected).
        self.code_value = None
        self.last_stab = None

        if self.draw:
            self.code_value = RunningCode()
            self.code_value.start()

        ros_required('bbox_to_redis.SwarmRedisDetector (ROS Image abonesi)')
        rospy.init_node('yildizlar_bbox', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.image_callback,
                                          queue_size=1)
        print(f"ROS node started, waiting for '{topic}'...")

    # ---------------- engagement decision ----------------

    def _handoff_range(self):
        """Estimator range [m] of the location at 'handoff_state' or None.

        t_mono is time.monotonic() in both processes; On Linux, CLOCK_MONOTONIC is compared directly
        because it is system wide.
        """
        try:
            raw_value = self.r.get('handoff_state')
            if not raw_value:
                return None
            handoff = json.loads(raw_value)
            if time.monotonic() - float(handoff['t_mono']) > self.HANDOFF_STALE_S:
                return None
            return float(handoff['range_m'])
        except Exception:
            return None

    def _visual_alive(self):
        """Has the display controller shown signs of life in the last ALIVE_STALE_S? Since the key is written
as TTL, it disappears automatically if the process dies; Also a second freshness check is done with
t_mono (to avoid relying on the rounding allowance of TTL).

        If Redis cannot be accessed, True is returned: the authorization mechanism itself is already
        based on Redis, without Redis this door cannot make decisions and the OLD BEHAVIOR is
        preserved (so that the door does not accidentally lock into 'position' all the time).
        """
        try:
            raw_value = self.r.get('visual_alive')
            if not raw_value:
                return False
            d = json.loads(raw_value)
            return (time.monotonic() - float(d['t_mono'])) <= self.ALIVE_STALE_S
        except Exception:
            return True

    def _evaluate_frame(self, valid_detection, coverage, area_px2=None):
        """Does this square count for 'pass'?

        Two criteria, selected with YILDIZ_TRANSITION_AREA_PCT : * Default horizontal coverage: w /
        framing_genisligi * 100 >= threshold. * Area criterion when YILDIZ_TRANSITION_AREA_PCT > 0 :
        determine whether the bbox area exceeds a rectangle spanning p % o f frame width and p % o f
        frame height. With p= 5 , the threshold is 0.05 * 1280 x 0.05 * 720 = 2304 px ^ 2 .

        WHY AREA OPTION: horizontal coverage measures single axis and the aspect ratio of the target
        varies with the aspect (measured: w/h median 2.33 ). The field sees both axes. MEASURED
        calibration (n= 1945 , actual runs): bbox_area ~ 4.65e5 / r^ 2 [ px ^ 2 , r meters] bbox_w ~
        1015 / r [ px ] i.e. % 2 horizontal coverage -> ~ 40 m ;  %5x % 5 area -> ~ 14 m. The range
        equivalent of area threshold is INDEPENDENT of the encounter type (tail 14.3 / cross 13.8 /
        head-on 14.1 m).
        """
        if not valid_detection:
            return False
        if self.TRANSITION_AREA_PCT > 0.0 and area_px2 is not None:
            if self.current_mode == 'position_based':
                return area_px2 >= self._transition_area_threshold()
            # FAIL criterion remains in coverage (low threshold, hysteresis)
            return coverage >= self.MIN_COVERAGE_HOLD
        threshold = (self.MIN_COVERAGE_TRANSITION if self.current_mode == 'position_based'
                     else self.MIN_COVERAGE_HOLD)
        return coverage >= threshold

    def _transition_area_threshold(self):
        """The area of ​​the rectangle (p% x p%) is [px^2]. Frame size from the first frame."""
        p = self.TRANSITION_AREA_PCT / 100.0
        return (p * self._framing_w) * (p * self._framing_h)

    def _make_decision(self, valid_count):
        now = time.time()
        try:
            val = self.r.get('manual_stop')
            manual = bool(val) and val.decode('utf-8') == '1'
        except Exception:
            manual = False

        if manual:
            self.revert_pending_since = None
            if self.current_mode != 'position_based':
                self._transition_reason = 'manual_stop'
                return 'position_based', True
            return 'position_based', False

        # MISS: the controller on the video side (mpc_guidance) voluntarily gave up the authority (see
        # guidance_allstar/visual_base.py, under 'cmd = self.k.command(o)'). Freshness window 2 s: an
        # older key is ignored because it may have been switched back to 'display' before this decision cycle
        # sees it.
        try:
            raw_value = self.r.get('visual_release')
            if raw_value:
                d = json.loads(raw_value)
                if (self.current_mode == 'visual'
                        and time.monotonic() - float(d['t_mono']) < 2.0):
                    self.r.delete('visual_release')
                    self._transition_reason = 'visual_release (MISS)'
                    return 'position_based', True
        except Exception:
            pass

        # DEAD-MAN'S DOOR ( 2026 - 08 - 05 ): if the video controller is NOT WORKING, authority is not
        # transferred to it; If it has been transferred, it will be taken back. The controller refreshes the
        # key ' visual_alive ' with TTL ( visual_base . _notify_alive ). If the process was never
        # started or hung at _connect(), there is NO key. FAILURE LOG: without this gate, authority was
        # delegated while the controller was not working and NO ONE commanded the vehicle -- range 40 -> 124
        # opened in m at 5.9 s, flutter yaw increased by 6 .6x. The external symptom was " MPC flickering, not
        # following the target"; whereas MPC was not running at all.
        if not self._visual_alive():
            if self.current_mode == 'visual':
                print("[DECISION] video controller NOT RESPONDING "
                      "('visual_alive' stale/nonexistent) -> returning to positional",
                      flush=True)
                self._transition_reason = 'dead-man (visual_alive corny)'
                return 'position_based', True
            # Do not count frames seen while the controller is away as credit towards a future engagement.
            # Otherwise, when the process is opened later, the counter may be 200+ and it will be handoff without
            # even waiting for the first NEW frame.
            self.consecutive_valid = 0
            self.consecutive_large = 0
            if time.time() - getattr(self, '_last_alive_log', 0) > 10.0:
                self._last_alive_log = time.time()
                print("[DECISION] NO video controller "
                      "(key 'visual_alive' is stale/non-existent); transition "
                      "blocked. Did you forget to start video guidance?",
                      flush=True)
            return 'position_based', False

        # --- SIMPLE RULE: positioned -> displayed --- Only shorts the FORWARD direction. It doesn't wait for
        # the window to fill (45 frame @30 Hz = 1.5 s wait), it doesn't look at area/range doors. The return
        # path remains UNCHANGED below: window + REVERT_THRESHOLD + dwell processes while displayed in simple
        # mode (takes 1.5 s to fill as the window is cleared at handoff; during this time return is only with
        # dead-man/visual_release/manual -- OLD BEHAVIOR).
        if ((self.TRANSITION_LARGE_FRAME or self.TRANSITION_SIMPLE)
                and self.current_mode == 'position_based'):
            self.revert_pending_since = None
            waiting = time.monotonic() - getattr(
                self, '_last_to_position_turn', 0.0)
            if waiting < self.HANDOFF_COOLDOWN_S:
                if time.time() - getattr(self, '_last_cooldown_log', 0) > 5.0:
                    self._last_cooldown_log = time.time()
                    print(f"[DECISION] handoff cooling: {waiting:.1f}/"
                          f"{self.HANDOFF_COOLDOWN_S:.0f} s, waiting for migration",
                          flush=True)
                return 'position_based', False
            counter = (self.consecutive_large if self.TRANSITION_LARGE_FRAME
                     else self.consecutive_valid)
            required_item = (self.TRANSITION_LARGE_FRAME if self.TRANSITION_LARGE_FRAME
                       else self.TRANSITION_FRAME)
            if counter >= required_item:
                self._transition_reason = (
                    f"large_frame({counter} sequential, area "
                    f"%{self.TRANSITION_AREA_PCT:g})" if self.TRANSITION_LARGE_FRAME
                    else f"simple ({counter} sequential frame)")
                return 'visual', True
            return 'position_based', False

        if len(self.decision_window) < self.WINDOW_SIZE:
            return self.current_mode, False

        if self.current_mode == 'position_based':
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                # RANGE GATE: since the coverage depends on the aspect (200 + % 2.4 measured even at m in the bend)
                # the estimator range of the positioned for passing must also be close.  If handoff_state is
                # missing/stale (test alone without positional running) the port is skipped -- legacy behavior.
                range_value = self._handoff_range()
                if range_value is not None and range_value > self.TRANSITION_RANGE_M:
                    if time.time() - getattr(self, '_last_range_log', 0) > 5.0:
                        self._last_range_log = time.time()
                        print(f"[DECISION] window full ( {valid_count} /"
                              f"{self.WINDOW_SIZE}) but range {range_value:.0f} m > "
                              f"{self.TRANSITION_RANGE_M:.0f} m, waiting for migration",
                              flush=True)
                    return 'position_based', False
                self._transition_reason = (
                    f"old({valid_count}/{self.WINDOW_SIZE} square, area "
                    f"%{self.TRANSITION_AREA_PCT:g}, range gate "
                    f"{self.TRANSITION_RANGE_M:.0f} m)")
                return 'visual', True
            return 'position_based', False

        # imaged -> positioned: low threshold + dwell
        if valid_count <= self.REVERT_THRESHOLD:
            if self.revert_pending_since is None:
                self.revert_pending_since = now
                print(f"[DECISION] Subthreshold ( {valid_count} / {self.WINDOW_SIZE} ), "
                      f"dwell basladi ({self.REVERT_DWELL_SECONDS}s)", flush=True)
            if now - self.revert_pending_since >= self.REVERT_DWELL_SECONDS:
                self.revert_pending_since = None
                self._transition_reason = (f"detection loss ( {valid_count} /"
                                     f"{self.WINDOW_SIZE} + dwell)")
                return 'position_based', True
            return 'visual', False

        if self.revert_pending_since is not None:
            print(f"[DECISION] Target appeared back ({valid_count}/{self.WINDOW_SIZE}), "
                  f"dwell iptal", flush=True)
            self.revert_pending_since = None
        return 'visual', False

    def _update_decision(self, valid_detection, coverage, area_px2=None):
        transition_valid = self._evaluate_frame(valid_detection, coverage,
                                              area_px2)
        self.decision_window.append(transition_valid)
        valid_count = sum(self.decision_window)
        # SIMPLE RULE counter: RAW detection validity is counted (including shape gate, EXCEPT area/range
        # gate). A single invalid frame resets.
        if valid_detection:
            self.consecutive_valid += 1
        else:
            self.consecutive_valid = 0
        if self.current_mode == 'position_based' and transition_valid:
            self.consecutive_large += 1
        else:
            self.consecutive_large = 0
        new_mode, changed_value = self._make_decision(valid_count)
        if changed_value:
            previous = self.current_mode
            self.current_mode = new_mode
            self.decision_window.clear()
            self.revert_pending_since = None
            self.consecutive_valid = 0
            self.consecutive_large = 0
            if new_mode == 'position_based':
                self._last_to_position_turn = time.monotonic()
            try:
                self.r.set('command_authority', self.current_mode)
                # Which rule triggered it? visual_base writes this to the detail of event 'handoff_received'
                # (LOG_DICTIONARY.md).
                self.r.set('transition_reason', self._transition_reason or 'bilinmiyor')
            except Exception:
                pass
            print(f"[DECISION] >>> MODE CHANGED: {previous} -> {new_mode} "
                  f"[{self._transition_reason or 'bilinmiyor'}]", flush=True)
        return self.current_mode, valid_count

    # ---------------- detection ----------------

    def _detect(self, cv_image):
        """Find the largest color contour; return (x, y, w, h) or None.

        Moved to hsv_detection() at the body module level (so that the hardware bridge also calls the
        SAME function); behavior is exactly the same.

        When YILDIZ_MINRECT=1, in addition, the opaque rectangle of the same contour is written to
        self._last_rot (read on the broadcast side)."""
        if not MINRECT_ENABLED:
            return hsv_detection(cv_image, self.color_ranges,
                              self.min_blob_h, self.min_blob_fill)
        box, rot = hsv_detection(cv_image, self.color_ranges,
                              self.min_blob_h, self.min_blob_fill, frozen_value=True)
        self._last_rot = rot
        return box

    def image_callback(self, data):
        # ONE COMMON TIME: everything is stamped with time.monotonic() - both frames and attitude instances.
        # It does NOT depend on the simulation clock (header.stamp), so the same code runs on the real uck.
        # header.stamp is stored for informational purposes only.
        t_frame = time.monotonic()
        t_capture = data.header.stamp.to_select() or time.time()
        t_start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr(exc)
            return

        h_img, w_img = cv_image.shape[:2]
        self.frame_count += 1
        if self.first_frame_time is None:
            self.first_frame_time = t_start
            print(f"FIRST FRAME TAKEN: {w_img}x{h_img}", flush=True)

        box = self._detect(cv_image)
        valid_detection = box is not None
        horizontal_coverage = 0.0

        if valid_detection:
            x, y, w, h = box
            horizontal_coverage = (w / w_img) * 100.0
            self.detect_count += 1
            bbox = [int(x), int(y), int(w), int(h),
                    round(horizontal_coverage, 3), 1, round(t_capture, 4)]
            self.r.publish('tracker_bbox', json.dumps(bbox))

            # --- VIRTUAL GIMBAL --- The raw bbox channel is preserved AS IS (guidance codes under stars25/ read
            # it); stabilized values ​​are printed on a SEPARATE channel so that the two paths can be compared
            # side by side.
            attitude_sample = (self.attitude_value.get_attitude(t_frame)
                           if (self.attitude_value is not None and self.attitude_value.ready) else None)
            if self.gimbal is not None and attitude_sample is not None:
                roll_i, pitch_i, t_target = attitude_sample
                mx, my = x + w / 2.0, y + h / 2.0
                # The range is estimated from the width bbox; For reception absorption ONLY (guidance range gets from
                # telemetry).
                range_value = self.gimbal.range_prediction(w)
                # PHYSICAL GIMBAL: live joint angle instead of fixed mount. If the joint is None (tilt mode off) the
                # old way works exactly the same.
                joint, tilt_eps, tilt_age = self._joint_calculate(roll_i, pitch_i)
                self._last_joint_deg = joint
                sx, sy = self.gimbal.stabilize(mx, my, roll_i, pitch_i, range_value,
                                               joint_deg=joint)
                ex, ey = self.gimbal.angle_error_value(mx, my, roll_i, pitch_i, range_value,
                                                joint_deg=joint)
                # RAW error: the magnitude my steering would see if there was NO software de-rotation. CAUTION (gimbal
                # branch): when there is a physical gimbal the "raw" is already the output of the stabilized camera --
                # the body pitch is NO LONGER VISIBLE in raw_ey (criteria gimbal_evidence.py should be updated
                # accordingly; old criterion r_sy < 0.5*r_hy remained empty).
                raw_ex = math.degrees(math.atan((mx - self.gimbal.cx) / self.gimbal.fx))
                raw_ey = math.degrees(math.atan((my - self.gimbal.cy) / self.gimbal.fy))
                self.last_stab = (sx, sy, ex, ey)

                # --- AIM TRIM (low bandwidth, clamped) ---
                now_value = time.time()
                dt = 0.0 if self._last_frame_t is None else (now_value - self._last_frame_t)
                self._last_frame_t = now_value
                if self.aim_trim is not None and 0 < dt < 1.0:
                    new_aim_value = self.aim_trim.update_value(ey, range_value, dt)
                    if new_aim_value != self.gimbal.aim_pitch_deg:
                        self.gimbal.aim_pitch_deg = new_aim_value

                if self._log_w is not None:
                    self._log_w.writerow([
                        f"{t_frame:.6f}", f"{mx:.2f}", f"{my:.2f}", w, h,
                        f"{math.degrees(roll_i):.3f}",
                        f"{math.degrees(pitch_i):.3f}",
                        '' if range_value is None else f"{range_value:.1f}",
                        f"{raw_ex:.4f}", f"{raw_ey:.4f}",
                        f"{ex:.4f}", f"{ey:.4f}",
                        f"{self.gimbal.aim_pitch_deg:.3f}",
                        f"{self.gimbal.aim_active_deg(range_value):.3f}",
                        f"{t_capture:.4f}", f"{self.attitude_value.last_gap_s*1000:.2f}",
                        f"{self.attitude_value.latency_s*1000:.1f}",
                        ('' if self.tilt_commander is None
                         or self.tilt_commander.target_deg is None
                         else f"{self.tilt_commander.target_deg:.3f}"),
                        '' if tilt_eps is None else f"{tilt_eps:.4f}",
                        '' if tilt_age is None else f"{tilt_age*1000:.0f}",
                        '' if joint is None else f"{joint:.4f}"])
                    # PERIODIC FLUSH (2026-08-07): this CSV used to be flushed only at shutdown; In the crash/SIGKILL, the
                    # last block buffer (~8 KB, 30, ~2 s in Hz) was not written to disk at all -- i.e., the partial loss
                    # of the tilt command-vs-committed record closest to the moment of the crash. Same pattern as mpc_diagnostic
                    # and CSV with image: 20 one in a row.
                    self._log_counter += 1
                    if self._log_counter % 20 == 0:
                        self._log_f.flush()
                # --- PHASE C: TILT TRACKING --- ey measured relative to horizon (live-joint chain) -> target's world
                # ascension = -ey, INDEPENDENT FROM TILT. So this is not a closed feedback, but a filtered tracking of
                # the measured quantity.
                if self.tilt_tracking is not None and self.tilt_commander is not None:
                    now_t = time.monotonic()
                    dt_t = (1.0 / 30 if self._tracking_last_t is None
                            else now_t - self._tracking_last_t)
                    self._tracking_last_t = now_t
                    self.tilt_commander.target_value(
                        self.tilt_tracking.update_value(-ey, dt_t, now_value=now_t))

                # 8. element (PHASE O): camera elevation used by the chain in that frame -- builds guidance ey_ref
                # from it (Measurement.tilt_deg).
                _stab_elevation = [round(sx, 2), round(sy, 2), int(w), int(h),
                             round(ex, 4), round(ey, 4), round(t_capture, 4),
                             None if tilt_eps is None else round(tilt_eps, 3)]
                # 9-11. elements (YILDIZ_MINRECT): turned rectangle -- rot_w_px (long side), rot_h_px (short side),
                # rot_angle_deg (screen-CCW angle of long axis, [-90,+90), see fig. _rotated_rectangle). NOT ADDED WHEN
                # FLAG IS OFF: bit-same as old consumers, with payload element 8.
                if MINRECT_ENABLED:
                    _rot = self._last_rot
                    _stab_elevation += ([None, None, None] if _rot is None else
                                  [round(_rot[0], 2), round(_rot[1], 2),
                                   round(_rot[2], 3)])
                self.r.publish('tracker_bbox_stab', json.dumps(_stab_elevation))
            self.r.set('timing_bbox_to_redis', f"{(time.time() - t_start) * 1000:.2f}")

        # PHASE O loss path: if there is no detection, let the follow law run the loss counter (loss_hold_s
        # holds, then slow return to standoff angle)
        if ((not valid_detection) and self.tilt_tracking is not None
                and self.tilt_commander is not None):
            now_t = time.monotonic()
            dt_t = (1.0 / 30 if self._tracking_last_t is None
                    else now_t - self._tracking_last_t)
            self._tracking_last_t = now_t
            self.tilt_commander.target_value(
                self.tilt_tracking.update_value(None, dt_t, now_value=now_t))

        self._framing_w, self._framing_h = float(w_img), float(h_img)
        area_px2 = None
        if valid_detection:
            _bx, _by, _bw, _bh = box
            area_px2 = float(_bw) * float(_bh)
        command_authority, valid_count = self._update_decision(valid_detection,
                                                           horizontal_coverage,
                                                           area_px2)
        self._report(valid_detection, box, horizontal_coverage)

        if self.draw:
            self._draw(cv_image, box, command_authority, valid_count,
                       horizontal_coverage, w_img, h_img)

    def _joint_calculate(self, roll_i, pitch_i):
        """Gazebo returns the joint angle relative to the body from camera elevation.

        ``TiltStateReader`` reads topic Gazebo ``gimbal_tilt_status``. In stabilized mode,
        ``GimbalSmall2dPlugin`` writes the WORLD elevation of the camera to this topic, not the
        joint angle. Therefore, both the fresh status and the stale-status backup command should be
        converted to q with ``joint_angle``.

        The actual ArduPilot servo - mount stat has a different convention: it is the joint angle
        relative to the body. That path is covered separately in ``hardware/camera_bridge.py``
        without ROS; Hardware comment cannot be moved here.
        """
        if self.tilt_reader is None:
            return None, None, None
        age_value = self.tilt_reader.age_s_value()
        if age_value is not None and age_value < 1.5 and self.tilt_reader.value_rad is not None:
            eps = math.degrees(self.tilt_reader.value_rad)
        elif (self.tilt_commander is not None
              and self.tilt_commander.target_deg is not None):
            eps = self.tilt_commander.target_deg
        else:
            return None, None, age_value
        return joint_angle(eps, pitch_i, roll_i), eps, age_value

    def _report(self, valid_detection, box, coverage):
        """One summary line per second: fps + detection rate (for log file)."""
        now = time.time()
        if valid_detection:
            x, y, w, h = box
            print(f"TARGET center=({x + w // 2},{y + h // 2}) bbox=({x},{y},{w},{h}) "
                  f"cov={coverage:.2f}% t_unix={now:.3f}", flush=True)
        if now - self.last_report_time >= 5.0:
            frames = self.frame_count - self.last_report_frames
            fps = frames / (now - self.last_report_time)
            ratio_value = (self.detect_count / self.frame_count * 100) if self.frame_count else 0
            ek = ''
            if self.last_stab is not None and self.attitude_value is not None:
                sx, sy, ex, ey = self.last_stab
                ek = (f" | gimbal: virtual=( {sx:.0f} , {sy:.0f} ) "
                      f"error=({ex:+.2f},{ey:+.2f}) deg "
                      f"attitude=({math.degrees(self.attitude_value.roll):+.1f},"
                      f"{math.degrees(self.attitude_value.pitch):+.1f})")
                if self.aim_trim is not None:
                    ek += (f" aim={self.gimbal.aim_pitch_deg:+.2f}"
                           f"{'[K]' if self.aim_trim.at_clamp else ''}")
                if self.tilt_reader is not None:
                    age_value = self.tilt_reader.age_s_value()
                    cmd = (self.tilt_commander.target_deg
                           if self.tilt_commander is not None else None)
                    # Gazebo status is WORLD elevation. The report shows both the command/status in the same frame and the
                    # resulting body-by-body joint pains side by side.
                    st_eps = (None if self.tilt_reader.value_rad is None
                              else math.degrees(self.tilt_reader.value_rad))
                    q_cmd = (None if cmd is None else
                             joint_angle(cmd, self.attitude_value.pitch,
                                         self.attitude_value.roll))
                    q_st = (None if st_eps is None else
                            joint_angle(st_eps, self.attitude_value.pitch,
                                        self.attitude_value.roll))
                    ek += (f" tilt_world={'-' if cmd is None else f'{cmd:+.1f}'}"
                           f"/{'-' if st_eps is None else f'{st_eps:+.1f}'}"
                           f" joint={'-' if q_st is None else f'{q_st:+.1f}'}"
                           f"/{'-' if q_cmd is None else f'{q_cmd:+.1f}'}deg")
                    # dead-man-lesson (visual-dead-man-switch): silent deviation/staleness shout in SUMMARY
                    if (st_eps is not None and cmd is not None
                            and abs(st_eps - cmd) > 2.0):
                        ek += " [TILT DEVIATION WARNING]"
                    if age_value is None or age_value > 3.0:
                        ek += " [TILT STATUS STALE]"
            print(f"[SUMMARY] frame={self.frame_count} fps={fps:.1f} "
                  f"detection_ratio=%{ratio_value:.1f} mode={self.current_mode}{ek}", flush=True)
            self.last_report_time = now
            self.last_report_frames = self.frame_count

    # ---------------- drawing ----------------

    def _draw(self, cv_image, box, command_authority, valid_count, coverage,
              w_img, h_img):
        FONT, AA = cv2.FONT_HERSHEY_DUPLEX, cv2.LINE_AA

        # Strike area (middle band of the frame)
        cv2.rectangle(cv_image, (int(w_img * 0.25), int(h_img * 0.10)),
                      (int(w_img * 0.75), int(h_img * 0.90)), (0, 255, 255), 1)
        cv2.putText(cv_image, "Strike Zone", (int(w_img * 0.25) + 4,
                    int(h_img * 0.10) + 14), FONT, 0.35, (0, 255, 255), 1, AA)

        if box is not None:
            x, y, w, h = box
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.drawMarker(cv_image, (x + w // 2, y + h // 2), (255, 0, 255),
                           cv2.MARKER_CROSS, 12, 1)

        # --- VIRTUAL GIMBAL VISUALIZATION --- To show the gimbal WORKING in the video: virtual horizon line
        # (it tilts/shifts in the frame as the body tilts - the de-rotation itself), virtual frame center, and
        # virtual counterpart of the target.
        if self.gimbal is not None and self.attitude_value is not None and self.attitude_value.ready:
            g, tt = self.gimbal, self.attitude_value
            _sample = tt.get_attitude(time.monotonic())
            roll, pitch = (_sample[0], _sample[1]) if _sample else (tt.roll, tt.pitch)
            # Horizon: trace of points with ascension 0 in the raw frame. Physical gimbal mode uses live
            # articulation: the horizon now sits at a fixed height in the frame, but is tilted with the roll --
            # visual proof that the gimbal is working.
            _joint = self._last_joint_deg
            points_value = []
            for lateral in range(-30, 31, 3):
                pt = g.pixel_generate(0.0, lateral, roll, pitch, joint_deg=_joint)
                if pt and -2000 < pt[0] < 3000 and -2000 < pt[1] < 3000:
                    points_value.append((int(pt[0]), int(pt[1])))
            for i in range(1, len(points_value)):
                cv2.line(cv_image, points_value[i - 1], points_value[i], (0, 200, 255), 1, AA)
            if points_value:
                cv2.putText(cv_image, "horizon", (points_value[0][0] + 4, points_value[0][1] - 6),
                            FONT, 0.4, (0, 200, 255), 1, AA)
            # Virtual frame center (aim applied): -aim rising relative to the horizon
            center = g.pixel_generate(-g.aim_pitch_deg, 0.0, roll, pitch,
                                   joint_deg=_joint)
            if center and 0 <= center[0] < w_img and 0 <= center[1] < h_img:
                cv2.drawMarker(cv_image, (int(center[0]), int(center[1])),
                               (0, 255, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
                cv2.putText(cv_image, "virtual center",
                            (int(center[0]) + 14, int(center[1]) + 4),
                            FONT, 0.4, (0, 255, 255), 1, AA)
            if self.last_stab is not None:
                sx, sy, ex, ey = self.last_stab
                cv2.putText(cv_image, f"gimbal error: {ex:+.2f} / {ey:+.2f} deg",
                            (8, h_img - 34), FONT, 0.5, (0, 255, 255), 1, AA)
            cv2.putText(cv_image,
                        f"roll {math.degrees(roll):+5.1f}  pitch {math.degrees(pitch):+5.1f}"
                        f"  aim {g.aim_pitch_deg:+.2f} delay {tt.latency_s*1000:.0f}ms",
                        (8, h_img - 12), FONT, 0.5, (200, 200, 200), 1, AA)

        overlay = cv_image.copy()
        cv2.rectangle(overlay, (0, 0), (w_img, 72), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, cv_image, 0.45, 0, cv_image)

        if command_authority == 'visual':
            hud_color, mission_value = (0, 255, 0), "VISUAL_GUIDANCE"
        else:
            hud_color, mission_value = (0, 165, 255), "POSITION_BASED"
        cv2.putText(cv_image, f"MODE: {mission_value}", (8, 20), FONT, 0.5, hud_color, 1, AA)
        if self.code_value is not None:
            cv2.putText(cv_image, self.code_value.text, (8, 62), FONT, 0.45,
                        (255, 255, 255), 1, AA)

        target_text = "TARGET: FOUND" if box is not None else "TARGET: NONE"
        target_color_value = (0, 255, 0) if box is not None else (100, 100, 255)
        cv2.putText(cv_image, target_text, (8, 40), FONT, 0.45, target_color_value, 1, AA)

        window_text_value = f"Window: {valid_count} / {len(self.decision_window)}"
        (tw, _), _ = cv2.getTextSize(window_text_value, FONT, 0.4, 1)
        cv2.putText(cv_image, window_text_value, (w_img - tw - 10, 20), FONT, 0.4,
                    (180, 180, 180), 1, AA)
        if box is not None:
            cov_text = f"Cov: {coverage:.2f}%"
            (tw2, _), _ = cv2.getTextSize(cov_text, FONT, 0.4, 1)
            cv2.putText(cv_image, cov_text, (w_img - tw2 - 10, 40), FONT, 0.4,
                        (180, 220, 180), 1, AA)

        if self.record_path is not None:
            self._record(cv_image, w_img, h_img)
        if self.display:
            cv2.imshow("Yildizlar bbox", cv_image)
            cv2.waitKey(1)

    # This is where we accumulate the frames before the printer is installed (see error below).
    MEASUREMENT_DURATION_S = 2.0

    def _record(self, cv_image, w_img, h_img):
        """bbox write the drawn frame to the video.

        BUG AND ITS FIXATION (2026-08-01): printer was being installed on the FIRST frame; Since the
        elapsed time at that moment was ~0, it fell into the "If there is no measurement, assume 5
        fps" branch and 5 fps was written in the header of the file. Since the cameras were 30 Hz,
        the recordings were played in 6 FLOOR SLOW MOTION (measurement: 10913 frame in the file
        resulting from the run of 360 s, but fps=5 -> file 2183 s appears in the title). WRITER was
        at fault, not the camera. Solution: accumulate MEASUREMENT_DURATION_S frames before the printer is
        installed, measure the actual speed, then install the printer at that speed and empty the
        accumulation. It can still be fixed by hand with --record-fps.
        """
        if self.video_writer is None:
            if self.record_fps > 0:
                fps = self.record_fps
            else:
                elapsed_item = time.time() - (self.first_frame_time or time.time())
                if elapsed_item < self.MEASUREMENT_DURATION_S:
                    self._record_buffer.append(cv_image.copy())
                    return
                # frame_count has been increasing since the first frame; This is the measured speed.
                fps = self.frame_count / max(elapsed_item, 1e-3)
                fps = max(1.0, min(60.0, fps))
            self.video_writer = cv2.VideoWriter(
                self.record_path, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (w_img, h_img))
            print(f"Video kaydi: {self.record_path} ({w_img}x{h_img} @ "
                  f"{fps:.1f} fps from {len(self._record_buffer)} frame buffer)",
                  flush=True)
            for frame_value in self._record_buffer:
                self.video_writer.write(frame_value)
            self._record_buffer = []
        self.video_writer.write(cv_image)

    def close(self):
        if self.code_value is not None:
            self.code_value.stop()
        if self.tilt_commander is not None:
            self.tilt_commander.stop_value()
        if self.tilt_reader is not None:
            self.tilt_reader.stop_value()
        if self.attitude_value is not None:
            self.attitude_value.log_close()
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
        if self.video_writer is not None:
            self.video_writer.release()
            print(f"Video kaydedildi: {self.record_path}", flush=True)
        cv2.destroyAllWindows()


def main():
    # ROS path: this script itself is a ROS Image subscriber. ROS otherwise exit clearly BEFORE parsing
    # the arguments.
    ros_required('bbox_to_redis.py (ROS Image abonesi)')
    root = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default=os.environ.get(
        'YILDIZ_CAM_TOPIC', '/drone_1/webcam/image_raw'),
        help='to be listened to ROS camera topic\'i')
    parser.add_argument('--no-display', action='store_true',
                        help='OpenCV window opening (headless)')
    parser.add_argument('--record', nargs='?', const='', default=None,
                        metavar='FILE',
                        help='bbox save crossed out frames; if way is not given '
                             'videos/guidance_YYYYmmdd_HHMMSS.mp4')
    parser.add_argument('--record-fps', type=float, default=0.0,
                        help='record fps (use 0 = measured speed)')
    parser.add_argument('--mavlink-port', type=int, default=0,
                        help='ATTITUDE port for virtual gimbal; 0 = gimbal off')
    parser.add_argument('--mount', type=float, default=30.0,
                        help='physical camera mounting angle (degrees, + = up). '
                             'ONLY in the path of --no-tilt (legacy body-mounted camera) '
                             'used; live joint in physical gimbal mode '
                             'angle is valid')
    parser.add_argument('--tilt', type=float, default=None,
                        help='PHYSICAL GIMBAL: camera world to command '
                             'elevation (degrees, + = up). If not given '
                             'derived from standoff geometry as atan(down/back)')
    parser.add_argument('--no-tilt', dest='tilt_enabled', action='store_false',
                        default=True,
                        help='physical gimbal command/read OFF: legacy '
                             'body-fixed camera chain (--mount) is used')
    parser.add_argument('--tilt-fixed', action='store_true',
                        help='PHASE T OFF: tilt fixed at standoff '
                             'remains (Phase A behavior). Default: tilt '
                             'TRACKS the measured rise of the target')
    parser.add_argument('--gz-model', default=None,
                        help='gazebo wrapper model name (e.g. iris- 1 ); '
                             'if omitted, derive from --topic /drone_N/...')
    parser.add_argument('--aim', type=float, default=None,
                        help='aim offset (degrees); if not given --back / --down from \' '
                             'It is calculated analytically. virtual framing center = -aim')
    parser.add_argument('--back', type=float, default=25.0,
                        help='standoff back distance of guidance (m) - for analytical aim')
    parser.add_argument('--down', type=float, default=4.0,
                        help='standoff vertical offset (m) of the homing. tilt/aim '
                             'input to the derivation. DEFAULT 4 ='
                             'standoff_geom.sh Same as YILDIZ_DESIGN_DOWN '
                             '(old 13 was the value of assembly period +30; '
                             'incorrect default tilt start 27.5 and target '
                             'FOV disina atiyordu)')
    parser.add_argument('--aim-trim', dest='aim_trim', action='store_true',
                        default=True, help='slow motion trim ON (default)')
    parser.add_argument('--no-aim-trim', dest='aim_trim', action='store_false',
                        help='The aim is to keep the analytical value FIXED')
    parser.add_argument('--aim-clamp', type=float, default=6.0,
                        help='trim can deviate from the start by this much (degrees)')
    parser.add_argument('--gimbal-log', default=None, metavar='CSV',
                        help='raw/stabilized error per frame + attitude CSV\'ye')
    parser.add_argument('--attitude-log', default=None, metavar='CSV',
                        help='raw ATTITUDE time series to CSV\' (for time calibration)')
    parser.add_argument('--camera-latency-ms', type=float, default=0.0,
                        help='camera pipeline delay (ms). that\'s the attitude '
                             'GERIYE interpolasyonla tasinir. '
                             'Measured by tools/calibrate_gimbal_timing.py.')
    args = parser.parse_args()

    record_path = None
    env_video = os.environ.get('YILDIZ_VIDEO', '').strip().lower()
    if args.record is not None or env_video in ('1', 'true', 'yes', 'on'):
        record_path = args.record or ''
        if not record_path:
            os.makedirs(os.path.join(root, 'videos'), exist_ok=True)
            # YILDIZ_VIDEO_LABEL: for naming videos watchable in the trial campaign (e.g. "los_ellipse" ->
            # los_ellipse_20260803_...mp4).
            label_value = os.environ.get('YILDIZ_VIDEO_LABEL', '').strip() or 'guidance_value'
            record_path = os.path.join(
                root, 'videos',
                f"{label_value}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")

    display = not args.no_display and bool(os.environ.get('DISPLAY'))
    if not args.no_display and not display:
        print("no DISPLAY; The window will not open.")

    detector = None
    try:
        gimbal = attitude_value = trim = None
        tilt_reader = tilt_commander = tilt_tracking = None
        if args.mavlink_port:
            gz_model = args.gz_model or model_name_from_topic(args.topic)
            if args.tilt_enabled and gz_model is None:
                print("WARNING: --gz-model could not be derived (topic is in the expected pattern "
                      "not); physical gimbal OFF, dropping to old chain.")
            if args.tilt_enabled and gz_model is not None:
                # PHYSICAL GIMBAL MODE (gimbal branch, Phase A): camera in stabilized gimbal. Standoff geometry
                # determines the tilt COMMAND (eps = atan(down/back)); virtual current offset and constant are mount.
                # Aim trim is OFF in this phase (it will be directed to the tilt setpoint in Phase B).
                if args.tilt is not None:
                    eps_cmd = args.tilt
                    print(f"tilt manually set: {eps_cmd:+.2f} deg")
                else:
                    eps_cmd = -analytical_aim(args.back, args.down)
                    print(f"tilt (standoff geometry): back= {args.back:.0f} "
                          f"down={args.down:.0f} -> {eps_cmd:+.2f} deg")
                gimbal = VirtualGimbal(mount_phys_pitch_deg=0.0,
                                     aim_pitch_deg=0.0)
                print(gimbal.summary_value())
                print(f"PHYSICAL GIMBAL: {gz_model} <- tilt {eps_cmd:+.2f} deg "
                      "(aim/ mount disabled, live joint in chain)")
                tilt_reader = TiltStateReader(gz_model).start_value2()
                tilt_commander = TiltCommander(gz_model).start_value2()
                tilt_commander.target_value(eps_cmd)
                if args.tilt_fixed:
                    tilt_tracking = None
                    print("PHASE T OFF (--tilt-fixed): tilt standoff "
                          "fixed in value")
                else:
                    # PHASE C: tilt tracks the measured rise of the target; slowly returns to angle standoff on detection
                    # loss (reacquisition)
                    tilt_tracking = TiltTracking(default_deg=eps_cmd)
                    print(f"PHASE T ON: tilt tracking (default/re-acquisition "
                          f"{eps_cmd:+.2f} deg , backend "
                          f"{'PERMANENT' if tilt_commander.permanent else 'gz CLI'})")
            else:
                # THE OLD WAY (camera fixed to the body): (a) shikki - start the aim from YOUR OWN command geometry.
                # If given --aim it will crush.
                if args.aim is None:
                    aim0 = analytical_aim(args.back, args.down)
                    print(f"aim start (analytics): back = {args.back:.0f} "
                          f"down={args.down:.0f} -> {aim0:+.2f} deg")
                else:
                    aim0 = args.aim
                    print(f"aim manually set: {aim0:+.2f} deg")
                gimbal = VirtualGimbal(mount_phys_pitch_deg=args.mount,
                                     aim_pitch_deg=aim0)
                print(gimbal.summary_value())
                if args.aim_trim:
                    trim = AimTrim(aim0, clamp_deg=args.aim_clamp)
                    print(trim.summary_value())
                else:
                    print("aim trim OFF - aim constant")
            attitude_value = AttitudeReader(args.mavlink_port,
                                 latency_s=args.camera_latency_ms / 1000.0)
            if args.attitude_log:
                attitude_value.log_ac(args.attitude_log)
                print(f"Attitude log: {args.attitude_log}", flush=True)
            attitude_value.start()
            print(f"Camera pipeline delay: {args.camera_latency_ms:.1f} ms "
                  f"(attitude carried this far back)")
        detector = SwarmRedisDetector(args.topic, display, record_path,
                                     args.record_fps, gimbal, attitude_value,
                                     trim, args.gimbal_log,
                                     tilt_reader=tilt_reader,
                                     tilt_commander=tilt_commander,
                                     tilt_tracking=tilt_tracking)

        # --- CLEAN SHUTDOWN --- ERROR (2026-08-01): yildizlar_guidance.sh was sending SIGTERM/SIGKILL to process
        # --stop; Here, only SIGN (KeyboardInterrupt) was being captured, so VideoWriter.release() was not
        # working AT ALL and the moov atom of the mp4 was not being written -> 75 MB file was not being
        # opened. We save AFTER init_node because rospy installs its own SIINT handler.
        def _close(signum, frame):
            print(f"signal {signum} received, closes clean...", flush=True)
            rospy.signal_shutdown('close_value')
        signal.signal(signal.SIGTERM, _close)
        signal.signal(signal.SIGINT, _close)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None:
            detector.close()


if __name__ == '__main__':
    main()
