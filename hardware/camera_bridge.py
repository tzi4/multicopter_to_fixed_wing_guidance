#!/usr/bin/env python3
"""
hardware/camera_bridge.py - camera WITHOUT ROS -> Redis + gimbal bridge
================================================================================= SPACE IN ACTUAL
HARDWARE: bbox_to_redis.py produces the 'tracker_bbox' / 'tracker_bbox_stab' channels in the sim,
but that file is not a file. ROS is a subscriber to Image. NO ROS on Raspberry Pi 5; So right now
there is NO ONE on the hardware to broadcast those two channels. The guidance side
(guidance_allstar/visual_base.py, hardware/single_node_guidance.py) listens ONLY to those channels.
This bridge bridges that gap: camera (or dummy target) -> detection -> virtual gimbal chain -> SAME
two Redis channels -> (optional) real gimbal command.

NO COPIES - ALL IMPORT: bbox_to_redis.AttitudeReader MAVLink ATTITUDE, timestamped buffer + master
interpolation where frame is captured bbox_to_redis.hsv_detection purple/red HSV detection (same code
as sim) bbox_to_redis.build_color_ranges HSV windows (+ STAR_* envs) yildizlar_gimbal.VirtualGimbal
roll de-rotation, internal parameters yildizlar_gimbal.joint_angle world elevation of camera -> body
joint tools.gz_gimbal.TiltTracking EMA+slew+clamp+loss policy tools.mavlink_tilt.MavlinkTiltCommander
REAL gimbal (NOT Gazebo)

This bridge is NOT DECISION-MAKING: It does NOT touch the key 'command_authority'. bbox_to_redis on
the Sim both measures and decides engagement; In hardware, the decision lies with the guidance node
(hardware/single_node_guidance.py, EngagementGate). This place only publishes MEASUREMENT.

MEASUREMENT CONTRACT (EXACTLY with bbox_to_redis - non-modifiable): tracker_bbox [x, y, w, h,
coverage_pct, valid, t_capture] tracker_bbox_stab [sx, p, w, h, ex_deg, ey_deg, t_capture, tilt_eps]
t_capture is based on time.monotonic() (in the sim it was time ROS). Consumers use it to identify individual frames
and compute their age; monotonic is the same system clock in both processes (Linux
CLOCK_MONOTONIC system-wide).

=== MOST CRITICAL TRAP: RESOLUTION === Pixel -> angle conversion is done with the internal
parameters of the VirtualGimbal : fx = (width/ 2 ) / tan(hfov/ 2 ), cx = width/ 2 Default frame is
1280x720 / hfov 66 deg -> fx= 985.5 , cx= 640 . If you feed 1920x1080 to the camera and don't update
the frame, the target RIGHT IN THE CENTER of the frame will appear at px = 960 and chain it to ex =
atan (( 960 - 640 )/ 985.5 ) = + 18.0 shanar deviated to the right. It doesn't give any error
message, my command just keeps spinning. QUIET DISASTER. So: --width-value / --height-value / --hfov
MUST be passed to VirtualGimbal ( bbox_to_redis does not pass them, we do) and if the incoming frame
is of different size, WARNING + rescaling is done (output with --size-multiplier ).

=== THREE STAGE COMMISSIONING (on the table) === 1) without camera, without gimbal : --detector-value mock
--source-value file --no-gimbal 2) with camera, without gimbal : --detector-value hsv --source-value cv2 --no-gimbal
3) with camera, gimbal ON: --detector-value hsv --source-value cv2 --mavlink <address> Detailed description:
hardware/README.md "camera bridge" section.
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import redis                                                    # noqa: E402
from bbox_to_redis import (DEFAULT_TARGET_COLOR, build_color_ranges,  # noqa: E402
                           hsv_detection, AttitudeReader, _opt_int)
from tools.gz_gimbal import TiltTracking                           # noqa: E402
from yildizlar_gimbal import VirtualGimbal, joint_angle           # noqa: E402


# =================================================================== FRAME SOURCES
# ===================================================================

class Cv2Source:
    """cv2.VideoCapture: USB camera (--device N), video file, udp://, rtsp://

    VideoCapture(0) MAY NOT OPEN on the Pi's libcamera stack (known trap, see
    hardware/GIMBAL_TRACKING_TEST.md). In that case, print the image with rpicam-vid to UDP and use
    --source-value file --file-value udp://127.0.0.1:8554, or try --source-value picamera2.
    """

    def __init__(self, target_value, width_value, height_value, loop=False):
        self.target_value = target_value
        self.loop = loop
        self.cap = cv2.VideoCapture(target_value)
        if not self.cap.isOpened():
            raise SystemExit(f"ERROR: failed to open camera source: {target_value!r}")
        if isinstance(target_value, int):
            # Request this resolution from the driver. If it is not accepted, the frame may arrive at a
            # different size again; the main loop captures this and scales it.
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width_value)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height_value)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        print(f"[source] cv2 {target_value!r} opened, driver reports {w}x{h}",
              flush=True)

    def read_value(self):
        ok, frame_value = self.cap.read()
        if not ok:
            if self.loop and not isinstance(self.target_value, int):
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame_value = self.cap.read()
                if ok:
                    return frame_value
            return None
        return frame_value

    def close_value(self):
        self.cap.release()


class Picamera2Source:
    """Raspberry Pi camera (libcamera). UNDERSTANDABLE error if you don't have picamera2."""

    def __init__(self, width_value, height_value):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise SystemExit(
                f"ERROR: --source-value picamera2 requires picamera2 ({exc}).\n"
                "Pi'de:  sudo apt install -y python3-picamera2\n"
                "Alternative: --source-value cv2 --device 0, or with rpicam-vid "
                "Press UDP and --source-value file --file-value udp://127.0.0.1:8554")
        self.picam = Picamera2()
        # 'RGB888' is in the order BGR IN MEMORY in picamera2 (known and confusing naming). This is the
        # correct format because the cv2/HSV chain expects BGR; If we give 'BGR888' the channels are REVERSED
        # and the purple target escapes the HSV window (H shifts around 150 -> ~90).
        cfg = self.picam.create_video_configuration(
            main={"size": (width_value, height_value), "format": "RGB888"})
        self.picam.configure(cfg)
        self.picam.start()
        self.last_metadata = {}
        print(f"[source] picamera2 {width_value} x {height_value} started", flush=True)

    def read_value(self):
        request_value = self.picam.capture_request()
        try:
            frame_value = request_value.make_array("main")
            self.last_metadata = request_value.get_metadata()
        finally:
            request_value.release()
        return frame_value

    def close_value(self):
        try:
            self.picam.stop()
        except Exception:
            pass


class SyntheticSource:
    """CAMERA-FREE frame generator: gray sky + PURPLE rectangle to desired location.

    WHY IT'S THERE: stage-1 desk test (no camera, no gimbal) to run the entire chain -- virtual
    gimbal, Redis broadcast, tilt tracking law, log. It also makes it possible to verify 'fake' and
    'hsv' detectors SIDE BY SIDE: in the same frame the fake box and the box found by HSV should
    overlap.
    """

    def __init__(self, width_value, height_value, bbox=None):
        self.w, self.h = width_value, height_value
        self.bbox = bbox
        self._base = np.zeros((height_value, width_value, 3), np.uint8)
        self._base[:] = (140, 120, 100)          # BGR: soluk mavi-gri gokyuzu

    def read_value(self):
        frame_value = self._base.copy()
        if self.bbox is not None:
            x, y, w, h = [int(v) for v in self.bbox]
            # Gazebo /Purple = RGB ( 1 , 0 , 1 ) -> BGR ( 255 , 0 , 255 ), HSV H= 150 : The middle of
            # bbox_to_redis's PURPLE window (H 140 - 160).
            cv2.rectangle(frame_value, (x, y), (x + w, y + h), (255, 0, 255), -1)
        return frame_value

    def close_value(self):
        pass


# ========================================================== DETECTORS -> (x, y, w, h) or None, in the
# PROCESSING FRAME pixel =============================================================================

class HsvDetector:
    """Calls bbox_to_redis.hsv_detection (same code, same thresholds)."""

    def __init__(self, color_value, min_blob_h=0.0, min_blob_fill=0.0):
        self.color_value, self.windows_value = build_color_ranges(
            color_value, _opt_int('YILDIZ_HSV_SMIN'), _opt_int('YILDIZ_HSV_VMIN'))
        self.min_blob_h = float(min_blob_h)
        self.min_blob_fill = float(min_blob_fill)
        summary_value = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                          for lo, hi in self.windows_value)
        print(f"[detector] HSV, target color {self.color_value.upper()} -> {summary_value}",
              flush=True)

    def find_value(self, frame_value):
        return hsv_detection(frame_value, self.windows_value,
                          self.min_blob_h, self.min_blob_fill)


class MockDetector:
    """Cameraless: 'puts' the subject in the DESIRED location of the frame.

    WHY THERE IS (user's request): "What does the chain say when you put the target at this point of
    the frame / where does MPC command" so that the question can be answered on the table, without
    flying. The box is fixed; the whole sub-chain (stabilization + Redis + gimbal command) works as
    if it were real.

    SIGN CONTRACT (memorize this in the field): box TO THE RIGHT (x > cx) -> ex > 0 box ABOVE (y <
    cy) -> ey < 0 and target rise (-ey) > 0
    """

    def __init__(self, bbox, width_value, height_value):
        if bbox is None:
            # Default: RIGHT CENTER of the frame. Expected result ex~0, ey~0; If you see a non-zero value, the
            # internal parameters are incorrect.
            w, h = 60, 30
            bbox = (width_value // 2 - w // 2, height_value // 2 - h // 2, w, h)
        self.bbox = tuple(int(v) for v in bbox)
        print(f"[detector] FAKE bbox={self.bbox} (camera not used)",
              flush=True)

    def find_value(self, frame_value):
        return self.bbox


class YoloDetector:
    """ultralytics YOLO. The output is scaled back to the PROCESSING FRAMEWORK bbox.

    === CLASSIC ERROR (that's why this comment is long) === The YOLO network is fed letterbox
    (aspect preserving scaling + gray fill) to a FIXED input like 640x640. The boxes given by the
    The network coordinates refer to the 640-pixel canvas. Invert letterboxing to map them to the
    original frame: scale = min(640/W, 640/H), padding_x = (640 - W*scale)/2, and x_frame = (x_640 -
    padding_x) / scale. Skipping this conversion shifts the target toward the upper left and reduces
    reported ex/ey by approximately a factor of 2. Guidance then treats the target as closer and
    more central without an error message.

    ultralytics' predict() does this recycling ITSELF and returns the boxes in pixels of the frame
    you give them; The requirement is to render the frame AS IS (the trap of resizing it yourself
    and then forgetting to scale it back). So we give the exact processing framework, so the output
    is already in that framework. We still GUARD each frame: if the box extends out of frame, we
    SHOUT instead of silently producing the wrong angle.
    """

    def __init__(self, model_path, conf=0.35, class_value=None, imgsz=640):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise SystemExit(
                f"ERROR: --detector-value ultralytics required for yolo ({exc}).\n"
                "  pip install ultralytics\n"
                "Pi AI Camera (IMX500) kullanacaksan --yolo-model <ag>.rpk "
                "ver (runs agi sensor, no ultralytics needed).")
        try:
            self.model = YOLO(model_path)
        except Exception as exc:
            raise SystemExit(
                f"ERROR: Failed to load model YOLO: {model_path}\n ({exc})\n"
                "Provide the absolute weights-file path through --yolo-model. On the Pi "
                "If there is no internet, ultralytics cannot download the model; weight before "
                "elle kopyala.")
        self.conf = float(conf)
        self.class_value = class_value
        self.imgsz = int(imgsz)
        self._overflow_warning = False
        print(f"[detector] YOLO {model_path} conf= {self.conf} "
              f"imgsz= {self.imgsz} class= {self.class_value}", flush=True)

    def find_value(self, frame_value):
        H, W = frame_value.shape[:2]
        r = self.model.predict(frame_value, imgsz=self.imgsz, conf=self.conf,
                               classes=self.class_value, verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return None
        # Not the SAFERest, but the BIGGEST box (same rule as the 'largest contour' in the HSV pathway: the
        # target gets bigger as it gets closer, the parasite stays small).
        xyxy = r.boxes.xyxy.cpu().numpy()
        fields_value = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        x1, y1, x2, y2 = xyxy[int(np.argmax(fields_value))]
        if not self._overflow_warning and (x2 > W + 2 or y2 > H + 2
                                        or x1 < -2 or y1 < -2):
            self._overflow_warning = True
            print(f"WARNING: YOLO kutusu ({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                  f"OUTSIDE the processing frame ({W}x{H}). letterbox back "
                  f"It means the conversion is corrupted -- the produced ex / ey is UNRELIABLE.",
                  flush=True)
        x1 = max(0.0, min(float(x1), W - 1.0))
        y1 = max(0.0, min(float(y1), H - 1.0))
        x2 = max(0.0, min(float(x2), W - 1.0))
        y2 = max(0.0, min(float(y2), H - 1.0))
        if x2 - x1 < 1 or y2 - y1 < 1:
            return None
        return int(x1), int(y1), int(x2 - x1), int(y2 - y1)


class Imx500Detector:
    """Raspberry Pi AI Camera (IMX500): the network runs in the SENSOR, the boxes are in the metadata.

    NOT YET VALIDATE IN HARDWARE (no IMX500 in this repository). Structure and scale recycling were
    deliberately written clearly; It should be verified by comparing the 'dummy' and 'hsv' arms in
    the first run on Pi.

    SCALE RULE: IMX500 API gives the boxes on the INPUT CANVAS of the network (e.g. 640x640); The
    conversion get_outputs(..., add_batch=True) + convert_inference_coords() closes the ISR/scaling
    gap. We also clamp the result to the rendering frame -- same reasoning as YoloDetector.
    """

    def __init__(self, rpk_path, source_value, conf=0.35, class_value=None):
        if not isinstance(source_value, Picamera2Source):
            raise SystemExit(
                "ERROR: IMX500 (.rpk) detector --source-value requires picamera2 "
                "(net runs on sensor, boxes come from frame metadata).")
        try:
            from picamera2.devices import IMX500
            from picamera2.devices.imx500 import postprocess_nanodet_detection  # noqa: F401
        except ImportError as exc:
            raise SystemExit(
                f"ERROR: No IMX500 support ({exc}). On Pi:\n"
                "  sudo apt install -y imx500-all python3-picamera2")
        self.imx = IMX500(rpk_path)
        self.source_value = source_value
        self.conf = float(conf)
        self.class_value = class_value
        print(f"[dedektor] IMX500 {rpk_path} conf= {self.conf}", flush=True)

    def find_value(self, frame_value):
        md = self.source_value.last_metadata or {}
        output = self.imx.get_outputs(md, add_batch=True)
        if output is None:
            return None
        boxes_value, scores_value, classes_value = output[0][0], output[1][0], output[2][0]
        H, W = frame_value.shape[:2]
        best_candidate, max_area = None, 0.0
        for box_value, score_value, snf in zip(boxes_value, scores_value, classes_value):
            if score_value < self.conf:
                continue
            if self.class_value is not None and int(snf) not in self.class_value:
                continue
            # convert_inference_coords: mesh canvas -> SQUARE pixel (scale recycling is done here; DO NOT multiply
            # by hand).
            x, y, w, h = self.imx.convert_inference_coords(box_value, md,
                                                           self.source_value.picam)
            x = max(0, min(int(x), W - 1))
            y = max(0, min(int(y), H - 1))
            w = max(1, min(int(w), W - x))
            h = max(1, min(int(h), H - y))
            if w * h > max_area:
                best_candidate, max_area = (x, y, w, h), w * h
        return best_candidate


# ===================================================================
# COZUNURLUK GATE
# ===================================================================

class SizeGate:
    """Resize incoming frames to the processing dimensions and warn on the first mismatch.

    This class is the only defense of the 'QUIET DISASTER' in the module docstring: it calculates
    how many degrees the target in the middle of the frame will APPEAR to be off and warns, so that
    in the field, the question 'why is the target always shifted to the right' becomes a
    measurement, not a guess.
    """

    def __init__(self, width_value, height_value, fx, multiplier_value=False):
        self.W, self.H, self.fx, self.multiplier_value = width_value, height_value, fx, multiplier_value
        self.warned = False
        self.scaling_n = 0

    def fit(self, frame_value):
        h, w = frame_value.shape[:2]
        if (w, h) == (self.W, self.H):
            return frame_value
        if not self.warned:
            self.warned = True
            deviation = math.degrees(math.atan((w / 2.0 - self.W / 2.0) / self.fx))
            message_value = (f"WARNING: frame {w} x {h} arrived, processing frame "
                     f"{self.W} x {self.H} . If it wasn't scaled, the frame would be FULL. "
                     f"CENTER target ex={deviation:+.1f} deg is deviated "
                     f"gorunurdu (cx={self.W/2:.0f}, fx={self.fx:.1f}). "
                     f"Correct: --width-value {w} --height-value {h} and TRUE "
                     f"Also giving --hfov.")
            if self.multiplier_value:
                raise SystemExit("ERROR (--size-multiplier): " + message_value)
            print(message_value + " For now the frame is being rescaled.", flush=True)
        self.scaling_n += 1
        return cv2.resize(frame_value, (self.W, self.H),
                          interpolation=cv2.INTER_AREA)


# ===================================================================
# BRIDGE
# ===================================================================

class CameraBridge:

    def __init__(self, a):
        self.a = a
        self.W, self.H = int(a.width_value), int(a.height_value)
        self.gimbal_enabled = bool(a.mavlink) and not a.no_gimbal

        # --- VIRTUAL GIMBAL: resolution and hfov MUST go here --- ( bbox_to_redis does not exceed these, the
        # default 1280x720 remained.) mount : 0 IF the physical gimbal is ON -- the angle of the camera
        # relative to the body is not fixed, it is calculated LIVE with joint_angle () in each frame. If the
        # gimbal is closed, the camera is fixed to the body, --mount is valid.
        self.gimbal = VirtualGimbal(
            width=self.W, height=self.H, hfov_rad=math.radians(a.hfov),
            mount_phys_pitch_deg=(0.0 if self.gimbal_enabled else a.mount),
            aim_pitch_deg=0.0)
        # aim=0 INFORMATION: aim is a DC offset and shifts the vertical reference. This bridge broadcasts the
        # raw measurement; The guidance (mpc_guidance ey_ref) decides where the target should stand in the frame.
        print("[gimbal] " + self.gimbal.summary_value(), flush=True)

        self.size_value = SizeGate(self.W, self.H, self.gimbal.fx, a.size_multiplier)

        # --- REDIS ---
        self.r = redis.Redis(host=a.redis_host, port=a.redis_port, db=0)
        try:
            self.r.ping()
        except Exception as exc:
            raise SystemExit(
                f"Redis connection error ({a.redis_host}:{a.redis_port}): {exc}\n"
                "Pi'de:  sudo systemctl start redis-server")
        print(f"[redemption] {a.redis_host}:{a.redis_port} connected; channels "
              f"'tracker_bbox' + 'tracker_bbox_stab'", flush=True)
        # 'command_authority' CANNOT be touched: this bridge is not a decision maker.

        # --- SOURCE ---
        self.source_value = self._source_build()
        # --- DETECTOR ---
        self.detector_value = self._detector_build()

        # --- MAVLINK: attitude + gimbal (SINGLE SHARED CONNECTION) ---
        self.attitude_value = None
        self.commander = None
        self.tracking_value = None
        self._mav = None
        if a.mavlink:
            self._mavlink_build()

        # --- LOG ---
        self._log_f = self._log_w = None
        self._log_n = 0
        if a.log:
            self._log_f = open(a.log, 'w', newline='')
            self._log_w = csv.writer(self._log_f)
            # DIAGNOSTIC COLUMNS (how to read is written in hardware/README.md): raw_* false -> internal parameter
            # / resolution / detection problem raw true, stab false -> attitude, roll sign or time sync
            self._log_w.writerow([
                't', 'tilt_cmd_deg', 'tilt_status_deg',
                'raw_ex_deg', 'raw_ey_deg', 'stab_ex_deg', 'stab_ey_deg',
                'bbox_w', 'bbox_h', 'valid_value', 'fps'])
            print(f"[log] {a.log}", flush=True)

        # --- video kaydi ---
        self.writer_value = None
        self._record_path = a.save
        self._record_buffer = []

        # --- measurement/status ---
        self._frame_t = deque(maxlen=60)
        self._t_initial = None
        self._last_summary = 0.0
        self._tracking_last_t = None
        self.frame_n = 0
        self.detection_n = 0
        self.broadcast_n = 0
        self._stop = False

    # ------------------------------------------------------------ setup

    def _source_build(self):
        a = self.a
        if a.source_value == 'picamera2':
            return Picamera2Source(self.W, self.H)
        if a.source_value == 'cv2':
            return Cv2Source(int(a.device), self.W, self.H, loop=False)
        # 'file': SYNTHETIC frame if no path given (table test without camera)
        if not a.file_value:
            bbox = a.mock_bbox_solve
            if bbox is None:
                w, h = 60, 30
                bbox = (self.W // 2 - w // 2, self.H // 2 - h // 2, w, h)
            print("[source] --file-value not issued -> SYNTHETIC frame produced "
                  f"( {self.W} x {self.H} , purple box {tuple(bbox)} )", flush=True)
            return SyntheticSource(self.W, self.H, bbox)
        return Cv2Source(a.file_value, self.W, self.H, loop=a.loop)

    def _detector_build(self):
        a = self.a
        if a.detector_value == 'mock':
            return MockDetector(a.mock_bbox_solve, self.W, self.H)
        if a.detector_value == 'yolo':
            if str(a.yolo_model).endswith('.rpk'):
                return Imx500Detector(a.yolo_model, self.source_value,
                                      a.yolo_conf, a.yolo_class)
            return YoloDetector(a.yolo_model, a.yolo_conf, a.yolo_class,
                                a.yolo_imgsz)
        return HsvDetector(a.color_value,
                           float(os.environ.get('YILDIZ_MIN_BLOB_H', '0') or 0),
                           float(os.environ.get('YILDIZ_MIN_BLOB_FILL', '0') or 0))

    def _mavlink_build(self):
        """SINGLE connection: attitude reader (recv) + tilt command (send).

        ONE REASON: Opening the 'udpin:...' address twice is BINDING THE PORT TWICE (the second one
        gives an error); Opening the serial port twice produces corrupted frames.
        mavlink_tilt.MavlinkTiltCommander's contract also says this: scripter.start() makes its ONE
        recv synchronous, THEN another thread can be a single recv consumer. The order here is
        deliberately like this: first command.start(), then attitude.start().
        """
        a = self.a
        from pymavlink import mavutil
        print(f"[mavlink] baglaniliyor: {a.mavlink}", flush=True)
        self._mav = mavutil.mavlink_connection(a.mavlink, source_system=250)
        self._mav.wait_heartbeat(timeout=30)
        print(f"[mavlink] heartbeat received (sys={self._mav.target_system})",
              flush=True)

        if self.gimbal_enabled:
            from tools.mavlink_tilt import MavlinkTiltCommander
            self.commander = MavlinkTiltCommander(self._mav).start_value2(
                initial_target_deg=a.tilt)
            if a.tilt_fixed:
                print(f"[gimbal] tracking OFF (--tilt-fixed): tilt "
                      f"{a.tilt:+.1f} deg fixed", flush=True)
            else:
                # TiltTracking: EMA (tau) + slew limit + clamp + loss policy. MEASURED world elevation of the input
                # target (-ey); Since ey is measured relative to the horizon, this is NOT a CLOSED LOOP, but a
                # filtered tracking of the measured quantity (no stability risk).
                self.tracking_value = TiltTracking(default_deg=a.tilt, tau_s=a.tilt_tau,
                                       slew_dps=a.tilt_slew,
                                       alt_deg=a.tilt_alt, upper_deg=a.tilt_upper,
                                       loss_hold_s=3.0, turn_dps=10.0)
                print(f"[gimbal] tracking ON: default {a.tilt:+.1f} deg, "
                      f"clamp [{a.tilt_alt:+.0f}, {a.tilt_upper:+.0f}], "
                      f"slew {a.tilt_slew:.0f} deg/s", flush=True)
        else:
            print("[gimbal] --no-gimbal: NO tilt command, camera to body "
                  f"assumed constant ( --mount {a.mount:+.1f} deg )",
                  flush=True)

        self.attitude_value = AttitudeReader(a.mavlink, latency_s=a.camera_latency_ms / 1000.0,
                                  mav=self._mav)
        self.attitude_value.start()
        print(f"[mavlink] attitude reader started (camera delay "
              f"{a.camera_latency_ms:.0f} ms is moved backward)", flush=True)

    # ------------------------------------------------------------- yardim

    def _tilt_eps(self):
        """WORLD elevation of camera [deg] or None (gimbal off).

        ACTUAL HARDWARE LIMIT: this bridge does not read angle FEEDBACK independent of mount (the
        equivalent of gimbal_tilt_status in the sim would be MOUNT_ORIENTATION; reading it requires
        a SECOND recv consumer on the same connection and pymavlink is not thread-safe). So 'status'
        = last PUBLISHED command. ArduPilot mount driver is safe in slow DC regime because it sets
        the command at <1 s; There is a margin of delay in fast terminal maneuvering. OPEN JOB:
        MOUNT_ORIENTATION reader.
        """
        if self.commander is None:
            return None
        return (self.commander.published_deg
                if self.commander.published_deg is not None
                else self.commander.target_deg)

    def _get_attitude(self, t_frame):
        """(roll_rad, pitch_rad) - if there is no attitude (0, 0) = table assumption."""
        if self.attitude_value is None or not self.attitude_value.ready:
            return 0.0, 0.0
        sample_value = self.attitude_value.get_attitude(t_frame)
        if sample_value is None:
            return 0.0, 0.0
        return sample_value[0], sample_value[1]

    def _fps(self):
        if len(self._frame_t) < 2:
            return 0.0
        return (len(self._frame_t) - 1) / max(
            self._frame_t[-1] - self._frame_t[0], 1e-6)

    # --------------------------------------------------------------- main

    def run_value(self):
        a = self.a
        t_finish = None if a.duration_value <= 0 else time.monotonic() + a.duration_value
        period_value = 0.0 if a.fps <= 0 else 1.0 / a.fps
        last_frame_t = 0.0
        while not self._stop and (t_finish is None or time.monotonic() < t_finish):
            if period_value > 0:
                wait_value = period_value - (time.monotonic() - last_frame_t)
                if wait_value > 0:
                    time.sleep(wait_value)
            last_frame_t = time.monotonic()

            frame_value = self.source_value.read_value()
            if frame_value is None:
                print("[source] no frame (file out / camera cut)",
                      flush=True)
                break
            self._frame_process(frame_value)
        return 0

    def _frame_process(self, frame_value):
        # ONE COMMON CLOCK: frames and attitude examples are also time.monotonic(). t_capture printed on Redis
        # is also this watch (see module docstring).
        t_frame = time.monotonic()
        frame_value = self.size_value.fit(frame_value)
        self.frame_n += 1
        self._frame_t.append(t_frame)
        if self._t_initial is None:
            self._t_initial = t_frame
            print(f"INITIAL FRAME: {frame_value.shape[1]}x{frame_value.shape[0]}", flush=True)

        box_value = self.detector_value.find_value(frame_value)
        valid_value = box_value is not None

        raw_ex = raw_ey = ex = ey = None
        sx = sy = None
        range_value = None
        if valid_value:
            self.detection_n += 1
            x, y, w, h = [int(v) for v in box_value]
            mx, my = x + w / 2.0, y + h / 2.0
            coverage_value = (w / float(self.W)) * 100.0

            # --- CHANNEL 1: RAW bbox (contract verbatim with bbox_to_redis) ---
            self.r.publish('tracker_bbox', json.dumps(
                [x, y, w, h, round(coverage_value, 3), 1, round(t_frame, 4)]))

            roll, pitch = self._get_attitude(t_frame)
            eps = self._tilt_eps()
            # PHYSICAL GIMBAL: LIVE joint angle instead of fixed mount. joint None -> VirtualGimbal falls into the
            # old fixed-mount path.
            joint = (None if eps is None
                     else joint_angle(eps, pitch, roll))
            range_value = self.gimbal.range_prediction(w, self.a.target_width_m)

            sx, sy = self.gimbal.stabilize(mx, my, roll, pitch, range_value,
                                           joint_deg=joint)
            ex, ey = self.gimbal.angle_error_value(mx, my, roll, pitch, range_value,
                                            joint_deg=joint)
            # RAW error: the angle that would be read if there were no de-rotation. In diagnosis, SEPARATION of
            # this column and the stab column divides the fault into two.
            raw_ex = math.degrees(math.atan((mx - self.gimbal.cx) / self.gimbal.fx))
            raw_ey = math.degrees(math.atan((my - self.gimbal.cy) / self.gimbal.fy))

            # --- CHANNEL 2: STABILIZED (8. element = camera elevation used in that frame; mpc_guidance builds ey_ref
            # from it) ---
            self.r.publish('tracker_bbox_stab', json.dumps(
                [round(sx, 2), round(sy, 2), int(w), int(h),
                 round(ex, 4), round(ey, 4), round(t_frame, 4),
                 None if eps is None else round(eps, 3)]))
            self.broadcast_n += 1

        # --- TILT TRACKING (if detected, to the target, if not, to the loss policy) ---
        if self.tracking_value is not None and self.commander is not None:
            now_value = time.monotonic()
            dt = (1.0 / 30 if self._tracking_last_t is None
                  else now_value - self._tracking_last_t)
            self._tracking_last_t = now_value
            # ey is measured relative to the horizon -> earth ascension of the target = -ey.
            self.commander.target_value(self.tracking_value.update_value(
                None if not valid_value else -ey, dt, now_value=now_value))

        self._log(t_frame, raw_ex, raw_ey, ex, ey, box_value, valid_value)
        self._summary(t_frame, ex, ey, box_value, valid_value)
        if self.a.display_value or self._record_path is not None:
            self._draw_value(frame_value, box_value, ex, ey, range_value)

    # ------------------------------------------------------------------------------- output

    def _log(self, t, raw_ex, raw_ey, ex, ey, box_value, valid_value):
        if self._log_w is None:
            return
        cmd = None if self.commander is None else self.commander.target_deg
        st = self._tilt_eps()

        def _s(v, f='{:.4f}'):
            return '' if v is None else f.format(v)

        self._log_w.writerow([
            f"{t:.6f}", _s(cmd, '{:.3f}'), _s(st, '{:.3f}'),
            _s(raw_ex), _s(raw_ey), _s(ex), _s(ey),
            '' if box_value is None else int(box_value[2]),
            '' if box_value is None else int(box_value[3]),
            1 if valid_value else 0, f"{self._fps():.2f}"])
        # PERIODIC FLUSH: do not lose the block closest to the moment of crash in crash/SIGKILL (same lesson
        # in bbox_to_redis , once every 20 rows).
        self._log_n += 1
        if self._log_n % 20 == 0:
            self._log_f.flush()

    def _summary(self, t, ex, ey, box_value, valid_value):
        """~1 ONE line to stderr at s. Not to stdout: In a setup that pipes stdout, so it doesn't get mixed up
with summary data."""
        if t - self._last_summary < 1.0:
            return
        self._last_summary = t
        cmd = None if self.commander is None else self.commander.target_deg
        st = self._tilt_eps()
        box_s = ('-' if box_value is None
                  else f"({box_value[0]},{box_value[1]},{box_value[2]},{box_value[3]})")
        tilt_s = (f"{'-' if cmd is None else f'{cmd:+.1f}'}"
                  f"/{'-' if st is None else f'{st:+.1f}'}")
        ratio_value = (100.0 * self.detection_n / self.frame_n) if self.frame_n else 0.0
        print(f"[BRIDGE] fps= {self._fps():5.1f} frame= {self.frame_n} "
              f"detect=% {ratio_value:.0f} bbox={box_s} "
              f"ex={'-' if ex is None else f'{ex:+6.2f}'} "
              f"ey={'-' if ey is None else f'{ey:+6.2f}'} deg "
              f"tilt(cmd/st)={tilt_s} deg "
              f"{'' if valid_value else '[DETECTION NONE]'}",
              file=sys.stderr, flush=True)

    def _draw_value(self, frame_value, box_value, ex, ey, range_value):
        F, AA = cv2.FONT_HERSHEY_DUPLEX, cv2.LINE_AA
        cv2.drawMarker(frame_value, (int(self.gimbal.cx), int(self.gimbal.cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 22, 1)
        if box_value is not None:
            x, y, w, h = box_value
            cv2.rectangle(frame_value, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.drawMarker(frame_value, (x + w // 2, y + h // 2), (255, 0, 255),
                           cv2.MARKER_CROSS, 12, 1)
        cmd = None if self.commander is None else self.commander.target_deg
        row_value = (f"ex {'-' if ex is None else f'{ex:+.2f}'}  "
                 f"ey {'-' if ey is None else f'{ey:+.2f}'} deg  "
                 f"tilt {'-' if cmd is None else f'{cmd:+.1f}'}  "
                 f"fps {self._fps():.1f}")
        cv2.putText(frame_value, row_value, (8, 24), F, 0.55, (0, 255, 255), 1, AA)
        if range_value is not None:
            cv2.putText(frame_value, f"~{range_value:.0f} m (from bbox, rough)",
                        (8, frame_value.shape[0] - 12), F, 0.45, (200, 200, 200), 1, AA)
        if self._record_path is not None:
            self._save(frame_value)
        if self.a.display_value:
            cv2.imshow('camera_bridge', frame_value)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self._stop = True

    def _save(self, frame_value):
        """The printer installs at the ACTUAL measured speed.

        LESSON (bbox_to_redis 2026-08-01): in the first frame, the installed printer was writing 5
        fps to the header and 30 Hz recordings were playing 6 KAT slow motion. First accumulate 2 s
        frames, MEASURE the speed, then install the printer.
        """
        if self.writer_value is None:
            elapsed_item = time.monotonic() - (self._t_initial or time.monotonic())
            if elapsed_item < 2.0:
                self._record_buffer.append(frame_value.copy())
                return
            fps = max(1.0, min(60.0, self.frame_n / max(elapsed_item, 1e-3)))
            self.writer_value = cv2.VideoWriter(
                self._record_path, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (frame_value.shape[1], frame_value.shape[0]))
            print(f"[record] {self._record_path} ( {frame_value.shape[1]} x"
                  f"{frame_value.shape[0]} @ {fps:.1f} fps, "
                  f"from {len(self._record_buffer)} frame buffer)", flush=True)
            for k in self._record_buffer:
                self.writer_value.write(k)
            self._record_buffer = []
        self.writer_value.write(frame_value)

    # ------------------------------------------------------------- shutdown

    def close_value(self):
        self._stop = True
        if self.commander is not None:
            # Return to default (standoff/straight) pose on exit: the gimbal DOES NOT HANG at the last commanded
            # angle.
            try:
                self.commander.target_value(self.a.tilt)
                time.sleep(0.4)
            except Exception:
                pass
            self.commander.stop_value()
        if self._log_f is not None:
            self._log_f.close()
            self._log_f = None
        # SHORT RUN TRAP: printer only sets up AFTER 2 s speed measurement; a shorter run (e.g. --duration-value 1.5)
        # otherwise it wouldn't produce any files and would be considered "recording not working". If there is
        # a frame left in the buffer, it is emptied at the speed measured here.
        if self.writer_value is None and self._record_buffer:
            elapsed_item = max(time.monotonic() - (self._t_initial or 0.0), 1e-3)
            fps = max(1.0, min(60.0, self.frame_n / elapsed_item))
            k0 = self._record_buffer[0]
            self.writer_value = cv2.VideoWriter(
                self._record_path, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                (k0.shape[1], k0.shape[0]))
            print(f"[record] short run: {len(self._record_buffer)} frame "
                  f"{fps:.1f} is written from the buffer with fps", flush=True)
            for k in self._record_buffer:
                self.writer_value.write(k)
            self._record_buffer = []
        if self.writer_value is not None:
            self.writer_value.release()
            print(f"[record] saved: {self._record_path}", flush=True)
        try:
            self.source_value.close_value()
        except Exception:
            pass
        cv2.destroyAllWindows()
        ratio_value = (100.0 * self.detection_n / self.frame_n) if self.frame_n else 0.0
        print(f"[SUMMARY] frame= {self.frame_n} detected= {self.detection_n} (% {ratio_value:.1f} ) "
              f"stab_broadcast={self.broadcast_n} "
              f"olceklenen_frame={self.size_value.scaling_n}", flush=True)


# ===================================================================
# CLI
# ===================================================================

def _mock_bbox_solve(text_value):
    if not text_value:
        return None
    try:
        p = [int(float(v)) for v in text_value.replace(' ', '').split(',')]
    except ValueError:
        raise SystemExit(f"ERROR: --mock-bbox '{text_value}' could not be resolved; "
                         "format: x,y,w,h (e.g. 640 , 360 , 60 , 30 )")
    if len(p) != 4:
        raise SystemExit(f"ERROR: --mock-bbox 4 requires number (x,y,w,h), "
                         f"{len(p)} arrived")
    return tuple(p)


def arg_parse(argv=None):
    """CLI parsing SEPARATE: so that the offline verification harness (installing CameraBridge without
camera/MAVLink and running _frame_render) can use the same defaults. Setting up Namespace manually
would create a second copy of the defaults."""
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--source-value', default='cv2',
                   choices=['picamera2', 'cv2', 'file_value'],
                   help='frame source (file + --file-value or SYNTHETIC frame)')
    p.add_argument('--device', type=int, default=0,
                   help='Camera index for --source-value cv2')
    p.add_argument('--file-value', default=None,
                   help='video file / udp:// / rtsp:// (--source-value file)')
    p.add_argument('--loop', action='store_true',
                   help='When the video file is finished, start from the beginning')
    p.add_argument('--width-value', type=int, default=1280)
    p.add_argument('--height-value', type=int, default=720)
    p.add_argument('--hfov', type=float, default=66.0,
                   help='horizontal viewing angle [degrees]. IMX500 1280x720 -> 66')
    p.add_argument('--size-multiplier', action='store_true',
                   help='If the incoming frame is different from the frame, scale, EXIT')
    p.add_argument('--detector-value', default='hsv', choices=['hsv', 'yolo', 'mock'])
    p.add_argument('--mock-bbox', default=None, metavar='x,y,w,h',
                   help='Box for FAKE detector (e.g. 960,360,60,30)')
    p.add_argument('--color-value', default=os.environ.get('YILDIZ_TARGET_COLOR',
                                                    DEFAULT_TARGET_COLOR),
                   help='HSV target color: purple | rejection')
    p.add_argument('--yolo-model', default='yolov8n.pt',
                   help='.pt (ultralytics) or .rpk (runs on IMX500, sensor)')
    p.add_argument('--yolo-conf', type=float, default=0.35)
    p.add_argument('--yolo-imgsz', type=int, default=640)
    p.add_argument('--yolo-class', type=int, nargs='*', default=None,
                   help='only this class subscripts (empty = all)')
    p.add_argument('--mavlink', default=None, metavar='CONNECTION',
                   help="pymavlink adresi: /dev/ttyACM0 | udpin:127.0.0.1:14601 "
                        "| tsp:127.0.0.1:5760. NO attitude and gimbal if not given "
                        "(bench-test assumption: roll=pitch=0).")
    p.add_argument('--no-gimbal', action='store_true',
                   help='sending tilt command (read attitude only)')
    p.add_argument('--mount', type=float, default=0.0,
                   help='camera\'s mounting angle to the body WHEN the gimbal is OFF '
                        '[degree, + up]')
    p.add_argument('--tilt', type=float, default=0.0,
                   help='gimbal default/re-acquisition elevation [degrees]')
    p.add_argument('--tilt-fixed', action='store_true',
                   help='tilt tracking OFF: remain fixed at --tilt')
    p.add_argument('--tilt-alt', type=float, default=-35.0)
    p.add_argument('--tilt-upper', type=float, default=55.0)
    p.add_argument('--tilt-tau', type=float, default=0.4)
    p.add_argument('--tilt-slew', type=float, default=45.0)
    p.add_argument('--camera-latency-ms', type=float, default=0.0,
                   help='camera pipeline delay; interpolate attitude back to '
                        'capture time (tools/calibrate_gimbal_timing.py)')
    p.add_argument('--target-width-m', type=float, default=1.6,
                   help='apparent target width [m], used for a rough range estimate '
                        'from bbox width; guidance obtains its range '
                        'from telemetry instead')
    p.add_argument('--display-value', action='store_true',
                   help='OpenCV window (leave CLOSED on headless Pi\')')
    p.add_argument('--save', nargs='?', const='', default=None,
                   metavar='FILE', help='save the crossed out frames as mp4')
    p.add_argument('--log', default=None, metavar='CSV',
                   help='diagnosis per frame CSV \'')
    p.add_argument('--fps', type=float, default=0.0,
                   help='cycle rate limit [Hz]; 0 = whatever the source gives')
    p.add_argument('--duration-value', type=float, default=0.0,
                   help='second; 0 = up to Ctrl-C\'')
    p.add_argument('--redis-host', default='127.0.0.1')
    p.add_argument('--redis-port', type=int, default=6379)
    a = p.parse_args(argv)

    a.mock_bbox_solve = _mock_bbox_solve(a.mock_bbox)
    if a.save == '':
        root_value = Path(__file__).resolve().parent
        (root_value / 'videos').mkdir(exist_ok=True)
        a.save = str(root_value / 'videos' /
                       f"bridge_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    if a.display_value and not os.environ.get('DISPLAY'):
        print("WARNING: No DISPLAY, ignoring --display-value.", flush=True)
        a.display_value = False
    # The synthetic source rotates at unlimited speed; Don't burn the CPU in vain.
    if a.fps <= 0 and a.source_value == 'file_value' and not a.file_value:
        a.fps = 30.0
    return a


def main(argv=None):
    a = arg_parse(argv)
    bridge_value = CameraBridge(a)

    def _close(signum, frame):
        print(f"\nsignal {signum} received, closes cleanly...", flush=True)
        bridge_value._stop = True
    signal.signal(signal.SIGINT, _close)
    signal.signal(signal.SIGTERM, _close)

    try:
        return bridge_value.run_value()
    except KeyboardInterrupt:
        return 0
    finally:
        bridge_value.close_value()


if __name__ == '__main__':
    raise SystemExit(main())
