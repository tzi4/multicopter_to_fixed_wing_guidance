#!/usr/bin/env python3
"""TABLE DEMO: CONTINUOUS TARGET TRACKING (PHASE DRAW) with real gimbal + camera.

Move a purple object up/down with your hand -> the servo rotates the camera towards the object.
Exactly reduced version of the chain in the sim: square -> purple HSV detection -> ey [deg] ->
target_elev = tilt_current - ey -> TiltTracking (strainer+slew+clamp) -> MavlinkTiltCommander (ArduPilot)

ASSUMPTIONS (desk condition): the drone stands steady and level (roll/pitch ~ 0), i.e. body frame =
earth frame and joint angle = tilt. The full chain in flight (live attitude + joint_angle()) is in
bbox_to_redis; This tool is deliberately simple -- its purpose is hardware verification.

USAGE:
  python3 tools/gimbal_bench_tracking.py --source-value 0 --connection-value udpin:0.0.0.0:14550
    --source-value : source cv2.VideoCapture (0 = first camera, file path, 'udp://...', 'rtsp://...', or 0
    if you're running on a Pi) --connection-value: address pymavlink (YOU CANNOT share the same port when MP
    is on; Turn off MP or open MAVLink Mirror from MP and connect to it) --dry : MAVLink NONE,
    view+law loop only (development testing) --display-value : OpenCV window (if DISPLAY is present)

Exit: Ctrl-C. Security: 3 holds s if detection is lost, then rotates slowly to 0 degrees; clamp
[-40, +55] (safe margin within your measured -44..+45 command band).
"""

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.gz_gimbal import TiltTracking                       # noqa: E402

# In-camera parameters -- IMX500 1280x720 (same as yildizlar_gimbal). If there is a different
# resolution, fy will be scaled proportionally.
HFOV_RAD = 1.1519
PURPLE_LO = (135, 90, 50)       # Broadband working in real IMX500 test
PURPLE_HI = (165, 255, 255)


class PicamSource:
    """Picamera2/IMX500 source running in hardware.

    Since the camera is mounted 180 degrees upside down on the body, the default rotation corrects
    both the image and the LOS marks together."""

    def __init__(self, width_value=1280, height_value=720, rotate_180=True):
        from picamera2 import Picamera2
        from libcamera import Transform
        self.cam = Picamera2()
        tr = Transform(hflip=1, vflip=1) if rotate_180 else Transform()
        cfg = self.cam.create_video_configuration(
            main={'size': (width_value, height_value), 'format': 'RGB888'},
            transform=tr)
        self.cam.configure(cfg)
        self.cam.start()

    def isOpened(self):
        return True

    def read(self):
        return True, self.cam.capture_array()

    def release(self):
        self.cam.stop()
        self.cam.close()


def purple_find(frame_value):
    hsv = cv2.cvtColor(frame_value, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, PURPLE_LO, PURPLE_HI)
    pull = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, pull)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, pull)
    position_value, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not position_value:
        return None
    en = max(position_value, key=cv2.contourArea)
    if cv2.contourArea(en) < 40:
        return None
    x, y, w, h = cv2.boundingRect(en)
    M = cv2.moments(en)
    if M['m00'] > 0:
        return M['m10'] / M['m00'], M['m01'] / M['m00'], w, h
    return x + w / 2.0, y + h / 2.0, w, h


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--source-value', default='0')
    p.add_argument('--connection-value', default='udpin:0.0.0.0:14550')
    p.add_argument('--dry', action='store_true',
                   help='MAVLink opinion+law only (test) without shipping')
    p.add_argument('--display-value', action='store_true')
    p.add_argument('--straight-value', action='store_true',
                   help='rotating the camera 180 (default reverse mounting)')
    p.add_argument('--tilt-alt', type=float, default=-40.0)
    p.add_argument('--tilt-upper', type=float, default=55.0)
    p.add_argument('--duration-value', type=float, default=0.0,
                   help='second; 0 = Ctrl-C''until')
    a = p.parse_args()

    file_mi = not a.source_value.isdigit() and a.source_value != 'picam'
    cap = None
    if a.source_value != 'picam':
        source_value = int(a.source_value) if a.source_value.isdigit() else a.source_value
        cap = cv2.VideoCapture(source_value)
        ok = cap.isOpened()
        if ok and not file_mi:
            for _ in range(10):
                ok, _ = cap.read()
                if ok:
                    break
                time.sleep(0.1)
        if not ok:
            cap.release()
            cap = None
            if file_mi:
                raise SystemExit(f"ERROR: could not open resource: {a.source_value}")
            print("No frame from V4L2 -> Switching to Picamera2",
                  flush=True)
    if cap is None:
        cap = PicamSource(rotate_180=not a.straight_value)
        print(f"camera: Picamera2 (IMX500) 1280x720"
              f"{'' if a.straight_value else ' (180 cevrildi)'}", flush=True)

    commander = None
    if not a.dry:
        from tools.mavlink_tilt import MavlinkTiltCommander
        commander = MavlinkTiltCommander(
            a.connection_value, dead_band_deg=0.1, min_interval_s=0.02,
            servo_slew_dps=120.0, tick_s=0.01).start_value2(initial_target_deg=0.0)
        print(f"MAVLink connected: {a.connection_value}")

    tracking_value = TiltTracking(default_deg=0.0, tau_s=0.25, slew_dps=60.0,
                      alt_deg=a.tilt_alt, upper_deg=a.tilt_upper,
                      loss_hold_s=3.0, turn_dps=10.0)
    print(f"follow on: clamp [{a.tilt_alt:+.0f}, {a.tilt_upper:+.0f}] deg. "
          "Move the purple object vertically; Exit with Ctrl-C.")

    t_previous_value = time.monotonic()
    last_report = 0.0
    t_finish = None if a.duration_value <= 0 else time.monotonic() + a.duration_value
    trace_samples = []                              # (t, detection_var, target_elev, cmd)
    try:
        while t_finish is None or time.monotonic() < t_finish:
            ok, frame_value = cap.read()
            if not ok:
                if file_mi:                     # file finished
                    break
                time.sleep(0.05)
                continue
            h_img = frame_value.shape[0]
            fy = (frame_value.shape[1] / 2.0) / math.tan(HFOV_RAD / 2.0)
            now_value = time.monotonic()
            dt = now_value - t_previous_value
            t_previous_value = now_value

            detection = purple_find(frame_value)
            if detection is not None:
                mx, my, w, h = detection
                # table assumption: ey = offset from camera; target's world rise = current tilt - ey (body straight ->
                # joint = tilt)
                ey = math.degrees(math.atan((my - h_img / 2.0) / fy))
                if abs(ey) < 0.4:
                    ey = 0.0
                target_elev = tracking_value.cmd - ey
            else:
                target_elev = None
            cmd = tracking_value.update_value(target_elev, dt, now_value=now_value)
            if commander is not None:
                commander.target_value(cmd)
            trace_samples.append((now_value, detection is not None,
                       float('nan') if target_elev is None else target_elev, cmd))

            if now_value - last_report > 1.0:
                last_report = now_value
                state_value = ('NONE' if detection is None
                         else f"elev {target_elev:+6.2f}")
                print(f"  detect {state_value} -> tilt cmd {cmd:+6.2f} deg",
                      flush=True)
            if a.display_value:
                if detection is not None:
                    cv2.drawMarker(frame_value, (int(mx), int(my)), (0, 255, 255),
                                   cv2.MARKER_CROSS, 24, 2)
                cv2.putText(frame_value, f"tilt {cmd:+.1f}", (8, 28),
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 1)
                cv2.imshow('bench tracking', frame_value)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if commander is not None:
            commander.target_value(0.0)
            time.sleep(0.5)
            commander.stop_value()
        cv2.destroyAllWindows()
    return trace_samples


if __name__ == '__main__':
    trace_samples = main()
    n_detection = sum(1 for _, t, _, _ in trace_samples if t)
    print(f"\n summary: {len(trace_samples)} frame, {n_detection} detect; "
          f"tilt son {trace_samples[-1][3]:+.2f} deg" if trace_samples else "square not processed")
