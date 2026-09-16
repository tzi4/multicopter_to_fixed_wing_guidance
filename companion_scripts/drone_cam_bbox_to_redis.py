#!/usr/bin/env python3
"""
Drone Camera → Redis Detector + Embedded Decision Maker ========================================== -
ROS takes images from the camera topic - Detects a red target (HSV color filter) - Publishes the
BBox data to the Redis 'tracker_bbox' channel - decider.py logic is included in itself. runs (does
not require separate script) - shows ROLE/TASK/TARGET status on the OpenCV screen
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import time
from collections import deque


class SimRedisDetector:
    # ─── DECISION MAKER PARAMETERS (from decider.py) ───
    WINDOW_SIZE = 25                    # How many last frames to look at?
    TRANSITION_THRESHOLD = 20           # valid frame threshold for positioned→display
    REVERT_THRESHOLD = 3                # low threshold for displayed→located
    REVERT_DWELL_SECONDS = 2            # Return waiting time (sec)
    MIN_COVERAGE_TRANSITION = 0.85      # Minimum coverage % for migration
    MIN_COVERAGE_HOLD = 0.7             # Minimum coverage % for video stay

    def __init__(self):
        # --- REJECTION LINK ---
        print("Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.set('mission_value', 'absent')
            self.r.set('command_authority', 'position_based')
            print("Redis connection successful! Broadcast channel: 'tracker_bbox'")
        except Exception as e:
            print(f"Redis Connection Error: {e}")
            exit(1)

        # --- ROS CONNECTION ---
        rospy.init_node('sim_redis_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/drone_1/webcam/image_raw", Image, self.image_callback)
        print("ROS Node started, waiting for image from channel /drone_1/webcam/image_raw...")

        # --- COLOR SETTINGS (Red) ---
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])

        # --- DECISION MAKER INTERNAL SITUATION ---
        self.current_mode = 'position_based'               # Startup mode
        self.decision_window = deque(maxlen=self.WINDOW_SIZE)
        self.revert_pending_since = None             # Dwell timer
        self.last_frame_time = time.time()

        # --- VIDEO RECORDING ---
        self.video_writer = None
        self.video_filename = f"record_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        print(f"Video recording active: {self.video_filename}")

    # ═════════════════════════ ══════════════════════════ DECIDER MAKER — EMBEDDED (decider.py logic)
    # ═════════════════════════ ══════════════════════════

    def _evaluate_frame(self, valid_detection, coverage):
        """
        Evaluate whether a single frame meets the transition criteria. Since color detection is
        used, valid_detection is used instead of confidence. Coverage threshold varies BY MODE
        (hysteresis).
        """
        if not valid_detection:
            return False, "target_absent"

        if self.current_mode == 'position_based':
            # While on location: target must be NEAR to switch to video
            if coverage < self.MIN_COVERAGE_TRANSITION:
                return False, "far_target"
        else:
            # While on display: accept as long as the target is visible
            if coverage < self.MIN_COVERAGE_HOLD:
                return False, "cok_far"

        return True, "valid_value"

    def _make_decision(self, valid_count):
        """
        Generate mode decision based on window status. Dwell time logic: delays display→position
        rotation.
        """
        now = time.time()

        # Manual stop control
        try:
            val = self.r.get('manual_stop')
            manual = val and val.decode('utf-8') == '1'
        except Exception:
            manual = False

        if manual:
            self.revert_pending_since = None
            if self.current_mode != 'position_based':
                return 'position_based', True
            return 'position_based', False

        # Change your decision if the window is not filled enough
        if len(self.decision_window) < self.WINDOW_SIZE:
            return self.current_mode, False

        if self.current_mode == 'position_based':
            # Positioned → displayed: high threshold
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                return 'visual', True
            return 'position_based', False

        elif self.current_mode == 'visual':
            # Displayed → positioned: low threshold + dwell time
            if valid_count <= self.REVERT_THRESHOLD:
                if self.revert_pending_since is None:
                    self.revert_pending_since = now
                    print(f"[DECISION] Subthreshold ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell started ({self.REVERT_DWELL_SECONDS}s)", flush=True)

                dwell_elapsed = now - self.revert_pending_since
                if dwell_elapsed >= self.REVERT_DWELL_SECONDS:
                    self.revert_pending_since = None
                    return 'position_based', True

                return 'visual', False
            else:
                # Threshold exceeded — dwell canceled
                if self.revert_pending_since is not None:
                    elapsed = now - self.revert_pending_since
                    print(f"[DECISION] Target appeared back ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell iptal ({elapsed:.2f}s)", flush=True)
                    self.revert_pending_since = None
                return 'visual', False

        return self.current_mode, False

    def _update_decision(self, valid_detection, coverage):
        """Called every frame: update window, produce decision, write to Redis."""
        frame_valid, reason = self._evaluate_frame(valid_detection, coverage)

        self.decision_window.append(frame_valid)
        valid_count = sum(1 for v in self.decision_window if v)

        new_mode, changed_value = self._make_decision(valid_count)

        if changed_value:
            previous = self.current_mode
            self.current_mode = new_mode
            self.decision_window.clear()
            self.revert_pending_since = None
            try:
                self.r.set('command_authority', self.current_mode)
            except Exception:
                pass
            print(f"[DECISION] >>> MODE CHANGED: {previous} → {new_mode} "
                  f"(window: {valid_count} / {self.WINDOW_SIZE} )", flush=True)

        return self.current_mode

    # ═════════════════════════ ══════════════════════════ CAMERA CALLBACK ═════════════════════════
    # ══════════════════════════

    def image_callback(self, data):
        t_start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- Target Hit Area (Yellow Box) ---
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        cv2.rectangle(cv_image, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 1)
        cv2.putText(cv_image, "Strike Zone", (av_left + 4, av_top + 14),
                    cv2.FONT_HERSHEY_DUPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # --- HSV color detection ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_detection = False
        horizontal_coverage = 0.0

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            target_area = cv2.contourArea(c)

            if target_area > 18:
                x, y, w, h = cv2.boundingRect(c)
                valid_detection = True

                horizontal_coverage = (w / w_img) * 100
                validity_flag = 1

                t_detect = time.time()
                detect_ms = (t_detect - t_start) * 1000
                self.r.set('timing_bbox_to_redis', f"{detect_ms:.2f}")

                bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag, t_start]
                self.r.publish('tracker_bbox', str(bbox))

                center_x = int(x + w/2)
                center_y = int(y + h/2)
                print(f"Target Center: ({center_x}, {center_y}) | Coverage: {horizontal_coverage:.1f}%")

                # Mark the target with a red box
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

        # --- UPDATE DECISION MAKER ---
        command_authority = self._update_decision(valid_detection, horizontal_coverage)

        # --- EKRAN HUD ---
        FONT = cv2.FONT_HERSHEY_DUPLEX
        AA = cv2.LINE_AA

        if command_authority == 'visual':
            rol_text = "LIDER"
            mission_text = "VISUAL_GUIDANCE"
            hud_color = (0, 255, 0)      # Green
        else:
            rol_text = "UYE"
            mission_text = "POSITION_BASED"
            hud_color = (0, 165, 255)    # Turuncu

        # Top band background (translucent village stripe)
        overlay = cv_image.copy()
        cv2.rectangle(overlay, (0, 0), (w_img, 50), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, cv_image, 0.45, 0, cv_image)

        # Top left: Role | Task (on one line)
        cv2.putText(cv_image, f"{rol_text}  |  {mission_text}", (8, 20),
                    FONT, 0.5, hud_color, 1, AA)

        # target status
        target_text = "TARGET: FOUND" if valid_detection else "TARGET: NONE"
        target_color_value = (0, 255, 0) if valid_detection else (100, 100, 255)
        cv2.putText(cv_image, target_text, (8, 40),
                    FONT, 0.45, target_color_value, 1, AA)

        # Top right: Window status
        valid_count = sum(1 for v in self.decision_window if v)
        fill = len(self.decision_window)
        window_text_value = f"Window: {valid_count} / {fill}"
        # Calculate text width and age right
        (tw, _), _ = cv2.getTextSize(window_text_value, FONT, 0.4, 1)
        cv2.putText(cv_image, window_text_value, (w_img - tw - 10, 20),
                    FONT, 0.4, (180, 180, 180), 1, AA)

        # Upper right bottom: Coverage (if there is a target)
        if valid_detection:
            cov_text = f"Cov: {horizontal_coverage:.1f}%"
            (tw2, _), _ = cv2.getTextSize(cov_text, FONT, 0.4, 1)
            cv2.putText(cv_image, cov_text, (w_img - tw2 - 10, 40),
                        FONT, 0.4, (180, 220, 180), 1, AA)

        # --- VIDEO RECORDING ---
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.video_filename, fourcc, 20.0, (w_img, h_img))
            print(f"VideoWriter launched: {w_img}x{h_img} @ 20fps")
        self.video_writer.write(cv_image)

        cv2.imshow("Simulasyon Redis Dedektoru", cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    detector = None
    try:
        detector = SimRedisDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if detector and detector.video_writer:
            detector.video_writer.release()
            print(f"Video kaydedildi: {detector.video_filename}")
        cv2.destroyAllWindows()
