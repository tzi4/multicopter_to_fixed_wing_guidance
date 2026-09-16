#!/usr/bin/env python3
"""
DECISION MAKER (TransitionDecider) ================================

Purpose: autonomously decide the transition between position-based and visual guidance.

Logic: - Evaluates detection data from YOLO in each frame. - Keeps the transmission of the last N
frames in memory (deque). - Transition criteria: confidence + coverage + persistence - Uses
hysteresis: high threshold transition, low threshold return. - DWELL TIME: Extra waiting time for
view→position return. If the target disappeared for a short period of time, such as half a second,
but returned immediately, the decision to return is delayed by the dwell time to prevent
oscillation. - If manual stop is active, it automatically returns to position mode.

Output: Writes 'positioned' or 'displayed' to key 'command_authority' to Redis. Positional guidance and
visual guidance read this key and adjust their own behavior.
"""

import time
import redis
import ast
import threading
import csv
from datetime import datetime
from collections import deque


class TransitionDecider:
    # ─── THRESHOLD PARAMETERS ───

    # Window size — how many last frames to look at YOLO 30 FPS assuming N=20 ≈ 0.66 seconds
    WINDOW_SIZE = 25

    # Transition threshold (high): positioned → for image AT LEAST how many of the last WINDOW_SIZE frames
    # must all criteria be met
    TRANSITION_THRESHOLD = 20  # %75

    # Return threshold (low): for displayed → positioned (hysteresis) Return if the criterion is met in
    # MAXIMUM of the last WINDOW_SIZE frames
    REVERT_THRESHOLD = 3  # %25

    # ─── DWELL TIME (Return Waiting Time) ─── After the decision to return from visual to positional is
    # made, if the target does not appear back during this time, the decision is made. If the target
    # appears back, the decision is cancelled. Prevents oscillation.
    REVERT_DWELL_SECONDS = 2 # 2 seconds

    # NOTE: There is no dwell for positioned → video switching, as a high threshold (%75%) needs to be
    # maintained for the duration of WINDOW_SIZE anyway, this is debounce in itself.

    # Criteria for a frame to be considered "valid"
    MIN_CONFIDENCE = 0.65
    # Removed the old single threshold — replaced by two thresholds that vary by mode: In position mode:
    # target must be NEAR (to switch to image) In image mode: ok as long as the target is visible (to
    # remain in image)
    MIN_COVERAGE_TRANSITION = 1  # %10 — only key distance toggles
    MIN_COVERAGE_HOLD = 0.75          # %3 — hold for a while even if the video zooms out

    # ─── REDIS ANAHTARLARI ───
    KEY_COMMAND_AUTHORITY = 'command_authority'    # Output: 'located' / 'imaged'
    KEY_MANUAL_STOP = 'manual_stop'    # Input: '1' active, other value inactive
    CHANNEL_TRACKER = 'tracker_bbox'       # Release of Detection.py

    # ─── LOG SETTINGS ───
    ENABLE_CSV_LOG = True

    def __init__(self, redis_host='localhost', redis_port=6379):
        # Redis connection
        print("[DECISION] Connecting to server Redis...", flush=True)
        try:
            self.r = redis.Redis(host=redis_host, port=redis_port, db=0)
            self.r.ping()
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe(self.CHANNEL_TRACKER)
            print(f"[DECISION] Redis Subscribed to channel '{self.CHANNEL_TRACKER}'.", flush=True)

            # ─── PUB/FEB BUFFER FLUSH ─── Discard old messages accumulated in the channel while Decider starts
            # (so that old detections do not instantly fill the window when the system is restarted)
            print("[DECISION] Clearing pub/feb buffer (discard old messages)...", flush=True)
            flush_start = time.time()
            flushed_count = 0
            while time.time() - flush_start < 0.5:  # 0.5 clear for seconds
                msg = self.pubsub.get_message(ignore_subscribe_messages=True, timeout=0.01)
                if msg is None:
                    break
                if msg.get('type') == 'message':
                    flushed_count += 1
            print(f"[DECISION] {flushed_count} old message removed, the beginning is clear.", flush=True)
        except Exception as e:
            print(f"[DECISION] Redis Connection Error: {e}", flush=True)
            raise

        # Initial state: positioned (the system initially operates with positional guidance)
        self.current_mode = 'position_based'
        self._publish_mode()  # Write to Redis at startup

        # Window: holds each element (frame_valid: bool, conf: float, cov: float, reason: str)
        self.window = deque(maxlen=self.WINDOW_SIZE)

        # Timer to fill the window even when there is no detection
        self.last_frame_time = time.time()

        # ─── DWELL TIME STATE ─── Pending state for return None = no pending, normal operation float =
        # pending start time (time.time())
        self.revert_pending_since = None

        # Loglama
        self.csv_writer = None
        self.log_file = None
        if self.ENABLE_CSV_LOG:
            self._init_logger()

        # Lock for thread security
        self.lock = threading.Lock()

        print(f"[DECISION] Initiated | Mode: {self.current_mode}", flush=True)
        print(f"[KARAR] Parametreler: N={self.WINDOW_SIZE}, "
            f"Transition={self.TRANSITION_THRESHOLD}, Return={self.REVERT_THRESHOLD}, "
            f"Dwell={self.REVERT_DWELL_SECONDS}s, "
            f"MinConf={self.MIN_CONFIDENCE}, "
            f"Chow(Pass≥{self.MIN_COVERAGE_TRANSITION}% / Hold≥{self.MIN_COVERAGE_HOLD}%)", flush=True)

        # ─── STATUS PRINT STATE ───
        self.last_status_time = 0.0           # When was the last time we printed
        self.HEARTBEAT_INTERVAL = 1.0         # Even if there is no change, 1 starts per second.
        self.last_status_snapshot = None      # Status at last print (for change detection)

    def _init_logger(self):
        """CSV start log file"""
        log_filename = f"karar_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        headers = [
            "timestamp", "current_mode",
            "frame_valid", "confidence", "coverage",
            "reason",
            "valid_count_in_window",
            "window_fill",
            "revert_pending",
            "dwell_elapsed",
            "decision_change"
        ]
        self.csv_writer.writerow(headers)
        print(f"[DECISION] Log file: {log_filename}", flush=True)

    def _publish_mode(self):
        """Write current mode to Redis"""
        try:
            self.r.set(self.KEY_COMMAND_AUTHORITY, self.current_mode)
        except Exception as e:
            print(f"[DECISION] Redis publish error: {e}", flush=True)

    def _check_manual_stop(self):
        """Is manual stop active?"""
        try:
            val = self.r.get(self.KEY_MANUAL_STOP)
            return val and val.decode('utf-8') == '1'
        except Exception:
            return False


    def _evaluate_frame(self, bbox_data):
        """
        Evaluate whether a single frame meets the decision criteria. bbox_data: [x, y, w, h,
        h_coverage, validity_flag, confidence]

        Coverage threshold varies DEPENDING on MODE (hysteresis): - In positioned mode: requests a
        close target (MIN_COVERAGE_TRANSITION) → Switches to image only at key distance, distant
        targets do not trigger - In image mode: consider valid as long as the target is still
        visible (MIN_COVERAGE_HOLD) → Stays on image during temporary distances, does not oscillate

        Returns: (frame_valid: bool, confidence: float, coverage: float, reason: str)
        """
        if bbox_data is None or len(bbox_data) < 7:
            return False, 0.0, 0.0, "data_missing"

        try:
            x, y, w, h = bbox_data[0:4]
            coverage = float(bbox_data[4])
            validity = int(bbox_data[5])
            confidence = float(bbox_data[6])
        except (ValueError, IndexError) as e:
            return False, 0.0, 0.0, f"parse_error:{e}"

        # Validity: detection.py old data, consider invalid if in grace period
        if validity == 0:
            return False, confidence, coverage, "grace_period"

        if confidence < self.MIN_CONFIDENCE:
            return False, confidence, coverage, "low_confidence"

        # ─── COVERAGE THRESHOLD Varies BY MODE (HYSTERESIS) ───
        if self.current_mode == 'position_based':
            # While on location: target must be NEAR to switch to video
            if coverage < self.MIN_COVERAGE_TRANSITION:
                return False, confidence, coverage, "far_target"
        else:
            # While on display: accept as long as the target is visible
            if coverage < self.MIN_COVERAGE_HOLD:
                return False, confidence, coverage, "cok_far"

        return True, confidence, coverage, "valid_value"


    def _make_decision(self, valid_count, manual_stop):
        """
        Make decisions based on window status. Returns: (new_mode: str, mode_changed: bool,
        dwell_elapsed: float)

        DWELL TIME logic (only for display→dwell): - valid_count dropped below REVERT_THRESHOLD →
        start dwell timer - valid_count rose again within Dwell time → cancel dwell, stay on display
        - Dwell time expired and still low → switch to dwell
        """
        now = time.time()
        dwell_elapsed = 0.0

        # Manual stop crushes everything
        if manual_stop:
            self.revert_pending_since = None
            if self.current_mode != 'position_based':
                return 'position_based', True, 0.0
            return 'position_based', False, 0.0

        # Change your decision if the window is not filled enough
        if len(self.window) < self.WINDOW_SIZE:
            return self.current_mode, False, 0.0

        if self.current_mode == 'position_based':
            # Positioned → display: high threshold, instant decision (window already debounce)
            self.revert_pending_since = None
            if valid_count >= self.TRANSITION_THRESHOLD:
                return 'visual', True, 0.0
            return 'position_based', False, 0.0

        elif self.current_mode == 'visual':
            # Displayed → positioned: low threshold + dwell time

            if valid_count <= self.REVERT_THRESHOLD:
                # Below threshold — start or continue dwell
                if self.revert_pending_since is None:
                    self.revert_pending_since = now
                    print(f"[DECISION] Subthreshold detected ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell started ({self.REVERT_DWELL_SECONDS}s standby)", flush=True)

                dwell_elapsed = now - self.revert_pending_since

                if dwell_elapsed >= self.REVERT_DWELL_SECONDS:
                    # Dwell is full, make the comeback
                    self.revert_pending_since = None
                    return 'position_based', True, dwell_elapsed

                # We're still waiting, stay in video
                return 'visual', False, dwell_elapsed

            else:
                # Threshold exceeded — cancel dwell if present
                if self.revert_pending_since is not None:
                    elapsed = now - self.revert_pending_since
                    print(f"[DECISION] Target appeared back ({valid_count}/{self.WINDOW_SIZE}), "
                          f"dwell canceled (elapsed time: {elapsed:.2f}s)", flush=True)
                    self.revert_pending_since = None

                return 'visual', False, 0.0

        return self.current_mode, False, 0.0

    def _maybe_print_status(self, valid_count, frame_valid, reason, conf, cov,
                            manual_stop, dwell_elapsed, mode_changed):
        """
        Start the status line in the terminal — but only if: (a) there has been a significant
        change, OR (b) HEARTBEAT_INTERVAL has passed since the last print. The mode change is
        already suppressed elsewhere, we do not suppress it again here.
        """
        now = time.time()
        dwell_active = self.revert_pending_since is not None

        # "Signature" of the current state — printed if any of them have changed
        snapshot = (
            self.current_mode,
            frame_valid,
            reason,
            dwell_active,
            manual_stop,
        )

        changed = (snapshot != self.last_status_snapshot)
        heartbeat_due = (now - self.last_status_time) >= self.HEARTBEAT_INTERVAL

        # Mode change is already suppressed separately; let's not press it again here
        if mode_changed:
            self.last_status_snapshot = snapshot
            self.last_status_time = now
            return

        if not (changed or heartbeat_due):
            return

        # ─── Create line ─── Last frame summary
        if frame_valid:
            last_info = f"end: VALID (conf={conf:.2f}, conf={cov:.1f}%)"
        else:
            last_info = f"end: INVALID ({reason})"

        # Dwell information (if available)
        dwell_info = ""
        if dwell_active:
            dwell_info = f" | DWELL {dwell_elapsed:.2f}/{self.REVERT_DWELL_SECONDS}s"

        # Manual stop warning
        manual_info = " | [MANUAL STOP]" if manual_stop else ""

        # Window occupancy (specify if not yet full)
        if len(self.window) < self.WINDOW_SIZE:
            window_info = f"Window: {valid_count}/{self.WINDOW_SIZE} (not filled)"
        else:
            window_info = f"window: {valid_count} / {self.WINDOW_SIZE}"

        # Mark whether it is heartbeat or change
        tag = "Δ" if changed else "·"  # Δ=change, ·=heartbeat

        print(f"[KARAR] {tag} MODE:{self.current_mode.upper():10s} | "
              f"{window_info} | {last_info}{dwell_info}{manual_info}",
              flush=True)

        self.last_status_snapshot = snapshot
        self.last_status_time = now



    def process_frame(self, bbox_data):
        """It is called when a new frame arrives."""
        with self.lock:
            frame_valid, conf, cov, reason = self._evaluate_frame(bbox_data)

            self.window.append({
                'valid': frame_valid,
                'conf': conf,
                'cov': cov,
                'reason': reason,
                'time': time.time()
            })

            valid_count = sum(1 for f in self.window if f['valid'])
            manual_stop = self._check_manual_stop()
            new_mode, changed_value, dwell_elapsed = self._make_decision(valid_count, manual_stop)


            if changed_value:
                previous_mode = self.current_mode
                self.current_mode = new_mode
                self._publish_mode()

                if manual_stop:
                    print(f"[DECISION] >>> MANUAL STOP ACTIVE: {previous_mode} → {new_mode}", flush=True)
                elif new_mode == 'position_based':
                    print(f"[DECISION] >>> MODE CHANGED: {previous_mode} → {new_mode} "
                        f"(dwell completed: {dwell_elapsed:.2f}s, window: {valid_count}/{self.WINDOW_SIZE})", flush=True)
                else:
                    print(f"[DECISION] >>> MODE CHANGED: {previous_mode} → {new_mode} "
                        f"(window: {valid_count}/{self.WINDOW_SIZE} valid)", flush=True)

                # ─── RESET WINDOW WHEN MODE CHANGE ─── Data from the old mode should not contaminate the decision in
                # the new mode. The window will be filled from the beginning, so that there is no oscillation at the
                # full limit
                self.window.clear()
                self.revert_pending_since = None   # Cancel pending dwell, if any.
                print(f"[DECISION] Window and dwell state reset (clean start after mod switch)", flush=True)


            # ─── STATUS PRINT (change OR heartbeat) ───
            self._maybe_print_status(
                valid_count=valid_count,
                frame_valid=frame_valid,
                reason=reason,
                conf=conf,
                cov=cov,
                manual_stop=manual_stop,
                dwell_elapsed=dwell_elapsed,
                mode_changed=changed_value
            )


            if self.csv_writer:
                self.csv_writer.writerow([
                    f"{time.time():.4f}",
                    self.current_mode,
                    int(frame_valid),
                    f"{conf:.3f}",
                    f"{cov:.2f}",
                    reason,
                    valid_count,
                    f"{len(self.window)}/{self.WINDOW_SIZE}",
                    int(self.revert_pending_since is not None),
                    f"{dwell_elapsed:.2f}",
                    int(changed_value)
                ])
                if len(self.window) % 30 == 0:
                    self.log_file.flush()

            return self.current_mode

    def run(self):
        """Main loop: listen for Redis, make decisions for each incoming bbox"""
        print("[DECISION] Active. Tracker bbox is listening...", flush=True)
        try:
            while True:
                message = self.pubsub.get_message(
                    ignore_subscribe_messages=True,
                    timeout=0.1
                )

                bbox_data = None
                if message and message['type'] == 'message':
                    try:
                        data_str = message['data'].decode('utf-8')
                        bbox_data = ast.literal_eval(data_str)
                    except Exception as e:
                        print(f"[DECISION] Bbox parse error: {e}", flush=True)
                        bbox_data = None

                if bbox_data is not None:
                    self.process_frame(bbox_data)
                    self.last_frame_time = time.time()
                else:
                    # No message received — add dummy 'no detection' to window if no detection for long period of time
                    now = time.time()
                    if (now - self.last_frame_time) > 0.1:
                        self.process_frame(None)
                        self.last_frame_time = now

        except KeyboardInterrupt:
            print("\n[DECISION] Closing...", flush=True)
        finally:
            if self.log_file:
                self.log_file.close()
            try:
                self.pubsub.close()
            except Exception:
                pass


if __name__ == '__main__':
    decider = TransitionDecider()
    decider.run()
