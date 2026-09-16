#!/usr/bin/env python3
"""Fiziksel gimbal <-> python koprusu (gimbal dali, 2026-08-05).

Gazebo clashesic does not have python transport binding; the only way is `gz` CLI. Two measured
traps (GIMBAL_NOTES.md): * Do not call `gz topic -e` periodically: each call creates new transport
connection = ~1 s. Use ONE persistent process + background thread. * stdbuf -oL CONDITION: gz
buffers blocks to the pipeline; In low-speed status flow, the buffer does not empty for minutes. *
`gz topic -p` also pays ~1 s per process -> the command thread should NEVER block the main loop
(that's what TiltCommander is for).

Topic contract (plugin binds to parent model, name from wrapper in world): command :
/gazebo/default/<model>/gimbal_tilt_cmd (rad, GzString) = WORLD elevation of camera, positive = up
(stabilized mode) status : /gazebo/default/<model>/gimbal_tilt_status (rad, ~18 Hz measured)
"""

import os
import re
import signal
import subprocess
import threading
import time

_STATUS_RE = re.compile(r'data:\s*"(-?[\d.eE+-]+)"')


def command_topic(model):
    return f'/gazebo/default/{model}/gimbal_tilt_cmd'


def state_topic(model):
    return f'/gazebo/default/{model}/gimbal_tilt_status'


def model_name_from_topic(ros_topic):
    """'/drone_3/webcam/image_raw' -> 'iris-3' (worlds/*.world sarmalayici
    adlari). Eslesmezse None."""
    m = re.match(r'^/drone_(\d+)/', str(ros_topic))
    return f'iris-{m.group(1)}' if m else None


def tilt_command(model, rad, env=None, timeout=20):
    """ONE TIME release. ~1 blocks s; call from hot loops, use TiltCommander."""
    r = subprocess.run(['gz', 'topic', '-p', command_topic(model),
                        '-m', f'data: "{float(rad)}"'],
                       capture_output=True, text=True,
                       env=env, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(f'gz topic -p failed: {r.stderr.strip()}')


class TiltStateReader:
    """It reads gimbal_tilt_status in a single continuous process `gz topic -e`."""

    def __init__(self, model, env=None):
        self.model = model
        self.env = env
        self.value_rad = None
        self.time_value = 0.0          # monotonic; freshness = time.monotonic()-time
        self.n = 0
        self._stop = False
        self._proc = None

    def start_value2(self):
        self._proc = subprocess.Popen(
            ['stdbuf', '-oL', 'gz', 'topic', '-e', state_topic(self.model), '-u'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=self.env, start_new_session=True)
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _loop(self):
        buffer = ''
        while not self._stop:
            try:
                part = self._proc.stdout.read1(4096)
            except Exception:
                break
            if not part:
                break
            buffer += part.decode('utf-8', 'replace')
            last_value = 0
            for m in _STATUS_RE.finditer(buffer):
                try:
                    self.value_rad = float(m.group(1))
                except ValueError:
                    last_value = m.end()
                    continue
                self.time_value = time.monotonic()
                self.n += 1
                last_value = m.end()
            buffer = buffer[last_value:]
            if len(buffer) > 16384:
                buffer = buffer[-1024:]

    def age_s_value(self):
        return None if self.n == 0 else time.monotonic() - self.time_value

    def stop_value(self):
        self._stop = True
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                pass


def _permanent_publisher_path():
    ap_gz = os.environ.get('ARDUPILOT_GAZEBO_DIR',
                           os.path.expanduser('~/ardupilot_gazebo'))
    path_value = os.path.join(ap_gz, 'build', 'gz_tilt_pub')
    return path_value if os.access(path_value, os.X_OK) else None


class TiltCommander:
    """Broadcasts the pinball target in the background; It does not block the main loop.

    TWO BACK THREE (Phase O, 2026 - 08 - 06 ): * PERMANENT: gz_tilt_pub ( gimbal_setup , C++).
    Transport connection is established once, each subsequent broadcast ~ ms . Dynamic tilt tracking
    of Phase O is possible with this ( eps 40 - 70 deg /s can change in the terminal). * BACKUP: `gz
    topic -p` (~ 1 per transaction). On machines without gz_tilt_pub compiled, Phase A/B behavior is
    preserved;  min_interval automatically pulls to 1 . Deadband + refresh applies to both backends
    (refresh: command is not lost if gzserver restarts).
    """

    def __init__(self, model, env=None, dead_band_deg=0.1, min_interval_s=None,
                 refresh_s=30.0):
        self.model = model
        self.env = env
        self.dead_band = float(dead_band_deg)
        self.refresh_value = float(refresh_s)
        self.target_deg = None      # desired elevation [ deg ]
        self.published_deg = None
        self.last_broadcast_t = 0.0
        self.error_n = 0
        self.permanent = None         # Popen(gz_tilt_pub) or None
        self._stop = False
        self._wake = threading.Event()
        self._is = threading.Thread(target=self._loop, daemon=True)
        self._publisher_path = _permanent_publisher_path()
        if min_interval_s is None:
            min_interval_s = 0.03 if self._publisher_path else 1.0
        self.min_interval = float(min_interval_s)

    def start_value2(self):
        if self._publisher_path:
            try:
                self.permanent = subprocess.Popen(
                    [self._publisher_path, self.model],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE, env=self.env, text=True,
                    bufsize=1, start_new_session=True)
                # handshake: print "READY" when WaitForConnection completes
                ready = threading.Event()

                def _wait():
                    for row_value in self.permanent.stderr:
                        if 'READY' in row_value:
                            ready.set()
                            break
                threading.Thread(target=_wait, daemon=True).start()
                if not ready.wait(timeout=20):
                    self.permanent.kill()
                    self.permanent = None
            except Exception:
                self.permanent = None
        if self.permanent is None:
            self.min_interval = max(self.min_interval, 1.0)
        self._is.start()
        return self

    def target_value(self, deg):
        self.target_deg = float(deg)
        self._wake.set()

    def _publish(self, h_deg):
        import math
        if self.permanent is not None and self.permanent.poll() is None:
            self.permanent.stdin.write(f"{math.radians(h_deg):.6f}\n")
            self.permanent.stdin.flush()
        else:
            if self.permanent is not None:      # Became permanent publisher: became a backup
                self.permanent = None
                self.min_interval = max(self.min_interval, 1.0)
                self.error_n += 1
            tilt_command(self.model, math.radians(h_deg), env=self.env)

    def _loop(self):
        while not self._stop:
            self._wake.wait(timeout=1.0)
            self._wake.clear()
            h = self.target_deg
            if h is None:
                continue
            now_value = time.monotonic()
            changed_value = (self.published_deg is None
                       or abs(h - self.published_deg) > self.dead_band)
            stale_value = now_value - self.last_broadcast_t > self.refresh_value
            if not (changed_value or stale_value):
                continue
            if now_value - self.last_broadcast_t < self.min_interval:
                # try again soon (wakeup for quick updates in persistent mode is held, not missed)
                self._wake.set()
                time.sleep(self.min_interval / 2.0)
                continue
            try:
                self._publish(h)
                self.published_deg = h
                self.last_broadcast_t = now_value
            except Exception:
                self.error_n += 1
                time.sleep(1.0)

    def stop_value(self):
        self._stop = True
        self._wake.set()
        if self.permanent is not None and self.permanent.poll() is None:
            try:
                self.permanent.stdin.close()
            except Exception:
                pass
            try:
                os.killpg(os.getpgid(self.permanent.pid), signal.SIGTERM)
            except Exception:
                pass


class TiltTracking:
    """PHASE C: moves the tilt target to the MEASURED RISE OF THE TARGET.

    Since the input ey (stab vertical error, relative to the horizon; ey<0 = target is above the
    horizon) is already measured in the WORLD frame with the live-joint chain, target rise = -ey +
    current_virtual_center... flat at aim=0: e_t = -ey + 0 => e_t = -ey. CAUTION: ey is
    tilt-INDEPENDENT (measured relative to horizon), so this is not a feedback loop but a filtered
    FOLLOW-UP of the measured quantity -- no stability risk, just delay.

    Protection layers (lessons from AimTrim + Phase O facts): * EMA filter (tau): single frame
    detection noise does not drive tilt * slew limit: physical servo 6 rad/s; command stay slower
    than ten * clamp [bottom, top]: vs ground/sky scan * loss hold: last target held if detection
    was down (loss_hold_s), then SLOWLY returns to default (standoff) angle -- reacquisition pose
    """

    def __init__(self, default_deg, tau_s=0.4, slew_dps=60.0,
                 alt_deg=-30.0, upper_deg=60.0, loss_hold_s=3.0,
                 turn_dps=10.0):
        self.default_value2 = float(default_deg)
        self.tau = float(tau_s)
        self.slew = float(slew_dps)
        self.alt, self.upper_value = float(alt_deg), float(upper_deg)
        self.loss_hold = float(loss_hold_s)
        self.turn = float(turn_dps)
        self.cmd = float(default_deg)
        self._filter = None
        self._last_detection_t = None

    def update_value(self, target_elev_deg, dt, now_value=None):
        """target_elev_deg: measured world rise of the target (= -ey), None if not detected. Returns the new
tilt command [deg]."""
        if now_value is None:
            now_value = time.monotonic()
        dt = max(1e-3, min(float(dt), 0.5))
        if target_elev_deg is not None:
            self._last_detection_t = now_value
            if self._filter is None:
                self._filter = float(target_elev_deg)
            else:
                k = dt / (dt + self.tau)
                self._filter += k * (float(target_elev_deg) - self._filter)
            requested_value, speed_value = self._filter, self.slew
        elif (self._last_detection_t is not None
              and now_value - self._last_detection_t < self.loss_hold):
            return self.cmd                        # tutu
        else:
            requested_value, speed_value = self.default_value2, self.turn   # reacquisition
            self._filter = None
        step_value = max(-speed_value * dt, min(speed_value * dt, requested_value - self.cmd))
        self.cmd = max(self.alt, min(self.upper_value, self.cmd + step_value))
        return self.cmd
