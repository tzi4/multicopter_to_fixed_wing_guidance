#!/usr/bin/env python3
"""REAL HARDWARE tilt controller: ArduPilot servo-mount via MAVLink.

Hardware counterpart of tools/gz_gimbal.TiltCommander in sim -- SAME interface (start() / target(deg)
/ stop()), so TiltTracking law and bbox chain can drive the real gimbal without any changes.

Command path (both tried, working used): 1. MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (ArduPilot 4.3+ mount
drivers) 2. MAV_CMD_DO_MOUNT_CONTROL (old but common working way in servo mount) Angle sign: OUR
contract positive = UP (world elevation). MAVLink pitch also expects an upward positive degree
(ArduPilot mount comments like this).

Self-testing: python3 tools/mavlink_tilt.py --connection-value tcp:127.0.0.1:5760 (sweeps first; monitors
and reports channel mount on SERVO_OUTPUT_RAW)
"""

import argparse
import threading
import time


class MavlinkTiltCommander:
    """ArduPilot sends tilt (pitch) angle from background thread to mount."""

    def __init__(self, connection_value, dead_band_deg=0.2, min_interval_s=0.1,
                 refresh_s=2.0, method_value2='auto', sysid=1,
                 servo_slew_dps=150.0, tick_s=0.01):
        """connection: address pymavlink (e.g. 'tcp:127.0.0.1:5760', '/dev/ttyACM0', 'udpin:0.0.0.0:14550') OR
canned bluetil connection. method: 'auto' | 'new' (GIMBAL_MANAGER_PITCHYAW) | 'old'
        (DO_MOUNT_CONTROL).

        servo_slew_dps/tick_s: the sender thread does not jump DIRECTLY to the target, it RAMPS in
        the range of tick_s with the speed of servo_slew_dps. Reason: EMAX EG08MD (0.08-0.10 s/60
        deg ~ 600-750 dps, deadband 1.5 base) instantly captures sparse large steps and stands like
        a staircase; In the 100 Hz small steps are lost in mechanical inertia -> fluid movement.
        servo_slew_dps should be selected below the physical limit (~600)."""
        self._connection_arg = connection_value
        self.m = None if isinstance(connection_value, str) else connection_value
        self.dead_band = float(dead_band_deg)
        self.min_interval = float(min_interval_s)
        self.refresh_value = float(refresh_s)
        self.method_value2 = method_value2
        self.sysid = sysid
        self.servo_slew = float(servo_slew_dps)
        self.tick = float(tick_s)
        self.target_deg = None
        self.output_deg = None       # instantaneous output of ramp
        self._last_tick = None
        self.published_deg = None
        self.last_broadcast_t = 0.0
        self.error_n = 0
        self._stop = False
        self._wake = threading.Event()
        self._is = threading.Thread(target=self._loop, daemon=True)

    def start_value2(self, initial_target_deg=0.0):
        """It connects, selects the command path SYNCHRONOUSLY (if auto) and opens the thread.

        ATTENTION: The pymavlink connection is NOT thread-safe. This class only recv in start(); The
        sender thread ONLY sends. If anyone else is recving from the same connection, call start()
        BEFORE that.
        """
        if self.m is None:
            from pymavlink import mavutil
            self.m = mavutil.mavlink_connection(self._connection_arg,
                                                source_system=250)
            self.m.wait_heartbeat(timeout=30)
        if self.method_value2 == 'auto':
            self.method_value2 = self._method_select(float(initial_target_deg))
        else:
            self._mount_configure()
        print(f"[mavlink_tilt] command path: {self.method_value2}", flush=True)
        self._is.start()
        return self

    def target_value(self, deg):
        self.target_deg = float(deg)
        self._wake.set()

    # ---------------------------------------------------------------- ic
    def _send_new(self, deg):
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0,
            deg,        # pitch [deg, + up]
            float('nan'),  # yaw: dokunma
            0, 0,       # pitch/yaw speed: default
            0,          # bayraklar
            0, 0)       # gimbal device id, -

    def _send_previous(self, deg):
        # MEASURED (2026-08-08, ArduPilot 4.6.3 + servo mount): ONLY this command moves the servo. Stream
        # message MOUNT_CONTROL is ignored in 4.6 (difference=1 exponent), GIMBAL_MANAGER_SET_PITCHYAW also
        # does not play. To limit the ACK overhead, send <= 50 Hz should be kept (PWM is already 50 Hz, faster
        # will not help).
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
            deg,     # pitch [deg, + up]
            0,       # roll
            0,       # yaw
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def _ack_wait(self, command_id, duration_value=1.0):
        finish = time.time() + duration_value
        while time.time() < finish:
            a = self.m.recv_match(type='COMMAND_ACK', blocking=True,
                                  timeout=max(0.05, finish - time.time()))
            if a is not None and a.command == command_id:
                return a.result
        return None

    def _mount_configure(self):
        """Set the mount mode to MAVLINK_TARGETING (essential for listening to DO_MOUNT_CONTROL; measured at
SITL: without configure the servo remains at 1500)."""
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            0, 0, 0, 0, 0, 0)
        self._ack_wait(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE)

    def _method_select(self, deg):
        """Select the working path (auto). MEASURED COURSE (SITL, 2026-08-06): DO_GIMBAL_MANAGER_PITCHYAW is
ACK'd but in some setups the servo DOES NOT MOVE -- ACK is not evidence. Universal working recipe:
DO_MOUNT_CONFIGURE(MAVLINK_TARGETING) + DO_MOUNT_CONTROL. 'new' is selected only on purpose with
--method-value2 new."""
        self._mount_configure()
        self._send_previous(deg)
        from pymavlink import mavutil
        r = self._ack_wait(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL)
        if r != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.error_n += 1
        return 'previous'

    def _loop(self):
        while not self._stop:
            self._wake.wait(timeout=self.tick)
            self._wake.clear()
            h = self.target_deg
            if h is None:
                continue
            now_value = time.monotonic()
            dt = (self.tick if self._last_tick is None
                  else min(0.5, now_value - self._last_tick))
            self._last_tick = now_value
            # ramp to target: breaks large jumps into fine steps of 100 Hz
            if self.output_deg is None:
                self.output_deg = float(h)
            else:
                step_value = max(-self.servo_slew * dt,
                           min(self.servo_slew * dt, h - self.output_deg))
                self.output_deg += step_value
            changed_value = (self.published_deg is None
                       or abs(self.output_deg - self.published_deg)
                       > self.dead_band)
            stale_value = now_value - self.last_broadcast_t > self.refresh_value
            if not (changed_value or stale_value):
                continue
            if now_value - self.last_broadcast_t < self.min_interval:
                continue
            try:
                if self.method_value2 == 'new_value':
                    self._send_new(self.output_deg)
                else:
                    self._send_previous(self.output_deg)
                self.published_deg = self.output_deg
                self.last_broadcast_t = now_value
            except Exception:
                self.error_n += 1
                time.sleep(0.5)

    def stop_value(self):
        self._stop = True
        self._wake.set()


def _servo_track(m, channel_value, duration_value=2.0):
    """listens to SERVO_OUTPUT_RAW; Returns (min, max, message_n, all).

    all: {channel_value: (min, maks)} -- ALL non-zero outputs; With message_n, 'no flow' / 'channel empty'
    distinction and channel discovery can be made."""
    finish = time.time() + duration_value
    message_n = 0
    all_value3 = {}
    while time.time() < finish:
        s = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.5)
        if s is None:
            continue
        message_n += 1
        for k in range(1, 17):
            v = getattr(s, f'servo{k}_raw', None)
            if v:
                lo, hi = all_value3.get(k, (int(v), int(v)))
                all_value3[k] = (min(lo, int(v)), max(hi, int(v)))
    lo, hi = all_value3.get(channel_value, (None, None))
    return lo, hi, message_n, all_value3


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--connection-value', default='udp:127.0.0.1:14554')
    p.add_argument('--channel-value', type=int, default=9,
                   help='Output of servo mount pitch ( SERVOx_FUNCTION = 7 )')
    p.add_argument('--method-value2', default='auto', choices=['auto', 'new_value', 'previous'])
    a = p.parse_args()

    k = MavlinkTiltCommander(a.connection_value, method_value2=a.method_value2).start_value2(initial_target_deg=0.0)
    from pymavlink import mavutil
    # SERVO_OUTPUT_RAW (id 36 ) may not appear in the default stream;  Request 10 Hz
    k.m.mav.command_long_send(
        k.m.target_system, k.m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        36, int(1e5), 0, 0, 0, 0, 0)
    print(f"connected: {a.connection_value}; sweep begins (watching channel {a.channel_value})")
    result_value = []
    total_message = 0
    motion = {}  # channel -> seen across sweep (min, max)
    for deg in (0.0, 30.0, -30.0, 0.0):
        k.target_value(deg)
        time.sleep(2.0)
        lo, hi, message_n, all_value3 = _servo_track(k.m, a.channel_value, 1.5)
        total_message += message_n
        for ch, (l, h) in all_value3.items():
            gl, gh = motion.get(ch, (l, h))
            motion[ch] = (min(gl, l), max(gh, h))
        print(f"  target {deg:+6.1f} deg -> servo{a.channel_value} PWM {lo}..{hi} "
              f"({message_n} message)")
        result_value.append((deg, lo))
    k.stop_value()
    if total_message == 0:
        print("RESULT: SERVO_OUTPUT_RAW NEVER ARRIVED -- the problem is not the channel. "
              "telemetry stream. This connection (proxy/router?) message id 36 "
              "not transmitting or SET_MESSAGE_INTERVAL was rejected. direct "
              "'set streamrate 10' with autopilot connection or in mavproxy "
              "Try with .")
        raise SystemExit(2)
    varying = {ch: lohi for ch, lohi in motion.items()
               if lohi[1] - lohi[0] > 100}
    if varying:
        print("  outputs varying by >100us during the sweep: "
              + ", ".join(f"servo{ch} ({l}..{h})"
                          for ch, (l, h) in sorted(varying.items())))
    p0 = dict(result_value)
    if None in (p0.get(30.0), p0.get(-30.0)):
        print(f"RESULT: {total_message} message arrived but servo{a.channel_value} always 0 -- "
              f"No function is assigned to this output. Mount pitch channel "
              f"(SERVOx_FUNCTION=7) give with --channel-value; candidates above "
              f"in the 'varying outputs' row.")
        raise SystemExit(2)
    difference = p0[30.0] - p0[-30.0]
    print(f"RESULT: PWM difference between +30 and -30 {difference} "
          f"({'MOTION PRESENT' if abs(difference) > 100 else 'MOTION NONE - MNT/SERVO ayarlarini control et'})")
    raise SystemExit(0 if abs(difference) > 100 else 1)


if __name__ == '__main__':
    main()
