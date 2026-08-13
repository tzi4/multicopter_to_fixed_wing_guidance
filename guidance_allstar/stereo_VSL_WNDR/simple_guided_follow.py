#!/usr/bin/env python3
"""
Simple target-follow runner.

Reads the target position, estimates target velocity with filterwndr.py, computes a
slot behind the target, and repeatedly sends that slot to the pursuer as an
ArduCopter GUIDED local-NED position target.

This is intentionally separate from the PN/lag-pursuit attitude-control scripts.
"""

import argparse
import atexit
import csv
import signal
import time
from datetime import datetime
from pathlib import Path

import guidance_config as cfg
import mavlink_utils
import numpy as np
from filterwndr import (
    HeadingTurnRateEstimator,
    IMMLowPassFilter,
    aggregate_mode_probabilities,
    apply_fast_turn_onset_hint,
    apply_turn_rate_hint,
    clamp_filter_dt,
    ct_mode_probability,
    predict_imm_over_dt,
    restore_imm_vertical_state,
    setup_imm_filter,
    snapshot_imm_vertical_state,
    stabilize_omega_states,
    update_imm_preserving_vertical,
)
from guidance_gui import GuidanceGUI, gui_tick, push_snapshot
from pymavlink import mavutil

redcolor = "\033[0;31m"
bluecolor = "\033[0;34m"
endcolor = "\033[0m"

POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

POSITION_VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# Position + velocity + acceleration feedforward (only yaw/yaw-rate ignored).
POSITION_VELOCITY_ACCEL_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def clamp_norm(v, max_norm):
    norm = float(np.linalg.norm(v))
    if norm > max_norm and norm > 1e-6:
        return v * (max_norm / norm)
    return v


def target_feedforward_acceleration(state, max_accel):
    """World-frame (NED) target acceleration for setpoint feedforward.

    The IMM carries a coordinated turn as a turn rate, not in the ax/ay states
    (CTxy modes stabilize ax/ay to zero), so the horizontal acceleration during
    a turn is the centripetal term omega x v_xy, not state[6:8]. This combines
    both: the CAxy/CAz-weighted linear acceleration plus the CT centripetal
    acceleration. They do not double-count -- CT modes zero the linear part and
    CA modes gate omega to zero.

    Vertical acceleration is intentionally dropped (az = 0): feeding it forward
    reintroduces the Z command transients this project spent a long time taming,
    and the lead value of accel FF is almost entirely horizontal. The horizontal
    magnitude is clamped to reject noisy acceleration/omega spikes.
    """
    state = np.asarray(state, dtype=float).reshape(-1)
    vx, vy = float(state[3]), float(state[4])
    a_lin_x, a_lin_y = float(state[6]), float(state[7])
    omega = float(state[9]) if state.shape[0] > 9 else 0.0

    # Centripetal acceleration of a coordinated horizontal turn: a = omega x v.
    ax = a_lin_x - omega * vy
    ay = a_lin_y + omega * vx
    a_ff = np.array([ax, ay, 0.0])
    return clamp_norm(a_ff, max_accel)


def accel_ff_range_scale(range_m, extend_range_m, fade_band_m):
    """Fade the acceleration feedforward from 1 (midcourse) to 0 as the pursuer
    enters the terminal-extension band.

    Inside the extension range the aim point is deliberately pushed past the
    target, so adding target acceleration there makes the position controller
    anticipate continued acceleration and blow through the target (overshoot).
    Returns 1 beyond (extend_range + fade_band), ramping linearly to 0 at
    extend_range and below.
    """
    inner = float(extend_range_m)
    band = max(0.0, float(fade_band_m))
    if band <= 1e-6:
        return 1.0 if float(range_m) > inner else 0.0
    return float(np.clip((float(range_m) - inner) / band, 0.0, 1.0))


class GuidanceLogger:
    """Per-loop CSV of guidance state, for diagnosing misses/overshoots.

    Unlike the estimator's imm_diagnostics logs (which only cover target
    estimation), this records the pursuer, the commanded setpoint, the intercept
    geometry, and the terminal/accel-FF state, so a miss can be traced.
    """

    FIELDNAMES = [
        "wall_time",
        "range_m",
        "closing_velocity",
        "t_go_s",
        "guidance_horizon_s",
        "pursuer_x", "pursuer_y", "pursuer_z",
        "meas_x", "meas_y", "meas_z",
        "est_x", "est_y", "est_z",
        "slot_x", "slot_y", "slot_z",
        "slot_vx", "slot_vy", "slot_vz",
        "aff_x", "aff_y", "aff_z", "aff_mag",
        "extended", "latched", "latch_armed",
        "recovery_state",
        "z_limited", "aim_limited", "alt_floored",
        "lead_k_h", "lead_k_v", "mu_ct",
        "yaw_rate_dps", "alt_err_m", "cpa_range_m",
    ]

    def __init__(self, log_dir="logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"guided_follow_{stamp}.csv"
        self.file = self.path.open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self._rows_since_flush = 0
        # Flush the tail even when main() dies on an exception (close() is
        # idempotent, so the normal-path close costs nothing extra).
        atexit.register(self.close)

    def write(self, row):
        self.writer.writerow(row)
        self._rows_since_flush += 1
        if self._rows_since_flush >= 25:
            self.file.flush()
            self._rows_since_flush = 0

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()


class MissRecoveryController:
    """State machine for graceful recovery after a terminal miss.

    The aggressive terminal dive (extension + freeze latch) is kept intact for
    hit probability. But on the pass, instead of instantly re-chasing at full
    speed from a bad attitude (which sends the pursuer diverging), it hands off:

        CHASE     normal guidance (terminal dive + latch). Vehicle in GUIDED.
          | terminal latch releases after a pass (miss)
        HOLD      switch to a recovery flight mode (BRAKE/STABILIZE/LOITER) for
          |       recovery_hold_s to bleed off speed and level out. No setpoints.
        REENGAGE  back in GUIDED; re-approach the target speed-limited, ramping
          |       from SAFE_TURN_SPEED up to full over reengage_ramp_s, no
          |       extension/latch, until closing again (or a timeout).
        CHASE     resume full guidance.

    Only the transitions are decided here; the loop applies the flight mode,
    whether to send a setpoint, and the speed cap.
    """

    CHASE = "CHASE"
    HOLD = "HOLD"
    REENGAGE = "REENGAGE"

    def __init__(self, args):
        self.enabled = bool(args.miss_recovery) and not args.no_guided
        self.recovery_mode = str(args.recovery_mode)
        self.hold_s = max(0.0, float(args.recovery_hold_s))
        self.ramp_s = max(0.0, float(args.reengage_ramp_s))
        self.max_reengage_s = max(0.1, float(args.reengage_max_s))
        self.min_closing = float(getattr(cfg, "REENGAGE_MIN_CLOSING_MPS", 2.0))
        self.safe_turn_speed = float(getattr(cfg, "SAFE_TURN_SPEED", 7.0))
        self.max_speed = float(args.max_feedforward_speed)
        # Debounce (2026-07-11: latch noise fired BRAKE 12x in one flight,
        # several at 100+ m true range; each BRAKE-at-speed is a saturation
        # event). A fresh CHASE gets chase_dwell_s to fly before a miss may
        # interrupt it, and HOLDs are spaced at least brake_interval_s apart.
        self.chase_dwell_s = float(getattr(cfg, "RECOVERY_CHASE_DWELL_S", 15.0))
        self.brake_interval_s = float(
            getattr(cfg, "RECOVERY_BRAKE_MIN_INTERVAL_S", 5.0)
        )
        # Altitude-divergence abort: fire HOLD/BRAKE when the vehicle can't hold
        # its commanded altitude (motor saturation -> thrust collapse), a mode the
        # range-based miss trigger is blind to. Arms only after altitude is first
        # acquired so the initial climb doesn't trip it; disarms on fire until
        # re-acquired to avoid thrash. See RECOVERY_ALT_ABORT_M in the config.
        self.alt_abort_m = float(getattr(cfg, "RECOVERY_ALT_ABORT_M", 15.0))
        self.alt_abort_arm_m = float(getattr(cfg, "RECOVERY_ALT_ABORT_ARM_M", 3.0))
        self._alt_abort_armed = False
        # Yaw-rate hold gate: a fixed hold_s timer released the vehicle while it
        # was still tumbling from the pass. Extend the hold while it is still
        # spinning, bounded by hold_max_s so it can never stick.
        # CPA trigger: declare the terminal pass once range has climbed
        # cpa_margin_m above the minimum seen while latched, instead of waiting
        # for the latch to release at TERMINAL_LATCH_RELEASE_RANGE_M. Consumes
        # only time-consistent packet samples (caller passes range_m=None on
        # non-packet loops) and requires cpa_open_samples consecutive
        # beyond-margin samples -- the every-loop range is a +-5-8 m packet
        # staircase that fired the naive version BEFORE the true pass
        # (2026-07-24 review, log 110805 full-res).
        self.cpa_margin_m = float(getattr(cfg, "RECOVERY_CPA_MARGIN_M", 0.0))
        self.cpa_open_samples = max(
            1, int(getattr(cfg, "RECOVERY_CPA_OPEN_SAMPLES", 2))
        )
        self._cpa_min_range = None
        self._cpa_open_count = 0
        self.yaw_hold_rate = float(
            np.radians(float(getattr(cfg, "RECOVERY_YAW_RATE_HOLD_DPS", 0.0)))
        )
        self.hold_max_s = max(
            self.hold_s, float(getattr(cfg, "RECOVERY_HOLD_MAX_S", 5.0))
        )
        self._hold_extended = False
        self.state = self.CHASE
        self.state_t0 = 0.0
        self.chase_t0 = None
        self.last_hold_t = None
        self._prev_latched = False

    def _chase(self):
        return {
            "state": self.CHASE,
            "mode": "GUIDED",
            "send": True,
            "speed_cap": self.max_speed,
        }

    def update(
        self, now, latched, closing_velocity,
        alt_error=None, yaw_rate=None, range_m=None,
    ):
        """Advance the state machine one loop and return the directive.

        alt_error is |pursuer_z - commanded_z| [m]; when it exceeds
        alt_abort_m the vehicle has lost altitude control (saturation) and we
        HOLD/BRAKE immediately, independent of the range-based miss detection.

        range_m is the pursuer-to-estimate range [m]; while latched, a rise of
        cpa_margin_m above its minimum means the pass has happened (CPA) and we
        hand off to recovery immediately instead of waiting for the latch to
        release 2-6 s later.

        yaw_rate is the body yaw rate [rad/s] (None if unavailable): while it
        exceeds the threshold the HOLD window is extended so the vehicle is not
        handed back to GUIDED mid-tumble.
        """
        if not self.enabled:
            return self._chase()

        if self.state == self.CHASE:
            if self.chase_t0 is None:
                self.chase_t0 = now
            # Arm the altitude abort once the vehicle has demonstrated it can
            # hold the commanded altitude (excludes the initial climb).
            if (
                self.alt_abort_m > 0.0
                and alt_error is not None
                and alt_error <= self.alt_abort_arm_m
            ):
                self._alt_abort_armed = True
            # Altitude-divergence abort: highest priority, bypasses the miss
            # dwell/spacing debounce (a thrust collapse is an emergency, not
            # latch noise). Disarm until altitude is re-acquired so it does not
            # thrash while REENGAGE is still recovering.
            if (
                self._alt_abort_armed
                and alt_error is not None
                and alt_error > self.alt_abort_m
            ):
                self._alt_abort_armed = False
                self._prev_latched = bool(latched)
                # Clear CPA tracking too: a stale minimum from this aborted
                # approach must not survive into the next engagement.
                self._cpa_min_range = None
                self._cpa_open_count = 0
                self.state = self.HOLD
                self.state_t0 = now
                self.last_hold_t = now
                print(
                    f"[{redcolor}simple_follow{endcolor}] ALTITUDE ABORT: "
                    f"|pursuer_z-cmd_z|={alt_error:.1f} m > {self.alt_abort_m:.0f} m "
                    f"(thrust/attitude collapse) -> HOLD/{self.recovery_mode}"
                )
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            # CPA trigger: while the aim is latched, track the closest range
            # over TIME-CONSISTENT packet samples (range_m is None between
            # packets; the tracker holds). Once cpa_open_samples consecutive
            # samples sit cpa_margin_m above the minimum, the pass is real --
            # hand off NOW rather than waiting for the latch to release at
            # 40 m. Bypasses the chase dwell (a confirmed pass is not latch
            # noise); if the brake spacing forbids another HOLD, release the
            # latch instead so the vehicle is never left flying a frozen aim
            # it has already overshot (the balloon mechanism).
            if not latched:
                self._cpa_min_range = None
                self._cpa_open_count = 0
            elif self.cpa_margin_m > 0.0 and range_m is not None:
                r = float(range_m)
                if self._cpa_min_range is None or r < self._cpa_min_range:
                    self._cpa_min_range = r
                    self._cpa_open_count = 0
                elif r > self._cpa_min_range + self.cpa_margin_m:
                    self._cpa_open_count += 1
                    if self._cpa_open_count >= self.cpa_open_samples:
                        closest = self._cpa_min_range
                        samples = self._cpa_open_count
                        self._cpa_min_range = None
                        self._cpa_open_count = 0
                        spacing_ok = (
                            self.last_hold_t is None
                            or (now - self.last_hold_t) >= self.brake_interval_s
                        )
                        if spacing_ok:
                            self._prev_latched = bool(latched)
                            self.state = self.HOLD
                            self.state_t0 = now
                            self.last_hold_t = now
                            print(
                                f"[{redcolor}simple_follow{endcolor}] CPA PASS: "
                                f"closest {closest:.1f} m, opening at {r:.1f} m "
                                f"({samples} packet samples) -> HOLD/"
                                f"{self.recovery_mode} (stop flying the frozen aim)"
                            )
                            return {
                                "state": self.HOLD,
                                "mode": self.recovery_mode,
                                "send": False,
                                "speed_cap": 0.0,
                            }
                        # Pass confirmed but a HOLD is too soon after the last
                        # one: drop the frozen aim and keep chasing live
                        # geometry instead of ballooning against it.
                        self._prev_latched = False
                        print(
                            f"[{redcolor}simple_follow{endcolor}] CPA PASS: "
                            f"closest {closest:.1f} m, opening at {r:.1f} m -- "
                            f"HOLD suppressed (brake spacing), releasing latch "
                            f"and staying in CHASE"
                        )
                        directive = self._chase()
                        directive["unlatch"] = True
                        return directive
                else:
                    # Back inside the margin band: the opening streak is broken.
                    self._cpa_open_count = 0
            miss = self._prev_latched and not latched
            self._prev_latched = bool(latched)
            if miss:
                dwell_ok = (now - self.chase_t0) >= self.chase_dwell_s
                spacing_ok = (
                    self.last_hold_t is None
                    or (now - self.last_hold_t) >= self.brake_interval_s
                )
                if not (dwell_ok and spacing_ok):
                    reason = "chase dwell" if not dwell_ok else "brake spacing"
                    print(
                        f"[{redcolor}simple_follow{endcolor}] miss detected but "
                        f"HOLD suppressed ({reason}); staying in CHASE"
                    )
                    return self._chase()
                self.state = self.HOLD
                self.state_t0 = now
                self.last_hold_t = now
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            return self._chase()

        if self.state == self.HOLD:
            held = now - self.state_t0
            # Still spinning? Keep the recovery mode and go on stabilising
            # rather than handing a tumbling vehicle back to GUIDED. Fails open
            # when yaw_rate is None (no/stale ATTITUDE) and is capped by
            # hold_max_s so a stuck-high yaw rate can never freeze the machine.
            spinning = (
                self.yaw_hold_rate > 0.0
                and yaw_rate is not None
                and abs(float(yaw_rate)) > self.yaw_hold_rate
            )
            if held < self.hold_s or (spinning and held < self.hold_max_s):
                if spinning and held >= self.hold_s and not self._hold_extended:
                    self._hold_extended = True
                    print(
                        f"[{redcolor}simple_follow{endcolor}] HOLD extended: yaw rate "
                        f"{np.degrees(abs(float(yaw_rate))):.0f} deg/s > "
                        f"{np.degrees(self.yaw_hold_rate):.0f} -- still spinning, "
                        f"keep stabilising (cap {self.hold_max_s:.1f}s)"
                    )
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            if self._hold_extended:
                yr = "n/a" if yaw_rate is None else f"{np.degrees(abs(float(yaw_rate))):.0f} deg/s"
                print(
                    f"[{redcolor}simple_follow{endcolor}] HOLD released after "
                    f"{held:.1f}s (yaw rate {yr}) -> REENGAGE"
                )
            self._hold_extended = False
            self.state = self.REENGAGE
            self.state_t0 = now

        # REENGAGE
        elapsed = now - self.state_t0
        frac = 1.0 if self.ramp_s <= 1e-6 else min(1.0, elapsed / self.ramp_s)
        cap = self.safe_turn_speed + (self.max_speed - self.safe_turn_speed) * frac
        reengaged = elapsed >= self.ramp_s and float(closing_velocity) >= self.min_closing
        if reengaged or elapsed >= self.max_reengage_s:
            self.state = self.CHASE
            self.chase_t0 = now
            self._prev_latched = False
            return self._chase()
        return {
            "state": self.REENGAGE,
            "mode": "GUIDED",
            "send": True,
            "speed_cap": cap,
        }


def horizontal_unit(v_xy, fallback_xy):
    norm = float(np.linalg.norm(v_xy))
    if norm > 1e-6:
        return v_xy / norm
    return fallback_xy.copy()


def yaw_to_los(target_pos, pursuer_pos, fallback_yaw=0.0, min_range_m=0.0):
    rel_xy = (
        np.asarray(target_pos, dtype=float).reshape(3)[0:2]
        - np.asarray(pursuer_pos, dtype=float).reshape(3)[0:2]
    )
    if float(np.linalg.norm(rel_xy)) < float(min_range_m):
        return fallback_yaw
    if float(np.dot(rel_xy, rel_xy)) < 1e-6:
        return fallback_yaw
    return float(np.arctan2(rel_xy[1], rel_xy[0]))


def wrap_angle_pi(angle_rad):
    return (float(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi


def slew_angle_toward(current_yaw, target_yaw, max_rate_deg_s, dt):
    max_step = np.deg2rad(max(0.0, float(max_rate_deg_s))) * max(float(dt), 0.0)
    if max_step <= 0.0:
        return float(current_yaw)
    delta = wrap_angle_pi(target_yaw - current_yaw)
    delta = float(np.clip(delta, -max_step, max_step))
    return wrap_angle_pi(current_yaw + delta)


# Set once the runner owns the vehicle's mode, so the shutdown hook can put it
# somewhere safe no matter how we exit (Ctrl-C, SIGTERM, or an exception).
_SHUTDOWN_CONN = None
_SHUTDOWN_DONE = False


def stabilize_on_exit(reason=""):
    """Command the configured shutdown mode (default POSHOLD) once, on exit.

    Without this the vehicle keeps flying the last GUIDED setpoint after the
    script stops. Idempotent, never raises: shutdown must not be able to mask
    the original error or fail twice.
    """
    global _SHUTDOWN_DONE
    if _SHUTDOWN_DONE or _SHUTDOWN_CONN is None:
        return
    _SHUTDOWN_DONE = True
    mode = str(getattr(cfg, "SHUTDOWN_MODE", "POSHOLD") or "").strip()
    if not mode:
        return
    try:
        set_mode(_SHUTDOWN_CONN, mode)
        note = f" ({reason})" if reason else ""
        print(f"[{redcolor}simple_follow{endcolor}] Shutdown{note}: pursuer -> {mode}")
    except Exception as exc:
        print(
            f"[{redcolor}simple_follow{endcolor}] Shutdown: could not set {mode}: {exc}"
        )


def set_mode(master, mode_name):
    modes = master.mode_mapping()
    if not modes:
        # mode_mapping() returns None until the vehicle type is known; without
        # this guard the membership test raises TypeError instead of the
        # intended, catchable RuntimeError.
        raise RuntimeError("Vehicle mode mapping not available yet (no heartbeat?)")
    if mode_name not in modes:
        raise RuntimeError(f"Mode {mode_name!r} is not available on this vehicle")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes[mode_name],
    )


def wait_for_vehicle_heartbeat(
    master, label, expected_sysid=None, timeout_s=10.0, abort_check=None
):
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while True:
        if abort_check is not None and abort_check():
            raise RuntimeError(f"Aborted while waiting for {label} heartbeat")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"Timed out waiting for {label} heartbeat")
        # Recv in short slices so a SIGINT (which only flips the running flag)
        # is noticed within ~0.5 s instead of after the full timeout (PEP 475
        # restarts the interrupted recv otherwise).
        msg = master.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(0.5, remaining)
        )
        if msg is None:
            continue

        src_sys = int(msg.get_srcSystem())
        src_comp = int(msg.get_srcComponent())
        is_gcs = src_comp == mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER or int(
            getattr(msg, "type", -1)
        ) == int(mavutil.mavlink.MAV_TYPE_GCS)
        if expected_sysid is not None and src_sys != int(expected_sysid):
            continue
        if is_gcs:
            continue

        master.target_system = src_sys
        master.target_component = src_comp
        return msg


def request_message_interval(master, message_id, rate_hz):
    interval_us = 0
    if float(rate_hz) > 0.0:
        interval_us = int(round(1_000_000.0 / float(rate_hz)))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def set_param(master, name, value, param_type=None):
    if param_type is None:
        if float(value).is_integer():
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        else:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        str(name).encode("ascii"),
        float(value),
        int(param_type),
    )


def assert_startup_params(master, param_map):
    for name, value in param_map.items():
        set_param(master, name, value)
        print(f"[{redcolor}simple_follow{endcolor}] param_set {name}={value}")


def fetch_pursuer_home(pursuer_conn):
    pursuer_conn.mav.command_long_send(
        pursuer_conn.target_system,
        pursuer_conn.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    msg = pursuer_conn.recv_match(type="HOME_POSITION", blocking=True, timeout=5.0)
    if msg is not None:
        return msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0

    msg = pursuer_conn.recv_match(
        type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0
    )
    if msg is not None:
        return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0

    raise RuntimeError("Could not determine pursuer home/global origin")


def predict_state_copy(imm, horizon_s, max_substep=None):
    if horizon_s <= 0.0:
        return imm.x.copy()

    backup_xs = [kf.x.copy() for kf in imm.filters]
    backup_ps = [kf.P.copy() for kf in imm.filters]
    backup_mu = imm.mu.copy()
    backup_x = imm.x.copy()
    backup_p = imm.P.copy()

    predict_imm_over_dt(imm, horizon_s, max_substep=max_substep)
    predicted = imm.x.copy()

    for kf, x, p in zip(imm.filters, backup_xs, backup_ps):
        kf.x = x
        kf.P = p
    imm.mu = backup_mu
    imm.x = backup_x
    imm.P = backup_p

    return predicted


def clamp_lead_envelope(imm):
    """Bound each filter's poorly-observed states to the target's flight
    envelope, IN PLACE. Must be called inside a backup/restore of the filters.

    The lead is dominated by position + velocity + turn rate (all well
    observed). The states the horizon amplifies -- linear acceleration
    (ax,ay,az via ~t^2/2) and the CT centripetal (v*omega) -- are the least
    observable, so a transient estimate of a few m/s^2 becomes tens of metres
    of aim-point swing. Here we (a) clamp horizontal linear accel magnitude and
    vertical accel to the envelope and scale each by its inclusion factor, (b)
    optionally drop vz, and (c) cap omega so v*omega stays within the lateral
    envelope (OMEGA_ABS_MAX=1.5 alone permits ~28 m/s^2 at 19 m/s).
    """
    lat_max = float(getattr(cfg, "LEAD_LATERAL_ACCEL_MAX_MPS2", 5.0))
    az_max = float(getattr(cfg, "LEAD_VERTICAL_ACCEL_MAX_MPS2", 4.0))
    h_scale = float(getattr(cfg, "LEAD_HORIZONTAL_ACCEL_SCALE", 1.0))
    z_scale = float(getattr(cfg, "LEAD_VERTICAL_ACCEL_SCALE", 0.0))
    vz_scale = float(getattr(cfg, "LEAD_VERTICAL_VELOCITY_SCALE", 0.0))
    for kf in imm.filters:
        x = kf.x
        axy_n = float(np.hypot(x[6], x[7]))
        if axy_n > lat_max and axy_n > 1e-9:
            scale = lat_max / axy_n
            x[6] *= scale
            x[7] *= scale
        x[6] *= h_scale
        x[7] *= h_scale
        x[8] = float(np.clip(x[8], -az_max, az_max)) * z_scale
        x[5] *= vz_scale
        spd = float(np.hypot(x[3], x[4]))
        if spd > 1e-3:
            omega_env = lat_max / spd
            x[9] = float(np.clip(x[9], -omega_env, omega_env))


def predict_lead_state(imm, horizon_s, max_substep=None):
    """Envelope-clamped forward prediction for the guidance lead.

    Returns (x_pred, P_pred): the predicted mixed state and its covariance,
    leaving `imm` untouched. Unlike predict_state_copy it clamps the poorly
    observed states to the flight envelope first (see clamp_lead_envelope), so
    the ~t^2 amplification cannot manufacture an impossible aim point.
    """
    if horizon_s <= 0.0:
        return imm.x.copy(), imm.P.copy()

    backup_xs = [kf.x.copy() for kf in imm.filters]
    backup_ps = [kf.P.copy() for kf in imm.filters]
    backup_mu = imm.mu.copy()
    backup_x = imm.x.copy()
    backup_p = imm.P.copy()

    if bool(getattr(cfg, "LEAD_ENVELOPE_CLAMP_ENABLED", True)):
        clamp_lead_envelope(imm)
    predict_imm_over_dt(imm, horizon_s, max_substep=max_substep)
    x_pred = imm.x.copy()
    p_pred = imm.P.copy()

    for kf, x, p in zip(imm.filters, backup_xs, backup_ps):
        kf.x = x
        kf.P = p
    imm.mu = backup_mu
    imm.x = backup_x
    imm.P = backup_p

    return x_pred, p_pred


def lead_cov_gate(x_pred, p_pred, x_est):
    """Shrink the position lead toward the current estimate when the filter's
    own predicted covariance says the extrapolation is uncertain.

    The predicted position 1-sigma grows with the horizon and with how poorly
    the target is being tracked; when it is large, the mean lead is noise, so
    aiming at it is worse than not leading. Returns (aim_state, k_h, k_v) where
    k_* in [0,1] is the applied lead fraction (horizontal / vertical). Only the
    position channels are gated; velocity FF is left as estimated.
    """
    aim = np.asarray(x_pred, dtype=float).copy()
    if not bool(getattr(cfg, "LEAD_COV_GATE_ENABLED", True)):
        return aim, 1.0, 1.0

    est = np.asarray(x_est, dtype=float).reshape(-1)
    var = np.clip(np.asarray(np.diag(p_pred), dtype=float)[0:3], 0.0, None)
    sig = np.sqrt(var)
    sig_h = float(np.hypot(sig[0], sig[1]) / np.sqrt(2.0))  # rms horizontal std
    sig_v = float(sig[2])

    floor = float(getattr(cfg, "LEAD_COV_GATE_MIN_FRAC", 0.0))

    def frac(s, lo, hi):
        if hi <= lo:
            return 1.0
        return float(np.clip((hi - s) / (hi - lo), floor, 1.0))

    k_h = frac(
        sig_h,
        float(getattr(cfg, "LEAD_COV_GATE_H_SIGMA_LO_M", 10.0)),
        float(getattr(cfg, "LEAD_COV_GATE_H_SIGMA_HI_M", 45.0)),
    )
    k_v = frac(
        sig_v,
        float(getattr(cfg, "LEAD_COV_GATE_V_SIGMA_LO_M", 6.0)),
        float(getattr(cfg, "LEAD_COV_GATE_V_SIGMA_HI_M", 22.0)),
    )

    aim[0] = est[0] + k_h * (float(x_pred[0]) - est[0])
    aim[1] = est[1] + k_h * (float(x_pred[1]) - est[1])
    aim[2] = est[2] + k_v * (float(x_pred[2]) - est[2])
    return aim, k_h, k_v


class LeadPredictionCache:
    """Reuse the guidance lead prediction between target packets.

    The lead point is the predicted intercept location: it only carries new
    information when a measurement has updated the IMM, or when the intercept
    solution shifts materially (t_go, and with it the horizon). Recomputing it
    every guidance loop is what starved the 2026-07-11 flight's control loop:
    a 6 s horizon at 0.1 s substeps x 6 filters measured 415-660 ms per loop,
    so "30 Hz" guidance actually ran at ~2 Hz exactly when the horizon was
    long. Packets arrive at ~4-5 Hz; between them the cached point is simply
    the same impact-point estimate and costs nothing.

    Caches the envelope-clamped prediction (x_pred, P_pred). The covariance
    gate (lead_cov_gate) is cheap and runs every loop on the cached pair.
    """

    def __init__(self, max_substep, horizon_tol_s=0.5):
        self.max_substep = float(max_substep)
        self.horizon_tol_s = float(horizon_tol_s)
        self.x_pred = None
        self.p_pred = None
        self.horizon_s = None

    def get(self, imm, horizon_s, imm_updated):
        horizon_s = float(horizon_s)
        if (
            self.x_pred is None
            or imm_updated
            or abs(horizon_s - self.horizon_s) > self.horizon_tol_s
        ):
            self.x_pred, self.p_pred = predict_lead_state(
                imm, horizon_s, max_substep=self.max_substep
            )
            self.horizon_s = horizon_s
        return self.x_pred.copy(), self.p_pred.copy()


class BehindSlotGuidance:
    def __init__(self, back_m, side_m, down_m, min_heading_speed):
        self.back_m = float(back_m)
        self.side_m = float(side_m)
        self.down_m = float(down_m)
        self.min_heading_speed = float(min_heading_speed)
        self.heading_xy = np.array([1.0, 0.0])

    def update(self, target_state, pursuer_pos):
        target_pos = np.asarray(target_state[0:3], dtype=float).reshape(3)
        target_vel = np.asarray(target_state[3:6], dtype=float).reshape(3)
        pursuer_pos = np.asarray(pursuer_pos, dtype=float).reshape(3)

        target_speed_xy = float(np.linalg.norm(target_vel[0:2]))
        if target_speed_xy >= self.min_heading_speed:
            heading_xy = target_vel[0:2] / target_speed_xy
        else:
            rel_xy = target_pos[0:2] - pursuer_pos[0:2]
            heading_xy = horizontal_unit(rel_xy, self.heading_xy)

        self.heading_xy = heading_xy.copy()

        heading = np.array([heading_xy[0], heading_xy[1], 0.0])
        right = np.array([-heading_xy[1], heading_xy[0], 0.0])
        down = np.array([0.0, 0.0, 1.0])

        slot_pos = (
            target_pos
            - self.back_m * heading
            + self.side_m * right
            + self.down_m * down
        )

        slot_vel = target_vel.copy()
        return slot_pos, slot_vel, target_pos


class ZCommandSlewLimiter:
    """Smooth outgoing Z setpoints: always-on slew, turn windows, outliers.

    The always-on path is the pos+vel yaw-spike guard: any Z command step --
    the ~30 m initial catch-up offset, the 4-5 Hz packet staircase while the
    target genuinely climbs, mode-mix shoves, extension/latch transitions --
    becomes a bounded ramp the climb controller can track without spiking
    collective (motor saturation sheds yaw authority first; 2026-07-15 logs).
    """

    def __init__(
        self,
        slew_rate_mps,
        jump_m,
        active_s,
        dmu_threshold,
        mu_threshold,
        outlier_slew_rate_mps,
        outlier_jump_m,
        always_slew_rate_mps=0.0,
    ):
        self.slew_rate_mps = max(0.0, float(slew_rate_mps))
        self.jump_m = max(0.0, float(jump_m))
        self.active_s = max(0.0, float(active_s))
        self.dmu_threshold = max(0.0, float(dmu_threshold))
        self.mu_threshold = float(mu_threshold)
        self.outlier_slew_rate_mps = max(0.0, float(outlier_slew_rate_mps))
        self.outlier_jump_m = max(0.0, float(outlier_jump_m))
        self.always_slew_rate_mps = max(0.0, float(always_slew_rate_mps))
        self.prev_mu_ct = None
        self.active_until = 0.0
        self.active_reason = ""
        self.prev_cmd_z = None

    @property
    def enabled(self):
        return self.slew_rate_mps > 0.0 and self.active_s > 0.0

    @property
    def outlier_enabled(self):
        return self.outlier_slew_rate_mps > 0.0

    @property
    def always_enabled(self):
        return self.always_slew_rate_mps > 0.0

    def open_window(self, now, reason):
        if self.enabled:
            self.active_until = max(self.active_until, now + self.active_s)
            self.active_reason = reason

    def update_mode_probability(self, mu_ct, now):
        mu_ct = float(mu_ct)
        switched = False

        if self.prev_mu_ct is not None:
            crossed_threshold = (self.prev_mu_ct < self.mu_threshold <= mu_ct) or (
                self.prev_mu_ct >= self.mu_threshold > mu_ct
            )
            jumped = abs(mu_ct - self.prev_mu_ct) >= self.dmu_threshold
            switched = crossed_threshold or jumped

            if self.enabled and switched:
                self.open_window(now, "ct_switch")

        self.prev_mu_ct = mu_ct
        return switched

    def limit(self, slot_pos, slot_vel, now, dt, seed_z=None):
        pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
        vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
        raw_z = float(pos[2])

        if self.prev_cmd_z is None:
            if seed_z is None:
                # No seed: the first command passes through unlimited.
                self.prev_cmd_z = raw_z
                return pos, vel, False, 0.0, ""
            # Seed from the pursuer's own altitude so the very first command
            # already ramps from where the vehicle IS. Commanding the initial
            # ~30 m z offset as a step kept the climb controller saturated for
            # the whole catch-up sprint (yaw shed for ~20 s; 2026-07-15 log).
            self.prev_cmd_z = float(seed_z)

        z_delta = raw_z - self.prev_cmd_z
        dt_s = max(float(dt), 1e-3)
        active = self.enabled and now <= self.active_until
        switch_limit = active and abs(z_delta) > self.jump_m
        outlier_limit = self.outlier_enabled and abs(z_delta) > self.outlier_jump_m
        always_limit = (
            self.always_enabled and abs(z_delta) > self.always_slew_rate_mps * dt_s
        )
        should_limit = switch_limit or outlier_limit or always_limit
        reason = ""

        if should_limit:
            rates = []
            reasons = []
            if switch_limit:
                rates.append(self.slew_rate_mps)
                reasons.append(self.active_reason or "turn_window")
            if outlier_limit:
                rates.append(self.outlier_slew_rate_mps)
                reasons.append("z_outlier")
            if always_limit:
                rates.append(self.always_slew_rate_mps)
                reasons.append("z_always")
            rate = min(rates)
            reason = "+".join(reasons)

            max_step = rate * dt_s
            limited_delta = float(np.clip(z_delta, -max_step, max_step))
            pos[2] = self.prev_cmd_z + limited_delta
            vel[2] = float(np.clip(vel[2], -rate, rate))

        limited_by = raw_z - float(pos[2])
        self.prev_cmd_z = float(pos[2])
        return pos, vel, should_limit, limited_by, reason


def send_local_position_target(
    master,
    boot_time,
    position,
    velocity,
    use_velocity,
    yaw=0.0,
    use_yaw=False,
    acceleration=None,
):
    ax, ay, az = 0.0, 0.0, 0.0
    if use_velocity:
        vx, vy, vz = velocity
        if acceleration is not None:
            # Acceleration feedforward only makes sense alongside velocity.
            mask = POSITION_VELOCITY_ACCEL_MASK
            ax, ay, az = (float(a) for a in acceleration)
        else:
            mask = POSITION_VELOCITY_MASK
    else:
        mask = POSITION_ONLY_MASK
        vx, vy, vz = 0.0, 0.0, 0.0

    if use_yaw:
        mask &= ~mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

    master.mav.set_position_target_local_ned_send(
        int((time.monotonic() - boot_time) * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(vx),
        float(vy),
        float(vz),
        float(ax),
        float(ay),
        float(az),
        float(yaw) if use_yaw else 0.0,
        0.0,
    )


class TerminalPositionLatch:
    def __init__(self, latch_tgo_s, release_range_m, arm_margin_m=5.0):
        self.latch_tgo_s = max(0.0, float(latch_tgo_s))
        self.release_range_m = max(0.0, float(release_range_m))
        # The latch may only arm inside the release range (with margin), or a
        # latch fired at range > release unlatches on the very next loop and
        # fires the miss-recovery BRAKE. t_go alone is not a safe trigger: a
        # relative-velocity spike (e.g. stale pursuer telemetry catching up)
        # makes tCPA dip below the threshold at 100+ m estimated range. Range
        # here is pursuer-to-estimate (imm.x), never a true target feed.
        self.arm_range_m = self.release_range_m - max(0.0, float(arm_margin_m))
        self.active = False
        self.position = None

    def reset(self):
        """Drop any frozen aim point.

        The latch is only serviced while the FSM is in CHASE, so once recovery
        takes over (CPA/abort/miss) a still-active latch would hand the next
        CHASE entry a stale frozen point from the previous pass. Recovery calls
        this on leaving CHASE so every engagement starts from a clean latch.
        """
        self.active = False
        self.position = None

    def update(self, slot_pos, t_go_s, range_m):
        if self.active and range_m >= self.release_range_m:
            self.active = False
            self.position = None

        if self.active and self.position is not None:
            return np.asarray(self.position, dtype=float).reshape(3).copy(), True, False

        if (
            self.latch_tgo_s > 0.0
            and t_go_s <= self.latch_tgo_s
            and range_m <= self.arm_range_m
        ):
            self.position = np.asarray(slot_pos, dtype=float).reshape(3).copy()
            self.active = True
            return self.position.copy(), True, True

        return np.asarray(slot_pos, dtype=float).reshape(3).copy(), False, False


class AimPointRateLimiter:
    """Hard cap on how fast the outgoing position setpoint may move.

    A final safety governor, not a shaper: legitimate aim motion is bounded by
    target speed plus terminal geometry changes (~35 m/s at these speeds).
    Prediction transients moved the commanded point 100+ m between consecutive
    loops on 2026-07-11; the vehicle chasing those steps saturated its motors
    and shed yaw authority. dt is clamped so a post-stall loop does not
    inherit a huge displacement allowance.
    """

    def __init__(self, max_speed_mps, min_dt_s=0.02, max_dt_s=0.5):
        self.max_speed = float(max_speed_mps)
        self.min_dt = float(min_dt_s)
        self.max_dt = float(max_dt_s)
        self.prev = None
        self.prev_t = None

    def limit(self, pos, now):
        pos = np.asarray(pos, dtype=float).reshape(3).copy()
        if self.max_speed <= 0.0:
            return pos, False
        if self.prev is None:
            self.prev = pos.copy()
            self.prev_t = float(now)
            return pos, False
        dt = min(max(float(now) - self.prev_t, self.min_dt), self.max_dt)
        allowance = self.max_speed * dt
        delta = pos - self.prev
        dist = float(np.linalg.norm(delta))
        limited = dist > allowance
        if limited:
            pos = self.prev + delta * (allowance / dist)
        self.prev = pos.copy()
        self.prev_t = float(now)
        return pos, limited


def clamp_command_altitude(pos, min_altitude_m):
    """Floor the commanded altitude (local NED: altitude = -z, home-relative).

    Whatever upstream produces, the vehicle must never see a setpoint near or
    below the ground (the 2026-07-11 flight commanded underground five times).
    """
    pos = np.asarray(pos, dtype=float).reshape(3).copy()
    z_max = -float(min_altitude_m)
    floored = pos[2] > z_max
    if floored:
        pos[2] = z_max
    return pos, floored


def estimate_intercept_geometry(target_pos, pursuer_pos, target_vel, pursuer_vel):
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    pursuer_pos = np.asarray(pursuer_pos, dtype=float).reshape(3)
    target_vel = np.asarray(target_vel, dtype=float).reshape(3)
    pursuer_vel = np.asarray(pursuer_vel, dtype=float).reshape(3)

    rel_pos = target_pos - pursuer_pos
    range_m = float(np.linalg.norm(rel_pos))
    if range_m <= 1e-6:
        return {
            "range_m": 0.0,
            "los_hat": np.array([1.0, 0.0, 0.0]),
            "closing_velocity": 0.0,
            "range_rate": 0.0,
            "t_go_s": 0.0,
        }

    los_hat = rel_pos / range_m
    rel_vel = target_vel - pursuer_vel
    range_rate = float(np.dot(rel_vel, los_hat))
    closing_velocity = max(0.0, -range_rate)

    # Time-to-go is the time to the closest point of approach (CPA) under a
    # constant relative-velocity model:  t_cpa = -(rel_pos . rel_vel) / |rel_vel|^2.
    # This is well conditioned. The previous range/closing_velocity form blew up
    # (hundreds of seconds, or +inf) whenever the LOS-projected closing rate
    # passed through zero -- which happens on every oblique/crossing pass even
    # when the relative speed is large -- so the terminal latch never fired on
    # exactly the geometry it exists for. CPA-in-the-past (opening) is reported
    # as +inf so the horizon/latch consumers still read it as "not closing".
    rel_speed_sq = float(np.dot(rel_vel, rel_vel))
    if rel_speed_sq <= 1e-6:
        t_go_s = float("inf")
    else:
        t_cpa = -float(np.dot(rel_pos, rel_vel)) / rel_speed_sq
        t_go_s = t_cpa if t_cpa >= 0.0 else float("inf")

    return {
        "range_m": range_m,
        "los_hat": los_hat,
        "closing_velocity": closing_velocity,
        "range_rate": range_rate,
        "t_go_s": t_go_s,
    }


def compute_guidance_prediction_horizon(intercept, prev_mu_ct_xy, mode_probs):
    t_go_s = float(intercept["t_go_s"])
    if not np.isfinite(t_go_s):
        horizon_s = float(getattr(cfg, "TERMINAL_PREDICTION_MAX_S", 6.0))
    else:
        horizon_s = max(0.0, t_go_s)

    horizon_s = min(horizon_s, float(getattr(cfg, "TERMINAL_PREDICTION_MAX_S", 6.0)))

    ct_mu = float(mode_probs.get("ct_xy", 0.0))
    prev_mu = ct_mu if prev_mu_ct_xy is None else float(prev_mu_ct_xy)
    dmu = ct_mu - prev_mu
    if (
        dmu >= float(getattr(cfg, "TERMINAL_TURN_ENTRY_DMU_MIN", 0.02))
        and ct_mu <= float(getattr(cfg, "TERMINAL_TURN_ENTRY_CT_MU_MAX", 0.55))
    ):
        horizon_s = min(
            horizon_s,
            float(getattr(cfg, "TERMINAL_TURN_ENTRY_HORIZON_CAP_S", 1.25)),
        )

    return horizon_s


def compute_terminal_extension_distance(intercept, current_speed_mps):
    min_extend_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_DISTANCE_M", 18.0))
    accel_cmss = float(getattr(cfg, "GUIDED_STARTUP_PARAM_ASSERTS", {}).get("WPNAV_ACCEL", 600))
    accel_mps2 = max(0.1, accel_cmss / 100.0)
    brake_margin_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_BRAKE_MARGIN_M", 10.0))
    brake_distance_m = (max(0.0, float(current_speed_mps)) ** 2) / (2.0 * accel_mps2)
    extend_m = max(min_extend_m, brake_distance_m + brake_margin_m)
    # Cap how far past the target the aim point is placed. The full brake
    # distance (~43 m at 20 m/s) commits the pursuer to a large overshoot on a
    # miss, which is exactly what wrecks the post-pass recovery. Capping trades a
    # little terminal deceleration for a far shorter overshoot.
    extend_max_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_MAX_M", 25.0))
    return min(extend_m, extend_max_m)


def extend_position_target_past_target(
    slot_pos,
    target_pos,
    los_hat,
    range_m,
    extend_range_m,
    extend_distance_m,
    blend_band_m=0.0,
):
    pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
    if range_m > float(extend_range_m) or float(extend_distance_m) <= 0.0:
        return pos, False
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    spear = target_pos + float(extend_distance_m) * np.asarray(
        los_hat, dtype=float
    ).reshape(3)
    band = max(0.0, float(blend_band_m))
    if band > 1e-6:
        # Blend from the untouched slot at the activation boundary to the full
        # spear point over `band` meters of closure, so activation is
        # continuous instead of an instant ~30 m aim step (the step spikes
        # tilt+collective at speed; saturation sheds yaw first). Also removes
        # on/off toggling when the range dithers across the boundary.
        s = min(1.0, (float(extend_range_m) - float(range_m)) / band)
        pos = pos + s * (spear - pos)
    else:
        pos = spear
    return pos, True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simple IMM-based GUIDED follow script"
    )
    parser.add_argument("--pursuer", default=cfg.PURSUER_CONN_STR)
    parser.add_argument("--target", default=cfg.TARGET_CONN_STR)
    parser.add_argument(
        "--loop-hz", type=float, default=min(float(getattr(cfg, "LOOP_HZ", 20)), 20.0)
    )
    parser.add_argument(
        "--back", type=float, default=float(getattr(cfg, "LAG_PURSUIT_DIST", 8.0))
    )
    parser.add_argument("--side", type=float, default=0.0)
    parser.add_argument("--down", type=float, default=0.0)
    #parser.add_argument("--predict", type=float, default=0.25)
    parser.add_argument(
        "--target-message-rate-hz",
        type=float,
        default=float(getattr(cfg, "TARGET_MESSAGE_RATE_HZ", 15.0)),
        help="Requested GLOBAL_POSITION_INT rate for the target [Hz]",
    )
    parser.add_argument("--target-timeout", type=float, default=2.0)
    parser.add_argument("--min-heading-speed", type=float, default=1.0)
    parser.add_argument(
        "--max-feedforward-speed",
        type=float,
        default=float(getattr(cfg, "SPEED_MAX", 25.0)),
    )
    parser.add_argument(
        "--terminal-extend-range-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_POSITION_EXTEND_RANGE_M", 40.0)),
        help="Inside this range, push the position target past the target along LOS [m]",
    )
    parser.add_argument(
        "--terminal-extend-distance-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_POSITION_EXTEND_DISTANCE_M", 18.0)),
        help="Distance to place the position target beyond the target along LOS [m]",
    )
    parser.add_argument(
        "--terminal-extend-blend-band-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_EXTEND_BLEND_BAND_M", 10.0)),
        help="Blend the terminal extension in over this range band below the "
        "activation range [m]; 0 = legacy instant step",
    )
    parser.add_argument(
        "--terminal-latch-tgo-s",
        type=float,
        default=float(getattr(cfg, "TERMINAL_LATCH_TGO_S", 0.7)),
        help="Freeze & spear once estimated time-to-go falls below this [s]",
    )
    parser.add_argument(
        "--terminal-latch-release-range-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_LATCH_RELEASE_RANGE_M", 40.0)),
        help="Release the frozen terminal aim point after a miss once range opens back up [m]",
    )
    parser.add_argument(
        "--z-switch-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_SLEW_RATE", 0.0)),
        help="Enable turn-window Z command slew rate limit [m/s]; 0 disables",
    )
    parser.add_argument(
        "--z-switch-jump",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_JUMP_M", 0.6)),
        help="Minimum outgoing Z jump [m] before turn-window slew limiting is applied",
    )
    parser.add_argument(
        "--z-switch-window",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_WINDOW_S", 1.2)),
        help="Seconds after CT switch or fast turn onset where Z slew limiting may apply",
    )
    parser.add_argument(
        "--z-switch-dmu",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_DMU", 0.08)),
        help="CT probability change that opens the turn-window Z slew",
    )
    parser.add_argument(
        "--z-switch-mu-threshold",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_MU_THRESHOLD", 0.20)),
        help="CT probability crossing threshold that opens the turn-window Z slew",
    )
    parser.add_argument(
        "--z-outlier-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_OUTLIER_SLEW_RATE", 0.0)),
        help="Always-on Z command outlier slew rate [m/s]; 0 disables",
    )
    parser.add_argument(
        "--z-outlier-jump",
        type=float,
        default=float(getattr(cfg, "Z_OUTLIER_JUMP_M", 0.9)),
        help="Outgoing Z jump [m] that triggers always-on outlier slew limiting",
    )
    parser.add_argument(
        "--z-always-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_ALWAYS_SLEW_RATE", 0.0)),
        help="Always-on outgoing Z command slew rate [m/s]; 0 disables. Primary "
        "pos+vel yaw-spike guard.",
    )
    parser.add_argument(
        "--z-update-freeze-packets",
        type=int,
        default=int(getattr(cfg, "Z_UPDATE_FREEZE_PACKETS", 0)),
        help="Freeze estimator Z correction for this many packets after fast turn onset; 0 disables",
    )
    parser.add_argument(
        "--z-ct-freeze-packets",
        type=int,
        default=int(getattr(cfg, "Z_CT_FREEZE_PACKETS", 0)),
        help="Freeze estimator Z correction for this many packets starting when mu_ct_xy crosses the CT activation threshold upward; 0 disables",
    )
    parser.add_argument(
        "--z-ct-freeze-mu-threshold",
        type=float,
        default=float(getattr(cfg, "Z_CT_FREEZE_MU_THRESHOLD", 0.20)),
        help="Aggregate mu_ct_xy upward crossing that arms the CT-activation Z freeze",
    )
    pos_group = parser.add_mutually_exclusive_group()
    pos_group.add_argument(
        "--position-only",
        dest="position_only",
        action="store_true",
        help="Send position-only targets (no velocity/acceleration feedforward)",
    )
    pos_group.add_argument(
        "--no-position-only",
        dest="position_only",
        action="store_false",
        help="Send velocity feedforward too (required for --accel-feedforward)",
    )
    parser.add_argument(
        "--no-guided",
        action="store_true",
        help="Do not switch the pursuer to GUIDED on startup",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable the per-loop guidance CSV log (logs/guided_follow_*.csv)",
    )

    recovery_default = bool(getattr(cfg, "MISS_RECOVERY_ENABLED", True))
    recovery_group = parser.add_mutually_exclusive_group()
    recovery_group.add_argument(
        "--miss-recovery",
        dest="miss_recovery",
        action="store_true",
        help="After a terminal miss, stabilize (recovery flight mode) then "
        "re-engage before resuming chase (needs script mode control)",
    )
    recovery_group.add_argument(
        "--no-miss-recovery",
        dest="miss_recovery",
        action="store_false",
        help="Disable miss recovery; re-chase immediately after a pass",
    )
    parser.add_argument(
        "--recovery-mode",
        default=str(getattr(cfg, "RECOVERY_MODE", "BRAKE")),
        help="Flight mode used during the recovery HOLD window (e.g. BRAKE, "
        "STABILIZE, LOITER)",
    )
    parser.add_argument(
        "--recovery-hold-s",
        type=float,
        default=float(getattr(cfg, "RECOVERY_HOLD_S", 1.2)),
        help="Seconds to hold in the recovery flight mode before re-engaging",
    )
    parser.add_argument(
        "--reengage-ramp-s",
        type=float,
        default=float(getattr(cfg, "REENGAGE_RAMP_TIME_S", 1.5)),
        help="Seconds to ramp commanded speed from SAFE_TURN_SPEED to full "
        "during re-engagement",
    )
    parser.add_argument(
        "--reengage-max-s",
        type=float,
        default=float(getattr(cfg, "REENGAGE_MAX_S", 4.0)),
        help="Timeout [s] to force re-engagement back to full chase",
    )

    yaw_default = bool(getattr(cfg, "YAW_LOCK_ENABLED", False))
    yaw_group = parser.add_mutually_exclusive_group()
    yaw_group.add_argument(
        "--yaw-lock",
        dest="yaw_lock",
        action="store_true",
        help="Enable yaw lock to line-of-sight",
    )
    yaw_group.add_argument(
        "--no-yaw-lock",
        dest="yaw_lock",
        action="store_false",
        help="Disable yaw lock to line-of-sight",
    )

    accel_ff_default = bool(getattr(cfg, "ACCEL_FEEDFORWARD_ENABLED", False))
    accel_group = parser.add_mutually_exclusive_group()
    accel_group.add_argument(
        "--accel-feedforward",
        dest="accel_feedforward",
        action="store_true",
        help="Add target acceleration feedforward to the setpoint stream "
        "(requires velocity setpoints, i.e. --no-position-only)",
    )
    accel_group.add_argument(
        "--no-accel-feedforward",
        dest="accel_feedforward",
        action="store_false",
        help="Disable target acceleration feedforward",
    )
    parser.add_argument(
        "--accel-ff-max",
        type=float,
        default=float(getattr(cfg, "ACCEL_FEEDFORWARD_MAX_MPS2", 8.0)),
        help="Clamp on the feedforward acceleration magnitude [m/s^2]",
    )
    parser.add_argument(
        "--accel-ff-fade-band",
        type=float,
        default=float(getattr(cfg, "ACCEL_FEEDFORWARD_FADE_BAND_M", 20.0)),
        help="Range band [m] above the terminal-extension range over which accel "
        "feedforward fades from full to zero (prevents terminal overshoot)",
    )

    vvel_default = bool(getattr(cfg, "VELOCITY_FF_VERTICAL_ENABLED", False))
    vvel_group = parser.add_mutually_exclusive_group()
    vvel_group.add_argument(
        "--vertical-velocity-ff",
        dest="vertical_velocity_ff",
        action="store_true",
        help="Send the estimated target vertical velocity vz in the velocity "
        "setpoint (can excite yaw instability on Z transients during turns)",
    )
    vvel_group.add_argument(
        "--no-vertical-velocity-ff",
        dest="vertical_velocity_ff",
        action="store_false",
        help="Strip vz from the velocity setpoint; the position setpoint carries "
        "Z instead (default; avoids throttle-saturation yaw instability)",
    )
    parser.set_defaults(
        yaw_lock=yaw_default,
        position_only=bool(getattr(cfg, "POSITION_ONLY_DEFAULT", True)),
        accel_feedforward=accel_ff_default,
        miss_recovery=recovery_default,
        vertical_velocity_ff=vvel_default,
    )
    return parser.parse_args()


def main():
    args = parse_args()
    running = True

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    loop_dt = 1.0 / max(args.loop_hz, 1.0)

    print(f"[{redcolor}simple_follow{endcolor}] Connecting pursuer: {args.pursuer}")
    pursuer_conn = mavutil.mavlink_connection(args.pursuer)
    wait_for_vehicle_heartbeat(
        pursuer_conn, "pursuer", abort_check=lambda: not running
    )
    print(
        f"[{redcolor}simple_follow{endcolor}] Pursuer heartbeat sys={pursuer_conn.target_system} "
        f"comp={pursuer_conn.target_component}"
    )

    home_lat, home_lon, home_alt = fetch_pursuer_home(pursuer_conn)
    print(
        f"[{redcolor}simple_follow{endcolor}] Origin/home lat={home_lat:.7f} "
        f"lon={home_lon:.7f} alt={home_alt:.1f}"
    )

    print(f"[{redcolor}simple_follow{endcolor}] Connecting target: {args.target}")
    target_conn = mavutil.mavlink_connection(args.target)
    wait_for_vehicle_heartbeat(
        target_conn,
        "target",
        expected_sysid=getattr(cfg, "TARGET_EXPECTED_SYSID", None),
        abort_check=lambda: not running,
    )
    print(
        f"[{redcolor}simple_follow{endcolor}] Target heartbeat sys={target_conn.target_system} "
        f"comp={target_conn.target_component}"
    )

    local_position_msg_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
    )
    global_position_int_msg_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
    )
    attitude_msg_id = int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ATTITUDE", 30))
    request_message_interval(pursuer_conn, local_position_msg_id, args.loop_hz)
    request_message_interval(pursuer_conn, attitude_msg_id, args.loop_hz)
    request_message_interval(
        target_conn, global_position_int_msg_id, args.target_message_rate_hz
    )

    startup_params = getattr(cfg, "GUIDED_STARTUP_PARAM_ASSERTS", {})
    if startup_params:
        assert_startup_params(pursuer_conn, startup_params)

    if not args.no_guided:
        print(f"[{redcolor}simple_follow{endcolor}] Switching pursuer to GUIDED")
        set_mode(pursuer_conn, "GUIDED")
        # Arm the shutdown hook only when we are the one managing modes.
        global _SHUTDOWN_CONN
        _SHUTDOWN_CONN = pursuer_conn

    # ATTITUDE rides along for its yawspeed: the recovery machine uses the yaw
    # rate to decide whether the vehicle has stopped spinning before it hands
    # control back to GUIDED. LOCAL_POSITION_NED stays the primary message.
    pursuer_reader = mavlink_utils.MavStateReader(
        pursuer_conn,
        ["LOCAL_POSITION_NED", "ATTITUDE"],
        mavlink_utils.parse_local_ned,
    )
    target_reader = mavlink_utils.MavStateReader(
        target_conn,
        "GLOBAL_POSITION_INT",
        lambda msg: mavlink_utils.parse_global_int(msg, home_lat, home_lon, home_alt),
    )

    pursuer_reader.start()
    target_reader.start()

    print(
        f"[{redcolor}simple_follow{endcolor}] Waiting for first pursuer and target positions"
    )
    while running:
        pursuer_pos, _ = pursuer_reader.get()
        target_pos, _, target_stamp, _ = target_reader.get_with_times()
        if pursuer_pos is not None and target_pos is not None and target_stamp > 0.0:
            break
        time.sleep(0.05)

    if not running:
        return

    imm = setup_imm_filter(loop_dt)
    turn_rate_estimator = HeadingTurnRateEstimator()
    first_update = True
    last_target_stamp = None
    last_good_target_wall_time = time.monotonic()
    last_command_print = 0.0
    last_raw_turn_strength = 0.0
    last_fast_turn_onset = False
    last_yaw_cmd = 0.0
    z_update_freeze_remaining = 0
    z_update_freeze_active = False
    prev_mu_ct_xy = None
    mu_ct_xy_now = 0.0
    # Estimator diagnostics for the GUI (persist across non-packet loops).
    last_innov_norm = float("nan")
    last_jump_norm = float("nan")
    last_err_norm = float("nan")
    last_omega = 0.0
    boot_time = time.monotonic()
    guidance = BehindSlotGuidance(
        args.back, args.side, args.down, args.min_heading_speed
    )
    z_slew = ZCommandSlewLimiter(
        args.z_switch_slew_rate,
        args.z_switch_jump,
        args.z_switch_window,
        args.z_switch_dmu,
        args.z_switch_mu_threshold,
        args.z_outlier_slew_rate,
        args.z_outlier_jump,
        always_slew_rate_mps=args.z_always_slew_rate,
    )
    guidance_lpf = IMMLowPassFilter()
    terminal_latch = TerminalPositionLatch(
        args.terminal_latch_tgo_s,
        args.terminal_latch_release_range_m,
    )
    lead_cache = LeadPredictionCache(
        max_substep=float(getattr(cfg, "GUIDANCE_LEAD_PREDICT_SUBSTEP_S", 0.25))
    )
    aim_limiter = AimPointRateLimiter(
        float(getattr(cfg, "AIM_POINT_MAX_SPEED_MPS", 60.0))
    )
    min_command_alt_m = float(getattr(cfg, "MIN_COMMAND_ALTITUDE_M", 15.0))
    aim_lpf_tau = float(getattr(cfg, "LEAD_AIM_LPF_TAU_S", 0.0))
    aim_lpf_state = None  # low-pass state for the lead position (jitter smoother)
    pursuer_stale_timeout_s = float(getattr(cfg, "PURSUER_STALE_TIMEOUT_S", 1.0))

    guidance_logger = None if args.no_log else GuidanceLogger()
    if guidance_logger is not None:
        print(
            f"[{redcolor}simple_follow{endcolor}] Guidance log: {guidance_logger.path}"
        )

    recovery = MissRecoveryController(args)
    commanded_mode = "GUIDED"  # startup set it (unless --no-guided)
    if args.miss_recovery and args.no_guided:
        print(
            f"[{redcolor}simple_follow{endcolor}] miss recovery disabled "
            f"(needs script mode control; incompatible with --no-guided)"
        )
    elif recovery.enabled:
        print(
            f"[{redcolor}simple_follow{endcolor}] miss recovery ON "
            f"(hold={recovery.hold_s:.1f}s mode={recovery.recovery_mode} "
            f"reengage_ramp={recovery.ramp_s:.1f}s)"
        )

    if not args.position_only:
        print(
            f"[{redcolor}simple_follow{endcolor}] velocity feedforward ON "
            f"(vertical vz {'sent' if args.vertical_velocity_ff else 'stripped'})"
        )

    print(
        f"[{redcolor}simple_follow{endcolor}] Running "
        f"back={args.back:.1f}m side={args.side:.1f}m down={args.down:.1f}m "
        f"loop={args.loop_hz:.1f}Hz yaw_lock={int(args.yaw_lock)} "
        f"vff_z={int(not args.position_only and args.vertical_velocity_ff)}"
    )

    # --- Start GUI ---
    print(f"[{redcolor}simple_follow{endcolor}] Starting guidance GUI ...")
    gui = GuidanceGUI(param_module="guidance_config", refresh_hz=10)
    gui.start()

    while running:
        loop_start = time.monotonic()
        pursuer_pos, pursuer_vel, _pursuer_stamp, pursuer_wall_stamp = (
            pursuer_reader.get_with_times()
        )
        target_meas, _, target_stamp, target_wall_stamp = target_reader.get_with_times()

        if pursuer_pos is None or target_meas is None or target_stamp <= 0.0:
            time.sleep(0.05)
            continue

        pursuer_pos_np = np.asarray(pursuer_pos, dtype=float).reshape(3)
        pursuer_vel_np = (
            np.asarray(pursuer_vel, dtype=float).reshape(3)
            if pursuer_vel is not None
            else np.zeros(3)
        )

        imm_updated_this_loop = False
        if last_target_stamp is None or target_stamp > last_target_stamp:
            imm_updated_this_loop = True
            z_meas = np.asarray(target_meas, dtype=float).reshape(3)
            omega_hint = turn_rate_estimator.update(z_meas, target_stamp)
            if first_update:
                for filt in imm.filters:
                    filt.x[0:3] = z_meas
                imm.x = imm.filters[0].x.copy()
                stabilize_omega_states(imm)
                first_update = False
            else:
                assert last_target_stamp is not None
                last_target_stamp_value = float(last_target_stamp)
                dt_meas = clamp_filter_dt(target_stamp - last_target_stamp_value)
                last_raw_turn_strength = apply_fast_turn_onset_hint(
                    imm,
                    turn_rate_estimator.raw_omega,
                    turn_rate_estimator.speed_xy,
                )
                was_fast_turn_onset = last_fast_turn_onset
                last_fast_turn_onset = last_raw_turn_strength > 0.0
                if last_fast_turn_onset:
                    z_slew.open_window(loop_start, "fast_turn")
                if last_fast_turn_onset and not was_fast_turn_onset:
                    z_update_freeze_remaining = max(
                        z_update_freeze_remaining,
                        max(0, int(args.z_update_freeze_packets)),
                    )
                apply_turn_rate_hint(imm, omega_hint)
                predict_imm_over_dt(imm, dt_meas)
                # Predicted position before the measurement update: innovation is
                # meas - pred, the update jump is post - pred. These feed the GUI
                # residual-norm plot (same diagnostics filterwndr's own view shows).
                pred_pos_before_update = np.asarray(imm.x[0:3], dtype=float).copy()
                z_update_freeze_active = z_update_freeze_remaining > 0
                if z_update_freeze_active:
                    update_imm_preserving_vertical(imm, z_meas)
                    z_update_freeze_remaining -= 1
                else:
                    # Snapshot vertical state before the update so we can
                    # retroactively freeze Z if this packet is the one that
                    # pushes mu_ct_xy across the activation threshold.
                    ct_freeze_threshold = float(args.z_ct_freeze_mu_threshold)
                    ct_freeze_packets = max(0, int(args.z_ct_freeze_packets))
                    prev_mu = prev_mu_ct_xy
                    snapshot_before = (
                        snapshot_imm_vertical_state(imm)
                        if ct_freeze_packets > 0 and prev_mu is not None
                        else None
                    )
                    imm.update(z_meas)
                    mu_ct_xy_now = ct_mode_probability(imm)
                    if (
                        snapshot_before is not None
                        and prev_mu is not None
                        and prev_mu < ct_freeze_threshold <= mu_ct_xy_now
                    ):
                        # This packet activated CT. Retroactively freeze Z
                        # for this packet and arm the freeze for N-1 more.
                        restore_imm_vertical_state(imm, snapshot_before)
                        if ct_freeze_packets > 1:
                            z_update_freeze_remaining = max(
                                z_update_freeze_remaining,
                                ct_freeze_packets - 1,
                            )
                    prev_mu_ct_xy = mu_ct_xy_now
                apply_turn_rate_hint(imm, omega_hint)
                stabilize_omega_states(imm)

                post_pos = np.asarray(imm.x[0:3], dtype=float)
                last_innov_norm = float(
                    np.linalg.norm(z_meas - pred_pos_before_update)
                )
                last_jump_norm = float(
                    np.linalg.norm(post_pos - pred_pos_before_update)
                )
                last_err_norm = float(np.linalg.norm(post_pos - z_meas))
                last_omega = float(imm.x[9])

            last_target_stamp = target_stamp
            last_good_target_wall_time = target_wall_stamp

        target_age = loop_start - last_good_target_wall_time
        if target_age > args.target_timeout:
            if loop_start - last_command_print > 1.0:
                print(
                    f"[{redcolor}simple_follow{endcolor}] Target stale for {target_age:.1f}s, holding previous GUIDED target"
                )
                last_command_print = loop_start
            # Re-anchor the aim smoother on reacquisition rather than crawling
            # from a stale anchor toward a target that moved during the gap.
            aim_lpf_state = None
            time.sleep(min(loop_dt, 0.1))
            continue

        # Own-position staleness guard (2026-07-24 review): the target feed has
        # always had this; the pursuer did not. A frozen own position feeds
        # phantom range/t_go/CPA and defeats the altitude abort (alt_error uses
        # the frozen z), so hold setpoints and warn, exactly like target-stale.
        # The estimator keeps absorbing target packets above; only guidance and
        # decisions are gated.
        pursuer_age = loop_start - pursuer_wall_stamp
        if pursuer_age > pursuer_stale_timeout_s:
            if loop_start - last_command_print > 1.0:
                print(
                    f"[{redcolor}simple_follow{endcolor}] PURSUER position stale "
                    f"for {pursuer_age:.1f}s, holding previous GUIDED target"
                )
                last_command_print = loop_start
            aim_lpf_state = None
            time.sleep(min(loop_dt, 0.1))
            continue

        mode_probs = aggregate_mode_probabilities(imm)
        # Engagement geometry vs the target ESTIMATE (imm.x) -- never the raw
        # target feed (that only exists in the test rig; in deployment the
        # target is observed through the estimator, not telemetry). Everything
        # that decides -- the terminal latch, the extension activation, the
        # miss-recovery FSM -- and everything logged keys off this. The lead
        # prediction below is only where the vehicle is AIMED. (2026-07-11:
        # keying decisions off the lead point let prediction transients fire
        # BRAKE at 100+ m estimated range and made the logged range fictional.)
        intercept = estimate_intercept_geometry(
            np.asarray(imm.x[0:3], dtype=float).reshape(3),
            pursuer_pos_np,
            np.asarray(imm.x[3:6], dtype=float).reshape(3),
            pursuer_vel_np,
        )
        guidance_horizon_s = compute_guidance_prediction_horizon(
            intercept,
            prev_mu_ct_xy,
            mode_probs,
        )
        # Envelope-clamped lead (position+velocity+turn rate; linear accel and
        # CT centripetal bounded to the flight envelope) plus a covariance gate
        # that shrinks the lead toward the current estimate when the filter's
        # predicted position sigma is large. Together these replace the old hard
        # vertical pin: the vertical accel/vz are dropped by config (level
        # target) and the horizontal lead self-limits when tracking is poor.
        x_pred, p_pred = lead_cache.get(
            imm, guidance_horizon_s, imm_updated_this_loop
        )
        predicted_state, lead_k_h, lead_k_v = lead_cov_gate(
            x_pred, p_pred, imm.x
        )
        # Low-pass the lead position AND velocity at loop rate (time-constant
        # filter, runs every loop incl. between packets). This is the primary
        # jitter smoother: the aim is a projection of the noisy velocity
        # heading, so it dithers packet-to-packet; a rate cap bounds its speed
        # but not the direction reversals that shed copter yaw, whereas this
        # bounds both. Smoothing the velocity channels too matters -- the
        # behind-slot offset rotates with the velocity heading, so a jittery
        # heading re-injects setpoint whip (raw heading measured 1084 deg/s of
        # commanded turn rate, LPF'd heading 125 deg/s; replay 2026-07-13). The
        # velocity FF and behind-slot heading both read the smoothed value;
        # terminal geometry keys off imm.x (raw), so it is unaffected.
        if aim_lpf_tau > 1e-6:
            alpha = loop_dt / (loop_dt + aim_lpf_tau)
            lead_pv = np.asarray(predicted_state[0:6], dtype=float)
            if aim_lpf_state is None:
                aim_lpf_state = lead_pv.copy()
            else:
                aim_lpf_state = aim_lpf_state + alpha * (lead_pv - aim_lpf_state)
            predicted_state = np.asarray(predicted_state, dtype=float).copy()
            predicted_state[0:6] = aim_lpf_state
        filtered_state = guidance_lpf.filter(predicted_state, dt=loop_dt)
        slot_pos, slot_vel, target_pos = guidance.update(filtered_state, pursuer_pos_np)
        # Nominal aim altitude BEFORE the terminal spear/latch mutate slot_pos.
        # The altitude abort must judge "can the vehicle hold its commanded
        # altitude" against this, not the speared z: the LOS spear deliberately
        # offsets slot z by up to ~29 m in vertical-displacement geometry, which
        # read as phantom tracking failure (2026-07-24 review).
        aim_z_nominal = float(slot_pos[2])
        slot_vel = clamp_norm(slot_vel, args.max_feedforward_speed)
        # Strip the estimated vertical velocity from the velocity feedforward: the
        # smooth position setpoint carries Z instead. CT/CA turns inject Z transients
        # into vz, and feeding that to the copter spikes collective throttle ->
        # motor saturation -> yaw authority shed first -> physical yaw instability
        # (only in pos+vel, never pos-only). The target flies ~level, so no real
        # vertical motion is lost. --vertical-velocity-ff restores the raw vz.
        if not args.position_only and not args.vertical_velocity_ff:
            slot_vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
            slot_vel[2] = 0.0
        # The aggressive terminal dive (extension + freeze latch) only runs in
        # the CHASE state. After a miss the recovery machine takes over.
        extended_terminal = False
        terminal_latched = False
        terminal_latch_armed = False
        terminal_extend_distance_m = 0.0
        if recovery.state == MissRecoveryController.CHASE:
            terminal_extend_distance_m = compute_terminal_extension_distance(
                intercept,
                float(np.linalg.norm(pursuer_vel_np)),
            )
            # Spear THROUGH the lead point (where the intercept is expected),
            # but activate on the TRUE range to the target. As t_go shrinks
            # the lead collapses onto the target, so the two frames converge
            # exactly when it matters.
            aim_rel = np.asarray(target_pos, dtype=float).reshape(3) - pursuer_pos_np
            aim_dist = float(np.linalg.norm(aim_rel))
            aim_los_hat = (
                aim_rel / aim_dist if aim_dist > 1e-6 else intercept["los_hat"]
            )
            slot_pos, extended_terminal = extend_position_target_past_target(
                slot_pos,
                target_pos,
                aim_los_hat,
                intercept["range_m"],
                args.terminal_extend_range_m,
                terminal_extend_distance_m,
                blend_band_m=args.terminal_extend_blend_band_m,
            )
            slot_pos, terminal_latched, terminal_latch_armed = terminal_latch.update(
                slot_pos,
                intercept["t_go_s"],
                intercept["range_m"],
            )
            # Once the aim is FROZEN, drop the velocity feedforward with it. A
            # fixed position setpoint plus ~20 m/s of target-velocity FF is a
            # contradiction, and the moment the pursuer overshoots that point
            # the position error reverses while the FF still pushes forward ->
            # the controller fights itself, pitches back at high thrust and
            # balloons (log 110805). Zeroing it makes the latched phase a clean
            # "fly through to this point and stop", which is what the extension
            # already intends.
            if terminal_latched and bool(
                getattr(cfg, "TERMINAL_LATCH_ZERO_VELOCITY_FF", True)
            ):
                slot_vel = np.zeros(3, dtype=float)

        # Altitude tracking error: how far the vehicle is from the altitude it
        # is being commanded to hold. Judged against the NOMINAL aim altitude
        # (pre-spear/latch), so the deliberate terminal LOS offset never counts
        # as tracking failure. A large value means the copter cannot hold
        # altitude (saturation from the aggressive horizontal chase) and is
        # falling/ballooning -- the abort trigger inside recovery.update.
        alt_error = abs(float(pursuer_pos_np[2]) - aim_z_nominal)
        pursuer_yaw_rate = pursuer_reader.get_yaw_rate()
        # Time-consistent range sample for the CPA trigger, ONLY on loops where
        # the estimate is packet-fresh, with the pursuer dead-reckoned to now
        # via its own velocity. The naive every-loop range is a +-5-8 m
        # staircase (estimate frozen between target packets, own position ~3 Hz)
        # that fired CPA before the true pass (2026-07-24 review).
        cpa_range_m = None
        if imm_updated_this_loop:
            p_age = min(max(loop_start - pursuer_wall_stamp, 0.0), 0.6)
            pursuer_dr = pursuer_pos_np + pursuer_vel_np * p_age
            cpa_range_m = float(
                np.linalg.norm(
                    np.asarray(imm.x[0:3], dtype=float).reshape(3) - pursuer_dr
                )
            )
        directive = recovery.update(
            loop_start,
            terminal_latched,
            intercept["closing_velocity"],
            alt_error=alt_error,
            yaw_rate=pursuer_yaw_rate,
            range_m=cpa_range_m,
        )
        recovery_state = directive["state"]
        # Leaving CHASE means recovery owns the vehicle: drop the frozen aim so
        # the next CHASE entry cannot inherit a stale latch from this pass.
        if recovery_state != MissRecoveryController.CHASE:
            terminal_latch.reset()
        elif directive.get("unlatch"):
            # CPA pass confirmed but HOLD suppressed by brake spacing: release
            # the frozen aim and keep chasing live geometry (never balloon
            # against an overshot point). This loop still flew the frozen aim
            # (and is logged as latched); the release takes effect next loop.
            terminal_latch.reset()

        # Apply the requested flight mode (only when it changes).
        if recovery.enabled and directive["mode"] and directive["mode"] != commanded_mode:
            try:
                set_mode(pursuer_conn, directive["mode"])
                commanded_mode = directive["mode"]
                print(
                    f"[{redcolor}simple_follow{endcolor}] recovery -> {recovery_state} "
                    f"(mode {commanded_mode})"
                )
            except Exception as exc:
                print(
                    f"[{redcolor}simple_follow{endcolor}] recovery set_mode "
                    f"{directive['mode']!r} failed: {exc}"
                )

        # Diagnostics defaults for loops where we do not send a setpoint (HOLD).
        z_switch = False
        z_limited = False
        z_limited_by = 0.0
        z_limit_reason = "hold" if not directive["send"] else ""
        aim_limited = False
        alt_floored = False
        accel_ff = None
        if not directive["send"]:
            # Re-seed the Z command from the vehicle's own altitude when the
            # send path resumes, instead of ramping from the stale pre-HOLD
            # commanded z (the seed only applies while prev_cmd_z is None).
            z_slew.prev_cmd_z = None

        if directive["send"]:
            if recovery_state == MissRecoveryController.REENGAGE:
                # Re-approach the ACTUAL target estimate (not the lead
                # prediction, which whips during turns -- chasing it from a
                # bad post-miss attitude is what diverged before), no
                # extension/latch, speed limited and ramping up, so a
                # bad-attitude pursuer eases back in instead of instantly
                # re-committing at full speed.
                slot_pos = np.asarray(imm.x[0:3], dtype=float).reshape(3).copy()
                slot_vel = clamp_norm(slot_vel, directive["speed_cap"])

            z_switch = z_slew.update_mode_probability(
                ct_mode_probability(imm), loop_start
            )
            slot_pos, slot_vel, z_limited, z_limited_by, z_limit_reason = z_slew.limit(
                slot_pos,
                slot_vel,
                loop_start,
                loop_dt,
                seed_z=float(pursuer_pos_np[2]),
            )

            # Final output governors: a hard cap on how fast the commanded
            # point may move, then an altitude floor. Whatever upstream logic
            # produces, the vehicle never sees a teleporting or underground
            # setpoint.
            slot_pos, aim_limited = aim_limiter.limit(slot_pos, loop_start)
            slot_pos, alt_floored = clamp_command_altitude(
                slot_pos, min_command_alt_m
            )

            if args.yaw_lock:
                desired_yaw = yaw_to_los(
                    target_pos,
                    pursuer_pos_np,
                    last_yaw_cmd,
                    float(getattr(cfg, "YAW_LOCK_MIN_RANGE_M", 10.0)),
                )
                last_yaw_cmd = slew_angle_toward(
                    last_yaw_cmd,
                    desired_yaw,
                    float(getattr(cfg, "YAW_LOCK_MAX_RATE_DEG_S", 90.0)),
                    loop_dt,
                )

            # Acceleration feedforward: midcourse lead aid, CHASE only. Faded to
            # zero as the pursuer enters the terminal-extension band (adding
            # target accel there causes blow-through overshoot) and suppressed
            # once the terminal latch has frozen the aim.
            if (
                args.accel_feedforward
                and not args.position_only
                and recovery_state == MissRecoveryController.CHASE
                and not terminal_latched
            ):
                accel_scale = accel_ff_range_scale(
                    intercept["range_m"],
                    args.terminal_extend_range_m,
                    args.accel_ff_fade_band,
                )
                if accel_scale > 0.0:
                    accel_ff = accel_scale * target_feedforward_acceleration(
                        filtered_state, args.accel_ff_max
                    )

            send_local_position_target(
                pursuer_conn,
                boot_time,
                slot_pos,
                slot_vel,
                use_velocity=not args.position_only,
                yaw=last_yaw_cmd,
                use_yaw=args.yaw_lock,
                acceleration=accel_ff,
            )

        if guidance_logger is not None:
            aff = np.zeros(3) if accel_ff is None else np.asarray(accel_ff, dtype=float)
            est_now = np.asarray(imm.x[0:3], dtype=float)
            meas_now = np.asarray(target_meas, dtype=float).reshape(3)
            guidance_logger.write(
                {
                    "wall_time": loop_start,
                    "range_m": intercept["range_m"],
                    "closing_velocity": intercept["closing_velocity"],
                    "t_go_s": intercept["t_go_s"],
                    "guidance_horizon_s": guidance_horizon_s,
                    "pursuer_x": pursuer_pos_np[0],
                    "pursuer_y": pursuer_pos_np[1],
                    "pursuer_z": pursuer_pos_np[2],
                    "meas_x": meas_now[0],
                    "meas_y": meas_now[1],
                    "meas_z": meas_now[2],
                    "est_x": est_now[0],
                    "est_y": est_now[1],
                    "est_z": est_now[2],
                    "slot_x": slot_pos[0],
                    "slot_y": slot_pos[1],
                    "slot_z": slot_pos[2],
                    "slot_vx": slot_vel[0],
                    "slot_vy": slot_vel[1],
                    "slot_vz": slot_vel[2],
                    "aff_x": aff[0],
                    "aff_y": aff[1],
                    "aff_z": aff[2],
                    "aff_mag": float(np.linalg.norm(aff)),
                    "extended": int(extended_terminal),
                    "latched": int(terminal_latched),
                    "latch_armed": int(terminal_latch_armed),
                    "recovery_state": recovery_state,
                    "z_limited": int(z_limited),
                    "aim_limited": int(aim_limited),
                    "alt_floored": int(alt_floored),
                    "lead_k_h": lead_k_h,
                    "lead_k_v": lead_k_v,
                    "mu_ct": mode_probs["ct_xy"],
                    "yaw_rate_dps": (
                        "" if pursuer_yaw_rate is None
                        else float(np.degrees(pursuer_yaw_rate))
                    ),
                    "alt_err_m": alt_error,
                    "cpa_range_m": "" if cpa_range_m is None else cpa_range_m,
                }
            )

        if loop_start - last_command_print > 0.5:
            range_to_target = float(intercept["range_m"])
            range_to_slot = float(np.linalg.norm(slot_pos - pursuer_pos_np))
            tgo_val = float(intercept["t_go_s"])
            tgo_str = f"{tgo_val:5.2f}s" if np.isfinite(tgo_val) else "  inf"
            print(
                f"[{redcolor}simple_follow{endcolor}] range_target={bluecolor}{range_to_target:6.1f}m{endcolor} "
                f"range_slot={range_to_slot:6.1f}m "
                f"slot=({slot_pos[0]:.1f},{slot_pos[1]:.1f},{slot_pos[2]:.1f}) "
                f"tv={np.linalg.norm(filtered_state[3:6]):.1f}m/s "
                f"tgo={bluecolor}{tgo_str}{endcolor} vc={intercept['closing_velocity']:.1f}m/s "
                f"aff={0.0 if accel_ff is None else float(np.linalg.norm(accel_ff)):.1f}m/s2 "
                f"rcv={recovery_state} "
                f"pred={guidance_horizon_s:.2f}s ext={int(extended_terminal)} ext_m={terminal_extend_distance_m:.1f} "
                f"latch={int(terminal_latched)} arm={int(terminal_latch_armed)} "
                f"mu_xy=({mode_probs['cv_xy']:.2f},{mode_probs['ct_xy']:.2f},{mode_probs['ca_xy']:.2f}) "
                f"mu_z=({mode_probs['cv_z']:.2f},{mode_probs['ca_z']:.2f}) "
                f"raw_turn={turn_rate_estimator.raw_omega:+.3f}/{last_raw_turn_strength:.2f} "
                f"yaw_lock={int(args.yaw_lock)} "
                f"z_freeze={int(z_update_freeze_active)}:{z_update_freeze_remaining} "
                f"z_ct={mode_probs['ct_xy']:+.2f} "
                f"z_switch={int(z_switch)} z_fast={int(last_fast_turn_onset)} "
                f"z_slew={z_limited_by:+.2f}:{z_limit_reason or '-'} "
                f"gov=aim:{int(aim_limited)},floor:{int(alt_floored)} "
                f"leadk=({lead_k_h:.2f},{lead_k_v:.2f})"
            )
            last_command_print = loop_start

        elapsed = time.monotonic() - loop_start
        if elapsed < loop_dt:
            time.sleep(loop_dt - elapsed)

        # Push telemetry to GUI using this loop's aligned snapshot: target_meas
        # is the measurement imm.x was updated with, so est - meas is the true
        # filter residual (cm). Re-reading the target here would catch a newer
        # background-thread packet than imm.x reflects, injecting ~1 packet of
        # target motion (several metres) of phantom high-frequency plot error.
        push_snapshot(
            pursuer_pos=pursuer_pos_np,
            target_pos=np.asarray(target_meas, dtype=float).reshape(3),
            mode_probs=mode_probs,
            target_est=np.array(imm.x[0:3]),
            status={
                "range_m": float(intercept["range_m"]),
                "t_go_s": float(intercept["t_go_s"]),
                "closing_velocity": float(intercept["closing_velocity"]),
                "recovery_state": recovery_state,
                "extended": bool(extended_terminal),
                "latched": bool(terminal_latched),
            },
            diag={
                "omega": last_omega,
                "innov_norm": last_innov_norm,
                "jump_norm": last_jump_norm,
                "err_norm": last_err_norm,
            },
        )
        gui_tick()

    # Hand the vehicle to a self-stabilising mode before we stop commanding it.
    stabilize_on_exit("loop ended")
    gui.stop()
    if guidance_logger is not None:
        guidance_logger.close()
        print(f"[{redcolor}simple_follow{endcolor}] Guidance log written: {guidance_logger.path}")
    print(f"[{redcolor}simple_follow{endcolor}] Stopped")


if __name__ == "__main__":
    try:
        main()
    finally:
        # Backstop: covers an exception or a Ctrl-C caught outside the loop.
        # No-op if the normal path already ran it.
        stabilize_on_exit("interpreter exit")
