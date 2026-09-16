#!/usr/bin/env python3
"""Gimbal compatible, accessible LOS / PN terminal guide.

Positional guidance establishes the slot behind the target. This controller only works after the
video handoff and does the remaining work for that geometry:

* SETTLE: Shrinks the ratio LOS with the lateral acceleration PN. * HIT: When LOS becomes stable
enough, it increases the closing speed. * DON: returns the last accessible collision command when
the remaining time is shorter than the actuator delay; It does not optimize a horizon past the
target and produce a reverse command.

The only telemetry size used from the target is ``Measurement.range_m_value``. Target speed, heading, and
acceleration are not used. Other inputs are the camera LOS and the vehicle's own status.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from visual_base import (VisualLoop, VisualController, Command,
                             body_forward_ned)
from los_guidance import DerivativeFilter, eps_solve, clamp, wrap180


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    c = float(np.dot(a, b)) / (na * nb)
    return math.degrees(math.acos(clamp(c, -1.0, 1.0)))


def _horizontal_cone(v_now: np.ndarray, v_request: np.ndarray,
                maximum_angle_deg: float) -> np.ndarray:
    """Projects the desired horizontal direction to the safe cone around the current speed."""
    v = np.asarray(v_now, float).copy()
    u = np.asarray(v_request, float).copy()
    hv, hu = v[:2], u[:2]
    nv, nu = float(np.linalg.norm(hv)), float(np.linalg.norm(hu))
    if nv < 1.0 or nu < 1e-9:
        return u
    a0, a1 = math.atan2(hv[1], hv[0]), math.atan2(hu[1], hu[0])
    difference = math.radians(wrap180(math.degrees(a1 - a0)))
    limit_value = math.radians(float(maximum_angle_deg))
    if abs(difference) <= limit_value:
        return u
    a = a0 + clamp(difference, -limit_value, limit_value)
    u[:2] = nu * np.array([math.cos(a), math.sin(a)])
    return u


class TerminalLosController(VisualController):
    """PN + closing thrust + accessible command cone."""

    label_item = "terminal_los"

    def __init__(
            self,
            n_pn=4.0,
            pn_closure_floor_mps=10.0,
            a_horizontal_max_mps2=5.0,
            a_vertical_max_mps2=1.5,
            a_jerk_z_mps3=4.0,
            command_horizon_s=0.70,
            command_angle_max_deg=45.0,
            v_max_mps=35.0,
            settle_acceleration_mps2=0.5,
            strike_acceleration_mps2=4.0,
            closure_target_mps=7.0,
            closure_kp=0.35,
            settle_los_dps=4.0,
            settle_ex_deg=12.0,
            settle_duration_s=0.45,
            strike_mandatory_range_m=18.0,
            strike_mandatory_los_dps=7.0,
            strike_closure_min_mps=1.0,
            strike_output_los_dps=11.0,
            terminal_range_m=3.0,
            terminal_tgo_s=0.25,
            terminal_horizontal_correction_mps2=1.0,
            vertical_tau_s=1.4,
            vertical_speed_tau_s=0.8,
            vertical_ac_m=1.2,
            vertical_close_m=0.6,
            vertical_terminal_range_m=8.0,
            climb_speed_max_mps=2.5,
            descent_speed_max_mps=2.0,
            tau_derivative_s=0.22,
            tau_range_s=0.40,
            yaw_kp=0.70,
            yaw_kd=0.85,
            yaw_rate_max_dps=75.0,
            yaw_dead_band_deg=0.7,
            yaw_command_provide=True,
            aim_deg=0.0,
            min_altitude_m_value=15.0,
            miss_arm_m=20.0,
            miss_opening_m=8.0,
            miss_opening_rate_mps=2.0,
            miss_confirmation_loop=3):
        self.n_pn = float(n_pn)
        self.pn_closure_floor = float(pn_closure_floor_mps)
        self.a_horizontal_max = float(a_horizontal_max_mps2)
        self.a_vertical_max = float(a_vertical_max_mps2)
        self.a_jerk_z = float(a_jerk_z_mps3)
        self.command_horizon = float(command_horizon_s)
        self.command_angle_max = float(command_angle_max_deg)
        self.v_max = float(v_max_mps)
        self.settle_acceleration = float(settle_acceleration_mps2)
        self.strike_acceleration = float(strike_acceleration_mps2)
        self.closure_target = float(closure_target_mps)
        self.closure_kp = float(closure_kp)
        self.settle_los = float(settle_los_dps)
        self.settle_ex = float(settle_ex_deg)
        self.settle_duration = float(settle_duration_s)
        self.strike_mandatory_range = float(strike_mandatory_range_m)
        self.strike_mandatory_los = float(strike_mandatory_los_dps)
        self.strike_closure_min = float(strike_closure_min_mps)
        self.strike_output_los = float(strike_output_los_dps)
        self.terminal_range = float(terminal_range_m)
        self.terminal_tgo = float(terminal_tgo_s)
        self.terminal_horizontal_correction = float(terminal_horizontal_correction_mps2)
        self.vertical_tau = float(vertical_tau_s)
        self.vertical_speed_tau = float(vertical_speed_tau_s)
        self.vertical_ac = float(vertical_ac_m)
        self.vertical_close = float(vertical_close_m)
        self.vertical_terminal_range = float(vertical_terminal_range_m)
        self.climb_speed_max = float(climb_speed_max_mps)
        self.descent_speed_max = float(descent_speed_max_mps)
        self.tau_derivative = float(tau_derivative_s)
        self.tau_range = float(tau_range_s)
        self.yaw_kp = float(yaw_kp)
        self.yaw_kd = float(yaw_kd)
        self.yaw_rate_max = float(yaw_rate_max_dps)
        self.yaw_dead_band = float(yaw_dead_band_deg)
        self.yaw_command_provide = bool(yaw_command_provide)
        self.aim_deg = float(aim_deg)
        self.min_altitude = float(min_altitude_m_value)
        self.miss_arm = float(miss_arm_m)
        self.miss_opening = float(miss_opening_m)
        self.miss_opening_rate = float(miss_opening_rate_mps)
        self.miss_confirmation_loop = int(miss_confirmation_loop)
        self.diagnostic = {}
        self.seed_value2(None)

    def seed_value2(self, handoff):
        self.d_ex = DerivativeFilter(self.tau_derivative)
        self.d_eps = DerivativeFilter(self.tau_derivative)
        self.d_yaw = DerivativeFilter(self.tau_derivative)
        self.d_range = DerivativeFilter(self.tau_range)
        self.d_rz = DerivativeFilter(0.35)
        self.phase_value = "SETTLE"
        self._phase_previous = self.phase_value
        self._settle_counter_s = 0.0
        self._signature = None
        self._t_last_new = None
        self._t_previous = None
        self._range_last = None
        self._vertical_active = False
        self._a_z = 0.0
        self._frozen_v = None
        self._impact_event = False
        self._en_good_r = float("inf")
        self._miss_counter = 0
        self._handoff_v = None
        if handoff and handoff.get("cmd_vel_ned") is not None:
            try:
                self._handoff_v = np.asarray(handoff["cmd_vel_ned"], float)
            except (TypeError, ValueError):
                self._handoff_v = None
        if handoff and handoff.get("range_m") is not None:
            try:
                self._range_last = float(handoff["range_m"])
            except (TypeError, ValueError):
                pass

    def _reachable(self, v_now, v_raw):
        """Apply the acceleration envelope, directional cone and speed ceiling in order."""
        v = np.asarray(v_now, float).reshape(3)
        u = _horizontal_cone(v, np.asarray(v_raw, float).reshape(3),
                        self.command_angle_max)
        dv = u - v
        dv_ceiling = self.a_horizontal_max * self.command_horizon
        nd = float(np.linalg.norm(dv))
        if nd > dv_ceiling:
            u = v + dv * (dv_ceiling / nd)
        n = float(np.linalg.norm(u))
        if n > self.v_max:
            u *= self.v_max / n
        # The vertical channel cannot break the horizontal collision law and subsequent positional recovery.
        # In NED negative=ascent, positive=descend.
        u[2] = clamp(float(u[2]), -self.climb_speed_max,
                       self.descent_speed_max)
        # Numerical last defense: never reverse hemisphere command to a moving vehicle.
        if float(np.linalg.norm(v)) > 2.0 and float(np.dot(u, v)) < 0.0:
            u = v.copy()
        return u

    def _vertical_acceleration(self, r, eps_deg, vz, dt, terminal):
        """Relative vertical position/derivative relative tracking acceleration [NED]."""
        r_z = -float(r) * math.sin(math.radians(float(eps_deg)))
        r_z_point = self.d_rz.update_value(r_z, dt)
        if abs(r_z) >= self.vertical_ac:
            self._vertical_active = True
        elif abs(r_z) <= self.vertical_close:
            self._vertical_active = False

        if not self._vertical_active or terminal:
            raw_value = 0.0
        else:
            # r_z = z_target-z_own. r_z_dot = vz_target-vz_own. vz_request = vz_target + r_z/tau -> error =
            # r_z_dot+r_z/tau.
            speed_error = r_z_point + r_z / max(self.vertical_tau, 1e-3)
            raw_value = clamp(speed_error / max(self.vertical_speed_tau, 1e-3),
                          -self.a_vertical_max, self.a_vertical_max)
        step_value = self.a_jerk_z * dt
        self._a_z += clamp(raw_value - self._a_z, -step_value, step_value)
        return self._a_z, r_z, r_z_point

    def command_value(self, o) -> Command:
        dt = clamp(float(o.dt), 1e-3, 0.30)
        ex = 0.0 if o.ex_deg is None else float(o.ex_deg)
        ey = 0.0 if o.ey_deg is None else float(o.ey_deg)
        eps = clamp(eps_solve(ex, ey, self.aim_deg), -80.0, 80.0)

        yaw_deg = None if o.yaw_rad is None else math.degrees(o.yaw_rad)
        yaw_point = 0.0
        if yaw_deg is not None:
            if self.d_yaw.x_previous is not None:
                yaw_deg = self.d_yaw.x_previous + wrap180(
                    yaw_deg - self.d_yaw.x_previous)
            yaw_point = self.d_yaw.update_value(yaw_deg, dt)

        signature_value = o.t_capture if o.t_capture is not None else (
            o.ex_deg, o.ey_deg, o.bbox_w, o.bbox_h)
        new_value = signature_value != self._signature
        self._signature = signature_value
        t = float(o.t)
        dt_measurement = dt if self._t_last_new is None else clamp(
            t - self._t_last_new, 1e-3, 0.50)
        if self._t_previous is not None and t - self._t_previous > 0.7:
            self.d_ex.reset_value()
            self.d_eps.reset_value()
            self.d_range.reset_value()
            self.d_rz.reset_value()
            dt_measurement = dt
        self._t_previous = t

        if o.range_m_value is not None and math.isfinite(float(o.range_m_value)):
            r = clamp(float(o.range_m_value), 0.5, 500.0)
            self._range_last = r
        else:
            r = self._range_last if self._range_last is not None else 40.0

        if new_value:
            d_ex = self.d_ex.update_value(ex, dt_measurement)
            d_eps = self.d_eps.update_value(eps, dt_measurement)
            d_r = self.d_range.update_value(r, dt_measurement)
            self._t_last_new = t
        else:
            d_ex, d_eps, d_r = self.d_ex.d, self.d_eps.d, self.d_range.d
        q_az = d_ex + yaw_point
        closure = max(-d_r, 0.0)
        tgo = r / closure if closure > 0.5 else None

        v_ned = (np.asarray(o.vel_ned, float).reshape(3) if o.vel_ned is not None
                 else (self._handoff_v.copy() if self._handoff_v is not None
                       else np.zeros(3)))
        yaw = 0.0 if o.yaw_rad is None else float(o.yaw_rad)
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_h = np.array([cy * v_ned[0] + sy * v_ned[1],
                        -sy * v_ned[0] + cy * v_ned[1], v_ned[2]])

        # Phase machine. STRIKE can revert back to SETTLE in a hard corner; FROST latch.
        terminal = (r <= self.terminal_range
                    or (tgo is not None and tgo <= self.terminal_tgo))
        if terminal:
            self.phase_value = "DON"
        elif self.phase_value == "STRIKE" and (abs(q_az) > self.strike_output_los
                                     or (d_r > 0.0
                                         and r > self.strike_mandatory_range)):
            self.phase_value = "SETTLE"
            self._settle_counter_s = 0.0
        elif self.phase_value == "SETTLE":
            suitable = (abs(q_az) <= self.settle_los
                     and abs(ex) <= self.settle_ex
                     and closure >= self.strike_closure_min)
            self._settle_counter_s = (self._settle_counter_s + dt
                                      if suitable else max(0.0,
                                                        self._settle_counter_s-dt))
            if (self._settle_counter_s >= self.settle_duration
                    or (r <= self.strike_mandatory_range
                        and abs(q_az) <= self.strike_mandatory_los
                        and closure >= self.strike_closure_min)):
                self.phase_value = "STRIKE"

        # After the first pass, don't turn around with the camera and make a second head-on attack. It is the
        # task of the positioned layer to rebuild the background.
        self._en_good_r = min(self._en_good_r, r)
        miss_condition = (self._en_good_r <= self.miss_arm
                       and r >= self._en_good_r + self.miss_opening
                       and d_r >= self.miss_opening_rate)
        self._miss_counter = self._miss_counter + 1 if miss_condition else 0
        if self._miss_counter >= self.miss_confirmation_loop:
            reason_value = (f"opening after transition: best_candidate = {self._en_good_r:.1f} m "
                     f"now={r:.1f}m dr={d_r:+.1f}mps")
            return Command(vel_ned=v_ned.copy(), yaw_rate_dps=None,
                         release_value=True, release_reason=reason_value,
                         event_value="miss_release", event_detail=reason_value)

        los_c = math.radians(ex)
        u_los = np.array([math.cos(los_c), math.sin(los_c)])
        u_right = np.array([-math.sin(los_c), math.cos(los_c)])
        vc = max(closure, self.pn_closure_floor)
        a_lateral = self.n_pn * vc * math.radians(q_az)
        lat_ceiling = (self.terminal_horizontal_correction if self.phase_value == "DON"
                     else self.a_horizontal_max)
        a_lateral = clamp(a_lateral, -lat_ceiling, lat_ceiling)
        if self.phase_value == "SETTLE":
            a_forward = self.settle_acceleration
        elif self.phase_value == "STRIKE":
            a_forward = self.strike_acceleration + self.closure_kp * (
                self.closure_target - closure)
            a_forward = clamp(a_forward, 0.0, self.a_horizontal_max)
        else:
            a_forward = 0.0
        # In case of large direction error, giving gas increases the turning radius.
        if abs(q_az) > self.strike_output_los:
            a_forward = 0.0
        a_h = a_forward * u_los + a_lateral * u_right
        na = float(np.linalg.norm(a_h))
        if na > self.a_horizontal_max:
            a_h *= self.a_horizontal_max / na

        a_z, r_z, r_z_point = self._vertical_acceleration(
            r, eps, float(v_h[2]), dt_measurement if new_value else dt,
            self.phase_value == "DON" or r <= self.vertical_terminal_range)
        v_raw_h = v_h.copy()
        v_raw_h[:2] += self.command_horizon * a_h
        v_raw_h[2] += self.command_horizon * a_z
        v_raw = body_forward_ned(yaw, v_raw_h[0], v_raw_h[1], v_raw_h[2])

        if self.phase_value == "DON":
            if self._frozen_v is None:
                self._frozen_v = self._reachable(v_ned, v_raw)
            v_request = self._frozen_v.copy()
        else:
            self._frozen_v = None
            v_request = v_raw
        v_cmd = self._reachable(v_ned, v_request)
        if o.pos_ned is not None and -float(o.pos_ned[2]) < self.min_altitude:
            v_cmd[2] = min(v_cmd[2], 0.0)

        e_yaw = 0.0 if abs(ex) <= self.yaw_dead_band else (
            ex - math.copysign(self.yaw_dead_band, ex))
        yaw_rate = clamp(self.yaw_kp * e_yaw + self.yaw_kd * d_ex,
                           -self.yaw_rate_max, self.yaw_rate_max)

        event_value = ""
        detail = ""
        if self.phase_value != self._phase_previous:
            event_value = f"phase_{self.phase_value.lower()}"
            detail = (f"{self._phase_previous}->{self.phase_value} r={r:.1f}m "
                     f"qaz= {q_az:.1f} dps shutdown= {closure:.1f} mps")
            self._phase_previous = self.phase_value
        if (not self._impact_event and o.vibe_max is not None
                and float(o.vibe_max) > 10.0 and r < 3.0):
            event_value, detail = "impact_successful", f"vibe={o.vibe_max:.1f} r={r:.2f}m"
            self._impact_event = True

        self.diagnostic = {
            "phase_value": self.phase_value, "r": r, "d_r": d_r, "closure": closure,
            "tgo": tgo, "ex": ex, "eps": eps, "d_ex": d_ex,
            "d_eps": d_eps, "yaw_point": yaw_point, "q_az": q_az,
            "a_forward": a_forward, "a_lateral": a_lateral, "a_z": a_z,
            "r_z": r_z, "r_z_point": r_z_point,
            "cmd_real_angle": _angle_deg(v_cmd, v_ned),
            "cmd_real_dv": float(np.linalg.norm(v_cmd-v_ned)),
        }
        return Command(vel_ned=v_cmd,
                     yaw_rate_dps=(yaw_rate if self.yaw_command_provide else None),
                     event_value=event_value, event_detail=detail)


def arg_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration-value", type=float, default=None)
    p.add_argument("--loop-hz", type=float, default=20.0)
    p.add_argument("--tau", type=float, default=0.20,
                   help="common instruction LPF time constant [s]")
    p.add_argument("--log", default=None)
    p.add_argument("--n-pn", type=float, default=4.0)
    p.add_argument("--strike-acceleration", type=float, default=4.0)
    p.add_argument("--command-horizon", type=float, default=0.70)
    p.add_argument("--terminal-range", type=float, default=3.0)
    p.add_argument("--terminal-tgo", type=float, default=0.25)
    p.add_argument("--v-max", type=float, default=35.0)
    p.add_argument("--no-yaw", action="store_true",
                   help="yaw-rate sending; lateral LOS command again speed "
                        "setpointiyle roll/pitch uretir")
    return p


def main():
    a = arg_parser().parse_args()
    k = TerminalLosController(n_pn=a.n_pn, strike_acceleration_mps2=a.strike_acceleration,
                             command_horizon_s=a.command_horizon,
                             terminal_range_m=a.terminal_range,
                             terminal_tgo_s=a.terminal_tgo,
                             yaw_command_provide=not a.no_yaw,
                             v_max_mps=a.v_max)
    print("[terminal_los] SETTLE/STRIKE/DON "
          f"N={k.n_pn:.1f} a_strike={k.strike_acceleration:.1f}m/s2 "
          f"availability={k.a_horizontal_max:.1f}m/s2 x {k.command_horizon:.2f}s "
          f"direction_cone=+/-{k.command_angle_max:.0f}deg "
          f"terminal={k.terminal_range:.1f}m/{k.terminal_tgo:.2f}s "
          f"yaw={'enabled_value' if k.yaw_command_provide else 'otopilotta'}",
          flush=True)
    VisualLoop(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_path=a.log).run_value(a.duration_value)


if __name__ == "__main__":
    main()
