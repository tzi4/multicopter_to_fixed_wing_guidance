#!/usr/bin/env python3
"""Unlocked unit and closed-loop tests for the terminal_los_guidance."""

from __future__ import annotations

import math
import unittest

import numpy as np

from visual_base import Measurement
from terminal_los_guidance import TerminalLosController, _angle_deg


DT = 0.05


def measurement(t, ex=0.0, ey=0.0, range_value=25.0, vel=(20.0, 0.0, 0.0),
          yaw=0.0, pos=(0.0, 0.0, -60.0), vibe=None, tilt=0.0,
          bbox=(50.0, 20.0)):
    bw, bh = bbox
    return Measurement(
        t=t, dt=DT, ex_deg=ex, ey_deg=ey, bbox_w=bw, bbox_h=bh,
        area_root=math.sqrt(bw*bh), coverage_pct=100.0*bw/1280.0,
        bbox_age_s=0.02,
        range_m_value=range_value, pos_ned=np.asarray(pos, float),
        vel_ned=np.asarray(vel, float), yaw_rad=yaw, roll_rad=0.0,
        pitch_rad=0.0, t_capture=t, vibe_max=vibe, tilt_deg=tilt)


class Unit(unittest.TestCase):
    def new_value(self, **kw):
        k = TerminalLosController(**kw)
        k.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        return k

    def test_reachability_delta_v_and_direction_cone(self):
        k = self.new_value()
        v = np.array([20.0, 0.0, 0.0])
        for raw_value in ([-35, 0, 0], [0, 35, 0], [0, -35, 10], [80, 80, -20]):
            u = k._reachable(v, np.asarray(raw_value, float))
            self.assertLessEqual(np.linalg.norm(u-v),
                                 k.a_horizontal_max*k.command_horizon+1e-9)
            self.assertLessEqual(_angle_deg(u[:2], v[:2]),
                                 k.command_angle_max+1e-9)
            self.assertGreaterEqual(float(np.dot(u, v)), 0.0)
            self.assertGreaterEqual(u[2], -k.climb_speed_max-1e-9)
            self.assertLessEqual(u[2], k.descent_speed_max+1e-9)

    def test_settle_after_strike(self):
        k = self.new_value()
        for i in range(18):
            cmd = k.command_value(measurement(i*DT, range_value=25.0-0.1*i))
        self.assertEqual(k.phase_value, "STRIKE")
        self.assertGreater(k.diagnostic["a_forward"], 2.0)
        self.assertGreater(cmd.vel_ned[0], 20.0)

    def test_hard_los_at_rate_throttle_cuts_settles(self):
        k = self.new_value()
        for i in range(18):
            k.command_value(measurement(i*DT, range_value=25.0-0.1*i))
        self.assertEqual(k.phase_value, "STRIKE")
        for j in range(1, 10):
            k.command_value(measurement((18+j)*DT, ex=2.0*j, range_value=23.1))
        self.assertEqual(k.phase_value, "SETTLE")
        self.assertEqual(k.diagnostic["a_forward"], 0.0)

    def test_at_terminal_reverse_command_absent(self):
        k = self.new_value()
        cmd = k.command_value(measurement(0.0, ex=28.0, range_value=2.8,
                            vel=(26.0, 2.0, 0.0)))
        self.assertEqual(k.phase_value, "DON")
        self.assertGreater(float(np.dot(cmd.vel_ned, np.array([26., 2., 0.]))),
                           0.0)
        self.assertLessEqual(k.diagnostic["cmd_real_angle"], 45.0+1e-9)

    def test_vertical_channel_sign_and_terminal_damping(self):
        k = self.new_value()
        # Target above: ey negative -> eps positive -> NED vz decreases.
        for i in range(20):
            cmd = k.command_value(measurement(i*DT, ey=-8.0, range_value=20.0))
        self.assertLess(cmd.vel_ned[2], 0.0)
        # At the vertical terminal gate, the acceleration returns to zero with the jerk limit.
        previous_value = abs(k.diagnostic["a_z"])
        for j in range(20):
            k.command_value(measurement((20+j)*DT, ey=-8.0, range_value=7.0))
        self.assertLess(abs(k.diagnostic["a_z"]), previous_value)

    def test_live_tilt_guidance_geometry_does_not_offset(self):
        # Stabilized ey is already world LOS. Same ey, different camera tilt should give the same command;
        # Tilt is for framing/diagnostic reference only.
        a, b = self.new_value(), self.new_value()
        ca = a.command_value(measurement(0.0, ex=3.0, ey=-5.0, tilt=-20.0))
        cb = b.command_value(measurement(0.0, ex=3.0, ey=-5.0, tilt=+20.0))
        np.testing.assert_allclose(ca.vel_ned, cb.vel_ned, atol=1e-12)

    def test_yaw_when_disabled_also_lateral_los_command_remains(self):
        k = self.new_value(yaw_command_provide=False)
        cmd = k.command_value(measurement(0.0, ex=12.0, range_value=20.0))
        self.assertIsNone(cmd.yaw_rate_dps)
        self.assertNotEqual(cmd.vel_ned[1], 0.0)

    def test_hybrid_transition_latched(self):
        from hybrid_guidance import HybridController
        h = HybridController(transition_range_m=18.0)
        h.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        cmd = h.command_value(measurement(0.0, range_value=17.9))
        self.assertEqual(h.phase_value, "LOS")
        self.assertEqual(cmd.event_value, "hybrid_los_transition")
        h.command_value(measurement(DT, range_value=22.0))
        self.assertEqual(h.phase_value, "LOS")

    def test_hybrid_mpc_output_also_reachable(self):
        from hybrid_guidance import HybridController
        h = HybridController(transition_range_m=18.0)
        h.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 30.0})
        o = measurement(0.0, ex=18.0, ey=-8.0, range_value=30.0,
                  vel=(20.0, 0.0, 0.0))
        cmd = h.command_value(o)
        self.assertEqual(h.phase_value, "MPC")
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned-o.vel_ned),
                             h.terminal.a_horizontal_max*h.terminal.command_horizon+1e-9)
        self.assertLessEqual(_angle_deg(cmd.vel_ned[:2], o.vel_ned[:2]), 45.0)
        self.assertGreaterEqual(cmd.vel_ned[2], -h.terminal.climb_speed_max)

    def test_hybrid_default_range_transition_exactly_preserves(self):
        from hybrid_guidance import HybridController
        h = HybridController(transition_range_m=18.0)
        h.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 30.0})
        # Although the visual door is more than ready, the default source is RANGE.
        for i in range(10):
            h.command_value(measurement(i*DT, range_value=25.0, bbox=(80.0, 32.0)))
        self.assertEqual(h.phase_value, "MPC")
        h.command_value(measurement(10*DT, range_value=17.9, bbox=(20.0, 8.0)))
        self.assertEqual(h.phase_value, "LOS")

    def test_hybrid_visual_transition_from_range_independent(self):
        from hybrid_guidance import HybridController
        h = HybridController(transition_source="visual", visual_dwell_s=0.30)
        h.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 40.0})
        # 70x23 -> normalized square root area ~% 4.18 ; at the center gate.
        for i in range(5):
            h.command_value(measurement(i*DT, ex=1.0, ey=-10.0, range_value=40.0,
                          bbox=(70.0, 23.0)))
        self.assertEqual(h.phase_value, "MPC")
        cmd = h.command_value(measurement(5*DT, ex=1.0, ey=-10.0, range_value=40.0,
                            bbox=(70.0, 23.0)))
        self.assertEqual(h.phase_value, "LOS")
        self.assertIn("source=visual", cmd.event_detail)

    def test_hybrid_visual_transition_small_or_at_edge_rejected(self):
        from hybrid_guidance import HybridController
        h = HybridController(transition_source="visual")
        h.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 10.0})
        for i in range(10):
            h.command_value(measurement(i*DT, ex=9.0, ey=0.0, range_value=10.0,
                          bbox=(70.0, 23.0)))
        self.assertEqual(h.phase_value, "MPC")
        for i in range(10, 20):
            h.command_value(measurement(i*DT, ex=0.0, ey=0.0, range_value=10.0,
                          bbox=(30.0, 10.0)))
        self.assertEqual(h.phase_value, "MPC")

    def test_increasing_at_range_strike_absent_and_miss_released(self):
        k = self.new_value()
        t = 0.0
        # First, close and arm the MISS door.
        for r in (25, 23, 20, 17, 14, 10, 7):
            cmd = k.command_value(measurement(t, range_value=r)); t += DT
        # Then distinct and constantly urgent.
        for r in (9, 12, 16, 18, 20):
            for _ in range(4):
                cmd = k.command_value(measurement(t, range_value=r)); t += DT
        self.assertNotEqual(k.phase_value, "STRIKE")
        self.assertTrue(cmd.release_value)
        self.assertIn("after transition", cmd.release_reason)


class ClosedLoop(unittest.TestCase):
    def run_value2(self, target_turn_dps):
        k = TerminalLosController()
        k.seed_value2({"cmd_vel_ned": [20.0, 0.0, 0.0], "range_m": 25.0})
        p = np.array([0.0, 0.0, -60.0])
        v = np.array([20.0, 0.0, 0.0])
        h = np.array([25.0, 0.0, -60.0])
        yaw = 0.0
        nearest_distance = 1e9
        max_angle = 0.0
        for i in range(500):
            t = i*DT
            route = math.radians(target_turn_dps)*max(t-1.0, 0.0)
            vh = np.array([20.0*math.cos(route), 20.0*math.sin(route), 0.0])
            rel = h-p
            r = float(np.linalg.norm(rel))
            nearest_distance = min(nearest_distance, r)
            bearing_value = math.atan2(rel[1], rel[0])
            ex = (math.degrees(bearing_value-yaw)+180.0) % 360.0-180.0
            eps = math.degrees(math.atan2(-rel[2],
                                          max(np.linalg.norm(rel[:2]), 1e-6)))
            cmd = k.command_value(measurement(t, ex=ex, ey=-eps, range_value=r, vel=v,
                                yaw=yaw, pos=p))
            max_angle = max(max_angle, _angle_deg(cmd.vel_ned, v))
            # Simple ArduPilot surrogate: 0.45 s velocity response, 5 m/s2 horizontal ceiling.
            acc = (cmd.vel_ned-v)/0.45
            na = float(np.linalg.norm(acc[:2]))
            if na > 5.0:
                acc[:2] *= 5.0/na
            acc[2] = np.clip(acc[2], -5.0, 5.0)
            v += acc*DT
            p += v*DT
            h += vh*DT
            yaw += math.radians(cmd.yaw_rate_dps or 0.0)*DT
        return nearest_distance, max_angle

    def test_straight_at_tail_collision(self):
        nearest_distance, angle_value = self.run_value2(0.0)
        self.assertLess(nearest_distance, 1.0)
        self.assertLessEqual(angle_value, 45.0+1e-9)

    def test_ellipse_similar_hard_in_turn_collision_cone(self):
        nearest_distance, angle_value = self.run_value2(8.0)
        self.assertLess(nearest_distance, 2.0)
        self.assertLessEqual(angle_value, 45.0+1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
