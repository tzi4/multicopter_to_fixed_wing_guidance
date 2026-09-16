#!/usr/bin/env python3
"""
pid_test.py - OFFLINE verification for pid_guidance.PidController
============================================== DOES NOT REQUIRE SIM, Gazebo, SITL, Redis or MAVLink.
Generates Synthetic Measurement arrays and tests the controller's
pointing/direction/limit/anti-windup behavior.

    python3 pid_test.py            # run them all
    python3 pid_test.py -v         # ayrinti

WHY THESE TESTS: the most expensive bugs in this project were pointing errors (reversing the
target), constant-dt assumptions (giant circles with loop falling on 2 Hz), and integrator bloat.
The following tests target exactly these three classes.
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from visual_base import Measurement                       # noqa: E402
from pid_guidance import PidConfig, PidController, DerivativeFilterValue   # noqa: E402


# ------------------------------------------------------------------ yardimci

def measurement_perform(t=0.0, dt=0.05, ex=0.0, ey=0.0, range_value=30.0, area_root=25.0,
              yaw=0.0, altitude_value=50.0, vel=(18.0, 0.0, 0.0), pitch=0.0):
    """A single synthetic Measurement package. altitude: meters relative to home (NED z = -h).

    area_root DEFAULT 25 px: median of m band 25-35 in running 2 calibration (26). So the default
    measurement is "at terminal band, good quality" and the quality layer (q=1) is moved out of the
    way of the tests. The quality behavior is also tested in our own tests by giving small area_root."""
    return Measurement(
        t=t, dt=dt,
        ex_deg=ex, ey_deg=ey,
        bbox_w=30.0, bbox_h=10.0,
        area_root=area_root, coverage_pct=2.5, bbox_age_s=0.03,
        range_m_value=range_value,
        pos_ned=np.array([0.0, 0.0, -float(altitude_value)]),
        vel_ned=np.asarray(vel, dtype=float),
        yaw_rad=yaw, roll_rad=0.0, pitch_rad=pitch,
    )


def run_value3(k, steps_item, dt=0.05, t0=0.0, **fixed):
    """steps: (ex, ey) pairs or measurement generator. Returns the last request."""
    last_value = None
    t = t0
    for ex, ey in steps_item:
        last_value = k.body_request(measurement_perform(t=t, dt=dt, ex=ex, ey=ey, **fixed))
        t += dt
    return last_value


class PidTest(unittest.TestCase):

    def new_value(self, **override):
        # MOUNT FIXED TO 30 (2026-08-04): sim default moved to 0 degree (pitch-servo gimbal decision) and
        # camera_mounting_deg is now read from $YILDIZ_MOUNT. The numerical expectations of this package
        # (ey_target = -(assembly+pitch)) belong to the +30 geometry, on which PID was DEVELOPED AND VERIFIED.
        # PID frozen benchmark artifact; Instead of basing the test on the new geometry, we write ON the
        # geometry to which it is valid.
        override.setdefault('camera_mounting_deg', 30.0)
        return PidController(PidConfig(**override))

    # --1. SIGN VERIFICATION ------------------------------------------------

    def test_ex_pozitif_right_gider(self):
        """ex>0 (target RIGHT) -> yaw_rate>0 AND right lateral velocity>0."""
        k = self.new_value()
        s = run_value3(k, [(10.0, 0.0)] * 5)
        self.assertGreater(s['yaw_rate_dps'], 0.0)
        self.assertGreater(s['right_value'], 0.0)

    def test_ex_negative_to_left_gider(self):
        k = self.new_value()
        s = run_value3(k, [(-10.0, 0.0)] * 5)
        self.assertLess(s['yaw_rate_dps'], 0.0)
        self.assertLess(s['right_value'], 0.0)

    def test_ey_pozitif_descends(self):
        """ey>0 = target BELOW center (with aim=0: BELOW from us) -> descend."""
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        s = run_value3(k, [(0.0, 8.0)] * 5)
        self.assertGreater(s['down_value'], 0.0)

    def test_ey_negative_climbs(self):
        """ey<0 = target UP -> climb (down speed negative)."""
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        s = run_value3(k, [(0.0, -8.0)] * 5)
        self.assertLess(s['down_value'], 0.0)

    def test_ned_donusumu_yaw_with(self):
        """yaw=90 is deg (nose EAST) while forward command should be +y (east) in NED."""
        k = self.new_value()
        c = None
        for i in range(5):
            c = k.command_value(measurement_perform(t=0.05 * i, ex=0.0, ey=0.0,
                                  yaw=math.radians(90.0)))
        self.assertGreater(c.vel_ned[1], 10.0)
        self.assertLess(abs(c.vel_ned[0]), 1e-6)

    def test_atalet_los_rate(self):
        """The term PN should use INERTIA LOS speed: in fixed rotation ex constant (ex_dot~0) but yaw rotates
-> lateral command MUST NOT be ZERO.

        This is a regression test of the error caught in the dry run: Since ex is relative to the
        nose, the cycle yaw would reset it and the term PN would be."""
        k = self.new_value()
        t, yaw = 0.0, 0.0
        last_value = None
        for _ in range(60):                       # 3 s, 8 deg/s return
            t += 0.05
            yaw += math.radians(8.0) * 0.05
            last_value = k.body_request(measurement_perform(t=t, ex=1.0, yaw=yaw, range_value=30.0))
        self.assertAlmostEqual(k.diagnostic['ex_dot'], 0.0, delta=0.5)
        self.assertAlmostEqual(k.diagnostic['lambda_dot'], 8.0, delta=1.0)
        self.assertGreater(k.diagnostic['lateral_d'], 3.0)   # PN terimi LIVE
        self.assertGreater(last_value['right_value'], 3.0)

    def test_lambda_dot_clamp(self):
        """The ATTITUDE jump (step yaw) should not explode the lateral command."""
        k = self.new_value()
        k.body_request(measurement_perform(t=0.0, yaw=0.0))
        k.body_request(measurement_perform(t=0.05, yaw=math.radians(90.0)))
        self.assertLessEqual(abs(k.diagnostic['lambda_dot']),
                             k.a.lambda_dot_max + 1e-6)

    def test_forward_axis_los_azimuth(self):
        """forward axis should be yaw+ex: yaw=0, ex=+30 -> NED y component is positive and large (sin(30) of
the forward speed up), not just from the lateral term."""
        k = self.new_value(lateral_max=0.0)      # close lateral channel -> pure axis test
        k.command_value(measurement_perform(t=0.0, ex=30.0, yaw=0.0))
        c = k.command_value(measurement_perform(t=0.05, ex=30.0, yaw=0.0))
        ratio_value = c.vel_ned[1] / max(1e-6, np.linalg.norm(c.vel_ned))
        self.assertAlmostEqual(ratio_value, math.sin(math.radians(30.0)), delta=0.03)

    def test_body_forward_ablation(self):
        """--govde-ileri: forward axis must be NOSE (ex no rotation)."""
        k = self.new_value(body_forward=True, lateral_max=0.0)
        k.command_value(measurement_perform(t=0.0, ex=30.0, yaw=0.0))
        c = k.command_value(measurement_perform(t=0.05, ex=30.0, yaw=0.0))
        self.assertLess(abs(c.vel_ned[1]), 0.2)

    # --2. RANGE TIMING ---------------------------------------------

    def test_gain_with_range_increases(self):
        """Same angular error, twice the range -> ~twice the lateral speed request. (angle -> meter conversion:
offset = R*tan(e))"""
        s20 = run_value3(self.new_value(), [(6.0, 0.0)] * 3, range_value=20.0)
        s40 = run_value3(self.new_value(), [(6.0, 0.0)] * 3, range_value=40.0)
        self.assertGreater(s40['right_value'], 1.8 * s20['right_value'])
        self.assertLess(s40['right_value'], 2.2 * s20['right_value'])

    def test_range_clamp(self):
        """Must not blow out broken/excessive range gain (clamp 12..80 m)."""
        s = run_value3(self.new_value(), [(6.0, 0.0)] * 3, range_value=5000.0)
        s80 = run_value3(self.new_value(), [(6.0, 0.0)] * 3, range_value=80.0)
        self.assertAlmostEqual(s['right_value'], s80['right_value'], delta=1e-6)

    def test_range_if_absent_default(self):
        """range None -> default used, no crash."""
        k = self.new_value()
        s = k.body_request(measurement_perform(range_value=None))
        self.assertTrue(math.isfinite(s['forward']))

    def test_vertical_setpoint_camera_axis_dependent(self):
        """Setpoint is not fixed relative to the HORIZON, but relative to the CAMERA AXIS: axis = mount(30) +
body pitch. Full axis far away, fractional axis nearby."""
        k = self.new_value()
        # pitch = 0 -> axis 30 deg
        self.assertAlmostEqual(k._ey_target(80.0, 0.0), -30.0, delta=1e-6)
        self.assertAlmostEqual(k._ey_target(12.0, 0.0),
                               -30.0 * k.a.framing_weight_near, delta=1e-6)
        # AS pitch INCREASES, the axis shifts up -> the setpoint also becomes larger (more negative).
        self.assertLess(k._ey_target(80.0, 6.8), k._ey_target(80.0, 0.0))
        self.assertAlmostEqual(k._ey_target(80.0, 6.8), -36.8, delta=1e-6)

    def test_straight_route_drag_descent_commands(self):
        """REGRESSION (yaw-lock run geometry): unable to hold standoff positioned on the straight route, 41.8
drifts to m, vertical offset 13 m remains constant -> ascent 17.3 deg, pitch +6.8 -> camera axis
+36.8, i.e. BELOW the target axis 19.5 deg (frame edge 20.1).

        THE CORRECT MOVEMENT IS TO DESCEND (increase depth -> increase elevation -> bring the target
        to the center of the frame). The old version commanded CLIMB here."""
        k = self.new_value()
        s = run_value3(k, [(0.0, -17.3)] * 20, range_value=41.8, area_root=9.0,
                   altitude_value=60.0, pitch=math.radians(6.8))
        self.assertGreater(s['down_value'], 0.5)          # ALCALIYOR
        self.assertGreater(k.diagnostic['ey_target'], -36.9)
        self.assertLess(k.diagnostic['ey_target'], -25.0)   # setpoint close to axis

    def test_depth_ceiling_unintended_descent_cuts(self):
        """Maintaining a constant angle while extending the range would increase the depth; ceiling
        (max_depth_m) must stop this."""
        k = self.new_value()
        # 70 m range, 30 deg elevation -> depth 35 m > 25 m ceiling
        s = run_value3(k, [(0.0, -30.0)] * 20, range_value=70.0, area_root=20.0,
                   altitude_value=80.0)
        self.assertTrue(k.diagnostic['depth_exceeded'])
        self.assertLessEqual(s['down_value'], 0.0)        # the descent has ceased

    def test_at_handoff_framing_center_approaches(self):
        """handoff typical (range 50, ey -25, pitch 0 -> axis +30 deg): BELOW the target axis 5 deg. The
correct move is to bring it to the axis, i.e. SLIGHT Descent (depth increases -> ascent becomes
larger). The command should not be saturated.

        General variant: the vertical command should be in the direction that MINIMIZES the target's
        angle relative to the camera axis -- this is what keeps the detection going."""
        for ey, pitch_d, expected_descent in ((-25.0, 0.0, True),
                                              (-17.3, 6.8, True),
                                              (-40.0, 0.0, False)):
            k = self.new_value()
            s = run_value3(k, [(0.0, ey)] * 6, range_value=50.0,
                       pitch=math.radians(pitch_d))
            axis_value = 30.0 + pitch_d
            framing = -ey - axis_value          # <0: below target axis
            if expected_descent:
                self.assertLess(framing, 0.0)
                self.assertGreater(s['down_value'], 0.0)   # lower -> rise here you go
            else:
                self.assertGreater(framing, 0.0)
                self.assertLess(s['down_value'], 0.0)      # climb -> rise lower
            self.assertLess(abs(s['down_value']), k.a.climb_max)   # not saturated

    # -- 3. DERIVATIVE / KOSE KESME (PN terimi) ---------------------------------

    def test_derivative_lead_adds(self):
        """The growing ex (LOS is spinning) should require MORE lateral velocity than the stationary ex."""
        fixed = run_value3(self.new_value(), [(6.0, 0.0)] * 20)
        ramp_value = run_value3(self.new_value(), [(0.3 * i, 0.0) for i in range(1, 21)])
        # the final error of the ramp is the same as 6.0; the difference is entirely the D (+I) term
        self.assertGreater(ramp_value['right_value'], fixed['right_value'])

    def test_derivative_initial_in_frame_zero(self):
        """In the first example the derivative should be 0 (inheritance lesson: 1127 - 2817 deg /s
pseudoderivatives)."""
        d = DerivativeFilterValue(0.15)
        self.assertEqual(d.update_value(25.0, 0.0, 0.05), 0.0)

    def test_derivative_zoh_repetition_robust(self):
        """When the same bbox is read twice, the derivative is NOT refreshed (comb noise)."""
        d = DerivativeFilterValue(0.15)
        d.update_value(0.0, 0.00, 0.05)
        v1 = d.update_value(1.0, 0.05, 0.05)
        v2 = d.update_value(1.0, 0.10, 0.05)   # same value -> keep
        self.assertAlmostEqual(v1, v2, delta=1e-9)
        self.assertGreater(v1, 0.0)

    # --4. MEASURED DT (giant circle root cause) ------------------------------

    def test_dt_independence(self):
        """If the SAME time function 5 Hz and 30 Hz are instantiated the command should exit close. This test
would EXPLODE if there was a constant-dt assumption."""
        def run_value2(hz):
            k = self.new_value()
            dt = 1.0 / hz
            n = int(round(2.0 * hz))
            t = 0.0
            last_value = None
            for i in range(1, n + 1):
                t = i * dt
                ex = 4.0 * t          # fixed speed LOS: 4 deg/s
                last_value = k.body_request(measurement_perform(t=t, dt=dt, ex=ex))
            return last_value
        a, b = run_value2(5.0), run_value2(30.0)
        self.assertAlmostEqual(a['right_value'], b['right_value'], delta=0.15 * abs(b['right_value']) + 0.3)
        self.assertAlmostEqual(a['yaw_rate_dps'], b['yaw_rate_dps'],
                               delta=0.15 * abs(b['yaw_rate_dps']) + 1.0)

    # --5. LIMITS / SPEED BUDGET -----------------------------------------

    def test_speed_ceiling_not_exceeded(self):
        """In a large survey of measurements |v| <= ceiling (frame clamp must not engage at all)."""
        k = self.new_value()
        ceiling_value = k.a.v_forward_ceiling
        rng = np.random.default_rng(7)
        t = 0.0
        for _ in range(600):
            t += 0.05
            c = k.command_value(measurement_perform(
                t=t, ex=float(rng.uniform(-40, 40)),
                ey=float(rng.uniform(-40, 40)),
                range_value=float(rng.uniform(8, 90)),
                yaw=float(rng.uniform(-math.pi, math.pi)),
                altitude_value=float(rng.uniform(20, 120))))
            self.assertLessEqual(float(np.linalg.norm(c.vel_ned)), ceiling_value + 1e-6)

    def test_yaw_rate_clamp(self):
        k = self.new_value()
        s = run_value3(k, [(90.0, 0.0)] * 20)
        self.assertAlmostEqual(abs(s['yaw_rate_dps']), k.a.yaw_rate_max,
                               delta=1e-6)

    def test_vertical_asymmetric_limit(self):
        """Descending <= descent_max , climbing <= climb_max (autopilot aligned with WPNAV_SPEED_DN /UP)."""
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        down_value = run_value3(k, [(0.0, 40.0)] * 40, range_value=80.0, altitude_value=200.0)['down_value']
        self.assertLessEqual(down_value, k.a.descent_max + 1e-6)
        k2 = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        up_value = run_value3(k2, [(0.0, -40.0)] * 40, range_value=80.0)['down_value']
        self.assertGreaterEqual(up_value, -k2.a.climb_max - 1e-6)

    def test_maneuver_budget_forward_preserves(self):
        """Even with the hardest centering the forward speed should not fall below sqrt(18^2-12^2)=13.4 (budget
sharing)."""
        k = self.new_value()
        s = run_value3(k, [(35.0, 35.0)] * 60, range_value=80.0, altitude_value=200.0)
        self.assertLessEqual(math.hypot(s['right_value'], s['down_value']),
                             k.a.maneuver_max + 1e-6)
        self.assertGreater(s['forward'], 13.0)

    def test_forward_inclination_limit(self):
        """Forward speed should not vary by more than acceleration_max*dt per step."""
        k = self.new_value()
        k.v_forward = 0.0
        previous_value = 0.0
        t = 0.0
        for _ in range(10):
            t += 0.05
            s = k.body_request(measurement_perform(t=t, dt=0.05))
            self.assertLessEqual(s['forward'] - previous_value,
                                 k.a.forward_acceleration_mps2 * 0.05 + 1e-9)
            previous_value = s['forward']

    def test_floor_ceiling_equal(self):
        """BY DEFAULT full throttle: the target is faster than us, voluntary slowing down is permanent loss of
shutdown (running 2 measurement: 16 m/s base -> range opened from 29 m to 95 m and authorization
returned)."""
        k = self.new_value()
        self.assertAlmostEqual(k.a.v_forward_floor, k.a.v_forward_ceiling, delta=1e-9)
        s = run_value3(k, [(0.0, 0.0)] * 120, range_value=60.0, area_root=6.0)
        self.assertGreater(s['forward'], 17.9)      # full throttle even at a distance

    def test_commit_ramp(self):
        """The ramp should work if the base is lowered MANUALLY (test button)."""
        def last_forward(range_value, area_value):
            k = self.new_value(v_forward_floor=12.0)
            k.v_forward = 10.0
            return run_value3(k, [(0.0, 0.0)] * 60, range_value=range_value, area_root=area_value)['forward']
        far = last_forward(60.0, 6.0)
        near_value = last_forward(18.0, 6.0)
        self.assertGreater(near_value, far + 1.0)
        # area_root should be able to trigger a commit on its own (range is stale)
        k = self.new_value(v_forward_floor=12.0)
        k.v_forward = 10.0
        with_area = run_value3(k, [(0.0, 0.0)] * 60, range_value=None, area_root=30.0)['forward']
        self.assertGreater(with_area, far + 1.0)

    # -- 5b. MEASUREMENT QUALITY (small / stale bbox) ---------------------------

    def test_small_bbox_pn_term_reduces(self):
        """If area_root is small (tail image of straight route, ~8 px) the D term should be reduced; The P term
must remain the SAME."""
        def run_value2(area_value):
            k = self.new_value()
            t, yaw = 0.0, 0.0
            for _ in range(60):
                t += 0.05
                yaw += math.radians(10.0) * 0.05      # fixed LOS return
                k.body_request(measurement_perform(t=t, ex=6.0, yaw=yaw, range_value=55.0,
                                         area_root=area_value))
            return k.diagnostic
        large = run_value2(20.0)      # q_area = 1
        small = run_value2(7.0)       # q_area = 0.125
        self.assertGreater(large['q'], 0.9)
        self.assertLess(small['q'], 0.2)
        # D was attenuated; P remained unchanged
        self.assertLess(abs(small['lateral_d']), 0.35 * abs(large['lateral_d']))
        self.assertAlmostEqual(small['lateral_p'], large['lateral_p'], delta=1e-6)

    def test_low_quality_derivative_more_heavy_filtered(self):
        """As q decreases, the derivative time constant should grow (SNR low -> very average)."""
        k = self.new_value()
        k.body_request(measurement_perform(t=0.05, area_root=20.0))
        good_value = k.diagnostic['derivative_tau']
        k2 = self.new_value()
        k2.body_request(measurement_perform(t=0.05, area_root=5.0))
        poor = k2.diagnostic['derivative_tau']
        self.assertAlmostEqual(good_value, k.a.derivative_tau_s, delta=1e-6)
        self.assertAlmostEqual(poor, k.a.derivative_tau_s * 3.0, delta=1e-6)

    def test_stale_bbox_quality_reduces(self):
        """As bbox_age grows, q should decrease (detection interrupt)."""
        k = self.new_value()
        fresh_value = measurement_perform(t=0.05, area_root=20.0); fresh_value.bbox_age_s = 0.03
        k.body_request(fresh_value); q_fresh = k.diagnostic['q']
        k2 = self.new_value()
        stale_value = measurement_perform(t=0.05, area_root=20.0); stale_value.bbox_age_s = 0.60
        k2.body_request(stale_value); q_stale = k2.diagnostic['q']
        self.assertGreater(q_fresh, 0.9)
        self.assertAlmostEqual(q_stale, k.a.age_quality_floor, delta=1e-6)
        self.assertGreater(q_stale, 0.0)   # Not completely: let there be some pioneering

    def test_low_quality_integrator_freezes(self):
        """Noisy/stale_value measurement should not accumulate permanent bias."""
        k = self.new_value()
        run_value3(k, [(5.0, 5.0)] * 100, area_root=5.0)   # q ~ 0 -> freeze
        self.assertAlmostEqual(k.i_lateral.value_value, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_vertical.value_value, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_yaw.value_value, 0.0, delta=1e-6)

    def test_pn_absolute_clamp(self):
        """In any case, the D term must not exceed pn_max."""
        k = self.new_value()
        t, yaw = 0.0, 0.0
        for _ in range(80):
            t += 0.05
            yaw += math.radians(70.0) * 0.05          # very fast rotation LOS
            k.body_request(measurement_perform(t=t, ex=8.0, yaw=yaw, range_value=80.0,
                                     area_root=40.0))
        self.assertLessEqual(abs(k.diagnostic['lateral_d']), k.a.pn_max_mps + 1e-6)

    def test_terminal_in_band_pn_authority_full(self):
        """MOST IMPORTANT QUALITY INVARIANT: q=1 should be in the band where the collision will occur.

        running 2 calibration: 25-35 m -> area_root median 26, 15-25 m -> 36. The quality layer
        should suppress noise at long range, but should NOT reduce the PN term in the terminal
        phase, or the corner-cutting ability is lost."""
        k = self.new_value()
        for area_value in (26.0, 36.0):
            k.body_request(measurement_perform(t=0.05, area_root=area_value, range_value=30.0))
            self.assertAlmostEqual(k.diagnostic['q'], 1.0, delta=1e-6)
            self.assertAlmostEqual(k.diagnostic['derivative_tau'], k.a.derivative_tau_s,
                                   delta=1e-6)

    def test_quality_forward_rate_does_not_affect(self):
        """Even in the worst measurement the closure should persist: the forward channel is independent of q."""
        k = self.new_value()
        s = run_value3(k, [(0.0, 0.0)] * 120, area_root=5.0, range_value=55.0)
        self.assertGreater(s['forward'], 17.9)

    # -- 6. ANTI-WINDUP ----------------------------------------------------

    def test_gate_outside_integrates_does_not(self):
        """|error| > When int_gate_deg the integrator should not swell."""
        k = self.new_value()
        run_value3(k, [(30.0, 30.0)] * 200)     # 10 Frame edge along s
        self.assertAlmostEqual(k.i_lateral.value_value, 0.0, delta=1e-6)
        self.assertAlmostEqual(k.i_vertical.value_value, 0.0, delta=1e-6)

    def test_during_handoff_large_ey_integrator_does_not_enter(self):
        """handoff typical ey=-25 (~-9 after setpoint) saturates on fast climb; The integrator must not swell
(additive ceiling must not hang)."""
        k = self.new_value()
        run_value3(k, [(0.0, -25.0)] * 200, range_value=50.0)
        self.assertLessEqual(abs(k.i_vertical.ki * k.i_vertical.value_value),
                             k.a.int_contribution_max + 1e-6)

    def test_when_saturated_integrator_damped(self):
        """The integrator accumulated while the channel is clamped must be vented."""
        k = self.new_value()
        run_value3(k, [(5.0, 0.0)] * 60)                    # inside the door: accumulate
        accumulated = abs(k.i_lateral.value_value)
        self.assertGreater(accumulated, 0.0)
        k._saturated_lateral = True
        run_value3(k, [(0.0, 0.0)] * 20, t0=3.1)            # saturated + error-free
        self.assertLess(abs(k.i_lateral.value_value), accumulated)

    def test_bbox_loss_after_again_lock(self):
        """long gap between command() calls -> full reset."""
        k = self.new_value()
        run_value3(k, [(5.0, 0.0)] * 60)
        self.assertGreater(abs(k.i_lateral.value_value), 0.0)
        k.body_request(measurement_perform(t=100.0, ex=5.0))       # 97 s gap
        # After reset only ONE step accumulated: (error - dead band)*dt
        expected_value = (5.0 - k.a.dead_band_deg) * 0.05
        self.assertAlmostEqual(k.i_lateral.value_value, expected_value, delta=1e-3)
        self.assertAlmostEqual(k.d_ex._filtered, 0.0, delta=1e-9)

    def test_dead_band(self):
        """Small error within the deadband should not generate commands (limit cycle)."""
        k = self.new_value()
        s = run_value3(k, [(0.1, 0.1)] * 10,
                   range_value=30.0)
        self.assertAlmostEqual(s['yaw_rate_dps'], 0.0, delta=1e-6)

    # --7. handoff SEEDING ----------------------------------------------

    def test_seed_initial_command_equalizes(self):
        """The FIRST command after handoff should be the last command of ~ positioned, EVEN IF the vertical
error is large (residue based seeding)."""
        handoff = {'cmd_vel_ned': [17.0, 2.5, -1.5], 'cmd_yaw_rad': 0.0,
                 'range_m': 45.0}
        k = self.new_value()
        k.seed_value2(handoff)
        s = k.body_request(measurement_perform(t=0.0, dt=0.05, ex=0.0, ey=-25.0,
                                     range_value=45.0))
        self.assertAlmostEqual(s['right_value'], 2.5, delta=0.05)
        self.assertAlmostEqual(s['down_value'], -1.5, delta=0.05)
        self.assertAlmostEqual(s['forward'], 17.0, delta=0.4)

    def test_seed_decays(self):
        """The seed should go to zero with tau_seed (leaving no permanent bias)."""
        handoff = {'cmd_vel_ned': [17.0, 6.0, -4.0], 'cmd_yaw_rad': 0.0,
                 'range_m': 45.0}
        k = self.new_value()
        k.seed_value2(handoff)
        run_value3(k, [(0.0, -25.0)] * 200, range_value=45.0)     # 10 s
        self.assertLess(abs(k.seed_right), 0.05)
        self.assertLess(abs(k.seed_down), 0.05)

    def test_seed_yaw_with_rotated(self):
        """cmd_yaw_rad=90 deg while NED [0,17,0] should be FORWARD 17 on the body."""
        handoff = {'cmd_vel_ned': [0.0, 17.0, 0.0],
                 'cmd_yaw_rad': math.radians(90.0), 'range_m': 40.0}
        k = self.new_value()
        k.seed_value2(handoff)
        self.assertAlmostEqual(k.v_forward, 17.0, delta=1e-6)
        self.assertAlmostEqual(k._seed_request[0], 0.0, delta=1e-6)

    def test_handoff_if_absent_does_not_crash(self):
        k = self.new_value()
        k.seed_value2(None)
        k.seed_value2({})
        k.seed_value2({'cmd_vel_ned': None, 'cmd_yaw_rad': None, 'range_m': 'x'})
        s = k.body_request(measurement_perform())
        self.assertTrue(math.isfinite(s['forward']))

    # -- 8. EMNIYET / DAYANIKLILIK -----------------------------------------

    def test_altitude_floor_descent_prevents(self):
        """Descent below the altitude base should not be commanded."""
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        s = run_value3(k, [(0.0, 30.0)] * 20, altitude_value=8.0)
        self.assertLessEqual(s['down_value'], 0.0)
        self.assertLess(s['down_value'], 0.0)   # violation -> soft climbing

    def test_altitude_floor_integrator_freezes(self):
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        run_value3(k, [(0.0, 6.0)] * 100, altitude_value=8.0)
        self.assertAlmostEqual(k.i_vertical.value_value, 0.0, delta=1e-6)

    def test_yaw_if_absent_zero_command(self):
        """If the attribute never came, it cannot be converted to NED -> zero command."""
        k = self.new_value()
        o = measurement_perform(yaw=None)
        c = k.command_value(o)
        self.assertAlmostEqual(float(np.linalg.norm(c.vel_ned)), 0.0, delta=1e-9)
        self.assertIsNone(c.yaw_rate_dps)

    def test_yaw_when_lost_last_yaw_held(self):
        k = self.new_value()
        k.command_value(measurement_perform(t=0.0, yaw=math.radians(45.0)))
        c = k.command_value(measurement_perform(t=0.05, yaw=None))
        self.assertGreater(float(np.linalg.norm(c.vel_ned)), 1.0)

    def test_nan_and_none_robustness(self):
        k = self.new_value()
        o = measurement_perform(range_value=None, area_root=None)
        o.ex_deg = None
        o.ey_deg = None
        c = k.command_value(o)
        self.assertTrue(np.all(np.isfinite(c.vel_ned)))

    def test_zero_error_zero_maneuver(self):
        """If the target is dead center and stationary: forward speed only."""
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        s = run_value3(k, [(0.0, 0.0)] * 40)
        self.assertAlmostEqual(s['right_value'], 0.0, delta=1e-6)
        self.assertAlmostEqual(s['down_value'], 0.0, delta=1e-6)
        self.assertAlmostEqual(s['yaw_rate_dps'], 0.0, delta=1e-6)
        self.assertGreater(s['forward'], 15.0)

    # --9. CLOSED LOOP (kinematic dry running) -----------------------------

    def test_disabled_loop_horizontal_converges(self):
        """Simple 2D kinematics: lateral velocity should close angle LOS.

        Model: fixed range R, d(ex)/dt = -v_right/R (rad). This is the very kinematics that the
        controller assumes; The test shows that the gain/signal/clamp chain is truly closed loop
        stable."""
        k = self.new_value()
        R = 30.0
        ex = 20.0
        dt = 0.05
        t = 0.0
        history_value = []
        for _ in range(400):                              # 20 s
            t += dt
            s = k.body_request(measurement_perform(t=t, dt=dt, ex=ex, range_value=R))
            # yaw_rate turns the frame, lateral speed turns LOS; both in the direction that reduces ex
            # (simplified: lateral velocity only).
            ex += math.degrees(-s['right_value'] / R) * dt
            history_value.append(ex)
        self.assertLess(abs(history_value[-1]), 1.0)
        self.assertLess(max(history_value[200:]), 2.0)           # no hang/limit cycle

    def test_disabled_loop_vertical_converges(self):
        k = self.new_value(framing_weight_far=0.0, framing_weight_near=0.0)
        R = 30.0
        ey = -20.0
        dt = 0.05
        t = 0.0
        for _ in range(600):
            t += dt
            s = k.body_request(measurement_perform(t=t, dt=dt, ey=ey, range_value=R,
                                         altitude_value=80.0))
            ey += math.degrees(-s['down_value'] / R) * dt
        self.assertLess(abs(ey), 1.5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
