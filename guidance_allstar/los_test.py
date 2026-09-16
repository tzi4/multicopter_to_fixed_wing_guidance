#!/usr/bin/env python3
"""
los_test.py - los_guidance.py offline (SIM-LESS) verification suite
============================================================================
    python3 los_test.py            # all
    python3 los_test.py -v         # ayrintili
    python3 los_test.py Geometry   # single class

NO TESTS ASK FOR SIM/SITL/GAZEBO/Redis/MAVLINK. Measuring packages are produced synthetically;
Closed-loop tests run their own point-mass kinematics.

The tests are in three layers: 1) GEOMETRY -- REAL where los_guidance talks the same math as
yildizlar_gimbal (eps = -aim_active - ey, ex = side angle, receiving mirror) It is proved numerically
with the object VirtualGimbal. 2) UNIT -- individual law parts: pioneer mark, extraction of our own
yaw rate, clamps, same-sample guard, terminal gate, seeding. 3) CLOSED LOOP -- 3D point-mass engine:
target flies straight/turning, fighter follows command with first order delay, measurements are
generated from ACTUAL geometry. Hit/miss and hold on FOV are measured.
"""

from __future__ import annotations

import math
import sys
import unittest

import numpy as np

import guidance_config as cfg
import los_guidance as L
from visual_base import Measurement

# Speed ​​cap NOT WRITTEN CONSTANT: guidance_config is the only source of accuracy
# (2026-08-03'te 18 -> 20 yukseltildi; params/swarm_copter.parm WPNAV_SPEED
# 2000 cm/s = 20 m/s already allowed this). The test reads this value so that if it changes again in
# the future, it will not silently test incorrectly.
V_MAX = float(cfg.VISUAL_MAX_SPEED_MPS)


# --------------------------------------------------------------- yardimcilar

def measurement(t, ex, ey, range_value=40.0, dt=0.05, yaw=0.0, vel=None, pos=None,
          bbox=(30.0, 18.0), age_value=0.02, pitch=0.0):
    """Synthetic Measurement package. bbox_w/h exists only for area_root and same-sample signature; the
controller DOES NOT DERIVE RANGE from these (smoke test: bbox width depends on aspect)."""
    w, h = bbox
    return Measurement(
        t=t, dt=dt, ex_deg=ex, ey_deg=ey, bbox_w=w, bbox_h=h,
        area_root=math.sqrt(w * h), coverage_pct=100.0 * w / 1280.0,
        bbox_age_s=age_value, range_m_value=range_value,
        pos_ned=np.array([0.0, 0.0, -60.0]) if pos is None
        else np.asarray(pos, float),
        vel_ned=np.array([12.0, 0.0, 0.0]) if vel is None
        else np.asarray(vel, float),
        yaw_rad=yaw, roll_rad=0.0, pitch_rad=pitch)


def run_value2(k, array_value, dt=0.05, t0=0.0):
    """It outputs a sequence of (ex, ey) to the controller in order; Returns a list of diagnoses.

    array: [(ex, ey), ...] or [(ex, ey, kwargs_dict), ...]
    """
    record_value = []
    for i, ogr in enumerate(array_value):
        if len(ogr) == 3:
            ex, ey, kw = ogr
        else:
            (ex, ey), kw = ogr, {}
        o = measurement(t0 + i * dt, ex, ey, dt=dt, **kw)
        cmd = k.command_value(o)
        record_value.append((dict(k.diagnostic), cmd))
    return record_value


# ============================================================ 1) GEOMETRY

class Geometry(unittest.TestCase):
    """Is the angle contract of los_guidance the same as the REAL yildizlar_gimbal?"""

    @classmethod
    def setUpClass(cls):
        try:
            import yildizlar_gimbal as YG
        except Exception as exc:            # pragma: no cover
            raise unittest.SkipTest(f"yildizlar_gimbal import edilemedi: {exc}")
        cls.YG = YG

    def test_aim_mirror_analytical(self):
        """los_guidance.analytical_aim == yildizlar_gimbal.analytical_aim"""
        for back, down in [(25, 13), (25, 3), (40, 20), (15, 5), (30, 0)]:
            self.assertAlmostEqual(L.analytical_aim(back, down),
                                   self.YG.analytical_aim(back, down), places=12)

    def test_aim_mirror_ramp(self):
        """los_guidance.aim_active == VirtualGimbal.aim_active_deg (range ramp)"""
        g = self.YG.VirtualGimbal(aim_pitch_deg=-27.47)
        for r in [None, 10, 50, 119.9, 120, 150, 185, 249.9, 250, 400]:
            self.assertAlmostEqual(L.aim_active(-27.47, r),
                                   g.aim_active_deg(r), places=12,
                                   msg=f"range={r}")

    def test_eps_relation(self):
        """(1) eps = -aim_active - ey and ex = side angle.

        RAW pixels are generated from the real gimbal (pixel_generate), then ex/ey are read with
        angle_error_value(). De-rotation of the virtual gimbal is also included in the path by giving the
        body roll/pitch."""
        for aim in (0.0, -27.47, -10.0):
            g = self.YG.VirtualGimbal(aim_pitch_deg=aim)
            for eps_ger in (0.0, 5.0, 12.0, 27.5, -8.0):
                for lateral in (0.0, 6.0, -11.0, 20.0):
                    for roll, pitch in ((0.0, 0.0), (0.20, -0.09), (-0.15, 0.12)):
                        px = g.pixel_generate(eps_ger, lateral, roll, pitch)
                        if px is None:
                            continue
                        ex, ey = g.angle_error_value(px[0], px[1], roll, pitch,
                                              range_m_value=40.0)
                        aim_e = L.aim_active(aim, 40.0)
                        eps_backward = L.eps_solve(ex, ey, aim_e)
                        self.assertAlmostEqual(
                            ex, lateral, places=4,
                            msg=f"ex !=side (aim= {aim} eps = {eps_ger} side= {lateral} )")
                        # Since eps_solve performs the projection correction, the relation must be EXACT at aim=0; aim != ex in
                        # 0, small second order residual remains since aim is used instead of azimuth in the rotated frame.
                        tol = 0.01 if abs(aim) < 1e-9 else 1.6
                        self.assertLess(
                            abs(eps_backward - eps_ger), tol,
                            msg=(f"eps recovery (aim= {aim} eps = {eps_ger} "
                                 f"side= {lateral} roll = {roll} ) -> {eps_backward:.3f}"))

    def test_aim_at_zero_eps_equals_minus_ey(self):
        """scenario.sh AIM=0 -> eps = -ey (is-altitude geometry control)."""
        k = L.LosController(aim_deg=0.0)
        k.seed_value2(None)
        k.command_value(measurement(0.0, 0.0, -27.5, range_value=40.0))
        self.assertAlmostEqual(k.diagnostic['eps'], 27.5, places=6)
        k2 = L.LosController(aim_deg=0.0)
        k2.seed_value2(None)
        k2.command_value(measurement(0.0, 0.0, +8.0, range_value=40.0))
        self.assertAlmostEqual(k2.diagnostic['eps'], -8.0, places=6)

    def test_default_aim_is_zero(self):
        """If there is no env/arg, my name should be 0 (do not conflict with scenario.sh)."""
        import os
        previous = os.environ.pop('YILDIZ_AIM', None)
        try:
            k = L.LosController()
            self.assertEqual(k.aim_deg, 0.0)
            self.assertEqual(k.aim_source, 'default_value2')
            k2 = L.LosController(back_m=25, down_m=13)
            self.assertAlmostEqual(k2.aim_deg, -27.4744, places=3)
        finally:
            if previous is not None:
                os.environ['YILDIZ_AIM'] = previous


# =============================================================== 2) UNIT

class Unit(unittest.TestCase):

    def new_value(self, **kw):
        """Unit tests test the LAW CORE; raw framing protection (part 3b) is turned OFF by default (k_fov=0,
climb_max_value free) so that vertical command tests do not interfere with the protection's correction.
The protection is tested in its class (FovProtection)."""
        kw.setdefault('aim_deg', 0.0)
        kw.setdefault('k_fov', 0.0)
        kw.setdefault('climb_max_mps', 99.0)
        k = L.LosController(**kw)
        k.seed_value2(None)
        return k

    # ---- basic direction ----

    def test_centered_target_los_along(self):
        """ex=0, ey=0 fixed -> command parallel to horizon, noseward, no leading edge."""
        k = self.new_value()
        record_value = run_value2(k, [(0.0, 0.0)] * 40)
        diagnostic, cmd = record_value[-1]
        self.assertAlmostEqual(diagnostic['lead_az'], 0.0, places=6)
        self.assertAlmostEqual(diagnostic['lead_el'], 0.0, places=6)
        self.assertAlmostEqual(diagnostic['c_deg'], 0.0, places=6)
        self.assertAlmostEqual(diagnostic['g_deg'], 0.0, places=6)
        self.assertGreater(cmd.vel_ned[0], 10.0)          # forward
        self.assertAlmostEqual(cmd.vel_ned[1], 0.0, places=6)
        self.assertAlmostEqual(cmd.vel_ned[2], 0.0, places=6)

    def test_target_above_climb(self):
        """ey<0 (target above holocenter, aim=0 -> eps>0) -> climb."""
        k = self.new_value()
        _, cmd = run_value2(k, [(0.0, -27.5)] * 10)[-1]
        self.assertLess(cmd.vel_ned[2], -2.0)             # NED z down (+)
        self.assertGreater(cmd.vel_ned[0], 0.0)

    def test_target_below_descent(self):
        k = self.new_value()
        _, cmd = run_value2(k, [(0.0, +15.0)] * 10)[-1]
        self.assertGreater(cmd.vel_ned[2], 1.0)

    def test_target_to_right_right_speed_and_yaw(self):
        k = self.new_value()
        diagnostic, cmd = run_value2(k, [(20.0, 0.0)] * 10)[-1]
        self.assertGreater(cmd.vel_ned[1], 3.0)           # yaw=0 -> y = right
        self.assertGreater(cmd.yaw_rate_dps, 1.0)

    def test_sign_symmetry(self):
        """Mirror move -> mirror command.

NOTE: the limits on the vertical component of NED velocity are intentionally asymmetric        (WPNAV_SPEED_UP 10 m/s, WPNAV_SPEED_DN 5 m/s). Therefore, symmetry is tested via COMMAND
        DIRECTION (c_deg, g_deg, leading); In horizontal speed, it is searched exactly."""
        for i, array_value in enumerate([[(0.6 * n, 0.0) for n in range(40)],
                                  [(0.0, 0.6 * n) for n in range(40)]]):
            ka = self.new_value(gamma_min_deg=-55.0, gamma_max_deg=55.0)
            kb = self.new_value(gamma_min_deg=-55.0, gamma_max_deg=55.0)
            ta, ca = run_value2(ka, array_value)[-1]
            tb, cb = run_value2(kb, [(-a, -b) for a, b in array_value])[-1]
            self.assertAlmostEqual(ta['lead_az'], -tb['lead_az'], places=6)
            self.assertAlmostEqual(ta['lead_el'], -tb['lead_el'], places=6)
            self.assertAlmostEqual(ta['c_deg'], -tb['c_deg'], places=6)
            self.assertAlmostEqual(ta['g_deg'], -tb['g_deg'], places=6)
            np.testing.assert_allclose(ca.vel_ned[1], -cb.vel_ned[1],
                                       atol=1e-6, err_msg=f"series {i}")

    # ---- SIGN of legacy PN (what is fixed by legacy1) ----

    def test_lead_los_ratio_direction(self):
        """If LOS turns right, the leader must also turn RIGHT (PN). In legacy1 this term was reverse signed
(see los_guidance section 0-a)."""
        k = self.new_value()
        record_value = run_value2(k, [(0.4 * n, 0.0) for n in range(60)])   # ex artiyor
        diagnostic, cmd = record_value[-1]
        self.assertGreater(diagnostic['q_az_point'], 1.0)           # LOS right
        self.assertGreater(diagnostic['lead_az'], 1.0)               # leading right
        self.assertGreater(diagnostic['c_deg'], diagnostic['ex_ong'])     # LOS'un otesinde

    def test_lead_vertically_los_ratio_direction(self):
        """If eps is rising (target is rising) the leading must be UP."""
        k = self.new_value()
        # aim=0 -> eps = -ey; ey azaliyorsa eps artiyor
        record_value = run_value2(k, [(0.0, -0.4 * n) for n in range(60)])
        diagnostic, _ = record_value[-1]
        self.assertGreater(diagnostic['q_el_point'], 1.0)
        self.assertGreater(diagnostic['lead_el'], 1.0)
        self.assertGreater(diagnostic['g_deg'], diagnostic['eps_ong'] - 1e-9)

    def test_lead_washout_decays(self):
        """When the ratio LOS is reset, let the leader absorb towards zero with tau_lead."""
        k = self.new_value(tau_lead_s=1.0)
        run_value2(k, [(0.4 * n, 0.0) for n in range(60)])
        peak = k.diagnostic['lead_az']
        self.assertGreater(peak, 1.0)
        fixed = k.diagnostic['ex_ong']
        run_value2(k, [(fixed, 0.0)] * 120, t0=100.0)   # ex fixed -> LOS rate 0
        self.assertLess(abs(k.diagnostic['lead_az']), 0.25 * peak)

    # ---- DERIVING OUR OWN YAW RATIO (equation 2) ----

    def test_own_own_yaw_mock_ratio_does_not_generate(self):
        """Inertial LOS CONSTANT, we turn right. Since ex_point = -yaw_point, q_az_point ~ 0 -> PRIORITY SHOULD
NOT ACCUMULATE."""
        k = self.new_value()
        w = 20.0                       # deg /s yaw rate
        dt = 0.05
        for i in range(120):
            t = i * dt
            yaw = math.radians(w * t)
            ex = 30.0 - w * t          # LOS fixed, body rotating
            k.command_value(measurement(t, ex, 0.0, dt=dt, yaw=yaw))
        self.assertLess(abs(k.diagnostic['q_az_point']), 2.0,
                        "our own yaw rate is not issued")
        self.assertLess(abs(k.diagnostic['lead_az']), 3.0,
                        "our own conversion produces false pioneer")

    def test_yaw_ratio_unless_subtracted_distinguish_is(self):
        """Counterevidence: The SAME ex sequence produces large precursor while yaw is constant."""
        k = self.new_value()
        dt, w = 0.05, 20.0
        for i in range(120):
            t = i * dt
            k.command_value(measurement(t, 30.0 - w * t, 0.0, dt=dt, yaw=0.0))
        self.assertLess(k.diagnostic['lead_az'], -5.0)

    def test_yaw_wrap_derivative_preserves(self):
        """The fake giant yaw_point should not be generated when jumping at yaw +-pi."""
        k = self.new_value()
        dt, w = 0.05, 30.0
        for i in range(80):
            t = i * dt
            yaw = math.radians(L.wrap180(170.0 + w * t))
            k.command_value(measurement(t, 0.0, 0.0, dt=dt, yaw=yaw))
            self.assertLess(abs(k.diagnostic['yaw_point']), 60.0)
        self.assertAlmostEqual(k.diagnostic['yaw_point'], w, delta=4.0)

    # ---- speed law ----

    def test_speed_incremental_ramp(self):
        """v_d = |v_now| +k_a; While aligned, the exact k_a is added."""
        k = self.new_value()
        o = measurement(0.0, 0.0, 0.0, vel=[12.0, 0.0, 0.0])
        k.command_value(o)
        self.assertAlmostEqual(k.diagnostic['theta_deg'], 0.0, places=3)
        self.assertAlmostEqual(k.diagnostic['k_a'], k.ka_peak, places=6)
        self.assertAlmostEqual(k.diagnostic['v_d'], 14.0, places=6)

    def test_acceleration_gate_when_misaligned_closes(self):
        """theta >= theta_threshold -> k_a = 0 (steer first, throttle later)."""
        k = self.new_value(theta_threshold_deg=35.0)
        # command direction ~90 deg right, current speed forward -> theta ~ 90
        k.command_value(measurement(0.0, 80.0, 0.0, vel=[12.0, 0.0, 0.0]))
        self.assertGreater(k.diagnostic['theta_deg'], 35.0)
        self.assertAlmostEqual(k.diagnostic['k_a'], 0.0, places=9)
        self.assertAlmostEqual(k.diagnostic['v_d'], 12.0, places=6)

    def test_speed_ceiling(self):
        k = self.new_value()
        _, cmd = run_value2(k, [(0.0, 0.0, {'vel': [17.9, 0.0, 0.0]})] * 5)[-1]
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned), V_MAX + 1e-9)

    def test_vertical_ceiling_direction_preserves(self):
        """Vertical climber |vz|  It should not exceed WPNAV_SPEED_UP, the direction should not be distorted."""
        k = self.new_value(gamma_max_deg=55.0)   # new(): climb_max_value released
        _, cmd = run_value2(k, [(0.0, -55.0, {'vel': [17.0, 0.0, -3.0]})] * 20)[-1]
        self.assertLessEqual(-cmd.vel_ned[2], L.WPNAV_SPEED_UP_MPS + 1e-6)
        # Is direction preserved: horizontal/vertical ratio should be tan(g)
        horizontal = math.hypot(cmd.vel_ned[0], cmd.vel_ned[1])
        self.assertAlmostEqual(math.degrees(math.atan2(-cmd.vel_ned[2], horizontal)),
                               k.diagnostic['g_deg'], delta=0.5)

    def test_vertical_descent_ceiling(self):
        k = self.new_value(gamma_min_deg=-25.0)
        _, cmd = run_value2(k, [(0.0, 60.0, {'vel': [17.0, 0.0, 3.0],
                                      'pos': [0, 0, -200.0]})] * 20)[-1]
        self.assertLessEqual(cmd.vel_ned[2], L.WPNAV_SPEED_DN_MPS + 1e-6)

    # ---- clamps / emniyet ----

    def test_lead_clamps(self):
        k = self.new_value(lead_az_max_deg=10.0, lead_el_max_deg=5.0, k_pn=3.0)
        run_value2(k, [(2.0 * n, -2.0 * n) for n in range(80)])
        self.assertLessEqual(abs(k.diagnostic['lead_az']), 10.0 + 1e-9)
        self.assertLessEqual(abs(k.diagnostic['lead_el']), 5.0 + 1e-9)

    def test_yaw_rate_ceiling_and_slew(self):
        k = self.new_value()
        previous_value = 0.0
        for i in range(60):
            cmd = k.command_value(measurement(i * 0.05, 90.0, 0.0, dt=0.05))
            self.assertLessEqual(abs(cmd.yaw_rate_dps), k.yaw_rate_max + 1e-9)
            self.assertLessEqual(abs(cmd.yaw_rate_dps - previous_value),
                                 k.yaw_acceleration_max * 0.05 + 1e-6)
            previous_value = cmd.yaw_rate_dps

    def test_yaw_dead_band(self):
        k = self.new_value()
        for _ in range(30):
            cmd = k.command_value(measurement(0.0, 0.4, 0.0))
        self.assertAlmostEqual(cmd.yaw_rate_dps, 0.0, places=6)

    def test_yaw_disabled(self):
        k = self.new_value(yaw_enabled=False)
        cmd = k.command_value(measurement(0.0, 25.0, 0.0))
        self.assertIsNone(cmd.yaw_rate_dps)

    def test_altitude_floor(self):
        k = self.new_value()
        _, cmd = run_value2(k, [(0.0, 40.0, {'pos': [0, 0, -8.0]})] * 10)[-1]
        self.assertLessEqual(cmd.vel_ned[2], 0.0)

    def test_gamma_clamp(self):
        k = self.new_value(gamma_max_deg=30.0)
        run_value2(k, [(0.0, -80.0)] * 10)
        self.assertLessEqual(k.diagnostic['g_deg'], 30.0 + 1e-9)

    # ----measurement health----

    def test_same_sample_protection(self):
        """If the same bbox occurs again, the derivative should not be pulled to ZERO (low reading)."""
        k = self.new_value()
        dt = 0.05
        t = 0.0
        # new sample every 3 cycle: actual LOS rate 8 deg/s
        ex = 0.0
        for i in range(90):
            if i % 3 == 0:
                ex = 8.0 * t
            k.command_value(measurement(t, ex, 0.0, dt=dt))
            t += dt
        self.assertAlmostEqual(k.diagnostic['ex_point'], 8.0, delta=1.5)

    def test_same_sample_protection_without_low_reads(self):
        """Counterproof: If we give the SAME sequence as 'new sample every cycle' (breaking the signature with
bbox_w) the rate LOS is read LOW."""
        def _run_item(signature_break):
            k = self.new_value()
            dt, t, ex = 0.05, 0.0, 0.0
            for i in range(90):
                if i % 3 == 0:
                    ex = 8.0 * t
                w = 30.0 + (i * 1e-6 if signature_break else 0.0)
                k.command_value(measurement(t, ex, 0.0, dt=dt, bbox=(w, 18.0)))
                t += dt
            return k.diagnostic['ex_point']
        protected, unprotected = _run_item(False), _run_item(True)
        self.assertGreater(protected, unprotected + 1.0,
                           f"protection did not gain: {protected} vs {unprotected}")
        self.assertAlmostEqual(protected, 8.0, delta=1.5)

    def test_long_gap_derivative_resets(self):
        k = self.new_value()
        run_value2(k, [(0.5 * n, 0.0) for n in range(40)])
        self.assertGreater(abs(k.diagnostic['ex_point']), 2.0)
        k.command_value(measurement(100.0, 20.0, 0.0))          # 100 s gap
        self.assertAlmostEqual(k.diagnostic['ex_point'], 0.0, places=9)

    def test_latency_compensation(self):
        """As bbox ages, the bearing should be moved forward with ex_point."""
        k = self.new_value()
        run_value2(k, [(0.5 * n, 0.0) for n in range(40)])
        last_ex = 0.5 * 39
        k.command_value(measurement(2.0, last_ex, 0.0, age_value=0.25))
        self.assertGreater(k.diagnostic['ex_ong'], last_ex)

    # ---- range contract ----

    def test_range_only_from_estimator(self):
        """While range_m_value is None, NO RANGE IS DEDERIVED from the width of bbox."""
        k = self.new_value()
        k.command_value(measurement(0.0, 0.0, 0.0, range_value=None))
        self.assertIsNone(k.diagnostic['range_value'])
        self.assertEqual(k.diagnostic['range_source'], 'absent')

    def test_range_when_stale_last_value_held(self):
        k = self.new_value()
        k.command_value(measurement(0.0, 0.0, 0.0, range_value=33.0))
        k.command_value(measurement(0.05, 0.1, 0.0, range_value=None))
        self.assertAlmostEqual(k.diagnostic['range_value'], 33.0, places=6)
        self.assertEqual(k.diagnostic['range_source'], 'last_value')

    def test_without_range_command_generated(self):
        """The law should work even when there is no range (the inner loop is fed by the angles)."""
        k = self.new_value()
        _, cmd = run_value2(k, [(5.0, -10.0, {'range_value': None})] * 20)[-1]
        self.assertTrue(np.all(np.isfinite(cmd.vel_ned)))
        self.assertGreater(np.linalg.norm(cmd.vel_ned), 1.0)

    def test_progress_signal_area_root(self):
        """d(ln area_root )/dt = - R_point /R unscaled closing ratio."""
        k = self.new_value()
        dt = 0.05
        for i in range(120):
            # R 40 -> 20 m linear;  area_root ~ 1 /R
            R = 40.0 - 20.0 * (i / 119.0)
            measure = 1000.0 / R
            k.command_value(measurement(i * dt, 0.0, 0.0, range_value=R, dt=dt,
                          bbox=(measure, measure)))
        self.assertGreater(k.diagnostic['ln_area_point'], 0.0)     # buyuyor
        self.assertLess(k.diagnostic['range_point'], 0.0)         # is closing
        # -Is it consistent with R_point/R (rough)
        expected_value = -k.diagnostic['range_point'] / k.diagnostic['range_value']
        self.assertAlmostEqual(k.diagnostic['ln_area_point'], expected_value, delta=0.05)

    # ---- terminal gate ----

    def test_terminal_range_gate_lead_freezes(self):
        k = self.new_value(terminal_range_m=12.0)
        run_value2(k, [(0.4 * n, 0.0, {'range_value': 40.0}) for n in range(60)])
        lead = k.diagnostic['lead_az']
        self.assertGreater(lead, 1.0)
        run_value2(k, [(30.0 + 0.4 * n, 0.0, {'range_value': 8.0}) for n in range(60)],
            t0=10.0)
        self.assertTrue(k.diagnostic['terminal'])
        self.assertAlmostEqual(k.diagnostic['lead_az'], lead, places=9)

    def test_terminal_area_gate(self):
        k = self.new_value(terminal_area_root=50.0)
        k.command_value(measurement(0.0, 0.0, 0.0, range_value=40.0, bbox=(60.0, 60.0)))
        self.assertTrue(k.diagnostic['terminal'])

    def test_terminal_full_throttle(self):
        """The alignment gate is bypassed at the terminal."""
        k = self.new_value(terminal_range_m=12.0)
        k.command_value(measurement(0.0, 80.0, 0.0, range_value=8.0, vel=[12.0, 0.0, 0.0]))
        self.assertGreater(k.diagnostic['theta_deg'], 35.0)
        self.assertAlmostEqual(k.diagnostic['k_a'], k.ka_peak, places=9)

    # ----handoff/insemination----

    def test_seed_without_handoff(self):
        k = L.LosController(aim_deg=0.0)
        k.seed_value2(None)
        self.assertEqual(k.lead_az, 0.0)
        self.assertIsNone(k.diagnostic.get('range_value'))
        cmd = k.command_value(measurement(0.0, 0.0, 0.0))
        self.assertTrue(np.all(np.isfinite(cmd.vel_ned)))

    def test_seed_with_handoff(self):
        k = L.LosController(aim_deg=0.0)
        k.seed_value2({'t_mono': 1.0, 'cmd_vel_ned': [16.0, 1.0, -0.5],
                   'cmd_yaw_rad': 0.3, 'range_m': 47.5})
        k.command_value(measurement(0.0, 0.0, 0.0, range_value=None))
        self.assertAlmostEqual(k.diagnostic['range_value'], 47.5, places=6)

    def test_seed_invalid_handoff(self):
        k = L.LosController(aim_deg=0.0)
        k.seed_value2({'range_m': 'invalid'})
        self.assertIsNone(k._range_last)

    def test_handoff_after_jump_absent(self):
        """The first instruction immediately after handoff does not require overclocking in the first loop."""
        k = self.new_value()
        cmd = k.command_value(measurement(0.0, 0.0, -27.5, vel=[16.0, 0.0, 0.0]))
        self.assertLessEqual(np.linalg.norm(cmd.vel_ned), V_MAX + 1e-9)
        self.assertAlmostEqual(cmd.yaw_rate_dps, 0.0, delta=6.0)

    # ----argparse/main path----

    def test_argparse_controller_creates(self):
        a = L.arg_parser().parse_args(
            ['--k-pn', '1.4', '--tau-yak', '0.6', '--vertical-ratio', '0.6',
             '--yaw-disabled', '--aim', '0', '--terminal-range', '15'])
        k = L.controller_build(a)
        self.assertEqual(k.k_pn, 1.4)
        self.assertEqual(k.tau_yak, 0.6)
        self.assertEqual(k.vertical_ratio, 0.6)
        self.assertFalse(k.yaw_enabled)
        self.assertEqual(k.aim_deg, 0.0)
        self.assertEqual(k.terminal_range, 15.0)

    def test_vertical_ratio_reduced_climb_decreases(self):
        ka, kb = self.new_value(vertical_ratio=1.0), self.new_value(vertical_ratio=0.4)
        _, ca = run_value2(ka, [(0.0, -27.5)] * 10)[-1]
        _, cb = run_value2(kb, [(0.0, -27.5)] * 10)[-1]
        self.assertLess(-ca.vel_ned[2] * 0.999, -ca.vel_ned[2])   # sanity
        self.assertGreater(-ca.vel_ned[2], -cb.vel_ned[2])


class FovProtection(unittest.TestCase):
    """Part 3b: since the camera is fixed to the fuselage, the commanded climb produces nose up pitch and
the target RAW emerges from the bottom of the frame. This was the root of the 16 handoff/return loop
in the first ellipse run."""

    def new_value(self, **kw):
        kw.setdefault('aim_deg', 0.0)
        # MOUNT FIXED TO 30 (2026-08-04): sim default switched to 0 grade (pitch-servo gimbal decision), but
        # numerical expectations of this grade (alt_angle = mount+pitch-eps, clamp values) belong to the
        # mounting geometry +30, for which the law LOS was DEVELOPED AND VERIFIED. LOS is a frozen benchmark
        # artifact; Instead of basing the test on the new geometry, we write EXPLICIT which geometry it is
        # valid for.
        kw.setdefault('mount_deg', 30.0)
        k = L.LosController(**kw)
        k.seed_value2(None)
        return k

    def test_alt_angle_relation(self):
        """(12) alt_angle = (mount + pitch) - eps"""
        k = self.new_value(k_fov=0.0)
        # aim=0 -> eps = -ey (neutral at projection correction ex=0)
        k.command_value(measurement(0.0, 0.0, -15.7, range_value=35.0))
        # measure() returns pitch_rad=0 -> alt_angle = 30 - 15.7
        self.assertAlmostEqual(k.diagnostic['alt_angle'], 30.0 - 15.7, places=3)
        self.assertAlmostEqual(k.diagnostic['eps'], 15.7, places=3)

    def test_target_frame_when_below_climb_reduced(self):
        """alt_angle > fov_margin -> command path angle is pulled DOWN."""
        enabled_value = self.new_value(k_fov=1.0, climb_max_mps=99.0)
        disabled = self.new_value(k_fov=0.0, climb_max_mps=99.0)
        # measured ellipse state: eps=15.7, pitch=+11.2 -> alt_angle=25.5
        o_kw = {'range_value': 35.0, 'pitch': math.radians(11.2)}
        _, c_enabled = run_value2(enabled_value, [(0.0, -15.7, o_kw)] * 10)[-1]
        _, c_disabled = run_value2(disabled, [(0.0, -15.7, o_kw)] * 10)[-1]
        self.assertAlmostEqual(enabled_value.diagnostic['alt_angle'], 25.5, delta=0.1)
        self.assertAlmostEqual(enabled_value.diagnostic['fov_correction'], -(25.5 - 14.0),
                               delta=0.1)
        self.assertLess(enabled_value.diagnostic['g_deg'], disabled.diagnostic['g_deg'] - 5.0)
        self.assertLess(-c_enabled.vel_ned[2], -c_disabled.vel_ned[2])

    def test_target_frame_when_above_climb_increased(self):
        """Symmetrical direction: alt_angle < -fov_margin -> g is pushed UP."""
        k = self.new_value(k_fov=1.0, climb_max_mps=99.0)
        # eps large, pitch nose down -> ABOVE target axis
        run_value2(k, [(0.0, -50.0, {'range_value': 35.0,
                              'pitch': math.radians(-10.0)})] * 10)
        self.assertLess(k.diagnostic['alt_angle'], -14.0)
        self.assertGreater(k.diagnostic['fov_correction'], 0.0)
        self.assertGreater(k.diagnostic['g_deg'], k.diagnostic['g_raw'])

    def test_margin_within_protection_inactive(self):
        k = self.new_value(k_fov=1.0)
        run_value2(k, [(0.0, -25.0, {'range_value': 35.0,
                              'pitch': math.radians(0.0)})] * 10)
        self.assertAlmostEqual(k.diagnostic['alt_angle'], 5.0, delta=0.2)
        self.assertAlmostEqual(k.diagnostic['fov_correction'], 0.0, places=9)
        self.assertAlmostEqual(k.diagnostic['g_deg'], k.diagnostic['g_raw'], places=9)

    def test_protection_can_disable(self):
        k = self.new_value(k_fov=0.0)
        run_value2(k, [(0.0, -15.7, {'range_value': 35.0,
                              'pitch': math.radians(11.2)})] * 10)
        self.assertAlmostEqual(k.diagnostic['fov_correction'], 0.0, places=9)

    def test_correction_clamp(self):
        k = self.new_value(k_fov=1.0, fov_correction_max=6.0)
        run_value2(k, [(0.0, -15.7, {'range_value': 35.0,
                              'pitch': math.radians(11.2)})] * 10)
        self.assertAlmostEqual(k.diagnostic['fov_correction'], -6.0, places=9)

    def test_climb_ceiling(self):
        k = self.new_value(k_fov=0.0, climb_max_mps=3.0)
        _, cmd = run_value2(k, [(0.0, -45.0, {'range_value': 35.0,
                                       'vel': [15.0, 0.0, -3.0]})] * 20)[-1]
        self.assertLessEqual(-cmd.vel_ned[2], 3.0 + 1e-9)
        # horizontal speed NOT KILLED (intentionally no vector scaling)
        self.assertGreater(math.hypot(cmd.vel_ned[0], cmd.vel_ned[1]), 10.0)

    def test_descent_not_clipped(self):
        """Ceiling applies to CLIMBING only."""
        k = self.new_value(k_fov=0.0, climb_max_mps=1.0)
        _, cmd = run_value2(k, [(0.0, 30.0, {'range_value': 35.0,
                                      'pos': [0, 0, -200.0]})] * 10)[-1]
        self.assertGreater(cmd.vel_ned[2], 1.0)


# ========================================================== 3) CLOSED LOOP

class Motor:
    """3D point-mass collision engine.

    Hunter: tracks command rate with first-order delay (frame LPF + helicopter dynamics), |v| <= 18,
    vertical |vz| ceilings are applied. Target: applies the given speed profile. Measurement: ACTUAL
    is generated from relative vector -> ex = bearing - yaw, ey = -aim_active - eps. (The geometry
    class verifies this equation with the real gimbal, which we exploit here instead of
    reconstructing it.)
    """

    def __init__(self, k, p0, v0, h0, target_speed_fn, dt=0.05, yaw0=None,
                 tau_vehicle=0.45, tau_yaw=0.30, aim=0.0, bbox_scale=900.0,
                 pitch_model=None, mount=30.0):
        self.k = k
        self.p = np.asarray(p0, float)      # hunter NED
        self.v = np.asarray(v0, float)
        self.h = np.asarray(h0, float)      # target NED
        self.target_speed_fn = target_speed_fn
        self.dt, self.tau, self.tau_yaw = dt, tau_vehicle, tau_yaw
        self.aim, self.bbox_scale = aim, bbox_scale
        d = self.h - self.p
        self.yaw = math.atan2(d[1], d[0]) if yaw0 is None else yaw0
        self.yaw_rate = 0.0
        self.t = 0.0
        self.min_range_value = float(np.linalg.norm(d))
        self.trace_samples = []
        self.fov_loss = 0
        self.raw_loss = 0
        # pitch_model : None -> pitch = mount - eps , meaning the camera axis is right on target.  alt_angle =
        # 0 and protection FOV remains NEUTRAL; to test the homing law alone. 'copter' -> MEASUREMENT rough
        # proxy: in ellipse run (20260803_173736) climbing ~ 0 while pitch median - 1.8 deg measured + 11.2
        # deg while climb ~ 4 m/s -> pitch ~= - 1.8 + 3.2 * climb_rate [ m/s ].
        self.pitch_model = pitch_model
        self.mount = float(mount)
        self.pitch_deg = 0.0

    def _pitch(self, eps_deg):
        if self.pitch_model == 'kopter':
            climb_value = max(0.0, -float(self.v[2]))
            target_value = -1.8 + 3.2 * climb_value
            a = self.dt / (self.dt + 0.5)          # attitude cycle delay
            self.pitch_deg += a * (target_value - self.pitch_deg)
        else:
            self.pitch_deg = eps_deg - self.mount   # protection neutral
        return self.pitch_deg

    def _measurement(self):
        d = self.h - self.p
        R = float(np.linalg.norm(d))
        bearing_value = math.degrees(math.atan2(d[1], d[0]))
        eps = math.degrees(math.asin(L.clamp(-d[2] / max(R, 1e-6), -1.0, 1.0)))
        ex = L.wrap180(bearing_value - math.degrees(self.yaw))
        # INVERSE of eps_solve: simulate real camera perspective tan(-ey) = tan(eps + aim) / cos(ex)
        e_acc = math.radians(eps + L.aim_active(self.aim, R))
        ey = -math.degrees(math.atan(math.tan(e_acc)
                                     / max(1e-6, math.cos(math.radians(ex)))))
        w = L.clamp(self.bbox_scale / max(R, 1.0), 2.0, 600.0)
        pitch_deg = self._pitch(eps)
        # RAW framing: the target is by alt_angle below the (mount + pitch) axis. |alt_angle| > vertical
        # half-framing -> NO detection.
        self.alt_angle = (self.mount + pitch_deg) - eps
        if abs(self.alt_angle) > L.FOV_VERTICAL_HALF_DEG:
            self.raw_loss += 1
        return Measurement(t=self.t, dt=self.dt, ex_deg=ex, ey_deg=ey,
                     bbox_w=w, bbox_h=0.6 * w,
                     area_root=math.sqrt(w * 0.6 * w),
                     coverage_pct=100.0 * w / 1280.0, bbox_age_s=0.02,
                     range_m_value=R, pos_ned=self.p.copy(), vel_ned=self.v.copy(),
                     yaw_rad=self.yaw, roll_rad=0.0,
                     pitch_rad=math.radians(pitch_deg)), ex, ey, R

    def step_value(self):
        o, ex, ey, R = self._measurement()
        if abs(ex) > L.FOV_HORIZONTAL_HALF_DEG or abs(ey) > 40.0:
            self.fov_loss += 1
        cmd = self.k.command_value(o)
        u = np.asarray(cmd.vel_ned, float)
        n = float(np.linalg.norm(u))
        if n > self.k.v_max:
            u = u * (self.k.v_max / n)
        u[2] = L.clamp(u[2], -L.WPNAV_SPEED_UP_MPS, L.WPNAV_SPEED_DN_MPS)
        a = self.dt / (self.dt + self.tau)
        self.v = self.v + a * (u - self.v)
        if cmd.yaw_rate_dps is not None:
            ay = self.dt / (self.dt + self.tau_yaw)
            self.yaw_rate += ay * (math.radians(cmd.yaw_rate_dps) - self.yaw_rate)
            self.yaw += self.yaw_rate * self.dt
        self.p = self.p + self.v * self.dt
        self.h = self.h + np.asarray(self.target_speed_fn(self.t), float) * self.dt
        self.t += self.dt
        R2 = float(np.linalg.norm(self.h - self.p))
        self.min_range_value = min(self.min_range_value, R2)
        self.trace_samples.append((self.t, R2, ex, ey, self.k.diagnostic.get('lead_az', 0.0)))
        return R2

    def run_value2(self, duration_value):
        n = int(duration_value / self.dt)
        for _ in range(n):
            if self.step_value() < 1.5:      # temas
                break
        return self.min_range_value


def _build(k=None, **kw):
    # MOUNTING FIXED TO 30 (2026-08-04, Unit.same reason as new): producing measurement with closed loop
    # harness Flight(mount=30.0); If the controller reads 0 from $YILDIZ_MOUNT, the CONTROLLER and the
    # MODEL DISCONTROL and the scenario tests this mismatch, not the control law. LOS frozen benchmark
    # artifact; The geometry to which it is valid is written clearly. SPEED ROOF FIXED TO 20 (2026-08-05,
    # same reasoning): cfg roof 20 -> 35 increased to m/s (real hardware flies with 24, sim 36.6
    # measured). The DISTINCTION of these scenarios was based on speed deficit: "pure pursuit (k_pn=0)
    # misses, PN hits" premise 35 CRASHES on m/s --copter hits target (20 m/s) If 1.75 is strict, even
    # pure pursuit 0.98 hits m, so the test becomes a measure of speed superiority, not the law. LOS
    # frozen benchmark artifact; The ENVELOPE to which it is valid is written openly. (Invariant tests
    # where the clamp tracks LIVE cfg -- those using V_MAX -- are unchanged.)
    k = k or L.LosController(aim_deg=0.0, mount_deg=30.0, v_max_mps=20.0)
    k.seed_value2(None)
    return k


class ClosedLoop(unittest.TestCase):
    """Synthetic conflict scenarios. NO Sim/SITL, pure kinematics."""

    def handoff_state(self, back=25.0, down=13.0):
        """moment of handoff: fighter 25 m behind the target, 13 m below, at target speed."""
        target_value = np.array([0.0, 0.0, -100.0])
        pursuer_value = target_value - np.array([back, 0.0, -down])
        return pursuer_value, target_value

    def test_straight_target_stable_remains(self):
        """Target 20 m/s is flat. Copter 18 m/s -> TAIL CHASE CANNOT BE WIN (physics). Expected: law DOES NOT
DIVERTISE, vanguard remains reasonable, target remains at FOV, range does not explode uncontrolled."""
        pursuer_value, target_value = self.handoff_state()
        k = _build()
        m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                  lambda t: (20.0, 0.0, 0.0))
        m.run_value2(30.0)
        self.assertLess(m.fov_loss, 60, "the target is out of frame")
        self.assertLess(abs(k.diagnostic['lead_az']), 15.0, "the lead angle diverged")
        R_last = m.trace_samples[-1][1]
        self.assertLess(R_last, 200.0, f"range opened without control: {R_last:.0f}")

    def test_turning_target_captured(self):
        """The target is circling with a constant rate of rotation (the essence of the ellipse route). LOS/PN
adds the lead angle required for contact. All three rates of rotation are tested: the law must work in the
range, not just at one point."""
        for turn_dps, duration_value in ((6.0, 60.0), (9.0, 50.0), (14.0, 40.0)):
            with self.subTest(turn=turn_dps):
                pursuer_value, target_value = self.handoff_state()
                k = _build()
                w = math.radians(turn_dps)
                m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                          lambda t: (20.0 * math.cos(w * t),
                                     20.0 * math.sin(w * t), 0.0))
                mn = m.run_value2(duration_value)
                self.assertLess(mn, 3.0,
                                f"{turn_dps} deg/s closest range in rotation "
                                f"{mn:.1f} m")

    def test_pn_pure_than_pursuit_good(self):
        """k_pn=0 (no leader = PURE FOLLOW) vs. k_pn=1 (PN) in the SAME scenario. A target returning without
leadership will NOT be caught."""
        w = math.radians(9.0)
        target_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        result_value = {}
        for kp in (0.0, 1.0):
            pursuer_value, target_value = self.handoff_state()
            k = _build(L.LosController(aim_deg=0.0, k_pn=kp,
                                    mount_deg=30.0, v_max_mps=20.0))
            m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value, target_fn)
            result_value[kp] = m.run_value2(50.0)
        self.assertLess(result_value[1.0], 3.0, f"PN vurmadi: {result_value}")
        self.assertGreater(result_value[0.0], 15.0, f"pure pursuit unexpected: {result_value}")

    def test_lead_clamp_speed_deficit_when_present_critical(self):
        """lead_az_max is only binding WHEN THE HUNTER IS SLOWER THAN THE TARGET.

        Since the collision triangle is sin(d) = (V_t/V_p) sin(theta_t), when V_p < V_t, the
        solution requires a leader up to 80; tight handcuffs send the law into a tail chase. This
        test CLEARLY establishes the speed deficit (v_max=18 < target 20) -- even if guidance_config
        is increased to 20, it is possible for the deficit to re-occur in flight (in climb the
        horizontal component drops to V*cos(gamma), WPNAV_SPEED_UP 10 m/s also trims the vertical),
        so the rationale is preserved as a REGRESSION WATCH.

        Measured (circle, 60 s): clamp 60 deg -> 28.2 / 22.4 / 15.0 m ABUSE, 85 deg -> 3.2 / 1.4 /
        0.9 m HIT (return 3 / 6 / 9 deg/s).
        """
        for turn_dps in (3.0, 6.0, 9.0):
            with self.subTest(turn=turn_dps):
                w = math.radians(turn_dps)
                target_fn = (lambda t: (20.0 * math.cos(w * t),
                                       20.0 * math.sin(w * t), 0.0))
                result_value = {}
                for omax in (60.0, 85.0):
                    pursuer_value, target_value = self.handoff_state()
                    k = _build(L.LosController(aim_deg=0.0, v_max_mps=18.0,
                                            lead_az_max_deg=omax))
                    m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                              target_fn)
                    result_value[omax] = m.run_value2(60.0)
                self.assertGreater(result_value[60.0], 8.0,
                                   f"narrow clamp unexpectedly good: {result_value}")
                self.assertLess(result_value[85.0], 4.0,
                                f"wide clamp did not hit: {result_value}")

    def test_speed_at_parity_clamp_binding_not(self):
        """With guidance_config VISUAL_MAX_SPEED_MPS = 20 (2026-08-03) the fighter is at the SAME speed as
the target; The collision triangle is now resolved with the small leader and the clamp is no longer
binding. Documentation of the new truth: HIT is expected even with tight handcuffs."""
        self.assertGreaterEqual(V_MAX, 20.0,
                                "cfg.VISUAL_MAX_SPEED_MPS rolled back")
        for turn_dps in (4.0, 9.0, 20.0):
            with self.subTest(turn=turn_dps):
                w = math.radians(turn_dps)
                target_fn = (lambda t: (20.0 * math.cos(w * t),
                                       20.0 * math.sin(w * t), 0.0))
                for omax in (60.0, 85.0):
                    pursuer_value, target_value = self.handoff_state()
                    k = _build(L.LosController(aim_deg=0.0, lead_az_max_deg=omax))
                    m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                              target_fn)
                    mn = m.run_value2(60.0)
                    self.assertLess(mn, 3.0,
                                    f"turn={turn_dps} clamp={omax} "
                                    f"closest range {mn:.1f} m")

    def test_range_ceiling_critical(self):
        """v_max = VISUAL_MAX_SPEED_MPS. 18 -> target (20 m/s) is only captured with very wide lead; 20 ->
catches even with narrow lead. It is proof of guidance_config being upgraded from 18 -> 20
(2026-08-03); If the value is rolled back, this test shows the loss again."""
        w = math.radians(6.0)
        target_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        result_value = {}
        for vm in (18.0, 20.0):
            pursuer_value, target_value = self.handoff_state()
            k = _build(L.LosController(aim_deg=0.0, v_max_mps=vm,
                                    lead_az_max_deg=60.0))
            m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value, target_fn)
            result_value[vm] = m.run_value2(60.0)
        print(f"\n [info] closest range in narrow lead (60 deg): "
              f"v_max=18 -> {result_value[18.0]:.1f} m, v_max=20 -> {result_value[20.0]:.1f} m")
        self.assertLess(result_value[20.0], result_value[18.0])

    def test_legacy_with_sign_comparison(self):
        """legacy1's REVERSE signed derivative term (d_point = -K*d - (Kd+1)*qdot) should be significantly
worse in the returned target. We imitate that sign by giving k_pn < 0."""
        w = math.radians(9.0)
        target_fn = (lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        result_value = {}
        for label_value, kp in (('legacy_reverse', -1.0), ('pn', 1.0)):
            pursuer_value, target_value = self.handoff_state()
            k = _build(L.LosController(aim_deg=0.0, k_pn=kp,
                                    mount_deg=30.0, v_max_mps=20.0))
            m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value, target_fn)
            result_value[label_value] = m.run_value2(50.0)
        self.assertLess(result_value['pn'], result_value['legacy_reverse'],
                        f"sign fix yielded no gain: {result_value}")

    def test_static_target_hit(self):
        """User's '2loskf static target always hit' reference criterion: our law should also hit (pure tracking
is already enough, PN should not break it)."""
        pursuer_value, target_value = self.handoff_state()
        k = _build()
        m = Motor(k, pursuer_value, np.array([5.0, 0.0, 0.0]), target_value, lambda t: (0, 0, 0))
        mn = m.run_value2(30.0)
        self.assertLess(mn, 2.0, f"closest range on static target {mn:.2f} m")

    def test_crossing_target_hit(self):
        """The target crosses 90 degrees -- LOS is the highest rate.

        GEOMETRY CAPTUREABLE SELECTED: since the copter is 18 m/s, the target is 20 m/s, the
        collision triangle (|target(T) - pursuer_0| = 18 T) must have roots. Here there is the range T
        ~ [3.6, 24] s, so there is room for the law to recover. In the same scenario, WITHOUT
        PRELIMINARY (k_pn=0) the law misses -- this proves the distinctiveness of the test."""
        target_value = np.array([0.0, 0.0, -100.0])
        pursuer_value = np.array([-50.0, 60.0, -92.0])
        v0 = np.array([16.0, 4.0, 0.0])
        target_fn = (lambda t: (0.0, 20.0, 0.0))
        k = _build()
        mn = Motor(k, pursuer_value.copy(), v0.copy(), target_value.copy(), target_fn).run_value2(40.0)
        # 8 m threshold: climbing roof (3 m/s, section 3b-b) deliberately slows down vertical closing; Before
        # 2026-08-03 (without ceiling) the value was 2.8 m. Cost conscious: loss of lock loses everything.
        self.assertLess(mn, 8.0, f"closest range at cross target {mn:.1f} m")
        k0 = _build(L.LosController(aim_deg=0.0, k_pn=0.0,
                                 mount_deg=30.0, v_max_mps=20.0))
        mn0 = Motor(k0, pursuer_value.copy(), v0.copy(), target_value.copy(), target_fn).run_value2(40.0)
        self.assertGreater(mn0, 10.0,
                           f"pure pursuit also hit, the test is not distinctive ({mn0:.1f})")

    def test_crossing_escaping_geometry_no_solution(self):
        """For documentation purposes: the collision triangle has NO ROOT on the side of the target AWAY from
the fighter (76T^2 + 1000T + 2369 = 0, both roots are negative). No guiding law can catch up; The
only thing expected is that the law does not explode."""
        target_value = np.array([0.0, 0.0, -100.0])
        pursuer_value = np.array([-40.0, -25.0, -88.0])
        k = _build()
        m = Motor(k, pursuer_value, np.array([14.0, 8.0, 0.0]), target_value,
                  lambda t: (0.0, 20.0, 0.0))
        m.run_value2(30.0)
        self.assertTrue(np.all(np.isfinite(m.v)))
        self.assertLessEqual(abs(k.diagnostic['lead_az']), k.lead_az_max + 1e-9)

    def test_yaw_oscillation_absent(self):
        """The yaw command should not oscillate by changing sign (past quiver mode). In a fixed rotating
target, the number of sign changes should be limited."""
        pursuer_value, target_value = self.handoff_state()
        k = _build()
        w = math.radians(9.0)
        yaw_record = []
        m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                  lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t), 0.0))
        for _ in range(int(40.0 / m.dt)):
            if m.step_value() < 1.5:
                break
            yaw_record.append(k.diagnostic['yaw_rate'])
        sign_value = [1 if y > 1.0 else (-1 if y < -1.0 else 0) for y in yaw_record]
        change = sum(1 for a, b in zip(sign_value, sign_value[1:])
                      if a != 0 and b != 0 and a != b)
        self.assertLess(change, 12, f"yaw sign change {change} (flicker)")

    def test_fov_protection_raw_in_frame_maintains(self):
        """CLOSED LOOP PROOF (part 3b): with copter pitch surrogate, target moves out of raw frame WHEN guard
is OFF; When ON it stays in the frame.

        pitch surrogate fitted to measurement of ellipse run (pitch ~= -1.8 + 3.2 * climb_rate);
        detail at Motor._pitch."""
        result_value = {}
        for label_value, kw in (('disabled', dict(k_fov=0.0, climb_max_mps=99.0)),
                           ('enabled_value', dict(k_fov=1.0, climb_max_mps=3.0))):
            # Geometry of the ACTUAL run: drifting 41.8 m behind the positioned target (target 20 m/s, copter roof
            # 20 m/s), ascent atan(11/41.8) = 14.8 deg -- the median of eps measured was 14.9.
            pursuer_value, target_value = self.handoff_state(back=41.8, down=11.0)
            # mount_deg ON: the default of _setup does not take effect because the controller is already
            # installed; The engine also uses mount=30, if it separates, the test measures the non-compliance, not
            # the law.
            k = _build(L.LosController(aim_deg=0.0, mount_deg=30.0,
                                    v_max_mps=20.0, **kw))
            m = Motor(k, pursuer_value, np.array([19.0, 0.0, 0.0]), target_value,
                      lambda t: (20.0, 0.0, 0.0), pitch_model='kopter')
            m.run_value2(25.0)
            n = len(m.trace_samples)
            result_value[label_value] = (m.raw_loss, n, m.raw_loss / max(n, 1))
        print(f"\n [info] raw off-frame sample rate: "
              f"protection off %{100*result_value['disabled'][2]:.0f}, "
              f"on %{100*result_value['enabled_value'][2]:.0f}")
        # Measured: guard off %24, on open %2 (fold recovery 10).
        self.assertGreater(result_value['disabled'][2], 0.20,
                           f"loss expected with protection off: {result_value}")
        self.assertLess(result_value['enabled_value'][2], 0.05,
                        f"target must remain in frame when guard is on: {result_value}")
        self.assertLess(result_value['enabled_value'][2], 0.25 * result_value['disabled'][2])

    def test_none_command_nan_not(self):
        pursuer_value, target_value = self.handoff_state()
        k = _build()
        w = math.radians(20.0)
        m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                  lambda t: (20.0 * math.cos(w * t), 20.0 * math.sin(w * t),
                             3.0 * math.sin(0.7 * t)))
        for _ in range(int(40.0 / m.dt)):
            if m.step_value() < 1.5:
                break
            self.assertTrue(np.all(np.isfinite(m.v)))
            self.assertLessEqual(np.linalg.norm(m.v), k.v_max + 1e-6)

    def test_vertical_ratio_tail_pursuit(self):
        """--vertical-ratio < 1 should maintain range better in a flat (unwinnable) scenario: does not waste angle
advantage. Evidence of the button in the report."""
        result_value = {}
        for kd in (1.0, 0.4):
            pursuer_value, target_value = self.handoff_state()
            k = _build(L.LosController(aim_deg=0.0, vertical_ratio=kd))
            m = Motor(k, pursuer_value, np.array([17.0, 0.0, 0.0]), target_value,
                      lambda t: (20.0, 0.0, 0.0))
            m.run_value2(30.0)
            result_value[kd] = m.trace_samples[-1][1]
        # For informational purposes; We do not set hard thresholds (scenario sensitive).
        print(f"\n [information] straight route 30 Range at the end of s: "
              f"k_perpendicular=1.0 -> {result_value[1.0]:.1f} m, k_perpendicular=0.4 -> {result_value[0.4]:.1f} m")
        self.assertTrue(all(math.isfinite(v) for v in result_value.values()))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + sys.argv[1:], verbosity=2)
