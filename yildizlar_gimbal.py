#!/usr/bin/env python3
"""
yildizlar_gimbal.py - camera layer that mathematically cleans up body oscillation
============================================================================= Moved EXACTLY from
bumblebee/teva.py (stabilize_pixel, lines 841-864); The only thing that changes are the camera
parameters and the mounting angle. Reason: no physical gimbal, camera FIXED to the body. As the
copter/aircraft tilts, the target's position in the frame shifts, even if it does not actually move
at all; My instinct thought this shift was a target movement and went after it.

Transform chain, in the same order as teva.py : r_cam = K^- 1 @ [ px , py, 1 ], converting raw
pixels to a camera ray.  r_body = R_mount @ ( R_c_b @ r_cam ), applying physical mounting angle, +
30 upward, before de-rotation because the camera tilts with the body.  r_stab = R_stab @ r_body
removes body roll / pitch with yaw = 0 .  r_virt = R_aim @ r_stab applies the AIM after offset
de-rotation because it describes desired elevation relative to the horizon. Applying AIM with the
physical mounting transform previously introduced false horizontal error delta* sin ( roll ) during
banking. pixel = K @ ( R_c_b ^T @ r_virt )

TWO DISTINCT QUANTITIES (the key distinction in teva.py):
mount_phys_pitch_deg is the physical mounting angle. It is +30 degrees in this environment
(models/swarm_drone_*/model.sdf, sensor pose pitch), confirmed by measurement:
target elevation +26.26 degrees with bbox y=534 implies camera-axis elevation +29.4 degrees.
In the teva.py simulation it was 0.0 because the camera was parallel to the body.
aim_pitch_deg is the DC offset specifying the desired target location in the frame.
The virtual gimbal removes AC motion from body oscillation; this DC component is supplied manually.
RELATION (as in teva.py:709-714): virtual frame
      center = -aim relative to the horizon, i.e. aim to keep the target at eps degrees relative to
      the horizon = -eps. Geometry standoff of position guidance determines eps (back/down), so this
      value depends on the approach geometry and NOT on the PLAN.

AIM RANGE TERMINATION (teva.py:815-822): applied FULL when the current is close, decreasing linearly
to zero at a distance. Applying full angle at a distance caused the small angle difference to
require a large altitude difference and the command to become saturated.

WORK OF THE AIM ONLY VERTICAL CHANNEL (measured and corrected in 2026-08-02): R_aim is a Ry return,
meaning it also returns the horizontal sting. At the small side angle (psi) limit, the horizontal
angle read in the virtual frame is the actual bearing COMPRESSED by gain = cos(eps) / cos(eps + aim)
(eps = elevation of the target relative to the horizon). gain 1.000 at aim=0; At the nominal
operating point (at the target virtual centre, current = -eps) it decreases to a minimum, cos(eps).
In the live data (run/evidence/gimbal4.csv, aim=-27.47, eps~24) the measured gain was 0.909: the
guidance bearing was reading %8.8 LOW, meaning return to target was constantly undercommanded. AIM's
job is vertical DC offset; it has no use in the horizontal channel. Therefore, angle_error_value() takes
the horizontal component from the stabilization job BEFORE AIM IS APPLIED and the vertical component
from the post-aim is. The vertical channel is unchanged; In aim=0 the two paths are numerically
identical.
"""

import argparse
import math

import numpy as np

# Camera -> body axis transformation (teva.py:121). Camera: x right, y down, z forward (OpenCV). Body:
# x forward, y right, z down (NED).
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T


def compute_R_b_e(roll, pitch, yaw):
    """Body -> ground ( NED ) orientation matrix ( teva.py : 126 )."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _ry(deg):
    """Rotation around the Y (pitch) axis. POSITIVE = turns its work UP."""
    r = math.radians(deg)
    return np.array([[math.cos(r), 0.0, math.sin(r)],
                     [0.0,         1.0, 0.0],
                     [-math.sin(r), 0.0, math.cos(r)]])


def joint_angle(eps_cam_deg, pitch_rad, roll_rad):
    """PHYSICAL GIMBAL (gimbal branch): subtracts the articulation angle from the camera's world elevation.

    The stabilize gimbal plugin keeps the camera axis at world elevation eps; What the measurement
    chain needs is the angle of the camera relative to the BODY (the live equivalent of the old
    'mount' concept). World elevation since the camera axis is rotated with _ry(q) on the body:

        eps = asin( sin(pitch)*cos(q) + cos(pitch)*cos(roll)*sin(q) )

    Inverse solution (A = sin ( pitch ), B = cos ( pitch )* cos ( roll )):
        q = asin( sin(eps) / sqrt(A^2+B^2) ) - atan2(A, B)

    eps_cam_deg: camera world elevation [deg] (gimbal_tilt_status or commanded value in slow DC
    regime). Return value: q [deg], positive=up. At Roll=0 it simplifies to q = eps - pitch.
    """
    A = math.sin(pitch_rad)
    B = math.cos(pitch_rad) * math.cos(roll_rad)
    R = math.hypot(A, B)
    s = math.sin(math.radians(eps_cam_deg)) / max(R, 1e-9)
    s = max(-1.0, min(1.0, s))
    return math.degrees(math.asin(s) - math.atan2(A, B))


class VirtualGimbal:
    """Converts the raw bbox pixel to a pixel free of body oscillation."""

    def __init__(self, width=1280, height=720, hfov_rad=1.1519,
                 mount_phys_pitch_deg=0.0, aim_pitch_deg=0.0,
                 aim_full_range_m=120.0, aim_zero_range_m=250.0,
                 cx=None, cy=None):
        self.width, self.height = width, height
        self.hfov_rad = hfov_rad
        # Raspberry Pi AI Camera (IMX500) 1280x720: hfov 66 derece
        # -> fx = fy = (1280/2)/tan(33 deg) = 985.5
        self.fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = self.fx                      # square pixel
        self.cx = width / 2.0 if cx is None else cx
        self.cy = height / 2.0 if cy is None else cy

        self.mount_phys_pitch_deg = float(mount_phys_pitch_deg)
        self.aim_pitch_deg = float(aim_pitch_deg)
        self.aim_full_range_m = float(aim_full_range_m)
        self.aim_zero_range_m = float(aim_zero_range_m)

        self.K = np.array([[self.fx, 0.0, self.cx],
                           [0.0, self.fy, self.cy],
                           [0.0, 0.0, 1.0]])
        self.K_inv = np.linalg.inv(self.K)
        self.R_mount_phys = _ry(self.mount_phys_pitch_deg)
        self._last_aim_deg = None
        self.R_aim = _ry(self.aim_pitch_deg)

        self.vfov_rad = 2 * math.atan(math.tan(hfov_rad / 2) * height / width)

    # ---------------------------------------------------------------- aim

    def aim_active_deg(self, range_m_value):
        """Range-aware ammo (teva.py:815-822).

        If the range is None, full current is applied (if there is no range information, there is
        nothing to absorb).
        """
        if range_m_value is None:
            return self.aim_pitch_deg
        r = float(range_m_value)
        if r <= self.aim_full_range_m:
            k = 1.0
        elif r >= self.aim_zero_range_m:
            k = 0.0
        else:
            k = ((self.aim_zero_range_m - r)
                 / (self.aim_zero_range_m - self.aim_full_range_m))
        return self.aim_pitch_deg * k

    # -------------------------------------------------------------- forward

    def _R_camera_body(self, joint_deg):
        """Camera->body rotation: LIVE articulation angle (joint_deg, positive=up) if physical gimbal is
present, fixed mount otherwise. The fixed mount path is EXACTLY the same as the old behavior
(backfit)."""
        if joint_deg is None:
            return self.R_mount_phys
        return _ry(float(joint_deg))

    def pixel_generate(self, elevation_deg, lateral_deg, roll_rad, pitch_rad,
                    joint_deg=None):
        """REVERSE direction: where does the target fall in the RAW frame in the (ascending, lateral) direction
relative to the horizon? For verification/testing only; It is not used on the flight path.
        """
        eps, psi = math.radians(elevation_deg), math.radians(lateral_deg)
        r_stab = np.array([math.cos(eps) * math.cos(psi),
                           math.cos(eps) * math.sin(psi),
                           -math.sin(eps)])
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0)
        r_body = R_stab.T @ r_stab
        r_cam_body = self._R_camera_body(joint_deg).T @ r_body
        r_cam = R_c_b_T @ r_cam_body
        if r_cam[2] <= 1e-9:
            return None                     # BEHIND THE CAMERA
        p = self.K @ r_cam
        return p[0] / p[2], p[1] / p[2]

    def in_frame_mi(self, px, py):
        return px is not None and 0 <= px < self.width and 0 <= py < self.height

    # ------------------------------------------------------------------------------- stable

    def stabilize(self, px, py, roll_rad, pitch_rad, range_m_value=None,
                  joint_deg=None):
        """Raw pixel -> virtual (horizon aligned) pixel.  teva.py : 841 - 864 .

        If joint_deg is supplied (physical gimbal) live joint angle is used instead of fixed mount;
        R_stab remains SAME because R_stab @ _ry(joint) is exactly the real-world orientation of the
        camera (including roll de-rotation).
        """
        r_cam = self.K_inv @ np.array([px, py, 1.0])
        r_body = self._R_camera_body(joint_deg) @ (R_c_b @ r_cam)  # BEFORE de-rot
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0)

        aim_e = self.aim_active_deg(range_m_value)
        if aim_e != self._last_aim_deg:
            self._last_aim_deg = aim_e
            self.R_aim = _ry(aim_e)
        r_virt_body = self.R_aim @ (R_stab @ r_body)     # aim: AFTER de-rot
        r_virt_cam = R_c_b_T @ r_virt_body
        p = self.K @ r_virt_cam
        if p[2] == 0:
            return px, py
        return p[0] / p[2], p[1] / p[2]

    def angle_error_value(self, px, py, roll_rad, pitch_rad, range_m_value=None,
                   joint_deg=None):
        """Off-center (degrees) in the virtual frame. The angle-error measurement consumed by guidance.

        It is read from the job AFTER VERTICAL STRIP, BEFORE HORIZONTAL STRIP - justification is in
        the module docstring (since the current is a Ry rotation, it compresses the horizontal angle
        by cos(eps)/cos(eps+aim); measured loss %8.8).
        """
        # VERTICAL: from current applied work (DC offset is intentional).
        sx, sy = self.stabilize(px, py, roll_rad, pitch_rad, range_m_value,
                                joint_deg=joint_deg)
        ey = math.degrees(math.atan((sy - self.cy) / self.fy))
        # HORIZONTAL: from the BEFORE opening job. The first two steps of the chain are exactly the same as
        # stabilize(); only R_aim does not apply. The bearing is read directly from the components of the
        # body-horizontal work (atan2: gives the correct face also in off-frame/wide angle).
        r_cam = self.K_inv @ np.array([px, py, 1.0])
        r_body = self._R_camera_body(joint_deg) @ (R_c_b @ r_cam)
        r_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0) @ r_body
        ex = math.degrees(math.atan2(r_stab[1], r_stab[0]))
        return ex, ey

    def range_prediction(self, bbox_w_px, target_width_m=1.6):
        """Coarse range (m) from width bbox. For AIM FINISHING only.

        range = target_genisligi * fx / bbox_genisligi The apparent width of the target was taken by
        measurement 1.6 m (7 px @ 56 px/degree -> 1.57 m at 120 m). Rough enough: the damping
        120-250 is a linear ramp between m, a few meters of error only slides on the ramp. Guidance
        DOES NOT get range from here; it comes from telemetry.
        """
        if bbox_w_px is None or bbox_w_px < 1:
            return None
        return target_width_m * self.fx / float(bbox_w_px)

    def summary_value(self):
        return (f"VirtualGimbal {self.width}x{self.height} "
                f"hfov={math.degrees(self.hfov_rad):.1f} "
                f"vfov={math.degrees(self.vfov_rad):.1f} deg | "
                f"fx=fy={self.fx:.1f} cx={self.cx:.0f} cy={self.cy:.0f} | "
                f"mount_phys={self.mount_phys_pitch_deg:+.2f} "
                f"aim={self.aim_pitch_deg:+.2f} "
                f"(full<{self.aim_full_range_m:.0f}m, zero>{self.aim_zero_range_m:.0f}m)")


def analytical_aim(back_m, down_m):
    """(a) option: calculate am from YOUR OWN instruction geometry, no measurement required.

    In standoff tracking, the rise of the target relative to the horizon is entirely the slot
    geometry you provide: eps = atan(down/back). Since the virtual frame center is -aim, aim = -eps.
    Verified by measurement (back=25, down=13 -> -27.47): ellipse -29.25, flat -27.03. (wanderer was
    -10.52; since that plan changes altitude the slot geometry is not preserved - so trim is
    needed.)
    """
    return -math.degrees(math.atan(float(down_m) / max(1e-6, float(back_m))))


class AimTrim:
    """SLOWLY smoothes the aim in a SMALL band from the analytical start.

    WHY IT IS THERE: analytical value (a) knows your command geometry, but not the ACTUAL mounting
    error of the aircraft (~-1 degrees camera-autopilot angle measured at teva.py), angle of attack,
    and slot tracking error. These are DC quantities; The virtual gimbal clears the ON (body sway),
    intentionally leaving the DC in hand.

    WHY SLOW AND CLAMPED: setting up a fast loop SIMPLIFIES the virtual gimbal - if the feed starts
    chasing the target, the "virtual center of frame" is no longer a fixed reference, the DC/ON
    distinction collapses and what you're left with is a stealth tracking loop. Four protections
    prevent this:

      1. CLAMP: current can only travel within +-clamp_deg of the analytical value. Swallows
      assembly/AOA error, cannot chase the target. 2. SPEED LIMIT: max_speed_dps degrees/second. It
      takes minutes to get through the clamp -> it can't mix with the target dynamic. 3. RANGE GATE:
      update only in the near band where AIM is FULLY applied. ey is meaningless since the remote
      current is already absorbed. 4. OUTLIER VALUE REJECTION: |ey| If it is large (at the edge of
      the target frame, possibly partially detected) the sample is discarded.
    """

    def __init__(self, start_deg, clamp_deg=6.0, tau_s=25.0,
                 max_speed_dps=0.15, range_max_m=150.0, ey_max_deg=12.0,
                 min_sample=30):
        self.start_value3 = float(start_deg)
        self.aim = float(start_deg)
        self.clamp = float(clamp_deg)
        self.tau = float(tau_s)
        self.max_speed_value = float(max_speed_dps)
        self.range_max = float(range_max_m)
        self.ey_max = float(ey_max_deg)
        self.min_sample = int(min_sample)
        self.sample_value = 0
        self.at_clamp = False

    def update_value(self, ey_deg, range_m_value, dt):
        """ey_deg: VERTICAL angle error in virtual framing. Returns the new aim offset.

        ey<0 = ABOVE target center. To lower it to the center, it is necessary to move the virtual
        center up, that is, DECREASE the aim (center = -aim).
        """
        if ey_deg is None or dt <= 0:
            return self.aim
        if abs(ey_deg) > self.ey_max:
            return self.aim                      # opposite: frame edge
        if range_m_value is not None and range_m_value > self.range_max:
            return self.aim                      # far: the aim is already finite
        self.sample_value += 1
        if self.sample_value < self.min_sample:
            return self.aim                      # Gather enough evidence first

        # First-order response: filter the aim offset with time constant tau in the direction that takes ey to zero.
        requested_value = self.aim + ey_deg
        step_value = (requested_value - self.aim) * (dt / max(self.tau, 1e-3))
        limit_value = self.max_speed_value * dt
        step_value = max(-limit_value, min(limit_value, step_value))
        new_value = self.aim + step_value
        alt, upper_value = self.start_value3 - self.clamp, self.start_value3 + self.clamp
        self.at_clamp = not (alt < new_value < upper_value)
        self.aim = max(alt, min(upper_value, new_value))
        return self.aim

    def summary_value(self):
        return (f"AimTrim: start={self.start_value3:+.2f} "
                f"currently= {self.aim:+.2f} (clamp +- {self.clamp:.1f} , "
                f"tau={self.tau:.0f}s, max {self.max_speed_value:.2f} deg/s, "
                f"range<{self.range_max:.0f}m)"
                f"{'  [AT_CLAMP]' if self.at_clamp else ''}")


# ====================================================================== test

def static_test(g, verbose=True):
    """It confirms the ACTUAL PROMISE of the virtual gimbal:

    While the target remains FIXED relative to the horizon, RAW pixels shift as the body tilts, but
    VIRTUAL pixels should not shift. This is a closed verification because the forward model
    (pixel_generate) and the reverse model (stabilized) are opposites of each other.
    """
    print(g.summary_value())
    print()
    print("--- STATIC TEST: target stationary, body oscillating ---")
    successful = True
    for elevation_value, lateral in ((30.0, 0.0), (25.0, +8.0), (35.0, -8.0), (30.0, +15.0)):
        raw_value, virtual = [], []
        for roll_d in (-30, -15, 0, 15, 30):
            for pitch_d in (-20, -10, 0, 10, 20):
                p = g.pixel_generate(elevation_value, lateral, math.radians(roll_d), math.radians(pitch_d))
                if p is None or not g.in_frame_mi(*p):
                    continue
                raw_value.append(p)
                virtual.append(g.stabilize(p[0], p[1], math.radians(roll_d),
                                         math.radians(pitch_d), range_m_value=None))
        if len(virtual) < 4:
            print(f"  rise={elevation_value:+5.1f} side={lateral:+5.1f}: not enough samples "
                  f"( {len(virtual)} ) - off frame")
            continue
        raw_value = np.array(raw_value); virtual = np.array(virtual)
        raw_spread = float(np.hypot(*(raw_value.max(axis=0) - raw_value.min(axis=0))))
        virtual_spread = float(np.hypot(*(virtual.max(axis=0) - virtual.min(axis=0))))
        # EXPECTED VIRTUAL POSITION: the center of the virtual frame is at -aim relative to the horizon, NOT
        # at the mounting angle (the mount only determines which target is physically in the frame). In other
        # words, the virtual image is the raw image of a camera that is mounted and never tilted.
        bekl = VirtualGimbal(g.width, g.height, g.hfov_rad,
                           mount_phys_pitch_deg=-g.aim_pitch_deg
                           ).pixel_generate(elevation_value, lateral, 0.0, 0.0)
        deviation = float(np.hypot(*(virtual.mean(axis=0) - np.array(bekl))))
        ok = virtual_spread < 0.5 and deviation < 0.5
        successful &= ok
        print(f"  up= {elevation_value:+5.1f} side= {lateral:+5.1f} ( {len(virtual):2d} attitude): "
              f"RAW spread {raw_spread:7.1f} px -> VIRTUAL {virtual_spread:5.2f} px , "
              f"deviation from expected {deviation:.2f} px {'OK' if ok else 'ERROR'}")
    print()
    print("--- PHYSICAL GIMBAL: JOINT compensates as the torso swings ---")
    # gimbal branch (2026-08-05): stabilized plugin holds camera axis at world elevation eps_cmd -> joint
    # q = joint_angle(eps_cmd, pitch, roll). When live q is given to the chain instead of fixed mount, the
    # virtual pixel should still not move. Also q should simplify to (eps - pitch) at roll=0.
    g_fiz = VirtualGimbal(g.width, g.height, g.hfov_rad,
                        mount_phys_pitch_deg=0.0, aim_pitch_deg=0.0)
    eps_cmd = 10.0
    for elevation_value, lateral in ((10.0, 0.0), (5.0, +8.0), (15.0, -8.0)):
        virtual = []
        for roll_d in (-30, -15, 0, 15, 30):
            for pitch_d in (-20, -10, 0, 10, 20):
                q = joint_angle(eps_cmd, math.radians(pitch_d),
                                math.radians(roll_d))
                p = g_fiz.pixel_generate(elevation_value, lateral, math.radians(roll_d),
                                      math.radians(pitch_d), joint_deg=q)
                if p is None or not g_fiz.in_frame_mi(*p):
                    continue
                virtual.append(g_fiz.stabilize(p[0], p[1], math.radians(roll_d),
                                             math.radians(pitch_d),
                                             range_m_value=None, joint_deg=q))
        virtual = np.array(virtual)
        bekl = VirtualGimbal(g.width, g.height, g.hfov_rad,
                           mount_phys_pitch_deg=0.0).pixel_generate(elevation_value, lateral, 0.0, 0.0)
        spread = float(np.hypot(*(virtual.max(axis=0) - virtual.min(axis=0))))
        deviation = float(np.hypot(*(virtual.mean(axis=0) - np.array(bekl))))
        ok = spread < 0.5 and deviation < 0.5
        successful &= ok
        print(f"  elevation= {elevation_value:+5.1f} side= {lateral:+5.1f} ( {len(virtual):2d} attitude, "
              f"eps_cmd = {eps_cmd:+.1f} ): VIRTUAL propagation {spread:5.2f} px , "
              f"deviation {deviation:.2f} px {'OK' if ok else 'ERROR'}")
    q0 = joint_angle(eps_cmd, math.radians(7.0), 0.0)
    simplification_ok = abs(q0 - (eps_cmd - 7.0)) < 1e-9
    successful &= simplification_ok
    print(f"  roll=0 simplification: q({eps_cmd:.0f}, pitch=7) = {q0:.6f} "
          f"(expected {eps_cmd-7.0:.1f}) {'OK' if simplification_ok else 'ERROR'}")

    print()
    print("--- AIM OFFSET: where is the virtual framing center relative to the horizon? ---")
    for aim in (0.0, -10.0, -25.0, -30.0):
        g2 = VirtualGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                         aim_pitch_deg=aim)
        # Look for the rise falling in the center
        best_candidate, minimum_magnitude = None, 1e9
        for eps10 in range(-600, 601):
            eps = eps10 / 10.0
            p = g2.pixel_generate(eps, 0.0, 0.0, 0.0)
            if p is None:
                continue
            s = g2.stabilize(p[0], p[1], 0.0, 0.0, range_m_value=None)
            d = abs(s[1] - g2.cy)
            if d < minimum_magnitude:
                minimum_magnitude, best_candidate = d, eps
        print(f"  aim= {aim:+6.1f} -> virtual center elevation {best_candidate:+6.1f} deg "
              f"(relation: -aim = {-aim:+6.1f})")
    print()
    print("--- AIM HORIZONTAL GAIN: AIM MUST NOT LEAK into the horizontal channel ---")
    # R_aim is a Ry spin; In the past, the horizontal angle was also read from the frame AFTER the angle
    # and the bearing was stuck as much as cos(eps)/cos(eps+aim). In the live data (gimbal4.csv:
    # aim=-27.47, eps~24) this loss was measured as 0.909 - the guidance bearing was seeing 8.8% low.
    # After correction the gain should be 1.000 regardless of the aim offset and no matter how the body
    # is tilted.
    lateral_ref = 3.0
    gain_ok = True
    for aim, eps in ((0.0, 30.0), (-10.0, 30.0), (-27.47, 24.0),
                     (-27.47, 30.0), (-30.0, 35.0), (-20.0, 20.0)):
        g4 = VirtualGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                         aim_pitch_deg=aim)
        ratios = []
        for roll_d in (-20, 0, 20):
            for pitch_d in (-10, 0, 10):
                p = g4.pixel_generate(eps, lateral_ref, math.radians(roll_d),
                                   math.radians(pitch_d))
                if p is None or not g4.in_frame_mi(*p):
                    continue
                ex, _ = g4.angle_error_value(p[0], p[1], math.radians(roll_d),
                                      math.radians(pitch_d), range_m_value=None)
                ratios.append(ex / lateral_ref)
        if not ratios:
            print(f"  aim= {aim:+6.2f} eps = {eps:+5.1f} : out of frame - skipped")
            continue
        worst_value = max(ratios, key=lambda k: abs(k - 1.0))
        # Gain that the code will give BEFORE correction (small psi limit):
        previous = math.cos(math.radians(eps)) / math.cos(math.radians(eps + aim))
        ok = abs(worst_value - 1.0) < 0.01
        gain_ok &= ok
        print(f"  aim= {aim:+6.2f} eps = {eps:+5.1f} ( {len(ratios)} attitude): "
              f"gain {worst_value:.4f} (old code {previous:.4f})  "
              f"{'OK' if ok else 'ERROR'}")
    print(f"  horizontal gain within 1.000+-0.01: {'YES' if gain_ok else 'NO'}")
    print()
    print("--- RANGE TERMINATION ---")
    g3 = VirtualGimbal(g.width, g.height, g.hfov_rad, g.mount_phys_pitch_deg,
                     aim_pitch_deg=-28.0)
    for m in (50, 120, 185, 250, 400):
        print(f"  range {m:4d} m -> active range {g3.aim_active_deg(m):+6.2f} deg")
    print()
    print("--- AIM TRIM: FUNCTIONS of clamp and speed limit ---")
    # Bad scenario: the target constantly appears 10 higher (large DC error). The trim should swallow it
    # BUT not go over the clamp and go slow.
    t = AimTrim(start_deg=-27.0, clamp_deg=6.0, tau_s=25.0,
                max_speed_dps=0.15, min_sample=0)
    dt = 1.0 / 30
    for seconds in (0, 10, 30, 60, 120, 300):
        target_sn = seconds
        while getattr(t, '_t', 0) < target_sn:
            t.update_value(-10.0, 80.0, dt)
            t._t = getattr(t, '_t', 0) + dt
        print(f"  t={seconds:4d}s aim={t.aim:+7.3f}"
              f"{'  [AT_CLAMP]' if t.at_clamp else ''}")
    clamp_ok = abs(t.aim - (-27.0)) <= 6.0 + 1e-6
    print(f"  clamp protected: {'YES' if clamp_ok else 'NO'} "
          f"(|aim - start| = {abs(t.aim + 27.0):.3f} <= 6.0)")

    print()
    print("--- AIM TRIM: gates ---")
    t2 = AimTrim(-27.0, min_sample=0)
    a0 = t2.aim
    t2.update_value(-30.0, 80.0, 1.0);  print(f"  outlier (|ey|=30>12) -> change {t2.aim-a0:+.4f} (should be 0)")
    t2.update_value(-5.0, 400.0, 1.0);  print(f"  long range (400>150 m) -> change {t2.aim-a0:+.4f} (should be 0)")
    t2.update_value(-5.0, 80.0, 1.0);   print(f"  valid example -> change {t2.aim-a0:+.4f} (must be !=0)")
    gate_ok = abs(t2.aim - a0) > 0

    print()
    print("--- ANALYTICAL START ---")
    for b, d in ((25, 13), (25, 9), (40, 20), (30, 5)):
        print(f"  back={b:3d} down={d:3d} -> aim = {analytical_aim(b, d):+7.2f} deg")

    print()
    print("--- RANGE ESTIMATE (from width bbox) ---")
    for w in (300, 150, 60, 20):
        print(f"  bbox {w:4d} px -> ~{g.range_prediction(w):6.1f} m")

    successful = successful and clamp_ok and gate_ok and gain_ok
    print()
    print("RESULT:", "ALL STATIC TESTS PASSED" if successful else "TEST FAILED")
    return successful


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--test', action='store_true', help='run static validation')
    p.add_argument('--mount', type=float, default=30.0)
    p.add_argument('--aim', type=float, default=0.0)
    p.add_argument('--hfov', type=float, default=1.1519)
    a = p.parse_args()
    g = VirtualGimbal(hfov_rad=a.hfov, mount_phys_pitch_deg=a.mount,
                    aim_pitch_deg=a.aim)
    if a.test:
        raise SystemExit(0 if static_test(g) else 1)
    print(g.summary_value())


if __name__ == '__main__':
    main()
