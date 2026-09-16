#!/usr/bin/env python3
"""
los_guidance.py - LOS (Line-of-Sight) ratio homing, DISPLAY phase
==============================================================================

*** FROZEN ARM (gimbal branch, 2026-08-05) *** This arm was written assuming a BODY-FIXED CAMERA and
will NOT work in the PHYSICAL GIMBAL branch: the "camera axis = mount + body pitch" assumption is
INVALID. The camera is now on a self-stabilizing single axis (tilt) gimbal; body pitch is not
reflected in the image (measured: body +-35 deg while camera max 0.65 deg) and the vertical axis CAN
NOW BE COMMANDED (YILDIZ_TILT = atan(down/back)). The vertical channel here should not be used
without re-derived diagnostics based on FOV bands and pitch. For rehabilitation: GIMBAL_NOTES.md

Implements contract visual_base.VisualController. Angular error from input virtual gimbal
(ex_deg, ey_deg) + estimator range; output NED LINEAR SPEED (+yaw_rate). ATTITUDE IS NOT COMMANDED.


0. RELATIONSHIP WITH SOURCE CODES (stars25/legacy1.py, 2LOSKF*.py)
-------------------------------------------------------------- THE SKELETON OF THE LAW WAS
TRANSMITTED FROM legacy1.py; NOT the exact port:

  RECEIVED (ideas retained verbatim) * The command direction is an angle CASE (sigma_y, sigma_z) and
  is integrated at each step; The speed command is generated as v_d * n_d(fit). legacy1: sigma_cmd
  += dt * (K * ang_diff(q, fit) - Kd * qdot) * The velocity magnitude is INCREMENTAL: v_d =
  |v_now| +k_a (2LOSKF2, line 305). "Go k_a faster than you're going now" -- a closed-loop ramp
  that respects the vehicle's own acceleration limits. * k_a closes with alignment-dependent COsine
  CAN (2LOSKF2 calculate_ka_from_ey): acceleration when not aligned, steer first. * yaw law:
  yaw_rate = kp * e_x + kd * de_x/dt , kp=0.71, kd=0.917
      (legacy1 line 328 / 2LOSKF2 line 310 ) -- SINGLE tune duo, which is known to work well in the
      user's hands, has been ported to unit compatible.

  CHANGED (and WHY)
    (a) DERIVATIVE TERIMININ SIGN. legacy1'de sigma_dot = K*(q - sigma)
        - Kd*qdot. This is for lead angle d = fit - q
            d_point = -K*d - (Kd + 1) * qdot
        gives; i.e. while LOS turns right, the command shifts LEFT (leading BACK). Since the static
        target has qdot ~ 0, this term never wakes up -- the observation "2loskf always hit the
        static target" is exactly the consequence. In the case of a diagonal target, the law falls
        into delayed pursuit. The correct proportional navigation (PN) sign is PLUS:
            d_point = -d/tau_lead + (N - 1) * qdot                     (A)
        With (A), the inertial rotation speed of the velocity vector becomes N * qdot; This is the
        exact speed-commanded vehicle equivalent of the classic PN (see 2). (b) DE-ROTATION. legacy1
        constructs the unit vector LOS with Rbe (roll/pitch/yaw) from the raw pixel; The error
        signal is released as the body oscillates. In our case, the virtual gimbal
        (yildizlar_gimbal) already clears roll/pitch, ex/ey comes from the horizon-aligned frame.
        (c) EXTRACTION OF OUR OWN YAW RATE. ex is based on BODY; the inertial ratio LOS is chi_point
        = ex_point + yaw_point (see 1). legacy1 was taking this term implicitly because it installed
        LOS directly in the Earth frame; Since we are working with ex we have to add EXPLICITLY. If
        it is missing, the yaw controller thinks its own rotation is "missing the target" and gives
        a false rate to PN (self-feeding rotation). (d) DEPTH. legacy1/2LOSKF was taking the range
        from width bbox (c_ptz = REAL_TARGET_WIDTH * F_OC / w). INVALID IN THIS ENVIRONMENT: smoke
        testing showed that the width of bbox depends on the aspect (204 in bend 31 px, standoff 26
        in m 34 px). The range is taken from the estimator ALONE; Used with area_root as progress
        signal. (e) LIMITS. MAX_YAW_RATE 360 deg/s -> 60 deg/s, slew limited. Mission briefing and
        tremor notes at yildizlar_guidance.sh.

  UNUSED: Kalman/EKF imports in source codes (user note: that task was never done, dead import).

  old_los_codes/*.py IS INVALID REFERENCE (user, 2026-08-03); nothing was taken from there.


1. FRAMES AND MEASUREMENT MEANINGS ------------------------------ H frame ("fuselage-horizon"): x =
HORIZONTAL projection of fuselage nose (yaw), y = right, z = down.
visual_base.body_forward_ned(yaw, forward, right, down) turns this frame into NED.

  ex_deg : Horizontal bearing of LOS relative to the BODY nose (+ right).
  yildizlar_gimbal.angle_error_value() produces this with atan2(r_stab[1], r_stab[0]); r_stab is de-rotated
  with yaw=0, so ex is the azimuth in the H frame. ey_deg : Vertical deviation relative to VIRTUAL
  frame center (+ below). The imaginary center remains elevated relative to the horizon, so

               eps = -aim_active - ey_deg                              (1)

           eps = elevation of the target relative to the HORIZON [deg]. tools/scenario.sh
           YILDIZ_AIM=0 constants (2026-08-03) -> aim_active = 0 and eps = -ey; "ey -> 0" is the
           DIRECT co-altitude collision geometry.

           PROJECTION CORRECTION: ey is a PERSPECTIVE measurement, the vertical angle INFLATES as
           you move horizontally away from the frame center. Full correlation (los_test.py confirms
           with real gimbal):

               eps = atan( -tan(ey) * cos(ex) ) - aim_active              (1)

           Unfixed version ex=20 deg, eps=12 deg gave error %6
           (12.75 okunuyordu). aim=0 iken (1) TAMDIR; aim != 0 iken ex, aim
           used instead of azimuth in freeze frame (second order approximation).

Inertial LOS angles (equivalent to q_z / q_y in legacy1):

    q_az = yaw + ex        (azimut, NED)          <-> legacy1 q_z
    q_el = eps (ascension relative to horizon) <-> legacy1 -q_y

Derivatives (with MEASURED dt, filtered):

    q_az_point = ex_point + yaw_point                                 (2)
    q_el_point = d(eps)/dt                                            (3)

(3) is DIRECTLY the derivative of eps (not ey): thus both the projection correction in (1) and the
change of the current ramp enter into the derivative automatically.


2. CONTROL LAW -- RANGE SCALE LEAD angle
----------------------------------------------------------------- The command direction is defined
by the LEAD angle states according to LOS (same structure as the fit state of legacy1):

    sigma_az = q_az + d_az        sigma_el = q_el + d_el

COMPLETE INCREASE OF THE COLLISION TRIANGLE. The velocity component of the fighter perpendicular to
LOS is V*sin(d). Relative transverse speed is measured directly:

    R * q_point = (speed of target perpendicular to LOS) - V * sin(d) (4)

This identity does NOT slow down the target; it is just a PLUS read from range (estimator, allowed)
and angular LOS ratio (camera). The leading angle that will reset the ratio LOS is derived
ALGEBRAICALLY from here:

    sin(d_needed) = sin(d) + k_pn * R * q_point / V (5) d_needed = hang(clamp(sin(d_needed),
    +-sin(d_max)) )

k_pn = 1.0 means "full reset LOS rate at next instant" (dead-beat); >1 takes too long, <1 az.

LEAD DYNAMICS (first order approximation + washing):

    d_point = (d_needed - d) / tau_yak - d / tau_lead (6)

tau_yak is selected to the same order as the vehicle response (command LPF 0.35 s + helicopter
dynamics ~0.5 s); If faster is selected, the controller thinks that the lead that the vehicle has
not yet implemented is "insufficient" and starts to inflate (windup).

WHY THIS FORMAT IS NOT THE CLASSIC d_point = (N-1)*q_point: in the small angle limit the two are
identical and the ACTIVE navigation ratio

    N_active = 1 + R / (V * tau_yak * cos d)                               (7)

exits -- so (6) is a RANGE-TIMED PN. Its significance is this: when R -> 0, q_point explodes with
1/R; The classic PN with FIXED N now clamps the lead and flies the vehicle almost perpendicular to
LOS (measured: in the crossover scenario the lead saturates to 60 degrees in 1 s, the range turns
and opens in 21 m). Since the R factor in (5)-(6) decreases at the same time, the leading demand is
automatically eliminated. Typical values ​​of N_active: R=60 m, V=18, tau_yak=0.8 -> 5.2; R=25 m ->
2.7.

IF R is NOT present: if the range is not known at all (5) it is run with range_default_m and
logged as diagnosis['range_source'] = 'none'; The law still works, but the scale is crude.

WASH (-d/ tau_lead ) is the K*(q - sigma) term of legacy1 (K = 1 / tau_lead ): When the LOS ratio
decreases to zero, it reduces the law to pure pursuit back and prevents noise from being inflated by
random walk in the integrator. Since there is a LEAK, it is selected TOO LARGE than the collision
time (default 20 s; terminal phase ~ 5 - 15 s).

(5)-(6) is solved EVERY LOOP (even though the new bbox instance has not arrived): q_point is already
a DISTINCTIVE rate estimate, it is correct to integrate it up to the loop step with the zero-order
holder. Unless the sample arrives, the only thing that needs to be returned is the DERIVATION FILTER
(section 6).

LEADING CLAMP AND FOV : leading angle rotates the VELOCITY VECTOR, NOT THE CAMERA. Since the yaw in
the copter is driven independently of the velocity vector, the large lead does NOT remove the target
from the frame. The clamp ( lead_az_max , default 85 deg ) is only limited by "sacrifice all closing
speed". IT MUST BE WIDE: since the target is FASTER than us ( 20 vs 18 m/s ), the collision triangle
is sin (d) = ( V_t / V_p ) with sin ( theta_t ) Leading up to 80 degree may require;  los_test in
closed loop scanning (circle 6 / 9 / 14 deg /s) 60 deg clamp 22.4 / 15.1 / 9.8 m KIDNAPPING, 85 deg
handcuff 1.3 / 1.5 / 1.2 was a hit. Washing ( tau_lead ) and terminal freezing reduce the risk of
parking in saturation.

Command direction and speed (same format as legacy1 n_d / V_d, in frame H):

    c = ex + d_az (azimuth relative to the hull, deg) (6) g = k_perpendicular * eps + d_el (track angle
    relative to the horizon, deg) (7)
    n_d = [cos g cos c, cos g sin c, -sin g]                          (8)
    v_d = clamp(|v_now| + k_a, v_min, v_max) (9)
    v_H = v_d * n_d

k_perpendicular (--vertical-ratio) default is 1.0; While 1.0 (7) is pure LOS (see "VERTICAL RATIO").

WHERE RANGE IS USED: As the SCALE of the leading angle in (5) (permitted and clearly suggested use
in the briefing: "you can use the range in the closing calculation"), to solve the aam ramp in (1),
for TERMINAL gate and R_point diagnostics. Target SPEED / DIRECTION / ACCELERATION is not derived
anywhere -- (4) is a MEASUREMENT identity, not an estimate.

PROGRESS SIGNAL: range_m_value and area_root TOGETHER. area_root = sqrt(w*h) is proportional to 1/R in
fixed aspect, so d(ln area_root)/dt = -R_point/R is the scale-free closing ratio. area_root BY ITSELF
IS NOT a range proxy because the Aspect has changed (smoke test); It provides the terminal gate and
objective function with the range.


3. SPEED LAW (From 2LOSKF2) ---------------------------
    k_a = KA_PEAK * cos( min(theta/THETA_THRESHOLD, 1) * pi/2 )           (10)
    v_d = clamp(|v_now| + k_a, v_min, v_max)

theta = Angle between CURRENT speed vector and command direction n_d. The source code is |e_y|
instead of theta (vertical pixel error) was using; CHANGED HERE because in our handoff geometry the
target starts ~27 degrees UP (standoff back=25 / down=13) and |e_y| The threshold would remain
closed throughout the entire terminal and the vehicle would not accelerate at all. Since the camera
and the speed vector are already SEPARATE in the copter, the physical equivalent of the question "Am
I aligned" is theta.

Meaning: if the direction you need to head is very different from your current heading, DO NOT
ACCESS, freeze first (slowing down corrects direction quicker as the turning radius increases with
V^2/a); As you align, go to full throttle.

|v| ceiling VISUAL_MAX_SPEED_MPS (frame also clamps). upgraded 18 -> 20 m/s in 2026-08-03
(params/swarm_copter.parm WPNAV_SPEED 2000 cm/s = 20 m/s was already allowing). THIS NUMBER WAS THE
LARGEST PERSON: since the target was tracking 20 m/s, when staying at 18, the returning target was
only caught with a VERY wide lead (85 deg); Capturing even with narrow lead (60 deg) on 20
(los_test.test_range_ceiling_critical: 22.4 m -> 1.5 m; range at end of 30 s on straight route 88.7 m
-> 28.7 m). SPEED DEFICIT MAY BE REBORN IN FLIGHT: in climb the horizontal component drops to
V*cos(gamma) and WPNAV_SPEED_UP 10 m/s further trims the vertical; so the wide lead clamp is
retained. Vertical ceiling params/swarm_copter.parm: WPNAV_SPEED_UP 10, WPNAV_SPEED_DN 5. |v_z| If
it exceeds the ceiling, the ENTIRE vector is scaled first (DIRECTION IS PRESERVED -- that's what's
important for guidance), and when it goes down to the base level, only v_z is clipped.

VERTICAL RATIO (--vertical-ratio, k_perpendicular): target aircraft 20 m/s in cruise; the helicopter roof is also
20 m/s but WHEN CLIMBING the horizontal component drops to V*cos(gamma), meaning that during
vertical closing you are actually slower than the target. Since the range closing speed in tail
chase is R_point = V_t * cos(theta_t) - V_p (theta_t = between LOS and the target speed), LOS's
being DEVIATED from the target speed vector causes the closing. It heals. At the time of handoff the
target is ~27 degrees UP; Climbing immediately with pure LOS wastes this angle advantage in ~1.5 s,
leaving an unwinnable straight tail chase. k_perpendicular < 1 delays vertical closing and maintains the angle
advantage. The DEFAULT 1.0 (pure LOS) was left so that the fundamental law of the article would not
be violated; k_perpendicular is a TEST BUTTON. NOTE: theta_t is neither measured nor estimated; The above
correlation is just the JUSTIFICATION of the button.


3b. RAW FRAMING ( FOV ) PROTECTION -- 2026 - 08 - 03 CAUSE OF ELLIPSE RUN ROOT_VALUE
---------------------------------------------------------------------------------- Authorization in
the first sim run (run/tries/los_ellipse_20260803_173736) 16 BOUGHT AND RETURNED ONCE.  CSV
analysis:

  * AVERAGE 1.2-1.4 is detected after each handoff; the remaining ~2.7 s are the DM's dwell (no
  command, the skeleton fades to zero). Total 41.9 s freshness / 293 s window. * Average cmd_vz =
  -2.34 m/s (CLIMB) in 1 s before loss. * bbox.log attitude: pitch median in positioned phase
  -1.8 deg, median in DISPLAY phase +11.2, p95 +31.3 deg (NOSE UP).

GEOMETRY: camera FIXED TO THE BODY +30 deg mounted. The axis seen in the raw frame is rising
relative to the horizon (mount + pitch). If the rise of the target is eps, the target is

    alt_angle = (mount + pitch) - eps (12)

is BELOW by and the vertical half-frame is only 20.07 deg. With measured values: positional 30 - 1.8
- 11.1 = 17.1 deg (BARRELY IN), visual 30 + 11.2 - 15.7 = 25.5 deg (OUT OF FRAME).

In other words, what makes you miss the target is neither the quality of the bbox nor the horizontal
guidance; IT IS THE NOSE-UP PITCH PRODUCED BY OUR OWN CLIMBING COMMAND. The hopper tilts its thrust
vector back to climb (and maintain horizontal speed while climbing); the camera looks up with it,
and the target, already standing 16-19 degrees off-axis, moves out of frame from below. Then during
the dwell the skeleton command goes down to zero -> copter brakes -> pitch goes even higher ->
vicious circle.

SOLUTION (two-layer): (a) FOV PROTECTION: pitch IS MEASURED (Measure.pitch_rad), (12) is calculated
and the command path angle g is pulled DOWN enough to keep the target in the frame: over = |alt_angle|
- fov_margin_deg If alt_angle > 0 then g -= k_fov * more (target at the bottom -> az climb) If alt_angle <
0 then g += k_fov * more (target at the top -> climb further) This IT IS A CLOSED LOOP: As g
decreases, the climb decreases, pitch decreases, alt_angle decreases, the protection withdraws itself.
fov_margin_deg = 14 selected: 20.07 deg ~%70 of vertical half-frame, remaining margin pitch
oscillation (33 from 5 to 95 deg measured) and bbox for center noise. (b) CLIMB CEILING
(climb_max_mps): commanded UP speed is trimmed hard. CAUTION -- this is a deliberate EXCEPTION to
the "keep DIRECTION on vertical ceiling" rule: scaling the entire vector to preserve direction would
also kill horizontal velocity, whereas the target is going 20 m/s and our horizontal velocity budget
is already tight. We put the sensor constraint (being able to see the target) ABOVE the guidance
optimality: lock loss loses everything.

DELAYING VERTICAL CLOSE IS NOT EXPENSIVE: eps does NOT have to go to zero for collision; fixed
bearing + closing range is sufficient (see section 2). Slowing the climb only changes the contact
angle.


4. TERMINAL GATE ------------------ when range <= terminal_range_m OR area_root >=
terminal_area_root: * lead integrators are FREEZED (d_az, d_el remain constant), * k_a is forced to
its full value (alignment gate bypass) -> full throttle. Reason: R -> At 0, q_point explodes with
1/R; Since the remaining flight time (t_go) is smaller than the vehicle delay (LPF 0.35 s + hopper
dynamics), the correction produced at this stage CANNOT BE APPLIED ANYWAY, it only produces a skid
(and miss pass) at the last moment. Terminal freezing is also standard in classic PN applications.


5. YAW (FOV) CONTROLLER ------------ FOV 66 deg horizontal (+-33), without camera gimbal -> keeping
the target in the frame horizontally is the job of the guidance. legacy1/2LOSKF2 law carried over
verbatim (unit compatible):

    yaw_rate = kp * ex + kd * ex_point                                (11)
    kp = 0.71 [1/s], kd = 0.917 [s]

Since ex_point = q_az_point - yaw_point (11) implicitly contains both feedforward LOS (kd *
q_az_point) and YAW RATIO TERMINATION (-kd * yaw_point); This is the power of the source law.

MAX_YAW_RATE = 360 deg/s in the source code HERE 60 reduced to deg/s and put a slew limit on top:
yildizlar_guidance.sh and scenario.sh notes slow loop + big yaw He records that his steps cause tremors.
Additionally, the dead band (bbox center noise ~+-0.15 deg) interrupts the limit-cycle around zero.

The yaw command DOES NOT CHANGE THE VELOCITY VECTOR (the command is in NED); it only directs the
camera, so the aggressiveness of the yaw channel affects IMAGE quality, not orbital stability.


6. MEASUREMENT HEALTH ---------------- * SAME SAMPLE PROTECTION: cycle 20 Hz, camera at different
speed. If the same bbox appears in two cycles (ex,ey,w,h identical) the derivative comes out (ex -
ex_previous)/dt = 0 and the ratio LOS is read SYSTEMATICALLY LOW. The DERIVATION FILTER is frozen and
dt is accumulated until the new sample arrives. (The leading angle (6) is however resolved in each
cycle -- see section 2.) Measured: without protection 8 deg/s actual LOS rate was reading 6.3 deg/s.
* DELAY COMPENSATION: ex/ey are at the moment of capture; The current bearing is predicted with
delay ex + ex_point * (delay = bbox_age_s + fixed addition). * GAP RESET: derivative cases are reset
if > reset_gap_s between two valid measurements (stale derivative = fake large LOS ratio).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

from visual_base import (Command, VisualLoop, VisualController,
                             body_forward_ned)

# yildizlar_gimbal is in the repository root; The parent directory may not be in sys.path because
# scenario.sh does 'cd guidance_allstar'. READ-ONLY import (kept accessible only so tests can verify
# mirror functions).
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.append(_ROOT)


# ------------------------------------------------------------------ constants
#
# CAMERA (models/swarm_drone_1/model.sdf + yildizlar_gimbal.py:99-101)
CAMERA_WIDTH_PX = 1280.0
CAMERA_HEIGHT_PX = 720.0
CAMERA_HFOV_DEG = 66.0                  # -> fx = 640/tan(33) = 985.5 px
FX_PX = (CAMERA_WIDTH_PX / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
FOV_HORIZONTAL_HALF_DEG = CAMERA_HFOV_DEG / 2.0                       # +-33.0
FOV_VERTICAL_HALF_DEG = math.degrees(math.atan(
    math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
    * CAMERA_HEIGHT_PX / CAMERA_WIDTH_PX))                 # +-20.07

# Aim range damping (EXACT same ramp as yildizlar_gimbal.aim_active_deg; los_test.py confirms numerical
# equality of the two). This ramp is NO-OP in default operation because it gives scenario.sh
# YILDIZ_AIM=0.
AIM_FULL_RANGE_M = 120.0
AIM_ZERO_RANGE_M = 250.0

# Vehicle roofs (params/swarm_copter.parm line 30-40)
WPNAV_SPEED_UP_MPS = 10.0
WPNAV_SPEED_DN_MPS = 5.0


def wrap180(a_deg: float) -> float:
    """Wrap an angle to the interval [-180, 180)."""
    return (float(a_deg) + 180.0) % 360.0 - 180.0


def clamp(x, alt, upper_value):
    return alt if x < alt else (upper_value if x > upper_value else x)


def analytical_aim(back_m: float, down_m: float) -> float:
    """aim = -atan(down/back). yildizlar_gimbal.analytical_aim'in MIRROR.

    It is mirrored (not imported) so that it can run in an environment where the repository root is
    not accessible (unit test, different cwd). los_test.py verifies the numerical identity of two
    applications. Since scenario.sh fixes AIM=0, this function is only activated if --back/--down is
    tested manually.
    """
    return -math.degrees(math.atan(float(down_m) / max(1e-6, float(back_m))))


def aim_active(aim_deg: float, range_m_value):
    """I am range-aware. yildizlar_gimbal.VirtualGimbal.aim_active_deg MIRROR.

    range None -> full range (no information to absorb).
    """
    if range_m_value is None:
        return float(aim_deg)
    r = float(range_m_value)
    if r <= AIM_FULL_RANGE_M:
        k = 1.0
    elif r >= AIM_ZERO_RANGE_M:
        k = 0.0
    else:
        k = ((AIM_ZERO_RANGE_M - r)
             / (AIM_ZERO_RANGE_M - AIM_FULL_RANGE_M))
    return float(aim_deg) * k


def eps_solve(ex_deg, ey_deg, aim_active_deg):
    """(1) Target elevation relative to the HORIZON [deg], with projection correction.

        eps = atan( -tan(ey) * cos(ex) ) - aim_active

    WHY cos(ex): ey is a PERSPECTIVE angle read from the frame center; In the yildizlar_gimbal
    chain, it turns out tan(-ey) = tan(eps + aim) / cos(psi) (psi = horizontal angle in the virtual
    frame, ex itself when aim=0). Without correction, when the target is at the edge of the frame,
    the ascension will be read FLOAT: ex=20 deg, eps=12 12.75 deg (%6) at deg. los_test.Geometry
    confirms this relation with the REAL VirtualGimbal.
    """
    ey_r = math.radians(float(ey_deg))
    # Limit ey against tan(+-90) overflow (out-of-frame bbox may not come, but stale/corrupt data may
    # come).
    ey_r = clamp(ey_r, -math.radians(85.0), math.radians(85.0))
    ex_r = clamp(math.radians(float(ex_deg)), -math.radians(89.0),
                   math.radians(89.0))
    return math.degrees(math.atan(-math.tan(ey_r) * math.cos(ex_r))) \
        - float(aim_active_deg)


class DerivativeFilter:
    """First order filtered numerical derivative.

    Crude derivative (x - x_previous)/dt bbox center jitter (+-3 px = +-0.15 deg) dt=0.04 +-3.8 at s
    deg/s converts to noise; With tau this comes down to ~1/3. AS tau GROWS, the noise decreases,
    but the rate of LOS is delayed -> the lead is late. (In the source code, the same work was done
    with the fixed-weight LPF beta_qdot=0.2, written here as dt-aware because the loop step is
    variable -- fixed weight means a different cutoff frequency as the loop slows down, a bug class
    previously flagged as a root cause in the repository.)
    """

    def __init__(self, tau_s):
        self.tau = float(tau_s)
        self.x_previous = None
        self.d = 0.0

    def reset_value(self):
        self.x_previous = None
        self.d = 0.0

    def update_value(self, x, dt):
        if x is None or dt <= 1e-6:
            return self.d
        if self.x_previous is None:
            self.x_previous = float(x)
            return self.d
        raw_value = (float(x) - self.x_previous) / dt
        self.x_previous = float(x)
        a = dt / (dt + self.tau) if self.tau > 1e-6 else 1.0
        self.d += a * (raw_value - self.d)
        return self.d


# -------------------------------------------------------------- controller

class LosController(VisualController):
    """LOS ratio (PN) video guidance controller.

    All tune parameters are __init__ arguments; main() parses them with argparse (eg `python3
    los_guidance.py --n 4.0 --vertical-ratio 0.6`).
    """

    label_item = "los"

    def __init__(self,
                 # --- PN cekirdegi (denklem 5-6) ---
                 k_pn=1.0,              # ( 5 ) LOS rate gain. 1.0 = dead beat
                                        # (“fully cancel the LOS rate at the next instant”). > 1 overcorrects (overshoot), < 1 undercorrects. For active classical N
                                        # (7).
                 tau_yak_s=0.8,         # (6) time constant for approach to leading.
                                        # Must be the same order as the vehicle response (command LPF 0.35 s + helicopter dynamics ~0.5 s); If
                                        # it is chosen small, the controller will consider the lead that the vehicle does not implement as
                                        # "insufficient" and will inflate (windup).
                 tau_lead_s=20.0,        # leading flushing constant (of legacy1
                                        # K = 1 / tau 'that). It must be MUCH larger than the collision time (~5 - 15 s) or leakage will not
                                        # reduce the LOS rate to zero
                                        # (bkz. bolum 2).
                 lead_az_max_deg=85.0,   # horizontal leader clamp. RELATED TO FOV
                                        # HEC (the lead rotates the SPEED VECTOR, not the camera; yaw is driven separately). MUST BE WIDE:
                                        # since the collision triangle is sin(d) = (V_t/V_p) sin(theta_t), it requires a leader up to 80
                                        # degrees when V_p < V_t. los_test closed loop scan, v_max=18 (rate deficit), circle 6/9/14 deg/s: 60
                                        # deg -> 22.4/15.1/9.8 m HIJAB 75 deg -> 11.4/ 6.4/3.1 m 85 deg -> In parity 1.3/ 1.5/1.2 m HIT
                                        # v_max=20 the clamp is no longer binding (60 also hits deg), but the wide clamp is maintained as the
                                        # speed deficit returns in the climb. Washing (tau_lead) and terminal freezing reduce the risk of
                                        # parking in saturation.
                 lead_el_max_deg=25.0,   # vertical leader: the vertical speed of the helicopter
                                        # authority is already limited to 10 / 5 m/s , large vertical pioneer is not applicable (35 deg tried,
                                        # difference < 0.3 m).
                 # --- measurement processing (part 6) ---
                 tau_derivative_s=0.20,      # LOS rate filter (see DerivativeFilter )
                 tau_yaw_point_s=0.20,  # yaw rate filter: with ex_point in (2)
                                        # It should be the SAME as tau_derivative because it is COLLECTED. If different, the two signals would be
                                        # delayed differently and their sum would not fully account for our return -- measured: 20 in 0.15 vs
                                        # 0.20 deg /s yaw under 3.2 deg would remain the false lead.
                 latency_ek_s=0.05,     # Fixed pipeline on bbox_age_s
                                        # delay (ros -> Redis -> loop)
                 reset_gap_s=0.60, # If left unmeasured, derivatives
                                        # resets (stale derivative fake rate)
                 # --- rate law (equation 10) ---
                 v_max_mps=None,        # None -> cfg.VISUAL_MAX_SPEED_MPS
                 v_min_mps=4.0,         # |v_now| If the command is read incorrectly
                                        # base to prevent collapse
                 ka_peak_mps=2.0,       # 2LOSKF2 KA_PEAK = 2.0 (aynen)
                 theta_threshold_deg=35.0,   # cosine cani genisligi. 2LOSKF2'de
                                        # 300 px = 17.8 was deg but the criterion was |e_y|; Since the criterion here is theta (see section 3)
                                        # it is wider.
                 vertical_ratio=1.0,        # k_perpendicular : 1.0 = pure LOS
                 # --- raw framing ( FOV ) protection, part 3b ---
                 mount_deg=None,        # None -> $YILDIZ_MOUNT (0.0)
                 fov_margin_deg=14.0,     # |alt_angle| allowed in (12).
                                        # Vertical half-frame 20.07 deg; for the remaining 6 deg share pitch oscillation and bbox noise.
                 k_fov=1.0,             # protection gain (deg output / deg error)
                 fov_correction_max=30.0, # maximum the protection can absorb g
                 climb_max_mps=3.0,  # commanded UP speed ceiling.
                                        # Measurement: average at time of loss
                                        # cmd_vz = -2.34, p5 = -8.66 m/s idi.
                 gamma_min_deg=-25.0,   # road angle clamps: copter upright
                 gamma_max_deg=55.0,    # can climb but horizontal on top of 55 deg
                                        # closing stops altogether
                 # --- terminal door (section 4 ) ---
                 terminal_range_m=12.0,
                 terminal_area_root=55.0,  # IT WILL BE CALIBRATED WITH SIM. 26 in m
                                          # bbox ~34 px measured wide; Since area_root depends on the aspect, this threshold should be corrected
                                          # from the logs in the first run.
                 # --- yaw (FOV) controller (equation 11) ---
                 yaw_enabled=True,
                 kp_yaw=0.71,           # legacy1/2LOSKF2 tuned value
                 kd_yaw=0.917,          # legacy1/2LOSKF2 tuned value
                 yaw_dead_band_deg=0.8,  # bbox center noise ~+- 0.15 deg ;
                                        # 0.8 deg interrupts dead band limit-cycle
                 yaw_rate_max_dps=60.0, # Downloaded from 360 in source code
                 yaw_acceleration_max_dps2=120.0,  # ramp instead of steps -> camera
                                           # shake/blur decreases
                 tau_yaw_output_s=0.10,
                 # --- geometry / safety ---
                 aim_deg=None,          # None -> $YILDIZ_AIM (scenario.sh 0)
                 back_m=None, down_m=None,   # only when no current is given and
                                             # For derivation if YILDIZ_AIM is not available
                 min_altitude_m_value=12.0,     # Below this altitude, descent is reset.
                 range_min_m=3.0, range_max_m=400.0,
                 range_default_m=40.0):   # If there is no range for (5)
                                              # coarse scale used; middle of typical handoff range as handoff gate <=60 m
        import guidance_config as cfg   # local import: keep the test environment light
        self.v_max = float(v_max_mps if v_max_mps is not None
                           else getattr(cfg, "VISUAL_MAX_SPEED_MPS", 18.0))
        self.v_min = float(v_min_mps)

        self.k_pn = float(k_pn)
        self.tau_yak = float(tau_yak_s)
        self.tau_lead = float(tau_lead_s)
        self.lead_az_max = float(lead_az_max_deg)
        self.lead_el_max = float(lead_el_max_deg)

        self.tau_derivative = float(tau_derivative_s)
        self.latency_ek = float(latency_ek_s)
        self.reset_gap = float(reset_gap_s)

        self.ka_peak = float(ka_peak_mps)
        self.theta_threshold = float(theta_threshold_deg)
        self.vertical_ratio = float(vertical_ratio)
        # DEFAULT 30.0 -> 0.0 (2026-08-04 mount pass): sim mount 0 graded (pitch-servo gimbal decision; 0
        # fixed camera as the copter barely tilts in sim ~ideal gimbal). $YILDIZ_MOUNT is still the only
        # source (scripts/standoff_geom.sh).
        self.mount_deg = float(mount_deg if mount_deg is not None
                               else os.environ.get('YILDIZ_MOUNT', 0.0))
        self.fov_margin = float(fov_margin_deg)
        self.k_fov = float(k_fov)
        self.fov_correction_max = float(fov_correction_max)
        self.climb_max_value = float(climb_max_mps)
        self.gamma_min = float(gamma_min_deg)
        self.gamma_max = float(gamma_max_deg)

        self.terminal_range = float(terminal_range_m)
        self.terminal_area_root = float(terminal_area_root)

        self.yaw_enabled = bool(yaw_enabled)
        self.kp_yaw = float(kp_yaw)
        self.kd_yaw = float(kd_yaw)
        self.yaw_dead_band = float(yaw_dead_band_deg)
        self.yaw_rate_max = float(yaw_rate_max_dps)
        self.yaw_acceleration_max = float(yaw_acceleration_max_dps2)
        self.tau_yaw_output = float(tau_yaw_output_s)

        self.min_altitude = float(min_altitude_m_value)
        self.range_min = float(range_min_m)
        self.range_max = float(range_max_m)
        self.range_default = float(range_default_m)

        # AIM priority order: --aim > $YILDIZ_AIM > analytics(back,down) > 0.0. The tools/scenario.sh line
        # exports 84 `YILDIZ_AIM="${AIM:-0}"`, so in NORMAL operation current = 0 and (1) is reduced to eps =
        # -ey. DEFAULT OF ZERO IS CONSCIOUS: falling to env or analytic -27.5 conflicts with 0, which
        # scenario.sh pins, and shifts eps by 27 degrees. Analytical derivation is used only if --back/--down
        # is EXPRESSLY given.
        self.back_m = float(back_m) if back_m is not None else None
        self.down_m = float(down_m) if down_m is not None else None
        if aim_deg is not None:
            self.aim_deg, self.aim_source = float(aim_deg), 'arg'
        elif os.environ.get('YILDIZ_AIM') not in (None, ''):
            self.aim_deg, self.aim_source = float(os.environ['YILDIZ_AIM']), 'env'
        elif self.back_m is not None and self.down_m is not None:
            self.aim_deg = analytical_aim(self.back_m, self.down_m)
            self.aim_source = 'analytical'
        else:
            self.aim_deg, self.aim_source = 0.0, 'default_value2'

        # --- situations ---
        self.d_ex = DerivativeFilter(self.tau_derivative)
        self.d_eps = DerivativeFilter(self.tau_derivative)     # ( 3 ): DIRECT derivative of eps
        self.d_yaw = DerivativeFilter(float(tau_yaw_point_s))
        self.d_range = DerivativeFilter(0.50)        # diagnostics/log only
        self.d_ln_area = DerivativeFilter(0.50)       # non-scale closing rate
        self.lead_az = 0.0
        self.lead_el = 0.0
        self.yaw_rate_output = 0.0
        self.terminal = False
        self._signature = None            # latest bbox sample (same sample protection)
        self._t_last_new = None      # time of last NEW sample
        self._t_previous = None        # last command() call
        self._range_last = None
        self.diagnostic = {}               # test/debug window

    # -------------------------------------------------------------- handoff

    def seed_value2(self, handoff):
        """handoff moment. Derivative/leading states start from ZERO: the ratio LOS is ~0 as the positioned
phase is in balance at standoff, zero leading is the correct start. The skeleton's LPF already seeds
the speed continuum with the handoff speed (visual_base.py:388-393), which is not repeated here.
        """
        self.d_ex.reset_value()
        self.d_eps.reset_value()
        self.d_yaw.reset_value()
        self.d_range.reset_value()
        self.d_ln_area.reset_value()
        self.lead_az = 0.0
        self.lead_el = 0.0
        self.yaw_rate_output = 0.0
        self.terminal = False
        self._signature = None
        self._t_last_new = None
        self._t_previous = None
        self._range_last = None
        # If the positioned phase has a final range, let it be the seed to the range chain (handoff now occurs
        # with estimator range <= 60 m gate).
        if handoff and handoff.get('range_m'):
            try:
                self._range_last = clamp(float(handoff['range_m']),
                                           self.range_min, self.range_max)
            except (TypeError, ValueError):
                self._range_last = None

    # -------------------------------------------------------------- range

    def _range_solve(self, o):
        """Range ONLY from estimator; Otherwise, the last known value is kept.

        Range is NOT derived from the width of bbox: smoke testing (2026-08-03) showed that the
        width of bbox depends on the aspect (204 in the bend 31 px in m standoff 26 median from the
        back in m 34 px). c_ptz=REAL_TARGET_WIDTH*F_OC/w in source codes is INVALID in this
        environment.
        """
        if o.range_m_value is not None and math.isfinite(o.range_m_value):
            r = clamp(float(o.range_m_value), self.range_min, self.range_max)
            self._range_last = r
            return r, 'estimator'
        if self._range_last is not None:
            return self._range_last, 'last_value'
        return None, 'absent'

    # ------------------------------------------------------------- leading angle

    def _lead_update(self, lead_deg, q_point_dps, R_m, v_ref, lead_max, dt):
        """( 5 )-( 6 ): updates the increment leading angle of the collision triangle.

        Rotary: (new_lead_deg, N_active). N_active is for LOG/article only -- equivalent to CLASSIC PN
        in this cycle, calculated with (7).
        """
        sd = math.sin(math.radians(lead_deg))
        residual = self.k_pn * R_m * math.radians(q_point_dps) / max(v_ref, 1e-3)
        s_max = math.sin(math.radians(lead_max))
        needed_value = math.degrees(math.asin(clamp(sd + residual, -s_max, s_max)))
        lead_new = lead_deg + ((needed_value - lead_deg) / self.tau_yak
                              - lead_deg / self.tau_lead) * dt
        lead_new = clamp(lead_new, -lead_max, lead_max)
        cd = max(math.cos(math.radians(lead_deg)), 0.2)
        n_active = 1.0 + R_m / (max(v_ref, 1e-3) * self.tau_yak * cd)
        return lead_new, n_active

    # ------------------------------------------------------------------------------- command

    def command_value(self, o) -> Command:
        dt_loop = max(1e-3, float(o.dt))

        # ---0. our own rate of yaw (on every call, regardless of the freshness of bbox) ---
        yaw_deg = None if o.yaw_rad is None else math.degrees(o.yaw_rad)
        yaw_point = 0.0
        if yaw_deg is not None:
            # Wrapping: give a CONTINUOUS signal to the derivative filter; If the angle jumps at -180/+180, the
            # derivative produces a pseudo-giant value of 360/dt.
            if self.d_yaw.x_previous is not None:
                yaw_deg = self.d_yaw.x_previous + wrap180(yaw_deg
                                                        - self.d_yaw.x_previous)
            yaw_point = self.d_yaw.update_value(yaw_deg, dt_loop)

        # Is there a new bbox instance in this call? (same sample protection, part 6)
        signature_value = (o.ex_deg, o.ey_deg, o.bbox_w, o.bbox_h)
        new_sample = (signature_value != self._signature)
        self._signature = signature_value

        t = float(o.t)
        dt_measurement = dt_loop if self._t_last_new is None else (t - self._t_last_new)
        if self._t_previous is not None and (t - self._t_previous) > self.reset_gap:
            # Reset instead of continuing with the stale variant after the long gap.
            self.d_ex.reset_value()
            self.d_eps.reset_value()
            self.d_range.reset_value()
            self.d_ln_area.reset_value()
            dt_measurement = dt_loop
        self._t_previous = t
        dt_measurement = clamp(dt_measurement, 1e-3, 0.5)

        # ---1. range, current, progress signal ---
        range_value, range_source = self._range_solve(o)
        aim_e = aim_active(self.aim_deg, range_value)

        # ---2. angular errors and ratios of LOS (1)-(3) ---
        ex = 0.0 if o.ex_deg is None else float(o.ex_deg)
        ey = 0.0 if o.ey_deg is None else float(o.ey_deg)
        eps = clamp(eps_solve(ex, ey, aim_e), -80.0, 85.0)      # (1)

        if new_sample:
            ex_point = self.d_ex.update_value(ex, dt_measurement)
            q_el_point = self.d_eps.update_value(eps, dt_measurement)
            if range_value is not None:
                self.d_range.update_value(range_value, dt_measurement)
            if o.area_root is not None and o.area_root > 1e-3:
                self.d_ln_area.update_value(math.log(float(o.area_root)), dt_measurement)
            self._t_last_new = t
        else:
            ex_point, q_el_point = self.d_ex.d, self.d_eps.d

        q_az_point = ex_point + yaw_point           # (2) [deg/s]

        # ---3. delay compensation (at the moment of measurement capture) ---
        latency_value = clamp((o.bbox_age_s if math.isfinite(o.bbox_age_s) else 0.0)
                          + self.latency_ek, 0.0, 0.4)
        ex_ong = ex + clamp(ex_point * latency_value, -8.0, 8.0)
        eps_ong = eps + clamp(q_el_point * latency_value, -8.0, 8.0)

        # --- 3b. our own pace (for pioneer scale and alignment gate) ---
        v_now = 0.0
        speed_h_unit = None
        if o.vel_ned is not None:
            speed_value = np.asarray(o.vel_ned, dtype=float).reshape(3)
            v_now = float(np.linalg.norm(speed_value))
            if v_now > 1.0 and o.yaw_rad is not None:
                # translate current speed to frame H (rotation yaw only)
                cy_, sy_ = math.cos(o.yaw_rad), math.sin(o.yaw_rad)
                speed_h_unit = np.array(
                    [cy_ * speed_value[0] + sy_ * speed_value[1],
                     -sy_ * speed_value[0] + cy_ * speed_value[1], speed_value[2]]) / v_now

        # --- 4 . terminal door (section 4 ) ---
        terminal = False
        if range_value is not None and range_value <= self.terminal_range:
            terminal = True
        if o.area_root is not None and o.area_root >= self.terminal_area_root:
            terminal = True
        self.terminal = terminal

        # ---5. RANGE-SCALE LEADING angle (5)-(6) --- resolved in EVERY LOOP (see section 2): q_point is a
        # velocity estimate, it is correct to integrate with the zero-order trap as much as dt_loop. The only
        # thing that is frozen while waiting for the sample is the derivative filter.
        v_ref = clamp(v_now if v_now > 2.0 else self.v_max,
                        5.0, self.v_max)
        R_scale = range_value if range_value is not None else self.range_default
        n_active_az = n_active_el = 0.0
        if not terminal:
            self.lead_az, n_active_az = self._lead_update(
                self.lead_az, q_az_point, R_scale, v_ref,
                self.lead_az_max, dt_loop)
            self.lead_el, n_active_el = self._lead_update(
                self.lead_el, q_el_point, R_scale, v_ref,
                self.lead_el_max, dt_loop)

        # ---6. command direction (6)-(8) ---
        c_deg = wrap180(ex_ong + self.lead_az)
        g_raw = self.vertical_ratio * eps_ong + self.lead_el

        # RAW FRAME (FOV) PROTECTION -- part 3b. Since the camera is fixed to the body, the command goes out:
        # path angle -> climb -> nose up pitch -> target UNDER the frame. pitch MEASURED AND fed back.
        alt_angle = fov_correction = 0.0
        if o.pitch_rad is not None:
            alt_angle = (self.mount_deg + math.degrees(o.pitch_rad)) - eps
            if alt_angle > self.fov_margin:
                fov_correction = -self.k_fov * (alt_angle - self.fov_margin)
            elif alt_angle < -self.fov_margin:
                fov_correction = self.k_fov * (-self.fov_margin - alt_angle)
            fov_correction = clamp(fov_correction, -self.fov_correction_max,
                                   self.fov_correction_max)
        g_deg = clamp(g_raw + fov_correction, self.gamma_min, self.gamma_max)

        cg, sg = math.cos(math.radians(g_deg)), math.sin(math.radians(g_deg))
        cc, sc = math.cos(math.radians(c_deg)), math.sin(math.radians(c_deg))
        n_d = np.array([cg * cc, cg * sc, -sg])

        # ---7. speed magnitude (9)-(10) ---
        theta_deg = 0.0
        if speed_h_unit is not None:
            theta_deg = math.degrees(math.acos(
                clamp(float(np.dot(speed_h_unit, n_d)), -1.0, 1.0)))
        if terminal:
            k_a = self.ka_peak                      # full throttle, door bypass
        else:
            ratio_value = min(abs(theta_deg) / max(1e-6, self.theta_threshold), 1.0)
            k_a = self.ka_peak * math.cos(ratio_value * math.pi / 2.0)
        v_d = clamp(v_now + k_a, self.v_min, self.v_max)

        v_h = n_d * v_d

        # --- 8a. CLIMBING CEILING (section 3b-b): only v_z is clipped, the vector is NOT scaled. Scaling to
        # preserve orientation would also kill horizontal speed; The target is going to 20 m/s and the
        # horizontal budget is already tight. The sensor constraint (being able to see the target) is ABOVE
        # the guidance optimality.
        if v_h[2] < -self.climb_max_value:
            v_h[2] = -self.climb_max_value

        # --- 8b. vertical ceiling: first scale preserving DIRECTION, then crop ---
        vz = v_h[2]
        ceiling_value = WPNAV_SPEED_UP_MPS if vz < 0 else WPNAV_SPEED_DN_MPS
        if abs(vz) > ceiling_value:
            scale_value = ceiling_value / abs(vz)
            if v_d * scale_value >= self.v_min:
                v_h = v_h * scale_value                    # direction is preserved
            else:
                v_h = v_h * (self.v_min / v_d)
                v_h[2] = clamp(v_h[2], -WPNAV_SPEED_UP_MPS, WPNAV_SPEED_DN_MPS)

        # --- 9. NED'e cevir + emniyet ---
        yaw_r = 0.0 if o.yaw_rad is None else float(o.yaw_rad)
        v_ned = body_forward_ned(yaw_r, v_h[0], v_h[1], v_h[2])

        if o.pos_ned is not None:
            altitude_value = -float(o.pos_ned[2])       # pos_ned[2] down positive
            if altitude_value < self.min_altitude and v_ned[2] > 0.0:
                v_ned[2] = 0.0
        n = float(np.linalg.norm(v_ned))
        if n > self.v_max:
            v_ned = v_ned * (self.v_max / n)

        # --- 10. yaw (FOV) controller (11) ---
        yaw_rate = None
        if self.yaw_enabled:
            e = ex_ong
            e_dz = 0.0 if abs(e) <= self.yaw_dead_band else \
                e - math.copysign(self.yaw_dead_band, e)
            request_value = self.kp_yaw * e_dz + self.kd_yaw * ex_point
            request_value = clamp(request_value, -self.yaw_rate_max, self.yaw_rate_max)
            d_max = self.yaw_acceleration_max * dt_loop          # acceleration (slew) limit
            request_value = clamp(request_value, self.yaw_rate_output - d_max,
                            self.yaw_rate_output + d_max)
            a = (dt_loop / (dt_loop + self.tau_yaw_output)
                 if self.tau_yaw_output > 1e-6 else 1.0)
            self.yaw_rate_output += a * (request_value - self.yaw_rate_output)
            yaw_rate = self.yaw_rate_output

        # --- 11 . diagnostic window ---
        self.diagnostic = {
            'new_sample': new_sample, 'dt_measurement': dt_measurement,
            'range_value': range_value, 'range_source': range_source,
            'range_point': self.d_range.d,
            'area_root': o.area_root, 'ln_area_point': self.d_ln_area.d,
            'aim_active': aim_e, 'eps': eps, 'eps_ong': eps_ong,
            'ex_ong': ex_ong, 'ex_point': ex_point,
            'yaw_point': yaw_point,
            'q_az_point': q_az_point, 'q_el_point': q_el_point,
            'lead_az': self.lead_az, 'lead_el': self.lead_el,
            'n_active_az': n_active_az, 'n_active_el': n_active_el,
            'r_scale': R_scale, 'v_ref': v_ref,
            'sigma_az': wrap180(math.degrees(yaw_r) + ex_ong + self.lead_az),
            'sigma_el': g_deg,
            'c_deg': c_deg, 'g_deg': g_deg, 'n_d': n_d.copy(),
            'g_raw': g_raw, 'alt_angle': alt_angle,
            'fov_correction': fov_correction,
            'theta_deg': theta_deg, 'k_a': k_a, 'v_d': v_d,
            'v_now': v_now, 'terminal': terminal,
            'v_h': v_h.copy(), 'yaw_rate': yaw_rate,
        }
        return Command(vel_ned=v_ned, yaw_rate_dps=yaw_rate)


# ----------------------------------------------------------------- main

def arg_parser():
    p = argparse.ArgumentParser(
        description="LOS (PN) video guidance -- visual_base frame")
    g = p.add_argument_group('PN cekirdegi')
    g.add_argument('--k-pn', type=float, default=1.0,
                   help='LOS ratio gain;  1.0 = dead-beat (equation 5 )')
    g.add_argument('--tau-yak', type=float, default=0.8,
                   help='time constant [s] for approach to leading (equation 6)')
    g.add_argument('--tau-lead', type=float, default=20.0,
                   help='leading angle wash constant [s] (legacy1 K = 1/tau)')
    g.add_argument('--lead-az-max', type=float, default=35.0)
    g.add_argument('--lead-el-max', type=float, default=20.0)

    g = p.add_argument_group('measurement processing')
    g.add_argument('--tau-derivative', type=float, default=0.20)
    g.add_argument('--tau-yaw-point', type=float, default=0.20)
    g.add_argument('--latency-ek', type=float, default=0.05)
    g.add_argument('--reset-gap', type=float, default=0.60)

    g = p.add_argument_group('speed law')
    g.add_argument('--v-max', type=float, default=None,
                   help='default cfg.VISUAL_MAX_SPEED_MPS (currently 20). '
                        'DIKKAT: '
                        'skeleton (VisualLoop) also clamps with cfg value, '
                        'yani buradan 18 uzeri vermek SINGLE BASINA ETKISIZDIR; '
                        'guidance_config.VISUAL_MAX_SPEED_MPS should also increase.')
    g.add_argument('--v-min', type=float, default=4.0)
    g.add_argument('--ka-peak', type=float, default=2.0,
                   help='2LOSKF2 KA_PEAK')
    g.add_argument('--theta-threshold', type=float, default=35.0,
                   help='acceleration cosine life width [ deg ]')
    g.add_argument('--vertical-ratio', type=float, default=1.0,
                   help='k_perpendicular: 1.0 pure LOS; <1 retains the angle advantage')
    g.add_argument('--mount', type=float, default=None,
                   help='camera mounting angle [deg]; default $YILDIZ_MOUNT (0)')
    g.add_argument('--fov-margin', type=float, default=14.0,
                   help='|axis-target| allowed in raw framing angle [deg]')
    g.add_argument('--k-fov', type=float, default=1.0,
                   help='FOV protection gain; 0 = protection off')
    g.add_argument('--climb-max-value', type=float, default=3.0,
                   help='commanded UP speed ceiling [m/s]')
    g.add_argument('--gamma-min', type=float, default=-25.0)
    g.add_argument('--gamma-max', type=float, default=55.0)

    g = p.add_argument_group('terminal gate')
    g.add_argument('--terminal-range', type=float, default=12.0)
    g.add_argument('--terminal-area-root', type=float, default=55.0)

    g = p.add_argument_group('yaw (FOV) controller')
    g.add_argument('--yaw-disabled', action='store_true',
                   help='yaw_rate send (leave on autopilot) -- for A/B')
    g.add_argument('--kp-yaw', type=float, default=0.71)
    g.add_argument('--kd-yaw', type=float, default=0.917)
    g.add_argument('--yaw-dead-band', type=float, default=0.8)
    g.add_argument('--yaw-rate-max', type=float, default=60.0)
    g.add_argument('--yaw-acceleration-max', type=float, default=120.0)

    g = p.add_argument_group('geometry / safety')
    g.add_argument('--aim', type=float, default=None,
                   help='aim [deg]; verilmezse $YILDIZ_AIM (scenario.sh: 0)')
    g.add_argument('--back', type=float, default=None,
                   help='for current analytical derivation standoff back [m]')
    g.add_argument('--down', type=float, default=None,
                   help='for current analytical derivation standoff down [m]')
    g.add_argument('--min-altitude', type=float, default=12.0)

    g = p.add_argument_group('loop')
    g.add_argument('--loop-hz', type=float, default=20.0)
    g.add_argument('--tau', type=float, default=0.35, help='command LPF [s]')
    g.add_argument('--duration-value', type=float, default=None)
    g.add_argument('--log', type=str, default=None)
    return p


def controller_build(a):
    return LosController(
        k_pn=a.k_pn, tau_yak_s=a.tau_yak, tau_lead_s=a.tau_lead,
        lead_az_max_deg=a.lead_az_max, lead_el_max_deg=a.lead_el_max,
        tau_derivative_s=a.tau_derivative, tau_yaw_point_s=a.tau_yaw_point,
        latency_ek_s=a.latency_ek, reset_gap_s=a.reset_gap,
        v_max_mps=a.v_max, v_min_mps=a.v_min,
        ka_peak_mps=a.ka_peak, theta_threshold_deg=a.theta_threshold,
        vertical_ratio=a.vertical_ratio, mount_deg=a.mount,
        fov_margin_deg=a.fov_margin, k_fov=a.k_fov,
        climb_max_mps=a.climb_max_value,
        gamma_min_deg=a.gamma_min, gamma_max_deg=a.gamma_max,
        terminal_range_m=a.terminal_range,
        terminal_area_root=a.terminal_area_root,
        yaw_enabled=(not a.yaw_disabled), kp_yaw=a.kp_yaw, kd_yaw=a.kd_yaw,
        yaw_dead_band_deg=a.yaw_dead_band, yaw_rate_max_dps=a.yaw_rate_max,
        yaw_acceleration_max_dps2=a.yaw_acceleration_max,
        aim_deg=a.aim, back_m=a.back, down_m=a.down,
        min_altitude_m_value=a.min_altitude)


def main():
    a = arg_parser().parse_args()
    k = controller_build(a)
    print(f"[los] k_pn={k.k_pn} tau_yak={k.tau_yak}s tau_lead={k.tau_lead}s "
          f"aim={k.aim_deg:+.2f} deg ({k.aim_source}) "
          f"v=[{k.v_min:.1f}..{k.v_max:.1f}] m/s ka_peak={k.ka_peak:.1f} "
          f"yaw={'enabled_value' if k.yaw_enabled else 'disabled'} "
          f"(kp={k.kp_yaw} kd={k.kd_yaw}) k_perpendicular={k.vertical_ratio:.2f} "
          f"fov[ mount = {k.mount_deg:.0f} margin= {k.fov_margin:.0f} k= {k.k_fov:.2f} ] "
          f"climb_max_value={k.climb_max_value:.1f}", flush=True)
    VisualLoop(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_path=a.log).run_value(a.duration_value)


if __name__ == '__main__':
    main()
