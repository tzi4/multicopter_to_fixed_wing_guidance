#!/usr/bin/env python3
"""
mpc_guidance.py - DISPLAY guidance: Model Predictive Control (MPC) method
=========================================================== visual_base.VisualController
follows the convention. The decision variable is the DIRECT sequence of future 3D velocity commands;
Therefore, the MPC fits naturally on this platform (the same formulation could not be applied at
that time because 3D speed could not be commanded in the fixed wing).

------------------------------------------------------------------ MODEL Virtual-coordinate model
MPC in the presentation (old_los_codes/"Video guidance and ground detection.pdf", p.15-18):

    x = [u, v, Z]^T u,v: pixel coordinates of the target, Z: range
    u_input = [Vx,virt, Vy,virt, Vz,virt]^T
    u_{k+1} = u_k - (dt/Z_k) Vy,k
    v_{k+1} = v_k - (dt/Z_k) Vz,k
    Z_{k+1} = Z_k - dt Vx,k
    J = sum q1 u^2 + q2 v^2 + q3 (Z - Zref)^2 + r1 (Vx - Vprev)^2
    kisitlar: |u|<=umax, |v|<=vmax (FOV), Vmin<=Vx<=Vmax, V_NED = Rz(psi) V*

THE MODEL HERE IS THE SAME SAME THIS, with four differences (all true to the environment): ( 1 )
DEGREE INSTEAD OF PIXEL. The virtual gimbal already outputs ex_deg / ey_deg; This is exactly what
happens when the pixel pattern is divided by the focal length (small angle: u_px = fx * tan( ex ) ~
fx* ex_rad ). Thus, the dependence on 1280x720 is eliminated and the earnings become independent of
the camera. ( 2 ) DISTORTOR TERM d. The presentation model assumes the target is STATIONAL (u_dot
depends only on our own pace). Target 20 m/s is flying and deriving target speed from telemetry is
PROHIBITED (contract visual_base ). Instead, it is estimated from the IMAGE as d = (measured angle
derivative) - (model predicted angle derivative). d is physically "perpendicular velocity component
of target / range"; so the only target information MPC needs comes from bbox , NOT telemetry. ( 3 )
YAW SPEED 4 . ENTRY. The camera is fixed to the body and horizontal semi- FOV 33 deg ; It is
necessary to turn the nose to keep the target in the frame. Since the yaw speed is included in the
model, THERE IS NO SEPARATE FOV CONTROLLER -- MPC solves both steering and framing in the same cost
function. ( 4 ) REWARD LINEAR BBOX AREA. The term q3 (Z - Zref)^ 2 in the presentation penalizes
range by SQUARE, so the incentive to close is strong FAR FAR and weak NEAR -- the opposite of
      collision. Here the reward is directly bbox FIELD (w*h, px^2) and its FIRST DERIVATIVE. Since
      the field is ~ K/r^2, the incentive grows with 1/r^3 (1 units in 40 m, 8 units in 20 m):
      terminal aggressiveness increases consciously. Since they are both linearized around the
      nominal range orbit, ONLY the LINEAR term enters the cost -- the Hessian and the solution time
      do not change.
  (5) FOV HARD KIT. With visual guidance, the target leaving the frame ENDS the run, so framing is
  not a "choice" but a constraint. Key observation: the one-step-up framing variables are AFFINE on
  the input, meaning the vertical framing constraint boils down to narrowing the current VERTICAL
  SPEED BOX, and the horizontal framing constraint boils down to narrowing the YAW BOX -- both types
  of clusters that the projection already FULLY resolves. The hard constraint thus comes WITHOUT the
  COST of additional variables or additional iterations (see _dbf_bounds). Because it is written in
  the discrete form CBF, the cluster will NOT be emptied even when the target is already at the
  bottom edge at the time of handoff: the constraint says "heal", not "provide immediately". (6)
  ACTUATOR DELAY + ACCELERATION LIMIT. Chain: frame LPF (tau=0.35) -> |v|<=18 clamp -> speed loop of
  the autopilot, which WPNAV_ACCEL = 5 ACCELERATION with m/s^2 IT IS LIMITED. So the response to the
  speed command is not instantaneous, but RAMP: changing the lateral speed 15 m/s takes ~3 s. On
  model 1. The order is represented by tau = 1.0 s. Not putting this in the model produces overshoot
  and flicker; also the command acceleration is the Copter's TAKE ANGLE (atan(a/g)) and the roll is
  a direct matter of FOV as it rotates the camera -- so there is also an acceleration (|u - w|)
  penalty in the cost.

Critical distinction (PURE PURSUIT vs COLLISION COURSE): cost has both "ex -> 0" (framing) and
"INTERNAL LOS speed -> 0" (parallel course). The inertial speed LOS is the sum of the angle speed at
the sight + the speed yaw and SIMPLIFIES through the model:
        sigma_az = ex_dot + yaw_rate = -c_az * w2 + d_ex
In other words, only LATERAL SPEED (w2) resets the inertial LOS speed, turning the nose does not.
Putting this term in the cost automatically drives the MPC into proportional course (PN / collision
course): yaw frame makes lateral velocity steering. If only "ex -> 0" were placed, the optimization
would choose the cheapest path, yaw, and we would fall into tail chasing (18 m/s and 20 m/s never
reaching the target).

------------------------------------------------------------- GEOMETRY scenario.sh AIM=0 constants;
in that case the virtual framing center is aligned to the HORIZON: ey_deg = -(ascent of the target
relative to the horizon) [aim=0] general: ascent eps_deg = -(ey_deg + aim_deg) The mounting angle
comes from $YILDIZ_MOUNT (see Fig. environment_mount_deg); vertical semi-FOV ~20.07 deg. For the target to
remain in frame, its rise must be around (mount + body_pitch); On the ey axis, the CENTER of the
belt SLIDES with the body pitch:
        ey_ref = -(mount_deg + pitch_deg)
So "ey -> 0" (co-altitude) is NOT a FRAME target; The framing target is ey -> ey_ref and the band
FOV is around this center. What is required for collision is not a RESET of ascent, but a reset of
ascent SPEED (constant LOS + closing range = collision).

ASSEMBLY 0 PASSAGE (2026-08-04 user decision). In the actual hardware, the pitch-servo GIMBAL will
be used: it has been measured that the required mounting range of the FIXED camera, including the
wind, is spread out 36.3 deg and the vertical semi-FOV 20.07 is not enough. The 0 deg fixed camera ~
is the ideal gimbal emulation as the hopper barely tilts at all in the sim. New geometry: mount 0,
standoff back 25 / down 6 -> LOS rise
        eps0 = atan(6/25) = 13.50 deg
        beta_standoff = mount + pitch - eps = 0 - 2.5 - 13.5 = -16.0
        beta_carpma = mount + pitch - 0 ~ - 2.5 .. + 25 (brake) TWO IMPORTANT SIGN CHANGES: ( 1 )
        Target now starts ABOVE the axis instead of BELOW ( beta < 0 ). + 30 wanted the framing cost
        to be LOWER in the montage to center the target (type- 2 bottom dive regression);  mount 0
        also costs the same thing. CLIMBING -- so the vertical channel is now centered AWAY from the
        ground. Under-target depth ceilings are therefore rarely binding; It still remains as a
        safety deposit. ( 2 ) The risk of frame loss shifted from the LOWER edge to the TOP edge: in
        the approach phase, the target is at the top ( beta - 16 , edge - 20.07 ), in the terminal
        phase, when the brake lifts the nose, it shifts to the bottom. The bands were remeasured
        accordingly ( fov_upper_band / fov_alt_band ).

------------------------------------------------------------- SOLVER Condensed LTV-QP + FISTA
(accelerated projective gradient). Why not cvxpy/OSQP: target hardware Raspberry Pi 5, loop 20 Hz,
budget ~15 ms; also the constraint |v|<=speed_ceiling is a SPHERE, it requires SOCP not QP. With motion
blocking, the decision variable is reduced to the number 7 BLOCK x 4 INPUT = 28
(blocks=(1,1,2,2,3,4,7), total
n_step = 20 ; the first two steps are free one by one, the last block 7 step is fixed); Hessian
becomes 28x28. TIME ( 2026 - 08 - 05 ACTUAL RUN measurement, mpc_diagnostic duration_ms ): p50 7.3
ms | p95 13.4 ms | max 16.6 ms -- budget 13 ms , EXCEEDED (% 13 - 16 ). Hitting the Iteration
CEILING % 46 - 70 : i.e. the solution stalls not because it converges in most cycles but because it
runs out of budget
(intermediate point BASED on hot-start). (The old "1.7 ms/p95 2.3 ms" claim 18 m/s was a holdover
from a configuration with a roof and smaller horizon; TO_TEST item 7.) The ceiling is still
DETERMINISTIC (iteration ceiling + wall clock protection). MISSING METRIC: stopping criterion |dZ| <
It is tolerance_mps, so it says "my step has become smaller", it does NOT say "I am at optimum".
Optimality certificate (fixed-point residual) not detected -- TO_TEST item 7. Input constraint --
velocity sphere {|v|<=speed_ceiling} INTERSECTION vertical velocity slice PRODUCT yaw box -- CLOSED
FORM FULLY achieved by Euclidean projection (projection formula to sphere-slice intersection in
_projection_sphere_slice; scipy 1e-5 with SLSQP verified to m/s). The constraint FOV HAS TWO LAYERS:
(a) HARD layer -- written to the input box/slice with CBF and provided STRICT by projection; (b)
PLAN layer -- l1 FULL PENALTY (Huber-smoothed) along horizon, active in horizontal channel. No layer
can return "infeasible": the hard layer falls on best effort, the penalty layer always returns an
answer. Numerical notes (all measured, mpc_test.py): * Step size L constructs the Hessian of penalty
FOV ONLY from rows close to the violation; adding them all would bloat lambda_max by ~145 and make
FISTA unusable.
  * Proximal regularization with lambda_prox reduces the number of conditions from ~1.3e3 to ~2.7e2
  and limits the plan to per cycle (real-time iteration MPC). * Warm start: the solution is shifted
  one block; At the time of handoff, the final speed command of the positioner is the seed. The cold
  solution runs with a large budget (25 ms) in the first 2 cycle, then reaches full optimum in the
  3-5 cycle. #I STAYED HERE ------------------------------------------------------ HIT PHASE
  (2026-08-05) User request: "From the moment it sees the target, it accelerates and MULTIPLIERS on
  it without letting go". Two root causes were measured: (1) SPEED PARITY. The MpcConfig roof remained
  at 18 m/s, the frame went up to 35; target 21.05 m/s -> pure tail closure -3 m/s and 7/7 miss. The
  ceiling is now DERIVED from guidance_config (environment_speed_ceiling), meaning it cannot decompose again.
  (2) AT CLOSE RANGE THE COST WAS STILL A "FOLLOW-UP" COST. Framing protection (hard FOV constraint
  + brake throttling) is right for long-term tracking, wrong at the last second. STRIKE opens the
  FOV bands to the physical edge with a CONTINUOUS mixing factor with phase range (0 at 22 m, 1 at 8
  m), absorbs braking/acceleration throttling, enlarges framing and field reward weights, reduces
  the acceleration penalty. Detailed justification MpcConfig impact_* on your blog. Since the camera
  is 0 deg FIXED, the target leaves the frame leaves us BLIND; so it does NOT release phase framing,
  it just converts the centering request into an edge request. If bbox becomes stale in the last < 1
  s, the solver does not run and the last command is repeated (blind float, see _blind_command).

----------------------------------------------------------- HEAR MODE A state machine ABOVE MPC,
OUTSIDE the cost function: CLOSE -> TERMINAL -> HIT -> HIT -> (release authority) Why not in the
cost: it is CORRECT in terms of cost to produce the command towards the target even as the range is
opened (LOS speed, area award, framing -- all continue to point to the target). “I'm over it, I
should quit” is a TERMINATION decision, not an optimum; optimization can't find it. Pass detection
DOES NOT TOUCH THE TARGET TELEMETRY: its own derivative of the range (internal_range, LPF) OR the sign of the
growth rate of the bbox field. Detailed rationale and measurements are on MpcConfig's miss_* blog.
Reference design: formation_KILLER._attacker_intercept_thread (ARM/OPEN/TERMINAL/TIMEOUT binaries
set in real hardware).

USAGE
    python3 mpc_guidance.py                  # live (waiting for authorization)
    python3 mpc_guidance.py --no-miss        # ablation: old behavior
    python3 mpc_test.py                   # offline verification + duration
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

_HERE = str(Path(__file__).resolve().parent)
if _HERE not in sys.path:                       # Also in direct operation
    sys.path.insert(0, _HERE)                   # find sister modules

from visual_base import (VisualController, Command, Measurement,   # noqa: E402
                             body_forward_ned)

KDEG = 180.0 / math.pi          # rad/s -> deg/s and 1/m -> deg/(m/s)/m


def environment_mount_deg(default_value2: float = 0.0) -> float:
    """The elevation of the camera AXIS relative to the horizon FROM A SINGLE SOURCE.

    GIMBAL BRANCH (2026-08-05): the sim now has a REAL, self-stabilizing single-axis gimbal
    (measured at the end: body +-35 deg while the camera skids the world pitch 0.65 in deg). Camera
    axis = COMMANDED tilt, which is derived from the geometry standoff: $YILDIZ_TILT =
    atan(down/back) (exports scripts/standoff_geom.sh). PRIORITY ORDER: 1. $YILDIZ_TILT (physical
    gimbal command -- new single source) 2. $YILDIZ_MOUNT (frozen body-fixed arms/old runs) 3.
    default 0.0 The lesson of not writing constants in the code is the same: In 2026-08-04, mounting
    the stale constant in the 30 -> 0 transition led to complete frame loss.
    """
    for key_value in ('YILDIZ_TILT', 'YILDIZ_MOUNT'):
        value_value = os.environ.get(key_value)
        if value_value is not None:
            try:
                return float(value_value)
            except (TypeError, ValueError):
                pass
    return float(default_value2)


def _environment_count(key_value: str, default_value2: float) -> float:
    """Read numbers from the environment; If it's broken/not there, go back to default."""
    try:
        value_value = os.environ.get(key_value)
        return default_value2 if value_value is None else float(value_value)
    except (TypeError, ValueError):
        return default_value2


def _empty_nan(x, format_value: str = '.3f') -> str:
    """NaN / None -> empty cell, otherwise formatted number.

    CAUSE: If "mechanism OFF" and "value 0" are read in the same column in the diagnosis CSV, the
    measurement is incorrect (a tau or t_go of 0 s is physically impossible, but if 0 is written it
    will distort the average).
    """
    try:
        v = float(x)
    except (TypeError, ValueError):
        return ''
    return '' if not math.isfinite(v) else format(v, format_value)


def environment_q_area_multiplier(default_value2: float = 1.0) -> float:
    """YILDIZ_Q_AREA_MULTIPLIER -- bbox AREA prize multiplier (A/B button).

    TO_TEST item 1: closure is coded as a PURPOSE and not a BONUS; enlarging the reward 22-45 m
    breaks the "flying sideways" blockage. Since the reward goes ONLY LINEAR into the cost (see
    _field_reward) Hessian, the number of conditions and solver time DO NOT change -- a free button.

    Offline measurement, n=32/group with gimbal physics, crossing+lateral x straight+ellipse:
    multiplier 1, default: stalled closure 25%, timeout 75%. Multiplier 4: stalled closure 75%,
    timeout 41%. Multiplier 4 with reference horizon: stalled closure 100%, timeout 0%. Tail
    geometry remained unchanged at 67% in every branch. Default 1.0 preserves the previous behavior.
    Use YILDIZ_Q_AREA_MULTIPLIER=4 for A/B comparison.
    """
    return _environment_count('YILDIZ_Q_AREA_MULTIPLIER', default_value2)


def environment_horizon_range_ref(default_value2: float = 0.0) -> float:
    """YILDIZ_HORIZON_RANGE_REF -- Scales the COST horizon with range (0 = OFF).

    TO_TEST item 3. The COST horizon already scales with range (_dbf_limits, T = cbf_prediction_s * r /
    cbf_range_ref_m) but the COST horizon remained CONSTANT 2.4 s -- measured: median of t_go 0.61
    s, i.e. the horizon at STRIKE ~%73 AFTER COLLISION. The asymmetry was here.

    WHY NOT t_go RANGE: Scaling with t_go has already been tried and turned out to be INeffective in
    diagonal geometry (closure ~0 -> t_go infinite -> scaling does not come into effect at all; see
    note in _cbf_limits).

    If given, each step time is step_s * clip(r/ref, base, 1). Measured running band: ref 45-60 m.
    35 and 20 ARE INACTIVE.
    """
    return _environment_count('YILDIZ_HORIZON_RANGE_REF', default_value2)


def environment_vertical_terminal(default_value2: float = 0.0) -> float:
    """YILDIZ_VERTICAL_TERMINAL -- ALTITUDE-AGNOSTIC terminal vertical alignment.

    0 = OFF (DEFAULT, current behavior is preserved verbatim), 1 = ON.

    WHY THERE IS (2026-08-07 measurement, straight heading, three vertical standoff): DOWN +4
    (fighter below) : CPA 2.53 m, VERTICAL in CPA now +0.92 m DOWN 0 (same altitude): CPA 1.15 m,
    vertical residual -0.56 m DOWN -3 (fighter at top): CPA 1.42 m, vertical residual +0.64 m DOWN
    +4 range profile of vertical separation in run (|dz| median): 45-35 m: 3.41 35-25 m: 3.58 25-15
    m: 3.98 15-8 m: 2.34 So the vertical component of standoff DOES NOT CLOSE AT ALL until 15 m;
    Closing is left until the last 15 m, that is, ~1 seconds.

    REASON: the only term CLOSING vertical standoff in cost was q_ey and that term is defined on
    beta = ey - ey_ref. With PHASE O (gimbal tilt TRACKS the rise of the target) ey_ref is connected
    to the LIVE axis (_framing_constant), i.e. ey_ref -> -eps. So it becomes beta ~ 0: the camera
    frames the target perfectly and THE COST NO LONGER SEE THE VERTICAL ERROR. The only remaining
    vertical term is the term PN, which drives the vertical speed LOS sigma_el -> 0; its exact
    meaning is "PARALLEL NAVIGATION = PROTECT vertical standoff" (see _cost_lines). The measurement
    confirms this: 25-35 in the m band |dz| fixed ~3.5-4.0 m. While the gimbal solved the framing
    problem, it also took away the signal that maintained the vertical shutdown.

    MECHANISM (this button): ALTITUDE-AGNOSTIC and EARLY version of the old STROKE vertical
    alignment bias (stroke_align_*, retired). Vertical to speed reference LOS sigma_el_ref = s *
    clip( dead_band ( eps ) / tau , -ceiling, +ceiling) bias is placed; Since eps_dot = - sigma_el ,
    this is a request that drives eps (and hence dz = r* sin ( eps )) TO ZERO. There are THREE
    differences from the old one: ( 1 ) TWO SIDED: eps < 0 also works when (fighter on top) ->
    altitude-agnostic. ( 2 ) depends on the TERMINAL RANGE ( 45 m), not the STRIKE mix ( 22 m): the
    alignment spans the window of ~ 3 s, not the last second. ( 3 ) Disabled if bbox is stale: If
    eps comes from a rotated measurement, a self-sustaining vertical demand occurs (see
    impact_blind_coast ). SAFETY: touches only COST REFERENCE. The vertical speed box
    (ascent 9 / descent 4.5 m/s) stands EXACTLY as the input box in resolve() altitude floor CBF and
    depth ceiling; the reference does NOT pierce them.
    """
    return _environment_count('YILDIZ_VERTICAL_TERMINAL', default_value2)


def environment_vertical_error(default_value2: float = 1.0) -> float:
    # DEFAULT ON (2026-08-10 user decision): proof fixes do not require flags -- bare `python3
    # mpc_guidance.py` flies with champion setup. Expectations for mpc_test were updated the same day,
    # validating the old design (legacy-behavior tests clearly pin the flag to 0). OFF (old behavior):
    # YILDIZ_VERTICAL_ERROR=0 YILDIZ_VERTICAL_TGO=0
    """YILDIZ_VERTICAL_ERROR -- DIRECT vertical error (P) term in cost.

    1 = ON (DEFAULT since 2026-08-10), 0 = OFF (legacy behavior). INDEPENDENT from
    YILDIZ_VERTICAL_TERMINAL; Both can be opened together.

    VALUE IS A MULTIPLIER (on q_vertical_error): 1 = nominal weight, 2 = double, 0.5 = half. WHY NOT A
    SEPARATE ENV: the weight is the ONLY adjustment knob of this movement, and the first sim
    measurement (below) showed that the exact weight was INSUFFICIENT -- the knob itself is read as
    a multiplier in order to do the scanning with a single variable.

    WHY THERE IS (THE SECOND HALF OF THE ROOT CAUSE). environment_vertical_terminal() documents that after
    PHASE O (the gimbal tilt TRACKS the target rising) the cost NO LONGER sees the vertical error as
    beta = ey - ey_ref ~ 0. The previous round put a DERIVATIVE term in this space: bias to the
    vertical LOS SPEED reference (sigma_el_ref). Measured (2026-08-07, flat+ellipse, DOWN +4):
    mechanism running : alignment_ref active frame %65-83, terminal eps 7.0 -> 4.5 deg, 20-8 in m band
    |dz| 2.59 -> 1.98 m BUT the vertical in CPA has NOT fallen anymore (pool median 0.99 -> 1.09)
    Classic "D without P" signature: error STARTS TO CLOSE but DOES NOT CLOSE FULLY. The derivative
    term only penalizes CHANGE of the error; there is no line left in the cost for the error ITSELF.

    MECHANISM (this button): an error line is added to the cost DIRECTLY on the status ey. Geometry
    exact (see command(): eps = -(ey + aim)): eps -> eps_target <=> ey -> ey_target = -(eps_target +
    aim) i.e. target "line" (eps = 0 plane) is a FIXED reference in the case of MPC; No separate
    estimation is required. * REFERENCE DISCRIMINATION (PHASE GAIN IS NOT DISTURBED): framing term
    (q_ey) and FOV/CBF constraints REMAIN on ey_ref = LIVE gimbal axis -- the camera continues to
    center the target. ONLY this new line uses the "target line" reference. Two references are on
    the same situation but in DIFFERENT tasks: one is SEE, the other is HIT. * RAMP TO ZERO AT
    TERMINAL: weight increases from 0 to full value with range 45 -> 25 m. NO away term, i.e.
    standoff (DOWN parameter) is retained AS IS and the approach profile does not change; in the
    terminal the target eps = 0, i.e. vertical standoff is melted. * TWO-SIDED: eps>0 (target above
    us) and eps<0 (below us) symmetric -> DOWN -4..+4 works on all, altitude-agnostic. * DEAD TAPE:
    |eps| When <= relaxed the WEIGHT of the row is ZERO (not reference shifting). So in the small
    |eps| the solution is BIT-SAME as the CLOSED arm and the noise bbox (+-1-2 px) is not translated
    to the vertical command. * STALE BBOX GATE: If eps comes from a rotated measurement, the term
    produces self-fulfilling vertical demand (same trap measured on impact_blind_coast and
    vertical_terminal) -> stale ramp 0.
    WHY ON eps (angle), NOT ON dz (meter): NOT LINEAR in case dz = r*sin(eps) (r is also a case);
    The angle row is single column and does not change the Hessian, so the solution time remains the
    SAME. Moreover, the constant angular deadband automatically tightens the METRIC tolerance as the
    range closes: 1 deg = 25 m at 0.44 m, 10 m at 0.17 m -- our acceptance criterion (|vertical| in
    CPA < 0.4 m) is exactly also wants this behavior. SAFETY: only affects COST. The vertical speed
    box (ascent 9 / descent 4.5 m/s) stands EXACTLY as the input box in resolve() altitude floor CBF
    and depth ceiling; cost will NOT break them.
    """
    return _environment_count('YILDIZ_VERTICAL_ERROR', default_value2)


def environment_vertical_tgo(default_value2: float = 2.0) -> float:
    # Opens WITH YILDIZ_VERTICAL_ERROR (both were measured together in the campaign); DEFAULT 2.0 (2026-08-10)
    # -- flown and verified with champion setup multiplier=2, see Decision at the beginning of
    # environment_vertical_error(). This lever cuts the OVEROVER: at fixed tau e_dot was not zeroing at all
    # (measured -4.5..-5.1 m/s vertical speed @CPA); With t_go/k both e and e_dot go to zero.
    """YILDIZ_VERTICAL_TGO -- shapes the vertical speed reference with TIME REMAINING.

    DEFAULT 2.0 (ON since 2026-08-10); 0 = OFF (legacy behavior). VALUE IS A MULTIPLIER (on
    vertical_tgo_k): 1 = nominal k, 1.2 = 20% harder damping. Since the only knob is k (tau_min is a
    safety base, not a setting) the knob itself is the multiplier so that sensitivity scanning can
    be done with a SINGLE env -- same pattern as YILDIZ_VERTICAL_ERROR.

    WHY THERE IS (THIRD AND FINAL PART OF THE ROOT CAUSE). The previous two rounds were measured
    (see environment_vertical_terminal / environment_vertical_error): * D alone (VERTICAL_TERMINAL): error STARTING to
    close, not FULLY closed -- "D without P". * P alone ( VERTICAL_ERROR = 1.5 ): error closes
    monotone FIRST TIME (ellipse DOWN+ 4 , |dz| median 25 - 35 m 3.83 -> 8 - 15 m 0.92 -> 0 - 8 m
    0.67 ; in the closed arm 3.29 / 2.07 / 1.51 ) BUT the residual CPA DOES NOT FALL BECAUSE IT
    REACHES ZERO VERY RAPIDLY: zero crossing 0.20 s before CPA, vertical velocity @CPA - 4.5 ..- 5.1
    m/s -> 0.9 - 1.2 m VINE. The sign agrees: the disabled branch is + 2.02 (lags behind), while the
    P branch is - 1.10 /- 1.22 (overshoots). * P+D together do not solve it because the D time
    constant
        vertical_terminal_tau_s = 1.5 s is fixed and exceeds t_go at r < 15 m
        (< 1 s) LONG. So in the terminal D says "slow down", P says "close_value"; result EARLY
        BRAKING -- zero crossing NEVER occurs in ellipse, 0 - 8 |dz| in m band It remains at 2.20 .
        The common root cause is ONE: the reference says "reset error", not "reset error AND SPEED
        TOGETHER". Fixed tau imposes a TIME SCALE; In the collision problem, the only meaningful
        time scale is REMAINING TIME.

    MECHANISM: Time constant of arm D is connected to t_go instead of CONSTANT tau
        tau_eff = clip(t_go / k, tau_min, tau_max)
    i.e. (in the band where it is not clipped) the reference becomes sigma_el_ref = vertical_s * k *
    eps_excess / t_go. This is a ZEM/PN law: "according to the time remaining".

    WHY RESET THIS EXCESS (closed form, essence of justification). Let eps_dot = -sigma_el and
    (approximately) constant closure t_go = T - t: de/dt = -k e / (T - t) => e(t) = e0 * ((T-t)/T)^k
    i.e. for k > 1 BOTH e and e_dot = -k e0 (T-t)^(k-1) / T^k goes to ZERO on collision. At constant
    tau then e(t) = e0 exp(-t/tau): e becomes small but e_dot NEVER resets -- measured -4.5..-5.1
    m/s vertical velocity @CPA is exactly that. The physical vertical now behaves even better: dz ~
    dz0 * ((T-t)/T)^(k+1), d(dz)/dt ~ (k+1) dz / (T-t) -> since dz = r sin(eps) and r ~ r0 (T-t)/T
    0. So choosing k > 1 is a "critical end-like" terminal profile; As k grows, the shutdown becomes
    LATE and HARD (the error stalls for a long time, then dissolves quickly), at k -> 1 the hang
    comes back. For the selected k and the measured knee point, see MpcConfig.vertical_tgo_k.

    SECOND AND MANDATORY PIECE -- CEILING. The law only works as long as its demand is NOT CUT. The
    ceiling of the old (constant tau) arm is a CONSTANT angular velocity (8 dps) and this was
    precisely strangling the law at the terminal, as it changes meaning with range: clipped demand
    means CONSTANT eps_dot, constant eps_dot is precisely the persistent descent problem. In the TGO arm the ceiling is the
    PHYSICAL INPUT BOX (climb 9 / descent 4.5 m/s), selected by direction and converted to angular-rate limits
    using the current range. Measured: without this correction, increasing k changed NOTHING in eps >=
    8 deg (vertical speed @CPA constant between k=2.5 and k=8 -3.35 m/s). See Ceiling block in
    _cost_lines.

    WHY COMBINE WITH P (VERTICAL_ERROR). This lever shapes the SPEED reference only; there is still no row in
    the cost for the error ITSELF (that row is that of branch P). Without P the law is still "D
    without P" -- just a smarter D. The main branch is therefore P + TGO; TGO can also be opened
    alone (for ablation).

    SOURCE t_go (NO NEW ESTIMATE): self.range_rate_value that the controller ALREADY calculated (=
    d(internal_range)/dt, LPF tau 0.30 s; this is also used in the transition closing-speed condition) and the
    inner range r. t_go = r / (-range_rate_value). INVALID/STALE FALLS ON THE SAFE SIDE: if the closing speed
    is below vertical_tgo_closure_min_mps (opening, cross geometry, LPF warm-up for the first 0.3 s)
    ignore t_go and tau_eff = tau_max is taken -- i.e. the SOFTEST request, legacy fixed-tau
    behavior. DO NOT fall on the aggressive side. SAFETY: only touches COST REFERENCE, same line as
    vertical_terminal. The vertical speed box (ascent 9 / descent 4.5 m/s) stands EXACTLY as the input
    box in resolve() altitude floor CBF and depth ceiling; the reference does NOT pierce them. So is
    the existing bbox gate (via vertical_s). Hessian DOES NOT GROW: no new row, the REFERENCE of the
    existing row changes -- solver time is the same.
    """
    return _environment_count('YILDIZ_VERTICAL_TGO', default_value2)


def environment_solver_abundant(default_value2: float = 0.0) -> float:
    """YILDIZ_SOLVER_ABUNDANT -- Give FISTA PLENTY of budget (for SIM MEASUREMENT QUALITY).

    0 = OFF (DEFAULT, today's values ​​are literal: iteration_ceiling 26 , duration_budget_ms 13 ).  1 =
    ON -> 40 iteration, 18 ms .

    *** THIS IS NOT A PERFORMANCE BUTTON, IT IS A MEASURING TOOL. ACTUAL HARDWARE (Pi 5) DECIDED
    SEPARATELY -- see. WARNING below. ***

    WHY IS THERE (measured root cause). In the Tur-5 A/B table, the rate of 'budget_cut' in the
    diagnosis CSV was %52-89 IN EVERY ARM (including baseline). So FISTA rotates in most of the
    frames, WITHOUT CONVERGING to tolerance_mps (2 cm/s), hitting either the iteration ceiling (26)
    or the duration budget (13 ms). This means: the instruction given at each frame is NOT the
    OPTIMAL of that frame's QP, but an RANDOM POINT on the path to the optimum -- and that point
    depends on how much CPU the frame gets (OS timing, Gazebo load, log writing). Therefore, EVERY
    A/B difference measured is superimposed on noise that is UNRELEVANT to the experiment and varies
    from run to run. This noise was seen concretely: two runs of the arm k=2.0 in the ellipse DOWN+4
    with the SAME configuration 15-8 in the m band |dz| 0.19 and 2.78 yielded m -- meaning the
    INTRA-CONFIGURATION spread is greater than the INTER-CELL difference we were trying to measure.
    This is the reason why k cannot be selected with n=1-2.

    MECHANISM: ceiling 26 -> 40 (%54 more iterations), budget 13 -> 18 ms. NOTHING else changes --
    same cost, same constraints, same tolerance. The only expected effect: the solution gets MORE
    close to the optimum, and the question "how close" is attributed to the PROBLEM ITSELF rather
    than the machine load (determines tolerance, not clock).

    WHICH WAS CONNECTING -- MEASURED (offline closed loop, MAIN ARM P+TGO k=2, 24 scenario = 4 route
    x 3 handoff x 2 seed, N~2590 resolve() call): NORMAL 26/13 : budget_cut %64.5 | % of iteration
    67.5 ON CEILING iteration avg 22.9 p50 26 | duration p50 5.4 p95 7.8 p99 13.1 ms ABUNDANT 40/18 :
    budget_cut %49.5 | % of iteration 51.9 ON CEILING iteration avg 30.8 p50 40 | duration p50 6.1
    p95 10.3 p99 13.1 ms Two things are clear: (1) IT WAS NOT THE TIME BUDGET BUT THE ITERATION
    CEILING THAT BOUND. In the normal arm, the time is p95 = 7.8 ms while the budget is 13 ms; In
    other words, almost all of the interruptions were due to "Iteration 26 is over". 13 -> 18 ms by
    itself would change almost nothing; 26 -> 40 DOES THE JOB. (2) STILL NOT CONVERGING: %49.5 still
    high. So tolerance_mps (2 cm/s) for this problem is also not reached in iteration 40 -- the
    problem is not only the budget, but the convergence SPEED of FISTA in this scaling. Increasing
    the budget HALF the noise, it does not reset it. Price: solver time p50 +0.7 ms, p95 +2.5 ms.
    p99 SAME (13.1 ms), because the cold start solutions that specify p99 are already large budget.

    IS BEHAVIOR CHANGING? YES -- THIS IS NOT A PURE "MEASURING" BUTTON. Offline closes the ABUNDANT arm
    CLOSER (min range median 24 in scenario 7.84 -> 7.16 m; mpc_test 5-closed-loop median 6.41 m).
    Expected direction: the solver does better what the cost WANTS. The cost has also been measured:
    Framing loss with CORRECT mounting on test_mounting_zero %0.0 -> %9.2 (min range 11.9 -> 7.0 m),
    meaning that the harder closing moves the target more towards the FOV edge. Since that test uses
    a RATIO criterion that says "wrong mounting should cause frame loss 3 TIMES THE CORRECT", IT
    REMAINS WITH PLENTY=1 (%20.8 vs 3 x %9.2 = %27.6) -- the distinction remains ABSOLUTE (wrong
    mounting every It is bad on both axes: %20.8 vs %9.2 loss, 4.9 vs 7.0 m min range) but since the
    denominator of the ratio criterion grows, the threshold does not hold. The warehouse door
    (mpc_test 86/86) is not affected because the DEFAULT is OFF; If it is run with ABUNDANT=1, 85/86 is
    expected and THIS is the reason.

    *** SIM CLOSED LOOP PROVISION ( 2026 - 08 - 07 , eng- 5 ): THE BUTTON WORKS MECHANICALLY BUT
    DOES NOT BREAK THE NOISE -- IT REMAINS OFF BY DEFAULT. *** Conditioned: ellipse DOWN+ 4 and
    plain DOWN+ 4 , MAIN ARM, DURATION= 300 . MECHANICAL (intended effect, REALIZED): budget_cut %
    52 - 78 (n= 4 ) -> % 43 - 54 (n= 10 ) % a t ceiling iteration 48 - 69 -> % 29 - 41 command() p95
    [ ms ] 13.2 - 13.4 -> 18.3 - 19.1 loop dt p50 [s] 0.050 -> 0.050 ( 20 Hz PROTECTED) Note: In the
    NORMAL arm, the command() p95 is full 13.2 - 13.4 ms , that is, the time budget in the sim was
    also CONNECTED (it was not connected offline, because sim has payload Gazebo ). Both mechanisms
    relaxed as intended. MEASUREMENT QUALITY (PRIMARY PURPOSE, NOT ACHIEVED): * In-run spread was
    NOT narrowed, played mixed: 15 - 8 in m band |dz| k= 2 ellipse [ 0.19 .. 2.78 ] -> [ 0.88 ..
    2.18 ] (narrowed) AMA CPA vertical is now in the SAME cell [ 0.15 .. 0.50 ] -> [ 0.09 ..1.59 ]
          (genisledi).
        * WORSE, THERE IS A SHIFT WITH THE SIGN: REGARDLESS OF THE ARM in the ellipse DOWN+4 (k=2
        and k=3 together) CPA vertical is now NORMAL n=4 : 0.15 0.37 0.50 0.64 (all <= 0.64) ABUNDANT n=6
        : 0.09 0.13 1.39 1.50 1.59 1.74 (median 1.45) So ABUNDANT=1 WORSES OUR ACCEPTANCE CRITERIA
        (|vertical| in CPA). COMMENT (SAME ROOT as the offline assembly finding): the better
        convergent solver achieves more faithfully what COST WANTS -- and cost ALONE does not want
        the terminal vertical residual; PN/seeks a compromise in terms of framing. Increasing the
        budget leads to that compromise faster, so the "nonconvergence" is not a NOISE, but a
        deviation that works partially in our FAVOR. VERDICT: the source of the noise is NOT the
        solver budget. Lowering budget_cut from %62 to %46 did not close the in-run spread; So the
        driver of the propagation is different (target's route phase / handoff geometry, bbox noise,
        number of engagements). The button remains OFF; A/Bs are run in the NORMAL budget and
        decided by increasing n. ONLY LEGITIMATE USE: "is this difference coming from the solver
        budget?" To ask the question again, as an ablation switch.

    WHY 18 ms SAFE ON SIM: homing loop budget on sim machine 50 ms (20 Hz; measured Hz in runs
    19.8-19.9, i.e. cycle does not fill already full) and measured solver time p95 ~13-15 ms. 18 ms
    ceiling means 36% of the cycle; More than enough for remaining 32 ms MAVLink + log + bbox. So in
    the sim, this button does NOT reduce the cycle speed, it only aims to make the interruption
    rare.

    *** WARNING -- REAL HARDWARE (Pi 5) DOES NOT GET THIS VALUE AUTOMATICALLY. *** Even though the
    cycle budget on the flight computer is the same 50 ms the CPU is much slower and SHARED (bbox
    inference is on the same board). There 18 ms the ceiling MAY MISS THE LOOP instead of reducing
    truncation -- which loop miss is much worse than a non-converged instruction (see memory:
    "Goduem loop dt root cause" -- 2 The loop falling on Hz had produced giant circles). Hardware
    decision must be measured SEPARATELY: on the Pi 5 this switch will NOT be turned on without
    measuring the solver time p50/p95 and the loop dt distribution. The default so remains OFF and
    the button is EXPLICITLY given by env in sim runs.
    """
    return _environment_count('YILDIZ_SOLVER_ABUNDANT', default_value2)


def environment_apn(default_value2: float = 0.0) -> float:
    """YILDIZ_APN -- Estimate TARGET LATERAL ACCELERATION and SPRING TO THE HORIZON (APN).

    0 = DISABLED (default, preserving prior behavior bit for bit). >0 = ENABLED. The value also scales the a_perpendicular contribution
    (1.0 = full APN, 0.5 = half for ablation).

    COOK CAUSE (measured, ellipse target, lap-8 pool). The disturbance estimator produces SINGLE
    scalar d_ex; its physical meaning is KDEG*v_perpendicular/r, i.e. "SPEED of the target perpendicular to
    LOS". Assuming v_perpendicular CONSTANT while spreading over the horizon:
        d_ex_k = d_ex0 * r0/rbar_k  ==  KDEG*v_perpendicular0/rbar_k
    This is true for the STRAIGHT flying target and there the law works like pure PN (CPA 0.11-0.92
    m, contact range). Incorrect WHILE target ROTATING: v_perpendicular not fixed, |a_perpendicular| It varies by ~
    v^2/R. 92-99% of the measured miss vector is in the "outside of the turn" direction -- that is,
    the plan always searches for the target on the previous tangent. CPA on return 1.9-2.1 m.

    FIX (equivalent of the classical APN in these coordinates): estimate a second derivative (a_perpendicular
    = d(v_perpendicular)/dt) and LINEAR arc on the horizon:
        d_ex_k = (v_perpendicular0 + a_perpendicular*t_k) * KDEG / rbar_k
    APN is the optimal generalization of PN under constant target acceleration; here it only enters
    the FREE RESPONSE (Xf) -- it does NOT touch the Gam and Hessian, so solver cost and time do not
    change.

    WHY IT'S SAFE: * |a_perpendicular| <= apn_a_ceiling_mps2 (6 m/s^2) physical clamp. * Derivative noisy;
    SEPARATE and slower LPF (apn_tau_s). * DEAD BAND (apn_dead_band_mps2): estimation below noise
    floor is RESET -> behavior remains unchanged on FLAT target. * Multiplied by the disruptive
    trust ramp (DisturbanceEstimator.trust). * When r < disturbance_rotate_range_m, it FREEZES SAME as d_ex.
    """
    return _environment_count('YILDIZ_APN', default_value2)


def environment_actuator(default_value2: float = 1.0) -> float:
    """YILDIZ_ACTUATOR -- SATURATED (acceleration limited) actuator model.

    >0 = ON (DEFAULT since 1.0, 2026-08-10). 0 = OFF (old behavior BIT-SAME). CAUTION: Constants
    tau_lin/a_max ARE MEASURED at the iris; In a new vehicle (hummingbird etc.) step-response
    measurement should be repeated and YILDIZ_TAU_LIN / YILDIZ_A_MAX should be updated.

    Root cause measured in three independent runs on 2026-08-08, round 2: MPC uses w <- w + al*(U -
    w), al = h/(h + speed_latency_tau_s), tau = 1.00 s. This first-order actuator model has no
    acceleration limit. Larger error requests larger acceleration, a = |e|/tau, giving 15 m/s^2 for
    |e| = 15 m/s. The real vehicle is acceleration-limited. Measurement with horizontal e = cmd_v -
    vel: a_par = (dv/dt). e_unit ; conditions base8 / tyaw1 / tyawacc): PLATO (|e|> 8 m/s , a_par
    p90): 4.07 / 3.81 / 3.96 m/s ^ 2 linear tau (|e|< 4 m/s ): 2.33 / 1.56 / 1.74 s tau_active (|e|=
    10 - 20 m/s ): 5.47 / 6.75 / 6.58 s <-- MOST FREQUENT BAND So the model assumes a time constant
    in the studied band ~ 1 / 6 of the reality: MPC schedules an UNREACHABLE command, then
    reschedules each cycle and issues the same unattainable command again. NOTE: The restoration of
    WPNAV_ACCEL 250 -> 500 did NOT change the PLATO (speed setpoint path GUIDED passes through PSC,
    parameter of WPNAV_* waypoint controller); So this is not a parameter job, it is a MODEL job.

    CORRECTION (SUCCESSIVE LINEARIZATION of saturation). actual behavior
        w_dot = clip((u - w)/tau_lin, +-a_max)
    It fits into the LTV frame as ACTIVE TAU PER STEP:
        tau_eff_k = max(tau_lin, |u_nom_k - w_nom_k| / a_max)
        al_k      = h_k / (h_k + tau_eff_k)
    When the error is small it is tau_lin (linear region), when it is large |e|/a_max (saturation
    region) -- in the latter case al*|e|/h = a_max, i.e. full acceleration ceiling. Since the
    coefficients are ALREADY step-by-step (LTV), the STRUCTURE of QP is preserved; VALUES of the
    Hessian change (inevitably, solver time must be measured).

    BOTH AXES. In the first version ONLY HORIZONTAL was implemented, vertical was left in the old
    1.00 s ("also not measured, conservative"). THIS PRODUCED A FAILURE: the vertical remained
    artificially CHEAP and the solver shifted demand there (|u3| p90 2.2 fold, ALTITUDE
    ABORT/engagement 0.36 -> 0.69). Vertical has also been measured (tau_lin_z 2.16 s, plateau 5.25
    m/s^2 -- so vertical is SLOWER than horizontal) and is applied symmetrically. The _dbf_limits
    vertical slice also sees the SAME gain (argument tau_v), otherwise the constraint would assume
    "I'm fast" when the plan says "I'm slow".

    Override settings through environment variables: YILDIZ_TAU_LIN (default 1.7 s), YILDIZ_A_MAX (default 4.0
    m/s^2).
    """
    return _environment_count('YILDIZ_ACTUATOR', default_value2)


def environment_progress_clock(default_value2: float = 0.0) -> float:
    """YILDIZ_PROGRESS_CLOCK -- Link WAT timeout to PROGRESS.

    0 = DISABLED (default, preserving prior behavior bit for bit). >0 = ENABLED.

    COOK CAUSE (2026-08-08, measured). Timeout IS WALL CLOCK: elapsed = t - authority_t0 >
    miss_time_timeout_s (8 s) -> WATCH So its gate is "how long have I been in authority", whereas
    what we want to ask is "ARE I PROCEEDING". Two measurements: * Lap-1 pool (base8 + apn1 + apn1t,
    34 engagement): 4 (%27%) of 15 timeout cut while the vehicle is STILL OFF -- last 2 s range
    slopes -4.11 / -3.59 / -2.11 / -1.34 m/s, ranges 8.8 / 21.6 / 13.6 / 25.6 m. One cut at r=8.8 m
    with -4.1 m/s: given another second it would have been within contact range. * G run (sep1):
    BOTH runs timed out and both were interrupted while CLOSING on m 14-17. In other words, the door
    also cuts off engagements that are beneficial.

    CORRECTION. Keep a STATIONAL CLOCK instead of a wall clock: if we are moving forward : hour <-
    max(0, hour + dt*(1 - progress_gain)) if we are at rest : hour <- hour + dt WATCH : hour >
    miss_time_timeout_s (base 8 s REMAIN THE SAME) gain On progressive engagement with 1.5, the clock
    rewinds 0.5 s/s, i.e. "each advance jump PARTIALLY refreshes the clock" -- but the refresh is
    PROPORTIONAL, independent of the frame rate (20 Hz). Stagnation detection is unchanged: if there
    is no progress, the clock advances exactly as 1 s/s and fires at 8 s.

    PROGRESS = best-so-far recovery rate: (best_previous - best_now) / window >
    progress_closure_threshold_mps The two witnesses in the design note ("closing rate > threshold" OR
    "new minimum in last N s") merged into a SINGLE measure, because every version that kept the two
    apart was fooled by the oscillation -- see for justification and two measurements.
    MpcConfig.progress_closure_threshold_mps. The best-so-far trace is MONOTONOUS, meaning the meter is
    INSTRUCTIVELY immune to oscillation and covers both witnesses because "best-so-far goes down if
    you're really closing".

    ABSOLUTE CEILING (NO infinite loop): MISC even if there is progress if elapsed >
    progress_ceiling_s (22 s). 22 s means ~770 m path in 35 m/s; Beyond that, it's already a matter of
    repositioning.

    UNTOUCHED: the "range opening" (45/30 and pass-certified 30/8) and "absolute range" gates remain
    -- they are healthy and measure REVERSED progress, not STOPPED anyway.
    """
    return _environment_count('YILDIZ_PROGRESS_CLOCK', default_value2)


def environment_blind_pn(default_value2: float = 1.0) -> float:
    """YILDIZ_BLIND_PN -- MAINTAIN THE *LAW* NOT THE COMMAND IN BLIND COAST.

    >0 = ON (DEFAULT since 1.0, 2026-08-10). 0 = OFF (old behavior BIT-SAME).

    COOK CAUSE (TO_TEST item 4 + 2026-08-09 miss-vector measurement). The half meter that escaped is
    on the HORIZONTAL axis: horizontal p50 1.04 m vs |vertical| at 62 closest pass of the campaign
    p50 0.73 m; <1.0 horizontal 0.51 vs vertical 0.35 even in m subset. In the terminal, we remain
    blind: in the last 5 m, ~%78 of the loops are without bbox and THE COMMAND IS RETURNED IN THAT
    BLIND. There are two separate blindnesses: (1) SKELETON (visual_base: bbox_age > 0.7 s) ->
    MPC is NEVER summoned; (2) CONTROLLER (stale_constraint_s = 0.30 s) -> solver runs but hard
    constraint FOV is left; In the STROKE + r <= impact_blind_range_m (8 m) arm, _blind_command
    REPEATS THE LAST COMMAND EXACTLY (the solver NEVER runs). The problem is in the last step of
    (2): as the range closes the geometry changes (c2 = KVA/r grows) but the lateral command remains
    CONSTANT. So "I can't see the target" is confused with "I don't know the geometry" -- whereas
    the RANGE channel is INDEPENDENT and fresh from bbox.

    THIS ARM: While bbox is stale, advance ex/ey DEAD COMPUTE FROM INSIDE and KEEP RUNNING the
    solver (instead of repeating the command). The progression is the SAME equations in
    _nominal_orbit: ex <- ex + dt*(-c2*w2 - yaw_rate_value + d_ex), c2 = KVA/r ey <- ey + dt*(-c3*w3 +
    d_ey), c3 = KVA/r r comes from _range() (model-advanced + range measurement; NOT dependent on
    bbox). Thus, when closing, the same lateral velocity produces greater angular velocity and the
    law continues to follow the geometry.

    *** WRONG-DIRECTION RISK AND RAILS (this entire arm is for safety) *** If the last estimate is
    wrong, dead reckoning ENLARGES it. Five rails: R1 TIME LIMIT blind_pn_maximum_s ( 1.0 s,
    YILDIZ_BLIND_PN_MAXIMUM_S ). When this period is reached, the advancement is CANCELED and the
    current returned-command behavior is reduced.  t_go is already in the terminal < 1.6 p. R2 d_ex
    ENDING ZERO soner ( 0.7 s, YILDIZ_BLIND_PN_TAU ) with disruptor exp (- t_blind / blind_pn_tau_s
    ) used to advance through the blind. Reason: turned d_ex is the CURRENT maneuver of the target;
    1 has no validity after s. Zero = "assume target is going straight" = at az information, at az
    loss. R3 | ex | CLAMP advanced | ex | cannot exceed | ex | of the last ACTUAL measurement by
    more than blind_pn_ex_margin_deg
    (5.0). Dead reckoning should not drag the target "imaginary" out of the frame and trigger an
    aggressive maneuver. R4 RANGE FRESHNESS DOOR measurement.range_m_value NO advance if None -- do not
    combine two independent blank sources (bbox + range). R5 YAW today's "keep last yaw" behavior
    (HOLD_YAW, proven) is preserved; This arm does NOT touch the law yaw.
    """
    return _environment_count('YILDIZ_BLIND_PN', default_value2)


def environment_speed_ceiling(default_value2: float = 35.0) -> float:
    """Speed ​​clamp FROM SINGLE SOURCE: guidance_config.VISUAL_MAX_SPEED_MPS.

    WHY SINGLE SOURCE: skeleton (visual_base.VisualLoop) ALREADY clamps the command with
    this number. If MPC keeps its ceiling LOWER, the solver will never want a speed it can reach and
    it will NEVER touch the clamp -- measured: In run target_infinity (mpc_infinity_20260805_022808)
    MpcConfig ceiling was 18, skeleton ceiling was 35; clamp contact rate 0%, target 21.05 m/s,
    closing -3 m/s, 7/7 HIS "range opening". So in pure tail chasing, the ONLY obstacle to capture
    was this single line.

    The ceiling is a CLAMP, NOT A DEMAND: the cost uses whatever it wants (see the same note on
    guidance_config). Escalating does not impose aggression, it removes constraint; Aggressiveness
    is determined by q_acceleration and STRIKE phase.
    """
    try:
        import guidance_config as _cfg
        return float(getattr(_cfg, 'VISUAL_MAX_SPEED_MPS', default_value2))
    except Exception:                       # offline/partial installation
        return float(default_value2)


# ============================================================ SETTINGS

@dataclass
class MpcConfig:
    """All tunables in one place; There is a JUSTIFICATION for each of them."""

    # ---- horizon / discretization -------------------------------------
    n_step: int = 20
    # 20 x 0.18 = 3.6 s (HORIZON SAME, number of steps reduced). Reason: the actuator is ACCELERATION
    # LIMITED (5 m/s^2), so it takes 3 s to change the lateral velocity 15 m/s. If the horizon is shorter
    # than that, MPC plans "I'll fix it later" and misses. handoff scenario 25-60 m range, t_go typically
    # sees most of 3-10 s -> 3.6 s horizon terminal phase. A longer one does not add any information (the
    # disturbance d is assumed to be constant along the horizon), but simply increases the solution time.
    step_s: float = 0.12
    # Fixed prediction step, separate from loop dt. The first step uses measured dt, the command holding
    # time, and the remaining steps use the nominal interval. A loop slowdown to 5 Hz therefore does not
    # extend the horizon. The step changed from 0.18 to 0.12 in the 2026-08-05 35 m/s trials for two
    # reasons. (1) The horizon should not exceed time to go. At 18 m/s, 20 x 0.18 = 3.6 s spans
    # approximately 65 m, comparable to the handoff envelope <=60 m. At 35 m/s it spans 126 m, beyond the
    # encounter, which softens the current command. A horizon 20 x 0.12 = 2.4 s spans 84 m. (2) LTV
    # coefficients c = KDEG/r are frozen within each step. At 35 m/s, a 0.18 s step changes range by 6.3
    # m, comparable to the 6 m floor. At 0.12 s the change is 4.2 m. The number of decision blocks and
    # solver dimensions are unchanged.
    blocks: tuple = (1, 1, 2, 2, 3, 4, 7)
    # Motion blocking (total = n_step). The first two steps are free (the command to be applied at that
    # moment must be precise), the next ones become coarse. 32 decision variable -> milliseconds in Pi 5.

    # ---- kisitlar --------------------------------------------------
    speed_ceiling_mps: float = field(default_factory=environment_speed_ceiling)
    # Use the same source as VISUAL_MAX_SPEED_MPS, see environment_speed_ceiling. A fixed 18.0 value
    # previously remained when the shared framework ceiling increased to 35. Measured consequences were
    # closure - 3 m/s during pure tail pursuit and 7 / 7 misses. The value is now derived to keep both
    # ceilings consistent. Vertical limits at 35 m/s : WPNAV_SPEED_UP /_DN did not increase with the
    # horizontal ceiling. At LOS elevation 13.5 deg , closure 35 m/s requires 35 * sin ( 13.5 ) = 8.2 m/s
    # climb, near the 9.0 climb ceiling. Above approximately 15 deg LOS elevation, the vertical ceiling
    # limits closure. Standoff down 4 - 6 m, with eps 9 - 13.5 deg , retains margin. Steeper geometry
    # requires increasing climb_ceiling together with WPNAV_SPEED_UP. HANDOFF ACCELERATION RAMP: tested,
    # rejected in simulation, and reverted. (2026 - 08 - 05, tur- 2 sim run.) Weigh the q_acceleration 3x
    # in the rev.
    # tau=2 A damping ramp was added with s; The intention was to smooth out the nose-down dive in the
    # handoff transition (first 3 s pitch min -20.5 deg). SIM MEASURED: mechanical worked (3.0 at
    # acceleration_multiply handoff, 1.0 at ~2) BUT (1) DID NOT FIX handoff transient (fresh-not cycle 261 -> 269,
    # pitch min still -34 deg) and (2) HALF THE SHUTDOWN (range_rate_value avg -3.11 -> -1.40 m/s, strong
    # shutdown <= -10 m/s rate %7.0 -> %2.9). Net malicious -> WITHDRAWED. The correct solution to the
    # handoff transient is not the acceleration penalty, but the terminal VERTICAL ALIGNMENT
    # (stroke_align_*) output (see _cost_lines vertical reference bias): straight camera = az body
    # movement = az dive.
    #
    # SPEED-INCREASE CLAMP -- same type TESTED, ELIMINATED BY MEASUREMENT. | u_k | <= min(ceiling, max(|
    # w_k |, base) + a_forward * tau ) rigid sphere-radius clamp; closed form was preserved (radius per
    # block, no additional cost). It did NOT fix pitch , it killed shutdown: a_forward ceiling pitch speed
    # pitch min min range off 3.81 - 24.9 1.93 2.0 m/s ^ 2 4.43 - 22.8 11.17 Reason: ArduPilot compensates
    # the speed loop setpoint difference with a time constant of ~ 0.25 s; Even a difference of 1.25 m/s
    # saturates the acceleration to 5 m/s ^ 2 . Clamping the SIZE of the command does not make this
    # difference smaller; What makes you smaller is the PUNISHMENT on (u - w). The mechanism remains in
    # the code (block-head sequence v_ceiling ) but DEFAULT is OFF.
    forward_acceleration_ceiling_mps2: float = 0.0
    speed_increase_floor_mps: float = 6.0
    climb_ceiling_mps: float = 9.0    # WPNAV_SPEED_UP = 10 m/s , % 10 share
    descent_ceiling_mps: float = 4.5     # WPNAV_SPEED_DN = 5 m/s , % 10 share
    yaw_speed_ceiling_dps: float = 90.0
    # 50 WAS deg/s and SATURATED in wanderer run: in mpc_diagnostic_20260803_182755 logs %9.4 of the loop
    # |yaw|=50 stuck to ceiling, |ex|>33 (horizontal out of FOV) %3.9. Saturation at ellipse %0.3, |ex|>33
    # %0.3. So half of the wanderer distortion is directly YAW SATURATION. ATC_SLEW_YAW 180 deg allows/s;
    # The quasi-90 deg/s and 33 deg are scanned at FOV 0.37 s, the body oscillation is still reasonable.
    ex_limit_deg: float = 26.0
    # Horizontal semi-FOV 33 deg; 7 deg for allowance hull roll + detected edge loss. This is now a HARD
    # limit (translated to box CBF to yaw), not a soft penalty.
    fov_alt_band_deg: float = 14.0      # allowed BELOW axis
    fov_floor_band_deg: float = 4.0     # lower limit of brake reduction
    fov_upper_band_deg: float = 17.5      # allowed ABOVE the axis
    fov_upper_floor_band_deg: float = 6.0
    # Narrowing base of the TOP BAND (top edge counterpart of fov_floor_band_deg). 2026-08-05, 35 m/s
    # round: the lower band was narrowing with BRAKE (brake -> nose up -> target to lower edge), while the
    # upper band was FIXED. In the 18 m/s this was not a shortcoming because the forward ACCELERATION was
    # small; The first seconds of engagement on the 35 m/s ceiling are CONTINUOUS forward acceleration,
    # with each 1 m/s^2 forward acceleration tilting the nose KVA/g = 5.84 deg DOWN, turning the fixed
    # camera down -- In mount 0 this pushes directly to the TOP edge as the target is already ABOVE the
    # axis (beta ~ -16). That's why symmetric throttling was added (see term 'acceleration' in
    # _dbf_bounds). Vertical semi-FOV ~20.07 deg (atan(tan(33)*720/1280)); Both bands should leave margins
    # inside this EDGE. HISTORY (+30 MONTAGE): the losses were ALL coming from the BOTTOM (climb ->
    # nose-up -> fixed camera looks up -> the target that was already at the bottom comes out completely),
    # so the band was asymmetrical and the BOTTOM was narrow
    # (11/16).
    # ASSEMBLY 0 (2026-08-04): sign REVERSED. At standoff beta = mount + pitch - eps = 0 - 2.5 - 13.5 =
    # -16.0, i.e. target axis 16 deg ABOVE and to the TOP edge there is only 4 deg; 36 deg to the lower
    # edge. BEFORE MEASUREMENT: bands 6 NO DOMINANT VARIABLE in scenario closed loop scan -- frame loss %
    # for upper band 15/16/17.5/19 1.74-2.03, lower band No difference for 11/14/17 (the bottom edge is
    # not binding at all in this geometry anyway). So the following choice is not an optimum, but a
    # GEOMETRIC SHARE logic: * upper band 16.0 -> 17.5: 16.0 falls EXACTLY above the handoff moment, so
    # the constraint was binding at t=0 (not bad, but unnecessary). 1.5 deg to handoff port 17.5,
    #     leaves the physical aside 2.6 deg. * bottom band 11.0 -> 14.0: bottom edge now ABUNDANT; The only
    #     effect of keeping it narrow would be unnecessary climbing demand on terminal braking. The tape
    #     only becomes binding in endgame (eps -> 0, brake lifts the nose). MAIN LEVER
    #     fov_climb_demand_ceiling appeared (see below): 3.0 -> 5.0 reduced the framing loss from %2.37
    #     to %1.75.
    mount_pitch_deg: float = field(default_factory=environment_mount_deg)
    # $YILDIZ_MOUNT (exports standoff_geom.sh), fallback 0.0. NO LONGER WRITTEN FIXED: when the sim mount
    # was switched from 30 -> 0 this single line explained the entire frame loss. Crushed with --mount
    # (ablation/real hardware gimbal angle command).
    aim_deg: float = 0.0                # scenario.sh fixes AIM=0

    pitch_coupling: bool = False
    # BODY PITCH COUPLING (gimbal switch). ON : camera axis FIXED to body -> axis = mount + body pitch.
    # Climbing (3.2 deg/(m/s)) and brake (5.84 deg/(m/s^2)) shift the axis; THE MODEL predicts this (see
    # brake throttling in pitch_climb_coefficient and _cbf_limits). OFF: camera axis stabilized
    # BODY-INDEPENDENT (pitch-servo gimbal). ey_ref = -(mount + aim), climb/brake terms FALL; The input
    # sensitivity of the vertical constraint comes from the remaining PURE GEOMETRIC channel (if you
    # lower, the target appears higher) -- so the constraint does NOT become blunt when the switch is
    # closed. DEFAULT OFF (gimbal branch, 2026-08-05): sim now has ACTUAL stabilized gimbal -- measured in
    # flight: fuselage +-35 deg while skidding camera world pitch max 0.65 deg. Body pitch is NOT
    # reflected in the image; Keeping the coupling open introduces the model into a shift that DOES NOT
    # EXIST (double compensation). Turned on with --pitch-coupling for legacy body-fixed behavior
    # (mpc_test 5g measures both directions).

    # --- VERTICAL: SUB-TARGET DEPTH CEILING (2026-08-04 eng-2) ---
    standoff_depth_m: float = 6.0
    depth_ceiling_m: float = 15.0
    depth_approach_s: float = 2.0
    # ROBUST PROBLEM (test pilot type-2): vertical duct was aging to the bottom. Reason: the reference of
    # the framing cost is CENTER OF CAMERA AXIS (ey_ref = -(mount+pitch)). Since +30 sits BELOW the target
    # axis in the mount (summary.txt: -9.8 deg), the only way to center the target is to DESCEND and
    # increase its apparent ascent. This is geometrically impossible at long range (at r=190 m, it is
    # necessary to descend 95 m to get the target on the axis) -> the controller dives to the bottom.
    # SOLUTION (ONE-SIDED): I did NOT change the framing cost (draw to center behaves correctly in
    # captured/closed runs; a reference shift that disrupted it worsened the MAXIMUM descent in the
    # ellipse). Instead, I set a ceiling on LOW: go no more than depth_ceiling_m below the target.
    # Sub-target depth = r*sin(eps); this ceiling is inscribed on the upper limit of the vertical velocity
    # SLONE (same machine as the altitude floor, the projection remains in closed form). IT IS ONE SIDED:
    # it only stops the descent, it NEVER forces the climb -> it cannot break the closed runs. As the
    # range closes, r*sin(eps) shrinks, the roof opens, terminal collision is released. ASSEMBLY 0 UPDATE:
    # design standoff depth down = 6 m (back 25 / down 6), i.e. 13 -> 6. Ceiling with the same "design + 7
    # m share" logic 20 -> 15 (not 13): handoff door open at the same angle LOS to a range of 60 m and
    # depth at 55 m 55*sin(13.5) = 12.8 m; A ceiling of 13 would lock the descent IMMEDIATELY after
    # revving  15 passes m handoff geometry, cuts 2x over-dipping (>= 12 m design over). IMPORTANT ( mount
    # 0 ): this ceiling is RARELY binding anymore. In mount 0 , the framing cost requires
    # CLIMBING because the target is ABOVE the axis,
    # i.e. the "branch to center" mechanism in type-2 is radically absent. The ceiling was deliberately
    # left as a SAFETY INCREASE (the mechanism may come back if the gimbal aim is changed or the aim_deg
    # is manually given).
    vertical_depth_ceiling: bool = True

    pitch_climb_coefficient: float = 3.2
    pitch_coefficient_ceiling_dps: float = 25.0
    # CLIMBING -> PITCH MATCHING. Measurement of LOS agent: pitch_deg ~ - 1.8 + 3.2 * climb_rate [ m/s ]
    # This was the root mechanism of frame loss in + 30 mounting: 5 m/s climbing nose + 16 raises deg ,
    # raises the camera axis ( mount + pitch ) to 46 deg and raises the target vertical FOV (half) which
    # is already on the rise 22 deg 20 deg ) removed from the BOTTOM edge. At MOUNT 0 the sign is
    # reversed: since the target is ABOVE the axis ( beta < 0 ) climbing brings the target CLOSER to the
    # center, the danger is in the terminal phase ( beta ~ 0 while the brake lifts the nose) it slides to
    # the LOWER edge. The term has the same sign in both cases, except which edge is binding varies. THE
    # MODEL HAS TO PREDICT THIS: Constraint FOV beta_k = ey_k - kats * vz_k + C It is set up as (vz NED ,
    # negative=escalation), i.e. "if you escalate, you get your bottom share" IN the constraint. C is
    # anchored to the measured pitch (not to the absolute fit) so the fit error remains OD.
    # pitch_coefficient_ceiling : upper limit beyond which the mapping is considered linear (beyond which
    # pitch saturates). GIMBAL NOTE: this coefficient is RESET (separates the gimbal axis from the body)
    # when pitch_coupling = False . In that case, a pure geometric channel ( coefficient_geom = T*c3/ cos
    # ( eps ), see _cbf_limits) is added to the input sensitivity in both modes so that the vertical
    # constraint is not blunted -- in mount 0 this term is already ~% 40 of kats size, so when the switch
    # is closed, the constraint weakens but does NOT disappear.

    # ---- cost weights (normalized; unitless) ----------------- Each term is first divided by a SCALE,
    # then multiplied by the weight.
    scale_ex_deg: float = 10.0
    scale_ey_deg: float = 10.0
    scale_range_m: float = 50.0
    scale_los_speed_dps: float = 10.0
    scale_speed_mps: float = 18.0
    # ATTENTION: this is NOT a SPEED CEILING, it is a fixed NORMALIZATION scale, and the ceiling was
    # CONSCIOUSLY left at 18 in the round 18 -> 35. The three terms divided by it (r_speed level penalty,
    # r_delta_speed difference penalty, lambda_prox) express the ABSOLUTE actuator smoothness: the actuator
    # acceleration limit (WPNAV_ACCEL 5 m/s^2) is INDEPENDENT of the speed cap, i.e. "how many per block"
    # The answer to the question "m/s I can change" does not grow with the ceiling. Moving the scale to 35
    # multiplies these three weights by (18/35)^2 = 0.26; The effect of lambda_prox, which reduces the
    # condition number 1.3e3 -> 2.7e2, also weakens at the same rate (solver time is welcome). The scale
    # is the unit carrier, not the clamp.
    scale_yaw_dps: float = 50.0

    q_los_speed: float = 4.0
    # INERTIAL LOS speed -> 0. MAIN term (PN) establishing the collision course. The biggest weight should
    # be here; framing terms shouldn't overwhelm it.
    q_ex: float = 0.5      # framing (horizontal) -- mainly paid with yaw
    q_ey: float = 0.60     # framing (vertical) -- paid by u3, speed LOS
                           # yarisir; 18 m/s'de LOS'un ~1/10'u yetiyordu.
    # 0.35 -> 0.60 (2026-08-05, 35 m/s tour). RATIONALE WAS MEASURED: on high ceilings, the PRINCIPAL
    # channel of framing losses was VERTICAL. At the moment of loss in pure tail pursuit |ex| max 1.1 deg
    # (NO problem horizontally) -- so the target comes off the top/bottom edge. The mechanism is
    # geometric: standoff BELOW the target (down 4-6 m) and eps = hang(down/r) grows as the range closes;
    # Since the closing of 35 m/s is times faster than 2-5, the correction time of the vertical channel
    # has been shortened at the same rate. Closed loop panel (12 scenario): q_ey 0.35 -> 0.60 framing loss
    # %26.7 -> %23.4, min range MEDIAN 9.63 -> 8.51 m. So this setting improved FRAMING and CAPTURE AT THE
    # SAME TIME -- the usual trade-off isn't there because the lost frame was already cutting off the
    # capture. 0.9 was also measured (loss %21.3, median 8.23) but average min range gets worse (11.90 ->
    # 12.41): vertical channel starts to crush LOS speed term. 0.60 is the largest value that improves
    # both measures.

    # ---- REWARD: LINEAR BBOX FIELD (NOT SQUARE ROOT) -------------------
    q_area: float = 3.0
    q_area_rate: float = 1.5
    # A/B BUTTON (TO_TEST item 1): YILDIZ_Q_AREA_MULTIPLIER scales both. Default 1.0 -> behavior WILL NOT
    # change. Rationale/measurement: environment_q_area_multiplier associatestring.
    q_area_multiplier: float = field(default_factory=environment_q_area_multiplier)
    # A/B BUTTON (TO_TEST item 3): scale cost horizon with range. 0.0 = OFF (default, legacy behavior).
    # See environment_horizon_range_ref.
    horizon_range_ref_m: float = field(default_factory=environment_horizon_range_ref)
    horizon_step_floor_s: float = 0.05     # step lower limit after scaling
    q_range: float = 0.25
    # The reward is now directly bbox FIELD (w*h, px ^ 2 ) and its FIRST DERIVATIVE (growth rate = proxy
    # for approach speed). Since the area is ~ K/r^ 2 : a_k = A( rbar_k )/ A_0 = ( r_0 / rbar_k )^ 2
    # (relative area) also_k /dt = 2 a_k w1_k / rbar_k (growth rate) Both are linearized around rbar -> ONLY
    # LINEAR term enters the cost, Hessian does not change, solution time does not increase. WHY NOT
    # SQUARE ROOT/RANGE-SQUARED: q_range *(r/ 50 )^ 2 term's closing incentive is PROPORTIONAL to r, i.e.
    # strong FAR, weak near -- the exact opposite of collision. The excitation of the linear field grows
    # with 1 /r^ 3 : 40 is 1 units in m, 20 is 8 units in m. Terminal aggressiveness increases
    # consciously; what stabilizes it is now the HARD constraint FOV (see cbf_gamma ).  q_range is a small
    # relic: its only job is to give the Hessian curvature in the range channel (numerical conditioning),
    # the homing incentive comes from the residual field.
    p_multiplier: float = 3.0  # all state weights in terminal step x3

    r_speed: float = 0.02     # input level penalty (lateral/vertical and
                            # to yaw; NO penalty for closing FORWARD, our aim is to close quickly)
    r_yaw: float = 0.06

    q_acceleration: float = 0.80
    scale_acceleration_mps: float = 6.0
    # ACCELERATION (bank angle) PENALTY -- vital in this environment. The difference between the command
    # and the current speed (u - w) is directly the desired acceleration; the autopilot responds to this
    # with WPNAV_ACCEL=5 m/s^2 and the copter CLINKS up to atan(a/d). Hard braking lifts the nose 25 deg
    # UP; On the +30 mount, the camera axis would move to 55 degrees, leaving the target OUTSIDE the
    # vertical FOV (half 20 deg). At MOUNT 0, the same 25 degrees this time pushes the target to the LOWER
    # edge (axis rises to 25 degrees, target remains ~0 elevated) -- so the penalty is still required, the
    # lost edge has changed. Measured in the simulation: without this penalty, the target would disappear
    # in the first second. The second benefit is numerical: the u1 (closing) channel had no curvature in
    # the Hessian, since this term gives it the number of conditions drops by a factor of ~5. SETTING
    # NOTE: this is the MAIN switch between "how aggressive maneuver" and "keep framing". 0.45 -> more
    # aggressive, closer in tight corner 1-2 m but temporary framing losses increase; 1.5 -> framing is
    # very solid but convergence slows down in tail tracking. It should be readjusted by looking at the
    # binary in the sim logs (pitch, detection continuity).
    r_delta_speed: float = 0.55
    r_delta_yaw: float = 10.0
    r_delta_yaw_free: float = 1.0
    yaw_free_vperp_alt: float = 7.0
    yaw_free_vperp_upper: float = 18.0
    yaw_weight_tau_s: float = 0.6
    # Command change penalty (r1 in presentation). It is kept high: the LPF of the skeleton is already
    # smoothing, but the solution of the MPC should not jump from step to step, otherwise the LPF will
    # constantly chase after it (phase loss). r_delta_yaw 0.30 -> 10.0 (2026-08-04 tour-3 chatter
    # analysis): effective weight 0.30/50^2 = It was 1.2e-4 (deg/s)^-2; Since the tracking term of ex was
    # 5e-3 deg^-2 and the sensitivity of yaw was ~0.5, motion suppression was PRACTICALLY NON (3-4 is
    # orders of magnitude smaller). 10 selected by scan (0.3/3/10/30): |dYaw| step rms 2.6-3.7 -> 1.4 dps
    # (about 2.5x), capture unchanged. Terminal agility is broken in 30.
    #
    # CIRCUIT-4 REGRESSION AND SOLUTION (2026-08-04): FIXED 10.0 reduced chatter well below target (0.44-0.91
    # dps, target 2.5) BUT HE KILLED HILL YAW AUTHORITY: |yaw| max 90 -> 53-61 dps, >80 dps rate %1 -> %0.
    # The short bursts of high yaw required for Wanderer's sharp maneuvers could not be produced -> |ex|
    # p90 14.3 -> 29, area -%65, detect %94 -> %79. The ellipse (predictable path) was NOT affected by the
    # same absorption, in fact it improved -- so the problem was that a constant coefficient gave the same
    # response to two different regimes.
    #
    # SOLUTION: GAIN PROGRAMMING -- but ON THE CORRECT VARIABLE.
    #
    # FIRST ATTEMPT (failed, eliminated by measurement): criterion was max(|ex|, |d_ex|). It had two
    # separate defects, both measured in closed loop (8 scenario panel: sharp/bend/wanderer/ellipse/flat):
    # (1) |ex| It is the CONTROLLED variable. Opening authority over it establishes POSITIVE FEEDBACK: the
    # error grows -> the authority opens -> yaw exceeds -> the error grows with the opposite sign -> the
    # authority opens again. It was measured to produce limit cycle in tail tracking: RAW |dYaw| step rms
    # 2.48 (fixed 10) -> 8.44 dps, |ex| p90 13.0 -> 31.6,
    #      or even FRAME LOSS in a seed. (2) |d_ex| is the angle velocity, i.e. d = KDEG*v_perpendicular/r. The same
    #      target movement produces large d at NEAR range; The criterion hardens automatically with range
    #      and opens the authority exactly where the chatter was born (r<20 m, tour-3 measurement: 0.67 ->
    #      7.37 dps).
    #
    # CURRENT SOLUTION: criterion TARGET'S SPEED PERPENDIUM TO LOS
    #       v_perpendicular = |d_ex| * r / KDEG   [m/s]
    # i.e. the jammer's RANGE FREE state. Features: * DISSOLVE: The d_ex estimate is established by
    # subtracting our own yaw speed (DisturbanceEstimator), which is the answer to the question "is the target
    # manoeuvring"; is not a function of our command -> no feedback in (1). * RANGE INDEPENDENT: Same
    # target movement in 40 m and 15 m gives the same number -> No close-range hardening in (2). *
    # PHYSICAL: 20 becomes v_perpendicular 15-20 m/s on the sharp bend of the target of m/s, ~0-5 m/s on the
    # straight leg.
    #       v_perpendicular <= 7 m/s  -> r_delta_yaw          (10, steady)
    #       v_perpendicular >= 18 m/s -> r_delta_yaw_free  (1, cevik)
    # Apply a weight LPF with yaw_weight_tau_s = 0.6 s. A gain that jumps between iterations also makes
    # the solution jump. A sweep of tau 0.4/0.6/0.9 and thresholds 3-12/5-15/7-18 selected 7-18 with 0.6
    # s.
    #
    # MEASURED RESULT (based on scenario 8, two seeds; fixed 10.0):
    #   |ex| p90 ortalamasi 18.65 -> 17.60 deg
    #   RAW chatter: on sharp routes 2.48 -> 2.48, on real routes (ellipse/wanderer/straight) 1.21 -> 1.61
    #   dps (target < 2.5) chatter REACHING vehicle 1.79 -> 2.17 dps (target < 3.0) total frame loss 156
    #   -> 153 loop min range in tight bend 4.3/4.2 -> 3.8/4.3 m So maneuver authority is back, chatter
    #   margin is preserved.

    # ---- FOV: HARD CONSTRAINT (iki katmanli) ----------------------------
    cbf_prediction_s: float = 1.0
    cbf_range_ref_m: float = 45.0
    cbf_prediction_min_s: float = 0.35
    # cbf_range_ref_m : prediction horizon scales PROPORTIONALLY TO RANGE (T = cbf_prediction_s * r /
    # ref). Reason: box center to input The sensitivity goes with c2*T and c2 = KVA/r; At constant T
    # sensitivity explodes at close range. Measured (bend/lateral, |dYaw| step rms ): 35 - 60 in m 0.67
    # dps while 12 - 20 in m 7.37 . The choice T ~ r fixes c2*T.
    cbf_gamma: float = 0.50
    # LAYER 1 (HARD): the framing constraint is strictly satisfied on the APPLIED input. Key observation:
    # the framing variable one step ahead is AFFINE on the input -- beta_ {k+1} = B0 - kats*al*(
    # a_perpendicular . u) (dusey)
    #   ex_{k+1} = E0 - h*omega (horizontal), that is, the vertical constraint is the narrowing of the
    #   existing VERTICAL SPEED SECTION, the horizontal constraint is the narrowing of the YAW BOX. Both
    #   are set types that the projection already FULLY solves -> hard constraint comes FREE, no
    #   additional variables or additional iterations. The constraint is written in the form of a
    #   discrete-time control barrier (CBF): h = boundary - beta >= 0, h_{k+1} >= (1-gamma) h_k The
    #   reason: at the moment of handoff the target ALREADY comes close to the lower edge (drifting ~42 m
    #   behind the positioned target). If we write plain "beta<=limit" the set will be NULL and the solver
    #   will crash. When there is a violation in the form CBF, the constraint says "cure", not "provide
    #   immediately" -- so it is NEVER infeasible in flight. gamma=0.30: at each step 30% of the violation
    #   is closed (time constant ~0.17 s in 20 Hz).
    fov_hard: bool = True               # can be closed for ablation

    # --- HARD KISITIN EMNIYET SUBAPLARI (2026-08-04 cakilma dersi) ---
    fov_descent_demand_ceiling_mps: float = 0.4
    fov_climb_demand_ceiling_mps: float = 9.0
    # CLIMB DEMAND CEILING 5.0 -> 9.0 (2026-08-05, 35 m/s tour). THIS CEILING SCALES BY CLOSING SPEED,
    # because its work is geometric: standoff IS BELOW the target (down 4-6 m) and as the range closes the
    # displayed rise eps = hang(down/r) GROWS -- 30 11.5 deg at m, 12 30 at m deg, i.e. OUTSIDE of
    # vertical semi-FOV (20.07). The only way to keep the frame is to cover the depth PROPORTIONAL to the
    # range: climb ~ closure_rate * (down / r) Cover 15 m/s and down/r ~ 0.33 while this 5 m/s ASHAR.
    # Measured (closed loop panel, 12 scenario, ceiling 35 m/s): PURE TAIL tracking at ceiling 5.0 min
    # range 11.68 at m LOSING FRAME (|ex| max 1.1 deg -- so the loss is completely VERTICAL); In 9.0, the
    # same scenario goes down to 2 m and ends with a COLLISION. Panel overall: loss %26.9 -> %26.7, avg
    # min range 13.2 -> 12.4 m. 9.0 = climb_ceiling_mps, so the constraint can now request ALL physical
    # climbing authority of the vehicle. Climbing is not dangerous like Descent (FAR AWAY from the ground)
    # -- that's why the asymmetry is intentional: descent request 0.4, climb request 9.0. HISTORY (3.0 ->
    # 5.0, 2026-08-04 mount 0 measurement): On Mount 0, the framing saving direction is CLIMBING (ABOVE
    # the target axis): as the range closes at constant depth eps grows, so to maintain the frame it is
    # necessary to close the depth PROPORTIONAL to the range -- while the closing 10 m/s and d/r ~ 0.23
    # this alone means 2.3 m/s climbing, on top of the correction margin. The 3.0 ceiling was binding in
    # 18% of the cases in the algebra scan (the constraint could not say "climb more"); %6.5 at 5.0. In
    # closed loop
    # measured (scenario 6): framing loss %2.37 -> %1.75, min range and ground safety unchanged. Climbing
    # is as dangerous as Descent.
    # not (it moves AWAY from the ground), so the asymmetry is intentional: descent request 0.4, climb
    # request 5.0. THE FRAME PERSON'S JOB IS TO PROHIBIT CLIMBING, NOT TO ORDER Descent. Since the loss
    # mechanism is "climb -> nose up -> fixed camera looks up -> target emerges from the bottom", the
    # correct intervention is to stop climbing. Descending also geometrically reduces beta (the target
    # rises relative to us), but this is a DANGEROUS lever: measured in the simulation, when the
    # constraint was released, the vehicle descended 22 m to save the frame and flew to the ground (in
    # fov_hard=False it does not descend at all). So the descent that the constraint can demand is limited
    # to 0.4 m/s: in practice it means "climb <= 0". If it is not enough, it is already unrecoverable and
    # the timeout constraint is completely abandoned.
    altitude_floor_m: float = 25.0
    altitude_approach_s: float = 3.0
    # MY OWN ALTITUDE BASE. The visual_base has a method-independent base ( 15 m), but the controller
    # should NOT lean on it: it is the last defense, it cannot be guided. The base here is written on the
    # upper limit of the vertical speed SECTION: vz <= (altitude - base) / approach_s , that is, when
    # approaching the base, the permissible descent decreases linearly to zero. Since the normal one is
    # parallel to a_perpendicular , it falls into the same cluster as the CURRENT slice -> the projection
    # remains in closed form, free. WHY IT EXISTS: In the 2026 - 08 - 04 crash, the framing restriction
    # brought the vehicle down to the ground. The root cause has been verified, but the vertical channel
    # must have a base that works "no matter what" -- without that base in the simulation, it feeds on
    # itself once the descent begins.
    constraint_descent_threshold_mps: float = 0.25
    empty_set_ceiling_loop: int = 40
    empty_backward_threshold_loop: int = 15
    # If the constraint CANNOT be met for ~2 s (cycle 40 in 20 Hz) or imposes continuous descent (see
    # constraint_descent_threshold), the framing constraint is completely RELEASED. Reason: at that point the
    # target cannot be saved anyway; It is better to accept the loss of frame than to fly to the ground
    # chasing the frame. After the decision maker 1.5 s dwell zone returns to 'position' and repositions.
    # empty_backward_threshold: the released constraint is RE-enabled (hysteresis) when the meter fades below this
    # threshold. 40/15 band means ~1.25 s release; not unlimited latch (old bug).
    stale_constraint_s: float = 0.30
    # If bbox is older than this age, the framing constraint will NOT be reduced. LEAK CYCLE: ey FREEZES
    # WHILE bbox is stale; The only term that changes beta remains -kats*vz, meaning that the constraint
    # measures the vertical velocity it produces and becomes stiffer. Measured (ellipse block 2 ): ey -
    # 14.57 rotated in beta 1.4 s in 4.96 -> 37.42 . Since the skeleton HOLDS the last command 1 s in the
    # short space, the excess command generated in this window is executed for one second -- hence the
    # requirement for the gate.

    rho_fov: float = 8.0
    rho_fov_vertical: float = 0.0
    fov_l1_delta_deg: float = 1.5
    # LAYER 2 (PLAN): l1 exact penalty along the horizon. The quadratic penalty never completely resets
    # the constraint violation (its gradient fades with the violation); Since the gradient of l1 is
    # constant rho, if rho is large enough, the solution is mathematically equal to the HARD constrained
    # solution, if not, the solver does not collapse again, but chooses the most az violation. Since l1 is
    # non-differentiable, it is smoothed with Huber (|violation| < quadratic in the delta region): the
    # first-order method remains valid, delta= 0.75 deg resolution is already well above the noise of bbox
    # ( 0.08 deg ). rho SELECTION: the complete penalty theorem requires rho > ||lambda*||_inf. The
    # gradient of the prime cost over a degree is of the order ~ 0.4 (q/scale^ 2 * value), so rho= 8 is
    # approximately 20 times the fraction -- when the constraint is satisfiable the solution is the same
    # as the HARD constrained solution. Larger rho unnecessarily penalizes the Huber curvature (rho/delta)
    # and hence the step size FISTA (tried rho= 30 /delta= 0.75: L increased to 3e4, solver paralyzed).
    # rho_fov_vertical = 0 (DEFAULT, selected by meter): the vertical framing constraint ALREADY is fully
    # satisfied by the rigid CBF slice; Putting a l1 penalty on top of it doesn't help anything but is
    # very expensive. Because the beta line is VERY sensitive to input (the -kats* a_perpendicular .w term
    # is connected directly to the input with the al gain), Ga^T Ga turns out to be large and L jumps to
    # 14 -> 1.4e3 ;  FISTA 30 can't go anywhere in the iteration (measured: u1 = 9.6 remains while the
    # hard-constraint-alone solution u1 = 16.1 ). In the horizontal channel, the ex line is connected to
    # the input only through integration, Ga is small, the penalty is free; also yaw
    # Since his authority can be SATURATED (9.4% saturation was measured on wanderer) the horizon penalty
    # really works there.
    fov_awake_margin_deg: float = 4.0
    # The share considered "close to kink" in the L account. In Huber's LINEAR region, the curvature is
    # ZERO; Adding the lines there to L will make the step smaller in vain. So the margin is kept narrow
    # and the risk of divergence is left to adaptive restart. The share counted as "can be active after
    # az" in the step length (L) calculation. The Penal Hessian alone inflates lambda_max by a factor of
    # ~145; adding it to L ALWAYS makes my step smaller unnecessarily while the penalty is passive (most
    # of the time). So L is only set up with ACTIVE/close lines.

    lambda_prox: float = 1.0
    # Proximal regularization: penalty ||u - u_warm||^2. It does TWO things: (1) Increases the small
    # eigenvalue of the Hessian, reducing the number of conditions from ~1.3e3 to ~2.7e2 -> FISTA 2.3
    # achieves the same accuracy as the az iteration (measured). (2) Limits how much the plan can change
    # per cycle; "Real-time iteration" is the MPC's standard protection against linearization error and
    # measurement noise.

    # ---- jammer (target movement) estimation -------------
    disturbance_confidence_s: float = 0.8
    # BREAKING TRUST. at handoff time d = 0; but the target is flying 20 m/s and d is physically "vertical
    # speed of the target / range" (typical 15-25 deg/s). Assuming d=0, MPC thinks the target is
    # STATIONARY and performs the wrong (hard braking) maneuver -- measured in the simulation: frame loss
    # in the first second. Solution: (a) in the first examples the gain of LPF is taken as 1/n (running
    # average) so that d sits over several cycles; (b) LOS-speed (PN) weight multiplied by the confidence
    # during this period. When confidence is low, the dominant term is range -- so what position homing
    # does is the safe default.
    disturbance_tau_s: float = 0.45
    disturbance_box_tau_s: float = 1.2
    disturbance_absolute_ceiling_dps: float = 60.0
    disturbance_rotate_range_m: float = 15.0
    # disturbance_box_tau_s: Time constant of the SEPARATE, slow copy feeding the box center of the HARD PART.
    # The cost reference remains at 0.45 s (drive response), the constraint boundary is relaxed by 1.2 s.
    # In Type-3 the same noise entered both and was the direct source of the yaw chatter (4 Hz, +-16 dps).
    # disturbance_absolute_ceiling_dps: EK absolute ceiling to physics clamp. disturbance_rotate_range_m: below this
    # range the jammer is FREEZED. d = measured angle velocity - model angle velocity. The raw version is
    # very noisy (bbox center +-1-2 px shaking); 0.45 s LPF passes the band of target maneuver (under ~1
    # Hz), cuts pixel noise.
    target_speed_ceiling_mps: float = 40.0
    # d clamp via physics: |d| <= KDEG * v_target_max / range. 40 m/s twice the reasonable target (20 m/s)
    # -> safe upper limit.

    # ---- APN: target lateral ACCELERATION (with env button) --------- Rationale, root cause and
    # measurement: see environment_apn().
    apn: bool = field(default_factory=lambda: environment_apn() > 0.0)
    apn_multiplier: float = field(
        default_factory=lambda: max(environment_apn(), 0.0) or 1.0)
    apn_tau_s: float = 0.6
    # a_perpendicular = d(v_perpendicular)/dt is the DERIVATIVE: taking a second derivative of d_ex over its own LPF (0.45 s)
    # magnifies the noise by a factor of ~1/dt. 0.6 s SEPARATE and a slower LPF is required. 1/n quick
    # start NOT CONSCIOUSLY (this is the difference from d_ex): the first derivative example alone would
    # drive the prediction to the clamp (+-6). Ramp from zero + confidence multiplier = safe side; ~2 sits
    # on p., engagements 20-60 p.
    apn_a_ceiling_mps2: float = 6.0
    # The target is fixed wing; In 20 m/s 6 m/s^2 means lateral acceleration R ~ 67 m turning radius (~0.6
    # g). The actual demand of the ellipse target is below this; The ceiling is there to stop noise
    # explosions, not to restrict the law.
    apn_dead_band_mps2: float = 0.5
    # SUBTRACTED (soft) deadband: a_active = sign(a)*max(|a|-db, 0). HARD deadband produces bounce at the
    # threshold, and that bounce propagates directly to the horizon. The subtractive form is CONTINUOUS at
    # the threshold. Purpose: In the STRAIGHT leg (noise floor ~0.3-0.5 m/s^2 measured) keep the
    # contribution EXACTLY ZERO.
    range_disturbance_source: str = "disabled"
    # "closed" | "range". Subtracting the closing velocity from the range derivative means deriving the
    # RADIAL velocity of the target; The visual_base contract allows range ONLY from target telemetry.
    # Default is OFF. If it opens, d_r is predicted (the range term would be a little more accurate, the
    # behavior does not change because we are already closing at full throttle).

    # ---- range filter --------------------------------------------
    area_rate_tau_s: float = 0.35
    # bbox area growth rate LPF. Since the field is w*h, its noise is ~2 times that of the square root;
    # 0.35 s passes the approach signal.
    range_measurement_gain: float = 0.35
    # Estimator range 10 Hz and noisy; The internal state is advanced by the model (r -= w1*dt) and pulled
    # towards the measurement with this gain. So if I have the bbox but the range is stale, my guidance
    # won't remain blind.
    range_if_absent_m: float = 55.0
    # If no range arrives: the handoff gate opens the range at <=60 m, 55 m is a reasonable start (if
    # wrong, the filter corrects it at 1-2 s).
    range_floor_m: float = 6.0
    # c = KVA/r coefficient explodes at r->0; 6 We have already collided at m.

    # ---- actuator model -------------------------------------------
    speed_latency_tau_s: float = 1.00
    # Command -> actual speed delay. CHAIN: frame LPF (tau=0.35) -> |v|<=18 clamp -> speed loop of the
    # autopilot, which WPNAV_ACCEL = 5 m/s^2 with ACCELERATION LIMITED (params/swarm_copter.parm). So the
    # response is 1. it is RAMP, not order: 10 A speed change of m/s takes 2 s. 1. order equivalent tau ~
    # 0.63*dV/A; dV~8 Since m/s is typical tau ~ 1.0 p. This number also determines the horizon: MPC must
    # know that he can change the lateral speed ONLY in 3 s, otherwise he plans and misses it, thinking "I
    # will fix it at the last moment". Keeping Model tau SMALL (0.6) produced simulated hanging and
    # framing loss.
    #
    # *** 2026-08-08 MEASUREMENT: THIS NUMBER IS ~6 TIMES SMALLER IN THE OPERATING BAND. *** The above
    # assumption "dV~8 m/s" does not hold; measured |e| the top of the distribution is 10-20 m/s and there
    # tau_active 5.5-6.8 p. Moreover, the chain is 1. NOT order, BUT ACCELERATION LIMITED (plateau ~4
    # m/s^2) -- so it cannot be represented by a single tau. This field has NOT been changed (closed arm
    # bit-keep it the same); fix behind separate env button: see environment_actuator() / actuator* fields are
    # below.

    # ---- SATURATED actuator (with env button, default ON) ------------ Rationale, root cause and
    # measurement: see fig. environment_actuator().
    actuator: bool = field(default_factory=lambda: environment_actuator() > 0.0)
    actuator_tau_lin_s: float = field(
        default_factory=lambda: _environment_count('YILDIZ_TAU_LIN', 1.7))
    # Linear region time constant. Measured: 2.33 / 1.56 / 1.74 s (|e| < 4 m/s band, three runs) -> 1.7
    # close to average.
    actuator_a_max_mps2: float = field(
        default_factory=lambda: _environment_count('YILDIZ_A_MAX', 4.0))
    actuator_tau_lin_z_s: float = field(
        default_factory=lambda: _environment_count('YILDIZ_TAU_LIN_Z', 1.0))
    # VERTICAL linear region time constant. IT WAS NOT IN THE FIRST VERSION: the vertical was left in the
    # old 1.00 s of the model (also not measured, conservative, I said). I MEASURE (SAME method as
    # horizontal, SEP OFF 6 running, e = cmd_vz - vel_z , a_par = (dvz/dt)*sign(e), |e|< 3 line passing
    # through the origin in the region m/s ): slope 0.294 - 0.575 1 /s -> tau_lin_z = 1.74 / 1.77 / 1.86 /
    # 2.41 / 2.54 / 3.40 s, AVERAGE 2.16 s SO VERTICAL, SLOWER THAN HORIZONTAL ( 2.16 vs 1.70 ) -- the
    # model thought it was FASTER THAN HORIZONTAL with 1.00 s. Once the horizontal was realistically
    # slowed down , the solver shifted the demand to this ARTIFICIAL CHEAP channel:
    #     |u3| p90 6.77 -> 14.74 m/s, u3>u1 %7.5 -> %19.8,
    #     | cmd_vz | p90 6.38 -> 9.00 (on ceiling), abort/engagement 0.36 -> 0.69
    #
    # *** RETURNED TO DEFAULT 1.0 ( 2026 - 08 - 08 night- 2 , sep2 run). *** APPLYING THE measured 2.2 DID
    # NOT WORK, instead it made the result worse: ABORT IN SUBTITLE / engagement 0.54 - 0.92 (vertical old)
    # -> 1.11 (vertical new) | cmd_vz | SQUARE RATIO ON THE CEILING % 5.4 -> % 20.3 ( 4 floor) | cmd_vz |
    # p90 8.53 -> 8.99 (ceiling 9 ) So "making the vertical channel more expensive" did NOT reduce
    # vertical demand.  CBF is not the culprit either: the force-decrease imposed frame rate DROPPED to %
    # 7.6 -> % 2.2 (box opened) but the command still stuck to the ceiling. So it's NOT the actuator model
    # that sets up vertical demand, it's the cost/geometry side (chasing vertical standoff + eps ) --
    # that's a separate TYPE of work. MEASURED VALUES NOT DELETED: tau_lin_z = 2.16 s ( 1.74 / 1.77 / 1.86
    # / 2.41 / 2.54 / 3.40 , SEP-OFF 6 running)
    # and plateau 5.25 m/s^2 (= WPNAV_ACCEL_Z 500) are TRUE numbers; when vertical cost/geometry is
    # corrected it is OPEN with YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5.
    # DEFAULT 1.0 + a_max_z = infinity == NO vertical saturation, so YILDIZ_ACTUATOR=1 is exactly the
    # PROOF HORIZONTAL-ALONE arm.
    actuator_a_max_z_mps2: float = field(
        default_factory=lambda: _environment_count('YILDIZ_A_MAX_Z', float('inf')))
    # VERTICAL acceleration ceiling. IT WAS NOT IN THE FIRST VERSION, and this PRODUCED A MEASURABLE
    # FAILURE (2026-08-08 night-2): when saturation was applied to horizontal only, the solver satisfied
    # the same framing/occlusion demand from the vertical channel, which REMAINS OPTIMISTIC -- "flows
    # through that cheap channel". Measured (11 running, EYL off n=6 vs on n=5):
    #     |u3| p90        6.77 -> 14.74 m/s   (2.2 kat)
    #     u3 > u1 ratio %7.5 -> %19.8 (2.6 floor) |cmd_vz| p90 6.38 -> 9.00 m/s (OPEN arms ALL at ceiling)
    #     ALTITUDE ABORT / engagement 0.36 -> 0.69 DEFAULT infinity = VERTICAL SATURATION OFF (see
    #     negative result ep2 in actuator_tau_lin_z_s). The measured value is 5.0 and is opened with env.
    #     5.0 m/s^2 from: ACTUAL vertical acceleration of the arms CLOSED (|az| p90 avg 4.57, large-error
    #     plateau avg 5.25) and WPNAV_ACCEL_Z = 500 cm/s^2 = 5.0 EXACTLY MATCHES m/s^2. So, unlike
    #     horizontally, in vertically WPNAV_ACCEL_Z is REALLY binding. NOTE: the LINEAR region base of the
    #     vertical has been LEFT as speed_latency_tau_s (1.00 s) -- not measured separately, this change
    #     only adds the MISSING ACCELERATION BOUNDARY, does not readjust the linear region. Horizontal
    #     acceleration plateau. Measured: 4.07 / 3.81 / 3.96 m/s^2 (|e| > 8 m/s, a_par p90). Changing
    #     WPNAV_ACCEL 250 <-> 500 did NOT change this plateau, so the nerve is not WPNAV_ACCEL.
    pitch_lpf_tau_s: float = 0.30
    pitch_alt_deg: float = -35.0
    pitch_upper_deg: float = 35.0
    # ey_ref = -(mount + pitch): the camera axis ACTUALLY slides with pitch, it is PHYSICS for the tape to
    # follow pitch. HISTORY (important): before there were clamp 1.2 s LPF + [-20,+8]. The reason was that
    # following the momentary pitch produced POSITIVE FEEDBACK (brake -> nose up -> ey_ref down -> harder
    # maneuver -> more nose up). That clamp is NOW HARMFUL: the feedback turns off correctly because the
    # hard CBF constraint carries the escalation->pitch effect IN THE MODEL, while the clamp only BLINDS
    # beta when the actual pitch exceeds +8 -- measured in the simulation: beta p95 = 12 appeared "safe"
    # while the target RAW was coming off the bottom edge of the frame. The clamp opened to +-35 (physical
    # recline limit), LPF lowered to 0.30 s (the frame looks INSTANTLY to pitch, not to the past).

    # ---- solver ---------------------------------------------------- BOLDEN BUDGET BUTTON
    # (YILDIZ_SOLVER_ABUNDANT=1): iteration_ceiling 26 -> 40, duration_budget_ms 13 -> 18. See Rationale and
    # HARDWARE WARNING. environment_solver_abundant(). Summary: The budget_cut ratio is 52-89% on each branch and
    # this is a MEASUREMENT noise overlaid on the A/B differences. The button is for SIM measurement
    # quality; Pi 5 decision IS SEPARATE (risk of loop missing there).
    iteration_ceiling: int = field(
        default_factory=lambda: 40 if environment_solver_abundant() > 0.0 else 26)
    iteration_floor: int = 5
    initial_iteration_ceiling: int = 400
    initial_budget_ms: float = 25.0
    initial_solution_count: int = 2
    handoff_prox_seed: bool = True
    # handoff PROXIMAL IRON. solve() sets the difference penalty as ||u - u_previous||^2; u_previous is NONE
    # in the FIRST loop and used to be left ZERO. So the first QP is lied to as "the previous command was
    # stopping" and the weight of r_delta_speed was pulling the first command to STOP -- whereas the vehicle
    # is currently flying with 17 m/s. U_warm was ALREADY seeded with handoff speed (see _warm_start);
    # what was missing was the proximal IRON. Measured (offline handoff, 42 running x 5 handoff yaw
    # speed): difference between the first command and the last command of the positioner 4.36 -> 4.17
    # m/s, average of the first 1 s 3.38 -> 2.50 m/s. The effect is small, but the COST is ZERO and the
    # sign is the same in every run.
    handoff_yaw_seed: bool = True
    # FIRST SAMPLE gain of yaw speed LPF at handoff (k= 1). This is the only item that contaminates the
    # FIRST residue of the disturbance (see _measure_yaw_speed); The real handoff gain comes from here.
    # Both are separate keys to ablation; default is ON. COLD START: there is no previous solution at the
    # time of handoff, warm start is only the final speed of the positioner. Measured: the hot-started
    # solution fits the exact optimum in the CYCLE 3 - 5 ( 0.15 - 0.25 s), but is still far away in the
    # first cycle. Giving ample funding to the initial 2 solution eliminates this delay; insignificant
    # next to the one-off 25 ms , t_go 3 - 10 s.
    tolerance_mps: float = 0.02
    # Stopping criterion PHYSICAL: stops when the input variation between successive iterations drops
    # below 2 cm/s (speed yaw is pulled to the same scale). There's no point chasing below that -- the
    # autopilot's speed loop isn't at that resolution anyway.
    duration_budget_ms: float = field(
        default_factory=lambda: 18.0 if environment_solver_abundant() > 0.0 else 13.0)
    # WALL CLOCK PROTECTION. IT WAS 6.0 and was EXCEEDING 38-47% of the loop in tour-3 -- meaning the
    # solver was regularly cut IN HALF. Moreover, since the control is done every 8 iteration, the number
    # of iterations is CLUSTERED in 8/16/24, successive loops are interrupted at DIFFERENT points and the
    # command gets jump from this: 1-5 in the Hz band Only 12-29% of the variance of yaw was explained by
    # ex+d_ex, the rest was due to SOLVER. Cycle 50 ms, p95 already 9-10 ms; When the budget is increased
    # to 13 ms, truncation becomes a RARE event and the convergence tolerance determines the number of
    # iterations (consistent). Checking is now done at EVERY iteration (no clustering).
    sqp_transition: int = 1
    # The coefficients LTV (c=KVV/r) are frozen around the projected range trajectory. 1 transition is
    # sufficient with warm-start orbit; 2 is more accurate but ~2x the time. The test file measures both.

    # ---- yaw politikasi --------------------------------------------
    yaw_command_provide: bool = True
    # If False, yaw remains on autopilot and MPC only controls speed (for the ablation experiment: "is a
    # separate FOV controller required?").

    # ---- HIT PHASE (2026-08-05) ----------------------------------- REQUEST (user, 2026-08-05): "From
    # the moment it sees the target, it accelerates and multiplies on it without letting go". The defect
    # measured was two-part: (1) speed parity (above, speed_ceiling_mps), (2) CLOSE RANGE COST STILL BEING A
    # "FOLLOW-UP" COST.
    #
    # Mechanics of ( 2 ): dominant items of cost in the near range is framing protection -- the HARD
    # constraint FOV collapses the vertical velocity band and the box yaw , the breaking contraction
    # collapses the lower band by degrees 4 , and the constraint says "climb". This is all true for LONG
    # TERM TRACKING (losing frame ends the run) but FALSE in the last second: there the value of keeping
    # the target in frame is less than the value of HITTING the target. Measurement: min range on
    # suspended (static) target 0.47 m, so terminal sensitivity already exists; what is lost is the
    # terminal PHASE ITSELF in the moving target.
    #
    # SOLUTION: A CONTINUOUS (gradual) mixing layer with range. s = clip((impact_range - r) /
    # (impact_range - impact_full), 0, 1)
    # s = 0 -> today's cost same; s = 1 -> pure capture. s does three things: * Opens the FOV bands to the
    # PHYSICAL EDGE ( 14 / 17.5 -> 19.0 ; vertical quasi- FOV 20.07 , i.e. 1 degree margin). Framing is
    # still constrained -- the camera is 0 deg FIXED mount so a target moving out of the frame leaves us
    # BLIND -- but the constraint now says "touch the edge" rather than "center". Horizontal limit 26 ->
    # 31 (physical 33 ). * It compensates for band narrowing caused by braking/acceleration ( 1 -s): the
    # terminal does not take the cost of the maneuver
    # from the framing band. * turns weights: framing ( q_ex , q_ey ) UP -- locking on the center of bbox
    # is the collision itself ( ex = 0 , ey = ey_ref IS THE CENTER of the frame, see. _framing_constant)
    # --, reward ( q_area , q_area_rate ) UP, acceleration/leaning penalty ( q_acceleration ) DOWN
    # (aggressiveness button; accept for 27 deg leaning < 1 s).
    impact_mode: bool = True
    impact_range_m: float = 22.0
    # ENTRY RANGE TO STRIKE PHASE. Derived from three measured numbers: * handoff range 26.8-33.5 m (17
    # authorization segment) -> 22 m BELOW THIS: the phase handoff is NEVER opened at the moment, i.e.
    # aggressive costing does not come into play in the first second when the solver is cold. * pass
    # lever 12 is armed at m -> 22 > 12: strike starts BEFORE phase pass detection (correct sequence). *
    # t_go: 22 tail tracking shutdown at m 35-21 = 14 m/s -> 1.6 s; head to head 56 m/s -> 0.4 p. So the
    # phase is "last 0.4-1.6 s".
    impact_full_range_m: float = 8.0
    # s = 1 (FULL release) range. 8 m, the same number as the opening threshold of the crossing arm
    # (miss_transition_opening_m) and just above the fuselage (~6.4 m wingspan): inside this range nothing the
    # framing constraint can say will improve the hit.
    impact_band_deg: float = 19.0        # vertical semi-FOV 20.07 -> 1 deg share
    impact_ex_limit_deg: float = 31.0   # horizontal half-FOV 33 -> 2 deg share
    impact_ex_multiplier: float = 1.0       # q_ex multiplier (at s= 1)
    impact_ey_multiplier: float = 1.0       # q_ey multiplier (at s= 1)
    impact_los_multiplier: float = 1.0      # Factor q_los_speed (at s=1)
    # FRAMING/PN MULTIPLIERS WHY 1.0 -- i.e. STROKE DOES NOT CHANGE these weights (2026-08-05, selected by
    # measurement):
    #
    # The user request was "Lock to center bbox (ex,ey -> 0)". IMPORTANT: THIS IS ALREADY LIKE THIS. The
    # framing references of the cost are ex -> 0 and ey -> ey_ref and (ex=0, ey=ey_ref) BY DEFINITION It
    # is the framing CENTER of the bbox (see _framing_constant). In other words, it is not the REFERENCE
    # that will be changed during the shooting phase, but the BRAKE imposed by the frame.
    #
    # GROWING the weights has been tried and MEASURED (closed loop, 6 scenario x 5 seed): * q_ex x3 ->
    # sharp/cross median min range 12.87 -> 12.84, in bend/cross 2.34 -> single seed 9.48 m distortion.
    # The reason is theoretically obvious: "ex -> 0" is PURE PURSUIT (see file title, "PURE PURSUIT vs
    # COLLISION COURSE"); increasing the weight pushes the optimization towards tail chasing, whereas the
    # term that sets up the collision is INERTIAL LOS SPEED. * q_ey enlarges the demand for x3 -> u3
    # (vertical speed perpendicular to LOS) and that speed directly replaces u1 (shutdown); Since c3 =
    # KDEG/r explodes at close range, even a small error ey produces large u3 -> impact phase BRAKES ITSELF
    # (measured: u1 at s=1 27.33 -> 25.15 m/s, i.e. closure REDUCES). * q_los_speed x2 -> panel average 6.75
    # -> 8.14 m (bad). All three knobs are left ADJUSTABLE (for ablation and real hardware) but the
    # default is 1.0: The job of the STRIKE phase is not to change the weight, but to resolve the KIT and
    # ACCELERATION PENALTY. Vertical framing is already protected by TWO mechanisms: nominal q_ey
    # (increased to 0.35 -> 0.60) and HARD tape (19 deg in STRIKE).
    impact_acceleration_multiplier: float = 1.0     # q_acceleration multiplier (at s= 1)
    # 0.35 -> 1.0 (2026-08-05, AFTER SIM TYPE, by measurement). In the initial design, the acceleration
    # (banking) penalty during the STRIKE phase was AFFECTED to 0.35x: "aggressiveness button". Sim showed
    # that there is NO NEED for this button and it has a HUGE COST: * NO NEED: speed parity alone was
    # enough -- u1 max 35.00, cmd_speed p95 34.78, shutdown range_rate_value min -18.4 m/s. So it already touches
    # on the ceiling; The authority for extra acceleration lay vacant. * COST: acceleration = TIN angle
    # (pitch = -atan(a/g), 5.84 deg per m/s^2) and camera 0 deg CONSTANT. The released momentum directly
    # throws the camera axis. Sim: |pitch speed| median 3.6 -> 13.3 deg/s (~952 px/s horizon shift =
    # visual flicker in frame), frame loss %8.6 -> %48.3, %18.5 of losses From the top edge (bottom %6.0),
    # %10.9 of it is OUTSIDE PHYSICAL FOV. OFFLINE A/B (straight route, target 21.05 m/s, 3 handoff x 2
    # seed): multiplier pitch speed top edge % min range cmd p95 0.35 3.81 12.0 1.93 35.0 0.70 2.74 9.6
    # 2.03 35.0 1.00 2.30 9.6 1.90 35.0 So releasing the constraint reduces pitch speed by %40, reduces
    # top edge loss by %20, and DOES NOT TOUCH CLOSE AT ALL. Free. NOTE: BEYOND 1.0 (INCREASING penalty on
    # STRIKE) has also been tried; see measurement chart below -- omitted because it breaks the closure.
    impact_reward_multiplier: float = 2.0     # Factor q_area, q_area_rate

    # ---- STROKE TERMINAL VERTICAL ALIGNMENT (2026-08-05, tur-3) ---------- RETIRED (gimbal branch,
    # 2026-08-05, user approval): reason for the existence of the mechanism It was "When eps grows, the
    # target comes out from the TOP edge, so climb to the target line". Physically stabilized gimbal +
    # Phase O (tilt tracking eps) solves the same problem by rotating the CAMERA; climbing bias was eating
    # the shutdown (offline: u1 28.29 -> 27.59). Code and unit tests (with explicit parameter) remain; for
    # ablation
    # impact_alignment_closing=True verilebilir.
    impact_alignment_closing: bool = False
    impact_alignment_relaxed_deg: float = 5.0
    impact_alignment_tau_s: float = 1.5
    impact_alignment_ceiling_dps: float = 8.0
    # Relaxed band 10 -> 5, based on simulation round 3 on 2026-08-05. The mechanism was verified
    # closed-loop: alignment_ref was active for 39 frames, all positive, preserving one-sided behavior.
    # Vertical separation at CPA fell from 0.81 m to 0.05 m, and CPA from 0.84 m to 0.62 m, with contact
    # vibe 2.0 -> 25.5. Upper-edge physical-FOV loss remained 16.24% -> 16.26%. The mechanism was active
    # in only 8.1% of terminal frames because impact-state eps had median 4.6 deg and p95 13.6 deg. A 10
    # deg deadzone excluded approximately 90% of frames. A 5 deg band lies just above the measured median
    # and expands the active window from approximately 8% to 45%. Two checks support the change: none of
    # the 39 active frames was followed by lower-edge overflow, and closure suffered no penalty, with
    # active u1 20.31 versus inactive 17.92.
    #
    # WHY THE CEILING AND TAU DID NOT CHANGE (offline solver scan, in the REAL band of the eps
    # distribution: 4.6 / 6 / 7.5 / 9 / 11 / 13.6 deg): arm active/6 u1 loss cover/max climbing avg
    # r10,t8,tau1.5 (tur-3) 2 0.36 / 2.14 0.13 r5, t8,tau1.5 5 0.85 / 3.98 0.60 r5, t8,tau2.0 5 0.63 /
    # 3.52 0.48 r5,t10,tau2.0 5 0.63 / 3.52 0.48 r5, t6,tau2.0 5 0.63 / 3.52 0.48 CEILING IS NOT BINDING
    # in this band: ref = (eps-5)/tau at most (13.6-5)/1.5 = 5.7 dps, i.e. Ceilings 6/8/10 give the same
    # solution (the three rows tau=2.0 in the table are EXACTLY the same). Changing it would be an empty
    # change. TAU 1.5 -> 2.0 reduce u1 cost by 26%
    # This reduces climb by 20%. Simulation round 3 measured no closure penalty, with active u1 20.31 >
    # inactive 17.92, so that tradeoff was unnecessary and climb authority was the desired effect.
    # Changing one setting per round makes the next trial interpretable. The underlying observation is
    # that standoff vertical offset causes upper-edge loss. The pursuer flies approximately 4-6 m below
    # the target. The camera sees the target above its axis, beta<0, and eps = asin(down/r) increases as
    # range closes: 19.5 deg at 12 m and 30 deg at 8 m, exceeding vertical half-FOV 20.07. Widening the
    # impact-state band cannot recover a target outside the physical FOV. In simulation, 10.9% of
    # upper-edge loss occurred outside that FOV.
    #
    # SOLUTION (PURE GUIDANCE, WITHOUT TOUCHING standoff_geom / SDF): In the STRIKE phase, when the
    # target's apparent rise exceeds the COMFORT band, CLIMB the hunter towards the target LINE. So in the
    # terminal the vertical standoff melts, the camera becomes flat, the target does not come out from the
    # top edge; flat camera also reduces visual judder (body movement = camera movement).
    #
    # MECHANISM: BIAS to vertical INERTIAL LOS SPEED reference. Nominally that term lasts sigma_el
    # (vertical speed LOS) -> 0 (parallel travel = standoff maintained). Reference in IMPACT:
    #     sigma_el_ref = beat * clip(( eps - relaxed) / tau , 0 , ceiling) eps >wanting POSITIVE sigma_el
    #     while relaxed turns off eps
    # (eps_dot = -sigma_el). PURE GEOMETRY: yalnizca eps = f(ey, aim)
    # uses (from ey image), DOES NOT TOUCH target telemetry.
    #
    # WHY DEADZONE ( relaxed_deg ): climbing in terminal REQUIRES vertical component from |v|<=ceiling
    # sphere, i.e. CLOSING FORWARD. Measured (single shot, eps = 13 deg , no deadzone): u1 35 -> 30 m/s .
    # To pay this price ONLY when the target is actually close to the edge, the bias is ZERO when eps <=
    # relaxed ( 10 deg , vertical
    # half-half of FOV); However, it is gradually activated when the target moves beyond 10 deg (leaving
    # aside 10 deg). comfortable < In 8 deg the closing cost becomes widespread, > In 14 deg the
    # intervention does not reach the target before it leaves the edge; 10 is in between. tau 1.5 s,
    # ceiling 8 dps: eps=19 (edge) while (19-10)/1.5 = 6 dps vertical LOS requires speed -- scale 10 below
    # dps, does not saturate vertical channel. eps=22 fits in the ceiling (8). CAUTION (one-sided):
    # requires climbing only at eps>relax (above the target axis and close to the edge); descent is NEVER
    # forced.
    #
    # OFFLINE/SIM GAP (IMPORTANT): offline engine enters terminal phase with eps < 0 (target ALREADY
    # flattened: -1.6..-5.0 measured deg), because the vertical dynamics of the simulation are NOT the
    # SAME as that of the sim -- hunter in sim In standoff, ~4 m REMAINS at the bottom (top-edge loss from
    # there), climbs to the level up to the terminal offline. So this mechanism CANNOT be measured in the
    # OFFLINE CLOSED LOOP (bias remains 0 in all scenarios, there is no risk of regression). Tested
    # deterministically at the solver LEVEL (test_impact_vertical_alignment); CLOSED LOOP verification belongs
    # to the SIM. This is the exact offline->sim divergence that the user pointed out in type-2. ----
    # ALTITUDE-AGNOSTIC TERMINAL VERTICAL ALIGNMENT (2026-08-07) ------- A/B BUTTON:
    # YILDIZ_VERTICAL_TERMINAL=1. DEFAULT IS OFF. Root cause, measurement and safety justification: see
    # environment_vertical_terminal(). In short: cost as PHASE O (tilt follows eps) pulls beta to ~0
    # vertical DOES NOT SEE standoff ANYMORE; The term PN means "parallel course", that is, "PROTECT
    # standoff". This switch puts a BIASED bias on the vertical LOS speed reference, which turns off the
    # eps.
    vertical_terminal: bool = field(
        default_factory=lambda: environment_vertical_terminal() > 0.0)
    # RANGE RAMP. 45 m = terminal_range_m (the TERMINAL threshold of the state machine) and the same
    # number as miss_arm_m -- we are NOT making up a new phase definition, we are using the existing
    # terminal definition. At 25 m the bias is FULLY strengthened. WHY 45 -> 25: shutdown in queue
    # tracking ~14 m/s, so this band lasts ~1.4 s and from 25 m to CPA there are ~1.8 s more. A total
    # window of ~3 s means a climb of ~1.3 m/s to a vertical offset of 4 m -- one seventh of the climb
    # ceiling (9 m/s), so the speed stolen WITHOUT CLOSING FORWARD is negligible. The old (retired)
    # mechanism was based on STRIKE: 22 starts at m, 8 saturates at m -> ~1 p. “Last second” was exactly
    # that.
    vertical_terminal_range_m: float = 45.0
    vertical_terminal_full_m: float = 25.0
    # DEAD TAPE (two-sided). In the old movement, 5 was deg and the reason was "frame edge" (the target
    # should not come out from the top edge). The aim here is NOT the framing but the multiplication
    # GEOMETRY, so the threshold is much narrower: 2 deg. Meaning
    # dz = r*sin(2 deg) -- 45 m'de 1.6 m, 25 m'de 0.9 m, 10 m'de 0.35 m.
    # So "2 deg dead band" means in practice "vertical residual 0.3-0.4 m is allowed in CPA"; It is of the
    # same order as our acceptance criterion (|vert| < 0.4 m). Going narrower converts the measurement
    # noise eps (bbox +-1-2 px) into a vertical speed command -- 1 deg = 25 0.44 measurement at m
    # uncertainty itself.
    vertical_terminal_relaxed_deg: float = 2.0
    # TAU: "In how many seconds to turn off eps". Since eps_dot = -sigma_el, sigma_el_ref = eps/tau is a
    # first-order closure. 1.5 s is the same order as t_go (~1.8 s) in 25 m -- wanting faster (tau<1) eats
    # forward closure, slower (tau>2.5) Doesn't catch up with CPA. The tau of the retired mechanism was
    # also 1.5 and the closing price was not measured in this band in the offline scanning.
    vertical_terminal_tau_s: float = 1.5
    # CEILING: vertical LOS speed request upper limit. Since the required vertical speed is w3 = ref * r /
    # 57.3, 8 dps is 45 6.3 m/s in m, 25 is 3.5 m/s in m. On the lowering side the physical ceiling is
    # already 4.5 m/s and the input box clips it; 9 m/s on the climbing side. So the ceiling is not
    # PHYSICALLY IN FRONT of the box, it is a safety valve against noise.
    vertical_terminal_ceiling_dps: float = 8.0

    # ---- DIRECT VERTICAL ERROR (P) TERM (2026-08-07) ---------------- A/B BUTTON: YILDIZ_VERTICAL_ERROR.
    # DEFAULT ON (1.0). Root cause, geometry and safety justification: see environment_vertical_error(). In short:
    # the above vertical_terminal is a DERIVATIVE (D) term; There is no line in the cost for the error
    # ITSELF. This button adds a DIRECT error line on status ey with reference to "target line" (eps = 0).
    # Two of them open INDEPENDENT; When opened together it becomes P+D.
    vertical_error: bool = field(
        default_factory=lambda: environment_vertical_error() > 0.0)
    # ENV MULTIPLIER (YILDIZ_VERTICAL_ERROR value). 1 = nominal q_vertical_error.
    vertical_error_multiplier: float = field(
        default_factory=lambda: max(environment_vertical_error(), 0.0) or 1.0)
    # WEIGHT. FIRST TRIAL was the same order as q_ey (0.60) and MEASURED IN SIM WAS UNSUFFICIENT: in the
    # ellipse/DOWN+4 run the term was ON in 91.5% of the frames but in CPA the vertical is now 2.02 ->
    # 1.78 m, i.e. almost unchanged (dhP_ellipse_20260807_045603). REASON: this term COMPETES with the term
    # PN (q_los_speed=4.0) on the same case, and PN literally means "PROTECT vertical standoff". Since ey_k
    # ~ ey_0 - c3*w3*t_k at the solver horizon, equating the gradients of the two terms gives the
    # vertical velocity at equilibrium: w3 ~ q_vertical_error * e_0 * SUM(t_k) / (q_los_speed * c3 * N) (scales
    # equal: scale_ey_deg = scale_los_speed_dps = 10, simplifies). The vertical speed REQUIRED is in two
    # parts, both of which grow as r closes: (1) Even keeping eps CONSTANT requires climbing up to dz/r *
    # closing -- dz 4 m, r 25 m, closing 14 m/s -> 2.24 m/s requires (2) more dz/t_go to MELT dz up to CPA
    # -- 4 m / 1.8 s -> 2.2 m/s. So, the ~2.2-2.8 m/s rank is required in the terminal. solver scan
    # (eps=+8 deg, r=25 m): q=0.6 -> u3 1.63 m/s, q=1.5 -> 2.46 m/s,
    # q=2.0 -> 2.81, q=4.0 -> 3.97 (q>4'te doyuyor).
    # 1.5 was chosen because in a CLOSED LOOP, vertical_terminal (the DERIVATIVE), which is the arm it was
    # measured to produce, produces 2.22 m/s at the same point: that is, the P term is given the SAME
    # AUTHORITY as the mechanism known to work -- a new level of aggressiveness is NOT introduced. Scales
    # with ramp and env multiplier: 45 0 in m, 25 full in m.
    q_vertical_error: float = 1.5
    # RANGE RAMP: SAME numbers as vertical_terminal (45 -> 25 m). REASON SAME: 45 is the TERMINAL threshold
    # of m state machine and is the same number as miss_arm_m; We are NOT making up a new phase
    # definition. At 25 m the term becomes full strength, leaving CPA with ~1.8 s. Additionally, when two
    # buttons are opened together (P+D), both of them work in the same window, reducing the interpretation
    # of the lever to a single variable.
    vertical_error_range_m: float = field(
        default_factory=lambda: _environment_count('YILDIZ_VERTICAL_RAMP_START', 45.0))
    vertical_error_full_m: float = field(
        default_factory=lambda: _environment_count('YILDIZ_VERTICAL_RAMP_LAST', 25.0))
    # *** RAMP WINDOW IS NOW ENV-ADJUSTED ( 2026 - 08 - 09 , dz analysis). *** DEFAULTS UNCHANGED ( 45 ->
    # 25 ), so if env is not given, BIT-SAME. WHY IT WAS ADJUSTABLE -- dz< 0 root cause analysis (separate
    # agent): ( 1 ) VERTICAL CHANNEL DEAD TIME ~ 1.5 s: measured tau_lin_z 2.16 s, model assumes 1.0
    # seconds; The EXECUTION rate of the pre-CPA request is % 11 . ( 2 ) eps IS AN ANGLE: Even if dz remains
    # constant, eps grows as r decreases, so demand NATURALLY arises at the LAST SECOND and hits the
    # ceiling.  45 -> 25 m window so it's TOO LATE: R= 30 to CPA at m 4 - 6.7 while there are s the ramp
    # is just starting, R= 15 to CPA at m 0.4 - 1.4 s remaining -- SHORT THAN DEAD TIME. When the demand
    # arises, there is no time to execute. WHAT CHANGES IS NOT THE AMPLITUDE *TIMING*: bringing the ramp
    # forward (e.g. 80 -> 45 m) creates the demand BEFORE the dead time, so the same amplitude can be
    # EXECUTED by spreading smoothly. In the dv1 package YILDIZ_VERTICAL_RAMP_START = 80
    # YILDIZ_VERTICAL_RAMP_LAST = 45 together with YILDIZ_TAU_LIN_Z = 2.2 YILDIZ_A_MAX_Z = 5 is given:
    # correct tau is REQUIRED to softly spread the early born demand (tau ALONE failed on Sep2 because
    # demand was still rising late). DEADBAND (two-sided, resets the WEIGHT -- does not shift the
    # reference).  1.0 deg selected: dz = r* sin ( 1 deg ) means, 25 m in 0.44 m, 15 m in 0.26 m, 10 m at
    # 0.17 m. Acceptance criteria in CPA |vertical| Since < 0.4 m, this band remains BELOW the criterion
    # near the CPA, and at a distance it easily absorbs the noise of bbox (+- 1 - 2 px ~ 0.1 - 0.2 deg).
    # The dead band of vertical_terminal ( 2.0 deg ) is wider because it is DERIVATIVE
    # The term translates directly to the speed command; The P term here competes with other costs in the
    # solver, so it can be kept narrower.
    vertical_error_relaxed_deg: float = 1.0

    # ---- t_go SHAPE VERTICAL SPEED REFERENCE (2026-08-07, eng-3) ------- A/B BUTTON: YILDIZ_VERTICAL_TGO.
    # DEFAULT ON (2.0). Root cause, closed-form justification, and safety: see environment_vertical_tgo(). In
    # short: FIXED tau of branch D (vertical_terminal) remains longer than t_go in the following terminal;
    # This button connects tau to TIME REMAINING:
    #     tau_eff = clip(t_go / k, tau_min, tau_max)
    # and thus takes both the error and the SPEED to zero. OPEN INDEPENDENTLY from VERTICAL_TERMINAL: this
    # button alone engages the same speed-reference line (main lever is P + TGO).
    vertical_tgo: bool = field(
        default_factory=lambda: environment_vertical_tgo() > 0.0)
    # ENV MULTIPLIER (YILDIZ_VERTICAL_TGO value). 1 = nominal k.
    vertical_tgo_multiplier: float = field(
        default_factory=lambda: max(environment_vertical_tgo(), 0.0) or 1.0)
    # k -- "Close the error in what fraction of the remaining time". Since in closed form e(t) = e0 *
    # (t_go/T)^k the meaning of k is the direct EXPONENT of the terminal profile: k = 1 : e_dot remains
    # CONSTANT -> EXCEED (today's data error) k = 2 : e_dot ~ t_go is reset linearly
    #   k = 3 : e_dot ~ t_go^2, vertical residual dz ~ t_go^4 k >> 3 : error STOPS for a long time, forced
    #   to melt at the last moment; the gain is satisfied, but the profile becomes "LATE AND HARD".
    #
    # OFFLINE (solver-loop scan, P + TGO, DOWN=+4, eps0=+5.1 deg, target 21 m/s FLAT AND LEVEL) gave
    # VERTICAL SPEED @CPA:
    #   k=2.0 -3.18 | k=2.5 -0.99 | k=2.6 -0.71 | k=2.8 -0.56
    #   k=3.0 -0.52 | k=3.5 -0.45 | k=4.0 -0.40 | k=6.0 -0.32  [m/s]
    # (reference: P alone -5.35, closed arm -5.76 m/s.) Offline KNEE between 2.5-2.8; By looking at
    # offline k=3.0 was selected.
    #
    # *** CLOSED LOOP REFUSED THIS -- k=2.0 SELECTED. *** SIM A/B (DURATION=300, ellipse DOWN+4), vertical
    # residue in CPA [m]: closed arm : 2.02 / 1.03 P alone : 1.10
    #   k=3.0      : 0.58 / 0.64      k=2.0        : 0.41 / 0.15
    # and k=2.0 produced ACTUAL PHYSICAL CONTACT (impact_successful, vibe) ON ALL THREE ROUTES (ellipse
    # DOWN+4, straight DOWN+4, ellipse DOWN=0) 234/40/24; no other arm produced contact in the entire
    # campaign). WHY OFFLINE FAILED (important for the record): the failure mode of the offline simulation
    # was EXCEED, where big k is rewarded. In the REAL run -- especially WHEN the target is TURNING -- the
    # mode of failure is the OPPOSITE: the law closes LATE. The signature of the k=3.0 branch is exactly
    # this: it no longer SIGN + (can't catch up), |dz| 15-25 in the m band 2.77 HOLDS on m and the last
    # 1.3 dives on s. k=2.0 pushes shutdown EARLIER: in the same band
    # 8-15 m |dz| 2.95 -> 0.08 m. So k is the trade-off between "hanging" and "running late", and in REAL
    # geometry the optimum is SMALLER. (The same offline->sim split occurred in tur-2; rule: offline
    # scanning gives the DIRECTION, closed loop gives the DECISION.) NOTE: env MULTIPLIER; Since this row
    # has changed, the OLD running labels are shifted -- YILDIZ_VERTICAL_TGO=1.5 to reproduce the branch
    # k=3.0. Closing cost was MEASURED and HEC (arrival in offline CPA 3.15-3.20 s in each branch; in sim
    # horizontal CPA in ellipse 0.96/1.84 -> 0.24/0.21 m, i.e. IMPROVED).
    #
    # *** CIRCUIT-5 CLOSED LOOP PROVISION: k=2.0 APPROVED (n=6 vs n=6). *** The above selection was made with
    # n=1-2 and WAS NOT DECIDABLE: two runs of the same configuration 15-8 gave m in the m band 0.19 and
    # 2.78, so the INTRA-RUN propagation is due to the INTER-CELL difference. it was big. In Type-5, the
    # ellipse DOWN+4 cell is increased to n=6 in each branch (3 normal solver budget + 3
    # YILDIZ_SOLVER_ABUNDANT=1; budget is a COMMON FACTOR, not separating the arms, as both arms are balanced
    # in the same way). metric [median (min..max)] k=2.0 k=3.0 band |dz| 25-15 m 1.88 (0.82-2.98) 2.22
    # (2.00-2.50) band |dz| 15-8 m 1.23 (0.19-2.78) 1.81 (0.61-2.78) tape |dz| 8-0 m 1.21 (0.50-1.39) 1.17
    # (0.57-1.79) CPA vertical |residual| 0.35 (0.09-1.59) 0.51 (0.03-1.74) close-CPA |vertical| median
    # 0.58 (0.09-1.43) 0.86 (0.37-1.89) CPA horizontal 0.73 (0.37-2.04) 1.36 (0.25-2.04) ISCAN 1.5 (1-3)
    # 3.0 (1-5) PHYSICAL CONTACT 2/6 0/6 k=2 excels in six metrics, k=3 only 8-0 excels in the m band and
    # that too
    # with negligible difference (1.17 vs 1.21). Second cell (solid DOWN+4, n=2+2, ABUNDANT) facing the same
    # direction: 8-0 band 0.95 vs 1.54, CPA horizontal 1.68 etc 2.81, MISS 4 vs 7 -- all in favor of k=2.
    # INTEGRITY NOTE: in any VERTICAL metric the ranges are NOT discrete; The decision is based on the
    # consistent aspect of MEDIANS + number of contacts. All the physical contacts of the campaign (now 4
    # units) came from the handle k=2, k=3 did not produce vibe > 4 in any runs.
    vertical_tgo_k: float = 2.0
    # TAU BASE. At t_go -> 0 tau_eff -> 0 and demand explodes; the base cuts this. 0.30 s was chosen
    # because (a) the actuator time constant is 1.0 s, so a vertical velocity request shorter than 0.3 s
    # CANNOT be followed anyway -- anything smaller will only produce noise; (b) It comes into play under
    # t_go = 0.60 s (k=2.0), which means r < 9 m and there the closed-form error should already be
    # negligible with ~(t_go/T)^2. So the base should have NO binding in NORMAL running; If it is binding,
    # it means that the law has not worked (it is a diagnosis). SENSITIVITY MEASURED (offline, k=3.0,
    # DOWN=+4): tau_min 0.15/0.30/0.45/0.60 -> vertical speed @CPA -0.42/-0.52/-0.61/-0.71 m/s. So around
    # 0.30 is a FLAT region, it is not a critical setting. NOT binding in sim either: frame rate sticking
    # to tau base in runs is %2-12 (k=2.0 in arm %4-11), so the law almost always works with t_go itself.
    vertical_tgo_tau_min_s: float = 0.30
    # TAU CEILING = SAME NUMBER AS vertical_terminal_tau_s (1.5 s). So this lever CANNOT be FAR (or while
    # t_go is obsolete) more AGGRESSIVE than the old fixed-tau lever; it hardens only towards the
    # terminal. A new level of aggression is not being invented.
    vertical_tgo_tau_max_s: float = 1.50
    # t_go VALIDITY GATE. range_rate_value is LPF (tau 0.30 s) and in crossed geometry closure drops to ~0 ->
    # t_go explodes. When shutting down under 1.0 m/s, ignore t_go and downgraded to tau_max (softest).
    # CAUSE 1.0: 45 means 1 m/s shutdown t_go = 45 s, already exceeds the ceiling (vertical_tgo_ceiling_s); The
    # threshold is practically a signal test of "are we closing", not a setting.
    vertical_tgo_closure_min_mps: float = 1.0
    # t_go CEILING. The law only works within 45 m (ramp vertical_s) and there the realistic t_go 3-4 s.
    # Values ​​above 6.0 s mean LPF warm-up/temporary opening; Trimming puts tau_eff at tau_max, so it
    # falls on the softest branch again.
    vertical_tgo_ceiling_s: float = 6.0

    # ---- SUCCESSFUL IMPACT DETECTION (2026-08-05, iteration-4) -----------------
    impact_success_detection: bool = True
    impact_success_vibe: float = 15.0
    impact_success_range_m: float = 3.0
    # WHY THERE IS (MEASUREMENT PROBLEM, user request): user wants REPEATABLE hit but every SUCCESSFUL hit
    # ENDS the run -- vehicle hits target, rolls over and crashes (type-3: range 1.04 vibe at m 25.5 then
    # roll -106 deg / pitch -51.8 deg). So a sample comes out of a run and stays at n=5; no statistical
    # power. Moreover, the question "did we hit" was removed from the CPA PREDICTION until now.
    #
    # Detection: vibe > impact_success_vibe AND range < impact_success_range_m . Thresholds come from
    # measured round- 3 values: contact gave vibe 2.0 -> 17.4 at 2.32 m and 25.5 at 1.04 m. A non-contact
    # round- 2 pass at 0.85 m gave vibe only 3.3 . The 15.0 threshold separates these groups with a 4 .5x
    # margin over the non-contact pass. A range gate is required: LOG_DICTIONARY records ground-contact
    # vibe 150 - 345 . The r < 3 m condition excludes most ground contacts. A ground impact within 3 m of
    # the target remains difficult to distinguish, although the altitude floor is already 25 m.
    #
    # RULE NOTE: vibe is OUR telemetry (VIBRATION message), NOT the target's. The "only range from target"
    # rule is not violated; The second input of detection is the range itself. LATCHed: declared once per
    # engagement (reset() clears), otherwise the event would be written every cycle for the duration of
    # the engagement. ---- PN RESUME IN BLIND FLASH (with env push button, default ON) ---- Root cause,
    # mechanism and rails R1-R5: see fig.  environment_blind_pn ().
    blind_pn: bool = field(default_factory=lambda: environment_blind_pn() > 0.0)
    blind_pn_maximum_s: float = field(
        default_factory=lambda: _environment_count('YILDIZ_BLIND_PN_MAXIMUM_S', 1.0))
    blind_pn_tau_s: float = field(
        default_factory=lambda: _environment_count('YILDIZ_BLIND_PN_TAU', 0.7))
    blind_pn_ex_margin_deg: float = 5.0
    blind_pn_ex_absolute_deg: float = field(
        default_factory=lambda: _environment_count('YILDIZ_BLIND_PN_EX_ABSOLUTE', 26.0))
    # R3' ABSOLUTE FRAME CLAMP (2026-08-09, kpn1b+kpn2 measurement). R3 alone was NOT ENOUGH: clamp base
    # "last REAL |ex|" and on the RETURN it already becomes 30+ deg, meaning the base was shifting.
    # Measured -- in blind squares |ex| p90: kpn1b 22.9, kpn2 38.6 deg. Framing HALF-angle 20.07 deg. In
    # other words, dead reckoning was dragging the target FAR OUTSIDE the frame, the "dream", and the law
    # was chasing that ghost; rotation phase is therefore broken (shutdown +3.87 -> -7.93/-2.89, fresh %74
    # -> %46/%56). CORRECTION: |ex| <= min(|ex0| + numerator, blind_pn_ex_absolute_deg).
    #
    # *** DEFAULT 20 -> 26 (2026-08-09, flight proof). *** 20 was trimming leakage, but it was also
    # trimming LEGITIMATE terminal overflows. Measured by 26 (engagement pool, COMMON LAYER off0>=3):
    # ceiling 20 (3 running, n=21): CPA3B p50 5.99 | HORIZONTAL p50 4.66 | <3m 3 (%14) ceiling 26 (2
    # running, n=10): CPA3B p50 3.50 | HORIZONTAL p50 2.77 | <3m 4 (%40) -> median CPA -%42, horizontal
    # component -%41, <3m ratio 3 SOLID. RETURN band preserved (closing > 0, opening < %50) and blind |ex|
    # MAX chirped right at 26.0 (NO leaks). REMAINING OPEN: branch-meter queue (m transitions 0.69-0.92)
    # NOT RETURNED -- best transition 1.08 -> 1.22. The clamp corrected the median but did not explain the
    # tail; as a separate question on TO_TEST. RATIONALE: IT IS UNKNOWN WHERE the target is outside the
    # frame; The fact that the dead account goes there "safely" is not information but a fabrication. LET
    # him STAND to the side, do not go out.
    blind_pn_range_m: float = field(
        default_factory=lambda: _environment_count('YILDIZ_BLIND_PN_RANGE', 12.0))
    # Range gate: when the internal range exceeds this threshold, no advancement is applied, preserving
    # the prior returned-command behavior. Benefit occurs in the terminal window: with blind_pn=1, range
    # p50 was 3.4 m for kpn1b and 8.9 m for kpn2. Closest CPA improved from 1.44 m to 0.84/0.75 m. Loss
    # comes from blind frames at long range during return. The 12 m threshold covers the terminal window
    # beyond the 8 m impact range and excludes distant frames.

    impact_blind_coast: bool = True
    impact_blind_range_m: float = 8.0
    # BLIND FLOATING (coast). While in the STRIKE phase, the resolver DOES NOT RUN WHEN range is within
    # impact_blind_range_m and bbox is stale; The last command is repeated exactly. Reason: in stale bbox,
    # ey FROZIES and the framing constraint becomes harder by measuring the vertical speed it produces
    # (measured, ellipse block 2: ey -14.57 rotated in beta 1.4 in s 4.96 -> 37.42). Inside 8 m is t_go <
    # 0.5 s: the plan is already set, there is nothing a new maneuver can fix, and since the camera is 0
    # deg FIXED, we are already blind.
    #
    # RANGE GATE ADDED WITH MEASUREMENT (2026-08-05). In the first release, the gate was ONLY stale
    # (during the entire STRIKE phase). MEASURED in closed loop which BREAKS THE CAPTURE: bend/cross min
    # 2.47 -> 9.51 m, sharp/cross 3.81 -> 12.97 Reason: skeleton counts bbox FRESH up to 0.70 s and
    # continues to call the controller; our door stale_constraint_s 0.30 p. So EVERY space between 0.30-0.70 s
    # was converted to glide -- 35 m/s 14 m uncommanded flight, at the moment when the geometry changes
    # most rapidly. The CORRECT answer already existed in that interval: drop the hard constraint (gate
    # stale_constraint_s), KEEP solving. Blind glide is only in effect at the last 8 m where correction is
    # physically meaningless. At actual loss exceeding 0.70 s, the skeleton already holds the last command
    # 1 s (gap_hold_s) -- this is the natural continuation of blind glide. The SCA state machine
    # (INDEPENDENT from the range witness bbox, comes from the estimator) terminates the glide anyway.

    # ---- MISSING CASE MACHINE (2026-08-05) -------------------------- WHY NOT IN COST FUNCTION: MPC
    # CANNOT SOLVE "I passed, I should quit" situation by optimization. It is cost CORRECT to produce
    # commands to the target even as the range is opened (speed LOS -> 0, field reward, framing -- all
    # still point to the target). So "leave" is not an optimum, but a TERMINATION DECISION; It is
    # installed as a state machine ON TOP of the MPC: SHUTDOWN -> TERMINAL -> HAZARD -> (release
    # authority)
    #
    # PROBLEM MEASURED (static target, mpc_diagnostic_20260804_220521 / _223143 / _223930, authorization segment
    # 17): * First terminal handover in each segment occurs at t+4..5 s (CPA 0.9-17 m), then the
    # controller CONTINUES TO GIVE COMMAND. * Then the range is ON up to 220 m and then closed again:
    # 220521 seg0 profile 27 -> 15 (t+4) -> 220 (t+21) -> 3 (t+35). So MPC IS re-engaged but with a GIANT
    # CIRCLE of radius 110 m, lasting 30 s. * PHYSICS: 18 m/s and WPNAV_ACCEL 5 m/s^2 with the smallest
    # turning radius v^2/a = 65 m; U-turn 130 m displacement, i.e. ~11 s ROTATION ALONE. As the speed
    # ceiling increases to 35 m/s the radius jumps to 245 m -- meaning that post-transition
    # self-reengagement QUICKLY BECOMES IMPOSSIBLE. Cheap way: release authority, position guidance (with
    # full telemetry) reposition and reassign to standoff. GAIN of this machine (replayed offline in the
    # same 17 segment): wasted flight becomes shorter 4.4-64.4 s, average ~28 s.
    miss_mode: bool = True
    terminal_range_m: float = 45.0
    # If entered INSIDE this range, the status becomes TERMINAL (reference:
    # formation_KILLER.ATTACK_TERMINAL_M = 45, set in actual flight). Since we have the handoff gate 60 m
    # (bbox_to_redis.TRANSITION_RANGE_M), 45 m leaves a real CLOSING phase: The engagements inherited in 45-60
    # m occur first in CLOSING. TERMINAL is ONLY a status label here -- it does NOT change the control law
    # (the linear field reward of MPC already generates terminal aggressiveness itself).
    miss_arm_m: float = 45.0
    miss_opening_m: float = 30.0
    # CLASSIC TWO (reference: ATTACK_ABORT_ARM_M=60 / OPEN_M=30). ARM 60 -> 45 DROPPED because in KILLER
    # the attacker engages at 120-150 m and "60 m" was evidence that you were really close; IN US, the
    # authority itself is already transferred at <=60 m, that is, an ARM of 60 would be filled at t=0 at
    # each handoff time and would not carry the "we are close" information. 45 m is above the measured
    # handoff range band (26.8-33.5 m) but below the handoff door (60): the rule is only armed after it is
    # ACTUALLY closed. OPEN 30 retained as is: 25-30 m/s at closing speed ~1 s greater than 25 times the
    # range measurement noise (fit ~1.2 m sim).
    miss_transition_arm_m: float = 12.0
    miss_transition_opening_m: float = 8.0
    # PASS LEVER -- FAST shortcut to the classic duo. If the transition is approved, there is no need to
    # wait for 30 to open: 8 is enough. transition_arm 20 -> 12 m (2026-08-05, 35 m/s tour). OLD RATIONALE: CPA
    # at actual terminal pass 0.92 / 2.19 / 7.27 / 8.76 / 9.5 / 11.0 m, handoff range 26.8-33.5 m; 20 m
    # separated the two. NEW RATIONALE: there is a THIRD cluster to separate -- MID-PHASE Oscillation
    # (measured case: 18.1 at m -12.4 brakes and turns with m/s, 42 opens to m, then RETURNS and hits 0.43
    # atm). This cluster was separated by CLOSING SPEED at the ceiling 18 m/s (
    # transition_closure_threshold 15 m/s : actual transitions - 17.7 ..- 29.0 , oscillation - 12.4 ). At
    # the ceiling of 35 m/s that distinction is COLLAPSING: * the actual terminal pass closure rate in
    # pure tail chasing is 35 - 21.05 = 13.9 m/s -- i.e. BELOW THE OLD THRESHOLD. Measured:
    # transition_closure_threshold = 15 at tail tracking 0 / 16. * at the same ceiling, the braking speed
    # of the swing arm also increases. So the velocity axis does NOT separate anymore. GEOMETRY SEPARATES:
    # the actual transitions are all CPA <= 11 m, the CPA of the oscillation case is 18.1 m. The arm
    # circle 12 m separates these two clusters INDEPENDENT OF THE SPEED CAP (CPA is a length, it does not
    # scale with the ceiling) and is still well below the handoff range ( 26.8 +). Discrimination test
    # mpc_test protected by scanning at 5k(c).  transition_opening 8 m: after confirmed CPA the range is
    # opened with closing speed ( 14 - 56 m/s ), 8 m ~ 0.14 - 0.57 p. The ~ 6.7 fit of range noise (fit ~
    # 1.2 m), i.e. single sample bounce cannot be triggered.
    miss_absolute_m: float = 120.0
    # ABSOLUTE CANCEL (reference ATTACK_ABORT_MAX_M = 250). 120 = twice the handoff gate 60 m; The outside
    # of the video guidance's doctrinal envelope. Measured: missed segments 77-243 were holding authority
    # up to m. This is a REDUCED PROTECTION -- the 45/30 fires the lever first. IF SIMPLE PASSAGE RULE IS
    # OPEN (YILDIZ_TRANSITION_SIMPLE=1) MAKE THIS 300: since that rule removes the range gate, handoff can come
    # at 150+ m and 120 cancels the limit transfer at ~1 s and with bbox produces ping-pong (measured:
    # cycle ~1.2 s, ~50 handoff per minute). 300 = numerator above detection horizon (~200 m).
    miss_time_timeout_s: float = 8.0
    # TIMEOUT (reference ATTACK_TIMEOUT_S = 14). Its only job is to break the "it doesn't close but it
    # doesn't open either" (co-quick tail chase) deadlock: the 45/30 lever will NEVER fire there. 15 -> 8
    # s (2026-08-05, 35 m/s tour). The threshold is a measure of SPEED: the handoff envelope 60 m and the
    # REACHABLE closure in the pure tail now closes at 35 - 21.05 = 13.9 m/s, i.e. 60 m 4.3 s. 8 s is ~2
    # times that -- allowing for lags, bending, and settling, but diagnoses "not closing" at 8 s rather
    # than 15 s. WHY SHORTENING IS IMPORTANT: Smallest turning radius v^2/a = 245 m on 35 m/s (65 m on 18
    # m/s). Every second wasted means 35 m travel and geometry that is much more expensive to reposition;
    # timeout now scales INVERSE with speed cap. ---- PROGRESS-BASED TIMEOUT (with env button, default
    # OFF) Root cause, measurement and design: see environment_progress_clock().
    progress_clock: bool = field(
        default_factory=lambda: environment_progress_clock() > 0.0)
    progress_closure_threshold_mps: float = 1.0
    # PROGRESS MEASURE: If the best-so-far recovery SPEED (best_previous - best_now) / window >
    # threshold exceeds this threshold, WE PROGRESS.
    #
    # WHY BEST-SO-FAR RANGE: instantaneous and windowed raw-range criteria were both tested during
    # design, and oscillations fooled both.
    # * Instantaneous range_rate_value (LPF tau 0.30 s) oscillates at 1-2 Hz. The synthetic case
    # r(t) = 25 + 2*sin(2t), with no net change in range, reported "closing > 1.0" on about half the
    # frames. The timer moved backward at 0.5 s/s and forward at 1.0 s/s, for a net rate of about
    # +0.25 s/s. Its 8 s limit would take 32 s to reach, although the absolute 22 s limit fired
    # first. This incorrectly prolonged the constant-separation tail chase that the timeout should
    # terminate.
    # * Windowed raw range, (r_previous - r_now)/duration, was also fooled by the same sinusoid
    # (22.1 s). Oscillations with periods comparable to the window can fool a finite window;
    # enlarging it also adds delay.
    # Best-so-far range is monotonically non-increasing. An oscillation cannot repeatedly create a
    # new minimum, so this measure rejects oscillatory apparent progress by construction. It covers
    # both design cases, continuous closing and discrete new minima. Sustained closing continually
    # reduces the best-so-far range. The 1.0 m/s threshold is above the noise floor while still
    # detecting an equal-speed tail chase with progress near ~0.
    progress_window_s: float = 2.0
    # The window in which the gain rate is measured. Although individual frames appear sluggish with
    # intermittent occlusion (bend, frame gap), the gain of 2 s exceeds the threshold.
    progress_gain: float = 1.5
    # When advancing the clock rewinds (partial refresh) with ( 1 - 1.5 ) = - 0.5 s/s, when at rest + 1
    # s/s.  1.0 = returns only, > 1 = returns. Since it is proportional, it is independent of the frame
    # rate.
    progress_ceiling_s: float = 22.0
    # ABSOLUTE CEILING: ISTANBUL after this period even if there is progress. Infinite loop safety.
    miss_start_protection_s: float = 1.0
    # WATCH WILL NOT BE DECLARED in the first 1 s after handoff. handoff instantly the solver is cold (the
    # first 2 solution is large budget), LPF is seeded and the range filter fits; The temporary opening
    # produced in this window is not a miss.
    range_rate_tau_s: float = 0.30
    transition_range_rate_threshold_mps: float = 3.0
    transition_closure_threshold_mps: float = 10.0
    transition_confirmation_loop: int = 4
    transition_area_confirmation_loop: int = 6
    # PASS DETECTION -- TWO INDEPENDENT WITNESSES, NO TARGET TELEMETRY. KILLER uses the analytic range
    # derivative: d(rng)/dt = unit_bearing . (target_rate - own_hizimiz) but we have TARGET SPEED
    # FORBIDDEN (visual_base contract: only RANGE from the target). Our counterpart is two signals:
    # (1) range_rate_value = d(internal_range)/dt, LPF tau 0.30 p. Derivative of RAW range measurement CANNOT be used:
    # r_measurement jumps +-1.5 m in successive samples, at dt=0.05 s this means +-30 m/s noise. internal_range is already
    # a model-advanced + gain 0.35 filter (see _range) and its derivative is clean. MEASURED TRANSITION
    # SIGNATURE is very strong: dr/dt in CPA 2 turns from -27 m/s to +23 m/s in the cycle (jump of 50
    # m/s), i.e. 3 m/s threshold ~17 multiple. (2) area_rate = d(bbox field)/dt (already exists in
    # mpc_diagnostic, comes from bbox, NOT from telemetry). Changing sign means "the target is moving away". WHY
    # "OR" NOT "AND": measured so that both are UNRELIABLE on their own but distort differently -- *
    # area_rate SATURATES in CPA: 220521 in seg0 bbox takes up entire frame (area 34440 px^2 ceiling), as
    # the growth rate LPF increases to +43000, its sign change 2.5 s is DELAYED. * the range channel
    # estimator 10 is Hz so it rarely gets stale; Also in CPA detection style is lost frequently (223143
    # seg4: bbox_age 0.64 s, field is ROTATED) -- there the field is completely blind, the range works. So
    # while one is loud/blind, the other confirms: both witnesses ALONE are sufficient, but each with its
    # own CONFIRMATION COUNTER.
    # has to fill (4 cycle ~ 0.2 s range, 6 cycle ~ 0.3 s field; longer because the field is noisier).
    #
    # transition_closure_threshold_mps = 15: THIRD AND MANDATORY CONDITION -- for a pass to be declared, the range
    # speed must be this NEGATIVE (closing) as az INSIDE the circle transition_arm. WHY: just changing the sign
    # of the range does NOT mean "passed" -- MPC may make a wide oscillation in the mid-stage, 18 may slow
    # down at m and the range will open again. THE TWO ARE PHYSICALLY SEPARATE: the terminal crossing is a
    # FULL SPEED penetration. MEASUREMENT (17 real authority segment + offline engine; r<=20 most negative
    # range speed seen in m): real terminal pass : -17.7 -19.7 -19.9 -20.8 -23.9 -27.6 -28.7 -29.0 m/s
    # slow approach / braking : -6.7 -7.8 -8.2 -8.8 -11.9 m/s engine oscillation case : -12.4 m/s (CPA
    # 18.1 m, then opened to 42 m, TURNED BACK and hit 0.43 m -- so declaring a WAT is a REAL mistake)
    # 15.0 was between the two clusters, 2.6 in both directions m/s He was leaving a share.
    #
    # 15.0 -> 10.0 (2026-08-05, 35 m/s tour). OLD ASSUMPTION TURNED FALSE: "if the ceiling rises to 35 the
    # closing speed of the actual passages also increases" is true only in HEAD-HEAD geometry. In pure
    # TAIL tracking, the closing speed is LIMITED to the target speed: closing = v_pursuer - v_target = 35 -
    # 21.05 = 13.9 m/s i.e. 15 m/s threshold in the tail MATHEMATICALLY unattainable. Measured: 0/16 fever
    # (ellipse 0/9, target_infinity 0/7) -- so the toggle lever NEVER worked in this regime. 10.0 leaves 3.9
    # m/s below 13.9; if the target increases to 25 m/s (shut-down 10.0) the threshold again reaches the
    # full limit, so this number depends on the TARGET SPEED and
    # If the target speed changes, it must be measured again. DISTRIBUTOR TASK HAS BEEN ASSESSED TO
    # GEOMETRY: false declarations are no longer the closing speed but the miss_transition_arm_m = 12 m
    # eliminates the algebra (mid-stage oscillation CPA 18.1 m > 12). The closing condition has NOT been
    # removed, it remains as the second defense: 10 m/s is above the measured slow approach / brake
    # cluster (-6.7..-8.8 m/s).
    miss_coast_speed_mps: float = 12.0
    miss_coast_acceleration_mps2: float = 3.0
    # Miss coasting includes braking (2026-08-05, 35 m/s trials). The previous miss-state command equaled
    # measured velocity, producing zero acceleration and straight flight. At 35 m/s, handing control back
    # with a minimum turning radius v^2/a = 245 m requires a 490 m U-turn to reposition. At 12 m/s, the
    # radius is 29 m, or 13 m at 8 m/s. Release speed therefore directly affects re-engagement time. A
    # zero command means full braking, which raises the nose and the fixed camera. Instead, retain the
    # current velocity direction and reduce its magnitude to 12 m/s at 3.0 m/s^2. This is 60% of
    # WPNAV_ACCEL 5 and corresponds to atan(3/9.81) = 17 deg tilt. Position-based guidance performs the
    # subsequent turn.
    miss_redis_key: str = ''
    # OPTIONAL direct broadcast of Redis (default OFF). The main mechanism is the 'drop' flag placed on
    # the Command object (see _miss_command); This key can be used as a bridge before changing the COMMON
    # FILE. It is NOT written to 'command_authority': bbox_to_redis CRUSHES that key with its own mode in
    # every frame (bbox_to_redis.py:472), that is, the value written by the controller is deleted in 33
    # ms.

    def __post_init__(self):
        if sum(self.blocks) != self.n_step:
            raise ValueError(f"sum of blocks {sum(self.blocks)} != "
                             f"n_step {self.n_step}")
        if not self.yaw_command_provide:
            # Let the ablation be consistent: If yaw is not to be commanded, the MODEL must also know that yaw is
            # NOT authorized, otherwise it will base the plan on a control that does not exist.
            self.yaw_speed_ceiling_dps = 0.0


# ====================================================== HELPER FUNCTIONS

def los_triad(ex_deg, eps_deg):
    """LOS triad in the heading frame: (l, e2, e3).

    Frame: x forward (nose, HORIZONTAL), y right, z down -- i.e. NED with only yaw applied. ex:
    horizontal bearing of the target relative to the nose (+right), eps: elevation of the target
    relative to the horizon (+up). l : along LOS (u1 positive = RANGE CLOSES) e2 : perpendicular to
    LOS, horizontally, to the right (u2 positive = ex DECREASES) e3 : perpendicular to LOS, DOWN in
    the vertical plane (u3 positive = we descend, the target RISES relative to us: eps INCREASES, ey
    = -eps DECREASES) The three vectors are orthonormal (mpc_test.py confirms).
    """
    ex = math.radians(ex_deg)
    eps = math.radians(eps_deg)
    ce, se = math.cos(eps), math.sin(eps)
    cx, sx = math.cos(ex), math.sin(ex)
    l = np.array([ce * cx, ce * sx, -se])
    e2 = np.array([-sx, cx, 0.0])
    e3 = np.array([se * cx, se * sx, ce])
    return l, e2, e3


def _projection_sphere_slice(U, v_ceiling, a_perpendicular, vz_alt, vz_upper, yaw_alt, yaw_upper):
    """FULL Euclidean projection (block by block, vector) onto the input constraint set.

    v_ceiling / vz_alt / vz_upper / yaw_ceiling is array PER BLOCK; Since preconditioning scales each block
    differently, the limits also vary per block. Preconditioning scales the velocity triplet by the
    SAME factor, so the sphere remains a sphere (not an ellipsoid) and the projection remains in
    closed form -- that's why we chose preconditioning to be block-uniform.

    Set (for each block): {|v|<=v_ceiling} intersection {vz_alt <= a.v <= vz_upper} multiplication
    {yaw_alt<=yaw<=yaw_upper}. Since a = NED-z line of plane leg LOS, |a|=1, slice projection is
    simple. The vertical slice and the yaw box come constrained by the HARD FOV constraint (see
    MpcSolver._dbf_limits): the framing constraint is thus entrusted not to the penalty but to the
    PROJECTION, i.e. it is ensured precisely.

    CLOSED FORM of projection to sphere+slice intersection: 1 ) p = projection to sphere; In my
    slice, that is the answer (the projection to C1 is valid since it remains in the set containing
    C1).  2 ) q = projection to slice; In the sphere, that is the answer.  3 ), the answer lies in
    the intersection circle of the two boundaries: x = c*a + sqrt (R^ 2 -c^ 2 ) * unit(y - (a.y) a)
    """
    Ur = U.reshape(-1, 4).copy()
    v = Ur[:, :3]
    yaw = Ur[:, 3]

    # 1 ) kure
    n = np.linalg.norm(v, axis=1)
    scale_value = np.where(n > v_ceiling, v_ceiling / np.maximum(n, 1e-12), 1.0)
    scale_value = np.minimum(scale_value, 1.0)
    p = v * scale_value[:, None]
    s_p = p @ a_perpendicular
    complete = (s_p >= vz_alt - 1e-9) & (s_p <= vz_upper + 1e-9)

    if not np.all(complete):
        # 2) and 3) for violating blocks
        s_y = v @ a_perpendicular
        c = np.clip(s_y, vz_alt, vz_upper)
        q = v - (s_y - c)[:, None] * a_perpendicular[None, :]
        n_q = np.linalg.norm(q, axis=1)
        on_sphere = n_q <= v_ceiling + 1e-9
        # 3) circle cozumu
        y_perpendicular = v - s_y[:, None] * a_perpendicular[None, :]
        n_perpendicular = np.linalg.norm(y_perpendicular, axis=1)
        safe = np.maximum(n_perpendicular, 1e-12)
        radius_item = np.sqrt(np.maximum(v_ceiling ** 2 - c ** 2, 0.0))
        circle = (c[:, None] * a_perpendicular[None, :]
                  + (radius_item / safe)[:, None] * y_perpendicular)
        selection = np.where(on_sphere[:, None], q, circle)
        p = np.where(complete[:, None], p, selection)

    Ur[:, :3] = p
    Ur[:, 3] = np.clip(yaw, yaw_alt, yaw_upper)
    return Ur.reshape(-1)


class DisturbanceEstimator:
    """d = (measured angle velocity) - (predicted by the model from our own velocity).

    Physical meaning: velocity component of the target perpendicular to LOS / range. So the ONLY
    target information the MPC needs -- and that information comes from the bbox (image), NOT the
    target telemetry. Contract visual_base releases this.
    """

    def __init__(self, config_value: MpcConfig):
        self.a = config_value
        self.reset_value()

    def reset_value(self):
        self.d_ex = 0.0
        self.d_ey = 0.0
        self.d_r = 0.0
        # SEPARATE, SLOWER copy for BOX CENTER (see update).
        self.d_ex_box = 0.0
        self.d_ey_box = 0.0
        self.confidence_value = 0.0
        self._t_total = 0.0
        self._n = 0
        self._ex_previous = None
        self._ey_previous = None
        self._r_previous = None
        # APN (see environment_apn): velocity of the target NORMAL to LOS and its derivative.
        # v_perpendicular = d_ex * r / KDEG  [m/s]   (d_ex'in fiziksel tersi)
        # a_perpendicular = d(v_perpendicular)/dt [m/s^2] (SEPARATE, slower LPF)
        self.v_perpendicular = 0.0
        self.a_perpendicular = 0.0
        self._v_perpendicular_previous = None

    def update_value(self, ex, ey, r_measurement, dt, c2, c3, w1, w2, w3, yaw_dps):
        """dt is the MEASURED step. If the derivative window is corrupted (bbox gap, dt bounce) the update is
SKIPPED, old d is frozen."""
        # CLOSE RANGE PROTECTION (2026-08-04 eng-3): clamp was KDEG*v_max/r via physics; r=6 equals 382 dps in
        # m. Immediately after handoff (r=4-9 m) d_ex measured: -136...+160 dps released and yaw command
        # JUMPED SIGN between rails +-90. At close range, angle variants are truly huge and loud; The
        # disturbance prediction there is MEANINGLESS. Solution: RETURN update (keep last healthy value) if r
        # < rotate_range. In the terminal phase the plan is already established, t_go < 1 p.
        near_value = (r_measurement is not None
                 and r_measurement < self.a.disturbance_rotate_range_m)
        valid_value = (self._ex_previous is not None and 0.01 < dt < 0.35
                   and not near_value)
        if valid_value:
            ex_speed = (ex - self._ex_previous) / dt
            ey_speed = (ey - self._ey_previous) / dt
            # model: ex_dot = -c2*w2 - yaw ; ey_dot = -c3*w3
            raw_ex = ex_speed - (-c2 * w2 - yaw_dps)
            raw_ey = ey_speed - (-c3 * w3)
            self._n += 1
            self._t_total += dt
            # FAST START: 1/n (running average) in the first samples, then normal LPF. d sits for a few cycles.
            k = max(1.0 / self._n, dt / (dt + self.a.disturbance_tau_s))
            self.confidence_value = 1.0 - math.exp(
                -self._t_total / max(1e-3, self.a.disturbance_confidence_s))
            self.d_ex += k * (raw_ex - self.d_ex)
            self.d_ey += k * (raw_ey - self.d_ey)
            # SEPARATE SLOW COPY -> BOX CENTER OF HARD CARD. Tur-3 chatter root cause: the same noisy d was
            # entering both the cost and the CENTER of the CBF box (double counting). Box width 26 deg was fixed
            # but the center was jumping as it was recalculated every cycle: yaw_alt_cbf step difference median
            # 0.30 while rms 3.95, p95 6.2 dps (heavy tail) -> yaw command +-16 dps chatter on 4 Hz. The cost
            # reference must remain FAST (response to the drive), but the COST LINES must remain slow and steady.
            k_box = max(1.0 / self._n,
                         dt / (dt + self.a.disturbance_box_tau_s))
            self.d_ex_box += k_box * (raw_ex - self.d_ex_box)
            self.d_ey_box += k_box * (raw_ey - self.d_ey_box)
            if (self.a.range_disturbance_source == "range_value"
                    and r_measurement is not None and self._r_previous is not None):
                # model: r_dot = -w1 + d_r -> d_r = r_dot_measurement + w1
                raw_r = (r_measurement - self._r_previous) / dt + w1
                self.d_r += k * (raw_r - self.d_r)
        self._ex_previous, self._ey_previous = ex, ey
        if r_measurement is not None:
            self._r_previous = r_measurement

        # physics clamp: |d| <= KDEG * v_target_max / range PLUS ABSOLUTE CEILING: the physics clamp alone
        # allowed 382 dps at close range (r=6 m) -- on a vehicle with a yaw ceiling 90 dps, this means pushing
        # the constraint to saturation with noise. The absolute ceiling is kept slightly above the yaw
        # authorization.
        r_g = max(self.a.range_floor_m,
                  r_measurement if r_measurement else self.a.range_if_absent_m)
        ceiling_value = min(KDEG * self.a.target_speed_ceiling_mps / r_g,
                    self.a.disturbance_absolute_ceiling_dps)
        self.d_ex = float(np.clip(self.d_ex, -ceiling_value, ceiling_value))
        self.d_ey = float(np.clip(self.d_ey, -ceiling_value, ceiling_value))
        self.d_ex_box = float(np.clip(self.d_ex_box, -ceiling_value, ceiling_value))
        self.d_ey_box = float(np.clip(self.d_ey_box, -ceiling_value, ceiling_value))
        self.d_r = float(np.clip(self.d_r, -self.a.target_speed_ceiling_mps,
                                 self.a.target_speed_ceiling_mps))

        # ---- APN: vertical SPEED and ACCELERATION of the target (see environment_apn) -------- v_perpendicular derived from
        # CLAMPED d_ex; Thus, d_ex and v_perpendicular remain two sides of the same information, and the clamp does not
        # separate in two places. Physical definition: d_ex = KDEG * v_perpendicular / r.
        self.v_perpendicular = self.d_ex * r_g / KDEG
        if valid_value:
            if self._v_perpendicular_previous is not None:
                raw_a = (self.v_perpendicular - self._v_perpendicular_previous) / dt
                # SEPARATE and SLOWER LPF. 1/n quick start NOT CONSCIOUSLY (only difference from the d_ex branch): the
                # first example of the derivative alone would drive the prediction to the clamp.
                k_a = dt / (dt + self.a.apn_tau_s)
                self.a_perpendicular += k_a * (raw_a - self.a_perpendicular)
                ceiling_a = self.a.apn_a_ceiling_mps2
                self.a_perpendicular = float(np.clip(self.a_perpendicular, -ceiling_a, ceiling_a))
            self._v_perpendicular_previous = self.v_perpendicular
        else:
            # CONTINUITY BREAKED (r < rotate_range or dt/bbox gap). a_perpendicular is kept at the LAST HEALTHY value --
            # the SAME freezing rule as d_ex. The previous example is DISCARDED: dividing a v_perpendicular difference
            # accumulated over the seconds into a single dt in the first frame of the freeze release would produce
            # a FAKE momentum jump at the ceiling.
            self._v_perpendicular_previous = None
        return self.d_ex, self.d_ey, self.d_r

    def apn_a_active(self) -> float:
        """Target lateral acceleration TO be GIVEN to law [m/s^2]. 0 if the handle is closed.

        Two doors (see environment_apn):
          (1) CIKARMALI DEAD BAND: a_active = sign(a)*max(|a|-db, 0).
              Noise floor measured on STRAIGHT leg ~0.3-0.5 m/s^2; The dead band reduces it to EXACT
              ZERO, so on a flat target the law remains the SAME as today. Subtractive (not hard)
              was chosen: hard threshold crossing would instantly produce a bounce, and that bounce
              would propagate directly to the horizon. (2) TRUST RAMP: The same ramp that multiplies
              the weight PN of d_ex (DisturbanceEstimator.trust, tau = disturbance_confidence_s). The prediction at
              the time of handoff is not yet established; a_perpendicular is the loudest term coming from
              there, so it stays under the same ramp.
        """
        if not self.a.apn:
            return 0.0
        db = self.a.apn_dead_band_mps2
        magnitude = max(abs(self.a_perpendicular) - db, 0.0)
        return (math.copysign(magnitude, self.a_perpendicular)
                * max(self.confidence_value, 0.0) * self.a.apn_multiplier)


# ============================================================= SOLVER

class MpcSolver:
    """Condensed LTV-QP + FISTA, callable independently of the live guidance loop (mpc_test.py calls this directly)."""

    NX = 6      # [ex, ey, r, w1, w2, w3]
    NU = 4      # [u1, u2, u3, yaw_dps]

    def __init__(self, config_value: MpcConfig = None):
        self.a = config_value or MpcConfig()
        a = self.a
        N, nb = a.n_step, len(a.blocks)
        self.N, self.nb = N, nb
        self.nu_top = self.NU * nb
        self.nx_top = self.NX * N

        # step -> block matching
        self.block_of = np.empty(N, dtype=int)
        i = 0
        for b, length in enumerate(a.blocks):
            self.block_of[i:i + length] = b
            i += length

        # --- cost row/column indexes (FIXED; only values ​​are written each cycle -> no allocation, fast) ---
        k = np.arange(N)
        self.w_column = np.concatenate([
            self.NX * k + 0,    # ex        (seviye)
            self.NX * k + 1,    # ey        (seviye)
            self.NX * k + 2,    # r         (seviye)
            self.NX * k + 4,    # w2 (inertial LOS speed, horizontal)
            self.NX * k + 5,    # w3 (inertial LOS speed, vertical)
            self.NX * k + 1,    # ey (VERTICAL ERROR: target line ref)
        ])
        # 6. line block: DIRECT VERTICAL ERROR (with env button, default OFF -> weight 0, i.e. enters M and b
        # as zero lines; output remains BIT-SAME). The reason why it is a separate block is the reference
        # DISC: 2. block (q_ey) references the LIVE gimbal axis for framing, this block references the “target
        # line” for impact (eps=0). See environment_vertical_error().
        self.n_row = 6 * N
        self.w_deg = np.zeros(self.n_row)
        self.w_ref = np.zeros(self.n_row)

        # FOV lines. HORIZONTAL row = ex_k (single column). VERTICAL row = beta_k = ey_k - kats*(
        # a_perpendicular . w_k ) + C, i.e. FOUR columns
        # (ey, w1, w2, w3); Since a_perpendicular changes in each cycle, Gf is constructed from gathers (there is no
        # multiplication).
        self.column_ex = self.NX * k + 0
        self.column_ey = self.NX * k + 1
        self.column_r = self.NX * k + 2
        self.column_w = np.stack([self.NX * k + 3, self.NX * k + 4,
                               self.NX * k + 5])          # (3, N)
        self.fov_alt = np.zeros(2 * N)
        self.fov_upper = np.zeros(2 * N)
        self.fov_rho = np.concatenate([np.full(N, a.rho_fov),
                                       np.full(N, a.rho_fov_vertical)])
        self.fov_active = self.fov_rho > 0.0

        # --- input level and difference penalties (FIXED) ---
        r_lvl = np.array([0.0,                                  # u1: NO penalty
                          a.r_speed / a.scale_speed_mps ** 2,
                          a.r_speed / a.scale_speed_mps ** 2,
                          a.r_yaw / a.scale_yaw_dps ** 2])
        self.R_run = np.tile(r_lvl, nb)
        sdu = np.array([a.r_delta_speed / a.scale_speed_mps ** 2,
                        a.r_delta_speed / a.scale_speed_mps ** 2,
                        a.r_delta_speed / a.scale_speed_mps ** 2,
                        a.r_delta_yaw / a.scale_yaw_dps ** 2])
        self.sdu = sdu
        D = np.zeros((self.nu_top, self.nu_top))
        for b in range(nb):
            s = self.NU * b
            D[s:s + self.NU, s:s + self.NU] = np.eye(self.NU)
            if b > 0:
                D[s:s + self.NU, s - self.NU:s] = -np.eye(self.NU)
        self.D = D
        Sd = np.tile(sdu, nb)
        self.DtSD = D.T @ (Sd[:, None] * D)
        self.Sd = Sd
        # YAW difference penalty now CHANGE PER CYCLE PRESS (gain scheduling, see yaw_delta_weight). To avoid
        # rebuilding the 28x28 matrix in each cycle, the penalty is decomposed into SPEED and YAW parts; At
        # runtime, only a scalar multiplication-add remains (negligible cost).
        sdu_v = sdu.copy(); sdu_v[3] = 0.0          # speed only
        sdu_y = np.array([0.0, 0.0, 0.0, 1.0])      # unit yaw
        self.Sd_v = np.tile(sdu_v, nb)
        self.Sd_y = np.tile(sdu_y, nb)
        self.DtSD_v = D.T @ (self.Sd_v[:, None] * D)
        self.DtSD_y = D.T @ (self.Sd_y[:, None] * D)
        self.prox_run = np.tile(
            np.array([a.lambda_prox / a.scale_speed_mps ** 2] * 3
                     + [a.lambda_prox / a.scale_yaw_dps ** 2]), nb)
        self.acceleration_run = np.tile(
            np.array([a.q_acceleration / a.scale_acceleration_mps ** 2] * 3 + [0.0]), nb)

        # unit equalization for stop criterion (equivalent to yaw deg/s -> m/s)
        self.tol_weight = np.tile(
            np.array([1.0, 1.0, 1.0, a.scale_speed_mps / a.scale_yaw_dps]), nb)

        # working buffers
        self.Gam = np.zeros((self.nx_top, self.nu_top))
        self.Xf = np.zeros(self.nx_top)
        self.last_duration_ms = 0.0
        self.last_iteration = 0
        # BUDGET CUT (2026-08-07, actual flight logging): did the solver stop converging or hit the iteration
        # ceiling / time budget? They can both produce the same 'iters' and 'duration_ms' numbers, so until now
        # it's been a QUIET degradation on the Pi: a non-converged QP solution gives commands that "seem to
        # work" but are not optimal. The flag makes this visible.
        self.last_budget_cut = 0
        self.last_cost = 0.0
        self.last_beta = 0.0
        self.last_yaw_delta = 0.0
        self.last_band_alt = self.a.fov_alt_band_deg
        self.last_band_upper = self.a.fov_upper_band_deg
        self.last_impact = 0.0
        self.last_v_ceiling = self.a.speed_ceiling_mps
        self.last_acceleration_multiplier = 1.0
        self.last_alignment_ref = 0.0
        # t_go shaped diagnosis tau (see environment_vertical_tgo). NaN remains on the CLOSED lever -- separating
        # "lever closed" from "t_go invalid" in CSV.
        self.last_tgo = float('nan')
        self.last_vertical_tau = float('nan')
        # APN: target lateral acceleration USED in law [m/s^2] (with dead band and confidence multiplier
        # APPLIED). Full 0 with handle closed. See environment_apn; diagnosis CSV column 'apn_a'.
        self.last_apn_a = 0.0
        # SATURATED actuator (see environment_actuator): Writes _nominal_orbit on every call. None = lever CLOSED ->
        # old fixed path tau.
        self._al_h = None
        self._al_v = None
        self._tau_v_block = None
        self.last_tau_eff = float('nan')
        self.last_tau_eff_z = float('nan')
        self.last_vertical_error = 0.0
        self.last_fov_free = 0
        self.empty_counter = 0
        self._released = False
        self._last_cbf = (0.0, 0.0, 0.0, 0.0)
        self.solution_counter = 0
        # heading Components of the FORWARD axis on the LOS plane; resolve() updates on every call, where the
        # safe default stops.
        self._forward_eks = np.array([1.0, 0.0, 0.0])

    @property
    def coefficient(self):
        """Climb -> pitch -> CAMERA AXIS coupling [deg/(m/s)].

        When the pitch_coupling is off (gimbal) it is ZERO: climbing does not shift the frame as
        the axis is stabilized independently of the body. It remains as a property so that there is
        no stale copy left when the settings object is changed AFTER installation."""
        return self.a.pitch_climb_coefficient if self.a.pitch_coupling else 0.0

    # ------------------------------------------------------ ic parcalar

    def _step_durations(self, dt0):
        h = np.full(self.N, self.a.step_s)
        h[0] = float(np.clip(dt0, 0.02, 0.30))
        # TO_TEST item 3: scale cost horizon WITH RANGE (0 = off). r is stored from x0[2] in solve(). It is
        # not scaled because the first step is MEASURED dt -- that step is actually the command to be
        # executed.
        ref = self.a.horizon_range_ref_m
        if ref > 0.0:
            r = getattr(self, '_last_range', None)
            if r is not None:
                floor_value = self.a.horizon_step_floor_s / max(self.a.step_s, 1e-6)
                h[1:] = self.a.step_s * float(np.clip(r / ref, floor_value, 1.0))
        return h

    def _apn_d_ex(self, d_base, rg, t_k, apn_a):
        """APN: Adds the target ACCELERATION term to the v_perpendicular CONSTANT spread.

        The basic (today) spread is d_ex_k = d_ex0 * r0/rbar_k, which exactly means
        KDEG*v_perpendicular0/rbar_k (assuming v_perpendicular CONSTANT). APN explains this in first order:
            d_ex_k = KDEG*(v_perpendicular0 + a_perpendicular*t_k) / rbar_k
                   = d_base_k  +  KDEG*a_perpendicular*t_k / rbar_k

        (Name note: apn_a = LATERAL ACCELERATION of the target [m/s^2]. In this file, 'a_perpendicular' in
        _cbf_limits is a SEPARATE thing -- there it is the downward UNIT VECTOR. To avoid confusion,
        it is called apn_a here.)

        *** THIS FUNCTION CAN BE CALLED ONLY WHEN apn_a != 0. *** In the closed branch it does NOT
        go into the caller arithmetic AT ALL -> the behavior is BIT-SAME.

        NOTE: enters FREE ANSWER only. The scale (input->state) and hence the Hessian are NOT
        affected by this term -- the solver workload and time remain the same.

        Clamp: TOTAL |d_ex_k| is kept below the physics ceiling (the same formula is applied for k=0
        in DisturbanceEstimator). At the end of the horizon (t_k ~ 2.3 s) 6 m/s^2 means a change of v_perpendicular
        by linear propagation 13.7 m/s; The ceiling cuts off the non-physical tail of this linear
        extrapolation.
        """
        a = self.a
        d = d_base + (KDEG * apn_a) * t_k / rg
        ceiling_value = np.minimum(KDEG * a.target_speed_ceiling_mps / rg,
                           a.disturbance_absolute_ceiling_dps)
        return np.clip(d, -ceiling_value, ceiling_value)

    def _nominal_trajectory(self, x0, U, h, d_ex, d_ey, d_r, cos_eps, r0,
                         apn_a=0.0):
        """FULL nominal state trajectory with warm-start input.

        If three is useful: (1) LTV the coefficients c = KVVA/r are frozen around this
        (linearization SQP). Since the range equation is already linear at the input, rbar is NOT
        APPROXIMATE, but exact. (2) Center of acceleration penalty wbar (nominal speed per block).
        (3) To establish the per-block bounds of the HARD FOV constraint (CBF), the nominal ex/ey/w
        at the block head is required.

        apn_a: ACCELERATION of the target perpendicular to LOS [m/s^2], 0 = OFF (default, legacy
        behavior BIT-SAME). See environment_apn / _apn_d_ex.
        """
        Ur = U.reshape(self.nb, self.NU)
        w = np.array(x0[3:6], dtype=float)
        ex, ey, r = float(x0[0]), float(x0[1]), float(x0[2])
        duration_block = 0.0
        N, nb, NU = self.N, self.nb, self.NU
        rbar = np.empty(N)
        wbar = np.zeros(self.nu_top)
        # case per block (for CBF): 0..4 ex, ey, w1, w2, w3 | 5..7 c2, c3, h | 8 forward acceleration
        bb = np.zeros((nb, 9))
        forward = self._forward_eks           # heading-x components in the LOS triad
        w_block_start = None                 # components
        observed = -1
        tau = self.a.speed_latency_tau_s
        floor_value = self.a.range_floor_m
        t_k = 0.0                         # horizon time AT STEP BEGINNING (APN)
        # SATATIFIED ACTIVE (see environment_actuator). PER-STEP active tau is here calculated on the NOMINAL orbit
        # and transferred EXACTLY to the _orbit_ matrices (self._al_h) -- the two functions MUST use the SAME
        # linearization point, otherwise Xf and Gam will diverge and QP will be inconsistent.
        saturation = self.a.actuator
        tau_lin = self.a.actuator_tau_lin_s
        a_max = max(self.a.actuator_a_max_mps2, 1e-3)
        a_max_z = max(self.a.actuator_a_max_z_mps2, 1e-3)
        tau_lin_z = self.a.actuator_tau_lin_z_s
        al_h = np.empty(N) if saturation else None
        al_v = np.empty(N) if saturation else None
        tau_v_block = (np.full(nb, self.a.actuator_tau_lin_z_s)
                      if saturation else None)
        for k in range(N):
            rbar[k] = r
            hk = h[k]
            b = self.block_of[k]
            rg = max(r, floor_value)
            c2 = KDEG / (rg * cos_eps)
            c3 = KDEG / rg
            scale_value = r0 / rg
            # APN: v_perpendicular0 + a_perpendicular*t_k instead of constant v_perpendicular (see _apn_d_ex). When apn_a = 0 this branch NEVER
            # runs -> closed branch BIT-SAME.
            d_ex_k = (d_ex * scale_value if apn_a == 0.0 else
                      float(self._apn_d_ex(d_ex * scale_value, rg, t_k, apn_a)))
            if b != observed:
                if observed >= 0 and w_block_start is not None:
                    # average forward acceleration of the previous blog
                    bb[observed, 8] = ((w - w_block_start) @ forward) / max(
                        duration_block, 1e-3)
                wbar[NU * b:NU * b + 3] = w
                bb[b, :8] = (ex, ey, w[0], w[1], w[2], c2, c3, hk)
                w_block_start = w.copy()
                duration_block = 0.0
                observed = b
            duration_block += hk
            ex = ex + hk * (-c2 * w[1] + d_ex_k)
            ey = ey + hk * (-c3 * w[2] + d_ey * scale_value)
            r = r + hk * (-w[0] + d_r)
            al = hk / (hk + tau) if tau > 1e-6 else 1.0
            if saturation:
                # HORIZONTAL error magnitude (u1 = length LOS, u2 = vertical horizontal). tau_eff = max(tau_lin,
                # |e_horizontal|/a_max): al*|e|/h = a_max in the saturation region, i.e. FULL acceleration ceiling.
                eh = math.hypot(Ur[b, 0] - w[0], Ur[b, 1] - w[1])
                alh = hk / (hk + max(tau_lin, eh / a_max))
                al_h[k] = alh
                w0 = w[0] + alh * (Ur[b, 0] - w[0])
                w1n = w[1] + alh * (Ur[b, 1] - w[1])
                # VERTICAL (u3): linear base OLD tau, on ACCELERATION LIMIT. In the first release there was NO limit
                # and demand was flowing into this channel -- see. actuator_a_max_z_mps2.
                ev = abs(Ur[b, 2] - w[2])
                tau_vk = max(tau_lin_z, ev / a_max_z)
                alv = hk / (hk + tau_vk)
                al_v[k] = alv
                if k == 0 or self.block_of[k - 1] != b:
                    tau_v_block[b] = tau_vk
                w2n = w[2] + alv * (Ur[b, 2] - w[2])
                w = np.array([w0, w1n, w2n])
            else:
                w = w + al * (Ur[b, :3] - w)
            t_k += hk
        if observed >= 0 and w_block_start is not None:
            bb[observed, 8] = ((w - w_block_start) @ forward) / max(duration_block, 1e-3)
        # RETURN ARTICLE HAS NOT CHANGED (mpc_test.py opens the value 3 positionally); step-by-step is carried
        # through the SIDE CHANNEL.
        self._al_h = al_h
        self._al_v = al_v
        self._tau_v_block = tau_v_block
        self.last_tau_eff = (float('nan') if al_h is None else
                            float(h[0] * (1.0 - al_h[0]) / max(al_h[0], 1e-9)))
        self.last_tau_eff_z = (float('nan') if al_v is None else
                              float(h[0] * (1.0 - al_v[0]) / max(al_v[0], 1e-9)))
        return np.maximum(rbar, floor_value), wbar, bb

    def _cbf_boundaries(self, bb, a_perpendicular, beta_c, d_ex, d_ey, r0, rbar,
                       impact_value=0.0, tau_v=None):
        """HARD FOV kisitini INPUT KUTUSUNA/DILIMINE cevirir.

        Framing variables one step later are AFFINE in the input: beta_ {k+1} = B0 - kats*al*(
        a_perpendicular . u) -> dusey SLICE
            ex_{k+1}   = E0 - h*omega                 -> yaw KUTUSU
        Discrete CBF: h = limit - beta >= 0 for h_{k+1} >= (1-g) h_k i.e. beta_{k+1} <= g*limit +
        (1-g)*beta_k. When there is a violation, it says "cure", not "immediate cure" -> NEVER
        infeasible.

        If the cluster still comes up empty (physical speed/yaw ceiling is not enough), BEST EFFORT
        is selected: BOTTOM edge takes priority, because all measured losses occurred from the
        bottom.
        """
        a = self.a
        nb = self.nb
        tau = a.speed_latency_tau_s
        ex0, ey0 = bb[:, 0], bb[:, 1]
        w1, w2, w3 = bb[:, 2], bb[:, 3], bb[:, 4]
        c2, c3, hk = bb[:, 5], bb[:, 6], bb[:, 7]
        scale_value = r0 / np.maximum(rbar[0], a.range_floor_m)
        # PREDICTION HORIZON: constraint cannot be written over ONE loop step (0.05 s). Since the actuator is
        # tau=1.0 s, the alignment effect of the input in one step is al = h/(h+tau) = 0.048; Dividing the
        # constraint by this gain turns the model error of 1 degrees into a vertical velocity command of 6.6
        # m/s -- measured: the constraint goes bang-bang, the hopper lurches vertically, and the frame is lost
        # more with the hard constraint ON. The horizon is chosen to be of the same order as the actuator time
        # constant (0.8 s -> al_T = 0.44), so that the input sensitivity of the constraint is physically
        # meaningful. SCALE FORECAST HORIZON WITH t_go (2026-08-04 tur-3 chatter): At close range, two things
        # break down: (1) c = KDEG/r explodes, (2) constant 1.0 s horizon PAST THE MOMENT OF COLLISION, so the
        # constraint predicts a future that will no longer exist. Together they were swinging the box center:
        # |dYaw| According to the range band of rms, 0.67 dps was measured at 35-60 m while 7.37 dps was
        # measured at 12-20 m. Limiting the horizon to a fraction of t_go is what physics says: planning ahead
        # of the collision. The SENSITIVITY of the box center to input scales with c2*T and c2 = KVV/r. With
        # constant T the sensitivity explodes like 1/r. Choosing T PROPORTIONAL TO RANGE keeps the c2*T
        # product constant, meaning the drift of the box center is INDEPENDENT of range. (I first tried
        # scaling with t_go: since the closing speed w1 is small in the diagonal geometry, t_go turns out to
        # be large and scaling was not effective AT ALL -- measured, ineffective.)
        T = float(np.clip(a.cbf_prediction_s * rbar[0] / a.cbf_range_ref_m,
                          a.cbf_prediction_min_s, a.cbf_prediction_s))
        # VERTICAL SATURATION CONSISTENCY (2026-08-08 night-2): `al` below is the ONLY actuator gain in this
        # function and belongs to the VERTICAL slice. When the acceleration limit is added to the vertical
        # channel (see MpcConfig.actuator_a_max_z_mps2) the constraint must see the SAME gain, otherwise the
        # constraint assumes "I am fast" when the plan says "I am slow" and the slice requests a vz that is
        # not actually achievable. BLOCK HEAD tau_v is the vertical array tau_eff; None = lever closed -> old
        # constant tau, i.e. BIT-SAME.
        if tau_v is None:
            al = T / (T + tau) if tau > 1e-6 else 1.0     # BIT-SAME branch
        else:
            al = T / (T + np.maximum(tau_v, 1e-6))        # per block array

        # --- DUSEY: beta = ey - kats*vz + C ---
        coefficient = self.coefficient
        vz_now = w1 * a_perpendicular[0] + w2 * a_perpendicular[1] + w3 * a_perpendicular[2]
        beta_now = ey0 - coefficient * vz_now + beta_c
        # TWO CHANNELS: (1) PITCH coupling (factors; ZERO on gimbal), (2) PURE GEOMETRY -- if you go lower,
        # the apparent elevation of the target increases, so ey drops. Channel gain:
        #         w3 = (vz + w1*sin eps)/cos eps  ->  dw3/dvz = 1/cos eps
        #         d(ey1)/ds = -T*c3*al/ cos eps = -T*c2*al (c2 = c3/ cos eps already exists in the table per
        #         block.) The geometric channel WAS PREVIOUSLY NEGLECTED; In the + 30 mount it didn't matter
        #         because the pitch channel ( 3.2 ) was dominant, but in the gimbal (kats= 0 ) IT IS the only
        #         channel. Size: T selected, T*c2 ~ 1.3 deg /( m/s ) is constant, i.e. ~% 40 of channel pitch
        #         .
        coefficient_geom = T * c2                        # = T*c3/cos_eps
        ey1 = ey0 + T * (-c3 * w3 + d_ey * scale_value)
        B0 = (ey1 + coefficient_geom * al * vz_now
              - coefficient * (1.0 - al) * vz_now + beta_c)
        coefficient_value = (coefficient + coefficient_geom) * al         # beta1 = B0 - coefficient*s
        coefficient_value = np.maximum(coefficient_value, 1e-6)
        # BRAKE -> NOSE UP: the hopper accelerates while leaning, pitch ~ -atan(a_forward/g) i.e. every 1 m/s^2
        # BRAKE nose KDEV/g = 5.84 deg lifts. This is much larger than the climbing term (3.2 deg per m/s) and
        # is the main culprit of the measured framing losses. Since the normal of the constraint is NOT
        # PARALLEL to a_perpendicular, it cannot be written into the slice; instead, the share of the PLANNED brake will
        # be deducted from the tape (conservative narrowing, the projection remains in closed form). CAUTION
        # (second crash lesson 2026-08-04): the planned brake ABSOLUTELY cannot be used here. beta_c carries
        # pitch already MEASURED and that pitch CONTAINS the existing brake; Dropping the absolute brake again
        # would be DOUBLE COUNTING. We saw the result by measurement: Since there was always some deceleration
        # in the MPC plan, the belt constantly collapsed to its base (4 deg), beta ~9-12 was considered a
        # permanent "violation" and the constraint demanded a CONTINUOUS descent -- 2 Even with a ceiling of
        # m/s, 15 means altitude in s, 30 means space. Frankly, it's the same logic as the term escalation:
        # CHANGE based on the PRESENT situation. In block 0 the contraction is zero (we have already measured
        # pitch). GIMBAL: When pitch_coupling is closed, even though the brake raises the nose, the CAMERA
        # AXIS DOES NOT MOVE -> no narrowing is done. STROKE PHASE: the bands open to the PHYSICAL EDGE and
        # the braking/acceleration throttle is released (see block MpcConfig stroke_*). Below in s=0 is exactly
        # the same old behavior.
        s_v = float(np.clip(impact_value, 0.0, 1.0))
        band_alt_nom = (a.fov_alt_band_deg
                        + s_v * (a.impact_band_deg - a.fov_alt_band_deg))
        band_upper_nom = (a.fov_upper_band_deg
                        + s_v * (a.impact_band_deg - a.fov_upper_band_deg))
        if a.pitch_coupling:
            # BRAKE -> nose UP -> target LOWER edge (old term).
            braking = np.maximum(0.0, -(bb[:, 8] - bb[0, 8]))
            # ACCELERATION -> nose DOWN -> target UP edge. Added in round 35 m/s: mount In 0 the target is already
            # ABOVE the axis and the first seconds of engagement are constant forward acceleration.
            acceleration_value = np.maximum(0.0, bb[:, 8] - bb[0, 8])
            band_alt = np.maximum(
                band_alt_nom - (1.0 - s_v) * (KDEG / 9.80665) * braking,
                a.fov_floor_band_deg)
            band_upper = np.maximum(
                band_upper_nom - (1.0 - s_v) * (KDEG / 9.80665) * acceleration_value,
                a.fov_upper_floor_band_deg)
        else:
            band_alt = np.full(nb, band_alt_nom)
            band_upper = np.full(nb, band_upper_nom)
        g = a.cbf_gamma
        target_alt_value = g * band_alt + (1 - g) * beta_now
        target_upper = -g * band_upper + (1 - g) * beta_now
        # beta1 <= target_alt_value -> s >= (B0 - target_alt_value)/coefficient
        s_lo = (B0 - target_alt_value) / coefficient_value
        # beta1 >= target_upper -> s <= (B0 - target_upper)/coefficient
        s_hi = (B0 - target_upper) / coefficient_value
        # === SAFETY 1: THE LOWERING THAT THE PERSON CAN REQUEST IS LIMITED === 2026-08-04 DROP REGRESSION:
        # here s_lo was clipping directly onto the physical box. When the cluster was empty s_lo was BIG
        # POSITIVE, clipping was based on +descent_ceiling (+4.5 m/s) each time and the box was COOLING to
        # [4.5, 4.5] -> vertical command to MAXIMUM LOWER NAILED IT. In all three routes, the hopper hit the
        # GROUND, not the target (ellipse: pos_z -45.8 -> +2.7, target 128 m away). The residual constraint
        # MAY ask to be lowered by at most fov_descent_demand_ceiling; Beyond that it is already unrecoverable
        # geometry and the following timeout comes into play.
        s_lo_raw = s_lo
        s_lo = np.minimum(s_lo, a.fov_descent_demand_ceiling_mps)
        s_hi = np.maximum(s_hi, -a.fov_climb_demand_ceiling_mps)
        vz_alt = np.maximum(-a.climb_ceiling_mps, s_lo)
        vz_upper = np.minimum(a.descent_ceiling_mps, s_hi)
        # === SAFETY 2: EMPTY CLUSTER -> MIDPOINT (NO nailing) === Chebyshev cross BALANCES two one-sided
        # violations; It cannot be nailed to one side like before.
        empty_value = vz_alt > vz_upper
        if np.any(empty_value):
            mean = np.clip(0.5 * (vz_alt + vz_upper),
                          -a.climb_ceiling_mps, a.descent_ceiling_mps)
            vz_alt = np.where(empty_value, mean, vz_alt)
            vz_upper = np.where(empty_value, mean, vz_upper)
        # TIMEOUT INPUT. Two situations count: (a) the cluster is empty / the demand ceiling is exceeded
        # (constraint cannot be met),
        #  (b) the constraint prohibits even LEVEL FLIGHT (lower limit > 0), i.e. imposes uninterrupted
        #  descent. Condition (b) is critical: a constraint that fits EXACTLY to the demand ceiling appears
        #  "affordable" but orders descent for hours and grounds the vehicle. If 2 s uninterrupted descent is
        #  required for framing, the frame cannot be saved anyway.
        saturated = bool(empty_value[0] or s_lo_raw[0] > a.fov_descent_demand_ceiling_mps
                      or vz_alt[0] > a.constraint_descent_threshold_mps)

        # --- HORIZONTAL: ex1 = E0 - h*omega ---
        E0 = ex0 + T * (-c2 * w2 + d_ex * scale_value)
        lim = a.ex_limit_deg + s_v * (a.impact_ex_limit_deg - a.ex_limit_deg)
        target_u = g * lim + (1 - g) * ex0
        target_l = -g * lim + (1 - g) * ex0
        w_lo = (E0 - target_u) / T
        w_hi = (E0 - target_l) / T
        yaw_alt = np.maximum(-a.yaw_speed_ceiling_dps, w_lo)
        yaw_upper = np.minimum(a.yaw_speed_ceiling_dps, w_hi)
        empty_value = yaw_alt > yaw_upper
        if np.any(empty_value):
            mean = np.clip(0.5 * (w_lo + w_hi), -a.yaw_speed_ceiling_dps,
                          a.yaw_speed_ceiling_dps)
            yaw_alt = np.where(empty_value, mean, yaw_alt)
            yaw_upper = np.where(empty_value, mean, yaw_upper)
        return (vz_alt, vz_upper, yaw_alt, yaw_upper, beta_now,
                float(band_alt[0]), float(band_upper[0]), saturated)

    def _trajectory_matrices(self, x0, h, rbar, cos_eps, d_ex0, d_ey0, d_r,
                            r0, apn_a=0.0, al_h=None, al_v=None):
        """Condensation of Xf (free response) and Gam (input -> state).

        Matrix A is sparse and structured; We use 4 vector operation instead of 6x6 multiplication
        (Pi makes a difference in 5).

        apn_a: ACCELERATION of the target perpendicular to LOS [m/s^2], 0 = OFF (default, legacy
        behavior BIT-SAME). It enters Xf (free response) ONLY; It is already independent of the
        descaler, so the Hessian and solver cost do not change. See environment_apn / _apn_d_ex.

        al_h: PER-STEP horizontal gain of the SATURATED actuator (length N) or None = OFF (default,
        old behavior BIT-SAME). Generates the array _nominal_orbit; The two functions must share the
        SAME linearization point. It is applied to HORIZONTAL channels (w1,w2 / u1,u2). al_v is the
        VERTICAL channel (w3/u3) equivalent of the same thing; None = old fixed tau in portrait. See
        environment_actuator() and MpcConfig.actuator_a_max_z_mps2."""
        N, NU, NX = self.N, self.NU, self.NX
        tau = self.a.speed_latency_tau_s
        Gam = self.Gam
        Gam.fill(0.0)
        Xf = self.Xf
        f = np.array(x0, dtype=float)
        G = np.zeros((NX, self.nu_top))
        c2 = KDEG / (rbar * max(cos_eps, 0.2))
        c3 = KDEG / rbar
        # scaled by disruptive range: d = KDEG * v_perpendicular / r
        scale_d = r0 / rbar
        d_ex = d_ex0 * scale_d
        d_ey = d_ey0 * scale_d
        if apn_a != 0.0:
            # horizon time AT STEP BEGINNING (t_0 = 0). In Euler forward integration k. The disruptor used in a
            # step is the value at the BEGINNING of that step -- same convention as rbar[k].
            t_step = np.concatenate(([0.0], np.cumsum(h)[:-1]))
            d_ex = self._apn_d_ex(d_ex, rbar, t_step, apn_a)

        for k in range(N):
            hk = h[k]
            al = hk / (hk + tau) if tau > 1e-6 else 1.0
            column_value = NU * self.block_of[k]

            # --- G (status -> input sensitivity) ---
            G[0] -= (hk * c2[k]) * G[4]
            G[1] -= (hk * c3[k]) * G[5]
            G[2] -= hk * G[3]
            if al_h is None:
                G[3:6] *= (1.0 - al)
                G[0, column_value + 3] -= hk      # yaw speed directly reduces ex
                G[3, column_value + 0] += al
                G[4, column_value + 1] += al
                G[5, column_value + 2] += al
            else:
                alh = al_h[k]            # HORIZONTAL: saturated gain
                alv = al if al_v is None else al_v[k]   # VERTICAL
                G[3] *= (1.0 - alh)
                G[4] *= (1.0 - alh)
                G[5] *= (1.0 - alv)
                G[0, column_value + 3] -= hk
                G[3, column_value + 0] += alh
                G[4, column_value + 1] += alh
                G[5, column_value + 2] += alv

            # --- f (free answer) ---
            f[0] += hk * (-c2[k] * f[4] + d_ex[k])
            f[1] += hk * (-c3[k] * f[5] + d_ey[k])
            f[2] += hk * (-f[3] + d_r)
            if al_h is None:
                f[3:6] *= (1.0 - al)
            else:
                f[3] *= (1.0 - al_h[k])
                f[4] *= (1.0 - al_h[k])
                f[5] *= (1.0 - (al if al_v is None else al_v[k]))

            Gam[NX * k:NX * (k + 1), :] = G
            Xf[NX * k:NX * (k + 1)] = f

        return Xf, Gam, c2, c3, d_ex, d_ey

    def _cost_rows(self, c2, c3, d_ex, d_ey, ey_ref, confidence_value=1.0,
                           impact_value=0.0, eps_deg=0.0, vertical_s=0.0,
                           vertical_error_s=0.0, vertical_tau_s=None):
        a = self.a
        N = self.N
        multiply = np.ones(N)
        multiply[-1] = a.p_multiplier
        s = np.sqrt(multiply)
        # STROKE: framing weights UP (locking to center bbox), the boundaries of the lines FOV open to the
        # physical edge.
        s_v = float(np.clip(impact_value, 0.0, 1.0))
        multiply_ex = 1.0 + s_v * (a.impact_ex_multiplier - 1.0)
        multiply_ey = 1.0 + s_v * (a.impact_ey_multiplier - 1.0)
        wq_ex = math.sqrt(a.q_ex * multiply_ex) / a.scale_ex_deg * s
        wq_ey = math.sqrt(a.q_ey * multiply_ey) / a.scale_ey_deg * s
        wq_r = math.sqrt(a.q_range) / a.scale_range_m * s
        # The term PN is based on disturbance estimation; This weight is personal while d is unreliable (see
        # MpcConfig.disturbance_confidence_s).
        multiply_los = 1.0 + s_v * (a.impact_los_multiplier - 1.0)
        wq_s = math.sqrt(a.q_los_speed * multiply_los * max(confidence_value, 0.0)) \
            / a.scale_los_speed_dps * s

        self.w_deg[0 * N:1 * N] = wq_ex
        self.w_deg[1 * N:2 * N] = wq_ey
        self.w_deg[2 * N:3 * N] = wq_r
        self.w_deg[3 * N:4 * N] = -wq_s * c2
        self.w_deg[4 * N:5 * N] = -wq_s * c3

        self.w_ref[0 * N:1 * N] = 0.0
        self.w_ref[1 * N:2 * N] = wq_ey * ey_ref
        self.w_ref[2 * N:3 * N] = 0.0          # Zref = 0 -> collision
        # residual = -wq_s*c2*w2 + wq_s*d_ex => ref = -wq_s*d_ex
        self.w_ref[3 * N:4 * N] = -wq_s * d_ex
        # VERTICAL LOS SPEED REFERENCE: nominally sigma_el -> 0 (parallel travel = vertical standoff
        # maintained). Climbing bias is added to the line in STROKE (see MpcConfig .stroke_align_*): sigma_el
        # -> sigma_el_ref = stroke * clip( eps / tau , ceiling), while eps > 0. The reference becomes w_ref =
        # - wq_s *( d_ey - sigma_el_ref ); because the term lasts (-q3*w3 + d_ey ) -> sigma_el_ref .
        sigma_el_ref = 0.0
        if a.impact_alignment_closing and s_v > 0.0 and eps_deg > a.impact_alignment_relaxed_deg:
            sigma_el_ref = s_v * min(
                (eps_deg - a.impact_alignment_relaxed_deg) / max(a.impact_alignment_tau_s, 1e-3),
                a.impact_alignment_ceiling_dps)
        # ALTITUDE-AGNOSTIC TERMINAL VERTICAL ALIGNMENT (with env push button, default OFF; see
        # environment_vertical_terminal and MpcConfig.vertical_terminal). vertical_s is the range ramp (solve() calculates; 0
        # if bbox is stale). BOTH SIDED: eps>0 (target above us) -> POSITIVE sigma_el here (climb), eps<0
        # (target below us) -> NEGATIVE here (descend). Since eps_dot = -sigma_el, both arms have |eps|
        # becomes smaller. t_go SHAPE TAU (with env button, default ON; see environment_vertical_tgo and
        # MpcConfig.vertical_tgo). vertical_tau_s is calculated by solve(): If None, the OLD constant tau is used,
        # meaning this line remains BIT-SAME when TGO is off. LEVER SELECTION: the line is set with
        # vertical_terminal OR vertical_tgo on -- so that the main arm (P + TGO) can use this speed reference
        # WITHOUT turning vertical_terminal ON.
        if (a.vertical_terminal or a.vertical_tgo) and vertical_s > 0.0:
            dead = a.vertical_terminal_relaxed_deg
            if eps_deg > dead:
                excess = eps_deg - dead
            elif eps_deg < -dead:
                excess = eps_deg + dead
            else:
                excess = 0.0
            tau_d = (a.vertical_terminal_tau_s if vertical_tau_s is None
                     else float(vertical_tau_s))
            # CEILING. On the old (fixed tau) arm it is a FIXED angular ceiling (8 dps). In the TGO arm it is a
            # RANGE-CONSISTENT SPEED ceiling and this is a fix, not a relaxation: sigma_el [dps] -> vertical speed
            # = r * rad(fit) = fit / c3 i.e. CONSTANT angular ceiling changes its meaning with range: 8 dps 45 in
            # m 6.3 m/s (EXCEEDS the ceiling of the Descent box 4.5, so the reference asks for something that
            # cannot be delivered) but only 2.1 m/s in 15 m (SHOCKS IT exactly where the law t_go should work).
            # Measured (solver-loop scan): eps0 >= 8 Due to the fixed ceiling at deg the demand is clipped around
            # 25 m, the clipped demand means a FIXED eps_dot and the constant eps_dot is the EXCESS -- vertical
            # speed even at k=3.0 @CPA -5.4 m/s was staying. In the TGO arm, the ceiling is directly the INPUT BOX
            # itself (ascent 9 / descent 4.5 m/s), selected by direction. So the reference never wants MORE than
            # the box can deliver -- TIGHTER than the old ceiling in the 45 m, wider in the 25 m, PHYSICAL all
            # around.
            if vertical_tau_s is None:
                ceiling_d = a.vertical_terminal_ceiling_dps
            else:
                v_ceiling = (a.climb_ceiling_mps if excess > 0.0
                         else a.descent_ceiling_mps)
                ceiling_d = float(c3[0]) * v_ceiling
            ref_d = float(np.clip(
                vertical_s * excess / max(tau_d, 1e-3), -ceiling_d, ceiling_d))
            # If the retiree opens with the HIT bias (ablation), the BIGGER ONE wins; Don't let the two add up and
            # produce twice the demand.
            if abs(ref_d) > abs(sigma_el_ref):
                sigma_el_ref = ref_d
        self.last_alignment_ref = float(sigma_el_ref)
        self.w_ref[4 * N:5 * N] = -wq_s * (d_ey - sigma_el_ref)

        # DIRECT VERTICAL ERROR (P) LINE -- with env button, default ON (see environment_vertical_error and
        # MpcConfig.vertical_error). GEOMETRY: in command() eps = -(ey + aim) IS DEFINITION, i.e. the "target line"
        # (eps = eps_hed) is a FIXED reference ey in the case of MPC: ey_hed = -(eps_hed + am). NO additional
        # predictions. DEAD TAPE: |eps| While <= relaxed WEIGHT is zero -> the line enters M/b as zero, the
        # solution is bit-same as the CLOSED arm. Outside the band, the target is the EDGE of the band
        # (eps_hed = +-relax); so at the threshold it now starts from ~0 and the demand is CONTINUOUS (no
        # jumps). IT DOES NOT SCAL with STRIKE mix: the goal is to spread the alignment over the window of ~3
        # s given by the range ramp, not the last second.
        dead = a.vertical_error_relaxed_deg
        enabled_value = (a.vertical_error and vertical_error_s > 0.0
                and abs(eps_deg) > dead)
        if enabled_value:
            eps_hed = dead if eps_deg > 0.0 else -dead
            ey_hed = -(eps_hed + a.aim_deg)
            wq_perpendicular = (math.sqrt(a.q_vertical_error * a.vertical_error_multiplier
                                * vertical_error_s) / a.scale_ey_deg) * s
            self.w_deg[5 * N:6 * N] = wq_perpendicular
            self.w_ref[5 * N:6 * N] = wq_perpendicular * ey_hed
            # Diagnosis: vertical angle that the solver wants to cover (deg). The sign is the same as eps -> "how
            # many degrees up/down".
            self.last_vertical_error = float(eps_deg - eps_hed)
        else:
            self.w_deg[5 * N:6 * N] = 0.0
            self.w_ref[5 * N:6 * N] = 0.0
            self.last_vertical_error = 0.0

        ex_lim = a.ex_limit_deg + s_v * (a.impact_ex_limit_deg
                                          - a.ex_limit_deg)
        self.fov_alt[:N] = -ex_lim
        self.fov_upper[:N] = ex_lim
        # vertical lines are now on beta = ey - kats*vz + C
        self.fov_alt[N:] = -(a.fov_upper_band_deg
                             + s_v * (a.impact_band_deg - a.fov_upper_band_deg))
        self.fov_upper[N:] = (a.fov_alt_band_deg
                            + s_v * (a.impact_band_deg - a.fov_alt_band_deg))

    def _area_reward(self, rbar, r0, impact_value=0.0):
        """LINEARIZED gradient of the LINEAR bbox field reward.

        Since A ~ K/r^2, the relative area is a_k = (r0/rbar_k)^2 and the growth rate is also_k/dt = 2
        a_k w1_k / rbar_k. Both are linearized around rbar; ONLY LINEAR term enters the cost, that
        is, Hessian and solution time do not change.

        Returned: (c_r, c_w1) -- linear cost coefficients on the stacked case (J += c_r . r + c_w1 .
        w1).
        """
        a = self.a
        N = self.N
        rg = np.maximum(rbar, a.range_floor_m)
        relative = (r0 / rg) ** 2                    # A_k / A_0
        multiply = np.ones(N)
        multiply[-1] = a.p_multiplier
        # IMPACT: reward (closing incentive) is magnified.
        reward_multiply = 1.0 + float(np.clip(impact_value, 0.0, 1.0)) * (
            a.impact_reward_multiplier - 1.0)
        # A/B button ( TO_TEST item 1 ): default 1.0 -> no-op.
        reward_multiply *= a.q_area_multiplier
        # d(-q*a_k)/d r_k = +2 q a_k / rbar_k
        c_r = (2.0 * a.q_area * reward_multiply / N) * relative / rg * multiply
        # d(-q*also_k/dt)/d w1_k = -2 q a_k / rbar_k (1 s time scale)
        c_w1 = (-2.0 * a.q_area_rate * reward_multiply / N) * relative / rg * multiply
        return c_r, c_w1

    # --------------------------------------------------------------- coz

    def solve_value(self, x0, d_ex, d_ey, d_r, eps_deg, ey_ref, beta_c,
            dt0, U_warm=None, u_previous=None, confidence_value=1.0, bbox_age_s=0.0,
            altitude_m=None, d_ex_box=None, d_ey_box=None,
            yaw_delta_weight=None, impact_value=0.0, acceleration_multiplier=None,
            t_go_s=None, apn_a=0.0):
        """A MPC solves the step.

        x0 : [ ex_deg , ey_deg , r_m , w1, w2, w3] (w: our own speed MEASURED at the plane LOS ) d_*
        : disturbance estimates ( deg /s, deg /s, m/s ) eps_deg : elevation of the target relative
        to the horizon (for plane LOS ) ey_ref : framing center (= -( mount + pitch +aim)) beta_c :
        framing variable constant;  beta_k = ey_k - kats* vz_k + beta_c (see MpcController
        ._framing_constant) bbox_age_s : time since last valid detection. If stale_constraint_s is
        hung, the HARD framing constraint will be RELEASED (the constraint will reinforce itself as
        ey will have rotated) hit : STROKE phase mixing coefficient [ 0 .. 1 ] (see MpcConfig hit_*
        block).  0 = today's cost as is, 1 = pure capture (bands on physical edge, framing and
        reward weights up , acceleration penalty down ) t_go_s : TIME REMAINING estimate [s] or None
        . It comes from range_rate_value which the controller ALREADY calculated (see
        environment_vertical_tgo ); Used on vertical_tgo arm only.  None = invalid -> softest
        branch.  apn_a : ACCELERATION OF the target perpendicular to LOS [ m/s ^ 2 ] or 0 (OFF,
        default -- old behavior BIT-SAME). The controller gets this from DisturbanceEstimator.
        apn_a_active () (deadband + confidence multiplier + clamp applied THERE It is possible). See
        environment_apn / _apn_d_ex. dt0 : MEASURED time of first step Returns: (U (nb x NU), info
        dict)
        """
        t_start_value = time.perf_counter()
        a = self.a
        nu_top = self.nu_top
        # _step_times for range scaling (TO_TEST item 3) gets the current range from here; x0[2] = r_m.
        self._last_range = float(np.asarray(x0, dtype=float)[2])
        self.solution_counter += 1
        # Do not carry the old value from the branch where _fista is not called (analytical/gradient
        # solution): it is reset in each solution, _fista writes its own result.
        self.last_budget_cut = 0
        cold = self.solution_counter <= a.initial_solution_count
        it_ceiling = a.initial_iteration_ceiling if cold else a.iteration_ceiling
        budget_s_value = (a.initial_budget_ms if cold else a.duration_budget_ms) / 1000.0

        if U_warm is None:
            U = np.zeros(nu_top)
        else:
            U = np.array(U_warm, dtype=float).reshape(-1)
        if u_previous is None:
            u_previous = U[:self.NU].copy()
        u_previous = np.asarray(u_previous, dtype=float).reshape(self.NU)

        # YAW difference penalty gain programming (see MpcConfig).
        w_yaw = (a.r_delta_yaw if yaw_delta_weight is None
                 else float(yaw_delta_weight)) / a.scale_yaw_dps ** 2
        DtSD = self.DtSD_v + w_yaw * self.DtSD_y
        Sd = self.Sd_v + w_yaw * self.Sd_y
        self.last_yaw_delta = w_yaw * a.scale_yaw_dps ** 2

        h = self._step_durations(dt0)
        r0 = float(x0[2])
        cos_eps = math.cos(math.radians(eps_deg))
        # For vertical speed constraint: vz_ned = v . [0,0,1]; Since v is given in the plane LOS, the
        # coefficients are the z components of the plane. Since yaw rotation does not change z, heading frame
        # = NED z.
        l_v, e2_v, e3_v = los_triad(float(x0[0]), eps_deg)
        a_perpendicular = np.array([l_v[2], e2_v[2], e3_v[2]])
        # Components of the forward heading axis in the LOS basis
        self._forward_eks = np.array([l_v[0], e2_v[0], e3_v[0]])
        n_a = float(np.linalg.norm(a_perpendicular))
        a_perpendicular = a_perpendicular / n_a if n_a > 1e-9 else np.array([0.0, 0.0, 1.0])

        # difference penalty offset: u_previous in the first block (unweighted; weight is applied separately
        # with Sd)
        o_vec = np.zeros(nu_top)
        o_vec[:self.NU] = u_previous

        N = self.N
        s_v = float(np.clip(impact_value, 0.0, 1.0)) if a.impact_mode else 0.0
        self.last_impact = s_v
        # ACCELERATION (lying down ) PENALTY MULTIPLIER. Comes from the default STRIKE mix (
        # impact_acceleration_multiplier ; default 1.0 -> no-op). Since acceleration_run is installed once in
        # __init__, only a scalar multiplication remains here (no additional allocation).
        # acceleration_multiplier can also be given externally (ablation); handoff RAMP was eliminated and
        # reinstated in the tour- 2 sim (see comment MpcConfig handoff_acceleration ).
        if acceleration_multiplier is None:
            acceleration_multiplier = 1.0 + s_v * (a.impact_acceleration_multiplier - 1.0)
        self.last_acceleration_multiplier = float(acceleration_multiplier)
        acceleration_run = self.acceleration_run * float(acceleration_multiplier)
        # VERTICAL TERMINAL RAMP (with env button). It is set up by range, NOT by STRIKE mix: the goal is to
        # spread the alignment from the last second to the window of ~3 s. STALE BBOX GATE: If eps = -(ey +
        # aim) and ey comes from a rotated measurement, the vertical demand is self-reinforcing (the same trap
        # was measured at impact_blind_coast), so 0 in stale. ARM SELECTION: the ramp is armed with
        # vertical_terminal OR vertical_tgo on; so that the main lever (P + TGO) works WITHOUT turning on the old D
        # switch.
        vertical_s = 0.0
        if (a.vertical_terminal or a.vertical_tgo) and bbox_age_s <= a.stale_constraint_s:
            wide_value = max(a.vertical_terminal_range_m - a.vertical_terminal_full_m,
                        1e-6)
            vertical_s = float(np.clip(
                (a.vertical_terminal_range_m - r0) / wide_value, 0.0, 1.0))
        # t_go SHAPE TAU (with env button; see environment_vertical_tgo).
        # tau_eff = clip(t_go / k, tau_min, tau_max). KAPALIYKEN None
        # returns -> _cost_lines uses old CONSTANT tau, i.e. closed arm BIT-SAME. SAFE SIDE: If t_go is not
        # granted/invalid/negative/NaN or exceeds the ceiling, the ceiling value is used -> tau_eff sits on
        # tau_max, i.e. the SOFTEST request. Never fall into the aggressive side.
        vertical_tau = None
        t_go_diagnostic = float('nan')
        if a.vertical_tgo:
            k_eff = max(a.vertical_tgo_k * a.vertical_tgo_multiplier, 1e-3)
            tg = a.vertical_tgo_ceiling_s
            if t_go_s is not None:
                tg_f = float(t_go_s)
                if math.isfinite(tg_f) and tg_f > 0.0:
                    tg = min(tg_f, a.vertical_tgo_ceiling_s)
            t_go_diagnostic = tg
            vertical_tau = float(np.clip(tg / k_eff, a.vertical_tgo_tau_min_s,
                                      a.vertical_tgo_tau_max_s))
        self.last_tgo = t_go_diagnostic
        self.last_vertical_tau = (float('nan') if vertical_tau is None
                              else float(vertical_tau))
        # DIRECT VERTICAL ERROR (P) RAMP -- separate env button, same rationale (see environment_vertical_error). It is
        # kept SEPARATE variable because the two buttons can be opened INDEPENDENTLY and the parameters of
        # their ramps can be adjusted separately. STALE BBOX DOOR is the same: If eps comes from a returned
        # measurement, the term is self-reinforcing.
        vertical_error_s = 0.0
        if a.vertical_error and bbox_age_s <= a.stale_constraint_s:
            wide_h = max(a.vertical_error_range_m - a.vertical_error_full_m, 1e-6)
            vertical_error_s = float(np.clip(
                (a.vertical_error_range_m - r0) / wide_h, 0.0, 1.0))
        # APN (see environment_apn): IF the branch is CLOSED, apn_e is exactly 0 and in the following two functions
        # the corresponding branch does not work AT ALL -> BIT-SAME.
        apn_e = float(apn_a) if a.apn else 0.0
        if not math.isfinite(apn_e):
            apn_e = 0.0
        self.last_apn_a = apn_e
        transition = max(1, int(a.sqp_transition))
        for _ in range(transition):
            rbar, wbar, bb = self._nominal_trajectory(
                x0, U, h, d_ex, d_ey, d_r, cos_eps, r0, apn_a=apn_e)
            # SATURATED ACTUATOR: take the step-by-step horizontal gain EXACTLY from the NOMINAL orbit (same
            # linearization point required). If the arm is closed the _nominal_orbit leaves None -> old path
            # BIT-SAME.
            Xf, Gam, c2, c3, dex_v, dey_v = self._trajectory_matrices(
                x0, h, rbar, cos_eps, d_ex, d_ey, d_r, r0, apn_a=apn_e,
                al_h=self._al_h, al_v=self._al_v)
            self._cost_rows(c2, c3, dex_v, dey_v, ey_ref, confidence_value,
                                    impact_value=s_v, eps_deg=eps_deg,
                                    vertical_s=vertical_s,
                                    vertical_error_s=vertical_error_s,
                                    vertical_tau_s=vertical_tau)

            # M = W Scale (W is SINGLE zero per line -> add-multiply)
            M = self.w_deg[:, None] * Gam[self.w_column, :]
            b = self.w_deg * Xf[self.w_column] - self.w_ref

            H = 2.0 * (M.T @ M + DtSD)
            H[np.diag_indices(nu_top)] += 2.0 * (self.R_run + self.prox_run
                                                 + acceleration_run)
            f = 2.0 * (M.T @ b - self.D.T @ (Sd * o_vec)
                       - self.prox_run * U - acceleration_run * wbar)
            # LINEAR FIELD REWARD: enters gradient only (no Hessian)
            c_r, c_w1 = self._area_reward(rbar, r0, impact_value=s_v)
            f += Gam[self.column_r, :].T @ c_r
            f += Gam[self.column_w[0], :].T @ c_w1

            # --- FOV lines (PLAN layer, l1 full penalty) --- horizontal: ex_k ; vertical: beta_k = ey_k - kats*(
            # a_perpendicular . w_k ) + C
            coefficient = self.coefficient
            Gb = Gam[self.column_ey, :] - coefficient * (
                a_perpendicular[0] * Gam[self.column_w[0], :]
                + a_perpendicular[1] * Gam[self.column_w[1], :]
                + a_perpendicular[2] * Gam[self.column_w[2], :])
            Xb = Xf[self.column_ey] - coefficient * (
                a_perpendicular[0] * Xf[self.column_w[0]] + a_perpendicular[1] * Xf[self.column_w[1]]
                + a_perpendicular[2] * Xf[self.column_w[2]]) + beta_c
            Gf = np.vstack([Gam[self.column_ex, :], Gb])
            Xf_f = np.concatenate([Xf[self.column_ex], Xb])

            # --- PRECONDITIONING (Jacobi, block-uniform) ------------------- We have to use a COMMON factor for
            # the speed triple (let the sphere remain a sphere, not an ellipsoid -> let the projection remain in
            # closed form); yaw scales separately. Its main gain is to equalize the units (m/s and deg/s) and make
            # L independent of scale.
            kv = np.diag(H).reshape(self.nb, self.NU)
            pv = 1.0 / np.sqrt(np.maximum(kv[:, :3].mean(axis=1), 1e-9))
            py = 1.0 / np.sqrt(np.maximum(kv[:, 3], 1e-9))
            P = np.empty(nu_top)
            P.reshape(self.nb, self.NU)[:, :3] = pv[:, None]
            P.reshape(self.nb, self.NU)[:, 3] = py

            Hqs = (P[:, None] * H) * P[None, :]
            fs = P * f
            Gfs = Gf * P[None, :]

            # --- STEP SIZE: active-cluster awareness --- The Hessian of the FOV penalty alone inflates lambda_max
            # by a factor of ~145 (measured). ONLY lines that are close to the violation are added to L; so that
            # the step becomes full size when the penalty is passive.
            z0 = U / P
            Ltop = Hqs
            if np.any(self.fov_active):
                # Huber's CURVATURE IS ONLY AROUND THE KINK: |violation| > delta region is LINEAR (curvature 0),
                # violation When it is absent, the term is passive. Taking a large cluster "near the limit" increased
                # L to 1.6e3 , and FISTA 30 went nowhere in the iteration (measured: u1 2.9 remained, while the true
                # optimum was 14.2 ). That's why the criterion is DISTANCE from KINK.
                y_w = Xf_f + Gfs @ z0
                margin_value = a.fov_awake_margin_deg
                awake = self.fov_active & (
                    (np.abs(y_w - self.fov_upper) < margin_value)
                    | (np.abs(y_w - self.fov_alt) < margin_value))
                if np.any(awake):
                    measure = np.sqrt(self.fov_rho[awake] / a.fov_l1_delta_deg)
                    Ga = measure[:, None] * Gfs[awake, :]
                    Ltop = Hqs + (Ga.T @ Ga)
            L = 1.4 * max(float(np.linalg.eigvalsh(Ltop)[-1]), 1e-9)

            # --- HARD FOV -> input box/slice ( CBF ) --- THREE DOORS: ( 1 ) No throttling if bbox is stale
            # (runaway loop), ( 2 ) restriction ~ 0.5 leave if s not met (crash lesson), ( 3 ) ablation switch.
            stale_value = bbox_age_s > a.stale_constraint_s
            self.last_fov_free = 0
            if stale_value:
                self.last_fov_free = 1
            elif self._released:
                self.last_fov_free = 2
            if a.fov_hard and not self.last_fov_free:
                (vz_a, vz_u, yaw_a, yaw_u, beta_now, self.last_band_alt,
                 self.last_band_upper, saturated) = self._cbf_boundaries(
                    bb, a_perpendicular, beta_c,
                    d_ex if d_ex_box is None else d_ex_box,
                    d_ey if d_ey_box is None else d_ey_box, r0, rbar,
                    impact_value=s_v, tau_v=self._tau_v_block)
                self.last_beta = float(beta_now[0])
                # HYSTERESIS (up): accumulates in cycles that cannot be satisfied, cleared when satisfied.
                if saturated:
                    self.empty_counter += 1
                    if self.empty_counter > a.empty_set_ceiling_loop:
                        self._released = True
                else:
                    self.empty_counter = 0
            else:
                # LEFT AS WELL: PHYSICAL ceilings only. Framing loss is accepted -- the decision maker returns to 'on
                # position' after the 1.5 s dwell and repositions.
                vz_a = np.full(self.nb, -a.climb_ceiling_mps)
                vz_u = np.full(self.nb, a.descent_ceiling_mps)
                yaw_a = np.full(self.nb, -a.yaw_speed_ceiling_dps)
                yaw_u = np.full(self.nb, a.yaw_speed_ceiling_dps)
                self.last_band_alt = a.fov_alt_band_deg
                self.last_band_upper = a.fov_upper_band_deg
                self.last_beta = float(x0[1] + beta_c
                                      - self.coefficient
                                      * float(a_perpendicular @ x0[3:6]))
                # LATCH ERROR FIX (analysis of 2026 - 08 - 04 , eng- 3 ): Here the meter USEFULLY was not absorbing at
                # all; once it crossed the ceiling ( 41 ) it stayed there forever and the hard FOV constraint was
                # PERMANENTLY OFF at 59 - 75 % o f the run (only bbox staleness was resetting). So the hard constraint
                # the user wanted was actually disabled. Now left off, the counter is DETERMINED every cycle and the
                # constraint is RE-INACTED when it falls below backward_threshold. The hysteresis band (ceiling ->
                # backward_threshold ) prevents sudden switching between regimes.
                if self.last_fov_free == 2:
                    self.empty_counter = max(0, self.empty_counter - 1)
                    if self.empty_counter <= a.empty_backward_threshold_loop:
                        self._released = False
                else:
                    self.empty_counter = 0      # Resets stale/ablation counter
            # SUB-TARGET DEPTH CEILING: In +30 mounting, the frame cost was diving to the bottom to pull the
            # sub-axis target to the center (mount 0 does NOT have this mechanism, the ceiling remains as a safety
            # device -- see MpcConfig.depth_ceiling_m). Cuts: below target up to depth_ceiling_m. Depth below
            # target = r*sin(eps) (eps>0: target above). ONE-SIDED: cuts only descent (vz_u).
            if a.vertical_depth_ceiling and eps_deg > 0.0:
                depth_value = r0 * math.sin(math.radians(eps_deg))
                vz_ceiling_d = np.clip(
                    (a.depth_ceiling_m - depth_value) / a.depth_approach_s,
                    0.0, a.descent_ceiling_mps)
                vz_u = np.minimum(vz_u, vz_ceiling_d)
                vz_a = np.minimum(vz_a, vz_u)
            # ALTITUDE BASE: in all conditions (even when restricted)
            if altitude_m is not None:
                vz_ceiling = np.clip(
                    (float(altitude_m) - a.altitude_floor_m) / a.altitude_approach_s,
                    0.0, a.descent_ceiling_mps)
                vz_u = np.minimum(vz_u, vz_ceiling)
                vz_a = np.minimum(vz_a, vz_u)
            self._last_cbf = (float(vz_a[0]), float(vz_u[0]),
                             float(yaw_a[0]), float(yaw_u[0]))
            # ACCELERATION RAMP: sphere radius per block min(ceiling, max(| w_k |, base) + a_forward * tau ) (see
            # MpcConfig . forward_acceleration_ceiling_mps2 ).  tau actuator is the delay: the command-speed
            # difference (u - w) translates into acceleration via tau , so a* tau is exactly "how much can I
            # accelerate in a time constant".
            if a.forward_acceleration_ceiling_mps2 > 0.0:
                w_block = np.linalg.norm(
                    wbar.reshape(self.nb, self.NU)[:, :3], axis=1)
                v_ceiling_block = np.minimum(
                    a.speed_ceiling_mps,
                    np.maximum(w_block, a.speed_increase_floor_mps)
                    + a.forward_acceleration_ceiling_mps2 * max(a.speed_latency_tau_s, 1e-3))
            else:
                v_ceiling_block = np.full(self.nb, a.speed_ceiling_mps)
            self.last_v_ceiling = float(v_ceiling_block[0])
            limit_value = {
                "v": v_ceiling_block / pv,
                "vz_alt": vz_a / pv,
                "vz_upper": vz_u / pv,
                "yaw_alt": yaw_a / py,
                "yaw_upper": yaw_u / py,
            }
            z = self._fista(z0, Hqs, fs, Gfs, Xf_f, L, a_perpendicular, t_start_value,
                            limit_value, P, it_ceiling, budget_s_value)
            U = P * z
            self.last_cost = float(0.5 * z @ (Hqs @ z) + fs @ z
                                     + b @ b)

        Ur = U.reshape(self.nb, self.NU)
        self.last_duration_ms = (time.perf_counter() - t_start_value) * 1000.0
        info_value = {
            "duration_ms": self.last_duration_ms,
            "iteration": self.last_iteration,
            "budget_cut": self.last_budget_cut,
            "cost": self.last_cost,
            "rbar_last": float(rbar[-1]),
            "L": L,
            "beta": self.last_beta,
            "yaw_delta": self.last_yaw_delta,
            "band_alt": self.last_band_alt,
            "band_upper": self.last_band_upper,
            "impact_value": self.last_impact,
            "v_ceiling": self.last_v_ceiling,
            "acceleration_multiplier": self.last_acceleration_multiplier,
            "alignment_ref": self.last_alignment_ref,
            "tgo": self.last_tgo,
            "vertical_tau": self.last_vertical_tau,
            "vertical_error": self.last_vertical_error,
            "apn_a": self.last_apn_a,
            "tau_eff": self.last_tau_eff,
            "tau_eff_z": self.last_tau_eff_z,
            "fov_free": self.last_fov_free,
            "empty_counter": self.empty_counter,
            "cbf": self._last_cbf,
        }
        return Ur, info_value

    def _fista(self, z0, H, f, Gf, Xf_f, L, a_perpendicular, t_start_value, limit_value, P,
               it_ceiling, budget_s_value):
        """Accelerated projective gradient (FISTA) + adaptive restart. It operates in preconditioned (z) space;
The stopping criterion is read in PHYSICAL (U = P z) space."""
        a = self.a
        alt_w, upper_w = self.fov_alt, self.fov_upper
        rho_v = self.fov_rho
        delta = max(a.fov_l1_delta_deg, 1e-6)
        tol_ag = self.tol_weight * P
        sv = limit_value["v"]
        s_alt, s_upper = limit_value["vz_alt"], limit_value["vz_upper"]
        y_alt, y_upper = limit_value["yaw_alt"], limit_value["yaw_upper"]

        GfT = Gf.T
        step_boy = 1.0 / L

        Y = z0.copy()
        Zo = z0.copy()
        t = 1.0
        it = 0
        converged = False       # LOG ONLY: did the stopping criterion terminate the solver?
        for it in range(1, it_ceiling + 1):
            y = Xf_f + Gf @ Y
            violation = np.clip(y - upper_w, 0.0, None)
            violation += np.clip(y - alt_w, None, 0.0)
            g = H @ Y + f
            # HUBER-l1 FULL PENALTY gradient: |violation| > CONSTANT rho (l1) in delta region, under linear
            # (quadratic penalty) -> remains differentiable, first order method is valid.
            np.clip(violation, -delta, delta, out=violation)
            violation *= rho_v
            g += (1.0 / delta) * (GfT @ violation)
            Zn = _projection_sphere_slice(Y - step_boy * g, sv, a_perpendicular, s_alt,
                                     s_upper, y_alt, y_upper)
            difference = Zn - Zo
            # adaptive restart (gradient criterion): t is reset if the momentum is running in the opposite
            # direction -- cuts the oscillation seen with constant momentum, and is safe against divergence when
            # the step size is chosen optimistic
            if float(np.dot(Y - Zn, difference)) > 0.0:
                t = 1.0
                Y = Zn.copy()
            else:
                tn = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
                Y = Zn + ((t - 1.0) / tn) * difference
                t = tn
            step_value = float(np.max(np.abs(difference) * tol_ag))
            Zo = Zn
            if it >= a.iteration_floor and step_value < a.tolerance_mps:
                converged = True
                break
            if (time.perf_counter() - t_start_value) > budget_s_value:
                break
        self.last_iteration = it
        # LOG-ONLY FLAG: if we have left convergence, either the time budget or the iteration ceiling has been
        # cut; They both mean "we issued a suboptimal instruction in this loop." The control path does NOT
        # read this value.
        self.last_budget_cut = 0 if converged else 1
        return Zo


# =========================================================== CONTROLLER

class MpcController(VisualController):
    """Method MPC that complies with contract visual_base."""

    label_item = "mpc"

    def __init__(self, config_value: MpcConfig = None, diagnostic_log=None):
        self.a = config_value or MpcConfig()
        self.solver = MpcSolver(self.a)
        self.disturbance = DisturbanceEstimator(self.a)
        self.diagnostic_log_path = diagnostic_log
        self._diagnostic_f = None
        self._diagnostic = None
        self._redis = None          # MISS bridge broadcast (optional);
                                    # NOT in reset(): connection is preserved between handovers
        self.reset_value()

    # ------------------------------------------------------- situation

    def reset_value(self):
        self.internal_range = None                     # home range status
        # BLIND PN (see environment_blind_pn): frame condition advanced by dead reckoning and how long the blindness
        # lasts.
        self._blind_ex = None                  # predicted ex [deg]
        self._blind_ey = None                  # predicted ey [deg]
        self._blind_t = 0.0                    # duration of uninterrupted blindness [s]
        self._blind_ex0 = None                 # |ex| of last ACTUAL measurement (R3)
        self.blind_pn_ran = 0                # diagnosis: has this cycle progressed
        self.U = None                        # final solution (nb x NU)
        # None = "we have not issued a command yet". The first command() is seeded with handoff speed (see
        # MpcConfig.handoff_prox_seed); Since the plane LOS is not known before the first measurement, seeding
        # cannot be done HERE.
        self.u_previous = None
        self.v_ned_seed = None
        self.pitch_lpf = None
        self.area_value = None
        self.area_rate = 0.0
        self.yaw_applied_value = 0.0
        self._yaw_previous = None
        self.yaw_speed_lpf = 0.0
        self._yaw_n = 0                  # yaw rate LPF sample counter
        self.yaw_weight_lpf = None      # yaw difference penalty gain LPF
        self.disturbance.reset_value()
        self.counter = 0
        # --- MISS STATE MACHINE (see block MpcConfig miss_*) --- They are all reset HERE: each new handoff
        # starts FRESH because seed() calls reset() (best_range and the pass flags are NOT carried over
        # from the previous engagement -- if they were carried over, the second handoff would cancel itself as
        # "range opens" as soon as it was born).
        self.state_value = 'CLOSURE'           # CLOSURE | TERMINAL | IMPACT | MISS
        self.impact_blend = 0.0         # IMPACT phase mix [ 0 .. 1 ]
        self.hit_value = False             # Is IMPACT_SUCCESSFUL declared (latch)
        self.impact_vibe_value = 0.0            # vibe at the moment of announcement
        self.impact_range = 0.0          # Range at time of announcement [m]
        self._pending_event = None       # Discrete event to attach to command
        self.blind_loop = 0               # Number of blind filtered loops in IMPACT
        self.best_range = float('inf')
        self.range_rate_value = 0.0           # d(internal_range)/dt, LPF [m/s]
        self.passed_value = False             # Is the pass approved?
        self.miss_reason = ''
        self._r_previous = None
        self._transition_counter = 0
        self._transition_area_counter = 0
        self._closure_peak = 0.0         # the most within the transition circle
                                         # negative range speed [m/s]
        self._authority_t0 = None
        # PROGRESS CLOCK (see environment_progress_clock ). The beginning of the engagement is reset: the
        # stagnation of the previous period is not carried over.
        self._stagnation_clock = 0.0       # Clock [s] that fires MISS
        self._range_trace = deque()       # (t, best_candidate ) window
        # deque: popleft O(1). List + pop(0) had popped the state machine into base/loop 21 (lock 20 base).
        self._last_v_ned = None           # Last command for MISS gliding

    def seed_value2(self, handoff):
        """once at the time of handoff. The positioner's final speed command is WARM-START: the initial
solution starts from the CURRENT motion of the vehicle, not from zero."""
        self.reset_value()
        if handoff and 'cmd_vel_ned' in handoff:
            self.v_ned_seed = np.asarray(handoff['cmd_vel_ned'], dtype=float)
        print(f"[mpc] seeded, handoff rate="
              f"{None if self.v_ned_seed is None else np.round(self.v_ned_seed, 2).tolist()}")

    # ------------------------------------------------- yardimcilar

    def _range(self, measurement, w1, dt):
        """Inner range status: advance with the model, pull to it if there is a measurement.

        Measurement ONLY. range_m_value is used (bbox width is NOT a range proxy: we measured 31 px at
        corner 204 m, 26 at rear 34 px)."""
        if self.internal_range is None:
            self.internal_range = (float(measurement.range_m_value) if measurement.range_m_value is not None
                         else self.a.range_if_absent_m)
            return self.internal_range
        self.internal_range += dt * (-w1)              # advance the model
        if measurement.range_m_value is not None:
            self.internal_range += self.a.range_measurement_gain * (
                float(measurement.range_m_value) - self.internal_range)
        self.internal_range = float(max(self.a.range_floor_m * 0.5, self.internal_range))
        return self.internal_range

    def _framing_constant(self, measurement, vz_now):
        """Framing variable beta = ey - constant C for kats*vz + C.

        Elevation of camera axis relative to horizon = mount + body_pitch. beta is the vertical
        deviation of the target relative to this AXIS (+ = BELOW the axis). By definition:
            beta = ey - ey_ref,  ey_ref = -(mount + pitch + aim)
        Since climbing pushes pitch up , pitch is PREDICTED by pitch_k = pitch_measured - kats*(
        vz_k - vz_0 ) along the horizon. Putting this in beta : beta_k = ey_k - kats* vz_k + C, C =
        mount + am + pitch_measured + kats* vz_0 CAUTION: C is anchored to the MEASURED pitch and
        not to an ABSOLUTE fit pitch ; so at k= 0 beta is exactly the measured deviation and the fit
        error affects only the CHANGE term along the horizon. Pitch is LPF (so that the tape does
        not shake as the raw pitch oscillates).

        GIMBAL (pitch_coupling=False): the axis is INDEPENDENT from the body, i.e. ey_ref = -(mount
        + aim) and the climb term is dropped. pitch_lpf still updates -- it is read in the
        diagnostic log and the history is not corrupted when the key is turned on and off."""
        a = self.a
        pitch_deg = (math.degrees(measurement.pitch_rad)
                     if measurement.pitch_rad is not None else 0.0)
        pitch_deg = float(np.clip(pitch_deg, a.pitch_alt_deg, a.pitch_upper_deg))
        if self.pitch_lpf is None:
            self.pitch_lpf = pitch_deg
        else:
            dt = float(np.clip(measurement.dt, 0.02, 0.30))
            k = dt / (dt + a.pitch_lpf_tau_s)
            self.pitch_lpf += k * (pitch_deg - self.pitch_lpf)
        axis_pitch = self.pitch_lpf if a.pitch_coupling else 0.0
        coefficient = a.pitch_climb_coefficient if a.pitch_coupling else 0.0
        # PHASE O (gimbal branch): tilt is now DYNAMIC -- bbox tracks the target's elevation and broadcasts
        # the actual elevation it used in that frame with tracker_bbox_stab[7] (Measurement.tilt_deg). ey_ref
        # is established from LIVE value instead of static YILDIZ_TILT; if there is no space (old recording /
        # tilt off) static drops to mount_pitch_deg.
        tilt_live = getattr(measurement, 'tilt_deg', None)
        axis_floor = (float(tilt_live) if tilt_live is not None
                       else a.mount_pitch_deg)
        ey_ref = -(axis_floor + axis_pitch + a.aim_deg)
        beta_c = -ey_ref + coefficient * vz_now
        return ey_ref, beta_c


    def _impact_successful_control(self, measurement, r):
        """PHYSICAL CONTACT detection: OUR vibration + range.

        See MpcConfig .hit_success_* -- thresholds measured from type- 3 sim run (actual contact
        17.4 - 25.5 vs non-contact pass 3.3 ). With LATCH: once per engagement. Puts the returned
        (event, detail) pair into command() Command; skeleton writes to _event.csv and success to
        display.log.

        Target telemetry is NOT used: vibe is our VIBRATION message, range is the only target size
        allowed anyway.
        """
        a = self.a
        if (not a.impact_success_detection or self.hit_value
                or measurement.vibe_max is None):
            return None
        # RANGE: MEASURED value is used, NOT internal filter status (internal_range). TRAP (caught in 2026-08-05):
        # _range() internal state max(range_floor_m*0.5, ...) = 3.0 BASED on m (c = KVA/r coefficient in r->0
        # numerical protection due to explosion). So internal_range would NEVER go below 3.0 m and gate "r < 3 m" would
        # NEVER open with internal_range -- detection would silently never fire. Contact is a PHYSICAL event; is judged
        # by the measured range, not by a digitally based copy of the solver.
        r_measure = float(measurement.range_m_value) if measurement.range_m_value is not None else float(r)
        if (float(measurement.vibe_max) > a.impact_success_vibe
                and r_measure < a.impact_success_range_m):
            self.hit_value = True
            self.impact_vibe_value = float(measurement.vibe_max)
            self.impact_range = r_measure
            detail = (f"vibe= {self.impact_vibe_value:.1f} (threshold "
                     f"{a.impact_success_vibe:.0f}) range={r_measure:.2f} m "
                     f"status={self.state_value} hit={self.impact_blend:.2f}")
            print(f"[mpc] IMPACT_SUCCESSFUL: {detail}")
            return ('impact_successful', detail)
        return None

    def _yaw_delta_weight(self, d_ex, r, dt):
        """GAIN PROGRAMMING of yaw difference penalty (see MpcConfig).

        The criterion is the SPEED OF THE TARGET PERPENDICULAR TO LOS: v_perpendicular = |d_ex| *r/KDEG.
        Because the disturbance estimate subtracts our own yaw velocity, this number is DISSAL
        (target maneuver), meaning the gain programming is not fed back into the control loop; Since
        it is divided into ranges, it does not harden spontaneously at close range. Both were flaws
        of the previous benchmarks (|ex| and raw |d_ex|) that were eliminated by the measurement.

        The weight is also LPF: if the gain itself jumps to the loop press, the solution of QP also
        jumps (the chatter itself).
        """
        a = self.a
        v_perpendicular = abs(float(d_ex)) * max(float(r), a.range_floor_m) / KDEG
        s = float(np.clip(
            (v_perpendicular - a.yaw_free_vperp_alt)
            / max(a.yaw_free_vperp_upper - a.yaw_free_vperp_alt, 1e-6),
            0.0, 1.0))
        raw_value = a.r_delta_yaw + s * (a.r_delta_yaw_free - a.r_delta_yaw)
        if self.yaw_weight_lpf is None:
            self.yaw_weight_lpf = raw_value
        else:
            k = dt / (dt + max(a.yaw_weight_tau_s, 1e-6))
            self.yaw_weight_lpf += k * (raw_value - self.yaw_weight_lpf)
        return self.yaw_weight_lpf

    def _area_update(self, measurement, dt):
        """LINEAR bbox area (w*h, px^2) and growth rate (px^2/s).

        The reward definition is the field itself, NOT the square root; melee range is rewarded with
        square as the area is ~ K/r^2. Growth rate is the IMAGE side proxy for approach speed (it
        leaves telemetry untouched)."""
        if measurement.bbox_w is None or measurement.bbox_h is None:
            return self.area_value, self.area_rate
        A = float(measurement.bbox_w) * float(measurement.bbox_h)
        if self.area_value is None:
            self.area_value, self.area_rate = A, 0.0
            return self.area_value, self.area_rate
        if 0.01 < dt < 0.35:
            raw_value = (A - self.area_value) / dt
            k = dt / (dt + self.a.area_rate_tau_s)
            self.area_rate += k * (raw_value - self.area_rate)
        self.area_value = A
        return self.area_value, self.area_rate

    def _t_go(self):
        """TIME REMAINING [s] or None -- NOT NEW ESTIMATE.

        Its inputs are two numbers that the controller ALREADY holds: * self._r_inner range r
        (allowed single target measure), * self, where_r_previous is updated. range_rate_value = d( internal_range
        )/dt, LPF tau 0.30 s -- the same number is used in the transition closing-speed condition
        (see _state_machine and transition_closure_threshold_mps ). So the "only range from target" rule is
        not violated and a new observer/estimator is NOT added.

        WHY None MAY RETURN: range_rate_value negative = we are closing. If the turn-off is below the
        threshold (opening, cross-geometry, LPF warm-up) t_go is either infinite or meaningless.
        None returns and resolve() drops to the SOFT branch (tau_max) -- no AGGRESSIVE request is
        generated from invalid data. range_rate_value is fresh because _state_machine is called BEFORE
        solve().
        """
        a = self.a
        closure = -float(self.range_rate_value)
        if (self._r_previous is None
                or closure < a.vertical_tgo_closure_min_mps):
            return None
        return float(self._r_previous) / closure

    # ------------------------------------------- MISS state machine

    def _state_machine(self, measurement, r, area_rate, dt):
        """CLOSE -> TERMINAL -> MISS. ABOVE the MPC, OUTSIDE the cost.

        Why not in cost: see. MpcConfig miss_* block -- "I passed, I must quit" is a TERMINATION
        decision, not an optimum.

        Cost: ~ 20 floating point operation + two counters. Solver budget (p95 ~ 8.6 ms / ceiling 13
        ms ) cannot be measured next to it; MISS declaration After this, the cycle cost DECREASES
        because the solver NEVER runs.

        This function DOES NOT TOUCH TARGET TELEMETRY: its inputs are r (range, single target
        measure allowed), area_rate (from bbox) and our own clock. Target speed is NOT derived.
        """
        a = self.a
        if self._authority_t0 is None:
            self._authority_t0 = measurement.t
        elapsed_item = float(measurement.t - self._authority_t0)

        # --- range speed: internal_range derivative + LPF (raw r_measurement derivative cannot be used, +-30 m/s introduces
        # noise; see MpcConfig) ---
        if self._r_previous is not None:
            raw_value = float(np.clip((r - self._r_previous) / dt, -60.0, 60.0))
            k = dt / (dt + max(a.range_rate_tau_s, 1e-6))
            self.range_rate_value += k * (raw_value - self.range_rate_value)
        self._r_previous = float(r)
        if r < self.best_range:
            self.best_range = float(r)

        # --- ADVANCE TIME (see environment_progress_clock ) --- ALL BEHIND THE FLAG. First it was written
        # INDEPENDENT from the flag ("to be read consecutively in base conditions") but mpc_test's "state
        # machine CHEAP (< 20 exponent/cycle)" unlocked: 21.0 exponent measured. Not a single microsecond of
        # the closed arm is wasted for ease of diagnosis -- moreover, subsequent analysis is already possible:
        # CSV with r and t images is available, the question "if the arm were open" can be recalculated
        # offline. PROGRESS MEASURE = RECOVERY RATE OF BEST-SO-FAR. The trace is a MONOTONOUS DECREASING
        # signal, so it CANNOT be fooled by oscillation -- see for its measurement.
        # progress_closure_threshold_mps .
        if a.progress_clock:
            trace_samples = self._range_trace
            trace_samples.append((float(measurement.t), float(self.best_range)))
            while len(trace_samples) > 2 and measurement.t - trace_samples[0][0] > a.progress_window_s:
                trace_samples.popleft()
            t_previous, best_previous = trace_samples[0]
            duration_value = measurement.t - t_previous
            progress_gain_item = ((best_previous - self.best_range) / duration_value
                       if duration_value >= 0.5 * a.progress_window_s else 0.0)
            progressing = progress_gain_item > a.progress_closure_threshold_mps
            self._stagnation_clock = max(
                0.0, self._stagnation_clock
                + dt * ((1.0 - a.progress_gain) if progressing else 1.0))

        # --- PHASE (CLOSURE -> TERMINAL -> STRIKE) --- INDEPENDENT from SCA: these are the GUIDANCE phases,
        # not the termination decision. In the past, phase assignment was BEHIND gate miss_mode; With the
        # --no-miss ablation, the phase would not change at all, meaning the ablation was turning off two
        # things at once.
        if self.state_value != 'MISS':
            if self.best_range <= a.terminal_range_m:
                self.state_value = 'TERMINAL'
            if a.impact_mode and self.best_range <= a.impact_range_m:
                self.state_value = 'IMPACT'
        # STRIKE MIX: phase is LATCHed, but the mixture lasts with INSTANT range. If we miss and open, the
        # mixture will automatically turn into 0 (the cost returns to nominal), while the phase label remains
        # throughout -- for log readability.
        if self.state_value == 'IMPACT':
            width_value = max(a.impact_range_m - a.impact_full_range_m, 1e-6)
            self.impact_blend = float(np.clip(
                (a.impact_range_m - r) / width_value, 0.0, 1.0))
        else:
            self.impact_blend = 0.0

        if not a.miss_mode or self.state_value == 'MISS':
            return                       # MISS has LATCH: up to circuit

        # --- PASS determination: two independent witnesses, "OR"; MANDATORY closing-speed requirement
        # (penetration proof) on ---
        if r <= a.miss_transition_arm_m:
            self._closure_peak = min(self._closure_peak, self.range_rate_value)
        fresh_value = measurement.bbox_age_s <= a.stale_constraint_s
        self._transition_counter = (self._transition_counter + 1
                             if self.range_rate_value > a.transition_range_rate_threshold_mps
                             else 0)
        self._transition_area_counter = (self._transition_area_counter + 1
                                  if (fresh_value and area_rate < 0.0) else 0)
        if (self.best_range <= a.miss_transition_arm_m
                and self._closure_peak <= -a.transition_closure_threshold_mps
                and (self._transition_counter >= a.transition_confirmation_loop
                     or self._transition_area_counter >= a.transition_area_confirmation_loop)):
            self.passed_value = True

        # --- MISS ilani ---
        if elapsed_item < a.miss_start_protection_s:
            return
        reason_value = ''
        if self.passed_value and r > self.best_range + a.miss_transition_opening_m:
            reason_value = (f"passage approved, range opening ({r:.0f} m > best "
                     f"{self.best_range:.0f} + {a.miss_transition_opening_m:.0f})")
        elif (self.best_range <= a.miss_arm_m
                and r > self.best_range + a.miss_opening_m):
            reason_value = (f"range opening ({r:.0f} m > best "
                     f"{self.best_range:.0f} + {a.miss_opening_m:.0f})")
        elif r > a.miss_absolute_m:
            reason_value = f"absolute range ({r:.0f} m > {a.miss_absolute_m:.0f})"
        elif not a.progress_clock and elapsed_item > a.miss_time_timeout_s:
            reason_value = (f"timeout ({elapsed_item:.1f} s > {a.miss_time_timeout_s:.0f}), "
                     f"best range {self.best_range:.1f} m")
        elif a.progress_clock and self._stagnation_clock > a.miss_time_timeout_s:
            # PROGRESS CLOCK lever: the door is no longer "how many seconds have I been in authority" but "how
            # many seconds have I NOT PROGRESSED". Both are written in the reason string so that it can be read in
            # ONE LINE from the log which arm it is and how much profit it uses. PREFIX "timeout/" CONSCIOUS:
            # Causes of ISS are everywhere (mpc_test, metric scripts, run summaries, your A/B tables) are
            # classified with this TOKEN. Writing only "recession" would silently relegate the misses in the same
            # family to the "other" bucket -- the cross-branch comparison would be BROKEN. The prefix is
            # ​​retained, new information is added in parentheses.
            reason_value = (f"timeout/halt ({self._stagnation_clock:.1f} s > "
                     f"{a.miss_time_timeout_s:.0f}; engagement {elapsed_item:.1f} s), "
                     f"best range {self.best_range:.1f} m")
        elif a.progress_clock and elapsed_item > a.progress_ceiling_s:
            reason_value = (f"timeout/progress ceiling ({elapsed_item:.1f} s > "
                     f"{a.progress_ceiling_s:.0f}), best range "
                     f"{self.best_range:.1f} m")
        if reason_value:
            self.state_value = 'MISS'
            self.miss_reason = reason_value
            print(f"[mpc] MISS: {reason_value} -> deauthorizing")
            self._miss_publish()

    def _miss_publish(self):
        """OPTIONAL Redis bridge spring (no miss_redis_key bossa).

        'command_authority' is not written CONSCIOUSLY: bbox_to_redis crushes that key with its own mode
        in every frame (bbox_to_redis.py:472), so the value written by the controller would be
        deleted in ~33 ms. 'manual_stop' is also unused: it is an OPERATOR kill-switch, LATCHED
        (if not cleared the decision maker will NEVER go back to 'display') and shared."""
        key_value = self.a.miss_redis_key
        if not key_value:
            return
        try:
            import json
            import redis
            if getattr(self, '_redis', None) is None:
                self._redis = redis.Redis(host='localhost', port=6379, db=0)
            self._redis.set(key_value, json.dumps({
                't_mono': time.monotonic(), 'reason_value': self.miss_reason,
                'best_range': round(float(self.best_range), 2)}))
        except Exception as exc:                     # Broadcast NEVER disrupts the flight
            print(f"[mpc] MISS Redis broadcast failed: {exc}")

    def _miss_command(self, measurement, ex, ey, eps, r, d_ex, d_ey, ey_ref,
                    area_value, area_rate, dt):
        """Command in MISS: decelerating coast + 'release' flag.

        WHY NOT ZERO: zero is not "not giving a command", it is a FULL BRAKE command from full
        throttle (same lesson learned on visual_base on 2026-08-04). WHY IT'S NOT GUIDANCE: this
        is the very defect measured -- still producing command towards the target with 9-12 m/s
        after passing the target. Yaw is NOT COMMANDED (yaw_rate=None): blind turn makes it
        difficult for the DM to reposition.

        BRAKE (2026-08-05, 35 m/s tour): glide is a command that no longer descends at CONSTANT
        SPEED but RAMPS to miss_coast_speed_mps, MAINTAINING DIRECTION (see fig.
        MpcConfig.miss_filter_*). The reason is geometric: the smallest turning radius is v^2/a, i.e.
        245 m on 35 m/s, 29 m on 12 m/s. The speed at which authority is released determines
        the repositioning time of positional guidance.

        The solver does NOT run -- MISS cycle is CHEAPER than normal."""
        a = self.a
        v = (np.asarray(measurement.vel_ned, dtype=float).copy()
             if measurement.vel_ned is not None
             else (self._last_v_ned.copy() if self._last_v_ned is not None
                   else np.zeros(3)))
        # Lower the SPEED MAGNITUDE with the ramp, maintain the DIRECTION.
        speed_value = float(np.linalg.norm(v))
        if speed_value > 1e-6:
            target_speed = max(a.miss_coast_speed_mps,
                            speed_value - a.miss_coast_acceleration_mps2 * dt)
            v = v * (min(target_speed, speed_value) / speed_value)
        self._last_v_ned = v.copy()
        self.yaw_applied_value = 0.0
        self._diagnostic_write(measurement, ex, ey, eps, r, d_ex, d_ey,
                       np.zeros(MpcSolver.NU), v, ey_ref, area_value, area_rate,
                       {'duration_ms': 0.0, 'iteration': 0, 'cost': 0.0})
        self.counter += 1
        k = Command(vel_ned=v, yaw_rate_dps=None)
        # COMMON FILE AGREEMENT RECOMMENDATION: 'drop: bool = False' and 'release_reason: str = ""' fields should
        # be added to visual_base.Command; When VisualLoop leaves=True, it should activate=False and
        # notify the decision maker. It is put as an attribute UNTIL the field is ADDED (The command is a
        # dataclass, no __slots__): skeleton ignores, behavior is not broken, tests read.
        k.release_value = True
        k.release_reason = self.miss_reason
        return k

    def _blind_command(self, measurement, ex, ey, eps, r, d_ex, d_ey, ey_ref,
                   area_value, area_rate):
        """IF bbox IS stale during the STRIKE phase: repeat the last command EXACTLY (coast).

        The correct response to the stale bbox at long range is to RELEASE THE HARD CONTRACTOR (gate
        stale_constraint_s): While ey is rotated, the constraint measures the vertical velocity it
        produces and becomes even harder. In HIT, the correct answer is to COMPLETE the PLAN -- t_go
        < 1.6 s, the target is on the last seen LOS, the camera is FIXED 0 deg, so we are already
        blind and there is no use in deepening that blindness with a new maneuver. solver DOES NOT
        run; the command continues smoothly through the skeleton's LPF. The SCA state machine
        terminates this glide (range witness comes from estimator, INDEPENDENT from bbox).

        Yaw is COMMANDED (last value): if the nose turn stops, we lose the last bearing of the
        target; The nose should continue to point in the right direction even if it is out of frame."""
        v = (self._last_v_ned.copy() if self._last_v_ned is not None
             else (np.asarray(measurement.vel_ned, dtype=float).copy()
                   if measurement.vel_ned is not None else np.zeros(3)))
        u0 = (self.u_previous.copy() if self.u_previous is not None
              else np.zeros(MpcSolver.NU))
        self.blind_loop += 1
        self._diagnostic_write(measurement, ex, ey, eps, r, d_ex, d_ey, u0, v, ey_ref,
                       area_value, area_rate,
                       {'duration_ms': 0.0, 'iteration': 0, 'cost': 0.0,
                        'impact_value': self.impact_blend})
        self.counter += 1
        k = Command(vel_ned=v,
                  yaw_rate_dps=(float(self.yaw_applied_value)
                                if self.a.yaw_command_provide else None))
        k.release_value = False
        k.release_reason = ''
        return k

    def _warm_start(self, v_heading, l, e2, e3):
        """Shift the previous solution by one block; Otherwise, produce at handoff speed.

        v_heading: the final speed command of the positioner in the revolution is translated into
        the HEADING frame (Rz(-yaw)). So the initial solution does not start from scratch but from
        the current motion of the vehicle -- the same logic as seeding the skeleton LPF, no jumping."""
        nb, NU = self.solver.nb, MpcSolver.NU
        if self.U is not None:
            U = np.vstack([self.U[1:], self.U[-1:]])
            return U.reshape(-1)
        if v_heading is None:
            return np.zeros(nb * NU)
        u0 = np.array([float(v_heading @ l), float(v_heading @ e2),
                       float(v_heading @ e3), 0.0])
        return np.tile(u0, nb)

    def _yaw_rate_measure(self, yaw_rad, dt):
        """Measure yaw speed from YOUR attitude (not the commanded one).

        The disturbance estimate will extract the effect yaw from ex_dot; Because the commanded yaw
        speed occurs with a delay in the autopilot, using the command leaks a dummy component into
        the jammer.

        handoff COOLNESS (2026-08-04, measured): LPF was starting from zero and the FIRST INCREASE
        OF THE DISTORTOR is calculated in the first instance of this very LPF. At the moment of
        handoff, the vehicle generally TURNS (positioned guidance controls the target yaw); Starting
        from scratch, LPF in the first example only sees k=dt/(dt+0.15)~0.25 of the real speed of
        yaw, meaning 75% of the speed of yaw CANNOT be extracted from the jammer and is directly fed
        into d_ex. It enters as a component. Moreover, in the first example, the jammer adopts that
        value AS IS, with the gain 1/n=1. Solution: IN THE FIRST EXAMPLE ONLY k=1 (the first step of
        the running average), i.e. yaw enters the velocity as measured in the SAME WINDOW as the
        first residue of the disturbance. Subsequent examples remain at normal LPF -- INTENTIONAL: 2
        with max(1/n, ...). and 3. Speeding up the sample also exposes the estimation to noise and
        makes the 25 s of closed loop benchmarks (test 5i) play chaotically. The gain changes only
        in the single instance where the residue is CONTAMINATED."""
        if yaw_rad is None:
            return self.yaw_applied_value
        if self._yaw_previous is None or not (0.01 < dt < 0.35):
            self._yaw_previous = yaw_rad
            return self.yaw_speed_lpf
        difference = math.degrees(
            (yaw_rad - self._yaw_previous + math.pi) % (2 * math.pi) - math.pi)
        self._yaw_previous = yaw_rad
        raw_value = difference / dt
        self._yaw_n += 1
        k = dt / (dt + 0.15)          # short LPF: yaw speed is already fast
        if self.a.handoff_yaw_seed and self._yaw_n == 1:
            k = 1.0
        self.yaw_speed_lpf += k * (float(np.clip(raw_value, -200.0, 200.0))
                                 - self.yaw_speed_lpf)
        return self.yaw_speed_lpf

    # ------------------------------------------------------------- command

    def command_value(self, measurement: Measurement) -> Command:
        a = self.a
        dt = float(np.clip(measurement.dt, 0.02, 0.30))
        ex = float(measurement.ex_deg)
        ey = float(measurement.ey_deg)
        # --- BLIND PN: bbox IF THE FRAME IS OLD, DEAD COMPUTE FORWARD -----------
        # (bkz. environment_blind_pn; R1-R5 raylari orada gerekcelendirildi)
        # In the CLOSED branch this block does not work AT ALL -> behavior is BIT-SAME.
        self.blind_pn_ran = 0
        stale_value = measurement.bbox_age_s > a.stale_constraint_s
        if a.blind_pn and stale_value and measurement.range_m_value is not None:   # R4
            self._blind_t += dt
            # RANGE GATE: gain occurs AT TERMINAL, loss comes from blind frames at long range (see
            # blind_pn_range_m).
            r_gate = (self.internal_range if self.internal_range is not None
                      else float(measurement.range_m_value))
            if (self._blind_t <= a.blind_pn_maximum_s                 # R1
                    and r_gate <= a.blind_pn_range_m             # RANGE GATE
                    and self._blind_ex is not None):
                damping = math.exp(-self._blind_t / max(a.blind_pn_tau_s, 1e-3))
                r_i = max(self.internal_range if self.internal_range is not None else
                          float(measurement.range_m_value), a.range_floor_m)
                eps_i = -(self._blind_ey + a.aim_deg)
                c2_i = KDEG / (r_i * max(math.cos(math.radians(eps_i)), 0.2))
                c3_i = KDEG / r_i
                w_i = (self.u_previous[:3] if self.u_previous is not None
                       else np.zeros(3))
                yaw_i = float(self.yaw_applied_value)
                self._blind_ex += dt * (-c2_i * w_i[1] - yaw_i
                                      + self.disturbance.d_ex * damping)
                self._blind_ey += dt * (-c3_i * w_i[2]
                                      + self.disturbance.d_ey * damping)
                # R3 + R3': clamp relative to both the last ACTUAL measurement (numerator) and the ABSOLUTE framing
                # half-angle.
                ceiling = a.blind_pn_ex_absolute_deg
                if self._blind_ex0 is not None:
                    ceiling = min(self._blind_ex0 + a.blind_pn_ex_margin_deg, ceiling)
                self._blind_ex = float(np.clip(self._blind_ex, -ceiling, ceiling))
                ex, ey = self._blind_ex, self._blind_ey
                self.blind_pn_ran = 1
        elif not stale_value:
            # FRESH measurement: RE-SEED with dead reckoning measurement.
            self._blind_t = 0.0
            self._blind_ex, self._blind_ey = ex, ey
            self._blind_ex0 = abs(ex)
        eps = -(ey + a.aim_deg)              # elevation of the target relative to the horizon

        yaw = measurement.yaw_rad if measurement.yaw_rad is not None else 0.0
        l, e2, e3 = los_triad(ex, eps)
        c, s = math.cos(yaw), math.sin(yaw)

        def _ned_to_heading(v):
            v = np.asarray(v, dtype=float)
            return np.array([c * v[0] + s * v[1],
                             -s * v[0] + c * v[1], v[2]])

        # --- own speed on plane LOS (MEASURED, not commanded: actuator delay so does not leak into
        # estimation) ---
        if measurement.vel_ned is not None:
            v_h = _ned_to_heading(measurement.vel_ned)
            w = np.array([float(v_h @ l), float(v_h @ e2), float(v_h @ e3)])
        elif self.u_previous is not None:
            w = self.u_previous[:3].copy()
        else:
            w = np.zeros(3)

        r = self._range(measurement, w[0], dt)
        r_g = max(r, a.range_floor_m)
        c2 = KDEG / (r_g * max(math.cos(math.radians(eps)), 0.2))
        c3 = KDEG / r_g
        yaw_rate_value = self._yaw_rate_measure(measurement.yaw_rad, dt)

        d_ex, d_ey, d_r = self.disturbance.update_value(
            ex, ey, measurement.range_m_value, dt, c2, c3, w[0], w[1], w[2], yaw_rate_value)
        if a.range_disturbance_source != "range_value":
            d_r = 0.0

        vz_now = float(l[2] * w[0] + e2[2] * w[1] + e3[2] * w[2])
        ey_ref, beta_c = self._framing_constant(measurement, vz_now)
        area_value, area_rate = self._area_update(measurement, dt)

        # --- SCALE STATE MACHINE (BEFORE solver) --- Everything up to here is MEASUREMENT update (few
        # microseconds); It is an expensive single-step solver and is skipped in SCA. Location conscious: the
        # state machine does NOT look at the solver's answer, because the answer to the "pass" question is in
        # geometry, not optimization.
        self._state_machine(measurement, r, area_rate, dt)
        # PHYSICAL CONTACT: AFTER the state machine (keep the mix of state and beat current in the detail
        # line), BEFORE the command path -- the event gets stuck in the Command no matter WHICH branch we
        # return from (see _event_tak).
        self._pending_event = self._impact_successful_control(measurement, r)
        if self.state_value == 'MISS':
            return self._event_tak(self._miss_command(
                measurement, ex, ey, eps, r, d_ex, d_ey, ey_ref, area_value,
                area_rate, dt))
        # STRIKE + STAFF BBOX -> BLIND FLOATING (see _blind_command) WHEN BLIND PN is ON, this lever is
        # SKIPPED: the real gain is right here -- running the solver with the ADVANCED framing instead of
        # repeating the command in the terminal (HIT, r <= 8 m). When R1 expires, blind_pn_ran becomes 0 and
        # the arm is activated again (safe fall back).
        if (self.state_value == 'IMPACT' and a.impact_blind_coast
                and not (a.blind_pn and self.blind_pn_ran)
                and r <= a.impact_blind_range_m
                and measurement.bbox_age_s > a.stale_constraint_s
                and self._last_v_ned is not None):
            return self._event_tak(self._blind_command(
                measurement, ex, ey, eps, r, d_ex, d_ey, ey_ref, area_value, area_rate))

        x0 = np.array([ex, ey, r, w[0], w[1], w[2]])
        v_seed = (None if self.v_ned_seed is None
                   else _ned_to_heading(self.v_ned_seed))
        U_warm = self._warm_start(v_seed, l, e2, e3)
        if self.u_previous is None:
            # FIRST CYCLE: the anchor of the difference penalty. Zero means "the vehicle was stopped" and the
            # first command is to apply the brakes; It is actually the last instruction of the positioner (exactly
            # the first block of U_warm). Yaw channel 0 remains: position yaw does not command SPEED.
            self.u_previous = np.zeros(MpcSolver.NU)
            if a.handoff_prox_seed:
                self.u_previous[:3] = np.asarray(U_warm[:3], dtype=float)
        U, info_value = self.solver.solve_value(
            x0, d_ex, d_ey, d_r, eps, ey_ref, beta_c,
            dt0=dt, U_warm=U_warm, u_previous=self.u_previous,
            confidence_value=self.disturbance.confidence_value, bbox_age_s=measurement.bbox_age_s,
            altitude_m=(None if measurement.pos_ned is None
                      else -float(measurement.pos_ned[2])),
            d_ex_box=self.disturbance.d_ex_box,
            d_ey_box=self.disturbance.d_ey_box,
            yaw_delta_weight=self._yaw_delta_weight(d_ex, r, dt),
            impact_value=self.impact_blend,
            t_go_s=self._t_go(),
            # APN (with env button, default OFF -> full 0). Deadband, confidence multiplier and clamp are applied
            # in the CUTTER.
            apn_a=self.disturbance.apn_a_active())
        self.U = U
        u0 = U[0]
        self.u_previous = u0.copy()

        # --- LOS triad -> heading frame -> NED ---
        v_h = u0[0] * l + u0[1] * e2 + u0[2] * e3
        v_ned_cmd = body_forward_ned(yaw, v_h[0], v_h[1], v_h[2])
        yaw_rate = float(u0[3]) if a.yaw_command_provide else None
        self.yaw_applied_value = float(u0[3]) if a.yaw_command_provide else 0.0

        self._diagnostic_write(measurement, ex, ey, eps, r, d_ex, d_ey, u0, v_ned_cmd,
                       ey_ref, area_value, area_rate, info_value)
        self.counter += 1
        self._last_v_ned = v_ned_cmd.copy()
        k = Command(vel_ned=v_ned_cmd, yaw_rate_dps=yaw_rate)
        k.release_value = False                  # see _miss_command contract note
        k.release_reason = ''
        return self._event_tak(k)

    def _event_tak(self, k):
        """Attaches the pending discrete event to the Command (writes to skeleton _event.csv).

        ONE POINT: event generation is INDEPENDENT of the command path (contact can be in all three
        arms -- normal / HIS / blind glide --), so generation is done once at the beginning of
        command(), carry here."""
        event_value = getattr(self, '_pending_event', None)
        if event_value:
            k.event_value, k.event_detail = event_value
            self._pending_event = None
        self._command_start(k)
        return k

    def _command_start(self, k):
        """Send the requested command to the terminal (default 1 Hz).

        _diagnosis CSV already writes EVERY step; This line is for live viewing. The location is
        deliberately _event_tak: all three command arms (normal / HIS / blind glide) pass here, so
        the value printed is the command itself to the vehicle. The reason for the throttle is loop
        speed: there is a loop history that drops to 2 Hz (see LOG_DICTIONARY), so writing to stdout at
        every tick doesn't add salt to that wound. $YILDIZ_COMMAND_PRESSURE_S sets the period; 0 or
        closes negative."""
        period_value = getattr(self, '_command_pressure_period', None)
        if period_value is None:
            try:
                period_value = float(os.environ.get('YILDIZ_COMMAND_PRESSURE_S',
                                               '1.0'))
            except ValueError:
                period_value = 1.0
            self._command_pressure_period = period_value
        if period_value <= 0:
            return
        now_value = time.monotonic()
        if now_value - getattr(self, '_command_pressure_t', 0.0) < period_value:
            return
        self._command_pressure_t = now_value
        v = np.asarray(k.vel_ned, dtype=float)
        yr = '  --' if k.yaw_rate_dps is None else f"{k.yaw_rate_dps:+6.1f}"
        ek = f" RELEASE({k.release_reason})" if k.release_value else ''
        print(f"[mpc] command vN= {v[0]:+6.2f} vE= {v[1]:+6.2f} "
              f"vD={v[2]:+6.2f} m/s |v|={float(np.linalg.norm(v)):5.2f} "
              f"yaw_rate_value={yr} dps status={self.state_value}{ek}", flush=True)

    # ------------------------------------------------------ diagnostic log

    def _diagnostic_write(self, measurement, ex, ey, eps, r, d_ex, d_ey, u0, v_ned,
                  ey_ref, area_value, area_rate, info_value):
        """REWARD COLUMNS ALIGNED TO LINEAR AREA (revision 2026 - 08 - 04 ): 'area' = w*h [ px ^ 2 ] (NOT
square root), ' area_rate ' = dA/dt [ px^2/s]. ' beta ' = vertical deviation of the target relative
to the CAMERA AXIS
(+ = below the axis) and 'beta_limit' the current hard lower limit -- the live measure of the
framing margin, the main column of the loss analysis.

        ASSEMBLY 0 READING NOTE: beta is now typically NEGATIVE (ABOVE target axis: ~ -16 deg at
        standoff, ~ 0 at impact). Framing margin should therefore be read relative to -fov_upper_band
        (-17.5) and physical edge (-20.07), not 'beta_limit'; The lower limit is binding only on the
        terminal brake. 'yaw_ceza': CURRENT programmed value of differential penalty yaw (10 =
        calm/responsible, 1 = agile). It is derived from the vertical velocity component of the
        target maneuver; In SIM, the question "Is authorization granted?" is read from this column.

        SCARE COLUMNS (probe, 2026-08-05): 'status' : CLOSE | TERMINAL | WSC 'best_range' :
        smallest r [m] achieved in this engagement 'range_rate_value' : d(internal_range)/dt filtered [m/s];
        NEGATIVE = we are closing, POSITIVE = we are opening. Terminal transition is the moment when
        this column changes sign. Miss diagnostics in one line: find the first line where the status
        is Miss, 'range_rate_value' and 'best_range' verify the cause (line '[mpc] MISS:' in
        display.log) with their current values."""
        if self.diagnostic_log_path is None:
            return
        if self._diagnostic is None:
            os.makedirs(os.path.dirname(self.diagnostic_log_path), exist_ok=True)
            self._diagnostic_f = open(self.diagnostic_log_path, 'w', newline='')
            self._diagnostic = csv.writer(self._diagnostic_f)
            self._diagnostic.writerow(
                ['t', 'dt', 'bbox_age', 'ex', 'ey', 'eps', 'ey_ref',
                 'beta', 'beta_limit', 'depth_value', 'fov_free',
                 'empty_counter', 'internal_range', 'r_measurement', 'area_value', 'area_rate',
                 'd_ex', 'd_ey', 'u1', 'u2', 'u3', 'yaw_dps', 'yaw_ceza',
                 'los_speed_az', 'los_speed_el', 'vx', 'vy', 'vz', 'pitch_lpf',
                 'vz_alt_cbf', 'vz_upper_cbf', 'yaw_alt_cbf', 'yaw_upper_cbf',
                 'duration_ms', 'iter', 'cost',
                 # MISSING CASE MACHINE -- columns deliberately added to the END: so that the INDEX of existing columns
                 # does not shift (so that the test pilot's awk/cut single lines do not break).
                 'state_value', 'best_range', 'range_rate_value',
                 # absolute clock bridge (video/bbox alignment); at the end for the same index-preservation reasons.
                 't_unix',
                 # STRIKE PHASE (2026-08-05) -- again SONA, index-protection. 'hit' : mixing coefficient [0..1]. 0 =
                 # nominal cost, 1 = pure capture. 0 at 22, 1 at 8 m when 'state' is STRIKE. 'band_upper' : current TOP
                 # framing band [deg]. On STRIKE opens to 17.5 -> 19.0; it narrows when there is forward acceleration
                 # (new acceleration term).
                 'impact_value', 'band_upper',
                 #  ' acceleration_multiply ': Multiplier applied to q_acceleration in that cycle (IMPACT acceleration
                 #  multiplier; default is 1.0). ' alignment_ref ' : STRIKE terminal vertical alignment bias [dps].
                 #  eps > 10 deg while positive = "climb even" ( standoff melting).  0 = in deadzone or out of HIT.
                 'acceleration_multiply', 'alignment_ref',
                 #  'hit' : PHYSICAL CONTACT latch (0/1). When vibe > 15 AND range < 3 m is seen, it switches to 1 and
                 #  remains throughout the engagement. In the run summary, NUMBER of innings is the number of passes of
                 #  this column 0->1. 'vibe': OUR OWN vibration (the largest axis).
                 'hit_value', 'vibe',
                 # 'budget_cut' (2026-08-07): 1 = FISTA did not converge, i.e. hit the iteration ceiling
                 # (iteration_ceiling) or duration budget (duration_budget_ms). 'iter' and 'duration_ms' ALONE do not say this:
                 # a solution touching the ceiling or a solution converging can both print the same numbers. If the CPU
                 # gets stuck on the Raspberry Pi 5 , the solver SILENTLY starts issuing suboptimal commands; this
                 # column is the only visible signature of that degradation.  0 = stop criterion met (healthy). Added
                 # to the end again (index-protection).
                 'budget_cut',
                 # ' vertical_error ' ( 2026 - 08 - 07 ): The angle residual that the DIRECT vertical error (P) term
                 # imposes on the cost in that frame ( deg , sign same as eps ).  0 = term off, in deadband, out of
                 # range ramp or bbox stale. It is read SIDE BY SIDE with ' alignment_ref ' (DERIVATION branch): The
                 # only visible answer to the question of whether it works P or D. added to the end (index-protection).
                 'vertical_error',
                 # 'tgo' / 'vertical_tau' (2026-08-07, tur-3): Diagnostics of the t_go shaped vertical speed reference
                 # (see environment_vertical_tgo). 'tgo' : remaining time [s] USED in law
                 #               (cap/override applied). ' vertical_tau ' : tau_eff = clip(tgo/k, tau_min , tau_max ).
                 #               Both EMPTY = arm closed.  vertical_tau to tau_min ( 0.30 ) IF IT IS STICKED, it means
                 #               it is saturated with age -- the first column to look at in vine diagnosis. Added END
                 #               (index-protection).
                 'tgo', 'vertical_tau',
                 # DIAGNOSIS OF APN (2026-08-08) -- again SONA, index-protection. See environment_apn. 'v_perpendicular' : estimated
                 # target VERTICAL speed [m/s]
                 #            (= d_ex * r / KDEG; kol KAPALIYKEN de
                 #            written -- also needed in the base run to measure what APN will bring). 'a_perpendicular' :
                 #            filtered derivative of v_perpendicular [m/s^2], RAW estimate (DEADBAND/CONFIDENCE NOT IMPLEMENTED).
                 #            'apn_a' : the value ACTUALLY used in the law; Deadband + trust + multiplier applied. 0 if
                 #            the arm is closed or in dead band. The rotation/direct phase separation is read from
                 #            these three columns with 'a_perpendicular' p50/p90: a_perpendicular is large, but if apn_a 0 is the dead
                 #            band, not noise, the door is interrupted.
                 'v_perpendicular', 'a_perpendicular', 'apn_a',
                 # DIAGNOSIS OF SATURATED ACTION (2026-08-08) -- SONA, index-protection. See environment_actuator. 'tau_eff'
                 # : HORIZONTAL effective time constant of the FIRST step [s] = max(tau_lin, |e_horizontal|/a_max). EMPTY =
                 # lever CLOSED (old constant 1.00 s path). If we stick to tau_lin (1.7), we are in the linear region;
                 # as it grows, the plan is SATURATED -- the state where the model knows the maneuver takes time,
                 # rather than generating unachievable commands. 'tau_eff_z': VERTICAL OF THE SAME = max(tau_lin_z,
                 # |e_v|/a_max_z). In the first version, there was NO vertical limit and the demand was escaping to
                 # this channel (|u3| p90 increased 2.2-fold, ALTITUDE ABORT increased 1.9-fold) -- see. MpcConfig.actuator_a_max_z_mps2.
                 # 'u_saturation' : |u_horizontal| did it touch 99% of the speed cap (0/1). Expected effect: WHEN the lever is
                 # OPEN this rate SHOULD decrease (unreachable u1 jumps decrease).
                 'tau_eff', 'tau_eff_z', 'u_saturation', 'blind_pn',
                 # PROGRESSION TIME DIAGNOSIS (2026-08-08) -- SONA, index-protection. See environment_progress_clock. 'blind_pn'
                 # : has the frame been advanced by DEAD CALCULATION in this cycle (0/1). Always 0 when the handle is
                 # closed. "Command update rate in stale frames" = rate in stale frames where this column is 1; If the
                 # arm is running, the solver is also running on those frames (cross-validated with duration_ms > 0).
                 # 'recession': STAGE hour [s] that ignited ISA. NULL = handle CLOSED (then the decision is made by the
                 # wall clock and the time is not calculated at all -- not a single microsecond is spent on the closed
                 # handle, see note on _state_machine). When exceeding miss_time_timeout_s (8), SCA occurs. It rewinds
                 # as it progresses.
                 'stagnation'])
        r_g = max(r, self.a.range_floor_m)
        los_az = -KDEG / r_g * u0[1] + d_ex
        los_el = -KDEG / r_g * u0[2] + d_ey
        s = info_value.get('cbf', (float('nan'),) * 4)
        self._diagnostic.writerow([
            f"{measurement.t:.4f}", f"{measurement.dt:.4f}", f"{measurement.bbox_age_s:.3f}",
            f"{ex:.3f}", f"{ey:.3f}", f"{eps:.3f}", f"{ey_ref:.3f}",
            f"{info_value.get('beta', float('nan')):.3f}",
            # band IN EFFECT (brake reduction APPLIED). Previously the constant was written as fov_alt_band_deg;
            # The test pilot needed this column to diagnose the crash (LOG BLINDNESS).
            f"{info_value.get('band_alt', float('nan')):.2f}",
            f"{r * math.sin(math.radians(eps)) if eps > 0 else 0.0:.1f}",
            f"{info_value.get('fov_free', 0)}", f"{info_value.get('empty_counter', 0)}",
            f"{r:.2f}",
            '' if measurement.range_m_value is None else f"{measurement.range_m_value:.2f}",
            '' if area_value is None else f"{area_value:.1f}",
            f"{area_rate:.2f}",
            f"{d_ex:.3f}", f"{d_ey:.3f}",
            f"{u0[0]:.3f}", f"{u0[1]:.3f}", f"{u0[2]:.3f}", f"{u0[3]:.2f}",
            f"{info_value.get('yaw_delta', float('nan')):.2f}",
            f"{los_az:.3f}", f"{los_el:.3f}",
            f"{v_ned[0]:.3f}", f"{v_ned[1]:.3f}", f"{v_ned[2]:.3f}",
            f"{self.pitch_lpf:.2f}",
            f"{s[0]:.2f}", f"{s[1]:.2f}", f"{s[2]:.1f}", f"{s[3]:.1f}",
            f"{info_value['duration_ms']:.3f}", info_value['iteration'],
            f"{info_value['cost']:.4f}",
            self.state_value,
            ('' if not math.isfinite(self.best_range)
             else f"{self.best_range:.2f}"),
            f"{self.range_rate_value:.2f}",
            f"{time.time():.3f}",
            f"{self.impact_blend:.3f}",
            f"{info_value.get('band_upper', float('nan')):.2f}",
            f"{info_value.get('acceleration_multiplier', float('nan')):.3f}",
            f"{info_value.get('alignment_ref', float('nan')):.3f}",
            1 if self.hit_value else 0,
            '' if measurement.vibe_max is None else f"{measurement.vibe_max:.1f}",
            int(info_value.get('budget_cut', 0)),
            f"{info_value.get('vertical_error', float('nan')):.3f}",
            # NaN = arm closed; write the empty string so that it does not get confused with "0" in CSV (0 cannot
            # be a real value of tau/t_go).
            _empty_nan(info_value.get('tgo', float('nan'))),
            _empty_nan(info_value.get('vertical_tau', float('nan'))),
            f"{self.disturbance.v_perpendicular:.3f}", f"{self.disturbance.a_perpendicular:.3f}",
            f"{info_value.get('apn_a', 0.0):.3f}",
            _empty_nan(info_value.get('tau_eff', float('nan')), '.3f'),
            _empty_nan(info_value.get('tau_eff_z', float('nan')), '.3f'),
            (1 if (math.hypot(u0[0], u0[1])
                   >= 0.99 * info_value.get('v_ceiling', self.a.speed_ceiling_mps))
             else 0),
            int(self.blind_pn_ran),
            (f"{self._stagnation_clock:.2f}" if self.a.progress_clock
             else "")])
        if self.counter % 20 == 0:
            self._diagnostic_f.flush()


# =============================================================== main

def main():
    p = argparse.ArgumentParser(description="MPC video guidance")
    p.add_argument('--duration-value', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--horizon', type=int, default=None,
                   help='predictive number of steps (default 24 )')
    p.add_argument('--step-s', type=float, default=None)
    p.add_argument('--sqp', type=int, default=None, help='SQP number of passes')
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw COMMAND (ablation: yaw remains on autopilot, '
                        '"Is a separate FOV controller necessary" experiment)')
    p.add_argument('--aim', type=float, default=None,
                   help='virtual gimbal aim offset (scenario.sh gives AIM=0)')
    p.add_argument('--mount', type=float, default=None,
                   help='camera mounting angle [deg, UP +]; if not given '
                        '$YILDIZ_MOUNT (standoff_geom.sh), otherwise 0')
    p.add_argument('--pitch-coupling', dest='pitch_coupling',
                   action='store_true', default=None,
                   help='camera axis FIXED TO BODY (default): climbing '
                        'and brake pitch terms are on the model')
    p.add_argument('--no-pitch-coupling', dest='pitch_coupling',
                   action='store_false',
                   help='camera axis GIMBALLE stabilized: pitch terms '
                        'lowered (real pitch-servo gimbal hardware)')
    p.add_argument('--q-acceleration', type=float, default=None,
                   help='acceleration/banking penalty (aggressiveness <-> framing protection)')
    p.add_argument('--speed-ceiling', type=float, default=None,
                   help='speed clamp [m/s]; if not given '
                        'guidance_config.VISUAL_MAX_SPEED_MPS')
    p.add_argument('--no-impact', action='store_true',
                   help='CLOSE STRIKE phase (ablation: at close range '
                        'cost remains nominal tracking cost)')
    p.add_argument('--no-miss', action='store_true',
                   help='CLOSE ISC state machine (ablation: obsolete '
                        'behavior -- continue issuing commands after switch)')
    p.add_argument('--miss-redis-key', default=None,
                   help='The key Redis to be written in the MISS announcement (hyperlink; '
                        'default is OFF, actual path Command.release flag)')
    p.add_argument('--diagnostic-log', default=None)
    args = p.parse_args()

    config_value = MpcConfig()
    if args.horizon is not None:
        config_value = MpcConfig(n_step=args.horizon,
                       blocks=_block_generate(args.horizon))
    if args.step_s is not None:
        config_value.step_s = args.step_s
    if args.sqp is not None:
        config_value.sqp_transition = args.sqp
    if args.no_yaw:
        config_value.yaw_command_provide = False
    if args.aim is not None:
        config_value.aim_deg = args.aim
    if args.mount is not None:
        config_value.mount_pitch_deg = args.mount
    if args.pitch_coupling is not None:
        config_value.pitch_coupling = args.pitch_coupling
    if args.q_acceleration is not None:
        config_value.q_acceleration = args.q_acceleration
    if args.speed_ceiling is not None:
        config_value.speed_ceiling_mps = args.speed_ceiling
    if args.no_impact:
        config_value.impact_mode = False
    if args.no_miss:
        config_value.miss_mode = False
    if args.miss_redis_key is not None:
        config_value.miss_redis_key = args.miss_redis_key
    print(f"[mpc] skid mode="
          f"{'ENABLED' if config_value.miss_mode else 'DISABLED'} "
          f"(terminal {config_value.terminal_range_m:.0f} m, arm "
          f"{config_value.miss_arm_m:.0f}/{config_value.miss_transition_arm_m:.0f}, opening "
          f"{config_value.miss_opening_m:.0f}/{config_value.miss_transition_opening_m:.0f}, "
          f"timeout {config_value.miss_time_timeout_s:.0f} s)")
    print(f"[mpc] speed cap={config_value.speed_ceiling_mps:.1f} m/s "
          f"(same source as guidance_config.VISUAL_MAX_SPEED_MPS), "
          f"beat phase={'ENABLED' if config_value.impact_mode else 'DISABLED'} "
          f"({config_value.impact_range_m:.0f} m -> {config_value.impact_full_range_m:.0f} m), "
          f"horizon={config_value.n_step * config_value.step_s:.2f} s")
    print(f"[mpc] assembly={config_value.mount_pitch_deg:+.2f} deg "
          f"(YILDIZ_MOUNT={os.environ.get('YILDIZ_MOUNT', '<absent>')}), "
          f"aim={config_value.aim_deg:+.2f}, pitch_coupling="
          f"{'ENABLED' if config_value.pitch_coupling else 'DISABLED (gimbal)'}, "
          f"vertical tape -{config_value.fov_upper_band_deg:.1f}..+"
          f"{config_value.fov_alt_band_deg:.1f} deg")
    # Let the A/B button be logged at the BEGINNING of the run: which arm is running can be read from the
    # first line, not from CSV later.
    print(f"[mpc] terminal vertical alignment="
          f"{'ENABLED' if config_value.vertical_terminal else 'DISABLED'} "
          f"(YILDIZ_VERTICAL_TERMINAL, {config_value.vertical_terminal_range_m:.0f} m -> "
          f"{config_value.vertical_terminal_full_m:.0f} m, deadband "
          f"+-{config_value.vertical_terminal_relaxed_deg:.1f} deg, tau "
          f"{config_value.vertical_terminal_tau_s:.1f} s)")
    print(f"[mpc] direct vertical error (P)="
          f"{'ENABLED' if config_value.vertical_error else 'DISABLED'} "
          f"(YILDIZ_VERTICAL_ERROR, q={config_value.q_vertical_error:.2f}"
          f"x{config_value.vertical_error_multiplier:.2f}, "
          f"{config_value.vertical_error_range_m:.0f} m [YILDIZ_VERTICAL_RAMP_START] -> "
          f"{config_value.vertical_error_full_m:.0f} m [YILDIZ_VERTICAL_RAMP_LAST], deadband "
          f"+-{config_value.vertical_error_relaxed_deg:.1f} deg)")
    print(f"[mpc] t_go shaped vertical tau="
          f"{'ENABLED' if config_value.vertical_tgo else 'DISABLED'} "
          f"(YILDIZ_VERTICAL_TGO, k={config_value.vertical_tgo_k:.2f}"
          f"x{config_value.vertical_tgo_multiplier:.2f}"
          f"={config_value.vertical_tgo_k * config_value.vertical_tgo_multiplier:.2f}, tau "
          f"{config_value.vertical_tgo_tau_min_s:.2f}..{config_value.vertical_tgo_tau_max_s:.2f} s, "
          f"closing door {config_value.vertical_tgo_closure_min_mps:.1f} m/s)")
    # WHICH COMBINATION IS RUNNING: since the three buttons are opened independently, the name of the
    # lever can be read in a single line (we label the A/B table from the first lines of the log, not from
    # CSV).
    branch = '+'.join([label_item for label_item, enabled_value in (
        ('D(TERMINAL)', config_value.vertical_terminal), ('P(ERROR)', config_value.vertical_error),
        ('TGO', config_value.vertical_tgo)) if enabled_value]) or 'NONE (baseline branch)'
    print(f"[mpc] VERTICAL CHANNEL ARM = {branch}")
    # APN and SATISFACTIONAL ACTIVE should also be written into the log. LOG BLIND FIX (2026-08-08):
    # banner line was not written when these two buttons were added; It could only be understood which arm
    # flew from the running folder by looking at the mpc_diagnostic columns (apn_a / tau_eff). Same discipline
    # as the vertical buttons above: keep the sleeve name on the first lines of the log.
    print(f"[mpc] APN (target lateral acceleration)="
          f"{'ENABLED' if config_value.apn else 'DISABLED'} "
          f"(YILDIZ_APN, multiplier {config_value.apn_multiplier:.2f}, tau "
          f"{config_value.apn_tau_s:.2f} s, deadband "
          f"{config_value.apn_dead_band_mps2:.2f}, ceiling "
          f"{config_value.apn_a_ceiling_mps2:.1f} m/s2)")
    print(f"[mpc] DOYUMLU ACTUATOR="
          f"{'ENABLED' if config_value.actuator else 'DISABLED'} "
          f"(YILDIZ_ACTUATOR, tau_lin {config_value.actuator_tau_lin_s:.2f} s "
          f"[YILDIZ_TAU_LIN], a_max {config_value.actuator_a_max_mps2:.2f} m/s2 "
          f"[YILDIZ_A_MAX]; VERTICAL a_max {config_value.actuator_a_max_z_mps2:.2f} "
          f"m/s2 [YILDIZ_A_MAX_Z], vertical tau_lin "
          f"{config_value.actuator_tau_lin_z_s:.2f} s [YILDIZ_TAU_LIN_Z])")
    print(f"[mpc] BLIND PN (maintain law in blind glide)="
          f"{'ENABLED' if config_value.blind_pn else 'DISABLED'} "
          f"(YILDIZ_BLIND_PN, maximum {config_value.blind_pn_maximum_s:.1f} s "
          f"[YILDIZ_BLIND_PN_MAXIMUM_S], my end tau {config_value.blind_pn_tau_s:.2f} s "
          f"[YILDIZ_BLIND_PN_TAU], |ex| share {config_value.blind_pn_ex_margin_deg:.1f} deg, "
          f"|ex| ABSOLUTE {config_value.blind_pn_ex_absolute_deg:.1f} deg "
          f"[YILDIZ_BLIND_PN_EX_ABSOLUTE], range gate "
          f"{config_value.blind_pn_range_m:.1f} m [YILDIZ_BLIND_PN_RANGE])")
    print(f"[mpc] PROGRESS CLOCK="
          f"{'ENABLED' if config_value.progress_clock else 'DISABLED'} "
          f"(YILDIZ_PROGRESS_CLOCK, base {config_value.miss_time_timeout_s:.0f} s, "
          f"switch-off threshold {config_value.progress_closure_threshold_mps:.1f} m/s, "
          f"new-min window {config_value.progress_window_s:.1f} s, "
          f"gain {config_value.progress_gain:.2f}, "
          f"absolute ceiling {config_value.progress_ceiling_s:.0f} s)")
    # SOLVER BUDGET should be logged at the beginning of the run: When interpreting the budget_cut rate
    # later, it should be known with which ceiling it is run.
    print(f"[mpc] solver budget="
          f"{'ABUNDANT' if config_value.iteration_ceiling > 26 else 'NORMAL'} "
          f"(YILDIZ_SOLVER_ABUNDANT, iteration ceiling {config_value.iteration_ceiling}, "
          f"duration {config_value.duration_budget_ms:.0f} ms; For SIM measurement quality, "
          f"hardware decision SEPARATE)")

    stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
    diagnostic = args.diagnostic_log or str(Path(__file__).resolve().parent / 'logs'
                                / f"mpc_diagnostic_{stamp_value}.csv")
    from visual_base import VisualLoop
    VisualLoop(MpcController(config_value, diagnostic_log=diagnostic),
                   loop_hz=args.loop_hz).run_value(args.duration_value)


def _block_generate(n):
    """Reasonable ascending block pattern for n steps (total = n)."""
    pattern_value = []
    remaining_value = n
    length = 1
    while remaining_value > 0:
        u = min(length, remaining_value)
        pattern_value.append(u)
        remaining_value -= u
        if len(pattern_value) % 2 == 0:
            length += 1
    return tuple(pattern_value)


if __name__ == '__main__':
    main()
