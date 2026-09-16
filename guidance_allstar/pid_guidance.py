#!/usr/bin/env python3
"""
pid_guidance.py - IMAGE guidance, PID method (PID branch of the LOS / PID / MPC half)
===========================================================================

*** FROZEN ARM (gimbal branch, 2026-08-05) *** This arm was written assuming a BODY-FIXED CAMERA and
will NOT work in the PHYSICAL GIMBAL branch: the "camera axis = mount + body pitch" assumption is
INVALID. The camera is now on a self-stabilizing single axis (tilt) gimbal; body pitch is not
reflected in the image (measured: body +-35 deg while camera max 0.65 deg) and the vertical axis CAN
NOW BE COMMANDED (YILDIZ_TILT = atan(down/back)). The vertical channel here should not be used
without re-derived diagnostics based on FOV bands and pitch. For rehabilitation: GIMBAL_NOTES.md

Fits on base visual_base.VisualController; The contract is defined there. Input is angular
errors from virtual gimbal (ex_deg, ey_deg), output is NED LINEAR SPEED + yaw_rate. Attitude is not
commanded.

BASIC IDEA ----------- The command vector is established on the MEASURED LOS AXIS, not on the BODY
axis:

    azimuth of forward axis = yaw + ex_deg

So "forward" always looks at the MEASURED horizontal bearing of the target; How far behind the yaw
loop is does not disrupt command direction. Reason: yaw_rate is clamped on both the autopilot and us
(+-60 deg/s) and in the past, combined with the constant-dt assumption, it was the source of the
jitter/giant circle. By REMOVING Yaw from the steering and leaving it solely to the FOV task (keep
the camera on target), the guidance becomes independent of the yaw performance. --govde-ileri
returns the flag to legacy (nose-pivoted) behavior; is a neat A/B button for the article.

Three channels + one FOV loop:

  1) LATERAL (speed to the right perpendicular to axis LOS) <- ex_deg PID 2) VERTICAL (speed down
  NED) <- ey_deg PID 3) FORWARD (along azimuth LOS) <- INCREASING from speed budget + range/commit
  ramp based on area_root 4) YAW (yaw_rate_dps) <- ex_deg PD+I for framing ONLY

GAIN TIMING WITH RANGE (the most valuable lesson of the legacy)
----------------------------------------------------------- measured on bumblebee/teva.py: if the
loop measures the error angle and gives the command METER, the fixed "m/degree" gain is wrong,
because 1 the meter equivalent of degrees grows with range. There correction was m_per_deg = R*tan(1
deg) and the measured result was: average vertical error 1.01 -> 0.73 m, saturation %19.4 -> %7.4.

Here we take the same idea one step further: the angle is first converted to METERS with a range,
then to m/s with a TIME CONSTANT:

    lateral_offset_m = R * tan(ex) # lateral deviation of the target from the LOS axis
    v_lateral_P     = lateral_offset_m / tau_lateral
    v_lateral_D = k_pn * R * lambda_dot # (rad/s) -> Speed ​​format of m/s, PN

The term D is deliberately "PN-like": R*lambda_dot is the apparent velocity of the target
perpendicular to LOS; Giving it that much lateral velocity means NOT turning the angle LOS, i.e.
collision course. k_pn ~ 1-2 is selected (corresponds to N). THIS is the term that enables corner
cutting (catching on ellipse/wanderer routes); pure pursuit (P) alone always gives tail chase and
does not reach the target of 18 m/s and 20 m/s.

lambda_dot is the INERTIA LOS speed, NOT the raw derivative of ex (ex is relative to the nose and
the cycle yaw resets it). Its derivation and delay mapping are in the code; The difference measured
in dry running is large (80 s ellipse, final range 317 m -> 36 m).

SPEED BUDGET (why is the forward axis calculated as "increasing")
---------------------------------------------------------------- Skeleton |v| If > V_MAX , it scales
the vector IN ONE PIECE; This does not distort the direction, but it plays forward windup
unpredictably and makes anti-windup undefined. So the budget is CLEARLY apportioned here: maneuver =
hypot( v_lateral , v_vertical ) -> clamp with maneuver_max first forward = min( forward_request ,
sqrt ( V_MAX ^ 2 - maneuver^ 2 )) Thus |v| it never hangs the ceiling, the frame's clamp never comes
into play and the centering channels always get their budget.

"Bold forward throttle": forward_request is ALWAYS the ceiling (V_MAX, guidance_config
VISUAL_MAX_SPEED_MPS). SIM MEASUREMENT (running 2): if the base is kept below the ceiling, the
range is permanently opened because the target is faster than us. There is NO slowdown -- what is
wanted is collision; The share of centering is already deducted from the maneuver budget.

MEASUREMENT QUALITY (small/noisy bbox and detection interruptions)
-------------------------------------------------------------- Measured on straight heading: bbox ~8
px on target tail image, detection %2.7. The D (PN) term in such a measurement is DANGEROUS because
the noise gain GROWS with range: v_lateral_D = k_pn * R * lambda_dot. 60 A center jitter of 0.2 deg at
m can produce ~4 m/s spurious lateral velocity -- right where the target looks most difficult. The P
term is almost unaffected by the same noise.
(R*tan(0.2 deg)/tau = 0.14 m/s), yani sorun tamamen DERIVATIVE kanalindadir.

A measurement-quality scalar q in [0,1] is therefore computed: q_area is low when area_root is small
(6 px -> 0, 20 px -> 1). q_age is low when bbox_age_s is large (0.15 s -> 1, 0.45 s -> floor).
    q = q_area * q_age
and is used in three places: 1. D terms SCALE with q (+ absolute clamp pn_max). 2. INCREASED by the
time constant of the derivative filter (1 + multiplier*(1-q)) -- more averaging as SNR decreases.
The lag price is acceptable: low quality already means long range, there is no need for terminal
sensitivity. 3. If q is below the threshold, the integrators are FREEZED (so that the stale/noisy
measurement does not accumulate permanent bias). P and the forward channel are NOT affected by q:
tracking and closing must continue at every quality, otherwise we will be left behind as soon as we
cannot see the target anyway. Yaw D is also unaffected (it is dumping; weakening it disrupts the end
of the loop).

VERTICAL CHANNEL: WHAT IS ey AND WHY DOES THE SETPOINT DEPEND ON RANGE
------------------------------------------------------
tools/scenario.sh fixes AIM=0. With aim=0, the virtual frame is aligned with the horizon.
The following relation comes from the yildizlar_gimbal.stabilize transformation:


        ey_deg = -(elevation angle of the target relative to the horizon)

So ey < 0 = target ABOVE US, ey = 0 = EQUAL ALTITUDE. Smoke testing confirmed this: at handoff ey
typically - 15 ..- 30 deg (geometry standoff back = 25 / down = 13 -> elevation 27.5 deg ).  Driving
ey to zero means "climb"; it is REQUIRED for combat but cannot be done IMMEDIATELY:

  CAMERA FRAME PHYSICAL LIMIT. Camera mounted fixed on the body +30 deg UP
  (models/swarm_drone_*/model.sdf). fx=fy=985.5, semi-vertical framing atan(360/985.5) = 20.1 deg.
  So visible bullish band [ (30 + body_pitch) - 20 , (30 + body_pitch) + 20 ]. In stationary
  tracking pitch ~-2.5 -> band [7.5, 47.5] deg. Equal altitude (ascent 0) is OUTSIDE this band;
  However, as the vehicle leans nose down at full throttle (pitch -15..-20), the band shifts to [-5,
  35] and the same altitude becomes visible. In other words, the command "climb immediately to the
  same altitude" risks throwing the target out of the BOTTOM EDGE of the frame.

So vertical SETPOINT IS TIMED WITH RANGE (--ey-hedef-uzak/-close): range >= 55 m -> ey_target = -18
deg (keep target up, frame safe, climb slowly) range <= 12 m -> ey_target = 0 between deg (equal
altitude = collision) -> linear 12 means 0 degree target remaining at m, 0 m vertical miss at 12 m;
At 25 m the setpoint becomes -5.4 deg which is at 25 m 2.4 m up, still in the center of the frame.
Thus, the vehicle ascends to the target altitude slowly from a distance and steadily from near.

DANGER: if the range is OPENING (target is running away) and the setpoint is at the same altitude,
the vertical channel may say "descend". There is an absolute altitude floor (--min-altitude, default 15
m): below which descent is prohibited + vertical integrator is frozen + soft ascent is commanded.

NOTE (if aim != 0 is used): The definition of ey shifts "relative to aim" and the remote value
ey_target must be set to zero (aim already carries the DC offset). The code is neutral in this
regard; the only thing to change is the two setpoint numbers.

ANTI-WINDUP (four layers) ------------- 1. DOOR: |error| > If int_gate_deg is NOT integrated
(detection at target frame edge / part -- outlier rejection of teva.py). 2. SATURATION: if the
relevant channel is in the clamp, it is absorbed by the integrator tau_aw (integral in teva.py *=
0.9; here dt-normalized). 3. LEAK: always slow leak (tau_leak). It also washes the handoff seed
over time, meaning the seed does not leave any permanent bias. 4. RE-LOCK: if there is a gap between
two command() calls (loss of bbox -- skeleton DOES NOT call us in those loops) integrators are
halved, derivative filters are reset; If the space is long, exactly zero.

handoff SEEDING (residue-based, SEPARATE from the integrator)
---------------------------------------- Classic "integral seeding" would be WRONG here: at handoff
time ex ~ 0 but ey ~ -20 is deg (part above), so the vertical P term is ALREADY large at handoff
time. Putting the seed into the integrator would overwhelm it.

Instead there is a separate SEED term that fades out with its own time constant: * handoff:
cmd_vel_ned is translated to the body axis by cmd_yaw_rad and stored (the forward component directly
seeds the forward-speed state). * In the FIRST command() call, the seed is set as = handoff_bileseni -
(P + I + D) AFTER P+I+D is calculated. So the first command is EXACTLY the last command of the
positioned -- no jump whatever the error. * In each subsequent step the seed is reset to zero with
tau_seed (2 s); PID takes over. Integrators start from scratch, meaning the seed leaves no
permanent bias and the anti-windup logic is not broken.

USAGE
--------
    python3 pid_guidance.py                    # with default earnings
    python3 pid_guidance.py --kp-yaw 2.0 --tau-yanal 2.0 --duration-value 300
    python3 pid_guidance.py --entries            # print earnings and exit
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import asdict, dataclass, field

import numpy as np

import guidance_config as cfg
from visual_base import (Command, VisualLoop, VisualController,
                             body_forward_ned)


# --------------------------------------------------------------------- settings

@dataclass
class PidConfig:
    """All gains and limits in one place. The note next to each value is the JUSTIFICATION."""

    # --- LATERAL channel (ex_deg -> LOS right speed perpendicular to axis) ---
    tau_lateral_s: float = 1.5
    # "In how many seconds to close the lateral offset of the target". R*tan(ex) offset in meters;
    # Dividing it by tau gives m/s. WHY 1.5 s AND NOT SHORTER (closed loop analysis): thanks to range
    # scaling the closed loop is RANGE INDEPENDENT first order -- d(e)/dt = -v/R = -e/tau. So tau is
    # directly the closed loop time constant. DEAD TIME budget in the loop: bbox/camera ~0.15 s + skeleton
    # command LPF 0.35 s + speed loop of autopilot ~0.4 s = ~0.9 s. Rough rule for first order + T dead
    # time tau >~ 2T; 1.5 s are right at this limit. The shorter tau brings back the past flicker/giant
    # circle root cause. The PRELIMINARY required for corner cutting is achieved by the term D (PN), not
    # by abbreviating tau.
    k_pn_horizontal: float = 1.8
    # Navigation constant of PN in speed form. v_lateral = k_pn * R * lambda_dot is the speed that does NOT
    # rotate the angle LOS; k_pn>1 allows corner cutting. In classic PN, N=3-5 is for acceleration; In
    # velocity form 1 already means "swallow angular velocity completely". 1.8 SELECTION (offline dry
    # running scan, 60-80 s ellipse 8 deg/s return, pessimistic model: vehicle speed delay 0.8 s + camera
    # 0.30 s): k_pn 1.2 -> final range 43 m, |ex| peak 4.2 deg k_pn 1.8 -> final range 36 m, |ex| peak 5.9
    # deg k_pn 2.5 -> final range 30 m, |ex| peak 6.8 deg k_pn 3.5 -> final range 30 m, |ex| peak 7.9 deg
    # (saturates, no gain) None have divergence/limit cycle. 1.8 takes most of the occlusion and leaves
    # the framing margin (horizontal half-frame 33 deg) wide. THIS IS THE FIRST SETTING BUTTON OF THE SIM
    # PHASE: if it cannot be caught, it first goes to 2.5.
    ki_lateral: float = 0.06          # (m/s) / (deg*s)
    # Turns off DC biases such as constant crosswind / constant rotation of the target. Minor: the
    # integrator is essentially the handoff SEED carrier in this problem.
    lateral_max: float = 10.0         # m/s
    # 18 m/s this is the most the budget can be pure lateral; The rest remains forward.

    # --- VERTICAL channel (ey_deg -> NED down speed) ---
    tau_vertical_s: float = 1.6
    # Slightly slower than horizontal: the vertical acceleration rating of the hopper (WPNAV_ACCEL_Z 250
    # cm/s^2) is lower than its horizontal rating; Also, on the vertical axis, the error is driven towards
    # the BOTTOM EDGE OF THE FRAME (camera +30 mounted), so the aggressiveness translates directly into
    # loss of detection here.
    k_pn_vertical: float = 1.0
    ki_vertical: float = 0.05          # (m/s) / (deg*s)
    climb_max: float = 8.0       # m/s (negative down = climb)
    descent_max: float = 4.5        # m/s (positive down = going down)
    # ASYMMETRIC and aligned with autopilot limits: guidance_config GUIDED_STARTUP_PARAM_ASSERTS ->
    # WPNAV_SPEED_UP 1000 cm/s (10 m/s), WPNAV_SPEED_DN 500 cm/s (5 m/s). Commanding a speed that the
    # autopilot cannot service effectively OPEN LOOP that channel of the loop; Staying slightly below the
    # ceiling prevents this. Narrower descent also ensures safety: in the escaping target the vertical
    # channel tends to become saturated downward (see altitude floor).

    # --- MANEUVER / FORWARD budget ---
    maneuver_max: float = 12.0       # m/s, hypot(lateral, vertical) ceiling
    # 18 m/s on budget 12 m/s if maneuver allowed forward axis at worst sqrt(18^2-12^2) = 13.4 remains
    # m/s. This means that ~%75 of the closing ability is preserved even at the harshest centering.
    v_forward_floor: float = float(getattr(cfg, "VISUAL_MAX_SPEED_MPS", 18.0))
    # m/s -- Read FROM THE SAME SOURCE AS THE CEILING (on purpose; they cannot be separated). SIM
    # MEASUREMENT (running 2, pid_ellipse_20260803_170119): command size after handoff remained at 16.1 m/s
    # while the base was 16 m/s; Since the target was 20 m/s, the range was opened to 12 29 m -> 95 m per
    # second and decreased to bbox 52 px -> 7 px. authority reverted to 'situated'. Lesson: THERE IS NO
    # WAY TO VOLUNTARILY CUT THE SPEED BUDGET IN THIS PROBLEM. The target is faster than us; Every m/s
    # below the ceiling is a permanent shutdown loss. The share required for centering is already deducted
    # from the maneuvering budget (below), there is no need for additional braking of the forward channel.
    # Since floor == ceiling, the "commit" ramp below is NON-EFFECTIVE BY DEFAULT; The parameter is
    # preserved for an experiment where intentional slowing is desired from a distance (e.g. framing
    # stability study).
    v_forward_ceiling: float = float(getattr(cfg, "VISUAL_MAX_SPEED_MPS", 18.0))
    forward_acceleration_mps2: float = 6.0
    # The slope limit of the forward speed state. Spreads the transition from handoff seed (positioner's
    # speed) to full throttle by ~0.5 s; It does not enter digits into the vehicle's interior position
    # controller along with command LPF (tau 0.35).

    # "Commit" ramp: move forward as you approach floor -> ceiling.
    range_far_m: float = 60.0     # in this range s=0 (base gas)
    range_near_m: float = 20.0    # in this range s=1 (full throttle)
    # SAME as 60 m handoff gate (bbox_to_redis decision maker: ~1.5 s framing + coverage >= %2 + estimator
    # range <= 60 m). So at handoff time we start from s=0; the ramp naturally aligns with the revs.
    area_far_px: float = 12.0      # area_root to this work s=0 (range ~60 m)
    area_near_px: float = 34.0     # area_root to this work s=1 (range ~20 m)
    # SIM CALIBRATION (running 2, pid_ellipse_20260803_170119, 799 paired sample): range band -> area_root
    # median 80-200 m : 6 50- 80 m : 9 35- 50 m : 18 25- 35 m : 26 15- 25 m : 36 These thresholds are
    # combined with the range ramp. I aligned: range_far=60 m -> area~12, range_near=20 m -> area~34.
    # The first provisional values ​​(8/25) gave s_area=1 even at range 40-50 m; measurement pulled this
    # to 18. CAUTION (still valid): bbox WIDTH IS NOT a range proxy -- depends on viewing angle. Measured:
    # bend 204 at m 31 px, standoff 26 at m behind median 34 px. area_root is SECONDARY evidence ONLY here;
    # the primary source is the estimator range (handoff gate <= 60 m guarantees freshness). Additionally,
    # since floor==ceiling, commit ramp is DISABLED by default; These thresholds are only meaningful for
    # diagnostic/log and experiments when v_forward_floor is manually lowered.

    # --- YAW / FOV loop ( ex_deg -> yaw_rate_dps ) ---
    kp_yaw: float = 2.5             # (deg/s) / deg  -> 1/s
    # Closed loop time constant ~1/2.5 = 0.4 s. The plant of the yaw loop is the pure integrator + ~0.3 s
    # autopilot delay; Since Kp*T = 0.75 < 1, the phase margin is reasonable. Higher Kp brings back the
    # past yaw flicker.
    ki_yaw: float = 0.5             # (deg/s) / (deg*s)
    # On a continuously rotating target (ellipse) P alone leaves a PERMANENT bearing error of
    # lambda_dot/Kp: 8 degrees at speed 20 deg/s LOS. i shuts this down in a few seconds; contribution is
    # limited to yaw_i_contribution_max.
    kd_yaw: float = 0.12            # (deg/s) / (deg/s) -> dimensionless
    yaw_rate_max: float = 60.0      # deg/s
    yaw_i_contribution_max: float = 18.0   # deg /s, ceiling of term I alone

    # --- ortak PID sertlestirmeleri ---
    dead_band_deg: float = 0.25
    # teva.py measurement: noise floor 25 0.019 deg below m, 50-100 0.164 deg at m. Our working band is
    # 20-60 m; 0.25 deg several times the noise but only 0.13 m in 30 m -- does not degrade collision
    # sensitivity. Applies to P and I, DOES NOT APPLY to D (dead band in D is phase loss).
    derivative_tau_s: float = 0.15
    # Derivative EMA time constant. teva.py alpha=0.05 @10 Hz (~1.9 s) was using; there Kd was very small.
    # In our case, D term is CORNER CUTTING term, 1.9 s delay makes it useless. 0.15 s ~ 30 Hz bbox frame
    # averaging ~4-5 in stream: absorbs pixel noise, not LOS dynamics.
    int_gate_deg: float = 10.0
    # |error| don't integrate while above that. Frame edge = possibly partial detection; Moreover, in this
    # region, P is already saturated and I can only swell. 10 deg was specifically chosen for the VERTICAL
    # channel: at the moment of handoff comes ey ~ -20 deg and this large angle error should not be fed to
    # the integrator (setpoint timed with the range reduces the error to ~-9 degrees, again at full
    # throttle climb If it exceeds 10, the integrator will shut down automatically).
    int_contribution_max: float = 3.0      # m/s, ceiling of lateral/vertical I term
    lambda_dot_max: float = 90.0    # deg/s, inertia LOS speed clamp

    # --- MEASUREMENT QUALITY (small/noisy bbox, detection interruption) --- Rationale and measurement in
    # the module docstring ("MEASUREMENT QUALITY" section).
    area_quality_alt_px: float = 6.0    # area_root below and q_area = 0
    area_quality_upper_px: float = 20.0   # q_area = 1 on this and above
    # CALIBRATION (running 2 , 799 paired example): area_root 6 -> 80 - 200 m, 9 -> 50 - 80 m, 18 -> 35 -
    # 50 m, 26 -> 25 - 35 m, 36 -> 15 - 25 m.  6 px = "target almost point" (tail image of straight route
    # falls here). UPPER VALUE SELECTION 20 was made by offline noise scanning (sigma= 0.10 deg center
    # jitter, area_root = 14 , detection rate % 80 ):
    #     top=14 -> lateral command jitter 1.261, peak 10.0 (SATURATED to lateral_max) top=20 -> jitter
    #     0.477, peak 7.11 <-- selected top=24 -> jitter 0.354, top 4.87 and corner cutoff remained THE
    #     SAME ON ALL THREE (ellipse dry run end range 35.8 m unchanged), so noise suppression came for
    #     free. It remained at 20 because with the actual calibration, q=1 in the terminal band (25-35 m,
    #     area_root~26): PN authority remains FULL where the collision will occur. 35-50 q=0.86 at m, 50-80
    #     q=0.21 at m -- naturally personified where the noise prevails.
    age_quality_upper_s: float = 0.15     # bbox_age below and q_age = 1
    age_quality_alt_s: float = 0.45     # q_age = base on this and above
    age_quality_floor: float = 0.15     # NOT exactly zero: keep some lead
    # The skeleton calls us up to bbox_stale_s = 0.7 s;  0.45 s was selected under it so that the "we are
    # called but the measurement is old" zone gradually ends.
    quality_derivative_multiplier: float = 2.0  # tau -> tau*(1+2) = 3 floor in q=0
    quality_int_threshold: float = 0.5      # q do not integrate under this
    pn_max_mps: float = 8.0            # ABSOLUTE clamp of term D [ m/s ]
    # q scaling is proportional; pn_max is the "no matter what" ceiling. Two-thirds of the maneuver budget
    # (12 m/s): term D alone cannot eat into the budget. NOT DROPPED (on purpose): pn_max 8 -> 6 -> 5
    # broke corner clipping (ellipse end range 35.8 -> 42.7 -> 46.8 m) but When area_upper=20 it contributed
    # NOTHING to noise. Q dampens the noise, not the clamp.
    camera_latency_s: float = 0.12
    # ex 's delay in the camera pipeline;  yaw delayed by the same amount lambda = ex + yaw is kept in
    # phase. POSSIBLE: bbox_to_redis Package ' tracker_bbox_stab ' contains t_capture ; Since the
    # measurement does not carry this, it is nominal value for now. (Skeleton modification to be
    # recommended: Measurement. measurement_yasi )
    aw_tau_s: float = 0.5
    # Integrator damping time constant at saturation (dt-normalized; constant-dt-free version of teva's
    # per frame *0.9 rule).
    leak_tau_s: float = 8.0
    # Slow leakage (leaky integrator) applied at each step. Prevents I from remaining silently during long
    # runs.
    seed_tau_s: float = 2.0
    # dampening time constant of the handoff seed. 2 s: after the skeleton's command LPF (0.35 s)
    # finishes, it takes ~3 tau = 6 s for PID to take over completely; Since the terminal engagement is
    # 5-15 s, this means the "first quarter" of the transition.

    # --- range timing ---
    range_default_m: float = 40.0
    # Nominal used if there is no estimator range (stale/first moments). handoff gate range opens at <= 60
    # m and standoff is 25 m; 40 is the middle of the m band.
    range_alt_m: float = 12.0      # clamp for gain timing
    range_upper_m: float = 80.0
    # Clamp CONDITION: gains are CORRECTLY proportional to R; A broken range (estimator jump) would be a
    # direct gain jump.

    # --- emniyet ---
    min_altitude_m_value: float = 15.0
    # Altitude base according to Home. Below descent prohibited + vertical I is frozen; As the violation
    # grows, gentle escalation is commanded. The justification is in the module docstring (constant LOS
    # angle when opening range = descent towards the ground).
    altitude_backward_thrust: float = 0.5   # ( m/s ) / m violation

    # --- VERTICAL SETPOINT: Relative to CAMERA AXIS (pitch-aware) ---
    camera_mounting_deg: float = field(
        default_factory=lambda: float(os.environ.get('YILDIZ_MOUNT', 0.0)))
    # $YILDIZ_MOUNT (exports scripts/standoff_geom.sh; must be the SAME as
    # models/swarm_drone_*/model.sdf), fallback 0.0. 2026-08-04: fixed was 30.0; When the sim mounting was
    # moved to 0 degrees (pitch-servo gimbal decision), this fixed vertical setpoint was shifted by 30
    # degrees. WARNING: The axis clamp [5,45] at _ey_target still sits on its base (5 deg) at mount 0 --
    # code PID is FROZEN in this round, see. MPC agent report.
    framing_weight_far: float = 1.0    # keep the distant target EXACTLY in camera axis
    framing_weight_near: float = 0.55  # release axis soon, close vertical abduction
    ey_target_far_m: float = 55.0
    ey_target_near_m: float = 12.0
    max_depth_m: float = 25.0        # Don't get over getting this far below the target.
    # WHY RELATIVE TO THE AXIS, NOT RELATIVE TO THE HORIZON (Caught in the yaw-lock run): In the previous
    # version, the setpoint was a fixed angle relative to the HORIZON (-18 deg). What determines staying in
    # the frame is the angle of the target relative to the CAMERA AXIS and the axis shifts with the body
    # pitch: axis_yukselisi = mount(+30) + body_pitch At nominal tracking pitch ~-2.5 -> axis was 27.5
    # and -18 deg setpoint 9.5 rating shared. But it CANNOT hold standoff positioned on a straight course
    # (target and hopper both 20 m/s): The median drifts behind 41.8 m instead of 25 m, the rise 27.5 ->
    # as the vertical offset remains constant 13 m 17.3 leans towards the degree, while pitch rises to
    # +6.8 and moves the axis to +36.8. The result: the target is 19.5 degrees BELOW the axis, right at
    # the edge of the +-20.1 vertical frame. The old setpoint (-12.5 @41.8 m) commands CLIMB in this
    # case; Climbing further reduces elevation, meaning it pushes the target OUT OF THE BOTTOM EDGE of the
    # frame -- the exact opposite direction. New setpoint: ey_target = -(mount + pitch) * weight. In the
    # same geometry -31.7 deg appears, that is, LOW is commanded; Descending increases depth, magnifying
    # elevation and bringing the subject back to the center of the frame. WHY WEIGHT 1 -> 0.55 RAMP: since
    # vertical miss = R*tan(rise), the same angle ALREADY means small miss as the range closes (27 degrees
    # at R=5 m only 2.5 m). So, forcing the same altitude (old close-three setpoint 0) is unnecessary;
    # Instead, we try to miss by leaving the axis a little bit nearby, but WE DON'T LOSE DETECTION --
    # staying in the frame was the main constraint in this problem (detection on a straight route was
    # %2.7).

    again_lock_s: float = 0.6
    # If the gap between command() calls exceeds this, it is considered a lock again.
    full_zero_s: float = 2.0        # After all this space, reset to full zero.

    # --- yapisal secenekler ---
    body_forward: bool = False
    # False (default): forward axis yaw+ex (Measured azimuth of LOS). True: old behavior, forward axis
    # nose. Ablation button for article.


# ------------------------------------------------------------- yardimci parcalar

def _clamp_value(x, alt, upper_value):
    return alt if x < alt else (upper_value if x > upper_value else x)


def _dead_band(e, band_value):
    """Applies it by REMOVING the deadband (creating no bounce): |e|<band -> 0."""
    if e is None:
        return 0.0
    if abs(e) <= band_value:
        return 0.0
    return e - math.copysign(band_value, e)


class DerivativeFilterValue:
    """Derivative filter with protection against ZOH repetitions, working with measured dt.

    WHY SPECIAL: stream bbox 15-30 Hz, loop 20 Hz. When the same bbox is read twice, the raw
    derivative is ZERO, BUT VERY LARGE in the next new frame; This comb turns the D term into noise.
    Solution: differentiate only WHEN THE VALUE CHANGES and the ACTUAL elapsed time of that change.

    Also in the first example the derivative returns 0 (lecture teva.py: pseudo derivatives
    1127-2817 deg/s were measured in the first frame).
    """

    def __init__(self, tau_s, max_wait_s=0.35, min_dt_s=0.02):
        self.tau = float(tau_s)
        self.max_wait = float(max_wait_s)
        self.min_dt = float(min_dt_s)
        self.reset_value()

    def reset_value(self):
        self._previous_value = None
        self._previous_t = None
        self._filtered = 0.0

    def update_value(self, value_value, t, dt, tau=None):
        """value: raw error [deg], t: monotonic [s], dt: MEASURED cycle step.

        tau: time-constant override for this step. As the measurement quality decreases (small bbox /
        stale detection) it is ENLARGED externally: as the signal-to-noise ratio worsens, more
        averaging is required. The lag price is acceptable because low quality already means long
        range and there is no need for terminal sensitivity."""
        if value_value is None:
            return self._filtered
        if self._previous_value is None:
            self._previous_value, self._previous_t = float(value_value), float(t)
            return 0.0
        elapsed_item = float(t) - self._previous_t
        if value_value == self._previous_value and elapsed_item < self.max_wait:
            # Same bbox read again: derivative REFRESH, hold.
            return self._filtered
        raw_value = (float(value_value) - self._previous_value) / max(elapsed_item, self.min_dt)
        self._previous_value, self._previous_t = float(value_value), float(t)
        tau_e = self.tau if tau is None else float(tau)
        a = dt / (dt + tau_e) if tau_e > 1e-6 else 1.0
        self._filtered += a * (raw_value - self._filtered)
        return self._filtered


class Integrator:
    """Integrator with gate + saturation damping + leakage."""

    def __init__(self, ki, contribution_max, gate_deg_value, aw_tau_s, leak_tau_s):
        self.ki = float(ki)
        self.contribution_max = float(contribution_max)
        self.gate_value = float(gate_deg_value)
        self.aw_tau = float(aw_tau_s)
        self.leak_tau = float(leak_tau_s)
        self.value_value = 0.0            # accumulated error [deg*s]

    @property
    def _value_max(self):
        return (self.contribution_max / self.ki) if self.ki > 1e-9 else 0.0

    def reset_value(self):
        self.value_value = 0.0

    def attenuate(self, ratio_value):
        self.value_value *= float(ratio_value)

    def step_value(self, error_deg, dt, saturated, rotate=False):
        """Advance the integrator and return CONTRIBUTION [m/s]."""
        # 3. layer: always leak (also washes the handoff seed).
        self.value_value *= max(0.0, 1.0 - dt / max(self.leak_tau, 1e-6))
        if saturated:
            # 2 . layer: channel in clamp -> fast damping.
            self.value_value *= max(0.0, 1.0 - dt / max(self.aw_tau, 1e-6))
        elif not rotate and error_deg is not None and abs(error_deg) <= self.gate_value:
            # 1. layer: door. No accumulation at the edge of the frame / when frozen.
            self.value_value += error_deg * dt
        self.value_value = _clamp_value(self.value_value, -self._value_max, self._value_max)
        return self.ki * self.value_value


# ------------------------------------------------------------------ controller

class PidController(VisualController):
    """PID visual servo: ex/ey -> speed in frame LOS + yaw_rate."""

    label_item = "pid"

    def __init__(self, config_value: PidConfig | None = None):
        self.a = config_value or PidConfig()
        a = self.a
        self.i_lateral = Integrator(a.ki_lateral, a.int_contribution_max, a.int_gate_deg,
                                  a.aw_tau_s, a.leak_tau_s)
        self.i_vertical = Integrator(a.ki_vertical, a.int_contribution_max, a.int_gate_deg,
                                  a.aw_tau_s, a.leak_tau_s)
        # The "contribution" of the yaw integrator is in deg/s; The same class is used.
        self.i_yaw = Integrator(a.ki_yaw, a.yaw_i_contribution_max, a.int_gate_deg,
                                a.aw_tau_s, a.leak_tau_s)
        self.d_ex = DerivativeFilterValue(a.derivative_tau_s)
        self.d_ey = DerivativeFilterValue(a.derivative_tau_s)
        self.d_lambda = DerivativeFilterValue(a.derivative_tau_s)  # ATALET LOS kerterizi
        self._yaw_enabled = None                      # sarilmasi cozulmus yaw [deg]
        self._yaw_buffer = []                      # [(t, yaw_enabled)] for delay
        self.v_forward = a.v_forward_floor       # forward speed status (slope limited)
        self.seed_right = 0.0                 # handoff seed (SEPARATE from PID)
        self.seed_down = 0.0
        self._seed_request = None            # converted to residue at first command()
        self._last_t_value = None
        self._last_yaw = None
        self._last_range = a.range_default_m
        self._saturated_lateral = False
        self._saturated_vertical = False
        self._saturated_yaw = False
        self.diagnostic = {}                       # son dongunun ic terimleri (log/test)

    # -- handoff ---------------------------------------------------------------

    def seed_value2(self, handoff):
        """Convert the last command of the positioner to the body axis and save it as SEED.

        The seed is NOT written to the integrator: at the time of handoff the vertical error is
        already large (ey ~ -20 deg), so the P term is not zero and putting the seed into the
        integrator would overlap it. The seed is installed as the RESIDUE "handoff_command - (P+I+D)"
        in the first command() call and is sonified with tau_seed (justification is in the module
        docstring)."""
        self.i_lateral.reset_value()
        self.i_vertical.reset_value()
        self.i_yaw.reset_value()
        self.d_ex.reset_value()
        self.d_ey.reset_value()
        self.d_lambda.reset_value()
        self._yaw_enabled = None
        self._yaw_buffer.clear()
        self._last_t_value = None
        self.v_forward = self.a.v_forward_floor
        self.seed_right = 0.0
        self.seed_down = 0.0
        self._seed_request = None
        if not handoff:
            return
        try:
            v = np.asarray(handoff.get('cmd_vel_ned'), dtype=float).reshape(3)
        except Exception:
            v = None
        yaw = handoff.get('cmd_yaw_rad')
        if v is not None and yaw is not None:
            c, s = math.cos(float(yaw)), math.sin(float(yaw))
            forward = c * v[0] + s * v[1]
            right_value = -s * v[0] + c * v[1]
            down_value = float(v[2])
            # Forward: seed as state (slope boundary ramps from here).
            self.v_forward = _clamp_value(forward, 0.0, self.a.v_forward_ceiling)
            self._seed_request = (right_value, down_value)
            self._last_yaw = float(yaw)
        r = handoff.get('range_m')
        try:
            if r is not None and math.isfinite(float(r)) and float(r) > 0.0:
                self._last_range = float(r)
        except (TypeError, ValueError):
            pass

    # -- ic hesap (test edilebilir) ---------------------------------------

    def _range_timing(self, measurement):
        """Range [m] (clamped) to use for gain timing."""
        m = measurement.range_m_value
        if m is not None and math.isfinite(m) and m > 0.0:
            self._last_range = float(m)
        return _clamp_value(self._last_range, self.a.range_alt_m,
                        self.a.range_upper_m)

    def _commit(self, measurement):
        """Forward throttle ramp [ 0 , 1 ]: full throttle as you approach.

        PRIMARY source estimator range (handoff gate <= 60 m guarantees freshness). area_root
        SECONDARY: bbox CANNOT be used as a range proxy as its size depends on angle of view, it
        just carries the "too close" evidence when range gets stale."""
        a = self.a
        s_range = 0.0
        m = measurement.range_m_value
        if m is not None and math.isfinite(m) and m > 0.0:
            width_value = max(1e-6, a.range_far_m - a.range_near_m)
            s_range = _clamp_value((a.range_far_m - m) / width_value, 0.0, 1.0)
        s_area = 0.0
        ak = measurement.area_root
        if ak is not None and math.isfinite(ak) and ak > 0.0:
            width_value = max(1e-6, a.area_near_px - a.area_far_px)
            s_area = _clamp_value((ak - a.area_far_px) / width_value, 0.0, 1.0)
        # max: two independent evidence; If one is missing (None) the other carries it.
        return max(s_range, s_area)

    def _quality(self, measurement):
        """Measurement quality q in [0,1] (small bbox / stale detection -> low).

        It only affects the DERIVATIVE (PN) channel and the integrator gate; P and forward channels
        work fully at any quality. The justification is in the module docstring."""
        a = self.a
        ak = measurement.area_root
        if ak is None or not math.isfinite(ak):
            q_area = 1.0        # This field is filled only for 'fresh' calls
        else:
            width_value = max(1e-6, a.area_quality_upper_px - a.area_quality_alt_px)
            q_area = _clamp_value((ak - a.area_quality_alt_px) / width_value, 0.0, 1.0)
        age_value = measurement.bbox_age_s
        if age_value is None or not math.isfinite(age_value):
            q_age = a.age_quality_floor
        else:
            width_value = max(1e-6, a.age_quality_alt_s - a.age_quality_upper_s)
            fresh_value = _clamp_value((a.age_quality_alt_s - age_value) / width_value, 0.0, 1.0)
            q_age = a.age_quality_floor + (1.0 - a.age_quality_floor) * fresh_value
        return q_area * q_age

    def _ey_target(self, r, pitch_deg):
        """VERTICAL setpoint [deg], timed with range, relative to CAMERA AXIS.

        axis_yukselisi = assembly + body_pitch; setpoint is a fraction of that. At a distance, the
        full axis (maximum frame allowance) is achieved, at a distance the fraction is reduced and
        vertical abduction is performed. Justification is in PidConfig."""
        a = self.a
        width_value = max(1e-6, a.ey_target_far_m - a.ey_target_near_m)
        u = _clamp_value((r - a.ey_target_near_m) / width_value, 0.0, 1.0)
        weight_value = a.framing_weight_near + (a.framing_weight_far
                                            - a.framing_weight_near) * u
        # The axis is kept within a reasonable range: do not blow the damaged/excessive pitch setpoint (the
        # body may temporarily find pitch 40 degrees).
        axis_value = _clamp_value(a.camera_mounting_deg + pitch_deg, 5.0, 45.0)
        return -axis_value * weight_value

    def body_request(self, measurement):
        """Generate (forward, right, down) + yaw_rate in frame LOS.

        The ENTIRE speed command occurs here; command() just translates to NED. The reason it stands
        apart is testability: unit tests validate the sign/direction from this dictionary, they
        don't need MAVLink or Redis."""
        a = self.a
        dt = max(float(measurement.dt), 1e-3)

        # --- relock (bbox loss): skeleton does not call us on loss loops, so the gap is measured HERE (4.
        # anti-windup layer).
        if self._last_t_value is not None:
            gap_value = float(measurement.t) - self._last_t_value
            if gap_value > a.full_zero_s:
                self.i_lateral.reset_value()
                self.i_vertical.reset_value()
                self.i_yaw.reset_value()
                self.d_ex.reset_value()
                self.d_ey.reset_value()
                self.d_lambda.reset_value()
                self._yaw_enabled = None
                self._yaw_buffer.clear()
                self.seed_right = self.seed_down = 0.0
            elif gap_value > a.again_lock_s:
                self.i_lateral.attenuate(0.5)
                self.i_vertical.attenuate(0.5)
                self.i_yaw.attenuate(0.5)
                self.d_ex.reset_value()
                self.d_ey.reset_value()
                self.d_lambda.reset_value()
                self._yaw_enabled = None
                self._yaw_buffer.clear()
        self._last_t_value = float(measurement.t)

        ex = 0.0 if measurement.ex_deg is None else float(measurement.ex_deg)
        ey_raw = 0.0 if measurement.ey_deg is None else float(measurement.ey_deg)
        r = self._range_timing(measurement)
        pitch_deg = (0.0 if measurement.pitch_rad is None
                     else math.degrees(float(measurement.pitch_rad)))
        ey_target = self._ey_target(r, pitch_deg)
        ey = ey_raw - ey_target

        # --- MEASUREMENT QUALITY: small/stale bbox -> winter derivative channel ---
        q = self._quality(measurement)
        # The lower the SNR, the more average (the lag cost is knowingly accepted).
        derivative_tau = a.derivative_tau_s * (1.0 + a.quality_derivative_multiplier * (1.0 - q))

        # --- derivatives (measured dt, ZOH protected) ---
        ex_dot = self.d_ex.update_value(measurement.ex_deg, measurement.t, dt, derivative_tau)
        ey_dot = self.d_ey.update_value(measurement.ey_deg, measurement.t, dt, derivative_tau)

        # INERTIA LOS ANGLE SPEED (ERROR caught in dry run, 'ellipse' heading): ex = lambda - yaw (lambda =
        # INERTIAL bearing, ex = relative to NOSE) => d(ex)/dt = lambda_dot - yaw_dot Since the yaw cycle
        # keeps ex at zero, IN STABLE ROTATION becomes ex_dot ~ 0 and the term PN BECOME. The size that
        # enables corner cutting is the INERTIA LOS speed. Dry run (80 s, ellipse, 8 deg/s turn) final range:
        # 36 m instead of 317 m with ex_dot -- the revival of the term is decisive in itself.
        #
        # DELAY MATCHING: ex comes from the camera pipeline with a DELAY of ~0.1-0.2 s, while yaw is fresh
        # from ATTITUDE. If the two are added together in their raw form, a FAKE lambda_dot is born in each
        # yaw transient, and that fake value (because it is multiplied by k_pn * R) appears large in the
        # lateral channel -- meaning an artificial feedback is established between the yaw loop and the
        # lateral channel. Therefore yaw is DELAYED BY THE SAME TIME and lambda = ex + yaw_gecikmeli is
        # differentiated as a SINGLE signal (single filter, consistent phase). Correction is NOT required in
        # the VERTICAL channel: ey_dot is ALREADY the inertial LOS ascent rate, as ey is read aligned to the
        # horizon in the virtual gimbal.
        yaw_late = None
        if measurement.yaw_rad is not None:
            raw_value = math.degrees(float(measurement.yaw_rad))
            if self._yaw_enabled is None:
                self._yaw_enabled = raw_value
            else:
                self._yaw_enabled += ((raw_value - self._yaw_enabled + 180.0) % 360.0) - 180.0
            self._yaw_buffer.append((float(measurement.t), self._yaw_enabled))
            target_t = float(measurement.t) - a.camera_latency_s
            while len(self._yaw_buffer) > 1 and self._yaw_buffer[1][0] <= target_t:
                self._yaw_buffer.pop(0)
            if len(self._yaw_buffer) > 200:        # Don't let the buffer grow unlimited
                del self._yaw_buffer[:-200]
            yaw_late = self._yaw_buffer[0][1]
        if yaw_late is None:
            lambda_dot_raw = ex_dot                # No yaw: that's what's available
        else:
            lambda_dot_raw = self.d_lambda.update_value(ex + yaw_late, measurement.t,
                                                    dt, derivative_tau)
        # CLAMP: lambda_dot enters the lateral command multiplied by k_pn*R, meaning an ATTITUDE jump / hug
        # error can suddenly call for a huge lateral speed. Physical upper limit: our own ceiling yaw_rate 60
        # deg/s, speed LOS the target can generate at close range ~40/12 rad/s. 90 deg/s is above these but
        # well below the noise jumps.
        lambda_dot = _clamp_value(lambda_dot_raw,
                              -a.lambda_dot_max, a.lambda_dot_max)  # deg/s

        # --- LATERAL channel ---
        ex_db = _dead_band(ex, a.dead_band_deg)
        # Convert angular error to meters using range (lesson teva.py m_per_deg), then to m/s with time constant. tan():
        # gives the correct offset even at large angles.
        lateral_p = r * math.tan(math.radians(ex_db)) / max(a.tau_lateral_s, 1e-3)
        # Velocity form of PN: R * lambda_dot [m/s]. lambda_dot is the inertial LOS angular rate
        # (ex_dot + yaw_dot); see the rationale above.
        lateral_d = _clamp_value(q * a.k_pn_horizontal * r * math.radians(lambda_dot),
                           -a.pn_max_mps, a.pn_max_mps)
        low_quality = q < a.quality_int_threshold
        lateral_i = self.i_lateral.step_value(ex_db, dt, self._saturated_lateral,
                                    rotate=low_quality)

        # --- VERTICAL channel ---
        ey_db = _dead_band(ey, a.dead_band_deg)
        # Descent AUTHORITY (replaced the old "reverse direction lock"). The old lock said "never go down when
        # the target is up "; the leak in the dry run prevented the dive, but the yaw -lock run showed that
        # the CORRECT move on a flat course is precisely to DESCEND (when increasing separation reduces the elevation angle, the target
        # escapes to the lower edge of the frame; descending brings it back to the center). Because the
        # setpoint is now relative to the camera axis, the descent is self-LIMITED -- once the target elevation angle reaches
        # the setpoint, the error closes. Only two absolute safeties remain: 1 . altitude floor (
        # min_altitude_m_value ) 2 . DEPTH ceiling below the target: depth = R* sin (- ey ). If the range was
        # increasing, a fixed elevation angle would make depth grow with R; this ceiling limits that growth.
        # (Does not use anything other than R and camera angle -- range rule is preserved.)
        altitude_value = None
        if measurement.pos_ned is not None:
            altitude_value = -float(measurement.pos_ned[2])       # NED z down positive
        floor_violation = (altitude_value is not None and altitude_value < a.min_altitude_m_value)
        # With RAW range (self._last_range), NOT with the clamped r of the gain timing: r was clipped to
        # [12,80] so it showed depth at far range to be LOWER and the ceiling was late to kick in (measured on
        # a dry run: down to 35 m instead of 25 m on flat course).
        depth_m = self._last_range * math.sin(math.radians(max(0.0, -ey_raw)))
        depth_exceeded = depth_m > a.max_depth_m
        # ey_db > 0 = DESCENT request. If one of the two absolute safeties is active, the request is reset (it
        # does not enter the integrator -> it does not inflate).
        if ey_db > 0.0 and (floor_violation or depth_exceeded):
            ey_db = 0.0
        vertical_p = r * math.tan(math.radians(ey_db)) / max(a.tau_vertical_s, 1e-3)
        vertical_d = _clamp_value(q * a.k_pn_vertical * r * math.radians(ey_dot),
                           -a.pn_max_mps, a.pn_max_mps)
        # FREEZING the integrator in case of depth/base violation is NOT ENOUGH: previously accumulated
        # descent authority leaves a long tail (measured: descended to 35.9 m even though the ceiling was
        # triggered at 25 m). It is discharged from the saturation path by ACTIVE absorption (aw_tau).
        vertical_i = self.i_vertical.step_value(
            ey_db, dt, self._saturated_vertical or depth_exceeded or floor_violation,
            rotate=floor_violation or low_quality or depth_exceeded)

        # --- handoff SEED: Separate from PID, relic-based, with its own tau ---
        if self._seed_request is not None:
            # First command: seed = last command of positional - current output of PID => FIRST command becomes
            # exactly the last command of positioned (without jumping).
            target_right, target_down = self._seed_request
            self._seed_request = None
            self.seed_right = _clamp_value(target_right - (lateral_p + lateral_d + lateral_i),
                                      -a.maneuver_max, a.maneuver_max)
            self.seed_down = _clamp_value(target_down - (vertical_p + vertical_d + vertical_i),
                                        -a.maneuver_max, a.maneuver_max)
        else:
            last_value = max(0.0, 1.0 - dt / max(a.seed_tau_s, 1e-6))
            self.seed_right *= last_value
            self.seed_down *= last_value

        v_right_raw = lateral_p + lateral_d + lateral_i + self.seed_right
        v_right = _clamp_value(v_right_raw, -a.lateral_max, a.lateral_max)
        v_down_raw = vertical_p + vertical_d + vertical_i + self.seed_down
        v_down = _clamp_value(v_down_raw, -a.climb_max, a.descent_max)
        if depth_exceeded:
            # Depth ceiling: the inlet gate alone is not enough, the outlet is also clamped -- so that the
            # seed/integrator/D residue does not continue to lower.
            v_down = min(v_down, 0.0)
        if floor_violation:
            # Soft pushback: forced escalation as the breach escalates.
            backward = -(a.min_altitude_m_value - altitude_value) * a.altitude_backward_thrust
            v_down = min(v_down, backward)

        # --- maneuver budget ---
        maneuver = math.hypot(v_right, v_down)
        budget_saturated = maneuver > a.maneuver_max
        if budget_saturated and maneuver > 1e-9:
            scale_value = a.maneuver_max / maneuver
            v_right *= scale_value
            v_down *= scale_value
            maneuver = a.maneuver_max
        # Anti-windup gate of the next step (conditional integration, 1 step delayed -- standard method).
        self._saturated_lateral = budget_saturated or abs(v_right_raw) > a.lateral_max
        self._saturated_vertical = (budget_saturated
                              or v_down_raw > a.descent_max
                              or v_down_raw < -a.climb_max)

        # --- FORWARD channel: INCREASING in budget + commit ramp ---
        s = self._commit(measurement)
        forward_request = a.v_forward_floor + (a.v_forward_ceiling - a.v_forward_floor) * s
        remaining_value = math.sqrt(max(0.0, a.v_forward_ceiling ** 2 - maneuver ** 2))
        forward_target = min(forward_request, remaining_value)
        # Slope limit: ramp from handoff seed (or previous value).
        step_max = a.forward_acceleration_mps2 * dt
        self.v_forward += _clamp_value(forward_target - self.v_forward, -step_max, step_max)
        self.v_forward = _clamp_value(self.v_forward, 0.0, remaining_value)

        # --- YAW / FOV cycle (INDEPENDENT from steering) ---
        yaw_p = a.kp_yaw * ex_db
        yaw_d = a.kd_yaw * ex_dot
        yaw_i = self.i_yaw.step_value(ex_db, dt, self._saturated_yaw,
                                rotate=low_quality)
        yaw_raw = yaw_p + yaw_d + yaw_i
        yaw_rate = _clamp_value(yaw_raw, -a.yaw_rate_max, a.yaw_rate_max)
        self._saturated_yaw = abs(yaw_raw) > a.yaw_rate_max

        self.diagnostic = {
            'r': r, 's_commit': s, 'ex': ex, 'ey': ey,
            'ey_raw': ey_raw, 'ey_target': ey_target,
            'seed_right': self.seed_right, 'seed_down': self.seed_down,
            'ex_dot': ex_dot, 'ey_dot': ey_dot,
            'lambda_dot': lambda_dot, 'q': q, 'derivative_tau': derivative_tau,
            'lateral_p': lateral_p, 'lateral_d': lateral_d, 'lateral_i': lateral_i,
            'vertical_p': vertical_p, 'vertical_d': vertical_d, 'vertical_i': vertical_i,
            'yaw_p': yaw_p, 'yaw_d': yaw_d, 'yaw_i': yaw_i,
            'maneuver': maneuver, 'remaining_value': remaining_value, 'altitude_value': altitude_value,
            'floor_violation': floor_violation, 'depth_m': depth_m,
            'depth_exceeded': depth_exceeded, 'pitch_deg': pitch_deg,
        }
        return {'forward': self.v_forward, 'right_value': v_right, 'down_value': v_down,
                'yaw_rate_dps': yaw_rate, 'ex_deg': ex}

    # -- sozlesme ---------------------------------------------------------

    def command_value(self, measurement):
        request_value = self.body_request(measurement)
        yaw = measurement.yaw_rad
        if yaw is None:
            yaw = self._last_yaw
        if yaw is None:
            # If the Attitude never came, it cannot be converted to NED. With zero command; The skeleton LPF
            # breathes smoothly from full speed, the decision maker returns to 'position' in no time.
            return Command(vel_ned=np.zeros(3), yaw_rate_dps=None)
        self._last_yaw = float(yaw)
        # Forward axis: MEASURED LOS azimuth (yaw + ex). Nose with --govde-ileri.
        axis_value = float(yaw) if self.a.body_forward else \
            float(yaw) + math.radians(request_value['ex_deg'])
        v = body_forward_ned(axis_value, request_value['forward'], request_value['right_value'], request_value['down_value'])
        return Command(vel_ned=v, yaw_rate_dps=request_value['yaw_rate_dps'])


# ----------------------------------------------------------------------- CLI

def config_argparse(p: argparse.ArgumentParser):
    """PidConfig alanlarini --kebab-case bayraklarina acar."""
    var = PidConfig()
    for label_item, value_value in asdict(var).items():
        flag = '--' + label_item.replace('_', '-')
        if isinstance(value_value, bool):
            p.add_argument(flag, action='store_true', default=value_value)
        else:
            p.add_argument(flag, type=type(value_value), default=value_value)
    return p


def config_generate(args) -> PidConfig:
    fields_value = asdict(PidConfig())
    return PidConfig(**{label_item: getattr(args, label_item) for label_item in fields_value})


def main():
    p = argparse.ArgumentParser(
        description="PID visual guidance (on visual_base frame)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--duration-value', type=float, default=None,
                   help='stop after seconds (None = WAIT WAIT)')
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--tau-lpf', type=float, default=0.35,
                   help='skeleton instruction LPF time constant [s]')
    p.add_argument('--log', default=None, help='CSV log path')
    p.add_argument('--entries', action='store_true',
                   help='print earnings and exit (sim does not initialize)')
    config_argparse(p)
    args = p.parse_args()

    config_value = config_generate(args)
    if args.entries:
        print("PID settings:")
        for label_item, value_value in asdict(config_value).items():
            print(f"  {label_item:24s} = {value_value}")
        return

    VisualLoop(PidController(config_value), loop_hz=args.loop_hz,
                   tau_s=args.tau_lpf, log_path=args.log).run_value(args.duration_value)


if __name__ == '__main__':
    main()
