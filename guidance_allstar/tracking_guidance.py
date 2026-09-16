#!/usr/bin/env python3
"""
tracking_guidance.py - Video guidance with ARDUPILOT "FOLLOW" LAW
============================================================ ALTERNATIVE arm to MPC. Same skeleton
(visual_base.VisualLoop), same contract (ex/ey from virtual gimbal, range ONLY from target,
command = speed NED), same MISS/HIT state machine -- ONLY THING CHANGED GUIDANCE LAW: instead of
optimization ArduPilot's own tracking law.

--------------------------------------------------------------------- ORIGIN The law is derived
EXACTLY from two files in the repository ArduPilot (local copy $ARDUPILOT_DIR, 2025-07-13):

  ArduCopter/mode_follow.cpp Rate law of FOLLOW mode libraries/AP_Follow/AP_Follow.cpp target
  estimation + offset (FOLL_* param) libraries/AC_Avoidance/AC_Avoid.cpp::limit_velocity_2D /
  get_max_speed libraries/AP_Math/control.cpp::sqrt_controller

Kernel ModeFollow::run() in version Copter-4.4 ("classic" law in this file):

    desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P
    |v_xy| <= WPNAV_SPEED ;  v_z in [-WPNAV_SPEED_DN, WPNAV_SPEED_UP]
    limit_velocity_2D(PSC_POSXY_P, WPNAV_ACCEL/2, ...) # slow down when approaching the target
    v_z    <= get_max_speed(PSC_POSZ_P, WPNAV_ACCEL_Z/2, |dz|)
    yaw = target bearing # FOLL_YAW_BEHAVE=0

Copter-4.5+ does the same job as pos_control->input_pos_vel_accel_NE; the inner loop there is
AC_P_2D::update_all = sqrt_controller(pos_error, PSC_POSXY_P, acceleration_ceiling, dt). In this file, that
arm is named "poscon" (--law poscon).

------------------------------------------------------------------ FOUR MANDATORY ADAPTATIONS
ArduPilot FOLLOW mode attempts to "stay next to the target"; we are trying to CRASH and can't use
much of the target telemetry. We split in three places, all three are CONSCIOUS and can be undone in
one line:

(1) WHERE IS THE TARGET? AP_Follow inherits the target from MAVLink GLOBAL_POSITION_INT. In our
case, it is FORBIDDEN (project rule: only RANGE from target telemetry). Use IMAGE + RANGE instead:
eps = -(ey + aim) elevation of the target relative to the horizon [deg]
        u_los = Rz(yaw) . los_triad(ex, eps)[0]
        target_pos_value = own_pos + range * u_los The "vision:get_estimated_target()" binding that the
        user asked about is NOT present in ArduPilot; These three lines are exactly what is produced
        on the companion side.

(2) NO SPEED FORWARD FEED. The first term of the classical law is vel_of_target and it DOES the work
on a moving target. In our case, it is forbidden -> pure P remains. The result is arithmetic:
permanent tracking distance d* = v_target / kp in pure P. Target 21 m/s and ArduPilot defaults to
FOLL_POS_P=0.1 while d* = 210 m, i.e. completely outside the engagement envelope (handoff <=60 m) --
the vanilla setting cannot MATHEMATICALLY capture this task. So the default kp=1.0 (same number as
default ArduCopter PSC_POSXY_P): 35 At m errors the command already saturates the speed cap, i.e.
"full throttle towards the target" throughout the engagement. This is the only justification UNDER
kp; Interchangeable with --kp.

(3) CLOSE BRAKE DEFAULT OFF. limit_velocity_2D trims the speed with sqrt(2*a*d) when approaching the
target; At 25 m (a=2.5 m/s^2) the ceiling becomes 10.9 m/s -- 21 CANNOT REACH the target of m/s. The
brake is "stay next to" itself, it's not our job. --braking is ON with ap (for those who want station
keeping / safe distance behavior), --braking is ON with range only away.

(4) SPEED MAGNITUDE IS FROM THE NAVIGATION CEILING, NOT FROM THE LAW. The inevitable consequence of
(2): |v| = kp*not every pure P law with error can hit the moving target -- range FROZES at
equilibrium distance d* = v_target /kp (measured in closed loop: 22.75 m, theory 21.05 ). The
solution is plane_follow .lua's own architecture: there also DIRECTION ( GUIDED_CHANGE_HEADING ) and
SPEED ( GUIDED_CHANGE_SPEED ) are SEPARATE channels. In our case, speed comes from cruise ceiling
(speed_source ='ceiling'). The select the pure mode_follow configuration with the --speed-source p and is measured
for comparison.

Other than that, the code does what ArduPilot does; in particular there is NOT anything specific to
MPC such as FOV/frame cost, term PN, target maneuver estimation (DisturbanceEstimator). The aim is to
measure the question "How far is the 150 line ArduPilot law enough?"

--------------------------------------- WHO PROTECTS THE FRAME (critical difference) There is NO
term in this law that ensures that the target remains in the frame. Measured result: at the time of
handoff the command 17 -> 35 STEPS to m/s, acceleration tilts the nose 15.7 deg down, the body-fixed
camera descends with it and the target exits the TOP edge (18 frames without detection in closed loop). The only
defense is kinematic shaping (acceleration_shaping_mps2, default 3.0 -- measurement chart below) and
that's ArduPilot's own tool. The REAL answer to ArduPilot is not in the guidance but in the
hardware: It locks the gimbal to the target when entering FOLLOW mode.
(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER, mode_follow.cpp init()).

--------------------------------------------------------------- WHY OFFSET 0 FOLL_OFS_* gives the
point to stop NEXT to the target; In the multiplication scenario, its natural value is zero and that
is the default. Free side benefit of zero offset: since the command is looking DIRECTLY at the
target, the fighter climbs to the target's altitude, goes eps -> 0, and on a fixed 0 degree camera
the target does not go out of frame. In the MPC this was a separate mechanism (stroke_align_*,
lap-3); here it comes from the law itself. Those who want follow-up with standoff give
--ofs-backward/--ofs-down; ofs melts linearly to zero as the range decreases to ofs_reduction_range_m (or
else the fighter will oscillate around the offset point with the brake closed).

------------------------------------------------------------- USAGE
    cd guidance_allstar && python3 tracking_guidance.py
    DURATION=360 DISPLAY="tracking_guidance.py" PLAN=missions/target_infinity.plan \\ tools/scenario.sh # full
    trial (METHOD=follow-up)
    python3 tracking_test.py                    # offline tests (simulation not required)
"""

from __future__ import annotations

import argparse
import csv
import math
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

KDEG = 180.0 / math.pi


# ============================================================ ENVIRONMENT DOORS EXACT copy of its
# twins in mpc_guidance.py. Copied (not imported) because this module should be able to fly INDEPENDENT
# from MPC: importing mpc_guidance also loads the solver and the 3000 row tuning block, and A/B
# comparison becomes meaningless if the two branches accidentally share each other's settings.

def environment_mount_deg(default_value2: float = 0.0) -> float:
    """The elevation of the camera axis relative to the horizon FROM A SINGLE SOURCE (standoff_geom.sh).

    GIMBAL BRANCH: physical stabilized gimbal command $YILDIZ_TILT priority; $YILDIZ_MOUNT
    replacement for frozen body-hard runs."""
    for key_value in ('YILDIZ_TILT', 'YILDIZ_MOUNT'):
        value_value = os.environ.get(key_value)
        if value_value is not None:
            try:
                return float(value_value)
            except (TypeError, ValueError):
                pass
    return float(default_value2)


def environment_speed_ceiling(default_value2: float = 35.0) -> float:
    """Speed ​​ceiling FROM THE SINGLE SOURCE: guidance_config.VISUAL_MAX_SPEED_MPS.

    The skeleton ( visual_base ) command ALREADY clamps with this number; If the actuator's own
    ceiling is lower, it won't touch the clamp at all and the bottleneck will silently shift to the
    homing's own setting (this is exactly what happened with the mpc_infinity_20260805_022808)."""
    try:
        import guidance_config as _cfg
        return float(getattr(_cfg, 'VISUAL_MAX_SPEED_MPS', default_value2))
    except Exception:                       # offline/partial installation
        return float(default_value2)


# ============================================================ ARDUPILOT MATH

def sqrt_controller(error_value: float, p: float, second_order_ceiling: float,
                    dt: float) -> float:
    """AP_Math/control.cpp::sqrt_controller EXACTLY portu.

    P controller near Setpoint, sqrt(2*a*dx) far away: that is, the "if I go at this speed, can I
    stop at the exact point with my acceleration ceiling" curve. This is the core of all position
    controllers of ArduPilot (AC_P_2D, AC_P_1D, AC_Avoid).
    """
    if second_order_ceiling <= 0.0:
        correction = error_value * p
    elif p == 0.0:
        if error_value > 0.0:
            correction = math.sqrt(2.0 * second_order_ceiling * error_value)
        elif error_value < 0.0:
            correction = -math.sqrt(2.0 * second_order_ceiling * (-error_value))
        else:
            correction = 0.0
    else:
        linear_distance = second_order_ceiling / (p * p)
        if error_value > linear_distance:
            correction = math.sqrt(2.0 * second_order_ceiling
                                 * (error_value - linear_distance / 2.0))
        elif error_value < -linear_distance:
            correction = -math.sqrt(2.0 * second_order_ceiling
                                  * (-error_value - linear_distance / 2.0))
        else:
            correction = error_value * p
    if dt > 0.0:                       # do not overshoot the remaining error in the final step
        return float(np.clip(correction, -abs(error_value) / dt, abs(error_value) / dt))
    return float(correction)


def sqrt_controller_2d(error_xy, p: float, second_order_ceiling: float,
                       dt: float):
    """Vector form (AP_Math / control.cpp): direction is preserved, size is shaped."""
    error_xy = np.asarray(error_xy, dtype=float)
    boy = float(np.linalg.norm(error_xy))
    if boy <= 0.0:
        return np.zeros(2)
    return error_xy * (sqrt_controller(boy, p, second_order_ceiling, dt) / boy)


def los_triad(ex_deg, eps_deg):
    """Unit vector LOS in the nose (heading) frame (SAME as mpc_guidance).

    Frame: x forward (nose, HORIZONTAL), y right, z down -- only yaw applied NED. ex: horizontal
    bearing of the target relative to the nose (+right), eps: elevation of the target relative to
    the horizon (+up). Only the first leg (unit vector along LOS) is required; The e2/e3 legs of MPC
    are not used in this law because the instruction is installed directly in NED without splitting
    it into LOS components."""
    ex = math.radians(ex_deg)
    eps = math.radians(eps_deg)
    ce, se = math.cos(eps), math.sin(eps)
    return np.array([ce * math.cos(ex), ce * math.sin(ex), -se])


# ==================================================================== SETTINGS

@dataclass
class TrackingConfig:
    """Settings corresponding to ArduPilot parameter names.

    Next to each field is the corresponding ArduPilot and the justification for the deviation, IF
    ANY. Any number with no reason for deviation defaults to ArduPilot."""

    # ---------------------------------------------------- AP_Follow (FOLL_*)
    kp: float = 1.0
    # FOLL_POS_P. AP default 0.1; here 1.0 -- see file title (2). 1.0 is also the default PSC_POSXY_P,
    # i.e. the number already used in the "poscon" branch; the same so that two knobs can be compared to a
    # single knob.
    ofs_backward_m: float = 0.0             # FOLL_OFS_X sign has been translated (see
    ofs_down_m: float = 0.0            # file title "WHY OFFSET 0")
    ofs_reduction_range_m: float = 25.0
    # The range where the offset has fully decayed to zero, rather than where decay starts: scale =
    # clip((r - melt) / (terminal - melt), 0 , 1 ). With a zero offset, this scaling has no effect.
    yaw_p: float = 4.5                  # ATC_ANG_YAW_P (ArduCopter default)
    yaw_speed_ceiling_dps: float = 60.0    # ATC_SLEW_YAW 6000 cdeg/s
    yaw_command_provide: bool = True         # ablation: leave yaw on autopilot

    # ----------------------------------------- pos_control limitleri (WPNAV_*)
    speed_ceiling_mps: float = field(default_factory=environment_speed_ceiling)
    climb_ceiling_mps: float = 10.0   # WPNAV_SPEED_UP 1000 cm/s
    descent_ceiling_mps: float = 5.0     # WPNAV_SPEED_DN 500 cm/s
    acceleration_horizontal_mps2: float = 5.0        # WPNAV_ACCEL 500 cm/s^2
    acceleration_vertical_mps2: float = 5.0        # WPNAV_ACCEL_Z 500 cm/s^2
    psc_pos_p: float = 1.0              # PSC_POSXY_P
    psc_pos_z_p: float = 1.0            # PSC_POSZ_P

    # ------------------------------------------------------------------ law
    law: str = 'classical'                # 'classical' (Copter<=4.4) | 'poscon' (>=4.5)
    braking: str = 'disabled'                # 'closed' | 'ap' | 'range'
    braking_range_m: float = 45.0         # the brake is ON on the 'range' lever.
                                        # over range (closes below)
    speed_source: str = 'ceiling_value'          # 'ceiling' | 'p'
    # ------------------------------------------------------------------------------- WHERE DOES SPEED
    # COME FROM -- THIS IS THE SINGLE MOST IMPORTANT DECISION OF THIS BRANCH.
    #
    # 'p' : mode_follow.cpp itself. Velocity MAGNITUDE also follows from the law: |v| = kp * |error|.
    # MEASURED RESULT (tracking_test 4 and 8): without target velocity feedforward this law MATHEMATICALLY
    # cannot hit a MOVING target -- permanent equilibrium distance d* = v_target / kp (21.05 m/s target +
    # kp=1.0 -> 21 m). Measured in closed loop: min range 22.75 m, shutdown +0.8 m/s. The only way to
    # reduce the offset distance is to increase kp, which increases the bearing noise by the same amount
    # (v = kp*r).
    #
    # 'ceiling': plane_follow.lua ARCHITECTURE brought into the hopper. There, too, direction and speed
    # are SEPARATE channels: GUIDED_CHANGE_HEADING gives direction, GUIDED_CHANGE_SPEED gives speed (and
    # speed comes from a PID around the target's airspeed, it doesn't scale with distance). In our case,
    # the direction comes from the FOLLOW law (LOS + offset) and the speed comes from the cruise ceiling.
    # There is no need to know anything about target speed: "towards target, as fast as I can". THIS IS
    # THE DEFAULT because the task is to MULTIPLY; The 'p' lever stands for comparison and --speed-source
    # opens with p.
    acceleration_shaping_mps2: float = 3.0
    # Command SIZE INCREASE rate limit [m/s^2]; 0 = off. ArduPilot equivalent: kinematic shaping of
    # pos_control
    # (AP_Math/control.cpp::shape_vel_accel, WPNAV_ACCEL/WPNAV_JERK);
    # mode_follow >= 4.5 already passes the target there, so this line is "Copter puts back the step that
    # 4.4 skipped". Only INCREASE is clipped; reduction is free (brake duct separate, see brake).
    #
    # Default 3.0, measured with tracking_test across 6 scenarios x 4 seeds and a 21.05 m/s target.
    # Minimum-range medians and framing-loss percentages: a=0.0: 1.78 / 15.36 / 7.96 / 28.86 / 14.96 /
    # 30.00, loss 50-72%. a=2.0: 5.37 / 18.65 / 5.48 / 18.02 / 8.60 / 23.42, loss 0-8%. a=3.0: 1.35 / 1.37
    # / 1.88 / 21.48 / 5.20 / 17.78, loss 0-23%. a=5.0: 1.68 / 2.83 / 2.50 / 37.11 / 1.74 / 39.81, loss
    # 7-41%. In this law, shaping is a condition for closure.
    #
    # THE OPPOSITE WAS MEASURED AT MPC (FOLLOW_UP.md eng-2: forward_acceleration_ceiling 2 m/s^2 -> min range 1.93 ->
    # 11.17 m, "measured and eliminated"). NOT A CONTRADICTION: MPC ALREADY protected the frame at its
    # cost, shaping there only brought a closing cost. There is NO term in this law that protects framing;
    # STEP 17 -> 35 m/s at the moment of handoff tilts the nose 15.7 deg down, the fixed camera goes down
    # with it and the target exits the TOP edge (frame blind 18 in the closed loop trace). The framing
    # gained by shaping is greater than the coverage it loses.

    # ------------------------------------------------------- camera geometry
    mount_pitch_deg: float = field(default_factory=environment_mount_deg)
    aim_deg: float = 0.0                # scenario.sh fixes AIM=0
    # eps = -(ey + aim): the virtual gimbal produces ey relative to the camera AXIS, the axis is also
    # tilted by (mount + aim). mount 0 + 0 in eps = -ey. mount_pitch_deg is only used in LOG and frame
    # allowance calculation: this law does NOT impose a framing constraint (ArduPilot does not have such a
    # thing).

    # ------------------------------------------------------- range filter mpc_guidance._SAME as range: since
    # the measurement is sparse/noisy, the internal state is advanced with the model and pulled into the
    # measurement with partial gain.
    range_measurement_gain: float = 0.35
    range_if_absent_m: float = 55.0
    range_floor_m: float = 6.0
    range_rate_tau_s: float = 0.30
    area_rate_tau_s: float = 0.35
    stale_constraint_s: float = 0.30

    # ------------------------------------------------- HIT / MISS (COMMON) THIS BLOCK IS EXACTLY THE SAME
    # NUMBERS AS MPC. Conscious: termination and contact detection are NOT THE LAW OF GUIDANCE; If two
    # arms are not measured by the same yardstick, the comparison becomes contaminated. Reasons: In hit_*/
    # hit_* blocks in mpc_guidance.MpcConfig; It is NOT repeated here, it is not changed.
    impact_mode: bool = True
    impact_range_m: float = 22.0
    impact_full_range_m: float = 8.0
    terminal_range_m: float = 45.0
    impact_success_detection: bool = True
    impact_success_vibe: float = 15.0
    impact_success_range_m: float = 3.0
    miss_mode: bool = True
    miss_source: str = 'range_value'         # 'range' | 'area'
    # ------------------------------------------------------------------------------- WHERE DOES THE
    # REFEREE FEED -- the last step of the user's "I don't trust the range" rule.
    #
    # MEASURED CONDITION ( tracking_test 11 and 12 ): * COMMAND PATH is ALREADY completely independent of
    # range. In the default setting (ofs= 0 , speed_source ='ceiling') it simplifies algebraically to: v =
    # kp*(r* u_los ) -> when scaled to the ceiling -> V* u_los In closed loop, the 8 jamming arm (x0. 5 ,
    # x2, + 20 m, % 30 noise, rotated, % 50 broken, NO RANGE) gave the EXACT SAME result. * The only
    # remaining consumer is the SQUARE REFEREE and IT IS BREAKING: in the same synthetic profile the
    # moment of fire is x0. 5 -> 70.6 m, clear -> 18.7 m, + 20 m -> 40.7 m, x2. 0 -> 25.7 scrolls to m.
    # Reason: difference rules (r > best_candidate + 30 ) are nested with ABSOLUTE gates ( 12 / 45 / 120 m).
    #
    # The 'field' ARM also removes this dependency. Physics: since the target is fixed size s = sqrt(bbox
    # area) ~ C/r. Even if C is not known, the RATIO is known: r / r_en_good == s_peak / s So the test
    # "range opened to 2 times the best" translates ONE TO ONE to "apparent height dropped to half the
    # top" and C SIMPLIFIES -- NO calibration REQUIRED. Absolute doors are not required either: the
    # question "have we passed" is already answered by area_rate's change of sign (it was getting bigger,
    # it got smaller).
    #
    # COST (to be honest): bbox IF IT IS STALE, the field freezes and the referee remains BLIND. Range
    # estimation would continue to flow independently of vision. In the blind phase, the only protection
    # left is TIMEOUT. So the default is still 'range' (to avoid polluting sim benchmarks); 'field'
    # --miss-source with field.
    #
    # WHY ONLY RATIO, NEVER ABSOLUTE RANGE (measured by log mining, 14.816 frames / 12 runs, 2026 - 08 -
    # 05 ): * C = median( sqrt (area)* ground_truth_range ) remained between 685 - 730 across 9 runs in
    # the AIRCRAFT group, or approximately +/- 3.3 %, supporting calibration across runs. * Individual
    # frames had a wide distribution: for r_visual = C/ sqrt (area), the absolute error in the 0 - 20 m
    # band was median 2.74 m and p90 21.4 m. The range estimator achieved 1.07 / 2.21 m in the same band,
    # making the visual estimate's tail 9 .7x worse. * The model is valid only at approximately 9 - 50 m.
    # For min(w,h) <= 8 px , C drops to 128 - 407 . It also gradients above min(w,h) > 80 px ,
    # approximately 7 m near impact. In the 0 - 5 m band, C becomes 133.7 instead of 698. * Error is
    # episodic, rather than white noise: episodes with > 30 % e rror have p90 duration 0.65 s, so
    # filtering does not resolve it. CONCLUSION: area cannot provide absolute range. The evaluator uses a
    # relative question, whether area has halved from its peak, which cancels C. The validity window still
    # requires a quality gate, miss_area_pixel_floor , and a debounced decision.
    miss_opening_ratio: float = 2.0
    # "range opening" in branch 'area': s < s_peak / rate. 2.0 = "apparent length halved" = "range
    # doubled". The additive threshold of +30 m in the range arm in typical best_candidate values (10-20 m)
    # corresponded to 2.5-4x, and at 45 m corresponded to 1.67x -- that is, according to the old rule
    # range It was INCONSISTENTLY harsh; The ratio rule is the same on every scale.
    miss_transition_opening_ratio: float = 1.6    # after the transition is confirmed (narrow)
    miss_area_pixel_floor: float = 9.0
    # Quality gate [px]: exclude area measurements from the evaluator when min(bbox_w, bbox_h) is below
    # this threshold. Measured median relative errors were 267% at 0-4 px, 844% at 5-6 px, 86% at 7-8 px,
    # 10% at 9-12 px, and 6% at 13-20 px. The equivalent coverage threshold is approximately 1.0%.
    miss_area_tau_s: float = 0.35       # sqrt (field) LPF (episodic jump)
    miss_area_confirmation_loop: int = 6       # rate threshold how many cycles in a row
    miss_area_small_loop: int = 25
    # If this many cycles remain BELOW the quality gate, the "target is too far/small". Visual equivalent
    # of miss_absolute_m (120 m): in the sim 402 was the arm that captured the fake rev made in m, this
    # replaces it in the 'field' arm -- and WITHOUT READING RANGE.
    miss_arm_m: float = 45.0
    miss_opening_m: float = 30.0
    miss_transition_arm_m: float = 12.0
    miss_transition_opening_m: float = 8.0
    miss_absolute_m: float = 120.0
    miss_time_timeout_s: float = 8.0
    miss_time_source: str = 'progress'   # 'progress' | 'straight'
    # ------------------------------------------------------------------------------- WHAT DOES TIMEOUT
    # COUNT -- here is the most expensive defect measured.
    #
    # 'plain' (default, same as MPC ): WALL CLOCK elapsed since after handoff. The defect was caught in
    # the offline trace (ellipse/cross, seed 3 ): t= 0.0 r= 45.0 shutdown - 3.0 m/s t= 6.6 r= 31.4
    # shutdown + 5.9 t= 7.8 r= 23.2 shutdown + 7.4
    #     t=8.0 TIMEOUT -> MISS closing +7.7 (AND ACCELERATING) In other words, the range decreased to 45
    #     -> 19 m, the closing increased with each stroke and the referee blew the whistle just as he was
    #     WINNING. The same pattern exists in the hanging target sim run: 9 of engagement 9 interrupted by
    #     timeout, best ranges 20.8/19.2/14.3/13.4/8.8/5.6/4.6 m -- all while shutting down.
    #
    # 'progress': the clock only ticks when there is NO PROGRESS. The definition of “progress” is VISUAL
    # and scale-independent: if the visible area of ​​the target GROWS, we are getting closer. Relative
    # growth (dA/dt)/A is used because A ~ 1 /r^ 2 since (dA/dt)/A = 2 *closure/r [ 1 /s] -- i.e. a
    # measure of "am I getting closer" that is INDEPENDENT of range. If there was an absolute threshold of
    # px ^ 2 /s, it would never trigger at a distance, but would always trigger at a distance.
    #
    # No leaks: the counter is NOT reset, it is REWOUNDED (leaky integrator) -- so that the noisy
    # area_rate's signal flapping cannot stop the clock indefinitely. An absolute ceiling
    # (miss_absolute_duration_s) is placed on top.
    miss_progress_threshold_1s: float = 0.05
    # Relative area growth threshold [ 1 /s].  0.05 = " 2 *off/r > 0.05 ", i.e. 40 at m 1 m/s , 20 at m
    # 0.5 m/s off. Above the noise band, below significant closure.
    miss_absolute_duration_s: float = 25.0
    # THE ABSOLUTE ceiling of engagement [s]: if this is exceeded, SCA even if progress continues. Safety
    # valve to prevent endless engagement.
    miss_start_protection_s: float = 1.0
    transition_range_rate_threshold_mps: float = 3.0
    transition_closure_threshold_mps: float = 10.0
    transition_confirmation_loop: int = 4
    transition_area_confirmation_loop: int = 6
    miss_coast_speed_mps: float = 12.0
    miss_coast_acceleration_mps2: float = 3.0
    miss_redis_key: str = ''

    def __post_init__(self):
        if self.law not in ('classical', 'poscon'):
            raise ValueError(f"law 'classical' | 'poscon' should be: {self.law!r}")
        if self.braking not in ('disabled', 'ap', 'range_value'):
            raise ValueError(f"brake must be 'off'|'ap'|'range': {self.braking!r}")
        if self.speed_source not in ('ceiling_value', 'p'):
            raise ValueError(f"speed_source should be 'ceiling'|'p': {self.speed_source!r}")
        if self.miss_source not in ('range_value', 'area_value'):
            raise ValueError(f"miss_source should be 'range'|'area': "
                             f"{self.miss_source!r}")
        if self.miss_time_source not in ('straight', 'progress'):
            raise ValueError(f"miss_time_source should be 'flat'|'progress': "
                             f"{self.miss_time_source!r}")


# ================================================================ CONTROLLER

class TrackingController(VisualController):
    """ArduPilot FOLLOW law conforming to contract visual_base."""

    label_item = "tracking_value"

    def __init__(self, config_value: TrackingConfig = None, diagnostic_log=None):
        self.a = config_value or TrackingConfig()
        self.diagnostic_log_path = diagnostic_log
        self._diagnostic_f = None
        self._diagnostic = None
        self._redis = None
        self.reset_value()

    # ------------------------------------------------------------- situation

    def reset_value(self):
        self.internal_range = None
        self.area_value = None
        self.area_rate = 0.0
        self.counter = 0
        self.v_ned_seed = None
        self._last_v_ned = None
        self._last_error_z = 0.0           # for vertical brake (refreshed in command())
        self.yaw_applied_value = 0.0
        # --- state machine (same meaning as mpc_guidance) ---
        self.state_value = 'CLOSURE'           # CLOSURE | TERMINAL | IMPACT | MISS
        self.impact_blend = 0.0
        self.hit_value = False
        self.impact_vibe_value = 0.0
        self.impact_range = 0.0
        self._pending_event = None
        self.best_range = float('inf')
        self.range_rate_value = 0.0
        self.passed_value = False
        self.miss_reason = ''
        self._r_previous = None
        self._transition_counter = 0
        self._transition_area_counter = 0
        self._closure_peak = 0.0
        self._authority_t0 = None
        # --- visual referee status (miss_source='area') ---
        self.s_lpf = None                # sqrt (field) LPF [ px ]
        self.s_peak = 0.0                # The largest (closest) value seen
        self._area_increased = False        # Did it even grow once (for transition)
        self._area_opening_counter = 0
        self._area_small_counter = 0
        self._stalled_s = 0.0     # 'progress' timeout clock

    def seed_value2(self, handoff):
        """once at the time of handoff. Same role as the init() of the ArduPilot FOLLOW mode: initializing the
pos_control state from the CURRENT speed of the vehicle. In our case, the LPF of the skeleton is
already seeded with the handoff speed, only a record is kept here (so that the 'handoff speed'
appears in the diagnostic log)."""
        self.reset_value()
        if handoff and 'cmd_vel_ned' in handoff:
            self.v_ned_seed = np.asarray(handoff['cmd_vel_ned'], dtype=float)
            # IRON OF THE FIRST COMMAND: acceleration shaping and HSI gliding require a "previous command". If not
            # seeded, the first cycle would SKIP shaping -- and the 18 -> 35 m/s step at handoff would happen
            # exactly in that first cycle, so shaping would do nothing.
            self._last_v_ned = self.v_ned_seed.copy()
        print(f"[tracking] seeded, handoff speed="
              f"{None if self.v_ned_seed is None else np.round(self.v_ned_seed, 2).tolist()}")

    # -------------------------------------------------------- yardimcilar

    def _range(self, measurement, closure_mps, dt):
        """Inside range status (same method as mpc_guidance._range).

        Measurement ONLY.range_m_value is used. closure_mps: Projection of our own speed along LOS (+ =
        range becomes shorter) -- model advancement term."""
        if self.internal_range is None:
            self.internal_range = (float(measurement.range_m_value) if measurement.range_m_value is not None
                         else self.a.range_if_absent_m)
            return self.internal_range
        self.internal_range += dt * (-closure_mps)
        if measurement.range_m_value is not None:
            self.internal_range += self.a.range_measurement_gain * (
                float(measurement.range_m_value) - self.internal_range)
        self.internal_range = float(max(self.a.range_floor_m * 0.5, self.internal_range))
        return self.internal_range

    def _area_update(self, measurement, dt):
        """bbox area (px^2) and its growth rate -- one of the transition witnesses."""
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

    def _impact_successful_control(self, measurement, r):
        """PHYSICAL CONTACT: our own vibration + MEASURED range.

        Exactly the same thresholds and the same latch logic as the
        mpc_guidance._hit_successful_control -- the number of hits of both arms should be counted with
        the same definition."""
        a = self.a
        if (not a.impact_success_detection or self.hit_value
                or measurement.vibe_max is None):
            return None
        r_measure = float(measurement.range_m_value) if measurement.range_m_value is not None else float(r)
        if (float(measurement.vibe_max) > a.impact_success_vibe
                and r_measure < a.impact_success_range_m):
            self.hit_value = True
            self.impact_vibe_value = float(measurement.vibe_max)
            self.impact_range = r_measure
            detail = (f"vibe= {self.impact_vibe_value:.1f} (threshold "
                     f"{a.impact_success_vibe:.0f}) range={r_measure:.2f} m "
                     f"status={self.state_value} hit={self.impact_blend:.2f}")
            print(f"[following] IMPACT_SUCCESSFUL: {detail}")
            return ('impact_successful', detail)
        return None

    # -------------------------------------------------- MISS state machine

    def _state_machine(self, measurement, r, area_rate, dt):
        """CLOSE -> TERMINAL -> HIT, and SCA termination.

        EXACT equivalent of mpc_guidance._situation_machine (same witnesses, same thresholds). Its
        inputs are: range (single target measurement allowed), field speed bbox, and our own clock.
        Target speed is NOT derived."""
        a = self.a
        if self._authority_t0 is None:
            self._authority_t0 = measurement.t
        total_value = float(measurement.t - self._authority_t0)
        elapsed_item = self._time_timeout_clock(measurement, area_rate, dt, total_value)

        if self._r_previous is not None:
            raw_value = float(np.clip((r - self._r_previous) / dt, -60.0, 60.0))
            k = dt / (dt + max(a.range_rate_tau_s, 1e-6))
            self.range_rate_value += k * (raw_value - self.range_rate_value)
        self._r_previous = float(r)
        if r < self.best_range:
            self.best_range = float(r)

        if self.state_value != 'MISS':
            if self.best_range <= a.terminal_range_m:
                self.state_value = 'TERMINAL'
            if a.impact_mode and self.best_range <= a.impact_range_m:
                self.state_value = 'IMPACT'
        if self.state_value == 'IMPACT':
            width_value = max(a.impact_range_m - a.impact_full_range_m, 1e-6)
            self.impact_blend = float(np.clip(
                (a.impact_range_m - r) / width_value, 0.0, 1.0))
        else:
            self.impact_blend = 0.0

        if not a.miss_mode or self.state_value == 'MISS':
            return
        if a.miss_source == 'area_value':
            return self._evaluator_area(measurement, area_rate, dt, elapsed_item)

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
        elif elapsed_item > a.miss_time_timeout_s:
            reason_value = (f"timeout ({elapsed_item:.1f} s > {a.miss_time_timeout_s:.0f}), "
                     f"best range {self.best_range:.1f} m")
        if reason_value:
            self.state_value = 'MISS'
            self.miss_reason = reason_value
            print(f"[follow-up] MISS: {reason_value} -> authorization is released")
            self._miss_publish()

    def _time_timeout_clock(self, measurement, area_rate, dt, total_value):
        """Returns the timeout time (see TrackingConfig.miss_time_source).

        'straight': wall clock -- time since handoff. 'progress': A leaky clock that runs ONLY when
        there is no progress. The measure of progress is VISUAL: relative area growth (dA/dt)/A >
        threshold. Since A ~ 1 /r^ 2 , this magnitude is equal to 2 *closing/r, so it answers the
        question "am I getting closer" WITHOUT reading the range. If the absolute ceiling is hung on
        both arms, the clock is locked to the ceiling."""
        a = self.a
        if a.miss_time_source == 'straight':
            return total_value
        fresh_value = measurement.bbox_age_s <= a.stale_constraint_s
        relative = 0.0
        if fresh_value and self.area_value is not None and self.area_value > 1e-9:
            relative = float(area_rate) / float(self.area_value)
        if relative > a.miss_progress_threshold_1s:
            # PROCEEDING: rewind the clock (NO reset -- do not stop the clock indefinitely if area_rate ticks).
            self._stalled_s = max(0.0, self._stalled_s - dt)
        else:
            self._stalled_s += dt
        if total_value > a.miss_absolute_duration_s:
            # SAFETY VALVE: even if progress continues, engagement cannot be endless.
            return max(self._stalled_s, a.miss_time_timeout_s + 1e-6)
        return self._stalled_s

    def _evaluator_area(self, measurement, area_rate, dt, elapsed_item):
        """VISUAL REFEREE: WATCH decision from bbox ONLY, WITHOUT READING THE RANGE.

        Three rules, all three SCALE-INDEPENDENT (C simplifies): 1. OPENING: s = sqrt(area) fell to
        1/rate of the top (since s_peak/s == r/r_en_good, this literally means "range increased to
        the rate times the best") 2. TRANSITION : area was growing, now shrinking -> passed
        (doppelganger of the 'closing peak' requirement in the range arm) 3. TOO SMALL: we remain
        persistently under the quality gate -> target too far (rangeless counterpart of
        miss_absolute_m) Timeout lever common; He doesn't read range anyway.

        QUALITY GATE condition: measured validity window min(w,h) >= 9 px (relative error %86-844
        below). The squares under the gate DO NOT update the top either -- otherwise a single blown
        bbox would put an unreachable top and the referee would immediately call 'opening'."""
        a = self.a
        edge = None
        if measurement.bbox_w is not None and measurement.bbox_h is not None:
            edge = min(float(measurement.bbox_w), float(measurement.bbox_h))
        fresh_value = measurement.bbox_age_s <= a.stale_constraint_s
        high_quality = (fresh_value and edge is not None
                    and edge >= a.miss_area_pixel_floor
                    and self.area_value is not None and self.area_value > 0.0)

        if high_quality:
            self._area_small_counter = 0
            s = math.sqrt(float(self.area_value))
            if self.s_lpf is None:
                self.s_lpf = s
            else:
                k = dt / (dt + max(a.miss_area_tau_s, 1e-6))
                self.s_lpf += k * (s - self.s_lpf)
            self.s_peak = max(self.s_peak, self.s_lpf)
            if area_rate > 0.0:
                self._area_increased = True
            # (2) TRANSITION: we were big, now it's getting smaller
            self._transition_area_counter = (self._transition_area_counter + 1
                                      if (self._area_increased and area_rate < 0.0)
                                      else 0)
            if self._transition_area_counter >= a.transition_area_confirmation_loop:
                self.passed_value = True
            # (1) OPENING
            ratio_value = (a.miss_transition_opening_ratio if self.passed_value
                    else a.miss_opening_ratio)
            opened = (self.s_peak > 0.0 and self.s_lpf * ratio_value < self.s_peak)
            self._area_opening_counter = (self._area_opening_counter + 1
                                       if opened else 0)
        elif fresh_value or measurement.bbox_age_s > a.stale_constraint_s:
            # Under quality gate OR bbox stale: peak/rate is frozen (referee DOES NOT decide in blind phase), only
            # 'too small' counter processes.
            if edge is not None and edge < a.miss_area_pixel_floor:
                self._area_small_counter += 1
            self._area_opening_counter = 0

        if elapsed_item < a.miss_start_protection_s:
            return
        reason_value = ''
        if self._area_opening_counter >= a.miss_area_confirmation_loop:
            ratio_value = (a.miss_transition_opening_ratio if self.passed_value
                    else a.miss_opening_ratio)
            reason_value = (f"gorunen boy tepenin 1/{ratio_value:.1f}'ine dustu "
                     f"(s {self.s_lpf:.1f} < peak {self.s_peak:.1f} / {ratio_value:.1f} )"
                     + (", pass approved" if self.passed_value else ""))
        elif self._area_small_counter >= a.miss_area_small_loop:
            reason_value = (f"target too small ({edge:.1f} px < "
                     f"{a.miss_area_pixel_floor:.0f}) {self._area_small_counter} loop")
        elif elapsed_item > a.miss_time_timeout_s:
            reason_value = (f"timeout ({elapsed_item:.1f} s > {a.miss_time_timeout_s:.0f}), "
                     f"visible peak {self.s_peak:.1f} px")
        if reason_value:
            self.state_value = 'MISS'
            self.miss_reason = reason_value
            print(f"[follow-up] MISS (visual referee): {reason_value} -> authorization is released")
            self._miss_publish()

    def _miss_publish(self):
        """OPTIONAL Redis bridge publication (same contract as mpc_guidance)."""
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
        except Exception as exc:
            print(f"[follow-up] MISS Redis broadcast failed: {exc}")

    # ------------------------------------------------------- FOLLOW law

    def _target_estimate(self, measurement, r, ex, eps):
        """(1) Replaces AP_Follow VIEW: location of target NED.

        AP_Follow takes this from GLOBAL_POSITION_INT and extrapolates it to the target's SPEED
        (get_target_pos_vel_accel_NED_m). We have two differences: * location is established from
        the image + range, * NO EXTRAPOLATION (target speed is prohibited). So the prediction always
        belongs to the 'now' moment; It is behind by the delay of bbox. Rotary: (target_pos_ned,
        u_los_ned). If pos_ned is not present the target position remains relative (assuming pos =
        0) -- the law already uses the difference."""
        yaw = measurement.yaw_rad if measurement.yaw_rad is not None else 0.0
        l_h = los_triad(ex, eps)                    # heading cercevesi
        u_los = body_forward_ned(yaw, l_h[0], l_h[1], l_h[2])
        pos = (np.asarray(measurement.pos_ned, dtype=float)
               if measurement.pos_ned is not None else np.zeros(3))
        return pos + r * u_los, u_los

    def _offset(self, u_los, r):
        """Equivalent of FOLL_OFS_*, expressed in the LOS frame.

        AP_Follow applies offset either at NED or relative to the NOSE of the target (FRD); FRD asks
        for the heading of the target -> FORBIDDEN. Our frame is LOS: 'back' along the horizontal
        LOS towards us from the target, 'down' NED. Thus, we assume nothing about the target's
        orientation.

        MELT: offset decreases linearly to zero with range. The reasoning is simple -- a target
        point with a constant offset cannot be held when the brake is closed (the P law oscillates
        around the offset point); In the impact scenario, the task of the offset is to feed the
        frame only in the FAR phase."""
        a = self.a
        if a.ofs_backward_m == 0.0 and a.ofs_down_m == 0.0:
            return np.zeros(3), 1.0
        width_value = max(a.terminal_range_m - a.ofs_reduction_range_m, 1e-6)
        scale_value = float(np.clip((r - a.ofs_reduction_range_m) / width_value, 0.0, 1.0))
        horizontal = np.array([u_los[0], u_los[1], 0.0])
        n = float(np.linalg.norm(horizontal))
        if n > 1e-6:
            horizontal = horizontal / n
        ofs = scale_value * (-a.ofs_backward_m * horizontal + np.array([0.0, 0.0, a.ofs_down_m]))
        return ofs, scale_value

    def _speed_law(self, error_ned, r_horizontal, dt):
        """ModeFollow::run() speed law. error_ned = (target+ofs) - own position.

        THE ORDER IN THE ARDUPILOT is important: first P (or sqrt), then horizontal scaling, then
        vertical clamping, last approach brake. Since the brake is at the end, it clips a vector
        that has PASSED horizontal scaling -- as does ArduPilot (mode_follow.cpp: AFTER the
        limit_velocity_2D call scale/constrain)."""
        a = self.a
        v = np.zeros(3)
        if a.law == 'classical':
            # Copter <= 4.4: pure P (+ target speed FF -- we don't have it).
            v = a.kp * np.asarray(error_ned, dtype=float)
        else:
            # Copter >= 4.5: AC_P_2D / AC_P_1D = sqrt_controller.
            v[:2] = sqrt_controller_2d(error_ned[:2], a.psc_pos_p,
                                       a.acceleration_horizontal_mps2, dt)
            v[2] = sqrt_controller(float(error_ned[2]), a.psc_pos_z_p,
                                   a.acceleration_vertical_mps2, dt)

        # --- SPEED SOURCE (see TrackingConfig.speed_source) --- The law only determines the DIRECTION, its
        # magnitude gives the cruise ceiling. The DIRECTION source is again consciously the output of the law
        # (P/poscon vector), NOT the error vector itself: the offset and clamps also shape the direction, we
        # do not want to skip that shaping.
        if a.speed_source == 'ceiling_value':
            n = float(np.linalg.norm(v))
            if n > 1e-6:
                v = v * (a.speed_ceiling_mps / n)

        # --- horizontal speed cap (WPNAV_SPEED): DIRECTION PRESERVED, scaled ---
        horizontal_speed = float(math.hypot(v[0], v[1]))
        clamp_horizontal = horizontal_speed > a.speed_ceiling_mps
        if clamp_horizontal and horizontal_speed > 1e-9:
            v[0] *= a.speed_ceiling_mps / horizontal_speed
            v[1] *= a.speed_ceiling_mps / horizontal_speed

        # ---vertical ceilings (WPNAV_SPEED_UP/_DN); z down + --- on NED
        v[2] = float(np.clip(v[2], -a.climb_ceiling_mps, a.descent_ceiling_mps))

        # --- APPROACH FRENI (AC_Avoid::limit_velocity_2D) ---
        braking_enabled = (a.braking == 'ap'
                     or (a.braking == 'range_value' and r_horizontal > a.braking_range_m))
        braking_ceiling = float('inf')
        if braking_enabled:
            # CAUTION (ArduPilot's own quirk, deliberately preserved): limit_direction is the direction of the
            # DESIRED SPEED, NOT the direction towards the target (mode_follow.cpp builds dir_to_target_xy from
            # desired_velocity). Distance is the horizontal distance to the target.
            horizontal_v = np.array([v[0], v[1]])
            n = float(np.linalg.norm(horizontal_v))
            if n > 1e-9:
                direction = horizontal_v / n
                braking_ceiling = sqrt_controller(r_horizontal, a.psc_pos_p,
                                              0.5 * a.acceleration_horizontal_mps2, dt)
                speed_direction = float(horizontal_v @ direction)
                if speed_direction > braking_ceiling:
                    horizontal_v = horizontal_v + direction * (braking_ceiling - speed_direction)
                    v[0], v[1] = float(horizontal_v[0]), float(horizontal_v[1])
            # vertical brake: |v_z| <= get_max_speed(PSC_POSZ_P, accel_z/2, |dz|)
            dz = abs(float(self._last_error_z))
            vz_ceiling = sqrt_controller(dz, a.psc_pos_z_p,
                                       0.5 * a.acceleration_vertical_mps2, dt)
            v[2] = float(np.clip(v[2], -vz_ceiling, vz_ceiling))

        # --- OPTIONAL kinematic shaping (pos_control/shape_vel_accel)
        if a.acceleration_shaping_mps2 > 0.0 and self._last_v_ned is not None:
            n_new = float(np.linalg.norm(v))
            n_previous = float(np.linalg.norm(self._last_v_ned))
            ceiling_value = n_previous + a.acceleration_shaping_mps2 * dt
            if n_new > ceiling_value > 0.0:
                v = v * (ceiling_value / n_new)      # Preserve direction and limit the increment
        return v, clamp_horizontal, braking_ceiling

    def _yaw(self, ex):
        """FOLL_YAW_BEHAVE = 0 (FACE_LEAD_VEHICLE).

        ArduCopter yaw commands the angle (auto_yaw.set_yaw_angle_rate) and converts the angle error
        into alignment with ATC_ANG_YAW_P. The skeleton only gives us the yaw SPEED channel, so we
        do the same conversion: yaw_rate = ATC_ANG_YAW_P * (target bearing - nose) and we ALREADY
        have the bearing error: the camera's ex. Applies 120 deg/s^2 slew + 0.15 s LPF on the
        skeleton (common hygiene)."""
        a = self.a
        if not a.yaw_command_provide:
            return None
        return float(np.clip(a.yaw_p * ex,
                             -a.yaw_speed_ceiling_dps, a.yaw_speed_ceiling_dps))

    # ----------------------------------------------------------- MISS command

    def _miss_command(self, measurement, ex, ey, eps, r, area_value, area_rate, dt):
        """MISS: decelerating coast + 'release' flag (same design as mpc_guidance).

        We DO NOT command zero: zero is not "not giving a command" but FULL BRAKING. We maintain the
        direction and ramp down the magnitude; turning radius 35 m/s 245 m, 12 m/s 29 m --
        position guidance needs this reduction to reposition."""
        a = self.a
        v = (np.asarray(measurement.vel_ned, dtype=float).copy()
             if measurement.vel_ned is not None
             else (self._last_v_ned.copy() if self._last_v_ned is not None
                   else np.zeros(3)))
        speed_value = float(np.linalg.norm(v))
        if speed_value > 1e-6:
            target_speed = max(a.miss_coast_speed_mps,
                            speed_value - a.miss_coast_acceleration_mps2 * dt)
            v = v * (min(target_speed, speed_value) / speed_value)
        self._last_v_ned = v.copy()
        self.yaw_applied_value = 0.0
        self._diagnostic_write(measurement, ex, ey, eps, r, v, area_value, area_rate,
                       {'clamp_horizontal': 0, 'braking_ceiling': float('nan'),
                        'ofs_scale': 0.0, 'error_value': np.zeros(3),
                        'v_raw': np.zeros(3)})
        self.counter += 1
        k = Command(vel_ned=v, yaw_rate_dps=None)
        k.release_value = True
        k.release_reason = self.miss_reason
        return k

    # ------------------------------------------------------------- command

    def command_value(self, measurement: Measurement) -> Command:
        a = self.a
        dt = float(np.clip(measurement.dt, 0.02, 0.30))
        ex = float(measurement.ex_deg)
        ey = float(measurement.ey_deg)
        eps = -(ey + a.aim_deg)              # elevation of the target relative to the horizon

        # ---LOS and range ---
        yaw = measurement.yaw_rad if measurement.yaw_rad is not None else 0.0
        l_h = los_triad(ex, eps)
        u_los = body_forward_ned(yaw, l_h[0], l_h[1], l_h[2])
        # our own speed (+ = closing) along LOS for the model term of the range filter. The measured speed is
        # used, not the commanded speed.
        closure = (float(np.asarray(measurement.vel_ned, dtype=float) @ u_los)
                   if measurement.vel_ned is not None else 0.0)
        r = self._range(measurement, closure, dt)
        area_value, area_rate = self._area_update(measurement, dt)

        # --- state machine and contact detection (BEFORE the command path) ---
        self._state_machine(measurement, r, area_rate, dt)
        self._pending_event = self._impact_successful_control(measurement, r)
        if self.state_value == 'MISS':
            return self._event_tak(self._miss_command(
                measurement, ex, ey, eps, r, area_value, area_rate, dt))

        # --- (1) target estimation, (2) offset, (3) rate law ---
        pos = (np.asarray(measurement.pos_ned, dtype=float)
               if measurement.pos_ned is not None else np.zeros(3))
        target_pos_value = pos + r * u_los
        ofs, ofs_scale = self._offset(u_los, r)
        error_value = (target_pos_value + ofs) - pos          # = r*u_los + ofs
        self._last_error_z = float(error_value[2])       # for vertical brake
        r_horizontal = float(math.hypot(error_value[0], error_value[1]))
        v_ned, clamp_horizontal, braking_ceiling = self._speed_law(error_value, r_horizontal, dt)
        yaw_rate = self._yaw(ex)
        self.yaw_applied_value = 0.0 if yaw_rate is None else float(yaw_rate)

        self._diagnostic_write(measurement, ex, ey, eps, r, v_ned, area_value, area_rate,
                       {'clamp_horizontal': int(clamp_horizontal),
                        'braking_ceiling': braking_ceiling, 'ofs_scale': ofs_scale,
                        'error_value': error_value, 'v_raw': a.kp * error_value})
        self.counter += 1
        self._last_v_ned = v_ned.copy()
        k = Command(vel_ned=v_ned, yaw_rate_dps=yaw_rate)
        k.release_value = False
        k.release_reason = ''
        return self._event_tak(k)

    def _event_tak(self, k):
        event_value = getattr(self, '_pending_event', None)
        if event_value:
            k.event_value, k.event_detail = event_value
            self._pending_event = None
        return k

    # ------------------------------------------------------------------------------- diagnostic log

    DIAGNOSTIC_COLUMNS = [
        't', 't_unix', 'dt', 'state_value', 'impact_value',
        'ex', 'ey', 'eps', 'range_value', 'range_measure', 'range_rate_value', 'best_candidate',
        'error_n', 'error_e', 'error_d', 'error_horizontal', 'ofs_scale',
        'v_raw_n', 'v_raw_e', 'v_raw_d',
        'cmd_n', 'cmd_e', 'cmd_d', 'cmd_speed', 'cmd_horizontal',
        'clamp_horizontal', 'braking_ceiling', 'yaw_cmd_dps',
        'area_value', 'area_rate', 's_lpf', 's_peak', 'bbox_age', 'vibe', 'hit_value',
        'pitch_deg', 'yaw_deg', 'altitude_m',
    ]

    def _diagnostic_write(self, measurement, ex, ey, eps, r, v_ned, area_value, area_rate, ek):
        """follow_diagnosis_*.csv: make EVERY STEP of the law readable.

        The column selection deliberately OVERLAPS the diagnostic columns of the MPC (status, hit,
        range, range_rate_value , best_candidate , cmd_*, vibe, hit): so that the runs of both arms can be
        read with the same tool (tools/ explain_run .py, compare_results .py). Law-SPECIFIC columns:
        error_*, ofs_scale , v_raw_* (unclamped P output), braking_ceiling ."""
        if self.diagnostic_log_path is None:
            return
        if self._diagnostic_f is None:
            os.makedirs(os.path.dirname(self.diagnostic_log_path), exist_ok=True)
            self._diagnostic_f = open(self.diagnostic_log_path, 'w', newline='')
            self._diagnostic = csv.writer(self._diagnostic_f)
            self._diagnostic.writerow(self.DIAGNOSTIC_COLUMNS)
        error_value = np.asarray(ek.get('error_value', np.zeros(3)), dtype=float)
        v_raw = np.asarray(ek.get('v_raw', np.zeros(3)), dtype=float)
        ft = ek.get('braking_ceiling', float('nan'))
        self._diagnostic.writerow([
            f"{measurement.t:.4f}", f"{time.time():.3f}", f"{measurement.dt:.4f}",
            self.state_value, f"{self.impact_blend:.3f}",
            f"{ex:.4f}", f"{ey:.4f}", f"{eps:.4f}", f"{r:.2f}",
            '' if measurement.range_m_value is None else f"{float(measurement.range_m_value):.2f}",
            f"{self.range_rate_value:.2f}",
            ('' if not math.isfinite(self.best_range)
             else f"{self.best_range:.2f}"),
            f"{error_value[0]:.2f}", f"{error_value[1]:.2f}", f"{error_value[2]:.2f}",
            f"{math.hypot(error_value[0], error_value[1]):.2f}",
            f"{float(ek.get('ofs_scale', 0.0)):.3f}",
            f"{v_raw[0]:.2f}", f"{v_raw[1]:.2f}", f"{v_raw[2]:.2f}",
            f"{v_ned[0]:.3f}", f"{v_ned[1]:.3f}", f"{v_ned[2]:.3f}",
            f"{float(np.linalg.norm(v_ned)):.3f}",
            f"{math.hypot(v_ned[0], v_ned[1]):.3f}",
            int(ek.get('clamp_horizontal', 0)),
            '' if not math.isfinite(ft) else f"{ft:.2f}",
            f"{self.yaw_applied_value:.2f}",
            '' if area_value is None else f"{area_value:.0f}", f"{area_rate:.1f}",
            '' if self.s_lpf is None else f"{self.s_lpf:.1f}",
            f"{self.s_peak:.1f}",
            f"{measurement.bbox_age_s:.3f}" if math.isfinite(measurement.bbox_age_s) else '',
            '' if measurement.vibe_max is None else f"{measurement.vibe_max:.1f}",
            1 if self.hit_value else 0,
            '' if measurement.pitch_rad is None else f"{math.degrees(measurement.pitch_rad):.2f}",
            '' if measurement.yaw_rad is None else f"{math.degrees(measurement.yaw_rad):.2f}",
            '' if measurement.pos_ned is None else f"{-float(measurement.pos_ned[2]):.2f}",
        ])
        if self.counter % 20 == 0:
            self._diagnostic_f.flush()


# =============================================================== main

def main():
    p = argparse.ArgumentParser(
        description="ArduPilot Visual guidance with FOLLOW law")
    p.add_argument('--duration-value', type=float, default=None)
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--law', choices=('classical', 'poscon'), default=None,
                   help="classical = Copter <= 4.4 pure P; poscon = Copter >= 4.5 "
                        "sqrt_controller (classic default)")
    p.add_argument('--kp', type=float, default=None,
                   help='Equivalent to FOLL_POS_P (default 1.0 ; AP 0.1 )')
    p.add_argument('--braking', choices=('disabled', 'ap', 'range_value'), default=None,
                   help='approach brake ( AC_Avoid :: limit_velocity_2D ); '
                        'default off -- for multiplication')
    p.add_argument('--speed-source', choices=('ceiling_value', 'p'), default=None,
                   help="ceiling = plane_follow architecture (direction from law, speed "
                        "from the navigational ceiling; DEFAULT); p = pure mode_follow "
                        "(|v| = kp*error -- cannot hit moving target)")
    p.add_argument('--acceleration-shape', type=float, default=None,
                   help='instruction size increase limit [m/s^2]; 0=off')
    p.add_argument('--ofs-backward', type=float, default=None,
                   help='FOLL_OFS back [m], LOS frame (default 0 )')
    p.add_argument('--ofs-down', type=float, default=None,
                   help='FOLL_OFS down [m] (default 0)')
    p.add_argument('--yaw-p', type=float, default=None,
                   help='Equivalent to ATC_ANG_YAW_P (default is 4.5 )')
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw COMMAND (ablation: remains on autopilot)')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--mount', type=float, default=None)
    p.add_argument('--speed-ceiling', type=float, default=None)
    p.add_argument('--no-impact', action='store_true')
    p.add_argument('--no-miss', action='store_true')
    p.add_argument('--miss-source', choices=('range_value', 'area_value'), default=None,
                   help="What should the MISS referee read: 'range' (default, MPC "
                        "Same criterion as) | 'area' = bbox area rate, RANGE "
                        "NO READ -- entire system operates without range")
    p.add_argument('--miss-time-source', choices=('straight', 'progress'),
                   default=None,
                   help="what should timeout count: 'plain' (wall clock, "
                        "default) | 'progress' = time only target "
                        "Render visible area WITHOUT enlargement (range is not read)")
    p.add_argument('--miss-time-timeout', type=float, default=None,
                   help='MISS timeout [s] (default 8, same as MPC). '
                        'REQUIRES 15 ON PRINCIPAL TARGET: handoff to STATIONAL vehicle '
                        '(seed rate 0) and with command 3 m/s^2 '
                        '20 m 8 does not fit into s\' because it is ramped -- measured, '
                        'angajmanlar 5.6/4.6 m\'de kesildi. MPC\'nin asili '
                        'The target record (0.47 m) was also taken with a threshold of 15 s.')
    p.add_argument('--miss-redis-key', default=None)
    p.add_argument('--diagnostic-log', default=None)
    args = p.parse_args()

    config_value = TrackingConfig()
    if args.law is not None:
        config_value.law = args.law
    if args.kp is not None:
        config_value.kp = args.kp
    if args.braking is not None:
        config_value.braking = args.braking
    if args.speed_source is not None:
        config_value.speed_source = args.speed_source
    if args.acceleration_shape is not None:
        config_value.acceleration_shaping_mps2 = args.acceleration_shape
    if args.ofs_backward is not None:
        config_value.ofs_backward_m = args.ofs_backward
    if args.ofs_down is not None:
        config_value.ofs_down_m = args.ofs_down
    if args.yaw_p is not None:
        config_value.yaw_p = args.yaw_p
    if args.no_yaw:
        config_value.yaw_command_provide = False
    if args.aim is not None:
        config_value.aim_deg = args.aim
    if args.mount is not None:
        config_value.mount_pitch_deg = args.mount
    if args.speed_ceiling is not None:
        config_value.speed_ceiling_mps = args.speed_ceiling
    if args.no_impact:
        config_value.impact_mode = False
    if args.no_miss:
        config_value.miss_mode = False
    if args.miss_source is not None:
        config_value.miss_source = args.miss_source
    if args.miss_time_source is not None:
        config_value.miss_time_source = args.miss_time_source
    if args.miss_time_timeout is not None:
        config_value.miss_time_timeout_s = args.miss_time_timeout
    if args.miss_redis_key is not None:
        config_value.miss_redis_key = args.miss_redis_key
    config_value.__post_init__()                 # validate the explicitly overridden fields

    print(f"[follow] ArduPilot FOLLOW law: law={config_value.law} kp={config_value.kp:.2f} "
          f"speed_source={config_value.speed_source} brake={config_value.braking} "
          f"ofs=(back {config_value.ofs_backward_m:.1f} / down "
          f"{config_value.ofs_down_m:.1f} m, melting {config_value.ofs_reduction_range_m:.0f} m)"
          + (f" acceleration_shape={config_value.acceleration_shaping_mps2:.1f} m/s^2"
             if config_value.acceleration_shaping_mps2 > 0 else ""))
    print(f"[tracking] speed cap={config_value.speed_ceiling_mps:.1f} m/s "
          f"(same source as guidance_config), climb/descend="
          f"{config_value.climb_ceiling_mps:.0f}/{config_value.descent_ceiling_mps:.0f} m/s, "
          f"yaw_p={config_value.yaw_p:.1f} ceiling={config_value.yaw_speed_ceiling_dps:.0f} dps")
    print(f"[tracking] miss mode={'ENABLED' if config_value.miss_mode else 'DISABLED'}, "
          f"beat phase={'ENABLED' if config_value.impact_mode else 'DISABLED'} "
          f"({config_value.impact_range_m:.0f} m -> {config_value.impact_full_range_m:.0f} m), "
          f"assembly={config_value.mount_pitch_deg:+.1f} aim={config_value.aim_deg:+.1f}")

    stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
    diagnostic = args.diagnostic_log or str(Path(__file__).resolve().parent / 'logs'
                                / f"tracking_diagnostic_{stamp_value}.csv")
    from visual_base import VisualLoop
    VisualLoop(TrackingController(config_value, diagnostic_log=diagnostic),
                   loop_hz=args.loop_hz).run_value(args.duration_value)


if __name__ == '__main__':
    main()
