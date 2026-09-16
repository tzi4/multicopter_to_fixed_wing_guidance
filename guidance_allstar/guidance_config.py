# ============================================================
#  Guidance Configuration
#
#  Layout: every parameter USED by the active pipeline
#  (simple_guided_follow.py + filterwndr.py) is at the top, grouped by
#  subsystem. Parameters that are NOT referenced by the active pipeline are
#  commented out and pushed to the bottom (see "UNUSED" section). Many of the
#  unused ones belong to the retired lag_pursuit_pid.py / pronav_runner.py /
#  velocity_control.py runners; uncomment them if you revive those.
# ============================================================

# --- Guidance loop ---
LOOP_HZ = 30  # guidance / GUI update rate [Hz]

# --- MAVLink connections --- STARS ENVIRONMENT ADAPTATION ( multicopter_to_fixed_wing_guidance /
# yildizlar_guidance .sh) In this environment, the ports and SysID 's are different; the values are
# defined in ONE place there: fighter drone_1 SysID 1 -> 14550 (QGC) 14551 (tools) 14651 (wait) 14652
# (BU) target fixed wing SysID 6 -> 14550 (QGC) 14601 (tools) 14602 (plan) 14603 (BU) Guidance uses
# its own PORT: 14551 / 14601 to manual command tools ( tools/ swarm_command .py) belonging to; No two
# processes can connect the same UDP port.
PURSUER_CONN_STR = "udpin:127.0.0.1:14652"
TARGET_CONN_STR = "udpin:127.0.0.1:14603"
# Visual guidance processes ( LOS / PID / MPC based on visual_base .py ) stand at the same time as
# positioned; udpin cannot connect the same port twice. So they use their own mavproxy exit
# (yildizlar_guidance .sh):
VISUAL_PURSUER_CONN_STR = "udpin:127.0.0.1:14654"
VISUAL_TARGET_CONN_STR = "udpin:127.0.0.1:14604"
# Size clamp of display speed command [ m/s ];  It must come from the SAME SOURCE as WPNAV_SPEED
# (params/ swarm_copter .parm), if it separates, either the clamp works in vain or we cannot use what
# the vehicle can do.  18 -> 20 ( 2026 - 08 - 03 ): Measurement of agent LOS, in narrow lead ( 60 deg
# )
#   v_max=18 -> 22.4 m KACIRMA, v_max=20 -> 1.5 m isabet.
# 20 -> 35 (2026-08-05): 20/20 SPEED PARITY was the root of all 'uncatchable' findings (target
# aircraft 20 m/s cruise). The actual hardware flies with 24 (formation_KILLER
# NAV_SPEED/ATTACK_SPEED_MPS=24) and 35 m/s has also been seen in flight. 24 MEASURED in Sim: 24.15
# m/s, bank 18.5 deg (ceiling 60), altitude loss 0.1 m -> ceiling was not attached. Increased to 35.
# NOTE: this is a CLAMP, NOT A CLAIM -- MPC does not say "ceiling 35, 35 head"; cost uses whatever it
# wants, clamp only interrupts unreachable commands. So raising does not impose aggression, it removes
# the constraint.
VISUAL_MAX_SPEED_MPS = 35.0
TARGET_EXPECTED_SYSID = 6  # target aircraft in the stars environment SysID 6 (was 2 in the repo)
# Hunter is also verified: only drone_1 (SysID 1) should flow to 14652, but if the port is given
# incorrectly or a GCS heartbeat comes first, the script silently commands the WRONG broker. The
# hunter's equivalent of control (above) on the target.
PURSUER_EXPECTED_SYSID = 1  # fighter copter in the stars environment drone_1 -> SysID 1

# --- MAIN TARGET KEY (TARGET_VEHICLE) ------------------------------- Moves the target role from the
# fixed wing aircraft (SysID 6) to the drone_2 copter (SysID 2). REASON: the plane is fixed wing, it
# really CANNOT STOP; The "target suspended in the air" scenario can only be set up with a copter
# (purple image added to drone_2 in 5d3d127). The only thing that changes is WHICH VEHICLE's telemetry
# is being read -- the scope of the INFORMATION being read does not change: the imaging side still
# derives ONLY RANGE from the target (visual_base.RangeEstimator.range, single pull-out
# measurement).
#
# The ports come from the mavproxy fanout of yildizlar_guidance.sh: 14651+10i / 14652+10i / 14653+10i /
# 14654+10i for drone_i (i = sysid-1). So drone_2: positional target -> 14662 (corresponding to 14603
# in aircraft) visual target-> 14664 (corresponding to 14604 in aircraft) Fighter ports (14652/14654)
# DO NOT CHANGE: fighter always drone_1.
#
# DEFAULT BEHAVIOR: if env is absent/empty, the block does not work at all, the plane remains the target.
import os as _os

_TARGET_VEHICLE = _os.environ.get("TARGET_VEHICLE", "").strip().lower()
if _TARGET_VEHICLE in ("drone2", "drone_2", "2"):
    TARGET_CONN_STR = "udpin:127.0.0.1:14662"
    VISUAL_TARGET_CONN_STR = "udpin:127.0.0.1:14664"
    TARGET_EXPECTED_SYSID = 2
elif _TARGET_VEHICLE not in ("", "plane", "plane", "target_value"):
    raise ValueError(
        f"TARGET_VEHICLE not recognized: {_TARGET_VEHICLE!r} "
        "(valid: empty/aircraft = fixed wing, drone2 = hovering copter target)")

# --- Runtime param assertion for GUIDED intercept sim --- ATTENTION: its name is "ASSERT" but
# assert_startup_params() DOES NOT READ them, WRITES them (simple_guided_follow.py:715 -> set_param).
# So this dictionary CRUSHES the values ​​in params/ swarm_copter.parm at runtime. So the stars are
# aligned with the aggressive envelope of their environment (see params/swarm_copter.parm):
# WPNAV_ACCEL 600 -> 500 : In ArduPilot the parameter range [50..500], 600 is silently clipped; with
# the trimmed value, compute_terminal_extension_distance() was calculating the braking distance LESS
# 20%. ANGLE_MAX 4500 -> 6000 : measured, iris SITL 38 m/s at 26-39 degrees lies and holds altitude;
# 45 was unnecessarily narrowing the rating ceiling. WPNAV_SPEED_UP/DN, ACCEL_Z retreated to the
# ceiling (so that the vertical maneuver does not lose the target). WP_YAW_BEHAVIOR 0 PROTECTED:
# gimbal SINGLE AXIS (tilt), horizontal framing is still completely dependent on the airframe yaw;
# Turning the nose onto the course by the autopilot takes the target out of the SIDE of the frame. The
# direction will be locked from --yaw-lock to LOS. (Gimbal branch 2026-08-05: the vertical axis is now
# in the gimbal, this justification stands for the horizontal axis ONLY.) --- TRACKING PHASE SMOOTHING
# (2026-08-01 measurement) --------------------- params/swarm_copter.parm pulls acceleration/jerk TO
# THE CEILING; this is correct in the CLOSE phase (distance 1000 m -> 25 m), but it was running out of
# camera in the STANDOFF phase during the OLD BODY-FIXED CAMERA period: trial 20260801_141115 (with
# ceiling values, measured during the OLD BODY-FIXED CAMERA period): body pitch %5 -42.5 deg, %95
# +39.4 deg -> 81.9
#   degree oscillation. The vertical framing of the camera is only 12.95 degrees. So the target is ON
#   AVERAGE on the camera axis (target - axis = +1.5 deg)
#   Even though it is, it is thrown out of the frame frame by frame; The detection rate remained at
#   4%. At that time, the virtual gimbal COULD NOT SAVE this: it mathematically subtracts the body
#   pitch, but could not bring back the target that was out of frame because the physical camera was
#   tilted with the body (same detection recorded in bumblebee/teva.py:702-707). [GIMBAL BRANCH
#   2026-08-05] PHYSICAL single-axis tilt gimbal CLOSED this vertical mechanism: the camera stabilizes
#   itself, measured in flight as the body -35.4..+35.2 deg skids around the world pitch to max |0.65|
#   deg (see GIMBAL_NOTES.md). So PITCH oscillation no longer distorts vertical framing. The
#   smoothing still remains, but the reasoning has changed: (a) the ROLL is still reflected in the
#   image (single axis), (b) the horizontal framing is tied to the airframe yaw and the hard command
#   profile offsets the yaw, (c) the soft profile also calms the guess/april chain. The vertical
#   justification is NO LONGER INVALID; This must be measured AGAIN on the gimbal branch if one wants
#   to return to ceiling values. The speed ceiling (WPNAV_SPEED) and the lean ceiling (ANGLE_MAX 6000)
#   remain in place, i.e. the ability to close is maintained, only the command PROFILE is softened.
GUIDED_STARTUP_PARAM_ASSERTS = {
    "WP_YAW_BEHAVIOR": 0,
    "WPNAV_SPEED": 3500,     # 2026-08-05: 2000 -> 2400 (same as actual hardware).
                             # CAUTION: this dictionary CRUSHES the .parm file at run time; Changing params/swarm_copter.parm ALONE
                             # WILL NOT WORK, this should also be changed.
    # 2026 - 08 - 08 : Removed 250 / 5 / 250 anti-aliasing -> returned to file ceilings. Rationale: (a)
    # the cause of the anti-aliasing was the body-fixed camera (block above); invalid in gimbal branch.
    # (b) User request: aggressive profile. (c) Measured: this dictionary was crushing the parm file and
    # the write from MAVLink EVERY RUN — the runs labeled "arm 500" were actually blown by 250 / 5. Pitch
    # swing will be remeasured in the gimbal branch (verification list MPC_NOTES .md).  2026 - 08 - 08
    # night REVISED — "measured aggressive": full-roof profile ( 500 / 20 / 500 / 6000 ) MEASURED to break
    # the camera chain 17 running, 162 engagement): roll RMS 13.5 -> 26 , roll RMS vs freshness r=- 0.851
    # ;  Vertical thrust collapses at 50 - 60 deg roll (6x ALTITUDE ABORT + ground contact, eyl1b_203809
    # ). Horizontal acceleration REMAINS (critical for turning; the measured plateau ~ 4 m/s2 is already
    # achieved by banking ~ 22 deg), the oscillation-generating axes are set:
    "WPNAV_ACCEL": 500,      # file ceiling — REMAINS HORIZONTAL
    "WPNAV_JERK": 3,
    "PSC_JERK_XY": 10,       # 20 ceiling back; 5 old ultra-soft, 10 middle ground
    "WPNAV_SPEED_UP": 1000,
    "WPNAV_SPEED_DN": 500,
    "WPNAV_ACCEL_Z": 500,    # 2026 - 08 - 08 night- 2 : 250 attempt WITHDRAWED — vertical
                             # cutting the budget INCREASED the ABORT (H1: 8 abort; H2: t=34 somersault at s, roll -131).
                             # Vertical authority sustains the attitude; 500 remains.
    "ANGLE_MAX": 4500,       # 60 deg was oscillatory pathology; 45 deg = 9.8 m/s2 ceiling,
                             # still 2 times above the measured 4 m/s2 plateau
    # Permanent immunity to POSHOLD pinwheel crash (pilot modes only; no effect on GUIDED performance).
    # See SHUTDOWN_MODE review.
    "PILOT_SPEED_DN": 100,
    # 2026 - 08 - 08 user request: Safety failsafes in the SIM were breaking the test flow (EKF failsafe
    # -> LAND was changing the mode mid-run, crash-check disarm was breaking the chained runs). SIM-ONLY
    # relaxation:
    "FS_EKF_ACTION": 2,     # AltHold instead of EKF failsafe LAND (no mod hijacking)
    "FS_CRASH_CHECK": 1,    # 2026-08-08 night-2: RE-ENABLED. With 0, a vehicle that overturned
                            # The armed mold produced 6 min null data (olcag2_215920). The landing-out problem has already been
                            # solved with SHUTDOWN=BRAKE; Crash-check now only fires during a true roll.
    "FS_THR_ENABLE": 0,     # RC/throttle failsafe off (SITL bars virtual)
}
# Upper limit of commanded speed [m/s]. WPNAV_SPEED = 2400 must be the SAME as cm/s: if the script
# commands a higher feedforward speed, the difference is overlaid on the vehicle's internal position
# controller (default --max-feedforward-speed). In the past, this name WAS NOT in the config and
# simple_guided_follow.py would silently drop to 25.
SPEED_MAX = 35.0

# --- Target telemetry ---
TARGET_MESSAGE_RATE_HZ = 15.0  # requested GLOBAL_POSITION_INT rate for the target [Hz]
# Which altitude field of the target's GLOBAL_POSITION_INT to believe.
#
# True  = relative_alt, the target's height above ITS OWN home. The pursuer's
#         LOCAL_POSITION_NED z is height above ITS home, so as long as both
#         aircraft launched from the same field the two are directly comparable
#         -- and it does not matter one bit whether they agree on what that
#         field's AMSL elevation is.
# False = alt (AMSL) minus the pursuer's home AMSL. Mathematically equivalent
#         ONLY if both vehicles derived the same home elevation.
#
# WHY THIS DEFAULTS TO True (2026-07-31 flight): the two aircraft disagreed
# about the elevation of the SAME field by 36.80 m -- pursuer home 115.02 m
# AMSL, target home 78.22 m AMSL, each set from its own GPS fix (tens of metres
# of GPS vertical error is routine). In AMSL mode that error lands directly in
# the vertical command: guidance believed the target was 36.8 m lower than it
# was and flew the drone 36.8 m below it, constant through climbs and descents.
# It was invisible everywhere -- the estimator tracked the reported position to
# 0.06 m, the logs showed the drone exactly where commanded, and Mission Planner
# showed BOTH AIRCRAFT AT THE SAME AMSL while they were 36.8 m apart in the air.
# Relative altitude cannot express that failure at all.
#
# Set False only if the two aircraft launch from genuinely different elevations
# (then AMSL is the correct common frame, and their home altitudes must agree).
TARGET_ALT_USE_RELATIVE = True
# Startup sanity check: warn when the two vehicles' HOME_POSITION altitudes
# differ by more than this [m]. Diagnostic only -- with the relative-altitude
# path above a mismatch is harmless, but it is a loud early symptom of a bad GPS
# fix, and it is critical if TARGET_ALT_USE_RELATIVE is ever set False. 0 disables.
HOME_ALT_MISMATCH_WARN_M = 5.0
# Own-position staleness guard (2026-07-24 review): the target feed always had
# a stale-hold; the pursuer's own LOCAL_POSITION_NED did not, so a frozen own
# position silently fed phantom range/t_go/CPA AND defeated the altitude abort
# (alt_error uses the frozen z). Older than this -> hold setpoints + warn.
PURSUER_STALE_TIMEOUT_S = 1.0

# --- Slot / position-only default ---
POSITION_ONLY_DEFAULT = False  # keep simple_follow on position targets by default
LAG_PURSUIT_DIST = 0.0  # distance [m] behind the target for the slot setpoint

# --- IMM Low Pass Filter (guidance-side smoothing of estimator output) ---
IMM_LPF_ENABLED = False  # enable low pass filter for IMM outputs in simple_follow
IMM_LPF_ALPHA = 0.8  # LPF smoothing factor (0.0 = frozen, 1.0 = raw IMM)
IMM_LPF_MOTION_COMPENSATED = False  # predict LPF state forward before smoothing to cut steady-state lag

# --- Guidance lead-prediction horizon ---
# 0.75 s (was 6.0 -> 3.0 -> 0.75). Two independent reasons, both from the
# 2026-07-11 replay:
#  (1) Smoothness: the aim point is a projection of the estimated velocity
#      heading forward by the horizon, so aim jitter ~= horizon * heading-noise.
#      At 2-3 s the COMMANDED-setpoint turn rate (what sheds copter yaw) stayed
#      ~3500 deg/s even with a low-pass filter; at 0.5-0.75 s + LPF it drops to
#      ~30-150 deg/s (trackable). Horizon length is the dominant jitter term.
#  (2) Geometry: pursuer (~20 m/s) and target (~19 m/s) are near co-speed, so a
#      long lead cannot cut the corner to intercept -- it only commits the aim
#      to a mispredicted point during maneuvers. Mid-course is near-pursuit; the
#      deliberate terminal spear (extension + freeze latch, true-geometry
#      triggered) does the actual intercept, so a short lead loses nothing.
# The horizon still tightens to t_go below this ceiling (-> 0 at intercept).
TERMINAL_PREDICTION_MAX_S = 0.75  # hard ceiling on guidance lead prediction [s]
GUIDANCE_LEAD_PREDICT_SUBSTEP_S = 0.25  # IMM substep for the lead prediction only [s]; the filter's own predict keeps PREDICT_MAX_SUBSTEP (0.1)
# Low-pass the aim point at loop rate (time constant, so loop-rate independent):
# alpha = dt/(dt+tau). This is the PRIMARY jitter smoother -- a rate cap bounds
# aim SPEED but not direction reversals (the yaw-shed driver), a low-pass bounds
# both. 0 disables (falls back to the AimPointRateLimiter backstop only).
LEAD_AIM_LPF_TAU_S = 0.5
TERMINAL_TURN_ENTRY_CT_MU_MAX = 0.55  # treat rising CT below this as low-confidence turn entry
TERMINAL_TURN_ENTRY_DMU_MIN = 0.02  # minimum positive mu_ct rise to declare turn entry
# NB: inert while TERMINAL_PREDICTION_MAX_S (0.75) < this cap -- the horizon is
# already clamped below it. Kept for the day the max horizon is raised again.
TERMINAL_TURN_ENTRY_HORIZON_CAP_S = 1.25  # cap prediction horizon during low-confidence turn entry

# --- Lead construction (2026-07-13): build the aim point from the well-observed
#     states (position, velocity, turn rate omega), bound the poorly-observed
#     ones (linear acceleration, and the CT centripetal via omega) to the
#     target's flight envelope, and shrink the lead toward the current estimate
#     when the filter's OWN predicted covariance says the extrapolation is
#     guesswork. This is the principled generalization of the earlier hard
#     vertical pin + fixed horizon cap: it kills the transient amplification at
#     its source (a small spurious az * t^2/2 made the aim swing +-100 m on
#     2026-07-11) and self-adapts when a noisy real sensor replaces the test's
#     clean telemetry (higher measurement noise -> larger predicted sigma ->
#     automatically shorter effective lead). ---
LEAD_ENVELOPE_CLAMP_ENABLED = True
# Lateral cap bounds BOTH the horizontal linear accel |a_xy| AND the CT
# centripetal accel v*omega used in the lead (omega <= LAT/speed). 5 m/s^2 ~=
# 30 deg bank at 19 m/s. NOTE OMEGA_ABS_MAX=1.5 rad/s alone permits v*omega =
# 28 m/s^2 (2.9 g) at 19 m/s -- far past any real airframe -- so this cap, not
# the raw omega clamp, is what makes the lead's turn arc realistic.
LEAD_LATERAL_ACCEL_MAX_MPS2 = 5.0
LEAD_VERTICAL_ACCEL_MAX_MPS2 = 4.0   # cap on |az| used in the lead [m/s^2] (~1.5x Iris WPNAV_ACCEL_Z); a backstop, since vertical accel is dropped from the lead by default
# Per-channel inclusion (0..1): how much of the (clamped) state to actually
# extrapolate. Horizontal accel kept (bounded); vertical accel + vz dropped,
# because the target flies ~level so their true value is ~0 and any nonzero
# estimate is transient noise the t^2/t horizon amplifies. Raise to A/B.
LEAD_HORIZONTAL_ACCEL_SCALE = 1.0
LEAD_VERTICAL_ACCEL_SCALE = 0.0
LEAD_VERTICAL_VELOCITY_SCALE = 0.0
# Covariance gate: predicted position 1-sigma (per axis group) below LO -> full
# lead; above HI -> lead fully suppressed (aim at the current estimate); linear
# in between. Floor keeps a minimum lead fraction so a confident-but-noisy patch
# doesn't drop the lead to zero. Tuned from 2026-07-11 replay (see tests).
# Thresholds set from the 2026-07-11 replay: on the test's clean telemetry the
# predicted horizontal sigma sits at 8-16 m (median 8) and vertical at 6-9 m, so
# LO is placed just above that band -> the gate is DORMANT on clean data (full
# lead, correct) and only engages as sigma climbs, which is what a noisy real
# sensor does (higher R -> larger predicted sigma -> automatically shorter lead).
LEAD_COV_GATE_ENABLED = True
LEAD_COV_GATE_H_SIGMA_LO_M = 15.0
LEAD_COV_GATE_H_SIGMA_HI_M = 50.0
LEAD_COV_GATE_V_SIGMA_LO_M = 10.0
LEAD_COV_GATE_V_SIGMA_HI_M = 30.0
LEAD_COV_GATE_MIN_FRAC = 0.0

# --- Terminal position extension (push the aim point past the target so GUIDED
#     does not decelerate to a stop on it) ---
TERMINAL_POSITION_EXTEND_RANGE_M = 60.0  # range at/below which the extension becomes active [m]
TERMINAL_POSITION_EXTEND_DISTANCE_M = 15.0
TERMINAL_POSITION_EXTEND_BRAKE_MARGIN_M = 10.0  # extra margin on top of v^2/(2a) when sizing extension
TERMINAL_POSITION_EXTEND_MAX_M = 30.0
# Blend the extension in over this band below the activation range instead of
# stepping the aim ~30 m in one loop (the step spikes tilt+collective at speed;
# motor saturation then sheds yaw authority first -- 2026-07-15 pos+vel logs).
# 0 restores the legacy instant step.
TERMINAL_EXTEND_BLEND_BAND_M = 10.0

# --- Terminal freeze-&-spear latch ---
TERMINAL_LATCH_TGO_S = 0.4  # freeze the aim point when estimated time-to-go falls below this [s]
TERMINAL_LATCH_RELEASE_RANGE_M = 25.0  # release the frozen aim point after a pass before range opens back up [m]
# Once the aim is FROZEN, the velocity feedforward must go with it: commanding a fixed point while
# still feeding ~ 20 m/s of target velocity is self-contradictory, and once the pursuer overshoots
# that point the position error reverses while the FF still pushes forward -- the controller fights
# itself, pitches back hard at high thrust and balloons (log 110805 ). Zeroing the FF turns the
# latched phase into a clean "fly through to this point and stop", which is what the extension already
# intends.  False restores the old behavior.
TERMINAL_LATCH_ZERO_VELOCITY_FF = True

# --- Terminal fly-through climb-from-below (2026-07-28, replaces freeze+brake) ---
# The freeze-&-stop terminal above braked a fast approach into a frozen point:
# from 34 m/s that pitch-back brake drove an UNCOMMANDED +22 m balloon (target
# dead level) -> collective saturation -> yaw authority shed -> tumble -> crash
# (log 20260728_145054; a slower 16.7 m/s latch the same flight survived). The
# balloon ADDS energy on a miss, worsening the crash. New strategy: never brake
# or freeze in the terminal -- fly THROUGH at co-speed while climbing UP into the
# target from underneath. Approach SLOT_DOWN_OFFSET_M below the target, then in
# the terminal window aim TERMINAL_UP_OFFSET_M above it and command an upward
# velocity, so the intercept is a climb from below. A miss then becomes an upward
# ZOOM that gravity decelerates and stabilises (recoverable), not a forward
# over-speed the brake balloons. The climb rate is deliberately BOUNDED (well
# under the copter's climb ceiling) so collective keeps reserve for yaw/attitude
# control -- saturation is the whole crash chain, so we approach max effort but
# never reach it. False restores the legacy freeze+brake latch.
TERMINAL_FLYTHROUGH_ENABLED = True
SLOT_DOWN_OFFSET_M = 3.0        # default --down: approach this far BELOW the target [m]
TERMINAL_CLIMB_RATE_MPS = 4.0   # commanded upward velocity in the terminal window [m/s].
                                # HARD MAX is WPNAV_SPEED_UP = 5.0 m/s (the copter's climb
                                # ceiling; ArduCopter clamps any command above it). Keep this
                                # UNDER 5 so throttle keeps reserve for yaw/attitude control --
                                # commanding the full 5 would drive collective to saturation,
                                # which is the entire yaw-shed crash chain. 4.0 = 5 - 1 margin.
TERMINAL_UP_OFFSET_M = 2.0      # aim this far ABOVE the target in the terminal, to commit to
                                # climbing THROUGH its altitude (hit from under, don't level off)

# --- KILL MODE: master switch for the whole terminal intercept (2026-07-31) ---
# ON  = intercept. The full terminal chain runs: LOS extension/spear, the
#       terminal window, the fly-through climb-from-below, and the slot sits ON
#       the target (LAG_PURSUIT_DIST) so the vehicle converges to a hit.
# OFF = SIMPLE FOLLOW, for real-world testing of tracking/estimation/comms
#       without the vehicle ever being aimed at the aircraft. Nothing in the
#       terminal chain runs, and the slot is held at a STANDOFF behind and
#       below the target (FOLLOW_STANDOFF_*) instead of on it -- with kill mode
#       off but the intercept slot geometry (back=0) still in force, the
#       vehicle would fly to the target's exact position, which is a collision
#       course by another name.
# Everything protective stays live in BOTH modes: turn clamp, carrot clamp,
# super_safe_turn, Z slew, aim/altitude governors, the altitude-divergence
# abort, mission failsafe and impact/crash detection. The miss-recovery FSM
# degrades on its own with kill mode off (no latch -> no miss, no CPA), while
# its altitude abort keeps protecting the vehicle.
# Toggle at runtime with --kill-mode / --no-kill-mode; the startup banner and
# the CSV kill_mode column both record which one flew.
KILL_MODE_ENABLED = True
FOLLOW_STANDOFF_BACK_M = 15.0  # follow-test slot: hold this far BEHIND the target [m]
FOLLOW_STANDOFF_DOWN_M = 5.0   # follow-test slot: hold this far BELOW the target [m] (out of its wake; a loss of control falls away, not into it)
# NOTE (vertical geometry convention): the above binary is a valid substitute only if
# simple_guided_follow.py runs WITHOUT ARGUMENTS. In camera runs (yildizlar_guidance.sh /
# tools/scenario.sh) --back/--down is ALWAYS given externally. [GIMBAL BRANCH 2026-08-05 - DEPENDENCE
# DIRECTION REVERSED] The camera is no longer fixed to the body, but is on the self-stabilizing
# physical single-axis (tilt) gimbal. PREVIOUSLY down HAD TO comply with fixed camera angle:
#   down = round(back * tan(MOUNT + PITCH_TRIM)),  MOUNT=+30 deg (model.sdf),
#   PITCH_TRIM=-2.5 deg -> back=25 for down=13 (measured at that time: detection %98.4; In down=3, the
#   target axis 22 deg was missing and the detection dropped to %5). NOW INVERSE: DOWN is a free
#   MISSION DESIGN button, the camera angle is derived from it: YILDIZ_TILT_DEG = atan(DOWN / BACK)
#   (world elevation, positive = up). bbox_to_redis.py issues the tilt command; gimbal plugin
#   compensates for body pitch, 0 in MOUNT model.sdf. The ONLY SOURCE of derivation is still
#   scripts/standoff_geom.sh. Since this place does not know the camera, these numbers will NOT be
#   connected there; If you fly a camera run with this spare pair, the spare geometry will fly, NOT
#   the geometry designed by the scenario, since the tilt will also be derived from this pair
#   (back=15/down=5 -> tilt 18.4 deg).

# --- Terminal-entry geometry gate (2026-07-31) ---
# The terminal window (latch arm / fly-through climb) may only OPEN when the
# engagement geometry is actually healthy. The 2026-07-29 crash logs show
# latch-ONs firing with closing velocity ~0-2 m/s and the vehicle 26+ m off its
# commanded altitude -- a near-vertical "intercept" the climb-from-below then
# makes worse. Committing to the terminal from bad geometry converts a
# recoverable approach into a saturation event. Both gates apply only to
# ARMING; an already-active terminal window still releases normally. 0 disables
# the individual check.
TERMINAL_ENTRY_MIN_CLOSING_MPS = 3.0  # don't open the terminal window unless actually closing this fast [m/s]
TERMINAL_ENTRY_MAX_ALT_ERR_M = 8.0    # ...or while the vehicle is further than this from its commanded altitude [m]

# --- Miss recovery (CHASE -> HOLD -> REENGAGE -> CHASE state machine) ---
MISS_RECOVERY_ENABLED = True  # on by default; toggle with --miss-recovery / --no-miss-recovery
RECOVERY_MODE = "BRAKE"  # flight mode during the HOLD window (BRAKE recommended; STABILIZE / LOITER also work)
RECOVERY_HOLD_S = 2  # seconds to hold in the recovery mode to bleed speed / level out
RECOVERY_CHASE_DWELL_S = 15.0  # CHASE must run this long before a miss may trigger HOLD/BRAKE (debounces the 12x BRAKE thrash of 2026-07-11)
RECOVERY_BRAKE_MIN_INTERVAL_S = 5.0  # minimum spacing between HOLD/BRAKE activations [s]
# Altitude-divergence abort (2026-07-15): the range-based miss trigger is blind
# to the actual crash mode on aggressive configs -- motor saturation from the
# hard horizontal chase steals collective, the copter can't hold its commanded
# altitude, and it falls out of the sky while still within horizontal range (log
# 112728: pursuer fell 65 m, commanded steady at -49, HOLD fired 0.9 s too late
# on range). This fires HOLD/BRAKE the instant |pursuer_z - commanded_z| exceeds
# the threshold -- the RIGHT signal (vertical tracking failure), bypassing the
# miss dwell/spacing debounce (a thrust collapse is an emergency, not latch
# noise). Arms only after altitude is first acquired (< ARM_M) so the initial
# climb doesn't trip it, and disarms on fire until altitude is re-acquired (no
# thrash). Set from 112728: self-recovered sags peaked <=12 m, the fatal run
# blew through 15 m to 62 m, so 15 m sits in the gap. 0 disables.
# 15.0 -> 8.0 (2026-07-24): with the CPA trigger below now catching the pass
# itself, this is a backstop rather than the primary trigger, so it can sit
# lower and catch a divergence while BRAKE can still arrest it. At 15 m the
# vehicle was already ballooning past recovery (log 110805).
# 8.0 -> 12.0 (2026-07-28): the fly-through terminal now COMMANDS a climb of
# SLOT_DOWN_OFFSET_M + TERMINAL_UP_OFFSET_M (~5 m) up through the target; this
# backstop only fires on a genuine runaway (>12 m off the approach altitude),
# not the intended zoom. It measures vs the pre-terminal approach altitude.
RECOVERY_ALT_ABORT_M = 15.0
RECOVERY_ALT_ABORT_ARM_M = 3.0  # abort arms before |pursuer_z - commanded_z| first drops below this [m]
# CPA (closest-point-of-approach) trigger, 2026-07-24. The old miss signal was the latch RELEASING at
# range >= TERMINAL_LATCH_RELEASE_RANGE_M (40 m) -- which is 2-6 s AFTER the pass, during which the
# pursuer is fighting a frozen aim point it has already flown past (position error reversed while the
# velocity FF still commanded ~ 20 m/s forward) -> pitch - back -> balloon ( 50 -> 95 m, log 110805 ).
# Instead, declare the pass once the range has climbed this far above the minimum seen while latched,
# and hand off to recovery.  0 disables, falling back to the latch-release miss.
#
# REVIEW FIX (2026-07-24): the trigger consumes ONLY time-consistent range
# samples -- taken on target-packet loops with the pursuer dead-reckoned to
# now via its own velocity. The naive every-loop range is a +-5-8 m staircase
# (target estimate frozen between ~2.4-4 Hz packets, own position ~3 Hz), and
# 44% of latched packet rows jumped it past this margin BEFORE the true pass,
# which would have aborted 1.3-2.7 m near-hits into ~9-14 m misses (log
# 110805, full-res). It additionally requires RECOVERY_CPA_OPEN_SAMPLES
# consecutive beyond-margin samples so one jittery packet cannot fire it.
RECOVERY_CPA_MARGIN_M = 2.0
RECOVERY_CPA_OPEN_SAMPLES = 2  # consecutive opening packet-samples to confirm the pass (~0.5-0.8 s at 2.4-4 Hz)
# Yaw-rate hold gate (2026-07-24): RECOVERY_HOLD_S alone exits the recovery
# mode on a fixed timer, which released the vehicle while it was still
# tumbling from the terminal pass (log 110805: HOLD lasted 1.2 s while the
# copter kept ballooning 50 -> 95 m). While |yaw rate| exceeds this threshold
# the vehicle is still fighting itself, so STAY in the recovery mode and keep
# stabilising; only then REENGAGE. Needs ATTITUDE telemetry; if that is
# missing/stale the gate fails open (plain timer) rather than holding forever.
# 0 disables. RECOVERY_HOLD_MAX_S caps the extended hold so it can never stick.
RECOVERY_YAW_RATE_HOLD_DPS = 40.0
# 2026-08-09: 5.0 -> 8.0. MEASURED NEED, not an estimate. 14 of recovery event 14 in kpn26c + kpn26d
# was also cut in the CEILING and alt_err in return to CHASE did not go down to 3 m
# (RECOVERY_ALT_ABORT_ARM_M): measured values 14.2 / 24.8 / 5.7 / 13.0 / 6.1 / 5.1 / 34.0 / 18.5 /
# 15.7 / 14.8 / 13.3 / 23.1 / 17.0 ...m. The condition to hold until altitude
# recovers was never met in the altitude-fault branch of simple_guided_follow.
# The ceiling bound every time. Moving from 2 to 5 s already reduced alt_err p95
# from 56 to 23 m and the fraction with alt_err>15 from 86-91% to 15%, removing
# the overturning behavior. A duration of 8 s provides further margin. The ceiling
# remains finite, and the branch fails open when alt_error is None. Each recovery
# consumes 3 s more pursuit time. Monitor the effect on engagement count in
# kpn_straight3 and later trials. The following setting can override it for A/B tests:
# YILDIZ_HOLD_CEILING.
RECOVERY_HOLD_MAX_S = float(_os.environ.get("YILDIZ_HOLD_CEILING", 8.0))

# --- Impact detection (hit counter + crash screen on the GUI) ---
# Classify each terminal pass as HIT / CRASH from telemetry. Preferred signal is
# airframe VIBRATION (m/s^2): a spike is a physical impact -- a HIT if it lands
# within HIT_RANGE_M of the target, a CRASH otherwise (e.g. ground strike). If
# VIBRATION is absent, it falls back to kinematics: a sub-HIT_RANGE_M pass where
# the pursuer's speed suddenly collapses = hit; a tumbling/inverted attitude =
# crash. Tune to your SITL: if it never produces a vibe spike or speed drop on a
# clean <1 m pass, set HIT_REQUIRE_SPEED_DROP=False to count the pass itself.
HIT_RANGE_M = 1.0                 # a pass this close to the target is a hit candidate [m]
IMPACT_VIBE_THRESHOLD = 60.0      # VIBRATION level counted as a physical impact [m/s^2]
HIT_REQUIRE_SPEED_DROP = True     # (no-vibe path) require a speed collapse to confirm a hit
HIT_SPEED_DROP_MPS = 8.0          # pursuer speed drop within the window that confirms a hit [m/s]
HIT_SPEED_DROP_WINDOW_S = 0.5     # window over which the speed drop is measured [s]
CRASH_VIBE_THRESHOLD = 60.0       # VIBRATION away from the target counted as a crash [m/s^2]
CRASH_TILT_DEG = 120.0            # (no-vibe path) tilt beyond this = tumbling/inverted = crash [deg]

# --- Shutdown --- Flight mode commanded when the runner exits (Ctrl-C / SIGTERM / crash) so the
# vehicle stabilizes itself instead of coasting on the last GUIDED setpoint. "" externalables. Only
# applied when the script job managing modes (note --no-guided). USING POSHOLD: pilot mode, in SITL,
# since the throttle stick remains at the bottom (C3=1000), zero-thrust descent occurs at the speed of
# PILOT_SPEED_DN, yaw authority is lost and the vehicle crashes upside down with a spinner.
# (2026-08-08, 5 running overlap; dataflash 00000178-182). BRAKE doesn't read the stick at all: it
# stops and maintains altitude (same way as RECOVERY_MODE).
SHUTDOWN_MODE = "BRAKE"
REENGAGE_RAMP_TIME_S = 1.5  # ramp commanded speed from SAFE_TURN_SPEED to full over this [s]
REENGAGE_MAX_S = 4.0  # timeout forcing re-engagement back to full chase [s]
REENGAGE_MIN_CLOSING_MPS = 2.0  # closing speed that counts as "re-engaged" and returns to CHASE [m/s]
SAFE_TURN_SPEED = 7.0  # speed the re-engage ramp starts from after the HOLD window [m/s]
# First-order low-pass on the REENGAGE setpoint (pos+vel), seeded from the
# vehicle's own state each time it re-enters REENGAGE. Without it the setpoint
# steps straight to the target estimate the instant HOLD releases -- a full-
# authority re-commit of a possibly still-settling pursuer. The LPF eases the
# command from where the vehicle IS toward the target over ~tau, on top of the
# speed ramp. Toggle --reengage-lpf / --no-reengage-lpf. 0 tau also disables.
REENGAGE_LPF_ENABLED = True
REENGAGE_LPF_TAU_S = 0.8  # command smoothing time-constant during REENGAGE [s]

# --- Velocity feedforward vertical component ---
# When velocity setpoints are used (--no-position-only), the estimated target
# vertical velocity vz is stripped from the outgoing setpoint by default and the
# smooth position setpoint carries Z instead. Reason: CT/CA turns inject Z
# transients into the estimated vz; feeding that vz to the copter spikes collective
# throttle, saturates motors, and ArduCopter sheds yaw authority first -> physical
# yaw instability (seen only in pos+vel, never pos-only). The target flies ~level,
# so dropping vz loses no real motion. Set True to send the raw vz (A/B testing).
VELOCITY_FF_VERTICAL_ENABLED = False

# --- Acceleration feedforward (GUIDED position+velocity+acceleration setpoints) ---
# Adds the estimated target acceleration (CT centripetal + CA linear, horizontal
# only) to the SET_POSITION_TARGET stream so the pursuer leads a curving target
# one derivative sooner. Only takes effect with velocity setpoints (NOT
# position-only), faded to zero inside the terminal-extension band, and never
# vertical. Off by default: velocity+accel runs destabilized terminal passes.
ACCEL_FEEDFORWARD_ENABLED = False  # opt-in only (--accel-feedforward)
# Capped to the same bank envelope as the command turn clamp below
# (COMMAND_LATERAL_ACCEL_MAX_MPS2, ~35 deg). The accel FF is the derivative of
# the velocity FF; if it fed forward a sharper turn than the clamped velocity
# rotation the two would be inconsistent and the controller would overshoot.
ACCEL_FEEDFORWARD_MAX_MPS2 = 6.9  # clamp on FF accel magnitude [m/s^2] = g*tan(35 deg)
ACCEL_FEEDFORWARD_FADE_BAND_M = 20.0  # range band above the extension range over which FF fades to zero [m]

# --- Yaw control ---
# DEFAULT: yaw is OFF. We do not command yaw at all -- ArduCopter holds its own
# heading (WP_YAW_BEHAVIOR=0). Every yaw law we tried (point-at-target LOS,
# point-along-velocity) made things worse: at the bank needed to follow a
# co-speed turn the mixer has no yaw headroom, so commanding ANY yaw is what
# diverged (spin/tumble/crash, logs 165119 & 172525). The fix is upstream --
# the command turn clamp below caps the commanded bank so headroom always
# remains -- which makes yaw-locking unnecessary. These settings are kept only
# for the opt-in --yaw-lock escape hatch and are inert unless it is passed.
YAW_LOCK_ENABLED = False  # off by default; --yaw-lock re-enables the (deprecated) yaw law
YAW_LOCK_MIN_RANGE_M = 10.0  # hold the previous yaw inside this horizontal LOS range [m]
YAW_LOCK_MAX_RATE_DEG_S = 90.0  # slew limit for yaw lock [deg/s]
YAW_LOCK_MODE = "los"  # "velocity" | "los" (only used if --yaw-lock)
YAW_LOCK_MIN_SPEED_MPS = 3.0  # below this horizontal speed the course is ill-defined -> hold yaw
YAW_FREEZE_TILT_DEG = 38.0  # freeze yaw slew above this tilt [deg]; 0 disables (only if --yaw-lock)

# --- Command turn clamp (authority-aware command conditioning) ---
# Root-cause fix for the terminal crash family (spin / balloon / altitude
# collapse). The bank a copter must hold to follow a moving pos+vel setpoint is
# set by the LATERAL ACCELERATION of that setpoint: a_lat = speed * d(heading)
# /dt. A co-speed pursuer following a hard target turn banks to ANGLE_MAX, and
# at the tilt limit the mixer sheds yaw authority first -> any disturbance
# diverges. So we never command more turn than a fixed bank margin can execute:
# rate-limit the heading of the horizontal velocity command (and rotate the
# position lead to match) to omega_max = a_lat_max / speed. This is the NDI-
# style "don't demand what the vehicle can't do" fix; it makes yaw-locking moot.
# TRADEOFF: turns sharper than ~a_lat_max/speed are followed on a gentler arc
# (may miss a hard-turning target) but never crash. cmd_lat_accel_mps2 in the
# CSV logs the raw demanded lateral accel -> how sharp a turn the target flew.
COMMAND_TURN_CLAMP_ENABLED = True
COMMAND_LATERAL_ACCEL_MAX_MPS2 = 6.9  # g*tan(35 deg); 10 deg margin under a 45 deg ANGLE_MAX
COMMAND_TURN_CLAMP_MIN_SPEED_MPS = 2.0  # below this the heading is ill-defined -> hold, no clamp
COMMAND_TURN_CLAMP_ROTATE_POSITION = True  # also de-curve the position lead so PosP doesn't refight the turn
# 2026-08-02: rotation is now applied to the LEAD OFFSET around the target, not to the LINE OF SIGHT
# (see clamp_commanded_turn assortment). In addition, the HARD ceiling of the angle that can be applied
# in a single call: Since rot_corr carried the accumulated title error, it could go up to +-180
# degrees, and it ripped the command from the target and made it draw a giant circle. 10 degree allows
# correction of 200 deg/s in 20 Hz.
COMMAND_TURN_CLAMP_MAX_ROTATION_DEG = 10.0  # largest correction angle that can be applied in a single cycle [deg]

# --- super_safe_turn: pre-emptive speed cap ahead of a coordinated turn ---
# The turn clamp above conditions the DIRECTION of the command so the pursuer
# never banks past its envelope. super_safe_turn is the complementary SPEED
# measure: when the IMM is confident the target is turning (mu_ct >= threshold)
# it caps the commanded speed to v_safe = a_lat_max / omega -- the fastest speed
# at which the turn of rate omega still needs only a_lat_max of lateral accel.
# Slowing to v_safe lets the turn clamp pass the FULL target turn (omega_max =
# a_lat_max/v_safe == omega) instead of shaving it to a gentler arc, so you keep
# the target centred through the turn AND never demand an impossible bank. It
# trades closure speed for turn authority -- an extra margin against losing a
# real airframe. Default OFF (opt-in): closure speed is unrestricted otherwise.
# Toggle at runtime with --super-safe-turn / --no-super-safe-turn. pos+vel only.
SUPER_SAFE_TURN_ENABLED = True  # master default for the --super-safe-turn CLI toggle
SUPER_SAFE_TURN_LATERAL_ACCEL_MPS2 = 6.9  # bank envelope used for v_safe; match the turn clamp (g*tan35)
SUPER_SAFE_TURN_MU_CT_THRESHOLD = 0.5  # only slow when CT-mode probability >= this (a turn is really happening)
SUPER_SAFE_TURN_OMEGA_MIN_RAD_S = 0.1  # ignore near-zero turn rates (v_safe would explode); ~0.1 rad/s = gentle turn
SUPER_SAFE_TURN_MIN_SPEED_MPS = 5.0  # never slow below this; keeps closing and avoids a crawl on a very sharp turn

# --- Mission failsafe (Redis runner only, 2026-07-31) ---
# Mission-level safety supervisor for simple_guided_follow_shaykh.py: when the
# data the guidance depends on goes STALE or CORRUPT, stop steering the vehicle
# and hand it back to its pre-planned mission instead of flying on garbage.
# Follows the project's established pattern (GOAT_guidance.crash_manuever ->
# set_mode('AUTO'); its documented "GUIDED and >=2 s without a message -> AUTO
# due to inactivity" watchdog) and its Redis conventions ('uav_mode' published
# by the bridge, 'guid' as the guidance-active flag).
#
# TWO mechanisms, deliberately redundant:
#   1. STOP PUBLISHING the slot key. Transport-independent -- any bridge with
#      its own setpoint-staleness watchdog falls back on its own (the real
#      2026-07-30 flights show ~50 GUIDED<->AUTO toggles, consistent with the
#      bridge doing exactly this). This one always works.
#   2. PUBLISH an explicit mode request + health payload to
#      MISSION_FAILSAFE_REDIS_KEY, and the conventional 'guid' flag. The exact
#      key/schema the absent 4drone4_combined.py consumes is UNKNOWN, so both
#      are configurable -- set them to whatever the bridge actually reads.
# Recovery is hysteretic (trip after TRIP_AFTER_S continuously unhealthy, clear
# after CLEAR_AFTER_S continuously healthy) so a flapping feed cannot chatter
# the flight mode.
MISSION_FAILSAFE_ENABLED = True
# 2026-08-02: IT WAS "AUTO". AUTO only makes sense in a task-laden vehicle; In this environment, the
# plan is loaded ONLY to the target aircraft (yildizlar_guidance.sh:368), no mission is loaded to the
# fighter copter -> AUTO is either rejected or behaves unexpectedly with the empty mission. The
# self-stable LOITER provides accurate handoff delivery.
MISSION_FAILSAFE_MODE = "LOITER"                # mode commanded/requested when guidance is unsafe
MISSION_FAILSAFE_REDIS_KEY = "guidance_failsafe"  # JSON {active, mode, reason, ts}; "" disables this channel
MISSION_FAILSAFE_GUID_KEY = "guid"              # project convention: "True"/"False" guidance-active flag; "" disables
MISSION_FAILSAFE_TRIP_AFTER_S = 1.0             # HARD faults unhealthy this long -> trip
MISSION_FAILSAFE_CLEAR_AFTER_S = 2.0            # healthy this long after a HARD fault -> resume (bias to safe)
MISSION_FAILSAFE_REDIS_ERRORS = 5               # consecutive setpoint-write failures that count as unhealthy
# Two fault CLASSES with very different timings, because they mean different
# things:
#   DATA GAP  -- no fresh target measurement (a camera tracker dropping a few
#                frames). This is normal and self-heals, so it must NOT stop
#                the chase: withholding setpoints every few seconds would make
#                the vehicle repeatedly slow down, and in the MAVLink runner it
#                would thrash the flight mode. Instead we COAST (see below) and
#                only hand off to the mission if the feed really is gone.
#   HARD FAULT -- estimator diverged, setpoint sending failing, crash detected.
#                These do not self-heal, so they use the slower TRIP/CLEAR
#                timings above and bias hard toward staying safe.
#
# The data-gap response is a THREE-tier ladder on the age of the last accepted
# measurement:
#   age <= GRACE      -- nothing happens at all. Normal between-packet flight.
#   GRACE < age <= HANDOFF -- COASTING: keep commanding, and dead-reckon the
#                     guidance state forward at the last estimated velocity so
#                     the slot keeps moving WITH the target instead of freezing
#                     in space (a frozen slot is what makes the vehicle slow
#                     down). No mode change, no withheld setpoints.
#   age > HANDOFF     -- the feed is genuinely gone: withhold setpoints and go
#                     to MISSION_FAILSAFE_MODE.
# GRACE is therefore only "when do we start extrapolating", NOT "when do we
# stop flying" -- so it is safe to set it near the normal packet interval.
# Measured feeds for reference: Shaykh's real 2026-07-30 run had a median inter-measurement gap of
# 0.151 s (p95 0.353 s); the SITL target arrives at 2.5-3.2 Hz (~0.3-0.4 s). Coasting business cheap;
# The handoff is the real decision. 2026-08-02 CORRECTION: these two values ​​were REVERSE (GRACE 1.7
# > HANDOFF 1.0). The above ladder assumes GRACE < HANDOFF; The middle digit (coast / dead reckoning)
# was becoming INaccessible -- coast_age_s 0.0 exactly 100% on each record -- and failsafe was hitting
# first instead. Now GRACE is just above the normal pack range, while HANDOFF is for a truly cut feed.
MISSION_FAILSAFE_TARGET_GRACE_S = 0.4     # gap beyond this -> coast on a dead-reckoned slot [s]
MISSION_FAILSAFE_TARGET_HANDOFF_S = 1.5   # gap beyond this -> withhold setpoints + failsafe mode [s]
MISSION_FAILSAFE_COAST_DEADRECKON = True  # while coasting, advance the slot at the last estimated velocity
MISSION_FAILSAFE_DATA_CLEAR_S = 0.3       # fresh data flowing this long after a handoff -> resume commanding [s]
# Target-payload sanity gate. Rejects corrupt telemetry BEFORE it reaches the
# estimator. The implied-speed check is not theoretical: Shaykh's real
# 2026-07-30 feed carried position glitches implying up to 1350 m/s.
TARGET_MAX_IMPLIED_SPEED_MPS = 120.0  # reject a measurement implying more than this since the last accepted one; 0 disables
TARGET_MAX_ALTITUDE_M = 2000.0        # reject |relative altitude| above this [m]; 0 disables

# --- Carrot-distance clamp (closure governor, 2026-07-31) ---
# In pos+vel mode the vehicle's speed is our velocity FF (capped 25, logged
# ~19-21 m/s) PLUS ArduCopter's INTERNAL position-error correction -- the one
# command authority this script cannot otherwise bound. After a re-approach gap
# the 30-60 m position error made that internal term sprint the vehicle to
# 34-41 m/s (the terminal over-speed crash family; 16.7 m/s latch survived,
# 34 m/s died, log 145054; the 07-29 fly-through crashes arrived at 35-40).
# Fix: cap how far AHEAD of the vehicle the commanded position may sit --
# cmd_xy = pursuer_xy + clamp(slot_xy - pursuer_xy, CARROT_MAX_AHEAD_M). The
# internal P-term then contributes at most ~ PSC_POSXY_P * D_max (~+ 10 m/s at the default 10 m) over
# the FF: a CONSTANT, controlled overtake everywhere. This is a closure GOVERNOR, not a brake -- the
# carrot recedes as the vehicle advances, so it never decelerates, and once the true offset is inside
# D_max it is inactive (terminal pass-through unaffected; closure stays at the design overtake rate
# through impact). Horizontal only: commanded Z is governed by the z-slew, and scaling it would dilute
# the terminal climb. Direction preserved (still flies AT the spear). pos+vel only -- pos-only GUIDED
# plans an arrive-and-stop to the point, which a 10 m carrot would cripple;  WPNAV_SPEED bounds that
# mode already. Trade-off: long-range catch- up closes at only ~P* D_max over the target's speed --
# raise D_max if the merge takes too long. Toggle --carrot-clamp / --no-carrot-clamp .
CARROT_CLAMP_ENABLED = True
CARROT_MAX_AHEAD_M = 10.0  # max commanded-point lead over the vehicle [m]; ~+10 m/s of P-term at PSC_POSXY_P=1
# The clamp must bound the SPRINT regime only, NOT the long-range merge. Inside
# CARROT_ACTIVE_RANGE_M the allowance is CARROT_MAX_AHEAD_M; beyond it the
# allowance grows 1:1 with the offset, so a distant merge keeps essentially its
# full position error and closes fast. Clamping at every range starved the
# merge: with the position error pinned at 10 m, the target-VELOCITY feedforward
# dominates the command -- and that vector points along the target's TRACK, not
# at the target. Sim 20260731_102802 (target flying a 544x211 m circuit 478 m
# away) commanded a velocity 138 deg off the line of sight, so the pursuer flew
# sideways at 16 m/s and closed at only 5 m/s. Growing the allowance is
# continuous, so there is no step in commanded speed at the boundary. 0 =
# clamp at CARROT_MAX_AHEAD_M everywhere (the old, merge-starving behaviour).
CARROT_ACTIVE_RANGE_M = 80.0  # offsets beyond this are effectively unclamped [m]; covers the 30-60 m sprint regime
# --- Command position budget (FIXED metric ceiling, 2026-08-02) --- Since the carrot budget above
# grew 1:1 over long distance, it was PRACTICALLY limiting nothing: in the 20260802 run |slot -
# hunter| While the median was 821 m, the trace increased to 751 m, carrot_limited=1 was lit in 99.4%
# of the samples, but the command point was still hundreds of meters away. ArduCopter
# SET_POSITION_TARGET_ When position + speed are given together in LOCAL_NED, it adds the position
# term ABOVE the speed feedforward: in the dataflash the desired speed appeared as 39.4 m/s = 2 x
# WPNAV_SPEED. So the command point can be at most (command rate * BUDGET_S + BUDGET_MARGIN_M) away
# from the hunter, INDEPENDENT of the carrot logic; ~20 m/s at 60 m. It is applied on horizontal axis
# and in pos+vel mode.
COMMAND_POSITION_BUDGET_S = 2.0        # The command point can only be this many seconds ahead [s]
COMMAND_POSITION_BUDGET_MARGIN_M = 20.0  # Fixed share added to budget [m]

# --- Output governors (final clamps on the outgoing position setpoint) ---
# Safety nets, not shapers: legitimate aim motion is bounded by target speed
# plus terminal geometry changes (~35 m/s); prediction transients moved the
# commanded point 100+ m between loops (and underground 5x) on 2026-07-11.
AIM_POINT_MAX_SPEED_MPS = 60.0  # hard 3-D rate cap on the outgoing position setpoint [m/s]; 0 disables
MIN_COMMAND_ALTITUDE_M = 15.0  # never command below this altitude (home-relative NED z, assumes ~flat field) [m]

# --- Command-side Z slew limiting (simple_follow) ---
# Always-on slew on the outgoing Z command [m/s]; 0 disables. This is the
# primary yaw-spike guard in pos+vel mode (2026-07-15 logs): it turns the
# 30 m catch-up z offset, the 4-5 Hz packet staircase during real target
# climbs (14 m/s per-loop bursts), and mode-mix shoves into a <=4 m/s ramp
# the climb controller can track without saturating collective (saturation
# sheds yaw authority first). Seeded from the pursuer's own altitude on the
# first command so the initial climb is also ramped.
Z_ALWAYS_SLEW_RATE = 4.0
Z_SWITCH_SLEW_RATE = 0  # Z slew during CT/fast-turn windows [m/s]; 0 disables
Z_SWITCH_JUMP_M = 0.5  # Z command jump needed to apply switch-window slew [m]
Z_SWITCH_WINDOW_S = 1.2  # switch/fast-turn Z slew window duration [s]
Z_SWITCH_DMU = 0.08  # CT probability jump that opens switch-window Z slew
Z_SWITCH_MU_THRESHOLD = 0.20  # CT probability crossing that opens switch-window Z slew
Z_OUTLIER_SLEW_RATE = 0  # always-on Z outlier slew [m/s]; 0 disables
Z_OUTLIER_JUMP_M = 0.9  # Z command jump needed to apply always-on outlier slew [m]

# --- Estimator Z freeze on CT activation ---
# Freezes the estimator's vertical correction for N packets starting at the exact
# packet where aggregate mu_ct_xy crosses Z_CT_FREEZE_MU_THRESHOLD upward. XY
# still updates; Z/VZ/AZ are restored to predicted. 0 disables.
Z_CT_FREEZE_PACKETS = 0
Z_CT_FREEZE_MU_THRESHOLD = 0.20  # CT probability crossing that arms the freeze


# ============================================================
#  UNUSED by the active pipeline (simple_guided_follow.py + filterwndr.py).
#  Commented out to keep the live config focused. Values preserved for
#  reference. NOTE: entries tagged [retired] are still referenced by the
#  retired runners (lag_pursuit_pid.py / pronav_runner.py / velocity_control.py)
#  and must be uncommented before running those.
# ============================================================

# --- PN / FRPN guidance law (retired runners) ---
# COMMAND_DT = 0.5                  # [retired] effective dt for PN velocity update [s]
# NAV_GAIN = 3                     # FRPN navigation gain, tunes aggression
# WEIGHTING_GAIN = 0.5             # FRPN weighting: lower aims further ahead of target
# Z_AXIS_PN = False                # [retired] Z-axis accel via PD instead of FRPN
# PD_KP = 2.7                      # [retired] terminal PD proportional gain
# PD_KD = 0.9                      # [retired] terminal PD derivative gain
# FRPN_PREDICTION_STEPS = 1        # IMM lead ticks for FRPN (1 = current state)
# MAX_THRUST_G = 1.2               # max thrust [g] (1.0 = hover)
# ATTITUDE_TAU = 0.2               # quad attitude response time constant [s]

# --- Lag pursuit / PID (retired runners) ---
# LAG_PURSUIT_MAX_SPEED = 21.0     # [retired] max pursuer velocity command [m/s]
# Z_STEADY_ERROR = 3.0             # [retired] steady-state Z error to keep target in sight [m]
# LAG_PID_XY_KP = 0.8              # [retired]
# LAG_PID_XY_KI = 0.05             # [retired]
# LAG_PID_XY_KD = 0.3              # [retired]
# LAG_PID_Z_KP = 1.2               # [retired]
# LAG_PID_Z_KI = 0.1               # [retired]
# LAG_PID_Z_KD = 0.4               # [retired]

# --- Maneuver / command limits (retired runners) ---
# MAX_TURN_DEG = 80.0              # [retired] max turn-rate [deg/s]
# MAX_TILT_DEG = 30.0              # [retired] max commanded tilt [deg]
# MAX_OMEGA = 2.2                  # [retired] max LOS rate [rad/s]
# MAX_THRUST = 12.0                # [retired] max total thrust [m/s^2]
# CMD_SMOOTHING_ALPHA = 0.6        # [retired] EMA smoothing on velocity command
# TRANSITION_T_MIN = 0.3           # min transition time for smoothing [s]
# TRANSITION_T_MAX = 1.5           # max transition time for smoothing [s]
# TRANSITION_K_MARGIN = 1.2        # transition safety coefficient
# JERK_RESET_THRESH = 1.8          # accel command diff to trigger new smoothing transition

# --- Safety / engagement ranges (retired runners) ---
# MIN_ALT_M = 10.0                 # [retired] minimum altitude floor [m]
# KP_ALT = 3.0                     # [retired] altitude protection gain [1/s^2]
# DECEL_RANGE = 150                # [retired] range where aggressiveness ramps [m]
# PN_ENGAGE_RANGE = 100.0          # [retired] pure pursuit above / PN below [m]
# APN_ENGAGE_VC_MIN = 5.0          # [retired] APN engages above this closing velocity [m/s]
# APN_ENGAGE_SPEED_MIN = 20.0      # [retired] APN engages above this pursuer speed [m/s]
# APN_TRANSITION_TIME_S = 0.6      # [retired] APN accel sigmoid transition [s]

# --- Miss detection / legacy re-engagement (retired runners) ---
# MISS_DETECT_RANGE = 200.0        # [retired] range to begin terminal/miss evaluation [m]
# ZEM_LIMIT = 1.0                  # Zero-Effort-Miss threshold [m]: less = freeze&spear
# MISS_ANGULAR_RATE_THRESHOLD = 2.0  # [retired] LOS rate declaring a miss [rad/s]
# REENGAGE_FACING_DEG = 30.0       # [retired] re-enter kinematic mode within this of LOS [deg]
# REENGAGEMENT_ACCEL_MULT = 2.2    # rate-limit multiplier during re-engagement

# --- Velocity sender (retired runners) ---
# VEHICLE_CONN_STR = "127.0.0.1:14551"  # [retired] Dronekit connection for velocity sender
# SEND_RATE_HZ = 10                # [retired] velocity command send rate [Hz]

# --- Aspirational / never-wired (kept for reference) ---
# EARTH_RADIUS = 6378137.0         # WGS-84 equatorial radius [m]
# HIT_REQ_RANGE = 1e-7             # minimum range to count as a hit [m]
# NAV_CONSTANT = 3.0               # PN gain N
# GUIDANCE_MODE = "APN"            # "APN" | "TPN" | "PPN" | "FRPN"
# PN_GAIN_DECAY_ENABLED = True     # logarithmic N decay with time-to-go
# RC_SWITCH_CHANNEL = 7            # RC channel for PPN / pursuit toggle
