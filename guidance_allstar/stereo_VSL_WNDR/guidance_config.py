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

# --- MAVLink connections ---
PURSUER_CONN_STR = "udpin:localhost:14552"
TARGET_CONN_STR = "udpin:localhost:14600"
TARGET_EXPECTED_SYSID = 2  # ArduPlane sysid on the target out link

# --- Runtime param assertion for GUIDED intercept sim ---
GUIDED_STARTUP_PARAM_ASSERTS = {
    "WP_YAW_BEHAVIOR": 0,
    "WPNAV_SPEED": 2200,
    "WPNAV_ACCEL": 600,
    "WPNAV_JERK": 5,
    "WPNAV_SPEED_UP": 500,
    "WPNAV_SPEED_DN": 400,
    "WPNAV_ACCEL_Z": 300,
    "ANGLE_MAX": 4500,
}

# --- Target telemetry ---
TARGET_MESSAGE_RATE_HZ = 15.0  # requested GLOBAL_POSITION_INT rate for the target [Hz]
# Own-position staleness guard (2026-07-24 review): the target feed always had
# a stale-hold; the pursuer's own LOCAL_POSITION_NED did not, so a frozen own
# position silently fed phantom range/t_go/CPA AND defeated the altitude abort
# (alt_error uses the frozen z). Older than this -> hold setpoints + warn.
PURSUER_STALE_TIMEOUT_S = 1.0

# --- Slot / position-only default ---
POSITION_ONLY_DEFAULT = True  # keep simple_follow on position targets by default
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
LEAD_AIM_LPF_TAU_S = 0.25
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
TERMINAL_POSITION_EXTEND_RANGE_M = 80.0  # range at/below which the extension becomes active [m]
TERMINAL_POSITION_EXTEND_DISTANCE_M = 15.0
TERMINAL_POSITION_EXTEND_BRAKE_MARGIN_M = 10.0  # extra margin on top of v^2/(2a) when sizing extension
TERMINAL_POSITION_EXTEND_MAX_M = 30.0
# Blend the extension in over this band below the activation range instead of
# stepping the aim ~30 m in one loop (the step spikes tilt+collective at speed;
# motor saturation then sheds yaw authority first -- 2026-07-15 pos+vel logs).
# 0 restores the legacy instant step.
TERMINAL_EXTEND_BLEND_BAND_M = 10.0

# --- Terminal freeze-&-spear latch ---
TERMINAL_LATCH_TGO_S = 0.7  # freeze the aim point when estimated time-to-go falls below this [s]
TERMINAL_LATCH_RELEASE_RANGE_M = 40.0  # release the frozen aim point after a pass once range opens back up [m]
# Once the aim is FROZEN, the velocity feedforward must go with it: commanding a
# fixed point while still feeding ~20 m/s of target velocity is self-
# contradictory, and once the pursuer overshoots that point the position error
# reverses while the FF still pushes forward -- the controller fights itself,
# pitches back hard at high thrust and balloons (log 110805). Zeroing the FF
# turns the latched phase into a clean "fly through to this point and stop",
# which is what the extension already intends. False restores the old behaviour.
TERMINAL_LATCH_ZERO_VELOCITY_FF = True

# --- Miss recovery (CHASE -> HOLD -> REENGAGE -> CHASE state machine) ---
MISS_RECOVERY_ENABLED = True  # on by default; toggle with --miss-recovery / --no-miss-recovery
RECOVERY_MODE = "BRAKE"  # flight mode during the HOLD window (BRAKE recommended; STABILIZE / LOITER also work)
RECOVERY_HOLD_S = 1.2  # seconds to hold in the recovery mode to bleed speed / level out
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
RECOVERY_ALT_ABORT_M = 8.0
RECOVERY_ALT_ABORT_ARM_M = 3.0  # abort arms once |pursuer_z - commanded_z| first drops below this [m]
# CPA (closest-point-of-approach) trigger, 2026-07-24. The old miss signal was
# the latch RELEASING at range >= TERMINAL_LATCH_RELEASE_RANGE_M (40 m) -- which
# is 2-6 s AFTER the pass, during which the pursuer is fighting a frozen aim
# point it has already flown past (position error reversed while the velocity FF
# still commanded ~20 m/s forward) -> pitch-back -> balloon (50->95 m, log
# 110805). Instead, declare the pass once range has climbed this far above the
# minimum seen while latched, and hand off to recovery. 0 disables, falling
# back to the latch-release miss.
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
RECOVERY_HOLD_MAX_S = 5.0

# --- Shutdown ---
# Flight mode commanded when the runner exits (Ctrl-C / SIGTERM / crash) so the
# vehicle stabilises itself instead of coasting on the last GUIDED setpoint.
# "" disables. Only applied when the script is managing modes (not --no-guided).
SHUTDOWN_MODE = "POSHOLD"
REENGAGE_RAMP_TIME_S = 1.5  # ramp commanded speed from SAFE_TURN_SPEED to full over this [s]
REENGAGE_MAX_S = 4.0  # timeout forcing re-engagement back to full chase [s]
REENGAGE_MIN_CLOSING_MPS = 2.0  # closing speed that counts as "re-engaged" and returns to CHASE [m/s]
SAFE_TURN_SPEED = 7.0  # speed the re-engage ramp starts from after the HOLD window [m/s]

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
ACCEL_FEEDFORWARD_MAX_MPS2 = 8.0  # clamp on FF accel magnitude [m/s^2]; below copter lateral cap (~9.8 at 45 deg)
ACCEL_FEEDFORWARD_FADE_BAND_M = 20.0  # range band above the extension range over which FF fades to zero [m]

# --- Yaw control ---
YAW_LOCK_ENABLED = False  # lock commanded yaw to LOS from pursuer to predicted target
YAW_LOCK_MIN_RANGE_M = 10.0  # hold the previous yaw inside this horizontal LOS range [m]
YAW_LOCK_MAX_RATE_DEG_S = 90.0  # slew limit for yaw lock [deg/s]

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
