"""
stereo_config.py -- tunables for the bearings-only (stereo camera) front end.

The sensor is no longer a position source. Two cameras each report a direction
to the target as (yaw, pitch) with per-axis angular noise. Everything here
describes that sensor and how the estimator should consume it.

Geometry primer (drives most of the numbers below):
    cross-LOS position error ~ R * sigma_angle          (good)
    depth (along-LOS) error  ~ R^2 / b * sigma_angle    (brutal)
At R=200 m, b=1 m, sigma=1 mrad: 0.2 m across, ~40 m deep. The position
uncertainty is a long thin cigar pointed down the line of sight, so nothing
here may assume an isotropic position measurement.
"""

import numpy as np

# ============================================================
#  Camera rig
# ============================================================
# Poses are (north, east, down) in metres and (yaw, pitch, roll) in DEGREES,
# body->NED 3-2-1: yaw about down, pitch nose-up positive, roll right-down
# positive. Camera axes: x = boresight, y = right, z = down.
#
# Default: a ground stereo pair on an east-west baseline, both looking north
# and tilted up 10 deg. BASELINE_M is the separation; depth precision scales
# with it linearly, so this is the single most valuable physical knob.
BASELINE_M = 6.0
CAMERA_ALTITUDE_M = 2.0  # height above the local origin [m] (NED down = -alt)

CAMERAS = [
    {
        "name": "left",
        "position_ned": (0.0, -BASELINE_M / 2.0, -CAMERA_ALTITUDE_M),
        "yaw_deg": 0.0,
        "pitch_deg": 10.0,
        "roll_deg": 0.0,
        "sigma_yaw_deg": 0.06,    # ~1.0 mrad, a decent detector on a narrow lens
        "sigma_pitch_deg": 0.06,
        "fov_yaw_deg": 60.0,      # full horizontal FOV
        "fov_pitch_deg": 40.0,
        "max_range_m": 1200.0,    # detection range limit
        # Surveyed boresight, SUBTRACTED from this camera's measurements at
        # ingest. See the note under "Boresight bias estimation" before
        # entering anything here.
        "bias_yaw_deg": 0.0,
        "bias_pitch_deg": 0.0,
    },
    {
        "name": "right",
        "position_ned": (0.0, BASELINE_M / 2.0, -CAMERA_ALTITUDE_M),
        "yaw_deg": 0.0,
        "pitch_deg": 10.0,
        "roll_deg": 0.0,
        "sigma_yaw_deg": 0.06,
        "sigma_pitch_deg": 0.06,
        "fov_yaw_deg": 60.0,
        "fov_pitch_deg": 40.0,
        "max_range_m": 1200.0,
    },
]

# ============================================================
#  Measurement consumption
# ============================================================
# The estimator updates on ANGLES, not on a triangulated position. Four scalars
# per frame (yaw/pitch per camera), applied sequentially so each carries its own
# gate and so a single camera's detection is still a usable update.
SEQUENTIAL_SCALAR_UPDATES = True

# Per-scalar innovation gating (chi-square, 1 dof).
#   "reject"  drop the scalar when NIS > gate
#   "inflate" keep it but scale R up so it can still nudge the state (softer,
#             avoids the maneuver-onset latency a hard gate introduces)
#   "off"     no gating
NIS_GATE_MODE = "inflate"
NIS_GATE_CHI2 = 9.0          # 3-sigma on 1 dof
NIS_INFLATE_MAX = 100.0      # cap on the R multiplier in "inflate" mode
NIS_HARD_REJECT_CHI2 = 400.0  # even in inflate mode, 20-sigma is a bad detection

# Frame-level consistency: the perpendicular miss distance between the two rays.
# Skew is NOT an error to be hidden -- it is a free per-frame quality signal.
# Above SKEW_INFLATE_M the frame's angular noise is inflated in proportion;
# above SKEW_REJECT_M the frame is dropped outright.
SKEW_INFLATE_M = 3.0
SKEW_REJECT_M = 60.0

# ============================================================
#  Track initialisation (from triangulation)
# ============================================================
# Triangulation still runs every frame, but only for: seeding the track, the
# turn-rate hint, and diagnostics -- never as the primary measurement.
INIT_MIN_FRAMES = 4           # consecutive good dual-camera frames before init
INIT_MAX_SKEW_M = 15.0        # frames dirtier than this do not count toward init
INIT_POS_SIGMA_INFLATE = 2.0  # inflate the GDOP covariance when seeding P
INIT_VEL_SIGMA_MPS = 12.0     # 1-sigma on the seeded velocity
INIT_ACC_SIGMA_MPS2 = 5.0     # 1-sigma on the seeded acceleration
TRIANGULATION_GN_ITERS = 6    # Gauss-Newton iterations for the ML fix
TRIANGULATION_MIN_PARALLAX_DEG = 0.05  # below this the pair is degenerate

# ============================================================
#  Track maintenance
# ============================================================
COAST_TIMEOUT_S = 2.0        # no usable detection this long -> track lost
# Clamp on a single propagate step. This MUST NOT be smaller than
# COAST_TIMEOUT_S: a gap between the two would be propagated for less time than
# actually elapsed, and the filter would then have to explain the measurement's
# extra displacement by inflating velocity. Measured on the bench at 0.5 Hz with
# the old 1.0 s value: position looked fine (2.5 m) while the 1 s prediction was
# 18.5 m out, because |v| was scaled by (true gap / clamp). Silent, and invisible
# in the position residual. Gaps longer than COAST_TIMEOUT_S are refused
# outright (-> LOST) rather than clamped.
MAX_PREDICT_DT_S = 2.0
MIN_PREDICT_DT_S = 1e-3
PREDICT_SUBSTEP_S = 0.1      # matches filterwndr's PREDICT_MAX_SUBSTEP

# ============================================================
#  Input validation
# ============================================================
# Drop detections whose angles or stamp are not finite. Without this a single
# NaN is accepted by the chi-square gate (every comparison against NaN is
# False), then poisons every mode's likelihood, and the WHOLE frame is
# discarded -- including the healthy camera's scalars. A camera emitting
# garbage then kills the track in COAST_TIMEOUT_S, while a camera that simply
# stops does not. Dropping the bad detection degrades to mono instead.
REJECT_NONFINITE_DETECTIONS = True

# Refuse measurements whose stamp does not advance. An out-of-order packet is
# otherwise applied as if current (dt clamps to MIN_PREDICT_DT_S) AND rewinds
# the filter clock, so the next live frame sees a huge dt; a duplicate packet is
# counted as independent evidence and shrinks the covariance for free.
REJECT_NONMONOTONIC_STAMPS = True

# A sender whose clock freezes while still publishing keeps the tracker in
# TRACK forever, because COAST_TIMEOUT_S is measured in MEASUREMENT time and
# that clock has stopped. Counting consecutive non-advancing frames catches it
# deterministically, with no dependence on wall time (which would make replay
# non-reproducible). 0 disables.
STALE_FRAME_LIMIT = 20

# Feed the per-frame triangulated fix to filterwndr's HeadingTurnRateEstimator
# so the CT mode still gets its turn-rate hint. Uses the independent per-frame
# fix (not the filter's own output), so this stays non-circular.
# DEFAULT OFF as of 2026-07-28, on evidence from the real VSL rig.
#
# Turn rate is a DIFFERENTIATED quantity, so feeding it per-frame fixes is only
# safe when the target's motion between fixes dominates the fix noise. On the
# real rig (21 July flight) frames arrive at ~20 Hz with ~3 m fix noise, so the
# difference is almost pure noise. It drove the CT modes hard enough to fling
# the IMM: median error 17.4 m, p90 5.8 km, velocity state reaching 1373 m/s on
# a 20 m/s target. Turning the hint off: median 7.9 m, p90 183 m.
#
# Decimating (TURN_HINT_MIN_DT_S) is NOT sufficient -- at 4 Hz the differenced
# noise is still ~12 m/s, tens of deg/s of phantom turn rate.
#
# It does not pay for itself even where it is safe: on the synthetic bench it is
# marginally WORSE than off at both 6 m and 100 m baselines (5.86 vs 5.47 m and
# 0.43 vs 0.41 m). The CT modes get their turn rate from the IMM's own mixing
# perfectly well; the hint was insurance that turned out to cost more than it
# paid. Enable it only for a fix stream whose noise is small compared with the
# target's displacement between hints.
USE_TRIANGULATION_TURN_HINT = False
TURN_HINT_MIN_DT_S = 0.25       # decimate the hint to at most ~4 Hz
TURN_HINT_MAX_SKEW_M = 8.0      # only hint from frames whose rays actually meet

# ============================================================
#  Boresight bias estimation (optional)
# ============================================================
# ENTERING A MEASURED BORESIGHT (`bias_yaw_deg` / `bias_pitch_deg` above).
# Worth doing -- on the 30 July real data a fitted constant cut the
# geometry-dependent error from 15.1 m to 6.7 m -- but with one trap.
#
# A constant ANGULAR offset and an error in the camera POSITION are not
# distinguishable from a single deployment's residuals, and the fitter will
# happily pour one into the other. Fitting with the (wrong) entered CAM2
# position gave CAM1 yaw -2.08 deg; fitting with the position free gave
# -0.45 deg. So ~1.6 deg of that "boresight" was really an 8 m survey error
# wearing an angular costume. It cancels at the range it was fitted at and
# drifts elsewhere: error by range band was 4.4 / 5.8 / 11.2 m for the
# bias-only fit versus 4.9 / 4.9 / 7.3 m once the position was corrected too.
#
# Consequence: a boresight entered here is valid ONLY for the camera
# positions in force when it was fitted. Re-survey the mounts and it becomes
# actively wrong. Record the positions alongside the offsets and refuse to
# use one against the other -- this is exactly the failure in VSL's
# boresight_offsets.json, which stores calib_heading_deg and never checks it.
#
# A persistent, direction-consistent skew is extrinsic misalignment, not noise.
# This tracks a slow per-camera yaw/pitch offset and can feed it back as a
# correction. It refines whatever was entered above rather than replacing it.
# Off by default: verify against a known-good rig first.
ESTIMATE_BORESIGHT_BIAS = False
BORESIGHT_BIAS_TAU_S = 30.0   # smoothing time constant
BORESIGHT_BIAS_MAX_DEG = 1.0  # refuse to "calibrate" beyond a plausible offset

# ============================================================
#  Diagnostics
# ============================================================
LOG_DIR = "logs"
DIAG_PRINT_INTERVAL_S = 1.0


def sigma_yaw_rad(cam_cfg):
    return float(np.radians(cam_cfg["sigma_yaw_deg"]))


def sigma_pitch_rad(cam_cfg):
    return float(np.radians(cam_cfg["sigma_pitch_deg"]))
