"""
stereo_measurement.py -- feed ANGLES to the IMM, not a triangulated position.

This is the core architectural change of the stereo pivot. The old sensor
handed us a position and we set R = sigma_pos^2 * I. A stereo bearing pair
does NOT measure a position: it measures four scalars

    z = [yaw_left, pitch_left, yaw_right, pitch_right]

each with its own known angular sigma. Triangulating first and calling the
result "a position measurement" throws away the anisotropy (0.2 m across the
line of sight, tens of metres along it) and forces a lie about R. Updating on
the angles directly keeps the geometry honest: the filter tightens what the
rays constrain and leaves depth uncertain until parallax -- from the baseline
or, far more powerfully, from motion over time -- earns it.

Three consequences worth stating, because they are free wins:
  * non-intersecting rays stop being a special case. There is nothing to
    intersect; four scalars are fused independently.
  * a SINGLE camera still updates the track (two constraints instead of four).
    No stereo pair required, no dropout hole.
  * per-scalar innovation gating falls out naturally, so one camera throwing a
    false detection cannot drag the state.

Implementation notes:
  * Sequential scalar updates. Each of the four angles is applied one at a
    time, recomputing h(x) and its jacobian against the partially-updated
    state. That is the exact chain-rule factorisation of the joint likelihood
    p(z1..z4) = p(z1) p(z2|z1) p(z3|z1,z2) p(z4|...), so the mode likelihoods
    handed to the IMM stay correct, and it needs no matrix inversion.
  * Joseph-form covariance update, matching filterwndr's linear modes.
  * Gating is decided ONCE from the IMM's combined prior and then applied
    identically to every mode. If each mode gated independently they would be
    conditioning on different measurement sets and their likelihoods would no
    longer be comparable -- which silently corrupts the mode probabilities.
  * The caller must PREDICT before calling here. filterpy's IMM computes cbar
    during mixing, and mu = cbar * likelihood; updating without a preceding
    predict consumes a stale cbar. Same invariant the position pipeline relies
    on (see the OOSM warning in filterwndr.py).
"""

import numpy as np

import stereo_config as scfg
from stereo_geometry import wrap_pi

_EPS = 1e-12


# ------------------------------------------------------------------
#  Scalar Kalman update
# ------------------------------------------------------------------
def scalar_update(x, P, h_row, innovation, var):
    """One scalar measurement update. Returns (x, P, S, nis, loglik).

    h_row : (n,) measurement jacobian row
    innovation : already-wrapped residual (z - h(x))
    var : measurement variance for this scalar
    """
    n = x.shape[0]
    H = np.asarray(h_row, dtype=float).reshape(1, n)
    PHt = P @ H.T                       # (n,1)
    S = float(H @ PHt) + float(var)
    if not np.isfinite(S) or S <= _EPS:
        return x, P, S, float("inf"), 0.0
    K = PHt / S                         # (n,1)
    y = float(innovation)
    x_new = x + (K * y).reshape(n)
    A = np.eye(n) - K @ H
    P_new = A @ P @ A.T + (K * float(var)) @ K.T   # Joseph form
    P_new = 0.5 * (P_new + P_new.T)
    nis = (y * y) / S
    loglik = -0.5 * (nis + np.log(2.0 * np.pi * S))
    return x_new, P_new, S, float(nis), float(loglik)


# ------------------------------------------------------------------
#  Per-frame measurement plan (gating decided once, on the IMM prior)
# ------------------------------------------------------------------
class ScalarSpec:
    """One angular scalar to be applied: which camera, which axis, what noise."""

    __slots__ = ("cam_index", "axis", "z", "var", "base_var", "nis_prior",
                 "innov", "accepted", "inflation", "reason")

    def __init__(self, cam_index, axis, z, base_var):
        self.cam_index = cam_index
        self.axis = axis            # 0 = yaw, 1 = pitch
        self.z = float(z)
        self.base_var = float(base_var)
        self.var = float(base_var)
        self.nis_prior = float("nan")
        self.innov = float("nan")   # prior residual, drives bias estimation
        self.accepted = True
        self.inflation = 1.0
        self.reason = ""

    def __repr__(self):
        ax = "yaw" if self.axis == 0 else "pitch"
        return (f"ScalarSpec(cam{self.cam_index},{ax},"
                f"nis={self.nis_prior:.1f},infl={self.inflation:.1f},"
                f"acc={self.accepted})")


def build_measurement_plan(cameras, detections, x_prior, P_prior,
                           frame_noise_scale=1.0, cfg=scfg):
    """Expand detections into gated scalar specs using the combined prior.

    frame_noise_scale lets a frame-level quality signal (e.g. ray skew) inflate
    all of that frame's angular noise before gating.
    """
    specs = []
    pos = np.asarray(x_prior, dtype=float).reshape(-1)[0:3]
    gate_mode = getattr(cfg, "NIS_GATE_MODE", "inflate")
    gate_chi2 = float(getattr(cfg, "NIS_GATE_CHI2", 9.0))
    infl_max = float(getattr(cfg, "NIS_INFLATE_MAX", 100.0))
    hard_chi2 = float(getattr(cfg, "NIS_HARD_REJECT_CHI2", 400.0))
    s2 = float(frame_noise_scale) ** 2

    for det in detections:
        if not det.valid:
            continue
        cam = cameras[det.cam_index]
        j_world, pred = cam.bearing_jacobian(pos)
        for axis, (z_meas, pred_ang, sigma) in enumerate(
            (
                # Boresight is removed at ingest (StereoTracker._debias), so
                # the angles arriving here are already corrected. Subtracting
                # again would double-apply it.
                (det.yaw, pred[0], cam.sigma_yaw),
                (det.pitch, pred[1], cam.sigma_pitch),
            )
        ):
            spec = ScalarSpec(det.cam_index, axis, z_meas, (sigma ** 2) * s2)
            # NIS against the combined prior
            n = np.asarray(x_prior).shape[0]
            H = np.zeros(n)
            H[0:3] = j_world[axis]
            S = float(H @ P_prior @ H) + spec.var
            y = float(wrap_pi(z_meas - pred_ang))
            spec.innov = y
            spec.nis_prior = (y * y) / max(S, _EPS)

            # Non-finite must be caught EXPLICITLY: every comparison against
            # NaN below is False, so a NaN would sail through the gate,
            # reach the update, and take every mode's likelihood to -inf --
            # discarding the whole frame including the other camera's good
            # scalars. Callers normally filter these first; this is the
            # backstop that keeps one bad scalar from costing the frame.
            if not (np.isfinite(y) and np.isfinite(S) and np.isfinite(spec.var)):
                spec.accepted = False
                spec.reason = "non-finite"
                specs.append(spec)
                continue

            if gate_mode == "off":
                pass
            elif spec.nis_prior > hard_chi2:
                spec.accepted = False
                spec.reason = "hard-reject"
            elif spec.nis_prior > gate_chi2:
                if gate_mode == "reject":
                    spec.accepted = False
                    spec.reason = "gated"
                else:  # inflate: soften rather than drop, so a real manoeuvre
                    # onset is not stonewalled for a full packet
                    infl = min(spec.nis_prior / gate_chi2, infl_max)
                    spec.inflation = infl
                    spec.var *= infl
                    spec.reason = "inflated"
            specs.append(spec)
    return specs


# ------------------------------------------------------------------
#  Filter- and IMM-level updates
# ------------------------------------------------------------------
def update_filter_with_specs(kf, cameras, specs):
    """Apply the accepted scalars sequentially to one filter. -> total loglik."""
    x = np.asarray(kf.x, dtype=float).reshape(-1).copy()
    P = np.asarray(kf.P, dtype=float).copy()
    n = x.shape[0]
    total_ll = 0.0
    nis_list = []

    for spec in specs:
        if not spec.accepted:
            continue
        cam = cameras[spec.cam_index]
        # Recompute h and H against the partially-updated state: this is the
        # correct sequential factorisation, not an approximation of a batch.
        j_world, pred = cam.bearing_jacobian(x[0:3])
        H = np.zeros(n)
        H[0:3] = j_world[spec.axis]
        y = float(wrap_pi(spec.z - pred[spec.axis]))
        x, P, S, nis, ll = scalar_update(x, P, H, y, spec.var)
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(P)):
            return None, float("-inf"), nis_list
        total_ll += ll
        nis_list.append(nis)

    kf.x = x
    kf.P = P
    return x, total_ll, nis_list


def imm_bearing_update(imm, cameras, detections, frame_noise_scale=1.0,
                       cfg=scfg, filterwndr=None):
    """Update the whole product-IMM on bearing scalars.

    Mirrors filterpy's IMMEstimator.update contract (likelihood -> mu ->
    mixing probabilities -> combined estimate) but with a nonlinear angular
    measurement, sequential scalars, and shared gating.

    Returns a diagnostics dict. Requires a preceding predict (see module docs).
    """
    diags = {
        "n_scalars": 0, "n_accepted": 0, "n_gated": 0, "n_inflated": 0,
        "nis_max": 0.0, "nis_mean": float("nan"), "specs": [], "updated": False,
    }
    if not detections:
        return diags

    x_prior = np.asarray(imm.x, dtype=float).reshape(-1)
    P_prior = np.asarray(imm.P, dtype=float)
    specs = build_measurement_plan(
        cameras, detections, x_prior, P_prior,
        frame_noise_scale=frame_noise_scale, cfg=cfg,
    )
    if not specs:
        return diags

    diags["specs"] = specs
    diags["n_scalars"] = len(specs)
    diags["n_accepted"] = sum(1 for s in specs if s.accepted)
    diags["n_gated"] = sum(1 for s in specs if not s.accepted)
    diags["n_inflated"] = sum(1 for s in specs if s.accepted and s.inflation > 1.0)
    nis_all = [s.nis_prior for s in specs if np.isfinite(s.nis_prior)]
    if nis_all:
        diags["nis_max"] = float(np.max(nis_all))
        diags["nis_mean"] = float(np.mean(nis_all))
    if diags["n_accepted"] == 0:
        return diags

    # --- per-mode update -------------------------------------------------
    logliks = np.zeros(len(imm.filters))
    for i, kf in enumerate(imm.filters):
        _, ll, _ = update_filter_with_specs(kf, cameras, specs)
        logliks[i] = ll

    # --- mode probabilities (filterpy IMM contract) ----------------------
    finite = np.isfinite(logliks)
    if not np.any(finite):
        return diags
    ll_max = float(np.max(logliks[finite]))
    likelihood = np.zeros_like(logliks)
    likelihood[finite] = np.exp(logliks[finite] - ll_max)   # stabilised
    imm.likelihood = likelihood

    cbar = getattr(imm, "cbar", None)
    if cbar is None:
        cbar = np.asarray(imm.mu, dtype=float) @ imm.M
    mu = np.asarray(cbar, dtype=float) * likelihood
    total = float(np.sum(mu))
    if np.isfinite(total) and total > 0.0:
        imm.mu = mu / total
    # else: keep the prior mu -- a degenerate likelihood must not blank it out

    if filterwndr is not None:
        filterwndr._normalize_mode_probabilities(imm)
    imm._compute_mixing_probabilities()
    if filterwndr is not None:
        filterwndr.stabilize_omega_states(imm)
    imm._compute_state_estimate()
    if filterwndr is not None:
        imm.x[9] = filterwndr.get_effective_turn_rate(imm)
    imm.x_post = imm.x.copy()
    imm.P_post = imm.P.copy()

    diags["updated"] = True
    diags["loglik"] = logliks
    return diags


# ------------------------------------------------------------------
#  Frame quality -> noise scaling
# ------------------------------------------------------------------
class BoresightEstimator:
    """Recover slow DIFFERENTIAL boresight offsets from persistent residuals.

    Why this matters more than it looks: a fixed angular misalignment is not
    noise, so the filter cannot average it away -- but it does keep shrinking
    its covariance as if it could. The result is a biased estimate that is also
    *confident*, which is the one combination the downstream covariance gate
    cannot defend against. Measured on the test bench, a mere 0.02 deg
    differential bias roughly doubles depth error and leaves the filter ~3x
    overconfident; 0.05 deg is far worse. In depth terms the penalty is
    R^2/b * delta_theta -- the same brutal lever that makes depth hard in the
    first place, now driven by a screw that moved.

    Only the DIFFERENTIAL component (camera-to-camera disagreement) is
    estimated. A common-mode offset, where both cameras are rotated the same
    way, barely shows up in the residuals -- it mostly slides the whole track
    sideways rather than breaking the triangulation -- so trying to observe it
    invites slow drift against the target's own motion. The differential part
    is exactly what corrupts depth, and it is exactly what ray skew reveals.
    """

    def __init__(self, n_cameras, tau_s=30.0, max_bias_rad=0.0175, enabled=False):
        self.n = int(n_cameras)
        self.tau = float(tau_s)
        self.max_bias = abs(float(max_bias_rad))
        self.enabled = bool(enabled)
        # `base` is the operator-entered boresight; `bias` is the increment
        # this estimator has learned on top of it. Keeping them apart matters:
        # max_bias must bound the DRIFT we are willing to infer online, not the
        # surveyed constant someone deliberately entered, and apply() must not
        # silently wipe that constant back to zero on the first frame.
        self.base = np.zeros((self.n, 2))
        self.bias = np.zeros((self.n, 2))
        self.samples = 0

    def set_base(self, cameras):
        """Adopt the cameras' entered boresight as the zero of estimation."""
        for i, cam in enumerate(cameras[: self.n]):
            self.base[i, 0] = float(cam.bias_yaw)
            self.base[i, 1] = float(cam.bias_pitch)

    def observe(self, residuals, dt):
        """Fold one frame's residuals into the bias estimate.

        `residuals` must be {(cam_index, axis): residual} measured AT THE ML
        TRIANGULATION FIX, not the filter innovation. That distinction is the
        whole ballgame. The filter (and any position solve) will happily absorb
        a differential bias by moving the target in depth -- the innovation
        then goes to zero while the estimate is quietly wrong. Only the part of
        the residual that NO position can explain is evidence of misalignment,
        and at the ML fix that is exactly what is left over.
        """
        if not self.enabled or dt <= 0.0 or not residuals:
            return
        alpha = float(dt / (dt + max(self.tau, 1e-6)))
        for axis in (0, 1):
            vals = {c: r for (c, a), r in residuals.items() if a == axis and np.isfinite(r)}
            if len(vals) < self.n:
                continue  # differential is undefined unless every camera saw it
            mean = float(np.mean(list(vals.values())))
            for cam_index, y in vals.items():
                b = self.bias[cam_index, axis] + alpha * (y - mean)
                self.bias[cam_index, axis] = float(np.clip(b, -self.max_bias, self.max_bias))
            self.samples += 1

    def apply(self, cameras):
        if not self.enabled:
            return
        for i, cam in enumerate(cameras[: self.n]):
            cam.bias_yaw = float(self.base[i, 0] + self.bias[i, 0])
            cam.bias_pitch = float(self.base[i, 1] + self.bias[i, 1])

    def report_deg(self):
        """Total correction in force (entered + estimated)."""
        return np.degrees(self.base + self.bias)

    def estimated_deg(self):
        """Only the part this estimator inferred online."""
        return np.degrees(self.bias)


def skew_noise_scale(skew_m, cfg=scfg):
    """Turn the ray miss distance into an angular-noise multiplier.

    Skew is the one thing a stereo pair gives you for free that a single
    position sensor never could: an instantaneous, self-contained consistency
    check. Small skew = the rays agree = trust the frame. Large skew = boresight
    drift, a bad detection, a time-sync slip, or a ghost pairing.
    """
    lo = float(getattr(cfg, "SKEW_INFLATE_M", 3.0))
    hi = float(getattr(cfg, "SKEW_REJECT_M", 60.0))
    if not np.isfinite(skew_m):
        return float("inf")
    if skew_m <= lo:
        return 1.0
    if skew_m >= hi:
        return float("inf")   # caller drops the frame
    return float(skew_m / lo)
