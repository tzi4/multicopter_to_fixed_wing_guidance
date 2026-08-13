"""
stereo_geometry.py -- camera model, bearing rays, triangulation, conditioning.

Conventions (fixed once, used everywhere):
    World frame  : NED. north, east, down (metres). Altitude = -down.
    Camera frame : x = boresight, y = right, z = down.
    Pose         : yaw/pitch/roll, body->NED 3-2-1 (yaw about down, pitch
                   nose-up positive, roll right-wing-down positive).
    Bearing      : yaw   = atan2(y_c, x_c)  -- right of boresight positive
                   pitch = atan2(-z_c, hypot(x_c, y_c)) -- up positive

The two rays from a stereo pair are skew lines: they essentially never
intersect. That is the normal case, not a failure. The perpendicular miss
distance ("skew") is kept and returned everywhere as a free quality signal.

Triangulation here is deliberately NOT the primary measurement path -- the
estimator consumes angles directly (see stereo_measurement.py). These helpers
exist to seed a track, hint the turn-rate estimator, and report conditioning.
"""

import numpy as np

_EPS = 1e-12


# ------------------------------------------------------------------
#  Rotations
# ------------------------------------------------------------------
def rot_body_to_ned(yaw_rad, pitch_rad, roll_rad):
    """3-2-1 body->NED direction cosine matrix."""
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def wrap_pi(angle):
    """Wrap to (-pi, pi]. Essential: bearing residuals must never be taken
    naively, or a target crossing the +-pi seam produces a ~2pi innovation."""
    return (np.asarray(angle, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


# ------------------------------------------------------------------
#  Camera
# ------------------------------------------------------------------
class Camera:
    """A bearing-only sensor: known pose, per-axis angular noise, finite FOV."""

    def __init__(
        self,
        name,
        position_ned,
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        sigma_yaw_deg=0.06,
        sigma_pitch_deg=0.06,
        fov_yaw_deg=60.0,
        fov_pitch_deg=40.0,
        max_range_m=1200.0,
        bias_yaw_deg=0.0,
        bias_pitch_deg=0.0,
    ):
        self.name = str(name)
        self.position = np.asarray(position_ned, dtype=float).reshape(3)
        self.set_orientation(yaw_deg, pitch_deg, roll_deg)
        self.sigma_yaw = float(np.radians(sigma_yaw_deg))
        self.sigma_pitch = float(np.radians(sigma_pitch_deg))
        self.half_fov_yaw = float(np.radians(fov_yaw_deg)) / 2.0
        self.half_fov_pitch = float(np.radians(fov_pitch_deg)) / 2.0
        self.max_range_m = float(max_range_m)
        # Boresight correction SUBTRACTED from incoming measurements. Applied
        # once at ingest (StereoTracker.process) so that triangulation, the
        # skew gate, the init seed and the filter update all see the same
        # corrected angles -- before 2026-07-31 only the update did, which
        # meant a hand-entered offset silently left the geometry path biased.
        self.bias_yaw = float(np.radians(bias_yaw_deg))
        self.bias_pitch = float(np.radians(bias_pitch_deg))

    @classmethod
    def from_config(cls, cfg):
        return cls(
            name=cfg["name"],
            position_ned=cfg["position_ned"],
            yaw_deg=cfg.get("yaw_deg", 0.0),
            pitch_deg=cfg.get("pitch_deg", 0.0),
            roll_deg=cfg.get("roll_deg", 0.0),
            sigma_yaw_deg=cfg.get("sigma_yaw_deg", 0.06),
            sigma_pitch_deg=cfg.get("sigma_pitch_deg", 0.06),
            fov_yaw_deg=cfg.get("fov_yaw_deg", 60.0),
            fov_pitch_deg=cfg.get("fov_pitch_deg", 40.0),
            max_range_m=cfg.get("max_range_m", 1200.0),
            bias_yaw_deg=cfg.get("bias_yaw_deg", 0.0),
            bias_pitch_deg=cfg.get("bias_pitch_deg", 0.0),
        )

    def set_orientation(self, yaw_deg, pitch_deg, roll_deg):
        self.yaw = float(np.radians(yaw_deg))
        self.pitch = float(np.radians(pitch_deg))
        self.roll = float(np.radians(roll_deg))
        self.R_cw = rot_body_to_ned(self.yaw, self.pitch, self.roll)  # cam->world
        self.R_wc = self.R_cw.T                                       # world->cam

    def set_pose(self, position_ned, yaw_deg, pitch_deg, roll_deg):
        """Move the camera (rig mounted on a moving vehicle)."""
        self.position = np.asarray(position_ned, dtype=float).reshape(3)
        self.set_orientation(yaw_deg, pitch_deg, roll_deg)

    # -- forward model -------------------------------------------------
    def to_camera_frame(self, point_ned):
        d = np.asarray(point_ned, dtype=float).reshape(3) - self.position
        return self.R_wc @ d

    def bearing_of(self, point_ned):
        """True (yaw, pitch) of a world point, noise-free."""
        d = self.to_camera_frame(point_ned)
        rho = float(np.hypot(d[0], d[1]))
        yaw = float(np.arctan2(d[1], d[0]))
        pitch = float(np.arctan2(-d[2], max(rho, _EPS)))
        return yaw, pitch

    def range_to(self, point_ned):
        return float(
            np.linalg.norm(np.asarray(point_ned, dtype=float).reshape(3) - self.position)
        )

    def ray_direction(self, yaw, pitch):
        """Unit direction in WORLD frame for a measured bearing."""
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        d_cam = np.array([cp * cy, cp * sy, -sp], dtype=float)
        d_world = self.R_cw @ d_cam
        n = np.linalg.norm(d_world)
        return d_world / n if n > _EPS else d_world

    def sees(self, point_ned):
        """FOV + range check against the true geometry (simulator-side)."""
        d = self.to_camera_frame(point_ned)
        r = float(np.linalg.norm(d))
        if r > self.max_range_m or d[0] <= 0.0:
            return False  # behind the camera or too far
        yaw, pitch = self.bearing_of(point_ned)
        return abs(yaw) <= self.half_fov_yaw and abs(pitch) <= self.half_fov_pitch

    def bearing_in_fov(self, yaw, pitch):
        return abs(yaw) <= self.half_fov_yaw and abs(pitch) <= self.half_fov_pitch

    # -- linearisation -------------------------------------------------
    def bearing_jacobian(self, point_ned):
        """d(yaw, pitch)/d(world position): 2x3, plus the predicted bearing.

        In camera frame with d = R_wc (x - p), r = |d|, rho = hypot(dx, dy):
            d(yaw)/dd   = [-dy/rho^2,  dx/rho^2, 0]
            d(pitch)/dd = [dx*dz/(r^2*rho), dy*dz/(r^2*rho), -rho/r^2]
        then chain through dd/dx = R_wc.

        Both rows blow up as rho -> 0 (target on the boresight axis for yaw) --
        that is real: bearing tells you nothing about range, and the yaw of a
        point straight ahead is genuinely undefined. Guarded, and the caller
        should treat huge jacobian norms as a degenerate frame.
        """
        d = self.to_camera_frame(point_ned)
        dx, dy, dz = float(d[0]), float(d[1]), float(d[2])
        r2 = dx * dx + dy * dy + dz * dz
        rho2 = dx * dx + dy * dy
        rho = float(np.sqrt(max(rho2, _EPS)))
        r2 = max(r2, _EPS)
        rho2 = max(rho2, _EPS)

        j_cam = np.array(
            [
                [-dy / rho2, dx / rho2, 0.0],
                [dx * dz / (r2 * rho), dy * dz / (r2 * rho), -rho / r2],
            ],
            dtype=float,
        )
        j_world = j_cam @ self.R_wc  # 2x3, w.r.t. world position
        yaw = float(np.arctan2(dy, dx))
        pitch = float(np.arctan2(-dz, rho))
        return j_world, np.array([yaw, pitch], dtype=float)

    def __repr__(self):
        p = self.position
        return (
            f"Camera({self.name!r}, pos=({p[0]:.1f},{p[1]:.1f},{p[2]:.1f}), "
            f"yaw={np.degrees(self.yaw):.1f}deg, pitch={np.degrees(self.pitch):.1f}deg, "
            f"sigma=({np.degrees(self.sigma_yaw)*1000:.2f},"
            f"{np.degrees(self.sigma_pitch)*1000:.2f})mdeg)"
        )


# ------------------------------------------------------------------
#  Detections
# ------------------------------------------------------------------
class Detection:
    """One camera's angular observation at one instant."""

    __slots__ = ("cam_index", "yaw", "pitch", "stamp", "valid", "meta")

    def __init__(self, cam_index, yaw, pitch, stamp, valid=True, meta=None):
        self.cam_index = int(cam_index)
        self.yaw = float(yaw)
        self.pitch = float(pitch)
        self.stamp = float(stamp)
        self.valid = bool(valid)
        self.meta = meta or {}

    def __repr__(self):
        return (
            f"Detection(cam={self.cam_index}, yaw={np.degrees(self.yaw):.3f}deg, "
            f"pitch={np.degrees(self.pitch):.3f}deg, t={self.stamp:.3f})"
        )


# ------------------------------------------------------------------
#  Two-ray geometry
# ------------------------------------------------------------------
def closest_approach(p1, d1, p2, d2):
    """Closest points of two (skew) rays.

    Returns (q1, q2, s, t, skew, parallel) where q1 = p1 + s*d1, q2 = p2 + t*d2
    and skew = |q1 - q2| is the perpendicular miss distance. `parallel` flags a
    degenerate pair (rays nearly collinear -> depth unobservable this frame).
    """
    p1 = np.asarray(p1, dtype=float).reshape(3)
    p2 = np.asarray(p2, dtype=float).reshape(3)
    d1 = np.asarray(d1, dtype=float).reshape(3)
    d2 = np.asarray(d2, dtype=float).reshape(3)
    d1 = d1 / max(np.linalg.norm(d1), _EPS)
    d2 = d2 / max(np.linalg.norm(d2), _EPS)

    w0 = p1 - p2
    b = float(d1 @ d2)
    d = float(d1 @ w0)
    e = float(d2 @ w0)
    denom = 1.0 - b * b  # a*c - b^2 with a = c = 1 for unit directions

    if denom < 1e-9:
        # Near-parallel: no meaningful common perpendicular. Fall back to the
        # foot of p2 on ray 1 so callers still get something finite.
        s = -d
        t = 0.0
        parallel = True
    else:
        s = (b * e - d) / denom
        t = (e - b * d) / denom
        parallel = False

    q1 = p1 + s * d1
    q2 = p2 + t * d2
    return q1, q2, float(s), float(t), float(np.linalg.norm(q1 - q2)), parallel


def parallax_angle(d1, d2):
    """Angle between the two rays [rad]. Small parallax = poor depth."""
    d1 = np.asarray(d1, dtype=float).reshape(3)
    d2 = np.asarray(d2, dtype=float).reshape(3)
    c = float(np.clip((d1 @ d2) / max(np.linalg.norm(d1) * np.linalg.norm(d2), _EPS), -1.0, 1.0))
    return float(np.arccos(c))


def triangulate_midpoint(cam1, det1, cam2, det2):
    """Weighted closest-point fix. Cheap, robust; the seed for the ML solve.

    The weighting is the point: a naive midpoint trusts both rays equally, but
    cross-range error is (range * sigma), so the nearer / sharper camera should
    dominate. Weights are inverse cross-range variance.
    """
    d1 = cam1.ray_direction(det1.yaw, det1.pitch)
    d2 = cam2.ray_direction(det2.yaw, det2.pitch)
    q1, q2, s, t, skew, parallel = closest_approach(cam1.position, d1, cam2.position, d2)

    sig1 = max(np.hypot(cam1.sigma_yaw, cam1.sigma_pitch), _EPS)
    sig2 = max(np.hypot(cam2.sigma_yaw, cam2.sigma_pitch), _EPS)
    # cross-range 1-sigma at each ray's closest point
    c1 = max(abs(s), 1.0) * sig1
    c2 = max(abs(t), 1.0) * sig2
    w1 = 1.0 / (c1 * c1)
    w2 = 1.0 / (c2 * c2)
    point = (w1 * q1 + w2 * q2) / (w1 + w2)

    info = {
        "skew_m": skew,
        "parallel": parallel,
        "range1_m": abs(s),
        "range2_m": abs(t),
        "parallax_rad": parallax_angle(d1, d2),
        "behind": (s <= 0.0) or (t <= 0.0),
    }
    return point, info


def triangulate_ml(cameras, detections, x0, iters=6, huber_delta=None):
    """Maximum-likelihood fix in ANGLE space (the statistically right snapshot).

    Minimises sum over observations of (wrapped angular residual / sigma)^2 by
    Gauss-Newton. This is where the known per-axis yaw/pitch errors enter
    natively -- unlike a midpoint, which implicitly assumes isotropic metric
    error and quietly mis-weights long-range rays.

    Works with ONE camera too (2 equations, 3 unknowns): the solve is then
    rank-deficient along the ray and the damped normal equations simply leave
    depth at the seed value. That is the correct behaviour, not a failure.
    """
    x = np.asarray(x0, dtype=float).reshape(3).copy()
    n_obs = len(detections)
    if n_obs == 0:
        return x, {"converged": False, "cost": float("nan"), "iters": 0, "rank_deficient": True}

    cost = float("nan")
    for it in range(int(max(1, iters))):
        rows, res, wts = [], [], []
        for det in detections:
            cam = cameras[det.cam_index]
            j_world, pred = cam.bearing_jacobian(x)
            rows.append(j_world[0]); res.append(wrap_pi(det.yaw - pred[0])); wts.append(1.0 / cam.sigma_yaw ** 2)
            rows.append(j_world[1]); res.append(wrap_pi(det.pitch - pred[1])); wts.append(1.0 / cam.sigma_pitch ** 2)
        J = np.asarray(rows, dtype=float)          # (2N, 3)
        r = np.asarray(res, dtype=float)           # (2N,)
        w = np.asarray(wts, dtype=float)           # (2N,)

        if huber_delta is not None and huber_delta > 0.0:
            # Down-weight gross angular outliers without discarding them.
            a = np.abs(r) * np.sqrt(w)
            scale = np.where(a > huber_delta, huber_delta / np.maximum(a, _EPS), 1.0)
            w = w * scale

        cost = float(np.sum(w * r * r))
        JtW = J.T * w
        H = JtW @ J                                 # (3,3) Fisher information
        g = JtW @ r
        # Levenberg damping keeps the ill-conditioned depth direction sane.
        lam = 1e-9 * max(float(np.trace(H)), _EPS)
        try:
            delta = np.linalg.solve(H + lam * np.eye(3), g)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(H) @ g
        if not np.all(np.isfinite(delta)):
            return x, {"converged": False, "cost": cost, "iters": it, "rank_deficient": True}
        # Cap the step: a wild first iteration from a bad seed can throw the
        # solution behind the cameras, from which Gauss-Newton will not recover.
        step = float(np.linalg.norm(delta))
        max_step = max(50.0, 0.5 * float(np.linalg.norm(x - cameras[detections[0].cam_index].position)))
        if step > max_step:
            delta *= max_step / step
        x = x + delta
        if step < 1e-4:
            break

    return x, {"converged": True, "cost": cost, "iters": it + 1, "rank_deficient": False}


def triangulation_covariance(cameras, detections, x):
    """3x3 position covariance of the fix (inverse Fisher information / CRLB).

    THIS is the product, not a diagnostic: it is the anisotropic 'cigar' that
    must be handed to the filter. Returns (P, info) with the condition number
    and the 1-sigma extent along the dominant (usually depth) axis.
    """
    x = np.asarray(x, dtype=float).reshape(3)
    rows, wts = [], []
    for det in detections:
        cam = cameras[det.cam_index]
        j_world, _ = cam.bearing_jacobian(x)
        rows.append(j_world[0]); wts.append(1.0 / cam.sigma_yaw ** 2)
        rows.append(j_world[1]); wts.append(1.0 / cam.sigma_pitch ** 2)
    if not rows:
        return np.eye(3) * 1e6, {"cond": float("inf"), "rank_deficient": True, "sigma_max_m": float("inf")}

    J = np.asarray(rows, dtype=float)
    w = np.asarray(wts, dtype=float)
    H = (J.T * w) @ J
    try:
        evals = np.linalg.eigvalsh(H)
        cond = float(np.max(evals) / max(np.min(evals), _EPS))
    except np.linalg.LinAlgError:
        cond = float("inf")

    rank_deficient = (not np.isfinite(cond)) or cond > 1e12
    if rank_deficient:
        P = np.linalg.pinv(H, rcond=1e-10)
        # pinv leaves the unobservable direction at zero variance, which would
        # be a catastrophic lie to the filter. Restore it as "very uncertain".
        evals_h, evecs = np.linalg.eigh(H)
        floor = 1e-8 * max(float(np.max(evals_h)), _EPS)
        weak = evecs[:, evals_h < floor]
        if weak.size:
            P = P + (weak @ weak.T) * (1e4 ** 2)
    else:
        P = np.linalg.inv(H)

    P = 0.5 * (P + P.T)
    sig_max = float(np.sqrt(max(np.max(np.linalg.eigvalsh(P)), 0.0)))
    return P, {"cond": cond, "rank_deficient": rank_deficient, "sigma_max_m": sig_max}


def los_decompose(error_vec, los_unit):
    """Split a position error into along-LOS (depth) and cross-LOS parts.

    Reporting a single scalar RMS hides the whole story of a bearings-only
    sensor, where depth error is an order of magnitude worse than cross-range.
    """
    e = np.asarray(error_vec, dtype=float).reshape(3)
    u = np.asarray(los_unit, dtype=float).reshape(3)
    n = np.linalg.norm(u)
    if n < _EPS:
        return float(np.linalg.norm(e)), 0.0
    u = u / n
    along = float(e @ u)
    cross = float(np.linalg.norm(e - along * u))
    return along, cross


def build_cameras(cfg_list):
    return [Camera.from_config(c) for c in cfg_list]
