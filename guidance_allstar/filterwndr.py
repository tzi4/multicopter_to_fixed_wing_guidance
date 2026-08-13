import ast
import csv
import math
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
from filterpy.kalman import (
    IMMEstimator,
    KalmanFilter,
    MerweScaledSigmaPoints,
    UnscentedKalmanFilter,
)

SIGMA_A_CV = 2.0
SIGMA_A_CT = 2.0
SIGMA_J_CT_Z = 5.0
SIGMA_J_CT_3D = SIGMA_J_CT_Z
SIGMA_J_CA = 5.0
SIGMA_OMEGA_DOT = 0.3
OMEGA_ABS_MAX = 1.5
OMEGA_MODE_PROB_MIN = 0.20
OMEGA_STRAIGHT_THRESH = 0.05
CT_OMEGA_INITIAL_STD = 0.5
CT_MIN_SPEED_FOR_AXIS = 0.5
CT_MIN_TURN_ACCEL = 0.05
UNUSED_STATE_VARIANCE = 1e-6
TURN_HINT_MIN_SPEED = 2.0
TURN_HINT_DEADBAND = 0.08
TURN_HINT_FULL_SCALE = 0.35
TURN_HINT_ALPHA = 0.45
TURN_HINT_MU_BLEND = 0.0
TURN_HINT_CT_MU_MIN = 0.12
TURN_HINT_CT_MU_MAX = 0.75
TURN_HINT_CA_MU_MAX = 0.15
OMEGA_DIFF_MAX_HISTORY = 7
OMEGA_DIFF_MIN_SAMPLES = 3
OMEGA_DIFF_WINDOW_S = 3.0
OMEGA_DIFF_MIN_SPAN_S = 0.15
FAST_TURN_ONSET_RAW_OMEGA = 0.07
FAST_TURN_ONSET_FULL_SCALE = 0.25
FAST_TURN_ONSET_ALPHA = 0.25
FAST_TURN_ONSET_CT_MU_FLOOR = 0.2

MIN_FILTER_DT = 0.03
MAX_FILTER_DT = 3.0
PREDICT_MAX_SUBSTEP = 0.1
DT_SMOOTHING_KEEP_PREV = 0.0
MEASUREMENT_R_DIAG = np.array([0.0225, 0.0225, 0.0225], dtype=float)

H_MODE_CV = "CVxy"
H_MODE_CT = "CTxy"
H_MODE_CA = "CAxy"
Z_MODE_CV = "CVz"
Z_MODE_CA = "CAz"

PRODUCT_MODE_SPECS = (
    ("CVxy_CVz", H_MODE_CV, Z_MODE_CV),
    ("CVxy_CAz", H_MODE_CV, Z_MODE_CA),
    ("CTxy_CVz", H_MODE_CT, Z_MODE_CV),
    ("CTxy_CAz", H_MODE_CT, Z_MODE_CA),
    ("CAxy_CVz", H_MODE_CA, Z_MODE_CV),
    ("CAxy_CAz", H_MODE_CA, Z_MODE_CA),
)
PRODUCT_MODE_NAMES = tuple(name for name, _, _ in PRODUCT_MODE_SPECS)
PRODUCT_MODE_FIELD_SUFFIXES = tuple(name.lower() for name in PRODUCT_MODE_NAMES)

H_MODE_ORDER = (H_MODE_CV, H_MODE_CT, H_MODE_CA)
Z_MODE_ORDER = (Z_MODE_CV, Z_MODE_CA)
H_MODE_INDEX = {mode: idx for idx, mode in enumerate(H_MODE_ORDER)}
Z_MODE_INDEX = {mode: idx for idx, mode in enumerate(Z_MODE_ORDER)}
H_MODE_INITIAL = np.array([0.45, 0.35, 0.20], dtype=float)
Z_MODE_INITIAL = np.array([0.70, 0.30], dtype=float)
H_MODE_TRANSITION = np.array(
    [
        [0.93, 0.06, 0.01],  # From CVxy
        [0.08, 0.90, 0.02],  # From CTxy
        [0.07, 0.06, 0.87],  # From CAxy
    ],
    dtype=float,
)
Z_MODE_TRANSITION = np.array(
    [
        [0.94, 0.06],  # From CVz
        [0.10, 0.90],  # From CAz
    ],
    dtype=float,
)

LEGACY_MODE_SPECS = (
    ("CV", H_MODE_CV, Z_MODE_CV),
    ("CT", H_MODE_CT, Z_MODE_CA),
    ("CA", H_MODE_CA, Z_MODE_CA),
)

DYNAMIC_FILTER_PARAM_NAMES = frozenset(
    (
        "SIGMA_A_CV",
        "SIGMA_A_CT",
        "SIGMA_J_CT_Z",
        "SIGMA_J_CT_3D",
        "SIGMA_J_CA",
        "SIGMA_OMEGA_DOT",
        "OMEGA_ABS_MAX",
        "OMEGA_MODE_PROB_MIN",
        "OMEGA_STRAIGHT_THRESH",
        "CT_OMEGA_INITIAL_STD",
        "CT_MIN_SPEED_FOR_AXIS",
        "CT_MIN_TURN_ACCEL",
        "UNUSED_STATE_VARIANCE",
        "TURN_HINT_MIN_SPEED",
        "TURN_HINT_DEADBAND",
        "TURN_HINT_FULL_SCALE",
        "TURN_HINT_ALPHA",
        "TURN_HINT_MU_BLEND",
        "TURN_HINT_CT_MU_MIN",
        "TURN_HINT_CT_MU_MAX",
        "TURN_HINT_CA_MU_MAX",
        "OMEGA_DIFF_MAX_HISTORY",
        "OMEGA_DIFF_MIN_SAMPLES",
        "OMEGA_DIFF_WINDOW_S",
        "OMEGA_DIFF_MIN_SPAN_S",
        "FAST_TURN_ONSET_RAW_OMEGA",
        "FAST_TURN_ONSET_FULL_SCALE",
        "FAST_TURN_ONSET_ALPHA",
        "FAST_TURN_ONSET_CT_MU_FLOOR",
        "MIN_FILTER_DT",
        "MAX_FILTER_DT",
        "PREDICT_MAX_SUBSTEP",
        "DT_SMOOTHING_KEEP_PREV",
        "MEASUREMENT_R_DIAG",
        "H_MODE_INITIAL",
        "Z_MODE_INITIAL",
        "H_MODE_TRANSITION",
        "Z_MODE_TRANSITION",
    )
)
DYNAMIC_FILTER_ARRAY_SHAPES = {
    "MEASUREMENT_R_DIAG": (3,),
    "H_MODE_INITIAL": (3,),
    "Z_MODE_INITIAL": (2,),
    "H_MODE_TRANSITION": (3, 3),
    "Z_MODE_TRANSITION": (2, 2),
}
_DYNAMIC_FILTER_PARAM_PATH = Path(__file__)
_DYNAMIC_FILTER_PARAM_MTIME_NS = None
_DYNAMIC_FILTER_PARAM_WARNING = None


def _eval_dynamic_param_node(node, env):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -_eval_dynamic_param_node(node.operand, env)
    if isinstance(node, ast.Name):
        return env[node.id]
    if isinstance(node, ast.List):
        return [_eval_dynamic_param_node(elt, env) for elt in node.elts]
    if isinstance(node, ast.Tuple):
        return tuple(_eval_dynamic_param_node(elt, env) for elt in node.elts)
    if isinstance(node, ast.Call):
        func = node.func
        is_np_array = (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "np"
            and func.attr == "array"
        )
        if is_np_array and node.args:
            return np.array(_eval_dynamic_param_node(node.args[0], env), dtype=float)
    raise ValueError(
        f"unsupported parameter expression: {ast.dump(node, include_attributes=False)}"
    )


def _normalize_probability_vector(value, shape, name):
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.all(np.isfinite(arr)) or np.any(arr < 0.0):
        raise ValueError(
            f"{name} must be a finite nonnegative array with shape {shape}"
        )
    total = float(arr.sum())
    if total <= 0.0:
        raise ValueError(f"{name} must have a positive sum")
    return arr / total


def _normalize_transition_matrix(value, shape, name):
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.all(np.isfinite(arr)) or np.any(arr < 0.0):
        raise ValueError(
            f"{name} must be a finite nonnegative array with shape {shape}"
        )
    row_sums = arr.sum(axis=1)
    if np.any(row_sums <= 0.0):
        raise ValueError(f"{name} rows must have positive sums")
    return arr / row_sums[:, None]


def _coerce_dynamic_param(name, value):
    if name == "MEASUREMENT_R_DIAG":
        arr = np.asarray(value, dtype=float)
        if arr.shape != (3,) or not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
            raise ValueError(
                "MEASUREMENT_R_DIAG must be three positive finite variances"
            )
        return arr
    if name in ("H_MODE_INITIAL", "Z_MODE_INITIAL"):
        return _normalize_probability_vector(
            value, DYNAMIC_FILTER_ARRAY_SHAPES[name], name
        )
    if name in ("H_MODE_TRANSITION", "Z_MODE_TRANSITION"):
        return _normalize_transition_matrix(
            value, DYNAMIC_FILTER_ARRAY_SHAPES[name], name
        )
    if name in DYNAMIC_FILTER_ARRAY_SHAPES:
        arr = np.asarray(value, dtype=float)
        if arr.shape != DYNAMIC_FILTER_ARRAY_SHAPES[name]:
            raise ValueError(
                f"{name} must have shape {DYNAMIC_FILTER_ARRAY_SHAPES[name]}"
            )
        return arr
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def refresh_dynamic_filter_params(force=False):
    """
    Reload tunable parameter assignments from this file when it is saved.

    This intentionally parses only an allow-list of simple constant assignments.
    It does not execute the module while the estimator is running.
    """
    global _DYNAMIC_FILTER_PARAM_MTIME_NS, _DYNAMIC_FILTER_PARAM_WARNING

    try:
        stat = _DYNAMIC_FILTER_PARAM_PATH.stat()
    except OSError as exc:
        warning = f"Could not stat dynamic filter params: {exc}"
        if warning != _DYNAMIC_FILTER_PARAM_WARNING:
            print(warning)
            _DYNAMIC_FILTER_PARAM_WARNING = warning
        return False

    if not force and _DYNAMIC_FILTER_PARAM_MTIME_NS == stat.st_mtime_ns:
        return False

    try:
        tree = ast.parse(_DYNAMIC_FILTER_PARAM_PATH.read_text())
        env = dict(globals())
        updates = {}
        for stmt in tree.body:
            if not isinstance(stmt, ast.Assign):
                continue
            for target in stmt.targets:
                if (
                    isinstance(target, ast.Name)
                    and target.id in DYNAMIC_FILTER_PARAM_NAMES
                ):
                    value = _eval_dynamic_param_node(stmt.value, env)
                    value = _coerce_dynamic_param(target.id, value)
                    updates[target.id] = value
                    env[target.id] = value

        for name, value in updates.items():
            globals()[name] = value

    except Exception as exc:
        warning = f"Ignoring invalid dynamic filter parameter edit: {exc}"
        if warning != _DYNAMIC_FILTER_PARAM_WARNING:
            print(warning)
            _DYNAMIC_FILTER_PARAM_WARNING = warning
        return False

    _DYNAMIC_FILTER_PARAM_MTIME_NS = stat.st_mtime_ns
    _DYNAMIC_FILTER_PARAM_WARNING = None
    return True


def measurement_R():
    refresh_dynamic_filter_params()
    return np.diag(MEASUREMENT_R_DIAG)


def clamp_filter_dt(dt_raw):
    refresh_dynamic_filter_params()
    return min(max(float(dt_raw), MIN_FILTER_DT), MAX_FILTER_DT)


def refresh_filter_runtime_params(imm):
    refresh_dynamic_filter_params()
    r = measurement_R()
    for kf in imm.filters:
        kf.R = r.copy()
    if _is_product_imm(imm):
        imm.M = _build_product_markov_matrix()


def _axis_indices(axis):
    return axis, axis + 3, axis + 6


def _add_cv_q_axis(Q, axis, dt, sigma_a):
    p, v, a = _axis_indices(axis)
    q = sigma_a**2
    Q[p, p] = 0.25 * dt**4 * q
    Q[p, v] = 0.5 * dt**3 * q
    Q[v, p] = 0.5 * dt**3 * q
    Q[v, v] = dt**2 * q
    Q[a, a] = 1e-8


def _add_ca_q_axis(Q, axis, dt, sigma_j):
    p, v, a = _axis_indices(axis)
    q = sigma_j**2

    Q[p, p] = dt**6 / 36 * q
    Q[p, v] = dt**5 / 12 * q
    Q[p, a] = dt**4 / 6 * q

    Q[v, p] = dt**5 / 12 * q
    Q[v, v] = dt**4 / 4 * q
    Q[v, a] = dt**3 / 2 * q

    Q[a, p] = dt**4 / 6 * q
    Q[a, v] = dt**3 / 2 * q
    Q[a, a] = dt**2 * q


def q_cv_10d(dt, sigma_a):
    Q = np.zeros((10, 10))
    q = sigma_a**2

    for i in range(3):
        p = i
        v = i + 3

        Q[p, p] = 0.25 * dt**4 * q
        Q[p, v] = 0.5 * dt**3 * q
        Q[v, p] = 0.5 * dt**3 * q
        Q[v, v] = dt**2 * q

    Q[6, 6] = 1e-8
    Q[7, 7] = 1e-8
    Q[8, 8] = 1e-8
    Q[9, 9] = 1e-8

    return Q


def q_ct_10d(dt, sigma_j, sigma_omega_dot):
    Q = np.zeros((10, 10))
    q = sigma_j**2

    # CT now uses all 3 acceleration states to infer a 3D turn plane, so all
    # axes get jerk-driven position/velocity/acceleration uncertainty.
    for i in range(3):
        p = i
        v = i + 3
        a = i + 6

        Q[p, p] = dt**6 / 36 * q
        Q[p, v] = dt**5 / 12 * q
        Q[p, a] = dt**4 / 6 * q

        Q[v, p] = dt**5 / 12 * q
        Q[v, v] = dt**4 / 4 * q
        Q[v, a] = dt**3 / 2 * q

        Q[a, p] = dt**4 / 6 * q
        Q[a, v] = dt**3 / 2 * q
        Q[a, a] = dt**2 * q

    # omega random walk
    Q[9, 9] = sigma_omega_dot**2 * dt

    return Q


def q_ca_10d(dt, sigma_j):
    Q = np.zeros((10, 10))
    q = sigma_j**2

    for i in range(3):
        p = i
        v = i + 3
        a = i + 6

        Q[p, p] = dt**6 / 36 * q
        Q[p, v] = dt**5 / 12 * q
        Q[p, a] = dt**4 / 6 * q

        Q[v, p] = dt**5 / 12 * q
        Q[v, v] = dt**4 / 4 * q
        Q[v, a] = dt**3 / 2 * q

        Q[a, p] = dt**4 / 6 * q
        Q[a, v] = dt**3 / 2 * q
        Q[a, a] = dt**2 * q

    Q[9, 9] = 1e-8
    return Q


def q_product_10d(dt, h_mode, z_mode):
    """Process noise for one horizontal x vertical product mode."""
    Q = np.zeros((10, 10))

    if h_mode == H_MODE_CA:
        _add_ca_q_axis(Q, 0, dt, SIGMA_J_CA)
        _add_ca_q_axis(Q, 1, dt, SIGMA_J_CA)
    else:
        sigma_a = SIGMA_A_CT if h_mode == H_MODE_CT else SIGMA_A_CV
        _add_cv_q_axis(Q, 0, dt, sigma_a)
        _add_cv_q_axis(Q, 1, dt, sigma_a)

    if z_mode == Z_MODE_CA:
        _add_ca_q_axis(Q, 2, dt, SIGMA_J_CA)
    else:
        _add_cv_q_axis(Q, 2, dt, SIGMA_A_CV)

    Q[9, 9] = sigma_omega_q(dt, h_mode == H_MODE_CT)
    return Q


def sigma_omega_q(dt, omega_active):
    if omega_active:
        return SIGMA_OMEGA_DOT**2 * dt
    return 1e-8


def f_product_matrix(dt, h_mode, z_mode):
    F = np.eye(10)

    F[0, 3] = dt
    F[1, 4] = dt
    if h_mode == H_MODE_CA:
        F[3, 6] = dt
        F[4, 7] = dt
        F[0, 6] = 0.5 * dt**2
        F[1, 7] = 0.5 * dt**2

    F[2, 5] = dt
    if z_mode == Z_MODE_CA:
        F[5, 8] = dt
        F[2, 8] = 0.5 * dt**2

    return F


def _ctxy_step(x, y, vx, vy, omega, dt):
    omega = float(np.clip(omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
    if abs(omega) < 1e-6:
        return x + vx * dt, y + vy * dt, vx, vy

    s = np.sin(omega * dt)
    c = np.cos(omega * dt)
    x_new = x + (s * vx - (1.0 - c) * vy) / omega
    y_new = y + ((1.0 - c) * vx + s * vy) / omega
    vx_new = c * vx - s * vy
    vy_new = s * vx + c * vy
    return x_new, y_new, vx_new, vy_new


def fx_ctxy_product(state, dt, z_mode):
    x, y, z, vx, vy, vz, ax, ay, az, omega = state
    x_new = np.asarray(state, dtype=float).copy()
    x_new[0], x_new[1], x_new[3], x_new[4] = _ctxy_step(x, y, vx, vy, omega, dt)
    x_new[6] = 0.0
    x_new[7] = 0.0

    if z_mode == Z_MODE_CA:
        x_new[2] = z + vz * dt + 0.5 * az * dt**2
        x_new[5] = vz + az * dt
        x_new[8] = az
    else:
        x_new[2] = z + vz * dt
        x_new[5] = vz
        x_new[8] = 0.0

    x_new[9] = float(np.clip(omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
    return x_new


def _make_ctxy_fx(z_mode):
    def fx(state, dt):
        return fx_ctxy_product(state, dt, z_mode)

    return fx


def _get_mode_specs(imm):
    specs = getattr(imm, "mode_specs", None)
    if specs is not None:
        return specs
    return LEGACY_MODE_SPECS[: len(imm.filters)]


def _is_product_imm(imm):
    return tuple(_get_mode_specs(imm)) == PRODUCT_MODE_SPECS


def _mode_indices(imm, h_mode=None, z_mode=None):
    indices = []
    for idx, (_, h, z) in enumerate(_get_mode_specs(imm)):
        if h_mode is not None and h != h_mode:
            continue
        if z_mode is not None and z != z_mode:
            continue
        indices.append(idx)
    return indices


def _distribution_or_default(values, default):
    values = np.asarray(values, dtype=float)
    total = float(np.sum(values))
    if total <= 0.0:
        values = np.asarray(default, dtype=float)
        total = float(np.sum(values))
    return values / total


def aggregate_mode_probabilities(imm):
    """Return horizontal and vertical marginal probabilities for product modes."""
    mu = np.asarray(imm.mu, dtype=float)
    aggregates = {
        "cv_xy": 0.0,
        "ct_xy": 0.0,
        "ca_xy": 0.0,
        "cv_z": 0.0,
        "ca_z": 0.0,
    }

    h_keys = {
        H_MODE_CV: "cv_xy",
        H_MODE_CT: "ct_xy",
        H_MODE_CA: "ca_xy",
    }
    z_keys = {
        Z_MODE_CV: "cv_z",
        Z_MODE_CA: "ca_z",
    }

    for idx, (_, h, z) in enumerate(_get_mode_specs(imm)):
        if idx >= len(mu):
            continue
        aggregates[h_keys[h]] += float(mu[idx])
        aggregates[z_keys[z]] += float(mu[idx])

    aggregates["cv"] = aggregates["cv_xy"]
    aggregates["ct"] = aggregates["ct_xy"]
    aggregates["ca"] = aggregates["ca_xy"]
    return aggregates


def aggregate_mode_likelihoods(imm):
    likelihood = np.asarray(
        getattr(imm, "likelihood", np.zeros(len(imm.filters))), dtype=float
    )
    aggregates = {
        "cv_xy": 0.0,
        "ct_xy": 0.0,
        "ca_xy": 0.0,
        "cv_z": 0.0,
        "ca_z": 0.0,
    }

    h_keys = {
        H_MODE_CV: "cv_xy",
        H_MODE_CT: "ct_xy",
        H_MODE_CA: "ca_xy",
    }
    z_keys = {
        Z_MODE_CV: "cv_z",
        Z_MODE_CA: "ca_z",
    }

    for idx, (_, h, z) in enumerate(_get_mode_specs(imm)):
        if idx >= len(likelihood):
            continue
        aggregates[h_keys[h]] += float(likelihood[idx])
        aggregates[z_keys[z]] += float(likelihood[idx])

    aggregates["cv"] = aggregates["cv_xy"]
    aggregates["ct"] = aggregates["ct_xy"]
    aggregates["ca"] = aggregates["ca_xy"]
    return aggregates


def ct_mode_probability(imm):
    return aggregate_mode_probabilities(imm)["ct_xy"]


def _horizontal_marginal(imm):
    p = aggregate_mode_probabilities(imm)
    return _distribution_or_default(
        [p["cv_xy"], p["ct_xy"], p["ca_xy"]],
        H_MODE_INITIAL,
    )


def _vertical_marginal(imm):
    p = aggregate_mode_probabilities(imm)
    return _distribution_or_default(
        [p["cv_z"], p["ca_z"]],
        Z_MODE_INITIAL,
    )


def _mu_from_marginals_for_imm(imm, h_probs, z_probs):
    h_probs = _distribution_or_default(h_probs, H_MODE_INITIAL)
    z_probs = _distribution_or_default(z_probs, Z_MODE_INITIAL)

    if not _is_product_imm(imm):
        return h_probs.copy()

    mu = []
    for _, h, z in _get_mode_specs(imm):
        mu.append(h_probs[H_MODE_INDEX[h]] * z_probs[Z_MODE_INDEX[z]])
    return _distribution_or_default(mu, np.ones(len(mu)))


def _set_horizontal_marginal_preserving_vertical(imm, h_probs):
    imm.mu = _mu_from_marginals_for_imm(imm, h_probs, _vertical_marginal(imm))
    _normalize_mode_probabilities(imm)


def get_weighted_ct_omega(imm):
    ct_indices = _mode_indices(imm, h_mode=H_MODE_CT)
    ct_mu = float(np.sum([imm.mu[idx] for idx in ct_indices]))
    if ct_mu <= 1e-9:
        return 0.0

    omega = 0.0
    for idx in ct_indices:
        omega += float(imm.mu[idx]) * float(imm.filters[idx].x[9])
    return float(np.clip(omega / ct_mu, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))


def get_weighted_ct_speed_xy(imm):
    ct_indices = _mode_indices(imm, h_mode=H_MODE_CT)
    ct_mu = float(np.sum([imm.mu[idx] for idx in ct_indices]))
    if ct_mu <= 1e-9:
        return 0.0

    speed = 0.0
    for idx in ct_indices:
        speed += float(imm.mu[idx]) * float(np.linalg.norm(imm.filters[idx].x[3:5]))
    return speed / ct_mu


def dominant_mode_name(imm):
    specs = _get_mode_specs(imm)
    idx = int(np.argmax(imm.mu))
    if idx >= len(specs):
        return ""
    return specs[idx][0]


def get_effective_turn_rate(imm):
    refresh_dynamic_filter_params()
    omega_ct = get_weighted_ct_omega(imm)
    if ct_mode_probability(imm) < OMEGA_MODE_PROB_MIN:
        return 0.0

    omega_ct = float(np.clip(omega_ct, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
    if abs(omega_ct) < OMEGA_STRAIGHT_THRESH:
        return 0.0

    return omega_ct


def stabilize_omega_states(imm):
    """Keep weakly observable omega states from polluting the IMM mixture."""
    refresh_dynamic_filter_params()
    for kf, (_, h_mode, z_mode) in zip(imm.filters, _get_mode_specs(imm)):
        if h_mode == H_MODE_CT:
            kf.x[9] = float(np.clip(kf.x[9], -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
            kf.P[9, 9] = min(float(kf.P[9, 9]), OMEGA_ABS_MAX**2)
        else:
            kf.x[9] = 0.0
            kf.P[9, :] = 0.0
            kf.P[:, 9] = 0.0
            kf.P[9, 9] = UNUSED_STATE_VARIANCE

        if _is_product_imm(imm):
            for axis in (0, 1):
                _, _, a = _axis_indices(axis)
                if h_mode != H_MODE_CA:
                    kf.x[a] = 0.0
                    kf.P[a, :] = 0.0
                    kf.P[:, a] = 0.0
                    kf.P[a, a] = UNUSED_STATE_VARIANCE

            _, _, az_idx = _axis_indices(2)
            if z_mode != Z_MODE_CA:
                kf.x[az_idx] = 0.0
                kf.P[az_idx, :] = 0.0
                kf.P[:, az_idx] = 0.0
                kf.P[az_idx, az_idx] = UNUSED_STATE_VARIANCE

    if hasattr(imm, "x"):
        imm._compute_state_estimate()
        imm.x[9] = get_effective_turn_rate(imm)


def _normalize_mode_probabilities(imm):
    imm.mu = np.asarray(imm.mu, dtype=float)
    total = float(np.sum(imm.mu))
    if total <= 0.0:
        imm.mu = np.zeros(len(imm.filters), dtype=float)
        imm.mu[0] = 1.0
    else:
        imm.mu /= total

    imm._compute_mixing_probabilities()
    imm._compute_state_estimate()


def turn_hint_strength(omega_hint):
    refresh_dynamic_filter_params()
    if omega_hint is None:
        return 0.0

    omega_abs = abs(float(omega_hint))
    if omega_abs <= TURN_HINT_DEADBAND:
        return 0.0

    return float(
        np.clip(
            (omega_abs - TURN_HINT_DEADBAND)
            / (TURN_HINT_FULL_SCALE - TURN_HINT_DEADBAND),
            0.0,
            1.0,
        )
    )


def apply_turn_rate_hint(imm, omega_hint):
    """
    Seed CT omega and gently bias mode probabilities from measured heading rate.

    Position-only IMM has weak direct observability of omega. This hint does not
    replace the Kalman update; it gives the CT model a physically plausible turn
    rate before the likelihood competition.
    """
    refresh_filter_runtime_params(imm)
    strength = turn_hint_strength(omega_hint)
    if omega_hint is None:
        stabilize_omega_states(imm)
        return 0.0

    omega_hint = float(np.clip(omega_hint, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
    ct_indices = _mode_indices(imm, h_mode=H_MODE_CT)

    if strength > 0.0:
        for idx in ct_indices:
            ct = imm.filters[idx]
            ct.x[9] = (1.0 - TURN_HINT_ALPHA) * float(
                ct.x[9]
            ) + TURN_HINT_ALPHA * omega_hint
            ct.P[9, 9] = max(float(ct.P[9, 9]), (0.25 * strength) ** 2)

        target_ct = TURN_HINT_CT_MU_MIN + strength * (
            TURN_HINT_CT_MU_MAX - TURN_HINT_CT_MU_MIN
        )
        target_ca = strength * TURN_HINT_CA_MU_MAX
        target_cv = max(0.05, 1.0 - target_ct - target_ca)
        target_h = _distribution_or_default(
            [target_cv, target_ct, target_ca],
            H_MODE_INITIAL,
        )
        target_mu = _mu_from_marginals_for_imm(imm, target_h, _vertical_marginal(imm))

        imm.mu = (1.0 - TURN_HINT_MU_BLEND) * imm.mu + TURN_HINT_MU_BLEND * target_mu
        _normalize_mode_probabilities(imm)

    stabilize_omega_states(imm)
    return strength


def fast_turn_onset_strength(raw_omega, speed_xy, min_speed=None):
    refresh_dynamic_filter_params()
    if raw_omega is None or speed_xy is None:
        return 0.0

    if min_speed is None:
        min_speed = TURN_HINT_MIN_SPEED
    if float(speed_xy) < min_speed:
        return 0.0

    omega_abs = abs(float(raw_omega))
    if omega_abs < FAST_TURN_ONSET_RAW_OMEGA:
        return 0.0

    denom = max(FAST_TURN_ONSET_FULL_SCALE - FAST_TURN_ONSET_RAW_OMEGA, 1e-6)
    return float(np.clip((omega_abs - FAST_TURN_ONSET_RAW_OMEGA) / denom, 0.0, 1.0))


def apply_fast_turn_onset_hint(imm, raw_omega, speed_xy):
    """
    Lightly seed CT from raw heading-rate onset before the smoothed turn hint reacts.

    This raises CTxy only to a floor below OMEGA_MODE_PROB_MIN. If CTxy is
    already above the gate from normal IMM evidence, raw onset may still seed the
    active CTxy omega state.
    """
    refresh_filter_runtime_params(imm)
    strength = fast_turn_onset_strength(raw_omega, speed_xy)
    if strength <= 0.0:
        stabilize_omega_states(imm)
        return 0.0

    raw_omega = float(np.clip(raw_omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
    ct_indices = _mode_indices(imm, h_mode=H_MODE_CT)
    for idx in ct_indices:
        ct = imm.filters[idx]
        ct.x[9] = (1.0 - FAST_TURN_ONSET_ALPHA) * float(
            ct.x[9]
        ) + FAST_TURN_ONSET_ALPHA * raw_omega
        ct.P[9, 9] = max(float(ct.P[9, 9]), (0.15 * strength) ** 2)

    target_ct = min(FAST_TURN_ONSET_CT_MU_FLOOR, OMEGA_MODE_PROB_MIN - 1e-3)
    h_probs = _horizontal_marginal(imm)
    if h_probs[H_MODE_INDEX[H_MODE_CT]] < target_ct:
        needed = target_ct - float(h_probs[H_MODE_INDEX[H_MODE_CT]])
        take_cv = min(needed, max(0.0, float(h_probs[H_MODE_INDEX[H_MODE_CV]]) - 0.05))
        h_probs[H_MODE_INDEX[H_MODE_CV]] -= take_cv
        h_probs[H_MODE_INDEX[H_MODE_CT]] += take_cv
        needed -= take_cv

        if needed > 0.0:
            take_ca = min(
                needed, max(0.0, float(h_probs[H_MODE_INDEX[H_MODE_CA]]) - 0.02)
            )
            h_probs[H_MODE_INDEX[H_MODE_CA]] -= take_ca
            h_probs[H_MODE_INDEX[H_MODE_CT]] += take_ca

        _set_horizontal_marginal_preserving_vertical(imm, h_probs)

    stabilize_omega_states(imm)
    return strength


class HeadingTurnRateEstimator:
    """Estimates horizontal heading rate from timestamped position samples."""

    def __init__(self, min_speed=None):
        self.min_speed = None if min_speed is None else float(min_speed)
        self.history = deque(maxlen=max(3, int(round(OMEGA_DIFF_MAX_HISTORY))))
        self.prev_pos = None
        self.prev_stamp = None
        self.prev_heading = None
        self.omega = 0.0
        self.raw_omega = 0.0
        self.speed_xy = 0.0
        self.fast_onset_strength = 0.0

    def _sync_history_limit(self):
        max_history = max(3, int(round(OMEGA_DIFF_MAX_HISTORY)))
        if self.history.maxlen != max_history:
            self.history = deque(list(self.history)[-max_history:], maxlen=max_history)

    def _prune_history_window(self, stamp):
        window_s = max(0.0, float(OMEGA_DIFF_WINDOW_S))
        if window_s <= 0.0:
            return
        while len(self.history) > 1 and stamp - self.history[0][0] > window_s:
            self.history.popleft()

    def _causal_poly_omega(self, min_speed):
        min_samples = max(3, int(round(OMEGA_DIFF_MIN_SAMPLES)))
        samples = list(self.history)
        if len(samples) < min_samples:
            return None

        t_latest = float(samples[-1][0])
        t = np.array([float(s[0]) - t_latest for s in samples], dtype=float)
        x = np.array([float(s[1]) for s in samples], dtype=float)
        y = np.array([float(s[2]) for s in samples], dtype=float)

        if (
            not np.all(np.isfinite(t))
            or not np.all(np.isfinite(x))
            or not np.all(np.isfinite(y))
        ):
            return None

        span = float(t[-1] - t[0])
        if span < max(float(OMEGA_DIFF_MIN_SPAN_S), 1e-6):
            return None

        scale = max(span, 1e-6)
        u = t / scale
        design = np.column_stack((np.ones_like(u), u, u * u))

        try:
            coeff_x, *_ = np.linalg.lstsq(design, x, rcond=None)
            coeff_y, *_ = np.linalg.lstsq(design, y, rcond=None)
        except np.linalg.LinAlgError:
            return None

        vx = float(coeff_x[1] / scale)
        vy = float(coeff_y[1] / scale)
        ax = float(2.0 * coeff_x[2] / (scale * scale))
        ay = float(2.0 * coeff_y[2] / (scale * scale))

        if not all(np.isfinite(v) for v in (vx, vy, ax, ay)):
            return None

        speed_sq = vx * vx + vy * vy
        speed_xy = math.sqrt(speed_sq)
        if speed_xy < min_speed or speed_sq < 1e-9:
            return 0.0, speed_xy

        omega = (vx * ay - vy * ax) / speed_sq
        omega = float(np.clip(omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
        return omega, speed_xy

    def update(self, pos, stamp):
        refresh_dynamic_filter_params()
        self._sync_history_limit()
        if pos is None or stamp is None:
            self.raw_omega = 0.0
            self.speed_xy = 0.0
            self.fast_onset_strength = 0.0
            return None

        pos = np.asarray(pos, dtype=float).reshape(3)
        stamp = float(stamp)

        if self.prev_pos is None or self.prev_stamp is None:
            self.prev_pos = pos.copy()
            self.prev_stamp = stamp
            self.history.append((stamp, float(pos[0]), float(pos[1])))
            self.raw_omega = 0.0
            self.speed_xy = 0.0
            self.fast_onset_strength = 0.0
            return None

        dt = stamp - self.prev_stamp
        if dt <= 1e-6:
            self.raw_omega = 0.0
            self.speed_xy = 0.0
            self.fast_onset_strength = 0.0
            return None

        self.history.append((stamp, float(pos[0]), float(pos[1])))
        self._prune_history_window(stamp)

        v_xy = (pos[0:2] - self.prev_pos[0:2]) / dt
        pair_speed_xy = float(np.linalg.norm(v_xy))

        self.prev_pos = pos.copy()
        self.prev_stamp = stamp

        min_speed = TURN_HINT_MIN_SPEED if self.min_speed is None else self.min_speed
        omega_poly = self._causal_poly_omega(min_speed)
        if omega_poly is not None:
            omega_meas, speed_xy = omega_poly
        elif pair_speed_xy >= min_speed:
            heading = math.atan2(v_xy[1], v_xy[0])
            if self.prev_heading is None:
                self.prev_heading = heading
                self.raw_omega = 0.0
                self.speed_xy = pair_speed_xy
                self.fast_onset_strength = 0.0
                return None
            d_heading = (heading - self.prev_heading + math.pi) % (
                2.0 * math.pi
            ) - math.pi
            omega_meas = float(np.clip(d_heading / dt, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))
            speed_xy = pair_speed_xy
        else:
            self.prev_heading = None
            self.raw_omega = 0.0
            self.speed_xy = pair_speed_xy
            self.fast_onset_strength = 0.0
            return None

        if pair_speed_xy >= min_speed:
            self.prev_heading = math.atan2(v_xy[1], v_xy[0])

        self.speed_xy = speed_xy
        self.raw_omega = omega_meas
        self.fast_onset_strength = fast_turn_onset_strength(
            self.raw_omega, self.speed_xy, min_speed
        )
        self.omega = (1.0 - TURN_HINT_ALPHA) * self.omega + TURN_HINT_ALPHA * omega_meas

        if abs(self.omega) < TURN_HINT_DEADBAND:
            return 0.0

        return self.omega


def hx(state):
    return state[0:3]


def _rotate_vector(vec, axis, angle):
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    return (
        vec * cos_a
        + np.cross(axis, vec) * sin_a
        + axis * np.dot(axis, vec) * (1.0 - cos_a)
    )


def _integrate_rotating_velocity(pos, vel, omega_vec, dt):
    omega_mag = float(np.linalg.norm(omega_vec))
    if omega_mag < 1e-7:
        return pos + vel * dt, vel.copy()

    axis = omega_vec / omega_mag
    theta = omega_mag * dt
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)

    vel_parallel = axis * np.dot(axis, vel)
    vel_perp = vel - vel_parallel
    axis_cross_vel = np.cross(axis, vel_perp)

    pos_new = (
        pos
        + vel_parallel * dt
        + (sin_t / omega_mag) * vel_perp
        + ((1.0 - cos_t) / omega_mag) * axis_cross_vel
    )
    vel_new = vel_parallel + cos_t * vel_perp + sin_t * axis_cross_vel
    return pos_new, vel_new


def _ct_omega_vector_3d(vel, accel, omega):
    speed_sq = float(np.dot(vel, vel))
    if speed_sq < CT_MIN_SPEED_FOR_AXIS**2:
        return np.zeros(3)

    speed = np.sqrt(speed_sq)
    vel_hat = vel / speed
    accel_perp = accel - np.dot(accel, vel_hat) * vel_hat

    if np.linalg.norm(accel_perp) >= CT_MIN_TURN_ACCEL:
        omega_vec = np.cross(vel, accel_perp) / speed_sq
        omega_mag = float(np.linalg.norm(omega_vec))
        if omega_mag > OMEGA_ABS_MAX:
            omega_vec *= OMEGA_ABS_MAX / omega_mag
        return omega_vec

    if abs(omega) >= OMEGA_STRAIGHT_THRESH:
        return np.array(
            [0.0, 0.0, float(np.clip(omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))]
        )

    return np.zeros(3)


def fx_ct(state, dt):
    x, y, z, vx, vy, vz, ax, ay, az, omega = state
    pos = np.array([x, y, z], dtype=float)
    vel = np.array([vx, vy, vz], dtype=float)
    accel = np.array([ax, ay, az], dtype=float)
    omega = float(np.clip(omega, -OMEGA_ABS_MAX, OMEGA_ABS_MAX))

    omega_vec = _ct_omega_vector_3d(vel, accel, omega)
    omega_mag = float(np.linalg.norm(omega_vec))

    x_new = np.zeros(10)
    if omega_mag < 1e-7:
        pos_new = pos + vel * dt + 0.5 * accel * dt**2
        vel_new = vel + accel * dt
        accel_new = accel.copy()
    else:
        pos_new, vel_new = _integrate_rotating_velocity(pos, vel, omega_vec, dt)
        axis = omega_vec / omega_mag
        accel_new = _rotate_vector(accel, axis, omega_mag * dt)

    x_new[0:3] = pos_new
    x_new[3:6] = vel_new
    x_new[6:9] = accel_new
    x_new[9] = omega
    return x_new


import guidance_config as cfg


class IMMLowPassFilter:
    """
    Smooth IMM output without adding avoidable constant-velocity position lag.

    A plain EMA on position has a steady ramp error for moving targets. When
    motion compensation is enabled, the previous filtered state is first
    propagated with its own velocity/acceleration, then the newest IMM state
    corrects that prediction. This keeps the high-frequency smoothing behavior
    but removes most of the steady-state lag during CV-like motion.
    """

    def __init__(self):
        self.state = None
        self.enabled = getattr(cfg, "IMM_LPF_ENABLED", False)
        self.alpha = getattr(cfg, "IMM_LPF_ALPHA", 0.8)
        self.motion_compensated = getattr(cfg, "IMM_LPF_MOTION_COMPENSATED", True)

    def _default_dt(self):
        loop_hz = float(getattr(cfg, "LOOP_HZ", 0.0))
        if loop_hz > 0.0:
            return 1.0 / loop_hz
        return 0.0

    def _predict_filtered_state(self, dt):
        predicted = self.state.copy()
        if dt <= 0.0 or predicted.shape[0] < 6:
            return predicted

        predicted[0:3] = predicted[0:3] + predicted[3:6] * dt
        if predicted.shape[0] >= 9:
            predicted[0:3] = predicted[0:3] + 0.5 * predicted[6:9] * dt**2
            predicted[3:6] = predicted[3:6] + predicted[6:9] * dt
        return predicted

    def filter(self, new_state, dt=None):
        if not self.enabled:
            return new_state

        new_state = np.asarray(new_state, dtype=float).copy()
        if self.state is None:
            self.state = new_state.copy()
            return self.state

        alpha = float(np.clip(self.alpha, 0.0, 1.0))
        if self.motion_compensated:
            if dt is None:
                dt = self._default_dt()
            base_state = self._predict_filtered_state(float(dt))
        else:
            base_state = self.state

        self.state = base_state + alpha * (new_state - base_state)
        return self.state


class IMMDiagnosticLogger:
    """CSV logger for estimator residuals, mode probabilities, and turn-onset diagnosis."""

    FIELDNAMES = [
        "step",
        "wall_time_s",
        "stamp",
        "dt_raw",
        "dt_meas",
        "dt_actual",
        "motion_label",
        "meas_x",
        "meas_y",
        "meas_z",
        "pred_x",
        "pred_y",
        "pred_z",
        "est_x",
        "est_y",
        "est_z",
        "est_vx",
        "est_vy",
        "est_vz",
        "err_x",
        "err_y",
        "err_z",
        "err_norm",
        "innov_x",
        "innov_y",
        "innov_z",
        "innov_norm",
        "innov_along",
        "innov_cross",
        "innov_down",
        "jump_x",
        "jump_y",
        "jump_z",
        "jump_norm",
        "jump_along",
        "jump_cross",
        "jump_down",
        "mu_cv",
        "mu_ct",
        "mu_ca",
        "mu_cv_xy",
        "mu_ct_xy",
        "mu_ca_xy",
        "mu_cv_z",
        "mu_ca_z",
        "mu_cvxy_cvz",
        "mu_cvxy_caz",
        "mu_ctxy_cvz",
        "mu_ctxy_caz",
        "mu_caxy_cvz",
        "mu_caxy_caz",
        "omega_hint",
        "turn_hint_strength",
        "raw_omega",
        "raw_turn_strength",
        "raw_speed_xy",
        "omega_ct_raw",
        "omega_ct",
        "omega_eff",
        "ct_speed_xy",
        "likelihood_cv",
        "likelihood_ct",
        "likelihood_ca",
        "likelihood_cv_xy",
        "likelihood_ct_xy",
        "likelihood_ca_xy",
        "likelihood_cv_z",
        "likelihood_ca_z",
        "likelihood_cvxy_cvz",
        "likelihood_cvxy_caz",
        "likelihood_ctxy_cvz",
        "likelihood_ctxy_caz",
        "likelihood_caxy_cvz",
        "likelihood_caxy_caz",
        "apparent_delay_s",
    ]

    def __init__(self, log_dir="logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"imm_diagnostics_{stamp}.csv"
        self.file = self.path.open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.rows_since_flush = 0

    def write(self, row):
        self.writer.writerow(row)
        self.rows_since_flush += 1
        if self.rows_since_flush >= 25:
            self.file.flush()
            self.rows_since_flush = 0

    def close(self):
        if not self.file.closed:
            self.file.flush()
        self.file.close()


VERTICAL_STATE_INDICES = (2, 5, 8)


def snapshot_imm_vertical_state(imm):
    return [(filt.x.copy(), filt.P.copy()) for filt in imm.filters]


def restore_imm_vertical_state(imm, snapshots):
    for filt, (x_ref, p_ref) in zip(imm.filters, snapshots):
        state_len = int(filt.x.shape[0])
        idx = [i for i in VERTICAL_STATE_INDICES if i < state_len]
        if not idx:
            continue

        filt.x[idx] = x_ref[idx]
        filt.P[idx, :] = p_ref[idx, :]
        filt.P[:, idx] = p_ref[:, idx]

    imm._compute_state_estimate()


def update_imm_preserving_vertical(imm, z_meas):
    snapshots = snapshot_imm_vertical_state(imm)
    z_update = np.asarray(z_meas, dtype=float).reshape(3).copy()
    z_update[2] = float(imm.x[2])
    imm.update(z_update)
    restore_imm_vertical_state(imm, snapshots)


class OOSM_IMM_Tracker:
    """
    Out-of-Sequence Measurement (OOSM) Tracker for the IMM Estimator.
    Solves the Delayed Measurement problem by rewinding state history, applying
    late updates, and re-propagating to the current time.

    WARNING (2026-07-24 review): NOT used by the active pipeline
    (simple_guided_follow), only by retired runners. process_delayed_measurement
    calls imm.update() with no preceding predict, so the update consumes a
    stale cbar (filterpy computes mu = cbar * likelihood) -- it violates the
    predict-before-update invariant the active pipeline relies on. Fix that
    before reviving this class.
    """

    def __init__(self, imm, dt, max_history_size=50):
        self.imm = imm
        self.dt = dt
        self.history = []  # List of tuples: (timestamp, mu, filters_x, filters_P, imm_x)
        self.current_time = 0.0
        self.max_history_size = max_history_size

    def step_forward(self):
        """Advances the internal clock and saves prediction states."""
        self.imm.predict()
        stabilize_omega_states(self.imm)
        self.current_time += self.dt
        self._record_state()

    def _record_state(self):
        backup_xs = [kf.x.copy() for kf in self.imm.filters]
        backup_Ps = [kf.P.copy() for kf in self.imm.filters]
        self.history.append(
            (
                self.current_time,
                self.imm.mu.copy(),
                backup_xs,
                backup_Ps,
                self.imm.x.copy(),
            )
        )
        if len(self.history) > self.max_history_size:
            self.history.pop(0)

    def process_delayed_measurement(self, z_meas, t_meas, freeze_z_update=False):
        """
        Rewinds the filter to match t_meas, applies the measurement, and repropagates
        forward to the present time.
        """
        if not self.history:
            # If no history, just update directly
            if freeze_z_update:
                update_imm_preserving_vertical(self.imm, z_meas)
            else:
                self.imm.update(z_meas)
            return

        # 1. Find the closest historical time index
        idx_to_restore = 0
        min_diff = float("inf")
        for i, (t_hist, _, _, _, _) in enumerate(self.history):
            diff = abs(t_hist - t_meas)
            if diff < min_diff:
                min_diff = diff
                idx_to_restore = i

        t_hist, mu, b_xs, b_Ps, _ = self.history[idx_to_restore]

        # 2. Restore IMM to that past state
        for i, kf in enumerate(self.imm.filters):
            kf.x = b_xs[i].copy()
            kf.P = b_Ps[i].copy()
        self.imm.mu = mu.copy()

        # 3. Update the filter using the delayed measurement
        if freeze_z_update:
            update_imm_preserving_vertical(self.imm, z_meas)
        else:
            self.imm.update(z_meas)
        stabilize_omega_states(self.imm)

        # 4. Re-propagate forward to current time to anchor changes to reality
        # Calculate how many predict steps are needed to reach self.current_time
        steps_to_catch_up = len(self.history) - 1 - idx_to_restore

        for _ in range(steps_to_catch_up):
            self.imm.predict()
            stabilize_omega_states(self.imm)
            # In a full tracking system, we would re-apply any other valid updates that happened here.

        # 5. Overwrite the latest recorded state with the re-propagated reality
        self.history[-1] = (
            self.current_time,
            self.imm.mu.copy(),
            [kf.x.copy() for kf in self.imm.filters],
            [kf.P.copy() for kf in self.imm.filters],
            self.imm.x.copy(),
        )


def predict_n_steps_ahead(imm, dt, steps):
    """
    Predicts the target's state `steps` ticks into the future.
    It temporarily projects the current IMM state forward without updating it with measurements.

    Returns:
        Predicted state vector X (10,1) at `steps * dt` in the future.
    """
    # 1. Back up current true IMM states to avoid permanently altering the filter
    backup_xs = [kf.x.copy() for kf in imm.filters]
    backup_Ps = [kf.P.copy() for kf in imm.filters]
    backup_mu = imm.mu.copy()
    backup_x = imm.x.copy()
    backup_P = imm.P.copy()

    # 2. Project forward N steps
    predicted_x = np.zeros_like(imm.x)
    for _ in range(steps):
        refresh_filter_dt(imm, dt)
        imm.predict()
        stabilize_omega_states(imm)
        predicted_x = imm.x.copy()
        predicted_x[9] = get_effective_turn_rate(imm)

    # 3. Restore the true state so the next real update() cycle is unaffected
    for i, kf in enumerate(imm.filters):
        kf.x = backup_xs[i]
        kf.P = backup_Ps[i]
    imm.mu = backup_mu
    imm.x = backup_x
    imm.P = backup_P

    return predicted_x


def refresh_filter_dt(imm, dt):
    """Dynamically updates dt-dependent matrices (F, Q) in the IMM sub-filters."""
    refresh_filter_runtime_params(imm)
    if _is_product_imm(imm):
        for kf, (_, h_mode, z_mode) in zip(imm.filters, _get_mode_specs(imm)):
            if h_mode == H_MODE_CT:
                if hasattr(kf, "_dt"):
                    kf._dt = dt
                else:
                    kf.dt = dt
            else:
                kf.F = f_product_matrix(dt, h_mode, z_mode)

            kf.Q = q_product_10d(dt, h_mode, z_mode)
        return

    cv = imm.filters[0]
    ct = imm.filters[1]
    ca = imm.filters[2]

    cv.F = np.eye(10)
    cv.F[0, 3] = dt
    cv.F[1, 4] = dt
    cv.F[2, 5] = dt
    cv.Q = q_cv_10d(dt, SIGMA_A_CV)

    if hasattr(ct, "_dt"):
        ct._dt = dt
    else:
        ct.dt = dt
    ct.Q = q_ct_10d(dt, SIGMA_J_CT_3D, SIGMA_OMEGA_DOT)

    ca.F = np.eye(10)
    ca.F[0, 3] = dt
    ca.F[1, 4] = dt
    ca.F[2, 5] = dt
    ca.F[3, 6] = dt
    ca.F[4, 7] = dt
    ca.F[5, 8] = dt
    ca.F[0, 6] = 0.5 * dt**2
    ca.F[1, 7] = 0.5 * dt**2
    ca.F[2, 8] = 0.5 * dt**2
    ca.Q = q_ca_10d(dt, SIGMA_J_CA)


def update_ct_filter_dynamics(imm, dt):
    """Compatibility wrapper for older callers."""
    refresh_filter_dt(imm, dt)
    stabilize_omega_states(imm)


def _mix_imm_initial_conditions_once(imm):
    """
    Apply the IMM model-mixing step once for the current measurement interval.

    FilterPy's IMMEstimator.predict() mixes model states every time it is called.
    We substep the dynamics for numerical stability, so calling imm.predict()
    per substep would also mix product modes per substep. That makes CTxy lose its
    separate turn state before the next measurement arrives.
    """
    imm._compute_mixing_probabilities()

    mixed_xs = []
    mixed_Ps = []
    for weights in imm.omega.T:
        x = np.zeros(imm.x.shape)
        for kf, weight in zip(imm.filters, weights):
            x += kf.x * weight
        mixed_xs.append(x)

        P = np.zeros(imm.P.shape)
        for kf, weight in zip(imm.filters, weights):
            y = kf.x - x
            P += weight * (np.outer(y, y) + kf.P)
        mixed_Ps.append(P)

    for kf, x, P in zip(imm.filters, mixed_xs, mixed_Ps):
        kf.x = x.copy()
        kf.P = P.copy()

    imm.mu = _distribution_or_default(imm.cbar, np.ones(len(imm.filters)))


def predict_imm_over_dt(imm, dt, max_substep=None):
    """Predict over the full measurement interval using stable smaller substeps."""
    refresh_filter_runtime_params(imm)
    if max_substep is None:
        max_substep = PREDICT_MAX_SUBSTEP
    remaining = max(0.0, float(dt))
    if remaining <= 1e-9:
        stabilize_omega_states(imm)
        return

    _mix_imm_initial_conditions_once(imm)

    while remaining > 1e-9:
        step_dt = min(max_substep, remaining)
        refresh_filter_dt(imm, step_dt)
        stabilize_omega_states(imm)
        for kf in imm.filters:
            kf.predict()
        stabilize_omega_states(imm)
        remaining -= step_dt

    stabilize_omega_states(imm)
    imm._compute_state_estimate()
    imm.x[9] = get_effective_turn_rate(imm)
    imm.x_prior = imm.x.copy()
    imm.P_prior = imm.P.copy()


def _build_product_markov_matrix():
    M = np.zeros((len(PRODUCT_MODE_SPECS), len(PRODUCT_MODE_SPECS)))
    for i, (_, h_from, z_from) in enumerate(PRODUCT_MODE_SPECS):
        for j, (_, h_to, z_to) in enumerate(PRODUCT_MODE_SPECS):
            M[i, j] = (
                H_MODE_TRANSITION[H_MODE_INDEX[h_from], H_MODE_INDEX[h_to]]
                * Z_MODE_TRANSITION[Z_MODE_INDEX[z_from], Z_MODE_INDEX[z_to]]
            )
    return M


def setup_imm_filter(dt):
    refresh_dynamic_filter_params()

    # Measurement matrix H (Mapping 10D state to 3D [x, y, z] sensor measurement)
    H = np.zeros((3, 10))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    H[2, 2] = 1.0

    filters = []
    mu = []
    for _, h_mode, z_mode in PRODUCT_MODE_SPECS:
        if h_mode == H_MODE_CT:
            sigmas = MerweScaledSigmaPoints(10, alpha=0.1, beta=2.0, kappa=0)
            kf = UnscentedKalmanFilter(
                dim_x=10,
                dim_z=3,
                dt=dt,
                fx=_make_ctxy_fx(z_mode),
                hx=hx,
                points=sigmas,
            )
        else:
            kf = KalmanFilter(dim_x=10, dim_z=3)
            kf.F = f_product_matrix(dt, h_mode, z_mode)
            kf.H = H

        kf.Q = q_product_10d(dt, h_mode, z_mode)
        kf.x = np.zeros(10)  # 1D Array is safer for mixed UKF/KF filters
        kf.P = np.eye(10) * 50.0
        kf.R = measurement_R()
        kf.P[9, 9] = (
            CT_OMEGA_INITIAL_STD**2 if h_mode == H_MODE_CT else UNUSED_STATE_VARIANCE
        )

        filters.append(kf)
        mu.append(
            H_MODE_INITIAL[H_MODE_INDEX[h_mode]] * Z_MODE_INITIAL[Z_MODE_INDEX[z_mode]]
        )

    M = _build_product_markov_matrix()

    imm = IMMEstimator(filters, mu, M)
    imm.mode_specs = PRODUCT_MODE_SPECS
    imm.mode_names = PRODUCT_MODE_NAMES
    imm.horizontal_modes = H_MODE_ORDER
    imm.vertical_modes = Z_MODE_ORDER
    stabilize_omega_states(imm)
    return imm


# ==========================================
# Main Loop Example for ArduPilot SITL
# ==========================================
if __name__ == "__main__":
    import time

    import mavlink_utils
    from pymavlink import mavutil

    from guidance_gui import GuidanceGUI, push_snapshot

    print(
        "Connecting to Target MAVLink via LOCAL_POSITION_NED / GLOBAL_POSITION_INT..."
    )
    # Adjust this string if your target is on a different port (e.g. 14600)
    target_conn_str = "udpin:localhost:14600"
    target_conn = mavutil.mavlink_connection(target_conn_str)
    target_conn.wait_heartbeat()
    print("Heartbeat received!")

    # Using the MavStateReader from your utils
    # For a target plane, it might broadcast GLOBAL_POSITION_INT.
    # We will initialize it with a dummy home at first, or we can use local position
    target_reader = mavlink_utils.MavStateReader(
        target_conn,
        ["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
        lambda msg: (
            mavlink_utils.parse_local_ned(msg)
            if msg.get_type() == "LOCAL_POSITION_NED"
            else mavlink_utils.parse_global_int(msg, 0.0, 0.0, 0.0)
        ),  # Fallback wrapper
    )
    target_reader.start()

    dt = 0.1  # 10Hz calculation loop
    imm = setup_imm_filter(dt)

    # --- Launch the shared guidance GUI; its Estimator tab renders the error /
    #     IMM-probability / residual-norm plots this script used to draw itself. ---
    gui = GuidanceGUI(param_module="guidance_config", refresh_hz=15)
    gui.start()

    print("Running IMM Simulation...")
    step = 0
    lpf = IMMLowPassFilter()
    diagnostic_logger = IMMDiagnosticLogger()
    console_summary_interval = 50
    print(f"Writing IMM diagnostics to: {diagnostic_logger.path}")

    last_filter_time = time.monotonic()
    last_stamp = None
    dt_actual_prev = dt
    turn_rate_estimator = HeadingTurnRateEstimator()

    import atexit

    atexit.register(diagnostic_logger.close)

    print("Streaming estimator telemetry to the GUI. Ctrl-C or close the window to stop.")
    while gui._running:
        # Get positions with timestamp to reject stale data
        pos, vel, stamp = target_reader.get_with_stamp()

        if pos is None:
            time.sleep(0.01)
            continue

        if stamp is not None and last_stamp is not None and stamp == last_stamp:
            # Skip update if no new message arrived
            time.sleep(0.005)
            continue

        now = time.monotonic()
        if stamp is not None and last_stamp is not None:
            dt_raw = stamp - last_stamp
        elif step == 0:
            dt_raw = dt
        else:
            dt_raw = now - last_filter_time

        if dt_raw <= 0.0:
            time.sleep(0.005)
            continue

        if stamp is not None:
            last_stamp = stamp
        last_filter_time = now

        dt_meas = clamp_filter_dt(dt_raw)
        dt_actual = (
            DT_SMOOTHING_KEEP_PREV * dt_actual_prev
            + (1.0 - DT_SMOOTHING_KEEP_PREV) * dt_meas
        )
        dt_actual_prev = dt_actual

        # Z represents down, so just pass the 3D position directly
        z_meas = np.array([pos[0], pos[1], pos[2]])
        omega_hint = turn_rate_estimator.update(
            z_meas, stamp if stamp is not None else now
        )

        if step == 0:
            # Force first update to true to prevent fly-in lines
            for f in imm.filters:
                f.x[0:3] = z_meas
            imm.x = imm.filters[0].x.copy()

        # 2. IMM Predict and Update
        raw_turn_strength = apply_fast_turn_onset_hint(
            imm,
            turn_rate_estimator.raw_omega,
            turn_rate_estimator.speed_xy,
        )
        turn_strength = apply_turn_rate_hint(imm, omega_hint)
        predict_imm_over_dt(imm, dt_actual)
        x_pred = imm.x.copy()
        innovation = z_meas - x_pred[0:3]
        imm.update(z_meas)
        turn_strength = max(turn_strength, apply_turn_rate_hint(imm, omega_hint))
        stabilize_omega_states(imm)
        x_upd = imm.x.copy()
        update_jump = x_upd[0:3] - x_pred[0:3]

        # 3. Extract the predicted state for your FRPN Guidance Law
        imm_state_smoothed = lpf.filter(imm.x, dt_actual)

        target_pos = imm_state_smoothed[0:3].flatten()
        target_vel = imm_state_smoothed[3:6].flatten()
        err_vec = target_pos - z_meas
        err_norm = float(np.linalg.norm(err_vec))
        innov_norm = float(np.linalg.norm(innovation))
        jump_norm = float(np.linalg.norm(update_jump))

        v_horiz = np.array([target_vel[0], target_vel[1], 0.0])
        v_horiz_norm = float(np.linalg.norm(v_horiz))
        if v_horiz_norm > 1e-6:
            h = v_horiz / v_horiz_norm
            r = np.array([-h[1], h[0], 0.0])
        else:
            h = np.array([1.0, 0.0, 0.0])
            r = np.array([0.0, 1.0, 0.0])

        innov_along = float(np.dot(innovation, h))
        innov_cross = float(np.dot(innovation, r))
        innov_down = float(innovation[2])
        jump_along = float(np.dot(update_jump, h))
        jump_cross = float(np.dot(update_jump, r))
        jump_down = float(update_jump[2])

        mode_probs = aggregate_mode_probabilities(imm)
        mode_likelihoods = aggregate_mode_likelihoods(imm)
        mode_mu_fields = {
            f"mu_{suffix}": float(imm.mu[idx]) if idx < len(imm.mu) else ""
            for idx, suffix in enumerate(PRODUCT_MODE_FIELD_SUFFIXES)
        }
        mode_likelihood_fields = {
            f"likelihood_{suffix}": float(imm.likelihood[idx])
            if idx < len(imm.likelihood)
            else ""
            for idx, suffix in enumerate(PRODUCT_MODE_FIELD_SUFFIXES)
        }

        omega_ct_raw = get_weighted_ct_omega(imm)
        omega_eff = get_effective_turn_rate(imm)
        omega_ct = omega_eff
        ct_speed_xy = get_weighted_ct_speed_xy(imm)

        if abs(omega_eff) > OMEGA_STRAIGHT_THRESH:
            motion_label = "turning"
        else:
            motion_label = "quasi-straight"

        speed_sq = float(np.dot(target_vel, target_vel))
        apparent_delay_s = -float(np.dot(err_vec, target_vel)) / (speed_sq + 1e-6)

        diagnostic_logger.write(
            {
                "step": step,
                "wall_time_s": now,
                "stamp": "" if stamp is None else stamp,
                "dt_raw": dt_raw,
                "dt_meas": dt_meas,
                "dt_actual": dt_actual,
                "motion_label": motion_label,
                "meas_x": z_meas[0],
                "meas_y": z_meas[1],
                "meas_z": z_meas[2],
                "pred_x": x_pred[0],
                "pred_y": x_pred[1],
                "pred_z": x_pred[2],
                "est_x": target_pos[0],
                "est_y": target_pos[1],
                "est_z": target_pos[2],
                "est_vx": target_vel[0],
                "est_vy": target_vel[1],
                "est_vz": target_vel[2],
                "err_x": err_vec[0],
                "err_y": err_vec[1],
                "err_z": err_vec[2],
                "err_norm": err_norm,
                "innov_x": innovation[0],
                "innov_y": innovation[1],
                "innov_z": innovation[2],
                "innov_norm": innov_norm,
                "innov_along": innov_along,
                "innov_cross": innov_cross,
                "innov_down": innov_down,
                "jump_x": update_jump[0],
                "jump_y": update_jump[1],
                "jump_z": update_jump[2],
                "jump_norm": jump_norm,
                "jump_along": jump_along,
                "jump_cross": jump_cross,
                "jump_down": jump_down,
                "mu_cv": mode_probs["cv_xy"],
                "mu_ct": mode_probs["ct_xy"],
                "mu_ca": mode_probs["ca_xy"],
                "mu_cv_xy": mode_probs["cv_xy"],
                "mu_ct_xy": mode_probs["ct_xy"],
                "mu_ca_xy": mode_probs["ca_xy"],
                "mu_cv_z": mode_probs["cv_z"],
                "mu_ca_z": mode_probs["ca_z"],
                **mode_mu_fields,
                "omega_hint": "" if omega_hint is None else omega_hint,
                "turn_hint_strength": turn_strength,
                "raw_omega": turn_rate_estimator.raw_omega,
                "raw_turn_strength": raw_turn_strength,
                "raw_speed_xy": turn_rate_estimator.speed_xy,
                "omega_ct_raw": omega_ct_raw,
                "omega_ct": omega_ct,
                "omega_eff": omega_eff,
                "ct_speed_xy": ct_speed_xy,
                "likelihood_cv": mode_likelihoods["cv_xy"],
                "likelihood_ct": mode_likelihoods["ct_xy"],
                "likelihood_ca": mode_likelihoods["ca_xy"],
                "likelihood_cv_xy": mode_likelihoods["cv_xy"],
                "likelihood_ct_xy": mode_likelihoods["ct_xy"],
                "likelihood_ca_xy": mode_likelihoods["ca_xy"],
                "likelihood_cv_z": mode_likelihoods["cv_z"],
                "likelihood_ca_z": mode_likelihoods["ca_z"],
                **mode_likelihood_fields,
                "apparent_delay_s": apparent_delay_s,
            }
        )

        # Feed this step to the shared GUI. No pursuer here, so no range/t_go/FSM
        # is supplied — the Estimator tab (error / IMM probability + omega /
        # residual norms) is fully populated; the Approach cards stay "--".
        push_snapshot(
            pursuer_pos=np.zeros(3),
            target_pos=z_meas,
            mode_probs=mode_probs,
            target_est=target_pos,
            diag={
                "omega": omega_ct,
                "innov_norm": innov_norm,
                "jump_norm": jump_norm,
                "err_norm": err_norm,
            },
        )

        # Format output strings
        errors = np.sqrt(np.diag(imm.P))
        pos_err, vel_err = errors[0:3], errors[3:6]

        pos_str = f"[{target_pos[0]:.1f}(±{pos_err[0]:.1f}), {target_pos[1]:.1f}(±{pos_err[1]:.1f}), {target_pos[2]:.1f}(±{pos_err[2]:.1f})]"

        if step % console_summary_interval == 0:
            print(
                f"Step {step:5d} | raw_dt={dt_raw:.3f}s | dt={dt_actual:.3f}s | err={err_norm:.2f}m | "
                f"innov={innov_norm:.2f}m | jump={jump_norm:.2f}m | "
                f"mu_xy=[{mode_probs['cv_xy']:.2f}, {mode_probs['ct_xy']:.2f}, {mode_probs['ca_xy']:.2f}] "
                f"mu_z=[{mode_probs['cv_z']:.2f}, {mode_probs['ca_z']:.2f}] | "
                f"omega={omega_ct:.3f} hint={0.0 if omega_hint is None else omega_hint:.3f} raw={turn_rate_estimator.raw_omega:.3f} | "
                f"{motion_label} | Pos: {pos_str}"
            )

        step += 1

    gui.stop()
    diagnostic_logger.close()
    print("GUI closed; estimator stopped.")
