"""Evaluate the 25minfirst session: measurement quality, VSL outputs, and the
rays-vs-position architecture question on real close-range data.

The mono log gives us, per triangulation event, exactly what the estimator
would consume in flight: two boresight-corrected rays with timestamps. Truth
comes from the target's own dataflash logs. Everything is scored the way
guidance consumes it -- as a prediction ``p + v*tau + 0.5*a*tau^2`` at
tau = 0..1 s -- pooled over the user's hand-picked good windows.

Architectures compared on identical frames:
  rays-IMM   six-mode product IMM updated directly on the four angles
  tri-IMM    per-frame ML triangulation -> IMM position update, CRLB R
  tri-iso    same, but isotropic R of equal trace (the naive port)
  CV-fit     sliding constant-velocity least squares on skew-gated fixes
  hold       last skew-gated fix, zero velocity
plus VSL's own outputs (Calc instant, LPF) scored as delivered.
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

import filterwndr as fw
import stereo_config as scfg
import stereo_geometry as sg
from mono_ingest import GOOD_WINDOWS, MonoLog, read_mono, truth_enu_at, truth_tracks
from stereo_estimator import StereoTracker
from vsl_predict import Cfg, predict_state
from vsl_run import enu_to_ned

HORIZONS = (0.0, 0.25, 0.5, 0.75, 1.0)
SKEW_GATE_M = 10.0


# ---------------------------------------------------------------------------
#  data assembly
# ---------------------------------------------------------------------------

class Session:
    """Log rows restricted to the good windows, with truth and geometry.

    The entered camera coordinates are typed defaults (``DEFAULT_GPS1/2``) and
    put the site ~81 m below and ~13 m sideways of where GPS truth says the
    target flew, so the raw truth-vs-output error is dominated by a constant
    datum offset that has nothing to do with the sensor. ``datum_correct=True``
    fits one constant offset per flight (median of truth-minus-Calc over the
    good windows) and moves truth into the site frame before scoring.
    """

    def __init__(self, log: MonoLog, datum_correct: bool = True):
        self.log = log
        self.frame = log.frame
        self.tracks = truth_tracks()
        self.cam2_enu = np.array(self.frame.to_enu(*log.cam2_lla))
        self.masks = [log.window_mask(a, b) for a, b in GOOD_WINDOWS]
        self.datum_enu: List[np.ndarray] = [np.zeros(3) for _ in self.tracks]
        if datum_correct:
            self._fit_datum()
        # dense truth grid: geodetic conversion once, then pure np.interp
        grids, enus = [], []
        for tr, d in zip(self.tracks, self.datum_enu):
            g = np.arange(tr.span[0], tr.span[1], 0.02)
            grids.append(g)
            enus.append(truth_enu_at([tr], self.frame, g) - d)
        self._grid = np.concatenate(grids)
        self._genu = np.vstack(enus)
        self._gned = enu_to_ned(self._genu)

    def truth_enu(self, t: np.ndarray) -> np.ndarray:
        t = np.atleast_1d(np.asarray(t, dtype=float))
        out = np.stack([np.interp(t, self._grid, self._genu[:, k])
                        for k in range(3)], axis=-1)
        covered = np.zeros(t.size, bool)
        for tr in self.tracks:
            covered |= tr.covers(t)
        out[~covered] = np.nan
        return out

    def _fit_datum(self) -> None:
        c = self.log.cols
        pooled = np.zeros(len(self.log), bool)
        for m in self.masks:
            pooled |= m
        t = c["loc_ts"][pooled]
        calc = np.stack([c["calc_e"][pooled], c["calc_n"][pooled],
                         c["calc_u"][pooled]], -1)
        for k, tr in enumerate(self.tracks):
            m = tr.covers(t) & np.isfinite(calc[:, 0])
            if m.sum() < 50:
                continue
            truth = truth_enu_at([tr], self.frame, t[m])
            self.datum_enu[k] = np.median(truth - calc[m], axis=0)

    def truth_ned(self, t: np.ndarray) -> np.ndarray:
        t = np.atleast_1d(np.asarray(t, dtype=float))
        out = np.stack([np.interp(t, self._grid, self._gned[:, k])
                        for k in range(3)], axis=-1)
        covered = np.zeros(t.size, bool)
        for tr in self.tracks:
            covered |= tr.covers(t)
        out[~covered] = np.nan
        return out

    def rows(self, mask: np.ndarray) -> Dict[str, np.ndarray]:
        c = self.log.cols
        order = np.argsort(c["loc_ts"][mask], kind="stable")
        return {k: v[mask][order] for k, v in c.items()}


def ray_enu(head_deg: np.ndarray, pitch_deg: np.ndarray) -> np.ndarray:
    h = np.radians(head_deg)
    p = np.radians(pitch_deg)
    return np.stack([np.sin(h) * np.cos(p), np.cos(h) * np.cos(p),
                     np.sin(p)], axis=-1)


def make_cameras(sess: Session, sig1: float, sig2: float) -> List[sg.Camera]:
    cams = []
    for name, pos_enu, sig in (("CAM1", np.zeros(3), sig1),
                               ("CAM2", sess.cam2_enu, sig2)):
        cams.append(sg.Camera(
            name, enu_to_ned(pos_enu), 0.0, 0.0, 0.0,
            sigma_yaw_deg=sig, sigma_pitch_deg=sig,
            fov_yaw_deg=359.0, fov_pitch_deg=179.0, max_range_m=20000.0))
    return cams


def precompute(sess: Session, mask: np.ndarray, cams: List[sg.Camera]) -> Dict:
    """Per-row detections, ML fix, skew and CRLB covariance."""
    r = sess.rows(mask)
    n = r["loc_ts"].size
    out = {
        "t": r["loc_ts"],
        "dets": [None] * n,
        "fix": np.full((n, 3), np.nan),
        "skew": np.full(n, np.nan),
        "cov": [None] * n,
        "calc": enu_to_ned(np.stack([r["calc_e"], r["calc_n"], r["calc_u"]], -1)),
        "lpf": enu_to_ned(np.stack([r["lpf_e"], r["lpf_n"], r["lpf_u"]], -1)),
        "raw": enu_to_ned(np.stack([r["raw_e"], r["raw_n"], r["raw_u"]], -1)),
    }
    for i in range(n):
        dets = []
        for ci, (h, p) in enumerate((("c1_head", "c1_pitch"),
                                     ("c2_head", "c2_pitch"))):
            if math.isfinite(r[h][i]) and math.isfinite(r[p][i]):
                dets.append(sg.Detection(ci, math.radians(r[h][i]),
                                         math.radians(r[p][i]),
                                         float(r["loc_ts"][i])))
        out["dets"][i] = dets
        if len(dets) < 2:
            continue
        seed, info = sg.triangulate_midpoint(cams[0], dets[0], cams[1], dets[1])
        if not np.all(np.isfinite(seed)):
            continue
        fix, _ = sg.triangulate_ml(cams, dets, seed)
        if not np.all(np.isfinite(fix)):
            continue
        cov, cinfo = sg.triangulation_covariance(cams, dets, fix)
        if cinfo.get("rank_deficient"):
            continue
        out["fix"][i] = fix
        out["skew"][i] = float(info.get("skew_m", np.nan))
        out["cov"][i] = np.asarray(cov, float)
    return out


# ---------------------------------------------------------------------------
#  scoring
# ---------------------------------------------------------------------------

class Score:
    """Pooled prediction errors by horizon, total and along-LOS."""

    def __init__(self, label: str):
        self.label = label
        self.tot: Dict[float, List[float]] = {h: [] for h in HORIZONS}
        self.alo: Dict[float, List[float]] = {h: [] for h in HORIZONS}

    def add(self, sess: Session, t: float, x: np.ndarray) -> None:
        for tau in HORIZONS:
            tn = sess.truth_ned(np.array([t + tau]))[0]
            if not np.all(np.isfinite(tn)):
                continue
            nrm = float(np.linalg.norm(tn))
            if nrm < 1.0:
                continue
            pe = predict_state(x, tau)
            a, _ = sg.los_decompose(pe - tn, tn / nrm)
            self.tot[tau].append(float(np.linalg.norm(pe - tn)))
            self.alo[tau].append(abs(float(a)))

    def med(self, tau: float, depth: bool = False) -> float:
        v = (self.alo if depth else self.tot)[tau]
        return float(np.median(v)) if v else float("nan")

    def line(self, depth: bool = False) -> str:
        parts = [f"{self.med(h, depth):6.2f}" for h in HORIZONS]
        n = len(self.tot[0.0])
        return f"  {self.label:24s} n={n:5d}  " + "  ".join(parts)


def state_from_fix(fix: np.ndarray, vel: Optional[np.ndarray] = None) -> np.ndarray:
    x = np.zeros(9)
    x[0:3] = fix
    if vel is not None:
        x[3:6] = vel
    return x


# ---------------------------------------------------------------------------
#  architectures
# ---------------------------------------------------------------------------

def run_rays(sess: Session, pre: Dict, cams: List[sg.Camera], sc: Score,
             min_dt: float = 0.0) -> None:
    cfg = Cfg(scfg)
    tracker = StereoTracker(cams, cfg=cfg, nominal_dt=0.14)
    last_fed = None
    for i, t in enumerate(pre["t"]):
        if min_dt and last_fed is not None and (t - last_fed) < min_dt:
            continue
        dets = pre["dets"][i]
        if not dets:
            continue
        last_fed = t
        snap = tracker.process(dets, float(t))
        if snap["tracking"]:
            sc.add(sess, float(t), np.asarray(snap["x"], float))


def run_tri(sess: Session, pre: Dict, sc: Score, iso: bool = False,
            r_scale: float = 1.0, skew_gate: float = 60.0,
            min_dt: float = 0.0, nominal_dt: float = 0.14) -> None:
    imm = None
    last_stamp = None
    last_fed = None
    for i, t in enumerate(pre["t"]):
        fix, cov, skew = pre["fix"][i], pre["cov"][i], pre["skew"][i]
        if cov is None or not np.all(np.isfinite(fix)) or skew > skew_gate:
            continue
        if min_dt and last_fed is not None and (t - last_fed) < min_dt:
            continue
        last_fed = t
        R = cov * r_scale
        if iso:
            R = np.eye(3) * (np.trace(R) / 3.0)
        R = R + np.eye(3) * 1e-6
        if imm is None:
            imm = fw.setup_imm_filter(nominal_dt)
            for kf in imm.filters:
                kf.x[0:3] = fix
                kf.P[0:3, 0:3] = R * 4.0
            imm.x = np.array(imm.filters[0].x, dtype=float).copy()
            last_stamp = t
            continue
        dt = float(np.clip(t - last_stamp, 1e-3, 2.0))
        try:
            fw.predict_imm_over_dt(imm, dt, max_substep=0.1)
        except np.linalg.LinAlgError:
            imm = None
            continue
        last_stamp = t
        for kf in imm.filters:
            kf.R = R.copy()
        try:
            imm.update(np.asarray(fix, float).reshape(3))
        except np.linalg.LinAlgError:
            imm = None
            continue
        sc.add(sess, float(t), np.asarray(imm.x, float).reshape(-1))


def run_cv(sess: Session, pre: Dict, sc: Score, fit_s: float,
           skew_gate: float = SKEW_GATE_M, min_pts: int = 5,
           source: str = "fix") -> None:
    hist: List[Tuple[float, np.ndarray]] = []
    for i, t in enumerate(pre["t"]):
        fix = pre[source][i] if source != "fix" else pre["fix"][i]
        skew = pre["skew"][i]
        if not np.all(np.isfinite(fix)) or (source == "fix" and skew > skew_gate):
            continue
        hist.append((float(t), fix))
        while hist and (t - hist[0][0]) > fit_s:
            hist.pop(0)
        if fit_s <= 0.0:
            sc.add(sess, float(t), state_from_fix(fix))
            continue
        if len(hist) < min_pts:
            continue
        ts = np.array([h[0] for h in hist]) - t
        ps = np.stack([h[1] for h in hist])
        A = np.stack([np.ones_like(ts), ts], axis=1)
        coef, *_ = np.linalg.lstsq(A, ps, rcond=None)
        sc.add(sess, float(t), state_from_fix(coef[0], coef[1]))


# ---------------------------------------------------------------------------
#  characterisation
# ---------------------------------------------------------------------------

def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def mad_sigma(x: np.ndarray) -> float:
    x = x[np.isfinite(x)]
    if x.size < 8:
        return float("nan")
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def lag_corr(t: np.ndarray, x: np.ndarray, lag: float, tol: float) -> float:
    """Correlation of pairs (x_i, x_j) with t_j - t_i in [lag-tol, lag+tol]."""
    ok = np.isfinite(x)
    t, x = t[ok], x[ok] - np.mean(x[ok])
    j0 = np.searchsorted(t, t + lag - tol)
    j1 = np.searchsorted(t, t + lag + tol)
    a, b = [], []
    for i in range(t.size):
        for j in range(j0[i], min(j1[i], t.size)):
            a.append(x[i])
            b.append(x[j])
    if len(a) < 50:
        return float("nan")
    a, b = np.array(a), np.array(b)
    den = math.sqrt(float(np.dot(a, a)) * float(np.dot(b, b)))
    return float(np.dot(a, b) / den) if den > 0 else float("nan")


def characterise(sess: Session, mask: np.ndarray) -> Dict[str, Dict[str, float]]:
    r = sess.rows(mask)
    out = {}
    for cam, pos in (("c1", np.zeros(3)), ("c2", sess.cam2_enu)):
        t = r[f"{cam}_ts"]
        ok = np.isfinite(t) & np.isfinite(r[f"{cam}_head"])
        enu = sess.truth_enu(t[ok]) - pos
        rng = np.linalg.norm(enu, axis=-1)
        head_t = np.degrees(np.arctan2(enu[:, 0], enu[:, 1]))
        pitch_t = np.degrees(np.arcsin(np.clip(enu[:, 2] / np.maximum(rng, 1e-6),
                                               -1, 1)))
        dh = wrap180(r[f"{cam}_head"][ok] - head_t) * np.cos(np.radians(pitch_t))
        dp = r[f"{cam}_pitch"][ok] - pitch_t
        ts = t[ok]
        o = np.argsort(ts)
        ts, dh_s = ts[o], dh[o]
        out[cam] = {
            "n": int(ok.sum()),
            "bias_yaw": float(np.median(dh)),
            "bias_pitch": float(np.median(dp)),
            "sig_yaw": mad_sigma(dh - np.median(dh)),
            "sig_pitch": mad_sigma(dp - np.median(dp)),
            "rho_nat": lag_corr(ts, dh_s, 0.14, 0.06),
            "rho_1s": lag_corr(ts, dh_s, 1.0, 0.1),
            "range": float(np.median(rng)),
        }
    return out


def latency_sweep(sess: Session, pre: Dict) -> Tuple[float, float]:
    """Best time shift for the Calc series (should be ~0 if latency comp works)."""
    t = pre["t"]
    calc = pre["calc"]
    ok = np.isfinite(calc[:, 0])
    best = (0.0, float("inf"))
    for shift in np.arange(-0.4, 0.401, 0.05):
        tn = sess.truth_ned(t[ok] + shift)
        e = np.linalg.norm(calc[ok] - tn, axis=-1)
        med = float(np.nanmedian(e))
        if med < best[1]:
            best = (float(shift), med)
    return best


# ---------------------------------------------------------------------------
#  main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--windows", default="", help="comma list of window indices")
    ap.add_argument("--fast", action="store_true", help="skip R sweep")
    args = ap.parse_args()

    log = read_mono(verbose=True)
    sess = Session(log)
    for tr, d in zip(sess.tracks, sess.datum_enu):
        import os
        print(f"  datum offset {os.path.basename(tr.path):30s} "
              f"E {d[0]:+6.1f}  N {d[1]:+6.1f}  U {d[2]:+6.1f} m "
              f"(truth minus site frame; removed before scoring)")
    idxs = ([int(i) for i in args.windows.split(",")] if args.windows
            else list(range(len(GOOD_WINDOWS))))

    # -- 1. measurement characterisation (pooled) ---------------------------
    print("\n== ray residuals vs truth (per camera, pooled good windows) ==")
    pooled = np.zeros(len(sess.log), bool)
    for i in idxs:
        pooled |= sess.masks[i]
    ch = characterise(sess, pooled)
    for cam, st in ch.items():
        print(f"  {cam.upper()}: n={st['n']:5d} range~{st['range']:5.0f} m  "
              f"bias yaw {st['bias_yaw']:+6.3f} pitch {st['bias_pitch']:+6.3f} deg  "
              f"sigma yaw {st['sig_yaw']:5.3f} pitch {st['sig_pitch']:5.3f} deg  "
              f"rho(0.14s)={st['rho_nat']:+5.2f} rho(1s)={st['rho_1s']:+5.2f}")
    sig1 = max(0.02, ch["c1"]["sig_yaw"])
    sig2 = max(0.02, ch["c2"]["sig_yaw"])

    cams = make_cameras(sess, sig1, sig2)
    pres = {i: precompute(sess, sess.masks[i], cams) for i in idxs}

    sk = np.concatenate([pres[i]["skew"] for i in idxs])
    sk = sk[np.isfinite(sk)]
    print(f"\n  skew: median {np.median(sk):.2f} m  p90 "
          f"{np.percentile(sk, 90):.2f} m  frac>10m {np.mean(sk > 10):.1%}")
    sh, er = latency_sweep(sess, pres[idxs[0]])
    print(f"  latency check (window {idxs[0]}): best shift {sh:+.2f} s "
          f"(median err {er:.2f} m)")

    # -- 2. architectures ---------------------------------------------------
    scores: Dict[str, Score] = {}

    def get(label):
        return scores.setdefault(label, Score(label))

    for i in idxs:
        pre = pres[i]
        run_cv(sess, pre, get("VSL Calc (as logged)"), 0.0, source="calc")
        run_cv(sess, pre, get("VSL LPF (as logged)"), 0.0, source="lpf")
        run_cv(sess, pre, get("hold fix (skew<10)"), 0.0)
        run_cv(sess, pre, get("CV fit 1s"), 1.0)
        run_cv(sess, pre, get("CV fit 2s"), 2.0)
        run_cv(sess, pre, get("CV fit 5s"), 5.0)
        run_cv(sess, pre, get("CV fit 10s"), 10.0)
        run_rays(sess, pre, cams, get("rays-IMM native"))
        run_rays(sess, pre, cams, get("rays-IMM 3.3Hz"), min_dt=0.3)
        run_tri(sess, pre, get("tri-IMM (CRLB R)"))
        run_tri(sess, pre, get("tri-IMM iso R"), iso=True)
        run_tri(sess, pre, get("tri-IMM skew<10"), skew_gate=SKEW_GATE_M)

    hdr = "  " + " " * 24 + "  n      " + "  ".join(f"{h:5.2f}s" for h in HORIZONS)
    print("\n== prediction error, median TOTAL [m] by horizon ==")
    print(hdr)
    for sc in scores.values():
        print(sc.line())
    print("\n== prediction error, median ALONG-LOS (depth) [m] ==")
    print(hdr)
    for sc in scores.values():
        print(sc.line(depth=True))

    # -- 3. R scale on tri --------------------------------------------------
    if not args.fast:
        print("\n== R scale sweep (tri-IMM skew<10) ==")
        print(hdr)
        for s in (0.25, 1.0, 4.0, 16.0, 64.0):
            sc = Score(f"tri-IMM R x{s:g}")
            for i in idxs:
                run_tri(sess, pres[i], sc, r_scale=s, skew_gate=SKEW_GATE_M)
            print(sc.line())

    # -- 4. per-window table ------------------------------------------------
    print("\n== per-window medians, total [m] (tau=0 / tau=1.0) ==")
    for i in idxs:
        pre = pres[i]
        row = [f"window {i} {GOOD_WINDOWS[i][0]}-{GOOD_WINDOWS[i][1]}"]
        for label, runner in (
                ("Calc", lambda s: run_cv(sess, pre, s, 0.0, source="calc")),
                ("LPF", lambda s: run_cv(sess, pre, s, 0.0, source="lpf")),
                ("CV10", lambda s: run_cv(sess, pre, s, 10.0)),
                ("triIMM", lambda s: run_tri(sess, pre, s,
                                             skew_gate=SKEW_GATE_M))):
            s = Score(label)
            runner(s)
            row.append(f"{label} {s.med(0.0):5.1f}/{s.med(1.0):5.1f}")
        print("  " + "  ".join(row))


if __name__ == "__main__":
    main()
