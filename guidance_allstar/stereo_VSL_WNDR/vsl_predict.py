"""Prediction accuracy, R tuning, and the rays-vs-triangulate architecture call.

Guidance does not consume the current estimate; it consumes a *prediction*. At
20 m/s closing, a 1 s lead error is the miss. So the metric here is not "how
close is the state to truth now" but "how close is p + v*tau + 0.5*a*tau^2 to
where the target actually was tau seconds later" -- computed from the estimator
state exactly the way the guidance layer computes it.

Three questions, one harness:

**1. How far ahead can this sensor actually predict?** Errors are reported at
tau = 0, 0.25, 0.5, 0.75, 1.0 s, split along-LOS (depth) and cross-LOS, against
two dumb baselines -- holding the last fix, and constant-velocity extrapolation
of per-frame triangulation. Anything the estimator cannot beat is not worth its
complexity.

**2. What R does this camera want?** A scale sweep on the measured sigma, and --
more importantly -- a feed-rate sweep. The residuals are strongly
autocorrelated (see vsl_noise.py), so a filter fed at full rate with white-noise
R believes it has far more independent information than it does. Thinning and
inflating R are two ways to pay the same debt; the sweep says which the filter
prefers.

**3. Rays or triangulate-first?** Two architectures over identical data:

  ``rays``  the IMM updates directly on the four angles (current design). The
            measurement is what the sensor actually reports, the anisotropic
            depth uncertainty falls out of the geometry, and a single camera
            still constrains the track.
  ``tri``   per-frame ML triangulation to a 3D point, fed to the IMM as a
            position measurement with the CRLB covariance from
            ``triangulation_covariance``. Simpler, reuses the position pipeline
            that already flies -- but it needs both cameras every frame and it
            linearises the geometry once per frame rather than inside the filter.

``tri-iso`` is the same as ``tri`` but with an isotropic R of the same trace --
included because that is what a naive "just give the filter a position" port
would do, and the gap between ``tri`` and ``tri-iso`` is the cost of throwing
away the anisotropy.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import filterwndr as fw
import stereo_config as scfg
import stereo_geometry as sg
import stereo_measurement as sm
from stereo_estimator import StereoTracker, TrackState
from vsl_calibrate import BoresightTable, CamCalibration, fit_site
from vsl_eval import Window, build_windows, rig_geometry
from vsl_ingest import EnuFrame, enu_to_heading_pitch, utc
from vsl_run import associate, enu_to_ned

HORIZONS = (0.0, 0.25, 0.5, 0.75, 1.0)


class Cfg:
    """A copy of stereo_config with overrides."""

    def __init__(self, base, **kw):
        for k in dir(base):
            if k.isupper():
                setattr(self, k, getattr(base, k))
        for k, v in kw.items():
            setattr(self, k, v)


def predict_state(x: np.ndarray, tau: float, use_accel: bool = True) -> np.ndarray:
    """Where guidance thinks the target will be in tau seconds."""
    p = np.asarray(x[0:3], dtype=float)
    v = np.asarray(x[3:6], dtype=float)
    if tau <= 0.0:
        return p.copy()
    out = p + v * tau
    if use_accel and x.shape[0] >= 9:
        out = out + 0.5 * np.asarray(x[6:9], dtype=float) * tau * tau
    return out


@dataclass
class PredScore:
    label: str
    n: Dict[float, int] = field(default_factory=dict)
    tot: Dict[float, np.ndarray] = field(default_factory=dict)
    alo: Dict[float, np.ndarray] = field(default_factory=dict)

    def line(self) -> str:
        parts = []
        for h in HORIZONS:
            if h in self.tot and self.tot[h].size:
                parts.append(f"{h:.2f}s:{np.median(self.tot[h]):6.1f}")
            else:
                parts.append(f"{h:.2f}s:{'--':>6s}")
        n0 = self.n.get(0.0, 0)
        return f"  {self.label:26s} n={n0:5d}  " + "  ".join(parts)

    def line_depth(self) -> str:
        parts = []
        for h in HORIZONS:
            if h in self.alo and self.alo[h].size:
                parts.append(f"{h:.2f}s:{np.median(self.alo[h]):6.1f}")
            else:
                parts.append(f"{h:.2f}s:{'--':>6s}")
        return f"  {self.label:26s} depth   " + "  ".join(parts)


def _accumulate(store: Dict[float, List[float]], h: float, val: float) -> None:
    store.setdefault(h, []).append(val)


def run_arch(win: Window, cals: Tuple[CamCalibration, CamCalibration],
             table: Optional[BoresightTable], frame: EnuFrame, geo,
             arch: str = "rays", r_scale: float = 1.0, feed_hz: float = 0.0,
             t_lo: Optional[float] = None, t_hi: Optional[float] = None,
             label: Optional[str] = None) -> Tuple[PredScore, PredScore, PredScore, Dict]:
    """Run one architecture/tuning over a window, scoring predictions."""
    t_lo = win.t0 if t_lo is None else t_lo
    t_hi = win.t1 if t_hi is None else t_hi
    frames = associate(win, cals[0], cals[1], table, frame, t_lo, t_hi)
    if not frames:
        return PredScore(label or arch), PredScore("hold"), PredScore("tri+CV"), {}

    cams = []
    for cal, key in ((cals[0], "cam1"), (cals[1], "cam2")):
        p = frame.to_enu(*geo[key]) + np.array([0.0, 0.0, cal.d_alt])
        cams.append(sg.Camera(
            cal.cam_id, enu_to_ned(p), 0.0, 0.0, 0.0,
            sigma_yaw_deg=max(0.02, cal.sigma_yaw * math.sqrt(r_scale)),
            sigma_pitch_deg=max(0.02, cal.sigma_pitch * math.sqrt(r_scale)),
            fov_yaw_deg=359.0, fov_pitch_deg=179.0, max_range_m=20000.0))

    cfg = Cfg(scfg)
    est = PredScore(label or arch)
    hold = PredScore("baseline: hold last fix")
    tricv = PredScore("baseline: tri + CV")
    e_tot: Dict[float, List[float]] = {}
    e_alo: Dict[float, List[float]] = {}
    h_tot: Dict[float, List[float]] = {}
    h_alo: Dict[float, List[float]] = {}
    c_tot: Dict[float, List[float]] = {}
    c_alo: Dict[float, List[float]] = {}

    station_ned = enu_to_ned(frame.to_enu(*geo["cam1"]))
    min_dt = (1.0 / feed_hz) if feed_hz and feed_hz > 0 else 0.0
    last_fed = None

    tracker = None
    imm = None
    last_stamp = None
    tri_hist: List[Tuple[float, np.ndarray]] = []
    n_upd = 0

    if arch == "rays":
        tracker = StereoTracker(cams, cfg=cfg, nominal_dt=0.05)

    for fr in frames:
        if min_dt and last_fed is not None and (fr.stamp - last_fed) < min_dt:
            continue
        last_fed = fr.stamp

        dets = []
        for i, ray in ((0, fr.ray1), (1, fr.ray2)):
            if ray is not None:
                h, p = enu_to_heading_pitch(ray)
                dets.append(sg.Detection(i, math.radians(h), math.radians(p), fr.stamp))
        if not dets:
            continue

        x = None
        fix = None
        if arch == "rays":
            snap = tracker.process(dets, fr.stamp)
            g = snap.get("geom")
            fix = np.asarray(g["fix"], float) if g is not None else None
            if snap["tracking"]:
                x = np.asarray(snap["x"], float)
        else:
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
            if float(info.get("skew_m", 0.0)) > float(getattr(cfg, "SKEW_REJECT_M", 60.0)):
                continue
            R = np.asarray(cov, float) * r_scale
            if arch == "tri-iso":
                R = np.eye(3) * (np.trace(R) / 3.0)
            R = R + np.eye(3) * 1e-6
            if imm is None:
                imm = fw.setup_imm_filter(0.05)
                for kf in imm.filters:
                    kf.x[0:3] = fix
                    kf.P[0:3, 0:3] = R * 4.0
                imm.x = np.array(imm.filters[0].x, dtype=float).copy()
                last_stamp = fr.stamp
                continue
            dt = float(np.clip(fr.stamp - last_stamp, 1e-3, 1.0))
            try:
                fw.predict_imm_over_dt(imm, dt, max_substep=0.1)
            except np.linalg.LinAlgError:
                imm = None
                continue
            last_stamp = fr.stamp
            for kf in imm.filters:
                kf.R = R.copy()
            try:
                imm.update(np.asarray(fix, float).reshape(3))
            except np.linalg.LinAlgError:
                imm = None
                continue
            x = np.asarray(imm.x, float).reshape(-1)
        if fix is not None:
            tri_hist.append((fr.stamp, fix))
            if len(tri_hist) > 40:
                tri_hist.pop(0)
        if x is None:
            continue
        n_upd += 1

        for tau in HORIZONS:
            tq = fr.stamp + tau
            if tq > win.truth.span[1]:
                continue
            lat_t, lon_t, alt_t = win.truth.sample([tq])
            tn = enu_to_ned(frame.to_enu(lat_t[0], lon_t[0], alt_t[0]))
            los = tn - station_ned
            nrm = float(np.linalg.norm(los))
            if nrm < 1.0:
                continue
            u = los / nrm

            pe = predict_state(x, tau)
            a, _ = sg.los_decompose(pe - tn, u)
            _accumulate(e_tot, tau, float(np.linalg.norm(pe - tn)))
            _accumulate(e_alo, tau, abs(float(a)))

            if fix is not None:
                a, _ = sg.los_decompose(fix - tn, u)
                _accumulate(h_tot, tau, float(np.linalg.norm(fix - tn)))
                _accumulate(h_alo, tau, abs(float(a)))
                if len(tri_hist) >= 5:
                    t0, p0 = tri_hist[0]
                    t1, p1 = tri_hist[-1]
                    span = max(t1 - t0, 1e-3)
                    vel = (p1 - p0) / span
                    pc = p1 + vel * tau
                    a, _ = sg.los_decompose(pc - tn, u)
                    _accumulate(c_tot, tau, float(np.linalg.norm(pc - tn)))
                    _accumulate(c_alo, tau, abs(float(a)))

    for store, sc in ((e_tot, est), (h_tot, hold), (c_tot, tricv)):
        for h, v in store.items():
            sc.tot[h] = np.array(v)
            sc.n[h] = len(v)
    for store, sc in ((e_alo, est), (h_alo, hold), (c_alo, tricv)):
        for h, v in store.items():
            sc.alo[h] = np.array(v)
    return est, hold, tricv, {"updates": n_upd, "frames": len(frames)}


def site_windows(args) -> Tuple[List[Window], Tuple[CamCalibration, CamCalibration], EnuFrame, Dict]:
    table = BoresightTable()
    wins = [w for w in build_windows(verbose=False) if w.duration >= args.min_dur]

    def key(w):
        s1 = w.cam1.station_position()
        return f"{s1[0]:.4f},{s1[1]:.4f}"

    wins = [w for w in wins if key(w) == args.site]
    if not wins:
        raise SystemExit(f"no windows at site {args.site}")
    geo = rig_geometry(wins[0])
    frame = EnuFrame(*geo["cam1"])
    cals = []
    for which, k in (("cam1", "cam1"), ("cam2", "cam2")):
        segs = [(getattr(w, which), w.truth) for w in wins]
        cal = fit_site(segs, frame, geo[k], table, subsample=args.subsample)
        if cal is None:
            raise SystemExit(f"site calibration failed for {which}")
        cals.append(cal)
    return wins, (cals[0], cals[1]), frame, geo, table


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", required=True,
                    help="deployment selector as 'lat,lon' to 4 decimal places")
    ap.add_argument("--min-dur", type=float, default=300.0)
    ap.add_argument("--subsample", type=int, default=3)
    ap.add_argument("--window", default="13:01", help="substring of the window to score")
    ap.add_argument("--mode", default="all",
                    choices=["all", "arch", "rscale", "rate"])
    args = ap.parse_args()

    wins, cals, frame, geo, table = site_windows(args)
    scored = [w for w in wins if args.window in w.label]
    if not scored:
        raise SystemExit(f"no window matching {args.window}")
    win = scored[0]
    t_mid = win.t0 + 0.5 * win.duration

    print(f"site {args.site}  baseline {geo['baseline_m']:.1f} m")
    for c in cals:
        print(f"  {c.describe()}")
    print(f"scoring {win.label} second half ({utc(t_mid)}..{utc(win.t1)})")
    print(f"\nprediction error, median total [m] by horizon "
          f"(guidance uses p + v*tau + 0.5*a*tau^2)")
    print(f"  {'config':26s} {'':7s}  " +
          "  ".join(f"{h:.2f}s{'':>2s}" for h in HORIZONS))

    def show(tag, r_scale=1.0, feed_hz=0.0, arch="rays", baselines=False):
        est, hold, tricv, info = run_arch(
            win, cals, table, frame, geo, arch=arch, r_scale=r_scale,
            feed_hz=feed_hz, t_lo=t_mid, t_hi=win.t1, label=tag)
        print(est.line())
        if baselines:
            print(hold.line())
            print(tricv.line())
        return est

    if args.mode in ("all", "arch"):
        print("\n-- architecture --")
        show("rays (current)", arch="rays", baselines=True)
        show("triangulate -> IMM", arch="tri")
        show("triangulate -> IMM (iso R)", arch="tri-iso")

    if args.mode in ("all", "rscale"):
        print("\n-- R scale (rays) --")
        for s in (0.25, 1.0, 4.0, 16.0, 64.0):
            show(f"rays  R x{s:g}", r_scale=s, arch="rays")

    if args.mode in ("all", "rate"):
        print("\n-- feed rate (rays, R x1) --")
        for hz in (0.0, 10.0, 5.0, 2.0, 1.0):
            show(f"rays  feed {hz:g} Hz" if hz else "rays  feed native", feed_hz=hz)


if __name__ == "__main__":
    main()
