"""Run the bearings-only IMM on real VSL stereo data and score it against truth.

Protocol
--------
Each flight window is split in half. The per-camera systematics (residual
boresight, station altitude, image latency) are fitted on the FIRST half and the
estimator is scored on the SECOND half, so the numbers reported are held-out.
Truth is used for (a) that calibration and (b) scoring -- never inside the
tracker, which sees only angles.

Frame association is truth-free: a CAM1 row is paired with the nearest CAM2 row
in latency-corrected image time, and frames where the two rays do not come close
are dropped by the existing skew gate. That gate is doing real work here -- the
detector spends much of each window locked onto something that is not the target
aircraft, and skew is what separates "both cameras on the plane" from "one
camera on a bird" without consulting truth.

Three estimates are scored side by side:
  * ``imm``   -- the bearings-only IMM (this stack)
  * ``tri``   -- per-frame ML triangulation only (the baseline)
  * ``vsl``   -- VSL's own Target_Lat/Lon/Alt as logged by their pipeline

Errors are split along-LOS (depth) and cross-LOS, because a single RMS hides the
anisotropy that dominates this geometry.
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import stereo_config as scfg
import stereo_geometry as sg
from stereo_estimator import StereoTracker, TrackState
from vsl_calibrate import (BoresightTable, CamCalibration, build_rays,
                           fit_camera, fit_site)
from vsl_eval import Window, build_windows, rig_geometry
from vsl_ingest import EnuFrame, enu_to_heading_pitch, utc, wrap180
from vsl_truth import TruthTrack


def enu_to_ned(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(-1, 3) if v.ndim > 1 else np.asarray(v, float).reshape(3)
    if v.ndim == 1:
        return np.array([v[1], v[0], -v[2]])
    return np.stack([v[:, 1], v[:, 0], -v[:, 2]], axis=1)


# ---------------------------------------------------------------------------
# frame association
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    stamp: float                  # latency-corrected image time (unix)
    ray1: Optional[np.ndarray]    # unit ENU
    ray2: Optional[np.ndarray]
    zoom1: float
    zoom2: float
    vsl_enu: Optional[np.ndarray] # VSL's own triangulation output, ENU
    skew_ms: float                # |t1 - t2| after latency correction, ms


def associate(win: Window, cal1: CamCalibration, cal2: CamCalibration,
              table: Optional[BoresightTable], frame: EnuFrame,
              t_lo: float, t_hi: float, max_pair_dt: float = 0.10
              ) -> List[Frame]:
    """Pair CAM1/CAM2 rows in latency-corrected image time."""
    c1, c2 = win.cam1, win.cam2
    m1 = c1.locked & np.isfinite(c1.u) & np.isfinite(c1.zoom) & np.isfinite(c1.elevation)
    m2 = c2.locked & np.isfinite(c2.u) & np.isfinite(c2.zoom) & np.isfinite(c2.elevation)
    t1 = c1.t - cal1.latency
    t2 = c2.t - cal2.latency
    m1 &= (t1 >= t_lo) & (t1 <= t_hi)
    m2 &= (t2 >= t_lo) & (t2 <= t_hi)
    i1 = np.flatnonzero(m1)
    i2 = np.flatnonzero(m2)
    if i1.size == 0:
        return []

    rays1 = build_rays(c1, i1, table, cal1.d_yaw, cal1.d_pitch)
    rays2 = build_rays(c2, i2, table, cal2.d_yaw, cal2.d_pitch) if i2.size else np.zeros((0, 3))

    ta = t1[i1]
    tb = t2[i2] if i2.size else np.zeros(0)
    order = np.argsort(tb) if tb.size else np.zeros(0, dtype=int)
    tb_sorted = tb[order] if tb.size else tb

    frames: List[Frame] = []
    for k, ia in enumerate(i1):
        if not np.all(np.isfinite(rays1[k])):
            continue
        r2 = None
        z2 = math.nan
        dt_ms = math.nan
        if tb_sorted.size:
            j = int(np.searchsorted(tb_sorted, ta[k]))
            cand = [c for c in (j - 1, j) if 0 <= c < tb_sorted.size]
            if cand:
                best = min(cand, key=lambda c: abs(tb_sorted[c] - ta[k]))
                if abs(tb_sorted[best] - ta[k]) <= max_pair_dt:
                    kk = int(order[best])
                    if np.all(np.isfinite(rays2[kk])):
                        r2 = rays2[kk]
                        z2 = float(c2.zoom[i2[kk]])
                        dt_ms = 1e3 * abs(tb_sorted[best] - ta[k])
        vsl = None
        if np.isfinite(c1.tri_lat[ia]) and abs(c1.tri_lat[ia]) < 90.0:
            vsl = frame.to_enu(c1.tri_lat[ia], c1.tri_lon[ia], c1.tri_alt[ia])
        frames.append(Frame(float(ta[k]), rays1[k], r2, float(c1.zoom[ia]), z2,
                            vsl, dt_ms))
    return frames


# ---------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------

@dataclass
class Score:
    name: str
    n: int
    total: np.ndarray
    along: np.ndarray
    cross: np.ndarray

    def line(self) -> str:
        if self.n == 0:
            return f"    {self.name:6s} n=0"
        q = lambda a, p: float(np.percentile(a, p))
        return (f"    {self.name:6s} n={self.n:5d}  "
                f"total med={np.median(self.total):7.1f} p90={q(self.total,90):7.1f}  "
                f"depth med={np.median(self.along):7.1f} p90={q(self.along,90):7.1f}  "
                f"cross med={np.median(self.cross):6.2f} p90={q(self.cross,90):6.2f}  [m]")


def score(name: str, est_ned: List[np.ndarray], truth_ned: List[np.ndarray],
          station_ned: np.ndarray) -> Score:
    tot, alo, crs = [], [], []
    for e, t in zip(est_ned, truth_ned):
        if e is None or not np.all(np.isfinite(e)):
            continue
        los = t - station_ned
        n = float(np.linalg.norm(los))
        if n < 1.0:
            continue
        err = np.asarray(e, float) - np.asarray(t, float)
        a, c = sg.los_decompose(err, los / n)
        tot.append(float(np.linalg.norm(err)))
        alo.append(abs(float(a)))
        crs.append(float(np.linalg.norm(c)) if np.ndim(c) else abs(float(c)))
    return Score(name, len(tot), np.array(tot), np.array(alo), np.array(crs))


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def site_key(win: Window) -> Tuple[float, float]:
    s1 = win.cam1.station_position()
    return (round(s1[0], 4), round(s1[1], 4))


def calibrate_site(wins: Sequence[Window], table: Optional[BoresightTable], args
                   ) -> Optional[Tuple[CamCalibration, CamCalibration]]:
    """One boresight per camera per site, fitted on the FIRST HALF of every window.

    The mount is fixed, so the boresight belongs to the site rather than the
    window (see vsl_calibrate.fit_site). Pooling only first halves keeps the
    scored second halves genuinely held out.
    """
    geo = rig_geometry(wins[0])
    frame = EnuFrame(*geo["cam1"])
    out = []
    for which, stkey in (("cam1", "cam1"), ("cam2", "cam2")):
        segs = []
        for w in wins:
            cam = getattr(w, which)
            t_mid = w.t0 + 0.5 * w.duration
            segs.append((cam.subset((cam.t >= w.t0) & (cam.t <= t_mid)), w.truth))
        cal = fit_site(segs, frame, geo[stkey], table, subsample=args.subsample)
        if cal is None:
            return None
        out.append(cal)
    return out[0], out[1]


def run_window(win: Window, table: Optional[BoresightTable], args,
               cals: Optional[Tuple[CamCalibration, CamCalibration]] = None) -> None:
    geo = rig_geometry(win)
    s1, s2 = geo["cam1"], geo["cam2"]
    frame = EnuFrame(*s1)

    t_mid = win.t0 + 0.5 * win.duration
    print(f"\n{'='*90}")
    print(f"WINDOW {win.label}  dur={win.duration:.0f}s  "
          f"truth={os.path.basename(win.truth.path)}")
    print(f"  baseline {geo['baseline_m']:.1f} m az={geo['baseline_az_deg']:.0f}deg   "
          f"score on {utc(t_mid)}..{utc(win.t1)}")

    cal_cams = []
    for k, (cam, st) in enumerate(((win.cam1, s1), (win.cam2, s2))):
        if cals is not None:
            cal = cals[k]
        else:
            sub = cam.subset((cam.t >= win.t0) & (cam.t <= t_mid))
            cal = fit_camera(sub, win.truth, frame, st, table, subsample=args.subsample)
        if cal is None:
            print(f"  {cam.cam_id}: calibration FAILED -- skipping window")
            return
        print(f"  {cal.describe()}")
        if cal.alt_pitch_corr > 0.9:
            print(f"        (dAlt/dPitch correlation {cal.alt_pitch_corr:.2f}: not separable, "
                  f"treat dAlt as absorbed into dPitch)")
        if cal.heading_error_deg is not None:
            print(f"        -> entered Gimbal_Heading is wrong by "
                  f"{cal.heading_error_deg:+.1f} deg (well supported); re-survey it")
        for why in cal.suspect:
            print(f"        !! SUSPECT: {why}")
        cal_cams.append((cal, st))
    if any(c.suspect for c, _ in cal_cams):
        print("  -> calibration not credible; window reported but NOT headline-quality")
    cal1, cal2 = cal_cams[0][0], cal_cams[1][0]

    # measurement sigma for the filter: azimuth-domain, so undo the cos(elev)
    # scaling the characterisation applies.
    cos_e = max(0.2, math.cos(math.radians(args.elev_ref)))
    cams = []
    for (cal, st) in cal_cams:
        p_enu = frame.to_enu(*st) + np.array([0.0, 0.0, cal.d_alt])
        cams.append(sg.Camera(
            name=cal.cam_id, position_ned=enu_to_ned(p_enu),
            yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0,
            sigma_yaw_deg=max(0.02, cal.sigma_yaw / cos_e),
            sigma_pitch_deg=max(0.02, cal.sigma_pitch),
            fov_yaw_deg=359.0, fov_pitch_deg=179.0, max_range_m=20000.0))
    print(f"  filter R: {cams[0].name} sigma=({math.degrees(cams[0].sigma_yaw):.3f},"
          f"{math.degrees(cams[0].sigma_pitch):.3f})deg  "
          f"{cams[1].name} sigma=({math.degrees(cams[1].sigma_yaw):.3f},"
          f"{math.degrees(cams[1].sigma_pitch):.3f})deg")

    frames = associate(win, cal1, cal2, table, frame, t_mid, win.t1,
                       max_pair_dt=args.max_pair_dt)
    both = sum(1 for f in frames if f.ray2 is not None)
    print(f"  frames: {len(frames)} (both cameras: {both}, "
          f"{100*both/max(1,len(frames)):.0f}%)")
    if both < 50:
        print("  too few dual-camera frames to score")
        return

    tracker = StereoTracker(cams, cfg=scfg, nominal_dt=args.nominal_dt)
    est_imm: List[Optional[np.ndarray]] = []
    est_tri: List[Optional[np.ndarray]] = []
    est_vsl: List[Optional[np.ndarray]] = []
    truth_ned: List[np.ndarray] = []
    n_track = 0
    skew_hist: List[float] = []

    for fr in frames:
        dets = []
        if fr.ray1 is not None:
            h, p = enu_to_heading_pitch(fr.ray1)
            dets.append(sg.Detection(0, math.radians(h), math.radians(p), fr.stamp))
        if fr.ray2 is not None:
            h, p = enu_to_heading_pitch(fr.ray2)
            dets.append(sg.Detection(1, math.radians(h), math.radians(p), fr.stamp))
        if not dets:
            continue
        snap = tracker.process(dets, fr.stamp)
        lat_t, lon_t, alt_t = win.truth.sample([fr.stamp])
        t_enu = frame.to_enu(lat_t[0], lon_t[0], alt_t[0])
        t_ned = enu_to_ned(t_enu)

        geom = snap.get("geom")
        if geom is not None and np.isfinite(geom.get("skew_m", np.nan)):
            skew_hist.append(float(geom["skew_m"]))

        if snap["tracking"]:
            n_track += 1
            est_imm.append(np.asarray(snap["position"], float))
        else:
            est_imm.append(None)
        est_tri.append(np.asarray(geom["fix"], float) if geom is not None else None)
        est_vsl.append(enu_to_ned(fr.vsl_enu) if fr.vsl_enu is not None else None)
        truth_ned.append(t_ned)

    station_ned = enu_to_ned(frame.to_enu(*s1))
    print(f"  tracker: {n_track}/{len(truth_ned)} frames with a track "
          f"({100*n_track/max(1,len(truth_ned)):.0f}%), "
          f"state={tracker.state}, updates={tracker.updates}, reinits={tracker.reinits}")
    if skew_hist:
        sk = np.array(skew_hist)
        print(f"  ray skew: med={np.median(sk):.2f} m p90={np.percentile(sk,90):.1f} m "
              f"(reject>{scfg.SKEW_REJECT_M:.0f} m: {100*np.mean(sk>scfg.SKEW_REJECT_M):.0f}%)")

    # score only where an estimate exists, but keep the sets comparable by
    # reporting each estimator on its own valid frames plus a common subset.
    print("  --- all frames each estimator produced ---")
    for nm, est in (("imm", est_imm), ("tri", est_tri), ("vsl", est_vsl)):
        print(score(nm, est, truth_ned, station_ned).line())

    common = [i for i in range(len(truth_ned))
              if est_imm[i] is not None and est_tri[i] is not None]
    if common:
        print(f"  --- common frames (imm & tri both valid, n={len(common)}) ---")
        for nm, est in (("imm", est_imm), ("tri", est_tri), ("vsl", est_vsl)):
            print(score(nm, [est[i] for i in common],
                        [truth_ned[i] for i in common], station_ned).line())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--root",
        default=os.environ.get(
            "VSL_DATA_ROOT",
            os.path.expanduser("~/savasan_iha_yildizlar_data/VSL/logs"),
        ),
    )
    ap.add_argument("--min-dur", type=float, default=300.0)
    ap.add_argument("--subsample", type=int, default=2)
    ap.add_argument("--max-pair-dt", type=float, default=0.10)
    ap.add_argument("--nominal-dt", type=float, default=0.05)
    ap.add_argument("--elev-ref", type=float, default=28.0,
                    help="reference elevation for converting angular to azimuth sigma")
    ap.add_argument("--no-table", action="store_true",
                    help="ignore VSL's boresight_offsets.json and fit from scratch")
    ap.add_argument("--only", default=None, help="substring filter on window label")
    ap.add_argument("--per-window", action="store_true",
                    help="fit boresight per window instead of pooling per site")
    args = ap.parse_args()

    table = None if args.no_table else BoresightTable()
    if table is not None and not table.ok:
        print("warning: boresight table unavailable; fitting from scratch")
        table = None

    wins = [w for w in build_windows(args.root, verbose=False) if w.duration >= args.min_dur]
    if args.only:
        wins = [w for w in wins if args.only in w.label]
    print(f"{len(wins)} window(s) to run")

    if args.per_window:
        for w in wins:
            run_window(w, table, args)
        return

    sites: Dict[Tuple[float, float], List[Window]] = {}
    for w in wins:
        sites.setdefault(site_key(w), []).append(w)
    for key, group in sites.items():
        geo = rig_geometry(group[0])
        print(f"\n{'#'*90}")
        print(f"# SITE {key}  baseline {geo['baseline_m']:.1f} m  "
              f"heading {np.median(group[0].cam1.heading):.0f}/"
              f"{np.median(group[0].cam2.heading):.0f}  windows={len(group)}")
        print(f"{'#'*90}")
        cals = calibrate_site(group, table, args)
        if cals is None:
            print("  site calibration FAILED -- skipping")
            continue
        print(f"  pooled: {cals[0].describe()}")
        print(f"  pooled: {cals[1].describe()}")
        for w in group:
            run_window(w, table, args, cals=cals)


if __name__ == "__main__":
    main()
