"""Characterise the real VSL stereo rig against target truth.

This is the measurement stage that has to happen before any filter is trusted:
feeding an estimator the wrong R is the one failure mode a covariance gate
cannot defend against, so the noise model comes from the data, not from a guess.

What it measures, per camera, per flight window:

* **effective latency** -- the logged ``System_Timestamp`` is when the row was
  written; the image behind it is older (VSL's own config declares 0.308 s
  capture->GUI plus 0.087 s YOLO). Latency is recovered by scanning a time
  offset and taking the one that minimises bearing residual scatter. This both
  corrects the residuals and independently checks their declared constants.
* **boresight bias** -- the mean residual in yaw and pitch. This is the term
  that maps straight into depth error along the baseline.
* **angular noise** -- the robust scatter about that bias, split yaw/pitch, and
  broken out by zoom so it can be expressed in pixels as well as degrees.
* **rig geometry** -- the actual baseline vector, so the depth/cross-LOS error
  anisotropy can be predicted before running any filter.

Bearing residuals are computed in the *camera-relative* angular frame the
measurement actually lives in: yaw residual is scaled by cos(elevation) so a
high-elevation pass does not inflate the apparent azimuth error.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from vsl_ingest import (CamStream, EnuFrame, enu_to_heading_pitch, read_allstar,
                        utc, vsl_intrinsics, vsl_ray_enu, wrap180)
from vsl_truth import TruthTrack, load_all_truth, pair_truth_to_window

DEFAULT_ROOT = os.environ.get(
    "VSL_DATA_ROOT", os.path.expanduser("~/savasan_iha_yildizlar_data/VSL/logs"))


def discover_sessions(root: str = DEFAULT_ROOT) -> Tuple[str, ...]:
    """Any subdirectory holding a pair of allstar camera logs.

    Discovered rather than hardcoded: the delivered folders get renamed as
    telemetry arrives (``27-no-target-telem`` became ``27``), and a hardcoded
    list silently drops a session when that happens.
    """
    if not os.path.isdir(root):
        return ()
    out = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if (os.path.isdir(d)
                and os.path.exists(os.path.join(d, "allstar_cam1_log.txt"))
                and os.path.exists(os.path.join(d, "allstar_cam2_log.txt"))):
            out.append(name)
    return tuple(out)


SESSIONS = discover_sessions()


# ---------------------------------------------------------------------------
# window discovery
# ---------------------------------------------------------------------------

@dataclass
class Window:
    session: str
    t0: float
    t1: float
    cam1: CamStream
    cam2: CamStream
    truth: Optional[TruthTrack] = None
    overlap: float = 0.0

    @property
    def label(self) -> str:
        return f"{self.session}/{utc(self.t0)}"

    @property
    def duration(self) -> float:
        return self.t1 - self.t0


def segment_bounds(t: np.ndarray, gap: float = 60.0, min_rows: int = 200
                   ) -> List[Tuple[float, float]]:
    idx = np.flatnonzero(np.diff(t) > gap)
    out = []
    for seg in np.split(np.arange(t.size), idx + 1):
        if seg.size >= min_rows:
            out.append((float(t[seg[0]]), float(t[seg[-1]])))
    return out


def baseline_of(lat1, lon1, alt1, lat2, lon2, alt2) -> float:
    return float(np.linalg.norm(
        EnuFrame(lat1, lon1, alt1).to_enu(lat2, lon2, alt2)))


def baseline_mask(cam1: CamStream, cam2: CamStream,
                  lo: float, hi: float) -> np.ndarray:
    """Per-row mask: is the LOGGED station geometry physically plausible?

    The rig config is edited live, and a bad edit is not rare: a 1 deg longitude
    typo put CAM2 84 km away for the first nine minutes of the 23 July flight,
    and their triangulator happily emitted 15 km target altitudes off it. Rows
    logged under an impossible baseline are config noise, not measurements, so
    they are dropped rather than averaged over.
    """
    order = np.argsort(cam2.t)
    t2 = cam2.t[order]
    j = np.clip(np.searchsorted(t2, cam1.t), 0, max(0, t2.size - 1))
    k = order[j] if t2.size else np.zeros(cam1.t.size, dtype=int)
    keep = np.zeros(cam1.t.size, dtype=bool)
    if t2.size == 0:
        return keep
    # station config is piecewise-constant, so evaluate once per distinct combo
    combos: Dict[Tuple, List[int]] = {}
    for i in range(cam1.t.size):
        key = (round(float(cam1.lat[i]), 7), round(float(cam1.lon[i]), 7),
               round(float(cam1.alt[i]), 2),
               round(float(cam2.lat[k[i]]), 7), round(float(cam2.lon[k[i]]), 7),
               round(float(cam2.alt[k[i]]), 2))
        combos.setdefault(key, []).append(i)
    for key, rows in combos.items():
        if not all(math.isfinite(v) for v in key):
            continue
        b = baseline_of(*key)
        if lo <= b <= hi:
            keep[np.array(rows)] = True
    return keep


def build_windows(root: str = DEFAULT_ROOT, sessions: Sequence[str] = SESSIONS,
                  require_truth: bool = True, verbose: bool = True,
                  baseline_lo: float = 80.0, baseline_hi: float = 150.0,
                  min_rows: int = 400) -> List[Window]:
    """Discover flight windows, keeping only rows with a plausible baseline.

    Segments are re-derived AFTER the baseline gate, so a session whose config
    was fixed mid-flight yields a window covering only the healthy part rather
    than one window straddling both regimes.
    """
    tracks = load_all_truth(root, verbose=False) if require_truth else []
    seen_spans = set()
    windows: List[Window] = []
    for sess in sessions:
        p1 = os.path.join(root, sess, "allstar_cam1_log.txt")
        p2 = os.path.join(root, sess, "allstar_cam2_log.txt")
        if not (os.path.exists(p1) and os.path.exists(p2)):
            continue
        cam1 = read_allstar(p1, "CAM1")
        cam2 = read_allstar(p2, "CAM2")
        ok = baseline_mask(cam1, cam2, baseline_lo, baseline_hi)
        if verbose and not ok.all():
            print(f"   [{sess}] baseline gate [{baseline_lo:.0f},{baseline_hi:.0f}] m "
                  f"drops {int((~ok).sum())}/{ok.size} rows "
                  f"({100*(~ok).mean():.1f}%)")
        cam1 = cam1.subset(ok)
        if len(cam1) < min_rows:
            continue
        for t0, t1 in segment_bounds(cam1.t, min_rows=min_rows):
            # Sessions are cumulative appends of ONE log file, so a later folder
            # re-contains earlier flights (folder 27 holds the 23 July window
            # again). Dedup to whole seconds: the copies differ by well under a
            # frame, and a tighter key lets the duplicate through.
            key = (int(round(t0)), int(round(t1)))
            if key in seen_spans:
                continue
            seen_spans.add(key)
            hits = pair_truth_to_window(tracks, t0, t1) if tracks else []
            if require_truth and not hits:
                continue
            tr, frac = hits[0] if hits else (None, 0.0)
            w = Window(sess, t0, t1,
                       cam1.subset((cam1.t >= t0) & (cam1.t <= t1)),
                       cam2.subset((cam2.t >= t0) & (cam2.t <= t1)),
                       tr, frac)
            windows.append(w)
            if verbose:
                print(f"   {w.label} dur={w.duration:5.0f}s "
                      f"cam1={len(w.cam1):6d} cam2={len(w.cam2):6d} "
                      f"truth={os.path.basename(tr.path) if tr else '-':28s} "
                      f"ov={100*frac:5.1f}%")
    return windows


# ---------------------------------------------------------------------------
# bearing residuals
# ---------------------------------------------------------------------------

@dataclass
class ResidualSet:
    cam_id: str
    n: int
    latency: float
    bias_yaw: float
    bias_pitch: float
    sigma_yaw: float
    sigma_pitch: float
    range_med: float
    elev_med: float
    zoom_med: float
    sigma_px_yaw: float
    sigma_px_pitch: float
    d_yaw: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    d_pitch: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    rng: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    zoom: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    t: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))


def _robust_sigma(x: np.ndarray) -> float:
    """MAD-based sigma; immune to the detector's outlier tail."""
    if x.size == 0:
        return math.nan
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def measured_rays(cam: CamStream, mask: Optional[np.ndarray] = None) -> np.ndarray:
    """Unit ENU rays for the selected rows (boresight NOT applied)."""
    m = np.ones(len(cam), dtype=bool) if mask is None else np.asarray(mask)
    idx = np.flatnonzero(m)
    out = np.full((idx.size, 3), np.nan)
    for k, i in enumerate(idx):
        if not (np.isfinite(cam.u[i]) and np.isfinite(cam.zoom[i])
                and np.isfinite(cam.elevation[i]) and np.isfinite(cam.yaw_world[i])):
            continue
        try:
            out[k] = vsl_ray_enu(cam.u[i], cam.v[i], cam.zoom[i],
                                 cam.elevation[i], cam.roll[i], cam.yaw_world[i],
                                 cam.cam_id)
        except ValueError:
            continue
    return out


def bearing_residuals(cam: CamStream, truth: TruthTrack, frame: EnuFrame,
                      station: Sequence[float], latency: float,
                      locked_only: bool = True) -> Tuple[np.ndarray, ...]:
    """Angular residual (measured ray - true bearing) at a given latency.

    Returns (d_yaw, d_pitch, range_m, elev_deg, zoom, t). The yaw residual is
    multiplied by cos(elevation) so it is a true angular miss on the sky rather
    than an azimuth number inflated near the zenith.
    """
    m = cam.locked if locked_only else np.ones(len(cam), dtype=bool)
    m = m & np.isfinite(cam.u) & np.isfinite(cam.zoom) & np.isfinite(cam.elevation)
    m = m & truth.covers(cam.t - latency)
    idx = np.flatnonzero(m)
    if idx.size == 0:
        z = np.zeros(0)
        return z, z, z, z, z, z

    t_img = cam.t[idx] - latency
    lat_t, lon_t, alt_t = truth.sample(t_img)
    station_enu = frame.to_enu(*station)

    d_yaw = np.full(idx.size, np.nan)
    d_pitch = np.full(idx.size, np.nan)
    rng = np.full(idx.size, np.nan)
    elev = np.full(idx.size, np.nan)

    for k, i in enumerate(idx):
        try:
            d_meas = vsl_ray_enu(cam.u[i], cam.v[i], cam.zoom[i],
                                 cam.elevation[i], cam.roll[i], cam.yaw_world[i],
                                 cam.cam_id)
        except ValueError:
            continue
        vec = frame.to_enu(lat_t[k], lon_t[k], alt_t[k]) - station_enu
        r = float(np.linalg.norm(vec))
        if not (r > 1.0):
            continue
        h_m, p_m = enu_to_heading_pitch(d_meas)
        h_t, p_t = enu_to_heading_pitch(vec / r)
        d_yaw[k] = wrap180(h_m - h_t) * math.cos(math.radians(p_t))
        d_pitch[k] = p_m - p_t
        rng[k] = r
        elev[k] = p_t

    ok = np.isfinite(d_yaw) & np.isfinite(d_pitch)
    return (d_yaw[ok], d_pitch[ok], rng[ok], elev[ok],
            cam.zoom[idx][ok], cam.t[idx][ok])


def fit_latency(cam: CamStream, truth: TruthTrack, frame: EnuFrame,
                station: Sequence[float],
                grid: Sequence[float] = tuple(np.arange(-0.20, 0.85, 0.02)),
                subsample: int = 4) -> Tuple[float, Dict[float, float]]:
    """Pick the latency that minimises combined robust residual scatter.

    The gimbal is slewing, so a wrong latency shows up as residual scatter that
    correlates with angular rate. Minimising the MAD-based sigma therefore
    recovers the true image age without needing their declared constants.
    """
    thin = cam.subset(np.arange(len(cam)) % max(1, subsample) == 0)
    curve: Dict[float, float] = {}
    best_lat, best_cost = math.nan, math.inf
    for lat in grid:
        dy, dp, *_ = bearing_residuals(thin, truth, frame, station, float(lat))
        if dy.size < 50:
            continue
        cost = math.hypot(_robust_sigma(dy), _robust_sigma(dp))
        curve[float(lat)] = cost
        if cost < best_cost:
            best_cost, best_lat = cost, float(lat)
    return best_lat, curve


def characterise(cam: CamStream, truth: TruthTrack, frame: EnuFrame,
                 station: Sequence[float], latency: Optional[float] = None
                 ) -> Optional[ResidualSet]:
    if latency is None:
        latency, _ = fit_latency(cam, truth, frame, station)
    if not np.isfinite(latency):
        return None
    dy, dp, rng, elev, zoom, t = bearing_residuals(cam, truth, frame, station, latency)
    if dy.size < 30:
        return None
    # sigma in pixels: sigma_deg -> px via the focal length actually in use
    fx_list, fy_list = [], []
    for z in zoom:
        fx, fy, _, _ = vsl_intrinsics(cam.cam_id, float(z))
        fx_list.append(fx)
        fy_list.append(fy)
    fx = np.array(fx_list)
    fy = np.array(fy_list)
    sy, sp = _robust_sigma(dy), _robust_sigma(dp)
    return ResidualSet(
        cam_id=cam.cam_id, n=int(dy.size), latency=float(latency),
        bias_yaw=float(np.median(dy)), bias_pitch=float(np.median(dp)),
        sigma_yaw=sy, sigma_pitch=sp,
        range_med=float(np.median(rng)), elev_med=float(np.median(elev)),
        zoom_med=float(np.median(zoom)),
        sigma_px_yaw=float(_robust_sigma(np.radians(dy) * fx)),
        sigma_px_pitch=float(_robust_sigma(np.radians(dp) * fy)),
        d_yaw=dy, d_pitch=dp, rng=rng, zoom=zoom, t=t)


def rig_geometry(win: Window) -> Dict[str, float]:
    """Baseline vector and the depth/cross-LOS error scaling it implies."""
    s1 = win.cam1.station_position()
    s2 = win.cam2.station_position()
    frame = EnuFrame(*s1)
    b = frame.to_enu(*s2)
    blen = float(np.linalg.norm(b))
    az = (math.degrees(math.atan2(b[0], b[1])) + 360.0) % 360.0
    return {"baseline_m": blen, "baseline_az_deg": az,
            "baseline_up_m": float(b[2]),
            "b_e": float(b[0]), "b_n": float(b[1]),
            "cam1": s1, "cam2": s2}


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Characterise the real VSL rig")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--min-dur", type=float, default=300.0)
    args = ap.parse_args()

    print("=" * 78)
    print("TRUTH-COVERED WINDOWS")
    print("=" * 78)
    wins = [w for w in build_windows(args.root) if w.duration >= args.min_dur]

    for w in wins:
        geo = rig_geometry(w)
        print()
        print("=" * 78)
        print(f"WINDOW {w.label}  dur={w.duration:.0f}s  "
              f"truth={os.path.basename(w.truth.path)}  ov={100*w.overlap:.0f}%")
        print("=" * 78)
        print(f"  CAM1 station {geo['cam1']}")
        print(f"  CAM2 station {geo['cam2']}")
        print(f"  baseline {geo['baseline_m']:.1f} m  az={geo['baseline_az_deg']:.1f} deg"
              f"  dUp={geo['baseline_up_m']:+.1f} m")

        frame = EnuFrame(*geo["cam1"])
        for cam, station in ((w.cam1, geo["cam1"]), (w.cam2, geo["cam2"])):
            rs = characterise(cam, w.truth, frame, station)
            if rs is None:
                print(f"  {cam.cam_id}: insufficient locked+covered rows")
                continue
            print(f"  {rs.cam_id}: n={rs.n:6d} latency={rs.latency:+.2f}s "
                  f"R_med={rs.range_med:6.0f}m elev={rs.elev_med:+5.1f}deg "
                  f"zoom={rs.zoom_med:.1f}")
            print(f"        bias  yaw={rs.bias_yaw:+.4f} deg  pitch={rs.bias_pitch:+.4f} deg")
            print(f"        sigma yaw={rs.sigma_yaw:.4f} deg  pitch={rs.sigma_pitch:.4f} deg"
                  f"   ({rs.sigma_px_yaw:.2f} / {rs.sigma_px_pitch:.2f} px)")
            depth_err = geo["baseline_m"] and (rs.range_med ** 2 / geo["baseline_m"]
                                               * math.radians(rs.sigma_yaw))
            cross_err = rs.range_med * math.radians(rs.sigma_yaw)
            print(f"        predicted per-frame: cross-LOS {cross_err:.1f} m, "
                  f"depth {depth_err:.0f} m")
