"""What the estimator actually consumes: ray angles and their real statistics.

The filter does not see cameras, pixels or calibration -- it sees four angles
per frame and an R matrix. This module measures the properties of those angles
that decide whether R is right:

1. **Magnitude** -- sigma per axis, per camera, in degrees AND pixels. Which of
   the two is constant tells you the error's origin: constant pixel error means
   detector-limited (R must scale with FOV/zoom), constant angular error means
   gimbal/attitude-limited (R is zoom-independent). NOTE: VSL will start sending
   a per-frame variance over the link, which supersedes the zoom model fitted
   here -- but only for the INSTANTANEOUS part. A per-frame sigma cannot express
   point 2 below, and point 2 is the larger error.

2. **Whiteness** -- the autocorrelation of the residual sequence. This is the
   property that R tuning usually gets wrong. A Kalman filter assumes successive
   measurements are independent; if they are correlated, feeding them at full
   rate makes the filter believe it has N independent looks when it has far
   fewer, and it becomes overconfident no matter how carefully sigma itself was
   measured. VSL already documented this in selfcal_ekf.py ("Inovasyonlar beyaz
   DEGIL; 9-10 Hz'de lag-1 otokor ~+0.92") and handle it by thinning to 3 Hz.

3. **Shape** -- Gaussian core vs heavy tail, which decides whether the NIS gate
   should reject or inflate.

4. **Skew** -- the truth-free consistency check between the two rays, and how it
   relates to the actual position error. Skew is the only one of these the
   system can measure in flight, so its relationship to real error is what makes
   it usable as a quality signal.

Everything is measured on the residual between the delivered ray and the true
bearing to the target, on the frames where the detector really is on the target.
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import stereo_geometry as sg
from vsl_calibrate import (BoresightTable, CamCalibration, _angular_residual,
                           build_rays, fit_site)
from vsl_eval import Window, build_windows, rig_geometry
from vsl_ingest import EnuFrame, vsl_intrinsics
from vsl_run import enu_to_ned


def robust_sigma(x: np.ndarray) -> float:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return math.nan
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


@dataclass
class RaySamples:
    """Per-frame residuals for one camera, on frames that are on the target."""

    cam_id: str
    t: np.ndarray
    d_yaw: np.ndarray        # deg, cos(elev)-scaled (true angular miss)
    d_pitch: np.ndarray      # deg
    zoom: np.ndarray
    rng: np.ndarray
    fx: np.ndarray
    fy: np.ndarray

    def __len__(self) -> int:
        return int(self.t.size)


def collect(cam, truth, frame: EnuFrame, station, table, cal: CamCalibration,
            gate_deg: float = 2.0) -> RaySamples:
    m = cam.locked & np.isfinite(cam.u) & np.isfinite(cam.zoom) & np.isfinite(cam.elevation)
    m &= truth.covers(cam.t - cal.latency)
    idx = np.flatnonzero(m)
    if idx.size == 0:
        z = np.zeros(0)
        return RaySamples(cam.cam_id, z, z, z, z, z, z, z)
    lat_t, lon_t, alt_t = truth.sample(cam.t[idx] - cal.latency)
    tgt = np.stack([frame.to_enu(a, b, c) for a, b, c in zip(lat_t, lon_t, alt_t)])
    st = frame.to_enu(*station) + np.array([0.0, 0.0, cal.d_alt])
    rays = build_rays(cam, idx, table, cal.d_yaw, cal.d_pitch)
    dy, dp, rng = _angular_residual(rays, tgt, st)
    ok = np.isfinite(dy) & np.isfinite(dp) & (np.hypot(dy, dp) < gate_deg)
    idx = idx[ok]
    fx = np.array([vsl_intrinsics(cam.cam_id, float(z))[0] for z in cam.zoom[idx]])
    fy = np.array([vsl_intrinsics(cam.cam_id, float(z))[1] for z in cam.zoom[idx]])
    return RaySamples(cam.cam_id, cam.t[idx] - cal.latency, dy[ok], dp[ok],
                      cam.zoom[idx], rng[ok], fx, fy)


def autocorr(t: np.ndarray, x: np.ndarray, lags_s: Sequence[float],
             tol: float = 0.02) -> Dict[float, Tuple[float, int]]:
    """Autocorrelation of an irregularly-sampled residual at given time lags.

    Pairs samples separated by ``lag`` within ``tol`` rather than assuming a
    fixed rate -- the camera stream drops frames, so index-based lags would mix
    different real time separations.
    """
    out: Dict[float, Tuple[float, int]] = {}
    if x.size < 50:
        return out
    xc = x - np.mean(x)
    var = float(np.mean(xc * xc))
    if var <= 0:
        return out
    order = np.argsort(t)
    ts, xs = t[order], xc[order]
    for lag in lags_s:
        j = np.searchsorted(ts, ts + lag)
        j = np.clip(j, 0, ts.size - 1)
        good = np.abs(ts[j] - (ts + lag)) <= tol
        if good.sum() < 30:
            continue
        a, b = xs[good], xs[j[good]]
        out[float(lag)] = (float(np.mean(a * b) / var), int(good.sum()))
    return out


def effective_rate(rho1: float, nominal_hz: float) -> float:
    """Independent-sample rate implied by a lag-1 correlation.

    For an AR(1) sequence the variance of a mean over N samples is inflated by
    (1+rho)/(1-rho); the filter's effective information rate is reduced by the
    same factor. This is the number that says how hard to thin, or equivalently
    how much to inflate R if you refuse to.
    """
    if not (-1.0 < rho1 < 1.0):
        return nominal_hz
    return nominal_hz * (1.0 - rho1) / (1.0 + rho1)


def describe(rs: RaySamples, nominal_hz: float) -> Dict[str, float]:
    d: Dict[str, float] = {"n": float(len(rs))}
    if len(rs) < 50:
        return d
    d["sigma_yaw_deg"] = robust_sigma(rs.d_yaw)
    d["sigma_pitch_deg"] = robust_sigma(rs.d_pitch)
    d["sigma_yaw_px"] = robust_sigma(np.radians(rs.d_yaw) * rs.fx)
    d["sigma_pitch_px"] = robust_sigma(np.radians(rs.d_pitch) * rs.fy)
    # tail: ratio of the 99th percentile to what a Gaussian would give (2.58 sigma)
    for name, v in (("yaw", rs.d_yaw), ("pitch", rs.d_pitch)):
        s = robust_sigma(v)
        c = np.median(v)
        if s > 0:
            d[f"tail_{name}"] = float(np.percentile(np.abs(v - c), 99) / (2.58 * s))
    ac_y = autocorr(rs.t, rs.d_yaw, [0.05, 0.1, 0.2, 0.5, 1.0])
    ac_p = autocorr(rs.t, rs.d_pitch, [0.05, 0.1, 0.2, 0.5, 1.0])
    for lag, (r, _) in ac_y.items():
        d[f"acf_yaw_{lag}"] = r
    for lag, (r, _) in ac_p.items():
        d[f"acf_pitch_{lag}"] = r
    lag1 = min(ac_y) if ac_y else None
    if lag1 is not None:
        rho = 0.5 * (ac_y[lag1][0] + ac_p.get(lag1, (ac_y[lag1][0], 0))[0])
        d["rho_lag1"] = rho
        d["eff_hz"] = effective_rate(rho, nominal_hz)
    return d


def zoom_split(rs: RaySamples) -> List[Tuple[float, float, float, float, float, int]]:
    """sigma in deg and px within zoom bands -- which one is constant?"""
    bands = [(1, 3), (3, 6), (6, 12), (12, 30)]
    out = []
    for lo, hi in bands:
        m = (rs.zoom >= lo) & (rs.zoom < hi)
        if m.sum() < 50:
            continue
        out.append((lo, hi,
                    robust_sigma(rs.d_yaw[m]), robust_sigma(rs.d_pitch[m]),
                    robust_sigma(np.radians(rs.d_yaw[m]) * rs.fx[m]), int(m.sum())))
    return out


def skew_vs_error(win: Window, cals, table, frame: EnuFrame, geo) -> Dict[str, float]:
    """Is ray skew a usable in-flight proxy for position error?"""
    from vsl_run import associate
    frames = associate(win, cals[0], cals[1], table, frame, win.t0, win.t1)
    cams = []
    for cal, key in ((cals[0], "cam1"), (cals[1], "cam2")):
        p = frame.to_enu(*geo[key]) + np.array([0.0, 0.0, cal.d_alt])
        cams.append(sg.Camera(cal.cam_id, enu_to_ned(p), 0, 0, 0,
                              sigma_yaw_deg=max(0.02, cal.sigma_yaw),
                              sigma_pitch_deg=max(0.02, cal.sigma_pitch),
                              fov_yaw_deg=359, fov_pitch_deg=179, max_range_m=20000))
    skew, err = [], []
    for fr in frames:
        if fr.ray1 is None or fr.ray2 is None:
            continue
        from vsl_ingest import enu_to_heading_pitch
        h1, p1 = enu_to_heading_pitch(fr.ray1)
        h2, p2 = enu_to_heading_pitch(fr.ray2)
        d1 = sg.Detection(0, math.radians(h1), math.radians(p1), fr.stamp)
        d2 = sg.Detection(1, math.radians(h2), math.radians(p2), fr.stamp)
        fix, info = sg.triangulate_midpoint(cams[0], d1, cams[1], d2)
        if not np.all(np.isfinite(fix)):
            continue
        lat_t, lon_t, alt_t = win.truth.sample([fr.stamp])
        tn = enu_to_ned(frame.to_enu(lat_t[0], lon_t[0], alt_t[0]))
        skew.append(float(info["skew_m"]))
        err.append(float(np.linalg.norm(fix - tn)))
    if len(skew) < 50:
        return {}
    skew = np.array(skew)
    err = np.array(err)
    out = {"n": float(skew.size), "skew_med": float(np.median(skew))}
    for lo, hi in ((0, 2), (2, 5), (5, 10), (10, 30), (30, 1e9)):
        m = (skew >= lo) & (skew < hi)
        if m.sum() >= 20:
            out[f"err_med_skew_{lo}_{hi if hi < 1e9 else 'inf'}"] = float(np.median(err[m]))
            out[f"n_skew_{lo}_{hi if hi < 1e9 else 'inf'}"] = float(m.sum())
    good = skew < 10.0
    if good.sum() > 20 and (~good).sum() > 20:
        out["err_med_skew_lt10"] = float(np.median(err[good]))
        out["err_med_skew_ge10"] = float(np.median(err[~good]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", required=True,
                    help="'lat,lon' to 4 dp selecting ONE deployment. The 21 and "
                         "27 July rigs share a latitude but sit ~8 m apart in "
                         "longitude and were aimed differently, so a latitude-only "
                         "match silently pools two deployments.")
    ap.add_argument("--min-dur", type=float, default=300.0)
    ap.add_argument("--subsample", type=int, default=2)
    args = ap.parse_args()

    table = BoresightTable()
    wins = [w for w in build_windows(verbose=False) if w.duration >= args.min_dur]
    def site_of(w):
        s1 = w.cam1.station_position()
        return f"{s1[0]:.4f},{s1[1]:.4f}"

    wins = [w for w in wins if site_of(w) == args.site]
    if not wins:
        print("no windows at that site")
        return
    geo = rig_geometry(wins[0])
    frame = EnuFrame(*geo["cam1"])
    print(f"site {geo['cam1'][0]:.4f},{geo['cam1'][1]:.4f}  baseline "
          f"{geo['baseline_m']:.1f} m  windows={len(wins)}")

    cals = []
    for which, key in (("cam1", "cam1"), ("cam2", "cam2")):
        segs = [(getattr(w, which), w.truth) for w in wins]
        cal = fit_site(segs, frame, geo[key], table, subsample=args.subsample)
        if cal is None:
            print(f"{which}: site calibration failed")
            return
        cals.append(cal)
        print(f"  {cal.describe()}")

    print(f"\n{'='*78}\nWHAT THE FILTER IS FED (residual ray angle vs truth)\n{'='*78}")
    allsamp = {}
    for cal, which, key in ((cals[0], "cam1", "cam1"), (cals[1], "cam2", "cam2")):
        parts = [collect(getattr(w, which), w.truth, frame, geo[key], table, cal)
                 for w in wins]
        rs = RaySamples(cal.cam_id,
                        np.concatenate([p.t for p in parts]),
                        np.concatenate([p.d_yaw for p in parts]),
                        np.concatenate([p.d_pitch for p in parts]),
                        np.concatenate([p.zoom for p in parts]),
                        np.concatenate([p.rng for p in parts]),
                        np.concatenate([p.fx for p in parts]),
                        np.concatenate([p.fy for p in parts]))
        allsamp[cal.cam_id] = rs
        d = describe(rs, nominal_hz=20.0)
        if d.get("n", 0) < 50:
            print(f"\n{cal.cam_id}: too few on-target frames ({d.get('n',0):.0f})")
            continue
        print(f"\n{cal.cam_id}: n={d['n']:.0f} on-target frames")
        print(f"  sigma      yaw {d['sigma_yaw_deg']:.3f} deg / {d['sigma_yaw_px']:.1f} px"
              f"     pitch {d['sigma_pitch_deg']:.3f} deg / {d['sigma_pitch_px']:.1f} px")
        print(f"  tail (p99 / 2.58sigma; 1.0 = Gaussian): "
              f"yaw {d.get('tail_yaw', float('nan')):.2f}  pitch {d.get('tail_pitch', float('nan')):.2f}")
        acf = [(k, v) for k, v in sorted(d.items()) if k.startswith("acf_yaw_")]
        if acf:
            print("  autocorrelation (yaw):  " +
                  "  ".join(f"{k.split('_')[-1]}s={v:+.2f}" for k, v in acf))
        acf = [(k, v) for k, v in sorted(d.items()) if k.startswith("acf_pitch_")]
        if acf:
            print("  autocorrelation (pitch):" +
                  "  ".join(f"{k.split('_')[-1]}s={v:+.2f}" for k, v in acf))
        if "rho_lag1" in d:
            print(f"  -> lag-1 rho={d['rho_lag1']:+.2f} at ~20 Hz  =>  effective "
                  f"independent rate {d['eff_hz']:.1f} Hz")
            print(f"     (feeding at 20 Hz with white-noise R over-counts information "
                  f"{20.0/max(d['eff_hz'],1e-3):.0f}x)")
        zs = zoom_split(rs)
        if zs:
            print("  sigma vs zoom (is the error detector- or gimbal-limited?)")
            for lo, hi, sy, sp, spx, n in zs:
                print(f"     zoom {lo:2d}-{hi:2d}: yaw {sy:.3f} deg  pitch {sp:.3f} deg"
                      f"   yaw {spx:6.1f} px   n={n}")

    print(f"\n{'='*78}\nSKEW AS AN IN-FLIGHT QUALITY SIGNAL\n{'='*78}")
    for w in wins:
        sv = skew_vs_error(w, cals, table, frame, geo)
        if not sv:
            continue
        bins = [(k.replace("err_med_skew_", ""), v) for k, v in sv.items()
                if k.startswith("err_med_skew_") and "lt10" not in k and "ge10" not in k]
        print(f"  {w.label}: n={sv['n']:.0f} skew_med={sv['skew_med']:.2f} m  " +
              "  ".join(f"skew{a}m->err {v:.1f}m" for a, v in bins))


if __name__ == "__main__":
    main()
