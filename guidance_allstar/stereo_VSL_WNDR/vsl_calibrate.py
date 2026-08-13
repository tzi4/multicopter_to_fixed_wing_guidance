"""Recover the real per-camera systematics of the VSL rig from target truth.

The raw logs do not point where they claim. Measured against the target's own
flight-controller log, ray-to-truth separation runs from ~1.5 deg (best case) to
tens of degrees, so before any estimator is scored the systematic part has to be
identified and removed. This module fits, per camera and per flight window:

    d_yaw    additive azimuth offset  [deg]   (residual boresight)
    d_pitch  additive elevation offset [deg]  (residual boresight)
    d_alt    station altitude correction [m]
    latency  image age behind the logged timestamp [s]

``d_pitch`` and ``d_alt`` are only separable when the target's range varies
across the window (an altitude error is a range-dependent elevation error, a
boresight is not). ``fit_camera`` reports the range spread and the fitted
correlation so an ill-conditioned split is visible rather than silently trusted.

Robustness matters more than usual here: the detector spends much of each window
locked onto something that is not the truth aircraft (birds, other traffic,
ground clutter), so a plain least-squares fit is meaningless. The solver is
iteratively-trimmed -- fit, re-gate on residual, refit -- which converges onto
the subset that really is the aircraft.

Nothing in this module is used at flight time; it is an offline instrument for
characterising the sensor. The estimator itself never sees truth.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from vsl_ingest import (CamStream, EnuFrame, enu_to_heading_pitch, vsl_intrinsics,
                        vsl_ray_enu, wrap180)
from vsl_truth import TruthTrack

MIN_SUPPORT = 0.15   # a fit standing on less than this is a coincidence
BORESIGHT_JSON = os.environ.get(
    "VSL_BORESIGHT_JSON",
    os.path.expanduser(
        "~/savasan_iha_yildizlar_data/VSL/STEREO-boresight/boresight_offsets.json"),
)


class BoresightTable:
    """VSL's per-zoom boresight offsets (boresight_offsets.json).

    Their pipeline runs with ``boresight=1``, so reproducing what their
    triangulator saw means applying this table. Offsets are interpolated in
    zoom, matching ``mono_hub._boresight_correction_for_zoom`` closely enough
    for characterisation (they snap to the nearest logged zoom key).
    """

    def __init__(self, path: str = BORESIGHT_JSON):
        self.path = path
        self.ok = False
        self._z: Dict[str, np.ndarray] = {}
        self._y: Dict[str, np.ndarray] = {}
        self._p: Dict[str, np.ndarray] = {}
        try:
            blob = json.load(open(path))
        except Exception:
            return
        for cam in ("CAM1", "CAM2"):
            rec = blob.get("records", {}).get(cam, {})
            if not rec:
                continue
            keys = sorted(rec, key=float)
            self._z[cam] = np.array([float(k) for k in keys])
            self._y[cam] = np.array([float(rec[k]["yaw_offset_deg"]) for k in keys])
            self._p[cam] = np.array([float(rec[k]["pitch_offset_deg"]) for k in keys])
        self.ok = bool(self._z)
        self.calib_heading = {
            cam: float(next(iter(blob["records"][cam].values()))["calib_heading_deg"])
            for cam in self._z
        }

    def offsets(self, cam_id: str, zoom: float) -> Tuple[float, float]:
        if cam_id not in self._z:
            return 0.0, 0.0
        z = float(zoom)
        return (float(np.interp(z, self._z[cam_id], self._y[cam_id])),
                float(np.interp(z, self._z[cam_id], self._p[cam_id])))

    def offsets_array(self, cam_id: str, zoom: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if cam_id not in self._z:
            return np.zeros_like(zoom), np.zeros_like(zoom)
        return (np.interp(zoom, self._z[cam_id], self._y[cam_id]),
                np.interp(zoom, self._z[cam_id], self._p[cam_id]))


@dataclass
class CamCalibration:
    """Fitted systematics for one camera over one window."""

    cam_id: str
    d_yaw: float
    d_pitch: float
    d_alt: float
    latency: float
    sigma_yaw: float          # residual scatter after the fit [deg]
    sigma_pitch: float
    sigma_px_yaw: float
    sigma_px_pitch: float
    n_used: int
    n_total: int
    range_med: float
    range_iqr: float
    alt_pitch_corr: float     # |correlation| of the d_alt / d_pitch columns
    use_table: bool

    @property
    def inlier_frac(self) -> float:
        return self.n_used / max(1, self.n_total)

    @property
    def suspect(self) -> List[str]:
        """Why this fit should not be believed, if anything.

        The mode-seeking bootstrap can recover an arbitrarily large offset, which
        is what makes badly-aimed windows usable at all -- but it also means a
        bad window now returns a confident number instead of failing loudly. So
        every fit is graded: a real mount boresight is a few degrees, a detector
        that resolves the target is well under a degree, and a fit standing on a
        small minority of frames is a coincidence, not a calibration.
        """
        why = []
        big = max(abs(self.d_yaw), abs(self.d_pitch))
        # A large offset is only suspicious when little data backs it. The rig is
        # a fixed ground mount, so its boresight cannot wander -- but the
        # hand-entered Gimbal_Heading that the offset is measured against is
        # re-typed at every deployment, and a wrong entry there shows up as a
        # large, WELL-SUPPORTED offset. That is a real finding, not a bad fit.
        # A large offset on a handful of frames is the opposite: the fitter
        # aligning wrong-object rays to truth.
        if big > 5.0 and self.inlier_frac < 0.40:
            why.append(f"{big:.1f}deg offset on only {100*self.inlier_frac:.0f}% "
                       f"of frames -- fit is aligning wrong-object rays, not a boresight")
        if max(self.sigma_yaw, self.sigma_pitch) > 1.5:
            why.append(f"residual sigma {max(self.sigma_yaw, self.sigma_pitch):.2f}deg "
                       f"too large for a working detector")
        if self.inlier_frac < 0.15:
            why.append(f"only {100*self.inlier_frac:.0f}% of frames support it")
        return why

    @property
    def heading_error_deg(self) -> Optional[float]:
        """Large, well-supported yaw offset = the entered mount heading is wrong."""
        if abs(self.d_yaw) > 5.0 and self.inlier_frac >= 0.40 and self.sigma_yaw < 1.5:
            return self.d_yaw
        return None

    def describe(self) -> str:
        return (f"{self.cam_id}: dYaw={self.d_yaw:+.3f} dPitch={self.d_pitch:+.3f} deg  "
                f"dAlt={self.d_alt:+.1f} m  lat={self.latency:.2f} s  "
                f"sigma={self.sigma_yaw:.3f}/{self.sigma_pitch:.3f} deg "
                f"({self.sigma_px_yaw:.1f}/{self.sigma_px_pitch:.1f} px)  "
                f"used {self.n_used}/{self.n_total} ({100*self.inlier_frac:.0f}%)")


# ---------------------------------------------------------------------------
# geometry helpers (vectorised where it matters)
# ---------------------------------------------------------------------------

def build_rays(cam: CamStream, idx: np.ndarray, table: Optional[BoresightTable],
               d_yaw: float = 0.0, d_pitch: float = 0.0) -> np.ndarray:
    """Unit ENU rays for rows ``idx``, with table + delta corrections applied."""
    out = np.full((idx.size, 3), np.nan)
    if table is not None and table.ok:
        by, bp = table.offsets_array(cam.cam_id, cam.zoom[idx])
    else:
        by = np.zeros(idx.size)
        bp = np.zeros(idx.size)
    for k, i in enumerate(idx):
        try:
            out[k] = vsl_ray_enu(cam.u[i], cam.v[i], cam.zoom[i],
                                 cam.elevation[i] + bp[k] + d_pitch,
                                 cam.roll[i],
                                 cam.yaw_world[i] + by[k] + d_yaw,
                                 cam.cam_id)
        except ValueError:
            continue
    return out


def _angular_residual(rays: np.ndarray, tgt_enu: np.ndarray, station_enu: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(d_yaw*cos(elev), d_pitch, range) between rays and bearings to targets."""
    vec = tgt_enu - station_enu[None, :]
    rng = np.linalg.norm(vec, axis=1)
    ok = rng > 1.0
    unit = np.zeros_like(vec)
    unit[ok] = vec[ok] / rng[ok, None]
    h_t = np.degrees(np.arctan2(unit[:, 0], unit[:, 1]))
    p_t = np.degrees(np.arctan2(unit[:, 2], np.hypot(unit[:, 0], unit[:, 1])))
    h_m = np.degrees(np.arctan2(rays[:, 0], rays[:, 1]))
    p_m = np.degrees(np.arctan2(rays[:, 2], np.hypot(rays[:, 0], rays[:, 1])))
    dy = np.array([wrap180(a) for a in (h_m - h_t)]) * np.cos(np.radians(p_t))
    dp = p_m - p_t
    dy[~ok] = np.nan
    dp[~ok] = np.nan
    return dy, dp, rng


def _robust_sigma(x: np.ndarray) -> float:
    if x.size == 0:
        return math.nan
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


# ---------------------------------------------------------------------------
# the fit
# ---------------------------------------------------------------------------

def _mode_seek(dy: np.ndarray, dp: np.ndarray, span: float = 90.0,
               bin_deg: float = 1.0) -> Tuple[float, float]:
    """Coarse (d_yaw, d_pitch) from the densest bin of the residual cloud.

    Needed because a trimmed fit cannot bootstrap itself: if a camera carries a
    large offset (the 23 July CAM1 is ~37 deg out) then EVERY sample sits
    outside a few-degree gate on the first iteration and the fit reports failure
    on data that is actually fine. A 2-D histogram peak survives an inlier
    fraction well under half, which a median would not.
    """
    ok = (np.isfinite(dy) & np.isfinite(dp)
          & (np.abs(dy) < span) & (np.abs(dp) < span))
    if ok.sum() < 30:
        return 0.0, 0.0
    edges = np.arange(-span, span + bin_deg, bin_deg)
    hist, xe, ye = np.histogram2d(dy[ok], dp[ok], bins=[edges, edges])
    i, j = np.unravel_index(int(np.argmax(hist)), hist.shape)
    return 0.5 * (xe[i] + xe[i + 1]), 0.5 * (ye[j] + ye[j + 1])


def fit_camera(cam: CamStream, truth: TruthTrack, frame: EnuFrame,
               station: Sequence[float], table: Optional[BoresightTable],
               latency_grid: Sequence[float] = tuple(np.arange(0.0, 0.85, 0.05)),
               gate_schedule: Sequence[float] = (20.0, 8.0, 4.0, 2.0, 1.2),
               subsample: int = 1) -> Optional[CamCalibration]:
    """Iteratively-trimmed fit of (d_yaw, d_pitch, d_alt, latency).

    Bootstrapped by a mode-seeking pass so an arbitrarily large boresight is
    recoverable; the trim schedule then tightens onto the target aircraft.
    """
    m = cam.locked & np.isfinite(cam.u) & np.isfinite(cam.v) & np.isfinite(cam.zoom)
    m &= np.isfinite(cam.elevation) & np.isfinite(cam.yaw_world)
    if subsample > 1:
        m &= (np.arange(len(cam)) % subsample == 0)
    idx_all = np.flatnonzero(m)
    if idx_all.size < 60:
        return None

    station_enu0 = frame.to_enu(*station)
    best: Optional[Tuple[float, CamCalibration]] = None

    for latency in latency_grid:
        cov = truth.covers(cam.t[idx_all] - latency)
        idx = idx_all[cov]
        if idx.size < 60:
            continue
        lat_t, lon_t, alt_t = truth.sample(cam.t[idx] - latency)
        tgt = np.stack([frame.to_enu(a, b, c) for a, b, c in zip(lat_t, lon_t, alt_t)])

        # Coarse bootstrap before any trimming (see _mode_seek).
        rays0 = build_rays(cam, idx, table, 0.0, 0.0)
        dy0, dp0, _ = _angular_residual(rays0, tgt, station_enu0)
        m_dy, m_dp = _mode_seek(dy0, dp0)
        d_yaw, d_pitch, d_alt = -m_dy, -m_dp, 0.0

        keep = np.ones(idx.size, dtype=bool)
        corr = math.nan
        for gate in gate_schedule:
            for _ in range(3):
                rays = build_rays(cam, idx, table, d_yaw, d_pitch)
                st = station_enu0 + np.array([0.0, 0.0, d_alt])
                dy, dp, rng = _angular_residual(rays, tgt, st)
                good = keep & np.isfinite(dy) & (np.hypot(dy, dp) < gate)
                if good.sum() < 40:
                    break
                # Jacobian of (dy, dp) wrt (d_yaw, d_pitch, d_alt).
                # d_yaw/d_pitch enter the ray directly (unit slope); d_alt moves
                # the station, changing elevation by -cos^2(elev)/R per metre.
                g = np.flatnonzero(good)
                elev = np.radians(np.degrees(np.arctan2(
                    (tgt[g, 2] - st[2]), np.hypot(tgt[g, 0] - st[0], tgt[g, 1] - st[1]))))
                dpitch_dalt = np.degrees(-np.cos(elev) ** 2 / np.maximum(rng[g], 1.0))
                rows = 2 * g.size
                A = np.zeros((rows, 3))
                r = np.zeros(rows)
                A[0::2, 0] = 1.0                 # dy wrt d_yaw
                A[1::2, 1] = 1.0                 # dp wrt d_pitch
                A[1::2, 2] = -dpitch_dalt        # dp wrt d_alt (residual = meas - model)
                r[0::2] = dy[g]
                r[1::2] = dp[g]
                # Huber weights on the stacked residual
                s = max(1e-3, _robust_sigma(r))
                w = np.minimum(1.0, 2.0 * s / np.maximum(np.abs(r), 1e-9))
                Aw = A * w[:, None]
                rw = r * w
                try:
                    step, *_ = np.linalg.lstsq(Aw, rw, rcond=None)
                except np.linalg.LinAlgError:
                    break
                d_yaw -= float(step[0])
                d_pitch -= float(step[1])
                d_alt -= float(step[2])
                d_alt = float(np.clip(d_alt, -200.0, 200.0))
                keep = good
                try:
                    ata = Aw.T @ Aw
                    inv = np.linalg.inv(ata)
                    corr = abs(float(inv[1, 2] / math.sqrt(max(1e-18, inv[1, 1] * inv[2, 2]))))
                except np.linalg.LinAlgError:
                    corr = math.nan

        rays = build_rays(cam, idx, table, d_yaw, d_pitch)
        st = station_enu0 + np.array([0.0, 0.0, d_alt])
        dy, dp, rng = _angular_residual(rays, tgt, st)
        # Final inlier set by iterative sigma-clipping. A fixed gate is not good
        # enough here: if it stays loose the reported sigma absorbs the
        # not-the-target detections, and that inflated sigma then becomes the
        # filter's R -- which is exactly the mistuning that a covariance gate
        # cannot recover from.
        fin = np.isfinite(dy) & np.isfinite(dp)
        sep = np.hypot(dy, dp)
        inl = fin & (sep < 2.0)
        if inl.sum() < 40:
            inl = fin & (sep < 5.0)
        for _ in range(6):
            if inl.sum() < 40:
                break
            s = max(0.05, math.hypot(_robust_sigma(dy[inl]), _robust_sigma(dp[inl])))
            centre = math.hypot(np.median(dy[inl]), np.median(dp[inl]))
            new = fin & (sep < centre + 3.0 * s)
            if new.sum() < 40 or int(new.sum()) == int(inl.sum()):
                inl = new if new.sum() >= 40 else inl
                break
            inl = new
        if inl.sum() < 40:
            continue
        sy, sp = _robust_sigma(dy[inl]), _robust_sigma(dp[inl])
        # Support enters LINEARLY, not as a square root. A spurious mode -- the
        # fitter aligning wrong-object rays -- is typically very tight (those
        # frames really are consistent with each other) but stands on a handful
        # of frames, so a sqrt penalty lets it beat the honest solution. Seen
        # for real: pooling first halves at the 21 July site picked -65.2 deg at
        # 5 % support over +2.0 deg at 36 %.
        cost = math.hypot(sy, sp) / max(1e-6, inl.mean())
        fx = np.array([vsl_intrinsics(cam.cam_id, float(z))[0] for z in cam.zoom[idx][inl]])
        fy = np.array([vsl_intrinsics(cam.cam_id, float(z))[1] for z in cam.zoom[idx][inl]])
        cal = CamCalibration(
            cam_id=cam.cam_id, d_yaw=d_yaw, d_pitch=d_pitch, d_alt=d_alt,
            latency=float(latency), sigma_yaw=sy, sigma_pitch=sp,
            sigma_px_yaw=_robust_sigma(np.radians(dy[inl]) * fx),
            sigma_px_pitch=_robust_sigma(np.radians(dp[inl]) * fy),
            n_used=int(inl.sum()), n_total=int(idx.size),
            range_med=float(np.median(rng[inl])),
            range_iqr=float(np.percentile(rng[inl], 75) - np.percentile(rng[inl], 25)),
            alt_pitch_corr=corr, use_table=bool(table and table.ok))
        if best is None or cost < best[0]:
            best = (cost, cal)

    return best[1] if best else None


def fit_site(segments: Sequence[Tuple[CamStream, TruthTrack]], frame: EnuFrame,
             station: Sequence[float], table: Optional[BoresightTable],
             latency_grid: Sequence[float] = tuple(np.arange(0.0, 0.85, 0.05)),
             subsample: int = 1) -> Optional[CamCalibration]:
    """Fit ONE boresight for a camera across every window at a site.

    The rig is a camera on a fixed ground mount, so its boresight is a property
    of the site, not of the window. Fitting per window is not just wasteful, it
    is actively harmful: a window in which the detector spends most of its time
    on the wrong object offers no honest solution, and a per-window fit will
    happily invent a large offset that aligns those wrong-object rays to truth.
    That is where the 13-65 deg "offsets" came from -- every one of them stood on
    under 15% of frames, while the windows with real support at the same site
    agreed to a few tenths of a degree.

    Pooling makes the good windows outvote the bad ones, so one honest number
    falls out for the whole deployment.
    """
    pooled: List[Tuple[CamStream, np.ndarray, np.ndarray]] = []
    cam_id = segments[0][0].cam_id if segments else "CAM?"
    station_enu0 = frame.to_enu(*station)
    best: Optional[Tuple[float, CamCalibration]] = None

    for latency in latency_grid:
        pooled.clear()
        for cam, truth in segments:
            m = cam.locked & np.isfinite(cam.u) & np.isfinite(cam.v) & np.isfinite(cam.zoom)
            m &= np.isfinite(cam.elevation) & np.isfinite(cam.yaw_world)
            if subsample > 1:
                m &= (np.arange(len(cam)) % subsample == 0)
            m &= truth.covers(cam.t - latency)
            idx = np.flatnonzero(m)
            if idx.size < 30:
                continue
            lat_t, lon_t, alt_t = truth.sample(cam.t[idx] - latency)
            tgt = np.stack([frame.to_enu(a, b, c)
                            for a, b, c in zip(lat_t, lon_t, alt_t)])
            pooled.append((cam, idx, tgt))
        if not pooled:
            continue

        def residuals(d_yaw, d_pitch, d_alt):
            dys, dps, rngs, zooms = [], [], [], []
            st = station_enu0 + np.array([0.0, 0.0, d_alt])
            for cam, idx, tgt in pooled:
                rays = build_rays(cam, idx, table, d_yaw, d_pitch)
                dy, dp, rng = _angular_residual(rays, tgt, st)
                dys.append(dy); dps.append(dp); rngs.append(rng)
                zooms.append(cam.zoom[idx])
            return (np.concatenate(dys), np.concatenate(dps),
                    np.concatenate(rngs), np.concatenate(zooms))

        dy0, dp0, _, _ = residuals(0.0, 0.0, 0.0)
        m_dy, m_dp = _mode_seek(dy0, dp0)
        d_yaw, d_pitch, d_alt = -m_dy, -m_dp, 0.0
        for gate in (20.0, 8.0, 4.0, 2.0, 1.2):
            for _ in range(3):
                dy, dp, rng, _ = residuals(d_yaw, d_pitch, d_alt)
                good = np.isfinite(dy) & (np.hypot(dy, dp) < gate)
                if good.sum() < 60:
                    break
                d_yaw -= float(np.median(dy[good]))
                d_pitch -= float(np.median(dp[good]))

        dy, dp, rng, zoom = residuals(d_yaw, d_pitch, d_alt)
        fin = np.isfinite(dy) & np.isfinite(dp)
        sep = np.hypot(dy, dp)
        inl = fin & (sep < 2.0)
        for _ in range(6):
            if inl.sum() < 60:
                break
            s = max(0.05, math.hypot(_robust_sigma(dy[inl]), _robust_sigma(dp[inl])))
            new = fin & (sep < math.hypot(np.median(dy[inl]), np.median(dp[inl])) + 3.0 * s)
            if new.sum() < 60 or int(new.sum()) == int(inl.sum()):
                break
            inl = new
        if inl.sum() < 60:
            continue
        sy, sp = _robust_sigma(dy[inl]), _robust_sigma(dp[inl])
        # Support enters LINEARLY, not as a square root. A spurious mode -- the
        # fitter aligning wrong-object rays -- is typically very tight (those
        # frames really are consistent with each other) but stands on a handful
        # of frames, so a sqrt penalty lets it beat the honest solution. Seen
        # for real: pooling first halves at the 21 July site picked -65.2 deg at
        # 5 % support over +2.0 deg at 36 %.
        cost = math.hypot(sy, sp) / max(1e-6, inl.mean())
        fx = np.array([vsl_intrinsics(cam_id, float(z))[0] for z in zoom[inl]])
        fy = np.array([vsl_intrinsics(cam_id, float(z))[1] for z in zoom[inl]])
        cal = CamCalibration(
            cam_id=cam_id, d_yaw=d_yaw, d_pitch=d_pitch, d_alt=d_alt,
            latency=float(latency), sigma_yaw=sy, sigma_pitch=sp,
            sigma_px_yaw=_robust_sigma(np.radians(dy[inl]) * fx),
            sigma_px_pitch=_robust_sigma(np.radians(dp[inl]) * fy),
            n_used=int(inl.sum()), n_total=int(dy.size),
            range_med=float(np.median(rng[inl])),
            range_iqr=float(np.percentile(rng[inl], 75) - np.percentile(rng[inl], 25)),
            alt_pitch_corr=math.nan, use_table=bool(table and table.ok))
        # Never let a latency whose solution is unsupported win outright: prefer
        # any candidate clearing MIN_SUPPORT, and only fall back below it when
        # nothing else exists.
        supported = cal.inlier_frac >= MIN_SUPPORT
        if best is None:
            best = (cost, cal, supported)
        elif supported and not best[2]:
            best = (cost, cal, supported)
        elif supported == best[2] and cost < best[0]:
            best = (cost, cal, supported)
    return best[1] if best else None


def apply_calibration(cam: CamStream, idx: np.ndarray, table: Optional[BoresightTable],
                      cal: CamCalibration) -> np.ndarray:
    return build_rays(cam, idx, table, cal.d_yaw, cal.d_pitch)


def station_enu(frame: EnuFrame, station: Sequence[float], cal: CamCalibration) -> np.ndarray:
    return frame.to_enu(*station) + np.array([0.0, 0.0, cal.d_alt])
