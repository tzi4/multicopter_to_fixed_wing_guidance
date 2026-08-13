"""Blind survey solve for the 25minfirst site: how much of the truth-vs-output
error is *entered geometry* rather than sensor noise?

The camera coordinates in the config are typed defaults; the boresight table
was calibrated against those coordinates; and truth showed a +81 m vertical
datum plus a range-dependent horizontal wander. This script jointly fits, over
all good windows:

    d      (3)  truth-datum shift  (site frame -> GPS frame), per flight
    e2     (3)  CAM2 position correction relative to CAM1 (baseline error)
    b1,b2  (2+2) per-camera residual boresight (yaw, pitch)
    tau    (1)  common clock offset between camera and truth time

CAM1's position is the gauge (fixed); its error is absorbed into d. The
residual after this fit is the irreducible per-frame measurement noise -- what
a properly surveyed site would deliver. Truth is used for fitting only; nothing
here feeds the tracker.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from mono_ingest import GOOD_WINDOWS, read_mono, truth_tracks, truth_enu_at


def main() -> None:
    log = read_mono(verbose=True)
    tracks = truth_tracks()
    frame = log.frame
    cam2 = np.array(frame.to_enu(*log.cam2_lla))
    c = log.cols

    pooled = np.zeros(len(log), bool)
    for a, b in GOOD_WINDOWS:
        pooled |= log.window_mask(a, b)

    # one sample per camera per row, at that camera's own capture time
    obs = []      # (cam_idx, t, heading_deg, pitch_deg, flight_idx)
    for ci, cam in enumerate(("c1", "c2")):
        t = c[f"{cam}_ts"][pooled]
        h = c[f"{cam}_head"][pooled]
        p = c[f"{cam}_pitch"][pooled]
        ok = np.isfinite(t) & np.isfinite(h) & np.isfinite(p)
        fl = np.full(t.size, -1)
        for k, tr in enumerate(tracks):
            fl[tr.covers(t)] = k
        ok &= fl >= 0
        obs.append((ci, t[ok], h[ok], p[ok], fl[ok]))

    # dense truth grid per flight for cheap shifted lookups
    grids = []
    for tr in tracks:
        g = np.arange(tr.span[0] - 1.0, tr.span[1] + 1.0, 0.02)
        enu = truth_enu_at([tr], frame, np.clip(g, tr.span[0], tr.span[1]))
        grids.append((g, enu))

    def truth_at(t, fl, tau):
        out = np.empty((t.size, 3))
        for k, (g, enu) in enumerate(grids):
            m = fl == k
            if m.any():
                for ax in range(3):
                    out[m, ax] = np.interp(t[m] + tau, g, enu[:, ax])
        return out

    def unpack(x):
        d = [x[0:3], x[3:6]]
        e2 = x[6:9]
        b = [(x[9], x[10]), (x[11], x[12])]
        tau = (x[13], x[14])
        return d, e2, b, tau

    def residuals(x):
        d, e2, b, tau = unpack(x)
        res = []
        for ci, t, h, p, fl in obs:
            pos = np.zeros(3) if ci == 0 else cam2 + e2
            tr = truth_at(t, fl, tau[ci])
            for k in (0, 1):
                tr[fl == k] -= d[k]
            v = tr - pos
            rng = np.linalg.norm(v, axis=-1)
            head_t = np.degrees(np.arctan2(v[:, 0], v[:, 1]))
            pitch_t = np.degrees(np.arcsin(np.clip(v[:, 2] / np.maximum(rng, 1e-6), -1, 1)))
            dh = (h + b[ci][0] - head_t + 180.0) % 360.0 - 180.0
            dh = dh * np.cos(np.radians(pitch_t))
            dp = p + b[ci][1] - pitch_t
            res.append(dh)
            res.append(dp)
        return np.concatenate(res)

    x0 = np.zeros(15)
    # site_truth = gps_truth - d, so d starts at the measured truth-minus-Calc
    x0[0:3] = [13.7, -4.3, 81.3]
    x0[3:6] = [8.0, -2.9, 81.8]
    r0 = residuals(x0)
    print(f"\nfit over {r0.size // 4} frames x 4 angles")
    print(f"initial (datum only): RMS {np.sqrt(np.mean(r0**2)):.3f} deg  "
          f"med|r| {np.median(np.abs(r0)):.3f} deg")

    x0[13], x0[14] = -0.17, -0.05
    sol = least_squares(residuals, x0, loss="huber", f_scale=0.3,
                        x_scale=[10.0] * 9 + [0.5] * 4 + [0.1] * 2, verbose=0)
    d, e2, b, tau = unpack(sol.x)
    r = residuals(sol.x)
    n4 = r.size // 4
    parts = {"C1 yaw": r[0:n4], "C1 pitch": r[n4:2 * n4],
             "C2 yaw": r[2 * n4:3 * n4], "C2 pitch": r[3 * n4:]}

    print("\n== solved geometry ==")
    for k, dd in enumerate(d):
        print(f"  flight{k + 1} datum d = E {dd[0]:+7.2f}  N {dd[1]:+7.2f}  "
              f"U {dd[2]:+7.2f} m")
    print(f"  CAM2 position corr = E {e2[0]:+6.2f}  N {e2[1]:+6.2f}  "
          f"U {e2[2]:+6.2f} m  (|e2| {np.linalg.norm(e2):.2f} m, "
          f"baseline {np.linalg.norm(cam2):.1f} -> "
          f"{np.linalg.norm(cam2 + e2):.1f} m)")
    print(f"  boresight resid   C1 yaw {b[0][0]:+6.3f} pitch {b[0][1]:+6.3f}  "
          f"C2 yaw {b[1][0]:+6.3f} pitch {b[1][1]:+6.3f} deg")
    print(f"  clock offset      C1 {tau[0] * 1e3:+6.1f} ms  C2 {tau[1] * 1e3:+6.1f} ms "
          f"(truth sampled at t+tau; negative = stamp too early)")
    print("\n== residual angle error after survey fit ==")
    for name, v in parts.items():
        mad = 1.4826 * np.median(np.abs(v - np.median(v)))
        print(f"  {name:9s} med {np.median(v):+6.3f}  MAD-sigma {mad:6.3f} deg")
    print(f"  overall RMS {np.sqrt(np.mean(r**2)):.3f} deg  "
          f"med|r| {np.median(np.abs(r)):.3f} deg")


if __name__ == "__main__":
    main()
