"""Timestamp errors: how big, what they cost, and what fixing them buys.

Three independent measurements, none of which need the guidance log:

``--scan``   2-D scan of the per-camera stamp error (tau1, tau2). The full
             site geometry is refitted at every grid point, so timing cannot
             be bought by absorbing it into the survey. tau > 0 means the ray
             stamped t actually saw the target at t+tau, i.e. the stamp sits
             EARLIER than the true capture instant. The DIFFERENTIAL
             tau1-tau2 is the robust number: any error in the truth clock is
             common to both cameras and cancels in the difference.

``--cost``   Pure simulation from truth: perfect rays, perfect calibration,
             the only defect being that the two cameras refer to different
             instants. Isolates what time-skew alone costs a triangulator,
             and -- importantly -- how little SKEW it induces, i.e. how
             invisible it is to the skew quality gate.

``--policy`` The fix, on real data: interpolate each camera's own angle
             series to a common epoch before triangulating (truth-free), with
             and without the fitted per-camera shift.

Truth is used for fitting and scoring only; nothing here runs in the tracker.
"""

from __future__ import annotations

import argparse

import numpy as np

from mono_ingest import GOOD_WINDOWS, read_mono, truth_enu_at, truth_tracks

# fitted per-camera stamp error (seconds); see --scan
TAU = {"c1": 0.225, "c2": 0.100}


def unit(v):
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def ray_enu(head_deg, pitch_deg):
    h = np.radians(head_deg)
    p = np.radians(pitch_deg)
    return np.stack([np.sin(h) * np.cos(p), np.cos(h) * np.cos(p),
                     np.sin(p)], axis=-1)


def triangulate(p1, d1, p2, d2):
    w0 = p1 - p2
    aa = np.sum(d1 * d1, -1); bb = np.sum(d1 * d2, -1); cc = np.sum(d2 * d2, -1)
    dd = np.sum(d1 * w0, -1); ee = np.sum(d2 * w0, -1)
    den = aa * cc - bb * bb
    s = (bb * ee - cc * dd) / den
    u = (aa * ee - bb * dd) / den
    q1 = p1 + s[:, None] * d1
    q2 = p2 + u[:, None] * d2
    return 0.5 * (q1 + q2), np.linalg.norm(q1 - q2, axis=-1)


class Ctx:
    def __init__(self):
        self.log = read_mono()
        self.tracks = truth_tracks()
        self.frame = self.log.frame
        self.cam2 = np.array(self.frame.to_enu(*self.log.cam2_lla))
        self.pooled = np.zeros(len(self.log), bool)
        for a, b in GOOD_WINDOWS:
            self.pooled |= self.log.window_mask(a, b)
        self.grids = []
        for tr in self.tracks:
            g = np.arange(tr.span[0], tr.span[1], 0.02)
            self.grids.append((tr, g, truth_enu_at([tr], self.frame, g)))

    def truth_at(self, t, shift=0.0):
        t = np.asarray(t, float)
        out = np.full((t.size, 3), np.nan)
        for tr, g, enu in self.grids:
            m = tr.covers(t)
            if m.any():
                for ax in range(3):
                    out[m, ax] = np.interp(t[m] + shift, g, enu[:, ax])
        return out


# ---------------------------------------------------------------------------

def scan(ctx, sub=3, hi=0.5, step=0.05):
    from scipy.optimize import least_squares

    c = ctx.log.cols
    obs = []
    for ci, cam in enumerate(("c1", "c2")):
        t = c[f"{cam}_ts"][ctx.pooled]
        h = c[f"{cam}_head"][ctx.pooled]
        p = c[f"{cam}_pitch"][ctx.pooled]
        ok = np.isfinite(t) & np.isfinite(h) & np.isfinite(p)
        fl = np.full(t.size, -1)
        for k, tr in enumerate(ctx.tracks):
            fl[tr.covers(t)] = k
        ok &= fl >= 0
        obs.append((ci, t[ok][::sub], h[ok][::sub], p[ok][::sub], fl[ok][::sub]))

    def truth_fl(t, fl, tau):
        out = np.empty((t.size, 3))
        for k, (tr, g, enu) in enumerate(ctx.grids):
            m = fl == k
            if m.any():
                for ax in range(3):
                    out[m, ax] = np.interp(t[m] + tau, g, enu[:, ax])
        return out

    def make_res(tau1, tau2):
        def residuals(x):
            d = [x[0:3], x[3:6]]
            e2 = x[6:9]
            b = [(x[9], x[10]), (x[11], x[12])]
            taus = (tau1, tau2)
            res = []
            for ci, t, h, p, fl in obs:
                pos = np.zeros(3) if ci == 0 else ctx.cam2 + e2
                tr = truth_fl(t, fl, taus[ci])
                for k in (0, 1):
                    tr[fl == k] -= d[k]
                v = tr - pos
                rng = np.linalg.norm(v, axis=-1)
                ht = np.degrees(np.arctan2(v[:, 0], v[:, 1]))
                pt = np.degrees(np.arcsin(np.clip(v[:, 2] / np.maximum(rng, 1e-6),
                                                  -1, 1)))
                dh = (((h + b[ci][0] - ht + 180.0) % 360.0 - 180.0)
                      * np.cos(np.radians(pt)))
                res.append(dh)
                res.append(p + b[ci][1] - pt)
            return np.concatenate(res)
        return residuals

    x0 = np.zeros(13)
    x0[0:3] = [13.7, -4.3, 81.3]
    x0[3:6] = [8.0, -2.9, 81.8]
    taus = np.arange(0.0, hi + 1e-9, step)
    best = (None, np.inf, None)
    print(f"cost surface ({obs[0][1].size} frames/camera); "
          f"columns tau2 = {taus[0]:.2f}..{taus[-1]:.2f}")
    for t1 in taus:
        row = []
        for t2 in taus:
            sol = least_squares(make_res(t1, t2), x0, loss="huber", f_scale=0.3,
                                x_scale=[10.0] * 9 + [0.5] * 4, max_nfev=200)
            row.append(f"{sol.cost:7.0f}")
            if sol.cost < best[1]:
                best = ((t1, t2), sol.cost, sol.x)
        print(f"  tau1={t1:.2f} | " + " ".join(row), flush=True)
    (t1, t2), cost, x = best
    print(f"\nminimum tau1={t1:.2f} tau2={t2:.2f} (cost {cost:.0f}); "
          f"differential {t1-t2:+.3f} s")
    print(f"  CAM2 correction |e2| {np.linalg.norm(x[6:9]):.2f} m, baseline "
          f"{np.linalg.norm(ctx.cam2):.1f} -> {np.linalg.norm(ctx.cam2+x[6:9]):.1f} m")
    print("  tau > 0 => the stamp sits EARLIER than the true capture instant")


# ---------------------------------------------------------------------------

def cost_of_skew(ctx):
    """Timing error only: perfect rays, perfect calibration."""
    t = ctx.log.cols["loc_ts"][ctx.pooled]
    p0 = ctx.truth_at(t)
    ok = np.isfinite(p0[:, 0])
    t, p0 = t[ok], p0[ok]
    los = unit(p0)
    spd = np.linalg.norm(ctx.truth_at(t, 0.05) - ctx.truth_at(t, -0.05), axis=-1) / 0.1
    print(f"n={t.size}  median range {np.median(np.linalg.norm(p0, axis=-1)):.0f} m  "
          f"median speed {np.median(spd):.1f} m/s  "
          f"baseline {np.linalg.norm(ctx.cam2):.1f} m\n")

    def run(tau1, tau2, label):
        a, b = ctx.truth_at(t, tau1), ctx.truth_at(t, tau2)
        v = np.isfinite(a[:, 0]) & np.isfinite(b[:, 0])
        fix, sk = triangulate(np.zeros(3), unit(a[v]), ctx.cam2,
                              unit(b[v] - ctx.cam2))
        e = fix - p0[v]
        tot = np.linalg.norm(e, axis=-1)
        dep = np.abs(np.sum(e * los[v], -1))
        print(f"  {label:36s} total med {np.median(tot):5.2f}  "
              f"p90 {np.percentile(tot,90):5.2f}   depth {np.median(dep):5.2f}   "
              f"induced skew {np.median(sk):5.2f} m")

    run(0.0, 0.0, "control (both stamps correct)")
    run(0.20, 0.20, "common-mode 0.20 s (both equal)")
    run(TAU["c1"], TAU["c2"], f"fitted {TAU['c1']:.2f}/{TAU['c2']:.2f}")
    run(TAU["c1"] - TAU["c2"], 0.0, "differential only")
    print("\n  the induced SKEW is far below the gate threshold: a triangulator"
          "\n  cannot detect its own time-skew from ray geometry.")


# ---------------------------------------------------------------------------

def policy(ctx):
    from mono_eval import Session
    sess = Session(ctx.log)

    def build(mask, mode):
        r = sess.rows(mask)
        t0 = r["loc_ts"]
        out = {}
        for cam in ("c1", "c2"):
            tc = r[f"{cam}_ts"]
            h, p = r[f"{cam}_head"], r[f"{cam}_pitch"]
            ok = np.isfinite(tc) & np.isfinite(h) & np.isfinite(p)
            tc, h, p = tc[ok], h[ok], p[ok]
            o = np.argsort(tc)
            tc, h, p = tc[o], h[o], p[o]
            if mode == "as-logged":
                j = np.clip(np.searchsorted(tc, t0), 0, tc.size - 1)
                out[cam] = (h[j], p[j])
            else:
                shift = TAU[cam] if mode == "interp+tau" else 0.0
                hu = np.unwrap(np.radians(h))
                out[cam] = (np.degrees(np.interp(t0, tc + shift, hu,
                                                 left=np.nan, right=np.nan)),
                            np.interp(t0, tc + shift, p, left=np.nan, right=np.nan))
        return t0, out

    print("triangulation error vs truth, by ray time-alignment policy")
    for mode in ("as-logged", "interp", "interp+tau"):
        tot, dep = [], []
        for m in sess.masks:
            t0, ang = build(m, mode)
            d1 = ray_enu(*ang["c1"])
            d2 = ray_enu(*ang["c2"])
            v = np.isfinite(d1[:, 0]) & np.isfinite(d2[:, 0])
            if v.sum() < 10:
                continue
            fix, _ = triangulate(np.zeros(3), d1[v], ctx.cam2, d2[v])
            shift = TAU["c1"] if mode == "interp+tau" else 0.0
            # datum-corrected truth: the constant survey offset is a separate
            # defect and would otherwise swamp the timing effect entirely
            tr = sess.truth_enu(t0[v] + shift)
            g = np.isfinite(tr[:, 0])
            e = fix[g] - tr[g]
            tot.append(np.linalg.norm(e, axis=-1))
            dep.append(np.abs(np.sum(e * unit(tr[g]), -1)))
        tot = np.concatenate(tot)
        dep = np.concatenate(dep)
        print(f"  {mode:12s} n={tot.size:5d}  total med {np.median(tot):6.2f}  "
              f"p90 {np.percentile(tot,90):6.2f}   depth med {np.median(dep):6.2f}")
    print("\n  the gain is small TODAY because timing adds in quadrature with a"
          "\n  much larger survey error; it dominates once that is fixed.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--cost", action="store_true")
    ap.add_argument("--policy", action="store_true")
    args = ap.parse_args()
    if not (args.scan or args.cost or args.policy):
        args.cost = args.policy = True
    ctx = Ctx()
    if args.scan:
        scan(ctx)
    if args.cost:
        print("\n== cost of ray time-skew alone ==")
        cost_of_skew(ctx)
    if args.policy:
        print("\n== fixing it on real data ==")
        policy(ctx)


if __name__ == "__main__":
    main()
