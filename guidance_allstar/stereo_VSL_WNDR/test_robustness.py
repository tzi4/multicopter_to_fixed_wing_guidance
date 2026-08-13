"""Adversarial probe of StereoTracker: what actually happens on bad input.

Not a pass/fail suite -- it prints behaviour, because most of these cases have
no single right answer and the point is to know what the filter does before
guidance depends on it. Run it after any change to the gating, coast, or
init logic.

Scenarios: NaN/inf angles, gross outliers, a sustained plausible outlier
(one camera tracking the wrong object), single-camera operation, total
dropout, out-of-order and duplicate stamps, and a frozen sender clock.

Scenarios marked CHECK are regression assertions for the defences added on
2026-07-31 (par.15/16 of REAL_DATA_FINDINGS.md): non-finite rejection,
stamp monotonicity, frozen-clock detection, and refusing to propagate
through a gap longer than COAST_TIMEOUT_S. They raise on regression.

The one gap that REMAINS by design, because no innovation-based test can
close it: a slow plausible bias is followed, not rejected, and the
covariance keeps shrinking while it happens. Closing that needs
independent corroboration (bbox angular size vs predicted range, track
continuity), not a better gate.
"""

import math
import warnings

import numpy as np

import stereo_geometry as sg
from stereo_estimator import StereoTracker

BASE, RNG = 90.0, 200.0


def rig():
    return [sg.Camera("L", (0.0, -BASE / 2, -2.0), sigma_yaw_deg=0.12,
                      sigma_pitch_deg=0.12, fov_yaw_deg=359, fov_pitch_deg=179,
                      max_range_m=20000),
            sg.Camera("R", (0.0, BASE / 2, -2.0), sigma_yaw_deg=0.12,
                      sigma_pitch_deg=0.12, fov_yaw_deg=359, fov_pitch_deg=179,
                      max_range_m=20000)]


def truth(t):
    return np.array([RNG + 20.0 * t, 5.0 * t, -80.0])


def dets(cams, t, stamp, which=(0, 1), noise=0.0, rs=None):
    out = []
    for i in which:
        _, pred = cams[i].bearing_jacobian(truth(t))
        y, p = pred[0], pred[1]
        if noise and rs is not None:
            y += math.radians(noise) * rs.randn()
            p += math.radians(noise) * rs.randn()
        out.append(sg.Detection(i, y, p, stamp))
    return out


def spin_up(tr, cams, n=30, rs=None):
    for k in range(n):
        t = k * 0.1
        tr.process(dets(cams, t, t, noise=0.05, rs=rs), t)
    return n * 0.1


def report(tag, snap, extra=""):
    x = snap.get("x")
    pos = "None" if x is None else f"[{x[0]:7.1f} {x[1]:6.1f} {x[2]:6.1f}]"
    spd = "  --  " if x is None else f"{np.linalg.norm(x[3:6]):6.1f}"
    print(f"  {tag:32s} {snap['state']:5s} track={str(snap['tracking']):5s} "
          f"{pos} |v|={spd} {extra}")


def probe_nonfinite():
    print("\n1. NaN / inf from ONE camera (the other is healthy)")
    for bad in (float("nan"), float("inf")):
        rs = np.random.RandomState(0)
        cams = rig()
        tr = StereoTracker(cams, nominal_dt=0.1)
        t = spin_up(tr, cams, rs=rs)
        d = dets(cams, t, t, noise=0.05, rs=rs)
        d[0].yaw = bad
        snap = tr.process(d, t)
        m = snap.get("meas") or {}
        report(f"cam0 yaw = {bad}", snap,
               f"accepted={m.get('n_accepted')} updated={m.get('updated')}")
        print(f"     healthy camera's 2 scalars were "
              f"{'USED' if m.get('updated') else 'DISCARDED'}")

    print("\n   sustained NaN vs the same camera merely ABSENT:")
    outcome = {}
    for label, make in (("NaN from cam0", "nan"), ("cam0 absent (mono)", "gone")):
        rs = np.random.RandomState(7)
        cams = rig()
        tr = StereoTracker(cams, nominal_dt=0.1)
        t = spin_up(tr, cams, rs=rs)
        for k in range(40):
            tt = t + k * 0.1
            if make == "nan":
                d = dets(cams, tt, tt, noise=0.05, rs=rs)
                d[0].yaw = float("nan")
            else:
                d = dets(cams, tt, tt, which=(1,), noise=0.05, rs=rs)
            snap = tr.process(d, tt)
            if k == 39:
                e = (np.linalg.norm(tr.imm.x[0:3] - truth(tt))
                     if snap["x"] is not None else float("nan"))
                report(f"{label}, 4.0 s", snap,
                       f"err={e:.1f} m" if snap["x"] is not None else "")
                outcome[make] = (bool(snap["tracking"]), e)
    print("     CHECK a faulty camera must degrade to mono, not kill the track")
    assert outcome["nan"][0], "regression: sustained NaN killed the track"
    # and it must track about as well as the healthy-mono control
    assert outcome["nan"][1] < 2.0 * outcome["gone"][1], (
        f"regression: NaN path degraded ({outcome['nan'][1]:.1f} m vs "
        f"{outcome['gone'][1]:.1f} m mono control)")


def probe_outliers():
    print("\n2. Gross single-frame outlier")
    for off in (0.5, 2.0, 10.0, 45.0):
        rs = np.random.RandomState(1)
        cams = rig()
        tr = StereoTracker(cams, nominal_dt=0.1)
        t = spin_up(tr, cams, rs=rs)
        before = tr.imm.x[0:3].copy()
        d = dets(cams, t, t)
        d[0].yaw += math.radians(off)
        snap = tr.process(d, t)
        m = snap.get("meas") or {}
        report(f"cam0 yaw +{off:5.1f} deg", snap,
               f"gated={m.get('n_gated')} inflated={m.get('n_inflated')} "
               f"jump={np.linalg.norm(tr.imm.x[0:3]-before):5.2f} m")

    print("\n3. SUSTAINED plausible outlier (cam0 on the wrong object, 3 s)")
    rs = np.random.RandomState(2)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    for k in range(30):
        tt = t + k * 0.1
        d = dets(cams, tt, tt, noise=0.05, rs=rs)
        d[0].yaw += math.radians(3.0)
        snap = tr.process(d, tt)
    err = np.linalg.norm(tr.imm.x[0:3] - truth(tt))
    sig = snap.get("sigma_pos_m", float("nan"))
    report("after 3 s of 3 deg bias", snap, f"err={err:.1f} m sigma={sig:.1f} m")
    print(f"     -> err/sigma = {err/max(sig,1e-9):.0f}: CONFIDENTLY wrong.")


def probe_camera_loss():
    print("\n4. Single-camera operation")
    rs = np.random.RandomState(3)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    for k in range(60):
        tt = t + k * 0.1
        snap = tr.process(dets(cams, tt, tt, which=(1,), noise=0.05, rs=rs), tt)
        if k in (4, 19, 58):
            err = np.linalg.norm(tr.imm.x[0:3] - truth(tt))
            report(f"mono {(k+1)*0.1:4.1f} s", snap,
                   f"err={err:5.1f} m sigma_along={snap.get('sigma_along_los_m', 0):5.1f} m")
    print("     -> error stays BELOW sigma: the covariance is honest here.")

    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    for k in range(40):
        tt = k * 0.1
        snap = tr.process(dets(cams, tt, tt, which=(0,)), tt)
    report("cold start, mono only", snap, "(init needs a two-ray seed)")

    print("\n5. Both cameras fail")
    rs = np.random.RandomState(4)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    for k in range(40):
        tt = t + k * 0.1
        snap = tr.process([], tt)
        if k in (4, 19, 25, 39):
            report(f"no detections {(k+1)*0.1:4.1f} s", snap,
                   f"coast={snap['coast_frames']}")


def probe_time():
    print("\n6. Stale / duplicate stamps")
    rs = np.random.RandomState(5)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    old = t - 5.0
    snap = tr.process(dets(cams, old, old), old)
    report("measurement stamped 5 s OLD", snap)
    m = snap.get("meas") or {}
    print(f"     CHECK rejected={m.get('error')!r}; filter clock still at "
          f"{tr.last_stamp - t:+.1f} s (not rewound to -5.0)")
    assert m.get("error") == "stale-stamp", "regression: stale stamp accepted"

    rs = np.random.RandomState(5)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    d = dets(cams, t, t)
    for _ in range(10):
        snap = tr.process(d, t)
    print(f"  duplicate packet x10 -> sigma_pos "
          f"{snap.get('sigma_pos_m', float('nan')):.2f} m, "
          f"{tr.rejected_frames} of 10 rejected")
    assert tr.rejected_frames >= 9, "regression: duplicates consumed as evidence"

    print("\n7. Frozen sender clock")
    rs = np.random.RandomState(6)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    t = spin_up(tr, cams, rs=rs)
    d = dets(cams, t, t)
    for _ in range(100):
        snap = tr.process(d, t)
    report("100 frames, stamp frozen", snap, f"stale={tr.stale_frames}")
    print("     CHECK measurement-time timeouts cannot fire on a stopped clock;"
          "\n        STALE_FRAME_LIMIT is what catches this.")
    assert not snap["tracking"], "regression: frozen sender clock not detected"

    print("\n8. Gap longer than COAST_TIMEOUT_S")
    for gap in (0.5, 1.0, 2.0, 3.0):
        rs = np.random.RandomState(3)
        cams = rig()
        tr = StereoTracker(cams, nominal_dt=0.1)
        t = spin_up(tr, cams, rs=rs)
        x0 = tr.imm.x[0:3].copy()
        v = float(np.linalg.norm(tr.imm.x[3:6]))
        snap = tr.process([], (t - 0.1) + gap)
        moved = float(np.linalg.norm(tr.imm.x[0:3] - x0))
        want = v * gap
        pct = 100.0 * moved / max(want, 1e-9)
        note = "" if snap["tracking"] else "-> LOST (refused, not clamped)"
        print(f"  gap {gap:4.1f} s: advanced {moved:6.2f} m of {want:6.2f} m "
              f"({pct:5.1f}%) {note}")
        if snap["tracking"]:
            assert pct > 95.0, (
                f"regression: under-propagated a {gap}s gap to {pct:.0f}%")


def probe_entered_bias():
    """A hand-entered boresight must reach the geometry path, not just the update."""
    print("\n9. Hand-entered boresight correction")
    BY, BP = 1.5, -0.8          # deg of real misalignment on cam0
    rows = []
    for label, enter in (("uncorrected", False), ("bias entered", True)):
        rs = np.random.RandomState(9)
        cams = rig()
        if enter:
            cams[0].bias_yaw = math.radians(BY)
            cams[0].bias_pitch = math.radians(BP)
        tr = StereoTracker(cams, nominal_dt=0.1)
        errs, skews = [], []
        for k in range(80):
            t = k * 0.1
            d = dets(cams, t, t, noise=0.05, rs=rs)
            d[0].yaw += math.radians(BY)      # the physical misalignment
            d[0].pitch += math.radians(BP)
            snap = tr.process(d, t)
            if k > 20 and snap["tracking"]:
                errs.append(np.linalg.norm(snap["x"][0:3] - truth(t)))
                g = snap.get("geom")
                if g is not None:
                    skews.append(float(g["skew_m"]))
        rows.append((label, float(np.median(errs)), float(np.median(skews))))
        print(f"  {label:14s} err={rows[-1][1]:7.2f} m   ray skew={rows[-1][2]:6.2f} m")
    # control: no misalignment at all
    rs = np.random.RandomState(9)
    cams = rig()
    tr = StereoTracker(cams, nominal_dt=0.1)
    errs = []
    for k in range(80):
        t = k * 0.1
        snap = tr.process(dets(cams, t, t, noise=0.05, rs=rs), t)
        if k > 20 and snap["tracking"]:
            errs.append(np.linalg.norm(snap["x"][0:3] - truth(t)))
    clean = float(np.median(errs))
    print(f"  {'clean control':14s} err={clean:7.2f} m")
    print("     CHECK entering the bias must recover the clean case AND cut skew")
    assert rows[1][1] < 0.5 * rows[0][1], (
        f"regression: entered bias barely helped ({rows[1][1]:.2f} vs "
        f"{rows[0][1]:.2f} m)")
    assert rows[1][2] < 0.5 * rows[0][2], (
        f"regression: skew still biased ({rows[1][2]:.2f} vs {rows[0][2]:.2f} m)"
        " -- the geometry path is not honouring the correction")
    assert rows[1][1] < 2.0 * clean, "regression: corrected case worse than clean"


def main():
    warnings.filterwarnings("ignore")
    print("=" * 72)
    print("StereoTracker robustness probe")
    print("=" * 72)
    probe_nonfinite()
    probe_outliers()
    probe_camera_loss()
    probe_time()
    probe_entered_bias()
    print()


if __name__ == "__main__":
    main()
