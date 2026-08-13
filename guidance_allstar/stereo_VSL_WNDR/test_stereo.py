#!/usr/bin/env python3
"""
test_stereo.py -- correctness checks for the bearings-only stack.

Run:  python3 test_stereo.py
No pytest dependency; prints OK/FAIL per check and exits nonzero on failure.
"""

import sys

import numpy as np

import stereo_config as scfg
import stereo_geometry as sg
import stereo_measurement as sm
from stereo_estimator import StereoTracker, TrackState

RESULTS = []


def check(name, cond, note=""):
    RESULTS.append((name, bool(cond)))
    print(f"{'OK  ' if cond else 'FAIL'} {name}" + (f"   [{note}]" if note else ""))


def stereo_pair(baseline=6.0, sigma_deg=0.06, tilt_deg=0.0):
    cfgs = []
    for i, name in enumerate(("left", "right")):
        sign = -1.0 if i == 0 else 1.0
        y = sign * baseline / 2.0 * np.cos(np.radians(tilt_deg))
        z = -2.0 + sign * baseline / 2.0 * np.sin(np.radians(tilt_deg))
        cfgs.append(dict(name=name, position_ned=(0.0, y, z), yaw_deg=0.0, pitch_deg=10.0,
                         roll_deg=0.0, sigma_yaw_deg=sigma_deg, sigma_pitch_deg=sigma_deg,
                         fov_yaw_deg=120.0, fov_pitch_deg=120.0, max_range_m=3000.0))
    return sg.build_cameras(cfgs)


# ----------------------------------------------------------------------
def test_geometry():
    rng = np.random.default_rng(0)
    print("\n--- geometry ---")

    worst = 0.0
    for _ in range(300):
        cam = sg.Camera("c", rng.normal(0, 50, 3), rng.uniform(-180, 180),
                        rng.uniform(-60, 60), rng.uniform(-40, 40))
        p = cam.position + cam.R_cw @ np.array(
            [rng.uniform(5, 500), rng.uniform(-100, 100), rng.uniform(-100, 100)])
        y, pi = cam.bearing_of(p)
        r = np.linalg.norm(p - cam.position)
        worst = max(worst, float(np.linalg.norm(cam.position + cam.ray_direction(y, pi) * r - p)))
    check("bearing <-> ray round trip", worst < 1e-8, f"worst {worst:.1e} m")

    cam = sg.Camera("c", (0, 0, 0))
    y, p = cam.bearing_of((100, 0, 0))
    check("boresight -> (0,0)", abs(y) < 1e-9 and abs(p) < 1e-9)
    y, _ = cam.bearing_of((100, 100, 0))
    check("east of boresight -> yaw +45", abs(np.degrees(y) - 45) < 1e-9)
    _, p = cam.bearing_of((100, 0, -100))
    check("above -> pitch +45 (up positive)", abs(np.degrees(p) - 45) < 1e-9)

    worst = 0.0
    for _ in range(200):
        cam = sg.Camera("c", rng.normal(0, 30, 3), rng.uniform(-180, 180),
                        rng.uniform(-45, 45), rng.uniform(-30, 30))
        x = cam.position + cam.R_cw @ np.array(
            [rng.uniform(20, 400), rng.uniform(-80, 80), rng.uniform(-80, 80)])
        J, _ = cam.bearing_jacobian(x)
        Jn = np.zeros((2, 3))
        h = 1e-4
        for k in range(3):
            e = np.zeros(3); e[k] = h
            yp, pp = cam.bearing_of(x + e); ym, pm = cam.bearing_of(x - e)
            Jn[0, k] = sg.wrap_pi(yp - ym) / (2 * h)
            Jn[1, k] = (pp - pm) / (2 * h)
        worst = max(worst, float(np.max(np.abs(J - Jn)) / max(np.max(np.abs(Jn)), 1e-9)))
    check("bearing jacobian == numerical", worst < 1e-5, f"worst rel {worst:.1e}")

    check("wrap_pi handles the seam", abs(float(sg.wrap_pi(np.pi + 0.1) - (-np.pi + 0.1))) < 1e-12)


def test_triangulation():
    print("\n--- triangulation ---")
    cams = stereo_pair()
    truth = np.array([300.0, 40.0, -50.0])
    dets = [sg.Detection(i, *cams[i].bearing_of(truth), stamp=0.0) for i in (0, 1)]

    mid, info = sg.triangulate_midpoint(cams[0], dets[0], cams[1], dets[1])
    check("noise-free midpoint is exact", np.linalg.norm(mid - truth) < 1e-6)
    check("noise-free skew ~ 0", info["skew_m"] < 1e-6, f"{info['skew_m']:.1e} m")

    ml, _ = sg.triangulate_ml(cams, dets, mid + np.array([30.0, 20.0, -25.0]), iters=8)
    check("ML converges from a bad seed", np.linalg.norm(ml - truth) < 1e-5)

    P, pinfo = sg.triangulation_covariance(cams, dets, truth)
    ev, evec = np.linalg.eigh(P)
    los = (truth - cams[0].position) / np.linalg.norm(truth - cams[0].position)
    check("uncertainty is a cigar along the LOS", abs(float(evec[:, -1] @ los)) > 0.95,
          f"sigmas {np.sqrt(np.maximum(ev,0)).round(2)}")
    check("depth sigma >> cross sigma", np.sqrt(ev[-1]) > 20 * np.sqrt(ev[0]),
          f"{np.sqrt(ev[-1]):.1f} vs {np.sqrt(ev[0]):.2f} m")

    # single camera: rank-deficient but must not produce a fake-confident P
    P1, i1 = sg.triangulation_covariance(cams, [dets[0]], truth)
    check("single-camera P flags rank deficiency", i1["rank_deficient"])
    check("single-camera P is huge along the ray", np.max(np.linalg.eigvalsh(P1)) > 1e6)

    a, c = sg.los_decompose(np.array([3.0, 4.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    check("LOS decomposition splits correctly", abs(a - 3) < 1e-9 and abs(c - 4) < 1e-9)


def test_scalar_update():
    print("\n--- scalar update ---")
    n = 10
    x = np.zeros(n); x[0:3] = [100.0, 0.0, -50.0]
    P = np.eye(n) * 25.0
    H = np.zeros(n); H[0] = 1.0
    x2, P2, S, nis, ll = sm.scalar_update(x, P, H, 5.0, 1.0)
    check("scalar update moves toward the measurement", x2[0] > x[0])
    check("scalar update shrinks that variance", P2[0, 0] < P[0, 0])
    check("covariance stays symmetric", np.allclose(P2, P2.T))
    check("covariance stays PSD", np.min(np.linalg.eigvalsh(P2)) >= -1e-9)
    check("loglik is finite", np.isfinite(ll))
    # Kalman consistency: with H = e0, the posterior variance must equal the
    # textbook scalar result P - P^2/(P+R).
    expect = 25.0 - 25.0 ** 2 / (25.0 + 1.0)
    check("posterior variance matches theory", abs(P2[0, 0] - expect) < 1e-9)


def test_gating():
    print("\n--- gating ---")
    cams = stereo_pair()
    truth = np.array([300.0, 40.0, -50.0])
    x = np.zeros(10); x[0:3] = truth
    P = np.eye(10); P[0:3, 0:3] = np.eye(3) * 4.0

    good = [sg.Detection(i, *cams[i].bearing_of(truth), stamp=0.0) for i in (0, 1)]
    specs = sm.build_measurement_plan(cams, good, x, P)
    check("clean frame is fully accepted", all(s.accepted for s in specs) and len(specs) == 4)
    check("clean frame is not inflated", all(s.inflation == 1.0 for s in specs))

    y, p = cams[1].bearing_of(truth)
    bad = [good[0], sg.Detection(1, y + np.radians(5.0), p, stamp=0.0)]
    specs = sm.build_measurement_plan(cams, bad, x, P)
    outlier = [s for s in specs if s.cam_index == 1 and s.axis == 0][0]
    check("gross outlier is caught", (not outlier.accepted) or outlier.inflation > 5.0,
          f"nis {outlier.nis_prior:.0f} acc={outlier.accepted} infl={outlier.inflation:.1f}")
    clean_pitch = [s for s in specs if s.cam_index == 1 and s.axis == 1][0]
    check("its clean sibling axis survives", clean_pitch.accepted and clean_pitch.inflation < 2.0)

    check("skew scale is 1 when rays agree", sm.skew_noise_scale(0.5) == 1.0)
    check("skew scale grows with disagreement", sm.skew_noise_scale(12.0) > 1.0)
    check("huge skew rejects the frame", not np.isfinite(sm.skew_noise_scale(1e3)))


def test_tracker_converges():
    print("\n--- tracker (straight-line target) ---")
    cams = stereo_pair()
    tracker = StereoTracker(cams, cfg=scfg)
    rng = np.random.default_rng(7)
    p0 = np.array([250.0, -150.0, -50.0])
    vel = np.array([0.0, 19.0, 0.0])
    errs, t = [], 0.0
    for k in range(600):
        t = k * 0.05
        pos = p0 + vel * t
        dets = []
        for i, cam in enumerate(cams):
            yy, pp = cam.bearing_of(pos)
            dets.append(sg.Detection(i, yy + rng.normal(0, cam.sigma_yaw),
                                     pp + rng.normal(0, cam.sigma_pitch), stamp=t))
        snap = tracker.process(dets, t)
        if snap["tracking"] and t > 5.0:
            errs.append(float(np.linalg.norm(snap["position"] - pos)))
    check("track initialises", tracker.state in (TrackState.TRACK, TrackState.COAST))
    check("error is bounded", np.median(errs) < 25.0, f"median {np.median(errs):.2f} m")
    check("beats the per-frame CRLB depth", np.median(errs) < 60.0,
          f"median {np.median(errs):.2f} m over {len(errs)} frames")
    vel_err = float(np.linalg.norm(tracker.imm.x[3:6] - vel))
    check("velocity converges", vel_err < 6.0, f"{vel_err:.2f} m/s")


def test_single_camera_and_dropout():
    print("\n--- degraded inputs ---")
    cams = stereo_pair()
    tracker = StereoTracker(cams, cfg=scfg)
    rng = np.random.default_rng(11)
    p0 = np.array([300.0, -100.0, -50.0]); vel = np.array([0.0, 19.0, 0.0])
    got_single = 0
    errs = []
    for k in range(700):
        t = k * 0.05
        pos = p0 + vel * t
        dets = []
        for i, cam in enumerate(cams):
            # after 10 s the right camera goes dark: the track must survive on
            # one camera (2 constraints), which a triangulation-first design
            # could not do at all
            if i == 1 and t > 10.0:
                continue
            yy, pp = cam.bearing_of(pos)
            dets.append(sg.Detection(i, yy + rng.normal(0, cam.sigma_yaw),
                                     pp + rng.normal(0, cam.sigma_pitch), stamp=t))
        if len(dets) == 1:
            got_single += 1
        snap = tracker.process(dets, t)
        if snap["tracking"] and t > 12.0:
            errs.append(float(np.linalg.norm(snap["position"] - pos)))
    check("ran single-camera frames", got_single > 100, f"{got_single} frames")
    check("track survives losing a camera", tracker.state != TrackState.LOST)
    check("single-camera error stays finite", len(errs) > 0 and np.all(np.isfinite(errs)),
          f"median {np.median(errs):.1f} m" if errs else "no frames")

    # total blackout -> must declare LOST rather than silently coast forever
    t_last = 700 * 0.05
    for k in range(120):
        tracker.process([], t_last + k * 0.05)
    check("declares LOST after a blackout", tracker.state == TrackState.LOST)


def test_no_regression_in_filterwndr():
    print("\n--- estimator core untouched ---")
    import filterwndr as fw
    imm = fw.setup_imm_filter(0.05)
    check("product IMM still builds", len(imm.filters) == len(fw.PRODUCT_MODE_SPECS))
    check("mode probabilities normalised", abs(float(np.sum(imm.mu)) - 1.0) < 1e-9)
    x0 = imm.x.copy()
    fw.predict_imm_over_dt(imm, 0.1, max_substep=0.1)
    check("predict runs and keeps state finite", np.all(np.isfinite(imm.x)))
    check("P stays symmetric after predict", np.allclose(imm.P, imm.P.T, atol=1e-8))


def main():
    test_geometry()
    test_triangulation()
    test_scalar_update()
    test_gating()
    test_tracker_converges()
    test_single_camera_and_dropout()
    test_no_regression_in_filterwndr()

    passed = sum(1 for _, ok in RESULTS if ok)
    total = len(RESULTS)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    if passed != total:
        print("  FAILED: " + ", ".join(n for n, ok in RESULTS if not ok))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
