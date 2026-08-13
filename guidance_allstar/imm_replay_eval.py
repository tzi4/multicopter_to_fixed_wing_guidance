#!/usr/bin/env python3
"""
Offline estimator evaluation: replay logged measurements through filterwndr.py.

Re-runs the exact runtime update sequence (hints -> predict -> update -> hints
-> stabilize) over the meas_*/stamp columns of an imm_diagnostics CSV, builds
ground truth by central cubic fits of the measurements (valid because the
message-time transport noise floor is cm-level), and reports est-vs-truth
metrics plus per-turn-entry transient peaks.

Usage:
    python3 imm_replay_eval.py logs/imm_diagnostics_YYYYMMDD_HHMMSS.csv [more.csv ...]
    python3 imm_replay_eval.py --set MEASUREMENT_R_DIAG=[0.04,0.04,0.04] --set SIGMA_OMEGA_DOT=0.5 LOG.csv

Without --set it evaluates the current constants in filterwndr.py, so a
baseline replay of a log made with the same constants reproduces the logged
est_* bit-exactly (reported as the fidelity check).
"""
import argparse
import ast
import csv

import numpy as np

import filterwndr as fw


def load_log(path):
    rows = list(csv.DictReader(open(path)))

    def col(name, default=np.nan):
        out = []
        for r in rows:
            v = r.get(name, "")
            try:
                out.append(float(v))
            except (ValueError, TypeError):
                out.append(default)
        return np.array(out)

    return {
        "stamp": col("stamp"),
        "meas": np.column_stack([col("meas_x"), col("meas_y"), col("meas_z")]),
        "est_logged": np.column_stack([col("est_x"), col("est_y"), col("est_z")]),
    }


def ground_truth(stamp, meas, half_window_s=1.25, min_pts=8):
    n = len(stamp)
    pos = np.full((n, 3), np.nan)
    vel = np.full((n, 3), np.nan)
    acc = np.full((n, 3), np.nan)
    for i in range(n):
        t0 = stamp[i]
        idx = np.where(np.abs(stamp - t0) <= half_window_s)[0]
        if len(idx) < min_pts:
            lo = max(0, i - min_pts // 2)
            hi = min(n, lo + min_pts)
            lo = max(0, hi - min_pts)
            idx = np.arange(lo, hi)
        tau = stamp[idx] - t0
        A = np.column_stack([np.ones_like(tau), tau, tau**2, tau**3])
        for ax in range(3):
            c, *_ = np.linalg.lstsq(A, meas[idx, ax], rcond=None)
            pos[i, ax] = c[0]
            vel[i, ax] = c[1]
            acc[i, ax] = 2.0 * c[2]
    sp2 = vel[:, 0] ** 2 + vel[:, 1] ** 2
    omega = np.where(
        sp2 > 4.0,
        (vel[:, 0] * acc[:, 1] - vel[:, 1] * acc[:, 0]) / np.maximum(sp2, 1e-9),
        0.0,
    )
    return pos, vel, acc, omega


def replay(log):
    imm = fw.setup_imm_filter(0.1)
    tre = fw.HeadingTurnRateEstimator()
    n = len(log["stamp"])
    est = np.zeros((n, 10))
    mu_ct = np.zeros(n)
    omega_eff = np.zeros(n)

    last_stamp = None
    for i in range(n):
        z = log["meas"][i]
        stamp = log["stamp"][i]
        dt_raw = 0.1 if last_stamp is None else stamp - last_stamp
        last_stamp = stamp
        dt_actual = fw.clamp_filter_dt(dt_raw)

        omega_hint = tre.update(z, stamp)

        if i == 0:
            for f in imm.filters:
                f.x[0:3] = z
            imm.x = imm.filters[0].x.copy()

        fw.apply_fast_turn_onset_hint(imm, tre.raw_omega, tre.speed_xy)
        fw.apply_turn_rate_hint(imm, omega_hint)
        fw.predict_imm_over_dt(imm, dt_actual)
        imm.update(z)
        fw.apply_turn_rate_hint(imm, omega_hint)
        fw.stabilize_omega_states(imm)

        est[i] = np.asarray(imm.x, dtype=float).ravel()
        mu_ct[i] = fw.ct_mode_probability(imm)
        omega_eff[i] = fw.get_effective_turn_rate(imm)

    return est, mu_ct, omega_eff


def turn_events(omega_true, thresh=0.15, debounce=10):
    events = []
    last = -(10**9)
    for i in range(1, len(omega_true)):
        if abs(omega_true[i]) >= thresh and abs(omega_true[i - 1]) < thresh:
            if i - last >= debounce:
                events.append(i)
            last = i
    return events


def report(path, log, truth, est, mu_ct, skip=15):
    pos_t, vel_t, _, om_t = truth
    n = len(log["stamp"])
    lo, hi = skip, n - 6
    sl = slice(lo, hi)

    pe = np.linalg.norm(est[sl, 0:3] - pos_t[sl], axis=1)
    ve = np.linalg.norm(est[sl, 3:6] - vel_t[sl], axis=1)
    ups = int(np.sum((mu_ct[1:] > 0.5) & (mu_ct[:-1] <= 0.5)))
    events = [e for e in turn_events(om_t) if lo + 4 < e < hi - 16]

    print(f"\n{path}")
    print(
        f"  pos err vs truth: median={np.nanmedian(pe):.3f}  p95={np.nanpercentile(pe, 95):.3f}  max={np.nanmax(pe):.3f} m"
    )
    print(
        f"  vel err vs truth: median={np.nanmedian(ve):.3f}  p95={np.nanpercentile(ve, 95):.3f}  max={np.nanmax(ve):.3f} m/s"
    )
    print(f"  mu_ct>0.5 upcrossings={ups}  detected turn entries={len(events)}")
    for e in events:
        w = slice(max(lo, e - 2), min(hi, e + 16))
        pp = np.nanmax(np.linalg.norm(est[w, 0:3] - pos_t[w], axis=1))
        vp = np.nanmax(np.linalg.norm(est[w, 3:6] - vel_t[w], axis=1))
        print(
            f"    entry@row{e:4d} omega_true={om_t[e]:+.2f}: peak pos err {pp:.2f} m, peak vel err {vp:.1f} m/s"
        )

    d = np.linalg.norm(est[:, 0:3] - log["est_logged"], axis=1)
    print(
        f"  fidelity vs logged est_*: median={np.nanmedian(d):.4f} m, max={np.nanmax(d):.4f} m"
        " (only ~0 if the log was made with the same constants)"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", help="imm_diagnostics CSV paths")
    ap.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="override a filterwndr constant for this run, e.g. MEASUREMENT_R_DIAG=[0.04,0.04,0.04]",
    )
    args = ap.parse_args()

    # Record the file mtime first so later internal refresh calls do not
    # overwrite the CLI overrides.
    fw.refresh_dynamic_filter_params()
    for item in args.set:
        name, _, raw = item.partition("=")
        value = ast.literal_eval(raw)
        if isinstance(value, (list, tuple)):
            value = np.array(value, dtype=float)
        setattr(fw, name, value)
        print(f"override: {name} = {value}")

    for path in args.logs:
        log = load_log(path)
        truth = ground_truth(log["stamp"], log["meas"])
        est, mu_ct, _ = replay(log)
        report(path, log, truth, est, mu_ct)


if __name__ == "__main__":
    main()
