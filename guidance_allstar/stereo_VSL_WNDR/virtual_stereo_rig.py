#!/usr/bin/env python3
"""
virtual_stereo_rig.py -- two virtual cameras, noisy rays, and a scoreboard.

Stands up a synthetic stereo rig at a fixed position, points it at a target
whose TRUE position comes from one of three sources, casts a noisy bearing ray
from each camera every frame, and feeds those rays to the bearings-only
tracker. Because the truth is known exactly, every frame can be scored.

    --source mavlink   live telemetry from the plane (the real test rig)
    --source log       replay a guided_follow_*.csv (its meas_* columns are the
                       plane's true NED track -- lets the whole stereo stack be
                       validated against real flight data with no SITL running)
    --source synthetic  canned racetrack, for when nothing else is up

The scoreboard deliberately splits error into ALONG-LOS and CROSS-LOS. A single
RMS number hides the entire character of a bearings-only sensor, where depth is
~100x worse than cross-range; anyone reporting one scalar is fooling themselves.
It also runs a per-frame triangulation-only tracker alongside, so the value the
IMM adds (mostly: recovering depth by integrating angles over time) is visible
rather than asserted.

Examples
--------
    # live against the plane
    python3 virtual_stereo_rig.py --source mavlink --target udpin:localhost:14600

    # replay real flight truth, 6 m baseline, 1 mrad cameras
    python3 virtual_stereo_rig.py --source log --log ../logs/guided_follow_20260724_110805.csv \
        --baseline 6 --sigma-deg 0.06 --plot

    # stress: dropouts, false detections, a boresight bias on one camera
    python3 virtual_stereo_rig.py --source synthetic --dropout 0.1 --outlier-rate 0.02 \
        --bias-deg 0.05
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

import stereo_config as scfg
import stereo_geometry as sg
from stereo_estimator import StereoTracker, TriangulationOnlyTracker, TrackState

_EPS = 1e-12


# ======================================================================
#  Truth sources
# ======================================================================
class SyntheticTruth:
    """Racetrack at constant altitude -- straights plus hard turns, matching
    the real target's ~19 m/s / 50 m profile so the IMM's CV/CT modes are both
    exercised."""

    def __init__(self, duration_s=120.0, rate_hz=20.0, speed=19.0,
                 radius=120.0, straight=260.0, altitude=50.0, seed=0):
        self.duration = float(duration_s)
        self.dt = 1.0 / float(rate_hz)
        self.speed = float(speed)
        self.radius = float(radius)
        self.straight = float(straight)
        self.altitude = float(altitude)
        self.name = "synthetic racetrack"

    def __iter__(self):
        # Racetrack perimeter: two straights + two semicircles.
        per = 2.0 * self.straight + 2.0 * math.pi * self.radius
        t = 0.0
        while t <= self.duration:
            s = (self.speed * t) % per
            if s < self.straight:                      # north leg
                x, y = s, -self.radius
                hdg = 0.0
            elif s < self.straight + math.pi * self.radius:   # east turn
                a = (s - self.straight) / self.radius
                x = self.straight + self.radius * math.sin(a)
                y = -self.radius * math.cos(a)
                hdg = a
            elif s < 2.0 * self.straight + math.pi * self.radius:  # south leg
                d = s - self.straight - math.pi * self.radius
                x, y = self.straight - d, self.radius
                hdg = math.pi
            else:                                       # west turn
                a = (s - 2.0 * self.straight - math.pi * self.radius) / self.radius
                x = -self.radius * math.sin(a)
                y = self.radius * math.cos(a)
                hdg = math.pi + a
            pos = np.array([x + 200.0, y, -self.altitude])
            vel = np.array([self.speed * math.cos(hdg), self.speed * math.sin(hdg), 0.0])
            yield t, pos, vel
            t += self.dt


class LogTruth:
    """Replay the plane's true NED track from a guided_follow_*.csv."""

    def __init__(self, path, t0=None, t1=None):
        self.path = path
        self.name = f"log {os.path.basename(path)}"
        rows = list(csv.DictReader(open(path)))
        if not rows:
            raise SystemExit(f"empty log: {path}")
        need = {"wall_time", "meas_x", "meas_y", "meas_z"}
        missing = need - set(rows[0].keys())
        if missing:
            raise SystemExit(f"log missing columns {sorted(missing)}: {path}")
        wt = np.array([float(r["wall_time"]) for r in rows])
        self.t = wt - wt[0]
        self.pos = np.stack(
            [[float(r["meas_x"]), float(r["meas_y"]), float(r["meas_z"])] for r in rows]
        )
        sel = np.ones(len(self.t), bool)
        if t0 is not None:
            sel &= self.t >= float(t0)
        if t1 is not None:
            sel &= self.t <= float(t1)
        self.t, self.pos = self.t[sel], self.pos[sel]
        # de-duplicate: the log samples a slower packet stream at loop rate, so
        # consecutive rows repeat. Keep only genuine packet updates.
        keep = [0]
        for i in range(1, len(self.pos)):
            if not np.allclose(self.pos[i], self.pos[keep[-1]]):
                keep.append(i)
        self.t, self.pos = self.t[keep], self.pos[keep]
        d = np.gradient(self.pos, self.t, axis=0) if len(self.t) > 2 else np.zeros_like(self.pos)
        self.vel = d

    def __iter__(self):
        for i in range(len(self.t)):
            yield float(self.t[i]), self.pos[i].copy(), self.vel[i].copy()


class MavlinkTruth:
    """Live GLOBAL_POSITION_INT from the plane, converted to a local NED frame
    anchored on the plane's first fix (no pursuer connection needed)."""

    def __init__(self, conn_str, rate_hz=20.0, duration_s=None, sysid=None,
                 request_hz=15.0):
        from pymavlink import mavutil
        import mavlink_utils

        self.name = f"mavlink {conn_str}"
        self.rate_hz = float(rate_hz)
        self.duration = duration_s
        self._mavutil = mavutil
        self._mavlink_utils = mavlink_utils

        print(f"[rig] connecting to target: {conn_str}")
        self.conn = mavutil.mavlink_connection(conn_str)
        hb = self.conn.wait_heartbeat(timeout=15)
        if hb is None:
            raise SystemExit("no heartbeat from target")
        print(f"[rig] heartbeat sys={self.conn.target_system} comp={self.conn.target_component}")

        try:  # ask for a decent stream rate
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
                1e6 / max(request_hz, 1.0), 0, 0, 0, 0, 0,
            )
        except Exception as exc:
            print(f"[rig] SET_MESSAGE_INTERVAL failed (continuing): {exc}")

        print("[rig] waiting for first GLOBAL_POSITION_INT to anchor the NED origin...")
        msg = None
        while msg is None:
            msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=15)
            if msg is None:
                raise SystemExit("no GLOBAL_POSITION_INT from target")
            if sysid is not None and int(msg.get_srcSystem()) != int(sysid):
                msg = None
        self.home = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
        self.sysid = sysid
        print(f"[rig] origin lat={self.home[0]:.7f} lon={self.home[1]:.7f} alt={self.home[2]:.1f}")

    def __iter__(self):
        t0 = time.monotonic()
        while True:
            msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0)
            now = time.monotonic() - t0
            if self.duration is not None and now > self.duration:
                return
            if msg is None:
                print("[rig] target telemetry stale...")
                continue
            if self.sysid is not None and int(msg.get_srcSystem()) != int(self.sysid):
                continue
            pos, vel = self._mavlink_utils.parse_global_int(msg, *self.home)
            yield now, np.asarray(pos, dtype=float), np.asarray(vel, dtype=float)


# ======================================================================
#  The virtual rig
# ======================================================================
class VirtualStereoRig:
    """Two cameras that emit noisy bearings to a known target position."""

    def __init__(self, cameras, sigma_yaw_deg=None, sigma_pitch_deg=None,
                 dropout_prob=0.0, outlier_prob=0.0, outlier_sigma_mult=25.0,
                 bias_deg=0.0, seed=0):
        self.cameras = cameras
        self.rng = np.random.default_rng(seed)
        self.dropout_prob = float(dropout_prob)
        self.outlier_prob = float(outlier_prob)
        self.outlier_sigma_mult = float(outlier_sigma_mult)
        for cam in self.cameras:
            if sigma_yaw_deg is not None:
                cam.sigma_yaw = float(np.radians(sigma_yaw_deg))
            if sigma_pitch_deg is not None:
                cam.sigma_pitch = float(np.radians(sigma_pitch_deg))
        # A boresight bias on ONE camera only -- a symmetric bias is mostly
        # unobservable, while a differential one is exactly what shows up as
        # persistent ray skew.
        self.true_bias = np.zeros((len(self.cameras), 2))
        if abs(bias_deg) > 0.0 and len(self.cameras) > 1:
            self.true_bias[1, 0] = float(np.radians(bias_deg))
            self.true_bias[1, 1] = float(np.radians(bias_deg)) * 0.5
        self.stats = {"emitted": 0, "dropped": 0, "outliers": 0, "out_of_fov": 0}

    @staticmethod
    def build_aimed(rig_center, aim_point, baseline_m, cfg_cameras,
                    sigma_yaw_deg=None, sigma_pitch_deg=None, tilt_deg=0.0):
        """Place a stereo pair symmetric about rig_center, baseline
        PERPENDICULAR to the line of sight (maximising parallax, hence depth
        observability), both cameras aimed at aim_point.

        tilt_deg rotates the baseline about the line of sight, out of
        horizontal. This is not cosmetic: with a purely horizontal baseline a
        differential YAW misalignment is structurally unobservable -- two yaw
        measurements against two horizontal unknowns leaves no redundancy, so
        the error is absorbed silently as a depth shift. Tilting the baseline
        mixes yaw into the pitch-constrained direction and makes it observable.
        """
        rig_center = np.asarray(rig_center, dtype=float).reshape(3)
        aim_point = np.asarray(aim_point, dtype=float).reshape(3)
        los = aim_point - rig_center
        hn = float(np.hypot(los[0], los[1]))
        if hn < _EPS:
            perp = np.array([0.0, 1.0, 0.0])
        else:
            perp = np.array([-los[1] / hn, los[0] / hn, 0.0])
        if abs(tilt_deg) > 1e-9:
            u = los / max(np.linalg.norm(los), _EPS)
            w = np.cross(u, perp)
            wn = np.linalg.norm(w)
            if wn > _EPS:
                w = w / wn
                a = math.radians(tilt_deg)
                perp = perp * math.cos(a) + w * math.sin(a)

        cams = []
        for i, base in enumerate(cfg_cameras[:2]):
            sign = -1.0 if i == 0 else 1.0
            pos = rig_center + sign * (baseline_m / 2.0) * perp
            d = aim_point - pos
            yaw = math.degrees(math.atan2(d[1], d[0]))
            pitch = math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
            cfg = dict(base)
            cfg.update({"position_ned": tuple(pos), "yaw_deg": yaw,
                        "pitch_deg": pitch, "roll_deg": 0.0})
            if sigma_yaw_deg is not None:
                cfg["sigma_yaw_deg"] = sigma_yaw_deg
            if sigma_pitch_deg is not None:
                cfg["sigma_pitch_deg"] = sigma_pitch_deg
            cams.append(sg.Camera.from_config(cfg))
        return cams

    def slew_to(self, aim_point, dt, max_rate_deg_s=60.0):
        """Point both cameras at aim_point, rate-limited (gimbal, fixed mount).

        Only the orientations move -- the rig stays bolted down, so the
        baseline is unchanged and the extrinsics remain exactly known (a real
        gimbal reports encoder angles).
        """
        aim_point = np.asarray(aim_point, dtype=float).reshape(3)
        max_step = float("inf") if max_rate_deg_s <= 0 else abs(max_rate_deg_s) * max(dt, 0.0)
        for cam in self.cameras:
            d = aim_point - cam.position
            want_yaw = math.degrees(math.atan2(d[1], d[0]))
            want_pitch = math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
            cur_yaw = math.degrees(cam.yaw)
            cur_pitch = math.degrees(cam.pitch)
            dyaw = (want_yaw - cur_yaw + 180.0) % 360.0 - 180.0
            dpitch = want_pitch - cur_pitch
            dyaw = float(np.clip(dyaw, -max_step, max_step))
            dpitch = float(np.clip(dpitch, -max_step, max_step))
            cam.set_orientation(cur_yaw + dyaw, cur_pitch + dpitch, math.degrees(cam.roll))

    def observe(self, target_pos, stamp):
        """-> list of Detection (0, 1 or 2 of them)."""
        dets = []
        for i, cam in enumerate(self.cameras):
            if not cam.sees(target_pos):
                self.stats["out_of_fov"] += 1
                continue
            if self.rng.random() < self.dropout_prob:
                self.stats["dropped"] += 1
                continue
            yaw_t, pitch_t = cam.bearing_of(target_pos)
            if self.rng.random() < self.outlier_prob:
                m = self.outlier_sigma_mult
                self.stats["outliers"] += 1
                meta = {"outlier": True}
            else:
                m = 1.0
                meta = {}
            yaw = yaw_t + self.rng.normal(0.0, cam.sigma_yaw * m) + self.true_bias[i, 0]
            pitch = pitch_t + self.rng.normal(0.0, cam.sigma_pitch * m) + self.true_bias[i, 1]
            meta["range_true_m"] = cam.range_to(target_pos)
            dets.append(sg.Detection(i, yaw, pitch, stamp, valid=True, meta=meta))
            self.stats["emitted"] += 1
        return dets


# ======================================================================
#  Scoring
# ======================================================================
def summarize(records, rig_center, label_width=26):
    if not records:
        print("no scored frames")
        return {}
    R = {k: np.array([r[k] for r in records], dtype=float) for k in
         ("t", "err", "err_along", "err_cross", "range",
          "tri_err", "tri_along", "tri_cross",
          "sig_along", "sig_cross", "verr", "skew", "nis_max")}
    tracked = np.array([r["tracked"] for r in records], dtype=bool)
    ok = tracked & np.isfinite(R["err"])
    tri_ok = np.isfinite(R["tri_err"])

    def line(name, a, b, c):
        v = lambda z: "   --  " if not np.any(np.isfinite(z)) else f"{np.nanmedian(z):7.2f}"
        p = lambda z: "   --  " if not np.any(np.isfinite(z)) else f"{np.nanpercentile(z,90):7.2f}"
        print(f"  {name:<{label_width}} {v(a)} {p(a)} | {v(b)} {p(b)} | {v(c)} {p(c)}")

    print("\n" + "=" * 78)
    print(f"  SCORE  ({int(ok.sum())} tracked frames of {len(records)})")
    print("=" * 78)
    print(f"  {'':<{label_width}} {'total med':>7} {'p90':>7} | {'along-LOS':>7} {'p90':>7} | "
          f"{'cross-LOS':>7} {'p90':>7}")
    print(f"  {'-'*(label_width+50)}")
    line("IMM (bearings-only) [m]", R["err"][ok], R["err_along"][ok], R["err_cross"][ok])
    line("triangulation-only  [m]", R["tri_err"][tri_ok], R["tri_along"][tri_ok], R["tri_cross"][tri_ok])
    if np.any(ok) and np.any(tri_ok):
        g_a = np.nanmedian(R["tri_along"][tri_ok]) / max(np.nanmedian(R["err_along"][ok]), 1e-9)
        g_c = np.nanmedian(R["tri_cross"][tri_ok]) / max(np.nanmedian(R["err_cross"][ok]), 1e-9)
        print(f"  {'-> IMM improvement':<{label_width}} {'':>7} {'':>7} | {g_a:6.1f}x {'':>7} | {g_c:6.1f}x")

    print(f"\n  range to rig      : median {np.nanmedian(R['range'][ok]):.0f} m "
          f"[{np.nanmin(R['range'][ok]):.0f}-{np.nanmax(R['range'][ok]):.0f}]")
    if np.any(np.isfinite(R["verr"][ok])):
        print(f"  velocity error    : median {np.nanmedian(R['verr'][ok]):.2f} m/s, "
              f"p90 {np.nanpercentile(R['verr'][ok],90):.2f}")
    print(f"  ray skew          : median {np.nanmedian(R['skew']):.2f} m, "
          f"p90 {np.nanpercentile(R['skew'],90):.2f}")

    # Calibration: does the filter's own sigma match the error it actually
    # makes? Overconfidence here is the same failure the position-era R sweep
    # exposed -- and it silently disables the covariance gate downstream.
    print("\n  --- covariance calibration (|error| / reported sigma; ~1.0 is honest) ---")
    for axis, e, s in (("along-LOS", R["err_along"][ok], R["sig_along"][ok]),
                       ("cross-LOS", R["err_cross"][ok], R["sig_cross"][ok])):
        m = np.isfinite(e) & np.isfinite(s) & (s > 1e-6)
        if np.any(m):
            ratio = np.abs(e[m]) / s[m]
            verdict = ("well calibrated" if 0.5 <= np.median(ratio) <= 2.0
                       else ("OVERCONFIDENT" if np.median(ratio) > 2.0 else "conservative"))
            print(f"    {axis}: sigma med {np.nanmedian(s[m]):7.2f} m, "
                  f"|err|/sigma med {np.median(ratio):5.2f}  -> {verdict}")
    return R


# ======================================================================
#  Main
# ======================================================================
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Virtual stereo camera rig + bearings-only tracker test bench",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--source", choices=("mavlink", "log", "synthetic"), default="synthetic")
    p.add_argument("--target", default="udpin:localhost:14600", help="MAVLink target connection")
    p.add_argument("--sysid", type=int, default=None, help="expected target sysid")
    p.add_argument("--log", default=None, help="guided_follow_*.csv for --source log")
    p.add_argument("--t0", type=float, default=None, help="log start time [s]")
    p.add_argument("--t1", type=float, default=None, help="log end time [s]")
    p.add_argument("--duration", type=float, default=120.0, help="synthetic/mavlink duration [s]")
    p.add_argument("--rate", type=float, default=20.0, help="synthetic frame rate [Hz]")

    p.add_argument("--baseline", type=float, default=scfg.BASELINE_M, help="stereo baseline [m]")
    p.add_argument("--rig-north", type=float, default=0.0, help="rig centre north [m]")
    p.add_argument("--rig-east", type=float, default=0.0, help="rig centre east [m]")
    p.add_argument("--rig-alt", type=float, default=scfg.CAMERA_ALTITUDE_M, help="rig height [m]")
    p.add_argument("--no-auto-aim", action="store_true",
                   help="keep the config's camera angles instead of aiming at the track")
    p.add_argument("--sigma-deg", type=float, default=None,
                   help="override BOTH per-axis angular sigmas [deg]")
    p.add_argument("--fov", type=float, default=None,
                   help="override both cameras' FOV (yaw and pitch) [deg]")
    p.add_argument("--slew", action="store_true",
                   help="gimballed rig: re-aim both cameras at the current track "
                        "estimate every frame (cameras stay bolted in place). "
                        "This is how a real long-range tracker keeps a narrow "
                        "FOV on a target that crosses the whole sky.")
    p.add_argument("--slew-rate", type=float, default=60.0,
                   help="max gimbal slew rate [deg/s]; 0 = instant")
    p.add_argument("--baseline-tilt", type=float, default=0.0,
                   help="rotate the baseline out of horizontal about the LOS "
                        "[deg]. 90 = fully vertical baseline. Makes differential "
                        "yaw misalignment observable (see build_aimed).")

    p.add_argument("--dropout", type=float, default=0.0, help="per-camera per-frame dropout prob")
    p.add_argument("--outlier-rate", type=float, default=0.0, help="false-detection probability")
    p.add_argument("--bias-deg", type=float, default=0.0,
                   help="boresight bias injected on camera 2 [deg]")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--calibrate", action="store_true",
                   help="enable online differential boresight-bias estimation")
    p.add_argument("--calib-tau", type=float, default=scfg.BORESIGHT_BIAS_TAU_S,
                   help="boresight estimator time constant [s]")

    p.add_argument("--out", default=None, help="write a per-frame CSV here")
    p.add_argument("--plot", action="store_true", help="save a PNG summary")
    p.add_argument("--realtime", action="store_true", help="pace synthetic/log to wall clock")
    p.add_argument("--quiet", action="store_true")
    return p.parse_args(argv)


def build_truth(args):
    if args.source == "synthetic":
        return SyntheticTruth(duration_s=args.duration, rate_hz=args.rate, seed=args.seed)
    if args.source == "log":
        if not args.log:
            raise SystemExit("--source log requires --log <csv>")
        return LogTruth(args.log, t0=args.t0, t1=args.t1)
    return MavlinkTruth(args.target, rate_hz=args.rate,
                        duration_s=args.duration, sysid=args.sysid)


def preview_aim_point(truth, args):
    """Pick an aim point so the target is actually inside the FOV.

    For finite sources this is the track centroid; for live MAVLink we cannot
    look ahead, so the first fix is used and the FOV is widened.
    """
    if isinstance(truth, MavlinkTruth):
        return None
    pts = []
    for i, (_, pos, _) in enumerate(truth):
        pts.append(pos)
        if i > 20000:
            break
    return np.mean(np.asarray(pts), axis=0) if pts else None


def main(argv=None):
    args = parse_args(argv)
    truth = build_truth(args)
    rig_center = np.array([args.rig_north, args.rig_east, -args.rig_alt], dtype=float)

    # --- place the cameras ------------------------------------------------
    if args.no_auto_aim:
        cameras = sg.build_cameras(scfg.CAMERAS)
        aim = None
    else:
        aim = preview_aim_point(truth, args)
        if aim is None:  # live: aim at the first sample, and open the FOV up
            first = next(iter(truth))
            aim = first[1]
            wide = []
            for c in scfg.CAMERAS:
                c = dict(c); c["fov_yaw_deg"] = 170.0; c["fov_pitch_deg"] = 170.0
                wide.append(c)
            base_cfgs = wide
        else:
            base_cfgs = scfg.CAMERAS
        cameras = VirtualStereoRig.build_aimed(
            rig_center, aim, args.baseline, base_cfgs,
            sigma_yaw_deg=args.sigma_deg, sigma_pitch_deg=args.sigma_deg,
            tilt_deg=args.baseline_tilt,
        )
    if args.fov is not None:
        for cam in cameras:
            cam.half_fov_yaw = float(np.radians(args.fov)) / 2.0
            cam.half_fov_pitch = float(np.radians(args.fov)) / 2.0

    rig = VirtualStereoRig(
        cameras, sigma_yaw_deg=args.sigma_deg, sigma_pitch_deg=args.sigma_deg,
        dropout_prob=args.dropout, outlier_prob=args.outlier_rate,
        bias_deg=args.bias_deg, seed=args.seed,
    )
    tracker = StereoTracker(cameras, cfg=scfg)
    tri_only = TriangulationOnlyTracker(cameras, cfg=scfg)
    if args.calibrate:
        tracker.boresight.enabled = True
        tracker.boresight.tau = float(args.calib_tau)

    print("=" * 78)
    print(f"  VIRTUAL STEREO RIG   source: {truth.name}")
    print("=" * 78)
    for cam in cameras:
        print(f"  {cam}")
    print(f"  baseline {args.baseline:.1f} m | rig at N{rig_center[0]:.0f} E{rig_center[1]:.0f} "
          f"alt {-rig_center[2]:.0f} m" + ("" if aim is None else
          f" | aimed at N{aim[0]:.0f} E{aim[1]:.0f} alt {-aim[2]:.0f}"))
    print(f"  noise {np.degrees(cameras[0].sigma_yaw)*1000:.1f} mdeg "
          f"({np.degrees(cameras[0].sigma_yaw)*17.45:.2f} mrad) | dropout {args.dropout:.0%} | "
          f"outliers {args.outlier_rate:.1%} | bias {args.bias_deg:.3f} deg")
    print(f"  gate: {scfg.NIS_GATE_MODE} @ chi2 {scfg.NIS_GATE_CHI2}")
    print("-" * 78)

    records = []
    t_init = None
    last_print = 0.0
    prev_stamp = None
    wall0 = time.monotonic()

    for stamp, pos_true, vel_true in truth:
        if args.realtime:
            target_wall = wall0 + stamp
            slp = target_wall - time.monotonic()
            if slp > 0:
                time.sleep(min(slp, 0.5))

        # Gimbal: point at where WE think the target is, never at the truth --
        # a rig that cheats here would flatter the whole test.
        if args.slew:
            dt_frame = 0.05 if prev_stamp is None else max(stamp - prev_stamp, 0.0)
            last_pos = tracker.last_diag.get("position") if tracker.last_diag else None
            if last_pos is not None:
                rig.slew_to(last_pos, dt_frame, args.slew_rate)
        prev_stamp = stamp

        dets = rig.observe(pos_true, stamp)
        snap = tracker.process(dets, stamp)
        tri = tri_only.process(dets, stamp)

        if t_init is None and snap["tracking"]:
            t_init = stamp
            print(f"  [t={stamp:7.2f}s] track initialised, "
                  f"range {np.linalg.norm(pos_true - rig_center):.0f} m")

        los = pos_true - rig_center
        rec = {
            "t": stamp, "tracked": bool(snap["tracking"]), "state": snap["state"],
            "range": float(np.linalg.norm(los)),
            "err": np.nan, "err_along": np.nan, "err_cross": np.nan, "verr": np.nan,
            "tri_err": np.nan, "tri_along": np.nan, "tri_cross": np.nan,
            "sig_along": np.nan, "sig_cross": np.nan,
            "skew": np.nan, "nis_max": np.nan, "n_det": len(dets),
            "true_x": pos_true[0], "true_y": pos_true[1], "true_z": pos_true[2],
            "est_x": np.nan, "est_y": np.nan, "est_z": np.nan,
        }
        if snap["tracking"] and snap["position"] is not None:
            e = snap["position"] - pos_true
            a, c = sg.los_decompose(e, los)
            rec.update({"err": float(np.linalg.norm(e)), "err_along": abs(a), "err_cross": c,
                        "est_x": snap["position"][0], "est_y": snap["position"][1],
                        "est_z": snap["position"][2],
                        "sig_along": snap.get("sigma_along_los_m", np.nan),
                        "sig_cross": snap.get("sigma_cross_los_m", np.nan)})
            if vel_true is not None and snap["velocity"] is not None:
                rec["verr"] = float(np.linalg.norm(snap["velocity"] - vel_true))
        if tri["tracking"] and tri["position"] is not None:
            e = tri["position"] - pos_true
            a, c = sg.los_decompose(e, los)
            rec.update({"tri_err": float(np.linalg.norm(e)), "tri_along": abs(a), "tri_cross": c})
        if snap.get("geom"):
            rec["skew"] = snap["geom"]["skew_m"]
        if snap.get("meas"):
            rec["nis_max"] = snap["meas"].get("nis_max", np.nan)
        records.append(rec)

        if not args.quiet and stamp - last_print >= scfg.DIAG_PRINT_INTERVAL_S:
            last_print = stamp
            if snap["tracking"]:
                print(f"  t={stamp:7.2f} {snap['state']:<5} R={rec['range']:6.0f}m "
                      f"err={rec['err']:6.2f}m (along {rec['err_along']:6.2f} "
                      f"cross {rec['err_cross']:5.2f}) "
                      f"sig_a={rec['sig_along']:6.2f} skew={rec['skew']:5.2f} "
                      f"det={len(dets)}")
            else:
                print(f"  t={stamp:7.2f} {snap['state']:<5} (no track) det={len(dets)}")

    # --- report -----------------------------------------------------------
    print("-" * 78)
    print(f"  frames {tracker.frames} | updates {tracker.updates} | "
          f"re-inits {tracker.reinits} | init at t={t_init if t_init is not None else float('nan'):.2f}s")
    print(f"  detections emitted {rig.stats['emitted']} | dropped {rig.stats['dropped']} | "
          f"outliers injected {rig.stats['outliers']} | out-of-FOV {rig.stats['out_of_fov']}")
    if args.calibrate:
        inj = np.degrees(rig.true_bias)
        est = tracker.boresight.report_deg()
        # Only the differential is observable, so score the differential.
        inj_d = inj[1] - inj[0]
        est_d = est[1] - est[0]
        print(f"  boresight (differential, deg): injected yaw {inj_d[0]:+.4f} pitch {inj_d[1]:+.4f}"
              f" | recovered yaw {est_d[0]:+.4f} pitch {est_d[1]:+.4f}"
              f" | residual yaw {inj_d[0]-est_d[0]:+.4f} pitch {inj_d[1]-est_d[1]:+.4f}")
    summarize(records, rig_center)

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(records[0].keys()))
            w.writeheader()
            w.writerows(records)
        print(f"\n  per-frame CSV -> {args.out}")

    if args.plot:
        _plot(records, args)
    return records


def _plot(records, args):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"  (plot skipped: {exc})")
        return
    t = np.array([r["t"] for r in records])
    g = lambda k: np.array([r[k] for r in records], dtype=float)

    plt.style.use("dark_background")
    fig, ax = plt.subplots(2, 2, figsize=(15, 9))
    fig.patch.set_facecolor("#0f1117")
    for a in ax.ravel():
        a.set_facecolor("#151823"); a.grid(True, color="#2b3140", lw=0.5)
        a.tick_params(colors="#8b93a7")
        for s in a.spines.values():
            s.set_color("#2b3140")

    ax[0, 0].plot(g("true_y"), g("true_x"), color="#e6e9ef", lw=2, label="truth")
    ax[0, 0].plot(g("est_y"), g("est_x"), color="#3ddc84", lw=1.2, label="IMM estimate")
    ax[0, 0].scatter([args.rig_east], [args.rig_north], marker="^", s=120,
                     color="#ffb454", label="rig", zorder=5)
    ax[0, 0].set_title("track (north vs east)", color="#e6e9ef")
    ax[0, 0].set_xlabel("east [m]", color="#8b93a7"); ax[0, 0].set_ylabel("north [m]", color="#8b93a7")
    ax[0, 0].legend(facecolor="#1e2230", edgecolor="#2b3140", labelcolor="#e6e9ef", fontsize=8)
    ax[0, 0].set_aspect("equal", "datalim")

    ax[0, 1].semilogy(t, g("tri_err"), color="#8b93a7", lw=0.7, label="triangulation-only")
    ax[0, 1].semilogy(t, g("err"), color="#3ddc84", lw=1.2, label="IMM bearings-only")
    ax[0, 1].set_title("position error", color="#e6e9ef")
    ax[0, 1].set_xlabel("t [s]", color="#8b93a7"); ax[0, 1].set_ylabel("|error| [m]", color="#8b93a7")
    ax[0, 1].legend(facecolor="#1e2230", edgecolor="#2b3140", labelcolor="#e6e9ef", fontsize=8)

    ax[1, 0].plot(t, g("err_along"), color="#ff5c6c", lw=1.0, label="along-LOS (depth) err")
    ax[1, 0].plot(t, g("sig_along"), color="#ff5c6c", lw=1.0, ls="--", alpha=0.7, label="reported sigma")
    ax[1, 0].plot(t, g("err_cross"), color="#4f9cff", lw=1.0, label="cross-LOS err")
    ax[1, 0].plot(t, g("sig_cross"), color="#4f9cff", lw=1.0, ls="--", alpha=0.7, label="reported sigma")
    ax[1, 0].set_yscale("log")
    ax[1, 0].set_title("error vs reported sigma, split by axis", color="#e6e9ef")
    ax[1, 0].set_xlabel("t [s]", color="#8b93a7"); ax[1, 0].set_ylabel("[m]", color="#8b93a7")
    ax[1, 0].legend(facecolor="#1e2230", edgecolor="#2b3140", labelcolor="#e6e9ef", fontsize=8)

    ax[1, 1].plot(t, g("range"), color="#ffb454", lw=1.0, label="range to rig")
    ax[1, 1].set_ylabel("range [m]", color="#ffb454")
    a2 = ax[1, 1].twinx()
    a2.plot(t, g("skew"), color="#26c6a2", lw=0.8, label="ray skew")
    a2.set_ylabel("skew [m]", color="#26c6a2"); a2.tick_params(colors="#26c6a2")
    ax[1, 1].set_title("geometry: range and ray skew", color="#e6e9ef")
    ax[1, 1].set_xlabel("t [s]", color="#8b93a7")

    fig.suptitle("Virtual stereo rig -- bearings-only IMM tracking", color="#e6e9ef", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = args.out.replace(".csv", ".png") if args.out else "stereo_rig_test.png"
    fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
    print(f"  plot -> {out}")


if __name__ == "__main__":
    main()
