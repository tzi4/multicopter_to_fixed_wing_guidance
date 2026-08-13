"""Target ground truth from ArduPilot .bin dataflash logs.

The VSL flights carry the target aircraft's own flight-controller log. That log
is the only independent measurement of where the target actually was, so it is
what we score the bearings-only estimator against.

Two problems to solve:

1. **Time base.** Dataflash stamps are ``TimeUS`` (microseconds since boot),
   while the camera logs are unix epoch. ``GPS`` messages carry both
   (``TimeUS`` and ``GWk``/``GMS``), so a robust linear fit over all GPS rows
   maps one to the other. We fit rather than use a single anchor so a bad first
   fix cannot shift the whole track.

2. **Pairing.** The delivered folder names do NOT match the dates inside the
   logs (session "21" holds a 23 July flight and vice versa), so .bin files are
   matched to camera segments purely by resolved unix time overlap.

Parsing a 100 MB .bin takes ~1 min, so results are cached as .npz next to a
scratch directory.
"""

from __future__ import annotations

import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

GPS_EPOCH_UNIX = 315964800.0     # 1980-01-06 00:00:00 UTC
GPS_WEEK_SECONDS = 604800.0
GPS_LEAP_SECONDS = 18.0          # UTC = GPS - 18 s (valid since 2017-01-01)
DEFAULT_ROOT = os.environ.get(
    "VSL_DATA_ROOT", os.path.expanduser("~/savasan_iha_yildizlar_data/VSL/logs"))
DEFAULT_CACHE_DIR = os.environ.get(
    "VSL_CACHE_DIR", os.path.expanduser("~/.cache/savasan_iha_yildizlar/truth"))


@dataclass
class TruthTrack:
    """Target truth in unix time + geodetic, plus attitude if available."""

    path: str
    t: np.ndarray            # unix seconds
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray          # metres AMSL
    t_att: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    yaw: np.ndarray
    fit_rms_s: float         # residual of the TimeUS->unix fit
    n_gps: int

    def __len__(self) -> int:
        return int(self.t.size)

    @property
    def span(self) -> Tuple[float, float]:
        return float(self.t[0]), float(self.t[-1])

    def sample(self, t_query: Sequence[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Linearly interpolate (lat, lon, alt) at the requested unix times."""
        tq = np.asarray(t_query, dtype=float)
        return (np.interp(tq, self.t, self.lat),
                np.interp(tq, self.t, self.lon),
                np.interp(tq, self.t, self.alt))

    def covers(self, t_query: Sequence[float], margin: float = 0.0) -> np.ndarray:
        tq = np.asarray(t_query, dtype=float)
        return (tq >= self.t[0] - margin) & (tq <= self.t[-1] + margin)


def _gps_unix(gwk: float, gms: float) -> float:
    return GPS_EPOCH_UNIX + float(gwk) * GPS_WEEK_SECONDS + float(gms) / 1000.0 - GPS_LEAP_SECONDS


def parse_bin(path: str, cache_dir: Optional[str] = None,
              verbose: bool = False) -> Optional[TruthTrack]:
    """Extract POS/ATT from a dataflash log, resolved onto the unix clock."""
    cache = None
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        key = os.path.basename(path).replace(" ", "_") + ".npz"
        cache = os.path.join(cache_dir, key)
        if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(path):
            z = np.load(cache, allow_pickle=False)
            return TruthTrack(path, z["t"], z["lat"], z["lon"], z["alt"],
                              z["t_att"], z["roll"], z["pitch"], z["yaw"],
                              float(z["fit_rms_s"]), int(z["n_gps"]))

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(path)
    gps_us: List[float] = []
    gps_unix: List[float] = []
    pos_us: List[float] = []
    pos_lla: List[Tuple[float, float, float]] = []
    att_us: List[float] = []
    att_rpy: List[Tuple[float, float, float]] = []

    while True:
        msg = conn.recv_match(type=["GPS", "POS", "ATT"], blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "GPS":
            if int(getattr(msg, "Status", 0)) >= 3 and int(getattr(msg, "GWk", 0)) > 0:
                gps_us.append(float(msg.TimeUS))
                gps_unix.append(_gps_unix(msg.GWk, msg.GMS))
        elif mtype == "POS":
            lat, lng = float(msg.Lat), float(msg.Lng)
            if abs(lat) > 1e-6 or abs(lng) > 1e-6:
                pos_us.append(float(msg.TimeUS))
                pos_lla.append((lat, lng, float(msg.Alt)))
        elif mtype == "ATT":
            att_us.append(float(msg.TimeUS))
            att_rpy.append((float(msg.Roll), float(msg.Pitch), float(msg.Yaw)))

    if len(gps_us) < 10 or len(pos_us) < 10:
        if verbose:
            print(f"   [skip] {os.path.basename(path)}: "
                  f"gps={len(gps_us)} pos={len(pos_us)}")
        return None

    # TimeUS -> unix: robust linear fit (slope ~1e-6 s/us). GPS week rollover or
    # a single bad fix would otherwise drag the whole track.
    gus = np.array(gps_us)
    gun = np.array(gps_unix)
    slope, intercept = np.polyfit(gus, gun, 1)
    resid = gun - (slope * gus + intercept)
    keep = np.abs(resid - np.median(resid)) < max(0.5, 5.0 * np.std(resid))
    if keep.sum() >= 10:
        slope, intercept = np.polyfit(gus[keep], gun[keep], 1)
        resid = gun[keep] - (slope * gus[keep] + intercept)
    fit_rms = float(np.sqrt(np.mean(resid ** 2)))

    def to_unix(us):
        return slope * np.asarray(us, dtype=float) + intercept

    pus = np.array(pos_us)
    lla = np.array(pos_lla)
    order = np.argsort(pus)
    t = to_unix(pus[order])
    lat, lon, alt = lla[order, 0], lla[order, 1], lla[order, 2]

    if att_us:
        aus = np.array(att_us)
        arp = np.array(att_rpy)
        ao = np.argsort(aus)
        t_att = to_unix(aus[ao])
        roll, pitch, yaw = arp[ao, 0], arp[ao, 1], arp[ao, 2]
    else:
        t_att = np.zeros(0)
        roll = pitch = yaw = np.zeros(0)

    track = TruthTrack(path, t, lat, lon, alt, t_att, roll, pitch, yaw,
                       fit_rms, len(gps_us))
    if cache:
        np.savez_compressed(cache, t=t, lat=lat, lon=lon, alt=alt, t_att=t_att,
                            roll=roll, pitch=pitch, yaw=yaw,
                            fit_rms_s=fit_rms, n_gps=len(gps_us))
    return track


def find_bins(root: str = DEFAULT_ROOT) -> List[str]:
    return sorted(glob.glob(os.path.join(root, "**", "*.bin"), recursive=True))


def load_all_truth(root: str = DEFAULT_ROOT,
                   cache_dir: str = DEFAULT_CACHE_DIR,
                   verbose: bool = True) -> List[TruthTrack]:
    tracks = []
    for path in find_bins(root):
        tr = parse_bin(path, cache_dir=cache_dir, verbose=verbose)
        if tr is None:
            continue
        tracks.append(tr)
        if verbose:
            from vsl_ingest import utc
            print(f"   {os.path.basename(path):32s} n={len(tr):6d} "
                  f"{utc(tr.span[0])} -> {utc(tr.span[1])} "
                  f"fit_rms={tr.fit_rms_s*1e3:.1f}ms")
    return tracks


def pair_truth_to_window(tracks: Sequence[TruthTrack], t0: float, t1: float
                         ) -> List[Tuple[TruthTrack, float]]:
    """Rank truth tracks by fractional temporal overlap with [t0, t1]."""
    out = []
    for tr in tracks:
        a, b = tr.span
        ov = max(0.0, min(b, t1) - max(a, t0))
        if ov > 0.0:
            out.append((tr, ov / max(1e-9, t1 - t0)))
    return sorted(out, key=lambda kv: -kv[1])


if __name__ == "__main__":
    import argparse

    from vsl_ingest import read_allstar, utc

    ap = argparse.ArgumentParser(description="Extract + pair target truth logs")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    args = ap.parse_args()

    print("=" * 74)
    print("TRUTH LOGS (.bin -> unix)")
    print("=" * 74)
    tracks = load_all_truth(args.root)

    print()
    print("=" * 74)
    print("PAIRING: camera segments <-> truth")
    print("=" * 74)
    for sess in ("21", "23", "27-no-target-telem"):
        path = os.path.join(args.root, sess, "allstar_cam1_log.txt")
        if not os.path.exists(path):
            continue
        cam = read_allstar(path, "CAM1")
        gaps = np.flatnonzero(np.diff(cam.t) > 60.0)
        for seg in np.split(np.arange(len(cam)), gaps + 1):
            if len(seg) < 200:
                continue
            t0, t1 = float(cam.t[seg[0]]), float(cam.t[seg[-1]])
            hits = pair_truth_to_window(tracks, t0, t1)
            tag = f"{sess}  {utc(t0)}->{utc(t1)} ({t1-t0:5.0f}s)"
            if hits:
                for tr, frac in hits[:2]:
                    print(f"{tag}  <-  {os.path.basename(tr.path):28s} "
                          f"overlap={100*frac:5.1f}%")
            else:
                print(f"{tag}  <-  (no truth)")
