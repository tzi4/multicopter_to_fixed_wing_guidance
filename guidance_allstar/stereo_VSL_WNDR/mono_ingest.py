"""Ingest for the ``mono_log.txt`` format (25minfirst session, 2026-07-30).

Unlike the allstar logs, this log carries the complete pipeline state per
triangulation event: both cameras' rays with boresight applied
(``C*_RayHeading/RayPitch``), the pixel/intrinsics chain that produced them,
VSL's instant triangulation (``Calc_*``), its low-pass output (``LPF_*``) and
the raw-mode alternative (``RawMode_*``). MAVLink target telemetry was off for
the whole session (``MAV_Fix_Type`` is empty on every row), so truth comes from
the target's dataflash .bin logs alone.

Time bases: ``Timestamp`` is wall clock at logging; ``Loc_TS`` is the
latency-compensated time the localization refers to (0.4 s older); each
camera's ``C*_RayRealTS`` is its own capture time. Angles are scored at the
per-camera time, 3D fixes at ``Loc_TS``. The ``DateTime`` column is local
(UTC+3); user-supplied window times are given on that clock.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from vsl_ingest import EnuFrame

DEFAULT_VSL_ROOT = os.environ.get(
    "VSL_DATA_ROOT", os.path.expanduser("~/savasan_iha_yildizlar_data/VSL/logs"))
DEFAULT_CACHE_DIR = os.environ.get(
    "VSL_CACHE_DIR", os.path.expanduser("~/.cache/savasan_iha_yildizlar/truth"))
MONO_LOG = os.environ.get(
    "VSL_MONO_LOG", os.path.join(DEFAULT_VSL_ROOT, "25minfirst", "mono_log.txt"))

# hand-picked good segments (user, local HH:MM:SS on 2026-07-30)
GOOD_WINDOWS = [
    ("20:02:20", "20:03:35"),
    ("20:03:52", "20:05:10"),
    ("20:05:35", "20:07:30"),
    ("20:08:13", "20:14:35"),
    ("20:15:29", "20:19:56"),
    ("20:34:56", "20:35:13"),
    ("20:35:27", "20:35:42"),
    ("20:36:26", "20:41:33"),
]

_FIELDS = {
    "t_wall": "Timestamp",
    "loc_ts": "Loc_TS",
    "spread_ms": "Spread_ms",
    "calc_e": "Calc_E", "calc_n": "Calc_N", "calc_u": "Calc_U",
    "calc_dist": "Calc_Dist",
    "lpf_e": "LPF_E", "lpf_n": "LPF_N", "lpf_u": "LPF_U",
    "raw_e": "RawMode_E", "raw_n": "RawMode_N", "raw_u": "RawMode_U",
    "c1_head": "C1_RayHeading", "c1_pitch": "C1_RayPitch",
    "c2_head": "C2_RayHeading", "c2_pitch": "C2_RayPitch",
    "c1_ts": "C1_RayRealTS", "c2_ts": "C2_RayRealTS",
    "c1_zoom": "C1_Zoom", "c2_zoom": "C2_Zoom",
    "c1_fx": "C1_fx", "c2_fx": "C2_fx",
    "c1_score": "T1_Score", "c2_score": "T2_Score",
    "c1_bore_yaw": "C1_BoreYawOffset", "c1_bore_pitch": "C1_BorePitchOffset",
    "c2_bore_yaw": "C2_BoreYawOffset", "c2_bore_pitch": "C2_BorePitchOffset",
}


@dataclass
class MonoLog:
    """All TRIANGULATION rows of one mono_hub session, as columns."""

    path: str
    config: Dict
    cols: Dict[str, np.ndarray]
    cam1_lla: Tuple[float, float, float]
    cam2_lla: Tuple[float, float, float]
    local_utc_offset_s: float

    def __len__(self) -> int:
        return int(self.cols["loc_ts"].size)

    @property
    def frame(self) -> EnuFrame:
        """ENU frame at CAM1 -- verified to match the log's Calc_E/N/U origin."""
        return EnuFrame(*self.cam1_lla)

    def local_to_unix(self, hhmmss: str, date: str = "2026-07-30") -> float:
        dt = datetime.strptime(f"{date} {hhmmss}", "%Y-%m-%d %H:%M:%S")
        epoch = datetime(1970, 1, 1)
        return (dt - epoch).total_seconds() - self.local_utc_offset_s

    def window_mask(self, start: str, end: str) -> np.ndarray:
        t0, t1 = self.local_to_unix(start), self.local_to_unix(end)
        return (self.cols["loc_ts"] >= t0) & (self.cols["loc_ts"] <= t1)


def _f(x: str) -> float:
    x = x.strip()
    if not x or x == "-":
        return math.nan
    try:
        return float(x)
    except ValueError:
        return math.nan


def read_mono(path: str = MONO_LOG, verbose: bool = False) -> MonoLog:
    import json

    config: Dict = {}
    name_to_idx: Optional[Dict[str, int]] = None
    idx: Dict[str, int] = {}
    data: Dict[str, List[float]] = {k: [] for k in _FIELDS}
    cam_idx: Dict[str, int] = {}
    cam_vals: Dict[str, float] = {}
    offset: Optional[float] = None

    with open(path, "r", errors="replace") as fh:
        for line in fh:
            if line.startswith("#"):
                brace = line.find("{")
                if brace >= 0 and not config:
                    try:
                        config = json.loads(line[brace:])
                    except json.JSONDecodeError:
                        pass
                continue
            if line.startswith("Timestamp"):
                if name_to_idx is None:
                    names = [c.strip() for c in line.split("|")]
                    name_to_idx = {n: i for i, n in enumerate(names)}
                    idx = {k: name_to_idx[v] for k, v in _FIELDS.items()}
                    for k in ("C1_GPS_Lat", "C1_GPS_Lon", "C1_GPS_Alt",
                              "C2_GPS_Lat", "C2_GPS_Lon", "C2_GPS_Alt"):
                        cam_idx[k] = name_to_idx[k]
                    ev_i = name_to_idx["Event_Type"]
                    dt_i = name_to_idx["DateTime"]
                    ts_i = name_to_idx["Timestamp"]
                continue
            if name_to_idx is None:
                continue
            cells = line.split("|")
            if len(cells) < len(name_to_idx):
                continue
            if cells[ev_i].strip() != "TRIANGULATION":
                continue
            if offset is None:
                ts = _f(cells[ts_i])
                local = datetime.strptime(cells[dt_i].strip(),
                                          "%Y-%m-%d %H:%M:%S.%f")
                offset = (local - datetime(1970, 1, 1)).total_seconds() - ts
                offset = round(offset / 900.0) * 900.0
            if not cam_vals:
                for k, i in cam_idx.items():
                    cam_vals[k] = _f(cells[i])
            for k, i in idx.items():
                data[k].append(_f(cells[i]))

    cols = {k: np.array(v, dtype=float) for k, v in data.items()}
    log = MonoLog(
        path=path, config=config, cols=cols,
        cam1_lla=(cam_vals["C1_GPS_Lat"], cam_vals["C1_GPS_Lon"],
                  cam_vals["C1_GPS_Alt"]),
        cam2_lla=(cam_vals["C2_GPS_Lat"], cam_vals["C2_GPS_Lon"],
                  cam_vals["C2_GPS_Alt"]),
        local_utc_offset_s=offset or 0.0,
    )
    if verbose:
        f = log.frame
        b = f.to_enu(*log.cam2_lla)
        print(f"{os.path.basename(path)}: {len(log)} triangulation rows, "
              f"baseline {np.linalg.norm(b):.1f} m, "
              f"local-utc offset {offset/3600:+.0f} h")
    return log


def truth_tracks():
    """The two dataflash truth logs of this session."""
    from vsl_truth import parse_bin
    root = os.path.dirname(MONO_LOG)
    out = []
    for name in sorted(os.listdir(root)):
        if name.endswith(".bin"):
            tr = parse_bin(os.path.join(root, name), cache_dir=DEFAULT_CACHE_DIR)
            if tr is not None:
                out.append(tr)
    return out


def truth_enu_at(tracks, frame: EnuFrame, t: np.ndarray) -> np.ndarray:
    """Truth position in the CAM1 ENU frame at unix times t (nan if uncovered)."""
    t = np.asarray(t, dtype=float)
    out = np.full((t.size, 3), np.nan)
    for tr in tracks:
        m = tr.covers(t)
        if not m.any():
            continue
        lat, lon, alt = tr.sample(t[m])
        out[m] = np.stack([frame.to_enu(la, lo, al)
                           for la, lo, al in zip(lat, lon, alt)])
    return out


if __name__ == "__main__":
    log = read_mono(verbose=True)
    tracks = truth_tracks()
    from vsl_ingest import utc
    for tr in tracks:
        print(f"  truth {os.path.basename(tr.path):28s} "
              f"{utc(tr.span[0])} -> {utc(tr.span[1])}")
    for (a, b) in GOOD_WINDOWS:
        m = log.window_mask(a, b)
        n = int(m.sum())
        rng = np.nanmedian(log.cols["calc_dist"][m]) if n else float("nan")
        tq = log.cols["loc_ts"][m]
        cov = 0.0
        if n:
            enu = truth_enu_at(tracks, log.frame, tq)
            cov = float(np.isfinite(enu[:, 0]).mean())
        print(f"  window {a}-{b}: {n:5d} fixes  "
              f"median range {rng:6.1f} m  truth coverage {100*cov:5.1f}%")
