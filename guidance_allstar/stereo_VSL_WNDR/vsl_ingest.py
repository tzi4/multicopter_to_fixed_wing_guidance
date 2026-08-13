"""Ingest real VSL stereo-camera logs into bearings-only Detections.

This module is a FAITHFUL REPLICA of the VSL (ITU/ITUNOM) ground-stereo
pipeline's ray construction, transcribed from their sources so that the
numbers we feed the IMM are the same numbers their own triangulator saw:

  * intrinsics       <- ``stereo_shared.get_interpolated_intrinsics`` +
                        ``apply_digital_zoom_intrinsics`` + ``resize_intrinsics``
                        over ``system_variables.CAM_PARAMS``
  * world angles     <- ``mono_hub._world_gimbal_angles_for_sample``
                        (pitch += bore_pitch, yaw += heading + bore_yaw)
  * ENU ray          <- ``mono_hub._selfcal_ray_hp`` /
                        ``stereo_shared.calculate_camera_rotation_matrix``

Replicating rather than importing is deliberate: their tree is a Qt/Redis
application (``mono_hub.py`` is 400 kB and imports the world), and we only need
~60 lines of geometry. ``validate_against_selfcal()`` proves the replica agrees
with their logged H/P to floating-point noise, so the transcription is checked
rather than trusted.

Log families consumed
---------------------
``allstar_camN_log.txt``  per-camera detection stream (the primary source):
    System_Timestamp, Gimbal_Lat/Lon/Alt, Gimbal_Heading, Encoder_Pitch/Roll/Yaw,
    Target_Pixel_X/Y, FOV_Deg, Zoom, Target_Status, ...
``selfcal_log.txt``       VSL's boresight self-cal EKF trace. Carries both the
    inputs (U,V,Pitch,Roll,Yaw,Head,Z,BoreY,BoreP) and the resulting world ray
    angles (H1,P1,H2,P2) -> used as the ground truth for the replica.

Note on the encoder convention: ``allstar`` logs the raw SIYI encoder pitch,
which is ``wrap180(180 - elevation)``; the selfcal log carries the already
converted elevation. ``encoder_pitch_to_elevation`` does that conversion and
``validate_encoder_convention`` checks it against time-matched selfcal rows
instead of taking it on faith.
"""

from __future__ import annotations

import bisect
import datetime
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:  # scipy is used by VSL for the euler composition; fall back if absent
    from scipy.spatial.transform import Rotation as _ScipyRotation
except Exception:  # pragma: no cover
    _ScipyRotation = None


# ---------------------------------------------------------------------------
# VSL camera parameters (system_variables.py)
# ---------------------------------------------------------------------------

RESIZE_WIDTH = 1280
RESIZE_HEIGHT = 720
REAL_WIDTH_2K = 2560
REAL_HEIGHT_2K = 1440
OPTICAL_ZOOM_MAX = 10.0

# native (2560x1440) fx, fy, cx, cy per optical zoom step
CAM_PARAMS: Dict[str, Dict[float, Tuple[float, float, float, float]]] = {
    "CAM1": {
        1.0: (2.68505736e03, 2.68336781e03, 1280.0, 720.0),
        3.0: (7.19773046e03, 7.19299817e03, 1280.0, 720.0),
        5.0: (1.17185608e04, 1.17385906e04, 1280.0, 720.0),
        7.0: (1.74055697e04, 1.74407233e04, 1280.0, 720.0),
        8.0: (1.87972026e04, 1.87809410e04, 1280.0, 720.0),
        10.0: (2.72246435e04, 2.74174327e04, 1280.0, 720.0),
        15.0: (3.87e04, 3.89e04, 1280.0, 720.0),
        20.0: (5.19e04, 5.21e04, 1280.0, 720.0),
        25.0: (6.50e04, 6.53e04, 1280.0, 720.0),
        30.0: (7.82e04, 7.86e04, 1280.0, 720.0),
    },
    "CAM2": {
        1.0: (2.63585902e03, 2.63008692e03, 1280.0, 720.0),
        3.0: (6.03122200e03, 5.98952000e03, 1280.0, 720.0),
        5.0: (9.43233000e03, 9.37771000e03, 1280.0, 720.0),
        7.0: (1.37112300e04, 1.36228200e04, 1280.0, 720.0),
        8.0: (1.47595000e04, 1.46224200e04, 1280.0, 720.0),
        10.0: (2.10996512e04, 2.10557106e04, 1280.0, 720.0),
        15.0: (2.97e04, 2.96e04, 1280.0, 720.0),
        20.0: (3.96e04, 3.95e04, 1280.0, 720.0),
        25.0: (4.95e04, 4.93e04, 1280.0, 720.0),
        30.0: (5.94e04, 5.92e04, 1280.0, 720.0),
    },
}

_GIMBAL_TO_CAM = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
_ENU_TO_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563


# ---------------------------------------------------------------------------
# intrinsics
# ---------------------------------------------------------------------------

def _interpolated_intrinsics(cam_id: str, zoom: float) -> Tuple[float, float, float, float]:
    requested = float(zoom)
    optical = min(requested, OPTICAL_ZOOM_MAX)
    levels = sorted(z for z in CAM_PARAMS[cam_id] if z <= OPTICAL_ZOOM_MAX)
    if not levels:
        levels = sorted(CAM_PARAMS[cam_id])
    optical = max(min(levels), min(optical, max(levels)))
    fx = float(np.interp(optical, levels, [CAM_PARAMS[cam_id][z][0] for z in levels]))
    fy = float(np.interp(optical, levels, [CAM_PARAMS[cam_id][z][1] for z in levels]))
    cx = float(np.interp(optical, levels, [CAM_PARAMS[cam_id][z][2] for z in levels]))
    cy = float(np.interp(optical, levels, [CAM_PARAMS[cam_id][z][3] for z in levels]))
    return _apply_digital_zoom(fx, fy, cx, cy, requested)


def _apply_digital_zoom(fx, fy, cx, cy, zoom, optical_zoom_max=OPTICAL_ZOOM_MAX):
    zoom = float(zoom)
    if zoom <= optical_zoom_max or optical_zoom_max <= 0.0:
        return float(fx), float(fy), float(cx), float(cy)
    scale = zoom / optical_zoom_max
    ccx = REAL_WIDTH_2K / 2.0
    ccy = REAL_HEIGHT_2K / 2.0
    return (float(fx) * scale, float(fy) * scale,
            ccx + scale * (float(cx) - ccx), ccy + scale * (float(cy) - ccy))


def vsl_intrinsics(cam_id: str, zoom: float) -> Tuple[float, float, float, float]:
    """fx, fy, cx, cy at the 1280x720 working resolution (VSL's resize_intrinsics)."""
    fx, fy, cx, cy = _interpolated_intrinsics(cam_id, zoom)
    sx = RESIZE_WIDTH / REAL_WIDTH_2K
    sy = RESIZE_HEIGHT / REAL_HEIGHT_2K
    return fx * sx, fy * sy, cx * sx, cy * sy


# ---------------------------------------------------------------------------
# ray construction
# ---------------------------------------------------------------------------

def _euler_zyx_inv(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).inv().as_matrix()."""
    if _ScipyRotation is not None:
        return _ScipyRotation.from_euler(
            "ZYX", [yaw_deg, pitch_deg, roll_deg], degrees=True
        ).inv().as_matrix()
    cz, sz = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cy_, sy_ = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cx_, sx_ = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy_, 0.0, sy_], [0.0, 1.0, 0.0], [-sy_, 0.0, cy_]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx_, -sx_], [0.0, sx_, cx_]])
    return (rz @ ry @ rx).T


def vsl_rotation_matrix(pitch_deg: float, roll_deg: float, yaw_deg: float) -> np.ndarray:
    """ENU -> camera frame, replicating stereo_shared.calculate_camera_rotation_matrix."""
    return _GIMBAL_TO_CAM @ _euler_zyx_inv(yaw_deg, pitch_deg, roll_deg) @ _ENU_TO_NED


def vsl_ray_enu(u, v, zoom, pitch_deg, roll_deg, yaw_deg, cam_id) -> np.ndarray:
    """Unit ENU ray direction for a pixel detection (mono_hub._selfcal_ray_hp)."""
    fx, fy, cx, cy = vsl_intrinsics(cam_id, zoom)
    R = vsl_rotation_matrix(pitch_deg, roll_deg, yaw_deg)
    d_cam = np.array([(float(u) - cx) / fx, (float(v) - cy) / fy, 1.0])
    d = R.T @ d_cam
    n = float(np.linalg.norm(d))
    if not (n > 0.0) or not math.isfinite(n):
        raise ValueError("degenerate ray")
    return d / n


def enu_to_heading_pitch(d: Sequence[float]) -> Tuple[float, float]:
    """ENU unit vector -> (heading deg CW from North, pitch deg up)."""
    e, n, up = float(d[0]), float(d[1]), float(d[2])
    h = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    p = math.degrees(math.atan2(up, math.hypot(e, n)))
    return h, p


def heading_pitch_to_enu(h_deg: float, p_deg: float) -> np.ndarray:
    """(heading, pitch) -> ENU unit vector (selfcal_ekf.ray_dir)."""
    hr, pr = math.radians(h_deg), math.radians(p_deg)
    cp = math.cos(pr)
    return np.array([math.sin(hr) * cp, math.cos(hr) * cp, math.sin(pr)])


def vsl_ray_hp(u, v, zoom, pitch_deg, roll_deg, yaw_deg, cam_id) -> Tuple[float, float]:
    return enu_to_heading_pitch(vsl_ray_enu(u, v, zoom, pitch_deg, roll_deg, yaw_deg, cam_id))


def wrap180(a: float) -> float:
    return (float(a) + 180.0) % 360.0 - 180.0


def encoder_pitch_to_elevation(encoder_pitch_deg: float) -> float:
    """allstar ``Encoder_Pitch`` -> optical-axis elevation (deg, up positive).

    The SIYI encoder reports ``wrap180(180 - elevation)``; verified against
    time-matched selfcal rows on both cameras (see validate_encoder_convention).
    """
    return wrap180(180.0 - float(encoder_pitch_deg))


# ---------------------------------------------------------------------------
# geodetic <-> local ENU
# ---------------------------------------------------------------------------

class EnuFrame:
    """Local tangent-plane ENU about a geodetic origin."""

    def __init__(self, lat0_deg: float, lon0_deg: float, alt0_m: float):
        self.lat0 = float(lat0_deg)
        self.lon0 = float(lon0_deg)
        self.alt0 = float(alt0_m)
        lat = math.radians(self.lat0)
        e2 = WGS84_F * (2.0 - WGS84_F)
        s = math.sin(lat)
        rn = WGS84_A / math.sqrt(1.0 - e2 * s * s)          # prime vertical
        rm = rn * (1.0 - e2) / (1.0 - e2 * s * s)            # meridional
        self._m_per_deg_lat = rm * math.pi / 180.0
        self._m_per_deg_lon = rn * math.cos(lat) * math.pi / 180.0

    def to_enu(self, lat_deg, lon_deg, alt_m) -> np.ndarray:
        return np.array([
            (float(lon_deg) - self.lon0) * self._m_per_deg_lon,
            (float(lat_deg) - self.lat0) * self._m_per_deg_lat,
            float(alt_m) - self.alt0,
        ])

    def to_geodetic(self, enu: Sequence[float]) -> Tuple[float, float, float]:
        return (self.lat0 + float(enu[1]) / self._m_per_deg_lat,
                self.lon0 + float(enu[0]) / self._m_per_deg_lon,
                self.alt0 + float(enu[2]))


# ---------------------------------------------------------------------------
# log readers
# ---------------------------------------------------------------------------

def _read_pipe_table(path: str, wanted: Sequence[str]) -> Dict[str, List[str]]:
    """Read a VSL '|'-delimited log. Handles repeated headers and '#' comments."""
    out: Dict[str, List[str]] = {w: [] for w in wanted}
    with open(path, errors="replace") as fh:
        header = [c.strip() for c in fh.readline().split("|")]
        idx = {w: header.index(w) for w in wanted if w in header}
        missing = [w for w in wanted if w not in idx]
        if missing:
            raise KeyError(f"{path}: missing columns {missing}")
        ncol = len(header)
        for line in fh:
            if line.startswith("#"):
                continue
            parts = line.split("|")
            if len(parts) < ncol:
                continue
            head = parts[0].strip()
            if not head[:1].isdigit():
                continue  # repeated header block
            for w, i in idx.items():
                out[w].append(parts[i].strip())
    return out


def _one_float(x: str) -> float:
    """Tolerant float: VSL logs occasionally emit truncated/marked cells.

    Seen in the wild: ``-10.47180~`` (a rate-limiter truncation marker). Such a
    cell is unusable, so it becomes NaN rather than aborting the whole load.
    """
    try:
        return float(x)
    except ValueError:
        return math.nan


def _to_float(values: Sequence[str]) -> np.ndarray:
    return np.array([_one_float(x) for x in values])


@dataclass
class CamStream:
    """One camera's detection stream from an allstar log."""

    cam_id: str                 # "CAM1" / "CAM2"
    t: np.ndarray               # unix seconds
    u: np.ndarray               # pixel x @1280x720
    v: np.ndarray               # pixel y
    zoom: np.ndarray
    fov_deg: np.ndarray
    elevation: np.ndarray       # optical-axis elevation, deg up
    roll: np.ndarray
    yaw_world: np.ndarray       # Encoder_Yaw + Gimbal_Heading (boresight NOT applied)
    encoder_yaw: np.ndarray
    heading: np.ndarray
    locked: np.ndarray          # bool
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    track_id: np.ndarray
    tri_lat: np.ndarray         # VSL's own triangulation output (for comparison)
    tri_lon: np.ndarray
    tri_alt: np.ndarray

    def __len__(self) -> int:
        return int(self.t.size)

    def station_position(self) -> Tuple[float, float, float]:
        """Modal (lat, lon, alt) of the camera station."""
        keys = [(round(a, 7), round(b, 7), round(c, 2))
                for a, b, c in zip(self.lat, self.lon, self.alt)]
        vals, counts = np.unique(np.array(keys), axis=0, return_counts=True)
        return tuple(float(x) for x in vals[int(np.argmax(counts))])

    def subset(self, mask) -> "CamStream":
        m = np.asarray(mask)
        return CamStream(self.cam_id, *[getattr(self, f)[m] for f in (
            "t", "u", "v", "zoom", "fov_deg", "elevation", "roll", "yaw_world",
            "encoder_yaw", "heading", "locked", "lat", "lon", "alt", "track_id",
            "tri_lat", "tri_lon", "tri_alt")])


ALLSTAR_COLUMNS = (
    "System_Timestamp", "Gimbal_Lat", "Gimbal_Lon", "Gimbal_Alt", "Gimbal_Heading",
    "Encoder_Pitch", "Encoder_Roll", "Encoder_Yaw", "Target_Status",
    "Target_Pixel_X", "Target_Pixel_Y", "Frame_Width", "Frame_Height",
    "FOV_Deg", "Zoom", "Target_Lat", "Target_Lon", "Target_Alt", "Track_ID",
)


def read_allstar(path: str, cam_id: str) -> CamStream:
    raw = _read_pipe_table(path, ALLSTAR_COLUMNS)
    t = _to_float(raw["System_Timestamp"])
    fw = _to_float(raw["Frame_Width"])
    fh = _to_float(raw["Frame_Height"])
    u = _to_float(raw["Target_Pixel_X"])
    v = _to_float(raw["Target_Pixel_Y"])
    # Pixels are logged in frame coordinates; VSL's geometry works at 1280x720.
    with np.errstate(invalid="ignore", divide="ignore"):
        u = np.where(fw > 0, u * (RESIZE_WIDTH / np.where(fw > 0, fw, 1.0)), u)
        v = np.where(fh > 0, v * (RESIZE_HEIGHT / np.where(fh > 0, fh, 1.0)), v)
    enc_pitch = _to_float(raw["Encoder_Pitch"])
    enc_yaw = _to_float(raw["Encoder_Yaw"])
    heading = _to_float(raw["Gimbal_Heading"])
    return CamStream(
        cam_id=cam_id,
        t=t, u=u, v=v,
        zoom=_to_float(raw["Zoom"]),
        fov_deg=_to_float(raw["FOV_Deg"]),
        elevation=np.array([encoder_pitch_to_elevation(p) for p in enc_pitch]),
        roll=_to_float(raw["Encoder_Roll"]),
        yaw_world=enc_yaw + heading,
        encoder_yaw=enc_yaw,
        heading=heading,
        locked=np.array([s == "LOCKED" for s in raw["Target_Status"]]),
        lat=_to_float(raw["Gimbal_Lat"]),
        lon=_to_float(raw["Gimbal_Lon"]),
        alt=_to_float(raw["Gimbal_Alt"]),
        track_id=_to_float(raw["Track_ID"]),
        tri_lat=_to_float(raw["Target_Lat"]),
        tri_lon=_to_float(raw["Target_Lon"]),
        tri_alt=_to_float(raw["Target_Alt"]),
    )


SELFCAL_COLUMNS = (
    "Loc_TS", "T_Eval", "Now_TS", "Skew_ms",
    "U1", "V1", "Pitch1", "Roll1", "Yaw1", "Head1", "Z1", "BoreY1", "BoreP1",
    "U2", "V2", "Pitch2", "Roll2", "Yaw2", "Head2", "Z2", "BoreY2", "BoreP2",
    "H1", "P1", "H2", "P2", "IntersectAngle", "Range1", "Range2",
    "Accepted", "Reason", "Miss", "Miss_NoInterp",
    "dY1", "dP1", "dY2", "dP2", "SigY1", "SigP1", "SigY2", "SigP2",
)


def read_selfcal(path: str) -> Dict[str, np.ndarray]:
    raw = _read_pipe_table(path, SELFCAL_COLUMNS)
    out: Dict[str, np.ndarray] = {}
    for k, vals in raw.items():
        if k in ("Reason",):
            out[k] = np.array(vals)
        else:
            out[k] = _to_float(vals)
    return out


def read_selfcal_config(path: str) -> Dict[str, str]:
    """Parse the '# CONFIG ...' line of a selfcal log into a dict."""
    with open(path, errors="replace") as fh:
        for lineno, line in enumerate(fh):
            if line.startswith("# CONFIG"):
                cfg = {}
                for tok in line[len("# CONFIG"):].split():
                    if "=" in tok:
                        k, _, val = tok.partition("=")
                        cfg[k] = val
                return cfg
            if lineno > 50:
                break
    return {}


# ---------------------------------------------------------------------------
# validation of the replica against VSL's own logged output
# ---------------------------------------------------------------------------

def validate_against_selfcal(selfcal_path: str, limit: int = 20000) -> Dict[str, float]:
    """Recompute H/P from the selfcal log's own inputs and diff against its H/P.

    Agreement to ~1e-9 deg proves the transcribed intrinsics + rotation chain is
    identical to VSL's, so any later disagreement is data, not a porting bug.
    """
    d = read_selfcal(selfcal_path)
    stats: Dict[str, float] = {}
    for cam, sfx in (("CAM1", "1"), ("CAM2", "2")):
        u, v = d[f"U{sfx}"], d[f"V{sfx}"]
        pitch, roll, yaw = d[f"Pitch{sfx}"], d[f"Roll{sfx}"], d[f"Yaw{sfx}"]
        head, zoom = d[f"Head{sfx}"], d[f"Z{sfx}"]
        bory, borp = d[f"BoreY{sfx}"], d[f"BoreP{sfx}"]
        hh, pp = d[f"H{sfx}"], d[f"P{sfx}"]
        ok = np.isfinite(u) & np.isfinite(v) & np.isfinite(pitch) & np.isfinite(yaw) \
            & np.isfinite(head) & np.isfinite(zoom) & np.isfinite(hh) & np.isfinite(pp)
        idx = np.flatnonzero(ok)[:limit]
        dh, dp = [], []
        for i in idx:
            b_y = bory[i] if np.isfinite(bory[i]) else 0.0
            b_p = borp[i] if np.isfinite(borp[i]) else 0.0
            r = roll[i] if np.isfinite(roll[i]) else 0.0
            h_c, p_c = vsl_ray_hp(u[i], v[i], zoom[i],
                                  pitch[i] + b_p, r, yaw[i] + head[i] + b_y, cam)
            dh.append(abs(wrap180(h_c - hh[i])))
            dp.append(abs(p_c - pp[i]))
        if dh:
            stats[f"{cam}_n"] = float(len(dh))
            stats[f"{cam}_dh_max"] = float(np.max(dh))
            stats[f"{cam}_dp_max"] = float(np.max(dp))
            stats[f"{cam}_dh_med"] = float(np.median(dh))
            stats[f"{cam}_dp_med"] = float(np.median(dp))
    return stats


def validate_encoder_convention(allstar_path: str, selfcal_path: str, cam_sfx: str,
                                max_dt: float = 0.05, limit: int = 4000) -> Dict[str, float]:
    """Check ``encoder_pitch_to_elevation`` against time-matched selfcal rows."""
    cam = read_allstar(allstar_path, "CAM1" if cam_sfx == "1" else "CAM2")
    d = read_selfcal(selfcal_path)
    ts, el_s = d["T_Eval"], d[f"Pitch{cam_sfx}"]
    yaw_s = d[f"Yaw{cam_sfx}"]
    order = np.argsort(cam.t)
    tt = cam.t[order]
    d_el, d_yaw, n = [], [], 0
    for i in range(len(ts)):
        if not (np.isfinite(ts[i]) and np.isfinite(el_s[i])):
            continue
        j = bisect.bisect_left(tt, ts[i])
        for k in (j - 1, j):
            if 0 <= k < len(tt) and abs(tt[k] - ts[i]) <= max_dt:
                m = order[k]
                d_el.append(abs(cam.elevation[m] - el_s[i]))
                if np.isfinite(yaw_s[i]):
                    d_yaw.append(abs(wrap180(cam.encoder_yaw[m] - yaw_s[i])))
                n += 1
                break
        if n >= limit:
            break
    if not d_el:
        return {"n": 0.0}
    return {
        "n": float(len(d_el)),
        "elev_med": float(np.median(d_el)),
        "elev_p90": float(np.percentile(d_el, 90)),
        "yaw_med": float(np.median(d_yaw)) if d_yaw else math.nan,
    }


def utc(ts: float) -> str:
    return datetime.datetime.utcfromtimestamp(float(ts)).strftime("%Y-%m-%d %H:%M:%S")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Validate the VSL geometry replica")
    ap.add_argument(
        "--root",
        default=os.environ.get(
            "VSL_DATA_ROOT",
            os.path.expanduser("~/savasan_iha_yildizlar_data/VSL/logs"),
        ),
    )
    ap.add_argument("--session", default="21")
    args = ap.parse_args()

    base = f"{args.root}/{args.session}"
    sc = f"{base}/selfcal_log.txt"

    print("=" * 68)
    print(f"session {args.session}")
    print("=" * 68)
    cfg = read_selfcal_config(sc)
    if cfg:
        print("selfcal CONFIG:")
        for k in ("schema", "axes", "sigma_ang_deg", "gate_sigma", "apply",
                  "boresight", "cam1_enu", "cam2_enu", "update_hz"):
            if k in cfg:
                print(f"   {k:14s} = {cfg[k]}")

    print("\n-- replica vs VSL logged H/P (deg) --")
    st = validate_against_selfcal(sc)
    for cam in ("CAM1", "CAM2"):
        if f"{cam}_n" in st:
            print(f"   {cam}: n={st[f'{cam}_n']:.0f}  "
                  f"|dH| med={st[f'{cam}_dh_med']:.2e} max={st[f'{cam}_dh_max']:.2e}  "
                  f"|dP| med={st[f'{cam}_dp_med']:.2e} max={st[f'{cam}_dp_max']:.2e}")

    print("\n-- encoder convention vs selfcal (deg) --")
    for sfx in ("1", "2"):
        r = validate_encoder_convention(f"{base}/allstar_cam{sfx}_log.txt", sc, sfx)
        if r.get("n"):
            print(f"   CAM{sfx}: n={r['n']:.0f} elev|d| med={r['elev_med']:.3f} "
                  f"p90={r['elev_p90']:.3f}  yaw|d| med={r['yaw_med']:.3f}")
        else:
            print(f"   CAM{sfx}: no time-matched rows")
