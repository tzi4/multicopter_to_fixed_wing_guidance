#!/usr/bin/env python3
"""Raw target telemetry for balloon testing -> scalar range only.

This layer compares the target's GLOBAL_POSITION_INT position with the hunter's LOCAL_POSITION_NED
position in the same NED frame. The control layer is not given target position, speed or direction;
The only control measurement popped out is the number ``||target_ned - pursuer_ned||`` returned by
``range()``. LOS direction comes from camera.

On the way to the competition, the estimator range source will be used instead. The purpose of this
module is to keep the Microhard/second telemetry link balloon test independent of the estimator and
easily auditable.
"""

from __future__ import annotations

import math
import time

import numpy as np


def range_norm(own_pos_ned, target_pos_ned):
    """Scalar 3D ranging from two NED positions; ``None`` on invalid input."""
    if own_pos_ned is None or target_pos_ned is None:
        return None
    try:
        own_value = np.asarray(own_pos_ned, dtype=float).reshape(3)
        target_value = np.asarray(target_pos_ned, dtype=float).reshape(3)
    except (TypeError, ValueError):
        return None
    if not (np.all(np.isfinite(own_value)) and np.all(np.isfinite(target_value))):
        return None
    result_value = float(np.linalg.norm(target_value - own_value))
    return result_value if math.isfinite(result_value) else None


class RawTelemetryRange:
    """GLOBAL_POSITION_INT produces only scalar range from target link.

    ``home`` is the pursuer's HOME_POSITION tuple ``(lat_deg, lon_deg, alt_m)``.
    ``relative_alt=True`` avoids GPS AMSL offsets when both vehicles establish home at the same
    site. For homes at different elevations, select ``relative_alt=False`` to use the target's AMSL
    ``alt`` field.
    """

    label_item = "telemetry"

    def __init__(
        self,
        target_conn_str,
        home,
        hz=10.0,
        stale_s=1.0,
        relative_alt=True,
    ):
        from pymavlink import mavutil
        try:
            from guidance_allstar import mavlink_utils
        except ImportError:  # Pi'nin self-contained/flat modul duzeni
            import mavlink_utils

        self.stale_s = float(stale_s)
        home_lat, home_lon, home_alt = (float(x) for x in home)
        conn = mavutil.mavlink_connection(target_conn_str, source_system=251)
        heartbeat = conn.wait_heartbeat(timeout=30)
        if heartbeat is None:
            raise TimeoutError(
                f"target heartbeat not received: {target_conn_str}"
            )
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            int(1e6 / max(1.0, float(hz))),
            0, 0, 0, 0, 0,
        )
        self._reader_value = mavlink_utils.MavStateReader(
            conn,
            "GLOBAL_POSITION_INT",
            lambda msg: mavlink_utils.parse_global_int(
                msg,
                home_lat,
                home_lon,
                home_alt,
                use_relative_alt=bool(relative_alt),
            ),
        )
        self._reader_value.start()

    def range_value(self, own_pos_ned):
        target_value, _vel, _stamp, wall = self._reader_value.get_with_times()
        if target_value is None or wall <= 0.0:
            return None
        if time.monotonic() - wall > self.stale_s:
            return None
        return range_norm(own_pos_ned, target_value)

    def ref_target_state(self):
        # Target telemetry should not leak into the control path and log columns.
        return {
            "pos": None,
            "vel": None,
            "acc": None,
            "turn_dps": None,
            "est_pos": None,
        }
