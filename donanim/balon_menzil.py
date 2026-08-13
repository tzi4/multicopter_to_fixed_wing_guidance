#!/usr/bin/env python3
"""Balon testi icin ham hedef telemetrisi -> yalniz skaler menzil.

Bu katman hedefin GLOBAL_POSITION_INT konumunu avcinin LOCAL_POSITION_NED
konumuyla ayni NED cercevesinde karsilastirir. Kontrol katmanina hedef konumu,
hizi veya yonu verilmez; disari acilan tek kontrol olcumu ``menzil()`` ile
donen ``||hedef_ned - avci_ned||`` sayisidir. LOS yonu kameradan gelir.

Yarisma yolunda bunun yerine estimator menzil kaynagi kullanilacaktir. Bu
modulun amaci Microhard/ikinci telemetri linkli balon testini estimator'dan
bagimsiz ve kolay denetlenebilir tutmaktir.
"""

from __future__ import annotations

import math
import time

import numpy as np


def menzil_normu(kendi_pos_ned, hedef_pos_ned):
    """Iki NED konumundan skaler 3B menzil; gecersiz girdide ``None``."""
    if kendi_pos_ned is None or hedef_pos_ned is None:
        return None
    try:
        kendi = np.asarray(kendi_pos_ned, dtype=float).reshape(3)
        hedef = np.asarray(hedef_pos_ned, dtype=float).reshape(3)
    except (TypeError, ValueError):
        return None
    if not (np.all(np.isfinite(kendi)) and np.all(np.isfinite(hedef))):
        return None
    sonuc = float(np.linalg.norm(hedef - kendi))
    return sonuc if math.isfinite(sonuc) else None


class HamTelemetriMenzil:
    """GLOBAL_POSITION_INT hedef linkinden yalniz skaler menzil uretir.

    ``home`` avcinin ``(lat_deg, lon_deg, alt_m)`` HOME_POSITION degeridir.
    ``relative_alt=True`` iki arac ayni sahada kendi home'larini aldiginda
    GPS AMSL ofsetlerinden etkilenmez. Farkli irtifalardaki home'lar icin
    ``relative_alt=False`` secilerek hedefin AMSL ``alt`` alani kullanilir.
    """

    ad = "telemetri"

    def __init__(
        self,
        target_conn_str,
        home,
        hz=10.0,
        bayat_s=1.0,
        relative_alt=True,
    ):
        from pymavlink import mavutil
        try:
            from guidance_allstar import mavlink_utils
        except ImportError:  # Pi'nin self-contained/flat modul duzeni
            import mavlink_utils

        self.bayat_s = float(bayat_s)
        home_lat, home_lon, home_alt = (float(x) for x in home)
        conn = mavutil.mavlink_connection(target_conn_str, source_system=251)
        heartbeat = conn.wait_heartbeat(timeout=30)
        if heartbeat is None:
            raise TimeoutError(
                f"hedef heartbeat gelmedi: {target_conn_str}"
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
        self._okuyucu = mavlink_utils.MavStateReader(
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
        self._okuyucu.start()

    def menzil(self, kendi_pos_ned):
        hedef, _vel, _stamp, wall = self._okuyucu.get_with_times()
        if hedef is None or wall <= 0.0:
            return None
        if time.monotonic() - wall > self.bayat_s:
            return None
        return menzil_normu(kendi_pos_ned, hedef)

    def ref_hedef_durum(self):
        # Hedef telemetrisi kontrol yoluna ve log kolonlarina sizmasin.
        return {
            "pos": None,
            "vel": None,
            "acc": None,
            "donus_dps": None,
            "est_pos": None,
        }
