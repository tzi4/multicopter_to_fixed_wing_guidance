#!/usr/bin/env python3
"""SALT TRACK: MPC or LOS/PN command generation + virtual gimbal bench test.

Written for SAFE TESTING ON A REAL DRONE (2026-08-09): * NO commands are SENT to MAVLink -- there is
NO send path in this file (SpeedCommander is not imported, set_position_target is no / arm / mode call).
The only text MAVLink is the request SET_MESSAGE_INTERVAL (ATTITUDE + LOCAL_POSITION_NED flow rate;
cannot move the vehicle). * The selected controller is IMPORTED, not copied -- the EXACT law that
will run in flight is tested. If LOS is selected, MPC/optimizer is not installed. * The engagement
gate is NOT WAITED: As long as bbox arrives, the law runs in every cycle and the command it will
produce is pressed on the SCREEN.

WHAT'S ON THE SCREEN (all just CALCULATED values): raw ex/ey : tracker_bbox (AI raw bbox) with
pinhole from center — virtual gimbal WITHOUT CORRECTION angle error virtual ex/ey: tracker_bbox_stab
— error produced by the virtual gimbal (currently ROLL de-rotation only; pitch on the physical tilt
gimbal) v_ned : speed command [m/s, NED] + magnitude yaw : yaw speed to produce law [dps] r/status :
controller internal range/phase status

CONDITION: the detection chain must be running (raspberry_cam_ai.py + hardware/bbox_to_redis.py) and
the camera must NOT be in another process (gimbal_bench_tracking.py must be closed).

USAGE:
  python3 tools/monitor_mpc_commands.py                       # default 14550
  python3 tools/monitor_mpc_commands.py --law los --range-m-value 20
  python3 tools/monitor_mpc_commands.py --no-mavlink          # without attitude/speed
Exit: Ctrl-C.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
_GA_CANDIDATES = (_ROOT / 'hardware' / 'guidance_allstar',
                _ROOT / 'guidance_allstar')
_GA = next((p for p in _GA_CANDIDATES if p.is_dir()), _GA_CANDIDATES[0])
for _p in (str(_GA), str(_ROOT / 'hardware')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import redis                                                     # noqa: E402
from pymavlink import mavutil                                    # noqa: E402
import mavlink_utils                                             # noqa: E402
from visual_base import (BboxReader, Measurement,                 # noqa: E402
                             _FRAMING_FX, _FRAMING_W, _FRAMING_H)
from terminal_los_guidance import TerminalLosController              # noqa: E402


class ReadOnlyTelemetry:
    """Hunter attitude/position/speed READER. There is KNOWINGLY no method to send commands."""

    def __init__(self, connection_value):
        self.conn = mavutil.mavlink_connection(connection_value, source_system=249)
        print(f"[watch] MAVLink heartbeat expected: {connection_value}...")
        self.conn.wait_heartbeat(timeout=30)
        print(f"[watch] heartbeat received (sys {self.conn.target_system})")
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self.reader = mavlink_utils.MavStateReader(
            self.conn,
            ["LOCAL_POSITION_NED", "ATTITUDE", "VIBRATION", "HEARTBEAT"],
            mavlink_utils.parse_local_ned)
        self.reader.start()


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--connection-value', default='udpin:127.0.0.1:14550',
                   help='pursuer MAVLink (mavproxy output; 14551 / 14552 full '
                        'possible, default 14550 )')
    p.add_argument('--no-mavlink', action='store_true',
                   help='Run without FC connection (attitude/speed None; live '
                        'still runs, assuming yaw=0)')
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--law', choices=['mpc', 'los'], default='mpc',
                   help='merely the law to be followed; old behavior of the file mpc')
    p.add_argument('--n-pn', type=float, default=4.0,
                   help='--law coefficient for los PN')
    p.add_argument('--strike-acceleration', type=float, default=4.0,
                   help='--law forward acceleration for los [m/s2]')
    p.add_argument('--report-hz', type=float, default=5.0,
                   help='write speed (independent of cycle rate)')
    p.add_argument('--range-m-value', type=float, default=None,
                   help='FIXED range [m]; LOS issued in field observation '
                        'onerilir')
    p.add_argument('--mount', type=float, default=None,
                   help='camera mounting angle [deg]; if not given $YILDIZ_MOUNT')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw command generation (speed channels only)')
    p.add_argument('--stale-s', type=float, default=0.7,
                   help='bbox staleness threshold [s] (same as skeleton)')
    p.add_argument('--duration-value', type=float, default=None)
    p.add_argument('--diagnostic-log', default=None,
                   help='MPC diagnostic CSV (default logs/mpc_track_diagnostic_*.csv)')
    a = p.parse_args()

    config_value = None
    if a.law == 'mpc':
        from mpc_guidance import MpcConfig, MpcController
        config_value = MpcConfig()
        if a.mount is not None:
            config_value.mount_pitch_deg = a.mount
        if a.aim is not None:
            config_value.aim_deg = a.aim
        if a.no_yaw:
            config_value.yaw_command_provide = False

    from datetime import datetime
    stamp_value = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_directory = _ROOT / 'logs'
    log_directory.mkdir(exist_ok=True)
    diagnostic = a.diagnostic_log or str(log_directory / f"mpc_track_diagnostic_{stamp_value}.csv")

    print("=" * 72)
    print("[watch] *** MONITOR ONLY MODE: NO COMMAND SENT TO MAVLink, ARM/MODE")
    print("[watch] *** CANNOT BE CHANGED. Only calculated values ​​are printed.")
    print("=" * 72)
    if a.law == 'mpc':
        print(f"[watch] mount={config_value.mount_pitch_deg:+.2f} "
              f"aim={config_value.aim_deg:+.2f} "
              f"yaw_command={'ENABLED' if config_value.yaw_command_provide else 'DISABLED'}")
        print(f"[watch] range source: "
              f"{'FIXED %.1f m' % a.range_m_value if a.range_m_value is not None else 'NONE (ic varsayim %.0f m)' % config_value.range_if_absent_m}")
        print(f"[watch] MPC diagnostic log: {diagnostic}")
    else:
        print(f"[watch] LOS / PN : N= {a.n_pn:g} strike_acceleration = {a.strike_acceleration:g} m/s2 "
              f"aim={(a.aim or 0.0):+.1f} "
              f"yaw_command={'DISABLED' if a.no_yaw else 'ENABLED'}")
        print(f"[watch] range source: "
              f"{'FIXED %.1f m' % a.range_m_value if a.range_m_value is not None else 'NONE (initial ic value_value 40 m)'}")

    r = redis.Redis(host='localhost', port=6379, db=0)
    r.ping()
    bbox = BboxReader(r)
    bbox.start()

    tele = None
    if not a.no_mavlink:
        tele = ReadOnlyTelemetry(a.connection_value)

    k = (MpcController(config_value, diagnostic_log=diagnostic) if a.law == 'mpc'
         else TerminalLosController(n_pn=a.n_pn,
                                   strike_acceleration_mps2=a.strike_acceleration,
                                   aim_deg=(a.aim or 0.0),
                                   yaw_command_provide=not a.no_yaw))
    k.seed_value2(None)

    loop_dt = 1.0 / a.loop_hz
    report_dt = 1.0 / max(a.report_hz, 0.1)
    previous_t = None
    last_report = 0.0
    last_cmd = None
    start_value = time.monotonic()
    n_run = 0
    try:
        while a.duration_value is None or time.monotonic() - start_value < a.duration_value:
            t0 = time.monotonic()
            now_value = t0
            dt = (loop_dt if previous_t is None
                  else min(max(now_value - previous_t, 0.5 * loop_dt), 0.5))
            previous_t = now_value

            stab, bbox_age, coverage_value = bbox.last_value()
            raw_value, raw_age = bbox.raw_value()
            fresh_value = stab is not None and bbox_age <= a.stale_s

            pos = vel = att = vibe = None
            if tele is not None:
                pos, vel = tele.reader.get()
                att = tele.reader.get_attitude()
                vibe = tele.reader.get_vibration()

            raw_ex = raw_ey = None
            if raw_value is not None and raw_age <= a.stale_s:
                cxp = float(raw_value[0]) + float(raw_value[2]) / 2.0
                cyp = float(raw_value[1]) + float(raw_value[3]) / 2.0
                raw_ex = math.degrees(math.atan((cxp - _FRAMING_W / 2.0)
                                                / _FRAMING_FX))
                raw_ey = math.degrees(math.atan((cyp - _FRAMING_H / 2.0)
                                                / _FRAMING_FX))

            if fresh_value:
                o = Measurement(
                    t=now_value, dt=dt,
                    ex_deg=float(stab[4]), ey_deg=float(stab[5]),
                    bbox_w=float(stab[2]), bbox_h=float(stab[3]),
                    area_root=math.sqrt(float(stab[2]) * float(stab[3])),
                    coverage_pct=coverage_value,
                    bbox_age_s=bbox_age,
                    t_capture=(float(stab[6]) if len(stab) > 6 else None),
                    tilt_deg=(float(stab[7]) if len(stab) > 7
                              and stab[7] is not None else None),
                    range_m_value=a.range_m_value,
                    pos_ned=(np.asarray(pos, float) if pos is not None
                             else None),
                    vel_ned=(np.asarray(vel, float) if vel is not None
                             else None),
                    yaw_rad=att[2] if att is not None else None,
                    roll_rad=att[0] if att is not None else None,
                    pitch_rad=att[1] if att is not None else None,
                    px_virtual_x=float(stab[0]), px_virtual_y=float(stab[1]),
                    px_raw_cx=None, px_raw_cy=None,
                    vibe_max=None if vibe is None else float(max(vibe)),
                )
                cmd = k.command_value(o)
                n_run += 1
                last_cmd = cmd
                if getattr(cmd, 'event_value', ''):
                    print(f"[watch] {a.law.upper()} EVENT: {cmd.event_value} "
                          f"{getattr(cmd, 'event_detail', '')}", flush=True)
                if getattr(cmd, 'release_value', False):
                    print(f"[watch] {a.law.upper()} declared MISS/RELEASE "
                          f"(reason={cmd.release_reason!r}) -- replay in trace "
                          f"seeded and continued", flush=True)
                    k.seed_value2(None)

            if now_value - last_report >= report_dt:
                last_report = now_value
                if not fresh_value:
                    cause = ('bbox NONE' if stab is None
                             else f'bbox stale ( {bbox_age:.1f} s)')
                    print(f"[watch] waiting for detection: {cause}", flush=True)
                else:
                    v = np.asarray(last_cmd.vel_ned, float)
                    speed_value = float(np.linalg.norm(v))
                    yaw_s = ('---' if last_cmd.yaw_rate_dps is None
                             else f"{last_cmd.yaw_rate_dps:+6.2f}")
                    raw_s = ('  no / no ' if raw_ex is None
                             else f"{raw_ex:+6.2f}/{raw_ey:+6.2f}")
                    if a.law == 'mpc':
                        internal_range, state_value = k.internal_range, k.state_value
                    else:
                        internal_range = float(k.diagnostic.get('r', a.range_m_value or 40.0))
                        state_value = str(k.diagnostic.get('phase_value', k.phase_value))
                    print(f"raw {raw_s} | virtual {float(stab[4]):+6.2f} /"
                          f"{float(stab[5]):+6.2f} deg | "
                          f"v N{v[0]:+5.2f} E{v[1]:+5.2f} D{v[2]:+5.2f} "
                          f"|{speed_value:5.2f}| m/s | yaw {yaw_s} dps | "
                          f"r {internal_range:5.1f} m | {state_value}", flush=True)

            remaining_value = loop_dt - (time.monotonic() - t0)
            if remaining_value > 0:
                time.sleep(remaining_value)
    except KeyboardInterrupt:
        pass
    finally:
        ek = f", diagnostic log: {diagnostic}" if a.law == 'mpc' else ''
        print(f"\n [watch] finished: {n_run} {a.law.upper()} cos {ek}")


if __name__ == '__main__':
    main()
