#!/usr/bin/env python3
"""Suru araclarinin MAVLink uclarini bekler ve SysID'leri dogrular.

Dinlenen portlar BILEREK companion portlaridir (14651+10i ve hedef icin
14602): config.py'nin connection_string portlari (14551...) yer istasyonu /
komut araclarina ait; onlari burada baglayip birakmak yarisa yol acabiliyor.
"""

import argparse
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drones', type=int, default=5, help='kac kopter beklenecek (1..5)')
    parser.add_argument('--timeout', type=float, default=120)
    args = parser.parse_args()

    # (port, beklenen SysID)
    endpoints = [(14651 + 10 * i, i + 1) for i in range(args.drones)]
    endpoints.append((14602, 6))

    masters = [
        (mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=252,
                                    source_component=193), port, sysid)
        for port, sysid in endpoints
    ]
    pending = {port for _, port, _ in masters}
    deadline = time.monotonic() + args.timeout
    while pending and time.monotonic() < deadline:
        for master, port, sysid in masters:
            if port not in pending:
                continue
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is None:
                continue
            if msg.get_srcSystem() != sysid:
                raise SystemExit(f'port {port}: beklenen SysID {sysid}, alinan {msg.get_srcSystem()}')
            pending.remove(port)
            print(f'port {port}: SysID {sysid} heartbeat hazir', flush=True)
        time.sleep(0.05)
    for master, _, _ in masters:
        master.close()
    if pending:
        raise SystemExit('heartbeat zaman asimi: ' + ','.join(map(str, sorted(pending))))


if __name__ == '__main__':
    main()
