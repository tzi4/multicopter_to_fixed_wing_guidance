#!/usr/bin/env python3
"""Does the TASK INSTALLED ON THE VEHICLE match the REQUESTED PLAN? (scenario.sh ten door)

WHY THERE IS (2026-08-09, TWO TRIES FIRED SILENTLY): In tools/scenario.sh, the PLAN variable goes
onto the stack as YILDIZ_TARGET_PLAN ONLY in the "RESTART == 1" branch. When I run with
RESTART=0 (our standard rule for protecting the GUI) PLAN DOES NOTHING: the target keeps
flying the loaded task while the stack is being opened (default target_ellipse.plan). Moreover, ">>>
target aircraft: AUTO, straight course" was written on the screen -- that label comes from PLAN_NAME,
NOT THE ONE LOADED ON THE VEHICLE. Result: two attempts as "straight regression" (tyawacc_straight, kpn_straight)
actually flew ELLIPSE and one was reported as "straight regression PASSED".

HOW IT COMPARISES: GEOMETRY, not NUMBER of waypoints. The number is unreliable as home items can be
added or removed during installation; Instead, the location coverage (north-south and east-west
span) and presence of DO_JUMP are compared. If all three hold, it is the same route.

EXIT CODE: 0 = compliant, 1 = INCOMPATIBLE, 2 = decision could not be made (tool/plan could not be
read). scenario.sh STOPS at 1, warns at 2 and continues.
"""
import argparse, json, math, sys, time

R_WORLD = 6371000.0


def coverage_item(points_value):
    """(north-south [m], east-west [m]) span."""
    la = [p[0] for p in points_value]
    lo = [p[1] for p in points_value]
    kg = (max(la) - min(la)) * math.pi / 180.0 * R_WORLD
    db = ((max(lo) - min(lo)) * math.pi / 180.0 * R_WORLD
          * math.cos(math.radians(la[0])))
    return kg, db


def from_plan(path_value):
    it = json.load(open(path_value))['mission']['items']
    n = [(i['params'][4], i['params'][5]) for i in it
         if i.get('params') and i['params'][4] not in (None, 0)
         and i['params'][5] not in (None, 0)]
    if not n:
        return None
    kg, db = coverage_item(n)
    return dict(kg=kg, db=db, jump=any(i.get('command') == 177 for i in it))


def from_vehicle(connection_value, time_timeout=25.0):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(connection_value, source_system=248)
    if m.wait_heartbeat(timeout=time_timeout) is None:
        return None
    m.mav.mission_request_list_send(m.target_system, m.target_component)
    t0 = time.time(); count_value = None
    while time.time() - t0 < 10:
        x = m.recv_match(type='MISSION_COUNT', blocking=True, timeout=2)
        if x:
            count_value = x.count; break
    if not count_value:
        return None
    n, jump = [], False
    for seq in range(count_value):
        m.mav.mission_request_int_send(m.target_system, m.target_component, seq)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            x = m.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=1)
            if x and x.seq == seq:
                if x.command == 177:
                    jump = True
                if x.x and x.y:
                    n.append((x.x / 1e7, x.y / 1e7))
                break
    if not n:
        return None
    kg, db = coverage_item(n)
    return dict(kg=kg, db=db, jump=jump)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', required=True)
    p.add_argument('--connection-value', default='udpin:127.0.0.1:14601')
    p.add_argument('--tolerance', type=float, default=0.15,
                   help='relative tolerance for scope (default %%15)')
    a = p.parse_args()

    d = from_plan(a.plan)
    if d is None:
        print(f"plan_alignment: PLAN okunamadi ({a.plan})", file=sys.stderr)
        return 2
    v = from_vehicle(a.connection_value)
    if v is None:
        print("plan_alignment: failed to read mission from vehicle (no heartbeat/task)",
              file=sys.stderr)
        return 2

    def near_value(x, y):
        return abs(x - y) <= a.tolerance * max(abs(x), abs(y), 1.0)

    alignment_value = (near_value(d['kg'], v['kg']) and near_value(d['db'], v['db'])
            and d['jump'] == v['jump'])
    print(f"plan_alignment: FILE scope K-G {d['kg']:.0f} m / D-B {d['db']:.0f} m "
          f"DO_JUMP {d['jump']}")
    print(f"plan_alignment: VEHICLE scope K-G {v['kg']:.0f} m / D-B {v['db']:.0f} m "
          f"DO_JUMP {v['jump']}")
    if alignment_value:
        print("plan_alignment: UYUMLU")
        return 0
    print("plan_alignment: *** INCOMPATIBLE -- THE TASK INSTALLED ON THE VEHICLE IS NOT THE REQUESTED PLAN ***",
          file=sys.stderr)
    print("plan_alignment: reason: when RESTART=0 the PLAN is not loaded into the vehicle; "
          "restart the stack with YILDIZ_TARGET_PLAN=<plan>.",
          file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
