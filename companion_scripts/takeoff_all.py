import time
from pymavlink import mavutil
from config import SWARM_CONFIG

def arm_and_takeoff_all(target_alt=15):
    """
    It connects to all drones sequentially, puts them in GUIDED mode, arms them and sends the
    Takeoff command.
    """
    conns = []
    # Connect to all of them first
    for drone_id, cfg in SWARM_CONFIG["DRONES"].items():
        print(f"UAV {drone_id} connecting to: {cfg['connection_string']}")
        conn = mavutil.mavlink_connection(cfg['connection_string'])
        conn.wait_heartbeat()
        conns.append((drone_id, conn))
        print(f"UAV {drone_id} Heartbeat received.")

    print("\nAll UAVs are put into GUIDED mode and ARMed...")
    for drone_id, conn in conns:
        # Switch to GUIDED Mode (Custom Mode 4 for ArduCopter)
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)

        # ARM the engines
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(5)

    print("\nAll UAVs are given the TakeOFF command...")
    for drone_id, conn in conns:
        # takeoff command
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_alt)

        print(f"UAV {drone_id} taking off ({target_alt} meters).")
        time.sleep(5)

    print("\nTakeoff commands completed! Drones are expected to take off...")

if __name__ == "__main__":
    arm_and_takeoff_all(15) # 15 takeoff to meter
