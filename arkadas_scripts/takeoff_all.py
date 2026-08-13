import time
from pymavlink import mavutil
from config import SWARM_CONFIG

def arm_and_takeoff_all(target_alt=15):
    """
    Tüm dronelara sırayla bağlanır, GUIDED moduna alır, Arm eder ve Kalkış (Takeoff) komutu gönderir.
    """
    conns = []
    # Önce hepsine bağlan
    for drone_id, cfg in SWARM_CONFIG["DRONES"].items():
        print(f"İHA {drone_id} bağlanıyor: {cfg['connection_string']}")
        conn = mavutil.mavlink_connection(cfg['connection_string'])
        conn.wait_heartbeat()
        conns.append((drone_id, conn))
        print(f"İHA {drone_id} Heartbeat alındı.")

    print("\nTüm İHA'lar GUIDED moda alınıyor ve ARM ediliyor...")
    for drone_id, conn in conns:
        # GUIDED Moduna geçir (ArduCopter için Custom Mode 4)
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4)
        
        # Motorları ARM et
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(5)

    print("\nTüm İHA'lara Kalkış (TAKEOFF) komutu veriliyor...")
    for drone_id, conn in conns:
        # Kalkış komutu
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_alt)
            
        print(f"İHA {drone_id} kalkış yapıyor ({target_alt} metre).")
        time.sleep(5)

    print("\nKalkış komutları tamamlandı! Droneların havalanması bekleniyor...")

if __name__ == "__main__":
    arm_and_takeoff_all(15) # 15 metreye kalkış
