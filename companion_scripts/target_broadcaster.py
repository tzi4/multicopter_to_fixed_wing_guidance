import time
import json
import redis
import random
import math
from pymavlink import mavutil
from config import SWARM_CONFIG
from math_utils import get_target_location

def broadcast_noisy_target():
    """
    Receives telemetry from target aircraft (6. SITL) with MAVLink. Adds a random offset equal to
    the margin in the config on top of the GPS coordinate. It broadcasts this faulty location via 1
    Hz (or rate in config) and Redis.
    """
    conn_str = SWARM_CONFIG['TARGET_PLANE']['connection_string']
    print(f"Connecting to Target Aircraft MAVLink: {conn_str}")
    try:
        conn = mavutil.mavlink_connection(conn_str)
        conn.wait_heartbeat(timeout=15)
        print(f"Target Aircraft Contact Established (sys:{conn.target_system} comp:{conn.target_component})")
    except Exception as e:
        print(f"Connection error: {e}")
        return

    try:
        redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"])
    except Exception as e:
        print(f"Redis connection error: {e}")
        return

    update_rate = SWARM_CONFIG.get("TARGET_UPDATE_RATE", 1.0)
    error_margin = SWARM_CONFIG.get("TARGET_GPS_ERROR_MARGIN", 30.0)

    print(f"Incorrect Target Broadcast Started. (Frequency: {update_rate} Hz, Margin: {error_margin}m)")

    while True:
        # Read current telemetry of target aircraft (from SITL)
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2.0)
        if msg:
            real_lat = msg.lat / 1e7
            real_lon = msg.lon / 1e7
            real_alt = msg.relative_alt / 1000.0

            # Add random deviation (Circular random distribution) Let's choose a random direction (0-360 degrees)
            # and a random distance (0-error_margin)
            random_angle = random.uniform(0, 360)
            random_distance = random.uniform(0, error_margin)

            # math_utils.get_target_location function gets dx(right) and dy(forward) offset. If we accept the
            # heading angle 0 (with North reference), we can find dx and dy with Trigonometry.
            dx = random_distance * math.sin(math.radians(random_angle)) # East/West offset
            dy = random_distance * math.cos(math.radians(random_angle)) # North/South offset

            # Find the new (incorrect) coordinate
            noisy_lat, noisy_lon = get_target_location(real_lat, real_lon, dx, dy, 0.0)

            payload = {
                "status": "MOVING_TARGET",
                "lat": noisy_lat,
                "lon": noisy_lon,
                "alt": real_alt,
                "error_margin_applied": random_distance
            }

            redis_client.publish("TARGET_DETECTION", json.dumps(payload))
            print(f"Target Aircraft: Actual({real_lat:.5f}, {real_lon:.5f}) -> Released({noisy_lat:.5f}, {noisy_lon:.5f}) | Deviation: {random_distance:.1f}m")

        time.sleep(1.0 / update_rate)

if __name__ == "__main__":
    broadcast_noisy_target()
