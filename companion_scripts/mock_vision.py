import time
import json
import redis
from config import SWARM_CONFIG

def simulate_detection():
    """
    Script that generates fake Redis messages to test the network until the camera and YOLO are
    integrated.
    """
    r = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"])

    print("SIMULATION STARTED: UAVs fly in formation for 10 seconds...")
    for i in range(10, 0, -1):
        print(f"{i} seconds left...")
        time.sleep(1)

    # Scenario: UAV 3 (Sub Tracker) saw the target on its own camera
    finder_drone_id = 3

    print(f"SIMULATION: UAV {finder_drone_id} spotted the target! TARGET_FOUND signal is being sent...")
    msg = {
        "status": "TARGET_FOUND",
        "drone_id": finder_drone_id
    }
    r.publish("TARGET_DETECTION", json.dumps(msg))

    print("SIMULATION: Target tracking has started. Bounding Box coordinates are printed on Redis...")

    # Let's simulate it as if the target is standing at the center of the camera (320, 240).
    center_x = 320
    center_y = 240

    for i in range(200): # Tracking will take about 20 seconds
        # Let the target move very slightly left and right (to test PID)
        bbox_x = center_x + (i % 30) - 15
        bbox_y = center_y + (i % 20) - 10

        msg = {
            "status": "TARGET_TRACKING",
            "drone_id": finder_drone_id,
            "bbox_center_x": bbox_x,
            "bbox_center_y": bbox_y
        }
        r.publish("TARGET_DETECTION", json.dumps(msg))

        if i % 10 == 0:
            print(f"Released on BBOX -> X: {bbox_x}, Y: {bbox_y}")

        time.sleep(0.1) # 10 FPS (10 Hz)

    print("SIMULATION IS OVER.")

if __name__ == "__main__":
    simulate_detection()
