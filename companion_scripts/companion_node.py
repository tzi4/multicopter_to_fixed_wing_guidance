import time
import json
import redis
import argparse
from pymavlink import mavutil
from config import SWARM_CONFIG
from visual_servo import VisualServoController

class CompanionNode:
    """
    It runs on each UAV's own auxiliary computer (Raspberry Pi/Jetson, etc.). It just connects
    directly to the MAVLink port of its UAV (Zero Latency). It reads the camera (YOLO), if the BBox
    finds it, it runs the Visual Servo PID and immediately transfers the speed to the FCU. It also
    takes over the Leadership by sending message TARGET_FOUND to the Ground Station (via Redis).
    """
    def __init__(self, drone_id):
        self.drone_id = drone_id
        cfg = SWARM_CONFIG["DRONES"][self.drone_id]

        self.visual_servo = VisualServoController()

        # Companion Node connects to its own dedicated port (does not conflict with ground_station)
        companion_conn = cfg.get('companion_string', cfg['connection_string'])
        print(f"UAV {self.drone_id} Companion Node connecting to local MAVLink: {companion_conn}")
        self.conn = mavutil.mavlink_connection(companion_conn)
        self.conn.wait_heartbeat(timeout=15)
        print(f"Received UAV {self.drone_id} Heartbeat (sys:{self.conn.target_system} comp:{self.conn.target_component})")

        # Redis connection (to Ground Station or Network Center)
        self.redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"], decode_responses=True)

        self.target_locked = False

    def send_velocity_command(self, vx, vy, vz):
        """Sends calculated PID speeds directly (without going to the network) to local FCU"""
        # MAV_FRAME_BODY_OFFSET_NED (9): Body-relative speeds (forward, right, down) ArduPilot does not
        # support MAV_FRAME_BODY_NED, BODY_OFFSET_NED must be used
        #
        # Bitmask: 0b0000_1_1_1_111_000_111 = 0x0FC7 = 4039 Bit 11 ( yaw_rate )= 1 ignore, Bit 10 ( yaw )= 1
        # ignore, Bit 9 (force)= 1 ignore Bit 8 - 6 (accelerations)= 1 ignore, Bit 5 - 3 (vx,vy,vz)= USE 0,
        # ignore Bit 2 - 0 (pos)= 1
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # = 4039 → Use speeds only
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)

    def run(self):
        print(f"Companion Node (UAV {self.drone_id}) Started. Camera and MAVLink are bugged...")

        # Simulation behavior: retrieve our UAV's bounding boxes from mock_vision through Redis instead of
        # reading a physical camera frame. Hardware would use cv2.VideoCapture here.
        pubsub = self.redis_client.pubsub()
        pubsub.subscribe("TARGET_DETECTION")

        while True:
            # Image Processing / YOLO Estimated Cycle
            message = pubsub.get_message()
            if message and message['type'] == 'message':
                data = json.loads(message['data'])

                # Only if OUR drone's camera spotted the target
                if data["status"] == "TARGET_TRACKING" and data.get("drone_id") == self.drone_id:
                    if not self.target_locked:
                        self.target_locked = True
                        print(f"[UAV {self.drone_id} EDITOR CONTROL] Camera Detected the Target! Leadership is taken...")
                        # Report to the network: "I've seen the target, the ground station should stop giving me waypoints!"
                        msg = {"status": "TARGET_FOUND", "drone_id": self.drone_id}
                        self.redis_client.publish("TARGET_DETECTION", json.dumps(msg))

                    bbox_x = data["bbox_center_x"]
                    bbox_y = data["bbox_center_y"]

                    # DISTRIBUTED CONTROL: Visual Servo Runs on Its Own Companion Computer (In this way, network ping or
                    # ground station load is not reflected)
                    vx, vy, vz = self.visual_servo.calculate_velocity(bbox_x, bbox_y)
                    self.send_velocity_command(vx, vy, vz)

            time.sleep(0.05) # 20 FPS (Waits as long as the speed of the camera FPS)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Companion Computer Node")
    parser.add_argument("--id", type=int, required=True, help="Drone ID (0-4)")
    args = parser.parse_args()

    node = CompanionNode(args.id)
    node.run()
