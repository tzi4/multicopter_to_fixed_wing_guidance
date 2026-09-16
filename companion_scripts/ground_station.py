import time
import json
import redis
import math
from pymavlink import mavutil
from config import SWARM_CONFIG
from math_utils import get_target_location

class SwarmManager:
    """
    Ground Station Manager: - Listens for the target's faulty GPS location (MOVING_TARGET) from
    Redis. - Sends the current Leader of the Pack to this faulty location (as Guided). - Calculates
    and sends the coordinates of follower UAVs (square formation). - When any UAV sees the target on
    its camera (TARGET_FOUND), it passes the Leadership to it and CUTS Sending Waypoint to it
    (Because Visual Servo will work within it).
    """
    def __init__(self):
        self.drones = {}
        self.state = "FORMATION_FLIGHT"
        self.leader_id = 0

        # Dynamic formation map: drone_id → offset When the leader changes, the old leader takes the position
        # vacated by the new leader
        self.formation_map = dict(SWARM_CONFIG["FORMATION"])

        # Anti-collision timer during leader transition
        self.transition_start = None
        self.transition_duration = 5.0  # seconds — gradual transition time

        # Coordinates of the Moving Target (Swarm Focus Point)
        self.swarm_target_lat = 0.0
        self.swarm_target_lon = 0.0
        self.swarm_target_alt = 0.0

        # Historical data queue for filtering (Moving Average)
        self.target_history = []
        self.filter_window_size = 5 # The last 5 data will be averaged

        # Redis connection
        try:
            self.redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"], decode_responses=True)
            self.pubsub = self.redis_client.pubsub()
            self.pubsub.subscribe("TARGET_DETECTION")
        except Exception as e:
            print(f"[ERROR] Redis connection error: {e}")

        self.connect_to_drones()

    def connect_to_drones(self):
        for drone_id, cfg in SWARM_CONFIG["DRONES"].items():
            print(f"Connecting UAV {drone_id} Telemetry: {cfg['connection_string']}")
            try:
                connection = mavutil.mavlink_connection(cfg['connection_string'])
                connection.wait_heartbeat(timeout=15)
                print(f"Received UAV {drone_id} Heartbeat (sys:{connection.target_system} comp:{connection.target_component})")
                self.drones[drone_id] = {
                    "conn": connection,
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "heading": 0.0,
                    "vx": 0.0,  # North speed (m/s)
                    "vy": 0.0,  # Eastern speed (m/s)
                    "vz": 0.0   # Down speed (m/s)
                }
            except Exception as e:
                print(f"[ERROR] UAV {drone_id} connection failed: {e}")

    def update_telemetry(self):
        for drone_id, drone in self.drones.items():
            conn = drone["conn"]
            # Read ALL messages in the queue, use the latest data (stale data precaution)
            while True:
                msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    drone["lat"] = msg.lat / 1e7
                    drone["lon"] = msg.lon / 1e7
                    drone["alt"] = msg.relative_alt / 1000.0
                    drone["heading"] = msg.hdg / 100.0
                    drone["vx"] = msg.vx / 100.0   # cm/s → m/s (North)
                    drone["vy"] = msg.vy / 100.0   # cm/s → m/s (East)
                    drone["vz"] = msg.vz / 100.0   # cm/s → m/s (Down)
                elif msg.get_type() == 'VFR_HUD':
                    drone["heading"] = msg.heading

    def send_target_location(self, drone_id, lat, lon, alt, yaw_rad=None, vel_ned=None):
        """
        Sends target GPS, optional Yaw and velocity vector via MAVLink to a UAV.

        vel_ned: (vx, vy, vz) with tube - speed (m/s) in frame NED. The leader's velocity vector is
        given as a feed-forward, so the followers match the leader's velocity (e.g. 17 m/s).
        """
        conn = self.drones[drone_id]["conn"]

        vx, vy, vz = vel_ned if vel_ned else (0.0, 0.0, 0.0)
        yaw_val = yaw_rad if yaw_rad is not None else 0.0

        # Bitmask: 1 =ignore, 0 =use Bit 0 - 2 : lat,lon,alt | Bit 3 - 5 : vx,vy,vz | Bit 6 - 8 : ax , ay , az
        # Bit 9 : force | Bit 10 : yaw | Bit 11 : yaw_rate
        if vel_ned and yaw_rad is not None:
            # Position + Speed ​​+ Yaw
            type_mask = 0b0000101111000000  # 3008
        elif yaw_rad is not None:
            # Position + Yaw (no speed)
            type_mask = 0b0000101111111000  # 3064
        elif vel_ned:
            # Position + Speed ​​(no yaw)
            type_mask = 0b0000111111000000  # 4032
        else:
            # Position Only
            type_mask = 0b0000111111111000  # 4088

        conn.mav.set_position_target_global_int_send(
            0,
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            type_mask,
            int(lat * 1e7), int(lon * 1e7), alt,
            vx, vy, vz,
            0, 0, 0,
            yaw_val, 0)

    def _swap_leader(self, new_leader_id):
        """
        It changes leadership. The old leader remains in the REVERSE position relative to the new
        leader (minimum movement). The drone in the reverse position moves to the vacant slot.

        Example: Drone 3 (dz:-5, bottom) becomes leader: - Drone 0 (old leader) is already 5m above
        drone 3 → takes slot dz:+5 (stays in place) - Drone 1 (was in the dz:+5 slot) → moves to the
        vacant dz:-5 slot
        """
        old_leader_id = self.leader_id

        if new_leader_id in self.formation_map:
            # New leader's old formation offset
            vacated_slot = self.formation_map.pop(new_leader_id)

            # Old leader in REVERSE position relative to new leader → minimum movement
            inverse_slot = {
                "dx": -vacated_slot["dx"],
                "dy": -vacated_slot["dy"],
                "dz": -vacated_slot["dz"]
            }

            # Does this reverse location belong to another drone?
            displaced_drone_id = None
            for did, slot in self.formation_map.items():
                if (slot["dx"] == inverse_slot["dx"] and
                    slot["dy"] == inverse_slot["dy"] and
                    slot["dz"] == inverse_slot["dz"]):
                    displaced_drone_id = did
                    break

            if displaced_drone_id is not None:
                # Move that drone to the vacant slot
                self.formation_map[displaced_drone_id] = vacated_slot
                print(f"[FORMATION] UAV {displaced_drone_id} → position dx:{vacated_slot['dx']}, dy:{vacated_slot['dy']}, dz:{vacated_slot['dz']}")

            # Former leader of the village in the opposite position (remains in place)
            self.formation_map[old_leader_id] = inverse_slot
            print(f"[FORMATION] UAV {old_leader_id} → position dx:{inverse_slot['dx']}, dy:{inverse_slot['dy']}, dz:{inverse_slot['dz']}")

        self.leader_id = new_leader_id
        self.transition_start = time.time()  # Start gradual migration
        print(f"[LEADER CHANGE] UAV {old_leader_id} → UAV {new_leader_id} (transition started)")

    def _get_yaw_offset_for_position(self, drone_id):
        """
        Determines the yaw offset (in degrees) relative to the pattern position. - Right side drone
        (dx > 0): +20° (facing right) - Left side drone (dx < 0): -20° (facing left) -
        Top/Bottom/Center (dx == 0): 0° (facing left) let him look)
        """
        offsets = self.formation_map.get(drone_id, {"dx": 0, "dy": 0, "dz": 0})
        dx = offsets.get("dx", 0)

        if dx > 0:      # Drone on the right
            return 20.0
        elif dx < 0:    # Drone on the left
            return -20.0
        else:           # Top, bottom or center → facing forward
            return 0.0

    def maintain_formation(self):
        """It calculates and sends the formation coordinates of the Follower UAVs according to the Leader's position.

        Collision avoidance: Each drone is assigned a unique safe altitude layer during the leader
        pass. Drones first spread out to different altitudes, take their horizontal positions, and
        then gradually descend/ascend to the formation altitude.
        """
        if self.leader_id not in self.drones: return

        leader = self.drones[self.leader_id]
        if leader["lat"] == 0: return

        # Instant NED velocity vector of the leader → will be given as feed-forward to the followers
        leader_vel = (leader["vx"], leader["vy"], leader["vz"])

        # Check migration status
        in_transition = False
        progress = 1.0  # 1.0 = transition completed, normal formation
        if self.transition_start is not None:
            elapsed = time.time() - self.transition_start
            if elapsed < self.transition_duration:
                in_transition = True
                progress = elapsed / self.transition_duration  # 0.0 → 1.0
            else:
                self.transition_start = None  # Migration completed

        for drone_id, drone in self.drones.items():
            if drone_id == self.leader_id: continue

            offsets = self.formation_map.get(drone_id, {"dx": 0, "dy": 0, "dz": 0})

            # Horizontal position is always the final target (go immediately to correct x/y)
            target_lat, target_lon = get_target_location(
                leader["lat"], leader["lon"],
                offsets["dx"], offsets["dy"],
                leader["heading"]
            )
            final_alt = leader["alt"] + offsets["dz"]

            if in_transition:
                # COLLISION PREVENTION: Unique safe altitude layer to each drone stacked 4m apart in order drone_id
                safe_alt = leader["alt"] + 8 + (drone_id * 4)
                # Gradually move from safe altitude to formation altitude
                target_alt = safe_alt + progress * (final_alt - safe_alt)
            else:
                target_alt = final_alt

            # Calculate yaw offset based on formation position
            yaw_offset_deg = self._get_yaw_offset_for_position(drone_id)
            target_yaw_rad = math.radians((leader["heading"] + yaw_offset_deg) % 360)

            self.send_target_location(drone_id, target_lat, target_lon, target_alt, target_yaw_rad, vel_ned=leader_vel)

    def listen_redis_events(self):
        message = self.pubsub.get_message()
        if message and message['type'] == 'message':
            data = json.loads(message['data'])

            # When a Moving and Faulty Target Transmits GPS
            if data["status"] == "MOVING_TARGET":
                # Add incoming erroneous (jumping) data to the history list
                self.target_history.append((data["lat"], data["lon"], data["alt"]))

                # Maintain tail size
                if len(self.target_history) > self.filter_window_size:
                    self.target_history.pop(0)

                # Filter/smooth data with Moving Average
                self.swarm_target_lat = sum(p[0] for p in self.target_history) / len(self.target_history)
                self.swarm_target_lon = sum(p[1] for p in self.target_history) / len(self.target_history)
                self.swarm_target_alt = sum(p[2] for p in self.target_history) / len(self.target_history)

            # If a UAV (Companion Computer) sees the target on its camera
            elif data["status"] == "TARGET_FOUND" and self.state == "FORMATION_FLIGHT":
                finder_id = data["drone_id"]
                print(f"!!! [ATTENTION] UAV {finder_id} spotted the target! Ground station releases command, Edge Control is activated. !!!")

                self.state = "VISUAL_SERVO"
                self._swap_leader(finder_id)
                # NOTE: Waypoints with "send_target_location" will NO longer be sent to this new leader.
                # companion_node.py handles speed commands autonomously.

    def run(self):
        print(f"Herd Manager Started. Current Leader: UAV {self.leader_id}")
        while True:
            self.update_telemetry()
            self.listen_redis_events()

            if self.state == "FORMATION_FLIGHT":
                # Direct the leader to the received target 1 Hz even if it is incorrect (let the leader look at the target)
                if self.swarm_target_lat != 0.0:
                    leader = self.drones.get(self.leader_id)
                    if leader and leader["lat"] != 0.0:
                        # Calculate bearing from leader to target → have the drone face the direction it flies
                        d_lat = self.swarm_target_lat - leader["lat"]
                        d_lon = self.swarm_target_lon - leader["lon"]
                        # North reference bearing (radians) with atan2
                        bearing_rad = math.atan2(d_lon * math.cos(math.radians(leader["lat"])), d_lat)
                        # Normalize to range 0-2π
                        if bearing_rad < 0:
                            bearing_rad += 2 * math.pi
                        leader_yaw_rad = bearing_rad
                    else:
                        leader_yaw_rad = 0.0
                    self.send_target_location(self.leader_id, self.swarm_target_lat, self.swarm_target_lon, self.swarm_target_alt, leader_yaw_rad)

                # Have followers line up behind the leader
                self.maintain_formation()

            elif self.state == "VISUAL_SERVO":
                # NO COMMAND to the Leader UAV. Companion computer pushes speed from MAVLink. We, as the Ground
                # Station, only ensure that the other 4 UAV follows the new leader.
                self.maintain_formation()

            time.sleep(0.05) # 20 Hz loop

if __name__ == "__main__":
    manager = SwarmManager()
    manager.run()
