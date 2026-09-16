import json

SWARM_CONFIG = {
    # Redis Settings
    "REDIS_HOST": "localhost",
    "REDIS_PORT": 6379,

    # MAVLink Connections (Swarm UAVs) connection_string: Ground Station connection (telemetry + command)
    # companion_string: Companion Node connection (edge ​​control)
    "DRONES": {
        0: {"connection_string": "udp:127.0.0.1:14551", "companion_string": "udp:127.0.0.1:14651", "role": "LEADER"},
        1: {"connection_string": "udp:127.0.0.1:14561", "companion_string": "udp:127.0.0.1:14661", "role": "FOLLOWER"},
        2: {"connection_string": "udp:127.0.0.1:14571", "companion_string": "udp:127.0.0.1:14671", "role": "FOLLOWER"},
        3: {"connection_string": "udp:127.0.0.1:14581", "companion_string": "udp:127.0.0.1:14681", "role": "FOLLOWER"},
        4: {"connection_string": "udp:127.0.0.1:14591", "companion_string": "udp:127.0.0.1:14691", "role": "FOLLOWER"}
    },

    # Target Aircraft (Fixed Wing) Mount (6. SITL)
    "TARGET_PLANE": {
        "connection_string": "udp:127.0.0.1:14601"
    },

    # Moving Target Settings
    "TARGET_GPS_ERROR_MARGIN": 30.0, # Random GPS offset to be added to position in meters
    "TARGET_UPDATE_RATE": 1.0,       # How many times per second (Hz) will the location be broadcast?

    # Formation Settings (in Meters) v shape "FORMATION": { 1: {"dx": -5.0, "dy": -5.0, "dz": 0.0}, 2:
    # {"dx": 5.0, "dy": -5.0, "dz": 0.0}, 3: {"dx": -5.0, "dy": -10.0, "dz": 0.0}, 4: {"dx": 5.0, "dy":
    # -10.0, "dz": 0.0} },

    #diamond shape
    "FORMATION": {
        1: {"dx": 0, "dy": 0.0, "dz": 5.0},
        2: {"dx":  5.0, "dy": 0.0, "dz": 0.0},
        3: {"dx": 0, "dy": 0.0, "dz": -5.0},
        4: {"dx": -5.0, "dy": 0.0, "dz": 0.0}
    },

    # Camera/Gimbal Search Pains (Degrees)
    "CAMERA_ANGLES": {
        0: {"pitch": -15, "yaw": 0},
        1: {"pitch": -30, "yaw": -30},
        2: {"pitch": -30, "yaw": 30},
        3: {"pitch": -45, "yaw": -45},
        4: {"pitch": -45, "yaw": 45}
    },

    # Visual Servo PID Coefficients
    "PID": {
        "Kp_x": 0.005,
        "Ki_x": 0.0,
        "Kd_x": 0.001,
        "Kp_y": 0.005,
        "Ki_y": 0.0,
        "Kd_y": 0.001
    },

    "IMAGE_WIDTH": 640,
    "IMAGE_HEIGHT": 480
}
