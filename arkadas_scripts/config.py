import json

SWARM_CONFIG = {
    # Redis Ayarları
    "REDIS_HOST": "localhost",
    "REDIS_PORT": 6379,
    
    # MAVLink Bağlantıları (Sürü İHA'ları)
    # connection_string: Ground Station bağlantısı (telemetri + komut)
    # companion_string:  Companion Node bağlantısı (edge kontrol)
    "DRONES": {
        0: {"connection_string": "udp:127.0.0.1:14551", "companion_string": "udp:127.0.0.1:14651", "role": "LEADER"},
        1: {"connection_string": "udp:127.0.0.1:14561", "companion_string": "udp:127.0.0.1:14661", "role": "FOLLOWER"},
        2: {"connection_string": "udp:127.0.0.1:14571", "companion_string": "udp:127.0.0.1:14671", "role": "FOLLOWER"},
        3: {"connection_string": "udp:127.0.0.1:14581", "companion_string": "udp:127.0.0.1:14681", "role": "FOLLOWER"},
        4: {"connection_string": "udp:127.0.0.1:14591", "companion_string": "udp:127.0.0.1:14691", "role": "FOLLOWER"}
    },
    
    # Hedef Uçak (Sabit Kanat) Bağlantısı (6. SITL)
    "TARGET_PLANE": {
        "connection_string": "udp:127.0.0.1:14601"
    },
    
    # Hareketli Hedef Ayarları
    "TARGET_GPS_ERROR_MARGIN": 30.0, # Metre cinsinden konuma eklenecek rastgele GPS sapması
    "TARGET_UPDATE_RATE": 1.0,       # Saniyede kaç kere (Hz) konum yayını yapılacak
    
    # Formasyon Ayarları (Metre cinsinden) v shape
    # "FORMATION": {
    #     1: {"dx": -5.0, "dy": -5.0, "dz": 0.0},
    #     2: {"dx":  5.0, "dy": -5.0, "dz": 0.0},
    #     3: {"dx": -5.0, "dy": -10.0, "dz": 0.0},
    #     4: {"dx":  5.0, "dy": -10.0, "dz": 0.0} 
    # },

    #diamond shape
    "FORMATION": {
        1: {"dx": 0, "dy": 0.0, "dz": 5.0},
        2: {"dx":  5.0, "dy": 0.0, "dz": 0.0},
        3: {"dx": 0, "dy": 0.0, "dz": -5.0},
        4: {"dx": -5.0, "dy": 0.0, "dz": 0.0}
    },
    
    # Kamera / Gimbal Arama Açıları (Derece)
    "CAMERA_ANGLES": {
        0: {"pitch": -15, "yaw": 0},
        1: {"pitch": -30, "yaw": -30},
        2: {"pitch": -30, "yaw": 30},
        3: {"pitch": -45, "yaw": -45},
        4: {"pitch": -45, "yaw": 45}
    },
    
    # Visual Servo PID Katsayıları
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
