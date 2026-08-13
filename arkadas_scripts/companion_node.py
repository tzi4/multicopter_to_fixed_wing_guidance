import time
import json
import redis
import argparse
from pymavlink import mavutil
from config import SWARM_CONFIG
from visual_servo import VisualServoController

class CompanionNode:
    """
    Her bir İHA'nın kendi yardımcı bilgisayarında (Raspberry Pi/Jetson vb.) çalışır.
    Sadece kendi İHA'sının MAVLink portuna doğrudan bağlıdır (Sıfır Gecikme).
    Kamerayı (YOLO) okur, BBox bulursa Visual Servo PID çalıştırır ve hızı FCU'ya anında basar.
    Ayrıca Yer İstasyonuna (Redis üzerinden) TARGET_FOUND mesajı göndererek Liderliği devralır.
    """
    def __init__(self, drone_id):
        self.drone_id = drone_id
        cfg = SWARM_CONFIG["DRONES"][self.drone_id]
        
        self.visual_servo = VisualServoController()
        
        # Companion Node kendi özel portuna bağlanır (ground_station ile çakışmaz)
        companion_conn = cfg.get('companion_string', cfg['connection_string'])
        print(f"İHA {self.drone_id} Companion Node yerel MAVLink'e bağlanıyor: {companion_conn}")
        self.conn = mavutil.mavlink_connection(companion_conn)
        self.conn.wait_heartbeat(timeout=15)
        print(f"İHA {self.drone_id} Heartbeat alındı (sys:{self.conn.target_system} comp:{self.conn.target_component})")
        
        # Redis bağlantısı (Yer İstasyonuna veya Ağ Merkezine)
        self.redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"], decode_responses=True)
        
        self.target_locked = False

    def send_velocity_command(self, vx, vy, vz):
        """Hesaplanan PID hızlarını doğrudan (ağa çıkmadan) yerel FCU'ya gönderir"""
        # MAV_FRAME_BODY_OFFSET_NED (9): Body-relative hızlar (ileri, sağ, aşağı)
        # ArduPilot MAV_FRAME_BODY_NED'i desteklemez, BODY_OFFSET_NED kullanılmalı
        #
        # Bitmask: 0b0000_1_1_1_111_000_111 = 0x0FC7 = 4039
        #   Bit 11 (yaw_rate)=1 yoksay, Bit 10 (yaw)=1 yoksay, Bit 9 (force)=1 yoksay
        #   Bit 8-6 (ivmeler)=1 yoksay, Bit 5-3 (vx,vy,vz)=0 KULLAN, Bit 2-0 (pos)=1 yoksay
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # = 4039 → Sadece hızları kullan
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)

    def run(self):
        print(f"Companion Node (İHA {self.drone_id}) Başladı. Kamera ve MAVLink dinleniyor...")
        
        # Simülasyon gereği: Gerçek bir kameradan frame okumak yerine mock_vision'dan
        # kendi İHA'mıza ait BBox verilerini Redis'ten çekiyoruz. (Gerçekte burada cv2.VideoCapture olur).
        pubsub = self.redis_client.pubsub()
        pubsub.subscribe("TARGET_DETECTION") 
        
        while True:
            # Görüntü İşleme / YOLO Tahmini Döngüsü
            message = pubsub.get_message()
            if message and message['type'] == 'message':
                data = json.loads(message['data'])
                
                # Sadece BİZİM drone'umuzun kamerası hedef gördüyse
                if data["status"] == "TARGET_TRACKING" and data.get("drone_id") == self.drone_id:
                    if not self.target_locked:
                        self.target_locked = True
                        print(f"[İHA {self.drone_id} EDGE KONTROL] Kamera Hedefi Tespit Etti! Liderlik alınıyor...")
                        # Ağa bildir: "Hedefi Ben Gördüm, Yer İstasyonu Bana Waypoint Atmayı Bıraksın!"
                        msg = {"status": "TARGET_FOUND", "drone_id": self.drone_id}
                        self.redis_client.publish("TARGET_DETECTION", json.dumps(msg))
                    
                    bbox_x = data["bbox_center_x"]
                    bbox_y = data["bbox_center_y"]
                    
                    # DAĞITIK KONTROL: Visual Servo Kendi Companion Bilgisayarında Çalışıyor
                    # (Bu sayede ağ pingi veya yer istasyonu yükü hıza yansımaz)
                    vx, vy, vz = self.visual_servo.calculate_velocity(bbox_x, bbox_y)
                    self.send_velocity_command(vx, vy, vz)
                    
            time.sleep(0.05) # 20 FPS (Kameranın FPS hızı kadar bekler)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Companion Computer Node")
    parser.add_argument("--id", type=int, required=True, help="Drone ID (0-4)")
    args = parser.parse_args()
    
    node = CompanionNode(args.id)
    node.run()
