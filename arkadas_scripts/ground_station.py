import time
import json
import redis
import math
from pymavlink import mavutil
from config import SWARM_CONFIG
from math_utils import get_target_location

class SwarmManager:
    """
    Yer İstasyonu (Ground Station) Yöneticisi:
    - Hedefin hatalı GPS lokasyonunu (MOVING_TARGET) Redis'ten dinler.
    - Sürünün mevcut Liderini bu hatalı lokasyona (Guided olarak) gönderir.
    - Takipçi İHA'ların (kare formasyonu) koordinatlarını hesaplar ve yollar.
    - Herhangi bir İHA kamerasında hedefi gördüğünde (TARGET_FOUND),
      Liderliği O'na geçirir ve O'na Waypoint yollamayı KESER (Çünkü Visual Servo O'nun kendi içinde çalışacaktır).
    """
    def __init__(self):
        self.drones = {}
        self.state = "FORMATION_FLIGHT"  
        self.leader_id = 0               
        
        # Dinamik formasyon haritası: drone_id → ofset
        # Lider değiştiğinde eski lider, yeni liderin boşalttığı pozisyonu alır
        self.formation_map = dict(SWARM_CONFIG["FORMATION"])
        
        # Lider geçişi sırasında çarpışma önleme zamanlayıcısı
        self.transition_start = None
        self.transition_duration = 5.0  # saniye — kademeli geçiş süresi
        
        # Hareketli Hedefin (Sürü Odak Noktası) Koordinatları
        self.swarm_target_lat = 0.0
        self.swarm_target_lon = 0.0
        self.swarm_target_alt = 0.0
        
        # Filtreleme (Moving Average) için geçmiş veri kuyruğu
        self.target_history = []
        self.filter_window_size = 5 # Son 5 verinin ortalaması alınacak
        
        # Redis bağlantısı
        try:
            self.redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"], decode_responses=True)
            self.pubsub = self.redis_client.pubsub()
            self.pubsub.subscribe("TARGET_DETECTION")
        except Exception as e:
            print(f"[HATA] Redis bağlantı hatası: {e}")
            
        self.connect_to_drones()
        
    def connect_to_drones(self):
        for drone_id, cfg in SWARM_CONFIG["DRONES"].items():
            print(f"İHA {drone_id} Telemetrisi bağlanıyor: {cfg['connection_string']}")
            try:
                connection = mavutil.mavlink_connection(cfg['connection_string'])
                connection.wait_heartbeat(timeout=15)
                print(f"İHA {drone_id} Heartbeat alındı (sys:{connection.target_system} comp:{connection.target_component})")
                self.drones[drone_id] = {
                    "conn": connection,
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "heading": 0.0,
                    "vx": 0.0,  # Kuzey hızı (m/s)
                    "vy": 0.0,  # Doğu hızı (m/s)
                    "vz": 0.0   # Aşağı hızı (m/s)
                }
            except Exception as e:
                print(f"[HATA] İHA {drone_id} bağlantı başarısız: {e}")

    def update_telemetry(self):
        for drone_id, drone in self.drones.items():
            conn = drone["conn"]
            # Kuyruktaki TÜM mesajları oku, en güncel veriyi kullan (stale data önlemi)
            while True:
                msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    drone["lat"] = msg.lat / 1e7
                    drone["lon"] = msg.lon / 1e7
                    drone["alt"] = msg.relative_alt / 1000.0  
                    drone["heading"] = msg.hdg / 100.0
                    drone["vx"] = msg.vx / 100.0   # cm/s → m/s (Kuzey)
                    drone["vy"] = msg.vy / 100.0   # cm/s → m/s (Doğu)
                    drone["vz"] = msg.vz / 100.0   # cm/s → m/s (Aşağı)
                elif msg.get_type() == 'VFR_HUD':
                    drone["heading"] = msg.heading

    def send_target_location(self, drone_id, lat, lon, alt, yaw_rad=None, vel_ned=None):
        """
        Bir İHA'ya MAVLink üzerinden GPS hedefi, isteğe bağlı Yaw ve hız vektörü gönderir.
        
        vel_ned: (vx, vy, vz) tuple - NED çerçevesinde hız (m/s).
                 Liderin hız vektörü feed-forward olarak verilir, böylece
                 takipçiler liderin hızını (ör. 17 m/s) eşleştirir.
        """
        conn = self.drones[drone_id]["conn"]
        
        vx, vy, vz = vel_ned if vel_ned else (0.0, 0.0, 0.0)
        yaw_val = yaw_rad if yaw_rad is not None else 0.0
        
        # Bitmask: 1=yoksay, 0=kullan
        #   Bit 0-2: lat,lon,alt  | Bit 3-5: vx,vy,vz | Bit 6-8: ax,ay,az
        #   Bit 9: force | Bit 10: yaw | Bit 11: yaw_rate
        if vel_ned and yaw_rad is not None:
            # Pozisyon + Hız + Yaw
            type_mask = 0b0000101111000000  # 3008
        elif yaw_rad is not None:
            # Pozisyon + Yaw (hız yok)
            type_mask = 0b0000101111111000  # 3064
        elif vel_ned:
            # Pozisyon + Hız (yaw yok)
            type_mask = 0b0000111111000000  # 4032
        else:
            # Sadece Pozisyon
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
        Liderliği değiştirir. Eski lider, yeni lidere göre TERS konumda
        kalır (minimum hareket). Ters konumdaki drone ise boşalan slota geçer.
        
        Örnek: Drone 3 (dz:-5, alt) lider olunca:
          - Drone 0 (eski lider) zaten drone 3'ün 5m üstünde → dz:+5 slotunu alır (yerinde kalır)
          - Drone 1 (dz:+5 slotundaydı) → boşalan dz:-5 slotuna geçer
        """
        old_leader_id = self.leader_id
        
        if new_leader_id in self.formation_map:
            # Yeni liderin eski formasyon ofseti
            vacated_slot = self.formation_map.pop(new_leader_id)
            
            # Eski lider, yeni lidere göre TERS konumda → minimum hareket
            inverse_slot = {
                "dx": -vacated_slot["dx"],
                "dy": -vacated_slot["dy"],
                "dz": -vacated_slot["dz"]
            }
            
            # Bu ters konum başka bir drone'a ait mi?
            displaced_drone_id = None
            for did, slot in self.formation_map.items():
                if (slot["dx"] == inverse_slot["dx"] and 
                    slot["dy"] == inverse_slot["dy"] and 
                    slot["dz"] == inverse_slot["dz"]):
                    displaced_drone_id = did
                    break
            
            if displaced_drone_id is not None:
                # O drone'u boşalan slota taşı
                self.formation_map[displaced_drone_id] = vacated_slot
                print(f"[FORMASYON] İHA {displaced_drone_id} → pozisyon dx:{vacated_slot['dx']}, dy:{vacated_slot['dy']}, dz:{vacated_slot['dz']}")
            
            # Eski lideri ters konuma koy (yerinde kalır)
            self.formation_map[old_leader_id] = inverse_slot
            print(f"[FORMASYON] İHA {old_leader_id} → pozisyon dx:{inverse_slot['dx']}, dy:{inverse_slot['dy']}, dz:{inverse_slot['dz']}")
        
        self.leader_id = new_leader_id
        self.transition_start = time.time()  # Kademeli geçişi başlat
        print(f"[LİDER DEĞİŞİM] İHA {old_leader_id} → İHA {new_leader_id} (geçiş başladı)")

    def _get_yaw_offset_for_position(self, drone_id):
        """
        Formasyon pozisyonuna göre yaw ofsetini belirler (derece cinsinden).
        - Sağ taraftaki drone (dx > 0): +20°  (sağa baksın)
        - Sol taraftaki drone (dx < 0): -20°  (sola baksın)
        - Üst/Alt/Merkez (dx == 0):      0°   (karşıya baksın)
        """
        offsets = self.formation_map.get(drone_id, {"dx": 0, "dy": 0, "dz": 0})
        dx = offsets.get("dx", 0)
        
        if dx > 0:      # Sağ taraftaki drone
            return 20.0
        elif dx < 0:    # Sol taraftaki drone
            return -20.0
        else:           # Üst, alt veya merkez → karşıya baksın
            return 0.0

    def maintain_formation(self):
        """Takipçi İHA'ların Liderin pozisyonuna göre formasyon koordinatlarını hesaplar ve gönderir.
        
        Çarpışma önleme: Lider geçişi sırasında her drone'a benzersiz güvenli
        irtifa katmanı atanır. Drone'lar önce farklı irtifalara yayılır, yatay
        konumlarını alır, sonra kademeli olarak formasyon irtifasına iner/çıkar.
        """
        if self.leader_id not in self.drones: return
        
        leader = self.drones[self.leader_id]
        if leader["lat"] == 0: return 
        
        # Liderin anlık NED hız vektörü → takipçilere feed-forward olarak verilecek
        leader_vel = (leader["vx"], leader["vy"], leader["vz"])
        
        # Geçiş durumunu kontrol et
        in_transition = False
        progress = 1.0  # 1.0 = geçiş tamamlandı, normal formasyon
        if self.transition_start is not None:
            elapsed = time.time() - self.transition_start
            if elapsed < self.transition_duration:
                in_transition = True
                progress = elapsed / self.transition_duration  # 0.0 → 1.0
            else:
                self.transition_start = None  # Geçiş tamamlandı
        
        for drone_id, drone in self.drones.items():
            if drone_id == self.leader_id: continue 
            
            offsets = self.formation_map.get(drone_id, {"dx": 0, "dy": 0, "dz": 0})
            
            # Yatay konum her zaman nihai hedef (hemen doğru x/y'ye git)
            target_lat, target_lon = get_target_location(
                leader["lat"], leader["lon"], 
                offsets["dx"], offsets["dy"], 
                leader["heading"]
            )
            final_alt = leader["alt"] + offsets["dz"]
            
            if in_transition:
                # ÇARPIŞMA ÖNLEME: Her drone'a benzersiz güvenli irtifa katmanı
                # drone_id sırasına göre 4m aralıklarla istiflenir
                safe_alt = leader["alt"] + 8 + (drone_id * 4)
                # Kademeli olarak güvenli irtifadan formasyon irtifasına geç
                target_alt = safe_alt + progress * (final_alt - safe_alt)
            else:
                target_alt = final_alt
            
            # Formasyon pozisyonuna göre yaw ofseti hesapla
            yaw_offset_deg = self._get_yaw_offset_for_position(drone_id)
            target_yaw_rad = math.radians((leader["heading"] + yaw_offset_deg) % 360)
            
            self.send_target_location(drone_id, target_lat, target_lon, target_alt, target_yaw_rad, vel_ned=leader_vel)

    def listen_redis_events(self):
        message = self.pubsub.get_message()
        if message and message['type'] == 'message':
            data = json.loads(message['data'])
            
            # Hareketli ve Hatalı Hedef GPS Verisi Geldiğinde
            if data["status"] == "MOVING_TARGET":
                # Gelen hatalı (ziplama yapan) veriyi geçmiş listesine ekle
                self.target_history.append((data["lat"], data["lon"], data["alt"]))
                
                # Kuyruk boyutunu koru
                if len(self.target_history) > self.filter_window_size:
                    self.target_history.pop(0)
                    
                # Hareketli Ortalama (Moving Average) ile veriyi filtrele/yumuşat
                self.swarm_target_lat = sum(p[0] for p in self.target_history) / len(self.target_history)
                self.swarm_target_lon = sum(p[1] for p in self.target_history) / len(self.target_history)
                self.swarm_target_alt = sum(p[2] for p in self.target_history) / len(self.target_history)
            
            # Bir İHA (Companion Computer) hedefi kamerasında gördüyse
            elif data["status"] == "TARGET_FOUND" and self.state == "FORMATION_FLIGHT":
                finder_id = data["drone_id"]
                print(f"!!! [DİKKAT] İHA {finder_id} hedefi gördü! Yer istasyonu komutu bırakıyor, Edge Kontrol devrede. !!!")
                
                self.state = "VISUAL_SERVO"
                self._swap_leader(finder_id)
                # NOT: Artık bu yeni lidere "send_target_location" ile Waypoint GÖNDERİLMEYECEK.
                # Hız komutlarını otonom olarak companion_node.py hallediyor.

    def run(self):
        print(f"Sürü Yöneticisi Başlatıldı. Mevcut Lider: İHA {self.leader_id}")
        while True:
            self.update_telemetry()
            self.listen_redis_events()
            
            if self.state == "FORMATION_FLIGHT":
                # Lideri hatalı da olsa alınan 1 Hz hedefine yönlendir (lider hedefe baksın)
                if self.swarm_target_lat != 0.0:
                    leader = self.drones.get(self.leader_id)
                    if leader and leader["lat"] != 0.0:
                        # Liderden hedefe bearing hesapla → drone uçtuğu yöne baksın
                        d_lat = self.swarm_target_lat - leader["lat"]
                        d_lon = self.swarm_target_lon - leader["lon"]
                        # atan2 ile Kuzey referanslı bearing (radyan)
                        bearing_rad = math.atan2(d_lon * math.cos(math.radians(leader["lat"])), d_lat)
                        # 0-2π aralığına normalize et
                        if bearing_rad < 0:
                            bearing_rad += 2 * math.pi
                        leader_yaw_rad = bearing_rad
                    else:
                        leader_yaw_rad = 0.0
                    self.send_target_location(self.leader_id, self.swarm_target_lat, self.swarm_target_lon, self.swarm_target_alt, leader_yaw_rad)
                
                # Takipçiler liderin arkasında hizalansın
                self.maintain_formation()
                
            elif self.state == "VISUAL_SERVO":
                # Lider İHA'ya KOMUT YOK. Companion bilgisayarı MAVLink'ten hızı basıyor.
                # Biz sadece Yer İstasyonu olarak diğer 4 İHA'nın yeni liderin peşine takılmasını sağlıyoruz.
                self.maintain_formation()
                
            time.sleep(0.05) # 20 Hz döngü

if __name__ == "__main__":
    manager = SwarmManager()
    manager.run()
