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
    Hedef uçaktan (6. SITL) MAVLink ile telemetri alır.
    GPS koordinatının üzerine config'deki hata payı (margin) kadar rastgele bir sapma ekler.
    Bu hatalı konumu 1 Hz (veya config'deki rate) ile Redis üzerinden yayınlar.
    """
    conn_str = SWARM_CONFIG['TARGET_PLANE']['connection_string']
    print(f"Hedef Uçak MAVLink'ine bağlanılıyor: {conn_str}")
    try:
        conn = mavutil.mavlink_connection(conn_str)
        conn.wait_heartbeat(timeout=15)
        print(f"Hedef Uçak Bağlantısı Kuruldu (sys:{conn.target_system} comp:{conn.target_component})")
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        return
        
    try:
        redis_client = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"])
    except Exception as e:
        print(f"Redis bağlantı hatası: {e}")
        return

    update_rate = SWARM_CONFIG.get("TARGET_UPDATE_RATE", 1.0)
    error_margin = SWARM_CONFIG.get("TARGET_GPS_ERROR_MARGIN", 30.0)

    print(f"Hatalı Hedef Yayını Başladı. (Frekans: {update_rate} Hz, Hata Payı: {error_margin}m)")

    while True:
        # Hedef uçağın anlık telemetrisini oku (SITL'den)
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2.0)
        if msg:
            real_lat = msg.lat / 1e7
            real_lon = msg.lon / 1e7
            real_alt = msg.relative_alt / 1000.0
            
            # Rastgele sapma ekle (Dairesel rastgele dağılım)
            # Rastgele bir yön (0-360 derece) ve rastgele bir mesafe (0-error_margin) seçelim
            random_angle = random.uniform(0, 360)
            random_distance = random.uniform(0, error_margin)
            
            # math_utils.get_target_location fonksiyonu dx(sağ) ve dy(ileri) ofset alır.
            # Heading açısını 0 kabul edersek (Kuzey referanslı), Trigonometri ile dx ve dy'yi bulabiliriz.
            dx = random_distance * math.sin(math.radians(random_angle)) # Doğu/Batı ofseti
            dy = random_distance * math.cos(math.radians(random_angle)) # Kuzey/Güney ofseti
            
            # Yeni (hatalı) koordinatı bul
            noisy_lat, noisy_lon = get_target_location(real_lat, real_lon, dx, dy, 0.0)
            
            payload = {
                "status": "MOVING_TARGET",
                "lat": noisy_lat,
                "lon": noisy_lon,
                "alt": real_alt,
                "error_margin_applied": random_distance
            }
            
            redis_client.publish("TARGET_DETECTION", json.dumps(payload))
            print(f"Hedef Uçak: Gerçek({real_lat:.5f}, {real_lon:.5f}) -> Yayınlanan({noisy_lat:.5f}, {noisy_lon:.5f}) | Sapma: {random_distance:.1f}m")
            
        time.sleep(1.0 / update_rate)

if __name__ == "__main__":
    broadcast_noisy_target()
