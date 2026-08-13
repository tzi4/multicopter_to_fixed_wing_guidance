import time
import json
import redis
from config import SWARM_CONFIG

def simulate_detection():
    """
    Kamera ve YOLO entegrasyonu yapılana kadar ağı test etmek için
    sahte Redis mesajları üreten script.
    """
    r = redis.Redis(host=SWARM_CONFIG["REDIS_HOST"], port=SWARM_CONFIG["REDIS_PORT"])
    
    print("SİMÜLASYON BAŞLADI: 10 saniye boyunca İHA'lar formasyonda uçuyor...")
    for i in range(10, 0, -1):
        print(f"{i} saniye kaldı...")
        time.sleep(1)
        
    # Senaryo: İHA 3 (Alt Takipçi) hedefi kendi kamerasında gördü
    finder_drone_id = 3
    
    print(f"SİMÜLASYON: İHA {finder_drone_id} hedefi gördü! TARGET_FOUND sinyali gönderiliyor...")
    msg = {
        "status": "TARGET_FOUND",
        "drone_id": finder_drone_id
    }
    r.publish("TARGET_DETECTION", json.dumps(msg))
    
    print("SİMÜLASYON: Hedef takibi başladı. Bounding Box koordinatları Redis'e basılıyor...")
    
    # Hedef kameranın merkezinde (320, 240) duruyormuş gibi simüle edelim.
    center_x = 320
    center_y = 240
    
    for i in range(200): # Yaklaşık 20 saniye sürecek takip
        # Hedef çok hafif sağa sola hareket etsin (PID'i test etmek için)
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
            print(f"Yayınlandı BBOX -> X: {bbox_x}, Y: {bbox_y}")
            
        time.sleep(0.1) # 10 FPS (10 Hz)
        
    print("SİMÜLASYON BİTTİ.")

if __name__ == "__main__":
    simulate_detection()
