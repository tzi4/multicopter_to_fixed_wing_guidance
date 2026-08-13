import math

# Dünya yarıçapı (metre)
R_EARTH = 6378137.0

def get_target_location(lat, lon, dx, dy, heading_deg):
    """
    Verilen (lat, lon) konumuna, heading (yönelim) açısını dikkate alarak 
    dx (sağ/sol) ve dy (ileri/geri) ofsetleri ekler ve yeni GPS koordinatını döner.
    
    :param lat: Liderin Latitude (Enlem)
    :param lon: Liderin Longitude (Boylam)
    :param dx: Sağa doğru olan ofset (metre) - Sola doğruysa eksi (-)
    :param dy: İleriye doğru olan ofset (metre) - Geriye doğruysa eksi (-)
    :param heading_deg: Liderin pusula açısı (Kuzey 0, Doğu 90)
    :return: (new_lat, new_lon)
    """
    # Heading açısını radyana çevir
    heading_rad = math.radians(heading_deg)
    
    # dx ve dy'yi Dünya eksenlerindeki (Kuzey/Doğu) vektörlere çeviriyoruz
    # X ekseni kuzey (0 derece), Y ekseni doğu (90 derece)
    d_north = dy * math.cos(heading_rad) - dx * math.sin(heading_rad)
    d_east = dy * math.sin(heading_rad) + dx * math.cos(heading_rad)
    
    # Koordinat ofsetlerini Derece cinsine çevir (Haversine yaklaşımı)
    d_lat = (d_north / R_EARTH) * (180.0 / math.pi)
    d_lon = (d_east / (R_EARTH * math.cos(math.pi * lat / 180.0))) * (180.0 / math.pi)
    
    # Liderin konumuna ekleyip yeni GPS koordinatlarını bul
    new_lat = lat + d_lat
    new_lon = lon + d_lon
    
    return new_lat, new_lon
