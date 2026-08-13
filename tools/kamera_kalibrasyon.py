#!/usr/bin/env python3
"""
kamera_kalibrasyon.py - kameranin GERCEK bakis eksenini olcer
=============================================================
Neden gerekli: models/suru_drone_*/model.sdf icindeki
<pose>0 0 0.15 0 <pitch> 0</pose> degerinin isareti Gazebo'da hangi yone
karsilik geliyor, dolayli testlerle netlesmedi. Bu arac dolayli akil
yurutmeyi tamamen atlar:

  - Araclarin GERCEK konum/yonelimini Gazebo'dan alir (/gazebo/model_states),
    MAVLink'ten DEGIL. MAVLink akisi bayat olabiliyor (olculdu: 6 s'de 9
    mesaj birikiyor), Gazebo poz'u ise simulasyonun kendisidir.
  - Ayni anda kameradan mor hedefi tespit eder.
  - Her tespit icin (gercek yukselis, gercek yan aci) <-> (bbox y, bbox x)
    ciftini kaydeder ve dogru uydurur.

Ciktilar:
  - kadraj merkezine (y=360) karsilik gelen YUKSELIS = kameranin optik
    ekseninin ufka gore acisi (govde pitch'i cikarilmis halde)
  - derece/piksel olcegi -> beklenen 12.95/720 = 0.01799 (dikey)
                                     22.81/1280 = 0.01782 (yatay)
  Olculen olcek beklenenden farkliysa FOV/goruntu ayari tutarsiz demektir.
"""

import argparse
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image


def q_to_euler(q):
    sr = 2 * (q.w * q.x + q.y * q.z)
    cr = 1 - 2 * (q.x * q.x + q.y * q.y)
    sp = 2 * (q.w * q.y - q.z * q.x)
    sy = 2 * (q.w * q.z + q.x * q.y)
    cy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return (math.atan2(sr, cr), math.asin(max(-1.0, min(1.0, sp))),
            math.atan2(sy, cy))


class Kalibrasyon:
    def __init__(self, avci, hedef, topic, sure):
        self.avci_ad, self.hedef_ad, self.sure = avci, hedef, sure
        self.bridge = CvBridge()
        self.durum = None
        self.ornekler = []
        # DIKKAT: kare sayaci sart. rospy.wait_for_message() ile olcum
        # YAPILMAZ - gazebo_ros_camera TEMBEL yayin yapar, her yeni
        # abonelikte once BAYAT onbellek karesi doner (olculdu: 6 ardisik
        # wait_for_message ayni seq=4604'u dondurdu). Kalici abone sart.
        self.kare_sayisi = 0
        self.son_seq = -1
        self.en_yakin = None
        rospy.init_node('kamera_kalibrasyon', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._durum_cb,
                         queue_size=1)
        rospy.Subscriber(topic, Image, self._kare_cb, queue_size=1)
        print(f"dinleniyor: {topic}  (avci={avci} hedef={hedef})", flush=True)

    def _durum_cb(self, msg):
        try:
            ai = msg.name.index(self.avci_ad)
            hi = msg.name.index(self.hedef_ad)
        except ValueError:
            return
        self.durum = (msg.pose[ai], msg.pose[hi])

    def _kare_cb(self, msg):
        self.kare_sayisi += 1
        self.son_seq = msg.header.seq
        if self.durum is None:
            return
        avci, hedef = self.durum
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([140, 120, 60]),
                           np.array([160, 255, 255]))
        if int(mask.sum() // 255) < 15:
            return
        ys, xs = np.where(mask > 0)
        bx, by = float(xs.mean()), float(ys.mean())

        # Hedefin avciya gore GERCEK yonu (Gazebo dunya cercevesinde)
        dx = hedef.position.x - avci.position.x
        dy = hedef.position.y - avci.position.y
        dz = hedef.position.z - avci.position.z
        roll, pitch, yaw = q_to_euler(avci.orientation)

        # Once govde cercevesine cevir (yaw), sonra pitch/roll'u cikar:
        # kameranin GOVDEYE gore sabit acisini olcmek istiyoruz.
        ileri = dx * math.cos(yaw) + dy * math.sin(yaw)
        sag = -dx * math.sin(yaw) + dy * math.cos(yaw)
        # govde pitch/roll de-rotasyonu (kucuk aci degil, tam donusum)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        x_b = ileri * cp + dz * sp
        y_b = sag * cr - (-ileri * sp + dz * cp) * sr
        z_b = sag * sr + (-ileri * sp + dz * cp) * cr
        menzil_yatay = math.hypot(x_b, y_b)
        if menzil_yatay < 5:
            return
        yukselis = math.degrees(math.atan2(z_b, menzil_yatay))
        yan = math.degrees(math.atan2(y_b, x_b))
        self.en_yakin = (yukselis, yan)
        self.ornekler.append((yukselis, yan, by, bx,
                              math.sqrt(dx * dx + dy * dy + dz * dz)))
        if len(self.ornekler) % 20 == 1:
            print(f"  n={len(self.ornekler):4d}  yukselis={yukselis:+6.2f} "
                  f"yan={yan:+6.2f}  bbox y={by:5.0f} x={bx:5.0f}  "
                  f"menzil={self.ornekler[-1][4]:6.1f} m", flush=True)

    def rapor(self):
        n = len(self.ornekler)
        print()
        if n < 10:
            print(f"YETERSIZ ORNEK ({n}). Hedef kadraja hic girmedi mi?")
            return
        yuk = np.array([o[0] for o in self.ornekler])
        yan = np.array([o[1] for o in self.ornekler])
        by = np.array([o[2] for o in self.ornekler])
        bx = np.array([o[3] for o in self.ornekler])

        print(f"=== KAMERA KALIBRASYONU ({n} ornek) ===")
        # DIKEY: by = a*yukselis + b  ->  y=360 olan yukselis = ekseni verir
        a, b = np.polyfit(yuk, by, 1)
        eksen = (360.0 - b) / a
        print(f"  dikey  : bbox_y = {a:+.2f}*yukselis + {b:.1f}")
        print(f"           olcek {abs(1/a):.5f} derece/piksel "
              f"(beklenen 12.950/720 = 0.01799)")
        print(f"           >>> KADRAJ MERKEZI = {eksen:+.2f} derece yukselis "
              f"(govdeye gore)")
        print(f"           SDF'de yazan: models/suru_drone_1/model.sdf "
              f"pose pitch")
        # YATAY: bx = c*yan + d
        c, d = np.polyfit(yan, bx, 1)
        merkez_yan = (640.0 - d) / c
        print(f"  yatay  : bbox_x = {c:+.2f}*yan + {d:.1f}")
        print(f"           olcek {abs(1/c):.5f} derece/piksel "
              f"(beklenen 22.813/1280 = 0.01782)")
        print(f"           >>> KADRAJ MERKEZI = {merkez_yan:+.2f} derece yan")
        print(f"  yukselis araligi: {yuk.min():+.1f} .. {yuk.max():+.1f} derece")
        print(f"  menzil araligi  : {min(o[4] for o in self.ornekler):.0f} .. "
              f"{max(o[4] for o in self.ornekler):.0f} m")

    tarama = None
    hedef_alt = 0.0

    def calistir(self, mav=None, tut=None):
        """tut = (lat, lon, alt) verilirse konumu SURDURUR ve burnu hedefe cevirir.

        Konum/yaw komutlari ayri bir surece birakilirsa omurleri tutmuyor
        (olcum sirasinda yaw takipcisinin suresi doldugu icin iki tarama
        bosa gitti); bu yuzden ayni dongude yapiliyor.
        """
        from pymavlink import mavutil as mu
        self.t0 = rospy.Time.now()
        son = rospy.Time.now()
        deadline = rospy.Time.now() + rospy.Duration(self.sure)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if mav is not None and self.durum is not None:
                avci, hedef = self.durum
                if tut is not None:
                    alt = tut[2]
                    if self.tarama:
                        # Irtifayi yavasca tara: kameranin ekseni nerede olursa
                        # olsun hedefin yukselisi bir noktada onu KESER.
                        gecen = (rospy.Time.now() - self.t0).to_sec()
                        lo, hi, per = self.tarama
                        faz = (gecen % per) / per
                        alt = lo + (hi - lo) * (1 - abs(2 * faz - 1))
                        self.hedef_alt = alt
                    mav.mav.set_position_target_global_int_send(
                        0, mav.target_system, mav.target_component,
                        mu.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        0b0000111111111000,
                        int(tut[0] * 1e7), int(tut[1] * 1e7), alt,
                        0, 0, 0, 0, 0, 0, 0, 0)
                yon = math.degrees(math.atan2(
                    hedef.position.y - avci.position.y,
                    hedef.position.x - avci.position.x)) % 360
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mu.mavlink.MAV_CMD_CONDITION_YAW, 0, yon, 120, 0, 0, 0, 0, 0)
                if (rospy.Time.now() - son).to_sec() > 20:
                    son = rospy.Time.now()
                    dz = hedef.position.z - avci.position.z
                    yatay = math.hypot(hedef.position.x - avci.position.x,
                                       hedef.position.y - avci.position.y)
                    print(f"  [izleme] menzil={math.hypot(yatay,dz):6.0f} m "
                          f"yukselis={math.degrees(math.atan2(dz,yatay)):+6.1f} deg "
                          f"komut_alt={self.hedef_alt:5.1f} tespit={len(self.ornekler)} kare={self.kare_sayisi} "
                          f"seq={self.son_seq}", flush=True)
            rospy.sleep(0.4)
        self.rapor()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--avci', default='iris-1')
    p.add_argument('--hedef', default='hedef')
    p.add_argument('--topic', default='/drone_1/webcam/image_raw')
    p.add_argument('--sure', type=float, default=240)
    p.add_argument('--port', type=int, default=0,
                   help='verilirse avciyi bu porttan konumda tutar + burnu hedefe cevirir')
    p.add_argument('--kuzey', type=float, default=None, help='home\'a gore kuzey (m)')
    p.add_argument('--dogu', type=float, default=None, help='home\'a gore dogu (m)')
    p.add_argument('--alt', type=float, default=None, help='irtifa (m)')
    p.add_argument('--tarama', default='', metavar='LO,HI,PERIYOT',
                   help='irtifayi LO..HI arasinda PERIYOT saniyede gidip gel')
    a = p.parse_args()
    k = Kalibrasyon(a.avci, a.hedef, a.topic, a.sure)
    if a.tarama:
        lo, hi, per = (float(x) for x in a.tarama.split(','))
        k.tarama = (lo, hi, per)
        print(f'irtifa taramasi: {lo:.0f} - {hi:.0f} m, {per:.0f} s periyot')
    mav = tut = None
    if a.port:
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(f'udpin:127.0.0.1:{a.port}', source_system=254)
        mav.wait_heartbeat()
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               b'WP_YAW_BEHAVIOR', 0,
                               mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        print(f'avci kontrolu: port {a.port}, WP_YAW_BEHAVIOR=0')
        if None not in (a.kuzey, a.dogu, a.alt):
            EARTH_R = 6378137.0
            # Public ArduPilot SITL/CMAC referansi; gercek saha koordinati degil.
            LAT0, LON0 = -35.363261, 149.165230
            lat = LAT0 + math.degrees(a.kuzey / EARTH_R)
            lon = LON0 + math.degrees(a.dogu / (EARTH_R * math.cos(math.radians(LAT0))))
            tut = (lat, lon, a.alt)
            print(f'konum tutuluyor: {a.kuzey:.0f} m kuzey, {a.dogu:.0f} m dogu, {a.alt:.0f} m')
    k.calistir(mav, tut)


if __name__ == '__main__':
    main()
