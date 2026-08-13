#!/usr/bin/env python3
"""
suru_komut.py - suru + hedef ucak icin elle komut araci
=======================================================
Port sozlesmesi yildizlar_gudum.sh ve arkadas_scripts/config.py ile aynidir:
    drone N (1..5) -> udp:127.0.0.1:{14551 + 10*(N-1)}   SysID N
    hedef ucak     -> udp:127.0.0.1:14601                SysID 6

Alt komutlar:
    durum                      tum araclarin mod/arm/konum/irtifa ozeti
    hedef-kalkis               hedef ucagi ARM edip AUTO'ya alir (plan zaten yuklu)
    drone-kalkis --id N        kopteri GUIDED'da ARM edip --alt metreye cikarir
    takip --id N               kopteri hedefin arkasinda tutar + burnunu hedefe cevirir

NOT (WSL2): her UDP portunu YALNIZ BIR surec baglayabilir. Bu arac calisirken
ayni portu dinleyen baska bir arac (or. ground_station.py) calistirilmamali.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

DRONE_PORT_BASE = 14551
TARGET_PORT = 14601
TARGET_SYSID = 6

# ArduCopter / ArduPlane custom mode numaralari
COPTER_GUIDED = 4
COPTER_LOITER = 5
PLANE_AUTO = 10
PLANE_TAKEOFF = 13

EARTH_R = 6378137.0


def baglan(port, sysid, timeout=30):
    """Verilen porta baglanip BEKLENEN SysID'den heartbeat gelmesini bekler.

    SysID dogrulamasi sart: 14550'de tum araclar bir arada oldugu icin yanlis
    porta baglanildiginda sessizce baska bir araca komut gitmesi mumkun.
    """
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}',
                                        source_system=255, source_component=190)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and msg.get_srcSystem() == sysid:
            master.target_system = sysid
            master.target_component = msg.get_srcComponent()
            akis_hizi_iste(master)
            # Baglanis anindaki birikimi at: olculdu, ilk okumalarda 2-5 s
            # bayat veri geliyordu, tampon bosalinca gecikme 0.02 s'ye dusuyor.
            son_temizlik = time.monotonic() + 1.0
            while time.monotonic() < son_temizlik:
                if master.recv_match(blocking=False) is None:
                    break
            return master
    master.close()
    raise SystemExit(f"port {port}: SysID {sysid} heartbeat alinamadi ({timeout}s)")


def drone_baglan(drone_id, timeout=30):
    if not 1 <= drone_id <= 5:
        raise SystemExit("drone id 1..5 olmali")
    return baglan(DRONE_PORT_BASE + 10 * (drone_id - 1), drone_id, timeout)


def komut(master, command, *params, bekle=True):
    """COMMAND_LONG gonderir; bekle=True ise ACK'i okuyup dondurur."""
    params = list(params) + [0] * (7 - len(params))
    master.mav.command_long_send(master.target_system, master.target_component,
                                 command, 0, *params[:7])
    if not bekle:
        return None
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack is not None and ack.command == command:
            return ack.result
    return None


def mod_ayarla(master, custom_mode):
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             custom_mode)


def mod_bekle(master, custom_mode, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is not None and msg.get_srcSystem() == master.target_system:
            if msg.custom_mode == custom_mode:
                return True
            mod_ayarla(master, custom_mode)
    return False


def akis_hizi_iste(master, hz=10.0):
    """GLOBAL_POSITION_INT + ATTITUDE akis hizini yukselt (SET_MESSAGE_INTERVAL).

    MAVProxy varsayilaninda GLOBAL_POSITION_INT ~1.5 Hz geliyor; 20 m/s'lik
    bir hedefte iki ornek arasi 13 m demek. Gudum donguleri icin yetersiz.
    """
    aralik_us = int(1e6 / hz)
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                   mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
        komut(master, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
              msg_id, aralik_us, bekle=False)


def konum_al(master, timeout=5):
    """(lat, lon, rel_alt_m, hdg_deg) - GLOBAL_POSITION_INT'ten, EN TAZE ornek.

    DIKKAT - burada bir hata vardi ve olculdu (2026-08-01): tek bir
    recv_match() cagrisi UDP tamponundaki EN ESKI mesaji dondurur. Dongu
    basina bir okuma yapilinca tampon doluyor ve okunan veri bayatliyordu:
      6 s bekleyip tek okuma -> tamponda 9 mesaj birikmis, ILK okunan
      ~6 s eski. 20 m/s'lik hedefte 120 m konum hatasi.
    Cozum: tamponu SONUNA KADAR bosalt, en son gelen ornegi kullan.
    (guidance_allstar/simple_guided_follow.py bundan etkilenmiyordu; o
    surekli okuyan bir thread - mavlink_utils.MavStateReader - kullaniyor.)
    """
    son = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT',
                                blocking=(son is None), timeout=1)
        if msg is None:
            if son is not None:
                break
            continue
        if msg.get_srcSystem() == master.target_system:
            son = msg
    if son is None:
        return None
    return (son.lat / 1e7, son.lon / 1e7, son.relative_alt / 1000.0,
            son.hdg / 100.0)


def prearm_bekle(master, timeout=120):
    """EKF/GPS hazir olana kadar bekler (SYS_STATUS + GPS_FIX)."""
    deadline = time.monotonic() + timeout
    son = ''
    while time.monotonic() < deadline:
        msg = master.recv_match(type=['GPS_RAW_INT', 'STATUSTEXT'], blocking=True,
                                timeout=1)
        if msg is None:
            continue
        if msg.get_srcSystem() != master.target_system:
            continue
        if msg.get_type() == 'STATUSTEXT':
            metin = msg.text.strip()
            if metin != son:
                print(f"  [FCU] {metin}", flush=True)
                son = metin
        elif msg.fix_type >= 3 and msg.satellites_visible >= 6:
            return True
    return False


def arm_et(master, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sonuc = komut(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if sonuc == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if hb is not None and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return True
            return True
        print(f"  ARM reddedildi (sonuc={sonuc}), tekrar deneniyor...", flush=True)
        time.sleep(2)
    return False


def hiz_ayarla(master, hiz_ms):
    """GUIDED yatay hiz tavanini CALISMA ANINDA yukselt (DO_CHANGE_SPEED).

    NEDEN: WPNAV_SPEED parametresinin ust siniri 2000 cm/s = 20 m/s'tir, yani
    parametre dosyasindan 20 m/s'in ustune CIKILAMAZ. Ama GUIDED'da
    MAV_CMD_DO_CHANGE_SPEED (tip 1 = yer hizi) dogrudan
    ModeGuided::set_speed_xy_cms() -> AC_PosControl::set_max_speed_accel_NE_cm()
    cagirir ve bu yolda WPNAV_SPEED'e KIRPMA YOKTUR
    (ArduCopter/GCS_MAVLink_Copter.cpp:690, mode_guided.cpp:289).

    DIKKAT: deger GUIDED'a HER girildiginde sifirlanir (pva_control_start
    tekrar WPNAV_SPEED yazar). Gudum kodu GUIDED'a gectikten SONRA bunu
    yeniden gondermeli.
    """
    return komut(master, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                 1,               # param1: 1 = yer hizi
                 hiz_ms,          # param2: m/s
                 -1, 0)           # param3: gaz (-1 = degistirme)


def nokta_git(master, lat, lon, rel_alt):
    """GUIDED'da mutlak konum hedefi (yalniz pozisyon bitleri acik)."""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                      # yalniz x,y,z kullan
        int(lat * 1e7), int(lon * 1e7), rel_alt,
        0, 0, 0, 0, 0, 0, 0, 0)


def yaw_ayarla(master, heading_deg, hiz_dps=90):
    """CONDITION_YAW: mutlak yon (param4=0), en kisa yoldan (param3=0).

    hiz_dps 30 (ilk deger) yakin gecislerde YETMIYORDU: 120 m'den gecen
    20 m/s hedefin kerteriz hizi ~9.5 derece/s'e ciksa da kopter komutlar
    arasi 30 derece/s ile donerken geride kaliyor ve hedef kadrajin SAG
    kenarinda kaliyordu (olcum: 17 tespitin hepsi x>500/640). 90 derece/s
    ile donus kerterizi yakaliyor.
    """
    komut(master, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
          heading_deg % 360.0, hiz_dps, 0, 0, bekle=False)


def mesafe_yon(lat1, lon1, lat2, lon2):
    """(mesafe_m, bearing_deg) - kucuk mesafelerde duz yaklasim yeterli."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    kuzey = dlat * EARTH_R
    dogu = dlon * EARTH_R * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(kuzey, dogu), math.degrees(math.atan2(dogu, kuzey)) % 360


def kaydir(lat, lon, kuzey_m, dogu_m):
    lat2 = lat + math.degrees(kuzey_m / EARTH_R)
    lon2 = lon + math.degrees(dogu_m / (EARTH_R * math.cos(math.radians(lat))))
    return lat2, lon2


# ------------------------------------------------------------------ komutlar

def cmd_durum(args):
    hedefler = [(DRONE_PORT_BASE + 10 * i, i + 1, f"drone_{i + 1}")
                for i in range(args.drones)]
    hedefler.append((TARGET_PORT, TARGET_SYSID, "hedef_ucak"))
    for port, sysid, ad in hedefler:
        try:
            master = baglan(port, sysid, timeout=8)
        except SystemExit as exc:
            print(f"{ad:12s} port {port}: {exc}")
            continue
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        konum = konum_al(master)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        mod = hb.custom_mode if hb else -1
        if konum:
            print(f"{ad:12s} port={port} sysid={sysid} mod={mod} "
                  f"arm={'E' if armed else 'H'} "
                  f"lat={konum[0]:.6f} lon={konum[1]:.6f} alt={konum[2]:.1f}m "
                  f"hdg={konum[3]:.0f}")
        else:
            print(f"{ad:12s} port={port} sysid={sysid} mod={mod} "
                  f"arm={'E' if armed else 'H'} (konum yok)")
        master.close()


def cmd_hedef_kalkis(args):
    master = baglan(TARGET_PORT, TARGET_SYSID)
    print("Hedef ucak baglandi. GPS/EKF bekleniyor...", flush=True)
    if not prearm_bekle(master):
        raise SystemExit("hedef ucak GPS fix alamadi")
    print("AUTO moduna aliniyor...", flush=True)
    mod_ayarla(master, PLANE_AUTO)
    if not mod_bekle(master, PLANE_AUTO):
        raise SystemExit("hedef ucak AUTO moduna gecmedi")
    print("ARM ediliyor...", flush=True)
    if not arm_et(master):
        raise SystemExit("hedef ucak ARM edilemedi")
    # AUTO'da ARM sonrasi ArduPlane kalkis oğesini kendi baslatir; bazi
    # surumlerde ilk tetik icin MISSION_START gerekiyor.
    komut(master, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    print("Kalkis izleniyor...", flush=True)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        konum = konum_al(master)
        if konum is None:
            continue
        print(f"  irtifa={konum[2]:6.1f} m  lat={konum[0]:.6f} lon={konum[1]:.6f}",
              flush=True)
        if konum[2] >= args.alt:
            print(f"Hedef ucak {konum[2]:.1f} m'de, gorev devam ediyor.")
            master.close()
            return
        time.sleep(2)
    master.close()
    raise SystemExit(f"hedef ucak {args.alt} m'ye {args.timeout}s icinde cikmadi")


def cmd_drone_kalkis(args):
    master = drone_baglan(args.id)
    print(f"drone_{args.id} baglandi. GPS/EKF bekleniyor...", flush=True)
    if not prearm_bekle(master):
        raise SystemExit("GPS fix alinamadi")
    print("GUIDED moduna aliniyor...", flush=True)
    mod_ayarla(master, COPTER_GUIDED)
    if not mod_bekle(master, COPTER_GUIDED):
        raise SystemExit("GUIDED moduna gecmedi")
    print("ARM ediliyor...", flush=True)
    if not arm_et(master):
        raise SystemExit("ARM edilemedi")
    print(f"{args.alt} m'ye kalkis...", flush=True)
    komut(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        konum = konum_al(master)
        if konum is None:
            continue
        print(f"  irtifa={konum[2]:6.1f} m", flush=True)
        if konum[2] >= args.alt * 0.95:
            print(f"drone_{args.id} {konum[2]:.1f} m'de hazir.")
            master.close()
            return
        time.sleep(1)
    master.close()
    raise SystemExit(f"drone {args.alt} m'ye {args.timeout}s icinde cikmadi")


def cmd_takip(args):
    """Kopteri hedefin GERISINDE tutar ve burnunu hedefe cevirir.

    Amac carpmak DEGIL: hedefi kadraja sokup bbox akisini beslemek. Bu yuzden
    komut noktasi hedefin arkasina --geri metre, altina/ustune --dikey metre
    kaydirilmis bir noktadir; koptere hedefin KENDISI verilmez.
    """
    hedef = baglan(TARGET_PORT, TARGET_SYSID)
    drone = drone_baglan(args.id)
    print(f"takip basladi: drone_{args.id} <- hedef ucak "
          f"(geri={args.geri} m, dikey={args.dikey:+.0f} m)", flush=True)

    son_yaw = 0.0
    deadline = time.monotonic() + args.sure
    while time.monotonic() < deadline:
        h = konum_al(hedef, timeout=3)
        d = konum_al(drone, timeout=3)
        if h is None or d is None:
            print("  telemetri bekleniyor...", flush=True)
            continue

        h_lat, h_lon, h_alt, h_hdg = h
        d_lat, d_lon, d_alt, _ = d

        # Hedefin gidis yonunun TERSINE 'geri' metre: arkasindan takip.
        rad = math.radians(h_hdg)
        git_lat, git_lon = kaydir(h_lat, h_lon,
                                  -args.geri * math.cos(rad),
                                  -args.geri * math.sin(rad))
        git_alt = max(5.0, h_alt + args.dikey)
        nokta_git(drone, git_lat, git_lon, git_alt)

        mesafe, yon = mesafe_yon(d_lat, d_lon, h_lat, h_lon)
        # Yaw komutu her karede degil, 3 dereceden fazla saptiginda: surekli
        # CONDITION_YAW gondermek kopterin donusunu resetleyip titretiyor.
        if abs((yon - son_yaw + 180) % 360 - 180) > 3:
            yaw_ayarla(drone, yon)
            son_yaw = yon
        print(f"  hedef alt={h_alt:5.1f} hdg={h_hdg:3.0f} | drone alt={d_alt:5.1f} "
              f"| mesafe={mesafe:7.1f} m yon={yon:3.0f}", flush=True)
        time.sleep(args.periyot)

    hedef.close()
    drone.close()
    print("takip suresi doldu.")


def cmd_pusu(args):
    """Kopteri hedefin ROTASI UZERINDE bir noktaya park eder ve burnunu
    surekli hedefe cevirir.

    NEDEN TAKIP DEGIL PUSU: kopterin tavani WPNAV_SPEED 18 m/s, hedef ucak
    20 m/s seyirde -> kovalayan kopter mesafeyi HIC kapatamiyor (olcum:
    takip modunda mesafe 450-620 m bandinda kaldi). Pusuda kopter hedefin
    rotasi uzerinde bekler, yani kapanmayi HIZ degil GEOMETRI saglar.

    [GIMBAL DALI 2026-08-05 - GEREKCE DUZELTMESI] Burada eskiden ikinci bir
    gerekce yaziliydi: "tam gazda kopter ~20 derece one yatiyor, kamera sabit
    ve ileri baktigi icin hedef dikey kadrajin ust kenarina kaciyor". Bu
    ARTIK GECERSIZ: kamera kendini stabilize eden fiziksel tilt gimbalinde,
    govde +-35 deg savrulurken kamera dunya pitch'i max 0.65 deg olculdu
    (NOTLAR_GIMBAL.md). Yani pusunun DIKEY kadraj avantaji kalmadi.
    Ayakta kalan gerekceler: (a) hiz farki (asil sebep), (b) asili kopterde
    yaw savrulmasi yok -- gimbal TEK EKSEN oldugu icin yatay kadraj hala
    airframe yaw'ina bagli, (c) roll sifir (tek eksen gimbal roll'u telafi
    etmez, goruntuye yansir).

    Konum home'a gore kuzey/dogu metre olarak verilir; home = araclarin
    kalkis noktasi (Gazebo dunya orijini).
    """
    drone = drone_baglan(args.id)
    hedef = baglan(TARGET_PORT, TARGET_SYSID)

    ilk = konum_al(drone, timeout=10)
    if ilk is None:
        raise SystemExit("drone konumu okunamadi")
    # Home: kalkis noktasi degil, o anki konumdan geri hesap yapilmaz; --kuzey
    # /--dogu HOME'a goredir, bu yuzden home'u FCU'dan istiyoruz.
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                0, 0, 0, 0, 0, 0, 0, 0)
    home_msg = None
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        home_msg = drone.recv_match(type='HOME_POSITION', blocking=True, timeout=2)
        if home_msg is not None and home_msg.get_srcSystem() == drone.target_system:
            break
    if home_msg is None:
        raise SystemExit("HOME_POSITION alinamadi")
    h_lat, h_lon = home_msg.latitude / 1e7, home_msg.longitude / 1e7

    pusu_lat, pusu_lon = kaydir(h_lat, h_lon, args.kuzey, args.dogu)
    print(f"pusu noktasi: lat={pusu_lat:.6f} lon={pusu_lon:.6f} alt={args.alt} m "
          f"(home'dan {args.kuzey:+.0f} m kuzey, {args.dogu:+.0f} m dogu)", flush=True)

    nokta_git(drone, pusu_lat, pusu_lon, args.alt)
    varis_deadline = time.monotonic() + args.varis_timeout
    while time.monotonic() < varis_deadline:
        d = konum_al(drone, timeout=3)
        if d is None:
            continue
        kalan, _ = mesafe_yon(d[0], d[1], pusu_lat, pusu_lon)
        print(f"  pusu noktasina {kalan:6.1f} m, irtifa={d[2]:5.1f} m", flush=True)
        if kalan < 15 and abs(d[2] - args.alt) < 5:
            break
        time.sleep(2)
    else:
        print("UYARI: pusu noktasina varilamadi, yine de beklemeye gecliyor.")

    print("pusuda: burun hedefe cevriliyor, konum komutu ARTIK GONDERILMIYOR "
          "(yeni konum hedefi CONDITION_YAW'i sifirlar).", flush=True)
    son_yaw = None
    en_yakin = float('inf')
    deadline = time.monotonic() + args.sure
    while time.monotonic() < deadline:
        h = konum_al(hedef, timeout=3)
        d = konum_al(drone, timeout=3)
        if h is None or d is None:
            continue
        mesafe, yon = mesafe_yon(d[0], d[1], h[0], h[1])
        en_yakin = min(en_yakin, mesafe)
        if son_yaw is None or abs((yon - son_yaw + 180) % 360 - 180) > 1.5:
            yaw_ayarla(drone, yon)
            son_yaw = yon
        print(f"  hedef mesafe={mesafe:7.1f} m yon={yon:3.0f} "
              f"hedef_alt={h[2]:5.1f} | en_yakin={en_yakin:7.1f} m", flush=True)
        time.sleep(args.periyot)

    print(f"pusu bitti. En yakin gecis: {en_yakin:.1f} m")
    drone.close()
    hedef.close()


def cmd_hiz_testi(args):
    """Kopterin GERCEKTE ulastigi yatay hizi ve irtifa tutmasini olcer.

    Parametre tavani ile FIZIKSEL tavan ayni sey degil: WPNAV_SPEED 2000
    yazmak 20 m/s'i garanti etmez, kopterin itki/agirlik orani ve ANGLE_MAX
    izin verdigi kadarina ulasilir. Bu komut uzak bir noktaya gidip
    GLOBAL_POSITION_INT'ten yer hizini ve irtifa sapmasini raporlar.
    """
    drone = drone_baglan(args.id)
    d = konum_al(drone, timeout=10)
    if d is None:
        raise SystemExit("drone konumu okunamadi")
    if d[2] < 5:
        raise SystemExit("once 'drone-kalkis' ile havalandirin")

    baslangic_alt = d[2]
    git_lat, git_lon = kaydir(d[0], d[1], args.mesafe, 0)
    if args.hiz > 0:
        sonuc = hiz_ayarla(drone, args.hiz)
        print(f"DO_CHANGE_SPEED {args.hiz} m/s -> sonuc={sonuc} "
              f"({'kabul' if sonuc == 0 else 'RED'})", flush=True)
    print(f"{args.mesafe:.0f} m kuzeye tam gaz kosu, irtifa {baslangic_alt:.1f} m",
          flush=True)
    nokta_git(drone, git_lat, git_lon, baslangic_alt)

    en_hizli = 0.0
    en_dusuk_alt = baslangic_alt
    en_cok_yatma = 0.0
    ornek = 0
    deadline = time.monotonic() + args.sure
    while time.monotonic() < deadline:
        msg = drone.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'],
                               blocking=True, timeout=2)
        if msg is None or msg.get_srcSystem() != drone.target_system:
            continue
        if msg.get_type() == 'ATTITUDE':
            yatma = math.degrees(math.hypot(msg.roll, msg.pitch))
            en_cok_yatma = max(en_cok_yatma, yatma)
            continue
        hiz = math.hypot(msg.vx, msg.vy) / 100.0
        alt = msg.relative_alt / 1000.0
        en_hizli = max(en_hizli, hiz)
        en_dusuk_alt = min(en_dusuk_alt, alt)
        ornek += 1
        if ornek % 5 == 0:
            print(f"  hiz={hiz:5.2f} m/s  irtifa={alt:6.2f} m  "
                  f"yatma={en_cok_yatma:4.1f} deg", flush=True)

    print(f"\nSONUC: en yuksek yer hizi = {en_hizli:.2f} m/s")
    print(f"       en cok yatma        = {en_cok_yatma:.1f} derece")
    print(f"       irtifa {baslangic_alt:.1f} -> en dusuk {en_dusuk_alt:.1f} m "
          f"(kayip {baslangic_alt - en_dusuk_alt:.1f} m)")
    drone.close()


def cmd_hiz_kilidi(args):
    """GUIDED hiz tavanini periyodik olarak yeniden dayatan yan surec.

    NEDEN AYRI SUREC: guidance_allstar/simple_guided_follow.py hic
    DO_CHANGE_SPEED gondermiyor (repoda gectigi tek yer yok), dolayisiyla avci
    WPNAV_SPEED tavaninda = 20 m/s kaliyor. Hedef ucak da 20 m/s seyirde
    oldugu icin duz bacakta mesafe HIC kapanmiyor; kapanma yalnizca ucak
    donerken kestirme atmaktan geliyor. Olculdu: ayni iris 38 m/s'i irtifa
    kaybetmeden tutuyor (tools/suru_komut.py hiz-testi).

    NEDEN PERIYODIK: deger GUIDED'a her girildiginde WPNAV_SPEED'e sifirlanir;
    miss-recovery makinesi BRAKE'e dusup GUIDED'a geri dondugunde tavan da
    geri duser. Bu yuzden komut --periyot saniyede bir tazelenir.

    Gudum kodunun kendisine DOKUNULMAZ: bu yalnizca bir COMMAND_LONG, setpoint
    yolu ile yarismaz.
    """
    drone = drone_baglan(args.id)
    print(f"hiz kilidi: drone_{args.id} -> {args.hiz} m/s, her {args.periyot} s'de "
          f"yenileniyor (Ctrl+C ile cik)", flush=True)
    son_sonuc = None
    deadline = time.monotonic() + args.sure
    while time.monotonic() < deadline:
        sonuc = hiz_ayarla(drone, args.hiz)
        if sonuc != son_sonuc:
            print(f"  DO_CHANGE_SPEED sonuc={sonuc} "
                  f"({'kabul' if sonuc == 0 else 'RED/yanit yok'})", flush=True)
            son_sonuc = sonuc
        time.sleep(args.periyot)
    drone.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='komut', required=True)

    p = sub.add_parser('durum', help='tum araclarin ozeti')
    p.add_argument('--drones', type=int, default=5)
    p.set_defaults(func=cmd_durum)

    p = sub.add_parser('hedef-kalkis', help='hedef ucagi AUTO ile kaldir')
    p.add_argument('--alt', type=float, default=40, help='dogrulanacak irtifa (m)')
    p.add_argument('--timeout', type=float, default=180)
    p.set_defaults(func=cmd_hedef_kalkis)

    p = sub.add_parser('drone-kalkis', help='kopteri GUIDED ile kaldir')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--alt', type=float, default=50)
    p.add_argument('--timeout', type=float, default=120)
    p.set_defaults(func=cmd_drone_kalkis)

    p = sub.add_parser('takip', help='kopteri hedefin arkasinda tut')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--geri', type=float, default=120, help='hedefin arkasinda kalinacak mesafe (m)')
    p.add_argument('--dikey', type=float, default=-10, help='hedefe gore dikey ofset (m)')
    p.add_argument('--sure', type=float, default=300)
    p.add_argument('--periyot', type=float, default=1.0)
    p.set_defaults(func=cmd_takip)

    p = sub.add_parser('pusu', help='kopteri hedefin rotasi uzerine park et')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--kuzey', type=float, default=500, help='home\'a gore kuzey (m)')
    p.add_argument('--dogu', type=float, default=120, help='home\'a gore dogu (m)')
    p.add_argument('--alt', type=float, default=62, help='pusu irtifasi (m)')
    p.add_argument('--sure', type=float, default=300)
    p.add_argument('--periyot', type=float, default=0.5)
    p.add_argument('--varis-timeout', type=float, default=180)
    p.set_defaults(func=cmd_pusu)

    p = sub.add_parser('hiz-testi', help='ulasilan gercek hizi olc')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--mesafe', type=float, default=1500, help='kosu mesafesi (m)')
    p.add_argument('--hiz', type=float, default=0,
                   help='DO_CHANGE_SPEED ile istenen yer hizi (m/s); 0 = gonderme')
    p.add_argument('--sure', type=float, default=90)
    p.set_defaults(func=cmd_hiz_testi)

    p = sub.add_parser('hiz-kilidi', help='GUIDED hiz tavanini surekli dayat')
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--hiz', type=float, default=35, help='istenen yer hizi (m/s)')
    p.add_argument('--periyot', type=float, default=2.0)
    p.add_argument('--sure', type=float, default=100000)
    p.set_defaults(func=cmd_hiz_kilidi)

    args = parser.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        print("\nkesildi.", file=sys.stderr)


if __name__ == '__main__':
    main()
