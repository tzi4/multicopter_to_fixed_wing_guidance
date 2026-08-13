#!/usr/bin/env python3
"""GERCEK DONANIM tilt komutcusu: MAVLink uzerinden ArduPilot servo-mount.

Sim'deki tools/gz_gimbal.TiltKomutcu'nun donanim karsiligi -- AYNI arayuz
(basla() / hedef(deg) / dur()), boylece TiltTakip yasasi ve bbox zinciri
hicbir degisiklik olmadan gercek gimbali surebilir.

Komut yolu (ikisi de denenir, calisan kullanilir):
  1. MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (ArduPilot 4.3+ mount suruculeri)
  2. MAV_CMD_DO_MOUNT_CONTROL (eski ama servo mount'ta yaygin calisan yol)
Aci isareti: BIZIM sozlesme pozitif = YUKARI (dunya elevasyonu). MAVLink
pitch de yukari pozitif derece bekler (ArduPilot mount boyle yorumlar).

Kendi basina test:  python3 tools/mavlink_tilt.py --baglanti tcp:127.0.0.1:5760
(once sweep yapar; SERVO_OUTPUT_RAW'daki mount kanalini izleyip raporlar)
"""

import argparse
import threading
import time


class MavlinkTiltKomutcu:
    """ArduPilot mount'a arka plan thread'inden tilt (pitch) acisi gonderir."""

    def __init__(self, baglanti, olu_bant_deg=0.2, min_aralik_s=0.1,
                 tazeleme_s=2.0, yontem='auto', sysid=1,
                 servo_slew_dps=150.0, tick_s=0.01):
        """baglanti: pymavlink adresi (or. 'tcp:127.0.0.1:5760',
        '/dev/ttyACM0', 'udpin:0.0.0.0:14550') YA DA hazir mavutil baglantisi.
        yontem: 'auto' | 'yeni' (GIMBAL_MANAGER_PITCHYAW) | 'eski'
        (DO_MOUNT_CONTROL).

        servo_slew_dps/tick_s: donanimda calisan EMAX ES08MD yoluyla ayni
        rampa. Hedefe tek buyuk PWM adimi yerine tick_s araliginda kucuk
        adimlarla gider; komut hizi fiziksel servo hizinin altinda kalir."""
        self._baglanti_arg = baglanti
        self.m = None if isinstance(baglanti, str) else baglanti
        self.olu_bant = float(olu_bant_deg)
        self.min_aralik = float(min_aralik_s)
        self.tazeleme = float(tazeleme_s)
        self.yontem = yontem
        self.sysid = sysid
        self.servo_slew = float(servo_slew_dps)
        self.tick = float(tick_s)
        self.hedef_deg = None
        self.cikis_deg = None
        self._son_tick = None
        self.yayinlanan_deg = None
        self.son_yayin_t = 0.0
        self.hata_n = 0
        self._dur = False
        self._uyandir = threading.Event()
        self._is = threading.Thread(target=self._dongu, daemon=True)

    def basla(self, ilk_hedef_deg=0.0):
        """Baglanir, komut yolunu SENKRON secer (auto ise) ve thread'i acar.

        DIKKAT: pymavlink baglantisi thread-safe DEGIL. Bu sinif yalniz
        basla() icinde recv yapar; gonderici thread SADECE gonderir. Ayni
        baglantidan baska recv yapan varsa basla()'yi ondan ONCE cagirin.
        """
        if self.m is None:
            from pymavlink import mavutil
            self.m = mavutil.mavlink_connection(self._baglanti_arg,
                                                source_system=250)
            self.m.wait_heartbeat(timeout=30)
        if self.yontem == 'auto':
            self.yontem = self._yontem_sec(float(ilk_hedef_deg))
        else:
            self._mount_configure()
        print(f"[mavlink_tilt] komut yolu: {self.yontem}", flush=True)
        self._is.start()
        return self

    def hedef(self, deg):
        self.hedef_deg = float(deg)
        self._uyandir.set()

    # ---------------------------------------------------------------- ic
    def _gonder_yeni(self, deg):
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0,
            deg,        # pitch [deg, + yukari]
            float('nan'),  # yaw: dokunma
            0, 0,       # pitch/yaw hizi: varsayilan
            0,          # bayraklar
            0, 0)       # gimbal device id, -

    def _gonder_eski(self, deg):
        # Gercek ArduPilot 4.6.3 + servo mount testinde calisan yol budur.
        # PWM 50 Hz oldugu icin gonderim de 50 Hz'i asmaz.
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
            deg,     # pitch [deg, + yukari]
            0,       # roll
            0,       # yaw
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def _ack_bekle(self, komut_id, sure=1.0):
        bitis = time.time() + sure
        while time.time() < bitis:
            a = self.m.recv_match(type='COMMAND_ACK', blocking=True,
                                  timeout=max(0.05, bitis - time.time()))
            if a is not None and a.command == komut_id:
                return a.result
        return None

    def _mount_configure(self):
        """Mount modunu MAVLINK_TARGETING'e al (DO_MOUNT_CONTROL'un
        dinlenmesi icin sart; SITL'de olculdu: configure olmadan servo
        1500'de kaliyor)."""
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            0, 0, 0, 0, 0, 0)
        self._ack_bekle(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE)

    def _yontem_sec(self, deg):
        """Calisan yolu sec (auto). OLCULMUS DERS (SITL, 2026-08-06):
        DO_GIMBAL_MANAGER_PITCHYAW ACK'leniyor ama bazi kurulumlarda servo
        KIPIRDAMIYOR -- ACK kanit degil. Evrensel calisan recete:
        DO_MOUNT_CONFIGURE(MAVLINK_TARGETING) + DO_MOUNT_CONTROL.
        'yeni' yalnizca --yontem yeni ile bilerek secilir."""
        self._mount_configure()
        self._gonder_eski(deg)
        from pymavlink import mavutil
        r = self._ack_bekle(mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL)
        if r != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.hata_n += 1
        return 'eski'

    def _dongu(self):
        while not self._dur:
            self._uyandir.wait(timeout=self.tick)
            self._uyandir.clear()
            h = self.hedef_deg
            if h is None:
                continue
            simdi = time.monotonic()
            dt = (self.tick if self._son_tick is None
                  else min(0.5, simdi - self._son_tick))
            self._son_tick = simdi
            if self.cikis_deg is None:
                self.cikis_deg = float(h)
            else:
                adim = max(-self.servo_slew * dt,
                           min(self.servo_slew * dt, h - self.cikis_deg))
                self.cikis_deg += adim
            degisti = (self.yayinlanan_deg is None
                       or abs(self.cikis_deg - self.yayinlanan_deg)
                       > self.olu_bant)
            bayat = simdi - self.son_yayin_t > self.tazeleme
            if not (degisti or bayat):
                continue
            if simdi - self.son_yayin_t < self.min_aralik:
                continue
            try:
                if self.yontem == 'yeni':
                    self._gonder_yeni(self.cikis_deg)
                else:
                    self._gonder_eski(self.cikis_deg)
                self.yayinlanan_deg = self.cikis_deg
                self.son_yayin_t = simdi
            except Exception:
                self.hata_n += 1
                time.sleep(0.5)

    def dur(self):
        self._dur = True
        self._uyandir.set()


def _servo_izle(m, kanal, sure=2.0):
    """SERVO_OUTPUT_RAW'dan verilen kanalin PWM'ini (min,maks) olarak dondurur."""
    bitis = time.time() + sure
    gor = []
    while time.time() < bitis:
        s = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.5)
        if s is not None:
            v = getattr(s, f'servo{kanal}_raw', None)
            if v:
                gor.append(int(v))
    return (min(gor), max(gor)) if gor else (None, None)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--baglanti', default='tcp:127.0.0.1:5760')
    p.add_argument('--kanal', type=int, default=9,
                   help='mount pitch servosunun cikisi (SERVOx_FUNCTION=7)')
    p.add_argument('--yontem', default='auto', choices=['auto', 'yeni', 'eski'])
    a = p.parse_args()

    k = MavlinkTiltKomutcu(a.baglanti, yontem=a.yontem).basla(ilk_hedef_deg=0.0)
    from pymavlink import mavutil
    # SERVO_OUTPUT_RAW (id 36) varsayilan akista gelmeyebilir; 10 Hz iste
    k.m.mav.command_long_send(
        k.m.target_system, k.m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        36, int(1e5), 0, 0, 0, 0, 0)
    print(f"bagli: {a.baglanti}; sweep basliyor (kanal {a.kanal} izleniyor)")
    sonuc = []
    for deg in (0.0, 30.0, -30.0, 0.0):
        k.hedef(deg)
        time.sleep(2.0)
        lo, hi = _servo_izle(k.m, a.kanal, 1.5)
        print(f"  hedef {deg:+6.1f} deg -> servo{a.kanal} PWM {lo}..{hi}")
        sonuc.append((deg, lo))
    k.dur()
    p0 = dict(sonuc)
    if None in (p0.get(30.0), p0.get(-30.0)):
        print("SONUC: SERVO_OUTPUT_RAW okunamadi (kanal dogru mu?)")
        raise SystemExit(2)
    fark = p0[30.0] - p0[-30.0]
    print(f"SONUC: +30 ile -30 arasi PWM farki {fark} "
          f"({'HAREKET VAR' if abs(fark) > 100 else 'HAREKET YOK - MNT/SERVO ayarlarini kontrol et'})")
    raise SystemExit(0 if abs(fark) > 100 else 1)


if __name__ == '__main__':
    main()
