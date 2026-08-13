#!/usr/bin/env python3
"""
Görsel Servo Kontrolcü — Çift Modlu (Konumlu + Görüntülü)
==========================================================
- KONUMLU mod: Hedef uçağın MAVLink telemetrisinden GPS takibi
- GÖRÜNTÜLÜ mod: Redis bbox verisinden PID visual servo

Kullanım:
    python3 visual_servo_controller.py [--conn udp:127.0.0.1:14551] [--target udp:127.0.0.1:14561]
"""

import time
import math
import csv
import argparse
import ast
import threading
import queue
import numpy as np
from datetime import datetime
from pymavlink import mavutil

# ═══════════════════════════════════════════════════════════════
# SANAL GİMBAL
# ═══════════════════════════════════════════════════════════════
R_c_b   = np.array([[0,0,1],[1,0,0],[0,1,0]], dtype=float)
R_c_b_T = R_c_b.T

def compute_R_b_e(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])

# ═══════════════════════════════════════════════════════════════
# CSV LOGGER
# ═══════════════════════════════════════════════════════════════
class FlightLogger:
    def __init__(self):
        fn = f"vs_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._f = open(fn, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow([
            "t","elapsed","dt","gorev_mode","state","flight_mode",
            "bbox_cx","bbox_cy","bbox_w","bbox_h","area",
            "stab_cx","stab_cy",
            "err_x_deg","err_y_deg",
            "vx_cmd","vy_cmd","vz_cmd",
            "drone_lat","drone_lon","drone_alt",
            "target_lat","target_lon","target_alt",
            "alt_m","roll_deg","pitch_deg","yaw_deg",
            "data_age_ms"
        ])
        self._f.flush()
        self._cnt = 0
        print(f"[LOG] {fn}")

    def log(self, row):
        self._w.writerow(row)
        self._cnt += 1
        if self._cnt % 10 == 0:
            self._f.flush()

    def close(self):
        self._f.flush(); self._f.close()

# ═══════════════════════════════════════════════════════════════
# REDIS DİNLEYİCİ
# ═══════════════════════════════════════════════════════════════
class RedisListener(threading.Thread):
    def __init__(self, data_queue, host='localhost', port=6379):
        super().__init__(daemon=True)
        self.q = data_queue
        self.running = True
        import redis
        self.r = redis.Redis(host=host, port=port, db=0)
        self.ps = self.r.pubsub()
        self.ps.subscribe('tracker_bbox')
        self.komut_yetkisi = 'konumlu'
        print("[REDIS] 'tracker_bbox' kanalına abone olundu.")

    def get_komut_yetkisi(self):
        try:
            val = self.r.get('komut_yetkisi')
            if val:
                self.komut_yetkisi = val.decode('utf-8')
        except Exception:
            pass
        return self.komut_yetkisi

    def run(self):
        while self.running:
            try:
                msg = self.ps.get_message(ignore_subscribe_messages=True, timeout=0.01)
                if msg and msg['type'] == 'message':
                    data = ast.literal_eval(msg['data'].decode('utf-8'))
                    if len(data) >= 6 and data[5] == 1:
                        x,y,w,h = data[0], data[1], data[2], data[3]
                        self.q.put({
                            'x': int(x), 'y': int(y),
                            'w': float(w), 'h': float(h),
                            'cx': x + w/2.0, 'cy': y + h/2.0,
                            'area': float(w*h),
                            'ts': data[6] if len(data)>6 else time.time()
                        })
            except Exception:
                time.sleep(0.05)

# ═══════════════════════════════════════════════════════════════
# DRONE MAVLINK YÖNETİCİ
# ═══════════════════════════════════════════════════════════════
class MavManager(threading.Thread):
    def __init__(self, conn_str):
        super().__init__(daemon=True)
        self.running = True
        self.lock = threading.Lock()
        self.mode = "UNKNOWN"
        self.yaw = 0.0; self.roll = 0.0; self.pitch = 0.0
        self.alt = 50.0
        self.lat = 0.0; self.lon = 0.0
        print(f"[MAV-DRONE] Bağlanıyor: {conn_str}")
        self.m = mavutil.mavlink_connection(conn_str)
        self.m.wait_heartbeat(timeout=20)
        print(f"[MAV-DRONE] Heartbeat OK  sys:{self.m.target_system}")
        for stream in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                       mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
            self.m.mav.request_data_stream_send(
                self.m.target_system, self.m.target_component, stream, 10, 1)

    def run(self):
        while self.running:
            try:
                with self.lock:
                    msg = self.m.recv_match(
                        type=['ATTITUDE','HEARTBEAT','GLOBAL_POSITION_INT'],
                        blocking=False)
                if msg:
                    t = msg.get_type()
                    if t == 'ATTITUDE':
                        self.roll  = msg.roll
                        self.pitch = msg.pitch
                        self.yaw   = msg.yaw
                    elif t == 'HEARTBEAT' and self.m.flightmode:
                        self.mode = self.m.flightmode
                    elif t == 'GLOBAL_POSITION_INT':
                        self.alt = msg.relative_alt / 1000.0
                        self.lat = msg.lat / 1e7
                        self.lon = msg.lon / 1e7
                time.sleep(0.005)
            except Exception:
                time.sleep(0.1)

    def send_velocity(self, vx, vy, vz):
        """Body-frame hız komutu"""
        with self.lock:
            self.m.mav.set_position_target_local_ned_send(
                0, self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,
                0,0,0, vx, vy, vz, 0,0,0, 0, 0)

    def send_gps_target(self, lat, lon, alt, yaw=None):
        """GPS konumuna git, isteğe bağlı yaw açısı ile"""
        with self.lock:
            if yaw is not None:
                # Bit 10 = 0 → yaw KULLAN
                # 0b0000_10_1_111_111_000 = 0x0BF8
                typemask = 0b0000101111111000
                yaw_rad = yaw
            else:
                typemask = 0b0000111111111000
                yaw_rad = 0
            self.m.mav.set_position_target_global_int_send(
                0, self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                typemask,
                int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0, 0, 0, 0, yaw_rad, 0)

# ═══════════════════════════════════════════════════════════════
# HEDEF UÇAK TELEMETRİ DİNLEYİCİ
# ═══════════════════════════════════════════════════════════════
class TargetTracker(threading.Thread):
    """Hedef uçağın MAVLink telemetrisini dinler (GPS pozisyonu)"""
    def __init__(self, conn_str):
        super().__init__(daemon=True)
        self.running = True
        self.lat = 0.0; self.lon = 0.0; self.alt = 0.0
        self.last_update = 0.0
        print(f"[MAV-HEDEF] Bağlanıyor: {conn_str}")
        self.m = mavutil.mavlink_connection(conn_str)
        self.m.wait_heartbeat(timeout=20)
        print(f"[MAV-HEDEF] Heartbeat OK  sys:{self.m.target_system}")
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)

    def run(self):
        while self.running:
            try:
                msg = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
                    self.alt = msg.relative_alt / 1000.0
                    self.last_update = time.time()
                time.sleep(0.005)
            except Exception:
                time.sleep(0.1)

    @property
    def has_fix(self):
        return self.lat != 0.0 and (time.time() - self.last_update) < 5.0

# ═══════════════════════════════════════════════════════════════
# ANA KONTROLCÜ
# ═══════════════════════════════════════════════════════════════
class VisualServoAttackController:
    # ── Kamera parametreleri ──
    IMG_W = 640; IMG_H = 480
    CX = 320; CY = 240
    FX = 467.7; FY = 467.7
    HFOV = 1.2

    # ── PID — Yatay (vy) ──
    KP_Y = 0.06; KI_Y = 0.001; KD_Y = 0.02
    # ── PID — Dikey (vz) ──
    KP_Z = 0.03; KI_Z = 0.0008; KD_Z = 0.025

    # ── İleri hız (vx) ──
    VX_BASE = 17.0; VX_MAX = 20.0; VX_MIN = 12.0
    VX_AREA_GAIN = 0.0001; VX_ERR_THRESH = 12.0

    # ── Limitler ──
    VY_MAX = 8.0; VZ_MAX = 6.0
    INTEGRAL_MAX = 50.0; DEADZONE_DEG = 0.3

    # ── Zamanlama ──
    CMD_INTERVAL = 0.05; COAST_TIMEOUT = 2.0; FAILSAFE_TIMEOUT = 4.0

    # ── Konumlu güdüm ──
    KONUMLU_CMD_INTERVAL = 0.2  # 5Hz GPS komut gönderim
    LEAD_DISTANCE = 50.0        # metre — waypoint'i hedefin bu kadar ilerisine koy

    # ── Türev filtre ──
    DERIV_ALPHA = 0.08

    # ── Çıkış yumuşatma (EMA) ──
    # 0.0 = tamamen eski değer (değişmez), 1.0 = filtre yok (ham PID)
    # 0.3 = %30 yeni + %70 eski → yumuşak geçiş
    OUTPUT_SMOOTH = 0.2

    # ── Konumlu irtifa yumuşatma ──
    ALT_SMOOTH = 0.15   # hedef irtifası EMA katsayısı
    GPS_SMOOTH = 0.25    # lat/lon EMA katsayısı (titreşim önleme)

    def __init__(self, conn_str, target_conn_str):
        self.dq = queue.Queue()
        self.logger = FlightLogger()
        self.mav    = MavManager(conn_str)
        self.redis  = RedisListener(self.dq)
        self.target = TargetTracker(target_conn_str)
        self.mav.start()
        self.redis.start()
        self.target.start()

        # PID iç durumlar
        self.prev_err_x = 0.0; self.prev_err_y = 0.0
        self.int_err_x  = 0.0; self.int_err_y  = 0.0
        self.prev_der_x = 0.0; self.prev_der_y = 0.0
        self.last_t = time.time()

        # Çıkış yumuşatma state
        self.smooth_vy = 0.0
        self.smooth_vz = 0.0

        # Konumlu GPS yumuşatma
        self.smooth_alt = None
        self.smooth_lat = None
        self.smooth_lon = None

        # Coasting
        self.last_vx = 0.0; self.last_vy = 0.0; self.last_vz = 0.0
        self.last_target_t = 0.0
        self.last_cmd_t    = 0.0

        self.start_t = time.time()
        self.K     = np.array([[self.FX,0,self.CX],[0,self.FY,self.CY],[0,0,1]])
        self.K_inv = np.linalg.inv(self.K)

    def _stabilize(self, px, py):
        p = self.K_inv @ np.array([px, py, 1.0])
        b = R_c_b @ p
        R = compute_R_b_e(self.mav.roll, self.mav.pitch, 0)
        vb = R @ b; vc = R_c_b_T @ vb; h = self.K @ vc
        if h[2] != 0:
            return h[0]/h[2], h[1]/h[2]
        return px, py

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _reset_pid(self):
        self.prev_err_x = self.prev_err_y = 0.0
        self.int_err_x  = self.int_err_y  = 0.0
        self.prev_der_x = self.prev_der_y = 0.0
        self.smooth_vy  = 0.0
        self.smooth_vz  = 0.0
        self.smooth_alt  = None

    def _pid_step(self, stab_cx, stab_cy, area, dt):
        err_x_deg = math.degrees(math.atan((stab_cx - self.CX) / self.FX))
        err_y_deg = math.degrees(math.atan((stab_cy - self.CY) / self.FY))

        px = 0.0 if abs(err_x_deg) < self.DEADZONE_DEG else \
             err_x_deg - math.copysign(self.DEADZONE_DEG, err_x_deg)
        py = 0.0 if abs(err_y_deg) < self.DEADZONE_DEG else \
             err_y_deg - math.copysign(self.DEADZONE_DEG, err_y_deg)

        a = self.DERIV_ALPHA
        raw_dx = (err_x_deg - self.prev_err_x) / dt
        raw_dy = (err_y_deg - self.prev_err_y) / dt
        dx = a*raw_dx + (1-a)*self.prev_der_x
        dy = a*raw_dy + (1-a)*self.prev_der_y

        self.int_err_x += px * dt
        self.int_err_y += py * dt
        self.int_err_x = self._clamp(self.int_err_x, -self.INTEGRAL_MAX, self.INTEGRAL_MAX)
        self.int_err_y = self._clamp(self.int_err_y, -self.INTEGRAL_MAX, self.INTEGRAL_MAX)

        vy_raw = self._clamp(self.KP_Y*px + self.KI_Y*self.int_err_x + self.KD_Y*dx, -self.VY_MAX, self.VY_MAX)
        vz_raw = self._clamp(self.KP_Z*py + self.KI_Z*self.int_err_y + self.KD_Z*dy, -self.VZ_MAX, self.VZ_MAX)

        # EMA çıkış yumuşatma — titreşimi önler
        s = self.OUTPUT_SMOOTH
        self.smooth_vy = s * vy_raw + (1 - s) * self.smooth_vy
        self.smooth_vz = s * vz_raw + (1 - s) * self.smooth_vz
        vy = self.smooth_vy
        vz = self.smooth_vz

        total_err_deg = math.sqrt(err_x_deg**2 + err_y_deg**2)
        if total_err_deg > self.VX_ERR_THRESH:
            err_ratio = self._clamp(total_err_deg / (self.VX_ERR_THRESH * 3), 0, 1)
            vx = self._clamp(self.VX_BASE * (1.0 - err_ratio * 0.35), self.VX_MIN, self.VX_BASE)
        else:
            vx = self._clamp(self.VX_BASE + self.VX_AREA_GAIN * area, self.VX_BASE, self.VX_MAX)

        self.prev_err_x = err_x_deg; self.prev_err_y = err_y_deg
        self.prev_der_x = dx; self.prev_der_y = dy
        return vx, vy, vz, err_x_deg, err_y_deg

    def _anti_windup(self, vy, vz):
        if abs(vy) >= self.VY_MAX * 0.95: self.int_err_x *= 0.9
        if abs(vz) >= self.VZ_MAX * 0.95: self.int_err_y *= 0.9

    # ─────────────────────────────────────────────────
    # KONUMLU GÜDÜM — Hedef uçağın GPS'ine git
    # ─────────────────────────────────────────────────
    def _run_konumlu(self, now, elapsed, mode, alt, r_deg, p_deg, y_deg):
        """Hedef uçağın telemetri konumuna GPS waypoint gönderir."""
        if not self.target.has_fix:
            if (now - self.last_cmd_t) >= 1.0:
                print(f"[KONUMLU] Hedef uçak telemetrisi bekleniyor…")
                self.last_cmd_t = now
            return

        t_lat_raw, t_lon_raw = self.target.lat, self.target.lon
        t_alt_raw = self.target.alt

        # GPS + irtifa yumuşatma (EMA) — titreşimi önler
        if self.smooth_lat is None:
            self.smooth_lat = t_lat_raw
            self.smooth_lon = t_lon_raw
            self.smooth_alt = t_alt_raw
        else:
            sg = self.GPS_SMOOTH
            sa = self.ALT_SMOOTH
            self.smooth_lat = sg * t_lat_raw + (1 - sg) * self.smooth_lat
            self.smooth_lon = sg * t_lon_raw + (1 - sg) * self.smooth_lon
            self.smooth_alt = sa * t_alt_raw + (1 - sa) * self.smooth_alt

        t_lat = self.smooth_lat
        t_lon = self.smooth_lon
        t_alt = self.smooth_alt

        if mode == "GUIDED" and (now - self.last_cmd_t) >= self.KONUMLU_CMD_INTERVAL:
            # Mesafe ve yön hesapla
            R_EARTH = 6378137.0
            dn = math.radians(t_lat - self.mav.lat) * R_EARTH
            de = math.radians(t_lon - self.mav.lon) * R_EARTH * math.cos(math.radians(self.mav.lat))
            dist = math.sqrt(dn**2 + de**2)

            # Drone→Hedef yönünde bearing hesapla (radyan, kuzey=0, saat yönü pozitif)
            bearing = math.atan2(de, dn)  # NED: atan2(east, north)

            # Waypoint'i hedefin LEAD_DISTANCE metre ilerisine koy
            if dist > 1.0:
                scale = self.LEAD_DISTANCE / dist
                lead_dn = dn * scale
                lead_de = de * scale
                wp_lat = t_lat + (lead_dn / R_EARTH) * (180.0 / math.pi)
                wp_lon = t_lon + (lead_de / (R_EARTH * math.cos(math.radians(t_lat)))) * (180.0 / math.pi)
            else:
                wp_lat, wp_lon = t_lat, t_lon

            self.mav.send_gps_target(wp_lat, wp_lon, t_alt, yaw=bearing)
            self.last_cmd_t = now

            bearing_deg = math.degrees(bearing) % 360
            print(f"[KONUMLU] Mesafe: {dist:.0f}m | Bearing: {bearing_deg:.0f}° | "
                  f"Lead: {self.LEAD_DISTANCE:.0f}m | Alt: {t_alt:.0f}m")

        self.logger.log([
            f"{now:.4f}", f"{elapsed:.2f}", "", "konumlu", "GPS_TRACK", mode,
            "","","","","", "","", "","",
            "","","",
            f"{self.mav.lat:.7f}", f"{self.mav.lon:.7f}", f"{alt:.1f}",
            f"{t_lat:.7f}", f"{t_lon:.7f}", f"{t_alt:.1f}",
            f"{alt:.1f}", f"{r_deg:.1f}", f"{p_deg:.1f}", f"{y_deg:.1f}", ""
        ])

    # ─────────────────────────────────────────────────
    # GÖRÜNTÜLÜ GÜDÜM — Visual Servo PID
    # ─────────────────────────────────────────────────
    def _run_goruntulu(self, now, dt, elapsed, mode, alt, r_deg, p_deg, y_deg):
        """BBox verisine göre PID visual servo çalıştırır."""
        try:
            data = self.dq.get(timeout=0.03)
        except queue.Empty:
            data = None

        # ── HEDEF VAR ──
        if data:
            self.last_target_t = now
            cx, cy = data['cx'], data['cy']
            w, h   = data['w'],  data['h']
            area   = data['area']
            age_ms = (now - data['ts']) * 1000

            stab_cx, stab_cy = self._stabilize(cx, cy)
            vx, vy, vz, ex, ey = self._pid_step(stab_cx, stab_cy, area, dt)
            self._anti_windup(vy, vz)
            self.last_vx, self.last_vy, self.last_vz = vx, vy, vz
            state = "TRACKING"

            if mode == "GUIDED" and (now - self.last_cmd_t) >= self.CMD_INTERVAL:
                self.mav.send_velocity(vx, vy, vz)
                self.last_cmd_t = now
                print(f"[GÖRÜNTÜLÜ] vx={vx:+5.1f}  vy={vy:+5.2f}  vz={vz:+5.2f}  "
                      f"err({ex:+.1f}°,{ey:+.1f}°)  area={area:.0f}  age={age_ms:.0f}ms")

            self.logger.log([
                f"{now:.4f}", f"{elapsed:.2f}", f"{dt:.4f}", "goruntulu", state, mode,
                f"{cx:.1f}", f"{cy:.1f}", f"{w:.0f}", f"{h:.0f}", f"{area:.0f}",
                f"{stab_cx:.1f}", f"{stab_cy:.1f}",
                f"{ex:.3f}", f"{ey:.3f}",
                f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}",
                "","","", "","","",
                f"{alt:.1f}", f"{r_deg:.1f}", f"{p_deg:.1f}", f"{y_deg:.1f}",
                f"{age_ms:.1f}"
            ])
            return

        # ── HEDEF YOK — COASTING / FAILSAFE ──
        if self.last_target_t == 0:
            return

        gap = now - self.last_target_t
        if gap <= self.COAST_TIMEOUT:
            state = "COASTING"
            if mode == "GUIDED" and (now - self.last_cmd_t) >= self.CMD_INTERVAL:
                self.mav.send_velocity(self.last_vx, self.last_vy, self.last_vz)
                self.last_cmd_t = now
                print(f"[GÖRÜNTÜLÜ-COAST {gap:.1f}s] vx={self.last_vx:+.1f} vy={self.last_vy:+.2f} vz={self.last_vz:+.2f}")
        elif gap <= self.FAILSAFE_TIMEOUT:
            state = "FAILSAFE"
            vx_fs = max(2.0, self.last_vx * 0.5)
            if mode == "GUIDED" and (now - self.last_cmd_t) >= 0.2:
                self.mav.send_velocity(vx_fs, 0, 0)
                self.last_cmd_t = now
                print(f"[GÖRÜNTÜLÜ-FAILSAFE {gap:.1f}s] vx={vx_fs:.1f}")
        else:
            state = "LOST"
            self._reset_pid()
            if mode == "GUIDED" and (now - self.last_cmd_t) >= 0.5:
                self.mav.send_velocity(0, 0, 0)
                self.last_cmd_t = now
                print(f"[GÖRÜNTÜLÜ-LOST {gap:.1f}s] Hover")

        self.logger.log([
            f"{now:.4f}", f"{elapsed:.2f}", f"{gap:.4f}", "goruntulu", state, mode,
            "","","","","", "","", "","",
            f"{self.last_vx:.3f}", f"{self.last_vy:.3f}", f"{self.last_vz:.3f}",
            "","","", "","","",
            f"{alt:.1f}", f"{r_deg:.1f}", f"{p_deg:.1f}", f"{y_deg:.1f}",
            f"{gap*1000:.0f}"
        ])

    # ─────────────────────────────────────────────────
    # ANA DÖNGÜ
    # ─────────────────────────────────────────────────
    def run(self):
        print("="*60)
        print("  ÇİFT MODLU KONTROLCÜ — KONUMLU + GÖRÜNTÜLÜ")
        print("  Redis 'komut_yetkisi' anahtarına göre mod seçer")
        print("="*60)

        last_mode_print = ""

        try:
            while True:
                now = time.time()
                dt  = max(0.001, min(now - self.last_t, 0.2))
                self.last_t = now
                elapsed = now - self.start_t

                alt   = self.mav.alt
                mode  = self.mav.mode
                r_deg = math.degrees(self.mav.roll)
                p_deg = math.degrees(self.mav.pitch)
                y_deg = math.degrees(self.mav.yaw) % 360

                # Redis'ten komut yetkisini oku
                komut = self.redis.get_komut_yetkisi()

                # Mod değişimi logla
                if komut != last_mode_print:
                    print(f"\n{'='*50}")
                    print(f"  MOD DEĞİŞİMİ → {komut.upper()}")
                    print(f"{'='*50}\n")
                    # Görüntülüden konumluya geçişte PID sıfırla
                    if komut == 'konumlu':
                        self._reset_pid()
                        self.last_target_t = 0
                    # Konumludan görüntülüye geçişte bbox kuyruğunu temizle
                    elif komut == 'goruntulu':
                        while not self.dq.empty():
                            try: self.dq.get_nowait()
                            except: break
                    last_mode_print = komut

                # Moda göre ilgili kontrolcüyü çalıştır
                if komut == 'goruntulu':
                    self._run_goruntulu(now, dt, elapsed, mode, alt, r_deg, p_deg, y_deg)
                else:
                    self._run_konumlu(now, elapsed, mode, alt, r_deg, p_deg, y_deg)
                    time.sleep(0.05)  # Konumlu modda CPU kullanımını düşür

        except KeyboardInterrupt:
            print("\n[KAPANIYOR] Hover komutu gönderiliyor…")
            try: self.mav.send_velocity(0, 0, 0)
            except: pass
            self.logger.close()
            self.mav.running = False
            self.redis.running = False
            self.target.running = False


# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Çift Modlu Kontrolcü — Konumlu + Görüntülü")
    parser.add_argument('--conn', default='udp:127.0.0.1:14551',
                        help='Drone MAVLink bağlantısı (varsayılan: udp:127.0.0.1:14551)')
    parser.add_argument('--target', default='udp:127.0.0.1:14561',
                        help='Hedef uçak MAVLink bağlantısı (varsayılan: udp:127.0.0.1:14561)')
    args = parser.parse_args()

    ctrl = VisualServoAttackController(args.conn, args.target)
    ctrl.run()
