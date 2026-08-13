#!/usr/bin/env python3
"""ucus_stab_test.py - GIMBAL STABILIZASYON UCUS TESTI (tek drone, headless)

Sorulan soru: govde pitch yaparken kamera DUNYA pitch'i sabit kaliyor mu?
Kanit sekli: kopteri 30 m'de asili tutup ILERI 15 m/s hiz komutu vermek.
Ivmelenme sirasinda govde -10..-25 derece burun asagi yatar; gimbal
calisiyorsa gimbal_tilt_status (= kameranin DUNYA pitch'i, pozitif=yukari)
0'da kalmalidir.

NEDEN IVMELENME: bu sim kopterinin surtunmesi yok denecek kadar az; SABIT
seyirde pitch ~-1 derece, yani sabit hizda test hicbir sey kanitlamaz.
Tek gozlemlenebilir govde pitch penceresi ivmelenme (ve fren) fazidir.

=====================================================================
RUNBOOK
=====================================================================
Calistirma:   python3 ucus_stab_test.py          (argumansiz)
Sure:         ~2.5-4 dakika (gzserver 20-60 s, EKF/prearm 30-90 s,
              kalkis ~15 s, olcum 15 s)
Cikis kodu:   0 = PASS, 1 = FAIL, 2 = kurulum hatasi (port/surec)

KENDI BASLATTIGI SURECLER (hepsi setsid + killpg ile temizlenir):
  gzserver     worlds/tek_avci.world            gazebo master TCP 11345
  arducopter   SITL -I0, SysID 1                TCP 5760 (MAVLink, serial0)
                                                UDP 5501 (RCin)
                                                UDP 9002 (SITL->Gazebo FDM)
                                                UDP 9003 (Gazebo->SITL FDM)
  gz topic -e  gimbal_tilt_status akisi          (gazebo transport)

BASLATMAZ: roscore, MAVProxy, redis, bbox, QGC, hedef ucak SITL.
  - MAVProxy YOK: dogrudan tcp:127.0.0.1:5760'a pymavlink ile baglanilir.
    Yan fayda: MAVProxy'nin periyodik REQUEST_DATA_STREAM'i bizim
    SET_MESSAGE_INTERVAL'imizi EZMEZ (bkz. yildizlar_gudum.sh notu).
  - roscore YOK: kamera gerekmiyor, gzserver ROS eklentisi olmadan kosar.
    libgazebo_ros_camera.so ros::isInitialized() false gorup sessizce
    devre disi kalir (ve render maliyeti de dusar).
    Zorla ROS istenirse: UCUS_STAB_ROS=1 python3 ucus_stab_test.py

CIKTILAR:
  <scratchpad>/ucus_stab_test.csv   10 Hz ham kayit
  <scratchpad>/ucus_stab_gz.log     gzserver
  <scratchpad>/ucus_stab_sitl.log   arducopter SITL

PASS OLCUTU:
  ivmelenme fazinda |govde pitch| tepe >= 5 derece   VE
  |kamera dunya pitch| (gimbal_tilt_status) p95 < 2 derece

ISARET SOZLESMESI:
  ATTITUDE.pitch  : pozitif = burun YUKARI (ileri ivmelenme -> NEGATIF)
  gimbal status   : pozitif = kamera YUKARI (stabilize modunda DUNYA pitch'i)

BILINEN OLCUM SINIRI: plugin status'u her 101 fizik adiminda bir yayinlar;
tek_avci.world 500 Hz fizikle kostugu icin ~5 Hz eder (RTF<1 ise daha da
az). 10 Hz kayitta ardisik satirlar ayni ornegi tasiyabilir; bu yuzden CSV'de
'durum_yasi_ms' sutunu ve ozette 'benzersiz ornek' sayisi vardir.
"""

import math
import os
import tempfile
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

# --------------------------------------------------------------- sabitler
PROJE = os.environ.get('YILDIZLAR_DIR',
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ARDUPILOT = os.environ.get('ARDUPILOT_DIR', os.path.expanduser('~/ardupilot'))
AP_GAZEBO = os.environ.get('ARDUPILOT_GAZEBO_DIR',
                           os.path.expanduser('~/ardupilot_gazebo'))
IQ_SIM = os.environ.get('IQ_SIM_MODELS',
                        os.path.expanduser('~/catkin_ws/src/iq_sim/models'))

DUNYA = os.path.join(PROJE, 'worlds', 'tek_avci.world')
SITL_BIN = os.path.join(ARDUPILOT, 'build', 'sitl', 'bin', 'arducopter')
VARSAYILANLAR = ','.join([
    os.path.join(ARDUPILOT, 'Tools/autotest/default_params/copter.parm'),
    os.path.join(ARDUPILOT, 'Tools/autotest/default_params/gazebo-iris.parm'),
    os.path.join(PROJE, 'params', 'swarm_copter.parm'),
])
HOME_POS = os.environ.get('YILDIZ_HOME', '-35.363261,149.165230,0,0')

MODEL = 'iris-1'                      # worlds/tek_avci.world sarmalayici model adi
CMD_TOPIC = f'/gazebo/default/{MODEL}/gimbal_tilt_cmd'
STATUS_TOPIC = f'/gazebo/default/{MODEL}/gimbal_tilt_status'

BASELINE = os.environ.get('UCUS_STAB_BASELINE') == '1'   # gimbal'siz kiyas kosusu

MAVLINK_ADRES = 'tcp:127.0.0.1:5760'  # SITL -I0 serial0 (MAVProxy'siz)
SYSID = 1

GEREKLI_PORTLAR = [('tcp', 5760), ('tcp', 11345), ('udp', 5501),
                   ('udp', 9002), ('udp', 9003)]

KALKIS_ALT = 30.0                     # m
ILERI_HIZ = 15.0                      # m/s, govde +X
IVME_SURESI = 10.0                    # s, 15 m/s komutu
FREN_SURESI = 5.0                     # s, 0 m/s komutu (ters isaretli pitch tepesi)
KAYIT_HZ = 10.0
TELEMETRI_HZ = 20.0                   # SET_MESSAGE_INTERVAL (ATTITUDE / GLOBAL_POSITION_INT)

PITCH_TEPE_ESIK = 5.0                 # derece, gozlem penceresi gecerlilik sarti
STATUS_P95_ESIK = 2.0                 # derece, PASS esigi

COPTER_GUIDED = 4
TIP_MASKE_HIZ = 0b0000110111000111    # yalniz vx,vy,vz kullan (pos/acc/yaw yok)
MAV_FRAME_BODY_NED = 8                # ArduCopter bunu yaw ile dondurur

# log/CSV dizini: tools/ icini kirletme, /tmp altinda calis
SCRATCH = os.environ.get('UCUS_STAB_DIR',
    os.path.join(tempfile.gettempdir(), 'gimbal_ucus_test'))
os.makedirs(SCRATCH, exist_ok=True)
CSV_YOL = os.path.join(SCRATCH, 'ucus_stab_test.csv')
GZ_LOG = os.path.join(SCRATCH, 'ucus_stab_gz.log')
SITL_LOG = os.path.join(SCRATCH, 'ucus_stab_sitl.log')
SITL_IS_DIZINI = os.path.join(SCRATCH, 'sitl0')

STATUS_RE = re.compile(r'data:\s*"([^"]*)"')

_surecler = []          # (ad, Popen) - killpg ile toplu kapatilir


def yaz(metin):
    print(metin, flush=True)


# ------------------------------------------------------- surec yonetimi
def baslat(ad, argv, log_yolu, env=None, cwd=None):
    """setsid'li surec baslatir; cikti log dosyasina yazilir."""
    log = open(log_yolu, 'wb')
    # start_new_session=True == setsid: her surec KENDI surec grubunda, boylece
    # killpg ile cocuklariyla (or. SITL supervisor zinciri) birlikte olur.
    proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                            env=env, cwd=cwd, start_new_session=True)
    _surecler.append((ad, proc))
    yaz(f'  {ad} basladi (PID {proc.pid}, log: {log_yolu})')
    return proc


def hepsini_temizle():
    for ad, proc in reversed(_surecler):
        if proc.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    bitis = time.monotonic() + 8
    while time.monotonic() < bitis:
        if all(p.poll() is not None for _, p in _surecler):
            break
        time.sleep(0.2)
    for ad, proc in _surecler:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    yaz('temizlik tamam.')


def canli_mi(proc, ad, log_yolu):
    if proc.poll() is not None:
        kuyruk = ''
        try:
            with open(log_yolu, 'r', errors='replace') as f:
                kuyruk = ''.join(f.readlines()[-25:])
        except OSError:
            pass
        raise Kurulum(f'{ad} beklenmedik bicimde sonlandi '
                      f'(cikis {proc.returncode}).\n--- {log_yolu} son satirlar ---\n{kuyruk}')


class Kurulum(Exception):
    """Kurulum/ortam hatasi - cikis kodu 2."""


# ------------------------------------------------------------- on kontrol
def on_kontrol():
    eksik = [y for y in (DUNYA, SITL_BIN, os.path.join(PROJE, 'params/swarm_copter.parm'),
                         os.path.join(AP_GAZEBO, 'build', 'libGimbalSmall2dPlugin.so'),
                         os.path.join(AP_GAZEBO, 'build', 'libArduPilotPlugin.so'))
             if not os.path.exists(y)]
    for parca in VARSAYILANLAR.split(','):
        if not os.path.exists(parca):
            eksik.append(parca)
    if eksik:
        raise Kurulum('Eksik bagimlilik:\n  ' + '\n  '.join(eksik))
    for komut in ('gzserver', 'gz', 'stdbuf', 'ss'):
        if subprocess.run(['bash', '-c', f'command -v {komut}'],
                          capture_output=True).returncode != 0:
            raise Kurulum(f'Eksik komut: {komut}')
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        raise Kurulum('pymavlink kurulu degil (pip install pymavlink)')

    cikti = subprocess.run(['ss', '-H', '-lntu'], capture_output=True, text=True).stdout
    mesgul = []
    for satir in cikti.splitlines():
        alan = satir.split()
        if len(alan) < 5:
            continue
        proto = alan[0].lower()
        yerel = alan[4]
        try:
            port = int(yerel.rsplit(':', 1)[1])
        except (IndexError, ValueError):
            continue
        for p_tip, p_no in GEREKLI_PORTLAR:
            if port == p_no and proto.startswith(p_tip):
                mesgul.append(f'{p_tip}/{p_no}')
    if mesgul:
        raise Kurulum(
            'Gerekli portlar KULLANIMDA: ' + ', '.join(sorted(set(mesgul))) +
            '\nBaska bir SITL/Gazebo kosuyor olmali. Once kapatin:\n'
            f'  {PROJE}/yildizlar_gudum.sh --stop\n'
            '  (ya da: pkill -f gzserver; pkill -f arducopter)')


def gazebo_ortami():
    env = dict(os.environ)
    env['GAZEBO_MODEL_PATH'] = ':'.join([
        os.path.join(PROJE, 'models'),
        os.path.join(AP_GAZEBO, 'models'),
        IQ_SIM,
        '/usr/share/gazebo-11/models',
    ])
    env['GAZEBO_PLUGIN_PATH'] = ':'.join([
        os.path.join(AP_GAZEBO, 'build'),
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
    ])
    env.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345')
    return env


# ---------------------------------------------------------------- gazebo
def gazebo_baslat(env):
    argv = ['gzserver', '--verbose']
    if os.environ.get('UCUS_STAB_ROS') == '1':
        argv += ['-s', 'libgazebo_ros_api_plugin.so']
    argv.append(DUNYA)
    proc = baslat('gzserver', argv, GZ_LOG, env=env)

    yaz('  gimbal topic\'i bekleniyor (gz topic -l)...')
    bitis = time.monotonic() + 120
    while time.monotonic() < bitis:
        canli_mi(proc, 'gzserver', GZ_LOG)
        try:
            r = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                               text=True, env=env, timeout=20)
        except subprocess.TimeoutExpired:
            continue          # master henuz yok, gz asili kaldi
        hedef = f'/gazebo/default/{MODEL}/' if BASELINE else STATUS_TOPIC
        if hedef in r.stdout:
            yaz(f'  Gazebo hazir ({hedef} gorundu).')
            return proc
        time.sleep(1.0)
    raise Kurulum(f'{STATUS_TOPIC} 120 s icinde gorunmedi. '
                  f'gimbal plugin yuklenmemis olabilir; bak: {GZ_LOG}')


def tilt_komut(env, rad):
    r = subprocess.run(['gz', 'topic', '-p', CMD_TOPIC, '-m', f'data: "{rad}"'],
                       capture_output=True, text=True, env=env, timeout=20)
    if r.returncode != 0:
        raise Kurulum(f'gz topic -p basarisiz: {r.stderr.strip()}')


class DurumOkuyucu:
    """gimbal_tilt_status'u SUREKLI akitan tek bir 'gz topic -e' surecinden okur.

    Periyodik 'gz topic -e -d 1' cagirmak yerine tek surec: her cagri yeni bir
    gazebo transport baglantisi kurup ~1 s bekliyor, 10 Hz kayitta imkansiz.
    stdbuf -oL SART: gz cikti boru hattina baglaninca blok tamponlar ve
    ~5 Hz'lik kucuk mesajlarla tampon dakikalarca dolmaz.
    """

    def __init__(self, env):
        self.env = env
        self.deger = None
        self.zaman = 0.0
        self.n = 0
        self._dur = False
        self._proc = None
        self._is = None

    def basla(self):
        self._proc = subprocess.Popen(
            ['stdbuf', '-oL', 'gz', 'topic', '-e', STATUS_TOPIC, '-u'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=self.env, start_new_session=True)
        _surecler.append(('gz_topic_echo', self._proc))
        self._is = threading.Thread(target=self._dongu, daemon=True)
        self._is.start()

    def _dongu(self):
        tampon = ''
        while not self._dur:
            try:
                parca = self._proc.stdout.read1(4096)
            except Exception:
                break
            if not parca:
                break
            tampon += parca.decode('utf-8', 'replace')
            son = 0
            for m in STATUS_RE.finditer(tampon):
                try:
                    v = float(m.group(1))
                except ValueError:
                    son = m.end()
                    continue
                self.deger = v
                self.zaman = time.monotonic()
                self.n += 1
                son = m.end()
            tampon = tampon[son:]
            if len(tampon) > 16384:
                tampon = tampon[-1024:]

    def bekle(self, timeout=20):
        bitis = time.monotonic() + timeout
        while time.monotonic() < bitis:
            if self.n > 0:
                return True
            time.sleep(0.2)
        return False

    def dur(self):
        self._dur = True
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                pass


# ------------------------------------------------------------------ SITL
def sitl_baslat(env):
    os.makedirs(SITL_IS_DIZINI, exist_ok=True)
    # eeprom.bin kalirsa onceki kosunun kaydedilmis parametreleri --defaults'i
    # ezer; her kosu TEMIZ parametreyle baslasin.
    for ad in ('eeprom.bin', 'eeprom.bin.bak'):
        try:
            os.unlink(os.path.join(SITL_IS_DIZINI, ad))
        except FileNotFoundError:
            pass
    argv = [SITL_BIN, '--model', 'gazebo-iris', '--speedup', '1',
            '--sysid', str(SYSID), '--slave', '0',
            '--defaults', VARSAYILANLAR,
            '--sim-address=127.0.0.1', '-I0', '--home', HOME_POS]
    yaz('  SITL komutu: ' + ' '.join(argv))
    return baslat('arducopter', argv, SITL_LOG, env=env, cwd=SITL_IS_DIZINI)


# --------------------------------------------------------------- mavlink
class Ucak:
    """pymavlink sarmalayicisi: tek okuyucu is parcacigi + kilitli gonderim.

    Repodaki olculmus ders (tools/suru_komut.py:konum_al): tek recv_match()
    cagrisi UDP/TCP tamponundaki EN ESKI mesaji dondurur; dongu basina bir
    okuma yapinca veri bayatlar. Burada ayri bir is parcacigi tamponu surekli
    bosaltir ve YALNIZ en taze ornek saklanir.
    """

    def __init__(self, adres, sysid):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(adres, source_system=254,
                                            source_component=190)
        self.sysid = sysid
        self.kilit = threading.Lock()
        self._dur = False
        self.attitude = None          # (t_mono, roll, pitch, yaw) rad
        self.konum = None             # (t_mono, rel_alt_m, vx, vy, vz) m, m/s
        self.heartbeat = None         # (t_mono, custom_mode, armed)
        self.acks = {}                # komut_no -> (t_mono, result)
        self.statustext = []
        self._is = None

    def baglan(self, timeout=90):
        bitis = time.monotonic() + timeout
        while time.monotonic() < bitis:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg is not None and msg.get_srcSystem() == self.sysid:
                self.m.target_system = self.sysid
                self.m.target_component = msg.get_srcComponent()
                self._is = threading.Thread(target=self._dongu, daemon=True)
                self._is.start()
                return True
        return False

    def _dongu(self):
        while not self._dur:
            with self.kilit:
                msg = self.m.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.002)
                continue
            if msg.get_srcSystem() != self.sysid:
                continue
            t = time.monotonic()
            tip = msg.get_type()
            if tip == 'ATTITUDE':
                self.attitude = (t, msg.roll, msg.pitch, msg.yaw)
            elif tip == 'GLOBAL_POSITION_INT':
                self.konum = (t, msg.relative_alt / 1000.0,
                              msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
            elif tip == 'HEARTBEAT':
                armed = bool(msg.base_mode &
                             self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.heartbeat = (t, msg.custom_mode, armed)
            elif tip == 'GPS_RAW_INT':
                self.gps = (msg.fix_type, msg.satellites_visible)
            elif tip == 'COMMAND_ACK':
                self.acks[msg.command] = (t, msg.result)
            elif tip == 'STATUSTEXT':
                metin = msg.text.strip()
                if not self.statustext or self.statustext[-1] != metin:
                    self.statustext.append(metin)
                    yaz(f'    [FCU] {metin}')

    gps = (0, 0)

    def gonder(self, fn, *a, **kw):
        with self.kilit:
            fn(*a, **kw)

    def komut(self, komut_no, *params, bekle=True, timeout=5):
        """COMMAND_LONG gonderir. ACK'i okuyucu is parcacigi topladigi icin
        burada recv_match cagirilmaz (iki yerden okumak mesaj kaybettirir)."""
        self.acks.pop(komut_no, None)
        params = list(params) + [0] * (7 - len(params))
        self.gonder(self.m.mav.command_long_send, self.m.target_system,
                    self.m.target_component, komut_no, 0, *params[:7])
        if not bekle:
            return None
        bitis = time.monotonic() + timeout
        while time.monotonic() < bitis:
            ack = self.acks.get(komut_no)
            if ack is not None:
                return ack[1]
            time.sleep(0.01)
        return None

    def akis_iste(self, hz=TELEMETRI_HZ):
        aralik = int(1e6 / hz)
        for msg_id in (self.mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       self.mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
            self.komut(self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       msg_id, aralik, bekle=False)
        # Kemer+askı: bazi surumler SET_MESSAGE_INTERVAL'i yalniz belirli
        # akislar icin uygular; klasik istek de gonderilir.
        self.gonder(self.m.mav.request_data_stream_send,
                    self.m.target_system, self.m.target_component,
                    self.mavutil.mavlink.MAV_DATA_STREAM_ALL, int(hz), 1)

    def mod_ayarla(self, custom_mode):
        self.gonder(self.m.mav.set_mode_send, self.m.target_system,
                    self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode)

    def mod_bekle(self, custom_mode, timeout=20):
        bitis = time.monotonic() + timeout
        while time.monotonic() < bitis:
            hb = self.heartbeat
            if hb and hb[1] == custom_mode:
                return True
            self.mod_ayarla(custom_mode)
            time.sleep(0.5)
        return False

    def hiz_komutu(self, vx, vy=0.0, vz=0.0):
        """Govde cercevesinde hiz setpoint'i (ArduCopter yaw ile dondurur)."""
        self.gonder(self.m.mav.set_position_target_local_ned_send,
                    0, self.m.target_system, self.m.target_component,
                    MAV_FRAME_BODY_NED, TIP_MASKE_HIZ,
                    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

    def kapat(self):
        self._dur = True
        time.sleep(0.2)
        try:
            self.m.close()
        except Exception:
            pass


# ------------------------------------------------------------- ucus adimlari
def prearm_bekle(u, timeout=180):
    bitis = time.monotonic() + timeout
    while time.monotonic() < bitis:
        fix, sat = u.gps
        if fix >= 3 and sat >= 6 and u.konum is not None:
            return True
        time.sleep(0.5)
    return False


def arm_et(u, timeout=90):
    mavlink = u.mavutil.mavlink
    bitis = time.monotonic() + timeout
    while time.monotonic() < bitis:
        sonuc = u.komut(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        alt_bitis = time.monotonic() + 3
        while time.monotonic() < alt_bitis:
            hb = u.heartbeat
            if hb and hb[2]:
                return True
            time.sleep(0.1)
        yaz(f'    ARM reddedildi (sonuc={sonuc}), yeniden deneniyor...')
        time.sleep(2)
    return False


def kalkis(u, alt, timeout=120):
    mavlink = u.mavutil.mavlink
    u.komut(mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    bitis = time.monotonic() + timeout
    son_rapor = 0.0
    while time.monotonic() < bitis:
        k = u.konum
        if k is not None:
            if time.monotonic() - son_rapor > 2:
                yaz(f'    irtifa {k[1]:5.1f} m')
                son_rapor = time.monotonic()
            if k[1] >= alt * 0.97:
                return True
        time.sleep(0.2)
    return False


def sabitlenme_bekle(u, alt, timeout=40):
    """Dikey hiz ve pitch oturana kadar bekler: olcum SIFIRDAN baslasin."""
    bitis = time.monotonic() + timeout
    sabit_baslangic = None
    while time.monotonic() < bitis:
        k, a = u.konum, u.attitude
        if k and a:
            sakin = (abs(k[4]) < 0.5 and math.hypot(k[2], k[3]) < 1.0
                     and abs(math.degrees(a[2])) < 2.0 and abs(k[1] - alt) < 2.0)
            if sakin:
                if sabit_baslangic is None:
                    sabit_baslangic = time.monotonic()
                elif time.monotonic() - sabit_baslangic > 3.0:
                    return True
            else:
                sabit_baslangic = None
        time.sleep(0.2)
    return False


# ------------------------------------------------------------------ olcum
def p95(dizi):
    if not dizi:
        return float('nan')
    s = sorted(dizi)
    i = min(len(s) - 1, int(round(0.95 * (len(s) - 1))))
    return s[i]


def ozet_satiri(ad, dizi, birim='deg'):
    if not dizi:
        return f'  {ad:26s} ORNEK YOK'
    return (f'  {ad:26s} min={min(dizi):7.2f}  ort={statistics.median(dizi):7.2f}  '
            f'max={max(dizi):7.2f}  ({birim})')


def olcum_dongusu(u, durum):
    """10 Hz kayit: ivme fazi (+15 m/s) + fren fazi (0 m/s)."""
    satirlar = []
    t0 = time.monotonic()
    toplam = IVME_SURESI + FREN_SURESI
    periyot = 1.0 / KAYIT_HZ
    sonraki = t0
    yaz(f'  faz IVME: {ILERI_HIZ} m/s govde +X, {IVME_SURESI} s')
    faz_yazildi = 'ivme'
    while True:
        simdi = time.monotonic()
        t = simdi - t0
        if t >= toplam:
            break
        faz = 'ivme' if t < IVME_SURESI else 'fren'
        if faz != faz_yazildi:
            yaz(f'  faz FREN: 0 m/s, {FREN_SURESI} s')
            faz_yazildi = faz
        # GUIDED hiz hedefi ~3 s'de zaman asimina ugrar: her karede tazelenir.
        u.hiz_komutu(ILERI_HIZ if faz == 'ivme' else 0.0)

        a, k = u.attitude, u.konum
        pitch = math.degrees(a[2]) if a else float('nan')
        roll = math.degrees(a[1]) if a else float('nan')
        yer_hiz = math.hypot(k[2], k[3]) if k else float('nan')
        alt = k[1] if k else float('nan')
        st = durum.deger
        st_deg = math.degrees(st) if st is not None else float('nan')
        st_yas = (simdi - durum.zaman) * 1000.0 if durum.zaman else float('nan')
        # Iki yorum: status DUNYA pitch'i ise A dogru, EKLEM acisi ise B.
        # Yeni (stabilize=1) plugin surumunde A gecerlidir.
        yorum_b = pitch + st_deg
        satirlar.append(dict(t=t, faz=faz, yer_hiz=yer_hiz, alt=alt,
                             pitch=pitch, roll=roll, status_deg=st_deg,
                             status_yas_ms=st_yas, dunya_yorumA=st_deg,
                             dunya_yorumB=yorum_b, n_ornek=durum.n))
        sonraki += periyot
        uyku = sonraki - time.monotonic()
        if uyku > 0:
            time.sleep(uyku)
        else:
            sonraki = time.monotonic()
    u.hiz_komutu(0.0)
    return satirlar


def csv_yaz(satirlar):
    basliklar = ['t_s', 'faz', 'yer_hiz_ms', 'irtifa_m', 'govde_pitch_deg',
                 'govde_roll_deg', 'status_deg', 'status_yasi_ms',
                 'kamera_dunya_pitch_A_deg', 'kamera_dunya_pitch_B_deg',
                 'status_ornek_no']
    with open(CSV_YOL, 'w') as f:
        f.write(','.join(basliklar) + '\n')
        for s in satirlar:
            f.write('{t:.3f},{faz},{yer_hiz:.3f},{alt:.2f},{pitch:.3f},'
                    '{roll:.3f},{status_deg:.4f},{status_yas_ms:.1f},'
                    '{dunya_yorumA:.4f},{dunya_yorumB:.4f},{n_ornek}\n'.format(**s))
    yaz(f'\nCSV: {CSV_YOL}')


def ozet_bas(satirlar, durum):
    if not satirlar:
        yaz('\nOZET: hic kayit alinamadi.')
        return 1
    ivme = [s for s in satirlar if s['faz'] == 'ivme']
    pitchler = [s['pitch'] for s in satirlar if not math.isnan(s['pitch'])]
    ivme_pitch = [s['pitch'] for s in ivme if not math.isnan(s['pitch'])]
    statusler = [s['status_deg'] for s in satirlar if not math.isnan(s['status_deg'])]
    yorumB = [s['dunya_yorumB'] for s in satirlar if not math.isnan(s['dunya_yorumB'])]
    hizlar = [s['yer_hiz'] for s in satirlar if not math.isnan(s['yer_hiz'])]
    yaslar = [s['status_yas_ms'] for s in satirlar if not math.isnan(s['status_yas_ms'])]

    yaz('\n' + '=' * 66)
    yaz('OZET')
    yaz('=' * 66)
    yaz(f'  kayit satiri {len(satirlar)}  |  benzersiz gimbal ornegi {durum.n}  '
        f'(~{durum.n / max(1e-6, satirlar[-1]["t"]):.1f} Hz)')
    yaz(ozet_satiri('govde pitch', pitchler))
    yaz(ozet_satiri('kamera dunya pitch (status)', statusler))
    yaz(ozet_satiri('yer hizi', hizlar, 'm/s'))
    if yaslar:
        yaz(f'  {"status ornek yasi":26s} ortanca={statistics.median(yaslar):6.0f} ms  '
            f'p95={p95(yaslar):6.0f} ms')

    ivme_tepe = max((abs(p) for p in ivme_pitch), default=0.0)
    tum_tepe = max((abs(p) for p in pitchler), default=0.0)
    status_p95 = p95([abs(s) for s in statusler])
    status_max = max((abs(s) for s in statusler), default=float('nan'))
    yorumB_p95 = p95([abs(s) for s in yorumB])

    yaz('')
    yaz(f'  |govde pitch| tepe (ivme fazi)   = {ivme_tepe:6.2f} deg  '
        f'(esik >= {PITCH_TEPE_ESIK})')
    yaz(f'  |govde pitch| tepe (tum kayit)   = {tum_tepe:6.2f} deg')
    yaz(f'  |kamera dunya pitch| p95         = {status_p95:6.2f} deg  '
        f'(esik <  {STATUS_P95_ESIK})')
    yaz(f'  |kamera dunya pitch| max         = {status_max:6.2f} deg')
    yaz('')
    yaz('  YORUM KONTROLU (status ne yayinliyor?)')
    yaz(f'    A) status = kamera DUNYA pitch\'i       -> |A| p95 = {status_p95:6.2f} deg')
    yaz(f'    B) status = EKLEM acisi (govde+eklem)  -> |B| p95 = {yorumB_p95:6.2f} deg')
    if status_p95 < yorumB_p95:
        yaz('    -> A tutarli: status stabilize (DUNYA pitch\'i) yayinliyor. [beklenen]')
    else:
        yaz('    -> B tutarli: status hala EKLEM acisi yayinliyor '
            '(gimbal_small_2d model.sdf icinde <stabilize>1</stabilize> var mi?)')

    gecerli = ivme_tepe >= PITCH_TEPE_ESIK
    stabil = status_p95 < STATUS_P95_ESIK
    yaz('')
    if not gecerli:
        yaz(f'SONUC: GECERSIZ - govde pitch {ivme_tepe:.2f} deg, {PITCH_TEPE_ESIK} deg '
            'esigin altinda kaldi. Kopter yeterince ivmelenmedi; ILERI_HIZ / '
            'IVME_SURESI artirilmali ya da WPNAV_ACCEL kontrol edilmeli.')
        return 1
    yaz('SONUC: ' + ('PASS - govde yatarken kamera dunya pitch\'i sabit kaldi.'
                     if stabil else
                     'FAIL - kamera dunya pitch\'i govdeyle birlikte kayiyor.'))
    return 0 if stabil else 1


# ------------------------------------------------------------------- main
def main():
    yaz('=' * 66)
    yaz('UCUS STABILIZASYON TESTI: ileri hiz -> govde pitch -> gimbal')
    yaz('=' * 66)
    yaz('\n[1] on kontrol (bagimliliklar + portlar)')
    on_kontrol()
    yaz('  tamam.')

    env = gazebo_ortami()
    durum = DurumOkuyucu(env)
    u = None
    try:
        yaz('\n[2] gzserver')
        gz_proc = gazebo_baslat(env)

        yaz('\n[3] ArduCopter SITL')
        sitl_proc = sitl_baslat(env)

        yaz(f'\n[4] MAVLink baglantisi ({MAVLINK_ADRES}, SysID {SYSID})')
        # SITL serial0 TCP dinleyicisi acilana kadar bagli deneme.
        u = None
        bitis = time.monotonic() + 90
        while time.monotonic() < bitis:
            canli_mi(sitl_proc, 'arducopter', SITL_LOG)
            canli_mi(gz_proc, 'gzserver', GZ_LOG)
            try:
                u = Ucak(MAVLINK_ADRES, SYSID)
            except Exception:
                time.sleep(1.0)
                continue
            if u.baglan(timeout=20):
                break
            u.kapat()
            u = None
        if u is None:
            raise Kurulum(f'{MAVLINK_ADRES} uzerinden SysID {SYSID} heartbeat '
                          f'alinamadi. Bak: {SITL_LOG}')
        yaz('  heartbeat tamam.')
        u.akis_iste()

        yaz('\n[5] EKF / GPS hazirligi')
        if not prearm_bekle(u):
            raise Kurulum('GPS/EKF hazir olmadi (prearm zaman asimi)')
        yaz(f'  GPS fix={u.gps[0]} uydu={u.gps[1]}')

        yaz('\n[6] GUIDED + ARM')
        if not u.mod_bekle(COPTER_GUIDED):
            raise Kurulum('GUIDED moduna gecilemedi')
        yaz('  GUIDED.')
        if not arm_et(u):
            raise Kurulum('ARM edilemedi')
        yaz('  ARM.')

        yaz(f'\n[7] {KALKIS_ALT:.0f} m kalkis')
        if not kalkis(u, KALKIS_ALT):
            raise Kurulum(f'{KALKIS_ALT} m irtifaya cikilamadi')
        if not sabitlenme_bekle(u, KALKIS_ALT):
            yaz('  UYARI: tam sabitlenme saglanamadi, yine de devam ediliyor.')
        else:
            yaz('  asili ve sakin.')

        yaz('\n[8] gimbal 0.0 rad (ufka bak) + durum akisi')
        if BASELINE:
            yaz('  BASELINE modu: gimbal yok, status atlanacak.')
        else:
            durum.basla()
        for _ in range(0 if BASELINE else 3):
            tilt_komut(env, 0.0)
            time.sleep(0.4)
        if not BASELINE and not durum.bekle(timeout=25):
            # Iki basarisizligi ayirt et: (a) plugin hic yayinlamiyor,
            # (b) yayinliyor ama akis surecinin ciktisi tamponlaniyor.
            try:
                tek = subprocess.run(
                    ['gz', 'topic', '-e', STATUS_TOPIC, '-d', '3', '-u'],
                    capture_output=True, text=True, env=env, timeout=15).stdout
            except subprocess.TimeoutExpired:
                tek = ''
            if STATUS_RE.search(tek):
                raise Kurulum(
                    'Tek atislik "gz topic -e -d 3" ornek DONDURDU ama surekli '
                    'akis bos: cikti tamponlama sorunu (stdbuf -oL ise '
                    'yaramamis). Cozum: gz surumu/stdbuf yerine `script -qfc` '
                    'ile psodo-terminal kullanin.')
            raise Kurulum(
                f'{STATUS_TOPIC} uzerinden hic ornek gelmedi (surekli akis da, '
                'tek atis da bos).\nOlasi neden: gimbal plugin yuklenmedi ya da '
                'status yayinlamiyor.\n'
                f'Elle dogrula: gz topic -l | grep gimbal ; '
                f'gz topic -e {STATUS_TOPIC} -d 3')
        if not BASELINE:
            time.sleep(4.0)   # PID'in oturmasi icin
            yaz(f'  status = {math.degrees(durum.deger):+.3f} deg '
                f'({durum.n} ornek alindi)')

        yaz(f'\n[9] olcum: {ILERI_HIZ} m/s ileri, {KAYIT_HZ:.0f} Hz kayit')
        satirlar = olcum_dongusu(u, durum)
        canli_mi(sitl_proc, 'arducopter', SITL_LOG)
        canli_mi(gz_proc, 'gzserver', GZ_LOG)

        csv_yaz(satirlar)
        return ozet_bas(satirlar, durum)

    finally:
        durum.dur()
        if u is not None:
            u.kapat()
        hepsini_temizle()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Kurulum as e:
        print(f'\nKURULUM HATASI: {e}', file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print('\nkesildi.', file=sys.stderr)
        sys.exit(130)
