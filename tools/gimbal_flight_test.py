#!/usr/bin/env python3
"""flight_stab_test.py - GIMBAL STABILIZATION FLIGHT TEST (single drone, headless)

Question asked: does the camera remain steady WORLD pitch while the body does pitch? Proof form:
suspending the hopper at 30 m and giving the speed command FORWARD 15 m/s. During acceleration, the
fuselage leans nose down -10..-25; If the gimbal is working, gimbal_tilt_status (= camera's WORLD
pitch, positive=up) should remain at 0.

WHY ACCELERATION: this sim hopper has almost no friction az; pitch ~-1 degrees in CONSTANT cruise,
so the test at constant speed proves nothing. The only observable body pitch window is the
acceleration (and braking) phase.

========================================================== RUNBOOK
========================================================================== Run: python3
flight_stab_test.py (no arguments) Duration: ~2.5-4 minutes (gzserver 20-60) 30-90 s, start ~15 s,
measurement 15 s) Exit code: 0 = PASS, 1 = FAIL, 2 = setup error (port/process)

SELF-STARTED PROCESSES (all cleared with setsid + killpg): gzserver worlds/single_pursuer.world gazebo
master TCP 11345 arducopter SITL -I0, SysID 1 TCP 5760 (MAVLink, strand0) UDP 5501 (RCin) UDP 9002
(SITL->Gazebo FDM) UDP 9003 (Gazebo->SITL FDM) gz topic -e gimbal_tilt_status stream (gazebo
transport)

INITIALIZATION: roscore, MAVProxy, redish, bbox, QGC, target plane SITL. - NO MAVProxy: directly
connected to tsp:127.0.0.1:5760 with pymavlink. Side benefit: MAVProxy's periodic
REQUEST_DATA_STREAM does NOT crush our SET_MESSAGE_INTERVAL (see yildizlar_guidance.sh note). - NO
roscore: no camera required, runs without gzserver ROS plugin. libgazebo_ros_camera.so
ros::jobInitialized() sees false and is silently disabled (and rendering costs decrease). If forced
ROS: FLIGHT_STAB_ROS=1 python3 flight_stab_test.py

OUTPUTS: <scratchpad>/flight_stab_test.csv 10 Hz raw log <scratchpad>/flight_stab_gz.log gzserver
<scratchpad>/flight_stab_sitl.log juniper SITL

PASS CRITERIA: during acceleration phase |body pitch| peak >= 5 degree AND |cam world pitch|
(gimbal_tilt_status) p95 < 2 degrees

SIGN AGREEMENT: ATTITUDE.pitch : positive = nose UP (forward acceleration -> NEGATIVE) gimbal status
: positive = camera UP (WORLD pitch in stabilized mode)

KNOWN MEASUREMENT LIMIT: plugin publishes status every 101 physics step; Since single_pursuer.world 500 Hz
runs on physics, ~5 becomes Hz (even more az if RTF<1). In the 10 Hz record, successive lines can
carry the same sample; so CSV has the shot 'state_yasi_ms' and the summary has the number 'unique
sample'.
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

# --------------------------------------------------------------- constants
PROJECT = os.environ.get('YILDIZLAR_DIR',
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ARDUPILOT = os.environ.get('ARDUPILOT_DIR', os.path.expanduser('~/ardupilot'))
AP_GAZEBO = os.environ.get('ARDUPILOT_GAZEBO_DIR',
                           os.path.expanduser('~/ardupilot_gazebo'))
IQ_SIM = os.environ.get('IQ_SIM_MODELS',
                        os.path.expanduser('~/catkin_ws/src/iq_sim/models'))

WORLD = os.path.join(PROJECT, 'worlds', 'single_pursuer.world')
SITL_BIN = os.path.join(ARDUPILOT, 'build', 'sitl', 'bin', 'arducopter')
DEFAULTS = ','.join([
    os.path.join(ARDUPILOT, 'Tools/autotest/default_params/copter.parm'),
    os.path.join(ARDUPILOT, 'Tools/autotest/default_params/gazebo-iris.parm'),
    os.path.join(PROJECT, 'params', 'swarm_copter.parm'),
])
HOME_POS = os.environ.get('YILDIZ_HOME', '-35.363261,149.165230,0,0')

MODEL = 'iris-1'                      # worlds/ single_pursuer .world wrapper model name
CMD_TOPIC = f'/gazebo/default/{MODEL}/gimbal_tilt_cmd'
STATUS_TOPIC = f'/gazebo/default/{MODEL}/gimbal_tilt_status'

BASELINE = os.environ.get('FLIGHT_STAB_BASELINE') == '1'   # gimbal'siz kiyas kosusu

MAVLINK_ADDRESS = 'tcp:127.0.0.1:5760'  # SITL -I0 serial0 (MAVProxy'siz)
SYSID = 1

REQUIRED_PORTS = [('tcp', 5760), ('tcp', 11345), ('udp', 5501),
                   ('udp', 9002), ('udp', 9003)]

TAKEOFF_ALT = 30.0                     # m
FORWARD_SPEED = 15.0                      # m/s, body +X
ACCELERATION_DURATION = 10.0                    # s, 15 m/s command
BRAKING_DURATION = 5.0                     # s, 0 m/s instruction (pitch top with reverse sign)
RECORD_HZ = 10.0
TELEMETRY_HZ = 20.0                   # SET_MESSAGE_INTERVAL (ATTITUDE / GLOBAL_POSITION_INT)

PITCH_PEAK_THRESHOLD = 5.0                 # degree, observation window validity condition
STATUS_P95_THRESHOLD = 2.0                 # degree, PASS threshold

COPTER_GUIDED = 4
TYPE_MASK_SPEED = 0b0000110111000111    # use vx,vy,vz only (no pos/acc/yaw)
MAV_FRAME_BODY_NED = 8                # ArduCopter returns this with yaw

# log/ CSV directory: tools/ do not dirty it, work under /tmp
SCRATCH = os.environ.get('FLIGHT_STAB_DIR',
    os.path.join(tempfile.gettempdir(), 'gimbal_flight_test'))
os.makedirs(SCRATCH, exist_ok=True)
CSV_PATH = os.path.join(SCRATCH, 'flight_stab_test.csv')
GZ_LOG = os.path.join(SCRATCH, 'flight_stab_gz.log')
SITL_LOG = os.path.join(SCRATCH, 'flight_stab_sitl.log')
SITL_IS_DIRECTORY = os.path.join(SCRATCH, 'sitl0')

STATUS_RE = re.compile(r'data:\s*"([^"]*)"')

_processes = []          # (name, Popen) - batch shut down with killpg


def write_value(text_value):
    print(text_value, flush=True)


# ------------------------------------------------------- process management
def launch(label_item, argv, log_path, env=None, cwd=None):
    """starts process with setsid; The output is written to the log file."""
    log = open(log_path, 'wb')
    # start_new_session=True == setsid: each process is in its OWN process group, so killpg with its
    # children (i.e. SITL supervisor chain).
    proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                            env=env, cwd=cwd, start_new_session=True)
    _processes.append((label_item, proc))
    write_value(f'  {label_item} basladi (PID {proc.pid}, log: {log_path})')
    return proc


def all_temizle():
    for label_item, proc in reversed(_processes):
        if proc.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    finish = time.monotonic() + 8
    while time.monotonic() < finish:
        if all(p.poll() is not None for _, p in _processes):
            break
        time.sleep(0.2)
    for label_item, proc in _processes:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    write_value('cleaning is ok.')


def live_mi(proc, label_item, log_path):
    if proc.poll() is not None:
        tail = ''
        try:
            with open(log_path, 'r', errors='replace') as f:
                tail = ''.join(f.readlines()[-25:])
        except OSError:
            pass
        raise Setup(f'{label_item} terminated unexpectedly '
                      f'(output {proc.returncode}).\n---{log_path} last lines ---\n{tail}')


class Setup(Exception):
    """Setup/environment error - exit code 2."""


# ------------------------------------------------------------- ten control
def on_control():
    missing_value = [y for y in (WORLD, SITL_BIN, os.path.join(PROJECT, 'params/swarm_copter.parm'),
                         os.path.join(AP_GAZEBO, 'build', 'libGimbalSmall2dPlugin.so'),
                         os.path.join(AP_GAZEBO, 'build', 'libArduPilotPlugin.so'))
             if not os.path.exists(y)]
    for part in DEFAULTS.split(','):
        if not os.path.exists(part):
            missing_value.append(part)
    if missing_value:
        raise Setup('Missing dependency:\n  ' + '\n  '.join(missing_value))
    for command_value in ('gzserver', 'gz', 'stdbuf', 'ss'):
        if subprocess.run(['bash', '-c', f'command -v {command_value}'],
                          capture_output=True).returncode != 0:
            raise Setup(f'Missing command: {command_value}')
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        raise Setup('pymavlink is not installed (pip install pymavlink)')

    output = subprocess.run(['ss', '-H', '-lntu'], capture_output=True, text=True).stdout
    busy = []
    for row_value in output.splitlines():
        area_value = row_value.split()
        if len(area_value) < 5:
            continue
        proto = area_value[0].lower()
        local_value = area_value[4]
        try:
            port = int(local_value.rsplit(':', 1)[1])
        except (IndexError, ValueError):
            continue
        for p_type, p_no in REQUIRED_PORTS:
            if port == p_no and proto.startswith(p_type):
                busy.append(f'{p_type}/{p_no}')
    if busy:
        raise Setup(
            'Required ports IN USE: ' + ', '.join(sorted(set(busy))) +
            '\nAnother SITL/Gazebo should be running. Turn off first:\n'
            f'  {PROJECT}/yildizlar_guidance.sh --stop\n'
            '  (or: pkill -f gzserver; pkill -f arducopter)')


def gazebo_environment():
    env = dict(os.environ)
    env['GAZEBO_MODEL_PATH'] = ':'.join([
        os.path.join(PROJECT, 'models'),
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
def gazebo_launch(env):
    argv = ['gzserver', '--verbose']
    if os.environ.get('FLIGHT_STAB_ROS') == '1':
        argv += ['-s', 'libgazebo_ros_api_plugin.so']
    argv.append(WORLD)
    proc = launch('gzserver', argv, GZ_LOG, env=env)

    write_value('  waiting for the gimbal topic (gz topic -l)...')
    finish = time.monotonic() + 120
    while time.monotonic() < finish:
        live_mi(proc, 'gzserver', GZ_LOG)
        try:
            r = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                               text=True, env=env, timeout=20)
        except subprocess.TimeoutExpired:
            continue          # Master is not there yet, eyes are hanging
        target_value = f'/gazebo/default/{MODEL}/' if BASELINE else STATUS_TOPIC
        if target_value in r.stdout:
            write_value(f'  Gazebo is ready ({target_value} appeared).')
            return proc
        time.sleep(1.0)
    raise Setup(f'{STATUS_TOPIC} 120 did not appear in s. '
                  f'gimbal plugin yuklenmemis olabilir; bak: {GZ_LOG}')


def tilt_command(env, rad):
    r = subprocess.run(['gz', 'topic', '-p', CMD_TOPIC, '-m', f'data: "{rad}"'],
                       capture_output=True, text=True, env=env, timeout=20)
    if r.returncode != 0:
        raise Setup(f'gz topic -p failed: {r.stderr.strip()}')


class StateReader:
    """It reads from a single 'gz topic -e' process that CONSTANTLY streams gimbal_tilt_status.

    Instead of periodically calling 'gz topic -e -d 1' single process: each call establishes a new
    gazebo transport connection and waits for ~1 s, 10 Hz is impossible in registration. stdbuf -oL
    CONDITION: when gz is connected to the output pipeline, the buffer will not be filled for
    minutes with block buffers and small messages of ~5 Hz.
    """

    def __init__(self, env):
        self.env = env
        self.value_value = None
        self.time_value = 0.0
        self.n = 0
        self._stop = False
        self._proc = None
        self._is = None

    def start_value2(self):
        self._proc = subprocess.Popen(
            ['stdbuf', '-oL', 'gz', 'topic', '-e', STATUS_TOPIC, '-u'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=self.env, start_new_session=True)
        _processes.append(('gz_topic_echo', self._proc))
        self._is = threading.Thread(target=self._loop, daemon=True)
        self._is.start()

    def _loop(self):
        buffer = ''
        while not self._stop:
            try:
                part = self._proc.stdout.read1(4096)
            except Exception:
                break
            if not part:
                break
            buffer += part.decode('utf-8', 'replace')
            last_value = 0
            for m in STATUS_RE.finditer(buffer):
                try:
                    v = float(m.group(1))
                except ValueError:
                    last_value = m.end()
                    continue
                self.value_value = v
                self.time_value = time.monotonic()
                self.n += 1
                last_value = m.end()
            buffer = buffer[last_value:]
            if len(buffer) > 16384:
                buffer = buffer[-1024:]

    def wait_value(self, timeout=20):
        finish = time.monotonic() + timeout
        while time.monotonic() < finish:
            if self.n > 0:
                return True
            time.sleep(0.2)
        return False

    def stop_value(self):
        self._stop = True
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                pass


# ------------------------------------------------------------------ SITL
def sitl_launch(env):
    os.makedirs(SITL_IS_DIRECTORY, exist_ok=True)
    # If eeprom.bin remains, the saved parameters from the previous run overwhelm --defaults; Let each run
    # start with the CLEAN parameter.
    for label_item in ('eeprom.bin', 'eeprom.bin.bak'):
        try:
            os.unlink(os.path.join(SITL_IS_DIRECTORY, label_item))
        except FileNotFoundError:
            pass
    argv = [SITL_BIN, '--model', 'gazebo-iris', '--speedup', '1',
            '--sysid', str(SYSID), '--slave', '0',
            '--defaults', DEFAULTS,
            '--sim-address=127.0.0.1', '-I0', '--home', HOME_POS]
    write_value('  SITL command: ' + ' '.join(argv))
    return launch('arducopter', argv, SITL_LOG, env=env, cwd=SITL_IS_DIRECTORY)


# --------------------------------------------------------------- mavlink
class Plane:
    """pymavlink wrapper: single reader thread + locked send.

    Measured lesson in repo (tools/swarm_command.py:get_position): single call to recv_match() returns the
    OLDEST message in buffer UDP/TCP; When you read one per cycle, the data becomes stale. Here a
    separate thread constantly flushes the buffer and ONLY the freshest sample is kept.
    """

    def __init__(self, address, sysid):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(address, source_system=254,
                                            source_component=190)
        self.sysid = sysid
        self.lock_value = threading.Lock()
        self._stop = False
        self.attitude = None          # (t_mono, roll, pitch, yaw) rad
        self.position_value2 = None             # (t_mono, rel_alt_m, vx, vy, vz) m, m/s
        self.heartbeat = None         # (t_mono, custom_mode, armed)
        self.acks = {}                # command_no -> (t_mono, result)
        self.statustext = []
        self._is = None

    def connect_value(self, timeout=90):
        finish = time.monotonic() + timeout
        while time.monotonic() < finish:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg is not None and msg.get_srcSystem() == self.sysid:
                self.m.target_system = self.sysid
                self.m.target_component = msg.get_srcComponent()
                self._is = threading.Thread(target=self._loop, daemon=True)
                self._is.start()
                return True
        return False

    def _loop(self):
        while not self._stop:
            with self.lock_value:
                msg = self.m.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.002)
                continue
            if msg.get_srcSystem() != self.sysid:
                continue
            t = time.monotonic()
            type_value = msg.get_type()
            if type_value == 'ATTITUDE':
                self.attitude = (t, msg.roll, msg.pitch, msg.yaw)
            elif type_value == 'GLOBAL_POSITION_INT':
                self.position_value2 = (t, msg.relative_alt / 1000.0,
                              msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
            elif type_value == 'HEARTBEAT':
                armed = bool(msg.base_mode &
                             self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.heartbeat = (t, msg.custom_mode, armed)
            elif type_value == 'GPS_RAW_INT':
                self.gps = (msg.fix_type, msg.satellites_visible)
            elif type_value == 'COMMAND_ACK':
                self.acks[msg.command] = (t, msg.result)
            elif type_value == 'STATUSTEXT':
                text_value = msg.text.strip()
                if not self.statustext or self.statustext[-1] != text_value:
                    self.statustext.append(text_value)
                    write_value(f'    [FCU] {text_value}')

    gps = (0, 0)

    def send_value(self, fn, *a, **kw):
        with self.lock_value:
            fn(*a, **kw)

    def command_value(self, command_no, *params, wait_value=True, timeout=5):
        """Sends COMMAND_LONG. recv_match is not called here because the reader thread collects the ACK
(reading from two places causes message loss)."""
        self.acks.pop(command_no, None)
        params = list(params) + [0] * (7 - len(params))
        self.send_value(self.m.mav.command_long_send, self.m.target_system,
                    self.m.target_component, command_no, 0, *params[:7])
        if not wait_value:
            return None
        finish = time.monotonic() + timeout
        while time.monotonic() < finish:
            ack = self.acks.get(command_no)
            if ack is not None:
                return ack[1]
            time.sleep(0.01)
        return None

    def flow_request(self, hz=TELEMETRY_HZ):
        interval = int(1e6 / hz)
        for msg_id in (self.mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       self.mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
            self.command_value(self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       msg_id, interval, wait_value=False)
        # Belt+love: some versions implement SET_MESSAGE_INTERVAL for certain streams only; The classic
        # request is also sent.
        self.send_value(self.m.mav.request_data_stream_send,
                    self.m.target_system, self.m.target_component,
                    self.mavutil.mavlink.MAV_DATA_STREAM_ALL, int(hz), 1)

    def mode_set(self, custom_mode):
        self.send_value(self.m.mav.set_mode_send, self.m.target_system,
                    self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode)

    def mode_wait(self, custom_mode, timeout=20):
        finish = time.monotonic() + timeout
        while time.monotonic() < finish:
            hb = self.heartbeat
            if hb and hb[1] == custom_mode:
                return True
            self.mode_set(custom_mode)
            time.sleep(0.5)
        return False

    def speed_command(self, vx, vy=0.0, vz=0.0):
        """Speed ​​setpoint in the body frame (ArduCopter returns yaw)."""
        self.send_value(self.m.mav.set_position_target_local_ned_send,
                    0, self.m.target_system, self.m.target_component,
                    MAV_FRAME_BODY_NED, TYPE_MASK_SPEED,
                    0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

    def close_value(self):
        self._stop = True
        time.sleep(0.2)
        try:
            self.m.close()
        except Exception:
            pass


# ------------------------------------------------------------- flight steps
def prearm_wait(u, timeout=180):
    finish = time.monotonic() + timeout
    while time.monotonic() < finish:
        fix, sat = u.gps
        if fix >= 3 and sat >= 6 and u.position_value2 is not None:
            return True
        time.sleep(0.5)
    return False


def arm_et(u, timeout=90):
    mavlink = u.mavutil.mavlink
    finish = time.monotonic() + timeout
    while time.monotonic() < finish:
        result_value = u.command_value(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        alt_finish = time.monotonic() + 3
        while time.monotonic() < alt_finish:
            hb = u.heartbeat
            if hb and hb[2]:
                return True
            time.sleep(0.1)
        write_value(f'    ARM rejected (result={result_value}), trying again...')
        time.sleep(2)
    return False


def takeoff_value(u, alt, timeout=120):
    mavlink = u.mavutil.mavlink
    u.command_value(mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
    finish = time.monotonic() + timeout
    last_report = 0.0
    while time.monotonic() < finish:
        k = u.position_value2
        if k is not None:
            if time.monotonic() - last_report > 2:
                write_value(f'    altitude {k[1]:5.1f} m')
                last_report = time.monotonic()
            if k[1] >= alt * 0.97:
                return True
        time.sleep(0.2)
    return False


def convergence_wait(u, alt, timeout=40):
    """Vertical speed and waits until pitch settles: measurement starts from ZERO."""
    finish = time.monotonic() + timeout
    fixed_start = None
    while time.monotonic() < finish:
        k, a = u.position_value2, u.attitude
        if k and a:
            steady = (abs(k[4]) < 0.5 and math.hypot(k[2], k[3]) < 1.0
                     and abs(math.degrees(a[2])) < 2.0 and abs(k[1] - alt) < 2.0)
            if steady:
                if fixed_start is None:
                    fixed_start = time.monotonic()
                elif time.monotonic() - fixed_start > 3.0:
                    return True
            else:
                fixed_start = None
        time.sleep(0.2)
    return False


# ------------------------------------------------------------------ measurement
def p95(array_value):
    if not array_value:
        return float('nan')
    s = sorted(array_value)
    i = min(len(s) - 1, int(round(0.95 * (len(s) - 1))))
    return s[i]


def summary_row(label_item, array_value, unit_value='deg'):
    if not array_value:
        return f'  {label_item:26s} NO SAMPLE'
    return (f'  {label_item:26s} min={min(array_value):7.2f}  mean={statistics.median(array_value):7.2f}  '
            f'max={max(array_value):7.2f}  ({unit_value})')


def measurement_loop(u, state_value):
    """10 Hz register: acceleration phase (+ 15 m/s ) + braking phase ( 0 m/s )."""
    rows_value = []
    t0 = time.monotonic()
    total_value = ACCELERATION_DURATION + BRAKING_DURATION
    period_value = 1.0 / RECORD_HZ
    next_value = t0
    write_value(f'  phase ACCELERATION: {FORWARD_SPEED} m/s body +X, {ACCELERATION_DURATION} s')
    phase_written = 'acceleration_value2'
    while True:
        now_value = time.monotonic()
        t = now_value - t0
        if t >= total_value:
            break
        phase_value = 'acceleration_value2' if t < ACCELERATION_DURATION else 'braking'
        if phase_value != phase_written:
            write_value(f'  phase BRAKE: 0 m/s, {BRAKING_DURATION} s')
            phase_written = phase_value
        # GUIDED speed target times out at ~3 s: refreshed every frame.
        u.speed_command(FORWARD_SPEED if phase_value == 'acceleration_value2' else 0.0)

        a, k = u.attitude, u.position_value2
        pitch = math.degrees(a[2]) if a else float('nan')
        roll = math.degrees(a[1]) if a else float('nan')
        ground_speed = math.hypot(k[2], k[3]) if k else float('nan')
        alt = k[1] if k else float('nan')
        st = state_value.value_value
        st_deg = math.degrees(st) if st is not None else float('nan')
        st_age = (now_value - state_value.time_value) * 1000.0 if state_value.time_value else float('nan')
        # Two comments: A is correct if the status is WORLD pitch, and B is the JOINT angle. In the new
        # (stabilized=1) plugin version, A is valid.
        comment_b = pitch + st_deg
        rows_value.append(dict(t=t, phase_value=phase_value, ground_speed=ground_speed, alt=alt,
                             pitch=pitch, roll=roll, status_deg=st_deg,
                             status_age_ms=st_age, world_commentA=st_deg,
                             world_commentB=comment_b, n_sample=state_value.n))
        next_value += period_value
        sleep_item = next_value - time.monotonic()
        if sleep_item > 0:
            time.sleep(sleep_item)
        else:
            next_value = time.monotonic()
    u.speed_command(0.0)
    return rows_value


def csv_write(rows_value):
    headers_value = ['t_s', 'phase_value', 'ground_speed_ms', 'altitude_m', 'body_pitch_deg',
                 'body_roll_deg', 'status_deg', 'status_yasi_ms',
                 'camera_world_pitch_A_deg', 'camera_world_pitch_B_deg',
                 'status_sample_no']
    with open(CSV_PATH, 'w') as f:
        f.write(','.join(headers_value) + '\n')
        for s in rows_value:
            f.write('{t:.3f},{phase_value},{ground_speed:.3f},{alt:.2f},{pitch:.3f},'
                    '{roll:.3f},{status_deg:.4f},{status_age_ms:.1f},'
                    '{world_commentA:.4f},{world_commentB:.4f},{n_sample}\n'.format(**s))
    write_value(f'\nCSV: {CSV_PATH}')


def summary_start(rows_value, state_value):
    if not rows_value:
        write_value('\n SUMMARY: No records were obtained.')
        return 1
    acceleration_value2 = [s for s in rows_value if s['phase_value'] == 'acceleration_value2']
    pitches = [s['pitch'] for s in rows_value if not math.isnan(s['pitch'])]
    acceleration_pitch = [s['pitch'] for s in acceleration_value2 if not math.isnan(s['pitch'])]
    statuses = [s['status_deg'] for s in rows_value if not math.isnan(s['status_deg'])]
    commentB = [s['world_commentB'] for s in rows_value if not math.isnan(s['world_commentB'])]
    speeds = [s['ground_speed'] for s in rows_value if not math.isnan(s['ground_speed'])]
    ages = [s['status_age_ms'] for s in rows_value if not math.isnan(s['status_age_ms'])]

    write_value('\n' + '=' * 66)
    write_value('SUMMARY')
    write_value('=' * 66)
    write_value(f'  record line {len(rows_value)} | unique gimbal sample {state_value.n}  '
        f'(~{state_value.n / max(1e-6, rows_value[-1]["t"]):.1f} Hz)')
    write_value(summary_row('body pitch', pitches))
    write_value(summary_row('camera world pitch (status)', statuses))
    write_value(summary_row('ground speed', speeds, 'm/s'))
    if ages:
        write_value(f'  {"status sample_value yasi":26s} median={statistics.median(ages):6.0f} ms  '
            f'p95={p95(ages):6.0f} ms')

    acceleration_peak = max((abs(p) for p in acceleration_pitch), default=0.0)
    all_peak = max((abs(p) for p in pitches), default=0.0)
    status_p95 = p95([abs(s) for s in statuses])
    status_max = max((abs(s) for s in statuses), default=float('nan'))
    commentB_p95 = p95([abs(s) for s in commentB])

    write_value('')
    write_value(f'  |body pitch| peak (acceleration phase) = {acceleration_peak:6.2f} deg  '
        f'(threshold >= {PITCH_PEAK_THRESHOLD} )')
    write_value(f'  |body pitch| peak (all record) = {all_peak:6.2f} deg')
    write_value(f'  |camera world pitch| p95 = {status_p95:6.2f} deg  '
        f'(threshold < {STATUS_P95_THRESHOLD} )')
    write_value(f'  |camera world pitch| max = {status_max:6.2f} deg')
    write_value('')
    write_value('  COMMENT KONTROLU (status ne yayinliyor?)')
    write_value(f'    A) status = camera WORLD pitch\'i -> |A| p95 = {status_p95:6.2f} deg')
    write_value(f'    B) stats = JOINT angle (body+joint) -> |B| p95 = {commentB_p95:6.2f} deg')
    if status_p95 < commentB_p95:
        write_value('    -> A consistent: status stabilized (WORLD pitch \' i) is broadcasting. [expected]')
    else:
        write_value('    -> B consistent: status still broadcasts JOINT angle '
            '(Is there <stabilize>1</stabilize> in gimbal_small_2d model.sdf?)')

    valid_value = acceleration_peak >= PITCH_PEAK_THRESHOLD
    stable = status_p95 < STATUS_P95_THRESHOLD
    write_value('')
    if not valid_value:
        write_value(f'RESULT: INVALID - body pitch {acceleration_peak:.2f} deg, {PITCH_PEAK_THRESHOLD} deg '
            'remained below the threshold. The hopper did not accelerate sufficiently; FORWARD_SPEED / '
            'ACCELERATION_DURATION should be increased or WPNAV_ACCEL should be checked.')
        return 1
    write_value('RESULT: ' + ('PASS - camera world pitch\'i remained stationary while the body was tilted.'
                     if stable else
                     'FAULT - camera world pitch\'i slides with the body.'))
    return 0 if stable else 1


# ------------------------------------------------------------------- main
def main():
    write_value('=' * 66)
    write_value('FLIGHT STABILIZATION TEST: forward speed -> fuselage pitch -> gimbal')
    write_value('=' * 66)
    write_value('\n[1] ten controls (dependencies + ports)')
    on_control()
    write_value('  Ok.')

    env = gazebo_environment()
    state_value = StateReader(env)
    u = None
    try:
        write_value('\n[2] gzserver')
        gz_proc = gazebo_launch(env)

        write_value('\n[3] ArduCopter SITL')
        sitl_proc = sitl_launch(env)

        write_value(f'\n[4] MAVLink connection ({MAVLINK_ADDRESS}, SysID {SYSID})')
        # Connected trial until SITL slave0 TCP listener opens.
        u = None
        finish = time.monotonic() + 90
        while time.monotonic() < finish:
            live_mi(sitl_proc, 'arducopter', SITL_LOG)
            live_mi(gz_proc, 'gzserver', GZ_LOG)
            try:
                u = Plane(MAVLINK_ADDRESS, SYSID)
            except Exception:
                time.sleep(1.0)
                continue
            if u.connect_value(timeout=20):
                break
            u.close_value()
            u = None
        if u is None:
            raise Setup(f'{MAVLINK_ADDRESS} uzerinden SysID {SYSID} heartbeat '
                          f'alinamadi. Bak: {SITL_LOG}')
        write_value('  heartbeat ok.')
        u.flow_request()

        write_value('\n[5] EKF / GPS hazirligi')
        if not prearm_wait(u):
            raise Setup('GPS/EKF was not ready (prearm timeout)')
        write_value(f'  GPS fix={u.gps[0]} uydu={u.gps[1]}')

        write_value('\n[6] GUIDED + ARM')
        if not u.mode_wait(COPTER_GUIDED):
            raise Setup('GUIDED moduna gecilemedi')
        write_value('  GUIDED.')
        if not arm_et(u):
            raise Setup('ARM edilemedi')
        write_value('  ARM.')

        write_value(f'\n [ 7 ] {TAKEOFF_ALT:.0f} m departure')
        if not takeoff_value(u, TAKEOFF_ALT):
            raise Setup(f'{TAKEOFF_ALT} m irtifaya cikilamadi')
        if not convergence_wait(u, TAKEOFF_ALT):
            write_value('  WARNING: complete fixation not achieved, still continuing.')
        else:
            write_value('  hanging and calm.')

        write_value('\n[8] gimbal 0.0 rad (look at horizon) + status stream')
        if BASELINE:
            write_value('  BASELINE mode: no gimbal, status will be skipped.')
        else:
            state_value.start_value2()
        for _ in range(0 if BASELINE else 3):
            tilt_command(env, 0.0)
            time.sleep(0.4)
        if not BASELINE and not state_value.wait_value(timeout=25):
            # Distinguish two failures: (a) the plugin does not stream at all, (b) it streams but the output of
            # the streaming process is buffered.
            try:
                single = subprocess.run(
                    ['gz', 'topic', '-e', STATUS_TOPIC, '-d', '3', '-u'],
                    capture_output=True, text=True, env=env, timeout=15).stdout
            except subprocess.TimeoutExpired:
                single = ''
            if STATUS_RE.search(single):
                raise Setup(
                    'One shot "gz topic -e -d 3" RETURNed the example but continuously '
                    'stream empty: output buffering problem (if stdbuf -oL '
                    'did not work). Solution: `script -qfc` instead of gz version/stdbuf '
                    'Use psodo-terminal with .')
            raise Setup(
                f'No samples came through {STATUS_TOPIC} (also continuous streaming, '
                'one shot is also empty).\nPossible cause: gimbal plugin is not installed or '
                'status yayinlamiyor.\n'
                f'Verify manually: gz topic -l | grep gimbal ; '
                f'gz topic -e {STATUS_TOPIC} -d 3')
        if not BASELINE:
            time.sleep(4.0)   # To fit PID
            write_value(f'  status = {math.degrees(state_value.value_value):+.3f} deg '
                f'({state_value.n} taken as an example)')

        write_value(f'\n[9] measurement: {FORWARD_SPEED} m/s forward, {RECORD_HZ:.0f} Hz record')
        rows_value = measurement_loop(u, state_value)
        live_mi(sitl_proc, 'arducopter', SITL_LOG)
        live_mi(gz_proc, 'gzserver', GZ_LOG)

        csv_write(rows_value)
        return summary_start(rows_value, state_value)

    finally:
        state_value.stop_value()
        if u is not None:
            u.close_value()
        all_temizle()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Setup as e:
        print(f'\nINSTALLATION ERROR: {e}', file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print('\nkesildi.', file=sys.stderr)
        sys.exit(130)
