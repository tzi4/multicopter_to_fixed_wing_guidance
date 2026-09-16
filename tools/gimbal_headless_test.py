#!/usr/bin/env python3
"""Vertical gimbal headless validation test (gimbal branch).

Removes own gzserver + roscore process, verifies two layers: 1. PHYSICS: gimbal_tilt_cmd -> does the
joint angle follow the command (status topic) 2. OPTICAL: does the red reference box placed in the
frame shift vertically with tilt (tilt up -> the box moves down in the frame; quantitative: 0.25 rad
~ 250 px @720p)

Usage: python3 tools/gimbal_headless_test.py Exit code 0 = PASS. ROS will reboot itself if the
environment is not sourced.

IMPORTANT: keep the camera subscription active. Repeated single-frame capture and release can freeze
gazebo_ros camera lazy rendering and return stale frames (measured on 2026-08-05).
"""
import os, sys, re, time, signal, subprocess, tempfile

if 'ROS_DISTRO' not in os.environ:
    os.execvp('bash', ['bash', '-lc',
        'source /opt/ros/noetic/setup.bash && exec python3 "$1" "${@:2}"',
        'bash', os.path.abspath(__file__), *sys.argv[1:]])

import numpy as np
import rospy
from sensor_msgs.msg import Image

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AP_GZ = os.environ.get('ARDUPILOT_GAZEBO_DIR', os.path.expanduser('~/ardupilot_gazebo'))
IQ_SIM = os.path.expanduser('~/catkin_ws/src/iq_sim/models')
MODEL = 'iris-1'          # worlds/ single_pursuer Wrapper model name in .world
CMD = f'/gazebo/default/{MODEL}/gimbal_tilt_cmd'
STATUS = f'/gazebo/default/{MODEL}/gimbal_tilt_status'

BOX = '''
    <model name="kirmizi_box">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><box><size>1 2 1</size></box></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
      </link>
      <pose>5 0 0.14 0 0 0</pose>
    </model>
  </world>'''

last_value = {"img": None, "n": 0}
def _cb(m):
    last_value["img"] = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
    last_value["n"] += 1

def gz(*args):
    return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=30)

def tilt_command(a):
    r = gz('topic', '-p', CMD, '-m', f'data: "{a}"')
    if r.returncode != 0:
        raise RuntimeError(f'gz topic -p failed: {r.stderr}')

def joint_angle():
    r = subprocess.run(['timeout', '6', 'gz', 'topic', '-e', STATUS, '-d', '1', '-u'],
                       capture_output=True, text=True)
    m = re.search(r'data: "([-\d.e]+)"', r.stdout)
    return float(m.group(1)) if m else None

def box_row():
    img = last_value["img"]
    if img is None:
        return 0, None
    im = img.astype(int); r, g, b = im[:, :, 0], im[:, :, 1], im[:, :, 2]
    ys, _ = np.nonzero((r > 60) & (g < 50) & (b < 50))
    return len(ys), (float(ys.mean()) if len(ys) else None)

def main():
    world = open(os.path.join(PROJECT, 'worlds', 'single_pursuer.world')).read()
    tmp = tempfile.NamedTemporaryFile('w', suffix='.world', delete=False)
    tmp.write(world.replace('</world>', BOX, 1)); tmp.close()

    env = dict(os.environ)
    env['GAZEBO_MODEL_PATH'] = ':'.join([
        os.path.join(PROJECT, 'models'), os.path.join(AP_GZ, 'models'),
        IQ_SIM, '/usr/share/gazebo-11/models'])
    env['GAZEBO_PLUGIN_PATH'] = os.path.join(AP_GZ, 'build') + \
        ':/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    increasing = []
    try:
        if subprocess.run(['bash', '-c', 'rostopic list'], capture_output=True).returncode != 0:
            increasing.append(subprocess.Popen(['roscore'], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid))
            time.sleep(5)
        increasing.append(subprocess.Popen(
            ['gzserver', '-s', 'libgazebo_ros_api_plugin.so', tmp.name], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid))

        rospy.init_node('gimbal_headless_test', anonymous=True)
        rospy.Subscriber('/drone_1/webcam/image_raw', Image, _cb, queue_size=1)
        t0 = time.time()
        while last_value["n"] < 5 and time.time() - t0 < 90:
            time.sleep(0.5)
        if last_value["n"] < 5:
            print('PERPETRATOR: camera feed did not arrive'); return 1
        print(f'camera streaming (n={last_value["n"]})')

        error_value = 0
        rows_value = {}
        for a in (0.0, 0.25, -0.25):
            tilt_command(a); time.sleep(6)
            measure = joint_angle()
            n1 = last_value["n"]; time.sleep(1.0)
            pixel, row_value = box_row()
            flow = 'AKIYOR' if last_value["n"] > n1 else 'FROZEN'
            print(f'tilt={a:+.2f}: joint={measure} square={flow} box_row={row_value} ({pixel}px)')
            if measure is None or abs(measure - a) > 0.06:
                print(f'  PHYSICS ERROR: joint {measure} != {a}'); error_value = 1
            if flow == 'FROZEN':
                print('  RENDER ERROR: frame streaming stopped'); error_value = 1
            rows_value[a] = row_value

        s0, s_up, s_dn = rows_value[0.0], rows_value[0.25], rows_value[-0.25]
        if s0 is None:
            print('OPTICAL ERROR: no box at plain view'); error_value = 1
        else:
            if not ((s_up is None or s_up > s0 + 100) and (s_dn is None or s_dn < s0 - 100)):
                print(f'OPTICAL ERROR: wrong shift direction/amount (should be {s_dn} < {s0} < {s_up})')
                error_value = 1
        print('RESULT:', 'PASS' if error_value == 0 else 'FAIL')
        return error_value
    finally:
        for p in increasing:
            try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception: pass
        os.unlink(tmp.name)

if __name__ == '__main__':
    sys.exit(main())
