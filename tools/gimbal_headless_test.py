#!/usr/bin/env python3
"""Dikey gimbal headless dogrulama testi (gimbal branch).

Kendi gzserver + roscore surecini kaldirir, iki katman dogrular:
  1. FIZIK : gimbal_tilt_cmd -> eklem acisi komutu izliyor mu (status topic)
  2. OPTIK : kadraja konan kirmizi referans kutusu tilt ile dikeyde kayiyor mu
     (yukari tilt -> kutu kadrajda asagi iner; nicel: 0.25 rad ~ 250 px @720p)

Kullanim:  python3 tools/gimbal_headless_test.py
Cikis kodu 0 = PASS. ROS ortami sourcelanmamissa kendini yeniden baslatir.

ONEMLI: kamera aboneligi SUREKLI tutulur; tek kare cek-birak dongusu
gazebo_ros kamerasinin tembel render'ini dondurup bayat kare dondurebiliyor
(2026-08-05'te olculdu).
"""
import os, sys, re, time, signal, subprocess, tempfile

if 'ROS_DISTRO' not in os.environ:
    os.execvp('bash', ['bash', '-lc',
        'source /opt/ros/noetic/setup.bash && exec python3 "$1" "${@:2}"',
        'bash', os.path.abspath(__file__), *sys.argv[1:]])

import numpy as np
import rospy
from sensor_msgs.msg import Image

PROJE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AP_GZ = os.environ.get('ARDUPILOT_GAZEBO_DIR', os.path.expanduser('~/ardupilot_gazebo'))
IQ_SIM = os.path.expanduser('~/catkin_ws/src/iq_sim/models')
MODEL = 'iris-1'          # worlds/tek_avci.world icindeki sarmalayici model adi
CMD = f'/gazebo/default/{MODEL}/gimbal_tilt_cmd'
STATUS = f'/gazebo/default/{MODEL}/gimbal_tilt_status'

KUTU = '''
    <model name="kirmizi_kutu">
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

son = {"img": None, "n": 0}
def _cb(m):
    son["img"] = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
    son["n"] += 1

def gz(*args):
    return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=30)

def tilt_komut(a):
    r = gz('topic', '-p', CMD, '-m', f'data: "{a}"')
    if r.returncode != 0:
        raise RuntimeError(f'gz topic -p basarisiz: {r.stderr}')

def eklem_acisi():
    r = subprocess.run(['timeout', '6', 'gz', 'topic', '-e', STATUS, '-d', '1', '-u'],
                       capture_output=True, text=True)
    m = re.search(r'data: "([-\d.e]+)"', r.stdout)
    return float(m.group(1)) if m else None

def kutu_satiri():
    img = son["img"]
    if img is None:
        return 0, None
    im = img.astype(int); r, g, b = im[:, :, 0], im[:, :, 1], im[:, :, 2]
    ys, _ = np.nonzero((r > 60) & (g < 50) & (b < 50))
    return len(ys), (float(ys.mean()) if len(ys) else None)

def main():
    dunya = open(os.path.join(PROJE, 'worlds', 'tek_avci.world')).read()
    tmp = tempfile.NamedTemporaryFile('w', suffix='.world', delete=False)
    tmp.write(dunya.replace('</world>', KUTU, 1)); tmp.close()

    env = dict(os.environ)
    env['GAZEBO_MODEL_PATH'] = ':'.join([
        os.path.join(PROJE, 'models'), os.path.join(AP_GZ, 'models'),
        IQ_SIM, '/usr/share/gazebo-11/models'])
    env['GAZEBO_PLUGIN_PATH'] = os.path.join(AP_GZ, 'build') + \
        ':/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    acilan = []
    try:
        if subprocess.run(['bash', '-c', 'rostopic list'], capture_output=True).returncode != 0:
            acilan.append(subprocess.Popen(['roscore'], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid))
            time.sleep(5)
        acilan.append(subprocess.Popen(
            ['gzserver', '-s', 'libgazebo_ros_api_plugin.so', tmp.name], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid))

        rospy.init_node('gimbal_headless_test', anonymous=True)
        rospy.Subscriber('/drone_1/webcam/image_raw', Image, _cb, queue_size=1)
        t0 = time.time()
        while son["n"] < 5 and time.time() - t0 < 90:
            time.sleep(0.5)
        if son["n"] < 5:
            print('FAIL: kamera yayini gelmedi'); return 1
        print(f'kamera akiyor (n={son["n"]})')

        hata = 0
        satirlar = {}
        for a in (0.0, 0.25, -0.25):
            tilt_komut(a); time.sleep(6)
            olc = eklem_acisi()
            n1 = son["n"]; time.sleep(1.0)
            piksel, satir = kutu_satiri()
            akis = 'AKIYOR' if son["n"] > n1 else 'DONUK'
            print(f'tilt={a:+.2f}: eklem={olc} kare={akis} kutu_satiri={satir} ({piksel}px)')
            if olc is None or abs(olc - a) > 0.06:
                print(f'  FIZIK HATA: eklem {olc} != {a}'); hata = 1
            if akis == 'DONUK':
                print('  RENDER HATA: kare akisi durdu'); hata = 1
            satirlar[a] = satir

        s0, s_up, s_dn = satirlar[0.0], satirlar[0.25], satirlar[-0.25]
        if s0 is None:
            print('OPTIK HATA: duz bakista kutu yok'); hata = 1
        else:
            if not ((s_up is None or s_up > s0 + 100) and (s_dn is None or s_dn < s0 - 100)):
                print(f'OPTIK HATA: kayma yonu/miktari yanlis ({s_dn} < {s0} < {s_up} olmali)')
                hata = 1
        print('SONUC:', 'PASS' if hata == 0 else 'FAIL')
        return hata
    finally:
        for p in acilan:
            try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception: pass
        os.unlink(tmp.name)

if __name__ == '__main__':
    sys.exit(main())
