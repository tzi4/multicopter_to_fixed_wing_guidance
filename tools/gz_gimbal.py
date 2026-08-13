#!/usr/bin/env python3
"""Fiziksel gimbal <-> python koprusu (gimbal dali, 2026-08-05).

Gazebo classic'te python transport binding'i yok; tek yol `gz` CLI.
Iki olculmus tuzak (NOTLAR_GIMBAL.md):
  * `gz topic -e`yi periyodik cagirmak olmaz: her cagri yeni transport
    baglantisi = ~1 s. TEK surekli surec + arka plan thread kullan.
  * stdbuf -oL SART: gz, boru hattina blok tamponlar; dusuk hizli status
    akisinda tampon dakikalarca bosalmaz.
  * `gz topic -p` de surec basina ~1 s oder -> komutcu thread'i ana
    donguyu ASLA bloke etmemeli (TiltKomutcu bunun icin var).

Topic sozlesmesi (plugin ust modele baglanir, ad dunyadaki sarmalayicidan):
  komut : /gazebo/default/<model>/gimbal_tilt_cmd   (rad, GzString)
          = kameranin DUNYA elevasyonu, pozitif = yukari (stabilize mod)
  durum : /gazebo/default/<model>/gimbal_tilt_status (rad, ~18 Hz olculdu)
"""

import os
import re
import signal
import subprocess
import threading
import time

_STATUS_RE = re.compile(r'data:\s*"(-?[\d.eE+-]+)"')


def komut_topic(model):
    return f'/gazebo/default/{model}/gimbal_tilt_cmd'


def durum_topic(model):
    return f'/gazebo/default/{model}/gimbal_tilt_status'


def model_adi_topikten(ros_topic):
    """'/drone_3/webcam/image_raw' -> 'iris-3' (worlds/*.world sarmalayici
    adlari). Eslesmezse None."""
    m = re.match(r'^/drone_(\d+)/', str(ros_topic))
    return f'iris-{m.group(1)}' if m else None


def tilt_komut(model, rad, env=None, timeout=20):
    """TEK SEFERLIK yayin. ~1 s bloke eder; sicak dongulerden cagirma,
    TiltKomutcu kullan."""
    r = subprocess.run(['gz', 'topic', '-p', komut_topic(model),
                        '-m', f'data: "{float(rad)}"'],
                       capture_output=True, text=True,
                       env=env, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(f'gz topic -p basarisiz: {r.stderr.strip()}')


class TiltDurumOkuyucu:
    """gimbal_tilt_status'u tek surekli `gz topic -e` sureciyle okur."""

    def __init__(self, model, env=None):
        self.model = model
        self.env = env
        self.deger_rad = None
        self.zaman = 0.0          # monotonic; tazelik = time.monotonic()-zaman
        self.n = 0
        self._dur = False
        self._proc = None

    def basla(self):
        self._proc = subprocess.Popen(
            ['stdbuf', '-oL', 'gz', 'topic', '-e', durum_topic(self.model), '-u'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env=self.env, start_new_session=True)
        threading.Thread(target=self._dongu, daemon=True).start()
        return self

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
            for m in _STATUS_RE.finditer(tampon):
                try:
                    self.deger_rad = float(m.group(1))
                except ValueError:
                    son = m.end()
                    continue
                self.zaman = time.monotonic()
                self.n += 1
                son = m.end()
            tampon = tampon[son:]
            if len(tampon) > 16384:
                tampon = tampon[-1024:]

    def yas_s(self):
        return None if self.n == 0 else time.monotonic() - self.zaman

    def dur(self):
        self._dur = True
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                pass


def _kalici_yayinci_yolu():
    ap_gz = os.environ.get('ARDUPILOT_GAZEBO_DIR',
                           os.path.expanduser('~/ardupilot_gazebo'))
    yol = os.path.join(ap_gz, 'build', 'gz_tilt_pub')
    return yol if os.access(yol, os.X_OK) else None


class TiltKomutcu:
    """Tilt hedefini arka planda yayinlar; ana donguyu bloke etmez.

    IKI ARKA UC (Faz C, 2026-08-06):
      * KALICI: gz_tilt_pub (gimbal_kurulum, C++). Transport baglantisi
        bir kez kurulur, sonraki her yayin ~ms. Faz C'nin dinamik tilt
        takibi bununla mumkun (terminalde eps 40-70 deg/s degisebilir).
      * YEDEK: `gz topic -p` (surec basina ~1 s). gz_tilt_pub derlenmemis
        makinelerde Faz A/B davranisi korunur; min_aralik otomatik 1 s'e
        cekilir.
    Olu bant + tazeleme her iki arka ucta da gecerli (tazeleme: gzserver
    yeniden baslarsa komut kaybolmasin).
    """

    def __init__(self, model, env=None, olu_bant_deg=0.1, min_aralik_s=None,
                 tazeleme_s=30.0):
        self.model = model
        self.env = env
        self.olu_bant = float(olu_bant_deg)
        self.tazeleme = float(tazeleme_s)
        self.hedef_deg = None      # istenen elevasyon [deg]
        self.yayinlanan_deg = None
        self.son_yayin_t = 0.0
        self.hata_n = 0
        self.kalici = None         # Popen(gz_tilt_pub) ya da None
        self._dur = False
        self._uyandir = threading.Event()
        self._is = threading.Thread(target=self._dongu, daemon=True)
        self._yayinci_yolu = _kalici_yayinci_yolu()
        if min_aralik_s is None:
            min_aralik_s = 0.03 if self._yayinci_yolu else 1.0
        self.min_aralik = float(min_aralik_s)

    def basla(self):
        if self._yayinci_yolu:
            try:
                self.kalici = subprocess.Popen(
                    [self._yayinci_yolu, self.model],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE, env=self.env, text=True,
                    bufsize=1, start_new_session=True)
                # el sikisma: WaitForConnection tamamlaninca "HAZIR" basar
                hazir = threading.Event()

                def _bekle():
                    for satir in self.kalici.stderr:
                        if 'HAZIR' in satir:
                            hazir.set()
                            break
                threading.Thread(target=_bekle, daemon=True).start()
                if not hazir.wait(timeout=20):
                    self.kalici.kill()
                    self.kalici = None
            except Exception:
                self.kalici = None
        if self.kalici is None:
            self.min_aralik = max(self.min_aralik, 1.0)
        self._is.start()
        return self

    def hedef(self, deg):
        self.hedef_deg = float(deg)
        self._uyandir.set()

    def _yayinla(self, h_deg):
        import math
        if self.kalici is not None and self.kalici.poll() is None:
            self.kalici.stdin.write(f"{math.radians(h_deg):.6f}\n")
            self.kalici.stdin.flush()
        else:
            if self.kalici is not None:      # kalici yayinci oldu: yedege dus
                self.kalici = None
                self.min_aralik = max(self.min_aralik, 1.0)
                self.hata_n += 1
            tilt_komut(self.model, math.radians(h_deg), env=self.env)

    def _dongu(self):
        while not self._dur:
            self._uyandir.wait(timeout=1.0)
            self._uyandir.clear()
            h = self.hedef_deg
            if h is None:
                continue
            simdi = time.monotonic()
            degisti = (self.yayinlanan_deg is None
                       or abs(h - self.yayinlanan_deg) > self.olu_bant)
            bayat = simdi - self.son_yayin_t > self.tazeleme
            if not (degisti or bayat):
                continue
            if simdi - self.son_yayin_t < self.min_aralik:
                # yakinda tekrar dene (kalici moddaki hizli guncellemeler
                # icin uyandirma bekletilir, kacirilmaz)
                self._uyandir.set()
                time.sleep(self.min_aralik / 2.0)
                continue
            try:
                self._yayinla(h)
                self.yayinlanan_deg = h
                self.son_yayin_t = simdi
            except Exception:
                self.hata_n += 1
                time.sleep(1.0)

    def dur(self):
        self._dur = True
        self._uyandir.set()
        if self.kalici is not None and self.kalici.poll() is None:
            try:
                self.kalici.stdin.close()
            except Exception:
                pass
            try:
                os.killpg(os.getpgid(self.kalici.pid), signal.SIGTERM)
            except Exception:
                pass


class TiltTakip:
    """FAZ C: tilt hedefini HEDEFIN OLCULEN YUKSELISINE surer.

    Girdi ey (stab dikey hata, ufka gore; ey<0 = hedef ufkun ustunde)
    zaten canli-eklem zinciriyle DUNYA cercevesinde olculdugu icin hedef
    yukselisi = -ey + mevcut_sanal_merkez... aim=0'da duz: e_t = -ey +
    0 => e_t = -ey. DIKKAT: ey tilt'ten BAGIMSIZDIR (ufka gore olculur),
    yani bu bir geri-besleme dongusu degil, olculen buyuklugun suzgecli
    TAKIBIdir -- kararlilik riski yok, yalniz gecikme var.

    Koruma katmanlari (AimTrim'in dersleri + Faz C gercekleri):
      * EMA suzgec (tau): tek karelik tespit gurultusu tilt'i surmesin
      * slew siniri: fiziksel servo 6 rad/s; komut ondan yavas kalsin
      * kelepce [alt, ust]: yer/gok taramasina karsi
      * kayip tutma: tespit dususse son hedef tutulur (kayip_tut_s),
        sonra varsayilan (standoff) acisina YAVASCA doner -- yeniden
        edinim pozu
    """

    def __init__(self, varsayilan_deg, tau_s=0.4, slew_dps=60.0,
                 alt_deg=-30.0, ust_deg=60.0, kayip_tut_s=3.0,
                 donus_dps=10.0):
        self.varsayilan = float(varsayilan_deg)
        self.tau = float(tau_s)
        self.slew = float(slew_dps)
        self.alt, self.ust = float(alt_deg), float(ust_deg)
        self.kayip_tut = float(kayip_tut_s)
        self.donus = float(donus_dps)
        self.cmd = float(varsayilan_deg)
        self._suzgec = None
        self._son_tespit_t = None

    def guncelle(self, hedef_elev_deg, dt, simdi=None):
        """hedef_elev_deg: hedefin olculen dunya yukselisi (= -ey), tespit
        yoksa None. Yeni tilt komutunu [deg] dondurur."""
        if simdi is None:
            simdi = time.monotonic()
        dt = max(1e-3, min(float(dt), 0.5))
        if hedef_elev_deg is not None:
            self._son_tespit_t = simdi
            if self._suzgec is None:
                self._suzgec = float(hedef_elev_deg)
            else:
                k = dt / (dt + self.tau)
                self._suzgec += k * (float(hedef_elev_deg) - self._suzgec)
            istenen, hiz = self._suzgec, self.slew
        elif (self._son_tespit_t is not None
              and simdi - self._son_tespit_t < self.kayip_tut):
            return self.cmd                        # tut
        else:
            istenen, hiz = self.varsayilan, self.donus   # yeniden edinim
            self._suzgec = None
        adim = max(-hiz * dt, min(hiz * dt, istenen - self.cmd))
        self.cmd = max(self.alt, min(self.ust, self.cmd + adim))
        return self.cmd
