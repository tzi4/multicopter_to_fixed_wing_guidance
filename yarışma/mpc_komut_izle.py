#!/usr/bin/env python3
"""SALT IZLEME: MPC veya LOS/PN komut uretimi + sanal gimbal bench testi.

GERCEK DRONE UZERINDE GUVENLI TEST icin yazildi (2026-08-09):
  * MAVLink'e HICBIR komut GONDERILMEZ -- bu dosyada gonderme yolu YOKTUR
    (HizKomutcu import edilmez, set_position_target / arm / mode cagrisi yok).
    Tek MAVLink yazisi SET_MESSAGE_INTERVAL istegidir (ATTITUDE +
    LOCAL_POSITION_NED akis hizi; araci hareket ettiremez).
  * Secilen kontrolcu IMPORT edilir, kopyalanmaz -- ucusta kosacak yasanin
    TA KENDISI test edilir. LOS secilirse MPC/optimizer yuklenmez.
  * Angajman kapisi BEKLENMEZ: bbox geldigi surece yasa her dongude kosar
    ve uretecegi komut EKRANA basilir.

EKRANDA NE VAR (hepsi yalniz HESAPLANAN degerler):
  ham  ex/ey : tracker_bbox (AI ham bbox) merkezinden pinhole ile — sanal
               gimbal DUZELTMESIZ aci hatasi
  sanal ex/ey: tracker_bbox_stab — sanal gimbalin (su an yalniz ROLL
               de-rotasyonu; pitch fiziksel tilt gimbalda) urettigi hata
  v_ned      : yasanin uretecegi hiz komutu [m/s, NED] + buyuklugu
  yaw        : yasanin uretecegi yaw hizi [dps]
  r/durum    : kontrolcunun ic menzil/faz durumu

ON KOSUL: tespit zinciri calisiyor olmali (raspberry_cam_ai.py +
donanim/bbox_to_redis.py) ve kamera baska surecte OLMAMALI
(gimbal_bench_takip.py kapatilmis olmali).

KULLANIM:
  python3 tools/mpc_komut_izle.py                       # varsayilan 14550
  python3 tools/mpc_komut_izle.py --yasa los --menzil-m 20
  python3 tools/mpc_komut_izle.py --no-mavlink          # tutum/hiz olmadan
Cikis: Ctrl-C.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

_KOK = Path(__file__).resolve().parent.parent
_GA_ADAYLARI = (_KOK / 'donanim' / 'guidance_allstar',
                _KOK / 'guidance_allstar')
_GA = next((p for p in _GA_ADAYLARI if p.is_dir()), _GA_ADAYLARI[0])
for _p in (str(_GA), str(_KOK / 'donanim')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import redis                                                     # noqa: E402
from pymavlink import mavutil                                    # noqa: E402
import mavlink_utils                                             # noqa: E402
from goruntulu_temel import (BboxOkuyucu, Olcum,                 # noqa: E402
                             _KADRAJ_FX, _KADRAJ_W, _KADRAJ_H)
from terminal_los_gudum import TerminalLosKontrolcu              # noqa: E402


class SaltOkurTelemetri:
    """Avci tutum/konum/hiz OKUYUCUSU. Komut gonderme metodu BILEREK yok."""

    def __init__(self, baglanti):
        self.conn = mavutil.mavlink_connection(baglanti, source_system=249)
        print(f"[izle] MAVLink heartbeat bekleniyor: {baglanti} ...")
        self.conn.wait_heartbeat(timeout=30)
        print(f"[izle] heartbeat alindi (sys {self.conn.target_system})")
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self.okuyucu = mavlink_utils.MavStateReader(
            self.conn,
            ["LOCAL_POSITION_NED", "ATTITUDE", "VIBRATION", "HEARTBEAT"],
            mavlink_utils.parse_local_ned)
        self.okuyucu.start()


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--baglanti', default='udpin:127.0.0.1:14550',
                   help='avci MAVLink (mavproxy cikisi; 14551/14552 dolu '
                        'olabilir, varsayilan 14550)')
    p.add_argument('--no-mavlink', action='store_true',
                   help='FC baglantisi olmadan kos (tutum/hiz None; yasa '
                        'yine kosar, yaw=0 varsayilir)')
    p.add_argument('--loop-hz', type=float, default=20.0)
    p.add_argument('--yasa', choices=['mpc', 'los'], default='mpc',
                   help='salt izlenecek yasa; dosyanin eski davranisi mpc')
    p.add_argument('--n-pn', type=float, default=4.0,
                   help='--yasa los icin PN katsayisi')
    p.add_argument('--vur-ivme', type=float, default=4.0,
                   help='--yasa los icin ileri ivme [m/s2]')
    p.add_argument('--rapor-hz', type=float, default=5.0,
                   help='ekrana yazma hizi (dongu hizindan bagimsiz)')
    p.add_argument('--menzil-m', type=float, default=None,
                   help='SABIT menzil [m]; LOS saha gozleminde verilmesi '
                        'onerilir')
    p.add_argument('--mount', type=float, default=None,
                   help='kamera montaj acisi [deg]; verilmezse $YILDIZ_MOUNT')
    p.add_argument('--aim', type=float, default=None)
    p.add_argument('--no-yaw', action='store_true',
                   help='yaw komutu uretme (yalniz hiz kanallari)')
    p.add_argument('--bayat-s', type=float, default=0.7,
                   help='bbox bayatlik esigi [s] (iskeletle ayni)')
    p.add_argument('--sure', type=float, default=None)
    p.add_argument('--tani-log', default=None,
                   help='MPC tani CSV (varsayilan logs/mpc_izle_tani_*.csv)')
    a = p.parse_args()

    ayar = None
    if a.yasa == 'mpc':
        from mpc_gudum import MpcAyar, MpcKontrolcu
        ayar = MpcAyar()
        if a.mount is not None:
            ayar.mount_pitch_deg = a.mount
        if a.aim is not None:
            ayar.aim_deg = a.aim
        if a.no_yaw:
            ayar.yaw_komutu_ver = False

    from datetime import datetime
    damga = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dizin = _KOK / 'logs'
    log_dizin.mkdir(exist_ok=True)
    tani = a.tani_log or str(log_dizin / f"mpc_izle_tani_{damga}.csv")

    print("=" * 72)
    print("[izle] *** SALT IZLEME MODU: MAVLink'e KOMUT GONDERILMEZ, ARM/MOD")
    print("[izle] *** DEGISIKLIGI YAPILMAZ. Yalniz hesaplanan degerler basilir.")
    print("=" * 72)
    if a.yasa == 'mpc':
        print(f"[izle] mount={ayar.mount_pitch_deg:+.2f} "
              f"aim={ayar.aim_deg:+.2f} "
              f"yaw_komutu={'ACIK' if ayar.yaw_komutu_ver else 'KAPALI'}")
        print(f"[izle] menzil kaynagi: "
              f"{'SABIT %.1f m' % a.menzil_m if a.menzil_m is not None else 'YOK (ic varsayim %.0f m)' % ayar.menzil_yoksa_m}")
        print(f"[izle] MPC tani logu: {tani}")
    else:
        print(f"[izle] LOS/PN: N={a.n_pn:g} vur_ivme={a.vur_ivme:g} m/s2 "
              f"aim={(a.aim or 0.0):+.1f} "
              f"yaw_komutu={'KAPALI' if a.no_yaw else 'ACIK'}")
        print(f"[izle] menzil kaynagi: "
              f"{'SABIT %.1f m' % a.menzil_m if a.menzil_m is not None else 'YOK (ilk ic deger 40 m)'}")

    r = redis.Redis(host='localhost', port=6379, db=0)
    r.ping()
    bbox = BboxOkuyucu(r)
    bbox.start()

    tele = None
    if not a.no_mavlink:
        tele = SaltOkurTelemetri(a.baglanti)

    k = (MpcKontrolcu(ayar, tani_log=tani) if a.yasa == 'mpc'
         else TerminalLosKontrolcu(n_pn=a.n_pn,
                                   vur_ivme_mps2=a.vur_ivme,
                                   aim_deg=(a.aim or 0.0),
                                   yaw_komutu_ver=not a.no_yaw))
    k.tohumla(None)

    loop_dt = 1.0 / a.loop_hz
    rapor_dt = 1.0 / max(a.rapor_hz, 0.1)
    onceki_t = None
    son_rapor = 0.0
    son_cmd = None
    bas = time.monotonic()
    n_kosu = 0
    try:
        while a.sure is None or time.monotonic() - bas < a.sure:
            t0 = time.monotonic()
            simdi = t0
            dt = (loop_dt if onceki_t is None
                  else min(max(simdi - onceki_t, 0.5 * loop_dt), 0.5))
            onceki_t = simdi

            stab, bbox_yas, kapsama = bbox.son()
            ham, ham_yas = bbox.ham()
            taze = stab is not None and bbox_yas <= a.bayat_s

            pos = vel = att = vibe = None
            if tele is not None:
                pos, vel = tele.okuyucu.get()
                att = tele.okuyucu.get_attitude()
                vibe = tele.okuyucu.get_vibration()

            ham_ex = ham_ey = None
            if ham is not None and ham_yas <= a.bayat_s:
                cxp = float(ham[0]) + float(ham[2]) / 2.0
                cyp = float(ham[1]) + float(ham[3]) / 2.0
                ham_ex = math.degrees(math.atan((cxp - _KADRAJ_W / 2.0)
                                                / _KADRAJ_FX))
                ham_ey = math.degrees(math.atan((cyp - _KADRAJ_H / 2.0)
                                                / _KADRAJ_FX))

            if taze:
                o = Olcum(
                    t=simdi, dt=dt,
                    ex_deg=float(stab[4]), ey_deg=float(stab[5]),
                    bbox_w=float(stab[2]), bbox_h=float(stab[3]),
                    alan_kok=math.sqrt(float(stab[2]) * float(stab[3])),
                    kapsama_pct=kapsama,
                    bbox_yas_s=bbox_yas,
                    t_capture=(float(stab[6]) if len(stab) > 6 else None),
                    tilt_deg=(float(stab[7]) if len(stab) > 7
                              and stab[7] is not None else None),
                    menzil_m=a.menzil_m,
                    pos_ned=(np.asarray(pos, float) if pos is not None
                             else None),
                    vel_ned=(np.asarray(vel, float) if vel is not None
                             else None),
                    yaw_rad=att[2] if att is not None else None,
                    roll_rad=att[0] if att is not None else None,
                    pitch_rad=att[1] if att is not None else None,
                    px_sanal_x=float(stab[0]), px_sanal_y=float(stab[1]),
                    px_ham_cx=None, px_ham_cy=None,
                    vibe_max=None if vibe is None else float(max(vibe)),
                )
                cmd = k.komut(o)
                n_kosu += 1
                son_cmd = cmd
                if getattr(cmd, 'olay', ''):
                    print(f"[izle] {a.yasa.upper()} OLAY: {cmd.olay} "
                          f"{getattr(cmd, 'olay_detay', '')}", flush=True)
                if getattr(cmd, 'birak', False):
                    print(f"[izle] {a.yasa.upper()} ISKA/BIRAK ilan etti "
                          f"(sebep={cmd.birak_sebep!r}) -- izlemede yeniden "
                          f"tohumlanip devam ediliyor", flush=True)
                    k.tohumla(None)

            if simdi - son_rapor >= rapor_dt:
                son_rapor = simdi
                if not taze:
                    neden = ('bbox HIC yok' if stab is None
                             else f'bbox bayat ({bbox_yas:.1f} s)')
                    print(f"[izle] tespit bekleniyor: {neden}", flush=True)
                else:
                    v = np.asarray(son_cmd.vel_ned, float)
                    hiz = float(np.linalg.norm(v))
                    yaw_s = ('---' if son_cmd.yaw_rate_dps is None
                             else f"{son_cmd.yaw_rate_dps:+6.2f}")
                    ham_s = ('  yok  /  yok ' if ham_ex is None
                             else f"{ham_ex:+6.2f}/{ham_ey:+6.2f}")
                    if a.yasa == 'mpc':
                        r_ic, durum = k.r_ic, k.durum
                    else:
                        r_ic = float(k.tani.get('r', a.menzil_m or 40.0))
                        durum = str(k.tani.get('faz', k.faz))
                    print(f"ham {ham_s} | sanal {float(stab[4]):+6.2f}/"
                          f"{float(stab[5]):+6.2f} deg | "
                          f"v N{v[0]:+5.2f} E{v[1]:+5.2f} D{v[2]:+5.2f} "
                          f"|{hiz:5.2f}| m/s | yaw {yaw_s} dps | "
                          f"r {r_ic:5.1f} m | {durum}", flush=True)

            kalan = loop_dt - (time.monotonic() - t0)
            if kalan > 0:
                time.sleep(kalan)
    except KeyboardInterrupt:
        pass
    finally:
        ek = f", tani logu: {tani}" if a.yasa == 'mpc' else ''
        print(f"\n[izle] bitti: {n_kosu} {a.yasa.upper()} kosumu{ek}")


if __name__ == '__main__':
    main()
