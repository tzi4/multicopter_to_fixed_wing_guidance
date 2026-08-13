#!/usr/bin/env python3
"""
uc_boyut_video.py - QGC yerine gecen 3B izleme videosu (CEVRIMDISI)
===================================================================
QGC penceresini kaydetmek MUMKUN DEGIL: WSLg'nin X sunucusu XGetImage'i
reddediyor (Pillow, mss ve ffmpeg'in x11grab'i ayni cagriyi kullanir; ucu de
denendi, ucu de basarisiz). Bu arac o boslugu doldurur - ekrani KAYDETMEZ,
ucus loglarindan 3B sahneyi YENIDEN CIZER.

CPU: tamamen CEVRIMDISI calisir, ucus sirasinda hicbir maliyeti yoktur.
Cikti hizi --fps ile dusuk tutulur (varsayilan 10), veri o hiza seyreltilir.
Matplotlib Agg arka ucunda cizer (pencere acilmaz), kareler cv2 ile mp4'e
yazilir - sistemde ffmpeg olmadigi icin matplotlib'in kendi yazicisi
kullanilamaz.

Girdi:
    guidance_allstar/logs/guided_follow_*.csv   (avci + hedef konumu, NED)
    run/kanit/gimbal*.csv (istege bagli)        tespit anlarini isaretlemek icin

Kullanim:
    tools/uc_boyut_video.py guidance_allstar/logs/guided_follow_XXX.csv \\
        --cikti run/kanit/3b.mp4 --fps 10
"""

import argparse
import csv
import math
import os

import cv2
import matplotlib
matplotlib.use('Agg')                      # pencere ACMAZ
import matplotlib.pyplot as plt            # noqa: E402
import numpy as np                         # noqa: E402
from mpl_toolkits.mplot3d import Axes3D     # noqa: F401,E402

# Renkler: dogrulanmis kategorik paletten (dataviz/references/palette.md,
# koyu yuzey sutunu). validate_palette.js ile dogrulandi - tum kontroller
# gecti (en kotu komsu CVD dE 8.4, normal gorus dE 26.6, kontrast >= 3:1).
# Renk ENTITY'ye bagli, siraya degil: avci hep mavi, hedef hep turuncu.
AVCI = '#3987e5'
HEDEF = '#d95936'
TESPIT = '#199e70'
YUZEY = '#1a1a19'
ANA_YAZI = '#ffffff'
IKINCIL = '#c3c2b7'


def oku(yol):
    with open(yol) as f:
        return list(csv.DictReader(f))


def fl(satir, ad):
    try:
        return float(satir[ad])
    except (TypeError, ValueError, KeyError):
        return float('nan')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('guidance_csv')
    p.add_argument('--cikti', default='run/kanit/3b_izleme.mp4')
    p.add_argument('--fps', type=float, default=10.0,
                   help='cikti video hizi (dusuk = ucuz)')
    p.add_argument('--iz', type=int, default=250, help='kac ornek iz birakilsin')
    p.add_argument('--boyut', default='1280x720')
    a = p.parse_args()

    satirlar = oku(a.guidance_csv)
    # NED -> gosterim: kuzey(x), dogu(y), yukseklik(-z)
    px = [fl(s, 'pursuer_x') for s in satirlar]
    py = [fl(s, 'pursuer_y') for s in satirlar]
    pz = [-fl(s, 'pursuer_z') for s in satirlar]
    tx = [fl(s, 'meas_x') for s in satirlar]
    ty = [fl(s, 'meas_y') for s in satirlar]
    tz = [-fl(s, 'meas_z') for s in satirlar]
    rng = [fl(s, 'range_m') for s in satirlar]
    gecerli = [i for i in range(len(satirlar))
               if not any(math.isnan(v) for v in (px[i], py[i], pz[i], tx[i], ty[i], tz[i]))]
    if len(gecerli) < 20:
        raise SystemExit(f"yetersiz gecerli ornek: {len(gecerli)}")

    # Veri ~20 Hz; cikti fps'ine seyrelt.
    adim = max(1, int(round(20.0 / a.fps)))
    idx = gecerli[::adim]
    print(f"{len(satirlar)} ornek -> {len(idx)} kare ({a.fps:.0f} fps, "
          f"~{len(idx)/a.fps:.0f} s video)")

    W, H = (int(v) for v in a.boyut.split('x'))
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=YUZEY)
    ax = fig.add_subplot(111, projection='3d', facecolor=YUZEY)

    tum_x = [v for v in px + tx if not math.isnan(v)]
    tum_y = [v for v in py + ty if not math.isnan(v)]
    tum_z = [v for v in pz + tz if not math.isnan(v)]
    # Esit olcek: 3B'de eksenler farkli olceklenirse yorunge yaniltici egilir.
    orta = [(min(v) + max(v)) / 2 for v in (tum_x, tum_y, tum_z)]
    yari = max(max(v) - min(v) for v in (tum_x, tum_y)) / 2 * 1.05
    yari_z = max(30.0, (max(tum_z) - min(tum_z)) / 2 * 1.4)

    out = cv2.VideoWriter(a.cikti, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (W, H))
    for k, i in enumerate(idx):
        ax.clear()
        ax.set_facecolor(YUZEY)
        j0 = max(0, k - a.iz) * adim
        iz = gecerli[max(0, gecerli.index(i) - a.iz):gecerli.index(i) + 1]
        # Izler: ince cizgi (thin marks)
        ax.plot([px[q] for q in iz], [py[q] for q in iz], [pz[q] for q in iz],
                color=AVCI, linewidth=1.6, label='avci (kopter)')
        ax.plot([tx[q] for q in iz], [ty[q] for q in iz], [tz[q] for q in iz],
                color=HEDEF, linewidth=1.6, label='hedef (Talon)')
        # Anlik konumlar + LOS
        ax.scatter([px[i]], [py[i]], [pz[i]], color=AVCI, s=45, depthshade=False)
        ax.scatter([tx[i]], [ty[i]], [tz[i]], color=HEDEF, s=45, depthshade=False)
        yakin = not math.isnan(rng[i]) and rng[i] < 60
        ax.plot([px[i], tx[i]], [py[i], ty[i]], [pz[i], tz[i]],
                color=TESPIT if yakin else IKINCIL,
                linewidth=1.4 if yakin else 0.8,
                linestyle='-' if yakin else '--',
                label='gorus hatti' if k == 0 else None)

        ax.set_xlim(orta[0] - yari, orta[0] + yari)
        ax.set_ylim(orta[1] - yari, orta[1] + yari)
        ax.set_zlim(max(0, orta[2] - yari_z), orta[2] + yari_z)
        # Recessive eksenler
        for eksen in (ax.xaxis, ax.yaxis, ax.zaxis):
            eksen.pane.set_facecolor(YUZEY)
            eksen.pane.set_alpha(1.0)
            eksen._axinfo['grid']['color'] = (1, 1, 1, 0.10)
        ax.tick_params(colors=IKINCIL, labelsize=7)
        ax.set_xlabel('kuzey (m)', color=IKINCIL, fontsize=8)
        ax.set_ylabel('dogu (m)', color=IKINCIL, fontsize=8)
        ax.set_zlabel('irtifa (m)', color=IKINCIL, fontsize=8)
        ax.view_init(elev=22, azim=(-60 + k * 0.15) % 360)   # yavas donen bakis

        # Basliktaki sayi: menzil (tek headline)
        m = '' if math.isnan(rng[i]) else f"{rng[i]:.0f} m"
        ax.set_title(f"konumlu gudum - hedefe mesafe {m}",
                     color=ANA_YAZI, fontsize=13, pad=14)
        leg = ax.legend(loc='upper left', facecolor=YUZEY, edgecolor='none',
                        fontsize=8, labelcolor=IKINCIL)
        if leg:
            leg.get_frame().set_alpha(0.6)

        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        out.write(cv2.cvtColor(buf, cv2.COLOR_RGB2BGR))
        if k % 50 == 0:
            print(f"  kare {k}/{len(idx)}", flush=True)
    out.release()
    plt.close(fig)
    print(f"yazildi: {a.cikti} ({os.path.getsize(a.cikti)/1e6:.1f} MB)")


if __name__ == '__main__':
    main()
