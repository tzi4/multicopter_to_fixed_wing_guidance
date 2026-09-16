#!/usr/bin/env python3
"""
three_dimensional_video.py - 3D walkthrough video as a replacement for QGC (OFFLINE)
=================================================================================== Saving the QGC
window is NOT POSSIBLE: WSLg's X server rejects This tool fills that gap - it does NOT save the
screen, it REDRAWS the 3D scene from the flight logs.

CPU: works completely OFFLINE, costs nothing in flight. The output rate is kept low with --fps
(default is 10), data is diluted there. Matplotlib plots on the Agg backend (no window opens),
frames are written to mp4 with cv2 - matplotlib's own printer cannot be used as there is no ffmpeg
on the system.

Input: guidance_allstar/logs/guided_follow_*.csv (fighter + target position, NED)
run/evidence/gimbal*.csv (optional) to mark detection moments

Usage: tools/three_dimensional_video.py guidance_allstar/logs/guided_follow_XXX.csv \\ --output
run/evidence/3b.mp4 --fps 10
"""

import argparse
import csv
import math
import os

import cv2
import matplotlib
matplotlib.use('Agg')                      # The window DOES NOT open
import matplotlib.pyplot as plt            # noqa: E402
import numpy as np                         # noqa: E402
from mpl_toolkits.mplot3d import Axes3D     # noqa: F401,E402

# Colors: from verified categorical palette (dataviz/references/palette.md, village surface smash).
# Verified with validate_palette.js - all checks passed (worst neighbor CVD dE 8.4, normal vision dE
# 26.6, contrast >= 3:1). The color depends on the ENTITY, not the order: the hunter is always blue,
# the target is always orange.
PURSUER = '#3987e5'
TARGET = '#d95936'
DETECTION = '#199e70'
SURFACE = '#1a1a19'
MAIN_TEXT = '#ffffff'
SECONDARY = '#c3c2b7'


def read_value(path_value):
    with open(path_value) as f:
        return list(csv.DictReader(f))


def fl(row_value, label_item):
    try:
        return float(row_value[label_item])
    except (TypeError, ValueError, KeyError):
        return float('nan')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('guidance_csv')
    p.add_argument('--output', default='run/evidence/3d_tracking.mp4')
    p.add_argument('--fps', type=float, default=10.0,
                   help='output video speed (low = cheap)')
    p.add_argument('--trace', type=int, default=250, help='How many samples should be left behind?')
    p.add_argument('--size-value', default='1280x720')
    a = p.parse_args()

    rows_value = read_value(a.guidance_csv)
    # NED -> display: north(x), east(y), elevation(-z)
    px = [fl(s, 'pursuer_x') for s in rows_value]
    py = [fl(s, 'pursuer_y') for s in rows_value]
    pz = [-fl(s, 'pursuer_z') for s in rows_value]
    tx = [fl(s, 'meas_x') for s in rows_value]
    ty = [fl(s, 'meas_y') for s in rows_value]
    tz = [-fl(s, 'meas_z') for s in rows_value]
    rng = [fl(s, 'range_m') for s in rows_value]
    valid_value = [i for i in range(len(rows_value))
               if not any(math.isnan(v) for v in (px[i], py[i], pz[i], tx[i], ty[i], tz[i]))]
    if len(valid_value) < 20:
        raise SystemExit(f"insufficient valid example: {len(valid_value)}")

    # Data ~20 Hz; dilute to output fps.
    step_value = max(1, int(round(20.0 / a.fps)))
    idx = valid_value[::step_value]
    print(f"{len(rows_value)} sample -> {len(idx)} frame ({a.fps:.0f} fps, "
          f"~{len(idx)/a.fps:.0f} s video)")

    W, H = (int(v) for v in a.size_value.split('x'))
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=SURFACE)
    ax = fig.add_subplot(111, projection='3d', facecolor=SURFACE)

    all_x = [v for v in px + tx if not math.isnan(v)]
    all_y = [v for v in py + ty if not math.isnan(v)]
    all_z = [v for v in pz + tz if not math.isnan(v)]
    # Equal scale: If the axes are scaled differently in 3D, the orbit will be misleadingly tilted.
    middle = [(min(v) + max(v)) / 2 for v in (all_x, all_y, all_z)]
    half = max(max(v) - min(v) for v in (all_x, all_y)) / 2 * 1.05
    half_z = max(30.0, (max(all_z) - min(all_z)) / 2 * 1.4)

    out = cv2.VideoWriter(a.output, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (W, H))
    for k, i in enumerate(idx):
        ax.clear()
        ax.set_facecolor(SURFACE)
        j0 = max(0, k - a.trace_samples) * step_value
        trace_samples = valid_value[max(0, valid_value.index(i) - a.trace_samples):valid_value.index(i) + 1]
        # Izler: ince cizgi (thin marks)
        ax.plot([px[q] for q in trace_samples], [py[q] for q in trace_samples], [pz[q] for q in trace_samples],
                color=PURSUER, linewidth=1.6, label='hunter (copter)')
        ax.plot([tx[q] for q in trace_samples], [ty[q] for q in trace_samples], [tz[q] for q in trace_samples],
                color=TARGET, linewidth=1.6, label='target (Talon)')
        # Current positions + LOS
        ax.scatter([px[i]], [py[i]], [pz[i]], color=PURSUER, s=45, depthshade=False)
        ax.scatter([tx[i]], [ty[i]], [tz[i]], color=TARGET, s=45, depthshade=False)
        near_value = not math.isnan(rng[i]) and rng[i] < 60
        ax.plot([px[i], tx[i]], [py[i], ty[i]], [pz[i], tz[i]],
                color=DETECTION if near_value else SECONDARY,
                linewidth=1.4 if near_value else 0.8,
                linestyle='-' if near_value else '--',
                label='gorus hatti' if k == 0 else None)

        ax.set_xlim(middle[0] - half, middle[0] + half)
        ax.set_ylim(middle[1] - half, middle[1] + half)
        ax.set_zlim(max(0, middle[2] - half_z), middle[2] + half_z)
        # Subdued axes
        for axis_value in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis_value.pane.set_facecolor(SURFACE)
            axis_value.pane.set_alpha(1.0)
            axis_value._axinfo['grid']['color'] = (1, 1, 1, 0.10)
        ax.tick_params(colors=SECONDARY, labelsize=7)
        ax.set_xlabel('north (m)', color=SECONDARY, fontsize=8)
        ax.set_ylabel('birth)', color=SECONDARY, fontsize=8)
        ax.set_zlabel('altitude (m)', color=SECONDARY, fontsize=8)
        ax.view_init(elev=22, azim=(-60 + k * 0.15) % 360)   # slowly turning gaze

        # Number in the headline: range (single headline)
        m = '' if math.isnan(rng[i]) else f"{rng[i]:.0f} m"
        ax.set_title(f"positional guidance - distance to target {m}",
                     color=MAIN_TEXT, fontsize=13, pad=14)
        leg = ax.legend(loc='upper left', facecolor=SURFACE, edgecolor='none',
                        fontsize=8, labelcolor=SECONDARY)
        if leg:
            leg.get_frame().set_alpha(0.6)

        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        out.write(cv2.cvtColor(buf, cv2.COLOR_RGB2BGR))
        if k % 50 == 0:
            print(f"  square {k} / {len(idx)}", flush=True)
    out.release()
    plt.close(fig)
    print(f"written: {a.output} ({os.path.getsize(a.output)/1e6:.1f} MB)")


if __name__ == '__main__':
    main()
