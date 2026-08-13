#!/usr/bin/env python3
"""Uzakta MPC, yakinda erisilebilir LOS/PN kullanan deney kolu.

Konumlu gudum goruntulu devri yaklasik 25 m'de teslim eder. Bu kol devrin ilk
kisminda mevcut MPC'yi korur, menzil ``--gecis-menzil`` altina indiginde ise
angajman sonuna kadar ``TerminalLosKontrolcu``ye gecer. Gecis latch'lidir;
menzil gurultusu iki yasa arasinda chatter uretemez.
"""

from __future__ import annotations

import argparse
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from goruntulu_temel import GoruntuluDongu, GoruntuluKontrolcu
from mpc_gudum import MpcAyar, MpcKontrolcu
from terminal_los_gudum import TerminalLosKontrolcu


class HibritKontrolcu(GoruntuluKontrolcu):
    ad = "hibrit"

    def __init__(self, gecis_menzil_m=18.0, mpc_ayar=None, tani_log=None,
                 terminal=None, gecis_kaynagi="menzil",
                 gorsel_alan_pct=3.4, gorsel_ex_deg=6.0,
                 gorsel_ey_deg=15.0, gorsel_dwell_s=0.30):
        self.gecis_menzil = float(gecis_menzil_m)
        if gecis_kaynagi not in ("menzil", "gorsel"):
            raise ValueError("gecis_kaynagi menzil veya gorsel olmali")
        self.gecis_kaynagi = gecis_kaynagi
        self.gorsel_alan_pct = float(gorsel_alan_pct)
        self.gorsel_ex = float(gorsel_ex_deg)
        self.gorsel_ey = float(gorsel_ey_deg)
        self.gorsel_dwell = float(gorsel_dwell_s)
        ayar = mpc_ayar or MpcAyar()
        # MPC'nin problemli yakin-menzil VURUS maliyetini bu kolda hic acma;
        # o bolge terminal yasaya aittir. Dis ISKA emniyeti korunur.
        ayar.vurus_modu = False
        self.mpc = MpcKontrolcu(ayar, tani_log=tani_log)
        self.terminal = terminal or TerminalLosKontrolcu()
        self.faz = "MPC"
        self._gorsel_sure = 0.0
        self.gorsel_tani = {}

    def tohumla(self, devir):
        self.mpc.tohumla(devir)
        self.terminal.tohumla(devir)
        self.faz = "MPC"
        self._gorsel_sure = 0.0
        self.gorsel_tani = {}

    def _gorsel_gecis_hazir(self, o):
        """Telemetri menzilinden bagimsiz, kisa-dwell goruntu kapisi.

        ``alan_pct`` bbox alaninin kadraj alanina gore karekok yuzdesidir.
        Boyut degisse de ayni acisal olcegi verir. Mevcut HSV zincirinde
        detector confidence yoktur; taze/gecerli kare sozlesmesi onun yerini
        tutar. Gercek detector confidence ileride ayni kapida eklenmelidir.
        """
        alan_kok = getattr(o, "alan_kok", None)
        ex = getattr(o, "ex_deg", None)
        ey = getattr(o, "ey_deg", None)
        if alan_kok is None or ex is None or ey is None:
            self._gorsel_sure = 0.0
            self.gorsel_tani = {"hazir": False, "sebep": "eksik_olcum"}
            return False
        # Sim/gercek ortak goruntu sozlesmesi. Olcum bbox koordinatlari bu
        # 1280x720 uzayinda uretilir; gercek kamera farkliysa once resize/crop
        # ve intrinsics uyumu yapilmalidir.
        alan_pct = 100.0 * float(alan_kok) / math.sqrt(1280.0 * 720.0)
        anlik = (alan_pct >= self.gorsel_alan_pct
                 and abs(float(ex)) <= self.gorsel_ex
                 and abs(float(ey)) <= self.gorsel_ey)
        dt = max(0.0, min(float(getattr(o, "dt", 0.0) or 0.0), 0.20))
        self._gorsel_sure = self._gorsel_sure + dt if anlik else 0.0
        hazir = self._gorsel_sure >= self.gorsel_dwell
        self.gorsel_tani = {
            "hazir": hazir, "anlik": anlik, "alan_pct": alan_pct,
            "ex": float(ex), "ey": float(ey), "dwell": self._gorsel_sure,
        }
        return hazir

    def komut(self, o):
        gorsel_hazir = (self._gorsel_gecis_hazir(o)
                        if self.faz == "MPC" else False)
        menzil_hazir = (o.menzil_m is not None
                        and float(o.menzil_m) <= self.gecis_menzil)
        gecis = (menzil_hazir if self.gecis_kaynagi == "menzil"
                 else gorsel_hazir)
        if self.faz == "MPC" and gecis:
            devir = {}
            if o.menzil_m is not None:
                devir["range_m"] = float(o.menzil_m)
            if o.vel_ned is not None:
                devir["cmd_vel_ned"] = np.asarray(o.vel_ned, float).tolist()
            self.terminal.tohumla(devir)
            self.faz = "LOS"
            cmd = self.terminal.komut(o)
            cmd.olay = "hibrit_los_gecisi"
            rtxt = ("yok" if o.menzil_m is None
                    else f"{float(o.menzil_m):.1f}m")
            gt = self.gorsel_tani
            cmd.olay_detay = (
                f"MPC->LOS kaynak={self.gecis_kaynagi} r={rtxt} "
                f"alan={gt.get('alan_pct', float('nan')):.2f}% "
                f"ex={gt.get('ex', float('nan')):+.1f} "
                f"ey={gt.get('ey', float('nan')):+.1f} "
                f"dwell={gt.get('dwell', 0.0):.2f}s")
            return cmd
        if self.faz == "LOS":
            return self.terminal.komut(o)
        cmd = self.mpc.komut(o)
        # MPC burada bir planlayicidir; ilk komutu da ayni fiziksel
        # erisilebilirlik zarfina girmek zorundadir. Boylece 18 m disinda da
        # tek-kare ters hiz ve dikey ray komutu hibrite sizamaz.
        if not getattr(cmd, "birak", False) and o.vel_ned is not None:
            cmd.vel_ned = self.terminal._erisilebilir(
                np.asarray(o.vel_ned, float), np.asarray(cmd.vel_ned, float))
        return cmd


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sure", type=float, default=None)
    p.add_argument("--loop-hz", type=float, default=20.0)
    p.add_argument("--tau", type=float, default=0.20)
    p.add_argument("--gecis-menzil", type=float, default=18.0)
    p.add_argument("--gecis-kaynagi", choices=("menzil", "gorsel"),
                   default="menzil")
    p.add_argument("--gorsel-alan-pct", type=float, default=3.4)
    p.add_argument("--gorsel-ex", type=float, default=6.0)
    p.add_argument("--gorsel-ey", type=float, default=15.0)
    p.add_argument("--gorsel-dwell", type=float, default=0.30)
    p.add_argument("--n-pn", type=float, default=4.0)
    p.add_argument("--vur-ivme", type=float, default=4.0)
    p.add_argument("--tirmanma-hiz-max", type=float, default=2.5)
    p.add_argument("--alcalma-hiz-max", type=float, default=2.0)
    p.add_argument("--terminal-menzil", type=float, default=3.0)
    p.add_argument("--terminal-tgo", type=float, default=0.25)
    p.add_argument("--log", default=None)
    p.add_argument("--tani-log", default=None)
    a = p.parse_args()
    damga = datetime.now().strftime("%Y%m%d_%H%M%S")
    tani = a.tani_log or str(Path(__file__).resolve().parent / "logs"
                             / f"hibrit_mpc_tani_{damga}.csv")
    terminal = TerminalLosKontrolcu(
        n_pn=a.n_pn,
        vur_ivme_mps2=a.vur_ivme,
        tirmanma_hiz_max_mps=a.tirmanma_hiz_max,
        alcalma_hiz_max_mps=a.alcalma_hiz_max,
        terminal_menzil_m=a.terminal_menzil,
        terminal_tgo_s=a.terminal_tgo)
    k = HibritKontrolcu(
        gecis_menzil_m=a.gecis_menzil, tani_log=tani, terminal=terminal,
        gecis_kaynagi=a.gecis_kaynagi,
        gorsel_alan_pct=a.gorsel_alan_pct, gorsel_ex_deg=a.gorsel_ex,
        gorsel_ey_deg=a.gorsel_ey, gorsel_dwell_s=a.gorsel_dwell)
    print(f"[hibrit] MPC -> terminal LOS gecisi {k.gecis_menzil:.1f} m; "
          f"kaynak={k.gecis_kaynagi} "
          f"gorsel=alan>={k.gorsel_alan_pct:.1f}% "
          f"|ex|<={k.gorsel_ex:.0f} |ey|<={k.gorsel_ey:.0f} "
          f"dwell={k.gorsel_dwell:.2f}s; "
          f"N={terminal.n_pn:.1f} a_vur={terminal.vur_ivme:.1f}m/s2 "
          f"vz=[-{terminal.tirmanma_hiz_max:.1f},"
          f"+{terminal.alcalma_hiz_max:.1f}]m/s; "
          "MPC VURUS maliyeti kapali", flush=True)
    GoruntuluDongu(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_yolu=a.log).calistir(a.sure)


if __name__ == "__main__":
    main()
