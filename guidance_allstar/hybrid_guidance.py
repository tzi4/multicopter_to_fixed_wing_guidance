#!/usr/bin/env python3
"""Experimental arm using remote MPC, nearby accessible LOS/PN.

Positional guidance delivers visual turnover in approximately 25 m. This arm maintains the current
MPC for the first part of the cycle, and when the range drops below ``--transition-range``, it switches
to ``TerminalLosController`` until the end of the engagement. The passage has a latch; range noise
cannot produce chatter between the two laws.
"""

from __future__ import annotations

import argparse
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from visual_base import VisualLoop, VisualController
from mpc_guidance import MpcConfig, MpcController
from terminal_los_guidance import TerminalLosController


class HybridController(VisualController):
    label_item = "hybrid"

    def __init__(self, transition_range_m=18.0, mpc_config=None, diagnostic_log=None,
                 terminal=None, transition_source="range_value",
                 visual_area_pct=3.4, visual_ex_deg=6.0,
                 visual_ey_deg=15.0, visual_dwell_s=0.30):
        self.transition_range = float(transition_range_m)
        if transition_source not in ("range_value", "visual"):
            raise ValueError("transition_source should be range or visual")
        self.transition_source = transition_source
        self.visual_area_pct = float(visual_area_pct)
        self.visual_ex = float(visual_ex_deg)
        self.visual_ey = float(visual_ey_deg)
        self.visual_dwell = float(visual_dwell_s)
        config_value = mpc_config or MpcConfig()
        # Don't bring up MPC's problematic close-range STRIKE cost in this branch; that region belongs to the
        # terminal law. External MISS safety is maintained.
        config_value.impact_mode = False
        self.mpc = MpcController(config_value, diagnostic_log=diagnostic_log)
        self.terminal = terminal or TerminalLosController()
        self.phase_value = "MPC"
        self._visual_duration = 0.0
        self.visual_diagnostic = {}

    def seed_value2(self, handoff):
        self.mpc.seed_value2(handoff)
        self.terminal.seed_value2(handoff)
        self.phase_value = "MPC"
        self._visual_duration = 0.0
        self.visual_diagnostic = {}

    def _visual_transition_ready(self, o):
        """A short-dwell viewport independent of telemetry range.

        ``area_pct`` bbox is the square root percentage of the area relative to the framing area. It
        gives the same angular scale even though the size changes. There is no detector confidence
        in the current HSV chain; The fresh/current frame contract replaces it. Real detector
        confidence should be added at the same door in the future.
        """
        area_root = getattr(o, "area_root", None)
        ex = getattr(o, "ex_deg", None)
        ey = getattr(o, "ey_deg", None)
        if area_root is None or ex is None or ey is None:
            self._visual_duration = 0.0
            self.visual_diagnostic = {"ready": False, "reason_value": "missing_measurement"}
            return False
        # Sim/real co-image agreement. The measurement bbox coordinates are generated in this 1280x720 space;
        # If the real camera is different, resize/crop and intrinsics matching must be done first.
        area_pct = 100.0 * float(area_root) / math.sqrt(1280.0 * 720.0)
        instantaneous = (area_pct >= self.visual_area_pct
                 and abs(float(ex)) <= self.visual_ex
                 and abs(float(ey)) <= self.visual_ey)
        dt = max(0.0, min(float(getattr(o, "dt", 0.0) or 0.0), 0.20))
        self._visual_duration = self._visual_duration + dt if instantaneous else 0.0
        ready = self._visual_duration >= self.visual_dwell
        self.visual_diagnostic = {
            "ready": ready, "instantaneous": instantaneous, "area_pct": area_pct,
            "ex": float(ex), "ey": float(ey), "dwell": self._visual_duration,
        }
        return ready

    def command_value(self, o):
        visual_ready = (self._visual_transition_ready(o)
                        if self.phase_value == "MPC" else False)
        range_ready = (o.range_m_value is not None
                        and float(o.range_m_value) <= self.transition_range)
        transition = (range_ready if self.transition_source == "range_value"
                 else visual_ready)
        if self.phase_value == "MPC" and transition:
            handoff = {}
            if o.range_m_value is not None:
                handoff["range_m"] = float(o.range_m_value)
            if o.vel_ned is not None:
                handoff["cmd_vel_ned"] = np.asarray(o.vel_ned, float).tolist()
            self.terminal.seed_value2(handoff)
            self.phase_value = "LOS"
            cmd = self.terminal.command_value(o)
            cmd.event_value = "hybrid_los_transition"
            rtxt = ("absent" if o.range_m_value is None
                    else f"{float(o.range_m_value):.1f}m")
            gt = self.visual_diagnostic
            cmd.event_detail = (
                f"MPC -> LOS source={self.transition_source} r={rtxt} "
                f"area={gt.get('area_pct', float('nan')):.2f}% "
                f"ex={gt.get('ex', float('nan')):+.1f} "
                f"ey={gt.get('ey', float('nan')):+.1f} "
                f"dwell={gt.get('dwell', 0.0):.2f}s")
            return cmd
        if self.phase_value == "LOS":
            return self.terminal.command_value(o)
        cmd = self.mpc.command_value(o)
        # MPC is a planner here; The first command must also enter the same physical accessibility envelope.
        # Thus, the single-frame reverse speed and vertical rail command cannot leak into the hybrid outside
        # of the 18 m.
        if not getattr(cmd, "release_value", False) and o.vel_ned is not None:
            cmd.vel_ned = self.terminal._reachable(
                np.asarray(o.vel_ned, float), np.asarray(cmd.vel_ned, float))
        return cmd


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration-value", type=float, default=None)
    p.add_argument("--loop-hz", type=float, default=20.0)
    p.add_argument("--tau", type=float, default=0.20)
    p.add_argument("--transition-range", type=float, default=18.0)
    p.add_argument("--transition-source", choices=("range_value", "visual"),
                   default="range_value")
    p.add_argument("--visual-area-pct", type=float, default=3.4)
    p.add_argument("--visual-ex", type=float, default=6.0)
    p.add_argument("--visual-ey", type=float, default=15.0)
    p.add_argument("--visual-dwell", type=float, default=0.30)
    p.add_argument("--n-pn", type=float, default=4.0)
    p.add_argument("--strike-acceleration", type=float, default=4.0)
    p.add_argument("--climb-speed-max", type=float, default=2.5)
    p.add_argument("--descent-speed-max", type=float, default=2.0)
    p.add_argument("--terminal-range", type=float, default=3.0)
    p.add_argument("--terminal-tgo", type=float, default=0.25)
    p.add_argument("--log", default=None)
    p.add_argument("--diagnostic-log", default=None)
    a = p.parse_args()
    stamp_value = datetime.now().strftime("%Y%m%d_%H%M%S")
    diagnostic = a.diagnostic_log or str(Path(__file__).resolve().parent / "logs"
                             / f"hybrid_mpc_diagnostic_{stamp_value}.csv")
    terminal = TerminalLosController(
        n_pn=a.n_pn,
        strike_acceleration_mps2=a.strike_acceleration,
        climb_speed_max_mps=a.climb_speed_max,
        descent_speed_max_mps=a.descent_speed_max,
        terminal_range_m=a.terminal_range,
        terminal_tgo_s=a.terminal_tgo)
    k = HybridController(
        transition_range_m=a.transition_range, diagnostic_log=diagnostic, terminal=terminal,
        transition_source=a.transition_source,
        visual_area_pct=a.visual_area_pct, visual_ex_deg=a.visual_ex,
        visual_ey_deg=a.visual_ey, visual_dwell_s=a.visual_dwell)
    print(f"[hybrid] MPC -> terminal LOS pass {k.transition_range:.1f} m; "
          f"source= {k.transition_source} "
          f"image=area>= {k.visual_area_pct:.1f} % "
          f"|ex|<={k.visual_ex:.0f} |ey|<={k.visual_ey:.0f} "
          f"dwell={k.visual_dwell:.2f}s; "
          f"N={terminal.n_pn:.1f} a_strike={terminal.strike_acceleration:.1f}m/s2 "
          f"vz=[-{terminal.climb_speed_max:.1f},"
          f"+{terminal.descent_speed_max:.1f}]m/s; "
          "MPC STRIKE cost off", flush=True)
    VisualLoop(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_path=a.log).run_value(a.duration_value)


if __name__ == "__main__":
    main()
