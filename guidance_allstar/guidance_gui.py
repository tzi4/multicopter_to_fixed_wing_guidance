#!/usr/bin/env python3
"""
guidance_gui.py  –  Real-time estimator telemetry & parameter tuning GUI.

A Material-styled, dark-themed, tabbed interface fed by the guidance runner.

TAB  "Estimator"      –  the three live estimator panels (same ones filterwndr's
                         own view shows, minus the 3-D plot):
                           1. Estimation error (est - measurement), X / Y / Z
                           2. IMM product-model probabilities (CVxy/CTxy/CAxy +
                              CVz/CAz dashed) with effective omega on a twin axis
                           3. Residual norms (innovation / update-jump / error)
TAB  "Approach"       –  live status cards (range, t_go, closing speed, dominant
                         IMM mode, FSM/recovery state), the closest approach in
                         the last 30 s, a per-30-s window minima log, and a log
                         of every sub-5 m approach.
TAB  "Config"         –  live-editable guidance_config.py parameters.

USAGE (from any runner):
    from guidance_gui import GuidanceGUI, push_snapshot, gui_tick
    gui = GuidanceGUI(param_module="guidance_config")
    gui.start()                                    # non-blocking, own thread
    ...
    push_snapshot(pursuer_pos, target_pos, mode_probs,
                  target_est=est, status={...}, diag={...})   # each loop
    gui_tick()                                     # cheap no-op, safe to call
    ...
    gui.stop()

Threading model:
    The GUI toolkit (PyQt5 or tkinter) owns its event loop entirely inside the
    background thread created by start(). Runners only ever touch the thread-safe
    _DataHub via push_snapshot(); they never call a toolkit method directly. This
    keeps the Tk backend from deadlocking — earlier versions pumped _root.update()
    from the runner thread while mainloop() ran in the GUI thread, which drives
    one Tcl interpreter from two threads and freezes.
"""

from __future__ import annotations

import ast
import importlib
import math
import os
import re
import sys
import threading
import time
import tokenize
import traceback
from collections import deque
from datetime import datetime
from typing import Optional

import numpy as np

# ── attempt PyQt5, fallback to Tk ──────────────────────────────────────────
try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as _Canvas
    from matplotlib.figure import Figure
    from PyQt5 import QtCore, QtWidgets

    _USE_QT = True
except ImportError:
    try:
        import tkinter as tk
        from tkinter import ttk

        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as _Canvas
        from matplotlib.figure import Figure

        _USE_QT = False
    except ImportError:
        print("[gui] No suitable GUI backend found (PyQt5 or tkinter). Exiting.")
        sys.exit(1)

_BACKEND_NAME = "PyQt5" if _USE_QT else "tkinter"

# ── shared state ───────────────────────────────────────────────────────────
_GUI_INSTANCE = None  # type: Optional[GuidanceGUI]

RC = "\033[0;31m"
NC = "\033[0m"

# ── Material dark palette ──────────────────────────────────────────────────
PAL = {
    "bg": "#0f1117",       # window background
    "surface": "#171a23",  # tab / bar surface
    "panel": "#1e2230",     # cards, plot axes
    "border": "#2b3140",
    "text": "#e6e9ef",
    "muted": "#8b93a7",
    "primary": "#4f9cff",
    "accent": "#26c6a2",
    "amber": "#ffb454",
    "danger": "#ff5c6c",
    "ok": "#3ddc84",
    "grid": "#2b3140",
}

# estimator plot line colours (tuned for the dark panel)
_ERR_STYLE = (("x", "#ff6b6b"), ("y", "#4ade80"), ("z", "#60a5fa"))
_MODE_STYLE = (
    ("cv_xy", "#4f9cff", "-", "CVxy"),
    ("ct_xy", "#ffb454", "-", "CTxy"),
    ("ca_xy", "#3ddc84", "-", "CAxy"),
    ("cv_z", "#22d3ee", ":", "CVz"),
    ("ca_z", "#c084fc", ":", "CAz"),
)
_MODE_COLOR = {"cv": "#4f9cff", "ct": "#ffb454", "ca": "#3ddc84"}
_RES_STYLE = (
    ("innov", "#c084fc", "Innovation"),
    ("jump", "#ff5c6c", "Update jump"),
    ("errnorm", "#9aa4bc", "Error norm"),
)
_FSM_COLOR = {"CHASE": "#4f9cff", "HOLD": "#ffb454", "REENGAGE": "#26c6a2"}


def _split_trailing_comment(text):
    # type: (str) -> tuple
    """Split 'value  # note' into ('value  ', '# note'), ignoring any '#' that
    lives inside a string literal (e.g. a connection string)."""
    quote = None
    escaped = False
    for i, ch in enumerate(text):
        if quote:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#":
            return text[:i], text[i:]
    return text, ""


class _ParamStore:
    """Thread-safe parameter dict with live-edit write-back support."""

    # Only these types can be round-tripped back into the source file safely.
    _EDITABLE_TYPES = (bool, int, float, str)

    def __init__(self, module_name):
        # type: (str) -> None
        self.module_name = module_name
        self._lock = threading.Lock()
        self._params = {}  # type: dict
        self._mtime = 0.0
        self._reload()

    def _reload(self):
        try:
            mod = importlib.import_module(self.module_name)
            importlib.reload(mod)
        except Exception as exc:
            print(f"[gui] WARNING: could not reload {self.module_name}: {exc}")
            return
        with self._lock:
            self._params = {}
            for name in dir(mod):
                if name.startswith("_"):
                    continue
                val = getattr(mod, name)
                if callable(val) or isinstance(val, type):
                    continue
                self._params[name] = val
            try:
                path = mod.__file__
                if path:
                    self._mtime = os.path.getmtime(path)
            except Exception:
                pass

    def snapshot(self):
        # type: () -> dict
        with self._lock:
            return dict(self._params)

    def is_editable(self, name):
        # type: (str) -> bool
        with self._lock:
            return isinstance(self._params.get(name), self._EDITABLE_TYPES)

    def set_param(self, name, value_str):
        # type: (str, str) -> bool
        """Write a single scalar parameter back to the module file.
        Returns True on success. Refuses non-scalar params so the source file
        can never be corrupted by the editor."""
        try:
            mod = importlib.import_module(self.module_name)
            old = getattr(mod, name)
        except Exception:
            return False

        # Only round-trip types we can render unambiguously back to source.
        if not isinstance(old, self._EDITABLE_TYPES):
            return False

        # parse the string back into the original type (bool before int: bool is int)
        try:
            if isinstance(old, bool):
                new_val = value_str.strip().lower() in ("true", "1", "yes", "on")
            elif isinstance(old, int):
                new_val = int(value_str)
            elif isinstance(old, float):
                new_val = float(value_str)
            else:  # str
                new_val = value_str
        except ValueError:
            return False

        path = getattr(mod, "__file__", None)
        if not path or not os.path.isfile(path):
            return False
        try:
            with open(path, "r") as fh:
                lines = fh.readlines()
        except OSError:
            return False

        pattern = re.compile(rf"^(\s*{re.escape(name)}\s*=\s*)(.*)")
        replaced = False
        out = []
        for line in lines:
            m = pattern.match(line)
            if m and not replaced:
                # Keep the trailing comment: it is this parameter's
                # documentation (the Config tab's detail panel reads it), so a
                # value edit must never silently delete it.
                code, comment = _split_trailing_comment(m.group(2))
                gap = code[len(code.rstrip()):] if comment else ""
                if isinstance(new_val, str):
                    # Keep the file's double-quote style when safe; fall back
                    # to repr() when the value embeds quotes/backslashes so we
                    # can never write syntactically invalid Python to disk.
                    if '"' not in new_val and "\\" not in new_val:
                        value_txt = f'"{new_val}"'
                    else:
                        value_txt = repr(new_val)
                else:
                    value_txt = str(new_val)
                out.append(f"{m.group(1)}{value_txt}{gap}{comment}\n")
                replaced = True
            else:
                out.append(line)
        if not replaced:
            out.append(f"\n{name} = {new_val!r}\n")

        try:
            with open(path, "w") as fh:
                fh.writelines(out)
        except OSError:
            return False

        self._reload()
        return True


# ── parameter documentation (QGroundControl-style metadata) ────────────────
_SECTION_RE = re.compile(r"^-{2,}\s*(.+?)\s*-{2,}$")
_ASSIGN_RE = re.compile(r"^([A-Za-z_]\w*)\s*=")
# The consumers read optional params as getattr(cfg, "NAME", <default>); that
# fallback IS the code's default, so harvest it rather than keeping a second
# hand-maintained table that would drift.
_GETATTR_RE = re.compile(r"""getattr\(\s*cfg\s*,\s*["'](\w+)["']\s*,\s*([^()]*?)\s*\)""")


class _ParamDocs:
    """Per-parameter metadata for the Config detail panel.

    Everything is harvested from source at load time, so the docs can never
    drift from the code:

        section      nearest '# --- Section ---' banner above the assignment
        summary      the assignment's trailing comment (one-line gist)
        description  the contiguous comment block above the assignment
        default      the fallback in a consumer's getattr(cfg, "NAME", <default>)
        line         where it is defined, so the panel can point at the source

    Re-parsed when the config file's mtime changes (the editor writes it back).
    """

    def __init__(self, module_name, consumer_files=("simple_guided_follow.py", "filterwndr.py")):
        # type: (str, tuple) -> None
        self.module_name = module_name
        self._consumers = consumer_files
        self._lock = threading.Lock()
        self._meta = {}  # type: dict
        self._mtime = 0.0
        self.refresh()

    def _config_path(self):
        try:
            mod = importlib.import_module(self.module_name)
            return getattr(mod, "__file__", None)
        except Exception:
            return None

    def refresh(self, force=False):
        path = self._config_path()
        if not path or not os.path.isfile(path):
            return
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        with self._lock:
            if not force and mtime == self._mtime and self._meta:
                return
        meta = self._parse_config(path)
        for name, raw in self._parse_defaults(os.path.dirname(path)).items():
            if name in meta:
                meta[name]["default"] = raw
        with self._lock:
            self._meta = meta
            self._mtime = mtime

    @staticmethod
    def _comment_lines(path):
        """line number -> comment text. Tokenize so a '#' inside a string
        literal is never mistaken for a comment."""
        out = {}
        try:
            with open(path, "rb") as fh:
                for tok in tokenize.tokenize(fh.readline):
                    if tok.type == tokenize.COMMENT:
                        out[tok.start[0]] = tok.string.lstrip("#").strip()
        except Exception:
            pass
        return out

    @staticmethod
    def _section_name(text):
        """'--- Lead construction (2026-07-13): build the aim ...' -> 'Lead
        construction'. Cut at the first ':' or '(' so the wordy banners in this
        config still yield a short QGC-style group label."""
        t = text.strip().strip("-").strip()
        cut = len(t)
        for sep in (":", "("):
            p = t.find(sep)
            if p > 0:
                cut = min(cut, p)
        return t[:cut].strip(" -")

    def _banners(self, lines, comments):
        """[(start, end, name)] for '# --- Section ---' banners. Several of
        them wrap across many comment lines ('# --- Lead construction (...)'
        ... '... shorter effective lead). ---'), so a banner runs from the line
        that OPENS with dashes to the first one that CLOSES with them. Getting
        this right matters twice: the label, and stopping the description
        walk-up from swallowing the banner prose."""
        out = []
        n = len(lines)
        i = 0
        while i < n:
            if not lines[i].strip().startswith("#"):
                i += 1
                continue
            text = comments.get(i + 1, "")
            if not text.startswith("--"):
                i += 1
                continue
            end = i
            j = i
            while j < n and lines[j].strip().startswith("#"):
                t = comments.get(j + 1, "")
                if t.endswith("--") and (j > i or len(t) > 2):
                    end = j
                    break
                j += 1
            else:
                end = i
            # Keep the banner's full prose too: for the handful of params that
            # sit directly under a wordy banner and carry no comment of their
            # own, that paragraph IS their documentation.
            prose = " ".join(
                comments.get(k + 1, "") for k in range(i, end + 1)
            ).strip().strip("-").strip()
            out.append((i, end, self._section_name(text), prose))
            i = end + 1
        return out

    def _parse_config(self, path):
        try:
            with open(path, "r") as fh:
                lines = fh.readlines()
        except OSError:
            return {}
        comments = self._comment_lines(path)
        banners = self._banners(lines, comments)
        banner_lines = set()
        starts = {}
        for start, end, name, prose in banners:
            starts[start] = (name, prose)
            banner_lines.update(range(start, end + 1))

        meta = {}
        section = ""
        section_doc = ""
        run_doc = ""  # block documenting the current unbroken run of assignments
        for idx, raw in enumerate(lines):
            # Track the section on the way down: it applies to every param
            # below it, not just the first (walking up from each param would
            # stop at the previous assignment and never reach the banner).
            if idx in starts:
                section, section_doc = starts[idx]
            if not raw.strip():
                run_doc = ""  # a blank line ends the group a block documents
                continue
            if raw.strip().startswith("#"):
                continue
            m = _ASSIGN_RE.match(raw)
            if not m:
                continue
            name = m.group(1)
            if name.startswith("_"):
                continue
            block = []
            j = idx - 1
            while j >= 0:
                if not lines[j].strip().startswith("#") or j in banner_lines:
                    break
                block.append(comments.get(j + 1, ""))
                j -= 1
            block.reverse()
            own = " ".join(x for x in block if x).strip()
            # The config documents GROUPS of adjacent params with one block
            # (e.g. the block above LEAD_HORIZONTAL_ACCEL_SCALE also covers the
            # LEAD_VERTICAL_* lines right below it). Inherit it for params that
            # have no block of their own, but flag it so the panel can say the
            # text is shared rather than implying it is param-specific.
            if own:
                run_doc = own
            doc = own or run_doc or section_doc
            shared = bool(not own and doc)
            meta[name] = {
                "section": section,
                "summary": comments.get(idx + 1, ""),
                "description": doc,
                "shared": shared,
                "default": None,
                "line": idx + 1,
            }
        return meta

    def _parse_defaults(self, base_dir):
        out = {}
        for fname in self._consumers:
            try:
                with open(os.path.join(base_dir, fname), "r") as fh:
                    src = fh.read()
            except OSError:
                continue
            for m in _GETATTR_RE.finditer(src):
                name, raw = m.group(1), m.group(2).strip()
                if name not in out:
                    out[name] = raw
                elif out[name] != raw and "ambiguous" not in out[name]:
                    # Consumers disagree on the fallback (e.g. LOOP_HZ is 20 in
                    # the runner, 0.0 in filterwndr): show both instead of
                    # silently picking one. literal_eval of the composite fails
                    # -> default_value() returns None -> the "modified" badge is
                    # suppressed rather than judged against the wrong number.
                    out[name] = f"ambiguous: {out[name]} | {raw}"
        return out

    def get(self, name):
        # type: (str) -> dict
        self.refresh()
        with self._lock:
            return dict(self._meta.get(name, {}))

    def default_value(self, name):
        """(value, raw_text). value is None when there is no code default or it
        is not a literal we can evaluate."""
        raw = self.get(name).get("default")
        if raw is None:
            return None, None
        try:
            return ast.literal_eval(raw), raw
        except Exception:
            return None, raw


# ── data ring-buffers ──────────────────────────────────────────────────────
_WINDOW_S = 30.0          # tumbling-window length for per-window minima
_SUB5_M = 5.0             # "under 5 m" approach threshold
_SUB5_RELEASE_M = 5.5     # hysteresis release so one pass = one logged event


class _DataHub:
    """Thread-safe ring-buffer store for telemetry pushed by the runner."""

    def __init__(self, max_history=900):
        # type: (int) -> None
        self._lock = threading.Lock()
        self._max = max_history
        # aligned estimator-plot history (all appended together each push)
        self.t_hist = deque(maxlen=max_history)       # type: deque
        self.mode_hist = deque(maxlen=max_history)     # type: deque
        self.err_hist = deque(maxlen=max_history)      # type: deque  (3-vec)
        self.omega_hist = deque(maxlen=max_history)    # type: deque
        self.innov_hist = deque(maxlen=max_history)    # type: deque
        self.jump_hist = deque(maxlen=max_history)     # type: deque
        self.errnorm_hist = deque(maxlen=max_history)  # type: deque
        # latest positions / status
        self.pursuer_pos = np.zeros(3)
        self.target_pos = np.zeros(3)
        self.status = {}  # type: dict
        # approach tracking
        self.range_win = deque()          # (t, range) rolling last-30-s window
        self.window_log = deque(maxlen=60)  # (start_wall, min_range, min_wall)
        self.sub5_log = deque(maxlen=40)    # (min_range, min_wall)
        self._t0 = None
        self._win_idx = 0
        self._win_start_wall = "--:--:--"
        self._win_min = float("inf")
        self._win_min_wall = "--:--:--"
        self._in_appr = False
        self._appr_min = float("inf")
        self._appr_min_wall = "--:--:--"

    def push_step(
        self, t, pursuer_pos, target_pos, mode_probs=None, target_est=None,
        status=None, diag=None,
    ):
        with self._lock:
            self.pursuer_pos = np.asarray(pursuer_pos, dtype=float).ravel()[:3]
            self.target_pos = np.asarray(target_pos, dtype=float).ravel()[:3]
            if status:
                self.status = dict(status)

            now_str = datetime.now().strftime("%H:%M:%S")

            # Approach tracking only runs when the runner supplies a real
            # pursuer↔target range (guidance). A pure-estimator run (filterwndr,
            # no pursuer) passes no range_m, so range/t_go/FSM cards and the
            # approach logs stay "--" instead of filling with meaningless data.
            rng = float(self.status.get("range_m", float("nan")))

            # ── aligned estimator-plot history ──
            self.t_hist.append(float(t))
            self.mode_hist.append(dict(mode_probs) if mode_probs else {})
            if target_est is not None:
                est = np.asarray(target_est, dtype=float).ravel()[:3]
                self.err_hist.append(est - self.target_pos)
            else:
                self.err_hist.append(np.full(3, np.nan))
            d = diag or {}
            self.omega_hist.append(float(d.get("omega", np.nan)))
            self.innov_hist.append(float(d.get("innov_norm", np.nan)))
            self.jump_hist.append(float(d.get("jump_norm", np.nan)))
            self.errnorm_hist.append(float(d.get("err_norm", np.nan)))

            # ── approach tracking ──
            if np.isfinite(rng):
                self._track_approach(float(t), rng, now_str)

    def _track_approach(self, t, rng, now_str):
        # rolling last-30-s window
        self.range_win.append((t, rng))
        cutoff = t - _WINDOW_S
        while self.range_win and self.range_win[0][0] < cutoff:
            self.range_win.popleft()

        # tumbling 30-s windows -> per-window minima log
        if self._t0 is None:
            self._t0 = t
            self._win_start_wall = now_str
            self._win_min = float("inf")
        widx = int((t - self._t0) // _WINDOW_S)
        if widx != self._win_idx:
            if np.isfinite(self._win_min):
                self.window_log.append(
                    (self._win_start_wall, self._win_min, self._win_min_wall)
                )
            self._win_idx = widx
            self._win_start_wall = now_str
            self._win_min = float("inf")
        if rng < self._win_min:
            self._win_min = rng
            self._win_min_wall = now_str

        # sub-5 m passes (one event per crossing, logged at its minimum)
        if rng < _SUB5_M:
            if not self._in_appr:
                self._in_appr = True
                self._appr_min = rng
                self._appr_min_wall = now_str
            elif rng < self._appr_min:
                self._appr_min = rng
                self._appr_min_wall = now_str
        elif self._in_appr and rng > _SUB5_RELEASE_M:
            self.sub5_log.append((self._appr_min, self._appr_min_wall))
            self._in_appr = False

    # ── snapshot helpers used by both backends ──
    def plot_arrays(self):
        with self._lock:
            if not self.t_hist:
                return None
            t = np.asarray(self.t_hist, dtype=float)
            err = np.asarray(self.err_hist, dtype=float)
            modes = {
                k: np.asarray([m.get(k, np.nan) for m in self.mode_hist], dtype=float)
                for (k, *_rest) in _MODE_STYLE
            }
            omega = np.asarray(self.omega_hist, dtype=float)
            innov = np.asarray(self.innov_hist, dtype=float)
            jump = np.asarray(self.jump_hist, dtype=float)
            errnorm = np.asarray(self.errnorm_hist, dtype=float)
        return {
            "t": t, "err": err, "modes": modes, "omega": omega,
            "innov": innov, "jump": jump, "errnorm": errnorm,
        }

    def status_snapshot(self):
        with self._lock:
            st = dict(self.status)
            last30 = min((r for _, r in self.range_win), default=float("nan"))
            last_mode = dict(self.mode_hist[-1]) if self.mode_hist else {}
        return st, last30, last_mode

    def log_snapshot(self):
        with self._lock:
            return list(self.window_log), list(self.sub5_log)

    def header_snapshot(self):
        """(#samples seen, latest error norm) — for the live header indicator."""
        with self._lock:
            n = len(self.t_hist)
            err = float(self.errnorm_hist[-1]) if self.errnorm_hist else float("nan")
            return n, err


def _dominant(mode_probs, keys):
    """Return (label, prob, colour) of the most-likely mode among keys."""
    best_k, best_v = None, -1.0
    for k in keys:
        v = float(mode_probs.get(k, 0.0))
        if v > best_v:
            best_k, best_v = k, v
    if best_k is None:
        return "--", 0.0, PAL["muted"]
    fam = best_k.split("_")[0]
    return best_k.replace("_", " ").upper(), best_v, _MODE_COLOR.get(fam, PAL["text"])


# ── estimator plot (3 stacked panels, shared draw logic) ────────────────────
def _style_ax(ax, title, ylabel):
    ax.set_facecolor(PAL["panel"])
    ax.set_title(title, color=PAL["text"], fontsize=10, loc="left", pad=6)
    ax.set_ylabel(ylabel, color=PAL["muted"], fontsize=8)
    ax.tick_params(colors=PAL["muted"], labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(PAL["border"])
    ax.grid(True, color=PAL["grid"], alpha=0.5, linewidth=0.6)


def _legend(ax, **kw):
    leg = ax.legend(
        loc="upper right", fontsize=7, framealpha=0.85,
        facecolor=PAL["panel"], edgecolor=PAL["border"], **kw
    )
    if leg:
        for txt in leg.get_texts():
            txt.set_color(PAL["text"])
    return leg


def _init_estimator_axes(canvas):
    fig = canvas.fig
    fig.patch.set_facecolor(PAL["surface"])
    ax_err = fig.add_subplot(311)
    ax_mode = fig.add_subplot(312)
    ax_res = fig.add_subplot(313)
    ax_om = ax_mode.twinx()

    _style_ax(ax_err, "Estimation Error  (est − measurement)", "error [m]")
    ax_err.axhline(0.0, color=PAL["muted"], linewidth=0.7, alpha=0.6)
    _style_ax(ax_mode, "IMM Product-Model Probability", "probability")
    ax_mode.set_ylim(-0.05, 1.05)
    _style_ax(ax_res, "Residual Norms", "norm [m]")
    ax_res.set_xlabel("time [s]", color=PAL["muted"], fontsize=8)
    # twin omega axis
    ax_om.set_ylabel("ω [rad/s]", color="#ff5c6c", fontsize=8)
    ax_om.tick_params(colors="#ff5c6c", labelsize=7)
    for sp in ax_om.spines.values():
        sp.set_color(PAL["border"])

    lines = {}
    for k, c in _ERR_STYLE:
        (lines[f"err_{k}"],) = ax_err.plot([], [], color=c, linewidth=1.0,
                                           label=f"{k.upper()}")
    for k, c, ls, lbl in _MODE_STYLE:
        (lines[f"mode_{k}"],) = ax_mode.plot([], [], color=c, linestyle=ls,
                                             linewidth=1.2, label=lbl)
    (lines["omega"],) = ax_om.plot([], [], color="#ff5c6c", linewidth=1.0,
                                   linestyle="--", label="ω")
    for k, c, lbl in _RES_STYLE:
        (lines[f"res_{k}"],) = ax_res.plot([], [], color=c, linewidth=1.0, label=lbl)

    _legend(ax_err, ncol=3)
    _legend(ax_mode, ncol=3)
    _legend(ax_res, ncol=3)

    canvas._ax = {"err": ax_err, "mode": ax_mode, "om": ax_om, "res": ax_res}
    canvas._lines = lines
    fig.tight_layout(pad=1.2)


def _autoscale(ax, arrs, floor=0.5, symmetric=False, lo=0.0):
    vals = np.concatenate([a[np.isfinite(a)] for a in arrs if a.size]) if arrs else np.array([])
    if vals.size == 0:
        return
    m = float(np.nanmax(np.abs(vals)))
    if not np.isfinite(m):
        return
    m = max(floor, m * 1.15)
    if symmetric:
        ax.set_ylim(-m, m)
    else:
        ax.set_ylim(lo, m)


def _draw_estimator(canvas, hub):
    data = hub.plot_arrays()
    if data is None:
        return
    t = data["t"]
    n = len(t)
    if n == 0:
        return
    L = canvas._lines
    err = data["err"]
    if err.ndim == 2 and err.shape[0] == n and err.shape[1] >= 3:
        for i, (k, _c) in enumerate(_ERR_STYLE):
            L[f"err_{k}"].set_data(t, err[:, i])
        _autoscale(canvas._ax["err"], [err[:, i] for i in range(3)], symmetric=True)
    for k, *_ in _MODE_STYLE:
        d = data["modes"][k]
        if len(d) == n:
            L[f"mode_{k}"].set_data(t, d)
    om = data["omega"]
    if len(om) == n:
        L["omega"].set_data(t, om)
        _autoscale(canvas._ax["om"], [om], floor=0.1, symmetric=True)
    for key in ("innov", "jump", "errnorm"):
        d = data[key]
        if len(d) == n:
            L[f"res_{key}"].set_data(t, d)
    _autoscale(canvas._ax["res"], [data["innov"], data["jump"], data["errnorm"]],
               floor=0.5, lo=0.0)

    if n >= 2 and t[-1] > t[0]:
        for ax in (canvas._ax["err"], canvas._ax["mode"], canvas._ax["res"]):
            ax.set_xlim(t[0], t[-1])
        canvas._ax["om"].set_xlim(t[0], t[-1])
    canvas.draw_idle()


if _USE_QT:

    class _EstimatorPlot(_Canvas):
        def __init__(self, parent=None):
            self.fig = Figure(figsize=(7, 7), dpi=100)
            super().__init__(self.fig)
            self.setParent(parent)
            _init_estimator_axes(self)

        def update_plot(self, hub):
            _draw_estimator(self, hub)

    class _ClickLabel(QtWidgets.QLabel):
        """Param-name label that reports clicks, so selecting a row works from
        the name as well as the value box."""

        clicked = QtCore.pyqtSignal(str)

        def __init__(self, text, name, parent=None):
            super().__init__(text, parent)
            self._name = name

        def mousePressEvent(self, ev):
            self.clicked.emit(self._name)
            super().mousePressEvent(ev)

    class _ParamEdit(QtWidgets.QLineEdit):
        """Value box that reports focus, so clicking/tabbing into it selects
        the row (QLineEdit has no focus-in signal of its own)."""

        focused = QtCore.pyqtSignal(str)

        def __init__(self, text, name, parent=None):
            super().__init__(text, parent)
            self._name = name

        def focusInEvent(self, ev):
            self.focused.emit(self._name)
            super().focusInEvent(ev)

else:

    class _EstimatorPlot(_Canvas):
        def __init__(self, parent=None):
            self.fig = Figure(figsize=(7, 7), dpi=100)
            super().__init__(self.fig, parent)
            _init_estimator_axes(self)

        def update_plot(self, hub):
            _draw_estimator(self, hub)


# ── main GUI class ─────────────────────────────────────────────────────────
class GuidanceGUI:
    """Non-blocking estimator telemetry & parameter GUI.

    Call ``push_snapshot(...)`` from the runner's main loop to feed telemetry.
    The GUI refreshes itself on its own timer inside the background thread."""

    def __init__(
        self, update_callback=None, param_module="guidance_config", refresh_hz=10.0
    ):
        # type: (Optional[callable], str, float) -> None
        self._hub = _DataHub()
        self._params = _ParamStore(param_module)
        self._docs = _ParamDocs(param_module)
        self._selected_param = None  # type: Optional[str]
        self._refresh_interval_ms = int(1000.0 / max(refresh_hz, 1.0))
        self._running = False
        self._thread = None  # type: Optional[threading.Thread]
        self._gui_ready = threading.Event()

        self._user_callback = update_callback
        self._app = None
        self._root = None

        global _GUI_INSTANCE
        _GUI_INSTANCE = self  # allow external push via singleton

    # ── public API ──────────────────────────────────────────────────────
    def push_snapshot(
        self, pursuer_pos, target_pos, mode_probs=None, t=None, target_est=None,
        status=None, diag=None,
    ):
        """Thread-safe push from runner loop.

        target_pos is the measured target position. target_est (the estimator
        output) feeds the error plot. status carries scalar telemetry (range_m,
        t_go_s, closing_velocity, recovery_state, ...); diag carries estimator
        residual norms (omega, innov_norm, jump_norm, err_norm)."""
        if t is None:
            t = time.monotonic()
        self._hub.push_step(
            t, pursuer_pos, target_pos, mode_probs, target_est, status, diag
        )

    def start(self):
        """Launch the GUI in a background thread (non-blocking)."""
        if self._running:
            return
        self._running = True
        self._gui_ready.clear()
        print(f"[gui] starting ({_BACKEND_NAME} backend)")
        self._thread = threading.Thread(target=self._run_gui, daemon=True)
        self._thread.start()
        if not self._gui_ready.wait(timeout=8.0):
            print("[gui] WARNING: GUI did not become ready within 8 s")

    def stop(self):
        """Shut the GUI down from its own thread and wait for it to exit."""
        if not self._running and self._thread is None:
            return
        self._running = False
        try:
            if _USE_QT and self._app is not None:
                self._app.quit()
            elif not _USE_QT and self._root is not None:
                self._root.after(0, self._safe_tk_quit)
        except Exception:
            pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def tick(self):
        """No-op. Kept for API compatibility; the GUI self-refreshes."""
        return

    # ── internal GUI construction ───────────────────────────────────────
    def _run_gui(self):
        try:
            if _USE_QT:
                self._run_qt()
            else:
                self._run_tk()
        except Exception as exc:
            print(f"[gui] {_BACKEND_NAME} backend crashed during init/run: {exc}")
            traceback.print_exc()
        finally:
            self._running = False
            self._gui_ready.set()  # always release start(), even on failure

    def _safe_tk_quit(self):
        try:
            self._root.quit()
            self._root.destroy()
        except Exception:
            pass

    # ════════════════════ Qt5 implementation ════════════════════════════
    def _run_qt(self):
        self._app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            sys.argv[:1]
        )
        self._app.setStyleSheet(_qt_stylesheet())
        self._win = QtWidgets.QMainWindow()
        self._win.setWindowTitle("Gudum  ·  Estimator Console")
        self._win.resize(1060, 800)

        tabs = QtWidgets.QTabWidget()
        self._win.setCentralWidget(tabs)

        tab_est = QtWidgets.QWidget()
        tabs.addTab(tab_est, "Estimator")
        self._build_estimator_qt(tab_est)

        tab_appr = QtWidgets.QWidget()
        tabs.addTab(tab_appr, "Approach")
        self._build_approach_qt(tab_appr)

        tab_cfg = QtWidgets.QWidget()
        tabs.addTab(tab_cfg, "Config")
        self._build_config_qt(tab_cfg)

        # Crash notification (corner toast), hidden until a crash is detected.
        self._crash_banner_qt = self._build_crash_banner_qt()

        timer = QtCore.QTimer()
        timer.timeout.connect(self._qt_refresh)
        timer.start(self._refresh_interval_ms)
        self._timer = timer

        self._gui_ready.set()
        self._win.show()
        self._app.exec_()

    def _build_crash_banner_qt(self):
        """Compact crash NOTIFICATION pinned to the window's top-right corner.

        Deliberately not a full-window overlay: a crash is exactly the moment
        the estimator plots, range cards and approach log matter most, so the
        alert must be impossible to miss without hiding the data behind it.
        """
        bar = QtWidgets.QWidget(self._win)
        bar.setObjectName("crashBanner")
        bar.setStyleSheet(
            "#crashBanner{background:#2a0b0e;border:2px solid %s;border-radius:6px;}"
            % PAL["danger"]
        )
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(10)
        icon = QtWidgets.QLabel("⚠")
        icon.setStyleSheet("color:%s;font-size:22px;font-weight:bold;" % PAL["danger"])
        title = QtWidgets.QLabel("CRASH DETECTED")
        title.setStyleSheet(
            "color:%s;font-size:15px;font-weight:bold;" % PAL["danger"]
        )
        self._crash_detail_qt = QtWidgets.QLabel("")
        self._crash_detail_qt.setStyleSheet("color:#ffb0b0;font-size:12px;")
        lay.addWidget(icon)
        lay.addWidget(title)
        lay.addWidget(self._crash_detail_qt)
        bar.hide()
        return bar

    def _build_estimator_qt(self, parent):
        layout = QtWidgets.QVBoxLayout(parent)
        self._est_plot_qt = _EstimatorPlot()
        layout.addWidget(self._est_plot_qt)

    def _build_approach_qt(self, parent):
        layout = QtWidgets.QVBoxLayout(parent)
        # status cards
        row = QtWidgets.QHBoxLayout()
        self._cards_qt = {}
        for key, title in _CARD_DEFS:
            box = QtWidgets.QFrame()
            box.setObjectName("card")
            v = QtWidgets.QVBoxLayout(box)
            cap = QtWidgets.QLabel(title)
            cap.setObjectName("cap")
            val = QtWidgets.QLabel("--")
            val.setObjectName("val")
            v.addWidget(cap)
            v.addWidget(val)
            row.addWidget(box)
            self._cards_qt[key] = val
        layout.addLayout(row)

        self._last30_qt = QtWidgets.QLabel("Closest in last 30 s:  -- m")
        self._last30_qt.setObjectName("headline")
        layout.addWidget(self._last30_qt)

        layout.addWidget(_qt_section("Per-30 s window minima"))
        self._win_list_qt = QtWidgets.QListWidget()
        layout.addWidget(self._win_list_qt, stretch=1)

        layout.addWidget(_qt_section("Sub-5 m approaches"))
        self._sub5_list_qt = QtWidgets.QListWidget()
        layout.addWidget(self._sub5_list_qt, stretch=1)

    # ── param detail panel (shared by both backends) ────────────────────
    def _param_detail(self, name):
        # type: (str) -> dict
        """Everything the detail panel shows for one parameter."""
        cur = self._params.snapshot().get(name)
        meta = self._docs.get(name)
        dv, raw = self._docs.default_value(name)
        modified = raw is not None and dv is not None and cur != dv

        summary = meta.get("summary", "")
        desc = meta.get("description", "")
        parts = []
        if summary:
            parts.append(summary)
        if desc and desc != summary:
            parts.append(desc)
        body = "\n\n".join(parts)
        if meta.get("shared") and desc:
            body += "\n\n(this text documents the group this parameter belongs to)"
        if not body:
            body = "No description in guidance_config.py."

        if raw is None:
            default_txt = "—  (no code default)"
        else:
            default_txt = str(dv) if dv is not None else raw
        return {
            "name": name,
            "section": meta.get("section", ""),
            "current": str(cur),
            "default": default_txt,
            "type": type(cur).__name__,
            "modified": modified,
            "editable": self._params.is_editable(name),
            "desc": body,
            "line": meta.get("line"),
        }

    def _select_param(self, name):
        self._selected_param = name
        if _USE_QT:
            self._refresh_detail_qt()
        else:
            self._refresh_detail_tk()

    def _build_config_qt(self, parent):
        outer = QtWidgets.QHBoxLayout(parent)
        left = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        container = QtWidgets.QWidget()
        self._param_grid_qt = QtWidgets.QGridLayout(container)
        scroll.setWidget(container)
        layout.addWidget(scroll)
        btn = QtWidgets.QPushButton("Apply All Changes")
        btn.clicked.connect(self._apply_params_qt)
        layout.addWidget(btn)
        outer.addWidget(left, 3)
        outer.addWidget(self._build_param_detail_qt(), 2)
        self._param_entries_qt = {}  # type: dict
        self._param_labels_qt = {}  # type: dict
        self._rebuild_param_grid_qt()

    def _build_param_detail_qt(self):
        box = QtWidgets.QWidget()
        box.setObjectName("card")
        box.setMinimumWidth(320)
        box.setMaximumWidth(430)
        v = QtWidgets.QVBoxLayout(box)
        self._d_name_qt = QtWidgets.QLabel("Select a parameter")
        self._d_name_qt.setStyleSheet(
            f"color:{PAL['text']}; font-weight:bold; font-size:13px;"
        )
        self._d_name_qt.setWordWrap(True)
        self._d_sect_qt = QtWidgets.QLabel("")
        self._d_sect_qt.setStyleSheet(f"color:{PAL['muted']}; font-size:10px;")
        self._d_sect_qt.setWordWrap(True)
        v.addWidget(self._d_name_qt)
        v.addWidget(self._d_sect_qt)

        form = QtWidgets.QFormLayout()
        self._d_cur_qt = QtWidgets.QLabel("—")
        self._d_def_qt = QtWidgets.QLabel("—")
        self._d_type_qt = QtWidgets.QLabel("—")
        for w in (self._d_cur_qt, self._d_def_qt, self._d_type_qt):
            w.setStyleSheet(f"color:{PAL['text']}; font-family:Consolas;")
        form.addRow("Current", self._d_cur_qt)
        form.addRow("Default", self._d_def_qt)
        form.addRow("Type", self._d_type_qt)
        v.addLayout(form)

        row = QtWidgets.QHBoxLayout()
        self._d_edit_qt = QtWidgets.QLineEdit()
        self._d_edit_qt.returnPressed.connect(self._apply_detail_qt)
        self._d_set_qt = QtWidgets.QPushButton("Set")
        self._d_set_qt.clicked.connect(self._apply_detail_qt)
        row.addWidget(self._d_edit_qt)
        row.addWidget(self._d_set_qt)
        v.addLayout(row)

        self._d_desc_qt = QtWidgets.QTextEdit()
        self._d_desc_qt.setReadOnly(True)
        v.addWidget(self._d_desc_qt, 1)
        self._d_src_qt = QtWidgets.QLabel("")
        self._d_src_qt.setStyleSheet(f"color:{PAL['muted']}; font-size:9px;")
        v.addWidget(self._d_src_qt)
        return box

    def _refresh_detail_qt(self):
        name = self._selected_param
        if not name:
            return
        d = self._param_detail(name)
        self._d_name_qt.setText(d["name"])
        self._d_sect_qt.setText(d["section"])
        self._d_cur_qt.setText(d["current"])
        self._d_def_qt.setText(
            d["default"] + ("    ← modified" if d["modified"] else "")
        )
        self._d_def_qt.setStyleSheet(
            f"color:{PAL['amber'] if d['modified'] else PAL['muted']}; font-family:Consolas;"
        )
        self._d_type_qt.setText(d["type"])
        self._d_edit_qt.setText(d["current"])
        self._d_edit_qt.setEnabled(d["editable"])
        self._d_set_qt.setEnabled(d["editable"])
        self._d_desc_qt.setPlainText(
            d["desc"] if d["editable"]
            else d["desc"] + "\n\nRead-only: not a scalar the editor can write back."
        )
        self._d_src_qt.setText(
            f"guidance_config.py:{d['line']}" if d.get("line") else ""
        )
        for pname, lbl in self._param_labels_qt.items():
            lbl.setStyleSheet(
                f"color:{PAL['accent']}; font-weight:bold;" if pname == name
                else f"color:{PAL['muted']};"
            )

    def _apply_detail_qt(self):
        name = self._selected_param
        if not name or not self._params.is_editable(name):
            return
        if self._params.set_param(name, self._d_edit_qt.text().strip()):
            print(f"[gui] {RC}{name} updated{NC}")
            self._rebuild_param_grid_qt()
            self._refresh_detail_qt()

    def _rebuild_param_grid_qt(self):
        while self._param_grid_qt.count():
            item = self._param_grid_qt.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._param_entries_qt.clear()
        self._param_labels_qt.clear()
        params = self._params.snapshot()
        for i, (name, val) in enumerate(sorted(params.items())):
            lbl = _ClickLabel(name, name)
            lbl.setStyleSheet(f"color:{PAL['muted']};")
            lbl.setToolTip(str(val))
            lbl.clicked.connect(self._select_param)
            entry = _ParamEdit(str(val), name)
            entry.setMinimumWidth(140)
            editable = self._params.is_editable(name)
            entry.setReadOnly(not editable)
            if not editable:
                entry.setToolTip("read-only (non-scalar parameter)")
            entry.focused.connect(self._select_param)
            self._param_grid_qt.addWidget(lbl, i, 0)
            self._param_grid_qt.addWidget(entry, i, 1)
            self._param_entries_qt[name] = entry
            self._param_labels_qt[name] = lbl
        if self._selected_param in self._param_labels_qt:
            self._refresh_detail_qt()

    def _apply_params_qt(self):
        changed = 0
        for name, entry in self._param_entries_qt.items():
            if entry.isReadOnly():
                continue
            new_str = entry.text().strip()
            old = self._params.snapshot().get(name)
            if old is not None and str(old) == new_str:
                continue
            if self._params.set_param(name, new_str):
                changed += 1
        if changed:
            self._rebuild_param_grid_qt()
            print(f"[gui] {RC}{changed} parameter(s) updated{NC}")

    def _qt_refresh(self):
        try:
            if hasattr(self, "_est_plot_qt"):
                self._est_plot_qt.update_plot(self._hub)
            st, last30, last_mode = self._hub.status_snapshot()
            win_log, sub5_log = self._hub.log_snapshot()
            vals = _format_cards(st, last_mode)
            for key, val in vals.items():
                if key in self._cards_qt:
                    self._cards_qt[key].setText(val)
            self._last30_qt.setText(
                f"Closest in last 30 s:  {last30:.2f} m"
                if np.isfinite(last30) else "Closest in last 30 s:  -- m"
            )
            # crash notification: corner toast on a detected crash, hide otherwise.
            # Re-anchored every tick so it tracks window resizes (the widget is
            # a free-floating child of the window, outside any layout).
            if hasattr(self, "_crash_banner_qt"):
                if st.get("crashed"):
                    self._crash_detail_qt.setText(
                        f"vibration {st.get('vibe_max', 0):.0f}"
                        f"   ·   hits this run {st.get('hits', 0)}"
                    )
                    self._crash_banner_qt.adjustSize()
                    bw = self._crash_banner_qt.width()
                    self._crash_banner_qt.move(
                        max(8, self._win.width() - bw - 18), 12
                    )
                    self._crash_banner_qt.raise_()
                    self._crash_banner_qt.show()
                else:
                    self._crash_banner_qt.hide()
            self._win_list_qt.clear()
            for start, mn, mnw in reversed(win_log):
                self._win_list_qt.addItem(f"{start} → +30s    min {mn:.2f} m  @ {mnw}")
            self._sub5_list_qt.clear()
            for mn, mnw in reversed(sub5_log):
                self._sub5_list_qt.addItem(f"{mnw}    {mn:.2f} m")
            if self._user_callback:
                self._user_callback()
        except Exception:
            traceback.print_exc()

    # ════════════════════ Tk implementation ═════════════════════════════
    def _run_tk(self):
        self._root = tk.Tk()
        self._root.title("Gudum  ·  Estimator Console")
        self._root.geometry("1060x800")
        self._root.configure(bg=PAL["bg"])
        self._root.protocol("WM_DELETE_WINDOW", self._on_tk_close)
        _apply_tk_theme(self._root)

        header = tk.Frame(self._root, bg=PAL["surface"], height=44)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)
        tk.Label(
            header, text="  ● Gudum Estimator Console", bg=PAL["surface"],
            fg=PAL["text"], font=("Segoe UI", 13, "bold"),
        ).pack(side="left", padx=8)
        self._hdr_status_tk = tk.Label(
            header, text="waiting for telemetry…", bg=PAL["surface"],
            fg=PAL["muted"], font=("Segoe UI", 10),
        )
        self._hdr_status_tk.pack(side="right", padx=14)

        nb = ttk.Notebook(self._root)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        tab_est = ttk.Frame(nb, style="Card.TFrame")
        nb.add(tab_est, text="  Estimator  ")
        self._build_estimator_tk(tab_est)

        tab_appr = ttk.Frame(nb, style="Card.TFrame")
        nb.add(tab_appr, text="  Approach  ")
        self._build_approach_tk(tab_appr)

        tab_cfg = ttk.Frame(nb, style="Card.TFrame")
        nb.add(tab_cfg, text="  Config  ")
        self._build_config_tk(tab_cfg)

        # Crash notification (corner toast), built hidden and placed on demand.
        self._crash_banner_tk = self._build_crash_banner_tk()

        self._gui_ready.set()
        self._root.after(self._refresh_interval_ms, self._tk_tick)
        self._root.mainloop()

    def _build_crash_banner_tk(self):
        """Compact crash NOTIFICATION pinned to the window's top-right corner.

        Mirrors the Qt toast: alerts without hiding the plots and cards that
        matter most at exactly the moment a crash is detected.
        """
        bar = tk.Frame(self._root, bg="#2a0b0e",
                       highlightbackground=PAL["danger"], highlightthickness=2)
        tk.Label(bar, text="⚠", bg="#2a0b0e", fg=PAL["danger"],
                 font=("Segoe UI", 16, "bold")).pack(side="left", padx=(10, 6), pady=6)
        tk.Label(bar, text="CRASH DETECTED", bg="#2a0b0e", fg=PAL["danger"],
                 font=("Segoe UI", 12, "bold")).pack(side="left", pady=6)
        self._crash_detail_tk = tk.Label(
            bar, text="", bg="#2a0b0e", fg="#ffb0b0", font=("Segoe UI", 10))
        self._crash_detail_tk.pack(side="left", padx=(10, 12), pady=6)
        return bar

    def _build_estimator_tk(self, parent):
        self._est_plot_tk = _EstimatorPlot(parent)
        w = self._est_plot_tk.get_tk_widget()
        w.configure(bg=PAL["surface"], highlightthickness=0)
        w.pack(fill="both", expand=True, padx=6, pady=6)

    def _build_approach_tk(self, parent):
        outer = tk.Frame(parent, bg=PAL["bg"])
        outer.pack(fill="both", expand=True, padx=10, pady=10)

        # ── status cards row ──
        cards = tk.Frame(outer, bg=PAL["bg"])
        cards.pack(fill="x", pady=(0, 10))
        self._cards_tk = {}
        for i, (key, title) in enumerate(_CARD_DEFS):
            card = tk.Frame(cards, bg=PAL["panel"], highlightbackground=PAL["border"],
                            highlightthickness=1)
            card.grid(row=0, column=i, sticky="nsew", padx=4)
            cards.grid_columnconfigure(i, weight=1)
            tk.Label(card, text=title, bg=PAL["panel"], fg=PAL["muted"],
                     font=("Segoe UI", 9)).pack(anchor="w", padx=10, pady=(8, 0))
            val = tk.Label(card, text="--", bg=PAL["panel"], fg=PAL["text"],
                           font=("Segoe UI", 18, "bold"))
            val.pack(anchor="w", padx=10, pady=(0, 8))
            self._cards_tk[key] = val

        # ── headline: closest in last 30 s ──
        hl = tk.Frame(outer, bg=PAL["panel"], highlightbackground=PAL["border"],
                      highlightthickness=1)
        hl.pack(fill="x", pady=(0, 10))
        tk.Label(hl, text="CLOSEST APPROACH · LAST 30 s", bg=PAL["panel"],
                 fg=PAL["muted"], font=("Segoe UI", 9)).pack(anchor="w", padx=12,
                                                             pady=(8, 0))
        self._last30_tk = tk.Label(hl, text="-- m", bg=PAL["panel"], fg=PAL["accent"],
                                   font=("Segoe UI", 26, "bold"))
        self._last30_tk.pack(anchor="w", padx=12, pady=(0, 10))

        # ── two log tables side by side ──
        logs = tk.Frame(outer, bg=PAL["bg"])
        logs.pack(fill="both", expand=True)
        logs.grid_columnconfigure(0, weight=1)
        logs.grid_columnconfigure(1, weight=1)
        logs.grid_rowconfigure(0, weight=1)

        self._win_tree_tk = self._make_log_table_tk(
            logs, 0, "Per-30 s window minima",
            ("window", "min", "at"), (150, 90, 90),
        )
        self._sub5_tree_tk = self._make_log_table_tk(
            logs, 1, "Sub-5 m approaches", ("min range", "at"), (140, 120),
        )

    def _make_log_table_tk(self, parent, col, title, columns, widths):
        holder = tk.Frame(parent, bg=PAL["panel"], highlightbackground=PAL["border"],
                          highlightthickness=1)
        holder.grid(row=0, column=col, sticky="nsew", padx=4)
        tk.Label(holder, text=title.upper(), bg=PAL["panel"], fg=PAL["muted"],
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=10, pady=(8, 4))
        tree = ttk.Treeview(holder, columns=columns, show="headings",
                            style="Card.Treeview", height=12)
        for c, w in zip(columns, widths):
            tree.heading(c, text=c)
            tree.column(c, width=w, anchor="center")
        tree.tag_configure("hit", foreground=PAL["ok"])
        tree.tag_configure("near", foreground=PAL["amber"])
        tree.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        return tree

    def _build_config_tk(self, parent):
        outer = tk.Frame(parent, bg=PAL["bg"])
        outer.pack(fill="both", expand=True)

        # Detail panel first so it keeps its fixed width when the list grows.
        right = tk.Frame(outer, bg=PAL["panel"], width=360,
                         highlightbackground=PAL["border"], highlightthickness=1)
        right.pack(side="right", fill="y", padx=(4, 6), pady=6)
        right.pack_propagate(False)
        self._build_param_detail_tk(right)

        left = tk.Frame(outer, bg=PAL["bg"])
        left.pack(side="left", fill="both", expand=True)
        btn = ttk.Button(left, text="Apply All Changes",
                         command=self._apply_params_tk, style="Accent.TButton")
        btn.pack(side="bottom", pady=8)

        wrap = tk.Frame(left, bg=PAL["bg"])
        wrap.pack(fill="both", expand=True)
        canvas = tk.Canvas(wrap, borderwidth=0, bg=PAL["bg"], highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
        scroll_frame = tk.Frame(canvas, bg=PAL["bg"])
        scroll_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self._param_entries_tk = {}  # type: dict
        self._param_labels_tk = {}  # type: dict
        self._param_frame_tk = scroll_frame
        self._rebuild_param_grid_tk()

    def _build_param_detail_tk(self, parent):
        pad = {"padx": 10}
        self._d_name_tk = tk.Label(
            parent, text="Select a parameter", bg=PAL["panel"], fg=PAL["text"],
            font=("Segoe UI", 11, "bold"), anchor="w", justify="left", wraplength=330,
        )
        self._d_name_tk.pack(fill="x", pady=(10, 0), **pad)
        self._d_sect_tk = tk.Label(
            parent, text="", bg=PAL["panel"], fg=PAL["muted"], font=("Segoe UI", 8),
            anchor="w", justify="left", wraplength=330,
        )
        self._d_sect_tk.pack(fill="x", **pad)

        grid = tk.Frame(parent, bg=PAL["panel"])
        grid.pack(fill="x", pady=(8, 4), **pad)

        def _row(r, label):
            tk.Label(grid, text=label, bg=PAL["panel"], fg=PAL["muted"],
                     font=("Segoe UI", 9), anchor="w", width=8).grid(
                row=r, column=0, sticky="w")
            val = tk.Label(grid, text="—", bg=PAL["panel"], fg=PAL["text"],
                           font=("Consolas", 9), anchor="w", justify="left",
                           wraplength=240)
            val.grid(row=r, column=1, sticky="w")
            return val

        self._d_cur_tk = _row(0, "Current")
        self._d_def_tk = _row(1, "Default")
        self._d_type_tk = _row(2, "Type")

        edit = tk.Frame(parent, bg=PAL["panel"])
        edit.pack(fill="x", pady=(6, 6), **pad)
        self._d_var_tk = tk.StringVar()
        self._d_entry_tk = ttk.Entry(edit, textvariable=self._d_var_tk, width=18)
        self._d_entry_tk.pack(side="left", fill="x", expand=True)
        self._d_entry_tk.bind("<Return>", lambda e: self._apply_detail_tk())
        self._d_set_tk = ttk.Button(edit, text="Set", width=5,
                                    command=self._apply_detail_tk,
                                    style="Accent.TButton")
        self._d_set_tk.pack(side="right", padx=(6, 0))

        tk.Label(parent, text="DESCRIPTION", bg=PAL["panel"], fg=PAL["muted"],
                 font=("Segoe UI", 8, "bold"), anchor="w").pack(
            fill="x", pady=(4, 0), **pad)
        # width=1: let fill/expand size it to the panel instead of the Text's
        # natural request (which is far wider than the 360 px panel).
        self._d_desc_tk = tk.Text(
            parent, bg=PAL["bg"], fg=PAL["text"], wrap="word", font=("Segoe UI", 9),
            relief="flat", width=1, height=12, highlightthickness=0, padx=6, pady=6,
        )
        self._d_desc_tk.pack(fill="both", expand=True, pady=(2, 6), **pad)
        self._d_desc_tk.configure(state="disabled")
        self._d_src_tk = tk.Label(parent, text="", bg=PAL["panel"], fg=PAL["muted"],
                                  font=("Consolas", 8), anchor="w")
        self._d_src_tk.pack(fill="x", pady=(0, 8), **pad)

    def _refresh_detail_tk(self):
        name = self._selected_param
        if not name:
            return
        d = self._param_detail(name)
        self._d_name_tk.configure(text=d["name"])
        self._d_sect_tk.configure(text=d["section"])
        self._d_cur_tk.configure(text=d["current"])
        self._d_def_tk.configure(
            text=d["default"] + ("   ← modified" if d["modified"] else ""),
            fg=PAL["amber"] if d["modified"] else PAL["muted"],
        )
        self._d_type_tk.configure(text=d["type"])
        self._d_var_tk.set(d["current"])
        state = "normal" if d["editable"] else "disabled"
        self._d_entry_tk.configure(state=state)
        self._d_set_tk.configure(state=state)
        body = d["desc"]
        if not d["editable"]:
            body += "\n\nRead-only: not a scalar the editor can write back."
        self._d_desc_tk.configure(state="normal")
        self._d_desc_tk.delete("1.0", "end")
        self._d_desc_tk.insert("1.0", body)
        self._d_desc_tk.configure(state="disabled")
        self._d_src_tk.configure(
            text=f"guidance_config.py:{d['line']}" if d.get("line") else ""
        )
        for pname, lbl in self._param_labels_tk.items():
            lbl.configure(fg=PAL["accent"] if pname == name else PAL["muted"])

    def _apply_detail_tk(self):
        name = self._selected_param
        if not name or not self._params.is_editable(name):
            return
        if self._params.set_param(name, self._d_var_tk.get().strip()):
            print(f"[gui] {RC}{name} updated{NC}")
            self._rebuild_param_grid_tk()
            self._refresh_detail_tk()

    def _rebuild_param_grid_tk(self):
        for w in self._param_frame_tk.winfo_children():
            w.destroy()
        self._param_entries_tk.clear()
        self._param_labels_tk.clear()
        params = self._params.snapshot()
        for i, (name, val) in enumerate(sorted(params.items())):
            lbl = tk.Label(self._param_frame_tk, text=name, width=32, anchor="e",
                           bg=PAL["bg"], fg=PAL["muted"], font=("Consolas", 9),
                           cursor="hand2")
            lbl.grid(row=i, column=0, sticky="e", padx=(6, 4), pady=1)
            lbl.bind("<Button-1>", lambda e, n=name: self._select_param(n))
            sv = tk.StringVar(value=str(val))
            editable = self._params.is_editable(name)
            entry = ttk.Entry(self._param_frame_tk, textvariable=sv, width=24,
                              state=("normal" if editable else "readonly"))
            entry.grid(row=i, column=1, sticky="w", padx=(0, 6), pady=1)
            entry.bind("<FocusIn>", lambda e, n=name: self._select_param(n))
            entry.bind("<Button-1>", lambda e, n=name: self._select_param(n))
            self._param_entries_tk[name] = (sv, editable)
            self._param_labels_tk[name] = lbl
        if self._selected_param in self._param_labels_tk:
            self._refresh_detail_tk()

    def _apply_params_tk(self):
        changed = 0
        for name, (sv, editable) in list(self._param_entries_tk.items()):
            if not editable:
                continue
            new_str = sv.get().strip()
            old = self._params.snapshot().get(name)
            if old is not None and str(old) == new_str:
                continue
            if self._params.set_param(name, new_str):
                changed += 1
        if changed:
            self._rebuild_param_grid_tk()
            print(f"[gui] {RC}{changed} parameter(s) updated{NC}")

    def _tk_tick(self):
        if not self._running:
            return
        try:
            if hasattr(self, "_est_plot_tk"):
                self._est_plot_tk.update_plot(self._hub)

            st, last30, last_mode = self._hub.status_snapshot()
            win_log, sub5_log = self._hub.log_snapshot()

            # status cards
            vals = _format_cards(st, last_mode)
            for key, lbl in self._cards_tk.items():
                lbl.config(text=vals.get(key, "--"), fg=_card_color(key, st, last_mode))

            self._last30_tk.config(
                text=(f"{last30:.2f} m" if np.isfinite(last30) else "-- m")
            )

            # crash notification: corner toast on a detected crash, hide otherwise
            if hasattr(self, "_crash_banner_tk"):
                if st.get("crashed"):
                    self._crash_detail_tk.config(
                        text=f"vibration {st.get('vibe_max', 0):.0f}"
                             f"   ·   hits this run {st.get('hits', 0)}"
                    )
                    self._crash_banner_tk.place(
                        relx=1.0, x=-18, y=12, anchor="ne"
                    )
                    self._crash_banner_tk.lift()
                else:
                    self._crash_banner_tk.place_forget()
            # header status: show guidance range/FSM when a pursuer is present,
            # otherwise an estimator-live indicator (filterwndr sends no range).
            rng = st.get("range_m")
            fsm = st.get("recovery_state", "--")
            n_samp, last_err = self._hub.header_snapshot()
            if rng is not None:
                self._hdr_status_tk.config(
                    text=f"range {rng:.1f} m   ·   FSM {fsm}", fg=PAL["text"]
                )
            elif n_samp > 0:
                xy_l, _p, _c = _dominant(last_mode, ("cv_xy", "ct_xy", "ca_xy"))
                err_txt = f"err {last_err:.2f} m" if np.isfinite(last_err) else "err --"
                self._hdr_status_tk.config(
                    text=f"● estimator live · {xy_l.split()[0]} · {err_txt}",
                    fg=PAL["ok"],
                )

            # per-window minima table
            self._refill_tree(self._win_tree_tk, [
                ((f"{start} +30s", f"{mn:.2f} m", mnw), _range_tag(mn))
                for (start, mn, mnw) in reversed(win_log)
            ])
            # sub-5 m table
            self._refill_tree(self._sub5_tree_tk, [
                ((f"{mn:.2f} m", mnw), _range_tag(mn))
                for (mn, mnw) in reversed(sub5_log)
            ])

            if self._user_callback:
                self._user_callback()
        except Exception:
            traceback.print_exc()
        finally:
            if self._running and self._root is not None:
                self._root.after(self._refresh_interval_ms, self._tk_tick)

    @staticmethod
    def _refill_tree(tree, rows):
        tree.delete(*tree.get_children())
        for values, tag in rows:
            tree.insert("", "end", values=values, tags=(tag,) if tag else ())

    def _on_tk_close(self):
        self._running = False
        self._safe_tk_quit()


# ── status-card definitions & formatting (shared) ───────────────────────────
_CARD_DEFS = (
    ("range", "RANGE"),
    ("tgo", "t_go"),
    ("closing", "CLOSING"),
    ("xy_mode", "XY MODE"),
    ("z_mode", "Z MODE"),
    ("fsm", "FSM STATE"),
    ("hits", "HITS"),
)


def _format_cards(st, last_mode):
    rng = st.get("range_m")
    tgo = st.get("t_go_s")
    vc = st.get("closing_velocity")
    xy_l, xy_p, _ = _dominant(last_mode, ("cv_xy", "ct_xy", "ca_xy"))
    z_l, z_p, _ = _dominant(last_mode, ("cv_z", "ca_z"))
    tgo_str = "--"
    if tgo is not None:
        tgo_str = f"{tgo:.2f} s" if np.isfinite(tgo) else "∞"
    return {
        "range": f"{rng:.1f} m" if rng is not None else "--",
        "tgo": tgo_str,
        "closing": f"{vc:.1f} m/s" if vc is not None else "--",
        "xy_mode": f"{xy_l.split()[0]} {xy_p:.0%}" if last_mode else "--",
        "z_mode": f"{z_l.split()[0]} {z_p:.0%}" if last_mode else "--",
        "fsm": st.get("recovery_state", "--"),
        "hits": str(st["hits"]) if "hits" in st else "--",
    }


def _card_color(key, st, last_mode):
    if key == "fsm":
        return _FSM_COLOR.get(st.get("recovery_state", ""), PAL["text"])
    if key == "range":
        rng = st.get("range_m")
        if rng is not None and rng < _SUB5_M:
            return PAL["ok"]
        return PAL["text"]
    if key == "xy_mode":
        _l, _p, c = _dominant(last_mode, ("cv_xy", "ct_xy", "ca_xy"))
        return c if last_mode else PAL["text"]
    if key == "z_mode":
        _l, _p, c = _dominant(last_mode, ("cv_z", "ca_z"))
        return c if last_mode else PAL["text"]
    if key == "hits":
        return PAL["ok"] if st.get("hits", 0) > 0 else PAL["text"]
    return PAL["text"]


def _range_tag(r):
    if r < 2.0:
        return "hit"
    if r < _SUB5_M:
        return "near"
    return ""


# ── Tk theming ──────────────────────────────────────────────────────────────
def _apply_tk_theme(root):
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    style.configure("Card.TFrame", background=PAL["bg"])
    style.configure("TFrame", background=PAL["bg"])
    style.configure("TLabel", background=PAL["bg"], foreground=PAL["text"])
    # Notebook
    style.configure("TNotebook", background=PAL["bg"], borderwidth=0)
    style.configure("TNotebook.Tab", background=PAL["surface"], foreground=PAL["muted"],
                    padding=(16, 8), borderwidth=0, font=("Segoe UI", 10, "bold"))
    style.map("TNotebook.Tab",
              background=[("selected", PAL["panel"])],
              foreground=[("selected", PAL["primary"])])
    # Entry
    style.configure("TEntry", fieldbackground=PAL["surface"], foreground=PAL["text"],
                    bordercolor=PAL["border"], insertcolor=PAL["text"])
    # Buttons
    style.configure("TButton", background=PAL["surface"], foreground=PAL["text"],
                    bordercolor=PAL["border"], focuscolor=PAL["primary"])
    style.map("TButton", background=[("active", PAL["panel"])])
    style.configure("Accent.TButton", background=PAL["primary"], foreground="#0b1220",
                    font=("Segoe UI", 10, "bold"))
    style.map("Accent.TButton", background=[("active", PAL["accent"])])
    # Scrollbar
    style.configure("TScrollbar", background=PAL["surface"], troughcolor=PAL["bg"],
                    bordercolor=PAL["bg"], arrowcolor=PAL["muted"])
    # Treeview
    style.configure("Card.Treeview", background=PAL["panel"], fieldbackground=PAL["panel"],
                    foreground=PAL["text"], borderwidth=0, rowheight=22,
                    font=("Consolas", 9))
    style.configure("Card.Treeview.Heading", background=PAL["surface"],
                    foreground=PAL["muted"], borderwidth=0, font=("Segoe UI", 9, "bold"))
    style.map("Card.Treeview", background=[("selected", PAL["border"])])


# ── Qt theming helpers ──────────────────────────────────────────────────────
def _qt_stylesheet():
    return f"""
    QMainWindow, QWidget {{ background: {PAL['bg']}; color: {PAL['text']};
        font-family: 'Segoe UI'; font-size: 11pt; }}
    QTabWidget::pane {{ border: 0; }}
    QTabBar::tab {{ background: {PAL['surface']}; color: {PAL['muted']};
        padding: 8px 16px; font-weight: bold; }}
    QTabBar::tab:selected {{ background: {PAL['panel']}; color: {PAL['primary']}; }}
    QFrame#card {{ background: {PAL['panel']}; border: 1px solid {PAL['border']};
        border-radius: 6px; }}
    QLabel#cap {{ color: {PAL['muted']}; font-size: 9pt; }}
    QLabel#val {{ color: {PAL['text']}; font-size: 18pt; font-weight: bold; }}
    QLabel#headline {{ color: {PAL['accent']}; font-size: 16pt; font-weight: bold; }}
    QLabel#section {{ color: {PAL['muted']}; font-weight: bold; }}
    QListWidget {{ background: {PAL['panel']}; border: 1px solid {PAL['border']};
        border-radius: 6px; }}
    QLineEdit {{ background: {PAL['surface']}; border: 1px solid {PAL['border']};
        border-radius: 4px; padding: 2px 4px; }}
    QPushButton {{ background: {PAL['primary']}; color: #0b1220; font-weight: bold;
        border-radius: 6px; padding: 8px; }}
    QPushButton:hover {{ background: {PAL['accent']}; }}
    QScrollArea {{ border: 0; }}
    """


if _USE_QT:
    def _qt_section(text):
        lbl = QtWidgets.QLabel(text)
        lbl.setObjectName("section")
        return lbl


# ── convenience helpers ────────────────────────────────────────────────────
def start_gui(param_module="guidance_config", refresh_hz=10.0):
    """Convenience: create & start the GUI, returning the handle."""
    global _GUI_INSTANCE
    gui = GuidanceGUI(update_callback=None, param_module=param_module,
                      refresh_hz=refresh_hz)
    gui.start()
    _GUI_INSTANCE = gui
    return gui


def push_snapshot(pursuer_pos, target_pos, mode_probs=None, target_est=None,
                  status=None, diag=None):
    """Push a telemetry snapshot from a runner without holding a GUI reference.

    target_pos is the measured target position; target_est is the estimator
    output (feeds the error plot). status = scalar telemetry (range_m, t_go_s,
    closing_velocity, recovery_state); diag = residual norms (omega, innov_norm,
    jump_norm, err_norm)."""
    global _GUI_INSTANCE
    if _GUI_INSTANCE is not None:
        _GUI_INSTANCE.push_snapshot(
            pursuer_pos, target_pos, mode_probs, target_est=target_est,
            status=status, diag=diag,
        )


def gui_tick():
    """Kept for API compatibility. The GUI self-refreshes; this is a no-op."""
    global _GUI_INSTANCE
    if _GUI_INSTANCE is not None:
        _GUI_INSTANCE.tick()


# ── standalone test ────────────────────────────────────────────────────────
if __name__ == "__main__":
    gui = GuidanceGUI(param_module="guidance_config", refresh_hz=20)
    gui.start()

    t0 = time.monotonic()
    pp = np.array([0.0, 0.0, -30.0])
    tp = np.array([120.0, 40.0, -30.0])
    states = ["CHASE", "CHASE", "CHASE", "HOLD", "REENGAGE"]

    try:
        for step in range(4000):
            if not gui._running:
                break
            t = time.monotonic() - t0
            # close the gap so we exercise the sub-5 m / window logic
            gap = tp - pp
            pp += gap * 0.01 + np.array([0.02 * math.sin(t), 0.0, 0.0])
            tp[0] += 0.15
            tp[1] += 0.2 * math.cos(t * 0.3)
            tp[2] += 0.02 * math.sin(t * 0.2)

            probs = {
                "cv_xy": 0.5 + 0.2 * math.sin(t * 0.3),
                "ct_xy": 0.3 + 0.2 * math.cos(t * 0.25),
                "ca_xy": 0.2,
                "cv_z": 0.7,
                "ca_z": 0.3,
            }
            est = tp + np.array([0.3 * math.sin(t * 0.7), 0.2 * math.cos(t * 0.9),
                                 0.1 * math.sin(t)])
            rng = float(np.linalg.norm(tp - pp))
            status = {
                "range_m": rng,
                "t_go_s": max(0.0, rng / 12.0),
                "closing_velocity": 8.0 + 2.0 * math.sin(t),
                "recovery_state": states[(step // 200) % len(states)],
            }
            diag = {
                "omega": 0.15 * math.sin(t * 0.4),
                "innov_norm": 0.2 + 0.1 * abs(math.sin(t)),
                "jump_norm": 0.15 + 0.1 * abs(math.cos(t)),
                "err_norm": 0.1 + 0.05 * abs(math.sin(t * 1.3)),
            }
            gui.push_snapshot(pp.copy(), tp.copy(), probs, target_est=est,
                              status=status, diag=diag)
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass

    gui.stop()
    print("Done.")
