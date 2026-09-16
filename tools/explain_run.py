#!/usr/bin/env python3
"""explain_run.py - translates the logs of a run into a human timeline.

WHY THERE THERE (2026-08-05): our logs told "what numbers were there", not "what happened". "The
target was coming towards me but MPC seems to be running away," the user said in a video; To verify
this, the angle between the target's direction of travel and the bearing had to be calculated
MANUALLY, because no logs stated "was this encounter a tail chase or head-on?" This tool answers
that question -- and the next ten questions -- in seconds.

USAGE:
    python3 tools/explain_run.py run/trials/mpc_straight_20260804_191600
    python3 tools/explain_run.py --visual guidance_allstar/logs/visual_mpc_*.csv
    python3 tools/explain_run.py <klasor> --full-value     # write down each event (summarize)

WHAT DOES 1. Finds the four logs of the run and aligns them to COMMON TIME:
guided_<method>_<stamp>.csv (visual guidance, t = time.monotonic()) guided_follow_<imprint>.csv
(positioned guidance, wall_time = monotonic) mpc_diagnosis_<stamp>.csv (MPC internal diagnostics, t
= monotonic) bbox.log (detector text, NO TIMESTAMP) The first three files use the SAME
CLOCK_MONOTONIC clock (in Linux this clock is common between processes), so they are directly
aligned. The new display CSVs also have the t_unix; it also allows binding to the absolute clock
(this is the only hyperlink for bbox.log and video files). 2. Extracts the target's state
(position/velocity/acceleration) and encounter geometry. On the new CSVs these are READY in the
ref_* columns. It is not present in the old races; then it is derived from the positional log
(meas_x/y/z) -- so the tool runs in the PAST runs as well. 3. Achieve the timeline with Turkish
sentences.

IMPORTANT RULE REMINDER: ref_* columns and everything this tool produces are for ANALYSIS ONLY.
Guidance uses only RANGE from the target's telemetry (user rule, 2026-08-03). This vehicle feeds
nothing back to my drive.
"""

import argparse
import csv
import glob
import math
import os
import re
import sys
from datetime import datetime
from pathlib import Path

ROOT_VALUE = Path(__file__).resolve().parent.parent
LOG_DIRECTORY = ROOT_VALUE / 'guidance_allstar' / 'logs'
ANSI = re.compile(r'\x1b\[[0-9;]*m')

# Encounter type thresholds -- must be the SAME as visual_base.py.
TYPE_HEAD_TOWARD_HEAD_DEG = 60.0
TYPE_TAIL_DEG = 120.0

TYPE_TEXT = {
    'head_toward_head': 'HEAD TOWARD_HEAD',
    'tail': 'TAIL TAKIBI',
    'crossing': 'CROSSING',
    'stationary': 'TARGET IS STILL',
    '': 'bilinmiyor',
}


# --------------------------------------------------------------- yardimcilar

def _f(row_value, key_value):
    """Converts cell CSV to float; if empty/corrupt/missing then None."""
    raw_value = row_value.get(key_value, '')
    if raw_value in ('', None):
        return None
    try:
        d = float(raw_value)
    except (TypeError, ValueError):
        return None
    return d if math.isfinite(d) else None


def _read_value(path_value):
    with open(path_value, newline='') as f:
        return list(csv.DictReader(f))


def _direction(deg):
    """Writes the degree along with the compass name: '090 (east)'."""
    if deg is None:
        return '---'
    names_item = ['north_value', 'kuzeydogu', 'east_value', 'guneydogu',
             'guney', 'guneybati', 'bati', 'kuzeybati']
    return f"{deg:03.0f} ({names_item[int((deg % 360) / 45 + 0.5) % 8]})"


def _type_determine(approximation_deg, target_speed):
    if target_speed is not None and target_speed < 1.0:
        return 'stationary'
    if approximation_deg is None:
        return ''
    if approximation_deg < TYPE_HEAD_TOWARD_HEAD_DEG:
        return 'head_toward_head'
    if approximation_deg > TYPE_TAIL_DEG:
        return 'tail'
    return 'crossing'


def _duration(s):
    return f"{s:6.1f} s"


# ------------------------------------------------------------- file finder

def files_find(trial_dir, visual_arg=None):
    """Finds log files of the run. Doner: dict."""
    d = {'trial': trial_dir, 'visual': None, 'position_based': None,
         'mpc_diagnostic': None, 'bbox': None, 'event_value': None}

    if visual_arg:
        d['visual'] = Path(visual_arg)
    elif trial_dir is not None:
        # MOST RELIABLE LINK: the image process writes its path CSV to stdout and scenario.sh redirects it to
        # display.log. Don't guess the name.
        gl = trial_dir / 'visual.log'
        if gl.exists():
            m = re.search(r'\] log: (\S+\.csv)',
                          ANSI.sub('', gl.read_text(errors='replace')))
            if m and Path(m.group(1)).exists():
                d['visual'] = Path(m.group(1))
        if d['visual'] is None:
            # Backup: CSV with the first image AFTER the stamp in the folder name.
            m = re.search(r'([a-z0-9]+)_.*_(\d{8}_\d{6})$', trial_dir.name)
            if m:
                method_value, stamp_value = m.group(1), m.group(2)
                candidate_value = sorted(LOG_DIRECTORY.glob(f'visual_{method_value}_*.csv'))
                after_value = [a for a in candidate_value
                         if re.search(r'(\d{8}_\d{6})', a.name).group(1) >= stamp_value]
                if after_value:
                    d['visual'] = after_value[0]

    if d['visual'] is not None:
        m = re.search(r'(\d{8}_\d{6})', d['visual'].name)
        if m:
            diagnostic = LOG_DIRECTORY / f"mpc_diagnostic_{m.group(1)}.csv"
            if diagnostic.exists():
                d['mpc_diagnostic'] = diagnostic
        event_value = Path(str(d['visual']).replace('.csv', '_event.csv'))
        if event_value.exists():
            d['event_value'] = event_value

    if trial_dir is not None and (trial_dir / 'bbox.log').exists():
        d['bbox'] = trial_dir / 'bbox.log'

    # Positioned log: candidate with OVERLAPPING monotonic intervals is selected (more robust than name
    # guess; two runs may have started at the same minute).
    if d['visual'] is not None:
        d['position_based'] = _overlapping_position_based(d['visual'])
    if d['position_based'] is None and trial_dir is not None:
        # The image process may not have produced any CSV (e.g. pid_ellipse 20260803_175339: HOME_POSITION could
        # not be retrieved and the process crashed). The run can still be described -- positional phase alone
        # speaks volumes.
        d['position_based'] = _from_stamp_position_based(trial_dir)
    return d


def _from_stamp_position_based(trial_dir):
    """Select the FIRST positioned log starting after the stamp in the folder name."""
    m = re.search(r'(\d{8}_\d{6})$', trial_dir.name)
    if not m:
        return None
    stamp_value = m.group(1)
    candidate_value = []
    for y in sorted(LOG_DIRECTORY.glob('guided_follow_*.csv')):
        d2 = re.search(r'(\d{8}_\d{6})', y.name)
        if d2 and d2.group(1) >= stamp_value:
            candidate_value.append((d2.group(1), y))
    return candidate_value[0][1] if candidate_value else None


def _time_interval(path_value, key_value):
    try:
        with open(path_value, newline='') as f:
            r = csv.DictReader(f)
            initial = last_value = None
            for row_value in r:
                v = _f(row_value, key_value)
                if v is None:
                    continue
                if initial is None:
                    initial = v
                last_value = v
        return (initial, last_value) if initial is not None else None
    except Exception:
        return None


def _overlapping_position_based(visual_path):
    g = _time_interval(visual_path, 't')
    if g is None:
        return None
    best_candidate, best_overlap = None, 0.0
    for candidate_value in sorted(LOG_DIRECTORY.glob('guided_follow_*.csv')):
        k = _time_interval(candidate_value, 'wall_time')
        if k is None:
            continue
        overlap_value = min(g[1], k[1]) - max(g[0], k[0])
        if overlap_value > best_overlap:
            best_candidate, best_overlap = candidate_value, overlap_value
    return best_candidate


# ------------------------------------------------------------------------------- time alignment

class Aligner:
    """Farkli loglari ORTAK zamana oturtur.

    t_mono : time.monotonic() -- CSV 't' with image, CSV 'wall_time' with position, mpc_diagnostic CSV
    't'. On Linux, CLOCK_MONOTONIC is COMMON between processes, so these three files are directly
    aligned. t_unix : absolute time. The new display CSV has it as a column; In older cases, it is
    extrapolated back from the modification time of the file (+-1 s precision; enough for video/bbox
    mapping). t_rel : seconds from the BEGINNING of the run (the output uses this).
    """

    def __init__(self, t0_mono, mono_to_unix_offset):
        self.t0 = t0_mono
        self.offset_value = mono_to_unix_offset     # unix = mono + offset

    def rel(self, t_mono):
        return None if t_mono is None else t_mono - self.t0

    def unix(self, t_mono):
        if t_mono is None or self.offset_value is None:
            return None
        return t_mono + self.offset_value

    def clock(self, t_mono):
        u = self.unix(t_mono)
        return '--:--:--' if u is None else \
            datetime.fromtimestamp(u).strftime('%H:%M:%S')


def aligner_build(observe_row, position_row, observe_path):
    """Sets the t0 of the run and the monotonic->unix offset."""
    mono_starts = []
    if position_row:
        v = _f(position_row[0], 'wall_time')
        if v is not None:
            mono_starts.append(v)
    if observe_row:
        v = _f(observe_row[0], 't')
        if v is not None:
            mono_starts.append(v)
    t0 = min(mono_starts) if mono_starts else 0.0

    offset_value = None
    for s in observe_row[:50]:                      # NEW format: there is a colon
        tm, tu = _f(s, 't'), _f(s, 't_unix')
        if tm is not None and tu is not None:
            offset_value = tu - tm
            break
    if offset_value is None and observe_row and observe_path:   # OLD format: from mtime
        last_value = _f(observe_row[-1], 't')
        if last_value is not None:
            offset_value = os.path.getmtime(observe_path) - last_value
    return Aligner(t0, offset_value)


# ------------------------------------------------- target state + geometry

def _derived_target(position_row):
    """It extracts the speed of the target from the positional log by NUMERICAL DERIVATION.

    For OLD runs. Position log records the MEASURED position of the target (meas_x/y/z) but not its
    speed; The speed is found with the central difference of ~0.5 s. Returns: [(t_mono, pos3,
    vel3|None), ...]
    """
    raw_value = []
    for s in position_row:
        t = _f(s, 'wall_time')
        p = [_f(s, k) for k in ('meas_x', 'meas_y', 'meas_z')]
        if t is None or any(v is None for v in p):
            continue
        raw_value.append((t, p))
    output = []
    for i, (t, p) in enumerate(raw_value):
        v = None
        j = i
        while j > 0 and t - raw_value[j][0] < 0.5:
            j -= 1
        k = i
        while k < len(raw_value) - 1 and raw_value[k][0] - t < 0.5:
            k += 1
        dt = raw_value[k][0] - raw_value[j][0]
        if dt > 0.05:
            v = [(raw_value[k][1][a] - raw_value[j][1][a]) / dt for a in range(3)]
        output.append((t, p, v))
    return output


def geometry_value(pos, vel, hpos, hvel):
    """Independent copy of visual_base._encounter_geometry.

    Same definitions (sign conventions in LOG_DICTIONARY.md): approximation_deg 0 = the target is coming
    right at us, 180 = we are on its tail.
    """
    g = {'range_m_value': None, 'bearing_deg': None, 'elevation_deg': None,
         'approximation_deg': None, 'type_value': '', 'closure_mps': None, 'tgo_s': None,
         'cpa_m': None, 'cpa_s': None}
    if pos is None or hpos is None:
        return g
    r = [hpos[i] - pos[i] for i in range(3)]
    range_value = math.sqrt(sum(x * x for x in r))
    g['range_m_value'] = range_value
    horizontal = math.hypot(r[0], r[1])
    g['bearing_deg'] = math.degrees(math.atan2(r[1], r[0])) % 360.0
    g['elevation_deg'] = math.degrees(math.atan2(-r[2], max(horizontal, 1e-6)))
    if vel is None or hvel is None or range_value < 1e-6:
        return g
    u = [x / range_value for x in r]
    vb = [hvel[i] - vel[i] for i in range(3)]
    g['closure_mps'] = -sum(vb[i] * u[i] for i in range(3))
    if g['closure_mps'] > 0.1:
        g['tgo_s'] = range_value / g['closure_mps']
    n2 = sum(x * x for x in vb)
    if n2 > 1e-6:
        tc = -sum(r[i] * vb[i] for i in range(3)) / n2
        if tc < 0.0:
            g['cpa_s'], g['cpa_m'] = 0.0, range_value
        else:
            g['cpa_s'] = tc
            g['cpa_m'] = math.sqrt(sum((r[i] + vb[i] * tc) ** 2 for i in range(3)))
    hh = math.sqrt(sum(x * x for x in hvel))
    if hh < 1.0:
        g['type_value'] = 'stationary'
        return g
    run_value2 = sum((hvel[i] / hh) * (-u[i]) for i in range(3))
    g['approximation_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, run_value2))))
    g['type_value'] = _type_determine(g['approximation_deg'], hh)
    return g


def authority_intervals(observe_value, gap_s=1.0):
    """[start, end] time intervals for which the visual guide is authorized.

    CRITICAL: two processes log at the same time -- the embedded process continues to measure and
    log after losing privilege (it just stops sending setpoints). So nesting two CSVs in time gives
    the illusion that "power changes hands every cycle." Correct rule: DISPLAY CSV writes lines only
    when authorized; A space between lines greater than 1 s means the authority has changed to
    positioned (same definition as compare_results.py).
    """
    intervals = []
    start_value = previous_value = None
    for s in observe_value:
        t = _f(s, 't')
        if t is None:
            continue
        if previous_value is None or t - previous_value > gap_s:
            if start_value is not None:
                intervals.append((start_value, previous_value))
            start_value = t
        previous_value = t
    if start_value is not None:
        intervals.append((start_value, previous_value))
    return intervals


class Samples:
    """A combined table that answers the question 'what was happening' at every moment of the run.

    Each example is a dictionary: t (mono), source ('located'/'imaged'), range, geometry, framing
    information, status. The sample of that moment is taken FROM whichever process is in authority
    (see authority_intervals).
    """

    def __init__(self, observe_value, position_value, speed_value):
        self.speed_value = speed_value
        self.g = observe_value
        self.k = position_value
        self.intervals = authority_intervals(observe_value)
        self.sample_value = []
        self._merge()
        self._range_correct()

    def _under_authority(self, t):
        for start_value, last_value in self.intervals:
            if start_value <= t <= last_value:
                return True
        return False

    def _merge(self):
        # Old format detection: If there is no ref_target_x column, derive the target from positioned
        new_format = bool(self.g) and 'ref_target_x' in self.g[0]
        # The target's velocity is NOT in the positional log (only its measured position). For the POSITIONED
        # phase it is always derived; For the DISPLAY phase only in old runs (ref_target_v* ready in new CSV).
        derived = _derived_target(self.k)

        for s in self.k:
            t = _f(s, 'wall_time')
            if t is None or self._under_authority(t):
                continue          # At that moment the authority was on video: use it
            self.sample_value.append(self._position_based_sample(t, s, derived))

        for s in self.g:
            t = _f(s, 't')
            if t is None:
                continue
            self.sample_value.append(self._visual_sample(t, s, new_format,
                                                    derived))
        self.sample_value.sort(key=lambda o: o['t'])

    def _range_correct(self):
        """SMOOTH the range and closing speed.

        The range comes from two separate sources (positioned: true_range_m; imaged: IMM range) and
        both are noisy. Calculating the closing velocity from the range difference of ~0.5 s, rather
        than from instantaneous velocity vectors, both cuts noise and keeps the two sources
        consistent -- events that look at sign change, such as 'nearest pass', would otherwise
        trigger dozens of times.
        """
        for i, x in enumerate(self.sample_value):
            if x['range_value'] is None:
                x['range_straight'] = x['closure_straight'] = None
                continue
            j, k = i, i
            while j > 0 and x['t'] - self.sample_value[j]['t'] < 0.3:
                j -= 1
            while k < len(self.sample_value) - 1 and self.sample_value[k]['t'] - x['t'] < 0.3:
                k += 1
            window_value = [y['range_value'] for y in self.sample_value[j:k + 1]
                       if y['range_value'] is not None]
            x['range_straight'] = (sorted(window_value)[len(window_value) // 2]
                               if window_value else x['range_value'])
            r0, r1 = self.sample_value[j]['range_value'], self.sample_value[k]['range_value']
            dt = self.sample_value[k]['t'] - self.sample_value[j]['t']
            x['closure_straight'] = ((r0 - r1) / dt
                                if None not in (r0, r1) and dt > 0.05 else None)

    def _position_based_sample(self, t, s, derived):
        pos = [_f(s, k) for k in ('pursuer_x', 'pursuer_y', 'pursuer_z')]
        vel = [_f(s, k) for k in ('pursuer_vx', 'pursuer_vy', 'pursuer_vz')]
        hpos = [_f(s, k) for k in ('meas_x', 'meas_y', 'meas_z')]
        pos = None if any(v is None for v in pos) else pos
        vel = None if any(v is None for v in vel) else vel
        hpos = None if any(v is None for v in hpos) else hpos
        _, hvel = self._en_near_target(t, derived)
        g = geometry_value(pos, vel, hpos, hvel)
        if g.get('closure_mps') is None:
            g['closure_mps'] = _f(s, 'closing_velocity')
        if g.get('tgo_s') is None:
            g['tgo_s'] = _f(s, 't_go_s')
        return {
            't': t, 'source_value': 'position_based',
            'range_value': _f(s, 'true_range_m') or _f(s, 'range_m'),
            'geo': g,
            'pos': pos, 'vel': vel, 'hpos': hpos, 'hvel': hvel,
            'ex': None, 'ey': None, 'coverage_value': None, 'state_value': '',
            'kurtarma': s.get('recovery_state', ''),
            'vibe': _f(s, 'vibe_max'),
            'clamp': {},
        }

    def _visual_sample(self, t, s, new_format, derived):
        pos = [_f(s, k) for k in ('pos_x', 'pos_y', 'pos_z')]
        vel = [_f(s, k) for k in ('vel_x', 'vel_y', 'vel_z')]
        pos = None if any(v is None for v in pos) else pos
        vel = None if any(v is None for v in vel) else vel
        if new_format:
            hpos = [_f(s, k) for k in ('ref_target_x', 'ref_target_y', 'ref_target_z')]
            hvel = [_f(s, k) for k in ('ref_target_vx', 'ref_target_vy', 'ref_target_vz')]
            hpos = None if any(v is None for v in hpos) else hpos
            hvel = None if any(v is None for v in hvel) else hvel
            g = {'range_m_value': _f(s, 'ref_range_ground_truth_m'),
                 'bearing_deg': _f(s, 'ref_bearing_deg'),
                 'elevation_deg': _f(s, 'ref_elevation_deg'),
                 'approximation_deg': _f(s, 'ref_approximation_angle_deg'),
                 'type_value': s.get('ref_encounter_type', ''),
                 'closure_mps': _f(s, 'ref_closure_rate_mps'),
                 'tgo_s': _f(s, 'ref_tgo_s'), 'cpa_m': _f(s, 'ref_cpa_m'),
                 'cpa_s': _f(s, 'ref_cpa_s')}
        else:
            hpos, hvel = self._en_near_target(t, derived)
            g = geometry_value(pos, vel, hpos, hvel)
        return {
            't': t, 'source_value': 'visual',
            'range_value': _f(s, 'range_m_value'), 'geo': g,
            'pos': pos, 'vel': vel, 'hpos': hpos, 'hvel': hvel,
            'ex': _f(s, 'ex_deg'), 'ey': _f(s, 'ey_deg'),
            'coverage_value': _f(s, 'coverage_pct'),
            'state_value': s.get('state_value', ''), 'kurtarma': '',
            'vibe': _f(s, 'vibe_max'),
            'bbox_age': _f(s, 'bbox_age_s'),
            'clamp': {k: _f(s, k) for k in
                        ('clamp_speed', 'clamp_altitude', 'clamp_yaw_slew')
                        if s.get(k) not in ('', None)},
        }

    @staticmethod
    def _en_near_target(t, derived):
        """Old format: get the target sample closest in time from the location log."""
        if not derived:
            return None, None
        best_candidate = min(derived, key=lambda x: abs(x[0] - t))
        if abs(best_candidate[0] - t) > 1.0:
            return None, None
        return best_candidate[1], best_candidate[2]


# ---------------------------------------------------------------- anlatici

def explain(sources, full_value=False):
    observe_value = _read_value(sources['visual']) if sources['visual'] else []
    position_value = _read_value(sources['position_based']) if sources['position_based'] else []
    if not observe_value and not position_value:
        raise SystemExit(
            "No log files found. Expected:\n"
            "  line ']log: <path>.csv' in <test>/display.log, or\n"
            "  guidance_allstar/logs/display_<method>_<stamp>.csv / "
            "guided_follow_<damga>.csv\n"
            "If video guidance did not work at all (e.g. HOME_POSITION could not be received) "
            "lone-location log is described; Otherwise, pre-running video "
            "donemden olabilir.")
    if not observe_value:
        print("[note] no CSV with display -- only POSITIVE phase is described.")
    speed_value = aligner_build(observe_value, position_value, sources['visual'])
    table_item = Samples(observe_value, position_value, speed_value)
    o = table_item.sample_value

    label_item = (sources['trial'].name if sources['trial']
          else Path(sources['visual']).stem)
    print("=" * 78)
    print(f"RUN ANLATIMI: {label_item}")
    print("=" * 78)
    print("kaynaklar:")
    for label_value, key_value in (('visual', 'visual'), ('located ', 'position_based'),
                            ('mpc diagnosis ', 'mpc_diagnostic'), ('events  ', 'event_value'),
                            ('bbox     ', 'bbox')):
        y = sources.get(key_value)
        print(f"  {label_value}: {y if y else '(absent)'}")
    n_observe, n_position = len(observe_value), len(position_value)
    new_value = bool(observe_value) and 'ref_target_x' in observe_value[0]
    print(f"  line : {n_observe} with image, {n_position} with position   "
          f"CSV format: {'NEW (ref_* present)' if new_value else 'PREVIOUS (target state derived from position guidance)'}")
    if o:
        start_value, last_value = o[0]['t'], o[-1]['t']
        print(f"  time : t=0 -> {speed_value.clock(start_value)} "
              f"(unix {speed_value.unix(start_value):.0f}) duration {last_value - start_value:.1f} s"
              if speed_value.unix(start_value) else
              f"  time : duration {last_value - start_value:.1f} s (absolute time unknown)")
    print()
    print("--- TIMELINE ---")

    rows_value = []                      # ( t_rel , row, text)
    previous_state = None
    loss_beginning = None
    threshold_passed = set()
    nearest_distance = {'m': float('inf'), 't': None, 'sample_value': None}
    clamp_enabled = {}
    clamp_duration = {}
    terminal_written = False

    def add_value(t, text_value, sequence=1):
        rows_value.append((t, sequence, text_value))

    if o:
        x0 = o[0]
        add_value(speed_value.rel(x0['t']),
             f"THE RACE HAS STARTED -- {_phase_text(x0, x0['range_value'], x0['geo'])}", 0)

    # --- handoff MOMENTS: from authorization ranges (see authority_intervals) ---
    for start_value, last_value in table_item.intervals:
        if last_value - start_value < 0.3:
            continue                    # hesitation lasting several cycles
        initial = _en_near_sample(o, start_value, 'visual')
        if initial is not None:
            add_value(speed_value.rel(start_value), f">>> HANDOFF RECEIVED (visual guidance). "
                               f"{_handoff_text(initial)}", 0)
        last_x = _en_near_sample(o, last_value, 'visual')
        add_value(speed_value.rel(last_value), f"<<< authorization TO_POSITION frozen "
                           f"(range {_m(last_x['range_straight'] if last_x else None)}, "
                           f"imaged phase lasted {last_value - start_value:.1f} s).", 2)

    for i, x in enumerate(o):
        t = speed_value.rel(x['t'])
        g = x['geo']
        mz = x['range_straight']

        # --- internal state of imaged phase: fresh / hold / water ---
        if x['source_value'] == 'visual':
            d = x['state_value']
            if d and d != previous_state:
                if d in ('hold_value', 'coast') and previous_state == 'fresh_value':
                    loss_beginning = t
                elif d == 'fresh_value' and loss_beginning is not None:
                    if t - loss_beginning >= 0.4:
                        add_value(t, f"    target {t - loss_beginning:.1f} s out of frame "
                                f"WAS OUT, came back (range {_m(mz)})")
                    loss_beginning = None
                previous_state = d
            for k, v in x['clamp'].items():
                enabled_value = bool(v)
                if enabled_value and not clamp_enabled.get(k):
                    clamp_enabled[k] = t
                elif not enabled_value and clamp_enabled.get(k):
                    clamp_duration[k] = clamp_duration.get(k, 0.0) + (t - clamp_enabled[k])
                    clamp_enabled[k] = None

        # --- range thresholds (first pass) ---
        if mz is not None:
            for threshold_value in (500, 200, 100, 50, 30, 20, 10, 5):
                if mz <= threshold_value and threshold_value not in threshold_passed:
                    threshold_passed.add(threshold_value)
                    if threshold_value <= 100:
                        add_value(t, f"    range decreased below {threshold_value} m "
                                f"({_gel2(x)})")
            # measured in "closest" RAW range (smoothing smoothes out fast transients, reporting a false bottom as
            # 6.3 m; the real 2.6 was m).
            if x['range_value'] is not None and x['range_value'] < nearest_distance['m']:
                nearest_distance = {'m': x['range_value'], 't': t, 'sample_value': x}

        if (not terminal_written and mz is not None and mz < 20.0
                and x['source_value'] == 'visual'):
            terminal_written = True
            add_value(t, f"    TERMINAL PHASE (range {_m(mz)}, {_gel2(x)})")

    # --- CLOSEST PASSAGES: SIGNIFICANT local minima of the range series --- Looking at the instantaneous
    # close signal produced dozens of false triggers; Here a minimum is considered a "pass" only if the
    # range on either side of it increases significantly.
    for x in _belirgin_minima(o):
        dip_m = x['range_value']
        # Read the encounter type from the example 2 s BEFORE the transition (at the moment of transition the
        # approach angle swings from 0 to 180; the question is HOW we approach).
        previous_value = _en_near_sample(o, x['t'] - 2.0) or x
        add_value(speed_value.rel(x['t']),
             f"*** EN NEAR TRANSITION: {_m(dip_m)} -- yaklasirken {_gel2(previous_value)}", 3)
        for y in o:
            if y['t'] > x['t'] and y['range_straight'] is not None and \
                    y['range_straight'] > max(2.0 * dip_m, dip_m + 15.0):
                add_value(speed_value.rel(y['t']),
                     f"    MISS: the nearest was {dip_m:.1f} m, "
                     f"now {_m(y['range_straight'])} -- range opening.", 4)
                break

    for t, _, text_value in sorted(rows_value, key=lambda s: (s[0], s[1])):
        print(f"t={t:7.1f} s  {text_value}")

    # --------------------------------------------------------------- summary
    print()
    print("--- RESULT ---")
    if nearest_distance['sample_value'] is not None:
        x, g = nearest_distance['sample_value'], nearest_distance['sample_value']['geo']
        verdict_value = ('IMPACT' if nearest_distance['m'] <= 3.0 else
                 'NEAR' if nearest_distance['m'] <= 8.0 else 'MISS')
        print(f"  law: {verdict_value} (nearest {nearest_distance['m']:.2f} m, "
              f"t= {nearest_distance['t']:.1f} s, source= {x['source_value']} )")
        # Read the encounter type 2 s BEFORE the pass, not AT THE TIME OF PASSING: at the moment of passing
        # the angle changes rapidly (passing 90 degrees when passing sideways), whereas the question is "how
        # did we approach" -- from the tail or from the front.
        previous_value = _en_near_sample(o, x['t'] - 2.0)
        gy = (previous_value or x)['geo']
        print(f"  encounter type: {TYPE_TEXT.get(gy.get('type_value'), '?')}"
              + (f"  (approach angle {gy['approximation_deg']:.0f} deg, "
                 f"2 s before transition)"
                 if gy.get('approximation_deg') is not None else ''))
        print(f"  closing speed: "
              f"{_ms((previous_value or x).get('closure_straight'))} m/s (before transition 2 s)")
        if x['hvel'] is not None:
            hh = math.sqrt(sum(v * v for v in x['hvel']))
            hr = math.degrees(math.atan2(x['hvel'][1], x['hvel'][0])) % 360.0
            print(f"  destination: {hh:.1f} m/s, route {_direction(hr)}")
        if x['vel'] is not None:
            bh = math.sqrt(sum(v * v for v in x['vel']))
            br = math.degrees(math.atan2(x['vel'][1], x['vel'][0])) % 360.0
            print(f"  we: {bh:.1f} m/s, route {_direction(br)}")
        if g.get('elevation_deg') is not None:
            print(f"  target rise: {g['elevation_deg']:+.1f} deg "
                  f"({'above' if g['elevation_deg'] > 0 else 'below'})")

    # Match type distribution: the answer to the question "how many times were we head to head recently"
    # -- what the user had to calculate MANUALLY in 2026 - 08 - 04.
    near_value = [x for x in o if x['source_value'] == 'visual'
             and (x['range_value'] or 1e9) < 60 and x['geo'].get('type_value')]
    if near_value:
        count_value3 = {}
        for x in near_value:
            count_value3[x['geo']['type_value']] = count_value3.get(x['geo']['type_value'], 0) + 1
        print(f"  COMPARISON DISTRIBUTION (visual phase, range<60 m, n={len(near_value)}):")
        for type_value, n in sorted(count_value3.items(), key=lambda kv: -kv[1]):
            print(f"      {TYPE_TEXT.get(type_value, type_value):15s} %{100*n/len(near_value):5.1f} "
                  f"({n} example)")

    observe_sample = [x for x in o if x['source_value'] == 'visual']
    if observe_sample:
        fresh_value = sum(1 for x in observe_sample if x['state_value'] == 'fresh_value')
        if fresh_value:
            print(f"  stay in frame: %{100*fresh_value/len(observe_sample):.1f} "
                  f"({fresh_value}/{len(observe_sample)} cycle 'fresh')")
        vb = [x['vibe'] for x in observe_sample if x['vibe'] is not None]
        if vb:
            print(f"  vibe top: {max(vb):.1f}  "
                  f"(GROUND contact >50; Gazebo DOES NOT MODEL target impact)")
    if clamp_duration or any(clamp_enabled.values()):
        print("  Restraint/clamp total time:")
        for k in set(list(clamp_duration) + list(clamp_enabled)):
            duration_value = clamp_duration.get(k, 0.0)
            print(f"      {k:20s} {duration_value:5.1f} s")

    if sources['bbox']:
        _bbox_summary(sources['bbox'])
    if sources['event_value']:
        _event_summary(sources['event_value'], speed_value, full_value)


def _en_near_sample(sample_value, t, source_value=None):
    candidate_value = [x for x in sample_value if source_value is None or x['source_value'] == source_value]
    if not candidate_value:
        return None
    return min(candidate_value, key=lambda x: abs(x['t'] - t))


def _belirgin_minima(sample_value, lowest_elevation=8.0, ratio_value=0.5):
    """Finds the 'true' closest transitions in the range series.

    A local minimum is counted as crossing only AFTER the range widens significantly (by
    min(lowest_elevation, ratio*range)). This eliminates hundreds of so-called minima produced by
    noise; the rest really correspond to "we're close, we're over, we're opening" moments.
    """
    series = [x for x in sample_value if x['range_straight'] is not None]
    if len(series) < 5:
        return []
    # BOTTOM <-> TOP SCANNING. Just saying "bottom up to threshold" is not enough: once you trigger it,
    # each step looks like a new "pass" as the range continues to OPEN (12 fake pass appeared on the first
    # try). A new pass is only counted if there is a REAL crest in between and it is approached again.
    def threshold_value(v):
        return max(lowest_elevation, ratio_value * v)

    result_value = []
    mode_value, dip, peak = 'dip', series[0], series[0]
    for x in series:
        v = x['range_straight']
        if mode_value == 'dip':
            if v < dip['range_straight']:
                dip = x
            elif v - dip['range_straight'] >= threshold_value(dip['range_straight']):
                result_value.append(dip)
                mode_value, peak = 'peak', x
        else:
            if v > peak['range_straight']:
                peak = x
            elif peak['range_straight'] - v >= threshold_value(v):
                mode_value, dip = 'dip', x
    # END OF RUN: The log may have ended before the opening was completed. The MOST IMPORTANT line of the
    # closest passage explanation -- the global minimum is written in any case.
    small = min(series, key=lambda y: y['range_straight'])
    if small not in result_value:
        result_value.append(small)
    # FINDS SMOOTHING, MEASURS RAW DATA: 38 A pass of m/s emits a median window of 0.6 s 11 m; Smooth
    # series (no false trigger) is used when LOOKING for the bottom, RAW smallest range within +-1 s is
    # used when REPORTING the bottom.
    return sorted((_raw_dip(series, s) for s in result_value if s['range_straight'] < 100.0),
                  key=lambda y: y['t'])


def _raw_dip(series, dip, window_s_value=1.0):
    near_value = [y for y in series if abs(y['t'] - dip['t']) <= window_s_value
             and y['range_value'] is not None]
    return min(near_value, key=lambda y: y['range_value']) if near_value else dip


def _m(v):
    return '---' if v is None else f"{v:.1f} m"


def _ms(v):
    return '---' if v is None else f"{v:.1f}"


def _gel2(x):
    """One-sentence summary of the encounter: type + (smoothed) closure."""
    g = x['geo']
    part = []
    if g.get('type_value'):
        part.append(TYPE_TEXT.get(g['type_value'], g['type_value']))
        if g.get('approximation_deg') is not None:
            part[-1] += f" [{g['approximation_deg']:.0f} deg]"
    kap = x.get('closure_straight')
    if kap is None:
        kap = g.get('closure_mps')
    if kap is not None:
        part.append(f"shutdown {kap:+.1f} m/s")
        if kap > 0.3 and x.get('range_straight') and x['range_straight'] / kap < 30.0:
            part.append(f"t_go {x['range_straight'] / kap:.1f} s")
    return ', '.join(part) if part else 'no geometry'


def _phase_text(x, mz, g):
    p = ['POSITION_BASED phase' if x['source_value'] == 'position_based' else 'DISPLAY phase',
         f"range {_m(mz)}"]
    if x['hvel'] is not None:
        hh = math.sqrt(sum(v * v for v in x['hvel']))
        hr = math.degrees(math.atan2(x['hvel'][1], x['hvel'][0])) % 360.0
        p.append(f"destination {hh:.1f} m/s route {_direction(hr)}")
    p.append(_gel2(x))
    return ', '.join(p)


def _handoff_text(x):
    """The exact description of the handoff moment: whoever reads this line MUST SEE the handoff moment."""
    p = [f"range {_m(x['range_straight'])}"]
    # ey sign: Measurement convention -> ey_deg + = target BELOW. "above/below the frame" REGARDING WHAT:
    # center of frame = OPTICAL AXIS of the camera. Gimbal branch (2026-08-05): this axis is no longer the
    # trunk pitch, but the world elevation (tilt) of the gimbal; the axis remains fixed even if the body
    # is swung (measured: camera max 0.65 deg while the body is +-35 deg). So the expression "BELOW the
    # target frame X deg" is a direct TILT error.
    if x['ey'] is not None:
        p.append(f"target frame {abs(x['ey']):.1f} deg "
                 f"{'ALTINDA' if x['ey'] > 0 else 'USTUNDE'}")
    if x['ex'] is not None:
        p.append(f"{abs(x['ex']):.1f} deg "
                 f"{'SAGINDA' if x['ex'] > 0 else 'SOLUNDA'}")
    if x['coverage_value'] is not None:
        p.append(f"Coverage {x['coverage_value']:.2f}")
    p.append(_gel2(x))
    return ', '.join(p)


def _bbox_summary(path_value):
    """bbox.log WITHOUT TIMESTAMP: only aggregate statistics can be given.

    The only bridges for alignment are the px_raw_cx/px_raw_cy columns in the new imaged CSV (the
    same raw centers are there, with timestamp).
    """
    text_value = ANSI.sub('', Path(path_value).read_text(errors='replace'))
    box_value = re.findall(r'center=\((\d+),(\d+)\).*?cov=([0-9.]+)%', text_value)
    summary_value = re.findall(r'frame=(\d+)\s+fps=([0-9.]+)\s+detection_ratio=%([0-9.]+)', text_value)
    print()
    print("--- DETECTOR (bbox.log; NO TIMESTAMP, batch statistics) ---")
    if summary_value:
        print(f"  processed frame : {summary_value[-1][0]} (avg {sum(float(x[1]) for x in summary_value)/len(summary_value):.1f} fps)")
        print(f"  cumulative detection: {summary_value[-1][2]}")
    if box_value:
        ys = sorted(int(k[1]) for k in box_value)
        cov = sorted(float(k[2]) for k in box_value)
        n = len(box_value)
        print(f"  detection number: {n}")
        # Center of frame (360 px) = optical axis of the camera. GIMBAL BRANCH: this axis is determined by the
        # tilt command, NOT by the trunk pitch; is a systematic deviation tilt/standoff geometry error
        # (standoff_geom.sh).
        print(f"  frame y middle : {ys[n//2]} (center of frame 360 = camera "
              f"optical axis/tilt; small = target UP)")
        print(f"  Coverage median: {cov[n//2]:.2f}% peak {cov[-1]:.2f}%")


def _event_summary(path_value, speed_value, full_value):
    row_value = _read_value(path_value)
    if not row_value:
        return
    print()
    print("--- EVENT LOG (discrete events of the video skeleton) ---")
    for s in (row_value if full_value else row_value[:60]):
        t = _f(s, 't')
        print(f"t={speed_value.rel(t):7.1f} s {s['event_value']:28s} "
              f"range={s.get('range_m_value', ''):>8s} {s.get('detail', '')}")
    if not full_value and len(row_value) > 60:
        print(f"  ... more events {len(row_value)-60} (see all with --full-value)")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('trial', nargs='?',
                   help='run/tests/<test> folder')
    p.add_argument('--visual', help='direct video CSV way')
    p.add_argument('--full-value', action='store_true',
                   help='print event log without truncating it')
    a = p.parse_args()
    if not a.trial and not a.visual:
        p.error("give a test folder or --visual CSV")
    d = Path(a.trial) if a.trial else None
    if d is not None and not d.exists():
        raise SystemExit(f"no folder: {d}")
    explain(files_find(d, a.visual), a.full_value)


if __name__ == '__main__':
    main()
