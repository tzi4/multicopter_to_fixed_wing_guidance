#!/usr/bin/env python3
"""
Simple target-follow runner.

Reads the target position, estimates target velocity with filterwndr.py, computes a
slot behind the target, and repeatedly sends that slot to the pursuer as an
ArduCopter GUIDED local-NED position target.

This is intentionally separate from the PN/lag-pursuit attitude-control scripts.
"""

import argparse
import atexit
import csv
import json
import os
import signal
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import guidance_config as cfg
import mavlink_utils
import numpy as np
from filterwndr import (
    HeadingTurnRateEstimator,
    IMMLowPassFilter,
    aggregate_mode_probabilities,
    apply_fast_turn_onset_hint,
    apply_turn_rate_hint,
    clamp_filter_dt,
    ct_mode_probability,
    predict_imm_over_dt,
    restore_imm_vertical_state,
    setup_imm_filter,
    snapshot_imm_vertical_state,
    stabilize_omega_states,
    update_imm_preserving_vertical,
)
from guidance_gui import GuidanceGUI, gui_tick, push_snapshot

# Komut yetkisi devri icin istege bagli: Redis yoksa eski davranis korunur.
try:
    import redis as redis_lib
except ImportError:
    redis_lib = None
from pymavlink import mavutil

redcolor = "\033[0;31m"
bluecolor = "\033[0;34m"
endcolor = "\033[0m"

POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

POSITION_VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# Position + velocity + acceleration feedforward (only yaw/yaw-rate ignored).
POSITION_VELOCITY_ACCEL_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def clamp_norm(v, max_norm):
    norm = float(np.linalg.norm(v))
    if norm > max_norm and norm > 1e-6:
        return v * (max_norm / norm)
    return v


def target_feedforward_acceleration(state, max_accel):
    """World-frame (NED) target acceleration for setpoint feedforward.

    The IMM carries a coordinated turn as a turn rate, not in the ax/ay states
    (CTxy modes stabilize ax/ay to zero), so the horizontal acceleration during
    a turn is the centripetal term omega x v_xy, not state[6:8]. This combines
    both: the CAxy/CAz-weighted linear acceleration plus the CT centripetal
    acceleration. They do not double-count -- CT modes zero the linear part and
    CA modes gate omega to zero.

    Vertical acceleration is intentionally dropped (az = 0): feeding it forward
    reintroduces the Z command transients this project spent a long time taming,
    and the lead value of accel FF is almost entirely horizontal. The horizontal
    magnitude is clamped to reject noisy acceleration/omega spikes.
    """
    state = np.asarray(state, dtype=float).reshape(-1)
    vx, vy = float(state[3]), float(state[4])
    a_lin_x, a_lin_y = float(state[6]), float(state[7])
    omega = float(state[9]) if state.shape[0] > 9 else 0.0

    # Centripetal acceleration of a coordinated horizontal turn: a = omega x v.
    ax = a_lin_x - omega * vy
    ay = a_lin_y + omega * vx
    a_ff = np.array([ax, ay, 0.0])
    return clamp_norm(a_ff, max_accel)


def accel_ff_range_scale(range_m, extend_range_m, fade_band_m):
    """Fade the acceleration feedforward from 1 (midcourse) to 0 as the pursuer
    enters the terminal-extension band.

    Inside the extension range the aim point is deliberately pushed past the
    target, so adding target acceleration there makes the position controller
    anticipate continued acceleration and blow through the target (overshoot).
    Returns 1 beyond (extend_range + fade_band), ramping linearly to 0 at
    extend_range and below.
    """
    inner = float(extend_range_m)
    band = max(0.0, float(fade_band_m))
    if band <= 1e-6:
        return 1.0 if float(range_m) > inner else 0.0
    return float(np.clip((float(range_m) - inner) / band, 0.0, 1.0))


class GuidanceLogger:
    """Per-loop CSV of guidance state, for diagnosing misses/overshoots.

    Unlike the estimator's imm_diagnostics logs (which only cover target
    estimation), this records the pursuer, the commanded setpoint, the intercept
    geometry, and the terminal/accel-FF state, so a miss can be traced.
    """

    FIELDNAMES = [
        "wall_time",
        "range_m",
        "closing_velocity",
        "t_go_s",
        "guidance_horizon_s",
        "pursuer_x", "pursuer_y", "pursuer_z",
        "meas_x", "meas_y", "meas_z",
        "est_x", "est_y", "est_z",
        "slot_x", "slot_y", "slot_z",
        "slot_vx", "slot_vy", "slot_vz",
        "aff_x", "aff_y", "aff_z", "aff_mag",
        "extended", "latched", "latch_armed",
        "recovery_state",
        "z_limited", "aim_limited", "alt_floored",
        "lead_k_h", "lead_k_v", "mu_ct",
        "yaw_rate_dps", "alt_err_m", "cpa_range_m",
        # Vehicle attitude + own velocity, to measure crab (heading vs course)
        # and total tilt at CT-activation (the saturation/yaw-shed diagnosis).
        "roll_deg", "pitch_deg", "yaw_deg",
        "pursuer_vx", "pursuer_vy", "pursuer_vz",
        "yaw_frozen",
        # Command turn clamp: raw demanded lateral accel (how sharp a turn the
        # target flew) and whether that demand hit the bank-margin cap.
        "cmd_lat_accel_mps2", "turn_clamped",
        # super_safe_turn: the v_safe speed cap applied this loop (blank when the
        # feature is off or no qualifying turn was detected).
        "safe_turn_vmax",
        # Carrot clamp: whether the commanded point was pulled back to
        # CARROT_MAX_AHEAD_M ahead of the vehicle this loop (closure governor).
        "carrot_limited",
        # Mission failsafe: 1 while tripped (setpoints withheld because the
        # target data went stale/corrupt or a hard fault was detected), and the
        # seconds of dead-reckoning applied while coasting a tracker dropout.
        "failsafe_active", "coast_age_s",
        # Which mode flew: 1 = intercept (terminal chain live), 0 = simple
        # follow. Constant per run, but it makes a log self-describing.
        "kill_mode",
        # Impact detection: true pursuer<->target range, airframe vibration, and
        # the running hit count (for tuning the thresholds and post-hoc scoring).
        "true_range_m", "vibe_max", "hit_count",
        # Loop health (2026-08-02): OLCULEN dongu adimi ve sim-zamani/duvar-saati
        # orani. GUI/CPU yuku altinda dongu 20 Hz yerine 2.2 Hz'e dustugunde her
        # entegrasyon nominal dt ile 9 kat yanlis calisiyordu; oran ise basari/
        # basarisizlik ayirt edicisiydi (basarisiz kosuda 0.74, basarilida 1.00).
        "loop_dt_meas_s", "simtime_ratio",
    ]

    def __init__(self, log_dir="logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"guided_follow_{stamp}.csv"
        self.file = self.path.open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self._rows_since_flush = 0
        # Flush the tail even when main() dies on an exception (close() is
        # idempotent, so the normal-path close costs nothing extra).
        atexit.register(self.close)

    def write(self, row):
        self.writer.writerow(row)
        self._rows_since_flush += 1
        if self._rows_since_flush >= 25:
            self.file.flush()
            self._rows_since_flush = 0

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()


class MissRecoveryController:
    """State machine for graceful recovery after a terminal miss.

    The aggressive terminal dive (extension + freeze latch) is kept intact for
    hit probability. But on the pass, instead of instantly re-chasing at full
    speed from a bad attitude (which sends the pursuer diverging), it hands off:

        CHASE     normal guidance (terminal dive + latch). Vehicle in GUIDED.
          | terminal latch releases after a pass (miss)
        HOLD      switch to a recovery flight mode (BRAKE/STABILIZE/LOITER) for
          |       recovery_hold_s to bleed off speed and level out. No setpoints.
        REENGAGE  back in GUIDED; re-approach the target speed-limited, ramping
          |       from SAFE_TURN_SPEED up to full over reengage_ramp_s, no
          |       extension/latch, until closing again (or a timeout).
        CHASE     resume full guidance.

    Only the transitions are decided here; the loop applies the flight mode,
    whether to send a setpoint, and the speed cap.
    """

    CHASE = "CHASE"
    HOLD = "HOLD"
    REENGAGE = "REENGAGE"

    def __init__(self, args):
        self.enabled = bool(args.miss_recovery) and not args.no_guided
        self.recovery_mode = str(args.recovery_mode)
        self.hold_s = max(0.0, float(args.recovery_hold_s))
        self.ramp_s = max(0.0, float(args.reengage_ramp_s))
        self.max_reengage_s = max(0.1, float(args.reengage_max_s))
        self.min_closing = float(getattr(cfg, "REENGAGE_MIN_CLOSING_MPS", 2.0))
        self.safe_turn_speed = float(getattr(cfg, "SAFE_TURN_SPEED", 7.0))
        self.max_speed = float(args.max_feedforward_speed)
        # Debounce (2026-07-11: latch noise fired BRAKE 12x in one flight,
        # several at 100+ m true range; each BRAKE-at-speed is a saturation
        # event). A fresh CHASE gets chase_dwell_s to fly before a miss may
        # interrupt it, and HOLDs are spaced at least brake_interval_s apart.
        self.chase_dwell_s = float(getattr(cfg, "RECOVERY_CHASE_DWELL_S", 15.0))
        self.brake_interval_s = float(
            getattr(cfg, "RECOVERY_BRAKE_MIN_INTERVAL_S", 5.0)
        )
        # Altitude-divergence abort: fire HOLD/BRAKE when the vehicle can't hold
        # its commanded altitude (motor saturation -> thrust collapse), a mode the
        # range-based miss trigger is blind to. Arms only after altitude is first
        # acquired so the initial climb doesn't trip it; disarms on fire until
        # re-acquired to avoid thrash. See RECOVERY_ALT_ABORT_M in the config.
        self.alt_abort_m = float(getattr(cfg, "RECOVERY_ALT_ABORT_M", 15.0))
        self.alt_abort_arm_m = float(getattr(cfg, "RECOVERY_ALT_ABORT_ARM_M", 3.0))
        self._alt_abort_armed = False
        # Yaw-rate hold gate: a fixed hold_s timer released the vehicle while it
        # was still tumbling from the pass. Extend the hold while it is still
        # spinning, bounded by hold_max_s so it can never stick.
        # CPA trigger: declare the terminal pass once range has climbed
        # cpa_margin_m above the minimum seen while latched, instead of waiting
        # for the latch to release at TERMINAL_LATCH_RELEASE_RANGE_M. Consumes
        # only time-consistent packet samples (caller passes range_m=None on
        # non-packet loops) and requires cpa_open_samples consecutive
        # beyond-margin samples -- the every-loop range is a +-5-8 m packet
        # staircase that fired the naive version BEFORE the true pass
        # (2026-07-24 review, log 110805 full-res).
        self.cpa_margin_m = float(getattr(cfg, "RECOVERY_CPA_MARGIN_M", 0.0))
        self.cpa_open_samples = max(
            1, int(getattr(cfg, "RECOVERY_CPA_OPEN_SAMPLES", 2))
        )
        self._cpa_min_range = None
        self._cpa_open_count = 0
        self.yaw_hold_rate = float(
            np.radians(float(getattr(cfg, "RECOVERY_YAW_RATE_HOLD_DPS", 0.0)))
        )
        self.hold_max_s = max(
            self.hold_s, float(getattr(cfg, "RECOVERY_HOLD_MAX_S", 5.0))
        )
        # HOLD'DAN CIKISI IRTIFANIN TOPARLANMASINA BAGLA (2026-08-09).
        # VARSAYILAN ACIK; YILDIZ_HOLD_IRTIFA=0 eski (yalniz-zamanlayici)
        # davranisi geri getirir. Bu bir DENEY KOLU DEGIL, HATA DUZELTMESI:
        #
        # KANIT ZINCIRI
        #  1) 12 kosuda TAKLA/SAGLAM ayrimini %100 dogru yapan tek olcu
        #     SUREKLI IRTIFA HATASI: alt_err p95 56-57 m (4 takla) vs
        #     19-33 m (8 saglam); alt_err>15 m orani %66-91 vs %9-18.
        #     Butun kosular bir noktada 15 m'yi asiyor -- fark ASMAK degil,
        #     TOPARLANAMAMAK.
        #  2) ALTITUDE ABORT yatayi DOGRU kesiyor (send=False, speed_cap=0)
        #     ama HOLD'dan cikis YALNIZ ZAMANLAYICIYDI (hold_s=2 s; uzatma
        #     sadece `spinning` icin). Ariza SURERKEN hiz geri veriliyordu.
        #     kpn26 zaman cizelgesi: t=25.8 HOLD alt_err 15.6 ->
        #     t=30.8 REENGAGE alt_err 12.9 (13 m hatayla hiz geri) ->
        #     t=34.8 CHASE alt_err 20.2; TAKLA t=32.3 (REENGAGE ortasinda).
        #  3) Kacak dongu: irtifa hatasi -> agresif dikey+yatay talep ->
        #     buyuk yatma (takla oncesi tilt ORTANCASI 67 deg) -> cos kaybi
        #     (54 deg'de dikey bilesen 0.59) -> gaz zaten karelerin
        #     %10-23'unde TAM DOYGUN -> irtifa tutulamaz -> hata buyur.
        # DUZELTME: `spinning` uzatmasiyla AYNI desen, AYNI tavan
        # (hold_max_s), AYNI fail-open (alt_error None -> eski davranis) ve
        # YENI ESIK YOK: alt_abort_arm_m zaten "abort su degerin altina
        # inince yeniden silahlanir" diye tanimli, yani "irtifa toparlandi"
        # olcutunun TA KENDISI. Yan fayda: HOLD uzayinca alt_err o esigin
        # altina inme sansi artar ve ABORT YENIDEN SILAHLANIR (ikinci
        # zayiflik: :342 kendini silahsizlandiriyor, tekrar silahlanma
        # sarti alt_error <= alt_abort_arm_m).
        # NOT: hold_max_s 5 -> 8 BILINCLI OLARAK YAPILMADI (TO_TEST adayi);
        # bugun uc kez "ikinci dugme no-op cikti" dersi alindi, once tek
        # degisken olculecek.
        self.hold_irtifa = str(os.environ.get('YILDIZ_HOLD_IRTIFA', '1')) != '0'
        self._hold_extended = False
        self.state = self.CHASE
        self.state_t0 = 0.0
        self.chase_t0 = None
        self.last_hold_t = None
        self._prev_latched = False

    def _chase(self):
        return {
            "state": self.CHASE,
            "mode": "GUIDED",
            "send": True,
            "speed_cap": self.max_speed,
        }

    def update(
        self, now, latched, closing_velocity,
        alt_error=None, yaw_rate=None, range_m=None,
    ):
        """Advance the state machine one loop and return the directive.

        alt_error is |pursuer_z - commanded_z| [m]; when it exceeds
        alt_abort_m the vehicle has lost altitude control (saturation) and we
        HOLD/BRAKE immediately, independent of the range-based miss detection.

        range_m is the pursuer-to-estimate range [m]; while latched, a rise of
        cpa_margin_m above its minimum means the pass has happened (CPA) and we
        hand off to recovery immediately instead of waiting for the latch to
        release 2-6 s later.

        yaw_rate is the body yaw rate [rad/s] (None if unavailable): while it
        exceeds the threshold the HOLD window is extended so the vehicle is not
        handed back to GUIDED mid-tumble.
        """
        if not self.enabled:
            return self._chase()

        if self.state == self.CHASE:
            if self.chase_t0 is None:
                self.chase_t0 = now
            # Arm the altitude abort once the vehicle has demonstrated it can
            # hold the commanded altitude (excludes the initial climb).
            if (
                self.alt_abort_m > 0.0
                and alt_error is not None
                and alt_error <= self.alt_abort_arm_m
            ):
                self._alt_abort_armed = True
            # Altitude-divergence abort: highest priority, bypasses the miss
            # dwell/spacing debounce (a thrust collapse is an emergency, not
            # latch noise). Disarm until altitude is re-acquired so it does not
            # thrash while REENGAGE is still recovering.
            if (
                self._alt_abort_armed
                and alt_error is not None
                and alt_error > self.alt_abort_m
            ):
                self._alt_abort_armed = False
                self._prev_latched = bool(latched)
                # Clear CPA tracking too: a stale minimum from this aborted
                # approach must not survive into the next engagement.
                self._cpa_min_range = None
                self._cpa_open_count = 0
                self.state = self.HOLD
                self.state_t0 = now
                self.last_hold_t = now
                print(
                    f"[{redcolor}simple_follow{endcolor}] ALTITUDE ABORT: "
                    f"|pursuer_z-cmd_z|={alt_error:.1f} m > {self.alt_abort_m:.0f} m "
                    f"(thrust/attitude collapse) -> HOLD/{self.recovery_mode}"
                )
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            # CPA trigger: while the aim is latched, track the closest range
            # over TIME-CONSISTENT packet samples (range_m is None between
            # packets; the tracker holds). Once cpa_open_samples consecutive
            # samples sit cpa_margin_m above the minimum, the pass is real --
            # hand off NOW rather than waiting for the latch to release at
            # 40 m. Bypasses the chase dwell (a confirmed pass is not latch
            # noise); if the brake spacing forbids another HOLD, release the
            # latch instead so the vehicle is never left flying a frozen aim
            # it has already overshot (the balloon mechanism).
            if not latched:
                self._cpa_min_range = None
                self._cpa_open_count = 0
            elif self.cpa_margin_m > 0.0 and range_m is not None:
                r = float(range_m)
                if self._cpa_min_range is None or r < self._cpa_min_range:
                    self._cpa_min_range = r
                    self._cpa_open_count = 0
                elif r > self._cpa_min_range + self.cpa_margin_m:
                    self._cpa_open_count += 1
                    if self._cpa_open_count >= self.cpa_open_samples:
                        closest = self._cpa_min_range
                        samples = self._cpa_open_count
                        self._cpa_min_range = None
                        self._cpa_open_count = 0
                        spacing_ok = (
                            self.last_hold_t is None
                            or (now - self.last_hold_t) >= self.brake_interval_s
                        )
                        if spacing_ok:
                            self._prev_latched = bool(latched)
                            self.state = self.HOLD
                            self.state_t0 = now
                            self.last_hold_t = now
                            print(
                                f"[{redcolor}simple_follow{endcolor}] CPA PASS: "
                                f"closest {closest:.1f} m, opening at {r:.1f} m "
                                f"({samples} packet samples) -> HOLD/"
                                f"{self.recovery_mode} (stop flying the frozen aim)"
                            )
                            return {
                                "state": self.HOLD,
                                "mode": self.recovery_mode,
                                "send": False,
                                "speed_cap": 0.0,
                            }
                        # Pass confirmed but a HOLD is too soon after the last
                        # one: drop the frozen aim and keep chasing live
                        # geometry instead of ballooning against it.
                        self._prev_latched = False
                        print(
                            f"[{redcolor}simple_follow{endcolor}] CPA PASS: "
                            f"closest {closest:.1f} m, opening at {r:.1f} m -- "
                            f"HOLD suppressed (brake spacing), releasing latch "
                            f"and staying in CHASE"
                        )
                        directive = self._chase()
                        directive["unlatch"] = True
                        return directive
                else:
                    # Back inside the margin band: the opening streak is broken.
                    self._cpa_open_count = 0
            miss = self._prev_latched and not latched
            self._prev_latched = bool(latched)
            if miss:
                dwell_ok = (now - self.chase_t0) >= self.chase_dwell_s
                spacing_ok = (
                    self.last_hold_t is None
                    or (now - self.last_hold_t) >= self.brake_interval_s
                )
                if not (dwell_ok and spacing_ok):
                    reason = "chase dwell" if not dwell_ok else "brake spacing"
                    print(
                        f"[{redcolor}simple_follow{endcolor}] miss detected but "
                        f"HOLD suppressed ({reason}); staying in CHASE"
                    )
                    return self._chase()
                self.state = self.HOLD
                self.state_t0 = now
                self.last_hold_t = now
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            return self._chase()

        if self.state == self.HOLD:
            held = now - self.state_t0
            # Still spinning? Keep the recovery mode and go on stabilising
            # rather than handing a tumbling vehicle back to GUIDED. Fails open
            # when yaw_rate is None (no/stale ATTITUDE) and is capped by
            # hold_max_s so a stuck-high yaw rate can never freeze the machine.
            spinning = (
                self.yaw_hold_rate > 0.0
                and yaw_rate is not None
                and abs(float(yaw_rate)) > self.yaw_hold_rate
            )
            # Irtifa hala bozuksa HOLD'da kal (bkz. self.hold_irtifa notu):
            # yatay talep kesik kalir, itki butcesi dikeye ayrilir, kacak
            # dongu kirilir. `spinning` ile ayni tavana (hold_max_s) tabi.
            irtifa_bozuk = (
                self.hold_irtifa
                and alt_error is not None
                and float(alt_error) > self.alt_abort_arm_m
            )
            if held < self.hold_s or ((spinning or irtifa_bozuk)
                                      and held < self.hold_max_s):
                if (spinning or irtifa_bozuk) and held >= self.hold_s \
                        and not self._hold_extended:
                    self._hold_extended = True
                    # SEBEP-FARKINDA mesaj: uzatma artik iki nedenden
                    # olabiliyor. yaw_rate irtifa kolunda None OLABILIR --
                    # eski metin onu kosulsuz bicimliyordu, o yuzden
                    # sebebe gore ayri yaziliyor (None'da cokmemesi sart).
                    if spinning:
                        sebep = (
                            f"yaw rate {np.degrees(abs(float(yaw_rate))):.0f} deg/s > "
                            f"{np.degrees(self.yaw_hold_rate):.0f} -- still spinning"
                        )
                    else:
                        sebep = (
                            f"alt_err {float(alt_error):.1f} m > "
                            f"{self.alt_abort_arm_m:.1f} -- altitude not recovered"
                        )
                    print(
                        f"[{redcolor}simple_follow{endcolor}] HOLD extended: {sebep}, "
                        f"keep stabilising (cap {self.hold_max_s:.1f}s)"
                    )
                return {
                    "state": self.HOLD,
                    "mode": self.recovery_mode,
                    "send": False,
                    "speed_cap": 0.0,
                }
            if self._hold_extended:
                yr = "n/a" if yaw_rate is None else f"{np.degrees(abs(float(yaw_rate))):.0f} deg/s"
                ae = 'n/a' if alt_error is None else f"{float(alt_error):.1f} m"
                print(
                    f"[{redcolor}simple_follow{endcolor}] HOLD released after "
                    f"{held:.1f}s (yaw rate {yr}, alt_err {ae}) -> REENGAGE"
                )
            self._hold_extended = False
            self.state = self.REENGAGE
            self.state_t0 = now

        # REENGAGE
        elapsed = now - self.state_t0
        frac = 1.0 if self.ramp_s <= 1e-6 else min(1.0, elapsed / self.ramp_s)
        cap = self.safe_turn_speed + (self.max_speed - self.safe_turn_speed) * frac
        reengaged = elapsed >= self.ramp_s and float(closing_velocity) >= self.min_closing
        if reengaged or elapsed >= self.max_reengage_s:
            self.state = self.CHASE
            self.chase_t0 = now
            self._prev_latched = False
            return self._chase()
        return {
            "state": self.REENGAGE,
            "mode": "GUIDED",
            "send": True,
            "speed_cap": cap,
        }


def horizontal_unit(v_xy, fallback_xy):
    norm = float(np.linalg.norm(v_xy))
    if norm > 1e-6:
        return v_xy / norm
    return fallback_xy.copy()


def yaw_to_los(target_pos, pursuer_pos, fallback_yaw=0.0, min_range_m=0.0):
    rel_xy = (
        np.asarray(target_pos, dtype=float).reshape(3)[0:2]
        - np.asarray(pursuer_pos, dtype=float).reshape(3)[0:2]
    )
    if float(np.linalg.norm(rel_xy)) < float(min_range_m):
        return fallback_yaw
    if float(np.dot(rel_xy, rel_xy)) < 1e-6:
        return fallback_yaw
    return float(np.arctan2(rel_xy[1], rel_xy[0]))


def wrap_angle_pi(angle_rad):
    return (float(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_to_velocity(vel, fallback_yaw=0.0, min_speed_mps=0.0):
    """Heading along the (horizontal) velocity vector -- fly forward, zero crab.

    Holds fallback_yaw when too slow for the course to be well defined. Unlike
    yaw_to_los this does NOT blow up at close range (the velocity direction
    changes smoothly while the LOS bearing rate ~ 1/range explodes).
    """
    v_xy = np.asarray(vel, dtype=float).reshape(3)[0:2]
    if float(np.linalg.norm(v_xy)) < float(min_speed_mps):
        return float(fallback_yaw)
    return float(np.arctan2(v_xy[1], v_xy[0]))


def slew_angle_toward(current_yaw, target_yaw, max_rate_deg_s, dt):
    max_step = np.deg2rad(max(0.0, float(max_rate_deg_s))) * max(float(dt), 0.0)
    if max_step <= 0.0:
        return float(current_yaw)
    delta = wrap_angle_pi(target_yaw - current_yaw)
    delta = float(np.clip(delta, -max_step, max_step))
    return wrap_angle_pi(current_yaw + delta)


def clamp_commanded_turn(
    slot_pos,
    slot_vel,
    target_pos,
    prev_cmd_heading,
    dt,
    a_lat_max_mps2,
    min_speed_mps=2.0,
    rotate_position=True,
    max_rotation_deg=10.0,
):
    """Cap the commanded turn to a fixed bank margin (command conditioning).

    The bank a copter must hold to track a moving pos+vel setpoint is set by the
    setpoint's LATERAL acceleration, a_lat = speed * d(heading)/dt. A co-speed
    pursuer following a hard target turn banks to ANGLE_MAX, and at the tilt
    limit the mixer sheds yaw authority first -> any disturbance diverges (the
    spin / balloon / fall crash family). We never want to command more turn than
    a fixed bank margin can execute, so we rate-limit the heading of the
    horizontal velocity command to omega_max = a_lat_max / speed. a_lat_max =
    g*tan(bank_margin); 6.9 m/s^2 implies ~35 deg, 10 deg under a 45 deg
    ANGLE_MAX.

    The SAME angular correction is applied to the horizontal position LEAD
    OFFSET -- the slot's offset ABOUT THE TARGET (slot_pos - target_pos, tens of
    metres) -- when rotate_position is set, so the position controller does not
    refight the smoothed velocity feedforward by chasing a sharper-curving
    position setpoint.

    2026-08-02 DUZELTMESI: eskiden bu donus TUM gorus hattina (slot_pos -
    pursuer_pos, 600-1400 m) ve AVCININ etrafina uygulaniyordu. Docstring'in
    tarif ettigi ~25 m'lik lead ofseti yerine kilometrelik vektorun donmesi
    ayni aciyi ~50 kat buyutuyor, ve dongu yavasladiginda komut hedeften
    BAGIMSIZ olarak sabit hizla donup dev bir daire ciziyordu (678 m yaricap).
    Ayrica cagri basina uygulanan donus max_rotation_deg ile SERT sinirlanir:
    birikmis (+-180 dereceye kadar cikabilen) hata tek adimda uygulanamaz.

    Returns (slot_pos, slot_vel, new_heading, implied_a_lat, clamped) where
    implied_a_lat is the RAW demanded lateral accel this loop (pre-clamp) -- the
    signal that says how sharp a turn the target actually flew.
    """
    pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
    vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
    tgt = np.asarray(target_pos, dtype=float).reshape(3)
    v_xy = vel[0:2]
    speed = float(np.linalg.norm(v_xy))
    dt = max(float(dt), 1e-3)
    # Heading is ill-defined below the speed floor: hold state, no clamp.
    if speed < float(min_speed_mps):
        return pos, vel, prev_cmd_heading, 0.0, False
    psi_raw = float(np.arctan2(v_xy[1], v_xy[0]))
    if prev_cmd_heading is None:
        # First valid sample: seed the heading, nothing to rate-limit against.
        return pos, vel, psi_raw, 0.0, False
    dpsi_raw = wrap_angle_pi(psi_raw - float(prev_cmd_heading))
    implied_a_lat = speed * abs(dpsi_raw) / dt
    omega_max = float(a_lat_max_mps2) / max(speed, float(min_speed_mps))
    max_step = omega_max * dt
    dpsi = float(np.clip(dpsi_raw, -max_step, max_step))
    clamped = abs(dpsi_raw) > max_step + 1e-9
    rot_corr = dpsi - dpsi_raw  # rotate the raw command back into the envelope
    # Tek dongude uygulanabilecek donus SERT tavani: rot_corr birikmis basligi
    # tasidigi icin (+-180 dereceye kadar) sinirsiz uygulanmasi komutu hedeften
    # kopariyordu. Kirpilinca uygulanan baslik ham ile kelepcelenmis arasinda
    # kalir; state tutarli olsun diye new_heading UYGULANAN aciyla yazilir.
    max_rot = np.deg2rad(max(0.0, float(max_rotation_deg)))
    if max_rot > 0.0:
        rot_corr = float(np.clip(rot_corr, -max_rot, max_rot))
    if abs(rot_corr) > 1e-12:
        c, s = float(np.cos(rot_corr)), float(np.sin(rot_corr))
        vx, vy = float(v_xy[0]), float(v_xy[1])
        vel[0] = c * vx - s * vy
        vel[1] = s * vx + c * vy
        if rotate_position:
            # Lead ofsetini HEDEFIN etrafinda dondur (slot_pos - target_pos):
            # kelepce, gorus hattini degil yalnizca one-alma ofsetini duzeltir.
            ox = float(pos[0] - tgt[0])
            oy = float(pos[1] - tgt[1])
            pos[0] = float(tgt[0]) + (c * ox - s * oy)
            pos[1] = float(tgt[1]) + (s * ox + c * oy)
    new_heading = wrap_angle_pi(psi_raw + rot_corr)
    return pos, vel, new_heading, implied_a_lat, clamped


# Set once the runner owns the vehicle's mode, so the shutdown hook can put it
# somewhere safe no matter how we exit (Ctrl-C, SIGTERM, or an exception).
_SHUTDOWN_CONN = None
_SHUTDOWN_DONE = False


def stabilize_on_exit(reason=""):
    """Command the configured shutdown mode (default POSHOLD) once, on exit.

    Without this the vehicle keeps flying the last GUIDED setpoint after the
    script stops. Idempotent, never raises: shutdown must not be able to mask
    the original error or fail twice.
    """
    global _SHUTDOWN_DONE
    if _SHUTDOWN_DONE or _SHUTDOWN_CONN is None:
        return
    _SHUTDOWN_DONE = True
    mode = str(getattr(cfg, "SHUTDOWN_MODE", "POSHOLD") or "").strip()
    if not mode:
        return
    try:
        set_mode(_SHUTDOWN_CONN, mode)
        note = f" ({reason})" if reason else ""
        print(f"[{redcolor}simple_follow{endcolor}] Shutdown{note}: pursuer -> {mode}")
    except Exception as exc:
        print(
            f"[{redcolor}simple_follow{endcolor}] Shutdown: could not set {mode}: {exc}"
        )


def set_mode(master, mode_name):
    modes = master.mode_mapping()
    if not modes:
        # mode_mapping() returns None until the vehicle type is known; without
        # this guard the membership test raises TypeError instead of the
        # intended, catchable RuntimeError.
        raise RuntimeError("Vehicle mode mapping not available yet (no heartbeat?)")
    if mode_name not in modes:
        raise RuntimeError(f"Mode {mode_name!r} is not available on this vehicle")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes[mode_name],
    )


def wait_for_vehicle_heartbeat(
    master, label, expected_sysid=None, timeout_s=10.0, abort_check=None
):
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while True:
        if abort_check is not None and abort_check():
            raise RuntimeError(f"Aborted while waiting for {label} heartbeat")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"Timed out waiting for {label} heartbeat")
        # Recv in short slices so a SIGINT (which only flips the running flag)
        # is noticed within ~0.5 s instead of after the full timeout (PEP 475
        # restarts the interrupted recv otherwise).
        msg = master.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(0.5, remaining)
        )
        if msg is None:
            continue

        src_sys = int(msg.get_srcSystem())
        src_comp = int(msg.get_srcComponent())
        is_gcs = src_comp == mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER or int(
            getattr(msg, "type", -1)
        ) == int(mavutil.mavlink.MAV_TYPE_GCS)
        if expected_sysid is not None and src_sys != int(expected_sysid):
            continue
        if is_gcs:
            continue

        master.target_system = src_sys
        master.target_component = src_comp
        return msg


def request_message_interval(master, message_id, rate_hz):
    interval_us = 0
    if float(rate_hz) > 0.0:
        interval_us = int(round(1_000_000.0 / float(rate_hz)))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def set_param(master, name, value, param_type=None):
    if param_type is None:
        if float(value).is_integer():
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        else:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        str(name).encode("ascii"),
        float(value),
        int(param_type),
    )


def read_param_value(master, name, timeout_s=1.5):
    """PARAM_VALUE ile parametrenin GERCEK degerini oku (None = cevap yok).

    PARAM_SET fire-and-forget'tir: gonderilmesi uygulandigi anlamina GELMEZ.
    2026-08-02 dataflash'inda WPNAV_ACCEL 250'de kaldi (betik 500 istiyordu) ve
    kimse fark etmedi. Bu yuzden her yazimin ardindan geri-okuma yapilir.
    """
    want = str(name).strip().upper()
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        want.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        msg = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=min(0.5, remaining)
        )
        if msg is None:
            continue
        got = msg.param_id
        if isinstance(got, bytes):
            got = got.decode("ascii", "ignore")
        if str(got).strip("\x00").strip().upper() == want:
            return float(msg.param_value)


def param_value_matches(actual, wanted):
    """Float toleransli karsilastirma (MAVLink her parametreyi float tasir)."""
    return abs(float(actual) - float(wanted)) <= max(
        1e-3, abs(float(wanted)) * 1e-3
    )


def assert_startup_params(master, param_map, attempts=3, timeout_s=1.5):
    """PARAM_SET gonder, PARAM_VALUE ile DOGRULA, tutmayani tekrar dene.

    Uctan fazla denemede oturmayan parametreleri gurultulu bicimde raporlar ve
    listesini dondurur (bos liste = hepsi oturdu). Cagiran taraf bunu avci
    heartbeat'i ALINDIKTAN sonra cagirmali; hedef sistem/komponent bilinmeden
    PARAM_SET bir yere gitmez.
    """
    failed = []
    for name, value in param_map.items():
        actual = None
        ok = False
        for attempt in range(1, max(1, int(attempts)) + 1):
            set_param(master, name, value)
            actual = read_param_value(master, name, timeout_s)
            if actual is not None and param_value_matches(actual, value):
                print(
                    f"[{redcolor}simple_follow{endcolor}] param_set {name}={value} "
                    f"DOGRULANDI (okunan {actual:g}, {attempt}. deneme)"
                )
                ok = True
                break
        if not ok:
            failed.append((name, value, actual))
    if failed:
        print(
            f"[{redcolor}simple_follow{endcolor}] *** UYARI: {len(failed)} "
            f"parametre UYGULANMADI (istenen -> okunan): "
            + ", ".join(
                f"{n}={v}->" + ("cevap yok" if a is None else f"{a:g}")
                for n, v, a in failed
            )
            + " -- ucus profili beklenenden FARKLI olacak ***"
        )
    return failed


def fetch_pursuer_home(pursuer_conn):
    pursuer_conn.mav.command_long_send(
        pursuer_conn.target_system,
        pursuer_conn.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    msg = pursuer_conn.recv_match(type="HOME_POSITION", blocking=True, timeout=5.0)
    if msg is not None:
        return msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0

    msg = pursuer_conn.recv_match(
        type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0
    )
    if msg is not None:
        return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0

    raise RuntimeError("Could not determine pursuer home/global origin")


def fetch_home_altitude(conn, timeout_s=3.0):
    """Best-effort HOME_POSITION altitude [m AMSL], or None.

    Diagnostic only: used to compare the two vehicles' idea of the field's
    elevation at startup. Never raises and never blocks startup -- a vehicle
    that does not answer simply yields None.
    """
    try:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0,
        )
        msg = conn.recv_match(
            type="HOME_POSITION", blocking=True, timeout=max(0.1, float(timeout_s))
        )
        if msg is not None:
            return float(msg.altitude) / 1000.0
    except Exception:
        pass
    return None


def predict_state_copy(imm, horizon_s, max_substep=None):
    if horizon_s <= 0.0:
        return imm.x.copy()

    backup_xs = [kf.x.copy() for kf in imm.filters]
    backup_ps = [kf.P.copy() for kf in imm.filters]
    backup_mu = imm.mu.copy()
    backup_x = imm.x.copy()
    backup_p = imm.P.copy()

    predict_imm_over_dt(imm, horizon_s, max_substep=max_substep)
    predicted = imm.x.copy()

    for kf, x, p in zip(imm.filters, backup_xs, backup_ps):
        kf.x = x
        kf.P = p
    imm.mu = backup_mu
    imm.x = backup_x
    imm.P = backup_p

    return predicted


def clamp_lead_envelope(imm):
    """Bound each filter's poorly-observed states to the target's flight
    envelope, IN PLACE. Must be called inside a backup/restore of the filters.

    The lead is dominated by position + velocity + turn rate (all well
    observed). The states the horizon amplifies -- linear acceleration
    (ax,ay,az via ~t^2/2) and the CT centripetal (v*omega) -- are the least
    observable, so a transient estimate of a few m/s^2 becomes tens of metres
    of aim-point swing. Here we (a) clamp horizontal linear accel magnitude and
    vertical accel to the envelope and scale each by its inclusion factor, (b)
    optionally drop vz, and (c) cap omega so v*omega stays within the lateral
    envelope (OMEGA_ABS_MAX=1.5 alone permits ~28 m/s^2 at 19 m/s).
    """
    lat_max = float(getattr(cfg, "LEAD_LATERAL_ACCEL_MAX_MPS2", 5.0))
    az_max = float(getattr(cfg, "LEAD_VERTICAL_ACCEL_MAX_MPS2", 4.0))
    h_scale = float(getattr(cfg, "LEAD_HORIZONTAL_ACCEL_SCALE", 1.0))
    z_scale = float(getattr(cfg, "LEAD_VERTICAL_ACCEL_SCALE", 0.0))
    vz_scale = float(getattr(cfg, "LEAD_VERTICAL_VELOCITY_SCALE", 0.0))
    for kf in imm.filters:
        x = kf.x
        axy_n = float(np.hypot(x[6], x[7]))
        if axy_n > lat_max and axy_n > 1e-9:
            scale = lat_max / axy_n
            x[6] *= scale
            x[7] *= scale
        x[6] *= h_scale
        x[7] *= h_scale
        x[8] = float(np.clip(x[8], -az_max, az_max)) * z_scale
        x[5] *= vz_scale
        spd = float(np.hypot(x[3], x[4]))
        if spd > 1e-3:
            omega_env = lat_max / spd
            x[9] = float(np.clip(x[9], -omega_env, omega_env))


def predict_lead_state(imm, horizon_s, max_substep=None):
    """Envelope-clamped forward prediction for the guidance lead.

    Returns (x_pred, P_pred): the predicted mixed state and its covariance,
    leaving `imm` untouched. Unlike predict_state_copy it clamps the poorly
    observed states to the flight envelope first (see clamp_lead_envelope), so
    the ~t^2 amplification cannot manufacture an impossible aim point.
    """
    if horizon_s <= 0.0:
        return imm.x.copy(), imm.P.copy()

    backup_xs = [kf.x.copy() for kf in imm.filters]
    backup_ps = [kf.P.copy() for kf in imm.filters]
    backup_mu = imm.mu.copy()
    backup_x = imm.x.copy()
    backup_p = imm.P.copy()

    if bool(getattr(cfg, "LEAD_ENVELOPE_CLAMP_ENABLED", True)):
        clamp_lead_envelope(imm)
    predict_imm_over_dt(imm, horizon_s, max_substep=max_substep)
    x_pred = imm.x.copy()
    p_pred = imm.P.copy()

    for kf, x, p in zip(imm.filters, backup_xs, backup_ps):
        kf.x = x
        kf.P = p
    imm.mu = backup_mu
    imm.x = backup_x
    imm.P = backup_p

    return x_pred, p_pred


def lead_cov_gate(x_pred, p_pred, x_est):
    """Shrink the position lead toward the current estimate when the filter's
    own predicted covariance says the extrapolation is uncertain.

    The predicted position 1-sigma grows with the horizon and with how poorly
    the target is being tracked; when it is large, the mean lead is noise, so
    aiming at it is worse than not leading. Returns (aim_state, k_h, k_v) where
    k_* in [0,1] is the applied lead fraction (horizontal / vertical). Only the
    position channels are gated; velocity FF is left as estimated.
    """
    aim = np.asarray(x_pred, dtype=float).copy()
    if not bool(getattr(cfg, "LEAD_COV_GATE_ENABLED", True)):
        return aim, 1.0, 1.0

    est = np.asarray(x_est, dtype=float).reshape(-1)
    var = np.clip(np.asarray(np.diag(p_pred), dtype=float)[0:3], 0.0, None)
    sig = np.sqrt(var)
    sig_h = float(np.hypot(sig[0], sig[1]) / np.sqrt(2.0))  # rms horizontal std
    sig_v = float(sig[2])

    floor = float(getattr(cfg, "LEAD_COV_GATE_MIN_FRAC", 0.0))

    def frac(s, lo, hi):
        if hi <= lo:
            return 1.0
        return float(np.clip((hi - s) / (hi - lo), floor, 1.0))

    k_h = frac(
        sig_h,
        float(getattr(cfg, "LEAD_COV_GATE_H_SIGMA_LO_M", 10.0)),
        float(getattr(cfg, "LEAD_COV_GATE_H_SIGMA_HI_M", 45.0)),
    )
    k_v = frac(
        sig_v,
        float(getattr(cfg, "LEAD_COV_GATE_V_SIGMA_LO_M", 6.0)),
        float(getattr(cfg, "LEAD_COV_GATE_V_SIGMA_HI_M", 22.0)),
    )

    aim[0] = est[0] + k_h * (float(x_pred[0]) - est[0])
    aim[1] = est[1] + k_h * (float(x_pred[1]) - est[1])
    aim[2] = est[2] + k_v * (float(x_pred[2]) - est[2])
    return aim, k_h, k_v


class LeadPredictionCache:
    """Reuse the guidance lead prediction between target packets.

    The lead point is the predicted intercept location: it only carries new
    information when a measurement has updated the IMM, or when the intercept
    solution shifts materially (t_go, and with it the horizon). Recomputing it
    every guidance loop is what starved the 2026-07-11 flight's control loop:
    a 6 s horizon at 0.1 s substeps x 6 filters measured 415-660 ms per loop,
    so "30 Hz" guidance actually ran at ~2 Hz exactly when the horizon was
    long. Packets arrive at ~4-5 Hz; between them the cached point is simply
    the same impact-point estimate and costs nothing.

    Caches the envelope-clamped prediction (x_pred, P_pred). The covariance
    gate (lead_cov_gate) is cheap and runs every loop on the cached pair.
    """

    def __init__(self, max_substep, horizon_tol_s=0.5):
        self.max_substep = float(max_substep)
        self.horizon_tol_s = float(horizon_tol_s)
        self.x_pred = None
        self.p_pred = None
        self.horizon_s = None

    def get(self, imm, horizon_s, imm_updated):
        horizon_s = float(horizon_s)
        if (
            self.x_pred is None
            or imm_updated
            or abs(horizon_s - self.horizon_s) > self.horizon_tol_s
        ):
            self.x_pred, self.p_pred = predict_lead_state(
                imm, horizon_s, max_substep=self.max_substep
            )
            self.horizon_s = horizon_s
        return self.x_pred.copy(), self.p_pred.copy()


class BehindSlotGuidance:
    def __init__(self, back_m, side_m, down_m, min_heading_speed):
        self.back_m = float(back_m)
        self.side_m = float(side_m)
        self.down_m = float(down_m)
        self.min_heading_speed = float(min_heading_speed)
        self.heading_xy = np.array([1.0, 0.0])

    def update(self, target_state, pursuer_pos):
        target_pos = np.asarray(target_state[0:3], dtype=float).reshape(3)
        target_vel = np.asarray(target_state[3:6], dtype=float).reshape(3)
        pursuer_pos = np.asarray(pursuer_pos, dtype=float).reshape(3)

        target_speed_xy = float(np.linalg.norm(target_vel[0:2]))
        if target_speed_xy >= self.min_heading_speed:
            heading_xy = target_vel[0:2] / target_speed_xy
        else:
            rel_xy = target_pos[0:2] - pursuer_pos[0:2]
            heading_xy = horizontal_unit(rel_xy, self.heading_xy)

        self.heading_xy = heading_xy.copy()

        heading = np.array([heading_xy[0], heading_xy[1], 0.0])
        right = np.array([-heading_xy[1], heading_xy[0], 0.0])
        down = np.array([0.0, 0.0, 1.0])

        slot_pos = (
            target_pos
            - self.back_m * heading
            + self.side_m * right
            + self.down_m * down
        )

        slot_vel = target_vel.copy()
        return slot_pos, slot_vel, target_pos


def kesisme_zamani(rel_pos, tgt_vel, atk_speed):
    """|rel_pos + tgt_vel*t| = atk_speed*t  koklerinin en kucuk pozitifi.

    formation_KILLER.solve_intercept_time'in BIREBIR ayni matematigi
    (o dosya bir SURU kontrolcusudur ve config.py/waypoints.json olmadan
    kosmaz; burada yalnizca YASA tasindi, program degil).

        (|Vt|^2 - Sa^2) t^2 + 2 (R . Vt) t + |R|^2 = 0
    """
    vt2 = float(np.dot(tgt_vel, tgt_vel))
    a = vt2 - atk_speed * atk_speed
    b = 2.0 * float(np.dot(rel_pos, tgt_vel))
    c = float(np.dot(rel_pos, rel_pos))
    if abs(a) < 1e-6:                       # hedef hizi == avci hizi
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t > 1e-3 else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = float(np.sqrt(disc))
    kokler = [(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
    poz = [t for t in kokler if t > 1e-3]
    return min(poz) if poz else None


class InterceptGuidance:
    """KESISME (carpisma rotasi) yaklasimi -- BehindSlotGuidance'in ALTERNATIFI.

    Iki yasa arasindaki fark TEMELDIR, karistirmayin:
      * BehindSlotGuidance  : hedefin ARKASINDA bir slota gider (back/down).
        Hedefe NISAN ALMAZ; kuyruk takibi geometrisi kurar, kamera hedefi
        onde gorur. Goruntuluye devir bu geometriden yapilir.
      * InterceptGuidance   : hedefin GIDECEGI yere (lead) nisan alir ve
        kesismenin OTESINE uzatir. Yani bastan carpisma rotasidir; devir
        ani genelde CAPRAZ/KAFA-KAFAYA geometride olur.

    formation_KILLER'daki ATTACK fazinin yasasi budur (INT/PURSUE/TERM
    safhalari + OVERSHOOT uzatmasi aynen korundu). Hedef hiz vektoru
    KONUMLU fazda zaten kullaniliyor (estimator turetiyor); 'yalniz menzil'
    kurali GORUNTULU faza aittir, bu yasa devirden ONCE calisir.

    UYARI (olculdu): capraz/kafa-kafaya devir MPC'nin tarihsel olarak EN
    KOTU geometrisidir -- kerteriz-rota acisi >=45 deg olan 11 segmentin
    11'i iska (bkz. mpc_memory.md). q_alan/ufuk duzeltmeleri bunu offline'da
    cozdu ama SIM'de dogrulanmadi. Bu yuzden VARSAYILAN degil, secenek.
    """

    def __init__(self, atk_speed_mps, terminal_m, overshoot_m,
                 terminal_lead_s=0.3):
        self.atk_speed = float(atk_speed_mps)
        self.terminal_m = float(terminal_m)
        self.overshoot_m = float(overshoot_m)
        self.terminal_lead_s = float(terminal_lead_s)
        self.faz = "INT"

    def update(self, target_state, pursuer_pos):
        target_pos = np.asarray(target_state[0:3], dtype=float).reshape(3)
        target_vel = np.asarray(target_state[3:6], dtype=float).reshape(3)
        pursuer_pos = np.asarray(pursuer_pos, dtype=float).reshape(3)

        rel = target_pos - pursuer_pos
        rng = float(np.linalg.norm(rel))

        if rng <= self.terminal_m:
            # Yakinda kesisme cozumu kotu kosullanir: kisa sabit onculuk.
            ip = target_pos + target_vel * self.terminal_lead_s
            self.faz = "TERM"
        else:
            t_int = kesisme_zamani(rel, target_vel, self.atk_speed)
            if t_int is None:
                ip = target_pos.copy()       # cozum yok -> saf takip
                self.faz = "PURSUE"
            else:
                ip = target_pos + target_vel * t_int
                self.faz = "INT"

        # Nisan noktasini kesismenin OTESINE uzat: boylece avci kesisme
        # aninda FRENLEMEZ, tam hizla icinden gecer (formation_KILLER
        # ATTACK_OVERSHOOT_M ile ayni gerekce).
        vek = ip - pursuer_pos
        n = float(np.linalg.norm(vek))
        slot_pos = ip + (vek / n) * self.overshoot_m if n > 1e-3 else ip

        # slot_vel: hedefin hizi (ileri besleme) -- BehindSlot ile ayni
        # sozlesme, asagi akis (z_slew, aim_limiter) degismeden calisir.
        slot_vel = target_vel.copy()
        return slot_pos, slot_vel, target_pos


class ZCommandSlewLimiter:
    """Smooth outgoing Z setpoints: always-on slew, turn windows, outliers.

    The always-on path is the pos+vel yaw-spike guard: any Z command step --
    the ~30 m initial catch-up offset, the 4-5 Hz packet staircase while the
    target genuinely climbs, mode-mix shoves, extension/latch transitions --
    becomes a bounded ramp the climb controller can track without spiking
    collective (motor saturation sheds yaw authority first; 2026-07-15 logs).
    """

    def __init__(
        self,
        slew_rate_mps,
        jump_m,
        active_s,
        dmu_threshold,
        mu_threshold,
        outlier_slew_rate_mps,
        outlier_jump_m,
        always_slew_rate_mps=0.0,
    ):
        self.slew_rate_mps = max(0.0, float(slew_rate_mps))
        self.jump_m = max(0.0, float(jump_m))
        self.active_s = max(0.0, float(active_s))
        self.dmu_threshold = max(0.0, float(dmu_threshold))
        self.mu_threshold = float(mu_threshold)
        self.outlier_slew_rate_mps = max(0.0, float(outlier_slew_rate_mps))
        self.outlier_jump_m = max(0.0, float(outlier_jump_m))
        self.always_slew_rate_mps = max(0.0, float(always_slew_rate_mps))
        self.prev_mu_ct = None
        self.active_until = 0.0
        self.active_reason = ""
        self.prev_cmd_z = None

    @property
    def enabled(self):
        return self.slew_rate_mps > 0.0 and self.active_s > 0.0

    @property
    def outlier_enabled(self):
        return self.outlier_slew_rate_mps > 0.0

    @property
    def always_enabled(self):
        return self.always_slew_rate_mps > 0.0

    def open_window(self, now, reason):
        if self.enabled:
            self.active_until = max(self.active_until, now + self.active_s)
            self.active_reason = reason

    def update_mode_probability(self, mu_ct, now):
        mu_ct = float(mu_ct)
        switched = False

        if self.prev_mu_ct is not None:
            crossed_threshold = (self.prev_mu_ct < self.mu_threshold <= mu_ct) or (
                self.prev_mu_ct >= self.mu_threshold > mu_ct
            )
            jumped = abs(mu_ct - self.prev_mu_ct) >= self.dmu_threshold
            switched = crossed_threshold or jumped

            if self.enabled and switched:
                self.open_window(now, "ct_switch")

        self.prev_mu_ct = mu_ct
        return switched

    def limit(self, slot_pos, slot_vel, now, dt, seed_z=None):
        pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
        vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
        raw_z = float(pos[2])

        if self.prev_cmd_z is None:
            if seed_z is None:
                # No seed: the first command passes through unlimited.
                self.prev_cmd_z = raw_z
                return pos, vel, False, 0.0, ""
            # Seed from the pursuer's own altitude so the very first command
            # already ramps from where the vehicle IS. Commanding the initial
            # ~30 m z offset as a step kept the climb controller saturated for
            # the whole catch-up sprint (yaw shed for ~20 s; 2026-07-15 log).
            self.prev_cmd_z = float(seed_z)

        z_delta = raw_z - self.prev_cmd_z
        dt_s = max(float(dt), 1e-3)
        active = self.enabled and now <= self.active_until
        switch_limit = active and abs(z_delta) > self.jump_m
        outlier_limit = self.outlier_enabled and abs(z_delta) > self.outlier_jump_m
        always_limit = (
            self.always_enabled and abs(z_delta) > self.always_slew_rate_mps * dt_s
        )
        should_limit = switch_limit or outlier_limit or always_limit
        reason = ""

        if should_limit:
            rates = []
            reasons = []
            if switch_limit:
                rates.append(self.slew_rate_mps)
                reasons.append(self.active_reason or "turn_window")
            if outlier_limit:
                rates.append(self.outlier_slew_rate_mps)
                reasons.append("z_outlier")
            if always_limit:
                rates.append(self.always_slew_rate_mps)
                reasons.append("z_always")
            rate = min(rates)
            reason = "+".join(reasons)

            max_step = rate * dt_s
            limited_delta = float(np.clip(z_delta, -max_step, max_step))
            pos[2] = self.prev_cmd_z + limited_delta
            vel[2] = float(np.clip(vel[2], -rate, rate))

        limited_by = raw_z - float(pos[2])
        self.prev_cmd_z = float(pos[2])
        return pos, vel, should_limit, limited_by, reason


def send_local_position_target(
    master,
    boot_time,
    position,
    velocity,
    use_velocity,
    yaw=0.0,
    use_yaw=False,
    acceleration=None,
):
    ax, ay, az = 0.0, 0.0, 0.0
    if use_velocity:
        vx, vy, vz = velocity
        if acceleration is not None:
            # Acceleration feedforward only makes sense alongside velocity.
            mask = POSITION_VELOCITY_ACCEL_MASK
            ax, ay, az = (float(a) for a in acceleration)
        else:
            mask = POSITION_VELOCITY_MASK
    else:
        mask = POSITION_ONLY_MASK
        vx, vy, vz = 0.0, 0.0, 0.0

    if use_yaw:
        mask &= ~mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

    master.mav.set_position_target_local_ned_send(
        int((time.monotonic() - boot_time) * 1000.0) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(vx),
        float(vy),
        float(vz),
        float(ax),
        float(ay),
        float(az),
        float(yaw) if use_yaw else 0.0,
        0.0,
    )


class TerminalPositionLatch:
    def __init__(self, latch_tgo_s, release_range_m, arm_margin_m=5.0):
        self.latch_tgo_s = max(0.0, float(latch_tgo_s))
        self.release_range_m = max(0.0, float(release_range_m))
        # The latch may only arm inside the release range (with margin), or a
        # latch fired at range > release unlatches on the very next loop and
        # fires the miss-recovery BRAKE. t_go alone is not a safe trigger: a
        # relative-velocity spike (e.g. stale pursuer telemetry catching up)
        # makes tCPA dip below the threshold at 100+ m estimated range. Range
        # here is pursuer-to-estimate (imm.x), never a true target feed.
        self.arm_range_m = self.release_range_m - max(0.0, float(arm_margin_m))
        self.active = False
        self.position = None

    def reset(self):
        """Drop any frozen aim point.

        The latch is only serviced while the FSM is in CHASE, so once recovery
        takes over (CPA/abort/miss) a still-active latch would hand the next
        CHASE entry a stale frozen point from the previous pass. Recovery calls
        this on leaving CHASE so every engagement starts from a clean latch.
        """
        self.active = False
        self.position = None

    def update(self, slot_pos, t_go_s, range_m, allow_arm=True):
        if self.active and range_m >= self.release_range_m:
            self.active = False
            self.position = None

        if self.active and self.position is not None:
            return np.asarray(self.position, dtype=float).reshape(3).copy(), True, False

        # allow_arm gates ARMING only (terminal-entry geometry gate): an active
        # window above still holds/releases normally regardless.
        if (
            allow_arm
            and self.latch_tgo_s > 0.0
            and t_go_s <= self.latch_tgo_s
            and range_m <= self.arm_range_m
        ):
            self.position = np.asarray(slot_pos, dtype=float).reshape(3).copy()
            self.active = True
            return self.position.copy(), True, True

        return np.asarray(slot_pos, dtype=float).reshape(3).copy(), False, False


class AimPointRateLimiter:
    """Hard cap on how fast the outgoing position setpoint may move.

    A final safety governor, not a shaper: legitimate aim motion is bounded by
    target speed plus terminal geometry changes (~35 m/s at these speeds).
    Prediction transients moved the commanded point 100+ m between consecutive
    loops on 2026-07-11; the vehicle chasing those steps saturated its motors
    and shed yaw authority. dt is clamped so a post-stall loop does not
    inherit a huge displacement allowance.
    """

    def __init__(self, max_speed_mps, min_dt_s=0.02, max_dt_s=0.5):
        self.max_speed = float(max_speed_mps)
        self.min_dt = float(min_dt_s)
        self.max_dt = float(max_dt_s)
        self.prev = None
        self.prev_t = None

    def limit(self, pos, now):
        pos = np.asarray(pos, dtype=float).reshape(3).copy()
        if self.max_speed <= 0.0:
            return pos, False
        if self.prev is None:
            self.prev = pos.copy()
            self.prev_t = float(now)
            return pos, False
        dt = min(max(float(now) - self.prev_t, self.min_dt), self.max_dt)
        allowance = self.max_speed * dt
        delta = pos - self.prev
        dist = float(np.linalg.norm(delta))
        limited = dist > allowance
        if limited:
            pos = self.prev + delta * (allowance / dist)
        self.prev = pos.copy()
        self.prev_t = float(now)
        return pos, limited


def clamp_command_altitude(pos, min_altitude_m):
    """Floor the commanded altitude (local NED: altitude = -z, home-relative).

    Whatever upstream produces, the vehicle must never see a setpoint near or
    below the ground (the 2026-07-11 flight commanded underground five times).
    """
    pos = np.asarray(pos, dtype=float).reshape(3).copy()
    z_max = -float(min_altitude_m)
    floored = pos[2] > z_max
    if floored:
        pos[2] = z_max
    return pos, floored


def estimate_intercept_geometry(target_pos, pursuer_pos, target_vel, pursuer_vel):
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    pursuer_pos = np.asarray(pursuer_pos, dtype=float).reshape(3)
    target_vel = np.asarray(target_vel, dtype=float).reshape(3)
    pursuer_vel = np.asarray(pursuer_vel, dtype=float).reshape(3)

    rel_pos = target_pos - pursuer_pos
    range_m = float(np.linalg.norm(rel_pos))
    if range_m <= 1e-6:
        return {
            "range_m": 0.0,
            "los_hat": np.array([1.0, 0.0, 0.0]),
            "closing_velocity": 0.0,
            "range_rate": 0.0,
            "t_go_s": 0.0,
        }

    los_hat = rel_pos / range_m
    rel_vel = target_vel - pursuer_vel
    range_rate = float(np.dot(rel_vel, los_hat))
    closing_velocity = max(0.0, -range_rate)

    # Time-to-go is the time to the closest point of approach (CPA) under a
    # constant relative-velocity model:  t_cpa = -(rel_pos . rel_vel) / |rel_vel|^2.
    # This is well conditioned. The previous range/closing_velocity form blew up
    # (hundreds of seconds, or +inf) whenever the LOS-projected closing rate
    # passed through zero -- which happens on every oblique/crossing pass even
    # when the relative speed is large -- so the terminal latch never fired on
    # exactly the geometry it exists for. CPA-in-the-past (opening) is reported
    # as +inf so the horizon/latch consumers still read it as "not closing".
    rel_speed_sq = float(np.dot(rel_vel, rel_vel))
    if rel_speed_sq <= 1e-6:
        t_go_s = float("inf")
    else:
        t_cpa = -float(np.dot(rel_pos, rel_vel)) / rel_speed_sq
        t_go_s = t_cpa if t_cpa >= 0.0 else float("inf")

    return {
        "range_m": range_m,
        "los_hat": los_hat,
        "closing_velocity": closing_velocity,
        "range_rate": range_rate,
        "t_go_s": t_go_s,
    }


def compute_guidance_prediction_horizon(intercept, prev_mu_ct_xy, mode_probs):
    t_go_s = float(intercept["t_go_s"])
    if not np.isfinite(t_go_s):
        horizon_s = float(getattr(cfg, "TERMINAL_PREDICTION_MAX_S", 6.0))
    else:
        horizon_s = max(0.0, t_go_s)

    horizon_s = min(horizon_s, float(getattr(cfg, "TERMINAL_PREDICTION_MAX_S", 6.0)))

    ct_mu = float(mode_probs.get("ct_xy", 0.0))
    prev_mu = ct_mu if prev_mu_ct_xy is None else float(prev_mu_ct_xy)
    dmu = ct_mu - prev_mu
    if (
        dmu >= float(getattr(cfg, "TERMINAL_TURN_ENTRY_DMU_MIN", 0.02))
        and ct_mu <= float(getattr(cfg, "TERMINAL_TURN_ENTRY_CT_MU_MAX", 0.55))
    ):
        horizon_s = min(
            horizon_s,
            float(getattr(cfg, "TERMINAL_TURN_ENTRY_HORIZON_CAP_S", 1.25)),
        )

    return horizon_s


def compute_terminal_extension_distance(intercept, current_speed_mps):
    min_extend_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_DISTANCE_M", 18.0))
    accel_cmss = float(getattr(cfg, "GUIDED_STARTUP_PARAM_ASSERTS", {}).get("WPNAV_ACCEL", 600))
    accel_mps2 = max(0.1, accel_cmss / 100.0)
    brake_margin_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_BRAKE_MARGIN_M", 10.0))
    brake_distance_m = (max(0.0, float(current_speed_mps)) ** 2) / (2.0 * accel_mps2)
    extend_m = max(min_extend_m, brake_distance_m + brake_margin_m)
    # Cap how far past the target the aim point is placed. The full brake
    # distance (~43 m at 20 m/s) commits the pursuer to a large overshoot on a
    # miss, which is exactly what wrecks the post-pass recovery. Capping trades a
    # little terminal deceleration for a far shorter overshoot.
    extend_max_m = float(getattr(cfg, "TERMINAL_POSITION_EXTEND_MAX_M", 25.0))
    return min(extend_m, extend_max_m)


def extend_position_target_past_target(
    slot_pos,
    target_pos,
    los_hat,
    range_m,
    extend_range_m,
    extend_distance_m,
    blend_band_m=0.0,
):
    pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
    if range_m > float(extend_range_m) or float(extend_distance_m) <= 0.0:
        return pos, False
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    spear = target_pos + float(extend_distance_m) * np.asarray(
        los_hat, dtype=float
    ).reshape(3)
    band = max(0.0, float(blend_band_m))
    if band > 1e-6:
        # Blend from the untouched slot at the activation boundary to the full
        # spear point over `band` meters of closure, so activation is
        # continuous instead of an instant ~30 m aim step (the step spikes
        # tilt+collective at speed; saturation sheds yaw first). Also removes
        # on/off toggling when the range dithers across the boundary.
        s = min(1.0, (float(extend_range_m) - float(range_m)) / band)
        pos = pos + s * (spear - pos)
    else:
        pos = spear
    return pos, True


class ImpactDetector:
    """Classify terminal passes as HIT / CRASH, for the GUI hit counter and
    crash screen.

    Preferred signal is airframe VIBRATION: a spike is a physical impact -- a
    HIT if it lands within hit_range_m of the target, a CRASH otherwise (e.g. a
    ground strike). Without VIBRATION it falls back to kinematics: a
    sub-hit_range_m pass whose pursuer speed suddenly collapses is a hit, and a
    tumbling/inverted attitude (tilt >= crash_tilt_deg) is a crash regardless of
    vibe. Hits are debounced to one per close pass; a crash latches (terminal).
    """

    def __init__(
        self,
        hit_range_m=1.0,
        impact_vibe=60.0,
        require_speed_drop=True,
        speed_drop_mps=8.0,
        speed_window_s=0.5,
        crash_vibe=60.0,
        crash_tilt_deg=120.0,
    ):
        self.hit_range_m = float(hit_range_m)
        self.impact_vibe = float(impact_vibe)
        self.require_speed_drop = bool(require_speed_drop)
        self.speed_drop_mps = float(speed_drop_mps)
        self.speed_window_s = max(1e-3, float(speed_window_s))
        self.crash_vibe = float(crash_vibe)
        self.crash_tilt_deg = float(crash_tilt_deg)
        self.hits = 0
        self.crashed = False
        self.last_event = ""       # "HIT" / "CRASH" / ""
        self.last_hit_range = None
        self.vibe_max = 0.0
        self._speed_hist = deque()  # (t, speed) over the drop window
        self._pass_open = False     # currently inside a < hit_range_m pass
        self._pass_counted = False  # already scored this pass

    def update(self, now, range_m, pursuer_speed, tilt_deg=None, vibe=None):
        now = float(now)
        # Speed history for the sudden-drop test.
        self._speed_hist.append((now, float(pursuer_speed)))
        cutoff = now - self.speed_window_s
        while self._speed_hist and self._speed_hist[0][0] < cutoff:
            self._speed_hist.popleft()
        speed_peak = max((s for _, s in self._speed_hist), default=float(pursuer_speed))
        speed_drop = speed_peak - float(pursuer_speed)

        vmax = None
        if vibe is not None:
            vmax = max(abs(float(vibe[0])), abs(float(vibe[1])), abs(float(vibe[2])))
            self.vibe_max = vmax

        rng_finite = range_m is not None and np.isfinite(range_m)
        near = rng_finite and float(range_m) < self.hit_range_m

        # One hit per close pass: open on entry, close once range clearly opens.
        if near:
            if not self._pass_open:
                self._pass_open = True
                self._pass_counted = False
        elif self._pass_open and (
            not rng_finite or float(range_m) > self.hit_range_m * 2.0
        ):
            self._pass_open = False
            self._pass_counted = False

        event = ""

        # CRASH (latches). Impact-level vibration away from the target, or a
        # tumbling/inverted attitude (a strong, vibe-independent crash signal).
        if not self.crashed:
            crash = False
            if vmax is not None and vmax >= self.crash_vibe and not near:
                crash = True
            if tilt_deg is not None and float(tilt_deg) >= self.crash_tilt_deg:
                crash = True
            if crash:
                self.crashed = True
                self.last_event = event = "CRASH"

        # HIT (debounced one per pass; never scored after a crash).
        if near and not self._pass_counted and not self.crashed:
            if vmax is not None:
                hit = vmax >= self.impact_vibe
            else:
                # No vibe telemetry: a close pass plus a sudden speed collapse.
                hit = (not self.require_speed_drop) or speed_drop >= self.speed_drop_mps
            if hit:
                self.hits += 1
                self._pass_counted = True
                self.last_hit_range = float(range_m)
                self.last_event = event = "HIT"

        return {
            "hits": self.hits,
            "crashed": self.crashed,
            "event": event,
            "vibe_max": self.vibe_max,
        }


def measurement_is_plausible(
    meas, prev_meas=None, dt_s=0.0, max_implied_speed_mps=0.0, max_altitude_m=0.0
):
    """Sanity-check one target measurement BEFORE it reaches the estimator.

    A corrupt position is absorbed as a real measurement and poisons the filter
    (and every setpoint derived from it) long after the bad sample is gone, so
    it must be rejected at the door. The implied-speed gate is not theoretical:
    Shaykh's real 2026-07-30 feed carried glitches implying up to 1350 m/s.

    Returns (ok, reason). Limits of 0 disable the corresponding check.
    """
    meas = np.asarray(meas, dtype=float).reshape(3)
    if not np.all(np.isfinite(meas)):
        return False, "non-finite"
    if float(max_altitude_m) > 0.0 and abs(float(meas[2])) > float(max_altitude_m):
        return False, f"altitude {abs(float(meas[2])):.0f}m"
    if (
        float(max_implied_speed_mps) > 0.0
        and prev_meas is not None
        and float(dt_s) > 1e-3
    ):
        implied = float(
            np.linalg.norm(meas - np.asarray(prev_meas, dtype=float).reshape(3))
        ) / float(dt_s)
        if implied > float(max_implied_speed_mps):
            return False, f"jump {implied:.0f}m/s"
    return True, ""


class MissionFailsafe:
    """Mission-level safety supervisor: stop steering and hand the vehicle back
    to its mission when the data guidance depends on goes stale or corrupt.

    Flying a copter on stale or poisoned setpoints is the worst outcome
    available -- worse than not guiding at all -- so when the inputs are not
    trustworthy this stops commanding and falls back to the mission mode
    (default AUTO). Mirrors the project's own pattern
    (GOAT_guidance.crash_manuever -> set_mode('AUTO'), and its documented
    ">=2 s without a message -> AUTO due to inactivity" watchdog). This class
    only decides; the caller applies the action (switch the flight mode over
    MAVLink, or withhold setpoints and publish a mode request over Redis).

    Hysteretic, with TWO fault classes whose timings differ because the faults
    differ (see MISSION_FAILSAFE_* in the config):

      "data"  -- no fresh target measurement. A camera tracker dropping a few
                 frames is normal and self-heals, so this trips almost at once
                 (target_grace_s) and clears quickly (data_clear_s): stop
                 commanding for a fraction of a second, resume the moment
                 tracking returns. Withholding briefly is nearly free.
      "hard"  -- estimator diverged, setpoint sending failing, crash detected.
                 These never self-heal, so they use the slower trip_after_s /
                 clear_after_s and bias toward staying safe.

    A hard fault always outranks a data gap: it cannot be cleared by the fast
    data-recovery path.
    """

    def __init__(
        self,
        enabled=True,
        mode="AUTO",
        trip_after_s=1.0,
        clear_after_s=2.0,
        data_clear_s=0.3,
    ):
        self.enabled = bool(enabled)
        self.mode = str(mode)
        self.trip_after_s = max(0.0, float(trip_after_s))
        self.clear_after_s = max(0.0, float(clear_after_s))
        self.data_clear_s = max(0.0, float(data_clear_s))
        self.tripped = False
        self.reason = ""
        self.fault_class = ""   # "data" | "hard" while tripped
        self._bad_since = None
        self._good_since = None

    def update(self, now, healthy, reason="", fault_class="hard", immediate=False):
        """Advance the supervisor. Returns True when the state CHANGED.

        healthy is the caller's per-loop verdict; reason describes the fault
        and fault_class selects the timing profile ("data" or "hard").

        immediate=True trips without waiting out trip_after_s -- for a fault
        already debounced by its own timer (a data gap has its own
        target_grace_s; a feed past --target-timeout has been dead for 2 s).
        """
        now = float(now)
        changed = False
        if not self.enabled:
            return False

        if healthy:
            self._bad_since = None
            if self._good_since is None:
                self._good_since = now
            # A brief data gap recovers fast; a hard fault must prove itself
            # healthy for much longer before we command again.
            dwell = self.data_clear_s if self.fault_class == "data" else self.clear_after_s
            if self.tripped and (now - self._good_since) >= dwell:
                self.tripped = False
                self.reason = ""
                self.fault_class = ""
                changed = True
        else:
            self._good_since = None
            if self._bad_since is None:
                self._bad_since = now
            elapsed = now - self._bad_since
            if not self.tripped and (immediate or elapsed >= self.trip_after_s):
                self.tripped = True
                self.reason = str(reason)
                self.fault_class = str(fault_class)
                changed = True
            elif self.tripped:
                # Keep the reported cause current, and let a hard fault
                # promote a data gap (never the reverse) so the slow clear
                # dwell applies once something serious is wrong.
                if reason and reason != self.reason:
                    self.reason = str(reason)
                if fault_class == "hard":
                    self.fault_class = "hard"
        return changed


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simple IMM-based GUIDED follow script"
    )
    parser.add_argument("--pursuer", default=cfg.PURSUER_CONN_STR)
    parser.add_argument("--target", default=cfg.TARGET_CONN_STR)
    parser.add_argument(
        "--loop-hz", type=float, default=min(float(getattr(cfg, "LOOP_HZ", 20)), 20.0)
    )
    parser.add_argument(
        "--back", type=float, default=None,
        help="Slot distance behind the target [m]. Default depends on kill "
        "mode: LAG_PURSUIT_DIST (on the target) with kill mode ON, "
        "FOLLOW_STANDOFF_BACK_M with it OFF",
    )
    parser.add_argument("--side", type=float, default=0.0)
    parser.add_argument(
        "--down", type=float, default=None,
        help="Slot distance below the target [m]. Default depends on kill "
        "mode: SLOT_DOWN_OFFSET_M (approach from underneath) with kill mode "
        "ON, FOLLOW_STANDOFF_DOWN_M with it OFF",
    )
    #parser.add_argument("--predict", type=float, default=0.25)
    parser.add_argument(
        "--target-message-rate-hz",
        type=float,
        default=float(getattr(cfg, "TARGET_MESSAGE_RATE_HZ", 15.0)),
        help="Requested GLOBAL_POSITION_INT rate for the target [Hz]",
    )
    parser.add_argument("--target-timeout", type=float, default=2.0)
    parser.add_argument("--min-heading-speed", type=float, default=1.0)
    parser.add_argument(
        "--max-feedforward-speed",
        type=float,
        default=float(getattr(cfg, "SPEED_MAX", 25.0)),
    )
    parser.add_argument(
        "--terminal-extend-range-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_POSITION_EXTEND_RANGE_M", 40.0)),
        help="Inside this range, push the position target past the target along LOS [m]",
    )
    parser.add_argument(
        "--terminal-extend-distance-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_POSITION_EXTEND_DISTANCE_M", 18.0)),
        help="Distance to place the position target beyond the target along LOS [m]",
    )
    parser.add_argument(
        "--terminal-extend-blend-band-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_EXTEND_BLEND_BAND_M", 10.0)),
        help="Blend the terminal extension in over this range band below the "
        "activation range [m]; 0 = legacy instant step",
    )
    parser.add_argument(
        "--terminal-latch-tgo-s",
        type=float,
        default=float(getattr(cfg, "TERMINAL_LATCH_TGO_S", 0.7)),
        help="Freeze & spear once estimated time-to-go falls below this [s]",
    )
    parser.add_argument(
        "--terminal-latch-release-range-m",
        type=float,
        default=float(getattr(cfg, "TERMINAL_LATCH_RELEASE_RANGE_M", 40.0)),
        help="Release the frozen terminal aim point after a miss once range opens back up [m]",
    )
    parser.add_argument(
        "--z-switch-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_SLEW_RATE", 0.0)),
        help="Enable turn-window Z command slew rate limit [m/s]; 0 disables",
    )
    parser.add_argument(
        "--z-switch-jump",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_JUMP_M", 0.6)),
        help="Minimum outgoing Z jump [m] before turn-window slew limiting is applied",
    )
    parser.add_argument(
        "--z-switch-window",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_WINDOW_S", 1.2)),
        help="Seconds after CT switch or fast turn onset where Z slew limiting may apply",
    )
    parser.add_argument(
        "--z-switch-dmu",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_DMU", 0.08)),
        help="CT probability change that opens the turn-window Z slew",
    )
    parser.add_argument(
        "--z-switch-mu-threshold",
        type=float,
        default=float(getattr(cfg, "Z_SWITCH_MU_THRESHOLD", 0.20)),
        help="CT probability crossing threshold that opens the turn-window Z slew",
    )
    parser.add_argument(
        "--z-outlier-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_OUTLIER_SLEW_RATE", 0.0)),
        help="Always-on Z command outlier slew rate [m/s]; 0 disables",
    )
    parser.add_argument(
        "--z-outlier-jump",
        type=float,
        default=float(getattr(cfg, "Z_OUTLIER_JUMP_M", 0.9)),
        help="Outgoing Z jump [m] that triggers always-on outlier slew limiting",
    )
    parser.add_argument(
        "--z-always-slew-rate",
        type=float,
        default=float(getattr(cfg, "Z_ALWAYS_SLEW_RATE", 0.0)),
        help="Always-on outgoing Z command slew rate [m/s]; 0 disables. Primary "
        "pos+vel yaw-spike guard.",
    )
    parser.add_argument(
        "--z-update-freeze-packets",
        type=int,
        default=int(getattr(cfg, "Z_UPDATE_FREEZE_PACKETS", 0)),
        help="Freeze estimator Z correction for this many packets after fast turn onset; 0 disables",
    )
    parser.add_argument(
        "--z-ct-freeze-packets",
        type=int,
        default=int(getattr(cfg, "Z_CT_FREEZE_PACKETS", 0)),
        help="Freeze estimator Z correction for this many packets starting when mu_ct_xy crosses the CT activation threshold upward; 0 disables",
    )
    parser.add_argument(
        "--z-ct-freeze-mu-threshold",
        type=float,
        default=float(getattr(cfg, "Z_CT_FREEZE_MU_THRESHOLD", 0.20)),
        help="Aggregate mu_ct_xy upward crossing that arms the CT-activation Z freeze",
    )
    pos_group = parser.add_mutually_exclusive_group()
    pos_group.add_argument(
        "--position-only",
        dest="position_only",
        action="store_true",
        help="Send position-only targets (no velocity/acceleration feedforward)",
    )
    pos_group.add_argument(
        "--no-position-only",
        dest="position_only",
        action="store_false",
        help="Send velocity feedforward too (required for --accel-feedforward)",
    )
    parser.add_argument(
        "--no-guided",
        action="store_true",
        help="Do not switch the pursuer to GUIDED on startup",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable the per-loop guidance CSV log (logs/guided_follow_*.csv)",
    )

    recovery_default = bool(getattr(cfg, "MISS_RECOVERY_ENABLED", True))
    recovery_group = parser.add_mutually_exclusive_group()
    recovery_group.add_argument(
        "--miss-recovery",
        dest="miss_recovery",
        action="store_true",
        help="After a terminal miss, stabilize (recovery flight mode) then "
        "re-engage before resuming chase (needs script mode control)",
    )
    recovery_group.add_argument(
        "--no-miss-recovery",
        dest="miss_recovery",
        action="store_false",
        help="Disable miss recovery; re-chase immediately after a pass",
    )
    parser.add_argument(
        "--recovery-mode",
        default=str(getattr(cfg, "RECOVERY_MODE", "BRAKE")),
        help="Flight mode used during the recovery HOLD window (e.g. BRAKE, "
        "STABILIZE, LOITER)",
    )
    parser.add_argument(
        "--recovery-hold-s",
        type=float,
        default=float(getattr(cfg, "RECOVERY_HOLD_S", 1.2)),
        help="Seconds to hold in the recovery flight mode before re-engaging",
    )
    parser.add_argument(
        "--reengage-ramp-s",
        type=float,
        default=float(getattr(cfg, "REENGAGE_RAMP_TIME_S", 1.5)),
        help="Seconds to ramp commanded speed from SAFE_TURN_SPEED to full "
        "during re-engagement",
    )
    parser.add_argument(
        "--reengage-max-s",
        type=float,
        default=float(getattr(cfg, "REENGAGE_MAX_S", 4.0)),
        help="Timeout [s] to force re-engagement back to full chase",
    )

    # --- YAKLASIM YASASI (goruntuluye devirden ONCEKI faz) ---
    # 'slot'    : hedefin ARKASINDA duran standoff slotu (VARSAYILAN, kanitli)
    # 'kesisme' : hedefin GIDECEGI yere nisan alan carpisma rotasi
    #             (formation_KILLER ATTACK yasasi; bkz. InterceptGuidance)
    parser.add_argument(
        "--yaklasim", choices=("slot", "kesisme"), default="slot",
        help="devir oncesi yaklasim yasasi: slot (arkadan standoff, "
             "varsayilan) | kesisme (carpisma rotasi, formation_KILLER)")
    parser.add_argument("--kesisme-hiz", type=float, default=24.0,
                        help="kesisme yaklasiminda komutlanan hiz [m/s] "
                             "(formation_KILLER ATTACK_SPEED_MPS=24)")
    parser.add_argument("--kesisme-terminal", type=float, default=45.0,
                        help="bu menzilin altinda kisa sabit onculuge gec [m] "
                             "(ATTACK_TERMINAL_M=45)")
    parser.add_argument("--kesisme-overshoot", type=float, default=50.0,
                        help="nisan noktasini kesismenin otesine uzat [m] "
                             "(ATTACK_OVERSHOOT_M=50)")

    yaw_default = bool(getattr(cfg, "YAW_LOCK_ENABLED", False))
    yaw_group = parser.add_mutually_exclusive_group()
    yaw_group.add_argument(
        "--yaw-lock",
        dest="yaw_lock",
        action="store_true",
        help="Enable yaw lock to line-of-sight",
    )
    yaw_group.add_argument(
        "--no-yaw-lock",
        dest="yaw_lock",
        action="store_false",
        help="Disable yaw lock to line-of-sight",
    )

    accel_ff_default = bool(getattr(cfg, "ACCEL_FEEDFORWARD_ENABLED", False))
    accel_group = parser.add_mutually_exclusive_group()
    accel_group.add_argument(
        "--accel-feedforward",
        dest="accel_feedforward",
        action="store_true",
        help="Add target acceleration feedforward to the setpoint stream "
        "(requires velocity setpoints, i.e. --no-position-only)",
    )
    accel_group.add_argument(
        "--no-accel-feedforward",
        dest="accel_feedforward",
        action="store_false",
        help="Disable target acceleration feedforward",
    )
    parser.add_argument(
        "--accel-ff-max",
        type=float,
        default=float(getattr(cfg, "ACCEL_FEEDFORWARD_MAX_MPS2", 8.0)),
        help="Clamp on the feedforward acceleration magnitude [m/s^2]",
    )
    parser.add_argument(
        "--accel-ff-fade-band",
        type=float,
        default=float(getattr(cfg, "ACCEL_FEEDFORWARD_FADE_BAND_M", 20.0)),
        help="Range band [m] above the terminal-extension range over which accel "
        "feedforward fades from full to zero (prevents terminal overshoot)",
    )

    vvel_default = bool(getattr(cfg, "VELOCITY_FF_VERTICAL_ENABLED", False))
    vvel_group = parser.add_mutually_exclusive_group()
    vvel_group.add_argument(
        "--vertical-velocity-ff",
        dest="vertical_velocity_ff",
        action="store_true",
        help="Send the estimated target vertical velocity vz in the velocity "
        "setpoint (can excite yaw instability on Z transients during turns)",
    )
    vvel_group.add_argument(
        "--no-vertical-velocity-ff",
        dest="vertical_velocity_ff",
        action="store_false",
        help="Strip vz from the velocity setpoint; the position setpoint carries "
        "Z instead (default; avoids throttle-saturation yaw instability)",
    )
    reengage_lpf_default = bool(getattr(cfg, "REENGAGE_LPF_ENABLED", True))
    reengage_lpf_group = parser.add_mutually_exclusive_group()
    reengage_lpf_group.add_argument(
        "--reengage-lpf",
        dest="reengage_lpf",
        action="store_true",
        help="Low-pass the REENGAGE position+velocity setpoint (seeded from the "
        "vehicle's own state) so it eases back in after a miss instead of "
        "stepping straight to the target (default)",
    )
    reengage_lpf_group.add_argument(
        "--no-reengage-lpf",
        dest="reengage_lpf",
        action="store_false",
        help="Send the raw REENGAGE setpoint with no command smoothing",
    )
    kill_mode_default = bool(getattr(cfg, "KILL_MODE_ENABLED", True))
    kill_group = parser.add_mutually_exclusive_group()
    kill_group.add_argument(
        "--kill-mode",
        dest="kill_mode",
        action="store_true",
        help="INTERCEPT: run the full terminal chain (LOS spear, terminal "
        "window, fly-through climb-from-below) with the slot ON the target "
        "(default)",
    )
    kill_group.add_argument(
        "--no-kill-mode",
        dest="kill_mode",
        action="store_false",
        help="SIMPLE FOLLOW: disable the entire terminal intercept and hold a "
        "standoff behind/below the target instead. For real-world tracking "
        "tests -- the vehicle is never aimed at the aircraft. All protective "
        "limiters, the altitude abort and the mission failsafe stay active",
    )
    # YILDIZLAR ORTAMI EKLEMESI: estimator penceresi artik OPT-IN.
    # Gerekce: pencere ayni surecte bir PyQt5/matplotlib is parcacigi acar ve
    # 10 Hz'lik her cizim GIL'i uzun sure tutar; ana 20 Hz dongusu GIL
    # bekleyerek ~5 Hz'e duser (olculdu: GUI acik 4.9 Hz / kapali 19.3 Hz,
    # makine her iki kosuda da %52 bos). Pencere sadece --gui ya da
    # YILDIZ_GUI=1 ile acilir.
    parser.add_argument(
        "--gui",
        dest="gui",
        action="store_true",
        default=None,
        help="Open the in-process estimator/parameter window. OFF by default: "
        "it runs a PyQt5+matplotlib thread in this process and its redraws hold "
        "the GIL, collapsing the 20 Hz guidance loop to ~5 Hz. Only use it for "
        "offline inspection, never while flying (env: YILDIZ_GUI=1)",
    )
    carrot_default = bool(getattr(cfg, "CARROT_CLAMP_ENABLED", True))
    carrot_group = parser.add_mutually_exclusive_group()
    carrot_group.add_argument(
        "--carrot-clamp",
        dest="carrot_clamp",
        action="store_true",
        help="Cap the commanded position to CARROT_MAX_AHEAD_M ahead of the "
        "vehicle (closure governor: bounds ArduCopter's internal position-error "
        "sprint to ~P*D_max over the velocity FF; never brakes -- the carrot "
        "recedes as the vehicle advances). pos+vel only (default)",
    )
    carrot_group.add_argument(
        "--no-carrot-clamp",
        dest="carrot_clamp",
        action="store_false",
        help="Send the raw commanded position however far ahead it sits "
        "(unbounded internal catch-up sprint)",
    )
    super_safe_turn_default = bool(getattr(cfg, "SUPER_SAFE_TURN_ENABLED", False))
    super_safe_turn_group = parser.add_mutually_exclusive_group()
    super_safe_turn_group.add_argument(
        "--super-safe-turn",
        dest="super_safe_turn",
        action="store_true",
        help="Pre-emptively slow the pursuer when the estimator sees a coordinated "
        "turn (mu_ct high) so the commanded speed never exceeds v_safe = "
        "a_lat_max/omega -- the max speed at which the required turn is inside "
        "the bank envelope. Trades closure speed for the ability to actually "
        "make the turn without saturating (protects the physical airframe)",
    )
    super_safe_turn_group.add_argument(
        "--no-super-safe-turn",
        dest="super_safe_turn",
        action="store_false",
        help="Do not slow down ahead of turns (default); the turn clamp still "
        "conditions the command but closure speed is unrestricted",
    )
    parser.set_defaults(
        yaw_lock=yaw_default,
        position_only=bool(getattr(cfg, "POSITION_ONLY_DEFAULT", True)),
        accel_feedforward=accel_ff_default,
        miss_recovery=recovery_default,
        vertical_velocity_ff=vvel_default,
        reengage_lpf=reengage_lpf_default,
        carrot_clamp=carrot_default,
        super_safe_turn=super_safe_turn_default,
        kill_mode=kill_mode_default,
    )
    args = parser.parse_args()
    # Slot geometry follows the mode: an intercept closes ON the target, a
    # follow test MUST hold a standoff (flying the intercept slot with the
    # terminal chain disabled would still put the vehicle on the target).
    if args.back is None:
        args.back = float(
            getattr(cfg, "LAG_PURSUIT_DIST", 8.0) if args.kill_mode
            else getattr(cfg, "FOLLOW_STANDOFF_BACK_M", 15.0)
        )
    if args.down is None:
        args.down = float(
            getattr(cfg, "SLOT_DOWN_OFFSET_M", 0.0) if args.kill_mode
            else getattr(cfg, "FOLLOW_STANDOFF_DOWN_M", 5.0)
        )
    return args


def main():
    args = parse_args()
    running = True

    def handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    loop_dt = 1.0 / max(args.loop_hz, 1.0)

    print(f"[{redcolor}simple_follow{endcolor}] Connecting pursuer: {args.pursuer}")
    pursuer_conn = mavutil.mavlink_connection(args.pursuer)
    wait_for_vehicle_heartbeat(
        pursuer_conn,
        "pursuer",
        expected_sysid=getattr(cfg, "PURSUER_EXPECTED_SYSID", None),
        abort_check=lambda: not running,
    )
    print(
        f"[{redcolor}simple_follow{endcolor}] Pursuer heartbeat sys={pursuer_conn.target_system} "
        f"comp={pursuer_conn.target_component}"
    )

    home_lat, home_lon, home_alt = fetch_pursuer_home(pursuer_conn)
    print(
        f"[{redcolor}simple_follow{endcolor}] Origin/home lat={home_lat:.7f} "
        f"lon={home_lon:.7f} alt={home_alt:.1f}"
    )

    print(f"[{redcolor}simple_follow{endcolor}] Connecting target: {args.target}")
    target_conn = mavutil.mavlink_connection(args.target)
    wait_for_vehicle_heartbeat(
        target_conn,
        "target",
        expected_sysid=getattr(cfg, "TARGET_EXPECTED_SYSID", None),
        abort_check=lambda: not running,
    )
    print(
        f"[{redcolor}simple_follow{endcolor}] Target heartbeat sys={target_conn.target_system} "
        f"comp={target_conn.target_component}"
    )

    # Do the two aircraft agree on the elevation of the field they both launched
    # from? They set home independently from their own GPS fixes, and tens of
    # metres of disagreement is routine (36.80 m on 2026-07-31). Harmless while
    # the relative-altitude path is used, but it is the earliest visible symptom
    # of a bad fix -- and it is a silent, flight-ruining error the moment anyone
    # switches TARGET_ALT_USE_RELATIVE off. Diagnostic only: never blocks.
    use_rel_alt = bool(getattr(cfg, "TARGET_ALT_USE_RELATIVE", True))
    home_warn_m = float(getattr(cfg, "HOME_ALT_MISMATCH_WARN_M", 5.0))
    if home_warn_m > 0.0:
        target_home_alt = fetch_home_altitude(target_conn)
        if target_home_alt is None:
            print(
                f"[{redcolor}simple_follow{endcolor}] target HOME_POSITION "
                f"unavailable -- home-altitude cross-check skipped"
            )
        else:
            dh = float(home_alt) - target_home_alt
            print(
                f"[{redcolor}simple_follow{endcolor}] home altitudes: "
                f"pursuer={home_alt:.2f} m target={target_home_alt:.2f} m AMSL "
                f"(diff {dh:+.2f} m)"
            )
            if abs(dh) > home_warn_m:
                print(
                    f"[{redcolor}simple_follow{endcolor}] *** WARNING: the two "
                    f"vehicles disagree about the field elevation by {abs(dh):.1f} m. "
                    + (
                        "Harmless with relative altitude, but check the GPS fixes"
                        if use_rel_alt
                        else f"AMSL MODE IS ACTIVE -- THIS PUTS THE DRONE {abs(dh):.0f} m OFF VERTICALLY"
                    )
                    + " ***"
                )

    local_position_msg_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
    )
    global_position_int_msg_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
    )
    attitude_msg_id = int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ATTITUDE", 30))
    vibration_msg_id = int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_VIBRATION", 241))
    request_message_interval(pursuer_conn, local_position_msg_id, args.loop_hz)
    request_message_interval(pursuer_conn, attitude_msg_id, args.loop_hz)
    # Vibration is slow-changing; a few Hz is plenty for impact detection.
    request_message_interval(pursuer_conn, vibration_msg_id, 5.0)
    request_message_interval(
        target_conn, global_position_int_msg_id, args.target_message_rate_hz
    )

    startup_params = getattr(cfg, "GUIDED_STARTUP_PARAM_ASSERTS", {})
    if startup_params:
        assert_startup_params(pursuer_conn, startup_params)

    if not args.no_guided:
        print(f"[{redcolor}simple_follow{endcolor}] Switching pursuer to GUIDED")
        set_mode(pursuer_conn, "GUIDED")
        # Arm the shutdown hook only when we are the one managing modes.
        global _SHUTDOWN_CONN
        _SHUTDOWN_CONN = pursuer_conn

    # ATTITUDE rides along for its yawspeed: the recovery machine uses the yaw
    # rate to decide whether the vehicle has stopped spinning before it hands
    # control back to GUIDED. LOCAL_POSITION_NED stays the primary message.
    # VIBRATION rides along for the impact detector (hit counter / crash
    # screen): a spike is a physical impact -- a hit at the target, a crash
    # otherwise. Absent-telemetry-safe (get_vibration returns None -> kinematic
    # fallback). LOCAL_POSITION_NED stays the primary message.
    pursuer_reader = mavlink_utils.MavStateReader(
        pursuer_conn,
        ["LOCAL_POSITION_NED", "ATTITUDE", "VIBRATION"],
        mavlink_utils.parse_local_ned,
    )
    # Altitude frame for the target. Relative (height above its own home) by
    # default: the pursuer's LOCAL_POSITION_NED z is height above ITS home, so
    # with both aircraft on the same field the two are directly comparable and
    # immune to them disagreeing about that field's AMSL elevation (they were
    # 36.80 m apart on 2026-07-31). See TARGET_ALT_USE_RELATIVE in the config.
    print(
        f"[{redcolor}simple_follow{endcolor}] target altitude frame: "
        f"{'relative_alt (height above target home)' if use_rel_alt else 'AMSL - pursuer home'}"
    )
    target_reader = mavlink_utils.MavStateReader(
        target_conn,
        "GLOBAL_POSITION_INT",
        lambda msg: mavlink_utils.parse_global_int(
            msg, home_lat, home_lon, home_alt, use_relative_alt=use_rel_alt
        ),
    )

    pursuer_reader.start()
    target_reader.start()

    print(
        f"[{redcolor}simple_follow{endcolor}] Waiting for first pursuer and target positions"
    )
    while running:
        pursuer_pos, _ = pursuer_reader.get()
        target_pos, _, target_stamp, _ = target_reader.get_with_times()
        if pursuer_pos is not None and target_pos is not None and target_stamp > 0.0:
            break
        time.sleep(0.05)

    if not running:
        return

    imm = setup_imm_filter(loop_dt)
    turn_rate_estimator = HeadingTurnRateEstimator()

    # --- KOMUT YETKISI (konumlu -> goruntulu devri) -------------------------
    # bbox_to_redis.py karar vericisi 'komut_yetkisi' anahtarini yazar. Deger
    # 'goruntulu' oldugunda bu surec YETKISIZDIR: setpoint gondermez, ucus
    # moduna dokunmaz; ama estimator calismaya devam eder ki karar verici
    # 'konumlu'ya geri donerse gudum sicramasiz surdurulebilsin. Devir aninda
    # goruntulu kodun entegratorlerini tohumlayabilmesi icin son gonderilen
    # komut her dongude 'devir_durumu' anahtarina yazilir.
    yetki_client = None
    if redis_lib is not None:
        try:
            yetki_client = redis_lib.Redis(host="localhost", port=6379, db=0,
                                           socket_timeout=0.25)
            yetki_client.ping()
            print(f"[{redcolor}simple_follow{endcolor}] komut yetkisi izleniyor "
                  f"(Redis 'komut_yetkisi')")
        except Exception as exc:
            yetki_client = None
            print(f"[{redcolor}simple_follow{endcolor}] Redis yok, yetki devri "
                  f"kapali: {exc}")
    konumlu_yetkili = True
    last_yetki_poll = 0.0

    first_update = True
    last_target_stamp = None
    last_good_target_wall_time = time.monotonic()
    last_command_print = 0.0
    last_raw_turn_strength = 0.0
    last_fast_turn_onset = False
    last_yaw_cmd = 0.0
    last_gate_print = 0.0  # throttle for the terminal-entry-gate diagnostic
    # --- Mission failsafe state ---
    failsafe = MissionFailsafe(
        enabled=bool(getattr(cfg, "MISSION_FAILSAFE_ENABLED", True)),
        mode=str(getattr(cfg, "MISSION_FAILSAFE_MODE", "LOITER")),
        trip_after_s=float(getattr(cfg, "MISSION_FAILSAFE_TRIP_AFTER_S", 1.0)),
        clear_after_s=float(getattr(cfg, "MISSION_FAILSAFE_CLEAR_AFTER_S", 2.0)),
        data_clear_s=float(getattr(cfg, "MISSION_FAILSAFE_DATA_CLEAR_S", 0.3)),
    )
    target_grace_s = float(getattr(cfg, "MISSION_FAILSAFE_TARGET_GRACE_S", 0.2))
    target_handoff_s = float(getattr(cfg, "MISSION_FAILSAFE_TARGET_HANDOFF_S", 1.0))
    coast_deadreckon = bool(getattr(cfg, "MISSION_FAILSAFE_COAST_DEADRECKON", True))
    coast_age = 0.0        # seconds of dead-reckoning applied to the guidance state
    coast_loops = 0        # loops spent coasting through a dropout (diagnostic)
    data_gap_trips = 0
    last_gap_print = 0.0
    bad_meas_count = 0
    last_bad_meas_print = 0.0
    last_good_meas = None  # last ACCEPTED measurement, for the jump gate
    send_setpoint = False
    last_cmd_heading = None  # heading of the last horizontal velocity command (turn clamp state)
    z_update_freeze_remaining = 0
    z_update_freeze_active = False
    prev_mu_ct_xy = None
    mu_ct_xy_now = 0.0
    # Estimator diagnostics for the GUI (persist across non-packet loops).
    last_innov_norm = float("nan")
    last_jump_norm = float("nan")
    last_err_norm = float("nan")
    last_omega = 0.0
    # --- Dongu sagligi (2026-08-02) ---
    # loop_dt yalnizca UYKU TEMPOSU icin nominal kalir; her entegrasyon
    # (aim LPF, tur kelepcesi, reengage LPF, Z/yaw slew) OLCULEN adimla
    # calisir. GUI acikken dongu 20 Hz yerine 2.22 Hz'e dusmustu ve nominal
    # dt ile calisan kelepce komut basligini hedeften bagimsiz, sabit
    # 2.27 deg/s hizla dondurup 678 m yaricapli daire ciziyordu.
    prev_iter_time = None
    loop_dt_meas = loop_dt
    loop_dt_window = deque(maxlen=40)
    slow_loop_streak = 0
    last_slow_loop_warn = 0.0
    # Sim-zamani / duvar-saati orani: avcinin TIME_BOOT_MS artisi bolu gecen
    # gercek sure. Basarisiz kosuda 0.74, basarililarda 1.00 -- basarisizligi
    # birebir onceden haber veren gosterge.
    simtime_ref_wall = None
    simtime_ref_sim = None
    simtime_ratio = float("nan")
    last_simtime_warn = 0.0
    boot_time = time.monotonic()
    if args.yaklasim == 'kesisme':
        guidance = InterceptGuidance(
            args.kesisme_hiz, args.kesisme_terminal, args.kesisme_overshoot
        )
        print(f"[{redcolor}simple_follow{endcolor}] YAKLASIM = KESISME "
              f"(carpisma rotasi, formation_KILLER ATTACK yasasi): "
              f"hiz={args.kesisme_hiz:g} terminal={args.kesisme_terminal:g} m "
              f"overshoot={args.kesisme_overshoot:g} m -- devir CAPRAZ "
              f"geometride olabilir, bkz. mpc_memory.md")
    else:
        guidance = BehindSlotGuidance(
            args.back, args.side, args.down, args.min_heading_speed
        )
    z_slew = ZCommandSlewLimiter(
        args.z_switch_slew_rate,
        args.z_switch_jump,
        args.z_switch_window,
        args.z_switch_dmu,
        args.z_switch_mu_threshold,
        args.z_outlier_slew_rate,
        args.z_outlier_jump,
        always_slew_rate_mps=args.z_always_slew_rate,
    )
    guidance_lpf = IMMLowPassFilter()
    terminal_latch = TerminalPositionLatch(
        args.terminal_latch_tgo_s,
        args.terminal_latch_release_range_m,
    )
    lead_cache = LeadPredictionCache(
        max_substep=float(getattr(cfg, "GUIDANCE_LEAD_PREDICT_SUBSTEP_S", 0.25))
    )
    aim_limiter = AimPointRateLimiter(
        float(getattr(cfg, "AIM_POINT_MAX_SPEED_MPS", 60.0))
    )
    min_command_alt_m = float(getattr(cfg, "MIN_COMMAND_ALTITUDE_M", 15.0))
    aim_lpf_tau = float(getattr(cfg, "LEAD_AIM_LPF_TAU_S", 0.0))
    aim_lpf_state = None  # low-pass state for the lead position (jitter smoother)
    reengage_lpf_tau = float(getattr(cfg, "REENGAGE_LPF_TAU_S", 0.5))
    reengage_lpf_pos = None  # low-pass state for the REENGAGE setpoint (pos, vel)
    reengage_lpf_vel = None  # reset whenever not in REENGAGE so each re-entry re-seeds
    pursuer_stale_timeout_s = float(getattr(cfg, "PURSUER_STALE_TIMEOUT_S", 1.0))

    guidance_logger = None if args.no_log else GuidanceLogger()
    if guidance_logger is not None:
        print(
            f"[{redcolor}simple_follow{endcolor}] Guidance log: {guidance_logger.path}"
        )

    recovery = MissRecoveryController(args)
    impact = ImpactDetector(
        hit_range_m=float(getattr(cfg, "HIT_RANGE_M", 1.0)),
        impact_vibe=float(getattr(cfg, "IMPACT_VIBE_THRESHOLD", 60.0)),
        require_speed_drop=bool(getattr(cfg, "HIT_REQUIRE_SPEED_DROP", True)),
        speed_drop_mps=float(getattr(cfg, "HIT_SPEED_DROP_MPS", 8.0)),
        speed_window_s=float(getattr(cfg, "HIT_SPEED_DROP_WINDOW_S", 0.5)),
        crash_vibe=float(getattr(cfg, "CRASH_VIBE_THRESHOLD", 60.0)),
        crash_tilt_deg=float(getattr(cfg, "CRASH_TILT_DEG", 120.0)),
    )
    commanded_mode = "GUIDED"  # startup set it (unless --no-guided)
    if args.miss_recovery and args.no_guided:
        print(
            f"[{redcolor}simple_follow{endcolor}] miss recovery disabled "
            f"(needs script mode control; incompatible with --no-guided)"
        )
    elif recovery.enabled:
        reengage_lpf_txt = (
            f"lpf={reengage_lpf_tau:.2f}s" if args.reengage_lpf else "lpf=off"
        )
        print(
            f"[{redcolor}simple_follow{endcolor}] miss recovery ON "
            f"(hold={recovery.hold_s:.1f}s mode={recovery.recovery_mode} "
            f"reengage_ramp={recovery.ramp_s:.1f}s {reengage_lpf_txt})"
        )

    if not args.position_only:
        print(
            f"[{redcolor}simple_follow{endcolor}] velocity feedforward ON "
            f"(vertical vz {'sent' if args.vertical_velocity_ff else 'stripped'})"
        )
        if args.carrot_clamp:
            print(
                f"[{redcolor}simple_follow{endcolor}] carrot clamp ON "
                f"(cmd point <= {float(getattr(cfg, 'CARROT_MAX_AHEAD_M', 10.0)):.1f}m "
                f"ahead inside {float(getattr(cfg, 'CARROT_ACTIVE_RANGE_M', 80.0)):.0f}m; "
                f"long-range merge unclamped)"
            )
        print(
            f"[{redcolor}simple_follow{endcolor}] komut konum butcesi "
            f"(her zaman): hiz*{float(getattr(cfg, 'COMMAND_POSITION_BUDGET_S', 2.0)):.1f}s "
            f"+ {float(getattr(cfg, 'COMMAND_POSITION_BUDGET_MARGIN_M', 20.0)):.0f}m"
        )
        if args.super_safe_turn:
            print(
                f"[{redcolor}simple_follow{endcolor}] super_safe_turn ON "
                f"(slow to v_safe=a_lat/omega when mu_ct>="
                f"{float(getattr(cfg, 'SUPER_SAFE_TURN_MU_CT_THRESHOLD', 0.5)):.2f}, "
                f"a_lat_max={float(getattr(cfg, 'SUPER_SAFE_TURN_LATERAL_ACCEL_MPS2', 6.9)):.1f}m/s^2, "
                f"floor={float(getattr(cfg, 'SUPER_SAFE_TURN_MIN_SPEED_MPS', 5.0)):.1f}m/s)"
            )

    if args.kill_mode:
        print(
            f"[{redcolor}simple_follow{endcolor}] *** KILL MODE ON -- INTERCEPT: "
            f"terminal chain LIVE, slot ON the target "
            f"(back={args.back:.1f}m down={args.down:.1f}m) ***"
        )
    else:
        print(
            f"[{redcolor}simple_follow{endcolor}] *** KILL MODE OFF -- SIMPLE "
            f"FOLLOW: terminal chain DISABLED, standoff "
            f"back={args.back:.1f}m down={args.down:.1f}m (vehicle is never "
            f"aimed at the target) ***"
        )
    flythrough = bool(getattr(cfg, "TERMINAL_FLYTHROUGH_ENABLED", True))
    print(
        f"[{redcolor}simple_follow{endcolor}] Running "
        f"back={args.back:.1f}m side={args.side:.1f}m down={args.down:.1f}m "
        f"loop={args.loop_hz:.1f}Hz yaw_lock={int(args.yaw_lock)} "
        f"vff_z={int(not args.position_only and args.vertical_velocity_ff)}"
    )
    if not args.kill_mode:
        print(f"[{redcolor}simple_follow{endcolor}] terminal=DISABLED (kill mode off)")
    elif flythrough:
        print(
            f"[{redcolor}simple_follow{endcolor}] terminal=FLY-THROUGH-CLIMB "
            f"(down={args.down:.1f}m below, climb={float(getattr(cfg, 'TERMINAL_CLIMB_RATE_MPS', 4.0)):.1f}m/s, "
            f"up_off={float(getattr(cfg, 'TERMINAL_UP_OFFSET_M', 2.0)):.1f}m; no freeze/brake)"
        )
    else:
        print(f"[{redcolor}simple_follow{endcolor}] terminal=LEGACY freeze+brake latch")

    # GORUNURLUK (2026-08-09 dersi): acilan/kapali kollari loga YAZ; yoksa
    # hangi kolun uctugu ancak CSV kolonlarindan cikarilabiliyor.
    print(
        f"[{redcolor}simple_follow{endcolor}] HOLD irtifa kapisi="
        f"{'ACIK' if getattr(recovery, 'hold_irtifa', False) else 'KAPALI'} "
        f"(YILDIZ_HOLD_IRTIFA; HOLD alt_err <= "
        f"{getattr(recovery, 'alt_abort_arm_m', float('nan')):.1f} m olana "
        f"kadar surer, tavan "
        f"{getattr(recovery, 'hold_max_s', float('nan')):.1f} s)"
    )

    if failsafe.enabled:
        print(
            f"[{redcolor}simple_follow{endcolor}] mission failsafe ON "
            f"(stale/corrupt data -> withhold setpoints + {failsafe.mode}; "
            f"coast>{target_grace_s * 1000.0:.0f}ms "
            f"handoff>{target_handoff_s:.1f}s resume<{failsafe.data_clear_s * 1000.0:.0f}ms, "
            f"hard fault trip={failsafe.trip_after_s:.1f}s "
            f"clear={failsafe.clear_after_s:.1f}s)"
        )
    else:
        print(
            f"[{redcolor}simple_follow{endcolor}] mission failsafe OFF "
            f"(stale/corrupt data will NOT hand the vehicle back to its mission)"
        )

    # --- Start GUI ---
    # YILDIZLAR ORTAMI EKLEMESI: estimator penceresi OPT-IN'dir (eskiden
    # DISPLAY varsa kendiliginden aciliyordu). Pencere ayri bir surec DEGIL,
    # bu surecte bir PyQt5+matplotlib is parcacigidir; 10 Hz'lik cizimleri
    # GIL'i tuttugu icin ana dongu GIL bekler ve 20 Hz -> ~5 Hz'e coker.
    # OLCUM (GUI yigini -- gzclient+QGC+bbox penceresi -- her iki kosuda da
    # ACIK): GUI acik 4.9 Hz, GUI kapali 19.3 Hz; sistem CPU'su ikisinde de
    # 1600%'un ~%52'si, yani DONANIM DEGIL GIL darbogazi.
    # Acma kapilari: --gui bayragi ya da YILDIZ_GUI=1.
    # GERIYE UYUMLULUK: YILDIZ_NO_GUI=1 ve DISPLAY yoklugu HALA kesin kapatir.
    _gui_zorla_kapali = (
        os.environ.get("YILDIZ_NO_GUI", "").strip().lower() in ("1", "true", "yes", "on")
        or not os.environ.get("DISPLAY")
    )
    _gui_istendi = bool(args.gui) or (
        os.environ.get("YILDIZ_GUI", "").strip().lower() in ("1", "true", "yes", "on")
    )
    if _gui_zorla_kapali or not _gui_istendi:
        if _gui_istendi and _gui_zorla_kapali:
            print(
                f"[{redcolor}simple_follow{endcolor}] GUI istendi ama kapali "
                f"(YILDIZ_NO_GUI=1 ya da DISPLAY yok)"
            )
        else:
            print(
                f"[{redcolor}simple_follow{endcolor}] GUI kapali (varsayilan). "
                f"Acmak icin --gui / YILDIZ_GUI=1 -- dongu 20 Hz'den ~5 Hz'e duser, "
                f"ucus sirasinda ACMAYIN"
            )
        gui = None
    else:
        print(
            f"[{redcolor}simple_follow{endcolor}] Starting guidance GUI ... "
            f"(UYARI: ayni surecte PyQt5 cizimi GIL tutar, dongu ~5 Hz'e duser)"
        )
        gui = GuidanceGUI(param_module="guidance_config", refresh_hz=10)
        gui.start()

    while running:
        loop_start = time.monotonic()
        # OLCULEN dongu adimi: yarim nominal adimla tabanlanir (bir dongude
        # iki kez calisan zamanlayici zipmasina karsi) ve 0.5 s ile tavanlanir
        # (uzun bir takilma butun entegratorleri firlatmasin).
        if prev_iter_time is None:
            loop_dt_meas = loop_dt
        else:
            loop_dt_meas = min(max(loop_start - prev_iter_time, 0.5 * loop_dt), 0.5)
        prev_iter_time = loop_start
        loop_dt_window.append(loop_dt_meas)
        if loop_dt_meas > 2.0 * loop_dt:
            slow_loop_streak += 1
        else:
            slow_loop_streak = 0
        if slow_loop_streak >= 10 and loop_start - last_slow_loop_warn > 5.0:
            dt_avg = float(np.mean(loop_dt_window))
            print(
                f"[{redcolor}simple_follow{endcolor}] *** DONGU YAVAS: "
                f"olculen {1.0 / max(dt_avg, 1e-6):.2f} Hz (istenen "
                f"{args.loop_hz:.1f} Hz, dt {dt_avg * 1000.0:.0f} ms) -- "
                f"{slow_loop_streak} ardisik dongu. CPU yukunu azaltin "
                f"(YILDIZ_NO_GUI=1 / video kaydi kapali) ***"
            )
            last_slow_loop_warn = loop_start

        # Komut yetkisi 10 Hz'de yoklanir; baglanti hatasi son bilinen degeri
        # korur (kisa Redis kesintisi arac uzerinde mod savasina donmesin).
        if yetki_client is not None and loop_start - last_yetki_poll >= 0.1:
            last_yetki_poll = loop_start
            try:
                _v = yetki_client.get("komut_yetkisi")
                yeni_yetkili = (_v is None
                                or _v.decode("utf-8", "replace") != "goruntulu")
            except Exception:
                yeni_yetkili = konumlu_yetkili
            if yeni_yetkili != konumlu_yetkili:
                konumlu_yetkili = yeni_yetkili
                print(
                    f"[{redcolor}simple_follow{endcolor}] KOMUT YETKISI: "
                    + ("konumlu (geri devir, gudum surduruluyor)"
                       if konumlu_yetkili else
                       "goruntulu (setpoint/mod komutlari durduruldu, "
                       "estimator calismaya devam ediyor)")
                )

        pursuer_pos, pursuer_vel, pursuer_stamp, pursuer_wall_stamp = (
            pursuer_reader.get_with_times()
        )
        target_meas, _, target_stamp, target_wall_stamp = target_reader.get_with_times()

        # Sim-zamani orani: avcinin kendi saati (TIME_BOOT_MS) ~5 s'lik duvar
        # saati penceresinde ne kadar ilerledi? 1.0'in altina dusmesi SITL'in
        # gercek zamanin gerisinde kaldigini soyler -- gudum ne yaparsa yapsin
        # bu kosu gecerli degildir.
        if pursuer_stamp > 0.0:
            if simtime_ref_wall is None:
                simtime_ref_wall = loop_start
                simtime_ref_sim = pursuer_stamp
            elif loop_start - simtime_ref_wall >= 5.0:
                d_wall = loop_start - simtime_ref_wall
                d_sim = pursuer_stamp - simtime_ref_sim
                simtime_ratio = d_sim / d_wall if d_wall > 1e-6 else float("nan")
                simtime_ref_wall = loop_start
                simtime_ref_sim = pursuer_stamp
                if (
                    np.isfinite(simtime_ratio)
                    and simtime_ratio < 0.9
                    and loop_start - last_simtime_warn > 5.0
                ):
                    print(
                        f"[{redcolor}simple_follow{endcolor}] *** SIMULASYON "
                        f"GERIDE: sim/duvar orani {simtime_ratio:.2f} (<0.90) -- "
                        f"SITL gercek zamana yetisemiyor, bu kosunun sonuclari "
                        f"gecersizdir ***"
                    )
                    last_simtime_warn = loop_start

        if pursuer_pos is None or target_meas is None or target_stamp <= 0.0:
            time.sleep(0.05)
            continue

        pursuer_pos_np = np.asarray(pursuer_pos, dtype=float).reshape(3)
        pursuer_vel_np = (
            np.asarray(pursuer_vel, dtype=float).reshape(3)
            if pursuer_vel is not None
            else np.zeros(3)
        )

        imm_updated_this_loop = False
        if last_target_stamp is None or target_stamp > last_target_stamp:
            z_meas = np.asarray(target_meas, dtype=float).reshape(3)
            # Sanity-gate the measurement BEFORE the estimator absorbs it: one
            # corrupt position poisons the filter (and every setpoint derived
            # from it) long after the bad sample is gone.
            meas_ok, meas_why = measurement_is_plausible(
                z_meas,
                prev_meas=last_good_meas,
                dt_s=(
                    target_stamp - float(last_target_stamp)
                    if last_target_stamp is not None else 0.0
                ),
                max_implied_speed_mps=float(
                    getattr(cfg, "TARGET_MAX_IMPLIED_SPEED_MPS", 120.0)
                ),
                max_altitude_m=float(getattr(cfg, "TARGET_MAX_ALTITUDE_M", 2000.0)),
            )
            if not meas_ok:
                # Rejected: do NOT update the filter and do NOT refresh the
                # good-data clock, so a sustained corrupt feed reads exactly
                # like a dead one and trips the failsafe.
                bad_meas_count += 1
                last_target_stamp = target_stamp
                if loop_start - last_bad_meas_print > 1.0:
                    print(
                        f"[{redcolor}simple_follow{endcolor}] target measurement "
                        f"REJECTED ({meas_why}; {bad_meas_count} total)"
                    )
                    last_bad_meas_print = loop_start
                time.sleep(min(loop_dt, 0.1))
                continue
            last_good_meas = z_meas.copy()
            imm_updated_this_loop = True
            omega_hint = turn_rate_estimator.update(z_meas, target_stamp)
            if first_update:
                for filt in imm.filters:
                    filt.x[0:3] = z_meas
                imm.x = imm.filters[0].x.copy()
                stabilize_omega_states(imm)
                first_update = False
            else:
                assert last_target_stamp is not None
                last_target_stamp_value = float(last_target_stamp)
                dt_meas = clamp_filter_dt(target_stamp - last_target_stamp_value)
                last_raw_turn_strength = apply_fast_turn_onset_hint(
                    imm,
                    turn_rate_estimator.raw_omega,
                    turn_rate_estimator.speed_xy,
                )
                was_fast_turn_onset = last_fast_turn_onset
                last_fast_turn_onset = last_raw_turn_strength > 0.0
                if last_fast_turn_onset:
                    z_slew.open_window(loop_start, "fast_turn")
                if last_fast_turn_onset and not was_fast_turn_onset:
                    z_update_freeze_remaining = max(
                        z_update_freeze_remaining,
                        max(0, int(args.z_update_freeze_packets)),
                    )
                apply_turn_rate_hint(imm, omega_hint)
                predict_imm_over_dt(imm, dt_meas)
                # Predicted position before the measurement update: innovation is
                # meas - pred, the update jump is post - pred. These feed the GUI
                # residual-norm plot (same diagnostics filterwndr's own view shows).
                pred_pos_before_update = np.asarray(imm.x[0:3], dtype=float).copy()
                z_update_freeze_active = z_update_freeze_remaining > 0
                if z_update_freeze_active:
                    update_imm_preserving_vertical(imm, z_meas)
                    z_update_freeze_remaining -= 1
                else:
                    # Snapshot vertical state before the update so we can
                    # retroactively freeze Z if this packet is the one that
                    # pushes mu_ct_xy across the activation threshold.
                    ct_freeze_threshold = float(args.z_ct_freeze_mu_threshold)
                    ct_freeze_packets = max(0, int(args.z_ct_freeze_packets))
                    prev_mu = prev_mu_ct_xy
                    snapshot_before = (
                        snapshot_imm_vertical_state(imm)
                        if ct_freeze_packets > 0 and prev_mu is not None
                        else None
                    )
                    imm.update(z_meas)
                    mu_ct_xy_now = ct_mode_probability(imm)
                    if (
                        snapshot_before is not None
                        and prev_mu is not None
                        and prev_mu < ct_freeze_threshold <= mu_ct_xy_now
                    ):
                        # This packet activated CT. Retroactively freeze Z
                        # for this packet and arm the freeze for N-1 more.
                        restore_imm_vertical_state(imm, snapshot_before)
                        if ct_freeze_packets > 1:
                            z_update_freeze_remaining = max(
                                z_update_freeze_remaining,
                                ct_freeze_packets - 1,
                            )
                    prev_mu_ct_xy = mu_ct_xy_now
                apply_turn_rate_hint(imm, omega_hint)
                stabilize_omega_states(imm)

                post_pos = np.asarray(imm.x[0:3], dtype=float)
                last_innov_norm = float(
                    np.linalg.norm(z_meas - pred_pos_before_update)
                )
                last_jump_norm = float(
                    np.linalg.norm(post_pos - pred_pos_before_update)
                )
                last_err_norm = float(np.linalg.norm(post_pos - z_meas))
                last_omega = float(imm.x[9])

            last_target_stamp = target_stamp
            last_good_target_wall_time = target_wall_stamp

        target_age = loop_start - last_good_target_wall_time
        if target_age > args.target_timeout:
            if loop_start - last_command_print > 1.0:
                print(
                    f"[{redcolor}simple_follow{endcolor}] Target stale for {target_age:.1f}s, "
                    f"withholding setpoints"
                )
                last_command_print = loop_start
            # Re-anchor the aim smoother on reacquisition rather than crawling
            # from a stale anchor toward a target that moved during the gap.
            aim_lpf_state = None
            # This branch skips the send path (and with it the failsafe's mode
            # command), so engage the handoff here too. Normally the handoff
            # limit is shorter than --target-timeout and has already fired; this
            # covers a config where it is not.
            if failsafe.update(
                loop_start, False, f"target stale {target_age:.1f}s",
                fault_class="data", immediate=True,
            ) and konumlu_yetkili and failsafe.mode and failsafe.mode != commanded_mode:
                try:
                    set_mode(pursuer_conn, failsafe.mode)
                    commanded_mode = failsafe.mode
                    print(
                        f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE: "
                        f"pursuer -> {commanded_mode} ({failsafe.reason})"
                    )
                except Exception as exc:
                    print(
                        f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE "
                        f"set_mode {failsafe.mode!r} FAILED: {exc}"
                    )
            time.sleep(min(loop_dt, 0.1))
            continue

        # Own-position staleness guard (2026-07-24 review): the target feed has
        # always had this; the pursuer did not. A frozen own position feeds
        # phantom range/t_go/CPA and defeats the altitude abort (alt_error uses
        # the frozen z), so hold setpoints and warn, exactly like target-stale.
        # The estimator keeps absorbing target packets above; only guidance and
        # decisions are gated.
        pursuer_age = loop_start - pursuer_wall_stamp
        if pursuer_age > pursuer_stale_timeout_s:
            if loop_start - last_command_print > 1.0:
                print(
                    f"[{redcolor}simple_follow{endcolor}] PURSUER position stale "
                    f"for {pursuer_age:.1f}s, holding previous GUIDED target"
                )
                last_command_print = loop_start
            aim_lpf_state = None
            time.sleep(min(loop_dt, 0.1))
            continue

        # --- Mission health verdict for this loop ---
        # Everything the setpoint depends on must be trustworthy. The estimator
        # state is checked explicitly: once a filter goes non-finite it never
        # recovers on its own, and every downstream setpoint would be NaN.
        estimator_ok = bool(np.all(np.isfinite(np.asarray(imm.x, dtype=float))))
        # HARD faults first -- they outrank a data gap and clear far slower.
        health_class = "hard"
        if not estimator_ok:
            health_reason = "estimator diverged (non-finite state)"
        elif impact.crashed:
            health_reason = "crash detected"
        elif target_handoff_s > 0.0 and target_age > target_handoff_s:
            # The feed is genuinely gone (well past the coast window):
            # stop commanding and hand the vehicle to its mission.
            health_reason = f"no target data {target_age:.1f}s"
            health_class = "data"
        else:
            health_reason = ""
        if failsafe.update(
            loop_start,
            not health_reason,
            health_reason,
            fault_class=health_class,
            immediate=(health_class == "data"),
        ):
            if failsafe.tripped:
                # A short tracker dropout is expected and self-healing, so it
                # is reported quietly (and throttled); a hard fault is not.
                if failsafe.fault_class == "data":
                    data_gap_trips += 1
                    if loop_start - last_gap_print > 2.0:
                        print(
                            f"[{redcolor}simple_follow{endcolor}] failsafe: "
                            f"{failsafe.reason} -> setpoints withheld "
                            f"({data_gap_trips} dropouts so far)"
                        )
                        last_gap_print = loop_start
                else:
                    print(
                        f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE: "
                        f"{failsafe.reason} -> {failsafe.mode}"
                    )
            else:
                print(
                    f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE CLEARED: "
                    f"data healthy again -> resuming guidance"
                )

        mode_probs = aggregate_mode_probabilities(imm)
        # Engagement geometry vs the target ESTIMATE (imm.x) -- never the raw
        # target feed (that only exists in the test rig; in deployment the
        # target is observed through the estimator, not telemetry). Everything
        # that decides -- the terminal latch, the extension activation, the
        # miss-recovery FSM -- and everything logged keys off this. The lead
        # prediction below is only where the vehicle is AIMED. (2026-07-11:
        # keying decisions off the lead point let prediction transients fire
        # BRAKE at 100+ m estimated range and made the logged range fictional.)
        intercept = estimate_intercept_geometry(
            np.asarray(imm.x[0:3], dtype=float).reshape(3),
            pursuer_pos_np,
            np.asarray(imm.x[3:6], dtype=float).reshape(3),
            pursuer_vel_np,
        )
        guidance_horizon_s = compute_guidance_prediction_horizon(
            intercept,
            prev_mu_ct_xy,
            mode_probs,
        )
        # Envelope-clamped lead (position+velocity+turn rate; linear accel and
        # CT centripetal bounded to the flight envelope) plus a covariance gate
        # that shrinks the lead toward the current estimate when the filter's
        # predicted position sigma is large. Together these replace the old hard
        # vertical pin: the vertical accel/vz are dropped by config (level
        # target) and the horizontal lead self-limits when tracking is poor.
        x_pred, p_pred = lead_cache.get(
            imm, guidance_horizon_s, imm_updated_this_loop
        )
        predicted_state, lead_k_h, lead_k_v = lead_cov_gate(
            x_pred, p_pred, imm.x
        )
        # Low-pass the lead position AND velocity at loop rate (time-constant
        # filter, runs every loop incl. between packets). This is the primary
        # jitter smoother: the aim is a projection of the noisy velocity
        # heading, so it dithers packet-to-packet; a rate cap bounds its speed
        # but not the direction reversals that shed copter yaw, whereas this
        # bounds both. Smoothing the velocity channels too matters -- the
        # behind-slot offset rotates with the velocity heading, so a jittery
        # heading re-injects setpoint whip (raw heading measured 1084 deg/s of
        # commanded turn rate, LPF'd heading 125 deg/s; replay 2026-07-13). The
        # velocity FF and behind-slot heading both read the smoothed value;
        # terminal geometry keys off imm.x (raw), so it is unaffected.
        if aim_lpf_tau > 1e-6:
            alpha = loop_dt_meas / (loop_dt_meas + aim_lpf_tau)
            lead_pv = np.asarray(predicted_state[0:6], dtype=float)
            if aim_lpf_state is None:
                aim_lpf_state = lead_pv.copy()
            else:
                aim_lpf_state = aim_lpf_state + alpha * (lead_pv - aim_lpf_state)
            predicted_state = np.asarray(predicted_state, dtype=float).copy()
            predicted_state[0:6] = aim_lpf_state
        filtered_state = guidance_lpf.filter(predicted_state, dt=loop_dt_meas)
        # COAST through a tracker dropout. Between packets the estimate is
        # frozen, so a gap leaves the slot standing still in space while the
        # target flies on -- the vehicle catches up to a static point and slows
        # down, which is exactly what a few dropped frames must NOT cost us.
        # Past the grace window we therefore dead-reckon the guidance state
        # forward at its own estimated velocity, so the slot keeps moving WITH
        # the target. Only applied to abnormal gaps: inside the grace window
        # normal between-packet behaviour is left bit-identical to what has
        # already been flight-tested. Bounded by the handoff limit (beyond it
        # we stop commanding entirely rather than extrapolate into fiction).
        coast_age = 0.0
        if (
            coast_deadreckon
            and target_grace_s > 0.0
            and target_age > target_grace_s
            and not failsafe.tripped
        ):
            coast_age = min(
                target_age - target_grace_s,
                max(0.0, target_handoff_s - target_grace_s),
            )
            filtered_state = np.asarray(filtered_state, dtype=float).copy()
            filtered_state[0:3] = filtered_state[0:3] + filtered_state[3:6] * coast_age
            coast_loops += 1
        slot_pos, slot_vel, target_pos = guidance.update(filtered_state, pursuer_pos_np)
        # Nominal aim altitude BEFORE the terminal spear/latch mutate slot_pos.
        # The altitude abort must judge "can the vehicle hold its commanded
        # altitude" against this, not the speared z: the LOS spear deliberately
        # offsets slot z by up to ~29 m in vertical-displacement geometry, which
        # read as phantom tracking failure (2026-07-24 review).
        aim_z_nominal = float(slot_pos[2])
        slot_vel = clamp_norm(slot_vel, args.max_feedforward_speed)
        # Strip the estimated vertical velocity from the velocity feedforward: the
        # smooth position setpoint carries Z instead. CT/CA turns inject Z transients
        # into vz, and feeding that to the copter spikes collective throttle ->
        # motor saturation -> yaw authority shed first -> physical yaw instability
        # (only in pos+vel, never pos-only). The target flies ~level, so no real
        # vertical motion is lost. --vertical-velocity-ff restores the raw vz.
        if not args.position_only and not args.vertical_velocity_ff:
            slot_vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
            slot_vel[2] = 0.0
        # super_safe_turn: when the estimator is confident the target is in a
        # coordinated turn, pre-emptively cap the commanded SPEED to the fastest
        # value at which the required turn still fits inside the lateral-accel
        # (bank) envelope. To follow a turn of rate omega at speed v the pursuer
        # needs a_lat = v*omega; solving for the envelope limit gives
        # v_safe = a_lat_max / omega. Slowing to v_safe means the downstream turn
        # clamp no longer has to shave the heading (omega_max = a_lat_max/v_safe
        # equals the target omega), so the pursuer tracks the full turn instead
        # of drifting a gentler arc -- and never demands a bank it cannot hold.
        # Opt-in safety measure (default off); pos+vel only, since the whole
        # saturation/yaw-shed family is a velocity-setpoint phenomenon.
        safe_turn_vmax = ""
        if args.super_safe_turn and not args.position_only:
            sst_mu_thresh = float(
                getattr(cfg, "SUPER_SAFE_TURN_MU_CT_THRESHOLD", 0.5)
            )
            sst_omega_min = float(
                getattr(cfg, "SUPER_SAFE_TURN_OMEGA_MIN_RAD_S", 0.1)
            )
            sst_a_lat_max = float(
                getattr(
                    cfg,
                    "SUPER_SAFE_TURN_LATERAL_ACCEL_MPS2",
                    getattr(cfg, "COMMAND_LATERAL_ACCEL_MAX_MPS2", 6.9),
                )
            )
            sst_min_speed = float(
                getattr(cfg, "SUPER_SAFE_TURN_MIN_SPEED_MPS", 5.0)
            )
            sst_omega = abs(last_omega)
            if mu_ct_xy_now >= sst_mu_thresh and sst_omega >= sst_omega_min:
                v_safe = max(sst_a_lat_max / sst_omega, sst_min_speed)
                # clamp_norm only ever reduces the vector, so this can slow the
                # command but never speed it up past the existing FF cap.
                slot_vel = clamp_norm(slot_vel, v_safe)
                safe_turn_vmax = v_safe
        # Command conditioning: cap the commanded turn to a fixed bank margin so
        # the pursuer never demands more lateral accel than it can execute with
        # yaw headroom to spare (clamp_commanded_turn). Runs on the CHASE/
        # midcourse command -- where the co-speed turn-following crashes happen
        # -- before the terminal spear/latch reshape the setpoint. pos+vel only:
        # the saturation/yaw-shed family is a velocity-setpoint phenomenon.
        cmd_lat_accel = 0.0
        turn_clamped = False
        if (
            not args.position_only
            and bool(getattr(cfg, "COMMAND_TURN_CLAMP_ENABLED", True))
        ):
            (
                slot_pos,
                slot_vel,
                last_cmd_heading,
                cmd_lat_accel,
                turn_clamped,
            ) = clamp_commanded_turn(
                slot_pos,
                slot_vel,
                target_pos,
                last_cmd_heading,
                loop_dt_meas,
                float(getattr(cfg, "COMMAND_LATERAL_ACCEL_MAX_MPS2", 6.9)),
                float(getattr(cfg, "COMMAND_TURN_CLAMP_MIN_SPEED_MPS", 2.0)),
                bool(getattr(cfg, "COMMAND_TURN_CLAMP_ROTATE_POSITION", True)),
                float(getattr(cfg, "COMMAND_TURN_CLAMP_MAX_ROTATION_DEG", 10.0)),
            )
        # The aggressive terminal dive (extension + freeze latch) only runs in
        # the CHASE state. After a miss the recovery machine takes over.
        extended_terminal = False
        terminal_latched = False
        terminal_latch_armed = False
        terminal_extend_distance_m = 0.0
        # KILL MODE OFF -> the entire terminal chain is skipped: no LOS spear,
        # no terminal window, no fly-through climb. The slot stays the plain
        # standoff behind/below the target and the vehicle simply follows.
        if args.kill_mode and recovery.state == MissRecoveryController.CHASE:
            terminal_extend_distance_m = compute_terminal_extension_distance(
                intercept,
                float(np.linalg.norm(pursuer_vel_np)),
            )
            # Spear THROUGH the lead point (where the intercept is expected),
            # but activate on the TRUE range to the target. As t_go shrinks
            # the lead collapses onto the target, so the two frames converge
            # exactly when it matters.
            aim_rel = np.asarray(target_pos, dtype=float).reshape(3) - pursuer_pos_np
            aim_dist = float(np.linalg.norm(aim_rel))
            aim_los_hat = (
                aim_rel / aim_dist if aim_dist > 1e-6 else intercept["los_hat"]
            )
            slot_pos, extended_terminal = extend_position_target_past_target(
                slot_pos,
                target_pos,
                aim_los_hat,
                intercept["range_m"],
                args.terminal_extend_range_m,
                terminal_extend_distance_m,
                blend_band_m=args.terminal_extend_blend_band_m,
            )
            # Terminal-entry geometry gate: only OPEN the terminal window from
            # healthy geometry -- actually closing, and holding the commanded
            # altitude. The 2026-07-29 logs show latch-ONs at closing ~0-2 m/s
            # with the vehicle 26+ m off altitude (a near-vertical "intercept");
            # committing the climb-from-below there turns a recoverable approach
            # into a saturation event. Gates ARMING only; an active window still
            # releases normally. 0 disables the individual check.
            gate_min_closing = float(
                getattr(cfg, "TERMINAL_ENTRY_MIN_CLOSING_MPS", 3.0)
            )
            gate_max_alt_err = float(
                getattr(cfg, "TERMINAL_ENTRY_MAX_ALT_ERR_M", 8.0)
            )
            entry_alt_err = abs(float(pursuer_pos_np[2]) - aim_z_nominal)
            terminal_arm_ok = (
                gate_min_closing <= 0.0
                or float(intercept["closing_velocity"]) >= gate_min_closing
            ) and (gate_max_alt_err <= 0.0 or entry_alt_err <= gate_max_alt_err)
            if (
                not terminal_arm_ok
                and not terminal_latch.active
                and terminal_latch.latch_tgo_s > 0.0
                and intercept["t_go_s"] <= terminal_latch.latch_tgo_s
                and intercept["range_m"] <= terminal_latch.arm_range_m
                and loop_start - last_gate_print > 1.0
            ):
                print(
                    f"[{redcolor}simple_follow{endcolor}] terminal entry BLOCKED "
                    f"(closing {intercept['closing_velocity']:.1f} m/s, "
                    f"alt_err {entry_alt_err:.1f} m; need >="
                    f"{gate_min_closing:.1f} m/s and <={gate_max_alt_err:.1f} m)"
                )
                last_gate_print = loop_start
            if bool(getattr(cfg, "TERMINAL_FLYTHROUGH_ENABLED", True)):
                # Fly-through climb-from-below (2026-07-28). No freeze, no brake:
                # the horizontal spear above already aims PAST the target, so we
                # keep the co-speed velocity FF and fly straight through. In the
                # terminal window we additionally climb UP into the target from
                # underneath -- aim above it and command a bounded upward
                # velocity. A miss then zooms up and gravity decelerates it
                # (recoverable) instead of the old brake-into-a-frozen-point,
                # which ballooned a 34 m/s approach into a saturated tumble
                # (log 145054). The climb rate is capped well under the copter's
                # climb ceiling so collective keeps reserve for yaw/attitude
                # control -- saturation is the entire crash chain.
                # Reuse the latch's arm/release HYSTERESIS for a debounced
                # terminal window -- the packet-quantised range/t_go staircase
                # (2026-07-24) would otherwise flicker a bare threshold on and
                # off -- but DISCARD its frozen position: we keep flying the
                # live spear + climb and never freeze.
                _frozen, terminal_active, terminal_latch_armed = terminal_latch.update(
                    slot_pos,
                    intercept["t_go_s"],
                    intercept["range_m"],
                    allow_arm=terminal_arm_ok,
                )
                if terminal_active:
                    up_off = float(getattr(cfg, "TERMINAL_UP_OFFSET_M", 2.0))
                    climb = float(getattr(cfg, "TERMINAL_CLIMB_RATE_MPS", 4.0))
                    slot_pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
                    # NED: smaller z is higher. Aim above the target so the
                    # vehicle commits to climbing THROUGH its altitude.
                    slot_pos[2] = float(target_pos[2]) - up_off
                    slot_vel = np.asarray(slot_vel, dtype=float).reshape(3).copy()
                    slot_vel[2] = -climb  # NED: negative = climb (throttle up, bounded)
                terminal_latched = terminal_active
            else:
                # Legacy freeze-&-brake latch: freeze the aim and drop the
                # velocity FF (a fixed point plus ~20 m/s of target-velocity FF
                # is self-contradictory -- after overshoot the controller fights
                # itself and balloons, log 110805). Kept behind the config flag.
                slot_pos, terminal_latched, terminal_latch_armed = terminal_latch.update(
                    slot_pos,
                    intercept["t_go_s"],
                    intercept["range_m"],
                    allow_arm=terminal_arm_ok,
                )
                if terminal_latched and bool(
                    getattr(cfg, "TERMINAL_LATCH_ZERO_VELOCITY_FF", True)
                ):
                    slot_vel = np.zeros(3, dtype=float)

        # Altitude tracking error: how far the vehicle is from the altitude it
        # is being commanded to hold. Judged against the NOMINAL aim altitude
        # (pre-spear/latch), so the deliberate terminal LOS offset never counts
        # as tracking failure. A large value means the copter cannot hold
        # altitude (saturation from the aggressive horizontal chase) and is
        # falling/ballooning -- the abort trigger inside recovery.update.
        alt_error = abs(float(pursuer_pos_np[2]) - aim_z_nominal)
        pursuer_yaw_rate = pursuer_reader.get_yaw_rate()
        pursuer_att = pursuer_reader.get_attitude()
        # Impact detection (hit counter + crash screen). Scores against the TRUE
        # pursuer<->target distance -- test-only telemetry is legitimate for
        # scoring/diagnostics/GUI, never for guidance decisions (which stay on
        # imm.x). Plus the pursuer's own speed, tilt, and vibration. tilt is None
        # when attitude is stale -> the tilt-crash path simply idles.
        impact_tilt_deg = (
            float(np.degrees(np.hypot(pursuer_att[0], pursuer_att[1])))
            if pursuer_att is not None else None
        )
        true_range_m = float(
            np.linalg.norm(
                pursuer_pos_np - np.asarray(target_meas, dtype=float).reshape(3)
            )
        )
        impact_status = impact.update(
            loop_start,
            true_range_m,
            float(np.linalg.norm(pursuer_vel_np)),
            tilt_deg=impact_tilt_deg,
            vibe=pursuer_reader.get_vibration(),
        )
        if impact_status["event"]:
            r = impact.last_hit_range
            rtxt = f" at {r:.2f} m" if (impact_status["event"] == "HIT" and r is not None) else ""
            print(
                f"[{redcolor}simple_follow{endcolor}] IMPACT: "
                f"{impact_status['event']}{rtxt} "
                f"(hits={impact_status['hits']}, vibe_max={impact_status['vibe_max']:.0f})"
            )
        # Time-consistent range sample for the CPA trigger, ONLY on loops where
        # the estimate is packet-fresh, with the pursuer dead-reckoned to now
        # via its own velocity. The naive every-loop range is a +-5-8 m
        # staircase (estimate frozen between target packets, own position ~3 Hz)
        # that fired CPA before the true pass (2026-07-24 review).
        cpa_range_m = None
        if imm_updated_this_loop:
            p_age = min(max(loop_start - pursuer_wall_stamp, 0.0), 0.6)
            pursuer_dr = pursuer_pos_np + pursuer_vel_np * p_age
            cpa_range_m = float(
                np.linalg.norm(
                    np.asarray(imm.x[0:3], dtype=float).reshape(3) - pursuer_dr
                )
            )
        directive = recovery.update(
            loop_start,
            terminal_latched,
            intercept["closing_velocity"],
            alt_error=alt_error,
            yaw_rate=pursuer_yaw_rate,
            range_m=cpa_range_m,
        )
        recovery_state = directive["state"]
        # Drop the REENGAGE command LPF state whenever we are not re-engaging, so
        # each re-entry re-seeds from the vehicle's current pose (a stale state
        # would step the setpoint on the next re-engagement).
        if recovery_state != MissRecoveryController.REENGAGE:
            reengage_lpf_pos = None
            reengage_lpf_vel = None
        # Leaving CHASE means recovery owns the vehicle: drop the frozen aim so
        # the next CHASE entry cannot inherit a stale latch from this pass.
        if recovery_state != MissRecoveryController.CHASE:
            terminal_latch.reset()
        elif directive.get("unlatch"):
            # CPA pass confirmed but HOLD suppressed by brake spacing: release
            # the frozen aim and keep chasing live geometry (never balloon
            # against an overshot point). This loop still flew the frozen aim
            # (and is logged as latched); the release takes effect next loop.
            terminal_latch.reset()

        # Mission failsafe OUTRANKS the recovery FSM: while tripped the vehicle
        # goes to (and stays in) the mission mode and we send no setpoints at
        # all, whatever the state machine wants. Unlike the Redis runner this
        # one owns the MAVLink link, so it commands the mode itself rather than
        # asking a bridge for it.
        send_setpoint = (
            bool(directive["send"]) and not failsafe.tripped and konumlu_yetkili
        )
        if failsafe.tripped and konumlu_yetkili:
            if failsafe.mode and failsafe.mode != commanded_mode:
                try:
                    set_mode(pursuer_conn, failsafe.mode)
                    commanded_mode = failsafe.mode
                    print(
                        f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE: "
                        f"pursuer -> {commanded_mode} ({failsafe.reason})"
                    )
                except Exception as exc:
                    print(
                        f"[{redcolor}simple_follow{endcolor}] MISSION FAILSAFE "
                        f"set_mode {failsafe.mode!r} FAILED: {exc} "
                        f"(setpoints still withheld)"
                    )
        # Apply the requested flight mode (only when it changes).
        elif (
            konumlu_yetkili
            and recovery.enabled
            and directive["mode"]
            and directive["mode"] != commanded_mode
        ):
            try:
                set_mode(pursuer_conn, directive["mode"])
                commanded_mode = directive["mode"]
                print(
                    f"[{redcolor}simple_follow{endcolor}] recovery -> {recovery_state} "
                    f"(mode {commanded_mode})"
                )
            except Exception as exc:
                print(
                    f"[{redcolor}simple_follow{endcolor}] recovery set_mode "
                    f"{directive['mode']!r} failed: {exc}"
                )

        # Diagnostics defaults for loops where we do not send a setpoint (HOLD).
        z_switch = False
        z_limited = False
        z_limited_by = 0.0
        z_limit_reason = (
            "failsafe" if failsafe.tripped else ("hold" if not send_setpoint else "")
        )
        aim_limited = False
        alt_floored = False
        carrot_limited = False
        accel_ff = None
        yaw_frozen = False
        if not send_setpoint:
            # Re-seed the Z command from the vehicle's own altitude when the
            # send path resumes, instead of ramping from the stale pre-HOLD
            # commanded z (the seed only applies while prev_cmd_z is None).
            z_slew.prev_cmd_z = None

        if send_setpoint:
            if recovery_state == MissRecoveryController.REENGAGE:
                # Re-approach the ACTUAL target estimate (not the lead
                # prediction, which whips during turns -- chasing it from a
                # bad post-miss attitude is what diverged before), no
                # extension/latch, speed limited and ramping up, so a
                # bad-attitude pursuer eases back in instead of instantly
                # re-committing at full speed.
                slot_pos = np.asarray(imm.x[0:3], dtype=float).reshape(3).copy()
                slot_vel = clamp_norm(slot_vel, directive["speed_cap"])
                # Optional command low-pass: seed from the vehicle's own state on
                # re-entry, then ease the setpoint toward the target over ~tau,
                # so a possibly still-settling pursuer is not re-committed with a
                # full-authority step. Runs on top of the speed ramp; toggle
                # --reengage-lpf / --no-reengage-lpf.
                if args.reengage_lpf and reengage_lpf_tau > 1e-6:
                    if reengage_lpf_pos is None:
                        reengage_lpf_pos = pursuer_pos_np.copy()
                        reengage_lpf_vel = pursuer_vel_np.copy()
                    a_re = loop_dt_meas / (loop_dt_meas + reengage_lpf_tau)
                    reengage_lpf_pos = reengage_lpf_pos + a_re * (slot_pos - reengage_lpf_pos)
                    reengage_lpf_vel = reengage_lpf_vel + a_re * (slot_vel - reengage_lpf_vel)
                    slot_pos = reengage_lpf_pos.copy()
                    slot_vel = reengage_lpf_vel.copy()

            # Carrot-distance clamp (closure governor): cap how far AHEAD of
            # the vehicle the commanded position may sit, so ArduCopter's
            # internal position-error P-term -- the one command authority this
            # script cannot otherwise bound -- adds at most ~PSC_POSXY_P*D_max
            # of speed over the velocity FF (the 34-41 m/s crash sprints were
            # this internal term acting on a 30-60 m re-approach error).
            # Horizontal only (commanded Z is governed by the z-slew; scaling
            # it would dilute the terminal climb), direction preserved (still
            # flies AT the spear), and the carrot recedes as the vehicle
            # advances -- this never brakes; closure holds at the design
            # overtake rate all the way through impact. pos+vel only.
            #
            # 2026-08-02: eski butce d_allow = carrot_max + max(0, d_xy -
            # carrot_range) idi; 1:1 buyudugu icin 821 m'lik bir ofsette bile
            # 751 m'ye "kelepceleniyor" ve carrot_limited=1 goruntusune ragmen
            # HICBIR sinir uygulanmis olmuyordu. Uzerine, HER ZAMAN gecerli,
            # SABIT METRIK bir butce eklendi: komut noktasi avcidan en fazla
            # speed_cmd * BUDGET_S + BUDGET_MARGIN_M kadar ilerde olabilir.
            # Konum hatasi boylece ArduCopter'in ic P-terimine ~20 m/s'lik
            # ekstra hiz enjekte edemez (dataflash: istenen hiz 39.4 m/s =
            # 2 x WPNAV_SPEED).
            if not args.position_only:
                carrot_max_m = float(getattr(cfg, "CARROT_MAX_AHEAD_M", 10.0))
                carrot_range_m = float(getattr(cfg, "CARROT_ACTIVE_RANGE_M", 80.0))
                budget_s = float(getattr(cfg, "COMMAND_POSITION_BUDGET_S", 2.0))
                budget_margin_m = float(
                    getattr(cfg, "COMMAND_POSITION_BUDGET_MARGIN_M", 20.0)
                )
                off_xy = (
                    np.asarray(slot_pos, dtype=float).reshape(3)[0:2]
                    - pursuer_pos_np[0:2]
                )
                d_xy = float(np.linalg.norm(off_xy))
                d_allow = float("inf")
                if args.carrot_clamp and carrot_max_m > 0.0:
                    # Bound the SPRINT regime only. Beyond carrot_range_m the
                    # allowance grows 1:1 with the offset, so a long-range merge
                    # keeps essentially its full position error and closes fast.
                    # Clamping everywhere starved the merge: with the error
                    # pinned at carrot_max_m the target-VELOCITY feedforward
                    # dominates, and that vector points along the target's
                    # TRACK, not at it (sim 20260731_102802: 478 m range, FF
                    # 138 deg off LOS, pursuer flew sideways closing 5 m/s).
                    # Growing the allowance keeps the command continuous.
                    d_allow = carrot_max_m
                    if carrot_range_m > 0.0:
                        d_allow += max(0.0, d_xy - carrot_range_m)
                # SABIT METRIK TAVAN: kosula bagli degil, her dongude gecerli.
                cmd_speed_xy = float(
                    np.linalg.norm(np.asarray(slot_vel, dtype=float).reshape(3)[0:2])
                )
                d_allow = min(d_allow, cmd_speed_xy * budget_s + budget_margin_m)
                if d_xy > d_allow > 0.0:
                    slot_pos = np.asarray(slot_pos, dtype=float).reshape(3).copy()
                    slot_pos[0:2] = (
                        pursuer_pos_np[0:2] + off_xy * (d_allow / d_xy)
                    )
                    carrot_limited = True

            z_switch = z_slew.update_mode_probability(
                ct_mode_probability(imm), loop_start
            )
            slot_pos, slot_vel, z_limited, z_limited_by, z_limit_reason = z_slew.limit(
                slot_pos,
                slot_vel,
                loop_start,
                loop_dt_meas,
                seed_z=float(pursuer_pos_np[2]),
            )

            # Final output governors: a hard cap on how fast the commanded
            # point may move, then an altitude floor. Whatever upstream logic
            # produces, the vehicle never sees a teleporting or underground
            # setpoint.
            slot_pos, aim_limited = aim_limiter.limit(slot_pos, loop_start)
            slot_pos, alt_floored = clamp_command_altitude(
                slot_pos, min_command_alt_m
            )

            if args.yaw_lock:
                # Yaw law: "velocity" points the nose along the pursuer's course
                # (fly forward, zero crab); "los" is the legacy point-at-target.
                min_range = float(getattr(cfg, "YAW_LOCK_MIN_RANGE_M", 10.0))
                if str(getattr(cfg, "YAW_LOCK_MODE", "velocity")) == "velocity":
                    desired_yaw = yaw_to_velocity(
                        pursuer_vel_np,
                        last_yaw_cmd,
                        float(getattr(cfg, "YAW_LOCK_MIN_SPEED_MPS", 3.0)),
                    )
                else:
                    desired_yaw = yaw_to_los(
                        target_pos, pursuer_pos_np, last_yaw_cmd, min_range,
                    )
                # Freeze the yaw slew when saturating (near tilt limit) or very
                # close (LOS blowup): feeding a yaw command the mixer can't
                # honour is what diverged (log 172525: 46 deg bank + 64 deg crab
                # -> 2136 dps runaway). Hold the last heading instead.
                freeze_tilt = float(getattr(cfg, "YAW_FREEZE_TILT_DEG", 38.0))
                tilt_deg = (
                    float(np.degrees(np.hypot(pursuer_att[0], pursuer_att[1])))
                    if pursuer_att is not None else None
                )
                yaw_frozen = (
                    (freeze_tilt > 0.0 and tilt_deg is not None and tilt_deg > freeze_tilt)
                    or (min_range > 0.0 and float(intercept["range_m"]) < min_range)
                )
                if not yaw_frozen:
                    last_yaw_cmd = slew_angle_toward(
                        last_yaw_cmd,
                        desired_yaw,
                        float(getattr(cfg, "YAW_LOCK_MAX_RATE_DEG_S", 90.0)),
                        loop_dt_meas,
                    )

            # Acceleration feedforward: midcourse lead aid, CHASE only. Faded to
            # zero as the pursuer enters the terminal-extension band (adding
            # target accel there causes blow-through overshoot) and suppressed
            # once the terminal latch has frozen the aim.
            if (
                args.accel_feedforward
                and not args.position_only
                and recovery_state == MissRecoveryController.CHASE
                and not terminal_latched
            ):
                accel_scale = accel_ff_range_scale(
                    intercept["range_m"],
                    args.terminal_extend_range_m,
                    args.accel_ff_fade_band,
                )
                if accel_scale > 0.0:
                    accel_ff = accel_scale * target_feedforward_acceleration(
                        filtered_state, args.accel_ff_max
                    )

            send_local_position_target(
                pursuer_conn,
                boot_time,
                slot_pos,
                slot_vel,
                use_velocity=not args.position_only,
                yaw=last_yaw_cmd,
                use_yaw=args.yaw_lock,
                acceleration=accel_ff,
            )

            if yetki_client is not None:
                # Devir tohumu: goruntulu kod entegratorlerini bu son komutla
                # tohumlar. Yalniz komut gonderilen dongulerde tazelenir;
                # tuketici bayatligi t_mono ile ayirt eder.
                try:
                    yetki_client.set("devir_durumu", json.dumps({
                        "t_mono": loop_start,
                        "cmd_vel_ned": [float(slot_vel[0]), float(slot_vel[1]),
                                        float(slot_vel[2])],
                        "cmd_yaw_rad": float(last_yaw_cmd),
                        # OLCULEN GOVDE YAW HIZI (2026-08-04): goruntulu
                        # kontrolcu, bozucu artigindan KENDI donusumuzu
                        # cikarmak zorunda (d_ex = olculen aci hizi - kendi
                        # hareketimizin ongordugu). Devir aninda arac genelde
                        # DONUYOR ve karsi taraf yaw hizini bir dongu gec
                        # ogrendigi icin kendi donusumuzun bir kismi sahte
                        # "hedef hareketi" olarak bozucuya giriyordu. Bu bizim
                        # KENDI durumumuz; hedef telemetrisi kisitinin disinda.
                        "pursuer_yaw_rate_dps": (
                            float(np.degrees(pursuer_yaw_rate))
                            if pursuer_yaw_rate is not None else None
                        ),
                        "pursuer_pos_ned": [float(pursuer_pos_np[0]),
                                            float(pursuer_pos_np[1]),
                                            float(pursuer_pos_np[2])],
                        "pursuer_vel_ned": [float(pursuer_vel_np[0]),
                                            float(pursuer_vel_np[1]),
                                            float(pursuer_vel_np[2])],
                        "range_m": float(intercept["range_m"]),
                    }))
                except Exception:
                    pass

        if guidance_logger is not None:
            aff = np.zeros(3) if accel_ff is None else np.asarray(accel_ff, dtype=float)
            est_now = np.asarray(imm.x[0:3], dtype=float)
            meas_now = np.asarray(target_meas, dtype=float).reshape(3)
            guidance_logger.write(
                {
                    "wall_time": loop_start,
                    "range_m": intercept["range_m"],
                    "closing_velocity": intercept["closing_velocity"],
                    "t_go_s": intercept["t_go_s"],
                    "guidance_horizon_s": guidance_horizon_s,
                    "pursuer_x": pursuer_pos_np[0],
                    "pursuer_y": pursuer_pos_np[1],
                    "pursuer_z": pursuer_pos_np[2],
                    "meas_x": meas_now[0],
                    "meas_y": meas_now[1],
                    "meas_z": meas_now[2],
                    "est_x": est_now[0],
                    "est_y": est_now[1],
                    "est_z": est_now[2],
                    "slot_x": slot_pos[0],
                    "slot_y": slot_pos[1],
                    "slot_z": slot_pos[2],
                    "slot_vx": slot_vel[0],
                    "slot_vy": slot_vel[1],
                    "slot_vz": slot_vel[2],
                    "aff_x": aff[0],
                    "aff_y": aff[1],
                    "aff_z": aff[2],
                    "aff_mag": float(np.linalg.norm(aff)),
                    "extended": int(extended_terminal),
                    "latched": int(terminal_latched),
                    "latch_armed": int(terminal_latch_armed),
                    "recovery_state": recovery_state,
                    "z_limited": int(z_limited),
                    "aim_limited": int(aim_limited),
                    "alt_floored": int(alt_floored),
                    "lead_k_h": lead_k_h,
                    "lead_k_v": lead_k_v,
                    "mu_ct": mode_probs["ct_xy"],
                    "yaw_rate_dps": (
                        "" if pursuer_yaw_rate is None
                        else float(np.degrees(pursuer_yaw_rate))
                    ),
                    "alt_err_m": alt_error,
                    "cpa_range_m": "" if cpa_range_m is None else cpa_range_m,
                    "roll_deg": "" if pursuer_att is None else float(np.degrees(pursuer_att[0])),
                    "pitch_deg": "" if pursuer_att is None else float(np.degrees(pursuer_att[1])),
                    "yaw_deg": "" if pursuer_att is None else float(np.degrees(pursuer_att[2])),
                    "pursuer_vx": float(pursuer_vel_np[0]),
                    "pursuer_vy": float(pursuer_vel_np[1]),
                    "pursuer_vz": float(pursuer_vel_np[2]),
                    "yaw_frozen": int(yaw_frozen),
                    "cmd_lat_accel_mps2": float(cmd_lat_accel),
                    "turn_clamped": int(turn_clamped),
                    "safe_turn_vmax": (
                        "" if safe_turn_vmax == "" else float(safe_turn_vmax)
                    ),
                    "carrot_limited": int(carrot_limited),
                    "failsafe_active": int(failsafe.tripped),
                    "coast_age_s": float(coast_age),
                    "kill_mode": int(args.kill_mode),
                    "true_range_m": true_range_m,
                    "vibe_max": float(impact_status["vibe_max"]),
                    "hit_count": int(impact_status["hits"]),
                    "loop_dt_meas_s": float(loop_dt_meas),
                    "simtime_ratio": (
                        "" if not np.isfinite(simtime_ratio) else float(simtime_ratio)
                    ),
                }
            )

        if loop_start - last_command_print > 0.5:
            range_to_target = float(intercept["range_m"])
            range_to_slot = float(np.linalg.norm(slot_pos - pursuer_pos_np))
            tgo_val = float(intercept["t_go_s"])
            tgo_str = f"{tgo_val:5.2f}s" if np.isfinite(tgo_val) else "  inf"
            print(
                f"[{redcolor}simple_follow{endcolor}] range_target={bluecolor}{range_to_target:6.1f}m{endcolor} "
                f"range_slot={range_to_slot:6.1f}m "
                f"slot=({slot_pos[0]:.1f},{slot_pos[1]:.1f},{slot_pos[2]:.1f}) "
                f"tv={np.linalg.norm(filtered_state[3:6]):.1f}m/s "
                f"tgo={bluecolor}{tgo_str}{endcolor} vc={intercept['closing_velocity']:.1f}m/s "
                f"aff={0.0 if accel_ff is None else float(np.linalg.norm(accel_ff)):.1f}m/s2 "
                f"rcv={recovery_state} "
                f"pred={guidance_horizon_s:.2f}s ext={int(extended_terminal)} ext_m={terminal_extend_distance_m:.1f} "
                f"latch={int(terminal_latched)} arm={int(terminal_latch_armed)} "
                f"mu_xy=({mode_probs['cv_xy']:.2f},{mode_probs['ct_xy']:.2f},{mode_probs['ca_xy']:.2f}) "
                f"mu_z=({mode_probs['cv_z']:.2f},{mode_probs['ca_z']:.2f}) "
                f"raw_turn={turn_rate_estimator.raw_omega:+.3f}/{last_raw_turn_strength:.2f} "
                f"yaw_lock={int(args.yaw_lock)} "
                f"z_freeze={int(z_update_freeze_active)}:{z_update_freeze_remaining} "
                f"z_ct={mode_probs['ct_xy']:+.2f} "
                f"z_switch={int(z_switch)} z_fast={int(last_fast_turn_onset)} "
                f"z_slew={z_limited_by:+.2f}:{z_limit_reason or '-'} "
                f"gov=aim:{int(aim_limited)},floor:{int(alt_floored)} "
                f"leadk=({lead_k_h:.2f},{lead_k_v:.2f}) "
                f"loop={1.0 / max(loop_dt_meas, 1e-6):4.1f}Hz "
                f"sim={'--' if not np.isfinite(simtime_ratio) else f'{simtime_ratio:.2f}'}"
            )
            last_command_print = loop_start

        elapsed = time.monotonic() - loop_start
        if elapsed < loop_dt:
            time.sleep(loop_dt - elapsed)

        # Push telemetry to GUI using this loop's aligned snapshot: target_meas
        # is the measurement imm.x was updated with, so est - meas is the true
        # filter residual (cm). Re-reading the target here would catch a newer
        # background-thread packet than imm.x reflects, injecting ~1 packet of
        # target motion (several metres) of phantom high-frequency plot error.
        push_snapshot(
            pursuer_pos=pursuer_pos_np,
            target_pos=np.asarray(target_meas, dtype=float).reshape(3),
            mode_probs=mode_probs,
            target_est=np.array(imm.x[0:3]),
            status={
                "range_m": float(intercept["range_m"]),
                "t_go_s": float(intercept["t_go_s"]),
                "closing_velocity": float(intercept["closing_velocity"]),
                "recovery_state": recovery_state,
                "extended": bool(extended_terminal),
                "latched": bool(terminal_latched),
                "hits": int(impact_status["hits"]),
                "crashed": bool(impact_status["crashed"]),
                "vibe_max": float(impact_status["vibe_max"]),
                "true_range_m": true_range_m,
            },
            diag={
                "omega": last_omega,
                "innov_norm": last_innov_norm,
                "jump_norm": last_jump_norm,
                "err_norm": last_err_norm,
            },
        )
        gui_tick()

    # Hand the vehicle to a self-stabilising mode before we stop commanding it.
    stabilize_on_exit("loop ended")
    if gui is not None:
        gui.stop()
    if guidance_logger is not None:
        guidance_logger.close()
        print(f"[{redcolor}simple_follow{endcolor}] Guidance log written: {guidance_logger.path}")
    print(f"[{redcolor}simple_follow{endcolor}] Stopped")


if __name__ == "__main__":
    try:
        main()
    finally:
        # Backstop: covers an exception or a Ctrl-C caught outside the loop.
        # No-op if the normal path already ran it.
        stabilize_on_exit("interpreter exit")
