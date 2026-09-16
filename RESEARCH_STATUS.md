# AUTORESEARCH STATUS — single LOS/PN hardware candidate (closing 2026-08-11)

> **This is the file transfer document.** A new agent/session can take over from here. Experiment backlog and individual item records: `TO_TEST.md` (item 19-27 of this campaign). Running procedure: `MPC_NOTES.md`. This file is a summary + priority + trap list ABOVE THEM.

## 0. 2026-08-11 CURRENT ARCHITECT DECISION

The current main direction is **positioned back-slot approach + rotation with only display + law with single display LOS/PN**. MPC was not deleted, but no longer runs on the default hardware path or visual phase; only historical A/B/fallback remains with `--guidance-value mpc`.

Turnover rule: successive **5 fresh bbox**, each with at least `%3×%3 = 829 px²` in frame 1280×720. Hub and target telemetry is not sought at the gateway. When YOLO is used, the counter only sees the frames that the detector accepts because the `--yolo-conf` filter runs before broadcasting the box to Redis.

RoboFly/ellipse live results:

| arm | engagement | actual CPA median/p90 | `<5 m` | `impact_successful` |
|---|---:|---:|---:|---:|
| LOS `N=4`, area `%3`, 5 large square | 6 | **1.09 / 2.03 m** | **6/6** | **2/6** |
| LOS `N=5`, same door | 12 | 1.65 / 3.76 m | 11/12 | 0/12 |
| previous pure LOS, without gate/early cycle | 19 | 1.95 / 39.13 m | 15/19 | 0/19 |

N=4 produced two independent initial contacts and crashed the vehicle (`vibe=369.5, r=0.87 m` and `vibe=95.2, r=0.52 m`). Lines starting after contact were removed from the performance pool. The only heavy miss in the N=5 branch is `8.94 m`. So the defaults are frozen as `N=4`, `strike_acceleration=4 m/s²`, `large_frame=5`, `area_pct=3`.

`competition/gimbal_bench_tracking.py`, `mavlink_tilt.py` and `monitor_mpc_commands.py`, which worked on the hardware, were referenced. IMX500 1280×720/180° rotation, real HSV band, moment center, servo ramp and `DO_MOUNT_CONTROL` moved to main tools. The mere observer can now run `--law los` and actual LOS output without driving the vehicle. Current single-line commands: `LOS_HARDWARE_NOTES.md`.

Important limit: **pass** is independent of telemetry range; while the current LOS/PN still uses `range_m_value` for gain PN, `t_go`, vertical resolution and dropout. bbox scale/optical magnification or independent distance sensor fusion for fully telemetry-free terminal flight is next research. ArduPilot already converts the lateral velocity request into roll/pitch; yaw-rate retains only horizontal vision and can be tested separately with the new `--no-yaw` tonearm, but contact generator is on by default.

## 0b. 2026-08-10 HISTORICAL HYBRID DECISION

As a result of live ellipse A/B, the main direction was frozen as **positioned approximation + mid-phase MPC + terminal PN/LOS hybrid**:

- Positioned `simple_guided_follow.py` establishes the slot behind the target and creates visual turnover conditions.
- After display handover, `hybrid_guidance.py` keeps MPC as FOV and constraint scheduler while `R>18 m`; `R<=18 m` switches to terminal law `terminal_los_guidance.py` PN/LOS.
- Both outputs are projected from the actual speed to the achievable command in terms of acceleration/direction. Vertical speed is also at the limit of jerk and absolute speed.

The pure MPC in the live base produced CPA median 11.20 m and 59 produced `<5 m` in only 3 of the passage, while the safe hybrid produced 3.22/3.42/4.00 m in three passages. `R<8 m` command reversal 171.6° decreased from median to 5.35°, `>90°` ratio decreased from %91.4 to 0/103. But there is no physical contact yet; three `ALTITUDE ABORT`s left in safe hybrid. Therefore, the next goal is not "closer estimation" but safe and repeatable actual contact. Detail: `TO_TEST.md`, **LIVE T1 ASSESSMENT**.

## 1. HISTORICAL PURE-MPC CHAMPION

```bash
DURATION=360 RESTART=0 VISUAL_GUIDANCE="mpc_guidance.py" \
PLAN=missions/target_ellipse.plan METHOD=<label> YILDIZ_VIDEO=1 \
YILDIZ_HOLD_YAW=1 YILDIZ_ACTUATOR=1 YILDIZ_VERTICAL_ERROR=1 YILDIZ_VERTICAL_TGO=2 \
YILDIZ_BLIND_PN=1 tools/scenario.sh
```

The above five proof levers (`HOLD_YAW`, `ACTUATOR`, `VERTICAL_ERROR=1`, `VERTICAL_TGO=2`, `BLIND_PN`) since 2026-08-10 **default on in bare operation**; Writing them explicitly in the command only keeps the experiment record readable. Each can be switched off individually with the corresponding override `YILDIZ_*=0`.

| flag | what does | evidence |
|---|---|---|
| `YILDIZ_HOLD_YAW=1` | detection `hold_value` yaw takes time when in position | closing on return +2.66→+3.59, opening %40→%18.5 |
| `YILDIZ_ACTUATOR=1` | horizontal actuator saturation model (tau_lin 1.7 s, a_max 4) | unreachable command %40→%11-15, cost +%3.5 |
| `YILDIZ_VERTICAL_ERROR=1` + `YILDIZ_VERTICAL_TGO=2` | P + t_go-shaped vertical arm (**note: env value is NOT k, it is MULTIPLIED by k** → 2 if you give k=4) | k=4 lever gave best contact run of the day (minR p50 5.34) |
| `YILDIZ_BLIND_PN=1` | on blind squares continue the LAW, not the command (dead reckoning + solver runs) | solver running on 100% of blind frames; CPA p50 5.99→3.50 in common layer |

Defaults to (no flag required): five-proof branch in the table; blind-PN clamp 26°, range gate 12 m, HOLD altitude gate and `RECOVERY_HOLD_MAX_S`=8 p.

## 2. FIXED IN THIS CAMPAIGN

| # | correction | evidence |
|---|---|---|
| 1 | **tut-yaw sustain** — main evasion loop on return broken | yaw command was not going at all in %40 of return frames |
| 2 | **Saturated actuator (horizontal)** — model fitted to reality | measured: horizontal tau 1.7 s / plateau 4 m/s²; model 1.0 thought s+unlimited |
| 3 | **BLIND_PN + R3′ clamp (26°) + range gate** | the raw version was regression on rotation (imaginary target out of frame); handcuffs closed |
| 4 | **HOLD altitude gate** (tumble pathology) | somersault 3 running → 0; alt_err p95 56→23, >15 m ratio %86-91→%15 |
| 5 | **Three silent-decomposition gates** | `plan_alignment` (two "straight" runs actually flew ellipse), `tilt_alignment` (9° chain separation), prearm health bit |
| 6 | **Operational** | BRAKE shutdown (POSHOLD), EEPROM clear (gyro-play), measured-aggressive parameter profile |

**Rejects (all with flight):** APN derivative term (noise), vertical saturation (abort ↑), vertical ramp (no-op — ramp already saturated at rev ~25 m), advance clock (gain 0.00 m, in two scenarios), DOWN=0 (dz<0 not broken), intercept approach (cannot recover visual phase overshoot). **Damage blocked:** `MOT_THST_HOVER=0.68` assert (true love veteran 0.28-0.35; would inflate feedforward 2×).

## 3. OPEN PROBLEMS — ORDER OF PRIORITY

> **2026-08-10 live T0 changed this order.** Detailed proof and experimental descriptions `TO_TEST.md` item 28–33. The following sequence replaces the previous P0-vertical provision.

### P0 — Safe, repeatable physical contact
Terminal speed reversal was covered by the common availability layer of the hybrid arm. The new bottleneck is to convert 3.22–4.00 m real CPA to vibration/contact verified impact. `N`, rev range and STRIKE speed difference will be factorial controlled; referee `ref_range_ground_truth_m + vibe/impact_successful`. The altitude deviation of the first hybrid `cmd_vz=-9 m/s` and 78.9 m has been corrected, but the contact gain will not be taken at the expense of safety, as three `ALTITUDE ABORT` remain in the safe run.

### P1 — Arc/constant-turn target model
Hard rotation CPA median **13.36 m**, smooth phase **9.96 m**; no `<5 m` in hard turning. APN acceleration is 0 in all live run. Raw derivative will not be reopened: MINRECT signal will be verified in two directions and trust-gated CT/IMM propagation and PN feed-forward will be attempted.

### P2 — Back-cone gate and speed program
The system is already in the back quadrant at R<20 (median angle of approach is 160°), but at R<8 the setpoint reverses, degrading to 149°. `SETTLE` (LOS speed/lateral relative speed reset) and `STRIKE` acceleration only at the stable `>=165°` tail cone will be separate phases.

### P3 — Vertical branch/rise damping
P+TGO CPA reduced the vertical residue; It is no longer the primary cause of misses. However, in 23 CPA's 20 in the previous 5 s, vel_z changed sign at least twice; 8–15 m vertical rail occupancy %28.7. dz/dz_dot critical end + hysteresis + jerk/slew A/B.

### P4 — Remaining work
- GUI budget-cut ~%90: every admission prompts headless replay.
- Sub-meter tail (TO_TEST.26), uncontrolled yaw in recovery (TO_TEST.27) and official contact meter remain open.
- Direct roll/attitude command is not the first option; Roll is produced by passing the desired lateral acceleration through the ArduPilot velocity/acceleration layer.

## 4. MEASUREMENT METHODOLOGY (results that do not comply with these are invalid)

1. **minR is NOT a single-run referee.** Intra-run SD 0.91 (log) vs inter-run 0.23; IOC 0.06. For arm difference you need 6-8 running/arm. **Reliable endpoints (1-2 running sufficient): flat-phase fresh%, timeout%, roll RMS.**
2. **The unit of analysis is ENGAGEMENT**, the running block. In arm comparison, the cycle geometry balance (closing0/R0/aspect0 medians) is always reported; If it is unbalanced, the syllogism is invalid.
3. **Layer:** `kapanma0>=3 m/s` and `<3` are read separately. The strongest predictor of turnover instantaneous closure rate minR (Spearman −0.56) — MPC preserves existing closure, does not create it.
4. **GUI comment:** GUI running throttles the solver (p95 21.8 vs headless 13.7, `budget_cut` %88 vs %75). Read the GUI run against the GUI base.
5. **Measure traps:** `los_speed_az` is NOT the residual PN (uses commanded u2 — `ref_bearing_deg` derivative for real); `|u3|` does not measure vertical demand (LOS plane, horizontal share grows with eps — use `cmd_vz`); `summary_value.txt` "cumulative detected %" not comparable across runs (bbox.log accumulates — use column CSV `state_value`); `ref_target_ax/ay` IS DEFECTIVE.

## 5. OPERATIONAL TRAPS (all burned in this campaign)

| trap | symptom | solution |
|---|---|---|
| **PLAN is loaded only on restart** | While `RESTART=0` PLAN is silently disabled; target flies the old plan, writes "straight route" on the screen again | Gate `plan_alignment` now stops; remove stack with `YILDIZ_TARGET_PLAN=... ` for straight running |
| **VIDEO only opens on restart** | `scenario.sh` exports `YILDIZ_VIDEO=1` only in its restart branch; **no video recorded** on manually removed stack (none of the runs 2026-08-09 were recorded) | Remove stack with **`YILDIZ_VIDEO=1 ./yildizlar_guidance.sh --headless`** |
| **Parameter authority** | writing file/EEPROM/MAVLink is invalid | `guidance_config.GUIDED_STARTUP_PARAM_ASSERTS` crushes every run — **that's the real authority** |
| **EEPROM corruption** | "Gyros not calibrated" on fresh batch drops to EKF3 DCM | `rm run/sitl0/eeprom.bin` every refresh |
| **Arm preparation** | PreArm message silence is misleading (ArduPilot restricts) | `SYS_STATUS` `MAV_SYS_STATUS_PREARM_CHECK` health bit |
| **Tilt chain** | Giving DOWN to the scenario alone separates the camera/MPC reference 9° | `tilt_alignment` gate; DOWN change stack + scenario both |
| **BIN↔CSV clock offset** | dataflash unix time 0.65-3.2 s ahead of wall clock | cut offset to running press; use same-row CSV columns if possible |

## 6. STANDARD RUNNING PROCEDURE

```bash
# 1) stack (headless + video + fresh EEPROM)
./yildizlar_guidance.sh --stop ; sleep 3 ; rm -f run/sitl0/eeprom.bin
YILDIZ_VIDEO=1 ./yildizlar_guidance.sh --headless
#    for straight route: + YILDIZ_TARGET_PLAN=missions/target_straight.plan
#    For route S: + YILDIZ_TARGET_PLAN=missions/target_s.plan
#    for arc detection:+ YILDIZ_MINRECT=1

# 2) doors (pre-run): prearm health bit tools/plan_alignment.py tools/tilt_alignment.py

# 3) trial (champion configuration — section 1)
```

**Data locations:** running folders `run/trials/<method>_<route>_<timestamp>/` (summary.txt, guidance.log, display.log, bbox.log) · CSVs `guidance_allstar/logs/visual_mpc_<timestamp>.csv` + `mpc_diagnostic_<timestamp>.csv` + `guided_follow_<timestamp>.csv` · videos `videos/<label>_<timestamp>.mp4` · dataflash `run/sitl0/logs/*.BIN` (hunter), `run/sitl5/logs/*.BIN` (target).

## 7. TOP THREE JOBS FOR NEW AGENT

1. **Factorial terminal scan:** `N={4,5,6}` × revolution `{18,20 m}` × climb limit `{1.5,2.0 m/s}`. At least six engagements in each cell; CPA, `vibe/impact_successful`, `ALTITUDE ABORT`, freshness and cycle geometry are reported together.
2. **STRIKE/DON last meter setting:** scan the VAT off difference and the DON/t_go release threshold only if physical contact is increasing. The second head-on attack or vertical overflow after the first pass is considered regression.
3. **Constant-turn target model:** verify MINRECT right/left flag after hybrid terminal base is fixed; Try trust-gated CT/IMM propagation first in the middle phase of `R>18 m` MPC. Do not feed raw bbox derivatives to terminal PN.
