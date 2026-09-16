# CONTINUE — pending jobs (update 2026-08-05 night, after the target-wiring revision)

> **SIDE BRANCH: the archived ArduPilot FOLLOW configuration** (2026-08-05 at noon). Instead of MPC, ArduPilot's own tracking law (estimator `mode_follow.cpp` + `AP_Follow` + `AC_PrecLand`) was tried. Details and justifications: **`ARDUPILOT_TRACKING.md`**. Sim result (same environment, same skeleton, same miss measure): pendant **0.5 m** (equal to MPC 0.5) / infinite **1.2 m** (MPC tur-4 1.3) / ellipse **0.2 m** (best in table) / wanderer 5.5 m (MPC 4.9-6.7). Cycle cost 43 µs (MPC solver p95 13 ms). Its weak point: it has no frame-saving terms — in fast geometry `ex_rms` 11-19, yaw lags behind. The decision to merge was not made; PrecLand EKF in §7 should be tried first.

This file is the handover note: which job is waiting and why, in what order it should be done, and concrete acceptance criteria for each. For context, `MPC_NOTES.md` (running the environment) and `guidance_allstar/LOG_DICTIONARY.md` (reading logs) should be read first.

## Status summary

- **The winning method is MPC.** LOS and PID are frozen benchmark artifacts.
- Tests: `cd guidance_allstar && python3 mpc_test.py && python3 los_test.py && python3 pid_test.py` → **67/67, 66/66, 51/51** (mpc 48 → 67: 35 added m/s tour + STRIKE phase tests, see item 1).
- Sim geometry: **mount 0°**, standoff **back 25 / down 4**.
  > **[GIMBAL BRANCH UPDATE 2026-08-05]** In the `gimbal` branch "mount" is no longer a setting: on the camera self-stabilizing **physical single-axis (tilt) gimbal**, the SDF mount pitch 0 remains. **TILT** determines the vertical axis: `YILDIZ_TILT = atan(down/back)` (earth elevation, + = up, `scripts/standoff_geom.sh`). Measured in flight: camera pitch relative to the world max |0.65°| with fuselage rolling −35.4…+35.2°. Vehicle: `tools/set_tilt.py`. Detail: `GIMBAL_NOTES.md`.
- Speed ceiling frame 35 m/s; **MPC now derives from the same source** (2026-08-05, item 1 — pending sim run).
- Records: static ground target **0.92 m**; hanging target (drone_2) **0.47 m** (`videos/mpcasili2_ellipse_20260805_031421.mp4`, but see hanging target note).

## COMPLETED (night session 2026-08-05)

### MISS frame connection — DONE
The three parts of the old item 1 were connected exactly + the `release_waiting` lock was added (fake reacquisition occurred in the 1-37 ms window between the miss announcement and `bbox_to_redis` dialing `command_authority`; 9 until the real handover 17 `handoff_received`). Acceptance run (ellipse, 360 s): 9 MISS, CPA→release median **7.0 s** (formerly 40-77 s), authority segment **4 → 9**, 9/9 from switch `visual_release` (rejection monitor proof). Lock verification in target_infinity run: 8 inheritance = 8 segment, zero pseudo inheritance.

### target_infinity run — DONE, critical finding
`run/trials/mpc_infinity_20260805_022808/`. In the near band (<30 m), **%100 of encounters were tail pursuits**. In target_straight, %30.6 were head-on, confirming that the earlier "straight route" results were not pure tail pursuit. **CRITICAL:** MPC has no positive closing speed in pure tail pursuit. The target moves at 21.05 m/s and `MpcConfig.speed_ceiling_mps=18`, giving approximately −3 m/s closing speed. All 7/7 misses were classified as "range increasing", with the closest pass limited to about 21 m. The well-known 4.66 m pass in target_straight was also found to have head-on geometry. **Item 2 now has a measured, specific cause.** In addition, the FOV constraint is active in %96.8 of cycles and solver p95 is 13.3 ms, exceeding its 12.7 ms budget. Both must be checked in the 35 m/s round.

### Hanging target wiring — DONE, run done
`TARGET_VEHICLE=drone2` env key (`guidance_config.py` + `scenario.sh`); When empty the behavior is exactly the same as the old one. drone_2 ports 14662/14664, sysid 2. Running: 6 transition, min **0.47 m**, median 1.25 m; purple body detection %81.9. **WARNINGS (just so you know):** (1) purple target parks on the GROUND near the camera before takeoff → purple block covering the screen in the first frames of the video (dev %72.8) — scenario artifact, not a glitch; (2) dual SITL on the same machine `simtime_ratio ~0.44` → video 13 fps, camera is NOT fluent; Numbers indicate direction and are not considered exact metrics. Improvement ideas: spawn target far / start recording after takeoff / do not start plane SITL at all while `TARGET_VEHICLE=drone2` (requests key to yildizlar_guidance.sh). Also, on a stationary target, a miss is often declared with a timeout of 15 s; `miss_opening_m` must be remeasured for the hanging target.

### Pattern A/B — CLOSED (no sim required, with code reading)
Answer: **There are no two types to compare.** The FORMATION phase of the `formation_KILLER.py` does not include homing (its input is the leader drone); In that architecture, `simple_guided_follow_shaykh.py` chases the target and its guidance law is exactly the same as our `simple_guided_follow.py` at the AST level (26/45 symbol is identical, including `BehindSlotGuidance`). The only difference is that the transport layer (MAVLink↔Redis) + shaykh did NOT receive the `a5a28eb` turn-clamp patch (i.e. arm B = old version of A with 678 m circle error). Also formation_KILLER ATTACK law derives target speed VECTOR → violation of the "range only" rule; If a reference will be given in the article, a note should be made. `formation_KILLER.py` is also inoperable (7 missing dependency: no config.py/controller.json/waypoints.json etc.).

---

## 1. Round of moving MPC to 35 m/s + HIT PHASE — CODE/TEST OVER, SIM WAITING

Done (drive side only; environment/plan/param files left untouched):
- `MpcConfig.speed_ceiling_mps` now **derives** from `guidance_config.VISUAL_MAX_SPEED_MPS` (`mpc_guidance.environment_speed_ceiling`) → 35 m/s cannot be decomposed again. Offline: pure tail (target 21.05 m/s) **30 m → 1.6 m COLLISION**, touching the ceiling %73-98. Same scenario 18 at m/s 30.00 m (does not close at all) — exact equivalent to the target_infinity finding.
- **STRIKE PHASE**: `CLOSURE → TERMINAL(45 m) → IMPACT(22 m) → MISS`. A continuous blend, equal to 0 at 22 m and 1 at 8 m, opens the FOV bands to the physical edge (14/17.5 → 19/19, ex 26 → 31), removes the band narrowing caused by braking or acceleration, doubles the area reward (2x), and scales `q_acceleration` by 0.35x. Measured under the same conditions: closing command **u1 27.3 → 32.3 m/s**. The field-of-view/PN weights deliberately remain at 1.0. Increasing them led to pure pursuit in the measurements. See the impact_* block in `MpcConfig` for the rationale.
- `step_s` 0.18 → 0.12 (horizon 3.6 → 2.4 s): 35 at m/s 3.6 s horizon 126 at m, engagement envelope 60 m. Measured: Framing loss 37% on 0.18, 27% on 0.12.
- `fov_climb_demand_ceiling_mps` 5 → 9 and `q_ey` 0.35 → 0.60: the dominant channel of framing losses at high ceiling is VERTICAL (at the moment of loss at pure tail |ex| max 1.1). Both missing %26.9 → %23.4, min range median 9.63 → 8.51 m.
- The miss thresholds were re-established: `miss_transition_arm_m` 20 → **12**, because GEOMETRY now distinguishes the cases. Actual passes had CPA ≤ 11 m, whereas the oscillation case had CPA 18.1 m. `transition_closure_threshold_mps` 15 → **10**, because attainable closing speed in tail pursuit was 35−21.05 = 13.9 m/s. The old threshold was mathematically unreachable, consistent with the measured 0/16 triggers. `miss_time_timeout_s` 15 → **8**.
- **Design decision made:** ishkada "give up authority" RETAINED (after the switch fixed camera cannot see the target except 0, MPC remains blind — **[GIMBAL BRANCH UPDATE: this justification has been refuted in vertical; the camera is now in the tilt gimbal and can look up. After the switch the main thing that makes the target lose is YAW (gimbal single axis) — decision stands, reason shifted to yaw. "Release authority" should be measured again in the gimbaled tour condition.]**) but the glide is now **BRAKE**: decreasing from 3 m/s² to 12 m/s (turning radius 245 → 29 m). This is how the benefit of "slow down and freeze" was taken away from common file swapping.

### SIM TYPE-1 RESULT (target_infinity, recorded revision) and CIRCUIT-2 CORRECTION

Lap-1 won: speed parity fully worked (u1 max 35.00, cmd p95 34.78, shutdown −18.4 m/s), STRIK phase %93.8 active in the loop, **21 m wall broken** (CPA min 20.48→2.36, median 21.65→5.84, 10/12 transition ≤10 m). Solver p95 13.26 ms (unchanged).

Lap-1 defect: **sub-meter hit 0/12** and the last 0.2-1.3 s of the terminal phase are BLIND on EVERY pass. Root cause measured: forward acceleration tilts nose down (5.84°/(m/s²)), FIXED 0° camera goes down with it, target already ABOVE axis on mount 0 comes off the top edge. **[GIMBAL BRANCH UPDATE 2026-08-05: the causal chain behind this failure is broken. The "camera descends with the nose" ring broke with the physical tilt gimbal (camera max 0.65° while the fuselage is ±35°). The following eng-1/tur-2 numbers were measured in the BODY-FIXED period; `pitch rate` column to frame conversion (°/s → px/s) INVALID in gimbaled setup. The gimbaled equivalent of top-edge loss is tilt error and must be measured again. Additionally, the tour-2 countermeasures taken against this root cause (`impact_acceleration_multiplier`, rev acceleration ramp, forward acceleration clamp) may now be solving ANOTHER problem — whether they can be lifted in the gimbaled tour should be measured.]** %18.5 of losses upper / %6.0 lower (3:1), of which **%10.9 is outside the physical FOV** (beta < −20.07) — untaping cannot recover that part. Side effect: |pitch speed| median 3.6→13.3 °/s (~950 px/s at frame = user-complaint shake), frame loss %8.6→%48.3, turnover transient pitch at first 3 s −20.5°, fresh-not cycle 2→261.

Lap-2 correction (MPC side only, measured offline — straight course, target 21.05 m/s, 3 rev × 2 seed):

| arm | pitch speed | top edge % | % out of physical FOV | min range | cmd p95 |
|---|---|---|---|---|---|
| tour-1 (`impact_acceleration` 0.35) | 3.81 | 12.0 | 14.0 | 1.93 | 35.0 |
| **eng-2 (1.0 + rev ramp)** | **1.83** | **10.5** | **12.4** | **2.13** | **35.0** |

- `impact_acceleration_multiplier` **0.35 → 1.0**: HIT no longer reduces the acceleration penalty. Speed parity alone was touching the ceiling; The extra acceleration authority was not free, it was paid with the camera axis. (The intermediate value 0.7 was also measured: 2.74 °/s.)
- **Speed acceleration ramp** (new): At speed `q_acceleration` ×3.0 is vented to 1.0 with τ=2 s. 5.0/2.5 s tried → min range 12.27 m (kills shutdown); 2.0/1.5 s → no gain.
- **Speed-rise clamp ELIMINATED by measurement** (`forward_acceleration_ceiling_mps2` default 0.0): did not correct pitch, killed shutdown (2 m/s² → min range 1.93→11.17 m). Reason: ArduPilot velocity loop tries to close the setpoint difference at ~0.25 s, even a difference of 1.25 m/s saturates the acceleration to 5 m/s² — clamping the SIZE of the instruction does not make that difference smaller, it is the PENALTY on (u−w) that makes it smaller.
- `impact_ey_multiplier` 2.0 tried → neutral (1.83→1.88 °/s, min range 2.13→2.09). Left at 1.0.

**Remainder open:** non-physical FOV portion of top-edge loss (~%12) IS GEOMETRIC — standoff BELOW target (down 4-6 m), growing eps = down/r as range closes. Tried PURE GUIDANCE solution on Type-3 (below).

### SIM CIRCUIT-2 RESULT and CIRCUIT-3 CORRECTION

Tur-2 in sim **FIRST sub-meter hit arrived**: CPA min 2.36 → 0.84 m, ≤1m 0/12 → 2/16 — entirely from `impact_acceleration` 0.35→1.0 (deep terminal). Parity was preserved (u1 max 34.96, now 0.995).

Tur-2 defects and tur-3 resolutions:
- **RPM acceleration ramp REVERSED**: net detriment in sim — halved the shutdown (cover range_speed −3.11 → −1.40 m/s, strong shutdown ≤−10 m/s %7.0 → %2.9), did not fix the rpm transient either (fresh-not 261 → 269). `handoff_acceleration_multiplier`/`_acceleration_multiplier`/ `_authority_t0` ramp has been removed.
- **Quiver targeted by (b), retained (c)**: pitch speed in sim 13.32 → 14.12 (offline −%52 claim debunked — acceleration-clamped, `impact_acceleration` 1.0 increasing body acceleration). The user said "shake the shiver, crash first"; `impact_acceleration` 1.0 REMAINED.
- **TERMINAL VERTICAL ALIGNMENT** (new, original idea — `impact_alignment_*`): Climb bias is added to the vertical LOS-speed reference when the target apparent rise (eps) on STRIKE exceeds the COMFORT band (10°) → the fighter climbs the target line and melts the standoff, the camera becomes level, the target does not come out of the top edge. Pure geometry (eps=f(ey only), no telemetry), one-sided (no forced descent), with deadzone (eps≤10° does nothing → no closing cost).

**IMPORTANT — OFFLINE/SIM GAP**: this mechanism CANNOT BE MEASURED IN THE OFFLINE CLOSED LOOP. I measured: the emulation enters the terminal with **eps<0** (the target is already flattened: −1.6…−5.0°), because the vertical dynamics of the offline engine is not the same as that of the sim — in the sim, the fighter in standoff REMAINS at the bottom ~4m (from there the top-edge loss), in offline it is aligned up to the terminal climbing. So the mechanism remains in the deadzone in all offline scenarios (bias 0, NO regression risk) and has been tested **deterministically at the solver level** (5p): in eps=19° the CLOSED arm descends (u3 +0.14, worsens the top edge) while the OPEN arm climbs (u3 −0.46). **Closed loop authentication belongs to SIM.** This is the exact offline→sim separation you pointed out in genre-2.

### SIM CIRCUIT-3 RESULT and CIRCUIT-4 CORRECTION

**REAL CRASH HAPPENED.** CPA 0.62 m (scenario record; type-2 0.84, tour-1 2.36). Evidence physical: 2.32 vibe at m 2.0→17.4, 1.04 at m 25.5, then somersault (roll −106°, pitch −51.8°) — at that moment the steering was in blind glide (u1=u3=0), that is, the tumble command is not a CONTACT command. In Tur-2 the vibe on 0.85 m pass was only 3.3 (we didn't touch).

Vertical alignment verified in closed loop: `alignment_ref` 39 frame active, all positive (one-sidedness retained), u3 +1.53 → −3.72; **Vertical standoff on CPA 0.81 m → 0.05 m** (standoff actually melted). Left the frame late (last-fresh range 22.3 → 8.1 m, blind time 2.72 → 0.85 s), flicker dropped (pitch speed p50 14.12 → 10.93), bottom-edge NOT DISTORTED (39 of active frame 0), NO closing fee (active u1 20.31 vs passive 17.92).

Unhit target: top-edge non-physical %16.26 (tur-2 %16.24) — never fell, because the mechanism only opened in **%8.1** of the terminal frames (eps p50 4.6°, p95 13.6° in HIT; 10° leaves out ~%90 of deadzone frames).

Tur-4 decisions:
- **`impact_alignment_relaxed_deg` 10 → 5** (SINGLE knob). Active window ~%8 → ~%45. `ceiling_value`/`tau` deliberately NOT changed: solver scanning showed that the ceiling is not binding in this band (ref = (eps−5)/1.5 ≤ 5.7 dps, i.e. 6/8/10 same solution) and tau 2.0's u1 save is not needed in the sim (the cost is already measured as zero). Single knob = next run interpretable.
- **HIT SUCCESS DETECTION** (measurement problem): `vibe > 15` AND **measured** range `< 3 m` → event `impact_successful` + latch column `hit_value`. Now the innings count is read directly from the run summary; There is no need for the guess CPA and the constraint "n=5 since every hit ends the run".

**THINGS TO LOOK FOR IN THE SIM RUN** (column → hypothesis):
1. `cmd_speed_mps` / `clamp_speed` → are we really touching the ceiling (in target_infinity it was %0). If not, check if the vertical ceiling connects `u1`, `vz_*_cbf` and `cmd_vz`.
2. `mpc_diagnostic.state_value` + new `impact_value` column → Is the STRIKE phase entered, does the mixture reach 1 (i.e. does 8 enter into m).
3. `beta` vs new `band_upper` → loss from which edge. Hypothesis: upper edge (forward acceleration tilts nose down).
4. `duration_ms=0` lines → blind glide; design if at STRIKE and < 1 s.
5. `_event.csv` miss lines + `range_rate_value` → new 12 m / 10 m/s toggle arm does it actually fire in tail following (formerly 0/16).
6. **`pitch_deg` variant** (consecutive row difference / `dt`): median in lap-1 was 13.3 °/s. Target < 7 °/s. This is a direct measure of visual jitter (1 °/s ≈ 17 px/s framing shift).
7. **`mpc_diagnostic.acceleration_multiply`** (new column): should be 3.0 in the cycle, decreasing to 1.0 in ~4 s. If it does not descend, the ramp is stuck; If it is always 1.0, the ramp did not work at all (`_authority_t0` may not have been seeded).
8. First 3 s at the time of transfer: `framing_edge_px`, `pitch_deg` min, `state_value=fresh_value` rate. In tour-1, fresh-not cycle 2 → 261, pitch min −20.5°. If the ramp is working, the pitch min should go up to around ~−12°.
9. Top/bottom edge loss ratio and `beta < −20.07` percentage: %15.8/... and %10.9 (6:1) in lap-2. If terminal vertical alignment is working, the top margin should decrease AND the bottom-edge loss should not increase (deadzone + single-sided should prevent it — but verify on SIM, I couldn't test it offline).
10. **`mpc_diagnostic.alignment_ref`** (new, type-3): Must be positive when `eps>10°` at the STRIKE terminal (0-8 dps). **If it always stays 0**, the fighter is already aligned (like offline) — then the top-edge loss is due to something out of alignment (pure pitch jitter) and the mechanism is running amok; Read with `pitch_deg` min.
11. Closing cost of terminal vertical alignment: `u1`/`cmd_speed_mps` falling on `alignment_ref` rising frames? In Type-3, the cost was measured as ZERO (active 20.31 vs passive 17.92). If it breaks, raise `impact_alignment_relaxed_deg` or downgrade `impact_alignment_ceiling_dps`.
12. **CIRCUIT-4 MAIN ACCEPTANCE CRITERIA**: `alignment_ref > 0` frame rate must be %8 → **~%45** (deadzone 10→5). Hence: the upper-edge must remain non-physical %16.3 → **<%12** AND the lower-edge must remain non-physical **<%4.5**. If the bottom-edge increases, the deadzone is too narrow → pull `relaxed`i to 6-7.
13. **`impact_successful` event** (`_event.csv`) + `mpc_diagnostic.hit_value` column: number of hits = `hit_value` 0→1 number of passes = number of event rows. If both do not match, there is a latch/reset error. Read with column `vibe`: contact must be in band 17-26; **>If 150 and altitude~0, GROUND contact**, not hit (range gate must have eliminated).

## 2. small open items

- `bbox.log` no timestamp — unix stamp on line `TARGET center=` (public file, **pending user approval**).
- `t_unix` to `mpc_diagnostic` CSV: added once, tested (48/48), RETURNED by user request (2026-08-05). If requested again, the change to be made is known: Header in `_diagnostic_write` + `time.time()` at the end of the line.
- `mpc_test` derives `YILDIZ_DOWN` default 6.0, `standoff_geom.sh` derives 4. `YILDIZ_DOWN=4`te, 5c and 5i fall (also on clean base) → belongs to standoff change, must be rebased. 5c is significant: at down=4, the hard constraint FOV loses its superiority over the soft one.

---

## Working rules (must be repeated when giving to agents)

1. **Only RANGE is used from target telemetry.** Deriving target speed/direction/acceleration PROHIBITED. Logging is free, but with the prefix `ref_` and AFTER the command is sent.
2. **Common files are left untouched while the Sim is running.**
3. **One simulation at a time**; The orchestrator orders the runs.
4. `scenario.sh` `temizle()` `python3 *_guidance.py` kills patterned processes; This pattern MUST NOT be passed in the command line of the wrapper shell (exit 144).
5. The installation angle is not the only button: use `tools/set_mounting.py`. **[GIMBAL BRANCH UPDATE: There is no longer a button called "mounting angle". For vertical geometry use `tools/set_tilt.py --down .. --back .. --apply-value`; Writes to a single file (`scripts/standoff_geom.sh`). The glass sensor pose pitch in the SDF SHOULD REMAIN 0 — typing angle there imposes a QUIET offset on top of the elevation commanded by the gimbal. The write path of `set_mounting.py` is closed, only `--display-value` works.]**
6. **The second client does NOT connect to ports MAVLink during the run** — `swarm_command.py state_value` killed a run (stole the second bind ARM ACK to 14561). Status query only after the run is finished.
7. Agent selection: Opus alone or orchestrator itself (user preference, 2026-08-05).
