# TO_TEST — MPC strike law: root-cause based test sequence (ROUND-5+)

Session: 2026-08-05. Synthesis of three parallel analyzes (running dump + code structural scan + literature) + orchestrator's own solver/video verification. Context: `FOLLOW_UP.md` (rounds 1..4 campaign) and `guidance_allstar/LOG_DICTIONARY.md` before.

> **[GIMBAL BRANCH UPDATE 2026-08-05]** This entire file was written assuming a **body-fixed camera**. 5 drone's camera in sim on branch `gimbal` on self-stabilizing **physical single axis (tilt) gimbal**: measured in flight, body pitch −35.4…+35.2° while pitching, camera pitch relative to the world **max |0.65°|**. Result: **body pitch not reflected in the image**, **roll reflected** (single axis), **no gimbal in yaw**. `YILDIZ_TILT = atan(down/back)` now determines the vertical axis (`scripts/standoff_geom.sh`), so it can be **commanded**. Any item/measurement that relies on pitch should be re-read in this light; The item is also marked 5 and 6 below. Detail: `GIMBAL_NOTES.md`.

## ★ LIVE T3 ASSESSMENT — 2026-08-11, big-box gated single LOS

The large misses in the pure-LOS T2 runs were caused by early handover at long range. The four large-miss cases started with a bbox scale of only `%2.03–2.94`. The new gate requires a scale above `%3` for 5 consecutive frames, without requiring image centering or telemetry range (`YILDIZ_TRANSITION_LARGE_FRAME=5`, `YILDIZ_TRANSITION_AREA_PCT=3`). The hardware node uses the same rule through `--large-frame 5 --area-pct 3`.

| arm | n | actual CPA median/p90 | `<5 m` | contact | heavy tail |
|---|---:|---:|---:|---:|---:|
| LOS N=4 | 6 | **1.09 / 2.03 m** | **6/6** | **2** | 0 |
| LOS N=5 | 12 | 1.65 / 3.76 m | 11/12 | 0 | 1 (`8.94 m`) |

N=4 CPAs `0.42, 0.79, 2.60, 1.45, 1.39, 0.63 m`; `impact_successful` occurred at the beginning of two separate stack starts and contact crashed the vehicle. N=5 CPAs `0.48, 1.64, 1.50, 1.67, 2.93, 2.71, 2.60, 0.79, 3.85, 8.94, 1.20, 0.43 m`; no contact. The first automatic N=5 process exited during a HOME_POSITION race. The dead-man gate prevented an invalid handover. Reconnecting the process during the same healthy flight produced 12 engagements. The `292` frame counter accumulated when the controller is not present has also been fixed to be reset so that there is no credit for the process opened later.

**Decision:** T2 provision changed; The only hardware candidate N=4 is LOS. MPC A/B/fallback only. The next variable is not LOS gain, but gradual verification in real hardware: pure tracking → gimbal/cue/delay → hover dry-run → low speed geofence flight. Going without telemetry is OK; Terminal range without telemetry is not yet complete. Yaw shutdown is a separate A/B of safety/FOV and will not be confused with the champion default.

## ★ LIVE T0 ASSESSMENT — 2026-08-10 18:19, bare defaults, target_ellipse

Active running: `visual_mpc_20260810_180540.csv` / `mpc_diagnostic_20260810_180540.csv`. Snapshot taken without stopping the process: ~10.5 minute, 4.2k imaged row, 23 individual close pass. The running `mpc_guidance.py` process had no `YILDIZ_*` overrides; P+TGO, saturable actuator, BLIND_PN and HOLD_YAW are on with new dispatch defaults.

### New root cause: unreachable 180° speed reversal in terminal

- The system can **locate** behind the target: `R<20 m` median angle of approach **160.2°** (180° pure tail). The problem is not "not being able to get to the back at all".
- However, in `R<8 m`, the median angle between the horizontal speed command of MPC and the actual speed direction of the vehicle is **171.6°** (in the closed examples); command-actual speed difference **51.9 m/s**. The command is also **175.7° opposite to the target's flight direction**.
- In the same band, yaw command median is **88.6°/s**, actual roll is only **5.0°**. In 91.4% of the samples the desired speed is out of alignment by more than 90°.
- MPC horizon fixed **2.4 s**; `R<8 m` actual `t_go` median in closed samples **1.54 s**. The majority of the cost remains after the anticipated CPA/crash; After `rbar` is nailed to the range base, the solver selects the return setpoint for the future that is not physically present.
- This is the horizontal equivalent of the "dives/passes once, gets up and hits on the second attempt" behavior that the user sees: the setpoint is reversed on the first pass, the vehicle cannot implement it in time with the 5 m/s² acceleration ceiling, then enters the second lap.

### Rotation and vertical findings

- CPA median: **9.96 m** in straight phases where the target rotated `<4°/s` in the last 3 s; `>=8°/s` in hard turning **13.36 m**. None of the hard-spin 8 crossovers have `<5 m`; a transition in straight phase **2.58 m**.
- In hard turning, the MPC already requires large lateral velocity (`|u2|` median **29.1 m/s**) and enlarges the yaw (**67.7°/s**), but the real roll median is only **9.4°**; command-actual speed difference **21.7 m/s**. `apn_a` on all lines **0**: target curvature does not extend to the horizon at all.
- Vertical "branch-then-rise" real: In the 20 of 23 CPA the real `vel_z` changed sign at least twice in the previous 5 s. In the `R=8..15 m` band, the command is %16.6 on the climbing rail (`cmd_vz<=-8.5`), %12.1 on the descending rail (`>=+4`). However, the median of CPA `|dz|` is ~1.2–1.7 m; vertical is now an issue but not the primary cause of hard turn misses.
- Solver budget-cut ratio in GUI run is ~%90. This is the known GUI cost; The final decisions below prompt headless replay. However, since 171° inversion is systematic in many terminal samples, it cannot be considered solver jitter alone.

### Architectural decision

**MPC will not be completely discarded; will be stripped of its role as a pure terminal striker.** MPC FOV in mid-phase/useful for altitude/speed constraints and rear-quadrant placement. An accessible **PN/ZEM collision-cone reference** will be generated at the terminal; MPC will either be the limited distributor of this reference, or it will be directly replaced by the acceleration-limited terminal law in the `R<12..15 m` band. Yaw holds the camera; roll is not commanded directly, the desired inertial lateral acceleration is given to the ArduPilot speed/acceleration layer. Direct roll/attitude control is not the first option because it bypasses security layers.

## ★ LIVE T1 ASSESSMENT — 2026-08-10 19:18, pure LOS and hybrid ellipse flights

The T0 hypothesis was confirmed by live A/B. New arms: `terminal_los_guidance.py` (pure image PN/LOS) and `hybrid_guidance.py` (18 m outside MPC, inside PN/LOS). In both arms, the first command is projected from the actual speed to the accessible cluster with acceleration and direction cone; vertical command also goes through jerk and absolute speed limit.

| arm | valid engagement | real CPAs (m) | CPA median | `<5 m` | physical contact |
|---|---:|---|---:|---:|---:|
| long lively pure MPC base | 59 | — | 11.20 | 3/59 | not measured |
| pure terminal PN/LOS | 2 | 7.03, 6.48 | 6.76 | 0/2 | 0 |
| first hybrid, before vertical limit | 4 | 1.79, 3.47, 2.74, 7.27 | 3.11 | 3/4 | 0 |
| safe hybrid (`hybrid4`) | 3 | 3.22, 3.42, 4.00 | 3.42 | 3/3 | 0 |

`hybrid4`te `R<8 m` closing 103 command–actual velocity angle in the frame p50/p90 **5.35°/6.72°**, `>90°` inversion **0/103**; speed jump p50/p90 **2.76/3.15 m/s**. Angle p90 in the projected outer MPC band (`18<R<35`) **8.12°**, inversion **0/232**. The 171.6° terminal reversal at T0 was thus not only alleviated but structurally closed.

### Limits and operational findings

- Proximity achieved, but **impact not yet achieved**: in all hybrid transitions, jitter remained in the band 0.6–2.0, `impact_successful` did not occur. The next referee will be `ref_range_ground_truth_m + vibe/temas`, not just the estimated range.
- `cmd_vz=-9 m/s`, altitude 78.9 m and visual post-phase recovery crash were seen on the first hybrid arm. With the vertical limit `cmd_vz=-2.5..+0.70 m/s`, the altitude became 52.1–67.9 m and there was no crash; but at the end of the run, three `ALTITUDE ABORT` remained. Vertical energy is still P0 safety work.
- Pure LOS would not leave any authority after the first pass, returning head-on to the second attack; Added post-transition release. Pure LOS is a good A/B/fallback, not the main architect chosen.
- **20 s estimation/altitude settling time** is required before the controller on the fresh stack (`CONTROL_WAIT_S=20`). Also fixed script cleanup regex to kill underscore controller names; The first ~66 seconds of the first LOS run were excluded from analysis due to dual controller.

### T1 architect decision

**Selected direction hybrid:** positional guidance brings to the rear slot of the target; after visual handover MPC is FOV/constraint planner only in the middle phase (`R>18 m`); PN/LOS `R<=18 m` is the terminal collision law; The output of both passes through the same layer of accessibility and vertical safety. Pure MPC was removed from the terminal, and pure LOS was removed from the main nomination. A direct roll command will not be added: the achievable lateral velocity/acceleration request is already translated into roll by ArduPilot; yaw only keeps the view of the fixed camera.

## ★ LIVE T2 VERDICT — 2026-08-11, RoboFly/ellipse, long pass campaign

The aim was to replace the telemetry range-dependent MPC→LOS transition with a decision made from the image alone and remeasure pure LOS with sufficient samples. Current range-hybrid behavior is frozen by default; experimental visual decision added behind `hybrid_guidance.py --transition-source visual`. Since the HSV detector does not produce confidence, fresh/valid detection was used instead of confidence in this round.

Visual gate: normalized bbox scale `100*sqrt(A/(1280*720)) >= %3.4`, `|ex| <= 6°`, `|ey| <= 15°`, seamless `0.30 s`. Thresholds were first selected in the 22 engagement replay of the current good flight; 600 s were then flown closed loop.

| arm | transfer | Switching to LOS | actual CPA median/p90 across all revs | `<5 m` | Median/p90 recently switched to LOS | abort/collapse |
|---|---:|---:|---:|---:|---:|---:|
| previous live champion, range-hybrid | 22 | 16 | 1.97 / 26.06 m | 14/22 | **1.44 / 5.24 m** | 0 / 0 |
| visual-switching hybrid, 600 s | 29 | 15 | 5.09 / 30.08 m | 14/29 | 1.50 / 6.95 m | 2 / 0 |
| pure LOS, 600 s | 19 | 19 | 1.95 / 39.13 m | **15/19** | same | 2 / 1 |
| co-condition range-hybrid, 600 s | 16 | 9 | 12.69 / 31.50 m | 7/16 | 1.91 / 19.17 m | 2 / 1 |

The four large misses in pure LOS were `13.74, 20.35, 39.13, 48.74 m`. Some trials handed control to visual guidance at 29–49 m and skipped MPC's setup phase. In the visual hybrid, 13 of the 15 engagements that reached LOS achieved `<5 m`, so the terminal law still performed well. However, the area-and-center gate did not open in 14/29 engagements, and delayed some transitions until the actual range was `11.37 m`. Requiring the target to be exactly centered prevents handover in turns where MPC cannot establish that condition.

**Verdict:** the champion is still the previous range-hybrid; The default `--transition-source range_value` will be retained. MPC will not be abandoned; In the middle phase, it prepares the target for terminal geometry in FOV. Pure LOS solo control/fallback. The visual passcode will remain, but the correct next design in the real sensor is not the binary threshold: confidence+dwell, bbox scale and center error, as well as the low angular velocity of LOS; Fusion/discord gate with onboard range, if available. If the telemetry range is incorrect, it should not be used alone either in the pass or in the terminal PN gain/t_go calculation.

**Next single-variable A/B:** Design gate `lambda_dot` (bbox angle rate) from log before relaxing center gate. For the small oscillation on the MPC, the command dead-zone/hysteresis can be tried in a separate run; will not be mixed in the same run as the transition experiment. The priority is higher safety work, repeated altitude collapse on both RoboFly's control arms: narrowing the speed/bank/vertical command envelope and transitioning to actual flight before ground contact is reset.

## NEW PRIORITY ORDER — item 28–33

### 28. Terminal reversal replay and new mandatory metrics — P0 prerequisite

**Work:** Set up hot-start replay feeding this live CSV in natural order. Log the following metrics in each cycle: `angle(cmd_xy, vel_xy)`, `angle(cmd_xy, target_vel_xy)`, `|cmd_xy-vel_xy|`, projected time to first CPA, remaining fraction of the horizon after CPA, actual/desired lateral acceleration, and equivalent bank.

**Acceptance:** replay must produce command reversal >150° on band `R<8 m` in at least one engagement. If he cannot produce, he is not an offline referee; item 29 is sim A/B directly. The current 23 CPA base is frozen.

### 29. Terminal reachability + collision-cone law — P0

Three arms will be made separately A/B/C; will not be mixed in one patch:

1. **Horizon cut at CPA:** `T_h <= k*t_go` (`k` scan 0.6/0.8/1.0), cost/area reward only before CPA; post-collision `rbar=6 m` tail will not cost.
2. **First-command reachable set:** Even though the internal model of MPC is saturated, 52 does not send m/s bounce out. `|u0-v|` will be limited by acceleration×time and jerk; The speed direction will be prohibited from changing more than 90° in a single frame.
3. **Terminal PN/ZEM reference:** `a_lat = N*V_c*lambda_dot` + vertical equivalent; The purpose is not to chase the image center but to reset the ZEM/LOS speed. MPC will follow this acceleration reference within constraints. First scan is `N=2.5/3/4`.

**Primary acceptance:** Command–velocity angle median when closing `R<8 m` `<45°`, `|cmd_xy-vel_xy| p90 <15 m/s`; The CPA median decreases and the 2.58 m tail present in the flat phase does not disappear. **Safety:** FOV fresh rate, altitude floor and recovery count are non-regressive.

### 30. Back-cone gate and two-phase speed program — user intent

**Purpose:** "get behind it and accelerate and crash from behind" to be a direct state machine.

- **SETTLE:** Approach target speed at `R>12..15 m` or tail angle `<165°`, reset speed LOS and lateral relative speed; Don't give full throttle just to close the range.
- **HIT:** If the tail angle `>=165°`, `|lambda_dot|` is small and the condition is stable at least 0.4–0.6 s, open the forward speed difference with m/s sweep +3/+5/+8.
- If the cone is broken, SETTLE again; Generating 180° return setpoint in 8 m.

**Acceptance:** `R<10 m` approach angle p10 `>=155°`, median `>=165°`; The closing frame rate increases, the command direction does not contradict the target rate. There will be no regression on the straight legs of the ellipse and `target_straight`.

### 31. Constant-turn/arc target model — P1 of the turn target

The Raw APN variant was previously eliminated for noise; It is forbidden to reopen the same crude variant. The turned-bbox angle marked with `YILDIZ_MINRECT=1` + range-free LOS upright speed will enter the trust-gated constant-turn/IMM state. First the directional sign will be verified with the current left-turn pool + **single right ellipse run**. Then the target velocity will spread across the horizon in a CT arc instead of a straight line; Target acceleration will be added to PN as feed-forward only when it is safe.

**Accept:** `|target turn|>=8°/s` CPA median **13.36→<9 m**, at least one `<5 m` pass; flat phase CPA median does not worsen from 9.96 m. Lateral command-actual speed difference and actual roll response are reported together.

### 32. Vertical single-pass critical ending—P2 of the visual dip/go problem

P+TGO reduced error magnitude but chatter remained. The critical reference CPA will be installed with `dz≈R*sin (eps)` and `dz_dot`; The A/B tests will evaluate hysteresis at small `|dz|`, jerk/slew limits on `cmd_vz`, and a gate preventing an unnecessary second sign change during an engagement.

**Acceptance:** Actual median of `vel_z` sign change in 5 s before CPA **2→<=1**; 8–15 m vertical rail occupancy sum **%28.7→<%12**; CPA `|dz|` median does not worsen and ground/altitude gates are preserved.

### 33. Architectural decision experiment — When do we abandon MPC?

At the end of article 29 the three arms are compared on the same starting-geometry layers: full MPC, mid-phase MPC + terminal PN/ZEM hybrid, pure PN/ZEM + safety clamps. Ellipse + flat + wanderer, headless, engagement/arm at least 6.

**MPC is removed from the terminal only if** the hybrid/pure arm clearly exceeds full MPC on both CPA/contact, freshness and recovery metrics on two different routes. Expected verdict: MPC remains as mid-phase constraint manager, hybrid terminal wins.

## VERDICT — why it doesn't always succeed

Not a single error, **chain of three independent structural layers**; and the dominant ring shifted throughout the campaign (each type closed one ring, the next appeared, total closure monotonic melted: e.g. range velocity −3.47 → −0.68 m/s CIRCUIT1→CIRCUIT4).

1. **NO REASON — shutdown is coded as a PRIM, not an PURPOSE.** Input level penalty `[0.0, r_speed, r_speed, r_yaw]` (`mpc_guidance.py:1586`) conscious zero on the forward channel; but `acceleration_run` `‖u−w‖²` iron is applied to all three speed channels (`:1620`), including forward; the only term requiring closure (linear space reward) is divided by `/N` and 20 (`:2018`). Result: a valid solution to the "three side by side with the target" cost. Data: 24-32 `u1` median at m CIRCUIT1 +34.7 → CIRCUIT4 **−2.63**; In that band, the target is in the center of the frame, the speed is equal, the range is rotated, 8 s formation flight, then timeout. **12 12 of WATCH timeout is in this band; 11 takes over segment 11 with heading angle ≥45°.**
2. **Most of the horizon AFTER impact.** Horizon constant 2.4 s, t_go median 0.61 s; Average of the horizon in STRIKE. In the future without %73, `rbar` 6 chirped to m, ×3 terminal weight there. Constraint horizon scales with range (p50 0.45 s), cost horizon does not → 5 floor asymmetry.
3. **Terminal blindness overlaps by design.** In the last 5 m, 78% of the cycles are blind; end of segment ~2.7 s %100 not-fresh; last fresh measurement avg. 18 m. Four gates are in phase: disruptor freeze (r<15 m), hard FOV release (stale bbox), blind glide (solver does not run), skeleton hold/without (MPC not called at all, ~⅓ cycle).

### Two fixes (orchestrator own control)
- **"FOV %96.8 binding" is FALSE.** `fov_free=0` = "constraint APPLIED", not binding (`mpc_guidance.py:2185`; `LOG_DICTIONARY.md:263` is incorrectly defined). Real activity is low. Tuning decisions made based on this metric are questionable.
- **The resolver is UNRELIABLE for one-shot redecoding of the logged occlusion moment.** Diagnostic CSV does not log hot start/`u_previous`/`confidence_value`/box disruptors; cold solution fails log command (difference seen 38 m/s). This in itself is a finding: flying instruction is not optimal, intermediate point shifted to hot-start (hitting iteration ceiling %46-70). **Conclusion: solver experiments should be done with CLOSED-LOOP replay, not single-shot resolve.** (Item 0 below.)

### Measurement alert
ENG-4's "0 shot" is NOT a control error: `vibe>15 ∧ range<3` fires only in pure tail geometry; TYPE-4's 0.94 m transition remained in transverse geometry, `vibe` remained in 2.7. CIRCUIT-4 CPA distribution is the best of the campaign (15 in the segment 3 <3 m).

---

## WORKING RULES (each experiment)
1. **RANGE only from telemetry** (for now; low confidence). **Camera EXTRACTION FREE** — including bearing, bbox size and target kinematics estimated from them (user refinement 2026-08-05). The ground detection system will be discussed in a separate branch. AFTER logging command with prefix `ref_`.
2. Don't touch shared files while your sim is running. One sim at a time.
3. Agent: solo opus or orchestrator. Sonnet/Haiku FORBIDDEN (user, 2026-08-05).
4. **Keep what works, throw away what doesn't, perfect it.** Each item is clearly marked with a single knob + measurable acceptance criteria + offline→sim space.
5. Offline test running OFF budget (`mpc_test.py:596`) — not flight regime; regression lock, not absolute performance. Sim run last word.

---

## ITEM 0 — SADIK OFFLINE REPLAY HARNESS (prerequisite, highest intelligence)
**Why first:** Articles 1-3 all require a CLOSED-LOOP answer to the question "is the cost stationary, what happens when the horizon cuts off"; The one-shot reload solution is unreliable (up). **Work:** Add CROSS geometry scenario (bearing-route ≥45°) onto the scenario infrastructure of `mpc_test.py` (straight heading, target 21.05 m/s, 3 rev × 2 seed) and generate the occlusion offline. If not: write replay that feeds the actual `Measurement` sequence from the log (let the hot boot come back naturally). **Acceptance:** offline harness reproduces `u1<target speed` occlusion in seed 24-32 in m ≥1 (qualitatively similar to %79.8 in sim). If it fails to produce item 1 is tested directly in the sim, skipping offline (and this will be NOTED). **Intelligence:** high (orchestrator). **Bulk inference:** Opus.

---

## ‼ MOST CRITICAL ON: GIMBAL IS STABILIZED BUT **NOT TRACKING** (2026-08-05 23:10)

`bbox_to_redis.py:1037` — `tilt_commander.target_value(eps_cmd)` **ONLY BEFORE**, called at startup. There are NO calls in the flight loop that update the tilt (`grep '\.target_value('` → single result). So the camera stands at earth-fixed **+9.09°** (= `atan(down/back)`, nominal standoff geometry), no matter where the target is.

**Gimbal can rotate ±90°** (SDF `tilt_joint` limits) — i.e. hardware sufficient, no commands.

### Geometric result: <8.2 target DOES NOT FIT in frame at m
Apparent rise of target `eps = asin(4/r)` while vertical standoff `down=4 m`:

| range | 25 m | 15 m | 10 m | **8.2 m** | 6 m | 5.5 m |
|---|---|---|---|---|---|---|
| eps | 9.2° | 15.5° | 23.6° | **29.2°** | 41.8° | 46.7° |

Camera axis +9.09°, vertical semi-FOV ±20.07° → exits from the top edge when the target is `eps > 29.16°` → **critical range 8.21 m**.

**VERIFIED BY MEASUREMENT:** range median at the moment of loss of frame A arm **8.1 m**, B arm **5.5 m** (B goes deeper because it closes faster, at the time of loss ey = −25.2° i.e. 25° above the target axis). Estimate 8.2 m — exactly as measured.

**THIS IS NOT A GUIDANCE PROBLEM, IT'S A LACK OF COMMAND.** No cost/weight setting can fix this: the target is physically outside FOV. The solution is to have the tilt FOLLOW `eps` (the work that the gimbal branch planned as "Phase O").

**Expected gain:** the entire terminal band comes back (0-5 m freshness A %60 / B %42 → theoretical ~%100), and the justification for retiring `impact_alignment` ("the gimbal solves by rotating the camera") only then becomes valid — that At this moment the tilt is NOT revolving, so the reason has not yet been met.

## ★★ SIM A/B RESULT (2026-08-05 22:37–22:55, target_straight, 360 s, WITH GIMBALL)

Two runs, same plan/duration/gimbal; the only difference is two env buttons. A = `mpcA_baseline_straight_20260805_223745`, B = `mpcB_corrected_straight_20260805_224747` B arm: `YILDIZ_Q_AREA_MULTIPLIER=4 YILDIZ_HORIZON_RANGE_REF=60`

| criterion | A (baseline) | B (with correction) | provision |
|---|---|---|---|
| MISS timeout | 2 | **0** | ✅ that was the goal |
| closest range | 1.76 m | **1.30 m** | ✅ |
| CPA < 3 m (8 in transition) | 2 | **4** | ✅ double |
| shutdown `range_rate_value` p50 | −3.23 | **−4.45** m/s | ✅ |
| closing average | −2.25 | **−4.28** m/s | ✅ ~2× |
| `u1` p50 (forward command) | 25.1 | **33.9** m/s | ✅ approached the ceiling |
| frame loss incident | 9 | 7 | ✅ lightweight |
| **fresh rate (general)** | %72 | **%65** | ❌ price |
| **fresh, 0-5 m** | %60 | **%42** | ❌ terminal framing |
| pitch speed p50 | 11.4 | **14.2** °/s | ❌ tremor increased |
| "range is opening" MISS | 0 | 1 | ❌ light |

**JUDGMENT: fix hit its target, cost out of expected range.** Timeout completely expired, shutdown ~2× increased, number of sub-meter passes doubled. The cost was in the terminal frame (0-5 m freshness %60→%42) and flicker - it was in the same direction in offline (frame loss %18→%29), so **offline→sim harmony held this time**. Hit confirmation (`impact_successful`) 0 on both arms, but that criterion is blind in cross geometry (see mpc_memory).

### SECOND VERIFICATION — target_ellipse (23:12–23:27), same A/B
`mpcA2_baseline_ellipse_231245` / `mpcB2_corrected_ellipse_232030`

| criterion | STRAIGHT A→B | ELLIPSE A→B |
|---|---|---|
| MISS timeout | 2 → **0** | 2 → 2 |
| closest range | 1.76 → **1.30** | 1.55 → **1.35** |
| CPA median | 3.62 → 3.50 | **16.19 → 6.52** |
| CPA < 3 m | 2 → **4** | 3 → 3 |
| number of engagements | 8 → 8 | 9 → **14** |
| 0-5 m freshness | %60 → %42 | %70 → %62 |

**SAME DIRECTION, DIFFERENT CHANNEL ON BOTH ROUTES.** Ending the gain timeout on the straight route + doubling the sub-meter pass; and in the ellipse, improving the CPA median **2.5×** (16.2 → 6.5 m) and increasing the number of engagements. Common: **closest range improved on both routes**, price is terminal freshness on both routes. The fix works regardless of the route → **ACCEPT**.

The source of the charge was also diagnosed and is irrelevant to this fix: fixed-tilt geometry (see "GIMBAL STABILIZES BUT DOES NOT TRACK") above.

**NEXT BOTTLENECK IS NOW TERMINAL FRAMING** → TO_TEST article 1c (PAMPC image-plane velocity penalty) and article 5/6 aim to recover this cost.

## ★ REMEASUREMENT AFTER GIMBAL (2026-08-05 night, offline n=32/group)

**In the MPC code, there is ONLY the 2 parameter that changes with gimbal switching** (archived baseline → gimbal revision): `pitch_coupling` True→False and `impact_alignment_closing` True→False (mechanism RETIRED). The core of the guidance law (J, constraints, solver, phase machine) has NOT changed. So any algorithmic fixes on TO_TEST HAVE NOT BEEN IMPLEMENTED YET.

### Effect of gimbal alone (same MPC, different physics)
| | frame loss | top edge | pitch speed | **closing** | **timeout** |
|---|---|---|---|---|---|
| body-fixed physics | %18.3 / %28.3 | %18.2 / %28.3 | 5.5 / 11.4 | %25 / %67 | %75 |
| **GIMBAL physics** | **%0.0 / %0.0** | **%0.0 / %0.0** | 5.5 / 4.1 | %25 / %67 | %75 |

**COMPLETELY erased gimbal frame loss** (%18-28 → %0) and cut shake in half. **BUT it NEVER touched shutdown and timeout** — 22-45 m blockage remains the same. The two faults were independent, measurement confirmed this.

### TO_TEST item 1+3 STILL REQUIRED in gimbal physics and STILL WORKING
| arm | CLOGGED closure% | med | timeout% | TAIL closure% | pitch speed |
|---|---|---|---|---|---|
| current (uncorrected) | 25 | 24.7 | 75 | 67 | 9.1 |
| +q_area ×4 | 75 | 3.2 | 41 | 67 | 4.9 |
| + horizon ref=60 | 75 | 4.2 | 25 | 67 | 4.8 |
| ***+ BOTH** | **100** | 3.7 | **0** | 67 | **4.4** |

It was the same BEFORE the gimbal (%25→%100, TO %75→%0). So the fix works independently of the gimbal; Moreover, it also reduces shaking in gimbaled physics (9.1 → 4.4 °/s) and framing loss remains zero. **This is the next sim run.**

## PRIORITY REVISION (user, 2026-08-05 evening)

The user INCREASED the priority of three items while reading the code MPC. New order:

| new row | article | Why did it rise? |
|---|---|---|
| **1** | J FUNCTION JOB (item 1 + new 1b/1c) | the cost function was the root cause; The form of reward has never been verified by literature |
| **2** | item 7 — SOLVER METRICS (+ new: fixed-point residual) | The question "did we solve it" has NO measure; pushes the ceiling %46-70 |
| **3** | item 10 — ACTUATOR MODEL | τ=1.0 wrong in both directions; miss timeout unreachable alignment calibration |
| 4+ | old sequence (2, 3, 4, 5, 6, 9) | — |

### 1b. REWARD FORM A/B: area vs SQRT area [NEW, user request]
Currently the prize is LINEAR FIELD (`A = w·h`, `_area_update` says "NOT square root"). The square root arm has never been measured. Difference: `A ~ K/r²` while incentive grows with `1/r³`; `sqrt(A) ~ 1/r` while the incentive is `1/r²` — so the square root is SMOOTHER in the terminal. **Experiment:** Run A/B on `_area_reward` with a lever that makes the relative reward `(r0/rg)` instead of `relative = (r0/rg)**2` (+derivative coefficient accordingly). **Acceptance:** cross closure and tail CPA compared to arm q_area×4; terminal judder (pitch speed) and framing loss are read as side effects. **Hypothesis:** square root less aggressive → less frame loss but later closure. The knee point will be sought. **Intelligence:** high (orchestrator). Offline.

### 1c. LITERATURE VERIFICATION: Two additional terms to J [NEW]
Two J changes that came up in the scan but have NEVER been tested:
- **PAMPC image-plane velocity penalty** (arXiv 1804.04811): suppresses the target's drift velocity IN-frame before the penalty → loss. Our new bottleneck (frame loss) is exactly this.
- **ZEM-shaping terminal cost** (Lee & Shin): cost-to-earth of short horizon; Answers the question "what will happen at the end of the horizon" when the horizon is shorter than t_go. **Intelligence:** high (orchestrator). It can be tested offline.

## PRIORITY-ORDERED EXPERIMENTS

### 1. ABSOLUTE DEMAND FOR SHUTDOWN SPEED [ROOT CAUSE — VERIFIED OFFLINE]
**Hypothesis (revised):** Initial hypothesis (exempt `u1` from iron `‖u−w‖²`) DISRUPTED in OFFLINE A/B — alone does not break occlusion (min 23→13 m, timeout %75 fixed, frame loss %18→%25 increased). **CORRECT ARM: enlarging the bbox AREA reward (`q_area`)** — driving the closing signal from the BBOX AREA, not from the range (true to the user philosophy: see with your eyes, kinematics do not extract). **Location:** `mpc_guidance.py:2018` (field reward /N), `MpcConfig.q_area`. **OFFLINE RESULT (mpc_test closed-loop, cross+lateral × straight+ellipse × 4 seed; with tail regression control):**
| arm | CLOGGED minR | timeout% | framing loss% | pitch speed | TAIL minR |
|---|---|---|---|---|---|
| baseline | 23.0 | 75 | 18.2 | 5.5 | 1.8 |
| q_area ×3 | 4.4 | 38 | 23.3 | 8.1 | 1.9 |
| **q_area ×4** | **2.2** | **26** | SUB-METER TAIL: sub-meter transitions lost, source unknown | **OPEN** | uncorrected-BLIND_PN pool (kpn1b/kpn2/kpndplain) vs corrected (kpn3b/kpn4/d0kpn/kpn26c/kpn26d) | **0.69-0.92 m transitions were seen ONLY during the uncorrected-BLIND_PN period** (while the blind \|ex\| escaped out of frame to 28-47°). Once the R3′ clamp cut the leak, the tail went too: best pass 0.69/0.73/0.75 → 1.08 (ceiling 20) → 1.22 (ceiling 26). Clamp 26 corrected **median** (CPA3B 5.99→3.50 in common layer, horizontal 4.66→2.77, <3m %14→%40) **but he didn't bring the tail**. So it is NOT the clamp width that produces branch-meter transients. Candidates: (i) the chance of a runaway advance (blindly going and hitting the target out of frame — unrepeatable, unreliable), (ii) a different side condition of that period (vertical arm/profile), (iii) the branch-meter CPA already comes with luck tail and n. To distinguish: increase n in the same configuration (pool 10→20+) and compare the cycle geometry of m transitions <1.5 |
| **27** | UNCONTROLLED YAW DURING RESCUE (500-800 deg/s) | **OPEN — to be excavated** | kpn26c (149-505 deg/s), kpn26d (663, 813, 546, 356 deg/s) | During HOLD/BRAKE recovery, the vehicle spins at **500-800 deg/s**. **Not a command:** In HOLD the steering does not send setpoint (`send=False`, mode BRAKE) and `ATC_SLEW_YAW`=180 deg/s — measured 813 is **4.5 times**. **Not an artifact either:** the value is from `ATTITUDE.yawspeed`, i.e. direct gyro (`mavlink_utils.py:110-115`), not digital derivative. That is, increased **uncontrolled angular momentum** from aggressive manoeuvring/departing. This is exactly what HOLD's `spinning` extension (RECOVERY_YAW_RATE_HOLD_DPS=40) is for and works (most of the 14 recovery is extended from this arm). QUESTIONS: (i) where is the momentum coming from—horizontal braking, departure residual, BRAKE mode releasing yaw; (ii) **what is the framing/detection cost** — when rotating 800 deg/s the bbox definitely disappears, this may extend the re-acquisition time after recovery; (iii) Is it possible to actively use (suspend) yaw authority in rescue? |
| **25** | 20.4 | 9.0 | **1.9** |
| q_area ×6 | 4.4 | 12 | 23.3 | 10.0 | 2.0 |

**Knee point ×3-4:** occlusion range 23→2.2 m, timeout %75→%25, **tail geometry NOT DISTORTED** (1.8→1.9 m). On ×6 the min range goes back (overclosing / penetrating). u1 iron is NOT the main arm. **Accept (sim):** 24-32 m occlusion range drops, timeout ≤%30, no TAIL CPA regression. **OFFLINE→SIM SPACE:** frame loss and judder side-effects are less reliable offline (the engine enters the terminal flat); min-range and timeout are reliable. Verify in sim: Does q_area ×4 break the wall, does the frame loss/shake not go above baseline. **Risk:** overly aggressive → penetration + loss of frame; ×Stay in 3-4 band. **Intelligence:** high (orchestrator). **Status:** OFFLINE OK, SIM WAITING (another agent is in the sim).

### 2. GEOMETRY CONDITION FOR THE TRANSFER GATE [cheapest-highest return]
**Hypothesis:** Delay handover / "queue first" while `|bearing_value − route| ≥ ~45°`. 11/11 miss is PRE-known with this splitter (<45°: 31 seg, min range avg. 4.5 m; ≥45°: 11 seg, all miss, avg. 23.7 m). **Location:** `bbox_to_redis.py` cycle gates (COMMON FILE — A/B in one run). **Acceptance:** proportion of inherited segments ≥45° ~0; Missing timeout expires. **Risk:** cycle count decreases, wait per engagement increases (currently ~17.7 s/rev). **Intelligence:** low-medium. **Executor:** Opus (orchestrator acceptance meter).

### 3. SCALE THE COST HORIZON WITH RANGE [VERIFIED OFFLINE]
**NOT t_go, BUT RANGE.** `mpc_guidance.py:1765` note: t_go scaling has ALREADY been tried and turned out to be INeffective on diagonal geometry (closure ~0 → t_go infinity → scaling does not take effect at all). The constraint horizon therefore scales with range (`:1767`); the cost horizon wasn't scaling — that's where the asymmetry was. **Application:** `step_s *= clip(r/ref, floor_value, 1.0)` in `_step_durations`, ref≈60 m. **Hypothesis DISRUPT / new mechanism:** terminal repeatability NOT FIXED (inter-seed scatter 8.85→9.15). Real effect: shortening the horizon at close range makes MPC MYOPIC; “three side by side” is the long-horizon balance, while the space reward is INSTANT → myopia releases closure. **OFFLINE RESULT (n=32/group, 8 seed):** blocked shutdown %25→%75, timeout %75→%25, queue NOT BROKEN. ref=45 and 60 are working, 35 and 20 are inactive. **Status:** OFFLINE OK, SIM WAITING.

### 1+3 TOGETHER — 2×2 FACTORIAL (n=32/group, 8 seed) ★ MAIN FINDING
| arm | CLOGGED container% | med | timeout% | framing loss% | TAIL cap% |
|---|---|---|---|---|---|
| baseline | 25 | 22.2 | 75 | 18.3 | 67 |
| M1 (q_area ×4) | 75 | 2.0 | 22 | 19.6 | 67 |
| M3 (horizon ref=60) | 75 | 4.5 | 25 | 23.8 | 67 |
| **M1+M3** | **100** | 3.7 | **0** | **29.3** | 67 |

**THEY COLLECT** (not redundant). Run finish reason breakdown — the real evidence:
| arm | CLOGGED endings | TAIL |
|---|---|---|
| baseline | MISS_RELEASED 24, FRAMING_LOSS 8, **CRASH 0** | COLLISION 16, ISS 8 |
| M1+M3 | FRAMING_LOSS 16, MISS_RELEASED 9, **CRASH 7** | COLLISION 16, ISS 8 |

**Comment:** Fix changes “pursue” to “engage” — FIRST collisions in crossover geometry (0/32 → 7/32). Tail geometry is not affected at all (16/16 collision on both arms), no ground contact (min altitude 47.2 m). **Cost: run ending in frame loss 8→16.** So the frame loss increase is not a REGRESSION, but the engagement fee that was never paid before when "flying safely side by side" — but the NEW BOTTLENKLE. **Next logical step:** item 5 (pitch compensation with vertical acceleration) + item 6 (pitch delay) — aims to regain framing while maintaining aggressiveness. **[GIMBAL BRANCH UPDATE 2026-08-05: this "next step" has changed. Item 6 was gimballed CLOSED and item 5 was SPLIT IN TWO (see below). The vertical component of the framing loss bottleneck is now solved by TILT, not pitch—a cheaper and more direct arm of dynamic tilt tracking (item 5b). The landscape/roll component remains on (gimbal single axis).]**

### 4. BLIND TERMINAL: "return last speed LOS, resume PN"
**Hypothesis:** Change blind glide to open-loop sustain PN by freezing the last valid speed LOS instead of straight glide (JHU APL homing; IEEE 9066667). The last 18 is flying unseen → directly to CPA. **Location:** `mpc_guidance.py:2741` (`_blind_command`), `:2891` blind glide door. **Acceptance:** CPA median decreases in blind segments; blind time does not change. **Risk:** noisy final measurement grows by freezing → bbox trust/size quality gate. **Intelligence:** low-medium. **Orchestrator:** Opus (orchestrator designs the quality gate).

### 5. BUY PITCH WITH VERTICAL ACCELERATION
**Hypothesis:** `θ≈atan(a_forward/(g+a_up))`; measured 5.8°/(m/s²)≡1/g so the model assumes `a_up=0`. `a_up=+3` → nose 27°→21.3°. The same maneuver extinguishes two modes: nose lifts + `eps=asin(down/r)` shrinks as the standoff is at the bottom. Literature: StableTracker (arXiv 2509.14147). **Location:** `mpc_guidance.py:1826` (pitch=f(acceleration)), release vertical acceleration. **Accept:** top-edge non-physical (%7.4) falls; bottom-margin (%4.2) does not increase; vertical speed ceiling (`WPNAV_SPEED_UP`) is not hung. **Risk:** If the vertical ceiling is not in the model, the plan cannot be implemented → enter the model. **Intelligence:** high (orchestrator). **Bulk:** Opus.

> **[GIMBAL BRANCH UPDATE 2026-08-05 — ARTICLE 5 SPLIT IN TWO]** HAS BECOME THE **FIRST** of the two mods of this article: the "nose lifts → camera lifts" chain is broken with the physical tilt gimbal (fuselage ±35° while camera max is 0.65°). Compensating for pitch has no benefit to the frame. **SECOND mode is ON, but the remedy has changed:** `eps=asin(down/r)` still grows at close range (standoff at the bottom), but now it is necessary to turn it off not by vertical acceleration** but by **dynamic command of TILT**: - **5a (WAS):** pitch compensation with vertical acceleration — pointless with gimbal. - **5b (NEW, cheap, high return):** **DYNAMIC TILT TRACKING.** Tilt is now STATIC: `YILDIZ_TILT = atan(down/back)` is commanded once. In the terminal, `eps` exceeds this. Arm: live command range/measure tilt with `ey` (gimbal tracking 6 rad/s, deadband ±0.17°). **Acceptance:** top-edge non-physical loss drops; tilt command-state difference remains <0.06 rad; bottom-edge does not increase. **Risk:** driving tilt from bbox creates closed loop → RETURN tilt on stale/lost bbox, return to standoff value. - Note: the VERTICAL component of "framing loss", which is the new bottleneck of item 1+3, is solved from this branch; the horizontal component remains in yaw (gimbal single axis).

### 6. REMOVE 0.30 s PITCH DELAY IN beta
**Hypothesis:** Pitch speed p95 48°/s → p95 ~14° error in LPF `beta` (vertical quasi-FOV 20°). Hard FOV constraint sees frame "0.3 s previous". **Location:** `mpc_guidance.py:839` (`pitch_lpf_tau_s`), `:2480` beta installation. **Accept:** top-edge loss drops; band flicker does not return. **Risk:** Removing LPF completely may cause jitter → feed raw pixel margin to constraint separately. **Intelligence:** medium (orchestrator). **Bulk:** Opus.

> **[GIMBAL BRANCH UPDATE 2026-08-05 — ARTICLE 6 CLOSED]** The raison d'être of this article was "camera axis = mount + body pitch"; The pitch LPF was imposing 0.30 s latency on that axis. With the physical tilt gimbal, the pitch is OFF the camera axis, hence the delay that the LPF adds to the `beta`. **New job, not adjustment FIX:** If the `beta` setup is still feeding from pitch, this is no longer an ADJUSTMENT issue, it must be the WRONG UNIT — the axis is `tilt_status`. (The code is in `mpc_guidance.py`; marked here alone because someone else was running during this wave.) The actual remaining latency source: gimbal servo tracking (measured <0.06 rad) and roll (single axis, no compensation).

### 7. SOLVER: METRIC FIRST, THEN BUDGET [PRIORITY 2 — escalated]
**7a. ADD OPTIMALITY METRIC (this one first).** The current stopping metric is `max|ΔZ|·weight < tolerance_mps` (`:2357`) — it says "the step has become smaller", NOT "I'm at optimum"; Even if FISTA is stuck, the step becomes smaller. We have `duration_ms`, `iter`, `cost` but **no optimality certificate**. **Suggestion:** log fixed-point residual: `art = ‖z − Π(z − (1/L)∇f(z))‖` This is the KKT residual of the projected gradient: 0 if the point is optimal. It's cheap (single additional line with `g` and `Π` already calculated), and answers the "have we really solved it" question for the first time. Column `kkt_residual` to `mpc_diagnostic`. **Acceptance:** `iter`, `duration_ms` and `kkt_residual` can be read together; It seems that the residual is really large (or not) in cycles that hit the ceiling. WITHOUT this measurement, budget setting is done blindly. **7b. BUDGET.** p95 13.4 ms / budget 13 ms, asim %13-16, push-ceiling impact %46-70 → command biased to hot-start, item 1 It strengthens. Drop block/horizon or tie budget to measured cycle rate. **7c.** Offline tests are running OFF the budget (`mpc_test.py:596`) — meaning the settings are selected in a solver regime that is never seen in flight. Separate correction. **Location:** `mpc_guidance.py:857`, `:888`, `:2340-2362`; `mpc_test.py:596`. **Intelligence:** medium-high (orchestrator).

### 8. BEARING-ANGLE TMA — target kinematics from camera [ON, high ceiling]
**EXPRESS VERSION OF THE RULE (user, 2026-08-05):** "THEY CAN SEE WITH HIS EYES AND MAKE INFERENCES; ONLY get RANGE from telemetry right now." So **everything derived from the camera is free** (bearing, bbox size, target speed/direction estimated from these); Retrieving kinematics from telemetry/ground detection, which is prohibited. The range is used with low confidence. The ground detection system will be taken into account in a SEPARATE BRANCH (user will open it). **Hypothesis:** bbox center=bearing, bbox size=stretched angle (invariant to camera rotation); 7-guided pseudo-linear KF estimates target position+velocity+unknown physical dimension together (Ning et al. IJRR 2024). Classical bearing-only TMA requires the observer to maneuver perpendicular to the bearing — it breaks down right in our tail geometry; bearing-angle removes this perpendicularity requirement. **Opens:** full version of item 1 (terminal cost based on ZEM), full geometry gate of item 2, better blind-terminal extrapolation in item 4. **Admission:** target speed against ground-truth in offline harness <%15 error and converges on TAIL geometry (where classic bearing-only sinks). **Risk:** apparent bbox width varies with aspect-angle 3-4x (fixed wing side/front) → range/size error. Reduction: use diagonal or elevation, give the telemetry range to the filter as an INDEPENDENT measurement (range is already free). **Intelligence:** high (orchestrator). **Status:** offline prototype in progress. **Note:** Item 1 was also resolved without this (from the closing bbox area); TMA is no longer a REQUIREMENT, it's a CEILING RISER.

### 9. FIX MEASUREMENT TOOLS [visibility, not performance]
- `impact_successful` blind in cross geometry → add CPA + vertical separation criterion (addition to `vibe`). `mpc_guidance.py:2493`.
- The dictionary definition of `fov_free` / `empty_counter` is incorrect. `LOG_DICTIONARY.md:263-264`.
- `internal_range` 3 m-based → final 3 m-state machine blind. `mpc_guidance.py:2452`.
- Dictionary HIT multipliers are stuck at lap-1 (code 1.0). `LOG_DICTIONARY.md:275`. **Intelligence:** low. **Executor:** Opus.

### 10. MAKE ACTUATOR MODEL ACCELERATION-LIMITED [PRIORITY 3 — upgraded]
**Hypothesis:** 1. order τ=1.0 wrong in both directions (2.7x optimistic in the major command, 4x pessimistic in the minor); Model 35 thinks m/s, measured max 29.9; `miss_time_timeout_s =8` unreachable 13.9 calibrated to close in m/s. With rate-limited model. **Location:** `mpc_guidance.py:829`, `:1715`; `:1231` timeout. **Acceptance:** <%20 difference measured from model predicted closure; The timeout is reset. **Risk:** Without item 1 alone it just flies longer side-by-side. **Intelligence:** medium (orchestrator). **Bulk:** Opus.

---

## RECOMMENDED TOUR STRUCTURE
- **CIRCUIT-5a (cheap package, single sim):** 2 + 4 + 9. Geometry gate + blind terminal + measurement repair. A/B is read in one run.
- **ENG-5b (structural, stand-alone):** 1 (item 0 before harness). THE STRUCTURE of the closure request is changing — its impact should be read in isolation.
- **ENG-5c:** 3 (horizon cutting), then 5-6-7 (pitch/delay/solver hygiene). **[GIMBAL BRANCH UPDATE: 5-6 is now **5b (dynamic tilt tracking)**
  + 7. Item 5a became, item 6 closed — see. above.]**
- **SEPARATE PROJECT:** 8 (bearing-angle TMA) — The full ZEM version of 1 is based on this.

## ‼ ITEM 11 — BLIND AFTER VERTICAL CHANNEL PHASE C (2026-08-07, OPEN WORK)

**This is the most critical vulnerability at the moment.** The only cost term that covers the vertical standoff was `q_ey` and is defined on `beta = ey − ey_ref` (`_cost_rows`, `mpc_guidance.py:~2148`). `_framing_constant` (`~:2699`) with PHASE O connected `ey_ref` to **live gimbal tilt**; `beta ≈ 0` → **cost DOES NOT see vertical error because tilt follows eps**. The only remaining vertical term is `sigma_el → 0`, and it by definition says "**maintain** vertical standoff".

**Measurement (DOWN=+4, flat):** actual vertical separation 4 m while target−camera axis difference appears −1.3°. `|dz|` median 45→15 m across **3.4-4.0 m constant**; `alignment_ref` active frame **0**. **2.02 m of CPA 2.24 m in ellipse is vertical residue**. Eliminated hypotheses: `‖u−w‖²` brake (contact with vertical TMF box only %11) and climbing ceiling (required ≈2.5 m/s, ceiling 9.0).

**EXPERIMENT-1 (done, remains in code):** `YILDIZ_VERTICAL_TERMINAL=1` — bilateral, range-ramped (45→25 m) bias to vertical LOS velocity reference. 6 running (straight ×2 double, ellipse ×1 double, DOWN=+4). **Mechanism working** (`alignment_ref` %0→%65-83, terminal eps 7.0→4.5, 20-8 m band `|dz|` 2.59→1.98), WSC 5→0, no regression. **BUT the vertical at CPA no longer dropped** (pool |vertical| median 0.99→1.09); 2/2 slightly worsened on the flat, clear good on the ellipse (vertical 2.02→0.63, CPA 2.24→1.56). **Default remained OFF.** Diagnostics: derivative driven but no error term — D control without P.

**EXPERIMENT-2 (done):** `YILDIZ_VERTICAL_ERROR` (multiplier, default OFF; `q_vertical_error=1.5` derived by weight solver sweep) — to cost **direct error term on eps** (6. line block, `n_row 5N→6N`). `q_ey` and FOV/CBF remained on the live gimbal axis; only the new line references the "target line".

**RESULT — the diagnosis has changed: the problem is no longer ERROR but EXCESS.** The P term actually resets the vertical error; Profile `|dz|` **monotone closing for the first time** (ellipse DOWN+4: 25-35 m 3.83 → 8-15 m 0.92 → 0-8 m 0.67; 3.29/2.07/1.51 in closed arm). But the residual CPA does not decrease because **it reaches zero very quickly**:

| arm | zero crossing | vertical speed @CPA | CPA vertical |
|---|---|---|---|
| off (flat, best) | r=2.70 m, 0.25 s ago | **−0.35 m/s** | −0.13 |
| P q1.5 (straight) | r=2.28 m, 0.20 s ago | **−5.13 m/s** | −1.22 |
| P q1.5 (ellipse) | r=4.21 m, 0.20 s ago | **−4.54 m/s** | −1.10 |

The sign confirms this: closed **+2.02** (not reaching), P arm **−1.10/−1.22** (passing by). 0.2 s × ~4.5-5 m/s ≈ 0.9-1.0 m = entire remaining residue. **P+D does not solve:** `vertical_terminal_tau_s=1.5 s` constant and r<15 longer than t_go (<1 s) in m → early brake, never reaching zero (NO zero crossing in P+D ellipse).

**ROOUT-SPECIFICITY (important):** base CPA vertical on straight route **0.13/0.30 m** — criterion already met; 0.18 m at DOWN=0; 2.02 m in ellipse. The problem occurs when **the target is rotating**. The P arm created **regression** on the straight (0.13→1.22) and DOWN=0 (0.18→1.93) routes. Both buttons remained OFF by default.

**EXPERIMENT-3 (done) — ★ CRITERIA SATISFIED.** `YILDIZ_VERTICAL_TGO` (multiplier, default OFF; `vertical_tgo_k=2.0`). The time constant of the D arm is connected to **remaining time** instead of the constant `tau`: `tau_eff = clip(t_go/k, 0.30, 1.50)`, `sigma_el_ref = vertical_s·k·eps_excess/t_go`. t_go controller from existing `range_rate_value` (no new predictions); Ignored if closure <1.0 m/s → `tau_max`.

**Why does it stop overshooting (closed form):** `de/dt = −k·e/(T−t)` → `e(t) = e0·((T−t)/T)^k`; For k>1 **both e and ė** go to zero on collision. At constant tau `e = e0·exp(−t/tau)`: e shrinks but ė never resets — this is exactly the measured −4.5…−5.1 m/s.

**MANDATORY SECOND PART:** old constant **8 dps** angular ceiling was choking the law at the terminal (6.3 m/s at 45 m = TOP of the descent box, 2.1 m/s alone at 15 m); trimmed demand = constant `eps_dot` = exceedance itself. Without this correction, increasing k changed **nothing** (vertical speed @CPA fixed at −3.35 from k=2.5 to k=8). The ceiling is now derived by direction from the physical input box (ascent 9 / descent 4.5).

**A/B — CPA vertical residual [m]:**

| Route / DOWN | closed (base) | P (EXPERIMENT-2) | **P+TGO k=2** |
|---|---|---|---|
| Ellipse +4 (originally open) | +2.02 / +1.03 | −1.10 | **−0.41 / +0.15** |
| Straight +4 (regression gate) | +0.30 | −1.22 ❌ | **+0.06** |
| Ellipse 0 (regression gate) | +0.18 | −1.93 ❌ | **+0.09** |

Two regressions of P **completely repaired**. Horizontal CPA **improved** in ellipse (0.96/1.84→0.24/0.21) and DOWN=0 (2.45→0.65); WSC 10→2-4. **The campaign's only physical contacts are in this arm:** three `impact_successful`s on three routes (vibe 234/40/24) — none of the closed, D, P, P+D, k=3 arms produced contacts.

**METHODOLOGICAL LESSON:** offline scan said k≈2.6-2.8, **closed loop said 2.0**. Offline's failure mode is *overrun* (capital k is rewarded), real run's is *lateness*. **Rule: offline scanning gives direction, closed loop decides.**

**EXPERIMENT-4 (done) — all three gates passed. JUDGMENT: OPEN.**

**(A) Wanderer** — the movement's **cleanest win is here**. The closed arm does not *at all* close the vertical standoff (belt profile straight: −2.34 → −2.18 m); The main arm closes monotonously. Terminal `|dz|`: 15-8 m **2.81 → 0.52**, 8-0 m **2.18 → 0.59**. `altitude_abort` 1 vs 1, WSA 5 vs 5. **BUT in wanderer CPA vertical CANNOT be measured:** engagements break at 13-37 m on both arms. The reason is not vertical → **item 13** (shutdown fault).

**(B) `altitude_abort` — DIAGNOSIS OK, NO DAMAGE.** Trigger `simple_guided_follow.py:3171`: `alt_error = |pursuer_z − aim_z_nominal|` and `aim_z_nominal` (`:2975`) **z of positioned SLOT** — visual arm does not have to monitor that slot. 23 running / 67 abort classified:

| class | piece | comment |
|---|---|---|
| within engagement | 28/67 | **literal no-op** — `send_setpoint`/`set_mode` is already gated with `position_based_authorized` (only 42 of 67 has line `recovery → HOLD`) |
| ≤5 s after cycle | 27/67 | **Range was increasing on 27/27** — rust gone, correct brake behavior |
| positioned phase | 12/67 | range 96-795 m, **same for baseline** |
| closing, cutting off approach | **0/67** | — |

The number itself is noise (same config repeats 1 vs 6, 2 vs 5, 3 vs 1); **none** increase in new A/B pairs (wanderer 1-1, ellipse−3 3-3). Root cause noted: `aim_z_nominal` exception made for LOS spear in 2026-07-24, **visual handover not covered at all** → item 14 (fix in `simple_guided_follow.py`, not needed today).

**(C) DOWN=−3 asymmetry — HYPOTHESIS REFUSED.** Reverse overshoot at k=3 (+1.75) **Not repeating at k=2**: marked dz 8-0 m **+0.09 vs closed +0.92**. The "descension box (4.5) half of the climb (9)" claim was measured in three layers, **none binding**: cost reference ceiling in descent branch **0 of frame 674 clipped (%0)**; `cmd_vz` touching +4.5 %0.0-0.7; The MPF vertical slice is not specific to DOWN (%21 in −3, %20 in 0). Separating by direction would be a **no-op** → no code changes.

**OPENING REASON:** marked terminal vertical is now healing in cell **4/4**; MISS decreases in every cell; campaign's **3 physical contact 3** in this branch (vibe 234/41/40); No regression in DOWN +4/0/−3; The abort cost was proven harmless.

**HONEST RECORD:** the *letter* of the benchmark is not provided in the wanderer (CPA vertical 2.51) — but there the benchmark does not measure the vertical channel (1.86 of the closed arm is also excluded). And **k=2 vs k=3 NOT SOLVED**: k=3 is more consistent in the band metric (4/4), k=2 is two triples (two runs of the same configuration 0.19 and 2.78); Physical contacts that make you choose k=2. **Intra-configuration spread is greater than inter-cell difference** → n≥3 required.

**EXPERIMENT-5 (interrupted — user went to field).** 11 was run with `YILDIZ_SOLVER_ABUNDANT` (iter 26→40, budget 13→18 ms, **for sim measurement quality**); Verification of `ABUNDANT=0` (shipment setting) 2 remained in the run.

**IMPORTANT — HONEST CORRECTION:** EXPERIMENT-5 runs **markedly bad** from the headline of EXPERIMENT-3 (ellipse k=2: 0.41/0.15). |vertical| on CPA Now:

| route | k=2 | k=3 |
|---|---|---|
| ellipse (n=3) | 1.39 / **0.09** (HIT, vibe 72) / 1.59 | 1.74 / 0.13 / 1.50 |
| straight (n=2) | 0.11 / 1.80 | 1.10 / 0.97 |
| **pool hydrangea** | 1.39 (avg. 1.00) | 1.10 (avg. 1.09) |

**Result: EXPERIMENT-3's "CPA |vertical| < 0.4 m" cuff COULD NOT BE REPRODUCED.** The dissipation within the same configuration (0.09→1.80) is greater than the difference between the arms, that is, **k=2 etc. k=3 does not decompose** (item 11d remains open) and single condition values of CPA cannot be judged.

**But the mechanism works SOLID** — reliable signal CPA is not the only point, **band metrics**: in wanderer the closed arm does not turn off the vertical standoff *at all* (profile flat −2.34→−2.18), the main arm closes monotonously (terminal 2.18→0.59); MISS decreases in every cell; 3 of the campaign's physical contact 3 is on this sleeve. **Correct expression:** "vertical channel now sees and closes" ✅ / "0.4 goes below m on every run" ❌.

**SHIPMENT DECISION (2026-08-07, user):** altitude correction **will be used in flight**, but **turned on with ENV** — the default 0 remains:

```
YILDIZ_VERTICAL_ERROR=1 YILDIZ_VERTICAL_TGO=1   # ON in flight
```

`vertical_tgo_k=2.0` was preserved (contact generating arm; data was not sufficient to separate k). `YILDIZ_VERTICAL_TERMINAL` (old D-arm) and `YILDIZ_SOLVER_ABUNDANT` are closed (the budget is already tight on the latter Pi 5).

**ITEM 11e — WHY IS IT NOT THE DEFAULT (open job):** making the default 1 drops to `mpc_test` **77/86**. 8 of the remaining 9 test confirms the **OLD** design of the vertical channel — `alignment_ref` with 8 saturating at dps ceiling, `eps<0` with no bias (one-sided), `impact_alignment` mixed, `--impact_alignment_closing`. The campaign has changed all of this on purpose (the ceiling is now from the physical speed box; the bias is two-sided because altitude-agnosticism is desired). So the tests are outdated, the code isn't wrong — but since it wasn't right to hastily rewrite the 8 test expectation before flight, **the exact same setup verified in the sim** was shipped (all A/B runs were done with env). 9. test load sensitive timing measurement (20.85 µs vs 20 µs threshold). **Before turning on the default:** update this 9 test to the new design.

## ★ ALTITUDE VARIANT FINDING (2026-08-07, straight route, n=1/variant)

`--down` sign: **positive = fighter BELOW target** (`simple_guided_follow.py:1798`). The current default is `DOWN=+4` i.e. bottom tracking.

| | +4 (from bottom, available) | **0 (same altitude)** | −3 (from top) |
|---|---|---|---|
| Real CPA | 2.53 m | **0.67 m** | 1.42 m |
| CPA vertical residual | **+0.92 m** | −0.36 m | +0.64 m |
| Turnover range (median) | 23.7 m | **21.1 m** | 28.0 m |
| <6 m closing engagement | 2/8 | **7/13** | 1/10 |
| Near band (<25 m) detected | 63.5% | 69.0% | **86.8%** |
| MISS | 3 | 6 | 8 |

**Recommendation:** `DOWN=0`; If the fear of frame loss prevails, −1..−2 (slightly from above). Bottom tracking leaves unflying vertical residue in the terminal phase and makes the target's apparent ascent less than 72° (the speed-limited gimbal cannot keep up). **Note:** `tools/scenario.sh:18`'s "If you mash DOWN, the camera axis will be broken!" warning **no longer valid** — PHASE O tilt tracking absorbs geometry change (gimbal error the same across three variants: median 0.162-0.168°, p95 ~0.41°).

## WINDOW 45 → 25 SQUARE (2026-08-07, APPLIED)

Decision window is in **frames**, not duration. In the sim the camera would say 30 fps (45 frames = 1.5 s) but in reality YOLO ~20 Hz → the same 45 frames **2.25 s**, the cycle would be silently delayed. 25 frame: 30 0.83 s at fps, 20 1.25 s at Hz; rate remained at 80% (20/25). Env: `YILDIZ_WINDOW_FRAME` / `YILDIZ_WINDOW_RATIO`. **Side effect (measured):** straight route baseline CPA 2.53 m → **0.19/0.35 m** (one actual contact). There is practically no room for improvement on the straight; in open ellipse.

## ITEM 12 — CLOSING STOPS AT STOPPED/SLOW TARGET (2026-08-07)

During the hanging target run **there was no turnover**, the range remained at 150 m. The root cause is not on the display side: `simple_guided_follow.py:~3380` `d_allow = min(d_allow, cmd_speed_xy * budget_s + budget_margin_m)` — `cmd_speed_xy` The speed of the **TARGET**. When the target is stationary, `d_allow = 0*2+20 = 20 m`, ArduCopter internal P term gives ~3.4 m/s (for the moving target, it was 20 m/s). Second component: current rate in the two-copter world is 0.44. **Recommendation (NOT IMPLEMENTED):** scale the budget with the speed ability of the **hunter**, not the target (`max(|slot_vel_xy|, |pursuer_vel_xy|)`), or increase the range of `COMMAND_POSITION_BUDGET_MARGIN_M` at long range; The 20 m shield in the near phase (intentionally placed in 2026-07-31) must be protected. **Quick review:** `TARGET_DISTANCE=400 DURATION=420`.

## SOLVER BUDGET — FIELD WARNING (2026-08-07)

Column `budget_cut` was added and showed the corruption in the first job: **%65-80** in sim runs, `iter` p50 = 26 = `iteration_ceiling`, `duration_ms` p95 ≈ 13 = ms `duration_budget_ms`. In other words, the solver stops in most of the loops **not because it converges, but because the budget runs out**; command suboptimal. On the shim the knock still produces (not failure proof), but on the Raspberry Pi 5 it will be worse. **This is the first column to look at in the field.** Reducers are ready: reduce `--horizon`, `duration_budget_ms`, `--no-yaw`.

## CONDITION MONITORING
| # | article | situation | running | conclusion |
|---|---|---|---|---|
| 0 | replay harness | **FINISHED** | offline | mpc_test generates closed-loop occlusion: cross/lateral miss at 23 m, tail closes at 1.6 m |
| 1 | shutdown (q_area ↑) | **VERIFIED ON TWO ROUTES ✅✅** | mpcB_corrected_straight_224747 | timeout 2→0, shutdown 2×, CPA<3m 2→4; price: 0-5 m freshness %60→%42 |
| **1b** | Prize type: area vs SQUARE | WAITING (priority 1) | — | the square root arm was never measured; incentive 1/r³ → 1/r² softens |
| **1c** | J literature terms (PAMPC img-vel, ZEM) | WAITING (priority 1) | — | targets the framing loss bottleneck |
| **7a** | solver KKT residue metric | WAITING (priority 2) | — | NO optimality certificate; pushes the ceiling %46-70 |
| 2 | geometry gate | WAITING | — | — |
| 3 | scaling with horizon RANGE | **NOW VERIFIED ✅** | mpcB (1+3 together) | not parsed in sim alone; Won package 1+3 |
| **1+3** | **together** | **NOW VERIFIED ★✅** | A/B 22:37–22:55 | MISS timeout 2→0, shutdown −2.25→−4.28 m/s, minR 1.76→1.30 m |
| 4 | blind terminal PN | WAITING | — | — |
| 5a | vertical acceleration pitch | **[GIMBAL BRANCH: IT HAPPENED]** | — | pitch no longer enters camera axis (body ±35° → camera 0.65°) |
| **5b** | **dynamic TILT tracking** [NEW] | WAITING | — | eps=hang(down/r) exceeds static tilt in terminal; gimbal 6 is tracking rad/s |
| 6 | pitch delay | **[GIMBAL BRANCH: CLOSED]** | — | pitch is off camera axis; the remaining job is to feed the beta from tilt (correction, not adjustment) |
| 7 | solver budget | WAITING | — | — |
| 8 | bearing-angle TMA | **ON** (free to exit camera) | — | offline prototype next; ceiling riser, not a necessity |
| 9 | measurement repair | WAITING | — | — |
| 10 | actuator model | WAITING | — | — |
| **11** | **vertical channel blind after PHASE D** | **CRITERIA SATISFIED ★ (EXPERIMENT-3)** — default still OFF | EXPERIMENT-1: 6, EXPERIMENT-2: 12, EXPERIMENT-3: ~14 running (2026-08-07) | `P+TGO k=2`: ellipse 2.02→0.41/0.15, plain 0.30→0.06, DOWN=0 0.18→0.09; **the campaign's only physical contact of 3**; WSC 10→2-4. EXPERIMENT-4 (wanderer + abort diagnosis + DOWN=−3) is running |
| **11b** | heading-specificity: vertical now only on ROTATING target | ON (new) | flat vs ellipse 2026-08-07 | flat base 0.13/0.30 m (criterion met), ellipse 2.02 m — hypothesis: disturbance d_ey in rotation feeds PN, “parallel travel” locks vertical offset |
| **11c** | `budget_cut` %52-82 on ALL arms | **EXPERIMENT-5 is running** | all runs 2026-08-07 | FISTA fails to converge in 2/3 of the runs — noise on top of each measured A/B difference; `YILDIZ_SOLVER_ABUNDANT` (iter 26→40, budget 13→18 ms) testing |
| **11d** | selection of k=2 vs k=3 | **ON** (n≥3 required) | ellipse+4 n=2/arm | the intra-configuration spread (0.19-2.78) is greater than the inter-cell difference; 11c cannot be decided without breaking |
| **13** | CLOSE fault in wanderer (not vertical) | **ON (new)** | wanderer A/B 2026-08-07 | fighter 15.5 / target 15.0 m/s, speed clamp %0 → MPC does not choose to close (recurrence of "three abreast" on rotating target); engagement 13-37 breaks at m, MISS 4/5 "range opening" |
| **14** | Reference `altitude_abort` measures wrong size | ON (harmless, hygiene) | 67 abort classification | `simple_guided_follow.py:3171` is looking at positioned slot z; ~5 s must be disarmed while in video authorization and after handover (same as done for LOS spear in 2026-07-24) |
| **15** | eps measurement noise puts terminal base | OPEN | r<20 %23-44 of m frames | actual rise difference with bbox eps >2° (0.52 m on 15 m); acceptance threshold 0.5 m right at this base — if the criterion is to be tightened, the bbox vertical center must be corrected first |
| **12** | stopping/slow closing stops on the target | ON (on positioned side) | hanging target 2026-08-07 | `d_allow` scaling with target's speed → 20 m clamp → 3.4 m/s |
| **16** | APN: estimate target lateral acceleration + arc to horizon (`YILDIZ_APN`) | **PARK — flag will remain OFF** (code remains) | apn1_ellipse_20260808_183316, apn1t_ellipse_20260808_185351 | `a_perpendicular` DERIVATIVE term NOISE appeared. Derived from actual reference `v·ω` (`ref_target_ax/ay` columns BROKEN, max 0.13 m/s²): predicted-actual correlation **+0.05 / +0.11**; \|a_perpendicular\| estimated p50 **1.3-1.8** vs actual **0.01-0.02** m/s²; The frame with real \|a\|>0.5 in STRAIGHT phase is **%0.0** while `apn_a` **%70-73** is active; only **%38-43 of the assets are correctly marked** (worse than a coin toss). Root cause: `v_perpendicular` RMS error 6.5-8.8 m/s, 0.6 s derivative with LPF ≈10 m/s² noise → ±6 based on clamp. The first order term is ROBUST (v_perpendicular correlation +0.62/+0.75). Behavior with flag off is BIT-SAME (500 on random input `array_equal`, Scale/Hessian does not change) |
| **17** | Yaw sustain on lever `hold_value` (`YILDIZ_HOLD_YAW`) | **TESTING ISOLATED** | tyaw1_ellipse_20260808_190812 (+tyawacc, flat regression) | In apn1t it was mixed with APN, it is being parsed. The mechanism works: the yaw command goes in **%100** of frames `hold_value` (previously %0), in the frame without yaw STRAIGHT **%18.0 → %7.7**. Finite (τ=1.0 s) + limited time (1.5 s), does not touch the arm `coast` |
| **18** | WPNAV_ACCEL NOT GIVEN to vehicle (file 500, vehicle 250) | **MEASURED → restoration test** | MAVLink 14551 reading param 2026-08-08 | It says `params/swarm_copter.parm:55` **500**, the vehicle read **250**; `WPNAV_ACCEL_Z` also 500→250. `WPNAV_SPEED` fits 3500. Not overfit, but restoration of lost parameter (see memory "eeprom crushes parm file") |
| **25** | Tumble/DEPARTURE: exiting with recovery timer → **HOLD altitude gate APPLIED (default-ON)** | **APPLIED ✅ — flight verification in progress (kpn26c)** | kpn1/kpn_straight2/kpn26/kpn26b (4 somersault) vs 8 solid running, 2026-08-09 | **FINDING: The only dimension distinguishing 12/12 is CONTINUOUS ALTITUDE ERROR** — alt_err p95 **56-57 m** (tumble) vs **19-33 m** (intact); alt_err>15 m ratio **%66-91 vs %9-18**. All runs exceed 15 m at some point; The distinguishing factor is the **failure to recover**, rather than crossing the threshold. **MECHANISM (runaway loop):** altitude error → aggressive vertical+horizontal demand → large roll (tilt before rollover **median 67°**) → loss of cos (vertical component 0.59 at 54°) → throttle is already **fully saturated in 10-23% of frames** (ThO=1.00, versus hover throttle 0.35) → altitude cannot be held → error grows → departure. **ROOT CAUSE (code):** `ALTITUDE ABORT` cuts horizontal correctly (`send=False, speed_cap=0`) **but exit from HOLD WAS TIMER ONLY** (`hold_s`=2 s; extension `spinning` only). kpn26: t=25.8 HOLD alt_err 15.6 → t=30.8 REENGAGE **12.9** (13 m speed reverse with error) → t=34.8 CHASE **20.2**; somersault t=32.3. Also abort `:342` **disarms itself** (re-arm requires `alt_err ≤ 3 m`, never happens in roll runs). **CORRECTION:** `altitude_invalid` condition to `simple_guided_follow.py` HOLD output — same pattern as `spinning`, same `hold_max_s`=5 s ceiling, same fail-open behavior, **no new threshold** (`alt_abort_arm_m` already 'altitude recovered' criterion). `YILDIZ_HOLD_ALTITUDE=0` old behavior. Unit testing: alt_err None/0.5/2.9 → 2.0 s (**identical to the previous behavior**), 3.1/20 → 5.0 s (extension, ceiling locked). **REFUNDED SIDE HYPOTHESIS:** MOT_THST_HOVER/EEPROM-erasing is NOT the culprit — ThH everywhere 0.390-0.407 (also in runs 2026-08-01), measured hover throttle 0.28-0.35, so ThH is already correct; **If the 0.68 assert was added, it would inflate the forward feed 2× and be harmful**. **OPEN CANDIDATE (NOT DONE today, 'second button no-op' lesson):** `RECOVERY_HOLD_MAX_S` 5 → 8 s. The second button will not be touched until a single variable is measured |
| **24** | VERTICAL SEPARATION (dz<0): all four adjustment knobs REFUSED → **STRUCTURAL design type required** | **ON — new design type** | sep2, d0kpn, dv1 (+ kpn3b/kpn4 base), 2026-08-08/09 | **PATTERN:** Hunter BELOW target in CPA — dz<0 **%76 (26/34)** in corrected pool BLIND_PN; same in raw pool and DOWN=0. **FOUR BUTTONS TESTED AND REFUSED:** **(1) vertical saturation actuator** (`YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5`, sep2): abort/engagement 0.69→**1.11**, `|cmd_vz|` ceiling %5.4→%20.3 — made worse, defaults neutralized. **(2) DOWN=0 standoff** (d0kpn): |dz| DID NOT land on tgoK20d0 band (0.04-0.11), all CPA |dz| p50 1.62 = unchanged, **and dz<0 still 9/10** → underrun does NOT come from standoff. **(3) forward vertical ramp window** (`YILDIZ_VERTICAL_RAMP_START=80/LAST=45`, dv1): **PROVEN NO-OP** — signature measurement (range at which `vertical_error` is first non-zero) dv1 28.5 m vs kpn3b 29.6 / kpn4 26.1 m, i.e. UNCHANGED; arithmetic reason: `s=clip((START−r)/(START−LAST),0,1)` and **speed range R0 p50 ~25 m**, at that point the window 45→25 and the window 80→45 are already **s=1.00 (full saturation)**. Engagement begins at the saturated end of the window — the ramp is not limiting at all. **(4) (1)+(3) package** (dv1): guard triggered (abort/engagement 1.11 = sep2 signature), CPA |dz| p50 3.59 (worst in pool), dz<0 %78. **RESIDING ROOT CAUSE (analysis agent, still valid):** vertical channel dead time ~1.5 s (measured τ_z 2.16 s, model 1.0; demand execution before CPA %11) It rises and hits the ceiling. **CONCLUSION: this is not solved by the tuning knob.** Candidate structural aspects: (i) separate **lead/prediction** term for vertical (generate demand from predicted dz and t_go, not from eps — bypasses angular birth lag); (ii) **increasing the rev range** — all the vertical work starts at ~25 m, because that's where the visual authority gets; if the turnover threshold (~%6 coverage) is relaxed, the vertical budget also grows; (iii) treating the vertical channel at the MPC horizon with separate weighting. They all call for a new type of design |
| **22** | PN MAINTAIN IN BLIND COAST (`YILDIZ_BLIND_PN`) | **IMPLEMENTED + FIXED; n is increasing** | kpn1b/kpn2/kpndplain (raw), kpn3b/kpn4 (R3′+range gate) | Root cause: in the last 5 m ~%78 of the loops were bboxless and **the last command was being repeated** in that blindness (`_blind_command` repeats the last command verbatim, the solver does not run at all); whereas the range channel is independent and fresh from bbox. Lever: **advance ex/ey from inside **dead reckon** (`c2=KDEG/r` grows as it closes) and keep solver running — continue **law**, not command. Rails R1 duration 1.0 s, R2 d_ex end to zero with `exp(-t/0.7)`, R3 `|ex0|+5°`, R4 range freshness, R5 touch yaw. **RAW VERSION REGRESSED ON RETURN** (closing +3.87 → −7.93/−2.89/+0.28, fresh %74→%46-58): dead reckoning target out of frame was dragging (blind \|ex\| MAX 28.3/46.9/36.4 vs frame half-angle 20.07) because the base of the R3 is the "last real \|ex\|" was and on the return it is already 30°+. **FIX (R3′ + range gate):** `\|ex\| ≤ min(\|ex0\|+5, blind_pn_ex_absolute_deg=20)` (`YILDIZ_BLIND_PN_EX_ABSOLUTE`) + `internal_range ≤ blind_pn_range_m=12` (`YILDIZ_BLIND_PN_RANGE`). Result (kpn3b): blind \|ex\| MAX **20.0** (not exceeded even once), blind_pn activity %100→%52 and remaining in terminal (internal_range p50 3.1 m), **RETURN closing +2.36 / opening %31.5** (even better than pt1b), ALTITUDE ABORT 6 (lowest of the day), `\|cmd_vz\|` ceiling %4.2, all CPA \|dz\| p50 1.64. Bitwise equality (flag off) max\|difference\| **0.000e+00**; mpc_test without flag 86/86. **EXPRESS COST:** Sub-meter transitions lost in kpn3b (nearest 0.73/0.75 → 1.47) — kpn4 replication will distinguish this. **ADJUSTMENT CANDIDATE (tomorrow, not run):** stretch absolute clamp **20 → 26°** (`YILDIZ_BLIND_PN_EX_ABSOLUTE=26`). Reason: old gains were from blind advance in band 20-28° (kpn1b made minR 0.73 with MAX 28.3); The real damage was leaks 30-47°. 26 still cuts the leak, allowing legitimate overflows in the terminal. SECONDARY candidate for loosening the range gate |
| **23** | MEASUREMENT INFRASTRUCTURE: plan-compliance gate | **APPLIED ✅** | tools/plan_alignment.py + scenario.sh:148-168 | **TWO TRIES BURNED SILENTLY**: `PLAN=` is only loaded into the tool on the `RESTART=1` branch; With `RESTART=0` (our standard convention for maintaining the GUI) `PLAN` was **doing nothing** and the target was flying the task at stack opening (default ellipse). Moreover, the screen said `">>> target aircraft: AUTO, straight route"` — the label came from `PLAN_NAME`, not the one installed on the vehicle. Victims: `tyawacc_straight_straight_20260808_193448` (yesterday's "E", **"straight regression PASSED" ruling withdrawn**) and `kpn_straight_straight_20260809_123228`; both flew ELLIPSE (target position spread 1177/696 and 986/695 m; E-B on truly flat runs **4-8 m**). 16 old straight run (b5min*, dt*, mpc*, tgo*) SOLID — they are from the `RESTART=1` era. Gate: downloads the loaded task from MAVLink and compares it from **geometry** (N-S scope, E-W scope, DO_JUMP) — waypoint count is not used (home item may change during loading). Incompatible → **EXIT 1** + resolution line; `SCENARIO_PLAN_CONTROL=0` jumps; If it is unreadable, it warns and makes the label "(not verified)". Live test: flat→1 INCOMPATIBLE, ellipse→0 COMPATIBLE |
| **21** | INTERSECTION approach (for image phase) | **REJECTED** — keep button, default `slot` | KES1 (kes1_ellipse_20260808_231825), `APPROXIMATION=intersection` | **MEDITED the ten conditions but did NOT produce contact — an instructive rejection.** The intercept cycle actually feels faster and diagonal: close0 p50 **6.92 m/s** (slot ~4.4), |aspect0| p50 **135°** (slot ~171). However, minR p50 **14.57 m** (slot arms 6.4-12.6), <3m 1/14, best 2.66 m, and **4 with engagement "range opening" broke** (0-1 in slot). Since the intersection does not brake, the visual overshoot cannot recover. **INFERENCE: The "high rpm shut down → good contact" correlation was valid IN slot geometry; breaks when causally challenged by crossover geometry.** SIDE GAINS (notable): (1) **straight-phase fresh% 97.7 — campaign record** (slot 63.8-67.9), but reverse on return: fresh%62.2, |ex| p50 21.1 (slot 7-11), closing −4.28 (opening). (2) Positioned phase ALONE **CPA 1.33 did m** (vibe max 5.80, no contact) → bottleneck TERMINAL CONVERSION. (3) Overshoot/recovery healthy: 9 transition cycle, 8 re-engagement, overshoot→new engagement p50 12.4 s; The target does not stay behind and wait for a tour. INFRASTRUCTURE: `tools/scenario.sh:187-199,208` **APPROACH passthrough** added (bossa bit-same; `--approximation` was already in `simple_guided_follow.py:1985`). FORWARD: **hybrid "intercept-approach / slot-handover"** may be considered — taking the straight-phase freshness and closure advantage of the intercept and entering the terminal with slot geometry; not now |
| **20** | SATURATING ACTUATOR model (`YILDIZ_ACTUATOR`) | **APPLIED, VALID MEASUREMENT IN SIM n=1** | eyl1_ellipse_20260808_202707 (sep1b INVALID: roll oscillation → 6× ABORT IN ALTITUDE → ground contact) | Measured root cause: MPC `speed_latency_tau_s`=1.00 assumes s + **unlimited acceleration**; real vehicle acceleration limited (horizontal plateau **~4 m/s²**), in the studied band (|to|=10-20 m/s) actual tau **5.5-6.8 s** → MPC plans unattainable command. Correction: successive linearization of saturation, `tau_eff_k = max(tau_lin, |u_nom_k−w_nom_k|/a_max)`, tau_lin 1.7 s / a_max 4.0 m/s², **horizontal only** (vertical in old tau → `_cbf_boundaries` untouched). Measured effect: `u_saturation` return %40.8→%14.9, flat %36.4→%11.0. Solver cost **+%3.5** p50. Bitwise equality: `array_equal` in input 400. **NOTE: WPNAV_ACCEL 250↔500 did NOT change the plateau** — GUIDED speed-setpoint path passes through PSC, `WPNAV_*` is the parameter of the waypoint controller. **VERTICAL EXPANSION TRIED AND REVERSED (sep2, 2026-08-08 night-2):** horizontal-only implementation left the vertical channel artificially CHEAP and shifted solver demand there (`|cmd_vz|` p90 6.38→9.00 ceiling, ALTITUDE ABORT/engagement 0.36→0.69). Vertical was also measured (**tau_lin_z 2.16 s, plateau 5.25 m/s² = WPNAV_ACCEL_Z 500**; i.e. vertical is SLOWER than horizontal, the model thought it was FASTER with 1.0) and applied symmetrically — **IT DID NOT WORK, MADE WORSE**: abort/engagement **1.11** (campaign record), `|cmd_vz|` frame rate at ceiling %5.4→**%20.3**. MPF is not guilty (force-descension dropped to %7.6→%2.2, box opened, command is on the ceiling again). **Inference: it's not the actuator model that sets up the vertical demand, it's the cost/geometry side (vertical standoff + eps chase) — work for a separate iteration.** Defaults are neutralized (`tau_lin_z`=1.0, `a_max_z`=∞ ⇒ vertical saturation off), so `YILDIZ_ACTUATOR=1` = proven HORIZONTAL-ALONE arm; measured values are opened with `YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5`. **MEASUREMENT TRAP (for the record): `|u3|` does NOT measure vertical demand** — u3 is the e3 component in the plane LOS, and `e3=(sin ε cos ex, sin ε sin ex, cos ε)` the horizontal share of e3 grows as ε grows. For vertical demand use **`cmd_vz` (NED)** |
| **19** | 8 s WATCH timeout cuts while CLOSING | **CLOSED — prolonged time PRODUCING NO CONTACT (evidence in scenario 2). Code stops, flag defaults to OFF** | H1b (olcag1b, clock off) vs H2b (olcag2b, clock on), 2026-08-08 | **Lever operated but NEVER FIRED: stagnation clock p50 0.46 / p90 1.94 / MAX 4.14 s (threshold 8); 6 s over frame 0/1092.** H1b 2/13 → H2b 0/12 timeout difference DOES NOT BELONG to arm, from engagement time distribution: in H1b >4/13 with 8 s, 1/12 in H2b. **Counterfactual column complete 0.00 m in both arms** (minR@8s → final; minR always occurs within the first 8 s) — conclusive evidence. Why: the evidence base of the arm was lap-1 (15 was cut while closing timeout 4), those runs were in the OLD profile and engagements were running up to the wall clock; In the new profile+actuator combination, engagements end spontaneously at 6-8 s, meaning **the targeted pathology is NOT present in this setup**. The branch is not incorrect, but this scenario does not exercise it. Design note: progress measure **best-so-far recovery rate** (monotonic → immune to oscillation); instantaneous and windowed raw range versions tricked out with `r=25+2·sin(2t)` **[CLOSE, CUT1 2026-08-08]** In the intercept scenario the arm is NOW FIRED (still max 8.02 s, >6 s frames %3.4, an MISS `time_value asimi/durgunluk (8.0 s > 8; angajman 10.7 s)`) — so the "scenario is not testing" excuse is gone. **Counterfactual gain 0.00 m** (extending 2/14: minR@8s 19.62→19.62, 22.53→22.53). Same result in two independent scenarios (slot H2b + intersection KES1): minR always occurs within the first 8 s, extended time does not ensure descent to the contact band. Article CLOSED. | |
| 19a | (design/proof record of 19) | — | tur-1 pool (base8+apn1+apn1t, 34 engagement) + synthetic | EVIDENCE BASE: **4 (%27)** of timeout 15 was interrupted while the vehicle was STILL shutting down (last-2s slope -4.11/-3.59/-2.11/-1.34 m/s; r = 8.8/21.6/13.6/25.6 m); On the remaining 11 the range was actually opening (+0.36...+7.31 m/s). SYNTHETIC VERIFICATION: strong/intermittent shutdown 8.1 -> 22.1 floating s, quiescent 8.05 s and **oscillating-stationary 8.25 s**. SIDE FINDING (independent of item 19, still valid): strongest correlation with minR **\|aspect shift rev->CPA\|**, correlation **-0.577** -- closest passes come not from tail tracking, but from swinging next to the target with large yaw and closing from there |
| — | window 45→25 square | **APPLIED ✅** | flat baseline 2026-08-07 | CPA 2.53 → 0.19/0.35 m; 20 Hz is also the correct time for YOLO |
| — | altitude variant (down 0/±) | **MEASURED ✅** | 3 running 2026-08-07 | DOWN=0 best (CPA 0.67 m); tracking from below leaves vertical residue |
