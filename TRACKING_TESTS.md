# TRACKING_TESTS — ArduPilot tracking law (`tracking_guidance.py`) sequence of experiments

Session: 2026-08-05. Scope: **the archived ArduPilot FOLLOW configuration arm only.** The test list of MPC is in a separate file (`TO_TEST.md`) and **it is left untouched**.

## OWNERSHIP RULE (the reason for this file's existence)

User rule: *"If you think there's even a slight chance it could belong to MPC, leave it."*

Each item in this file is **a change to `guidance_allstar/tracking_guidance.py` or `tracking_test.py` only**. This deduplicates ownership by definition: MPC has nothing in those files. If the IDEA UNDER an article can be useful to MPC, I write this clearly, but **I do not own the article** - MPC evaluates it independently in its own list.

### THOSE NOT INCLUDED IN THIS FILE (may belong to MPC → keep it at `TO_TEST.md`)

| Item TO_TEST | Why didn't I buy it? |
|---|---|
| 1, 1b, 1c — J function / reward type | cost function MPC |
| 3 — cost horizon, 7 — solver metric | there is no horizon/resolver in the chase but the item is MPC |
| 6 — `beta` pitch delay | Framing variant of `beta` MPC |
| **2 — geometry requirement for speed gate** | `bbox_to_redis.py` **joint file** feeds into both laws |
| **4 — blind terminal "Return speed LOS, resume PN"** | Blind-terminal arm of MPC; my T5 is in a different place (predictor of tracking) |
| **5b — dynamic tilt tracking** | gimbal **common**; tilt command is in `bbox_to_redis` |
| **8 — bearing-angle TMA** | law-independent estimator falls into both |
| **9 — measuring tools**, **10 — actuator model (τ)** | `mpc_test.py`/`tools/` common |
| 0 — replay harness | Designed for MPC solver |

**Note (not T-item, notice):** item 10 proposes to change the actuator model of `mpc_test.Simulation`. That engine is also the basis of this file → `tracking_test.py` test 0 (**engine seal**) 9 seals the physics constant; If item 10 is applied, the test fires and all base numbers there are remeasured.

---

## USER PERSON AND PURPOSE (2026-08-05 — this list is anchored accordingly)

1. **THE MAIN PURPOSE IS CRASH.** Framing/centering/shake metrics are tool metrics, not the goal. The **primary** acceptance criterion for each item is collision (ratio CPA ≤ 2 m or `finish == collision`); framing etc. It is pronounced as **side-effect**.
2. **SOLUTION SPACE:** user view — the solution for visual guidance is either in **code within ArduPilot that can be applied to visual guidance** or in the code **MPC**; No solution is expected other than this. → Each item now states **which ArduPilot/MPC mechanism it is based on**. “Importing” academic law is not considered a justification in this list; at best it is a footnote as *verification/quantitative reference*.
3. **TESTED AND NOT WORKED (do not try again):** *Precise Interception of Flight Targets by IBVS of Multicopter* (arXiv 2409.17497) — the user read this article and **compiled it into code, it did not work well in practice**. In this list, the holistic architecture of that article (IBVS+PNG shell) is **not used as a source**. Only two of his observations remain as background footnotes (separate consideration of the yaw channel; relationship of speed increase to `arctan(a/g)`) — both already exist in their counterparts in ArduPilot, and the articles are based **there**.

---

## WORKING RULES

1. **RANGE only from telemetry** (low confidence). **Extraction from the camera is FREE.** Ground detection is in a separate branch. Reminder: follow's command path doesn't use **any** range (evidence: `tracking_test` 6b/11), neither does the referee with `--miss-source area_value`. **If a new article violates this feature, the reason must be written.**
2. Don't touch shared files while your sim is running. One sim at a time. **There is another agent in the sim now** — offline items first.
3. Agent: solo opus or orchestrator.
4. Each item: single knob + measurable acceptance criterion + offline→sim space marked.
5. A/B before changing default; If the default changes, the `ARDUPILOT_TRACKING.md` tables are refreshed.

---

## BASE (reference of acceptance criteria)

Offline, `mpc_test.Simulation` in gimbal physics, 6 scenario × 3 seed, 40 s, min range median (m) — `ARDUPILOT_TRACKING.md` §6e:

| arm | flat/well | flat/diameter | ellipse/well | ellipse/diameter | wand/well | wand/diameter |
|---|---|---|---|---|---|---|
| current default | 1.35 | 1.69 | 1.87 | **2.89** | 4.85 | **13.24** |
| + visual referee | 1.35 | 1.69 | 1.87 | 2.89 | 4.85 | **11.54** |

Framing loss: `0 / 0 / 0 / 10.1 / 19.3 / 2.1 %`. Sim (BEFORE gimbal, body-fixed camera): hanging 0.5 / infinity 1.2 / ellipse 0.2 / wanderer 5.5 m. **Gimball sim run NO YET** — T6/T7 are there.

**The remaining two bottlenecks are:** (a) wanderer/diagonal 11.5–13.2 m — lateral delay, (b) ellipse/diagonal ending with frame loss (%10.1).

---

## BASED ON: MECHANISMS AVAILABLE IN ArduPilot and MPC

This is the basis of the articles in accordance with the constraint 2. All read on local copy (`~/ardupilot`, 2025-07-13) or available in our own MPC.

| mechanism | where | What gives tracking? |
|---|---|---|
| `AC_AttitudeControl::input_shaping_angle` | `AC_AttitudeControl.cpp:1103` | **Correct form of Yaw.** Acceleration clamp with `desired_ang_vel += sqrt_controller(error_value, 1/ATC_INPUT_TC, ATC_ACCEL_Y_MAX, dt)` then `input_shaping_ang_vel`. **Forward feed slot is already in the signature** (ArduCopter puts `ang_vel_target` there) → T1 |
| `AP_Follow` / `mode_follow` term `vel_of_target` | `mode_follow.cpp` | **Lead mechanism.** This was the term we deleted; If it is put back, the pure pursuit lag is structurally closed → T2 |
| `AC_PrecLand` EKF + dead account | `AC_PrecLand.cpp:514,552` | Resume target when frame drops → T5 |
| `AC_PrecLand` `land_slowdown` door | `mode.cpp:667` | "Approach without alignment" gate → T8 alternative |
| `AP_Math::shape_vel_accel` / `sqrt_controller` | `control.cpp` | Already ported; Derivation of T3 |
| MPC `DisturbanceEstimator` | `mpc_guidance.py:1435` | **target angular motion** estimation from bbox; It generates our own yaw speed. Estimation source of T2 (as a method; the code is written in its own tracking) |
| MPC ISS state machine | `mpc_guidance.py:2579` | Already copied (same thresholds) |

**Measured mismatch (reason for T1):** In `params/swarm_copter.parm`, `ATC_ACCEL_Y_MAX = 72000` cdeg/s² = **720 °/s²**, i.e. the vehicle is set for too fast yaw acceleration. In contrast, the skeleton (`visual_base`) clamps the yaw command to **120 °/s²** — **6× slower**. That handcuff is in the COMMON FILE, it cannot be overcome through tracking; T1 measures and reports this, the decision to change is shared.

### Background footnotes (NOT basis, quantitative reference only)

- *Towards Safe Mid-Air Drone Interception* (arXiv 2405.13542): 100 in orbit **pure pursuit %72 success / first contact 32.2 s**, PN mixture **%100 / 5.93 s**. Our law is pure pursuit → **quantitative** justification of T2. (We're not taking the law from there; we're putting back ArduPilot's own FF term.)
- Tau/time-to-contact: `(dA/dt)/A = 2/TTC` — the size we already used in the referee; T8 asks to move this to the speed channel as well.
- Single axis gimbal literature: tilt **does not compensate for roll** → T9 measurement item.
- *IBVS of Multicopter* (arXiv 2409.17497): **user tried, did not work well in code** — not used as architecture (see Constraint 3).

---

# ARTICLES (order of priority)

## T1. SWITCH YAW TO ArduPilot's OWN SHAPE [offline, cheap]

**Based on:** `AC_AttitudeControl::input_shaping_angle` (`:1103`) — This is how ArduCopter drives yaw angle:
```cpp
desired_ang_vel += sqrt_controller(error_value, 1.0/ATC_INPUT_TC, ATC_ACCEL_Y_MAX, dt);
desired_ang_vel  = constrain(±ATC_RATE_Y_MAX);
return input_shaping_ang_vel(previous_value, desired_ang_vel, ATC_ACCEL_Y_MAX, dt, 0);
```
We have `_yaw()` **raw P**: `4.5·ex`, 60 dps clamp. So we don't have three things that are in the autopilot's own channel: (a) `sqrt_controller` shape, (b) **feedforward slot** (`+=` — ArduCopter puts `ang_vel_target` there), (d) acceleration clamp with ArduPilot's `ATC_ACCEL_Y_MAX`.

**Problem (measured):** `|ex|` p95 in sim **24.6°** (horizontal semi-FOV 33°) → margin goes down to 8°; **%8–37 of the framing losses are from the side edge**. The gimbal does not resolve yaw (single axis). It also clamps with the skeleton command **120 °/s²** while `ATC_ACCEL_Y_MAX=720 °/s²` (6× is slow, in the common file).

**Arm:** `_yaw(ex)` → exact port of ArduPilot's chain; `sqrt_controller` is already ported in `tracking_guidance.py`. The **bearing speed measured from the image** is put into the feedforward slot (with our own yaw speed subtracted — MPC Same method as `DisturbanceEstimator`, written into the code follow itself).

**Accept (primary = IMPACT):** wanderer/cross min range base **below 13.24**; The rate of runs ending in collision increases. *Side-effect:* `|ex|` p95 <15°, side framing loss margin decreases, yaw command step-difference rms does not increase. **Also report:** does the skeleton's 120 °/s² clamp connect (if it does, the 6× mismatch with the ArduPilot setting is carried over to the user — common file decision). **Risk:** derivative noise → LPF required. **Intelligence:** medium-high. **Where:** offline.

## T2. `mode_follow`'S DELETED FEED FORWARD TERM RESTORED [highest ceiling]

**Baseline (ArduPilot's own code):** `mode_follow.cpp`'s law
```cpp
desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P;
                       ^^^^^^^^^^^^^ THIS TERM WAS REMOVED
```
We deleted `vel_of_target` because it was coming from **telemetry** (canon). We measured the result and it determined the architecture: pure P cannot hit the moving target, so we attributed the magnitude to the cruise ceiling (`speed_source='ceiling_value'`). But **the direction is still pure following** — no leading.

**The new version of the rule reinstates this term:** I leave the camera FREE. In other words, ArduPilot's own term can be put back by predicting `vel_of_target` **from the image, not from the telemetry**. This isn't "importing law from outside", it's **putting back the line we deleted**.

**Estimation source (method from code MPC):** MPC `DisturbanceEstimator` (`mpc_guidance.py:1435`) estimates the angular movement of the target from bbox and extracts our own yaw speed. The same method is written to follow itself. (Full version — bearing-angle TMA — `TO_TEST.md` item is in 8 and **COMMON**; I do not own it, I will consume it if they produce it.)

**Out of range warning:** The component of `vel_of_target` perpendicular to LOS prompts `r·λ̇` → goes into range. Apply in **bite form** to maintain rangelessness: rotate direction vector by `δ`, `δ̇ = K·λ̇`, `|δ| ≤ δ_max` (yaw angle form). Thus `r` simplifies. Two arms A/B: (a) sting form (non-range), (b) `r·λ̇` form (ranged, more true to ArduPilot) — **measured which one hits harder.**

**Accept (primary = CRASH):** wanderer/cross **<9 m** and/or collision rate increase; ellipse/diagonal ≤2.89 m maintained; NO flat/tail regression (≤1.5 m). *Side-effect:* framing loss should not increase (leading pushes target aside — main risk, should be measured **together** with T1). **Risk:** high `K` → oscillation + frame loss; Limit to `δ_max`. **Intelligence:** high (orchestrator). **Where:** offline → sim. **Quantitative reference (not anchor):** pure pursuit %72 / PN mix %100 success (arXiv 2405.13542).

## T3. MAKE ACCELERATION FORMING INTO PHYSICS [cheap, cleaner]

**Status:** `acceleration_shaping_mps2 = 3.0` **empirical** selected (scan: 0/2/3/5 → frame loss %50-72 / %0-8 / %0-23 / %7-41).

**Literature:** The IBVS article derives the same problem: limit the acceleration to `Δq_d ≤ arctan(k_a/g)` (`k_a ≈ 1–3 m/s²`), i.e. there is closed form between **admissible error LOS** and acceleration. We have the equivalent: `Δpitch ≈ arctan(a/g)` → vertical semi-share FOV. Since the gimbal has deleted the pitch, now **constraint is horizontal**: roll angle produces `arctan(a/g)` roll, roll is NOT compensated in single axis gimbal.

**Handle:** fixed `a_max = g·tan(margin_angle)` instead of 3.0; Let `margin_angle` be derived from the horizontal margin FOV. Single knob: `margin_angle`. **Acceptance:** same framing loss ≤ present, no min range regression, number now **derived** (automatically updated when mount/FOV changes). **Risk:** low. **Intelligence:** medium. **Where:** offline.

## T4. FRAME DELAY COMPENSATION [cheap, unmetered]

**Problem:** `Measurement.t_capture` (frame capture instant) is given by the skeleton but tracking is not **used**; The command is generated based on a delayed bearing. In sim, cycle 20 Hz, camera 30 Hz → 30–80 ms typical.

**Literatur:** The IBVS article puts a separate **delayed KF** for this ("to mitigate image processing delays").

**Arm:** Move `ex/ey` forward within `command_value()` by `(t_now − t_capture)` at **our own angular velocity** (own telemetry only — rule clear). Second branch: full prediction with `ė_x` of T1. **Acceptance:** `|ex|` p95 drops; latency compensation off/on No min range regression in A/B. **Risk:** false sign → indecision; sign test is required. **Intelligence:** medium. **Where:** offline (does the engine produce `t_capture` — verify that first; if not, the item moves to **sim**, NOTE WILL BE OFF).

## T5. MAINTAIN TARGET IN BLIND PHASE (PrecLand EKF) [base 10.1% framing loss]

**Problem:** When bbox gets stale, tracking's prediction changes; The skeleton shifts to gliding. ellipse/diagonal now ends with **frame loss** (%10.1).

**Source:** `AC_PrecLand` EKF per axis + inertial dead reckoning: `_target_pos_rel_est_NE −= inertialNavVelocity·dt` outlier elimination with APR. In our case, it is placed next to `_target_estimate()`.

**Arm:** maintain target relative position **at our own speed** dead reckoning (NO target speed — rule), fix when fresh bbox arrives. `u_los` is maintained during the blank period, not frozen. **Acceptance:** the rate of runs ending in frame loss decreases; ellipse/cross ≤2.89 m maintained or improved; No command direction jump in blind phase. **Risk:** dead reckoning scrolling in long blindness → duration gate (e.g. >1.0 release after s). **Intelligence:** high. **Where:** offline. **Note:** `TO_TEST.md` item 4 is the blind-terminal arm of MPC; This is in a separate place (predictor of pursuit), I do not own it.

## T6. SIM VERIFICATION OF VISUAL REFEREE [evidence gap]

**Condition:** `--miss-source area_value` measured offline **free** and **good** from range judge on gimbal wanderer/crossover (13.24 → 11.54). But the offline engine doesn't trigger the quality gate (`min(w,h) ≥ 9 px`) at all — **not verified on real camera**.

**Arm:** `VISUAL_GUIDANCE="tracking_guidance.py --miss-source area_value"`, ellipse + wanderer in sim. **Acceptance:** reasons for miss are reasonable (ratio arm fires, "too small" arm does not misfire); min range no worse than range-judge run; Diagnostic columns `s_lpf`/`s_peak` are significant. **Risk:** real bbox noise may jitter rate test → debounce already exists. **Intelligence:** low-medium. **Where:** SIM (waiting in line).

## T7. SIM VERIFICATION OF PROGRESS CLOCK [evidence gap]

**Status:** defaulted; offline ellipse/diagonal 21.4 → 5.9 (fixed to body), 20.6 → 2.89 (gimbal). The pattern was also confirmed in the actual sim logs (while closing 79% of timeout misses in tracking). **But no sim running with gimbal.**

**Arm:** measure with T6 on the same run (comparison arm `--miss-time-source straight`). **Acceptance:** closing rate median of timeout misses ≤0 (we no longer cut the winner); Number of engagements decreases, min range per engagement improves. **Intelligence:** low-medium. **Where:** SIM.

## T8. TAU (TIME-TO-CONTACT) BASED SPEED PROFILE [open, mid-ceiling]

**Idea:** `(dA/dt)/A = 2·closing_speed/r = 2/TTC` — so we read **time-to-contact directly from the image** and we already use it in the referee. In the literature (Tau theory) this quantity is used to produce a **velocity profile**.

**Arm:** A third option for `speed_source`: `tau` — command size to anchor a target TTC rather than a fixed ceiling. DOES NOT REACH. **Acceptance:** flat/no tail regression; terminal jitter (pitch/roll speed) decreases; The collision rate is preserved. **Hypothesis:** closes as fast as the ceiling handle, but enters the terminal more smoothly → frame loss is reduced. **Risk:** TTC is noisy (field episodic) → activate only in near band. **Intelligence:** high. **Where:** offline. **Note:** MPC's reward form question (`TO_TEST.md` 1b) is a **separate thing**; This item is the speed channel of tracking, it does not interfere there.

## T9. GIMBAL INCREASE: MEASURE THE EFFECT OF ROLL ON TRACKING [measurement, cheap]

**Cause:** gimbal single axis — **roll not compensated** (known limit of single axis also in the literature; 0.5–2° typical in aggressive maneuvering). In lateral maneuvering, the roll is at the level of `arctan(a_lateral/g)`; tracking's `ex/ey` undergoes roll de-rotation but **physical framing** rotates.

**Arm:** Measurement to `tracking_test`: roll distribution at moments of loss and correlation of loss with roll; Roll peak value on `acceleration_shaping` arms. **Acceptance:** The question "how much of the lateral losses are due to roll" is answered IN NUMBER → The priority of T1/T2/T3 is updated accordingly. **Intelligence:** medium. **Where:** offline (measurement item, not correction).

## T10. SIM ABLATION OF LAW BRANCHES [supplementary]

Arms measured offline but never run in sim: `--law poscon`, `--braking ap`, `--speed-source p`, `--acceleration-shape 0`. **Acceptance:** single sim run per arm; The sim equivalent of the ablation table `ARDUPILOT_TRACKING.md` §9 is dollar. **Low priority** — for scientific completeness.

---

## CONDITION MONITORING

| article | Subject | situation | place |
|---|---|---|---|
| T1 | yaw → `input_shaping_angle` (ArduPilot's own chain) | **NEXT** | offline |
| T2 | Put `mode_follow`'s `vel_of_target` FF back from view | OPEN | offline → sim |
| T3 | derive acceleration shaping | OPEN | offline |
| T4 | frame lag compensation | OPEN | offline (verify t_capture first) |
| T5 | PrecLand EKF, maintenance in blind phase | OPEN | offline |
| T6 | visual referee sim verification | SIM IS WAITING | shimmer |
| T7 | advance clock sim verification | SIM IS WAITING | shimmer |
| T8 | tau based velocity profile | OPEN | offline |
| T9 | Measure the effect of roll residue | OPEN | offline |
| T10 | sim ablation of law branches | OPEN | shimmer |

**Rationale for order:** T1 and T2 directly address the remaining two bottlenecks (lateral delay, side frame loss) and **both are ready-made mechanisms in ArduPilot's own code** (Constraint 2) — T1 `input_shaping_angle`, T2 `mode_follow`'s term `vel_of_target`, which we deleted. T3/T4 cheap cleaning. T5 targets the rest of the frame loss (`AC_PrecLand`). When T6/T7 sim is empty, proof is to clear the debt.

**Reminder:** the primary criterion is CRASH on each item (CPA ≤ 2 m / collision rate); framing, centering, shaking are the side-effect column.
