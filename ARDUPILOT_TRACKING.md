# Visual guidance with ArduPilot guidance laws — research and `tracking_guidance.py`

Branch: the archived ArduPilot FOLLOW configuration. Question: **Can ArduPilot's own tracking law be used instead of MPC?** This file describes (a) what actually happens in ArduPilot, (b) what can be used with our constraints, (d) the measured result. Source: local copy of `~/ardupilot` (2025-07-13) + upstream `plane_follow.lua`.

---

## 1. ArduPilot components applicable to visual target guidance

There are four families and **two complement each other** — one for prediction, one for control.

### (A) `AC_PrecLand` — “land” family. **PRIMARY side.**

Precision Landing establishes the relative position of the target by taking the **unit LOS vector** from the camera/pointing system and multiplying it by the distance:

```cpp
// AC_PrecLand.cpp: construct_pos_meas_using_rangefinder()
if (retrieve_los_meas(target_vec_unit, frame)) {          // bearing (camera)
    ...
    _target_pos_rel_meas_NED = (target_vec_unit_ned * dist_to_target)
                                + cam_pos_ned_rel_imu;    // times range (rangefinder)
}
```

On top of that, it installs **per-axis EKF** (`_ekf_x`, `_ekf_y`), performs dead reckoning with inertial speed between frames, eliminates outliers with APR:

```cpp
_target_pos_rel_est_NE.x -= inertialNavVelocity.x * dt;   // propagate between frames
if (NIS_x < _outlier_reject_num) _ekf_x.fusePos(meas, var);
```

And `mode.cpp::precland_run()` has a **framing/alignment gate**: if you are far from the target horizontally, lower the descent rate, if the error is large, reduce the descent speed:

```cpp
const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent/acceptable_error));
cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms + land_slowdown);
```

**Our take:** first line — `tracking_guidance._target_estimate()` does exactly `unit_LOS(ex, eps) × range`. **What we did not receive (conscious, item 7):** EKF + dead reckoning and alignment gate.

`plane_precland.lua` is just a QuadPlane wrapper: it reads the target of `AC_PrecLand` and calls `vehicle:set_target_location` in the VTOL phase. It doesn't have its own control law, nor does it work on a normal fixed wing.

### (B) `ArduCopter/mode_follow.cpp` + `AP_Follow` — **CONTROL side.**

The situation in Copter-4.4 (the ~150 line cpp that the user mentioned) is exactly this — and this is the law we copied:

```cpp
desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P;
|v_xy| <= WPNAV_SPEED;   v_z ∈ [-WPNAV_SPEED_DN, WPNAV_SPEED_UP];
avoid.limit_velocity_2D(PSC_POSXY_P, WPNAV_ACCEL*0.5, ...);   // slow down on approach
v_z <= avoid.get_max_speed(PSC_POSZ_P, WPNAV_ACCEL_Z*0.5, |dz|);
yaw  = target bearing;                                      // FOLL_YAW_BEHAVE=0
```

Copter-4.5+ does the same job as `pos_control->input_pos_vel_accel_NE()`; the inner loop there is `AC_P_2D::update_all` = `sqrt_controller(error_value, PSC_POSXY_P, acceleration_value2, dt)`. `tracking_guidance.py` has both (`--law classical|poscon`).

The target position comes to `AP_Follow` from **MAVLink telemetry** (`GLOBAL_POSITION_INT` / `FOLLOW_TARGET`). So the law is ready, **this data source is excluded by the project rules.**

### (C) `plane_follow.lua` — fixed wing, **architectural template.**

It doesn't use images, it gets position+velocity+heading from `AP_Follow`. The important part for us is not the control but the **architecture**: direction and speed are SEPARATE channels.

```
GUIDED_CHANGE_HEADING (43002)   <- bearing + crosstrack PID
GUIDED_CHANGE_SPEED   (43000)   <- range PID around target airspeed
GUIDED_CHANGE_ALTITUDE(43001)   <- target altitude
```

Speed is **not** a P output that scales with distance; a separate channel. This distinction has been vital for us (item 4).

### (D)`CAMERA_TRACKING_IMAGE_STATUS` / `fake_camera_tracking.lua`

As user LLM said: this message is **not a navigation entry** in ArduPilot, just the GCS interface. It is not a guidance input.

---

## 2. Why we rewrote in Python, but not flight software

The most "native" option would be: the companion `FOLLOW_TARGET` sends the target position it established from the image to the autopilot with the message MAVLink, the copter flies in **FOLLOW mode**, the law actually runs in ArduPilot's own code.

We didn't try because:

1. **FOLLOW mode brakes as if it were "stay alongside" mode** — `limit_velocity_2D` cannot be turned off from the parameter. You approach the target of 21 m/s with a ceiling of 10.9 m/s at 25 m; multiplication impossible (§4, item 3).
2. **The speed architecture would be broken.** The whole stack is based on speed setpoint `command_authority` + GUIDED (`bbox_to_redis` ↔ `visual_base`). Changing mode means rewriting the entire handover/reacquisition chain.
3. **The comparison would be dirty.** For A/B with MPC to make sense, the frame, LPF, speed clamp, miss criteria must remain the same. Staying on the same `VisualController` interface gives this away for free.

The LLM the user asked also pointed to the same place: in the image processing and target estimation companion, there is no ArduPilot binding called `vision:get_estimated_target()`. TRUE; We have the equivalent of that function as `_target_estimate()`.

---

## 3. Rule compliance (range only from target)

| Entry | Source | Allowed by the project rules? |
|---|---|---|
| `ex`, `ey` | virtual gimbal (`tracker_bbox_stab`) | yes, image |
| `range_value` | `RangeEstimator` | yes, only permitted target telemetry quantity |
| `pos/vel/yaw/pitch/vibe` | **our** telemetry | Yes |
| target speed/direction/acceleration | — | **not used** |

`mode_follow`'s `vel_of_target` forward feed term **completely deleted**. The cost of this was measured and determined the architecture (§4 item 4).

---

## 4. Four mandatory adaptations (all measured, all one button)

**(1) Where is the target?** Method `AC_PrecLand` instead of `AP_Follow`: `target_pos_value = own_pos + range × Rz(yaw)·LOS(ex, eps)`. No extrapolation (target speed prohibited) → estimation always belongs to the "now".

**(2) `FOLL_POS_P` 0.1 → 1.0.** When there is no forward feed, the P term has to produce the entire speed of the target by itself; permanent following distance `d* = v_target / kp`:

| kp | balance distance (21.05 m/s target) | saturation to the ceiling |
|---|---|---|
| 0.10 (AP default) | 210.5 m | 350 m |
| 0.35 | 60.1 m | 100 m |
| 1.00 | 21.1 m | 35 m |

Since the engagement envelope is ≤60 m, the AP cannot **mathematically** capture the default.

**(3) Approach brake OFF.** `sqrt_controller(25 m, 1.0, 2.5)` = 10.90 m/s. Target 21.05 m/s. Closing is impossible when the brake is open; closed loop: min range 30.00 m (never closes). It switches back on with `--braking ap` (this is the correct behavior for the station keeping / safe distance scenario).

**(4) The velocity magnitude is from the cruise ceiling, not from the law.** The inevitable consequence of (2): Any pure P-law that is `|v| = kp·error` cannot hit the moving target; Range freezes on `d*`. Verified in closed loop:

```
pure mode_follow (kp=1.0):  minimum range 22.47 m  (theory 21.1)  closing speed +0.8 m/s
'ceiling_value' branch            :  minimum range  1.54 m  CONTACT      closing speed +4.3 m/s
```

Solution `plane_follow.lua`'s own architecture: **direction** from the FOLLOW law, **speed** from the cruise ceiling (`GUIDED_CHANGE_SPEED` logic). The pure arm stands with `--speed-source p`.

---

## 5. Who guards the frame? (the main loophole of this law)

**No terms.** MPC has FOV hard constraint, acceleration penalty, terminal vertical alignment; not here. Measured chain:

```
at handover: command STEP from 17 → 35 m/s
  → forward acceleration saturates (5 m/s²) → nose pitches 15.7° DOWN
  → the body-fixed camera pitches down with it
  → the target ABOVE the axis leaves through the TOP edge
  → 18 frames (≈0.9 s) BLIND
```

> **[GIMBAL BRANCH UPDATE 2026-08-05]** The chain above was measured on the the archived ArduPilot FOLLOW configuration branch and applies there. **On the `gimbal` branch, its third link ("the body-fixed camera tilts down with the body") is broken:** the camera sits on a self-stabilizing physical single-axis tilt gimbal. While the body pitch ranged over −35.4…+35.2°, the maximum camera pitch relative to the world was |0.65°|. The VERTICAL component of this issue therefore does not apply there. Yaw and roll remain, since the gimbal has only one axis. If the branches are merged, `acceleration_shaping_mps2` must be measured again. The table below used a body-fixed camera. See `GIMBAL_NOTES.md`.

The only defense is ArduPilot's own tool: **kinematic shaping** (`shape_vel_accel` / `WPNAV_JERK` counterpart, `acceleration_shaping_mps2`). 6 scenario × 4 seed, min range median (m) / framing loss:

| a [m/s²] | flat/well | flat/diameter | ellipse/well | ellipse/diameter | wand/well | wand/diameter | loss |
|---|---|---|---|---|---|---|---|
| 0.0 (closed) | 1.78 | 15.36 | 7.96 | 28.86 | 14.96 | 30.00 | %50–72 |
| 2.0 | 5.37 | 18.65 | 5.48 | 18.02 | 8.60 | 23.42 | %0–8 |
| **3.0 (default)** | **1.35** | **1.37** | **1.88** | **21.48** | **5.20** | **17.78** | %0–23 |
| 5.0 | 1.68 | 2.83 | 2.50 | 37.11 | 1.74 | 39.81 | %7–41 |

**On MPC the opposite was measured** (FOLLOW_UP.md eng-2: `forward_acceleration_ceiling` 2 m/s² → min range 1.93 → 11.17 m, “measured eliminated”). It's not a contradiction: MPC already protected the frame at its cost, shaping there only brought a closing cost. There is nothing protecting the frame here, so the frame gain gained by shaping is greater than the frame loss it loses.

ArduPilot's **real** answer to this problem is not in the guidance, but in the hardware: `mode_follow.cpp::init()` locks the gimbal to the target sysid when entering FOLLOW mode (`AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER`).

---

## 6. Offline benchmarking — SAME engine, SAME rule

`mpc_test.Simulation` (point-mass fighter + real virtual gimbal + skeleton chain), target 21.05 m/s, 1.5 s running ends at non-framing (`LOSS_FINISH_S`). Min range [m], 2 seed average:

| scenario | MPC | follow (a=3) | |
|---|---|---|---|
| straight/tail | 1.63 | **1.35** | equal |
| straight/cross | 32.9 | **1.37** | **tracking is much better** |
| ellipse / tail | 1.90 | 1.88 | equal |
| ellipse / cross | 20.7 | 21.5 | equal |
| wanderer / tail | **3.18** | 5.20 | MPC good |
| wanderer / cross | **11.5** | 17.8 | MPC good |

Cycle cost: **43 µs** (MPC solver p95 ≈ 13 000 µs) — 300× is cheap.

Interpretation: the law **matches MPC in non-maneuvering geometry**, and is even better on the straight/crossing case, where MPC declares a miss at 33 m. The gap grows as the target maneuvers, because of **loss of the target from the field of view**. The difference between %23 and %30-35 alone is insufficient to explain this. The losses occur at critical moments.

---

## 6b. SIM RESULTS (2026-08-05, four scenarios)

`tools/compare_results.py` provision — `IMPACT` = nearest pass ≤3 m, `NEAR` ≤8 m. **Warning (vehicle own note):** Gazebo does not **model contact** between two SITL vehicles; vibe doesn't jump on actual impact, so `min_m` is the benchmark (`impact_successful` vibe gate didn't fire at all in this scenario — expected).

| route | running | provision | min_m | transfer | ex_rms | vault | detection% |
|---|---|---|---|---|---|---|---|
| hanging target | tracking 121424 | **SHOT** | **0.7** | 13 | 2.53 | 0.88 | 56.4 |
| hovering, miss 15 s | tracking 124843 | **SHOT** | **0.5** | 6 | **1.33** | **0.14** | 75.5 |
| infinite (straight) | tracking 122116 | **SHOT** | **1.2** | 10 | 11.30 | **0.15** | 61.5 |
| ellipse | tracking 122843 | **SHOT** | **0.2** | 12 | 13.44 | 1.82 | 68.8 |
| wanderer | tracking 123835 | CLOSE | 5.5 | 9 | 18.88 | 3.03 | 72.2 |

MPC references with same environment/same skeleton:

| route | MPC run | provision | min_m |
|---|---|---|---|
| hanging target | mpc 031804 (15 s miss threshold) | SHOT | 0.5 |
| forever | mpc 093533 (tur-4) | SHOT | 1.3 |
| forever | mpc 063956 (tur-3) | SHOT | 0.3 |
| ellipse | mpc 021132 (BEFORE 35 m/s tour) | MISS | 9.2 |
| wanderer | mpc 190919 / 191719 | CLOSE | 6.7 / 4.9 |

**Reading:**

* **In the same class** as MPC in three scenarios; best number on the chart on the ellipse (0.2 m) — but MPC does not run lap-4 on that route, the comparison is missing.
* Wanderer is on in both: follow 5.5 m, MPC 4.9-6.7 m.
* **Speed jump** (command magnitude step in speed) at infinity **0.15** — the smoothest speed in the chart. Direct result of kinematic shaping.
* **`ex_rms` 2.53** (hanging) best centering in the table; but in infinity/ellipse/wanderer 11-19 — yaw channel lags behind in fast geometry (`|ex|` p95 24.6°). MPC's yaw comes from cost, ours is pure P.
* **All 9 engagement on suspended target was interrupted by 8 s miss timeout** (best ranges 20.8/19.2/14.3/13.4/8.8/5.6/4.6 m). The reason is clear in the diagnostic log: the handover is made to the **stationary** vehicle (seed speed 0) and the command ramps with 3 m/s² — in an engagement `cmd_speed` 0.15 → 16.4 m/s, range 27.2 → 5.6 m and time is up right there. So what cuts is not the law **running configuration**; The 8 s threshold was calibrated for tail engagements starting at ~20 m/s (the hanging record of MPC was also taken with the 15 s threshold). `--miss-time-timeout 15` was added for this.
* **Hypothesis confirmed in sim** (tracking 124843): 15 s min range with threshold 0.7 → **0.5 m**, number of revolutions 13 → 6 (engagements are now completed, no futile retakes), `ex_rms` 2.53 → **1.33** and `ey_rms` 8.80 → **3.54** — both the best values in the entire comparison table. Detection %56 → %76, rev jump 0.14. **equals** the record of MPC in the same scenario (0.5 m).

## 6c. HOW LONG DO WE GET OLD? (2026-08-05 afternoon)

User's rule: range comes from **ground detection**; The purpose of visual guidance is to eliminate the error of ground detection and see the target with one's own eyes. That's why range was a "necessarily added, unreliable" input. Question: How old does this law get to him?

### Answer: command path uses NO range (algebraic)

In default setting (`ofs=0`, `speed_source='ceiling_value'`):

```
error = r·u_los + 0
v    = kp·error                = kp·r·u_los
'ceiling_value':  v = v·(V/|v|)       = V·u_los          ← r cancels completely
horizontal limit: |v_xy| = V·cos(eps) ≤ V             ← never active
vertical clamp: v_z = clip(−V·sin(eps), −10, +5)  ← depends only on eps
```

The command is the function of `(ex, ey, yaw, aim)`. Test 6b confirms this at bit level: range 10 m / 60 m / 200 m and **with no range measurement** command `[19.9908, 1.3979, −2.1062]` — all four are the same.

### Closed loop: 8 break lever, all exactly the same

Degradation patterns of ground detection (`test 11`), 3 scenario × 3 seed:

| breaking | flat/tail | ellipse/tail | wanderer/tail | ellipse/cross (SCA) |
|---|---|---|---|---|
| clean | 1.35 | 1.87 | 5.20 | 21.43 |
| bias ×0.5 | 1.35 | 1.87 | 5.20 | 21.43 |
| bias ×2.0 | 1.35 | 1.87 | 5.20 | 21.43 |
| bias +20 m | 1.35 | 1.87 | 5.20 | 21.43 |
| noise %30 | 1.35 | 1.87 | 5.20 | 21.43 |
| frozen (initial value) | 1.35 | 1.87 | 5.20 | 21.43 |
| broken %50 | 1.35 | 1.87 | 5.20 | 21.43 |
| **NO RANGE** | 1.35 | 1.87 | 5.20 | 21.43 |

*(Trap: if separate RNG is not given to the jammer, loop jitter will drift, and what reads as "noise corrupted range" is actually a different `dt` string — dropped once, separated by `disturbance_rng`.)*

### The only remaining dependency: the MISS referee — and it's BREAKING.

In the above scenarios, downrange arms were never tried as the miss always fired with **timeout**. Tested directly with synthetic “trap then release” profile (`test 12`): **actual** range fired by the emplacement —

| range jam | ×0.5 | clean | +20 m | ×2.0 |
|---|---|---|---|---|
| firing range | 70.6 m | **18.7 m** | 40.7 m | 25.7 m |

Propagation **51.9 m**. Reason: the rule is not a pure difference rule; difference tests (`r > best_candidate + 30`) are intertwined with absolute gates (`12 / 45 / 120 m`), changing which lever fires when the bias gates are shifted.

### Recommendation and application: VISUAL REFEREE (`--miss-source area_value`)

The question asked by the referee is **proportional, not absolute**: "How many times the range is the best?" Because it is `s = sqrt(bbox area) ≈ C/r`

```
r / r_en_good  ==  s_peak / s        →  C cancels, no calibration required
```

Three rules, all three are scale-independent: (1) `s < s_peak/ratio_value` → the range is expanding, (2) the area was growing and now it is shrinking → we passed, (3) bbox remains persistently under the quality gate → the target is too far (rangeless equivalent of `miss_absolute_m`; in sim 402 was the arm that caught the fake revolution made in m).

**Measured result:**
- It fires at **exactly 31.2 m** on all four jamming arms in the same synthetic profile (the range judge was swinging between 18.7–70.6). Cost ~0.7 s delay (LPF 0.35 s + debounce 0.3 s).
- In the closed loop **6 is exactly the same result as the range judge in all six scenarios** (the biggest difference is 0.000 m) → **no cost**.
- `miss_source='area_value'` + **no range** → the system flies the same (the biggest difference is 5.7e-14 m). In other words, if the range cable is cut, the vehicle will fly the same way.

### Limit: bbox area CANNOT BE A RANGE SOURCE (measured, suggestion refuted)

12 running / 14.816 square log mining (plane group, quality gated):

| tape | visual `C/√area_value` MAE / p90 | current estimate MAE/p90 |
|---|---|---|
| 0–20 m | 2.74 m / **21.4 m** | 1.07 m / **2.21 m** |

Visual error >3 m in %45 of frames (%0 in prediction). The model is valid only between **~9–50 m**: below `min(w,h) ≤ 8 px` it collapses to 128–407 (real 698), and above `> 80 px` (≈7 m, i.e. hit sudden). The error is not white noise **episodic** (>%30 p90 duration of error episodes 0.65 s), so filtering does not recover. **That's why area is used only as RATIO, never as absolute range.** The quality gate comes from this measurement: `min(bbox_w,bbox_h) ≥ 9 px` (≈ `coverage_pct ≥ %1`).

Side finding: `range_m_value ≡ true range` on runs `TARGET_VEHICLE=drone2` (“hanging”) (exactly zero difference in 97–99% of frames) — in those runs the homing used **actual range**, not the estimate. The range numbers of the hanging runs cannot therefore be read as "predictive quality".

## 6d. REFEREE CUTS THE WINNER (verified by real sim logs)

The pattern captured in the offline trace — the miss being declared as we **close** — was tested in the event + telemetry logs of the 14 run (61 WATER, 128 engagement).

**Closing speed at the time of MISS, depending on the reason:**

| reason | n | closing med | previous 2 s med | real range med | closing>+2 m/s |
|---|---|---|---|---|---|
| **tracking** / timeout | 14 | **+3.50** | +3.22 | 17.2 m | **%79** |
| tracking / range opening | 7 | −12.54 | −12.77 | 58.2 m | %0 |
| mpc/timeout | 22 | −1.53 | −1.61 | 31.5 m | %9 |
| mpc / range opening | 14 | −5.18 | −5.89 | 52.3 m | %0 |

**Closure rate throughout engagement (quartile medians):**

| arm | Q1 | Q2 | Q3 | Q4 |
|---|---|---|---|---|
| follow/timeout | −0.10 | +1.71 | +2.36 | **+3.79** |
| mpc/timeout | +0.55 | +0.42 | −1.34 | −1.03 |

Closing in tracking is **monotonously increasing** and the meter is full in the quarter where we close the fastest. Concrete cuts: 6.4 at m, +9.6 m/s, 14.6 at m, +12.3 m/s, 13.3 left at m while closing with +7.5 m/s.

**Honest breakdown (agent added nuance):** "cut off on closing" alone is not enough — CPA may have already been passed within the engagement. The actual number of "trimmed winners" that are both closing and **never went through CPA** are 7/61 (%11); 5/23 (%22) in tracking, 2/38 (%5) in MPC. Of these, 3 would descend to zero range within 2 seconds.

**Two separate results:**
- `range increasing` logic **true in all conditions** (21 no closure > +2 m/s, median −6.9 m/s). It should not be touched.
- Defect **in fixed time counter only**. The `miss_time_source='progress'` targets exactly this arm.
- **MPC does not have this problem** (%5): %77–93 of MPC engagements have already passed through CPA on average 4.6–6.9 s before miss, median 22 missed with m. So MPC's miss comes from my guidance, not the umpire's — this matches up exactly with the root-cause analysis of MPC ("close coded as bonus").

**Warning (measured):** In the MPC/timeout arm the bbox area was growing in 57% of the frames when the shutdown median was −1.53 m/s — so area and range can conflict. So the 'advance' clock **is not reset, it rewinds** (leaky integrator) and an absolute ceiling (25 s) is placed on top.

## 6e. LAW vs PHYSICS — 2×2 (after gimbal branch, 2026-08-05 evening)

Question: Is there a significant difference between this version of the law and the previous version, and how much of it comes from the gimbal?

First, an observation: **The gimbal refactor did NOT change the guidance law.** Not a single line of the control path changed between the archived baseline and the gimbal revision and `tracking_guidance.py`; the only difference is that `environment_mount_deg()` now reads the camera axis from `$YILDIZ_TILT` (the *source* of the axis, not its value). The real changes to the law are my additions: **advance time** (default) and optional **visual referee**. The main thing that changes is **the physics of the plant**: `gimbal_camera` defaults to `False → True` in `Simulation`.

To separate the two, 2×2 (6 scenario × 3 seed, 40 s, min range median):

**A) Body fixed camera (OLD physics)**

| arm | flat/well | flat/diameter | ellipse/well | **ellipse/diameter** | wand/well | wand/diameter |
|---|---|---|---|---|---|---|
| OLD law (plain clock) | 1.35 | 1.29 | 1.87 | **21.43** | 5.20 | 17.79 |
| NEW law (progress) | 1.35 | 1.29 | 1.87 | **5.93** | 5.20 | 17.79 |
| NEW + visual referee | 1.35 | 1.29 | 1.87 | 5.93 | 5.20 | 17.79 |

**B) Gimbal (NEW physics)**

| arm | flat/well | flat/diameter | ellipse/well | **ellipse/diameter** | wand/well | **wand/diameter** |
|---|---|---|---|---|---|---|
| OLD law (plain clock) | 1.35 | 1.69 | 1.87 | **20.60** | 4.85 | 16.44 |
| NEW law (progress) | 1.35 | 1.69 | 1.87 | **2.89** | 4.85 | **13.24** |
| NEW + visual referee | 1.35 | 1.69 | 1.87 | 2.89 | 4.85 | **11.54** |

Framing loss (new law): fixed to body `0 / 7.0 / 0.9 / 18.5 / 22.9 / 23.4 %` → gimbal `0 / 0 / 0 / 10.1 / 19.3 / 2.1 %`.

### Reading

1. **Nothing changed in the four scenarios** (1.35 / 1.87 already collision; 5.20 and 1.29→1.69 in the noise band). So "something seriously changing" **only exists in two difficult scenarios** — and there it is very serious.
2. **Two changes COLLIDE, not add up** (ellipse/cross):
   - old law + old physics: **21.43 m**
   - gimbal alone: 20.60 m → **%4 gain** (almost nothing)
   - law alone: 5.93 m → **3.6×**
   - both: **2.89 m** → **7.4×** The gimbal alone has little to no contribution to the CPA; but when the law is corrected, the value of the gimbal is **revealed** (5.93 → 2.89, also 2.05×).
3. **on wanderer/crossover law alone does NOTHING** (17.79 → 17.79) but with gimbal it's 13.24, with doppelganger it's 11.54. The bottleneck there is framing: loss %23.4 → %2.1 (11×).
4. **Visual referee is not free, it is PROFITABLE:** in old physics it was neutral (5.93 = 5.93), in gimball wanderer/crossover 13.24 → 11.54. The arm that never reads the range is now **better** than the one that reads the range.

### Conclusion

I said before that "the gimbal raises the ceiling, the algorithm corrects the floor"; 2×2 confirms this with the number and adds something: **base correction without raising the ceiling does not give its full return.** If the two are done separately, the gain is %4 and 3.6×; together 7.4×.

## 6f. OWNERSHIP: Who owns the items TO_TEST and the COMMON ENGINE?

There are two separate ownership questions and should not be confused.

### (a) Items `TO_TEST.md`

File MPC was born from root-cause analysis, but not all of the items belong to MPC:

| article | who belongs to | From where |
|---|---|---|
| 1 (q_area), 1b (award form), 1c (additional term to J) | **MPC** | cost function; no J in follow |
| 3 (cost horizon), 7 (solver metric/budget) | **MPC** | horizon/solver; the FOLLOW branch has neither |
| 6 (beta pitch delay) | **MPC** | Framing variant of `beta` MPC |
| **2 (geometry requirement for speed gate)** | **PARTNER** | `bbox_to_redis.py` — gives transfer to **both laws** |
| **4 (blind terminal)** | **PARTNER** | they both go blind in the terminal |
| **5b (dynamic tilt tracking)** | **PARTNER** | gimbal common hardware; Hence the top-edge loss of pursuit |
| **8 (bearing-angle TMA)** | **PARTNER** | law independent estimator; Natural solution to tracking's lateral deficit |
| **9 (measuring tools)** | **PARTNER** | `compare_results.py`/`explain_run.py` reads both methods |
| **10 (actuator model τ)** | **PARTNER** | **Inside `mpc_test.py`** — also the engine of tracking |
| 0 (replay harness) | May be PARTNER | tracking has no solver, but closed-loop replay works for both. |

**Ingredients specific to Tracking, which are not present in TO_TEST:** PrecLand EKF (frame thought maintenance), FOV controller for yaw, sim verification of visual referee.

Also an observation: **the conclusion of article 1 is already structurally present in the follow-up.** Article 1 says "send the closing signal from the BBOX AREA, not from the range"; Closing in tracking is not a cost term, it is **absolute demand** with `speed_source='ceiling_value'`. So the follow-up is a ready-made **control group** for item 1: a working answer to the question "what happens if closure is coded as the goal" (ellipse 0.2 m, infinity 1.2 m).

### (b) `mpc_test.Simulation` — here's the real confusion

`Simulation` **Does not belong to MPC**: point-mass fighter + real virtual gimbal + skeleton's LPF/clamp/acceleration chain. The common ground of both laws is; it just sits historically in MPC's file. `tracking_test.py` imports it.

This is a concrete risk: **TO_TEST item 10 proposes changing this engine's actuator model (τ)**, and item 7c changes the budget gate in `mpc_test.py:596`. Either change would **silently** shift every reference value in this file, including the §6e 2×2 table.

**Measure taken (in this branch):** `tracking_test.py` test 0 = **engine seal** — 9 physics constant + `gimbal_camera` default fixed. If the engine changes, the test **pops** and says "refresh numbers". Noisy breaking instead of silent sliding.

**Permanent solution (recommended, not done):** Move `Simulation` and physics constants to a neutral module (`guidance_allstar/simulation.py`) and import both `mpc_test.py` and `tracking_test.py` from there. It wasn't done because another agent is currently working on `mpc_test.py`; Transport requires coordination of both parties.

## 7. Untested, worth trying (order of priority)

0. **NEXT BOTTLENECK: frame loss at the moment of rotation.** When the advance clock moves the ellipse/diagonal to 21.4 → 5.9 m, that scenario no longer ends with a miss, but with **frame loss**. Cut location in offline trace: **1.8 s after the handoff, at r≈46 m, the target exits the TOP edge** (last seen pixel y=14, y=−4 at time of loss). So it's still the same chain: acceleration tilts the nose down → the fixed camera goes down with it → the on-axis target comes out overhead. Kinematic shaping (3 m/s²) alleviated this but did not end it in cross-over speed. Candidate solutions are 1 and 3.

1. **PrecLand's EKF.** Sustaining the target with inertial speed when the frame drops. Now that bbox is getting stale, the skeleton is disappearing; moments of loss directly disrupt CPA. Rule compliant (own pace + final view).
2. **PrecLand's `land_slowdown` door.** Our equivalent of "lowering without aligning horizontally" is: "reduce the speed request when approaching the edge of the frame". A cheap one-parameter version of MPC's FOV constraint.
3. **Gimbal** (`MOUNT_FOLLOW_ON_ENTER`). The real hardware already has a pitch-servo gimbal; A/B with `set_mounting.py` in sim. Since it touches common files, it was not done in this branch — to avoid contaminating the comparison.
4. **Part feedforward from range speed.** Since `ṙ = v_target·u − v_own·u` is `v_target·u = ṙ + v_own·u` — so the LOS component of the target speed can be established from the allowed data (which is how MPC already uses `range_rate_value`). Returns the LOS component of the deleted FF term of `mode_follow`. **Requires rule interpretation**, so not done: user decision.

---

## 8. how to run

```bash
cd guidance_allstar && python3 tracking_test.py          # 55/55, sim gerekmez

# full trial (METHOD = follow-up)
DURATION=360 VISUAL_GUIDANCE="tracking_guidance.py" PLAN=missions/target_infinity.plan tools/scenario.sh

# ablasyonlar
VISUAL_GUIDANCE="tracking_guidance.py --speed-source p"     # pure mode_follow
VISUAL_GUIDANCE="tracking_guidance.py --braking ap"           # AP 'stand by'
VISUAL_GUIDANCE="tracking_guidance.py --law poscon"       # Copter >= 4.5 direction law
VISUAL_GUIDANCE="tracking_guidance.py --acceleration-shape 0"      # shaping off
```

Diagnostic log: `guidance_allstar/logs/tracking_diagnostic_*.csv` (columns deliberately overlap `mpc_diagnostic`: `state_value`, `impact_value`, `range_value`, `range_rate_value`, `best_candidate`, `cmd_*`, `vibe`, `hit_value`).
