# LOG DICTIONARY

This file defines every log column in the guidance stack, including its units, sign convention, and interpretation. It provides the context needed to explain what happened in a run from its logs.

There was a tangible cost in the logs telling "what numbers were" but not "what happened" (2026-08-04): "the target was coming towards me but MPC seemed to be avoiding the target", the user said in one video; To verify this, the difference between the target's heading and the bearing had to be calculated MANUALLY, because no logs stated "was this encounter a tail chase or head-on". The result (most of the close range matches are head to head) SHOULD have been in the log. This dictionary and the `ref_*` columns exist to bridge that gap.

Quickstart: describe a run **before reading**:

    python3 tools/explain_run.py run/trials/<trial>

---

## 0. GOLDEN RULE: `ref_` prefix

Each column starting with `ref_` is for **ANALYSIS ONLY**. The guidance controller must not read it.

Project rule (2026-08-03): target 3D telemetry has limited reliability. The ONLY target quantity available to guidance is **range** (`range_m_value`). Target position, velocity, acceleration, and turn rate must not enter the control path.

But ANALYSIS cannot be done without them. The solution is the distinction:

| column | who reads |
|---|---|
| `range_m_value` | read by guidance (the only permitted target quantity) and analysis |
| `ref_*` | analysis tools ONLY. Reading it in the controller violates the project rule. |

The distinction is explicit in code: guidance can call `RangeEstimator.range_value()`, while `RangeEstimator.ref_target_state()` is called only by the log writer and AFTER the command is sent (`visual_base.py`, after the "LOG" section). If you see `ref_target_state` or `ref_` on a controller, report it.

---

## 1. TIME AND ALIGNING THE THREE LOGS

Three different clocks circulate. If mixed the alignment will be wrong:

| name | What | where |
|---|---|---|
| `t_mono` | `time.monotonic()`. On Linux, CLOCK_MONOTONIC is **COMMON across processes** (time since machine startup). It does not go back and is not affected by NTP. | with image CSV `t`/`t_mono`, with position CSV `wall_time` (monotonic despite the name!), mpc_diagnostic CSV `t` |
| `t_unix` | `time.time()`, absolute epoch second. The common time reference for videos, file timestamps, and wall-clock time. | display CSV `t_unix` (runs after 2026-08-05), mpc_diagnostic CSV `t_unix` (END), bbox.log `t_unix=` (at end of TARGET line) |
| `t_capture` | CAPTURE moment of the frame, time ROS (`tracker_bbox_stab[6]`). **Do not mix it with another clock**; only the DIFFERENCE of successive measurements is significant (delay-sensitive derivatives, lambda-point phase matching). | display CSV `t_capture`, mpc_diagnostic |

**Alignment recipe**

1. imaged / positioned / mpc_diagnostic CSVs in the same monotonic clock -> placed directly next to each other. No other conversions required.
2. To switch to absolute time, `offset_value = t_unix - t` is taken from the display CSV; the same offset is applied to the other two files (same machine, same time). In older (without t_unix) runs, the offset is estimated from the mtime of the file (+-1 s).
3. `bbox.log` **NO TIMESTAMP** -- the detector prints rows consecutively without timestamps. The only way to tie it to time is with the `px_raw_cx`/`px_raw_cy` columns in the new image CSV: the same raw bbox centers are there with the timestamp. `tools/explain_run.py` uses bbox.log only as aggregate statistics.

The `Aligner` class in `tools/explain_run.py` does all of this.

**Time related trap:** the localized process continues to measure and log (it just stops sending the setpoint) AFTER delegating authority to the display. So nesting two CSVs in time gives the illusion that "authority changes hands every cycle." Correct rule: **visual-guidance CSV contains rows only while that process has control authority**; Space between lines greater than 1 s = returned to authority position. (Same designation as `explain_run.authority_intervals`, `compare_results.py`.)

---

## 2. FILES

| file | manufacturer | what does it say |
|---|---|---|
| `guidance_allstar/logs/visual_<method>_<timestamp>.csv` | `visual_base.VisualLoop` | Analogue of VISUAL guidance. Full glossary below. |
| `guidance_allstar/logs/visual_<method>_<timestamp>_event.csv` | same | DISCRETE EVENTS (turnover, loss, constraint, range threshold, nearest pass, miss). |
| `guidance_allstar/logs/mpc_diagnostic_<timestamp>.csv` | `mpc_guidance.py` | MPC's INTERNAL diagnostics (solver, TMF limits, disturbance estimation). |
| `guidance_allstar/logs/guided_follow_<timestamp>.csv` | `simple_guided_follow.py` | POSITIONAL guidance: standoff slot, IMM prediction, recovery machine. |
| `logs/gimbal_<timestamp>.csv` | `bbox_to_redis.py --gimbal-log` | GIMBAL chain: command vs actual tilt, raw vs stabilized error, attitude interpolation gap. **Generated on EVERY run since 2026-08-07** (`yildizlar_guidance.sh` defaults the flag; `YILDIZ_GIMBAL_LOG=0` turns it off). 21 columns, see 4.5. |
| `logs/bbox.log`, `run/trials/<trial>/bbox.log` | `bbox_to_redis.py` | Detector text. NO timestamp. |
| `run/trials/<trial>/{guidance,visual,aim,summary_value}.{log,txt}` | `tools/scenario.sh` | Process outputs collected on the run. |

---

## 3. `visual_<method>_<timestamp>.csv` -- COMPLETE DICTIONARY

**75 columns** (2026-08-07: 69 -> 70 with `tilt_deg`, -> 75 with system health block). Column NAMES are authoritative, rather than their order (all tools use `csv.DictReader`; NO tools reading index-based -- checked 2026-08-07). Single source: `visual_base.LOG_COLUMNS`. Empty cell = "this magnitude did NOT exist at that moment" (NOT 0 -- keep distinction).

### 3.1 Time and phase

| column | unit | meaning |
|---|---|---|
| `t` | s | `time.monotonic()`. Retained name for compatibility with legacy tools. |
| `t_mono` | s | SAME value as `t`, just to be clear. |
| `t_unix` | s | `time.time()`, absolute epoch. Key to bbox/video alignment. |
| `dt` | s | **MEASURED** cycle step (nominal unreliable). Typical 0.050. |
| `authority` | - | In this line it is always `visual` (the loop writes only when authorized). |
| `state_value` | - | Which branch the control path goes to: `fresh_value` \| `hold_value`\| `coast`. |

`state_value` values:
* `fresh_value` -- bbox exists and is fresh; The controller worked with real measurement. **Requested.**
* `hold_value` -- SHORT loss (bbox age 0.7-1.7 s): retain the last valid command. By default, yaw rate is not retained because blind turning can move the target out of the image horizontally. **[YILDIZ_HOLD_YAW=1, 2026-08-08]** With this option enabled, the last yaw command decays as `exp(-elapsed/1.0 s)` for at most 1.5 s, then becomes `None`. The logged state remains `hold_value` so the phase fractions remain comparable. Identify this behavior by rows with **`state_value=='hold_value'` and nonempty `cmd_yaw_rate_dps`**. With the option disabled this proportion is %0. In ellipse turns, yaw commands were absent in 38-42% of frames. That could move the target out of view in the direction of the turn, further reduce detection, and create more hold frames.
* `coast` -- LONG loss: the command decays toward the MEASURED velocity ("coast"). **It is not zero.** Sending zero commands a complete stop from 18 m/s. With the OLD BODY-FIXED CAMERA, this produced a feedback loop: braking → nose-up pitch → fixed camera points up → target near the edge leaves the image completely → persistent loss. This was measured on 2026-08-04 in `mpc_20260804_160604`. **[GIMBAL BRANCH 2026-08-05]** The physical tilt gimbal breaks the VERTICAL link in that loop, since body pitch no longer appears in the image. The `coast` branch remains appropriate, because braking can still separate the vehicle from the target, disturb yaw, and increase roll. Sending zero remains incorrect.

### 3.2 Image / image plane

Image size 1280x720, horizontal FOV 66 deg, fx = fy = 985.5 px, center (640, 360).

| column | unit | meaning / sign |
|---|---|---|
| `ex_deg` | deg | Horizontal angular error from VIRTUAL gimbal. **Positive = target on the RIGHT.** This is the error used by guidance. |
| `ey_deg` | deg | Vertical angular error. **Positive = target BELOW** (negative = target ABOVE). |
| `bbox_w`, `bbox_h` | px | Raw bbox dimensions. |
| `area_root` | px | `sqrt(w*h)`. It is kept for compatibility with past runs. |
| `area_px2` | px^2 | `w*h`. This is the **PRIMARY REWARD** (linear area, not square root; user decision 2026-08-04). Growing = approaching. |
| `coverage_pct` | % | Horizontal coverage = `w / 1280 * 100`. The rev threshold is around 6% (aircraft ~6.4 m). |
| `bbox_age_s` | s | Time since last STAB detection. `> 0.7` -> `state_value` is no longer `fresh_value`. |
| `t_capture` | s (ROS) | Frame capture time. Use time DIFFERENCES within the same clock. |
| `px_virtual_x`, `px_virtual_y` | px | VIRTUAL (stabilized) pixel. Body sway mathematically cleaned frame. It may extend out of frame -- this is a calculation result, not an error. |
| `px_raw_cx`, `px_raw_cy` | px | **RAW** bbox center (direct from detector). Where the subject is PHYSICALLY in the frame. `cy < 360` = target is in the upper half of the frame. |
| `framing_edge_px` | px | The distance of the raw center to the nearest frame edge. **Small = target about to leave frame.** < 50 px alarm. |
| `framing_edge_deg` | deg | Same thing with angle (`atan(px/985.5)`). |
| `raw_age_s` | s | Age of detection on RAW channel (stab channel does not come when gimbal is off, raw always comes). |
| `tilt_deg` | deg | **[GIMBAL BRANCH]** The camera's EARTH elevation in that frame (broadcasts `tracker_bbox_stab[7]`, `bbox_to_redis`). Because the physical tilt gimbal is dynamic, the camera axis is now read FROM THIS, not the static `YILDIZ_TILT`. Empty = tilt chain closed (`--no-tilt`) or that frame is stale. See `tilt_cmd_deg`/`tilt_status_deg` on your gimbal CSV for the command/executed distinction. |

**Virtual vs raw:** `ex/ey` comes from the virtual gimbal and is the input to the guidance; `px_raw_*` tells you if the target is actually in the frame. If the two separate (at the virtual center, at the raw edge), the target has moved out of the physical FOV.

> **[GIMBAL BRANCH 2026-08-05]** It used to be written here that "classic problem of fixed camera copter: the software clears the gimbal oscillation but CANNOT bring the target BACK into the frame". This sentence is no longer valid in the VERTICAL axis: the camera is on a self-stabilizing PHYSICAL single-axis tilt gimbal, so the trunk pitch oscillation does not already rotate the physical FOV (measured in flight: camera pitch relative to the world at max |0.65| when the trunk is at -35.4..+35.2). Dissociation is still possible, but the reason has changed: (a) the yaw is still not gimbaled, (b) the ROLL is reflected in the image (single axis gimbal), (c) the tilt command is incorrect or at the joint limit. see GIMBAL_NOTES.md

### 3.3 Range (single target size allowed for guidance)

| column | unit | meaning |
|---|---|---|
| `range_m_value` | m | Length LOS from prediction `RangeEstimator` (filterwndr IMM). **this is the range the guidance uses.** Empty if data is stale (>2 s). |

There is also `ref_range_ground_truth_m` (below): measured directly from telemetry. If the two are separated by more than 5 m, the estimator has drifted.

### 3.4 Command and common protections

| column | unit | meaning |
|---|---|---|
| `cmd_vx`, `cmd_vy`, `cmd_vz` | m/s | SENT speed setpoint is NED. `vz` **+ = DOWN**. |
| `cmd_speed_mps` | m/s | `norm(cmd_v)`. Ceiling `VISUAL_MAX_SPEED_MPS` (**35**, 2026-08-05). The MPC's own roof now comes from the same source (`mpc_guidance.environment_speed_ceiling`); In the past, it was NEVER attached to the clamp because it was fixed on 18. |
| `cmd_yaw_rate_dps` | deg/s | Commanded yaw rate after slew limiting and LPF. Empty means that yaw control is left to the autopilot. |
| `clamp_speed` | 0/1 | touched the speed ceiling (LPF output exceeded 18 m/s). |
| `clamp_altitude` | 0/1 | ABSOLUTE ALTITUDE FLOOR active: The descent command is suppressed below 15 m. 2026-08-04 is the final defense of the crash course. **If 1 is continuous in the terminal phase, there is an error in the vertical channel.** |
| `clamp_yaw_slew` | 0/1 | yaw slew clamp cut (120 dps^2 * dt). 1 = controller yaw flapping constantly. |

Command path sequence: controller -> LPF(tau=0.35, seeded in rev) -> speed clamp -> altitude floor -> slew (for yaw) + LPF(tau=0.15) -> MAVLink.

### 3.5 Our own situation

| column | unit | meaning |
|---|---|---|
| `pos_x`, `pos_y`, `pos_z` | m | LOCAL_POSITION_NED. **`z` is positive downward: altitude = -z.** `pos_z = -54` -> 54 m altitude. |
| `altitude_m` | m | `-pos_z` for ease of reading. |
| `vel_x`, `vel_y`, `vel_z` | m/s | NED speed. `vel_z` + = we are descending. |
| `speed_mps` | m/s | `norm(vel)`. |
| `acc_x_mps2`, `acc_y_mps2`, `acc_z_mps2` | m/s^2 | OUR acceleration: numerical derivative of velocity (backwards O(h^2)) + LPF(tau=0.20). NOT from IMU -- 20 derived from Hz telemetry, delayed ~0.2 s. Indicator of maneuver severity; 5 m/s^2 above hard. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | deg | ATTITUDE. `pitch` + = nose up. |
| `route_deg` | deg | Course over ground, 0=north, +east. Difference with yaw = crab angle. |
| `vibe_max` | m/s^2 | The biggest axis of VIBRATION. **NOT evidence of impact:** Gazebo DOES NOT MODEL CONTACT between two SITL vehicles (vibe 0.9 measured at passage of 0.92 m). Vibe's job is to distinguish the ground contact: there it jumps to 150-345. |

### 3.6 `ref_*` -- TARGET CONDITION (analysis only)

| column | unit | meaning |
|---|---|---|
| `ref_target_x/y/z` | m | MEASURED position of the target (GLOBAL_POSITION_INT -> NED). |
| `ref_target_vx/vy/vz` | m/s | MEASURED speed of the target (telemetry), NED. |
| `ref_target_speed_mps` | m/s | `norm`. ~15-20 in Wanderer plan. |
| `ref_target_route_deg` | deg | Target's direction of travel, 0=north, +east. |
| `ref_target_ax/ay/az_mps2` | m/s^2 | Acceleration estimation of IMM (state vector 10D: `[x,y,z, vx,vy,vz, ax,ay,az, omega]`). Maneuver detection. |
| `ref_target_turn_dps` | deg/s | Effective rotation speed (CT mode) of IMM. Moving away from zero = target has entered the curve. |

### 3.7 `ref_*` -- COMPARISON GEOMETRY (analysis only)

**This block is the actual information that tells "what happened".**

| column | unit | meaning / sign |
|---|---|---|
| `ref_range_ground_truth_m` | m | Distance from measured locations. Compare with `range_m_value`: estimator drift. |
| `ref_bearing_deg` | deg | Compass bearing from us to destination, 0=north, +east, [0,360). |
| `ref_elevation_deg` | deg | The target's elevation angle relative to us. **+ = target ABOVE.** Compared to camera axis; If the difference exceeds +-20.07 (vertical half-FOV), the target is physically out of frame. **[GIMBAL BRANCH 2026-08-05]** The camera axis is NOT `montaj + pitch` anymore, the world elevation of the gimbal is: `tilt` (command `YILDIZ_TILT` = atan(down/back), actual value is `tilt_status_deg` in the gimbal log). Body pitch does NOT enter the equation (the gimbal is compensating; the measured residual pitch effect is at max 0.65). The old `montaj + pitch` formula is only valid for the `--no-tilt` (frozen body-fixed) path. |
| `ref_approximation_angle_deg` | deg | **Encounter angle.** The angle between the DIRECTION of the target and the vector "from the target to us". `0` = target coming right at us; `180` = the target is moving away from us, we are behind it. |
| `ref_encounter_type` | - | `head_toward_head` (<60 deg) \| `crossing` (60-120) \| `tail` (>120) \| `stationary` (target speed <1 m/s). |
| `ref_closure_rate_mps` | m/s | The closing speed of the range. ***+ = distance gets SHORTER.** |
| `ref_tgo_s` | s | `range / closing_speed` (only when the range is closing). |
| `ref_cpa_m` | m | Closest approach distance, assuming CONSTANT velocity. |
| `ref_cpa_s` | s | Time to closest approach. If `0` is written, the transition is in the PAST (clamped, `ref_cpa_m` is synced to the current range). |

**Sign reminder (most confused):**
```
ref_approximation_angle_deg =  0  -> HEAD-ON (target approaches us)
                        = 90  -> CROSSING (passes to the side)
                        = 180 -> TAIL PURSUIT (we are behind the target)
ey_deg  > 0 -> target BELOW the image center
ref_elevation_deg > 0 -> target ABOVE us
pos_z   < 0 -> airborne (altitude = -pos_z)
cmd_vz  > 0 -> DESCENT command
```

**Trap at the moment of passing:** `ref_approximation_angle_deg` is SWITCHED from 0 to 180 at the moment of closest passage (passes 90 as it passes). The answer to the question "What was this encounter?" is not read from the moment of transition, but from the line **~2 s BEFORE** the transition. This is how `explain_run.py` does it.

### 3.8 `event_value`

Event names generated in that loop, if more than one, are separated by `|`. Most lines are blank. The detailed version is in the file `_event.csv` (see 4).

### 3.9 SYSTEM HEALTH (2026-08-07 added for real flight)

All five columns **LOG-ONLY**: none go into guidance decision, command or timing. They answer the first three questions asked after the accident: *was the cycle in real time, was the process alive, was the autopilot listening to us.*

| column | unit | meaning |
|---|---|---|
| `loop_hz_mean` | Hz | Average `1/dt` over the last ~2 s (window = 2 s / nominal step, or 40 samples at 20 Hz). Computed from the raw, unclamped step, so the 0.5 s cap on the `dt` column does not affect this value. A value well below `loop_hz` indicates a slow loop. |
| `dt_excess` | 0/1 | If `raw_dt > 1.5 x nominal step_value` is 1. Individual 1s are normal (GC, disk); See **RATE**. When 10 SEQUENTIAL 1 is seen, the warning `*** LOOP SLOW ***` is printed to stderr (5 every s, no spam). |
| `alive_ttl` | s | REMAINING TTL of Redis `visual_alive` dead-man key. In healthy running it is always `2` (= `ALIVE_TTL_S`). If `1` appears, the loop hangs longer than ~1 s; `-1` key not present at all / refresh missed -- in this case the decision maker does NOT delegate the authority to the display or takes it back (error 2026-08-05: authority was delegated but the controller was not running at all). Sample once every 10 loop iterations to limit round-trip overhead; repeat the last value between samples. |
| `ap_mode` | - | Mode name from autopilot's HEARTBEAT (`GUIDED`, `LOITER`, `RTL`, `LAND`, `STABILIZE`...). **In every line where a NON-`GUIDED` value is seen, all guidance columns are incorrect**: the command is being sent, but it is not passed to the vehicle. Blank = no heartbeat from autopilot. |
| `hb_age_s` | s | Time since the last autopilot HEARTBEAT, indicating MAVLink link health. The autopilot sends at 1 Hz, so typical age is 0-1 s. When age grows, the row's `pos_*`, `vel_*`, and attitude fields are stale cached values. Empty means no heartbeat has been received. |

**Why only these five columns:** The `authority` column does not carry information because there is `visual` in each row (the loop writes only when authorized). The answer to the real "am I working" question is the `alive_ttl` + `hb_age_s` duo.

---

## 4. `visual_<method>_<timestamp>_event.csv` -- EVENT LOG

Columns: `t` (monotonic), `t_unix`, `event_value`, `range_m_value`, `detail`. It is edge triggered: if a constraint 100 remains open, the event `_enabled` is written once, and the event `_disabled` is written once.

| event | what does it mean |
|---|---|
| `handoff_received` | Authority has passed to video. `detail`: seed rate + `handoff_state` with/without + `kural=` (revolution triggering gate: `simple(5 ardisik frame_value)` or `previous(38/45 frame_value, area_value %2, range_value kapisi 60 m)`; source Redis `transition_reason`). **If `handoff_state=NONE` the transition may be jumpy** (the LPF was seeded with our own measured speed, not the last command of the locale). |
| `authority_to_position_returned` | The decision maker turned to `position_based`; command was interrupted. |
| `detection_fresh_to_hold` | bbox has become stale, the last command is kept. |
| `detection_hold_to_coast` | 1.7 exceeded s, switched to coast. |
| `detection_coast_to_fresh` | The target is back. |
| `clamp_speed_enabled/disabled` | 18 m/s ceiling. |
| `clamp_altitude_enabled/disabled` | 15 m altitude floor stopped descending. |
| `clamp_yaw_slew_enabled/disabled` | yaw acceleration clamp. |
| `range_100m` ... `range_3m` | FIRST crossing of the threshold. `detail`: current encounter type + shutdown. |
| `en_near_transition` | The closing speed changed sign (closing -> opening) and the range < 200 m. `detail`: type + approach angle (value at **transition time**; see trap 3.7). |
| `miss_value` | Range doubled (or +15 m) after closest pass. |
| `ap_mode_changed` | **AUTOPILOT flight MODE CHANGED** (2026-08-07). `detail`: `<previous> -> <current>`. The first seen mode is also written (`- -> GUIDED`), that is, it is fixed which mode the vehicle is in at the time of rotation. **Line `GUIDED -> LAND/RTL/STABILIZE` is the beginning of the accident timeline**: from that moment on, no setpoints sent by the guidance were passed to the vehicle. The same information appears line by line in the column `ap_mode` in the main CSV. |
| `impact_successful` | **PHYSICAL CONTACT** (2026-08-05, eng-4). Detection from controller'S OWN vibration: `vibe > 15` AND measured range `< 3 m`. `detail`: vibe value, range, phase and beat mix. BEFORE per engagement (latch). **Count the number of hits in the run summary by counting these lines** -- since each successful hit ends the run (the vehicle rolls over and crashes), the CPA estimate was misleading and the sample was small. |

---

## 4.5 `gimbal_<timestamp>.csv` -- GIMBAL CHAIN

Prints `bbox_to_redis.py`, **one line per frame** (~30 Hz, valid detections only). `t_frame` is DIRECTLY juxtaposed with `time.monotonic()`, i.e. other CSVs. Before 2026-08-07 this file was NEVER generated (flagged); The default is now on.

| column | unit | meaning |
|---|---|---|
| `t_frame` | s | The processing of the frame is monotonic. Alignment key. |
| `bbox_cx`, `bbox_cy`, `bbox_w`, `bbox_h` | px | RAW bbox center and size. |
| `roll_deg`, `pitch_deg` | deg | Body attitude INTERPOSED to the frame moment. |
| `range_m_value` | m | Range estimated from bbox width (for current absorption only; guidance range gets from telemetry). |
| `raw_ex_deg`, `raw_ey_deg` | deg | Angular error that would be seen if there was NO software de-rotation. |
| `stab_ex_deg`, `stab_ey_deg` | deg | error to steering (same as `ex_deg`/`ey_deg` in video CSV). |
| `aim_deg`, `aim_active_deg` | deg | Aim trim status (not used in tilt mode). |
| `ros_stamp` | s (ROS) | Same watch as `t_capture`. |
| `interp_gap_ms` | ms | Sample range used in attitude interpolation. Growth = attitude flow diluted, de-rotation unreliable. |
| `latency_ms` | ms | Camera pipeline latency assumption (`--camera-latency-ms`). |
| `tilt_cmd_deg` | deg | Tilt target SENT to Gimbale. |
| `tilt_status_deg` | deg | The actual tilt READ from the gimbal. |
| `tilt_age_ms` | ms | Age of pin status reading. If it is growing, gimbal telemetry has been cut off. |
| `joint_deg` | deg | Live joint angle entering the de-rotation chain. |

**First place to look:** `tilt_cmd_deg` vs `tilt_status_deg`. If it is dissociating, the gimbal cannot follow the command or is at the joint limit; If both are correct and `ey_deg` still escapes, the fault is in the standoff geometry (see 8).

---

## 5. `mpc_diagnostic_<timestamp>.csv` -- MPC INTERNAL DIAGNOSIS

`t` is on the SAME monotonic clock as CSV with image `dt`; can be placed side by side, line by line. Important columns:

| column | meaning |
|---|---|
| `t_unix` | `time.time()`, absolute epoch (FAST, by index-preservation). |
| `bbox_age`, `ex`, `ey` | Same as their counterparts in the illustrated CSV. |
| `eps` | Standoff vertical error residual. |
| `beta`, `beta_limit` | Value and limit of constraint FOV; `beta` constraint ACTIVE if it rests on the limit. |
| `depth_value` | Estimated depth (from bbox). |
| `fov_free` | 1 = FOV constraint relaxed, 0 = applied. Whether it is binding depends on the value relative to its limit. See `TO_TEST.md`. |
| `empty_counter` | There has been no detection for how many cycles in a row? |
| `internal_range`, `r_measurement` | Internal model range vs measured range; divergence = pattern shift. |
| `area_value`, `area_rate` | bbox area and growth rate (reward signal). |
| `d_ex`, `d_ey` | Distortion estimate (residual of target movement seen in the IMAGE). Target speed is NOT derived from telemetry; This is the visual residue. |
| `u1`, `u2`, `u3`, `yaw_dps` | Internal command of MPC (BEFORE the LPF of the skeleton). `cmd_*` in imaged CSV is the post-LPF+clamp version of this. |
| `vz_alt_cbf`, `vz_upper_cbf`, `yaw_alt_cbf`, `yaw_upper_cbf` | Instantaneous upper/lower bounds produced by CBFs. If `u` adheres to the boundary, it is a MFF binding. |
| `pitch_lpf` | Drained body pitch. **[GIMBAL BRANCH 2026-08-05]** The camera axis used to be calculated from this (`axis_value = montaj + pitch`); With the physical tilt gimbal, this link has been BREAKED -- the camera axis is now `tilt` (see `tilt_status_deg`), while the pitch is just a diagnostic column that describes the vehicle's attitude. |
| `duration_ms`, `iter` | Solver time and iteration. **Budget: p95 <= 12.7 ms, ceiling 13 ms** (20 Hz cycle). If `duration_ms = 0` and `iter = 0`, the solver NEVER RUNNED IN THAT LOOP: either MISS (braked glide) or HIT blind glide. |
| `budget_cut` | **0/1, AT THE END** (2026-08-07). 1 = FISTA exited WITHOUT reaching stopping criterion: hit either iteration ceiling (`iteration_ceiling`) or duration budget (`duration_budget_ms`); that is, a **non-optimal command was issued** in that cycle. `iter` and `duration_ms` alone don't say this -- both the ceiling-touching solution and the converging solution can print the same numbers. **This is the column to watch on the Raspberry Pi 5:** When CPU gets stuck the solver breaks QUIETLY, its only visible signature being the rate of this flag. In healthy running mostly 0 (except for the first few solutions on cold start). 0 on lines where the solver never runs (HIT blind glide). |
| `cost` | QP objective value. |
| `state_value` | Phase: `CLOSURE` \| `TERMINAL` (<=45 m) \| `IMPACT` (<=22 m) \| `MISS`. TERMINAL is just a label; **CHANGES the HIT and MISS control law**. |
| `best_range`, `range_rate_value` | The smallest achieved in this engagement is `r` [m]; `d(internal_range)/dt` filtered [m/s] (**negative = we are closing**). Terminal transition is when `range_rate_value` changes sign. |
| `impact_value` | STRIKE mixing coefficient [0..1] (2026-08-05). Linear with range while `state_value=IMPACT`: 0.00 at 22 m, 0.50 at 15 m, 1.00 at 8 m and below. What it does: Opens FOV bands to physical edge (14/17.5 -> 19/19), suppresses brake/acceleration band throttling, 3x opens `q_ex`/`q_ey`, field reward 2x enlarges, 0.35x dims `q_acceleration`. **Phase LATCHed, not blend**: if missed and out of range, `impact_value` turns into 0 but `state_value` remains HIT. |
| `acceleration_multiply` | The multiplier (2026-08-05) applied to `q_acceleration` (acceleration = LYING angle penalty) that cycle. HIT multiplier; default 1.0 (does not touch the STRIKE acceleration penalty — was 0.35 in type-1, removed). **Note:** the rev acceleration ramp was tested and REVERSED in the lap-2 sim (halved the shutdown), so this column is now ~always 1.0. |
| `hit_value` | **PHYSICAL CONTACT latch** (0/1), 2026-08-05 eng-4. When `vibe > 15` AND **measured** range `< 3 m` is seen, it switches to 1 and remains throughout the engagement. **In the run summary, innings NUMBER = number of passes of this column 0→1** — No need to guess CPA. Thresholds were measured from type-3: true contact vibe 17.4-25.5, non-contact flyby (tur-2, 0.85 m) 3.3 only. At the same time, line `impact_successful` is written to `_event.csv` and line `EVENT:` is written to `visual.log`. |
| `vibe` | The biggest axis of our OWN vibration (VIBRATION message). **Our telemetry, not the target** — the "range only from target" rule is not violated. GROUND contact if `> 150` and altitude ~0 (see §8). |
| `alignment_ref` | STROKE terminal vertical alignment bias [deg/s] (2026-08-05, eng-3). It becomes **positive** when `eps` (target apparent rise) exceeds the COMFORT band (10°) = "climb even, melt standoff" command; the camera becomes flat, the target does not come off the top edge. `0` = in deadzone (eps≤10°), off HIT, or below target axis (one-sided — descent is not forced). It closes the geometric root of the top-edge loss (standoff `down` → eps=down/r exceeds FOV) with pure homing. **Watch this column in Sim**: If 0 remains in the terminal, it means the mechanism is not triggered (the fighter is already aligned). |
| `tau_eff`, `u_saturation` | **SATURATED ACTUATOR DIAGNOSIS (2026-08-08, AT THE END).** `tau_eff` = HORIZONTAL effective time constant [s] of the FIRST step = `max(tau_lin, |e_horizontal|/a_max)`. **EMPTY = lever CLOSED** (old constant `speed_latency_tau_s`=1.00 s path). If it is stuck to `tau_lin` (1.7), we are in the linear region; As it grows, the plan is SATURATED. Measured (G/Gb runs): spin p50 3.1-3.5 / p90 9.1-11.4 s, flat p50 2.9 / p90 8.7-11.4 s -- with flight measurement (tau_active 5.5-6.8 s) are of the same order. `u_saturation` = Did `|u_horizontal|` touch 99% of the speed cap (0/1). **WHEN the lever is OPEN, this rate DECREASES** (measured: return %40.8 -> %14.9-27.6, straight %36.4 -> %5.9-11.0) -- a direct measure of the reduction in unachievable instruction jumps. Movement: `mpc_guidance.environment_actuator()`. |
| `stagnation` | **PROGRESS CLOCK DIAGNOSIS (2026-08-08, AT LATEST).** **stagnation clock** [s] that fires the WSC timeout. **EMPTY = handle CLOSED** -- then the decision is made from the old WALL clock (`t - authority_t0`) and the clock is not calculated at all (not a single microsecond is spent on the closed handle). With the handle open: if we are moving forward, the clock will rewind with `1 - progress_gain` = **-0.5 s/s**, if we are stationary, it will advance with +1 s/s; Exceeding `miss_time_timeout_s` (8), SCA (string `reason_value` becomes `"durgunluk (... ; angajman ...)"`). PROGRESS MEASURE = **best-so-far recovery speed** > `progress_closure_threshold_mps` (1.0 m/s); best-so-far is inherently immune to gauge oscillation because it is MONOTONOUS (instantaneous and windowed RAW range versions are tricked with sine -- see `environment_progress_clock`). Absolute ceiling `progress_ceiling_s` (22 s) interrupts the infinite loop; reason is `"progress tavani"`. **The "range opening" and "absolute range" gates were NOT touched.** |
| `v_perpendicular`, `a_perpendicular`, `apn_a` | **APN DIAGNOSIS (2026-08-08, AT THE END).** `v_perpendicular` = predicted target vertical speed [m/s] (`d_ex * r / KDEG`) -- **also written WHILE lever CLOSED**, because it is also needed in the base run to measure what APN will gain. `a_perpendicular` = filtered derivative of `v_perpendicular` [m/s^2], **RAW estimate** (deadband and confidence factor NOT APPLIED; `apn_tau_s`=0.6 in LPF, `+-6` in clamp, `r<15 m` FREEZING with d_ex). `apn_a` = value **actually used** in the law: deadband (`apn_dead_band_mps2`=0.5, subtractive) + disturbance confidence ramp + multiplier `YILDIZ_APN` applied. **If the lever is closed, `apn_a` is always 0.** Reading rule: If `a_perpendicular` is large but `apn_a` is 0, the contribution is cut by the DEAD BAND, not the noise; If both are ~0, the target is actually flying straight. Make the rotation/direct phase separation with `ref_target_turn_dps` and read this triplet as p50/p90. Movement: `mpc_guidance.environment_apn()`. |
| `band_upper` | Current TOP framing band [deg]. Nominal 17.5; Opens to 19.0 on BEAT; It narrows when FORWARD ACCELERATION is planned. **[GIMBAL BRANCH 2026-08-05]** The OLD justification for this narrowing was "nose down -> fuselage fixed camera looks down -> target top edge"; with the physical tilt gimbal this chain is BREAKED (body pitch does not rotate the frame). Narrowing is still a harmless margin of safety, but it is no longer against pitch, but against roll/yaw drift and tilt tracking lag in acceleration; Must be measured AGAIN on the gimbal branch. Base `fov_upper_floor_band_deg` = 6.0. `beta` approaches this band on the **NEGATIVE** side: `beta < -band_upper` means loss from the upper edge. |

---

## 6. `guided_follow_<timestamp>.csv` -- POSITIONAL guidance

`wall_time` is **monotonic** (name misleading; `time.monotonic()`).

| column | meaning |
|---|---|
| `range_m` / `true_range_m` | Predicted/actual target distance. |
| `closing_velocity`, `t_go_s` | Closing speed and remaining time. |
| `pursuer_x/y/z`, `pursuer_vx/vy/vz` | Our own case (NED). |
| `meas_x/y/z` | MEASURED position of the target. |
| `est_x/y/z` | IMM's nap location. Divergence with `meas` = filter delay/shift. |
| `slot_x/y/z`, `slot_vx/vy/vz` | Standoff slot (point held behind the target) and speed. |
| `aff_*`, `aff_mag` | Feedforward acceleration. |
| `recovery_state` | `CHASE` / status of the recovery machine. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | Attitude. |
| `cpa_range_m` | The closest crossing predicted. |
| `kill_mode`, `failsafe_active`, `hit_count` | Mod and failsafe flags. |
| `loop_dt_meas_s`, `simtime_ratio` | **MEASURED** cycle step and sim/real time ratio. `simtime_ratio` is a prediction of success: moving away from 1.0 (specifically <0.8) means choking the loop, and giant circles/flicker is its smell. |
| `yaw_frozen`, `z_limited`, `aim_limited`, `alt_floored`, `turn_clamped`, `carrot_limited` | What protection is in effect. |

The target's speed is NOT in this log. `explain_run.py` derives `meas_x/y/z` when necessary by differentiating ~0.5 with a central difference of s.

---

## 7. `bbox.log` -- DETECTOR

NO timestamp. Two line type:

```
TARGET center=(669,232) bbox=(665,229,9,7) cov=0.70%
[SUMMARY] frame=1485 fps=30.0 detection_ratio=%59.5 mode=position_based | gimbal: virtual=(646,79) error=(+0.33,-15.94) deg attitude=(-0.3,-9.1) aim=-0.22
```

* `center` is RAW pixel center (frame 1280x720, center 640/360). **`y < 360` = target is in the TOP race of the frame.**
* `bbox=(x, y, w, h)` -- `x,y` is the UPPER LEFT corner.
* `cov` horizontal coverage (%); rev threshold ~%6.
* In `[SUMMARY]`, `error=(ex, ey)` is the angular error (value) of the virtual gimbal, `attitude=(roll, pitch)`, `aim` is the vertical angle trim.

Use the `px_raw_cx/cy` columns of the new display CSV to time them; the same centers remain there with the timestamp.

---

## 8. "IF YOU SEE THIS VALUE, IT MEANS THIS"

| observation | comment |
|---|---|
| `dt` median greater than 0.05 / `1/dt` 20 less than Hz | The loop is drowning. Any controller assuming constant dt produces giant circles and flickering (2026-08-04 is the root cause). Look with `simtime_ratio`. Now there are direct columns: `loop_hz_mean` and `dt_excess` (see 3.9). |
| `dt_excess` ratio > %10 or `loop_hz_mean` below nominal | Real time is wasted. Verify with lines `*** LOOP SLOW ***` in **stderr** of `visual.log`; CPU reduce load (video recording, GUI). |
| `alive_ttl` = falls into -1 or 1 | Dead-man key cannot be refreshed: either the Redis misses or the loop gets stuck at >1 s. In this case, the decision maker does NOT grant / revoke authority to the video player -- the "command interrupted" lines that follow are the RESULT, not the CAUSE. |
| `ap_mode` NOT `GUIDED` | **Look at this first.** The vehicle wasn't listening to our setpoints; All guidance columns are misleading. The line `ap_mode_changed` in `_event.csv` gives the moment of switching (mode change with failsafe / RC / GCS intervention). |
| `hb_age_s` 2-3 goes over s | The MAVLink connection is lost. The `pos_*`, `vel_*`, and `roll/pitch/yaw` entries in those rows are FROZEN cached values. Do not interpret them as evidence that the vehicle was stationary. |
| High rate of `budget_cut` (mpc_diagnostic) | Solver fails to converge: Pi CPU is not sufficient or the problem is poorly conditioned. Commands are not optimal. Read along with `duration_ms` and `loop_hz_mean`. |
| `state_value` long time `coast` | The target is out of frame. The command is absorbed into the measured alignment; The vehicle is just sad. The decision maker must return to the position when his dwell is full. |
| `bbox_age_s` is constantly in the band 0.5-0.7 | We are at the limit of detection rate; the detector hardly finds the frame (small bbox / far range). |
| `framing_edge_px` < 50 and falling | The target is about to leave the frame. The subsequent `detection_fresh_to_hold` is the result. |
| `px_virtual_y` is out of frame but `px_raw_cy` is in | Normal: virtual frame is a calculation, it may overflow. |
| `ey_deg` continuous large negative (target above) | The camera points below the target. **[GIMBAL BRANCH 2026-08-05]** The culprit is NOT `pitch_deg` anymore (the gimbal compensates for it): TILT error. First look at `tilt_cmd_deg` vs `tilt_status_deg` -- if it's diverging, the gimbal can't track/at the joint limit; If both are true, the standoff geometry is incorrect (`YILDIZ_TILT` != atan(down/back), see `scripts/standoff_geom.sh`). |
| `clamp_altitude` continuous in terminal phase 1 | The controller commands continuous descent; Without the base it would hit the ground. There is an error in the vertical channel. |
| `clamp_yaw_slew` highly 1 | Controller flaps yaw (measured +-16 dps at 4 Hz with hard FOV constraint on at MPC). |
| `ref_closure_rate_mps` changed sign | It was the closest pass. The next one is `miss_value`. |
| `ref_encounter_type = head_toward_head` and the range is small | The approach is too fast, t_go is short, the response window of the controller is narrow. The answer to the question "Why does MPC seem to be running away?" is generally here. |
| `range_m_value` and `ref_range_ground_truth_m` 5 separated by more than m | The estimator has shifted; My guidance is working with the wrong range. |
| `vibe_max` > 50 **and** `altitude_m` ~0 | GROUND CONTACT (NOT impact). |
| `vibe_max` low but range < 1 m | Possibly REAL impact: Gazebo does not model contact between two SITL vehicles, vibe does not bounce. |
| `area_px2` plateaued, range constant | There's follow-up but no closure -- we're left hanging in the standoff. |
| `cmd_speed_mps` never exceeds m/s 33 (ceiling 35) | 35 is the main question of m/s type. Either the geometry (binding steep LOS -> vertical ceiling, see `cmd_vz`), `q_acceleration` (leaning penalty) or the hard constraint FOV trims the closure. Read along with `vz_alt_cbf`/`vz_upper_cbf` and `u1`. |
| `state_value=IMPACT` but `impact_value` always < 0.3 | Moved below 22 m but failed to penetrate 15 m: terminal phase started, stroke not completed. Typically followed by ICA. |
| `duration_ms=0` rows cluster in STRIKE | Blind glide works (bbox stale). If it is short (< 1 s), it is DESIGN; If it is long, the detector completely loses the target in the terminal phase. |
| `pitch_deg` oscillates quickly (> 10 deg/s in successive lines) | **[GIMBAL BRANCH 2026-08-05: this line was written for the OLD BODY-FIXED CAMERA.]** At that time, pitch speed = FRAME speed: 1 deg/s ~ 17 px/s, 13 deg/s ~ 950 px/s -> visual flicker. With a physical tilt gimbal this multiplication is INVALID (camera world pitch measured at max 0.65); For the vertical component of frame shake, now look at the variant `tilt_status_deg`. Pitch still describes the acceleration of the vehicle (pitch ~ -atan(a/g), 5.84 per m/s^2). |
| `ey_deg` too negative (target at top edge) | The target comes out from the TOP edge. If `beta < -20.07`, the target is PHYSICALLY outside FOV -- taping (HIT) cannot save. **[GIMBAL BRANCH 2026-08-05]** The old remedy was to "turn down the acceleration/pitch"; now the correct remedy is TILT: if the axis `tilt_status_deg` is not pointing up enough, the standoff `down`/`back` duo or the tilt command chain is faulty. The following measurement is from the OLD body-fixed, mount 0 era: 2026-08-05 top loss on sim %18.5 vs bottom %6.0, of which 10.9% was non-physical FOV -- Must be measured AGAIN on the gimbal branch. |

---

## 9. TOOLS

| vehicle | what does |
|---|---|
| `tools/explain_run.py <trial>` | **Run this first.** Aligns logs, outputs timeline and encounter type distribution in human language. |
| `tools/compare_results.py` | It puts runs LOS/PID/MPC side by side with the same reward description. |
| `tools/trial_summary.py <trial>` | Summary of positioned phase + camera, frame geometry analysis. |

## 10. WHEN ADDING COLUMN

1. Add **END** to list `visual_base.LOG_COLUMNS`.
2. Add to list `row_value = [...]` in `run_value()` in the same order. If it does not match, the loop returns "LOG WARNING: columns may be SHIPPED" on the first line.
3. Add unit, sign and comment to this file.
4. If it is derived from the target, its name MUST start with **`ref_`** and the control path MUST NOT read it.
5. Measure additional cost: 20 In Hz cycle, budget per row is of the order of ~100 exponent (measured: 69 column + geometry = 110 exponent/row, 0.2% of cycle).
6. If the column comes from an EXTERNAL source (Redis, MAVLink) in a separate round trip each cycle, **SAMPLE**: read once every 10 loop iterations and repeat the last value between samples (pattern `VisualLoop._alive_ttl`). The column remains full in every row, the loop budget remains intact.
7. Where new columns are read should be UNDER the control path -- in the "LOG" block after the command is sent. Thus, it can be read from the ORDER of the code that the colon does not leak into the command.

## 11. DISC WRITE GUARANTEE (2026-08-07)

On crash, the open file buffer is LOST. All processes now flush periodically; don't mess this up:

| file | flush |
|---|---|
| `visual_*.csv` | 20 per line (~1 Hz @ 20 Hz cycle) |
| `visual_*_event.csv` | In EVERY event (events are rare) |
| `mpc_diagnostic_*.csv` | 20 one per line |
| `gimbal_*.csv` | 20 per line (~1.5 Hz @ 30 fps) |
| attitude CSV (`--attitude-log`) | 20 one per line |

The missing window is thus at most ~1 s (in the past: the file was only emptied on PROPER close, i.e. on SIGKILL/crash the last ~2 s were not written at all -- exactly the part to be analyzed).
