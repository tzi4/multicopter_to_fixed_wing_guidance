# hardware/ — Real hardware layer without ROS

> Current default (2026-08-11): `single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3`. The following description of MPC includes historical architectural details; use file `../LOS_HARDWARE_NOTES.md` for a short and up-to-date flight sequence.

Two files, two separate jobs:

| File | Heat |
|---|---|
| **`camera_bridge.py`** | **Camera -> detection -> `tracker_bbox` / `tracker_bbox_stab` + actual gimbal command.** Without ROS. See section **10**. |
| `single_node_guidance.py` | Read those channels and make **engagement decision + default LOS/PN + speed command**. MPC is only `--guidance-value mpc` A/B option. |

Together they are the hardware equivalent of the `bbox_to_redis.py` + `visual_base.py` + `mpc_guidance.py` chain in the sim. Without `camera_bridge.py` there is **no one broadcasting** channel `tracker_bbox` in hardware — `single_node_guidance.py` listens silently idle.

---

## 1–9: `single_node_guidance.py`

`single_node_guidance.py`: **independent, single-process, self-determining** visual guidance node. Performs **same job** as existing `bbox_to_redis.py` + `visual_base.py` + `mpc_guidance.py` chain; The difference is in the architecture.

None of the existing files have been modified — the guidance law and safety layer are **imported** and reused.

---

## 1. Purpose

In today's chain, the decision is made by the **detection process**:

```
bbox_to_redis.py  --(Redis 'command_authority' = visual|position_based)-->  guidance
visual_base.VisualLoop   ->  WAITS for authority, then runs MPC
```

In other words, the **seeing eye** answers the question "should I engage?", not the hand holding the gun. This is his teammate's objection. The new node performs the same function **in one process** and **at its own discretion**:

```
bbox_to_redis.py --('tracker_bbox_stab' publication = MEASUREMENTS ONLY)--> single_node_guidance.py
single_node_guidance: evaluate gates -> ENGAGE -> MPC -> velocity command -> RELEASE
```

`command_authority` **not read at all**.

---

## 2. Architectural difference table

| | Available (bbox_to_redis + visual_base + mpc_guidance) | New (`single_node_guidance.py`) |
|---|---|---|
| Number of processes | 2 (detection decision maker + guidance) | **1** (detect only broadcasts measurement) |
| engagement decision | Returns bbox_to_redis, Redis reports `command_authority` | **self-guided** (`EngagementGate`) |
| decision rule | 25 valid at frame >=20 + area %2 + range <=60 m | **Same rule**, moved from class to class |
| Simple rule | `YILDIZ_TRANSITION_SIMPLE=1` + `YILDIZ_TRANSITION_FRAME` | `--simple-frame N` (default **off**) |
| guidance law | `mpc_guidance.MpcController` | **Same class, import** (no copies) |
| safety layer | In body `VisualLoop.run_value()` | Same logic moved (no importable surface) |
| Range | estimator (`RangeEstimator`), single source | estimator **or** Redis (`--range-source`) |
| If the range is interrupted | `None` -> MPC assumes `range_if_absent_m` (55 m) | **Last valid range FROZEN + warning** |
| Heartbeat | `visual_alive` (reads bbox_to_redis) | `single_node_alive` (does not conflict) |
| Status announcement | no (`command_authority` only) | `single_node_state` + **MAVLink STATUSTEXT** |
| MISS (release MPC) | `visual_release` is written, bbox_to_redis **confirms** | Node **releases itself**, does not wait for confirmation |
| log | `visual_*.csv` (75 columns) | Same 75 columns + 15 single-node columns |
| table test | no | `--dry-run` / `--mock-mavlink` |

---

## 3. What it does (step by step)

1. **Measurement**: Redis subscribes to channel `tracker_bbox_stab` (`[sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]`). If the virtual gimbal is turned off, angular error with pixel center is **derived** from the raw `tracker_bbox` channel with 7 element (backup path; not free from body oscillation, if seen in the run, there is a fault in the gimbal chain).
2. **Range** (range only — project rule): `--range-source estimator` (default) reads the target's `GLOBAL_POSITION_INT` from its MAVLink connection (udpin:14604, `source_system 252`) and passes it to IMM, and **range only** goes out. If `--range-source redis`, it is read as `handoff_state.range_m`. If the range is interrupted **the last valid value is frozen** and a warning is printed to stderr; It is subsequently distinguished from the columns `range_fresh` / `range_age_s`.
3. **Verdict**: `EngagementGate` (below).
4. **guidance**: `mpc_guidance.MpcController` (import) imports `Measurement`, returns `Command`.
5. **Command**: `SET_POSITION_TARGET_LOCAL_NED`, `vx,vy,vz` only (+ optional `yaw_rate`), hunter link udpin:14654 / `source_system 251`. Safety: command LPF (tau 0.35 s) -> speed clamp (`VISUAL_MAX_SPEED_MPS`) -> **altitude floor 15 m** -> yaw slew (120 dps^2) + yaw LPF (0.15 s).
6. **Status publication**: `single_node_state`, `single_node_alive` (TTL), STATUSTEXT.
7. **Log**: `hardware/logs/single_node_mpc_<timestamp>.csv` (+ `_event.csv` + MPC diagnostic log), a flush on line 20.

---

## 4. engagement status machine

```
WAIT  --(angajman kapisi opened)-->  ENGAGED
ENGAGED --(loss ladder | detection-loss window | MPC MISS)--> RELEASE -> WAIT
                                                          |
                                                    COOLDOWN 3 s
```

### Door (default: three doors)

| Door | Threshold | env |
|---|---|---|
| Stability | last **25 frame >= 20** valid (%80) | `YILDIZ_WINDOW_FRAME` / `YILDIZ_WINDOW_RATIO` |
| Area | bbox area larger than frame rectangle (%2 x %2) | `YILDIZ_TRANSITION_AREA_PCT` |
| Range | `range <= 60 m` (**skip if out of range/stale**) | `YILDIZ_TRANSITION_RANGE` |

All three are the **same** numbers and the same env variables as `bbox_to_redis.py`.

**Window is in SQUARES, not duration.** 25 frame 30 is 0.83 s at fps, 20 is 1.25 s at YOLO Hz. (The old 45 frames were respectively 1.5 s / 2.25 s; the code was engaged half a second late in hardware without changing.) To see this difference without guessing from the log, **both speeds are logged**:

* `detection_fps` — **broadcast** speed of bbox_to_redis (individual counter measures thread)
* `observation_fps` — discrete frame rate **seen** by the loop = `min(loop_hz, detection_fps)`

The window operates on observed frames (the guidance can only react to the frame it sees), so in a cycle of 20 Hz, 25 frames are 1.25 s — even if the camera is 30 fps. If shorter is desired, `--loop-hz 30` or `--window-frame`.

Alternatives:
* `--simple-frame 5` — three gates are skipped, "consecutive 5 valid frame" is enough (`YILDIZ_TRANSITION_SIMPLE` logic in bbox_to_redis). Default is **off**.
* `--window-s-value 1.5` — window is set in **duration** instead of frames (not the default; fps is ready for heavy gaming hardware).

### Release

| Path | Condition |
|---|---|
| lost ladder | bbox age > `--release-s` (2.5 s) uninterrupted |
| Detection loss window | current frame in window <= 3 **and** 2 s dwell (shaky detection) |
| MISS | `mpc_guidance` state machine said `Command.release_value=True` |
| Closing | SIGINT/SIGTERM — if we are engaged, leave and leave |

Loss ladder (same thresholds and rationales as visual_base):

| bbox age | status column | command |
|---|---|---|
| <= 0.7 s | `fresh_value` | MPC runs |
| 0.7 – 1.7 s | `hold_value` | last valid command is kept (except yaw) |
| >1.7 s | `coast` | **measured alignment damping** (not zero — zero is the full brake command) |
| >2.5 s | — | LEAVE |

---

## 5. Redis keys

| Key | Direction | Contents |
|---|---|---|
| `tracker_bbox_stab` | **reader** (subscriber) | `[sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]` |
| `tracker_bbox` | **reader** (subscriber) | raw bbox — coverage + backup angular error |
| `handoff_state` | **reader** | `range_m` for warm-start seed (`cmd_vel_ned`) + `--range-source redis` |
| `single_node_state` | **writer** | JSON engagement status (no TTL, staleness with `t_mono`/`t_unix`) |
| `single_node_alive` | **writer** | Heartbeat/dead-man switch with TTL (2 s) |
| `command_authority` | **writer** (announcement) | `visual` when engaged, `position_based` when released. **Not read by this node.** Closes with `--no-authority-write` |

`single_node_state` example:

```json
{"state_value": "ENGAGED", "reason_value": "three_gate(25/25 frames, area %2, range <= 60 m, fps=19.9)",
 "method_value2": "mpc", "kural": "three_gate_frame", "pid": 95009, "t_mono": 14259.14,
 "t_unix": 1786063317.7, "range_m_value": 48.92, "detection_fps": 29.6,
 "observation_fps": 19.9, "valid_ratio": 1.0, "dry": false}
```

> **WARNING — conflict:** `bbox_to_redis.py` also says `command_authority` in the decision maker role (only in mode change). If they both run at the same time, **last one to write wins**. In a single node setup, `bbox_to_redis` should remain in a detection-only role; If the positioned process (`simple_guided_follow.py`) does not run at all, issue `--no-authority-write`. (The permanent solution is to add a `--no-karar` flag to `bbox_to_redis`; existing files were not modified in this work.)

---

## 6. formation_KILLER integration

There are two channels; **authority is Redis**, STATUSTEXT is human/GCS echo.

### a) Redis `single_node_state` (machine read)

```python
import json, redis
r = redis.Redis()
d = json.loads(r.get('single_node_state') or b'{}')
if d.get('state_value') == 'ENGAGED':
    ...   # vehicle under visual guidance: DO NOT SEND a formation slot command
```

Vitality: `single_node_alive` switch with TTL (2 s). **Otherwise the process may have exited** — `single_node_state` may indicate an old `ENGAGED`, do not trust it.

### b) MAVLink STATUSTEXT (protocol texts)

| Text | Meaning |
|---|---|
| `VISUAL_GUIDANCE: command_value bende` | The node is engaged; He is driving the vehicle now |
| `VISUAL_GUIDANCE: biraktim, dinliyorum` | He left a knot; free to vehicle formation/positioned side |

`formation_KILLER.py` is already logging STATUSTEXT (`_handle_statustext` -> `swarm.log` + `events.jsonl`). Texts in constants `single_node_guidance.ST_ENGAGED` / `ST_RELEASE`; If it is changed, it must be changed on both sides. The messages go by `source_system 251`, meaning it appears on the panel as **d251** (not the vehicle's own sysid).

Disable these messages with `--no-statustext`. Redis status publication continues.

---

## 7. Operating

### Shimmer

```bash
# 1) detection + virtual gimbal (engagement-decision role disabled)
python3 bbox_to_redis.py --no-display &

# 2) single node guidance (default: three ports, estimator range)
python3 hardware/single_node_guidance.py --loop-hz 20
```

### Pi/hardware

```bash
# If YOLO prints ~20 Hz, 25 is square 1.25 s; verify detection_fps in logs
python3 hardware/single_node_guidance.py \
    --loop-hz 20 --range-source estimator --mount 30
```

### Table test (NO vehicle/SITL)

```bash
python3 hardware/single_node_guidance.py --dry-run --mock-mavlink --duration-value 15
```

The `--dry-run` command **computes and logs commands without sending them to MAVLink**. `--mock-mavlink` also does not establish the vehicle connection (dummy state + dummy closing range) and turns on `--dry-run` by itself.

### Important CLI flags

```
--loop-hz 20              guidance loop [Hz]
--range-source {estimator,redis}
--window-frame 25         stability window [frames]
--valid-ratio 0.8        minimum valid fraction in the window (-> 20/25)
--window-s-value X             OPTIONAL: specify the window in seconds
--simple-frame N            SIMPLE rule (bypasses the three gates), disabled by default
--area-pct / --transition-range    area and range gate thresholds
--release-s 2.5 / --cooldown-s 3.0
--no-statustext / --no-authority-write
--dry-run / --mock-mavlink
--mount / --aim / --no-yaw / --speed-ceiling / --horizon / --step-s / --no-miss  (MPC)
```

---

## 8. Log columns

Master CSV = `visual_base.LOG_COLUMNS` (**imported**, 75 columns — including health columns `loop_hz_mean`, `dt_excess`, `alive_ttl`, `ap_mode`, `hb_age_s`) + that 15 single-node columns:

| Column | Meaning |
|---|---|
| `single_node_state` | `WAIT` / `ENGAGED` |
| `kural` | `three_gate_frame` / `three_gate_duration` / `simple` |
| `detection_fps` | bbox broadcast rate (LOG-ONLY, separate counter thread) |
| `observation_fps` | the frame rate the loop sees (which is what the window handles) |
| `window_sample` | number of samples in window (<= 25) |
| `valid_frame` | valid number of samples in window (threshold 20) |
| `valid_ratio` | rate [0..1] |
| `gate_area` | Is the area door open (0/1) |
| `gate_range` | Is the range door open/bypassed (0/1) |
| `consecutive_frame` | simple rule counter |
| `loss_s` | seamless bbox loss[s] |
| `range_source` | `estimator` / `redis` / `mock` |
| `range_fresh` | 1 = measured, 0 = **frozen** |
| `range_age_s` | time since last valid range |
| `dry` | 1 = `--dry-run` (command did not go to MAVLink) |

Rows are constructed from the **dictionary** (not the index): If a new column is added to `visual_base`, it will not slide silently here, it will remain empty.

The `authority` column continues to read `visual`/`position_based` for compatibility with older vehicles (engaged = `visual`); the actual status is in column `single_node_state`.

---

## 9. Offline verification

Harness `py_compile` + fake Redis / fake MAVLink (see mission report; harness `/tmp/single_node_dry_test.py`). Confirmed behaviors:

* three-door rule 25 is ENGAGED in the observed frame (`--duration-value 15`),
* `--simple-frame 5` and 5 are ENGAGED in the frame,
* When detection is stopped, `fresh -> hold -> coast -> RELEASE` operates the ladder,
* After cooling down, it becomes ENGAGED again,
* command vector is generated (`cmd_vx/vy/vz`, `|v|` 12–31 m/s),
* **0** command goes to MAVLink in `--dry-run`,
* Writing `single_node_state` / `single_node_alive` (TTL 2) / `command_authority`,
* When the range source becomes stale, the last value is frozen (`range_fresh=0`).

---
---

# 10. `camera_bridge.py` — camera bridge (entry point without ROS)

## 10.1 Why is there

`bbox_to_redis.py` produces channels `tracker_bbox` / `tracker_bbox_stab` on the Sim — but that file is a **ROS Image subscriber** and there is no ROS on the Pi 5. So on the real hardware **there was no one to stream those two channels**; The guidance side (`single_node_guidance.py`, `visual_base.py`) just listens to them. `camera_bridge.py` closes that gap:

```
camera / synthetic target -> detection (hsv|yolo|mock) -> VirtualGimbal pipeline
   -> Redis 'tracker_bbox' + 'tracker_bbox_stab'
   -> (optional) TiltTracking -> MavlinkTiltCommander -> ArduPilot mount -> servo
```

**No duplicates, all import:** `bbox_to_redis.AttitudeReader` (MAVLink ATTITUDE, timestamped buffer + frame instant interpolation), `bbox_to_redis.hsv_detection` (**same** detection code as sim), `yildizlar_gimbal.VirtualGimbal` + `joint_angle`, `tools.gz_gimbal.TiltTracking`, `tools.mavlink_tilt. MavlinkTiltCommander`. The gimbal rear three are **MAVLink**, not Gazebo.

**This bridge is NOT a decision maker:** It does not touch the switch `command_authority`. In hardware, the engagement decision lies with the guidance node (section 4). This place only publishes measurements.

## 10.2 three-stage commissioning

DO NOT SKIP the sequence — each tier confirms the assumption of the next.

### Stage 1 — NO camera, NO gimbal (on table, 30 seconds)

```bash
python3 hardware/camera_bridge.py --detector-value mock --source-value file \
    --no-gimbal --mock-bbox 940,360,60,30 --log /tmp/k1.csv --duration-value 10
```

It puts the target **where you want** in the frame and runs the whole chain (stabilization + Redis + log) as if it were real. The question "where does MPC command when you put the target there" is thus answered without flying: run `python3 hardware/single_node_guidance.py --dry-run --mock-mavlink` at the same time.

Expected flags (1280x720, hfov 66 -> `fx=985.5`, `cx=640`, `cy=360`):

| `--mock-bbox` (center) | `ex` | `ey` |
|---|---|---|
| `610,345,60,30` (640,360 = full middle) | **0.000** | **0.000** |
| `910,345,60,30` (940,360 = RIGHT) | **+16.931** | 0.000 |
| `610,145,60,30` (640,160 = ABOVE) | 0.000 | **−11.472** |
| `610,545,60,30` (640,560 = BELOW) | 0.000 | **+11.472** |

**Sign contract:** right -> `ex > 0`; up -> `ey < 0` (and target world up `-ey > 0`). If it deviates from this chart, do not go any further.

### Stage 2 — camera YES, gimbal NO

```bash
python3 hardware/camera_bridge.py --source-value cv2 --device 0 --detector-value hsv \
    --no-gimbal --mount 30 --log /tmp/k2.csv --display-value
```

Hold a purple object. `--display-value` is left **off** on the headless Pi; get mp4 with `--save` instead. If `cv2.VideoCapture(0)` does not open in the Pi's libcamera stack: `--source-value picamera2`, or `rpicam-vid -t 0 --inline --width 1280 --height 720 --codec h264 -o udp://127.0.0.1:8554`
+ `--source-value file --file-value udp://127.0.0.1:8554`.

`--mount` is the mounting angle when the camera is **fixed to the body**; There should be exactly the difference `--mount` between `raw_ey` and `stab_ey` (measured: mount +30 -> difference 30.000).

### Stage 3 — camera YES, gimbal ON

```bash
python3 hardware/camera_bridge.py --source-value cv2 --device 0 --detector-value hsv \
    --mavlink /dev/ttyACM0 --tilt 0 --tilt-alt -35 --tilt-upper 55 \
    --log /tmp/k3.csv
```

**Pass the `tools/mavlink_tilt.py` sweep test first** (see `hardware/GIMBAL_TRACKING_TEST.md`): tracking must not be attempted until the chain of command is proven. Criterion of success: when you move the object vertically, the servo tracks it in ~half a second and the object sits vertically in the center of the frame; When you hide the object, it holds 3 s, then returns to `--tilt` at approximately 10 deg/s.

When `--mavlink` is issued, the **single** MAVLink connection is opened and the attitude reader and tilt controller **share** it. Reason: `udpin:` address cannot be opened a second time (port bind), the serial port gives a bad frame on the second opening. You cannot share the serial port while Mission Planner is open — split it with mavproxy: `mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14601 --daemon`, then `--mavlink udpin:127.0.0.1:14601`.

## 10.3 RESOLUTION — silent disaster

Pixel -> angle conversion is done with `fx = (width_value/2)/tan(hfov/2)`, `cx = width_value/2`. Default frame **1280x720, hfov 66** -> `fx=985.5, cx=640`.

If you feed **1920x1080** into the camera and do not update the frame, the target **right in the middle** of the frame will appear at `px=960` and the chain will mark it as `atan((960−640)/985.5) = +18.0 deg to the right of center`. No error message appears, my command goes blank. So the bridge:

* Passes `--width-value/--height-value/--hfov` values to `VirtualGimbal`** (`bbox_to_redis` did not do this, always assumed 1280x720),
* If the incoming frame is of a different size, **calculates how many degrees of deviation there will be** and warns and rescales the frame,
* If `--size-multiplier` is given, **exits** instead of scaling (exit 1).

Verified: source 1920x1080, runs `--width-value 1280` (scaling) and `--width-value 1920` (native) both gave `ex = +17.99 deg`.

**Rule:** give your camera's actual resolution and actual HFOV; Scaling is a safety net, not the solution.

## 10.4 YOLO handle — letterbox recycling

The network is fed via **letterbox** to a fixed input such as 640x640; its output is in the pixel of that canvas. If not moved back into the rendering frame, the target shifts to the upper left corner and reads `ex/ey` ~2 times smaller — a **silent** error.

`ultralytics.predict()` does the recycling itself and returns the boxes at pixels of the **frame you give them**; The requirement is to render the frame **as is** (a classic mistake is to manually resize first and then forget to scale back). The bridge gives the exact rendering frame and **seals** each box against the frame boundaries; If there is overflow, achieve `WARNING: ... invalid letterbox coordinate conversion`.

In the IMX500 (.rpk) branch, the network runs on the sensor, the boxes are moved from the frame metadata to the frame with `convert_inference_coords()` (do not multiply by hand). This arm is **not yet verified in hardware**; Should be verified by comparing with `--detector-value mock` and `--detector-value hsv` on first run on Pi. If `ultralytics` / `picamera2` are not present, they both give understandable error saying the installation command.

## 10.5 Log columns — how to read field diagnostics

With `--log X.csv`, flush once every 20 rows to limit the data lost if the process crashes.

| Column | Meaning |
|---|---|
| `t` | `time.monotonic()` — Same clock as Redis `t_capture` |
| `tilt_cmd_deg` | elevation **desired** by the law of pursuit |
| `tilt_status_deg` | **last published** value of the controller (after dead band/speed limit) |
| `raw_ex_deg`, `raw_ey_deg` | The angle that would be read if there was no de-rotation |
| `stab_ex_deg`, `stab_ey_deg` | angle from the virtual gimbal chain (seen by my guidance) |
| `bbox_w`, `bbox_h` | detection size (input of range/area gate) |
| `valid_value` | 1 = detected |
| `fps` | frame rate over the last ~60 frames |

**Diagnostic tree — divides the fault into two:**

* `raw_*` **false** (non-zero when target is in the middle of the frame) -> problem **in detection or internal parameters**: resolution/HFOV mismatch, wrong `--width-value`, YOLO scale recycle, HSV selecting wrong blob. First, return to table section 10.2 step 1.
* `raw_*` **correct** but `stab_*` **false** -> problem **in attitude / roll / time sync**: MAVLink does not come into ATTITUDE (no `--mavlink`?), roll sign is reversed, or camera pipeline delay is not measured (`--camera-latency-ms`, see `tools/calibrate_gimbal_timing.py`). The two should be **equal** with the body at rest (gimbal off and `--mount 0`).
* `tilt_cmd_deg` moves but servo does not move -> problem **in command path**: run `tools/mavlink_tilt.py` sweep test.
* `fps` lower than expected -> engagement windows count **frames**; handover is delayed (note `detection_fps` in section 4).

Additionally, the bridge prints ~1 summary line to **stderr** every second (not to stdout, so as not to interfere with the measurement stream fed to the pipeline):

```
[BRIDGE] fps= 29.9 frame_value=418 detection=%97 bbox=(910,345,60,30) ex=+16.93 ey= +0.00 deg tilt(cmd/st)=+11.5/+11.4 deg
```

## 10.6 Known limit (OPEN JOB)

`tilt_status_deg` **is not an independent angle feedback** — it is the last issued command. The hardware equivalent of `gimbal_tilt_status` in the Sim would be the message `MOUNT_ORIENTATION`; reading it requires a **second recv consumer** on the same connection, and pymavlink is not thread-safe. The ArduPilot mount driver is safe in the slow regime because it mounts the instruction at <1 s; There is a margin of delay in fast terminal maneuvering. Challenge to solve: distribution of `MOUNT_ORIENTATION` from single recv consumer on shared connection.

**MEASURED result of this — command overshoot (offline emulation, fixed target +20):** since the chain assumes *command* as the angle of the camera, the tracking law turns into an **integrator** when the servo lags behind (`cmd_new = cmd + (target − ground_truth_angle)`). It doesn't escape because of the clamp and slew limit, but the command overshoots the actual angle:

| Servo delay | peak command | Overshoot | Settling time |
|---|---|---|---|
| none (ideal) | +20.00 | %0 | 0.53 s |
| tau = 0.15 s | +24.03 | %20 | 0.57 s |
| tau = 0.30 s | +27.35 | %37 | 0.73 s |
| tau = 0.60 s (heavy gimbal) | +32.77 | %64 | 0.93 s |

Meaning in the field: **do not place** the `--tilt-alt/--tilt-upper` clamp at the mechanical stops (default −35/+55, actual stops −38/+58 — allowance left on purpose) and look at the peak value of the `tilt_cmd_deg` column on the first bench run. This source of overshoot is removed when independent `MOUNT_ORIENTATION` feedback is available (this is how the `gimbal_tilt_status` path in the sim works).

The closed loop was **verified under the ideal-servo assumption**. With the target fixed at +20.0 deg in world coordinates, the tilt command settled at **+20.007 deg in 0.5 s**, leaving **0.1 pixels** of vertical image error. When detection stopped, it held for 3 s, then returned to `--tilt` at approximately 10 deg/s.

## 10.7 Redis contract (EXACTLY with `bbox_to_redis`)

```
tracker_bbox       [x, y, w, h, coverage_pct, valid_value, t_capture]
tracker_bbox_stab  [sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]
```

Based on `t_capture` **`time.monotonic()`** (in sim it was watch ROS). Consumers use it for square **signature** and age calculation; Since CLOCK_MONOTONIC is system-wide in Linux, the two processes compare directly.
8. element (`tilt_eps`) is the camera elevation used by the chain in that frame; When the gimbal is off it is `null` and `mpc_guidance` sets `ey_ref` accordingly.
