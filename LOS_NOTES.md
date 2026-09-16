# LOS notes — current decision and usage document LOS/PN

Last update: 2026-08-11

This document is the current authority. `MPC_NOTES.md` preserves the date of the MPC campaign, and `LOS_HARDWARE_NOTES.md` preserves the first hardware sketch; If there is a conflict between them, this file and `LAST_TO_DO.md` will prevail.

## 1. summary judgment

Current research base:

`position-based rear-slot approach → 5 large and fresh bbox detections → one TerminalLosController`

- The default law of the visual phase is LOS/PN in `guidance_allstar/terminal_los_guidance.py`; `N=4`, forward HIT acceleration `4 m/s²`.
- The direct input to be used in the simulation is `guidance_allstar/terminal_los_guidance.py`, the wrapper in real hardware is `hardware/single_node_guidance.py --guidance-value los`.
- MPC was not deleted. It stands as A/B and offline research arm with `--guidance-value mpc`; today's terminal flight is not the default.
- The vehicle does not receive a direct roll command. LOS generates lateral velocity/acceleration request; ArduPilot converts this to roll/pitch. Yaw-rate is only the auxiliary channel that keeps the target horizontal FOV.
- The architectural direction has been finalized, but the actual flight statch is not yet ready. The propeller terminal should not be tested until the camera orientation, single-command-writer interlock, time synchronization and moving gimbal angle `LAST_TO_DO.md` P0 items are completed.

### 2026-08-11 real hardware result

- The Pi production entry is file `hardware/visual_guidance.py --guidance-value los`, with existing hardware adaptations preserved; `MPC` is only loaded if `--guidance-value mpc` is explicitly selected.
- MAVProxy `--streamrate=20` is mandatory. The default 4 Hz produced 125–237 ms stale ATTITUDE for the dynamic virtual gimbal.
- The pitch of the ArduPilot servo-mount `GIMBAL_DEVICE_ATTITUDE_STATUS` is the joint angle relative to the body in this setup; The goal of DO_MOUNT_CONTROL is world elevation. The crafting bridge uses a fresh stat as direct joint angle, but if the stat is stale it uses it as command-derived angle.
- The balloon deployment uses `RawTelemetryRange` to extract only scalar 3D range from target `GLOBAL_POSITION_INT`. That integration requires the separate deployment entry point described in [BALLOON_TEST.md](hardware/BALLOON_TEST.md). The tracked single-node script supports `--range-source estimator` for the competition configuration. In both configurations, target direction comes from the image.
- Fixed range is rejected in the living; fresh LOCAL_POSITION, GUIDED no engagement/setpoint without heartbeat and true range. Command examples and network topology are in `hardware/BALLOON_TEST.md`.

## 2. Did we start LOS from scratch?

No. The new part is that we remove terminal optimization and directly generate analytical LOS/PN acceleration. Many of the physical and security layers learned in the MPC campaign have been retained.

| protected idea | Equivalent to current LOS |
|---|---|
| Hot start from position guidance | `seed_value2()` with speed and range at rev |
| Inertial LOS-rate | PN via `q_az = d(ex) + yaw_rate` |
| Available command | Acceleration envelope around current speed, ±45° heading cone and speed ceiling |
| Prevent reverse command in terminal | `DON` latch when `R≤3 m` or `t_go≤0.25 s` |
| vertical safety | Hysteresis, vertical speed cap and vertical jerk limit |
| Not attacking head-on again after the first pass | `MISS/RELEASE` then installs the back-slot repositioned layer |
| Collaborative work safety | Fresh/hold/free/release ladder, command LPF, yaw slew and altitude floor |

Those who are not protected; is the polynomial cost MPC, 2.4 is the fixed optimization horizon of seconds, the cyclic solver, and the erroneous fixed-target propagation along the horizon. Therefore, today's structure is not to "give up MPC and bring back the old LOS", but to reduce the useful constraints of MPC to an analytical nominal law.

## 3. Why is MPC no longer the terminal default?

The decision is based on measurement, not aesthetics:

- In the base of long pure-MPC the real CPA median is about `11.20 m`; The lone 3 of the 59 engagement became `<5 m`.
- At `R<8 m` closures, the actual `t_go` median was `1.54 s` while the MPC horizon was `20×0.12=2.4 s`. The cost also optimized the non-physical part after the collision, and the speed command was reversed in the current alignment median `171.6°`.
- MPC was already using display error, LOS was using plane, LOS was using rate and range. Therefore, just saying "Let's give MPC the model LOS" does not solve the basic problem; The bad horizon, wrong actuator response and unknown target maneuver remain.
- The measured horizontal vehicle response was approximately in the `4 m/s²` plateau and `1.7 s` delay regime; the old MPC modeled this distinctly optimistically.
- APN trial feeding the target acceleration with derivative turned out to be noise: false activation of `%70–73` even in flat phase, correct sign only `%38–43`.
- In live GUI work, the frames where the solver budget was cut reached approximately `%90`. This alone is not the root cause, but it increases the hardware error surface.

MPC has not been completely abandoned. The most valuable future branch is the one where LOS continues to generate the nominal command in each cycle and a small constraint/reference governor simultaneously projects only acceleration, jerk, bank/FOV and speed limit. This is not two passages of “first MPC then LOS”.

## 4. Current algorithm

`TerminalLosController` is three phase:

1. `SETTLE`: LOS-reduces rate and horizontal image error; Forward acceleration is low.
2. `STRIKE`: LOS is stable enough and accelerates from behind if the range is closing.
3. `DON`: returns the last reachable collision command when the remaining time is shorter than the actuator response; post-CPA does not reverse.

Current main values:

| Parameter | Value |
|---|---:|
| PN coefficient `N` | 4 |
| PN closing base | 10 m/s |
| In-code horizontal acceleration ceiling | 5 m/s² |
| STRIKE forward acceleration | 4 m/s² |
| command horizon | 0.70 s |
| Command direction cone | ±45° |
| Visual speed ceiling | 35 m/s |
| Exit from STRIKE LOS-rate | 11°/s |
| FROST | `R≤3 m` or `t_go≤0.25 s` |
| Yaw dead zone | 0.7° |
| Video handover | 5 consecutive fresh bbox each ≥829 px² |

Two important distinctions:

- `35 m/s` is not a demand, it is a ceiling; but peak speeds seen in current actual flights in `IRL_Tests` are approximately `24–27.3 m/s`. The first real LOS flight should begin under this proof envelope.
- The relay is independent of telemetry range; The current terminal code still uses PN scale, `t_go`, vertical solution and `range_m_value` for miss release. Therefore, although the transition is robust, the terminal is not yet completely telemetry-free.

## 5. Handover to visual guidance

`--large-frame 5 --area-pct 3` is the currently maintained rule:

- In the 1280×720 image, the threshold is `(1280×0.03) × (720×0.03) ≈ 829 px²`.
- The same frame is not counted twice; Five distinct, fresh detections are required.
- It is not necessary for the target to be in the center. After handover, LOS begins correcting the lateral error.
- The handover gate does not require target telemetry or a ground-provided range.
- YOLO confidence filter runs on camera bridge before bbox broadcast; The counter only sees the boxes that the detector accepts. The Confidence value is not carried separately to the Redis message.
- Five frames span approximately `0.25 s` at 20 Hz, or `0.17 s` at 30 FPS.

This rule largely removed the heavy tail from the early/far pure-LOS cycles. Do not add a center-position or LOS-rate gate yet. First measure the existing gate's behavior using actual `.rpk` confidence values and coordinates.

## 6. Straight route test — 2026-08-11

Eight separate visual cycles were created in a single campaign with RoboFly, `target_straight.plan`, N=4 and 5×%3 doors.

Real CPAs:

`1.85 · 0.44 · 1.09 · 0.17 · 1.52 · 0.61 · 1.50 · 0.64 m`

| criterion | Conclusion |
|---|---:|
| engagement | 8 |
| CPA median/p90/max | **0.87 / 1.62 / 1.85** |
| `<5 m` | **8/8** |
| `<3 m` | **8/8** |
| `impact_successful` | **2/8** |
| last contact | `vibe≈117` followed by crash/LOITER and abort at altitude |

It has been confirmed that it can capture flat targets geometrically. Despite this, no physical contact occurred during the transition between `0.17 m` and CPA; So CPA alone does not mean "hit". The final physical contact crashed the vehicle; The safety outcome should also be included in the success metric.

Even with a flat target, phase change numbers were `8, 2, 7, 3, 12, 2, 14, 2` per engagement. Approximately two phase events are expected in a normal single transition. This data is real `SETTLE↔STRIKE` chatter evidence for the vibration seen by the user.

Data:

- Experiment: `run/trials/los_big5a3_n4_straight_20260811_145455/`
- LOS CSV: `guidance_allstar/logs/visual_terminal_los_20260811_145628.csv`
- Video: `videos/los_big5a3_n4_straight_20260811_145517.mp4`
- Video verification: 1280×720, approximately 27.28 FPS, 482.64 s, 126 MB.

This is good preliminary evidence as the eight engagements come from a single-stack start, not a substitute for independent running repetitions.

## 7. Ellipse and previous method comparison

| Method/gate | n | CPA median/p90 | `<5 m` | physical contact | notes |
|---|---:|---:|---:|---:|---|
| Pure MPC long base | 59 | 11.20 / — | 3/59 | not measured | terminal inversion |
| Early/non-gate pure LOS | 19 | 1.95 / 39.13 m | 15/19 | 0 | 13.74–48.74 m heavy tail |
| LOS N=4, 5×%3, ellipse | 6 | **1.09 / 2.03 m** | **6/6** | **2** | current ellipse base |
| LOS N=5, 5×%3, ellipse | 12 | 1.65 / 3.76 m | 11/12 | 0 | an 8.94 m miss |
| LOS N=4, 5×%3, straight | 8 | **0.87 / 1.62 m** | **8/8** | **2** | last contact crashed the vehicle |

N=5 did not resolve jitter and came out no better than N=4; Upgrading to `N` alone is not the answer to the return problem. The current N=4 ellipse pool is also not evidence of hard rotation: in six passes the pre-CPA target rotation was approximately in the `0.06–0.35°/s` band. The median CPA of the four passes with rotation `1.6–6.22°/s` sustained in the N=5 log was approximately `2.66 m`. Rigid `±8/±12°/s` cells have not yet been systematically measured.

## 8. Current diagnostics for vibration and rotation leakage

The order of priority is as follows:

1. **Camera–yaw time mismatch.** Terminal is calculating `q_az=d_ex+yaw_rate_now`. `d_ex` comes from delayed image and yaw-rate comes from fresh ATTITUDE. `pid_guidance.py` previously described the same error as “artificial feedback” and solved it by moving the yaw to the image capture time.
2. **Phase chatter.** No minimum dwell for STRIKE; When `q_az` moves around 11°/s, forward momentum can jump between `0/0.5 ↔ 4+ m/s²`.
3. **Incompatibility of lateral saturation and forward gas.** When N=4 and closing 10 m/s, the lateral channel hits the ceiling at approximately `7.2°/s`, when closing is 20 m/s, 5 m/s² at `3.6°/s`; forward throttle can continue up to 11°/s.
4. **No horizontal real jerk limit.** The 0.70 s reachability envelope alone does not prevent the `+5↔−5 m/s²` raw lateral request jump in successive cycles.
5. **The actuator model is optimistic.** When using terminal 5 m/s², the MPC campaign measured a plateau of approximately 4 m/s² and a response of 1.7 s.
6. **Moving gimbal angle proxy and image chaining.** Open-loop servo delay, reverse camera, bbox center jitter and attitude delay can directly produce fake LOS-rate.
7. **The classic PN is reactive.** It only sees the target spin after the LOS starts spinning; There is no reliable target-acceleration observer.

The first solution is not to add a large dead-zone. It can also kill the real small maneuver on LOS-rate and enlarge CPA. Time matching, measurement quality, actual per-step jerk and phase hysteresis must be measured first. The dead-zone should however be A/B as a small threshold derived from the straight-flight noise distribution.

The roll command should not be given directly. The chain `lambda_dot → a_lateral → velocity setpoint → desired roll/rate → actual roll/rate → gyro/motor` must first be separated in the same time base. If the opening roll vibrates while the command is correct, the problem is FC/mechanical/filter; If the command also flickers, the problem is the external LOS loop.

## 9. Real hardware stack: how many processes, how many files?

### Running processes

The entire task consists of three logical control roles:

1. **Position-based approach:** establishes the slot behind the target and definitively stops sending the setpoint to the same vehicle once the visual phase takes over.
2. **Camera/gimbal measurement:** `hardware/camera_bridge.py`.
3. **Terminal LOS:** `hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3`.

The minimum visual runtime on the Pi is two Python processes: camera bridge and single-node LOS. The positioned role may be on another computer, but it remains alive throughout the mission. To these are added the `redis-server` and a MAVLink router/MAVProxy with separate output for each consumer.

### Critical authority status

`simple_guided_follow.py` actually interrupts setpoint and mode commands when Redis `command_authority=visual`. The current `_attacker_intercept_thread` in `formation_KILLER.py` does not read `command_authority`, `single_node_state` or `single_node_alive` and continues to send `goto_shared_ned`.

Therefore today's `formation_KILLER + single_node_guidance` combination is not flight ready; Two script writers may conflict. Also, there is no `config.py` imported by `formation_KILLER.py` in the repo. The specific `config.py` used in the hardware must also be included in the distribution manifest.

### Dependencies to copy

Just copying two entry-point files is not enough. At least the following files are required, preserving the repo directory structure:

- Camera: `hardware/camera_bridge.py`, `bbox_to_redis.py`, `yildizlar_gimbal.py`, `tools/gz_gimbal.py`, `tools/mavlink_tilt.py`, `.rpk` model.
- LOS: `hardware/single_node_guidance.py` and `guidance_allstar/{guidance_config.py,visual_base.py,terminal_los_guidance.py,los_guidance.py,mavlink_utils.py,vector_math.py,numeric_differentiation.py,filterwndr.py}`.
- Located: selected actual approach input file and all its dependencies; Including external `config.py` if `formation_KILLER.py` is selected.
- Python: `numpy`, `redis`, `pymavlink`, `opencv`, `filterpy`; Picamera2/IMX500 packages for `.rpk`.
- System: `redis-server`, `python3-picamera2`, `imx500-all`, MAVLink router.

`requirements.txt` does not currently contain `filterpy`; Even if LOS is selected, this package is required in a clean Pi installation due to the `visual_base → filterwndr` import chain. This is P0 distribution error.

## 10. What do the three `competition/` files that worked on the hardware prove?

| File | real role | Flight runtime? |
|---|---|---|
| `competition/gimbal_bench_tracking.py` | Camera+servo table tracking assuming a fixed/flat vehicle | No |
| `competition/mavlink_tilt.py` | Servo mount driver and 0/+30/−30/0 sweep diagnostics | As a module yes, as a sweep no |
| `competition/monitor_mpc_commands.py` | A mere observer calculating the actual LOS or MPC output; does not send speed/arm/mode | No |

The fact that these files ran in hardware does not prove that the gimbal angle was read or that LOS flew the vehicle. `monitor_mpc_commands.py` does not manage camera/gimbal; Waits for a existing `tracker_bbox_stab` stream in Redis.

## 11. Is gimbal angle necessary?

Short answer:

- Not required to **switch** to video via bbox/confidence/area.
- Angle feedback is not required to **send open-loop commands** to the servo.
- A separate angle sensor is not required in a fixed and calibrated camera-body conversion; Fixed mount and FC roll/pitch are sufficient.
- True gimbal angle or a validated state observer is required for accurate 3D/stabilized LOS under significant vehicle motion with moving tilt gimbal.

The production bridge does not read real mechanical angle today. `camera_bridge._tilt_eps()` considers the recently released servo command camera world elevation. This proxy is incorrect if there is servo lag, backlash, load, stall or saturation. With the same geometry, the 10° tilt proxy error is approximately:

| vehicle roll | fake horizontal LOS |
|---:|---:|
| 15° | 2.7° |
| 30° | 5.8° |
| 45° | 10.2° |

`SERVO_OUTPUT_RAW` is just PWM output, not mechanical angle. There is no IMU that provides gimbal angle in the official specifications of the Raspberry Pi AI Camera/IMX500. There is a MAVLink `GIMBAL_DEVICE_ATTITUDE_STATUS` quaternion path, but it should be verified with aciolcer on the bench whether the servo mount backend is actually issuing measured angle or command status.

The lowest risk way for the first real flight is to lock the gimbal to a stable, calibrated angle and prove the markings LOS with FC ATTITUDE. Animated mode; The encoder/pot should be turned on after verified gimbal attitude or measured command→angle delay and backlash pattern.

## 12. Useful details in bench code but not in production bridge

| proven on the bench | Current state of the production bridge | Decision |
|---|---|---|
| `hflip+vflip` to 180° rotation | No transform in Picamera2 configuration | P0 if physical assembly is reversed |
| HSV OPEN+CLOSE | Only two dilations | Detector A/B |
| Contour moment center | Axis-aligned bbox midpoint | Vibration A/B |
| 0.4° image dead-zone | There is no pinball tracking input | A/B by measuring from noise |
| 50 Hz, 120°/s, 0.1° servo deadband | Manufacturing default is approximately 10 Hz, 150°/s, 0.2° | Actual angle/cadence measurement |
| 3 keep s at loss, slowly return to center | Protected in production with `TiltTracking` | protect |

The most critical silent difference is the 180° rotation. If the same physical reverse assembly is in progress, both the `ex` and `ey` signs may remain reversed.

## 13. Ideas worthy of protection from other laws

What can be moved first:

- `pid_guidance.py`: yaw mapping at image capture time; measurement quality from bbox area+age; heavier LOS-rate filter at lower quality; Resetting derivative status after loss.
- `los_guidance.py`: only up to measured delay, clamped LOS prediction; Absorbing lead state instead of noisy raw acceleration.
- `mpc_guidance.py`: measured actuator envelope, command smoothness and proximal/constraint governor idea.
- `tracking_guidance.py`: use of bbox field as relative grow/miss signal rather than absolute range; limiting speed increase without losing direction.
- `formation_KILLER.py`: single MAVLink receiver/outpatcher, source timestamp and staleness gate, speed jump rejection, analytic range-rate, post-effect multiple health check and attack dynamics restore.
- Bench: 180° rotation, morphology, moment center, servo ramp and loss policy.

Things that should not be moved for now:

- APN with raw target acceleration derivative.
- Making the value `minAreaRect rot_angle_deg` a direct return feed-forward; offline signature/correlation is weak and varies with range.
- Legacy long-horizon intercept forecast. There are poorly conditioned examples such as `t_int=1707.7 s` in the IRL log and aam about 40 km away.
- Old vertical LOS/FOV assumptions written based on fixed body camera.

## 14. Envelope and warnings from IRL_Tests

These records are not current video LOS flight evidence; The old location/intersection is evidence of the statch and the actual vehicle envelope.

- Total 138 no successful hits recorded in the old attack: 70 range-opening miss, 63 timeout, 3 target-ran-away and 2 shutdown.
- The best range median in the first campaign was approximately 32 m; `16/100 ≤5 m`.
- In the last campaign the median was approximately 49.5 m; `2/38 ≤5 m`.
- Peak velocity median/p90/max in the last 38 attack window is approximately `18.86/23.99/24.16 m/s`.
- Peak `|roll|` about `32.03/44.04/45.76°`; peak `|pitch|` `39.41/45.38/46.60°`; peak angular velocity `36.2/49.7/70.1°/s`.
- Although the desired ATTITUDE was 25 Hz, the actual actual per vehicle was approximately 5 Hz in the blog. 20 Hz is not enough to separate guidance vibration.
- Attack window VIBRATION peak median/p90/max approximately `18.5/25.3/41.6`. The current `vibe>10 && R<3 m` sim hit threshold cannot be directly carried over to real flight.

## 15. What the literature says about this stack

- PN based image-based visual servoing in real flight; It successfully uses the roll/pitch as the main capture channel and the small yaw-PD alone as the FOV capture channel. This supports the decision to “not replace the yaw roll but not completely eliminate it”: [Yan et al.](https://arxiv.org/html/2409.17497v2)
- Approximately 100 ms camera/processing delay can correspond to 2 m position difference in 20 m/s; Moving the delayed image to the command moment with IMU proved beneficial in real circular target tests: [Yang et al.](https://arxiv.org/html/2404.08296)
- The classic PN assumes a non-maneuverable target; APN is advantageous only if there is reliable target acceleration: [Johns Hopkins APL, Modern Homing Guidance](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Homing.pdf)
- Measurement noise sensitivity of derivative/acceleration estimation: [Johns Hopkins APL, Guidance Filter Fundamentals](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Guidance.pdf)
- Structure that works well, preserves the nominal law, and changes the reference only in case of constraint violation: [Reference Governor review](https://shadow.merl.com/publications/docs/TR2016-102.pdf)
- Fully perceptive-aware NMPC is possible, but the model/solver/calibration cost is higher and the last research step should be here: [Model Predictive Spherical IBVS](https://arxiv.org/abs/2212.09613)
- Standard message for actual gimbal attitude: [MAVLink GIMBAL_DEVICE_ATTITUDE_STATUS](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS)
- In-camera gimbal/attitude IMU is missing from the IMX500 product brief: [Raspberry Pi AI Camera product brief](https://datasheets.raspberrypi.com/camera/ai-camera-product-brief.pdf)

## 16. Simulation commands — sequentially, one line each

Start the environment with RoboFly, door LOS and video recording:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_VIDEO=1 YILDIZ_VIDEO_LABEL=los_robofly YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 YILDIZ_HANDOFF_COOLDOWN_S=3 ./yildizlar_guidance.sh --robofly
```

Give the target an ellipse task and remove the hunter:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 tools/launch_mission.py --drones 1 --drone-alt 60 --plan missions/target_ellipse.plan
```

Run the positioned back-slot approach:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
```

Run display LOS in separate terminal:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 terminal_los_guidance.py
```

Stop the environment properly and complete the MP4 title:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && ./yildizlar_guidance.sh --stop
```

Fully automatic ellipse campaign:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 YILDIZ_HANDOFF_COOLDOWN_S=3 DURATION=600 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="terminal_los_guidance.py" PLAN=missions/target_ellipse.plan METHOD=los_big5a3_n4 tools/scenario.sh
```

Fully automatic flat campaign:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 YILDIZ_HANDOFF_COOLDOWN_S=3 DURATION=360 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="terminal_los_guidance.py" PLAN=missions/target_straight.plan METHOD=los_big5a3_n4 tools/scenario.sh
```

Unit/closed-loop tests:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 guidance_allstar/terminal_los_test.py
```

On 2026-08-11 this test gave `15/15 OK`.

## 17. For now, only safe test commands are available in hardware.

Bench camera+servo, vehicle stationary and propellers secure:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/gimbal_bench_tracking.py --source-value picam --dry --display-value
```

Servo mount sweep, with vehicle stationary and propellers secure:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/mavlink_tilt.py --connection-value udp:127.0.0.1:14554 --channel-value 9
```

Watch the real LOS law without driving:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/monitor_mpc_commands.py --law los --range-m-value 20 --n-pn 4 --strike-acceleration 4
```

Run the entire single-node chain without writing a speed command to MAVLink:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3 --dry-run --duration-value 60
```

The actual command line `camera_bridge + single_node_guidance + formation_KILLER` was intentionally omitted. `LAST_TO_DO.md` Before P0 was completed, with today's sources that combination has two authors and the camera direction is unclear.
