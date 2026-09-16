# Hybrid notes — historical handle MPC→LOS

> 2026-08-11 current pure hardware candidate is pure display LOS/PN. See file `LOS_HARDWARE_NOTES.md` for short commands, 5 big bbox pass, and actual hardware sequence. This file is preserved as the historical record of the MPC→LOS hybrid used in the comparison.

Current main candidate: `guidance_allstar/hybrid_guidance.py`.

## Code map

| File | Mission |
|---|---|
| `simple_guided_follow.py` | Situated approach; establishes the slot behind the target |
| `hybrid_guidance.py` | Main actuator of the visual phase and transition MPC→LOS |
| `mpc_guidance.py` | Mid-range planer of hybrid outside 18 m |
| `terminal_los_guidance.py` | Terminal law PN/LOS in 18 m |
| `visual_base.py` | Camera measurement, MAVLink command, Redemption authority and common LPF |
| `bbox_to_redis.py` | Location→makes the video authorization decision |

When doing main development, look at `hybrid_guidance.py` and `terminal_los_guidance.py` first. If you want to change the pure MPC behavior, edit `mpc_guidance.py`; This change also affects the outer band of the hybrid.

## Manual work sequence

Run each line in a separate terminal. After the first two commands are completed, the location and video terminals remain on.

### 1. Start the environment with the selected vehicle model

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_VIDEO=1 YILDIZ_VIDEO_LABEL=hybrid_robofly ./yildizlar_guidance.sh --robofly
```

The current test vehicle is RoboFly. When comparison is needed, you can change the vehicle option to `--iris` or `--hummingbird`. If you don't want a GUI, add `--headless` to the end of the same line. Even though video is now on by default, `YILDIZ_VIDEO=1` keeps the intent visible in the command; only `YILDIZ_VIDEO=0` closes recording. This option is read **when starting the environment**; Adding it to a post-positioned or hybrid command does not start recording.

Video and model verification a few seconds after startup:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && printf 'model=' && cat run/selected_model && rg "Video kaydi:" logs/bbox.log | tail -1 && ls -lh videos/*.mp4 | tail -1
```

### 2. Engage the target and lift the fighter into the air

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 tools/launch_mission.py --drones 1 --drone-alt 60 --plan missions/target_ellipse.plan
```

### 3. Activate position guidance

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6
```

### 4. Activate visual hybrid guidance

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 hybrid_guidance.py
```

### turn off media

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && ./yildizlar_guidance.sh --stop
```

## Automatic ellipse experiment

To run the above sequence with a single command and collect log/summary:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly YILDIZ_VIDEO=1 DURATION=360 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="hybrid_guidance.py" PLAN=missions/target_ellipse.plan METHOD=hybrid tools/scenario.sh
```

`tools/scenario.sh` also forcibly turns on video while `RESTART=1`; `YILDIZ_VIDEO=1` in the command was written to make the intent visible.

If the stack is already healthy:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly DURATION=360 RESTART=0 CONTROL_WAIT_S=0 VISUAL_GUIDANCE="hybrid_guidance.py" PLAN=missions/target_ellipse.plan METHOD=hybrid tools/scenario.sh
```

This second command only retrieves video if the stack was previously initialized with `YILDIZ_VIDEO=1`. Video feature cannot be added to the running bbox process later; If necessary, stop the stack and 1. Restart with the command in step.

Use `RESTART=1` in the first automatic run of the new model. Once the model, camera subject, MAVLink ports and ArduPilot parameters are verified, it can be redone faster with `RESTART=0`.

## Two different eras

### 1. Positioned → display hybrid

Positioned and hybrid process work simultaneously. The hybrid initially sends only `visual_alive` heartbeat, not commands.

Default decision rule:

- Valid detection at least 20 of the last 25 frame,
- bbox area threshold: `%2 × %2` rectangular area of the frame,
- positional estimation range maximum `60 m`,
- working video controller heartbeat.

When the conditions are met, `bbox_to_redis.py` turns `command_authority` into `visual`. The positioned process stops sending setpoints. The hybrid is seeded with the end-position speed command in `handoff_state`; thus reducing the speed jump on the first command.

If the valid detection count remains at the return threshold and the 2 second dwell is reached, the video process dies, or the terminal reports a miss after the transition, authority reverts to the position.

### 2. MPC in hybrid → PN/LOS

This is not a Redis or flight mode change. The software status in the same `hybrid_guidance.py` process changes:

```text
visual guidance authority acquired
        |
        v
 R > 18 m: MPC + shared reachability limit
        |
        | measured range <= 18 m
        v
 R <= 18 m: terminal PN/LOS (latched for the engagement)
```

On passing, the terminal law is reseeded with the current vehicle speed and range. Even if the range goes back above 18 m, it will not return to MPC in the same engagement; This prevents MPC/LOS vibration around the threshold. After the work, the video authorization is released and the positioned layer installs the back slot again.

The mandatory input for this transition in the real vehicle is the reliable `range_m_value` measurement. In the simulation, this magnitude is generated from the IMM estimator that feeds the target's location packets; target direction/speed is not given to the terminal law, but range is again based on target position telemetry. In real hardware this should be replaced by onboard stereo/depth/radar or verified image-based range. If the range `None` remains, the transition MPC→LOS will not occur.

Experimental image-only passthrough (not default):

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 hybrid_guidance.py --transition-source visual
```

Default thresholds are `area_value=%3.4`, `|ex|<=6°`, `|ey|<=15°`, and `dwell=0.30 s`. They can be changed with `--visual-area-pct`, `--visual-ex`, `--visual-ey`, and `--visual-dwell`, respectively. In the long RoboFly/ellipse test on 2026-08-11, this branch performed well in the 15 engagements that reached LOS (`CPA median 1.50 m`, 13/15 `<5 m`), but the gate opened in only 15 of 29 handovers. The default range-based transition is therefore retained. See **LIVE T2 ASSESSMENT** in `TO_TEST.md` for the detailed table.

Important: making the transition visual does not make the terminal law completely telemetry-free. The current PN/LOS still uses gain, `t_go` uses `range_m_value` for vertical resolution and miss dropout. Real vehicle requires stereo/depth/radar or verified monocular range. The confidence value of the future YOLO detector should also be added to the Redis measurement/log; The current HSV detector does not produce confidence.

## Camera pairing

RoboFly simulation camera:

| Feature | Value |
|---|---:|
| Image | 1280 × 720, BGR8 |
| camera speed | 30 FPS |
| Horizontal viewing angle | 66.0° |
| Vertical viewing angle | about 40.1° |
| Internal parameter | `fx=fy=985.5`, `cx=640`, `cy=360` |
| ROS subject | `/drone_1/webcam/image_raw` |

In the real camera, matching the resolution alone is not enough. The same horizontal FOV/crop and calibrated internal parameters should be used; otherwise the pixel-by-pixel and bbox-space proximity thresholds change.

For the actual camera, also measure auto exposure/white balance behavior, actual FPS, end-to-end delay and timestamp source. Digital zoom, stabilization and variable crop must be off or fixed as used in calibration. Rolling-shutter time should also be tested under fast yaw/roll.

## Real gimbal pre-flight verification

If you cannot read the gimbal angle, accepting the commanded angle as real angle only works if servo delay, backlash and saturation can be neglected; In an offensive manoeuvre, this is not a safe assumption. Order of preference:

1. Real angle from servo encoder/potentiometer or gimbal telemetry,
2. Relative angle to camera carrier IMU and body IMU
3. Command+angle estimation with calibrated servo dynamics,
4. Controlled test with fixed camera and `--without-gimbal` if there is no gimbal feedback.

With steering off, run these tests in order:

1. Verification of bbox center, `ex/ey` mark, size symmetry and FOV/intrinsics in seven static images (center, four corners, top-middle, bottom-middle).
2. When the drone is stationary, step the gimbal `0, ±5, ±10, ±15°`; Measure pixel/° gain, settling time, overshoot, deadband, backlash and saturation.
3. While the target is stationary, roll/pitch the body in a safe pattern; The stabilized target error must remain constant. Single-axis tilt gimbal cannot correct roll; The code must compensate for image rotation caused by roll.
4. Measure the delay between the gimbal command, true angle, camera timestamp and bbox timestamp in the test with the propellers inserted or securely connected.
5. In Hover, horizontal guidance is off, only gimbal tracking is on: center RMS/p95, lost frame rate, saturation time and recapture time are recorded.
6. Finally, gradual visual guidance testing is performed with low speed, geofence, altitude floor and manual override.

When seven photographs arrive, a numerical acceptance table should be prepared for the first item with the same detector and real camera calibration; It is not enough to evaluate "obvious" by eye alone.

## Current defaults

| Parameter | Value |
|---|---:|
| Range of MPC→LOS | 18 m |
| PN coefficient `N` | 4 |
| STRIKE forward acceleration | 4 m/s² |
| Climbing speed limit | 2.5 m/s |
| Descent speed limit | 2.0 m/s |
| FROST range | 3 m |
| FROST `t_go` | 0.25 s |

Univariate example:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly DURATION=360 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="hybrid_guidance.py --n-pn 5 --transition-range 20 --climb-speed-max 2.0" PLAN=missions/target_ellipse.plan METHOD=h_n5_r20_vz2 tools/scenario.sh
```

Give each different combination a unique name `METHOD`. If you change more than one parameter in the same run, you cannot separate the cause of the result.

## Control and logs

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 guidance_allstar/terminal_los_test.py
cd /path/to/multicopter_to_fixed_wing_guidance && python3 -m py_compile guidance_allstar/hybrid_guidance.py guidance_allstar/terminal_los_guidance.py
```

Expected events:

- `handoff_received`: positioned→visual,
- `hybrid_los_transition`: MPC→LOS,
- `phase_strike` / `phase_don`: terminal states,
- `miss_release`: The terminal left the authority to the controller after the transition.

Main outputs:

- `videos/guidance_<date>.mp4` in manual registration,
- `videos/<METHOD>_<route>_<date>.mp4` in automatic experiment,
- `run/trials/<METHOD>_.../visual.log`
- `run/trials/<METHOD>_.../summary_value.txt`
- `guidance_allstar/logs/visual_hybrid_*.csv`
- `guidance_allstar/logs/hybrid_mpc_diagnostic_*.csv`

The video starts recording with the media. Use the `./yildizlar_guidance.sh --stop` command at the end of the experiment to close the MP4 file properly.

Success is not just the minimum range. Check together:

- `ref_range_ground_truth_m`,
- `vibe_max` or `impact_successful`,
- `ALTITUDE ABORT`,
- target freshness rate,
- Command–actual velocity angle in the MPC/LOS transition.

Current live verdict and next trial matrix: `TO_TEST.md`, **LIVE T1 JUDGMENT**. Detailed pure-MPC history: `MPC_NOTES.md`.
