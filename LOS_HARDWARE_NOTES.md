# LOS hardware notes — current plain flight line

Current decision: position guidance establishes the slot behind the target. After 5 consecutive fresh, sufficiently large bbox detections, visual control passes to the single `TerminalLosController` law. MPC remains available for A/B comparisons and fallback. The hardware default uses LOS/PN for the visual phase.

## Simulation — each command one line

Start the environment with RoboFly and video recording:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_VIDEO=1 YILDIZ_VIDEO_LABEL=los_robofly YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 YILDIZ_HANDOFF_COOLDOWN_S=3 ./yildizlar_guidance.sh --robofly
```

Insert the target into the ellipse and raise the hunter to 60 meters:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 tools/launch_mission.py --drones 1 --drone-alt 60 --plan missions/target_ellipse.plan
```

Run the positioned approach:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
```

Operate single LOS visual guidance; The transition decision comes from the env values given when the environment starts:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 terminal_los_guidance.py
```

Fully automatic 600 second ellipse test:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 YILDIZ_HANDOFF_COOLDOWN_S=3 DURATION=600 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="terminal_los_guidance.py" PLAN=missions/target_ellipse.plan METHOD=los_big5a3 tools/scenario.sh
```

Stop the environment cleanly and finalize the MP4 file:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && ./yildizlar_guidance.sh --stop
```

## Real hardware — observe first

The references that worked on the hardware were not changed: `competition/gimbal_bench_tracking.py`, `competition/mavlink_tilt.py` and `competition/monitor_mpc_commands.py`. The verified IMX500 1280×720/180° rotation, HSV, moment center, servo ramp and `MAV_CMD_DO_MOUNT_CONTROL` behaviors were moved to the main hardware tools.

Run gimbal and camera in bench test without command:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/gimbal_bench_tracking.py --source-value picam --dry --display-value
```

Watch the actual LOS output without driving the vehicle, with a pure observer running on the hardware:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/monitor_mpc_commands.py --law los --range-m-value 20 --n-pn 4 --strike-acceleration 4
```

Scan the servo mount channel with the vehicle stationary and the propellers secure:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 competition/mavlink_tilt.py --connection-value udp:127.0.0.1:14554 --channel-value 9
```

Launch real camera, detector, Redis and gimbal:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 hardware/camera_bridge.py --source-value picamera2 --width-value 1280 --height-value 720 --hfov 66 --detector-value yolo --yolo-model MODEL.rpk --yolo-conf 0.70 --mavlink udp:127.0.0.1:14554 --save
```

Testing the LOS line without sending a vehicle command:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3 --dry-run --duration-value 60
```

Once the geofence, altitude floor, manual override and dry log are verified, run the same line with the actual command:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3 --loop-hz 20 --range-source estimator
```

## Transition rule

`--area-pct 3` sets an area threshold equal to 3% of image width multiplied by 3% of image height. For a 1280×720 image, this is approximately 829 px², or 0.09% of the total image area. Five consecutive fresh frames take approximately 0.25 s at a 20 Hz detector rate, or 0.17 s at 30 FPS. There is no center condition; LOS begins correcting lateral error immediately after handover. Target range is not used at the gate. YOLO discards boxes below `--yolo-conf`, so the five-frame counter only receives detections that meet that confidence threshold.

Important limitation: switching is done with image only, but the current LOS/PN law still uses the value `range_m_value` for gain PN, `t_go` for vertical resolution and miss drop. In fact, if target telemetry is unreliable, the next research task is range estimation with bbox scale/optical magnification or fusion of a separate distance sensor.

If the gimbal servo angle cannot be read independently, the camera bridge counts the last command as the angle proxy value. While this may be sufficient for bench tracking, it is not evidence of performance in physical flight. For first flights it is necessary to verify gimbal status with the encoder/pot, camera IMU or measured servo delay model.

## Current settings and acceptance criteria

- Transition: 5 successive large bbox, field threshold `%3 × %3`.
- Display law: LOS/PN, `N=4`, STRIKE acceleration `4 m/s²`.
- The lateral velocity command is translated into roll/pitch by ArduPilot; yaw-rate remains on by default. For the yaw experiment, `terminal_los_guidance.py --no-yaw` or `single_node_guidance.py ... --no-yaw` in hardware can be used.
- Freeze: range `3 m` or `t_go≤0.25 s`.
- Dropout: bbox loss/detection window or post-transition opening.
- Success: real CPA, `<5 m` ratio, `impact_successful`, near-target oscillation, `ALTITUDE ABORT`, frame freshness and physical crash are evaluated together.

In the RoboFly/ellipse campaign on 2026-08-11, all 6 complete N=4 engagements achieved `<5 m`. Actual CPA median/p90 was `1.09/2.03 m`, with two physical contacts in separate runs. The N=5 comparison configuration produced no contact over 12 engagements, CPA median/p90 `1.65/3.76 m`, and `11/12 <5 m`. These results supported retaining N=4. Since the vehicle crashed after contact, subsequent lines were not included in the performance pool. Detailed decision is in the **LIVE T3 ASSESSMENT** in `TO_TEST.md`.
