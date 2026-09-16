# multicopter_to_fixed_wing_guidance: Visual Guidance and Autonomous Interception Research Platform

This repository provides an end-to-end research platform for detecting a moving aerial target in a multivehicle simulation, approaching it from behind, transferring control, and intercepting it with camera-based terminal guidance. The system integrates ArduPilot SITL, Gazebo, ROS, Redis, MAVLink, a physical gimbal model, Raspberry Pi 5, and the Raspberry Pi AI Camera.

The project is **open source under the MIT license**. The distribution includes source code, license and attribution records, a pinned validation environment, contribution and security documentation, and automated tests. Raw flight telemetry, personal field data, and private development history are excluded from the distribution.

> **Current status:** The strongest simulation configuration uses position-based approach followed by a single-stage **Terminal LOS/PN law with `N=4`**, engaged when the target's bounding box is sufficiently large. All six independent trials achieved a true closest approach below 5 m, and two produced a physical-contact signature. The hardware software chain has been prepared on Raspberry Pi 5. The staged validation and safety checks below remain prerequisites for flight.

## System behavior

The simulation can run up to five pursuing multicopters and one fixed-wing target. Each pursuer first uses target telemetry to occupy a slot behind the target. Control transfers to visual terminal guidance when the camera sees a sufficiently large, stable target. The terminal controller uses image error, time to go, and line-of-sight dynamics to produce velocity commands.

```text
Target mission / telemetry
          │
          ▼
Position-based rear approach
          │  5 consecutive large, fresh bounding boxes
          ▼
Terminal LOS/PN (N=4)
          │
          ▼
MAVLink velocity command → ArduPilot → vehicle

Camera → detector → Redis bbox ─────────┘
             │
             └── gimbal angle command
```

At the default transition threshold, the bounding-box area must exceed the product of `3%` of image width and `3%` of image height for **5 consecutive fresh frames**. In a 1280×720 frame, that area is approximately 829 px², or 0.09% of the image. This visual transition is independent of range. The terminal law still uses `range_m_value` for PN gain, time-to-go estimation, vertical control, and missed-interception decisions. Hardware may obtain this input from telemetry or an estimator. Removing the telemetry dependency is a principal research objective.

## Results for the selected competition guidance law

RoboFly trials on the elliptical route on 11 August 2026 produced the following results for Terminal LOS/PN with large-box handoff:

| Configuration | Trials | True CPA median / p90 | `< 5 m` | Contact signature |
|---|---:|---:|---:|---:|
| **LOS/PN `N=4`** | 6 | **1.09 / 2.03 m** | **6/6** | **2/6** |
| LOS/PN `N=5` | 12 | 1.65 / 3.76 m | 11/12 | 0/12 |
| Safe hybrid | 3 | 3.42 / 3.88 m | 3/3 | 0/3 |
| Long-horizon pure MPC | 59 | 11.20 m median | 3/59 | — |

The six individual true closest-point-of-approach distances for `N=4` were:

```text
0.42 m, 0.79 m, 2.60 m, 1.45 m, 1.39 m, 0.63 m
```

Two independent trials triggered the `impact_successful` event, and the simulated vehicle fell after contact. Measurements after contact were excluded. Here, contact is an outcome measured within Gazebo's physics model. It does not establish safe, repeatable contact with a real aircraft.

In physical gimbal simulation validation, body pitch varied from approximately `−35.4°` to `+35.2°`. The camera's absolute world-frame pitch stayed below `0.65°`, with a p95 of `0.43°`. This supports the separation of image centering from body motion.

## Terminal-controller comparison

The MPC formulation includes field-of-view and actuator constraints directly in the optimization. It produces useful trajectories at medium range. Within the final 8 m, however, its fixed `2.4 s` horizon exceeded the actual time to go, whose median was approximately `1.54 s`. Planning beyond the encounter caused the commanded velocity to point almost opposite to the actual velocity:

- For pure MPC within the final 8 m, the median velocity angle was approximately **171.6°**, and `91.4%` of samples exceeded `90°`.
- Hybrid PN/LOS removed the reverse command: median/p90 **5.35° / 6.72°**, with **0/103** samples above `90°`.
- Maintaining position-based approach until a large bounding box was visible, then applying terminal LOS/PN with `N=4`, reduced the tail of the closest-approach distribution and produced the first physical-contact signatures in these trials.

MPC remains available for A/B comparison. The main research direction is staged hardware validation of the selected LOS/PN law, range estimation without target telemetry, and safe, repeatable terminal behavior.

## Implemented capabilities

- Concurrent ArduPilot SITL launch, port conventions, and mission loading for five multicopters and one fixed-wing target.
- A pursuer speed ceiling of 35 m/s, increased from the evaluated 18 m/s configuration to close on a target moving at approximately 20–21 m/s.
- Purple-target detection to distinguish the target from Gazebo sky colors, with HSV and IMX500/YOLO detection paths.
- A timestamped Redis interface for bounding boxes, image dimensions, confidence, gimbal state, and range.
- Position-based rear-slot approach, visual handoff, dead-man control, release, and cooldown logic.
- MPC with a FISTA solver, field-of-view and actuator constraints, blind PN continuation, and miss, CPA, and contact logs.
- Comparisons among Terminal LOS/PN, conventional LOS, PID baselines, and hybrid guidance.
- Iris, Hummingbird, and RoboFly models, including a Hummingbird/RoboFly bridge plugin.
- Physical single-axis tilt-gimbal simulation, body-motion compensation, and gain correction.
- Raspberry Pi 5 and Raspberry Pi AI Camera IMX500 support with Picamera2, 180° image rotation, HSV/moment centering, servo ramps, and `DO_MOUNT_CONTROL`.
- Hardware plan/tilt compatibility checks, pre-arm health checks, minimum-altitude HOLD recovery, BRAKE at shutdown, and temporary-parameter restoration.
- Repeated experiments from one command, summaries, A/B comparisons, video, CSV, DataFlash, and simulation-time health checks.

Additional evaluated approaches include a raw APN derivative, saturated vertical actuation, an early vertical ramp, progress-clock-only control, and an intersection approach. These are excluded from the default configuration because the evaluated variants increased noise, altitude aborts, or the tail of the closest-approach distribution.

## Repository structure

| Path | Contents |
|---|---|
| `competition/` | Selected competition and hardware code package |
| `guidance_allstar/` | MPC, hybrid, LOS/PN, PID, tracking, and test implementations |
| `hardware/` | Raspberry Pi 5, AI Camera, Redis, gimbal, and single-node hardware code |
| `tools/` | Mission launch, experiment automation, summaries, and comparisons |
| `missions/` | Elliptical, straight, S-shaped, and wanderer target routes |
| `models/`, `models_hummingbird/`, `models_robofly/` | Gazebo vehicle models |
| `plugins/hummingbird_bridge/` | Gazebo–ArduPilot bridge plugin |
| `gimbal_setup/` | Gimbal models and installation files |
| `params/` | ArduPilot parameter profiles |
| `worlds/` | Gazebo worlds |
| `run/`, `logs/`, `videos/` | Runtime output, logs, and recordings |

## Simulation environment setup

The validated development environment uses an Ubuntu 20.04-class system, **ROS Noetic**, and **Gazebo Classic**. Other distributions may work but have not been validated as supported installations.

Required components:

- ArduPilot with compiled `arducopter` and `arduplane` SITL binaries
- ROS Noetic at `/opt/ros/noetic/setup.bash`
- Gazebo Classic with `gzserver` and optional `gzclient`
- `ardupilot_gazebo` with compiled `libArduPilotPlugin.so`
- Redis, MAVProxy, Python 3, CMake, and Protobuf
- Optional QGroundControl for a graphical interface

Install Python dependencies:

```bash
python3 -m pip install -r requirements-lock.txt
```

Set local installation paths:

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
export ARDUPILOT_GAZEBO_DIR=/path/to/ardupilot_gazebo
export QGC_BIN=/path/to/QGroundControl.AppImage  # optional
```

The launcher checks for `gzserver`, `redis-cli`, `mavproxy.py`, `setsid`, `flock`, and `ss`. Both `cmake` and `protoc` must also be available if the Hummingbird/RoboFly bridge needs rebuilding.

## Quick simulation trial

From the repository root, run the selected competition candidate with RoboFly on the elliptical route:

```bash
YILDIZ_DRONE_MODEL=robofly \
YILDIZ_TRANSITION_LARGE_FRAME=5 \
YILDIZ_TRANSITION_AREA_PCT=3 \
YILDIZ_HANDOFF_COOLDOWN_S=3 \
DURATION=600 CONTROL_WAIT_S=20 \
VISUAL_GUIDANCE="terminal_los_guidance.py" \
PLAN=missions/target_ellipse.plan \
METHOD=los_big5a3 \
tools/scenario.sh
```

This scenario launches the simulation headlessly, enables video, loads the target mission, takes off the vehicles, runs position-based approach and visual handoff, and generates a result summary. Stop the processes with:

```bash
./yildizlar_guidance.sh --stop
```

Select a vehicle model separately:

```bash
./yildizlar_guidance.sh --iris
./yildizlar_guidance.sh --hummingbird
./yildizlar_guidance.sh --robofly
```

`--headless` disables the graphical interface. `--without-gimbal` selects a model without a gimbal for comparison. RoboFly in headless mode is recommended for measured experiments.

## Step-by-step simulation

Use four terminals to observe the flow or debug an individual component.

Terminal 1, simulation:

```bash
YILDIZ_VIDEO=1 YILDIZ_VIDEO_LABEL=los_robofly \
YILDIZ_TRANSITION_LARGE_FRAME=5 YILDIZ_TRANSITION_AREA_PCT=3 \
YILDIZ_HANDOFF_COOLDOWN_S=3 \
./yildizlar_guidance.sh --robofly
```

Terminal 2, target mission and takeoff:

```bash
python3 tools/launch_mission.py \
  --drones 1 \
  --drone-alt 60 \
  --plan missions/target_ellipse.plan
```

Terminal 3, rear position-based approach:

```bash
cd guidance_allstar
python3 simple_guided_follow.py \
  --no-kill-mode \
  --yaw-lock \
  --back 25 \
  --down 4
```

Terminal 4, terminal visual guidance:

```bash
cd guidance_allstar
python3 terminal_los_guidance.py
```

Example target routes:

- `missions/target_ellipse.plan`
- `missions/target_straight.plan`
- `missions/target_wanderer.plan`
- `missions/target_s.plan`

## A/B comparisons

Select a retained guidance method through `VISUAL_GUIDANCE` in the single-command scenario:

| Value | Method |
|---|---|
| `terminal_los_guidance.py` | Selected terminal LOS/PN candidate |
| `mpc_guidance.py` | Pure MPC comparison |
| `hybrid_guidance.py` | MPC with terminal PN/LOS hybrid |
| `los_guidance.py` | Conventional LOS baseline |
| `pid_guidance.py` | PID baseline |

Example:

```bash
VISUAL_GUIDANCE="hybrid_guidance.py" \
PLAN=missions/target_ellipse.plan \
METHOD=hybrid_ellipse \
DURATION=600 \
tools/scenario.sh
```

Use at least 6–8 independent engagements and a clean process start for each experimental cell. Vary `N={4,5,6}`, the visual handoff threshold, and the route through a controlled factorial design.

## Outputs and experiment validity

Automated experiments produce:

```text
run/trials/<method_route_time>/
├── guidance.log
├── visual.log
├── bbox.log
├── summary_value.txt
└── aim.txt

videos/                         image recordings
guidance_allstar/logs/          guidance CSV logs
run/sitl0/logs/                 pursuer DataFlash
run/sitl5/logs/                 target DataFlash
```

Regenerate a summary and compare methods:

```bash
python3 tools/trial_summary.py run/trials/<method_route_time>
python3 tools/compare_results.py --method-value mpc los pid --csv report.csv
```

Before accepting a trial:

- Verify that `simtime_ratio` is close to 1.
- Reject a trial if the log reports `SIMULASYON GERIDE`.
- Repeat final measurements headlessly, since the GUI can increase computation-budget overruns, especially for MPC.
- Assess true CPA, contact/vibration signatures, and `ALTITUDE ABORT` events alongside command telemetry.
- Keep plan, tilt, and pre-arm safety gates enabled during measurement.

## Offline validation tests

Run the algorithm checks from the repository root without starting Gazebo:

```bash
python3 hardware/test_balloon_range.py

(
  cd guidance_allstar
  python3 terminal_los_test.py
  python3 mpc_test.py
  python3 los_test.py
  python3 pid_test.py
)

python3 -m compileall -q \
  bbox_to_redis.py hardware guidance_allstar tools competition

bash -n yildizlar_guidance.sh tools/scenario.sh
```

Previously recorded validation results:

| Test group | Result |
|---|---:|
| Terminal LOS/PN | 15/15 |
| MPC | 88/88 |
| LOS | 66/66 |
| PID | 51/51 |
| Balloon/range helpers | 4/4 |

## Raspberry Pi 5 and hardware use

Hardware development modules are in `hardware/`, and the selected competition package is in `competition/`. Raspberry Pi 5 operation does not require ROS. Install the core dependencies:

```bash
sudo apt update
sudo apt install -y \
  redis-server \
  python3-opencv \
  python3-numpy \
  python3-picamera2 \
  imx500-all

python3 -m pip install -r requirements.txt
sudo systemctl enable --now redis-server
```

The camera configuration assumes a Raspberry Pi AI Camera/IMX500, 1280×720 resolution, and approximately 66° horizontal field of view. Hardware tests on Raspberry Pi 5 informed the 180° image rotation for the physical installation, HSV/moment centering, YOLO/IMX500 path, and servo ramp.

The hardware system has two principal processes:

1. `camera_bridge.py` operates the camera and detector, publishes fresh bounding boxes to Redis, and commands the gimbal.
2. `single_node_guidance.py` decides visual handoff, runs Terminal LOS/PN, and produces MAVLink commands.

### Staged commissioning

Camera/gimbal bench test with propellers removed:

```bash
python3 competition/gimbal_bench_tracking.py \
  --source-value picam \
  --dry \
  --display-value
```

Observe calculated LOS commands:

```bash
python3 competition/monitor_mpc_commands.py \
  --law los \
  --range-m-value 20 \
  --n-pn 4 \
  --strike-acceleration 4
```

Check servo direction and limits with propellers removed:

```bash
python3 competition/mavlink_tilt.py \
  --connection-value udp:127.0.0.1:14554 \
  --channel-value 9
```

Camera bridge example:

```bash
python3 competition/camera_bridge.py \
  --source-value picamera2 \
  --width-value 1280 \
  --height-value 720 \
  --hfov 66 \
  --detector-value yolo \
  --yolo-model MODEL.rpk \
  --yolo-conf 0.70 \
  --mavlink udp:127.0.0.1:14554 \
  --save
```

Replace `MODEL.rpk` with an actual model prepared for IMX500. Adapt the MAVLink endpoint to the flight-controller connection.

First perform a dry run that sends no commands:

```bash
python3 competition/single_node_guidance.py \
  --guidance-value los \
  --large-frame 5 \
  --area-pct 3 \
  --dry-run \
  --duration-value 60
```

Live-operation candidate, after completing all safety checks:

```bash
python3 competition/single_node_guidance.py \
  --guidance-value los \
  --large-frame 5 \
  --area-pct 3 \
  --loop-hz 20 \
  --range-source estimator
```

These commands illustrate the runtime interface. Validate connection addresses, servo channels, parameter authority, range sources, and flight limits for the actual vehicle before live flight.

## Hardware flight safety boundary

The software chain is prepared for transfer to the competition vehicle. This is not an airworthiness certification. Initial live validation follows this sequence:

1. Read-only observation and logging.
2. Gimbal direction, mechanical limits, and latency measurements with propellers removed.
3. Dry command generation with a stationary or suspended vehicle.
4. Low-speed, low-energy flight within a geofence, away from people.
5. Moving-target trials after validating manual takeover, the kill switch, minimum altitude, and BRAKE/HOLD recovery.

People, animals, and unprotected property must not be used as targets. Preserve line-of-sight operation, local regulatory compliance, site permission, and independent safety-pilot requirements.

## Research directions

The experimental findings motivate the following work:

1. **Staged hardware validation:** read-only operation, gimbal direction and latency, hover dry runs, and low-speed flight within a geofence.
2. **Range without target telemetry:** range and time-to-go estimation from bounding-box scale and optical expansion, with optional fusion of a separate range sensor.
3. **Closed-loop gimbal state:** an encoder, potentiometer, camera IMU, or measured latency model in place of commanded angle alone.
4. **Repeatable, safe terminal behavior:** controlled factorial trials of `N={4,5,6}`, handoff distance and thresholds, climb behavior, and the contact window.
5. **Constant-turn target modeling:** signed `minAreaRect`, gated constant-turn/IMM estimation, and bounded feed-forward in place of a raw bounding-box derivative.
6. **Detector robustness:** data collection and retraining across target types, backgrounds, lighting, blur, and partial occlusion.

Evaluation considers the true CPA distribution, its tail, contact/vibration signatures, altitude aborts, handoff stability, computation time, and safe manual recovery together.

## Detailed documentation

- [Competition and hardware package](competition/README.md)
- [Research status](RESEARCH_STATUS.md)
- [LOS and hardware notes](LOS_HARDWARE_NOTES.md)
- [Test and experiment records](TO_TEST.md)
- [Hardware guide](hardware/README.md)
- [Gimbal notes](GIMBAL_NOTES.md)
- [Guidance log dictionary](guidance_allstar/LOG_DICTIONARY.md)
- [Public release procedure](PUBLIC_RELEASE.md)
- [Third-party licenses and attribution](THIRD_PARTY_NOTICES.md)
- [Data and privacy policy](DATA_POLICY.md)

## Open-source release

The repository was released as open source on 13 August 2026. The release includes:

- An MIT license for original project code and a third-party attribution record
- Version-pinned Python dependencies validated in CI
- Contribution, security, issue, and pull-request templates
- Offline algorithm tests and public-release data checks
- Reproducible simulation and hardware instructions
- A data policy excluding raw flight telemetry from distribution

Release checks are documented in [`PUBLIC_RELEASE.md`](PUBLIC_RELEASE.md). Contributions to simulation scenarios, perception models, aircraft integration, and safe control are welcome.

---

This is a research and competition platform. Strong simulation results do not remove flight risk. Hardware operation requires staged validation, independent pilot supervision, and physical safety limits.
