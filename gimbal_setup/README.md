# Gimbal setup: external components for the physical gimbal

The gimbal setup has two parts. **The external plugin and model must be installed in addition to this repository**:

| Piece | Where | What |
|---|---|---|
| 1 | This repo (gimbal branch) | `models/swarm_drone_*/model.sdf` (gimbal include + camera tilt pivot), `bbox_to_redis --tilt` chain, `yildizlar_gimbal.joint_angle`, `tools/gz_gimbal.py`, tests. **World files HAVE NOT CHANGED.** |
| 2 | `~/ardupilot_gazebo` (non-repo) | Model `GimbalSmall2dPlugin` (Gazebo 11 port + STABILIZE mode + speed-servo) and `gimbal_small_2d` (meshes included — needed because tilt collision uses mesh). This folder contains a copy and installer for that component. |

## Teammate setup (assuming our setup)

Prerequisites (skip if already installed): Ubuntu 20.04 + ROS Noetic + Gazebo 11 (classic) + ArduPilot SITL compiled (`~/ardupilot`) + SwiftGust/khancyr derivative `~/ardupilot_gazebo` (ArduPilotPlugin compiled tree) + `redis-server`
+ `pymavlink/mavproxy` + python3 `numpy opencv rospy`.

```bash
git clone <repo> && cd <repo> && git checkout gimbal
./gimbal_setup/install.sh          # install plugin + model into $ARDUPILOT_GAZEBO_DIR and build
python3 tools/gimbal_headless_test.py   # expected result: PASS (~2 minutes)
python3 tools/gimbal_flight_test.py       # SITL flight proof: camera <0.65° (~4 min) while fuselage ±35°
```

Normal flow afterwards: `./yildizlar_guidance.sh --headless` or single command trial `DURATION=300 tools/scenario.sh`. Gimbal DEFAULT ON; `--without-gimbal` / `YILDIZ_GIMBAL=0` for body-fixed legacy behavior.

- Command interface: `/gazebo/default/iris-N/gimbal_tilt_cmd` (rad, EARTH elevation, positive=up), true angle `.../gimbal_tilt_status` (~18 Hz).
- Tilt from one source: `YILDIZ_TILT = atan(down/back)` (`scripts/standoff_geom.sh`; setting `tools/set_tilt.py`).
- Detail + measured traps (implicit_spring_damper overlap, inertia/EKF, lazy render...): `GIMBAL_NOTES.md`.

## Notes for different installations

- If `ARDUPILOT_GAZEBO_DIR` is different: `ARDUPILOT_GAZEBO_DIR=/path ./gimbal_setup/install.sh` and `yildizlar_guidance.sh` with the same env.
- `install.sh` recognizes and removes port `GAZEBO_VERSION < 8.0` in CMakeLists; check the warning if your CMake layout differs (make target unconditional manually).
- Source model: SwiftGust/ardupilot_gazebo `models_gazebo/gimbal_small_2d` (in our copy: limits ±90°, mass ~7 g, images removed because they occupied the camera frame, COLLISIONS ON, plugin stabilize+servo parameters in SDF).
