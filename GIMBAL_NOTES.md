# Gimbal notes — Vertical (tilt) gimbal branch

**Branch:** `gimbal` (2026-08-05). The camera is no longer fixed to the body: it is only attached to a true gimbal that rotates on the vertical axis, **self-stabilizing**. Motivation: some of the framing loss is outside of the physical FOV — the software rotates the gimbal pixel but not the physical FOV.

## What was founded

Ready parts (gimbal NOT written from scratch):

1. **Model:** `gimbal_small_2d` (downloaded from SwiftGust/ardupilot_gazebo) → `~/ardupilot_gazebo/models/gimbal_small_2d/`. Download: `https://raw.githubusercontent.com/SwiftGust/ardupilot_gazebo/master/models_gazebo/gimbal_small_2d/{model.config,model.sdf,meshes/{base_arm,base_main,tilt}.dae}` Settings in installed copy:
   - `tilt_joint` limits ±90° (original −0.1..π)
   - **crash geometries ON IN ORIGINAL condition** (user requirement: for vehicle crash tests). The assembly was moved to z=0.18 so that they would not touch the ground.
   - REMOVED `implicit_spring_damper` + `damping` at the joint — the two were using the ODE joint motor and crushing the plugin's speed-servo with speed=0 at each step, locking the joint (measured)
   - masses: base 0.2→0.005 kg, tilt 0.01→0.002 kg (0.21 kg nose load was disrupting control authority — user reported). Inertias 1e-5 (the smaller one breaks the MARGIN constraint condition, below).
   - built-in 640×480 camera off (`always_on=0`) — main camera webcam
   - **images (mesh/cylinder) removed** (2026-08-05, user request): gimbal fork arms were entering the frame when the stabilized camera rotated relative to the body (nose up → joint down on braking). %100 invisible to camera (dark pixel %0 measured even at extreme angle of −0.9 rad); THE COLLISIONS STOP, and the gimbal is no longer visible in the GUI.
2. **Plugin:** `~/ardupilot_gazebo/src/GimbalSmall2dPlugin.cc` Gazebo has been ported to 11 with **two major changes**:
   - **STABILIZE mode** (`<stabilize>1</stabilize>`): command, WORLD-frame pitch (elevation) of the camera. The articulation automatically compensates no matter how much the body is tilted — true gimbal behavior. Orientation of camera optical axis `<camera_axis>` in local frame tilt_link (`0 1 0` in our assembly). old behavior (joint angle relative to body) with stabilize=0.
   - **speed-SERVO control** (not torque-PID): ODE joint motor, `vel = clamp(kv·error, ±vel_max)`, torque limit `fmax`, dead band. Defaults: kv=150, vel_max=6 rad/s, fmax=0.15 N·m, deadband=0.003 rad. SDF: `<servo_kv> <servo_vel_max> <servo_fmax> <servo_deadband> <initial_angle>`.
   - Compilation: `cd ~/ardupilot_gazebo/build && cmake .. && make GimbalSmall2dPlugin`
3. **Integration:** `models/swarm_drone_{1..5}/model.sdf` — gimbal inchlude (pose `0.35 0 0.18`, yaw −90° → tilt axis body Y), `cam_link` (webcam ditto) welded to the tilt pivot (`0.35 0 0.20`).

## control interface

- command : `/gazebo/default/iris-N/gimbal_tilt_cmd` (GzString, rad, **WORLD elevation**, positive = up; ex. `gz topic -p ... -m 'data: "0.3"'`)
- case : `/gazebo/default/iris-N/gimbal_tilt_status` (rad, ~25 one sample at Hz each 21 physics step; REAL world elevation of camera)
- The Topic name comes from the wrapper model name in the world (plugin connects to the parent model): `worlds/*.world` → `iris-1..5`. Five drones are independent.
- Accuracy: sits within ±0.17° band due to deadband; tracking speed 6 rad/s.

## Verification (2026-08-05, all headless, all PASS)

1. **SITL flight test** (`python3 tools/gimbal_flight_test.py`, decision test — user criteria "press the gas, window remain fixed"): GUIDED 30 m + 15 m/s acceleration (10 s) + brake (5 s). Body pitch **−35.4..+35.2°** camera pitch relative to the world while skidding **max |0.65°|, p95 0.43°**. In steady course (body −25°) the camera is locked at 0.15°. The flight is stable, no failsafe. Baseline (without gimbal, same scenario) is also clean — the script can run the benchmark without gimbal with `FLIGHT_STAB_BASELINE=1`.
2. **Swinging platform** (physical stabilization isolation): camera |p95| while the platform is shaking ±19° 1.37°.
3. **Optical test** (`python3 tools/gimbal_headless_test.py`): 251 px registration in red reference box 0.25 rad tilt (expected 257 in %2); frame stream live; joint command tracking <0.06 rad.
4. **Version:** 5 gimbal standalone (iris-1..5 separate topic, separate targets).
5. **Location:** 30 slip in sec 0.1 mm; gimbal collision geometry ON but not in contact with anything (thanks to mount z=0.18).

## Tuning history / TRAPS (all found by measuring)

- **Torque-PID does not work:** in soft gain the body oscillation passes to the camera (±15 of ±19), in hard gain the reaction torque shakes the carrier and the two enter osulation together. Speed-servo (PAT motor) architecture is a must.
- **`implicit_spring_damper`/`damping` + `SetParam("vel")` conflicts:** Gazebo implements both with the same ODE joint motor; When damping is active, the velocity command is overridden at every step, the joint appears locked.
- **Tiny inertia chain kills EKF:** 1.5 kg welded to the body 2-10 g links inertia 1e-5 produces MARGIN micro-vibration when it goes below → SITL "Gyros not on the ground calibrated", ARM impossible. Keep the inertia cam_link at 1e-4, the gimbal links at 1e-5.
- **fmax balance:** 0.3 N·m + 0.001 inertia → roll on takeoff (AngErr=171). 0.02 → 22° deviation in transitions (acceleration limited). 0.15 + 1e-4 inertia → flight stable AND transitions <0.65°.
- **Dead band condition:** kv=150 was converting the measurement jitter into a continuous speed command and causing vibration to the body (arm obstacle). 0.003 solved the rad band.
- **Camera lazy rendering:** gazebo_ros camera does not render when there is no subscriber; single-frame pull-and-drop returns stale frames. Always stay subscribed (bbox_to_redis already is).
- **`gz topic -e` + `timeout`:** happens before stdout buffer is emptied in low speed topics → empty file. Use flag `-d <seconds>`.

## PHASE C — DYNAMIC TILT TRACKING (2026-08-06, DEFAULT ON)

Tilt is no longer fixed: bbox tracks the target's measured world elevation (−ey; measured INDEPENDENT of tilt, thanks to the live-joint chain — so it's not implicit feedback, but a filtered tracking of the measured magnitude) on each frame.

Parts:
1. **`gz_tilt_pub`** (gimbal_setup/src, C++): command bridge with persistent transport connection. `gz topic -p` was paying ~1 s per stream; eps = hang(down/r) in the terminal 40-70 °/s changes in the last seconds, broadcast with bridge ~ms. They say `install.sh`; Otherwise, TiltCommander automatically falls into the gz-CLI backup (Phase A behavior).
2. **`TiltTracking`** (tools/gz_gimbal.py): EMA filter (tau 0.4 s) + slew limit (60 °/s) + clamp [−30°, +60°] + hold loss policy (3 s → 10 Return to standoff angle with °/s = re-acquisition exposure). Unit tested.
3. **Live ε to steering:** 8 to `tracker_bbox_stab`. element (camera elevation used in that frame) → `Measurement.tilt_deg` → `mpc_guidance._framing_constant` `ey_ref = −(live_tilt + aim)` (if there is no field, static drops to YILDIZ_TILT; old recordings and tilt-off mode are not broken). Added column `tilt_deg` to Guidance CSV.
4. Shutdown button: `bbox_to_redis --tilt-fixed` (Phase A: fixed standoff tilt).

Verification (2026-08-06):
- Bridge ramp: 0→34.4° / 2 s, median tracking error 0.86°.
- Moving target (purple box moved vertically): tilt 9.09→**2.0**→**28.2**→2.0 followed the target; RAW pixel median 356 (center 360) with tilt at 28° — target physically at center of frame; detected transitions %95-100.
- Joint chain consistency max 0.0005°.
- Offline: mpc 86/86, tracking 79/79, static tests complete; `_framing_constant` live tilt unit test (tilt 23.7 → ey_ref −23.7).

NOTICE — fixed trap: bbox `--down` argparse default was stuck at 13 (old +30 assembly value); the wrong default was starting the tilt at 27.5°, throwing the target off FOV, and could NOT acquire tracking at all (tracking cannot start if there is no detection — the acquisition pose relies on standoff geometry). Default reverted to 4 (same as YILDIZ_DESIGN_DOWN); The real source is again standoff_geom.sh.

## WARNINGS / known limits

- **MEASURING CHAIN:** `yildizlar_gimbal.py` de-rotation assumes the camera is fixed to the body. Now the camera is stabilized: the body pitch is NOT reflected in the image AT ALL, so the current de-rotation overcorrects. Before the steering starts giving the tilt command, the chain must be updated: actual elevation of the camera = `gimbal_tilt_status` (world frame, ready data). Files to change: `yildizlar_gimbal.py` + `bbox_to_redis.py` + `tools/set_mounting.py`.
- **`tools/set_mounting.py` became HISTORICAL (2026-08-05).** Write path (`--apply-value`/`--restore-backup`) closed: Writing to the SDF sensor pose imposes a QUIET offset on top of the gimbal commanded world elevation. SDF pose `cam` should remain **0** and no vehicles should write to it. Replaces **`tools/set_tilt.py`**: updates `YILDIZ_DESIGN_DOWN`/`YILDIZ_BACK` in `scripts/standoff_geom.sh` only, reports `YILDIZ_TILT = atan(down/back)`, **verifies** that SDF exposure is 0 (warns if not 0), and achieves terminal framing budget (for down/back eps=hang(down/r) vertical semi-FOV hangs 20.07° at what range → Phase O takeover range).
- New benchmarks of measurement tools: `tools/gimbal_evidence.py` two-layer (physical: |corr(raw_ey,pitch)|<0.3; software: ex·sin(roll) leak raw→stab reduced) + tilt chain health report; `tools/calibrate_gimbal_timing.py` now minimizes roll leakage instead of pitch (no pitch signal left). If there is not enough attitude excitation, both will say "INSUFFICIENT DATA" and exit with 2.
- It does not resolve tilt yaw (horizontal framing must be protected by airframe yaw).
- `~/ardupilot_gazebo` is OUTSIDE the project repository — this file is the only record of the changes there.
