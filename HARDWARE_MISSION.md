# HARDWARE_MISSION.md — Raspberry Pi 5 + IMX500 + servo gimbal fetch task

This file is for the teammate who has just received the repository. Purpose: to move the visual guidance chain (camera → detection → gimbal → MPC) running in the simulation to real hardware. **NO NEED TO INSTALL THE SIMULATION (Gazebo/ROS)** — all the following work is done on the bench, without the sim. Branch: `gimbal`.

## What code goes to the hardware?

Clone the complete repo, but here's what will **work** on the Pi:

| Path | Role |
|---|---|
| `bbox_to_redis.py` (root) | Detect + decision maker + gimbal command. Your IMX500 bridge will be connected to this |
| `guidance_allstar/` | Steering brain: `mpc_guidance.py`, `visual_base.py`, `mavlink_utils.py`, `guidance_config.py` |
| `tools/gz_gimbal.py` | Gimbal tracking law (`TiltTracking`); Gazebo spring will be converted to MAVLink in partial integration |
| `scripts/standoff_geom.sh` | Single source of tracking geometry |
| redis-server (apt) | Interprocess communication — Must be installed on Pi |

The rest (`gimbal_setup/`, `models/`, `worlds/`, `missions/`, `tools/scenario.sh` etc.) **simulation side** — not put into hardware, no need to tinker with it.

## Target architecture (on real hardware)

```
IMX500 AI camera ──> Redis "tracker_bbox" ──> bbox_to_redis.py
                                                 │  (engagement decision + virtual gimbal)
                                                 ├─> Redis "tracker_bbox_stab" ──> mpc_guidance.py ──> MAVLink velocity setpoint
                                                 └─> MAVLink DO_MOUNT_CONTROL ──> otopilot ──> servo (pitch gimbal)
```

In the Sim, the two ends of this chain were different: the detection came from the ROS camera with HSV color threshold, the gimbal command went to the Gazebo topic. **Everything in the middle (control law, stabilization math, Redis contract, MPC) remains the same.** Your job is to connect the two ends to the real hardware.

---

## Task 1 — Pi 5 setup (half hour)

```
sudo apt install redis-server python3-opencv python3-numpy
pip install pymavlink redis
# For IMX500: Raspberry Pi OS Bookworm + picamera2 + imx500 firmware package
sudo apt install python3-picamera2 imx500-all
```

- ROS **install** — sim dependency, not actually used.
- Pi ↔ autopilot connection: UART (57600 or 921600) or USB from TELEM port. Verify that you see HEARTBEAT with `mavproxy.py --master=/dev/ttyAMA0`.

## Task 2 — IMX500 → Redis bridge (main job)

Write a small script (like `hardware/imx500_bridge.py`) that takes the NN detection output of IMX500 and publishes it to Redis with the contract in the sim. Use the interface defined in `bbox_to_redis.py` without changing it:

- **`tracker_bbox`** — 7 element: `[x, y, w, h, coverage_pct, valid_value, t_capture]`
  - `x,y,w,h`: pixels, top-left corner in frame 1280×720 + size
  - `coverage_pct`: ratio of bbox area to frame (%)
  - `valid_value`: 0/1
  - `t_capture`: the moment the frame was captured, in base `time.monotonic()` (produced on the same Pi to avoid time difference — critical)
- Target frame rate ≥ 30 fps; If not detected, broadcast with `valid_value=0`.

Camera internals are already in the code according to IMX500: 1280×720, HFOV 66° (`tools/set_tilt.py`). If you're going to use a different resolution, ask first.

Integration point (stage two, we will do it together): `_detect` (HSV detection) and ROS `Image` subscription in `bbox_to_redis.py` will change with your bridge. For now, just show that the bridge alone is broadcasting correctly: Watch with `redis-cli --csv subscribe tracker_bbox`.

## Mission 3 — Servo + autopilot (after soldering)

We drive the servo NOT from the Pi, but from the AUX output of the autopilot** — ArduPilot's mount driver does the body pitch compensation (stabilization) itself from the IMU, this is the exact equivalent of the Gazebo plugin in the sim.

Autopilot parameters (whichever output the servo is soldered to x its number):

```
MNT1_TYPE        = 1      # Servo gimbal
SERVOx_FUNCTION  = 7      # mount1 pitch
MNT1_MODE        = 2      # MAVLINK_TARGETING (Pi will give the command)
SERVOx_MIN/MAX/TRIM       # calibrate to the physical limit of the gimbal
MNT1_PITCH_MIN/MAX        # 3D printing gimbal's actual angle range (degrees)
```

Calibration: `SERVOx_TRIM` = camera facing fully horizontal; Adjust MIN/MAX without relying on mechanical limits. Verify the linearity of angle→PWM with aciolcer at two known angles (i.e., 0° and 45°).

**Bench acceptance test (no flight, no propeller):** Command `MAV_CMD_DO_MOUNT_CONTROL` to −20°/0°/+40° from Pi; then manually tilt the body ±35° — the camera's world elevation should remain constant (yaw < ~2°). This criterion was proven in the Sim (the camera played 0.65° while the body was ±35°); hardware must pass the same bar. It is enough to measure it with your phone's acometer.

## Task 4 — MPC solver benchmark (OPTIONAL, postponed)

We postponed the processing time concern for now; After the flight, we will look at the `duration_ms` column in the `mpc_diagnostic_*.csv` logs. If you have more time, just run `cd guidance_allstar && python3 mpc_test.py` and send us the printout.

## Traps (known, falling)

1. EXECUTE with flag `--no-tilt` — that branch silently implements the old +30° assembly assumption (argparse default is still 30).
2. `bbox_to_redis.py` opens the gimbal mode with a door facing the Gazebo model name; In real hardware, this door cannot be breached and the gimbal **closes silently**. It will be bypassed during integration (known work, don't touch the code, just remind me).
3. `TiltTracking` software slew limit 60°/s — if the servo is slower than this, we will reduce the limit according to the servo.
4. The gimbal command means **earth elevation** (horizontal = 0, up +). It's NOT angle to the body — it's the autopilot that compensates. Don't be surprised when testing.

## What's ready, what's not (summary)

| Piece | Situation |
|---|---|
| Control law, stabilization mathematics, MPC, Redemption contract | Ready, untouched |
| Translating gimbal command to MAVLink (`tools/gz_gimbal.py` → `TiltCommander._publish`) | ~10 row, integration day together |
| IMX500 → Redis bridge | **YOU (Task 2)** |
| Servo + autopilot param + bench test | **YOU (Task 3)** |
| MPC benchmark on Pi | **YOU (Task 4)** |
