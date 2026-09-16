# GIMBAL CONTINUOUS TARGET TRACKING - BENCH TEST (For Aziz Başar)

This document is for bench testing the servo gimbal on the real drone **self-tracking a purple target**. You can give this file to a Claude/agent and say “run this test” — there are instructions for the agent at the end of the file.

## WHAT this test is NOT (know this first)

* **MPC/guidance does NOT operate.** The tracking mechanism is on the sight side (`tools/gimbal_bench_tracking.py`), `mpc_guidance` does not undergo this test.
* **Not armed, no propeller attached.** The drone is on the table, disarmed.
* One process, one script; ROS/Redis/sim NOT REQUIRED.

## Architecture (30 updates per second)

```
camera frame → purple HSV detection → vertical error (ey)
   → TiltTracking law (filter + 45°/s rate limit + clamp + target-loss policy)
   → MavlinkTiltCommander (DO_MOUNT_CONFIGURE + DO_MOUNT_CONTROL)
   → ArduPilot mount → servo
```

The same law was proven in the simulation: when the target was moved vertically, the tilt followed 9°→2°→28°→2°, keeping the target dead center in the raw pixel. Stabilization (keeping the camera steady when the body is tilted) is done by ArduPilot's own mount driver — the script just says "look where".

## Current calibration status (2026-08-07)

* The servo channel is in **reversed** convention (tick mark in MP) — DO NOT CHANGE THIS, calibration was done accordingly.
* `MNT1_PITCH_MIN = -38`, `MNT1_PITCH_MAX = +58` (measured mechanical limits).
* Command 0° = camera MUST be parallel to the horizon. Verify before starting the test: Send 0 from MP Payload Control → camera should be facing straight; Send +20 → ~20° up (via phone protractor). If it doesn't work, fix this FIRST (rederive MNT1_PITCH_MIN/MAX with "angle = (PWM − flat_PWM)/8.89").

## Step by step

### 0. Update repository
```bash
git pull    # gimbal branch; tools/mavlink_tilt.py and gimbal_bench_tracking.py should be there
```

### 1. Establish the MAVLink connection

If the autopilot is connected via USB, the device is `/dev/ttyACM0`. For failure to connect:

1. Is there a device: `ls /dev/ttyACM*` (if not, change the cable/port, see what appears after plugging and unplugging with `dmesg | tail`).
2. **Permission**: `sudo usermod -aG dialout $USER` → logout-login (this is the most common reason; this is for sure if the error is "Permission denied").
3. **Is another process holding the port**: If Mission Planner/QGC is open, it holds the serial port, the second process cannot be connected. Use MAVProxy to split the connection:
   ```bash
   mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 \
       --out=udp:127.0.0.1:14601 --daemon
   ```
   and give scripts `--connection-value udpin:127.0.0.1:14601`. (If you add a separate `--out` to the MP, both will work at the same time.)
4. On Ubuntu desktop **ModemManager** can grab ACM: `sudo systemctl stop ModemManager` (persistent: `sudo apt remove modemmanager`).
5. If you are going to connect directly, the address `/dev/ttyACM0` is also valid: `--connection-value /dev/ttyACM0`.

### 2. Sweep test (chain of command proof, ~30 sec)
```bash
python3 tools/mavlink_tilt.py --connection-value <address> --channel-value 5
```
(`--channel-value` = Mount1Pitch assigned servo output; in our case 5.) Expected: servo physical movement + "MOTION PRESENT" on commands 0/+30/−30. This was confirmed in SITL (PWM 1500/1765/1233 one-to-one linear).

### 3. Follow-up demo (actual test)
```bash
python3 tools/gimbal_bench_tracking.py \
    --source-value <camera> --connection-value <address> \
    --tilt-alt -35 --tilt-upper 55 --display-value
```
* `--source-value`: `0` if running on Pi; If `cv2.VideoCapture(0)` does not open in the Pi's libcamera stack, give the image to UDP with the head and URL: `rpicam-vid -t 0 --inline --width 1280 --height 720 --codec h264 -o udp://127.0.0.1:8554` → `--source-value udp://127.0.0.1:8554`. (If the team is already streaming the image another way, use that URL.)
* Grab a purple/magenta object (HSV H 140-160 band — same as sim target).
* **Success criterion**: when it moves the object up/down, the servo tracks it within ~half a second and the object is centered vertically in the image. If you hide the object, it waits for 3 seconds, then slowly returns to 0°.
* Exit Ctrl-C — the script drops the gimbal to 0 and exits.

### 4. (Bonus) Stabilization check
While tracking is running, tilt the drone ±20° from its nose with your hand: the horizon in the image should remain in place (ArduPilot does this, not the script). If not, the mount stabilization parameter is off — write it in the QUESTIONS section below, we will tell you the parameter according to your ArduPilot version.

## Known pitfalls (measured in simulation/SITL)

* Even if `DO_GIMBAL_MANAGER_PITCHYAW` is ACKed, the servo may not play — that's why the script uses `DO_MOUNT_CONFIGURE(MAVLINK_TARGETING)` + `DO_MOUNT_CONTROL`. If you tap the MP's payload control, the mount mode can change; Just restarting the script is enough.
* TWO processes cannot recv to the serial port at the same time (pymavlink is not thread-safe) — so the mavproxy splitter is recommended.
* Servo 3M stops with tape: if it shifts to zero in jerk, repeat the "0 command = straight view" check.

---

## INSTRUCTION TO AGENT

If you are the agent running this test: follow the steps above in order, recording the OUTPUT of each step (command + observation) in short notes. If a step does not go as expected, do not change the architecture on its own — specifically, `tools/mavlink_tilt.py`, `tools/gz_gimbal.py`, and the MNT/SERVO parameter convention (reversed + −38/+58) must remain unchanged.

**If you have any questions or problems, ask us**: open the `## QUESTIONS` section at the bottom of this file and write the question (with date + observed output), commit and push or forward it to tzi4. The agent who wrote this repository (owner of the gimbal branch) will see and answer the questions from there. ANY time you're unclear, it's better to ask than to guess.
