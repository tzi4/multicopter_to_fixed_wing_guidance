# Competition codes — flight package

This folder makes the final visual guidance line to be used in the competition and on the real vehicle visible in one place. The current default law is the **Terminal LOS/PN** controller in `terminal_los_guidance.py`: `N=4`, hit acceleration `4 m/s²`, `5` large/fresh bbox repeatedly for switching and `%3 × %3` area threshold. MPC is not deleted; only the A/B comparison and fallback option are preserved.

## Files in folder

| File | Mission | Source/situation |
|---|---|---|
| `camera_bridge.py` | IMX500/USB camera → detection → Redis → real gimbal | Current `hardware/camera_bridge.py` flight copy |
| `single_node_guidance.py` | Engagement decision, LOS/PN, safety and speed command MAVLink | Current `hardware/single_node_guidance.py` flight copy |
| `terminal_los_guidance.py` | The final guidance law to be used in the competition | Current copy of `guidance_allstar/terminal_los_guidance.py` |
| `gimbal_bench_tracking.py` | Camera and servo-gimbal table tracking | Hardware version from Raspberry Pi 5 + IMX500 works |
| `mavlink_tilt.py` | ArduPilot servo-mount tilt command and channel scan | Verified version on actual ArduPilot 4.6.3/servo-mount |
| `monitor_mpc_commands.py` | Monitors LOS or MPC output without driving the vehicle | Read-only acceptance tool for real hardware |

Flight copies have been frozen along with the main sources in this commit. Common dependencies (`guidance_allstar/`, `tools/`, `bbox_to_redis.py`, and `yildizlar_gimbal.py`) are used from the repository root; so run the commands in the repository root.

## Raspberry Pi 5 works

Some parts of this line were developed on Raspberry Pi 5 and Raspberry Pi AI Camera (IMX500). The following details revealed in the field were included in the code:

- Using `Picamera2` instead of OpenCV `/dev/video0` on Pi 5/libcamera line;
- 1280×720 image, 66° horizontal viewing angle and 180° rotation for reverse mounting;
- HSV band operating in the real image and moment center reducing vibration;
- Ramp servo command for EMAX EQ08MD;
- `MAV_CMD_DO_MOUNT_CONTROL` path running on ArduPilot servo mount.

Therefore the folder contains both the hardware references used on the Pi 5 and the final competition codes ported from them to the mainline.

## flight order

First start the camera, Redis and gimbal:

```bash
python3 competition/camera_bridge.py \
  --source-value picamera2 --width-value 1280 --height-value 720 --hfov 66 \
  --detector-value yolo --yolo-model MODEL.rpk --yolo-conf 0.70 \
  --mavlink udp:127.0.0.1:14554 --save
```

First do the dry acceptance run with the propellers disabled:

```bash
python3 competition/single_node_guidance.py \
  --guidance-value los --large-frame 5 --area-pct 3 --dry-run --duration-value 60
```

Activate competition line after geofence, altitude floor, manual override, telemetry, gimbal orientation and dry log are verified:

```bash
python3 competition/single_node_guidance.py \
  --guidance-value los --large-frame 5 --area-pct 3 \
  --loop-hz 20 --range-source estimator
```

## readiness

The software line is **flight ready** with competition configuration: real camera and gimbal bridge, single node controller with self-engagement decision, current LOS/PN law, dry run mode, drop/altitude hold and logging ready together. In the RoboFly/ellipse verification the `N=4` arm went below the 5 meter in the 6/6 engagement and produced physical contact on two separate starts.

This statement describes software preparation. Mechanical check, control override, geofence, propeller safety, correct servo orientation and dry acceptance test must be performed again before each actual flight. See file `../LOS_HARDWARE_NOTES.md` for detailed field sequence and `../hardware/README.md` for hardware architecture.
