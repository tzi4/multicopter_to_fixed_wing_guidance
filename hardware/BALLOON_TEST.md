# Balloon test: scalar telemetry range

The balloon-test interface uses `range_m = ||target_ned-pursuer_ned||` from target telemetry. LOS direction, elevation and bearing come from the camera image. Target position, velocity and direction must not be passed to the controller or `ref_*` log fields.

## Components in this checkout

`balloon_range.py` provides `RawTelemetryRange`, which reads target `GLOBAL_POSITION_INT` messages and returns scalar range. Its offline tests are in `test_balloon_range.py`.

`single_node_guidance.py` supports the `estimator` and `redis` range sources. It does not connect `RawTelemetryRange` to its command-line interface. The balloon deployment used a separate entry point called `visual_guidance.py`, which is not included in this checkout. Its target-link and altitude-source options therefore cannot be passed to the tracked single-node script.

## Balloon deployment interface

The required data flow is:

- Microhard target `GLOBAL_POSITION_INT` → `RawTelemetryRange` → scalar `range_m`.
- Camera/model → `tracker_bbox` → `bbox_to_redis` → `tracker_bbox_stab` → `TerminalLosController`.
- Cube `LOCAL_POSITION_NED` and `ATTITUDE` → own-vehicle state and authority gates.

The deployment must reject a fixed range source during live operation. Live control requires `GUIDED`, a fresh heartbeat, fresh `LOCAL_POSITION_NED` and fresh measured range. Release authority without sending a command when own-state age exceeds 0.50 s, heartbeat age exceeds 2.0 s, or range age exceeds 2.0 s.

Use relative altitude when both vehicles establish home at the same ground elevation. Use AMSL when home elevations differ and the AMSL measurements are reliable. `RawTelemetryRange(relative_alt=True)` selects relative altitude; `relative_alt=False` selects AMSL.

A target telemetry relay can supply the deployment's target-link endpoint:

```sh
mavproxy.py --master=<MICROHARD_TARGET_SERIAL>,<BAUD> --out=udp:<PI_IP>:14604 --streamrate=10 --source-system=253 --non-interactive --no-state
```

## Estimator dry-run from the source checkout

From the repository root, with the camera, Redis and own-vehicle telemetry configured as described in [the hardware guide](README.md):

```sh
python3 -u hardware/single_node_guidance.py --guidance-value los --large-frame 5 --area-pct 3 --dry-run --no-authority-write --no-statustext --range-source estimator --log logs/competition_estimator_dry.csv
```

This exercises the tracked estimator path. It does not perform the separate balloon telemetry integration. Keep the dry-run settings until the deployment's telemetry integration, logs and authority gates have been verified.
