# LAST_TO_DO — LOS transition to real flight and final research list

Last update: 2026-08-11

This list is in working order. Before moving on to a subheading, the acceptance door of the upper heading must be closed. Every change should be made in univariate A/B; The current `N=4, large-frames=5, area=%3` base is the reference arm.

## frozen reference

- [x] Display law: single `TerminalLosController`, `N=4`, HIT `4 m/s²`.
- [x] Transition: five consecutive fresh and ≥829 px² bbox; No hub/telemetry port.
- [x] Positioned approach purpose: to establish the slot behind the target.
- [x] RoboFly ellipse: 6/6 `<5 m`, CPA median/p90 `1.09/2.03 m`, two contacts.
- [x] RoboFly flat: 8/8 `<3 m`, CPA median/p90 `0.87/1.62 m`, two contacts.
- [x] Unit/closed-loop: `terminal_los_test.py` 15/15 passed.
- [ ] Write the reference snapshot, full command, git diff and log paths to the experiment log before the files/parameters to be changed.

Success is not just CPA. In each experiment, at least the following results will be written together:

`CPA median/p90/p95/max · first-pass physical contact · remaining within FOV · longest bbox loss · ex/ey p95 · lateral acceleration/jerk/saturation · roll RMS/peak/frequency · desired↔actual roll delay · image→command latency p50/p95 · abort/crash`

### 2026-08-11 hardware bench intermediate door

- [x] MAVProxy default 4 Hz stream found to age ATTITUDE/gimbal state `125–237 ms`; With `--streamrate=20` the actual flow became `20.1 Hz`, the image–total mapping became median/p95 `24.5/50.5 ms`.
- [x] It has been proven by dynamic pitch that the `GIMBAL_DEVICE_ATTITUDE_STATUS` pitch is joint angle relative to the body and not world elevation in this servo-mount setup; Fixed body-pitch extraction bug for the second time.
- [x] gimbal raw vertical center error median `0.15–0.20°` in straight, `pitch=-16.7°` and `pitch=+21.7°` postures; expected–reported joint difference `0.01–0.07°` was measured.
- [x] Real camera + physical gimbal + Cube with 90 s LOS output dry-run: `1796/1796 dry=1`, vehicle STABILIZED/excluded; The marking `ex→yaw` and `ey→NED-vz` can be evaluated towards the 100% of samples.
- [x] Added `hardware/balloon_range.py` for balloon testing: only `||target_ned-pursuer_ned||` exits to control path from target telemetry; The target position/speed/direction and `ref_*` fields are excluded.
- [x] Added live fail-closed gates: fixed range forbidden on live; If LOCAL_POSITION 0.50 s, heartbeat 2.0 s or true range 2.0 s are stale, no engagement/no setpoint and current engagement is dropped.
- [ ] Connect the microhard target link to the Pi `14604` endpoint and verify the raw telemetry range with the measuring strip/GPS reference.
- [ ] Verify fresh `LOCAL_POSITION_NED`/rate flow after EKF origin outdoors; These areas were not available in today's table run.
- [ ] New aircraft detector confidence + emit unique inference-id/timestamp; Today's purple detector was able to engage the wrong purple target.

---

## P0 — Propeller video mandatory before flight

### P0.1 Single script writer and forced interlock

- [ ] Make `single_node_state`, `single_node_alive` TTL and/or a single authority key into actual protocol for `_attacker_intercept_thread` in `formation_KILLER.py`.
- [ ] Prove at register level MAVLink that the positional/formation process does not send setpoint and mode command when `ENGAGED` is displayed.
- [ ] LOS Prove controlled fallback/deadman behavior to positioned layer if process happens, TTL drops or bbox drops.
- [ ] Verify in the router log that there is no second independent `SET_POSITION_TARGET` writer on the same vehicle.
- [ ] Add the `config.py` dependency of `formation_KILLER.py` that is not in the repo to the distribution manifest and archive the actual version used.

Acceptance: In the 30 minute props-off integration run, two different controller setpoints will not be seen in the same 100 ms window; All process kill tests will go to the expected safe mode.

### P0.2 Camera direction and coordinate mark

- [ ] Record whether the physical IMX500 assembly is actually 180° reversed.
- [ ] Match the behavior of bench-running `hflip+vflip` with the production `hardware/camera_bridge.py` output on the same raw frame.
- [ ] Take raw 1280×720 frames directly from the camera for seven positions: top-left, top-middle, top-right, bottom-left, bottom-middle, bottom-right, and center.
- [ ] Verify sign at each frame: target right `ex>0`; target `ey<0` according to the above-defined display contract; center `ex≈ey≈0`.
- [ ] Note that the eight available JPEGs in `system_static_tests/` are monitor photos and do not count as intrinsics/time calibration.
- [ ] Do the same test with the `.rpk` detector output; Verify whether bbox coordinates come before or after the rotation.

Acceptance: false sign zero in seven raw frames; center bias within measured tolerance; coordinates are the same between reboots.

### P0.3 Camera calibration and detector agreement

- [ ] Calibrate 1280×720 resolution, actual HFOV/VFOV, `fx/fy/cx/cy` and lens distortion with checkerboard/Charuco.
- [ ] Verify the model's letterbox/ROI coordinates by converting them back to the full image coordinate.
- [ ] `.rpk` select confidence threshold on real dataset with precision/recall and false positive cost; Just don't assume the old `0.70`.
- [ ] Decide whether you want Confidence to be moved to Redis or upstream acceptance should be sufficient; Log the evidence used in the pass.
- [ ] Measure bbox center jitter in pixels/value when changing exposure, gain, blur, rolling-shutter and target size.

Acceptance: calibrate the bbox center and area using a target with known position and size. The false-engagement rate of the 5-frame gate must remain below the stated threshold on the test set.

### P0.4 MAVLink and Redis topology

- [ ] Write separate MAVLink output ports for camera bridge, LOS, positional process and GCS in one schema; Do not allow two processes to connect the same `udpin` port.
- [ ] MAVLink Verify router/MAVProxy source-system IDs and target SYSIDs.
- [ ] Note that `single_node_guidance` is reading Redis from `localhost`; If the processes are on different computers, design a common Redis and time contract.
- [ ] Test that `handoff_state.t_mono` is significant only in the same Linux monotonic clock domain; Do not transfer monotonic timestamps between machines.
- [ ] Log actually received ATTITUDE, LOCAL_POSITION_NED, target telemetry and VIBRATION speeds at startup and throughout the run.

Accept: each process sees the correct vehicle SysID; no port conflicts; When the data rate drops below the minimum requirement, the health gate closes instead of flying silently.

### P0.5 Gimbal operating mode and actual angle

- [ ] Select first flight mode: **recommended first digit is fixed, calibrated gimbal/mount**. Do not mix motion mode in the same flight.
- [ ] Physical camera in fixed mode→measure body angle and verify FC roll/pitch and `VirtualGimbal` marks on right/left bank bench.
- [ ] Select option for floating mode: encoder/pot, actually measured `GIMBAL_DEVICE_ATTITUDE_STATUS`, or measured command→angle observer.
- [ ] Write in the test plan that `SERVO_OUTPUT_RAW` PWM is not mechanical angle; Also measure actual angle during stall/backlash/under load.
- [ ] Command, PWM and perform step/sweep test for real angle; Remove rise time, lag, overshoot, backlash, saturation and load effects.
- [ ] Bench 50 Hz/120°/s with production A/B behavior of approximately 10 Hz/150°/s; Measure the effect of large steps on LOS-rate.
- [ ] Design gimbal status/ATTITUDE reading via a single MAVLink receiver/outpatcher, not the second `recv_match` thread.

Acceptance: the actual angle error of the camera angle used is within the specified limit throughout the flight; If gimbal feedback is stale, no visual command is given.

### P0.6 Camera–telemetry time synchronization

- [ ] Log camera capture time, mid-exposure, end of inference, Redivision broadcast, guidance reading and MAVLink transmission time on the same monotonic basis.
- [ ] Measure `tools/calibrate_gimbal_timing.py` or `--camera-latency-ms` with equivalent physical sweep; default 0 ms release.
- [ ] Interpolate ATTITUDE to the image diameter instant.
- [ ] Prohibit production bridge in flight from silently continuing with `(roll,pitch)=(0,0)` when ATTITUDE is absent/stale; Produce health failure.
- [ ] Measure actual telemetry speed; Note that the approximate 5 Hz ATTITUDE stream in the IRL archive is not enough to resolve the 20 Hz external guidance jitter.

Accept: image→command latency p50/p95 known; Diameter-time attitude error and drop rate are in the log; The artificial yaw/roll phase difference is limited.

### P0.7 Clean install and dependencies

- [ ] Run the import smoke test for two input files in a clean Raspberry Pi environment.
- [ ] Match `requirements.txt` to actual import network; close missing dependency `filterpy`.
- [ ] Fix `python3-picamera2`, `imx500-all`, Redis, OpenCV, pymavlink and model versions.
- [ ] Package the dependency manifest, preserving the repo directory structure; Copying only two entries `.py` file.
- [ ] Get the actual `config.py`, `.rpk`, ArduPilot parameter dump and servo mount parameters in the same release package.
- [ ] Automatically verify Redis, model, camera, MAVLink heartbeat and logging health checks after boot.

Acceptance: on a clean Pi without a network, after reboot, all processes start, produce a dry-run log and understandably fail-closed in case of missing dependency.

### P0.8 Flight envelope and safety

- [ ] Select the first IRL ceiling LOS under the envelope `24–27 m/s` with evidence in the current archive; Do not directly use the current 35 m/s simulation ceiling.
- [ ] Extracts bank, acceleration, jerk, ascent/descension and minimum altitude limits from the log of the actual vehicle.
- [ ] Verify geofence, pilot override, independent kill/release, mode fallback and telemetry-loss procedure in props-off and tethered testing.
- [ ] Add to the acceptance criteria that physical contact with the target crashes the vehicle in two sim runs; Don't let “smaller CPA” get in the way of safety.
- [ ] Reset VIBRATION hit threshold from IRL base. While the median vibration peak in previous attacks was 18,5, the current threshold of `>10` is not evidence of a real hit.
- [ ] Establish impact decision with multiple evidence such as range + acceleration/vibration impulse + speed reduction + health change.

Acceptance: independent security officer/operator can interrupt the command at any time; All failsafe injections end with the expected safe result.

---

## P1 — Measurable vibration diagnosis

### P1.1 Mandatory log columns

- [ ] Raw bbox center/area/confidence and diameter timestamp.
- [ ] Raw LOS, stabilized LOS, `lambda`, `lambda_dot`, `yaw_capture`, `yaw_now`, gimbal command/status/real angle.
- [ ] `a_lateral_raw`, clamped `a_lateral`, forward acceleration, phase, saturation rate and phase change reason.
- [ ] Sent velocity setpoint and per-step lateral acceleration/jerk.
- [ ] Autopilot desired roll/rate, activated roll/rate, gyro, vibration, motor outputs and saturation.
- [ ] Display, inference, Redis, controller and MAVLink times.

Acceptance: “is the command shaking or is it just the tool?” can be answered from a single concurrent chart.

### P1.2 Layer separation experiment

- [ ] Output the roll/roll-rate spectrum for fixed target, flat target and gimbal step.
- [ ] If `lambda_dot` and setpoint are vibrating, press detector/time/guidance arm; If the desired roll is smooth and the actual roll is vibrating, go to the FC tune/mechanical/filter arm.
- [ ] If it occurs only when the gimbal is moving, change the gimbal-airframe duty sharing.
- [ ] Set ArduPilot notch only if motor/gyro frequency is proven; Hiding low frequency guidance oscillation with notch.

Acceptance: No gain/dead-zone changes will be made until the dominant frequency and root layer are determined.

### P1.3 Number of reference repetitions

- [ ] Engagement of at least 20, preferably 50 in straight shim; independent stack starts and identical seed blocks.
- [ ] Same number of repetitions in ellipse.
- [ ] At least 5 repetitions for each cell in safe real flight.
- [ ] Consider the current straight eight engagement as preliminary evidence; final statistics counting.

Acceptance: p90/p95/max and inter-run distribution are reported along with median.

---

## P2 — Reduce vibration without damaging the LOS base

Changes will be made one by one, in the following order.

### P2.1 Diameter-time yaw/LOS phase matching

- [ ] Move the evidenced pattern from `pid_guidance.py` to terminal LOS: move yaw to image capture time and co-differentiate the `lambda=ex+yaw_capture` signal.
- [ ] Blinds the `d_ex + yaw_rate_now` branch of the current code as the A/B reference.

Acceptance: straight route CPA/contact cannot stretch; roll high-pass RMS, command sign change and `lambda_dot` variance decrease.

### P2.2 LOS-rate according to measurement quality

- [ ] Generate `q∈[0,1]` from Bbox space, age, confidence and gimbal movement.
- [ ] At low `q`, just reduce the derivative gain and increase the filter time constant without killing the P/forward closure.
- [ ] Reset/weaken derivative state after Bbox loss.

Acceptance: pseudolateral command is reduced in small/stale bbox; no spike on reacquisition; FOV loss and CPA intact.

### P2.3 Real lateral jerk/constraint governor

- [ ] Add lateral acceleration and jerk limit between successive cycles; Don't mistake the 0.70's accessibility envelope for a real per-step governor.
- [ ] Measured from approximately `a_max≈4 m/s²`, `tau≈1.7 s`.
- [ ] Quickly time the bank/turn-rate limit; Have the governor choose the applicable command closest to the nominal PN.

Acceptance: `+a_max↔−a_max` bounces and roll peak are reduced; closing and straight are protected CPA.

### P2.4 Remove phase chatter

- [ ] For STRIKE, try minimum dwell and different input/output hysteresis A/B or continuous forward-acceleration mix based on lateral authority instead of binary `SETTLE/STRIKE`.
- [ ] Decrease forward veteran continuously as `|a_lateral_raw|/a_max` grows, instead of only cutting at the threshold of `|q|>11°/s`.
- [ ] Use current phase change base as `8,2,7,3,12,2,14,2` in straight test.

Acceptance: the phase change median approaches approximately 2 in normal transition; flat CPA does not worsen significantly from p90 `1.62 m`.

### P2.5 Dead-zone only after measurement

- [ ] Output the actual `lambda_dot` noise distribution on flat target.
- [ ] If necessary, A/B the small and smooth LOS-rate dead-zone derived from this distribution alone.
- [ ] Adding wide `ex` center dead-zone or blind raw derivative cut.

Acceptance: small real maneuvers are not lost; return CPA does not worsen.

### P2.6 Yaw duty sharing

- [ ] Yaw on/off/low-gain FOV-yaw triple A/B enable.
- [ ] Roll/velocity main capture channel; Yaw only helps with low gain and speed limit when the target approaches the FOV edge.
- [ ] Verify that yaw and lateral channel are not chasing the same image error with two fast cycles.

Accept: target remains at FOV, loss does not increase relative to yaw off; roll vibration and camera-yaw cross-feedback are reduced.

---

## P3 — Capturing turns

### P3.1 Actual return base

- [ ] Create/verify straight, right/left fixed `±4, ±8, ±12°/s`, circle/ellipse, sudden change of direction, sine and accelerated-escape routes.
- [ ] Offset back, side and vertical offset starts in each cell.
- [ ] Each cell at least 20, preferably 50 sim engagement; right/left peer seed.
- [ ] Ignore the law comparison if the range, shutdown, approach angle, bbox age/area and gimbal condition at the time of handoff are not balanced.

Acceptance: The actual turn-rate→CPA curve of N=4 and the onset of saturation are known.

### P3.2 Delay up to LOS prediction

- [ ] After time matching is completed, try `lambda_pred=lambda+lambda_dot·delay` only until the measured delay.
- [ ] Clamp prediction with FOV/physical turn-rate; In case of loss, extinguish it to zero in a short time.

Acceptance: `±8/±12°/s` CPA recovers; straight route and wrong direction activation are not disrupted.

### P3.3 Constant-turn/APN with safety gate

- [ ] First install low-order, timestamped LOS/turn observer; Do not directly use the raw bbox second derivative.
- [ ] Turn on limited constant-turn/APN contribution if the maneuver is seen with the same sign and sufficient confidence in several consecutive new frames; When your confidence drops, it drops to zero.
- [ ] `rot_angle_deg/minAreaRect` should remain as a log only feature; No control input until right/left sign and range dependency are verified.
- [ ] APN flight gate: flat phase incorrect activation `<%5`, correct sign `>%80`, significant correlation with actual turn-rate/acceleration.

Acceptance: hard turn CPA p90 drops; flat CPA, FOV loss, jerk and abort do not increase.

---

## P4 — Range, transit and reacquisition

- [ ] Measure 5×%3 transition unchanged on actual `.rpk` video; Write false positive, migration delay and bbox area distribution.
- [ ] Make clear in the UI/log that the toggle is image-only and the terminal is range-dependent.
- [ ] Measure PN/t_go/vertical/missing behavior with fault injection when target telemetry is corrupted.
- [ ] Search for conflict gate between Bbox optical growth (`area` or angular size rate), independent distance sensor and telemetry.
- [ ] Do not consider the absolute bbox length as a range alone; Model the target view/aspect effect.
- [ ] In case of imminent bbox loss, the dead-account will only be a research arm with `R<12 m`, `<1 s`, FOV handcuffed and confidence falling.
- [ ] Verify back-slot setup after FAIL/DROP, without ping-pong and two-writers.

Acceptance: wrong location telemetry does not lead the terminal to dangerous command; Passage performance does not deteriorate through the existing gate.

---

## P5 — To be retrieved from MPC, in order

### P5.1 PN + constraint/reference governor — recommended true hybrid

- [ ] PN maintain nominal acceleration each cycle.
- [ ] Projects to the nearest applicable command for acceleration, jerk, bank/turn-rate, speed, vertical and FOV limit with analytic or small QP in the same cycle.
- [ ] Give bit-same nominal PN fallback if solver fails/timeout.
- [ ] A/B et with current analytical base `_reachable`, not full MPC.

Accept: constraint violated and jitter decreases; CPA/contact and CPU deadlines cannot be exceeded.

### P5.2 Short-horizon LOS-rate/ZEM MPC — research only

- [ ] Status `[lambda, lambda_dot, R, R_dot, a_y]`, limit input to lateral acceleration/jerk.
- [ ] Make the horizon `T_h≤0.6–0.8·t_go` and do not include the horizon after CPA at any cost.
- [ ] Let terminal cost be ZEM/missing distance; post-CPA no bbox space or image center reward.
- [ ] Use measured `tau≈1.7 s`, `a_max≈4 m/s²`.
- [ ] Replay first, then closed-loop sim; actual flight last.

Acceptance: production stanchion is not included if it is not statistically and operationally superior to constraint-governor in two different routes.

### P5.3 Fully perceptive-aware NMPC — last option

- [ ] Evaluate only if P2–P5.2 fails and the missing performance requirement is quantified.
- [ ] Let the model uncertainty, optimizer deadline, fallback and hardware load CPU enter the acceptance criteria from the beginning.

Acceptance: additional complexity does not enter production without satisfying a clear requirement that simple methods cannot solve.

---

## P6 — Real life test ladder

The queue will not be skipped:

- [ ] A. Props-off: clean boot, router, Redis, camera, `.rpk`, video/log, authorization and deadman fault injection.
- [ ] B. Props-off: fixed mount/gimbal seven-point pointing test and dynamic target sweep.
- [ ] C. Props-off: servo step/sweep; Command, PWM, real angle and camera LOS together.
- [ ] D. Fixed tool: `monitor_mpc_commands.py --law los` with shadow command; There is no speed command.
- [ ] E. Hover: camera+gimbal on, LOS dry-run only; The pilot moves the target around the entire frame.
- [ ] F. Low speed, fixed gimbal, large geofence and pilot override: horizontal tracking only FOV; terminal acceleration is off.
- [ ] G. Low speed flat target: limited LOS, no contact; desired/actual roll and latency are verified.
- [ ] H. Low speed rotation target: `±4°/s`, then `±8°/s`; door of acceptance at every step.
- [ ] I. Moving gimbal only after evidence of angle feedback/delay model.
- [ ] J. Terminal closure/contact only after all previous steps and independent safety review have passed.

Mandatory registration on every actual flight:

`raw_value video · detector metadata · camera CSV · LOS CSV · MAVLink tlog/bin · ArduPilot desired/actual attitude/rate · motor outputs · servo command/PWM/ real angle · authority transitions · operator/failsafe events`

---

## P7 — Release/operation

- [ ] Fix the process start order by service/supervisor: router → Redis → camera/health → positioned → LOS standby.
- [ ] LOS If health is not ready, keep the transition fail-closed.
- [ ] Separate log, rotation, disk-full behavior and common run-id for each process.
- [ ] Complete video title on shutdown, return gimbal to safe angle, controller to safe mode.
- [ ] Copy the parameter dump, model hash, git revision/diff, and calibration files to each flight folder.
- [ ] Do not treat `HARDWARE_MISSION.md` or the earlier MPC notes as the current operating procedure. Link the release procedure to this document and `LOS_NOTES.md`.
- [ ] Verify last flight checklist with two people: operator + safety observer.

## Final decision door

LOS/PN remains the production base, if:

- CPA/contact target on straight and turn cells,
- FOV and reacquisition target,
- roll/jerk/abort safety target,
- real time deadline and failover target

if provided at the same time.

PN + constraint governor only corrects this base constraint violation. Full MPC will only be returned if the simpler base cannot meet the quantitative requirement on two different routes and LOS-state MPC shows significant superiority on the same data.
