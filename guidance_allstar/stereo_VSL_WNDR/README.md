# stereo_VSL_WNDR — bearings-only target tracking

The position feed is gone. The sensor is now two cameras that each report a
**direction** to the target, with known yaw/pitch error, from rays that
essentially never intersect.

The central design decision, and the thing to understand before reading any
code here:

> **Four angles are the measurement. A position is not.**

Triangulating first and handing the filter "a position with some R" throws away
the one thing that matters about this sensor — that its error is wildly
anisotropic — and forces a lie about R. Instead the IMM is updated directly on
`[yaw_L, pitch_L, yaw_R, pitch_R]`, each with its own sigma. Triangulation
survives only as a seed, a turn-rate hint, and a diagnostic.

Three things fall out for free:

* **Non-intersecting rays stop being a problem.** There is nothing to
  intersect. Skew becomes a *quality signal* instead of an error to hide.
* **One camera still updates the track** (two constraints instead of four). No
  stereo pair required, no dropout hole — measured below.
* **Per-angle gating** comes naturally, so one camera's false detection cannot
  drag the state.

## The physics you are fighting

```
cross-LOS error  ~  R · σ_angle          (easy:  0.23 m at 300 m, 1 mrad)
depth error      ~  R² / b · σ_angle     (brutal: 23 m, same conditions)
```

Position uncertainty is a long thin cigar pointed down the line of sight
(verified: `test_stereo.py` asserts the covariance's dominant axis aligns with
the LOS to >0.95). Every design choice here follows from that ~100:1 anisotropy,
and every number is reported split into along-LOS and cross-LOS. A single RMS
figure hides the whole character of the sensor.

## Files

| file | role |
|---|---|
| `stereo_config.py` | camera rig, noise, gating, init/coast tunables |
| `stereo_geometry.py` | `Camera`, rays, skew, ML triangulation, GDOP covariance |
| `stereo_measurement.py` | sequential scalar EKF bearing update into the IMM, gating, boresight estimator |
| `stereo_estimator.py` | `StereoTracker` — init → track → coast → lost, plus a triangulation-only baseline to beat |
| `virtual_stereo_rig.py` | **the test bench**: two virtual cameras, noisy rays, scoreboard |
| `test_stereo.py` | 39 correctness checks (`python3 test_stereo.py`) |
| `filterwndr.py`, `mavlink_utils.py`, `vector_math.py`, `guidance_config.py`, `simple_guided_follow.py` | copied unchanged from the position-era stack |

`simple_guided_follow.py` is copied for the eventual guidance integration and is
**not yet wired to the stereo front end**.

## Running the test bench

```bash
# canned racetrack, no SITL needed
python3 virtual_stereo_rig.py --source synthetic --duration 90

# replay REAL flight truth (a guided_follow log's meas_* is the plane's true track)
python3 virtual_stereo_rig.py --source log --log ../logs/guided_follow_20260724_110805.csv \
    --slew --fov 40 --plot

# live against the plane
python3 virtual_stereo_rig.py --source mavlink --target udpin:localhost:14600 --slew

# stress it
python3 virtual_stereo_rig.py --source synthetic --dropout 0.15 --outlier-rate 0.03 \
    --bias-deg 0.05 --baseline-tilt 90
```

Useful flags: `--baseline`, `--sigma-deg`, `--baseline-tilt`, `--slew` (gimballed
rig — aims at the *estimate*, never at truth), `--calibrate`, `--out`, `--plot`.

## Measured results

Synthetic racetrack, 6 m baseline, 1.05 mrad cameras, ~420 m median range:

| | total | along-LOS (depth) | cross-LOS |
|---|---|---|---|
| triangulation only (per frame) | 31.5 m | 27.9 m | 0.42 m |
| **IMM on angles** | **6.3 m** | **6.0 m** | **0.17 m** |

Straight-line target (`test_stereo.py`): **1.6 m median** where the per-frame
CRLB depth is 23 m — that gap is time. The IMM accumulates angles across the
target's motion, which is a far bigger depth lever than the 6 m baseline.

Covariance calibration comes out at `|error|/σ ≈ 1.1` on both axes — the filter
knows what it does not know, which is what keeps the downstream covariance gate
honest.

**Losing a camera:** with the right camera dark from t=10 s, the track survives
on one camera at 3.3 m median error. A triangulation-first design cannot do this
at all.

### The finding that should drive your hardware

A **differential boresight misalignment** is far more damaging than sensor
noise, and the baseline direction decides which axis hurts:

| baseline | yaw bias 0.05° | pitch bias 0.05° | no bias |
|---|---|---|---|
| horizontal | **33.8 m** | 4.0 m | 4.0 m |
| vertical | 4.6 m | **19.5 m** | 4.6 m |

> **The angular axis parallel to your baseline carries the depth information —
> and therefore its misalignment maps straight into depth error. Calibrate that
> axis hard; the perpendicular one is nearly free.**

A horizontal pair (the obvious build) is worst-case for **yaw** slop, which is
exactly where a pan/tilt mount tends to be loose. With a horizontal baseline,
0.05° of yaw error costs 34 m of depth and leaves the filter **5.5× over-
confident** — biased *and* confident, the one combination the covariance gate
cannot defend against.

Online self-calibration (`--calibrate`) recovers the misalignment when it is
observable (pitch on a horizontal baseline: 0.0246° recovered vs 0.025°
injected) but is **structurally blind to yaw on a horizontal baseline** — two
yaw measurements against two horizontal unknowns leaves no redundancy, so the
error is absorbed silently as a depth shift. Controls showed tilting the
baseline fixes the *geometry* (34 m → 10 m) while the calibrator's correction
adds nothing on top. **Treat the estimator as a diagnostic ("your rig has
drifted"), not as a substitute for surveying the extrinsics.**

## Real hardware

The stack has now been run against real VSL ground-stereo logs scored on the
target aircraft's own flight-controller telemetry — see
**[REAL_DATA_FINDINGS.md](REAL_DATA_FINDINGS.md)**. Headlines:

* the ray-construction replica matches VSL's own logged output to 4.5e-05 deg,
  so the numbers are comparable to what their triangulator saw;
* a 1° longitude typo gave CAM2 an **84 km baseline** for nine minutes of flight
  (their own selfcal header records `cam2_enu=-84170.718,...`);
* the stored boresight table is silently invalidated by changing the
  hand-entered `Gimbal_Heading`, which makes two of three sessions uncalibrated;
* measured angular noise is **0.2–0.5 deg/axis**, versus the 5 px their window
  triangulator assumes (5–35x optimistic);
* it exposed a real bug in *this* stack — the triangulation turn-rate hint
  diverged the IMM (p90 5.8 km, velocity 1373 m/s on a 20 m/s target).
  `USE_TRIANGULATION_TURN_HINT` now defaults to `False`; it did not pay for
  itself on the synthetic bench either.

Held-out best window (100.8 m baseline, R ≈ 250 m, n = 995): IMM **2.5 m** depth
vs per-frame triangulation 2.8 m vs VSL's own logged output 8.1 m.

Entry point: `python3 vsl_run.py [--only 13:01]`.

## Known gaps / next steps

* Not wired to guidance. Splitting consumers by error axis is the intended
  design: pursuit off *bearing* (excellent), terminal timing off τ = θ/θ̇
  (angle-domain, no range needed), CPA off bearing-rate reversal.
* Cameras are assumed static or gimballed with known pose. Rig-on-pursuer needs
  attitude-error propagation into the angular noise budget.
* No monocular size/looming channel yet (an independent range cue whose error
  grows like R, not R²/b — it should beat stereo depth at long range).
* Deliberate observability manoeuvres (a midcourse weave to manufacture
  parallax) are unexplored and are probably the largest remaining depth win.
* `--source mavlink` is code-complete but has only been exercised against
  synthetic and log sources; it needs a live SITL run.
