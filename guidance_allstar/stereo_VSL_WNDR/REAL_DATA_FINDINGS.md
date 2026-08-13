# Real VSL stereo data — findings

Analysis of the VSL (ITU/ITUNOM) ground stereo rig logs delivered 2026-07-28,
scored against the target aircraft's own flight-controller logs.

Everything below is measured, not assumed. The pipeline that produces it:

| file | role |
|---|---|
| `vsl_ingest.py` | faithful replica of VSL's ray construction; **validated against their own logged output** |
| `vsl_truth.py` | `.bin` dataflash → target truth on the unix clock |
| `vsl_eval.py` | window discovery, rig geometry, bearing-residual characterisation |
| `vsl_calibrate.py` | robust fit of per-camera boresight / station altitude / latency |
| `vsl_run.py` | held-out scoring: IMM vs triangulation vs VSL's own output |

Run: `python3 vsl_run.py` (add `--only 13:01` for the cleanest window,
`--per-window` to fit boresight per window instead of pooling per site).

---

## 0. The replica is exact

Before trusting any number, the transcribed geometry was checked against VSL's
own logged ray angles (`selfcal_log.txt` carries both inputs and outputs):

```
CAM1: n=2067  |dH| med=4.5e-05 deg  max=3.7e-04
CAM2: n=2067  |dH| med=3.4e-05 deg  max=2.8e-04
```

That residual is the log's own 5-decimal print precision. The intrinsics LUT,
digital-zoom scaling, `ZYX` euler composition, ENU↔NED handedness and boresight
application are therefore identical to theirs — later disagreements are data,
not a porting bug.

The `allstar` logs report raw encoder pitch; elevation is
`wrap180(180 − Encoder_Pitch)`, confirmed against time-matched selfcal rows at
**0.000° median** on both cameras.

Folder names do **not** match content dates (folder `21` holds a 23 July flight,
folder `23` holds 21 July). Everything is paired by resolved unix time instead.
`.bin` time base comes from a robust fit over all `GPS.GWk/GMS` rows
(residual 7–10 ms).

---

## 1. CAM2's longitude was wrong by 1.0° — an 84 km baseline

In the 23 July session, `Gimbal_Lon` for CAM2 appears with two values one
degree apart (absolute site coordinates removed):

| value | share | when |
|---|---|---|
| expected longitude `L` | 17.5 % | 15:53:51 → 16:02:43 UTC |
| erroneous `L + 1°` | 82.5 % | 16:03:26 → 16:33:51 UTC |

The erroneous value puts CAM2 **84 170 m west and 556 m below** CAM1 — confirmed
verbatim in that session's selfcal header:

```
cam1_enu=0.000,0.000,0.000   cam2_enu=-84170.718,389.030,-555.840
```

Downstream, their triangulator emitted target altitudes of **15 032 m** and
ranges of 27 km / 70 km. Someone corrected it mid-session at 16:03:26. The real
baseline is **92.6 m**.

**Worth a config guard**: reject any camera pair whose baseline is not within a
plausible range (say 5–2000 m) at startup, rather than letting a typo silently
produce 15 km altitudes.

## 2. The boresight table is coupled to the hand-entered heading

`Gimbal_Heading` is a manually entered mount azimuth, and `boresight_offsets.json`
was calibrated on 21 July with `calib_heading_deg = 143.0` (CAM1) / `126.0` (CAM2).

| session | site | heading cam1/cam2 | matches calibration? |
|---|---|---|---|
| 21 Jul | Site A (coordinates removed) | 143 / 126 | **yes** |
| 23 Jul | Site B (coordinates removed) | 40 / 37 | **no** |
| 27 Jul | Site A, second setup (coordinates removed) | 81 / 128 | **no** |

Because the stored offset absorbs whatever heading constant was in force when it
was taken, changing the heading invalidates it. On the 23 July session CAM1's
rays sit a median **37.6°** off the truth aircraft with the table applied, and
**0 %** of samples land inside 2° — consistent with the 15 km altitudes that
session produced. It is recoverable (a consistent +42.9° offset, §8), but not by
their pipeline as configured.

**Recommendation**: store the heading constant *inside* the boresight record and
refuse to apply the table when the live heading differs. The record already
carries `calib_heading_deg` — it just is not checked.

## 3. Independent confirmation that the table's values are right

Fitting boresight from truth *without* using their table recovers what they
stored, from three independent windows:

| | recovered (fit-only) | stored table |
|---|---|---|
| CAM1 yaw | +7.0, +7.3, +7.5, +7.6° | +7.8 … +8.7° |
| CAM2 yaw | +14.9, +14.9° | +12.7 … +14.4° |

So their calibration procedure is sound. Applying the table leaves a residual of
**−0.3 … −1.4° (CAM1)** and **+0.8 … +3.2° (CAM2)** — CAM2's is the staler.

## 4. Measured angular noise: ~0.2–0.5° per axis, not 5 px

After removing the systematics, the residual scatter on inliers is:

| window | CAM1 σ yaw/pitch | CAM2 σ yaw/pitch |
|---|---|---|
| 21 Jul 12:02 | 0.41 / 0.16° | — |
| 21 Jul 12:45 | 0.50 / 0.34° | 1.20 / 0.15° |
| 21 Jul 13:01 | 1.07 / 0.34° | 0.23 / 0.18° |
| 21 Jul 15:38 | 0.68 / 0.33° | 0.81 / 0.61° |

Their two assumptions sit either side of this:

* `selfcal_log` `sigma_ang_deg = 0.35` — **about right**.
* `WINDOW_TRI_SIGMA_PX = 5.0` — the measured equivalent is **25–175 px**, so
  this is optimistic by 5–35×. A window triangulator fed that σ will be badly
  overconfident. (`WINDOW_TRI_ENABLED` is currently `false`, so it is not
  hurting anything yet — but it would if switched on.)

Image latency behind the logged timestamp fits at **0.05–0.55 s**, bracketing
their declared `0.308 + 0.087 = 0.395 s`.

**Station altitude is not separately identifiable** from pitch boresight: the
fitted `dAlt`/`dPitch` correlation is **0.96–1.00** in every window, because the
target's range barely varies. Reported `dAlt` values (−11 … +15 m) are therefore
meaningless individually — worth knowing before anyone tries to "fix" the
altitude from a fit like this. Note that 21 July logs `Gimbal_Alt = 0.0` for 89 %
of rows, which is certainly wrong and gets absorbed into pitch.

## 5. The detector is often not on the target aircraft

Fraction of `LOCKED` samples whose ray points within 2° of the truth aircraft,
after full calibration: **20 – 93 %**, window dependent. In the 12:45 window the
median ray skew in the scored half is **54 m**, i.e. the two cameras are locked
onto different objects almost the whole time.

This is why the skew gate matters: it separates "both cameras on the plane" from
"one camera on a bird" **without consulting truth**, and it is what makes the
scoring below possible at all.

---

## 6. A real bug this exposed: the turn-rate hint diverges the IMM

The synthetic bench never caught this. On real data the IMM was producing a
**p90 of 5.8 km** and a velocity state reaching **1373 m/s on a 20 m/s target**.

Cause: `USE_TRIANGULATION_TURN_HINT` fed `HeadingTurnRateEstimator` a
differentiated per-frame fix. Real frames arrive at ~20 Hz with ~3 m fix noise,
so the difference is essentially pure noise, and it drove the CT modes hard.

| variant | total med | total p90 | \|v\| p99 |
|---|---|---|---|
| hint on (as shipped) | 15.9 m | 8131 m | 1876 m/s |
| **hint off** | **7.9 m** | **183 m** | 467 m/s |

Decimating to 4 Hz is **not** sufficient — the differenced noise is still ~12 m/s.
And the hint does not pay for itself even where it is safe: on the synthetic
bench it is marginally *worse* than off at both baselines (5.86 → 5.47 m at 6 m;
0.43 → 0.41 m at 100 m). The IMM's own mode mixing recovers turn rate perfectly
well.

**`USE_TRIANGULATION_TURN_HINT` now defaults to `False`**, with a quality/rate
gate (`TURN_HINT_MIN_DT_S`, `TURN_HINT_MAX_SKEW_M`) retained for anyone who
enables it against a clean fix stream. 39/39 existing checks still pass.

---

## 7. Held-out scoreboard

Systematics fitted on the **first half** of a window, estimator scored on the
**second half**. Truth is used only for that calibration and for scoring — never
inside the tracker, which sees four angles.

Rows whose logged baseline falls outside **80–150 m** are dropped as config
noise (`vsl_eval.baseline_mask`); segments are re-derived after that gate, so the
23 July flight yields a healthy 1868 s window instead of one straddling the 84 km
period. Every fit is then graded (`CamCalibration.suspect`) — 5 of 7 windows fail
the grade and are excluded from the headline.

### The two credible windows

**21 July 13:01 — common dual-camera frames (n = 1008), b = 100.8 m, R ≈ 250 m**
Boresight pooled across the site's five windows (first halves only), scored on
this window's held-out second half.

| estimator | total med | depth med | cross med | total p90 |
|---|---|---|---|---|
| **IMM (this stack)** | **5.3 m** | **4.7 m** | 2.18 m | 47.3 m |
| per-frame ML triangulation | 5.6 m | 5.1 m | 2.14 m | 44.7 m |
| VSL's own logged output | 12.5 m | 7.6 m | 9.18 m | 61.0 m |

Switching from per-window to per-site pooling improved the estimator as well as
the honesty of the calibration: the IMM's **p90 fell from 103.8 m to 47.3 m**,
now level with triangulation, because a boresight fitted on five windows is
stable enough that the filter stops re-acquiring. It also produces a position on
**1474 frames against triangulation's 1060**, coasting through the 46 % of frames
where only one camera holds a lock.

**21 July 15:38 (n = 425)** — calibration grades clean (offsets ≤ 3.3°,
σ ≤ 0.81°) yet every estimator lands ~170 m from truth, with a ray skew of only
10 m. Both cameras agree with each other on a point that is not the target
aircraft. **This is the limitation of the skew gate**: it verifies that the two
cameras concur, not that they concur about the right object. Catching this needs
a second cue — track continuity, or bbox angular size versus predicted range.

### The decisive window for "is the estimator the bottleneck?"

**21 July 12:02 (n = 4463)** — calibration is flagged suspect (CAM2 needs 49°,
supported by 9 % of frames) so the absolute numbers are not headline material,
but the *structure* is the cleanest evidence in the whole dataset. Ray skew
median **0.90 m**, p90 2.1 m: the rays intersect essentially perfectly, and the
tracker runs on 4648 of 4651 frames.

| estimator | total med | depth med | cross med | total p90 |
|---|---|---|---|---|
| IMM | 9.2 m | 8.9 m | 2.42 m | 10.6 m |
| triangulation | 9.2 m | 8.8 m | 2.40 m | 10.6 m |
| VSL's own output | 75.7 m | 75.7 m | **0.08 m** | 80.2 m |

Two things to read off this:

1. **IMM and triangulation agree to 0.1 m, and p90 ≈ median for both.** A tight
   error distribution is a *bias*, not noise. Filtering removes noise; it cannot
   remove an offset that every measurement agrees on. So on this window the
   estimator is provably not the limiting factor — it is already sitting exactly
   where the rays put it.
2. **VSL's own output has 0.08 m cross-LOS error and 75.7 m depth error.** Their
   *pointing* is essentially perfect; their *range* is broken. That localises
   their remaining error to the baseline/station geometry rather than to camera
   aim, which is a much cheaper thing to fix.

## 8. Boresight is a property of the SITE, not the window

The rig is a camera on a fixed ground mount, so its boresight cannot wander
between windows. Fitting per window is therefore not just wasteful — it is
actively misleading. A window in which the detector spends most of its time on
the wrong object admits no honest solution, and a per-window fit will invent a
large offset that aligns those wrong-object rays to truth.

Per-window fits at the **same site, same day, over four hours**:

| window | CAM1 dYaw | CAM2 dYaw | CAM2 support |
|---|---|---|---|
| 21 Jul 11:45 | −0.45° | **−65.12°** | 9 % |
| 21 Jul 12:02 | −0.60° | **+49.22°** | 9 % |
| 21 Jul 12:45 | −0.69° | +2.81° | 13 % |
| 21 Jul 13:01 | −1.00° | +1.93° | 76 % |
| 21 Jul 15:38 | −1.36° | +3.18° | 46 % |

CAM1 is stable to ±0.35°. CAM2 is stable to ±0.6° *on the windows with real
support*, and the two wild values both stand on **9 %** of frames. They were
never a pose fault; they were the fitter aligning wrong-object rays.

`fit_site()` pools every window at a site so the good windows outvote the bad,
and the nonsense disappears:

| site | heading entered | CAM1 dYaw | CAM2 dYaw | sigma | support |
|---|---|---|---|---|---|
| 21 Jul (5 windows) | 143 / 126 | **−1.00°** | **+1.97°** | 0.55–0.70° | 53 / 36 % |
| 23 Jul (2 windows) | 40 / 37 | **+43.19°** | **−6.08°** | 0.80–0.98° | 71 / 60 % |

### The 23 July offset is a typo, not a fault

+43° with 71 % support and σ 0.80° is real, and it is *not* the mount moving.
The mount is fixed; the **hand-entered `Gimbal_Heading` that the offset is
measured against is re-typed at every deployment**. The 23 July operator entered
40 when the true mount azimuth was ≈ 83. (For reference the 27 July config
carries 81 for CAM1.)

So the discriminator is support, not magnitude — `CamCalibration.suspect` now
only flags a large offset when under 40 % of frames back it, and
`heading_error_deg` reports the well-supported case as what it is: a wrong
heading constant, fixable by typing the right number.

**An earlier version of this document called these offsets a "pose/encoder
fault". That was wrong** — a static ground mount has no such failure mode, and
the numbers were an artefact of per-window fitting.

### 27 July confirms it: the mount is stable, the typed number is not

Target telemetry for 27 July arrived after the first pass, adding a third
deployment — at the **same physical site as 21 July** (~8 m between setup
points; absolute coordinates removed) but with the heading re-entered as 81/128 instead of
143/126.

Per-window CAM1 fits across 7.5 hours of that day:

| window | CAM1 dYaw | sigma | support |  | CAM2 dYaw | sigma | support |
|---|---|---|---|---|---|---|---|
| 07:33 | +37.03° | 6.41° | 52 % | | FAIL | — | — |
| 08:07 | +33.56° | 5.80° | 74 % | | +3.69° | 0.35° | 6 % |
| 09:20 | +39.79° | 9.89° | 62 % | | −8.07° | 8.42° | 52 % |
| 10:33 | +5.36° | 0.12° | 6 % | | −4.75° | 0.31° | 37 % |
| 11:42 | +39.61° | 0.78° | 33 % | | +55.61° | 28.30° | 60 % |
| 11:54 | +36.31° | 4.79° | 47 % | | −8.66° | 1.89° | 42 % |
| 13:03 | +35.30° | 5.43° | 44 % | | −8.63° | 1.85° | 41 % |
| 13:53 | +39.56° | 0.93° | 28 % | | −10.40° | 2.54° | 45 % |
| 14:49 | +42.59° | 1.05° | 81 % | | −2.38° | 0.37° | 21 % |

CAM1 sits at **+33.6 … +42.6°** all day (the lone +5.36° stands on 6 % of
frames), and the three best-conditioned windows give +39.6, +39.6, +42.6.
CAM2 clusters at **−8.1, −8.7, −8.6, −10.4°**. Stable across seven and a half
hours — a fixed mount behaving exactly like a fixed mount.

Putting all three deployments together:

| deployment | CAM1 entered → true | error | CAM2 entered → true | error |
|---|---|---|---|---|
| 21 Jul | 143 → 142.0 | **−1.0°** | 126 → 128.0 | **+2.0°** |
| 23 Jul | 40 → 83.2 | **+43.2°** | 37 → 30.9 | −6.1° |
| 27 Jul | 81 → 120.4 | **+39.4°** | 128 → 119.9 | −8.1° |

Two things stand out. **21 July is the only deployment where the heading was
entered correctly** — and it is also the only one the boresight table was
calibrated on (`calib_heading_deg` = 143/126). That closes the loop on §2: the
table is good because it was taken on the one day the heading was right, and it
is useless on the other two because the heading underneath it is wrong.

And CAM1's error is **+43.2° and +39.4°** on the two bad deployments — nearly
the same number twice, which does not look like a random typo. Something
systematic is being mis-read or mis-converted when CAM1's mount azimuth is
entered. That is worth chasing at the source: it is one number, entered by hand,
and getting it wrong costs the entire session.

### The +39 deg is adjudicated, not assumed

A large fitted offset deserves scepticism, so it was tested against the
alternatives before being believed:

* **Raw rays never point at the target.** With the logged heading, CAM1's
  ray-to-truth separation is **35.8–42.2 deg in all nine windows with 0 % of
  samples inside 2 deg**. Applying +39.4 deg brings it to 3.5–22.5 deg with up
  to 26 % inside 2 deg.
* **Not a swapped station.** Computing CAM1's rays from CAM2's station (which
  would fake a large constant bearing error) makes the residual *worse* in three
  of four windows.
* **Not their online self-calibration.** `apply=0` (shadow mode) and
  `apply_max_deg=1.5` — it is clamped an order of magnitude below the error.
* **Not truth leaking into their logs.** `MAV_Lat` is unpopulated in all three
  sessions (0 of 22 589 rows), so their `Target_*` really is camera-derived and
  the comparison is fair.

### Their self-calibration has been running on the 84 km baseline

The selfcal EKF records its own camera geometry in its config header:

| session | `cam2_enu` | implied baseline |
|---|---|---|
| 21 Jul flights | `-82.847, -57.438, 0.059` | **100.8 m** — correct |
| 23 Jul | `-84170.718, 389.030, -555.840` | **84 km** |
| 27 Jul | `-84170.718, 389.030, -555.840` | **84 km**, still, four days later |

Meanwhile the *triangulation* path on 27 July had the correct 92.6 m baseline
from `Gimbal_Lat/Lon`. So the two halves of the system were running on different
camera geometries, and the self-calibrator was structurally unable to work: only
**14.6 %** of its updates are accepted and its bias states sit pinned at the
±1.5 deg clamp (`dY1` and `dP2` both hitting ±1.5).

This is the most actionable defect found. The self-calibration that would
otherwise have caught the heading error has been silently disabled by a stale
camera position for at least five days — and because it runs in shadow mode,
nothing complained. **Initialise selfcal from the same live station config the
triangulator uses, and alarm when the two disagree.**

**Recommendation: measure the mount azimuth, do not type it.** Two surveyed
ground points per camera, or a single boresight shot at a target of known
position, would pin it to well under a degree and remove the largest error term
in the whole system.

### VSL reached the same conclusion independently

`scratch/fitted_bore_const4.json` records their own constant 4-DOF boresight fit
converging to **~1e-7 deg on all four axes** with a median error of 13.25 m —
i.e. there was no constant boresight left to find, and the residual error is
elsewhere. `scratch/fitted_bore_offsets.json` (a per-zoom spline fit) reaches
10.33 m median / 30.94 m p90. Both agree with §7: the remaining error is not
camera aim.

Two cautions on reusing those files: `boresight_rerun_results.json` carries
surveyed `cam_gps` for the 23 July site whose logged baseline is 3.3 % short
(92.62 m vs 95.70 m, worth 8.3 m of depth at R = 250 m) with the vertical
component sign-flipped — but its own tick stamps place it ~13 days earlier than
the 23 July flight, and substituting those coordinates makes that window
slightly *worse* (tri depth 16.3 → 17.7 m). It is a different deployment.

Per `README_MONO.md` §Latency Compensation, `frame_ts` is already pulled back by
the latency constants before logging, attitude by
`GIMBAL_TO_GUI − GIMBAL_TO_PHYSICAL` = 56 ms and zoom forward by
`ZOOM_TO_IMAGE − ZOOM_TO_GUI` = 308 ms. So the timestamps are **already
compensated**, and the 0.05–0.20 s this analysis fits is residual, not the full
image age — consistent with, not contradicting, their declared constants.

## 9. What to fix, in order

The dominant error term is systematic, so the ranking is calibration-first:

1. **Fix the entered `Gimbal_Heading` per deployment.** The mount is static, but
   the number typed in for it is not: the 23 July entry is off by **+43°** on
   CAM1 and **−6°** on CAM2 (§8). Survey the mount azimuth once per setup.
2. **Initialise selfcal from the live station config.** Its EKF has been running
   on the 84 km `cam2_enu` since 23 July while the triangulator used the correct
   92.6 m — the self-calibration that would have caught the heading error is
   silently dead, and shadow mode means nothing complained (§8).
3. **Bind the boresight table to its heading constant.** `calib_heading_deg` is
   already stored and never checked; checking it would have flagged 23 and
   27 July as uncalibrated automatically.
4. **Baseline sanity check at startup.** A 1° longitude typo gave an 84 km
   baseline and 15 km target altitudes for nine minutes.
5. **Chase the range, not the aim.** Their pointing is already sub-0.1 m
   cross-LOS (§7); the 75.7 m depth error is baseline/station geometry.
6. **Survey the station altitude.** `Gimbal_Alt = 0.0` for 89 % of 21 July rows,
   and it cannot be recovered from a fit (§4).
7. **Raise `WINDOW_TRI_SIGMA_PX` from 5 to ~30** before enabling the window
   triangulator, or it will be 5–35× overconfident.

On our side, two things are genuinely estimator work and worth doing:

* **The p90 tail** was ours (104 m vs triangulation's 40 m) and per-site pooled
  calibration mostly closed it (47.3 vs 44.7 m). What remains is re-acquisition
  after single-camera stretches: triangulation has no tail because it produces
  nothing on those frames, whereas the IMM produces something.
* **A second association cue** beyond skew, per the 15:38 window — both cameras
  can agree confidently on the wrong object.

## 10. What the estimator is actually fed (`vsl_noise.py`)

Measured on the 21 July deployment — the one where the entered heading was
correct — so none of this depends on the calibration argument.

### Magnitude, and how it scales

| camera | sigma yaw | sigma pitch | in pixels |
|---|---|---|---|
| CAM1 | 0.662° | 0.508° | 80.5 / 70.6 px |
| CAM2 | 0.518° | 0.459° | 78.8 / 56.6 px |

Neither the angular nor the pixel error is constant — they move in *opposite*
directions with zoom:

| zoom band | CAM1 sigma yaw (deg) | CAM1 sigma yaw (px) |
|---|---|---|
| 1–3 | 0.913 | 37 |
| 3–6 | 0.724 | 68 |
| 6–12 | 0.522 | 92 |
| 12–30 | 0.413 | 177 |

So the error is neither purely detector-limited (that would hold pixels
constant) nor purely gimbal/attitude-limited (that would hold degrees constant).
Angular error falls roughly as `zoom^-0.35`. **R must be a function of zoom**;
a single sigma is wrong by 2× across the operating range. A usable model:

    sigma_yaw_deg ~ 0.9 * (zoom / 2) ** -0.35        (CAM1, similar for CAM2)

Tails are close to Gaussian (p99 / 2.58·sigma = 1.06–1.39), so the NIS gate is
right to *inflate* rather than hard-reject.

### The finding that matters most: the residuals are not white

| lag | 0.05 s | 0.1 s | 0.2 s | 0.5 s | 1.0 s |
|---|---|---|---|---|---|
| CAM1 yaw | +0.95 | +0.88 | +0.83 | +0.79 | **+0.78** |
| CAM1 pitch | +0.97 | +0.95 | +0.92 | +0.84 | **+0.84** |
| CAM2 yaw | +0.89 | +0.85 | +0.81 | +0.77 | **+0.69** |

The angular error is still **~80 % correlated a full second later**. That is not
measurement noise; it is a slowly-varying bias — residual boresight moving with
gimbal angle and zoom, plus detector centroid bias on the target's changing
aspect.

The consequence is quantitative and severe. For an AR(1) sequence the
independent-information rate is `f·(1−rho)/(1+rho)`, which gives an effective
rate of **0.4 Hz for CAM1 and 1.1 Hz for CAM2**. Feeding the filter at 20 Hz
with a white-noise R therefore over-counts the available information by
**49× and 18×**. The filter is not slightly overconfident — it believes it has
fifty independent looks where it has one.

VSL had already found the same thing from the other side (`selfcal_ekf.py`:
"Inovasyonlar beyaz DEGIL; 9-10 Hz'de lag-1 otokor ~+0.92") and thin their
selfcal to 3 Hz. The measurement here says even 3 Hz is optimistic.

### Skew is necessary but NOT sufficient

Skew is the only quality signal available in flight, so how it maps to real
error decides whether it can be trusted:

| window | skew < 2 m → error | 2–5 m | 5–10 m | 10–30 m |
|---|---|---|---|---|
| 13:01 | **4.6 m** | 3.0 m | 6.2 m | 146.6 m |
| 12:02 | 65.5 m | 358.8 m | — | — |
| 11:45 | 232.3 m | 215.5 m | 132.8 m | 74.7 m |
| 15:38 | 157.3 m | 86.0 m | 124.3 m | 165.0 m |

In 13:01 skew works exactly as designed — a threshold near 10 m cleanly
separates 3–6 m errors from 150 m ones. **In the other three windows a skew
under 2 m coexists with 65–232 m of error.** Two rays agreeing with each other
says nothing about whether they agree on the *right object*, and that case is
common, not rare. Any guidance consumer gating solely on skew will be
confidently wrong for minutes at a time.

## 11. Prediction, R tuning, and the architecture call (`vsl_predict.py`)

Guidance consumes a prediction, not a fix, so everything below is scored as
"where does `p + v·tau + 0.5·a·tau²` say the target will be, versus where it
actually was". Held-out second half of 21 July 13:01, median total error [m]:

| config | 0 s | 0.25 s | 0.5 s | 0.75 s | 1.0 s | n |
|---|---|---|---|---|---|---|
| rays → IMM (current) | 9.1 | 9.5 | 10.0 | 10.5 | 11.1 | 1457 |
| triangulate → IMM | 7.9 | 8.0 | 8.3 | 8.6 | 8.8 | 1020 |
| triangulate → IMM, isotropic R | 7.5 | 7.7 | 7.8 | 8.2 | 8.7 | 1018 |
| baseline: hold last fix | 6.1 | 6.2 | 8.3 | 11.3 | 14.5 | 991 |
| **baseline: triangulate + constant velocity** | **6.1** | **6.7** | **7.2** | **7.7** | **8.2** | 990 |

**The dumbest method wins at every horizon.** Per-frame triangulation with a
constant-velocity extrapolation beats the six-mode IMM on rays by 26 % at 1 s,
and beats triangulate-into-IMM too. That is not a bug — it follows directly
from §10. When the measurement error is a slowly-varying bias rather than white
noise, there is no √N averaging to be had, so a filter's smoothing buys nothing
while its process model actively fights the bias.

The tuning sweeps say the same thing three ways:

| R scale | 1.0 s error | | feed rate | 1.0 s error |
|---|---|---|---|---|
| ×0.25 | 10.0 m | | native (~20 Hz) | 11.1 m |
| ×1 | 11.1 m | | 10 Hz | 10.1 m |
| ×4 | 12.2 m | | 5 Hz | 10.0 m |
| ×16 | 20.0 m | | 2 Hz | 10.2 m |
| ×64 | 22.4 m | | **1 Hz** | **8.2 m** |

*Trusting the measurement more is better* (smaller R wins), and *feeding it less
often is better* — at 1 Hz the filter converges to the constant-velocity
baseline's 8.2 m. Both are the signature of correlated error: the filter should
follow the biased-but-stable measurement rather than smooth it, and extra
samples carry no extra information.

### Recommendation

**Triangulate first, filter lightly, and keep rays only where they are unique.**

1. **Triangulate per frame** into a 3D point with the CRLB covariance. On this
   rig (b/R ≈ 0.4, ~22° parallax) depth is well observed per frame, so the
   anisotropy that motivated the rays design barely matters — isotropic R scored
   the same as anisotropic (8.7 vs 8.8 m).
2. **Feed at 1–5 Hz, not 20**, or inflate R by the correlation factor. Do not do
   both.
3. **Use a light CV/CA filter**, not the six-mode IMM, unless a maneuvering
   target demonstrably needs it — on this data the mode set is unpaid overhead.
4. **Keep the rays path as the single-camera fallback.** It is the one place it
   is strictly better: it produced estimates on 1457 frames against
   triangulation's 1020, covering the ~30 % of frames where only one camera
   holds a lock. That is a real capability, not a tie-breaker.

The 15:38 window was run as a check and cannot discriminate: it is dominated by
the wrong-object problem, so every method lands within 5 % of ~160 m (tri+CV
162.8, tri→IMM 163.9, rays 166.2 at 1 s). Same ordering, but no signal — when
the input is pointed at the wrong object, no architecture rescues it. The
recommendation therefore rests on 13:01.

Two further honest limits on this recommendation. The scored window is ~5 minutes of
relatively benign target motion, which is exactly the regime where constant
velocity is hardest to beat — an IMM earns its keep during maneuvers, and this
window has few. And "triangulate + CV" is not free of filtering: its velocity
comes from a 40-sample sliding window, which is itself a smoother. The
conclusion to draw is *"the current filtering is not paying for itself on this
data"*, not *"filtering is useless"*.

## 12. Bias vs scatter: how much of the error is removable, and by what

"`tri+CV` still carries a bias, so the true error is lower" — correct, but the
bias is the smaller half. Decomposed in the LOS frame (a boresight error is
angular, so it rotates with bearing and must be decomposed there, not in NED):

**tri+CV, 21 July 13:01 held out**

| tau | bias depth | scatter depth | raw \|err\| | debiased |
|---|---|---|---|---|
| 0.00 s | +3.32 m | 7.83 m | 6.70 | **5.55** |
| 0.50 s | +4.35 m | 9.67 m | 7.82 | **6.98** |
| 1.00 s | +4.89 m | 11.22 m | 8.91 | **8.52** |

Removing a perfect constant bias buys 17 % at tau=0 and only 4 % at 1 s. The
error is **scatter-dominated**, so calibration alone will not transform these
numbers.

### But what kind of scatter? Two different timescales

| lag | 0.05 s | 1 s | 2 s | 5 s | 10 s | 30 s |
|---|---|---|---|---|---|---|
| CAM1 yaw (angle) | +0.92 | +0.77 | +0.70 | +0.31 | −0.06 | +0.06 |
| CAM1 pitch (angle) | +0.94 | +0.82 | +0.78 | +0.49 | +0.23 | +0.26 |
| **triangulated depth error** | **+0.63** | **+0.33** | **+0.24** | **+0.07** | −0.06 | +0.06 |

The *angular* error wanders on a **~5 s** timescale — not a static bias, and not
white. But the *depth* error decorrelates in **~0.2 s**, far faster. That is the
geometry: common-mode angular wander (both cameras drifting together) pushes the
target sideways, not in depth; only the *differential* part drives depth, and the
two cameras wander independently.

Fast-decorrelating error is exactly what smoothing removes. Testing a sliding
constant-velocity fit on skew-gated fixes:

| fit window | tau=0 total/depth | tau=0.5 s | tau=1.0 s |
|---|---|---|---|
| hold fix (none) | 5.3 / 4.8 | 8.0 / 4.6 | 14.2 / 6.0 |
| 0.5 s | 5.4 / 4.7 | 6.0 / 5.4 | 7.9 / 6.8 |
| 1 s | 5.6 / 5.0 | 6.2 / 5.7 | 7.2 / 6.2 |
| 5 s | 5.8 / 5.4 | 6.8 / 5.8 | 7.8 / 6.1 |
| **10 s** | 5.5 / 4.5 | 6.2 / 5.2 | **6.6 / 5.0** |

Two things fall out. **Applying a skew < 10 m gate alone** took tau=0 from 6.7 m
to 5.3 m before any smoothing — the cheapest win available. And a **10 s CV fit
predicts 1 s ahead at 6.6 m**, against 8.2 m for the earlier 2 s-window baseline
and 11.1 m for the six-mode IMM on rays.

Depth, however, does **not** average down: 4.8 m unsmoothed, 4.5–5.4 m at every
window length. Per-frame depth sigma is 8.84 m, so ~70 % of its variance is fast
and does get averaged away, but ~30 % is correlated over seconds and leaves a
floor near sqrt(0.3)·8.84 ≈ 4.8 m — exactly what is observed.

### The full budget for depth

    8.8 m   per-frame triangulation
    4.8 m   after smoothing (the fast 70 % of the variance is gone)
   ~3.5 m   if the constant bias (3.3 m) were also calibrated out
            [sqrt(4.8^2 - 3.3^2)]

So calibration and smoothing are worth roughly the same amount, they are not
substitutes, and together they take depth from ~8.8 m to ~3.5 m. Beating 3.5 m
needs something else: a differential-boresight state estimated online (their
selfcal, once it is fed the right baseline), or an independent range channel.

### Revised recommendation

This supersedes the "filtering does not pay" reading in §11, which was measured
before the skew gate and with too short a smoothing window:

1. **Gate on skew (<10 m) before anything else** — biggest single win, free.
2. **Smooth over ~5–10 s with a constant-velocity fit**, not 0.5–2 s. The
   optimum is long because the fast noise is worth averaging and the target's
   motion is benign over that span; a maneuvering target will want shorter, so
   this window should adapt rather than be fixed.
3. **Filtering does pay** — a 10 s CV fit beats the per-frame fix by 2.2x at
   1 s (6.6 vs 14.2 m). What did not pay was the *six-mode IMM at 20 Hz with
   angular R*, which is a different claim.
4. Keep the rays path as the single-camera fallback, as in §11.

## 13. The 25minfirst session (30 July): close range, real maneuvers

New inland site (absolute coordinates removed), new log format (`mono_log.txt`, 270
columns: both rays, intrinsics, boresight state, VSL's Calc/LPF/Raw outputs per
event). 7,799 triangulation fixes at ~5–7 Hz across 8 hand-picked windows
(~25 min). Entered baseline 87.4 m, ranges 63–192 m — **b/R ≈ 0.5–1.4, the
geometry we asked for**. Truth from two dataflash bins (HAcc 0.58 m,
VAcc 0.81 m); MAVLink telemetry was off the whole session. And the maneuvering
request was honoured: p95 horizontal accel 7–14 m/s² (the old site never left
constant velocity). Tools: `mono_ingest.py`, `mono_eval.py`, `mono_survey.py`.

### 13.1 The dominant error is the site survey, not the cameras

The truth-vs-output offset is **+81 m UP** and ~+13 m E, constant across all 8
windows (MAD ≤ 7 m). The camera coordinates in the config are literally
`DEFAULT_GPS1/2` — sea-level defaults (1.83/1.43 m alt) at a site that
actually sits ~70–80 m AMSL. A blind survey solve (datum + CAM2 position +
per-camera boresight + per-camera clock, truth used offline only) finds:

* CAM2's entered position is **~7–8 m wrong**; the real baseline is ≈ 93–95 m,
  not 87.4 m. This is the source of the range-dependent 10–20 m horizontal
  wander (window 6 at 63 m range: only ~4 m error; 150+ m windows: 15–25 m).
* Residual boresight after their table: ±1.3–2.5° yaw. CAM1 again carries an
  **18° boresight yaw offset** absorbing a mount-heading entry error (§2's
  disease, third occurrence).
* Per-camera timestamps are wrong by **~+0.22 s (CAM1) / +0.10 s (CAM2)**,
  differential **+0.15 s** (`mono_timing.py --scan`: a 2-D grid refitting the
  whole site geometry at every point, minimum cost 1046 vs 1391 at zero, and
  bracketed on all sides). Sign convention: tau > 0 means the ray stamped t
  actually saw the target at t+tau, i.e. **the stamp sits EARLIER than the
  true capture instant — over-compensation, not under-**. (An earlier draft
  of this section had the sign backwards.) At the 10–25 °/s rates of these
  close passes, 0.2 s is 2–5°. Window 5 (hardest maneuver) is the worst
  window for every architecture *because of this*, not because of filter
  modes. **Only the differential is certain to be camera-side**: a common
  offset is degenerate with any lag in the target's own EKF output, and the
  two cannot be separated from this data.
* After the full fit the residual is 0.3–0.8° MAD — an **upper bound** on
  camera noise, since truth GPS alone contributes ~0.25° at these ranges.

### 13.2 What actually feeds the estimator

Per-frame **white** noise (first-difference MAD, per window, stable):

    CAM1  0.117° yaw   0.047° pitch
    CAM2  0.126° yaw   0.059° pitch

Pitch sits at the detector floor (5 px ≈ 0.04–0.06°); yaw carries ~2× that.
Same class as the old site — the sensor did not get worse at close range, the
*errors that matter* just changed owner (timing + survey instead of boresight
drift). Residuals are still ρ ≈ 1.0 at native rate; skew median 5.2 m,
p90 9.2 m, 6.5 % > 10 m — the gate stays.

### 13.3 Median prediction errors (datum-corrected, pooled 8 windows)

Only the constant per-flight datum is removed (a survey error, not a sensor
one); baseline error, boresight residual and timing stay in — this is
as-flown performance. Median total [m] at tau = 0 / 1.0 s:

| config | tau=0 | tau=1 s | depth 0/1 s |
|---|---|---|---|
| VSL Calc (as logged) | 14.4 | 19.4 | 8.4 / 14.0 |
| VSL LPF (as logged) | 14.8 | **22.6** | 9.6 / 16.3 |
| hold gated fix | 13.8 | 18.5 | 7.7 / 12.9 |
| **CV fit 2 s** | 13.3 | **12.2** | 7.5 / 7.4 |
| CV fit 5 s | **12.1** | 12.8 | 7.2 / 7.8 |
| CV fit 10 s | 14.8 | 19.6 | 9.2 / 11.1 |
| rays-IMM (six-mode, native) | 14.4 | 15.5 | 9.6 / 12.1 |
| tri-IMM CRLB R | 14.1 | 14.3 | 8.8 / 10.0 |
| tri-IMM iso R | 13.1 | 14.4 | 8.6 / 9.3 |
| tri-IMM skew<10 | 13.5 | 13.5 | 8.0 / 9.1 |
| tri-IMM skew<10, R×0.25 | 13.6 | 12.8 | — |
| CV 3 s on VSL's own Calc | 13.3 | 12.6 | — |

* **Their LPF is worse than their raw Calc at every horizon** — it adds lag
  and pays back nothing. Tell them to drop it or shorten it drastically.
* The smoothing optimum moved from 10 s (§12, benign target) to **2–5 s**:
  with real maneuvers, 10 s over-smooths (19.6 m at 1 s). The window must
  adapt to target dynamics; 2–3 s is the safe default.
* `CV 3 s on Calc` = 12.6 m at 1 s is implementable in their hub *today*
  with no architecture change.
* R sweep on tri-IMM: ×0.25 best at 1 s (12.75), ×1 fine, ×16+ ruinous. With
  a skew gate in front, trust the measurement more, not less.

### 13.4 The architecture answer, second dataset in a row

Rays-in vs position-in changes the result by ~10 %, again. Six IMM modes buy
nothing even with genuine maneuvers — the dominant error is quasi-static
(survey + timing), which no process-model bank can remove. Anisotropic CRLB R
vs isotropic R: **iso was marginally better**, because at b/R ≈ 0.6 the
anisotropy is mild and the CRLB's white-angle assumption overstates depth
confidence when the real error is correlated wander. The decision stays:
**triangulate first, feed positions to a simple short-memory smoother/filter,
keep rays only as the single-camera fallback.**

### 13.5 What to fix, ranked by expected return

1. **Survey the mounts** (RTK/PPK, once per deployment): removes the 80 m
   absolute datum — which would hand the pursuer coordinates 80 m below the
   target — and the ~8 m baseline error behind most of the 13 m quasi-static
   error. Zero software.
2. **Fix per-camera latency** (~170/50 ms residual): dominates every fast
   pass. Their README claims compensation; the constants need re-measuring
   per camera.
3. **Re-run boresight after 1–2** — its ±1.5–2.5° residual is partly
   absorbing the survey and timing errors.
4. **Guidance feed**: skew-gated fixes → 2–3 s CV fit (or tri-IMM, R×0.25).
   Retire the LPF.
5. Per-frame variances over wireless should carry the *white* part
   (≈0.12°/0.05°); no R can whiten the correlated part — thin or model it.

Post-survey bound: ~0.4° residual at 150 m ≈ 1–2 m cross, 2–3 m depth per
frame; smoothed, a 1 s prediction of **3–5 m** looks reachable against
12.2–12.8 m as-flown. The gap is all geometry and timing, none of it sensor.

### 13.6 The live run (Shakyh's logs) and the "~2.5 m error"

`VSL/Shakyh/logs/filteroutlatest.csv` is the guidance-side IMM running live
during this same session (19,924 rows, 2368 s — the whole 25 min). Feed
identification by per-candidate time alignment: the redis key
(`dogru_rakip_telemetri`) carried **VSL's Calc** — it matches the mono log's
Calc series to **0.40 m median** (LPF 0.90, Raw 94) — at ~6.6 Hz median
cadence. The guidance correctly consumed only the estimate; no true telemetry
reached it.

The ~2.5 m figure is real and reproducible from that log — but it is scored
against the *feed*, not the target:

| metric | median | p90 |
|---|---|---|
| innovation (est before update vs next measurement) | 2.22 m | 6.6 m |
| lead/slot point vs next measurement | **2.49 m** | 23 m |
| est after update vs measurement | 0.50 m | 1.6 m |
| **est vs bin truth, all constant offsets fitted away** | **17.7 m** | 49 m |
| est vs bin truth, raw site frame | ~80+ m | — |

Both are true simultaneously because the feed's error is correlated over
seconds (§13.2, ρ≈1): consecutive measurements agree with each other while
being wrong *together*, so predicting the next measurement is easy and being
right about the target is not. An innovation-style score can never see the
common-mode part — this is the measurement-vs-truth distinction of §12 played
out on live hardware.

Why 17.7 m live vs 13.8 m offline hold-Calc: **staleness**. The absolute
transport delay is *not directly measurable* from these logs — `filterout`
stamps `time.monotonic()`, and the only independent clock anchor (pursuer
telemetry) is unusable because `pursuer_*` is identically zero for the entire
run: no leader state was connected. (An earlier draft quoted "0.85 s" from a
cross-correlation of two soft alignments; that method cannot separate
transport delay from content correspondence and the number is withdrawn.)
What can be said:

* `Loc_Age_ms` is logged by VSL itself: **median 396 ms**, p10 365, p90 436,
  max 4.4 s — capture to localisation output, before any wireless hop.
* The live IMM tracks its feed to 0.50 m, so it behaves as a hold of that
  feed; reading 17.7 m off the hold-fix error-growth curve (13.8 m at tau=0,
  18.5 m at tau=1.0) puts the total consumption age at roughly **0.6–0.9 s**.
  That is an inference from the growth curve, not a measurement.
* The consumer adds **no age term at all**. `meas_age` is computed
  (line 1485) and used only for the staleness gate; the aim point is
  `x(t_meas + t_go)` where it must be `x(t_meas + age + t_go)`. With no
  leader connected, `t_go` was unavailable and the horizon fell back to the
  fixed `--predict` constant for the whole run — which is exactly what the
  `pred_0.25` / `pred_0.5` filenames were varying.
* Between packets the filter is not propagated at all (predict happens only
  on arrival), adding up to one packet interval (~0.15 s) of extra lag.

### 13.7 "But we use RTK" — reconciling the survey finding

RTK gives centimetre accuracy *relative to the base station*; the absolute
frame is whatever the base's declared position was. Three observations say the
triangulator did not run on RTK coordinates this session:

1. The in-use camera coordinates are the config **defaults** and never change
   across 5 h of log (`C1/C2_GPS_*` constant; alt 1.83/1.43 m).
2. The drones' GPS frame puts the site ground >100 m AMSL (pursuer on ground:
   `alt_amsl` 112.0 with `rel_alt` 0.56; target truth VAcc 0.81 m). No RTK
   output reports 1.83 m there — that is a typed default or a survey anchored
   to an arbitrary local datum ("base at 0 m").
3. The blind fit *demands* ~8 m of CAM2 correction along the baseline
   (87.4 m entered → ~95 m solved): freezing e2 = 0 doubles CAM1's yaw
   residual (0.82° → 1.76°) and raises the robust cost 41 %. An 8 m relative
   error cannot come out of a single-session RTK survey — so either the RTK
   values never reached the config (position source configured `dynamic` but
   the logged values are static defaults), or the two mounts were surveyed
   against different base declarations.

Instant arbitration available to VSL: compare their RTK-surveyed baseline
length against 87.4 m (entered) and ~95 m (solved). Whichever it matches
settles who is wrong, no truth required.

**Resolved 2026-07-31: the operator confirmed camera positions are NOT
RTK-surveyed** — RTK flies on the drones only; the mount coordinates are
hand-entered. So the fitted geometry stands as-is: ~8 m baseline error
(~9 % — and triangulated *range scales linearly with baseline*, so every
range this site produced was ~9 % short on top of everything else) and the
+81 m altitude datum. The cheap fix needs no new hardware: **set one of the
RTK drones on each camera mount for a minute and average its position** —
that surveys the mounts in the *same GPS frame the target and pursuer
navigate in*, which is the frame consistency that actually matters. Re-enter
those coordinates, re-run boresight, done.

### 13.8 Timing, elaborated: two independent defects, two different fixes

Timing enters twice, and conflating them is why "fix the latency" is not one
task. `mono_timing.py` measures both.

**(a) Differential — the two rays refer to different instants.** Median sync
spread `Spread_ms` is 65 ms (p90 122 ms) of *random* offset, on top of the
**+0.15 s systematic** differential of §13.1. Simulated from truth with
perfect rays and perfect calibration, so timing is the only defect:

| defect | total | depth | **induced skew** |
|---|---|---|---|
| control (correct stamps) | 0.00 m | 0.00 m | 0.00 m |
| common-mode 0.20 s | 2.81 m | 1.35 m | 0.00 m |
| fitted 0.22 / 0.10 | 3.58 m | 2.54 m | 0.47 m |
| differential only (0.15 s) | 3.36 m | 3.07 m | 0.47 m |
| random jitter, sigma 65 ms | 1.74 m | 1.58 m | 0.19 m |

The last column is the important one. A 0.15 s desync costs **3.4 m** while
producing **0.47 m of skew** — an order of magnitude below the 10 m gate. **A
triangulator cannot detect its own time-skew from ray geometry**: the two
rays still nearly intersect, just at the wrong point. Every quality metric we
have (skew, chi2, condition number) is blind to it, which is why it survived
this long. It is also *not* removable downstream — no filter can undo two
observations of different instants presented as one.

*Fix at the source:* interpolate each camera's own angle series to a common
epoch before triangulating. Each camera runs ~7 Hz, the gap to bridge is
≤70 ms, and target angular motion is smooth over that, so linear
interpolation is ample. VSL already has the machinery (`SYNC_HISTORY_BUFFER`,
the `WINDOW_TRI` polynomial fit). Better still, hardware-trigger both cameras.

**(b) Common-mode — everything is old by the time guidance uses it.** This
one *is* removable downstream, but only if the stamp means what it claims.
Currently it does not (§13.1: ~0.2 s early), so `now − ctrl_ts` is wrong even
before transport. Fix in two parts: define `ctrl_ts` as **the instant the
photons hit the sensor** (measured, per camera, not a config constant), then
have guidance predict forward by `(now − ctrl_ts) + t_go` instead of `t_go`.

### 13.9 Why fixing timing looks worthless today — and is not

Applying the common-epoch fix to the real data barely moves the median:

| policy | total | p90 | depth |
|---|---|---|---|
| as-logged (what VSL does) | 15.20 m | 45.6 | 8.46 |
| interpolate to common epoch | 14.78 m | 44.4 | 8.68 |
| + per-camera stamp shift | 14.56 m | 41.8 | 8.95 |

0.6 m of median for all that work. But errors add in **quadrature**:
`sqrt(15.20² − 3.4²) = 14.8` — almost exactly the 14.78 measured, so the
timing term is fully accounted for and simply hides behind the ~13 m survey
error. Fix the survey first and the total drops toward 4–5 m, at which point
the same 3.4 m timing term is **the dominant remaining error**. This is the
whole justification for the ordering in §13.5: survey, then timing, then
boresight. Doing timing first would look like a wasted field day.

**Negative result worth recording:** the rays architecture *ought* to handle
desync for free — each detection carries its own stamp and the filter can
predict between them. Feeding each camera's detection as its own event at its
own timestamp made it **worse** (16.30 → 17.83 m median total, depth
10.49 → 12.91). Splitting one well-conditioned two-ray update into two weakly
constraining single-bearing updates costs more than the timing gain,
especially with an R that only describes the white noise. So the fix is
source-side interpolation, **not** an architecture switch — and our own
`StereoTracker.process()` treats a frame as simultaneous regardless, so it
does not currently exploit per-ray timestamps either.

## 15. Failure-mode audit of our own tracker (`test_robustness.py`)

Probed before wiring to guidance. Behaviour, not opinion — the script prints
all of this.

**Defences that work as intended**

| case | behaviour |
|---|---|
| single-frame outlier ≥10° | hard-rejected (NIS > 400); state moves no more than on a clean frame |
| outlier 0.5–2° | R inflated, not dropped — deliberate, so a real maneuver onset is not stonewalled |
| one camera absent | degrades to mono: track held 6 s, error 1.0 → 4.9 m, and **error stays below sigma** (honest covariance) |
| both cameras absent | COAST 2 s → LOST; `x` becomes `None` and `tracking` False, so a consumer cannot silently use a stale state |
| ray skew | inflates R above 3 m, drops the frame above 60 m |
| rank-deficient geometry | `pinv` plus an explicit 10⁴ m variance restored on the unobservable direction, rather than pinv's zero-variance lie |
| non-finite state mid-update | detected, mode left un-updated (its pre-update `kf.x` is kept) |

**Gaps found**

1. **A faulty camera is more dangerous than a dead one.** A non-finite angle
   is *not* caught at the gate — `NaN > chi2` is False, so the scalar is
   accepted — and the failure only surfaces later, when every mode returns
   `-inf` and the **whole frame is discarded, including the healthy camera's
   two good scalars**. Sustained, that coasts to LOST in 2.6 s. The same
   camera merely *absent* holds the track indefinitely at ~2 m. Fix: reject
   non-finite scalars per-scalar in `build_measurement_plan`, so the frame
   degrades to mono instead of dying.
2. **A slow plausible bias is followed, not rejected.** 3° on one camera for
   3 s → 57.7 m error with sigma 1.3 m: **err/sigma ≈ 46**, i.e. confidently
   wrong, and the velocity state collapsed from 20.6 to 9.7 m/s. Gating is
   against the *prior*, so once the filter has moved, subsequent measurements
   agree with it. This is the synthetic twin of the real-data finding that
   both cameras can agree on the wrong object (§7), and of time-skew being
   invisible to skew (§13.8). No innovation-based test can catch it.
3. **Stamps are not checked for monotonicity.** A 5 s-old packet is applied as
   current (`dt` clamps to 1e-3) *and* rewinds `last_stamp`, so the next live
   frame sees a +5 s `dt`. A duplicate packet replayed 10× is counted as ten
   independent measurements and shrinks sigma accordingly. The guidance layer
   happens to check `ctrl_ts > last_ctrl_ts`, but the tracker must not depend
   on its caller for this.
4. **`COAST_TIMEOUT_S` is measured in measurement time.** A sender whose clock
   freezes while still publishing keeps the tracker in TRACK forever
   (100 frames, `coast_frames` 0). Wall-clock liveness is the caller's job
   today — and the caller only checks `target_timeout` on arrival, not on
   stamp advance.
5. **No FOV or max-range validation** on incoming detections, though the
   config carries both.

**The theme:** every defence we have is a *consistency* check — innovation vs
prior, ray vs ray. All of them pass when the input is self-consistent and
wrong, which is precisely the failure mode the real data keeps producing
(wrong object, boresight error, time-skew, survey error). Independent
corroboration is what's missing: bbox angular size vs predicted range, track
continuity, or agreement with a second sensor.

## 16. Lowest survivable data rate

Two different floors, and the binding one was not the informational one.

**Accuracy floor — much lower than expected.** On the real 25minfirst rays
(native 7.5 Hz), decimating the feed barely matters:

| feed | err @0 | @0.5 s | @1 s |
|---|---|---|---|
| native 7.5 Hz | 13.87 | 13.73 | 13.44 |
| 3 Hz | 13.96 | 13.86 | 13.77 |
| **1 Hz** | 14.09 | 14.13 | **14.10** |
| 0.5 Hz | 14.20 | 14.47 | 14.62 |
| 0.25 Hz | 14.06 | 16.16 | 21.25 |

Dropping 7.5 Hz → 1 Hz costs **5 %**. That is §10's correlated-noise result
seen from the other side: the effective independent rate of this sensor is
~1 Hz, so frames beyond that carry almost no new information. On *synthetic*
white noise the same sweep rewards high rates (2.1 m at 20 Hz vs 3.9 m at
1 Hz) — a good reminder that a synthetic bench will always overvalue frame
rate.

**Structural floor — this is what actually binds.** It was 1 Hz, silently,
and is now 0.5 Hz honestly:

* `MAX_PREDICT_DT_S` was **1.0 s while `COAST_TIMEOUT_S` is 2.0 s**. Any gap
  between those was propagated for less time than actually elapsed, so the
  filter had to explain the measurement's extra displacement by inflating
  velocity. Measured at 0.5 Hz: position error looked *fine* (2.5 m) while the
  1 s prediction was **18.5 m** out. A 3 s gap advanced the state 33 % of the
  distance it should have. Invisible in the position residual, which is what
  makes it dangerous.
* Fixed by raising `MAX_PREDICT_DT_S` to `COAST_TIMEOUT_S` and refusing gaps
  beyond that outright (→ LOST) instead of clamping. A 2 s gap now propagates
  98.6 %; a 3 s gap propagates nothing and says so. At 0.5 Hz the 1 s
  prediction went **18.5 → 4.7 m**.

**Answer to give guidance:** ≥1 Hz for full performance, 0.5 Hz absolute
floor, below that the track drops and re-acquires (~20 % availability at
0.33 Hz). Since the useful rate is ~1 Hz, **link budget is not the
constraint — latency is** (§13.8b): a 1 Hz feed that is fresh beats a 7 Hz
feed that is 0.5 s stale.

## 17. Defences added 2026-07-31 (`test_robustness.py` asserts all of these)

| gap (§15) | fix | verified |
|---|---|---|
| non-finite angle discards the frame | reject non-finite detections in `process()`, plus an explicit non-finite check in `build_measurement_plan` (the chi-square gate cannot catch NaN — every comparison is False) | faulty camera now degrades to mono: 2.1 m vs 2.0 m for the healthy-mono control, where before it went LOST in 2.6 s |
| stamps not checked | `REJECT_NONMONOTONIC_STAMPS`: a stamp that does not advance is refused, so it neither updates the state nor rewinds the clock | 5 s-old packet rejected (`stale-stamp`), clock unmoved; 9 of 10 duplicates rejected |
| frozen sender clock | `STALE_FRAME_LIMIT` (20) consecutive non-advancing stamps → LOST. Deterministic, no wall clock, so replay stays reproducible | 100 frozen frames → LOST |
| gap silently clamped | refuse gaps > `COAST_TIMEOUT_S`; `MAX_PREDICT_DT_S` raised to match | 98.6 % propagation at 2 s, refusal at 3 s |

New snapshot fields for the consumer: `stale_frames`, `rejected_frames`,
`rejected_dets`.

**These are not hypothetical defects — the VSL feed contains them.** Across
the 7,799 good-window frames: **15 non-advancing stamps**, **19 gaps > 1 s**
(silently clamped before, so velocity was corrupted at each) and **9 gaps
> 2 s** (now refused rather than clamped). No non-finite angles in this
dataset, so that guard is insurance rather than a live fix.

**No regression.** A/B of the rays path on windows 3+7 (n=4479), identical
except for the four guards:

| config | 0 s | 0.25 s | 0.5 s | 0.75 s | 1 s |
|---|---|---|---|---|---|
| pre-fix | 16.79 | 16.81 | 16.67 | 16.38 | 16.14 |
| post-fix | 16.75 | 16.71 | 16.68 | 16.38 | 16.10 |
| delta | −0.04 | −0.09 | +0.01 | +0.00 | −0.04 |

(Absolute values are higher than §13.3 because this A/B pins sigma at
0.12/0.13° rather than the fitted per-camera values; both arms share that, so
the comparison is valid.) The guards cost nothing in the normal case and only
act on the 0.4 % of frames that are actually defective.

**Still open, deliberately:** the sustained plausible bias (§15.2). 3° on one
camera for 3 s still reaches 57.7 m error at 1.3 m sigma. No innovation-based
test can catch it, because after the filter follows the bias every subsequent
measurement agrees. It needs independent corroboration — bbox angular size vs
predicted range, or track continuity — which is a design decision, not a gate.

## 18. Pre-entering a measured boresight

Yes, and it is worth more than the §12 "bias is the smaller half" reading
suggested — that was measured on a rig whose survey was roughly right. Here
the survey is not, and the bias is doing much heavier lifting.

Scored on the 30 July data. A constant *offset* is a datum error that only
surveying fixes, so the best constant is removed from **every** row; what is
left is the part that varies with geometry, which is what a boresight can
address:

| correction | median | p90 | R<130 | 130–170 | R>170 | range spread |
|---|---|---|---|---|---|---|
| none (as logged) | 15.06 | 43.7 | 11.27 | 11.89 | 24.86 | 13.60 |
| **bias only, survey untouched** | **6.74** | 27.1 | 4.43 | 5.79 | 11.24 | 6.81 |
| joint bias + CAM2 position | **5.67** | 13.9 | 4.89 | 4.86 | 7.31 | **2.46** |
| survey only, no bias entered | 9.74 | 34.4 | 6.08 | 9.01 | 17.76 | 11.69 |

**The trap, and it is the same one VSL fell into.** A constant angular offset
and an error in camera *position* are not separable from one deployment's
residuals, and the fitter pours one into the other. Fitting with CAM2 frozen
at its wrong entered position gives CAM1 yaw **−2.08°**; fitting with the
position free gives **−0.45°**. So ~1.6° of that "boresight" is an 8 m survey
error wearing an angular costume. It cancels at the range it was fitted at
and drifts elsewhere — see the range-band columns, and the spread column
(6.81 m bias-only vs 2.46 m once the position is also corrected).

So: **enter it, but record the camera positions it was fitted against, and
invalidate it when they change.** Re-survey the mounts and the entered offset
becomes actively wrong. This is precisely `boresight_offsets.json` storing
`calib_heading_deg` and never checking it (§2) — we should not reimplement
their bug with different variable names.

Note also that fitting requires a truth reference: it needs a calibration
pass with a GPS-logging target. It cannot be derived from camera data alone,
because the thing being estimated is exactly what the cameras cannot see
about themselves.

**Plumbing fixed to make this work.** The correction existed
(`Camera.bias_yaw/bias_pitch`) but was applied in only half the pipeline:
`build_measurement_plan` subtracted it for the IMM update, while
`triangulate_midpoint` / `triangulate_ml` / `triangulation_covariance` took
raw `det.yaw`. An entered offset therefore corrected the filter update while
leaving the init seed, the skew gate and `geom["fix"]` biased. It is now
subtracted **once at ingest** (`StereoTracker._debias`), so every consumer
sees the same corrected angles, and the duplicate subtractions downstream are
removed. `BoresightEstimator` now keeps the entered value as `base` and
estimates an increment on top, instead of overwriting it with zero on the
first frame.

Verified end-to-end with a 1.5°/−0.8° injected misalignment: position error
26.99 → **0.31 m**, matching the clean control exactly, and ray skew
3.65 → **0.29 m** — the skew collapse is what proves the geometry path now
honours the correction. Asserted in `test_robustness.py` §9.

This does **not** address the sustained-outlier gap (§15.2), which is a
transient mis-association rather than a static offset.

## 14. Caveats

* Only 2 of 7 gated windows have a credible calibration, and one of those
  (15:38) is tracking the wrong object. The headline table therefore rests on a
  **single** window (13:01). Treat it as one well-characterised sample.
* The 12:02 structural argument (§7) stands on a suspect calibration; the
  *relative* result (IMM ≡ triangulation, p90 ≈ median) is robust to that, the
  absolute 8.9 m is not.
* The 27 July session has no target telemetry — characterised, never scored.
* `dAlt` is reported but not identifiable (§4).
* Truth is the target's own EKF solution and carries its own metre-level error.
* §13's datum correction is fitted from truth (offline only); everything
  scored sits on top of it. The survey decomposition (baseline vs boresight vs
  clock) has correlated parameters — magnitudes are robust, the exact split
  between them less so.
* §13 scores use `Loc_TS` as delivered; with ~0.4 s `Loc_Age` and the residual
  under-compensation of 13.1, real-time guidance operates ~0.5 s behind the
  pixels, so the practical horizon is tau ≈ 0.5–1.5 s — where the CV-fit
  advantage is largest.
