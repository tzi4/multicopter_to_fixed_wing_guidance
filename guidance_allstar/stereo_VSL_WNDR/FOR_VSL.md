# Stereo rig — findings and fixes from the 30 July session

Analysis of `mono_log.txt` + the two target dataflash logs from the 25-minute
session (7,799 triangulation events over your 8 hand-picked windows, ranges
63–192 m, entered baseline 87.4 m). Scored against the target's own flight
controller log (0.58 m horizontal / 0.81 m vertical accuracy, 15 sats).

**Headline: the cameras are fine. The errors are in the geometry we hand them
and in the timestamps — both fixable without touching the vision pipeline.**

Measured per-frame angular noise, first-difference so slow drift can't inflate
it:

| | yaw | pitch |
|---|---|---|
| CAM1 | 0.117° | 0.047° |
| CAM2 | 0.126° | 0.059° |

Pitch is at the detector's pixel floor. That is a good sensor. Yet the
delivered position error is 13–15 m median. Everything below is about closing
that gap.

---

## 1. Survey the camera mounts — biggest single win, no software

**What we see.** `C1_GPS_*` and `C2_GPS_*` never change across five hours of
log, and match `DEFAULT_GPS1/2` in the config — including altitudes of 1.83 m
and 1.43 m. The site is inland, and your own drones sitting on the ground there
report ~112 m AMSL.

Consequences, both measured:

- Every position you output is **~81 m below and ~13 m east** of where the
  target actually is. Constant across all 8 windows (MAD ≤ 7 m), i.e. a pure
  datum error. If those coordinates are ever handed to an interceptor, it is
  aimed 80 m under the target.
- A blind geometry fit says CAM2's entered position is **~8 m off**, making the
  real baseline **≈95 m, not 87.4 m**. This is not a fitting artifact: freeze
  that correction at zero and CAM1's angular residual doubles (0.82° → 1.76°)
  and the robust cost rises 41 %. Triangulated **range scales linearly with
  baseline**, so every range this site produced was ~9 % short on top of
  everything else.

**Fix.** You already have RTK on the aircraft. Park one on each camera mount
for a minute and average its position, then enter those coordinates. That
surveys the mounts in the *same GPS frame the drones navigate in*, which is the
frame consistency that actually matters — chasing absolute WGS-84 correctness
is beside the point.

**Check it yourself, no external data needed:** compare your surveyed baseline
length against the 87.4 m currently entered. And compare the configured mount
altitude against what your own drones report on the ground at that site.

---

## 2. Timestamps — two separate defects, two different fixes

### 2a. The two cameras' rays refer to different instants

`Spread_ms` is 65 ms median (p90 122 ms) of random offset, and a geometry fit
finds a **systematic 0.15 s differential** on top (CAM1's stamp ≈ +0.22 s,
CAM2's ≈ +0.10 s relative to true capture).

Simulated with perfect rays and perfect calibration, so time-skew is the only
defect present:

| defect | position error | depth | **skew it induces** |
|---|---|---|---|
| correct stamps | 0.00 m | 0.00 m | 0.00 m |
| 0.15 s differential | **3.36 m** | 3.07 m | **0.47 m** |
| random jitter σ 65 ms | 1.74 m | 1.58 m | 0.19 m |

**Read the last column carefully.** A desync that costs 3.4 m produces less
than half a metre of ray skew. Your quality gates — skew, chi², condition
number — are all *structurally blind* to it, because the two rays still nearly
intersect; they just intersect at the wrong point. That is why this has
survived. It is also not removable downstream: no filter can undo two
observations of different instants presented as one.

**Fix at the source:** interpolate each camera's own angle series to a common
epoch before triangulating. Each camera runs ~7 Hz, the gap to bridge is
≤70 ms, and the target's angular motion is smooth over that, so linear
interpolation is ample — you already have `SYNC_HISTORY_BUFFER` and the
`WINDOW_TRI` polynomial fit for this. Hardware-triggering both cameras is the
better long-term answer.

### 2b. Everything is old by the time guidance uses it

`Loc_Age_ms` is 396 ms median (p90 436, max 4374) before any wireless hop. And
the stamps themselves are **over-compensated by ~0.2 s** — they sit *earlier*
than the true capture instant, so the data is fresher than its timestamp
claims, and `now − ctrl_ts` is wrong even before transport.

**Fix, two parts:** (i) define the published timestamp as *the instant the
photons hit the sensor*, measured per camera rather than taken from config
constants; (ii) publish the age alongside it so the consumer can predict
forward by `(now − capture_ts) + t_go` rather than a fixed constant.

**Check it yourself:** instrument the capture path and compare a hardware
capture stamp against what the pipeline publishes. This is a plumbing
measurement, not a data fit — it needs no truth reference.

---

## 3. Re-run the boresight calibration — but only *after* 1 and 2

The residual boresight after your current table is ±1.3–2.5°, and CAM1 again
carries an ~18° offset that is really a wrong hand-entered mount heading being
absorbed. Some of that residual is currently soaking up the survey and timing
errors above, so recalibrating first would bake those in.

**We measured how much.** A constant angular offset and an error in camera
*position* are not separable from one deployment's residuals — the fitter
pours one into the other. Fitting with CAM2 frozen at its currently entered
position gives CAM1 yaw **−2.08°**; fitting with the position free gives
**−0.45°**. So ~1.6° of that "boresight" is the 8 m survey error wearing an
angular costume. The giveaway is that it cancels at the range it was fitted at
and drifts elsewhere — error by range band was 4.4 / 5.8 / 11.2 m for the
bias-only fit, versus 4.9 / 4.9 / 7.3 m once the position was also corrected.

Worth doing though: a fitted constant cut the geometry-dependent error from
**15.1 m to 6.7 m** even with the survey untouched, and to **5.7 m** with it.

**Calibrate against a hovering target, not a flying one.** You do not need a
dynamic pass. Hover an RTK aircraft at 4–6 known points and log both cameras
staring at it. A static target has **zero timing sensitivity**, so you get a
clean boresight instead of one entangled with the ~0.15 s desync of §2a — the
two defects stop contaminating each other. Spread the points in range *and*
bearing (near/mid/far, both sides, varied elevation): that geometric diversity
is what separates boresight from residual position error. Calibrate at a
single range and the fit will look excellent and generalise badly.

Related, and worth fixing properly: `boresight_offsets.json` records
`calib_heading_deg` but nothing ever checks it. Re-typing `Gimbal_Heading` on a
later deployment silently invalidates the whole table. Compare on load and
refuse or warn. Same for the mount coordinates — store them **in the same file
as the offsets** and refuse to use one against the other, because as shown
above the offsets are only valid for the positions they were fitted against.

---

## 4. Retire the LPF — it is measurably worse than your raw output

Median error against truth, in metres, "now" and predicting 1 s ahead. The
constant datum offset from §1 is removed for all rows so this compares
filtering only:

| output | now | +1 s |
|---|---|---|
| your `Calc` (instant triangulation) | 14.4 | 19.4 |
| your `LPF` | 14.8 | **22.6** |
| sliding 2 s constant-velocity fit on skew-gated fixes | 13.3 | **12.2** |
| 3 s CV fit **on your own `Calc` output** | 13.3 | 12.6 |

The LPF adds lag and pays nothing back — it is worse than the raw `Calc` at
every horizon we tested. A 2–3 s sliding constant-velocity fit is better by
7 m at a 1 s horizon and is implementable in the hub today with no other
change.

Two notes on that: gate on skew < 10 m first (6.5 % of frames fail it, and
they're the bad ones), and don't make the window longer than ~3 s — with a
genuinely maneuvering target 10 s over-smooths badly (19.6 m at 1 s).

---

## 5. Per-frame variances over the link

Send the **white** part — what we measure is ~0.12° yaw and ~0.05° pitch. But
be aware a per-frame variance cannot express the larger problem: the residuals
are ~100 % correlated frame-to-frame and wander on a ~5 s timescale. A filter
fed at full rate with white-noise variances believes it has far more
independent information than it does. If you can also send a correlation
timescale, that is more valuable than the variance itself.

---

## Why the ordering matters

If you do §2 first it will look like a wasted day. We applied the common-epoch
fix to the real data and the median barely moved: 15.20 → 14.56 m. But errors
add in quadrature — √(15.20² − 3.4²) = 14.8, almost exactly what we measured.
The timing error is fully accounted for; it is simply hiding behind the much
larger survey error. **Fix the survey first**, the total drops toward 4–5 m,
and that same 3.4 m timing term becomes the dominant remaining error.

Post-fix, the residuals support roughly **3–5 m at a 1 s prediction horizon**,
against 12–14 m as flown. None of that gap is sensor noise.

---

## Field checklist

The whole thing as a running order. Steps 1–3 need no aircraft time at all.

**Before the field — desk work**

- [ ] Make the published timestamp mean *the instant the photons hit the
      sensor*, measured per camera, not taken from a config constant (§2b).
- [ ] Publish the age alongside each sample so the consumer can lead by
      `(now − capture_ts) + t_go` instead of a fixed constant.
- [ ] Interpolate each camera's angle series to a common epoch before
      triangulating (§2a). Or hardware-trigger the pair, which retires the
      problem permanently.
- [ ] Make `boresight_offsets.json` carry the mount coordinates *and*
      `calib_heading_deg`, and refuse to load offsets against different ones.

**On site, before flying — ~30 min**

- [ ] Park an RTK aircraft on each mount, 60 s, average the position.
- [ ] Enter position **and altitude** for both cameras. Sanity check: the
      altitude should match what your aircraft report on the ground there
      (~112 m AMSL last time, not the 1.83 / 1.43 m currently in the config).
- [ ] Record the resulting baseline length. If it is not ≈95 m, something
      disagrees with our fit and is worth understanding before you fly.
- [ ] Confirm `Gimbal_Heading` is what you intend, and do not re-type it after
      this point.

**Boresight pass — hovering target, ~15 min**

- [ ] Hover the RTK aircraft at 4–6 points spanning near / mid / far, both
      sides of boresight, and two altitudes. Static, not flying.
- [ ] Log both cameras on each point for ~20 s.
- [ ] Fit the offsets, enter them, and save them together with the surveyed
      coordinates from the previous block.

**Go / no-go before the real run — all truth-free**

- [ ] Fitted boresight is **under a degree or so**. Tens of degrees means the
      mount heading was re-typed and the calibration is void.
- [ ] Median ray skew small and stable.
- [ ] Detection rate ≥1 Hz sustained. (Our estimator holds full accuracy down
      to 1 Hz and works to 0.5 Hz; below that it drops and re-acquires. A
      *fresh* 1 Hz feed beats a stale 7 Hz one — link budget is not your
      constraint, latency is.)
- [ ] `Loc_Age` in the range you expect, not the 4.4 s outliers we saw.

**Then fly the test.**

**What this will not fix, so do not be surprised**

* **Latency** — separate problem, addressed by the desk work above, and
  currently the larger one for a 1 s prediction.
* **Wrong-object lock.** If a camera settles on a bird with a plausible
  bearing, everything downstream follows it confidently. No boresight helps.
  Catching it needs an independent cue — bbox angular size versus predicted
  range is the cheapest.
* **The ~0.3–0.8° correlated wander** that remains after all of the above.
  That is the floor of this rig; do not burn field time chasing it with more
  calibration.

---

## On the "2.5 m" from the live run

We looked at `filteroutlatest.csv` from the same session. That number is real
and reproducible, but it is scored against **the stereo feed, not the target**:

| | median |
|---|---|
| filter vs its next measurement (innovation) | 2.22 m |
| filter vs measurement after update | 0.50 m |
| **filter vs the target's actual position** | **17.7 m** |

Both are true simultaneously, and the reason is §5: the feed's error is
correlated over seconds, so consecutive measurements agree with *each other*
while being wrong *together*. Predicting the next measurement is easy;
being right about the target is not. Any innovation-style metric is
structurally incapable of seeing this — please don't use one as an accuracy
figure.

(Incidentally the live guidance also had no leader state connected the whole
run — `pursuer_*` is identically zero — so its prediction horizon fell back to
a fixed constant rather than time-to-go.)

---

## What we're confident about, and what we're not

**Solid:** the per-frame noise figures; the 81 m datum offset (your target's
own GPS is accurate to 0.8 m, so this cannot be a truth artifact); the ~8 m
baseline error; the 0.15 s *differential* timing error; the LPF being worse
than `Calc`; skew being blind to time-skew.

**Less certain:** the *common-mode* part of the timing error is degenerate with
any lag in the target's own EKF output, so we can't attribute all ~0.2 s to
your side — the differential is the part we can prove is camera-side. The
absolute end-to-end transport delay is not measurable from the logs we have
(no shared clock anchor); we can only bound total consumption age at roughly
0.6–0.9 s by inference.

**A note on our own scoring:** the datum offset in §1 is fitted from truth and
removed before we compare anything, otherwise it swamps every other effect.
Absolute numbers therefore assume that one constant is genuinely constant —
which it is, to within 7 m, across all 8 windows and both flights.

---

## What would help most from the next flight

1. A segment where the target maneuvers hard and *close* (this session finally
   gave us real accelerations — 7–14 m/s² at p95 — which is what made the
   timing error visible at all).
2. Capture-instant timestamps published per camera, even before they're
   correct — just knowing what they mean lets us separate the two defects.
3. The mount coordinates you actually surveyed, with the method used.
