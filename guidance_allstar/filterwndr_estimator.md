# filterwndr.py Estimator Reference

Last updated: 2026-07-09

This is the living technical reference for `filterwndr.py`. Keep it updated whenever the estimator structure, tuning constants, model assumptions, diagnostics, or caller integration changes.

Maintenance rule for this chat:

- If a later prompt changes the estimator code, update this file in the same response.
- If a later prompt only discusses behavior or tuning conclusions, update the relevant notes/changelog section if the conclusion affects how we interpret or tune the estimator.
- If a later prompt touches guidance code that consumes the estimator state, update the integration notes.

## 1. Purpose

`filterwndr.py` estimates the target aircraft state from position-only MAVLink measurements.

The input measurement is:

```text
z = [x, y, z]
```

where the coordinates are local/NED-style position components. In this project, `z` is down-positive when sourced from local NED.

The estimator output state is 10D:

```text
x_state = [
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    omega
]
```

Meaning:

```text
x, y, z      target position
vx, vy, vz   target velocity
ax, ay, az   target acceleration states
omega        scalar yaw/heading-rate hint and CT turn-rate memory [rad/s]
```

Only position is directly measured. Velocity, acceleration, and turn rate are inferred from position history, timing, model assumptions, and the IMM probability competition.

Important product-IMM detail: the estimator no longer has one monolithic CT model that decides both horizontal and vertical motion. It now factorizes the hypothesis into a horizontal mode and a vertical mode. CT means horizontal coordinated turn, while vertical motion is independently represented by CVz or CAz. This is the intended answer to the upward-helix case: horizontal motion may be CT while Z remains CV.

The intended architecture is:

```text
MAVLink target position
-> timestamp-aware IMM estimator
-> estimated target position/velocity/turn rate
-> guidance / slot-follow / chase logic
```

## 2. High-Level Estimator Design

The estimator is an IMM: Interacting Multiple Model estimator.

It now runs a six-mode product IMM. The horizontal model set is:

```text
H = {CVxy, CTxy, CAxy}
```

The vertical model set is:

```text
Z = {CVz, CAz}
```

The full IMM model set is the Cartesian product:

$$
\mathcal{M} = \mathcal{H}\times\mathcal{Z}
$$

Implemented mode order:

```text
0: CVxy_CVz
1: CVxy_CAz
2: CTxy_CVz
3: CTxy_CAz
4: CAxy_CVz
5: CAxy_CAz
```

Each subfilter still has the same 10D state. That is required by FilterPy's `IMMEstimator`, because IMM mixing blends each model's state and covariance before prediction.

The IMM stores a six-element probability vector:

```python
imm.mu = [
    mu_cvxy_cvz,
    mu_cvxy_caz,
    mu_ctxy_cvz,
    mu_ctxy_caz,
    mu_caxy_cvz,
    mu_caxy_caz,
]
```

The user-facing probabilities are usually the marginals:

$$
\mu_{\mathrm{CTxy}}
=
\mu_{\mathrm{CTxy,CVz}}
+
\mu_{\mathrm{CTxy,CAz}}
$$

and:

$$
\mu_{\mathrm{CAz}}
=
\mu_{\mathrm{CVxy,CAz}}
+
\mu_{\mathrm{CTxy,CAz}}
+
\mu_{\mathrm{CAxy,CAz}}
$$

The final IMM estimate remains the probability-weighted state estimate over all filters:

$$
\hat{x}
=
\sum_i \mu_i \hat{x}_i
$$

## 3. Measurement Model

The measurement function is:

```python
def hx(state):
    return state[0:3]
```

This means the sensor only observes:

```text
[x, y, z]
```

from the full state:

```text
[x, y, z, vx, vy, vz, ax, ay, az, omega]
```

The linear CV and CA filters use the equivalent measurement matrix:

```python
H = np.zeros((3, 10))
H[0, 0] = 1.0
H[1, 1] = 1.0
H[2, 2] = 1.0
```

So:

```text
z_pred = H @ x_state
       = [x, y, z]
```

## 4. Process Noise Constants

Current top-level process noise constants:

```python
SIGMA_A_CV = 2.0
SIGMA_A_CT = 2.0
SIGMA_J_CT_Z = 5.0
SIGMA_J_CT_3D = SIGMA_J_CT_Z
SIGMA_J_CA = 5.0
SIGMA_OMEGA_DOT = 0.3
```

Interpretation:

```text
SIGMA_A_CV       random acceleration level for the CV model
SIGMA_A_CT       random acceleration level for CTxy position/velocity uncertainty
SIGMA_J_CT_Z     legacy name from the older CT vertical-CA implementation
SIGMA_J_CT_3D    legacy alias retained for compatibility with old notes/helpers
SIGMA_J_CA       jerk level for CAxy and CAz product modes
SIGMA_OMEGA_DOT  turn-rate random-walk level for CT omega
```

These constants control `Q`, the process covariance.

General tuning rule:

```text
Increase Q -> model is less trusted, adapts faster, more noise
Decrease Q -> model is more trusted, smoother, more lag
```

## 5. Measurement Noise R

Current measurement covariance:

```python
MEASUREMENT_R_DIAG = np.array([0.0225, 0.0225, 0.0225], dtype=float)
kf.R = np.diag(MEASUREMENT_R_DIAG)
```

This means:

```text
X variance = 0.0225 m^2 -> sigma_x = 0.15 m
Y variance = 0.0225 m^2 -> sigma_y = 0.15 m
Z variance = 0.0225 m^2 -> sigma_z = 0.15 m
```

These values were re-fit on 2026-07-09 against the current transport
(message-time stamping, `SET_MESSAGE_INTERVAL`, ~4 Hz). The measured SITL
position noise floor is only 1.5-3 cm sigma per axis; 15 cm keeps a 5-10x
robustness margin above it. The previous `[1.0, 1.0, 0.5]` was fit when
receive-time stamping added ~1 m of effective timing noise. Keeping it after
the timing fix made the filter effectively deaf at turn entry: a 0.5 m
turn-onset innovation looked like ordinary noise, CTxy could not win the
likelihood competition until the error was huge, and the late catch-up was
the "CT switch transient".

R must track the real measurement source. If transport quality changes
(receive-time stamping, ~1-2 Hz data, camera detections), re-fit R first —
against the old 1 Hz wall-clock transport the effective noise was 20-40 cm
and this R would be too tight.

`R` answers this question:

```text
How noisy do we believe the incoming position measurements are?
```

Tuning rule:

```text
Estimate is too jumpy / update_jump too large:
    increase R

Estimate is too sluggish / lags real turns too much:
    decrease R
```

Important: `R` affects how strongly each new target position pulls the estimate. It does not fix stale data or timestamp errors. Timing must be correct first.

## 6. CV Model

CV means Constant Velocity.

Its physical assumption:

```text
The target keeps moving with approximately constant velocity.
Acceleration exists, but only as process noise.
```

State transition:

```text
x_new = x + vx * dt
y_new = y + vy * dt
z_new = z + vz * dt

vx_new = vx
vy_new = vy
vz_new = vz
```

The CV model does not use:

```text
ax, ay, az, omega
```

Those state slots still exist because all IMM models must share state dimensions.

### CV Q

`q_cv_10d(dt, sigma_a)` builds a physically structured random-acceleration process covariance.

For each axis:

```text
Q[p, p] = 0.25 * dt^4 * sigma_a^2
Q[p, v] = 0.5  * dt^3 * sigma_a^2
Q[v, p] = 0.5  * dt^3 * sigma_a^2
Q[v, v] = dt^2      * sigma_a^2
```

This is a standard discrete white-acceleration model. It says:

```text
Acceleration uncertainty causes both position and velocity uncertainty.
The longer the dt, the more uncertainty grows.
```

Note the powers of `dt`. A large timestamp gap can inflate covariance significantly.

## 7. CT Model

CT means Coordinated Turn.

In the current product IMM, CT is specifically the horizontal model `CTxy`. It models the XY projection as a coordinated turn and leaves Z to the paired vertical model:

```text
CTxy_CVz = horizontal coordinated turn + vertical constant velocity
CTxy_CAz = horizontal coordinated turn + vertical constant acceleration
```

This is different from the old single CT branch. The important change is that a horizontal turn no longer forces one fixed Z assumption. For an upward helix, the desired winning model can be:

```text
CTxy_CVz
```

For a turn entry with height loss or recovery, the desired winning model can be:

```text
CTxy_CAz
```

### CTxy Dynamics

The CTxy transition function is `fx_ctxy_product(state, dt, z_mode)`.

For the horizontal state:

$$
s_{xy}
=
\begin{bmatrix}
x & y & v_x & v_y
\end{bmatrix}^{T}
$$

and scalar turn rate \(\omega\), define:

$$
\theta = \omega \Delta t
$$

When \(|\omega|\) is not tiny, the exact constant-turn-rate update is:

$$
x_{k+1}
=
x_k
+
\frac{\sin\theta}{\omega}v_{x,k}
-
\frac{1-\cos\theta}{\omega}v_{y,k}
$$

$$
y_{k+1}
=
y_k
+
\frac{1-\cos\theta}{\omega}v_{x,k}
+
\frac{\sin\theta}{\omega}v_{y,k}
$$

$$
v_{x,k+1}
=
\cos\theta\,v_{x,k}
-
\sin\theta\,v_{y,k}
$$

$$
v_{y,k+1}
=
\sin\theta\,v_{x,k}
+
\cos\theta\,v_{y,k}
$$

As \(\omega \to 0\):

$$
\frac{\sin(\omega \Delta t)}{\omega}
\to
\Delta t
$$

and:

$$
\frac{1-\cos(\omega \Delta t)}{\omega}
\to
0
$$

so CTxy smoothly becomes CVxy:

$$
x_{k+1}=x_k+v_{x,k}\Delta t,\quad
y_{k+1}=y_k+v_{y,k}\Delta t
$$

The horizontal acceleration slots \(a_x,a_y\) are not part of CTxy dynamics and are stabilized to zero for CTxy modes. Horizontal maneuver uncertainty is carried through random acceleration process noise on \(x/y,v_x/v_y\), plus omega random walk.

### CTxy Paired With Z

The vertical part is selected by the product mode.

For `CVz`:

$$
z_{k+1}
=
z_k
+
v_{z,k}\Delta t
$$

$$
v_{z,k+1}=v_{z,k},\quad a_{z,k+1}=0
$$

For `CAz`:

$$
z_{k+1}
=
z_k
+
v_{z,k}\Delta t
+
\frac{1}{2}a_{z,k}\Delta t^2
$$

$$
v_{z,k+1}
=
v_{z,k}
+
a_{z,k}\Delta t
$$

$$
a_{z,k+1}=a_{z,k}
$$

### CTxy Q

For CTxy product modes, `q_product_10d(dt, H_MODE_CT, z_mode)` uses:

```text
X/Y position and velocity: random-acceleration covariance with SIGMA_A_CT
Z position/velocity: CVz or CAz covariance, depending on z_mode
omega: random walk with SIGMA_OMEGA_DOT
```

The horizontal CTxy process noise per horizontal axis is:

$$
Q_{p,p}
=
\frac{1}{4}\Delta t^4\sigma_{a,\mathrm{CT}}^2
$$

$$
Q_{p,v}
=
Q_{v,p}
=
\frac{1}{2}\Delta t^3\sigma_{a,\mathrm{CT}}^2
$$

$$
Q_{v,v}
=
\Delta t^2\sigma_{a,\mathrm{CT}}^2
$$

Turn-rate uncertainty is:

$$
Q_{\omega,\omega}
=
\sigma_{\dot{\omega}}^2\Delta t
$$

## 8. CA Model

CA means Constant Acceleration.

Its assumption:

```text
Position changes by velocity and acceleration.
Velocity changes by acceleration.
Acceleration changes slowly through jerk noise.
```

For each axis:

```text
p_new = p + v * dt + 0.5 * a * dt^2
v_new = v + a * dt
a_new = a
```

This model is useful for:

```text
pull-ups
dips
speed changes
vertical acceleration
non-turn acceleration transients
```

It does not model horizontal turning as a coordinated turn. It explains curvature indirectly through acceleration.

### CA Q

`q_ca_10d(dt, sigma_j)` uses a white-jerk process model:

```text
Q[p, p] = dt^6 / 36 * sigma_j^2
Q[p, v] = dt^5 / 12 * sigma_j^2
Q[p, a] = dt^4 / 6  * sigma_j^2
Q[v, v] = dt^4 / 4  * sigma_j^2
Q[v, a] = dt^3 / 2  * sigma_j^2
Q[a, a] = dt^2      * sigma_j^2
```

This lets acceleration evolve without becoming completely unconstrained.

## 9. IMM Transition Matrix

The product IMM uses separate horizontal and vertical Markov matrices, then expands them into a 6x6 product transition matrix.

Horizontal transition matrix:

```python
H_MODE_TRANSITION = np.array([
    [0.93, 0.06, 0.01],  # From CVxy
    [0.08, 0.90, 0.02],  # From CTxy
    [0.07, 0.06, 0.87],  # From CAxy
])
```

Vertical transition matrix:

```python
Z_MODE_TRANSITION = np.array([
    [0.94, 0.06],        # From CVz
    [0.10, 0.90],        # From CAz
])
```

For a product mode \(i=(h_i,z_i)\) and destination mode \(j=(h_j,z_j)\), the full transition probability is:

$$
M_{ij}
=
P(h_j,z_j\mid h_i,z_i)
$$

The implementation assumes horizontal and vertical mode transitions are conditionally independent:

$$
P(h_j,z_j\mid h_i,z_i)
=
P(h_j\mid h_i)P(z_j\mid z_i)
$$

Therefore:

$$
M_{ij}
=
M^{H}_{h_i,h_j}
M^{Z}_{z_i,z_j}
$$

This is the Kronecker/product construction:

$$
M
\equiv
M^{H}\otimes M^{Z}
$$

with row/column ordering:

```text
0: CVxy_CVz
1: CVxy_CAz
2: CTxy_CVz
3: CTxy_CAz
4: CAxy_CVz
5: CAxy_CAz
```

Effect:

```text
CV is normally dominant in straight motion.
CTxy can rise during turns and is deliberately sticky once active.
CAxy can persist through horizontal acceleration-heavy motion.
CAz can persist through vertical acceleration without requiring CTxy.
```

FilterPy's `IMMEstimator` uses the convention:

```text
M[i, j] = probability of switching from model i to model j
```

That means rows are source models and columns are destination models. The current matrix therefore says:

```text
if CTxy was trusted at the previous measurement, keep CTxy with 90% horizontal prior probability at the next measurement
```

The vertical transition is independent of that horizontal decision. For example:

$$
P(\mathrm{CTxy,CAz}\mid\mathrm{CTxy,CVz})
=
P(\mathrm{CTxy}\mid\mathrm{CTxy})
P(\mathrm{CAz}\mid\mathrm{CVz})
=
0.90\cdot0.06
=
0.054
$$

This matters because the previous three-mode IMM could only say "CT" or "not CT." The product IMM can separately express:

```text
turning horizontally but Z is constant velocity
turning horizontally and Z is accelerating
not turning horizontally but Z is accelerating
```

## 10. Omega Handling

`omega` is weakly observable because no turn rate sensor is provided. It must be inferred from position samples.

This creates two problems:

1. Raw CT omega can wander even when CT probability is low.
2. All non-CTxy modes have an omega state slot, but they do not physically use omega.

The code handles this with:

```python
get_effective_turn_rate(imm)
stabilize_omega_states(imm)
```

### Effective Turn Rate

`get_effective_turn_rate()` returns zero unless:

```text
mu_ct_xy >= OMEGA_MODE_PROB_MIN
abs(omega_ct) >= OMEGA_STRAIGHT_THRESH
```

Current values:

```python
OMEGA_MODE_PROB_MIN = 0.20
OMEGA_STRAIGHT_THRESH = 0.05
OMEGA_ABS_MAX = 1.5
```

So guidance should use `omega_eff`, not raw CT omega.

### Stabilizing Unused Omega And Acceleration States

`stabilize_omega_states()` forces:

```text
all non-CTxy omega states = 0
```

and gives those omega states tiny covariance:

```python
UNUSED_STATE_VARIANCE = 1e-6
```

It also clips CT omega:

```text
omega_ct in [-1.5, +1.5] rad/s
```

For product modes it also stabilizes inactive acceleration slots:

```text
if horizontal mode is not CAxy: ax = ay = 0
if vertical mode is not CAz: az = 0
```

This prevents unused state dimensions from polluting the IMM's mixed state when probability moves between product modes.

## 11. Turn-Rate Hint

Because CT turn rate is weakly observable from position-only measurements, the code includes a helper:

```python
HeadingTurnRateEstimator
apply_fast_turn_onset_hint()
apply_turn_rate_hint()
```

This is not a replacement for the Kalman update. It is a physically informed prior nudge.

### HeadingTurnRateEstimator

The estimator now uses a causal numerical-differentiation method over recent timestamped XY positions.

It keeps a rolling history of recent samples:

```text
(t_i, x_i, y_i)
```

At each new sample it fits a local quadratic, evaluated at the newest timestamp:

```text
x(t) ~= c0_x + c1_x tau + c2_x tau^2
y(t) ~= c0_y + c1_y tau + c2_y tau^2
tau = t - t_latest <= 0
```

Then it derives:

```text
vx = c1_x
vy = c1_y
ax = 2*c2_x
ay = 2*c2_y
```

and computes heading rate from planar curvature:

```text
omega_meas = (vx * ay - vy * ax) / (vx^2 + vy^2)
```

This is causal because the fit only uses the current and older samples. It is also less jumpy than directly differencing two headings, because velocity and acceleration come from one local fit.

While history is still too short for the quadratic fit, it falls back to the older causal heading-difference estimate:

```text
v_xy = (pos_xy - prev_pos_xy) / dt
heading = atan2(vy, vx)
omega_meas = wrapped_delta_heading / dt
```

The estimator keeps both raw and smoothed turn-rate values:

```text
raw_omega             immediate causal numerical-differentiation omega
speed_xy              measured horizontal speed used for heading-rate validity
fast_onset_strength   normalized raw turn-onset strength
omega                 smoothed heading-rate hint
```

The raw value is intentionally not the normal guidance turn rate. It is used to detect turn onset before the smoothed/deadbanded `omega_hint` reacts.

It ignores heading rate if horizontal speed is too low:

```python
TURN_HINT_MIN_SPEED = 2.0
```

It smooths the measured omega:

```python
self.omega = (1 - TURN_HINT_ALPHA) * self.omega + TURN_HINT_ALPHA * omega_meas
```

Current:

```python
TURN_HINT_ALPHA = 0.45
```

It returns zero for tiny turn rates:

```python
TURN_HINT_DEADBAND = 0.08
```

### Fast Raw Turn Onset

The fast onset path uses raw heading rate before EMA smoothing:

```python
FAST_TURN_ONSET_RAW_OMEGA = 0.07
FAST_TURN_ONSET_FULL_SCALE = 0.25
FAST_TURN_ONSET_ALPHA = 0.25
FAST_TURN_ONSET_CT_MU_FLOOR = 0.2
```

`apply_fast_turn_onset_hint(imm, raw_omega, speed_xy)` does three deliberately limited things:

```text
1. Checks speed_xy and abs(raw_omega).
2. Lightly moves every CTxy branch omega toward raw_omega.
3. Raises aggregate CTxy probability only to a floor below OMEGA_MODE_PROB_MIN.
```

The configured probability floor is clipped below:

```python
OMEGA_MODE_PROB_MIN = 0.20
```

So with the current code, raw onset can prepare CTxy before the normal turn hint catches up, but it does not raise aggregate CTxy above the normal `omega_eff` gate by itself. The effective floor is:

```python
min(FAST_TURN_ONSET_CT_MU_FLOOR, OMEGA_MODE_PROB_MIN - 1e-3)
```

With the current values, this is `0.199`. If CTxy is already above the gate because the IMM likelihood supports it, raw onset can still seed the active CTxy omega state.

### Spline-Based Omega Estimate

A spline can be useful for estimating turn rate, but only if it is used as a smoothing fit, not as exact interpolation through every noisy point.

The previous method estimated heading rate from two consecutive position samples:

```text
position samples -> finite-difference velocity -> heading -> heading difference / dt
```

That is causal and low-latency, but it is sensitive to sample jitter and position noise. With MAVLink data arriving roughly every 0.4-0.6 s, a small lateral position error can create a visibly noisy heading-rate estimate.

The implemented smoother version keeps a short window of recent timestamped XY positions and fits local causal curves:

```text
x = x(t)
y = y(t)
```

then compute derivatives:

```text
vx = dx/dt
vy = dy/dt
ax = d2x/dt2
ay = d2y/dt2
```

and estimate horizontal heading rate from curvature:

```text
omega = (vx * ay - vy * ax) / (vx^2 + vy^2)
```

This formula avoids directly differencing headings, which is good because heading differences can jump around when the velocity vector is noisy.

However, exact cubic spline interpolation is risky here:

```text
It passes exactly through noisy measurements.
Its first and second derivatives can amplify measurement noise.
It can overshoot near sharp turn onsets.
Centered splines need future samples, which adds delay.
Causal endpoint spline derivatives are often biased and can react late.
```

So the preferred version would not be "interpolate the points and differentiate." It would be one of these:

```text
causal smoothing spline
local weighted quadratic/cubic fit
Savitzky-Golay style local polynomial derivative
small Kalman/alpha-beta-gamma prefilter for XY before omega calculation
```

For the current online guidance loop, the best next step would be a causal local polynomial fit over the last 5-7 valid target samples. That gives smoother `vx`, `vy`, `ax`, and `ay` while keeping delay bounded. A centered spline would look better in logs but would be less honest for real-time control because it uses future data.

### apply_turn_rate_hint

`apply_turn_rate_hint(imm, omega_hint)` does two things:

1. Moves every CTxy branch omega toward the measured heading-rate hint.
2. Optionally blends the horizontal IMM marginal toward a turn-favoring distribution while preserving the current vertical marginal.

Strength is computed from hint magnitude:

```text
0 strength at |omega_hint| <= 0.06
1 strength near |omega_hint| >= 0.35
```

Current probability target range:

```python
TURN_HINT_CT_MU_MIN = 0.12
TURN_HINT_CT_MU_MAX = 0.75
TURN_HINT_CA_MU_MAX = 0.15
TURN_HINT_MU_BLEND = 0.0
```

Since 2026-07-09 the probability blend is disabled (`TURN_HINT_MU_BLEND = 0.0`).
With the re-fit measurement R, the normal IMM likelihood competition activates
CTxy within 1-2 packets of a real turn entry on its own, faster and cleaner than
the forced mu rewrite, which was both late and a direct source of output jumps
(shifting mu between models whose states disagree jumps the combined estimate
without any measurement evidence). The hint still seeds CTxy omega and inflates
its omega covariance; only the probability rewriting is off. The `TURN_HINT_CT_MU_*`
targets are inactive while the blend is zero.

During a clear horizontal turn, aggregate CTxy becomes dominant through the
likelihood competition. During straight flight, aggregate CVxy remains dominant.
If the probability blend is re-enabled, the vertical distribution is still not
overwritten by a horizontal turn hint:

$$
\mu'_{h,z}
=
\mu'_{h}\mu_{z}
$$

where \(\mu'_h\) is the hinted horizontal marginal and \(\mu_z\) is the current vertical marginal.

## 12. Timing Model

The estimator uses timestamp-based dt from target messages.

Runtime logic:

```python
dt_raw = stamp - last_stamp
dt_meas = min(max(dt_raw, MIN_FILTER_DT), MAX_FILTER_DT)
```

Current caps:

```python
MIN_FILTER_DT = 0.03
MAX_FILTER_DT = 3.0
```

Why cap dt?

```text
Very small dt can create unstable finite differences and tiny Q.
Very large dt should only guard truly pathological stalls.
```

Important: clamping dt below the true measurement gap is not a conservative
choice — it corrupts velocity. The filter then compresses the target's real
displacement into a shorter assumed interval, which scales every inferred
velocity by (true gap / clamped dt). With the old `MAX_FILTER_DT = 1.0`, a
logged 2 s MAVLink stall mid-turn produced a velocity estimate almost exactly
2x the true velocity, followed by a violent swing back. Prediction is
substepped (`PREDICT_MAX_SUBSTEP`), so honestly propagating several seconds is
numerically safe, and the honest Q growth over the gap is the correct way to
express the uncertainty. For stalls longer than the cap, re-initializing the
filter is better than lying about elapsed time.

## 13. Prediction Over Long dt

`predict_imm_over_dt(imm, dt)` predicts across the full dt using smaller substeps:

```python
PREDICT_MAX_SUBSTEP = 0.1
```

For example:

```text
dt = 0.54 s
```

is predicted as roughly:

```text
0.1 + 0.1 + 0.1 + 0.1 + 0.1 + 0.04
```

This is especially important for CTxy because the nonlinear horizontal term depends on:

```text
abs(omega) * dt
```

Smaller substeps make the UKF propagation more stable.

Important implementation detail:

```text
The IMM model-mixing step is applied once per measurement interval.
The individual product-mode filters are then propagated through the smaller substeps.
```

This is deliberate. FilterPy's `imm.predict()` performs both model mixing and filter prediction. If `imm.predict()` is called once per 0.1 s substep, then a 0.5 s MAVLink interval mixes the product modes five times before a single measurement update. That repeated mixing washes out CTxy's separate turn state and makes CTxy look like it only activates momentarily at turn onset.

The current `predict_imm_over_dt()` therefore does the IMM mixing once, then calls each sub-filter's own `predict()` across the substeps. This preserves numerical stability without accidentally making the transition matrix much more aggressive than intended.

## 14. Runtime Update Sequence

The main diagnostic loop performs:

```text
1. Read target position and timestamp.
2. Reject stale samples with the same timestamp.
3. Compute dt_raw from target timestamps.
4. Clamp dt to [0.03, 1.0].
5. Convert target measurement to z_meas.
6. Estimate heading-rate hint from recent measurements.
7. Apply turn-rate hint before prediction.
8. Predict IMM over dt.
9. Compute innovation.
10. Update IMM with z_meas.
11. Apply turn-rate hint again.
12. Stabilize omega states.
13. Compute update jump and diagnostics.
14. Log to CSV and update plots.
```

The two most important diagnostic vectors are:

```python
innovation = z_meas - x_pred[0:3]
update_jump = x_upd[0:3] - x_pred[0:3]
```

Interpretation:

```text
Large innovation:
    prediction and measurement disagree.

Large update_jump:
    the measurement pulled the estimator strongly.

Large innovation but small jump:
    filter saw disagreement but mostly trusted the model.

Large jump:
    filter accepted a large measurement correction.
```

### Innovation And Update-Jump Synchronization

It is normal for innovation and update jump to be synchronized. In a Kalman update:

$$
\nu_k
=
z_k - H\hat{x}_k^{-}
$$

is the innovation, and:

$$
\hat{x}_k^{+}
=
\hat{x}_k^{-}
+
K_k\nu_k
$$

Therefore the full state update jump is:

$$
\Delta \hat{x}_k
=
\hat{x}_k^{+}-\hat{x}_k^{-}
=
K_k\nu_k
$$

The logged position update jump is the position slice of that correction:

$$
\Delta \hat{p}_k
=
L_pK_k\nu_k
$$

where \(L_p\) selects the position components from the 10D state.

So synchronized innovation and jump does not mean the jump caused the original prediction error. The innovation comes first:

```text
prediction disagrees with measurement -> innovation
Kalman gain decides how much to correct -> update jump
```

The ratio is the important clue:

```text
large innovation + small jump:
    filter mostly trusted the physical model / prior covariance was small / R was large.

large innovation + large jump:
    filter accepted the measurement strongly / gain was high / model uncertainty was large or R was small.

large innovation + large jump + bad-looking measurement:
    suspect measurement outlier, timing issue, or R too small.

large innovation + large jump + measurement is believable:
    model prediction is not keeping up, or Q/model switching should allow more maneuvering.
```

This is why innovation alone is not enough to say "model bad" or "model good." Innovation measures disagreement. Update jump measures how much the filter chose to believe the measurement over the predicted physical model.

## 15. Diagnostics CSV

`IMMDiagnosticLogger` writes:

```text
logs/imm_diagnostics_YYYYMMDD_HHMMSS.csv
```

Important fields:

```text
dt_raw              raw target timestamp gap
dt_meas             clamped dt
dt_actual           dt actually used after optional smoothing
meas_*              raw measured target position
pred_*              predicted position before update
est_*               posterior estimated position
est_v*              posterior estimated velocity
err_*               est - measurement
innov_*             measurement - prediction
jump_*              posterior - prediction
mu_cv/mu_ct/mu_ca   horizontal marginal aliases for CVxy/CTxy/CAxy
mu_cv_xy etc.       horizontal and vertical marginal probabilities
mu_ctxy_cvz etc.    individual six product-mode probabilities
omega_hint          measured heading-rate hint
turn_hint_strength  normalized turn hint strength
raw_omega           immediate unsmoothed heading-rate measurement
raw_turn_strength   normalized fast raw turn-onset strength
raw_speed_xy        horizontal speed used by raw heading-rate calculation
omega_ct_raw        aggregate CTxy-weighted raw omega
omega_eff           mode-gated effective turn rate
likelihood_*        aggregate and per-product-mode measurement likelihood
apparent_delay_s    estimated along-track delay
```

Use these fields to diagnose the estimator:

```text
High high-frequency noise:
    Check jump_norm and innovation_norm.

CT not activating:
    Check omega_hint, turn_hint_strength, likelihood_ct_xy, and mu_ct_xy.

Large spikes after target stalls:
    Check dt_raw and dt_actual.

Turn-rate false positives:
    Compare omega_ct_raw with omega_eff and mu_ct_xy.
```

## 16. Plot Layout

The diagnostic plot has three stacked panels. The former 3D trajectory panel was removed on 2026-07-09: it was the single heaviest X-server consumer (3D redraw + `tight_layout()` at 10 Hz) and destabilized WSLg's Xwayland when run alongside Gazebo, and it was not used for tuning.

```text
1. Estimation error per axis (est - measurement)
2. IMM product-model marginals + effective omega
3. Innovation norm / update jump norm / error norm
```

The plotted error is the estimate minus the raw MAVLink measurement, not the estimate minus perfect ground truth:

```text
error = estimate - measurement
```

For estimate-vs-truth analysis, use `imm_replay_eval.py`, which builds smoothed ground truth from the logged measurements offline and reports per-turn-entry transient peaks (see the 2026-07-09 changelog entry).

## 17. Optional Low-Pass Filter

`IMMLowPassFilter` can smooth the IMM output before plotting or legacy guidance consumers.

Config-controlled:

```python
IMM_LPF_ENABLED
IMM_LPF_ALPHA
IMM_LPF_MOTION_COMPENSATED
```

If disabled, it returns the raw IMM state.

Plain EMA behavior is:

```python
self.state = self.state + alpha * (new_state - self.state)
```

That reduces visible jitter, but it creates a steady-state position lag for a moving target. For a ramp-like constant-velocity target, the approximate lag is:

```text
position_lag ~= ((1 - alpha) / alpha) * velocity * dt
```

With `alpha = 0.8`, `dt = 0.1 s`, and target speed `10 m/s`, the expected lag is about `0.25 m`.

The current implementation supports motion compensation:

```python
IMM_LPF_MOTION_COMPENSATED = True
```

When enabled, the LPF first predicts the previous filtered state forward with its own velocity/acceleration, then blends the new IMM state:

```text
base_pos = filtered_pos + filtered_vel * dt + 0.5 * filtered_acc * dt^2
base_vel = filtered_vel + filtered_acc * dt
filtered = base + alpha * (new_state - base)
```

This preserves smoothing while removing most of the constant-velocity steady-state lag. The standalone estimator passes `dt_actual` into the LPF, and `lag_pursuit_pid.py` passes its loop `dt`. `simple_guided_follow.py` does not currently use `IMMLowPassFilter`; it predicts from the raw IMM state.

## 18. Out-of-Sequence Measurement Tracker

`OOSM_IMM_Tracker` is a helper for delayed measurements.

It stores a history of:

```text
timestamp
mode probabilities
per-filter x
per-filter P
IMM x
```

For a delayed measurement:

```text
1. Find closest historical state.
2. Restore filter bank to that state.
3. Apply delayed measurement.
4. Predict forward to present.
5. Replace latest history entry.
```

This is useful conceptually for delayed camera detections. It is less central to the current `filterwndr.py` diagnostic loop, which mainly uses fresh MAVLink positions.

## 19. Integration With simple_guided_follow.py

`simple_guided_follow.py` imports:

```python
HeadingTurnRateEstimator
aggregate_mode_probabilities
apply_fast_turn_onset_hint
apply_turn_rate_hint
ct_mode_probability
predict_imm_over_dt
setup_imm_filter
stabilize_omega_states
```

Its target-update path now uses two clocks deliberately:

```text
filter dt and HeadingTurnRateEstimator use target message time
(last GLOBAL_POSITION_INT time_boot_ms when available)
wall time is used only for stale-target detection and loop pacing
```

The runtime update sequence in `simple_guided_follow.py` is therefore:

```text
only update IMM when target message stamp advances
compute dt from target message stamp - last target message stamp
apply fast raw turn-onset hint
apply turn-rate hint
predict over dt
update with z_meas
apply hint again
stabilize omega states
```

After the IMM update, the runner now uses a motion-compensated `IMMLowPassFilter` on the predicted state before guidance consumes it. Guidance then uses a position-only terminal policy rather than a raw per-tick chase of the target position.

The Z switch limiter now tracks aggregate CTxy probability:

$$
\mu_{\mathrm{CTxy}}
=
\mu_{\mathrm{CTxy,CVz}}
+
\mu_{\mathrm{CTxy,CAz}}
$$

not a fixed `imm.mu[1]` index. This is required because `imm.mu[1]` is now `CVxy_CAz`, not CT.

### Z Transients During CT Switches

`simple_guided_follow.py` currently sends:

```text
slot_pos = target_pos - back * heading + side * right + down
slot_vel = target_vel
```

Then it sends `slot_pos` and optionally `slot_vel` to ArduPilot as a local-NED position target.

Important yaw detail:

```text
By default, simple_follow ignores yaw and yaw-rate.
If YAW_LOCK_ENABLED = True, it sends yaw toward LOS from pursuer to the predicted target position and still ignores yaw-rate.
```

So in `simple_guided_follow.py`, the estimator is not directly commanding yaw unless `YAW_LOCK_ENABLED` is true. In `lag_pursuit_pid.py`, the same flag forces yaw toward the LOS to the predicted target and `velocity_control.py` ignores body-rate fields in `SET_ATTITUDE_TARGET`, so the quaternion yaw is the authority.

A small Z transient can still create violent-looking behavior indirectly:

```text
CT/CV/CA model probability changes can create a small z/vz output jump.
slot_pos.z and slot_vel.z are sent directly to the vehicle.
the vehicle reacts vertically or changes thrust allocation.
if yaw is locked to velocity/heading elsewhere, command jitter can become yaw jitter.
```

Tuning `R_z` upward can hide this, but it is a blunt estimator-side fix. It tells the estimator to trust Z measurements less everywhere, including real dives and pull-ups. Use it only if Z measurements are actually too noisy.

Better first fixes:

```text
keep thrust allocation inside a non-inverted quad envelope
shape the commanded Z setpoint and VZ feedforward after the estimator
rate-limit commanded yaw or yaw-rate
compute yaw from a smoothed horizontal LOS vector, not from raw command velocity
hold previous yaw when horizontal LOS or velocity magnitude is too small
keep estimator R_z for measurement trust, not for actuator smoothness
```

`lag_pursuit_pid.py` now applies this before converting acceleration to attitude:

```text
specific thrust f = [ax, ay, az - g]
fz is clamped non-positive so the command cannot require inverted thrust
LAG_PURSUIT_MIN_LIFT keeps a minimum upward lift component during Z spikes
XY acceleration is scaled into the remaining MAX_THRUST and MAX_TILT_DEG budget
```

The current test implementation in `simple_guided_follow.py` is optional and command-side:

```text
--z-switch-slew-rate RATE_MPS
--z-switch-jump JUMP_M
--z-switch-window WINDOW_S
--z-switch-dmu DMU
--z-switch-mu-threshold MU
--z-outlier-slew-rate RATE_MPS
--z-outlier-jump JUMP_M
--z-update-freeze-packets N
--z-ct-freeze-packets N
--z-ct-freeze-mu-threshold MU
```

The command-side slew limiters are disabled by default because `--z-switch-slew-rate` and `--z-outlier-slew-rate` both default to `0.0`.

Note: the active sim/intercept configuration now enables them in `guidance_config.py` with:

```text
Z_SWITCH_SLEW_RATE = 1.5
Z_SWITCH_JUMP_M = 0.5
Z_OUTLIER_SLEW_RATE = 1.5
```

This keeps the limiter logic optional in the runner while making the default chase configuration safer around Z transients.

`--z-update-freeze-packets` defaults to `Z_UPDATE_FREEZE_PACKETS = 2`. It freezes the estimator's vertical correction for the first `N` target packets after a fast raw turn-onset edge. During those packets, XY still updates from the measurement, but Z/VZ/AZ are restored to the predicted values after the measurement update. This lets vertical state coast on the previous `vz` instead of accepting a short bad-packet Z jump. Set it to `0` to disable.

`--z-ct-freeze-packets` defaults to `Z_CT_FREEZE_PACKETS = 3`. It freezes the estimator's vertical correction for `N` packets starting at the exact packet where aggregate `mu_ct_xy` crosses `--z-ct-freeze-mu-threshold` (default `Z_CT_FREEZE_MU_THRESHOLD = 0.20`) upward. Unlike `--z-update-freeze-packets`, which fires on the raw heading-rate onset edge, this fires on the actual CT model activation. The crossing is detected by snapshotting vertical state before the measurement update, running the update, checking whether `mu_ct_xy` crossed the threshold, and retroactively restoring vertical state if it did. This means the freeze applies to the same packet that triggered the CT activation, not the one after. The remaining `N-1` packets are then frozen via the normal `update_imm_preserving_vertical` path. Set `--z-ct-freeze-packets 0` to disable. The two freeze triggers (fast-turn-onset and CT-activation) share the same `z_update_freeze_remaining` counter and either can (re)arm it.

The same vertical-preserving update helper is available to `OOSM_IMM_Tracker`, and `lag_pursuit_pid.py` uses it during the same configurable packet window when its own heading-rate estimator detects fast turn onset.

When switch-window slew is enabled, the runner watches:

```text
large |delta mu_ct_xy|
or crossing the CT active threshold
or fast raw turn onset before CT probability reacts
```

That opens a short Z-slew window. During that window only, if the outgoing slot Z command jumps more than `--z-switch-jump`, the command Z is rate-limited by `--z-switch-slew-rate`. This does not change the estimator state or covariance.

When outlier slew is enabled, the runner also watches outgoing Z command jumps even outside turn windows:

```text
abs(slot_z - previous_commanded_slot_z) > --z-outlier-jump
```

Those jumps are rate-limited by `--z-outlier-slew-rate`. This is meant for Z events that do not have turn evidence.

This is intentionally different from replaying previous measurements into the estimator. Replaying old packets would make the Kalman filters treat stale data as fresh independent measurements, which can shrink covariance and create false confidence.

Recent log check:

```text
logs/imm_diagnostics_20260509_150900.csv
```

showed the largest Z jumps around steps 163-171 while legacy `mu_ct` was almost flat near 0.17. In the current product logs, read that as aggregate `mu_ct_xy`. That pattern looks more like a vertical measurement/trajectory burst or vertical velocity mismatch than a pure CT model switch. CT probability only rose afterward around steps 172-175, where there was a smaller but still visible Z update jump.

So the model-switch-only limiter is a good targeted experiment for yaw/control smoothness, but it should not be expected to remove every Z transient. If the logs keep showing large Z jumps without CT probability movement, use a Z measurement gate or command-side Z smoother that is not tied only to model switches.

Follow-up log check:

```text
large Z event before CT switch can be real
but it is not the only cause of Z events
```

In `logs/imm_diagnostics_20260509_150900.csv`, recomputing raw horizontal heading rate from measured XY showed:

```text
step 166: raw omega about +0.017 rad/s
step 168: raw omega about +0.020 rad/s
step 169: raw omega about +0.037 rad/s
step 170: raw omega about +0.095 rad/s
step 171: raw omega about +0.092 rad/s
step 172: omega_hint finally becomes nonzero and mu_ct_xy crosses about 0.22
```

The large positive Z innovation/update jump is strongest around steps 168-171:

```text
step 170: innov_z about +1.27 m, jump_z about +0.71 m
step 171: innov_z about +1.16 m, jump_z about +0.72 m
```

So for that latest run, the user's hypothesis is plausible: the aircraft had started curving in measured XY, but the smoothed/deadbanded turn hint had not yet lifted CT probability. CV was still dominant while the target was entering a coupled turn/climb or turn/pull-up segment.

However, a scan of recent logs found many large Z events with no raw horizontal turn evidence. Those are probably vertical trajectory changes, measurement bursts, or prediction/measurement timing mismatch. Therefore, a turn-onset fix alone will not solve every Z transient.

Better solution than only reacting to CT model switch:

```text
detect turn onset before CT switch
```

Use raw or lightly filtered heading-rate evidence from the `HeadingTurnRateEstimator`, separate from the smoothed `omega_hint` used by guidance:

```text
raw heading-rate onset if:
    speed_xy is high enough
    abs(raw_omega) exceeds a small threshold
    optionally confirmed for two packets
```

When this fast onset detector fires:

```text
open the command-side Z slew window immediately
seed CT omega lightly
raise CT probability only to a modest floor
do not raise CTxy probability above the omega_eff gate by raw onset alone
```

This avoids the current delay where raw heading rate is visible for one or two packets before `omega_hint` and `mu_ct_xy` react.

Implemented behavior:

```text
HeadingTurnRateEstimator exposes raw_omega, speed_xy, and fast_onset_strength.
apply_fast_turn_onset_hint() uses abs(raw_omega) >= 0.06 rad/s.
simple_guided_follow.py opens the Z command slew window on:
    CT probability switch
    fast raw turn onset
simple_guided_follow.py can also apply a separate always-on Z outlier slew.
all command-side Z limiting is disabled by default
```

Do not simply replay previous packets into the estimator. Replayed old packets are not independent measurements; they can make the filter overconfident and can hide the real timing problem.

Estimator-side model fix now implemented:

```text
the estimator now uses a six-mode product IMM.
CTxy handles horizontal coordinated turn.
CVz/CAz independently handle vertical motion.
upward helix can be CTxy_CVz.
turn with height loss/recovery can be CTxy_CAz.
```

Current position-only `simple_guided_follow.py` guidance-side integration now adds four stabilizers on top of that estimator output:

```text
1. message-time dt for IMM updates and turn-rate estimation
2. motion-compensated IMMLowPassFilter before slot generation
3. terminal position-target extension: inside TERMINAL_POSITION_EXTEND_RANGE_M,
   place the commanded position TERMINAL_POSITION_EXTEND_DISTANCE_M beyond the
   target along LOS so GUIDED does not decelerate to stop on the target point
4. endgame latch (freeze & spear): when t_go < TERMINAL_LATCH_TGO_S, hold the
   current terminal position target until range re-opens after a miss
```

The runner still stays on the position-only path; no target velocity measurement is consumed in the estimator, and no velocity-only terminal setpoints are used here.

It also now asserts key GUIDED/WPNAV parameters at startup and requests target `GLOBAL_POSITION_INT` by `MAV_CMD_SET_MESSAGE_INTERVAL` instead of relying only on the older stream request. Those are command-path and transport fixes rather than estimator-structure changes, but they materially reduce the apparent estimator noise seen by guidance.

## 20. Known Current Behavior

Current known behavior from recent logs:

```text
Large old transient spikes were mainly timing-related.
Remaining high-frequency noise is mostly measurement-update pull.
CT raw omega can wander, but omega_eff is mode-gated.
CT activation was improved with heading-rate hints.
CTxy is now a horizontal marginal inside a six-mode product IMM.
Vertical behavior can choose CVz or CAz independently of CTxy.
```

The remaining noise source is generally:

```text
slow / irregular target position samples
+ noisy position measurement
+ position-only inferred velocity/turn rate
```

The estimator should not be expected to produce perfect velocity or turn rate from sparse position-only measurements.

## 21. Tuning Guide

### If Estimate Is Too Jumpy

Symptoms:

```text
jump_norm high
estimate visually snaps toward each measurement
high-frequency position noise
```

First try:

```text
Increase R, especially XY.
```

Example (relative to the current `0.0225` baseline):

```python
kf.R = np.diag([0.09, 0.09, 0.09])
```

Tradeoff:

```text
less jitter, more lag
```

### If Estimate Lags Turns

Symptoms:

```text
innovation grows during turns
CT probability rises too late
estimate cuts behind target path
```

Possible changes:

```text
lower R slightly
increase SIGMA_A_CT
increase TURN_HINT_MU_BLEND
lower TURN_HINT_DEADBAND
increase CV -> CT transition probability
```

Change one thing at a time.

### If CT Activates Falsely

Symptoms:

```text
omega_eff nonzero on straight segments
mu_ct_xy high without real heading rate
guidance widens slot unnecessarily
```

Possible changes:

```text
increase OMEGA_MODE_PROB_MIN
increase OMEGA_STRAIGHT_THRESH
increase TURN_HINT_DEADBAND
decrease TURN_HINT_CT_MU_MAX
```

### If CA Does Not Track Vertical Maneuvers

Symptoms:

```text
Z error grows during pull-up/drop
CA probability remains very low
CT/CA both underperform in vertical motion
```

Possible changes:

```text
increase SIGMA_J_CA
increase CAz prior/transition persistence if CAz is too reluctant
reduce R_z if Z measurements are trustworthy
```

### Verifying Physical Model Variance

A useful way to think about `Q` and `R` is:

```text
predict next position from the physical model
compare predicted position with the next observation
use that mismatch to judge whether the assumed model variance is realistic
```

This is already the central Kalman-filter loop. In the code, the diagnostic variable:

```python
innovation = z_meas - x_pred[0:3]
```

is exactly:

```text
observation - physical-model prediction
```

For CV, the physical prediction is equivalent to Euler / constant-velocity integration:

```text
p_next = p + v * dt
```

For CA, it is constant-acceleration integration:

```text
p_next = p + v * dt + 0.5 * a * dt^2
v_next = v + a * dt
```

For CT, it is the coordinated-turn nonlinear propagation.

The important distinction:

```text
The code already computes the mismatch and Kalman gain.
The code does not yet automatically retune Q/R from that mismatch.
```

`Q` tells the filter how uncertain the physical model is. `R` tells the filter how uncertain the measurement is. The Kalman gain is computed from both:

```text
larger Q -> larger predicted covariance -> more measurement correction
larger R -> smaller Kalman gain -> less measurement correction
```

The formal consistency metric for this idea is NIS:

```text
Normalized Innovation Squared
```

If NIS is consistently too high, the filter is more surprised than its own covariance predicted. That usually means:

```text
Q is too small
R is too small
the model class is wrong
dt/timestamp handling is wrong
or the measurements contain outliers
```

If NIS is consistently too low, the filter is overly conservative:

```text
Q/R may be too large
```

So the idea is sound, but the rigorous version is not just raw Euler error. It is innovation normalized by predicted innovation covariance.

### Fix Plan For Turn-Onset Z Spike

The current Z spike on turn onset should be treated as two coupled problems:

```text
estimator output transient
command/control sensitivity to that transient
```

The fastest safe mitigation is command-side:

```text
enable turn-window Z slew limiting
optionally enable always-on Z outlier slew limiting
```

Example runtime test:

```bash
python3 simple_guided_follow.py --z-switch-slew-rate 1.5 --z-switch-jump 0.5 --z-switch-window 1.2
```

If logs show large Z jumps even without turn evidence:

```bash
python3 simple_guided_follow.py --z-switch-slew-rate 1.5 --z-switch-jump 0.5 --z-outlier-slew-rate 1.5 --z-outlier-jump 0.9
```

This protects the drone/yaw controller without lying to the estimator.

The better estimator-side fix is a per-update Z innovation gate or adaptive \(R_z\), not a global increase in \(R_z\). For the Z component:

$$
\nu_z = z_{\text{meas}} - z_{\text{pred}}
$$

and:

$$
S_z = P_{zz}^{-} + R_z
$$

The 1D normalized innovation squared is:

$$
\operatorname{NIS}_z
=
\frac{\nu_z^2}{S_z}
$$

If:

$$
\operatorname{NIS}_z > \gamma_z
$$

then the Z measurement is inconsistent with the predicted vertical uncertainty. A reasonable first threshold is:

$$
\gamma_z \in [6.63,\ 9.0]
$$

where \(6.63\) is the 99% threshold for a 1D \(\chi^2\) test.

On that one update only, do one of:

```text
inflate R_z for this update
skip only the Z component of the update
limit the Z correction magnitude
```

The preferred first implementation is temporary \(R_z\) inflation:

$$
R_z' = \lambda R_z
$$

with:

$$
\lambda \in [5,\ 20]
$$

only for the suspicious update. This preserves normal vertical tracking during real dives/pull-ups better than permanently increasing \(R_z\).

Alternative fallback if residual Z spikes remain:

```text
decouple vertical filtering from horizontal model switching
```

That would mean:

```text
horizontal XY uses IMM CVxy/CTxy/CAxy competition
vertical Z uses a separate 1D CV/CA-style filter or smoother
final output combines XY from IMM and Z from the vertical filter
```

This prevents horizontal CT activation from directly creating a vertical output transient.

### Axis-Wise IMM vs Factorized IMM

Idea:

```text
run one IMM for X
run one IMM for Y
run one IMM for Z
```

This sounds attractive because each axis can choose its own model. For example, for an upward helix:

```text
XY plane: coordinated turn
Z axis: constant velocity
```

A fully independent axis-wise IMM would not force the Z channel to follow the XY turning model.

However, a pure per-axis IMM is not a good replacement for the current estimator because CT is not a 1D concept. Coordinated turn is fundamentally a coupled-plane model.

For a horizontal circular turn:

$$
x(t) = R\cos(\omega t)
$$

$$
y(t) = R\sin(\omega t)
$$

The velocities are:

$$
\dot{x}(t) = -R\omega\sin(\omega t)
$$

$$
\dot{y}(t) = R\omega\cos(\omega t)
$$

The accelerations are:

$$
\ddot{x}(t) = -\omega^2 x(t)
$$

$$
\ddot{y}(t) = -\omega^2 y(t)
$$

The key information is not in X or Y alone. It is in their phase-coupled geometry:

$$
x^2 + y^2 = R^2
$$

and:

$$
\mathbf{a}_{xy}
=
\omega
\begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
\mathbf{v}_{xy}
$$

If X and Y are filtered independently, the estimator can lose:

```text
shared turn rate
phase relationship between x and y
common circular/curved geometry
consistent heading estimate
```

So three independent 1D IMMs would make Z independence easy, but it would weaken the most important part of the CT model.

The implemented architecture is the finite product version of a factorized IMM:

```text
full shared 10D state
models = {CVxy, CTxy, CAxy} x {CVz, CAz}
```

The six product modes are:

```text
CVxy_CVz
CVxy_CAz
CTxy_CVz
CTxy_CAz
CAxy_CVz
CAxy_CAz
```

This would naturally represent an upward helix:

$$
x(t) = R\cos(\omega t)
$$

$$
y(t) = R\sin(\omega t)
$$

$$
z(t) = z_0 + v_z t
$$

The horizontal IMM can choose CT while the vertical IMM chooses CV.

This gives the estimator explicit models for:

```text
level horizontal turn
upward/downward helix
turn with altitude loss/recovery through CTxy_CAz
straight climb/descent
vertical pull-up/drop
```

Current recommendation:

```text
Do not split into three independent 1D IMMs.
Use the implemented product IMM and tune the horizontal/vertical marginals separately.
```

This preserves CT coupling in XY while allowing Z to choose CV/CA independently.

## 22. Tunable Parameter Reference

This section explains the top-level constants in `filterwndr.py`. Some are true tuning knobs. Others are structural definitions that should only change when the estimator architecture changes.

### Runtime Dynamic Parameter Reload

The estimator now supports live parameter reload for the allow-listed tuning parameters in `filterwndr.py`.

At runtime, `refresh_dynamic_filter_params()` checks the saved file's modification time, parses simple top-level assignments, and updates the module globals without re-executing the module. This means you can edit and save values such as:

```text
SIGMA_A_CT
SIGMA_OMEGA_DOT
TURN_HINT_DEADBAND
OMEGA_DIFF_MAX_HISTORY / OMEGA_DIFF_WINDOW_S
FAST_TURN_ONSET_RAW_OMEGA
MIN_FILTER_DT / MAX_FILTER_DT
PREDICT_MAX_SUBSTEP
H_MODE_INITIAL / Z_MODE_INITIAL
H_MODE_TRANSITION / Z_MODE_TRANSITION
MEASUREMENT_R_DIAG
```

and the next estimator cycle will use the new values.

Important behavior:

```text
Only the allow-listed parameters are hot-reloaded.
The code parses assignments; it does not execute the edited Python file.
Invalid edits are ignored until the file is valid again.
Probability vectors and transition rows are normalized after reload.
Filter measurement R and the IMM transition matrix are refreshed on running filters.
Changing PRODUCT_MODE_SPECS or state dimensions is not a live-tuning operation.
```

The main runtime call paths that refresh parameters are:

```text
setup_imm_filter()
refresh_filter_dt()
predict_imm_over_dt()
apply_turn_rate_hint()
apply_fast_turn_onset_hint()
HeadingTurnRateEstimator.update()
clamp_filter_dt()
```

`simple_guided_follow.py` now uses `clamp_filter_dt()`, so live edits to `MIN_FILTER_DT` and `MAX_FILTER_DT` affect both the standalone estimator plotter and the simple follower.

### Process Noise Parameters

```python
SIGMA_A_CV = 2.0
```

Random-acceleration level used by CV-like axes. In the current product IMM this affects:

```text
CVxy horizontal axes
CVz vertical axis
```

Increasing it makes CV/CVz less rigid and more willing to adapt to new measurements. Decreasing it makes straight/constant-velocity motion smoother but more laggy.

```python
SIGMA_A_CT = 2.0
```

Random-acceleration level used for the horizontal position/velocity uncertainty in `CTxy`. This does not directly make the CT mean turn faster. The mean turn rate mostly comes from CT omega. Increasing `SIGMA_A_CT` raises CTxy covariance, so CTxy can accept measurement corrections more strongly. If CT prediction is late at turn entry, tune turn hint / omega parameters first before raising this too far.

```python
SIGMA_J_CT_Z = 5.0
SIGMA_J_CT_3D = SIGMA_J_CT_Z
```

Legacy CT jerk names from the older 3D CT implementation and compatibility helpers. The active product IMM uses `q_product_10d()` and `fx_ctxy_product()` for CTxy modes, so normal product-mode tuning should use `SIGMA_A_CT`, `SIGMA_J_CA`, and `SIGMA_OMEGA_DOT`. Keep these names only for old helpers or older callers.

```python
SIGMA_J_CA = 5.0
```

White-jerk level for CA axes:

```text
CAxy horizontal acceleration modes
CAz vertical acceleration modes
```

Increasing it lets acceleration states change faster, which can help pull-ups, dives, speed changes, and vertical transients. Decreasing it makes CA smoother and more persistent, but can lag real acceleration changes.

```python
SIGMA_OMEGA_DOT = 0.3
```

Random-walk level for CTxy omega. Increasing it lets CT turn rate adapt faster between updates. This is one of the main CT entry-lag knobs. If CT enters with large innovation because omega is behind the actual curve, test this before increasing CT position process noise. Raised from `0.08` to `0.3` on 2026-07-09 so omega converges to the plane's ~0.5 rad/s turn rate within ~2 packets of turn entry instead of ~6; with the re-fit (much smaller) measurement R, position updates constrain omega strongly enough that the larger random walk does not cause visible omega wander mid-turn.

### Omega And CT State Parameters

```python
OMEGA_ABS_MAX = 1.5
```

Absolute clip on CT omega in radians per second. It prevents weakly observable turn-rate states from diverging. Raising it allows sharper modeled turns but can make bad omega estimates more damaging.

```python
OMEGA_MODE_PROB_MIN = 0.20
```

Minimum aggregate `mu_ct_xy` required before `get_effective_turn_rate()` can return a nonzero omega. Below this gate, CT omega is treated as not trustworthy for the final effective turn rate.

```python
OMEGA_STRAIGHT_THRESH = 0.05
```

Deadband for effective omega. Even if CT probability is high enough, smaller absolute omega values are treated as straight flight.

```python
CT_OMEGA_INITIAL_STD = 0.5
```

Initial standard deviation for omega covariance in CTxy branches. Larger values make initial CT turn-rate uncertainty wider. Non-CTxy modes do not use this and get `UNUSED_STATE_VARIANCE` for omega.

```python
CT_MIN_SPEED_FOR_AXIS = 0.5
CT_MIN_TURN_ACCEL = 0.05
```

Legacy helper thresholds for estimating a 3D CT rotation axis from velocity and acceleration in `fx_ct()`. The active product IMM does not use `fx_ct()` in `setup_imm_filter()`, but these thresholds still matter if old compatibility paths call the legacy 3D CT helper.

```python
UNUSED_STATE_VARIANCE = 1e-6
```

Tiny covariance assigned to inactive state slots during stabilization:

```text
non-CTxy omega
non-CAxy ax/ay
non-CAz az
```

This prevents unused states from polluting the mixed IMM state when probability transfers between product modes.

### Normal Turn-Hint Parameters

The normal turn hint comes from the causal numerical-differentiation omega in `HeadingTurnRateEstimator`. It is smoother than raw onset and is used by `apply_turn_rate_hint()`.

```python
TURN_HINT_MIN_SPEED = 2.0
```

Minimum horizontal measured speed needed before heading-rate measurements are trusted. Below this speed, heading is too noisy to infer reliable turn rate.

```python
TURN_HINT_DEADBAND = 0.08
```

Heading-rate magnitude below which the smoothed turn hint is treated as zero. Lowering it makes CTxy react earlier to gentle turns but increases false turn risk on noisy straight segments.

```python
TURN_HINT_FULL_SCALE = 0.35
```

Heading-rate magnitude where turn-hint strength reaches full scale. The strength ramps from zero at `TURN_HINT_DEADBAND` to one near this value.

```python
TURN_HINT_ALPHA = 0.45
```

EMA smoothing factor for the measured heading-rate hint and also the blend factor used to move CTxy omega toward a valid hint. Larger values react faster but pass more measurement jitter into omega.

```python
TURN_HINT_MU_BLEND = 0.0
```

Blend factor for moving IMM probabilities toward the turn-hint target distribution. Larger values make mode probabilities respond faster to turn evidence. Too large can force CTxy during noisy heading estimates. Set to `0.0` on 2026-07-09: with the re-fit measurement R the likelihood competition switches modes within 1-2 packets by itself, and the forced probability rewrite was a direct source of model-switch output transients. Raise only if a much noisier measurement source (with correspondingly larger R) makes likelihood-driven switching too slow again.

```python
TURN_HINT_CT_MU_MIN = 0.12
TURN_HINT_CT_MU_MAX = 0.75
```

Target aggregate CTxy probability range used by the normal turn hint. At low valid hint strength, CTxy is biased toward `TURN_HINT_CT_MU_MIN`; at full strength, toward `TURN_HINT_CT_MU_MAX`.

```python
TURN_HINT_CA_MU_MAX = 0.15
```

Maximum horizontal CAxy probability target added by the normal turn hint at full strength. This leaves some probability for non-CT acceleration-heavy motion during turn entry/exit.

```python
OMEGA_DIFF_MAX_HISTORY = 7
```

Maximum number of timestamped XY position samples kept by `HeadingTurnRateEstimator` for causal numerical differentiation.

```python
OMEGA_DIFF_MIN_SAMPLES = 3
```

Minimum number of samples needed before the causal quadratic fit is used. Before this many samples exist, the estimator falls back to heading differencing.

```python
OMEGA_DIFF_WINDOW_S = 3.0
```

Maximum age window for samples used by the causal fit. Older samples are dropped so stale path geometry does not keep influencing the current turn-rate estimate.

```python
OMEGA_DIFF_MIN_SPAN_S = 0.15
```

Minimum time span required across the retained samples before the quadratic derivative estimate is trusted. This prevents tiny timestamp spans from amplifying position noise into huge velocity/acceleration estimates.

### Fast Raw Turn-Onset Parameters

Fast onset uses the immediate raw heading-rate measurement before normal EMA smoothing. It is meant to prepare CTxy during the first one or two packets of a turn.

```python
FAST_TURN_ONSET_RAW_OMEGA = 0.07
```

Raw heading-rate magnitude required before fast onset activates. Lowering it catches turns earlier but increases sensitivity to measurement noise.

```python
FAST_TURN_ONSET_FULL_SCALE = 0.25
```

Raw heading-rate magnitude where fast-onset strength reaches one. The raw-onset strength ramps between `FAST_TURN_ONSET_RAW_OMEGA` and this value.

```python
FAST_TURN_ONSET_ALPHA = 0.25
```

Blend factor for moving CTxy omega toward raw omega during fast onset. Larger values seed CT omega faster, which can reduce turn-entry prediction lag, but can also inject noisy raw heading-rate spikes.

```python
FAST_TURN_ONSET_CT_MU_FLOOR = 0.2
```

Configured aggregate CTxy probability floor for fast onset. The implementation deliberately clips this below the normal effective-omega gate:

```python
target_ct = min(FAST_TURN_ONSET_CT_MU_FLOOR, OMEGA_MODE_PROB_MIN - 1e-3)
```

With the current values, the effective fast-onset floor is:

```text
min(0.2, 0.20 - 0.001) = 0.199
```

So raw onset can prepare CTxy and seed omega, but by itself it does not make `omega_eff` nonzero.

### Timing Parameters

```python
MIN_FILTER_DT = 0.03
```

Lower bound on measurement dt used by the estimator. Very small dt values make numerical derivative estimates noisy and process noise nearly zero.

```python
MAX_FILTER_DT = 3.0
```

Upper bound on measurement dt. Only a guard against truly pathological stalls. Do not set this below realistic stream gaps: clamping dt under the true gap compresses the target's real displacement into a shorter assumed time and scales every inferred velocity by (true gap / clamped dt) — the old `1.0` cap turned a 2 s stall into a 2x velocity spike. The prediction logic substeps the accepted interval, so multi-second propagation is numerically safe.

```python
PREDICT_MAX_SUBSTEP = 0.1
```

Maximum per-filter dynamic prediction substep. Long measurement intervals are predicted through smaller dynamics substeps for numerical stability, while IMM model mixing is still applied once per measurement interval.

```python
DT_SMOOTHING_KEEP_PREV = 0.0
```

Smoothing weight for dt. Current value disables dt smoothing, so `dt_actual = dt_meas`. If increased, the estimator would blend the previous dt with the newest clamped measurement gap. This can reduce dt jitter but can also reintroduce timing lag.

### Product-Mode Structure Parameters

```python
H_MODE_CV = "CVxy"
H_MODE_CT = "CTxy"
H_MODE_CA = "CAxy"
Z_MODE_CV = "CVz"
Z_MODE_CA = "CAz"
```

Mode labels for the horizontal and vertical factors. These are structural names, not tuning constants.

```python
PRODUCT_MODE_SPECS = (
    ("CVxy_CVz", H_MODE_CV, Z_MODE_CV),
    ("CVxy_CAz", H_MODE_CV, Z_MODE_CA),
    ("CTxy_CVz", H_MODE_CT, Z_MODE_CV),
    ("CTxy_CAz", H_MODE_CT, Z_MODE_CA),
    ("CAxy_CVz", H_MODE_CA, Z_MODE_CV),
    ("CAxy_CAz", H_MODE_CA, Z_MODE_CA),
)
```

The active six product modes. Changing this changes the estimator architecture and requires updating mode aggregation helpers, diagnostics, plots, and caller assumptions.

```python
PRODUCT_MODE_NAMES = tuple(name for name, _, _ in PRODUCT_MODE_SPECS)
PRODUCT_MODE_FIELD_SUFFIXES = tuple(name.lower() for name in PRODUCT_MODE_NAMES)
```

Derived names and CSV field suffixes for the product modes. These should normally change only as a consequence of changing `PRODUCT_MODE_SPECS`.

```python
H_MODE_ORDER = (H_MODE_CV, H_MODE_CT, H_MODE_CA)
Z_MODE_ORDER = (Z_MODE_CV, Z_MODE_CA)
H_MODE_INDEX = {mode: idx for idx, mode in enumerate(H_MODE_ORDER)}
Z_MODE_INDEX = {mode: idx for idx, mode in enumerate(Z_MODE_ORDER)}
```

Canonical ordering and lookup tables for horizontal and vertical modes. Aggregation helpers, probability initialization, transition-matrix construction, and diagnostics depend on these orders. Do not hardcode a CT index in caller code; use `aggregate_mode_probabilities()` or `ct_mode_probability()`.

```python
H_MODE_INITIAL = np.array([0.45, 0.35, 0.20], dtype=float)
Z_MODE_INITIAL = np.array([0.70, 0.30], dtype=float)
```

Initial horizontal and vertical mode probabilities. They are multiplied to form the initial six product-mode probabilities. Raising initial CTxy can make early startup more turn-ready, but it should not be used to hide steady-state mode-selection problems.

```python
H_MODE_TRANSITION = np.array([
    [0.93, 0.06, 0.01],
    [0.08, 0.90, 0.02],
    [0.07, 0.06, 0.87],
], dtype=float)
```

Horizontal Markov transition matrix. Rows are source modes and columns are destination modes:

```text
row 0: from CVxy to CVxy/CTxy/CAxy
row 1: from CTxy to CVxy/CTxy/CAxy
row 2: from CAxy to CVxy/CTxy/CAxy
```

Key tuning interpretations:

```text
CVxy -> CTxy controls how easily CT can become active from straight flight.
CTxy -> CTxy controls CT persistence once active.
CTxy -> CVxy controls how quickly the estimator leaves CT.
CAxy -> CAxy controls persistence of acceleration-heavy horizontal motion.
```

```python
Z_MODE_TRANSITION = np.array([
    [0.94, 0.06],
    [0.10, 0.90],
], dtype=float)
```

Vertical Markov transition matrix. Rows are source modes and columns are destination modes:

```text
row 0: from CVz to CVz/CAz
row 1: from CAz to CVz/CAz
```

Increasing `CVz -> CAz` helps vertical acceleration activate earlier. Increasing `CAz -> CAz` keeps vertical acceleration active longer.

```python
LEGACY_MODE_SPECS = (
    ("CV", H_MODE_CV, Z_MODE_CV),
    ("CT", H_MODE_CT, Z_MODE_CA),
    ("CA", H_MODE_CA, Z_MODE_CA),
)
```

Compatibility mapping for older three-mode callers/helpers. It is not the active estimator mode set.

### Measurement Noise Parameter

`R` is controlled by a hot-reloadable top-level variance vector:

```python
MEASUREMENT_R_DIAG = np.array([0.0225, 0.0225, 0.0225], dtype=float)
kf.R = np.diag(MEASUREMENT_R_DIAG)
```

Interpretation:

```text
X measurement variance = 0.0225 m^2 (sigma = 0.15 m)
Y measurement variance = 0.0225 m^2 (sigma = 0.15 m)
Z measurement variance = 0.0225 m^2 (sigma = 0.15 m)
```

Fit against the current message-time-stamped ~4 Hz transport, whose measured
noise floor is 1.5-3 cm sigma; see section 5 for the re-fit rationale and the
warning about noisier measurement sources.

Increasing an axis variance makes the estimator trust that measurement axis less on each update, reducing update jumps but increasing lag. Decreasing it makes the estimator follow measurements more tightly, reducing lag when measurements are clean but increasing noise and spike sensitivity.

## 23. Mathematical Appendix

This section collects the estimator equations in LaTeX form. It is meant to make the implementation easier to audit, tune, and defend.

### State, Measurement, and Noise

The shared IMM state is:

$$
\mathbf{x}
=
\begin{bmatrix}
p_x & p_y & p_z &
v_x & v_y & v_z &
a_x & a_y & a_z &
\omega
\end{bmatrix}^{T}
$$

where:

$$
\mathbf{p} =
\begin{bmatrix}
p_x & p_y & p_z
\end{bmatrix}^{T},
\quad
\mathbf{v} =
\begin{bmatrix}
v_x & v_y & v_z
\end{bmatrix}^{T},
\quad
\mathbf{a} =
\begin{bmatrix}
a_x & a_y & a_z
\end{bmatrix}^{T}
$$

Only position is measured:

$$
\mathbf{z}_k =
\begin{bmatrix}
z_x & z_y & z_z
\end{bmatrix}^{T}
=
H \mathbf{x}_k + \mathbf{r}_k
$$

with:

$$
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

and:

$$
\mathbf{r}_k \sim \mathcal{N}(0, R)
$$

Current measurement covariance:

$$
R =
\operatorname{diag}(0.0225, 0.0225, 0.0225)
$$

Larger values in \(R\) reduce measurement trust. This follows directly from the Kalman gain:

$$
K_k =
P_k^- H^T
\left(
H P_k^- H^T + R
\right)^{-1}
$$

If \(R\) increases, then the innovation covariance \(S_k = H P_k^- H^T + R\) increases, so \(S_k^{-1}\) and therefore the gain \(K_k\) decrease. That means the update moves the estimate less toward the newest measurement.

### Per-Model Kalman Update

For each IMM model \(i\), the residual/innovation is:

$$
\mathbf{\nu}_{i,k}
=
\mathbf{z}_k - H \mathbf{x}_{i,k}^-
$$

The innovation covariance is:

$$
S_{i,k}
=
H P_{i,k}^- H^T + R_i
$$

The Kalman update is:

$$
K_{i,k}
=
P_{i,k}^- H^T S_{i,k}^{-1}
$$

$$
\mathbf{x}_{i,k}^{+}
=
\mathbf{x}_{i,k}^{-}
+ K_{i,k}\mathbf{\nu}_{i,k}
$$

$$
P_{i,k}^{+}
=
(I - K_{i,k}H)P_{i,k}^{-}
$$

The model likelihood used by the IMM is the Gaussian likelihood of the innovation:

$$
\Lambda_{i,k}
=
\mathcal{N}
\left(
\mathbf{\nu}_{i,k};
0,
S_{i,k}
\right)
$$

Expanded:

$$
\Lambda_{i,k}
=
\frac{1}
{\sqrt{(2\pi)^m |S_{i,k}|}}
\exp
\left(
-\frac{1}{2}
\mathbf{\nu}_{i,k}^{T}
S_{i,k}^{-1}
\mathbf{\nu}_{i,k}
\right)
$$

where \(m=3\) because the measurement is 3D position.

The normalized innovation squared is:

$$
\operatorname{NIS}_{i,k}
=
\mathbf{\nu}_{i,k}^{T}
S_{i,k}^{-1}
\mathbf{\nu}_{i,k}
$$

If the model, \(Q_i\), \(R_i\), and timing are statistically consistent, then:

$$
\operatorname{NIS}_{i,k}
\sim
\chi^2_m
$$

where \(m=3\) for the current position measurement.

This gives a rigorous version of "compare the physical prediction with the next observation." The raw innovation:

$$
\mathbf{\nu}_{i,k}
=
\mathbf{z}_k - H\mathbf{x}_{i,k}^{-}
$$

is the physical prediction error. The NIS scales that error by what the filter expected its error covariance to be:

$$
S_{i,k}
=
H P_{i,k}^{-}H^T + R_i
$$

Interpretation:

$$
\operatorname{NIS}_{i,k} \gg m
$$

means the filter is more surprised than expected. The model variance is probably too optimistic, the measurement variance is too optimistic, the wrong model is active, or the sample is an outlier.

$$
\operatorname{NIS}_{i,k} \ll m
$$

means the filter is less surprised than expected. The assumed uncertainty is probably too conservative.

The current code logs raw innovation and model likelihoods. Adding per-model NIS to the diagnostics would make \(Q/R\) tuning more direct.

### IMM Mixing Equations

Let:

$$
\mu_{i,k}
=
\Pr(M_i \mid \mathbf{z}_{1:k})
$$

be the posterior probability of model \(i\) after measurement \(k\).

The transition matrix convention used by FilterPy here is:

$$
M_{ij}
=
\Pr(M_j \text{ at next step} \mid M_i \text{ now})
$$

Rows are source models and columns are destination models.

Before prediction, the prior probability for destination model \(j\) is:

$$
\bar{c}_j
=
\sum_i \mu_i M_{ij}
$$

The probability that model \(i\) was the source, given destination model \(j\), is:

$$
\omega_{ij}
=
\frac{\mu_i M_{ij}}{\bar{c}_j}
$$

These are the IMM mixing weights.

The mixed initial state for model \(j\) is:

$$
\mathbf{x}_{j}^{0}
=
\sum_i \omega_{ij}\mathbf{x}_{i}
$$

The mixed covariance is:

$$
P_{j}^{0}
=
\sum_i
\omega_{ij}
\left[
P_i
+
(\mathbf{x}_{i}-\mathbf{x}_{j}^{0})
(\mathbf{x}_{i}-\mathbf{x}_{j}^{0})^T
\right]
$$

After each model predicts and updates, IMM model probabilities are updated by likelihood:

$$
\mu_j^{+}
=
\frac{\bar{c}_j \Lambda_j}
{\sum_{\ell}\bar{c}_{\ell}\Lambda_{\ell}}
$$

The final IMM state estimate is:

$$
\mathbf{x}
=
\sum_j \mu_j \mathbf{x}_j
$$

and:

$$
P
=
\sum_j
\mu_j
\left[
P_j
+
(\mathbf{x}_j-\mathbf{x})
(\mathbf{x}_j-\mathbf{x})^T
\right]
$$

### Proof Note: Why Repeated Substep Mixing Was Wrong

Substepping dynamics is useful, but substepping IMM mixing is not.

Consider two models with transition matrix:

$$
M =
\begin{bmatrix}
1-\alpha & \alpha \\
\beta & 1-\beta
\end{bmatrix}
$$

For this simplified two-model case, repeated mixing contracts the difference between model-specific states. If the model states are scalar and have difference:

$$
\Delta_k = x_{1,k} - x_{2,k}
$$

then after one idealized mixing step, the difference is scaled approximately by:

$$
\Delta_{k+1}
\approx
(1-\alpha-\beta)\Delta_k
$$

After \(n\) repeated mixing steps:

$$
\Delta_{k+n}
\approx
(1-\alpha-\beta)^n\Delta_k
$$

Since:

$$
|1-\alpha-\beta| < 1
$$

the model states collapse toward each other as \(n\) grows.

That is exactly why repeatedly calling `imm.predict()` during dt substeps was harmful. For a 0.5 s measurement interval split into five 0.1 s substeps, the old implementation mixed models five times before one measurement update. That artificially erased CT/CV/CA separation. The corrected implementation does:

$$
\text{one IMM mixing step per measurement interval}
$$

then:

$$
\text{many per-filter dynamic prediction substeps}
$$

### CV Model Equations

The CV model assumes velocity is constant between updates, with random acceleration process noise.

For each axis:

$$
\begin{bmatrix}
p_{k+1} \\
v_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
$$

The corresponding random-acceleration covariance block is:

$$
Q_{\text{CV},1D}
=
\sigma_a^2
\begin{bmatrix}
\frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\
\frac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix}
$$

This is the standard discrete covariance produced by integrating white acceleration noise into position and velocity.

### CA Model Equations

The CA model assumes acceleration is part of the state and changes through jerk noise.

For each axis:

$$
\begin{bmatrix}
p_{k+1} \\
v_{k+1} \\
a_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t & \frac{1}{2}\Delta t^2 \\
0 & 1 & \Delta t \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
p_k \\
v_k \\
a_k
\end{bmatrix}
$$

With white jerk noise \(j \sim \mathcal{N}(0,\sigma_j^2)\), the covariance block is:

$$
Q_{\text{CA},1D}
=
\sigma_j^2
\begin{bmatrix}
\frac{\Delta t^6}{36} & \frac{\Delta t^5}{12} & \frac{\Delta t^4}{6} \\
\frac{\Delta t^5}{12} & \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\
\frac{\Delta t^4}{6} & \frac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix}
$$

### Product CTxy Model Equations

The implemented CT branch is `CTxy`, paired with either `CVz` or `CAz`.

Horizontal state:

$$
\mathbf{s}_{xy}
=
\begin{bmatrix}
x & y & v_x & v_y
\end{bmatrix}^{T}
$$

For scalar turn rate \(\omega\) and \(\theta=\omega\Delta t\):

$$
\mathbf{s}_{xy,k+1}
=
F_{\mathrm{CTxy}}(\omega,\Delta t)\mathbf{s}_{xy,k}
$$

where:

$$
F_{\mathrm{CTxy}}
=
\begin{bmatrix}
1 & 0 & \frac{\sin\theta}{\omega} & -\frac{1-\cos\theta}{\omega} \\
0 & 1 & \frac{1-\cos\theta}{\omega} & \frac{\sin\theta}{\omega} \\
0 & 0 & \cos\theta & -\sin\theta \\
0 & 0 & \sin\theta & \cos\theta
\end{bmatrix}
$$

For \(\omega=0\), the implementation uses the straight limit:

$$
F_{\mathrm{CVxy}}
=
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Limit Proof: CTxy Reduces To CVxy As \(\omega \to 0\)

Using:

$$
\lim_{\omega\to0}
\frac{\sin(\omega\Delta t)}{\omega}
=
\Delta t
$$

and:

$$
\lim_{\omega\to0}
\frac{1-\cos(\omega\Delta t)}{\omega}
=
0
$$

and:

$$
\lim_{\omega\to0}\cos(\omega\Delta t)=1,\quad
\lim_{\omega\to0}\sin(\omega\Delta t)=0
$$

therefore:

$$
\lim_{\omega\to0}
F_{\mathrm{CTxy}}(\omega,\Delta t)
=
F_{\mathrm{CVxy}}
$$

So the CTxy model is continuous at straight flight. This is important because small heading-rate estimates should not create a discontinuity in predicted XY motion.

#### Product Coupling With Z

The vertical component is independent at the model-selection level, not at the state-estimation level. The two CTxy product branches are:

$$
\mathrm{CTxy\_CVz}
$$

and:

$$
\mathrm{CTxy\_CAz}
$$

Their vertical transitions are:

$$
\mathrm{CVz}:
\quad
\begin{bmatrix}
z_{k+1} \\
v_{z,k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
z_k \\
v_{z,k}
\end{bmatrix}
$$

and:

$$
\mathrm{CAz}:
\quad
\begin{bmatrix}
z_{k+1} \\
v_{z,k+1} \\
a_{z,k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t & \frac{1}{2}\Delta t^2 \\
0 & 1 & \Delta t \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
z_k \\
v_{z,k} \\
a_{z,k}
\end{bmatrix}
$$

### Turn-Rate Formula From XY Motion

The measured horizontal heading is:

$$
\psi = \operatorname{atan2}(v_y, v_x)
$$

The derivative of heading is:

$$
\dot{\psi}
=
\frac{v_x a_y - v_y a_x}
{v_x^2 + v_y^2}
$$

Proof sketch:

For:

$$
\psi = \operatorname{atan2}(y, x)
$$

the total derivative is:

$$
d\psi
=
\frac{x\,dy - y\,dx}{x^2+y^2}
$$

Substitute:

$$
x=v_x,
\quad
y=v_y,
\quad
dx=a_x dt,
\quad
dy=a_y dt
$$

Then:

$$
\frac{d\psi}{dt}
=
\frac{v_x a_y - v_y a_x}
{v_x^2+v_y^2}
$$

This is why a local polynomial or smoothing spline derivative can estimate turn rate without directly differencing headings. But exact interpolation is risky: differentiation amplifies high-frequency noise. In frequency terms, if position noise contains a component \(e^{j\Omega t}\), then:

$$
\frac{d}{dt}e^{j\Omega t}
=
j\Omega e^{j\Omega t}
$$

and:

$$
\frac{d^2}{dt^2}e^{j\Omega t}
=
-\Omega^2 e^{j\Omega t}
$$

So first derivatives amplify noise proportional to \(\Omega\), and second derivatives amplify noise proportional to \(\Omega^2\).

### Fast Turn-Onset Hint Equations

Raw onset strength is:

$$
s_{\text{raw}}
=
\operatorname{clip}
\left(
\frac{|\omega_{\text{raw}}|-\omega_0}
{\omega_1-\omega_0},
0,
1
\right)
$$

where:

$$
\omega_0 = 0.06 \text{ rad/s}
$$

and:

$$
\omega_1 = 0.25 \text{ rad/s}
$$

CT omega is seeded lightly:

$$
\omega_{\text{CT}}'
=
(1-\alpha_{\text{raw}})
\omega_{\text{CT}}
+
\alpha_{\text{raw}}
\omega_{\text{raw}}
$$

with:

$$
\alpha_{\text{raw}} = 0.25
$$

CT probability is raised only to a floor:

$$
\mu_{\text{CT}}'
\le
0.199
$$

The normal effective turn-rate gate is:

$$
\mu_{\text{CT}} \ge 0.20
$$

Therefore:

$$
\mu_{\text{CT}}' < 0.20
\Rightarrow
\omega_{\text{eff}} = 0
$$

This is the key safety property: raw onset prepares CTxy internally but does not raise CTxy probability above the guidance turn-rate gate by itself.

### Command-Side Z Slew Equation

Let \(z_d[k]\) be the desired outgoing slot Z command and \(z_c[k]\) be the actually commanded Z after slew limiting.

The limiter applies:

$$
z_c[k]
=
z_c[k-1]
+
\operatorname{clip}
\left(
z_d[k]-z_c[k-1],
-r_z\Delta t,
r_z\Delta t
\right)
$$

Therefore:

$$
|z_c[k]-z_c[k-1]|
\le
r_z\Delta t
$$

and the commanded vertical rate is bounded by:

$$
\left|
\frac{z_c[k]-z_c[k-1]}{\Delta t}
\right|
\le
r_z
$$

This proves the command shaper limits Z command slew without changing the estimator state.

### Proof Note: Why Replaying Old Packets Is Dangerous

Suppose the same scalar measurement \(z\) is replayed \(m\) times with measurement variance \(R\). For a static scalar Kalman problem, posterior precision accumulates as:

$$
\frac{1}{P_m}
=
\frac{1}{P_0}
+
\frac{m}{R}
$$

That is equivalent to a single measurement with variance:

$$
R_{\text{equiv}} = \frac{R}{m}
$$

But repeated old packets are not independent measurements. They contain the same information. Replaying them makes the filter believe measurement uncertainty is \(m\) times smaller than it really is, so the filter becomes overconfident and can later react badly when fresh data arrives.

Correct alternatives are:

$$
\text{skip the bad measurement}
$$

or:

$$
\text{inflate } R \text{ for that one measurement}
$$

or:

$$
\text{shape the outgoing command without modifying estimator covariance}
$$

### dt Cap Reason

The process-noise terms grow with powers of \(\Delta t\). For example:

$$
Q_{\text{CV},p}
\propto
\Delta t^4
$$

and:

$$
Q_{\text{CA},p}
\propto
\Delta t^6
$$

Large timestamp gaps therefore cause large covariance growth and large nonlinear CTxy propagation. Capping dt prevents a communication stall from being interpreted as a long reliable prediction interval.

## 24. Changelog

### 2026-07-09 (GUI / plot)

- Removed the 3D trajectory panel from the `filterwndr.py` standalone plotter (`__main__`). It was the heaviest X-server load (3D redraw + per-frame `tight_layout()` at 10 Hz) and, alongside Gazebo, contributed to Xwayland/WSLg dropping the X connection (`xterm: fatal IO error 11`). The plotter now shows three stacked panels: per-axis error, IMM probabilities + omega, residual/jump norms. See section 16.
- Reworked `guidance_gui.py` (consumed by `simple_guided_follow.py`, `lag_pursuit_pid.py`, `pronav_runner.py`):
  - Removed its 3D "Flight View" canvas; that tab is now text telemetry (current range, closest-approach log, noise-spike log).
  - Fixed the freeze: the Tk fallback ran `mainloop()` in the GUI thread while runners drove `_root.update()` from the main thread via `gui_tick()` — two threads on one Tcl interpreter. `tick()`/`gui_tick()` are now no-ops; the GUI refreshes on its own `after()`/`QTimer` loop entirely within the background thread. (PyQt5 is broken on the current box — no `sip` — so the Tk path is what actually runs.)
  - Fixed "not initialising": the backend run is wrapped so exceptions are printed and `start()` is always released instead of silently blocking then returning a dead handle; the active backend is logged.
  - `stop()` now shuts the toolkit down on its own thread and joins.
  - Per-frame cost cut: the mode-probability plot uses persistent line artists (`set_data`) instead of `clear()` + `tight_layout()` every frame.
  - The live param editor now refuses to write back non-scalar params (dicts/lists), so it can no longer corrupt `guidance_config.py`.
- Added `imm_replay_eval.py` note references throughout (offline est-vs-truth analysis replaces the removed live 3D view).

### 2026-07-09

- Root-caused and fixed the "insane CT-switch transients" (large error spikes at every hard turn entry).
  - Method: a replay harness re-ran the exact estimator code over the logged measurements from `logs/imm_diagnostics_20260709_102459.csv` and `_102011.csv` (baseline replay reproduced the logged runs to 0.0000 m, so conclusions transfer), with ground truth from central cubic fits of the cm-accurate measurements.
  - Diagnosis: the spike is turn-entry lag followed by late catch-up, not a mixing artifact. With `R = diag(1.0, 1.0, 0.5)` the filter treated the first ~1 s of each ~0.5 rad/s turn as measurement noise: innovation built to ~3 m while CVxy stayed dominant (`mu_ct_xy` even sagged), then the turn hint force-switched the probabilities and the estimate lurched (peak position error ~1.5 m, velocity error ~6 m/s, velocity slewing faster than the plane can physically turn).
  - The measured position noise floor of the current transport (message-time stamping, ~4 Hz) is 1.5-3 cm sigma — R was ~2000x too pessimistic in variance. This was exactly the deferred §5.5 item in `intercept_stability_report.md` ("re-fit `MEASUREMENT_R_DIAG` after the timing fix").
- Changes (all hot-reloadable constants; no structural change):
  - `MEASUREMENT_R_DIAG`: `[1.0, 1.0, 0.5]` -> `[0.0225, 0.0225, 0.0225]` (sigma 15 cm, 5-10x above the measured floor).
  - `SIGMA_OMEGA_DOT`: `0.08` -> `0.3` (CT omega converges within ~2 packets of entry instead of ~6).
  - `TURN_HINT_MU_BLEND`: `0.50` -> `0.0` (the turn hint no longer rewrites IMM probabilities; likelihood competition now activates CTxy in 1-2 packets by itself. Omega seeding, omega covariance inflation, and the fast-onset floor remain active).
  - `MAX_FILTER_DT`: `1.0` -> `3.0` (clamping dt below a real stream gap scales all inferred velocities by true-gap/clamped-dt; a logged 2 s stall produced an almost exactly 2x velocity spike. Substepped prediction makes honest multi-second propagation safe).
- Replay validation on the current-pipeline logs:
  - turn-entry peak position error: 1.4-1.6 m -> 0.15-0.25 m per event (worst log event, during a 2 s stall: 3.56 m -> 0.86 m).
  - turn-entry peak velocity error: ~5-6 m/s -> ~1.3-2 m/s.
  - stall-packet velocity spike: 13.5 m/s -> ~4 m/s.
  - overall est-vs-truth position p95: 1.2 m -> 0.15-0.30 m.
  - `mu_ct_xy` 0.5-upcrossings exactly match real turn entries (no chatter); zero false CT activations on straight segments; `omega_eff` now converges to the true ~0.5 rad/s early in each turn.
  - Rejected candidates: an omega-preserving partial-mixing variant (no measurable benefit once R was correct) and fully disabling the hints (equivalent to seed-only; seeding kept for robustness).
  - Cross-check on the pre-transport-fix log `imm_diagnostics_20260707_161731.csv` (1.2 Hz, receive-time stamps, 20-40 cm effective noise): still strictly better than baseline, but with some mode wobble — that transport needs a larger R, reinforcing that R tracks transport quality.
- Consumer notes:
  - `Z_CT_FREEZE_PACKETS` / `Z_UPDATE_FREEZE_PACKETS` and the Z command slews in `guidance_config.py` were built to mask the old deaf-then-lurch transient. With the fix, freezing Z for 3 packets at CT activation discards genuinely informative Z measurements exactly when the plane starts maneuvering vertically. Recommend A/B testing the chase with `--z-ct-freeze-packets 0 --z-update-freeze-packets 0` (and possibly relaxed Z slews) before keeping them.
  - The `HeadingTurnRateEstimator` hint path is now advisory only (omega seeding); its lag no longer sets the CT activation time.

### 2026-07-08

- Implemented the position-only stability fixes from `intercept_stability_report.md` in `simple_guided_follow.py`, excluding the velocity-only terminal law and the position+velocity measurement recommendation:
  - `MavStateReader` now uses target message time (`time_boot_ms` when available) for the estimator timestamp and keeps wall time separately for stale-target detection.
  - `simple_guided_follow.py` now requests target `GLOBAL_POSITION_INT` using `MAV_CMD_SET_MESSAGE_INTERVAL` and filters target heartbeat latching by expected sysid so it does not accidentally lock onto a GCS heartbeat.
  - the runner now asserts key GUIDED/WPNAV parameters at startup (`WP_YAW_BEHAVIOR`, `WPNAV_*`, `ANGLE_MAX`) instead of trusting persisted SITL eeprom state.
  - motion-compensated `IMMLowPassFilter` is now active in the simple follower before slot generation.
  - the position-only terminal fix is the report's minimal-change variant: inside terminal range, the commanded position is pushed beyond the target along LOS so GUIDED does not enter arrive-and-stop behavior.
  - added an endgame latch (`freeze & spear`) based on estimated time-to-go; once active, the terminal position target is held instead of reacting to late estimator noise.
  - yaw lock now has a close-range hold threshold and a commanded yaw slew-rate limit.
  - enabled the existing Z command slew limiters in the default guidance config.
- Kept the estimator itself position-only. No target velocity measurement is fused into `filterwndr.py`, and the runner still uses position targets rather than the report's velocity-only terminal setpoints.

### 2026-07-03

- Ported Z-transient tuning from `filterdayeh.py` into `filterwndr.py`:
  - `H_MODE_TRANSITION` CVxy row changed from `[0.91, 0.08, 0.01]` to `[0.93, 0.06, 0.01]`. CV is now stickier and less likely to jump to CT on marginal turn evidence, reducing false CT activations that trigger Z transients.
  - `SIGMA_A_CT` raised from `1.8` to `2.0` (now equal to CV). CTxy is less assertive at activation, softening the cross-covariance pull on Z during model switches.
  - `SIGMA_OMEGA_DOT` lowered from `0.1` to `0.08`. CT omega wanders less, so activation produces a more stable turn-rate estimate and less coupled Z correction.
  - `FAST_TURN_ONSET_RAW_OMEGA` raised from `0.04` to `0.07`. Fast-turn-onset trigger fires less eagerly on raw heading-rate noise.
  - `FAST_TURN_ONSET_CT_MU_FLOOR` lowered from `0.24` to `0.2`. When fast onset does fire, it injects less CT probability.
  - `TURN_HINT_DEADBAND` widened from `0.06` to `0.08`. Smoothed turn hint ignores small heading rates, reducing eager CT bias on marginal turns.
  - Rationale: `filterdayeh.py` (a branch where the Z transient was fixed) carried these values. The combination targets the root cause — premature or over-confident CT activation — rather than only masking the symptom via command-side slew.
- Added CT-activation Z freeze in `simple_guided_follow.py`:
  - new `Z_CT_FREEZE_PACKETS = 3` and `Z_CT_FREEZE_MU_THRESHOLD = 0.20` in `guidance_config.py`.
  - CLI overrides `--z-ct-freeze-packets N` and `--z-ct-freeze-mu-threshold MU`.
  - fires on the exact packet where aggregate `mu_ct_xy` crosses the threshold upward, not one packet later.
  - implemented by snapshotting vertical state before `imm.update()`, running the update, checking the crossing, and retroactively restoring vertical state if CT just activated.
  - the remaining `N-1` packets are frozen via the existing `update_imm_preserving_vertical` path.
  - shares the `z_update_freeze_remaining` counter with the fast-turn-onset freeze; either trigger can (re)arm it.
  - diagnostic print line now includes `z_ct=<mu_ct_xy>` so the CT probability is visible alongside the freeze state.
  - distinct from `--z-update-freeze-packets`, which fires on the raw heading-rate onset edge and is kept for A/B comparison.

### 2026-05-28

- Reworked `IMMLowPassFilter` for guidance use:
  - added `IMM_LPF_MOTION_COMPENSATED` in `guidance_config.py`.
  - the LPF now predicts the previous filtered state forward before blending.
  - `filterwndr.py` passes `dt_actual`; `lag_pursuit_pid.py` passes loop `dt`.
  - this removes the plain EMA steady-state lag for constant-velocity target output.
- Added configurable yaw lock for guidance:
  - `YAW_LOCK_ENABLED` in `guidance_config.py` locks commanded yaw to LOS from pursuer to predicted target.
  - `simple_guided_follow.py` clears the yaw-ignore bit only when yaw lock is enabled and leaves yaw-rate ignored.
  - `lag_pursuit_pid.py` uses the predicted target position for LOS yaw and zeros yaw-rate.
  - `velocity_control.py` ignores all body-rate fields in `SET_ATTITUDE_TARGET` while yaw lock is enabled.
- Reworked `lag_pursuit_pid.py` thrust allocation:
  - clamps vertical specific thrust to the non-inverted quad envelope.
  - adds `LAG_PURSUIT_MIN_LIFT` to preserve lift and yaw authority during Z spikes.
  - scales XY acceleration into the remaining `MAX_THRUST` and `MAX_TILT_DEG` budget before attitude conversion.
  - adds `TelemetryLogger.step_count`, which the lag-pursuit runner already expected for console throttling.
- Added `simple_guided_follow.py` Z update freeze:
  - `Z_UPDATE_FREEZE_PACKETS = 2` freezes vertical correction for the first two target packets after a fast raw turn onset.
  - XY still updates from the measurement while Z/VZ/AZ are restored to their predicted values.
  - CLI override: `--z-update-freeze-packets N`; use `0` to disable.
  - `OOSM_IMM_Tracker.process_delayed_measurement()` also supports the freeze for delayed-update callers.
  - `lag_pursuit_pid.py` enables the same freeze window on fast turn-onset edges.
- Reworked `HeadingTurnRateEstimator` to estimate `omega` with causal numerical differentiation:
  - keeps a rolling timestamped XY position history.
  - fits a causal local quadratic at the newest sample.
  - computes omega from \( (v_x a_y - v_y a_x)/(v_x^2 + v_y^2) \).
  - falls back to heading differencing until enough history exists.
  - added live-tunable `OMEGA_DIFF_*` parameters.

### 2026-05-26

- Added dynamic runtime reload for estimator tuning parameters:
  - `filterwndr.py` now parses an allow-list of parameter assignments from the saved source file when its modification time changes.
  - runtime filters refresh `R`, `Q`, dt-dependent dynamics, and the IMM transition matrix from the current values.
  - `simple_guided_follow.py` now uses the estimator's dynamic dt clamp.
- Added a tunable parameter reference for the top-level constants in `filterwndr.py`, including process noise, CT omega gates, turn-hint parameters, fast raw onset, timing, product-mode priors/transitions, legacy constants, and inline measurement `R`.
- Updated project/context markdown files so they no longer present the old three-model IMM as current.
- `context_transfer.md`, `teknofest_drone_chat_context.md`, and `README.md` now describe the six-mode product IMM and aggregate `mu_ct_xy` usage.

### 2026-05-18

- Added an interpretation note for synchronized innovation and update jump:
  - \(\Delta \hat{x}=K\nu\), so correlation is expected.
  - innovation is the prediction/measurement disagreement.
  - update jump is the gain-scaled correction, not the original cause of the disagreement.
- Implemented the six-mode product IMM:
  - modes are `CVxy_CVz`, `CVxy_CAz`, `CTxy_CVz`, `CTxy_CAz`, `CAxy_CVz`, `CAxy_CAz`.
  - the Markov matrix is now a real 6x6 product matrix built from horizontal and vertical transition matrices.
  - CT probability now means aggregate `mu_ct_xy = mu_ctxy_cvz + mu_ctxy_caz`.
  - omega hints seed both CTxy branches and preserve the current vertical marginal.
  - diagnostics now log horizontal marginals, vertical marginals, all six product-mode probabilities, and per-mode likelihoods.
  - the live probability plot now shows `CVxy/CTxy/CAxy` plus `CVz/CAz`.
  - `simple_guided_follow.py` now uses aggregate CTxy probability for Z switch detection instead of `imm.mu[1]`.
- Added a LaTeX-heavy mathematical appendix:
  - state and measurement equations
  - Kalman update and likelihood equations
  - IMM mixing and probability update equations
  - CV/CA/CTxy dynamics and process-noise covariance formulas
  - proof note for why repeated substep IMM mixing collapses model separation
  - CTxy-to-straight-motion limit proof as \(\omega \to 0\)
  - turn-rate formula derivation from XY velocity/acceleration
  - proof note for why exact derivative methods amplify high-frequency noise
  - fast raw turn-onset equations and safety gate proof
  - command-side Z slew bound proof
  - stale-packet replay overconfidence proof
  - dt-cap growth-rate explanation
- Added a tuning note connecting the user's "Euler prediction versus next observation" idea to the existing Kalman innovation and the formal NIS consistency metric.
- Added a concrete fix plan for turn-onset Z spikes:
  - immediate command-side Z slew limiter usage
  - per-update Z NIS gate / adaptive \(R_z\)
  - longer-term vertical filter decoupling from horizontal IMM model switching
- Evaluated a constrained 3D CT approach earlier in the turn discussion, but superseded it with the product IMM:
  - the product model better represents cases where XY is turning but Z is CV.
  - the legacy `fx_ct()` helper and `SIGMA_J_CT_3D` alias remain in code for compatibility, but `setup_imm_filter()` now constructs CTxy product filters.
- Added and then implemented an architecture note evaluating three independent axis-wise IMMs:
  - pure X/Y/Z independent IMMs are not recommended because CT is a coupled-plane model.
  - upward helix is better represented by a factorized horizontal IMM plus vertical IMM.
  - the chosen implementation is the six-mode product IMM rather than three independent 1D IMMs.

### 2026-05-09

- Created this estimator reference document.
- Documented the initial 10D CV/CT/CA IMM structure.
- Documented timestamp-based prediction and substep prediction.
- Documented turn-rate hint logic.
- Documented omega stabilization and effective omega.
- Documented then-current `R = diag([3.0, 3.0, 2.0])`.
- Documented CT vertical-acceleration patch:
  - At that stage, CT became horizontal coordinated turn plus vertical constant acceleration.
  - At that stage, CT Z process noise used jerk covariance through `SIGMA_J_CT_Z`.
- Fixed CT persistence issue:
  - `predict_imm_over_dt()` now applies IMM model mixing once per measurement interval, then substeps only the individual filter dynamics.
  - This prevents repeated substep-level mixing from washing CT back into CV before the next measurement arrives.
  - Transition matrix changed from `CT -> CT = 80%` and `CT -> CV = 18%` to `CT -> CT = 90%` and `CT -> CV = 8%`.
- Added omega-estimation note:
  - Exact spline interpolation is not recommended for online `omega_hint` because derivative estimates can amplify noisy measurements and centered splines add delay.
  - A causal smoothing spline or local polynomial derivative over the last 5-7 valid samples is the better direction if the finite-difference heading-rate estimate remains noisy.
- Added guidance note for Z transients during CT switches:
  - `simple_guided_follow.py` ignores explicit yaw/yaw-rate fields, so spin during yaw-lock behavior is likely from autopilot/controller yaw policy reacting to jittery setpoints.
  - Prefer command-side Z/VZ shaping and yaw rate limiting before using `R_z` as a jitter suppressor.
  - At that stage, a longer-term estimator-side option was to decouple vertical output from horizontal model switching.
- Added optional model-switch-only Z command slew limiter in `simple_guided_follow.py`:
  - Disabled by default with `--z-switch-slew-rate 0.0`.
  - Opens a short slew window after a CT probability jump/crossing.
  - Rate-limits outgoing slot Z only during that window and only if the outgoing Z jump exceeds the configured threshold.
  - Existing latest log showed the largest Z jumps did not coincide exactly with CT switching, so this is a targeted control-smoothness experiment rather than a complete Z outlier fix.
- Added follow-up log interpretation for pre-switch turn onset:
  - In the latest log, raw XY heading rate begins rising before `omega_hint` and aggregate CT probability react, so a CV-dominant pre-CT transient is plausible.
  - Across recent logs, many Z jumps still have no raw horizontal turn evidence, so a separate Z outlier/command smoother remains necessary.
  - This motivated a fast raw-heading-rate turn-onset detector that can open the Z slew window before the CT model probability switches.
- Implemented fast raw turn-onset handling:
  - `HeadingTurnRateEstimator` now exposes `raw_omega`, `speed_xy`, and `fast_onset_strength`.
  - `apply_fast_turn_onset_hint()` lightly seeds CTxy omega and raises CTxy probability only to a floor below the normal `omega_eff` gate.
  - Diagnostic CSV now logs `raw_omega`, `raw_turn_strength`, and `raw_speed_xy`.
- Extended `simple_guided_follow.py` Z command limiting:
  - Turn-window slew can now open on CT probability switch or fast raw turn onset.
  - Added separate always-on Z outlier slew via `--z-outlier-slew-rate` and `--z-outlier-jump`.
  - Added `Z_SWITCH_*` and `Z_OUTLIER_*` defaults to `guidance_config.py`; all new limiters remain disabled by default.
