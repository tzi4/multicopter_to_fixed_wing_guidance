# `pronavwndr2.py` — Variable Naming Legend

Quick-reference for every abbreviation and naming convention used in the guidance code.

---

## Coordinate Frame

All positions and velocities are in the **NED (North-East-Down)** frame.

| Axis | Direction | Sign convention |
|------|-----------|-----------------|
| X | North | + = north |
| Y | East | + = east |
| Z | Down | + = below home altitude |

---

## Prefix Conventions

| Prefix | Meaning | Example |
|--------|---------|---------|
| `p` | **Pursuer** (our quad) | `px`, `pvx` |
| `t` | **Target** (fixed-wing) | `tx`, `tvy` |
| `r` | **Relative** (target − pursuer) | `rx`, `ry`, `rz` |
| `v` | **Velocity** | `pvx` = pursuer velocity X |
| `a` | **Acceleration** | `acx` = acceleration cmd X |
| `d` | **Delta** (difference) | `dvx` = change in vx |
| `vc` | **Velocity command** (legacy) | `vcx`, `vcy`, `vcz` |
| `cmd` | **Commanded** (output) | `cmd_acx`, `cmd_roll` |
| `_` (leading) | **Private** class attribute | `_prev_cmd`, `_miss_detected` |

---

## Suffixes

| Suffix | Meaning | Example |
|--------|---------|---------|
| `x`, `y`, `z` | NED component | `rx` = relative-north |
| `_m` | Value in **metres** | `range_m`, `alt_m` |
| `_mag` | **Magnitude** (scalar length of a vector) | `pvmag`, `acmag`, `dv_mag` |
| `_cmd` | **Commanded** value | `roll_cmd`, `yaw_cmd` |
| `_deg` | Value in **degrees** | `max_turn_deg`, `roll_deg` |
| `_ef` | **Earth-Frame** | `roll_ef`, `pitch_ef` |
| `_raw` | **Unmodified** version | `vrel_raw_x` |

---

## Core Geometry Variables

| Variable | Full Name | Definition |
|----------|-----------|------------|
| `rx, ry, rz` | Relative position | `target_pos − pursuer_pos` (LOS vector) |
| `dist` / `range_m` | Range | `‖R‖` — distance to target |
| `vrel_x/y/z` | Relative velocity | `target_vel − pursuer_vel` (may include accel compensation) |
| `vrel_raw_x/y/z` | Raw relative velocity | `target_vel − pursuer_vel` (always unmodified) |

---

## PN Law Variables

| Variable | Full Name | What it is |
|----------|-----------|------------|
| `cx, cy, cz` | Cross product | `R × V_rel` (intermediate for LOS rate) |
| `omega_x/y/z` | LOS rate vector | `Ω = (R × V_rel) / ‖R‖²` |
| `omega_mag` | LOS rate magnitude | `‖Ω‖` in rad/s |
| `range_rate` | Range rate | `(R · V_rel) / ‖R‖` — how fast range is changing |
| `closing_velocity` | Closing velocity | `Vc = −range_rate` — positive means getting closer |
| `acx, acy, acz` | NED accel command | `a = N × Vc × (Ω × u_LOS)` |
| `acmag` | Accel magnitude | `‖a_cmd‖` in m/s² |
| `navigation_constant` / `N` | Nav constant | PN gain (typically 3–5) |
| `n_eff` | Effective nav constant | `N` after logarithmic time-to-go decay |
| `t_go` | Time-to-go | `range / closing_velocity` [s] — estimated time to intercept |

---

## Attitude Control Variables

| Variable | Full Name | Description |
|----------|-----------|-------------|
| `roll_ef` | EF Roll | Earth-Frame roll angle in radians |
| `pitch_ef` | EF Pitch | Earth-Frame pitch angle in radians |
| `yaw_rate_ef` | EF Yaw Rate | Earth-Frame yaw rate in rad/s (tracks velocity vector) |
| `thrust_req` | Specific Force | Required thrust magnitude in m/s² (NED Z compensated) |
| `_current_yaw_cmd`| Heading State | Absolute yaw heading integrator for Earth-Frame commands |

---

## Telemetry Logging Formats (`drone_telemetry.log`)

### Header: `PURS`
| Column | Meaning | Unit |
|--------|---------|------|
| `RNG` | Range to target | metres |
| `VC` | Closing velocity | m/s |
| `P_POS` | Pursuer Position | [N, E, D] metres |
| `P_VEL` | Pursuer Velocity | [N, E, D] m/s |
| `T_POS` | Target Position | [N, E, D] metres |
| `T_VEL` | Target Velocity | [N, E, D] m/s |
| `T_ACC` | Target Accel | [N, E, D] m/s² (Estimated) |
| `CMD_A` | Accel Command | [X, Y, Z] m/s² (NED) |
| `CMD_ATT`| Attitude Command| [Roll, Pitch, YawRate] (Degrees & Deg/s) |

---

## Tail-Chase Heuristic Variables

| Variable | Meaning |
|----------|---------|
| `nx, ny, nz` | Unit normal to the plane of `V_pursuer` and `R` |
| `nm` | Magnitude of that normal vector |
| `heuristic` | Scale factor for the corrective turn |
| `dx, dy, dz` | Corrective acceleration: `n × V_pursuer × heuristic` |

---

## Rate Limiting & Speed Control

| Variable | Meaning |
|----------|---------|
| `_prev_cmd` | Previous commanded velocity (legacy integration) |
| `s_turn` | Scale factor for horizontal turn-rate clamping |
| `s_clamp` | Scale factor for PN acceleration clamping |

---

## Miss Detection & Re-engagement

| Variable | Meaning |
|----------|---------|
| `_prev_range` | Range on the previous step |
| `_min_range_seen` | Closest approach distance so far |
| `_miss_detected` | `True` after flyby detected |
| `facing_angle_deg` | Angle between velocity vector and LOS |
| `pursuit_speed` | Commanded speed during pure-pursuit re-engagement |

---

## Config Constants (from `guidance_config.py`)

| Constant | Type | Meaning |
|----------|------|---------|
| `NAV_CONSTANT` | float | PN gain `N` |
| `MAX_PN_ACCEL` | float | PN acceleration output clamp (prevents 90° tilts) |
| `PURSUER_KP` | float | P-gain for Pursuit mode speed tracking |
| `MAX_TURN_DEG` | float | Turn-rate cap in °/s |
| `SEND_RATE_HZ` | int | MAVLink send frequency (typically 10-20 Hz) |

---

## File Map — `pronavwndr2.py`

| Lines | Section |
|-------|---------|
| 1–11 | Imports & global constants (`hit_req_range`) |
| 16–86 | **Math Helper Functions** (`_compute_pn_acceleration`, etc.) |
| 88–315 | **`compute_guidance_command()`** — core PN logic (accel output) |
| 315–end | **`GuidanceLoop`** — MAVLink state handler + EF Attitude integrator |

---

## Naming Pattern Summary

```
[entity][quantity][component]
   │        │         │
   │        │         └─ x / y / z / mag / ef
   │        └─ v(velocity), a(accel), r(relative pos)
   └─ p(pursuer), t(target), cmd(commanded)
```

Examples: `pvx` = pursuer velocity X, `cmd_roll` = commanded roll, `acmag` = acceleration command magnitude.
