# FOLLOW notes — ArduPilot FOLLOW method: running tests yourself

> This file is the run note for branch `guidance_allstar/tracking_guidance.py` in branch **the archived ArduPilot FOLLOW configuration**. For handle MPC: **`MPC_NOTES.md`**. Design rationale for the method, ArduPilot source analysis, and measurements: **`ARDUPILOT_TRACKING.md`** (this file is its "how do I run" summary).  Stack, task launch, mount, and media buttons are **exactly the same as MPC** — except the visual guidance process changes.  **[GIMBAL BRANCH UPDATE 2026-08-05]** This note is for the the archived ArduPilot FOLLOW configuration branch; **Vertical geometry is different in the `gimbal` branch**. There the camera is not fixed to the body, but on a self-stabilizing **physical single axis (tilt) gimbal** (camera world pitch max 0.65° while the body is tilted ±35°). There is no "mounting angle" button; `YILDIZ_TILT = atan(down/back)` determines the vertical axis (`scripts/standoff_geom.sh`, tool `tools/set_tilt.py`). The following numbers `--back/--down` and `--ofs-backward/--ofs-down` are values for the body-fixed period; If two branches merge, any metrics based on vertical channel and pitch must be re-derived. Detail: `GIMBAL_NOTES.md`.

```bash
# =========================== WHERE IT WORKS =====================
cd /path/to/multicopter_to_fixed_wing_guidance     # ALL commands from repository root
# FOLLOW implementation: guidance_allstar/tracking_guidance.py

# =========================== STACK ============================
./yildizlar_guidance.sh --headless    # A flag needs to be added for --iris and --hummingbird models.
# If you want GUI, without --headless
./yildizlar_guidance.sh --stop

# ===================== PUT VEHICLES INTO MISSION ======================
# --plan OPTIONAL (default target_ellipse). WRITING CORNER BRACES.
python3 tools/launch_mission.py --drones 1 --drone-alt 60
python3 tools/launch_mission.py --drones 1 --drone-alt 60 --plan missions/target_straight.plan

# ====================== POSITIONED GUIDANCE ========================
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6

# ================== VIDEO GUIDANCE (FOLLOWING) ====================
# Stands up WITH the positioned, Redis 'command_authority'='visual'
# It will NOT send commands until it happens.
cd guidance_allstar && python3 tracking_guidance.py
# Ports SAME as MPC (14654/14604) -> DOES NOT WORK WITH MPC AND FOLLOWING AT THE SAME TIME.

# ====================== FULL TRIAL (single command) ==================
DURATION=360 VISUAL_GUIDANCE="tracking_guidance.py" PLAN=missions/target_infinity.plan tools/scenario.sh

# -----ablations (ARDUPILOT_TRACKING.md section 4 and 6) -----
VISUAL_GUIDANCE="tracking_guidance.py --speed-source p"    # pure mode_follow
VISUAL_GUIDANCE="tracking_guidance.py --braking ap"          # AP 'stay alongside' brake
VISUAL_GUIDANCE="tracking_guidance.py --law poscon"      # Copter >= 4.5 direction law
VISUAL_GUIDANCE="tracking_guidance.py --acceleration-shape 0"     # shaping off
VISUAL_GUIDANCE="tracking_guidance.py --miss-source area_value" # VISUAL REFEREE (rangeless miss)

# ================= OFFLINE TESTS (no simulator needed) ==============
cd guidance_allstar && python3 tracking_test.py       # 55/55
```

## This arm has unique buttons

| button | what does |
|---|---|
| `--law classical\|poscon` | FOLLOW direction law version |
| `--kp` | position error gain |
| `--braking disabled\|ap\|range_value` | terminal brake policy |
| `--speed-source ceiling_value\|p` | speed command source (ceiling = stroke lever) |
| `--acceleration-shape` | acceleration shaping coefficient |
| `--ofs-backward / --ofs-down` | standoff offset (back/down equivalent of MPC) |
| `--yaw-p`, `--no-yaw` | yaw channel |
| `--miss-source range_value\|area_value` | scorer: telemetry range or bbox area? |
| `--miss-time-source straight\|progress` | progress clock (does not sound timeout when shutting down) |
| `--miss-time-timeout` | miss timeout [s] |

## diagnostic log

`guidance_allstar/logs/tracking_diagnostic_*.csv` — columns **intentionally overlapping `mpc_diagnostic`** (`state_value`, `impact_value`, `range_value`, `range_rate_value`, `best_candidate`, `cmd_*`, `vibe`, `hit_value`), so that the two methods are compared with the same tools: `tools/compare_results.py`, `tools/trial_summary.py`.

## Common pitfalls (same as MPC)

- **Don't forget to start the video process.** If you forget, the dead-man switch (`visual_alive`) will block the transition and a warning will appear in the bbox.log. Without this door, the vehicle would remain without command (fault 2026-08-05, `mpc_memory.md`).
- **Single-image method** — MPC and TRACK use the same ports.
- **The second client is not connected to MAVLink ports while running.**
- **One simulation at a time**; put the runs in order.
- Running health: `simtime_ratio` should be 1.00; "SIMULATION BEHIND" = run invalid.
