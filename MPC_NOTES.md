# MPC notes — MPC method: run the tests yourself (2026-08-11 current)

> **CURRENT MAIN CANDIDATE (2026-08-11):** Positional guidance establishes the approach, then a single **LOS/PN law** takes over after 5 consecutive sufficiently large bbox observations. See **`LOS_HARDWARE_NOTES.md`** for daily operation and the hardware sequence. This file records the pure-MPC history. `HYBRID_NOTES.md` records the MPC→LOS comparison branch. In that hybrid, `hybrid_guidance.py` uses `mpc_guidance.py` outside 18 m and `terminal_los_guidance.py` inside 18 m. MPC therefore remains the hybrid's mid-range component.

> This file is the note for branch **MPC** (`guidance_allstar/mpc_guidance.py`). For ArduPilot FOLLOW branch: **`TRACKING_NOTES.md`** (branch: the archived ArduPilot FOLLOW configuration). The common stack/media sections are the same in both files.  **READ FIRST — 2026-08-05 malfunction:** If you forget to START the visual guidance process, the decision maker transfers authority to visual guidance and NO ONE commands the vehicle. The symptom is deceptive: it looks like "MPC is shaking and not tracking the target at all" but MPC is not running at all. Now that there is a dead-man switch (`visual_alive`), the switch is blocked and the following drops in bbox.log: `[DECISION] visual controller missing ... Start the visual-guidance process.`

```bash
# ====================== WHERE IT WORKS =======================
# ALL commands run from the repository root (except those with their own `cd` in them):
cd /path/to/multicopter_to_fixed_wing_guidance
# Lines that say `cd guidance_allstar && ...` assume that they are entered from the root.

# =========================== STACK ============================
YILDIZ_VIDEO=1 ./yildizlar_guidance.sh --robofly --headless    # current test tool + bbox video
# If you want GUI, without --headless (including QGC)
./yildizlar_guidance.sh --stop

# ===================== PUT VEHICLES INTO MISSION ======================
# plan default target_ellipse; others: target_straight / target_circuit / target_wanderer
# target_straight = ACTUALLY STRAIGHT, 200 km north, no DO_JUMP/RTL (2026-08-05:
#   corrected from the former rectangular circuit, which contradicted the name;
#   target_circuit already covers circuits). At 20 m/s the route takes 2.8 hours.
#   A 360 s run uses only %3.6 of it, so the target never turns or enters RTL.
# target_infinity = same idea, 40 km (shorter straight route)
# ATTENTION: the following LINE --plan is OPTIONAL. If you don't, target_ellipse will run.
# Do not type the square brackets. They mark an optional argument.
python3 tools/launch_mission.py --drones 1 --drone-alt 60
# ...or select plan (without brackets):
python3 tools/launch_mission.py --drones 1 --drone-alt 60 --plan missions/target_straight.plan

# ############################################################
# # RUNNING ORDER — TWO SEPARATE TERMINALS, BOTH REQUIRED #
# ############################################################
# [1] POSITION GUIDANCE (approach):
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
# [2] VISUAL GUIDANCE (takeover) — do not omit this:
cd guidance_allstar && python3 hybrid_guidance.py
#
# If [2] is not running, visual guidance cannot take over. The dead-man key
# ('visual_alive') blocks handover. bbox.log reports:
#   [DECISION] NO video controller... forgot to initialize?
# DIAGNOSIS: no guidance_allstar/logs/visual_hybrid_*.csv means the process
# has not started, because it opens the file BEFORE waiting for authority.
# This error was repeated THREE TIMES on 2026-08-05; The symptom is deceiving:
# It looks like "MPC flickers and doesn't track the target at all".

# ============ APPROACH LAW (phase BEFORE handover) ============
# THERE ARE TWO OPTIONS (2026-08-05). The difference is FUNDAMENTAL:
#   --approximation slot (DEFAULT, proven): standoff BEHIND target
#     slot; It does NOT aim at the target. It establishes TAIL geometry -> transfer from there.
#   --approximation intersection (formation_KILLER ATTACK law): aim at the predicted
#     target position and extend 50 m BEYOND the intersection without braking.
#     Handover generally occurs in CROSSING or HEAD-ON geometry.
#     WARNING: historical WORST case of cross-overhead MPC (bearing-course ≥45°
#     all 11 of 11 such encounters missed). q_area/horizon fixes helped offline,
#     NOT VERIFIED in sim. That's why it's not the default.
#   Buttons: --intersection-speed 24 --intersection-terminal 45 --intersection-overshoot 50
# NOTE: formation_KILLER.py ITSELF does not run (no config.py, swarm
# requires controller + keyboard/waypoints); only the LAW was moved.

# ====================== POSITIONED GUIDANCE ========================
# CHANGED: --yaw-lock is now the DEFAULT (grants scenario.sh; turns off YAW_LOCK=0).
# When it was off, the target would not enter the frame on a straight route and the detection would drop to %2.7.
# Estimator window is disabled by default. Use --gui for logs, not flight (GUI overhead).
#
# TWO APPROACH OPTIONS (phase before handover):
python3 simple_guided_follow.py --approximation slot      # DEFAULT, from behind (available)
python3 simple_guided_follow.py --approximation intersection   # formation_KILLER ATTACK law
#   buttons: --intersection-speed 24 --intersection-terminal 45 --intersection-overshoot 50
#
# full version (with default approach):
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6
# CHANGED: 25/6. In GIMBALLED/0° setup, down is NOT DERIVED, it is given manually
# (derived down=round(back*tan(mount+trim)) returns negative at mount=0 → puts the copter ON the target)
# [GIMBAL BRANCH UPDATE: this is no longer a temporary fix, it's the RULE. down/back
#  task design; camera angle is derived from IT (YILDIZ_TILT=atan(down/back)).
#  The old mount-based version survives only with YILDIZ_LEGACY_DERIVATION=1.]

# ====================== VIDEO GUIDANCE =======================
# Start it alongside position guidance. It sends no commands until Redis 'command_authority'='visual'.
cd guidance_allstar && python3 hybrid_guidance.py     # current: MPC outside 18 m, PN/LOS inside
# Pure-MPC comparison handle: python3 mpc_guidance.py
# Ports: positioned fighter 14652 / target 14603 ; DISPLAY hunter 14654 / target 14604 (the same port cannot be connected twice)

# ====================== FULL TRIAL (single command) ==================
# takeoff → positioned → HYBRID video takeover → video → aim → summary. Do not set AIM: aim=0 is intentional
YILDIZ_DRONE_MODEL=robofly DURATION=360 CONTROL_WAIT_S=20 VISUAL_GUIDANCE="hybrid_guidance.py" PLAN=missions/target_ellipse.plan METHOD=hybrid tools/scenario.sh
#   VISUAL_GUIDANCE= empty → position guidance only | METHOD=xxx → video/folder label
#   BACK=25 DOWN=6 → override standoff (keep COMPATIBLE with set_mounting.py)
#   YAW_LOCK=0 → disable yaw-lock in position guidance
# output: run/trials/<method>_<route>_<stamp>/ + videos/<method>_<route>_<stamp>.mp4

# 2026-08-10 LIVE A/B RESULT: current candidate mid-phase MPC + terminal PN/LOS hybrid.
# 20 s standby on fresh stack prevents takeoff/prediction altitude jump.
# Pure terminal comparison branch: VISUAL_GUIDANCE="terminal_los_guidance.py"
# Scan cell example:
# VISUAL_GUIDANCE="hybrid_guidance.py --n-pn 5 --transition-range 20 --climb-speed-max 2.0"
# Conclusion/decision and next scan: TO_TEST.md → LIVE T2 ASSESSMENT

# Running health: "CYCLE SLOW" = CPU congested; "SIMULATION BEHIND" = running INVALID
# live: loop=..Hz sim=.. CSV: loop_dt_meas_s, simtime_ratio (1.00 good, 0.74 garbage)

# =================== INSTALLATION angle (0° — NEW) ==================
# [GIMBAL BRANCH UPDATE 2026-08-05: this block is HISTORICAL. camera now
#  in a self-stabilizing PHYSICAL single axis (tilt) gimbal; "mounting angle"
#  THERE IS NO button left (SDF cam pose pitch 0 should remain — typing angle there
#  superimposes silent offset on top of the elevation commanded by the gimbal).
#  Dependency REVERSED: down/back free mission design, camera angle
#  derives from it → YILDIZ_TILT = atan(down/back) (earth elevation, + up).
#  NEW TOOL: python3 tools/set_tilt.py --down 6 --back 25 [--apply-value]
#  The WRITE path of set_mounting.py has been closed; Only --display-value works.
#  Detail: GIMBAL_NOTES.md]
# The mount is NOT ONE button: SDF + virtual gimbal + standoff THREE must switch together.
python3 tools/set_mounting.py --display-value                       # the current situation
python3 tools/set_mounting.py --mount 0 --back 25 --down 6   # dry running (NOT WRITING)
python3 tools/set_mounting.py --mount 0 --back 25 --down 6 --apply-value   # + runs static test
python3 tools/set_mounting.py --restore-backup                      # back with git checkout
# RULE: assembly ≈ target LOS rise at time of IMPACT. LOS→0 → mounting 0 in rear equal-altitude impact.
# Steep nose-down in real hardware trunk dash (18 m/s → −34°) → GIMBAL is a must there.

# ============ GIMBAL STATIC TEST / AIM / CALIBRATION ===========
python3 yildizlar_gimbal.py --test [--aim -27]    # NAME CHANGED (formerly: virtual_gimbal.py)
python3 tools/measure_aim.py --duration-value 300 --label-value trial   # scenario aim=0 while running; ey median = aim
python3 tools/camera_calibration.py --port 14551 --north-value 700 --east-value 250 --alt 60 --scan-value2 8,130,150 --duration-value 380

# ===================== MANUAL COMMAND / SUMMARY ========================
python3 tools/swarm_command.py state_value
python3 tools/swarm_command.py speed-test --id 1 --distance 3000 --speed-value 35
python3 tools/swarm_command.py target-takeoff | drone-takeoff --id 1 --alt 60 | ambush | tracking_value | speed-lock
python3 tools/trial_summary.py run/trials/<folder>
python3 tools/compare_results.py [--method-value mpc los pid] [--csv report.csv]   # NEW: method comparison
#   area_px2 = bbox area vertex [px², LINEAR, PRIMARY reward] | a_speed90 = area growth rate (SECONDARY)
#   ex/ey_rms = centering | IMPACT evidence: min_m<3 + vibe bounce
#   ATTENTION: vibe top is GROUND CONTACT if target is away + pos_z~0 (separates vibe_range_m/vibe_pos_z)
# CAUTION: trial_summary "loop example" ~1/10 of reality; GEOMETRY section reads the NEWEST csv and not the given folder

# ================= OFFLINE TESTS (no simulator needed) ==============
cd guidance_allstar && python3 mpc_test.py && python3 los_test.py && python3 pid_test.py

# ====================== ENV BUTTONS ===========================
#   YILDIZ_MOUNT=0 (Must be the SAME as model.sdf — set_mounting.py guarantees)
#   YILDIZ_PITCH_TRIM=-2.5   YILDIZ_BACK=25   YILDIZ_DOWN=6 (MANUAL with the 0° mount)
#   [GIMBAL BRANCH UPDATE: NEW BUTTON **YILDIZ_TILT** — camera world
#    elevation [deg, + = up] is derived from an empty setting = atan(DOWN/BACK). Vertical
#    This now defines the axis. YILDIZ_MOUNT and YILDIZ_PITCH_TRIM
#    no longer control the vertical channel. They are exported only for frozen
#    body-fixed branches. Single source: scripts/standoff_geom.sh]
#   YILDIZ_AIM: leave blank = derive (aim=-atan(down/back), AimTrim ±6°); scenario.sh 0 constants
#   YILDIZ_GUI=1 estimator window | YILDIZ_VIDEO=1 | YILDIZ_VIDEO_LABEL=mpc_straight
#   YILDIZ_DRONES=1..5 (+YILDIZ_WORLD=worlds/swarm.world) | YILDIZ_TARGET_PLAN
#   YILDIZ_CV_THREADS=1 | YILDIZ_STREAMRATE=20 | YILDIZ_TARGET_STREAMRATE=15 | YILDIZ_BBOX=0 (without detector)
#   --- cycle gates (bbox_to_redis.py) — NEW ---
#   YILDIZ_COV_TRANSITION=2.0 min horizontal coverage [%] for pass (~40 m)
#   YILDIZ_COV_KAL=0.3 video retention threshold [%]
#   YILDIZ_TRANSITION_RANGE=60 maximum estimator range for switching [m]
#   YILDIZ_TRANSITION_AREA_PCT=0 FIELD criteria (0=off, legacy behavior). If p is given
#     transition threshold area of the rectangle of the frame (p% x p%). MEASURED range
#     equivalent (bbox_area ~ 4.65e5/r^2, n=1945 real frame):
#       p=2 -> 369 px^2 -> 35 m | p=3 -> 829 -> 24 m | p=4 -> 1475 -> 18 m
#       p=5 -> 2304 px^2 -> 14 m | p=6 -> 3318 -> 12 m | p=8 -> 5898 -> 9 m
#     INDEPENDENT of range encounter type (tail 14.3/cross 13.8/
#     head-to-head 14.1 m). The field sees both axes; horizontal coverage single
#     The axis measures and scrolls with the aspect ratio of the target (median 2.33).
```

## VERIFICATION list when assembly/geometry changes

1. `python3 yildizlar_gimbal.py --test` → "ALL STATIC TESTS PASSED".
2. An ellipse run → `summary_value.txt` GEOMETRY: "target - axis" should be small; detected %80+.
3. Detection in `<15 m` band increased from %0 (endgame blindness of old 30°) — CSV with image.
4. Crash/ground contact 0 (min altitude + vibe context).
5. Does `bbox.log` have "MODE CHANGED" (position→visual cycle triggered).

## Current status

- **Mount 0°**, standoff **back 25 / down 6** (LOS +13.5° → 0° on impact, both in frame). Actually pitch-servo **gimbal** will be used; 0° ≈ ideal gimbal as the copter does not tilt in sim.
  > **[GIMBAL BRANCH UPDATE 2026-08-05]** The sentence "0° ≈ ideal gimbal because the copter doesn't tilt in the Sim" has been REFUSED — twice. (a) The copter is tilting: these notes' own measurements show stem pitch %5 −42.5° / %95 +39.4°, so the fixed camera 0° was NOT the ideal gimbal; This was the result of the detection "0 highly fixed camera cannot see the target, MPC remains blind" (see FOLLOW_UP.md). (b) There is now a **real** gimbal: the camera of the 5 drone in the sim is on a self-stabilizing physical single-axis tilt gimbal. Measured in flight: camera pitch relative to the world **max |0.65°|** while fuselage yaws −35.4…+35.2°. So it is no longer an emulation, but the reality itself. Vertical axis can be commanded: `YILDIZ_TILT = atan(down/back)`. Detail: `GIMBAL_NOTES.md`.
- **Winning method MPC**; LOS and PID are frozen (code stands for comparison).
- **yaw-lock on**; cycle three-door: ~1.5 s framing + coverage ≥%2 + estimator range ≤60 m.
- back **25 PROVEN** — back 40 attempt killed closure (range 12513→1800, min range 7→19 m).
