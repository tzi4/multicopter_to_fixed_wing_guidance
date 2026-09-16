#!/usr/bin/env bash
# Single-command trial: launch the stack, start the target on an ellipse,
# take off the pursuer, enable position guidance with standoff (no collision),
# and collect bbox measurements.
# Each trial writes run/trials/<timestamp>/:
# guidance.log: guidance output; bbox.log: detector output;
# summary.txt: range closure, detection count and bbox dimensions.
# YILDIZ_VIDEO=1 records videos/guidance_<timestamp>.mp4.
#
# tools/scenario.sh                   # Default: 300 s tracking
# DURATION=600 tools/scenario.sh       # Longer trial
# BACK=35 tools/scenario.sh            # Derive DOWN from the mounting angle
# BACK=25 DOWN=3 tools/scenario.sh     # Override derivation (misaligns camera)
# YILDIZ_MOUNT=20 tools/scenario.sh    # Use after changing mounting in model.sdf
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

DURATION="${DURATION:-300}"
# STANDOFF: BACK/DOWN is no longer STANDOFF here, but is derived from the camera mounting angle
# (scripts/standoff_geom.sh: down = round(back*tan(mount+trim))). The binary flown by the guide and
# the binary going to bbox_to_redis.py must be the SAME; diverges, the target remains off camera axis
# (see yildizlar_gimbal.analytical_aim = -atan(down/back)). If BACK=/DOWN= (or
# YILDIZ_BACK=/YILDIZ_DOWN=) is given, the derivation is CRUSHED.
[[ -n "${BACK:-}" ]] && export YILDIZ_BACK="$BACK"
[[ -n "${DOWN:-}" ]] && export YILDIZ_DOWN="$DOWN"
# shellcheck source=../scripts/standoff_geom.sh
source "$SCRIPT_DIR/scripts/standoff_geom.sh"
# The binary finalized after derivation; The following stack restart also inherits the same values
# ​​(exported YILDIZ_BACK/YILDIZ_DOWN).
BACK="$YILDIZ_BACK"
DOWN="$YILDIZ_DOWN"
SIDE="${SIDE:-0}"
DRONE_ALT="${DRONE_ALT:-60}"
RESTART="${RESTART:-1}"
# In fresh stacks, target altitude IMM and fighter EKF may temporarily jump immediately after takeoff.
# 0 is legacy behavior; In case of problematic initial A/Bs, a value such as CONTROL_WAIT_S=20 is set
# before the commands start.
CONTROL_WAIT_S="${CONTROL_WAIT_S:-0}"

# --- PRINCIPAL TARGET (TARGET_VEHICLE=drone2) --------------------------------- The fixed wing target
# actually CANNOT stop; The "target suspended in the air" scenario can only be established with a
# copter. With TARGET_VEHICLE=drone2, the target role passes to drone_2: - the aircraft is NOT lifted
# (stays on the ground; not put into AUTO), - drone_2 takes off at GUIDED and moves north for
# TARGET_DISTANCE m and remains suspended there, - target ports of the guidance side In
# guidance_config.py it returns 14662/14664 (drone_2) with the same env -- see block TARGET_VEHICLE there.
# If left blank (default) nothing changes.
TARGET_VEHICLE="${TARGET_VEHICLE:-}"
TARGET_ALT="${TARGET_ALT:-70}"
TARGET_DISTANCE="${TARGET_DISTANCE:-1200}"
TARGET_TRAVEL_S="${TARGET_TRAVEL_S:-90}"
TARGET_WAIT_S="${TARGET_WAIT_S:-90}"
if [[ "$TARGET_VEHICLE" == drone2 ]]; then
  export TARGET_VEHICLE          # pass to guidance processes via guidance_config.py
  # drone_2 only exists in the swarm world and when DRONE_COUNT>=2.
  export YILDIZ_DRONES="${YILDIZ_DRONES:-2}"
  export YILDIZ_WORLD="${YILDIZ_WORLD:-$SCRIPT_DIR/worlds/swarm.world}"
fi

# --- Experiment ID ----------------------------------------------------------------- PLAN: target
# route (straight / ellipse / wanderer). Default ellipse (medium level). DISPLAY: Visual guidance
# command that will run under guidance_allstar (e.g. DISPLAY="los_guidance.py"). If it is empty, it is
# run alone. METHOD: video/folder tag; If not given, DISPLAY is derived from the file name.
PLAN="${PLAN:-$SCRIPT_DIR/missions/target_ellipse.plan}"
PLAN_NAME="$(basename "$PLAN" .plan)"; PLAN_NAME="${PLAN_NAME#target_}"
if [[ -z "${METHOD:-}" ]]; then
  if [[ -n "${VISUAL_GUIDANCE:-}" ]]; then
    METHOD="$(basename "${VISUAL_GUIDANCE%% *}" .py)"; METHOD="${METHOD%_guidance}"
  else
    METHOD="position_based"
  fi
fi
LABEL="${METHOD}_${PLAN_NAME}"
export YILDIZ_VIDEO_LABEL="$LABEL"

STAMP="$(date +%Y%m%d_%H%M%S)"
TRIAL_DIR="$SCRIPT_DIR/run/trials/${LABEL}_$STAMP"
mkdir -p "$TRIAL_DIR"
echo ">>> trial: $TRIAL_DIR (video label: $LABEL)"

cleanup() {
  # pgrep/pkill ALSO MATCHES THE PATTERN TO OUR OWN COMMAND LINE: the square bracket trick ('[f]ollow')
  # saves the pattern from mapping to itself.
  for pid in $(pgrep -f "simple_guided_[f]ollow" || true); do kill "$pid" 2>/dev/null || true; done
  for pid in $(pgrep -f "swarm_command.py speed-[l]ock" || true); do kill "$pid" 2>/dev/null || true; done
  # PATTERN REQUIRES 'python3' PREFIX: bare The pattern "[a-z]*_guide\.py" also matched the command line
  # of the wrapper shell itself that STARTED the experiment (e.g. "bash run_value2.sh los_guidance.py ellipse"),
  # killing the process that started the experiment and issuing exit 144 -- the caller while the run was
  # detached it seemed. The prefix only fits the actual python process (and its timeout). The method
  # name can have multiple underscores (terminal_los_guidance.py). The old [a-z]* pattern missed such
  # processes, allowing two controllers to issue commands simultaneously from the same MAVLink port.
  for pid in $(pgrep -f "python3 visual_[b]ase|python3 [a-z_]*_[g]uidance\.py" || true); do kill "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT

cleanup
sleep 2

if [[ "$RESTART" == 1 ]]; then
  echo ">>>stack restarting (video recording on)"
  ./yildizlar_guidance.sh --stop >/dev/null 2>&1 || true
  sleep 3
  YILDIZ_VIDEO=1 YILDIZ_TARGET_PLAN="$PLAN" \
    YILDIZ_AIM="${AIM:-0}" ./yildizlar_guidance.sh --headless
fi

if [[ "$TARGET_VEHICLE" == drone2 ]]; then
  echo ">>> HOVERING TARGET: drone_2 copter, ${TARGET_ALT} m, ${TARGET_DISTANCE} m north"
  echo "(target aircraft remains on the ground; plan $PLAN_NAME is loaded but AUTO is not selected)"
  # WAITING FOR EKF POSITION ESTIMATE: right after the stack says "ready" drone_2's ARM is REJECTED with
  # "Arm: Need Position Estimate" (measured 2026-08-05, two runs in a row so it fell; same copter ~10 It
  # was removed by hand after minutes without any problems). It was not visible in the aircraft branch
  # because 'target-takeoff' works there first and that step takes minutes -- EKF was sitting at that
  # time. There is no delay in the hanging target branch, it is placed manually.
  echo ">>> Waiting ${TARGET_WAIT_S} s for EKF initialization"
  sleep "$TARGET_WAIT_S"
  took_off=0
  for trial in 1 2 3; do
    if python3 tools/swarm_command.py drone-takeoff --id 2 --alt "$TARGET_ALT" \
        --timeout 180 | tail -1; then
      took_off=1; break
    fi
    echo ">>> drone_2 take-off failed (try $trial/3), repeat after 30 s"
    sleep 30
  done
  [[ "$took_off" == 1 ]] || { echo "drone_2 takeoff failed -- trial canceled" >&2; exit 1; }
  # WHY 'speed-test': this is the only available command that sends the hopper to a remote point and
  # LEFT IT THERE (go_to_point -> GUIDED setpoint is permanent, the hopper hangs at the spot when the
  # command finishes). Velocity measurement by-product; The goal is to park drone_2 far from the
  # fighter's take-off point -- otherwise the two hoppers would spawn 2.5 m apart and the shutdown phase
  # would never occur.
  python3 tools/swarm_command.py speed-test --id 2 --distance "$TARGET_DISTANCE" \
    --speed-value 20 --duration-value "$TARGET_TRAVEL_S" | tail -3
else
  # --- PLAN COMPATIBILITY GATE (2026-08-09) -------------------------------- PLAN is loaded into the
  # tool only in the RESTART=1 branch (above). When run with RESTART=0, the PLAN remains
  # SILENTLY INACTIVE and the target continues to fly the mission (default ellipse) in the stack
  # opening. THIS FAILURE LOCKED TWO TRIALS (tyawacc_straight, kpn_straight: they flew ELLIPSE with the label
  # "flat"; one was reported as "straight regression PASSED"). So DEFAULT BEHAVIOR = STOP. It is skipped
  # with SCENARIO_PLAN_CONTROL=0.
  PLAN_LABEL="$PLAN_NAME"
  if [[ "$RESTART" != 1 && "${SCENARIO_PLAN_CONTROL:-1}" == 1 ]]; then
    echo ">>> checking the mission loaded on the vehicle against $PLAN_NAME"
    set +e
    python3 tools/plan_alignment.py --plan "$PLAN"
    PLAN_CODE=$?
    set -e
    if [[ "$PLAN_CODE" == 1 ]]; then
      echo "ERROR: loaded mission is not '$PLAN_NAME' -- trial canceled." >&2
      echo "Solution: restart the stack as follows:" >&2
      echo "      ./yildizlar_guidance.sh --stop ; sleep 3" >&2
      echo "      YILDIZ_TARGET_PLAN=$PLAN ./yildizlar_guidance.sh" >&2
      echo "      To skip this check explicitly: SCENARIO_PLAN_CONTROL=0" >&2
      exit 1
    elif [[ "$PLAN_CODE" != 0 ]]; then
      echo "WARNING: could not verify mission consistency (vehicle/mission unreadable)." >&2
      PLAN_LABEL="$PLAN_NAME (unverified)"
    fi
  elif [[ "$RESTART" != 1 ]]; then
    PLAN_LABEL="$PLAN_NAME (unverified: SCENARIO_PLAN_CONTROL=0)"
  fi
  # The label now carries the VERIFICATION STATUS: the old line was from PLAN_NAME and did NOT say what
  # was loaded into the vehicle -- it was misleading.
  echo ">>> target aircraft: AUTO, route $PLAN_LABEL"
  python3 tools/swarm_command.py target-takeoff --alt 55 --timeout 240 | tail -2
fi

echo ">>> pursuer multicopter: GUIDED, $DRONE_ALT m"
python3 tools/swarm_command.py drone-takeoff --id 1 --alt "$DRONE_ALT" --timeout 180 | tail -1
if awk -v s="$CONTROL_WAIT_S" 'BEGIN { exit !(s > 0) }'; then
  echo ">>> estimator/EKF settling delay: ${CONTROL_WAIT_S} s"
  sleep "$CONTROL_WAIT_S"
fi

# bbox Mark to separate this trial portion of the log
BBOX_START=$(wc -l < logs/bbox.log)

# AIM measurement: run SIMULTANEOUS to homing (ey = -rise when aim=0)
setsid python3 tools/measure_aim.py --duration-value "$DURATION" --label-value "$PLAN_NAME" \
  > "$TRIAL_DIR/aim.txt" 2>&1 < /dev/null &
disown

# Video guidance: stands up WITH positioned, DOES NOT send command until Redis 'command_authority' is
# 'visual' (visual_base.py waits).
if [[ -n "${VISUAL_GUIDANCE:-}" ]]; then
  echo ">>> visual guidance awaiting authority: $VISUAL_GUIDANCE"
  (
    cd guidance_allstar
    # PYTHONUNBUFFERED: Since the log goes to the file, prints are block buffered; flow line by line for
    # live monitoring/death diagnosis. shellcheck disabled=SC2086
    PYTHONUNBUFFERED=1 setsid timeout $((DURATION + 60)) python3 $VISUAL_GUIDANCE \
      > "$TRIAL_DIR/visual.log" 2>&1 < /dev/null &
  )
fi

# --- YAW LOCK ------------------------------------------------------------------ BACK (2026-08-03),
# closed with YAW_LOCK=0. WHY BACK: For visual handoff, the target must remain in the frame at
# positioned APPROACH; camera fixed to the body (horizontal FOV +-33 degrees) sees the target only if
# the nose is pointed at the target. MEASURED during the pid_straight run: When yaw is not commanded (on
# autopilot), in straight/intersecting geometry the copter hangs at ~-65 degrees, in close phase
# (range<60 m) the target bearing deviates from the nose by MEDIAN 95 degrees and the target frames
# are only %30 in the frame -- stable 1.5 s window was not formed, so the handoff was NEVER triggered
# (it was working because the target was in front of the ellipse). WHY WAS IT CLOSED: slow loop (2 Hz)
# + constant-dt assumption was making yaw chased and jittered by 344 deg/s; both recovered on a5a28eb
# (measured 19.9 Hz). Safeguards stand at guidance_config.py: YAW_LOCK_MODE="los" (nose to target), 90
# deg/s slew, 38 freeze above degree tilt, 10 keep last yaw below m. If the tremors return,
# YAW_LOCK=0.
YAW_LOCK="${YAW_LOCK:-1}"
YAW_FLAG=(); [[ "$YAW_LOCK" == 1 ]] && YAW_FLAG=(--yaw-lock)

# --- APPROACH LAW (positioned phase BEFORE handoff) ---------------------- APPROACH=slot|intersection
# -> simple_guided_follow.py moves to --approximation. IF IT IS LEAVED BLANK, NO FLAG WILL BE ADDED, so
# today's behavior is BIT-SAME (the script's own default is 'slot' anyway; we DO NOT write the default
# here, so that the default in the script remains the only resource). WARNING: 'intercept' AIMS at
# target during arm positioned phase (collision course); The contact can also be BEFORE after the
# DISPLAY handoff. See simple_guided_follow.py KesismeGuidance and memory: "approximation law two
# options".
APPROXIMATION="${APPROXIMATION:-}"
APPROXIMATION_FLAG=()
if [[ -n "$APPROXIMATION" ]]; then
  APPROXIMATION_FLAG=(--approximation "$APPROXIMATION")
  echo ">>> APPROXIMATION LAW = ${APPROXIMATION} (position-based phase)"
fi
echo ">>> position guidance: guidance_allstar (kill mode OFF, yaw lock=${YAW_LOCK})"
echo "standoff: back=${BACK} m side=${SIDE} m down=${DOWN} m, duration=${DURATION} s"
# --- TILT ADAPTATION DOOR (2026-08-09) --------------------------------- Standoff vertical geometry
# goes to THREE consumers: bbox (at the STACK opening), simple_guided_follow and mpc_guidance (at the RUN
# start, from here). Since the first two are fed from separate media, they can be separated silently
# -- measured: when the stack was up with DOWN=0, when the scenario did not give DOWN, it returned to
# the design value (4) and MPC thought "camera axis +9.09"; The camera was looking at 0.00. Same class
# control as plan_alignment.py: skipped with SCENARIO_TILT_CONTROL=0.
if [[ "${SCENARIO_TILT_CONTROL:-1}" == 1 ]]; then
  set +e
  python3 tools/tilt_alignment.py --expected-value "$YILDIZ_TILT"
  TILT_CODE=$?
  set -e
  if [[ "$TILT_CODE" == 1 ]]; then
    echo "ERROR: camera tilt commanded by the stack DOES NOT match that of the scenario" >&2
    echo "      -- trial canceled (image references differ)." >&2
    echo "      Resolution: run the scenario with the same DOWN as the stack (e.g. DOWN=0)," >&2
    echo "      or restart the stack with the requested DOWN." >&2
    echo "      To skip this check explicitly: SCENARIO_TILT_CONTROL=0" >&2
    exit 1
  elif [[ "$TILT_CODE" != 0 ]]; then
    echo "WARNING: could not verify tilt consistency (bbox.log unreadable)." >&2
  fi
fi
# YILDIZ_NO_GUI=1: estimator window should not open suddenly in headless test. If GUI is requested,
# YILDIZ_NO_GUI=0 is given.
(
  cd guidance_allstar
  YILDIZ_NO_GUI="${YILDIZ_NO_GUI:-1}" timeout "$DURATION" python3 simple_guided_follow.py \
    --no-kill-mode "${YAW_FLAG[@]}" "${APPROXIMATION_FLAG[@]}" \
    --back "$BACK" --side "$SIDE" --down "$DOWN" \
    > "$TRIAL_DIR/guidance.log" 2>&1 || true
)

echo ">>> trial finished, generating summary"
tail -n +"$BBOX_START" logs/bbox.log > "$TRIAL_DIR/bbox.log" || true
python3 tools/trial_summary.py "$TRIAL_DIR" | tee "$TRIAL_DIR/summary_value.txt"
echo; cat "$TRIAL_DIR/aim.txt" 2>/dev/null | tail -8
