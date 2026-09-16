#!/usr/bin/env bash
# =====================================================================
# yildizlar_guidance.sh - MAIN LAUNCHER (up to 5 copters + 1 fixed-wing target)
# =====================================================================
# Environment source: the supplied all_star_env.zip (swarm.world, five
# camera-equipped Iris vehicles, and gazebo-plane6). Launcher infrastructure
# comes from bumblebee_guidance.sh: PID-file ownership, preflight checks,
# supervised SITL/MAVProxy processes, and --stop.
#
# VEHICLE / PORT CONVENTION: keep consistent with config.py.
#   vehicle  -I  SysID  TCP    RCin  FDM(in/out)  MAVProxy --out
#   drone_1   0    1    5760   5501   9002/9003   14550, 14551, 14651, 14652, 14653, 14654(visual)
#   drone_2   1    2    5770   5511   9012/9013   14550, 14561, 14661, 14662
#   drone_3   2    3    5780   5521   9022/9023   14550, 14571, 14671, 14672
#   drone_4   3    4    5790   5531   9032/9033   14550, 14581, 14681, 14682
#   drone_5   4    5    5800   5541   9042/9043   14550, 14591, 14691, 14692
#   target    5    6    5810   5551   9052/9053   14550, 14601, 14602, 14603
# Last column: shared QGC / tools / wait+plan / guidance_allstar / bbox-gimbal.
# FDM ports MUST match <fdm_port_in> in models/*/model.sdf.
# SITL derives 9002+10N / 9003+10N from the instance argument -I N.
#
# WSL2 PITFALL: sim_vehicle.py sends its default --out to the Windows host,
# leaving the listener on 14551 empty. This launcher connects the SITL binary
# and MAVProxy directly, with an explicit --out list.
#
# USAGE:
#   ./yildizlar_guidance.sh              # GUI: gzclient, QGC, bbox window
#   ./yildizlar_guidance.sh --headless   # No GUI; bbox uses --no-display
#   ./yildizlar_guidance.sh --iris       # Iris model (default)
#   ./yildizlar_guidance.sh --hummingbird # RotorS Hummingbird model
#   ./yildizlar_guidance.sh --robofly    # CTU-MRS RoboFly model
#   ./yildizlar_guidance.sh --stop       # Stop processes started by this package
#   ./yildizlar_guidance.sh              # Bbox video recording is on by default
#   YILDIZ_VIDEO=0 ./yildizlar_guidance.sh   # Explicitly disable video recording
#   YILDIZ_GAZEBO_ONLY=1 ./yildizlar_guidance.sh  # Gazebo-only diagnostic mode
#   YILDIZ_DRONES=1 ./yildizlar_guidance.sh  # Launch only drone_1 instead of five
# =====================================================================
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/run"
LOG_DIR="$SCRIPT_DIR/logs"
PID_FILE="$RUN_DIR/pids"
LOCK_FILE="$RUN_DIR/launcher.lock"
LAUNCHER_FILE="$RUN_DIR/launcher.pid"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
IQ_SIM_MODELS="${IQ_SIM_MODELS:-${HOME}/catkin_ws/src/iq_sim/models}"
QGC_BIN="${QGC_BIN:-${HOME}/Applications/QGroundControl.AppImage}"

# Default world: a single pursuer, with iris-2..5 removed from swarm.world.
# Each removed Iris also removes its camera-rendering workload. To use the
# swarm, set YILDIZ_WORLD=worlds/swarm.world and YILDIZ_DRONES=5.
WORLD_FILE="${YILDIZ_WORLD:-$SCRIPT_DIR/worlds/single_pursuer.world}"
COPTER_PARAM="${YILDIZ_COPTER_PARAM:-$SCRIPT_DIR/params/swarm_copter.parm}"
TARGET_PARAM="${YILDIZ_TARGET_PARAM:-$SCRIPT_DIR/params/target_plane.parm}"
# Target AUTO mission. If TARGET_PLAN is empty, the plan is not loaded.
# The default ellipse exercises the guidance_allstar IMM estimator's CTxy
# turning mode continuously. On the rectangular route, turns occurred only
# at corners, so that mode rarely activated. Select the rectangular
# missions/target_circuit.plan using YILDIZ_TARGET_PLAN when needed.
TARGET_PLAN="${YILDIZ_TARGET_PLAN:-$SCRIPT_DIR/missions/target_ellipse.plan}"
# All vehicles use the same home at the Gazebo world origin; their relative
# positions come from <pose> entries in the world file. The default is the
# public ArduPilot SITL/CMMAT location. Override with YILDIZ_HOME.
HOME_POS="${YILDIZ_HOME:--35.363261,149.165230,0,0}"
# Number of copter SITL/MAVProxy instances to launch (1..5). The default 1
# reduces CPU load and is sufficient for one pursuer and one target.
# YILDIZ_DRONES=5 together with the swarm world restores the five-copter setup.
DRONE_COUNT="${YILDIZ_DRONES:-1}"
# Copter physics model. Iris is the default for backward compatibility. --iris/--hummingbird/--robofly
# on the command line overrides this environment variable.
DRONE_MODEL="${YILDIZ_DRONE_MODEL:-iris}"
# MAVPROXY STREAMRATE: MAVProxy defaults to 4 Hz and periodically enforces it
# through REQUEST_DATA_STREAM, overriding our SET_MESSAGE_INTERVAL requests.
# A requested 50 Hz ATTITUDE stream measured only 5.9 Hz, with a median sample
# interval of 249 ms. The virtual gimbal interpolates attitude to the frame
# capture time, so sparse attitude samples directly affect the angle error.
# Pursuer: 20 Hz matches the 20 Hz guidance loop. The rate was reduced from
# 50 Hz on 2026-08-02 because five UDP outputs at that rate overloaded the
# guidance reader thread and contributed to loop rates as low as 2.2 Hz.
# Target: 15 Hz. Previously, omitting --streamrate left MAVProxy at 4 Hz,
# overriding the requested 15 Hz GLOBAL_POSITION_INT rate and keeping target
# telemetry at 4-5 Hz. Filtering and dead reckoning depend on this timing.
# Override with YILDIZ_STREAMRATE and YILDIZ_TARGET_STREAMRATE.
#
# CAMERA PIPELINE LATENCY: 80 ms. An arriving frame was captured about 80 ms
# earlier; interpolate attitude to that capture time. Measured using
# tools/calibrate_gimbal_timing.py over -40..200 ms by minimizing
# |corr(stab_ey, pitch)|: 0 ms -> 0.253; 80 ms -> 0.232.
# VIRTUAL GIMBAL: YILDIZ_MOUNT is the physical mount angle (+30 degrees up,
# matching model.sdf). YILDIZ_AIM is the target's desired vertical image offset;
# the virtual frame center is -AIM relative to the horizon (yildizlar_gimbal.py).
# The nominal -27 degrees comes from the position-guidance standoff geometry:
#   aim = -atan(down/back) = -atan(13/25) = -27.5
# Measured angles: ellipse -29.25, straight -27.03, wanderer -10.52. The
# wanderer's altitude changes broaden its distribution. Recompute if BACK or
# DOWN changes.
# AIM affects only the vertical channel (corrected 2026-08-02). Previously,
# applying the Ry rotation R_aim to the horizontal angle compressed bearing
# by cos(eps)/cos(eps+aim). In gimbal4.csv, gain was 0.910: guidance read
# bearing 8.8% too low and commanded too little turning toward the target.
# yildizlar_gimbal.py:angle_error_value now reads the horizontal component
# from the ray before AIM, giving gain 1.004 on the same frames. The earlier
# roll correlation was a false alarm: bank-to-turn commands roll from bearing
# error, so the two quantities are coupled by the controller. Ground-truth
# validation gave corr(stab_ex,-ground_truth_lateral)=0.992, a residual
# sin(roll) slope of -0.15 deg, scatter 0.28 deg, and confirmed mount +30.0.
#
# STANDOFF VERTICAL GEOMETRY: derive it from the mounting angle. The sourced
# script exports YILDIZ_MOUNT (default 30, MUST match model.sdf),
# YILDIZ_PITCH_TRIM (default -2.5 deg, typical steady-tracking copter pitch),
# YILDIZ_BACK, and the derived YILDIZ_DOWN:
#   down = round(back * tan(mount + pitch_trim)) -> 13 m for back = 25 m.
# See scripts/standoff_geom.sh for the rationale and flight evidence.
# Override with YILDIZ_BACK / YILDIZ_DOWN / YILDIZ_MOUNT / YILDIZ_PITCH_TRIM.
# shellcheck source=scripts/standoff_geom.sh
source "$SCRIPT_DIR/scripts/standoff_geom.sh"

# The camera topic that the bbox detector will listen to.
CAM_TOPIC="${YILDIZ_CAM_TOPIC:-/drone_1/webcam/image_raw}"

HEADLESS=0
MODE=start

usage() {
  cat <<'USAGE'
Usage: yildizlar_guidance.sh [--iris|--hummingbird|--robofly] [--headless] [--without-gimbal] | --stop

  --iris          Use the Iris physics model (default)
  --hummingbird   Use the RotorS Hummingbird physics model
  --robofly       Use the CTU-MRS RoboFly physics model
  --headless      Omit the GUI; do not start gzclient or QGC
  --without-gimbal Fix the camera to the body (pre-gimbal behavior).
                   The vertical gimbal is enabled by default.
                   Equivalent: YILDIZ_GIMBAL=0 ./yildizlar_guidance.sh
  --stop          Stop all processes started by this package

Flags can be combined: ./yildizlar_guidance.sh --robofly --headless
Equivalent environment setting: YILDIZ_DRONE_MODEL=iris|hummingbird|robofly
USAGE
}

# Gimbal is enabled by default (1). --without-gimbal prepends the generated
# models_fixed_camera/ tree to GAZEBO_MODEL_PATH, so Gazebo resolves
# model://swarm_drone_N there first. tools/generate_without_gimbal.py creates
# that tree; the world file and original models/ tree remain unchanged.
GIMBAL="${YILDIZ_GIMBAL:-1}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --iris) DRONE_MODEL=iris ;;
    --hummingbird) DRONE_MODEL=hummingbird ;;
    --robofly) DRONE_MODEL=robofly ;;
    --headless) HEADLESS=1 ;;
    --without-gimbal) GIMBAL=0 ;;
    --stop) MODE=stop ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done

if [[ "$DRONE_MODEL" != iris && "$DRONE_MODEL" != hummingbird && "$DRONE_MODEL" != robofly ]]; then
  echo "YILDIZ_DRONE_MODEL must be iris, hummingbird or robofly (received: $DRONE_MODEL)" >&2
  exit 2
fi

if ! [[ "$DRONE_COUNT" =~ ^[1-5]$ ]]; then
  echo "YILDIZ_DRONES must be between 1 and 5 (received: $DRONE_COUNT)" >&2
  exit 2
fi

mkdir -p "$RUN_DIR" "$LOG_DIR" "$RUN_DIR/ros" "$SCRIPT_DIR/videos"

pid_matches() {
  local pid="$1" expected_ticks="$2" actual_ticks
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  actual_ticks="$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || true)"
  [[ "$actual_ticks" == "$expected_ticks" ]]
}

launcher_matches() {
  local pid="$1" ticks="$2" expected_pgid="$3" actual_pgid
  pid_matches "$pid" "$ticks" || return 1
  actual_pgid="$(awk '{print $5}' "/proc/$pid/stat" 2>/dev/null || true)"
  [[ "$actual_pgid" == "$expected_pgid" ]]
}

# launcher.pid did not exist before this fix. If an old launcher holds the lock, consider it to be the
# owner only if the process meets all three conditions: it runs this script from the same directory,
# it is not --stop, and it actually keeps LOCK_FILE open. This will also clear the suspended starter
# from the old version with the first --stop.
find_legacy_launcher() {
  local proc pid cwd arg fd found_script is_stop ticks pgid
  local -a argv
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    [[ "$pid" != "$$" && -r "$proc/cmdline" ]] || continue
    argv=()
    mapfile -d '' -t argv < "$proc/cmdline" 2>/dev/null || continue
    found_script=0
    is_stop=0
    for arg in "${argv[@]}"; do
      [[ "$arg" == --stop ]] && is_stop=1
      [[ "${arg##*/}" == "${BASH_SOURCE[0]##*/}" ]] && found_script=1
    done
    [[ "$found_script" -eq 1 && "$is_stop" -eq 0 ]] || continue
    cwd="$(readlink "$proc/cwd" 2>/dev/null || true)"
    [[ "$cwd" == "$SCRIPT_DIR" ]] || continue
    for fd in "$proc"/fd/*; do
      [[ -e "$fd" && "$fd" -ef "$LOCK_FILE" ]] || continue
      ticks="$(awk '{print $22}' "$proc/stat" 2>/dev/null || true)"
      pgid="$(awk '{print $5}' "$proc/stat" 2>/dev/null || true)"
      [[ -n "$ticks" && "$pgid" =~ ^[0-9]+$ ]] || continue
      printf '%s %s %s\n' "$pid" "$ticks" "$pgid"
      return 0
    done
  done
  return 1
}

read_launcher_record() {
  local pid ticks pgid
  if [[ -f "$LAUNCHER_FILE" ]]; then
    read -r pid ticks pgid < "$LAUNCHER_FILE" || true
    if [[ "$pid" =~ ^[0-9]+$ && "$ticks" =~ ^[0-9]+$ && "$pgid" =~ ^[0-9]+$ ]] \
        && launcher_matches "$pid" "$ticks" "$pgid"; then
      printf '%s %s %s\n' "$pid" "$ticks" "$pgid"
      return 0
    fi
  fi
  find_legacy_launcher
}

stop_launcher() {
  local record pid ticks pgid
  record="$(read_launcher_record)" || return 1
  read -r pid ticks pgid <<< "$record"
  echo "Baslatici durduruluyor (PID $pid)"

  # The job started in the interactive shell is in its own process group. Ctrl-Z stops this entire group
  # (for example, the currently running sleep); Sending CONT after TERM causes the signal trap to
  # operate and the lock to be released. In a common process group, only the launcher is touched to
  # avoid affecting the parent shell.
  if [[ "$pgid" == "$pid" ]]; then
    kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -CONT -- "-$pgid" 2>/dev/null || true
  else
    kill -TERM -- "$pid" 2>/dev/null || true
    kill -CONT -- "$pid" 2>/dev/null || true
  fi

  for _ in {1..50}; do
    launcher_matches "$pid" "$ticks" "$pgid" || return 0
    sleep 0.1
  done
  if [[ "$pgid" == "$pid" ]]; then
    kill -KILL -- "-$pgid" 2>/dev/null || true
  else
    kill -KILL -- "$pid" 2>/dev/null || true
  fi
  for _ in {1..20}; do
    launcher_matches "$pid" "$ticks" "$pgid" || return 0
    sleep 0.1
  done
  return 1
}

stop_owned() {
  [[ -f "$PID_FILE" ]] || { echo "There is no process record."; return 0; }
  mapfile -t records < "$PID_FILE"
  local index name pid ticks
  for ((index=${#records[@]}-1; index>=0; index--)); do
    read -r name pid ticks <<< "${records[$index]}"
    if pid_matches "$pid" "$ticks"; then
      echo "Stopping $name (PID $pid)"
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  for _ in {1..80}; do
    local alive=0
    for record in "${records[@]}"; do
      read -r name pid ticks <<< "$record"
      pid_matches "$pid" "$ticks" && alive=1
    done
    [[ "$alive" -eq 0 ]] && break
    sleep 0.1
  done
  for record in "${records[@]}"; do
    read -r name pid ticks <<< "$record"
    if pid_matches "$pid" "$ticks"; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  rm -f -- "$PID_FILE"
}

exec 9>"$LOCK_FILE"
if [[ "$MODE" == stop ]]; then
  # Get the lock first: if initialization continues, close the owner and try again. Deleting the
  # PID_FILE without acquiring the lock left the suspended launcher with the lock and unregistered. If a
  # new start intervenes, the next round catches it too.
  lock_acquired=0
  for _ in {1..3}; do
    if flock -n 9; then
      lock_acquired=1
      break
    fi
    if ! stop_launcher; then
      sleep 0.2
    fi
  done
  if [[ "$lock_acquired" -eq 0 ]] && flock -w 5 9; then
    lock_acquired=1
  fi
  if [[ "$lock_acquired" -eq 0 ]]; then
    echo "Could not stop the launcher; the lock is still held." >&2
    exit 1
  fi
  rm -f -- "$LAUNCHER_FILE"
  stop_owned
  exit 0
fi

if ! flock -n 9; then
  echo "The launcher is being used by another process." >&2
  exit 1
fi

LAUNCHER_TICKS="$(awk '{print $22}' "/proc/$$/stat")"
LAUNCHER_PGID="$(awk '{print $5}' "/proc/$$/stat")"
printf '%s %s %s\n' "$$" "$LAUNCHER_TICKS" "$LAUNCHER_PGID" > "$LAUNCHER_FILE"

cleanup_launcher_record() {
  local pid ticks pgid
  [[ -f "$LAUNCHER_FILE" ]] || return 0
  read -r pid ticks pgid < "$LAUNCHER_FILE" || return 0
  if [[ "$pid" == "$$" && "$ticks" == "$LAUNCHER_TICKS" ]]; then
    rm -f -- "$LAUNCHER_FILE"
  fi
}
trap cleanup_launcher_record EXIT

if [[ -f "$PID_FILE" ]]; then
  while read -r _ pid ticks; do
    if pid_matches "$pid" "$ticks"; then
      echo "The system is already working. Use --stop first." >&2
      exit 1
    fi
  done < "$PID_FILE"
  rm -f -- "$PID_FILE"
fi

preflight() {
  local failed=0 command path index
  for command in python3 gzserver redis-cli mavproxy.py setsid flock ss; do
    command -v "$command" >/dev/null 2>&1 || { echo "Missing command: $command" >&2; failed=1; }
  done
  for path in "$WORLD_FILE" "$COPTER_PARAM" "$TARGET_PARAM" \
    "$ARDUPILOT_DIR/build/sitl/bin/arducopter" "$ARDUPILOT_DIR/build/sitl/bin/arduplane" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane.parm" \
    "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" \
    "$SCRIPT_DIR/models/gazebo-plane6/model.sdf" \
    /opt/ros/noetic/setup.bash "$ARDUPILOT_GAZEBO_DIR/devel/setup.bash"; do
    [[ -e "$path" ]] || { echo "Missing dependency: $path" >&2; failed=1; }
  done
  for ((index=1; index<=DRONE_COUNT; index++)); do
    [[ -e "$SELECTED_MODEL_ROOT/swarm_drone_$index/model.sdf" ]] || {
      echo "Missing dependency: $SELECTED_MODEL_ROOT/swarm_drone_$index/model.sdf" >&2; failed=1; }
  done
  if [[ "$DRONE_MODEL" != iris ]]; then
    for command in cmake protoc; do
      command -v "$command" >/dev/null 2>&1 || { echo "Missing command: $command" >&2; failed=1; }
    done
    for path in \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/CMakeLists.txt" \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/ArduPilotMotorBridge.cc"; do
      [[ -e "$path" ]] || { echo "Missing dependency: $path" >&2; failed=1; }
    done
  fi
  if [[ "$DRONE_MODEL" == hummingbird ]]; then
    [[ -e "$SCRIPT_DIR/models_hummingbird/hummingbird/model.sdf" ]] || {
      echo "Missing dependency: $SCRIPT_DIR/models_hummingbird/hummingbird/model.sdf" >&2; failed=1; }
  elif [[ "$DRONE_MODEL" == robofly ]]; then
    for path in \
      "$SCRIPT_DIR/models_robofly/robofly/model.sdf" \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/upstream/fluid_resistance_plugin.cpp"; do
      [[ -e "$path" ]] || { echo "Missing dependency: $path" >&2; failed=1; }
    done
  fi
  if [[ "$HEADLESS" -eq 0 ]]; then
    command -v gzclient >/dev/null 2>&1 || { echo "Missing command: gzclient" >&2; failed=1; }
    [[ -x "$QGC_BIN" ]] || echo "WARNING: QGroundControl absent ($QGC_BIN), atlaniyor." >&2
  fi
  [[ "$failed" -eq 0 ]] || return 1

  # Fail preflight if required ports are occupied by another SITL/QGC process.
  local wanted occupied
  wanted='^(5760|5770|5780|5790|5800|5810|5501|5511|5521|5531|5541|5551|9002|9012|9022|9032|9042|9052|14551|14561|14571|14581|14591|14601)$'
  occupied="$(ss -H -lntu | awk '{print $5}' | sed 's/.*://' | grep -E "$wanted" | sort -u || true)"
  if [[ -n "$occupied" ]]; then
    echo "Required ports are in use: $(echo "$occupied" | paste -sd, -)" >&2
    return 1
  fi
}

build_external_multirotor_plugins() {
  local source_dir="$SCRIPT_DIR/plugins/hummingbird_bridge"
  local build_dir="$source_dir/build"
  echo ">>> ${DRONE_MODEL^^}: checking/building motor and body plugins"
  if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
    cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
  fi
  cmake --build "$build_dir" --parallel
  for path in "$build_dir/libArduPilotMotorBridge.so" \
              "$build_dir/libgazebo_motor_model.so" \
              "$build_dir/libpx4_motor_msgs.so" \
              "$build_dir/libMrsGazeboCommonResources_FluidResistancePlugin.so"; do
    [[ -f "$path" ]] || { echo "Harici multirotor eklentisi uretilmedi: $path" >&2; return 1; }
  done
}

source /opt/ros/noetic/setup.bash
source "$ARDUPILOT_GAZEBO_DIR/devel/setup.bash"
# Put this package's models first. models/gazebo-plane6 overrides iq_sim's
# model with the same name because that version has broken gazebo-plane:: link names.
case "$DRONE_MODEL" in
  hummingbird) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models_hummingbird" ;;
  robofly) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models_robofly" ;;
  *) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models" ;;
esac

if [[ "$GIMBAL" -eq 0 ]]; then
  # Without a gimbal, search the selected model's fixed-camera tree first.
  python3 "$SCRIPT_DIR/tools/generate_without_gimbal.py" --vehicle-value "$DRONE_MODEL" || exit 1
  case "$DRONE_MODEL" in
    hummingbird) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_hummingbird_fixed_camera" ;;
    robofly) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_robofly_fixed_camera" ;;
    *) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_fixed_camera" ;;
  esac
  export GAZEBO_MODEL_PATH="$FIXED_MODEL_ROOT:$SELECTED_MODEL_ROOT:$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$IQ_SIM_MODELS"
  echo ">>> CAMERA: FIXED TO THE BODY (--without-gimbal) -- using $(basename "$FIXED_MODEL_ROOT")/"
else
  export GAZEBO_MODEL_PATH="$SELECTED_MODEL_ROOT:$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$IQ_SIM_MODELS"
  echo ">>> CAMERA: VERTICAL GIMBAL (default) -- --without-gimbal for without gimbal"
fi
EXTERNAL_MULTIROTOR_PLUGIN_DIR="$SCRIPT_DIR/plugins/hummingbird_bridge/build"
if [[ "$DRONE_MODEL" != iris ]]; then
  export GAZEBO_PLUGIN_PATH="$EXTERNAL_MULTIROTOR_PLUGIN_DIR:$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="$EXTERNAL_MULTIROTOR_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"
else
  export GAZEBO_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
fi
echo ">>> COPTER MODEL: ${DRONE_MODEL^^}"
printf '%s\n' "$DRONE_MODEL" > "$RUN_DIR/selected_model"
export ROS_HOME="$RUN_DIR/ros"
export ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"

start_process() {
  local name="$1" log="$2" pid ticks
  shift 2
  : > "$log"
  setsid "$@" 9>&- >> "$log" 2>&1 &
  pid=$!
  for _ in {1..20}; do
    [[ -r "/proc/$pid/stat" ]] && break
    sleep 0.05
  done
  ticks="$(awk '{print $22}' "/proc/$pid/stat")"
  echo "$name $pid $ticks" >> "$PID_FILE"
  STARTED_PID="$pid"
  STARTED_TICKS="$ticks"
  echo "$name baslatildi (PID $pid, log: $log)"
}

wait_command() {
  local description="$1" timeout="$2"
  shift 2
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    (( SECONDS >= deadline )) && { echo "$description timeout." >&2; return 1; }
    sleep 0.5
  done
  echo "$description ready."
}

wait_process_command() {
  local description="$1" timeout="$2" pid="$3" ticks="$4" log="$5"
  shift 5
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "Process exited before $description started (PID $pid)." >&2
      tail -n 30 "$log" >&2 || true
      return 1
    fi
    (( SECONDS >= deadline )) && { echo "$description timeout." >&2; return 1; }
    sleep 0.5
  done
  echo "$description ready."
}

models_ready() {
  local output
  output="$(rosservice call /gazebo/get_world_properties 2>/dev/null || true)"
  grep -q 'iris-1' <<< "$output" && grep -q 'target_value' <<< "$output"
}

assert_owned_alive() {
  local name pid ticks
  while read -r name pid ticks; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "$name exited unexpectedly (PID $pid)." >&2
      return 1
    fi
  done < "$PID_FILE"
}

abort_launch() {
  local status="$1"
  trap - ERR INT TERM
  echo "Initialization failed; The processes belonging to this package are being closed." >&2
  stop_owned >/dev/null 2>&1 || true
  exit "$status"
}
trap 'abort_launch $?' ERR
trap 'abort_launch 130' INT
trap 'abort_launch 143' TERM
preflight
if [[ "$DRONE_MODEL" != iris ]]; then
  build_external_multirotor_plugins
fi
: > "$PID_FILE"

if ! rosparam list >/dev/null 2>&1; then
  start_process roscore "$LOG_DIR/roscore.log" roscore
  wait_command "ROS master" 20 rosparam list
else
  echo "Using existing ROS master."
fi

if ! redis-cli ping 2>/dev/null | grep -q PONG; then
  start_process redis "$LOG_DIR/redis.log" redis-server --port 6379 --save '' --appendonly no
  wait_command "Redis" 15 redis-cli ping
else
  echo "The current Redis is used."
fi
redis-cli set command_authority position_based >/dev/null

start_process gzserver "$LOG_DIR/gzserver.log" gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD_FILE"
gzserver_pid="$STARTED_PID"
gzserver_ticks="$STARTED_TICKS"
wait_process_command "Gazebo modelleri" 90 "$gzserver_pid" "$gzserver_ticks" "$LOG_DIR/gzserver.log" models_ready
if [[ "$HEADLESS" -eq 0 ]]; then
  start_process gzclient "$LOG_DIR/gzclient.log" gzclient --verbose
fi

if [[ "${YILDIZ_GAZEBO_ONLY:-0}" == 1 ]]; then
  trap - ERR INT TERM
  echo "Only Gazebo diagnostic mode is ready."
  exit 0
fi

# --- COPTERS: -I0..-I(N-1), SysID 1..N ---
for ((i=0; i<DRONE_COUNT; i++)); do
  sysid=$((i + 1))
  work="$RUN_DIR/sitl$i"
  mkdir -p "$work"
  start_process "ardupilot_drone_$sysid" "$LOG_DIR/ardupilot_drone_$sysid.log" \
    "$SCRIPT_DIR/scripts/ardupilot_supervisor.sh" "$work" \
    "$ARDUPILOT_DIR/build/sitl/bin/arducopter" --model gazebo-iris --speedup 1 \
    --sysid "$sysid" --slave 0 \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$COPTER_PARAM" \
    --sim-address=127.0.0.1 -I"$i" --home "$HOME_POS"
done

# --- TARGET FIXED WING: -I5, SysID 6 ---
plane_instance=5
plane_sysid=6
mkdir -p "$RUN_DIR/sitl$plane_instance"
start_process "ardupilot_target" "$LOG_DIR/ardupilot_target.log" \
  "$SCRIPT_DIR/scripts/ardupilot_supervisor.sh" "$RUN_DIR/sitl$plane_instance" \
  "$ARDUPILOT_DIR/build/sitl/bin/arduplane" --model gazebo-plane --speedup 1 \
  --sysid "$plane_sysid" --slave 0 \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane.parm,$TARGET_PARAM" \
  --sim-address=127.0.0.1 -I"$plane_instance" --home "$HOME_POS"

# --- MAVPROXY bridges --- For each vehicle: 14550 (QGC common) + config.py connection_string +
# companion_string
for ((i=0; i<DRONE_COUNT; i++)); do
  sysid=$((i + 1))
  start_process "mavproxy_drone_$sysid" "$LOG_DIR/mavproxy_drone_$sysid.log" \
    "$SCRIPT_DIR/scripts/mavproxy_supervisor.sh" "$RUN_DIR/sitl$i" \
    --non-interactive --retries 30 --streamrate "${YILDIZ_STREAMRATE:-20}" \
    --master "tcp:127.0.0.1:$((5760 + i * 10))" --sitl "127.0.0.1:$((5501 + i * 10))" \
    --out "udp:127.0.0.1:14550" \
    --out "udp:127.0.0.1:$((14551 + i * 10))" \
    --out "udp:127.0.0.1:$((14651 + i * 10))" \
    --out "udp:127.0.0.1:$((14652 + i * 10))" \
    --out "udp:127.0.0.1:$((14653 + i * 10))" \
    --out "udp:127.0.0.1:$((14654 + i * 10))"
done

start_process "mavproxy_target" "$LOG_DIR/mavproxy_target.log" \
  "$SCRIPT_DIR/scripts/mavproxy_supervisor.sh" "$RUN_DIR/sitl$plane_instance" \
  --non-interactive --retries 30 --streamrate "${YILDIZ_TARGET_STREAMRATE:-15}" \
  --master "tcp:127.0.0.1:$((5760 + plane_instance * 10))" \
  --sitl "127.0.0.1:$((5501 + plane_instance * 10))" \
  --out "udp:127.0.0.1:14550" \
  --out "udp:127.0.0.1:14601" \
  --out "udp:127.0.0.1:14602" \
  --out "udp:127.0.0.1:14603" \
  --out "udp:127.0.0.1:14604"

python3 "$SCRIPT_DIR/scripts/wait_heartbeat.py" --drones "$DRONE_COUNT" --timeout 120
assert_owned_alive

if [[ -n "$TARGET_PLAN" && -f "$TARGET_PLAN" ]]; then
  "$SCRIPT_DIR/scripts/load_plan.py" --plan "$TARGET_PLAN" --ports "14602:$plane_sysid"
  assert_owned_alive
fi

# --- Color-detecting bridge: camera -> Redis 'tracker_bbox' ---
if [[ "${YILDIZ_BBOX:-1}" != 0 ]]; then
  bbox_command=(python3 "$SCRIPT_DIR/bbox_to_redis.py" --topic "$CAM_TOPIC"
                --mavlink-port "${YILDIZ_GIMBAL_PORT:-14653}"
                --mount "$YILDIZ_MOUNT"
                --back "$YILDIZ_BACK" --down "$YILDIZ_DOWN")
  # PHYSICAL GIMBAL (gimbal branch): tilt = atan(down/back) from standoff_geom. YILDIZ_TILT_ENABLED=0 ->
  # legacy body-fixed chain (--mount/--aim).
  if [[ "${YILDIZ_TILT_ENABLED:-1}" == 0 ]]; then
    bbox_command+=(--no-tilt)
  else
    [[ -n "${YILDIZ_TILT:-}" ]] && bbox_command+=(--tilt "$YILDIZ_TILT")
  fi
  # Initialize aim analytically from --back / --down, then apply slow trim.
  # Set YILDIZ_AIM to override it. Tilt mode does not use aim or aim trim.
  [[ -n "${YILDIZ_AIM:-}" ]] && bbox_command+=(--aim "$YILDIZ_AIM")
  [[ "${YILDIZ_AIM_TRIM:-1}" == 0 ]] && bbox_command+=(--no-aim-trim)
  # Gimbal logging is enabled by default (2026-08-07). Previously, the
  # bbox_to_redis flag was omitted, so its 21-column command/status CSV was
  # never written. For persistent negative ey_deg, first compare tilt_cmd_deg
  # and tilt_status_deg (guidance_allstar/LOG_DICTIONARY.md, section 8).
  # Each run gets a timestamped file so the next launch cannot overwrite its
  # diagnostic record. Override with YILDIZ_GIMBAL_LOG=<path>; use 0 to disable.
  GIMBAL_LOG="${YILDIZ_GIMBAL_LOG:-$LOG_DIR/gimbal_$(date +%Y%m%d_%H%M%S).csv}"
  [[ "$GIMBAL_LOG" != 0 ]] && bbox_command+=(--gimbal-log "$GIMBAL_LOG")
  [[ -n "${YILDIZ_ATTITUDE_LOG:-}" ]] && bbox_command+=(--attitude-log "$YILDIZ_ATTITUDE_LOG")
  bbox_command+=(--camera-latency-ms "${YILDIZ_CAMERA_LATENCY_MS:-80}")
  [[ "$HEADLESS" -eq 1 ]] && bbox_command+=(--no-display)
  # Record video by default to preserve visual evidence from manual tests.
  # Explicitly set YILDIZ_VIDEO=0 to disable recording for disk/budget experiments.
  if [[ "${YILDIZ_VIDEO:-1}" != 0 ]]; then
    bbox_command+=(--record)
    echo "Video recording on: $SCRIPT_DIR/videos/"
  fi
  echo "Standoff geometry: mount=${YILDIZ_MOUNT} trim=${YILDIZ_PITCH_TRIM} deg" \
       "-> back=${YILDIZ_BACK} m down=${YILDIZ_DOWN} m"
  start_process bbox "$LOG_DIR/bbox.log" "${bbox_command[@]}"
fi

if [[ "$HEADLESS" -eq 0 && -x "$QGC_BIN" ]]; then
  start_process qgroundcontrol "$LOG_DIR/qgroundcontrol.log" "$QGC_BIN"
fi

trap - ERR INT TERM
echo "Yildizlar is ready. To stop: $SCRIPT_DIR/yildizlar_guidance.sh --stop"
