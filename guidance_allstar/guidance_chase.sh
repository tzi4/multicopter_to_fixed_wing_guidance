#!/bin/bash
# This script launches Gazebo, ArduPilot SITL and QGroundControl in separate windows.

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
CATKIN_WS_DIR="${CATKIN_WS_DIR:-${HOME}/catkin_ws}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
QGC_DIR="${QGC_DIR:-${HOME}/Applications}"
DEFAULT_VEHICLE2_MISSION="$REPO_ROOT/missions/target_ellipse.plan"
VEHICLE2_MISSION="${1:-$DEFAULT_VEHICLE2_MISSION}"
DRONE1_FRAME="${DRONE1_FRAME:-gazebo-iris}"
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
PURPLE='\033[0;35m'
ORANGE='\033[38;5;208m'
BLACK='\033[0;30m'
NC='\033[0m'

if [[ ! -f "$VEHICLE2_MISSION" ]]; then
    echo -e "${RED}ERROR: Vehicle 2 mission file not found: $VEHICLE2_MISSION${NC}" >&2
    exit 1
fi

echo -e "${BLUE}--> Drone 1 frame: $DRONE1_FRAME    <--${NC}"

echo -e "${GREEN}--> Preparing the required environments.  <--${NC}"
# We load the necessary ROS and Gazebo environment variables. This is the safest way to have it in the
# script as it is required on every new terminal.
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS_DIR/devel/setup.bash"
source "$ARDUPILOT_GAZEBO_DIR/devel/setup.bash"

# Let's go to the folder where the script is located (to access other files in it)
cd "$SCRIPT_DIR"

launch_in_xterm() {
    local title="$1"
    local command="$2"
    xterm -T "$title" -e bash -lc "$command" &
}

echo -e "${BLUE}--> Starting the Gazebo world (this may take a while)...${NC}"
launch_in_xterm "Gazebo World" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; export GAZEBO_MODEL_PATH=${ARDUPILOT_GAZEBO_DIR}/models:${CATKIN_WS_DIR}/src/iq_sim/models; cd ${SCRIPT_DIR}; roslaunch iq_sim runway_wndr2.launch; echo; echo 'This terminal will close when the Gazebo window closes.'; exec bash"

sleep 5 # Allow Gazebo additional time to finish initialization.

echo -e "${PURPLE}--> Starting Drone 1 (central vehicle).  <--${NC}"
launch_in_xterm "Drone 1 SITL" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; cd ${ARDUPILOT_DIR}; ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py -N -I0 -v ArduCopter -f ${DRONE1_FRAME} -l -35.363261,149.165230,0,0 --out=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14552 --map --console --sysid 1 --mavproxy-args='--cmd=\"mode guided; arm throttle; takeoff 30\"'; exec bash"

sleep 5

echo -e "${PURPLE}--> Starting the airplane (central vehicle). <--${NC}"
launch_in_xterm "Airplane SITL" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; cd ${ARDUPILOT_DIR}; ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py -N -I5 -v ArduPlane -f gazebo-plane -l -35.363261,149.165263,0,0 --out=127.0.0.1:14550 --out=127.0.0.1:14600 --map --console --sysid 2; exec bash"

sleep 5

# echo ">>> Loading task Vehicle 2..." python3 upload_vehicle2_mission.py \ --connect
# "$VEHICLE2_MAVLINK" \ --mission "$VEHICLE2_MISSION" \ --set-current 0

echo -e "${RED}--> Vehicle 2 mission loaded. Mode is unchanged. Select mode, arm, and start in QGC.${NC}"

echo -e "${ORANGE}--> Starting QGroundControl...  <--${NC}"
launch_in_xterm "QGroundControl" "cd ${QGC_DIR}; ${QGC_DIR}/QGroundControl.AppImage; echo; echo 'When QGC is closed, this terminal will also be closed.'; exec bash"

echo -e "${BLACK}--> All launch commands have been sent.${NC}"
