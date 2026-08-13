#!/bin/bash
# Bu betik, Gazebo, ArduPilot SITL ve QGroundControl'ü ayrı pencerelerde başlatır.

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
CATKIN_WS_DIR="${CATKIN_WS_DIR:-${HOME}/catkin_ws}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
QGC_DIR="${QGC_DIR:-${HOME}/Applications}"
DEFAULT_VEHICLE2_MISSION="$REPO_ROOT/missions/hedef_elips.plan"
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
    echo -e "${RED}HATA: Vehicle 2 mission file bulunamadı: $VEHICLE2_MISSION${NC}" >&2
    exit 1
fi

echo -e "${BLUE}--> Drone 1 frame: $DRONE1_FRAME    <--${NC}"

echo -e "${GREEN}--> Gerekli ortamlar hazırlanıyor.  <--${NC}"
# Gerekli ROS ve Gazebo ortam değişkenlerini yüklüyoruz.
# Bu, her yeni terminalde gerektiği için betiğin içinde olması en garantili yoldur.
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS_DIR/devel/setup.bash"
source "$ARDUPILOT_GAZEBO_DIR/devel/setup.bash"

# Betiğin bulunduğu klasöre gidelim (içindeki diğer dosyalara erişim için)
cd "$SCRIPT_DIR"

launch_in_xterm() {
    local title="$1"
    local command="$2"
    xterm -T "$title" -e bash -lc "$command" &
}

echo -e "${BLUE}--> Gazebo Dünyası Başlatılıyor (Bu işlem biraz sürebilir)...${NC}"
launch_in_xterm "Gazebo World" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; export GAZEBO_MODEL_PATH=${ARDUPILOT_GAZEBO_DIR}/models:${CATKIN_WS_DIR}/src/iq_sim/models; cd ${SCRIPT_DIR}; roslaunch iq_sim runway_wndr2.launch; echo; echo 'Gazebo penceresi kapatıldığında bu terminal de kapanacaktır.'; exec bash"

sleep 5 # Gazebo'nun tam olarak kendine gelmesi için bekleme süresini biraz artırdım

echo -e "${PURPLE}--> Drone 1 (Merkez) Başlatılıyor.  <--${NC}"
launch_in_xterm "Drone 1 SITL" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; cd ${ARDUPILOT_DIR}; ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py -N -I0 -v ArduCopter -f ${DRONE1_FRAME} -l -35.363261,149.165230,0,0 --out=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14552 --map --console --sysid 1 --mavproxy-args='--cmd=\"mode guided; arm throttle; takeoff 30\"'; exec bash"

sleep 5

echo -e "${PURPLE}--> Airplane (Merkez) Başlatılıyor. <--${NC}"
launch_in_xterm "Airplane SITL" "source /opt/ros/noetic/setup.bash; source ${CATKIN_WS_DIR}/devel/setup.bash; source ${ARDUPILOT_GAZEBO_DIR}/devel/setup.bash; cd ${ARDUPILOT_DIR}; ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py -N -I5 -v ArduPlane -f gazebo-plane -l -35.363261,149.165263,0,0 --out=127.0.0.1:14550 --out=127.0.0.1:14600 --map --console --sysid 2; exec bash"

sleep 5

#echo ">>> Vehicle 2 görevi yükleniyor..."
#python3 upload_vehicle2_mission.py \
#    --connect "$VEHICLE2_MAVLINK" \
#    --mission "$VEHICLE2_MISSION" \
#    --set-current 0

echo -e "${RED}--> Vehicle 2 görevi yüklendi. Araç modu değiştirilmedi; QGC'den istediğin mod/arm/start işlemini yap.${NC}"

echo -e "${ORANGE}--> QGroundControl Başlatılıyor...  <--${NC}"
launch_in_xterm "QGroundControl" "cd ${QGC_DIR}; ${QGC_DIR}/QGroundControl.AppImage; echo; echo 'QGC kapatıldığında bu terminal de kapanacaktır.'; exec bash"

echo -e "${BLACK}--> Tüm başlatma komutları gönderildi.${NC}"
