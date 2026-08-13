#!/usr/bin/env bash
# =====================================================================
# yildizlar_gudum.sh - ANA BASLATICI (5 kopter suru + 1 hedef sabit kanat)
# =====================================================================
# Ortam kaynagi: arkadastan gelen all_star_env.zip (suru.world + 5 kamerali
# iris + gazebo-plane6). Baslatici iskeleti bumblebee_gudum.sh'ten tasindi
# (pid-file sahipligi, preflight, supervisor'lu SITL/MAVProxy, --stop).
#
# ARAC / PORT SOZLESMESI  (config.py ile birebir uyumlu tutulmali)
#   arac     -I  SysID  TCP    RCin  FDM(in/out)  MAVProxy --out
#   drone_1   0    1    5760   5501   9002/9003   14550, 14551, 14651, 14652, 14653, 14654(goruntulu)
#   drone_2   1    2    5770   5511   9012/9013   14550, 14561, 14661, 14662
#   drone_3   2    3    5780   5521   9022/9023   14550, 14571, 14671, 14672
#   drone_4   3    4    5790   5531   9032/9033   14550, 14581, 14681, 14682
#   drone_5   4    5    5800   5541   9042/9043   14550, 14591, 14691, 14692
#   hedef     5    6    5810   5551   9052/9053   14550, 14601, 14602, 14603
#   (son sutun: QGC ortak / tools / wait+plan / guidance_allstar / bbox-gimbal)
#   (FDM portlari models/*/model.sdf icindeki <fdm_port_in> ile ESLESMELI;
#    SITL bunlari -I N ile 9002+10N / 9003+10N olarak kendisi turetir.)
#
# WSL2 TUZAGI: sim_vehicle.py kullanilmiyor - onun varsayilan --out'u
# Windows host'una gider ve 14551 dinleyicisi bos kalir. Burada SITL
# binary'si + MAVProxy ACIK --out listesiyle elle baglanir.
#
# KULLANIM:
#   ./yildizlar_gudum.sh              # GUI (gzclient + QGC + pencereli bbox)
#   ./yildizlar_gudum.sh --headless   # GUI yok, bbox --no-display
#   ./yildizlar_gudum.sh --iris       # mevcut Iris modeli (varsayilan)
#   ./yildizlar_gudum.sh --hummingbird # RotorS Hummingbird modeli
#   ./yildizlar_gudum.sh --robofly    # CTU-MRS RoboFly modeli
#   ./yildizlar_gudum.sh --stop       # bu paketin baslattigi her seyi kapatir
#   ./yildizlar_gudum.sh                  # bbox'li kamera videosu varsayilan ACIK
#   YILDIZ_VIDEO=0 ./yildizlar_gudum.sh   # video kaydini bilerek kapat
#   YILDIZ_GAZEBO_ONLY=1 ./yildizlar_gudum.sh  # yalniz Gazebo (tani modu)
#   YILDIZ_DRONES=1 ./yildizlar_gudum.sh  # 5 yerine yalniz drone_1 (hafif)
# =====================================================================
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/run"
LOG_DIR="$SCRIPT_DIR/logs"
PID_FILE="$RUN_DIR/pids"
LOCK_FILE="$RUN_DIR/launcher.lock"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
IQ_SIM_MODELS="${IQ_SIM_MODELS:-${HOME}/catkin_ws/src/iq_sim/models}"
QGC_BIN="${QGC_BIN:-${HOME}/Applications/QGroundControl.AppImage}"

# Varsayilan dunya TEK KOPTERLI: suru.world'un iris-2..5'i cikarilmis surumu.
# Cikarilan her iris 5 Hz'lik bir kamera render'i demekti; suru gerekince
# YILDIZ_WORLD=worlds/suru.world (ve YILDIZ_DRONES=5) ile geri donulur.
WORLD_FILE="${YILDIZ_WORLD:-$SCRIPT_DIR/worlds/tek_avci.world}"
COPTER_PARAM="${YILDIZ_COPTER_PARAM:-$SCRIPT_DIR/params/swarm_copter.parm}"
TARGET_PARAM="${YILDIZ_TARGET_PARAM:-$SCRIPT_DIR/params/hedef_ucak.parm}"
# Hedef ucagin AUTO plani. Bos birakilirsa plan yuklenmez.
# VARSAYILAN ELIPS: guidance-allstar'in IMM kestiricisi (CTxy modu) surekli
# donen bir hedefte anlamli calisir; dikdortgen rotanin koseleri disinda
# hedef duz uctugu icin donus modu hic uyanmiyordu. hedef_tur.plan (dortgen)
# YILDIZ_TARGET_PLAN ile hala secilebilir.
TARGET_PLAN="${YILDIZ_TARGET_PLAN:-$SCRIPT_DIR/missions/hedef_elips.plan}"
# Tum araclar AYNI home'u kullanir: Gazebo dunya orijini burasidir, araclarin
# birbirine gore konumu world dosyasindaki <pose> degerlerinden gelir.
# Public ArduPilot SITL/CMAC referansi; YILDIZ_HOME ile degistirilebilir.
HOME_POS="${YILDIZ_HOME:--35.363261,149.165230,0,0}"
# Kac kopter kaldirilacak (1..5). VARSAYILAN 1: bu asamada suru gerekmiyor,
# tek avci + tek hedef yeterli ve CPU'yu bosaltiyor (world dosyasi 5 iris'i
# yine yukler ama SITL/MAVProxy yalniz drone_1 icin kosar).
# YILDIZ_DRONES=5 ile arkadasin orijinal suru kurulumuna donulur.
DRONE_COUNT="${YILDIZ_DRONES:-1}"
# Kopter fizik modeli. Iris geriye donuk uyumluluk icin varsayilandir.
# Komut satirindaki --iris/--hummingbird/--robofly bu ortam degiskenini ezer.
DRONE_MODEL="${YILDIZ_DRONE_MODEL:-iris}"
# MAVPROXY STREAMRATE: MAVProxy'nin varsayilani 4 Hz'dir ve bunu PERIYODIK
# olarak REQUEST_DATA_STREAM ile dayatir; bizim SET_MESSAGE_INTERVAL'imizi
# EZER. Olculdu: ATTITUDE 50 Hz istenmesine ragmen 5.9 Hz geliyordu
# (ornekler arasi ortanca 249 ms). Sanal gimbal tutumu KARE ANINA
# interpolasyonla tasidigi icin bu aralik dogrudan hataya donusuyor.
# AVCI (drone) icin 20 Hz: gudum dongusu 20 Hz kosar, dolayisiyla bundan
# fazlasi ise yaramaz. 50 idi (2026-08-02'de dusuruldu): 5 ayri UDP
# cikisina 50 Hz basmak gudum okuma is parcacigini bogup dongunun 2.2 Hz'e
# dusmesine katkida bulunuyordu.
# HEDEF (ucak) icin 15 Hz: bu MAVProxy'ye --streamrate HIC verilmiyordu, yani
# 4 Hz varsayilani gudumun istedigi 15 Hz'lik GLOBAL_POSITION_INT'i eziyor ve
# hedef telemetrisi 4-5 Hz'de kaliyordu (kesim ve olu hesap butun ayari bunun
# uzerine kuruludur). YILDIZ_STREAMRATE / YILDIZ_TARGET_STREAMRATE ile degisir.
# KAMERA BORU HATTI GECIKMESI: 80 ms. Kare bize ULASTIGINDA zaten
# ~80 ms once YAKALANMISTIR; tutum o ana interpolasyonla tasinir.
# tools/gimbal_zaman_kalibre.py ile OLCULDU (tarama -40..200 ms,
# olcut |corr(stab_ey, pitch)| minimumu): 0 ms -> 0.253, 80 ms -> 0.232.
# SANAL GIMBAL: YILDIZ_MOUNT = fiziksel montaj (+30 yukari, model.sdf ile
# ayni). YILDIZ_AIM = "hedef kadrajda nerede dursun" DC ofseti; sanal
# kadraj merkezi ufka gore -AIM olur (yildizlar_gimbal.py'de ispatlandi).
# -27, konumlu gudumun standoff geometrisinden gelir:
#   aim = -atan(down/back) = -atan(13/25) = -27.5
# Olcum: elips -29.25, duz -27.03, wanderer -10.52 (wanderer irtifa
# degistirdigi icin dagilimi genis). BACK/DOWN degisirse YENIDEN hesapla.
# AIM YALNIZ DIKEY KANALI ETKILER (2026-08-02'de duzeltildi): R_aim bir Ry
# donusu oldugu icin eskiden yatay aci da onunla birlikte donuyor ve kerteriz
# cos(eps)/cos(eps+aim) kadar SIKISIYORDU. gimbal4.csv'de olculdu: kazanc
# 0.910, yani gudum kerterizi %8.8 DUSUK okuyup hedefe donusu eksik
# komutluyordu. yildizlar_gimbal.py:aci_hatasi artik yatay bileseni AIM ONCESI
# isindan okuyor -> ayni karelerde kazanc 1.004. Yatay eksende daha once
# "acik sorun" diye raporlanan roll korelasyonu ise SAHTE alarmdi:
# bank-to-turn'de roll kerteriz hatasindan komutlandigi icin endojendir.
# Yer gercegiyle dogrulandi (corr(stab_ex,-gercek_yan)=0.992, kalinti
# sin(roll) egimi -0.15 deg, sacilim 0.28 deg, montaj +30.0 teyit).
#
# STANDOFF DIKEY GEOMETRISI ARTIK MONTAJ ACISINDAN TURETILIR: asagidaki
# kaynaklama YILDIZ_MOUNT (varsayilan 30, model.sdf ile AYNI olmali),
# YILDIZ_PITCH_TRIM (varsayilan -2.5 deg, kopterin duragan takipteki tipik
# pitch'i), YILDIZ_BACK ve bunlardan turetilen YILDIZ_DOWN'u disa aktarir
# (down = round(back * tan(mount + pitch_trim)) -> 25 m icin 13 m).
# Ayrintili gerekce ve ucus kaniti scripts/standoff_geom.sh basindadir.
# Elle ezmek icin: YILDIZ_BACK / YILDIZ_DOWN / YILDIZ_MOUNT / YILDIZ_PITCH_TRIM.
# shellcheck source=scripts/standoff_geom.sh
source "$SCRIPT_DIR/scripts/standoff_geom.sh"

# bbox dedektorunun dinleyecegi kamera topic'i.
CAM_TOPIC="${YILDIZ_CAM_TOPIC:-/drone_1/webcam/image_raw}"

HEADLESS=0
MODE=start

usage() {
  cat <<'KULLANIM'
Kullanim: yildizlar_gudum.sh [--iris|--hummingbird|--robofly] [--headless] [--gimbalsiz] | --stop

  --iris       Mevcut Iris fizik modelini kullan (varsayilan)
  --hummingbird RotorS Hummingbird fizik modelini kullan
  --robofly    CTU-MRS RoboFly fizik modelini kullan
  --headless    GUI yok (gzclient/QGC baslatilmaz)
  --gimbalsiz   Kamera GOVDEYE SABIT (gimbal oncesi davranis).
                Varsayilan: DIKEY GIMBAL acik.
                Ayni sey: YILDIZ_GIMBAL=0 ./yildizlar_gudum.sh
  --stop        Bu paketin baslattigi her seyi kapatir

Bayraklar birlikte kullanilabilir:  ./yildizlar_gudum.sh --robofly --headless
Ortam degiskeni karsiligi: YILDIZ_DRONE_MODEL=iris|hummingbird|robofly
KULLANIM
}

# GIMBAL: VARSAYILAN ACIK (1). --gimbalsiz ile govdeye sabit kameraya donulur;
# o zaman models_sabit/ (tools/gimbalsiz_uret.py urunu) GAZEBO_MODEL_PATH'in
# BASINA konur ve Gazebo 'model://suru_drone_N'i once orada bulur.
# Dunya dosyasi ve asil models/ agaci DEGISMEZ.
GIMBAL="${YILDIZ_GIMBAL:-1}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --iris) DRONE_MODEL=iris ;;
    --hummingbird) DRONE_MODEL=hummingbird ;;
    --robofly) DRONE_MODEL=robofly ;;
    --headless) HEADLESS=1 ;;
    --gimbalsiz) GIMBAL=0 ;;
    --stop) MODE=stop ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done

if [[ "$DRONE_MODEL" != iris && "$DRONE_MODEL" != hummingbird && "$DRONE_MODEL" != robofly ]]; then
  echo "YILDIZ_DRONE_MODEL iris, hummingbird veya robofly olmali (verilen: $DRONE_MODEL)" >&2
  exit 2
fi

if ! [[ "$DRONE_COUNT" =~ ^[1-5]$ ]]; then
  echo "YILDIZ_DRONES 1..5 arasinda olmali (verilen: $DRONE_COUNT)" >&2
  exit 2
fi

mkdir -p "$RUN_DIR" "$LOG_DIR" "$RUN_DIR/ros" "$SCRIPT_DIR/videos"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  if [[ "$MODE" != stop ]]; then
    echo "Baslatici baska bir islem tarafindan kullaniliyor." >&2
    exit 1
  fi
fi

pid_matches() {
  local pid="$1" expected_ticks="$2" actual_ticks
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  actual_ticks="$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || true)"
  [[ "$actual_ticks" == "$expected_ticks" ]]
}

stop_owned() {
  [[ -f "$PID_FILE" ]] || { echo "Surec kaydi yok."; return 0; }
  mapfile -t records < "$PID_FILE"
  local index name pid ticks
  for ((index=${#records[@]}-1; index>=0; index--)); do
    read -r name pid ticks <<< "${records[$index]}"
    if pid_matches "$pid" "$ticks"; then
      echo "$name durduruluyor (PID $pid)"
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

if [[ "$MODE" == stop ]]; then
  stop_owned
  exit 0
fi

if [[ -f "$PID_FILE" ]]; then
  while read -r _ pid ticks; do
    if pid_matches "$pid" "$ticks"; then
      echo "Sistem zaten calisiyor. Once --stop kullanin." >&2
      exit 1
    fi
  done < "$PID_FILE"
  rm -f -- "$PID_FILE"
fi

preflight() {
  local failed=0 command path index
  for command in python3 gzserver redis-cli mavproxy.py setsid flock ss; do
    command -v "$command" >/dev/null 2>&1 || { echo "Eksik komut: $command" >&2; failed=1; }
  done
  for path in "$WORLD_FILE" "$COPTER_PARAM" "$TARGET_PARAM" \
    "$ARDUPILOT_DIR/build/sitl/bin/arducopter" "$ARDUPILOT_DIR/build/sitl/bin/arduplane" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
    "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane.parm" \
    "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" \
    "$SCRIPT_DIR/models/gazebo-plane6/model.sdf" \
    /opt/ros/noetic/setup.bash "$ARDUPILOT_GAZEBO_DIR/devel/setup.bash"; do
    [[ -e "$path" ]] || { echo "Eksik bagimlilik: $path" >&2; failed=1; }
  done
  for ((index=1; index<=DRONE_COUNT; index++)); do
    [[ -e "$SELECTED_MODEL_ROOT/suru_drone_$index/model.sdf" ]] || {
      echo "Eksik bagimlilik: $SELECTED_MODEL_ROOT/suru_drone_$index/model.sdf" >&2; failed=1; }
  done
  if [[ "$DRONE_MODEL" != iris ]]; then
    for command in cmake protoc; do
      command -v "$command" >/dev/null 2>&1 || { echo "Eksik komut: $command" >&2; failed=1; }
    done
    for path in \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/CMakeLists.txt" \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/ArduPilotMotorBridge.cc"; do
      [[ -e "$path" ]] || { echo "Eksik bagimlilik: $path" >&2; failed=1; }
    done
  fi
  if [[ "$DRONE_MODEL" == hummingbird ]]; then
    [[ -e "$SCRIPT_DIR/models_hummingbird/hummingbird/model.sdf" ]] || {
      echo "Eksik bagimlilik: $SCRIPT_DIR/models_hummingbird/hummingbird/model.sdf" >&2; failed=1; }
  elif [[ "$DRONE_MODEL" == robofly ]]; then
    for path in \
      "$SCRIPT_DIR/models_robofly/robofly/model.sdf" \
      "$SCRIPT_DIR/plugins/hummingbird_bridge/upstream/fluid_resistance_plugin.cpp"; do
      [[ -e "$path" ]] || { echo "Eksik bagimlilik: $path" >&2; failed=1; }
    done
  fi
  if [[ "$HEADLESS" -eq 0 ]]; then
    command -v gzclient >/dev/null 2>&1 || { echo "Eksik komut: gzclient" >&2; failed=1; }
    [[ -x "$QGC_BIN" ]] || echo "UYARI: QGroundControl yok ($QGC_BIN), atlaniyor." >&2
  fi
  [[ "$failed" -eq 0 ]] || return 1

  # Kullanacagimiz portlar bos mu? (baska bir SITL/QGC acikken sessiz
  # carpismalar yerine burada patlamasi tercih edilir)
  local wanted occupied
  wanted='^(5760|5770|5780|5790|5800|5810|5501|5511|5521|5531|5541|5551|9002|9012|9022|9032|9042|9052|14551|14561|14571|14581|14591|14601)$'
  occupied="$(ss -H -lntu | awk '{print $5}' | sed 's/.*://' | grep -E "$wanted" | sort -u || true)"
  if [[ -n "$occupied" ]]; then
    echo "Gerekli portlar kullanimda: $(echo "$occupied" | paste -sd, -)" >&2
    return 1
  fi
}

build_external_multirotor_plugins() {
  local source_dir="$SCRIPT_DIR/plugins/hummingbird_bridge"
  local build_dir="$source_dir/build"
  echo ">>> ${DRONE_MODEL^^}: motor ve govde eklentileri denetleniyor/derleniyor"
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
# Bu paketin modelleri EN BASTA: models/gazebo-plane6 iq_sim'deki ayni adli
# (link adlari 'gazebo-plane::' kalmis, bozuk) surumu bilerek ezer.
case "$DRONE_MODEL" in
  hummingbird) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models_hummingbird" ;;
  robofly) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models_robofly" ;;
  *) SELECTED_MODEL_ROOT="$SCRIPT_DIR/models" ;;
esac

if [[ "$GIMBAL" -eq 0 ]]; then
  # GIMBALSIZ KOL: secilen govde modelinin sabit-kamera agaci EN BASTA.
  python3 "$SCRIPT_DIR/tools/gimbalsiz_uret.py" --arac "$DRONE_MODEL" || exit 1
  case "$DRONE_MODEL" in
    hummingbird) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_hummingbird_sabit" ;;
    robofly) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_robofly_sabit" ;;
    *) FIXED_MODEL_ROOT="$SCRIPT_DIR/models_sabit" ;;
  esac
  export GAZEBO_MODEL_PATH="$FIXED_MODEL_ROOT:$SELECTED_MODEL_ROOT:$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$IQ_SIM_MODELS"
  echo ">>> KAMERA: GOVDEYE SABIT (--gimbalsiz) -- $(basename "$FIXED_MODEL_ROOT")/ kullaniliyor"
else
  export GAZEBO_MODEL_PATH="$SELECTED_MODEL_ROOT:$SCRIPT_DIR/models:$ARDUPILOT_GAZEBO_DIR/models:$IQ_SIM_MODELS"
  echo ">>> KAMERA: DIKEY GIMBAL (varsayilan) -- gimbalsiz icin --gimbalsiz"
fi
EXTERNAL_MULTIROTOR_PLUGIN_DIR="$SCRIPT_DIR/plugins/hummingbird_bridge/build"
if [[ "$DRONE_MODEL" != iris ]]; then
  export GAZEBO_PLUGIN_PATH="$EXTERNAL_MULTIROTOR_PLUGIN_DIR:$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="$EXTERNAL_MULTIROTOR_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"
else
  export GAZEBO_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
fi
echo ">>> KOPTER MODELI: ${DRONE_MODEL^^}"
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
    (( SECONDS >= deadline )) && { echo "$description zaman asimi." >&2; return 1; }
    sleep 0.5
  done
  echo "$description hazir."
}

wait_process_command() {
  local description="$1" timeout="$2" pid="$3" ticks="$4" log="$5"
  shift 5
  local deadline=$((SECONDS + timeout))
  until "$@" >/dev/null 2>&1; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "$description baslamadan surec sonlandi (PID $pid)." >&2
      tail -n 30 "$log" >&2 || true
      return 1
    fi
    (( SECONDS >= deadline )) && { echo "$description zaman asimi." >&2; return 1; }
    sleep 0.5
  done
  echo "$description hazir."
}

models_ready() {
  local output
  output="$(rosservice call /gazebo/get_world_properties 2>/dev/null || true)"
  grep -q 'iris-1' <<< "$output" && grep -q 'hedef' <<< "$output"
}

assert_owned_alive() {
  local name pid ticks
  while read -r name pid ticks; do
    if ! pid_matches "$pid" "$ticks"; then
      echo "$name beklenmedik bicimde sonlandi (PID $pid)." >&2
      return 1
    fi
  done < "$PID_FILE"
}

trap 'echo "Baslatma basarisiz; bu pakete ait surecler kapatiliyor." >&2; stop_owned >/dev/null 2>&1 || true' ERR INT TERM
preflight
if [[ "$DRONE_MODEL" != iris ]]; then
  build_external_multirotor_plugins
fi
: > "$PID_FILE"

if ! rosparam list >/dev/null 2>&1; then
  start_process roscore "$LOG_DIR/roscore.log" roscore
  wait_command "ROS master" 20 rosparam list
else
  echo "Mevcut ROS master kullaniliyor."
fi

if ! redis-cli ping 2>/dev/null | grep -q PONG; then
  start_process redis "$LOG_DIR/redis.log" redis-server --port 6379 --save '' --appendonly no
  wait_command "Redis" 15 redis-cli ping
else
  echo "Mevcut Redis kullaniliyor."
fi
redis-cli set komut_yetkisi konumlu >/dev/null

start_process gzserver "$LOG_DIR/gzserver.log" gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD_FILE"
gzserver_pid="$STARTED_PID"
gzserver_ticks="$STARTED_TICKS"
wait_process_command "Gazebo modelleri" 90 "$gzserver_pid" "$gzserver_ticks" "$LOG_DIR/gzserver.log" models_ready
if [[ "$HEADLESS" -eq 0 ]]; then
  start_process gzclient "$LOG_DIR/gzclient.log" gzclient --verbose
fi

if [[ "${YILDIZ_GAZEBO_ONLY:-0}" == 1 ]]; then
  trap - ERR INT TERM
  echo "Yalniz Gazebo tani modu hazir."
  exit 0
fi

# --- KOPTERLER: -I0..-I(N-1), SysID 1..N ---
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

# --- HEDEF SABIT KANAT: -I5, SysID 6 ---
plane_instance=5
plane_sysid=6
mkdir -p "$RUN_DIR/sitl$plane_instance"
start_process "ardupilot_hedef" "$LOG_DIR/ardupilot_hedef.log" \
  "$SCRIPT_DIR/scripts/ardupilot_supervisor.sh" "$RUN_DIR/sitl$plane_instance" \
  "$ARDUPILOT_DIR/build/sitl/bin/arduplane" --model gazebo-plane --speedup 1 \
  --sysid "$plane_sysid" --slave 0 \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-plane.parm,$TARGET_PARAM" \
  --sim-address=127.0.0.1 -I"$plane_instance" --home "$HOME_POS"

# --- MAVPROXY koprüleri ---
# Her arac icin: 14550 (QGC ortak) + config.py connection_string + companion_string
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

start_process "mavproxy_hedef" "$LOG_DIR/mavproxy_hedef.log" \
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

# --- Renk-tespit koprusu: kamera -> Redis 'tracker_bbox' ---
if [[ "${YILDIZ_BBOX:-1}" != 0 ]]; then
  bbox_command=(python3 "$SCRIPT_DIR/bbox_to_redis.py" --topic "$CAM_TOPIC"
                --mavlink-port "${YILDIZ_GIMBAL_PORT:-14653}"
                --mount "$YILDIZ_MOUNT"
                --back "$YILDIZ_BACK" --down "$YILDIZ_DOWN")
  # FIZIKSEL GIMBAL (gimbal dali): tilt = atan(down/back) standoff_geom'dan.
  # YILDIZ_TILT_ACIK=0 -> eski govdeye-sabit zincir (--mount/--aim).
  if [[ "${YILDIZ_TILT_ACIK:-1}" == 0 ]]; then
    bbox_command+=(--no-tilt)
  else
    [[ -n "${YILDIZ_TILT:-}" ]] && bbox_command+=(--tilt "$YILDIZ_TILT")
  fi
  # aim artik --back/--down'dan ANALITIK baslatilir ve yavas trim ile
  # duzeltilir; sabitlemek icin YILDIZ_AIM ver. (tilt modunda aim/trim yok)
  [[ -n "${YILDIZ_AIM:-}" ]] && bbox_command+=(--aim "$YILDIZ_AIM")
  [[ "${YILDIZ_AIM_TRIM:-1}" == 0 ]] && bbox_command+=(--no-aim-trim)
  # GIMBAL LOGU ARTIK VARSAYILAN ACIK (2026-08-07). NICIN: bbox_to_redis
  # tilt komut-vs-gerceklesen CSV'sini (tilt_cmd_deg / tilt_status_deg,
  # 21 kolon) yazabiliyordu ama bu bayrak HIC verilmiyordu -- yani gimbal
  # zincirinin tek olcum kaydi hicbir kosuda uretilmedi. "ey_deg surekli
  # negatif" turu bir arizada (bkz. LOG_SOZLUGU.md §8) once bu iki kolona
  # bakilir; olmayinca teshis yapilamiyor. Damgali ad: her kosu kendi
  # dosyasini yazar, kaza kaydi bir sonraki kalkisla EZILMEZ.
  # YILDIZ_GIMBAL_LOG=<yol> ile yol degistirilir, =0 ile kapatilir.
  GIMBAL_LOG="${YILDIZ_GIMBAL_LOG:-$LOG_DIR/gimbal_$(date +%Y%m%d_%H%M%S).csv}"
  [[ "$GIMBAL_LOG" != 0 ]] && bbox_command+=(--gimbal-log "$GIMBAL_LOG")
  [[ -n "${YILDIZ_TUTUM_LOG:-}" ]] && bbox_command+=(--tutum-log "$YILDIZ_TUTUM_LOG")
  bbox_command+=(--kamera-gecikme-ms "${YILDIZ_KAMERA_GECIKME_MS:-80}")
  [[ "$HEADLESS" -eq 1 ]] && bbox_command+=(--no-display)
  # Video varsayilan ACIK: manuel kosuda YILDIZ_VIDEO=1 unutulunca testin
  # tek gorsel kaniti tamamen kayboluyordu. Disk/butce deneyi icin yalniz
  # acik bir YILDIZ_VIDEO=0 kaydi kapatir.
  if [[ "${YILDIZ_VIDEO:-1}" != 0 ]]; then
    bbox_command+=(--record)
    echo "Video kaydi acik: $SCRIPT_DIR/videos/"
  fi
  echo "Standoff geometrisi: mount=${YILDIZ_MOUNT} trim=${YILDIZ_PITCH_TRIM} deg" \
       "-> back=${YILDIZ_BACK} m down=${YILDIZ_DOWN} m"
  start_process bbox "$LOG_DIR/bbox.log" "${bbox_command[@]}"
fi

if [[ "$HEADLESS" -eq 0 && -x "$QGC_BIN" ]]; then
  start_process qgroundcontrol "$LOG_DIR/qgroundcontrol.log" "$QGC_BIN"
fi

trap - ERR INT TERM
echo "Yildizlar sistemi hazir. Durdurmak icin: $SCRIPT_DIR/yildizlar_gudum.sh --stop"
