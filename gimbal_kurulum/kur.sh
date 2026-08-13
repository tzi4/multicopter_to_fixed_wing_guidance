#!/usr/bin/env bash
# =====================================================================
# gimbal_kurulum/kur.sh - FIZIKSEL GIMBALI ardupilot_gazebo'ya KURAR
# =====================================================================
# Gimbal iki parcadan olusur ve IKINCISI proje deposunun DISINDA yasar:
#   1. Bu repo: models/suru_drone_*/model.sdf (gimbal include'lu),
#      bbox_to_redis --tilt zinciri, tools/gz_gimbal.py, testler.
#   2. ~/ardupilot_gazebo: GimbalSmall2dPlugin (Gazebo 11 portu +
#      stabilize + hiz-servo) ve gimbal_small_2d modeli (mesh'ler dahil;
#      tilt collision'i mesh kullandigi icin mesh'ler GEREKLI).
# Bu script 2. parcayi bu repodaki kopyadan kurar ve derler.
#
# KULLANIM:
#   ./gimbal_kurulum/kur.sh                # varsayilan ~/ardupilot_gazebo
#   ARDUPILOT_GAZEBO_DIR=/baska/yol ./gimbal_kurulum/kur.sh
#
# Kurulumdan sonra dogrulama (roscore/gazebo gerektirir, ~2 dk):
#   python3 tools/gimbal_headless_test.py     -> SONUC: PASS beklenir
# Tam ucus dogrulamasi (SITL, ~4 dk):
#   python3 tools/gimbal_ucus_test.py         -> SONUC: PASS beklenir
# =====================================================================
set -Eeuo pipefail
BURASI="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AP_GZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"

[[ -d "$AP_GZ/src" && -f "$AP_GZ/CMakeLists.txt" ]] || {
  echo "HATA: $AP_GZ bir ardupilot_gazebo kaynagi degil." >&2
  echo "      ARDUPILOT_GAZEBO_DIR ile dogru yolu verin" >&2
  echo "      (ArduPilotPlugin'in derlendigi agac; SwiftGust/khancyr turevi)." >&2
  exit 1
}

echo ">>> plugin kaynaklari kopyalaniyor"
cp -v "$BURASI/src/GimbalSmall2dPlugin.cc" "$AP_GZ/src/"
cp -v "$BURASI/include/GimbalSmall2dPlugin.hh" "$AP_GZ/include/"
cp -v "$BURASI/src/gz_tilt_pub.cc" "$AP_GZ/src/"

echo ">>> gimbal_small_2d modeli kopyalaniyor (mesh'ler dahil)"
mkdir -p "$AP_GZ/models"
rm -rf "$AP_GZ/models/gimbal_small_2d"
cp -r "$BURASI/gimbal_small_2d" "$AP_GZ/models/"

echo ">>> CMakeLists: Gazebo<8 kapisi kaldiriliyor (varsa)"
python3 - "$AP_GZ/CMakeLists.txt" <<'PYEOF'
import sys
yol = sys.argv[1]
s = open(yol).read()
kapi = '''if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
    target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
    install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()'''
serbest = '''add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})'''
if kapi in s:
    open(yol, 'w').write(s.replace(kapi, serbest, 1))
    print("  kapi kaldirildi")
elif 'add_library(GimbalSmall2dPlugin' in s and 'VERSION_LESS "8.0")\n    add_library(GimbalSmall2dPlugin' not in s:
    print("  zaten serbest, dokunulmadi")
else:
    print("  UYARI: beklenen kalip bulunamadi. CMakeLists'te")
    print("  GimbalSmall2dPlugin hedefini Gazebo-11'de de derlenecek hale")
    print("  ELLE getirin (VERSION_LESS 8.0 kosulunun disina cikarin).")

# gz_tilt_pub hedefi (Faz C kalici yayinci) yoksa ekle
if 'gz_tilt_pub' not in s:
    s = open(yol).read()
    s += ('\n# gimbal_kurulum/kur.sh ekledi: Faz C kalici tilt yayincisi\n'
          'add_executable(gz_tilt_pub src/gz_tilt_pub.cc)\n'
          'target_link_libraries(gz_tilt_pub ${GAZEBO_LIBRARIES})\n')
    open(yol, 'w').write(s)
    print("  gz_tilt_pub hedefi eklendi")
else:
    print("  gz_tilt_pub hedefi zaten var")
PYEOF

echo ">>> derleniyor"
mkdir -p "$AP_GZ/build"
cd "$AP_GZ/build"
cmake .. > /dev/null
make GimbalSmall2dPlugin gz_tilt_pub 2>&1 | tail -3
[[ -f libGimbalSmall2dPlugin.so ]] || { echo "HATA: plugin derlemesi basarisiz" >&2; exit 1; }
[[ -f gz_tilt_pub ]] || { echo "HATA: gz_tilt_pub derlemesi basarisiz" >&2; exit 1; }

echo
echo "TAMAM: $AP_GZ/build/libGimbalSmall2dPlugin.so hazir."
echo "Not: yildizlar_gudum.sh GAZEBO_PLUGIN_PATH ve GAZEBO_MODEL_PATH'i"
echo "zaten $AP_GZ uzerinden kuruyor; ek ayar gerekmez."
echo "Dogrulama: python3 tools/gimbal_headless_test.py"
