#!/usr/bin/env bash
# ========================================================== gimbal_setup/install.sh - INSTALLS THE
# PHYSICAL GIMBAL ON ardupilot_gazebo
# ================================================================================== The gimbal
# consists of two parts and the SECOND one lives OUTSIDE the project repository: 1. This repo:
# models/swarm_drone_*/model.sdf (with gimbal include), bbox_to_redis --tilt chain,
# tools/gz_gimbal.py, tests. 2. ~/ardupilot_gazebo: GimbalSmall2dPlugin (Gazebo 11 port + stabilizer +
# speed-servo) and gimbal_small_2d model (meshes included; meshes REQUIRED because tilt collision uses
# mesh). This script is 2. It installs and compiles the part from the copy in this repo. USE:
# ./gimbal_setup/install.sh # default ~/ardupilot_gazebo ARDUPILOT_GAZEBO_DIR=/else/path
# ./gimbal_setup/install.sh Verification after installation (requires roscore/gazebo, ~2 min): python3
# tools/gimbal_headless_test.py -> RESULT: PASS expected Full flight verification (SITL, ~4 min):
# python3 tools/gimbal_flight_test.py -> RESULT: PASS expected
# ===========================================================================
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AP_GZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"

[[ -d "$AP_GZ/src" && -f "$AP_GZ/CMakeLists.txt" ]] || {
  echo "ERROR: $AP_GZ is not a source of ardupilot_gazebo." >&2
  echo "Give correct path with ARDUPILOT_GAZEBO_DIR" >&2
  echo "(compilation tree of ArduPilotPlugin; variant SwiftGust /khancyr)." >&2
  exit 1
}

echo ">>> Copying plugin sources"
cp -v "$HERE/src/GimbalSmall2dPlugin.cc" "$AP_GZ/src/"
cp -v "$HERE/include/GimbalSmall2dPlugin.hh" "$AP_GZ/include/"
cp -v "$HERE/src/gz_tilt_pub.cc" "$AP_GZ/src/"

echo ">>> Copying model gimbal_small_2d (including meshes)"
mkdir -p "$AP_GZ/models"
rm -rf "$AP_GZ/models/gimbal_small_2d"
cp -r "$HERE/gimbal_small_2d" "$AP_GZ/models/"

echo ">>> CMakeLists: removing the Gazebo<8 condition (if present)"
python3 - "$AP_GZ/CMakeLists.txt" <<'PYEOF'
import sys
path_value = sys.argv[1]
s = open(path_value).read()
gate_value = '''if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
    target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
    install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()'''
unconditional = '''add_library(GimbalSmall2dPlugin SHARED src/GimbalSmall2dPlugin.cc)
target_link_libraries(GimbalSmall2dPlugin ${GAZEBO_LIBRARIES})
install(TARGETS GimbalSmall2dPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})'''
if gate_value in s:
    open(path_value, 'w').write(s.replace(gate_value, unconditional, 1))
    print("  Version condition removed")
elif 'add_library(GimbalSmall2dPlugin' in s and 'VERSION_LESS "8.0")\n    add_library(GimbalSmall2dPlugin' not in s:
    print("  Already unconditional; no change needed")
else:
    print("  WARNING: expected pattern not found in CMakeLists.")
    print("  Edit the GimbalSmall2dPlugin target manually to build with Gazebo 11")
    print("  (move it outside the VERSION_LESS 8.0 condition).")

# Add target gz_tilt_pub (Phase O persistent issuer) if not present
if 'gz_tilt_pub' not in s:
    s = open(path_value).read()
    s += ('\n# Added by gimbal_setup/install.sh: Phase C persistent tilt publisher\n'
          'add_executable(gz_tilt_pub src/gz_tilt_pub.cc)\n'
          'target_link_libraries(gz_tilt_pub ${GAZEBO_LIBRARIES})\n')
    open(path_value, 'w').write(s)
    print("Added target gz_tilt_pub")
else:
    print("Target gz_tilt_pub already exists")
PYEOF

echo ">>> Building"
mkdir -p "$AP_GZ/build"
cd "$AP_GZ/build"
cmake .. > /dev/null
make GimbalSmall2dPlugin gz_tilt_pub 2>&1 | tail -3
[[ -f libGimbalSmall2dPlugin.so ]] || { echo "ERROR: plugin compilation failed" >&2; exit 1; }
[[ -f gz_tilt_pub ]] || { echo "ERROR: Compilation of gz_tilt_pub failed" >&2; exit 1; }

echo
echo "COMPLETE: $AP_GZ/build/libGimbalSmall2dPlugin.so ready."
echo "Note: yildizlar_guidance.sh already configures GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH"
echo "through $AP_GZ; no additional configuration is required."
echo "Validation: python3 tools/gimbal_headless_test.py"
