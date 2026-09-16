#!/usr/bin/env bash
# Shared standoff geometry. Source this file; it is not a standalone launcher.
#
# With the stabilized tilt gimbal, DOWN is a mission-design parameter and the
# camera's world elevation follows YILDIZ_TILT=atan(DOWN/BACK). bbox_to_redis.py
# sends this positive-up tilt command and the plugin compensates body pitch.
# YILDIZ_PITCH_TRIM remains exported for yaw and archived body-fixed setups.
#
# For a body-fixed camera, DOWN instead follows back*tan(mount+trim). Flight
# evidence: back=25/down=13 with +30 degree mounting gave 98.4% detection.
# down=3 displaced the target from the axis by 22 degrees and detection fell
# to 5%. In either setup, camera elevation must match the standoff geometry.
#
# Consumers: yildizlar_guidance.sh passes back/down and tilt to bbox_to_redis.py;
# tools/scenario.sh passes back/down to simple_guided_follow.py. If tilt is
# omitted, bbox_to_redis.py derives atan(down/back) itself.
# Explicit YILDIZ_BACK, YILDIZ_DOWN, YILDIZ_TILT, YILDIZ_MOUNT and
# YILDIZ_PITCH_TRIM environment values override these defaults.

YILDIZ_MOUNT="${YILDIZ_MOUNT:-0}"
YILDIZ_PITCH_TRIM="${YILDIZ_PITCH_TRIM:--2.5}"
YILDIZ_BACK="${YILDIZ_BACK:-25}"

# There is no tan() in bash: first python3, otherwise awk (there is no tan in awk either -> sin/cos).
# Both failures are caught with || so sourcing remains safe under set -e.
derive_standoff_down() {
  local d=''
  d=$(python3 -c 'import math,sys; b,m,t=(float(x) for x in sys.argv[1:4]); print(int(round(b*math.tan(math.radians(m+t)))))' \
        "$1" "$2" "$3" 2>/dev/null) || d=''
  if [[ -z "$d" ]]; then
    d=$(awk -v b="$1" -v m="$2" -v t="$3" \
          'BEGIN{r=(m+t)*atan2(0,-1)/180; printf "%.0f", b*sin(r)/cos(r)}' 2>/dev/null) || d=''
  fi
  if [[ -z "$d" ]]; then
    # Neither python3 nor awk: drop to flight-verified binary rather than giving empty --down, and report the fallback (this value is false if BACK is different from 25).
    echo "WARNING: Could not derive standoff DOWN (no python3/awk) -> 13 is used" >&2
    d=13
  fi
  printf '%s' "$d"
}

# A mount near 0 degrees makes back*tan(mount+trim) negative because the
# default trim is -2.5 degrees. That would place the pursuer above the target.
# The body-fixed derivation does not apply to the stabilized gimbal: use the
# design depth below. In the legacy branch, nonpositive derived depth falls
# back to this value and emits a warning. tools/set_mounting.py --down reads
# and updates this one shared design value.
YILDIZ_DESIGN_DOWN=4      # standoff depth for the gimbal configuration [m]
# GIMBAL BRANCH: DOWN is no longer derived from the camera; single source of mission design value. The
# old mount-based derivation survives with YILDIZ_LEGACY_DERIVATION=1 for frozen body-fixed arms only.
if [[ "${YILDIZ_LEGACY_DERIVATION:-0}" == 1 ]]; then
  YILDIZ_DOWN="${YILDIZ_DOWN:-$(derive_standoff_down "$YILDIZ_BACK" "$YILDIZ_MOUNT" "$YILDIZ_PITCH_TRIM")}"
  if [[ "${YILDIZ_DOWN%%.*}" -le 0 ]] 2>/dev/null; then
    echo "WARNING: legacy derivation DOWN=$YILDIZ_DOWN (<=0); design value ${YILDIZ_DESIGN_DOWN} m." >&2
    YILDIZ_DOWN="$YILDIZ_DESIGN_DOWN"
  fi
else
  YILDIZ_DOWN="${YILDIZ_DOWN:-$YILDIZ_DESIGN_DOWN}"
fi

# Camera tilt (world elevation, positive up) from standoff geometry:
derive_yildiz_tilt() {
  python3 -c 'import math,sys; d,b=(float(x) for x in sys.argv[1:3]); print(f"{math.degrees(math.atan2(d,max(b,1e-6))):.2f}")' \
    "$1" "$2" 2>/dev/null || \
  awk -v d="$1" -v b="$2" 'BEGIN{printf "%.2f", atan2(d,b)*180/atan2(0,-1)}'
}
YILDIZ_TILT="${YILDIZ_TILT:-$(derive_yildiz_tilt "$YILDIZ_DOWN" "$YILDIZ_BACK")}"

export YILDIZ_MOUNT YILDIZ_PITCH_TRIM YILDIZ_BACK YILDIZ_DOWN YILDIZ_TILT
