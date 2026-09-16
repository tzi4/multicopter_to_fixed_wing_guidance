#!/usr/bin/env bash
# THREE ROUTES A/B -- each algorithm change is run on THESE THREE ROUTES (user rule, 2026-08-06):
# straight (REALLY straight), ellipse, wanderer. WHY THREE AT ALL: single route leads to wrong
# judgment. Measured (ENG-5): gain on straight MISS timeout end (2->0) gain on ellipse CPA median 16.19
# -> 6.52 m Same fix, gained from DIFFERENT channel on two routes. Wanderer (zigzag) third regime:
# forces disturbance estimation and the yaw channel as the target constantly changes direction. USE:
# tools/three_route_ab.sh "LABEL" "ENV1=value ENV1=value ..." EXAMPLE (shutdown correction A/B):
# tools/three_route_ab.sh baseline "" tools/three_route_ab.sh corrected "YILDIZ_Q_AREA_MULTIPLIER=4
# YILDIZ_HORIZON_RANGE_REF=60" NOTE: this is a SCRIPT file; The pattern "*_guidance.py" is NOT passed on
# the command line, so scenario.sh clean()'s pkill does not hit the wrapper (FOLLOW_UP.md #4).
set -u
cd "$(dirname "$0")/.."

LABEL="${1:?usage: three_route_ab.sh LABEL \"ENV=value ...\"}"
ENV_OVERRIDES="${2:-}"
DURATION="${DURATION:-360}"
VISUAL="${VISUAL_GUIDANCE:-mpc_guidance.py}"

ROUTES="straight ellipse wanderer"

stop_stack() { ./yildizlar_guidance.sh --stop >/dev/null 2>&1 || true; sleep 8; }

echo "=========================================================="
echo "THREE ROUTES A/B label=$LABEL time=$DURATION s"
echo "env: ${ENV_OVERRIDES:-<absent, baseline>}"
echo "=========================================================="

for route in $ROUTES; do
  plan="missions/target_${route}.plan"
  if [[ ! -f "$plan" ]]; then
    echo "!!! no plan, skipping: $plan"; continue
  fi
  echo
  echo ">>> ROUTE=$route  ($(date +%H:%M))"
  # shellcheck disable=SC2086
  env $ENV_OVERRIDES METHOD="${LABEL}_${route}" DURATION="$DURATION" \
      VISUAL_GUIDANCE="$VISUAL" PLAN="$plan" tools/scenario.sh
  echo ">>> $route FINISHED ($(date +%H:%M)) -- cleaning"
  stop_stack
done

echo
echo "=========================================================="
echo "THREE ROUTE FINISHED: $LABEL  ($(date +%H:%M))"
echo "Comparison: python3 tools/compare_results.py"
echo "=========================================================="
