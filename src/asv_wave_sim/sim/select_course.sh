#!/bin/bash
# Selects the course every other script uses and writes course/generated/current.env.
#
#   select_course.sh                     # seed 0: the handbook layout as drawn
#   select_course.sh --seed 7            # randomized layout within the handbook ranges
#   select_course.sh --seed 7 --waves breezy_lake
#   select_course.sh --world path/to/world.sdf   # any world; uses <world>.answer_key.json if present
#   select_course.sh --legacy            # the original aquatonomous_world.sdf (no judge)
#
# Extra options after the seed are passed to tools/generate_course.py (e.g. --shadows).
set -e
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

SEED=0
WORLD=""
GEN_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --world) WORLD="$(realpath "$2")"; shift 2 ;;
    --legacy) WORLD="$ASV_DIR/gz-waves-models/worlds/aquatonomous_world.sdf"; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) GEN_ARGS+=("$1"); shift ;;
  esac
done

mkdir -p "$SIM_STATE_DIR"
if [ -z "$WORLD" ]; then
  OUT="$SIM_STATE_DIR/seed_$SEED/roboboat2026_course"
  mkdir -p "$(dirname "$OUT")"
  python3 "$ASV_DIR/tools/generate_course.py" --config "$COURSE_DIR/roboboat2026.yaml" \
    --seed "$SEED" --out "$OUT" "${GEN_ARGS[@]}"
  WORLD="$OUT.sdf"
fi

KEY="${WORLD%.sdf}.answer_key.json"
{
  echo "SIM_WORLD='$WORLD'"
  echo "SIM_WORLD_NAME='$(sed -n 's/.*<world name="\([^"]*\)".*/\1/p' "$WORLD" | head -n 1)'"
  if [ -f "$KEY" ]; then
    python3 - "$KEY" <<'EOF'
import json, sys
k = json.load(open(sys.argv[1]))
print(f"SIM_ANSWER_KEY='{sys.argv[1]}'")
print(f"SIM_SITL_HOME='{k['sitl_home']}'")
print(f"SIM_WATER_DENSITY='{k.get('water_density', 1025)}'")
print(f"SIM_SEED='{k.get('seed', '')}'")
EOF
  else
    echo "SIM_ANSWER_KEY=''"
    echo "SIM_SITL_HOME='51.566151,-4.034345,10.0,-135'"
    echo "SIM_WATER_DENSITY='1025'"
    echo "SIM_SEED=''"
  fi
} > "$SIM_STATE_DIR/current.env"

sim_load_course
echo "Course: $SIM_WORLD"
echo "  answer key: ${SIM_ANSWER_KEY:-none (judge unavailable)}"
echo "  SITL home:  $SIM_SITL_HOME   water density: $SIM_WATER_DENSITY kg/m^3"
