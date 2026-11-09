#!/bin/bash
# Terminal 1: Gazebo with the course, water and ourboat. Start this first.
#   1_gazebo.sh                 # course chosen by select_course.sh (seed 0 if none yet)
#   1_gazebo.sh --seed 7        # generate + select seed 7, then start
#   1_gazebo.sh --legacy        # the original aquatonomous_world.sdf
#   1_gazebo.sh --headless      # server only, sensors rendered on the GPU without a window (over
#                               # SSH); open the GUI elsewhere with: . env.sh && sim_load_course && gz sim -g
# Other arguments are passed to select_course.sh. SIM_HEADLESS=1 is the same as --headless.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

ARGS=()
for a in "$@"; do
  if [ "$a" = --headless ]; then SIM_HEADLESS=1; else ARGS+=("$a"); fi
done

if [ ${#ARGS[@]} -gt 0 ] || ! sim_load_course; then
  bash "$SIM_DIR/select_course.sh" "${ARGS[@]}" || exit 1
fi
sim_load_course

# Freshwater venue (regatta lake): gz-waves buoyancy reads this.
export GZ_WAVES_WATER_DENSITY="$SIM_WATER_DENSITY"
cd "$(dirname "$SIM_WORLD")"
if [ "${SIM_HEADLESS:-0}" = 1 ]; then
  # A forwarded X display would route the sensor rendering through it; EGL uses the local GPU.
  unset DISPLAY
  echo "Gazebo server only (headless). GUI: . $SIM_DIR/env.sh && sim_load_course && gz sim -g"
  exec gz sim -s -r -v 3 --headless-rendering "$SIM_WORLD"
fi
sim_check_gpu
exec gz sim -v 3 -r "$SIM_WORLD"
