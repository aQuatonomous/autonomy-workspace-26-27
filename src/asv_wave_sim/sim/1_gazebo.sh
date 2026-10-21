#!/bin/bash
# Terminal 1: Gazebo with the course, water and ourboat. Start this first.
#   1_gazebo.sh                 # course chosen by select_course.sh (seed 0 if none yet)
#   1_gazebo.sh --seed 7        # generate + select seed 7, then start
#   1_gazebo.sh --legacy        # the original aquatonomous_world.sdf
# Any arguments are passed to select_course.sh.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

if [ $# -gt 0 ] || ! sim_load_course; then
  bash "$SIM_DIR/select_course.sh" "$@" || exit 1
fi
sim_load_course
sim_check_gpu

# Freshwater venue (regatta lake): gz-waves buoyancy reads this.
export GZ_WAVES_WATER_DENSITY="$SIM_WATER_DENSITY"
cd "$(dirname "$SIM_WORLD")"
exec gz sim -v 3 -r "$SIM_WORLD"
