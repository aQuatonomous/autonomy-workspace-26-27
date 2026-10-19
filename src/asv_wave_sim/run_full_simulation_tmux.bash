#!/bin/bash
# Run the full boat simulation in tmux: Gazebo + bridges + ArduPilot SITL + MAVROS + sim judge.
# This now delegates to sim/run_competition.sh, which derives every path from the repo location
# and config.env, homes SITL at the course origin, uses MAVROS on TCP 5762 and the Airy lidar
# topic (/rslidar_points). See sim/README.md.
#
#   bash run_full_simulation_tmux.bash              RoboBoat 2026 course (seed 0), no autonomy stack
#   bash run_full_simulation_tmux.bash --seed 7     randomized course
#   bash run_full_simulation_tmux.bash --legacy     the original aquatonomous_world.sdf
#   SITL_GUI=1 bash run_full_simulation_tmux.bash   adds MAVProxy console + map windows
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
exec bash "$SCRIPT_DIR/sim/run_competition.sh" --no-stack "$@"
