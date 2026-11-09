#!/bin/bash
# Terminal 9: watch a run from another computer in Foxglove (or Lichtblick).
# Starts foxglove_bridge on FOXGLOVE_PORT (config.env; one port per user on a shared machine)
# and the sim viewer: course ground truth on /sim/course, the true boat pose on /sim/boat_truth,
# JPEG camera feeds on /sim/view/<camera>/compressed, and the map -> sim_world transform.
# Connect from the app with "Open connection" -> Foxglove WebSocket -> ws://<this machine>:<port>.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
sim_load_course || { echo "No course selected; run select_course.sh first." >&2; exit 1; }

ros2 run sim_judge viewer --answer-key "$SIM_ANSWER_KEY" &
VIEWER=$!
trap 'kill -INT $VIEWER 2>/dev/null' EXIT

echo "Foxglove: ws://$(hostname -I 2>/dev/null | awk '{print $1}'):$FOXGLOVE_PORT  (or ws://localhost:$FOXGLOVE_PORT through ssh -L $FOXGLOVE_PORT:localhost:$FOXGLOVE_PORT)"
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:="$FOXGLOVE_PORT"
