#!/bin/bash
# Terminal 6: the sim judge. Stands in for RoboCommand on TCP $ROBOCOMMAND_PORT (default 50000),
# scores the run from Gazebo ground truth, and writes a report to sim/runs/ on Ctrl+C.
# Extra arguments go to the judge (e.g. --duration 600, --report-tol 2).
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if ! sim_load_course || [ -z "$SIM_ANSWER_KEY" ]; then
  echo "No answer key for the selected course. Run select_course.sh (or 1_gazebo.sh --seed N) first." >&2
  exit 1
fi
exec ros2 run sim_judge judge --answer-key "$SIM_ANSWER_KEY" --port "$ROBOCOMMAND_PORT" "$@"
