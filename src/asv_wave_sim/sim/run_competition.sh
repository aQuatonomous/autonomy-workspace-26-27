#!/bin/bash
# Full competition-style run in one tmux session ("roboboat"): generate the course, then start
# Gazebo, bridges, SITL, MAVROS, the judge and the autonomy stack, each in its own window.
#
#   run_competition.sh --seed 0 --task 1          handbook layout, planner task 1
#   run_competition.sh --seed 7 --task 3 --waves breezy_lake
#   run_competition.sh --seed 3 --no-stack        everything but the autonomy stack (drive it yourself)
#   run_competition.sh ... --no-cv --rviz --no-attach
#
# Windows: Ctrl+b then w lists them (gazebo, bridges, sitl, mavros, judge, stack, rviz, shell).
# Stop with stop_competition.sh; it ends the judge first so the run report is written.
set -e
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION=roboboat
SEED=0 TASK=1 STACK=1 RVIZ=0 ATTACH=1
COURSE_ARGS=() STACK_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --world) COURSE_ARGS+=(--world "$2"); SEED=""; shift 2 ;;
    --legacy) COURSE_ARGS+=(--legacy); SEED=""; shift ;;
    --task) TASK="$2"; shift 2 ;;
    --waves) COURSE_ARGS+=(--waves "$2"); shift 2 ;;
    --no-stack) STACK=0; shift ;;
    --no-cv|--no-arm) STACK_ARGS+=("$1"); shift ;;
    --rviz) RVIZ=1; shift ;;
    --no-attach) ATTACH=0; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) COURSE_ARGS+=("$1"); shift ;;
  esac
done

command -v tmux >/dev/null || { echo "tmux is required (sudo apt install tmux)" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "A previous run is still open; stopping it first."
  bash "$SIM_DIR/stop_competition.sh"
fi

bash "$SIM_DIR/select_course.sh" ${SEED:+--seed "$SEED"} "${COURSE_ARGS[@]}"
( . "$SIM_DIR/env.sh" >/dev/null && sim_robocommand_address ) || true

win() {  # name "command"
  local cmd
  cmd="$(printf '%q ' bash "$SIM_DIR/_window.sh" "$1" "$2")"
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux new-window -t "$SESSION" -n "$1" "$cmd"
  else
    tmux new-session -d -s "$SESSION" -n "$1" -x 220 -y 50 "$cmd"
  fi
}
AFTER_GZ='. ./env.sh && sim_load_course && sim_wait_for_gazebo &&'

win gazebo  "bash 1_gazebo.sh"
win bridges "bash 2_bridges.sh"
win sitl    "$AFTER_GZ bash 3_sitl.sh"
win mavros  "bash 4_mavros.sh"
win judge   "$AFTER_GZ bash 6_judge.sh"
[ $STACK = 1 ] && win stack "$AFTER_GZ bash 7_stack.sh --task $TASK ${STACK_ARGS[*]}"
[ $RVIZ = 1 ] && win rviz "bash 5_view_sensors.sh"
win shell '. ./env.sh && sim_load_course && echo "ROS shell for course $SIM_WORLD" && exec bash -i'

echo
echo "Started tmux session '$SESSION' (seed $SEED, task $TASK)."
echo "  attach:  tmux attach -t $SESSION      windows: Ctrl+b w      stop: $SIM_DIR/stop_competition.sh"
tmux select-window -t "$SESSION:judge"
if [ $ATTACH = 1 ] && [ -t 1 ]; then exec tmux attach -t "$SESSION"; fi
