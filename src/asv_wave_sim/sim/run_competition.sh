#!/bin/bash
# Full competition-style run in one tmux session ("roboboat"): generate the course, then start
# Gazebo, bridges, SITL, MAVROS, the judge and the autonomy stack, each in its own window.
#
#   run_competition.sh --seed 0 --task 1          2027 course, boat before Task 1, planner task 1
#   run_competition.sh --seed 0 --task 3          boat before that task's entrance (any task)
#   run_competition.sh --seed 0 --task 123        no start of its own: the course start (full run)
#   run_competition.sh --course 2026 --task 1     the 2026 course; --course readiness: the two gates
#   run_competition.sh --seed 7 --task 3 --waves breezy_lake
#   run_competition.sh --seed 2 --light low_sun --wind-from 270   weather and time of day (SIMULATION.md)
#   run_competition.sh --seed 3 --no-stack        everything but the autonomy stack (drive it yourself)
#   run_competition.sh ... --no-cv --rviz --no-attach
#   run_competition.sh ... --full-res           cameras as on the boat (1920x1200@30); --camera WxH@HZ
#   run_competition.sh ... --labels             also record labelled CV training frames (autolabel window)
#   run_competition.sh ... --headless           Gazebo server only (over SSH); GUI: . env.sh && sim_load_course && gz sim -g
#
# Windows: Ctrl+b then w lists them (gazebo, bridges, sitl, mavros, judge, stack, rviz, autolabel, foxglove, shell).
# Stop with stop_competition.sh; it ends the judge first so the run report is written.
set -e
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION=roboboat
SEED=0 TASK="" STACK=1 RVIZ=0 ATTACH=1 LABELS=0 HEADLESS=0
COURSE_ARGS=() STACK_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --world) COURSE_ARGS+=(--world "$2"); SEED=""; shift 2 ;;
    --legacy) COURSE_ARGS+=(--legacy); SEED=""; shift ;;
    --task) TASK="$2"; shift 2 ;;
    --waves) COURSE_ARGS+=(--waves "$2"); shift 2 ;;
    --camera) COURSE_ARGS+=(--camera "$2"); shift 2 ;;
    --full-res) COURSE_ARGS+=(--full-res); shift ;;
    --labels) COURSE_ARGS+=(--labels); LABELS=1; shift ;;
    --no-stack) STACK=0; shift ;;
    --no-cv|--no-arm) STACK_ARGS+=("$1"); shift ;;
    --rviz) RVIZ=1; shift ;;
    --headless) HEADLESS=1; shift ;;
    --no-attach) ATTACH=0; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) COURSE_ARGS+=("$1"); shift ;;
  esac
done

command -v tmux >/dev/null || { echo "tmux is required (sudo apt install tmux)" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "A previous run is still open; stopping it first."
  bash "$SIM_DIR/stop_competition.sh"
fi

# A single task starts just before its own entrance. A task without its own start (the planner's
# sequence mode, --task 123) uses the course's default start: before Task 1 on the 2026 course,
# the sketched start on the 2027 course.
[ -n "$SEED" ] && [ -n "$TASK" ] && COURSE_ARGS+=(--start-task "$TASK")
bash "$SIM_DIR/select_course.sh" ${SEED:+--seed "$SEED"} "${COURSE_ARGS[@]}"
( . "$SIM_DIR/env.sh" >/dev/null && sim_robocommand_address --prompt ) || true

# Each window's output also goes to runs/logs/<time>/<window>.log (the last 5 runs are kept).
LOG_ROOT="${SIM_RUNS_DIR:-$SIM_DIR/runs}/logs"
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
ls -1d "$LOG_ROOT"/*/ 2>/dev/null | head -n -5 | xargs -r rm -rf

win() {  # name "command"
  local cmd
  cmd="$(printf '%q ' bash "$SIM_DIR/_window.sh" "$1" "$2")"
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux new-window -t "$SESSION" -n "$1" "$cmd"
  else
    tmux new-session -d -s "$SESSION" -n "$1" -x 220 -y 50 "$cmd"
  fi
  [ "$1" = shell ] || tmux pipe-pane -t "$SESSION:$1" "cat >> $(printf '%q' "$LOG_DIR/$1.log")"
}
AFTER_GZ='. ./env.sh && sim_load_course && sim_wait_for_gazebo &&'

win gazebo  "SIM_HEADLESS=$HEADLESS bash 1_gazebo.sh"
win bridges "bash 2_bridges.sh"
win sitl    "$AFTER_GZ bash 3_sitl.sh"
win mavros  "bash 4_mavros.sh"
win judge   "$AFTER_GZ bash 6_judge.sh"
[ $STACK = 1 ] && win stack "$AFTER_GZ bash 7_stack.sh --task ${TASK:-1} ${STACK_ARGS[*]}"
[ $RVIZ = 1 ] && win rviz "bash 5_view_sensors.sh"
[ $LABELS = 1 ] && win autolabel "$AFTER_GZ bash 8_autolabel.sh"
win foxglove "$AFTER_GZ bash 9_foxglove.sh"
win shell '. ./env.sh && sim_load_course && echo "ROS shell for course $SIM_WORLD" && exec bash -i'

echo
echo "Started tmux session '$SESSION' (seed $SEED, task ${TASK:-1})."
echo "  attach:  tmux attach -t $SESSION      windows: Ctrl+b w      stop: $SIM_DIR/stop_competition.sh"
echo "  logs:    $LOG_DIR"
echo "  watch:   Foxglove -> ws://<this machine>:$( . "$SIM_DIR/env.sh" >/dev/null 2>&1; echo "$FOXGLOVE_PORT")  (SIMULATION.md: Watching a run)"
tmux select-window -t "$SESSION:judge"
if [ $ATTACH = 1 ] && [ -t 1 ]; then exec tmux attach -t "$SESSION"; fi
