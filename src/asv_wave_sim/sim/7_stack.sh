#!/bin/bash
# Terminal 7: the team's autonomy stack, unmodified, against the simulator. Wired like
# run_comp_three_cameras.sh but fed by Gazebo: CV on /cameraN/image_raw, the LiDAR pipeline on
# /rslidar_points, global_frame, CV-LiDAR fusion, the global planner, and message_node, whose
# reports reach the sim judge at the competition address (10.10.10.1:50000).
#
#   7_stack.sh --task 1          planner task_id (1, 2, 3, ...)
#   7_stack.sh --task 2 --no-cv  skip the TensorRT CV pipeline (LiDAR-only detections)
#   7_stack.sh --no-arm          leave mode/arming to you (MAVProxy: mode GUIDED, arm throttle)
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

TASK=1 CV=1 ARM=1
while [ $# -gt 0 ]; do
  case "$1" in
    --task) TASK="$2"; shift 2 ;;
    --no-cv) CV=0; shift ;;
    --no-arm) ARM=0; shift ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
# CV buoy reference sizes: Task 2 uses small (A-0) buoys; the others use the larger ones.
CV_TASK=3; [ "$TASK" = 2 ] && CV_TASK=2
NUMBERS=false; [ "$TASK" = 5 ] && NUMBERS=true

set -m
PIDS=()
cleanup() {
  echo; echo "=== Stopping the autonomy stack"
  for p in "${PIDS[@]}"; do kill -INT -"$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -KILL -"$p" 2>/dev/null || true; done
  exit 130
}
trap cleanup INT TERM HUP
start() { echo "=== $1"; shift; "$@" & PIDS+=($!); sleep 1; }

start "global_frame (boat_state_node + detection_to_global_node)" \
  ros2 launch global_frame global_frame.launch.py use_fused_detections:=$([ $CV = 1 ] && echo true || echo false)
start "LiDAR buoy pipeline (team tuning) on the simulated Airy (/rslidar_points)" \
  ros2 launch "$SIM_DIR/lidar_pipeline_sim.launch.py"

if [ $CV = 1 ]; then
  ENGINES="$(bash "$SIM_DIR/build_engines.sh" --print-dir 2>/dev/null)"
  if bash "$SIM_DIR/build_engines.sh" --check 2>/dev/null; then
    start "CV pipeline (3 cameras, 960x600, task:=$CV_TASK, engines $ENGINES)" \
      ros2 launch cv_ros_nodes launch_cv_sim.py resolution:=960,600 task:="$CV_TASK" conf_threshold:=0.25 \
        engine_path:="$ENGINES/aqua_main.engine" number_detection_engine:="$ENGINES/number_detection.engine" \
        enable_indicator_buoy:=true enable_number_detection:=$NUMBERS
    start "CV-LiDAR fusion" ros2 run cv_lidar_fusion vision_lidar_fusion
  else
    echo "!!! No TensorRT engines for this GPU; running without CV. Build them with: $SIM_DIR/build_engines.sh" >&2
  fi
fi

# global_planner.launch.py passes map_bounds=[] as a parameter, which launch_ros rejects on Jazzy;
# running the node directly with the PLANNING_PATH that launch file sets is otherwise identical.
start "global planner (task_id:=$TASK)" \
  env PLANNING_PATH="$WS/planning" ros2 run global_planner global_planner_node --ros-args -p task_id:="$TASK"

sim_robocommand_address
if [ -d "$MESSAGE_NODE_PYDEPS/google/protobuf" ]; then
  start "message_node -> RoboCommand $ROBOCOMMAND_HOST:$ROBOCOMMAND_PORT (sim judge)" \
    env PYTHONPATH="$MESSAGE_NODE_PYDEPS${PYTHONPATH:+:$PYTHONPATH}" ros2 run message_node message_node
else
  echo "!!! $MESSAGE_NODE_PYDEPS is missing (protobuf for message_node); run setup_sim.sh. No reports this run." >&2
fi
[ $ARM = 1 ] && start "GUIDED + arm over MAVROS" bash "$SIM_DIR/arm_guided.sh"

echo "=== Stack running (Ctrl+C stops it)."
wait
