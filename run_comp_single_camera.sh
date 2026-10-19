#!/bin/bash
# Competition launch (single camera): all tasks enabled (Task 2/3/4/5).
# Same structure as comp_single_camera_task3.sh but with task4, indicator buoy, number detection enabled.
# Run from the repository root. Ctrl+C kills all.
#
# Env vars: NOLOG=1 disable logging | SOUND=1 also launch sound pipeline | TASK_ID=N override task

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ID="${TASK_ID:-3}"
source "${SCRIPT_DIR}/loggers comp ran/logging_lib.sh"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

echo "=== Single camera (all CV tasks): Setting format YUYV @ 960x600 @ 15fps ==="
"${SCRIPT_DIR}/set_camera_fps.sh" single || { echo "Warning: set_camera_fps failed (edit set_camera_fps.sh or run ./monitor_camera_move.sh)"; exit 1; }
CAMERA1_DEVICE="$(cat "${SCRIPT_DIR}/.camera_devices")"

echo "=== Sourcing ROS2 and workspace (root) ==="
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

# Single-camera only: fail if 3-camera pipeline is already running
if ros2 topic list 2>/dev/null | grep -qE '/camera0/|/camera2/'; then
  echo "ERROR: camera0/camera2 topics present (3-camera pipeline running). Run: ./kill_all_ros.sh  then start this script again."
  exit 1
fi

set -m
MAVROS_PID=""
GLOBAL_FRAME_PID=""
LIDAR_PID=""
CV_PID=""
FUSION_PID=""
PLANNER_PID=""
cleanup() {
    echo ""
    echo "=== Stopping MAVROS, global_frame, LiDAR, CV, fusion, planning, sound ==="
    stop_sound_pipeline
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$GLOBAL_FRAME_PID" ] && kill -TERM -"$GLOBAL_FRAME_PID" 2>/dev/null || true
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ]   && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$FUSION_PID" ] && kill -TERM -"$FUSION_PID" 2>/dev/null || true
    [ -n "$PLANNER_PID" ] && kill -TERM -"$PLANNER_PID" 2>/dev/null || true
    sleep 1
    _KILL="ros2 launch|mavros|global_frame|boat_state_node|detection_to_global|v4l2_camera|v4l2_camera_node|camera0_node|camera1_node|camera2_node|cv_ros_nodes|vision_preprocessing|vision_inference|vision_combiner|maritime_distance|vision_lidar_fusion|task4_supply_processor|indicator_buoy_processor|maritime_distance_estimator|pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker|global_planner_node|audio_capturer|sound_signal|message_node"
    pkill -f "$_KILL" 2>/dev/null || true
    sleep 1
    pkill -9 -f "$_KILL" 2>/dev/null || true
    parse_and_summarize
    echo "Done."
    exit 130
}
trap cleanup INT TERM

if [[ "$FCU_URL" != /dev/* ]] || [ -e "${FCU_URL%%:*}" ]; then
  echo "=== Launching MAVROS (Pixhawk at ${FCU_URL}) ==="
  ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
  MAVROS_PID=$!
  sleep 1
else
  echo "=== Skipping MAVROS (no device at ${FCU_URL%%:*}) ==="
fi

echo "=== Launching global_frame (use_fused_detections:=true) ==="
ros2 launch global_frame global_frame.launch.py use_fused_detections:=true &
GLOBAL_FRAME_PID=$!
sleep 1

echo "=== Launching LiDAR buoy pipeline (no RViz) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 2
echo "=== Launching CV pipeline (camera1, all tasks: task4, indicator buoy, number detection) ==="
ros2 launch cv_ros_nodes launch_cv_single_camera1.py \
  resolution:=960,600 \
  conf_threshold:=0.1 \
  preprocess_fps:=5 \
  inference_interval_front:=4 \
  task:=3 \
  enable_task4:=true \
  enable_indicator_buoy:=true \
  enable_number_detection:=true \
  camera1_device:="${CAMERA1_DEVICE}" \
  &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

sleep 2
echo "=== Starting global planner (task_id:=${TASK_ID}) ==="
ros2 launch global_planner global_planner.launch.py task_id:="${TASK_ID}" cmd_vel_topic:=/uas1/mavros/setpoint_velocity/cmd_vel_unstamped &
PLANNER_PID=$!

start_sound_pipeline

echo "=== Pipelines started. MAVROS: $MAVROS_PID  GLOBAL_FRAME: $GLOBAL_FRAME_PID  LiDAR: $LIDAR_PID  CV: $CV_PID  FUSION: $FUSION_PID  PLANNER: $PLANNER_PID ==="
echo "Press Ctrl+C to stop all. All node logs appear below (interleaved)."
echo "(If LiDAR is not connected, that launch may exit; other nodes keep running.)"
echo ""
wait
cleanup
