#!/bin/bash
# Task Sequence Competition Script
# Runs MAVROS, sound pipeline, and task sequence coordinator.
# Executes pre-programmed task sequence (Task 1 -> 3 -> 4) with sound interrupt handling.
# DO NOT RUN planning nodes, CV, or LiDAR.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

# If /dev/video0 is already in use at startup, kill the process holding it
if [ -e /dev/video0 ]; then
    fuser -k /dev/video0 2>/dev/null || true
fi

echo "=== Sourcing ROS2 and workspace ==="
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -m
MAVROS_PID=""
SOUND_PID=""
COORDINATOR_PID=""
CV_PID=""
PATROL_PID=""
cleanup() {
    echo ""
    echo "=== Stopping MAVROS, sound pipeline, and task coordinator ==="
    [ -n "$COORDINATOR_PID" ] && kill -TERM -"$COORDINATOR_PID" 2>/dev/null || true
    [ -n "$SOUND_PID" ] && kill -TERM -"$SOUND_PID" 2>/dev/null || true
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$CV_PID" ] && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$PATROL_PID" ] && kill -TERM -"$PATROL_PID" 2>/dev/null || true

    sleep 1
    
    # Force kill any remaining processes
    _KILL="ros2 launch|mavros|audio_capturer|sound_signal|message_node|task_sequence_coordinator|patrol_boat_detector"
    pkill -f "$_KILL" 2>/dev/null || true
    sleep 1
    pkill -9 -f "$_KILL" 2>/dev/null || true
    
    echo "=== Restarting ROS2 daemon ==="
    ros2 daemon stop 2>/dev/null || true
    sleep 0.5
    ros2 daemon start 2>/dev/null || true
    echo "Done."
    exit 130
}
trap cleanup INT TERM

# "${SCRIPT_DIR}/set_camera_fps.sh" single || { echo "Warning: set_camera_fps failed"; exit 1; }

CAMERA1_DEVICE_FILE="${SCRIPT_DIR}/.camera_devices"
if [ ! -f "$CAMERA1_DEVICE_FILE" ]; then
    echo "ERROR: Camera device list not found at ${CAMERA1_DEVICE_FILE}."
    echo "Run ${SCRIPT_DIR}/monitor_camera_move.sh (if available) and ${SCRIPT_DIR}/set_camera_fps.sh single to regenerate it."
    exit 1
fi

CAMERA1_DEVICE="$(cat "$CAMERA1_DEVICE_FILE")"
if [ -z "$CAMERA1_DEVICE" ]; then
    echo "ERROR: .camera_devices is empty. Cannot determine camera1 device."
    echo "Run ${SCRIPT_DIR}/monitor_camera_move.sh (if available) and ${SCRIPT_DIR}/set_camera_fps.sh single to regenerate it."
    exit 1
fi

if [ ! -e "$CAMERA1_DEVICE" ]; then
    echo "ERROR: Camera1 device from .camera_devices does not exist: ${CAMERA1_DEVICE}"
    echo "The physical camera may have moved USB ports or the by-path changed."
    echo "Run ${SCRIPT_DIR}/monitor_camera_move.sh (if available) and ${SCRIPT_DIR}/set_camera_fps.sh single to refresh .camera_devices."
    exit 1
fi

sleep 1
echo "=== Launching CV"
ros2 launch cv_ros_nodes launch_cv_single_camera1.py \
  resolution:=960,600 \
  conf_threshold:=0.3 \
  preprocess_fps:=8 \
  inference_interval_front:=2 \
  camera1_device:="${CAMERA1_DEVICE}" \
  &
CV_PID=$!

# Launch patrol boat detector node (listens to /camera1/image_preprocessed)
echo "=== Launching patrol boat detector ==="
ros2 run task_sequence_coordinator patrol_boat_detector --camera_id 1 &
PATROL_PID=$!
sleep 1


# Launch MAVROS
if [[ "$FCU_URL" != /dev/* ]] || [ -e "${FCU_URL%%:*}" ]; then
    echo "=== Launching MAVROS (Pixhawk at ${FCU_URL}) ==="
    ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
    MAVROS_PID=$!
    echo "MAVROS PID: $MAVROS_PID"
    sleep 2
else
    echo "=== Skipping MAVROS (no device at ${FCU_URL%%:*}) ==="
fi


# Launch sound pipeline (audio_capturer, sound_signal, message_node)
echo "=== Launching sound pipeline (audio_capturer, sound_signal, message_node) ==="
ros2 launch sound_pipeline_launch sound_pipeline.launch.py &
SOUND_PID=$!
echo "Sound pipeline PID: $SOUND_PID"
sleep 2

# Launch task sequence coordinator
echo "=== Launching task sequence coordinator ==="
ros2 run task_sequence_coordinator task_sequence_coordinator &
COORDINATOR_PID=$!
echo "Coordinator PID: $COORDINATOR_PID"

echo ""
echo "========================================="
echo " Task Sequence Competition Script Running"
echo "========================================="
echo "MAVROS PID: $MAVROS_PID"
echo "Sound Pipeline PID: $SOUND_PID"
echo "Coordinator PID: $COORDINATOR_PID"
echo ""
echo "Task Sequence:"
echo "  1. Task 1 (Entry/Exit): cur_task=1, 'start' @2s, 'end' @6s"
echo "  2. Task 3 (Speed): cur_task=3 @14s, 'speed_start', 'speed_end' @34s"
echo "  3. Task 4 (Delivery): cur_task=4 @44s, object_delivered"
echo ""
echo "Sound Interrupt:"
echo "  - 1 blast: Navigate to YELLOW_BUOY (GUIDED mode)"
echo "  - 2 blasts: Navigate to MARINA (GUIDED mode)"
echo "  - Returns to AUTO mode and resumes task after arrival"
echo ""
echo "Press Ctrl+C to stop all."
echo "========================================="
echo ""

wait
cleanup
