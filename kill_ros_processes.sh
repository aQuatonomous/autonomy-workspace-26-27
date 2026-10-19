#!/bin/bash
# Comprehensive ROS process killer. Run from the repository root.
# Usage: ./kill_ros_processes.sh  (all output to this terminal)

echo "=== Killing all ROS/autonomy processes ==="

# Kill by process name patterns (comprehensive list)
# Include bag replay (ros2 bag play) and visualizer so previous replay sessions are fully stopped
PATTERNS=(
    "ros2"
    "ros2 bag"
    "bag play"
    "ros2bag"
    "rosbag2"
    "map_visualizer"
    "web_server_map"
    "mavros"
    "apm.launch"
    "mavros_node"
    "unitree_lidar"
    "lidar_range_filter"
    "pointcloud_filters"
    "buoy_detector"
    "buoy_tracker"
    "buoy_visualizer"
    "tracked_buoy_visualizer"
    "camera.*_node"
    "v4l2_camera"
    "vision_preprocessing"
    "vision_inference"
    "vision_combiner"
    "cv_ros_nodes"
    "vision_lidar_fusion"
    "detection_to_global"
    "boat_state_node"
    "global_frame"
    "global_planner_node"
    "cheese_mission"
    "rqt"
    "rviz2"
    "rviz"
)

for pattern in "${PATTERNS[@]}"; do
    echo "Killing processes matching: $pattern"
    pkill -f "$pattern" 2>/dev/null || true
    sleep 0.5
    pkill -9 -f "$pattern" 2>/dev/null || true
done

echo "=== Waiting for processes to terminate ==="
sleep 2

echo "=== Force killing any remaining ROS processes ==="
pkill -9 -f "ros2|ros2bag|rosbag2|bag play|mavros|apm.launch|mavros_node|cheese_mission|unitree|lidar|buoy|camera|vision|cv_|detection|rqt|map_visualizer|web_server_map|rviz|pointcloud_filters" 2>/dev/null || true
sleep 1
# MAVROS: kill by executable name and by PID (pkill -f can miss if cmdline is truncated)
killall -9 mavros_node 2>/dev/null || true
killall -9 mavros_router 2>/dev/null || true
killall -9 mavros 2>/dev/null || true
pkill -9 -f "mavros" 2>/dev/null || true
pkill -9 -f "apm.launch" 2>/dev/null || true
# Explicit PID kill: pgrep outputs one PID per line
for pid in $(pgrep -f "mavros" 2>/dev/null); do
    kill -9 "$pid" 2>/dev/null || true
done
for pid in $(pgrep -f "apm.launch" 2>/dev/null); do
    kill -9 "$pid" 2>/dev/null || true
done
sleep 1
pkill -9 -f "mavros" 2>/dev/null || true

echo "=== Stopping ROS2 daemon ==="
source /opt/ros/jazzy/setup.bash 2>/dev/null
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true
sleep 1
ros2 daemon stop 2>/dev/null || true

echo "=== Cleanup complete ==="
echo "Remaining topics:"
sleep 2
ros2 topic list 2>/dev/null || echo "No ROS2 daemon running"
if ros2 topic list 2>/dev/null | grep -q mavros; then
    echo ""
    echo "  MAVROS topics still listed (discovery may be stale). Processes still running?"
    pgrep -af "mavros|apm.launch" 2>/dev/null || true
    echo "  If you see PIDs above, run:  kill -9 <PID>  for each, or:  sudo pkill -9 -f mavros"
fi