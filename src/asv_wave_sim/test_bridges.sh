#!/bin/bash
# Test ROS 2 bridges in separate terminals
# Run AFTER Gazebo is running
# Terminal 1: ./test_gazebo_gui.sh
# Terminal 2: ./test_bridges.sh

set -e

echo "========================================="
echo " Test 2: ROS 2 Bridges"
echo "========================================="
echo ""
echo "Prerequisites:"
echo "  - Gazebo must be running (run test_gazebo_gui.sh first)"
echo "  - Wait 10 seconds after Gazebo starts"
echo ""
echo "This will start 4 bridges in background and monitor topics"
echo ""

# Source ROS and bridges
source /opt/ros/jazzy/setup.bash
[ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash
source ~/bridge_ws/install/setup.bash

# Fix ROS 2 middleware if needed
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Start bridges in background
echo "Starting lidar bridge..."
ros2 run ros_gz_bridge parameter_bridge /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r /lidar_topic/points:=/laser_points &
LIDAR_PID=$!

sleep 2
echo "Starting camera1 bridge..."
ros2 run ros_gz_bridge parameter_bridge /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera1_topic:=/camera0/image_raw &
CAM1_PID=$!

sleep 1
echo "Starting camera2 bridge..."
ros2 run ros_gz_bridge parameter_bridge /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera2_topic:=/camera1/image_raw &
CAM2_PID=$!

sleep 1
echo "Starting camera3 bridge..."
ros2 run ros_gz_bridge parameter_bridge /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera3_topic:=/camera2/image_raw &
CAM3_PID=$!

sleep 3
echo ""
echo "Bridges started. Checking topics..."
echo ""

ros2 topic list | grep -E "laser_points|camera[0-2]/image_raw"

echo ""
echo "Testing lidar data (showing first message):"
ros2 topic echo /laser_points --once

echo ""
echo "Success! Press Ctrl+C to stop all bridges."
echo "Bridge PIDs: $LIDAR_PID $CAM1_PID $CAM2_PID $CAM3_PID"

trap "kill $LIDAR_PID $CAM1_PID $CAM2_PID $CAM3_PID 2>/dev/null" EXIT

wait
