#!/bin/bash
# Run MAVROS only (Pixhawk/ArduPilot). Run from the repository root. Ctrl+C to stop.

set -e
FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

echo "=== Sourcing ROS2 ==="
source /opt/ros/jazzy/setup.bash

echo "=== Launching MAVROS (Pixhawk at ${FCU_URL}) ==="
echo "Press Ctrl+C to stop."
exec ros2 launch mavros apm.launch fcu_url:="${FCU_URL}"
