#!/bin/bash
# Test MAVROS connection to SITL
# Run in separate terminal AFTER SITL is running

set -e

echo "========================================="
echo " Test 4: MAVROS"
echo "========================================="
echo ""
echo "Prerequisites:"
echo "  - Gazebo running"
echo "  - SITL running (listening on port 5760)"
echo ""

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Waiting for SITL on port 5760..."
until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do 
  echo "  Still waiting..."; 
  sleep 2; 
done

echo "SITL detected! Launching MAVROS..."
ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760
