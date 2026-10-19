#!/bin/bash
# Monitor Competition Topics Script
# Displays real-time output from all competition-related topics.
# Run this in a separate terminal to monitor the task sequence execution.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Sourcing ROS2 and workspace ==="
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

echo ""
echo "========================================="
echo " Monitoring Competition Topics"
echo "========================================="
echo ""
echo "Monitoring the following topics:"
echo "  - /cur_task (current task number)"
echo "  - /messages/gate_pass (gate pass messages)"
echo "  - /messages/object_detected (object detection reports)"
echo "  - /messages/object_delivered (delivery confirmations)"
echo "  - /messages/docking (docking reports)"
echo "  - /messages/patrol_boat (patrol boat STOPPING/RESUMING)"
echo "  - /sound_signal_interupt_freq (sound signals)"
echo "  - /mavros/state (MAVROS mode: AUTO/GUIDED)"
echo ""
echo "Press Ctrl+C to stop monitoring."
echo "========================================="
echo ""

# Function to echo a topic with a label
monitor_topic() {
    local topic=$1
    local label=$2
    ros2 topic echo "$topic" 2>/dev/null | while IFS= read -r line; do
        if [ -n "$line" ]; then
            echo "[$label] $line"
        fi
    done &
}

# Start monitoring all topics
monitor_topic "/cur_task" "CUR_TASK"
monitor_topic "/messages/gate_pass" "GATE_PASS"
monitor_topic "/messages/object_detected" "OBJ_DETECTED"
monitor_topic "/messages/object_delivered" "OBJ_DELIVERED"
monitor_topic "/messages/docking" "DOCKING"
monitor_topic "/messages/patrol_boat" "PATROL_BOAT"
monitor_topic "/sound_signal_interupt_freq" "SOUND_SIGNAL"

# Monitor MAVROS state but only show mode field
monitor_mavros_state() {
    ros2 topic echo /mavros/state 2>/dev/null | awk '
        /guided:/ { guided = $2 }
        /mode:/ {
            mode = $2
            if (mode != last_mode || guided != last_guided) {
                printf("[MAVROS_STATE] guided: %s\n", guided)
                printf("[MAVROS_STATE] mode: %s\n", mode)
                last_mode = mode
                last_guided = guided
            }
        }
    '
}

monitor_mavros_state &

echo "Monitoring started. Output will appear below as messages are published."
echo ""

# Wait for all background processes
wait
