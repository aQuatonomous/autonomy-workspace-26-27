#!/bin/bash
# Sound pipeline: audio capture, sound signal detection, message_node (heartbeat + confirmation of signal).
# All three nodes are started together via one launch. Run from the repository root. Ctrl+C stops all.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

echo "=== Starting sound pipeline (audio_capturer, sound_signal, message_node) ==="
echo "Press Ctrl+C to stop all."
exec ros2 launch sound_pipeline_launch sound_pipeline.launch.py
