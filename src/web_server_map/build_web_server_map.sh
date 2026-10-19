#!/bin/bash
# Build script for web_server_map package

set -e  # Exit on error

echo "========================================="
echo "Building web_server_map package"
echo "========================================="

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "Error: package.xml not found. Are you in the web_server_map directory?"
    exit 1
fi

# Navigate to workspace root
cd ..

echo ""
echo "Step 1: Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo ""
echo "Step 2: Checking dependencies..."
if ! python3 -c "import aiohttp" 2>/dev/null; then
    echo "Warning: aiohttp not found. Installing..."
    pip3 install aiohttp
else
    echo "✓ aiohttp is installed"
fi

echo ""
echo "Step 3: Building package..."
colcon build --packages-select web_server_map

echo ""
echo "Step 4: Sourcing workspace..."
source install/setup.bash

echo ""
echo "========================================="
echo "Build complete!"
echo "========================================="
echo ""
echo "To run the node:"
echo "  ros2 run web_server_map map_visualizer_node"
echo ""
echo "Or with launch file:"
echo "  ros2 launch web_server_map map_visualizer.launch.py"
echo ""
echo "Then open a browser to:"
echo "  http://localhost:8080"
echo ""
echo "For remote access via SSH:"
echo "  ssh -L 8080:localhost:8080 user@jetson-ip"
echo ""
