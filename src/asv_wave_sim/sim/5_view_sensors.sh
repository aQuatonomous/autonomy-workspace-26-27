#!/bin/bash
# Terminal 5: RViz with the Airy point cloud, LiDAR buoy markers and the three cameras.
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec rviz2 -d "$SIM_DIR/sensors.rviz"
