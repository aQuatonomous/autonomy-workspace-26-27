# CV–LiDAR Fusion

Single fusion node that fuses **final CV output** (with distance) and **LiDAR tracked buoys** by class_id and class_name (e.g. red_buoy, red_pole_buoy). Distance from camera is **not** used for matching; it is present in the CV topic for future use.

**Subscribes to:** `/combined/detection_info_with_distance` (CV final output), `/tracked_buoys` (LiDAR)  
**Publishes to:** `/fused_buoys` (FusedBuoyArray: same as TrackedBuoyArray plus `class_id`, `class_name`)

Requires `pointcloud_filters` from the mapping workspace (build mapping first, then source its `install/setup.bash` before building/running this package).

---

## How to run (camera → LiDAR → fusion)

1. **Camera pipeline** (final CV output = `/combined/detection_info_with_distance`):
   ```bash
   cd ~/Repos/School/autonomy-ws-25-26/computer_vision
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch cv_ros_nodes launch_cv.py
   ```

2. **LiDAR pipeline** (`/tracked_buoys`, `/tracked_buoys_json`):
   ```bash
   cd ~/Repos/School/autonomy-ws-25-26/mapping
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch pointcloud_filters buoy_pipeline.launch.py
   ```

3. **Fusion node** (single node; subscribes to both, publishes `/fused_buoys`):
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/Repos/School/autonomy-ws-25-26/mapping/install/setup.bash
   source ~/Repos/School/autonomy-ws-25-26/computer_vision/install/setup.bash
   ros2 run cv_lidar_fusion vision_lidar_fusion
   ```

4. **See fused output:** `ros2 topic echo /fused_buoys` — each buoy has `class_id`, `class_name` (e.g. red_buoy, red_pole_buoy) when matched to CV; otherwise `class_name: "unknown"`, `class_id: 255`. The tracked-buoy visualizer (in the LiDAR launch) subscribes to `/fused_buoys` and shows class_name in RViz.

---

## Build

```bash
# Build mapping first (defines FusedBuoy, FusedBuoyArray)
cd ~/Repos/School/autonomy-ws-25-26/mapping && colcon build --packages-select pointcloud_filters
# Then build fusion (source mapping install)
cd ~/Repos/School/autonomy-ws-25-26/computer_vision
source ../mapping/install/setup.bash
colcon build --packages-select cv_lidar_fusion
```

See `computer_vision/FUSION_NODE_GUIDE.md` for message formats and association strategies.
