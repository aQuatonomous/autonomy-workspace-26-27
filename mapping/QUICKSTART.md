# LiDAR pipeline – run now

**Build (from the mapping folder):**

```bash
cd ~/Repos/School/autonomy-ws-25-26/mapping
source /opt/ros/jazzy/setup.bash
colcon build --packages-select unitree_lidar_ros2 pointcloud_filters --symlink-install
source install/setup.bash
```

**Run just the LiDAR driver** (to test connection; uses `/dev/ttyUSB0`):

```bash
source install/setup.bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args -p port:=/dev/ttyUSB0
```

If the port is missing you’ll get an error. Check `ls /dev/ttyUSB*` and add yourself to the `dialout` group if needed: `sudo usermod -aG dialout $USER` (then log out and back in).

**Run the full pipeline** (driver → range filter → detector → tracker → visualizer):

```bash
cd ~/Repos/School/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py
```

LiDAR is fixed to `/dev/ttyUSB0`. With RViz: `ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=true`.

**RViz:** Set **Fixed Frame** to `unilidar_lidar`. Topics: **PointCloud2** `/unilidar/cloud` (raw), `/points_filtered` (filtered); **MarkerArray** `/buoy_markers` (detections), `/tracked_buoy_markers` (tracked/fused, only when CV fusion runs). Use **Reliable** QoS for filtered and markers if they don’t show.

- **Overrides, inside-bay, tuning, troubleshooting:** [COMPETITION.md](COMPETITION.md).
- **WSL2 + USB LiDAR:** [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md).
- **Full node and parameter details:** [README.md](README.md).
