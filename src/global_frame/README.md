# global_frame

Boat state (GPS + heading) and global-frame detections for planning.

## Nodes

### boat_state_node

- **Subscribes**: `mavros/global_position/global` (NavSatFix), configurable heading topic (e.g. `mavros/global_position/compass_hdg` or `mavros/local_position/pose`).
- **Publishes**: `/boat_pose` (BoatPose: east, north, heading_rad in map frame), optionally `/boat_pose_stamped` and TF `map` → `base_link`.
- **Parameters**: `global_position_topic`, `heading_topic`, `heading_topic_type` ("compass_hdg" | "pose"), `use_first_fix_as_origin`, `origin_lat_deg`, `origin_lon_deg`, `publish_pose_stamped`, `publish_tf`, `map_frame`, `base_link_frame`.
- **Heading**: Published `heading_rad` is angle from East (0 = East, positive = CCW). Compass topic (degrees from North) is converted automatically.

### detection_to_global_node

- **Subscribes**: `/boat_pose` (always); either `/fused_buoys` (when `use_fused_detections=true`) or `/tracked_buoys_json` (when false).
- **Publishes**: `/global_detections` (GlobalDetectionArray) in map frame (east, north) for planning.
- **Parameters**: `use_fused_detections` (default `true`) — use fused LiDAR+CV buoys with class_id; otherwise LiDAR-only. `merge_by_proximity_m` (optional): merge detections within this distance (meters); 0 = disabled.

## Launch

```bash
source install/setup.bash
ros2 launch global_frame global_frame.launch.py
```

## Planning integration

Run the task loop with pose and detections from ROS:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Repos/School/autonomy-ws-25-26/mapping/install/setup.bash
cd ~/Repos/School/autonomy-ws-25-26/planning
python3 run_task_with_ros.py --task 1 --entities Input_Entities/task1_entities.json
```

Ensure `boat_state_node`, `detection_to_global_node`, and your LiDAR/CV pipelines are running so `/boat_pose` and `/global_detections` are published.
