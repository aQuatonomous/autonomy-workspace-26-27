# Mapping and Boat Position Pipeline

This document describes the **entire** mapping and boat-position pipeline: how LiDAR, GPS, and compass data flow from hardware through ROS 2 nodes to produce **global buoy positions** in a local map frame. Every component and node that is actually used in the single-camera competition script is explained in depth.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Coordinate Frames](#2-coordinate-frames)
3. [Hardware and External Dependencies](#3-hardware-and-external-dependencies)
4. [Launch Order (run_comp_single_camera.sh)](#4-launch-order-run_comp_single_camerash)
5. [Global Frame Package (Boat State + Detection to Global)](#5-global-frame-package-boat-state--detection-to-global)
6. [LiDAR Buoy Pipeline (pointcloud_filters)](#6-lidar-buoy-pipeline-pointcloud_filters)
7. [End-to-End Data Flow](#7-end-to-end-data-flow)
8. [Message Types Reference](#8-message-types-reference)
9. [Verification and Debugging](#9-verification-and-debugging)

---

## 1. Overview

The pipeline does the following:

1. **Boat pose in the world**: GPS (lat/lon) and compass heading from the Pixhawk are converted into a boat pose in a local **map** frame: position `(east, north)` in meters and heading in radians.
2. **LiDAR in boat frame**: Raw LiDAR points are rotated and filtered so that the **+X axis always points forward** relative to the boat (aligned with the bow). Output is in `base_link` frame.
3. **Buoy detection**: Clusters of points are detected as buoys; each buoy gets a **range** and **bearing** in the boat frame.
4. **Tracking**: Buoys are tracked across frames with **persistent IDs**.
5. **Global detections**: Each boat-relative detection (range, bearing) is combined with the current boat pose (east, north, heading) to produce **global (east, north)** positions of buoys in the map frame.

**Final output**: `/global_detections` — list of buoys with `east`, `north` (meters from origin), `range_m`, `bearing_global_rad`, `source` (e.g. `lidar`), and stable `id`.

---

## 2. Coordinate Frames

| Frame | Description | Convention |
|-------|-------------|------------|
| **map** | Local world frame. Origin = first GPS fix (or configured origin). Fixed in the world. | **ENU**: +X = East, +Y = North, +Z = Up. Heading: 0 = East, positive = counter-clockwise (CCW). |
| **base_link** | Boat frame. Origin = boat center. Moves and rotates with the boat. | **FLU**: +X = Forward (bow), +Y = Left (port), +Z = Up. Bearing 0 = forward, + = left (CCW). |
| **unilidar_lidar** | LiDAR sensor frame. Raw point cloud from the Unitree driver. | Driver-defined; our static rotations align +X with boat forward. |

- **Boat pose** is published as `(east, north, heading_rad)` in the **map** frame.
- **TF**: `map` → `base_link` is published by `boat_state_node` (translation + rotation from boat pose).
- **LiDAR output** is labeled `base_link`; after the filter’s rotations, +X is aligned with the boat’s forward direction.

---

## 3. Hardware and External Dependencies

### Pixhawk (MAVROS)

- **Device**: `/dev/ttyACM0` (default, overridable via `FCU_URL`).
- **Role**: Provides GPS and compass (and optionally IMU) to ROS via MAVROS.

**Topics used by the mapping pipeline:**

| Topic | Type | Description |
|-------|------|-------------|
| `mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS: latitude, longitude, altitude. |
| `mavros/global_position/compass_hdg` | `std_msgs/Float64` | Compass heading in **degrees**: 0° = North, 90° = East. |

MAVROS is started by `comp_single_camera.sh` before the global_frame and LiDAR pipelines.

### Unitree LiDAR

- **Device**: `/dev/ttyUSB0`.
- **Role**: Provides 3D point clouds. The driver publishes in frame `unilidar_lidar`; no GPS/compass data is used inside the LiDAR pipeline itself.

---

## 4. Launch Order (run_comp_single_camera.sh)

From `run_comp_single_camera.sh`, the mapping-related launch order is:

1. **MAVROS** — `ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600`
2. **global_frame** — `ros2 launch global_frame global_frame.launch.py` (boat_state_node + detection_to_global_node)
3. **LiDAR buoy pipeline** — `ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false`

CV and fusion start after; they are not required for the core mapping/boat-position and LiDAR→global pipeline described here.

---

## 5. Global Frame Package (Boat State + Detection to Global)

Package: `global_frame`.  
Launch file: `global_frame/launch/global_frame.launch.py` — starts two nodes.

---

### 5.1 boat_state_node

**Executable**: `boat_state_node`  
**Role**: Convert GPS and compass into boat pose in the **map** frame and publish TF.

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS position. |
| `mavros/global_position/compass_hdg` | `std_msgs/Float64` | Compass heading (degrees, 0 = North). |

Heading can alternatively come from a `geometry_msgs/PoseStamped` topic (parameter `heading_topic_type`: `"pose"`).

#### Parameters (from launch)

- `global_position_topic`: `mavros/global_position/global`
- `heading_topic`: `mavros/global_position/compass_hdg`
- `heading_topic_type`: `compass_hdg`
- `use_first_fix_as_origin`: `True` — origin (lat0, lon0) is set from the first GPS fix.
- `publish_pose_stamped`: `True`
- `publish_tf`: `True`

#### Logic

1. **Origin**: If `use_first_fix_as_origin` is true, the first valid GPS fix sets `(origin_lat, origin_lon)`. Otherwise, use `origin_lat_deg` / `origin_lon_deg`.
2. **GPS → map**: Each NavSatFix is converted from (lat, lon) to (east, north) in meters using a local tangent-plane approximation (WGS84 ellipsoid, meridional radius at origin).
3. **Compass → heading**: Compass heading (0° = North, 90° = East) is converted to ENU heading: `heading_rad = (90° - compass_deg) * π/180`, so 0 rad = East, π/2 = North, positive = CCW.
4. **Publish**: On every GPS or compass update, the node publishes the current pose.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/boat_pose` | `global_frame/BoatPose` | `east`, `north` (m), `heading_rad`. Frame_id = `map`. |
| `/boat_pose_stamped` | `geometry_msgs/PoseStamped` | Same pose as position + quaternion; frame_id = `map`. |
| **TF** | `map` → `base_link` | Transform from map to boat: translation (east, north, 0), rotation from `heading_rad`. |

**BoatPose** (simplified): `header`, `east`, `north`, `heading_rad`.

---

### 5.2 detection_to_global_node

**Executable**: `detection_to_global_node`  
**Role**: Turn boat-relative detections (range, bearing) from LiDAR (and optionally CV fusion) into **global (east, north)** in the map frame.

#### Parameters

- `use_fused_detections` (default `true`): If true, subscribe to `/fused_buoys` (LiDAR+CV fusion, with `class_id`/`class_name`); if false, subscribe to `/tracked_buoys_json` (LiDAR only, no class). Planning consumes `/global_detections` either way.
- `boat_pose_topic`: `/boat_pose`
- `global_detections_topic`: `/global_detections`
- `map_frame`: `map`
- `merge_by_proximity_m`: 0 = no merging; >0 merges detections within that distance in the global frame.

#### Subscriptions (one of two, based on `use_fused_detections`)

| Topic | When | Description |
|-------|------|-------------|
| `/boat_pose` | always | Current boat position and heading in map frame. |
| `/fused_buoys` | `use_fused_detections=true` | Fused LiDAR+CV buoys with `class_id`, `class_name`. Source in output = `"fused"`. |
| `/tracked_buoys_json` | `use_fused_detections=false` | LiDAR-only buoys (no class). Source in output = `"lidar"`. |

#### Logic

1. **Boat pose**: Keeps the latest `(east, north, heading_rad)` from `/boat_pose`.
2. **Boat-relative → global**: For each detection with `range_m` and `bearing_rad` (bearing from boat +X, CCW positive):
   - `alpha = heading_rad + bearing_rad` (absolute bearing in map frame).
   - `east_global = boat_east + range_m * cos(alpha)`
   - `north_global = boat_north + range_m * sin(alpha)`
3. **Detection callback**: Reads either `/fused_buoys` or `/tracked_buoys_json`; converts each buoy to `GlobalDetection` with `east`, `north`, `range_m`, `bearing_global_rad`, `source` (`"fused"` or `"lidar"`), `class_id` (0–22 from class_mapping; 255 = unknown).
4. **Publish**: Optionally merges by proximity, then publishes one `GlobalDetectionArray` to `/global_detections`.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/global_detections` | `global_frame/GlobalDetectionArray` | All detections in map frame: east, north, range_m, bearing_global_rad, source, id, etc. |

**GlobalDetection** (simplified): `header`, `east`, `north`, `range_m`, `bearing_global_rad`, `source` (`"fused"` or `"lidar"`), `class_name`, `class_id` (0–22 or 255 = unknown), `id`.

---

## 6. LiDAR Buoy Pipeline (pointcloud_filters)

Launch file: `pointcloud_filters/launch/buoy_pipeline.launch.py`.  
Starts: Unitree driver → lidar_range_filter → buoy_detector → buoy_tracker → buoy_visualizer, tracked_buoy_visualizer; optionally RViz.

---

### 6.1 unitree_lidar_ros2_node (Unitree LiDAR driver)

**Package**: `unitree_lidar_ros2`  
**Role**: Reads from the LiDAR at `/dev/ttyUSB0` and publishes raw point clouds.

#### Parameters (from buoy_pipeline.launch.py)

- `port`: `/dev/ttyUSB0`
- `cloud_frame`: `unilidar_lidar`
- `cloud_topic`: `unilidar/cloud`
- `range_max`, `range_min`, `rotate_yaw_bias`, etc.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | Raw points; frame_id = `unilidar_lidar`. |

No GPS or compass are used here; the pipeline is purely LiDAR until detections are sent to `detection_to_global_node`.

---

### 6.2 lidar_range_filter

**Executable**: `lidar_range_filter`  
**Role**: Apply **static rotations** and filters so that the point cloud is in **boat frame** with **+X forward**. Optionally apply TF; in the current launch, TF is disabled and all alignment is done by rotations.

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | Raw LiDAR cloud in `unilidar_lidar` frame. |

#### Parameters (from buoy_pipeline.launch.py)

- `input_topic`: `/unilidar/cloud`
- `output_topic`: `/points_filtered`
- `base_frame`: `base_link`
- `use_tf_transform`: **False** — no TF lookup; only static rotations and filters.
- **Rotations** (applied in order to every point):
  1. **Z (clockwise)** `rotate_cw_deg`: 202.5°
  2. **X (clockwise)** `rotate_cw_x_deg`: -30° (i.e. 30° CCW around X)
  3. **Z again (CCW)** `rotate_ccw_z2_deg`: 90°
  4. **Y (CCW)** `rotate_ccw_y_deg`: 0°
- **Filters**: `z_min`, `z_max`, `range_max` (horizontal), `fov_max_angle_from_x` (0 = no FOV limit), `z_max_cutoff`, optional accumulation.

Rotation order in code: Z → X → Z2 → Y. Result: the cloud’s +X is aligned with the boat’s forward direction. The output is labeled `base_link` and is used as the boat-frame cloud for detection.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/points_filtered` | `sensor_msgs/PointCloud2` | Filtered and rotated cloud; frame_id = `base_link`. |

#### Optional

- Service `~/get_cloud_window` for time-windowed cloud data.
- If `use_tf_transform` were True, the node would look up `base_frame` from the cloud frame and apply that transform; currently unused.

---

### 6.3 buoy_detector

**Executable**: `buoy_detector`  
**Role**: Cluster the filtered point cloud with **DBSCAN** and output buoy detections as (range, bearing) in the boat frame.

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/points_filtered` | `sensor_msgs/PointCloud2` | Filtered cloud in `base_link`. |

#### Parameters (from launch, selected)

- `input_topic`: `/points_filtered`
- `output_topic`: `/buoy_detections`
- `eps`: DBSCAN neighborhood radius (m); e.g. 0.85
- `min_samples`: Minimum points to form a cluster; e.g. 1
- `min_lateral_extent`, `max_lateral_extent`: Reject clusters that are too small or too large (e.g. walls)
- `min_points_final`, `small_cluster_*`, `confidence_scale`, optional RANSAC for water plane, etc.

#### Logic

1. Extract (x, y, z) from PointCloud2.
2. Optionally remove water plane with RANSAC.
3. Run DBSCAN in (x, y); extract clusters.
4. For each cluster: compute centroid, convert to polar (range, bearing), check size/extent/confidence.
5. Publish `BuoyDetectionArray`: each detection has `range`, `bearing` (rad), `z_mean`, `num_points`, `confidence`, etc.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/buoy_detections` | `pointcloud_filters/BuoyDetectionArray` | One entry per detected buoy: range, bearing (boat frame), z_mean, confidence, etc. |

**BuoyDetection** (simplified): `range`, `bearing`, `z_mean`, `num_points`, `lateral_extent`, `radial_extent`, `confidence`.

---

### 6.4 buoy_tracker

**Executable**: `buoy_tracker`  
**Role**: Assign **persistent IDs** to buoys across frames and smooth positions (nearest-neighbor association + exponential smoothing).

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/buoy_detections` | `pointcloud_filters/BuoyDetectionArray` | New detections each frame. |

#### Parameters (from launch)

- `association_threshold`: max distance (m) to match a detection to an existing track
- `max_consecutive_misses`: frames before a track is removed
- `position_alpha`: smoothing (0 = keep old, 1 = use new)
- `min_observations_for_publish`, `min_observations_to_add`, `candidate_max_consecutive_misses`

#### Logic

- Maintains a set of **tracked buoys** (id, range, bearing, x, y, …) and **candidates**.
- Each frame: associate new detections to existing tracks by distance; update matched tracks (smoothing), create candidates for unmatched detections, promote candidates to tracks after enough observations, drop tracks after too many misses.
- Publishes the same content in two forms: ROS message and JSON.

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/tracked_buoys` | `pointcloud_filters/TrackedBuoyArray` | Tracked buoys with stable IDs. |
| `/tracked_buoys_json` | `std_msgs/String` | JSON: `{ "timestamp", "frame_id", "num_detections", "detections": [ { "id", "range", "bearing", "x", "y", "z_mean", "confidence", ... } ] }`. Used by **detection_to_global_node**. |

**TrackedBuoy** (simplified): `id`, `range`, `bearing`, `x`, `y`, `z_mean`, `confidence`, `observation_count`, `first_seen`, `last_seen`.

---

### 6.5 buoy_visualizer

**Executable**: `buoy_visualizer`  
**Role**: Visualize **raw** buoy detections in RViz (no tracking IDs).

#### Subscriptions

| Topic | Type |
|-------|------|
| `/buoy_detections` | `pointcloud_filters/BuoyDetectionArray` |

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/buoy_markers` | `visualization_msgs/MarkerArray` | Cylinder markers at (range, bearing) in base_link; confidence → color. |

Used for debugging the detector output in RViz (fixed frame `base_link`).

---

### 6.6 tracked_buoy_visualizer

**Executable**: `tracked_buoy_visualizer`  
**Role**: Visualize **fused** buoys (LiDAR tracks + class from CV) when the fusion node is running.

#### Subscriptions

| Topic | Type |
|-------|------|
| `/fused_buoys` | `pointcloud_filters/FusedBuoyArray` |

#### Publications

| Topic | Type |
|-------|------|
| `/tracked_buoy_markers` | `visualization_msgs/MarkerArray` |

For the **mapping/boat-position pipeline only** (no CV/fusion), the main visualization of LiDAR detections is `/buoy_markers` from `buoy_visualizer`. `tracked_buoy_visualizer` is used when CV–LiDAR fusion is running.

---

## 7. End-to-End Data Flow

```
Pixhawk (GPS + Compass)
        │
        ▼
  MAVROS  →  mavros/global_position/global
         →  mavros/global_position/compass_hdg
        │
        ▼
  boat_state_node  →  /boat_pose (east, north, heading_rad)
                   →  TF map → base_link
        │
        │     Unitree LiDAR  →  /unilidar/cloud (unilidar_lidar)
        │              │
        │              ▼
        │     lidar_range_filter  →  /points_filtered (base_link, +X forward)
        │              │
        │              ▼
        │     buoy_detector  →  /buoy_detections (range, bearing per buoy)
        │              │
        │              ▼
        │     buoy_tracker   →  /tracked_buoys
        │                   →  /tracked_buoys_json  ─────┐
        │                                                │
        ▼                                                ▼
  detection_to_global_node  ←────────────────────────────┘
        │
        │  (boat_pose + tracked_buoys_json)
        ▼
  /global_detections  (east, north, range_m, bearing_global_rad, source=lidar, id)
```

- **Boat pose** comes only from GPS + compass (MAVROS); no LiDAR in that part.
- **LiDAR** is turned into boat-frame detections (range, bearing), then tracked, then converted to global (east, north) using the current boat pose.

---

## 8. Message Types Reference

### global_frame

- **BoatPose**: `header`, `east` (float64), `north` (float64), `heading_rad` (float64). Heading: 0 = East, CCW positive.
- **GlobalDetection**: `header`, `east`, `north`, `range_m`, `bearing_global_rad`, `source`, `class_name`, `class_id`, `id`.
- **GlobalDetectionArray**: `header`, `GlobalDetection[] detections`.

### pointcloud_filters

- **BuoyDetection**: `range`, `bearing`, `z_mean`, `num_points`, `lateral_extent`, `radial_extent`, `confidence`.
- **BuoyDetectionArray**: `header`, `BuoyDetection[] detections`.
- **TrackedBuoy**: `id`, `range`, `bearing`, `x`, `y`, `z_mean`, `confidence`, `observation_count`, `first_seen`, `last_seen`.
- **TrackedBuoyArray**: `header`, `TrackedBuoy[] buoys`.

---

## 9. Verification and Debugging

### Sourcing the workspace

To use custom types (e.g. `ros2 topic echo /boat_pose` or `/global_detections`), source the mapping workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /home/lorenzo/Repos/School/autonomy-ws-25-26/mapping/install/setup.bash
```

### Useful topics

| Topic | What to check |
|-------|----------------|
| `mavros/global_position/global` | GPS lat/lon. |
| `mavros/global_position/compass_hdg` | Compass heading (degrees). |
| `/boat_pose` | Boat (east, north, heading_rad) in map frame. |
| `/points_filtered` | Filtered cloud in base_link; +X should be forward. |
| `/buoy_detections` | Raw detections (range, bearing). |
| `/tracked_buoys_json` | Tracked buoys with IDs (JSON). |
| `/global_detections` | Final global (east, north) per buoy; frame_id = map. |

### RViz

- **Fixed frame**: `base_link` for LiDAR and boat-relative markers; `map` for global view if you have a map display.
- **Point cloud**: Add `/points_filtered`; check that forward direction is +X.
- **Markers**: Add `/buoy_markers` to see detector output in boat frame.

### Quick sanity checks

1. **Boat pose**: With boat stationary, `ros2 topic echo /boat_pose` — east/north should be stable, heading_rad should match compass.
2. **Global detections**: `ros2 topic echo /global_detections` — each detection should have `east`, `north` in meters from origin, `source: lidar`, and stable `id` over time.
3. **Units**: All positions in the map frame are in **meters** (east, north from origin), not lat/lon. Origin is set from the first GPS fix (or from parameters) by `boat_state_node`.

---

This README reflects the pipeline as used in **comp_single_camera.sh**, **comp.sh**, and the **buoy_pipeline** and **global_frame** launch files. All nodes and topics described here are part of the current mapping and boat-position system.
