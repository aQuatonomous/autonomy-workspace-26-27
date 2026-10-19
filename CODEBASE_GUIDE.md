# Codebase Guide — Autonomy Workspace 2025–26

**ROS 2 Jazzy | Gazebo Harmonic | Unitree L1 LiDAR | Arducam AR0234 | Pixhawk + MAVROS**

This document is a complete reference for every important file and folder in the repo.
It exists so that next year's team can understand what was built, what each piece does,
and exactly what needs to change when deploying on new hardware or at a new venue.

---

## Architecture Overview

The full competition stack is a linear pipeline from raw sensors to actuator commands:

```
┌─────────────────────────────────────────────────────────────────────────┐
│  SENSORS                                                                │
│  Arducam ×3 (/dev/video*)     Unitree L1 LiDAR (/dev/ttyUSB0)          │
│       │                                 │                               │
│  [cv_ros_nodes]               [pointcloud_filters + unitree_lidar_ros2] │
│  image_raw → preprocess         /unilidar/cloud                        │
│  → YOLO inference               → range filter → buoy_detector         │
│  → vision_combiner              → buoy_tracker                         │
│  → distance_estimator                │                                  │
│       │ /combined/detection_info_with_distance                          │
│       └──────────────────┬──────────────────────┘ /tracked_buoys       │
│                           ▼                                             │
│                    [cv_lidar_fusion]                                    │
│                    /fused_buoys                                         │
│                           │                                             │
│                    [global_frame]                                       │
│                    GPS+compass → /boat_pose                             │
│                    fused_buoys → /global_detections                     │
│                           │                                             │
│                    [Global_Planner]  ← planning/ library                │
│                    10 Hz planning loop                                  │
│                    /cmd_vel → MAVROS → Pixhawk → thrusters              │
│                                                                         │
│  SIDE CHANNELS                                                          │
│  [sound_signal] → sound_signal_interupt (stop/start)                   │
│  [task_sequence_coordinator] → competition messages                     │
│  [message_node] → protobuf reports → ground station                    │
│  [web_server_map] → http://localhost:8080 live 2D map                  │
└─────────────────────────────────────────────────────────────────────────┘
```

MAVROS topics used by the stack:
- **In:** `/mavros/global_position/global`, `/mavros/global_position/compass_hdg`, `/mavros/global_position/gp_vel`, `/mavros/state`
- **Out:** `/uas1/mavros/setpoint_velocity/cmd_vel_unstamped` (motion), `/mavros/set_mode` (GUIDED), `/mavros/setpoint_position/global` (GPS waypoints)

---

## 1. Top-Level Shell Scripts

These are the scripts you run day-to-day. All must be run from the **repository root**.

### `build.sh`

Builds the entire ROS 2 workspace (`src/`) in one command.

**What it does:**
1. Detects stale CMake caches (from a previous clone path) and auto-deletes `build/`, `install/`, `log/` if found — prevents "No rule to make target" errors when the repo is moved.
2. Sources `/opt/ros/jazzy/setup.bash` with a clean environment (`AMENT_PREFIX_PATH` unset).
3. Exports `GZ_VERSION=harmonic` so `gz-waves1` resolves to Gazebo Harmonic instead of Garden.
4. Runs `colcon build --symlink-install --base-paths src`.

**To change:** Nothing for normal use. If upgrading to a new ROS distro, update line `source /opt/ros/jazzy/setup.bash` and the `GZ_VERSION` export.

---

### `comp_task1.sh` — `comp_task4.sh`

One-shot competition run scripts, one per task. They all follow the same pattern:

1. Run `set_camera_fps.sh single` to configure the camera and write `.camera_devices`.
2. Source ROS + workspace.
3. Launch MAVROS, global_frame, LiDAR pipeline, CV pipeline, CV-LiDAR fusion, and global planner in background process groups.
4. Optionally launch the sound pipeline (`SOUND=1`).
5. On Ctrl+C, kill all process groups, run `parse_and_summarize` from `logging_lib.sh`.

| Script | `task_id` | CV `task` arg | `enable_indicator_buoy` | `enable_task4` |
|--------|-----------|---------------|------------------------|----------------|
| `comp_task1.sh` | 1 | 2 | false | false |
| `comp_task2.sh` | 2 | 2 | true | false |
| `comp_task3.sh` | 3 | 3 | true | false |
| `comp_task4.sh` | 4 | 4 | true | true |

**Env vars:**
- `NOLOG=1` — disable file logging
- `SOUND=1` — also start sound pipeline
- `FCU_URL` — overrides default `/dev/ttyACM0:57600`

**What to change for new deployment:**
- `FCU_URL` if Pixhawk is on a different port
- Camera device paths (driven by `set_camera_fps.sh`)
- `conf_threshold` (currently `0.3`) if detection rate is too low/high

---

### `run_comp_single_camera.sh`

All-tasks run with a single center camera. Same as `comp_task3.sh` but with all CV features on (`enable_task4:=true`, `enable_number_detection:=true`, lower `conf_threshold:=0.1`). Overrides task via `TASK_ID` env var (default 3).

---

### `run_comp_three_cameras.sh`

Same as `run_comp_single_camera.sh` but launches all 3 cameras. Uses lower `preprocess_fps:=5` and longer `inference_interval` values to avoid overloading the GPU/CPU.

**What to change:** Camera device paths in `set_camera_fps.sh three` call, and potentially `preprocess_fps` depending on hardware.

---

### `run_task_sequence.sh`

**Alternative competition mode** — runs the task *sequence coordinator* instead of the autonomous planner. This is messaging + timing only; the boat navigates manually or via other means.

Launches: MAVROS + CV (for patrol boat detection) + sound pipeline + task_sequence_coordinator + message_node.

Use this when you want the competition messaging protocol (gate pass, object detected, etc.) without full autonomy.

---

### `run_mavros.sh`

Starts MAVROS only. Useful for manual testing and telemetry monitoring.

**Hardcoded:** `FCU_URL=/dev/ttyACM0:57600`. Override with `FCU_URL=/dev/ttyUSB1:57600 ./run_mavros.sh`.

---

### `run_sound_pipeline.sh`

Starts the 3-node sound stack: audio capturer → sound_signal detector → message_node.
Standalone — does not require the full CV/planning stack.

---

### `set_camera_fps.sh`

Configures USB cameras via `v4l2-ctl` and writes device paths to `.camera_devices` (git-ignored, machine-specific).

**Hardcoded device paths (must edit for new hardware):**

| Mode | Port | Device path |
|------|------|-------------|
| single | 1.4.2 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0` |
| three cam0 | 1.4.2 | same as above |
| three cam1 | 1.2 | `platform-3610000.usb-usb-0:1.2:1.0-video-index0` |
| three cam2 | 1.1 | `platform-3610000.usb-usb-0:1.1:1.0-video-index0` |

These are Jetson-specific USB port paths. On a new machine, run `./monitor_camera_move.sh` while plugging cameras in to discover the correct paths, then edit `set_camera_fps.sh`.

The script also sets: `YUYV 960x600 @ 15 fps` for each camera.

---

### `kill_ros_processes.sh`

Aggressively kills all ROS processes: `ros2 launch`, `mavros`, CV nodes, LiDAR nodes, planning, sound, bag replay, and web map visualizer. Use when Ctrl+C leaves zombie processes or when `ros2 topic list` shows stale nodes.

---

### Monitoring Scripts

| Script | Purpose |
|--------|---------|
| `monitor_key_topics.sh` | Live TUI: `/boat_pose`, `cmd_vel`, `/fused_buoys` refreshed in-place |
| `monitor_competition_topics.sh` | Live echo of `/cur_task`, messages, sound signal, MAVROS state |
| `monitor_camera_move.sh` | Detect USB port ↔ camera mapping as you plug/unplug |

---

## 2. `src/` ROS Packages

### `cv_ros_nodes`

**What it is:** The entire camera perception pipeline as a set of composable ROS nodes.

**Pipeline:**

```
v4l2_camera_node      → /cameraN/image_raw
vision_preprocessing  → /cameraN/image_preprocessed
vision_inference      → /cameraN/detection_info  (YOLO TensorRT bounding boxes)
task4_supply_processor → /cameraN/task4_detections  (blob detection for vessels)
indicator_buoy_processor → /cameraN/indicator_detections  (diamond colour indicator)
vision_combiner       → /combined/detection_info  (merged from all cameras)
maritime_distance_estimator → /combined/detection_info_with_distance  ← final output
```

**Launch files:**

| Launch file | Use case |
|-------------|----------|
| `launch_cv.py` | 3-camera full pipeline |
| `launch_cv_single_camera1.py` | Single center camera (camera1) |
| `launch_cv_sim.py` | Simulation (skips v4l2 driver, reads Gazebo camera bridges) |

**Key launch arguments:**

| Arg | Default | Notes |
|-----|---------|-------|
| `resolution` | `960,600` | Native YUYV resolution; lower to `480,360` for bandwidth |
| `engine_path` | auto via `workspace_paths.py` | Must rebuild engine for new GPU |
| `conf_threshold` | `0.1` | Lower = more detections, more false positives |
| `camera_devices` | Jetson by-path (3 cameras) | Edit or use `set_camera_fps.sh` output |
| `enable_task4` | `false` | Enable Task 4 vessel supply drop detector |
| `enable_indicator_buoy` | `false` | Enable Task 2/3 colour indicator detector |
| `enable_number_detection` | `false` | Enable Task 5 docking digit detector |
| `staleness_threshold` | `20.0` s | Lower to `8.0` when pipeline is stable |
| `inference_interval_front` | `4` | Run YOLO every N frames (front camera) |
| `inference_interval_sides` | `6` | Run YOLO every N frames (side cameras) |
| `distance_scale_factor` | `1.0` | Calibration multiplier — measure with `camera_calibration/` |

**`workspace_paths.py`** (`src/cv_ros_nodes/cv_ros_nodes/workspace_paths.py`):
A portable path resolver added this season. Finds the repo root without relying on hardcoded `~/autonomy-ws-25-26`. Resolution order:
1. `$AUTONOMY_WS` environment variable
2. Walk up from `__file__` looking for a dir containing both `src/` and `computer_vision/`
3. Legacy fallbacks: `~/Repos/School/autonomy-ws-25-26`, `~/autonomy-ws-25-26`

**Hardcoded values to change per deployment:**
- Camera device paths in `camera_devices` arg (or in `set_camera_fps.sh`)
- TRT engine files (must rebuild on new GPU — see `computer_vision/models/`)
- Camera mounting angles: `[-70°, 0°, +70°]` in `maritime_distance_estimator.py` — change if cameras are remounted
- Camera FOV: `HORIZONTAL_FOV_PER_CAMERA_DEG = 85.0` — confirm with your lens spec
- `distance_scale_factor` — recalibrate with `camera_calibration/estimate_distance_angle.py`

**Reusability:** Architecture is fully reusable. The YOLO model needs retraining if the competition objects change significantly. Engines need rebuilding on any new GPU.

---

### `cv_lidar_fusion`

**What it is:** Matches CV bearing detections to LiDAR tracks to assign class labels to tracked buoys.

| | |
|--|--|
| **Subscribes** | `/combined/detection_info_with_distance`, `/tracked_buoys` |
| **Publishes** | `/fused_buoys` (`FusedBuoyArray`) |
| **Fusion logic** | Bearing difference < `0.15 rad` → match; assign CV class_id to LiDAR track |
| **Fallback** | Unmatched LiDAR tracks get `class_id=255` (unknown) |

**Hardcoded:** `bearing_threshold_rad = 0.15`. Tune if getting too many or too few matches.

**Reusability:** Reuse as-is. If the bearing threshold produces bad matches, increase it slightly. Works as long as the upstream topic names are unchanged.

---

### `global_frame`

**What it is:** Converts GPS + compass to a local 2D map frame, and transforms fused detections into global (map-frame) coordinates for the planner.

**Nodes:**

| Node | Subscribes | Publishes |
|------|------------|-----------|
| `boat_state_node` | `/mavros/global_position/global`, `/mavros/global_position/compass_hdg` | `/boat_pose`, `/boat_pose_stamped`, TF `map→base_link` |
| `detection_to_global_node` | `/boat_pose`, `/fused_buoys` (or `/tracked_buoys_json`) | `/global_detections` (`GlobalDetectionArray`) |

**How it works:** The first GPS fix is used as the map origin (lat/lon = 0, 0). All subsequent positions are expressed as meters offset from that origin in a right-handed ENU frame.

**Launch arg:** `use_fused_detections:=true` — set to `false` to use raw LiDAR tracks without CV class labels (useful for testing without cameras).

**Hardcoded:** MAVROS topic prefix. Scripts use `/mavros/...` here, but `cmd_vel` outputs use `/uas1/mavros/...`. If your MAVROS namespace differs, check both `global_frame` launch and the comp scripts.

**Reusability:** Fully reusable. No venue-specific values.

---

### `pointcloud_filters` + `unitree_lidar_ros2` + `unitree_lidar_sdk`

**What it is:** Full LiDAR buoy pipeline from raw point cloud to tracked 2D buoy positions.

**Launch:** `ros2 launch pointcloud_filters buoy_pipeline.launch.py`

**Pipeline:**

```
unitree_lidar_ros2_node  →  /unilidar/cloud  (raw 3D cloud, frame: unilidar_lidar)
lidar_range_filter       →  /points_filtered  (rotated, z-clipped, range-clipped, accumulated)
buoy_detector            →  /buoy_detections  (DBSCAN clusters → 2D centroids)
buoy_tracker             →  /tracked_buoys, /tracked_buoys_json
```

**Hardware-specific rotation values in `buoy_pipeline.launch.py`:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `rotate_cw_deg` | `202.5°` | LiDAR yaw offset from boat forward |
| `rotate_cw_x_deg` | `-30.0°` | LiDAR tilt correction |
| `rotate_ccw_z2_deg` | `90.0°` | Fine alignment after tilt |

**These rotations are mount-specific.** If the LiDAR is remounted, re-measure the angles by pointing the boat at a known target and adjusting until the detected point cloud aligns with the bow direction.

**Other tunable values:**

| Parameter | Value | Effect |
|-----------|-------|--------|
| `z_min` | `-0.37` m | Clips ground returns |
| `z_max` | `5.0` m | Clips overhead objects |
| `range_max` | `15.0` m | Max detection range |
| `accumulation_window` | `0.6` s | Denser cloud from spinning LiDAR |
| DBSCAN `eps` | `0.85` m | Cluster radius — increase if buoys are missed |
| `max_consecutive_misses` | `50` | Frames before tracker drops a track |

**Hardcoded:** LiDAR serial port is `/dev/ttyUSB0`. The launch file raises `RuntimeError` immediately if the port doesn't exist. Add a udev rule to make the port consistent:
```bash
# /etc/udev/rules.d/99-unitree-lidar.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyUSB_lidar"
```
Then change `lidar_port = '/dev/ttyUSB0'` in `buoy_pipeline.launch.py`.

**Reusability:** DBSCAN algorithm and tracker are fully reusable. **Must re-tune** mount rotations and z-clip values for new LiDAR mounting position.

---

### `Global_Planner`

**What it is:** The ROS wrapper that feeds sensor data into the `planning/` Python library and sends velocity commands to MAVROS.

**Subscribes:**
- `/global_detections` — from `global_frame`
- `/mavros/global_position/global` — GPS (lat/lon)
- `/mavros/global_position/compass_hdg` — heading (degrees)
- `/mavros/global_position/gp_vel` — velocity
- `/mavros/state` — MAVROS arm/mode state
- `/sound_signal_interupt` — 1 = stop, 2 = resume

**Publishes:**
- `/planned_path` (`nav_msgs/Path`) — for visualization
- `/curr_task` (`Int32`) — current task number
- `/mavros/setpoint_velocity/cmd_vel_unstamped` (`Twist`) — motion commands
- `/gs_message_send` — competition messages (gate pass, object detected)
- `/messages/gate_pass` — individual message topics

**Launch args (`global_planner.launch.py`):**

| Arg | Default | Notes |
|-----|---------|-------|
| `task_id` | `1` | Which task to run (0=test, 1–4) |
| `planning_hz` | `10.0` | Planning loop frequency |
| `map_bounds` | `[]` | Optional `[min_x, min_y, max_x, max_y]` in meters |
| `pump_hold_script` | `""` | Absolute path to `pump_hold.py` for Task 4 |
| `cmd_vel_topic` | `/mavros/setpoint_velocity/cmd_vel_unstamped` | Override for different MAVROS namespace |

**Built-in watchdogs** (in `watchdogs.py`):
- Collision guard: if obstacle < 2 m, reverse 1 m at 0.5 m/s
- Spin search: 30 s timeout → 20 s spin → max 3 search cycles before giving up

**Reusability:** Fully reusable. Set `task_id` and `cmd_vel_topic` per deployment. The planner requires the boat to be in **GUIDED** mode to accept cmd_vel.

---

### `task_sequence_coordinator`

**What it is:** Timed competition messaging sequence. Not autonomous navigation — it sends the correct RoboNation protocol messages at the right times and handles the patrol-boat interrupt.

**Nodes:**

| Node | Role |
|------|------|
| `task_sequence_coordinator_node` | State machine: WAITING → TASK1 → TASK2 → ... → DONE |
| `patrol_boat_detector` | CV-based yellow flattened vessel detector → `/patrol_boat/detected` |

**Publishes:**
`/cur_task`, `/messages/gate_pass`, `/messages/object_detected`, `/messages/object_delivered`, `/messages/docking`, `/messages/patrol_boat`, `/mavros/setpoint_position/global`

**Subscribes:**
`/mavros/global_position/global`, `/sound_signal_interupt_freq`, `/mavros/state`, `/patrol_boat/detected`

**Venue-specific parameters (must update every competition):**

| Parameter | 2025–26 Value | Where to change |
|-----------|---------------|-----------------|
| `yellow_buoy_lat` | `27.37420` | `task_sequence_coordinator_node.py` line 71 |
| `yellow_buoy_lon` | `-82.45300` | line 72 |
| `marina_lat` | `27.37429` | line 73 |
| `marina_lon` | `-82.35259` | line 74 |
| Arrival threshold | `5 m` | Same file |

These are Sarasota, FL coordinates. **Update these before every competition.**

**Reusability:** The state machine logic and patrol-boat detector are reusable. GPS waypoints and timing must be updated per venue.

---

### `message_node` + `message_node_msgs`

**What it is:** Aggregates boat state and competition events and sends Protobuf-encoded reports to the RoboNation ground station server.

**Subscribes:** `/cur_task`, `/mavros/state`, GPS, velocity, heading, all `/messages/*`, `sound_signal_interupt_freq`

**Publishes:** `gs_message_send` (internal trigger for the sending loop)

**Hardcoded values that must change:**

| Value | Current | File | Line |
|-------|---------|------|------|
| Team ID | `"QUEU"` | `message_node.py` | search `team_id` |
| Vehicle ID | `"The Frontenac"` | `message_node.py` | search `vehicle_id` |
| GS server IP | `10.10.10.1` | `message_node.py` | line 163 |
| GS server port | `50000` | `message_node.py` | line 163 |
| Patrol boat server | `127.0.0.1:50000` | `message_node.py` | line 333 |

The Protobuf schema is in `message_node/message_node/report.proto` — regenerate `report_pb2.py` if the schema changes: `protoc --python_out=. report.proto`.

**Reusability:** Protocol and infrastructure fully reusable. **Must update** team/vehicle ID and GS IP at every competition.

---

### Sound Stack

Three packages work together:

#### `audio_common`
Records microphone input via PortAudio.
- **Publishes:** `/audio` (`AudioStamped`)
- **Config:** 16 kHz, mono, chunk size 500, device -1 (system default mic)
- **Change:** `device` parameter if mic is not the default input

#### `sound_signal`
FFT-based 1-blast / 2-blast detector.
- **Subscribes:** `/audio`
- **Publishes:** `/sound_signal_interupt` (`Int32`, 1=one-blast, 2=two-blast), `/sound_signal_interupt_freq` (`SoundSignalWithFreq`)
- **Config:** Target frequency `800 Hz`, sensitivity `5`, 1.5 s cooldown between detections
- **Change:** `sensitivity` and `frequency` parameters in `sound_pipeline_launch` — tune at the competition venue to match the actual horn frequency and ambient noise level

#### `sound_pipeline_launch`
Convenience launch that starts all three nodes (capturer + detector + message_node).

**Reusability:** Architecture is reusable. **Tune** `frequency` and `sensitivity` on-site — the horn frequency varies by competition. The 1.5 s cooldown prevents double-triggers.

---

### `web_server_map`

**What it is:** A live 2D map UI served over HTTP showing the boat position and all detections in real time.

- **Subscribes:** `/boat_pose`, `/global_detections`
- **Serves:** `http://0.0.0.0:8080` (default)
- **Depends on:** `aiohttp` Python package
- **Run:** `ros2 run web_server_map map_visualizer_node`
- **Remote access:** `ssh -L 8080:localhost:8080 user@boat` then open `http://localhost:8080` on your laptop

**Reusability:** Fully reusable as-is. No venue-specific values.

---

### `asv_wave_sim`

**What it is:** Gazebo Harmonic simulation of the competition environment, including wave physics, boat model, buoys, gates, and ArduPilot SITL integration.

**Key components:**

| Component | Purpose |
|-----------|---------|
| `gz-waves/` | C++ wave physics plugin (`gz-waves1`) — computes realistic water surface |
| `gz-waves-models/worlds/aquatonomous_world.sdf` | Competition world with buoys, gates, rope holds, wave model |
| `gz-waves-models/models/` | Individual model SDFs (boat, buoys, rope holds, etc.) |
| `run_full_simulation_tmux.bash` | One-command full sim: Gazebo + SITL + MAVROS in a tmux session |
| `test_gazebo_gui.sh` | Gazebo only (no SITL) — fastest way to check the world |
| `test_sitl.sh` | ArduPilot SITL only |
| `test_mavros.sh` | MAVROS bridge to SITL |
| `test_bridges.sh` | ROS 2 ↔ Gazebo bridges (lidar + 3 cameras) |
| `run_gazebo_standalone.sh` | Gazebo without SITL; works even without `ourboat` model |

**ROS topics (via `ros_gz_bridge`):** `/camera0/image_raw`, `/camera1/image_raw`, `/camera2/image_raw`, `/laser_points`

**External dependencies (not in this repo):**

| Dependency | Purpose | Default location |
|-----------|---------|-----------------|
| `SITL_Models/` | `ourboat` model SDF | `~/Repos/School/SITL_Models/Gazebo` |
| `ardupilot_gazebo/` | ArduPilotPlugin.so | `~/Repos/School/ardupilot_gazebo` |
| `ardupilot/` | `sim_vehicle.py` (SITL) | `~/Repos/School/ardupilot` |
| `bridge_ws/` | `ros_gz_bridge` | `~/bridge_ws` |

Scripts probe both `~/Repos/School/<dep>` and `~/<dep>` automatically.

**Key environment variables:**
```bash
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"   # Required — fixes gz CLI routing
export GZ_VERSION=harmonic
export LD_LIBRARY_PATH="<install>/gz-waves1/lib:$LD_LIBRARY_PATH"
```

**Hardcoded network:**
- SITL ↔ Gazebo: UDP `127.0.0.1:9002`
- MAVROS ↔ SITL: TCP `localhost:5760`

**Reusability:** Fully reusable on Jazzy + Harmonic + ArduPilot. The world file (`aquatonomous_world.sdf`) should be updated each year to match the new competition course layout. Wave parameters are in the SDF under `<plugin name="gz::sim::systems::WavesModel">`.

---

### Supporting Packages

| Package | Purpose | Reuse |
|---------|---------|-------|
| `maveasy` | Forwards `/gs_message_send` → `/mavros/statustext/send` so competition messages appear on FCU display | Optional debug aid |
| `pointcloud_to_laserscan` | Generic ROS package; converts `PointCloud2` ↔ `LaserScan`. Not in main comp pipeline but available. | Yes |
| `message_node_msgs` | Custom msg: `ObjectDetection`, `Dock` | Required by message_node and task_sequence_coordinator |
| `sound_signal_msgs` | Custom msg: `SoundSignalWithFreq` | Required by sound stack and planner |
| `audio_common_msgs` | Custom msg: `AudioStamped` | Required by audio_common |

---

## 3. `computer_vision/` — Non-ROS Assets

CV **ROS nodes** live in `src/cv_ros_nodes`. This folder holds the models, calibration data, and offline development tools.

### `model_building_and_training/`

| File | Purpose |
|------|---------|
| `model.engine` | Main YOLO TRT engine (classes 0–22: buoys, gates, vessels, etc.) |
| `number_detection.engine` | Digit detector TRT engine (digits 1, 2, 3 for Task 5 docking) |
| `aqua_main.pt` | PyTorch weights (source for both engines) |
| `aqua_main.onnx` | ONNX export (intermediate for TRT conversion) |

**TRT engines are GPU-architecture-specific.** If you change GPU (e.g., Jetson AGX → Jetson Orin, or any x86 GPU), you **must** rebuild the engines:
```bash
# 1. Export ONNX from weights:
cd computer_vision/models && python3 export_onnx.py

# 2. Build FP16 engine on the target GPU:
bash computer_vision/models/build_fp16_main_engine.sh

# 3. Build number detection engine:
bash computer_vision/task_specific/Docking/number_detection/build_number_engine.sh
```

See `computer_vision/models/TENSORRT.md` for detailed instructions.

---

### `models/`

| File | Purpose |
|------|---------|
| `export_onnx.py` | Converts `.pt` → `.onnx` (run on any machine with PyTorch) |
| `build_fp16_main_engine.sh` | Converts `.onnx` → `.engine` via `trtexec` (run on target GPU) |
| `TENSORRT.md` | Step-by-step TensorRT build guide |

---

### `class_mapping.yaml`

Maps integer class IDs (0–22) to human-readable names used across the entire stack.

**Critical:** This file must stay in sync with `planning/Global/entities.py`. If you add or rename a class in one place, update the other.

```yaml
# Example entries
0: red_buoy
1: green_buoy
2: red_pole_buoy
3: green_pole_buoy
# ...
255: unknown  # used by cv_lidar_fusion for unmatched LiDAR tracks
```

---

### `camera_calibration/`

| File | Purpose |
|------|---------|
| `calibration_measurements.csv` | Physical measurements at known distances (buoy size vs bbox size) |
| `estimate_distance_angle.py` | Fits a scale factor from measurements |
| `get_buoy_bbox.py` | Extracts bbox sizes from test images |

**How to calibrate `distance_scale_factor`:**
1. Place a known-size buoy (e.g., 30 cm diameter) at a known distance (e.g., 5 m).
2. Record a detection image.
3. Run `get_buoy_bbox.py` to extract bbox pixel size.
4. Run `estimate_distance_angle.py` with the measurement.
5. Use the output scale factor as `distance_scale_factor:=X` in CV launch.

---

### `task_specific/`

| Subfolder | Contents | Status |
|-----------|----------|--------|
| `task_2_3/` | Colour indicator buoy algorithm development (`colour_indicator_buoy_detector.py`), test images, debug outputs | Algorithm is embedded in `cv_ros_nodes/indicator_buoy_processor.py` |
| `task4/` | Supply drop blob detector source (`task4_simplified_pipeline.py`) | Algorithm is embedded in `cv_ros_nodes/task4_supply_processor.py` |
| `Docking/number_detection/` | Docking digit model, `build_number_engine.sh`, README | Active — used for Task 5 |
| `indicator/` | Legacy standalone indicator detector | Superseded by the ROS node version |

**For new tasks:** Add new subfolder here for algorithm development before integrating into `src/cv_ros_nodes`.

---

### `model_testing/`

Offline test scripts for validating engines and distance estimation. Run these on a developer machine without ROS to verify a rebuilt engine works before deploying.

| Script | Purpose |
|--------|---------|
| `test_engine_image.py` | Run inference on a single image using a `.engine` file |
| `test_inference.py` | Video inference test with bbox overlay |
| `run_inference_and_distance.py` | Combined inference + distance estimation test |
| `compute_distance_scale.py` | Alternative scale factor computation |
| `test_onnx_single_image.py` | Test ONNX model before TRT conversion |

---

## 4. `planning/` — Pure Python Planning Library

No ROS imports. All logic is in plain Python and consumed by `src/Global_Planner/global_planner/global_planner_node.py` at runtime via `PLANNING_PATH` or automatic path detection.

### `TaskMaster.py`

Top-level state machine. Decides which task is active and when to transition.

- Transitions from Task 1 → Task 2 → Task 3 based on completion signals from each task planner.
- Can also be initialized to a single `task_id` for direct task runs.

**To change:** Transition conditions (distance thresholds, completion criteria) are near the top of the file.

---

### `Global/Task1.py` — `Global/Task4.py`

Per-task goal computation. Each file implements a `step()` function that:
1. Looks at the current `EntityList` (detected objects in global frame)
2. Returns a goal position and any competition messages to send

| Task | Algorithm summary |
|------|------------------|
| **Task1** | Find two gates; compute intersection line; navigate through in order |
| **Task2** | Transit to debris field; navigate channel using red/green indicator; return |
| **Task3** | Find gate + yellow buoy; read colour indicator (red=circle right, green=circle left) |
| **Task4** | Find yellow vessels; navigate to each; trigger pump script via subprocess |

---

### `Global/entities.py`

Defines all known entity types, gate pairing logic, and the `class_id` ↔ entity type mapping.

**Must stay in sync with `computer_vision/class_mapping.yaml`.** If you retrain the model with new classes, update both files together.

Key constant: `CLASS_ID_TO_ENTITY_TYPE` dict. Also defines gate "no-go walls" that extend 30 m beyond the gate endpoints to keep the boat from going around gates.

---

### `Local/potential_fields_planner.py`

Computes collision-free velocity commands using a potential fields algorithm.

**Tunable constants (top of file):**

| Parameter | Value | Effect |
|-----------|-------|--------|
| `K_ATT` | `2000` | Attractive force toward goal — higher = more aggressive approach |
| `K_REP` | `0.15` | Repulsive force from obstacles — higher = wider obstacle avoidance |
| `D_INFLUENCE` | `2.5` m | Obstacle influence radius |
| `MAX_VELOCITY` | `1.5` m/s | Hard speed cap |
| `GOAL_THRESHOLD` | `2.0` m | Distance at which goal is considered reached |
| `GATE_APPROACH_DIST` | `35.0` m | Start gate alignment from this distance |
| `GATE_STRENGTH` | `0.6` | Gate alignment force |

**Tuning guidance:**
- If the boat overshoots goals: lower `K_ATT` or lower `MAX_VELOCITY`
- If the boat gets stuck near obstacles: raise `K_REP` or raise `D_INFLUENCE`
- If the boat doesn't make it through gates cleanly: raise `GATE_STRENGTH`

---

### Post-Mortem Fix Notes

| File | What it documents |
|------|------------------|
| `planning/GATE_DETECTION_EMERGENCY_FIX.md` | Fix applied when gates weren't being detected reliably |
| `planning/INITIAL_HEADING_FIX.md` | Fix for wrong initial heading causing the boat to spin |
| `planning/POTENTIAL_FIELDS_GAIN_FIX.md` | Gain tuning session notes from competition |
| `planning/TASKMASTER_STATE_MACHINE.md` | Full state machine diagram and transition logic |
| `planning/TRANSITION_2_3_UPDATE.md` | How Task 2→3 transition was updated mid-competition |

**Read these before tuning the planner** — they document why certain values were chosen and what went wrong when they were set differently.

---

## 5. `web_server_map/` (repo root) — Bag Scripts

Scripts for recording and replaying competition bag data (separate from the `src/web_server_map` ROS package).

| Script | Purpose |
|--------|---------|
| `record_map_data.sh` | `ros2 bag record` — captures all navigation topics: `/boat_pose`, `/global_detections`, `/fused_buoys`, `/combined/detection_info_with_distance`, `cmd_vel`, `/planned_path`, `/curr_task`, camera images, LiDAR cloud |
| `replay_map.sh` | Loop-replays a bag file; pair with `ros2 run web_server_map map_visualizer_node` to replay runs visually |

---

## 6. `loggers comp ran/` — Competition Logging

Automatic logging infrastructure used by all `comp_task*.sh` scripts.

| File | Purpose |
|------|---------|
| `logging_lib.sh` | Sourced by comp scripts; tees all output to `run_<timestamp>/raw_log.txt`. `NOLOG=1` disables. |
| `parse_logs.py` | Post-run: splits `raw_log.txt` by node into individual `.log` files + `summary.txt` |
| `run_and_log_competition.sh` | Wrapper: `./run_and_log_competition.sh ./comp_task1.sh` to run any script with automatic log parsing on exit |
| `open_latest_logs.sh` | Opens the most recent `run_*/` folder in the file manager |
| `run_2026-02-22_*/` | Historical run archives from the 2026 competition — useful for post-analysis |

**How logging works:** When a comp script starts and `NOLOG` is not set, `logging_lib.sh` creates a timestamped `run_<timestamp>/` directory and redirects all output through `tee`. On exit, `parse_and_summarize` is called which invokes `parse_logs.py` to split the log by node. Each node gets its own `.log` file and a `summary.txt` highlights key events.

**Reusability:** Fully reusable as-is.

---

## 7. Master Deployment Checklist

Everything that needs to change when deploying on new hardware or at a new venue.

### Hardware-Specific

| Item | 2025–26 Value | Where to change | How to find new value |
|------|---------------|-----------------|----------------------|
| FCU serial port | `/dev/ttyACM0:57600` | `FCU_URL` env var in all `comp_task*.sh` | `ls /dev/ttyACM*` when Pixhawk is plugged in |
| LiDAR serial port | `/dev/ttyUSB0` | `buoy_pipeline.launch.py` line 3 | `ls /dev/ttyUSB*` when LiDAR is plugged in |
| Camera USB paths | Jetson port `1.4.2, 1.2, 1.1` | `set_camera_fps.sh` lines 26–30 | Run `./monitor_camera_move.sh` while plugging cameras |
| LiDAR mount rotations | `202.5°, -30°, 90°` | `buoy_pipeline.launch.py` | Point boat at known target, adjust until cloud aligns |
| Camera mounting angles | `[-70°, 0°, +70°]` | `maritime_distance_estimator.py` | Measure physical mounting angles from boat centerline |
| Distance scale factor | `1.0` (needs calibration) | `launch_cv*.py` `distance_scale_factor` arg | Run `camera_calibration/estimate_distance_angle.py` |
| TRT engine files | Built for 2026 Jetson GPU | `computer_vision/model_building_and_training/` | Rebuild using `computer_vision/models/build_fp16_main_engine.sh` on new GPU |

### Venue-Specific

| Item | 2025–26 Value | Where to change |
|------|---------------|-----------------|
| Yellow buoy GPS | `27.37420, -82.45300` | `task_sequence_coordinator_node.py` lines 71–72 |
| Marina GPS | `27.37429, -82.35259` | `task_sequence_coordinator_node.py` lines 73–74 |
| Sound horn frequency | `800 Hz` | `sound_pipeline_launch` launch arg `frequency` |
| Sound sensitivity | `5` | `sound_pipeline_launch` launch arg `sensitivity` |

### Team/Competition

| Item | 2025–26 Value | Where to change |
|------|---------------|-----------------|
| Team ID | `"QUEU"` | `message_node.py` — `self.team_id` |
| Vehicle ID | `"The Frontenac"` | `message_node.py` — `self.vehicle_id` |
| Ground station IP | `10.10.10.1` | `message_node.py` line 163 |
| Ground station port | `50000` | `message_node.py` line 163 |
| MAVROS namespace | `/uas1/mavros` (cmd_vel) | comp scripts `CMD_VEL_TOPIC` variable |

### Model Retraining (if competition objects change)

1. Collect new training images of new objects
2. Annotate with the same class IDs (or add new IDs to `class_mapping.yaml`)
3. Train in `computer_vision/model_building_and_training/` — update `aqua_main.pt`
4. Export ONNX: `python3 computer_vision/models/export_onnx.py`
5. Build FP16 engine on target GPU: `bash computer_vision/models/build_fp16_main_engine.sh`
6. If class IDs changed: update `planning/Global/entities.py` `CLASS_ID_TO_ENTITY_TYPE`

---

## 8. Reusability Matrix

| Component | Reusability | Notes |
|-----------|-------------|-------|
| `build.sh` | Reuse as-is | Works on any Jazzy + Harmonic install |
| `comp_task*.sh` | Tune params | Update `FCU_URL`, camera paths, thresholds |
| `cv_ros_nodes` pipeline architecture | Reuse as-is | Nodes, topics, combiner all portable |
| YOLO TRT engines | Rebuild on new GPU | Architecture reusable; binaries are GPU-specific |
| YOLO model weights | Retrain if objects change | `aqua_main.pt` works for similar maritime objects |
| `cv_lidar_fusion` | Reuse as-is | Bearing matching logic is general |
| `global_frame` | Reuse as-is | No venue-specific values |
| LiDAR pipeline (DBSCAN + tracker) | Tune mount angles | Algorithm reusable; rotations are mount-specific |
| `Global_Planner` ROS node | Reuse as-is | Set `task_id` and `cmd_vel_topic` |
| Planning library (Task1–4) | Tune gains | Potential field gains and task logic may need competition-specific tuning |
| `planning/Global/entities.py` | Update with CV model | Keep in sync with `class_mapping.yaml` |
| `task_sequence_coordinator` | Update GPS + timing | State machine logic reusable; coordinates are venue-specific |
| `message_node` | Update team/GS details | Protocol reusable; credentials must change |
| Sound stack | Tune frequency/sensitivity | Algorithm reusable; parameters are venue-specific |
| `web_server_map` | Reuse as-is | No venue-specific values |
| `asv_wave_sim` simulation | Reuse as-is | Update world SDF for new course layout |
| Logging infrastructure | Reuse as-is | No venue-specific values |
| Bag recording scripts | Reuse as-is | No venue-specific values |

---

## 9. Recommended Reading Order (New Team Member)

1. **This document** — overall mental model
2. `mapping/MAPPING_AND_BOAT_POSITION_README.md` — how position and detections flow through the system
3. `computer_vision/NODES.md` — full CV topic reference with message formats
4. `planning/TASKMASTER_STATE_MACHINE.md` — what the planner actually does per task
5. `computer_vision/COMPETITION.md` — which script to run for which competition task
6. `src/asv_wave_sim/SIMULATION.md` — how to bring up the full simulation stack
7. `planning/POTENTIAL_FIELDS_GAIN_FIX.md` + other `*_FIX.md` files — lessons from competition

---

*Last updated: 2026-06-11 — ros2-jazzy branch*
