# Computer Vision – Nodes and pipeline

Build, launch, manual node commands, and topic inputs/outputs. For “what to run at competition” see [COMPETITION.md](COMPETITION.md). For system architecture and hardware see [README.md](README.md).

---

## Build

```bash
cd ~/Repos/School/autonomy-ws-25-26/computer_vision
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
```

Optional: `cv_lidar_fusion` for CV–LiDAR fusion (`/fused_buoys`). Build with `--packages-select cv_ros_nodes cv_lidar_fusion` if needed.

---

## Launch

- **Real cameras:** `ros2 launch cv_ros_nodes launch_cv.py` (set 15 fps first from repo root: `./set_camera_fps.sh single` or `./set_camera_fps.sh three`).
- **Simulation:** `ros2 launch cv_ros_nodes launch_cv_sim.py` or `ros2 launch cv_ros_nodes launch_cv.py use_sim:=true`.

Overrides: `resolution:=1920,1200`, `camera_devices:=/path1,/path2,/path3`, `engine_path:=/path/to/model.engine`, `conf_threshold:=0.25`, `inference_interval:=2`, `enable_task4:=true`, `enable_indicator_buoy:=true`, `enable_number_detection:=true`, `number_detection_engine:=/path/to/number_detection.engine`, `number_conf_threshold:=0.25`, `staleness_threshold:=1.0`, `distance_scale_factor:=1.0`.

---

## Pipeline (high level)

```
Camera0: /camera0/image_raw → preprocessing0 → /camera0/image_preprocessed → inference0 → /camera0/detections, /camera0/detection_info
Camera1: /camera1/image_raw → preprocessing1 → ... → inference1 → /camera1/detection_info
Camera2: /camera2/image_raw → preprocessing2 → ... → inference2 → /camera2/detection_info
         │
         ├── (optional) task4_supply_processor, indicator_buoy_processor
         └── combiner → /combined/detection_info → maritime_distance_estimator → /combined/detection_info_with_distance
```

---

## Node Architecture Details

### 1. Camera Driver Nodes (v4l2_camera_node)

**Node Name**: `camera{N}_node` (N = 0, 1, 2) with namespaces `/camera{N}`

**Purpose**: Interface with V4L2-compatible USB cameras

**Instances**:
- `camera0_node` in namespace `/camera0` (video0)
- `camera1_node` in namespace `/camera1` (video2)
- `camera2_node` in namespace `/camera2` (video4)

**Note**: The executable is `v4l2_camera_node` but launch file sets node name to `camera{N}_node`

**Parameters**:
- `video_device`: `/dev/video0`, `/dev/video2`, `/dev/video4` (default; override via launch)
- `image_size`: `[1920, 1200]`
- `framerate`: v4l2_camera has no FPS parameter; set 15 fps via `v4l2-ctl --set-parm 15` before launch. See `set_camera_fps.sh`.
- `namespace`: `/camera0`, `/camera1`, `/camera2`

**Subscriptions**: None (hardware interface)

**Publications**:
- `/camera{N}/image_raw` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: 1920x1200
  - Frame rate: ~15 Hz (set via v4l2-ctl)
  - Header includes timestamp and frame_id

### 2. Preprocessing Nodes (vision_preprocessing)

**Node Name**: `preprocess_camera{N}` (N = 0, 1, 2)

**Purpose**: Image preprocessing before inference (resize, normalization, enhancement)

**Parameters** (Command Line):
- `--camera_id`: Integer (0, 1, or 2) - Required

**Subscriptions**:
- `/camera{N}/image_raw` (sensor_msgs/Image)
  - Queue size: 10
  - Callback: `image_callback()`

**Publications**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: camera resolution (e.g. 1920×1200, pass-through)
  - Frame rate: Matches input (up to 15 Hz)
  - Header: Preserved from input message

**Processing Steps**:
1. Convert ROS2 Image → OpenCV BGR format
2. Resize: 1920x1200 → camera resolution (pass-through by default)
3. (Future: glare reduction, histogram equalization, color correction)
4. Convert OpenCV → ROS2 Image
5. Publish with preserved timestamp

**Performance Characteristics**:
- Processing time: ~2-5 ms per frame
- CPU-bound operation
- No GPU usage

### 3. Inference Nodes (vision_inference)

**Node Name**: `inference_camera{N}` (N = 0, 1, 2) - Note: Internal node name is `inference_node_camera{N}` but launch file uses `inference_camera{N}`

**Purpose**: Run TensorRT-optimized YOLO inference on preprocessed images

**Dependencies**:
- TensorRT 10.3.0
- CUDA 12.6
- NVIDIA GPU (development: Think; deployment e.g. Jetson Orin)

**Parameters** (Command Line):
- `--camera_id`: Integer (0, 1, or 2) - Required
- `--engine_path`: String (default: `model_building_and_training/model.engine`) - Path to TensorRT engine file
- `--conf_threshold`: Float (default: 0.25) - Confidence threshold (0.0-1.0)
- `--enable_number_detection`: Boolean flag - Enable docking number detection
- `--number_detection_engine`: String - Path to number detection TensorRT engine
- `--number_conf_threshold`: Float (default: 0.25) - Confidence threshold for numbers
- `--inference_interval`: Integer (default: 2) - Run inference every N frames (1=every frame)

**Subscriptions**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image)
  - Queue size: 10
  - Callback: `image_callback()`

**Publications**:
- `/camera{N}/detections` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: preprocessed (camera resolution, e.g. 1920×1200)
  - Frame rate: Matches input
  - Content: Original image with bounding boxes drawn
  - Header: Preserved from input

- `/camera{N}/detection_info` (std_msgs/String)
  - Format: JSON
  - Frequency: Matches inference rate
  - Content: Detection metadata (see Message Formats section)

**Processing Pipeline**:
1. Receive preprocessed image (1920x1200 BGR)
2. Preprocess: Resize to 640x640, BGR→RGB, normalize [0,1], HWC→CHW
3. Copy to GPU memory
4. Execute TensorRT inference (async)
5. Copy results from GPU
6. Post-process: Extract boxes, apply confidence threshold, NMS
7. Draw detections on image
8. Publish detection image and JSON metadata

**Performance Characteristics**:
- Inference time: ~10-20 ms per frame (depends on model complexity)
- GPU-bound operation
- Memory: ~100-200 MB GPU memory per instance

### 4. Detection Combiner Node (vision_combiner)

**Node Name**: `detection_combiner`

**Purpose**: Aggregate detections from all cameras into unified output

**Parameters** (Command Line):
- `--staleness_threshold`: Float (default: 1.0) - Maximum age in seconds before excluding camera
- `--use_timestamp_sync`: Boolean flag - Enable timestamp-based synchronization (advanced)
- `--sync_window`: Float (default: 0.05) - Time window for timestamp matching (seconds)

**Note**: Overlap zone deduplication is not currently implemented. All detections from all cameras are preserved in the output.

**Subscriptions**:
- `/camera0/detection_info`, `/camera1/detection_info`, `/camera2/detection_info` (std_msgs/String) - Queue size: 10
- `/camera0/task4_detections`, `/camera1/task4_detections`, `/camera2/task4_detections` (std_msgs/String) - Queue size: 10
- `/camera0/indicator_detections`, `/camera1/indicator_detections`, `/camera2/indicator_detections` (std_msgs/String) - Queue size: 10

**Publications**:
- `/combined/detection_info` (std_msgs/String)
  - Format: JSON
  - Frequency: ~30 Hz (timer-based at 0.033s interval, also publishes on new data)
  - Content: Combined detections with camera source tracking; all `bbox` in **local camera frame** (preprocessed frame dimensions, e.g. 1920×1200); Task4 supply drops have `source == "task4"` and `type` in `"yellow_supply_drop"`, `"black_supply_drop"`

**Functionality**:

**a) Detection Aggregation (Latest Value Approach)**:
- Maintains the most recent detection from each camera
- Combines all active detections into single list
- Simple and fast - no complex synchronization needed
- Works well when cameras have slightly different frame rates

**b) Staleness Filtering (Automatic Failure Detection)**:
- Tracks update timestamps for each camera
- Automatically excludes cameras whose latest detection is older than threshold
- Configurable staleness threshold (default: 1.0 seconds)
- Helps detect camera failures, processing stalls, or network issues
- Camera status: `active` (fresh data), `stale` (too old), `no_data` (never received)

**c) Detection Preservation**:
- **Current behavior**: Overlap-based deduplication: detections in the 20% overlap strip of adjacent cameras (same class_id, similar bbox size) are merged into one detection with `duplicate: true`, both bboxes, and averaged bearing/elevation; all other detections are preserved
- All detections from active cameras are included in the output
- This ensures maximum detection coverage and redundancy

**d) Health Monitoring**:
- Tracks camera status: `active`, `no_data`, `stale`
- Reports FPS and detection counts per camera
- Output includes counts: `num_active_cameras`, `num_stale_cameras`, `num_no_data_cameras`
- Logs warnings when cameras become stale

**e) Temporal Smoothing**:
- Timer-based publishing ensures output even if cameras are slow
- Prevents gaps in detection stream
- Publishes at ~30 Hz (0.033s timer interval) regardless of camera update rates

**Performance Characteristics**:
- Processing time: <1 ms per update
- CPU-bound operation
- Minimal memory footprint

### 5. Maritime Distance Estimator Node (maritime_distance_estimator)

**Node Name**: `maritime_distance_estimator`

**Purpose**: Add bearing, elevation, and distance estimates to detections

**Parameters**:
- `distance_scale_factor`: Float (default: 1.0) - One-point calibration scale factor

**Subscriptions**:
- `/combined/detection_info` (std_msgs/String)

**Publications**:
- `/combined/detection_info_with_distance` (std_msgs/String)
  - Format: JSON
  - Frequency: ~30 Hz (matches combiner output rate)
  - Content: Same as `/combined/detection_info` with added `bearing_deg`, `elevation_deg`, `distance_m`, `distance_ft` per detection

**Functionality**:
- Computes bearing and elevation from bounding box center using camera intrinsics and mounting angles
- Estimates distance using pinhole camera model with reference object sizes
- Applies distance scale factor calibration
- Outputs final CV detection format for navigation and fusion

### 6. Task4 Supply Processor Node (task4_supply_processor)

**Node Name**: `task4_supply_processor`

**Purpose**: Detect yellow and black supply-drop vessels directly from each camera frame using a simplified CV pipeline (color blob detection + shape verification). No YOLO shape detection dependency.

**Subscriptions**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image) – BGR frame for full detection pipeline

**Publications**:
- `/camera{N}/task4_detections` (std_msgs/String) – JSON: `{ "camera_id": N, "timestamp": <float>, "detections": [ { "type": "yellow_supply_drop"|"black_supply_drop", "vessel_bbox": [x1,y1,x2,y2], "confidence": <float>, "has_shape": <bool>, "source": "task4" } ] }`; `vessel_bbox` in **preprocessed frame** (camera resolution)

**Functionality**:
- **Simplified pipeline** (`task4_simplified_detector.py`): runs entirely from the raw image without relying on YOLO class 6/8 detections
- **Yellow blob detection** via HSV masking; **black blob detection** via combined HSV+LAB masking
- **Spatial filtering**: only blobs in the lower 60% of the frame are considered
- **Aspect ratio filtering**: blobs must be noticeably wider than tall (height/width < 0.7)
- **Blob merging**: nearby blobs of the same colour are merged into one bounding box
- **Overlap resolution**: when bounding boxes overlap (IoU > 0.3) the larger one is kept
- **Shape verification**: searches the region above each vessel for a black triangle (yellow boat) or black cross (black boat); confirmed shapes boost confidence
- **Confidence threshold**: only detections with `confidence >= 0.65` are published

### 7. Indicator Buoy Processor Node (indicator_buoy_processor)

**Node Name**: `indicator_buoy_processor`

**Purpose**: Detect color indicator buoys (red/green) for Tasks 2, 3, and 5 using CV-only diamond-based detection

**Subscriptions**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image)

**Publications**:
- `/camera{N}/indicator_detections` (std_msgs/String) – JSON format with `class_name` (`red_indicator_buoy` or `green_indicator_buoy`)

**Functionality**: 
- **CV-only pipeline** (not YOLO-based): Detects black diamond markers on white buoy bodies
- **Multi-diamond grouping**: Handles angled buoy views by grouping nearby diamonds (within 2× diamond size)
- **Collective centering**: Uses average position of grouped diamonds for indicator ROI positioning
- **Color classification**: Detects and classifies red/green indicator on top of buoy
- **Width-based distance**: Optimized for 20-inch buoy width reference (handled by maritime_distance_estimator)
- **Self-contained**: All detection logic embedded directly in node (no external task_specific imports)

**Parameters**:
- `--conf_threshold` (default: 0.6): Diamond shape confidence threshold
- `--max_black_brightness` (default: 230): Max brightness for black diamonds (handles glare)
- `--buoy_conf_threshold` (default: 0.3): Combined diamond + white blob confidence
- `--white_blob_expansion` (default: 2.0): White blob search region multiplier
- `--min_white_brightness` (default: 100): Minimum white brightness for outdoor scenes
- `--min_white_blob_score` (default: 0.15): Minimum white blob score

**Documentation**: See `task_specific/task_2_3/COLOUR_INDICATOR_BUOY.md` for complete pipeline details

---

## Manual node commands

When not using the launch file. Start cameras first, then preprocessing, then inference, then combiner.

**Camera nodes (v4l2_camera):**
```bash
# Camera 0 (left)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera0

# Camera 1 (center)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.3:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera1

# Camera 2 (right)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.4:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera2
```

**Note**: Device paths are for the Think development machine. On other hardware, use `v4l2-ctl --list-devices` to find your camera paths.

**CV nodes:**
- Preprocessing: `ros2 run cv_ros_nodes vision_preprocessing --camera_id 0` (and 1, 2)
- Inference: `ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/Repos/School/autonomy-ws-25-26/computer_vision/model_building_and_training/model.engine` (and 1, 2)
- Combiner: `ros2 run cv_ros_nodes vision_combiner`
- Distance estimator: `ros2 run cv_ros_nodes maritime_distance_estimator`
- Task4: `ros2 run cv_ros_nodes task4_supply_processor`
- Indicator buoy: `ros2 run cv_ros_nodes indicator_buoy_processor`
- Camera viewer (save images): `ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images`

---

## Complete Topic List

| Topic Name | Message Type | Publisher Node | Subscriber Nodes | Frequency | Purpose |
|------------|--------------|----------------|------------------|-----------|---------|
| `/camera0/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera0) | preprocess_camera0 | ~15 Hz | Raw camera feed 0 |
| `/camera1/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera1) | preprocess_camera1 | ~15 Hz | Raw camera feed 1 |
| `/camera2/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera2) | preprocess_camera2 | ~15 Hz | Raw camera feed 2 |
| `/camera0/image_preprocessed` | sensor_msgs/Image | preprocess_camera0 | inference_camera0 | ~15 Hz | Preprocessed images 0 |
| `/camera1/image_preprocessed` | sensor_msgs/Image | preprocess_camera1 | inference_camera1 | ~15 Hz | Preprocessed images 1 |
| `/camera2/image_preprocessed` | sensor_msgs/Image | preprocess_camera2 | inference_camera2 | ~15 Hz | Preprocessed images 2 |
| `/camera0/detections` | sensor_msgs/Image | inference_camera0 | (monitoring/visualization) | ~15 Hz | Detection visualization 0 |
| `/camera1/detections` | sensor_msgs/Image | inference_camera1 | (monitoring/visualization) | ~15 Hz | Detection visualization 1 |
| `/camera2/detections` | sensor_msgs/Image | inference_camera2 | (monitoring/visualization) | ~15 Hz | Detection visualization 2 |
| `/camera0/detection_info` | std_msgs/String | inference_camera0 | detection_combiner | ~15 Hz | Detection metadata 0 |
| `/camera1/detection_info` | std_msgs/String | inference_camera1 | detection_combiner | ~15 Hz | Detection metadata 1 |
| `/camera2/detection_info` | std_msgs/String | inference_camera2 | detection_combiner | ~15 Hz | Detection metadata 2 |
| `/camera0/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on image_preprocessed | Task4 supply drops, camera 0 |
| `/camera1/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on image_preprocessed | Task4 supply drops, camera 1 |
| `/camera2/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on image_preprocessed | Task4 supply drops, camera 2 |
| `/camera0/indicator_detections` | std_msgs/String | indicator_buoy_processor | detection_combiner | on image_preprocessed | Indicator buoy detections, camera 0 |
| `/camera1/indicator_detections` | std_msgs/String | indicator_buoy_processor | detection_combiner | on image_preprocessed | Indicator buoy detections, camera 1 |
| `/camera2/indicator_detections` | std_msgs/String | indicator_buoy_processor | detection_combiner | on image_preprocessed | Indicator buoy detections, camera 2 |
| `/combined/detection_info` | std_msgs/String | detection_combiner | maritime_distance_estimator | ~30 Hz | Unified detections; bbox in local camera frame |
| `/combined/detection_info_with_distance` | std_msgs/String | maritime_distance_estimator | (downstream nodes) | ~30 Hz | Final CV output with bearing, elevation, distance |

### Topic Dependency Graph

```
/camera0/image_raw
    ↓
/camera0/image_preprocessed
    ↓
/camera0/detections ──┐
/camera0/detection_info ──┐
                          │
/camera1/image_raw         │
    ↓                      │
/camera1/image_preprocessed│
    ↓                      │
/camera1/detections ──┐    │
/camera1/detection_info ──┤
                          ├──→ /combined/detection_info
/camera2/image_raw         │
    ↓                      │
/camera2/image_preprocessed│
    ↓                      │
/camera2/detections ──┐    │
/camera2/detection_info ───┘
                          │
                          ↓
              /combined/detection_info_with_distance
```

## Topic inputs and outputs (per node)

| Node | Subscribes | Publishes |
|------|------------|-----------|
| v4l2_camera (per camera) | — | `/camera{N}/image_raw` |
| vision_preprocessing | `/camera{N}/image_raw` | `/camera{N}/image_preprocessed` |
| vision_inference | `/camera{N}/image_preprocessed` | `/camera{N}/detections`, `/camera{N}/detection_info` |
| task4_supply_processor | `/camera{N}/image_preprocessed` | `/camera{N}/task4_detections` |
| indicator_buoy_processor | `/camera{N}/image_preprocessed` | `/camera{N}/indicator_detections` |
| vision_combiner | `/camera{N}/detection_info`, (optional) task4, indicator | `/combined/detection_info` |
| maritime_distance_estimator | `/combined/detection_info` | `/combined/detection_info_with_distance` |

---

## Final CV output (downstream)

| Topic | Type | Description |
|-------|------|-------------|
| **`/combined/detection_info_with_distance`** | `std_msgs/String` (JSON) | **Final CV output**: combined detections with `camera_id`, `class_name`, `score`, `bbox`, `bearing_deg`, `elevation_deg`, `distance_m`, `distance_ft`, `reference_height_m`. Use for navigation and CV–LiDAR fusion. ~30 Hz. |
| `/combined/detection_info` | `std_msgs/String` (JSON) | Same structure without distance (intermediate; distance estimator publishes the final topic above). |

### Final output JSON structure

**`/combined/detection_info_with_distance`** example:

```json
{
  "timestamp": 1234567890.123456,
  "num_cameras": 3,
  "num_active_cameras": 3,
  "num_stale_cameras": 0,
  "num_no_data_cameras": 0,
  "staleness_threshold": 1.0,
  "total_detections": 2,
  "camera_stats": {
    "0": {
      "status": "active",
      "num_detections": 1,
      "fps": 30.5,
      "timestamp": 1234567890.123,
      "time_since_update": 0.023
    },
    "1": {
      "status": "active",
      "num_detections": 1,
      "fps": 31.2,
      "timestamp": 1234567890.124,
      "time_since_update": 0.031
    },
    "2": {
      "status": "active",
      "num_detections": 0,
      "fps": 30.8,
      "timestamp": 1234567890.125,
      "time_since_update": 0.015
    }
  },
  "detections": [
    {
      "camera_id": 1,
      "class_id": 3,
      "class_name": "red_buoy",
      "score": 0.85,
      "bbox": [150, 150, 250, 250],
      "bearing_deg": -15.2,
      "elevation_deg": 2.5,
      "distance_m": 8.5,
      "distance_ft": 27.9,
      "reference_height_m": 0.254
    },
    {
      "camera_id": 0,
      "class_id": 1,
      "class_name": "green_buoy",
      "score": 0.78,
      "bbox": [500, 200, 600, 300],
      "bearing_deg": -45.8,
      "elevation_deg": -1.2,
      "distance_m": 12.3,
      "distance_ft": 40.4,
      "reference_height_m": 0.254
    }
  ],
  "synchronized": null,
  "camera_info": {
    "frame_width": 1920,
    "frame_height": 1200,
    "horizontal_fov_deg": 85.0,
    "vertical_fov_deg": 69.0,
    "mounting_angles_deg": [-70.0, 0.0, 70.0],
    "fx": 1186.67,
    "fy": 1186.67,
    "cx": 960.0,
    "cy": 600.0
  },
  "distance_estimation": {
    "method": "specs_based",
    "camera_model": "AR0234",
    "focal_length_mm": 3.56,
    "pixel_size_um": 3.0,
    "native_resolution": "1920×1200",
    "current_resolution": "1920×1200",
    "effective_fy_px": 1186.7,
    "num_object_references": 15,
    "distance_scale_factor": 1.0
  }
}
```

### Angle fields explained

**`bearing_deg`** (float, degrees):
- **Boat-relative bearing angle** from the forward direction
- **0°** = straight ahead (boat forward direction)
- **Positive values** = right of center (starboard)
- **Negative values** = left of center (port)
- Range: approximately -85° to +85° (depends on camera FOV and mounting)
- **Computation**: Camera mounting angle + pixel-to-angle conversion
  - Camera 0 (left): mounting angle -70°
  - Camera 1 (center): mounting angle 0°
  - Camera 2 (right): mounting angle +70°
- **Use**: Navigation planning, CV–LiDAR fusion (match by bearing), gate alignment

**`elevation_deg`** (float, degrees):
- **Vertical angle** relative to the horizon
- **0°** = horizon level
- **Positive values** = above horizon (up)
- **Negative values** = below horizon (down)
- Range: approximately -30° to +30° (depends on camera FOV)
- **Computation**: Vertical pixel position converted to angle using camera intrinsics
- **Use**: Object height estimation, approach angle calculation

**Angle conventions:**
- **Bearing**: Measured in the horizontal plane (X–Y plane in boat frame)
- **Elevation**: Measured in the vertical plane (perpendicular to bearing)
- Both angles are computed from the bounding box center pixel using camera intrinsics and mounting geometry

**Example interpretation:**
- `bearing_deg: -15.2` → Object is 15.2° to the left (port side) of forward
- `elevation_deg: 2.5` → Object is 2.5° above the horizon
- `bearing_deg: 45.8` → Object is 45.8° to the right (starboard side) of forward
- `elevation_deg: -1.2` → Object is 1.2° below the horizon

**Distance fields:**
- **`distance_m`**: Distance in meters (estimated using pinhole camera model with reference object sizes)
- **`distance_ft`**: Distance in feet (converted from meters)

---

## Message Formats

### 1. sensor_msgs/Image (Raw and Preprocessed)

**Standard ROS2 Image Message**:
- `header`: Standard ROS2 header (stamp, frame_id)
- `height`: Image height in pixels
- `width`: Image width in pixels
- `encoding`: `bgr8` (for all images)
- `is_bigendian`: Boolean
- `step`: Row step in bytes
- `data`: Raw pixel data (uint8 array)

**Example**:
- Raw: 1920x1200, ~7.2 MB per message
- Preprocessed: 1920x1200, ~7.2 MB per message (pass-through)

### 2. std_msgs/String (Detection Info)

**Individual Camera Detection Info** (`/camera{N}/detection_info`):

```json
{
  "camera_id": 0,
  "timestamp": 1234567890.123456,
  "frame_id": "camera0",
  "num_detections": 3,
  "inference_time_ms": 15.2,
  "fps": 32.5,
  "output_fps": 30.5,
  "frame_width": 1920,
  "frame_height": 1200,
  "detections": [
    {
      "class_id": 2,
      "score": 0.85,
      "x1": 100,
      "y1": 150,
      "x2": 200,
      "y2": 250,
      "width": 100.0,
      "height": 100.0,
      "bbox": [100, 150, 200, 250]
    },
    {
      "class_id": 5,
      "score": 0.72,
      "x1": 300,
      "y1": 400,
      "x2": 350,
      "y2": 450,
      "width": 50.0,
      "height": 50.0,
      "bbox": [300, 400, 350, 450]
    },
    {
      "class_id": 2,
      "score": 0.68,
      "x1": 500,
      "y1": 200,
      "x2": 600,
      "y2": 300,
      "width": 100.0,
      "height": 100.0,
      "bbox": [500, 200, 600, 300]
    }
  ]
}
```

**Field Descriptions**:
- `camera_id`: Source camera identifier (0, 1, or 2)
- `timestamp`: ROS2 timestamp in seconds (float)
- `frame_id`: Frame identifier string
- `num_detections`: Number of detected objects
- `inference_time_ms`: Inference latency in milliseconds
- `fps`: Current processing rate (frames per second)
- `frame_width`, `frame_height`: Preprocessed frame dimensions
- `detections`: Array of detection objects
  - `class_id`: Object class identifier (integer, see Class ID Mapping)
  - `score`: Confidence score (0.0-1.0)
  - `bbox`: Bounding box `[x1, y1, x2, y2]` in **preprocessed frame** (frame_width×frame_height)

**Combined Detection Info** (`/combined/detection_info`):

All `detections[].bbox` are in the **local camera frame** (preprocessed frame dimensions, e.g. 1920×1200 per camera). Task4 supply drops: `source == "task4"`, `type` in `"yellow_supply_drop"`, `"black_supply_drop"`. Detections that were merged from the overlap of two adjacent cameras include optional `duplicate: true`, `bbox_other` (the other camera’s bbox), and `camera_id_other`; in that case `bearing_deg` and `elevation_deg` are the **average** of the two cameras’ angles.

```json
{
  "timestamp": 1234567890.123456,
  "num_cameras": 3,
  "num_active_cameras": 2,
  "num_stale_cameras": 1,
  "num_no_data_cameras": 0,
  "staleness_threshold": 1.0,
  "total_detections": 5,
  "camera_stats": {
    "0": {
      "status": "active",
      "num_detections": 3,
      "fps": 30.5,
      "timestamp": 1234567890.123,
      "time_since_update": 0.023
    },
    "1": {
      "status": "active",
      "num_detections": 2,
      "fps": 31.8,
      "timestamp": 1234567890.124,
      "time_since_update": 0.031
    },
    "2": {
      "status": "stale",
      "num_detections": 0,
      "time_since_update": 1.234,
      "staleness_threshold": 1.0,
      "fps": 0.0,
      "last_timestamp": 1234567888.9
    }
  },
  "detections": [
    {
      "camera_id": 0,
      "class_id": 2,
      "class_name": "green_pole_buoy",
      "score": 0.85,
      "bbox": [100, 150, 200, 250],
      "bearing_deg": -15.2,
      "elevation_deg": 2.5
    },
    {
      "camera_id": 1,
      "class_id": 5,
      "class_name": "yellow_buoy",
      "score": 0.78,
      "bbox": [300, 400, 350, 450],
      "bearing_deg": 5.3,
      "elevation_deg": -1.2
    },
    {
      "camera_id": 0,
      "class_id": 11,
      "class_name": "yellow_supply_drop",
      "score": 0.82,
      "bbox": [500, 200, 600, 300],
      "bearing_deg": -25.8,
      "elevation_deg": 1.5,
      "source": "task4",
      "type": "yellow_supply_drop"
    }
  ],
  "synchronized": null,
  "camera_info": {
    "frame_width": 1920,
    "frame_height": 1200,
    "horizontal_fov_deg": 85.0,
    "vertical_fov_deg": 69.0,
    "mounting_angles_deg": [-70.0, 0.0, 70.0],
    "fx": 1186.67,
    "fy": 1186.67,
    "cx": 960.0,
    "cy": 600.0
  }
}
```

**Field Descriptions**:
- `timestamp`: Current system timestamp
- `num_cameras`: Total number of cameras (always 3)
- `num_active_cameras`: Number of cameras with fresh data (included in output)
- `num_stale_cameras`: Number of cameras with stale data (excluded from output)
- `num_no_data_cameras`: Number of cameras that never sent data
- `staleness_threshold`: Configured threshold for staleness detection (seconds)
- `total_detections`: Total number of detections from active cameras only
- `camera_stats`: Per-camera health and statistics (keys are strings: "0", "1", "2")
  - `status`: `"active"` (fresh data), `"no_data"` (never received), or `"stale"` (too old)
  - `num_detections`: Detections from this camera (0 if stale/no_data)
  - `fps`: Camera processing rate (uses `output_fps` from detection_info if available)
  - `timestamp`: Last update timestamp (if active)
  - `time_since_update`: Seconds since last update
  - `staleness_threshold`: Threshold used (if stale)
  - `last_timestamp`: Last detection timestamp (if stale)
- `detections`: Combined array with `camera_id`, `class_id`, `class_name`, `score`, `bbox`, `bearing_deg`, `elevation_deg`; Task4 entries include `source: "task4"` and `type` (`"yellow_supply_drop"` or `"black_supply_drop"`). Merged overlap duplicates include `duplicate: true`, `bbox_other`, `camera_id_other`, and averaged `bearing_deg`/`elevation_deg`. Example merged detection: `{ "camera_id": 0, "camera_id_other": 1, "class_id": 3, "class_name": "red_buoy", "score": 0.9, "bbox": [1500, 100, 1650, 250], "bbox_other": [100, 100, 250, 250], "bearing_deg": -35.1, "elevation_deg": 0.5, "duplicate": true }`
  - **Note**: Only includes detections from `active` cameras (stale cameras excluded)
  - **Bbox frame**: `bbox` is in **local camera frame** (preprocessed frame dimensions, e.g. 1920×1200), NOT global frame
  - **Trimmed fields**: `x1`, `y1`, `x2`, `y2`, `width`, `height`, `bearing_rad`, `elevation_rad` are removed from output
- `camera_info`: Camera intrinsics and configuration
  - `frame_width`, `frame_height`: Preprocessed frame dimensions (e.g. 1920, 1200)
  - `horizontal_fov_deg`, `vertical_fov_deg`: Field of view per camera (85°, 69°)
  - `mounting_angles_deg`: Camera mounting angles [-70°, 0°, 70°] for [left, center, right]
  - `fx`, `fy`, `cx`, `cy`: Camera intrinsics (focal length and principal point in pixels)
- `synchronized`: `true` if timestamp sync used, `null` if latest value approach

---

## Monitoring and Debugging

### Basic topic monitoring

```bash
# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /camera0/camera_info --window 20
ros2 topic hz /combined/detection_info --window 20
ros2 topic hz /combined/detection_info_with_distance --window 20

# Check topic type
ros2 topic type /combined/detection_info_with_distance

# See topic info
ros2 topic info /combined/detection_info_with_distance
```

### Viewing topic messages

**Basic echo:**
```bash
ros2 topic echo /combined/detection_info_with_distance
```

**View long messages (no truncation):**
```bash
# Use --no-arr to see full arrays/lists without truncation
ros2 topic echo /combined/detection_info_with_distance --no-arr

# For JSON topics, extract and pretty-print with jq
ros2 topic echo /combined/detection_info_with_distance --once --field data | jq .

# View full message continuously without truncation
ros2 topic echo /combined/detection_info_with_distance --no-arr --field data | jq .
```

**Single message:**
```bash
# Get one message and exit
ros2 topic echo /combined/detection_info_with_distance --once

# Single message, specific field, pretty-printed
ros2 topic echo /combined/detection_info_with_distance --once --field data | jq .
```

**Per-camera topics:**
```bash
# View detections from specific camera
ros2 topic echo /camera0/detection_info --no-arr --field data | jq .
ros2 topic echo /camera1/detection_info --no-arr --field data | jq .
ros2 topic echo /camera2/detection_info --no-arr --field data | jq .

# Check detection frequency per camera
ros2 topic hz /camera0/detection_info --window 20
ros2 topic hz /camera1/detection_info --window 20
ros2 topic hz /camera2/detection_info --window 20
```

### Debugging specific nodes

**Check if nodes are running:**
```bash
ros2 node list
ros2 node info /inference_camera0
ros2 node info /detection_combiner
ros2 node info /maritime_distance_estimator
```

**View node parameters:**
```bash
ros2 param list /inference_camera0
ros2 param list /maritime_distance_estimator
ros2 param get /maritime_distance_estimator distance_scale_factor
```

**Monitor node output/logs:**
```bash
# Check if nodes are publishing
ros2 topic info /camera0/detection_info --verbose

# View image topics (use rqt_image_view or rviz)
ros2 run rqt_image_view rqt_image_view
```

### Common debugging commands

```bash
# Check all CV-related topics
ros2 topic list | grep -E "(camera|detection|combined)"

# Monitor pipeline health
ros2 topic hz /camera0/image_raw --window 20
ros2 topic hz /camera0/image_preprocessed --window 20
ros2 topic hz /camera0/detection_info --window 20
ros2 topic hz /combined/detection_info --window 20
ros2 topic hz /combined/detection_info_with_distance --window 20

# Save topic messages to bag file for later analysis
ros2 bag record /combined/detection_info_with_distance /camera0/detection_info /camera1/detection_info /camera2/detection_info

# Play back bag file
ros2 bag play <bag_file_name>
```

### Troubleshooting

- **"Camera health: 0 active, 0 stale, 3 no data"** (combiner never sees any camera):
  - **DDS discovery:** Ensure all nodes use the same DDS config. Set before launching: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `export ROS_DOMAIN_ID=42` (or your value), `export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml` if you use CycloneDDS.
  - **Stale daemon:** Restart the ROS2 daemon so topic/node discovery is fresh: `ros2 daemon stop && sleep 1 && ros2 daemon start`. Then start the CV launch again. Using `comp.sh` does this automatically.
  - **Single camera test:** Run one camera + preprocessing + inference in the same shell (with the same env), then `ros2 topic hz /camera0/detection_info` to confirm data flow before running the full launch.
- **USB ports / cameras or LiDAR not found after moving cables:**
  - Launch defaults: **Cameras** = by-path `1.4.2`, `1.4.3`, `1.4.4` (Jetson, three Arducams e.g. on a hub). **LiDAR** = by-path `2.1` (Unitree LiDAR plugged directly into Jetson). If you moved the LiDAR from hub to Jetson, the LiDAR port is now `2.1` (default updated in buoy_pipeline).
  - From repo root run: `./monitor_camera_devices.sh` to see current by-path devices. Edit device paths in `set_camera_fps.sh` (root), or override: `CAMERA_DEVICES="/path1,/path2,/path3" ./set_camera_fps.sh three` then run the comp script.
- **No messages on topic:** Check if nodes are running (`ros2 node list`), check topic exists (`ros2 topic list`), verify camera devices are correct
- **Low frequency:** Check camera FPS (from repo root: `./set_camera_fps.sh single` or `three`), check inference engine path, verify preprocessing is completing
- **Truncated output:** Use `--no-arr` flag with `ros2 topic echo` to see full messages
- **JSON parsing:** Use `jq .` to pretty-print JSON messages from String topics

---

## Failure Modes and Mitigation

### Camera Failures

#### Failure Mode: Camera Disconnection

**Symptoms**:
- No messages on `/camera{N}/image_raw`
- Camera node logs show "Device not found"
- Preprocessing node reports no data

**Detection**:
- Monitor topic frequency: `ros2 topic hz /camera{N}/image_raw`
- Check camera node logs
- Combiner node reports `status: "no_data"` for affected camera

**Mitigation**:
- **Automatic**: Preprocessing node continues (no crash)
- **Automatic**: Inference node waits for data (no crash)
- **Automatic**: Combiner node excludes stale camera from output
- **Manual**: Restart camera node or reconnect hardware

**Recovery**:
- Reconnect camera hardware
- Restart camera node: `ros2 run v4l2_camera v4l2_camera_node ...`
- System automatically resumes processing

#### Failure Mode: Camera Frame Rate Degradation

**Symptoms**:
- Topic frequency drops below 15 Hz
- Increased latency in detection pipeline

**Detection**:
- Monitor: `ros2 topic hz /camera{N}/image_raw`
- Alert if frequency < 15 Hz for >5 seconds

**Mitigation**:
- Check USB bandwidth (use USB 3.0 ports)
- Reduce camera resolution if necessary
- Check for USB hub issues

### GPU Failures

#### Failure Mode: GPU Out of Memory

**Symptoms**:
- Inference node crashes with CUDA error
- Error: "out of memory" in logs
- GPU memory usage at 100%

**Detection**:
- Monitor GPU memory: `nvidia-smi`
- Check inference node logs for CUDA errors

**Mitigation**:
- **Prevention**: Limit concurrent inference instances
- **Prevention**: Use FP16 precision to reduce memory
- **Recovery**: Restart inference node(s)
- **Recovery**: Reduce batch size or model complexity

**Recovery**:
- Kill and restart affected inference nodes
- System continues with remaining cameras
- Gradually restart failed nodes

#### Failure Mode: GPU Overheating

**Symptoms**:
- GPU temperature >85°C
- Thermal throttling (reduced performance)
- Inference latency increases

**Detection**:
- Monitor: `nvidia-smi -q -d TEMPERATURE`
- Alert if temperature >80°C

**Mitigation**:
- **Immediate**: Reduce processing load
- **Immediate**: Enable GPU power limits
- **Long-term**: Improve cooling, check thermal paste

### Node Failures

#### Failure Mode: Preprocessing Node Crash

**Symptoms**:
- No messages on `/camera{N}/image_preprocessed`
- Node process not running
- Inference node reports no data

**Detection**:
- Monitor topic: `ros2 topic hz /camera{N}/image_preprocessed`
- Check node status: `ros2 node list`

**Mitigation**:
- **Automatic**: Process manager (systemd) restarts node
- **Automatic**: Inference node waits gracefully
- **Manual**: Restart node manually

**Recovery**:
- Process manager auto-restart (if configured)
- Manual restart: `ros2 run cv_ros_nodes vision_preprocessing --camera_id N`

#### Failure Mode: Inference Node Crash

**Symptoms**:
- No messages on `/camera{N}/detections` or `/camera{N}/detection_info`
- Node process not running
- GPU memory not released

**Detection**:
- Monitor topics for missing data
- Check node status
- Monitor GPU memory (should decrease)

**Mitigation**:
- **Automatic**: Process manager restarts node
- **Automatic**: GPU memory cleanup on restart
- **Automatic**: Combiner node handles missing camera gracefully

**Recovery**:
- Process manager auto-restart
- Manual restart: `ros2 run cv_ros_nodes vision_inference --camera_id N --engine_path model_building_and_training/model.engine`
- Verify GPU memory released: `nvidia-smi`

#### Failure Mode: Combiner Node Crash

**Symptoms**:
- No messages on `/combined/detection_info`
- Downstream nodes receive no data

**Detection**:
- Monitor topic: `ros2 topic hz /combined/detection_info`
- Check node status

**Mitigation**:
- **Critical**: This is a single point of failure
- **Prevention**: Robust error handling in combiner
- **Recovery**: Fast restart (<5 seconds)

**Recovery**:
- Process manager auto-restart
- Manual restart: `ros2 run cv_ros_nodes vision_combiner`
- System resumes immediately (no data loss from cameras)

### Network/ROS2 Failures

#### Failure Mode: DDS Discovery Issues

**Symptoms**:
- Nodes cannot see each other's topics
- "No subscribers" warnings
- Topics exist but no data flow

**Detection**:
- Check: `ros2 topic list` (should show all topics)
- Check: `ros2 node list` (should show all nodes)
- Check DDS configuration

**Mitigation**:
- **Prevention**: Use same DDS implementation (FastDDS or CycloneDDS)
- **Prevention**: Configure DDS domain ID consistently
- **Recovery**: Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

#### Failure Mode: Topic Buffer Overflow

**Symptoms**:
- Dropped messages in logs
- Stale data in topics
- Increased latency

**Detection**:
- Monitor topic queue sizes
- Check for "dropped message" warnings

**Mitigation**:
- **Prevention**: Increase queue sizes (currently 10)
- **Prevention**: Ensure subscribers process fast enough
- **Recovery**: Restart slow nodes

### Model/Inference Failures

#### Failure Mode: Invalid TensorRT Engine

**Symptoms**:
- Inference node fails to start
- Error: "Failed to load engine"
- Engine file corrupted or incompatible

**Detection**:
- Check node startup logs
- Verify engine file exists and is readable

**Mitigation**:
- **Prevention**: Validate engine before deployment
- **Prevention**: Version control for engine files
- **Recovery**: Rebuild engine from ONNX
- **Recovery**: Use backup engine version

#### Failure Mode: Inference Accuracy Degradation

**Symptoms**:
- Increased false positives/negatives
- Detection quality decreases
- Model performance below targets

**Detection**:
- Monitor detection confidence scores
- Compare with validation metrics
- User feedback

**Mitigation**:
- **Prevention**: Regular model validation
- **Prevention**: Monitor detection statistics
- **Recovery**: Retrain model with updated dataset
- **Recovery**: Deploy new engine version

### System-Level Failures

#### Failure Mode: System Reboot

**Symptoms**:
- All nodes stop
- System restarts

**Mitigation**:
- **Automatic**: Process manager (systemd) auto-starts nodes on boot
- **Automatic**: Launch file configured for auto-start
- **Recovery**: System resumes within 30-60 seconds

#### Failure Mode: Disk Space Exhaustion

**Symptoms**:
- Log files fill disk
- Nodes cannot write logs
- System becomes unresponsive

**Detection**:
- Monitor disk usage: `df -h`
- Alert if usage >80%

**Mitigation**:
- **Prevention**: Log rotation configured
- **Prevention**: Limit log file sizes
- **Recovery**: Clean old logs, increase disk space

### Failure Mode Summary Table

| Failure Mode | Detection Method | Automatic Mitigation | Manual Recovery | Impact Severity |
|--------------|------------------|---------------------|-----------------|-----------------|
| Camera disconnect | Topic monitoring | Graceful degradation | Reconnect hardware | Medium |
| GPU OOM | nvidia-smi, logs | Node restart | Reduce load | High |
| Node crash | Process monitoring | Auto-restart | Manual restart | Medium |
| Combiner crash | Topic monitoring | Auto-restart | Manual restart | High |
| DDS failure | Topic/node list | Restart daemon | Fix config | High |
| Invalid engine | Startup logs | Use backup | Rebuild engine | High |
| System reboot | System logs | Auto-start on boot | Wait for recovery | Low |

---

## Performance Targets

### Latency Targets

| Stage | Target | Measurement Method |
|-------|--------|-------------------|
| Camera capture | <66.7 ms (15 FPS) | Hardware dependent |
| Preprocessing | <5 ms | Node internal timing |
| Inference | <20 ms | TensorRT timing |
| Combination | <1 ms | Node internal timing |
| Distance estimation | <1 ms | Node internal timing |
| **End-to-end** | **<50 ms** | Topic timestamp diff |

### Throughput Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Per-camera FPS | ≥15 FPS | Sustained processing rate |
| Combined output rate | ≥15 Hz | Combiner node output |
| GPU utilization | 60-80% | Optimal range |
| CPU utilization | <80% | Per core |

### Resource Targets

| Resource | Target | Measurement |
|----------|--------|-------------|
| GPU memory | <2 GB | Total across all nodes |
| CPU memory | <1 GB | Per node |
| Network bandwidth | <100 MB/s | Total ROS2 traffic |

### Accuracy Targets

| Metric | Target | Notes |
|--------|--------|-------|
| mAP@0.5 | >0.75 | Mean Average Precision |
| Precision | >0.80 | False positive rate |
| Recall | >0.70 | False negative rate |
| FPS (inference) | >15 | Real-time requirement |

### Reliability Targets

| Metric | Target |
|--------|--------|
| Uptime | >99% |
| Mean Time Between Failures (MTBF) | >24 hours |
| Mean Time To Recovery (MTTR) | <5 minutes |
