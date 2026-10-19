# Computer Vision – Competition and testing

What to run at competition, when to use which launch, and how to iterate (calibration, overrides, tuning). For pipeline and node details see [NODES.md](NODES.md). For system architecture and hardware see [README.md](README.md).

---

## Quick start (one command)

```bash
cd ~/Repos/School/autonomy-ws-25-26/computer_vision
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
./set_camera_fps.sh single
ros2 launch cv_ros_nodes launch_cv.py
```

Set 15 fps on cameras before launch (from repo root: `./set_camera_fps.sh single` or `./set_camera_fps.sh three`; device paths are in that script).

---

## Task-specific launch (RoboBoat 2026)

| Task | Launch command |
|------|----------------|
| **Tasks 1–3** (Evacuation Route, Debris Clearance, Emergency Sprint) | `ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=true enable_task4:=false enable_number_detection:=false` |
| **Task 4** (Supply Drop) | `ros2 launch cv_ros_nodes launch_cv.py enable_task4:=true enable_indicator_buoy:=false enable_number_detection:=false` |
| **Task 5** (Navigate the Marina) | `ros2 launch cv_ros_nodes launch_cv.py enable_number_detection:=true enable_indicator_buoy:=false enable_task4:=false` |
| **Task 6** (Harbor Alert) | Use base pipeline or same as Task 3/5; CV supports navigation (yellow buoy, dock). |

For Task 5, ensure the number detection TensorRT engine exists (e.g. build from `task_specific/Docking/number_detection/` with `build_number_engine.sh`). Override if needed: `number_detection_engine:=/path/to/number_detection.engine`.

---

## Competition settings: detections and accuracy

The robot uses **`/combined/detection_info`** (and `..._with_distance`) when inference runs and publishes. More inference runs = more detection updates.

| Goal | Setting | Why |
|------|--------|-----|
| **Most detections, best accuracy** | **`inference_interval:=1`** | Run inference on every frame. Max detections/sec and lowest latency. Default is 2 (every other frame). |
| **Faster inference** | **FP16 TensorRT engine** | Build with `--fp16` (~2× faster). See [model_building_and_training/TENSORRT.md](model_building_and_training/TENSORRT.md). |
| **Confidence** | **`conf_threshold:=0.25`** (default) | Use **`0.2`** only if you need more marginal detections and accept more false positives. |
| **Real cameras at competition** | **`ros2 launch cv_ros_nodes launch_cv.py`** (+ task flags) | Default inference_interval is 2 (every other frame). Set `inference_interval:=1` for every frame. Do not use sim launch for real cameras. |
| **Sim / testing** | **`launch_cv_sim.py`** with **`inference_interval:=1`** | Matches competition. Use `inference_interval:=2` only for smoother display when debugging. |
| **Jetson GPU memory** | **`single_camera:=true`** (sim only) | If OOM with sim + CV, run one camera. On real hardware use as many cameras as the GPU can handle. |

**Summary:** Use **`launch_cv.py`** (real cameras) with task flags above. Build **`model.engine`** with **FP16**. Set **`inference_interval:=1`** for maximum detection rate (default is 2).

**Build FP16 engine (before competition):**
```bash
cd ~/Repos/School/autonomy-ws-25-26/computer_vision
python models/export_onnx.py model_building_and_training/aqua_main.pt
/usr/src/tensorrt/bin/trtexec --onnx=model_building_and_training/aqua_main.onnx --saveEngine=model_building_and_training/model.engine --fp16 --memPoolSize=workspace:4096 --skipInference
```

---

## Distance calibration (one-point)

The pipeline multiplies distance estimates by a **distance scale factor**. Use one or more photos at known distance/size to compute it; pass at launch (e.g. `distance_scale_factor:=0.87`). See **[camera_calibration/README.md](camera_calibration/README.md)** for steps and CSV-based calibration. Recompute after adding calibration photos.

---

## Iterating at competition

- **Override engine path:** `engine_path:=/path/to/model.engine`
- **Override resolution:** `resolution:=1920,1200`
- **Override cameras:** `camera_devices:=/path1,/path2,/path3` (exactly 3)
- **Simulation:** Use **`launch_cv_sim.py`** or **`launch_cv.py use_sim:=true`** when Gazebo bridges publish `/camera0/image_raw` etc. See [../src/asv_wave_sim/SIMULATION.md](../src/asv_wave_sim/SIMULATION.md).

For manual node commands and topic I/O, see [NODES.md](NODES.md).

---

## Task-Specific Computer Vision Requirements

### Overview

The computer vision pipeline provides the foundational perception capabilities required for all RoboBoat 2026 autonomy tasks. This section details how the vision system supports each task, what objects must be detected, and the specific computer vision capabilities required.

**Key Vision Capabilities Across All Tasks:**
- **Buoy Detection**: Red, green, black, and yellow buoys of various sizes
- **Color Indicator Detection**: Red/green color indicators mounted on custom buoys
- **Vessel Detection**: Yellow and black stationary vessels
- **Shape Detection**: Triangles, crosses/plus signs, and dock numbers
- **Spatial Understanding**: Bounding box coordinates for navigation and targeting

**Integration with Task Execution:**
- Vision detections are published to `/combined/detection_info_with_distance`; `bbox` is in local camera frame (preprocessed frame dimensions, e.g. 1920×1200); includes `bearing_deg`, `elevation_deg`, `distance_m`, `distance_ft`, `reference_height_m`
- Task4 supply drops: `source == "task4"`, `type` in `"yellow_supply_drop"`, `"black_supply_drop"`
- Task-specific nodes subscribe to detections and execute task logic
- Color indicator state detection is handled by `indicator_buoy_processor`
- Real-time detection enables dynamic navigation and decision-making

---

### Task 1: Evacuation Route & Return

**Objective**: Navigate through entry and exit gates marked by pairs of red and green buoys.

**Computer Vision Requirements:**

**Primary Detections:**
- **Red Buoys** (class_id: 3): Port marker buoys (Taylor Made Sur-Mark, 39in height, 18in diameter)
- **Green Buoys** (class_id: 1): Starboard marker buoys (Taylor Made Sur-Mark, 39in height, 18in diameter)

**Vision Processing:**
1. **Gate Detection**: Identify pairs of red and green buoys forming navigation gates
   - Minimum detection range: 6 ft before first gate (autonomous navigation start point)
   - Must detect both buoys in a pair to identify gate center
   - Calculate gate centerline for navigation path planning

2. **Spatial Analysis**:
   - Estimate distance to buoys using bounding box size and camera calibration
   - Calculate relative position (port/starboard) for gate alignment
   - Monitor ASV position relative to gate centerline

3. **Safety Monitoring**:
   - Continuous detection of all gate buoys during transit
   - Alert if ASV trajectory would cause collision
   - Verify complete passage through gate (both buoys behind ASV)

**Output Requirements:**
- Real-time detection of red/green gate buoys
- Bounding box coordinates for navigation planning
- Gate centerline calculation
- Completion verification (entry and exit gates)

**Performance Targets:**
- Detection range: >20 meters
- Update rate: ≥15 Hz for real-time navigation
- False positive rate: <5% (critical for collision avoidance)

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=false enable_task4:=false enable_number_detection:=false
```

---

### Task 2: Debris Clearance

**Objective**: Navigate through channel, enter debris field, identify hazards/survivors, and return.

**Computer Vision Requirements:**

**Primary Detections:**
- **Gate Buoys**: Red (class_id: 3) and green (class_id: 1) buoys marking channel entrance/exit
- **Black Buoys** (class_id: 0): Obstacle buoys representing debris (Polyform A-0, 0.5ft height)
- **Color Indicator Buoys**: Red or green indicators on custom buoys
  - Red indicator = hazard to avoid and report
  - Green indicator = survivor to rescue, circle, and report

**Vision Processing:**

1. **Channel Navigation**:
   - Detect red/green gate buoys forming channel boundaries
   - Maintain position within channel (between gate pairs)
   - Avoid contact with channel markers

2. **Debris Field Scanning**:
   - Detect all black obstacle buoys (debris)
   - Detect color indicator buoys and classify color (red/green)
   - Estimate positions of all detected objects for reporting

3. **Color Indicator Classification**:
   - Detect custom buoy with color indicator
   - Classify indicator color (red vs green) - see Color Indicator Detection section
   - Determine if indicator requires action (green = circle, red = avoid)

4. **Spatial Mapping**:
   - Build map of debris field with all object positions
   - Calculate relative positions for navigation planning
   - Identify safe paths through debris field

**Output Requirements:**
- Detection of all gate buoys (red/green)
- Detection of all black obstacle buoys
- Detection and color classification of color indicator buoys
- Position estimates (lat/long) for all detected objects
- Real-time obstacle map for path planning

**Performance Targets:**
- Detection range: >30 meters for debris field scanning
- Color classification accuracy: >95% (critical for survivor identification)
- Position estimation accuracy: ±1 meter (for reporting requirements)

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=true enable_task4:=false enable_number_detection:=false
```

---

### Task 3: Emergency Response Sprint

**Objective**: Pass through gate, circle yellow buoy in direction indicated by color indicator, exit through gate.

**Computer Vision Requirements:**

**Primary Detections:**
- **Gate Buoys**: Red (class_id: 3) and green (class_id: 1) buoys (Polyform A-2, 1ft height)
- **Yellow Buoy** (class_id: 5): Target buoy to circle (Polyform A-2, 1ft height)
- **Color Indicator Buoy**: Red or green indicator determining circling direction
  - Red = circle counter-clockwise (right side)
  - Green = circle clockwise (left side)

**Vision Processing:**

1. **Gate Detection and Transit**:
   - Detect entry gate (red/green buoys)
   - Navigate through gate centerline
   - Detect exit gate for return path

2. **Yellow Buoy Detection and Tracking**:
   - Detect yellow buoy in field of view
   - Track yellow buoy position continuously
   - Estimate distance and bearing for approach

3. **Color Indicator Classification**:
   - Detect color indicator on custom buoy
   - Classify as red or green (determines circling direction)
   - Verify indicator state before initiating circle maneuver

4. **Circling Maneuver Guidance**:
   - Monitor ASV position relative to yellow buoy
   - Provide real-time feedback for circle completion
   - Verify correct direction (clockwise vs counter-clockwise)

**Output Requirements:**
- Gate buoy detections (entry and exit)
- Yellow buoy detection with position tracking
- Color indicator detection and classification (red/green)
- Real-time position feedback during circling maneuver
- Completion verification

**Performance Targets:**
- Yellow buoy detection range: >25 meters
- Color indicator classification: >98% accuracy (critical for direction)
- Update rate: ≥15 Hz for high-speed maneuvering
- Low latency: <50ms end-to-end for responsive control

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=true enable_task4:=false enable_number_detection:=false
```

---

### Task 4: Supply Drop

**Objective**: Deliver water to yellow vessels (black triangle target) or racquetball to black vessels (black plus/cross target).

**Computer Vision Requirements:**

**Primary Detections:**
- **Yellow Vessels**: Stationary vessels identified by yellow colour blob with optional black triangle sign above
- **Black Vessels**: Stationary vessels identified by black colour blob with optional black cross/plus sign above

**Task4 Supply Processor** (required, `enable_task4:=true`): Subscribes only to `/camera{N}/image_preprocessed`; runs the simplified blob + shape pipeline internally; publishes `/camera{N}/task4_detections` with `type` (`yellow_supply_drop`, `black_supply_drop`), `vessel_bbox` (preprocessed frame), `confidence`, `has_shape`, `source: "task4"`. Combiner merges these into `/combined/detection_info`; `bbox` in combined is in local camera frame (preprocessed frame dimensions). Only detections with `confidence >= 0.65` are published.

**Vision Processing (Simplified Pipeline):**

1. **Blob Detection**:
   - Detect yellow blobs (HSV masking) and black blobs (combined HSV + LAB masking)
   - Apply morphological open/close to clean masks

2. **Filtering**:
   - **Spatial**: only blobs whose centre is in the lower 60% of the frame
   - **Aspect ratio**: height/width must be < 0.7 (vessel must be noticeably wider than tall)

3. **Blob Merging & Overlap Resolution**:
   - Merge nearby same-colour blobs into one bounding box
   - When bounding boxes of any colour overlap (IoU > 0.3), keep the larger one

4. **Shape Verification (confidence booster)**:
   - Search the region above each yellow vessel for a **black triangle** (≥2 similar-length sides)
   - Search the region above each black vessel for a **black cross** (intersecting vertical + horizontal bars)
   - A verified shape raises confidence from 0.70 up to a maximum of 1.00

5. **Confidence Threshold**:
   - Detections below 0.65 are discarded before publishing

**Output Requirements:**
- Yellow vessel detection with bounding box and confidence
- Black vessel detection with bounding box and confidence
- `has_shape` flag indicating whether a confirming shape was detected above the vessel

**Performance Targets:**
- Vessel detection range: >30 meters
- Confidence threshold: 0.65 (blob-only detections at 0.70; shape-verified up to 1.00)
- Real-time processing: ≥15 Hz

**Special Considerations:**
- No YOLO shape detection dependency — pipeline runs entirely from the raw camera image
- Robust detection in varying lighting conditions (water reflection)
- Handle partial occlusions (vessel may be partially visible)

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_task4:=true enable_indicator_buoy:=false enable_number_detection:=false
```

---

### Task 5: Navigate the Marina

**Objective**: Dock in available slip with lowest number, indicated by green color indicator.

**Computer Vision Requirements:**

**Primary Detections:**
- **Dock Structure** (class_id: 7): Floating dock with multiple slips
- **Color Indicators**: Red or green indicators on dock slips
  - Green = available slip
  - Red = occupied slip
- **Number Signs**: Black number banners (1, 2, or 3) on dock slips
- **Stationary Vessels**: Yellow/black vessels in occupied slips

**Vision Processing:**

1. **Dock Detection and Approach**:
   - Detect dock structure in marina
   - Identify dock orientation and slip layout
   - Plan approach path to marina

2. **Slip Availability Detection**:
   - Detect color indicators on each slip
   - Classify as green (available) or red (occupied)
   - Identify all available slips

3. **Number Sign Recognition**:
   - Detect number banners on dock slips
   - Recognize numbers 1, 2, or 3 (class_id: 20, 21, 22)
   - Associate numbers with slip positions

4. **Slip Selection Logic**:
   - Filter slips with green indicators (available)
   - Identify lowest number among available slips
   - Select target slip for docking

5. **Docking Guidance**:
   - Track target slip position
   - Provide real-time position feedback for docking maneuver
   - Verify successful docking in selected slip

**Output Requirements:**
- Dock structure detection
- Color indicator detection and classification (red/green) for each slip
- Number sign detection and recognition (1, 2, 3)
- Available slip identification with numbers
- Target slip selection (lowest number available)
- Real-time docking guidance

**Performance Targets:**
- Dock detection range: >40 meters
- Color indicator classification: >95% accuracy
- Number recognition accuracy: >90% (OCR or template matching)
- Slip identification: 100% accuracy (critical for correct docking)

**Special Considerations:**
- Must handle multiple slips simultaneously
- Robust number recognition in varying lighting/angles
- Handle cases where slip has red indicator but no vessel
- Accurate slip numbering is critical (docking in wrong slip = failure)

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_number_detection:=true enable_indicator_buoy:=false enable_task4:=false
```

**Note**: Ensure the number detection TensorRT engine exists (e.g. build from `task_specific/Docking/number_detection/` with `build_number_engine.sh`). Override if needed: `number_detection_engine:=/path/to/number_detection.engine`.

---

### Task 6: Harbor Alert

**Objective**: Detect audible signal and navigate to emergency zone (1 blast) or return to marina (2 blasts).

**Computer Vision Requirements:**

**Note**: This task is primarily audio-based (sound signal detection). Computer vision supports navigation to assigned zones but is not required for signal detection.

**Supporting Vision Requirements:**
- **Yellow Buoy Detection** (class_id: 5): Emergency response zone marker (Task 3 location)
- **Marina Detection** (class_id: 7): Dock structure for return navigation (Task 5 location)

**Vision Processing:**
1. **Emergency Zone Navigation** (1-blast signal):
   - Detect yellow buoy at Task 3 location
   - Navigate to yellow buoy position
   - Verify arrival at emergency zone

2. **Marina Return Navigation** (2-blast signal):
   - Detect dock structure (Task 5)
   - Navigate to marina entrance
   - Support docking if required

**Output Requirements:**
- Yellow buoy detection for emergency zone
- Dock detection for marina return
- Navigation guidance to assigned zone

**Performance Targets:**
- Detection range: >30 meters
- Fast response: <2 seconds from signal to navigation start

**Launch Command:**
```bash
ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=false enable_task4:=false enable_number_detection:=false
```

---

### Color Indicator Detection

**Overview**: Color indicators are 3D-printed cylinders (red or green) mounted on custom white buoys with black diamond markers. Used in Tasks 2, 3, and 5 for dynamic state information.

**Detection Method**: CV-only pipeline (not YOLO-based)
- **Stage 1**: Detect black diamond markers using edge detection and strict shape validation
- **Stage 2**: Validate white blob around diamonds (buoy body)
- **Stage 3**: **Multi-diamond grouping** - Group nearby diamonds (handles angled views)
- **Stage 4**: Detect and classify red/green indicator above diamonds
- **Stage 5**: Width-based distance estimation (20-inch reference)

**Key Features**:
- **Multi-diamond grouping**: Groups diamonds within 2× average size for angled buoy views
- **Collective centering**: Uses average position of grouped diamonds for accurate indicator ROI
- **Glare handling**: Adjusted thresholds (max_black_brightness: 230) for outdoor lighting
- **Robust classification**: Strict diamond validation + color masks for red/green

**Technical Specifications:**
- **Appearance**: Single-colored cylinder (red OR green), visible 360° horizontally
- **Mounting**: Custom buoy base (~18 inches across)
- **State**: Binary (red or green, not both simultaneously)
- **Visibility**: Horizontal plane only (not visible from above/below)

**Computer Vision Approach:**

**Detection Pipeline:**

1. **Buoy Detection**:
   - Primary vision pipeline detects custom buoy (class_id: 7 or specialized class)
   - Identifies bounding box of buoy structure

2. **Diamond Detection & Grouping**:
   - Detect all black diamonds using edge detection and shape validation
   - Group nearby diamonds (within 2× average size) that belong to same buoy
   - Calculate collective center for accurate indicator positioning
   - Validate white blob around diamonds (buoy body)

3. **Indicator ROI Computation**:
   - Position ROI directly above collective diamond center
   - ROI size: 3× diamond width, 2× diamond height
   - Handles both single and multi-diamond cases

4. **Color Classification**:
   - Apply BGR color masks (stricter for red, lenient for green)
   - Red mask: R > 120, R > G+25, R > B+25, G < 150, B < 150
   - Green mask: G > 80, G > R+5, G > B+5, R < 200, B < 200
   - Fallback: Channel dominance if primary masks fail

5. **State Reporting**:
   - Publish indicator state (red/green) with confidence scores
   - Include full bounding box for distance estimation
   - Report buoy confidence (diamond + white blob combined)

**Challenges and Solutions:**

**Challenge 1: Lighting Variations**
- **Solution**: Use HSV color space, adaptive thresholds, histogram analysis
- **Solution**: Temporal smoothing with state history

**Challenge 2: Partial Occlusion**
- **Solution**: Multi-camera fusion increases detection probability
- **Solution**: Confidence-based state reporting

**Challenge 3: Similar Colors (Red vs Green)**
- **Solution**: Precise color space thresholds
- **Solution**: Machine learning classifier if needed

**Performance Requirements:**
- Classification accuracy: >95% (critical for task decisions)
- Update rate: ≥10 Hz (sufficient for task execution)
- Detection range: >25 meters
- Robust to lighting: Works in direct sunlight, overcast, water reflection

**Integration:**
- **Node**: `indicator_buoy_processor` (self-contained, no external dependencies)
- **Subscribes**: `/camera{N}/image_preprocessed` (sensor_msgs/Image)
- **Publishes**: `/camera{N}/indicator_detections` (std_msgs/String JSON)
- **Output format**: `class_id` (9=red, 10=green), `score`, `shape_bbox`, `indicator_color`, `indicator_confidence`
- **Distance**: Handled by `maritime_distance_estimator` using bbox width (20-inch reference)
- **Documentation**: `task_specific/task_2_3/COLOUR_INDICATOR_BUOY.md`

**Parameters** (tuned for competition):
- `conf_threshold`: 0.6 (diamond confidence)
- `max_black_brightness`: 230 (handles glare)
- `buoy_conf_threshold`: 0.3 (combined diamond + white blob)
- `white_blob_expansion`: 2.0 (tight bounding boxes)
- `min_white_brightness`: 100 (outdoor scenes)
- `min_white_blob_score`: 0.15 (minimum white blob)

---

### Task Integration Summary

**Common Vision Capabilities Across Tasks:**

| Object Type | Class ID | Tasks Using | Critical Requirements |
|-------------|----------|-------------|----------------------|
| Red Buoy | 3 | 1, 2, 3 | Gate detection, navigation |
| Green Buoy | 1 | 1, 2, 3 | Gate detection, navigation |
| Black Buoy | 0 | 2 | Obstacle avoidance |
| Yellow Buoy | 5 | 3, 6 | Target identification |
| Yellow Vessel | 7 | 4 | Supply delivery target |
| Black Vessel | 7 | 4 | Supply delivery target |
| Dock | 7 | 5 | Marina navigation |
| Triangle | 8 | 4 | Water delivery target |
| Cross/Plus | 6 | 4 | Ball delivery target |
| Color Indicator | 9, 10 | 2, 3, 5 | Dynamic state detection |
| Docking Numbers | 20, 21, 22 | 5 | Slip identification |

**Task Execution Flow:**

```
Vision Pipeline → /combined/detection_info_with_distance (bbox, bearing, elevation, distance)
                    ↓
        Task-Specific Processors
                    ↓
    Task Execution Nodes (Navigation, Delivery, etc.)
```

**Key Design Decisions:**

1. **Separation of Concerns**: Primary vision pipeline focuses on object detection; task-specific processors handle task logic
2. **Modularity**: Each task can be developed/tested independently
3. **Reusability**: Common objects (buoys, vessels) detected once, used by multiple tasks
4. **Real-time Performance**: Combined detections enable fast task execution decisions
