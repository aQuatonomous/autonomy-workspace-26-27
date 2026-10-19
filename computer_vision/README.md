# Computer Vision ROS2 Pipeline

Multi-camera object detection with TensorRT-optimized YOLO on ROS 2 Jazzy: 3 cameras → preprocessing → inference → combiner → distance estimator. Final output: **`/combined/detection_info_with_distance`** (JSON, ~30 Hz) with bearing, elevation, and distance for navigation and CV–LiDAR fusion.

---

## Documentation

| Doc | Purpose |
|-----|--------|
| **[COMPETITION.md](COMPETITION.md)** | What to run at competition and when; task-specific launch table; inference/FP16/calibration; iterating at competition. |
| **[NODES.md](NODES.md)** | Complete node architecture, build, launch, manual commands, topic I/O, message formats, troubleshooting. |

**See also:** [camera_calibration/README.md](camera_calibration/README.md) (distance scale factor), [models/TENSORRT.md](models/TENSORRT.md) (TensorRT and engine build), [FUSION_NODE_GUIDE.md](FUSION_NODE_GUIDE.md) (CV + LiDAR fusion), [DISTANCE_ESTIMATOR_CHANGES.md](DISTANCE_ESTIMATOR_CHANGES.md) (distance estimator), [Simulation guide](../src/asv_wave_sim/SIMULATION.md) (sim + CV).

---

## System Architecture

### High-Level Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  Camera 0 (/dev/video0)  │  Camera 1 (/dev/video2)  │  Camera 2 (/dev/video4) │
│  1920x1200 @ 15 FPS      │  1920x1200 @ 15 FPS      │  1920x1200 @ 15 FPS     │
└──────────────┬────────────┴──────────────┬────────────┴──────────────┬─────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│   Camera Driver Node 0    │  │  Camera Driver Node 1  │  │  Camera Driver Node 2  │
│  (v4l2_camera_node)       │  │  (v4l2_camera_node)    │  │  (v4l2_camera_node)   │
│  Namespace: /camera0      │  │  Namespace: /camera1   │  │  Namespace: /camera2  │
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│  Preprocessing Node 0     │  │  Preprocessing Node 1  │  │  Preprocessing Node 2  │
│  (vision_preprocessing)   │  │  (vision_preprocessing)│  │  (vision_preprocessing)│
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│   Inference Node 0        │  │   Inference Node 1     │  │   Inference Node 2    │
│  (vision_inference)      │  │  (vision_inference)   │  │  (vision_inference)   │
│  TensorRT Engine         │  │  TensorRT Engine       │  │  TensorRT Engine       │
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
               └───────────────┬───────────┴───────────────┬───────────┘
                               │                           │
                    ┌──────────▼───────────────────────────▼──────────┐
                    │      Detection Combiner Node                    │
                    │      (vision_combiner)                          │
                    │      - Aggregates all detections                │
                    │      - Health monitoring                        │
                    └───────────────────┬────────────────────────────┘
                                        │
                    ┌───────────────────▼────────────────────────────┐
                    │    Maritime Distance Estimator                 │
                    │    (maritime_distance_estimator)                │
                    │    - Adds bearing, elevation, distance         │
                    └───────────────────┬────────────────────────────┘
                                        │
                    ┌───────────────────▼────────────────────────────┐
                    │         Final CV Output                         │
                    │         /combined/detection_info_with_distance │
                    └─────────────────────────────────────────────────┘
```

### Key Design Principles

- **Fault Isolation**: Independent pipelines per camera ensure single camera failures don't cascade
- **Scalability**: Modular design allows easy addition/removal of cameras
- **Performance**: TensorRT optimization for real-time inference (development on Think; deployment e.g. NVIDIA Jetson Orin)
- **Reliability**: Robust error handling and graceful degradation

### Data Flow Pipeline

**Stage 1: Image Acquisition**
- Raw images captured at 1920x1200 resolution, 15 FPS (FPS set via v4l2-ctl; see `set_camera_fps.sh` in repo root)
- Published as ROS2 `sensor_msgs/Image` messages

**Stage 2: Preprocessing**
- Resize to camera resolution (pass-through by default, 1920x1200)
- Future: glare reduction, color correction, histogram equalization
- Output: Preprocessed images ready for inference

**Stage 3: Inference**
- TensorRT engine processes preprocessed images
- YOLO-based object detection
- Output: Bounding boxes, class IDs, confidence scores

**Stage 4: Aggregation**
- Combines detections from all cameras
- Output: Unified detection list with camera source tracking
- Overlap-based merging: detections in the 20% overlap of adjacent cameras (same class, similar bbox size) are merged into one with a duplicate flag, both bboxes, and averaged bearing/elevation; all other detections are preserved

**Stage 5: Distance Estimation**
- Adds bearing, elevation, and distance to each detection
- Uses pinhole camera model with reference object sizes
- Applies distance scale factor calibration

---

## Hardware Requirements

**Development platform**: Think (x86). Camera USB paths are in `set_camera_fps.sh` (repo root); on Jetson or other hardware, run `./monitor_camera_devices.sh` and edit that script or override `CAMERA_DEVICES`.

**Deployment target (e.g. boat)**: NVIDIA Jetson Orin
- GPU: NVIDIA Orin (2048 CUDA cores)
- CPU: ARM Cortex-A78AE (12 cores)
- Memory: 32 GB LPDDR5
- Storage: 64 GB+ NVMe SSD
- OS: Ubuntu 20.04 / JetPack 5.x

**Camera Requirements**:
- 3x USB 3.0 cameras
- Resolution: 1920x1200 minimum
- Frame rate: 15 FPS capable
- V4L2 compatible

**Physical Configuration**:
- **Camera arrangement**: 3 cameras mounted side-by-side
- **Field of view per camera**: 85 degrees (horizontal)
- **Overlap between adjacent cameras**: 15 degrees
- **Total horizontal coverage**: ~245 degrees
- **Camera mounting angles**: Camera 0 (left) = -70°, Camera 1 (center) = 0°, Camera 2 (right) = +70°

---

## Software Stack

**Operating System**:
- Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- Linux kernel 5.15+ (Tegra for Jetson)

**ROS2 Distribution**:
- ROS2 Humble Hawksbill
- DDS Implementation: FastDDS or CycloneDDS

**Python Environment**:
- Python 3.8+
- Required packages:
  - `rclpy` (ROS2 Python client)
  - `sensor_msgs` (ROS2 message types)
  - `cv_bridge` (OpenCV-ROS2 bridge)
  - `opencv-python` (4.5+)
  - `numpy` (1.19+)
  - `tensorrt` (10.3.0)

**CUDA/TensorRT**:
- CUDA 12.6
- TensorRT 10.3.0
- cuDNN 8.x

**System Packages**:
- `v4l2-utils` (camera utilities)
- `ros-jazzy-v4l2-camera` (camera driver)

---

## Quick start

```bash
cd ~/Repos/School/autonomy-ws-25-26 && ./set_camera_fps.sh single
cd computer_vision && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cv_ros_nodes launch_cv_single_camera1.py camera1_device:="$(cat ../.camera_devices)"
```

Task-specific launch and competition settings: [COMPETITION.md](COMPETITION.md). Pipeline and nodes: [NODES.md](NODES.md).

---

## Project structure

```
computer_vision/
├── camera_calibration/     # Distance scale factor, calibration CSV
├── model_building_and_training/  # Trained models: .pt, .onnx, .engine
├── src/
│   ├── cv_ros_nodes/       # Nodes, launch_cv.py, launch_cv_sim.py
│   └── cv_lidar_fusion/    # vision_lidar_fusion → /fused_buoys
├── class_mapping.yaml      # YOLO class mapping (shared by CV and planner)
├── task_specific/          # Task 2/3, Docking number detection
│   # set_camera_fps.sh lives in repo root
├── README.md
├── COMPETITION.md
├── NODES.md
└── (FUSION_NODE_GUIDE.md, DISTANCE_ESTIMATOR_CHANGES.md)
```

---

## TensorRT Model Training Pipeline

### Overview

The model training pipeline converts a trained YOLO model (PyTorch/ONNX) into an optimized TensorRT engine for deployment (e.g. Jetson Orin; engines can also be built on the Think).

### Pipeline Stages

#### Stage 1: Model Training (External)

**Input**: Labeled dataset (images + annotations)  
**Output**: Trained PyTorch model (`.pt` file)

**Tools**: PyTorch, YOLOv5/YOLOv8 training scripts  
**Location**: External training environment (typically GPU workstation)

**Key Requirements**:
- Dataset format: COCO or YOLO format
- Model architecture: YOLOv5n, YOLOv5s, YOLOv8n, etc.
- Training parameters: Batch size, learning rate, epochs
- Validation split: 20% recommended

#### Stage 2: Model Export to ONNX

**Input**: Trained PyTorch model (`.pt`)  
**Output**: ONNX model (`.onnx`)

**Process**: Use `torch.onnx.export` with fixed input size (e.g. 1×3×640×640) for TensorRT. For machine-specific steps and where files live, see [TENSORRT.md](model_building_and_training/TENSORRT.md).

**Validation**: Verify ONNX loads, test inference on samples, compare with PyTorch.

#### Stage 3: TensorRT Engine Conversion

**Input**: ONNX model (`.onnx`)  
**Output**: TensorRT engine (`.engine`)

For machine-specific trtexec path, PATH, conversion commands (FP32/FP16/INT8), and Python API, see [TENSORRT.md](model_building_and_training/TENSORRT.md).

**Precision Selection Guide**:
- **FP32**: Highest accuracy, slowest (baseline)
- **FP16**: ~2x faster, <1% accuracy loss (recommended for Jetson / Think)
- **INT8**: ~4x faster, requires calibration dataset, 1-3% accuracy loss

**Optimization Parameters**:
- `workspace`: GPU memory for optimization (4096 MB recommended)
- `minShapes`: Minimum input dimensions
- `optShapes`: Optimal input dimensions (640x640)
- `maxShapes`: Maximum input dimensions

#### Stage 4: Engine Validation

**Input**: TensorRT engine (`.engine`)  
**Output**: Validation report

**Process**: Use `model_building_and_training/test_inference.py` (or equivalent) on validation images; compare with ground truth, compute mAP/precision/recall. See [TENSORRT.md](model_building_and_training/TENSORRT.md).

**Metrics**:
- Mean Average Precision (mAP@0.5)
- Inference latency (ms)
- Throughput (FPS)
- Memory usage (GPU)

#### Stage 5: Deployment

**Input**: Validated TensorRT engine  
**Output**: Deployed model in ROS2 nodes

**Process**:
1. Build `.engine` file into `model_building_and_training/` directory
2. Use default `--engine_path` from launch scripts (points to `~/Repos/School/autonomy-ws-25-26/computer_vision/model_building_and_training/model.engine`)
3. Verify inference nodes load engine successfully
4. Monitor performance in production

**File Locations**:
- Engine file: `/home/lorenzo/Repos/School/autonomy-ws-25-26/computer_vision/model_building_and_training/model.engine`
- ONNX file: `/home/lorenzo/Repos/School/autonomy-ws-25-26/computer_vision/model_building_and_training/aqua_main.onnx`
- PyTorch weights: `/home/lorenzo/Repos/School/autonomy-ws-25-26/computer_vision/model_building_and_training/aqua_main.pt`

### Model Versioning

**Naming Convention**: `model_v{version}_{precision}.engine`
- Example: `model_v1.0_fp16.engine`

**Version Tracking**:
- Maintain changelog with model versions
- Track performance metrics per version
- Document dataset and training parameters

---

## Class ID Mapping

**Maritime Object Detection Class IDs**:

| Class ID | Class Name | Category | Dataset Count |
|----------|------------|----------|---------------|
| 0 | black_buoy | Buoy | 4,931 |
| 1 | green_buoy | Buoy | 9,007 |
| 2 | green_pole_buoy | Buoy | 7,533 |
| 3 | red_buoy | Buoy | 8,760 |
| 4 | red_pole_buoy | Buoy | 6,767 |
| 5 | yellow_buoy | Buoy | 4,938 |
| 6 | cross | Navigation Marker | 3,311 |
| 7 | dock | Infrastructure | 4,991 |
| 8 | triangle | Navigation Marker | 3,345 |
| 9 | red_indicator_buoy | Indicator Buoy | - |
| 10 | green_indicator_buoy | Indicator Buoy | - |
| 11 | yellow_supply_drop | Supply Drop | - |
| 12 | black_supply_drop | Supply Drop | - |
| 20 | digit_1 | Docking Number | - |
| 21 | digit_2 | Docking Number | - |
| 22 | digit_3 | Docking Number | - |

**Total Classes**: 23 (as defined in `class_mapping.yaml`)

**Note**: 
- Class IDs 0-8: Main YOLO model detections
- Class IDs 9-10: **Indicator buoy processor** (CV-only, diamond-based detection with multi-diamond grouping)
- Class IDs 11-12: Task 4 supply processor
- Class IDs 20-22: Number detection model (remapped from model's 0,1,2)

**Indicator Buoy Detection** (Classes 9-10):
- **Method**: CV-only pipeline (not YOLO-based)
- **Features**: Black diamond detection → white blob validation → multi-diamond grouping → red/green classification
- **Distance**: Width-based estimation using 20-inch reference
- **Documentation**: See `task_specific/task_2_3/COLOUR_INDICATOR_BUOY.md`
- **Node**: `indicator_buoy_processor` (self-contained, no external dependencies)

**Configuration File**: `class_mapping.yaml` (in the `computer_vision/` root)

See [NODES.md](NODES.md) for complete message formats and detection output structure.

---

## License

MIT License. Contributors: Lorenzo DeMarni.
