# Roboboat 2026 – Autonomy Workspace

Welcome to the autonomy workspace that runs a full RoboBoat competition stack end‑to‑end:
perception, mapping, planning, and task‑level coordination, all built on ROS 2 Jazzy.

This repo is intended to be readable both as **production code** and as a **portfolio
project**. The sections below give a high‑level mental model first, then the minimum you
need to build and run it.

---

## What lives in this repo

- **Unified ROS 2 workspace**  
  All active ROS 2 packages live under `src/`:
  - `cv_ros_nodes`: multi‑camera TensorRT YOLO pipeline (preprocess → inference → combiner → distance estimator).
  - `task_sequence_coordinator`: competition state machine (Tasks 1, 3, 4, 5) plus patrol‑boat and sound‑interrupt handling, including pump control via MAVLink.
  - `message_node` + `message_node_msgs`: bridges internal ROS topics to RoboNation’s RoboCommand protocol (protobuf over TCP).
  - `sound_pipeline_launch` / sound packages: audio capture + sound‑signal detection (one‑blast / two‑blast).
  - Mapping / planning packages (`global_frame`, `global_planner`, `cv_lidar_fusion`, etc.) for global navigation.

- **Computer vision training + engines** (`computer_vision/`)  
  - `model_building_and_training/`: trained weights (`aqua_main.pt`), exported ONNX, and TensorRT engines (`model.engine`, `number_detection.engine`).
  - `models/`: conversion scripts and docs (`export_onnx.py`, `build_fp16_main_engine.sh`, `TENSORRT.md`).
  - `model_testing/`: standalone test harnesses for engines and ONNX.

- **Top‑level scripts**  
  - `build.sh`: builds the root workspace and the `computer_vision` workspace in one go.
  - `run_task_sequence.sh`: “lightweight competition” run – MAVROS + sound pipeline + task‑sequence coordinator (+ minimal CV for patrol boat).
  - `run_comp_three_cameras.sh` and friends: full autonomy stack (3‑camera CV, LiDAR, mapping, planner, messaging) for competition‑style runs.

---

## How the system fits together

At a high level, the boat runs as a set of cooperating ROS 2 nodes:

1. **Perception (CV + LiDAR)**  
   - Three USB cameras stream into `cv_ros_nodes` (`launch_cv.py`), which handles preprocessing and TensorRT YOLO inference.
   - A combiner merges detections from all cameras and a distance‑estimator annotates each detection with bearing/elevation/range.
   - Optional LiDAR fusion (`cv_lidar_fusion`) can combine CV and point clouds.

2. **Task‑level coordination**  
   - `task_sequence_coordinator` implements the RoboBoat task script as a timed state machine:
     - Task 1: entry/exit gates.
     - Task 3: speed gate + red‑light timing.
     - Task 4: object delivery, including pump control via MAVLink command‑ints to the flight controller relay.
     - Task 5: docking (hard‑coded slip for now).
   - It also reacts to:
     - **Patrol boat** detections from a dedicated CV node.
     - **Sound signals** (1 blast = yellow buoy, 2 blasts = marina), pausing and resuming the task timers cleanly.

3. **Mapping and planning**  
   - Mapping nodes fuse GPS, IMU, and LiDAR into a global frame.
   - A global planner produces waypoints that respect gates, obstacles, and task objectives.

4. **External messaging**  
   - `message_node` listens to internal topics (position, heading, ObjectDetected/ObjectDelivery, Dock, PatrolBoat, etc.).
   - It serializes reports into the RoboCommand wire format and sends them to the competition server.

The intention is to keep each concern (CV, mapping, planning, task logic, external comms) in its
own package, with clean topic/service interfaces between them.

---

## Build & run (quick start)

### 1. Build everything

From the repo root:

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

`build.sh` will:

- Build all ROS 2 packages under `src/`.
- Then build all CV packages under `computer_vision/src/` (so you can launch CV nodes as well).

If you only care about messaging + task sequence, `build_comp_messageing.sh` builds just:

- `message_node_msgs`, `message_node`, and `task_sequence_coordinator`.

---

### 2. Run the task‑sequence only (no planner / LiDAR)

This is the simplest way to see the state machine and pump control working:

```bash
cd ~/Repos/School/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_task_sequence.sh
```

`run_task_sequence.sh` will:

- Start MAVROS and connect to the flight controller.
- Start the sound pipeline + `message_node`.
- Start a single‑camera CV pipeline + patrol‑boat detector.
- Run the `task_sequence_coordinator` node, which:
  - Drives Tasks 1 → 3 → 4 → 5.
  - Commands the pump ON/OFF via MAVLink during Task 4.
  - Publishes all required RoboCommand reports via `message_node`.

You can monitor the high‑level messages with:

```bash
ros2 topic echo /cur_task
ros2 topic echo /messages/object_detected
ros2 topic echo /messages/object_delivered
ros2 topic echo /messages/docking
ros2 topic echo /messages/patrol_boat
```

### 3. Run the Gazebo boat simulator

If you want the simulation stack specifically, use [SIMULATION_QUICK_START.md](SIMULATION_QUICK_START.md).

That guide covers:

- the one-command tmux launcher for Gazebo + bridges + SITL + MAVROS
- what windows and panes should open
- which `test_*.sh` scripts are only for debugging individual pieces

---

## Where to look next

- **Task sequence logic:**  
  `src/task_sequence_coordinator/task_sequence_coordinator/task_sequence_coordinator_node.py`

- **RoboCommand bridge:**  
  `src/message_node/message_node/message_node.py`

- **Computer vision pipeline and docs:**  
  - `computer_vision/README.md` – CV system overview  
  - `computer_vision/models/TENSORRT.md` – how engines are built  
  - `src/cv_ros_nodes/cv_ros_nodes/vision_inference.py`

These files are good entry points if you want to review architecture, task logic, or CV
implementation details without getting lost in the rest of the tree.
