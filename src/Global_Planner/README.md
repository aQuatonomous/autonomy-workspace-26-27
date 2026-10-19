# Global Planner (ROS2 Node)

ROS2 node that runs the planning stack (Task1/2/3 + potential fields) on live perception and odometry, and publishes path and velocity commands to MAVROS. Includes watchdogs (collision avoidance, prediction-timeout spin, stuck detection) and Task 2 debris reporting (lat/long to ground station).

---

## Setup so the node works

- **Workspace:** Build `global_frame` and `global_planner` in the same colcon workspace (or have `global_frame` in underlay). The planning **library** (folder with `TaskMaster.py`, `Global/`, `Local/`) must be reachable: the launch file sets `PLANNING_PATH` to `.../src/planning` or `.../planning` (sibling of `install/`). If your layout differs, set `export PLANNING_PATH=/path/to/planning` before launch.
- **Files:** Ensure the planning library contains `TaskMaster.py`, `Global/` (entities, types, goal_utils, Task1/2/3), and `Local/potential_fields_planner.py`. No extra config files required.
- **Run:** `source install/setup.bash` then `ros2 launch global_planner global_planner.launch.py` (optionally `task_id:=2` or `task_id:=3`).

Full setup, layout options, and component-level logic/run/debug: **[planning README](../README.md)**.

---

## How It Works

1. **Subscriptions**
   - `/global_detections` (global_frame/GlobalDetectionArray): detections in map frame (east, north, id, class_id/class_name). class_id 0–22 map to entity types; 255 = unknown. The node updates an internal `EntityList` by **id** (cohort-based pruning as before).
   - `/mavros/global_position/global` (NavSatFix): first fix sets origin; each message converts lat/lon to local (east, north) in metres for pose.
   - `/mavros/global_position/compass_hdg` (std_msgs/Float64): compass heading (degrees, 0=North, 90=East); converted to ENU heading (rad) for pose.
   - `/mavros/global_position/gp_vel` (geometry_msgs/TwistStamped): velocity in NED; used for watchdogs (world-frame velocity).
   - `/sound_signal_interupt` (std_msgs/Int32): 1 = stop boat (publish zero cmd_vel), 2 = ignore.
   - `/mavros/state` (mavros_msgs/State, optional): planning runs **only when `mode == "GUIDED"`**.

2. **Each planning tick (e.g. 10 Hz)**
   - If **not** GUIDED: publish zero twist and return (no planning).
   - Otherwise: call TaskMaster `run_one_shot(pose, detections)` → task updates goals and runs potential-field planner → get path and velocities.
   - Run **watchdogs**: collision (stop + reverse), prediction timeout (spin then reorient), or stuck (stop forever). If any override, publish watchdog twist and return.
   - Otherwise: publish next waypoint to `/planned_path` and first velocity to the cmd_vel topic.
   - If **task_id == 2** and global reference is set: get black buoys from entity list, convert positions to lat/long, publish string to `/gs_message_send`.

3. **Units**
   - Internal planning, path, and MAVROS all use **meters** and **m/s**.

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/global_detections` | global_frame/GlobalDetectionArray | in | Detections in map frame (east, north, id, class_id/class_name). |
| `/mavros/global_position/global` | sensor_msgs/NavSatFix | in | GPS; first fix sets origin, then lat/lon → local x,y (m). |
| `/mavros/global_position/compass_hdg` | std_msgs/Float64 | in | Compass heading (deg). |
| `/mavros/global_position/gp_vel` | geometry_msgs/TwistStamped | in | Velocity (NED) for watchdogs. |
| `/sound_signal_interupt` | std_msgs/Int32 | in | 1 = stop boat, 2 = ignore. |
| `/mavros/state` | mavros_msgs/State | in | Pixhawk state; `mode == "GUIDED"` required to run planning. |
| `/planned_path` | nav_msgs/Path | out | Current goal waypoint (one pose, frame_id=map). |
| `/curr_task` | std_msgs/Int32 | out | Current task_id (1, 2, or 3). |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | geometry_msgs/Twist | out | Velocity command (body frame, m/s and rad/s). |
| `/gs_message_send` | std_msgs/String | out | Task 2 only: debris report string `DEBRIS_LATLON|id,lat,lon|...`. |

---

## Parameters (Node Knobs)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `task_id` | int | 1 | Task to run: 1 (two gates), 2 (debris clearance), 3 (speed challenge). |
| `planning_hz` | double | 10.0 | Rate (Hz) of the planning timer. |
| `cmd_vel_topic` | string | `/mavros/setpoint_velocity/cmd_vel_unstamped` | Topic to publish Twist commands. |
| `heading_topic` | string | `/mavros/global_position/compass_hdg` | Compass heading (Float64, degrees). |
| `gp_vel_topic` | string | `/mavros/global_position/gp_vel` | Velocity (TwistStamped, NED). |
| `sound_signal_topic` | string | `/sound_signal_interupt` | Int32: 1 = stop boat, 2 = ignore. |

---

## Troubleshooting: no velocity commands

- **Echo shows nothing or wrong topic:** The node publishes to `cmd_vel_topic` (default `/mavros/setpoint_velocity/cmd_vel_unstamped`). If MAVROS runs with a UAS prefix (e.g. `/uas1`), it subscribes under that prefix. Either echo the topic the node uses, or launch the planner so it publishes where MAVROS listens:
  ```bash
  ros2 launch global_planner global_planner.launch.py cmd_vel_topic:=/uas1/mavros/setpoint_velocity/cmd_vel_unstamped
  ```
- **Velocity is always zero:** Planning only sends non-zero velocity when: (1) Pixhawk is in **GUIDED** mode (`/mavros/state`), (2) at least one **GPS fix** has been received, (3) for Task 1, **gate (red–green) detections** exist so the planner has goals. Check logs for "Task1 phase=...", "Seen goal / next goal", and "Velocity given: ...".

---

## Watchdogs

The node calls `watchdogs.tick()` every planning tick. If the returned twist override is not `None`, that command is published instead of the planner output and a log line is emitted (`"This is the watchdog protocol happening. ..."`).

### Behavior

1. **Stuck**  
   If the boat has spun to look for goals multiple times and still has no goal, it is declared stuck: zero twist is published forever and logs report `"I am stuck."`

2. **Object avoidance (collision)**  
   If an obstacle is within a distance threshold and the boat is moving toward it (positive closing speed), the node commands **stop** then **reverse** for a fixed distance at a fixed speed, then resumes normal planning.

3. **Prediction timeout (no goal)**  
   If there is no goal for longer than a timeout, the node enters a **spin** phase (rotate in place for a fixed duration), then **reorients** back to the original heading. If after several such cycles there is still no goal, the **stuck** behavior is triggered.

### Watchdog Knobs (in `global_planner/watchdogs.py`)

| Constant | Default | Description |
|----------|---------|-------------|
| `COLLISION_DISTANCE_M` | 2.0 | Trigger avoidance when obstacle within this distance (m) and approaching. |
| `REVERSE_DISTANCE_M` | 1.0 | Distance to reverse (m) after collision trigger. |
| `REVERSE_SPEED_M_S` | 0.5 | Speed (m/s) during reverse. |
| `PREDICTION_TIMEOUT_SEC` | 25.0 | No-goal duration (s) before starting spin. |
| `SPIN_DURATION_SEC` | 20.0 | Duration (s) of spin when looking for goals. |
| `SPIN_ANGULAR_RATE_RAD_S` | 0.5 | Angular velocity (rad/s) during spin. |
| `REORIENT_RATE_RAD_S` | 0.4 | Angular velocity (rad/s) when reorienting to original heading. |
| `REORIENT_DONE_THRESHOLD_RAD` | 0.1 | Heading error (rad) below which reorient is done. |
| `MAX_SPIN_SEARCHES` | 3 | Number of spin+reorient cycles before declaring stuck. |

Tune these based on real-world behavior (e.g. increase collision distance if the boat gets too close, or increase prediction timeout if goals appear slowly).

---

## Debugging

- **Logs**  
  The node logs with a prefix `[t=<s>] x=<m> y=<m> (m) |` so you can correlate behavior with time and position. Look for:
  - `"Waiting for GUIDED"` / no planning: ensure `/mavros/state` is published and `mode == "GUIDED"`.
  - `"No goal seen"`: task has no current goal (e.g. no gates detected, or phase not yet set).
  - `"This is the watchdog protocol happening"` / `"We are spinning"` / `"I am stuck"`: watchdog is active; adjust watchdog knobs or perception/task logic.
  - `"Dropped entities"`: cohort prune removed old gate pair + obstacles; normal if the boat has passed to a new pair.

- **Points of failure**
  - **No cmd_vel when in GUIDED:** Check that `/mavros/state` is received and `mode` is exactly `"GUIDED"`. If `mavros_msgs` is not installed, the node allows planning without the gate (see log at startup).
  - **Boat never moves:** Guided gate passed? Task has goals? Planner returning non-zero velocity? Watchdog not overriding with zero? Check logs for "Velocity given: zero" vs "Velocity given: vx=...".
  - **Boat spins repeatedly:** Prediction-timeout watchdog is firing (no goal for too long). Increase `PREDICTION_TIMEOUT_SEC` or fix goal generation (gates/indicators) so goals appear.
  - **Boat hits obstacles:** Collision watchdog may be triggering too late or not at all. Decrease `COLLISION_DISTANCE_M` or check that obstacles are in the entity list and passed to the watchdog. Ensure odom and obstacle positions are in the same frame.
  - **Task 2 no debris report:** Ensure `task_id == 2`, global reference is set (subscribe to `/mavros/global_position/global` and get one message), and entity list has `black_buoy` entities. Check that `/gs_message_send` is published (e.g. `ros2 topic echo /gs_message_send`).

- **Useful commands**
  - See current mode: `ros2 topic echo /mavros/state --field mode`
  - See last cmd_vel: `ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped`
  - See planned path: `ros2 topic echo /planned_path`
  - See debris report: `ros2 topic echo /gs_message_send`

---

## Build and Run

From workspace root (with `planning` and `pointcloud_filters` in the workspace):

```bash
colcon build --packages-select global_planner
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

Run with a specific task:

```bash
ros2 run global_planner global_planner_node --ros-args -p task_id:=2
```

If the planning library is not on the path, set it before running:

```bash
export PLANNING_PATH=/path/to/workspace/src/planning
ros2 run global_planner global_planner_node
```

---

## Dependencies

- **planning** (sibling directory or `PLANNING_PATH`): TaskMaster, Task1/2/3, entities, potential_fields_planner.
- **pointcloud_filters**: FusedBuoyArray (and FusedBuoy) message.
- **nav_msgs**, **geometry_msgs**, **std_msgs**, **sensor_msgs**: Standard ROS2.
- **mavros_msgs** (optional): If available, planning runs only when `mode == "GUIDED"`; if not installed, the node runs planning without the gate and logs a warning.
