# Planning

High-level task planning and goal generation for autonomous navigation. Consumes perception (`/global_detections` from detection_to_global) and GPS/heading from MAVROS, produces ordered goals and velocity commands.

---

## Running the planning node (setup and run)

### 1. Workspace and file layout

The planning node needs **two things** in your workspace:

- **ROS package `global_planner`** — must be built by colcon (e.g. under `src/global_planner` or `planning/Global_Planner` symlinked/copied into `src/`).
- **Planning library (Python, no ROS)** — the folder that contains `TaskMaster.py`, `Global/`, and `Local/`. This can live **anywhere** as long as the node can add it to `sys.path`.

**Typical layouts:**

- **Option A (monorepo):** Repo root is the colcon workspace; `planning/` is at workspace root (sibling of `install/`). Launch file will set `PLANNING_PATH` to `workspace_root/planning` automatically.
- **Option B (src layout):** Colcon workspace has `src/`; put the planning library at `src/planning/` (so `src/planning/TaskMaster.py`, `src/planning/Global/`, etc.). Launch file looks for `share/../../../src/planning` first.

Ensure the planning library contains:

- `TaskMaster.py`
- `Global/` (`entities.py`, `types.py`, `goal_utils.py`, `Task1.py`, `Task2.py`, `Task3.py`, `Task4.py`)
- `Local/potential_fields_planner.py`

**Check that everything is there:** run this from the **planning** directory (the folder that contains `TaskMaster.py`). All lines should report "exists"; if any say "missing", add or fix that path.

```bash
cd /path/to/planning
for f in TaskMaster.py Global/entities.py Global/types.py Global/goal_utils.py Global/Task1.py Global/Task2.py Global/Task3.py Global/Task4.py Local/potential_fields_planner.py Global_Planner/package.xml Global_Planner/setup.py Global_Planner/launch/global_planner.launch.py Global_Planner/global_planner/global_planner_node.py Global_Planner/resource/global_planner; do
  [ -e "$f" ] && echo "exists: $f" || echo "missing: $f"
done
```

If you use a separate colcon workspace, the **planning library** is the directory that contains `TaskMaster.py` and `Global/`; `PLANNING_PATH` must point to that directory. The **Global_Planner** package can live inside it (as here) or elsewhere; colcon just needs to build it (e.g. `--paths planning/Global_Planner`).

### 2. Dependencies

- **global_frame** (from mapping): provides `GlobalDetectionArray`. Must be built before `global_planner`.
- **nav_msgs**, **geometry_msgs**, **std_msgs**, **sensor_msgs**: standard ROS 2.
- **mavros_msgs** (optional): if present, planning runs only when `mode == "GUIDED"`; otherwise the node runs without the gate and logs a warning.

### 3. PLANNING_PATH

The launch file sets `PLANNING_PATH` so the node can `import TaskMaster`, `Global.entities`, etc. It tries:

1. `share/../../../src/planning`
2. `share/../../../../planning`

If your layout is different, set it before launch:

```bash
export PLANNING_PATH=/path/to/planning   # folder containing TaskMaster.py
ros2 launch global_planner global_planner.launch.py
```

### 4. Build

From workspace root:

```bash
colcon build --packages-select pointcloud_filters global_planner
source install/setup.bash
```

If `global_planner` is not under `src/`, add the path to colcon (e.g. `--paths planning/Global_Planner` or symlink into `src/`).

### 5. Launch

```bash
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
# Task 2 or 3:
ros2 launch global_planner global_planner.launch.py task_id:=2
# Custom cmd_vel topic:
ros2 launch global_planner global_planner.launch.py cmd_vel_topic:=/cmd_vel
```

### 6. Verify it’s working

- **Node up:** `ros2 node list | grep global_planner`
- **Subscriptions:** Node subscribes to `/global_detections`, `/mavros/state`, `/mavros/global_position/global`, `/mavros/global_position/compass_hdg`, `/mavros/global_position/gp_vel`. Ensure at least `/global_detections` and GPS/heading are published.
- **Planning only in GUIDED:** If using MAVROS, put the vehicle in GUIDED mode; otherwise the node publishes zero twist.
- **Logs:** Look for `[t=...] x=... y=... (m) |` and "Buoy update", "Seen red-green buoy pairs", "Velocity given".

---

## Components: logic, run, debug

### Global planner (node)

- **Logic:** Subscribes to `/global_detections` (GlobalDetectionArray from detection_to_global), MAVROS GPS/heading/velocity. Each tick: if not GUIDED → publish zero twist; else update EntityList from detections, call TaskMaster `run_one_shot(pose, detections)`, run watchdogs; if watchdog override → publish it; else publish first goal to `/planned_path` and planner velocity to cmd_vel. Task 2: throttle-reported debris/indicator lat/lon to `/gs_message_send`. Task 3: throttle-reported indicator color to `/gs_message_send`.
- **Run:** See "Running the planning node" above. No separate "run" for the library; it’s used only via the node (or a custom script that imports TaskMaster).
- **Debug:** See [Global_Planner/README.md](Global_Planner/README.md). Quick checks: `ros2 topic echo /mavros/state --field mode`, `ros2 topic echo /planned_path`, `ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped`. Logs: "No goal seen", "Velocity given: zero", "This is the watchdog protocol happening".

---

### Watchdogs

- **Logic:** ROS-free. Called every planning tick with (state, time, dt, pose, velocity, obstacles, has_goal). (1) **Stuck:** if already stuck → return zero twist forever. (2) **Collision:** if obstacle within `COLLISION_DISTANCE_M` and closing speed > 0.1 → command reverse for `REVERSE_DISTANCE_M` at `REVERSE_SPEED_M_S`. (3) **Reversing:** decrement reverse remaining, keep publishing reverse. (4) **Has goal:** reset prediction/spin state. (5) **No goal:** track prediction start; after `PREDICTION_TIMEOUT_SEC` → **spin** for `SPIN_DURATION_SEC`, then **reorient** to original heading; after `MAX_SPIN_SEARCHES` cycles with still no goal → set stuck.
- **Run:** No standalone run; used only inside `global_planner_node`.
- **Debug:** Tune constants in `Global_Planner/global_planner/watchdogs.py` (e.g. `COLLISION_DISTANCE_M`, `PREDICTION_TIMEOUT_SEC`, `MAX_SPIN_SEARCHES`). Logs: "Watchdog protocol: object avoidance", "spinning to look", "reorienting", "I am stuck." If the boat spins forever, increase timeout or fix goal generation (gates/indicators).

---

### Task 1 (two gates)

- **Logic:** Gate centers from red_buoy + green_buoy pairs; order gates by projection onto forward direction (boat heading or start→nearest gate). Gate is “completed” when the segment (prev_pos → current_pos) intersects the gate segment (red→green). Writes goals as goal_wp1, goal_wp2, …; SUCCESS when both gates crossed in order. Fallback: if no gate detected, waypoint straight ahead along last gate normal (max 100 m) until next gate appears.
- **Run:** `ros2 launch global_planner global_planner.launch.py` (task_id defaults to 1).
- **Debug:** Check entity list has red_buoy and green_buoy; gates = `entities.get_gates()`. Logs: "Seen red-green buoy pairs", "No goal seen", "next goal". If no gates, perception or class_id mapping may be wrong (see `Global/entities.py` CLASS_ID_TO_ENTITY_TYPE; class_id 255 = unknown).

---

### Task 2 (debris clearance)

- **Logic:** Phases: **TRANSIT_OUT** (navigate gate centers out, save waypoints); **DEBRIS_FIELD** (go straight until green_indicator; avoid red_indicator and black_buoy; circle survivors with 4 waypoints; report debris/indicator lat/lon); **RETURN** (back through channel in reverse). Fallback when no gate in TRANSIT_OUT: forward waypoint along last gate normal (max 50 m) until gate detected.
- **Run:** `ros2 launch global_planner global_planner.launch.py task_id:=2`. Ensure `/mavros/global_position/global` is published so global reference is set for debris reports.
- **Debug:** Logs: phase, "Seen red-green buoy pairs", black/green/red indicator counts. Debris strings on `/gs_message_send`: `ros2 topic echo /gs_message_send`. If no reports, check task_id=2, global ref set, and entity list has black_buoy / red_indicator / green_indicator.

---

### Task 3 (speed challenge)

- **Logic:** Enter gate (red+green) → store gate for exit; detect color indicator (red → counter-clockwise, green → clockwise); generate waypoints around yellow buoy in that direction; exit through same gate. Gate crossing via segment intersection (prev_pos→current_pos vs gate segment). Reports indicator color and completion time to `/gs_message_send`.
- **Run:** `ros2 launch global_planner global_planner.launch.py task_id:=3`.
- **Debug:** Logs: gates, "indicator_color", goal/velocity. Check entity list has red_buoy, green_buoy, yellow_buoy, red_indicator/green_indicator; indicator color in logs and on `/gs_message_send`.

---

### Potential fields (Local)

- **Logic:** Attractive force to goal, repulsive from obstacles (within `D_INFLUENCE`). Optional gate alignment when close to goal (red/green positions). Gradient descent with step limit and damping; on local minimum, nudge with random angle spread; after `MAX_NUDGE_COUNT` fallback to straight line to goal. Path smoothed; velocities computed along path. Returns path waypoints and velocities (world frame).
- **Run:** No standalone; called by TaskMaster via `manager.plan(planner)` each tick.
- **Debug:** Tune in `Local/potential_fields_planner.py`: `K_ATT`, `K_REP`, `D_INFLUENCE`, `GOAL_THRESHOLD`, `MAX_VELOCITY`, `GATE_APPROACH_DIST`. If the boat doesn’t move toward goal, check that goals exist and obstacles aren’t blocking; "Velocity given: zero" suggests planner returned no velocity (e.g. at goal or stuck).

---

### Entities (Global)

- **Logic:** `EntityList`: list of `Entity` (type, position, name, entity_id). `apply_tracked_buoys()`: add/update by id from `GlobalDetectionArray` (`/global_detections`); when a new red-green pair is seen, push current ids to cohort; when cohorts > 2, prune oldest (drop entities not in current pair). `get_gates()`: pair red_buoy with nearest green_buoy within max width → (red_pos, green_pos). `get_obstacles()`: all non-goal, non-start types (red/green/black/yellow/indicators). `get_no_go_obstacle_points()`: sampled points along gate “walls” and optional map boundary. `get_goals()`: positions of type "goal". `get_black_buoys()`: (id, pos) for Task 2 reporting. Class mapping: `CLASS_ID_TO_ENTITY_TYPE` in `Global/entities.py` (0–22 per `computer_vision/class_mapping.yaml`; 255 = unknown).
- **Run:** No standalone; populated by node from `/global_detections`, read by Task1/2/3 and planner.
- **Debug:** Wrong gate pairing or missing buoys → check class_id in `/global_detections` (source: detection_to_global) and `CLASS_ID_TO_ENTITY_TYPE` in `Global/entities.py`. class_id 0–22 map to entity types; 255 = unknown (unmatched LiDAR or invalid). "Dropped entities" in logs = cohort prune (normal after passing a gate). Ensure frame matches (detection_to_global publishes map frame).

---

### TaskMaster

- **Logic:** Holds one task manager (Task1, Task2, or Task3) and one PotentialFieldsPlanner. Each tick: update manager pose and detections (from callbacks), update map_bounds if provided; call `manager.tick()` (task updates goals in EntityList); call `manager.plan(planner)` to get path and velocities; return status, path, velocities, speeds. SUCCESS when `manager.done()`.
- **Run:** Used by global_planner_node; no direct CLI. For custom use: instantiate TaskMaster(entities, start, task_id, map_bounds), then loop `run_one_shot(get_pose=..., get_detections=...)`.
- **Debug:** If status never SUCCESS, the task’s `done()` isn’t true (e.g. gate not crossed, phase not finished). Check manager.phase (Task2/3), manager’s goal completion logic, and entity list contents.

---

### Goal utils (Global)

- **Logic:** `segments_intersect(p1, p2, q1, q2)`: true if segment p1→p2 intersects q1→q2 (used for gate crossing). `nudge_goal_away_from_obstacles(desired, obstacles, from_point, min_clearance, step)`: slide goal back toward from_point until at least min_clearance from all obstacles; keeps direction from_point→goal.
- **Run:** Library only; used by Task1, Task2, Task3.
- **Debug:** Gate not registering as crossed → check segment intersection (boat segment vs gate segment) and that prev_pos/pose are updated. Goals too close to obstacles → adjust `DEFAULT_GOAL_CLEARANCE` or pass larger min_clearance in task code.

---

### Types (Global)

- **Logic:** `DetectedEntity(entity_id, entity_type, position)` — used when passing detections from EntityList back into TaskMaster/task (e.g. for consistency). `Vec2` = (float, float).
- **Run/Debug:** No standalone; used across Global and TaskMaster.

---

## Monitoring and debugging (quick reference)

| Check | Command |
|-------|--------|
| Node running | `ros2 node list \| grep global_planner` |
| MAVROS mode | `ros2 topic echo /mavros/state --field mode` |
| Planning output | `ros2 topic echo /planned_path`, `ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped` |
| Perception input | `ros2 topic hz /fused_buoys`, `ros2 topic echo /fused_buoys --once` |
| Task 2/3 reports | `ros2 topic echo /gs_message_send` |
| Params | `ros2 param list /global_planner_node`, `ros2 param get /global_planner_node task_id` |

**Troubleshooting:** No cmd_vel → check GUIDED mode and that task has goals. Boat spins forever → prediction-timeout watchdog; increase `PREDICTION_TIMEOUT_SEC` or fix gates/indicators. Boat hits obstacles → tune `COLLISION_DISTANCE_M` in watchdogs. No debris/indicator reports → task_id, global ref, and entity types (black_buoy, red_indicator, green_indicator).

---

## Links

- **Global_Planner** ([Global_Planner/README.md](Global_Planner/README.md)) — Node topics, parameters, watchdogs, build/run.
- **Mapping** ([../mapping/README.md](../mapping/README.md)) — LiDAR, `/tracked_buoys`, fusion → `/fused_buoys`.
- **Computer vision** ([../computer_vision/README.md](../computer_vision/README.md)) — CV pipeline, fusion input.
