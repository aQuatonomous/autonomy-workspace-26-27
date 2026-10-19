# Complete Planning System Guide

This document explains how the entire planning system works, based on the actual code implementation. It covers every component, how they interact, and what happens during execution.

---

## Table of Contents

1. [System Architecture Overview](#system-architecture-overview)
2. [Core Components](#core-components)
3. [Global Planner Node (ROS Integration)](#global-planner-node-ros-integration)
4. [TaskMaster (Orchestrator)](#taskmaster-orchestrator)
5. [EntityList (Data Management)](#entitylist-data-management)
6. [Task 1: Two Gates](#task-1-two-gates)
7. [Task 2: Debris Clearance](#task-2-debris-clearance)
8. [Task 3: Speed Challenge](#task-3-speed-challenge)
9. [Local Planner: Potential Fields](#local-planner-potential-fields)
10. [Watchdogs (Safety Protocols)](#watchdogs-safety-protocols)
11. [Data Flow and Execution](#data-flow-and-execution)

---

## System Architecture Overview

The planning system is a **two-layer architecture**:

- **Global Planner**: High-level task logic that determines *where* to go (goals/waypoints)
- **Local Planner**: Low-level path planning that determines *how* to get there (collision-free paths)

The system operates in a **reactive loop**:
1. Receive perception data (`/fused_buoys`) and odometry (`/odom`)
2. Update internal world model (EntityList)
3. Run task-specific logic to generate goals
4. Use potential fields planner to compute collision-free path to goals
5. Convert path to velocity commands for MAVROS
6. Publish commands and repeat at 10 Hz

---

## Core Components

### Component Hierarchy

```
GlobalPlannerNode (ROS Node)
    ├── EntityList (World Model)
    ├── TaskMaster (Orchestrator)
    │   ├── Task1Manager / Task2Manager / Task3Manager (Task Logic)
    │   └── PotentialFieldsPlanner (Local Planner)
    └── Watchdogs (Safety)
```

---

## Global Planner Node (ROS Integration)

**File**: `Global_Planner/global_planner/global_planner_node.py`

### Purpose
The ROS 2 node that bridges the ROS ecosystem with the planning library. It handles all ROS communication and orchestrates the planning loop.

### Subscriptions

1. **`/fused_buoys`** (FusedBuoyArray)
   - Input from CV-LiDAR fusion
   - Contains detected buoys with positions, IDs, and class IDs
   - Updates EntityList every time new detections arrive

2. **`/odom`** (nav_msgs/Odometry)
   - Boat position (x, y) and heading from MAVROS/Pixhawk
   - Used for:
     - Current pose in planning
     - Start position initialization (first odom message)
     - Velocity for collision detection (watchdogs)

3. **`/mavros/state`** (mavros_msgs/State, optional)
   - Pixhawk flight mode
   - Planning only runs when `mode == "GUIDED"`
   - If MAVROS not available, planning always runs (with warning)

4. **`/mavros/global_position/global`** (sensor_msgs/NavSatFix)
   - GPS coordinates for Task 2 debris reporting
   - Used to convert odom positions to lat/lon

### Publications

1. **`/planned_path`** (nav_msgs/Path)
   - Next waypoint/goal position
   - Published when goals exist
   - Frame: `odom` (or from odom header)

2. **`/mavros/setpoint_velocity/cmd_vel_unstamped`** (geometry_msgs/Twist)
   - Velocity commands in **body frame** (forward, left, up)
   - Converted from planner's world-frame velocities
   - Zero velocity if:
     - Not in GUIDED mode
     - No start position set
     - Watchdog override active
     - No goals/velocity from planner

3. **`/gs_message_send`** (std_msgs/String)
   - Task 2: Debris/indicator lat/lon reports (throttled every 2 seconds)
   - Task 3: Indicator color reports (throttled every 2 seconds)

### Planning Loop (10 Hz)

Every 0.1 seconds, the node:

1. **Checks GUIDED mode**: If not GUIDED, publish zero velocity and return
2. **Checks start position**: If not set, publish zero velocity and return
3. **Runs TaskMaster**: Calls `run_one_shot()` with current pose and detections
4. **Applies watchdogs**: Checks for collisions, stuck conditions, etc.
5. **Publishes results**: Path waypoint and velocity commands

### Key Functions

- **`_fused_callback()`**: Updates EntityList from `/fused_buoys`
- **`_odom_callback()`**: Updates pose, sets start position on first message
- **`_planning_tick()`**: Main planning loop (10 Hz timer callback)
- **`_odom_to_latlon()`**: Converts odom (m) to GPS (lat/lon) for Task 2 reports

---

## TaskMaster (Orchestrator)

**File**: `TaskMaster.py`

### Purpose
Central orchestrator that manages task lifecycle and coordinates between task managers and the local planner.

### Initialization

```python
TaskMaster(entities, start, task_id, map_bounds)
```

- **entities**: EntityList instance (shared world model)
- **start**: Starting position (x, y) in meters
- **task_id**: 1, 2, or 3 (determines which TaskManager to create)
- **map_bounds**: Optional (max_x, max_y) for arena boundaries

Creates:
- Appropriate TaskManager (Task1Manager, Task2Manager, or Task3Manager)
- PotentialFieldsPlanner instance

### Main Function: `run_one_shot()`

Called every planning tick (10 Hz) by the ROS node.

**Inputs**:
- `get_pose`: Function returning current (x, y, heading)
- `get_detections`: Function returning list of DetectedEntity
- `use_planning`: Whether to run local planner (default True)
- `map_bounds`: Optional map bounds update

**Process**:
1. Update task manager's pose from `get_pose()`
2. Add new detections to task manager
3. Call `manager.tick()` - updates goals based on task logic
4. Call `manager.plan(planner)` - computes path using potential fields
5. Return status, path, velocities, speeds

**Outputs**:
- `status`: "RUNNING" or "SUCCESS"
- `path`: List of waypoints [(x1, y1), (x2, y2), ...]
- `velocities`: Velocity vectors for each path point
- `speeds`: Speed magnitudes for each path point
- `phase`: Current phase (for Task 2/3)

---

## EntityList (Data Management)

**File**: `Global/entities.py`

### Purpose
Centralized world model that stores all detected entities (buoys, indicators, goals) and provides query methods for tasks.

### Entity Types

Mapped from class IDs (from CV-LiDAR fusion):
- `black_buoy`: Debris (class 0, 6, 7, 8, 11, 12, 20, 21, 22)
- `green_buoy`: Gate marker (class 1, 2)
- `red_buoy`: Gate marker (class 3, 4)
- `yellow_buoy`: Task 3 target (class 5)
- `red_indicator`: Task 2/3 indicator (class 9)
- `green_indicator`: Task 2/3 indicator (class 10)
- `goal`: Waypoints created by tasks (not from perception)

### Key Methods

#### `apply_tracked_buoys(entity_list, buoys, tolerance)`
Called by ROS node when `/global_detections` arrives (from detection_to_global, which feeds from `/fused_buoys` or `/tracked_buoys_json`).

**Process**:
1. For each buoy in message:
   - Extract position (x, y), ID, class_id (or color)
   - Map class_id to entity_type
   - Add or update entity in list (same ID = same entity)
2. Detect new red-green gate pairs
3. When new pair detected:
   - Push current entity IDs to "cohort" queue
   - If queue > 2 cohorts, prune oldest (removes old obstacles)

**Cohort System**: Prevents memory growth by dropping entities from gates that were passed long ago.

#### `get_gates(max_gate_width=15.0)`
Returns list of `(red_pos, green_pos)` gate pairs.

**Pairing Logic**:
- For each red buoy, find nearest green buoy within `max_gate_width`
- Filters: max Y difference (5 m), min X separation (2 m)
- Returns sorted pairs

#### `get_obstacles()`
Returns positions of all obstacle entities (everything except goals and start).

Used by:
- Local planner (potential fields repulsion)
- Watchdogs (collision detection)
- Goal nudging (avoid placing goals on obstacles)

#### `get_no_go_obstacle_points(map_bounds)`
Returns sampled points along gate "walls" (no-go zones).

**Process**:
- For each gate, creates left/right wall segments extending 15 m along gate normal
- Samples points every 1 m along walls
- If `map_bounds` provided, adds boundary points (arena walls)

#### `get_goals()`
Returns positions of all `goal` entities (waypoints created by tasks).

#### `get_black_buoys()`
Returns `(entity_id, position)` for all black buoys (Task 2 debris reporting).

---

## Task 1: Two Gates

**File**: `Global/Task1.py`

### Objective
Navigate through two gates in order, completing the mission when both gates are crossed.

### State Management

- **`goal_plan`**: List of GoalPlan objects (one per gate)
  - Each has: `goal_id`, `position`, `completed` flag
- **`locked_gates`**: Once 2 gates detected, locks them (prevents reordering)
- **`next_gate_index`**: Index of current active gate
- **`last_gate_normal`**: Direction vector for fallback (perpendicular to gate)

### Execution Flow

#### Phase 1: Gate Discovery and Locking

**`_lock_gates_once()`**:
- Waits until 2 gates detected
- Computes gate centers: `((red_x + green_x)/2, (red_y + green_y)/2)`
- Orders gates by projection onto forward direction:
  - Forward direction = boat heading (if available) OR start → nearest gate
- Creates GoalPlan for each gate center
- Locks gates (prevents changes)

#### Phase 2: Goal Publishing

**`publish_goals()`**:
- Clears old goals from EntityList
- Finds first incomplete goal in `goal_plan`
- Nudges goal away from obstacles (using `goal_utils.nudge_goal_away_from_obstacles`)
- Publishes as `goal_wp1` to EntityList
- Only publishes **one goal at a time** (next gate)

#### Phase 3: Gate Crossing Detection

**`_check_goal_completion()`**:
- Checks if boat motion segment (prev_pos → current_pos) intersects gate segment (red → green)
- Uses `goal_utils.segments_intersect()` for intersection test
- When gate crossed: marks corresponding GoalPlan as completed, increments `next_gate_index`

#### Phase 4: Fallback (No Gates Detected)

**`_fallback_goal()`**:
- If gates not locked yet:
  - Uses `last_gate_normal` (perpendicular to last seen gate) OR boat heading
  - Creates waypoint 100 m ahead along that direction
  - Clips to map_bounds if provided
  - Nudges away from obstacles

### Completion

**`done()`**: Returns `True` when all goals in `goal_plan` are completed.

### Planning

**`plan(planner, max_goals=1)`**:
- Gets current active goal from `goal_queue`
- Calls `planner.plan_multi_goal_path()` with:
  - Start: current pose
  - Goals: [next goal]
  - Obstacles: buoys + no-go points
  - Gates: locked gates (for gate alignment)
- Stores path, velocities, speeds in manager

---

## Task 2: Debris Clearance

**File**: `Global/Task2.py`

### Objective
Three-phase mission:
1. **TRANSIT_OUT**: Navigate through channel gates (outbound)
2. **DEBRIS_FIELD**: Find survivors (green indicators), circle them, avoid hazards
3. **RETURN**: Navigate back through channel to start

### State Management

- **`phase`**: TRANSIT_OUT, DEBRIS_FIELD, or RETURN
- **`report`**: In-memory storage of detected debris/indicators (for reporting)
- **`channel_centers_out`**: Gate centers saved during TRANSIT_OUT (for RETURN)
- **`handled_survivors`**: Set of survivor IDs already circled
- **`current_survivor_plan`**: 4-point circle plan for current survivor
- **`goal_reached_dist`**: Distance threshold (2 m) for goal completion

### Execution Flow

#### Phase A: TRANSIT_OUT

**`_lock_channel_centers()`**:
- Waits until 2+ gates detected
- Computes gate centers, sorts by Y (northbound)
- Saves centers to `channel_centers_out`
- Creates GoalPlan for each center (nudged away from obstacles)

**Goal Completion**:
- Checks distance to current goal
- If within `goal_reached_dist` (2 m), marks complete

**Transition to DEBRIS_FIELD**:
- When all channel goals completed → switch to DEBRIS_FIELD phase

**Fallback**:
- If no gates detected: forward waypoint 50 m along gate normal (same as Task 1 fallback)

#### Phase B: DEBRIS_FIELD

**Survivor Detection**:
- Looks for `green_indicator` entities
- For each unhandled survivor:
  - Creates 4-point circle plan around survivor (6 m radius)
  - Each point nudged away from obstacles
  - Stores in `current_survivor_plan`

**Goal Publishing**:
- If survivor circle active: publishes circle waypoints one at a time
- If no survivor seen: forward waypoint (same as fallback)

**Survivor Completion**:
- When all 4 circle points completed:
  - If more survivors: start next survivor circle
  - If no more survivors: transition to RETURN

**Reporting**:
- All detections added to `report` (green/red indicators, black buoys)
- ROS node publishes lat/lon reports to `/gs_message_send` (throttled)

#### Phase C: RETURN

**`_create_return_plan()`**:
- Reverses `channel_centers_out` list
- Creates GoalPlan for each waypoint (back through channel)
- Nudges away from obstacles

**Goal Publishing**:
- Publishes return waypoints one at a time

**Completion**:
- When all return goals completed → `mission_complete = True`

### Completion

**`done()`**: Returns `True` when `mission_complete` is True.

### Planning

**`plan(planner, max_goals=2)`**:
- Plans to next 2 goals (for smoother multi-waypoint paths)
- Uses same planner interface as Task 1

---

## Task 3: Speed Challenge

**File**: `Global/Task3.py`

### Objective
Fast completion:
1. Enter through gate (red + green buoys)
2. Detect indicator color (red = CCW, green = CW)
3. Circle yellow buoy in indicated direction
4. Exit through same gate

### State Management

- **`phase`**: ENTRY, SEARCH_YELLOW, LOOP_AROUND, EXIT
- **`entrance_gate`**: Gate pair (red, green) - locked once detected
- **`gate_center`**: Center of entrance gate (for entry/exit)
- **`entry_crossed`**: Boolean flag (gate crossing detected)
- **`exit_crossed`**: Boolean flag (gate crossing detected)
- **`indicator_color`**: "red" or "green" (set once, never changes)
- **`yellow_pos`**: Position of yellow buoy (when detected)
- **`loop_goal`**: Locked tangential waypoint around yellow (prevents flicker)
- **`start_time`**: Timestamp when entry gate crossed
- **`finish_time`**: Timestamp when exit gate crossed

### Execution Flow

#### Phase: ENTRY

**`_ensure_gate()`**:
- Waits for first gate pair
- Stores as `entrance_gate` and computes `gate_center`

**Goal Publishing**:
- Publishes `gate_center` as goal

**Gate Crossing**:
- `_check_gate_cross()` detects segment intersection
- When crossed: sets `entry_crossed = True`, starts timer (`start_time`)

**Transition**:
- When `entry_crossed` → switch to SEARCH_YELLOW phase

#### Phase: SEARCH_YELLOW

**Indicator Detection**:
- When `red_indicator` or `green_indicator` detected → sets `indicator_color`
- Never re-detects (locked once seen)

**Goal Publishing**:
- Bias waypoint based on indicator:
  - Red indicator → right side (CCW preparation)
  - Green indicator → left side (CW preparation)
- Waypoint: 25 m forward + 25 m to side

**Yellow Detection**:
- When `yellow_buoy` detected → stores `yellow_pos`

**Transition**:
- When both `yellow_pos` and `indicator_color` set → switch to LOOP_AROUND

#### Phase: LOOP_AROUND

**Locked Loop Goal**:
- **`_compute_locked_loop_goal()`** computes tangential point around yellow:
  - Vector from yellow to boat
  - Perpendicular vector (tangent direction)
  - Red indicator → CCW tangent (left normal)
  - Green indicator → CW tangent (right normal)
  - Point 20 m beyond yellow along tangent
- **Locked once**: `loop_goal` computed once on phase entry, never recomputed

**Goal Publishing**:
- Publishes `loop_goal` (locked, doesn't move)

**Completion Check**:
- When boat within 4 m of `loop_goal` → switch to EXIT

#### Phase: EXIT

**Goal Publishing**:
- Publishes `gate_center` (same gate as entry)

**Gate Crossing**:
- `_check_gate_cross()` detects exit crossing
- When crossed: sets `exit_crossed = True`, stops timer (`finish_time`)

### Completion

**`done()`**: Returns `True` when `exit_crossed` is True.

### Planning

**`plan(planner)`**:
- Plans to next 2 goals (entry/loop or loop/exit)
- Uses same planner interface

### Reporting

- ROS node publishes indicator color to `/gs_message_send` (throttled)
- Completion time computed: `finish_time - start_time`

---

## Local Planner: Potential Fields

**File**: `Local/potential_fields_planner.py`

### Purpose
Computes collision-free paths from current position to goals using potential field gradient descent.

### Algorithm Overview

**Potential Field** = Attractive (to goals) + Repulsive (from obstacles)

- **Attractive force**: Pulls toward goal (proportional to distance)
- **Repulsive force**: Pushes away from obstacles (exponential decay with distance)
- **Gate alignment**: When close to gate goal, adds alignment force (biases toward red side)

### Key Parameters

- **`K_ATT`** (1.8): Attractive gain (stronger pull to goals)
- **`K_REP`** (140.0): Repulsive gain (stronger push from obstacles)
- **`D_INFLUENCE`** (10.0 m): Obstacle influence distance
- **`MAX_VELOCITY`** (2.0 m/s): Maximum speed
- **`GOAL_THRESHOLD`** (2.0 m): Consider goal reached
- **`GATE_APPROACH_DIST`** (35.0 m): Start gate alignment when this close

### Path Computation: `gradient_descent_path()`

**Process**:
1. Start at current position
2. For each iteration (max 4000):
   - Compute gradient:
     - Attractive: `K_ATT * (current - goal)`
     - Repulsive: Sum over obstacles within `D_INFLUENCE`
       - Exponential decay: `K_REP * exp(-dist/2) / (2*dist)`
     - Gate alignment (if close to gate goal):
       - Bias toward red side (offset along gate normal)
       - Strength increases as distance decreases
   - Step along negative gradient (toward goal, away from obstacles)
   - Adaptive step size (damped by gradient magnitude)
   - If gradient too small (local minimum):
     - Nudge with random angle spread (toward goal direction)
     - After `MAX_NUDGE_COUNT` (50) nudges → fallback to straight line
   - Add point to path if moved enough (`MIN_POINT_SPACING`)
   - Check goal reached (`GOAL_THRESHOLD`)
3. Return path array

### Path Smoothing: `smooth_path()`

- Moving average filter (window size 5, 2 iterations)
- Reduces jaggedness from gradient descent
- Endpoints (goals) remain fixed

### Velocity Computation: `compute_velocities()`

**Process**:
1. Compute velocity vectors: `(path[i+1] - path[i]) / dt`
2. Smooth velocities (moving average, window 3)
3. Compute speeds: `norm(velocities)`
4. Clip to `MAX_VELOCITY`
5. Return velocities and speeds arrays

### Multi-Goal Planning: `plan_multi_goal_path()`

**Process**:
1. For each goal in sequence:
   - Compute path segment from current position to goal
   - Append to full path (skip duplicate start point)
   - Update current position to goal
2. Smooth entire path
3. Compute velocities for full path
4. Return: path, velocities, speeds, segment_indices

### Gate Alignment

When planning to a gate goal:
- If within `GATE_APPROACH_DIST` (35 m):
  - Computes gate normal (perpendicular to red→green vector)
  - Adds alignment force toward red side (offset `GATE_OFFSET` = 0.8 m)
  - Strength increases as distance decreases
  - Helps boat approach gate from correct side

---

## Watchdogs (Safety Protocols)

**File**: `Global_Planner/global_planner/watchdogs.py`

### Purpose
Safety layer that overrides planner commands when dangerous conditions detected.

### Protocols

#### 1. Stuck Protocol
- **Trigger**: After `MAX_SPIN_SEARCHES` (3) spin+reorient cycles with no goal
- **Action**: Publish zero velocity forever
- **State**: `stuck = True` (never resets)

#### 2. Collision Avoidance
- **Trigger**: Obstacle within `COLLISION_DISTANCE_M` (2.0 m) AND closing speed > 0.1 m/s
- **Action**: Reverse at `REVERSE_SPEED_M_S` (0.5 m/s) for `REVERSE_DISTANCE_M` (1.0 m)
- **State**: `reverse_remaining_m` counts down

#### 3. Prediction Timeout (No Goal)
- **Trigger**: No goal for `PREDICTION_TIMEOUT_SEC` (25 seconds)
- **Action**: Spin at `SPIN_ANGULAR_RATE_RAD_S` (0.5 rad/s) for `SPIN_DURATION_SEC` (20 seconds)
- **State**: `spin_until_time` tracks spin end time

#### 4. Reorient After Spin
- **Trigger**: After spin completes
- **Action**: Rotate back to `original_heading` at `REORIENT_RATE_RAD_S` (0.4 rad/s)
- **State**: `reorient_until_time` tracks reorient end
- **Completion**: When heading error < `REORIENT_DONE_THRESHOLD_RAD` (0.1 rad)

#### 5. Reset on Goal
- **Trigger**: When goal appears
- **Action**: Resets all prediction/spin/reorient state
- **State**: Clears timeouts, resets counters

### Execution Flow

**`tick(state, time_now, dt_sec, pose, velocity, obstacles, has_goal)`**:

1. **Check stuck**: If stuck → return zero velocity
2. **Check collision**: If obstacle close and closing → start reverse
3. **Check reversing**: If reversing → continue reverse, decrement remaining
4. **Check has goal**: If goal exists → reset prediction state, return None (no override)
5. **Check prediction start**: If no prediction started → start timer
6. **Check spin phase**: If spinning → continue spin
7. **Check spin complete**: If spin just finished → start reorient
8. **Check reorient phase**: If reorienting → rotate toward original heading
9. **Check prediction timeout**: If timeout exceeded → start spin
10. **Return None**: No override needed

### State Dictionary

```python
{
    "stuck": bool,
    "reverse_remaining_m": float,
    "spin_until_time": float | None,
    "reorient_until_time": float | None,
    "reorient_target_heading": float | None,
    "prediction_start_time": float | None,
    "original_heading": float,
    "spin_search_count": int,
}
```

### Integration

Called every planning tick by `global_planner_node._planning_tick()`:
- If watchdog returns override → publish override velocity, skip planner
- Otherwise → use planner output

---

## Data Flow and Execution

### Complete Execution Cycle (10 Hz)

```
1. ROS Node receives /fused_buoys
   └─> _fused_callback()
       └─> apply_tracked_buoys(EntityList, buoys)
           └─> Updates EntityList with new detections
           └─> Detects gate pairs, manages cohorts

2. ROS Node receives /odom
   └─> _odom_callback()
       └─> Updates _latest_pose (x, y, heading)
       └─> Sets start position (first message)
       └─> Computes world-frame velocity

3. Planning Timer (10 Hz)
   └─> _planning_tick()
       ├─> Check GUIDED mode → if not, publish zero, return
       ├─> Check start set → if not, publish zero, return
       ├─> TaskMaster.run_one_shot()
       │   ├─> Update manager pose
       │   ├─> Add detections to manager
       │   ├─> manager.tick()
       │   │   ├─> Task-specific goal generation
       │   │   ├─> Goal completion checks
       │   │   └─> Publish goals to EntityList
       │   └─> manager.plan(planner)
       │       └─> PotentialFieldsPlanner.plan_multi_goal_path()
       │           ├─> gradient_descent_path() for each goal
       │           ├─> smooth_path()
       │           └─> compute_velocities()
       ├─> Watchdog check
       │   └─> watchdog_tick()
       │       └─> Returns override or None
       ├─> If watchdog override → publish override, return
       └─> Publish planner output
           ├─> /planned_path (next waypoint)
           └─> /mavros/setpoint_velocity/cmd_vel_unstamped (body frame)
```

### Coordinate Frames

- **World Frame (odom)**: East-North-Up (ENU)
  - Used by: EntityList positions, planner paths, odometry
- **Body Frame (base_link)**: Forward-Left-Up (FLU)
  - Used by: MAVROS velocity commands
  - Conversion: Rotate world-frame velocity by boat heading

### Goal Lifecycle

1. **Detection**: Buoys detected → added to EntityList
2. **Task Logic**: Task manager generates goals based on task rules
3. **Nudging**: Goals nudged away from obstacles (if too close)
4. **Publishing**: Goals added to EntityList as `goal` entities
5. **Planning**: Local planner computes path to goals
6. **Execution**: Velocity commands sent to MAVROS
7. **Completion**: Task checks if goal reached (distance or gate crossing)
8. **Cleanup**: Completed goals removed from active list

### Entity Lifecycle

1. **Perception**: CV-LiDAR fusion publishes `/fused_buoys`
2. **Update**: `apply_tracked_buoys()` adds/updates entities
3. **Cohort Management**: New gate pairs trigger cohort creation
4. **Pruning**: Old cohorts pruned when > 2 cohorts exist
5. **Query**: Tasks query EntityList for gates, obstacles, goals
6. **Cleanup**: Pruned entities removed from list

---

## Summary

The planning system is a **reactive, hierarchical architecture**:

- **Global Planner Node**: ROS integration, data flow, safety
- **TaskMaster**: Orchestrates task execution
- **Task Managers**: Task-specific goal generation (Task 1/2/3)
- **EntityList**: Centralized world model
- **Potential Fields Planner**: Collision-free path computation
- **Watchdogs**: Safety overrides

The system runs at **10 Hz**, continuously:
1. Updating world model from perception
2. Generating goals based on task logic
3. Computing collision-free paths
4. Publishing velocity commands
5. Monitoring safety conditions

Each component is **ROS-free** (except the node), making the planning library reusable and testable independently of ROS.
