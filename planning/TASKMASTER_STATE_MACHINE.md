# TaskMaster State Machine: Sequential Task Execution with Explicit Transitions

## Overview

The new `TaskMaster` implements a rigorous state machine for running Task1 → Task2 → Task3 sequentially with **explicit transition sub-states** between tasks. Each transition has specific entry actions, behavior logic, and exit conditions.

## State Machine Architecture

### Top-Level States

```
RUN_TASK_1
    ↓ (Task 1 SUCCESS)
TRANSITION_1_TO_2
    ↓ (distance moved OR gate detected OR timeout)
RUN_TASK_2
    ↓ (Task 2 SUCCESS)
TRANSITION_2_TO_3
    ├─ Phase A: TURN_TO_WEST_30
    └─ Phase B: FORWARD_SEARCH_FOR_TASK3_GATE
    ↓ (gate detected OR timeout)
RUN_TASK_3
    ↓ (Task 3 SUCCESS)
ALL_DONE
```

### Key Rules

1. **No backward transitions**: Once a task is complete, never return to it
2. **Explicit state logging**: Every state change logs exactly once with clear diagnostics
3. **Frozen tasks**: When a task completes, its goal publishing is stopped immediately
4. **Initial heading preservation**: Each task's `initial_heading` is set ONCE at task start

---

## TRANSITION_1_TO_2: Forward Nudge

### Goal
After Task 1 completes, deliberately push forward 1-2 meters to position in front of Task 2's buoy gate pair.

### Entry Actions (execute once)

```python
def _enter_transition_1_to_2(self):
    1. Log completion with pose, current_heading, Task1 initial_heading
    2. Freeze Task 1: clear goal_queue, stop tick()
    3. Initialize transition context:
       - transition_start_pose = current pose
       - transition_forward_heading = current heading
       - transition_distance_target_m = 1.5m (configurable)
       - transition_timeout_s = 10s
```

### Behavior (every tick)

```python
def _tick_transition_1_to_2(self):
    A) Calculate distance moved from transition_start_pose
    B) Detect Task 2 buoy gates (red_buoy + green_buoy ONLY, NO poles)
       - Use left/right pairing rule
       - Filter: gate must be AHEAD of boat
    C) Check exit conditions:
       - Condition 1: moved >= 1.5m
       - Condition 2: valid Task 2 gate detected
       - Condition 3: timeout (10s)
    D) Publish forward goal:
       - Position: transition_start_pose + 1.5m along transition_forward_heading
       - NOT nudged away from obstacles (enforce forward progress)
       - Clamped to map bounds
```

### Exit Actions (execute once)

```python
def _exit_transition_1_to_2(self):
    1. Log: distance moved, gate detected status, time spent
    2. Create Task 2 with initial_heading = transition_forward_heading
       (NOT current drifted heading - use intended course direction)
    3. Switch to RUN_TASK_2
```

### Logging Example

```
[TaskMaster] === DONE TASK 1 -> TRANSITION_1_TO_2 ===
  Pose: (12.3, 45.6, 0.785)
  Current heading: 45.0°
  Task1 initial_heading: 0.0°
  Transition params: target_dist=1.5m, forward_hdg=45.0°, timeout=10s

[TaskMaster] TRANSITION_1_TO_2: dist=0.42m/1.5m, gates=0, elapsed=1.2s/10s, exit=False
[TaskMaster] TRANSITION_1_TO_2: dist=0.89m/1.5m, gates=0, elapsed=2.5s/10s, exit=False
[TaskMaster] TRANSITION_1_TO_2: dist=1.52m/1.5m, gates=1, elapsed=4.1s/10s, exit=True

[TaskMaster] === TRANSITION_1_TO_2 COMPLETE -> STARTING TASK 2 ===
  Distance moved: 1.52m
  Gate detected: True
  Time spent: 4.1s
```

---

## TRANSITION_2_TO_3: Turn -30° West, Then Search

### Goal
After Task 2 completes, apply a hard **-30° WEST turn** from current heading, then drive forward (straight for a while) until Task 3's buoy gate pair is detected.

### Entry Actions (execute once)

```python
def _enter_transition_2_to_3(self):
    1. Log completion with pose and current_heading
    2. Freeze Task 2: clear goal_queue, stop tick()
    3. Initialize transition context:
       - transition_start_pose = current pose
       - target_heading = wrap_to_pi(current_heading - π/6)  # -30° WEST
       - turn_tolerance = 5°
       - turn_timeout = 8s
       - search_timeout = 60s (longer - goes straight for a while)
    4. Set phase = TURN_TO_WEST_30
```

### Phase A: TURN_TO_WEST_30

```python
def _tick_t23_turn(self):
    A) Calculate heading error = |current_heading - target_heading|
    B) Check turn completion:
       - SUCCESS: heading_error <= 5°
       - TIMEOUT: elapsed >= 8s
    C) Set Task3 initial_heading:
       - If SUCCESS: task3_initial_heading = target_heading
       - If TIMEOUT: task3_initial_heading = current_heading (best effort)
    D) Publish turn goal:
       - Small radius waypoint in target_heading direction
       - Forces rotation via position goal
    E) On exit: switch to FORWARD_SEARCH_FOR_TASK3_GATE phase
```

### Phase B: FORWARD_SEARCH_FOR_TASK3_GATE

```python
def _tick_t23_search(self):
    A) Detect Task 3 buoy gates (red_buoy + green_buoy ONLY, NO poles)
       - Use left/right pairing relative to task3_initial_heading
       - Filter: gate must be AHEAD
    B) Check exit conditions:
       - Condition 1: valid Task 3 gate detected
       - Condition 2: search timeout (60s from turn end - long straight run)
    C) Publish forward search goal:
       - Position: current_pose + 40m along task3_initial_heading (longer for straight approach)
       - Prioritize forward progress (minimize lateral nudging)
       - Clamped to map bounds
```

### Exit Actions (execute once)

```python
def _exit_transition_2_to_3(self):
    1. Log: turn success, final heading, gate detected, time spent
    2. Create Task 3 with initial_heading = task3_initial_heading
       (set during turn phase, NOT current drifted heading)
    3. Switch to RUN_TASK_3
```

### Logging Example

```
[TaskMaster] === DONE TASK 2 -> TRANSITION_2_TO_3 ===
  Pose: (50.2, 78.4, 1.571)
  Current heading: 90.0°
  Turn target: 60.0° (-30° WEST from 90.0°)
  Turn tolerance: 5°
  Search timeout: 60s (long straight run)

[TaskMaster] TRANSITION_2_TO_3 [TURN]: current=88.3°, target=60.0°, error=28.3°, elapsed=0.8s/8s
[TaskMaster] TRANSITION_2_TO_3 [TURN]: current=75.5°, target=60.0°, error=15.5°, elapsed=2.3s/8s
[TaskMaster] TRANSITION_2_TO_3 [TURN]: current=61.2°, target=60.0°, error=1.2°, elapsed=4.1s/8s
[TaskMaster] TURN phase SUCCESS: Task3 initial_heading = 60.0°

[TaskMaster] TRANSITION_2_TO_3 [SEARCH]: heading=60.0°, gates=0, elapsed_search=5.2s/60s
[TaskMaster] TRANSITION_2_TO_3 [SEARCH]: heading=60.0°, gates=0, elapsed_search=15.8s/60s
[TaskMaster] TRANSITION_2_TO_3 [SEARCH]: heading=60.0°, gates=0, elapsed_search=28.3s/60s
[TaskMaster] TRANSITION_2_TO_3 [SEARCH]: heading=60.0°, gates=2, elapsed_search=35.7s/60s

[TaskMaster] === TRANSITION_2_TO_3 COMPLETE -> STARTING TASK 3 ===
  Turn succeeded: True
  Final heading: 60.0°
  Gate detected: True
  Time spent: 39.8s
```

---

## Gate Detection Rules (CRITICAL)

### Task 1: ONLY Poles
- Gates: `red_pole_buoy` + `green_pole_buoy`
- NO regular buoys

### Tasks 2 & 3: ONLY Buoys
- Gates: `red_buoy` + `green_buoy`
- NO poles

### Left/Right Pairing Rule (ALL tasks)

Given boat heading:
1. Calculate forward vector: `(cos(heading), sin(heading))`
2. Calculate left vector: `(-sin(heading), cos(heading))`
3. For each red/green candidate pair:
   - Red must be to the LEFT of boat heading
   - Green must be to the RIGHT of boat heading
   - Gate center must be AHEAD of boat
   - Distance between red and green must be < 15m

### Gate Obstacle Exclusion

**CRITICAL**: Gate buoys/poles must be excluded from obstacles:

```python
# In Task managers:
def _obstacles(self):
    gate_ids = self.entities.get_gate_entity_ids(
        boat_heading_rad=self.pose[2],
        boat_pos=(self.pose[0], self.pose[1]),
    )
    return list(self.entities.get_obstacles(exclude_entity_ids=gate_ids)) + walls
```

**Gate goals must NOT be nudged**:
- Gate center is a mandatory waypoint
- Do NOT call `nudge_goal_away_from_obstacles()` on gate center goals
- Only nudge fallback/search goals

---

## Initial Heading Rule (GLOBAL)

### Task 1
```python
# Set once at initialization:
self.initial_heading = float(start_pose[2])

# Use in fallback:
d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)])
```

### Task 2
```python
# Set once when Task 2 starts (from TRANSITION_1_TO_2):
task2_start_pose = (x, y, t12_forward_heading)  # NOT current heading
self.task2_mgr = Task2Manager(entities, map_bounds, task2_start_pose)

# Task2 stores:
self.initial_heading = float(start_pose[2])
```

### Task 3
```python
# Set once AFTER turn in TRANSITION_2_TO_3:
if turn_succeeded:
    task3_initial_heading = target_heading  # -30° WEST from Task 2 end
else:
    task3_initial_heading = current_heading  # Best effort

task3_start_pose = (x, y, task3_initial_heading)
self.task3_mgr = Task3Manager(entities, map_bounds, task3_start_pose)

# Task3 stores:
self.initial_heading = float(start_pose[2])
```

### Exception: Gate Normal Override

If `last_gate_normal` is available (perpendicular to detected gate), it takes priority:

```python
if self.last_gate_normal is not None:
    d = self.last_gate_normal
else:
    d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)])
```

---

## Usage

### Sequential Mode (Task 123)

```python
from planning.TaskMaster import TaskMaster
from planning.Global.entities import EntityList

entities = EntityList()
start = (0.0, 0.0, 0.0)  # x, y, heading
map_bounds = (-50, -50, 50, 50)  # min_x, min_y, max_x, max_y

master = TaskMaster(
    entities=entities,
    start=start,
    task_id=123,  # Sequential mode
    map_bounds=map_bounds
)

while True:
    result = master.run_one_shot(
        get_pose=lambda: get_current_pose(),
        get_detections=lambda: get_current_detections(),
        use_planning=True,
        map_bounds=map_bounds
    )
    
    print(f"State: {result['state']}, Status: {result['status']}")
    
    if result['status'] == 'SUCCESS':
        break
```

### Single Task Mode

```python
# Task 1 only
master = TaskMaster(entities=entities, start=start, task_id=1, map_bounds=map_bounds)

# Task 2 only
master = TaskMaster(entities=entities, start=start, task_id=2, map_bounds=map_bounds)

# Task 3 only
master = TaskMaster(entities=entities, start=start, task_id=3, map_bounds=map_bounds)
```

---

## Testing Checklist

### TRANSITION_1_TO_2
- [ ] Task 1 goals cleared immediately on transition entry
- [ ] Forward goal published along transition_forward_heading (not current heading)
- [ ] Forward goal NOT nudged away from obstacles
- [ ] Task 2 buoy gates detected correctly (NO poles)
- [ ] Left/right pairing rule applied
- [ ] Exit when distance >= 1.5m OR gate detected OR timeout
- [ ] Task 2 initial_heading = transition_forward_heading (not current heading)

### TRANSITION_2_TO_3 (Turn Phase)
- [ ] Task 2 goals cleared immediately on transition entry
- [ ] Turn target = current_heading - 30° (wrapped to [-π, π])
- [ ] Turn goal published to encourage rotation
- [ ] Turn completes when heading error <= 5°
- [ ] Turn times out at 8s if unsuccessful
- [ ] Task3 initial_heading set correctly (target if success, current if timeout)

### TRANSITION_2_TO_3 (Search Phase)
- [ ] Forward goal published along task3_initial_heading (40m for long straight run)
- [ ] Task 3 buoy gates detected correctly (NO poles)
- [ ] Left/right pairing rule applied
- [ ] Gate must be AHEAD filter applied
- [ ] Exit when gate detected OR search timeout (60s - long straight approach)
- [ ] Task 3 created with initial_heading from turn phase

### General
- [ ] Each state change logs exactly once with full diagnostics
- [ ] Throttled logging during transition ticks (0.5s interval)
- [ ] No backward transitions
- [ ] Map bounds respected in all goal publishing
- [ ] Gate entities excluded from obstacles in all tasks

---

## Implementation Files

- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/TaskMaster.py` - Main state machine
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task1.py` - Task 1 manager (uses `initial_heading`)
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task2.py` - Task 2 manager (uses `initial_heading`)
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task3.py` - Task 3 manager (uses `initial_heading`)
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/INITIAL_HEADING_FIX.md` - Initial heading documentation
