# Map Bounds Fix

## Problem Summary

The boat was driving past map bounds because:

1. **Issue 1:** TaskMaster was initialized with `map_bounds=None` (hardcoded)
2. **Issue 2:** Dynamic bounds from `get_map_bounds()` returned `None` before any gates/entities were detected
3. **Issue 3:** Dynamic bounds would shift every tick due to sensor noise on gate positions
4. Result: No arena perimeter walls generated → boat could drive anywhere, or walls would move unpredictably

## Solution

Added support for **static map bounds** via launch parameter AND **locked dynamic bounds** that stabilize once gates are detected:

### Usage

**Option 1: Static Map Bounds (Recommended for known arenas)**

```bash
# Example: 100m x 100m arena centered at origin
ros2 launch global_planner global_planner.launch.py \
  task_id:=1 \
  map_bounds:="[-50.0,-50.0,50.0,50.0]"
```

Format: `[min_x, min_y, max_x, max_y]` in meters (ENU frame: x=East, y=North)

**Option 2: Dynamic Map Bounds with Locking (Default)**

```bash
# No map_bounds parameter = dynamic from gates/entities (locks once gates detected)
ros2 launch global_planner global_planner.launch.py task_id:=1
```

Dynamic bounds behavior:
- Calculated from detected gate positions + entity positions
- Adds 20m padding around all entities
- **Once gates are detected, bounds are LOCKED** to prevent wall shifting from sensor noise
- Bounds unlock and recompute only if gate centers move >2m (indicates new gates or major reconfiguration)
- **Warning:** Returns `None` if no gates/entities detected yet!

### How It Works

1. **Initialization** (line 149):
   - If `map_bounds` parameter provided → use static bounds
   - Else → `None` (dynamic mode with locking)

2. **Runtime** (line 371-377):
   - Static mode: always use the provided bounds (never changes)
   - Dynamic mode: call `get_map_bounds()` each tick (locks after first gate detection)

3. **Dynamic Locking** (`entities.py`, lines 451-507):
   - First call with gates → compute bounds, lock them, store gate centers
   - Subsequent calls → check if current gate centers match locked centers (±2m tolerance)
   - If match → return locked bounds (stable walls)
   - If gates moved >2m → unlock, recompute, lock again with new positions
   - This prevents walls from shifting due to LiDAR/GPS noise while allowing updates if arena changes

4. **Boundary Wall Generation** (`entities.py`, line 433-448):
   - If `map_bounds` is 4-tuple `(min_x, min_y, max_x, max_y)`:
   - Samples vertical walls at `x=min_x` and `x=max_x`
   - Samples horizontal walls at `y=min_y` and `y=max_y`
   - Spacing: 1m between wall points
   - Creates full arena perimeter to constrain boat

### Task-Specific Behavior

- **Task 1:** Uses map_bounds (static or dynamic)
- **Task 2:** Uses map_bounds in TRANSIT_OUT and RETURN phases only
- **Task 3:** Ignores map_bounds (explicitly set to None)
- **Task 4:** Ignores map_bounds (explicitly set to None)

### Example Launch Commands

**Task 1 with 80m x 80m arena:**
```bash
ros2 launch global_planner global_planner.launch.py \
  task_id:=1 \
  map_bounds:="[-40.0,-40.0,40.0,40.0]" \
  cmd_vel_topic:=/uas1/mavros/setpoint_velocity/cmd_vel_unstamped
```

**Task 2 with rectangular arena (120m x 80m):**
```bash
ros2 launch global_planner global_planner.launch.py \
  task_id:=2 \
  map_bounds:="[-60.0,-40.0,60.0,40.0]"
```

**Task 3 (no bounds, speed challenge):**
```bash
ros2 launch global_planner global_planner.launch.py task_id:=3
# map_bounds parameter ignored for Task 3
```

### Dynamic Bounds Locking Behavior

When using dynamic bounds (no `map_bounds` parameter):

**Lock Trigger:**
- First detection of gates → compute bounds → **LOCK**
- Stores gate center positions at lock time

**While Locked:**
- Every tick: check if current gate centers match locked centers (±2m tolerance)
- If match → return locked bounds (walls stay stable)
- Prevents wall shifting from LiDAR/GPS noise (±0.1m to ±1m typical)

**Unlock Trigger:**
- Gate centers move >2m → **UNLOCK** → recompute → **LOCK** with new positions
- Allows adaptation if:
  - Arena configuration changes
  - New gates detected
  - Boat moves to different area with different gates

**Manual Unlock (for testing):**
```python
# From Python code if needed:
entity_list.unlock_map_bounds()
# Next call to get_map_bounds() will recompute
```

**Example Behavior:**
```
Tick 0:   No gates → bounds = None
Tick 1:   Gates detected at [(10,20), (30,20)] → compute bounds → LOCK
Tick 2-99: Gates at [(10.1,20.2), (30.3,19.9)] → within 2m → return locked bounds
Tick 100: Gates at [(15,25), (35,25)] → moved >2m → UNLOCK → recompute → LOCK
```

### Verifying Bounds Are Active

Check node logs for:
```
[global_planner_node]: Static map bounds: (-50.0, -50.0, 50.0, 50.0)
```
or
```
[global_planner_node]: Using dynamic map bounds from gates/entities
```

During planning tick, look for:
```
No-go zone identified: 402 boundary points
```

If you see `No-go zone: none`, bounds are not active!

### Integration with Comp Scripts

Add to your task launch scripts (e.g., `task1_comp.sh`, `task2_comp.sh`):

```bash
# Example: 100m x 100m arena
MAP_BOUNDS="[-50.0,-50.0,50.0,50.0]"

ros2 launch global_planner global_planner.launch.py \
  task_id:=1 \
  map_bounds:="${MAP_BOUNDS}" \
  cmd_vel_topic:=/uas1/mavros/setpoint_velocity/cmd_vel_unstamped &
```

Or use environment variable for easy override:
```bash
MAP_BOUNDS="${MAP_BOUNDS:-[-50.0,-50.0,50.0,50.0]}"
```

Then run:
```bash
# Default bounds
./task1_comp.sh

# Custom bounds
MAP_BOUNDS="[-30.0,-30.0,30.0,30.0]" ./task1_comp.sh
```
