# TRANSITION_2_TO_3 Update: -30° West Turn

## Changes Made

Updated the TRANSITION_2_TO_3 behavior based on real course requirements:

### **Turn Direction: -30° WEST (not +45° EAST)**

```python
# OLD: target_heading = wrap_to_pi(current_heading + π/4)  # +45° EAST
# NEW:
target_heading = wrap_to_pi(current_heading - π/6)  # -30° WEST
```

**Rationale**: The actual course layout requires turning LEFT (west) by 30° after Task 2, not right (east) by 45°.

### **Longer Forward Search**

Task 3 gates are far away - the boat goes straight for a while before encountering them.

#### Search Timeout
```python
# OLD: search_timeout_s = 25.0  # seconds
# NEW:
search_timeout_s = 60.0  # seconds (long straight run)
```

#### Forward Search Distance
```python
# OLD: search_dist = 25.0  # meters
# NEW:
search_dist = 40.0  # meters (longer for straight approach)
```

**Rationale**: Provides sufficient time and projection distance for the boat to find Task 3 gates after the turn, accounting for the longer straight run.

### **Updated Enum**

```python
# OLD:
class TransitionPhase(Enum):
    TURN_TO_EAST_45 = "TURN_TO_EAST_45"
    FORWARD_SEARCH_FOR_TASK3_GATE = "FORWARD_SEARCH_FOR_TASK3_GATE"

# NEW:
class TransitionPhase(Enum):
    TURN_TO_WEST_30 = "TURN_TO_WEST_30"
    FORWARD_SEARCH_FOR_TASK3_GATE = "FORWARD_SEARCH_FOR_TASK3_GATE"
```

### **Updated Logging**

```
[TaskMaster] === DONE TASK 2 -> TRANSITION_2_TO_3 ===
  Pose: (50.2, 78.4, 1.571)
  Current heading: 90.0°
  Turn target: 60.0° (-30° WEST from 90.0°)
  Turn tolerance: 5°
  Search timeout: 60s (long straight run)
```

## Behavior Summary

After Task 2 completes:

1. **Turn Phase**: Rotate -30° WEST (counterclockwise)
   - Target: `current_heading - 30°`
   - Success: heading error <= 5°
   - Timeout: 8s

2. **Search Phase**: Drive straight for a while
   - Direction: along `task3_initial_heading` (from turn)
   - Search distance: 40m forward goals
   - Look for Task 3 buoy gates (red_buoy + green_buoy)
   - Timeout: 60s (long enough for straight run to gates)

3. **Exit**: Start Task 3 when gates detected or timeout

## Course Geometry

```
        Task 2 End (heading: 90°)
              |
              | Turn -30° WEST
              ↓
        (heading: 60°)
              |
              | Long straight run (40m+ goals)
              | Search for Task 3 gates
              |
              ↓
        Task 3 Gates Detected
              |
              ↓
        Task 3 Start
```

## Files Modified

1. `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/TaskMaster.py`
   - Turn angle: -π/6 (-30°)
   - Search timeout: 60s
   - Search distance: 40m
   - Enum: `TURN_TO_WEST_30`

2. `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/TASKMASTER_STATE_MACHINE.md`
   - Updated all references to reflect -30° WEST
   - Updated search parameters
   - Updated logging examples
