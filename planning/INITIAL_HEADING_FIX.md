# Initial Heading Fix for Forward Goals

## Problem

Previously, all tasks used the **current heading** from `self.pose[2]` when computing fallback goals. This caused issues:

1. If the boat drifted/rotated due to currents or wind, fallback goals would point in the wrong direction
2. The boat would not maintain its original intended course
3. Search patterns (Task 3) would be based on current orientation rather than the mission's original direction

## Solution

Store the **initial heading** from `start_pose[2]` at task initialization and use it for all forward projections and fallback goals.

### Changes Made

#### Task 1 (`Task1.py`)

**1. Store initial heading in `__init__`:**
```python
self.initial_heading: float = float(start_pose[2])
```

**2. Update `_fallback_goal()` to use initial heading:**
```python
# OLD: d = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
# NEW:
d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)], dtype=float)
```

#### Task 2 (`Task2.py`)

**1. Store initial heading in `__init__`:**
```python
self.initial_heading: float = float(start_pose[2])
```

**2. Update `_fallback_goal_A()` to use initial heading:**
```python
# OLD: d = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
# NEW:
d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)], dtype=float)
```

#### Task 3 (`Task3.py`)

**1. Store initial heading in `__init__`:**
```python
self.initial_heading: float = float(start_pose[2])
```

**2. Update fallback goal (no gates detected) in `tick()`:**
```python
# OLD:
x, y, hdg = self.pose
step = 40.0
g = (float(x + step * np.cos(hdg)), float(y + step * np.sin(hdg)), 0.0)

# NEW:
x, y, _ = self.pose
hdg = self.initial_heading
step = 40.0
g = (float(x + step * np.cos(hdg)), float(y + step * np.sin(hdg)), 0.0)
```

**3. Update SEARCH_YELLOW phase in `tick()`:**
```python
# OLD:
x, y, hdg = self.pose
left = np.array([-np.sin(hdg), np.cos(hdg)], dtype=float)

# NEW:
x, y, _ = self.pose
hdg = self.initial_heading
left = np.array([-np.sin(hdg), np.cos(hdg)], dtype=float)
```

## Benefits

✅ **Consistent Course**: Boat maintains its original intended direction even with drift  
✅ **Predictable Behavior**: Forward goals always project along the mission's initial heading  
✅ **Better Search Patterns**: Task 3 yellow buoy search uses the original course direction  
✅ **Robust to Disturbances**: Current/wind-induced rotation doesn't affect goal placement  

## Gate Normal Priority

Note that `last_gate_normal` (if available) still takes priority over `initial_heading` in Tasks 1 and 2:

```python
if self.last_gate_normal is not None:
    d = self.last_gate_normal
else:
    d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)], dtype=float)
```

This ensures that after detecting gates, the boat follows the **channel direction** (gate perpendicular) rather than the arbitrary initial heading. This is correct behavior for channel navigation.

## Testing

To verify the fix works:

1. **Task 1**: Start boat at heading 0° (east), let it drift to 45°, verify fallback goal still points east
2. **Task 2**: Start boat heading north through channel, verify forward goals maintain northward direction even if boat yaws
3. **Task 3**: After passing gate, verify search pattern is based on original heading, not current orientation

## Related Files

- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task1.py`
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task2.py`
- `/home/lorenzo/Repos/School/autonomy-ws-25-26/planning/Global/Task3.py`
