#!/usr/bin/env python3
"""
TaskMaster - Sequential state machine for Task1 -> Task2 -> Task3 with explicit transitions

State Machine:
- RUN_TASK_1
- TRANSITION_1_TO_2 (forward nudge until gate detected or distance target)
- RUN_TASK_2
- TRANSITION_2_TO_3 (turn +45° east, then forward search for gate)
- RUN_TASK_3
- ALL_DONE

Each transition has explicit entry actions, behavior, and exit conditions.
"""

from __future__ import annotations
import math
import time
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

from Global.types import DetectedEntity
from Global.TaskTest import TaskTestManager
from Global.Task1 import Task1Manager
from Global.Task2 import Task2Manager
from Global.Task3 import Task3Manager
from Global.Task4 import Task4Manager
from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]


class TaskMasterState(Enum):
    """Top-level state machine states"""
    RUN_TASK_1 = "RUN_TASK_1"
    TRANSITION_1_TO_2 = "TRANSITION_1_TO_2"
    RUN_TASK_2 = "RUN_TASK_2"
    TRANSITION_2_TO_3 = "TRANSITION_2_TO_3"
    RUN_TASK_3 = "RUN_TASK_3"
    ALL_DONE = "ALL_DONE"


class TransitionPhase(Enum):
    """Sub-phases for TRANSITION_2_TO_3"""
    TURN_TO_WEST_30 = "TURN_TO_WEST_30"
    FORWARD_SEARCH_FOR_TASK3_GATE = "FORWARD_SEARCH_FOR_TASK3_GATE"


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]"""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def angle_diff(a: float, b: float) -> float:
    """Compute smallest angle difference between two angles (rad)"""
    diff = wrap_to_pi(a - b)
    return diff


class TaskMaster:
    """Sequential task state machine with explicit transitions"""
    
    def __init__(
        self, 
        *, 
        entities, 
        start: Vec2, 
        task_id: int, 
        map_bounds: Optional[Tuple[float, float, float, float]] = None
    ):
        """
        Initialize TaskMaster for sequential task execution.
        
        Args:
            entities: EntityList instance
            start: (x, y, heading) start position
            task_id: Task ID (123 for sequential Task1->Task2->Task3)
            map_bounds: Optional (min_x, min_y, max_x, max_y) for boundary enforcement
        """
        self.entities = entities
        self.map_bounds = map_bounds
        self.planner = PotentialFieldsPlanner(resolution=0.5)
        
        if hasattr(self.entities, "set_start_position"):
            self.entities.set_start_position(start)
        else:
            self.entities.start_position = start
        
        h = float(start[2]) if len(start) >= 3 else 0.0
        self.start_pose = (float(start[0]), float(start[1]), h)
        
        # Current state
        self.state = TaskMasterState.RUN_TASK_1
        self.current_pose: Pose = self.start_pose
        self.task_id = int(task_id)
        
        # Task managers (created on demand)
        self.task1_mgr: Optional[Task1Manager] = None
        self.task2_mgr: Optional[Task2Manager] = None
        self.task3_mgr: Optional[Task3Manager] = None
        
        # Transition 1->2 state
        self.t12_start_pose: Optional[Pose] = None
        self.t12_forward_heading: float = 0.0
        self.t12_distance_target_m: float = 1.5
        self.t12_timeout_s: float = 10.0
        self.t12_entry_time: float = 0.0
        
        # Transition 2->3 state
        self.t23_start_pose: Optional[Pose] = None
        self.t23_target_heading: float = 0.0
        self.t23_turn_tolerance_deg: float = 5.0
        self.t23_turn_timeout_s: float = 8.0
        self.t23_search_timeout_s: float = 60.0  # Longer search - goes straight for a while
        self.t23_entry_time: float = 0.0
        self.t23_phase: Optional[TransitionPhase] = None
        self.t23_turn_end_time: float = 0.0
        self.task3_initial_heading: Optional[float] = None
        
        # Logging throttle
        self._last_log_time: float = 0.0
        self._log_throttle_s: float = 0.5
        
        # Initialize based on task_id
        if task_id == 123:
            # Sequential mode: start with Task 1
            self._init_task1()
        elif task_id == 0:
            self.state = TaskMasterState.RUN_TASK_1
            self.task1_mgr = TaskTestManager(entities, map_bounds, self.start_pose)
        elif task_id == 1:
            self.state = TaskMasterState.RUN_TASK_1
            self._init_task1()
        elif task_id == 2:
            self.state = TaskMasterState.RUN_TASK_2
            self._init_task2(self.start_pose)
        elif task_id == 3:
            self.state = TaskMasterState.RUN_TASK_3
            self._init_task3(self.start_pose)
        elif task_id == 4:
            self.state = TaskMasterState.RUN_TASK_1
            self.task1_mgr = Task4Manager(entities, map_bounds=None, start_pose=self.start_pose)
        else:
            raise ValueError(f"Unknown task_id: {task_id}")
    
    def _init_task1(self):
        """Initialize Task 1 manager"""
        print(f"[TaskMaster] Initializing TASK 1 at pose {self.start_pose}")
        self.task1_mgr = Task1Manager(self.entities, self.map_bounds, self.start_pose)
        self.state = TaskMasterState.RUN_TASK_1
    
    def _init_task2(self, start_pose: Pose):
        """Initialize Task 2 manager with specified start pose"""
        print(f"[TaskMaster] Initializing TASK 2 at pose {start_pose}")
        self.task2_mgr = Task2Manager(self.entities, self.map_bounds, start_pose)
        self.state = TaskMasterState.RUN_TASK_2
    
    def _init_task3(self, start_pose: Pose):
        """Initialize Task 3 manager with specified start pose"""
        print(f"[TaskMaster] Initializing TASK 3 at pose {start_pose}")
        self.task3_mgr = Task3Manager(self.entities, self.map_bounds, start_pose)
        self.state = TaskMasterState.RUN_TASK_3
    
    def _log_throttled(self, message: str, force: bool = False):
        """Log message with throttling"""
        now = time.time()
        if force or (now - self._last_log_time) >= self._log_throttle_s:
            print(f"[TaskMaster] {message}")
            self._last_log_time = now
    
    def _get_buoy_gates_only(
        self,
        boat_heading_rad: Optional[float] = None,
        boat_pos: Optional[Tuple[float, float]] = None,
    ) -> List[Tuple[Vec2, Vec2]]:
        """Get gates using ONLY buoys (no poles) with left/right pairing"""
        # Filter entities to only buoys
        red_buoys = [e for e in self.entities.entities if e.type == "red_buoy"]
        green_buoys = [e for e in self.entities.entities if e.type == "green_buoy"]
        
        if not red_buoys or not green_buoys:
            return []
        
        pairs = []
        max_gate_width = 15.0
        used_green = set()
        
        if boat_heading_rad is None:
            # Fallback: nearest pairing
            for red in red_buoys:
                for green in green_buoys:
                    if id(green) in used_green:
                        continue
                    dist = math.hypot(
                        red.position[0] - green.position[0],
                        red.position[1] - green.position[1]
                    )
                    if dist <= max_gate_width:
                        pairs.append((red.position, green.position))
                        used_green.add(id(green))
                        break
        else:
            # Use left/right rule: red LEFT, green RIGHT relative to heading
            fwd_x = math.cos(boat_heading_rad)
            fwd_y = math.sin(boat_heading_rad)
            left_x = -math.sin(boat_heading_rad)
            left_y = math.cos(boat_heading_rad)
            
            if boat_pos is not None:
                bx, by = float(boat_pos[0]), float(boat_pos[1])
            else:
                s = self.entities.get_start()
                bx, by = float(s[0]), float(s[1])
            
            for red in red_buoys:
                for green in green_buoys:
                    if id(green) in used_green:
                        continue
                    
                    # Check distance
                    dist = math.hypot(
                        red.position[0] - green.position[0],
                        red.position[1] - green.position[1]
                    )
                    if dist > max_gate_width:
                        continue
                    
                    # Check red is LEFT, green is RIGHT
                    red_to_boat_x = red.position[0] - bx
                    red_to_boat_y = red.position[1] - by
                    green_to_boat_x = green.position[0] - bx
                    green_to_boat_y = green.position[1] - by
                    
                    red_cross = red_to_boat_x * left_x + red_to_boat_y * left_y
                    green_cross = green_to_boat_x * (-left_x) + green_to_boat_y * (-left_y)
                    
                    if red_cross > 0 and green_cross > 0:
                        # Check gate center is AHEAD
                        cx = (red.position[0] + green.position[0]) * 0.5
                        cy = (red.position[1] + green.position[1]) * 0.5
                        ahead = (cx - bx) * fwd_x + (cy - by) * fwd_y
                        
                        if ahead > 0:
                            pairs.append((red.position, green.position))
                            used_green.add(id(green))
                            break
        
        return pairs
    
    def _get_pole_gates_only(
        self,
        boat_heading_rad: Optional[float] = None,
        boat_pos: Optional[Tuple[float, float]] = None,
    ) -> List[Tuple[Vec2, Vec2]]:
        """Get gates using ONLY poles (no buoys) with left/right pairing"""
        red_poles = [e for e in self.entities.entities if e.type == "red_pole_buoy"]
        green_poles = [e for e in self.entities.entities if e.type == "green_pole_buoy"]
        
        if not red_poles or not green_poles:
            return []
        
        pairs = []
        max_gate_width = 15.0
        used_green = set()
        
        if boat_heading_rad is None:
            # Fallback: nearest pairing
            for red in red_poles:
                for green in green_poles:
                    if id(green) in used_green:
                        continue
                    dist = math.hypot(
                        red.position[0] - green.position[0],
                        red.position[1] - green.position[1]
                    )
                    if dist <= max_gate_width:
                        pairs.append((red.position, green.position))
                        used_green.add(id(green))
                        break
        else:
            # Use left/right rule
            fwd_x = math.cos(boat_heading_rad)
            fwd_y = math.sin(boat_heading_rad)
            left_x = -math.sin(boat_heading_rad)
            left_y = math.cos(boat_heading_rad)
            
            if boat_pos is not None:
                bx, by = float(boat_pos[0]), float(boat_pos[1])
            else:
                s = self.entities.get_start()
                bx, by = float(s[0]), float(s[1])
            
            for red in red_poles:
                for green in green_poles:
                    if id(green) in used_green:
                        continue
                    
                    dist = math.hypot(
                        red.position[0] - green.position[0],
                        red.position[1] - green.position[1]
                    )
                    if dist > max_gate_width:
                        continue
                    
                    red_to_boat_x = red.position[0] - bx
                    red_to_boat_y = red.position[1] - by
                    green_to_boat_x = green.position[0] - bx
                    green_to_boat_y = green.position[1] - by
                    
                    red_cross = red_to_boat_x * left_x + red_to_boat_y * left_y
                    green_cross = green_to_boat_x * (-left_x) + green_to_boat_y * (-left_y)
                    
                    if red_cross > 0 and green_cross > 0:
                        cx = (red.position[0] + green.position[0]) * 0.5
                        cy = (red.position[1] + green.position[1]) * 0.5
                        ahead = (cx - bx) * fwd_x + (cy - by) * fwd_y
                        
                        if ahead > 0:
                            pairs.append((red.position, green.position))
                            used_green.add(id(green))
                            break
        
        return pairs
    
    def _enter_transition_1_to_2(self):
        """ENTRY ACTIONS for TRANSITION_1_TO_2"""
        self._log_throttled(
            f"=== DONE TASK 1 -> TRANSITION_1_TO_2 ===\n"
            f"  Pose: {self.current_pose}\n"
            f"  Current heading: {math.degrees(self.current_pose[2]):.1f}°\n"
            f"  Task1 initial_heading: {math.degrees(self.task1_mgr.initial_heading):.1f}°",
            force=True
        )
        
        # Freeze Task 1 (stop publishing goals)
        if self.task1_mgr is not None:
            self.task1_mgr.goal_queue = []
            self.entities.clear_goals()
        
        # Initialize transition context
        self.t12_start_pose = self.current_pose
        self.t12_forward_heading = self.current_pose[2]  # Use current heading as forward direction
        self.t12_entry_time = time.time()
        
        self.state = TaskMasterState.TRANSITION_1_TO_2
        self._log_throttled(
            f"  Transition params: target_dist={self.t12_distance_target_m}m, "
            f"forward_hdg={math.degrees(self.t12_forward_heading):.1f}°, "
            f"timeout={self.t12_timeout_s}s",
            force=True
        )
    
    def _tick_transition_1_to_2(self):
        """BEHAVIOR for TRANSITION_1_TO_2: forward nudge until gate or distance"""
        # Calculate distance moved
        dx = self.current_pose[0] - self.t12_start_pose[0]
        dy = self.current_pose[1] - self.t12_start_pose[1]
        dist_moved = math.hypot(dx, dy)
        
        # Detect Task 2 buoy gates (NOT poles)
        boat_pos = (self.current_pose[0], self.current_pose[1])
        task2_gates = self._get_buoy_gates_only(
            boat_heading_rad=self.t12_forward_heading,
            boat_pos=boat_pos
        )
        
        # Check exit conditions
        elapsed = time.time() - self.t12_entry_time
        cond_distance = dist_moved >= self.t12_distance_target_m
        cond_gate = len(task2_gates) > 0
        cond_timeout = elapsed >= self.t12_timeout_s
        
        # Throttled logging
        self._log_throttled(
            f"TRANSITION_1_TO_2: dist={dist_moved:.2f}m/{self.t12_distance_target_m}m, "
            f"gates={len(task2_gates)}, elapsed={elapsed:.1f}s/{self.t12_timeout_s}s, "
            f"exit={cond_distance or cond_gate or cond_timeout}"
        )
        
        # Exit condition
        if cond_distance or cond_gate or cond_timeout:
            self._exit_transition_1_to_2(
                dist_moved=dist_moved,
                gate_detected=cond_gate,
                elapsed=elapsed
            )
            return
        
        # Publish forward goal (NOT nudged, respects map bounds)
        x = self.t12_start_pose[0] + self.t12_distance_target_m * math.cos(self.t12_forward_heading)
        y = self.t12_start_pose[1] + self.t12_distance_target_m * math.sin(self.t12_forward_heading)
        
        # Clamp to map bounds if provided
        if self.map_bounds is not None:
            min_x, min_y, max_x, max_y = self.map_bounds
            x = max(min_x + 1.0, min(max_x - 1.0, x))
            y = max(min_y + 1.0, min(max_y - 1.0, y))
        
        self.entities.clear_goals()
        self.entities.add("goal", (x, y, 0.0), name="goal_wp1")
    
    def _exit_transition_1_to_2(self, dist_moved: float, gate_detected: bool, elapsed: float):
        """EXIT ACTIONS for TRANSITION_1_TO_2"""
        self._log_throttled(
            f"=== TRANSITION_1_TO_2 COMPLETE -> STARTING TASK 2 ===\n"
            f"  Distance moved: {dist_moved:.2f}m\n"
            f"  Gate detected: {gate_detected}\n"
            f"  Time spent: {elapsed:.1f}s",
            force=True
        )
        
        # Create Task 2 with initial_heading from transition start
        task2_start_pose = (
            self.current_pose[0],
            self.current_pose[1],
            self.t12_forward_heading  # Use heading from transition start, not current drift
        )
        self._init_task2(task2_start_pose)
    
    def _enter_transition_2_to_3(self):
        """ENTRY ACTIONS for TRANSITION_2_TO_3"""
        self._log_throttled(
            f"=== DONE TASK 2 -> TRANSITION_2_TO_3 ===\n"
            f"  Pose: {self.current_pose}\n"
            f"  Current heading: {math.degrees(self.current_pose[2]):.1f}°",
            force=True
        )
        
        # Freeze Task 2
        if self.task2_mgr is not None:
            self.task2_mgr.goal_queue = []
            self.entities.clear_goals()
        
        # Transition context: -30° WEST turn
        self.t23_start_pose = self.current_pose
        self.t23_target_heading = wrap_to_pi(self.current_pose[2] - math.pi / 6.0)  # -30° = -π/6
        self.t23_entry_time = time.time()
        self.t23_phase = TransitionPhase.TURN_TO_WEST_30
        self.task3_initial_heading = None
        
        self.state = TaskMasterState.TRANSITION_2_TO_3
        self._log_throttled(
            f"  Turn target: {math.degrees(self.t23_target_heading):.1f}° "
            f"(-30° WEST from {math.degrees(self.t23_start_pose[2]):.1f}°)\n"
            f"  Turn tolerance: {self.t23_turn_tolerance_deg}°\n"
            f"  Search timeout: {self.t23_search_timeout_s}s (long straight run)",
            force=True
        )
    
    def _tick_transition_2_to_3(self):
        """BEHAVIOR for TRANSITION_2_TO_3: turn -30° west, then forward search"""
        if self.t23_phase == TransitionPhase.TURN_TO_WEST_30:
            self._tick_t23_turn()
        elif self.t23_phase == TransitionPhase.FORWARD_SEARCH_FOR_TASK3_GATE:
            self._tick_t23_search()
    
    def _tick_t23_turn(self):
        """Phase A: TURN_TO_WEST_30"""
        elapsed = time.time() - self.t23_entry_time
        heading_error_rad = abs(angle_diff(self.current_pose[2], self.t23_target_heading))
        heading_error_deg = math.degrees(heading_error_rad)
        
        turn_complete = heading_error_deg <= self.t23_turn_tolerance_deg
        turn_timeout = elapsed >= self.t23_turn_timeout_s
        
        self._log_throttled(
            f"TRANSITION_2_TO_3 [TURN]: current={math.degrees(self.current_pose[2]):.1f}°, "
            f"target={math.degrees(self.t23_target_heading):.1f}°, "
            f"error={heading_error_deg:.1f}°, elapsed={elapsed:.1f}s/{self.t23_turn_timeout_s}s"
        )
        
        if turn_complete or turn_timeout:
            # Set Task3 initial heading
            if turn_complete:
                self.task3_initial_heading = self.t23_target_heading
                result = "SUCCESS"
            else:
                self.task3_initial_heading = self.current_pose[2]
                result = "TIMEOUT"
            
            self._log_throttled(
                f"TURN phase {result}: Task3 initial_heading = {math.degrees(self.task3_initial_heading):.1f}°",
                force=True
            )
            
            # Switch to forward search phase
            self.t23_phase = TransitionPhase.FORWARD_SEARCH_FOR_TASK3_GATE
            self.t23_turn_end_time = time.time()
            return
        
        # Publish turn goal (small offset to force rotation)
        # Since we can only publish position goals, create a waypoint that encourages turning
        turn_radius = 2.0  # Small radius for in-place turn
        goal_x = self.current_pose[0] + turn_radius * math.cos(self.t23_target_heading)
        goal_y = self.current_pose[1] + turn_radius * math.sin(self.t23_target_heading)
        
        self.entities.clear_goals()
        self.entities.add("goal", (goal_x, goal_y, 0.0), name="goal_wp1")
    
    def _tick_t23_search(self):
        """Phase B: FORWARD_SEARCH_FOR_TASK3_GATE"""
        elapsed_total = time.time() - self.t23_entry_time
        elapsed_search = time.time() - self.t23_turn_end_time
        
        # Detect Task 3 buoy gates (NOT poles)
        boat_pos = (self.current_pose[0], self.current_pose[1])
        task3_gates = self._get_buoy_gates_only(
            boat_heading_rad=self.task3_initial_heading,
            boat_pos=boat_pos
        )
        
        gate_detected = len(task3_gates) > 0
        search_timeout = elapsed_search >= self.t23_search_timeout_s
        
        self._log_throttled(
            f"TRANSITION_2_TO_3 [SEARCH]: heading={math.degrees(self.task3_initial_heading):.1f}°, "
            f"gates={len(task3_gates)}, elapsed_search={elapsed_search:.1f}s/{self.t23_search_timeout_s}s"
        )
        
        if gate_detected or search_timeout:
            self._exit_transition_2_to_3(
                turn_succeeded=(abs(angle_diff(self.task3_initial_heading, self.t23_target_heading)) < math.radians(10)),
                gate_detected=gate_detected,
                elapsed_total=elapsed_total
            )
            return
        
        # Publish forward search goal (longer distance for straight run)
        search_dist = 40.0  # meters ahead - longer for straight approach to Task 3
        goal_x = self.current_pose[0] + search_dist * math.cos(self.task3_initial_heading)
        goal_y = self.current_pose[1] + search_dist * math.sin(self.task3_initial_heading)
        
        # Clamp to map bounds
        if self.map_bounds is not None:
            min_x, min_y, max_x, max_y = self.map_bounds
            goal_x = max(min_x + 1.0, min(max_x - 1.0, goal_x))
            goal_y = max(min_y + 1.0, min(max_y - 1.0, goal_y))
        
        self.entities.clear_goals()
        self.entities.add("goal", (goal_x, goal_y, 0.0), name="goal_wp1")
    
    def _exit_transition_2_to_3(self, turn_succeeded: bool, gate_detected: bool, elapsed_total: float):
        """EXIT ACTIONS for TRANSITION_2_TO_3"""
        self._log_throttled(
            f"=== TRANSITION_2_TO_3 COMPLETE -> STARTING TASK 3 ===\n"
            f"  Turn succeeded: {turn_succeeded}\n"
            f"  Final heading: {math.degrees(self.task3_initial_heading):.1f}°\n"
            f"  Gate detected: {gate_detected}\n"
            f"  Time spent: {elapsed_total:.1f}s",
            force=True
        )
        
        # Create Task 3 with the heading after the turn
        task3_start_pose = (
            self.current_pose[0],
            self.current_pose[1],
            self.task3_initial_heading
        )
        self._init_task3(task3_start_pose)
    
    def run_one_shot(
        self,
        get_pose: Optional[Callable[[], Pose]] = None,
        get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
        use_planning: bool = True,
        map_bounds: Optional[Tuple[float, float, float, float]] = None,
    ) -> Dict:
        """
        Run one planning tick: update pose/detections, execute state machine.
        
        Args:
            get_pose: Function to get current (x, y, heading)
            get_detections: Function to get detected entities
            use_planning: Whether to run potential fields planning
            map_bounds: Optional (min_x, min_y, max_x, max_y) boundary
        
        Returns:
            Dict with status, entities, path, velocities, speeds, phase, etc.
        """
        # Update map bounds if provided
        if map_bounds is not None:
            self.map_bounds = tuple(float(v) for v in map_bounds)
        
        # Update pose
        if get_pose is not None:
            pose = get_pose()
            self.current_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        
        # Update detections
        if get_detections is not None:
            for det in get_detections():
                # Add to entities (used by all tasks and transitions)
                if hasattr(self.entities, "add_or_update"):
                    self.entities.add_or_update(
                        det.entity_type,
                        det.position,
                        det.entity_id,
                        name=f"{det.entity_type}_{det.entity_id}"
                    )
        
        # State machine execution
        if self.state == TaskMasterState.RUN_TASK_1:
            self._tick_task1()
        
        elif self.state == TaskMasterState.TRANSITION_1_TO_2:
            self._tick_transition_1_to_2()
        
        elif self.state == TaskMasterState.RUN_TASK_2:
            self._tick_task2()
        
        elif self.state == TaskMasterState.TRANSITION_2_TO_3:
            self._tick_transition_2_to_3()
        
        elif self.state == TaskMasterState.RUN_TASK_3:
            self._tick_task3()
        
        elif self.state == TaskMasterState.ALL_DONE:
            pass
        
        # Get current manager
        manager = self._get_active_manager()
        
        # Run planning if requested and manager exists
        if use_planning and manager is not None:
            try:
                manager.plan(self.planner)
            except Exception as e:
                print(f"[TaskMaster] Planning warning: {e}")
        
        # Build result
        result = {
            "status": "SUCCESS" if self.state == TaskMasterState.ALL_DONE else "RUNNING",
            "state": self.state.value,
            "entities": self.entities,
            "path": manager.current_path if manager else [],
            "velocities": manager.current_velocities if manager else [],
            "speeds": manager.current_speeds if manager else [],
        }
        
        if manager and hasattr(manager, "phase"):
            p = manager.phase
            result["phase"] = p.value if hasattr(p, "value") else p
        
        if manager and hasattr(manager, "report") and hasattr(manager.report, "to_dict"):
            result["report"] = manager.report.to_dict()
        
        if self.state == TaskMasterState.TRANSITION_2_TO_3 and self.t23_phase:
            result["transition_phase"] = self.t23_phase.value
        
        return result
    
    def _get_active_manager(self):
        """Get the currently active task manager"""
        if self.state == TaskMasterState.RUN_TASK_1:
            return self.task1_mgr
        elif self.state == TaskMasterState.RUN_TASK_2:
            return self.task2_mgr
        elif self.state == TaskMasterState.RUN_TASK_3:
            return self.task3_mgr
        return None
    
    def _tick_task1(self):
        """Execute Task 1 tick and check for completion"""
        if self.task1_mgr is None:
            return
        
        # Update pose
        self.task1_mgr.update_pose(
            self.current_pose[0],
            self.current_pose[1],
            self.current_pose[2]
        )
        
        # Tick task
        self.task1_mgr.tick()
        
        # Check completion
        if self.task1_mgr.done():
            if self.task_id == 123:
                # Sequential mode: transition to Task 2
                self._enter_transition_1_to_2()
            else:
                # Single task mode: done
                self.state = TaskMasterState.ALL_DONE
                self._log_throttled("=== TASK 1 COMPLETE ===", force=True)
    
    def _tick_task2(self):
        """Execute Task 2 tick and check for completion"""
        if self.task2_mgr is None:
            return
        
        # Update pose
        self.task2_mgr.update_pose(
            self.current_pose[0],
            self.current_pose[1],
            self.current_pose[2]
        )
        
        # Tick task
        self.task2_mgr.tick()
        
        # Check completion
        if self.task2_mgr.done():
            if self.task_id == 123:
                # Sequential mode: transition to Task 3
                self._enter_transition_2_to_3()
            else:
                # Single task mode: done
                self.state = TaskMasterState.ALL_DONE
                self._log_throttled("=== TASK 2 COMPLETE ===", force=True)
    
    def _tick_task3(self):
        """Execute Task 3 tick and check for completion"""
        if self.task3_mgr is None:
            return
        
        # Update pose
        self.task3_mgr.update_pose(
            self.current_pose[0],
            self.current_pose[1],
            self.current_pose[2]
        )
        
        # Tick task
        self.task3_mgr.tick()
        
        # Check completion
        if self.task3_mgr.done():
            self.state = TaskMasterState.ALL_DONE
            self._log_throttled("=== TASK 3 COMPLETE -> ALL TASKS DONE ===", force=True)
