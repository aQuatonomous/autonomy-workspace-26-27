#!/usr/bin/env python3
"""
Global Planner Task 1 (Only 2 gates)

What this module does (Task 1):
- Input: EntityList (types + positions) from JSON/perception
- Output: Updated EntityList with ORDERED goal waypoints: goal_wp1, goal_wp2, ...

Key rules implemented:
1) Robust goal ordering:
   - Build gate centers from paired (red_buoy, green_buoy)
   - Compute a "forward direction" vector:
       a) from boat heading if available, else
       b) from start -> nearest gate center
   - Sort gate centers by projection onto that forward direction

2) Gate completion:
   - A gate is "completed" only when the boat motion segment
     (prev_pos -> current_pos) intersects the gate segment (red -> green)

3) Fallback (no gate detected):
    - If no next valid (red_buoy, green_buoy) gate pair is detected:
    - Define the forward direction as the unit vector perpendicular to the
      most recently detected gate line (red_buoy → green_buoy)
    - Generate a temporary forward waypoint by projecting straight ahead
      along this direction, up to 20 m
    - Exit fallback immediately once the next gate is detected

One-shot mode:
- Compute and write goals once, optionally compute a path once, return RUNNING/SUCCESS.

Loop mode:
- Repeatedly update pose/perception, update goals, optionally plan,
  return SUCCESS once both gates are crossed in order.
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner

from Global.goal_utils import nudge_goal_away_from_obstacles, segments_intersect

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]  # x, y, heading (rad)


@dataclass
class GoalPlan:
    """Individual goal with completion tracking"""
    goal_id: str
    position: Vec3  # (x, y, heading_rad); Task1 uses heading=0
    completed: bool = False


from Global.types import DetectedEntity, Vec3


class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


# -------------------------
# Task 1 manager
# -------------------------

class Task1Manager:
    def __init__(self, entities, map_bounds, start_pose: Pose):
        self.entities = entities
        self.map_bounds = tuple(float(v) for v in map_bounds) if map_bounds else None

        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None
        self.initial_heading: float = float(start_pose[2])

        # Persistent goal plan with completion tracking
        self.goal_plan: List[GoalPlan] = []
        self.locked_gates: List[Tuple[Vec2, Vec2]] = []  # Gates locked once discovered
        self.gates_locked: bool = False
        
        # completion tracking
        self.next_gate_index: int = 0
        self.last_gate_normal: Optional[np.ndarray] = None
        # Task 1 Evacuation: entry (both gates) -> report -> exit (re-cross both gates in reverse)
        self.phase: str = "entry"  # "entry" | "exit"

        # outputs
        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        # prev_pos is updated at end of tick() so it is always "previous tick's position". Here only init if needed.
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        """Add detected entity by id (same id = same entity). Skip add_or_update if already in list (e.g. from fusion)."""
        already = any(getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities)
        if not already:
            self.entities.add_or_update(det.entity_type, det.position, det.entity_id,
                                        name=f"{det.entity_type}_{det.entity_id}")

    @staticmethod
    def _snap_gate(gate: Tuple[Vec2, Vec2]) -> Tuple[Vec2, Vec2]:
        """Round gate buoy positions to nearest integer metre to stabilise noisy lidar."""
        r, g = gate
        return (
            (round(float(r[0])), round(float(r[1])), float(r[2]) if len(r) > 2 else 0.0),
            (round(float(g[0])), round(float(g[1])), float(g[2]) if len(g) > 2 else 0.0),
        )

    def _lock_gates_once(self) -> None:
        """Lock gates as we see them (ahead of boat, across the channel). Snaps to integer metre."""
        boat_xy = (self.pose[0], self.pose[1])
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2], boat_pos=boat_xy)
        
        print(f"[TASK1 LOCK] Attempting gate lock: detected {len(gates)} gates, gates_locked={self.gates_locked}")
        
        if not gates:
            print(f"[TASK1 LOCK] No gates detected yet")
            return

        fwd = self._forward_dir()
        bx, by = self.pose[0], self.pose[1]

        def score_along_fwd(gate):
            r, g = gate
            cx = (float(r[0]) + float(g[0])) * 0.5
            cy = (float(r[1]) + float(g[1])) * 0.5
            return (cx - bx) * fwd[0] + (cy - by) * fwd[1]

        scored = sorted(enumerate(gates), key=lambda t: score_along_fwd(t[1]))

        if not self.gates_locked:
            idx, gate = scored[0]
            snapped = self._snap_gate(gate)
            center = ((snapped[0][0] + snapped[1][0]) / 2.0,
                      (snapped[0][1] + snapped[1][1]) / 2.0, 0.0)
            self.locked_gates = [snapped]
            self.gates_locked = True
            self.goal_plan = [GoalPlan(goal_id="gate_1", position=center, completed=False)]
            return

        if len(self.locked_gates) == 1 and len(scored) >= 2:
            idx, gate = scored[1]
            snapped = self._snap_gate(gate)
            center = ((snapped[0][0] + snapped[1][0]) / 2.0,
                      (snapped[0][1] + snapped[1][1]) / 2.0, 0.0)
            if snapped != self.locked_gates[0]:
                self.locked_gates.append(snapped)
                self.goal_plan.append(GoalPlan(goal_id="gate_2", position=center, completed=False))

    def publish_goals(self) -> None:
        """Publish only current active goals to EntityList as goal_wp*; nudge away from obstacles."""
        self.entities.clear_goals()
        obstacles = self._obstacles()
        from_pt = (self.pose[0], self.pose[1], self.pose[2])

        active_goals = []
        for goal in self.goal_plan:
            if not goal.completed:
                pos = nudge_goal_away_from_obstacles(goal.position, obstacles, from_pt)
                active_goals.append(pos)
                # Only publish the next goal, not all remaining goals
                break

        for i, pos in enumerate(active_goals, start=1):
            self.entities.add("goal", pos, name=f"goal_wp{i}")
            print(f"[TASK1 GOAL] Published goal_wp{i} at ({pos[0]:.2f}, {pos[1]:.2f})")

        self.goal_queue = active_goals
        print(f"[TASK1 GOAL] Total goals published: {len(active_goals)}, phase={self.phase}")

    def _forward_dir(self, centers: List[Vec2] = None) -> np.ndarray:
        """Forward direction from boat heading. Always uses heading."""
        hdg = float(self.pose[2])
        return np.array([np.cos(hdg), np.sin(hdg)], dtype=float)

    def _obstacles(self) -> List[Tuple[float, ...]]:
        """Obstacles = buoys + no-go walls. Gate buoys excluded by entity ID."""
        gate_ids = self.entities.get_gate_entity_ids(
            boat_heading_rad=self.pose[2],
            boat_pos=(self.pose[0], self.pose[1]),
        )
        return list(self.entities.get_obstacles(exclude_entity_ids=gate_ids)) + list(
            self.entities.get_no_go_obstacle_points(
                map_bounds=self.map_bounds,
                boat_heading_rad=self.pose[2],
            )
        )

    def _fallback_goal(self) -> Vec2:
        """Project forward along initial heading (or last gate normal) by 40m. No grid clipping."""
        x, y, hdg = self.pose
        if self.last_gate_normal is not None:
            d = self.last_gate_normal
        else:
            d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)], dtype=float)
            d = d / (np.linalg.norm(d) + 1e-9)

        step = 20.0
        p = np.array([x, y], dtype=float) + d * step
        desired = (float(p[0]), float(p[1]), 0.0)
        return nudge_goal_away_from_obstacles(desired, self._obstacles(), (x, y, hdg))

    def _check_goal_completion(self) -> None:
        """Check if current goal was reached and mark it complete."""
        if self.prev_pos is None or not self.goal_plan:
            return

        cur = (self.pose[0], self.pose[1])
        if abs(cur[0] - self.prev_pos[0]) < 1e-9 and abs(cur[1] - self.prev_pos[1]) < 1e-9:
            return

        current_goal = None
        goal_index = -1
        for i, goal in enumerate(self.goal_plan):
            if not goal.completed:
                current_goal = goal
                goal_index = i
                break

        if current_goal is None:
            return

        if goal_index < len(self.locked_gates):
            # Exit phase: goal_plan is [exit_gate_1=gate2, exit_gate_2=gate1] so index 0->locked_gates[1], 1->locked_gates[0]
            gate_index = (1, 0)[goal_index] if self.phase == "exit" else goal_index
            red, green = self.locked_gates[gate_index]
            if segments_intersect(self.prev_pos, cur, red, green):
                current_goal.completed = True
                self.next_gate_index = goal_index + 1
                # After both entry gates: switch to exit phase (re-cross gates in reverse order)
                if self.phase == "entry" and len(self.goal_plan) >= 2 and all(g.completed for g in self.goal_plan):
                    self.phase = "exit"
                    g1, g2 = self.locked_gates[0], self.locked_gates[1]
                    c2 = ((g2[0][0] + g2[1][0]) / 2.0, (g2[0][1] + g2[1][1]) / 2.0, 0.0)
                    c1 = ((g1[0][0] + g1[1][0]) / 2.0, (g1[0][1] + g1[1][1]) / 2.0, 0.0)
                    self.goal_plan = [
                        GoalPlan(goal_id="exit_gate_1", position=c2, completed=False),
                        GoalPlan(goal_id="exit_gate_2", position=c1, completed=False),
                    ]
                    self.next_gate_index = 0

    def tick(self) -> None:
        """Main update loop - lock gates, check completion, publish active goals."""
        self._lock_gates_once()
        self._check_goal_completion()

        boat_xy = (self.pose[0], self.pose[1])
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2], boat_pos=boat_xy)
        if gates:
            red, green = gates[0]
            along = np.array([green[0] - red[0], green[1] - red[1]], dtype=float)
            n = np.array([-along[1], along[0]], dtype=float)
            nnorm = np.linalg.norm(n) + 1e-9
            if nnorm > 1e-6:
                self.last_gate_normal = n / nnorm

        if not self.gates_locked:
            fallback_goal = self._fallback_goal()
            self.entities.clear_goals()
            self.entities.add("goal", fallback_goal, name="goal_wp1")
            self.goal_queue = [fallback_goal]
        else:
            self.publish_goals()
            if not self.goal_queue:
                fallback_goal = self._fallback_goal()
                self.entities.clear_goals()
                self.entities.add("goal", fallback_goal, name="goal_wp1")
                self.goal_queue = [fallback_goal]

        self.prev_pos = (self.pose[0], self.pose[1])

    def done(self) -> bool:
        """Return True when entry and exit phases are complete (both gates crossed twice)."""
        return (
            self.phase == "exit"
            and len(self.goal_plan) >= 2
            and all(goal.completed for goal in self.goal_plan)
        )

    def entry_gates_completed(self) -> bool:
        """True once both entry gates have been crossed (used to trigger gate_pass 'start')."""
        return self.phase == "exit"

    def exit_gates_completed(self) -> bool:
        """True when both exit gates have been crossed (used to trigger gate_pass 'end')."""
        return self.done()

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 1) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = self._obstacles()
        gates = self.locked_gates if self.gates_locked else self.entities.get_gates(
            boat_heading_rad=self.pose[2], boat_pos=start,
        )
        goals = self.goal_queue[:max_goals]

        goals_xy = [(g[0], g[1]) for g in goals]
        out = planner.plan_multi_goal_path(start, goals_xy, obstacles, gates=gates, map_bounds=self.map_bounds)
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]


def run_task1(
    entities,
    map_bounds: Optional[Vec2],
    *,
    get_pose: Optional[Callable[[], Pose]] = None,
    get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
    plan: bool = True,
) -> Dict:
    """
    Run one shot: one tick (update pose/detections, tick, optionally plan), then return RUNNING/SUCCESS.
    Caller repeats run_task1 until done (e.g. from a ROS timer).
    """
    start = entities.get_start()
    mgr = Task1Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))  # map_bounds can be None
    planner = PotentialFieldsPlanner(resolution=0.5)

    if get_pose is not None:
        x, y, hdg = get_pose()
        mgr.update_pose(x, y, hdg)
    if get_detections is not None:
        for det in get_detections():
            mgr.add_detected(det)
    mgr.tick()
    if plan:
        mgr.plan(planner, max_goals=1)

    return {
        "status": TaskStatus.SUCCESS.value if mgr.done() else TaskStatus.RUNNING.value,
        "goals": list(mgr.goal_queue),
        "next_gate_index": mgr.next_gate_index,
        "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
        "velocities": mgr.current_velocities,
        "speeds": mgr.current_speeds,
        "entities": mgr.entities,
    }

