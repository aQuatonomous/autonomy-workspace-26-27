#!/usr/bin/env python3
"""Global Planner Task 2: Debris Clearance goal/phase manager + optional planning.

Location: src/Planner/Global/

What this module does (Task 2):
- Input: EntityList (types + positions) from JSON/perception
- Output: Updated EntityList with ORDERED task goals as goal_wp1, goal_wp2, ...

Task 2 behavior:
Phase A: TRANSIT_OUT
  - Calculate channel gate centers from red/green Buoy pairs.
  - If no gates known, set forward search goal.
  - Navigate gate centers in forward order while avoiding debris(channel).
  - Save gate centers for phase C return.
Phase B: DEBRIS_FIELD
  - Navigate into debris field straight until a green indicator (survivor) is seen.
  - Avoid hazards (red_indicator) + black buoys (black_buoy).
  - Report locations of all indicators + buoys seen.
  - For each survivor (green_indicator): "circle" (represented as circle waypoints).
    - Create 4 waypoints in a circle around survivor position that doesnt collide with obstacles.
  - Record/report positions (stored in returned dict; writing files is not done here).
Phase C: RETURN
  - Navigate back through the channel in reverse to start using waypoints saved in Phase A.

3) Fallback (no gate detected):
    - If no next valid (red_buoy, green_buoy) gate pair is detected for Part A ONLY:
    - Define the forward direction as the unit vector perpendicular to the
      most recently detected gate line (red_buoy → green_buoy)
    - Generate a temporary forward waypoint by projecting straight ahead
      along this direction, up to 30 m
    - Continue advancing and re-checking perception each update
    - Exit fallback immediately once the next gate is detected and
      resume normal gate ordering logic

Connections (kept):
- Called by: TaskMaster.py (uses Task2Manager via TaskMaster)
- Calls/Imports:
  - potential_fields_planner.py -> PotentialFieldsPlanner
  - Global.entities -> EntityList (passed in)
- Standard libs: numpy, os, sys, time, enum, json
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple, Set
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner

from Global.goal_utils import nudge_goal_away_from_obstacles

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]


@dataclass
class GoalPlan:
    """Individual goal with completion tracking"""
    goal_id: str
    position: Vec3  # (x, y, heading_rad); Task2 uses heading=0
    completed: bool = False


from Global.types import DetectedEntity, Vec3


class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class Phase(Enum):
    TRANSIT_OUT = "TRANSIT_OUT"
    DEBRIS_FIELD = "DEBRIS_FIELD"
    RETURN = "RETURN"


class Report:
    """In-memory report for Phase B. Each detection reported separately; key by entity_id when present (from /fused_buoys), else by position so distinct detections stay distinct."""
    def __init__(self):
        self.green_indicators: Dict[str, Vec3] = {}
        self.red_indicators: Dict[str, Vec3] = {}
        self.black_buoys: Dict[str, Vec3] = {}

    def _key(self, prefix: str, det: DetectedEntity) -> str:
        """Stable key: use entity_id when fusion provides it; else position so each distinct detection is separate."""
        if det.entity_id is not None and det.entity_id != 0:
            return f"{prefix}_{det.entity_id}"
        x, y = det.position[0], det.position[1]
        return f"{prefix}_{round(x, 2)}_{round(y, 2)}"

    def add(self, det: DetectedEntity):
        if det.entity_type == "green_indicator":
            self.green_indicators[self._key("green", det)] = det.position
        elif det.entity_type == "red_indicator":
            self.red_indicators[self._key("red", det)] = det.position
        elif det.entity_type == "black_buoy":
            self.black_buoys[self._key("black", det)] = det.position

    def to_dict(self) -> Dict:
        def pack(d):
            return [{"name": k, "x": float(v[0]), "y": float(v[1])} for k, v in d.items()]
        return {
            "green_indicators": pack(self.green_indicators),
            "red_indicators": pack(self.red_indicators),
            "black_buoys": pack(self.black_buoys),
            "counts": {
                "green": len(self.green_indicators),
                "red": len(self.red_indicators),
                "black": len(self.black_buoys),
            },
        }


class Task2Manager:
    def __init__(self, entities, map_bounds, start_pose: Pose):
        self.entities = entities
        self.map_bounds = tuple(float(v) for v in map_bounds) if map_bounds else None
        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose
        self.initial_heading: float = float(start_pose[2])

        # Persistent phase state
        self.phase = Phase.TRANSIT_OUT
        self.report = Report()

        # Persistent goal plan with completion tracking
        self.goal_plan: List[GoalPlan] = []
        
        # Phase A: 4 gate centers (for logic) + full waypoint list (gate centers + interpolated) for outbound and return
        self.channel_centers_out: List[Vec3] = []
        self.channel_waypoints_out: List[Vec3] = []  # full list: gate centers + points between, for return
        self.channel_locked: bool = False
        self.task2_start_position: Optional[Vec3] = None  # first gate center when task begun (return here)
        self.last_gate_normal: Optional[np.ndarray] = None

        # Phase B: survivor handling with persistent tracking
        self.handled_survivors: Set[str] = set()  # survivor IDs that were already circled
        self.current_survivor_plan: List[GoalPlan] = []  # 4-point circle plan for current survivor
        self._circling_survivor_entity_id: Optional[int] = None  # exclude from obstacles when circling

        # Phase C: return waypoints (reverse channel + start)
        self.return_waypoints: List[Vec3] = []

        # goal output
        self.goal_queue: List[Vec3] = []

        # completion / tunables
        self.goal_reached_dist = 2.0
        self.mission_complete = False

        # optional path
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def _obstacles(self) -> List[Vec2]:
        """Obstacles = buoys; in TRANSIT_OUT and RETURN also add no-go walls.
        Gate entrance buoys excluded by entity ID (same as Task 1).
        When circling a survivor (DEBRIS_FIELD), exclude that green_indicator from obstacles."""
        in_channel = self.phase != Phase.DEBRIS_FIELD
        exclude_ids = self.entities.get_gate_entity_ids(
            boat_heading_rad=self.pose[2],
            boat_pos=(self.pose[0], self.pose[1]),
        )
        if self.phase == Phase.DEBRIS_FIELD and self._circling_survivor_entity_id is not None:
            exclude_ids = exclude_ids | {self._circling_survivor_entity_id}
        obs = list(self.entities.get_obstacles(exclude_entity_ids=exclude_ids))
        if in_channel:
            obs.extend(self.entities.get_no_go_obstacle_points(
                map_bounds=self.map_bounds, boat_heading_rad=self.pose[2],
            ))
        return obs

    def update_pose(self, x: float, y: float, heading: float):
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity):
        """Add detected entity by id (same id = same entity). Skip add_or_update if already in list (e.g. from fusion)."""
        already = any(getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities)
        if not already:
            self.entities.add_or_update(det.entity_type, det.position, det.entity_id,
                                        name=f"{det.entity_type}_{det.entity_id}")
        # Always add to report for Phase B
        self.report.add(det)

    def publish_goals(self) -> None:
        """Publish only the next waypoint (one at a time); nudge away from obstacles."""
        self.entities.clear_goals()
        obstacles = self._obstacles()
        from_pt = (self.pose[0], self.pose[1], self.pose[2])

        active_goals = []
        for goal in self.goal_plan:
            if not goal.completed:
                pos = nudge_goal_away_from_obstacles(goal.position, obstacles, from_pt)
                active_goals.append(pos)
                break  # One waypoint at a time until we reach end

        for i, pos in enumerate(active_goals, start=1):
            self.entities.add("goal", pos, name=f"goal_wp{i}")

        self.goal_queue = active_goals

    def _publish_single_goal(self, goal: Vec3) -> None:
        """Publish one goal as goal_wp1 and set goal_queue (used for fallback and forward-in-debris)."""
        self.entities.clear_goals()
        self.entities.add("goal", goal, name="goal_wp1")
        self.goal_queue = [goal]

    def _get_unhandled_survivors(self) -> List[Tuple[str, Vec3, Optional[int]]]:
        """Return list of (survivor_id, position, entity_id) for green indicators not yet in handled_survivors."""
        green_ents = self.entities.get_by_type("green_indicator")
        def _sid(e):
            return f"green_{e.entity_id}" if e.entity_id is not None else f"green_{id(e)}"
        return [(_sid(e), e.position, e.entity_id) for e in green_ents if _sid(e) not in self.handled_survivors]

    def _is_circle_complete(self) -> bool:
        """True if we have a circle plan and all its goals are completed."""
        return (
            len(self.current_survivor_plan) > 0
            and all(goal.completed for goal in self.current_survivor_plan)
        )

    def _lock_channel_centers(self) -> None:
        """Lock on first pair of gates (2+), then extend as new gates appear (up to 4). Build waypoint list from current channel_centers_out; when more gates appear, extend waypoints and goal_plan (keep existing completed). Publish one at a time; goal_utils nudge at publish."""
        centers = self._gate_centers_sorted()
        if len(centers) < 2:
            return

        # Initial lock: first time we have at least 2 gates
        if not self.channel_locked:
            self.channel_centers_out = list(centers[: min(4, len(centers))])
            self.channel_locked = True
            self.task2_start_position = self.channel_centers_out[0]
        else:
            # Already locked: extend if we see more gates (up to 4)
            if len(centers) > len(self.channel_centers_out) and len(centers) <= 4:
                self.channel_centers_out = list(centers[:4])

        # Build full waypoint list from current channel_centers_out (gate centers + interpolated between each pair)
        num_between = 2
        waypoints: List[Vec3] = []
        for i in range(len(self.channel_centers_out)):
            a = self.channel_centers_out[i]
            waypoints.append((float(a[0]), float(a[1]), 0.0))
            if i < len(self.channel_centers_out) - 1:
                b = self.channel_centers_out[i + 1]
                for k in range(1, num_between + 1):
                    t = k / (num_between + 1.0)
                    x = a[0] + t * (b[0] - a[0])
                    y = a[1] + t * (b[1] - a[1])
                    waypoints.append((float(x), float(y), 0.0))
        old_len = len(self.channel_waypoints_out)
        self.channel_waypoints_out = waypoints

        # Goal plan: initial fill or extend (do not reset completed on existing goals)
        if old_len == 0:
            self.goal_plan = []
            for i, pt in enumerate(self.channel_waypoints_out):
                self.goal_plan.append(GoalPlan(
                    goal_id=f"channel_out_{i+1}",
                    position=pt,
                    completed=False
                ))
        else:
            for i, pt in enumerate(self.channel_waypoints_out[old_len:], start=old_len + 1):
                self.goal_plan.append(GoalPlan(
                    goal_id=f"channel_out_{i}",
                    position=pt,
                    completed=False
                ))

    def _check_goal_completion(self) -> None:
        """Check if current goal was reached and mark it complete"""
        if not self.goal_plan:
            return
            
        current_goal = None
        for goal in self.goal_plan:
            if not goal.completed:
                current_goal = goal
                break
                
        if current_goal is None:
            return
            
        # Check if we're close enough to the goal
        pos = (self.pose[0], self.pose[1])
        dist = self._distance(pos, current_goal.position[:2])
        if dist <= self.goal_reached_dist:
            current_goal.completed = True

    def _create_survivor_circle_plan(
        self, survivor_pos: Tuple[float, ...], survivor_id: str, entity_id: Optional[int] = None
    ) -> None:
        """Create 4-point circle goals around a survivor (nominal positions). Nudge applied in publish_goals() with boat as from_pt.
        When circling, entity_id is excluded from obstacles (detected and confirmed)."""
        if survivor_id in self.handled_survivors:
            return

        self._circling_survivor_entity_id = entity_id
        circle_goals = []
        cx, cy = survivor_pos[0], survivor_pos[1]
        radius = 6.0

        for i in range(4):
            angle = 2.0 * np.pi * (i / 4.0)
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            desired = (float(x), float(y), 0.0)
            goal = GoalPlan(
                goal_id=f"circle_{survivor_id}_{i+1}",
                position=desired,
                completed=False
            )
            circle_goals.append(goal)

        self.current_survivor_plan = circle_goals
        self.handled_survivors.add(survivor_id)

    def _transition_to_debris_field(self) -> None:
        """Transition from Phase A to Phase B only when we have 4 gates and all channel waypoints are complete."""
        if self.phase != Phase.TRANSIT_OUT:
            return
        all_channel_complete = all(goal.completed for goal in self.goal_plan)
        if not all_channel_complete:
            return
        # Require full channel (4 gates) before moving to debris field
        if len(self.channel_centers_out) != 4:
            return
        self.phase = Phase.DEBRIS_FIELD
        self.goal_plan = []  # Will use forward waypoints until green indicator seen

    def _transition_to_return(self) -> None:
        """Transition from Phase B to Phase C"""
        if self.phase != Phase.DEBRIS_FIELD:
            return
        unhandled = self._get_unhandled_survivors()
        more_survivors = len(unhandled) > 0
        circle_complete = self._is_circle_complete()

        if circle_complete and not more_survivors:
            self.phase = Phase.RETURN
            self._circling_survivor_entity_id = None
            self._create_return_plan()
        elif circle_complete and more_survivors:
            surv_id, surv_pos, surv_eid = unhandled[0]
            self._create_survivor_circle_plan(surv_pos, surv_id, entity_id=surv_eid)
            self.goal_plan = self.current_survivor_plan

    def _create_return_plan(self) -> None:
        """Return through channel in reverse (full waypoint list); goal_utils nudge at publish so we avoid black buoys etc."""
        self.return_waypoints = list(reversed(self.channel_waypoints_out))
        # Nominal position per waypoint; nudge_goal_away_from_obstacles applied in publish_goals()
        self.goal_plan = []
        for i, waypoint in enumerate(self.return_waypoints):
            goal = GoalPlan(
                goal_id=f"return_{i+1}",
                position=waypoint,
                completed=False
            )
            self.goal_plan.append(goal)

    def tick(self) -> None:
        """Main update loop - manage phases, lock channels, check completion, publish goals"""
        
        if self.phase == Phase.TRANSIT_OUT:
            # Lock channel centers once discovered
            self._lock_channel_centers()
            
            # Check goal completion
            self._check_goal_completion()
            
            # Check for transition to debris field
            self._transition_to_debris_field()
            
            # If no locked channel yet, use fallback
            if not self.channel_locked:
                self._publish_single_goal(self._fallback_goal_A())
            else:
                self.publish_goals()
                
        elif self.phase == Phase.DEBRIS_FIELD:
            self._check_goal_completion()
            self._transition_to_return()

            # Survivor at end of debris field: go forward one waypoint at a time until green indicator seen, then circle
            unhandled = self._get_unhandled_survivors()
            if unhandled or self.current_survivor_plan:
                if self.current_survivor_plan and not self._is_circle_complete():
                    self.goal_plan = self.current_survivor_plan
                elif self.current_survivor_plan and self._is_circle_complete():
                    if self.phase == Phase.DEBRIS_FIELD and unhandled:
                        surv_id, surv_pos, surv_eid = unhandled[0]
                        self._create_survivor_circle_plan(surv_pos, surv_id, entity_id=surv_eid)
                        self.goal_plan = self.current_survivor_plan
                else:
                    surv_id, surv_pos, surv_eid = unhandled[0]
                    self._create_survivor_circle_plan(surv_pos, surv_id, entity_id=surv_eid)
                    self.goal_plan = self.current_survivor_plan
                self.publish_goals()
            else:
                self._publish_single_goal(self._forward_goal_debris())
            
        elif self.phase == Phase.RETURN:
            # Check goal completion
            self._check_goal_completion()
            
            # Check if mission complete
            if all(goal.completed for goal in self.goal_plan):
                self.mission_complete = True
            
            # Publish active goals
            self.publish_goals()

    def done(self) -> bool:
        """Return True if mission complete (SUCCESS)"""
        return self.mission_complete

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 2) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = self._obstacles()
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2], boat_pos=start)
        goals = self.goal_queue[:max_goals]
        goals_xy = [(g[0], g[1]) for g in goals]

        out = planner.plan_multi_goal_path(start, goals_xy, obstacles, gates=gates, map_bounds=self.map_bounds)
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]

    def _fallback_goal_A(self) -> Vec3:
        """Forward along initial heading (or last gate normal) by 30m. No grid clipping."""
        x, y, hdg = self.pose
        if self.last_gate_normal is not None:
            d = self.last_gate_normal
        else:
            d = np.array([np.cos(self.initial_heading), np.sin(self.initial_heading)], dtype=float)
            d = d / (np.linalg.norm(d) + 1e-9)

        step = 30.0
        p = np.array([x, y], dtype=float) + d * step
        desired = (float(p[0]), float(p[1]), 0.0)
        return nudge_goal_away_from_obstacles(desired, self._obstacles(), (x, y, hdg))

    def _forward_goal_debris(self) -> Vec3:
        """One waypoint forward through debris field (along last gate normal or heading) until green indicator seen."""
        return self._fallback_goal_A()

    def _distance(self, a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _gate_centers_sorted(self) -> List[Vec3]:
        boat_xy = (self.pose[0], self.pose[1])
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2], boat_pos=boat_xy)
        if not gates:
            return []

        red, green = gates[-1]
        gv = np.array([green[0] - red[0], green[1] - red[1]], dtype=float)
        if np.linalg.norm(gv) > 1e-6:
            n = np.array([-gv[1], gv[0]], dtype=float)
            self.last_gate_normal = n / (np.linalg.norm(n) + 1e-9)

        hdg = self.pose[2]
        fwd = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
        bx, by = self.pose[0], self.pose[1]
        centers = [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0, 0.0) for (r, g) in gates]
        centers.sort(key=lambda p: (p[0] - bx) * fwd[0] + (p[1] - by) * fwd[1])
        return centers
