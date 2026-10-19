#!/usr/bin/env python3
"""
Task 4 – Supply Drop (water delivery only).

The ASV finds yellow stationary vessels (yellow_supply_drop), navigates to within
SERVICE_RANGE_M, aligns bow toward each vessel, turns pump on for SERVICE_DURATION_SEC,
then reverses away and moves to the next. Up to TARGET_COUNT vessels serviced.
No map bounds; boat searches in open water.

Phases: SEARCH → APPROACH → ALIGN → SERVICE → RETREAT → NEXT_TARGET
"""

from __future__ import annotations

from enum import Enum
from typing import List, Optional, Set, Tuple
import math
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner
from Global.types import DetectedEntity, Vec3

Vec2 = Tuple[float, float]
Pose = Vec3  # (x, y, heading_rad)

# Constants
SERVICE_RANGE_M = 5.0
ALIGNMENT_TOLERANCE_RAD = 0.26  # ~15°
SERVICE_DURATION_SEC = 3.0
RETREAT_DISTANCE_M = 3.0
SEARCH_GOAL_DISTANCE_M = 100.0
TARGET_COUNT = 3


class Phase(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    SERVICE = "SERVICE"
    RETREAT = "RETREAT"
    NEXT_TARGET = "NEXT_TARGET"


class Task4Manager:
    """Task 4: Water delivery to yellow supply drop vessels."""

    def __init__(self, entities, map_bounds: Optional[Vec2], start_pose: Pose):
        self.entities = entities
        self.map_bounds = None  # Task 4: no map bounds

        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None

        self.phase = Phase.SEARCH
        self.serviced_ids: Set[int] = set()
        self.serviced_for_report: List[Tuple[int, Vec2]] = []  # (id, pos) for /gs_message_send
        self.current_target_id: Optional[int] = None
        self.current_target_pos: Optional[Vec2] = None
        self.service_start_time: Optional[float] = None
        self.retreat_start_pos: Optional[Vec2] = None
        self.pump_on = False

        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        already = any(
            getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities
        )
        if not already:
            self.entities.add_or_update(
                det.entity_type, det.position, det.entity_id,
                name=f"{det.entity_type}_{det.entity_id}"
            )

    def _distance(self, a: Vec2, b: Vec2) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _get_yellow_supply_drops(self) -> List[Tuple[int, Vec2]]:
        """Return [(entity_id, position), ...] for yellow_supply_drop not in serviced_ids."""
        out: List[Tuple[int, Vec2]] = []
        for e in self.entities.get_by_type("yellow_supply_drop"):
            if e.entity_id is None:
                continue
            if e.entity_id not in self.serviced_ids:
                out.append((e.entity_id, e.position))
        return out

    def _obstacles_for_approach(self) -> List[Vec2]:
        """Obstacles = other yellow_supply_drop (not current target)."""
        out: List[Vec2] = []
        for e in self.entities.get_by_type("yellow_supply_drop"):
            if self.current_target_id is not None and e.entity_id == self.current_target_id:
                continue
            out.append(e.position)
        return out

    def _search_goal(self) -> Vec3:
        x, y, hdg = self.pose
        dx = SEARCH_GOAL_DISTANCE_M * math.cos(hdg)
        dy = SEARCH_GOAL_DISTANCE_M * math.sin(hdg)
        return (x + dx, y + dy, hdg)

    def _retreat_goal(self) -> Vec3:
        x, y, hdg = self.pose
        dx = RETREAT_DISTANCE_M * math.cos(hdg)
        dy = RETREAT_DISTANCE_M * math.sin(hdg)
        return (x - dx, y - dy, hdg)

    def tick(self) -> None:
        x, y, hdg = self.pose
        pos = (x, y)

        if self.phase == Phase.SEARCH:
            yellows = self._get_yellow_supply_drops()
            if yellows:
                # Nearest unserviced
                nearest = min(yellows, key=lambda p: self._distance(pos, p[1]))
                self.current_target_id = nearest[0]
                self.current_target_pos = nearest[1]
                self.phase = Phase.APPROACH
            else:
                goal = self._search_goal()  # (x, y, heading_rad)
                self.entities.clear_goals()
                self.entities.add("goal", goal, name="goal_wp1")
                self.goal_queue = [goal]

        elif self.phase == Phase.APPROACH:
            if self.current_target_pos is None:
                self.phase = Phase.NEXT_TARGET
                return
            dist = self._distance(pos, self.current_target_pos)
            if dist <= SERVICE_RANGE_M:
                self.phase = Phase.ALIGN
            else:
                # Goal = point within SERVICE_RANGE_M of target (approach from boat direction)
                tx, ty = self.current_target_pos[0], self.current_target_pos[1]
                angle_to_target = math.atan2(ty - y, tx - x)
                # Place goal at SERVICE_RANGE_M from target, between target and boat
                gx = tx - (SERVICE_RANGE_M - 0.5) * math.cos(angle_to_target)
                gy = ty - (SERVICE_RANGE_M - 0.5) * math.sin(angle_to_target)
                goal = (gx, gy, angle_to_target)
                self.entities.clear_goals()
                self.entities.add("goal", goal, name="goal_wp1")
                self.goal_queue = [goal]

        elif self.phase == Phase.ALIGN:
            if self.current_target_pos is None:
                self.phase = Phase.NEXT_TARGET
                return
            dist = self._distance(pos, self.current_target_pos)
            tx, ty = self.current_target_pos
            angle_to_target = math.atan2(ty - y, tx - x)
            heading_error = abs(
                (angle_to_target - hdg + math.pi) % (2 * math.pi) - math.pi
            )
            if dist <= SERVICE_RANGE_M and heading_error <= ALIGNMENT_TOLERANCE_RAD:
                self.phase = Phase.SERVICE
                self.service_start_time = time.time()
            else:
                # Hold position, no motion goal (align in place); desired heading = toward target
                self.entities.clear_goals()
                hold_goal = (x, y, angle_to_target)
                self.entities.add("goal", hold_goal, name="goal_wp1")
                self.goal_queue = [hold_goal]

        elif self.phase == Phase.SERVICE:
            self.pump_on = True
            self.entities.clear_goals()
            service_goal = (x, y, hdg)
            self.entities.add("goal", service_goal, name="goal_wp1")
            self.goal_queue = [service_goal]
            elapsed = time.time() - (self.service_start_time or time.time())
            if elapsed >= SERVICE_DURATION_SEC:
                self.pump_on = False
                if self.current_target_id is not None and self.current_target_pos is not None:
                    self.serviced_ids.add(self.current_target_id)
                    self.serviced_for_report.append(
                        (self.current_target_id, self.current_target_pos)
                    )
                self.retreat_start_pos = pos
                self.phase = Phase.RETREAT
                self.current_target_id = None
                self.current_target_pos = None

        elif self.phase == Phase.RETREAT:
            if self.retreat_start_pos is None:
                self.retreat_start_pos = pos
            dist_moved = self._distance(self.retreat_start_pos, pos)
            if dist_moved >= RETREAT_DISTANCE_M:
                self.phase = Phase.NEXT_TARGET
            else:
                goal = self._retreat_goal()
                self.entities.clear_goals()
                self.entities.add("goal", goal, name="goal_wp1")
                self.goal_queue = [goal]

        elif self.phase == Phase.NEXT_TARGET:
            if len(self.serviced_ids) >= TARGET_COUNT:
                # SUCCESS - done() returns True
                self.entities.clear_goals()
                self.goal_queue = []
                self.prev_pos = (x, y)
                return
            yellows = self._get_yellow_supply_drops()
            if yellows:
                nearest = min(yellows, key=lambda p: self._distance(pos, p[1]))
                self.current_target_id = nearest[0]
                self.current_target_pos = nearest[1]
                self.phase = Phase.APPROACH
                # Set approach goal for this tick
                tx, ty = self.current_target_pos[0], self.current_target_pos[1]
                angle_to_target = math.atan2(ty - y, tx - x)
                gx = tx - (SERVICE_RANGE_M - 0.5) * math.cos(angle_to_target)
                gy = ty - (SERVICE_RANGE_M - 0.5) * math.sin(angle_to_target)
                goal = (gx, gy, angle_to_target)
                self.entities.clear_goals()
                self.entities.add("goal", goal, name="goal_wp1")
                self.goal_queue = [goal]
            else:
                self.phase = Phase.SEARCH

        self.prev_pos = (x, y)

    def done(self) -> bool:
        return len(self.serviced_ids) >= TARGET_COUNT

    def plan(self, planner: PotentialFieldsPlanner) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        if self.phase == Phase.APPROACH:
            obstacles = self._obstacles_for_approach()
        else:
            obstacles = list(self.entities.get_obstacles())

        goals = self.goal_queue[:2]
        goals_xy = [(g[0], g[1]) for g in goals]

        out = planner.plan_multi_goal_path(
            start, goals_xy, obstacles, gates=None, map_bounds=None
        )
        path_raw = out["path"]
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in path_raw]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]
