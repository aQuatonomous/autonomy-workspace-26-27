#!/usr/bin/env python3
"""Pool Test Task (task_id = 0)

Simple test task for verifying the full pipeline on water:
- Waits for GPS + heading + GUIDED (same as Task 1)
- Sets one goal: 5m forward along original boat heading (never updated)
- If a red buoy is detected at any time, abandons the forward goal and
  goes to 0.5m ahead of the red buoy (past it)
- Never completes (runs until manually stopped)
"""

from __future__ import annotations
from typing import List, Optional, Tuple
import logging
import math
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner
from Global.goal_utils import nudge_goal_away_from_obstacles
from Global.types import DetectedEntity, Vec3

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]

STOP_MARGIN_M = 0.5
FORWARD_STEP_M = 5.0

logger = logging.getLogger("TaskTest")


class TaskTestManager:
    def __init__(self, entities, map_bounds, start_pose: Pose):
        self.entities = entities
        self.map_bounds = tuple(float(v) for v in map_bounds) if map_bounds else None
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None

        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)
        self._target_red_entity_id: Optional[int] = None

    def update_pose(self, x: float, y: float, heading: float) -> None:
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        already = any(
            getattr(e, "entity_id", None) == det.entity_id
            for e in self.entities.entities
        )
        if not already:
            self.entities.add_or_update(
                det.entity_type, det.position, det.entity_id,
                name=f"{det.entity_type}_{det.entity_id}",
            )

    def _obstacles(self, exclude_entity_ids: Optional[set] = None) -> List[Tuple[float, ...]]:
        return list(self.entities.get_obstacles(exclude_entity_ids=exclude_entity_ids))

    def _nearest_red_buoy(self) -> Tuple[Optional[Vec3], Optional[int]]:
        """Return (position, entity_id) of nearest red buoy, or (None, None)."""
        bx, by = self.pose[0], self.pose[1]
        best_pos = None
        best_id = None
        best_dist = float("inf")
        for e in self.entities.entities:
            if e.type not in ("red_buoy", "red_pole_buoy"):
                continue
            dx = float(e.position[0]) - bx
            dy = float(e.position[1]) - by
            d = math.hypot(dx, dy)
            if d < best_dist:
                best_dist = d
                best_pos = e.position
                best_id = e.entity_id
        return (best_pos, best_id)

    def tick(self) -> None:
        x, y, hdg = self.pose
        red, red_id = self._nearest_red_buoy()
        self._target_red_entity_id = red_id

        logger.info("boat pos=(%.1f, %.1f) hdg=%.2f rad", x, y, hdg)

        if red is not None:
            rx, ry = float(red[0]), float(red[1])
            dist = math.hypot(rx - x, ry - y)
            logger.info("RED BUOY DETECTED id=%s at (%.1f, %.1f) dist=%.1f m -> going for buoy", red_id, rx, ry, dist)
            dx, dy = rx - x, ry - y
            if dist > STOP_MARGIN_M:
                ratio = (dist - STOP_MARGIN_M) / dist
                gx = x + dx * ratio
                gy = y + dy * ratio
            else:
                gx, gy = x, y
            goal = (gx, gy, 0.0)
        else:
            gx = x + FORWARD_STEP_M * math.cos(hdg)
            gy = y + FORWARD_STEP_M * math.sin(hdg)
            goal = (gx, gy, 0.0)
            logger.info("No red buoy -> going for forward goal (%.1f, %.1f)", gx, gy)

        exclude_ids = {red_id} if red_id is not None else None
        obstacles = self._obstacles(exclude_entity_ids=exclude_ids)
        from_pt = (x, y, hdg)
        goal = nudge_goal_away_from_obstacles(goal, obstacles, from_pt)

        logger.info("GOAL APPLIED at (%.1f, %.1f) [%d obstacles considered]", goal[0], goal[1], len(obstacles))

        self.entities.clear_goals()
        self.entities.add("goal", goal, name="goal_wp1")
        self.goal_queue = [goal]
        self.prev_pos = (x, y)

    def done(self) -> bool:
        return False

    def plan(self, planner: PotentialFieldsPlanner) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            logger.info("plan() skipped: no goal in queue")
            return

        start = (self.pose[0], self.pose[1])
        exclude_ids = {self._target_red_entity_id} if self._target_red_entity_id is not None else None
        obstacles = self._obstacles(exclude_entity_ids=exclude_ids)
        goals_xy = [(self.goal_queue[0][0], self.goal_queue[0][1])]
        out = planner.plan_multi_goal_path(
            start, goals_xy, obstacles, gates=None, map_bounds=self.map_bounds,
        )
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]
        if len(self.current_velocities) > 0:
            vx, vy = float(self.current_velocities[0][0]), float(self.current_velocities[0][1])
            spd = float(self.current_speeds[0])
            logger.info("plan() -> path %d pts, vel=(%.2f, %.2f) speed=%.2f m/s", len(self.current_path), vx, vy, spd)
        else:
            logger.info("plan() -> path %d pts, no velocity", len(self.current_path))
