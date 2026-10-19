#!/usr/bin/env python3
"""Shared helpers for goal placement and geometry. Used by Task1, Task2, Task3."""

from __future__ import annotations
from typing import List, Tuple

import numpy as np

from Global.types import Vec2, Vec3, _to_vec3

# Default clearance from obstacles (m) so goals are not placed on/near black buoys
DEFAULT_GOAL_CLEARANCE = 5.0


def segments_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool:
    """True if segment p1->p2 intersects segment q1->q2. Used for gate crossing (Task1, Task3)."""
    a = np.array(p1, dtype=float)
    b = np.array(p2, dtype=float)
    c = np.array(q1, dtype=float)
    d = np.array(q2, dtype=float)

    def _orient(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> float:
        return float((y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0]))

    def _on_segment(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> bool:
        return (
            min(x[0], y[0]) - 1e-9 <= z[0] <= max(x[0], y[0]) + 1e-9
            and min(x[1], y[1]) - 1e-9 <= z[1] <= max(x[1], y[1]) + 1e-9
        )

    o1, o2 = _orient(a, b, c), _orient(a, b, d)
    o3, o4 = _orient(c, d, a), _orient(c, d, b)
    if (o1 * o2 < 0.0) and (o3 * o4 < 0.0):
        return True
    if abs(o1) < 1e-9 and _on_segment(a, b, c): return True
    if abs(o2) < 1e-9 and _on_segment(a, b, d): return True
    if abs(o3) < 1e-9 and _on_segment(c, d, a): return True
    if abs(o4) < 1e-9 and _on_segment(c, d, b): return True
    return False


def nudge_goal_away_from_obstacles(
    desired: Tuple[float, ...],
    obstacles: List[Tuple[float, ...]],
    from_point: Tuple[float, ...],
    min_clearance: float = DEFAULT_GOAL_CLEARANCE,
    step: float = 1.0,
    max_lateral_m: float = 10.0,
) -> Vec3:
    """Return a goal at least min_clearance from all obstacles, keeping the same direction from from_point.

    First tries shifting the goal left or right (perpendicular to from_point -> desired) so the goal
    sits slightly beside buoys while staying ahead. If no lateral offset is clear, slides the goal
    back toward from_point until clear.

    Args:
        desired: Preferred goal position (e.g. gate center, 50 m ahead).
        obstacles: Obstacle positions in meters (e.g. from entities.get_obstacles() + get_no_go_obstacle_points()).
        from_point: Reference point in meters (e.g. current boat position); direction from_point -> desired is preserved.
        min_clearance: Minimum distance (m) from any obstacle.
        step: Step size (m) for lateral offset and for sliding back.
        max_lateral_m: Maximum lateral offset (m) to try left/right before falling back to slide-back.

    Returns:
        A position at least min_clearance from all obstacles, or desired if no obstacles / already clear.
    """
    if not obstacles:
        return _to_vec3(desired)

    desired_arr = np.array(desired[:2], dtype=float)
    from_arr = np.array(from_point[:2], dtype=float)
    obs_arrs = [np.array(o[:2], dtype=float) for o in obstacles]
    default_h = desired[2] if len(desired) >= 3 else 0.0

    def clear(c: np.ndarray) -> bool:
        return all(float(np.linalg.norm(c - o)) >= min_clearance for o in obs_arrs)

    def still_ahead(c: np.ndarray) -> bool:
        d = c - from_arr
        return float(np.dot(d, dir_ahead)) >= -1e-6

    # Direction ahead (from_point -> desired)
    dir_ahead = desired_arr - from_arr
    dist_ahead = float(np.linalg.norm(dir_ahead))
    if dist_ahead < 1e-6:
        return _to_vec3(desired)
    dir_ahead = dir_ahead / dist_ahead
    # Perpendicular: left = 90° counter-clockwise, right = 90° clockwise
    left = np.array([-dir_ahead[1], dir_ahead[0]], dtype=float)
    right = np.array([dir_ahead[1], -dir_ahead[0]], dtype=float)

    if clear(desired_arr):
        return (float(desired_arr[0]), float(desired_arr[1]), float(default_h))

    # Try lateral nudge: desired + k*left and desired + k*right for k = step, 2*step, ... up to max_lateral_m
    k = step
    while k <= max_lateral_m + 1e-9:
        for offset in (k * left, k * right):
            candidate = desired_arr + offset
            if clear(candidate) and still_ahead(candidate):
                return (float(candidate[0]), float(candidate[1]), float(default_h))
        k += step

    # Fall back: slide back toward from_point
    dir_back = -dir_ahead
    candidate = desired_arr.copy()
    while True:
        if clear(candidate):
            return (float(candidate[0]), float(candidate[1]), float(default_h))
        candidate = candidate + dir_back * step
        if float(np.linalg.norm(candidate - from_arr)) <= step:
            return (float(candidate[0]), float(candidate[1]), float(default_h))
