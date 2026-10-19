"""
Watchdog protocols for the global planner node.

ROS-free. Called each planning tick with current state; returns an optional
twist override (body frame: m/s, rad/s) and a reason string for logging.
When the node applies an override it logs "Watchdog protocol: <reason>".
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

# Collision: trigger if obstacle within this distance (m) and we're moving toward it
COLLISION_DISTANCE_M = 2.0
# Reverse 1 m at this speed (m/s)
REVERSE_DISTANCE_M = 1.0
REVERSE_SPEED_M_S = 0.5
# Prediction timeout: no goal for this long (s) then spin to look
PREDICTION_TIMEOUT_SEC = 30.0
# Spin for this long (s) when we can't find goals
SPIN_DURATION_SEC = 20.0
# Angular rate when spinning (rad/s)
SPIN_ANGULAR_RATE_RAD_S = 0.5
# Reorient to original heading at this rate (rad/s)
REORIENT_RATE_RAD_S = 0.4
# Heading error (rad) below which reorient is done
REORIENT_DONE_THRESHOLD_RAD = 0.1
# After this many spin+reorient cycles with still no goal, declare stuck
MAX_SPIN_SEARCHES = 3


def _normalize_angle(a: float) -> float:
    """Wrap to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _closing_speed(
    boat_x_m: float,
    boat_y_m: float,
    vx_m_s: float,
    vy_m_s: float,
    obs_x_m: float,
    obs_y_m: float,
) -> float:
    """Closing speed (m/s): positive = approaching."""
    dx = obs_x_m - boat_x_m
    dy = obs_y_m - boat_y_m
    dist_m = math.hypot(dx, dy)
    if dist_m < 1e-6:
        return 1.0
    return -(dx * vx_m_s + dy * vy_m_s) / dist_m  # positive = approaching


def tick(
    state: Dict[str, Any],
    time_now: float,
    dt_sec: float,
    pose_m: Tuple[float, float, float],
    velocity_m_s: Tuple[float, float],
    obstacles_m: List[Tuple[float, float]],
    has_goal: bool,
) -> Tuple[Dict[str, Any], Optional[Dict[str, float]], Optional[str]]:
    """
    Run watchdog checks. All positions in m, velocity in m/s.
    Returns (new_state, twist_override, reason).
    twist_override: body frame {linear.x, linear.y, angular.z} in m/s and rad/s.
    """
    x_m, y_m, heading = pose_m
    vx_m, vy_m = velocity_m_s
    stuck = state.get("stuck", False)
    reverse_remaining_m = state.get("reverse_remaining_m", 0.0)
    spin_until_time = state.get("spin_until_time")
    reorient_until_time = state.get("reorient_until_time")
    reorient_target_heading = state.get("reorient_target_heading")
    prediction_start_time = state.get("prediction_start_time")
    original_heading = state.get("original_heading", heading)
    spin_search_count = state.get("spin_search_count", 0)

    # 1) Stuck: stop forever
    if stuck:
        return (
            state,
            {"linear.x": 0.0, "linear.y": 0.0, "angular.z": 0.0},
            "Watchdog protocol: I am stuck.",
        )

    # 2) Collision: reverse (unless already reversing)
    if reverse_remaining_m <= 0:
        for (ox, oy) in obstacles_m:
            d_m = math.hypot(ox - x_m, oy - y_m)
            if d_m < COLLISION_DISTANCE_M:
                closing = _closing_speed(x_m, y_m, vx_m, vy_m, ox, oy)
                if closing > 0.1:  # approaching
                    reverse_remaining_m = REVERSE_DISTANCE_M
                    state = dict(state)
                    state["reverse_remaining_m"] = reverse_remaining_m
                    return (
                        state,
                        {"linear.x": -REVERSE_SPEED_M_S, "linear.y": 0.0, "angular.z": 0.0},
                        "Watchdog protocol: object avoidance — entity too close, stopping and reversing.",
                    )

    # 3) Currently reversing
    if reverse_remaining_m > 0:
        state = dict(state)
        state["reverse_remaining_m"] = max(0.0, reverse_remaining_m - REVERSE_SPEED_M_S * dt_sec)
        return (
            state,
            {"linear.x": -REVERSE_SPEED_M_S, "linear.y": 0.0, "angular.z": 0.0},
            "Watchdog protocol: object avoidance — reversing.",
        )

    # 4) Reset prediction state when we have a goal
    if has_goal:
        state = dict(state)
        state["prediction_start_time"] = None
        state["spin_search_count"] = 0
        state["spin_until_time"] = None
        state["reorient_until_time"] = None
        return (state, None, None)

    # 5) No goal: track prediction start
    if prediction_start_time is None:
        state = dict(state)
        state["prediction_start_time"] = time_now
        state["original_heading"] = heading
        return (state, None, None)

    # 6) Spin phase (can't find goals — spinning to look)
    if spin_until_time is not None and time_now < spin_until_time:
        return (
            state,
            {"linear.x": 0.0, "linear.y": 0.0, "angular.z": SPIN_ANGULAR_RATE_RAD_S},
            "Watchdog protocol: prediction timeout — spinning to look for objects. We are spinning.",
        )

    # 7) Just finished spin: start reorient (next tick we'll be in reorient phase)
    if spin_until_time is not None and time_now >= spin_until_time:
        state = dict(state)
        state["spin_until_time"] = None
        state["reorient_until_time"] = time_now + 60.0  # end when heading ok
        state["reorient_target_heading"] = original_heading
        return (state, None, None)

    # 8) Reorient phase
    if reorient_until_time is not None:
        err = _normalize_angle(reorient_target_heading - heading)
        if abs(err) < REORIENT_DONE_THRESHOLD_RAD:
            state = dict[str, Any](state)
            state["reorient_until_time"] = None
            state["spin_search_count"] = spin_search_count + 1
            if state["spin_search_count"] >= MAX_SPIN_SEARCHES:
                state["stuck"] = True
                return (
                    state,
                    {"linear.x": 0.0, "linear.y": 0.0, "angular.z": 0.0},
                    "Watchdog protocol: I am stuck.",
                )
            state["prediction_start_time"] = time_now  # allow another timeout
            return (state, None, None)
        omega = REORIENT_RATE_RAD_S if err > 0 else -REORIENT_RATE_RAD_S
        return (
            state,
            {"linear.x": 0.0, "linear.y": 0.0, "angular.z": omega},
            "Watchdog protocol: reorienting to original heading.",
        )

    # 9) Prediction timeout: start spin
    if time_now - prediction_start_time > PREDICTION_TIMEOUT_SEC:
        state = dict[str, Any](state)
        state["spin_until_time"] = time_now + SPIN_DURATION_SEC
        state["original_heading"] = original_heading
        return (
            state,
            {"linear.x": 0.0, "linear.y": 0.0, "angular.z": SPIN_ANGULAR_RATE_RAD_S},
            "Watchdog protocol: prediction timeout — spinning to look for objects. We are spinning.",
        )

    return (state, None, None)


def initial_state() -> Dict[str, Any]:
    return {
        "stuck": False,
        "reverse_remaining_m": 0.0,
        "spin_until_time": None,
        "reorient_until_time": None,
        "reorient_target_heading": None,
        "prediction_start_time": None,
        "original_heading": 0.0,
        "spin_search_count": 0,
    }
