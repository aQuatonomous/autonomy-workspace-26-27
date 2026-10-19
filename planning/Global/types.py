#!/usr/bin/env python3
"""
Shared types for Global tasks (Task1/2/3/4) and TaskMaster.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

Vec2 = Tuple[float, float]
# (x, y, heading_rad) - heading in radians; use 0.0 when not applicable
Vec3 = Tuple[float, float, float]
Pose = Vec3  # alias


def _to_vec3(p, default_heading: float = 0.0) -> Vec3:
    """Normalize position to (x, y, heading_rad). Accepts (x,y) or (x,y,h)."""
    if len(p) >= 3:
        return (float(p[0]), float(p[1]), float(p[2]))
    return (float(p[0]), float(p[1]), default_heading)


@dataclass
class DetectedEntity:
    """A single detected entity (buoy, indicator, etc.) from perception."""
    entity_id: int
    entity_type: str
    position: Vec3  # (x, y, heading_rad); use 0.0 for heading when unknown
