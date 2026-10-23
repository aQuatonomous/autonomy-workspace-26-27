"""Was the object observable? The question that decides whether a miss means anything.

Punishing an object for not being detected while it sits 60 m astern behind a dock is how
existence estimates end up uncorrelated with existence. This module separates "we looked and saw
nothing" from "we were never going to see it".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from .config import VisibilityConfig
from .viewpoint import wrap_angle

VISIBLE = 'visible'
OUT_OF_RANGE_FAR = 'out_of_range_far'
OUT_OF_RANGE_NEAR = 'out_of_range_near'
OUT_OF_FOV = 'out_of_fov'
OCCLUDED = 'occluded'


@dataclass(frozen=True)
class BoatPose:
    """Observer pose in the map (ENU) frame."""

    east: float
    north: float
    yaw_rad: float            # heading, measured from east toward north (ENU convention)
    stamp: float = 0.0
    position_sigma_m: float = 0.5   # GPS 1-sigma; feeds the measurement covariance

    def relative(self, east: float, north: float) -> Tuple[float, float, float]:
        """(range, global_bearing, relative_bearing) from the boat to a map-frame point."""
        de = east - self.east
        dn = north - self.north
        rng = math.hypot(de, dn)
        global_bearing = math.atan2(dn, de)
        return rng, global_bearing, wrap_angle(global_bearing - self.yaw_rad)

    def covariance(self):
        import numpy as np
        s = self.position_sigma_m ** 2
        return np.diag([s, s])


@dataclass(frozen=True)
class VisibilityResult:
    visible: bool
    reason: str
    range_m: float
    global_bearing_rad: float
    relative_bearing_rad: float


@dataclass(frozen=True)
class Occluder:
    """A believed object that can cast a sensor shadow."""

    east: float
    north: float
    radius_m: float


def check_visibility(
    pose: BoatPose,
    east: float,
    north: float,
    config: Optional[VisibilityConfig] = None,
    occluders: Iterable[Occluder] = (),
) -> VisibilityResult:
    """Decide whether an object at (east, north) should have been seen from `pose`."""
    cfg = config or VisibilityConfig()
    rng, global_bearing, rel_bearing = pose.relative(east, north)

    def result(visible: bool, reason: str) -> VisibilityResult:
        return VisibilityResult(visible, reason, rng, global_bearing, rel_bearing)

    if rng > cfg.max_range_m:
        return result(False, OUT_OF_RANGE_FAR)
    if rng < cfg.min_range_m:
        return result(False, OUT_OF_RANGE_NEAR)
    if abs(rel_bearing) > cfg.fov_rad / 2.0:
        return result(False, OUT_OF_FOV)

    for occ in occluders:
        occ_rng, occ_bearing, _ = pose.relative(occ.east, occ.north)
        if occ_rng >= rng or occ_rng <= 1e-6:
            continue   # behind or co-located with the target: casts no shadow on it
        # Angular half-width of the occluder as seen from the boat.
        half_width = math.atan2(occ.radius_m, occ_rng)
        if abs(wrap_angle(global_bearing - occ_bearing)) <= half_width:
            return result(False, OCCLUDED)

    return result(True, VISIBLE)
