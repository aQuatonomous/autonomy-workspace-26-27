"""Correlation discount: how much *new* information a look carries.

The scorer's single biggest honesty risk is double-counting. A moored buoy observed 200 times from
one spot has not been confirmed 200 times; it has been confirmed roughly once, by a sensor that
might be systematically wrong. Every evidence update -- existence and class alike -- is therefore
scaled by how far the observing viewpoint is from the viewpoints already used for that object.

Novelty is measured against the *nearest* previously-used viewpoint rather than the most recent
one, so a boat oscillating between two headings cannot farm fresh evidence by alternating.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .config import ViewpointConfig


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Viewpoint:
    """Where the object was seen from, in object-relative terms.

    bearing_rad is the global bearing from the sensor to the object (map frame, ENU, measured
    from east toward north); range_m is the sensor-object distance.
    """

    bearing_rad: float
    range_m: float


class ViewpointHistory:
    """Bounded memory of the viewpoints already credited for one object."""

    def __init__(self, config: ViewpointConfig | None = None) -> None:
        self.config = config or ViewpointConfig()
        self._seen: List[Viewpoint] = []

    def __len__(self) -> int:
        return len(self._seen)

    @property
    def viewpoints(self) -> List[Viewpoint]:
        return list(self._seen)

    def novelty(self, vp: Viewpoint) -> float:
        """Distance, in 'fresh look' units, to the closest viewpoint already used.

        Returns math.inf for the first observation of an object.
        """
        if not self._seen:
            return math.inf
        cfg = self.config
        best = math.inf
        for prev in self._seen:
            d_bearing = abs(wrap_angle(vp.bearing_rad - prev.bearing_rad)) / cfg.bearing_scale_rad
            d_range = abs(vp.range_m - prev.range_m) / cfg.range_scale_m
            best = min(best, math.hypot(d_bearing, d_range))
        return best

    def weight(self, vp: Viewpoint) -> float:
        """Evidence weight in [min_weight, 1.0] for an observation from this viewpoint."""
        n = self.novelty(vp)
        if math.isinf(n):
            return 1.0
        cfg = self.config
        return cfg.min_weight + (1.0 - cfg.min_weight) * (1.0 - math.exp(-n))

    def record(self, vp: Viewpoint) -> None:
        """Remember a viewpoint so later looks from the same place are discounted."""
        self._seen.append(vp)
        if len(self._seen) > self.config.max_history:
            # Drop the oldest. The history is a coverage sketch, not an audit log, and the boat
            # does not revisit a buoy's early viewpoints after moving on down the course.
            self._seen.pop(0)

    def weigh_and_record(self, vp: Viewpoint) -> float:
        w = self.weight(vp)
        self.record(vp)
        return w
