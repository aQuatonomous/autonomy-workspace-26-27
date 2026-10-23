"""Map-frame data association.

Association runs in the map frame, not the boat frame. For static objects this is the difference
between matching things that genuinely are not moving and chasing the boat's own motion at 2.6 Hz
with a sub-metre gate -- which is what the existing LiDAR tracker does, and why its ids churn.

Two rules do most of the work:

* A hard 1.50 m Euclidean backstop sits under the Mahalanobis gate. Gate buoys are 2.4384 m apart,
  so this makes merging two adjacent gate buoys into one object geometrically impossible no matter
  how badly the covariance is mis-estimated. A merge there would silently poison every downstream
  metric, so it is worth being blunt about.

* Genuinely ambiguous measurements are *deferred*, not guessed. A wrong association corrupts two
  objects at once -- it drags one position off truth and starves the other of evidence. Dropping
  the frame costs almost nothing at 2.6 Hz.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .config import AssociationConfig


@dataclass(frozen=True)
class Measurement:
    """One fused detection, already converted to the map frame."""

    east: float
    north: float
    range_m: float
    global_bearing_rad: float
    class_id: int = 255
    class_score: float = 0.0
    track_id: int = 0
    source: str = 'lidar'
    stamp: float = 0.0
    cv_age_s: Optional[float] = None


@dataclass
class AssociationResult:
    # (object_index, measurement_index) pairs
    matches: List[Tuple[int, int]] = field(default_factory=list)
    # measurement indices that should spawn new objects
    births: List[int] = field(default_factory=list)
    # measurement indices too ambiguous to use this frame
    deferred: List[int] = field(default_factory=list)
    # object indices that received no measurement and deserve a miss verdict
    unmatched_objects: List[int] = field(default_factory=list)
    # object indices that were in the gate of a deferred measurement. There *is* a detection
    # here -- we simply cannot say whose it is -- so these must not be charged a miss.
    ambiguous_objects: List[int] = field(default_factory=list)

    @property
    def matched_object_indices(self) -> set:
        return {o for o, _ in self.matches}

    @property
    def matched_measurement_indices(self) -> set:
        return {m for _, m in self.matches}


class _Candidate:
    __slots__ = ('obj', 'meas', 'd2', 'dist')

    def __init__(self, obj: int, meas: int, d2: float, dist: float) -> None:
        self.obj = obj
        self.meas = meas
        self.d2 = d2
        self.dist = dist


def associate(
    objects: Sequence,
    measurements: Sequence[Measurement],
    config: Optional[AssociationConfig] = None,
    measurement_covariances: Optional[Sequence] = None,
) -> AssociationResult:
    """Match measurements to existing objects.

    `objects` is any sequence of items exposing `east`, `north` and
    `mahalanobis_squared(east, north, extra_covariance=None)` -- in practice
    `scorer.ScoredObjectState`, whose attributes proxy its PositionFilter.
    """
    cfg = config or AssociationConfig()
    result = AssociationResult()

    if not measurements:
        result.unmatched_objects = list(range(len(objects)))
        return result
    if not objects:
        result.births = list(range(len(measurements)))
        return result

    # --- score every (object, measurement) pair that survives both gates ----------------------
    candidates: List[_Candidate] = []
    # per-measurement list of (d2, object_index), used for the ambiguity check
    per_measurement: List[List[Tuple[float, int]]] = [[] for _ in measurements]

    for mi, m in enumerate(measurements):
        extra = measurement_covariances[mi] if measurement_covariances is not None else None
        for oi, obj in enumerate(objects):
            dist = math.hypot(m.east - obj.east, m.north - obj.north)
            if dist > cfg.max_distance_m:
                continue
            d2 = obj.mahalanobis_squared(m.east, m.north, extra)
            if d2 > cfg.mahalanobis_gate:
                continue
            candidates.append(_Candidate(oi, mi, d2, dist))
            per_measurement[mi].append((d2, oi))

    # --- defer measurements whose best two hypotheses are too close to call -------------------
    ambiguous_measurements = set()
    for mi, cands in enumerate(per_measurement):
        if len(cands) < 2:
            continue
        cands.sort()
        if cands[1][0] - cands[0][0] < cfg.ambiguity_margin:
            ambiguous_measurements.add(mi)

    # --- greedy assignment, best (smallest Mahalanobis) first ---------------------------------
    # Greedy rather than Hungarian: with a 1.5 m gate the candidate graph is tiny and almost
    # always already unambiguous, and greedy keeps scipy out of the runtime dependencies.
    candidates.sort(key=lambda c: (c.d2, c.dist))
    used_objects = set()
    used_measurements = set()
    for c in candidates:
        if c.meas in ambiguous_measurements:
            continue
        if c.obj in used_objects or c.meas in used_measurements:
            continue
        result.matches.append((c.obj, c.meas))
        used_objects.add(c.obj)
        used_measurements.add(c.meas)

    for mi in range(len(measurements)):
        if mi in used_measurements:
            continue
        if mi in ambiguous_measurements:
            result.deferred.append(mi)
        else:
            result.births.append(mi)

    # Objects contesting a deferred measurement are "explained but unassigned": a detection did
    # land on them, so charging them an expected miss would be punishing the scorer's own
    # indecision rather than any evidence of absence.
    contested = set()
    for mi in ambiguous_measurements:
        for _, oi in per_measurement[mi]:
            if oi not in used_objects:
                contested.add(oi)

    result.ambiguous_objects = sorted(contested)
    result.unmatched_objects = [oi for oi in range(len(objects))
                                if oi not in used_objects and oi not in contested]
    return result
