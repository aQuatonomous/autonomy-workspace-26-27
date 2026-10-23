"""ObjectMap: the scorer proper.

Owns a persistent map-frame set of objects, each with its own existence, class and position
estimate, and drives them from a stream of fused detections.

The identity here is deliberately *not* the LiDAR track id. `buoy_tracker.py` deletes a track
after 10 consecutive misses, so a buoy that passes behind another and reappears comes back with a
brand-new id. Anything keyed on track id therefore forgets an object precisely when re-acquiring
it, which is the worst possible moment. Objects here are keyed by map-frame position and survive
arbitrary track churn; the most recent contributing track id is retained only for debugging.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .association import AssociationResult, Measurement, associate
from .classes import UNKNOWN_ID, class_colour, class_name
from .classifier import ClassPosterior
from .config import ScorerConfig
from .existence import ExistenceFilter
from .position import PositionFilter, SOURCE_CAMERA, polar_measurement_covariance
from .viewpoint import Viewpoint, ViewpointHistory
from .visibility import BoatPose, Occluder, check_visibility


@dataclass
class ScoredObjectState:
    """One persistent object in the scorer's map."""

    oid: int
    position: PositionFilter
    existence: ExistenceFilter
    classifier: ClassPosterior
    viewpoints: ViewpointHistory
    first_seen: float
    last_seen: float
    observation_count: int = 0
    track_id: int = 0
    track_ids_seen: set = field(default_factory=set)
    last_update_time: float = 0.0

    # -- proxies so association can treat this like a point with a covariance -----------------

    @property
    def east(self) -> float:
        return self.position.east

    @property
    def north(self) -> float:
        return self.position.north

    def mahalanobis_squared(self, east: float, north: float, extra_covariance=None) -> float:
        return self.position.mahalanobis_squared(east, north, extra_covariance)

    # -- readout -------------------------------------------------------------------------------

    @property
    def p_exist(self) -> float:
        return self.existence.p_exist

    def track_id_churn_count(self) -> int:
        """How many distinct LiDAR tracks have described this one object.

        The direct before/after number against today's behaviour: anything above 1 is a case
        where a track-id-keyed map would have lost the object and started over.
        """
        return len(self.track_ids_seen)

    def snapshot(self) -> Dict:
        cid, cprob = self.classifier.argmax()
        major, minor = self.position.sigma_major_minor()
        colour, colour_prob = self.classifier.argmax_colour()
        return {
            'oid': self.oid,
            'east': self.position.east,
            'north': self.position.north,
            'position_cov': self.position.covariance_flat(),
            'sigma_major_m': major,
            'sigma_minor_m': minor,
            'p_exist': self.p_exist,
            'class_id': cid,
            'class_name': class_name(cid),
            'class_prob': cprob,
            'class_entropy': self.classifier.entropy(),
            'class_colour': class_colour(cid),
            'colour': colour,
            'colour_prob': colour_prob,
            'class_evidence_weight': self.classifier.evidence_weight,
            'observation_count': self.observation_count,
            'track_id': self.track_id,
            'track_id_churn': len(self.track_ids_seen),
            'hits': self.existence.hits,
            'expected_misses': self.existence.expected_misses,
            'first_seen': self.first_seen,
            'last_seen': self.last_seen,
        }


class ObjectMap:
    """The scorer's persistent world model."""

    def __init__(
        self,
        config: Optional[ScorerConfig] = None,
        confusion_matrix: Optional[Sequence[Sequence[float]]] = None,
    ) -> None:
        self.config = config or ScorerConfig()
        self.confusion_matrix = confusion_matrix
        self.objects: List[ScoredObjectState] = []
        self._next_oid = 1
        self._last_time: Optional[float] = None
        # Running tallies, surfaced in the report so a bad run is diagnosable.
        self.stats = {
            'frames': 0,
            'measurements': 0,
            'matches': 0,
            'births': 0,
            'deferred': 0,
            'expected_misses': 0,
            'invisible': 0,
            'pruned': 0,
        }

    # -- lifecycle -----------------------------------------------------------------------------

    def _new_object(self, m: Measurement, now: float) -> ScoredObjectState:
        cfg = self.config
        obj = ScoredObjectState(
            oid=self._next_oid,
            position=PositionFilter(m.east, m.north, cfg.position),
            existence=ExistenceFilter(cfg.existence),
            classifier=ClassPosterior(cfg.classifier, self.confusion_matrix),
            viewpoints=ViewpointHistory(cfg.viewpoint),
            first_seen=now,
            last_seen=now,
            track_id=m.track_id,
            last_update_time=now,
        )
        self._next_oid += 1
        self.objects.append(obj)
        self.stats['births'] += 1
        return obj

    def _occluders(self) -> List[Occluder]:
        r = self.config.visibility.occlusion_object_radius_m
        threshold = self.config.visibility.occlusion_min_p_exist
        return [Occluder(o.position.east, o.position.north, r)
                for o in self.objects if o.p_exist >= threshold]

    # -- main entry point ----------------------------------------------------------------------

    def update(
        self,
        measurements: Sequence[Measurement],
        pose: BoatPose,
        now: Optional[float] = None,
    ) -> AssociationResult:
        """Fold one frame of fused detections into the map."""
        now = float(now if now is not None else pose.stamp)
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        self.stats['frames'] += 1
        self.stats['measurements'] += len(measurements)

        for obj in self.objects:
            obj.position.predict(dt)

        meas_covs = [self._measurement_covariance(m, pose) for m in measurements]
        result = associate(self.objects, measurements, self.config.association, meas_covs)
        self.stats['matches'] += len(result.matches)
        self.stats['deferred'] += len(result.deferred)

        # --- hits -----------------------------------------------------------------------------
        for oi, mi in result.matches:
            self._apply_hit(self.objects[oi], measurements[mi], meas_covs[mi], pose, now)

        # --- misses and non-looks ---------------------------------------------------------------
        occluders = self._occluders()
        for oi in result.unmatched_objects:
            obj = self.objects[oi]
            vis = check_visibility(pose, obj.position.east, obj.position.north,
                                   self.config.visibility, occluders)
            if vis.visible:
                # We looked where it should be and found nothing. Discount by viewpoint novelty
                # for the same reason hits are: 200 identical looks are not 200 experiments.
                vp = Viewpoint(vis.global_bearing_rad, vis.range_m)
                w = obj.viewpoints.weigh_and_record(vp)
                obj.existence.update_expected_miss(w)
                obj.last_update_time = now
                self.stats['expected_misses'] += 1
            else:
                obj.existence.update_invisible(now - obj.last_update_time)
                obj.last_update_time = now
                self.stats['invisible'] += 1

        # Objects contesting a deferred measurement get neither credit nor blame, but their
        # belief should still age, so the clock keeps running for them.
        for oi in result.ambiguous_objects:
            self.objects[oi].last_update_time = now

        # --- births ---------------------------------------------------------------------------
        for mi in result.births:
            m = measurements[mi]
            obj = self._new_object(m, now)
            self._apply_hit(obj, m, meas_covs[mi], pose, now, is_birth=True)

        self._prune()
        return result

    def _measurement_covariance(self, m: Measurement, pose: BoatPose) -> np.ndarray:
        cfg = self.config.position
        if m.source == SOURCE_CAMERA:
            sigma_r = max(m.range_m * cfg.sigma_range_camera_frac, 0.05)
            sigma_b = cfg.sigma_bearing_camera_rad
        else:
            sigma_r = cfg.sigma_range_lidar_m
            sigma_b = cfg.sigma_bearing_lidar_rad
        R = polar_measurement_covariance(m.range_m, m.global_bearing_rad, sigma_r, sigma_b)
        return R + pose.covariance()

    def _apply_hit(
        self,
        obj: ScoredObjectState,
        m: Measurement,
        R: np.ndarray,
        pose: BoatPose,
        now: float,
        is_birth: bool = False,
    ) -> None:
        vp = Viewpoint(m.global_bearing_rad, m.range_m)
        w = obj.viewpoints.weigh_and_record(vp)

        obj.existence.update_hit(w)
        obj.position.update(m.east, m.north, R)
        if m.class_id != UNKNOWN_ID:
            obj.classifier.update(m.class_id, m.class_score, w, m.cv_age_s)

        obj.observation_count += 1
        obj.last_seen = now
        obj.last_update_time = now
        obj.track_id = m.track_id
        if m.track_id:
            obj.track_ids_seen.add(m.track_id)

    def _prune(self) -> None:
        before = len(self.objects)
        self.objects = [o for o in self.objects if not o.existence.should_prune()]
        self.stats['pruned'] += before - len(self.objects)

    # -- readout -------------------------------------------------------------------------------

    def confident_objects(self, min_p_exist: float = 0.5) -> List[ScoredObjectState]:
        return [o for o in self.objects if o.p_exist >= min_p_exist]

    def snapshot(self) -> List[Dict]:
        return [o.snapshot() for o in self.objects]

    def __len__(self) -> int:
        return len(self.objects)
