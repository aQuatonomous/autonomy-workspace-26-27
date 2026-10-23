"""Every tunable in one place, as plain dataclasses.

Defaults are deliberately conservative: it is much easier to loosen a filter after seeing a
calibration report than to discover that a confident number was never earned.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Any, Dict


@dataclass
class ViewpointConfig:
    """Controls the correlation discount.

    At ~2.6 Hz, consecutive LiDAR frames of a stationary buoy from a slow-moving boat share most of
    their points. Treating them as independent Bernoulli trials drives p_exist to ~1 within three
    frames on evidence that is really one look. Evidence is therefore weighted by how *new* the
    viewpoint is relative to every viewpoint already used for this object.
    """

    bearing_scale_rad: float = math.radians(20.0)  # bearing change that counts as a fresh look
    range_scale_m: float = 8.0                     # range change that counts as a fresh look
    min_weight: float = 0.05                       # a repeat look is never worth literally nothing
    max_history: int = 64                          # bounded memory of previous viewpoints


@dataclass
class ExistenceConfig:
    """Bounded log-odds existence filter."""

    p0: float = 0.30               # birth prior: a fresh cluster is more likely than not spurious
    p_detect: float = 0.85         # P(detect | object exists and is visible)
    clutter_rate: float = 0.15     # P(measurement here | no object) -- wakes, spray, multipath
    miss_penalty: float = 0.70     # log-odds subtracted per independent *expected* miss
    logit_min: float = -4.0        # p ~ 0.018
    logit_max: float = 4.0         # p ~ 0.982 -- never claim certainty
    decay_tau_s: float = 60.0      # relaxation toward the prior while the object is unobservable
    prune_below_p: float = 0.05
    prune_after_expected_misses: int = 12


@dataclass
class ClassifierConfig:
    """Categorical posterior over the class vocabulary."""

    score_floor: float = 0.15      # YOLO score at or below which a detection carries zero evidence
    score_reference: float = 0.60  # score at which a detection carries full evidence
    max_logprob_spread: float = 6.0  # bounds posterior confidence (~exp(6):1 odds vs the worst class)
    cv_stale_s: float = 0.75       # older than this contributes nothing at all, not "unknown"
    prior_strength: float = 1.0    # weight of the uniform prior at birth


@dataclass
class PositionConfig:
    """2-D Kalman filter for a static object."""

    sigma_range_lidar_m: float = 0.08
    sigma_bearing_lidar_rad: float = math.radians(0.6)
    sigma_range_camera_frac: float = 0.12   # pinhole distance is good to roughly +/-12 %
    sigma_bearing_camera_rad: float = math.radians(2.0)
    process_noise_m2_per_s: float = 1e-4    # objects are moored, not nailed down
    # Correlated GPS bias does not average away, so the filter is forbidden from claiming better
    # than this. Without it the covariance collapses and NEES explodes.
    covariance_floor_m: float = 0.25
    birth_sigma_m: float = 3.0


@dataclass
class AssociationConfig:
    """Map-frame gating and assignment."""

    # 99 % of a chi-square with 2 dof. Generous, because the hard metric gate below is the real
    # safety net and an over-tight statistical gate just manufactures duplicate objects.
    mahalanobis_gate: float = 9.21
    # Gate buoys are 2.4384 m apart (8 ft). Staying well under that makes merging adjacent gate
    # buoys into one object geometrically impossible, which is the failure that would silently
    # destroy every downstream number.
    max_distance_m: float = 1.50
    # If the best and second-best candidate are this close in Mahalanobis distance, the measurement
    # is ambiguous. Deferring beats guessing: a wrong association corrupts two objects at once.
    ambiguity_margin: float = 2.0


@dataclass
class VisibilityConfig:
    """Whether a miss is evidence of absence or merely evidence of not looking."""

    max_range_m: float = 35.0
    min_range_m: float = 0.8
    fov_rad: float = 2.0 * math.pi    # the LiDAR is a 360-degree scanner
    occlusion_object_radius_m: float = 0.3
    occlusion_min_p_exist: float = 0.6   # only a believed object can shadow another


@dataclass
class ScorerConfig:
    viewpoint: ViewpointConfig = field(default_factory=ViewpointConfig)
    existence: ExistenceConfig = field(default_factory=ExistenceConfig)
    classifier: ClassifierConfig = field(default_factory=ClassifierConfig)
    position: PositionConfig = field(default_factory=PositionConfig)
    association: AssociationConfig = field(default_factory=AssociationConfig)
    visibility: VisibilityConfig = field(default_factory=VisibilityConfig)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'ScorerConfig':
        """Build from a nested dict (e.g. parsed YAML). Unknown keys raise rather than pass
        silently -- a typo'd tunable that quietly does nothing is a debugging tar pit."""
        sections = {
            'viewpoint': ViewpointConfig,
            'existence': ExistenceConfig,
            'classifier': ClassifierConfig,
            'position': PositionConfig,
            'association': AssociationConfig,
            'visibility': VisibilityConfig,
        }
        kwargs = {}
        for name, cls in sections.items():
            payload = (data or {}).get(name) or {}
            valid = {f for f in cls.__dataclass_fields__}
            unknown = set(payload) - valid
            if unknown:
                raise ValueError(f'unknown {name} config key(s): {sorted(unknown)}')
            kwargs[name] = cls(**payload)
        unknown_sections = set(data or {}) - set(sections)
        if unknown_sections:
            raise ValueError(f'unknown config section(s): {sorted(unknown_sections)}')
        return ScorerConfig(**kwargs)
