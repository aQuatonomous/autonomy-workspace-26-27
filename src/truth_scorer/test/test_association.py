"""Association: the gate that must never merge two gate buoys."""

import math

import numpy as np
import pytest

from truth_scorer.core.association import Measurement, associate
from truth_scorer.core.config import AssociationConfig, PositionConfig
from truth_scorer.core.position import PositionFilter

GATE_BUOY_SPACING_M = 2.4384   # 8 ft, per the RoboBoat handbook


class FakeObject:
    """Minimal stand-in exposing the interface `associate` needs."""

    def __init__(self, east, north, sigma=0.5):
        self._f = PositionFilter(east, north, PositionConfig(birth_sigma_m=sigma,
                                                             covariance_floor_m=0.0))

    @property
    def east(self):
        return self._f.east

    @property
    def north(self):
        return self._f.north

    def mahalanobis_squared(self, east, north, extra_covariance=None):
        return self._f.mahalanobis_squared(east, north, extra_covariance)


def measurement(east, north, **kw):
    r = math.hypot(east, north)
    return Measurement(east=east, north=north, range_m=r,
                       global_bearing_rad=math.atan2(north, east), **kw)


def test_no_objects_means_everything_is_a_birth():
    res = associate([], [measurement(1.0, 1.0), measurement(5.0, 5.0)])
    assert res.births == [0, 1]
    assert res.matches == []


def test_no_measurements_means_everything_is_unmatched():
    res = associate([FakeObject(0, 0), FakeObject(10, 10)], [])
    assert res.unmatched_objects == [0, 1]


def test_obvious_match():
    res = associate([FakeObject(10.0, 10.0)], [measurement(10.05, 9.97)])
    assert res.matches == [(0, 0)]
    assert res.births == []


def test_far_measurement_births_instead_of_matching():
    res = associate([FakeObject(10.0, 10.0)], [measurement(30.0, 30.0)])
    assert res.matches == []
    assert res.births == [0]


def test_hard_distance_gate_is_enforced_regardless_of_covariance():
    """Even an absurdly uncertain object cannot claim a measurement past the metric backstop."""
    cfg = AssociationConfig(max_distance_m=1.5)
    loose = FakeObject(0.0, 0.0, sigma=50.0)   # covariance so large the chi-square gate is useless
    res = associate([loose], [measurement(3.0, 0.0)], cfg)
    assert res.matches == []
    assert res.births == [0]


def test_adjacent_gate_buoys_never_merge():
    """The failure that would silently destroy every downstream number."""
    cfg = AssociationConfig()
    red = FakeObject(0.0, 0.0)
    green = FakeObject(GATE_BUOY_SPACING_M, 0.0)
    res = associate([red, green],
                    [measurement(0.02, -0.01), measurement(GATE_BUOY_SPACING_M - 0.03, 0.02)],
                    cfg)
    assert len(res.matches) == 2
    assert {o for o, _ in res.matches} == {0, 1}
    # Crucially, each object got its *own* measurement.
    assert dict(res.matches) == {0: 0, 1: 1}


def test_gate_buoy_spacing_exceeds_the_hard_gate():
    """A structural guarantee, asserted so a future tuning change cannot quietly break it."""
    assert AssociationConfig().max_distance_m < GATE_BUOY_SPACING_M


def test_one_measurement_is_assigned_to_at_most_one_object():
    objs = [FakeObject(0.0, 0.0), FakeObject(0.6, 0.0)]
    res = associate(objs, [measurement(0.3, 0.0)])
    assert len(res.matches) <= 1


def test_ambiguous_measurement_is_deferred_not_guessed():
    """Exactly equidistant between two objects: guessing corrupts two objects at once."""
    cfg = AssociationConfig(ambiguity_margin=2.0)
    objs = [FakeObject(-0.5, 0.0), FakeObject(0.5, 0.0)]
    res = associate(objs, [measurement(0.0, 0.0)], cfg)
    assert res.matches == []
    assert res.deferred == [0]
    assert res.births == []


def test_deferred_measurement_does_not_punish_the_contested_objects():
    """A detection *did* land on them; charging a miss would punish our own indecision."""
    cfg = AssociationConfig(ambiguity_margin=2.0)
    objs = [FakeObject(-0.5, 0.0), FakeObject(0.5, 0.0)]
    res = associate(objs, [measurement(0.0, 0.0)], cfg)
    assert res.unmatched_objects == []
    assert res.ambiguous_objects == [0, 1]


def test_clear_winner_is_not_treated_as_ambiguous():
    cfg = AssociationConfig(ambiguity_margin=2.0)
    objs = [FakeObject(0.0, 0.0, sigma=0.2), FakeObject(1.2, 0.0, sigma=0.2)]
    res = associate(objs, [measurement(0.02, 0.0)], cfg)
    assert res.matches == [(0, 0)]
    assert res.deferred == []


def test_best_candidate_wins_when_several_compete():
    objs = [FakeObject(0.0, 0.0, sigma=0.3), FakeObject(1.4, 0.0, sigma=0.3)]
    res = associate(objs, [measurement(1.38, 0.01)], AssociationConfig(ambiguity_margin=0.5))
    assert res.matches == [(1, 0)]


def test_measurement_covariance_is_used_when_supplied():
    """A measurement with huge uncertainty should gate in where a precise one would not."""
    cfg = AssociationConfig(mahalanobis_gate=9.21, max_distance_m=1.5)
    obj = FakeObject(0.0, 0.0, sigma=0.1)
    m = measurement(0.9, 0.0)

    tight = associate([obj], [m], cfg, measurement_covariances=[np.diag([0.01, 0.01])])
    loose = associate([obj], [m], cfg, measurement_covariances=[np.diag([4.0, 4.0])])

    assert tight.matches == []     # 9 sigma away: not the same object
    assert loose.matches == [(0, 0)]


def test_every_measurement_is_accounted_for_exactly_once():
    objs = [FakeObject(0.0, 0.0), FakeObject(10.0, 0.0)]
    ms = [measurement(0.1, 0.0), measurement(10.1, 0.0), measurement(50.0, 50.0)]
    res = associate(objs, ms)
    seen = set(res.matched_measurement_indices) | set(res.births) | set(res.deferred)
    assert seen == {0, 1, 2}
    assert len(seen) == len(ms)


def test_every_object_is_accounted_for_exactly_once():
    objs = [FakeObject(0.0, 0.0), FakeObject(10.0, 0.0), FakeObject(-30.0, 0.0)]
    res = associate(objs, [measurement(0.05, 0.0)])
    seen = (set(res.matched_object_indices) | set(res.unmatched_objects)
            | set(res.ambiguous_objects))
    assert seen == {0, 1, 2}
