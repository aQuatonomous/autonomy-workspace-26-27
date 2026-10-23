"""Existence filter: does p_exist behave like a probability, or just like a counter?"""

import math

import pytest

from truth_scorer.core.config import ExistenceConfig
from truth_scorer.core.existence import ExistenceFilter, logit, sigmoid


def test_starts_at_prior():
    f = ExistenceFilter(ExistenceConfig(p0=0.3))
    assert f.p_exist == pytest.approx(0.3, abs=1e-9)


def test_logit_sigmoid_roundtrip():
    for p in (0.01, 0.25, 0.5, 0.75, 0.99):
        assert sigmoid(logit(p)) == pytest.approx(p, abs=1e-9)


def test_hits_increase_confidence_monotonically():
    f = ExistenceFilter()
    previous = f.p_exist
    for _ in range(10):
        f.update_hit(1.0)
        assert f.p_exist >= previous
        previous = f.p_exist
    assert f.p_exist > 0.9


def test_confidence_is_bounded_and_never_certain():
    """The single most important property: the filter must never report 1.0."""
    cfg = ExistenceConfig(logit_max=4.0)
    f = ExistenceFilter(cfg)
    for _ in range(1000):
        f.update_hit(1.0)
    assert f.p_exist < 1.0
    assert f.p_exist == pytest.approx(sigmoid(4.0), abs=1e-9)
    assert f.logit_value <= cfg.logit_max + 1e-12


def test_floor_is_bounded_too():
    cfg = ExistenceConfig(logit_min=-4.0)
    f = ExistenceFilter(cfg)
    for _ in range(1000):
        f.update_expected_miss(1.0)
    assert f.p_exist > 0.0
    assert f.p_exist == pytest.approx(sigmoid(-4.0), abs=1e-9)


def test_expected_misses_reduce_confidence():
    f = ExistenceFilter()
    for _ in range(5):
        f.update_hit(1.0)
    high = f.p_exist
    for _ in range(5):
        f.update_expected_miss(1.0)
    assert f.p_exist < high


def test_invisible_is_not_a_miss():
    """A buoy astern is not evidence of absence. This is the bug the module exists to avoid."""
    a = ExistenceFilter()
    b = ExistenceFilter()
    for f in (a, b):
        for _ in range(5):
            f.update_hit(1.0)
    before = a.p_exist

    a.update_invisible(0.4)          # not looked at for 0.4 s
    b.update_expected_miss(1.0)      # looked at and not seen

    assert a.p_exist > b.p_exist
    assert a.p_exist == pytest.approx(before, abs=0.02)   # barely moved


def test_invisible_decays_toward_prior_but_never_past_it():
    cfg = ExistenceConfig(p0=0.3, decay_tau_s=10.0)
    f = ExistenceFilter(cfg)
    for _ in range(10):
        f.update_hit(1.0)
    assert f.p_exist > 0.9

    for _ in range(400):
        f.update_invisible(1.0)      # 400 s of not looking

    assert f.p_exist == pytest.approx(0.3, abs=1e-3)
    assert f.p_exist >= 0.3 - 1e-6   # relaxes *to* the prior, never below it


def test_invisible_decay_from_below_rises_to_prior():
    """Symmetry check: decay is relaxation, not decrement."""
    cfg = ExistenceConfig(p0=0.3, decay_tau_s=10.0)
    f = ExistenceFilter(cfg)
    for _ in range(10):
        f.update_expected_miss(1.0)
    low = f.p_exist
    assert low < 0.3
    for _ in range(400):
        f.update_invisible(1.0)
    assert f.p_exist > low
    assert f.p_exist == pytest.approx(0.3, abs=1e-3)


def test_recovers_after_spurious_misses():
    """Occlusion the visibility model failed to catch must not permanently kill an object."""
    f = ExistenceFilter()
    for _ in range(8):
        f.update_hit(1.0)
    for _ in range(3):
        f.update_expected_miss(1.0)
    dipped = f.p_exist
    for _ in range(3):
        f.update_hit(1.0)
    assert f.p_exist > dipped
    assert f.p_exist > 0.9


def test_zero_weight_updates_are_exact_no_ops():
    f = ExistenceFilter()
    f.update_hit(1.0)
    state = (f.logit_value, f.hits, f.expected_misses)
    f.update_hit(0.0)
    f.update_expected_miss(0.0)
    assert (f.logit_value, f.hits, f.expected_misses) == state


def test_weight_scales_evidence():
    strong = ExistenceFilter()
    weak = ExistenceFilter()
    strong.update_hit(1.0)
    weak.update_hit(0.1)
    assert strong.p_exist > weak.p_exist


def test_consecutive_miss_counter_resets_on_hit():
    f = ExistenceFilter()
    f.update_expected_miss(1.0)
    f.update_expected_miss(1.0)
    assert f.consecutive_expected_misses == 2
    f.update_hit(1.0)
    assert f.consecutive_expected_misses == 0


def test_prune_requires_both_disbelief_and_sustained_misses():
    cfg = ExistenceConfig(prune_below_p=0.05, prune_after_expected_misses=12)
    f = ExistenceFilter(cfg)
    assert not f.should_prune()

    # Low confidence alone is not enough -- a newly born object is also low confidence.
    for _ in range(5):
        f.update_expected_miss(1.0)
    assert not f.should_prune()

    for _ in range(20):
        f.update_expected_miss(1.0)
    assert f.should_prune()


def test_hit_evidence_matches_the_likelihood_ratio():
    """The per-hit step must be log(P_D / clutter), not an arbitrary constant."""
    cfg = ExistenceConfig(p_detect=0.8, clutter_rate=0.2)
    f = ExistenceFilter(cfg)
    start = f.logit_value
    f.update_hit(1.0)
    assert f.logit_value - start == pytest.approx(math.log(0.8 / 0.2), abs=1e-12)
