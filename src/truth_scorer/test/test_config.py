"""Config: a typo'd tunable that silently does nothing is a debugging tar pit."""

import pytest

from truth_scorer.core.config import ScorerConfig


def test_defaults_round_trip():
    cfg = ScorerConfig()
    assert ScorerConfig.from_dict(cfg.to_dict()).to_dict() == cfg.to_dict()


def test_partial_override_keeps_other_defaults():
    cfg = ScorerConfig.from_dict({'existence': {'p0': 0.4}})
    assert cfg.existence.p0 == 0.4
    assert cfg.existence.p_detect == ScorerConfig().existence.p_detect
    assert cfg.association.max_distance_m == ScorerConfig().association.max_distance_m


def test_empty_dict_gives_defaults():
    assert ScorerConfig.from_dict({}).to_dict() == ScorerConfig().to_dict()
    assert ScorerConfig.from_dict(None).to_dict() == ScorerConfig().to_dict()


def test_unknown_key_raises_rather_than_passing_silently():
    with pytest.raises(ValueError, match='unknown existence config key'):
        ScorerConfig.from_dict({'existence': {'p_detectt': 0.9}})


def test_unknown_section_raises():
    with pytest.raises(ValueError, match='unknown config section'):
        ScorerConfig.from_dict({'existance': {}})


def test_safety_critical_defaults_are_sane():
    """These are the ones where a bad value produces plausible-looking nonsense."""
    cfg = ScorerConfig()
    assert 0.0 < cfg.existence.p0 < 1.0
    assert cfg.existence.p_detect > cfg.existence.clutter_rate, (
        'a detection must be evidence *for* existence')
    assert cfg.existence.logit_min < 0 < cfg.existence.logit_max
    assert cfg.position.covariance_floor_m > 0.0, 'without a floor the filter claims certainty'
    assert cfg.classifier.score_floor < cfg.classifier.score_reference
    assert cfg.association.max_distance_m < 2.4384, 'gate buoys would be able to merge'
    assert cfg.viewpoint.min_weight > 0.0
    assert cfg.viewpoint.min_weight < 1.0, 'no discount means no correlation handling at all'
