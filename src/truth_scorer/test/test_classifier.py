"""Class posterior: does it converge, stay bounded, and refuse to be bullied by noise?"""

import math

import pytest

from truth_scorer.core.classes import NUM_CLASSES, UNKNOWN_ID
from truth_scorer.core.classifier import ClassPosterior
from truth_scorer.core.config import ClassifierConfig
from truth_scorer.core.confusion import default_confusion_matrix

RED_BUOY = 3
GREEN_BUOY = 1
RED_POLE = 4
BLACK_BUOY = 0
CROSS = 6                 # colourless placard shape
RED_INDICATOR_BUOY = 9


def test_prior_is_uniform_and_maximally_uncertain():
    c = ClassPosterior()
    probs = c.probabilities()
    assert sum(probs) == pytest.approx(1.0, abs=1e-9)
    assert all(p == pytest.approx(1.0 / NUM_CLASSES) for p in probs)
    assert c.entropy() == pytest.approx(math.log(NUM_CLASSES), abs=1e-9)
    assert c.is_uninformed


def test_probabilities_always_sum_to_one():
    c = ClassPosterior()
    for _ in range(50):
        c.update(RED_BUOY, 0.8)
        assert sum(c.probabilities()) == pytest.approx(1.0, abs=1e-9)


def test_converges_to_the_observed_class():
    c = ClassPosterior()
    for _ in range(20):
        c.update(RED_BUOY, 0.9)
    assert c.class_id == RED_BUOY
    assert c.class_prob > 0.5
    assert c.entropy() < math.log(NUM_CLASSES)


def test_entropy_falls_as_evidence_accumulates():
    c = ClassPosterior()
    previous = c.entropy()
    for _ in range(10):
        c.update(RED_BUOY, 0.9)
        assert c.entropy() <= previous + 1e-9
        previous = c.entropy()


def test_low_score_observations_are_exact_no_ops():
    """The camera0 case: ~0.10 raw score, hundreds of frames, must change nothing at all."""
    cfg = ClassifierConfig(score_floor=0.15)
    c = ClassPosterior(cfg)
    before = c.log_probabilities()
    for _ in range(50):
        applied = c.update(RED_BUOY, 0.05)
        assert applied == 0.0
    assert c.log_probabilities() == before
    assert c.is_uninformed
    assert c.entropy() == pytest.approx(math.log(NUM_CLASSES), abs=1e-12)


def test_stale_cv_is_an_exact_no_op():
    cfg = ClassifierConfig(cv_stale_s=0.75)
    c = ClassPosterior(cfg)
    c.update(RED_BUOY, 0.9, cv_age_s=0.1)
    before = c.log_probabilities()
    applied = c.update(GREEN_BUOY, 0.95, cv_age_s=5.0)
    assert applied == 0.0
    assert c.log_probabilities() == before


def test_single_outlier_does_not_flip_an_established_posterior():
    c = ClassPosterior()
    for _ in range(15):
        c.update(RED_BUOY, 0.85)
    assert c.class_id == RED_BUOY
    c.update(GREEN_BUOY, 0.99)
    assert c.class_id == RED_BUOY, 'one confident frame should not overturn fifteen'


def test_alternating_observations_stay_uncertain_rather_than_flapping():
    """Red/green flicker should read as 'I do not know', not as a confident coin flip."""
    c = ClassPosterior()
    for _ in range(30):
        c.update(RED_BUOY, 0.8)
        c.update(GREEN_BUOY, 0.8)
    p_red = c.probability_of(RED_BUOY)
    p_green = c.probability_of(GREEN_BUOY)
    assert abs(p_red - p_green) < 0.1
    assert c.entropy() > 0.5, 'alternating evidence must leave real uncertainty'


def test_confidence_is_bounded():
    """1000 agreeing frames must not produce a probability of 1 - 1e-40."""
    cfg = ClassifierConfig(max_logprob_spread=6.0)
    c = ClassPosterior(cfg)
    for _ in range(1000):
        c.update(RED_BUOY, 1.0)
    assert c.class_prob < 1.0
    # Every class retains at least exp(-spread) relative odds against the best.
    worst = min(c.probabilities())
    assert worst > 0.0
    assert math.log(c.class_prob / worst) <= cfg.max_logprob_spread + 1e-6


def test_bounded_confidence_still_allows_reversal():
    """Bounding must not freeze the posterior: sustained contrary evidence has to win."""
    c = ClassPosterior()
    for _ in range(40):
        c.update(RED_BUOY, 0.9)
    assert c.class_id == RED_BUOY
    for _ in range(80):
        c.update(GREEN_BUOY, 0.9)
    assert c.class_id == GREEN_BUOY


def test_within_group_confusion_is_cheaper_than_across_group():
    """Seeing 'red_pole_buoy' should barely dent a 'red_buoy' belief; 'black_buoy' should dent it."""
    a = ClassPosterior()
    b = ClassPosterior()
    for c in (a, b):
        for _ in range(10):
            c.update(RED_BUOY, 0.9)
    baseline = a.probability_of(RED_BUOY)

    a.update(RED_POLE, 0.9)
    b.update(BLACK_BUOY, 0.9)

    assert a.probability_of(RED_BUOY) > b.probability_of(RED_BUOY)
    assert baseline - a.probability_of(RED_BUOY) < baseline - b.probability_of(RED_BUOY)


def test_score_weight_mapping():
    c = ClassPosterior(ClassifierConfig(score_floor=0.15, score_reference=0.60))
    assert c.score_weight(0.05) == 0.0
    assert c.score_weight(0.15) == 0.0
    assert c.score_weight(0.60) == pytest.approx(1.0)
    assert c.score_weight(0.95) == pytest.approx(1.0)
    assert 0.0 < c.score_weight(0.30) < 1.0


def test_viewpoint_weight_scales_class_evidence():
    strong = ClassPosterior()
    weak = ClassPosterior()
    strong.update(RED_BUOY, 0.9, viewpoint_weight=1.0)
    weak.update(RED_BUOY, 0.9, viewpoint_weight=0.05)
    assert strong.probability_of(RED_BUOY) > weak.probability_of(RED_BUOY)


def test_colour_marginal_aggregates_correctly():
    c = ClassPosterior()
    for _ in range(20):
        c.update(RED_BUOY, 0.9)
    colours = c.colour_probabilities()
    assert sum(colours.values()) == pytest.approx(1.0, abs=1e-9)
    assert colours['red'] >= c.probability_of(RED_BUOY)
    assert c.argmax_colour()[0] == 'red'


def test_colour_marginal_beats_the_argmax_class_colour_on_a_red_placard():
    """The Task 5 dock indicator case, and the reason colour is on the wire separately.

    A red cross placard splits its evidence between `cross` -- which is colourless in
    class_mapping.yaml -- and the three red buoy classes. `cross` can win the argmax outright
    while the summed red mass is larger, so reading colour off the argmax class reports "no
    colour" for an object the posterior knows perfectly well is red. Colour is what the handbook
    scores, so it must be marginalised, not derived.
    """
    from truth_scorer.core.classes import class_colour

    c = ClassPosterior()
    # Confident about the shape ...
    for _ in range(4):
        c.update(CROSS, 0.9)
    # ... and less sure which red thing it is, with that mass split three ways. This is the
    # realistic shape of the evidence: the placard's outline is crisp, its colour is one weak
    # cue spread over every red class in the vocabulary.
    c.update(RED_BUOY, 0.4)
    c.update(RED_POLE, 0.4)
    c.update(RED_INDICATOR_BUOY, 0.4)

    argmax_id, _ = c.argmax()
    assert class_colour(argmax_id) is None, 'precondition: a colourless class wins the argmax'

    colour, prob = c.argmax_colour()
    assert colour == 'red', 'the summed red mass is the honest answer'
    assert prob > c.probability_of(argmax_id)


def test_unknown_is_a_real_hypothesis():
    """An object nothing matches should accumulate unknown mass, not a confident wrong label."""
    c = ClassPosterior()
    for _ in range(20):
        c.update(UNKNOWN_ID, 0.9)
    assert c.probability_of(UNKNOWN_ID) > 1.0 / NUM_CLASSES


def test_snapshot_is_self_consistent():
    c = ClassPosterior()
    for _ in range(10):
        c.update(RED_BUOY, 0.9)
    snap = c.snapshot()
    assert snap['class_id'] == c.class_id
    assert snap['class_prob'] == pytest.approx(c.class_prob)
    assert snap['entropy'] == pytest.approx(c.entropy())


# -- the confusion matrix itself ---------------------------------------------------------------


def test_confusion_rows_are_distributions():
    m = default_confusion_matrix()
    assert len(m) == NUM_CLASSES
    for row in m:
        assert len(row) == NUM_CLASSES
        assert sum(row) == pytest.approx(1.0, abs=1e-9)


def test_confusion_has_no_zeros():
    """A zero likelihood is unrecoverable: one bad frame would kill a hypothesis forever."""
    for row in default_confusion_matrix():
        assert all(v > 0.0 for v in row)


def test_confusion_diagonal_dominates():
    """Every row, including unknown. A row where the correct reading is not the most likely one
    makes that observation evidence *against* the truth -- which is how the unknown hypothesis
    was originally being driven to zero by observations of unknown."""
    m = default_confusion_matrix()
    for i, row in enumerate(m):
        assert row[i] == max(row), f'row {i} is not diagonally dominant'


def test_observing_a_class_is_always_evidence_for_it():
    """The general form of the bug above: for any class, P(obs=X | true=X) must exceed
    P(obs=X | true=anything else), or the filter learns backwards."""
    m = default_confusion_matrix()
    for j in range(NUM_CLASSES):
        column = [m[i][j] for i in range(NUM_CLASSES)]
        assert column[j] == max(column), f'observing class index {j} is evidence against itself'
