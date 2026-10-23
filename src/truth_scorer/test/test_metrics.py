"""Metrics: a measuring instrument that is wrong is worse than no instrument.

Every number in the run report comes from this module, so a bug here does not produce an obvious
crash -- it produces a confident, plausible, wrong verdict about the scorer. The assignment solver
in particular is hand-written, so it is checked against brute force rather than taken on trust.
"""

import itertools
import math
import random

import pytest

from truth_scorer.core.metrics import (
    ClassOutcome,
    brier_score,
    churn_summary,
    class_summary,
    expected_calibration_error,
    flicker_rate,
    hungarian,
    match_to_truth,
    nees,
    nees_summary,
    percentiles,
    precision_recall_sweep,
    reliability_bins,
)


def _brute_force_cost(cost):
    """Optimal assignment by exhaustive search. Only usable for tiny matrices."""
    n, m = len(cost), len(cost[0])
    best = math.inf
    k = min(n, m)
    for rows in itertools.combinations(range(n), k):
        for cols in itertools.permutations(range(m), k):
            total = sum(cost[r][c] for r, c in zip(rows, cols))
            best = min(best, total)
    return best


# -- assignment ---------------------------------------------------------------------------------

def test_hungarian_matches_brute_force_on_random_squares():
    rng = random.Random(17)
    for _ in range(40):
        n = rng.randint(1, 5)
        cost = [[rng.uniform(0, 10) for _ in range(n)] for _ in range(n)]
        pairs = hungarian(cost)
        assert len(pairs) == n, 'a square matrix must be fully assigned'
        total = sum(cost[i][j] for i, j in pairs)
        assert total == pytest.approx(_brute_force_cost(cost), abs=1e-9)


def test_hungarian_matches_brute_force_on_random_rectangles():
    rng = random.Random(23)
    for _ in range(40):
        n, m = rng.randint(1, 5), rng.randint(1, 5)
        cost = [[rng.uniform(0, 10) for _ in range(m)] for _ in range(n)]
        pairs = hungarian(cost)
        assert len(pairs) == min(n, m)
        total = sum(cost[i][j] for i, j in pairs)
        assert total == pytest.approx(_brute_force_cost(cost), abs=1e-9)


def test_hungarian_is_one_to_one():
    rng = random.Random(5)
    cost = [[rng.uniform(0, 1) for _ in range(6)] for _ in range(4)]
    pairs = hungarian(cost)
    assert len({i for i, _ in pairs}) == len(pairs)
    assert len({j for _, j in pairs}) == len(pairs)


def test_hungarian_handles_empty():
    assert hungarian([]) == []
    assert hungarian([[]]) == []


# -- matching to truth ---------------------------------------------------------------------------

def test_perfect_match_is_exact():
    truth = [(0.0, 0.0), (5.0, 0.0), (0.0, 5.0)]
    result = match_to_truth(truth, truth, 2.0)
    assert len(result.matches) == 3
    assert not result.unmatched_scored and not result.unmatched_truth
    assert max(result.distances) == pytest.approx(0.0)


def test_gate_excludes_far_pairs_rather_than_forcing_them():
    truth = [(0.0, 0.0)]
    scored = [(50.0, 0.0)]
    result = match_to_truth(scored, truth, 2.0)
    assert result.matches == []
    assert result.unmatched_scored == [0]
    assert result.unmatched_truth == [0]


def test_global_match_beats_greedy_on_a_buoy_line():
    """The case that motivates using an optimal solver.

    Greedy nearest-first would grab the 0.9 m pair first and then cascade; the optimal
    assignment keeps both objects on their own truth element.
    """
    truth = [(0.0, 0.0), (2.4384, 0.0)]
    scored = [(0.9, 0.0), (2.0, 0.0)]
    result = match_to_truth(scored, truth, 2.0)
    assert sorted(result.matches) == [(0, 0), (1, 1)]


def test_matching_is_one_to_one_even_with_duplicates():
    truth = [(0.0, 0.0)]
    scored = [(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)]
    result = match_to_truth(scored, truth, 2.0)
    assert len(result.matches) == 1, 'one truth element cannot absorb three objects'
    assert len(result.unmatched_scored) == 2


# -- precision / recall ---------------------------------------------------------------------------

def test_pr_sweep_trades_recall_for_precision():
    truth = [(0.0, 0.0), (10.0, 0.0)]
    scored = [(0.0, 0.0), (10.0, 0.0), (100.0, 0.0)]   # the third is a phantom
    probs = [0.95, 0.80, 0.20]
    points = {p.threshold: p for p in precision_recall_sweep(scored, probs, truth,
                                                             thresholds=(0.1, 0.5, 0.9))}
    assert points[0.1].precision == pytest.approx(2 / 3), 'phantom included at a low threshold'
    assert points[0.5].precision == pytest.approx(1.0), 'phantom filtered out'
    assert points[0.5].recall == pytest.approx(1.0)
    assert points[0.9].recall == pytest.approx(0.5), 'raising the bar costs a real object'


def test_pr_rematches_at_each_threshold():
    """A low-confidence object must not be able to squat on a truth slot and depress precision
    for the high-confidence object that should have claimed it."""
    truth = [(0.0, 0.0)]
    scored = [(0.4, 0.0), (0.5, 0.0)]
    probs = [0.1, 0.9]
    points = {p.threshold: p for p in precision_recall_sweep(scored, probs, truth,
                                                             thresholds=(0.5,))}
    assert points[0.5].true_positives == 1
    assert points[0.5].false_positives == 0


def test_pr_point_f1_is_nan_when_undefined():
    points = precision_recall_sweep([], [], [], thresholds=(0.5,))
    assert math.isnan(points[0].precision)


# -- calibration ----------------------------------------------------------------------------------

def test_brier_rewards_being_right_and_sure():
    assert brier_score([1.0, 0.0], [1, 0]) == pytest.approx(0.0)
    assert brier_score([0.5, 0.5], [1, 0]) == pytest.approx(0.25)
    assert brier_score([0.0, 1.0], [1, 0]) == pytest.approx(1.0)


def test_perfectly_calibrated_input_has_near_zero_ece():
    rng = random.Random(99)
    probs, outcomes = [], []
    for _ in range(20000):
        p = rng.choice([0.1, 0.3, 0.5, 0.7, 0.9])
        probs.append(p)
        outcomes.append(1 if rng.random() < p else 0)
    assert expected_calibration_error(probs, outcomes) < 0.02


def test_overconfident_input_has_large_ece():
    """Claiming 0.95 while being right half the time must be caught, loudly."""
    probs = [0.95] * 1000
    outcomes = [1 if i % 2 == 0 else 0 for i in range(1000)]
    assert expected_calibration_error(probs, outcomes) == pytest.approx(0.45, abs=0.01)


def test_reliability_bins_partition_the_input():
    probs = [0.05, 0.15, 0.95, 1.0]
    bins = reliability_bins(probs, [0, 1, 1, 1], n_bins=10)
    assert sum(b.count for b in bins) == len(probs), 'every sample must land in exactly one bin'
    assert bins[-1].count == 2, 'p=1.0 belongs in the top bin, not off the end'


def test_ece_of_nothing_is_nan_not_zero():
    """An empty run must not report perfect calibration."""
    assert math.isnan(expected_calibration_error([], []))
    assert math.isnan(brier_score([], []))


# -- position -------------------------------------------------------------------------------------

def test_nees_of_a_unit_circle_is_the_squared_radius():
    cov = [1.0, 0.0, 0.0, 1.0]
    assert nees(1.0, 0.0, cov) == pytest.approx(1.0)
    assert nees(0.0, 2.0, cov) == pytest.approx(4.0)
    assert nees(3.0, 4.0, cov) == pytest.approx(25.0)


def test_nees_accounts_for_anisotropy():
    """An error along the well-known axis must cost more than the same error along the poor one."""
    cov = [0.01, 0.0, 0.0, 1.0]      # precise in east, vague in north
    assert nees(0.5, 0.0, cov) > nees(0.0, 0.5, cov)


def test_nees_averages_two_for_an_honest_filter():
    rng = random.Random(1234)
    sigma_e, sigma_n = 0.3, 1.2
    cov = [sigma_e ** 2, 0.0, 0.0, sigma_n ** 2]
    values = [nees(rng.gauss(0, sigma_e), rng.gauss(0, sigma_n), cov) for _ in range(20000)]
    assert nees_summary(values)['mean'] == pytest.approx(2.0, abs=0.1)
    assert nees_summary(values)['verdict'] == 'consistent'


def test_nees_verdict_names_the_dangerous_direction():
    assert 'over-confident' in nees_summary([8.0] * 10)['verdict']
    assert 'conservative' in nees_summary([0.3] * 10)['verdict']


def test_nees_of_a_singular_covariance_is_nan_not_an_exception():
    assert math.isnan(nees(1.0, 1.0, [0.0, 0.0, 0.0, 0.0]))


def test_percentiles_interpolate():
    assert percentiles([1.0, 2.0, 3.0, 4.0], (50,))['p50'] == pytest.approx(2.5)
    assert percentiles([5.0], (50, 90))['p50'] == pytest.approx(5.0)
    assert math.isnan(percentiles([], (50,))['p50'])


# -- class -----------------------------------------------------------------------------------------

def _outcome(pred, truth_id, acceptable, p_true, colour='red', truth_colour='red', entropy=0.5):
    return ClassOutcome(predicted_id=pred, predicted_prob=0.8, entropy=entropy,
                        truth_canonical_id=truth_id, acceptable_ids=frozenset(acceptable),
                        predicted_colour=colour, truth_colour=truth_colour, p_true_class=p_true)


def test_equivalence_group_counts_as_correct():
    """red_pole_buoy for a red sur_mark is our bookkeeping, not a perception error."""
    o = _outcome(pred=4, truth_id=3, acceptable=[3, 4, 9], p_true=0.7)
    assert o.class_correct
    assert o.colour_correct


def test_wrong_colour_is_always_wrong():
    o = _outcome(pred=1, truth_id=3, acceptable=[3, 4, 9], p_true=0.05,
                 colour='green', truth_colour='red')
    assert not o.class_correct
    assert not o.colour_correct


def test_nll_prefers_honest_uncertainty_to_confident_error():
    """The headline property. Under broken CV a scorer that shrugs must score better than one
    that guesses wrong, and top-1 alone cannot see the difference."""
    shrug = [_outcome(pred=255, truth_id=3, acceptable=[3, 4, 9], p_true=0.25, entropy=2.5)]
    wrong = [_outcome(pred=1, truth_id=3, acceptable=[3, 4, 9], p_true=0.01, entropy=0.1,
                      colour='green')]
    assert class_summary(shrug)['top1'] == class_summary(wrong)['top1'] == 0.0
    assert class_summary(shrug)['mean_nll'] < class_summary(wrong)['mean_nll']


def test_colour_accuracy_ignores_elements_with_no_colour():
    outcomes = [
        _outcome(pred=7, truth_id=7, acceptable=[7], p_true=0.9, colour=None, truth_colour=None),
        _outcome(pred=3, truth_id=3, acceptable=[3], p_true=0.9),
    ]
    summary = class_summary(outcomes)
    assert summary['colour_count'] == 1, 'a dock has no colour to get right or wrong'
    assert summary['colour_accuracy'] == pytest.approx(1.0)


def test_colour_accuracy_excludes_colours_the_vocabulary_cannot_express():
    """A red cross is a red object the detector has no way to call red.

    `cross` and `triangle` are colourless in class_mapping.yaml, so every answer the detector is
    allowed to give for a Task 5 dock indicator scores as a colour miss. Counting those drags
    colour accuracy down *and* pins it: it would read the same however good CV became. They are
    reported as a vocabulary gap instead.
    """
    indicator = _outcome(pred=6, truth_id=6, acceptable=[6, 8], p_true=0.9,
                         colour=None, truth_colour='red')
    indicator.colour_expressible = False
    buoy = _outcome(pred=3, truth_id=3, acceptable=[3], p_true=0.9)

    summary = class_summary([indicator, buoy])
    assert summary['colour_count'] == 1, 'only the buoy is scoreable on colour'
    assert summary['colour_inexpressible_count'] == 1
    assert summary['colour_accuracy'] == pytest.approx(1.0), (
        'the detector got everything it could get right')
    # The element is still scored on class, so the gap cannot be used to duck a real error.
    assert summary['top1'] == pytest.approx(1.0)
    assert summary['count'] == 2


def test_colour_expressibility_is_per_element_not_global():
    """'red' is expressible via red_buoy, but not for an element whose canonical class is a shape.

    Asked against the canonical class: a dock indicator's acceptable set lists the red buoy
    classes as scoring leniency, but the answer a correct detector gives is `cross`.
    """
    from truth_scorer.core.classes import colour_is_expressible
    assert colour_is_expressible('red', 3)        # red_buoy is red
    assert colour_is_expressible('green', 10)     # green_indicator_buoy is green
    assert not colour_is_expressible('red', 6)    # cross is colourless
    assert not colour_is_expressible('green', 8)  # triangle is colourless
    assert not colour_is_expressible(None, 3)


def test_empty_class_summary_is_nan_not_perfect():
    assert math.isnan(class_summary([])['top1'])


# -- stability ---------------------------------------------------------------------------------------

def test_flicker_rate_counts_changes_not_values():
    assert flicker_rate({1: [3, 3, 3, 3]})['overall'] == pytest.approx(0.0)
    assert flicker_rate({1: [3, 1, 3, 1]})['overall'] == pytest.approx(1.0)
    assert flicker_rate({1: [3, 3, 1, 1]})['overall'] == pytest.approx(1 / 3)


def test_flicker_per_object_is_not_dominated_by_one_long_track():
    stable_long = list(range(0))
    seqs = {1: [3] * 1000, 2: [3, 1]}
    assert flicker_rate(seqs)['mean_per_object'] == pytest.approx(0.5)
    assert flicker_rate(seqs)['overall'] < 0.01
    assert stable_long == []


def test_flicker_ignores_objects_seen_once():
    assert math.isnan(flicker_rate({1: [3]})['mean_per_object'])


def test_churn_summary_reports_the_worst_case():
    s = churn_summary([1, 1, 1, 8])
    assert s['max'] == 8, 'the re-acquired-eight-times object is the whole point'
    assert s['objects_with_churn'] == 1
    assert s['mean'] == pytest.approx(2.75)
