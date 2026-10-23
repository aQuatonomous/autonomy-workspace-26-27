"""Scoring the scorer: matching against truth, and the statistics that say whether it is honest.

Two principles run through this file.

**Calibration beats accuracy.** A scorer that says 0.9 and is right 90 % of the time is useful
even when it is often wrong; a scorer that says 0.9 and is right 60 % of the time is worse than
useless, because everything downstream trusts it. So the headline numbers here are ECE, Brier and
NEES, not top-1.

**Recall must be conditioned on opportunity.** Recall against all 44 answer-key elements measures
where the boat drove, not how well it perceived. Only objects the boat actually came within sensor
range of belong in the denominator, and the report says which denominator it used.

No ROS, no file IO, no scipy -- the assignment solver below is written out so that a report
produced on a laptop and one produced on the boat are bit-for-bit the same.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

INF = float('inf')


# ---------------------------------------------------------------------------------------------
# Assignment
# ---------------------------------------------------------------------------------------------

def hungarian(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    """Minimum-cost assignment on a rectangular matrix, O(n^2 m) shortest augmenting paths.

    Written out rather than imported from scipy so the evaluator has no optional dependency that
    could silently change a published metric depending on where it ran. Rows with no finite
    option are left unassigned.
    """
    rows = len(cost)
    if rows == 0 or len(cost[0]) == 0:
        return []
    cols = len(cost[0])

    # The JV recurrence below requires rows <= cols. Transposing is cheaper and far less
    # error-prone than padding the matrix with dummy columns.
    transposed = rows > cols
    if transposed:
        cost = [[cost[i][j] for i in range(rows)] for j in range(cols)]
        rows, cols = cols, rows

    n, m = rows, cols
    u = [0.0] * (n + 1)      # row potentials, 1-indexed
    v = [0.0] * (m + 1)      # column potentials
    p = [0] * (m + 1)        # p[j] = row currently assigned to column j (0 = free)
    way = [0] * (m + 1)      # alternating-path backpointers

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = 0
            row = cost[i0 - 1]
            for j in range(1, m + 1):
                if used[j]:
                    continue
                cur = row[j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    out = []
    for j in range(1, m + 1):
        i = p[j]
        if i:
            out.append((j - 1, i - 1) if transposed else (i - 1, j - 1))
    return sorted(out)


@dataclass
class MatchResult:
    """One scored-object <-> truth-element assignment pass at a fixed gate."""

    matches: List[Tuple[int, int]] = field(default_factory=list)   # (scored index, truth index)
    unmatched_scored: List[int] = field(default_factory=list)      # false positives
    unmatched_truth: List[int] = field(default_factory=list)       # misses
    distances: List[float] = field(default_factory=list)           # parallel to matches


def match_to_truth(
    scored_xy: Sequence[Tuple[float, float]],
    truth_xy: Sequence[Tuple[float, float]],
    max_distance_m: float = 2.0,
) -> MatchResult:
    """Globally optimal one-to-one match, gated at max_distance_m.

    Global rather than greedy on purpose: greedy nearest-neighbour on a line of gate buoys can
    cascade one bad pick into a chain of wrong matches, which then reads as a position-accuracy
    problem rather than the matching artefact it is.
    """
    if not scored_xy or not truth_xy:
        return MatchResult(unmatched_scored=list(range(len(scored_xy))),
                           unmatched_truth=list(range(len(truth_xy))))

    # Out-of-gate pairs get a large *finite* penalty rather than infinity. Infinities would
    # poison the dual potentials inside the solver; a penalty strictly larger than any admissible
    # total keeps the arithmetic well behaved, and the pairs are filtered out afterwards anyway.
    big = max_distance_m * (len(scored_xy) + len(truth_xy) + 2) + 1.0
    cost = [[0.0] * len(truth_xy) for _ in scored_xy]
    distance = [[0.0] * len(truth_xy) for _ in scored_xy]
    for i, (e, n) in enumerate(scored_xy):
        for j, (te, tn) in enumerate(truth_xy):
            d = math.hypot(e - te, n - tn)
            distance[i][j] = d
            cost[i][j] = d if d <= max_distance_m else big

    pairs = [(i, j) for i, j in hungarian(cost) if distance[i][j] <= max_distance_m]

    matched_s = {i for i, _ in pairs}
    matched_t = {j for _, j in pairs}
    return MatchResult(
        matches=pairs,
        unmatched_scored=[i for i in range(len(scored_xy)) if i not in matched_s],
        unmatched_truth=[j for j in range(len(truth_xy)) if j not in matched_t],
        distances=[distance[i][j] for i, j in pairs],
    )


# ---------------------------------------------------------------------------------------------
# Existence: precision / recall
# ---------------------------------------------------------------------------------------------

@dataclass
class PRPoint:
    threshold: float
    true_positives: int
    false_positives: int
    false_negatives: int

    @property
    def precision(self) -> float:
        d = self.true_positives + self.false_positives
        return self.true_positives / d if d else float('nan')

    @property
    def recall(self) -> float:
        d = self.true_positives + self.false_negatives
        return self.true_positives / d if d else float('nan')

    @property
    def f1(self) -> float:
        p, r = self.precision, self.recall
        if not (p == p) or not (r == r) or (p + r) == 0.0:
            return float('nan')
        return 2.0 * p * r / (p + r)

    def to_dict(self) -> Dict:
        return {'threshold': self.threshold, 'tp': self.true_positives,
                'fp': self.false_positives, 'fn': self.false_negatives,
                'precision': self.precision, 'recall': self.recall, 'f1': self.f1}


def precision_recall_sweep(
    scored_xy: Sequence[Tuple[float, float]],
    scored_p: Sequence[float],
    truth_xy: Sequence[Tuple[float, float]],
    thresholds: Sequence[float] = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9),
    max_distance_m: float = 2.0,
) -> List[PRPoint]:
    """Re-match at every threshold rather than matching once and filtering.

    Filtering a single match set would let a low-confidence object permanently occupy a truth slot
    that a high-confidence one could have claimed, which quietly depresses precision at high
    thresholds -- an artefact that looks exactly like a miscalibrated filter.
    """
    points = []
    for t in thresholds:
        keep = [i for i, p in enumerate(scored_p) if p >= t]
        sub = [scored_xy[i] for i in keep]
        result = match_to_truth(sub, truth_xy, max_distance_m)
        points.append(PRPoint(
            threshold=float(t),
            true_positives=len(result.matches),
            false_positives=len(result.unmatched_scored),
            false_negatives=len(result.unmatched_truth),
        ))
    return points


# ---------------------------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------------------------

def brier_score(probabilities: Sequence[float], outcomes: Sequence[int]) -> float:
    """Mean squared error of a probabilistic claim. Lower is better; 0.25 is "always say 0.5"."""
    if not probabilities:
        return float('nan')
    return sum((p - o) ** 2 for p, o in zip(probabilities, outcomes)) / len(probabilities)


@dataclass
class ReliabilityBin:
    low: float
    high: float
    count: int
    mean_predicted: float
    observed_rate: float

    def to_dict(self) -> Dict:
        return {'low': self.low, 'high': self.high, 'count': self.count,
                'mean_predicted': self.mean_predicted, 'observed_rate': self.observed_rate}


def reliability_bins(
    probabilities: Sequence[float],
    outcomes: Sequence[int],
    n_bins: int = 10,
) -> List[ReliabilityBin]:
    """The reliability diagram, as numbers: when it said 0.8, how often was it right?"""
    bins: List[ReliabilityBin] = []
    for b in range(n_bins):
        low = b / n_bins
        high = (b + 1) / n_bins
        if b == n_bins - 1:
            members = [(p, o) for p, o in zip(probabilities, outcomes) if low <= p <= high]
        else:
            members = [(p, o) for p, o in zip(probabilities, outcomes) if low <= p < high]
        if members:
            mean_p = sum(p for p, _ in members) / len(members)
            rate = sum(o for _, o in members) / len(members)
        else:
            mean_p = float('nan')
            rate = float('nan')
        bins.append(ReliabilityBin(low, high, len(members), mean_p, rate))
    return bins


def expected_calibration_error(
    probabilities: Sequence[float],
    outcomes: Sequence[int],
    n_bins: int = 10,
) -> float:
    """Count-weighted mean gap between claimed and observed frequency.

    This is the single number that answers "when it says 0.8, is it right 80 % of the time?".
    With only ~44 objects per run the bins are thin, so read it alongside the bin counts rather
    than on its own.
    """
    total = len(probabilities)
    if total == 0:
        return float('nan')
    error = 0.0
    for b in reliability_bins(probabilities, outcomes, n_bins):
        if b.count == 0:
            continue
        error += (b.count / total) * abs(b.mean_predicted - b.observed_rate)
    return error


# ---------------------------------------------------------------------------------------------
# Position
# ---------------------------------------------------------------------------------------------

def percentiles(values: Sequence[float], qs: Sequence[float] = (50, 90, 95)) -> Dict[str, float]:
    """Linear-interpolated percentiles, so the report does not need numpy either."""
    if not values:
        return {f'p{int(q)}': float('nan') for q in qs}
    ordered = sorted(values)
    out = {}
    for q in qs:
        if len(ordered) == 1:
            out[f'p{int(q)}'] = ordered[0]
            continue
        pos = (q / 100.0) * (len(ordered) - 1)
        lo = int(math.floor(pos))
        hi = min(lo + 1, len(ordered) - 1)
        frac = pos - lo
        out[f'p{int(q)}'] = ordered[lo] * (1.0 - frac) + ordered[hi] * frac
    return out


def nees(error_e: float, error_n: float, cov: Sequence[float]) -> float:
    """Normalised estimation error squared for a 2-D estimate, given row-major [ee, en, ne, nn].

    The honesty test for the covariance. For a correctly specified filter this averages 2.0
    (the dimension). Much above 2 means the filter is over-confident -- it is claiming an accuracy
    it does not have, which is the single most dangerous failure mode for a number a planner will
    trust. Much below 2 means it is needlessly timid.
    """
    a, b, c, d = (float(cov[0]), float(cov[1]), float(cov[2]), float(cov[3]))
    det = a * d - b * c
    if not math.isfinite(det) or abs(det) < 1e-15:
        return float('nan')
    # inverse of [[a, b], [c, d]]
    ia, ib = d / det, -b / det
    ic, idd = -c / det, a / det
    return (error_e * (ia * error_e + ib * error_n)
            + error_n * (ic * error_e + idd * error_n))


def nees_summary(values: Sequence[float]) -> Dict[str, float]:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return {'count': 0, 'mean': float('nan'), 'median': float('nan'), 'verdict': 'no data'}
    mean = sum(finite) / len(finite)
    med = percentiles(finite, (50,))['p50']
    if mean > 3.0:
        verdict = 'over-confident: covariance too small'
    elif mean < 1.2:
        verdict = 'conservative: covariance too large'
    else:
        verdict = 'consistent'
    return {'count': len(finite), 'mean': mean, 'median': med, 'verdict': verdict}


# ---------------------------------------------------------------------------------------------
# Class
# ---------------------------------------------------------------------------------------------

@dataclass
class ClassOutcome:
    """One matched object's class verdict, with everything needed to recompute the metrics."""

    predicted_id: int
    predicted_prob: float
    entropy: float
    truth_canonical_id: int
    acceptable_ids: frozenset
    predicted_colour: Optional[str]
    truth_colour: Optional[str]
    p_true_class: float = float('nan')   # posterior mass on the acceptable set, for NLL
    # True when *some* class in the vocabulary carries this element's truth colour. The Task 5
    # dock indicators are red and green crosses and triangles, but `cross` and `triangle` are
    # colourless in class_mapping.yaml, so no prediction the detector can emit will ever match.
    # Counting those as colour failures makes the metric read low *and* insensitive -- it would
    # sit at the same value however good the detector got. They are reported separately instead.
    colour_expressible: bool = True

    @property
    def class_correct(self) -> bool:
        return self.predicted_id in self.acceptable_ids

    @property
    def colour_scoreable(self) -> bool:
        return self.truth_colour is not None and self.colour_expressible

    @property
    def colour_correct(self) -> bool:
        return self.colour_scoreable and self.predicted_colour == self.truth_colour


def class_summary(outcomes: Sequence[ClassOutcome]) -> Dict:
    """Top-1, colour accuracy and mean NLL.

    NLL is the one to read when CV is degraded: it is the only one of the three that rewards
    saying "I don't know" instead of guessing. Top-1 and colour accuracy both treat a confident
    wrong answer and an honest shrug identically, and under today's cameras that difference is
    the whole point.
    """
    if not outcomes:
        return {'count': 0, 'top1': float('nan'), 'colour_accuracy': float('nan'),
                'mean_nll': float('nan'), 'mean_entropy': float('nan'),
                'colour_count': 0, 'colour_inexpressible_count': 0}
    coloured = [o for o in outcomes if o.colour_scoreable]
    inexpressible = [o for o in outcomes
                     if o.truth_colour is not None and not o.colour_expressible]
    nlls = [-math.log(max(o.p_true_class, 1e-12)) for o in outcomes
            if math.isfinite(o.p_true_class)]
    return {
        'count': len(outcomes),
        'top1': sum(o.class_correct for o in outcomes) / len(outcomes),
        'colour_accuracy': (sum(o.colour_correct for o in coloured) / len(coloured)
                            if coloured else float('nan')),
        'colour_count': len(coloured),
        'colour_inexpressible_count': len(inexpressible),
        'mean_nll': sum(nlls) / len(nlls) if nlls else float('nan'),
        'mean_entropy': sum(o.entropy for o in outcomes) / len(outcomes),
    }


# ---------------------------------------------------------------------------------------------
# Stability
# ---------------------------------------------------------------------------------------------

def flicker_rate(class_sequences: Dict[int, List[int]]) -> Dict:
    """Fraction of consecutive frames in which an object's reported class changed.

    The direct before/after number against vision_lidar_fusion's last-write-wins behaviour, where
    a single mislabelled frame flips the class outright. Measured per object and averaged, so one
    long-lived object cannot dominate.
    """
    rates = []
    changes_total = 0
    steps_total = 0
    for seq in class_sequences.values():
        if len(seq) < 2:
            continue
        changes = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        rates.append(changes / (len(seq) - 1))
        changes_total += changes
        steps_total += len(seq) - 1
    return {
        'objects': len(rates),
        'mean_per_object': sum(rates) / len(rates) if rates else float('nan'),
        'overall': changes_total / steps_total if steps_total else float('nan'),
        'changes': changes_total,
        'transitions': steps_total,
    }


def churn_summary(churn_counts: Sequence[int]) -> Dict:
    """How many LiDAR track ids each persistent object absorbed.

    Anything above 1 is a case where today's track-id-keyed map would have thrown the object away
    and started from scratch. `max` matters more than `mean`: one object re-acquired eight times
    is the scenario that motivated the whole component.
    """
    if not churn_counts:
        return {'objects': 0, 'mean': float('nan'), 'max': 0, 'objects_with_churn': 0}
    return {
        'objects': len(churn_counts),
        'mean': sum(churn_counts) / len(churn_counts),
        'max': max(churn_counts),
        'objects_with_churn': sum(1 for c in churn_counts if c > 1),
    }
