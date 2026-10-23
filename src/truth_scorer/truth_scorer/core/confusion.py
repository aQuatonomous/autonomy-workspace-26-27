"""The confusion matrix: P(CV says j | the object really is i).

This is the piece that turns a raw YOLO softmax -- which is notoriously over-confident and means
almost nothing in absolute terms -- into an actual likelihood. The default below is a hand-built
conservative prior whose only job is to be approximately right in *shape*: within a colour group
confusion is common, across colour groups it is rare, and anything can read as unknown.

The real matrix is fitted from a run's JSONL by analyse_run.py. Until that happens, the numbers
here should be read as "deliberately pessimistic", not as measurements.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

from .classes import CLASS_IDS, CONFUSABLE_GROUPS, NUM_CLASSES, UNKNOWN_ID, index_of


def _group_of(class_id: int) -> List[int]:
    for group in CONFUSABLE_GROUPS:
        if class_id in group:
            return group
    return [class_id]


def default_confusion_matrix(
    p_correct: float = 0.60,
    p_within_group: float = 0.22,
    p_unknown: float = 0.10,
    p_unknown_self: float = 0.50,
) -> List[List[float]]:
    """Row i = distribution over what the CV model reports when truth is class i.

    Remaining mass after correct / within-group / unknown is spread uniformly across every other
    class, so no observation has zero likelihood under any hypothesis. A zero there would be fatal:
    one bad frame could drive a correct hypothesis to exactly zero posterior, permanently.
    """
    matrix = [[0.0] * NUM_CLASSES for _ in range(NUM_CLASSES)]
    for true_id in CLASS_IDS:
        i = index_of(true_id)
        row = matrix[i]

        if true_id == UNKNOWN_ID:
            # Truth is "none of the catalogue". The detector most often declines to classify it,
            # and otherwise guesses more or less uniformly.
            #
            # This row must stay diagonally dominant. A flat row here is actively wrong: with
            # P(report unknown | truly unknown) = 1/17 but P(report unknown | truly a red buoy)
            # = 0.10, an "unknown" reading would become evidence *against* the object being
            # unknown, and the posterior would drive the one honest hypothesis to zero.
            spread = (1.0 - p_unknown_self) / (NUM_CLASSES - 1)
            for cid in CLASS_IDS:
                row[index_of(cid)] = spread
            row[index_of(UNKNOWN_ID)] = p_unknown_self
            _normalise_in_place(row)
            continue

        siblings = [c for c in _group_of(true_id) if c != true_id]
        others = [c for c in CLASS_IDS
                  if c != true_id and c != UNKNOWN_ID and c not in siblings]

        row[i] = p_correct
        row[index_of(UNKNOWN_ID)] = p_unknown
        if siblings:
            share = p_within_group / len(siblings)
            for c in siblings:
                row[index_of(c)] = share
            leftover = 1.0 - p_correct - p_within_group - p_unknown
        else:
            leftover = 1.0 - p_correct - p_unknown
        if others:
            share = max(leftover, 0.0) / len(others)
            for c in others:
                row[index_of(c)] = share
        _normalise_in_place(row)
    return matrix


def _normalise_in_place(row: List[float], floor: float = 1e-6) -> None:
    for k in range(len(row)):
        if row[k] < floor:
            row[k] = floor
    total = sum(row)
    for k in range(len(row)):
        row[k] /= total


def log_likelihood_column(matrix: Sequence[Sequence[float]], observed_class_id: int) -> List[float]:
    """log P(observed | true = k) for every k, i.e. a column of the matrix, logged."""
    j = index_of(observed_class_id)
    return [math.log(max(matrix[i][j], 1e-12)) for i in range(NUM_CLASSES)]


def matrix_from_counts(
    counts: Dict[int, Dict[int, float]],
    prior_strength: float = 2.0,
    prior: Optional[Sequence[Sequence[float]]] = None,
) -> List[List[float]]:
    """Fit a confusion matrix from observed (true_id -> {observed_id: count}) tallies.

    Smoothed toward `prior` (the hand-built default unless given) with weight `prior_strength`
    pseudo-counts per row, so classes the boat barely saw do not acquire wild likelihoods from a
    handful of frames.
    """
    base = prior if prior is not None else default_confusion_matrix()
    matrix = [[0.0] * NUM_CLASSES for _ in range(NUM_CLASSES)]
    for true_id in CLASS_IDS:
        i = index_of(true_id)
        observed = counts.get(true_id, {})
        row = [base[i][j] * prior_strength for j in range(NUM_CLASSES)]
        for obs_id, n in observed.items():
            row[index_of(obs_id)] += float(n)
        _normalise_in_place(row)
        matrix[i] = row
    return matrix
