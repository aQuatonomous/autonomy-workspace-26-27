"""Categorical class posterior for one object.

Design decisions that matter more than the maths:

* Evidence is a *confusion matrix column*, not the raw softmax. The softmax says what the network
  felt; the confusion matrix says what that feeling is worth.
* A low YOLO score contributes exactly zero, not a little. camera0 currently tops out around 0.10
  raw score; letting hundreds of such frames accumulate would manufacture a confident answer out
  of pure noise. Below `score_floor` the update is a bitwise no-op.
* Stale CV is likewise a no-op rather than an "unknown" observation. Not having looked recently is
  not evidence that the object is unrecognisable.
* Confidence is bounded by clamping the spread of the log-posterior. Without this, 200 correlated
  frames drive the posterior to 1 - 1e-40 and the reported probability stops meaning anything.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .classes import CLASS_IDS, NUM_CLASSES, UNKNOWN_ID, class_colour, class_name, index_of
from .config import ClassifierConfig
from .confusion import default_confusion_matrix, log_likelihood_column


class ClassPosterior:
    """Bounded Bayesian categorical estimate over the class vocabulary."""

    def __init__(
        self,
        config: Optional[ClassifierConfig] = None,
        confusion_matrix: Optional[Sequence[Sequence[float]]] = None,
    ) -> None:
        self.config = config or ClassifierConfig()
        self.matrix = confusion_matrix if confusion_matrix is not None else default_confusion_matrix()
        # Uniform prior in log space.
        self._logp: List[float] = [-math.log(NUM_CLASSES)] * NUM_CLASSES
        self.evidence_weight = 0.0   # total discounted evidence absorbed (an effective sample size)
        self.updates = 0

    # -- evidence weighting --------------------------------------------------------------------

    def score_weight(self, score: float) -> float:
        """Map a raw detector score to evidence weight in [0, 1]."""
        cfg = self.config
        if score <= cfg.score_floor:
            return 0.0
        span = max(cfg.score_reference - cfg.score_floor, 1e-9)
        return min(1.0, (score - cfg.score_floor) / span)

    def is_stale(self, cv_age_s: Optional[float]) -> bool:
        return cv_age_s is not None and cv_age_s > self.config.cv_stale_s

    # -- update --------------------------------------------------------------------------------

    def update(
        self,
        observed_class_id: int,
        score: float,
        viewpoint_weight: float = 1.0,
        cv_age_s: Optional[float] = None,
    ) -> float:
        """Fold in one CV observation. Returns the evidence weight actually applied (0 = no-op)."""
        if self.is_stale(cv_age_s):
            return 0.0
        w = self.score_weight(float(score)) * max(0.0, float(viewpoint_weight))
        if w <= 0.0:
            return 0.0

        loglik = log_likelihood_column(self.matrix, observed_class_id)
        for i in range(NUM_CLASSES):
            self._logp[i] += w * loglik[i]
        self._normalise()
        self._bound_confidence()
        self.evidence_weight += w
        self.updates += 1
        return w

    def _normalise(self) -> None:
        m = max(self._logp)
        total = sum(math.exp(v - m) for v in self._logp)
        offset = m + math.log(total)
        for i in range(NUM_CLASSES):
            self._logp[i] -= offset

    def _bound_confidence(self) -> None:
        """Keep the posterior from collapsing onto one class.

        Clamps every log-probability to within `max_logprob_spread` of the best one. This bounds
        the reported confidence while leaving the ordering, and the ability of future evidence to
        change that ordering, intact.
        """
        limit = self.config.max_logprob_spread
        if limit <= 0:
            return
        best = max(self._logp)
        floor = best - limit
        changed = False
        for i in range(NUM_CLASSES):
            if self._logp[i] < floor:
                self._logp[i] = floor
                changed = True
        if changed:
            self._normalise()

    # -- readout -------------------------------------------------------------------------------

    def probabilities(self) -> List[float]:
        return [math.exp(v) for v in self._logp]

    def log_probabilities(self) -> List[float]:
        return list(self._logp)

    def probability_of(self, class_id: int) -> float:
        return math.exp(self._logp[index_of(class_id)])

    def log_probability_of(self, class_id: int) -> float:
        return self._logp[index_of(class_id)]

    def argmax(self) -> Tuple[int, float]:
        """(class_id, probability) of the most likely class."""
        best_i = max(range(NUM_CLASSES), key=lambda i: self._logp[i])
        return CLASS_IDS[best_i], math.exp(self._logp[best_i])

    @property
    def class_id(self) -> int:
        return self.argmax()[0]

    @property
    def class_name(self) -> str:
        return class_name(self.class_id)

    @property
    def class_prob(self) -> float:
        return self.argmax()[1]

    def entropy(self) -> float:
        """Shannon entropy in nats. The honest 'I don't know' signal.

        Maximum is log(NUM_CLASSES); a posterior pinned on one class approaches zero.
        """
        total = 0.0
        for lp in self._logp:
            p = math.exp(lp)
            if p > 0.0:
                total -= p * lp
        return total

    def colour_probabilities(self) -> dict:
        """Marginal over colour, which is what the handbook actually scores."""
        out: dict = {}
        for i, cid in enumerate(CLASS_IDS):
            colour = class_colour(cid)
            key = colour if colour is not None else 'none'
            out[key] = out.get(key, 0.0) + math.exp(self._logp[i])
        return out

    def argmax_colour(self) -> Tuple[str, float]:
        probs = self.colour_probabilities()
        best = max(probs.items(), key=lambda kv: kv[1])
        return best

    @property
    def is_uninformed(self) -> bool:
        """True when nothing has moved the posterior off its uniform prior."""
        return self.evidence_weight <= 0.0

    def snapshot(self) -> dict:
        cid, p = self.argmax()
        return {
            'class_id': cid,
            'class_name': class_name(cid),
            'class_prob': p,
            'entropy': self.entropy(),
            'evidence_weight': self.evidence_weight,
            'updates': self.updates,
            'unknown_prob': self.probability_of(UNKNOWN_ID),
        }
