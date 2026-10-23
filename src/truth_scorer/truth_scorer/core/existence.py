"""P(an object physically exists here), as a bounded log-odds filter.

Three distinct things can happen to an object on a given frame, and conflating them is the usual
reason existence estimates are garbage:

  hit        we looked and saw it            -> evidence for
  miss       we looked and did not see it    -> evidence against
  invisible  we did not look                 -> no evidence either way

Only a miss that the visibility model calls *expected* -- in range, in field of view, not occluded
-- counts against an object. A buoy behind us is not evidence of anything. When an object is
unobservable the log-odds relaxes toward the birth prior, which encodes "the longer it has been
since I could check, the less I should trust my stale opinion" without ever flipping that opinion.
"""

from __future__ import annotations

import math
from typing import Optional

from .config import ExistenceConfig


def logit(p: float) -> float:
    p = min(max(p, 1e-9), 1.0 - 1e-9)
    return math.log(p / (1.0 - p))


def sigmoid(x: float) -> float:
    if x >= 0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    z = math.exp(x)
    return z / (1.0 + z)


class ExistenceFilter:
    """Bounded log-odds accumulator for a single object."""

    def __init__(self, config: Optional[ExistenceConfig] = None) -> None:
        self.config = config or ExistenceConfig()
        self._l0 = logit(self.config.p0)
        self._l = self._l0
        self.hits = 0
        self.expected_misses = 0
        self.consecutive_expected_misses = 0

    # -- state ---------------------------------------------------------------------------------

    @property
    def logit_value(self) -> float:
        return self._l

    @property
    def p_exist(self) -> float:
        return sigmoid(self._l)

    def _clamp(self) -> None:
        self._l = min(max(self._l, self.config.logit_min), self.config.logit_max)

    # -- updates -------------------------------------------------------------------------------

    @property
    def _hit_evidence(self) -> float:
        """Log-likelihood ratio of a detection: log P(detect | exists) / P(detect | empty)."""
        return math.log(self.config.p_detect / self.config.clutter_rate)

    def update_hit(self, weight: float = 1.0) -> None:
        """Register a detection carrying `weight` independent-observations worth of evidence."""
        weight = max(0.0, float(weight))
        if weight == 0.0:
            return
        self._l += weight * self._hit_evidence
        self._clamp()
        self.hits += 1
        self.consecutive_expected_misses = 0

    def update_expected_miss(self, weight: float = 1.0) -> None:
        """Register a look that should have seen the object and did not."""
        weight = max(0.0, float(weight))
        if weight == 0.0:
            return
        self._l -= weight * self.config.miss_penalty
        self._clamp()
        self.expected_misses += 1
        self.consecutive_expected_misses += 1

    def update_invisible(self, dt: float) -> None:
        """Register elapsed time during which the object could not have been observed.

        Relaxes exponentially toward the birth prior. This can never push the estimate past the
        prior, so "I haven't looked in a while" degrades confidence without inventing the opposite
        belief.
        """
        if dt <= 0.0:
            return
        tau = self.config.decay_tau_s
        if tau <= 0.0:
            self._l = self._l0
            return
        alpha = 1.0 - math.exp(-dt / tau)
        self._l += (self._l0 - self._l) * alpha
        self._clamp()

    # -- lifecycle -----------------------------------------------------------------------------

    def should_prune(self) -> bool:
        """True once the object has been actively disbelieved, not merely forgotten."""
        cfg = self.config
        return (self.p_exist < cfg.prune_below_p
                and self.consecutive_expected_misses >= cfg.prune_after_expected_misses)
