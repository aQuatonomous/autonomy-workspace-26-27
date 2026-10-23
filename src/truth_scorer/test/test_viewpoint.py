"""The correlation discount: does staring at a buoy count as confirming it?"""

import math

import pytest

from truth_scorer.core.config import ExistenceConfig, ViewpointConfig
from truth_scorer.core.existence import ExistenceFilter
from truth_scorer.core.viewpoint import Viewpoint, ViewpointHistory, wrap_angle


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi / 2) == pytest.approx(math.pi / 2)
    # Only two properties actually matter downstream, since every caller takes abs(): the result
    # stays inside one turn, and it is congruent to the input mod 2*pi. The sign at exactly +/-pi
    # is a floating-point coin toss and nothing should depend on it.
    for a in (0.0, 1.0, -1.0, 3 * math.pi, -3 * math.pi, 7.5, -12.25):
        w = wrap_angle(a)
        assert abs(w) <= math.pi + 1e-12
        assert math.sin(w) == pytest.approx(math.sin(a), abs=1e-9)
        assert math.cos(w) == pytest.approx(math.cos(a), abs=1e-9)


def test_first_look_gets_full_weight():
    h = ViewpointHistory()
    assert h.weight(Viewpoint(0.0, 10.0)) == pytest.approx(1.0)


def test_identical_repeat_look_gets_minimum_weight():
    cfg = ViewpointConfig(min_weight=0.05)
    h = ViewpointHistory(cfg)
    vp = Viewpoint(0.0, 10.0)
    h.weigh_and_record(vp)
    assert h.weight(vp) == pytest.approx(cfg.min_weight, abs=1e-9)


def test_weight_grows_with_viewpoint_change():
    h = ViewpointHistory(ViewpointConfig(bearing_scale_rad=math.radians(20.0)))
    h.weigh_and_record(Viewpoint(0.0, 10.0))
    small = h.weight(Viewpoint(math.radians(2.0), 10.0))
    large = h.weight(Viewpoint(math.radians(90.0), 10.0))
    assert small < large
    assert large > 0.9


def test_range_change_also_counts_as_novelty():
    h = ViewpointHistory(ViewpointConfig(range_scale_m=8.0))
    h.weigh_and_record(Viewpoint(0.0, 30.0))
    assert h.weight(Viewpoint(0.0, 5.0)) > h.weight(Viewpoint(0.0, 29.0))


def test_novelty_is_measured_against_nearest_not_latest():
    """A boat oscillating between two headings must not farm fresh evidence by alternating."""
    h = ViewpointHistory(ViewpointConfig(bearing_scale_rad=math.radians(20.0)))
    a = Viewpoint(0.0, 10.0)
    b = Viewpoint(math.radians(90.0), 10.0)
    h.weigh_and_record(a)
    h.weigh_and_record(b)
    # Returning to a previously used viewpoint is cheap even though it is far from the latest.
    assert h.weight(a) < 0.2


def test_history_is_bounded():
    cfg = ViewpointConfig(max_history=8)
    h = ViewpointHistory(cfg)
    for i in range(100):
        h.weigh_and_record(Viewpoint(math.radians(i), 10.0 + i))
    assert len(h) == 8


def test_twenty_static_looks_beat_by_five_varied_looks():
    """The headline property of the correlation discount.

    Twenty frames of staring from one spot is roughly one look repeated; five looks from genuinely
    different angles is five experiments. The second must move the estimate further.
    """
    cfg = ViewpointConfig(min_weight=0.05, bearing_scale_rad=math.radians(20.0), range_scale_m=8.0)
    ex_cfg = ExistenceConfig(logit_max=50.0)   # lift the clamp so the comparison is visible

    static = ExistenceFilter(ex_cfg)
    static_h = ViewpointHistory(cfg)
    for _ in range(20):
        static.update_hit(static_h.weigh_and_record(Viewpoint(0.0, 20.0)))

    varied = ExistenceFilter(ex_cfg)
    varied_h = ViewpointHistory(cfg)
    for i in range(5):
        vp = Viewpoint(math.radians(60.0 * i), 20.0 - 3.0 * i)
        varied.update_hit(varied_h.weigh_and_record(vp))

    assert varied.p_exist > static.p_exist
    assert varied.logit_value > static.logit_value


def test_static_looks_saturate_rather_than_accumulate():
    """Without the discount, 200 identical frames would reach absurd log-odds."""
    cfg = ViewpointConfig(min_weight=0.05)
    h = ViewpointHistory(cfg)
    vp = Viewpoint(0.0, 15.0)
    total = sum(h.weigh_and_record(vp) for _ in range(200))
    # 1.0 for the first look plus 199 * 0.05; two orders of magnitude below a naive count of 200.
    assert total < 15.0
