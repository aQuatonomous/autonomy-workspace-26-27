"""Visibility: separating 'looked and saw nothing' from 'never looked'."""

import math

import pytest

from truth_scorer.core.config import VisibilityConfig
from truth_scorer.core.visibility import (
    OCCLUDED,
    OUT_OF_FOV,
    OUT_OF_RANGE_FAR,
    OUT_OF_RANGE_NEAR,
    VISIBLE,
    BoatPose,
    Occluder,
    check_visibility,
)


def test_relative_geometry():
    pose = BoatPose(east=10.0, north=10.0, yaw_rad=0.0)
    rng, global_bearing, rel_bearing = pose.relative(20.0, 10.0)
    assert rng == pytest.approx(10.0)
    assert global_bearing == pytest.approx(0.0)
    assert rel_bearing == pytest.approx(0.0)


def test_relative_bearing_accounts_for_heading():
    pose = BoatPose(east=0.0, north=0.0, yaw_rad=math.pi / 2)   # pointing north
    _, global_bearing, rel_bearing = pose.relative(10.0, 0.0)   # object due east
    assert global_bearing == pytest.approx(0.0)
    assert rel_bearing == pytest.approx(-math.pi / 2)


def test_nearby_object_is_visible():
    pose = BoatPose(0.0, 0.0, 0.0)
    v = check_visibility(pose, 15.0, 0.0)
    assert v.visible
    assert v.reason == VISIBLE
    assert v.range_m == pytest.approx(15.0)


def test_distant_object_is_not_visible():
    cfg = VisibilityConfig(max_range_m=35.0)
    v = check_visibility(BoatPose(0.0, 0.0, 0.0), 100.0, 0.0, cfg)
    assert not v.visible
    assert v.reason == OUT_OF_RANGE_FAR


def test_object_in_the_blind_spot_is_not_visible():
    cfg = VisibilityConfig(min_range_m=0.8)
    v = check_visibility(BoatPose(0.0, 0.0, 0.0), 0.2, 0.0, cfg)
    assert not v.visible
    assert v.reason == OUT_OF_RANGE_NEAR


def test_lidar_sees_behind_by_default():
    """The boat's LiDAR is a 360-degree scanner; astern is not a blind spot for it."""
    v = check_visibility(BoatPose(0.0, 0.0, 0.0), -15.0, 0.0)
    assert v.visible


def test_limited_fov_sensor_excludes_astern():
    cfg = VisibilityConfig(fov_rad=math.radians(120.0))
    pose = BoatPose(0.0, 0.0, 0.0)   # facing east
    assert check_visibility(pose, 15.0, 0.0, cfg).visible
    behind = check_visibility(pose, -15.0, 0.0, cfg)
    assert not behind.visible
    assert behind.reason == OUT_OF_FOV


def test_occluder_casts_a_shadow():
    pose = BoatPose(0.0, 0.0, 0.0)
    occ = Occluder(east=5.0, north=0.0, radius_m=0.5)
    v = check_visibility(pose, 20.0, 0.0, occluders=[occ])
    assert not v.visible
    assert v.reason == OCCLUDED


def test_occluder_behind_the_target_casts_no_shadow():
    pose = BoatPose(0.0, 0.0, 0.0)
    occ = Occluder(east=25.0, north=0.0, radius_m=0.5)
    assert check_visibility(pose, 20.0, 0.0, occluders=[occ]).visible


def test_occluder_off_to_the_side_casts_no_shadow():
    pose = BoatPose(0.0, 0.0, 0.0)
    occ = Occluder(east=5.0, north=4.0, radius_m=0.3)
    assert check_visibility(pose, 20.0, 0.0, occluders=[occ]).visible


def test_occlusion_shadow_narrows_with_distance():
    """A 0.3 m buoy at 2 m hides a lot; the same buoy at 30 m hides almost nothing."""
    pose = BoatPose(0.0, 0.0, 0.0)
    target_offset = 0.4   # metres off the line of sight, at 40 m range

    near = Occluder(east=2.0, north=0.0, radius_m=0.3)
    far = Occluder(east=30.0, north=0.0, radius_m=0.3)

    cfg = VisibilityConfig(max_range_m=60.0)
    assert not check_visibility(pose, 40.0, target_offset, cfg, [near]).visible
    assert check_visibility(pose, 40.0, target_offset, cfg, [far]).visible
