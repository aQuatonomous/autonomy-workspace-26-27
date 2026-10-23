"""ObjectMap: the behaviours that justify the whole component existing."""

import math
import random

import pytest

from truth_scorer.core.association import Measurement
from truth_scorer.core.config import ScorerConfig, VisibilityConfig
from truth_scorer.core.scorer import ObjectMap
from truth_scorer.core.visibility import BoatPose

GATE_BUOY_SPACING_M = 2.4384
RED_BUOY = 3
GREEN_BUOY = 1


def observe(east, north, pose, class_id=RED_BUOY, score=0.9, track_id=1, noise=0.0,
            rng=None, stamp=0.0):
    """Build a Measurement of a map-frame point as seen from `pose`."""
    if noise and rng:
        east = east + rng.gauss(0, noise)
        north = north + rng.gauss(0, noise)
    de, dn = east - pose.east, north - pose.north
    return Measurement(
        east=east, north=north,
        range_m=math.hypot(de, dn),
        global_bearing_rad=math.atan2(dn, de),
        class_id=class_id, class_score=score, track_id=track_id, stamp=stamp,
    )


def test_first_measurement_creates_one_object():
    m = ObjectMap()
    pose = BoatPose(0.0, 0.0, 0.0)
    m.update([observe(10.0, 0.0, pose)], pose, now=0.0)
    assert len(m) == 1
    assert m.objects[0].oid == 1


def test_repeated_observations_build_confidence_and_do_not_duplicate():
    m = ObjectMap()
    rng = random.Random(5)
    for i in range(40):
        pose = BoatPose(float(i) * 0.4, 0.0, 0.0)
        m.update([observe(20.0, 5.0, pose, noise=0.05, rng=rng, stamp=i * 0.4)],
                 pose, now=i * 0.4)
    assert len(m) == 1
    obj = m.objects[0]
    assert obj.p_exist > 0.9
    assert obj.observation_count == 40
    assert obj.position.east == pytest.approx(20.0, abs=0.5)
    assert obj.position.north == pytest.approx(5.0, abs=0.5)


def test_object_identity_survives_track_id_churn():
    """The headline fix.

    buoy_tracker deletes a track after 10 consecutive misses, so a re-acquired buoy arrives with
    a brand-new id. Anything keyed on track id forgets the object exactly when it comes back.
    """
    m = ObjectMap()
    for i in range(10):
        pose = BoatPose(float(i), 0.0, 0.0)
        m.update([observe(25.0, 0.0, pose, track_id=7, stamp=i)], pose, now=float(i))
    assert len(m) == 1
    oid = m.objects[0].oid

    # LiDAR drops the track, then re-acquires it with a different id.
    for i in range(10, 16):
        pose = BoatPose(float(i), 0.0, 0.0)
        m.update([observe(25.0, 0.0, pose, track_id=99, stamp=i)], pose, now=float(i))

    assert len(m) == 1, 'track id churn must not spawn a second object'
    assert m.objects[0].oid == oid, 'the scorer-owned id must be stable'
    assert m.objects[0].track_id == 99
    assert m.objects[0].track_id_churn_count() >= 2


def test_two_gate_buoys_never_merge_over_a_long_noisy_run():
    """200 noisy frames at the real 8 ft gate spacing, from a moving boat."""
    m = ObjectMap()
    rng = random.Random(4242)
    red = (20.0, 0.0)
    green = (20.0, GATE_BUOY_SPACING_M)

    for i in range(200):
        t = i * 0.4
        pose = BoatPose(east=i * 0.05, north=GATE_BUOY_SPACING_M / 2, yaw_rad=0.0, stamp=t)
        ms = [
            observe(*red, pose, class_id=RED_BUOY, track_id=1, noise=0.12, rng=rng, stamp=t),
            observe(*green, pose, class_id=GREEN_BUOY, track_id=2, noise=0.12, rng=rng, stamp=t),
        ]
        rng.shuffle(ms)
        m.update(ms, pose, now=t)

    confident = m.confident_objects(0.5)
    assert len(confident) == 2, f'expected 2 objects, got {len(confident)}'
    norths = sorted(o.position.north for o in confident)
    assert norths[1] - norths[0] == pytest.approx(GATE_BUOY_SPACING_M, abs=0.5)


def test_classes_do_not_flicker_between_adjacent_objects():
    """The last-write-wins behaviour in vision_lidar_fusion, measured end to end."""
    m = ObjectMap()
    rng = random.Random(11)
    for i in range(120):
        t = i * 0.4
        pose = BoatPose(east=i * 0.05, north=GATE_BUOY_SPACING_M / 2, yaw_rad=0.0, stamp=t)
        # One frame in five, CV mislabels the red buoy as green.
        red_class = GREEN_BUOY if i % 5 == 0 else RED_BUOY
        m.update([
            observe(20.0, 0.0, pose, class_id=red_class, track_id=1, noise=0.1, rng=rng, stamp=t),
            observe(20.0, GATE_BUOY_SPACING_M, pose, class_id=GREEN_BUOY, track_id=2,
                    noise=0.1, rng=rng, stamp=t),
        ], pose, now=t)

    objs = sorted(m.confident_objects(0.5), key=lambda o: o.position.north)
    assert len(objs) == 2
    assert objs[0].classifier.class_id == RED_BUOY, '80 % red must beat 20 % green'
    assert objs[1].classifier.class_id == GREEN_BUOY


def test_unobserved_object_is_not_punished_when_out_of_range():
    cfg = ScorerConfig()
    cfg.visibility = VisibilityConfig(max_range_m=30.0)
    m = ObjectMap(cfg)

    pose = BoatPose(0.0, 0.0, 0.0)
    for i in range(10):
        m.update([observe(20.0, 0.0, pose, stamp=i)], pose, now=float(i))
    established = m.objects[0].p_exist
    assert established > 0.8

    # Boat drives far away; the object is now well beyond sensor range.
    for i in range(10, 40):
        far_pose = BoatPose(east=-200.0, north=0.0, yaw_rad=0.0, stamp=float(i))
        m.update([], far_pose, now=float(i))

    obj = m.objects[0]
    assert obj.p_exist < established, 'belief should age'
    assert obj.p_exist > 0.3, 'but being out of range is not evidence of absence'
    assert obj.existence.expected_misses == 0


def test_phantom_object_is_disbelieved_then_pruned():
    """A clutter return seen once, then driven past and repeatedly not found."""
    m = ObjectMap()
    pose = BoatPose(0.0, 0.0, 0.0)
    m.update([observe(15.0, 0.0, pose)], pose, now=0.0)
    assert len(m) == 1

    for i in range(1, 40):
        moving = BoatPose(east=float(i) * 0.5, north=float(i) * 0.2, yaw_rad=0.0)
        m.update([], moving, now=float(i))

    assert len(m) == 0, 'sustained expected misses from fresh viewpoints should retire a phantom'
    assert m.stats['pruned'] >= 1


def test_stationary_observer_retires_a_phantom_slowly():
    """Documents a real and intentional consequence of the correlation discount.

    Misses are discounted by viewpoint novelty exactly as hits are, because the same correlation
    argument applies: staring at empty water from one spot is one experiment repeated, not sixty.
    So a phantom that is never approached decays slowly rather than being retired outright.

    That is the honest answer, but it is surprising enough to be worth pinning down: the belief
    must still fall below the 'probably there' line, it just takes longer than a naive counter
    would suggest.
    """
    m = ObjectMap()
    pose = BoatPose(0.0, 0.0, 0.0)
    m.update([observe(15.0, 0.0, pose)], pose, now=0.0)
    born = m.objects[0].p_exist

    for i in range(1, 60):
        m.update([], pose, now=float(i))

    assert len(m) == 1, 'a stationary observer has not earned the right to delete it'
    obj = m.objects[0]
    assert obj.p_exist < born
    assert obj.p_exist < 0.5, 'it must at least stop being believed'
    assert obj.existence.expected_misses == 59


def test_real_object_is_never_pruned_while_being_seen():
    m = ObjectMap()
    for i in range(200):
        pose = BoatPose(east=math.sin(i * 0.1) * 5.0, north=0.0, yaw_rad=0.0)
        m.update([observe(20.0, 0.0, pose, stamp=i * 0.4)], pose, now=i * 0.4)
    assert len(m) == 1
    assert m.stats['pruned'] == 0


def test_low_score_cv_leaves_the_class_uninformed():
    """camera0 end to end: plenty of detections, no class knowledge, and the scorer says so."""
    m = ObjectMap()
    for i in range(60):
        pose = BoatPose(east=i * 0.1, north=0.0, yaw_rad=0.0)
        m.update([observe(20.0, 0.0, pose, score=0.08, stamp=i * 0.4)], pose, now=i * 0.4)

    obj = m.objects[0]
    assert obj.p_exist > 0.9, 'LiDAR still proves something is there'
    assert obj.classifier.is_uninformed, 'but nothing proves what it is'
    assert obj.classifier.entropy() == pytest.approx(
        math.log(len(obj.classifier.probabilities())), abs=1e-9)


def test_existence_and_class_are_independent_estimates():
    """Confident existence with an unknown class must be representable; conflating the two is
    exactly the failure of a single scalar 'confidence'."""
    m = ObjectMap()
    for i in range(40):
        pose = BoatPose(east=i * 0.2, north=0.0, yaw_rad=0.0)
        m.update([observe(18.0, 3.0, pose, score=0.02, stamp=i * 0.4)], pose, now=i * 0.4)
    snap = m.objects[0].snapshot()
    assert snap['p_exist'] > 0.9
    assert snap['class_entropy'] > 2.0


def test_oids_are_unique_and_never_reused():
    m = ObjectMap()
    pose = BoatPose(0.0, 0.0, 0.0)
    seen = set()
    for i in range(30):
        # A new phantom every frame, 5 m apart, so each births a distinct object.
        m.update([observe(10.0 + i * 5.0, 0.0, pose, stamp=i)], pose, now=float(i))
        for o in m.objects:
            seen.add(o.oid)
    assert len(seen) == len({o for o in seen})
    assert m.stats['births'] == 30


def test_stats_are_internally_consistent():
    m = ObjectMap()
    rng = random.Random(3)
    for i in range(50):
        pose = BoatPose(east=i * 0.1, north=0.0, yaw_rad=0.0)
        m.update([observe(20.0, 0.0, pose, noise=0.05, rng=rng, stamp=i * 0.4)],
                 pose, now=i * 0.4)
    assert m.stats['frames'] == 50
    assert m.stats['measurements'] == 50
    assert m.stats['matches'] + m.stats['births'] + m.stats['deferred'] == 50


def test_snapshot_shape_matches_the_message():
    m = ObjectMap()
    pose = BoatPose(0.0, 0.0, 0.0)
    m.update([observe(10.0, 0.0, pose)], pose, now=0.0)
    snap = m.objects[0].snapshot()
    for key in ('oid', 'east', 'north', 'position_cov', 'p_exist', 'class_id', 'class_name',
                'class_prob', 'class_entropy', 'observation_count', 'track_id',
                'first_seen', 'last_seen'):
        assert key in snap, f'ScoredObject field {key} missing from snapshot'
    assert len(snap['position_cov']) == 4
    assert 0.0 <= snap['p_exist'] <= 1.0
    assert 0.0 <= snap['class_prob'] <= 1.0
