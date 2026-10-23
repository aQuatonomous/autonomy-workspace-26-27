"""Position filter: is the point estimate good, and -- far more importantly -- is R honest?"""

import math
import random

import numpy as np
import pytest

from truth_scorer.core.config import PositionConfig
from truth_scorer.core.position import (
    PositionFilter,
    SOURCE_CAMERA,
    SOURCE_LIDAR,
    apply_covariance_floor,
    polar_measurement_covariance,
)


def test_polar_covariance_is_anisotropic_along_line_of_sight():
    """At 30 m a LiDAR knows range to centimetres and cross-range to decimetres."""
    R = polar_measurement_covariance(30.0, 0.0, sigma_range_m=0.08,
                                     sigma_bearing_rad=math.radians(0.6))
    # Bearing zero = looking due east, so along-range is the east axis.
    assert R[0, 0] == pytest.approx(0.08 ** 2, abs=1e-12)
    assert R[1, 1] == pytest.approx((30.0 * math.radians(0.6)) ** 2, rel=1e-9)
    assert R[1, 1] > R[0, 0], 'cross-range must be the loose axis for a LiDAR at range'


def test_polar_covariance_rotates_with_bearing():
    R = polar_measurement_covariance(30.0, math.pi / 2, 0.08, math.radians(0.6))
    # Looking due north: now the north axis is the tight one.
    assert R[1, 1] == pytest.approx(0.08 ** 2, abs=1e-12)
    assert R[0, 0] > R[1, 1]


def test_polar_covariance_eigenvalues_are_rotation_invariant():
    a = polar_measurement_covariance(20.0, 0.0, 0.1, 0.01)
    b = polar_measurement_covariance(20.0, 1.234, 0.1, 0.01)
    assert np.allclose(sorted(np.linalg.eigvalsh(a)), sorted(np.linalg.eigvalsh(b)))


def test_camera_range_is_worse_than_lidar_range():
    """The camera's pinhole distance is the opposite error shape: loose along range."""
    cfg = PositionConfig()
    lidar = PositionFilter(0.0, 0.0, cfg)
    camera = PositionFilter(0.0, 0.0, cfg)
    lidar.update_polar(0.0, 0.0, 25.0, 0.0, source=SOURCE_LIDAR)
    camera.update_polar(0.0, 0.0, 25.0, 0.0, source=SOURCE_CAMERA)
    # Along-range (east, at bearing 0) uncertainty must be far larger for the camera.
    assert camera.covariance[0, 0] > lidar.covariance[0, 0]


def test_converges_to_truth():
    rng = random.Random(1234)
    truth = (12.0, -7.0)
    f = PositionFilter(truth[0] + 2.0, truth[1] - 2.0, PositionConfig(covariance_floor_m=0.0))
    for _ in range(100):
        R = np.diag([0.25, 0.25])
        f.update(truth[0] + rng.gauss(0, 0.5), truth[1] + rng.gauss(0, 0.5), R)
    assert math.hypot(f.east - truth[0], f.north - truth[1]) < 0.25


def test_covariance_shrinks_with_observations():
    f = PositionFilter(0.0, 0.0, PositionConfig(covariance_floor_m=0.0))
    start = np.trace(f.covariance)
    for _ in range(20):
        f.update(0.0, 0.0, np.diag([0.25, 0.25]))
    assert np.trace(f.covariance) < start


def test_covariance_floor_is_respected():
    """GPS bias is correlated; averaging must not be allowed to claim it away."""
    floor = 0.25
    f = PositionFilter(0.0, 0.0, PositionConfig(covariance_floor_m=floor))
    for _ in range(500):
        f.update(0.0, 0.0, np.diag([0.01, 0.01]))
    eigenvalues = np.linalg.eigvalsh(f.covariance)
    assert eigenvalues.min() >= floor ** 2 - 1e-9
    major, minor = f.sigma_major_minor()
    assert minor >= floor - 1e-6


def test_apply_covariance_floor_preserves_orientation():
    cov = np.array([[4.0, 1.5], [1.5, 1.0]])
    floored = apply_covariance_floor(cov, 0.5)
    # Eigenvectors unchanged, eigenvalues raised only where needed.
    _, v_orig = np.linalg.eigh(cov)
    _, v_new = np.linalg.eigh(floored)
    assert np.allclose(np.abs(v_orig), np.abs(v_new), atol=1e-9)
    assert np.linalg.eigvalsh(floored).min() >= 0.25 - 1e-12


def test_covariance_stays_symmetric_and_positive_definite():
    rng = random.Random(7)
    f = PositionFilter(0.0, 0.0)
    for _ in range(200):
        bearing = rng.uniform(-math.pi, math.pi)
        f.update_polar(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(2, 30), bearing)
        P = f.covariance
        assert np.allclose(P, P.T, atol=1e-12)
        assert np.linalg.eigvalsh(P).min() > 0.0


def test_predict_inflates_uncertainty():
    f = PositionFilter(0.0, 0.0, PositionConfig(process_noise_m2_per_s=1e-3))
    before = np.trace(f.covariance)
    f.predict(10.0)
    assert np.trace(f.covariance) > before


def test_update_polar_places_the_point_correctly():
    f = PositionFilter(0.0, 0.0, PositionConfig(birth_sigma_m=100.0, covariance_floor_m=0.0))
    # Sensor at (10, 10), object 5 m due north of it.
    f.update_polar(10.0, 10.0, 5.0, math.pi / 2)
    assert f.east == pytest.approx(10.0, abs=0.2)
    assert f.north == pytest.approx(15.0, abs=0.2)


def test_mean_nees_is_consistent():
    """The test that proves R is honest.

    For a correctly specified 2-D filter, NEES averages the state dimension, 2.0. Over-confidence
    (R too small, or the covariance floor set below the real noise) pushes it up; timidity pushes
    it down. The [1.6, 2.5] band is the plan's acceptance criterion.

    No covariance floor and no GPS term here: this isolates the filter's own arithmetic.
    """
    rng = random.Random(20260101)
    cfg = PositionConfig(covariance_floor_m=0.0, process_noise_m2_per_s=0.0, birth_sigma_m=5.0)
    sigma_r, sigma_b = 0.08, math.radians(0.6)

    nees_samples = []
    for trial in range(300):
        true_e = rng.uniform(-30, 30)
        true_n = rng.uniform(-30, 30)
        f = PositionFilter(true_e + rng.gauss(0, 5.0), true_n + rng.gauss(0, 5.0), cfg)

        for _ in range(25):
            # Observer somewhere else each time, so the geometry varies like a moving boat.
            sx, sy = rng.uniform(-40, 40), rng.uniform(-40, 40)
            de, dn = true_e - sx, true_n - sy
            rng_true = math.hypot(de, dn)
            if rng_true < 2.0:
                continue
            bearing_true = math.atan2(dn, de)

            # Sample the measurement in polar space, exactly as the sensor model describes it.
            r_meas = rng_true + rng.gauss(0, sigma_r)
            b_meas = bearing_true + rng.gauss(0, sigma_b)
            z_e = sx + r_meas * math.cos(b_meas)
            z_n = sy + r_meas * math.sin(b_meas)
            R = polar_measurement_covariance(r_meas, b_meas, sigma_r, sigma_b)
            f.update(z_e, z_n, R)

        nees_samples.append(f.nees(true_e, true_n))

    mean_nees = sum(nees_samples) / len(nees_samples)
    assert 1.6 <= mean_nees <= 2.5, f'filter is inconsistent: mean NEES = {mean_nees:.3f}'


def test_covariance_floor_makes_the_filter_conservative_not_overconfident():
    """With the floor on, NEES should fall below 2 -- erring toward humility, never hubris."""
    rng = random.Random(99)
    cfg = PositionConfig(covariance_floor_m=0.25, process_noise_m2_per_s=0.0)
    sigma_r, sigma_b = 0.08, math.radians(0.6)

    samples = []
    for _ in range(200):
        true_e, true_n = rng.uniform(-20, 20), rng.uniform(-20, 20)
        f = PositionFilter(true_e, true_n, cfg)
        for _ in range(40):
            sx, sy = rng.uniform(-40, 40), rng.uniform(-40, 40)
            de, dn = true_e - sx, true_n - sy
            r = math.hypot(de, dn)
            if r < 2.0:
                continue
            b = math.atan2(dn, de)
            r_m, b_m = r + rng.gauss(0, sigma_r), b + rng.gauss(0, sigma_b)
            f.update(sx + r_m * math.cos(b_m), sy + r_m * math.sin(b_m),
                     polar_measurement_covariance(r_m, b_m, sigma_r, sigma_b))
        samples.append(f.nees(true_e, true_n))

    mean_nees = sum(samples) / len(samples)
    assert mean_nees < 2.0, 'the floor should buy conservatism'
    assert mean_nees > 0.0


def test_nis_is_returned_and_finite():
    f = PositionFilter(0.0, 0.0)
    nis = f.update(0.5, 0.5, np.diag([0.1, 0.1]))
    assert math.isfinite(nis)
    assert nis >= 0.0


def test_covariance_flat_is_row_major():
    f = PositionFilter(0.0, 0.0)
    f.P = np.array([[1.0, 2.0], [3.0, 4.0]])
    assert f.covariance_flat() == [1.0, 2.0, 3.0, 4.0]


def test_sensor_covariance_is_added():
    """Forgetting the observer's own GPS uncertainty is the classic over-confidence bug."""
    without = PositionFilter(0.0, 0.0, PositionConfig(covariance_floor_m=0.0))
    with_gps = PositionFilter(0.0, 0.0, PositionConfig(covariance_floor_m=0.0))
    for _ in range(10):
        without.update_polar(0.0, 0.0, 20.0, 0.3)
        with_gps.update_polar(0.0, 0.0, 20.0, 0.3, sensor_covariance=np.diag([0.25, 0.25]))
    assert np.trace(with_gps.covariance) > np.trace(without.covariance)
