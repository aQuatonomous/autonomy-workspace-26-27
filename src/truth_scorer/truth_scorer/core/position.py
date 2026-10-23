"""2-D Kalman filter for the map-frame position of a static object.

The point of this module is not to produce a better point estimate -- averaging would do nearly as
well -- it is to produce an *honest covariance*. Two things make that non-trivial:

1. Sensor noise is polar, not Cartesian. A LiDAR return at 30 m is known to a few centimetres in
   range and a couple of decimetres across it; a camera distance estimate is the opposite extreme,
   good in bearing and poor in range. Modelling R as isotropic throws that structure away and
   produces error ellipses that are wrong in both directions at once. R is therefore built in
   (along-range, cross-range) axes and rotated into ENU.

2. GPS error is correlated in time. Averaging 500 observations taken over 20 seconds does not
   reduce a slowly-wandering position bias, but the filter's arithmetic will happily claim it did
   and report millimetre certainty. The covariance floor forbids that claim.

The NEES test in the suite is what keeps this module honest: if R is wrong, NEES leaves [1.6, 2.5]
and the test fails.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .config import PositionConfig

# Observation sources, which have very different error structure.
SOURCE_LIDAR = 'lidar'
SOURCE_CAMERA = 'camera'


def polar_measurement_covariance(
    range_m: float,
    bearing_rad: float,
    sigma_range_m: float,
    sigma_bearing_rad: float,
) -> np.ndarray:
    """2x2 ENU covariance for a polar measurement.

    `bearing_rad` is the *global* bearing from sensor to object in ENU (east->north, i.e. the
    angle of the line of sight as atan2(dnorth, deast)). The along-range axis is the line of
    sight; the cross-range axis is perpendicular to it, with standard deviation range * sigma_b.
    """
    sigma_cross = max(range_m, 1e-3) * sigma_bearing_rad
    cov_polar = np.diag([sigma_range_m ** 2, sigma_cross ** 2])
    c, s = math.cos(bearing_rad), math.sin(bearing_rad)
    # Columns: along-range unit vector, then cross-range unit vector, both in ENU.
    rot = np.array([[c, -s],
                    [s, c]])
    return rot @ cov_polar @ rot.T


def apply_covariance_floor(cov: np.ndarray, floor_m: float) -> np.ndarray:
    """Raise every eigenvalue of `cov` to at least floor_m**2, preserving orientation."""
    if floor_m <= 0.0:
        return cov
    cov = 0.5 * (cov + cov.T)   # enforce symmetry before eigendecomposition
    vals, vecs = np.linalg.eigh(cov)
    vals = np.maximum(vals, floor_m ** 2)
    return vecs @ np.diag(vals) @ vecs.T


class PositionFilter:
    """Static-object position KF in the map (ENU) frame."""

    def __init__(
        self,
        east: float,
        north: float,
        config: Optional[PositionConfig] = None,
        initial_covariance: Optional[Sequence[Sequence[float]]] = None,
    ) -> None:
        self.config = config or PositionConfig()
        self.x = np.array([float(east), float(north)], dtype=float)
        if initial_covariance is not None:
            self.P = np.array(initial_covariance, dtype=float).reshape(2, 2)
        else:
            s = self.config.birth_sigma_m ** 2
            self.P = np.diag([s, s]).astype(float)
        self.update_count = 0

    # -- accessors -----------------------------------------------------------------------------

    @property
    def east(self) -> float:
        return float(self.x[0])

    @property
    def north(self) -> float:
        return float(self.x[1])

    @property
    def covariance(self) -> np.ndarray:
        return self.P.copy()

    def covariance_flat(self) -> list:
        """Row-major 2x2, matching ScoredObject.position_cov."""
        return [float(v) for v in self.P.reshape(-1)]

    def sigma_major_minor(self) -> Tuple[float, float]:
        """(major, minor) 1-sigma axis lengths of the error ellipse, in metres."""
        vals = np.linalg.eigvalsh(0.5 * (self.P + self.P.T))
        vals = np.maximum(vals, 0.0)
        return float(math.sqrt(vals[1])), float(math.sqrt(vals[0]))

    # -- filter --------------------------------------------------------------------------------

    def predict(self, dt: float) -> None:
        """Static motion model: the state does not move, but certainty erodes slightly.

        A moored buoy does drift on its chain and the map origin itself wanders with GPS, so a
        filter with literally zero process noise would eventually refuse new information.
        """
        if dt <= 0.0:
            return
        q = self.config.process_noise_m2_per_s * dt
        self.P = self.P + np.diag([q, q])

    def update(
        self,
        east: float,
        north: float,
        measurement_covariance: Sequence[Sequence[float]],
    ) -> float:
        """Fold in a position measurement. Returns the measurement's NIS (normalised innovation
        squared), which is the live, truth-free analogue of NEES and worth logging."""
        z = np.array([float(east), float(north)], dtype=float)
        R = np.array(measurement_covariance, dtype=float).reshape(2, 2)

        y = z - self.x                      # innovation (H is identity)
        S = self.P + R                      # innovation covariance
        S = 0.5 * (S + S.T)
        S_inv = np.linalg.inv(S)
        K = self.P @ S_inv

        self.x = self.x + K @ y
        I = np.eye(2)
        # Joseph form: numerically stable and stays positive-definite under a floored P.
        A = I - K
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = apply_covariance_floor(self.P, self.config.covariance_floor_m)
        self.update_count += 1
        return float(y @ S_inv @ y)

    def update_polar(
        self,
        sensor_east: float,
        sensor_north: float,
        range_m: float,
        global_bearing_rad: float,
        source: str = SOURCE_LIDAR,
        sensor_covariance: Optional[Sequence[Sequence[float]]] = None,
    ) -> float:
        """Convenience wrapper: build R from the sensor model, then update.

        `sensor_covariance` is the uncertainty of the *observer's* own position (GPS), which adds
        directly to the measurement uncertainty of the object. Omitting it is the classic way to
        end up with an over-confident map.
        """
        cfg = self.config
        if source == SOURCE_CAMERA:
            sigma_r = max(range_m * cfg.sigma_range_camera_frac, 0.05)
            sigma_b = cfg.sigma_bearing_camera_rad
        else:
            sigma_r = cfg.sigma_range_lidar_m
            sigma_b = cfg.sigma_bearing_lidar_rad

        R = polar_measurement_covariance(range_m, global_bearing_rad, sigma_r, sigma_b)
        if sensor_covariance is not None:
            R = R + np.array(sensor_covariance, dtype=float).reshape(2, 2)

        east = sensor_east + range_m * math.cos(global_bearing_rad)
        north = sensor_north + range_m * math.sin(global_bearing_rad)
        return self.update(east, north, R)

    # -- diagnostics ---------------------------------------------------------------------------

    def nees(self, true_east: float, true_north: float) -> float:
        """Normalised estimation error squared against a known truth.

        For a consistent 2-D filter this averages 2.0 over many runs. Much above means the filter
        is over-confident (the dangerous direction); much below means it is needlessly timid.
        """
        err = self.x - np.array([float(true_east), float(true_north)])
        return float(err @ np.linalg.inv(self.P) @ err)

    def mahalanobis_squared(self, east: float, north: float,
                            extra_covariance: Optional[Sequence[Sequence[float]]] = None) -> float:
        """Squared Mahalanobis distance from this object to a point, used for gating."""
        d = np.array([float(east), float(north)]) - self.x
        S = self.P if extra_covariance is None else self.P + np.array(
            extra_covariance, dtype=float).reshape(2, 2)
        return float(d @ np.linalg.inv(0.5 * (S + S.T)) @ d)
