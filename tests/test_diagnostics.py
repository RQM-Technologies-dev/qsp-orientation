"""Tests for qsp_orientation.diagnostics."""
import pytest
import math
import numpy as np
from qsp_orientation.diagnostics import (
    drift_angle,
    quaternion_distance,
    gyro_stability_metric,
    accel_consistency_metric,
    mag_consistency_metric,
    orientation_health_score,
)
from qsp_orientation.attitude import euler_to_quaternion


def test_drift_angle_same_quaternion_zero():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    assert drift_angle(q, q) < 1e-10


def test_drift_angle_identity_and_180_roll():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(math.pi, 0.0, 0.0)
    angle = drift_angle(q1, q2)
    assert abs(angle - math.pi) < 1e-6


def test_drift_angle_degrees():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(math.pi, 0.0, 0.0)
    angle_deg = drift_angle(q1, q2, degrees=True)
    assert abs(angle_deg - 180.0) < 1e-4


def test_drift_angle_90_degrees():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    angle = drift_angle(q1, q2)
    assert abs(angle - math.pi/2) < 1e-6


def test_drift_angle_is_nonnegative():
    q1 = euler_to_quaternion(0.3, 0.0, 0.0)
    q2 = euler_to_quaternion(-0.3, 0.0, 0.0)
    assert drift_angle(q1, q2) >= 0.0


def test_quaternion_distance_same_zero():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    assert quaternion_distance(q, q) < 1e-10


def test_quaternion_distance_antipodal_zero():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    assert quaternion_distance(q, -q) < 1e-10


def test_quaternion_distance_range():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.3, 0.2, 0.5)
    d = quaternion_distance(q1, q2)
    assert 0.0 <= d <= 1.0


def test_quaternion_distance_orthogonal():
    q1 = np.array([1.0, 0.0, 0.0, 0.0])
    q2 = np.array([0.0, 1.0, 0.0, 0.0])
    d = quaternion_distance(q1, q2)
    assert abs(d - 1.0) < 1e-10


def test_gyro_stability_constant_samples_zero():
    samples = np.ones((10, 3)) * 0.5
    metric = gyro_stability_metric(samples)
    assert metric < 1e-10


def test_gyro_stability_noisy_samples_positive():
    rng = np.random.default_rng(42)
    samples = rng.standard_normal((100, 3))
    metric = gyro_stability_metric(samples)
    assert metric > 0


def test_accel_consistency_perfect():
    # Magnitude exactly 9.81
    accel = np.array([0.0, 0.0, 9.81])
    score = accel_consistency_metric(accel)
    assert abs(score - 1.0) < 1e-10


def test_accel_consistency_wrong_magnitude():
    accel = np.array([0.0, 0.0, 5.0])  # half gravity
    score = accel_consistency_metric(accel)
    assert score < 1.0


def test_accel_consistency_range():
    accel = np.array([1.0, 1.0, 1.0])
    score = accel_consistency_metric(accel)
    assert 0.0 <= score <= 1.0


def test_mag_consistency_same_direction():
    v = np.array([1.0, 0.0, 0.0])
    score = mag_consistency_metric(v, v)
    assert abs(score - 1.0) < 1e-10


def test_mag_consistency_orthogonal():
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    score = mag_consistency_metric(v1, v2)
    assert abs(score) < 1e-10


def test_mag_consistency_range():
    v1 = np.array([1.0, 2.0, 3.0])
    v2 = np.array([3.0, 1.0, 2.0])
    score = mag_consistency_metric(v1, v2)
    assert 0.0 <= score <= 1.0


def test_orientation_health_score_range():
    score = orientation_health_score(0.1, 0.0, 1.0)
    assert 0.0 <= score <= 1.0


def test_orientation_health_score_perfect():
    score = orientation_health_score(0.0, 0.0, 1.0)
    assert abs(score - 1.0) < 1e-10


def test_orientation_health_score_with_mag():
    score = orientation_health_score(0.0, 0.0, 1.0, mag_consistency=1.0)
    assert abs(score - 1.0) < 1e-10


def test_orientation_health_score_bad_drift():
    score = orientation_health_score(math.pi, 0.0, 1.0)
    assert score < 1.0
