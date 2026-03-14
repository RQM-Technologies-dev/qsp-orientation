"""Tests for qsp_orientation.fusion."""
import pytest
import math
import numpy as np
from qsp_orientation.fusion import (
    complementary_filter,
    madgwick_update,
    mahony_update,
)
from qsp_orientation.imu import accel_tilt_estimate
from qsp_orientation.diagnostics import quaternion_distance


GRAVITY = np.array([0.0, 0.0, 9.81])
Q_ID = np.array([1.0, 0.0, 0.0, 0.0])


def test_complementary_filter_alpha1_returns_gyro():
    q_gyro = np.array([0.9239, 0.3827, 0.0, 0.0])  # ~45 deg roll
    q_gyro = q_gyro / np.linalg.norm(q_gyro)
    accel = GRAVITY
    result = complementary_filter(q_gyro, accel, alpha=1.0)
    assert quaternion_distance(result, q_gyro) < 1e-6


def test_complementary_filter_alpha0_returns_accel():
    q_gyro = np.array([0.9239, 0.3827, 0.0, 0.0])
    q_gyro = q_gyro / np.linalg.norm(q_gyro)
    accel = GRAVITY
    q_accel = accel_tilt_estimate(accel)
    result = complementary_filter(q_gyro, accel, alpha=0.0)
    assert quaternion_distance(result, q_accel) < 1e-6


def test_complementary_filter_output_normalized():
    q_gyro = Q_ID.copy()
    result = complementary_filter(q_gyro, GRAVITY, alpha=0.5)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_complementary_filter_midpoint():
    q_gyro = Q_ID.copy()
    result = complementary_filter(q_gyro, GRAVITY, alpha=0.5)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_madgwick_update_normalized():
    q = Q_ID.copy()
    result = madgwick_update(q, [0.1, 0.0, 0.0], GRAVITY)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_madgwick_update_changes_quaternion():
    q = Q_ID.copy()
    result = madgwick_update(q, [1.0, 0.0, 0.0], GRAVITY, dt=0.1)
    assert quaternion_distance(result, q) > 1e-6


def test_madgwick_update_zero_accel_graceful():
    q = Q_ID.copy()
    # Should not raise, should fall back to gyro integration
    result = madgwick_update(q, [0.1, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_madgwick_update_with_mag():
    q = Q_ID.copy()
    mag = np.array([0.3, 0.0, 0.5])
    result = madgwick_update(q, [0.1, 0.0, 0.0], GRAVITY, mag=mag)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_madgwick_update_returns_ndarray():
    q = Q_ID.copy()
    result = madgwick_update(q, [0.0, 0.0, 0.0], GRAVITY)
    assert isinstance(result, np.ndarray)
    assert result.shape == (4,)


def test_mahony_update_returns_tuple():
    q = Q_ID.copy()
    result = mahony_update(q, [0.1, 0.0, 0.0], GRAVITY)
    assert isinstance(result, tuple)
    assert len(result) == 2


def test_mahony_update_quaternion_normalized():
    q = Q_ID.copy()
    q_new, _ = mahony_update(q, [0.1, 0.0, 0.0], GRAVITY)
    assert abs(np.linalg.norm(q_new) - 1.0) < 1e-10


def test_mahony_update_with_zero_integral():
    q = Q_ID.copy()
    q_new, integral = mahony_update(q, [0.0, 0.0, 0.0], GRAVITY, integral_error=np.zeros(3))
    assert abs(np.linalg.norm(q_new) - 1.0) < 1e-10


def test_mahony_update_with_mag():
    q = Q_ID.copy()
    mag = np.array([0.3, 0.0, 0.5])
    q_new, integral = mahony_update(q, [0.0, 0.0, 0.0], GRAVITY, mag=mag)
    assert abs(np.linalg.norm(q_new) - 1.0) < 1e-10


def test_mahony_update_converges():
    # Apply many iterations with tilted accel to see convergence toward tilt
    q = Q_ID.copy()
    # Gravity appears along y (body rolled 90 deg)
    accel = np.array([0.0, 9.81, 0.0])
    integral = np.zeros(3)
    for _ in range(500):
        q, integral = mahony_update(q, [0.0, 0.0, 0.0], accel, dt=0.01, integral_error=integral)
    from qsp_orientation.attitude import quaternion_to_euler
    roll, pitch, yaw = quaternion_to_euler(q)
    assert abs(roll - math.pi/2) < 0.1


def test_mahony_integral_accumulates():
    q = Q_ID.copy()
    accel = np.array([0.0, 9.81, 0.0])
    _, integral = mahony_update(q, [0.0, 0.0, 0.0], accel, Ki=0.1, dt=0.01, integral_error=np.zeros(3))
    # Integral should be non-zero after one step with tilted accel
    assert np.linalg.norm(integral) > 0
