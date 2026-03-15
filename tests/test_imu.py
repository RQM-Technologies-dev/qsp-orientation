"""Tests for qsp.orientation.imu."""
import pytest
import math
import numpy as np
from qsp.orientation.imu import (
    integrate_gyro,
    accel_tilt_estimate,
    gyro_bias_correction,
    orientation_from_imu,
)
from qsp.orientation.diagnostics import quaternion_distance


def test_integrate_gyro_zero_omega_unchanged():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    result = integrate_gyro(q, [0.0, 0.0, 0.0], 0.01)
    assert quaternion_distance(result, q) < 1e-10


def test_integrate_gyro_nonzero_omega_changes():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    result = integrate_gyro(q, [0.1, 0.0, 0.0], 0.1)
    assert quaternion_distance(result, q) > 1e-6


def test_integrate_gyro_output_normalized():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    result = integrate_gyro(q, [0.5, 0.3, 0.1], 0.01)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_integrate_gyro_normalized_input():
    q = np.array([0.5, 0.5, 0.5, 0.5])  # already unit
    result = integrate_gyro(q, [0.0, 0.0, 0.1], 0.01)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_integrate_gyro_small_dt():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    result = integrate_gyro(q, [1.0, 0.0, 0.0], 1e-6)
    # Very small dt: nearly unchanged
    assert quaternion_distance(result, q) < 1e-4


def test_accel_tilt_estimate_gravity_down():
    # Gravity pointing down along z: body is level
    accel = np.array([0.0, 0.0, 9.81])
    q = accel_tilt_estimate(accel)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    # Should be near identity (no tilt)
    assert quaternion_distance(q, np.array([1.0, 0.0, 0.0, 0.0])) < 1e-10


def test_accel_tilt_estimate_pitch_90():
    # Gravity along -x: 90 deg pitch
    accel = np.array([-9.81, 0.0, 0.0])
    q = accel_tilt_estimate(accel)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    r, p, y = (0.0, 0.0, 0.0)
    from qsp.orientation.attitude import quaternion_to_euler
    roll, pitch, yaw = quaternion_to_euler(q)
    assert abs(pitch - math.pi/2) < 1e-6


def test_accel_tilt_estimate_roll():
    # Gravity along y: 90 deg roll
    accel = np.array([0.0, 9.81, 0.0])
    q = accel_tilt_estimate(accel)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    from qsp.orientation.attitude import quaternion_to_euler
    roll, pitch, yaw = quaternion_to_euler(q)
    assert abs(roll - math.pi/2) < 1e-6


def test_gyro_bias_correction_zero_bias():
    omega = np.array([0.1, 0.2, 0.3])
    bias = np.array([0.0, 0.0, 0.0])
    result = gyro_bias_correction(omega, bias)
    np.testing.assert_allclose(result, omega)


def test_gyro_bias_correction_nonzero_bias():
    omega = np.array([0.5, 0.3, 0.1])
    bias = np.array([0.1, 0.1, 0.1])
    result = gyro_bias_correction(omega, bias)
    np.testing.assert_allclose(result, [0.4, 0.2, 0.0], atol=1e-10)


def test_gyro_bias_correction_subtracts():
    omega = np.array([1.0, 2.0, 3.0])
    bias = np.array([0.5, 1.0, 1.5])
    result = gyro_bias_correction(omega, bias)
    np.testing.assert_allclose(result, [0.5, 1.0, 1.5])


def test_orientation_from_imu_normalized():
    accel = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.1, 0.05, 0.0])
    result = orientation_from_imu(accel, gyro, 0.01)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_orientation_from_imu_no_q0_uses_accel():
    accel = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.0, 0.0, 0.0])
    result = orientation_from_imu(accel, gyro, 0.01)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_orientation_from_imu_with_q0():
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.0, 0.0, 0.0])
    result = orientation_from_imu(accel, gyro, 0.01, q0=q0)
    assert quaternion_distance(result, q0) < 1e-10


def test_orientation_from_imu_zero_gyro_stable():
    accel = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.0, 0.0, 0.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    result = orientation_from_imu(accel, gyro, 0.01, q0=q0)
    # With zero gyro, quaternion shouldn't change
    assert quaternion_distance(result, q0) < 1e-10
