"""Tests for qsp.orientation.attitude."""
import pytest
import math
import numpy as np
from qsp.orientation.attitude import (
    euler_to_quaternion,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    slerp,
    relative_rotation,
)
from qsp.orientation.diagnostics import quaternion_distance


def test_euler_to_quaternion_zero_is_identity():
    q = euler_to_quaternion(0.0, 0.0, 0.0)
    np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-10)


def test_euler_to_quaternion_roll_only():
    q = euler_to_quaternion(math.pi/4, 0.0, 0.0)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    # w = cos(pi/8), x = sin(pi/8), y=0, z=0
    np.testing.assert_allclose(q, [math.cos(math.pi/8), math.sin(math.pi/8), 0.0, 0.0], atol=1e-10)


def test_euler_to_quaternion_pitch_only():
    q = euler_to_quaternion(0.0, math.pi/6, 0.0)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    np.testing.assert_allclose(q, [math.cos(math.pi/12), 0.0, math.sin(math.pi/12), 0.0], atol=1e-10)


def test_euler_to_quaternion_yaw_only():
    q = euler_to_quaternion(0.0, 0.0, math.pi/3)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-10
    np.testing.assert_allclose(q, [math.cos(math.pi/6), 0.0, 0.0, math.sin(math.pi/6)], atol=1e-10)


def test_euler_quaternion_roundtrip_zero():
    q = euler_to_quaternion(0.0, 0.0, 0.0)
    r, p, y = quaternion_to_euler(q)
    assert abs(r) < 1e-10
    assert abs(p) < 1e-10
    assert abs(y) < 1e-10


def test_euler_quaternion_roundtrip_general():
    roll, pitch, yaw = 0.3, 0.2, 0.5
    q = euler_to_quaternion(roll, pitch, yaw)
    r, p, y = quaternion_to_euler(q)
    assert abs(r - roll) < 1e-10
    assert abs(p - pitch) < 1e-10
    assert abs(y - yaw) < 1e-10


def test_euler_quaternion_roundtrip_negative():
    roll, pitch, yaw = -0.4, -0.1, -0.7
    q = euler_to_quaternion(roll, pitch, yaw)
    r, p, y = quaternion_to_euler(q)
    assert abs(r - roll) < 1e-10
    assert abs(p - pitch) < 1e-10
    assert abs(y - yaw) < 1e-10


def test_quaternion_to_euler_clamp_prevents_domain_error():
    # Gimbal lock: pitch = pi/2
    q = euler_to_quaternion(0.0, math.pi/2, 0.0)
    r, p, y = quaternion_to_euler(q)
    assert abs(p - math.pi/2) < 1e-6


def test_quaternion_to_rotation_matrix_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    R = quaternion_to_rotation_matrix(q)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-10)


def test_quaternion_to_rotation_matrix_orthogonal():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    R = quaternion_to_rotation_matrix(q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)


def test_quaternion_to_rotation_matrix_det_one():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    R = quaternion_to_rotation_matrix(q)
    assert abs(np.linalg.det(R) - 1.0) < 1e-10


def test_rotation_matrix_to_quaternion_identity():
    R = np.eye(3)
    q = rotation_matrix_to_quaternion(R)
    assert quaternion_distance(q, np.array([1.0, 0.0, 0.0, 0.0])) < 1e-10


def test_rotation_matrix_to_quaternion_roundtrip():
    q_orig = euler_to_quaternion(0.3, 0.2, 0.5)
    R = quaternion_to_rotation_matrix(q_orig)
    q_back = rotation_matrix_to_quaternion(R)
    assert quaternion_distance(q_orig, q_back) < 1e-10


def test_rotation_matrix_to_quaternion_180_roll():
    q_orig = euler_to_quaternion(math.pi, 0.0, 0.0)
    R = quaternion_to_rotation_matrix(q_orig)
    q_back = rotation_matrix_to_quaternion(R)
    assert quaternion_distance(q_orig, q_back) < 1e-10


def test_slerp_t0_returns_q1():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    result = slerp(q1, q2, 0.0)
    assert quaternion_distance(result, q1) < 1e-10


def test_slerp_t1_returns_q2():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    result = slerp(q1, q2, 1.0)
    assert quaternion_distance(result, q2) < 1e-10


def test_slerp_t05_is_normalized():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    result = slerp(q1, q2, 0.5)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_slerp_antipodal():
    q1 = np.array([1.0, 0.0, 0.0, 0.0])
    q2 = np.array([-1.0, 0.0, 0.0, 0.0])  # same orientation, opposite sign
    result = slerp(q1, q2, 0.5)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_relative_rotation_self_is_identity():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    q_rel = relative_rotation(q, q)
    assert quaternion_distance(q_rel, np.array([1.0, 0.0, 0.0, 0.0])) < 1e-10


def test_relative_rotation_correctness():
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    q_rel = relative_rotation(q1, q2)
    assert quaternion_distance(q_rel, q2) < 1e-10


def test_euler_to_quaternion_is_normalized():
    for angles in [(0.1, 0.2, 0.3), (1.0, -0.5, 0.7), (-1.0, 1.0, -1.0)]:
        q = euler_to_quaternion(*angles)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-10
