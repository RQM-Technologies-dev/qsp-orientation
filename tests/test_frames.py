"""Tests for qsp.orientation.frames."""
import pytest
import math
import numpy as np
from qsp.orientation.frames import (
    transform_vector,
    body_to_world,
    world_to_body,
    compose_rotations,
    invert_rotation,
)
from qsp.orientation.attitude import euler_to_quaternion
from qsp.orientation.diagnostics import quaternion_distance


def test_transform_vector_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.array([1.0, 2.0, 3.0])
    result = transform_vector(q, v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_transform_vector_90_yaw():
    # 90 deg yaw should rotate x-axis to y-axis
    q = euler_to_quaternion(0.0, 0.0, math.pi/2)
    v = np.array([1.0, 0.0, 0.0])
    result = transform_vector(q, v)
    np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-10)


def test_transform_vector_90_roll():
    # 90 deg roll: y-axis rotates to z-axis
    q = euler_to_quaternion(math.pi/2, 0.0, 0.0)
    v = np.array([0.0, 1.0, 0.0])
    result = transform_vector(q, v)
    np.testing.assert_allclose(result, [0.0, 0.0, 1.0], atol=1e-10)


def test_transform_vector_90_pitch():
    # 90 deg pitch: z-axis rotates to x-axis
    q = euler_to_quaternion(0.0, math.pi/2, 0.0)
    v = np.array([0.0, 0.0, 1.0])
    result = transform_vector(q, v)
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0], atol=1e-10)


def test_body_to_world_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.array([1.0, 2.0, 3.0])
    result = body_to_world(q, v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_world_to_body_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.array([1.0, 2.0, 3.0])
    result = world_to_body(q, v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_body_to_world_and_world_to_body_are_inverses():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    v = np.array([1.0, 2.0, 3.0])
    v_world = body_to_world(q, v)
    v_body_back = world_to_body(q, v_world)
    np.testing.assert_allclose(v_body_back, v, atol=1e-10)


def test_world_to_body_and_body_to_world_are_inverses():
    q = euler_to_quaternion(0.1, -0.3, 0.7)
    v = np.array([5.0, -2.0, 1.0])
    v_body = world_to_body(q, v)
    v_world_back = body_to_world(q, v_body)
    np.testing.assert_allclose(v_world_back, v, atol=1e-10)


def test_compose_rotations_identity():
    q_id = np.array([1.0, 0.0, 0.0, 0.0])
    q = euler_to_quaternion(0.3, 0.2, 0.1)
    result = compose_rotations(q_id, q)
    assert quaternion_distance(result, q) < 1e-10


def test_compose_rotations_two_yaws():
    # Two 45 deg yaw = 90 deg yaw
    q45 = euler_to_quaternion(0.0, 0.0, math.pi/4)
    q90 = euler_to_quaternion(0.0, 0.0, math.pi/2)
    result = compose_rotations(q45, q45)
    assert quaternion_distance(result, q90) < 1e-10


def test_invert_rotation_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    inv = invert_rotation(q)
    np.testing.assert_allclose(inv, q, atol=1e-10)


def test_invert_rotation_correctness():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    inv = invert_rotation(q)
    # inv should negate vector part
    np.testing.assert_allclose(inv[0], q[0], atol=1e-10)
    np.testing.assert_allclose(inv[1:], -q[1:], atol=1e-10)


def test_roundtrip_transform():
    q = euler_to_quaternion(0.5, -0.3, 1.0)
    v = np.array([2.0, -1.0, 0.5])
    v_rotated = transform_vector(q, v)
    q_inv = invert_rotation(q)
    v_back = transform_vector(q_inv, v_rotated)
    np.testing.assert_allclose(v_back, v, atol=1e-10)


def test_compose_rotations_order_matters():
    q_roll = euler_to_quaternion(math.pi/2, 0.0, 0.0)
    q_yaw = euler_to_quaternion(0.0, 0.0, math.pi/2)
    c1 = compose_rotations(q_roll, q_yaw)  # roll then yaw
    c2 = compose_rotations(q_yaw, q_roll)  # yaw then roll
    # They should be different (non-commutative)
    assert quaternion_distance(c1, c2) > 1e-5


def test_transform_vector_preserves_norm():
    q = euler_to_quaternion(0.3, 0.5, 0.7)
    v = np.array([1.0, 2.0, 3.0])
    result = transform_vector(q, v)
    np.testing.assert_allclose(np.linalg.norm(result), np.linalg.norm(v), atol=1e-10)


def test_compose_with_inverse_is_identity():
    q = euler_to_quaternion(0.3, 0.2, 0.5)
    q_inv = invert_rotation(q)
    result = compose_rotations(q, q_inv)
    assert quaternion_distance(result, np.array([1.0, 0.0, 0.0, 0.0])) < 1e-10
