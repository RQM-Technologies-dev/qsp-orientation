"""Tests for qsp_orientation.utils."""
import pytest
import numpy as np
from qsp_orientation.utils import (
    normalize_vector, vector_norm, clamp,
    _quat_normalize, _quat_multiply, _quat_conjugate, _quat_rotate_vector,
)


def test_normalize_vector_unit():
    v = np.array([3.0, 0.0, 0.0])
    result = normalize_vector(v)
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0])


def test_normalize_vector_arbitrary():
    v = np.array([1.0, 2.0, 2.0])
    result = normalize_vector(v)
    np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=1e-10)


def test_normalize_vector_zero_raises():
    with pytest.raises(ValueError):
        normalize_vector([0.0, 0.0, 0.0])


def test_vector_norm_basic():
    assert vector_norm([3.0, 4.0, 0.0]) == pytest.approx(5.0)


def test_vector_norm_unit():
    assert vector_norm([1.0, 0.0, 0.0]) == pytest.approx(1.0)


def test_clamp_within():
    assert clamp(0.5, 0.0, 1.0) == pytest.approx(0.5)


def test_clamp_below():
    assert clamp(-0.5, 0.0, 1.0) == pytest.approx(0.0)


def test_clamp_above():
    assert clamp(1.5, 0.0, 1.0) == pytest.approx(1.0)


def test_quat_normalize_identity():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    result = _quat_normalize(q)
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0, 0.0])


def test_quat_normalize_zero_returns_identity():
    q = np.array([0.0, 0.0, 0.0, 0.0])
    result = _quat_normalize(q)
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0, 0.0])


def test_quat_multiply_identity():
    q_id = np.array([1.0, 0.0, 0.0, 0.0])
    q = np.array([0.5, 0.5, 0.5, 0.5])
    result = _quat_multiply(q_id, q)
    np.testing.assert_allclose(result, q)


def test_quat_multiply_hamilton():
    # i * j = k: [0,1,0,0] * [0,0,1,0] = [0,0,0,1]
    qi = np.array([0.0, 1.0, 0.0, 0.0])
    qj = np.array([0.0, 0.0, 1.0, 0.0])
    result = _quat_multiply(qi, qj)
    np.testing.assert_allclose(result, [0.0, 0.0, 0.0, 1.0], atol=1e-10)


def test_quat_conjugate():
    q = np.array([1.0, 2.0, 3.0, 4.0])
    result = _quat_conjugate(q)
    np.testing.assert_allclose(result, [1.0, -2.0, -3.0, -4.0])


def test_quat_rotate_vector_identity():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.array([1.0, 2.0, 3.0])
    result = _quat_rotate_vector(q, v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_quat_rotate_vector_90_yaw():
    # 90 deg yaw: rotate x-axis to y-axis
    import math
    angle = math.pi / 2
    q = np.array([math.cos(angle/2), 0.0, 0.0, math.sin(angle/2)])
    v = np.array([1.0, 0.0, 0.0])
    result = _quat_rotate_vector(q, v)
    np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-10)
