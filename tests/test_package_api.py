"""Tests verifying all public functions are importable from qsp.orientation."""
import pytest
import numpy as np
import qsp.orientation as qsp_orientation


PUBLIC_FUNCTIONS = [
    "normalize_vector",
    "vector_norm",
    "clamp",
    "euler_to_quaternion",
    "quaternion_to_euler",
    "quaternion_to_rotation_matrix",
    "rotation_matrix_to_quaternion",
    "slerp",
    "relative_rotation",
    "transform_vector",
    "body_to_world",
    "world_to_body",
    "compose_rotations",
    "invert_rotation",
    "integrate_gyro",
    "accel_tilt_estimate",
    "gyro_bias_correction",
    "orientation_from_imu",
    "complementary_filter",
    "madgwick_update",
    "mahony_update",
    "drift_angle",
    "quaternion_distance",
    "gyro_stability_metric",
    "accel_consistency_metric",
    "mag_consistency_metric",
    "orientation_health_score",
]


@pytest.mark.parametrize("func_name", PUBLIC_FUNCTIONS)
def test_function_importable(func_name):
    assert hasattr(qsp_orientation, func_name), f"{func_name} not found in qsp_orientation"


@pytest.mark.parametrize("func_name", PUBLIC_FUNCTIONS)
def test_function_callable(func_name):
    func = getattr(qsp_orientation, func_name)
    assert callable(func), f"{func_name} is not callable"


def test_euler_to_quaternion_callable():
    q = qsp_orientation.euler_to_quaternion(0.0, 0.0, 0.0)
    np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-10)


def test_normalize_vector_callable():
    v = qsp_orientation.normalize_vector([3.0, 4.0, 0.0])
    np.testing.assert_allclose(np.linalg.norm(v), 1.0, atol=1e-10)


def test_drift_angle_callable():
    import math
    q1 = qsp_orientation.euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = qsp_orientation.euler_to_quaternion(0.0, 0.0, math.pi/2)
    angle = qsp_orientation.drift_angle(q1, q2)
    assert abs(angle - math.pi/2) < 1e-6


def test_module_importable():
    import qsp.orientation.utils
    import qsp.orientation.attitude
    import qsp.orientation.frames
    import qsp.orientation.imu
    import qsp.orientation.fusion
    import qsp.orientation.diagnostics


def test_all_contains_public_functions():
    for func_name in PUBLIC_FUNCTIONS:
        assert func_name in qsp_orientation.__all__, f"{func_name} not in __all__"
