"""qsp-orientation: Quaternion-based attitude estimation, frame transforms, IMU fusion, and drift diagnostics."""

from qsp.orientation.utils import normalize_vector, vector_norm, clamp
from qsp.orientation.attitude import (
    euler_to_quaternion,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    slerp,
    relative_rotation,
)
from qsp.orientation.frames import (
    transform_vector,
    body_to_world,
    world_to_body,
    compose_rotations,
    invert_rotation,
)
from qsp.orientation.imu import (
    integrate_gyro,
    accel_tilt_estimate,
    gyro_bias_correction,
    orientation_from_imu,
)
from qsp.orientation.fusion import (
    complementary_filter,
    madgwick_update,
    mahony_update,
)
from qsp.orientation.diagnostics import (
    drift_angle,
    quaternion_distance,
    gyro_stability_metric,
    accel_consistency_metric,
    mag_consistency_metric,
    orientation_health_score,
)


def estimate_attitude(accel, gyro, dt, q0=None):
    """Public front-door API for IMU-based attitude estimation.

    Estimates orientation from a single IMU sample. If no initial quaternion
    is provided, the attitude is initialised from the accelerometer tilt.

    Args:
        accel: array-like [ax, ay, az] accelerometer reading (m/s²)
        gyro:  array-like [wx, wy, wz] gyroscope reading (rad/s)
        dt:    float time step in seconds
        q0:    array-like [w, x, y, z] initial quaternion, or None to
               initialise from accelerometer tilt

    Returns:
        np.ndarray: updated normalised quaternion [w, x, y, z]
    """
    return orientation_from_imu(accel, gyro, dt, q0=q0)

__all__ = [
    "normalize_vector", "vector_norm", "clamp",
    "euler_to_quaternion", "quaternion_to_euler",
    "quaternion_to_rotation_matrix", "rotation_matrix_to_quaternion",
    "slerp", "relative_rotation",
    "transform_vector", "body_to_world", "world_to_body",
    "compose_rotations", "invert_rotation",
    "estimate_attitude",
    "integrate_gyro", "accel_tilt_estimate", "gyro_bias_correction",
    "orientation_from_imu",
    "complementary_filter", "madgwick_update", "mahony_update",
    "drift_angle", "quaternion_distance", "gyro_stability_metric",
    "accel_consistency_metric", "mag_consistency_metric",
    "orientation_health_score",
]
