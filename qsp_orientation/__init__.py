"""qsp-orientation: Quaternion-based attitude estimation, frame transforms, IMU fusion, and drift diagnostics."""

from qsp_orientation.utils import normalize_vector, vector_norm, clamp
from qsp_orientation.attitude import (
    euler_to_quaternion,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    slerp,
    relative_rotation,
)
from qsp_orientation.frames import (
    transform_vector,
    body_to_world,
    world_to_body,
    compose_rotations,
    invert_rotation,
)
from qsp_orientation.imu import (
    integrate_gyro,
    accel_tilt_estimate,
    gyro_bias_correction,
    orientation_from_imu,
)
from qsp_orientation.fusion import (
    complementary_filter,
    madgwick_update,
    mahony_update,
)
from qsp_orientation.diagnostics import (
    drift_angle,
    quaternion_distance,
    gyro_stability_metric,
    accel_consistency_metric,
    mag_consistency_metric,
    orientation_health_score,
)

__all__ = [
    "normalize_vector", "vector_norm", "clamp",
    "euler_to_quaternion", "quaternion_to_euler",
    "quaternion_to_rotation_matrix", "rotation_matrix_to_quaternion",
    "slerp", "relative_rotation",
    "transform_vector", "body_to_world", "world_to_body",
    "compose_rotations", "invert_rotation",
    "integrate_gyro", "accel_tilt_estimate", "gyro_bias_correction",
    "orientation_from_imu",
    "complementary_filter", "madgwick_update", "mahony_update",
    "drift_angle", "quaternion_distance", "gyro_stability_metric",
    "accel_consistency_metric", "mag_consistency_metric",
    "orientation_health_score",
]
