"""Orientation drift diagnostics and health monitoring."""
import math
import numpy as np
from qsp_orientation.utils import clamp, vector_norm, normalize_vector, _quat_normalize, _quat_multiply, _quat_conjugate
from qsp_orientation.attitude import relative_rotation


def drift_angle(q1, q2, degrees=False):
    """Compute angular difference between two orientations.
    
    Args:
        q1: array-like [w, x, y, z] first quaternion
        q2: array-like [w, x, y, z] second quaternion
        degrees: bool if True return angle in degrees, else radians
        
    Returns:
        float: angular difference in radians (or degrees if requested)
    """
    q_rel = _quat_multiply(_quat_conjugate(_quat_normalize(np.asarray(q1, dtype=float))),
                           _quat_normalize(np.asarray(q2, dtype=float)))
    angle = 2.0 * math.acos(clamp(abs(q_rel[0]), 0.0, 1.0))
    if degrees:
        return math.degrees(angle)
    return angle


def quaternion_distance(q1, q2):
    """Scalar distance between two quaternions.
    
    Range [0, 1], 0 = identical orientations.
    
    Args:
        q1: array-like [w, x, y, z]
        q2: array-like [w, x, y, z]
        
    Returns:
        float: distance in [0, 1]
    """
    q1 = _quat_normalize(np.asarray(q1, dtype=float))
    q2 = _quat_normalize(np.asarray(q2, dtype=float))
    return float(1.0 - abs(np.dot(q1, q2)))


def gyro_stability_metric(samples):
    """Compute gyro stability metric as mean variance across axes.
    
    Higher values indicate noisier/less stable gyroscope.
    
    Args:
        samples: array-like shape (N, 3) gyro readings
        
    Returns:
        float: mean variance across all axes
    """
    samples = np.asarray(samples, dtype=float)
    return float(np.mean(np.var(samples, axis=0)))


def accel_consistency_metric(accel_vector, expected_gravity=9.81):
    """Check how close the accelerometer magnitude is to expected gravity.
    
    Args:
        accel_vector: array-like [ax, ay, az] accelerometer reading
        expected_gravity: float expected gravity magnitude (default 9.81)
        
    Returns:
        float: consistency score in [0, 1], 1 = perfect
    """
    magnitude = vector_norm(accel_vector)
    error = abs(magnitude - expected_gravity) / expected_gravity
    return float(max(0.0, 1.0 - error))


def mag_consistency_metric(mag_vector, reference_field):
    """Check how closely magnetometer aligns with reference field.
    
    Args:
        mag_vector: array-like [mx, my, mz] measured field
        reference_field: array-like [rx, ry, rz] reference field
        
    Returns:
        float: consistency score in [0, 1], 1 = perfect alignment
    """
    return float(abs(np.dot(normalize_vector(mag_vector), normalize_vector(reference_field))))


def orientation_health_score(drift_rad, gyro_stability, accel_consistency, mag_consistency=None):
    """Combine orientation metrics into a single health score.
    
    Args:
        drift_rad: float angular drift in radians
        gyro_stability: float gyro variance metric (0 = perfectly stable)
        accel_consistency: float accel consistency score [0, 1]
        mag_consistency: float mag consistency score [0, 1] or None
        
    Returns:
        float: health score in [0, 1], 1 = perfect health
    """
    drift_score = max(0.0, 1.0 - drift_rad / math.pi)
    gyro_score = max(0.0, 1.0 - min(gyro_stability, 1.0))
    if mag_consistency is None:
        score = (drift_score + gyro_score + accel_consistency) / 3.0
    else:
        score = (drift_score + gyro_score + accel_consistency + mag_consistency) / 4.0
    return float(clamp(score, 0.0, 1.0))
