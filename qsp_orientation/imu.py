"""IMU integration and tilt estimation utilities."""
import math
import numpy as np
from qsp_orientation.utils import normalize_vector, _quat_normalize, _quat_multiply
from qsp_orientation.attitude import euler_to_quaternion


def integrate_gyro(q, omega, dt):
    """Integrate gyroscope measurements to update orientation.
    
    First-order quaternion integration.
    
    Args:
        q: array-like [w, x, y, z] current orientation
        omega: array-like [wx, wy, wz] angular velocity in rad/s
        dt: float time step in seconds
        
    Returns:
        np.ndarray: updated normalized quaternion [w, x, y, z]
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    omega = np.asarray(omega, dtype=float)
    wx, wy, wz = omega
    omega_quat = np.array([0.0, wx, wy, wz])
    q_dot = 0.5 * dt * _quat_multiply(q, omega_quat)
    q_new = q + q_dot
    return _quat_normalize(q_new)


def accel_tilt_estimate(accel):
    """Estimate roll and pitch from accelerometer reading.
    
    Assumes gravity dominates the accelerometer measurement.
    
    Args:
        accel: array-like [ax, ay, az] accelerometer reading
        
    Returns:
        np.ndarray: quaternion [w, x, y, z] with estimated roll and pitch (yaw=0)
    """
    accel_norm = normalize_vector(np.asarray(accel, dtype=float))
    roll = math.atan2(accel_norm[1], accel_norm[2])
    pitch = math.atan2(-accel_norm[0], math.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
    return euler_to_quaternion(roll, pitch, 0.0)


def gyro_bias_correction(omega, bias):
    """Subtract gyroscope bias from angular velocity measurement.
    
    Args:
        omega: array-like [wx, wy, wz] raw gyroscope reading
        bias: array-like [bx, by, bz] gyroscope bias estimate
        
    Returns:
        np.ndarray: bias-corrected angular velocity [wx, wy, wz]
    """
    return np.array(omega, dtype=float) - np.array(bias, dtype=float)


def orientation_from_imu(accel, gyro, dt, q0=None):
    """Estimate orientation from IMU measurements.
    
    If no initial quaternion provided, initialize from accelerometer tilt.
    
    Args:
        accel: array-like [ax, ay, az] accelerometer reading
        gyro: array-like [wx, wy, wz] gyroscope reading in rad/s
        dt: float time step in seconds
        q0: array-like [w, x, y, z] initial quaternion (or None)
        
    Returns:
        np.ndarray: updated normalized quaternion [w, x, y, z]
    """
    if q0 is None:
        q0 = accel_tilt_estimate(accel)
    q0 = _quat_normalize(np.asarray(q0, dtype=float))
    return integrate_gyro(q0, gyro, dt)
