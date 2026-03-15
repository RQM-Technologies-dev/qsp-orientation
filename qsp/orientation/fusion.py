"""Sensor fusion filter algorithms."""
import math
import numpy as np
from qsp.orientation.utils import normalize_vector, _quat_normalize, _quat_multiply
from qsp.orientation.attitude import slerp
from qsp.orientation.imu import accel_tilt_estimate, integrate_gyro


def complementary_filter(q_gyro, accel, alpha):
    """Complementary filter blending gyro and accelerometer estimates.
    
    Blends the gyro quaternion with accel tilt estimate using SLERP.
    alpha=1.0 trusts gyro fully; alpha=0.0 trusts accel fully.
    
    Args:
        q_gyro: array-like [w, x, y, z] gyro-integrated quaternion
        accel: array-like [ax, ay, az] accelerometer reading
        alpha: float in [0, 1], gyro trust factor
        
    Returns:
        np.ndarray: fused normalized quaternion [w, x, y, z]
    """
    q_accel = accel_tilt_estimate(accel)
    q_result = slerp(q_accel, q_gyro, alpha)
    return _quat_normalize(q_result)


def madgwick_update(q, gyro, accel, beta=0.1, dt=0.01, mag=None):
    """Simplified Madgwick filter update step.
    
    Gradient descent on objective function to correct gyro integration.
    
    Args:
        q: array-like [w, x, y, z] current quaternion
        gyro: array-like [gx, gy, gz] angular velocity rad/s
        accel: array-like [ax, ay, az] accelerometer reading
        beta: float filter gain (default 0.1)
        dt: float time step in seconds (default 0.01)
        mag: array-like [mx, my, mz] magnetometer (optional)
        
    Returns:
        np.ndarray: updated normalized quaternion [w, x, y, z]
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    q0, q1, q2, q3 = q
    accel = np.asarray(accel, dtype=float)
    accel_norm = np.linalg.norm(accel)
    if accel_norm < 1e-10:
        # Cannot use accel, just integrate gyro
        return integrate_gyro(q, gyro, dt)
    ax, ay, az = accel / accel_norm
    
    # Objective function F for gravity vector alignment
    F = np.array([
        2.0*(q1*q3 - q0*q2) - ax,
        2.0*(q0*q1 + q2*q3) - ay,
        2.0*(0.5 - q1**2 - q2**2) - az,
    ])
    
    # Jacobian J (3x4)
    J = np.array([
        [-2.0*q2,  2.0*q3, -2.0*q0,  2.0*q1],
        [ 2.0*q1,  2.0*q0,  2.0*q3,  2.0*q2],
        [ 0.0,    -4.0*q1, -4.0*q2,  0.0   ],
    ])
    
    gradient = J.T @ F
    grad_norm = np.linalg.norm(gradient)
    if grad_norm > 1e-10:
        gradient = gradient / grad_norm
    
    gx, gy, gz = np.asarray(gyro, dtype=float)
    q_dot = 0.5 * _quat_multiply(q, np.array([0.0, gx, gy, gz])) - beta * gradient
    q_new = q + q_dot * dt
    
    if mag is not None:
        # Simplified magnetometer correction: additional gradient step
        mag = np.asarray(mag, dtype=float)
        mag_norm = np.linalg.norm(mag)
        if mag_norm > 1e-10:
            mx, my, mz = mag / mag_norm
            # Estimated flux in earth frame
            h = _quat_multiply(q_new, _quat_multiply(
                np.array([0.0, mx, my, mz]),
                np.array([q_new[0], -q_new[1], -q_new[2], -q_new[3]])
            ))
            bx = math.sqrt(h[1]**2 + h[2]**2)
            bz = h[3]
            q0n, q1n, q2n, q3n = q_new
            Fm = np.array([
                2.0*bx*(0.5 - q2n**2 - q3n**2) + 2.0*bz*(q1n*q3n - q0n*q2n) - mx,
                2.0*bx*(q1n*q2n - q0n*q3n) + 2.0*bz*(q0n*q1n + q2n*q3n) - my,
                2.0*bx*(q0n*q2n + q1n*q3n) + 2.0*bz*(0.5 - q1n**2 - q2n**2) - mz,
            ])
            Jm = np.array([
                [-2.0*bz*q2n,            2.0*bz*q3n,           -4.0*bx*q2n-2.0*bz*q0n,  -4.0*bx*q3n+2.0*bz*q1n],
                [-2.0*bx*q3n+2.0*bz*q1n, 2.0*bx*q2n+2.0*bz*q0n, 2.0*bx*q1n+2.0*bz*q3n,  -2.0*bx*q0n+2.0*bz*q2n],
                [ 2.0*bx*q2n,             2.0*bx*q3n-4.0*bz*q1n,  2.0*bx*q0n-4.0*bz*q2n,  2.0*bx*q1n              ],
            ])
            grad_m = Jm.T @ Fm
            gm_norm = np.linalg.norm(grad_m)
            if gm_norm > 1e-10:
                grad_m = grad_m / gm_norm
            q_new = q_new - beta * grad_m * dt
    
    return _quat_normalize(q_new)


def mahony_update(q, gyro, accel, Kp=2.0, Ki=0.005, dt=0.01, integral_error=None, mag=None):
    """Mahony complementary filter update step.
    
    Args:
        q: array-like [w, x, y, z] current quaternion
        gyro: array-like [gx, gy, gz] angular velocity rad/s
        accel: array-like [ax, ay, az] accelerometer reading
        Kp: float proportional gain (default 2.0)
        Ki: float integral gain (default 0.005)
        dt: float time step in seconds (default 0.01)
        integral_error: array-like [ix, iy, iz] accumulated integral (or None)
        mag: array-like [mx, my, mz] magnetometer (optional)
        
    Returns:
        tuple: (q_new np.ndarray [w,x,y,z], integral_error np.ndarray [ix,iy,iz])
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    if integral_error is None:
        integral_error = np.zeros(3)
    else:
        integral_error = np.asarray(integral_error, dtype=float)
    
    accel = np.asarray(accel, dtype=float)
    accel_norm = np.linalg.norm(accel)
    if accel_norm < 1e-10:
        return q, integral_error
    accel_norm_vec = accel / accel_norm
    
    # Estimated gravity direction from current quaternion (body frame)
    q0, q1, q2, q3 = q
    v = np.array([
        2.0*(q1*q3 - q0*q2),
        2.0*(q0*q1 + q2*q3),
        q0**2 - q1**2 - q2**2 + q3**2,
    ])
    
    # Error is cross product of estimated and measured gravity
    e = np.cross(accel_norm_vec, v)
    
    if mag is not None:
        mag = np.asarray(mag, dtype=float)
        mag_norm = np.linalg.norm(mag)
        if mag_norm > 1e-10:
            mag_norm_vec = mag / mag_norm
            # Estimated magnetic north direction from quaternion
            w_mag = np.array([
                2.0*(q1*q2 + q0*q3),
                q0**2 - q1**2 + q2**2 - q3**2,
                2.0*(q2*q3 - q0*q1),
            ])
            e += np.cross(mag_norm_vec, w_mag)
    
    integral_error = integral_error + Ki * e * dt
    omega_corrected = np.asarray(gyro, dtype=float) + Kp * e + integral_error
    q_new = integrate_gyro(q, omega_corrected, dt)
    return q_new, integral_error
