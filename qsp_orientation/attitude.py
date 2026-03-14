"""Attitude representation utilities: Euler/quaternion/matrix conversions and SLERP."""
import math
import numpy as np
from qsp_orientation.utils import clamp, _quat_normalize, _quat_multiply, _quat_conjugate


def euler_to_quaternion(roll, pitch, yaw):
    """Convert ZYX Euler angles to quaternion [w, x, y, z].
    
    ZYX convention (aerospace): R = Rz(yaw) * Ry(pitch) * Rx(roll)
    
    Args:
        roll: rotation about X axis (radians)
        pitch: rotation about Y axis (radians)
        yaw: rotation about Z axis (radians)
        
    Returns:
        np.ndarray: unit quaternion [w, x, y, z]
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w, x, y, z])


def quaternion_to_euler(q):
    """Extract ZYX Euler angles from quaternion [w, x, y, z].
    
    Args:
        q: array-like [w, x, y, z] unit quaternion
        
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    pitch = math.asin(clamp(2*(w*y - z*x), -1.0, 1.0))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    return (roll, pitch, yaw)


def quaternion_to_rotation_matrix(q):
    """Convert unit quaternion to 3x3 rotation matrix.
    
    Args:
        q: array-like [w, x, y, z] unit quaternion
        
    Returns:
        np.ndarray: shape (3, 3) rotation matrix
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),         1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),         2*(y*z + w*x),       1 - 2*(x**2 + y**2)],
    ])


def rotation_matrix_to_quaternion(R):
    """Convert rotation matrix to quaternion using Shepperd method.
    
    Args:
        R: np.ndarray shape (3, 3) rotation matrix
        
    Returns:
        np.ndarray: unit quaternion [w, x, y, z]
    """
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([w, x, y, z]))


def slerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions.
    
    Args:
        q1: array-like [w, x, y, z] start quaternion
        q2: array-like [w, x, y, z] end quaternion
        t: float in [0, 1], interpolation parameter
        
    Returns:
        np.ndarray: interpolated unit quaternion [w, x, y, z]
    """
    q1 = _quat_normalize(np.asarray(q1, dtype=float))
    q2 = _quat_normalize(np.asarray(q2, dtype=float))
    dot = float(np.dot(q1, q2))
    # Ensure shortest path
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-10:
        # Fall back to linear interpolation
        return _quat_normalize(q1 + t * (q2 - q1))
    return _quat_normalize(
        (math.sin((1.0 - t) * theta) / sin_theta) * q1 +
        (math.sin(t * theta) / sin_theta) * q2
    )


def relative_rotation(q1, q2):
    """Compute relative rotation from q1 to q2: q_rel = conj(q1) * q2.
    
    Args:
        q1: array-like [w, x, y, z] start orientation
        q2: array-like [w, x, y, z] end orientation
        
    Returns:
        np.ndarray: relative quaternion [w, x, y, z]
    """
    q1 = _quat_normalize(np.asarray(q1, dtype=float))
    q2 = _quat_normalize(np.asarray(q2, dtype=float))
    return _quat_normalize(_quat_multiply(_quat_conjugate(q1), q2))
