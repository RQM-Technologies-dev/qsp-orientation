"""Utility functions: vector helpers and internal quaternion math primitives."""
import math
import numpy as np


def normalize_vector(v):
    """Normalize a vector to unit length.
    
    Args:
        v: array-like vector
        
    Returns:
        np.ndarray: unit vector
        
    Raises:
        ValueError: if vector has zero norm
    """
    v = np.asarray(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return v / norm


def vector_norm(v):
    """Compute the L2 norm of a vector.
    
    Args:
        v: array-like vector
        
    Returns:
        float: L2 norm
    """
    return float(np.linalg.norm(np.asarray(v, dtype=float)))


def clamp(val, min_val, max_val):
    """Clamp value between min and max.
    
    Args:
        val: value to clamp
        min_val: minimum bound
        max_val: maximum bound
        
    Returns:
        float: clamped value
    """
    return float(max(min_val, min(max_val, val)))


def _quat_normalize(q):
    """Normalize a quaternion [w, x, y, z].
    
    Args:
        q: array-like [w, x, y, z]
        
    Returns:
        np.ndarray: normalized quaternion
    """
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def _quat_multiply(q1, q2):
    """Hamilton product of two quaternions [w, x, y, z].
    
    Args:
        q1: array-like [w, x, y, z]
        q2: array-like [w, x, y, z]
        
    Returns:
        np.ndarray: product quaternion [w, x, y, z]
    """
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _quat_conjugate(q):
    """Return conjugate of quaternion [w, x, y, z] → [w, -x, -y, -z].
    
    Args:
        q: array-like [w, x, y, z]
        
    Returns:
        np.ndarray: conjugate quaternion
    """
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_rotate_vector(q, v):
    """Rotate a 3D vector by a quaternion.
    
    Computes q * [0, v] * q_conj and returns the vector part.
    
    Args:
        q: array-like [w, x, y, z] unit quaternion
        v: array-like [x, y, z] vector
        
    Returns:
        np.ndarray: rotated vector [x, y, z]
    """
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    q_v = np.array([0.0, v[0], v[1], v[2]])
    q_conj = _quat_conjugate(q)
    result = _quat_multiply(_quat_multiply(q, q_v), q_conj)
    return result[1:]
