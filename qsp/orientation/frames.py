"""Coordinate frame transformation utilities."""
import numpy as np
from qsp.orientation.utils import _quat_normalize, _quat_multiply, _quat_conjugate, _quat_rotate_vector


def transform_vector(q, v):
    """Rotate vector v by quaternion q.
    
    Computes v' = q * [0, v] * q_conj
    
    Args:
        q: array-like [w, x, y, z] unit quaternion
        v: array-like [x, y, z] vector
        
    Returns:
        np.ndarray: rotated vector [x, y, z]
    """
    q = _quat_normalize(np.asarray(q, dtype=float))
    v = np.asarray(v, dtype=float)
    return _quat_rotate_vector(q, v)


def body_to_world(q_body_to_world, v_body):
    """Transform vector from body frame to world frame.
    
    Args:
        q_body_to_world: array-like [w, x, y, z] quaternion rotating body to world
        v_body: array-like [x, y, z] vector in body frame
        
    Returns:
        np.ndarray: vector in world frame [x, y, z]
    """
    return transform_vector(q_body_to_world, v_body)


def world_to_body(q_body_to_world, v_world):
    """Transform vector from world frame to body frame.
    
    Args:
        q_body_to_world: array-like [w, x, y, z] quaternion rotating body to world
        v_world: array-like [x, y, z] vector in world frame
        
    Returns:
        np.ndarray: vector in body frame [x, y, z]
    """
    q_world_to_body = _quat_conjugate(_quat_normalize(np.asarray(q_body_to_world, dtype=float)))
    return transform_vector(q_world_to_body, v_world)


def compose_rotations(q1, q2):
    """Compose two rotations: apply q1 first, then q2.
    
    Returns q2 * q1 (Hamilton product, applying q1 first).
    
    Args:
        q1: array-like [w, x, y, z] first rotation
        q2: array-like [w, x, y, z] second rotation
        
    Returns:
        np.ndarray: composed quaternion [w, x, y, z]
    """
    q1 = _quat_normalize(np.asarray(q1, dtype=float))
    q2 = _quat_normalize(np.asarray(q2, dtype=float))
    return _quat_normalize(_quat_multiply(q2, q1))


def invert_rotation(q):
    """Invert a rotation quaternion (return conjugate for unit quaternion).
    
    Args:
        q: array-like [w, x, y, z] unit quaternion
        
    Returns:
        np.ndarray: inverse quaternion [w, x, y, z]
    """
    return _quat_conjugate(_quat_normalize(np.asarray(q, dtype=float)))
