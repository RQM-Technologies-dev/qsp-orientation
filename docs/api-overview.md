# API Overview

All public functions are importable directly from `qsp.orientation`. Internal helpers (prefixed with `_`) are not part of the public API and should not be used by downstream consumers.

```python
from qsp.orientation import euler_to_quaternion, body_to_world, madgwick_update, ...
```

---

## Attitude Conversions

Functions for converting between attitude representations: Euler angles, quaternions, rotation matrices, and SLERP interpolation.

### `euler_to_quaternion(roll, pitch, yaw) → np.ndarray`
ZYX Euler angles (radians) to unit quaternion `[w, x, y, z]`.
```python
from qsp.orientation import euler_to_quaternion
q = euler_to_quaternion(0.0, 0.0, 1.5708)  # 90 deg yaw
```

### `quaternion_to_euler(q) → (roll, pitch, yaw)`
Extract ZYX Euler angles (radians) from a unit quaternion.

### `quaternion_to_rotation_matrix(q) → np.ndarray (3×3)`
Convert unit quaternion to 3×3 rotation matrix.

### `rotation_matrix_to_quaternion(R) → np.ndarray`
Convert 3×3 rotation matrix to unit quaternion using Shepperd's method.

### `slerp(q1, q2, t) → np.ndarray`
Spherical linear interpolation between two quaternions. `t ∈ [0, 1]`.

### `relative_rotation(q1, q2) → np.ndarray`
Relative rotation from `q1` to `q2`: `conj(q1) * q2`.

---

## Frame Transforms

Functions for rotating vectors between reference frames and composing or inverting rotations.

### `transform_vector(q, v) → np.ndarray`
Rotate a 3D vector `v` by quaternion `q`.

### `body_to_world(q_body_to_world, v_body) → np.ndarray`
Transform a vector from body frame to world frame.

### `world_to_body(q_body_to_world, v_world) → np.ndarray`
Transform a vector from world frame to body frame.

### `compose_rotations(q1, q2) → np.ndarray`
Compose two rotations: apply `q1` first, then `q2`. Returns `q2 * q1`.

### `invert_rotation(q) → np.ndarray`
Return the inverse (conjugate) of a unit quaternion.

---

## IMU Integration

Functions for processing raw IMU sensor data into orientation estimates.

### `integrate_gyro(q, omega, dt) → np.ndarray`
First-order gyroscope integration. Propagates quaternion `q` given angular velocity `omega` (rad/s) over timestep `dt`.

### `accel_tilt_estimate(accel) → np.ndarray`
Estimate roll and pitch from accelerometer vector (assumes gravity dominates). Returns quaternion.

### `gyro_bias_correction(omega, bias) → np.ndarray`
Subtract a bias vector from a gyroscope reading.

### `orientation_from_imu(accel, gyro, dt, q0=None) → np.ndarray`
Single-step IMU orientation update combining gyro integration and accel tilt estimation.

---

## Sensor Fusion

Recursive filter updates that blend gyroscope and accelerometer (and optionally magnetometer) data into a fused orientation estimate.

### `complementary_filter(q_gyro, accel, alpha) → np.ndarray`
Blend gyro and accel estimates via SLERP. `alpha=1.0` is pure gyro; `alpha=0.0` is pure accel.

### `madgwick_update(q, gyro, accel, beta=0.1, dt=0.01, mag=None) → np.ndarray`
Madgwick gradient descent filter step. Optionally accepts magnetometer vector for yaw correction.

### `mahony_update(q, gyro, accel, Kp=2.0, Ki=0.005, dt=0.01, integral_error=None, mag=None) → tuple`
Mahony complementary filter step. Returns `(q_new, integral_error)`.

---

## Diagnostics

Functions for measuring orientation quality, detecting drift, and computing composite health scores. Suitable for production health-monitoring pipelines.

### `drift_angle(q1, q2, degrees=False) → float`
Angular difference between two orientations, in radians (or degrees if `degrees=True`).

### `quaternion_distance(q1, q2) → float`
Scalar distance `∈ [0, 1]` between two orientations (0 = identical).

### `gyro_stability_metric(samples) → float`
Mean variance of gyro samples (shape N×3). Lower = more stable.

### `accel_consistency_metric(accel_vector, expected_gravity=9.81) → float`
Score `∈ [0, 1]` for how closely accel magnitude matches expected gravity.

### `mag_consistency_metric(mag_vector, reference_field) → float`
Normalized dot-product alignment score `∈ [0, 1]` between measured and reference field.

### `orientation_health_score(drift_rad, gyro_stability, accel_consistency, mag_consistency=None) → float`
Combined orientation health score `∈ [0, 1]` from drift, gyro stability, and sensor consistency.

---

## Utility Helpers

General-purpose vector utilities. These are part of the public API and may also be useful to downstream consumers.

### `normalize_vector(v) → np.ndarray`
Normalize a vector to unit length. Raises `ValueError` on zero vector.
```python
from qsp.orientation import normalize_vector
normalize_vector([3.0, 0.0, 0.0])  # → [1.0, 0.0, 0.0]
```

### `vector_norm(v) → float`
Compute the L2 norm of a vector.
```python
vector_norm([3.0, 4.0, 0.0])  # → 5.0
```

### `clamp(val, min_val, max_val) → float`
Clamp a value between bounds.
```python
clamp(1.5, 0.0, 1.0)  # → 1.0
```

---

## Internal Helpers (not public API)

The following functions in `utils.py` are **internal** and prefixed with `_`. They exist because `qsp-core` is not yet available as a consumable package. Do not use them in downstream code.

| Symbol | Description |
|---|---|
| `_quat_normalize(q)` | Normalize a `[w, x, y, z]` quaternion |
| `_quat_multiply(q1, q2)` | Hamilton product of two quaternions |
| `_quat_conjugate(q)` | Conjugate of a quaternion |
| `_quat_rotate_vector(q, v)` | Rotate 3D vector by quaternion |

These will migrate to `qsp-core` once that package is available as a stable installable dependency.
