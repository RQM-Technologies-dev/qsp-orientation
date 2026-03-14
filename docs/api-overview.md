# API Overview

All public functions are importable directly from `qsp_orientation`.

## utils

### `normalize_vector(v) → np.ndarray`
Normalize a vector to unit length. Raises `ValueError` if zero vector.
```python
from qsp_orientation import normalize_vector
normalize_vector([3.0, 0.0, 0.0])  # → [1.0, 0.0, 0.0]
```

### `vector_norm(v) → float`
Compute L2 norm.
```python
vector_norm([3.0, 4.0, 0.0])  # → 5.0
```

### `clamp(val, min_val, max_val) → float`
Clamp value between bounds.
```python
clamp(1.5, 0.0, 1.0)  # → 1.0
```

---

## attitude

### `euler_to_quaternion(roll, pitch, yaw) → np.ndarray`
ZYX Euler angles to quaternion. All angles in radians.
```python
from qsp_orientation import euler_to_quaternion
q = euler_to_quaternion(0.0, 0.0, 1.5708)  # 90 deg yaw
```

### `quaternion_to_euler(q) → (roll, pitch, yaw)`
Extract ZYX Euler angles from quaternion.

### `quaternion_to_rotation_matrix(q) → np.ndarray (3×3)`
Convert unit quaternion to rotation matrix.

### `rotation_matrix_to_quaternion(R) → np.ndarray`
Convert rotation matrix to quaternion using Shepperd method.

### `slerp(q1, q2, t) → np.ndarray`
Spherical linear interpolation, `t ∈ [0, 1]`.

### `relative_rotation(q1, q2) → np.ndarray`
Relative rotation from q1 to q2: `conj(q1) * q2`.

---

## frames

### `transform_vector(q, v) → np.ndarray`
Rotate vector `v` by quaternion `q`.

### `body_to_world(q_body_to_world, v_body) → np.ndarray`
Transform vector from body to world frame.

### `world_to_body(q_body_to_world, v_world) → np.ndarray`
Transform vector from world to body frame.

### `compose_rotations(q1, q2) → np.ndarray`
Compose rotations: apply `q1` first, then `q2`. Returns `q2 * q1`.

### `invert_rotation(q) → np.ndarray`
Return inverse (conjugate) of unit quaternion.

---

## imu

### `integrate_gyro(q, omega, dt) → np.ndarray`
First-order gyroscope integration.

### `accel_tilt_estimate(accel) → np.ndarray`
Estimate roll/pitch from accelerometer (assumes gravity dominates).

### `gyro_bias_correction(omega, bias) → np.ndarray`
Subtract bias from gyroscope reading.

### `orientation_from_imu(accel, gyro, dt, q0=None) → np.ndarray`
Single-step IMU orientation update.

---

## fusion

### `complementary_filter(q_gyro, accel, alpha) → np.ndarray`
Blend gyro and accel estimates via SLERP. `alpha=1.0` → pure gyro.

### `madgwick_update(q, gyro, accel, beta=0.1, dt=0.01, mag=None) → np.ndarray`
Madgwick gradient descent filter step.

### `mahony_update(q, gyro, accel, Kp=2.0, Ki=0.005, dt=0.01, integral_error=None, mag=None) → tuple`
Mahony complementary filter step. Returns `(q_new, integral_error)`.

---

## diagnostics

### `drift_angle(q1, q2, degrees=False) → float`
Angular difference between two orientations.

### `quaternion_distance(q1, q2) → float`
Scalar distance `∈ [0, 1]`, 0 = identical.

### `gyro_stability_metric(samples) → float`
Mean variance of gyro samples (shape N×3).

### `accel_consistency_metric(accel_vector, expected_gravity=9.81) → float`
Score `∈ [0, 1]` for accel magnitude vs expected gravity.

### `mag_consistency_metric(mag_vector, reference_field) → float`
Normalized dot product alignment score `∈ [0, 1]`.

### `orientation_health_score(drift_rad, gyro_stability, accel_consistency, mag_consistency=None) → float`
Combined health score `∈ [0, 1]`.
