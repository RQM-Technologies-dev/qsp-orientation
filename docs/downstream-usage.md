# Downstream Usage

This document explains how downstream systems should consume `qsp-orientation` and gives representative architectural patterns for common use cases.

`qsp-orientation` is a composable building block. It provides reliable orientation state — it does not contain navigation engines, hardware drivers, or application control loops. Those belong in downstream repositories.

---

## Frame Transforms in Navigation Systems

Inertial navigation systems require consistent frame bookkeeping. Use `body_to_world` and `world_to_body` to transform measured vectors (acceleration, velocity, magnetic field) between the sensor body frame and the world/navigation frame.

```python
from qsp_orientation import euler_to_quaternion, body_to_world, world_to_body
import numpy as np

# Attitude quaternion estimated from IMU fusion
q = euler_to_quaternion(0.0, 0.1, 0.3)  # roll=0, pitch≈5.7°, yaw≈17.2°

# Accelerometer reading in body frame
accel_body = np.array([0.1, -0.05, 9.79])

# Project into world frame for dead-reckoning
accel_world = body_to_world(q, accel_body)
```

The `compose_rotations` and `invert_rotation` functions are also useful when chaining frame transforms (e.g., body → sensor → world).

---

## Diagnostics in Health-Monitoring Pipelines

For safety-critical or production systems, wrap sensor fusion with continuous orientation health checks. The diagnostics module provides all the building blocks:

```python
from qsp_orientation import (
    madgwick_update, drift_angle,
    gyro_stability_metric, accel_consistency_metric,
    orientation_health_score,
)
import numpy as np

# --- Fusion step ---
q = madgwick_update(q, gyro, accel, beta=0.1, dt=0.01)

# --- Health check ---
drift = drift_angle(q_reference, q)
gyro_samples = ...  # recent N×3 gyro readings
stability = gyro_stability_metric(gyro_samples)
accel_score = accel_consistency_metric(accel)
health = orientation_health_score(drift, stability, accel_score)

if health < 0.7:
    trigger_orientation_fault_flag()
```

This pattern is suitable for real-time health monitoring in robotics controllers, drone flight controllers, or AR/VR tracking pipelines.

---

## IMU Integration in Robotics and Drone Contexts

Use the IMU module for initial orientation bootstrapping and gyro propagation between sensor fusion steps:

```python
from qsp_orientation import accel_tilt_estimate, integrate_gyro, madgwick_update
import numpy as np

# Bootstrap orientation from accelerometer at startup
q = accel_tilt_estimate(np.array([0.0, 0.0, 9.81]))

# Main loop
for accel, gyro, dt in sensor_stream():
    q = madgwick_update(q, gyro, accel, beta=0.1, dt=dt)
```

For gyroscope-only propagation (e.g., high-rate prediction steps between slower fusion updates):

```python
q = integrate_gyro(q, gyro, dt=0.001)  # 1 kHz gyro rate
```

For systems with known gyro bias, apply correction before integration:

```python
from qsp_orientation import gyro_bias_correction
gyro_corrected = gyro_bias_correction(gyro_raw, bias_estimate)
q = integrate_gyro(q, gyro_corrected, dt=dt)
```

---

## Attitude Interpolation for Smooth Trajectories

Use `slerp` when smooth interpolation between two known orientations is required — for example, in trajectory replanning, animation, or interpolating between sparse sensor updates:

```python
from qsp_orientation import euler_to_quaternion, slerp

q_start = euler_to_quaternion(0.0, 0.0, 0.0)
q_end   = euler_to_quaternion(0.0, 0.0, 1.5708)  # 90° yaw rotation

# Interpolate at 50% between start and end
q_mid = slerp(q_start, q_end, t=0.5)
```

---

## What to Build Downstream (Not Here)

The following capabilities belong in downstream application repositories, not in `qsp-orientation`:

| Capability | Belongs in |
|---|---|
| GPS/INS integration | `quaternionic-navigation` or application repo |
| Full SLAM pipelines | Robotics application repo |
| Hardware sensor drivers | Application / HAL layer |
| Drone flight control logic | Drone application repo |
| AR/VR rendering or display | Application layer |
| Navigation state machines | Application repo |
| Product-specific calibration pipelines | Application / calibration tooling repo |
