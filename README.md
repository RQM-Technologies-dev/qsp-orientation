# qsp-orientation

Quaternion-based attitude estimation, coordinate frame transforms, IMU integration, sensor fusion, and drift diagnostics.

## Overview

`qsp-orientation` provides a self-contained Python library for:

- **Attitude representation**: Euler↔quaternion↔rotation-matrix conversions, SLERP
- **Coordinate frame transforms**: body↔world transforms, rotation composition
- **IMU integration**: gyroscope integration, accelerometer tilt estimation
- **Sensor fusion**: complementary filter, Madgwick filter, Mahony filter
- **Drift diagnostics**: angular drift, gyro stability, orientation health scoring

**Quaternion convention**: `[w, x, y, z]` (scalar-first). Euler angles use ZYX (aerospace) convention.

## Installation

```bash
pip install -e .
```

Or with dev dependencies:

```bash
pip install -e ".[dev]"
```

## Quick Examples

### Euler ↔ Quaternion

```python
import math
from qsp_orientation import euler_to_quaternion, quaternion_to_euler

q = euler_to_quaternion(0.0, 0.0, math.radians(90))
print(q)  # [0.7071, 0.0, 0.0, 0.7071]

roll, pitch, yaw = quaternion_to_euler(q)
print(math.degrees(yaw))  # 90.0
```

### Coordinate Frame Transforms

```python
import numpy as np
from qsp_orientation import euler_to_quaternion, body_to_world, world_to_body

q = euler_to_quaternion(0.0, 0.0, math.radians(45))
v_body = np.array([1.0, 0.0, 0.0])
v_world = body_to_world(q, v_body)
```

### IMU Integration

```python
import numpy as np
from qsp_orientation import integrate_gyro, accel_tilt_estimate

q = np.array([1.0, 0.0, 0.0, 0.0])
omega = np.array([0.0, 0.0, 1.0])  # 1 rad/s yaw
q = integrate_gyro(q, omega, dt=0.01)
```

### Sensor Fusion

```python
import numpy as np
from qsp_orientation import complementary_filter, madgwick_update

gravity = np.array([0.0, 0.0, 9.81])
q = np.array([1.0, 0.0, 0.0, 0.0])
gyro = np.array([0.0, 0.0, 0.1])

# Complementary filter (98% gyro trust)
q = complementary_filter(q, gravity, alpha=0.98)

# Madgwick filter
q = madgwick_update(q, gyro, gravity, beta=0.1, dt=0.01)
```

### Drift Diagnostics

```python
import math
from qsp_orientation import (
    euler_to_quaternion, drift_angle,
    gyro_stability_metric, orientation_health_score,
)
import numpy as np

q1 = euler_to_quaternion(0.0, 0.0, 0.0)
q2 = euler_to_quaternion(0.0, 0.0, math.radians(10))
print(f"Drift: {drift_angle(q1, q2, degrees=True):.2f} degrees")

stable_gyro = np.ones((100, 3)) * 0.01
stability = gyro_stability_metric(stable_gyro)
health = orientation_health_score(drift_angle(q1, q2), stability, accel_consistency=0.99)
print(f"Health score: {health:.3f}")
```

## Testing

```bash
pytest -v
```

The test suite covers 80+ tests across all modules.

## Module Structure

| Module | Responsibility |
|---|---|
| `utils.py` | Quaternion math primitives, vector helpers |
| `attitude.py` | Euler/quaternion/matrix conversions, SLERP |
| `frames.py` | Coordinate frame transforms |
| `imu.py` | IMU integration and tilt estimation |
| `fusion.py` | Sensor fusion filters |
| `diagnostics.py` | Drift metrics and orientation health |

## Drift Diagnostics

The `diagnostics` module provides tools to monitor and evaluate orientation quality:

- **`drift_angle(q1, q2)`**: Angular difference between two orientations in radians or degrees
- **`quaternion_distance(q1, q2)`**: Scalar distance [0, 1] between orientations
- **`gyro_stability_metric(samples)`**: Variance-based noise metric from gyro samples
- **`accel_consistency_metric(accel, expected_gravity)`**: Score how well accel magnitude matches gravity
- **`mag_consistency_metric(mag, reference)`**: Score magnetometer alignment with reference field
- **`orientation_health_score(...)`**: Combined [0, 1] health score from all metrics
