<img src="https://github.com/RQM-Technologies-dev/qsp-orientation/actions/workflows/ci.yml/badge.svg">

# qsp-orientation

Quaternion-based attitude estimation, coordinate frame transforms, IMU integration, sensor fusion, and drift diagnostics — the orientation and state-estimation layer of the [RQM Technologies QSP ecosystem](https://github.com/RQM-Technologies-dev/qsp-ecosystem).

## Overview

`qsp-orientation` provides a focused Python library for:

- **Attitude representation**: Euler↔quaternion↔rotation-matrix conversions, SLERP
- **Coordinate frame transforms**: body↔world transforms, rotation composition
- **IMU integration**: gyroscope integration, accelerometer tilt estimation
- **Sensor fusion**: complementary filter, Madgwick filter, Mahony filter
- **Drift diagnostics**: angular drift, gyro stability, orientation health scoring

**Quaternion convention**: `[w, x, y, z]` (scalar-first). Euler angles use ZYX (aerospace) convention.

---

## Role in the QSP Ecosystem

`qsp-orientation` is the **Layer-1 QSP library for orientation-aware state estimation and frame handling**.

The RQM Technologies QSP (Quaternionic Signal Processing) ecosystem is a collection of composable, domain-bounded libraries that apply quaternion mathematics across signal processing, communications, and physical-world estimation. `qsp-orientation` occupies the layer responsible for:

- attitude representation and conversion helpers
- coordinate frame transforms between body and world reference frames
- IMU integration and sensor fusion
- drift detection and orientation-health diagnostics

Within this ecosystem, `qsp-orientation` acts as a **bridge between low-level quaternion primitives and higher-level real-world systems** such as robotics, drones, inertial navigation, AR/VR tracking, and autonomy platforms. It transforms raw sensor data into trustworthy orientation state that downstream systems can act on.

### QSP Ecosystem Map

```
┌─────────────────────────────────────────────────────────────────┐
│                     QSP Ecosystem (RQM Technologies)            │
├─────────────────────┬───────────────────────────────────────────┤
│  qsp-core           │  Shared quaternion algebra primitives,     │
│  (foundation)       │  conventions, and math utilities           │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-orientation    │  Attitude estimation, frame transforms,    │
│  ◄ THIS REPO        │  IMU fusion, drift diagnostics             │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-fft            │  Quaternion-domain FFT and spectral tools  │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-filter         │  Signal filtering and normalization        │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-modulation     │  Quaternion-based digital modulation       │
├─────────────────────┴───────────────────────────────────────────┤
│  Downstream: quaternionic-navigation, robotics SDKs,            │
│  drone stacks, autonomy platforms, AR/VR tracking               │
└─────────────────────────────────────────────────────────────────┘
```

---

## QSP Perspective

Within the QSP ecosystem, orientation is not just a metadata convenience — it is part of the signal and state structure itself. `qsp-orientation` provides practical tools for systems where frame relationships, attitude stability, and directional coherence determine whether sensor or control information remains trustworthy. Incorrect or drifting orientation corrupts every downstream computation that depends on it; this library exists to make that failure mode detectable and correctable.

---

## Boundary

### What belongs in `qsp-orientation`

- Quaternion-based attitude representation and conversion (Euler, rotation matrix, SLERP)
- Coordinate frame transforms between body, world, and sensor frames
- Orientation interpolation utilities
- IMU integration helpers (gyroscope integration, accelerometer tilt)
- Complementary, Madgwick, and Mahony sensor fusion updates
- Drift-angle and orientation-health-score diagnostics

### What does not belong here

| Concern | Belongs in |
|---|---|
| Foundational quaternion algebra (multiply, conjugate, normalize) intended for reuse across all repos | `qsp-core` |
| General-purpose FFT and spectral analysis | `qsp-fft` |
| Generic signal filtering and normalization | `qsp-filter` |
| Digital modulation schemes | `qsp-modulation` |
| Product-specific robotics or navigation business logic | downstream application repos |
| Hardware drivers, large simulation frameworks, or hardware integration layers | downstream application repos |

**Note on local quaternion primitives**: `utils.py` currently contains internal helpers (`_quat_normalize`, `_quat_multiply`, `_quat_conjugate`, `_quat_rotate_vector`) because `qsp-core` is not yet available as a consumable package. These are temporary scaffolding. They are prefixed with `_` to reflect their internal-only status. As `qsp-core` becomes installable, these should migrate there rather than expand here.

---

## Why This Repo Matters Commercially

Orientation estimation is a foundational problem across a wide range of engineering domains. `qsp-orientation` addresses this in a composable, dependency-light package that can be embedded wherever quaternion-based attitude tracking is needed:

| Domain | Use case |
|---|---|
| **Robotics / robot arms** | Joint attitude tracking, end-effector frame alignment |
| **Drones / UAVs** | Flight stabilization, AHRS, attitude-mode control |
| **Inertial navigation** | Dead-reckoning orientation propagation, IMU fusion |
| **AR/VR head tracking** | Low-latency head pose estimation, drift suppression |
| **Sensor alignment** | Multi-sensor frame registration, calibration pipelines |
| **Autonomy platforms** | Reliable orientation state for planning and control |

The diagnostics module — drift detection, gyro stability, and orientation health scoring — is particularly valuable in production systems where silent orientation failure is a safety or reliability concern.

---

## Relationship to qsp-core

`qsp-core` is the intended home for shared quaternion algebra primitives that are reused across all QSP repositories. `qsp-orientation` is designed to build orientation-specific operations on top of those foundations.

**Current implementation constraint**: Some quaternion primitives (normalize, multiply, conjugate, rotate vector) are implemented locally in `utils.py` because `qsp-core` is not yet available as a consumable package. This is a deliberate, temporary packaging decision — not a design flaw. The internal helpers are prefixed with `_quat_` to signal their provisional status.

**Long-term direction**: As `qsp-core` matures and becomes installable, shared primitives will migrate there. `qsp-orientation` will then depend on `qsp-core` for those foundations rather than re-implementing them locally.

Do not expand the local `_quat_*` helpers in `utils.py` beyond what is strictly necessary for orientation-specific logic. Any broadly reusable quaternion math belongs in `qsp-core`.

---

## Downstream Systems

`qsp-orientation` is designed to be consumed by Layer-2 repositories and application-level systems, including:

- **quaternionic-navigation** — inertial navigation engines that require reliable orientation state
- **Robotics orientation SDKs** — platform-specific attitude tracking and frame management
- **Drone stabilization stacks** — real-time attitude hold and AHRS pipelines
- **Autonomy platforms** — systems that need trustworthy directional state for planning
- **possibly quaternionic-modem** — when frame or orientation awareness matters for signal receivers

These downstream systems should treat `qsp-orientation` as a stable, composable building block — not as an integration point for business logic or hardware drivers.

---

## Future Extensions

Appropriate future additions to `qsp-orientation` include:

- Magnetometer calibration helpers
- Quaternion trajectory smoothing
- Streaming / incremental orientation estimators
- Covariance-aware drift diagnostics
- Frame-consistency validation utilities
- Sensor quality monitoring helpers

The following work does **not** belong in this repository and should be built in downstream repos:

- Complete robotics stacks or control loops
- GPS/INS navigation engines
- Application-specific hardware integration layers
- Signal-processing utilities unrelated to orientation
- Large simulation frameworks

---

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
from qsp.orientation import euler_to_quaternion, quaternion_to_euler

q = euler_to_quaternion(0.0, 0.0, math.radians(90))
print(q)  # [0.7071, 0.0, 0.0, 0.7071]

roll, pitch, yaw = quaternion_to_euler(q)
print(math.degrees(yaw))  # 90.0
```

### Coordinate Frame Transforms

```python
import numpy as np
from qsp.orientation import euler_to_quaternion, body_to_world, world_to_body

q = euler_to_quaternion(0.0, 0.0, math.radians(45))
v_body = np.array([1.0, 0.0, 0.0])
v_world = body_to_world(q, v_body)
```

### IMU Integration

```python
import numpy as np
from qsp.orientation import integrate_gyro, accel_tilt_estimate

q = np.array([1.0, 0.0, 0.0, 0.0])
omega = np.array([0.0, 0.0, 1.0])  # 1 rad/s yaw
q = integrate_gyro(q, omega, dt=0.01)
```

### Sensor Fusion

```python
import numpy as np
from qsp.orientation import complementary_filter, madgwick_update

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
from qsp.orientation import (
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
