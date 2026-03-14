# Architecture

## Repository Purpose

`qsp-orientation` provides quaternion-based orientation estimation, coordinate frame transforms, and IMU integration utilities. It targets applications in robotics, navigation, drones, AR/VR, and signal processing systems.

## Relationship with qsp-core

`qsp-core` defines the overall QSP (Quaternion Signal Processing) ecosystem design boundary. `qsp-orientation` is a standalone, self-contained implementation of the orientation estimation layer. All quaternion math is implemented internally in `utils.py` — no runtime dependency on `qsp-core` is required or used.

This design ensures:
- Zero external runtime dependencies (only `math` and `numpy`)
- Easy installation and testing in isolated environments
- Clear module boundaries aligned with the QSP design boundary specification

## Module Boundaries

| Module | Responsibility |
|---|---|
| `utils.py` | Quaternion math primitives (`_quat_*`), vector helpers |
| `attitude.py` | Euler/quaternion/matrix conversions, SLERP |
| `frames.py` | Coordinate frame transforms (body ↔ world) |
| `imu.py` | IMU integration and tilt estimation |
| `fusion.py` | Sensor fusion filters (Complementary, Madgwick, Mahony) |
| `diagnostics.py` | Drift metrics and orientation health scoring |

## Data Flow

```
Sensor Data (accel, gyro, mag)
        │
        ▼
    imu.py ──────────────────► attitude.py
        │                          │
        ▼                          ▼
    fusion.py ──────────────► frames.py
        │
        ▼
  diagnostics.py
```

## Quaternion Convention

All quaternions are represented as `numpy` arrays in `[w, x, y, z]` order (scalar-first). Unit quaternions satisfy `w²+x²+y²+z²=1`.

Euler angles use ZYX (aerospace) convention in radians:
- `R = Rz(yaw) * Ry(pitch) * Rx(roll)`
