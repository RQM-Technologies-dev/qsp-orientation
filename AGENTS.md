# AGENTS.md

This file describes the coding conventions and agent guidelines for `qsp-orientation`.

## Repository Purpose

`qsp-orientation` provides orientation estimation, coordinate frame transforms, and IMU integration utilities for robotics, navigation, drones, AR/VR, and signal processing systems.

## Design Rules

- Pure functional design: no unnecessary classes
- Small, focused functions with clear docstrings
- Quaternions represented as numpy arrays `[w, x, y, z]`
- Euler angles in radians by default
- All public functions exported from `qsp_orientation/__init__.py`
- Minimal dependencies: only `math` and `numpy`

## Module Boundaries

| Module | Responsibility |
|---|---|
| `utils.py` | Quaternion math primitives, vector helpers |
| `attitude.py` | Euler/quaternion/matrix conversions, SLERP |
| `frames.py` | Coordinate frame transforms |
| `imu.py` | IMU integration and tilt estimation |
| `fusion.py` | Sensor fusion filters |
| `diagnostics.py` | Drift metrics and orientation health |

## Testing

Run tests with `pytest`. Target 80–100 tests across all modules.
