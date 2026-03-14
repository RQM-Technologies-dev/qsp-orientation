# Repo Roadmap

This document describes the intended future growth of `qsp-orientation` and clarifies what work belongs here versus in downstream or sibling repositories.

---

## Guiding Principle

`qsp-orientation` should remain **focused and composable**. Every addition should serve orientation estimation, frame handling, IMU integration, sensor fusion, or drift diagnostics — and nothing else. If a proposed feature could be its own standalone library, or clearly belongs in a downstream application, it does not belong here.

---

## Appropriate Future Extensions

The following are in-scope additions that would strengthen the orientation layer without violating its boundaries:

### Sensor Calibration Helpers
- **Magnetometer calibration** (hard-iron and soft-iron correction helpers)
- **Accelerometer calibration** utilities for multi-position static calibration

### Trajectory and Interpolation
- **Quaternion trajectory smoothing** (e.g., cubic SLERP, squad interpolation)
- Utility for building smooth orientation sequences from sparse waypoints

### Streaming Estimators
- **Incremental / streaming orientation estimators** for high-rate sensor pipelines
- Stateful wrapper around fusion functions for use in continuous loops (designed as simple dataclass, not class hierarchy)

### Advanced Diagnostics
- **Covariance-aware drift diagnostics** — propagating uncertainty alongside attitude estimates
- **Frame-consistency validation** — checking that a sequence of transforms is self-consistent
- **Sensor quality monitoring helpers** — per-axis noise characterization, step-change detection

### Utility Improvements
- Additional Euler convention support (XYZ, XZY, etc.) with explicit convention labeling
- Rotation vector / axis-angle conversion helpers
- Tait-Bryan angle helpers for aerospace and robotics toolchain interoperability

---

## What Should Not Be Added Here

The following work does not belong in `qsp-orientation` and should be built in downstream or sibling repositories:

| Proposed Feature | Belongs in |
|---|---|
| Complete robotics stacks or control loops | Robotics application repo |
| GPS/INS navigation engines | `quaternionic-navigation` or navigation repo |
| Application-specific calibration pipelines | Application-layer calibration tooling |
| Hardware sensor drivers or HAL layers | Application / hardware abstraction layer |
| Simulation frameworks | Separate simulation tooling repo |
| Signal-processing utilities unrelated to orientation | `qsp-filter`, `qsp-fft`, or `qsp-core` |
| Digital modulation or communications logic | `qsp-modulation` |
| Broad quaternion algebra primitives | `qsp-core` |
| AR/VR rendering, display, or scene management | Application layer |

---

## Relationship to qsp-core Convergence

As `qsp-core` matures into a stable, installable package, the following internal helpers in `utils.py` should migrate there rather than expanding locally:

- `_quat_normalize`
- `_quat_multiply`
- `_quat_conjugate`
- `_quat_rotate_vector`

Until that migration happens, keep these helpers minimal and prefixed with `_`. Do not expose them as public API.

---

## Version and Stability Notes

- The public API (all exports from `qsp_orientation/__init__.py`) should be treated as stable.
- Signature changes to public functions are breaking changes and require a version bump.
- Internal `_quat_*` helpers are not stable API and may change or be removed when `qsp-core` is available.
