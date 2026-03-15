# AGENTS.md

This file describes the coding conventions and agent guidelines for `qsp-orientation`.

## Repository Purpose

`qsp-orientation` is the **orientation and state-estimation layer** of the RQM Technologies QSP (Quaternionic Signal Processing) ecosystem. It provides quaternion-based attitude estimation, coordinate frame transforms, IMU integration, sensor fusion, and drift diagnostics for systems where orientation correctness is critical.

Target domains: robotics, drones, inertial navigation, AR/VR tracking, autonomy platforms, and sensor alignment pipelines.

## QSP Ecosystem Position

```
┌─────────────────────────────────────────────────────────────────┐
│                     QSP Ecosystem (RQM Technologies)            │
├─────────────────────┬───────────────────────────────────────────┤
│  qsp-core           │  Shared quaternion algebra primitives      │
│  (foundation)       │  (multiply, conjugate, normalize, etc.)    │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-orientation    │  Attitude estimation, frame transforms,    │
│  ◄ THIS REPO        │  IMU fusion, drift diagnostics             │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-fft            │  Quaternion-domain FFT / spectral tools    │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-filter         │  Signal filtering and normalization        │
├─────────────────────┼───────────────────────────────────────────┤
│  qsp-modulation     │  Quaternion-based digital modulation       │
├─────────────────────┴───────────────────────────────────────────┤
│  Downstream: quaternionic-navigation, robotics SDKs,            │
│  drone stacks, autonomy platforms, AR/VR tracking               │
└─────────────────────────────────────────────────────────────────┘
```

## Boundary Rules for Agents

**Always respect these boundaries when making changes:**

1. **This repo is orientation-focused.** Do not add code that belongs in `qsp-core` (general quaternion algebra), `qsp-fft` (spectral analysis), `qsp-filter` (signal filtering), or `qsp-modulation` (modulation schemes).

2. **Do not expand `_quat_*` helpers in `utils.py`** unless strictly necessary. These helpers (`_quat_normalize`, `_quat_multiply`, `_quat_conjugate`, `_quat_rotate_vector`) exist because `qsp-core` is not yet a consumable package. They are temporary scaffolding. Broad quaternion math primitives belong in `qsp-core`, not here.

3. **Do not absorb system-level logic.** Product-specific robotics stacks, GPS/INS navigation engines, hardware drivers, and application control loops belong in downstream repos — not here.

4. **Public API changes require care.** All public functions are exported from `qsp/orientation/__init__.py`. Adding or removing exports is a breaking change. Renaming or changing signatures requires updating `__init__.py`, `docs/api-overview.md`, and — if relevant — `docs/downstream-usage.md`.

5. **Keep the repo composable.** Every function should do one clearly defined thing. Avoid side effects, global state, or hidden cross-module dependencies.

6. **Preserve repository focus.** This repo should remain focused and lightweight. New additions must fit one of the six established module categories (attitude, frames, IMU, fusion, diagnostics, utils). Anything else is out of scope.

## Design Rules

- Pure functional design: no unnecessary classes
- Small, focused functions with clear docstrings
- Quaternions represented as numpy arrays `[w, x, y, z]`
- Euler angles in radians by default
- All public functions exported from `qsp/orientation/__init__.py`
- Minimal dependencies: only `math` and `numpy`

## Module Boundaries

| Module | Responsibility |
|---|---|
| `utils.py` | Public vector helpers + internal `_quat_*` primitives (temporary, pending qsp-core) |
| `attitude.py` | Euler/quaternion/matrix conversions, SLERP |
| `frames.py` | Coordinate frame transforms (body ↔ world) |
| `imu.py` | IMU integration and tilt estimation |
| `fusion.py` | Sensor fusion filters (Complementary, Madgwick, Mahony) |
| `diagnostics.py` | Drift metrics and orientation health scoring |

## Testing

Run tests with `pytest`. Target 80–100 tests across all modules.

Tests live in `tests/` and are named `test_<module>.py`. Do not remove or weaken existing tests. When adding functionality, add corresponding tests.

## Contribution Guidance

- Do not introduce new dependencies without strong justification.
- Do not create classes where a function suffices.
- Do not add business logic, hardware integration, or simulation frameworks.
- Do not create new modules outside the six established categories without architectural review.
- When in doubt about scope, consult `docs/architecture.md` and `README.md`.
