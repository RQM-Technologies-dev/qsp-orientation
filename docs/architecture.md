# Architecture

## Repository Purpose

`qsp-orientation` is the **Layer-1 QSP library for orientation-aware state estimation and frame handling** within the RQM Technologies QSP (Quaternionic Signal Processing) ecosystem. It provides quaternion-based attitude estimation, coordinate frame transforms, IMU integration, sensor fusion, and drift diagnostics.

---

## Role in the QSP Ecosystem

The QSP ecosystem is a collection of composable, domain-bounded libraries that apply quaternion mathematics across signal processing, communications, and physical-world estimation. `qsp-orientation` occupies the layer responsible for practical orientation operations:

| Library | Layer | Role |
|---|---|---|
| `qsp-core` | Foundation | Shared quaternion algebra primitives and conventions |
| **`qsp-orientation`** | **Layer 1** | **Attitude estimation, frame transforms, IMU fusion, drift diagnostics** |
| `qsp-fft` | Layer 1 | Quaternion-domain FFT and spectral analysis |
| `qsp-filter` | Layer 1 | Signal filtering and normalization |
| `qsp-modulation` | Layer 1 | Digital modulation schemes |
| Downstream repos | Layer 2+ | Application-specific systems (navigation, robotics, drones, etc.) |

`qsp-orientation` acts as a **bridge between low-level quaternion primitives and higher-level real-world systems**. It transforms raw IMU sensor data into trustworthy orientation state that downstream systems can safely consume.

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

---

## QSP Perspective

Within the QSP ecosystem, orientation is not just a metadata convenience — it is part of the signal and state structure itself. `qsp-orientation` provides practical tools for systems where frame relationships, attitude stability, and directional coherence determine whether sensor or control information remains trustworthy. Incorrect or drifting orientation corrupts every downstream computation that depends on it.

---

## Boundary

### What belongs in `qsp-orientation`

- Quaternion-based attitude representation and conversion (Euler, rotation matrix, SLERP)
- Coordinate frame transforms between body, world, and sensor reference frames
- Orientation interpolation utilities
- IMU integration helpers (gyroscope integration, accelerometer tilt)
- Complementary, Madgwick, and Mahony sensor fusion updates
- Drift-angle and orientation-health-score diagnostics

### What does not belong here

| Concern | Belongs in |
|---|---|
| Foundational quaternion algebra (multiply, conjugate, normalize) for reuse across all repos | `qsp-core` |
| General-purpose FFT and spectral analysis | `qsp-fft` |
| Generic signal filtering and normalization | `qsp-filter` |
| Digital modulation schemes | `qsp-modulation` |
| Product-specific robotics or navigation business logic | downstream application repos |
| Large simulation frameworks or hardware integration layers | downstream application repos |

---

## Relationship to qsp-core

`qsp-core` is the intended home for shared quaternion algebra primitives that are reused across all QSP repositories. `qsp-orientation` builds orientation-specific operations on top of those foundations.

**Current implementation constraint**: Some quaternion primitives are implemented locally in `utils.py` (`_quat_normalize`, `_quat_multiply`, `_quat_conjugate`, `_quat_rotate_vector`) because `qsp-core` is not yet available as a consumable package. This is a packaging constraint, not a design flaw. These helpers are prefixed with `_` to reflect their provisional, internal-only status.

**Long-term direction**: As `qsp-core` matures and stabilizes as an installable package, shared primitives should converge there. `qsp-orientation` will then depend on `qsp-core` for those foundations. Do not expand the local `_quat_*` helpers unless strictly required for orientation-specific logic.

---

## Module Boundaries

| Module | Responsibility |
|---|---|
| `utils.py` | Public vector helpers (`normalize_vector`, `vector_norm`, `clamp`) + internal `_quat_*` primitives |
| `attitude.py` | Euler/quaternion/matrix conversions, SLERP |
| `frames.py` | Coordinate frame transforms (body ↔ world) |
| `imu.py` | IMU integration and tilt estimation |
| `fusion.py` | Sensor fusion filters (Complementary, Madgwick, Mahony) |
| `diagnostics.py` | Drift metrics and orientation health scoring |

---

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

---

## Downstream Systems

`qsp-orientation` is intended to support Layer-2 and application-level systems, including:

- **quaternionic-navigation** — inertial navigation engines requiring reliable orientation state
- **Robotics orientation SDKs** — platform-specific attitude tracking and frame management
- **Drone stabilization stacks** — real-time AHRS and attitude-hold pipelines
- **Autonomy platforms** — systems that require trustworthy directional state for planning and control
- **quaternionic-modem** — when frame or orientation awareness matters for signal receivers

These downstream systems should use `qsp-orientation` as a stable, composable building block. They must not push navigation business logic, hardware drivers, or simulation frameworks back into this repo.

---

## Quaternion Convention

All quaternions are represented as `numpy` arrays in `[w, x, y, z]` order (scalar-first). Unit quaternions satisfy `w²+x²+y²+z²=1`.

Euler angles use ZYX (aerospace) convention in radians:
- `R = Rz(yaw) * Ry(pitch) * Rx(roll)`
