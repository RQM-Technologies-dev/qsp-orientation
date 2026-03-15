"""Demo: Simulated gyro integration and complementary filter."""
import math
import numpy as np
from qsp.orientation import (
    integrate_gyro,
    complementary_filter,
    quaternion_to_euler,
)


def main():
    print("=== IMU Demo ===\n")
    
    # Simulate steady rotation around Z axis
    dt = 0.01
    omega = np.array([0.0, 0.0, math.radians(90)])  # 90 deg/s around Z
    q = np.array([1.0, 0.0, 0.0, 0.0])
    gravity = np.array([0.0, 0.0, 9.81])
    
    print("Gyro integration (90 deg/s yaw for 1 second):")
    for step in range(0, 101, 25):
        r, p, y = quaternion_to_euler(q)
        print(f"  t={step*dt:.2f}s: yaw={math.degrees(y):.1f} deg")
        if step < 100:
            for _ in range(25):
                q = integrate_gyro(q, omega, dt)
    
    # Complementary filter demo
    print("\nComplementary filter (alpha=0.98):")
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for step in range(5):
        q = integrate_gyro(q, omega, dt)
        q_fused = complementary_filter(q, gravity, alpha=0.98)
        r, p, y = quaternion_to_euler(q_fused)
        print(f"  step {step+1}: yaw={math.degrees(y):.2f} deg")


if __name__ == "__main__":
    main()
