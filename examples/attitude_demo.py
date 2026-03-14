"""Demo: Euler → quaternion, quaternion → rotation matrix, SLERP."""
import math
import numpy as np
from qsp_orientation import (
    euler_to_quaternion,
    quaternion_to_rotation_matrix,
    slerp,
    quaternion_to_euler,
)


def main():
    print("=== Attitude Demo ===\n")
    
    # Euler → quaternion
    roll, pitch, yaw = math.radians(30), math.radians(15), math.radians(45)
    q = euler_to_quaternion(roll, pitch, yaw)
    print(f"Euler (deg): roll={math.degrees(roll):.1f}, pitch={math.degrees(pitch):.1f}, yaw={math.degrees(yaw):.1f}")
    print(f"Quaternion [w,x,y,z]: {q}")
    
    # quaternion → rotation matrix
    R = quaternion_to_rotation_matrix(q)
    print(f"\nRotation Matrix:\n{R}")
    
    # SLERP interpolation
    q_start = euler_to_quaternion(0.0, 0.0, 0.0)
    q_end = euler_to_quaternion(0.0, 0.0, math.radians(90))
    print("\nSLERP from identity to 90 deg yaw:")
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        q_interp = slerp(q_start, q_end, t)
        r, p, y = quaternion_to_euler(q_interp)
        print(f"  t={t:.2f}: yaw={math.degrees(y):.1f} deg, q={q_interp}")


if __name__ == "__main__":
    main()
