"""Demo: drift_angle, gyro stability, and orientation health score."""
import math
import numpy as np
from qsp_orientation import (
    euler_to_quaternion,
    drift_angle,
    gyro_stability_metric,
    accel_consistency_metric,
    orientation_health_score,
)


def main():
    print("=== Diagnostics Demo ===\n")
    
    # Drift angle between orientations
    q1 = euler_to_quaternion(0.0, 0.0, 0.0)
    q2 = euler_to_quaternion(0.0, 0.0, math.radians(30))
    angle = drift_angle(q1, q2, degrees=True)
    print(f"Drift between identity and 30 deg yaw: {angle:.2f} degrees")
    
    q3 = euler_to_quaternion(math.radians(45), math.radians(10), math.radians(90))
    angle2 = drift_angle(q1, q3, degrees=True)
    print(f"Drift between identity and 45/10/90 euler: {angle2:.2f} degrees")
    
    # Gyro stability
    rng = np.random.default_rng(42)
    stable_samples = np.ones((100, 3)) * 0.01
    noisy_samples = rng.standard_normal((100, 3)) * 0.5
    print(f"\nGyro stability (stable): {gyro_stability_metric(stable_samples):.6f}")
    print(f"Gyro stability (noisy):  {gyro_stability_metric(noisy_samples):.6f}")
    
    # Accel consistency
    perfect_accel = np.array([0.0, 0.0, 9.81])
    bad_accel = np.array([0.0, 0.0, 5.0])
    print(f"\nAccel consistency (perfect): {accel_consistency_metric(perfect_accel):.4f}")
    print(f"Accel consistency (bad):     {accel_consistency_metric(bad_accel):.4f}")
    
    # Health score from simulated IMU data
    drift = math.radians(5)  # 5 deg drift
    gyro_var = 0.001
    accel_score = 0.99
    health = orientation_health_score(drift, gyro_var, accel_score)
    print(f"\nOrientation health score (good IMU): {health:.4f}")
    
    drift_bad = math.radians(45)
    gyro_var_bad = 0.5
    accel_score_bad = 0.7
    health_bad = orientation_health_score(drift_bad, gyro_var_bad, accel_score_bad)
    print(f"Orientation health score (bad IMU):  {health_bad:.4f}")


if __name__ == "__main__":
    main()
