"""Demo: estimate attitude from simulated IMU data using the front-door API."""
import numpy as np

from qsp.orientation import estimate_attitude, integrate_gyro, quaternion_to_euler


def main():
    print("=== Orientation Demo ===\n")

    dt = 0.01
    accel = np.array([0.0, 0.0, 9.81])   # static, gravity pointing down
    gyro = np.array([0.0, 0.0, 0.0])      # no rotation

    # Single-step attitude estimate from accelerometer + gyro
    attitude = estimate_attitude(accel, gyro, dt)
    roll, pitch, yaw = quaternion_to_euler(attitude)
    print(f"Initial attitude estimate: roll={np.degrees(roll):.2f} deg, "
          f"pitch={np.degrees(pitch):.2f} deg, yaw={np.degrees(yaw):.2f} deg")

    # Simulate 100 steps of slow yaw rotation
    print("\nSimulating 100 steps of 1 deg/s yaw rotation:")
    gyro_yaw = np.array([0.0, 0.0, np.radians(1.0)])
    q = attitude.copy()
    for step in range(100):
        q = estimate_attitude(accel, gyro_yaw, dt, q0=q)

    roll, pitch, yaw = quaternion_to_euler(q)
    print(f"After 100 steps: yaw={np.degrees(yaw):.2f} deg (expected ~1.00 deg)")


if __name__ == "__main__":
    main()
