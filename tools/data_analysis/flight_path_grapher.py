import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1) Read in the data
df = pd.read_csv("FlightData/flight_data_20250328_173437.csv")

# 2) Convert quaternion (w, x, y, z) to rotation matrix.
#    Quaternions must be normalized, so we do that first.
def rotation_matrix_from_quaternion(w, x, y, z):
    """
    Convert a quaternion into a 3x3 rotation matrix.
    We assume the quaternion is normalized (w^2 + x^2 + y^2 + z^2 = 1).
    """
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1 - 2*(yy + zz),   2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),       1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),       2*(yz + wx),       1 - 2*(xx + yy)]
    ])

# Normalize the quaternion columns
norm = np.sqrt(df["quaternion_w"]**2 + df["quaternion_x"]**2 +
               df["quaternion_y"]**2 + df["quaternion_z"]**2)

df["quaternion_w"] /= norm
df["quaternion_x"] /= norm
df["quaternion_y"] /= norm
df["quaternion_z"] /= norm

# 3) Integrate acceleration to get rough position.
#    +Z is down, +X is forward, +Y is right.

velocity = np.zeros(3)   # m/s
position = np.zeros(3)   # m
positions = [position.copy()]

timestamps = df["timestamp"].values
for i in range(1, len(df)):
    dt = timestamps[i] - timestamps[i-1]
    if dt <= 0:
        # Fallback if timestamps are not strictly increasing
        dt = 0.01

    # Current quaternion
    w = df["quaternion_w"].iloc[i]
    x = df["quaternion_x"].iloc[i]
    y = df["quaternion_y"].iloc[i]
    z = df["quaternion_z"].iloc[i]

    # Rotation from body to inertial
    R = rotation_matrix_from_quaternion(w, x, y, z)

    # Body-frame acceleration in g. Convert to m/s^2
    a_body_g = np.array([df["accel_x"].iloc[i],
                         df["accel_y"].iloc[i],
                         df["accel_z"].iloc[i]])
    a_body_mss = a_body_g * 9.81

    # Since +Z is down, a stationary sensor sees ~+9.81 in Z.
    # Subtract that gravity component in the body frame:
    a_body_no_gravity = a_body_mss - np.array([0, 0, 9.81])

    # Rotate to inertial frame
    a_inertial = R @ a_body_no_gravity

    # Integrate to get velocity and position
    velocity += a_inertial * dt
    position += velocity * dt

    positions.append(position.copy())

positions = np.array(positions)

# 4) Plot the 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# If you want a "Z up" view, you could use -positions[:, 2] below.
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2])
ax.set_xlabel("X (m)")  # forward
ax.set_ylabel("Y (m)")  # right
ax.set_zlabel("Z (m)")  # down
ax.set_title("Estimated 3D Flight Path (+Z down)")

plt.show()
