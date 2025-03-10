import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 📌 Load Camera Trajectory File
def load_trajectory(file_path):
    trajectory = []
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split()
            if len(data) == 8:  # Ensure correct format
                timestamp = float(data[0])  # Timestamp (not used in plot)
                x, y, z = map(float, data[1:4])  # Camera position
                trajectory.append([x, y, z])
    return np.array(trajectory)

# 📌 Fix Coordinate System (If Needed)
def transform_coordinates(trajectory):
    trajectory[:, 2] = trajectory[:, 2]  # Flip Z-axis if visualization is inverted
    return trajectory

# 📌 Plot the Trajectory (3D, X vs. Z, and Y vs. Z)
def plot_trajectory(trajectory):
    fig = plt.figure(figsize=(18, 6))

    # 📌 3D Trajectory Plot
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="Camera Path", color='b', linewidth=2)
    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color="g", marker="o", label="Start")
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color="r", marker="x", label="End")
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Y (meters)")
    ax1.set_zlabel("Z (meters)")
    ax1.set_title("3D Camera Trajectory")
    ax1.legend()

    # 📌 X vs. Z 2D Plot (Horizontal Motion)
    ax2 = fig.add_subplot(132)
    ax2.plot(trajectory[:, 0], trajectory[:, 2], label="X vs. Z", color='r', linewidth=2)
    ax2.scatter(trajectory[0, 0], trajectory[0, 2], color="g", marker="o", label="Start")
    ax2.scatter(trajectory[-1, 0], trajectory[-1, 2], color="b", marker="x", label="End")
    ax2.set_xlabel("X (meters)")
    ax2.set_ylabel("Z (meters)")
    ax2.set_title("X vs. Z Path (Forward Motion)")
    ax2.legend()
    ax2.grid()

    # 📌 Y vs. Z 2D Plot (Height vs. Depth)
    ax3 = fig.add_subplot(133)
    ax3.plot(trajectory[:, 2], -trajectory[:, 1], label="Z vs. Y", color='g', linewidth=2)
    ax3.scatter(trajectory[0, 2], -trajectory[0, 1], color="g", marker="o", label="Start")
    ax3.scatter(trajectory[-1, 2], -trajectory[-1, 1], color="b", marker="x", label="End")
    ax3.set_xlabel("Z (meters)")
    ax3.set_ylabel("Y (meters)")
    ax3.set_title("Z vs. Y Path (Height Change)")
    ax3.legend()
    ax3.grid()

    plt.tight_layout()
    plt.show()

# 📌 Run the Script
file_path = "CameraTrajectory.txt"  # Replace with your actual file path
trajectory = load_trajectory(file_path)
trajectory = transform_coordinates(trajectory)  # Apply transformation
plot_trajectory(trajectory)
