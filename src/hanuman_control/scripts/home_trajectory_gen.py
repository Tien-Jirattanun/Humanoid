import numpy as np
import os

# Parameters
target = np.array([0., 0., 1.00870663, -1.9668205, 0.95811387, 0.])  # 6 joints
duration = 5.0
dt = 0.2
num_points = int(duration / dt) + 1
t = np.linspace(0, duration, num_points)

# Generate trajectory (cubic polynomial) for 6 joints
q6 = np.zeros((num_points, 6))
for i in range(6):
    a0 = 0.0  # Initial position (0)
    a1 = 0.0  # Initial velocity (0)
    a2 = (3 * target[i]) / (duration**2)  # Cubic coefficient
    a3 = (-2 * target[i]) / (duration**3)  # Cubic coefficient
    q6[:, i] = a0 + a1 * t + a2 * t**2 + a3 * t**3  # Position trajectory

# Pad to 12 joints (remaining joints stay at zero)
q12 = np.zeros((num_points, 12))
q12[:, :6] = q6  # First 6 joints get the trajectory, rest are zeros

# Save with key 'q' to match trajectory_publisher.py
try:
    save_path = "q_home_data.npz"
    np.savez(save_path, q=q12)
    print(f"Saved home trajectory to {save_path} with shape {q12.shape}")
    if os.path.exists(save_path):
        print("File verification: SUCCESS")
    else:
        print("File verification: FAILED (check permissions)")
except Exception as e:
    print(f"Save failed: {e}")