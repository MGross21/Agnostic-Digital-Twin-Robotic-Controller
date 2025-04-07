import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.signal import savgol_filter

class PathPlanner:
    def __init__(self, ik_solver, resolution=25, smooth=True):
        self.ik_solver = ik_solver
        self.resolution = resolution
        self.smooth = smooth

    def plan_path(self, start_pose, end_pose):
        positions = np.linspace(start_pose[:3], end_pose[:3], self.resolution)
        orientations = np.linspace(start_pose[3:], end_pose[3:], self.resolution)
        trajectory = []
        guess = np.zeros(self.ik_solver.robot.num_dof)

        for i in range(self.resolution):
            target_pose = np.concatenate([positions[i], orientations[i]])
            joint_angles = self.ik_solver.solve(target_pose, guess)
            guess = joint_angles
            trajectory.append(joint_angles)

        trajectory = np.array(trajectory)
        return self.smooth_trajectory(trajectory) if self.smooth else trajectory

    def smooth_trajectory(self, trajectory):
        smoothed = np.zeros_like(trajectory)
        t = np.linspace(0, self.resolution - 1, self.resolution)

        for j in range(trajectory.shape[1]):
            cs = CubicSpline(t, trajectory[:, j])
            smoothed[:, j] = cs(t)

        # Optional extra smoothing (Savitzky-Golay filter)
        try:
            smoothed = savgol_filter(smoothed, window_length=7, polyorder=3, axis=0)
        except ValueError:
            pass  # fallback if resolution < window_length

        return smoothed

    def plot_3d_trajectory(self, trajectory):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y, z = [], [], []
        for angles in trajectory:
            ee = self.ik_solver.robot.forward_kinematics(angles)
            x.append(ee[0])
            y.append(ee[1])
            z.append(ee[2])
        ax.plot(x, y, z, label="End-Effector Trajectory", lw=2)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("3D End-Effector Trajectory")
        ax.legend()
        plt.show()
