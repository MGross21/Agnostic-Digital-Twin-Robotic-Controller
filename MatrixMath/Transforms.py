import numpy as np
from typing import List, Dict

class Transforms:
    def __init__(self, dh_params: List[Dict[str, float]]):
        self.dh_params: List[Dict[str, float]] = dh_params

    def dh_to_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_forward_kinematics(self) -> np.ndarray:
        T = np.eye(4)
        for param in self.dh_params:
            T = np.dot(T, self.dh_to_transform(param['theta'], param['d'], param['a'], param['alpha']))
        return T

    def euler_to_quat(self, roll: float, pitch: float, yaw: float) -> List[float]:
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qw, qx, qy, qz]

    def quat_to_euler(self, q: List[float]) -> List[float]:
        qw, qx, qy, qz = q
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(2*(qw*qy - qz*qx))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return [roll, pitch, yaw]
