import numpy as np
from typing import List, Dict
from scipy.spatial.transform import Rotation as R

class Transform:
    @staticmethod
    def H(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """Compute the homogeneous transformation matrix for a single joint"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    @staticmethod
    def FK(params: Dict) -> np.ndarray:
        """Compute the forward kinematics for a robot arm
        If using DH Class, pass in DH.__dict__ as the argument
        
        """
        transform = np.eye(4)
        for param in params:
            transform = transform @ Transform.H(param["a"], param["alpha"], param["d"], param["theta"])
        return transform
    
class Conversion:
    @staticmethod
    def to_radians(degrees: float) -> float:
        return degrees * np.pi / 180
    
    @staticmethod
    def to_degrees(radians: float) -> float:
        return radians * 180 / np.pi
    
    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
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
    
    @staticmethod
    def quat_to_euler(q: List[float]) -> List[float]:
        qw, qx, qy, qz = q
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(2*(qw*qy - qz*qx))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return [roll, pitch, yaw]
    
    @staticmethod
    def rot_to_qut(rot: np.ndarray) -> List[float]:
        return R.from_matrix(rot).as_quat()