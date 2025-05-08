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