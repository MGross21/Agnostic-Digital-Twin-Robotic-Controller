import numpy as np
from scipy.optimize import minimize
from digital_twin.tools import Transformations as tf

class IKSolver:
    def __init__(self, robot, max_iter=1000, epsilon=1e-6):
        self.robot = robot
        self.max_iter = max_iter
        self.epsilon = epsilon

    def error_function(self, joint_angles, target_pose):
        current = self.robot.forward_kinematics(joint_angles)
        return np.linalg.norm(current[:3] - target_pose[:3])

    def solve(self, target_pose, initial_guess=None):
        if initial_guess is None:
            initial_guess = np.zeros(self.robot.num_dof)

        bounds = list(zip(self.robot.joint_limits["lower"], self.robot.joint_limits["upper"]))

        result = minimize(self.error_function, initial_guess,
                          args=(target_pose,),
                          method='SLSQP',
                          bounds=bounds,
                          options={'disp': False, 'maxiter': self.max_iter})

        if result.success:
            return result.x
        else:
            raise RuntimeError("IK solver failed to converge")
