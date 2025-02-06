import pybullet as p
import pybullet_data
from typing import List, Tuple, Optional

class Simulation:
    def __init__(self, robot_urdf_path: str, dof: int = 6, use_gui: bool = True, gravity: float = -9.8):
        self.client = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setGravity(0, 0, gravity)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
        self.dof = dof
        self.integral_error = [0.0] * self.dof
        self.previous_error = [0.0] * self.dof

    def set_home_position(self, joint_angles: Optional[List[float]] = None) -> None:
        if joint_angles is None:
            joint_angles = [0] * self.dof
        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=angle)

    def reset_integral_error(self) -> None:
        self.integral_error = [0.0] * self.dof

    def apply_pid_control(self, desired_positions: List[float], kp: float = 1.0, ki: float = 0.0, kd: float = 0.1, dt: float = 1./240., use_pd: bool = False) -> None:
        for i in range(self.dof):
            current_position = p.getJointState(self.robot_id, i)[0]
            current_velocity = p.getJointState(self.robot_id, i)[1]
            position_error = desired_positions[i] - current_position
            if not use_pd:
                self.integral_error[i] += position_error * dt
            derivative_error = (position_error - self.previous_error[i]) / dt
            self.previous_error[i] = position_error
            force = kp * position_error + (ki * self.integral_error[i] if not use_pd else 0) + kd * derivative_error
            p.setJointMotorControl2(self.robot_id, i, p.TORQUE_CONTROL, force=force)

    def forward_kinematics(self, joint_angles: Optional[List[float]] = None) -> Tuple[float, float, float]:
        if joint_angles is not None:
            self.set_home_position(joint_angles)
        p.stepSimulation()
        link_state = p.getLinkState(self.robot_id, self.dof - 1)  # Assuming the end-effector is at the last joint
        return link_state[0]

    def inverse_kinematics(self, target_position: List[float], target_orientation: List[float]) -> List[float]:
        return p.calculateInverseKinematics(self.robot_id, self.dof - 1, target_position, target_orientation)

    def spawn_object(self, object_urdf_path: str, position: List[float], orientation: List[float]) -> int:
        return p.loadURDF(object_urdf_path, position, orientation)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        p.setGravity(*gravity_vector)

    def step_simulation(self) -> None:
        p.stepSimulation()

    def disconnect(self) -> None:
        p.disconnect()

    def get_robot_joint_states(self) -> List[Tuple[float, float, float, float]]:
        return [p.getJointState(self.robot_id, i) for i in range(self.dof)]

    def reset_simulation(self, joint_angles: Optional[List[float]] = None) -> None:
        p.resetSimulation()
        self.robot_id = p.loadURDF(self.robot_urdf_path, basePosition=[0, 0, 0])
        self.set_home_position(joint_angles)

    def get_link_state(self, link_index: int) -> Tuple:
        return p.getLinkState(self.robot_id, link_index)

    def apply_external_force(self, link_index: int, force: List[float], position: List[float], flags: int = p.WORLD_FRAME) -> None:
        p.applyExternalForce(self.robot_id, link_index, force, position, flags)