import pybullet as p
import pybullet_data
from typing import List, Tuple, Optional

DEFAULT_KP = 1.0
DEFAULT_KI = 0.0
DEFAULT_KD = 0.1
DEFAULT_DT = 1.0 / 240.0

class PyBulletEnv:
    def __init__(self, robot_urdf_path: str, dof: int = 6, use_gui: bool = True, gravity: float = -9.8):
        """
        Initialize the PyBullet environment.

        :param robot_urdf_path: Path to the robot URDF file.
        :param dof: Degrees of freedom of the robot.
        :param use_gui: Whether to use the GUI.
        :param gravity: Gravity value.
        """
        self.client = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setGravity(0, 0, gravity)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        try:
            self.robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
        except p.error as e:
            raise FileNotFoundError(f"Cannot load URDF file: {robot_urdf_path}") from e
        self.dof = dof
        self.integral_error = [0.0] * self.dof
        self.previous_error = [0.0] * self.dof

    def __enter__(self):
        """Enter the runtime context related to this object."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit the runtime context related to this object."""
        self.disconnect()

    def __del__(self):
        """Ensure the PyBullet client is disconnected when the object is deleted."""
        self.disconnect()

    def set_home_position(self, joint_angles: Optional[List[float]] = None) -> None:
        """
        Set the home position of the robot.

        :param joint_angles: List of joint angles.
        """
        if joint_angles is None:
            joint_angles = [0] * self.dof
        for i, angle in enumerate(joint_angles):
            p.resetJointState(self.robot_id, i, targetValue=angle)

    def reset_integral_error(self) -> None:
        """Reset the integral error for PID control."""
        self.integral_error = [0.0] * self.dof

    def apply_pid_control(self, desired_positions: List[float], kp: float = DEFAULT_KP, ki: float = DEFAULT_KI, kd: float = DEFAULT_KD, dt: float = DEFAULT_DT, use_pd: bool = False) -> None:
        """
        Apply PID control to the robot joints.

        :param desired_positions: List of desired joint positions.
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        :param dt: Time step.
        :param use_pd: Whether to use PD control.
        """
        for i in range(self.dof):
            current_position, current_velocity = p.getJointState(self.robot_id, i)[:2]
            position_error = desired_positions[i] - current_position
            if not use_pd:
                self.integral_error[i] += position_error * dt
            derivative_error = (position_error - self.previous_error[i]) / dt
            self.previous_error[i] = position_error
            force = kp * position_error + (ki * self.integral_error[i] if not use_pd else 0) + kd * derivative_error
            p.setJointMotorControl2(self.robot_id, i, p.TORQUE_CONTROL, force=force)

    def apply_control(self, joint_angles: List[float], control_mode: int = p.POSITION_CONTROL) -> None:
        """
        Apply control to the robot joints.

        :param joint_angles: List of joint angles.
        :param control_mode: Control mode (e.g., POSITION_CONTROL, VELOCITY_CONTROL, TORQUE_CONTROL).
        """
        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(self.robot_id, i, control_mode, targetPosition=angle)

    def forward_kinematics(self, joint_angles: Optional[List[float]] = None) -> Tuple[float, float, float]:
        """
        Compute the forward kinematics of the robot.

        :param joint_angles: List of joint angles.
        :return: Position of the end-effector.
        """
        if joint_angles is not None:
            self.set_home_position(joint_angles)
        p.stepSimulation()
        link_state = p.getLinkState(self.robot_id, self.dof - 1)  # Assuming the end-effector is at the last joint
        return link_state[0]

    def inverse_kinematics(self, target_position: List[float], target_orientation: Optional[List[float]] = None) -> List[float]:
        """
        Compute the inverse kinematics of the robot.

        :param target_position: Target position of the end-effector.
        :param target_orientation: Target orientation of the end-effector.
        :return: List of joint angles.
        """
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([0, 0, 0])
        return p.calculateInverseKinematics(self.robot_id, self.dof - 1, target_position, target_orientation)

    def spawn_object(self, object_urdf_path: str, position: List[float], orientation: List[float]) -> int:
        """
        Spawn an object in the simulation.

        :param object_urdf_path: Path to the object URDF file.
        :param position: Position to spawn the object.
        :param orientation: Orientation to spawn the object.
        :return: Object ID.
        """
        return p.loadURDF(object_urdf_path, position, orientation)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        """
        Set the gravity in the simulation.

        :param gravity_vector: Gravity vector.
        """
        p.setGravity(*gravity_vector)

    def step_simulation(self) -> None:
        """Step the simulation."""
        p.stepSimulation()

    def disconnect(self) -> None:
        """Disconnect from the PyBullet client."""
        if p.isConnected():
            p.disconnect()

    def get_robot_joint_states(self) -> List[Tuple[float, float, float, float]]:
        """
        Get the states of the robot joints.

        :return: List of joint states.
        """
        return [p.getJointState(self.robot_id, i) for i in range(self.dof)]

    def reset_simulation(self, joint_angles: Optional[List[float]] = None) -> None:
        """
        Reset the simulation.

        :param joint_angles: List of joint angles.
        """
        p.resetSimulation()
        self.robot_id = p.loadURDF(self.robot_urdf_path, basePosition=[0, 0, 0])
        self.set_home_position(joint_angles)

    def get_link_state(self, link_index: int) -> Tuple:
        """
        Get the state of a link.

        :param link_index: Index of the link.
        :return: Link state.
        """
        return p.getLinkState(self.robot_id, link_index)

    def apply_external_force(self, link_index: int, force: List[float], position: List[float], flags: int = p.WORLD_FRAME) -> None:
        """
        Apply an external force to a link.

        :param link_index: Index of the link.
        :param force: Force vector.
        :param position: Position vector.
        :param flags: Flags for the force application.
        """
        p.applyExternalForce(self.robot_id, link_index, force, position, flags)

def main():
    with PyBulletEnv(robot_urdf_path="../Model/Static/mycobot_pro600/mycobot_pro600.urdf", use_gui=True) as env:
        env.set_home_position([0.0] * env.dof)
        for _ in range(1000):
            env.step_simulation()
        print("Simulation completed.")

if __name__ == "__main__":
    main()