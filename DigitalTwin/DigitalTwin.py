import time
from .Real.agnostic_controller import AgnosticController as Ctrl
from .Sim.Engine import PyBulletEnv as Env
from .Sim.mujoco_toolbox.mujoco_toolbox import Wrapper

class DigitalTwin:
    def __init__(self, robot:Ctrl=None, pb_simulation:Env=None):
        self.robot = robot
        self.pb_simulation = pb_simulation

    def start_simulation(self, home_position, duration=10000, dt=1./240.):
        if self.pb_simulation:
            self.pb_simulation.set_home_position(home_position)
            try:
                for _ in range(duration):
                    self.pb_simulation.apply_pid_control(home_position, dt=dt, kp=100.0, ki=0.1, kd=10.0, use_pd=False)
                    self.pb_simulation.step_simulation()
                    time.sleep(dt)
            finally:
                self.pb_simulation.disconnect()

    def control_robot(self, joint_positions=None, cartesian_position=None, constraints=None):
        if self.robot:
            self.robot.controller.connect()
            try:
                if constraints:
                    constrained_coords = self.robot.config_space.apply_constraints(constraints)
                    if constrained_coords is not None:
                        joint_positions = self.robot.controller.inverse_kinematics(constrained_coords, [0, 0, 0, 1])
                    else:
                        print("No reachable coordinates within the given constraints.")
                        return

                if joint_positions:
                    self.robot.controller.move_joints(joint_positions)
                if cartesian_position:
                    self.robot.controller.move_cartesian(*cartesian_position)
            finally:
                self.robot.controller.disconnect()

        if self.pb_simulation:
            if joint_positions:
                self.pb_simulation.set_home_position(joint_positions)
            if cartesian_position:
                self.pb_simulation.set_home_position(cartesian_position)

    def control_using_constraints(self, constraints):
        if self.robot:
            constrained_coords = self.robot.config_space.apply_constraints(constraints)
            if constrained_coords is not None:
                joint_positions = self.robot.controller.inverse_kinematics(constrained_coords, [0, 0, 0, 1])
                self.control_robot(joint_positions=joint_positions)
            else:
                print("No reachable coordinates within the given constraints.")
