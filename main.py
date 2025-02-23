import time
import json
from DigitalTwin import DigitalTwin as DT
from DigitalTwin.Sim.Model.Generation import URDFGenerator
from DigitalTwin.Sim.Engine import PyBulletEnv as Env


robotArchetype = "UR5"
robot_loc = f"RobotConfigs/{robotArchetype}/{robotArchetype}"

config_file = f"{robot_loc}.json"
robot_urdf_loc = f"{robot_loc}.urdf"

#################################
# Load Robot Configuration from JSON
#################################

# Create the robot object
robot = URDFGenerator(robotArchetype)

# Load robot configuration from JSON data
robot.load_from_json(config_file)

# Save the URDF file
robot.save_urdf(robot_urdf_loc)

#################################
# Create Configuration Space
#################################

# config_space = ConfigurationSpace(robot)

#################################
# Create PyBullet Simulation
#################################

pb_simulation = Env("RobotConfigs/UR5/UR5.urdf", dof=6)

# Start PyBullet Simulation
home_position = [0, -1.57, 1.57, 0, 1.57, 0]
pb_simulation.set_home_position(home_position)
try:
    for _ in range(10000):
        pb_simulation.step_simulation()
        time.sleep(1./240.)
finally:
    pb_simulation.disconnect()

#################################
# Create Digital Twin
#################################

# controller = AgnosticController(robot_data["ip_address"], robot_data["port"])
# robot_instance = Robot(config_space, controller=controller)
digital_twin = DT(pb_simulation=pb_simulation)

# Define Glovebox Constraints using default dimensions and safety margin
# glovebox_constraints = Glovebox.get_constraints()

# Control the Robot using Configuration Space Constraints
# digital_twin.control_robot(constraints=glovebox_constraints)

# Optionally, control the Robot using joint positions or Cartesian coordinates
# joint_positions = [0, -1.57, 0, -1.57, 0, 0]
# cartesian_position = [0.5, 0.0, 0.5, 0, 0, 0]
# digital_twin.control_robot(joint_positions=joint_positions, cartesian_position=cartesian_position)