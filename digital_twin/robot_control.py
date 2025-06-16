from armctl import *

with UR5() as robot:
    robot.get_joint_positions()