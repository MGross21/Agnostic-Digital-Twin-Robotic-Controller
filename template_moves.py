from agnostic_controller import Vention, UR5
import math
import time



def sweep_robot():
    ur5_sweep_vantage = [math.radians(angle) for angle in [90, -85, 60, -65, -90, 0]]

    with Vention() as vention, UR5() as ur5:
        ur5.home()
        vention.home()

        time.sleep(3)

        ur5.move_joints(ur5_sweep_vantage)
        vention.move_joints(800)


if __name__ == "__main__":
    sweep_robot()