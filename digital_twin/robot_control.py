from armctl import *
import time

with UR5() as robot:
    robot.home()
    time.sleep(5)
    