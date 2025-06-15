from armctl import *

with Pro600("192.158.1.159",5_001) as robot:
    robot.home()