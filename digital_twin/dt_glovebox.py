import time
import mujoco_toolbox as mjtb
from mujoco_toolbox import *
from armctl import UR5
from communication import LocalPubSub
from pathlib import Path
import math

UR5e = UR5()

ur5_root = Path(__file__).parent / "sim/model/static/ur5e_vention.xml"
# ur5_root = Path(__file__).parent / "sim/model/static/UniversalRobotics/UR5e/ur5e.xml"
# ur5_root = Path(__file__).parent / "sim/model/static/UniversalRobotics/UR5/"
# ur5_model = str(ur5_root / "UR5.urdf")
# ur5_meshes = str(ur5_root / "meshes" / "collision")


IN_TO_M = 0.0254

# Glovebox dimensions
width = 75 # in
depth = 60 # in
height = 40 # in

gb = glovebox(width=width*IN_TO_M, height=height*IN_TO_M, depth=depth*IN_TO_M, pos_x=0.5, pos_y=-0.4)

with (
    mjtb.Wrapper(str(ur5_root), WORLD_ASSETS, gb, 
                 controller=real_time) as ur5,
    LocalPubSub(port=5_000) as sub
):
    ur5._model.qpos0 = [0, 0, math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0]  # Initial joint positions
    ur5.launch(show_menu=False)  # Open the simulation window
    ur5.gravity = [0,0,0]  # Disable gravity
    while True:
        sub.subscribe("robot_pos", lambda pos: ur5.controller(ur5.model, ur5.data, {"qpos": pos}))
        time.sleep(0.1)  # Sleep to prevent busy waiting
