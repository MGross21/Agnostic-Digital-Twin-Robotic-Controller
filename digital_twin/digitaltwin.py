import time
import mujoco_toolbox as mjtb
from mujoco_toolbox.controllers import real_time
from agnostic_controller  import UR5
from localpubsub import LocalPubSub
from pathlib import Path

UR5e = UR5()

ur5_root = Path(__file__).parent / "sim/model/static/UniversalRobotics/UR5"
ur5_model = str(ur5_root / "UR5.urdf")
ur5_meshes = str(ur5_root / "meshes" / "collision")

with (
    mjtb.Wrapper(ur5_model, meshdir=ur5_meshes, controller=real_time) as ur5,
    LocalPubSub(port=5_000) as sub
):
    ur5.liveView(show_menu=False)  # Open the simulation window
    ur5.gravity = [0,0,0]  # Disable gravity
    while True:
        sub.subscribe("robot_pos", lambda pos: ur5.controller(ur5.model, ur5.data, {"qpos": pos}))
        time.sleep(0.1)  # Sleep to prevent busy waiting
