import time
import mujoco_toolbox as mjtb
from mujoco_toolbox.controllers import real_time
from agnostic_controller  import UR5
from digital_twin import LocalPubSub

UR5e = UR5()


with (
    mjtb.Wrapper("path/to/xml", controller=real_time) as ur5,
    LocalPubSub(port=5_000) as sub
):
    ur5.liveView(show_menu=False)  # Open the simulation window
    while True:
        sub.subscribe("robot_pos", lambda pos: ur5.controller(ur5.model, ur5.data, {"qpos": pos}))
        time.sleep(0.1)  # Sleep to prevent busy waiting
