import time
import mujoco_toolbox as mjtb
from mujoco_toolbox import *
from armctl import *
from communication.localpubsub import LocalPubSub
from pathlib import Path
import math
import numpy as np


model_dir = Path(__file__).resolve().parent / "sim" / "model" / "static"
ur_vention_model = model_dir / "ur5e_vention.xml"
cup_model = str(model_dir / "cup" / "cup.xml")
meshes_dir = model_dir / "UniversalRobotics" / "UR5e" / "meshes"

ur5e_vention = {
    "mjcf": (ur_vention_model.read_text()).format(meshdir=meshes_dir.as_posix()),
    "ur_controller": UR5e(),
    "vention_controller": Vention()
}

IN_TO_M = 0.0254

SIM_HOME = [90, -90, 90, -90, -90, 0]  # UR5 sim home position in degrees

LOCATIONS_DEGREES = [
    [90 - SIM_HOME[0], -90 - SIM_HOME[1], 90 - SIM_HOME[2], -90 - SIM_HOME[3], -90 - SIM_HOME[4], 0 - SIM_HOME[5]],  # Home position
    [84 - SIM_HOME[0], -45 - SIM_HOME[1], 53 - SIM_HOME[2], -100 - SIM_HOME[3], -87 - SIM_HOME[4], -12 - SIM_HOME[5]],  # Over Cup
    [85 - SIM_HOME[0], -35 - SIM_HOME[1], 55 - SIM_HOME[2], -108 - SIM_HOME[3], -87 - SIM_HOME[4], -12 - SIM_HOME[5]],  # In cup
    [106 - SIM_HOME[0], -79 - SIM_HOME[1], 129 - SIM_HOME[2], -140 - SIM_HOME[3], -87 - SIM_HOME[4], 9 - SIM_HOME[5]],  # In, Move cup
    [100 - SIM_HOME[0], -79 - SIM_HOME[1], 129 - SIM_HOME[2], -140 - SIM_HOME[3], -87 - SIM_HOME[4], 9 - SIM_HOME[5]],  # Adjust cup
    [106 - SIM_HOME[0], -90 - SIM_HOME[1], 123 - SIM_HOME[2], -123 - SIM_HOME[3], -87 - SIM_HOME[4], 9 - SIM_HOME[5]],  # Over Cup
    [90 - SIM_HOME[0], -90 - SIM_HOME[1], 90 - SIM_HOME[2], -90 - SIM_HOME[3], -90 - SIM_HOME[4], 0 - SIM_HOME[5]],  # Home position
]

# Convert to radians and generate interpolated positions
def generate_interpolated_positions(locations_degrees, steps=1000):
    locations_radians = np.radians(locations_degrees)
    interpolated = []
    for start, end in zip(locations_radians[:-1], locations_radians[1:]):
        interpolated.extend((1 - t) * start + t * end for t in np.linspace(0, 1, steps))
    return interpolated

INTERPOLATED_LOCATIONS = generate_interpolated_positions(LOCATIONS_DEGREES)


# Glovebox dimensions
GB_WIDTH = 75*IN_TO_M
GB_DEPTH = 60*IN_TO_M
GB_HEIGHT = 40*IN_TO_M

gb = glovebox(width=GB_WIDTH, height=GB_HEIGHT, depth=GB_DEPTH, pos_x=0.5, pos_y=-0.4)
build = Builder(ur5e_vention["mjcf"], cup_model, gb, WORLD_ASSETS)

with (
    mjtb.Simulation(build, controller=real_time, clear_screen=False) as sim,
    # LocalPubSub(port=5_000) as sub
):
    sim._model.qpos0 = [0, 0, math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0]  # Initial joint positions (vention, UR5e)
    sim.launch(show_menu=False)
    sim.gravity = [0,0,0]
    for location in INTERPOLATED_LOCATIONS:
        sim.controller(sim.model, sim.data, {"qpos":[-0.5] + list(location),"qvel":[0]*7})  # Set controller
        time.sleep(0.01)  # Sleep to simulate smoother delay between movements