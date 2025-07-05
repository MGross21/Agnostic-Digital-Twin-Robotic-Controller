import time
import mujoco
import mujoco_toolbox as mjtb
from mujoco_toolbox import *
from armctl import *
from pathlib import Path
import math
import numpy as np
import json


model_dir = Path(__file__).resolve().parent / "sim" / "model" / "static"
ur_vention_model = model_dir / "ur5e_vention.xml"
cup_model = str(model_dir / "cup" / "cup.xml")
meshes_dir = model_dir / "UniversalRobotics" / "UR5e" / "meshes"

ur5e_vention = ur_vention_model.read_text().format(meshdir=meshes_dir.as_posix())

# Constants
INTERPOLATION_STEPS = 1000
SLEEP_DELAY = 0.01
IN_TO_M = 0.0254
SH = [90, -90, 90, -90, -90, 0]  # UR5 sim home position in degrees (SH=Sim Home)
STATIC_CUP_Z = 0.05  # Static cup height in meters

# Glovebox dimensions
GB_WIDTH = 75*IN_TO_M
GB_DEPTH = 60*IN_TO_M
GB_HEIGHT = 40*IN_TO_M
GB = glovebox(width=GB_WIDTH, height=GB_HEIGHT, depth=GB_DEPTH, pos_x=0.5, pos_y=-0.4)

LOCATIONS_DEGREES = [
    [90 - SH[0], -90 - SH[1], 90 - SH[2], -90 - SH[3], -90 - SH[4], 0 - SH[5]],  # Home position
    [84 - SH[0], -45 - SH[1], 53 - SH[2], -100 - SH[3], -87 - SH[4], -12 - SH[5]],  # Over Cup
    [85 - SH[0], -35 - SH[1], 55 - SH[2], -108 - SH[3], -87 - SH[4], -12 - SH[5]],  # In cup
    [106 - SH[0], -79 - SH[1], 129 - SH[2], -140 - SH[3], -87 - SH[4], 9 - SH[5]],  # In, Move cup
    [100 - SH[0], -79 - SH[1], 129 - SH[2], -140 - SH[3], -87 - SH[4], 9 - SH[5]],  # Adjust cup
    [106 - SH[0], -90 - SH[1], 123 - SH[2], -123 - SH[3], -87 - SH[4], 9 - SH[5]],  # Over Cup
    [90 - SH[0], -90 - SH[1], 90 - SH[2], -90 - SH[3], -90 - SH[4], 0 - SH[5]],  # Home position
]

# Helper function for generating interpolated positions
def generate_interpolated_positions(locations_degrees, steps=INTERPOLATION_STEPS):
    """Generate interpolated positions between waypoints."""
    locations_radians = np.radians(locations_degrees)
    for start, end in zip(locations_radians[:-1], locations_radians[1:]):
        yield from ((1 - t) * start + t * end for t in np.linspace(0, 1, steps))

b_id = lambda model, name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

cup_tracker = []

# Main simulation logic
BUILD = Builder(ur5e_vention, cup_model, GB, WORLD_ASSETS)
try:
    with (
        mjtb.Simulation(BUILD, controller=real_time, clear_screen=False) as sim,
    ):
        sim._model.qpos0 = [0, 0, math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0]  # Initial joint positions
        sim.launch(show_menu=False)
        sim.gravity = [0, 0, 0]

        for move_id, location in enumerate(generate_interpolated_positions(LOCATIONS_DEGREES)):
            phase_idx = ((move_id) // INTERPOLATION_STEPS) + 1  # Calculate phase index based on move_id
            sim.controller(sim.model, sim.data, {"qpos": [-0.5] + list(location), "qvel": [0] * 7})

            if phase_idx in {1}:  # Home position
                pos = [0.5508301313487787, -0.79833108438147, STATIC_CUP_Z]
            elif phase_idx in {2, 3, 4, 5}:
                wrist_pos = sim.data.xpos[b_id(sim.model, 'wrist_3_link')]
                pos = [wrist_pos[0], wrist_pos[1], STATIC_CUP_Z]
            elif phase_idx in {6}:  # Adjust the cup position slightly
                pos = [0.7469567224132392, -0.37525749955280036, STATIC_CUP_Z]
            else:
                pos = [0, 0, 1]

            sim.model.body_pos[b_id(sim.model, 'cup')] = np.array(pos)

            cup_tracker.append(pos.copy())       

            time.sleep(SLEEP_DELAY)

finally:
    with open("cup_tracker.json", "w") as f:
        json.dump(cup_tracker, f)