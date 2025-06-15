import mujoco_toolbox as mjtb
from mujoco_toolbox import glovebox, WORLD_ASSETS
from pathlib import Path
from armctl import Pro600
import numpy as np

mjcf = str(Path(__file__).resolve().parent / "mycobot_pro600.xml")

home_position = np.array(Pro600().HOME_POSITION)

with mjtb.Simulation(mjcf, controller=mjtb.real_time) as sim:
    sim._model.qpos0 = home_position
    sim.gravity = np.array([0, 0, 0])
    sim.launch(show_menu=False)

    while True:
        sim.controller(sim.model, sim.data, {"qpos": home_position})
