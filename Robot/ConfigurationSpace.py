import numpy as np
from Utils import Transforms

class ConfigurationSpace:
    """
    A class to handle the configuration space of a robot.

    Example usage:
    --------------
    config_space = ConfigurationSpace("path/to/config.json", show_graph=True)
    reachable_coords = config_space.compute_reachable_coordinates()
    config_space.plot_surface_mesh()
    """
    def __init__(self, urdf_generator):
        self.config = urdf_generator.json_data
        self.transforms = Transforms(self.config['robot']['dh_parameters'])

    def compute_reachable_coordinates(self):
        T = self.transforms.compute_forward_kinematics()
        return T[:3, 3]

    def apply_constraints(self, constraints):
        reachable_coords = self.compute_reachable_coordinates()
        for constraint in constraints:
            if not self.is_within_constraint(reachable_coords, constraint):
                return None
        return reachable_coords

    def is_within_constraint(self, coords, constraint):
        x, y, z = coords
        x_min, x_max, y_min, y_max, z_min, z_max = constraint
        return (x_min <= x <= x_max) and (y_min <= y <= y_max) and (z_min <= z <= z_max)