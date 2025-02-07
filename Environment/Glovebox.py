from typing import List, Tuple

class Glovebox:
    DEFAULT_DIMENSIONS: Tuple[float, float, float, float, float, float] = (-0.5, 0.5, -0.2, 0.2, 0.0, 0.5)
    DEFAULT_SAFETY_MARGIN: float = 0.05

    @staticmethod
    def get_constraints(dimensions: Tuple[float, float, float, float, float, float] = None, safety_margin: float = None) -> List[float]:
        """
        Get the constraints for the glovebox environment with a safety margin.

        Parameters:
        -----------
        dimensions : tuple, optional
            A tuple containing the dimensions of the glovebox (x_min, x_max, y_min, y_max, z_min, z_max).
            If not provided, DEFAULT_DIMENSIONS will be used.
        safety_margin : float, optional
            The safety margin to apply to the dimensions.
            If not provided, DEFAULT_SAFETY_MARGIN will be used.

        Returns:
        --------
        list
            A list containing the constrained dimensions with the safety margin applied.
        """
        if dimensions is None:
            dimensions = Glovebox.DEFAULT_DIMENSIONS
        if safety_margin is None:
            safety_margin = Glovebox.DEFAULT_SAFETY_MARGIN

        x_min, x_max, y_min, y_max, z_min, z_max = dimensions
        return [
            x_min + safety_margin, x_max - safety_margin,
            y_min + safety_margin, y_max - safety_margin,
            z_min + safety_margin, z_max - safety_margin
        ]
