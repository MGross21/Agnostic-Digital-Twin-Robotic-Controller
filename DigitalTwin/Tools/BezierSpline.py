import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class BezierSpline(object):
    def __init__(self, control_points):
        self.control_points = control_points
        self.dimension = len(control_points[0])

    def de_casteljau(self, points, t):
        """Recursive implementation of De Casteljau's algorithm."""
        if len(points) == 1:
            return points[0]
        new_points = [
            [
                (1 - t) * points[i][d] + t * points[i + 1][d]
                for d in range(self.dimension)
            ]
            for i in range(len(points) - 1)
        ]
        return self.de_casteljau(new_points, t)

    def evaluate(self, t):
        """Evaluate the Bezier spline at parameter t."""
        if not (0 <= t <= 1):
            raise ValueError("Parameter t must be in the range [0, 1]")
        return self.de_casteljau(self.control_points, t)

    def plot(self, num_points=100):
        """Plot the Bezier spline."""
        t_values = np.linspace(0, 1, num_points)
        spline_points = np.array([self.evaluate(t) for t in t_values])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(spline_points[:, 0], spline_points[:, 1], spline_points[:, 2], label='Bezier Spline')
        ax.scatter(*zip(*self.control_points), color='red', label='Control Points')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

# Example usage
if __name__ == "__main__":
    control_points = [
        [0, 0, 0],
        [1, 2, 1],
        [3, 3, 2],
        [4, 0, 3]
    ]
    spline = BezierSpline(control_points)
    t = 0.5
    point = spline.evaluate(t)
    print(f"Point on Bezier spline at t={t}: {point}")
    spline.plot()