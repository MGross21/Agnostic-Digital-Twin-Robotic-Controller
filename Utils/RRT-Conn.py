# Python code for RRT-Connect path planning in joint space for a 6-DOF robotic arm
import numpy as np
import random
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ..Environment.Glovebox import Glovebox
from ..Utils.Math import Transform

class Node:
    def __init__(self, q):
        self.q = q  # Joint angles
        self.parent = None

def random_config(joint_limits):
    return np.array([random.uniform(l[0], l[1]) for l in joint_limits])

def distance(q1, q2):
    return np.linalg.norm(q1 - q2)

def nearest(tree, q):
    distances = [distance(node.q, q) for node in tree]
    return tree[np.argmin(distances)]

def steer(q_from, q_to, step_size):
    direction = q_to - q_from
    length = np.linalg.norm(direction)
    direction = direction / length
    return q_from + step_size * direction

def is_collision_free(q, obstacles, glovebox_constraints):
    # Check if the configuration is within the glovebox constraints
    x, y, z = forward_kinematics(q)
    x_min, x_max, y_min, y_max, z_min, z_max = glovebox_constraints
    if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
        return False
    # Implement additional collision checks with obstacles
    for obstacle in obstacles:
        if check_collision_with_obstacle(q, obstacle):
            return False
    return True

def check_collision_with_obstacle(q, obstacle):
    # Implement the collision check with a specific obstacle
    # This function should return True if there is a collision, False otherwise
    return False

def forward_kinematics(q):
    # Use the Transform class from Math.py to compute the forward kinematics
    dh_params = [
        {"a": 0, "alpha": 0, "d": 0, "theta": q[0]},
        {"a": 0, "alpha": 0, "d": 0, "theta": q[1]},
        {"a": 0, "alpha": 0, "d": 0, "theta": q[2]},
        {"a": 0, "alpha": 0, "d": 0, "theta": q[3]},
        {"a": 0, "alpha": 0, "d": 0, "theta": q[4]},
        {"a": 0, "alpha": 0, "d": 0, "theta": q[5]},
    ]
    transform = Transform.FK(dh_params)
    return transform[0, 3], transform[1, 3], transform[2, 3]

def rrt_connect(start, goal, joint_limits, obstacles, glovebox_constraints, max_iter=1000, step_size=0.1):
    tree_a, tree_b = [Node(np.array(start))], [Node(np.array(goal))]
    for _ in range(max_iter):
        rand_q = random_config(joint_limits)
        nearest_node = nearest(tree_a, rand_q)
        new_q = steer(nearest_node.q, rand_q, step_size)
        if is_collision_free(new_q, obstacles, glovebox_constraints):
            new_node = Node(new_q)
            new_node.parent = nearest_node
            tree_a.append(new_node)
            if connect_trees(tree_b, new_q, step_size, obstacles, glovebox_constraints):
                return extract_path(tree_a[-1], tree_b[-1])
        tree_a, tree_b = tree_b, tree_a
    return None

def connect_trees(tree, q, step_size, obstacles, glovebox_constraints):
    nearest_node = nearest(tree, q)
    while True:
        new_q = steer(nearest_node.q, q, step_size)
        if not is_collision_free(new_q, obstacles, glovebox_constraints):
            return False
        new_node = Node(new_q)
        new_node.parent = nearest_node
        tree.append(new_node)
        if np.allclose(new_q, q):
            return True
        nearest_node = new_node

def extract_path(node_a, node_b):
    path_a, path_b = [], []
    while node_a:
        path_a.append(node_a.q)
        node_a = node_a.parent
    while node_b:
        path_b.append(node_b.q)
        node_b = node_b.parent
    return path_a[::-1] + path_b

def plot_path(path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_data, y_data, z_data = [], [], []
    for q in path:
        x, y, z = forward_kinematics(q)
        x_data.append(x)
        y_data.append(y)
        z_data.append(z)
    ax.plot(x_data, y_data, z_data, marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Example usage
joint_limits = [(-np.pi, np.pi)] * 6
start = np.zeros(6)
goal = np.ones(6)
obstacles = []
glovebox_constraints = Glovebox.get_constraints()
path = rrt_connect(start, goal, joint_limits, obstacles, glovebox_constraints)
if path:
    print("Path found:", path)
    plot_path(path)
else:
    print("No path found.")
