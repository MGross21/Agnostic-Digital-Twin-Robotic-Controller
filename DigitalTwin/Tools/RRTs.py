import numpy as np
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0
        self.parent = None

class RRTStar:
    def __init__(self, start, goal, obstacle_list, x_range, y_range, expand_dist=2.0, path_resolution=0.5, max_iter=500):
        self.start = Node(start[0], start[0])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.x_range = x_range
        self.y_range = y_range
        self.expand_dist = expand_dist
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.node_list = [self.start]

    def planning(self):
         for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dist)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node, self.node_list)
                new_node = self.choose_parent(new_node, near_inds, nearest_node, self.obstacle_list)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds, self.obstacle_list)
                    
                    if self.is_near_goal(new_node):
                        return self.generate_final_course(self.node_list.index(new_node))
         return None
    
    def steer(self, from_node, to_node, expand_dist):
        new_node = Node(to_node.x, to_node.y)
        dist, theta = self.calc_distance_and_angle(from_node, to_node)
        
        new_node.x = from_node.x + expand_dist * math.cos(theta)
        new_node.y = from_node.y + expand_dist * math.sin(theta)
        
        new_node.cost = from_node.cost + expand_dist
        new_node.parent = from_node
        return new_node

    def choose_parent(self, new_node, near_inds, nearest_node, obstacle_list):
        if not near_inds:
            return None

        costs = [self.calc_new_cost(self.node_list[i], new_node) for i in near_inds]
        min_cost_ind = near_inds[costs.index(min(costs))]
        best_near_node = self.node_list[min_cost_ind]

        if self.check_collision_extend(new_node, best_near_node, obstacle_list):
             new_node.cost = best_near_node.cost + self.calc_distance(best_near_node, new_node)
             new_node.parent = best_near_node
             return new_node
        
        new_node.cost = nearest_node.cost + self.calc_distance(nearest_node, new_node)
        new_node.parent = nearest_node
        return new_node

    def rewire(self, new_node, near_inds, obstacle_list):
        for i in near_inds:
            near_node = self.node_list[i]
            if near_node == new_node.parent:
                continue
            
            new_cost = new_node.cost + self.calc_distance(near_node, new_node)
            if new_cost < near_node.cost:
                if self.check_collision_extend(near_node, new_node, obstacle_list):
                    near_node.parent = new_node
                    near_node.cost = new_cost
    
    def is_near_goal(self, node):
         dist_to_goal = self.calc_distance(node, self.goal)
         if dist_to_goal <= self.expand_dist:
            return True
         return False

    def generate_final_course(self, goal_index):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path[::-1]

    def check_collision_extend(self, node_a, node_b, obstacle_list):
        num_points = int(math.ceil(self.calc_distance(node_a, node_b) / self.path_resolution))
        
        for i in range(num_points + 1):
            interpolated_x = node_a.x + (node_b.x - node_a.x) * i / num_points
            interpolated_y = node_a.y + (node_b.y - node_a.y) * i / num_points
            
            if not self.check_collision(Node(interpolated_x, interpolated_y), obstacle_list):
                return False
        return True
    
    @staticmethod
    def get_random_node():
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        return Node(x, y)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        return dlist.index(min(dlist))

    @staticmethod
    def check_collision(node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = node.x - ox
            dy = node.y - oy
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False
        return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        theta = math.atan2(dy, dx)
        return dist, theta
    
    @staticmethod
    def calc_distance(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.sqrt(dx ** 2 + dy ** 2)

    @staticmethod
    def calc_new_cost(from_node, to_node):
        return from_node.cost + RRTStar.calc_distance(from_node, to_node)

    def find_near_nodes(self, new_node, node_list):
        nnode = len(node_list) + 1
        r = 5 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in node_list]
        near_inds = [ind for ind in range(len(node_list)) if dlist[ind] <= r**2]
        return near_inds
    


if __name__ == '__main__':
    start = (0, 0)
    goal = (10, 10)
    obstacles = [(2, 2, 1), (7, 8, 1), (5, 5, 1)]  # x, y, radius
    x_range = [0, 11]
    y_range = [0, 11]

    rrt_star = RRTStar(start, goal, obstacles, x_range, y_range)
    path = rrt_star.planning()

    print("Path:", path)