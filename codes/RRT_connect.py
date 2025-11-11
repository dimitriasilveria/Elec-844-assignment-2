import numpy as np
import random
from Maps import Map
import matplotlib.pyplot as plt


class RRT_connect:
    def __init__(self, start, goal, map_type,l=30, epsilon=0.01, step=2.5, goal_tolerance=1.0):
        self.start = start
        self.goal = goal
        self.epsilon = epsilon
        self.step = step
        self.goal_tolerance = goal_tolerance
        self.map_height = 100
        self.map_width = 100
        self.map = Map(self.map_width, self.map_height,step)
        if map_type == 1:
            self.map.obstacles_one(l)
        elif map_type == 2:
            self.map.obstacles_two()
        elif map_type == 3:
            self.map.obstacles_three()
        elif map_type == 4:
            self.map.obstacles_four()

        self.path_length = 0
        self.V_a = [self.start]  # List of vertices
        self.E_a = {}      # Dictionary of edges
        self.V_b = [self.goal]  # List of vertices
        self.E_b = {}      # Dictionary of edges
        self.V = None
        self.E = None
        self.is_start = True  # To alternate between trees
        self.i = 0


    def connect(self, q_new, V, E):
        q_near_other = self.nearest(q_new, V)
        q_last = q_near_other
        is_connected = False
        while True:
            q_new_other = self.steer(q_near_other, q_new)
            self.i += 1
            if self.map.is_valid(q_near_other,q_new_other):
                q_last = q_near_other
                V.append(q_new_other)
                E[q_new_other] = [q_near_other,np.linalg.norm(np.array(q_new_other) - np.array(q_near_other))]
                if np.linalg.norm(np.array(q_new_other) - np.array(q_new)) <= self.goal_tolerance:
                    # E[q_near_other] = [q_new,np.linalg.norm(np.array(q_near_other) - np.array(q_new))]
                    is_connected = True
                    break
                q_near_other = q_new_other

            else:
                break
        return V, E, is_connected, q_last
    
    def swap_trees(self, V_a, E_a, V_b, E_b):
        if self.is_start:
            self.is_start = False
            self.V_a, self.E_a, self.V_b, self.E_b = V_a, E_a, V_b, E_b
            return self.V_b, self.E_b, self.V_a, self.E_a
        else:
            self.is_start = True
            self.V_b, self.E_b, self.V_a, self.E_a = V_a, E_a, V_b, E_b
            return self.V_a, self.E_a, self.V_b, self.E_b

    def sample(self):
        # p = random.random()
        # if p < self.epsilon:
        #     return self.goal
        # else:
        is_free = False
        while not is_free:
            x = random.uniform(0, self.map.width)
            y = random.uniform(0, self.map.height)
            if self.map.is_free((x, y)):
                is_free = True
                return (x, y)
                
    def nearest(self, q_rand, V):
        min_dist = np.inf
        q_near = None
        for v in V:
            dist = np.linalg.norm(np.array(q_rand) - np.array(v))
            if dist < min_dist:
                min_dist = dist
                q_near = v
        return q_near
    
    def steer(self, q_near, q_rand):
        direction = np.array(q_rand) - np.array(q_near)
        length = np.linalg.norm(direction)
        if length == 0:
            return q_near
        direction = direction / length  # Normalize
        step_size = min(self.step, length)  # Limit step size to self.step units
        q_new = np.array(q_near) + step_size * direction
        return (q_new[0], q_new[1])

    def search(self, seed=None):
        if seed is not None:
            random.seed(seed)
        V_a = self.V_a
        E_a = self.E_a
        V_b = self.V_b
        E_b = self.E_b
        while True:
            q_rand = self.sample()
            q_nearest = self.nearest(q_rand, V_a)
            q_new = self.steer(q_nearest, q_rand)
            self.i += 1
            if self.map.is_valid(q_nearest, q_new):
                V_a.append(q_new)
                E_a[q_new] = [q_nearest,np.linalg.norm(np.array(q_new) - np.array(q_nearest))]
                V_b, E_b, is_connected, q_new_other = self.connect(q_new, V_b, E_b)
            if is_connected:
                self.V = V_a + V_b
                print("Goal reached!")
                if self.is_start:
                    self.V_a, self.E_a, self.V_b, self.E_b = V_a, E_a, V_b, E_b
                else:
                    self.V_b, self.E_b, self.V_a, self.E_a = V_a, E_a, V_b, E_b
                return self.reconstruct_path(q_new, q_new_other), self.i
            
            V_a, E_a, V_b, E_b = self.swap_trees(V_a, E_a, V_b, E_b)

    def reconstruct_path(self, q_new, q_new_other):
        path_start = []
        path_goal = []
        if self.is_start:
            current = q_new
            while current != self.start:
                path_start.append(current)
                if current in self.E_a:
                    self.path_length += self.E_a[current][1]  # Add edge length to path length
                    current = self.E_a[current][0]  # Move to the parent node
                else:
                    break
            path_start.append(self.start)
            path_start.reverse()
            current = q_new_other
            while current != self.goal:
                path_goal.append(current)
                if current in self.E_b:
                    self.path_length += self.E_b[current][1]  # Add edge length to path length
                    current = self.E_b[current][0]  # Move to the parent node
                else:                    
                    break
            path_goal.append(self.goal)
        else:
            current = q_new_other
            while current != self.start:
                path_start.append(current)
                if current in self.E_a:
                    self.path_length += self.E_a[current][1]  # Add edge length to path length
                    current = self.E_a[current][0]  # Move to the parent node
                else:
                    break
            path_start.append(self.start)
            path_start.reverse()
            current = q_new
            while current != self.goal:
                path_goal.append(current)
                if current in self.E_b:
                    self.path_length += self.E_b[current][1]  # Add edge length to path length
                    current = self.E_b[current][0]  # Move to the parent node
                else:
                    
                    break
        path_goal.append(self.goal)
        return path_start + path_goal

    def plot_path(self, path, fig_name="rrt_path.pdf"):
        if path is None:
            print("No path to plot.")
            return
        fig, ax = plt.subplots()
        ax = self.map.display(ax)
        xs, ys = zip(*self.V)
        ax.scatter(xs, ys, c='blue', s=5)
        if path:
            path_xs, path_ys = zip(*path)
            ax.plot(path_xs, path_ys, c='red', linewidth=2)
        plt.scatter([self.start[0]], [self.start[1]], c='green', s=50, label='Start')
        plt.scatter([self.goal[0]], [self.goal[1]], c='orange', s=50, label='Goal')
        for child, (parent, _) in self.E_a.items():
            plt.plot([child[0], parent[0]], [child[1], parent[1]], c='gray', linewidth=0.5)
        for child, (parent, _) in self.E_b.items():
            plt.plot([child[0], parent[0]], [child[1], parent[1]], c='gray', linewidth=0.5)
        plt.legend()
        plt.savefig(fig_name)
        # plt.show()

if __name__ == "__main__":
    rrt = RRT_connect(start=(25, 50), goal=(75, 50), map_type=1)
    path, iterations = rrt.search()
    rrt.plot_path(path)

