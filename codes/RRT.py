import numpy as np
import random
from Maps import Map
import matplotlib.pyplot as plt


class RRT:
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
        self.V = [self.start]  # List of vertices
        self.E = {}      # Dictionary of edges

    def sample(self):
        p = random.random()
        if p < self.epsilon:
            return self.goal
        else:
            is_free = False
            while not is_free:
                x = random.uniform(0, self.map.width)
                y = random.uniform(0, self.map.height)
                if self.map.is_free((x, y)):
                    is_free = True
                    return (x, y)
                
    def nearest(self, q_rand):
        min_dist = np.inf
        q_near = None
        for v in self.V:
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

        i = 0
        while 1:
            q_rand = self.sample()
            q_nearest = self.nearest(q_rand)
            q_new = self.steer(q_nearest, q_rand)
            if self.map.is_valid(q_nearest, q_new):
                if q_new not in self.V:
                    self.V.append(q_new)
                self.E[q_new] = [q_nearest,np.linalg.norm(np.array(q_new) - np.array(q_nearest))]
            i += 1
            if  q_new == self.goal:
                print("Goal reached!")
                return self.reconstruct_path(q_new), i
            
        print("Goal not reached within max iterations.")
        return None, i
    
    def reconstruct_path(self, q_new):
        path = [self.goal]
        current = q_new
        self.path_length = np.linalg.norm(np.array(self.goal) - np.array(q_new))
        while current != self.start:
            path.append(current)
            if current in self.E:
                self.path_length += self.E[current][1]  # Add edge length to path length
                current = self.E[current][0]  # Move to the parent node
            else:
                break
        path.append(self.start)
        path.reverse()
        return path
    
    def plot_path(self, path, fig_name="rrt_path.pdf"):
        fig, ax = plt.subplots()
        ax = self.map.display(ax)
        xs, ys = zip(*self.V)
        ax.scatter(xs, ys, c='blue', s=5)
        if path:
            path_xs, path_ys = zip(*path)
            ax.plot(path_xs, path_ys, c='red', linewidth=2)
        plt.scatter([self.start[0]], [self.start[1]], c='green', s=50, label='Start')
        plt.scatter([self.goal[0]], [self.goal[1]], c='orange', s=50, label='Goal')
        for child, (parent, _) in self.E.items():
            plt.plot([child[0], parent[0]], [child[1], parent[1]], c='gray', linewidth=0.5)
        plt.legend()
        plt.savefig(fig_name)
        # plt.show()

if __name__ == "__main__":
    rrt = RRT(start=(25, 50), goal=(75, 50), map_type=1)
    path, iterations = rrt.search()
    rrt.plot_path(path)

