import numpy as np
import random
from Maps import Map
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import math

class RRT_star:
    def __init__(self, start, goal, map_type,l=30, epsilon=0.01, step=2.5, goal_tolerance=1.0):
        self.start = start
        self.goal = goal
        self.epsilon = epsilon
        self.step = step
        self.goal_tolerance = goal_tolerance
        self.map_height = 100
        self.map_width = 100
        self.near_radius = 10.0
        self.path_length_500 = 0
        self.path_length_1000 = 0
        self.path_length_2500 = 0
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
    
    def neighborhood(self, q_new):
        k = int(np.ceil(math.e*1.5*np.log(len(self.V))))
        if k < 1:
            k = 1
        elif k > len(self.V):
            k = len(self.V)
        tree = KDTree(self.V)
        _, idxs = tree.query(q_new, k=k)
        if len(idxs.shape) == 0:
            idxs = [idxs]
        neighbors = [self.V[i] for i in idxs]
        return neighbors
    
    def best_parent(self, q_new, neighbors):
        best_cost = np.inf
        best_parent = None
        for neighbor in neighbors:
            cost_q = self.E[neighbor][1] if neighbor in self.E else 0
            dist = np.linalg.norm(np.array(q_new) - np.array(neighbor))
            total_cost = cost_q + dist
            if total_cost < best_cost and self.map.is_valid(neighbor, q_new):
                best_cost = total_cost
                best_parent = neighbor
        return best_parent

    def steer(self, q_near, q_rand):
        direction = np.array(q_rand) - np.array(q_near)
        length = np.linalg.norm(direction)
        if length == 0:
            return q_near
        direction = direction / length  # Normalize
        step_size = min(self.step, length)  # Limit step size to self.step units
        q_new = np.array(q_near) + step_size * direction
        return (q_new[0], q_new[1])

    def cost_to_come(self, q):
        cost = 0
        current = q
        while current != self.start:
            if current in self.E:
                cost += self.E[current][1]
                current = self.E[current][0]
            else:
                break
        return cost
    
    def rewire(self, q_new, neighbors):
        cost_new = self.cost_to_come(q_new)
        for neighbor in neighbors:
            if neighbor == self.start:
                continue  # Don't rewire the start node
            dist_new_neigh = np.linalg.norm(np.array(q_new) - np.array(neighbor))
            tentative_cost = cost_new + dist_new_neigh
            current_cost = self.E[neighbor][1] if neighbor in self.E else 0
            if tentative_cost < current_cost and self.map.is_valid(q_new, neighbor):
                self.E[neighbor] = [q_new, tentative_cost]

    def search(self, seed=None):
        if seed is not None:
            random.seed(seed)

        for i in range(2500):
            q_rand = self.sample()
            q_nearest = self.nearest(q_rand)
            q_new = self.steer(q_nearest, q_rand)
            if i == 499:
                self.path_length_500 = self.cost_to_come(q_nearest) + np.linalg.norm(np.array(q_new) - np.array(q_nearest))
            if i == 999:
                self.path_length_1000 = self.cost_to_come(q_nearest) + np.linalg.norm(np.array(q_new) - np.array(q_nearest))
            if i == 2499:
                self.path_length_2500 = self.cost_to_come(q_nearest) + np.linalg.norm(np.array(q_new) - np.array(q_nearest))
            if self.map.is_valid(q_nearest, q_new):
                neighbors = self.neighborhood(q_new)
                q_best = self.best_parent(q_new, neighbors)
                self.V.append(q_new)
                self.E[q_new] = [q_best,self.cost_to_come(q_best) + np.linalg.norm(np.array(q_new) - np.array(q_best))]
                self.rewire(q_new, neighbors)
            i += 1
            if  q_new == self.goal:
                print("Goal reached!")
                return self.reconstruct_path(q_new), i
            
        print("Goal not reached within max iterations.")
        return None, i
    
    def reconstruct_path(self, q_new):
        path = [self.goal]
        current = q_new
        self.path_length = np.linalg.norm(np.array(self.goal) - np.array(q_new)) + self.cost_to_come(q_new)
        while current != self.start:
            path.append(current)
            if current in self.E:
                current = self.E[current][0]  # Move to the parent node
            else:
                break
        path.append(self.start)
        path.reverse()
        return path
    
    def plot_path(self, path, fig_name="rrt_path.png"):
        fig, ax = plt.subplots()
        ax = self.map.display(ax)
        xs, ys = zip(*self.V)
        ax.scatter(xs, ys, c='blue', s=5)
        if path:
            path_xs, path_ys = zip(*path)
            ax.plot(path_xs, path_ys, c='red', linewidth=2)
        plt.scatter([self.start[0]], [self.start[1]], c='green', s=50, label='Start')
        plt.scatter([self.goal[0]], [self.goal[1]], c='orange', s=50, label='Goal')
        #plotting the entire tree
        for child, (parent, _) in self.E.items():
            plt.plot([child[0], parent[0]], [child[1], parent[1]], c='gray', linewidth=0.5)
        plt.legend()
        plt.savefig(fig_name)
        plt.show()

if __name__ == "__main__":
    rrt = RRT_star(start=(25, 50), goal=(75, 50), map_type=1)
    path, iterations = rrt.search()
    rrt.plot_path(path)

