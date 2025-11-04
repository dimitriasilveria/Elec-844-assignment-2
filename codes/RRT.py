import numpy as np
import random
from codes.Maps import Map


class Search:
    def __init__(self, start, goal, map_type, epsilon=0.01):
        self.start = start
        self.goal = goal
        self.epsilon = epsilon
        self.map = Map(100, 100)
        if map_type == 1:
            self.map.obstacles_one(30)
        elif map_type == 2:
            self.map.obstacles_two()
        elif map_type == 3:
            self.map.obstacles_three()
        elif map_type == 4:
            self.map.obstacles_four()

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
