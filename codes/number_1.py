from RRT import RRT
import numpy as np
import yaml
import statistics

if __name__ == "__main__":

    #l =  25
    rrt = RRT(start=(25, 50), goal=(75, 50), map_type=1, l=25)
    n_trials = 150
    iterations_dict = {}
    vertices_dict = {}
    sol_length_dict = {}
    for i in range(n_trials):
        path, iterations = rrt.search(seed=i)
        iterations_dict[i] = iterations
        vertices_dict[i] = len(rrt.V)
        sol_length_dict[i] = rrt.path_length
        if path:
            print(f"Trial {i+1}: Goal reached in {iterations} iterations.")
        else:
            print(f"Trial {i+1}: Goal not reached within max iterations.")

    # getting the medians
    median_iterations = statistics.median(iterations_dict.values())
    median_vertices = statistics.median(vertices_dict.values())
    median_sol_length = statistics.median(sol_length_dict.values())

    # Save the results to a YAML file
    with open("iterations.yaml", "w") as f:
        yaml.dump(iterations_dict, f)