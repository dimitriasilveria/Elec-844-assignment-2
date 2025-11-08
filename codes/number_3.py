from RRT import RRT
import numpy as np
import yaml
import statistics
import os
import matplotlib.pyplot as plt

if __name__ == "__main__":

    #l =  25
   
    n_trials = 100
    l = 25
    iterations_dict = {}
    vertices_dict = {}
    sol_length_dict = {}
    for i in range(n_trials):
        rrt = RRT(start=(25, 50), goal=(75, 50), map_type=1, l=l)
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
    results_dict = {
        'l': str(l),
        'n_trials': str(n_trials),
        'iterations': str(median_iterations),
        'vertices': str(median_vertices),
        'solution_lengths': str(median_sol_length)
    }
    os.makedirs("results_number_1", exist_ok=True)
    os.makedirs("pictures_number_1", exist_ok=True)
    with open(f"results_number_1/results_l_{l}.yaml", "w") as f:
        yaml.dump(results_dict, f)

    rrt.plot_path(path, fig_name=f"pictures_number_1/rrt_path_l_{l}.png")

    #sorting dictionaries to plot histograms
    iterations_sorted = dict(sorted(iterations_dict.items(), key=lambda item: item[1]))
    vertices_sorted = dict(sorted(vertices_dict.items(), key=lambda item: item[1]))
    sol_length_sorted = dict(sorted(sol_length_dict.items(), key=lambda item: item[1]))
    #plotting histograms

    plt.figure()
    plt.hist(iterations_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Iterations to Reach Goal (l={l})')
    plt.xlabel('Iterations')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/iterations_histogram_l_{l}.png")
    
    plt.figure()
    plt.hist(vertices_sorted.values(), bins=20, color='green', alpha=0.7)
    plt.title(f'Histogram of Number of Vertices (l={l})')
    plt.xlabel('Number of Vertices')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/vertices_histogram_l_{l}.png")

    #l = 2 
    l = 4

    iterations_dict = {}
    vertices_dict = {}
    sol_length_dict = {}
    for i in range(n_trials):
        rrt = RRT(start=(25, 50), goal=(75, 50), map_type=1, l=l)
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
    results_dict = {
        'l': str(l),
        'n_trials': str(n_trials),
        'iterations': str(median_iterations),
        'vertices': str(median_vertices),
        'solution_lengths': str(median_sol_length)
    }

    with open(f"results_number_1/results_l_{l}.yaml", "w") as f:
        yaml.dump(results_dict, f)

    rrt.plot_path(path, fig_name=f"pictures_number_1/rrt_path_l_{l}.png")

    #sorting dictionaries to plot histograms
    iterations_sorted = dict(sorted(iterations_dict.items(), key=lambda item: item[1]))
    vertices_sorted = dict(sorted(vertices_dict.items(), key=lambda item: item[1]))
    sol_length_sorted = dict(sorted(sol_length_dict.items(), key=lambda item: item[1]))
    #plotting histograms

    plt.figure()
    plt.hist(iterations_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Iterations to Reach Goal (l={l})')
    plt.xlabel('Iterations')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/iterations_histogram_l_{l}.png")
    
    plt.figure()
    plt.hist(vertices_sorted.values(), bins=20, color='green', alpha=0.7)
    plt.title(f'Histogram of Number of Vertices (l={l})')
    plt.xlabel('Number of Vertices')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/vertices_histogram_l_{l}.png")

    plt.show()