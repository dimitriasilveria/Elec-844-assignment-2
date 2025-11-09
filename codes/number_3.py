from RRT_star import RRT_star
import numpy as np
import yaml
import statistics
import os
import matplotlib.pyplot as plt

if __name__ == "__main__":

    #l =  25
   
    n_trials = 10
    l = 25
    iterations_dict = {}
    vertices_dict = {}
    sol_length_dict = {}
    path_lengths_500 = {}
    path_lengths_1000 = {}
    path_lengths_2500 = {}
    for i in range(n_trials):
        print(f"Starting trial {i+1}")
        rrt = RRT_star(start=(25, 50), goal=(75, 50), map_type=1, l=l)
        path, iterations = rrt.search(seed=i)
        iterations_dict[i] = iterations
        vertices_dict[i] = len(rrt.V)
        sol_length_dict[i] = rrt.path_length
        path_lengths_500[i] = rrt.path_length_500
        path_lengths_1000[i] = rrt.path_length_1000
        path_lengths_2500[i] = rrt.path_length_2500
        if path:
            print(f"Trial {i+1}: Goal reached")
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

    rrt.plot_path(path, fig_name=f"{rrt.pictures_dir}/rrt_path.png")

    #sorting dictionaries to plot histograms
    iterations_sorted = dict(sorted(iterations_dict.items(), key=lambda item: item[1]))
    vertices_sorted = dict(sorted(vertices_dict.items(), key=lambda item: item[1]))
    sol_length_sorted = dict(sorted(sol_length_dict.items(), key=lambda item: item[1]))
    path_lengths_500_sorted = dict(sorted(path_lengths_500.items(), key=lambda item: item[1]))
    path_lengths_1000_sorted = dict(sorted(path_lengths_1000.items(), key=lambda item: item[1]))
    path_lengths_2500_sorted = dict(sorted(path_lengths_2500.items(), key=lambda item: item[1]))
    #plotting histograms

    plt.figure()
    plt.hist(path_lengths_500_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Path Lengths at 500 Iterations (l={l})')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/path_lengths_500_histogram_l_{l}.png")

    plt.figure()
    plt.hist(path_lengths_1000_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Path Lengths at 1000 Iterations (l={l})')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/path_lengths_1000_histogram_l_{l}.png")

    plt.figure()
    plt.hist(path_lengths_2500_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Path Lengths at 2500 Iterations (l={l})')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"pictures_number_1/path_lengths_2500_histogram_l_{l}.png")

    plt.show()