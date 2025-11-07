from RRT import RRT
from RRT_connect import RRT_connect
import numpy as np
import yaml
import statistics
import os
import matplotlib.pyplot as plt

def run_rrt_experiments(l, n_trials, result_folder, picture_folder, search_algorithm, map_type):
    iterations_dict = {}
    vertices_dict = {}
    sol_length_dict = {}
    for i in range(n_trials):
        if search_algorithm == 'RRT':
            rrt = RRT(start=(25, 50), goal=(75, 50), map_type=map_type)
        elif search_algorithm == 'RRT_connect':
            rrt = RRT_connect(start=(25, 50), goal=(75, 50), map_type=map_type)
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
    os.makedirs(result_folder, exist_ok=True)
    os.makedirs(picture_folder, exist_ok=True)
    with open(f"{result_folder}/results_{search_algorithm}_{map_type}.yaml", "w") as f:
        yaml.dump(results_dict, f)

    rrt.plot_path(path, fig_name=f"{picture_folder}/path_{search_algorithm}_{map_type}.png")

    #sorting dictionaries to plot histograms
    iterations_sorted = dict(sorted(iterations_dict.items(), key=lambda item: item[1]))
    vertices_sorted = dict(sorted(vertices_dict.items(), key=lambda item: item[1]))
    sol_length_sorted = dict(sorted(sol_length_dict.items(), key=lambda item: item[1]))
    #plotting histograms

    plt.figure()
    plt.hist(iterations_sorted.values(), bins=20, color='blue', alpha=0.7)
    plt.title(f'Histogram of Iterations to Reach Goal (search_algorithm={search_algorithm})')
    plt.xlabel('Iterations')
    plt.ylabel('Frequency')
    plt.savefig(f"{picture_folder}/iterations_histogram_{search_algorithm}_{map_type}.png")

if __name__ == "__main__":

    #l =  25
   
    n_trials = 10
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT', map_type=2)
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT_connect', map_type=2)
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT', map_type=3)
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT_connect', map_type=3)
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT', map_type=4)
    run_rrt_experiments(l=25, n_trials=n_trials, result_folder="results_number_2", picture_folder="pictures_number_2", search_algorithm='RRT_connect', map_type=4)

    plt.show()