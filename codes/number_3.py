from RRT_star import RRT_star
import numpy as np
import yaml
import statistics
import os
import matplotlib.pyplot as plt
from icecream import ic

def get_first_quartile(data):
    #this function assumes sorted data
    q1 = np.percentile(data, 25)
    first_quartile_values = [x for x in data if x <= q1]
    if not first_quartile_values:
        return None
    median_first_quartile = statistics.median(first_quartile_values)
    return median_first_quartile

def get_third_quartile(data):
    #this function assumes sorted data
    q3 = np.percentile(data, 75)
    third_quartile_values = [x for x in data if x >= q3]
    if not third_quartile_values:
        return None
    median_third_quartile = statistics.median(third_quartile_values)
    return median_third_quartile

def find_percentage_trials(n_trials, path_lengths_500, path_lengths_1000, path_lengths_2500):
    #this function assumes sorted data
    count_500 = sum(v == np.inf for v in path_lengths_500)
    count_1000 = sum(v == np.inf for v in path_lengths_1000)
    count_2500 = sum(v == np.inf for v in path_lengths_2500)

    percentage_500 = (count_500 / n_trials) * 100
    percentage_1000 = (count_1000 / n_trials) * 100
    percentage_2500 = (count_2500 / n_trials) * 100

    return percentage_500, percentage_1000, percentage_2500

if __name__ == "__main__":

    #l =  25
   
    n_trials = 1
    l = 25
    iterations = []
    vertices = []
    sol_length = []
    path_lengths_500 = []
    path_lengths_1000 = []
    path_lengths_2500 = []
    has_plots = False
    for i in range(n_trials):
        print(f"Starting trial {i+1}")
        rrt = RRT_star(start=(25, 50), goal=(75, 50), map_type=1, l=l)
        path, iter_count = rrt.search(seed=i)
        if not has_plots and rrt.path_500 is not None:

            rrt.plot_path(rrt.path_500, fig_name=f"{rrt.pictures_dir}/rrt_star_path_iter_500_trial_{i+1}.pdf")
            rrt.plot_path(rrt.path_1000, fig_name=f"{rrt.pictures_dir}/rrt_star_path_iter_1000_trial_{i+1}.pdf")
            rrt.plot_path(rrt.path_2500, fig_name=f"{rrt.pictures_dir}/rrt_star_path_iter_2500_trial_{i+1}.pdf")    
            has_plots = True
        iterations.append(iter_count)
        vertices.append(len(rrt.V))
        sol_length.append(rrt.path_length)
        path_lengths_500.append(rrt.path_length_500)
        path_lengths_1000.append(rrt.path_length_1000)
        path_lengths_2500.append(rrt.path_length_2500)
        if path:
            print(f"Trial {i+1}: Goal reached")
        else:
            print(f"Trial {i+1}: Goal not reached within max iterations.")


    plt.show()
    results_dir = "results_number_3"
    os.makedirs(results_dir, exist_ok=True)


    # rrt.plot_path(path, fig_name=f"{rrt.pictures_dir}/rrt_path.pdf")

    #sorting dictionaries to plot histograms
    iterations.sort()
    vertices.sort()
    sol_length.sort()
    path_lengths_500.sort()
    path_lengths_1000.sort()
    path_lengths_2500.sort()
    #calculating statistics for path lengths

    percentage_500, percentage_1000, percentage_2500 = find_percentage_trials(n_trials, path_lengths_500, path_lengths_1000, path_lengths_2500)
    median = statistics.median(sol_length)
    first_quartile = get_first_quartile(sol_length)
    third_quartile = get_third_quartile(sol_length)
    median_vertices = statistics.median(vertices)

    # Save the results to a YAML file
    results_dict = {
        'l': str(l),
        'n_trials': str(n_trials),
        'vertices': str(median_vertices),
        'solution_lengths': str(median),
        'first_quartile_solution_length': str(first_quartile),
        'third_quartile_solution_length': str(third_quartile),
        'percentage_trials_no_path_500_iterations': str(percentage_500),
        'percentage_trials_no_path_1000_iterations': str(percentage_1000),
        'percentage_trials_no_path_2500_iterations': str(percentage_2500),
    }
    with open(f"{results_dir}/results_l_{l}.yaml", "w") as f:
        yaml.dump(results_dict, f)

    #eliminate infinite values for plotting histograms
    # Separate finite and infinite values
    path_lengths_500_sorted = [x for x in path_lengths_500 if np.isfinite(x)]
    num_infinite_500 = sum(np.isinf(x) for x in path_lengths_500)
    path_lengths_1000_sorted = [v for v in path_lengths_1000 if v != np.inf]
    num_infinite_1000 = sum(v == np.inf for v in path_lengths_1000)
    path_lengths_2500_sorted = [v for v in path_lengths_2500 if v != np.inf]
    num_infinite_2500 = sum(v == np.inf for v in path_lengths_2500)
    #plotting histograms
    plt.figure()
    counts, bins, patches = plt.hist(path_lengths_500_sorted, bins=20, color='blue', alpha=0.7)
    inf_bin_position = bins[-1] + (bins[-1] - bins[-2])
    plt.bar(inf_bin_position, num_infinite_500, width=(bins[-1] - bins[-2]), color='blue', alpha=0.7, edgecolor='black')
    plt.xticks(list(bins) + [inf_bin_position], [f"{b:.2f}" for b in bins] + [r'$\infty$'], rotation=45)
    plt.title(f'Histogram of Path Lengths at 500 Iterations')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"{rrt.pictures_dir}/path_lengths_500_histogram.pdf")

    plt.figure()
    counts, bins, patches = plt.hist(path_lengths_1000_sorted, bins=20, color='blue', alpha=0.7)
    inf_bin_position = bins[-1] + (bins[-1] - bins[-2])
    plt.bar(inf_bin_position, num_infinite_1000, width=(bins[-1] - bins[-2]), color='blue', alpha=0.7, edgecolor='black')
    plt.xticks(list(bins) + [inf_bin_position], [f"{b:.2f}" for b in bins] + [r'$\infty$'], rotation=45)
    plt.title(f'Histogram of Path Lengths at 1000 Iterations')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"{rrt.pictures_dir}/path_lengths_1000_histogram.pdf")

    plt.figure()
    counts, bins, patches = plt.hist(path_lengths_2500_sorted, bins=20, color='blue', alpha=0.7)
    inf_bin_position = bins[-1] + (bins[-1] - bins[-2])
    plt.bar(inf_bin_position, num_infinite_2500, width=(bins[-1] - bins[-2]), color='blue', alpha=0.7, edgecolor='black')
    plt.xticks(list(bins) + [inf_bin_position], [f"{b:.2f}" for b in bins] + [r'$\infty$'], rotation=45)
    plt.title(f'Histogram of Path Lengths at 2500 Iterations')
    plt.xlabel('Path Length')
    plt.ylabel('Frequency')
    plt.savefig(f"{rrt.pictures_dir}/path_lengths_2500_histogram.pdf")

    plt.show()