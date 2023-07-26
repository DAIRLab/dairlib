# When seeing "_tkinter.TclError: no display name and no $DISPLAY environment variable",
# uncomment the following two lines code (or just restart computer because it has something to do with ssh)
# import matplotlib
# matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import time
import sys
import traceback
import logging
import argparse

# I copied this function from examples/goldilocks_models/py_utils.py, because I had a path issue when running this on cluster
def FindVarValueInString(file_string, string_to_search):
    # We search from the end of the file
    word_location = file_string.rfind(string_to_search)
    number_idx_start = 0
    number_idx_end = 0
    idx = word_location
    while True:
        if file_string[idx] == '=':
            number_idx_start = idx
        elif file_string[idx] == '\n':
            number_idx_end = idx
            break
        idx += 1
    value = float(file_string[number_idx_start + 1: number_idx_end])
    return value



plt.rcParams.update({'font.size': 15.5})

save_figure = True

plot_cost_breakdown = True
only_plot_average_cost = True
normalize_by_nominal_cost = True
# only_add_successful_samples_to_average_cost = False


### Some plots settings
target_stride_length = None #0.3
target_stride_length_tol = 0.01


### argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--iter_start", help="", default=1, type=int)
parser.add_argument("--iter_end", help="", default=-1, type=int)
parser.add_argument("--robot_option", help="0 is five-link robot. 1 is cassie_fixed_spring", default=1, type=int, choices=[0, 1])
parser.add_argument("--path", help="", default="", type=str)
parser.add_argument('--exclude_failed_samples', action='store_true')
parser.add_argument('--no-exclude_failed_samples', dest='exclude_failed_samples', action='store_false')
parser.set_defaults(exclude_failed_samples=False)
args = parser.parse_args()

iter_start = args.iter_start
iter_end = args.iter_end
is_iter_end = (args.iter_end > 1)
robot_option = args.robot_option
only_add_successful_samples_to_average_cost = args.exclude_failed_samples

# Checks
if len(args.path) > 0:
    if args.path[-1] == "/":
        args.path = args.path[:-1]

### Some path settings
base = '../dairlib_data/goldilocks_models/find_models'

directory_list = []
## Automatically add folders in `base` directory if on cluster
if True:
    all_subfolders = [os.path.join(base, o) for o in os.listdir(base) if os.path.isdir(os.path.join(base, o))]
    for subfolder in all_subfolders:
        if 'explore_task_boundary' in subfolder:
            continue  # skip the folder that's for exploring boundary of task space
        data_folder = '%s/robot_%d/' % (subfolder, robot_option)
        if os.path.exists(data_folder):
            directory_list.append(data_folder)
## Manually add more folders here (note that the path needs to end with "/")
# directory_list.append('%s/robot_%d/' % (base if len(args.path) == 0 else args.path, robot_option))
# directory_list.append('/home/yuming/workspace/dairlib_data/goldilocks_models/planning/robot_1/models_20211229_3dlipm_fix_xy_big_w_vel_and_grad_main_cost_and_big_range/robot_1/')
# directory_list.append('/home/yuming/workspace/dairlib_data/goldilocks_models/planning/robot_1/20220417_rom27_big_range_bigger_step_size_6e-3_torque_weight_dominate/robot_1/')
## Sort and print
directory_list.sort()
[print(d) for d in directory_list]

### cost setting
# file_name = 'c.csv'
# file_name = 'c_without_tau.csv'
file_name = 'c.csv'
file_name_nominal_cost = 'c_main.csv'

# folder_name_nominal_cost = ""
# folder_name_nominal_cost = "nominal_no_constraint_traj/"
folder_name_nominal_cost = "nominal_traj_cubic_swing_foot/"


### visualization setting
ave_cost_prop = "k-" if only_plot_average_cost else "k--"
ave_cost_label = "Averaged cost (excluding failed samples) " if only_add_successful_samples_to_average_cost else "Averaged cost "
# ave_cost_label = ""

for dir_list_idx in range(len(directory_list)):
    directory = directory_list[dir_list_idx]
    print("%d/%d " % (dir_list_idx, len(directory_list)), end='')
    # Checks for the director_list
    # if len(directory_list) > 1 and not save_figure:
    #     raise ValueError("save_figure has to be true. Otherwise this script won't go through the list")
    if not os.path.exists(directory):
        print(directory, " doesn't exist, so we skip to the next one")
        continue

    # Extract unique_folder_name
    unique_folder_name = directory.split("/")[-3]
    if unique_folder_name == "robot_1" or unique_folder_name == "find_models":
        print("Warning: didn't extract unique_folder_name correctly")
    print("(%s)" % unique_folder_name)

    # Use try-catch, so that the script doesn't stop to continue to the next ROM optimization folder
    try:
        # Set N_sample
        # n_sampel_sl = 13  # should be > 0
        # n_sampel_gi = 1  # should be > 0
        # n_sampel_v = 1  # should be > 0
        # n_sampel_tr = 1  # should be > 0
        # n_sampel_ph = 3  # should be > 0
        # N_sample = n_sampel_sl * n_sampel_gi * n_sampel_v * n_sampel_tr * n_sampel_ph
        # print('n_sampel_sl = ' + str(n_sampel_sl))
        # print('n_sampel_gi = ' + str(n_sampel_gi))
        # print('n_sampel_v = ' + str(n_sampel_v))
        # print('n_sampel_tr = ' + str(n_sampel_tr))
        # print('n_sampel_ph = ' + str(n_sampel_ph))
        N_sample = int(np.loadtxt(directory + "n_sample.csv"))

        # get nomial cost
        nominal_cost = 0.0
        if normalize_by_nominal_cost:
            for sample_i in range(N_sample):
                cost = []
                assert os.path.isfile(directory + folder_name_nominal_cost + '0_'+str(sample_i)+'_'+file_name_nominal_cost), 'file does not exist'
                cost.append(np.genfromtxt(directory + folder_name_nominal_cost + '0_'+str(sample_i)+'_'+file_name_nominal_cost, delimiter=","))

                nominal_cost += cost[0] / N_sample
        else:
            nominal_cost = 1.0
        # print('nominal_cost = '+str(nominal_cost))

        # plot
        while 1:
            # fig = plt.figure(1)
            fig_size = (6.4, 4.8)  if only_plot_average_cost else (12.8, 9.6)
            fig = plt.figure(num=1, figsize=fig_size)
            ax = fig.gca()

            # Get the iteration length first (in case the lengths are different among samples). This is for plotting the average cost
            iteration_length = 0
            for sample_i in range(N_sample):
                iteration_length_of_this_sample = 0
                iteration = iter_start
                while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name):
                    # For some reason, sometimes the cost file exists but empty. (it looks to happen only in the last iteration, so it's probably the training was stopped unexpected, or reached the max hard drive capacity)
                    # For now, whenever I see an empty cost file, I set it as the final iteration. You can comment the following 3 lines off, if you don't want to do this.
                    file_content = open(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, "r").read()
                    if len(file_content) == 0:
                        break

                    iteration_length_of_this_sample += 1
                    if is_iter_end & (iteration == iter_end):
                        break
                    iteration += 1
                iteration_length = iteration_length_of_this_sample if (iteration_length == 0) else min(iteration_length, iteration_length_of_this_sample)
            if iteration_length == 0:
                print("There is at least one sample in Iteration %d that's not evaluated. Stop plotting." % iter_start)
                break
            t = list(range(iter_start, iteration_length+iter_start))

            cost_iter_1 = 0.0
            cost_min = 10000.0  # just a random big value
            # Initialize sum_cost and n_successful_sample_each_iter
            sum_cost_all = [0] * iteration_length
            sum_cost_x = [0] * iteration_length
            sum_cost_u = [0] * iteration_length
            sum_cost_accel = [0] * iteration_length
            sum_cost_main = [0] * iteration_length
            n_successful_sample_each_iter = [0] * iteration_length
            best_improvement_per_sample = [0] * N_sample

            # 1. Plot each sample
            for sample_i in range(N_sample):
                # read in cost
                cost_all = [0.0] * iteration_length
                cost_x = [0.0] * iteration_length
                cost_u = [0.0] * iteration_length
                cost_accel = [0.0] * iteration_length
                cost_main = [0.0] * iteration_length
                iteration = iter_start
                for idx in range(iteration_length):
                    # Read all costs
                    cost_all[idx] = np.genfromtxt(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, delimiter=",") / nominal_cost

                    # Read sub-costs
                    with open(directory+str(iteration)+'_'+str(sample_i)+'_trajopt_settings_and_cost_breakdown.txt', 'rt') as f:
                        contents = f.read()
                        cost_x[idx] = FindVarValueInString(contents, "cost_x =") / nominal_cost
                        cost_u[idx] = FindVarValueInString(contents, "cost_u =") / nominal_cost
                        cost_accel[idx] = FindVarValueInString(contents, "cost_joint_acceleration =") / nominal_cost
                        cost_main[idx] = cost_x[idx] + cost_u[idx] + cost_accel[idx]

                    if is_iter_end & (iteration == iter_end):
                        break
                    iteration += 1

                # plot cost for each sample
                if not only_plot_average_cost:
                    sl = np.loadtxt(directory+str(iter_start)+'_'+str(sample_i)+'_task.csv')[0]
                    task_criteria_satisfied = True if (target_stride_length is None) else (abs(sl-target_stride_length) < target_stride_length_tol)
                    if task_criteria_satisfied:
                        ax.plot(t, cost_main)
                        # ax.plot(t,cost_main, label='sample_idx = '+str(sample_i))
                        # TODO: not very important, but you can read the task value and use it as a label

                # Read in is_success
                is_success = [1] * iteration_length
                if only_add_successful_samples_to_average_cost:
                    idx = 0
                    for iter_i in range(iter_start, iter_start + iteration_length):
                        is_success[idx] = int(np.genfromtxt(directory+str(iter_i)+'_'+str(sample_i)+'_is_success.csv', delimiter=","))
                        idx += 1

                # For some reason, sometimes the cost file exists but empty. (it looks to happen only in the last iteration, so it's probably the training was stopped unexpected, or reached the max hard drive capacity)
                # I only need this code if I didn't set the final iteration to not include empty cost file
                for idx in range(iteration_length):
                    if not (cost_all[idx] >= 0):
                        is_success[idx] = 0
                        cost_all[idx] = 0

                # Add to accumulated success
                n_successful_sample_each_iter = [x + y for x, y in zip(n_successful_sample_each_iter, is_success)]

                # Add to total cost
                filtered_cost_all = [x * y for x, y in zip(is_success, cost_all[0:iteration_length])]
                sum_cost_all = [x + y for x, y in zip(sum_cost_all, filtered_cost_all[0:iteration_length])]
                filtered_cost_x = [x * y for x, y in zip(is_success, cost_x[0:iteration_length])]
                sum_cost_x = [x + y for x, y in zip(sum_cost_x, filtered_cost_x[0:iteration_length])]
                filtered_cost_u = [x * y for x, y in zip(is_success, cost_u[0:iteration_length])]
                sum_cost_u = [x + y for x, y in zip(sum_cost_u, filtered_cost_u[0:iteration_length])]
                filtered_cost_accel = [x * y for x, y in zip(is_success, cost_accel[0:iteration_length])]
                sum_cost_accel = [x + y for x, y in zip(sum_cost_accel, filtered_cost_accel[0:iteration_length])]
                filtered_cost_main = [x * y for x, y in zip(is_success, cost_main[0:iteration_length])]
                # import pdb;pdb.set_trace()
                sum_cost_main = [x + y for x, y in zip(sum_cost_main, filtered_cost_main[0:iteration_length])]

                # Others - Update best_improvement_per_sample
                best_improvement_per_sample[sample_i] = round((cost_main[0] - np.min(cost_main)) / cost_main[0], 2)
                # best_improvement_per_sample[sample_i] = round((cost_main[0] - cost_main[-1]) / cost_main[0], 2)
            # Reshape `best_improvement_per_sample`
            task_grid_dim = np.loadtxt(directory + 'n_samples.csv', delimiter=',').astype(int)
            task_ranges = np.loadtxt(directory + 'task_ranges.csv', delimiter=',')
            task_names = np.loadtxt(directory + 'task_names.csv', dtype=str, delimiter=',')
            code_version_before_two_task_planes = len(task_grid_dim.shape) == 1
            if code_version_before_two_task_planes:
                task_grid_dim = [task_grid_dim]
            non_degenerate_dim_indices = [[i for i in range(len(task_grid_dim_per_plane)) if task_grid_dim_per_plane[i] != 1] for task_grid_dim_per_plane in task_grid_dim]
            task_grid_dim = [np.max(task_grid_dim, 0)[indices__per_plane] for indices__per_plane in non_degenerate_dim_indices]
            task_ranges = [task_ranges[indices__per_plane] for indices__per_plane in non_degenerate_dim_indices]
            task_names = [task_names[indices__per_plane] for indices__per_plane in non_degenerate_dim_indices]

            best_improvement_per_sample__per_plane = []
            current_idx = 0
            for task_grid_dim__per_plane in task_grid_dim:
                N_sample__per_plane = np.prod(task_grid_dim__per_plane)
                best_improvement_per_sample__per_plane.append(np.array(best_improvement_per_sample[current_idx:current_idx+N_sample__per_plane]).reshape(task_grid_dim__per_plane))
                current_idx += N_sample__per_plane
            best_improvement_per_sample = best_improvement_per_sample__per_plane

            # 2. Plot average cost
            # Average cost
            average_cost_all = [x / y for x, y in zip(sum_cost_all, n_successful_sample_each_iter)]
            average_cost_main = [x / y for x, y in zip(sum_cost_main, n_successful_sample_each_iter)]

            # # Clean-up -- Remove the big spikes (which I believe is from bad solves)
            # delta_cost_threshold = 0.05
            # list_of_idx_to_be_remove = []
            # for delta_iter in [1, 2]:
            #     for idx in range(delta_iter, iteration_length):
            #         if average_cost_main[idx] - average_cost_main[idx-delta_iter] > delta_cost_threshold:
            #             list_of_idx_to_be_remove.append(idx)
            # list_of_idx_to_be_remove = list(set(list_of_idx_to_be_remove))
            # list_of_idx_to_be_remove.sort()
            # list_of_idx_to_be_remove.reverse()
            # for idx in list_of_idx_to_be_remove:
            #     t.pop(idx)
            #     average_cost_main.pop(idx)

            ax.plot(t, average_cost_all, ave_cost_prop, linewidth=3.0, label=ave_cost_label + " (all)")
            ax.plot(t, average_cost_main, ave_cost_prop, linewidth=3.0, label=ave_cost_label + " (main)")
            # ax.plot(t, average_cost_main, linewidth=3.0)

            # Get the cost of first iter and min cost
            cost_iter_1 = max(cost_iter_1, average_cost_all[0])
            cost_min = min(cost_min, min(average_cost_main))

            # Write jobs into file
            f = open("%s../costs_info%s.txt" % (directory, "_excluding_failed_samples" if only_add_successful_samples_to_average_cost else ""), "w")
            f.write("For %s\n" % unique_folder_name)
            f.write("  file_name_cost = %s\n" % "cost_main")
            f.write("  folder_name_nominal_cost = %s\n" % folder_name_nominal_cost)
            f.write("  nominal_cost = %.3f\n" % nominal_cost)
            f.write("  (iter 1 normalized cost, min normalized cost, improvement) = (%.3f, %.3f, %.1f%%)\n" % (average_cost_main[0], min(average_cost_main), 100 * (average_cost_main[0] - min(average_cost_main)) / average_cost_main[0]))
            print("  (nominal_cost, iter 1 normalized cost, min normalized cost, improvement) = (%.3f, %.3f, %.3f, %.1f%%)\n" % (nominal_cost, average_cost_main[0], min(average_cost_main), 100 * (average_cost_main[0] - min(average_cost_main)) / average_cost_main[0]), end='')
            for j in range(len(best_improvement_per_sample)):
                if len(best_improvement_per_sample) > 1:
                    f.write("  Task plane %d" % (j+1) + "\n")
                    print("  Task plane %d" % (j+1))
                f.write("    tasks with non-zero range = " + str([task_names[j][i] + ' ' + str(task_ranges[j][i]) for i in range(len(task_names[j]))]) + "\n")
                print("    tasks with non-zero range = " + str([task_names[j][i] + ' ' + str(task_ranges[j][i]) for i in range(len(task_names[j]))]))
                f.write("    best_improvement_per_sample = \n" + str(best_improvement_per_sample[j]) + "\n")
                print("    best_improvement_per_sample = \n" + str(best_improvement_per_sample[j]))
            f.close()

            # labels
            plt.xlabel('Iteration')
            if only_plot_average_cost & only_add_successful_samples_to_average_cost:
                plt.ylabel('Averaged (successful) task cost')
            else:
                plt.ylabel('Averaged task cost')
            if not only_plot_average_cost:
                plt.title('Cost over iterations')

            if only_plot_average_cost:
                plt.legend()

            # Change the ticks
            # ax.set_yticks(np.arange(1.05,1.301,0.05))

            # Set limit
            # ax.set_ylim(1, 2.5)
            ax.set_ylim(cost_min * 0.95, cost_iter_1 * 1.05)

            plt.grid()
            plt.title("Traj opt cost over model iteration")
            plt.draw()

            # so that the label is not cut off by the window
            # plt.tight_layout()
            plt.gcf().subplots_adjust(bottom=0.15)
            plt.gcf().subplots_adjust(left=0.15)

            if save_figure:
                affix = "_new" if os.path.exists(directory + "../cost.png") else ""
                plt.savefig("%s../cost%s%s.png" % (directory, "_excluding_failed_samples" if only_add_successful_samples_to_average_cost else "", affix))
                plt.savefig("../cost%s_%s.png" % ("_excluding_failed_samples" if only_add_successful_samples_to_average_cost else "", unique_folder_name))
                print(";  figure saved")
                # print("  figure saved for %s" % unique_folder_name)
                plt.close()

            ### Get a second plot with cost breakdown
            if plot_cost_breakdown:
                fig2 = plt.figure(num=2, figsize=(6.4, 4.8))
                ax = fig2.gca()
                average_cost_all = [x / y for x, y in zip(sum_cost_all, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost_all, linewidth=3.0, label=ave_cost_label + " (all)")
                average_cost_u = [x / y for x, y in zip(sum_cost_u, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost_u, linewidth=3.0, label=ave_cost_label + "$u$")
                average_cost_x = [x / y for x, y in zip(sum_cost_x, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost_x, linewidth=3.0, label=ave_cost_label + "$v$")
                average_cost_accel = [x / y for x, y in zip(sum_cost_accel, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost_accel, linewidth=3.0, label=ave_cost_label + "$\dot{v}$")
                average_cost_main = [x / y for x, y in zip(sum_cost_main, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost_main, linewidth=3.0, label=ave_cost_label + "main", color="k")

                # print("first iter, print(average_cost_all)", average_cost_all[1])
                # print("first iter, print(average_cost_u)", average_cost_u[1])
                # print("first iter, print(average_cost_x)", average_cost_x[1])
                # print("first iter, print(average_cost_accel)", average_cost_accel[1])
                # print("first iter, print(average_cost_main)", average_cost_main[1])
                # print("300 iter, print(average_cost_all)", average_cost_all[299])
                # print("300 iter, print(average_cost_u)", average_cost_u[299])
                # print("300 iter, print(average_cost_x)", average_cost_x[299])
                # print("300 iter, print(average_cost_accel)", average_cost_accel[299])
                # print("300 iter, print(average_cost_main)", average_cost_main[299])

                plt.legend(loc='upper right')
                plt.xlabel('Iteration')
                if only_plot_average_cost & only_add_successful_samples_to_average_cost:
                    plt.ylabel('Averaged (successful) task cost')
                else:
                    plt.ylabel('Averaged task cost')
                lb = min(min(average_cost_all), min(average_cost_x), min(average_cost_u), min(average_cost_accel), min(average_cost_main))
                ax.set_ylim(lb * 0.95, cost_iter_1 * 1.05)
                plt.grid()
                plt.title("Traj opt cost over model iteration")
                plt.draw()
                plt.gcf().subplots_adjust(bottom=0.15)
                plt.gcf().subplots_adjust(left=0.15)

                if save_figure:
                    affix = "_new" if os.path.exists(directory + "../subcost.png") else ""
                    plt.savefig("%s../subcost%s.png" % (directory, affix))
                    plt.savefig("../subcost_%s.png" % unique_folder_name)
                    # print(";  figure saved")
                    # print("  figure saved for %s" % unique_folder_name)
                    plt.close()

            ### Wait to replot the same folder or break to move on to the next folder
            if save_figure:
                break
            else:
                # plt.pause(60)
                # plt.clf()
                # plt.show()
                break

    except Exception as e:
        logging.error(traceback.format_exc())  # Logs the error appropriately.

if not save_figure:
    plt.legend(["Ex#1: 2nd order; 2D task", "Ex#2: 4th order; 2D task", "Ex#3: 2nd order; 4D task"])
    plt.show()

