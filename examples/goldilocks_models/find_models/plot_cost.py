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
plt.rcParams.update({'font.size': 18})

save_figure = True

only_plot_average_cost = True
normalize_by_nominal_cost = True
only_add_successful_samples_to_average_cost = False

iter_start = 1
iter_end = 11
is_iter_end = 0
robot_option = 1  # 0 is five-link robot. 1 is cassie_fixed_spring
# if len(sys.argv) >= 2:
#     iter_start = int(sys.argv[1])
# if len(sys.argv) >= 3:
#     iter_end = int(sys.argv[2])
#     is_iter_end = 1
# if len(sys.argv) >= 4:
#     robot_option = int(sys.argv[3])

### Some path settings
base = '../dairlib_data/goldilocks_models/find_models'

directory_list = []
## Automatically add folders in `base` directory if on cluster
if "/scratch/yminchen" in os.getcwd():
    all_subfolders = [os.path.join(base, o) for o in os.listdir(base) if os.path.isdir(os.path.join(base, o))]
    for subfolder in all_subfolders:
        data_folder = '%s/robot_%d/' % (subfolder, robot_option)
        if os.path.exists(data_folder):
            directory_list.append(data_folder)
else:
    directory_list.append('%s/robot_%d/' % (base, robot_option))
## Manually add more folders here (note that the path needs to end with "/")
# directory_list.append('/')
## Print
[print(d) for d in directory_list]

### cost setting
# file_name = 'c.csv'
# file_name = 'c_without_tau.csv'
file_name1 = 'c_main.csv'
file_name2 = 'c.csv'
file_name_nominal_cost = file_name1
file_name_list = [file_name1, file_name2]

# folder_name_nominal_cost = ""
# folder_name_nominal_cost = "nominal_no_constraint_traj/"
folder_name_nominal_cost = "nominal_traj_cubic_swing_foot/"

### visualization setting
ave_cost_prop = ""
if only_plot_average_cost:
    ave_cost_prop = "k-"
else:
    ave_cost_prop = "k--"
ave_cost_label = ""
if only_add_successful_samples_to_average_cost:
    ave_cost_label = "Averaged cost (excluding failed samples)"
else:
    ave_cost_label = "Averaged cost"


for i in range(len(directory_list)):
    directory = directory_list[i]
    print("%d/%d " % (i, len(directory_list)), end='')
    # Checks for the director_list
    if len(directory_list) > 1 and not save_figure:
        raise ValueError("save_figure has to be true. Otherwise this script won't go through the list")
    if not os.path.exists(directory):
        print(directory, " doesn't exist, so we skip to the next one")
        continue

    # Extract unique_folder_name
    unique_folder_name = directory.split("/")[-3]
    if unique_folder_name == "robot_1" or unique_folder_name == "find_models":
        print("Warning: didn't extract unique_folder_name correctly")
    print("(%s)" % unique_folder_name)

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

    # Use try-catch, so that the script doesn't stop to continue to the next ROM optimization folder
    try:

        # get nomial cost
        nominal_cost = 0.0
        if normalize_by_nominal_cost:
            for sample_i in range(N_sample):
                cost = []
                assert os.path.isfile(directory + folder_name_nominal_cost + '0_'+str(sample_i)+'_'+file_name_nominal_cost), 'file does not exist'
                matrix = np.genfromtxt(directory + folder_name_nominal_cost + '0_'+str(sample_i)+'_'+file_name_nominal_cost, delimiter=",")
                cost.append(matrix)

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
                cost = []
                iteration = iter_start
                while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name2):
                    matrix = np.genfromtxt(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name2, delimiter=",") / nominal_cost
                    cost.append(matrix)
                    if is_iter_end & (iteration == iter_end):
                        break
                    iteration += 1
                iteration_length = len(cost) if (iteration_length == 0) else min(iteration_length, len(cost))
            if iteration_length == 0:
                print("There is at least one sample in Iteration %d that's not evaluated. Stop plotting." % iter_start)
                break

            for file_name in file_name_list:
                # Initialize total_cost and n_successful_sample_each_iter
                total_cost = [0] * iteration_length
                n_successful_sample_each_iter = [0] * iteration_length

                # 1. Plot each sample
                for sample_i in range(N_sample):
                    # read in cost
                    cost = []
                    iteration = iter_start
                    while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name):
                        matrix = np.genfromtxt(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, delimiter=",") / nominal_cost
                        cost.append(matrix)
                        if is_iter_end & (iteration == iter_end):
                            break
                        iteration += 1

                    # plot cost for each sample
                    length = len(cost)
                    t = range(iter_start, length+iter_start)
                    if not only_plot_average_cost:
                        if file_name == 'c_main.csv':
                            ax.plot(t, cost)
                            # ax.plot(t,cost, label='sample_idx = '+str(sample_i))
                            # TODO: not very important, but you can read the task value and use it as a label

                    # Read in is_success
                    is_success = []
                    if only_add_successful_samples_to_average_cost:
                        for iter_i in range(iter_start, iter_start + iteration_length):
                            matrix = np.genfromtxt(directory+str(iter_i)+'_'+str(sample_i)+'_is_success.csv', delimiter=",")
                            is_success.append(matrix)
                    else:
                        is_success = [1] * iteration_length

                    # Add to accumulated success
                    n_successful_sample_each_iter = [x + y for x, y in zip(n_successful_sample_each_iter, is_success)]

                    # Add to total cost
                    filtered_cost = [x * y for x, y in zip(is_success, cost[0:iteration_length])]
                    total_cost = [x + y for x, y in zip(total_cost, filtered_cost[0:iteration_length])]

                # 2. Plot average cost
                average_cost = [x / y for x, y in zip(total_cost, n_successful_sample_each_iter)]
                ax.plot(t[0:iteration_length], average_cost, ave_cost_prop, linewidth=3.0, label=ave_cost_label + "; " + file_name)

                # Write jobs into file
                if file_name == "c_main.csv":
                    f = open(directory + "../costs_info.txt", "w")
                    f.write("For %s\n" % unique_folder_name)
                    f.write("  file_name_cost = %s\n" % file_name)
                    f.write("  folder_name_nominal_cost = %s\n" % folder_name_nominal_cost)
                    f.write("  nominal_cost = %.3f\n" % nominal_cost)
                    f.write("  (iter 1 normalized cost, min normalized cost, improvement) = (%.3f, %.3f, %.1f%%)\n" % (average_cost[0], min(average_cost), 100 * (average_cost[0] - min(average_cost)) / average_cost[0]))
                    f.close()
                    print("  (nominal_cost, iter 1 normalized cost, min normalized cost, improvement) = (%.3f, %.3f, %.3f, %.1f%%)" % (nominal_cost, average_cost[0], min(average_cost), 100 * (average_cost[0] - min(average_cost)) / average_cost[0]), end='')

            # labels
            plt.xlabel('Iteration')
            if only_plot_average_cost & only_add_successful_samples_to_average_cost:
                plt.ylabel('Averaged (successful) task cost')
            else:
                plt.ylabel('Averaged task cost')
            if not only_plot_average_cost:
                plt.title('Cost over iterations')

            if len(file_name_list) > 1 or not only_plot_average_cost:
                plt.legend()

            # Change the ticks
            # ax.set_yticks(np.arange(1.05,1.301,0.05))

            # Set limit
            # ax.set_ylim(0, 6)

            plt.title("Traj opt cost over model iteration")

            plt.draw()

            # so that the label is not cut off by the window
            # plt.tight_layout()
            plt.gcf().subplots_adjust(bottom=0.15)
            plt.gcf().subplots_adjust(left=0.15)

            if save_figure:
                affix = "_new" if os.path.exists(directory + "../cost.png") else ""
                plt.savefig("%s../cost%s.png" % (directory, affix))

                plt.savefig("../cost_%s.png" % unique_folder_name)
                print(";  figure saved")
                # print("  figure saved for %s" % unique_folder_name)

                plt.clf()
                break

            plt.pause(60)
            plt.clf()

    except Exception as e:
        logging.error(traceback.format_exc())  # Logs the error appropriately.
