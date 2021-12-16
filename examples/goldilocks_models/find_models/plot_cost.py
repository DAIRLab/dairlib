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
plt.rcParams.update({'font.size': 18})

save_figure = True

only_plot_average_cost = True
normalize_by_nominal_cost = True
only_add_successful_samples_to_average_cost = False

iter_start = 1
iter_end = 11
is_iter_end = 0
robot_option = 1;  # 0 is five-link robot. 1 is cassie_fixed_spring
if len(sys.argv) >= 2:
    iter_start = int(sys.argv[1])
if len(sys.argv) >= 3:
    iter_end = int(sys.argv[2])
    is_iter_end = 1
if len(sys.argv) >= 4:
    robot_option = int(sys.argv[3])


n_sampel_sl = 13  # should be > 0
n_sampel_gi = 1  # should be > 0
n_sampel_v = 1  # should be > 0
n_sampel_tr = 1  # should be > 0
n_sampel_ph = 3  # should be > 0
N_sample = n_sampel_sl * n_sampel_gi * n_sampel_v * n_sampel_tr * n_sampel_ph
print('n_sampel_sl = ' + str(n_sampel_sl))
print('n_sampel_gi = ' + str(n_sampel_gi))
print('n_sampel_v = ' + str(n_sampel_v))
print('n_sampel_tr = ' + str(n_sampel_tr))
print('n_sampel_ph = ' + str(n_sampel_ph))

# dist_0 = 0.2
# delta_dist = 0.015 #0.1
# incline_0 = 0.0
# delta_incline = 0.05 #0.08
# turning_0 = 0.0
# delta_turning = 0.15

# directory = 'data/robot_' + str(robot_option) + '/'
directory = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

# file_name = 'c.csv'
# file_name = 'c_without_tau.csv'
file_name1 = 'c_main.csv'
file_name2 = 'c.csv'
file_name_nominal_cost = file_name1

file_name_list = [file_name1, file_name2]

# preprocess
# min_dist = dist_0 - delta_dist * (n_sampel_sl - 1)/2.0
# min_incline = incline_0 - delta_incline * (n_sampel_gi - 1)/2.0
# min_turning = turning_0 - delta_turning * (n_sampel_tr - 1)/2.0

# visualization setting
ave_cost_prop = ""
if only_plot_average_cost:
    ave_cost_prop = "k-"
else:
    ave_cost_prop = "k--"
ave_cost_label = ""
if only_add_successful_samples_to_average_cost:
    ave_cost_label = "Averaged cost (excluding failed samples)";
else:
    ave_cost_label = "Averaged cost";

# get nomial cost
nominal_cost = 0.0
if normalize_by_nominal_cost:
    for sample_i in range(N_sample):
        cost = []
        assert os.path.isfile(directory+'nominal_no_constraint_traj/'+'0_'+str(sample_i)+'_'+file_name_nominal_cost), 'file does not exist'
        matrix = np.genfromtxt (directory+'nominal_no_constraint_traj/'+'0_'+str(sample_i)+'_'+file_name_nominal_cost, delimiter=",")
        cost.append(matrix)

        nominal_cost += cost[0] / N_sample
else:
    nominal_cost = 1.0;
print('nominal_cost = '+str(nominal_cost))

# plot
while 1:
    # fig1 = plt.figure(1)
    fig1 = plt.figure(num=1, figsize=(6.4, 4.8))
    ax = fig1.gca()

    for file_name in file_name_list:
        # Get the length of the cost first (in case the lengths of different samples are the not the same). This is for plotting the average cost
        len_total_cost = 0
        for sample_i in range(N_sample):
            cost = []
            iteration = iter_start
            while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name):
                matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, delimiter=",") / nominal_cost
                cost.append(matrix)
                if is_iter_end & (iteration == iter_end):
                    break;
                iteration+=1
            if len_total_cost == 0:
                len_total_cost = len(cost)
            else:
                len_total_cost = min(len_total_cost, len(cost))

        # Initialize total_cost with all zeros
        total_cost = [0] * len_total_cost

        #
        n_successful_sample_each_iter = [0] * len_total_cost;

        # 1. Plot each sample
        for sample_i in range(N_sample):
            # read in cost
            cost = []
            iteration = iter_start
            while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name):
                matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, delimiter=",") / nominal_cost
                cost.append(matrix)
                if is_iter_end & (iteration == iter_end):
                    break
                iteration+=1

            # plot cost for each sample
            length = len(cost)
            t = range(iter_start,length+iter_start)
            if not only_plot_average_cost:
                ax.plot(t,cost, label='sample_idx = '+str(sample_i))
                # TODO: not very important, but you can read the task value and use it as a label

            # Read in is_success
            is_success = []
            if only_add_successful_samples_to_average_cost:
                for iter_i in range(iter_start, iter_start + len_total_cost):
                    matrix = np.genfromtxt (directory+str(iter_i)+'_'+str(sample_i)+'_is_success.csv', delimiter=",")
                    is_success.append(matrix)
            else:
                is_success = [1] * len_total_cost

            # Add to accumulated success
            n_successful_sample_each_iter = [x + y for x, y in zip(n_successful_sample_each_iter, is_success)]

            # Add to total cost
            filtered_cost = [x * y for x, y in zip(is_success, cost[0:len_total_cost])]
            total_cost = [x + y for x, y in zip(total_cost, filtered_cost[0:len_total_cost])]

        # 2. Plot average cost
        average_cost = [x / y for x, y in zip(total_cost, n_successful_sample_each_iter)]
        ax.plot(t[0:len_total_cost],average_cost, ave_cost_prop, linewidth=3.0, label=ave_cost_label + "; " + file_name)

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
        plt.savefig("../cost.png")
        print("figure saved")
        break

    plt.pause(10)
    plt.clf()
