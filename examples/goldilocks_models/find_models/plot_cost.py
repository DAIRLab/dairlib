import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import time
import sys
plt.rcParams.update({'font.size': 18})

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


n_sampel_sl = 3  # should be > 0
n_sampel_gi = 3  # should be > 0
n_sampel_tr = 3  # should be > 0
N_sample = n_sampel_sl * n_sampel_gi * n_sampel_tr
print('n_sampel_sl = ' + str(n_sampel_sl))
print('n_sampel_gi = ' + str(n_sampel_gi))
print('n_sampel_tr = ' + str(n_sampel_tr))

dist_0 = 0.2
delta_dist = 0.015 #0.1
incline_0 = 0.0
delta_incline = 0.05 #0.08
turning_0 = 0.0
delta_turning = 0.15

# directory = 'data/robot_' + str(robot_option) + '/'
directory = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

# file_name = 'c.csv'
file_name = 'c_without_tau.csv'

# preprocess
min_dist = dist_0 - delta_dist * (n_sampel_sl - 1)/2.0
min_incline = incline_0 - delta_incline * (n_sampel_gi - 1)/2.0
min_turning = turning_0 - delta_turning * (n_sampel_tr - 1)/2.0

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
        assert os.path.isfile(directory+'nominal_no_constraint_traj/'+'0_'+str(sample_i)+'_'+file_name), 'file does not exist'
        matrix = np.genfromtxt (directory+'nominal_no_constraint_traj/'+'0_'+str(sample_i)+'_'+file_name, delimiter=",")
        cost.append(matrix)

        nominal_cost += cost[0] / N_sample;
else:
    nominal_cost = 1.0;
print('nominal_cost = '+str(nominal_cost))

# plot
while 1:
    # fig1 = plt.figure(1)
    fig1 = plt.figure(num=1, figsize=(6.4, 4.8))
    ax1 = fig1.gca()

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

    # Plot each sample
    for sample_i in range(N_sample):
        # read in cost
        cost = []
        iteration = iter_start
        while os.path.isfile(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name):
            # way1
            matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_i)+'_'+file_name, delimiter=",") / nominal_cost
            cost.append(matrix)
            # way2
            # with open(directory+str(iteration)+'_'+str(sample_i)+'_'+file_name,'r') as csvfile:
            #     plots = csv.reader(csvfile, delimiter=',')
            #     for row in plots:
            #         cost.append(row[0])
            if is_iter_end & (iteration == iter_end):
                break;
            iteration+=1

        # plot cost for each sample
        length = len(cost)
        t = range(iter_start,length+iter_start)
        if not only_plot_average_cost:
            if (n_sampel_gi == 1) & (n_sampel_tr == 1):
                ax1.plot(t,cost, label='sl = '+str(min_dist+(sample_i%n_sampel_sl)*delta_dist)+' (m)')
            elif n_sampel_tr == 1:
                ax1.plot(t,cost, label='sl = '+str(min_dist+(sample_i%n_sampel_sl)*delta_dist)+' (m), gi = '+str(min_incline+(sample_i/n_sampel_sl)*delta_incline)+' (rad)')
            else:
                ax1.plot(t,cost, label='sl = '+str(min_dist+(sample_i%n_sampel_sl)*delta_dist)+' (m), gi = '+str(min_incline+(sample_i/n_sampel_sl)*delta_incline)+' (rad), turning rate = '+str(min_turning+(sample_i/(n_sampel_sl*n_sampel_gi))*delta_turning)+' (rad/s)')

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

    # plot average cost
    average_cost = [x / y for x, y in zip(total_cost, n_successful_sample_each_iter)]
    ax1.plot(t[0:len_total_cost],average_cost, ave_cost_prop, linewidth=3.0, label=ave_cost_label)

    # labels
    plt.xlabel('Iteration')
    if only_plot_average_cost & only_add_successful_samples_to_average_cost:
        plt.ylabel('Averaged (successful) task cost')
    else:
        plt.ylabel('Averaged task cost')
    if not only_plot_average_cost:
        plt.title('Cost over iterations')
        # plt.legend()

    # Change the ticks
    # ax1.set_yticks(np.arange(1.05,1.301,0.05))

    # Set limit
    # ax1.set_ylim(0, 6)

    plt.draw()

    # so that the label is not cut off by the window
    # plt.tight_layout()
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)

    plt.pause(10)
    plt.clf()
