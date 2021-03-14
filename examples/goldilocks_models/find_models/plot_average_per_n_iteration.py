import matplotlib.pyplot as plt
import numpy as np
import os

# setting of program
robot_option = 1
normalized_cost = 1
use_landscape_for_nominal = 0
plot_successful_sample = 1
item_to_plot = 0

if item_to_plot == 0:
    item = 'c'
if item_to_plot == 1:
    item = 'solve_time'

if robot_option == 0:
    robot = 'five_link/'
else:
    if robot_option == 1:
        robot = 'cassie/'

# plot setting
rom1 = '1D_LIP'
rom2 = '2D_LIP'
rom3 = '3D_LIP'
task_space1 = '2D_task_space'
task_space2 = '3D_task_space'
task_space5 = '4D_task_space'
task_space3 = 'large_task_space'
task_space4 = 'small_task_space'

# task space
large_task_space = 1
if large_task_space == 1:
    min_gi1 = -0.325
    max_gi1 = 0.275
    min_sl1 = 0.0925
    max_sl1 = 0.3925
else:
    min_gi1 = -0.175
    max_gi1 = 0.125
    min_sl1 = 0.1675
    max_sl1 = 0.3175


file_dir = "../dairlib_data/goldilocks_models/find_models/"
file_dir2 = "../dairlib_data/goldilocks_models/find_boundary/"

rom = rom3
task_space = task_space5
method ='(nongrid)'
iter_start = 1
iter_end = 700
dir1 = file_dir+robot+'robot_1/'
label1 = 'optimizing '+rom+' over '+task_space+method
line_type1 = 'k-'

dir2 = file_dir+robot+task_space4+'robot_' + str(robot_option)+'_iter2000/'
label2 = 'optimizing 2D ROM over small task space'
line_type2 = 'k--'

dir3 = file_dir+robot+rom1+task_space1+task_space3+'robot_' + str(robot_option)+'_large_iter100_more_samples/'
label3 = 'optimizing 1D ROM over large task space with more samples'
line_type3 = 'k:'

dir_landscape = file_dir2+robot+rom1+task_space1+'/robot_' + str(robot_option)+ '_2d_nominal/'


def calculate_nominal_cost_from_cost_landscape(dir_find_boundary):
    i = 0
    nominal_cost = 0
    num = 0
    while os.path.isfile(dir_find_boundary + str(i) + '_' + str(0) + '_task.csv'):
        gamma = np.genfromtxt(dir_find_boundary + str(i) + '_' + str(0) + '_task.csv', delimiter=",")
        cost = float(np.genfromtxt(dir_find_boundary + str(i) + '_' + str(0) + '_c.csv', delimiter=","))
        if np.genfromtxt(dir_find_boundary + str(i) + '_' + str(0) + '_is_success.csv', delimiter=",") == 1:
            # within the optimization range
            if (gamma[0] >= min_sl1) & (gamma[0] <= max_sl1) & (gamma[1] >= min_gi1) & (gamma[1] <= max_gi1):
                nominal_cost = nominal_cost+cost
                num = num+1
        i = i+1
    nominal_cost = nominal_cost/num
    return nominal_cost


def average_cost_several_iter(iter_start, iter_end, n, dir, line_type, label_name, normalized, use_landscape, dir_nominal):
    # get nominal cost from iteration 0
    if item_to_plot == 0:
        if normalized == 1:
            if use_landscape == 1:
                nominal_cost = calculate_nominal_cost_from_cost_landscape(dir_nominal)
                print('normalize the cost by nominal cost:', nominal_cost)
            else:
                nominal_cost = []
                j = 0
                while os.path.isfile(dir_nominal+str(0)+'_'+str(j)+'_c.csv'):
                    if np.genfromtxt(dir_nominal+str(0)+'_'+str(j)+'_is_success.csv', delimiter=","):
                        nominal_cost.append(np.genfromtxt(dir_nominal+str(0)+'_'+str(j)+'_c.csv', delimiter=","))
                    j = j+1
                nominal_cost = np.array(nominal_cost).sum()/len(nominal_cost)
                print('normalize the cost by nominal cost:', nominal_cost)
        else:
            nominal_cost = 1
            print('without normalizing the cost by nominal cost')

    aver_cost = []
    for i in range(iter_start, iter_end+1):
        cost = []
        j = 0
        while os.path.isfile(dir+str(i)+'_'+str(j)+'_'+item+'.csv'):
            if plot_successful_sample:
                if float(np.genfromtxt(dir+str(i)+'_'+str(j)+'_is_success.csv', delimiter=",")) == 1:
                    if float(np.genfromtxt(dir+str(i)+'_'+str(j)+'_'+item+'.csv', delimiter=",")) < 15:
                        cost.append(np.genfromtxt(dir+str(i)+'_'+str(j)+'_'+item+'.csv', delimiter=","))
            else:
                cost.append(np.genfromtxt(dir + str(i) + '_' + str(j) + '_' + item + '.csv', delimiter=","))
            j = j+1
        aver_cost.append(np.array(cost).sum()/len(cost)/nominal_cost)
    print('initial cost:', aver_cost[0] * nominal_cost)
    print('normalized initial cost:', aver_cost[0])
    print('final cost:', aver_cost[-1]*nominal_cost)
    print('normalized final cost:', aver_cost[-1])
    # average the cost every n iterations
    cost = np.array(aver_cost)
    aver_cost = []
    j = 0
    while j < len(cost):
        if len(cost)-j < n:
            pass
            # aver_cost.append(cost[j:len(cost)].sum()/(len(cost)-j))
        else:
            aver_cost.append(cost[j:j+n].sum()/n)
        j = j + n
    ax1.plot(range(1, len(aver_cost)+1), aver_cost, line_type, linewidth=3.0, label=label_name)


fig1 = plt.figure(num=1, figsize=(6.4, 4.8))
ax1 = fig1.gca()
average_cost_several_iter(iter_start, iter_end, 5, dir1, line_type1, label1, normalized_cost, use_landscape_for_nominal, dir1)
# average_cost_several_iter(1, 2000, 20, dir2, line_type2, label2, normalized_cost, use_landscape_for_nominal, dir2)
# average_cost_several_iter(1, 100, 10, dir3, line_type3, label3, normalized_cost, use_landscape_for_nominal, dir_landscape)
plt.xlabel('Iteration')
if normalized_cost:
    plt.ylabel('Normalized average cost of samples')
else:
    plt.ylabel('Average cost of samples')
plt.legend()
plt.show()
