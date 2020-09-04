"""
This function is used for normalized cost landscape with another cost landscape and plot the landscape with
continuous color map.
The algorithm is similar to plot_landscape while we process the data on a ray from two data sets.
"""
import matplotlib.pyplot as plt
import numpy as np
import os
import math

robot_option = 1
file_dir = "../dairlib_data/goldilocks_models/find_boundary/"
if robot_option == 1:
    robot = 'cassie/'
else:
    robot = 'five_link/'
dir1 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
        '_range2_iter200/'
dir2 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
       '_nominal_sl_tr/'

# number of searching directions
n_direction = 16

# optimization range
min1 = 0.2775
max1 = 0.3075
min2 = -0.1875
max2 = 0.0625
plot_optimization_range = 0

# Note:decide which column of the task to plot according to the task dimensions
# Eg. column index 0 corresponds to stride length
task_name = ['Stride length', 'Ground incline', 'Velocity', 'Turning rate']
task_1_idx = 0
task_2_idx = 3


def process_data_from_direction(i, dir1, dir_nominal):
    # need to add central point on the points list
    task0 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_task.csv', delimiter=",")
    x = [task0[task_1_idx]]
    y = [task0[task_2_idx]]
    cost1 = float(np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_c.csv', delimiter=","))
    cost2 = float(np.genfromtxt(dir_nominal + str(0) + '_' + str(0) + '_c.csv', delimiter=","))
    z = [cost1 / cost2]

    data_dir1 = np.genfromtxt(dir1 + str(int(i+1)) + '_cost_list.csv', delimiter=",")
    data_dir2 = np.genfromtxt(dir_nominal + str(int(i+1)) + '_cost_list.csv', delimiter=",")

    if data_dir1.shape[0] >= data_dir2.shape[0]:
        num_small = data_dir2.shape[0]
        num_large = data_dir1.shape[0]
    else:
        num_small = data_dir1.shape[0]
        num_large = data_dir2.shape[0]

    # discard this line if the data is not reasonable
    if num_small < 10:
        return x, y, z
    # process the points on the line
    # set the value for intersected parts
    for j in range(num_small):
        cost1 = data_dir1[j, 1]
        cost2 = data_dir2[j, 1]
        # we only append reasonable point
        if (cost1 < 35) & (cost2 < 35) & (cost1/cost2 < 2):
            task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            x.append(task[task_1_idx])
            y.append(task[task_2_idx])
            z.append(cost1/cost2)
    for j in range(num_small, num_large):
        if data_dir1.shape[0] >= data_dir2.shape[0]:
            # extended range
            task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            x.append(task[task_1_idx])
            y.append(task[task_2_idx])
            z.append(0)
        else:
            # extended range
            task = np.genfromtxt(dir_nominal + str(int(data_dir2[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            x.append(task[task_1_idx])
            y.append(task[task_2_idx])
            z.append(2)
    return x, y, z


def find_adjacent_line(dir1, dir2, adj_index, i):
    # process data on adjacent line
    x2, y2, z2 = process_data_from_direction(adj_index[i], dir1, dir2)
    if len(x2) > 10:
        return x2, y2, z2
    else:
        x2, y2, z2 = find_adjacent_line(dir1, dir2, adj_index, adj_index[i])
        return x2, y2, z2


def generateplot(dir1, dir_nominal, adj_index, levels, ticks):
    total_max = 0
    for i in range(n_direction):
        # process data on one line
        x1, y1, z1 = process_data_from_direction(i, dir1, dir_nominal)
        if len(x1) > 10:
            # process data on adjacent line
            x2, y2, z2 = find_adjacent_line(dir1, dir_nominal, adj_index, i)
            # plot
            x = x1+x2
            y = y1+y2
            z = z1+z2
            surf = ax.tricontourf(x, y, z, levels=levels)
            if max(z) > total_max:
                total_max = max(z)
    print('max', total_max)
    fig.colorbar(surf, shrink=0.9, aspect=10, spacing='proportional', ticks=ticks)

# continuous color map
# plt.rcParams.update({'font.size': 28})
# fig, ax = plt.subplots(figsize=(11,5.5))
fig, ax = plt.subplots()
# ceil = int(math.ceil(max(z)))
# print('ceil:', ceil)
levels = [0.0, 0.5, 1, 1.5, 2] # manual specify level sets
ticks = [0.0, 0.5, 1, 1.5, 2] # manual specify ticker values (it seems 0 is ignored)
print(levels)
adjacent = np.genfromtxt('examples/goldilocks_models/find_boundary_utils/adjacent.csv', delimiter=",").astype(int)
generateplot(dir1,dir2, adjacent, levels, ticks)
ax.set_xlabel(task_name[task_1_idx])
ax.set_ylabel(task_name[task_2_idx])
ax.set_title('Normalized cost landscape')

if plot_optimization_range == 1:
    x_line = np.array([min1, max1, max1, min1, min1])
    y_line = np.array([min2, min2, max2, max2, min2])
    ax.plot(x_line, y_line, 'black', linewidth=3)

# so that the label is not cut off by the window
# plt.tight_layout()
# plt.gcf().subplots_adjust(bottom=0.18)
# plt.gcf().subplots_adjust(left=0.15)

plt.show()