"""
This function is used for comparing two cost landscape and plot the landscape with discrete color map.
Considering we search along several directions, we process the data along those directions
1.
For each direction, we compare the cost at each point on the searching line both in cost landscape 1 (C1) and
cost landscape. Set the value of the point according the cost.
2.
For those points which are not in two landscape, if it exists in C1, we set the value of this point -0.5,
else we set the value of this point 2
"""
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import os

robot_option = 1
file_dir = '/Users/jason-hu/'
if robot_option == 1:
    robot = 'cassie/'
else:
    robot = 'five_link/'
dir1 = file_dir + 'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
        '_range1_iter300/'
dir2 = file_dir + 'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
       '_grid_iter50_sl_tr/'

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


def process_data_from_direction(dir1, dir_nominal):
    # need to add central point on the points list
    task0 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_task.csv', delimiter=",")
    x0 = [task0[task_1_idx]]
    y0 = [task0[task_2_idx]]
    cost1 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_c.csv', delimiter=",")
    cost2 = np.genfromtxt(dir_nominal + str(0) + '_' + str(0) + '_c.csv', delimiter=",")
    z0 = [cost1 / cost2]
    for i in range(n_direction):
        data_dir1 = np.genfromtxt(dir1 + str(i+1) + '_cost_list.csv', delimiter=",")
        data_dir2 = np.genfromtxt(dir_nominal + str(i+1) + '_cost_list.csv', delimiter=",")

        if data_dir1.shape[0] >= data_dir2.shape[0]:
            num_small = data_dir2.shape[0]
            num_large = data_dir1.shape[0]
        else:
            num_small = data_dir1.shape[0]
            num_large = data_dir2.shape[0]

        # process the points on the line
        x = []
        y = []
        z = []

        if num_small > 10:
            # set the value for intersected parts
            for j in range(num_small):
                cost1 = data_dir1[j, 1]
                cost2 = data_dir2[j, 1]
                # only consider reasonable point
                if (cost1 < 30) & (cost2 < 30):
                    z.append(cost1 / cost2)
                    task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
                    x.append(task[task_1_idx])
                    y.append(task[task_2_idx])
            for j in range(num_small, num_large):
                if data_dir1.shape[0] >= data_dir2.shape[0]:
                    # extended range
                    task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
                    x.append(task[task_1_idx])
                    y.append(task[task_2_idx])
                    z.append(-0.5)
                else:
                    # shrunk range
                    task = np.genfromtxt(dir_nominal + str(int(data_dir2[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
                    x.append(task[task_1_idx])
                    y.append(task[task_2_idx])
                    z.append(2.5)
            x0 = x0 + x
            y0 = y0 + y
            z0 = z0 + z
    return np.array(x0), np.array(y0), np.array(z0)


# plt.rcParams.update({'font.size': 28})
# fig, ax = plt.subplots(figsize=(11,5.5))
fig, ax = plt.subplots()
x, y, z = process_data_from_direction(dir1, dir2)
# ax.scatter(x, y)
# discrete color map
print('max', max(z))
levels = [0, 0.25, 0.5, 0.75, 1, 2]
colors = ['darkgreen', 'green', 'seagreen', 'mediumseagreen', 'blue']
cmap, norm = matplotlib.colors.from_levels_and_colors(levels, colors)
cmap.set_over('yellow')
cmap.set_under('red')
surf = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, levels=levels, extend='both')
cbar = fig.colorbar(surf, shrink=0.9, aspect=10, extend='both')
cbar.ax.set_yticklabels(['0', '0.25', '0.5', '0.75', '1', 'Inf'])

ax.set_xlabel(task_name[task_1_idx])
ax.set_ylabel(task_name[task_2_idx])
ax.set_title('Compare two cost landscapes')

if plot_optimization_range == 1:
    x_line = np.array([min1, max1, max1, min1, min1])
    y_line = np.array([min2, min2, max2, max2, min2])
    ax.plot(x_line, y_line, 'black', linewidth=3)

# so that the label is not cut off by the window
# plt.tight_layout()
# plt.gcf().subplots_adjust(bottom=0.17)
# plt.gcf().subplots_adjust(left=0.16)

plt.show()