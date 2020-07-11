"""
This function is used for plotting normalized cost landscape.
Considering we search along several directions, we process the data along those directions
1.
For each direction, we compare the cost at each point on the searching line both in cost landscape 1 (C1) and
cost landscape. Set the value of reasonable point with the normalized cost.
2.
For those points which exist in the cost landscape we want to normalize but not in the nominal cost landscape,
we set the value of this point 0.5.
"""
import matplotlib.pyplot as plt
import numpy as np
import os

robot_option = 1
file_dir = '/Users/jason-hu/'
if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_grid_iter50_sl_tr/'
    dir2 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_nominal_sl_tr/'

# number of searching directions
n_direction = 16


# Note:decide which column of the task to plot according to the task dimensions
# Eg. column index 0 corresponds to stride length
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

        # set the value for intersected parts
        for j in range(num_small):
            cost1 = data_dir1[j, 1]
            cost2 = data_dir2[j, 1]
            # we only append reasonable point
            if (cost1/cost2 < 1.5) & (cost1/cost2 > 0.5):
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
                z.append(0.5)
        if len(x) > 10:
            x0 = x0+x
            y0 = y0+y
            z0 = z0+z
    return np.array(x0), np.array(y0), np.array(z0)


# continuous color map
fig, ax = plt.subplots()
x, y, z = process_data_from_direction(dir1, dir2)
# ax.scatter(x,y)
surf = ax.tricontourf(x, y, z)
fig.colorbar(surf, shrink=0.5, aspect=6)
ax.set_xlabel('Stride length')
ax.set_ylabel('Turning rate')
ax.set_title('normalized cost landscape')

plt.show()