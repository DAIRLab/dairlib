"""
Given points on two adjacent lines, fill the region between two lines with the color corresponding to the cost.
"""
import matplotlib.pyplot as plt
import numpy as np
import os

robot_option = 1
file_dir = '../dairlib_data/goldilocks_models/find_boundary/'

if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_nominal_sl_tr/'
    dir2 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_initial_sl_tr/'
    dir3 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_range1_iter300/'
    dir4 = file_dir + robot + '2D_LIP/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_range2_iter200/'
    dir5 = file_dir+'robot_1/'

# number of searching directions
n_direction = 16

# Note: we need to decide which column of the searching direction to use
# Eg. column index 0 corresponds to stride length

# f = open(dir + "task_names.csv", "r")
# task_name = f.read().splitlines()
task_name = ['Stride length', 'Ground incline', 'Velocity', 'Turning rate']
task_1_idx = 0
task_2_idx = 3


def ExtractAdjacentLine(dir):
    adj_direction = np.zeros([n_direction]).astype(int)
    for i in range(n_direction):
        min_sin = 1
        line1 = np.genfromtxt(dir + str(i + 1) + '_searching_direction.csv', delimiter=",")
        vec1 = np.array([line1[task_1_idx], line1[task_2_idx]])
        for j in range(n_direction):
            line2 = np.genfromtxt(dir + str(j+1) + '_searching_direction.csv', delimiter=",")
            vec2 = np.array([line2[task_1_idx], line2[task_2_idx]])
            sin = np.cross(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
            cos = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
            # restrict the direction
            if (cos > 0) & (sin > 0):
                # find the adjacent pair
                if sin < min_sin:
                    adj_index = j
                    min_sin = sin
        adj_direction[i] = adj_index
    return adj_direction


def process_data_from_direction(i, dir1):
    data_dir = np.genfromtxt(dir1 + str(int(i+1)) + '_cost_list.csv', delimiter=",")

    # need to add central point on the points list
    task0 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_task.csv', delimiter=",")
    x = [task0[task_1_idx]]
    y = [task0[task_2_idx]]
    z = [float(np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_c.csv', delimiter=","))]

    # process the points on the line
    if data_dir.ndim > 1:
        for j in range(data_dir.shape[0]):
            task = np.genfromtxt(dir1 + str(int(data_dir[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            cost = data_dir[j, 1]
            if cost < 35:
                x.append(task[task_1_idx])
                y.append(task[task_2_idx])
                z.append(cost)

    return x, y, z


def find_adjacent_line(dir1, adj_index, i):
    # process data on adjacent line
    x2, y2, z2 = process_data_from_direction(adj_index[i], dir1)
    if len(x2) > 10:
        return x2, y2, z2
    else:
        x2, y2, z2 = find_adjacent_line(dir1, adj_index, adj_index[i])
        return x2, y2, z2


def generateplot(dir1, adj_index):
    for i in range(n_direction):
        # process data on one line
        x1, y1, z1 = process_data_from_direction(i, dir1)
        # process data on adjacent line
        if len(x1) > 10:
            x2, y2, z2 = find_adjacent_line(dir1, adj_index, i)
            # plot
            x = x1+x2
            y = y1+y2
            z = z1+z2
            # surf = ax.scatter(x, y, z)
            surf = ax.tricontourf(x, y, z, levels=[0, 4, 8, 12, 16, 20, 24, 28, 32])
    fig.colorbar(surf, shrink=0.5, aspect=6)


# adjacent = ExtractAdjacentLine(dir3)
# np.savetxt('examples/goldilocks_models/find_boundary_utils/adjacent.csv', adjacent, delimiter=',')

fig, ax = plt.subplots()
ax.set_xlabel(task_name[task_1_idx])
ax.set_ylabel(task_name[task_2_idx])
ax.set_title('cost landscape')
adjacent = np.genfromtxt('examples/goldilocks_models/find_boundary_utils/adjacent.csv', delimiter=",").astype(int)
generateplot(dir4, adjacent)
plt.show()