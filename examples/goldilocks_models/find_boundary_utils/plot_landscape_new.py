"""
Given points on two adjacent lines, fill the region between two lines with the color corresponding to the cost.
"""
import matplotlib.pyplot as plt
import numpy as np
import os

robot_option = 1
file_dir = "/home/jianshu/workspace/dairlib_data/goldilocks_models/find_boundary/"
# file_dir = "../dairlib_data/goldilocks_models/find_boundary/"
if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + '_grid_iter50_sl_tr/'

# number of searching directions
n_direction = 16

# Note: we need to decide which column of the searching direction to use
# Eg. column index 0 corresponds to stride length
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
    for i in range(data_dir.shape[0]):
        gamma = np.genfromtxt(dir1 + str(int(data_dir[i, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
        cost = data_dir[i, 1]
        if cost < 35:
            x.append(gamma[task_1_idx])
            y.append(gamma[task_2_idx])
            z.append(cost)

    return x,y,z


def generateplot(dir1, adj_index):
    for i in range(n_direction):
        # process data on one line
        x1, y1, z1 = process_data_from_direction(i, dir1)
        # process data on adjacent line
        x2, y2, z2 = process_data_from_direction(adj_index[i], dir1)
        # plot
        x = x1+x2
        y = y1+y2
        z = z1+z2
        surf = ax.tricontourf(x, y, z, levels=[0, 4, 8, 12, 16, 20, 24, 28, 32])
    fig.colorbar(surf, shrink=0.5, aspect=6)


fig, ax = plt.subplots()
ax.set_xlabel(task_name[task_1_idx])
ax.set_ylabel(task_name[task_2_idx])
ax.set_title('cost landscape')
adjacent = ExtractAdjacentLine(dir1)
print(adjacent)
generateplot(dir1, adjacent)
plt.show()