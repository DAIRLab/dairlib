"""
Considering that we search along different directions to get the boundary, we can use the data point on the lines to
generate landscape.
Currently the best method is plotting the region within two adjacent rays using the function traicontourf with
continuous color map.
"""

import matplotlib.pyplot as plt
import numpy as np

robot_option = 1
file_dir = '../dairlib_data/goldilocks_models/find_boundary/'

if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir+'robot_1/'

# number of searching directions
n_direction = 16

# Note: we need to decide which column of the task to use
# Eg. column index 0 corresponds to stride length
# Currently we manually specify the task names, we will save the task name when generating the data in the future.
# f = open(dir + "task_names.csv", "r")
# task_name = f.read().splitlines()
task_name = ['Stride length', 'Ground incline', 'Velocity', 'Turning rate']
task_1_idx = 0
task_2_idx = 1

# define a flag used to decide the plotting function: use tricontourf or scatter
# scatter can used for debugging and checking the feasibility of the data
is_contour_plot = True


# function used to find the index of adjacent ray
def extract_adjacent_line(dir):
    # array used to save the index of adjacent ray for each ray
    adj_direction = np.zeros([n_direction]).astype(int)
    for i in range(n_direction):
        min_sin = 1
        line1 = np.genfromtxt(dir + str(i + 1) + '_searching_direction.csv', delimiter=",")
        vec1 = np.array([line1[task_1_idx], line1[task_2_idx]])
        for j in range(n_direction):
            line2 = np.genfromtxt(dir + str(j + 1) + '_searching_direction.csv', delimiter=",")
            vec2 = np.array([line2[task_1_idx], line2[task_2_idx]])
            # find the adjacent rays according to the angle between two rays
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


# function to process the data on one ray
def process_data_from_direction(i, dir1):
    # cost list is used to store the cost for each sample
    # the first column stores the sample index and the second column stores the corresponding cost
    data_dir = np.genfromtxt(dir1 + str(int(i+1)) + '_cost_list.csv', delimiter=",")

    # need to add central point on the points list
    task0 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_task.csv', delimiter=",")
    x = [task0[task_1_idx]]
    y = [task0[task_2_idx]]
    z = [float(np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_c.csv', delimiter=","))]

    # process the points on the line
    if data_dir.ndim > 1:
        # avoid error when there is only one sample on the ray
        for j in range(data_dir.shape[0]):
            task = np.genfromtxt(dir1 + str(int(data_dir[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            cost = data_dir[j, 1]
            if cost < 35:
                # only include reasonable samples
                x.append(task[task_1_idx])
                y.append(task[task_2_idx])
                z.append(cost)

    return x, y, z


# recursively find the reasonable adjacent ray and return the processed data
def find_adjacent_line(dir1, adj_index, i):
    # process data on adjacent line
    x2, y2, z2 = process_data_from_direction(adj_index[i], dir1)
    if len(x2) > 10:
        return x2, y2, z2
    else:
        x2, y2, z2 = find_adjacent_line(dir1, adj_index, adj_index[i])
        return x2, y2, z2


# plotting each segment between two rays
def generate_plot(dir1, adj_index):
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
            if is_contour_plot:
                # we can manually set the color discretization if needed
                surf = ax.tricontourf(x, y, z, levels=[0, 4, 8, 12, 16, 20, 24, 28, 32])
            else:
                surf = ax.scatter(x, y, z)
    if is_contour_plot:
        fig.colorbar(surf, shrink=0.5, aspect=6)


adjacent = extract_adjacent_line(dir1)
# np.savetxt('examples/goldilocks_models/find_boundary_utils/adjacent.csv', adjacent, delimiter=',')
# adjacent = np.genfromtxt('examples/goldilocks_models/find_boundary_utils/adjacent.csv', delimiter=",").astype(int)

fig, ax = plt.subplots()
generate_plot(dir1, adjacent)
ax.set_xlabel(task_name[task_1_idx])
ax.set_ylabel(task_name[task_2_idx])
ax.set_title('cost landscape')
plt.show()
