"""
Given points on two adjacent lines, fill the region between two lines with the color corresponding to the cost.
Input: points on two lines
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

def decidecolor(cost):
    if cost < 1:
        color = "green"
        if cost == 0:
            color = "red"
    if cost >= 1:
        color = "blue"
        if cost > 1.5:
            color = "yellow"
    return color


# x1, y1, x2, y2, x3, y3 are scalar
def tricolorplot(x1, y1, x2, y2, x3, y3, color):
    X = np.array([x1, x2, x3, x1])
    Y = np.array([y1, y2, y3, y1])
    XY = np.ones([X.shape[0], 2])
    XY[:, 0] = X
    XY[:, 1] = Y
    t1 = plt.Polygon(XY, color=color)
    ax1.add_patch(t1)
    plt.plot(X, Y, color=color)


# x1, y1, z1, x2, y2, z2 are array
def polycolorplot(x1, y1, z1, x2, y2, z2):

    # use i,j to represent the index of the two points on two lines;
    i = 1
    j = 1
    # plot two triangles
    while (i < x1.shape[0]) & (j < x2.shape[0]):
        average_cost = (z1[i-1]+z1[i]+z2[j-1]) / 3
        color = decidecolor(average_cost)
        tricolorplot(x1[i-1], y1[i-1], x1[i], y1[i], x2[j-1], y2[j-1], color)

        average_cost = (z1[i]+z2[i-1]+z2[i]) / 3
        color = decidecolor(average_cost)
        tricolorplot(x1[i], y1[i], x2[j-1], y2[j-1], x2[j], y2[j], color)

        i = i+1
        j = j+1
    # after point on one line reach the edge
    if i == x1.shape[0]:
        while j < x2.shape[0]:
            average_cost = (z1[i-1] + z2[j-1] + z2[j]) / 3
            color = decidecolor(average_cost)
            tricolorplot(x1[i-1], y1[i-1], x2[j-1], y2[j-1], x2[j], y2[j], color)
            j = j+1
    if j == x2.shape[0]:
        while i < x1.shape[0]:
            average_cost = (z1[i-1] + z1[i] + z2[j-1]) / 3
            color = decidecolor(average_cost)
            tricolorplot(x1[i-1], y1[i-1], x1[i], y1[i], x2[j-1], y2[j-1], color)
            i = i+1



def extractadjacentline(dir):
    adj_direction = np.zeros([n_direction])
    for i in range(n_direction):
        min_sin = 1
        for j in range(n_direction):
            # Note:decide which column of the searching direction to use
            # Eg. column index 0 corresponds to stride length
            line1 = np.genfromtxt(dir + str(i+1) + '_searching_direction.csv', delimiter=",")
            vec1 = np.array([line1[0], line1[3]])
            line2 = np.genfromtxt(dir + str(j+1) + '_searching_direction.csv', delimiter=",")
            vec2 = np.array([line2[0], line2[3]])
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


def process_data_from_direction(i, dir1, dir2):
    data_dir1 = np.genfromtxt(dir1 + str(int(i+1)) + '_cost_list.csv', delimiter=",")
    data_dir2 = np.genfromtxt(dir2 + str(int(i+1)) + '_cost_list.csv', delimiter=",")
    # choose the longer line
    if data_dir1.shape[0] >= data_dir2.shape[0]:
        num_large = data_dir1.shape[0]
        num_small = data_dir2.shape[0]
    else:
        num_large = data_dir2.shape[0]
        num_small = data_dir1.shape[0]
    # need to add central point on the points list
    task0 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_task.csv', delimiter=",")
    x0 = task0[0]
    y0 = task0[3]
    cost1 = np.genfromtxt(dir1 + str(0) + '_' + str(0) + '_c.csv', delimiter=",")
    cost2 = np.genfromtxt(dir2 + str(0) + '_' + str(0) + '_c.csv', delimiter=",")
    if cost1 > cost2:
        z0 = 1.5
    else:
        z0 = 0.5
    # process the points on the line
    x = np.zeros([num_large])
    y = np.zeros([num_large])
    z = np.zeros([num_large])
    # set the value for intersected parts
    # Note:decide which column of the task to plot according to the task dimensions
    # Eg. column index 0 corresponds to stride length
    for j in range(num_small):
        task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
        x[j] = task[0]
        y[j] = task[3]
        cost1 = data_dir1[j,1]
        cost2 = data_dir2[j,1]
        if cost1 > cost2:
            z[j] = 1.5
        else:
            z[j] = 0.5
    for j in range(num_small, num_large):
        if data_dir1.shape[0] >= data_dir2.shape[0]:
            # extended range
            task = np.genfromtxt(dir1 + str(int(data_dir1[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            x[j] = task[0]
            y[j] = task[3]
            z[j] = 0
        else:
            # shrunk range
            task = np.genfromtxt(dir2 + str(int(data_dir2[j, 0])) + '_' + str(0) + '_task.csv', delimiter=",")
            x[j] = task[0]
            y[j] = task[3]
            z[j] = 2
    if i != 0:
        x = np.concatenate([np.array([x0]), x])
        y = np.concatenate([np.array([y0]), y])
        z = np.concatenate([np.array([z0]), z])

    return x,y,z


def generateplot(dir1, dir2, adj_index):
    for i in range(n_direction):
        # process data on first line
        x1, y1, z1 = process_data_from_direction(i, dir1, dir2)
        # process data on adjacent line
        x2, y2, z2 = process_data_from_direction(adj_index[i], dir1, dir2)
        # plot
        polycolorplot(x1, y1, z1, x2, y2, z2)


fig1 = plt.figure(num=1, figsize=(6.4, 4.8))
ax1 = fig1.gca()
adjacent = extractadjacentline(dir1)
print(adjacent)
generateplot(dir1, dir2, adjacent)
plt.show()

[0,-1,-0.5,0.5,1]
[0,-1,-0.5,0.5,1]

0,-1
0,1
-1,0
-1,-1
-1,-0.5
-1,0.5
-1,1
-0.5,-1
-0.5,1
0.5,0
0.5,-1
0.5,-0.5
0.5,0.5
0.5,1
1,-0.5
1,0.5




