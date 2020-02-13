import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import time
import sys

# This script plots 's', 'ds', 'dds', or 'tau'.

iteration_start = 1
iteration_end = 35
iteration_spacing = 1
name_idx = 0
if len(sys.argv) >= 2:
    iteration_start = int(sys.argv[1])
if len(sys.argv) >= 3:
    iteration_end = int(sys.argv[2])
if len(sys.argv) >= 4:
    iteration_spacing = int(sys.argv[3])
if len(sys.argv) >= 5:
    name_idx = int(sys.argv[4])

batch = 0

robot_option = 1;  # 0 is five-link robot. 1 is cassie_fixed_spring
directory = 'data/robot_' + str(robot_option) + '/'


name_list = ['s', 'ds', 'dds', 'tau']
name = name_list[name_idx]

# Get the max and min
max_list = []
min_list = []
for iteration in range(iteration_start,iteration_end+1):
    if os.path.isfile(directory+str(iteration)+'_'+str(batch)+'_t_and_'+name+'.csv'):
        matrix = np.genfromtxt (directory+str(iteration)+'_'+str(batch)+'_t_and_'+name+'.csv', delimiter=",")
        n_rows = (matrix.shape)[0]

        for index in range(1,n_rows):
            i_th_element = matrix[index,:]
            max_list.append(np.amax(i_th_element))
            min_list.append(np.amin(i_th_element))
max_val = max(max_list)
min_val = min(min_list)

# Plot
fig = plt.figure(1)

for iteration in range(iteration_start,iteration_end+1,iteration_spacing):
    ax = fig.gca()
    if os.path.isfile(directory+str(iteration)+'_'+str(batch)+'_t_and_'+name+'.csv'):
        matrix = np.genfromtxt (directory+str(iteration)+'_'+str(batch)+'_t_and_'+name+'.csv', delimiter=",")
        n_rows = (matrix.shape)[0]

        for index in range(1,n_rows):
            t = matrix[0,:]
            i_th_element = matrix[index,:]
            ax.plot(t,i_th_element, label=name+'_'+str(index))
            ax.tick_params(axis='x', labelsize=15)
            ax.tick_params(axis='y', labelsize=15)

    cost = []
    if os.path.isfile(directory+str(iteration)+'_'+str(batch)+'_c.csv'):
        matrix = np.genfromtxt (directory+str(iteration)+'_'+str(batch)+'_c.csv', delimiter=",")
        cost.append(matrix)

    plt.xlabel('t (seconds)', fontsize=15)
    plt.ylabel(name, fontsize=15)
    # plt.title('Generalized position trajectories.')
    plt.title('Iteration #'+str(iteration)+': cost = '+str(cost[0]))
    leg = plt.legend()


    # Set the axis limit
    # ax.set_xlim(0, 0.95)
    ax.set_ylim(min_val, max_val)

    # Draw the figure so you can find the positon of the legend.
    plt.draw()

    # # Get the bounding box of the original legend
    # bb = leg.get_bbox_to_anchor().inverse_transformed(ax.transAxes)
    # # Change to location of the legend.
    # bb.x0 += 0
    # # bb.x1 += 0.12
    # leg.set_bbox_to_anchor(bb, transform = ax.transAxes)

    if (iteration == iteration_end):
        plt.show()
    else:
        plt.draw()
        plt.pause(.01)
        plt.clf()
