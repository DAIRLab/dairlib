import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import time
import sys

print("WARNING: the label for y axis has to be set manually")

min_dist = 0.24
delta_dist = 0.03

iteration_start = 1
iteration_end = 34
sample_idx = 0;
state_idx_start = 0;
state_idx_end = 6;
if len(sys.argv) >= 2:
    iteration_start = int(sys.argv[1])
if len(sys.argv) >= 3:
    iteration_end = int(sys.argv[2])
if len(sys.argv) >= 4:
    sample_idx = int(sys.argv[3])
if len(sys.argv) >= 5:
    state_idx_start = int(sys.argv[4])
if len(sys.argv) >= 6:
    state_idx_end = int(sys.argv[5])

robot_option = 1;  # 0 is five-link robot. 1 is cassie_fixed_spring
# directory = 'data/robot_' + str(robot_option) + '/'
directory = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

fig = plt.figure(1)

# get time from iteration 1
t = []
iteration = 1
if os.path.isfile(directory+str(iteration)+'_'+str(sample_idx)+'_time_at_knots.csv'):
    matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_idx)+'_time_at_knots.csv', delimiter=",")
    t = matrix

for iteration in range(iteration_start,iteration_end+1):
    ax = fig.gca()
    if os.path.isfile(directory+str(iteration)+'_'+str(sample_idx)+'_state_at_knots.csv'):
        matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_idx)+'_state_at_knots.csv', delimiter=",")
        for state_idx in range(state_idx_start, state_idx_end):
            pos_or_vel = matrix[state_idx,:]
            ax.plot(t,pos_or_vel, label='q('+str(state_idx)+')')
            ax.tick_params(axis='x', labelsize=15)
            ax.tick_params(axis='y', labelsize=15)

    cost = []
    if os.path.isfile(directory+str(iteration)+'_'+str(sample_idx)+'_c.csv'):
        matrix = np.genfromtxt (directory+str(iteration)+'_'+str(sample_idx)+'_c.csv', delimiter=",")
        cost.append(matrix)

    plt.xlabel('t (s)', fontsize=15)
    plt.ylabel('q (m or rad)', fontsize=15)
    # plt.ylabel('v (m/s or rad/s)', fontsize=15)

    # plt.title('Generalized position trajectories.')
    plt.title('Iteration #'+str(iteration)+': cost = '+str(cost[0]))
    leg = plt.legend()


    # Set the axis limit
    # ax.set_xlim(0, 0.5)
    # ax.set_ylim(-0.45, 1.1)

    # Draw the figure so you can find the positon of the legend.
    plt.draw()

    # # Get the bounding box of the original legend
    # bb = leg.get_bbox_to_anchor().inverse_transformed(ax.transAxes)
    # # Change to location of the legend.
    # bb.x0 += 0
    # # bb.x1 += 0.12
    # leg.set_bbox_to_anchor(bb, transform = ax.transAxes)

    if (iteration_start == iteration_end):
        plt.show()
    else:
        plt.draw()
        plt.pause(1)
        plt.clf()
