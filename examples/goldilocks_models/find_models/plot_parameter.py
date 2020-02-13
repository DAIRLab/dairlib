import matplotlib.pyplot as plt
import csv
import os
import time
import sys
import numpy as np

idx_start = 0
idx_end = 6
kin_or_dyn = 1
iter_start = 1
iter_spacing = 1
if len(sys.argv) >= 2:
    idx_start = int(sys.argv[1])
if len(sys.argv) >= 3:
    idx_end = int(sys.argv[2])
if len(sys.argv) >= 4:
    kin_or_dyn = int(sys.argv[3])
if len(sys.argv) >= 5:
    iter_start = int(sys.argv[4])
if len(sys.argv) >= 6:
    iter_spacing = int(sys.argv[5])


row_idx_to_be_plotted = range(idx_start,idx_end)
name = 'theta_sDDot' # theta_s or theta_sDDot
if kin_or_dyn == 0:
    name = 'theta_s'

robot_option = 1;  # 0 is five-link robot. 1 is cassie_fixed_spring
directory = 'data/robot_' + str(robot_option) + '/'


while 1:
    fig1 = plt.figure(1)
    ax1 = fig1.gca()

    for row_idx in row_idx_to_be_plotted:
        theta_i = []
        iteration = iter_start
        while os.path.isfile(directory+str(iteration)+'_'+name+'.csv'):
            matrix = np.genfromtxt (directory+str(iteration)+'_'+name+'.csv', delimiter=",")
            theta_i.append(matrix[row_idx])
            iteration+=iter_spacing
        length = len(theta_i)
        t = range(iter_start,length*iter_spacing+iter_start, iter_spacing)
        ax1.plot(t,theta_i, label=''+name+'('+str(row_idx)+')')

    plt.xlabel('iterations')
    plt.ylabel('parameter values')
    plt.title(name)
    # plt.legend()
    plt.draw()

    plt.pause(600)
    plt.clf()
