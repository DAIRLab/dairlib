import matplotlib.pyplot as plt
import numpy as np
import os

# setting of program
robot_option = 1
plot_successful_sample = 1

if robot_option == 0:
    robot = 'five_link/'
else:
    if robot_option == 1:
        robot = 'cassie/'

# plot setting
rom1 = '1D_LIP'
rom2 = '2D_LIP'
rom3 = '3D_LIP'
task_space1 = '2D_task_space'
task_space2 = '3D_task_space'
task_space3 = '4D_task_space'


file_dir = "../dairlib_data/goldilocks_models/find_models/"

rom = rom3
task_space = task_space3
method ='(nongrid)'
iter_start = 2
iter_end = 200
samples_number = 36
directory = file_dir+robot+'robot_1/'
label_name = ['stride_length', 'ground_incline', 'velocity', 'turning_rate']


def plot_task():
    task = np.zeros([(iter_end-iter_start+1)*samples_number, 4])
    for i in range(iter_start, iter_end+1):
        for j in range(samples_number):
            task[(i-iter_start)*samples_number+j, :] = np.genfromtxt(directory+str(i)+'_'+str(j)+'_task.csv', delimiter=",")
    axs[0, 0].scatter(range(task.shape[0]), task[:, 0], label=label_name[0])
    axs[0, 0].legend()
    axs[0, 1].scatter(range(task.shape[0]), task[:, 1], label=label_name[1])
    axs[0, 1].legend()
    axs[1, 0].scatter(range(task.shape[0]), task[:, 2], label=label_name[2])
    axs[1, 0].legend()
    axs[1, 1].scatter(range(task.shape[0]), task[:, 3], label=label_name[3])
    axs[1, 1].legend()
    # number = 0
    # for i in range(task.shape[0]):
    #     init = task[i, :]
    #     for j in range(task.shape[0]):
    #         second = task[j, :]
    #         if j!=i and np.allclose(init, second):
    #             number += 1
    # print(number)




fig, axs = plt.subplots(2,2)
# figsize=(6.4, 4.8))
plot_task()
plt.xlabel('Samples')
plt.ylabel('Task')
plt.show()
