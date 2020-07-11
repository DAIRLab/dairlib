import matplotlib.pyplot as plt
import numpy as np
import os

robot_option = 1
file_dir = '/Users/jason-hu/'

# optimization setting
# large range
min_gi1 = -0.325
max_gi1 = 0.275
min_sl1 = 0.0925
max_sl1 = 0.3925
# small range
min_gi2 = -0.175
max_gi2 = 0.125
min_sl2 = 0.1675
max_sl2 = 0.3175
plot_large_range = 0
plot_small_range = 0

if robot_option == 0:
    robot = 'five_link/'
    dir1 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_nominal/'
    dir2 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_initial_model/'
    dir3 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_large_task_space_iter1000/'
    dir4 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_large_task_space_iter2000/'
    dir5 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_large_task_space_iter3000_with_scaling/'
    dir6 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_small_task_space_iter500/'
    dir7 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_small_task_space_iter1000/'
    dir8 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_small_task_space_iter2000_with_scaling/'
if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir+'dairlib_data/find_boundary/' + robot + '1D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_2d_nominal/'
    dir2 = file_dir+'dairlib_data/find_boundary/' + robot + '1D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_2d_initial_model/'
    dir3 = file_dir+'dairlib_data/find_boundary/' + robot + '1D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_small_iter300/'
    dir4 = file_dir + 'dairlib_data/find_boundary/' + robot + '1D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_large_iter150/'
    dir5 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_2d_initial_model/'
    dir6 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_small_iter200/'
    dir7 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_large_iter100/'
    dir8 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_grid_iter50_sl_tr/'
    dir9 = file_dir+'dairlib_data/find_boundary/' + robot + '2D_rom/4D_task_space/' + 'robot_' + str(robot_option) + \
           '_nominal_sl_tr/'


i = 0
x = []
y = []
z = []
dir = dir9
while os.path.isfile(dir+str(i)+'_'+str(0)+'_task.csv'):
    gamma = np.genfromtxt(dir + str(i) + '_' + str(0) + '_task.csv', delimiter=",")
    cost = float(np.genfromtxt(dir + str(i) + '_' + str(0) + '_c.csv', delimiter=","))
    # only consider reasonable point
    if cost < 35:
        x.append(gamma[0])
        y.append(gamma[3])
        z.append(cost)
    i = i+1

fig, ax = plt.subplots()

print(np.shape(x))
print(np.shape(y))
print(np.shape(z))

# continuous color map
surf = ax.tricontourf(x, y, z)
fig.colorbar(surf, shrink=0.5, aspect=6)
ax.set_xlabel('Stride length')
ax.set_ylabel('Turning rate')
ax.set_title('cost landscape')

# optimization range
# large range
if plot_large_range == 1:
    x_line = np.array([min_sl1, max_sl1, max_sl1, min_sl1, min_sl1])
    y_line = np.array([min_gi1, min_gi1, max_gi1, max_gi1, min_gi1])
    ax.plot(x_line, y_line, 'black', linewidth=3)
# small range
if plot_small_range == 1:
    x_line2 = np.array([min_sl2, max_sl2, max_sl2, min_sl2, min_sl2])
    y_line2 = np.array([min_gi2, min_gi2, max_gi2, max_gi2, min_gi2])
    ax.plot(x_line2, y_line2, 'black', linewidth=3)

plt.show()