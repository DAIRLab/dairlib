import matplotlib.pyplot as plt
import numpy as np
import os

robot_option = 1
normalized_cost = True
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
plot_large_range = 1
plot_small_range = 1

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


i = 0
x = []
y = []
z = []
dir = dir4
dir_nominal = dir3
found_same_sample = False
while os.path.isfile(dir+str(i)+'_'+str(0)+'_gamma.csv'):
    gamma = np.genfromtxt(dir + str(i) + '_' + str(0) + '_gamma.csv', delimiter=",")
    cost = float(np.genfromtxt(dir + str(i) + '_' + str(0) + '_c.csv', delimiter=","))
    if np.genfromtxt(dir + str(i) + '_' + str(0) + '_is_success.csv', delimiter=",") == 1:
        if normalized_cost:
            j = 0
            # search the nominal cost map
            while os.path.isfile(dir_nominal+str(j)+'_'+str(0)+'_gamma.csv'):
                gamma_nominal = np.genfromtxt(dir_nominal + str(j) + '_' + str(0) + '_gamma.csv', delimiter=",")
                if np.array_equal(gamma, gamma_nominal):
                    found_same_sample = True
                    if np.genfromtxt(dir_nominal + str(j) + '_' + str(0) + '_is_success.csv', delimiter=",") == 1:
                        nominal_cost = float(np.genfromtxt(dir_nominal + str(j) + '_' + str(0) + '_c.csv', delimiter=","))
                        if cost/nominal_cost < 1.5:
                            x.append(gamma[0])
                            y.append(gamma[1])
                            z.append(cost/nominal_cost)
                    break
                j = j+1
            if found_same_sample:
                found_same_sample = False
            else:
                x.append(gamma[0])
                y.append(gamma[1])
                z.append(0)
        else:
            x.append(gamma[0])
            y.append(gamma[1])
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
ax.set_ylabel('Ground incline')
if normalized_cost:
    ax.set_title('normalized cost landscape')
else:
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