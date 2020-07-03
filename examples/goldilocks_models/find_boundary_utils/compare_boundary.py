"""
This function is used for comparing two cost landscape and plot the landscape with discrete color map.
1.For every point in cost landscape 1 (C1), we check if this point exists in cost landscape 2 (C2):
If it exists in C2, we compare the costs between them and set the corresponding value for this point.
Else, we set the value zero for this point.
2.For every point in C2, we do the same thing while we ignore the intersected point, and we set a large
value for point existing in C2 rather than in C1.
"""
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import os

robot_option = 1
file_dir = '/Users/jason-hu/'

# optimization setting
if robot_option == 0:
    min_gi1 = -0.325
    max_gi1 = 0.275
    min_sl1 = 0.0925
    max_sl1 = 0.3925
else:
    min_gi1 = -0.325
    max_gi1 = 0.275
    min_sl1 = 0.1425
    max_sl1 = 0.4425
plot_large_range = 1

if robot_option == 0:
    min_gi2 = -0.175
    max_gi2 = 0.125
    min_sl2 = 0.1675
    max_sl2 = 0.3175
else:
    min_gi2 = -0.175
    max_gi2 = 0.125
    min_sl2 = 0.2175
    max_sl2 = 0.3675
plot_small_range = 1

if robot_option == 0:
    robot = 'five_link/'
    dir1 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_nominal/'
    dir2 = file_dir+'dairlib_data/find_boundary/' + robot + 'robot_' + str(robot_option) + \
           '_initial_model/'
    dir3 = file_dir+'dairlib_data/find_boundary/' + robot + 'large_task_space/' + 'robot_' + str(robot_option) + \
           '_large_task_space_iter1000/'
    dir4 = file_dir+'dairlib_data/find_boundary/' + robot + 'large_task_space/' + 'robot_' + str(robot_option) + \
           '_large_task_space_iter2000/'
    dir5 = file_dir+'dairlib_data/find_boundary/' + robot + 'large_task_space/' + 'robot_' + str(robot_option) + \
           '_large_task_space_iter3000_with_scaling/'
    dir6 = file_dir+'dairlib_data/find_boundary/' + robot + 'small_task_space/' + 'robot_' + str(robot_option) + \
           '_small_task_space_iter500/'
    dir7 = file_dir+'dairlib_data/find_boundary/' + robot + 'small_task_space/' + 'robot_' + str(robot_option) + \
           '_small_task_space_iter1000/'
    dir8 = file_dir+'dairlib_data/find_boundary/' + robot + 'small_task_space/' + 'robot_' + str(robot_option) + \
           '_small_task_space_iter2000_with_scaling/'
if robot_option == 1:
    robot = 'cassie/'
    dir1 = file_dir+'dairlib_data/find_boundary/' + robot + '1D_rom/2D_task_space/'+'robot_' + str(robot_option) + \
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
    dir7 = file_dir + 'dairlib_data/find_boundary/' + robot + '2D_rom/2D_task_space/' + 'robot_' + str(robot_option) + \
           '_large_iter100/'


x = []
y = []
z = []
dir = dir4
dir_compare = dir3
i = 0
found_same_sample = False
while os.path.isfile(dir+str(i)+'_'+str(0)+'_gamma.csv'):
    gamma = np.genfromtxt(dir + str(i) + '_' + str(0) + '_gamma.csv', delimiter=",")
    cost = float(np.genfromtxt(dir + str(i) + '_' + str(0) + '_c.csv', delimiter=","))
    j = 0
    # search if the intersected samples
    while os.path.isfile(dir_compare+str(j)+'_'+str(0)+'_gamma.csv'):
        gamma_compare = np.genfromtxt(dir_compare + str(j) + '_' + str(0) + '_gamma.csv', delimiter=",")
        if np.array_equal(gamma, gamma_compare):
            found_same_sample = True
            # if np.genfromtxt(dir + str(i) + '_' + str(0) + '_is_success.csv', delimiter=",") == 1:
            #     if np.genfromtxt(dir_compare + str(j) + '_' + str(0) + '_is_success.csv', delimiter=",") == 1:
            compare_cost = float(np.genfromtxt(dir_compare + str(j) + '_' + str(0) + '_c.csv', delimiter=","))
            x.append(gamma[0])
            y.append(gamma[1])
            if cost > compare_cost:
                z.append(1.5)
            else:
                z.append(0.5)
            break
        j = j+1
    if found_same_sample:
        found_same_sample = False
    else:
        x.append(gamma[0])
        y.append(gamma[1])
        z.append(-1)
    i = i+1
# search another side
i = 0
while os.path.isfile(dir_compare+str(i)+'_'+str(0)+'_gamma.csv'):
    gamma_compare = np.genfromtxt(dir_compare + str(i) + '_' + str(0) + '_gamma.csv', delimiter=",")
    j = 0
    # search if the intersected samples
    while os.path.isfile(dir+str(j)+'_'+str(0)+'_gamma.csv'):
        gamma = np.genfromtxt(dir + str(j) + '_' + str(0) + '_gamma.csv', delimiter=",")
        if np.array_equal(gamma, gamma_compare):
            found_same_sample = True
            break
        j = j+1
    if found_same_sample:
        found_same_sample = False
    else:
        x.append(gamma_compare[0])
        y.append(gamma_compare[1])
        z.append(1.5)
    i = i+1


print(np.shape(x))
print(np.shape(y))
print(np.shape(z))

fig, ax = plt.subplots()

# discrete color map

levels = [0, 1, 2]
colors = ['green', 'blue']
cmap, norm = matplotlib.colors.from_levels_and_colors(levels, colors)
cmap.set_over('yellow')
cmap.set_under('red')
surf = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, levels=levels, extend='both')
cbar = fig.colorbar(surf, shrink=0.5, aspect=6, extend='both')
cbar.ax.set_yticklabels(['0', '1', 'Infinity'])

ax.set_xlabel('Stride length')
ax.set_ylabel('Ground incline')
ax.set_title('Compare two cost landscapes')

# optimization range
# large range
if plot_large_range:
    x_line = np.array([min_sl1, max_sl1, max_sl1, min_sl1, min_sl1])
    y_line = np.array([min_gi1, min_gi1, max_gi1, max_gi1, min_gi1])
    ax.plot(x_line, y_line, 'black', linewidth=3)
# small range
if plot_small_range:
    x_line2 = np.array([min_sl2, max_sl2, max_sl2, min_sl2, min_sl2])
    y_line2 = np.array([min_gi2, min_gi2, max_gi2, max_gi2, min_gi2])
    ax.plot(x_line2, y_line2, 'black', linewidth=3)

plt.show()