import numpy as np
import csv
import os

robot_option=1
directory = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

file_name = 'solve_time.csv'

N_sample = 48
i=0
total_time=0
while os.path.isfile(directory+str(0)+'_'+str(i)+'_'+file_name):
    total_time = total_time+float(np.genfromtxt(directory+str(0)+'_'+str(i)+'_'+file_name, delimiter=","))
    i=i+1
average_cost = total_time/i
print(average_cost)