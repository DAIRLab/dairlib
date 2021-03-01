import numpy as np
import csv
import os


directory = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

file_name = 'solve_time.csv'

N_sample = 48
i=0
total_time=0
while os.path.isfile(directory+str(0)+'_'+str(i)+'_'+file_name):
    total_time = total_time+float(np.genfromtxt(directory+str(0)+'_'+str(i)+'_'+file_name, delimiter=","))

average_cost = total_time/(i+1)
print(average_cost)