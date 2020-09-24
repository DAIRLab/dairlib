"""
To run this script, you need to 
1. first build this file to get dependency (we are using dairlib's python bindings)
        `bazel build examples/goldilocks_models/find_models:plot_q`
2. run 
        `bazel-bin/examples/goldilocks_models/find_models/plot_q` 
    instead of
        `bazel run examples/goldilocks_models/find_models:plot_q`
    because we are using relative data paths in this script.
"""

import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import time
import sys
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydairlib.common import FindResourceOrThrow
import pydairlib.multibody

# User parameters
iteration_start = 1
iteration_end = 1
sample_idx = 0;
state_idx_start = 7;
state_idx_end = 19;  # doesn't include
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

# Paths
# data_path = 'data/robot_' + str(robot_option) + '/'
data_path = '../dairlib_data/goldilocks_models/find_models/robot_' + str(robot_option) + '/'

# Build MBP
builder = DiagramBuilder()
plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
# Parser(plant).AddModelFromFile(FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"))
Parser(plant).AddModelFromFile(FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"))
plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
plant.Finalize()


# import pdb; pdb.set_trace()

# MBP params
nq = plant.num_positions()
nv = plant.num_velocities()
nx = plant.num_positions() + plant.num_velocities()
nu = plant.num_actuators()

# Maps
pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

inv_pos_map = {value : key for (key, value) in pos_map.items()}
inv_vel_map = {value : key for (key, value) in vel_map.items()}
inv_act_map = {value : key for (key, value) in act_map.items()}

inv_state_map = inv_pos_map
for (key, value) in vel_map.items():
    inv_state_map[key] = value + nq

# Plots
fig = plt.figure(1)

# get time from iteration 1
t = []
iteration = 1
if os.path.isfile(data_path+str(iteration)+'_'+str(sample_idx)+'_time_at_knots.csv'):
    matrix = np.genfromtxt (data_path+str(iteration)+'_'+str(sample_idx)+'_time_at_knots.csv', delimiter=",")
    t = matrix

for iteration in range(iteration_start,iteration_end+1):
    ax = fig.gca()
    if os.path.isfile(data_path+str(iteration)+'_'+str(sample_idx)+'_state_at_knots.csv'):
        matrix = np.genfromtxt (data_path+str(iteration)+'_'+str(sample_idx)+'_state_at_knots.csv', delimiter=",")
        for state_idx in range(state_idx_start, state_idx_end):
            state = matrix[state_idx,:]
            ax.plot(t, state, label=inv_state_map[state_idx])
            ax.tick_params(axis='x', labelsize=15)
            ax.tick_params(axis='y', labelsize=15)

    cost = []
    if os.path.isfile(data_path+str(iteration)+'_'+str(sample_idx)+'_c.csv'):
        matrix = np.genfromtxt (data_path+str(iteration)+'_'+str(sample_idx)+'_c.csv', delimiter=",")
        cost.append(matrix)

    plt.xlabel('t (s)', fontsize=15)
    plt.ylabel('state', fontsize=15)
    # plt.ylabel('q (m or rad)', fontsize=15)
    # plt.ylabel('v (m/s or rad/s)', fontsize=15)

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
