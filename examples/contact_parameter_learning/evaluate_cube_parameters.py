import os
import cube_sim
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import os
import sys
import json
from learn_cube_parameters import cube_data_folder, model_folder, log_folder
from matplotlib import pyplot as plt
import numpy as np

def visualize_learned_params(params, sim, toss_id):
    cube_data = cube_sim.load_cube_toss(cube_sim.make_cube_toss_filename(cube_data_folder, toss_id))
    initial_state = cube_data[0].ravel()

    vis_sim = drake_cube_sim.DrakeCubeSim(visualize=True)

    if (sim == 'mujoco'):
        data_sim = mujoco_cube_sim.MujocoCubeSim()
    elif (sim == 'drake'):
        data_sim = drake_cube_sim.DrakeCubeSim()
    elif (sim == 'bullet'):
        data_sim = bullet_cube_sim.BulletCubeSim()

    data_sim.init_sim(params)
    sim_data = data_sim.get_sim_traj_initial_state(initial_state, cube_data.shape[0], cube_sim.CUBE_DATA_DT)

    vis_sim.visualize_two_cubes(cube_data, sim_data, 0.2)

def load_params(simulator, id):
    filename = os.path.join(model_folder, simulator + '_' + id +'.json')
    with open(filename, 'r+') as fp:
        return json.load(fp)

def make_sim_to_real_comparison_plots(sim, params, data_folder, toss_id):
    data_traj = cube_sim.load_cube_toss(cube_sim.make_cube_toss_filename(data_folder, toss_id))

    sim.init_sim(params)        
    sim_traj = sim.get_sim_traj_initial_state(data_traj[0], data_traj.shape[0], cube_sim.CUBE_DATA_DT)
    tvec = sim.make_traj_timestamps(data_traj)

    position_error = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_POSITION_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_POSITION_SLICE], axis=1)
    position_error /= cube_sim.BLOCK_HALF_WIDTH

    vel_error = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_VELOCITY_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_VELOCITY_SLICE], axis=1)
    vel_error /= cube_sim.BLOCK_HALF_WIDTH
    
    omega_error = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_OMEGA_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_OMEGA_SLICE], axis=1)
    
    quat_error = np.zeros((data_traj.shape[0]))
    for i in range(data_traj.shape[0]):
        quat_error[i] = cube_sim.LossWeights.calc_rotational_distance(
            data_traj[i, cube_sim.CUBE_DATA_QUATERNION_SLICE], 
            data_traj[i, cube_sim.CUBE_DATA_QUATERNION_SLICE])
    

if (__name__ == '__main__'):
    pass