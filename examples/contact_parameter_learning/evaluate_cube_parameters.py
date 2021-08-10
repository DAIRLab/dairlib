import os
import cube_sim
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import os
import sys
import json
from learn_cube_parameters import cube_data_folder, model_folder, log_folder

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


if (__name__ == '__main__'):
    pass