import os

from numpy.core.fromnumeric import trace
import cube_sim
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import os
import json
from random import choice
from learn_cube_parameters import cube_data_folder, model_folder, log_folder
from matplotlib import pyplot as plt, use
import numpy as np
from plotting_utils import format_sim_name

mse_loss = cube_sim.LossWeights() # default weights are all ones
pos_rot_loss = cube_sim.FastLossWeights(pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)), bullet=True)
# pos_rot_loss = cube_sim.LossWeights(pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)), vel=np.zeros((3,)), omega=np.zeros((3,)))

def visualize_learned_params(params, data_sim, toss_id):
    cube_data = cube_sim.load_cube_toss(cube_sim.make_cube_toss_filename(cube_data_folder, toss_id))
    initial_state = cube_data[0].ravel()
    data_sim.init_sim(params)
    
    sim_data = data_sim.get_sim_traj_initial_state(initial_state, cube_data.shape[0], cube_sim.CUBE_DATA_DT)
    
    vis_sim = drake_cube_sim.DrakeCubeSim(visualize=True)
    vis_sim.init_sim(drake_cube_sim.default_drake_contact_params)
    # vis_sim.visualize_two_cubes_multipose(cube_data, sim_data, downsampling_rate=8)

    # input('Press enter to continue to video')

    vis_sim.visualize_two_cubes(cube_data, sim_data, 0.05)

# calculate the net contact impulse trajectory (N * s) by taking momentum
# differences between timestamps
def calculate_contact_impulse(traj):
    impulses = np.zeros((traj.shape[0]-1,2))
    for i in range(traj.shape[0]-1):
        impulses[i,0] = cube_sim.CUBE_MASS * np.linalg.norm(
            traj[i+1,cube_sim.CUBE_DATA_VELOCITY_SLICE][:2] - 
            traj[i, cube_sim.CUBE_DATA_VELOCITY_SLICE][:2]) / cube_sim.CUBE_DATA_DT
        impulses[i,1] = cube_sim.CUBE_MASS * (
            (traj[i+1,cube_sim.CUBE_DATA_VELOCITY_SLICE][-1] - 
            traj[i,cube_sim.CUBE_DATA_VELOCITY_SLICE][-1]) / cube_sim.CUBE_DATA_DT + 9.81)
    return impulses

#calculate signed distance function over the trajectory
def calculate_sdf_trajectory(traj):
    sdf = np.zeros((traj.shape[0],))
    for i in range(traj.shape[0]):
        sdf[i] = cube_sim.calc_lowest_corner_pos(traj[i])
    return sdf


def calc_damping_ratio(params):
    return params['damping'] / (2.0 * np.sqrt(params['stiffness']))

# visualize if there is action at a distance 
# by checking impulses against sdf
def plot_sdf_and_contact(traj, title=''):
    impulses = calculate_contact_impulse(traj)
    sdf = calculate_sdf_trajectory(traj) * 100.0
    times = cube_sim.CubeSim.make_traj_timestamps(traj)
    
    plt.figure()
    plt.plot(times, sdf)
    plt.step(times[1:], impulses[:,1])
    plt.legend(['sdf (mm)', 'Avg Force (N)'])


def plot_contact_impulses(traj_pair, title=''):
    data_impulses = calculate_contact_impulse(traj_pair[0])
    sim_impulses = calculate_contact_impulse(traj_pair[1])

    times = cube_sim.CubeSim.make_traj_timestamps(traj_pair[0])[1:]

    plt.figure()
    plt.step(times, data_impulses[:,0])
    plt.step(times, sim_impulses[:,0])
    plt.legend(['Data', 'Simulation'])
    plt.title(f'{title}Tangent Forces')

    plt.figure()
    plt.step(times, data_impulses[:,1])
    plt.step(times, sim_impulses[:,1])
    plt.legend(['Data', 'Simulation'])
    plt.title(f'{title}Normal Forces')
    

def load_traj_pairs(sim, params, test_set, print_progress=False):
    sim.init_sim(params)

    traj_pairs = {}
    i = 0
    for toss_id in test_set:
        cube_data = cube_sim.load_cube_toss(
            cube_sim.make_cube_toss_filename(cube_data_folder, toss_id))
        initial_state = cube_data[0]
        steps = cube_data.shape[0]
        sim_data = sim.get_sim_traj_initial_state(
            initial_state, steps, cube_sim.CUBE_DATA_DT)
        traj_pairs[toss_id] = (cube_data, sim_data)
        if (print_progress):
            if not (i % 5): print(i)
            i += 1
    
    return traj_pairs

def calc_error_between_trajectories(traj_pair):
    data_traj = traj_pair[0]     
    sim_traj = traj_pair[1]
    errors = {}
    errors['position_error'] = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_POSITION_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_POSITION_SLICE], axis=1) / (2*cube_sim.BLOCK_HALF_WIDTH)

    errors['velocity_error'] = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_VELOCITY_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_VELOCITY_SLICE], axis=1) 

    errors['omega_error'] = np.linalg.norm(
        data_traj[:,cube_sim.CUBE_DATA_OMEGA_SLICE] - \
        sim_traj[:,cube_sim.CUBE_DATA_OMEGA_SLICE], axis=1)
    
    quat_error = np.zeros((data_traj.shape[0]))

    for i in range(data_traj.shape[0]):
        quat_error[i] = cube_sim.LossWeights.calc_rotational_distance(
            data_traj[i, cube_sim.CUBE_DATA_QUATERNION_SLICE], 
            sim_traj[i, cube_sim.CUBE_DATA_QUATERNION_SLICE])
    errors['rotational_error'] = quat_error * 180 / 3.14159265

    return errors

def make_sim_to_real_comparison_plots_single_toss(traj_pair):
    data_traj = traj_pair[0]     
    tvec = cube_sim.CubeSim.make_traj_timestamps(data_traj)

    errors = calc_error_between_trajectories(traj_pair)
    
    for key in errors:
        plt.figure()
        plt.plot(tvec, errors[key])
        plt.title(key)

def calc_error_and_loss_stats(traj_pairs, loss_weights):
    pos = []
    vel = []
    omega = []
    rot = []
    loss = []

    i = 0
    for pair_idx in traj_pairs:
        pair = traj_pairs[pair_idx]
        errors = calc_error_between_trajectories(pair)
        pos.append(np.mean(errors['position_error']))
        vel.append(np.mean(errors['velocity_error']))
        omega.append(np.mean(errors['omega_error']))
        rot.append(np.mean(errors['rotational_error']))
        loss.append(loss_weights.CalculateLoss(pair[0], pair[1]))
        if not (i % 25): print(f'calculating means #{i}')
        i += 1
    
    pos_mean = np.mean(np.array(pos))
    vel_mean = np.mean(np.array(vel))
    omega_mean = np.mean(np.array(omega))
    rot_mean = np.mean(np.array(rot))
    loss_mean = np.mean(np.array(loss))

    pos_std = np.std(np.array(pos))
    vel_std = np.std(np.array(vel))
    omega_std = np.std(np.array(omega))
    rot_std = np.std(np.array(rot))
    loss_std = np.std(np.array(loss))

    pos_med = np.median(np.array(pos))
    vel_med = np.median(np.array(vel))
    omega_med = np.median(np.array(omega))
    rot_med = np.median(np.array(rot))
    loss_med = np.median(np.array(loss))


    return {'pos_mean' : pos_mean, 
            'vel_mean' : vel_mean,
            'omega_mean' : omega_mean, 
            'rot_mean' : rot_mean,
            'mse_mean' : loss_mean, 
            'pos_med' : pos_med, 
            'vel_med' : vel_med,
            'omega_med' : omega_med, 
            'rot_med' : rot_med,
            'mse_med' : loss_med, 
            'pos_std' : pos_std, 
            'vel_std' : vel_std,
            'omega_std' : omega_std, 
            'rot_std' : rot_std,
            'mse_std' : loss_std }

    
def sort_traj_pairs_by_loss(pairs, loss_weights):
    loss = {}
    for idx, pair in pairs.items():
        loss[idx] = loss_weights.CalculateLoss(pair[0], pair[1])
        
    sorted_pairs = {idx : pair for idx, pair in sorted(pairs.items(), 
        key=lambda item: loss[item[0]], reverse=True)}

    loss = {idx: loss for idx, loss in sorted(loss.items(), 
        key=lambda item : item[1], reverse=True)}

    return sorted_pairs, loss

def load_params(simulator, id):
    filename = os.path.join(model_folder, simulator + '_' + id +'.json')
    with open(filename, 'r+') as fp:
        return json.load(fp)

# load learned parameters and logging info
def load_params_and_logs(result_id):
    
    with open(os.path.join(model_folder, result_id + '.json'), 'r') as fp:
        learned_params = json.load(fp)

    logdir = os.path.join(log_folder, result_id)
    with open(os.path.join(logdir, 'test_set.json'), 'r') as fp:
        test_set = json.load(fp)
    
    with open(os.path.join(logdir, 'weights.json'), 'rb') as fp:
        loss_weights = cube_sim.LossWeights.load_weights(fp) 
    
    return learned_params, test_set, loss_weights

def compare_worst_case(result_losses):
    toss_id_lists = {}
    loss_lists = {}
    worst_case_union = []
    worst_case_by_result = {}
    # Un-nest dicts to rearrange output format
    for key in result_losses.keys():
        loss_lists[key] = list(result_losses[key].values())
        toss_id_lists[key] = list(result_losses[key].keys())
        worst_case_union = list_union(worst_case_union, toss_id_lists[key][:20]) 
        worst_case_by_result[key] = toss_id_lists[key][:20]
        num_traj = len(loss_lists[key])
        print(format_sim_name(key), end='     ')
    
    print()
    for i in range(num_traj):
        for key in result_losses.keys():
            print(f'{toss_id_lists[key][i]}, {loss_lists[key][i]}', end='\t')
        print()

    return worst_case_union, worst_case_by_result   

def list_complement(list1, list2):
    return list(set(list1) - set(list2))

def list_union(list1, list2):
    return list(set(list1) | set(list2))

def load_list_of_results(training_results, loss_to_compare, eval_all_traj=False):
    result_traj_pairs = {}
    result_losses = {}
    result_params = {}
    sims = {}
    union_of_test_sets = []
    
    for result in training_results:
        print(f'Loading logs for {result}')
        sims[result] = get_eval_sim(result)
        result_params[result], test_set, _ = load_params_and_logs(result)
        if (eval_all_traj):
            test_set = range(550)
        union_of_test_sets = list_union(union_of_test_sets, test_set)
        

    for result in training_results:
        print(f'Loading trajectories for {result}')
        traj_pairs = load_traj_pairs(sims[result], result_params[result], union_of_test_sets, print_progress=True)
        result_traj_pairs[result], result_losses[result] = sort_traj_pairs_by_loss(traj_pairs, loss_to_compare)

    return result_traj_pairs, result_losses, result_params, sims, union_of_test_sets


def get_eval_sim(result_id):
    sim_type = result_id.split('_')[0]
    substeps = int(result_id.split('_')[-1])

    if (sim_type == 'mujoco'):
        eval_sim = mujoco_cube_sim.MujocoCubeSim(substeps=substeps)
    elif (sim_type == 'drake'):
        eval_sim = drake_cube_sim.DrakeCubeSim(substeps=substeps)
    elif (sim_type == 'bullet'):
        eval_sim = bullet_cube_sim.BulletCubeSim(substeps=substeps)
    else:
        print(f'{sim_type} is not a supported simulator - please check for spelling mistakes and try again')
        eval_sim = None
    
    return eval_sim


if (__name__ == '__main__'):
    
    # ids = ['bullet_2021_08_31_16_53_100', 
    #        'bullet_2021_08_31_16_54_100',
    #        'bullet_2021_08_31_16_55_100']

    # ids = ['drake_2021_08_31_14_00_100', 
    #        'drake_2021_08_31_14_04_100',
    #        'drake_2021_08_31_17_28_100']

    # ids = ['mujoco_2021_08_31_16_17_100', 
    #        'mujoco_2021_08_31_16_18_100', 
    #        'mujoco_2021_08_31_16_21_100']
    
    # ids = ['mujoco_2021_08_31_16_17_100',
    #        'drake_2021_08_31_14_04_100', 
    #        'bullet_2021_08_31_16_55_100']

           
    ids = [
            'mujoco_2021_09_12_10_27_10', 
           'drake_2021_09_11_16_44_10',
           'bullet_2021_09_13_23_26_10']

    # ids = ['mujoco_2021_08_31_11_03_1',
    #        'mujoco_2021_08_31_12_05_10',
    #        'mujoco_2021_08_31_16_18_100']
    
    # ids = ['drake_2021_08_31_11_11_1', 
    #        'drake_2021_08_31_11_32_10', 
    #        'drake_2021_08_31_14_04_100']

    # ids = ['bullet_2021_08_31_11_12_1', 
    #        'bullet_2021_08_31_12_16_10', 
    #        'bullet_2021_08_31_16_55_100']

    # sorted_pairs, losses, params, sims, _ = load_list_of_results(ids, pos_rot_loss, eval_all_traj=True)

    # worst_case_set, worst_case_by_id = compare_worst_case(losses)
    # print()
    # for i in range(3):
    #     comp = list_complement([0, 1, 2], [i])

    #     fails = list_complement(list_complement(worst_case_set,
    #         worst_case_by_id[ids[comp[0]]]), worst_case_by_id[ids[comp[1]]])
        
    #     print(f'{format_sim_name(ids[i])} does poorly on:')
    #     for toss_id in fails:
    #         print(f'{toss_id}: {losses[ids[i]][toss_id]}, \
    #             {format_sim_name(ids[comp[0]])}: {losses[ids[comp[0]]][toss_id]}, \
    #                  {format_sim_name(ids[comp[1]])}: {losses[ids[comp[1]]][toss_id]}')

    #     print()

    # visualize_learned_params(params[ids[0]], sims[ids[0]], 69)

    # plot_sdf_and_contact(sorted_pairs[ids[0]][69][1])
    # plot_sdf_and_contact(sorted_pairs[ids[0]][69][0])
    # plt.show()


    # plot_estimated_loss_pdfs(losses)
    # plt.show()

#     ## INDIVIDUAL LOG FUNCTIONS
    learning_result = ids[2]
# # 
    eval_sim = get_eval_sim(learning_result)
    params, _, _ = load_params_and_logs(learning_result)
    # traj_pairs = load_traj_pairs(eval_sim, params, range(550))
    # weights=cube_sim.FastLossWeights(
    #         pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)))
    # sorted_pairs, losses = sort_traj_pairs_by_loss(traj_pairs, weights)
    # print('Test set sorted from highest to lowest MSE')
    # for key in sorted_pairs:
    #     print(f'Toss: {key} \t\t MSE: {losses[key]}')

    visualize_learned_params(params, eval_sim, 361) # list(sorted_pairs.keys())[0])
#     if (eval_sim == None): quit()
    # for id in ids:
    #     params, _, _ = load_params_and_logs(id)
    #     eval_sim = get_eval_sim(id)
    #     traj_pairs = load_traj_pairs(eval_sim, params, range(550))
    #     weights=cube_sim.FastLossWeights(
    #         pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)),
    #          bullet=(format_sim_name(id) == 'Bullet'))
    #     stats = calc_error_and_loss_stats(traj_pairs, weights)
    #     print()
    #     print(id) 
    #     print(stats)

#     sorted_pairs, losses = sort_traj_pairs_by_loss(traj_pairs, pos_rot_loss)
#     print('Test set sorted from highest to lowest MSE')
#     for key in sorted_pairs:
#         print(f'Toss: {key} \t\t MSE: {losses[key]}')

#     # stats = calc_error_and_loss_stats(traj_pairs, pos_rot_loss)
    # print(stats)
#     # plot_contact_impulses(sorted_pairs[list(sorted_pairs.keys())[0]])
#     # plot_sdf_and_contact(sorted_pairs[list(sorted_pairs.keys())[0]][1])
#     # plt.show()
#     visualize_learned_params(params, eval_sim, list(sorted_pairs.keys())[0])
