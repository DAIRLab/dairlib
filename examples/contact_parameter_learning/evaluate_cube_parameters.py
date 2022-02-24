import os
from numpy.core.fromnumeric import trace
from learn_cube_parameters import cube_data_folder, model_folder, log_folder
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import cube_sim
import os
import json
from random import choice

from matplotlib import pyplot as plt, use
import numpy as np
from plotting_utils import format_sim_name

mse_loss = cube_sim.LossWeights() # default weights are all ones
pos_rot_loss = cube_sim.FastLossWeights(
    pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)), bullet=True)

def visualize_learned_params(params, data_sim, toss_id):
    cube_data = cube_sim.load_cube_toss(cube_sim.make_cube_toss_filename(cube_data_folder, toss_id))
    initial_state = cube_data[0].ravel()
    data_sim.init_sim(params)
    
    sim_data = data_sim.get_sim_traj_initial_state(initial_state, cube_data.shape[0], cube_sim.CUBE_DATA_DT)
    
    vis_sim = drake_cube_sim.DrakeCubeSim(visualize=True)
    vis_sim.init_sim(drake_cube_sim.default_drake_contact_params)
    # vis_sim.visualize_two_cubes_multipose(cube_data, sim_data, downsampling_rate=8)

    # input('Press enter to continue to video')

    vis_sim.visualize_two_cubes(cube_data, sim_data, 0.25)

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
    impulses = calculate_contact_impulse(traj) / (0.37*9.81)
    sdf = calculate_sdf_trajectory(traj)
    sdf *= 1000.0
    times = cube_sim.CubeSim.make_traj_timestamps(traj)
    
    plt.figure()
    plt.plot(times, sdf)
    plt.step(times[1:], impulses[:,0])
    plt.legend(['sdf (mm)', 'Avg Force (% Cube Weight)'])


def get_energy_trajectory(traj):
    E = np.zeros((traj.shape[0],))
    for i in range(E.shape[0]):
        T_v = 0.5 * cube_sim.CUBE_MASS * np.dot(
            traj[i, cube_sim.CUBE_DATA_VELOCITY_SLICE],
            traj[i, cube_sim.CUBE_DATA_VELOCITY_SLICE])
        T_rot = 0.5 * 0.00081 * np.dot(
            traj[i, cube_sim.CUBE_DATA_OMEGA_SLICE],
            traj[i, cube_sim.CUBE_DATA_OMEGA_SLICE])
        V = cube_sim.CUBE_MASS * 9.81 * (
            traj[i, cube_sim.CUBE_DATA_POSITION_SLICE.stop-1] -
            cube_sim.BLOCK_HALF_WIDTH)
        E[i] = T_v + T_rot + V

    return E


def plot_penetration_vs_error(traj_pairs, loss):
    err = np.zeros((len(traj_pairs),))
    apex_com_real = np.zeros((len(traj_pairs),))
    apex_com_sim = np.zeros((len(traj_pairs),))
    p_i = np.zeros((len(traj_pairs),))
    for i in range(len(traj_pairs)):
        sdf = calculate_sdf_trajectory(traj_pairs[i][0])
        start = np.argwhere(sdf <= 0)[0, 0]
        sdf = sdf[start:]
        sdf_sim = calculate_sdf_trajectory(traj_pairs[i][1])
        start = np.argwhere(sdf_sim <= 0)[0, 0]
        sdf_sim = sdf_sim[start:]
        err[i] = loss.CalculateLoss(traj_pairs[i][0], traj_pairs[i][1])
        apex_t = np.argmax(sdf)
        apex_t_sim = np.argmax(sdf_sim)
        apex_com_real[i] = sdf[apex_t] #traj_pairs[i][0][apex_t, cube_sim.CUBE_DATA_POSITION_SLICE.stop-1]
        apex_com_sim[i] = sdf_sim[apex_t_sim] #traj_pairs[i][1][apex_t, cube_sim.CUBE_DATA_POSITION_SLICE.stop-1]
        p_i[i] = sdf[0]

    fail_idx = np.argwhere(err > 0.5)
    success_idx = np.argwhere(err < 0.5)

    plt.hist(apex_com_real, bins=50, alpha=0.5, label="Real Cube Toss")
    plt.hist(apex_com_sim, bins=50, alpha=0.5, label="Sim Cube Toss")
    plt.legend()
    plt.xlabel('maximum bounce height (table to cube corner, m)')
    plt.ylabel('Count')
    plt.title('Drake')
    # print(f'N fail:{fail_idx.shape[0]}')
    # print(f'fail med: {np.median(apex_com_real[fail_idx])},'
    #       f' success_med: {np.median(apex_com_real[success_idx])}')


def plot_omega_vs_err(traj_pairs, losses):
    omega = np.zeros((len(traj_pairs),))
    loss = []
    for i in range(omega.shape[0]):
        omega[i] = np.linalg.norm(
            traj_pairs[i][0][0, cube_sim.CUBE_DATA_OMEGA_SLICE])
        loss.append(losses[i])

    plt.scatter(omega, loss)


def plot_sliding_vel_vs_error(traj_pairs, loss):
    err = np.zeros((len(traj_pairs),))
    vel = np.zeros((len(traj_pairs),))
    for i in range(len(traj_pairs)):
        err[i] = loss.CalculateLoss(traj_pairs[i][0], traj_pairs[i][1])
        vel[i] = np.max(np.linalg.norm(
            traj_pairs[i][1][:, cube_sim.CUBE_DATA_VELOCITY_SLICE][:,:2], axis=1))
    plt.scatter(vel, err)


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

    
def sort_traj_pairs_by_loss(pairs, loss_weights, include_outliers=True):
    loss = {}

    for idx, pair in pairs.items():
        loss[idx] = loss_weights.CalculateLoss(pair[0], pair[1])

    sorted_pairs = {}
    if include_outliers:
        sorted_pairs = {idx : pair for idx, pair in sorted(pairs.items(),
            key=lambda item: loss[item[0]], reverse=True)}

        loss = {idx: loss for idx, loss in sorted(loss.items(),
            key=lambda item : item[1], reverse=True)}
    else:
        loss = {idx: loss for idx, loss in sorted(loss.items(),
            key=lambda item: item[1], reverse=True)[100:]}

        sorted_pairs = {idx: pairs[idx] for idx in loss}

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
        worst_case_union = list_union(worst_case_union, toss_id_lists[key][:50])
        worst_case_by_result[key] = toss_id_lists[key][:50]
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
        if format_sim_name(result) == 'MuJoCo':
            result_params[result]['stiffness'] *= 3
            result_params[result]['damping'] *= 3
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
        print(f'{sim_type} is not a supported simulator - '
              f'please check for spelling mistakes and try again')
        eval_sim = None
    
    return eval_sim


'''
MuJoCo does poorly on:
98: 1.716660188946459, Drake: 0.09156737726210927,  Bullet: 0.08788414835125526
131: 1.70630226926509, Drake: 0.002824340702355384, Bullet: 0.006856337891091856
69: 1.852155781179135, Drake: 0.034522080477405376, Bullet: 0.04861447755645776
40: 1.753181670344446, Drake: 0.0390318811175085,   Bullet: 0.030108881166298115
41: 1.772070828274563, Drake: 0.09489551415782743,  Bullet: 0.09992052713587063
424: 1.65708407838393, Drake: 0.004789650031465638, Bullet: 0.021473635831862514
74: 1.834700963188898, Drake: 0.07011218322986842,  Bullet: 0.06801649169201493
428: 1.89230347762956, Drake: 0.013299232835989206, Bullet: 0.014990523159786526
13: 1.678057795492889, Drake: 1.147878016063214,    Bullet: 1.5111092761444016
204: 1.93138896322582, Drake: 0.18843052278183464,  Bullet: 0.015637716905955806
361: 2.11957152033531, Drake: 0.08129174910387611,  Bullet: 0.08089075668020973
181: 1.80547033854560, Drake: 0.039653810290791555, Bullet: 0.008131049097260291
118: 1.67463106413349, Drake: 0.02274910139478128,  Bullet: 0.02369505796822118
471: 1.65733546143384, Drake: 0.020100468256805674, Bullet: 0.021532889326593252
88: 2.077534422678576, Drake: 1.4196046327396925,   Bullet: 1.429125869777734
250: 1.68928686946214, Drake: 0.004517231431237634, Bullet: 0.018244078880291222
443: 1.76718433018395, Drake: 1.346168545431131,    Bullet: 1.4754644350814576
'''

if __name__ == '__main__':

    ids = ['drake_2022_02_21_15_19_10',
           'mujoco_2022_02_23_15_29_10',
           'bullet_2022_02_22_00_36_10']
    # muj_ids = [
    #     'mujoco_2022_02_24_12_58_10',
    #     'mujoco_2022_02_24_12_59_10',
    #     'mujoco_2022_02_24_13_00_10',
    #     'mujoco_2022_02_24_13_07_10'
    # ]
    #
    # bullet_ids = ['bullet_2022_02_17_16_30_10',
    #               'bullet_2022_02_17_19_31_10',
    #               'bullet_2022_02_17_22_48_10',
    #               'bullet_2022_02_18_02_00_10',
    #               'bullet_2022_02_18_16_59_10',
    #               'bullet_2022_02_18_20_09_10',
    #               'bullet_2022_02_18_23_31_10',
    #               'bullet_2022_02_19_02_54_10',
    #               'bullet_2022_02_21_15_05_10',
    #               'bullet_2022_02_21_18_03_10',
    #               'bullet_2022_02_21_21_22_10',
    #               'bullet_2022_02_22_00_36_10',
    #               'bullet_2022_02_23_00_48_10',
    #               'bullet_2022_02_23_07_00_10',]

    sorted_pairs, losses, params, sims, _ = \
        load_list_of_results(ids, pos_rot_loss, eval_all_traj=True)

    # import pdb; pdb.set_trace()
    # for id in bullet_ids:
    #     print(f'id: {id}, loss: {np.average([losses[id][idx] for idx in losses[id]])}')

    # import pdb; pdb.set_trace()
    # fail_idxs = list(sorted_pairs[ids[1]].keys())[:75]
    # succes_idxs = list(sorted_pairs[ids[1]].keys())[100:]

    # for idx in succes_idxs:
    #     plt.plot(get_energy_trajectory(sorted_pairs[ids[1]][idx][1]) -
    #              get_energy_trajectory(sorted_pairs[ids[0]][idx][1]))

    # worst_case_set, worst_case_by_id = compare_worst_case(losses)
    # print()
    # for i in range(3):
    #     comp = list_complement([0, 1, 2], [i])
    #
    #     fails = list_complement(list_complement(worst_case_set,
    #         worst_case_by_id[ids[comp[0]]]), worst_case_by_id[ids[comp[1]]])
    #
    #     print(f'\n{format_sim_name(ids[i])} does poorly on:')
    #     for toss_id in fails:
    #         print(f'{toss_id}: {losses[ids[i]][toss_id]}, \
    #             {format_sim_name(ids[comp[0]])}: {losses[ids[comp[0]]][toss_id]}, \
    #                  {format_sim_name(ids[comp[1]])}: {losses[ids[comp[1]]][toss_id]}')

    #     print()

    # visualize_learned_params(params[ids[0]], sims[ids[0]], 69)
    # plot_penetration_vs_error(sorted_pairs[ids[2]], pos_rot_loss)
    # plot_omega_vs_err(sorted_pairs[ids[0]], losses[ids[0]])
    #
    # plot_sdf_and_contact(sorted_pairs[ids[0]][451][1])
    # plot_sdf_and_contact(sorted_pairs[ids[1]][451][1])
    # plot_sdf_and_contact(sorted_pairs[ids[2]][451][1])
    # plt.show()


    # plot_estimated_loss_pdfs(losses)
    # plt.show()

#     ## INDIVIDUAL LOG FUNCTIONS
#     learning_result = ids[2]
# # 
#     eval_sim = get_eval_sim(learning_result)
    # params, _, _ = load_params_and_logs(learning_result)
    # traj_pairs = load_traj_pairs(eval_sim, params, range(550))
    # weights=cube_sim.FastLossWeights(
    #         pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)))
    # sorted_pairs, losses = sort_traj_pairs_by_loss(traj_pairs, weights)
    # print('Test set sorted from highest to lowest MSE')
    # for key in sorted_pairs:
    #     print(f'Toss: {key} \t\t MSE: {losses[key]}')

    # visualize_learned_params(params, eval_sim, 361) # list(sorted_pairs.keys())[0])z
#     if (eval_sim == None): quit()
    for id in ids:
        # params, _, _ = load_params_and_logs(id)
        # eval_sim = get_eval_sim(id)
        # traj_pairs = load_traj_pairs(eval_sim, params, range(550))
        weights=cube_sim.FastLossWeights(
            pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)),
             bullet=(format_sim_name(id) == 'Bullet'))
        stats = calc_error_and_loss_stats(sorted_pairs[id], weights)
        print()
        print(id)
        print(stats)

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
