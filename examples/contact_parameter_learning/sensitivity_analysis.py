from cube_sim import LossWeights
import numpy as np
import sys
import matplotlib.pyplot as plt
from evaluate_cube_parameters import calc_error_between_trajectories, \
    load_traj_pairs, load_params_and_logs, get_eval_sim
from copy import deepcopy
from math import pi
import cube_sim
# import drake_cassie_sim
# import mujoco_cassie_sim


def get_cube_position_and_rotation_error_sensitivity(sim, optimal_params, params_range, traj_set):
    pos_avgs = {}
    pos_meds = {}
    rot_avgs = {}
    rot_meds = {}

    for param_key in params_range:
        pos_avg = []
        pos_med = []
        rot_avg = []
        rot_med = []
        params = deepcopy(optimal_params)
        print(f'sweeping {param_key}')

        for param_val in params_range[param_key]:
            pos = []
            rot = []
            assert param_key in params.keys()
            params[param_key] = param_val
            print(f'{param_key}: {param_val}')
            pairs = load_traj_pairs(sim, params, traj_set, print_progress=False)
            for pair in pairs.values():
                pos_error = np.linalg.norm(
                pair[0][:,cube_sim.CUBE_DATA_POSITION_SLICE] - \
                pair[1][:,cube_sim.CUBE_DATA_POSITION_SLICE], axis=1) / (2*cube_sim.BLOCK_HALF_WIDTH)
                pos.append(np.mean(pos_error))

                quat_error = np.zeros((pair[0].shape[0]))

                for i in range(pair[0].shape[0]):
                    quat_error[i] = cube_sim.LossWeights.calc_rotational_distance(
                    pair[0][i, cube_sim.CUBE_DATA_QUATERNION_SLICE], 
                    pair[1][i, cube_sim.CUBE_DATA_QUATERNION_SLICE])
                rot.append((180 / pi) * np.mean(quat_error))
            pos_avg.append(np.mean(pos))
            pos_med.append(np.median(pos))
            rot_avg.append(np.mean(rot))
            rot_med.append(np.median(rot))
        
        pos_avgs[param_key] = np.array(pos_avg)
        pos_meds[param_key] = np.array(pos_med)
        rot_avgs[param_key] = np.array(rot_avg)
        rot_meds[param_key] = np.array(rot_med)

    return pos_avgs, pos_meds, rot_avgs, rot_meds


def get_sensitivity_analysis(sim, loss_weights, optimal_params, params_range, test_traj_set, plant='cube'):
    
    loss_avgs = {}
    loss_meds ={}
    for param_key in params_range:
        loss_avg = []
        loss_med = []
        params = deepcopy(optimal_params)
        print(f'sweeping {param_key}')

        for param_val in params_range[param_key]:
            losses = []
            params[param_key] = param_val
            print(f'{param_key}: {param_val}')
            if (plant == 'cube'):
                pairs = load_traj_pairs(sim, params, test_traj_set,
                                        print_progress=False)
                for _, pair in pairs.items():
                    losses.append(loss_weights.CalculateLoss(pair[0], pair[1]))
                loss_avg.append(np.average(losses))
                loss_med.append(np.median(losses))
            else:
                for log_num in sim.log_nums_real:
                    sim_id = sim.run(params, log_num)
                    loss = sim.compute_loss(log_num, sim_id, params, plot=False)
                    losses.append(loss)
                loss_avg.append(np.average(losses))
                loss_med.append(np.median(losses))
        loss_avgs[param_key] = np.array(loss_avg)
        loss_meds[param_key] = np.array(loss_med)

    return loss_avgs, loss_meds


# So far only supports cube
def get_gridded_stiffness_damping_sensitivity_analysis(
    sim, loss_weights, optimal_params, params_ranges, test_traj_set,
        plant='cube'):
    assert('stiffness' in params_ranges)
    assert('damping' in params_ranges or 'dissipation' in params_ranges)

    damp_key = 'damping' if 'damping' in params_ranges else 'dissipation'
    stiffness, damping = \
        np.meshgrid(params_ranges['stiffness'], params_ranges[damp_key])
    loss_avgs = np.zeros(stiffness.shape)
    loss_meds = np.zeros(stiffness.shape)
    for i in range(stiffness.shape[0]):
        for j in range(stiffness.shape[1]):
            losses = []
            params = deepcopy(optimal_params)
            params['stiffness'] = stiffness[i, j]
            params[damp_key] = damping[i, j]
            pairs = load_traj_pairs(sim, params, test_traj_set,
                                    print_progress=True)
            for _, pair in pairs.items():
                losses.append(loss_weights.CalculateLoss(pair[0], pair[1]))
            loss_avgs[i, j] = np.average(losses)
            loss_meds[i, j] = np.median(losses)

    return {'stiffness': stiffness, damp_key: damping,
            'loss_avgs': loss_avgs, 'loss_mdes': loss_meds}


# for bullet / mujoco only
def get_dr_sensitivity_analysis(sim, loss_weights, optimal_params, params_range, test_traj_set):
    loss_avgs = []
    loss_meds = []
    params = deepcopy(optimal_params)
    for i in range(len(params_range['stiffness'])):
        losses = []
        params['stiffness'] = params_range['stiffness'][i]
        params['damping'] = params_range['damping'][i]
        pairs = load_traj_pairs(sim, params, test_traj_set, print_progress=False)
        for pair_idx in pairs:
            pair = pairs[pair_idx]
            losses.append(loss_weights.CalculateLoss(pair[0], pair[1]))
        loss_avgs.append(np.average(losses))
        loss_meds.append(np.median(losses))

    return {'stiffness': loss_avgs}, {'stiffness': loss_meds}


def get_damping_ratio_range(sim_type, k0, b0):
    params_range = {}
    mass = 0.37

    zeta = b0 / (2 * np.sqrt(k0))
    if not(sim_type == 'mujoco'):
        zeta /= np.sqrt(mass)
    print(zeta)

    stiffness = k0 * np.logspace(-2, 2, num=35)
    
    if (sim_type == 'mujoco'):
        params_range['damping'] = (2 * zeta * np.sqrt(stiffness)).tolist()
        params_range['stiffness'] = stiffness
    else:
        params_range['damping'] = (2 * zeta * np.sqrt(mass) * np.sqrt(stiffness)).tolist()
        params_range['stiffness'] = stiffness

    return params_range


def get_stiffness_range(sim_type, k0):
    return {'stiffness' : (k0 * np.logspace(-1.9, 1, num=5)).tolist()}

def get_friction_range(sim_type, mu_0):
    params_range = {}
    if (sim_type == 'drake'):
        params_range['mu'] = (mu_0 * np.logspace(-1, 1, base=2, num=25)).tolist()
    else:
        params_range['mu_tangent'] = (mu_0 * np.logspace(-1, 1, base=2, num=25)).tolist()
    return params_range

def get_damping_range(sim_type, b0):
    params_range = {}
    if (sim_type == 'drake'):
        params_range['dissipation'] = (b0 * np.logspace(-1, 1, base=10, num=5)).tolist()
    else:
        params_range['damping'] = (b0 * np.logspace(-1, 1, base=10, num=5)).tolist()
    return params_range


def get_cube_params_range(sim_type):
    params_range = {}
    if (sim_type == 'drake'):
        params_range['mu'] = np.arange(0.01, 0.2, 0.01).tolist()
        params_range['stiffness'] =  np.arange(10000.0, 30000.0, 1000.0).tolist()
        params_range['dissipation'] = np.arange(0, 1.0, 0.125).tolist()
        # params_range['stiction_tol'] = np.logspace(-6, -1, 20).tolist()
        return params_range
    else:
        params_range['stiffness'] = np.arange(1000, 8000, 500).tolist()
        params_range['mu_tangent'] = np.arange(0.01, 0.5, 0.05).tolist()
        # params_range['damping'] = np.arange(0, 500, 50).tolist()
        # params_range['mu_torsion'] = np.logspace(-3, 0, 10).tolist()
        # params_range['mu_rolling'] = np.logspace(-6, -2, 10).tolist()
    
    if (sim_type == 'bullet'):
        params_range['restitution'] = np.arange(0.01, 0.3, 0.05).tolist()
    
    return params_range

def get_cassie_params_range(sim_type):
    params_range = {}
    if (sim_type == 'drake'):
        # params_range['mu'] = np.arange(0.01, 0.6, 0.01).tolist()
        # params_range['stiffness'] = np.arange(10000.0, 100000.0, 1000.0).tolist()
        # params_range['dissipation'] = np.arange(0, 1.0, 0.125).tolist()
        # params_range['stiction_tol'] = np.logspace(-6, -1, 20).tolist()
        # params_range['dissipation'] = np.linspace(0, 5.0, num=50).tolist()

        # Below are the "true" ranges
        params_range['mu'] = np.linspace(0.0, 0.5, num=50).tolist()
        params_range['stiffness'] = np.linspace(500.0, 50000.0, num=50).tolist()
        params_range['dissipation'] = np.linspace(0, 5.0, num=50).tolist()
        return params_range
    elif (sim_type == 'mujoco'):
        # params_range['mu_tangent'] = np.arange(0.01, 0.5, 0.05).tolist()
        # params_range['stiffness'] = np.arange(10000, 100000, 1000.0).tolist()
        # params_range['damping'] = np.arange(0, 500, 50).tolist()
        # params_range['mu_tangent'] = np.linspace(0.05, 0.5, num=50).tolist()
        params_range['stiffness'] = np.linspace(000.0, 50000.0, num=50).tolist()
        # params_range['mu_torsion'] = np.logspace(-3, 0, 10).tolist()
        # params_range['mu_rolling'] = np.logspace(-6, -2, 10).tolist()

        # range used for final plots
        params_range['mu_tangent'] = np.linspace(0.0, 0.5, num=50).tolist()
        # params_range['stiffness'] = np.linspace(1000.0, 50000.0, num=50).tolist()
        params_range['damping'] = np.linspace(0, 500, num=50).tolist()
    else:
        print('Unknown simulator')

    return params_range

def cube_sensitivity_analysis_main(learning_result, use_all_traj=False):
    sim_type = learning_result.split('_')[0]

    eval_sim = get_eval_sim(learning_result)
    if (eval_sim == None): quit()

    params, test_set, training_loss = load_params_and_logs(learning_result)
    params_range = get_cube_params_range(sim_type)

    if (use_all_traj):
        test_set = range(550)

    avg, med = get_sensitivity_analysis(
        eval_sim, 
        training_loss, 
        params, 
        params_range, 
        test_set)

    return avg, med

def get_cassie_sim(result_id):
    sim_type = result_id.split('_')[0]
    # substeps = int(result_id.split('_')[-1])
    if (sim_type == 'mujoco'):
        eval_sim = mujoco_cassie_sim.LearningMujocoCassieSim(loss_filename='2021_09_07_weights')
    elif (sim_type == 'drake'):
        eval_sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5, loss_filename='2021_09_07_weights')
    else:
        eval_sim = None
    return eval_sim

def load_cassie_params(result_id):
    sim_type = result_id.split('_')[0]
    eval_sim = get_cassie_sim(result_id)
    if (eval_sim == None): quit()
    params = eval_sim.load_params(result_id).value
    return params

def cassie_sensitivity_analysis_main(optimal_param_file):
    sim_type = optimal_param_file.split('_')[0]
    eval_sim = get_cassie_sim(optimal_param_file)
    if (eval_sim == None): quit()

    params_range = get_cassie_params_range(sim_type)
    params = eval_sim.load_params(optimal_param_file).value

    avg, med = get_sensitivity_analysis(
        eval_sim,
        None,
        params,
        params_range,
        None,
        plant='cassie')

    return avg, med


if (__name__ == '__main__'):
    result_id = sys.argv[1]
    cube_sensitivity_analysis_main(result_id)
