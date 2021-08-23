import numpy as np
import plot_styler
import sys
import matplotlib.pyplot as plt
from evaluate_cube_parameters import load_traj_pairs, load_params_and_logs
from copy import deepcopy
from random import sample
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import cube_sim

def get_sensitivity_analysis(sim, loss_weights, optimal_params, params_range, test_traj_set, plant='cube'):
    
    loss_sweeps = {}
    for param_key in params_range:
        loss_sweep = []
        params = deepcopy(optimal_params)
        print(f'sweeping {param_key}')
        for param_val in params_range[param_key]:
            loss_sum = 0
            params[param_key] = param_val
            print(f'{param_key}: {param_val}')
            if (plant == 'cube'):
                pairs = load_traj_pairs(sim, params, test_traj_set)
                for pair_idx in pairs:
                    pair = pairs[pair_idx]
                    loss_sum += loss_weights.CalculateLoss(pair[0], pair[1])
                loss_sweep.append(loss_sum / len(pairs))
            else:
                raise NotImplementedError('Need to define a cassie method for this')
        loss_sweeps[param_key] = np.array(loss_sweep)

    return loss_sweeps

def plot_sensitivity_analysis(loss_sweeps, params_range):
    ps = plot_styler.PlotStyler()
    for key in loss_sweeps:
        plt.figure()
        ps.plot(params_range[key], loss_sweeps[key])
        plt.title(key)
        if (key == 'pen_allow' or key == 'stiction_tol' or key == 'mu_torsion' or key == 'mu_rolling'):
            plt.xscale('log')
    plt.show()

def get_cube_params_range(sim_type):
    params_range = {}
    if (sim_type == 'drake'):
        params_range['mu'] = np.arange(0.01, 0.2, 0.01).tolist()
        # params_range['stiffness'] = np.arange(100.0, 100000.0, 10000.0).tolist()
        # params_range['dissipation'] = np.arange(0, 2.5, 0.25).tolist()
        # params_range['stiction_tol'] = np.logspace(-6, -1, 20).tolist()
        return params_range
    else:
        params_range['stiffness'] = np.arange(1000, 10000, 250).tolist()
        params_range['damping'] = np.arange(0, 100, 4).tolist()
        params_range['mu_torsion'] = np.logspace(-3, 0, 10).tolist()
        params_range['mu_rolling'] = np.logspace(-6, -2, 10).tolist()

    if (sim_type == 'mujoco'):

        params_range['cube_mu_tangent'] = np.arange(0.1, 1.0, 0.05).tolist()
        params_range['table_mu_tangent'] = np.arange(0.1, 1.0, 0.05).tolist()
    elif (sim_type == 'bullet'):

        params_range['mu_tangent'] = np.arange(0.1, 1.0, 0.05).tolist()
        params_range['restitution'] = np.arange(0.01, 1.0, 0.1).tolist()
    
    return params_range


def cube_sensitivity_analysis_main(learning_result):
    sim_type = learning_result.split('_')[0]

    if (sim_type == 'mujoco'):
        eval_sim = mujoco_cube_sim.MujocoCubeSim()
    elif (sim_type == 'drake'):
        eval_sim = drake_cube_sim.DrakeCubeSim()
    elif (sim_type == 'bullet'):
        eval_sim = bullet_cube_sim.BulletCubeSim()
    else:
        print(f'{sim_type} is not a supported simulator - please check for spelling mistakes and try again')
        quit()
    
    params, test_set, training_loss = load_params_and_logs(learning_result)
    params_range = get_cube_params_range(sim_type)

    sweep = get_sensitivity_analysis(
        eval_sim, 
        training_loss, 
        params, 
        params_range, 
        test_set)
    
    plot_sensitivity_analysis(sweep, params_range)

if (__name__ == '__main__'):
    result_id = sys.argv[1]
    cube_sensitivity_analysis_main(result_id)