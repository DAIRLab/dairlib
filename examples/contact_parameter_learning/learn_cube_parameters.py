import nevergrad as ng
import cube_sim
import drake_cube_sim
import mujoco_cube_sim
import bullet_cube_sim
import os
import sys
from json import dump, load
from random import sample, choice
import numpy as np
from concurrent import futures
import time

####################################
## COMMON VALUES AND FUNCTIONS

# File paths
cube_data_folder = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/cleaned_cube_trajectories/data')
drake_data_folder = os.path.join(os.getcwd(), 
    'examples/contact_parameter_learning/simulated_cube_trajectories/drake')
mujoco_data_folder = os.path.join(os.getcwd(), 
    'examples/contact_parameter_learning/simulated_cube_trajectories/mujoco')
log_folder = os.path.join(os.getcwd(), 'examples/contact_paramter_learning/logs/cube')
model_folder = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/learned_parameters/cube')
default_loss = cube_sim.LossWeights(pos=(1.0/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)),
                                    vel=(0.1/cube_sim.BLOCK_HALF_WIDTH)*np.ones((3,)),
                                    omega=0.1*np.ones((3,)))

batch_size = 25
num_workers = 1
num_trials = 550
num_train = 445
budget = 10000
num_test = num_trials - num_train

# Make a list of train and test trials 
trial_idxs = range(num_trials)
training_idxs = sample(trial_idxs, num_train)
test_idxs = [idx if not (idx in training_idxs) else [] for idx in trial_idxs]

def log_optimization(sim_name, loss, weights, params):
    datetime_str = time.strftime("%Y_%m_%d_%H%M%S")
    base_filename = os.path.join(log_folder, sim_name + '_' + datetime_str + '_')
    loss_log_name = os.path.join(base_filename, 'loss.npy')
    weights_file_name = os.path.join(base_filename, 'weights.json')

    with open(loss_log_name, 'w+') as fpl:
        np.save(fpl, loss)

    with open(weights_file_name, 'w+') as fpw:
        dump(weights, fpw)

    save_params(sim_name, datetime_str, params)

def save_params(simulator, id, params):
    filename = os.path.join(model_folder, simulator + str(id) +'.json')
    with open(filename, 'w+') as fp:
        dump(params, fp)

def load_params(simulator, id):
    filename = os.path.join(model_folder, simulator + str(id) +'.json')
    with open(filename, 'r+') as fp:
        return load(fp)

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


    

####################################
## DRAKE FUNCTIONS

def get_drake_loss_mp(params):
    loss_sum = 0
    for i in range(batch_size):
        loss_sum += get_drake_loss(params)
    print(loss_sum / batch_size)
    return loss_sum / batch_size

def get_drake_loss(params, trial_num=None):
    if (trial_num == None): trial_num = choice(training_idxs)
    sim = drake_cube_sim.DrakeCubeSim(visualize=False)
    return cube_sim.calculate_cubesim_loss(params, trial_num, cube_data_folder, sim, debug=False, weights=default_loss)

def learn_drake_params():
    
    optimization_param = ng.p.Dict(
        mu_static = ng.p.Scalar(lower=0.1, upper=1.0), 
        mu_ratio = ng.p.Scalar(lower=0.1, upper=1.0),
        pen_allow=ng.p.Log(lower=1e-10, upper=1e-2),
        stiction_tol=ng.p.Log(lower=1e-6, upper=1e-1)
    )

    optimization_param.value=drake_cube_sim.default_drake_contact_params
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget, num_workers=num_workers)
    with futures.ThreadPoolExecutor(max_workers=optimizer.num_workers) as executor:
        optimal_params = optimizer.minimize(get_drake_loss_mp, executor=executor, batch_mode=True)
    save_params('drake', 809, optimal_params.value)

####################################
## MUJOCO FUNCTIONS

def get_mujoco_loss_mp(params):
    loss_sum = 0
    for i in range(batch_size):
        loss_sum += get_mujoco_loss(params)
    print(loss_sum / batch_size)
    return loss_sum / batch_size

def get_mujoco_loss(params, trial_num=None):
    if (trial_num == None): trial_num = choice(training_idxs)
    sim = mujoco_cube_sim.MujocoCubeSim(visualize=False)
    return cube_sim.calculate_cubesim_loss(params, trial_num, cube_data_folder, sim, debug=False, weights=default_loss)

def learn_mujoco_params():
    optimization_param = ng.p.Dict(
        stiffness=ng.p.Scalar(lower=100, upper=10000),
        damping=ng.p.Scalar(lower=0, upper=1000),
        cube_mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0),
        table_mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0),
        mu_torsion=ng.p.Scalar(lower=0.001, upper=1.0),
        mu_rolling=ng.p.Log(lower=0.000001, upper=0.01)
    )
    optimization_param.value=mujoco_cube_sim.default_mujoco_contact_params
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget, num_workers=num_workers)
    with futures.ThreadPoolExecutor(max_workers=optimizer.num_workers) as executor:
        optimal_params = optimizer.minimize(get_mujoco_loss_mp, executor=executor, batch_mode=True)
    save_params('mujoco', 809, optimal_params.value)

####################################
## BULLET FUNCTIONS

def get_bullet_loss_mp(params):
    loss_sum = 0
    for i in range(batch_size):
        loss_sum += get_bullet_loss(params)
    print(loss_sum / batch_size)
    return loss_sum / batch_size

def get_bullet_loss(params, trial_num=None):
    if (trial_num == None): trial_num = choice(training_idxs)
    sim = bullet_cube_sim.BulletCubeSim(visualize=False)
    return cube_sim.calculate_cubesim_loss(params, trial_num, cube_data_folder, sim, debug=False, weights=default_loss)

def learn_bullet_params():
    optimization_param = ng.p.Dict(
        stiffness=ng.p.Scalar(lower=100, upper=10000),
        damping=ng.p.Scalar(lower=0, upper=1000),
        restitution=ng.p.Scalar(lower=0.01, upper=0.3),
        mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0),
        mu_torsion=ng.p.Scalar(lower=0.001, upper=1.0),
        mu_rolling=ng.p.Log(lower=0.000001, upper=0.01)
    )
    optimization_param.value=bullet_cube_sim.default_bullet_contact_params
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget, num_workers=num_workers)
    with futures.ThreadPoolExecutor(max_workers=optimizer.num_workers) as executor:
        optimal_params = optimizer.minimize(get_bullet_loss_mp, executor=executor, batch_mode=True)
    save_params('bullet', 809, optimal_params.value)

if (__name__ == '__main__'):
    sim_choice = sys.argv[1]

    if (len(sys.argv) > 2):
        params_file_id = sys.argv[2]
        toss_id = sys.argv[3]
        print(f'Visualizing toss {toss_id} in {sim_choice} simulator with params from {params_file_id}')
        params = load_params(sim_choice, params_file_id)
        visualize_learned_params(params, sim_choice, toss_id)

    if (sim_choice == 'drake'):
        learn_drake_params()
    elif (sim_choice == 'mujoco'):
        learn_mujoco_params
    elif (sim_choice == 'bullet'):
        learn_bullet_params()

    else:
        print('please pick a simulator')