import numpy as np
import os
import pickle
from json import dump, load

import nevergrad as ng

from pydairlib.cassie.gym_envs.cassie_gym import *
# from cassie_utils import *
from pydairlib.cassie.controllers import OSCRunningControllerFactory
from pydairlib.cassie.controllers import OSCWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydairlib.cassie.gym_envs.reward_osudrl import RewardOSUDRL
from pydrake.common.yaml import yaml_load, yaml_dump
import time

import yaml

class OSCGainsOptimizer():

    def __init__(self, budget, reward_function):
        self.budget = budget
        self.total_loss = 0
        self.reward_function = reward_function
        self.end_time = 5.0
        self.gym_env = None
        self.sim = None
        self.controller = None

        self.urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
        self.default_osc_running_gains_filename = 'examples/Cassie/osc_run/osc_running_gains.yaml'
        self.osc_running_gains_filename = 'examples/Cassie/osc_run/learned_osc_running_gains.yaml'
        self.osqp_settings = 'examples/Cassie/osc_run/osc_running_qp_settings.yaml'

        self.drake_params_folder = "bindings/pydairlib/cassie/optimal_gains/"
        self.date_prefix = time.strftime("%Y_%m_%d_%H")
        self.loss_over_time = []

        self.default_osc_gains = {
            'swing_foot_kp': np.array([125, 80, 50]),
            'swing_foot_kd': np.array([5, 5, 1]),
            'pelvis_rot_kp': np.array([50, 100]),
            'pelvis_rot_kd': np.array([10, 5]),
            'w_input_reg': 0.001}

    def save_params(self, folder, params, budget):
        with open(folder + self.date_prefix + '_' + str(budget) + '.pkl', 'wb') as f:
            pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

    def load_params(self, param_file, folder):
        with open(folder + param_file + '.pkl', 'rb') as f:
            return pickle.load(f)


    def write_params(self, params):
        gains = yaml_load(filename=self.default_osc_running_gains_filename, private=True)
        new_gains = gains.copy()
        new_swing_foot_kp = [0]*9
        new_swing_foot_kd = [0]*9
        new_pelvis_rot_kp = [0]*9
        new_pelvis_rot_kd = [0]*9

        new_swing_foot_kp[0] = float(params['swing_foot_kp'][0])
        new_swing_foot_kp[4] = float(params['swing_foot_kp'][1])
        new_swing_foot_kp[8] = float(params['swing_foot_kp'][2])
        new_swing_foot_kd[0] = float(params['swing_foot_kd'][0])
        new_swing_foot_kd[4] = float(params['swing_foot_kd'][1])
        new_swing_foot_kd[8] = float(params['swing_foot_kd'][2])
        new_pelvis_rot_kp[0] = float(params['pelvis_rot_kp'][0])
        new_pelvis_rot_kp[4] = float(params['pelvis_rot_kp'][1])
        new_pelvis_rot_kd[0] = float(params['pelvis_rot_kd'][0])
        new_pelvis_rot_kd[4] = float(params['pelvis_rot_kd'][1])
        new_gains['SwingFootKp'] = new_swing_foot_kp
        new_gains['SwingFootKd'] = new_swing_foot_kd
        new_gains['PelvisRotKp'] = new_pelvis_rot_kp
        new_gains['PelvisRotKd'] = new_pelvis_rot_kd
        new_gains['w_input_reg'] = params['w_input_reg']
        # import pdb; pdb.set_trace()
        yaml_dump(new_gains, filename=self.osc_running_gains_filename)
        # yaml_dump(gains, filename=self.osc_running_gains_filename)

    def get_single_loss(self, params):
        self.write_params(params)
        controller_plant = MultibodyPlant(8e-5)
        addCassieMultibody(controller_plant, None, True, self.urdf, False, False)
        controller_plant.Finalize()
        self.controller = OSCRunningControllerFactory(controller_plant, self.osc_running_gains_filename, self.osqp_settings)
        self.gym_env.make(self.controller, self.urdf)
        # rollout a trajectory and compute the loss
        cumulative_reward = 0
        while self.gym_env.current_time < 5.0:
            state, reward = self.gym_env.step(np.zeros(18))
            cumulative_reward += reward
        self.loss_over_time.append(cumulative_reward)
        print(cumulative_reward)
        return -cumulative_reward

    def learn_drake_params(self, batch=True):
        self.loss_over_time = []
        self.default_params = ng.p.Dict(
            swing_foot_kp=ng.p.Array(shape=(3,)).set_bounds(lower=0., upper=150.),
            swing_foot_kd=ng.p.Array(shape=(3,)).set_bounds(lower=0., upper=10.),
            pelvis_rot_kp=ng.p.Array(shape=(2,)).set_bounds(lower=0., upper=150.),
            pelvis_rot_kd=ng.p.Array(shape=(2,)).set_bounds(lower=0., upper=10.),
            w_input_reg=ng.p.Log(lower=0.00001, upper=0.001),
        )
        self.gym_env = CassieGym(self.reward_function, visualize=False)
        self.default_params.value = self.default_osc_gains
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        params = optimizer.minimize(self.get_single_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.drake_params_folder + 'loss_trajectory_' + str(self.budget), loss_samples)
        self.save_params(self.drake_params_folder, params, budget)

if __name__ == '__main__':
    # budget = 2000
    budget = 2000

    reward_function = RewardOSUDRL()

    optimizer = OSCGainsOptimizer(budget, reward_function)
    # optimizer.get_single_loss(optimizer.default_osc_gains)
    # optimizer.write_params(optimizer.default_osc_gains)
    optimizer.learn_drake_params()

