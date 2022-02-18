import numpy as np
import os
import pickle
from json import dump, load

import nevergrad as ng

import drake_cassie_sim_v2
import mujoco_cassie_sim_v2
import isaac_cassie_sim
import bullet_cassie_sim
import cassie_loss_utils

# import plotstyler
# import matplotlib.pyplot as plt

from random import sample, choice
import time


class CassieContactParamsOptimizer():

    def __init__(self, budget, loss_function):
        self.budget = budget
        self.total_loss = 0
        hardware_trajs = np.arange(0, 29)
        selected_hardware_trajs = np.hstack((np.arange(0, 4), np.arange(6, 10), np.arange(10, 24)))
        self.hardware_traj_nums = ["%.2d" % i for i in hardware_trajs]
        self.selected_hardware_trajs_nums = ["%.2d" % i for i in selected_hardware_trajs]
        self.loss_function = loss_function
        self.end_time = 0.0495

        self.drake_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/drake_cassie_params/"
        self.mujoco_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/mujoco_cassie_params/"
        self.isaac_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/isaac_cassie_params/"
        self.bullet_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/bullet_cassie_params/"
        self.date_prefix = time.strftime("%Y_%m_%d_%H")
        self.loss_over_time = []

    def save_params(self, folder, params, budget):
        with open(folder + self.date_prefix + '_' + str(budget) + '.pkl', 'wb') as f:
            pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

    def load_params(self, param_file, folder):
        with open(folder + param_file + '.pkl', 'rb') as f:
            return pickle.load(f)

    def load_drake_params(self, param_file):
        return self.load_params(param_file, self.drake_params_folder)

    def load_mujoco_params(self, param_file):
        return self.load_params(param_file, self.mujoco_params_folder)

    def load_isaac_params(self, param_file):
        return self.load_params(param_file, self.isaac_params_folder)

    def get_single_loss(self, hardware_traj_num=None, params=None):
        if (hardware_traj_num == None):
            hardware_traj_num = choice(self.hardware_traj_nums)
        # initialize sim,
        # if parameters are specified, the simulator needs to be remade to initialize the new simulator parameters
        # reset can be used to initialize from a different set of initial conditions
        if (params == None):
            self.sim.reset(hardware_traj_num)
        else:
            self.sim.make(params, hardware_traj_num)
        # rollout a trajectory and compute the loss
        rollout = self.sim.advance_to(self.end_time)
        loss = self.loss_function.CalcLoss(rollout, self.sim.hardware_traj)
        self.loss_over_time.append(loss)
        # print(hardware_traj_num)
        # print(loss)
        return loss

    def get_batch_loss(self, params):
        self.total_loss = 0
        self.sim.make(params, '00')
        # for hardware_traj_num in self.hardware_traj_nums:
        for hardware_traj_num in self.selected_hardware_trajs_nums:
            self.total_loss += self.get_single_loss(hardware_traj_num)
            # self.total_loss += self.get_single_loss(hardware_traj_num, params)
        self.sim.free_sim()
        avg_loss = self.total_loss / len(self.hardware_traj_nums)
        print(avg_loss)
        return avg_loss

    def learn_drake_params(self, batch=True):
        self.loss_over_time = []
        self.default_params = ng.p.Dict(
            mu=ng.p.Scalar(lower=0.01, upper=1.0),
            stiffness=ng.p.Log(lower=1e3, upper=1e6),
            dissipation=ng.p.Scalar(lower=0.0, upper=3.0),
        )

        self.sim = drake_cassie_sim_v2.DrakeCassieSim()
        self.default_params.value = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)
        else:
            params = optimizer.minimize(self.get_single_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.drake_params_folder + 'training' + '_loss_trajectory_' + str(budget), loss_samples)
        self.save_params(self.drake_params_folder, params, budget)

    def learn_mujoco_params(self, batch=True):
        self.default_params = ng.p.Dict(
            stiffness=ng.p.Scalar(lower=0, upper=1e6),
            damping=ng.p.Scalar(lower=0, upper=1000),
            mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0)
        )
        self.sim = mujoco_cassie_sim_v2.MuJoCoCassieSim()
        self.default_params.value = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)
        else:
            params = optimizer.minimize(self.get_single_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.mujoco_params_folder + 'training' + '_loss_trajectory_' + str(budget), loss_samples)
        self.save_params(self.mujoco_params_folder, params, budget)

    def learn_isaac_params(self, batch=True):
        self.default_params = ng.p.Dict(
            mu=ng.p.Scalar(lower=0.01, upper=1.0),
            stiffness=ng.p.Log(lower=1e-1, upper=1e3),
            restitution=ng.p.Scalar(lower=0.0, upper=1),
        )
        self.sim = isaac_cassie_sim.IsaacCassieSim()
        self.default_params.value = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)
        else:
            params = optimizer.minimize(self.get_single_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.isaac_params_folder + 'training' + '_loss_trajectory_' + str(budget), loss_samples)
        self.save_params(self.isaac_params_folder, params, budget)

    def learn_bullet_params(self, batch=True):
        self.default_params = ng.p.Dict(
            mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0),
            stiffness=ng.p.Log(lower=1e-1, upper=1e6),
            damping=ng.p.Scalar(lower=0.0, upper=1000),
        )
        self.sim = bullet_cassie_sim.BulletCassieSim()
        self.default_params.value = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)
        else:
            params = optimizer.minimize(self.get_single_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.bullet_params_folder + 'training' + '_loss_trajectory_' + str(budget), loss_samples)
        self.save_params(self.bullet_params_folder, params, budget)


if __name__ == '__main__':
    budget = 1000
    loss_function = cassie_loss_utils.CassieLoss(filename='2021_09_07_weights')
    # loss_function = cassie_loss_utils.CassieLoss(filename='default_loss_weights')
    # loss_function = cassie_loss_utils.CassieLoss(filename='pos_loss_weights')
    optimizer = CassieContactParamsOptimizer(budget, loss_function)

    # optimizer.learn_drake_params()
    # optimizer.learn_mujoco_params()
    # optimizer.learn_isaac_params()
    optimizer.learn_bullet_params()

    # isaac_params = optimizer.load_isaac_params('2022_02_17_17_1000')
    # isaac_params = optimizer.load_isaac_params('2022_02_16_18_1000')
    # loss_trajectory = np.load(optimizer.isaac_params_folder + 'training_loss_trajectory_1000.npy')

    # drake_params = optimizer.load_drake_params('2022_02_17_17_1000')
    # drake_params = optimizer.load_drake_params('2022_02_17_11_1000')
    # loss_trajectory = np.load(optimizer.drake_params_folder + 'training_loss_trajectory_1000.npy')

    # mujoco_params = optimizer.load_mujoco_params('2022_02_17_11_1000')
    # mujoco_params = optimizer.load_mujoco_params('2022_02_18_11_1000')
    # loss_trajectory = np.load(optimizer.mujoco_params_folder + 'training_loss_trajectory_1000.npy')
    # import matplotlib.pyplot as plt
    # plt.plot(loss_trajectory)
    # plt.show()

    # import pdb; pdb.set_trace()
