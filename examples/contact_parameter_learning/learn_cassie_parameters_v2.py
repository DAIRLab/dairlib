import numpy as np
import os
import pickle
from json import dump, load

import nevergrad as ng

import drake_cassie_sim_v2
import mujoco_cassie_sim_v2
import bullet_cassie_sim
import cassie_loss_utils

import plotstyler
import matplotlib.pyplot as plt

from random import sample, choice
import time


class CassieContactParamsOptimizer():

    def __init__(self, budget, loss_function):
        self.budget = budget
        self.total_loss = 0
        self.hardware_traj_nums = np.arange(0, 29)
        self.loss_function = loss_function
        self.end_time = 0.0495

        self.loss_over_time = []

    def get_single_loss(self, params, hardware_traj_num=None):
        if (hardware_traj_num == None):
            hardware_traj_num = choice(self.hardware_traj_nums)

        # initialize sim
        self.sim.make(params, hardware_traj_num)

        # rollout a trajectory and compute the loss
        rollout = self.sim.advance_to(self.end_time)
        loss = self.loss_function.calc_loss(rollout, self.sim.hardware_traj)
        self.loss_over_time.append(loss)
        return loss

    def get_batch_loss(self, params):
        self.total_loss = 0
        for hardware_traj_num in self.hardware_traj_nums:
            self.total_loss += self.get_single_loss(params, hardware_traj_num)
        avg_loss = self.total_loss / len(self.hardware_traj_nums)
        return

    def learn_drake_params(self, batch=True):
        self.default_params = ng.p.Dict(
            mu=ng.p.Scalar(lower=0.01, upper=1.0),
            stiffness=ng.p.Log(lower=1e3, upper=1e6),
            dissipation=ng.p.Scalar(lower=0.0, upper=3.0),
        )

        self.sim = drake_cassie_sim_v2.DrakeCassieSim()
        self.default_params = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)

    def learn_mujoco_params(self, batch=True):
        self.default_params = ng.p.Dict(
            stiffness=ng.p.Scalar(lower=0, upper=1e6),
            damping=ng.p.Scalar(lower=0, upper=1000),
            mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0)
        )

        self.sim = drake_cassie_sim_v2.DrakeCassieSim()
        self.default_params = self.sim.default_params
        optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        if batch:
            params = optimizer.minimize(self.get_batch_loss)

if __name__ == '__main__':
    budget = 10
    loss_function = cassie_loss_utils.CassieLoss(loss_filename='2021_09_07_weights')
    optimizer = CassieContactParamsOptimizer(budget, loss_function)