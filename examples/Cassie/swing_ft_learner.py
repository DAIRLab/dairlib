import torch
import lcm
import yaml


class SwingFootLearner:
    def __init__(self, sim_env):
        self.sim_env = sim_env
        return

    def select_action(self, state, train):
        return
    
    def learn(self, num_env_steps):
        return


class QFunctionLearner(SwingFootLearner):
    def __init__(self, sim_env, net_arch, nominal_params):
        return

    def train(self, data):
        return

    def select_action(self, train=True):
        return


class EvolutionaryLearner:
    def __init__(self)
        return


class SACLearner:
    def __init__(self, net_arch):
        return
