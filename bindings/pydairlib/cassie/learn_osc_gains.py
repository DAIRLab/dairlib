import numpy as np
import os
import pickle
from json import dump, load
import concurrent.futures as futures
import nevergrad as ng

from pydairlib.cassie.gym_envs.drake_cassie_gym import *
from pydairlib.cassie.gym_envs.mujoco_cassie_gym import *

from pydairlib.cassie.controllers import OSCRunningControllerFactory
from pydairlib.cassie.controllers import OSCWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydairlib.cassie.gym_envs.reward_osudrl import RewardOSUDRL
from pydrake.common.yaml import yaml_load, yaml_dump
import time
import matplotlib.pyplot as plt
import yaml


class OSCGainsOptimizer():

    def __init__(self, budget, reward_function, visualize=False):
        self.budget = budget
        self.total_loss = 0
        self.reward_function = reward_function
        self.end_time = 10.0
        self.gym_env = None
        self.sim = None
        self.controller = None
        self.visualize = visualize

        self.urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
        self.controller_urdf = 'examples/Cassie/urdf/cassie_v2_conservative.urdf'
        self.default_osc_running_gains_filename = 'examples/Cassie/osc_run/osc_running_gains.yaml'
        self.osc_running_gains_filename = 'examples/Cassie/osc_run/learned_osc_running_gains.yaml'
        self.osqp_settings = 'examples/Cassie/osc_run/osc_running_qp_settings.yaml'

        self.drake_params_folder = "bindings/pydairlib/cassie/optimal_gains/"
        self.date_prefix = time.strftime("%Y_%m_%d_%H")
        self.loss_over_time = []

        self.default_osc_gains = {
            'w_soft_constraint': 100,
            'w_input_reg': 0.001,
            'mu': 0.6,
            'w_hip_yaw': 2.5,
            'hip_yaw_kp': 40,
            'PelvisKp': np.array([0, 0, 50]),
            'PelvisKd': np.array([0, 0, 5]),
            # 'PelvisRotKp': np.array([50, 100, 0]),
            'PelvisRotKd': np.array([10, 5, 1]),
            'SwingFootKp': np.array([125, 80, 50]),
            'SwingFootKd': np.array([5, 5, 1]),
            'FootstepKd': np.array([0.2, 0.45, 0]),
            'footstep_sagital_offset': 0,
            'mid_foot_height': 0.05,
            'rest_length': 0.95,
            'footstep_lateral_offset': 0.045,
            # 'stance_duration': 0.30,
            # 'flight_duration': 0.08,
        }

    def save_params(self, folder, params, budget):
        with open(folder + self.date_prefix + '_' + str(budget) + '.pkl', 'wb') as f:
            pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

    def load_params(self, param_file, folder):
        with open(folder + param_file + '.pkl', 'rb') as f:
            return pickle.load(f)

    def write_params(self, params):
        gains = yaml_load(filename=self.default_osc_running_gains_filename, private=True)
        new_gains = gains.copy()
        for key in params:
            if hasattr(params[key], "__len__"):
                new_gains[key] = np.diag(params[key]).flatten().tolist()
            else:
                new_gains[key] = params[key]
        yaml_dump(new_gains, filename=self.osc_running_gains_filename)

    def get_batch_loss(self, params):
        batch_size = 10
        batch_reward = 0
        for i in range(batch_size):
            batch_reward += self.get_single_loss(params)
        self.loss_over_time.append(batch_reward)
        return batch_reward

    def get_single_loss(self, params):
        self.write_params(params)
        if np.random.random() < 0.0:
            print('drake_sim')
            gym_env = DrakeCassieGym(self.reward_function, visualize=self.visualize)
        else:
            # print('mujoco')
            gym_env = MuJoCoCassieGym(self.reward_function, visualize=self.visualize)
        gym_env.end_time = self.end_time
        controller_plant = MultibodyPlant(8e-5)
        AddCassieMultibody(controller_plant, None, True, self.controller_urdf, False, False)
        controller_plant.Finalize()
        controller = OSCRunningControllerFactory(controller_plant, self.osc_running_gains_filename,
                                                 self.osqp_settings)
        gym_env.make(controller)
        # rollout a trajectory and compute the loss
        cumulative_reward = 0
        while gym_env.current_time < gym_env.end_time and not gym_env.terminated:
            state, reward = gym_env.step(np.zeros(18))
            cumulative_reward += reward
        # print(-cumulative_reward)
        gym_env.free_sim()
        return -cumulative_reward

    def learn_gains(self, batch=True):
        self.default_params = ng.p.Dict(
            w_soft_constraint=ng.p.Log(lower=1.0, upper=1000.0),
            w_input_reg=ng.p.Log(lower=1e-5, upper=1e-1),
            mu=ng.p.Scalar(lower=0.4, upper=1.0),
            w_hip_yaw=ng.p.Scalar(lower=0, upper=10),
            hip_yaw_kp=ng.p.Scalar(lower=20, upper=80),
            PelvisKp=ng.p.Array(lower=0., upper=75., shape=(3,)),
            PelvisKd=ng.p.Array(lower=0., upper=10., shape=(3,)),
            # PelvisRotKp=ng.p.Array(lower=20., upper=150., shape=(3,)),
            PelvisRotKd=ng.p.Array(lower=0., upper=15., shape=(3,)),
            SwingFootKp=ng.p.Array(lower=20., upper=150., shape=(3,)),
            SwingFootKd=ng.p.Array(lower=0., upper=15., shape=(3,)),
            FootstepKd=ng.p.Array(lower=0., upper=1., shape=(3,)),
            footstep_sagital_offset=ng.p.Scalar(lower=-0.1, upper=0.1),
            mid_foot_height=ng.p.Scalar(lower=0.03, upper=0.15),
            rest_length=ng.p.Scalar(lower=0.8, upper=1.0),
            footstep_lateral_offset=ng.p.Scalar(lower=-0.1, upper=0.05),
            # stance_duration=ng.p.Scalar(lower=0.25, upper=0.40),
            # flight_duration=ng.p.Scalar(lower=0.05, upper=0.15),
        )
        self.loss_over_time = []
        self.default_params.value = self.default_osc_gains
        # optimizer = ng.optimizers.NGOpt(parametrization=self.default_params, budget=self.budget)
        optimizer = ng.optimizers.CMandAS3(parametrization=self.default_params, budget=self.budget)
        optimizer.register_callback("tell", ng.callbacks.ProgressBar())
        # with futures.ThreadPoolExecutor(max_workers=optimizer.num_workers) as executor:
        # recommendation = optimizer.minimize(square, executor=executor, batch_mode=False)
        # params = optimizer.minimize(self.get_single_loss)
        params = optimizer.minimize(self.get_batch_loss)
        loss_samples = np.array(self.loss_over_time)
        np.save(self.drake_params_folder + 'loss_trajectory_' + str(self.budget), loss_samples)
        self.save_params(self.drake_params_folder, params, budget)

    def learn_gains_apex(self):
        from optimizers.ppo import run_experiment
        import argparse
        parser = argparse.ArgumentParser()

        # general args
        parser.add_argument("--logdir", type=str, default="./trained_models/ppo/")          # Where to log diagnostics to
        parser.add_argument("--seed", default=0, type=int)                                  # Sets Gym, PyTorch and Numpy seeds
        parser.add_argument("--history", default=0, type=int)                                         # number of previous states to use as input
        parser.add_argument("--redis_address", type=str, default=None)                      # address of redis server (for cluster setups)
        parser.add_argument("--viz_port", default=8097)                                     # (deprecated) visdom server port

        # PPO algo args
        parser.add_argument("--input_norm_steps", type=int, default=10000)
        parser.add_argument("--n_itr", type=int, default=10000, help="Number of iterations of the learning algorithm")
        parser.add_argument("--lr", type=float, default=1e-4, help="Adam learning rate") # Xie
        parser.add_argument("--eps", type=float, default=1e-5, help="Adam epsilon (for numerical stability)")
        parser.add_argument("--lam", type=float, default=0.95, help="Generalized advantage estimate discount")
        parser.add_argument("--gamma", type=float, default=0.99, help="MDP discount")
        parser.add_argument("--anneal", default=1.0, action='store_true', help="anneal rate for stddev")
        parser.add_argument("--learn_stddev", default=False, action='store_true', help="learn std_dev or keep it fixed")
        parser.add_argument("--std_dev", type=int, default=-1.5, help="exponent of exploration std_dev")
        parser.add_argument("--entropy_coeff", type=float, default=0.0, help="Coefficient for entropy regularization")
        parser.add_argument("--clip", type=float, default=0.2, help="Clipping parameter for PPO surrogate loss")
        parser.add_argument("--minibatch_size", type=int, default=64, help="Batch size for PPO updates")
        parser.add_argument("--epochs", type=int, default=3, help="Number of optimization epochs per PPO update") #Xie
        parser.add_argument("--num_steps", type=int, default=5096, help="Number of sampled timesteps per gradient estimate")
        parser.add_argument("--use_gae", type=bool, default=True,help="Whether or not to calculate returns using Generalized Advantage Estimation")
        parser.add_argument("--num_procs", type=int, default=30, help="Number of threads to train on")
        parser.add_argument("--max_grad_norm", type=float, default=0.05, help="Value to clip gradients at.")
        parser.add_argument("--max_traj_len", type=int, default=400, help="Max episode horizon")
        parser.add_argument("--recurrent",   action='store_true')
        parser.add_argument("--bounded",   type=bool, default=False)
        args = parser.parse_args()

        args = parse_previous(args)

        run_experiment(args)



if __name__ == '__main__':
    # budget = 2000
    budget = 500

    reward_function = RewardOSUDRL()

    optimizer = OSCGainsOptimizer(budget, reward_function, visualize=False)
    optimizer.learn_gains()
    # optimizer.learn_gains_apex()

    # optimal_params = optimizer.load_params('2022_06_09_15_500', optimizer.drake_params_folder).value
    # optimizer.write_params(optimal_params)
    # reward_over_time = np.load('bindings/pydairlib/cassie/optimal_gains/loss_trajectory_500.npy')
    # plt.plot(reward_over_time)
    # plt.show()
