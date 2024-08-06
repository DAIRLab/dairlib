"""
Train a policy for //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
"""
import argparse
import os
from os import path
import numpy as np
import torch as th
import torchvision
import torch.nn as nn

import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.RPO.rpo import RPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, CallbackList
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.monitor import Monitor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import VecMonitor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecVideoRecorder,
    VecNormalize,
)
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.policies import ActorCriticPolicy

from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.ppo_recurrent.ppo_recurrent import RecurrentPPO
from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.common.recurrent.policies import RecurrentActorCriticPolicy

from pydrake.geometry import Meshcat
from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource
)

from pydairlib.perceptive_locomotion.systems.cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from typing import Callable

def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Linear learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
      current learning rate depending on remaining progress
    """
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value

    return func

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias_const)
    return layer

class ResidualBlock_noNorm(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1, downsample=None):
        super(ResidualBlock_noNorm, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
        self.relu = nn.LeakyReLU()
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
        self.downsample = downsample

    def forward(self, x):
        residual = x
        out = self.conv1(x)
        out = self.relu(out)
        out = self.conv2(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)
        return out

class ResidualBlock_Norm(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1, downsample=None):
        super(ResidualBlock_Norm, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu = nn.LeakyReLU()
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.downsample = downsample

    def forward(self, x):
        residual = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)
        return out

class InitialBatchNorm(nn.Module):
    def __init__(self):
        super(InitialBatchNorm, self).__init__()
        # Batch normalization for the last channel
        self.bn = nn.BatchNorm2d(1)

    def forward(self, x):
        channel12 = x[:, :2, :, :]
        channel3 = x[:, 2:3, :, :]
        channel3 = self.bn(channel3)
        x = th.cat((channel12, channel3), dim=1)
        return x

class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 64, last_layer_dim_vf: int = 64):
        super().__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.vector_state_actor = 6+23
        self.vector_state_critic = 6+23
        self.heightmap_size = 64
        self.use_BN = False # Use BatchNorm2D
        if self.use_BN:
            ResidualBlock = ResidualBlock_Norm
        else:
            ResidualBlock = ResidualBlock_noNorm

        n_input_channels = 3

        self.actor_cnn = nn.Sequential(
            #InitialBatchNorm(),
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            #nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                #nn.BatchNorm2d(32)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=1, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=1, bias=False),
                #nn.BatchNorm2d(48)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(48, 64, stride=1, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=1, bias=False),
                #nn.BatchNorm2d(64)
                )),

            nn.Flatten(),
            nn.Linear(64*2*2, 512),
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 96),
        )

        self.critic_cnn = nn.Sequential(
            #InitialBatchNorm(),
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            #nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                #nn.BatchNorm2d(32)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=1, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=1, bias=False),
                #nn.BatchNorm2d(48)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(48, 64, stride=1, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                #nn.BatchNorm2d(64)
                )),

            nn.Flatten(),
            nn.Linear(64*2*2, 512),
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 96),
        )

        self.critic_cnn_gt = nn.Sequential(
            #InitialBatchNorm(),
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            #nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                #nn.BatchNorm2d(32)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=1, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=1, bias=False),
                #nn.BatchNorm2d(48)
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(48, 64, stride=1, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                #nn.BatchNorm2d(64)
                )),

            nn.Flatten(),
            nn.Linear(64*2*2, 512),
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 96),
        )

        # MLP for ALIP state + Vdes (6,) + State (23,)
        self.actor_alip_mlp = nn.Sequential(
            nn.Linear(self.vector_state_actor, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
        )
        self.critic_alip_mlp = nn.Sequential(
            nn.Linear(self.vector_state_critic, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
        )

        self.actor_combined_lstm = nn.LSTM(input_size=128, hidden_size=64, num_layers=2, batch_first=False)
        self.critic_combined_lstm = nn.LSTM(input_size=32+96+96, hidden_size=64, num_layers=2, batch_first=False)
        
        self.actor_combined_fc = nn.Sequential(
            nn.Linear(64, 256),
            nn.Tanh(),
            nn.Linear(256, 128),
            nn.Tanh(),
            nn.Linear(128, self.latent_dim_pi),
            nn.Tanh(),
        )
        self.critic_combined_fc = nn.Sequential(
            nn.Linear(64, 256),
            nn.Tanh(),
            nn.Linear(256, 128),
            nn.Tanh(),
            nn.Linear(128, self.latent_dim_vf),
            nn.Tanh(),
        )

    def forward(self, observations: th.Tensor):
        actor_combined_features = self.multihead_actor(observations)
        critic_combined_features = self.multihead_critic(observations)

        actor_combined_lstm_output, _ = self.actor_combined_lstm(actor_combined_features)
        critic_combined_lstm_output, _ = self.critic_combined_lstm(critic_combined_features)
        
        latent_pi = self.forward_actor(actor_combined_lstm_output)
        latent_vf = self.forward_critic(critic_combined_lstm_output)
        return latent_pi, latent_vf

    def multihead_actor(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size : 3 * self.heightmap_size * self.heightmap_size+6+23]

        actor_cnn_output = self.actor_cnn(image_obs)
        actor_alip_mlp_output = self.actor_alip_mlp(alip_state)
        actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1).unsqueeze(1)
        return actor_combined_features

    def multihead_critic(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        image_obs_gt = observations[:, -3 * self.heightmap_size * self.heightmap_size:].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)

        alip_state = th.cat((observations[:, 3*self.heightmap_size*self.heightmap_size : 3*self.heightmap_size*self.heightmap_size+6], \
        observations[:, 3*self.heightmap_size*self.heightmap_size+6+23:3*self.heightmap_size*self.heightmap_size+6+23+23]), dim=1)

        critic_cnn_output = self.critic_cnn(image_obs)
        critic_cnn_output_gt = self.critic_cnn_gt(image_obs_gt)
        
        critic_alip_mlp_output = self.critic_alip_mlp(alip_state)
        critic_combined_features = th.cat((critic_cnn_output, critic_cnn_output_gt, critic_alip_mlp_output), dim=1).unsqueeze(1)
        return critic_combined_features

    def forward_actor(self, features: th.Tensor) -> th.Tensor:
        actor_actions = self.actor_combined_fc(features)
        return actor_actions

    def forward_critic(self, features: th.Tensor) -> th.Tensor:
        critic_actions = self.critic_combined_fc(features)
        return critic_actions
        
class CustomActorCriticPolicy(RecurrentActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        lstm_actor = CustomNetwork().actor_combined_lstm,
        lstm_critic = CustomNetwork().critic_combined_lstm,
        optimizer_class= th.optim.RAdam, #torch.optim.RAdam Try RAdam with decoupled_weight_decay=True
        optimizer_kwargs = {'weight_decay': 1e-3, 'epsilon': 1e-5},
        *args,
        **kwargs,
    ):
        kwargs["ortho_init"] = True
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            lstm_actor,
            lstm_critic,
            *args,
            **kwargs,
        )

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork()

def bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKSPACE_DIRECTORY'])

def _run_training(config, args):
    env_name = config["env_name"]
    num_env = config["num_workers"]
    log_dir = config["local_log_dir"]
    policy_type = config["policy_type"]
    total_timesteps = config["total_timesteps"]
    eval_freq = config["model_save_freq"]
    sim_params_train = config["sim_params"]
    sim_params_eval = config["sim_params"]
    
    if not args.test:
        input("Starting...")
        # if visualize is True -> TypeError: cannot pickle 'pydrake.geometry.Meshcat' object
        sim_params_train.visualize = False 
        sim_params_train.meshcat = None
        env = make_vec_env(
                           env_name,
                           n_envs=num_env,
                           seed=42,
                           vec_env_cls=SubprocVecEnv,
                           env_kwargs={
                            'sim_params': sim_params_train,
                           },
                           #vec_env_kwargs=dict(start_method="spawn")
                           )
        env = VecNormalize(venv=env, norm_obs=False)
        # check_env(env)
    else:
        input("Testing...")
        sim_params_train.visualize = True
        sim_params_train.meshcat = Meshcat()
        env = gym.make(env_name,
                       sim_params = sim_params_train,
                       )
        check_env(env)
    
    if args.test:
        model = RecurrentPPO(
            policy_type, env, learning_rate = linear_schedule(3e-4), n_steps=int(512/num_env), n_epochs=10,
            batch_size=16*num_env, ent_coef=0.01, use_sde=False, verbose=1,
            )
    else:
        tensorboard_log = f"{log_dir}runs/test"
        model_path = 'RPPO_mirror_noise12.zip' # x/logs2/rl_model_1728000_steps

        # model = RecurrentPPO(policy_type, env, learning_rate = 3e-4, max_grad_norm = 0.5, #linear_schedule(1e-5)
        #                 clip_range = 0.2, ent_coef=0.03, target_kl = 0.2,
        #                 n_steps=int(1024*num_env/num_env), n_epochs=10,
        #                 batch_size=64*num_env, seed=42, verbose=1,
        #                 tensorboard_log=tensorboard_log)

        model = RecurrentPPO.load(model_path, env, learning_rate = 3e-7, max_grad_norm = 0.3, # linear_schedule(3e-6)
                        clip_range = 0.05, ent_coef=0.003, target_kl = 0.005, vf_coef=0.05, clip_range_vf=None,
                        n_steps=int(512), n_epochs=5,
                        batch_size=64, seed=55, init_cnn_weights=False, # init_cnn_weights: Initialize critic CNN with Actor CNN
                        tensorboard_log=tensorboard_log)
        
        print("Open tensorboard (optional) via " f"`tensorboard --logdir {tensorboard_log}`" "in another terminal.")

    #sim_params_eval.visualize = True
    #sim_params_eval.meshcat = Meshcat()
    # sim_params_eval.visualize = False
    # sim_params_eval.meshcat = None
    #eval_env = gym.make(env_name, sim_params = sim_params_eval,)

    #eval_env = DummyVecEnv([lambda: eval_env])
    #eval_env = VecNormalize(venv=eval_env, norm_obs=False)
    #eval_callback = EvalCallback(
    #    eval_env,
    #    best_model_save_path=log_dir+f'eval_logs/test',
    #    log_path=log_dir+f'eval_logs/test',
    #    eval_freq=eval_freq,
    #    n_eval_episodes=3,
    #    deterministic=True,
    #    render=False)

    checkpoint_callback = CheckpointCallback(
        save_freq=eval_freq*0.5,
        save_path="./logs/",
        name_prefix="rl_model",
    )

    callback = CallbackList([checkpoint_callback])

    input("Start learning...")

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
    )

    model.save("latest_model")
    #eval_env.close()

def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--log_path', help="path to the logs directory.",
                        default="./rl/tmp/DrakeCassie/")
    args = parser.parse_args()

    if args.test:
        num_env = 1
    else:
        num_env = 64

    # https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html
    config = {
        "policy_type": CustomActorCriticPolicy,
        "total_timesteps": 20e6 if not args.test else 5000,
        "env_name": "DrakeCassie-v0",
        "num_workers": num_env,
        "local_log_dir": args.log_path,
        "model_save_freq": 5000,
        "sim_params" : sim_params
    }

    #th.set_num_threads(32)
    #print(th.get_num_threads())
    _run_training(config, args)

if __name__ == '__main__':
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv",
        )
    _main()
