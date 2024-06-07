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
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias_const)
    return layer

# class CustomNetwork(nn.Module):
#     def __init__(self, last_layer_dim_pi: int = 64, last_layer_dim_vf: int = 64):
#         super().__init__()

#         self.latent_dim_pi = last_layer_dim_pi
#         self.latent_dim_vf = last_layer_dim_vf
#         self.vector_state_actor = 6
#         self.vector_state_critic = 6+23
#         self.heightmap_size = 64
#         # CNN for heightmap observations
#         n_input_channels = 3
#         self.actor_cnn = nn.Sequential(
#             nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Flatten(),
#         )
#         self.critic_cnn = nn.Sequential(
#             nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
#             nn.ReLU(),
#             nn.Flatten(),
#         )

#         # MLP for ALIP state + Vdes (6,)
#         self.actor_alip_mlp = nn.Sequential(
#             layer_init(nn.Linear(self.vector_state_actor, 128)),
#             nn.Tanh(),
#             layer_init(nn.Linear(128, 128)),
#             nn.Tanh(),
#             layer_init(nn.Linear(128, 64)),
#             nn.Tanh(),
#         )

#         self.critic_alip_mlp = nn.Sequential(
#             layer_init(nn.Linear(self.vector_state_critic, 128)),
#             nn.Tanh(),
#             layer_init(nn.Linear(128, 128)),
#             nn.Tanh(),
#             layer_init(nn.Linear(128, 64)),
#             nn.Tanh(),
#         )

#         # Combined MLP for actor
#         self.actor_combined_mlp = nn.Sequential(
#             layer_init(nn.Linear(2112, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, self.latent_dim_pi), std = 1.),
#             nn.Tanh(),
#         )

#         # Combined MLP for critic
#         self.critic_combined_mlp = nn.Sequential(
#             layer_init(nn.Linear(2112, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, 256)),
#             nn.Tanh(),
#             layer_init(nn.Linear(256, self.latent_dim_vf), std = 1.),
#             nn.Tanh(),
#         )
        
#     def forward(self, observations: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
#         return self.forward_actor(observations), self.forward_critic(observations)

#     def forward_actor(self, observations: th.Tensor) -> th.Tensor:
#         batch_size = observations.size(0)
#         image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
#         actor_cnn_output = self.actor_cnn(image_obs)
#         alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:3 * self.heightmap_size * self.heightmap_size+6]
#         actor_alip_mlp_output = self.actor_alip_mlp(alip_state)
#         actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1)
#         actor_actions = self.actor_combined_mlp(actor_combined_features)
#         return actor_actions

#     def forward_critic(self, observations: th.Tensor) -> th.Tensor:
#         batch_size = observations.size(0)
#         image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
#         critic_cnn_output = self.critic_cnn(image_obs)
#         alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]
#         critic_alip_mlp_output = self.critic_alip_mlp(alip_state)
#         critic_combined_features = th.cat((critic_cnn_output, critic_alip_mlp_output), dim=1)
#         critic_actions = self.critic_combined_mlp(critic_combined_features)
#         return critic_actions
    
# class CustomActorCriticPolicy(ActorCriticPolicy):
#     def __init__(
#         self,
#         observation_space: spaces.Space,
#         action_space: spaces.Space,
#         lr_schedule: Callable[[float], float],
#         optimizer_class= th.optim.Adam, #torch.optim.RAdam Try RAdam with decoupled_weight_decay=True
#         optimizer_kwargs = {'weight_decay': 1e-3, 'epsilon': 1e-5},
#         *args,
#         **kwargs,
#     ):
#         kwargs["ortho_init"] = True
#         super().__init__(
#             observation_space,
#             action_space,
#             lr_schedule,
#             *args,
#             **kwargs,
#         )

#     def _build_mlp_extractor(self) -> None:
#         self.mlp_extractor = CustomNetwork()

class ResidualBlock(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1, downsample=None):
        super(ResidualBlock, self).__init__()
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


class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 64, last_layer_dim_vf: int = 64):
        super().__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.vector_state_actor = 6+23
        self.vector_state_critic = 6+23
        self.heightmap_size = 64

        # CNN for heightmap observations
        n_input_channels = 3

        self.actor_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(32))),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=2, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(48))),

            ResidualBlock(48, 64, stride=2, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(64))),

            nn.Flatten(),
            nn.Linear(64, 1024),
            nn.Tanh(),
            nn.Linear(1024, 256),
            nn.Tanh(),
            nn.Linear(256, 96),
        )

        self.critic_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(32))),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=2, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(48))),

            ResidualBlock(48, 64, stride=2, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(64))),

            nn.Flatten(),
            nn.Linear(64, 1024),
            nn.Tanh(),
            nn.Linear(1024, 256),
            nn.Tanh(),
            nn.Linear(256, 96),
        )

        # MLP for ALIP state + Vdes (6,)
        self.actor_alip_mlp = nn.Sequential(
            nn.Linear(self.vector_state_actor, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 32),
            nn.Tanh(),
        )

        self.critic_alip_mlp = nn.Sequential(
            nn.Linear(self.vector_state_critic, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 32),
            nn.Tanh(),
        )

        # LSTM layers for combined features for actor
        self.actor_combined_lstm = nn.LSTM(input_size=128, hidden_size=128, num_layers=1, batch_first=False)
        self.actor_combined_fc = nn.Sequential(
            nn.Linear(128, 256),
            nn.Tanh(),
            nn.Linear(256, 128),
            nn.Tanh(),
            nn.Linear(128, self.latent_dim_pi),
            nn.Tanh(),
        )

        # LSTM layers for combined features for critic
        self.critic_combined_lstm = nn.LSTM(input_size=128, hidden_size=128, num_layers=1, batch_first=False)
        self.critic_combined_fc = nn.Sequential(
            nn.Linear(128, 256),
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
        alip_state = th.cat((observations[:, 3*self.heightmap_size*self.heightmap_size : 3*self.heightmap_size*self.heightmap_size+6],observations[:, -23:]), dim=1)

        critic_cnn_output = self.critic_cnn(image_obs)
        critic_alip_mlp_output = self.critic_alip_mlp(alip_state)
        critic_combined_features = th.cat((critic_cnn_output, critic_alip_mlp_output), dim=1).unsqueeze(1)
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
        optimizer_class= th.optim.Adam, #torch.optim.RAdam Try RAdam with decoupled_weight_decay=True
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
        # if visualize is True > TypeError: cannot pickle 'pydrake.geometry.Meshcat' object
        sim_params_train.visualize = False 
        sim_params_train.meshcat = None
        env = make_vec_env(
                           env_name,
                           n_envs=num_env,
                           seed=0,
                           vec_env_cls=SubprocVecEnv,
                           env_kwargs={
                            'sim_params': sim_params_train,
                           })
        env = VecNormalize(venv=env, norm_obs=False)

    else:
        input("Testing...")
        sim_params_train.visualize = False
        sim_params_train.meshcat = None
        env = gym.make(env_name,
                       sim_params = sim_params_train,
                       )
    
    if args.test:
        model = PPO(
            policy_type, env, learning_rate = linear_schedule(3e-4), n_steps=int(512/num_env), n_epochs=10,
            batch_size=16*num_env, ent_coef=0.01, use_sde=True, verbose=1,
            )
    else:
        tensorboard_log = f"{log_dir}runs/test"
        
        #test_folder = "rl/vdes_depth_angle_penalty"
        #model_path = path.join(test_folder, 'latest_model.zip')
        model_path = 'RPPO_initialize_no_batch_norm.zip'
        # model = RecurrentPPO(policy_type, env, learning_rate = 3e-5, max_grad_norm = 0.5, #linear_schedule(1e-5)
        #                 clip_range = 0.2, ent_coef=0.03, target_kl = 0.2,
        #                 n_steps=int(64*num_env/num_env), n_epochs=5,
        #                 batch_size=32*num_env, seed=42,
        #                 tensorboard_log=tensorboard_log)
        model = RecurrentPPO.load(model_path, env, learning_rate = 1e-5, max_grad_norm = 0.2, #linear_schedule(1e-5)
                        clip_range = 0.2, ent_coef=0.03, clip_range_vf=0.5, target_kl = .5,
                        n_steps=int(1024*num_env/num_env), n_epochs=10, # 400 | 100
                        batch_size=64*num_env, seed=1,
                        tensorboard_log=tensorboard_log)
        
        print("Open tensorboard (optional) via " f"`tensorboard --logdir {tensorboard_log}`" "in another terminal.")

    sim_params_eval.visualize = True
    sim_params_eval.meshcat = Meshcat()
    eval_env = gym.make(env_name, sim_params = sim_params_eval,)

    eval_env = DummyVecEnv([lambda: eval_env])
    eval_env = VecNormalize(venv=eval_env, norm_obs=False)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=log_dir+f'eval_logs/test',
        log_path=log_dir+f'eval_logs/test',
        eval_freq=eval_freq,
        n_eval_episodes=3,
        deterministic=True,
        render=False)

    checkpoint_callback = CheckpointCallback(
        save_freq=eval_freq*0.5,
        save_path="./logs/",
        name_prefix="rl_model",
    )

    callback = CallbackList([checkpoint_callback, eval_callback])

    input("Start learning...")

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
    )

    model.save("latest_model")
    eval_env.close()

def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--log_path', help="path to the logs directory.",
                        default="./rl/tmp/DrakeCassie/")
    args = parser.parse_args()

    if args.test:
        num_env = 1
    else:
        num_env = 30

    # https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html
    config = {
        "policy_type": CustomActorCriticPolicy,
        "total_timesteps": 5e6 if not args.test else 5,
        "env_name": "DrakeCassie-v0",
        "num_workers": num_env,
        "local_log_dir": args.log_path,
        "model_save_freq": 10000,
        "sim_params" : sim_params
    }
    _run_training(config, args)

gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv",
        )

if __name__ == '__main__':
    _main()
