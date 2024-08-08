"""
Train a policy for //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
"""
import argparse
import os
from os import path

import numpy as np
import torch as th
import torch.nn as nn

import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple

import gymnasium as gym

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.ppo_recurrent.ppo_recurrent import RecurrentPPO
from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.common.recurrent.policies import RecurrentActorCriticPolicy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.utils import get_schedule_fn

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.callbacks import EvalCallback
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecVideoRecorder,
)

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

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

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

class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 64, last_layer_dim_vf: int = 64):
        super(CustomNetwork, self).__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.vector_state_actor = 6+16
        self.vector_state_critic = 6+23
        self.h_size = 64
        ResidualBlock = ResidualBlock_noNorm

        n_input_channels = 3

        self.actor_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=1, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=1, bias=False),
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(48, 64, stride=1, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                )),

            nn.Flatten(),
            nn.Linear(64*2*2, 512),
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 64),
        )

        self.critic_cnn_gt = nn.Sequential(
            nn.Conv2d(n_input_channels, 16, kernel_size=4, stride=2, padding=1),
            nn.LeakyReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(16, 32, stride=2, downsample=nn.Sequential(
                nn.Conv2d(16, 32, kernel_size=1, stride=2, bias=False),
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(32, 48, stride=1, downsample=nn.Sequential(
                nn.Conv2d(32, 48, kernel_size=1, stride=1, bias=False),
                )),
            nn.MaxPool2d(kernel_size=2, stride=2),

            ResidualBlock(48, 64, stride=1, downsample=nn.Sequential(
                nn.Conv2d(48, 64, kernel_size=1, stride=2, bias=False),
                )),

            nn.Flatten(),
            nn.Linear(64*2*2, 512),
            nn.Tanh(),
            nn.Linear(512, 256),
            nn.Tanh(),
            nn.Linear(256, 64),
        )

        self.actor_combined_lstm = nn.LSTM(input_size=self.vector_state_actor + 64, hidden_size=64, num_layers=2, batch_first=False)
        self.critic_combined_lstm = nn.LSTM(input_size=self.vector_state_critic + 64, hidden_size=64, num_layers=2, batch_first=False)

    def forward(self, observations: th.Tensor):
        actor_combined_features = self.multihead_actor(observations)
        critic_combined_features = self.multihead_critic(observations)
        actor_combined_lstm_output, _ = self.actor_combined_lstm(actor_combined_features)
        critic_combined_lstm_output, _ = self.critic_combined_lstm(critic_combined_features)
        return actor_combined_lstm_output, critic_combined_lstm_output

    def multihead_actor(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3*self.h_size*self.h_size].reshape(batch_size, 3, self.h_size, self.h_size)
        state = observations[:, 3*self.h_size*self.h_size : 3*self.h_size*self.h_size+6+16]
        actor_cnn_output = self.actor_cnn(image_obs)
        actor_combined_features = th.cat((actor_cnn_output, state), dim=1).unsqueeze(1)
        return actor_combined_features

    def multihead_critic(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs_gt = observations[:, -3*self.h_size*self.h_size:].reshape(batch_size, 3, self.h_size, self.h_size)
        state = th.cat((observations[:, 3*self.h_size*self.h_size : 3*self.h_size*self.h_size+6], \
        observations[:, 3*self.h_size*self.h_size+6+16:3*self.h_size*self.h_size+6+16+23]), dim=1)
        critic_cnn_output_gt = self.critic_cnn_gt(image_obs_gt)
        critic_combined_features = th.cat((critic_cnn_output_gt, state), dim=1).unsqueeze(1)
        return critic_combined_features

class CustomActorCriticPolicy(RecurrentActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        lstm_hidden_size: int = 64,
        n_lstm_layers: int = 2,
        optimizer_class= th.optim.Adam,
        optimizer_kwargs = {'weight_decay': 1e-4, 'epsilon': 1e-5},
        *args,
        **kwargs,
    ):

        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            lstm_hidden_size = 64,
            n_lstm_layers = 2,
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

def sample(sim_params):
    terrain = 'params/stair_curriculum.yaml'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
    env = gym.make("DrakeCassie-v0",
        sim_params = sim_params,
    )
    rate = 1.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 1000
    obs, _ = env.reset()
    input("Start..")
    for i in range(int(max_steps)):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            input("The environment will reset. Press Enter to continue...")
            obs, _ = env.reset()

def run_play(sim_params, model_path=None):
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    # rate = 1.0
    # env.simulator.set_target_realtime_rate(rate)
    max_steps = 3e4
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)

    #model_path = 'RPPO_mirror_noise.zip'
    #model_path = 'RPPO_003_1.zip'
    model_path = 'rl_model_10260000_steps.zip'
    #model_path = 'logs/rl_model_6720000_steps.zip'
    
    model = RecurrentPPO.load(model_path, env, verbose=1)
    #th.save(model.policy.state_dict(), 'test')
    #model.load_state_dict(th.load('test'))

    obs, _ = env.reset()
    input("Start..")
    total_reward = 0
    model.policy.eval()
    for i in range(int(max_steps)):
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, states = model.predict(obs, deterministic=True)
        
        obs, reward, terminated, truncated, info = env.step(action)
        if lstm:
            episode_starts = terminated
            
        total_reward += reward
        if terminated or truncated:
            print(total_reward)
            if lstm:
                lstm_states = None
                episode_starts = np.ones((1,), dtype=bool)
            obs, _ = env.reset()
            total_reward = 0

def load_without_dependencies():
    max_steps = 3e4
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)

    #model_path = 'RPPO_mirror_noise.zip'
    #model_path = 'RPPO_003_1.zip'
    model_path = 'rl_model_9120000_steps.zip'
    #model_path = 'logs/rl_model_6720000_steps.zip'

    action_space = spaces.Box(low=np.asarray(np.array([-1., -1., -1.]), dtype="float32"),
                                  high=np.asarray(np.array([1., 1., 1.]), dtype="float32"),
                                  dtype=np.float32)
                                  
    observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                    shape=(3*64*64 +6+16+23 +3*64*64,),
                                    dtype=np.float32)
    model = CustomActorCriticPolicy(observation_space,action_space,get_schedule_fn(1.))

    model.load_state_dict(th.load('test')) # Save policy through -> th.save(model.policy.state_dict(), 'test')

    input("Start..")

    model.eval()

    obs = np.load('observation.npy')
    for i in range(int(max_steps)):
        action, lstm_states = model.predict(obs[i], state=lstm_states, episode_start=episode_starts, deterministic=True)
        print(action)
        input("==")

        reward = 0
        terminated = 0
        truncated = 0

        episode_starts = terminated

def _main(model_path=None):
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv")  # noqa

    #sample(sim_params)
    run_play(sim_params, model_path=None)
    #load_without_dependencies()

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--model_path',
        type=str,
        default=None,
    )
    args = parser.parse_args()

    _main(args.model_path)
