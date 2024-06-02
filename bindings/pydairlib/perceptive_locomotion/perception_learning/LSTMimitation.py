"""
Imitate a policy from supervising LQR dataset //bindings/pydairlib/perceptive_locomotion/perception_learning.utils.DrakeCassieEnv
"""
import argparse
import os
from os import path

import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple

from tqdm import tqdm
import numpy as np
import torch as th
import torch.nn as nn
import torch.nn.utils as nn_utils
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.data.dataset import Dataset, random_split

from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.common.recurrent.type_aliases import RNNStates

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.evaluation import evaluate_policy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.policies import ActorCriticPolicy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
)

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

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.RPO.rpo import RPO
from pydairlib.perceptive_locomotion.systems.cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

#th.autograd.set_detect_anomaly(True)

def bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKSPACE_DIRECTORY'])

class ExpertDataSet(Dataset):
    def __init__(self, expert_observations, expert_actions):
        self.observations = expert_observations
        self.actions = expert_actions
        
    def __getitem__(self, index):
        return (self.observations[index], self.actions[index])

    def __len__(self):
        return len(self.observations)

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
        self.vector_state_actor = 6
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
            nn.Linear(self.vector_state_actor, 32),
            nn.Tanh(),
            nn.Linear(32, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
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
        self.actor_combined_lstm = nn.LSTM(input_size=128, hidden_size=128, num_layers=2, batch_first=False)
        self.actor_combined_fc = nn.Sequential(
            nn.Linear(128, 256),
            nn.Tanh(),
            nn.Linear(256, 128),
            nn.Tanh(),
            nn.Linear(128, self.latent_dim_pi),
            nn.Tanh(),
        )

        # LSTM layers for combined features for critic
        self.critic_combined_lstm = nn.LSTM(input_size=128, hidden_size=128, num_layers=2, batch_first=False)
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
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:3 * self.heightmap_size * self.heightmap_size+6]

        actor_cnn_output = self.actor_cnn(image_obs)
        actor_alip_mlp_output = self.actor_alip_mlp(alip_state)
        actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1).unsqueeze(1)
        return actor_combined_features

    def multihead_critic(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]

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

# LSTM forward is done in RecurrentActorCriticPolicy
class CustomActorCriticPolicy(RecurrentActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        lstm_actor = CustomNetwork().actor_combined_lstm,
        lstm_critic = CustomNetwork().critic_combined_lstm,
        lstm_hidden_size: int = 128,
        n_lstm_layers: int = 2,
        *args,
        **kwargs,
    ):

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

def split(expert_observations, expert_actions):
    expert_dataset = ExpertDataSet(expert_observations, expert_actions)

    train_size = int(1. * len(expert_dataset))

    test_size = len(expert_dataset) - train_size

    train_expert_dataset, test_expert_dataset = random_split(
        expert_dataset, [train_size, test_size]
    )
    return train_expert_dataset, test_expert_dataset

def pretrain_agent(
    student,
    env,
    train_expert_dataset,
    test_expert_dataset,
    train_batch_size=64,
    test_batch_size=64,
    epochs=10,
    scheduler_gamma=0.7,
    clip_grad_max_norm=0.5,
    learning_rate=1.0,
    log_interval=100,
    cuda=False,
    seed=1,
    patience = 15,
    min_delta=0.0001
):
    use_cuda = cuda and th.cuda.is_available()
    th.manual_seed(seed)
    device = th.device("cuda" if use_cuda else "cpu")
    kwargs = {"num_workers": 1, "pin_memory": True} if use_cuda else {}

    if isinstance(env.action_space, gym.spaces.Box):
      criterion = nn.MSELoss()
    else:
      criterion = nn.CrossEntropyLoss()

    model = student.policy.to(device)

    optimizer = optim.Adadelta(model.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    best_loss = float('inf')
    epochs_no_improve = 0

    def train(model, device, train_loader, optimizer):
        model.train()
        total_loss = []
        
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.to(device), target.to(device)
            
            optimizer.zero_grad()
            lstm_hidden_state_shape = (model.lstm_hidden_state_shape[0], 1, model.lstm_hidden_state_shape[2])
            
            states = RNNStates(
                (
                    th.zeros(lstm_hidden_state_shape, device=device),
                    th.zeros(lstm_hidden_state_shape, device=device),
                ),
                (
                    th.zeros(lstm_hidden_state_shape, device=device),
                    th.zeros(lstm_hidden_state_shape, device=device),
                ),
                )
            episode_starts = th.zeros(1).to(device)
            #episode_starts = th.ones(data.size(0)).to(device)
            #episode_starts[0] = 1

            action, _, _, _ = model(data, states, episode_starts)
            
            loss = criterion(action.to(th.float32), target.to(th.float32))
            loss.backward(retain_graph=True)
            optimizer.step()
            
            total_loss.append(loss.item())
    
            if batch_idx % log_interval == 0 and batch_idx > 0:
                current_loss = np.mean(total_loss[-log_interval:])
                print(f"Train Epoch: {epoch} [{batch_idx * len(data)}/{len(train_loader.dataset)} ({100.0 * batch_idx / len(train_loader):.0f}%)]\tLoss: {current_loss:.6f}")
            
        return np.mean(total_loss)

    def test(model, device, test_loader):
        model.eval()
        total_loss = 0.0
        data_len = len(test_loader.dataset)
        if data_len == 0:
            data_len = 1

        with th.no_grad():
            lstm_hidden_state_shape = (model.lstm_hidden_state_shape[0], 1, model.lstm_hidden_state_shape[2])
            initial_lstm_states = RNNStates(
            (
                th.zeros(lstm_hidden_state_shape, device=device),
                th.zeros(lstm_hidden_state_shape, device=device),
            ),
            (
                th.zeros(lstm_hidden_state_shape, device=device),
                th.zeros(lstm_hidden_state_shape, device=device),
            ),
            )

            for data, target in test_loader:
                data, target = data.to(device), target.to(device)

                # Initialize LSTM states at the beginning of each episode
                lstm_states = initial_lstm_states
                episode_starts = th.zeros(1).to(device)

                if isinstance(env.action_space, gym.spaces.Box):
                    action, _, _, _ = model(data, lstm_states, episode_starts)
                    action_prediction = action.double()
                else:
                    dist = model.get_distribution(data)
                    action_prediction = dist.distribution.logits
                target = target.long()

                test_loss = criterion(action_prediction, target)
                total_loss += test_loss.item()
        
        return total_loss / data_len

    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=train_batch_size, shuffle=False, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=False, **kwargs,
    )
                    
    for epoch in range(1, epochs + 1):
        train_loss = train(model, device, train_loader, optimizer)
        test_loss = test(model, device, test_loader)
        scheduler.step()

        # Early stopping
        if test_loss < best_loss - min_delta:
            best_loss = test_loss
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
        
        if epochs_no_improve >= patience:
            print(f"Stopping early at epoch {epoch}!")
            break

        print(f"Epoch {epoch}/{epochs}, Training Loss: {train_loss:.6f}, Test Loss: {test_loss:.6f}")


def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    sim_params.terrain = os.path.join(
        perception_learning_base_folder, 'params/flat/flat_100.yaml'
        #'params/stair_curriculum.yaml'
        #'params/wavy_test.yaml'
        #'params/wavy_terrain.yaml'
        #'params/flat.yaml'
    )
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv")  # noqa
    
    env = gym.make("DrakeCassie-v0",
                sim_params = sim_params,
                )

    # General State-Dependent Exploration does not work for imitation learning. (use_sde = False)
    student = RecurrentPPO(CustomActorCriticPolicy, env, use_sde=False, verbose=1)

    obs_data = path.join(perception_learning_base_folder, 'tmp/observations_all.npy')
    action_data = path.join(perception_learning_base_folder, 'tmp/actions_all.npy')
    
    expert_observations = np.load(obs_data)
    expert_actions = np.load(action_data)
    train_expert_dataset, test_expert_dataset = split(expert_observations, expert_actions)
    print(f"Total Dataset: {len(expert_observations)}, Train Dataset: {len(train_expert_dataset)}, Test Dataset: {len(test_expert_dataset)}")

    pretrain_agent(
        student,
        env,
        train_expert_dataset,
        test_expert_dataset,
        epochs=20,
        scheduler_gamma=0.7,
        learning_rate=.7,
        log_interval=100,
        cuda=True,
        seed=42,
        train_batch_size=400,
        test_batch_size=400,
    )

    student.save("RPPO_initialize")
    mean_reward, std_reward = evaluate_policy(student, env, n_eval_episodes=5)
    print(f"Mean reward = {mean_reward} +/- {std_reward}")

if __name__ == '__main__':
    th.cuda.empty_cache()
    _main()