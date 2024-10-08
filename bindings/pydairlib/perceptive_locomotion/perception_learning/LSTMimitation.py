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
from torch.nn.utils.rnn import pad_sequence
import torch.nn.utils as nn_utils
import torch.optim as optim

from torch.optim.lr_scheduler import StepLR
from torch.utils.data.dataset import Dataset, random_split
from torch.utils.data.sampler import Sampler

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

class TrajectorySampler(Sampler):
    def __init__(self, data_source):
        self.data_source = data_source

    def __iter__(self):
        expert_indices = list(range(0, len(self.data_source), 400))
        expert_indices = th.randperm(len(expert_indices)).tolist()
        indices = [i + j for i in expert_indices for j in range(400)]
        return iter(indices)

    def __len__(self):
        return len(self.data_source)

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
        super(CustomNetwork, self).__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.vector_state_actor = 6
        self.vector_state_critic = 6+23
        self.h_size = 64
        self.use_BN = False # Use BatchNorm2D
        if self.use_BN:
            ResidualBlock = ResidualBlock_Norm
        else:
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

        # self.actor_multitask = nn.Linear(self.latent_dim_pi, 16)

    def forward(self, observations: th.Tensor):
        actor_combined_features = self.multihead_actor(observations)
        critic_combined_features = self.multihead_critic(observations)

        actor_combined_lstm_output, _ = self.actor_combined_lstm(actor_combined_features)
        critic_combined_lstm_output, _ = self.critic_combined_lstm(critic_combined_features)
        
        return actor_combined_lstm_output, critic_combined_lstm_output

    def multihead_actor(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3*self.h_size*self.h_size].reshape(batch_size, 3, self.h_size, self.h_size)
        state = observations[:, 3*self.h_size*self.h_size : 3*self.h_size*self.h_size+6]

        actor_cnn_output = self.actor_cnn(image_obs)
        actor_combined_features = th.cat((actor_cnn_output, state), dim=1).unsqueeze(1)
        return actor_combined_features

    def multihead_critic(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs_gt = observations[:, -3*self.h_size*self.h_size:].reshape(batch_size, 3, self.h_size, self.h_size)
        state = th.cat((observations[:, 3*self.h_size*self.h_size : 3*self.h_size*self.h_size+6], \
        observations[:, 3*self.h_size*self.h_size+6:3*self.h_size*self.h_size+6+23]), dim=1)
        critic_cnn_output_gt = self.critic_cnn_gt(image_obs_gt)
        critic_combined_features = th.cat((critic_cnn_output_gt, state), dim=1).unsqueeze(1)

        return critic_combined_features

# LSTM forward is done in RecurrentActorCriticPolicy
class CustomActorCriticPolicy(RecurrentActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        lstm_hidden_size: int = 64,
        n_lstm_layers: int = 2,
        optimizer_class= th.optim.Adam, #th.optim.RAdam,
        optimizer_kwargs = None,#{'weight_decay': 1e-4, 'epsilon': 1e-5},
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

def split(expert_observations, expert_actions, ratio=1.0, rand_split=False):
    expert_dataset = ExpertDataSet(expert_observations, expert_actions)
    total_len = expert_dataset.__len__()

    train_size = int((ratio * (total_len // 400))) * 400

    test_size = len(expert_dataset) - train_size
    
    if rand_split:
        train_expert_dataset, test_expert_dataset = random_split(
            expert_dataset, [train_size, test_size]
        )
    else:
        train_expert_dataset = ExpertDataSet(
            expert_observations[:train_size],
            expert_actions[:train_size]
        )
        test_expert_dataset = ExpertDataSet(
            expert_observations[train_size:],
            expert_actions[train_size:]
        )
    print(f'Number of Trajectories in Train: {len(train_expert_dataset) // 400}, Test: {len(test_expert_dataset) // 400} ...')

    return train_expert_dataset, test_expert_dataset

def pretrain_agent(
    student,
    env,
    train_expert_dataset,
    test_expert_dataset,
    train_batch_size=400,
    test_batch_size=400,
    epochs=15,
    scheduler_gamma=0.7,
    clip_grad_max_norm=0.5,
    learning_rate=1.0,
    log_interval=100,
    cuda=False,
    seed=42,
    patience = 5,
    min_delta=0.0000000001
):
    noise = False
    MTL = False # Multi-task Learning
    use_cuda = cuda and th.cuda.is_available()
    th.manual_seed(seed)
    device = th.device("cuda" if use_cuda else "cpu")
    kwargs = {"num_workers": 1, "pin_memory": True} if use_cuda else {}

    if isinstance(env.action_space, gym.spaces.Box):
      criterion = nn.MSELoss()
    else:
      criterion = nn.CrossEntropyLoss()

    model = student.policy.to(device)
    # print("Parameter sizes:")
    # total_params = 0
    # for name, param in model.named_parameters():
    #     num_params = param.numel()
    #     total_params += num_params
    #     print(f"{name}: {num_params} parameters")

    # print(f"Total number of parameters: {total_params}")
    print(model)
    optimizer = optim.Adadelta(model.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    best_loss = float('inf')
    epochs_no_improve = 0
    input('Start Training...')
    def train(model, device, train_loader, optimizer):
        model.train()
        total_loss = []
        total_multi_loss = []
        # total_mirror_loss = []
        sequence_length = 64
        lstm_hidden_state_shape = (model.lstm_hidden_state_shape[0], 1, model.lstm_hidden_state_shape[2]) # 2,1,64
        
        for batch_idx, (data, target) in enumerate(train_loader):
            # Inject noise
            if noise:
                camera_episode_noise = np.random.uniform(low=-0.02, high=0.02)
                camera_step_noise = np.random.uniform(low=-0.01, high=0.01, size=(data.size(0), 64*64))
                data[:, 2*64*64:3*64*64] = data[:, 2*64*64:3*64*64] + camera_episode_noise + camera_step_noise

                # alipxy_noise = np.random.uniform(low=-0.05, high=0.05, size=(data.size(0), 2)) 
                # aliplxly_noise = np.random.uniform(low=-0.05, high=0.05, size=(data.size(0), 2)) # 20%
                # vdes_noise = np.random.uniform(low=-0.05, high=0.05, size=(data.size(0), 2))
                # angle_noise = np.random.uniform(low=-0.05, high=0.05, size=(data.size(0), 16))
                # data[:, 3*64*64:3*64*64+4] = data[:, 3*64*64:3*64*64+4] + np.hstack((alipxy_noise, aliplxly_noise))
                # data[:, 3*64*64+4:3*64*64+4+2] = data[:, 3*64*64+4:3*64*64+4+2] + vdes_noise
                # data[:, 3*64*64+6:3*64*64+6+16] = data[:, 3*64*64+6:3*64*64+6+16] + angle_noise
            scaling_factor = np.array([2, 2, 4])
            target = target * scaling_factor
            data, target = data.to(device), target.to(device)
            
            ### Mirror ###
            # image = data[:, 2*64*64:3*64*64].clone()
            # image = image.view(-1, 64, 64)
            # image = th.flip(image, [1])
            # image = image.view(-1, 64*64)
            # y_pos = data[:, 64*64:2*64*64].clone()
            # y_pos = y_pos.view(-1, 64, 64)
            # y_pos = th.flip(-y_pos, [1])
            # y_pos = y_pos.view(-1, 64*64)

            # obs = data[:, 3*64*64:3*64*64+6+16].clone()
            # ALIP, vdes, joint = obs[:, :4].clone(), obs[:, 4:6].clone(), obs[:, -16:].clone()
            # ALIP[:, 1:3] = -ALIP[:, 1:3]
            # vdes[:, 1] = -vdes[:, 1]
            # joint[:, [0,1,2,3,4,5,6,7]], joint[:, [8,9,10,11,12,13,14,15]] = joint[:, [8,9,10,11,12,13,14,15]], joint[:, [0,1,2,3,4,5,6,7]]
            # joint[:, [0,1]], joint[:, [8,9]] = -joint[:, [0,1]], -joint[:, [8,9]] # Negate hip roll & hip yaw
            # mirror_data = th.cat((data[:, :64*64].clone(), y_pos, image, ALIP, vdes, joint, data[:, 3*64*64+6+16:].clone()), dim=1)
            
            batch_size = data.size(0)

            sequences = [data[i:i+sequence_length] for i in range(0, batch_size, sequence_length)]
            target_sequences = [target[i:i+sequence_length] for i in range(0, batch_size, sequence_length)]
            
            ### Mirror ###
            # mirror_sequences = [mirror_data[i:i+sequence_length] for i in range(0, batch_size, sequence_length)]
            # mirror_sequences = pad_sequence(mirror_sequences)

            sequences = pad_sequence(sequences)
            target_sequences = pad_sequence(target_sequences)
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

            # mirror_states = RNNStates(
            #         (
            #             th.zeros(lstm_hidden_state_shape, device=device),
            #             th.zeros(lstm_hidden_state_shape, device=device),
            #         ),
            #         (
            #             th.zeros(lstm_hidden_state_shape, device=device),
            #             th.zeros(lstm_hidden_state_shape, device=device),
            #         ),
            #         )

            optimizer.zero_grad()
            batch_loss = 0.0
            # batch_mirror_loss = 0.0
            if MTL:
                batch_multi_loss = 0.0

            for i in range(sequences.size(1)):
                sequence_data = sequences[:, i, :]
                sequence_target = target_sequences[:, i, :]
                ### Mirror ###
                # mirror_sequence_data = mirror_sequences[:, i, :]

                if MTL:
                    multi_target = sequence_data[:, -16:]

                if i == 0:
                    episode_starts = th.zeros(sequence_length, device=device)
                    episode_starts[0] = 1
                else:
                    episode_starts = th.zeros(sequence_length, device=device)
                
                if MTL:
                    action, _, _, states, multi_task = model(sequence_data, states, episode_starts, deterministic=True) # True when not changing the std
                else:
                    action, _, _, states = model.forward(sequence_data, states, episode_starts, deterministic=True)
                
                ### Mirror ###
                # mirror_action, _, _, mirror_states = model.forward(mirror_sequence_data, mirror_states, episode_starts, deterministic=True)
                # mirror_action[:, 1] = -mirror_action[:, 1]

                if (i+1 == sequences.size(1)):
                    action = action[:16]
                    # mirror_action = mirror_action[:16]
                    sequence_target = sequence_target[:16]
                    if MTL:
                        multi_task = multi_task[:16]
                        multi_target = multi_target[:16]

                loss = criterion(action.to(th.float32), sequence_target.to(th.float32))
                # mirror_loss = criterion(mirror_action.to(th.float32), sequence_target.to(th.float32))

                # if MTL:
                #     multi_loss = criterion(multi_task.to(th.float32) , multi_target.to(th.float32))
                #     batch_loss += loss + 0.3 * multi_loss + 0.3 * mirror_loss
                #     batch_multi_loss += 0.3 * multi_loss
                #     batch_mirror_loss += 0.3 * mirror_loss
                # else:
                #     batch_loss += loss + mirror_loss
                #     batch_mirror_loss += mirror_loss
                batch_loss += loss
            batch_loss.backward()
            optimizer.step()
            total_loss.append(batch_loss.item() / sequences.size(1))

            # total_mirror_loss.append(batch_mirror_loss.item() / sequences.size(1))

            if MTL:
                total_multi_loss.append(batch_multi_loss.item() / sequences.size(1))

            if batch_idx % log_interval == 0 and batch_idx > 0:
                current_loss = np.mean(total_loss[-log_interval:])
                current_multi_loss = np.mean(total_multi_loss[-log_interval:])
                # current_mirror_loss = np.mean(total_mirror_loss[-log_interval:])
                if MTL:
                    print(f"Train Epoch: {epoch} [{batch_idx * data.size(0)}/{len(train_loader.dataset)} "
                    f"({100.0 * batch_idx / len(train_loader):.0f}%)]\tLoss: {current_loss:.6f}\tLoss: {current_multi_loss*0.1:.6f}")
                else:
                    print(f"Train Epoch: {epoch} [{batch_idx * data.size(0)}/{len(train_loader.dataset)} "
                    f"({100.0 * batch_idx / len(train_loader):.0f}%)]\tLoss: {current_loss:.6f}")# Mirror Loss: {current_mirror_loss:.6f}

        return np.mean(total_loss)

    def test(model, device, test_loader):
        model.eval()
        total_loss = 0.0
        data_len = len(test_loader.dataset)
        if data_len == 0:
            data_len = 1

        with th.no_grad():
            sequence_length = 64
            lstm_hidden_state_shape = (model.lstm_hidden_state_shape[0], 1, model.lstm_hidden_state_shape[2])
            
            for batch_idx, (data, target) in enumerate(test_loader):
                scaling_factor = np.array([2, 2, 4])
                target = target * scaling_factor
                data, target = data.to(device), target.to(device)

                batch_size = data.size(0)
                
                sequences = [data[i:i+sequence_length] for i in range(0, batch_size, sequence_length)]
                target_sequences = [target[i:i+sequence_length] for i in range(0, batch_size, sequence_length)]
                sequences = pad_sequence(sequences, batch_first=False)
                target_sequences = pad_sequence(target_sequences, batch_first=False)
                
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

                for i in range(sequences.size(1)):
                    sequence_data = sequences[:, i, :]
                    sequence_target = target_sequences[:, i, :]
                    if MTL:
                        multi_target = sequence_data[:, -16:]

                    if i == 0:
                        episode_starts = th.zeros(sequence_length, device=device)
                        episode_starts[0] = 1
                    else:
                        episode_starts = th.zeros(sequence_length, device=device)
                    
                    if MTL:
                        action, _, _, states, multi_task = model(sequence_data, states, episode_starts,deterministic=True)
                    else:
                        action, _, _, states = model.forward(sequence_data, states, episode_starts,deterministic=True)
                    action_prediction = action.double()

                    if (i+1 == sequences.size(1)):
                        action_prediction = action_prediction[:16]
                        sequence_target = sequence_target[:16]
                        if MTL:
                            multi_task = multi_task[:16]
                            multi_target = multi_target[:16]

                    loss = criterion(action_prediction, sequence_target)
                    if MTL:
                        multi_loss = criterion(multi_task.to(th.float32) , multi_target.to(th.float32))
                        total_loss += loss + 0.3 * multi_loss
                    else:
                        total_loss += loss
        
        return total_loss / data_len

    # train_loader = th.utils.data.DataLoader(
    #     dataset=train_expert_dataset, batch_size=train_batch_size, sampler=TrajectorySampler(train_expert_dataset), **kwargs
    # )
    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=train_batch_size, shuffle=False, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=False, **kwargs,
    )
                    
    for epoch in range(1, epochs + 1):
        train_loss = train(model, device, train_loader, optimizer)
        test_loss = test(model, device, test_loader)
        # scheduler.step()
        # student.save(f"RPPO_initialize_{epoch}")
        
        # Early stopping
        if test_loss < best_loss - min_delta:
            best_loss = test_loss
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
        
        if epochs_no_improve >= patience:
            print(f"Stopping early at epoch {epoch}!")
            break

        print(f"Epoch {epoch}/{epochs}, Training Loss: {train_loss:.7f}, Test Loss: {test_loss:.7f}")


def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    sim_params.terrain = os.path.join(perception_learning_base_folder, 'params/flat/flat_0.yaml')

    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv") # noqa
    
    env = gym.make("DrakeCassie-v0",
                sim_params = sim_params,
                )

    # General State-Dependent Exploration does not work for imitation learning. (use_sde = False)
    student = RecurrentPPO(CustomActorCriticPolicy, env, use_sde=False, verbose=1)
    # student = RecurrentPPO.load('64_ALIP.zip', env)
    obs_data = path.join(perception_learning_base_folder, 'tmp/data_collection/observation.npy')
    action_data = path.join(perception_learning_base_folder, 'tmp/data_collection/action.npy')

    expert_observations = np.load(obs_data)
    expert_actions = np.load(action_data)
    expert_observations = np.hstack((expert_observations, np.zeros((expert_observations.shape[0], 23))))
    print(expert_observations.shape)

    train_expert_dataset, test_expert_dataset = split(expert_observations, expert_actions, ratio=0.9)
    print(f"Total Dataset: {len(expert_observations)}, Train Dataset: {len(train_expert_dataset)}, Test Dataset: {len(test_expert_dataset)}")

    pretrain_agent(
        student,
        env,
        train_expert_dataset,
        test_expert_dataset,
        epochs=100,
        scheduler_gamma=0.8,
        learning_rate=1.,
        log_interval=100,
        cuda=True,
        seed=77,
        train_batch_size=400,
        test_batch_size=400,
        patience=40,
    )

    student.save("64_ALIP")
    mean_reward, std_reward = evaluate_policy(student, env, n_eval_episodes=5)
    print(f"Mean reward = {mean_reward} +/- {std_reward}")

if __name__ == '__main__':
    th.cuda.empty_cache()
    _main()