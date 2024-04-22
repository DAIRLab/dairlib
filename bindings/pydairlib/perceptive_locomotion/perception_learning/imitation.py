"""
Imitate a policy from supervising LQR dataset //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
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

from stable_baselines3 import A2C, SAC, TD3

from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
)
_full_sb3_available = True

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

from pydairlib.perceptive_locomotion.perception_learning.PPO.ppo import PPO
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

class ReLUSquared(nn.Module):
    def forward(self, input):
        return th.relu(input) ** 2

class QReLU(nn.Module):
    def __init__(self, a=0.1):
        super(QReLU, self).__init__()
        self.a = a

    def forward(self, x):
        return th.max(0, x) + self.a * th.max(0, -x)**2

class ExpertDataSet(Dataset):
    def __init__(self, expert_observations, expert_actions):
        self.observations = expert_observations
        self.actions = expert_actions
        
    def __getitem__(self, index):
        return (self.observations[index], self.actions[index])

    def __len__(self):
        return len(self.observations)

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias_const)
    return layer

class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 128, last_layer_dim_vf: int = 128):
        super().__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # CNN for heightmap observations
        n_input_channels = 3
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )
        self.critic_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )

        # MLP for ALIP state
        alip_state_dim = 4
        self.actor_alip_mlp = nn.Sequential(
            layer_init(nn.Linear(alip_state_dim, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 64)),
            nn.Tanh(),
        )

        alip_state_dim = 4
        self.critic_alip_mlp = nn.Sequential(
            layer_init(nn.Linear(alip_state_dim, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 64)),
            nn.Tanh(),
        )

        # Combined MLP for actor
        self.actor_combined_mlp = nn.Sequential(
            layer_init(nn.Linear(320, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, self.latent_dim_pi), std = 0.1),
            nn.Tanh(),
        )

        # Combined MLP for critic
        self.critic_combined_mlp = nn.Sequential(
            layer_init(nn.Linear(320, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, self.latent_dim_vf), std = 1.),
            nn.Tanh(),
        )
        # Initialize the weights using Xavier initialization
        #self._initialize_weights()

    #def _initialize_weights(self):
    #    for m in self.modules():
    #        if isinstance(m, nn.Linear):
    #            nn.init.xavier_uniform_(m.weight)
    #            nn.init.constant_(m.bias, 0.0)
        
    def forward(self, observations: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        return self.forward_actor(observations), self.forward_critic(observations)

    def forward_actor(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, 0:3 * 64 * 64].reshape(batch_size, 3, 64, 64)
        actor_cnn_output = self.actor_cnn(image_obs)  # Shape: (batch_size, 2304)
        alip_state = observations[:, 3 * 64 * 64:]
        actor_alip_mlp_output = self.actor_alip_mlp(alip_state)  # Shape: (batch_size, 64)
        actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1)  # Concatenate along feature dimension
        actor_actions = self.actor_combined_mlp(actor_combined_features)
        return actor_actions

    def forward_critic(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, 0:3 *64 * 64].reshape(batch_size, 3, 64, 64)
        critic_cnn_output = self.critic_cnn(image_obs)  # Shape: (batch_size, 2304)
        alip_state = observations[:, 3 *64 * 64:]
        critic_alip_mlp_output = self.critic_alip_mlp(alip_state)  # Shape: (batch_size, 64)
        critic_combined_features = th.cat((critic_cnn_output, critic_alip_mlp_output), dim=1)  # Concatenate along feature dimension
        critic_actions = self.critic_combined_mlp(critic_combined_features)
        return critic_actions

class CustomActorCriticPolicy(ActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        *args,
        **kwargs,
    ):

        kwargs["ortho_init"] = True
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            *args,
            **kwargs,
        )

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork()

def split(expert_observations, expert_actions):
    expert_dataset = ExpertDataSet(expert_observations, expert_actions)

    train_size = int(0.95 * len(expert_dataset))

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
    batch_size=64,
    epochs=10,
    scheduler_gamma=0.7,
    clip_grad_max_norm=0.5,
    learning_rate=1.0,
    log_interval=100,
    no_cuda=True,
    seed=1,
    test_batch_size=64,
):
    use_cuda = not no_cuda and th.cuda.is_available()
    th.manual_seed(seed)
    device = th.device("cuda" if use_cuda else "cpu")
    kwargs = {"num_workers": 1, "pin_memory": True} if use_cuda else {}

    if isinstance(env.action_space, gym.spaces.Box):
      criterion = nn.MSELoss()
    else:
      criterion = nn.CrossEntropyLoss()

    # Extract initial policy
    model = student.policy.to(device)

    def train(model, device, train_loader, optimizer):
        model.train()
        total_loss = []
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()

            if isinstance(env.action_space, gym.spaces.Box):
              # A2C/PPO policy outputs actions, values, log_prob
              # SAC/TD3 policy outputs actions only
              if isinstance(student, (A2C, PPO)):
                action, _, _ = model(data)
              else:
                # SAC/TD3:
                action = model(data)
              action_prediction = action.double()
            else:
              # Retrieve the logits for A2C/PPO when using discrete actions
              dist = model.get_distribution(data)
              action_prediction = dist.distribution.logits
              target = target.long()

            loss = criterion(action_prediction, target)
            loss.backward()
            #loss.backward(retain_graph=True)
            total_loss.append(loss.item())
            
            nn_utils.clip_grad_norm_(model.parameters(), clip_grad_max_norm)
            optimizer.step()
            if batch_idx % log_interval == 0:
                print(
                    "Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}".format(
                        epoch,
                        batch_idx * len(data),
                        len(train_loader.dataset),
                        100.0 * batch_idx / len(train_loader),
                        loss.item(),
                    )
                )
            if batch_idx == len(train_loader)-1:
                print(np.sum(total_loss)/(len(train_loader)-1))

    def test(model, device, test_loader):
        model.eval()
        test_loss = 0
        with th.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)

                if isinstance(env.action_space, gym.spaces.Box):
                  # A2C/PPO policy outputs actions, values, log_prob
                  # SAC/TD3 policy outputs actions only
                  if isinstance(student, (A2C, PPO)):
                    action, _, _ = model(data)
                  else:
                    # SAC/TD3:
                    action = model(data)
                  action_prediction = action.double()
                else:
                  # Retrieve the logits for A2C/PPO when using discrete actions
                  dist = model.get_distribution(data)
                  action_prediction = dist.distribution.logits
                  target = target.long()

                test_loss = criterion(action_prediction, target)
        test_loss /= len(test_loader.dataset)

    # Here, we use PyTorch `DataLoader` to our load previously created `ExpertDataset` for training and testing
    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=batch_size, shuffle=True, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=True, **kwargs,
    )

    # Define an Optimizer and a learning rate schedule.
    optimizer = optim.Adadelta(model.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    # Now we are finally ready to train the policy model.
    for epoch in range(1, epochs + 1):
        train(model, device, train_loader, optimizer)
        test(model, device, test_loader)
        scheduler.step()

def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = os.path.join(
        perception_learning_base_folder, 'params/stair_curriculum.yaml'#'params/stair_curriculum.yaml'#'params/wavy_test.yaml' #'params/wavy_terrain.yaml' # 'params/flat.yaml'
    )
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.DrakeCassieEnv:DrakeCassieEnv")  # noqa
    
    env = gym.make("DrakeCassie-v0",
                sim_params = sim_params,
                )

    student = PPO(CustomActorCriticPolicy, env, verbose=1)#, use_sde=True)

    obs_data = path.join(perception_learning_base_folder, 'tmp/data/observationsNN_new.npy')
    action_data = path.join(perception_learning_base_folder, 'tmp/data/actionsNN_new.npy')
    
    expert_observations = np.load(obs_data)
    print(len(expert_observations))
    expert_actions = np.load(action_data)

    train_expert_dataset, test_expert_dataset = split(expert_observations, expert_actions)
    print(len(train_expert_dataset))
    print(len(test_expert_dataset))
    
    pretrain_agent(
        student,
        env,
        train_expert_dataset,
        test_expert_dataset,
        epochs=25,
        scheduler_gamma=0.8,
        learning_rate=1.5,
        log_interval=100,
        no_cuda=False,
        seed=64,
        batch_size=64,
        test_batch_size=64,
    )

    student.save("PPO_studentNN_newdata")
    mean_reward, std_reward = evaluate_policy(student, env, n_eval_episodes=2)
    print(f"Mean reward = {mean_reward} +/- {std_reward}")

if __name__ == '__main__':
    _main()