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
from pydairlib.perceptive_locomotion.perception_learning.utils.adadelta import Adadelta
from torch.optim.lr_scheduler import StepLR
from torch.utils.data.dataset import Dataset, random_split

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.evaluation import evaluate_policy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.policies import ActorCriticPolicy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
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

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.RPO.rpo import RPO
from pydairlib.perceptive_locomotion.systems.cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"
th.cuda.empty_cache()
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

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias_const)
    return layer

resnet = th.hub.load('pytorch/vision:v0.10.0', 'resnet18', pretrained=False)
resnet.conv1 = nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1, bias=False)
#resnet.maxpool = nn.Identity()

class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 64, last_layer_dim_vf: int = 64):
        super().__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.alip_state_dim = 6
        self.heightmap_size = 80
        # CNN for heightmap observations
        n_input_channels = 3
        use_resnet = True
        
        if use_resnet:
            combined_input = 576
            self.actor_cnn = nn.Sequential(
                *list(resnet.children())[:-1],
                nn.Flatten(),
            )
            self.critic_cnn = nn.Sequential(
                *list(resnet.children())[:-1],
                nn.Flatten(),
            )
        else:
            combined_input = 3264
            self.actor_cnn = nn.Sequential(
                nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Flatten(),
            )
            self.critic_cnn = nn.Sequential(
                nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Flatten(),
            )

        # MLP for ALIP state + Vdes (6,)
        self.actor_alip_mlp = nn.Sequential(
            nn.Linear(self.alip_state_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 64),
            nn.Tanh(),
        )

        self.critic_alip_mlp = nn.Sequential(
            nn.Linear(self.alip_state_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 64),
            nn.Tanh(),
        )

        # Combined MLP for actor
        self.actor_combined_mlp = nn.Sequential(
            nn.Linear(combined_input, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, self.latent_dim_pi),
            nn.Tanh(),
        )

        # Combined MLP for critic
        self.critic_combined_mlp = nn.Sequential(
            nn.Linear(combined_input, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, self.latent_dim_vf),
            nn.Tanh(),
        )
        
    def forward(self, observations: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        return self.forward_actor(observations), self.forward_critic(observations)

    def forward_actor(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        actor_cnn_output = self.actor_cnn(image_obs)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]
        actor_alip_mlp_output = self.actor_alip_mlp(alip_state)
        actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1)
        actor_actions = self.actor_combined_mlp(actor_combined_features)
        return actor_actions

    def forward_critic(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        critic_cnn_output = self.critic_cnn(image_obs)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]
        critic_alip_mlp_output = self.critic_alip_mlp(alip_state)
        critic_combined_features = th.cat((critic_cnn_output, critic_alip_mlp_output), dim=1)
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

    train_size = int(0.85 * len(expert_dataset))

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
    cuda=False,
    seed=1,
    test_batch_size=64,
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
    optimizer = Adadelta(model.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    best_loss = float('inf')
    epochs_no_improve = 0

    def train(model, device, train_loader, optimizer):
        model.train()
        total_loss = []
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()

            if isinstance(env.action_space, gym.spaces.Box):
                # PPO policy outputs actions, values, log_prob
                action, _, _ = model(data)                
                action_prediction = action.double().to(device)
            else:
                # Retrieve the logits for PPO when using discrete actions
                dist = model.get_distribution(data)
                action_prediction = dist.distribution.logits
                target = target.long()

            target = target.to(action_prediction.device)
            loss = criterion(action_prediction, target)
            
            loss.backward(retain_graph=True)
            #nn_utils.clip_grad_norm_(model.parameters(), clip_grad_max_norm)
            optimizer.step()
            total_loss.append(loss.item())

            if batch_idx % log_interval == 0 and batch_idx > 0:
                current_loss = np.mean(total_loss[-log_interval:])
                print(f"Train Epoch: {epoch} [{batch_idx * len(data)}/{len(train_loader.dataset)} ({100.0 * batch_idx / len(train_loader):.0f}%)]\tLoss: {current_loss:.6f}")
        return np.mean(total_loss)

    def test(model, device, test_loader):
        model.eval()
        total_loss = 0.0
        with th.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)

                if isinstance(env.action_space, gym.spaces.Box):
                    action, _, _ = model(data)
                    action_prediction = action.double()
                else:
                    dist = model.get_distribution(data)
                    action_prediction = dist.distribution.logits
                target = target.long()

                test_loss = criterion(action_prediction, target)
                total_loss += test_loss.item()
        
        return total_loss / len(test_loader.dataset)

    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=batch_size, shuffle=True, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=True, **kwargs,
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
        perception_learning_base_folder, 'params/stair/flat_stair_100.yaml'
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
    student = PPO(CustomActorCriticPolicy, env, use_sde=False, verbose=1)

    obs_data = path.join(perception_learning_base_folder, 'tmp/observations.npy')
    action_data = path.join(perception_learning_base_folder, 'tmp/actions.npy')
    
    expert_observations = np.load(obs_data)
    expert_actions = np.load(action_data)
    train_expert_dataset, test_expert_dataset = split(expert_observations, expert_actions)
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
        seed=22,
        batch_size=128,
        test_batch_size=128,
    )

    student.save("PPO_initialize_resnet")
    mean_reward, std_reward = evaluate_policy(student, env, n_eval_episodes=5)
    print(f"Mean reward = {mean_reward} +/- {std_reward}")

if __name__ == '__main__':
    th.cuda.empty_cache()
    _main()