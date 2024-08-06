import numpy as np
import torch as th
import torch.nn as nn
import argparse
import os
from os import path
import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple

import io
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.common.value import Value
from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
    BasicVector,
    LeafSystem,
    Context,
    InputPort,
    OutputPort
)

from pydairlib.systems.footstep_planning import (
    AlipStepToStepDynamics,
    MassNormalizedAlipStepToStepDynamics,
    ResetDiscretization,
    AlipGaitParams,
    CalcAd,
    CalcA,
    CalcMassNormalizedAd,
    CalcMassNormalizedA,
    Stance,
    CalcAlipStateInBodyYawFrame,
)

from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.common.recurrent.policies import RecurrentActorCriticPolicy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.utils import get_schedule_fn

from pydairlib.perceptive_locomotion.systems.elevation_map_converter import ElevationMapQueryObject

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

@dataclass
class AlipFootstepLQROptions:
    height: float
    mass: float
    stance_width: float
    single_stance_duration: float
    double_stance_duration: float
    reset_discretization: ResetDiscretization = ResetDiscretization.kFOH
    Q: np.ndarray = field(default_factory=lambda: np.eye(4))
    R: np.ndarray = field(default_factory=lambda: np.eye(2))

    @staticmethod
    def calculate_default_options(
        mpfc_gains_yaml: str, plant:
        MultibodyPlant, plant_context: Context) -> "AlipFootstepLQROptions":
        with io.open(mpfc_gains_yaml, 'r') as file:
            data = load(file, Loader=Loader)

        disc = {
            'FOH': ResetDiscretization.kFOH,
            'ZOH': ResetDiscretization.kZOH,
            'SPLIT': ResetDiscretization.kSPLIT,
        }

        return AlipFootstepLQROptions(
            height=data['height'],
            mass=plant.CalcTotalMass(plant_context),
            stance_width=data['stance_width'],
            single_stance_duration=data['single_stance_duration'],
            double_stance_duration=data['double_stance_duration'],
            #reset_discretization=disc[data['reset_discretization_method']],
            reset_discretization=disc['FOH'], # FOH is better for sim
            Q=np.array(data['Q']).reshape((4, 4)),
            R=np.array(data['R']).reshape(3, 3)[:2, :2],
        )

@dataclass
class fsm_info:
    fsm_state: int
    prev_switch_time_us: int
    next_switch_time_us: int

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

class RLSystem(LeafSystem):
    def __init__(self, alip_params: AlipFootstepLQROptions, model_path: str, env):
        super.__init__()
        ### Multi-Body Plant ###
        self.plant = MultibodyPlant(0.0)
        _ = AddCassieMultibody(
            self.plant,
            None,
            True,
            urdf,
            True,
            False,
            True
        )
        self.plant.Finalize()
        self.plant_context = self.plant.CreateDefaultContext()

        ### RL Model ###
        action_space = spaces.Box(low=np.asarray(np.array([-1., -1., -1.]), dtype="float32"),
                                high=np.asarray(np.array([1., 1., 1.]), dtype="float32"),
                                dtype=np.float32)                         
        observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                        shape=(3*64*64 +6+16+23 +3*64*64,),
                                        dtype=np.float32)
        self.model = CustomActorCriticPolicy(observation_space, action_space, get_schedule_fn(1.))
        self.model.load_state_dict(th.load('test')) # Save policy through -> th.save(model.policy.state_dict(), 'test')
        self.model.eval()
        
        self.lstm_states = None
        self.episode_starts = np.ones((1,), dtype=bool)
        self.empty = np.empty(12310)
        
        ### ALIP parameters ###
        self.params = alip_params
        self.K = np.zeros((2, 4))
        self.S = np.zeros((4, 4))
        self.A, self.B = MassNormalizedAlipStepToStepDynamics(
            self.params.height,
            self.params.single_stance_duration,
            self.params.double_stance_duration,
            self.params.reset_discretization
        )
        self.K, self.S = DiscreteTimeLinearQuadraticRegulator(
            self.A,
            self.B,
            self.params.Q,
            self.params.R
        )

        front_contact_pt = np.array((-0.0457, 0.112, 0))
        rear_contact_pt = np.array((0.088, 0, 0))
        mid_contact_pt = 0.5 * (front_contact_pt + rear_contact_pt)

        left_frame = self.plant.GetBodyByName("toe_left")
        right_frame = self.plant.GetBodyByName("toe_right")

        self.contacts = {
            stance.kLeft: (mid_contact_pt, left_frame), 
            stance.kRight: (mid_contact_pt, right_frame)
        }

        self.input_port_indices = {
                'desired_velocity': self.DeclareVectorInputPort(
                    "vdes", 2
                ).get_index(),
                'x' : self.DeclareVectorInputPort(
                    "x", self.ns
                ).get_index(),
                'vdes': self.DeclareVectorInputPort(
                    'vdes', 2
                ).get_index(),
                'state': self.DeclareVectorInputPort(
                    'x_u_t', 59
                ).get_index(),
                'height_map' : self.DeclareAbstractInputPort(
                "elevation_map", model_value=Value(ElevationMapQueryObject())
                ).get_index(),
            }
        self.output_port_indices = {
            'actions': self.DeclareVectorOutputPort(
                "actions", 3, self.calculate_actions
            ).get_index()
            }

    def calc_lqr_reference(self, vdes: np.ndarray, stance: Stance) -> Tuple[np.ndarray, np.ndarray]:
        s = -1.0 if stance == Stance.kLeft else 1.0
        u0 = np.zeros((2,))
        u0[0] = vdes[0] * (
            self.params.single_stance_duration +
            self.params.double_stance_duration
        )
        u0[1] = s * self.params.stance_width + vdes[1] * (
            self.params.single_stance_duration +
            self.params.double_stance_duration
        )
        u1 = np.copy(u0)
        u1[1] -= 2 * s * self.params.stance_width

        x0 = np.linalg.solve(
            np.eye(4) - self.A @ self.A,
            self.A @ self.B @ u0 + self.B @ u1
        )
        return x0, u0

    def get_stance(fsm: fsm_info) -> Stance:
        stance = Stance.kLeft if fsm.fsm_state == 0 or fsm.fsm_state == 3 else Stance.kRight
        return stance

    def calc_fsm(t: float) -> fsm_info:
        two_stride_period = 2 * (self.params.double_stance_duration + self.params.single_stance_duration)
        one_stride_period = self.params.double_stance_duration + self.params.single_stance_duration

        double periods = t / two_stride_period
        double r = periods - int(periods)
        double phase = r * two_stride_period
        double period_start = (periods - r) * two_stride_period
        fsm = fsm_info(0, 0, 0)

        if (phase < self.params.double_stance_duration) { // Post right double stance
            fsm.fsm_state = post_right_double_support_state_
            fsm.prev_switch_time_us = 1e6 * int(period_start)
            fsm.next_switch_time_us = 1e6 * int(period_start + self.params.double_stance_duration)
        } else if (phase < self.params.single_stance_duration + self.params.double_stance_duration) { // left stance
            fsm.fsm_state = left_stance_state_;
            fsm.prev_switch_time_us = 1e6 * int(period_start + self.params.double_stance_duration)
            fsm.next_switch_time_us = 1e6 * int(period_start + one_stride_period)
        } else if (phase < 2 * self.params.double_stance_duration + self.params.single_stance_duration) { // post left double stance
            fsm.fsm_state = post_left_double_support_state_
            fsm.prev_switch_time_us = 1e6 * int(period_start + one_stride_period)
            fsm.next_switch_time_us = 1e6 * int(period_start + one_stride_period + self.params.double_stance_duration)
        } else { // right stance
            fsm.fsm_state = right_stance_state_
            fsm.prev_switch_time_us = 1e6 * int(period_start + one_stride_period + self.params.double_stance_duration)
            fsm.next_switch_time_us = 1e6 * int(period_start + two_stride_period)
        }
        fsm.timestamp_us = 1e6 * t
        return fsm

    def calculate_actions(self, context: Context, output: BasicVector):
        vdes = self.EvalVectorInput(context, self.input_port_indices['vdes']).get_value()
        states = self.EvalVectorInput(context, self.input_port_indices['state']).get_value()

        t = states[-1]
        fsm = self.calc_fsm(t)
        stance_foot = self.contacts[self.get_stance(fsm)]

        alip = CalcAlipStateInBodyYawFrame(self.plant, self.plant_context, states[:45], "pelvis", stance_foot)
        
        xd, ud = calc_lqr_reference(vdes, self.get_stance(fsm))

        hmap_query = self.EvalAbstractInput(
            context, self.input_port_indices['height_map']
        ).get_value()
        
        adverserial_offset = np.zeros(2,)    
        hmap = hmap_query.calc_height_map_stance_frame(
            np.array([ud[0], ud[1], 0]), np.append(adverserial_offset, 0)
        )
        
        joint_angle = states[7:23] # Only joint angles (reject pelvis)
        joint_angle = [0 if math.isnan(x) else x for x in joint_angle]

        hmap = hmap.reshape(-1)

        obs = np.hstack((hmap, alip, vdes, joint_angle, self.empty)) # 24621
        actions, lstm_states = model.predict(obs, state=self.lstm_states, episode_start=self.episode_starts, deterministic=True)
        self.lstm_states = lstm_states
        self.episode_starts = np.zeros((1,), dtype=bool)
        output.set_value(actions)

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])