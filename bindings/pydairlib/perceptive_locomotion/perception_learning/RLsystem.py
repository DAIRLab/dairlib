import math
import numpy as np
import torch as th
import torch.nn as nn
import argparse
import os
from os import path
import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple
from dataclasses import dataclass, field

import io
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.common.value import Value
from pydrake.geometry import (Rgba, Meshcat)
from pydrake.systems.all import (
    State,
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    LeafSystem,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    ZeroOrderHold,
    DiscreteTimeLinearQuadraticRegulator,
    BasicVector,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
    ExternallyAppliedSpatialForce_,
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

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)
from pydairlib.perceptive_locomotion.systems.elevation_map_converter import ElevationMapQueryObject
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

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
    prev_switch_time: int
    next_switch_time: int

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
    def __init__(self, hidden_layer_size = 64):
        super(CustomNetwork, self).__init__()

        self.latent_dim_pi = hidden_layer_size
        self.latent_dim_vf = hidden_layer_size
        self.vector_state_actor = 6+16
        self.vector_state_critic = 6+23
        self.height_map_size = 64
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

        self.actor_combined_lstm = nn.LSTM(
            input_size=self.vector_state_actor + 64,
            hidden_size=hidden_layer_size,
            num_layers=2,
            batch_first=False
        )
        self.critic_combined_lstm = nn.LSTM(
            input_size=self.vector_state_critic + 64,
            hidden_size=hidden_layer_size,
            num_layers=2,
            batch_first=False
        )

    def forward(self, observations: th.Tensor):
        actor_combined_features = self.multihead_actor(observations)
        critic_combined_features = self.multihead_critic(observations)
        actor_combined_lstm_output, _ = self.actor_combined_lstm(actor_combined_features)
        critic_combined_lstm_output, _ = self.critic_combined_lstm(critic_combined_features)
        return actor_combined_lstm_output, critic_combined_lstm_output

    def multihead_actor(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.height_map_size * self.height_map_size].reshape(batch_size, 3, self.height_map_size, self.height_map_size)
        state = observations[:, 3 * self.height_map_size * self.height_map_size: 3 * self.height_map_size * self.height_map_size + 6 + 16]
        actor_cnn_output = self.actor_cnn(image_obs)
        actor_combined_features = th.cat((actor_cnn_output, state), dim=1).unsqueeze(1)
        return actor_combined_features

    def multihead_critic(self, observations: th.Tensor):
        batch_size = observations.size(0)
        image_obs_gt = observations[:, -3 * self.height_map_size * self.height_map_size:].reshape(batch_size, 3, self.height_map_size, self.height_map_size)
        state = th.cat((observations[:, 3 * self.height_map_size * self.height_map_size: 3 * self.height_map_size * self.height_map_size + 6], \
                        observations[:, 3 * self.height_map_size * self.height_map_size + 6 + 16:3 * self.height_map_size * self.height_map_size + 6 + 16 + 23]), dim=1)
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
        self.hidden_layer_size = lstm_hidden_size

        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            lstm_hidden_size = lstm_hidden_size,
            n_lstm_layers = 2,
            *args,
            **kwargs,
        )

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork(self.hidden_layer_size)


class RLSystem(LeafSystem):
    def __init__(self, model_path: str, hidden_size: int): # alip_params: AlipFootstepLQROptions,
        super().__init__()
        ### Multi-Body Plant ###
        self.plant = MultibodyPlant(0.0)
        _ = AddCassieMultibody(
            self.plant,
            None,
            True,
            "examples/Cassie/urdf/cassie_v2.urdf",
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

        self.model = CustomActorCriticPolicy(
            observation_space,
            action_space,
            get_schedule_fn(1.),
            lstm_hidden_size=hidden_size
        )
        self.model.load_state_dict(th.load(model_path, map_location=th.device('cpu'))) # Save policy through -> th.save(model.policy.state_dict(), 'test')
        self.model.eval()
        
        self.lstm_states = None
        self.episode_starts = np.ones((1,), dtype=bool)
        self.empty = np.empty(12311)
        
        ### ALIP parameters ###
        params_folder = "bindings/pydairlib/perceptive_locomotion/params"
        alip_params = AlipFootstepLQROptions.calculate_default_options(
                    path.join(params_folder, 'alip_s2s_mpfc_gains.yaml'),
                    self.plant,
                    self.plant_context,
                )

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
        
        ### FSM ###
        self.left_stance_state_ = 0
        self.right_stance_state_ = 1
        self.post_left_double_support_state_ = 3
        self.post_right_double_support_state_ = 4

        front_contact_pt = np.array((-0.0457, 0.112, 0))
        rear_contact_pt = np.array((0.088, 0, 0))
        mid_contact_pt = 0.5 * (front_contact_pt + rear_contact_pt)

        left_frame = self.plant.GetBodyByName("toe_left").body_frame()
        right_frame = self.plant.GetBodyByName("toe_right").body_frame()

        self.contacts = {
            Stance.kLeft: (mid_contact_pt.reshape(3, 1), left_frame), 
            Stance.kRight: (mid_contact_pt.reshape(3, 1), right_frame)
        }

        self.input_port_indices = {
                'desired_velocity': self.DeclareVectorInputPort(
                    "vdes", 2
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
                "actions", 6, self.calculate_actions
            ).get_index(),
            'ud': self.DeclareVectorOutputPort( # For plotting
                "ud", 2, self.calc_ud
            ).get_index(),
            'fsm': self.DeclareVectorOutputPort(
                "fsm", 1, self.calc_fsm_port,
                prerequisites_of_calc={
                    self.input_port_ticket(self.input_port_indices['state'])
                }
            ).get_index()
            }

    def calc_fsm_port(self, context: Context, fsm: BasicVector):
        states = self.EvalVectorInput(
            context, self.input_port_indices['state']
        ).get_value()
        fsm.set_value(np.array([self.calc_fsm(states[-1]).fsm_state]))

    def calc_ud(self, context: Context, ud: BasicVector):
        vdes = self.EvalVectorInput(context, self.input_port_indices['desired_velocity']).get_value()
        states = self.EvalVectorInput(context, self.input_port_indices['state']).get_value()

        t = states[-1]
        fsm = self.calc_fsm(t)
        x, u = self.calc_lqr_reference(vdes, self.get_stance(fsm))

        ud.set_value(u)

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

    def get_stance(self, fsm: fsm_info) -> Stance:
        stance = Stance.kLeft if fsm.fsm_state == 0 or fsm.fsm_state == 4 else Stance.kRight # 3
        return stance

    def calc_fsm(self, t: float) -> fsm_info:
        two_stride_period = 2 * (self.params.double_stance_duration + self.params.single_stance_duration)
        one_stride_period = self.params.double_stance_duration + self.params.single_stance_duration

        periods = t / two_stride_period
        r = periods - int(periods)
        phase = r * two_stride_period
        period_start = (periods - r) * two_stride_period
        fsm = fsm_info(0, 0, 0)

        if (phase < self.params.double_stance_duration): # Post right double stance
            fsm.fsm_state = self.post_right_double_support_state_
            fsm.prev_switch_time = period_start
            fsm.next_switch_time = period_start + self.params.double_stance_duration
        elif (phase < self.params.single_stance_duration + self.params.double_stance_duration): # left stance
            fsm.fsm_state = self.left_stance_state_
            fsm.prev_switch_time = period_start + self.params.double_stance_duration
            fsm.next_switch_time = period_start + one_stride_period
        elif (phase < 2 * self.params.double_stance_duration + self.params.single_stance_duration): # post left double stance
            fsm.fsm_state = self.post_left_double_support_state_
            fsm.prev_switch_time = period_start + one_stride_period
            fsm.next_switch_time = period_start + one_stride_period + self.params.double_stance_duration
        else: # right stance
            fsm.fsm_state = self.right_stance_state_
            fsm.prev_switch_time = period_start + one_stride_period + self.params.double_stance_duration
            fsm.next_switch_time = period_start + two_stride_period
        
        fsm.timestamp_us = 1e6 * t
        return fsm

    def calculate_actions(self, context: Context, output: BasicVector):
        vdes = self.EvalVectorInput(context, self.input_port_indices['desired_velocity']).get_value()
        states = self.EvalVectorInput(context, self.input_port_indices['state']).get_value()
        t = states[-1]

        fsm = self.calc_fsm(t)

        stance_foot = self.contacts[self.get_stance(fsm)]

        time_until_switch = np.maximum(0, fsm.next_switch_time - t)

        alip = CalcAlipStateInBodyYawFrame(self.plant, self.plant_context, states[:45], "pelvis", stance_foot)
        
        current_alip_state = alip.copy()
        current_alip_state[2:] /= self.params.mass
        alip = CalcMassNormalizedAd(
            self.params.height, time_until_switch
        ) @ current_alip_state

        xd, ud = self.calc_lqr_reference(vdes, self.get_stance(fsm))

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
        actions, lstm_states = self.model.predict(obs, state=self.lstm_states, episode_start=self.episode_starts, deterministic=True)
        self.lstm_states = lstm_states
        self.episode_starts = np.zeros((1,), dtype=bool)

        out = np.concatenate([actions, [fsm.fsm_state, fsm.prev_switch_time, fsm.next_switch_time]])

        output.set_value(out)

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])


def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions) \
        -> Tuple[CassieFootstepControllerEnvironment, RLSystem, Diagram]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    sim_env.set_name("CassieFootstepControllerEnvironment")
    builder.AddSystem(sim_env)
    footstep_zoh = ZeroOrderHold(0.05, 6)

    rl_system = RLSystem(model_path='test')
    
    builder.AddSystem(rl_system)
    builder.Connect(
        sim_env.get_output_port_by_name("height_map"),
        rl_system.get_input_port_by_name("height_map")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("state"),
        rl_system.get_input_port_by_name("state")
    )

    builder.AddSystem(footstep_zoh)
    builder.Connect(
        rl_system.get_output_port_by_name('actions'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )
    diagram = builder.Build()

    # DrawAndSaveDiagramGraph(diagram, '../Cassie_stand_alone')
    return sim_env, rl_system, diagram


def check_termination(sim_env, diagram_context, time) -> bool:
    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    sim_context = sim_env.GetMyMutableContextFromRoot(diagram_context)
    track_error = sim_env.get_output_port_by_name('swing_ft_tracking_error').Eval(sim_context)

    front_contact_pt = np.array((-0.0457, 0.112, 0))
    rear_contact_pt = np.array((0.088, 0, 0))

    toe_left_rotation = plant.GetBodyByName("toe_left").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
    left_toe_direction = toe_left_rotation @ (front_contact_pt - rear_contact_pt)
    left_angle = abs(np.arctan2(left_toe_direction[2], np.linalg.norm(left_toe_direction[:2])))
    
    toe_right_rotation = plant.GetBodyByName("toe_right").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
    right_toe_direction = toe_right_rotation @ (front_contact_pt - rear_contact_pt)
    right_angle = abs(np.arctan2(right_toe_direction[2], np.linalg.norm(right_toe_direction[:2])))

    left_toe_pos = plant.CalcPointsPositions(
        plant_context, plant.GetBodyByName("toe_left").body_frame(),
        np.array([0.02115, 0.056, 0.]), plant.world_frame()
    )
    right_toe_pos = plant.CalcPointsPositions(
        plant_context, plant.GetBodyByName("toe_right").body_frame(),
        np.array([0.02115, 0.056, 0.]), plant.world_frame()
    )
    com = plant.CalcCenterOfMassPositionInWorld(plant_context)

    z1 = com[2] - left_toe_pos[2]
    z2 = com[2] - right_toe_pos[2]

    return z1 < 0.2 or z2 < 0.2

def run(sim_env, rl_system, diagram, plot=False):
    
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic_new.npz'
        )
    )
    
    datapoint = ic_generator.random()
    
    v_des_theta = 0.35
    v_des_norm = 0.8
    v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    v_norm = np.random.uniform(0.0, v_des_norm)
    datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()
    print(datapoint['desired_velocity'])

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    
    # timing aliases
    t_ss = rl_system.params.single_stance_duration
    t_ds = rl_system.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 0.01  # small number that prevent impact

    # grab the sim and controller contexts for convenience
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    rl_context = rl_system.GetMyMutableContextFromRoot(context)
    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + t_ds + t_eps + datapoint['phase']

    print(f't_init: {t_init}')
    context.SetTime(t_init)

    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    rl_system.get_input_port_by_name("desired_velocity").FixValue(
        context=rl_context,
        value=datapoint['desired_velocity']
    )

    simulator.reset_context(context)
   
    terminate = False
    time = 0.0

    for i in range(1, 333): # 10 seconds
        if check_termination(sim_env, context, time):
            terminate = True
            break

        simulator.AdvanceTo(t_init + 0.03*i)
        ud = rl_system.get_output_port_by_name('ud').Eval(rl_context)
        # Depth map
        dmap_query = rl_system.EvalAbstractInput(
            rl_context, rl_system.input_port_indices['height_map']
        ).get_value()

        dmap = dmap_query.calc_height_map_stance_frame(
            np.array([ud[0], ud[1], 0])
        )

        # Plot depth image
        if plot:
            grid_world = dmap_query.calc_height_map_world_frame(
                np.array([ud[0], ud[1], 0])
            )
            dmap_query.plot_surface(
                "dmap", grid_world[0], grid_world[1],
                grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        time = context.get_time()-t_init
    
    return time

def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    
    # True if using depth sensor
    sim_params.simulate_perception = True

    # Visualization is False by default
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()

    terrain = 'params/flat.yaml'
    
    os.path.join(perception_learning_base_folder, terrain)
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)

    sim_env, rl_system, diagram = build_diagram(sim_params)
    input("Starting...")
    time = run(sim_env, rl_system, diagram, plot=True)
    print(f"Terminated in {time} seconds in {terrain}.")

if __name__ == '__main__':
    main()