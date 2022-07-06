import numpy as np
import gym
import yaml

from dairlib import lcmt_swing_foot_spline_params
from pydairlib.cassie.cassie_utils import *
from pydairlib.cassie.controllers import AlipWalkingControllerFactory

from pydrake.multibody.plant import MultibodyPlant


from pydairlib.cassie.cassie_gym.drake_cassie_gym import DrakeCassieGym
from pydairlib.cassie.cassie_gym.cassie_env_state import CassieEnvState, CASSIE_NRADIO
from pydairlib.cassie.cassie_gym.reward_osudrl import RewardOSUDRL
# from reward_osudrl import RewardOSUDRL

N_KNOT = 9
SWING_FOOT_ACTION_DIM = N_KNOT * 3 + 6


def get_default_params(
        gains_file="examples/Cassie/osc/osc_walking_gains_alip.yaml"):
    with open(gains_file, 'r') as f:
        gains = yaml.safe_load(f)

    knots = []
    for i in range(N_KNOT):
        t = i/(N_KNOT - 1)
        x = [0.5 * (np.sin(np.pi * (t - 0.5)) + 1),
             0.5 * (np.sin(np.pi * (t - 0.5)) + 1),
             (gains["mid_foot_height"]) * np.cos(np.pi * (t - 0.5))*t]
        knots += x
    vel_initial = [0, 0, 0]
    vel_final = [0, 0, gains["final_foot_velocity_z"]]
    return knots + vel_initial + vel_final


def get_poor_default_params():
    knots = []
    for i in range(N_KNOT):
        t = i / (N_KNOT - 1)
        x = [t, t, 0.75*t*(1-t)]
        knots += x
    vel_initial = [0, 0, 0]
    vel_final = [0, 0, 0]
    return knots + vel_initial + vel_final


def pack_action_message(action):
    return np.concatenate(([N_KNOT], action)).ravel()


class SwingFootEnv(DrakeCassieGym):

    def __init__(self, reward_func, visualize=False):
        super().__init__(reward_func, visualize)
        self.default_action = get_default_params()
        self.action_space = gym.spaces.Box(
            high=1, low=-1, shape=(SWING_FOOT_ACTION_DIM,), dtype=np.float32)
        self.add_controller()
        self.make(self.controller)
        self.ss_states = [0, 1]
        self.swing_ft_error_port = self.controller.get_swing_error_output_port()

    def add_controller(self):
        osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
        osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
        urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

        self.controller_plant = MultibodyPlant(8e-5)
        AddCassieMultibody(self.controller_plant, None, True, urdf, False, False)
        self.controller_plant.Finalize()
        self.controller = AlipWalkingControllerFactory(
            self.controller_plant, True, True, osc_gains, osqp_settings, N_KNOT)

    def step(self, action=None, vel_des=(1.0, 0.0)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")
        if action is None:
            action = np.zeros((len(self.default_action),))

        # assuming this gets called while the robot is in double support
        cur_fsm_state = self.controller.get_fsm_output_port().Eval(self.controller_context)[0]
        if cur_fsm_state in self.ss_states:
            done_with_ds = True
        else:
            done_with_ds = False
        cumulative_reward = 0

        # Essentially want to do till the end of current double stance and then 
        # the end of the next single stance in one environment step
        radio = np.zeros((CASSIE_NRADIO,))
        radio[2] = vel_des[0]  # currently, y vel is unused
        while not done_with_ds or cur_fsm_state in self.ss_states:
            next_timestep = self.drake_simulator.get_context().get_time() + self.sim_dt
            self.cassie_sim.get_radio_input_port().FixValue(
                self.cassie_sim_context, radio)
            self.controller.get_radio_input_port().FixValue(
                self.controller_context, radio)
            self.controller.get_swing_foot_params_input_port().FixValue(
                self.controller_context, pack_action_message(action))
            # Do sim step
            self.drake_simulator.AdvanceTo(next_timestep)
            self.current_time = self.drake_simulator.get_context().get_time()
            new_fsm_state = self.controller.get_fsm_output_port().Eval(self.controller_context)[0]
            if new_fsm_state != cur_fsm_state:
                done_with_ds = True
            cur_fsm_state = new_fsm_state

            x = self.plant.GetPositionsAndVelocities(
                self.plant.GetMyMutableContextFromRoot(
                    self.drake_simulator.get_context()))
            u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
            self.cassie_state = CassieEnvState(self.current_time, x, u, radio, action)
            self.traj.append(self.cassie_state)
            swing_ft_error = self.swing_ft_error_port.Eval(self.controller_context).ravel()
            print(f"swing foot env step swing_ft_error: {swing_ft_error}")
            reward = self.reward_func.compute_reward(
                self.sim_dt, self.cassie_state, self.prev_cassie_state,
                swing_foot_error=swing_ft_error)
            self.terminated = self.check_termination()
            if self.terminated:
                cumulative_reward += reward
                break
            self.prev_cassie_state = self.cassie_state
            self.traj.append(self.cassie_state, reward)

        return np.array(self.cassie_state.x), cumulative_reward, bool(self.terminated), {}


def make_swing_ft_env_fn(rank, seed=0):
    def _init():
        env = SwingFootEnv(reward_func=RewardOSUDRL(), visualize=False)
        env.seed(rank + seed)
        return env 
    return _init
