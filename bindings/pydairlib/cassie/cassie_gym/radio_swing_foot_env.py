import numpy as np
import gym
import yaml

from dairlib import lcmt_swing_foot_spline_params
from pydairlib.cassie.cassie_utils import *
from pydairlib.cassie.controllers import AlipWalkingControllerFactory

from pydrake.multibody.plant import MultibodyPlant


from pydairlib.cassie.cassie_gym.drake_cassie_gym import DrakeCassieGym
from pydairlib.cassie.cassie_gym.cassie_env_state import CassieEnvState, CASSIE_NRADIO, CASSIE_RADIO_TWISTS
from pydairlib.cassie.cassie_gym.reward_osudrl import RewardOSUDRL
import pydairlib.cassie.cassie_gym.swing_foot_env as swing_foot_env
# from reward_osudrl import RewardOSUDRL

N_KNOT = 9
SWING_FOOT_ACTION_DIM = N_KNOT * 3 + 6

class RadioSwingFootEnv(DrakeCassieGym):

    def __init__(self, reward_func, visualize=False):
        super().__init__(reward_func, visualize)
        self.default_action = swing_foot_env.get_default_params()
        self.action_space = gym.spaces.Box(
            high=1, low=-1, shape=(CASSIE_RADIO_TWISTS + SWING_FOOT_ACTION_DIM,), dtype=np.float32)
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

    def step(self, action=None):
        if not self.initialized:
            print("Call make() before calling step() or advance()")
        if action is None:
            swing_action = np.zeros((len(self.default_action),))
            radio_twist = np.zeros(CASSIE_RADIO_TWISTS)

        else:
            swing_action = action[CASSIE_RADIO_TWISTS:]
            radio_twist = action[0:CASSIE_RADIO_TWISTS]
            radio = np.zeros((CASSIE_NRADIO,))
            radio[[0, 1, 3]] = radio_twist

        # assuming this gets called while the robot is in double support
        cur_fsm_state = self.controller.get_fsm_output_port().Eval(self.controller_context)[0]
        if cur_fsm_state in self.ss_states:
            done_with_ds = True
        else:
            done_with_ds = False
        cumulative_reward = 0

        # Essentially want to do till the end of current double stance and then 
        # the end of the next single stance in one environment step
        while not done_with_ds or cur_fsm_state in self.ss_states:
            next_timestep = self.drake_simulator.get_context().get_time() + self.sim_dt
            self.cassie_sim.get_radio_input_port().FixValue(
                self.cassie_sim_context, radio)
            self.controller.get_radio_input_port().FixValue(
                self.controller_context, radio)
            self.controller.get_swing_foot_params_input_port().FixValue(
                self.controller_context, swing_foot_env.pack_action_message(swing_action))
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
            self.cassie_state = CassieEnvState(self.current_time, x, u, radio, swing_action)
            self.traj.append(self.cassie_state)
            swing_ft_error = self.swing_ft_error_port.Eval(self.controller_context).ravel()
            reward = self.reward_func.compute_reward(
                self.sim_dt, self.cassie_state, self.prev_cassie_state,
                swing_foot_error=swing_ft_error)
            self.terminated = self.check_termination()
            self.prev_cassie_state = self.cassie_state
            cumulative_reward += reward
            self.traj.append(self.cassie_state, reward)
        return np.array(self.cassie_state.x), cumulative_reward, bool(self.terminated), {}


def make_radio_swing_ft_env(rank, seed=0):
    def _init():
        env = RadioSwingFootEnv(reward_func=HighLevelReward(0.5), visualize=False)
        env.seed(seed + rank)
        return env
    return _init
