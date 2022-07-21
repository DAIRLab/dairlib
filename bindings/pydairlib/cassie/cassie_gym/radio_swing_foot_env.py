import numpy as np
import gym
import yaml
from scipy.spatial.transform import Rotation as R

from dairlib import lcmt_swing_foot_spline_params
from pydairlib.cassie.cassie_utils import *
from pydairlib.cassie.controllers import AlipWalkingControllerFactory

from pydrake.multibody.plant import MultibodyPlant


from pydairlib.cassie.cassie_gym.drake_cassie_gym import DrakeCassieGym
from pydairlib.cassie.cassie_gym.cassie_env_state import CassieEnvState, CASSIE_NRADIO, CASSIE_RADIO_TWISTS
from pydairlib.cassie.cassie_gym.reward_osudrl import RewardOSUDRL
import pydairlib.cassie.cassie_gym.swing_foot_env as swing_foot_env
from pydairlib.cassie.cassie_gym.high_level_reward import HighLevelReward

N_KNOT = 9
SWING_FOOT_ACTION_DIM = N_KNOT * 3 + 6

class RadioSwingFootEnv(DrakeCassieGym):

    def __init__(self, reward_func, visualize=False, max_step_magnitude=0.0, goal=0.8):
        super().__init__(reward_func, visualize, max_step_magnitude)
        self.default_action = swing_foot_env.get_default_params()
        highs = [1] * CASSIE_RADIO_TWISTS + [0.2] * SWING_FOOT_ACTION_DIM
        lows = [-1] * CASSIE_RADIO_TWISTS + [-0.2] * SWING_FOOT_ACTION_DIM
        self.action_space = gym.spaces.Box(
            high=np.array(highs), low=np.array(lows), shape=(CASSIE_RADIO_TWISTS + SWING_FOOT_ACTION_DIM,), dtype=np.float32)
        self.add_controller()
        self.make(self.controller)
        self.swing_ft_error_port = self.controller.get_swing_error_output_port()
        self.ss_states = [0, 1]
        self.action_log = []
        self.state_log = []
        self.goal = goal


    def add_controller(self):
        osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
        osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
        urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

        self.controller_plant = MultibodyPlant(8e-5)
        AddCassieMultibody(self.controller_plant, None, True, urdf, False, False)
        self.controller_plant.Finalize()
        self.controller = AlipWalkingControllerFactory(
            self.controller_plant, True, True, osc_gains, osqp_settings, N_KNOT)


    def save_action_log(self):
        np.save("swing_foot_commands.npy", np.array(self.action_log))
        np.save("swing_foot_ppo_states.npy", np.array(self.state_log))

    def step(self, action=None):
        if not self.initialized:
            print("Call make() before calling step() or advance()")
        if action is None:
            # swing_action = np.zeros((len(self.default_action),))
            swing_action = self.default_action
            radio_twist = np.zeros(CASSIE_RADIO_TWISTS)

        else:
            swing_action = action[CASSIE_RADIO_TWISTS:] + self.default_action
            # swing_action = self.default_action 
            radio_twist = action[0:CASSIE_RADIO_TWISTS]

        self.action_log.append(swing_action)
        self.state_log.append(self.cassie_state.x)
        radio = np.zeros((CASSIE_NRADIO,))
        radio[[2, 3, 5]] = radio_twist

        # assuming this gets called while the robot is in double support
        cur_fsm_state = self.controller.get_fsm_output_port().Eval(self.controller_context)[0]
        if cur_fsm_state in self.ss_states:
            done_with_ds = True
        else:
            done_with_ds = False
        cumulative_reward = 0
        total_pitch = 0
        total_roll = 0
        start_time = self.drake_simulator.get_context().get_time()

        # Essentially want to do till the end of current double stance and then 
        # the end of the next single stance in one environment step
        while not done_with_ds or cur_fsm_state in self.ss_states:
            x = self.plant.GetPositionsAndVelocities(
                self.plant.GetMyMutableContextFromRoot(
                    self.drake_simulator.get_context()))

            # normalized by sim_dt
            quat = x[0:4]  # TODO: not hardcode this. 
            euler = R.from_quat(quat).as_euler("xyz")
            pitch_mag = np.abs(euler[1])
            roll_mag = np.abs(euler[0])
            total_pitch += pitch_mag
            total_roll += roll_mag - np.pi # seems like pi is the nominal roll?

            next_timestep = self.drake_simulator.get_context().get_time() + self.sim_dt
            self.cassie_sim.get_radio_input_port().FixValue(
                self.cassie_sim_context, radio)
            self.controller.get_radio_input_port().FixValue(
                self.controller_context, radio)
            self.controller.get_swing_foot_params_input_port().FixValue(
                self.controller_context, swing_foot_env.pack_action_message(swing_action))
            # Do sim step
            try:
                self.drake_simulator.AdvanceTo(next_timestep)
            except RuntimeError:
                print("drake simulator ran into error!") 
                self.terminated = True
                return np.array(self.cassie_state.x), cumulative_reward, True, {}

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

            # reached the goal
            self.terminated = self.check_termination() or self.cassie_state.get_positions()[0] >= self.goal 
            # print(f"state positions: {self.cassie_state.get_positions()[0:3]}")
            # print(f"foot positions: {self.cassie_state.get_positions()[0:3]}")
            self.prev_cassie_state = self.cassie_state
            if self.terminated:
                cumulative_reward += 0 
                break
            else:
                cumulative_reward += reward
            self.traj.append(self.cassie_state, reward)
        total_time = self.current_time - start_time
        info = {"total_pitch_error":total_pitch/total_time, "total_roll_error":total_roll/total_time}
        return np.array(self.cassie_state.x), cumulative_reward, bool(self.terminated), info


def make_radio_swing_ft_env(rank, seed=0, visualize=False):
    def _init():
        env = RadioSwingFootEnv(reward_func=HighLevelReward(0.5), visualize=visualize)
        env.seed(seed + rank)
        return env
    return _init
