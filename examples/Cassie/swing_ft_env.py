import gym
import os
from gym import spaces
import lcm
import queue
import threading
from drake import lcmt_scope
import subprocess as sp
import time 

from dairlib import lcmt_swing_foot_spline_params, lcmt_robot_output


class LCMInterface:
    def __init__(self, channels, message_types, address=None):
        assert len(channels) == len(message_types)
        self.channels = channels 
        self.message_types = {channels[i]:message_types[i] for i in range(len(channels))}
        self.lc = lcm.LCM()
        self.channels = channels
        self.queues = {channel:queue.LifoQueue() for channel in self.channels}
        self.stop_listener = threading.Event()
        for c in channels:
            self.lc.subscribe(c, self.handler)

    def listener_loop(self):
        while True:
            self.lc.handle() 
            if self.stop_listener.is_set():
                break
            time.sleep(0.001)
 
    def start_listening(self):
        self.stop_listener.clear()
        self.listener_thread = threading.Thread(target=self.listener_loop) 
        self.listener_thread.start()
        return

    def stop_reset_listening(self):
        self.stop_listener.set()
        self.queues = {channel:queue.LifoQueue() for channel in self.channels}

    def handler(self, channel, data):
        self.queues[channel].put(self.message_types[channel].decode(data))

    def get_latest(self, channel):
        return self.queues[channel].get(block=True)

    def publish(self, channel, message):
        self.lc.publish(channel, message.encode())


class CassieSwingFootEnv(gym.Env):
    """Class that provides and interface to run_osc_walking_controller_alip
    simulation in Drake, with an action space defined by the lcmt_swing_foot_spline_params
    structure.
    """
    def __init__(self, lcm_address=None, sim=True, viz=True):
        self.reward_channel = "CASSIE_SWING_FOOT_REWARD"
        self.fsm_channel = "FINITE_STATE_MACHINE"
        self.using_sim = sim
        self.viz = viz
        if sim:
            self.state_channel = "CASSIE_STATE_SIMULATION"
        else:
            self.state_channel = "CASSIE_STATE_DISPATCHER"

        self.lcm_interface = LCMInterface([self.reward_channel, self.fsm_channel, self.state_channel],
                                          [lcmt_scope, lcmt_scope, lcmt_robot_output])

        # TODO: define the observation, action spaces if we want to use something like StableBaselines

        # Infrastructure definitions
        self.ctrlr, self.sim = None, None
        self.bin_dir = "./bazel-bin/examples/Cassie/"
        self.controller_p = "run_osc_walking_controller_alip"
        self.sim_p = "multibody_sim"
        self.ctrlr_options = ["--learn_swing_foot_path=1"]
        self.sim_options = []
        self.viz_options = ["--floating_base=true", "--channel="+self.state_channel]

        ### Spawning Director & visualizer Process ###
        if self.viz:
            # why is this not opening a window?
            self.drake_director = sp.Popen(["./bazel-bin/director/drake-director", "--use_builtin_scripts=frame,image", "--script=examples/Cassie/director_scripts/show_time.py"])
            self.visualizer = sp.Popen(["bazel-bin/examples/Cassie/visualizer"] + self.viz_options)
            # self.visualizer = None
            print("launched director process")
            time.sleep(5)
        else:
            self.drake_director = None
            self.visualizer = None
            time.sleep(1)


    def kill_procs(self):
        if self.sim is not None:
            self.sim.terminate()
            self.sim = None
        if self.ctrlr is not None:
            self.ctrlr.terminate()
            self.ctrlr = None
        
    def kill_director(self):
        if self.drake_director is not None:
            self.drake_director.terminate()
        if self.visualizer is not None:
            self.visualizer.terminate()

    def step(self, action):
        """ Sends swing foot spline action to Drake, and receives back
        the new state and reward. Runs SYNCHRNOUSLY with sim so is 
        runtime-sensitive.

        param action: 1 + 3*action[0] + 3 + 3 array
        """
        assert (len(action) == 1+3*action[0] + 6), "action length must match # of knot points!"
        # fill out action message lcm type
        action_msg = lcmt_swing_foot_spline_params()
        action_msg.n_knot = action[0]
        action_msg.knot_xyz = [[0]*3] * int(action[0])
        for n in range(action[0]):
            action_msg.knot_xyz[n] = action[1+3*n:1+3*(n+1)]
        action_msg.swing_foot_vel_initial = action[-6:-3]
        action_msg.swing_foot_vel_final = action[-3:]

        cur_fsm_state = self.lcm_interface.get_latest(self.fsm_channel).value[0]
        self.lcm_interface.publish(self.action_channel, action_msg)
        while self.lcm_interface.get_latest(self.fsm_channel).value[0] == cur_fsm_state:
            # TODO: change this polling to match the publish rate
            time.sleep(0.001)
        reward = self.lcm_interface.get_latest(self.state_channel).value[0]
        state = self.select_states(self.lcm_interface.get_latest(self.state_channel))
        # TODO: slice out the appropriate state variables
        # check failure on the state (has the robot fallen over?)
        return state, reward, False


    def reset(self):
        """ Restarts sim & controller, returns state
        """
        self.kill_procs()
        self.lcm_interface.stop_reset_listening()
        self.ctrlr = sp.Popen([os.path.join(self.bin_dir, self.controller_p)] + self.ctrlr_options)
        if self.using_sim:
            self.sim = sp.Popen([os.path.join(self.bin_dir, self.sim_p)] + self.sim_options)
            time.sleep(1.5)  # need to let the initial conditions solve, initialize
        self.lcm_interface.start_listening()
        state = self.select_states(self.lcm_interface.get_latest(self.reward_channel))
        return state


    def select_states(self, full_cassie_state):
        """ Selects relevant states from the full robot_out
        LCM message
        """
        return full_cassie_state 


    def check_failure(self, state):
        return False


def main():
    try:
        env = CassieSwingFootEnv() 
        # s = env.reset()
        time.sleep(10)
    except KeyboardInterrupt:
        env.kill_procs()
        env.kill_director()

if __name__ == "__main__":
    main()
