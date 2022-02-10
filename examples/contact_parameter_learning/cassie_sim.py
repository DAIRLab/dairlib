
import numpy as np
import cassie_sim_traj

class CassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 1e-5
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.end_time = 0.05
        self.traj = cassie_sim_traj()
        self.valid_ground_truth_trajs = np.arange(0, 24)

    def make(self):
        pass

    def reset(self, ground_truth_traj):
        self.traj = cassie_sim_traj()
        return

    def advance_to(self):
        return

    def sim_step(self, action=None):
        return

    def return_traj(self):
        return self.traj



