from abc import ABC, abstractmethod
from torch import load as torch_load
import numpy as np
import os

CUBE_DATA_DT = 1.0/148.0
CUBE_DATA_HZ = 148.0
BLOCK_HALF_WIDTH = 0.0524

CUBE_DATA_POSITION_SLICE = slice(0,3,1)
CUBE_DATA_QUATERNION_SLICE = slice(3,7,1)
CUBE_DATA_VELOCITY_SLICE = slice(7,10,1)
CUBE_DATA_OMEGA_SLICE = slice(10,13,1)

class CubeSim(ABC):

    @abstractmethod
    def sim_step(self, dt):
        pass

    @abstractmethod
    def init_sim(self, params):
        pass

    @abstractmethod
    def set_initial_condition(self, initial_state):
        pass

    def get_sim_traj_initial_state(self, initial_state, steps, dt):
        x_sim = np.zeros((steps, initial_state.size))
        self.set_initial_condition(initial_state.ravel())
        for i in range(steps):
            x_sim[i,:] = self.sim_step(dt)
        return x_sim

def make_cube_toss_filename(data_folder, toss_id):
    return os.path.join(data_folder, str(toss_id) + '.pt')


def get_window_around_contact_event(state_traj):
    # return whole trajectory for now
    window = slice(0, state_traj.shape[0],1)
    return window, state_traj

def load_cube_toss(filename):
    data_all = torch_load(filename)
    state_traj = data_all[0,:,0:13].numpy()
    state_traj[:, CUBE_DATA_POSITION_SLICE] *= BLOCK_HALF_WIDTH
    state_traj[:, CUBE_DATA_VELOCITY_SLICE] *= BLOCK_HALF_WIDTH
    return state_traj

''' Interface method with the optimizer'''
def calculate_cubesim_loss(contact_params, toss_id, data_folder, sim, debug=False):
    
    state_traj = load_cube_toss(make_cube_toss_filename(data_folder, toss_id))
    window, state_traj_in_window = get_window_around_contact_event(state_traj)

    sim.init_sim(contact_params)

    simulated_trajectory = sim.get_sim_traj_initial_state(
        state_traj_in_window[0], state_traj_in_window.shape[0], CUBE_DATA_DT)

    diff = simulated_trajectory.ravel() - state_traj_in_window.ravel()
    loss = np.dot(diff, diff) / state_traj_in_window.shape[1]
    if(debug):
        print(loss)
    return loss # normalize loss by duration of the trajectory
