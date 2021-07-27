from abc import ABC, abstractmethod
import numpy as np

class CubeSim(ABC):

    @abstractmethod
    def sim_step(self):
        pass

    @abstractmethod
    def init_sim(self, params):
        pass

    @abstractmethod
    def get_sim_traj_initial_state(self, initial_state, window):
        pass



def get_random_window_around_contact_event(state_traj):
    pass


def calculate_cubesim_loss(contact_params, state_traj, cube_sim):

    cube_sim.init_sim(contact_params)

    window, state_traj_in_window = get_random_window_around_contact_event(state_traj)
    simulated_trajectory = cube_sim.get_sim_traj_initial_state(state_traj_in_window[:,0], window)

