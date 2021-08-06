from abc import ABC, abstractmethod
from torch import load as torch_load
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

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

    def make_traj_timestamps(self, traj):
        return np.arange(0.0, CUBE_DATA_DT * (traj.shape[0]), CUBE_DATA_DT)

    def make_comparison_plot(self, params, data_folder, toss_id):
        data_traj = load_cube_toss(make_cube_toss_filename(data_folder, toss_id))
        _, data_traj = get_window_around_contact_event(data_traj)
        self.init_sim(params)
        sim_traj = self.get_sim_traj_initial_state(data_traj[0], data_traj.shape[0], CUBE_DATA_DT)

        tvec = self.make_traj_timestamps(data_traj)
        position_error = np.linalg.norm(data_traj[:,CUBE_DATA_POSITION_SLICE] - sim_traj[:,CUBE_DATA_POSITION_SLICE], axis=1)
        position_error /= BLOCK_HALF_WIDTH
        vel_error = np.linalg.norm(data_traj[:,CUBE_DATA_VELOCITY_SLICE] - sim_traj[:,CUBE_DATA_VELOCITY_SLICE], axis=1)
        vel_error /= BLOCK_HALF_WIDTH
        omega_error = np.linalg.norm(data_traj[:,CUBE_DATA_OMEGA_SLICE] - sim_traj[:,CUBE_DATA_OMEGA_SLICE], axis=1)

        plt.plot(tvec, position_error)
        plt.show()
        


class LossWeights():

    def __init__(self, 
                 pos=np.ones((3,)),
                 vel=np.ones((3,)),
                 omega=np.ones((3,)), 
                 quat=1):
        self.pos = np.diag(pos)
        self.vel = np.diag(vel)
        self.omega = np.diag(omega)
        self.quat = quat
    
    def CalcPositionsLoss(self, traj1, traj2):
        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.pos).ravel()) / diff.shape[0]
    
    def CalcVelocitiesLoss(self, traj1, traj2):
        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.pos).ravel()) / diff.shape[0]
    
    def CalcOmegaLoss(self, traj1, traj2):
        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.pos).ravel()) / diff.shape[0]

    def CalcQuatLoss(self, traj1, traj2):
        loss = 0
        for i in range(traj1.shape[0]):
            quat_diff = self.calc_rotational_distance(traj1[i], traj2[i])
            loss += quat_diff ** 2
        loss *= self.quat / traj1.shape[0]
        return loss 

    def CalculateLoss(self, traj1, traj2):
        l_pos = self.CalcPositionsLoss(traj1[:,CUBE_DATA_POSITION_SLICE], traj2[:,CUBE_DATA_POSITION_SLICE])
        l_vel = self.CalcVelocitiesLoss(traj1[:,CUBE_DATA_VELOCITY_SLICE], traj2[:,CUBE_DATA_VELOCITY_SLICE])
        l_omega = self.CalcOmegaLoss(traj1[:,CUBE_DATA_OMEGA_SLICE], traj2[:,CUBE_DATA_OMEGA_SLICE])
        l_quat = self.CalcQuatLoss(traj1[:,CUBE_DATA_QUATERNION_SLICE], traj2[:,CUBE_DATA_QUATERNION_SLICE])
        return l_pos + l_vel + l_omega + l_quat

    def calc_rotational_distance(self, quat1, quat2):
        q1 = quat1.ravel()
        q2 = quat2.ravel()
        R1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
        R2 = R.from_quat([q2[1], q2[2], q2[3], q2[0]])
        Rel = R1 * R2.inv()
        return np.linalg.norm(Rel.as_rotvec()) ** 2

def make_cube_toss_filename(data_folder, toss_id):
    return os.path.join(data_folder, str(toss_id) + '.pt')

def make_simulated_toss_filename(data_folder, toss_id):
    return os.path.join(data_folder, str(toss_id) + '.npy')


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

def save_sim_traj(filename, traj):
    with open(filename, 'w+') as fp:
        np.save(filename, traj)

def load_sim_traj(filename):
    return np.load(filename)

def make_simulated_traj_data_for_training(contact_params, toss_ids, cube_data_folder, save_folder, sim):
    for id in toss_ids:
        state_traj = load_cube_toss(make_cube_toss_filename(cube_data_folder, id))
        sim.init_sim(contact_params)
        sim_traj = sim.get_sim_traj_initial_state(state_traj[0], state_traj.shape[0], CUBE_DATA_DT)
        save_sim_traj(make_simulated_toss_filename(save_folder, id), sim_traj)
        print(id)


''' Interface method to calculate the loss for a given set of parameters and a trajectory'''
def calculate_cubesim_loss(contact_params, toss_id, data_folder, sim, weights=LossWeights(), debug=False, simulated=False):
    if simulated:
        state_traj = load_sim_traj(make_simulated_toss_filename(data_folder, toss_id))
    else:
        state_traj = load_cube_toss(make_cube_toss_filename(data_folder, toss_id))
    window, state_traj_in_window = get_window_around_contact_event(state_traj)

    sim.init_sim(contact_params)

    simulated_trajectory = sim.get_sim_traj_initial_state(
        state_traj_in_window[0], state_traj_in_window.shape[0], CUBE_DATA_DT)
    loss = weights.CalculateLoss(simulated_trajectory, state_traj_in_window)
    if(debug):
        print(f'toss id: {toss_id}\t\tloss: {loss}')
    return loss # normalize loss by duration of the trajectory
