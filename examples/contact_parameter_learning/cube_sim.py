from abc import ABC, abstractmethod
from torch import load as torch_load
import numpy as np
import os
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib.pyplot as plt
import pickle

CUBE_DATA_DT = 1.0/148.0
CUBE_DATA_HZ = 148.0
BLOCK_HALF_WIDTH = 0.0524
CUBE_MASS = 0.37

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

    @classmethod
    def make_traj_timestamps(cls, traj):
        return np.arange(0.0, CUBE_DATA_DT * (traj.shape[0]), CUBE_DATA_DT)

    @classmethod
    def reexpress_state_local_to_global_omega(cls, state):
        new_state = state.ravel()
        q = new_state[CUBE_DATA_QUATERNION_SLICE]
        rot = R.from_quat([q[1], q[2], q[3], q[0]])
        new_state[CUBE_DATA_OMEGA_SLICE] = rot.apply(new_state[CUBE_DATA_OMEGA_SLICE])
        return new_state

    @classmethod
    def reexpress_state_global_to_local_omega(cls, state):
        new_state = state.ravel()
        q = new_state[CUBE_DATA_QUATERNION_SLICE]
        rot = R.from_quat([q[1], q[2], q[3], q[0]])
        new_state[CUBE_DATA_OMEGA_SLICE] = rot.apply(new_state[CUBE_DATA_OMEGA_SLICE], inverse=True)
        return new_state

class LossWeights():

    def __init__(self, debug=False,
                 pos=np.ones((3,)),
                 vel=np.ones((3,)),
                 omega=np.ones((3,)), 
                 quat=1):
        self.pos = np.diag(pos)
        self.vel = np.diag(vel)
        self.omega = np.diag(omega)
        self.quat = quat
        self.debug=debug

    def save(self, fp):
        self.pos = self.pos.tolist()
        self.vel = self.vel.tolist()
        self.omega = self.omega.tolist()
        pickle.dump(self, fp)

    def CalcPositionsLoss(self, traj1, traj2):
        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.pos).ravel()) / diff.shape[0]
    
    def CalcVelocitiesLoss(self, traj1, traj2):
        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.vel).ravel()) / diff.shape[0]
    
    def CalcOmegaLoss(self, traj1, traj2):

        diff = traj1 - traj2
        return np.dot(diff.ravel(), (diff @ self.omega).ravel()) / diff.shape[0]

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
        if (self.debug): print(f'l_pos: {l_pos}, l_vel: {l_vel}, l_omega: {l_omega}, l_quat: {l_quat}')
        return l_pos + l_vel + l_omega + l_quat

    @classmethod
    def calc_rotational_distance(cls, quat1, quat2):
        q1 = quat1.ravel()
        q2 = quat2.ravel()
        R1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
        R2 = R.from_quat([q2[1], q2[2], q2[3], q2[0]])
        Rel = R1 * R2.inv()
        return np.linalg.norm(Rel.as_rotvec())

    @classmethod
    def load_weights(cls, fp):
        loss_weights = pickle.load(fp)
        loss_weights.pos = np.array(loss_weights.pos)
        loss_weights.vel = np.array(loss_weights.vel)
        loss_weights.omega = np.array(loss_weights.omega)
        return loss_weights

def make_cube_toss_filename(data_folder, toss_id):
    return os.path.join(data_folder, str(toss_id) + '.pt')

def make_simulated_toss_filename(data_folder, toss_id):
    return os.path.join(data_folder, str(toss_id) + '.npy')


def load_cube_toss(filename):
    data_all = torch_load(filename)
    state_traj = data_all[0,:,0:13].numpy()
    state_traj[:, CUBE_DATA_POSITION_SLICE] *= BLOCK_HALF_WIDTH
    state_traj[:, CUBE_DATA_VELOCITY_SLICE] *= BLOCK_HALF_WIDTH

    state_traj = get_window_around_contact_event(state_traj)

    return state_traj

def save_sim_traj(filename, traj):
    with open(filename, 'w+') as fp:
        np.save(filename, traj)

def load_sim_traj(filename):
    state_traj = np.load(filename)
    return get_window_around_contact_event(state_traj)

def get_index_before_contact(data_traj):
    start_idx = np.argwhere(data_traj[:,CUBE_DATA_POSITION_SLICE][:,2] <= BLOCK_HALF_WIDTH * np.sqrt(3.0))[0].tolist()[0] - 1
    start_idx = np.max([0, start_idx])

def fit_initial_condition(data_traj):
    idx_pre = get_index_before_contact(data_traj)
    if idx_pre == 0:
        return get_window_around_contact_event(data_traj)
        
    q1 = data_traj[0,CUBE_DATA_QUATERNION_SLICE].ravel()
    q2 = data_traj[idx_pre,CUBE_DATA_QUATERNION_SLICE].ravel()

    R1 = R.from_quat([q1[1], q1[2], q1[3], q1[0]])
    R2 = R.from_quat([q2[1], q2[2], q2[3], q2[0]])

    R_rel = R2.inv() * R1

    omega = R_rel.as_rotvec() / (CUBE_DATA_DT * idx_pre)



def get_window_around_contact_event(data_traj):
    # return whole trajectory for now
    start_idx = get_index_before_contact(data_traj)
    return data_traj[start_idx:]



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

    sim.init_sim(contact_params)

    simulated_trajectory = sim.get_sim_traj_initial_state(
        state_traj[0], state_traj.shape[0], CUBE_DATA_DT)
    loss = weights.CalculateLoss(simulated_trajectory, state_traj)
    if(debug):
        print(f'toss id: {toss_id}\t\tloss: {loss}')
    return loss # normalize loss by duration of the trajectory
