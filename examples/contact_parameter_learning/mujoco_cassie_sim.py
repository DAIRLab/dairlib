import numpy as np
import sys
import time
from scipy import interpolate
import cassie_loss_utils
import subprocess
import pickle
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml


class LearningMujocoCassieSim():

  def __init__(self, drake_sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
    self.sim_data_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/"
    self.params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/mujoco_cassie_params/"
    self.start_time = 30.595
    self.sim_time = 0.1
    self.end_time = self.start_time + self.sim_time
    self.default_mujoco_contact_params = {
      "mu_static": 0.8,
      "mu_ratio": 1.0,
      "pen_allow": 1e-5,
      "stiction_tol": 1e-3}
    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)
    self.iter_num = 0

  def save_params(self, params, sim_id):
    with open(self.params_folder + sim_id + '.pkl', 'wb') as f:
      pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

  def load_params(self, sim_id):
    with open(self.params_folder + sim_id + '.pkl', 'rb') as f:
      return pickle.load(f)

  def run(self, params, log_num):
    # params
    penetration_allowance = params['pen_allow']
    # print(penetration_allowance)
    mu_static = params['mu_static']
    mu_kinetic = params['mu_ratio'] * params['mu_static']
    stiction_tol = params['stiction_tol']
    # delta_x_init = params['delta_x_init']
    terrain_height = 0.00
    x_traj = np.load(self.folder_path + 'x_' + log_num + '.npy')
    t = np.load(self.folder_path + 't_x_' + log_num + '.npy')

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(self.start_time)

    lcm_traj = lcm_trajectory.LcmTrajectory(self.folder_path + 'u_traj_' + log_num)
    u_traj = lcm_traj.GetTrajectory("controller_inputs")
    u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)
    x_traj, u_traj, t_x = sim.run_sim_playback(x_init, self.start_time, self.end_time, u_init_traj)

    # x_traj = np.genfromtxt('x_traj.csv', delimiter=',', dtype='f16')
    # t_x = np.genfromtxt('t_x.csv')
    # t_x = np.append(t_x, self.start_time + self.sim_time)
    sim_id = log_num + str(np.abs(hash(frozenset(params))))
    # self.save_params(params, sim_id)
    np.save(self.sim_data_folder + 'x_traj' + sim_id, x_traj)
    np.save(self.sim_data_folder + 't_x' + sim_id, t_x)

    self.iter_num += 1
    print("iter: " + str(self.iter_num))

    return sim_id

  def load_sim_trial(self, sim_id):
    x_traj = np.load(self.sim_data_folder + 'x_traj' + sim_id + '.npy')
    t_x = np.load(self.sim_data_folder + 't_x' + sim_id + '.npy')

    return x_traj, t_x

  def get_window_around_contact_event(self, x_traj, t_x):
    # return whole trajectory for now

    start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[0][0]
    end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[0][0]
    window = slice(start_idx, end_idx)
    return window, x_traj

  def compute_loss(self, log_num, sim_id):
    x_traj_log = np.load(self.folder_path + 'x_' + log_num + '.npy')
    t_x_log = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj, t_x = self.load_sim_trial(sim_id)
    window, x_traj_in_window = self.get_window_around_contact_event(x_traj_log, t_x_log)

    loss = self.loss_func.CalculateLoss(x_traj.transpose(), x_traj_in_window[window])
    return loss


if __name__ == '__main__':
  log_num = sys.argv[1]
  loss_func = cassie_loss_utils.CassieLoss()
  sim = DrakeCassieSim()
  sim.run(sim.default_drake_contact_params, log_num)
