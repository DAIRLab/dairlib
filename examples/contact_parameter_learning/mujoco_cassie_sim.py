import numpy as np
import sys
import time
from scipy import interpolate
import cassie_loss_utils
import pickle
from mujoco_sim import MujocoCassieSim
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial
import xml.etree.ElementTree as ET
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper


class LearningMujocoCassieSim():

  def __init__(self, drake_sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.sim = MujocoCassieSim(publish_state=False, realtime_rate=2.0)
    self.folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
    self.sim_data_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/"
    self.params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/mujoco_cassie_params/"
    self.drake_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/drake_cassie_params/"
    self.date_prefix = 'mujoco_' + time.strftime("%Y_%m_%d_%H_%M")
    self.start_times = {'08': 30.61,
                        '09': 30.61,
                        '10': 30.61,
                        '11': 30.62,
                        '12': 30.64,
                        '13': 30.64,
                        '14': 30.65,
                        '15': 30.64,
                        '16': 30.64,
                        '17': 30.64,
                        '20': 30.64,
                        '21': 30.64,
                        '22': 30.64,
                        '23': 30.64,
                        '24': 30.64,
                        '25': 30.64,
                        '26': 30.64,
                        '27': 30.64,
                        '28': 30.64,
                        '29': 30.64,
                        '30': 30.64,
                        '31': 30.63,
                        '32': 30.63,
                        '33': 30.63,
                        }
    self.sim_time = 0.05
    # self.end_time = self.start_time + self.sim_time
    self.default_mujoco_contact_params = {
      "timeconst": 0.005,
      "dampratio": 1,
      "ground_mu_tangent": 1.0,
      "mu_torsion": 0.0001,
      "mu_rolling": 0.00001}
    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)
    self.iter_num = 0
    self.base_z_idx = 6
    self.base_vel_idx = slice(26,29)

    self.x_trajs = {}
    self.t_xs = {}

    with open(self.drake_params_folder + 'optimized_z_offsets.pkl', 'rb') as file:
      self.z_offsets = pickle.load(file)
    with open(self.drake_params_folder + 'optimized_vel_offsets.pkl', 'rb') as file:
      self.vel_offsets = pickle.load(file)

    # self.z_offsets = np.load(self.drake_params_folder + 'all_z_offset_50000.npy')
    # self.vel_offsets = np.load(self.drake_params_folder + 'all_vel_offset_50000.npy')

    self.log_nums_all = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_real = np.hstack((np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_sim = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_all = ['%0.2d' % i for i in self.log_nums_all]
    self.log_nums_real = ['%0.2d' % i for i in self.log_nums_real]
    for log_num in self.log_nums_all:
      self.x_trajs[log_num] = np.load(self.folder_path + 'x_' + log_num + '.npy')
      self.t_xs[log_num] = np.load(self.folder_path + 't_x_' + log_num + '.npy')

    self.tree = ET.parse(self.sim.default_model_file)

  def save_params(self, params, sim_id):
    with open(self.params_folder + self.date_prefix + sim_id + '.pkl', 'wb') as f:
      pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

  def load_params(self, sim_id):
    with open(self.params_folder + sim_id + '.pkl', 'rb') as f:
      return pickle.load(f)

  def run(self, params, log_num):

    timeconst = params['timeconst']
    dampratio = params['dampratio']
    mu_ground = params['ground_mu_tangent']
    mu_torsion = params['mu_torsion']
    mu_rolling = params['mu_rolling']
    self.tree.getroot().find('default').find('geom').set('solref', '%.5f %.5f' % (timeconst, dampratio))
    self.tree.getroot().find('default').find('geom').set('friction', '%.5f %.5f %.5f' % (mu_ground, mu_torsion, mu_rolling))
    self.tree.write(self.sim.default_model_directory + 'cassie_new_params.xml')
    self.sim.reinit_env(self.sim.default_model_directory + 'cassie_new_params.xml')
    # params
    log_idx = self.log_nums_real.index(log_num)
    x_traj = self.x_trajs[log_num]
    t = self.t_xs[log_num]

    start_time = self.start_times[log_num]
    end_time = start_time + self.sim_time

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(start_time)

    # z_offset = self.z_offsets[log_idx]
    # vel_offset = self.vel_offsets[3*log_idx:3*(log_idx + 1)]
    z_offset = self.z_offsets[log_num]
    vel_offset = self.vel_offsets[log_num]
    x_init[self.base_z_idx] += z_offset
    x_init[self.base_vel_idx] += vel_offset

    lcm_traj = lcm_trajectory.LcmTrajectory(self.folder_path + 'u_traj_' + log_num)
    u_traj = lcm_traj.GetTrajectory("controller_inputs")
    u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)
    x_traj, u_traj, t_x = self.sim.run_sim_playback(x_init, start_time, end_time, u_init_traj)

    sim_id = log_num + str(np.abs(hash(frozenset(params))))
    x_traj = np.array(x_traj)
    u_traj = np.array(u_traj)
    t_x = np.array(t_x)
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
    min_time_length = min(x_traj.shape[0], x_traj_in_window.shape[0])

    loss = self.loss_func.CalculateLossTraj(x_traj[:min_time_length], x_traj_in_window[:min_time_length])
    return loss
