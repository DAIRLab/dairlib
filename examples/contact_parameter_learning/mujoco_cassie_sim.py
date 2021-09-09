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
import matplotlib.pyplot as plt
import plot_styler

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper


class LearningMujocoCassieSim():

  def __init__(self, drake_sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.sim = MujocoCassieSim(publish_state=False, realtime_rate=5.0)
    self.folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
    self.sim_data_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/mujoco_cassie_sim_data/"
    self.params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/mujoco_cassie_params/"
    self.drake_params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/drake_cassie_params/"
    self.date_prefix = 'mujoco_' + time.strftime("%Y_%m_%d_%H")
    self.ps = plot_styler.PlotStyler()
    self.start_times = {'08': 30.61,
                        '09': 30.61,
                        '10': 30.60,
                        '11': 30.62,
                        '12': 30.66,
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
                        '31': 30.635,
                        '32': 30.635,
                        '33': 30.635,
                        }
    self.sim_time = 0.05
    # self.end_time = self.start_time + self.sim_time
    self.default_mujoco_contact_params = \
      {"stiffness" : 2000,
       "damping" : 36.02,
       "mu_tangent" : 0.18}
    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)
    self.iter_num = 0
    self.base_z_idx = 6
    self.base_vel_idx = slice(26, 29)

    with open("x_datatypes", "rb") as fp:
      self.x_datatypes = pickle.load(fp)

    self.x_trajs = {}
    self.lambda_trajs = {}
    self.t_x = {}

    self.log_nums_all = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    # self.log_nums_real = np.hstack((np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_real = np.hstack((np.arange(8, 12), np.arange(14, 18), np.arange(20, 34)))
    self.log_nums_sim = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_all = ['%0.2d' % i for i in self.log_nums_all]
    self.log_nums_real = ['%0.2d' % i for i in self.log_nums_real]
    self.log_nums_w_offset = ['21', '23', '24', '25', '26', '27', '28', '30', '31', '32', '33']
    for log_num in self.log_nums_real:
      self.x_trajs[log_num] = np.load(self.folder_path + 'x_' + log_num + '.npy')
      self.lambda_trajs[log_num] = np.load(self.folder_path + 'lambda_' + log_num + '.npy')
      self.t_x[log_num] = np.load(self.folder_path + 't_x_' + log_num + '.npy')

    with open(self.drake_params_folder + 'optimized_z_offsets.pkl', 'rb') as file:
      self.z_offsets = pickle.load(file)
    with open(self.drake_params_folder + 'optimized_vel_offsets.pkl', 'rb') as file:
      self.vel_offsets = pickle.load(file)

    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)
    self.iter_num = 0
    self.tree = ET.parse(self.sim.default_model_file)

  def save_params(self, params, sim_id):
    with open(self.params_folder + self.date_prefix + sim_id + '.pkl', 'wb') as f:
      pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

  def load_params(self, sim_id):
    with open(self.params_folder + sim_id + '.pkl', 'rb') as f:
      return pickle.load(f)

  def run(self, params, log_num):

    stiffness = params['stiffness']
    damping = params['damping']
    mu_tangent = params['mu_tangent']
    # mu_torsion = params['mu_torsion']
    # mu_rolling = params['mu_rolling']
    self.tree.getroot().find('default').find('geom').set('solref', '%.5f %.5f' % (-stiffness, -damping))
    self.tree.getroot().find('default').find('geom').set('friction',
                                                         '%.5f %.5f %.5f' % (mu_tangent, .001, .001))
    self.tree.write(self.sim.default_model_directory + 'cassie_new_params.xml')
    self.sim.reinit_env(self.sim.default_model_directory + 'cassie_new_params.xml')
    # params
    log_idx = self.log_nums_real.index(log_num)
    x_traj = self.x_trajs[log_num]
    t = self.t_x[log_num]

    start_time = self.start_times[log_num]
    end_time = start_time + self.sim_time

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(start_time)

    # z_offset = self.z_offsets[log_idx]
    # vel_offset = self.vel_offsets[3*log_idx:3*(log_idx + 1)]
    # z_offset = self.z_offsets[log_num]
    # vel_offset = self.vel_offsets[log_num]

    z_offset = np.zeros(1)
    vel_offset = np.zeros(3)
    if log_num in self.log_nums_w_offset:
      z_offset = np.array([-0.015])

    x_init[self.base_z_idx] += z_offset
    x_init[self.base_vel_idx] += vel_offset

    lcm_traj = lcm_trajectory.LcmTrajectory(self.folder_path + 'u_traj_' + log_num)
    u_traj = lcm_traj.GetTrajectory("controller_inputs")
    u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)
    x_traj, u_traj, t_x = self.sim.run_sim_playback(x_init, start_time, end_time, u_init_traj)

    # sim_id = log_num + str(np.abs(hash(frozenset(params))))
    x_traj = np.array(x_traj)
    u_traj = np.array(u_traj)
    t_x = np.array(t_x)
    np.save(self.sim_data_folder + 'x_' + log_num, x_traj)
    np.save(self.sim_data_folder + 't_x_' + log_num, t_x)

    self.iter_num += 1
    print("sim_run: " + str(self.iter_num))

    return log_num

  def load_sim_trial(self, log_num):
    x_traj = np.load(self.sim_data_folder + 'x_' + log_num + '.npy')
    t_x = np.load(self.sim_data_folder + 't_x_' + log_num + '.npy')

    return x_traj, t_x

  def get_window_around_contact_event(self, t_x_sim, t_x, x_traj):
    # return whole trajectory for now
    # start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[0][0]
    # end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[0][0]
    start_idx = np.argwhere(np.isclose(t_x, t_x_sim[0], atol=5e-4))[1][0]
    end_idx = np.argwhere(np.isclose(t_x, t_x_sim[-1], atol=5e-4))[1][0]
    window = slice(start_idx, end_idx)
    return t_x[window], x_traj[window]

  def compute_loss(self, log_num, sim_id, params, plot=False):
    # x_traj_hardware = np.load(self.folder_path + 'x_' + log_num + '.npy')
    # t_x_hardware = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj_hardware = self.x_trajs[log_num]
    t_x_hardware = self.t_x[log_num]

    x_traj_sim, t_x_sim = self.load_sim_trial(sim_id)
    t_x_hardware, x_traj_hardware = self.get_window_around_contact_event(t_x_sim,
                                                                         t_x_hardware,
                                                                         x_traj_hardware)
    min_time_length = min(x_traj_sim.shape[0], x_traj_hardware.shape[0])
    # import pdb; pdb.set_trace()
    # joints = np.arange(23, 45)
    joints = np.arange(35, 37)
    if plot:
      for joint in joints:
        plt.figure("joint vel: " + self.x_datatypes[joint])

        self.ps.plot(t_x_sim[:min_time_length], x_traj_sim[:min_time_length, joint])
        self.ps.plot(t_x_hardware[:min_time_length], x_traj_hardware[:min_time_length, joint])
        self.ps.add_legend([self.x_datatypes[joint] + ': sim', self.x_datatypes[joint] + ': real'])
        # self.ps.add_legend([self.x_datatypes[joint] + ': real'])
      # self.ps.plot(t_x_sim[:min_time_length], x_traj_sim[:min_time_length, 31:35], color='b')
      # self.ps.plot(t_x_hardware[window][:min_time_length], x_traj_hardware[:min_time_length, 31:35], color='r')
      # self.ps.plot(t_x_sim[:min_time_length], x_traj_sim[:min_time_length, 4:7], color='b')
      # self.ps.plot(t_x_sim[:min_time_length], x_traj_hardware[:min_time_length, 4:7], color='r')
      #   plt.figure('loss')
      #   self.ps.plot(t_x_sim[:min_time_length], x_traj_sim[:min_time_length, 23:45] - x_traj_hardware[:min_time_length, 23:45], color=self.ps.grey)
      plt.show()
    traj_loss = self.loss_func.CalculateLossTraj(x_traj_sim[:min_time_length], x_traj_hardware[:min_time_length])
    # regularization_loss = self.loss_func.CalculateLossParams(params)
    # return traj_loss + regularization_loss
    return traj_loss
