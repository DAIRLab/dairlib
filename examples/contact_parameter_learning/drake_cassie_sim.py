import numpy as np
import sys
import time
from scipy import interpolate
import cassie_loss_utils
import subprocess
import pickle
import matplotlib.pyplot as plt
import plot_styler

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml


class DrakeCassieSim():

  def __init__(self, drake_sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
    self.sim_data_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/"
    self.params_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/drake_cassie_params/"
    self.date_prefix = 'drake_' + time.strftime("%Y_%m_%d_%H_%M")
    # self.start_time = 30.595
    self.start_time = 30.6477
    self.sim_time = 0.05
    self.end_time = self.start_time + self.sim_time
    self.drake_sim_dt = drake_sim_dt
    self.realtime_rate = 1.0
    self.terrain_height = 0.00
    self.ps = plot_styler.PlotStyler()
    self.base_z_idx = 6
    self.base_vel_idx = slice(26,29)

    with open("x_datatypes", "rb") as fp:
      self.x_datatypes = pickle.load(fp)

    self.x_trajs = {}
    self.t_xs = {}


    self.log_nums_all = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_real = np.hstack((np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_sim = np.hstack((np.arange(0, 3), np.arange(8, 18), np.arange(20, 34)))
    self.log_nums_all = ['%0.2d' % i for i in self.log_nums_all]
    self.log_nums_real = ['%0.2d' % i for i in self.log_nums_real]
    for log_num in self.log_nums_all:
      self.x_trajs[log_num] = np.load(self.folder_path + 'x_' + log_num + '.npy')
      self.t_xs[log_num] = np.load(self.folder_path + 't_x_' + log_num + '.npy')

    # self.z_offsets = np.load(self.params_folder + 'all_z_offset_50000.npy')
    # self.vel_offsets = np.load(self.params_folder + 'all_vel_offset_50000.npy')

    with open(self.params_folder + 'optimized_z_offsets.pkl', 'rb') as file:
      self.z_offsets = pickle.load(file)
    with open(self.params_folder + 'optimized_vel_offsets.pkl', 'rb') as file:
      self.vel_offsets = pickle.load(file)

    self.default_drake_contact_params = {
      "mu_static": 0.8,
      "mu_ratio": 1.0,
      # "pen_allow": 1e-5,
      "stiffness": 4e4,
      "dissipation": 0.5,
      "stiction_tol": 1e-3,
      # "vel_offset": np.zeros(len(self.log_nums_real) * 3),
      # "z_offset": np.zeros(len(self.log_nums_real)),
      "vel_offset": np.zeros(3),
      "z_offset": np.zeros(1),
    }
    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)
    self.iter_num = 0



  def write_initial_state(self, x_init):
    gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/data/"
    # print(x_init)
    def float_representer(dumper, value):
      text = '{0:.5f}'.format(value)
      return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)

    yaml.add_representer(float, float_representer)

    with open(gains_path + 'initial_state.yaml', 'r') as f:
      data = yaml.load(f, Loader=Loader)
      data['x_init'] = x_init.tolist()

      f = open(gains_path + 'initial_state.yaml', 'w')
      f.write(yaml.dump(data, default_flow_style=None))
      f.close()

  def save_params(self, params, sim_id):
    with open(self.params_folder + self.date_prefix + sim_id + '.pkl', 'wb') as f:
      pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

  def load_params(self, sim_id):
    with open(self.params_folder + sim_id + '.pkl', 'rb') as f:
      return pickle.load(f)

  def run(self, params, log_num):
    # params

    # penetration_allowance = self.default_drake_contact_params['pen_allow']
    # mu_static = self.default_drake_contact_params['mu_static']
    # mu_kinetic = self.default_drake_contact_params['mu_ratio'] * self.default_drake_contact_params['mu_static']
    # stiction_tol = self.default_drake_contact_params['stiction_tol']
    # penetration_allowance = params['pen_allow']
    stiffness = params['stiffness']
    dissipation = params['dissipation']
    mu_static = params['mu_static']
    mu_kinetic = params['mu_ratio'] * params['mu_static']
    stiction_tol = params['stiction_tol']
    log_idx = self.log_nums_real.index(log_num)
    log_idx = self.log_nums_real.index('15')
    # print('log_idx' + str(log_idx))

    # x_traj = np.load(self.folder_path + 'x_' + log_num + '.npy')
    # t = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj = self.x_trajs[log_num]
    t = self.t_xs[log_num]

    ### Overrides here

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(self.start_time)

    # z_offset = self.z_offsets[log_num]
    # vel_offset = self.vel_offsets[log_num]
    z_offset = params['z_offset']
    vel_offset = params['vel_offset']

    x_init[self.base_z_idx] += z_offset
    x_init[self.base_vel_idx] += vel_offset
    self.write_initial_state(x_init)
    simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_playback',
                     '--folder_path=%s' % self.folder_path,
                     '--end_time=%.3f' % self.end_time,
                     '--terrain_height=%.4f' % self.terrain_height,
                     '--start_time=%.3f' % self.start_time,
                     '--log_num=' + log_num,
                     # '--penetration_allowance=%.7f' % penetration_allowance,
                     '--stiffness=%.1f' % stiffness,
                     '--dissipation_rate=%.3f' % dissipation,
                     '--stiction_tol=%.7f' % stiction_tol,
                     '--mu_static=%.5f' % mu_static,
                     '--mu_kinetic=%.5f' % mu_kinetic,
                     '--target_realtime_rate=%.2f' % self.realtime_rate,
                     # '--delta_x_init=%.5f' % delta_x_init,
                     ]
    # print((' ').join(simulator_cmd))
    simulator_process = subprocess.Popen(simulator_cmd)
    simulator_process.wait()
    if simulator_process.returncode == 1:
      return '-1'
    x_traj = np.genfromtxt('x_traj.csv', delimiter=',', dtype='f16')
    lambda_traj = np.genfromtxt('lambda_traj.csv', delimiter=',', dtype='f16')
    t_x = np.genfromtxt('t_x.csv')
    # t_x = np.append(t_x, self.start_time + self.sim_time)
    # sim_id = log_num + str(np.abs(hash(frozenset(params))))
    np.save(self.sim_data_folder + 'x_' + log_num, x_traj)
    np.save(self.sim_data_folder + 't_x_' + log_num, t_x)
    lambda_traj = lambda_traj.reshape((4, 3, lambda_traj.shape[1]))
    lambda_traj = np.transpose(lambda_traj, (0, 2, 1))
    # self.ps.plot(t_x, lambda_traj[0, 2, :].T)
    # self.ps.plot(t_x, lambda_traj[1, 2, :].T)
    # self.ps.plot(t_x, lambda_traj[2, 2, :].T)
    # self.ps.plot(t_x, lambda_traj[3, 2, :].T)
    # self.ps.show_fig()
    # import pdb; pdb.set_trace()
    np.save(self.sim_data_folder + 'lambda_' + log_num, lambda_traj)

    self.iter_num += 1
    print("iter: " + str(self.iter_num))

    return log_num

  def load_sim_trial(self, log_num):
    x_traj = np.load(self.sim_data_folder + 'x_' + log_num + '.npy')
    t_x = np.load(self.sim_data_folder + 't_x_' + log_num + '.npy')

    return x_traj.transpose(), t_x

  def get_window_around_contact_event(self, x_traj, t_x):
    # return whole trajectory for now

    # start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[0][0]
    # end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[0][0]
    start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[1][0]
    end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[1][0]
    window = slice(start_idx, end_idx)
    return window, x_traj[window]

  def compute_loss(self, log_num, sim_id, params, plot=False):
    # x_traj_log = np.load(self.folder_path + 'x_' + log_num + '.npy')
    # t_x_log = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj_log = self.x_trajs[log_num]
    t_x_log = self.t_xs[log_num]

    x_traj, t_x = self.load_sim_trial(sim_id)
    window, x_traj_in_window = self.get_window_around_contact_event(x_traj_log, t_x_log)
    min_time_length = min(x_traj.shape[0], x_traj_in_window.shape[0])
    # import pdb; pdb.set_trace()
    joints = np.arange(23, 45)
    if plot:
      for joint in joints:
        plt.figure("joint vel: " + self.x_datatypes[joint])

        self.ps.plot(t_x[:min_time_length], x_traj[:min_time_length, joint])
        self.ps.plot(t_x_log[window][:min_time_length], x_traj_in_window[:min_time_length, joint])
        self.ps.add_legend([self.x_datatypes[joint] + ': sim', self.x_datatypes[joint] + ': real'])
      # self.ps.plot(t_x[:min_time_length], x_traj[:min_time_length, 31:35], color='b')
      # self.ps.plot(t_x_log[window][:min_time_length], x_traj_in_window[:min_time_length, 31:35], color='r')
      # self.ps.plot(t_x[:min_time_length], x_traj[:min_time_length, 4:7], color='b')
      # self.ps.plot(t_x[:min_time_length], x_traj_in_window[:min_time_length, 4:7], color='r')
      #   plt.figure('loss')
      #   self.ps.plot(t_x[:min_time_length], x_traj[:min_time_length, 23:45] - x_traj_log[:min_time_length, 23:45], color=self.ps.grey)
      plt.show()

    traj_loss = self.loss_func.CalculateLossTraj(x_traj[:min_time_length], x_traj_in_window[:min_time_length])
    # regularization_loss = self.loss_func.CalculateLossParams(params)
    # return traj_loss + regularization_loss
    return traj_loss


if __name__ == '__main__':
  log_num = sys.argv[1]
  loss_func = cassie_loss_utils.CassieLoss()
  sim = DrakeCassieSim()
  sim.run(sim.default_drake_contact_params, log_num)
