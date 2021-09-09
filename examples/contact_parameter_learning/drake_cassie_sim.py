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
    self.date_prefix = 'drake_' + time.strftime("%Y_%m_%d_%H")
    # self.start_time = 30.595
    # self.start_time = 30.5
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
    self.drake_sim_dt = drake_sim_dt
    self.realtime_rate = 1.0
    self.terrain_height = 0.00
    self.ps = plot_styler.PlotStyler()
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

    # self.z_offsets = np.load(self.params_folder + 'all_z_offset_50000.npy')
    # self.vel_offsets = np.load(self.params_folder + 'all_vel_offset_50000.npy')

    with open(self.params_folder + 'optimized_z_offsets.pkl', 'rb') as file:
      self.z_offsets = pickle.load(file)
    with open(self.params_folder + 'optimized_vel_offsets.pkl', 'rb') as file:
      self.vel_offsets = pickle.load(file)

    self.default_drake_contact_params = {
      # "mu_static": 0.8,
      # "mu_ratio": 1.0,
      # "pen_allow": 1e-5,
      "mu" : 0.8,
      "stiffness": 4e4,
      "dissipation": 0.5,
      # "stiction_tol": 1e-3,
      # "vel_offset": np.zeros(len(self.log_nums_real) * 3),
      # "z_offset": np.zeros(len(self.log_nums_real)),
      # "vel_offset": np.zeros(3),
      # "z_offset": np.zeros(1),
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
    mu_static = params['mu']
    mu_kinetic = params['mu']
    stiction_tol = 1e-3
    log_idx = self.log_nums_real.index(log_num)
    log_idx = self.log_nums_real.index('15')
    # print('log_idx' + str(log_idx))

    # x_traj = np.load(self.folder_path + 'x_' + log_num + '.npy')
    # t = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj = self.x_trajs[log_num]
    t = self.t_x[log_num]

    start_time = self.start_times[log_num]
    end_time = start_time + self.sim_time

    ### Overrides here

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(start_time)


    # z_offset = self.z_offsets[log_num]
    # vel_offset = self.vel_offsets[log_num]
    # z_offset = params['z_offset']
    # vel_offset = params['vel_offset']

    z_offset = np.zeros(1)
    vel_offset = np.zeros(3)
    if log_num in self.log_nums_w_offset:
      z_offset = np.array([-0.015])
    # print(z_offset)
    # print(vel_offset)

    x_init[self.base_z_idx] += z_offset
    x_init[self.base_vel_idx] += vel_offset
    self.write_initial_state(x_init)

    simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_playback',
                     '--folder_path=%s' % self.folder_path,
                     '--end_time=%.3f' % end_time,
                     '--terrain_height=%.4f' % self.terrain_height,
                     '--start_time=%.3f' % start_time,
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
    t_x = np.genfromtxt('t_x.csv')
    # t_x = np.append(t_x, self.start_time + self.sim_time)
    # sim_id = log_num + str(np.abs(hash(frozenset(params))))
    np.save(self.sim_data_folder + 'x_' + log_num, x_traj)
    np.save(self.sim_data_folder + 't_x_' + log_num, t_x)

    self.iter_num += 1
    print("sim_run: " + str(self.iter_num))

    return log_num

  def load_sim_trial(self, log_num):
    x_traj = np.load(self.sim_data_folder + 'x_' + log_num + '.npy')
    t_x = np.load(self.sim_data_folder + 't_x_' + log_num + '.npy')
    # lambda_traj = np.load(self.sim_data_folder + 'lambda_' + log_num + '.npy')

    return x_traj.transpose(), t_x

  def get_window_around_contact_event(self, t_x_sim, t_x, x_traj):
    # return whole trajectory for now
    # start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[0][0]
    # end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[0][0]
    start_idx = np.argwhere(np.isclose(t_x, t_x_sim[0], atol=5e-4))[1][0]
    end_idx = np.argwhere(np.isclose(t_x, t_x_sim[-1], atol=5e-4))[1][0]
    window = slice(start_idx, end_idx)
    return t_x[window], x_traj[window]

  def compute_loss(self, log_num, sim_id, params, plot=False):
    x_traj_hardware = self.x_trajs[log_num]
    t_x_hardware = self.t_x[log_num]
    # lambda_traj_hardware = self.lambda_trajs[log_num]

    x_traj_sim, t_x_sim = self.load_sim_trial(sim_id)
    t_x_hardware, x_traj_hardware = self.get_window_around_contact_event(t_x_sim, t_x_hardware, x_traj_hardware)
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
      plt.show()
    traj_loss = self.loss_func.CalculateLossTraj(x_traj_sim[:min_time_length], x_traj_hardware[:min_time_length])
    # regularization_loss = self.loss_func.CalculateLossParams(params)
    # return traj_loss + regularization_loss
    return traj_loss


if __name__ == '__main__':
  log_num = sys.argv[1]
  loss_func = cassie_loss_utils.CassieLoss()
  sim = DrakeCassieSim()
  sim.run(sim.default_drake_contact_params, log_num)
