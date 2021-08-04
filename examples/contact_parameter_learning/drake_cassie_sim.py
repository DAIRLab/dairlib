import numpy as np
import sys
import time
from scipy import interpolate
import cassie_loss_utils
import subprocess
import pickle

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml


class DrakeCassieSim():

  def __init__(self, drake_sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
    self.sim_data_folder = "/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/"
    self.start_time = 30.595
    self.sim_time = 0.5
    self.end_time = self.start_time + self.sim_time
    self.drake_sim_dt = drake_sim_dt
    self.default_drake_contact_params = {
      "mu_static": 0.8,
      "mu_ratio": 1.0,
      "pen_allow": 1e-5,
      "stiction_tol": 1e-3}
    self.loss_func = cassie_loss_utils.CassieLoss(loss_filename)

  def write_initial_state(self, x_init):
    gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/data/"

    def float_representer(dumper, value):
      text = '{0:.5f}'.format(value)
      return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)

    yaml.add_representer(float, float_representer)

    new_gains = x_init
    with open(gains_path + 'initial_state.yaml', 'r') as f:
      data = yaml.load(f, Loader=Loader)
      data['x_init'] = x_init.tolist()

      f = open(gains_path + 'initial_state.yaml', 'w')
      f.write(yaml.dump(data, default_flow_style=None))
      f.close()

  def save_params(self, params, sim_id):
    with open('drake_cassie_params/' + sim_id + '.pkl', 'wb') as f:
      pickle.dump(params, f, pickle.HIGHEST_PROTOCOL)

  def load_params(self, sim_id):
    with open('drake_cassie_params/' + sim_id + '.pkl', 'rb') as f:
      return pickle.load(f)

  def run(self, params, log_num):
    # params

    penetration_allowance = params['pen_allow']
    print(penetration_allowance)
    mu_static = params['mu_static']
    mu_kinetic = params['mu_ratio'] * params['mu_static']
    stiction_tol = params['stiction_tol']
    # delta_x_init = params['delta_x_init']
    terrain_height = 0.00
    x_traj = np.load(self.folder_path + 'x_' + log_num + '.npy')
    t = np.load(self.folder_path + 't_x_' + log_num + '.npy')

    x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
    x_init = x_interp(self.start_time)
    self.write_initial_state(x_init)
    simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_playback',
                     '--folder_path=%s' % self.folder_path,
                     '--end_time=%.3f' % self.end_time,
                     '--terrain_height=%.4f' % terrain_height,
                     '--start_time=%.3f' % self.start_time,
                     '--log_num=' + log_num,
                     '--penetration_allowance=%.5f' % penetration_allowance,
                     '--stiction_tol=%.5f' % stiction_tol,
                     '--mu_static=%.5f' % mu_static,
                     '--mu_kinetic=%.5f' % mu_kinetic,
                     # '--delta_x_init=%.5f' % delta_x_init,
                     ]
    # print((' ').join(simulator_cmd))
    simulator_process = subprocess.Popen(simulator_cmd)
    simulator_process.wait()
    x_traj = np.genfromtxt('x_traj.csv', delimiter=',', skip_header=True, dtype='f16')
    t_x = np.genfromtxt('t_x.csv', skip_header=True)
    t_x = np.append(t_x, self.start_time + self.sim_time)
    sim_id = log_num + str(np.abs(hash(frozenset(params))))
    # self.save_params(params, sim_id)
    np.save(self.sim_data_folder + 'x_traj' + sim_id, x_traj)
    np.save(self.sim_data_folder + 't_x' + sim_id, t_x)
    print('saving trial to: ' + 'x_traj' + sim_id)
    return sim_id

  def load_sim_trial(self, sim_id):
    x_traj = np.load(self.sim_data_folder + 'x_traj' + sim_id + '.npy')
    t_x = np.load(self.sim_data_folder + 't_x' + sim_id + '.npy')

    return x_traj, t_x

  def get_window_around_contact_event(self, x_traj, t_x):
    # return whole trajectory for now
    import pdb; pdb.set_trace()
    start_idx = np.argwhere(np.isclose(t_x, self.start_time + 0.0005))[0][0]
    end_idx = np.argwhere(np.isclose(t_x, self.end_time))[0][0]
    window = slice(start_idx, end_idx)
    return window, x_traj

  def compute_loss(self, log_num, sim_id):
    x_traj_log = np.load(self.folder_path + 'x_' + log_num + '.npy')
    t_x_log = np.load(self.folder_path + 't_x_' + log_num + '.npy')
    x_traj, t_x = self.load_sim_trial(sim_id)
    window, x_traj_in_window = self.get_window_around_contact_event(x_traj_log, t_x_log)
    import pdb; pdb.set_trace()

    return


if __name__ == '__main__':
  log_num = sys.argv[1]
  loss_func = cassie_loss_utils.CassieLoss()
  sim = DrakeCassieSim()
  sim.run(sim.default_drake_contact_params, log_num)
