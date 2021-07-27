import numpy as np
import sys
import time
from scipy import interpolate
import subprocess

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml


def write_initial_state(x_init):
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

def run_sim(params):
  # params
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  start_time = 30.595
  sim_time = 0.5
  end_time = start_time + sim_time
  penetration_allowance = 2e-3
  terrain_height = 0.00
  realtime_rate = 0.25
  log_num = sys.argv[1]

  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')

  x_interp = interpolate.interp1d(t[:,0], x_traj, axis=0, bounds_error=False)
  x_init = x_interp(start_time)
  write_initial_state(x_init)
  simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_playback',
                   '--folder_path=%s' % folder_path,
                   '--end_time=%.3f' % end_time,
                   '--dt=%.5f' % 8e-5,
                   '--terrain_height=%.4f' % terrain_height,
                   '--penetration_allowance=%.5f' %
                   penetration_allowance,
                   '--target_realtime_rate=%.2f' % realtime_rate,
                   '--start_time=%.3f' % start_time,
                   '--log_num=' + log_num,
                   ]
  simulator_process = subprocess.Popen(simulator_cmd)
  time.sleep(sim_time / realtime_rate)
  simulator_process.kill()


if __name__ == '__main__':
  params = {}
  run_sim(params)