


import numpy as np
import scipy.io as io
import sys

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml

np.set_printoptions(precision = 3, suppress=True)
# gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc_jump/"
save_folder = '/home/yangwill/Documents/research/projects/cassie/hardware/gain_tuning/' # folder where all results are saved'

def write_walking_gains(point, filename):
  gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc/"
  def float_representer(dumper, value):
    text = '{0:.4f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)
  yaml.add_representer(float, float_representer)

  new_gains = point
  with open(gains_path + 'osc_walking_gains.yaml', 'r') as f:
    data = yaml.load(f, Loader=Loader)
    # data['k_ff_sagittal'] = float(new_gains[0])
    # data['k_fb_sagittal'] = float(new_gains[1])
    # data['k_ff_lateral'] = float(new_gains[2])
    # data['k_fb_lateral'] = float(new_gains[3])
    # data['max_CoM_to_footstep_dist'] = float(new_gains[4])
    data['footstep_offset'] = float(new_gains[0])
    # data['center_line_offset'] = float(new_gains[6])
    # data['mid_foot_height'] = float(new_gains[7])
    # data['final_foot_height'] = float(new_gains[8])
    # data['final_foot_velocity_z'] = float(new_gains[9])

    f = open(gains_path + filename, 'w')
    f.write(yaml.dump(data, default_flow_style=None))
    f.close()

def write_jumping_gains(point, filename):
  gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc_jump/"

  def float_representer(dumper, value):
    text = '{0:.4f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)
  yaml.add_representer(float, float_representer)

  new_gains = point
  with open(gains_path + 'osc_jumping_gains.yaml', 'r') as f:
    data = yaml.load(f, Loader=Loader)
    data['w_input'] = float(new_gains[0])
    data['w_accel'] = float(new_gains[1])
    data['w_soft_constraint'] = float(new_gains[2])
    data['w_swing_toe'] = float(new_gains[3])
    data['w_hip_yaw'] = float(new_gains[4])
    data['CoMW'] = np.diag(new_gains[5:8]).flatten().tolist()
    data['PelvisRotW'] = np.diag(new_gains[8:11]).flatten().tolist()
    data['FlightFootW'] = np.diag(new_gains[11:14]).flatten().tolist()
    data['hip_yaw_kp'] = float(new_gains[14:15])
    data['CoMKp'] = np.diag(new_gains[15:18]).flatten().tolist()
    data['PelvisRotKp'] = np.diag(new_gains[18:21]).flatten().tolist()
    data['FlightFootKp'] = np.diag(new_gains[21:24]).flatten().tolist()
    data['hip_yaw_kd'] = float(new_gains[24:25])
    data['CoMKd'] = np.diag(new_gains[25:28]).flatten().tolist()
    data['PelvisRotKd'] = np.diag(new_gains[28:31]).flatten().tolist()
    data['FlightFootKd'] = np.diag(new_gains[31:34]).flatten().tolist()

    f = open(gains_path + filename, 'w')
    f.write(yaml.dump(data, default_flow_style=None))
    f.close()

def test_jumping_gains():

  visited_points = np.load(save_folder + 'visited_points.npy')
  prev_points_to_sample = np.load(save_folder + 'prev_points_to_sample.npy')
  subspace_dict = np.load(save_folder + 'subspace_file.npy', allow_pickle=True).item()

  print(visited_points.shape[0])
  write_jumping_gains(visited_points[-2], 'osc_jumping_gains_a.yaml')
  write_jumping_gains(visited_points[-1], 'osc_jumping_gains_b.yaml')

  preferences = {}
  preferences['X'] = np.stack((visited_points[-2], visited_points[-1]), axis=1)
  preferences['y'] = input('0 or 1?: ')
  preferences['last_action'] = visited_points[-2 + int(preferences['y'])]
  io.savemat(save_folder + 'gain_tuning_data', preferences)

def test_walking_gains():

  visited_points = np.load(save_folder + 'visited_points.npy')
  prev_points_to_sample = np.load(save_folder + 'prev_points_to_sample.npy')
  subspace_dict = np.load(save_folder + 'subspace_file.npy', allow_pickle=True).item()

  print(visited_points.shape[0])
  write_walking_gains(visited_points[-2], 'osc_walking_gains_a.yaml')
  write_walking_gains(visited_points[-1], 'osc_walking_gains_b.yaml')

  preferences = {}
  preferences['X'] = np.stack((visited_points[-2], visited_points[-1]), axis=1)
  preferences['y'] = input('0 or 1?: ')
  preferences['last_action'] = visited_points[-2 + int(preferences['y'])]
  io.savemat(save_folder + 'gain_tuning_data', preferences)

def print_best_gains():

  subspace_file = save_folder + "/subspace_file.npy"
  subspace_dict = np.load(subspace_file,allow_pickle=True).item()
  import pdb; pdb.set_trace()

if __name__ == '__main__':
  global gains_path

  # print_best_gains()
  # test_jumping_gains()
  test_walking_gains()