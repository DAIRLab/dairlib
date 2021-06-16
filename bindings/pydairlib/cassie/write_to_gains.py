


import numpy as np
import sys

try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
import yaml

def main():

  def float_representer(dumper, value):
    text = '{0:.4f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)
  yaml.add_representer(float, float_representer)

  np.set_printoptions(precision = 3, suppress=True)
  save_folder = '/home/yangwill/Documents/research/projects/cassie/hardware/gain_tuning/' # folder where all results are saved'
  gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc_jump/"

  visited_points = np.load(save_folder + 'visited_points.npy')
  prev_points_to_sample = np.load(save_folder + 'prev_points_to_sample.npy')
  subspace_dict = np.load(save_folder + 'subspace_file.npy', allow_pickle=True).item()

  f = open(gains_path + 'osc_jumping_gains.yaml', 'r')
  filedata = f.read()
  f.close()
  print(visited_points.shape[0])
  iter = sys.argv[1]
  new_gains = visited_points[int(iter)]


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

    f = open(gains_path + 'osc_jumping_gains_edit.yaml', 'w')
    f.write(yaml.dump(data, default_flow_style=None))
    f.close()
    # import pdb; pdb.set_trace()


  # newdata = filedata.replace('w_input: %.3f', np.array2string(new_gains[0]))
  # f = open(gains_path + 'osc_jumping_gains_edit.yaml', 'w')
  # f.write(newdata)
  # f.close()


if __name__ == '__main__':
  main()