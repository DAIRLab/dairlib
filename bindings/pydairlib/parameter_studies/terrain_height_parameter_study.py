import subprocess
import time

import numpy as np


def main():

  folder_path = "/home/yangwill/Documents/research/projects/cassie" \
                "/jumping/saved_trajs/"
  file_name_sim = "target_trajs/jumping_0.3h_0.4d"
  file_name_controller = "target_trajs/jumping_0.3h_0.4d"
  log_folder = "/home/yangwill/Documents/research/projects/cassie/jumping" \
               "/logs/parameter_study/terrain_height/"
  mode_name = "state_input_trajectory"

  sim_time = 5.0
  # error_idx = 14, 15, 16, 17
  # order goes: lfoot x, rfoot x, lfoot z, rfoot z
  # error_idx = 17
  # start_time = 0.6
  # start_time = 0.5754
  start_time = 0.00

  penetration_allowances = np.array([1e-5, 1e-4])
  terrain_heights = np.linspace(-0.1, -0.05, 6)
  # Override for single run

  # penetration_allowances = np.array([5e-5, 4e-4])
  # penetration_allowances = np.array([1e-3])
  # time_offsets = np.array([0.000])
  # x_error_values = np.array([0.000])
  # start_time = 0.0
  delay_time = 1.4
  x_offset = 0.095
  init_fsm_state = 0
  parameter_file = open(log_folder + 'command_list.txt', 'w')

  for i in range(terrain_heights.shape[0]):
    for k in range(penetration_allowances.shape[0]):
      log_path = log_folder + \
                 'lcmlog-height_%.4f-pen_%.5f' % (
                   terrain_heights[i], penetration_allowances[k])
      print(log_path)
      controller_cmd = ['bazel-bin/examples/Cassie/run_osc_jumping_controller',
                        '--delay_time=%.3f' % delay_time,
                        '--x_offset=%.3f' % x_offset,
                        '--init_fsm_state=%d' % init_fsm_state,
                        '--traj_name=%s' % file_name_controller,
                        '--mode_name=%s' % mode_name,
                        ]
      simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim',
                       # '--folder_path=%s' % folder_path,
                       # '--trajectory_name=%s' % trajectory_name,
                       # '--file_name=%s' % file_name_sim,
                       '--publish_rate=%d' % 2000.0,
                       '--end_time=%.3f' % sim_time,
                       '--dt=%.5f' % 5e-5,
                       '--init_height=%.3f' % 1.0,
                       '--penetration_allowance=%.5f' %
                       penetration_allowances[k],
                       '--target_realtime_rate=0.5',
                       '--terrain_height=%.4f' % terrain_heights[i],
                       # '--start_time=%.3f' % start_time,
                       ]
      lcm_logger_cmd = ['lcm-logger',
                        '-f',
                        '%s' % log_path,
                        ]
      parameter_file.write(log_path + '\n')
      parameter_file.write(' '.join(controller_cmd) + '\n')
      parameter_file.write(' '.join(simulator_cmd) + '\n')
      parameter_file.write('**********\n')
      controller_process = subprocess.Popen(controller_cmd)
      logger_process = subprocess.Popen(lcm_logger_cmd)
      time.sleep(1)
      simulator_process = subprocess.Popen(simulator_cmd)
      time.sleep(10)
      simulator_process.kill()
      controller_process.kill()
      logger_process.kill()

  parameter_file.close()
if __name__ == "__main__":
  main()
