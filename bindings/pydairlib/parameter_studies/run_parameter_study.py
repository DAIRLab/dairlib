import subprocess
import time

import numpy as np


def main():

  folder_path = "/home/yangwill/Documents/research/projects/cassie" \
                "/jumping/saved_trajs/"
  file_name_sim = "June_5_jumping_0.2_for_sim"
  file_name_controller = "June_5_jumping_0.2_processed"
  trajectory_name = "cassie_jumping_trajectory_x"
  log_folder = "/home/yangwill/Documents/research/projects/cassie/jumping" \
               "/logs/parameter_study/new_sim_3/"

  sim_time = 5.0
  # error_idx = 14, 15, 16, 17
  # order goes: lfoot x, rfoot x, lfoot z, rfoot z
  # error_idx = 17
  # start_time = 0.6
  start_time = 0.5754
  # start_time = 0.00

  time_offsets = np.linspace(0.0, 0.05, 51)
  # time_offsets = np.linspace(0.0, 0.01, 5)
  # time_offsets = np.linspace(-0.01, 0.01, 21)
  # x_error_values = np.linspace(-0.05, 0.05, 11)
  # buffer_times = np.linspace(0.0, 0.1, 101)
  # penetration_allowances = np.array([1e-4, 1e-3, 5e-3])
  penetration_allowances = np.array([5e-3])
  # penetration_allowances = np.logspace(np.log10(1e-5), np.log10(2e-4), 20)

  # Override for single run

  # penetration_allowances = np.array([5e-5, 4e-4])
  # penetration_allowances = np.array([1e-3])
  # time_offsets = np.array([0.000])
  # x_error_values = np.array([0.000])
  # start_time = 0.0
  delay_time = -start_time
  x_offset = 0.01
  init_fsm_state = 2
  parameter_file = open(log_folder + 'command_list.txt', 'w')
  print(time_offsets)

  for i in range(time_offsets.shape[0]):
    for k in range(penetration_allowances.shape[0]):
      log_path = log_folder + \
                 'lcmlog-delay_%.4f-pen_%.5f' % (
                   time_offsets[i], penetration_allowances[k])
      print(log_path)
      controller_cmd = ['bazel-bin/examples/Cassie/run_osc_jumping_controller',
                        # '--delay_time=%.3f' % delay_time,
                        '--x_offset=%.3f' % x_offset,
                        '--init_fsm_state=%d' % init_fsm_state,
                        '--traj_name=%s' % file_name_controller,
                        '--transition_delay=%.4f' % time_offsets[i],
                        ]
      simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim',
                       '--folder_path=%s' % folder_path,
                       '--trajectory_name=%s' % trajectory_name,
                       '--file_name=%s' % file_name_sim,
                       '--publish_rate=%d' % 4000.0,
                       '--end_time=%.3f' % sim_time,
                       '--dt=%.5f' % 5e-5,
                       '--penetration_allowance=%.5f' %
                       penetration_allowances[k],
                       '--target_realtime_rate=0.5',
                       '--start_time=%.3f' % start_time,
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
      time.sleep(3)
      simulator_process = subprocess.Popen(simulator_cmd)
      time.sleep(10)
      simulator_process.kill()
      controller_process.kill()
      logger_process.kill()

  parameter_file.close()
if __name__ == "__main__":
  main()
