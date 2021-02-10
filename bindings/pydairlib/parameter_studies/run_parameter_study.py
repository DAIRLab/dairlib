import subprocess
import time

import numpy as np


def main():
  folder_path = "/home/yangwill/Documents/research/projects/cassie" \
                "/jumping/saved_trajs/"
  trajectory_path = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/"
  results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/walking/logs/impact_invariance_param_study/"
  gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc/"
  trajectory_name = "walking_0.16.0"
  traj_name_controller = trajectory_name + "_processed"

  sim_time = 0.8
  # error_idx = 14, 15, 16, 17
  # order goes: lfoot x, rfoot x, lfoot z, rfoot z
  # error_idx = 17
  # start_time = 0.6
  start_time = 0.0

  # start_time = 0.00

  # time_offsets = np.linspace(0.0, 0.05, 51)
  # time_offsets = np.linspace(0.0, 0.01, 5)
  # time_offsets = np.linspace(-0.01, 0.01, 21)
  # x_error_values = np.linspace(-0.05, 0.05, 11)
  terrain_heights = np.arange(0.00, 0.05, 0.001)
  # buffer_times = np.linspace(0.0, 0.1, 101)
  penetration_allowances = np.array([1e-5, 1e-4, 1e-3])
  # penetration_allowances = np.array([5e-3])
  # penetration_allowances = np.logspace(np.log10(1e-5), np.log10(2e-4), 20)

  # Override for single run

  delay_time = -start_time
  init_fsm_state = 0
  realtime_rate = 0.5
  # Add an extra second to the runtime of the simulator process to account for start up and stopping time
  sim_run_time = sim_time / realtime_rate + 2.0
  controller_startup_time = 1.0
  parameter_file = open(results_folder + 'command_list.txt', 'w')

  for i in range(terrain_heights.shape[0]):
    for k in range(penetration_allowances.shape[0]):
      log_suffix = 'height_%.4f-stiff_%.5f' % (terrain_heights[i], penetration_allowances[k])
      log_path = results_folder + 'lcmlog-' + log_suffix
      print(log_path)
      controller_cmd = ['bazel-bin/examples/Cassie/run_osc_walking_controller_tracking',
                        '--traj_name=%s' % trajectory_name,
                        ]
      simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_init',
                       '--folder_path=%s' % trajectory_path,
                       '--traj_name=%s' % trajectory_name,
                       '--publish_rate=%d' % 4000.0,
                       '--end_time=%.3f' % sim_time,
                       '--dt=%.5f' % 8e-5,
                       '--terrain_height=%.4f' % terrain_heights[i],
                       '--penetration_allowance=%.5f' %
                       penetration_allowances[k],
                       '--target_realtime_rate=%.2f' % realtime_rate,
                       '--start_time=%.3f' % start_time,
                       ]
      lcm_logger_cmd = ['lcm-logger',
                        '-f',
                        '%s' % log_path,
                        ]
      save_gains_cmd = ['cp',
                        '%s' % gains_path + 'osc_walking_gains.yaml',
                        '%s' % results_folder + 'osc_walking_gains' + log_suffix + '.yaml']
      parameter_file.write(log_path + '\n')
      parameter_file.write(' '.join(controller_cmd) + '\n')
      parameter_file.write(' '.join(simulator_cmd) + '\n')
      parameter_file.write(' '.join(save_gains_cmd) + '\n')
      parameter_file.write('**********\n')
      controller_process = subprocess.Popen(controller_cmd)
      logger_process = subprocess.Popen(lcm_logger_cmd)
      save_gains_process = subprocess.Popen(save_gains_cmd)
      time.sleep(controller_startup_time)
      simulator_process = subprocess.Popen(simulator_cmd)
      time.sleep(sim_run_time)
      simulator_process.kill()
      controller_process.kill()
      logger_process.kill()
      save_gains_process.kill()

  parameter_file.close()


if __name__ == "__main__":
  main()
