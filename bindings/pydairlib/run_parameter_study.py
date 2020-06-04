import subprocess
import time

import numpy as np


def main():

  folder_path = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/saved_trajs/"
  trajectory_name = "walking_4_24"
  # log_folder = "/home/yangwill/Documents/research/projects/five_link_biped" \
  #            "/hybrid_lqr/logs/parameter_study/stiffness/"
  log_folder = "/home/yangwill/Documents/research/projects/five_link_biped" \
             "/hybrid_lqr/logs/parameter_study/switching_time_adjusted/"
  # log_folder = "/home/yangwill/Documents/research/projects/five_link_biped" \
  #              "/hybrid_lqr/logs/"

  sim_time = 1.0
  # error_idx = 14, 15, 16, 17
  # order goes: lfoot x, rfoot x, lfoot z, rfoot z
  error_idx = 17
  start_time = 0.20
  # start_time = 0.00

  # time_offsets = np.linspace(0.0, 0.01, 21)
  time_offsets = np.linspace(-0.01, 0.01, 21)
  x_error_values = np.linspace(-0.05, 0.05, 11)
  # buffer_times = np.linspace(0.0, 0.1, 101)
  # penetration_allowances = np.array([1e-5, 1e-4, 2e-4])
  # penetration_allowances = np.logspace(np.log10(1e-5), np.log10(2e-4), 20)

  # Override for single run

  # penetration_allowances = np.array([5e-5, 4e-4])
  penetration_allowances = np.array([2e-4])
  # time_offsets = np.array([0.000])
  # x_error_values = np.array([0.000])
  # start_time = 0.0

  parameter_file = open(log_folder + 'command_list.txt', 'w')
  print(time_offsets)

  for i in range(time_offsets.shape[0]):
    for j in range(x_error_values.shape[0]):
    # for j in range(buffer_times.shape[0]):
      for k in range(penetration_allowances.shape[0]):
        log_path = log_folder + \
                   'lcmlog-delay_%.4f-error_%.2f-pen_%.5f' % (
                     time_offsets[i], x_error_values[j],
                     penetration_allowances[k])
        # log_path = log_folder + \
        #            'lcmlog-delay_%.4f-error_%.3f-pen_%.5f' % (
        #              time_offsets[i], buffer_times[j],
        #              penetration_allowances[k])
        # log_path = log_folder + \
        #            'lcmlog-delay_%.4f-error_%.2f' % (
        #         time_offsets[i], x_error_values[j])
        print(log_path)
        controller_cmd = ['bazel-bin/examples/five_link_biped/run_lqr',
                          '--contact_driven=%d' % False,
                          '--folder_path=%s' % folder_path,
                          '--naive=%d' % False,
                          # '--buffer_time=%.3f' % buffer_times[j],
                          '--trajectory_name=%s' % trajectory_name,
                          '--time_offset=%.4f' % time_offsets[i],
                          ]
        simulator_cmd = ['bazel-bin/examples/five_link_biped/rabbit_sim',
                         '--folder_path=%s' % folder_path,
                         '--trajectory_name=%s' % trajectory_name,
                         '--publish_rate=%d' % 8000.0,
                         '--sim_time=%.3f' % sim_time,
                         '--dt=%.5f' % 1e-5,
                         '--penetration_allowance=%.5f' %
                         penetration_allowances[k],
                         '--start_time=%.3f' % start_time,
                         '--error_idx=%d' % error_idx,
                         '--error=%.3f' % x_error_values[j]
                         ]
        lcm_logger_cmd = ['lcm-logger',
                          # '-f',
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
        time.sleep(5)
        controller_process.kill()
        logger_process.kill()

  parameter_file.close()
if __name__ == "__main__":
  main()
