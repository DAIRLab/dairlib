import subprocess
import time

import numpy as np


def main():


    folder_path = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/saved_trajs/"
    trajectory_name = "walking_4_24"
    log_folder = "/home/yangwill/Documents/research/projects/five_link_biped" \
               "/hybrid_lqr/logs/parameter_study/rfoot_z_velocity/"
    sim_time = 1.0
    penetration_allowance = 2e-4
    # error_idx = 1 + 7
    error_idx = 14
    start_time = 0.215

    # time_offsets = np.linspace(0.0, 0.01, 21)
    # x_error_values = np.linspace(-0.05, 0.05, 11)
    time_offsets = np.linspace(0.0, 0.01, 21)
    x_error_values = np.linspace(-0.05, 0.05, 11)
    # x_error_values = np.array([0.0])
    parameter_file = open(log_folder + 'command_list.txt', 'w')
    print(time_offsets)
    # import pdb; pdb.set_trace()

    for i in range(time_offsets.shape[0]):
        for j in range(x_error_values.shape[0]):
            log_path = log_folder + 'lcmlog-delay_%.4f-error_%.2f' % (
                    time_offsets[i], x_error_values[j])
            print(log_path)
            controller_cmd = ['bazel-bin/examples/five_link_biped/run_lqr',
                              '--contact_driven=%d' % False,
                              '--folder_path=%s' % folder_path,
                              '--trajectory_name=%s' % trajectory_name,
                              '--time_offset=%.4f' % time_offsets[i],
                              ]
            simulator_cmd = ['bazel-bin/examples/five_link_biped/rabbit_sim',
                             '--folder_path=%s' % folder_path,
                             '--trajectory_name=%s' % trajectory_name,
                             '--publish_rate=%d' % 8000.0,
                             '--sim_time=%.3f' % sim_time,
                             '--dt=%.5f' % 1e-5,
                             '--penetration_allowance=%.4f' %
                             penetration_allowance,
                             '--start_time=%.3f' % start_time,
                             '--error_idx=%d' % error_idx,
                             '--error=%.3f' % x_error_values[j]
                             ]
            lcm_logger_cmd = ['lcm-logger',
                              '-f',
                              '%s' % log_path,
                              ]
            parameter_file.write(log_path + '\n')
            parameter_file.write(' '.join(controller_cmd) + '\n')
            parameter_file.write(' '.join(simulator_cmd) + '\n')
            parameter_file.write('**********\n')
            # print(log_path)
            # print(controller_cmd)
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