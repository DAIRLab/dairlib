import subprocess
import time
from tqdm import *
import lcm
import dairlib.lcmt_radio_out

import numpy as np
def main():
    trajectory_path = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/"
    gains_path = 'examples/franka/parameters/'
    model_path = 'examples/franka/urdf/'
    results_folder = '/media/yangwill/backups/home/yangwill/Documents/research/projects/franka/logs/2024/01_22_24/parameter_study'
    sim_time = 10.0
    cooldown_time = 10.0
    start_time = 0
    realtime_rate = 1.0
    num_trials = 10

    sim_cmd = 'bazel-bin/examples/franka/franka_sim'
    osc_cmd = 'bazel-bin/examples/franka/franka_osc_controller'
    c3_cmd = 'bazel-bin/examples/franka/franka_c3_controller'
    fix_inertia_cmd = 'python3 -m pydrake.multibody.fix_inertia --in_place examples/franka/urdf/tray_parameter_sweep.sdf'

    # Add an extra second to the runtime of the simulator process to account for start up and stopping time
    sim_run_time = sim_time / realtime_rate + 1.0
    c3_start_up_time = 4.0
    controller_startup_time = 0.1
    lcm_logger_cmd = ''
    publisher = lcm.LCM()


    gain_filename = 'franka_c3_options_side_supports.yaml'
    model_filename = 'tray.sdf'
    # parameter = 'mu_c3'
    parameter = 'mass_real'

    nominal_mu_value = 'mu: [0.6, 0.6, 0.6, 0.1, 0.1, 0.1, 0.1]'
    nominal_tray_mass = '<mass>1</mass>'

    initial_radio_msg = dairlib.lcmt_radio_out()
    start_c3_radio_msg = dairlib.lcmt_radio_out()
    initial_radio_msg.channel[13] = 1
    initial_radio_msg.channel[11] = 1
    initial_radio_msg.channel[14] = 1
    start_c3_radio_msg.channel[13] = 1
    start_c3_radio_msg.channel[11] = 1
    start_c3_radio_msg.channel[14] = 0

    mu_range = np.arange(0.3, 0.8, 0.01)
    mass_range = np.arange(0.5, 2.0, 0.05)

    # for i in range(0, disturbances.shape[0]):
    for i in trange(mu_range.shape[0]):
        for j in trange(num_trials):
            log_path = results_folder + '/' + parameter +  '/simlog-' + '%02d_%1d' % (i, j)

            modified_mu_value = 'mu: [%.2f, %.2f, %.2f, 0.1, 0.1, 0.1, 0.1]' % (mu_range[i], mu_range[i], mu_range[i])
            modified_mass_value = '<mass> %.1f </mass>' % (mass_range[i])

            # f = open(gains_path + gain_filename, 'r')
            f = open(model_path + model_filename, 'r')

            filedata = f.read()
            f.close()
            # newdata = filedata.replace(nominal_mu_value, modified_mu_value)
            newdata = filedata.replace(nominal_tray_mass, modified_mass_value)

            # f = open(gains_path + 'franka_c3_options_parameter_sweep.yaml', 'w')
            f = open(model_path + 'tray_parameter_sweep.sdf', 'w')
            f.write(newdata)
            f.close()
            fix_inertia_process = subprocess.Popen(fix_inertia_cmd.split(' '))
            time.sleep(1.0)

            lcm_logger_cmd = ['lcm-logger',
                              '-f',
                              '%s' % log_path,
                              ]

            osc_process = subprocess.Popen(osc_cmd)
            c3_process = subprocess.Popen(c3_cmd)
            sim_process = subprocess.Popen(sim_cmd)
            time.sleep(controller_startup_time)
            publisher.publish("RADIO", initial_radio_msg.encode())
            logger_process = subprocess.Popen(lcm_logger_cmd)
            time.sleep(c3_start_up_time)
            publisher.publish("RADIO", start_c3_radio_msg.encode())
            time.sleep(sim_run_time)
            osc_process.kill()
            c3_process.kill()
            sim_process.kill()
            logger_process.kill()
            time.sleep(cooldown_time)

if __name__ == "__main__":
    main()
    # construct_success_plot()
    # convert_logs_to_costs()
    # plot_success()
    # plot_costs()
