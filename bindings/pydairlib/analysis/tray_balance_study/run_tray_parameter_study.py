import subprocess
import time
from tqdm import *
import lcm
import dairlib.lcmt_radio_out

import numpy as np
import yaml
def main():
    config_file = 'parameter_study_config.yaml'
    sim_time = 10.0
    cooldown_time = 10.0
    start_time = 0
    realtime_rate = 1.0
    num_trials = 10
    c3_start_up_time = 4.0
    sim_run_time = sim_time / realtime_rate
    controller_startup_time = 0.1

    with open('bindings/pydairlib/analysis/tray_balance_study/' + config_file) as f:
        filedata = f.read()
        config = yaml.load(filedata)

    # Add an extra second to the runtime of the simulator process to account for start up and stopping time

    lcm_logger_cmd = ''
    publisher = lcm.LCM()

    # parameter = 'mu_c3'
    # parameter = 'mass_real'
    parameter = config['parameter'][1]

    nominal_mu_value = 'mu: [0.6, 0.6, 0.6, 0.1, 0.1, 0.1, 0.1]'
    # nominal_real_mu_value = '<drake:mu_dynamic>0.4</drake:mu_dynamic>'
    nominal_real_mu_value = '<drake:mu_dynamic value="0.6"/>'
    nominal_tray_mass = '<mass>1</mass>'

    # mu_range = np.arange(0.3, 0.8, 0.05)
    # mass_range = np.arange(0.5, 2.0, 0.05)
    effective_mu_range = np.arange(0.2, 0.71, 0.05)
    tray_mu = 0.4

    initial_radio_msg = dairlib.lcmt_radio_out()
    start_c3_radio_msg = dairlib.lcmt_radio_out()
    initial_radio_msg.channel[13] = 1
    initial_radio_msg.channel[11] = 0
    initial_radio_msg.channel[14] = 1
    start_c3_radio_msg.channel[13] = 1
    start_c3_radio_msg.channel[11] = 0
    start_c3_radio_msg.channel[14] = 0

    # for i in trange(mu_range.shape[0]):
    for i in trange(effective_mu_range.shape[0]):
        plate_mu = 0.5 * effective_mu_range[i] / (1 - effective_mu_range[i]/(2 * tray_mu))

        # modified_mu_value = 'mu: [%.2f, %.2f, %.2f, 0.1, 0.1, 0.1, 0.1]' % (mu_range[i], mu_range[i], mu_range[i])
        # modified_mass_value = '<mass> %.1f </mass>' % (mass_range[i])
        # modified_real_mu_value = '<drake:mu_dynamic>%.2f</drake:mu_dynamic>' % (mu_range[i])
        modified_real_mu_value = '<drake:mu_dynamic value="%.2f"/>' % (plate_mu)

        # f = open(gains_path + gain_filename, 'r')
        f = open(config['model_path'] + config['nominal_model_filename'], 'r')

        filedata = f.read()
        f.close()
        # newdata = filedata.replace(nominal_mu_value, modified_mu_value)
        newdata = filedata.replace(nominal_real_mu_value, modified_real_mu_value)
        # newdata = filedata.replace(nominal_tray_mass, modified_mass_value)

        # f = open(config['gains_path'] +  config['modified_c3_gain_filename.yaml'], 'w')
        f = open(config['model_path'] + config['modified_model_filename'], 'w')
        print(config['model_path'] + config['modified_model_filename'])
        f.write(newdata)
        f.close()
        if parameter == 'mass_real':
            fix_inertia_process = subprocess.Popen(config['fix_inertia_cmd'].split(' '))
            time.sleep(1.0)


        for j in trange(num_trials):
            log_path = config['results_folder'] + parameter +  '/simlog-' + '%02d_%1d' % (i, j)
            lcm_logger_cmd = ['lcm-logger',
                              '-f',
                              '%s' % log_path,
                              ]

            osc_process = subprocess.Popen(config['osc_cmd'])
            c3_process = subprocess.Popen(config['c3_cmd'])
            sim_process = subprocess.Popen(config['sim_cmd'])
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
