import subprocess
import time
from tqdm import *
import lcm
from bindings.pydairlib.common import plot_styler, plotting_utils
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
import matplotlib.pyplot as plt
import numpy as np
import yaml
import dairlib


def process_c3_channel(data):
    t = []
    x = []
    for msg in data:
        t.append(msg.utime / 1e6)
        x.append(msg.state)
    t = np.array(t)
    x = np.array(x)

    return {'t': t,
            'x': x}

def load_c3_channels(data, c3_target_state_channel, c3_actual_state_channel, c3_debug_output_channel):
    c3_target = process_c3_channel(data[c3_target_state_channel])
    c3_actual = process_c3_channel(data[c3_actual_state_channel])
    c3_output = mbp_plots.process_c3_debug(data[c3_debug_output_channel])
    return c3_target, c3_actual, c3_output

def load_lcm_logs():
    config_file = 'parameter_study_config.yaml'
    parameters_directory = 'examples/franka/parameters/'
    lcm_channels_sim = 'lcm_channels_simulation.yaml'

    with open(parameters_directory + lcm_channels_sim) as f:
        filedata = f.read()
        lcm_channels = yaml.load(filedata)

    with open('bindings/pydairlib/analysis/tray_balance_study/' + config_file) as f:
        filedata = f.read()
        param_study = yaml.load(filedata)

    start_time = 0
    duration = -1

    c3_channels = {lcm_channels['c3_target_state_channel']: dairlib.lcmt_c3_state,
                   lcm_channels['c3_actual_state_channel']: dairlib.lcmt_c3_state,
                   lcm_channels['c3_debug_output_channel']: dairlib.lcmt_c3_output}
    callback = load_c3_channels
    mass_range = np.arange(0.5, 2.0, 0.05)
    # mu_range = np.arange(0.3, 0.8, 0.01)
    effective_mu_range = np.arange(0.1, 0.71, 0.1)

    print(effective_mu_range.shape[0])
    all_successes = np.zeros((effective_mu_range.shape[0], 3))
    for i in range(effective_mu_range.shape[0]):
        print('%02d', effective_mu_range[i])
        # print(mass_range[i])
        # ps = plot_styler.PlotStyler(nrows=2)

        successes = np.zeros(3)

        for j in range(5):
            # ps = plot_styler.PlotStyler()
            log_filename = param_study['results_folder'] + param_study['parameter'][1] + '/simlog-' + '%02d_%1d' % (i, j)
            print(log_filename)
            log = lcm.EventLog(log_filename, "r")
            c3_target, c3_actual, c3_output = \
                get_log_data(log, c3_channels, start_time, duration, callback,
                             lcm_channels['c3_target_state_channel'], lcm_channels['c3_actual_state_channel'],
                             lcm_channels['c3_debug_output_channel'])
            length = min(c3_target['t'].shape[0], c3_actual['t'].shape[0])
            # ps.plot(c3_actual['t'], c3_actual['x'][:, 8:9], subplot_index=0)
            # ps.plot(c3_actual['t'], c3_actual['x'][:, 9:10], subplot_index=0)
            # ps.plot(c3_actual['t'], c3_actual['x'][:, 1:2], subplot_index=0)
            # ps.plot(c3_actual['t'], c3_actual['x'][:, 2:3], subplot_index=0)
            first_target = np.array([0.45, 0, 0.485])
            second_target = np.array([0.45, 0, 0.6])
            third_target = np.array([0.7, 0.0, 0.485])
            reached_first_target = np.any(np.all(np.isclose(c3_target['x'][:, 7:10], second_target, atol=1e-3), axis=1))
            reached_second_target = np.any(np.all(np.isclose(c3_target['x'][:, 7:10], third_target, atol=1e-3), axis=1))
            reached_third_target = reached_second_target and np.linalg.norm(c3_target['x'][:length, 7:10] - c3_actual['x'][:length, 7:10], axis=1)[-1] < 0.1
            task_success = np.array([reached_first_target, reached_second_target, reached_third_target])
            # print('reached_targets: ', task_success)
            successes += task_success
            # print('reached_second_target: ', reached_second_target)
            # print('reached_third_target: ', reached_third_target)
            t_c3_slice = slice(c3_output['t'].size)
            # mbp_plots.plot_c3_inputs(c3_output, t_c3_slice, 1, ps)
            # plt.show()
        print(successes)
        all_successes[i] = successes
    np.save('target_successes_23', all_successes )

def plot_logs():
    all_successes = np.load('all_successes.npy')
    bar_width = 0.025
    bar_positions = np.arange(all_successes.shape[0])
    n = all_successes.shape[0]
    # mu_range = np.arange(0.3, 0.74, 0.05)
    effective_mu_range = np.arange(0.2, 0.61, 0.05)

    plt.rc('legend', fontsize=36)
    plt.rc('axes', labelsize=36, titlesize=36)
    plt.rc('xtick', labelsize=36)
    plt.rc('ytick', labelsize=36)
    # Plotting bars for each task
    for i in range(n):
        # for j in range(3):
        # plt.bar(bar_positions[i], all_successes[i, 2] / 30, width=bar_width)
        # plt.bar(effective_mu_range[i], all_successes[i, 2] / all_successes[i, 0], width=bar_width, color='grey')
        plt.bar(effective_mu_range[i], all_successes[i, 0] / 30, width=bar_width, color='grey')
    plt.ylabel('Target Success Rate')
    plt.xlabel('Tray/End Effector Friction Coefficient')

    # plt.savefig('/home/yangwill/Pictures/plot_styler/' + 'target_1_success')
    # plt.legend()
    # for i in range(3):
    #     plt.bar(bar_positions + i * (n + 1), all_successes[:, i], label=f'Task {i + 1}')

    plt.show()

def main():
    load_lcm_logs()
    # plot_logs()












if __name__ == '__main__':
    main()