import subprocess
import time
import lcm
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.colors
import fileinput
import dairlib
from bindings.pydairlib.common import plot_styler, plotting_utils
from bindings.pydairlib.cassie.cassie_utils import *
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from bindings.pydairlib.lcm import lcm_trajectory
from bindings.pydairlib.multibody import MakeNameToPositionsMap, \
  MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
  CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap
import pydairlib.analysis.cassie_plotting_utils as cassie_plots
import numpy as np
import glob
from matplotlib.ticker import FormatStrFormatter



def main():
  trajectory_path = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/"
  controller_type = "jumping"
  parameter = "landing_delay"
  simulator = 'DRAKE'
  # trajectory_names = ['jump', 'box_jump', 'long_jump', 'down_jump']
  trajectory_names = ['down_jump']
  gains_path = ''
  trajectory_name = ''
  results_folder = ''
  delay_time = 2.0
  traj_time = 5.0
  sim_time = 0
  start_time = 0
  if controller_type == 'jumping':
    gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc_jump/"
    results_folder = "/media/yangwill/backups/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/2023/param_study/"
    sim_time = delay_time + traj_time

  landing_times = np.arange(0.000, 0.055, 0.005)
  # nominal_delay = 0.040 # box
  # nominal_delay = 0.025 # long
  nominal_delay = 0.065 # down
  # nominal_delay = 0.000 # jump
  landing_times += nominal_delay - 0.5 * 0.05
  # impact_thresholds = np.arange(0.000, 0.100, 0.025)
  impact_thresholds = np.arange(0.000, 0.125, 0.025)

  realtime_rate = 0.5
  publish_rate = 2000.0

  # Add an extra second to the runtime of the simulator process to account for start up and stopping time
  sim_run_time = sim_time / realtime_rate + 1.0
  controller_startup_time = 1.0
  parameter_file = open(results_folder + 'command_list.txt', 'w')
  controller_cmd = ''
  simulator_cmd = ''
  lcm_logger_cmd = ''
  save_gains_cmd = ''

  # for i in range(0, disturbances.shape[0]):
  for traj in trajectory_names:
    for impact_threshold in impact_thresholds:
      for landing_time in landing_times:
        log_suffix = ''
        if parameter == 'landing_delay':
          log_suffix = 't_%.3f' % (landing_time) + '_duration_%.3f' % (
            impact_threshold)
        log_path = results_folder + traj + '/lcmlog-' + log_suffix
        print(log_path)
        gain_filename = ''
        if traj == 'jump' or traj == 'long_jump':
          if traj == 'jump':
            gain_filename = 'osc_jumping_gains.yaml'
          elif traj == 'long_jump':
            gain_filename = 'osc_jumping_gains_long.yaml'
          controller_cmd = [
            'bazel-bin/examples/Cassie/run_osc_jumping_controller',
            '--delay_time=%.1f' % delay_time,
            '--channel_u=OSC_JUMPING',
            '--gains_filename=examples/Cassie/osc_jump/osc_jumping_gains_param.yaml',
            '--traj_name=%s' % traj,
          ]
          simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim',
                           '--init_height=%.1f' % 0.9,
                           '--toe_spread=0.12',
                           '--target_realtime_rate=%.2f' % realtime_rate,
                           '--dt=%.5f' % 1e-3,
                           '--publish_rate=%d' % publish_rate,
                           '--end_time=%.3f' % sim_time,
                           '--channel_u=OSC_JUMPING',
                           ]
        elif traj == 'box_jump':
          controller_cmd = [
            'bazel-bin/examples/Cassie/run_osc_jumping_controller',
            '--delay_time=%.1f' % delay_time,
            '--channel_u=OSC_JUMPING',
            '--traj_name=%s' % traj,
            '--gains_filename=examples/Cassie/osc_jump/osc_jumping_gains_param.yaml',
          ]
          simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_w_platform',
                           '--init_height=%.1f' % 0.9,
                           '--target_realtime_rate=%.2f' % realtime_rate,
                           '--dt=%.5f' % 1e-3,
                           '--publish_rate=%d' % publish_rate,
                           '--end_time=%.3f' % sim_time,
                           '--channel_u=OSC_JUMPING',
                           '--traj_name=box_jump',
                           '--platform_height=0.5',
                           '--platform_x=0.25',
                           '--visualize=1',
                           ]
          gain_filename = 'osc_jumping_gains_box.yaml'
        elif traj == 'down_jump':
          controller_cmd = [
            'bazel-bin/examples/Cassie/run_osc_jumping_controller',
            '--delay_time=%.1f' % delay_time,
            '--channel_u=OSC_JUMPING',
            '--traj_name=%s' % traj,
            '--gains_filename=examples/Cassie/osc_jump/osc_jumping_gains_param.yaml',
          ]
          simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_w_platform',
                           '--init_height=%.1f' % 0.9,
                           '--target_realtime_rate=%.2f' % realtime_rate,
                           '--dt=%.5f' % 1e-3,
                           '--publish_rate=%d' % publish_rate,
                           '--end_time=%.3f' % sim_time,
                           '--channel_u=OSC_JUMPING',
                           '--traj_name=down_jump',
                           '--platform_height=0.5',
                           '--platform_x=-0.99',
                           '--visualize=1',
                           ]
          gain_filename = 'osc_jumping_gains_down.yaml'

        f = open(gains_path + gain_filename, 'r')
        filedata = f.read()
        f.close()
        newdata = filedata.replace('impact_threshold: %.3f' % 0.025,
                                   'impact_threshold: %.3f' % impact_threshold)
        newdata = newdata.replace('landing_delay: %.3f' % nominal_delay,
                                  'landing_delay: %.3f' % landing_time)
        f = open(gains_path + 'osc_jumping_gains_param.yaml', 'w')
        f.write(newdata)
        f.close()

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
        time.sleep(controller_startup_time)
        simulator_process = subprocess.Popen(simulator_cmd)
        time.sleep(sim_run_time)
        simulator_process.kill()
        controller_process.kill()
        logger_process.kill()

  parameter_file.close()


def construct_success_plot():
  results_folder = "/media/yangwill/backups/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/2023/param_study/"
  all_logs = sorted(glob.glob(results_folder + 'down_jump/' + 'lcmlog-*'))

  landing_times = np.arange(0.000, 0.055, 0.005)
  # nominal_delay = 0.040 # box
  # nominal_delay = 0.025 # long
  nominal_delay = 0.065 # down
  # nominal_delay = 0.000 # jump
  landing_times += nominal_delay - 0.5 * 0.05
  impact_thresholds = np.arange(0.000, 0.125, 0.025)
  success = np.zeros((landing_times.shape[0], impact_thresholds.shape[0]))
  # xx, yy = np.meshgrid(landing_times, impact_thresholds)
  for log_filename in all_logs:
    landing_time = log_filename.split('_')[-3]
    impact_threshold = log_filename.split('_')[-1]
    default_channels = {'CASSIE_STATE_SIMULATION': dairlib.lcmt_robot_output,
                        'OSC_JUMPING': dairlib.lcmt_robot_input,
                        'OSC_DEBUG_JUMPING': dairlib.lcmt_osc_output}
    callback = mbp_plots.load_default_channels
    start_time = 4
    duration = -1
    plant, context = cassie_plots.make_plant_and_context(
      floating_base=True, springs=True)
    log = lcm.EventLog(log_filename, "r")
    robot_output, robot_input, osc_debug = \
      get_log_data(log, default_channels, start_time, duration, callback,
                   plant,
                   'CASSIE_STATE_SIMULATION', 'OSC_JUMPING',
                   'OSC_DEBUG_JUMPING')
    land_idx = int(np.round((float(landing_time) - (nominal_delay - 0.025)) / 0.005))
    print(landing_time)
    print(land_idx)
    impact_idx = int(np.round(float(impact_threshold)/0.02500))
    success[land_idx, impact_idx] = not np.any(robot_output['q'][:, 6] < 0.4)
  np.save('down_jump_success', success)
  plt.imshow(success)
  plt.show()

def plot_success():
  landing_times = np.linspace(0.000, 0.050, 11)
  landing_times -= 0.025
  impact_thresholds = np.linspace(0.000, 0.100, 5)

  success = np.load('long_jump_success.npy')
  # plt.imshow(success, cmap='tab20')
  ps = plot_styler.PlotStyler()
  plot_styler.PlotStyler.set_default_styling()
  cmap = matplotlib.colors.ListedColormap([ps.grey, ps.blue])
  plt.figure()
  plt.imshow(success, cmap=cmap)
  plt.xlabel('Projection Window Duration (s)')
  plt.ylabel('Deviation from Nominal Transition Time (s)')
  ax = plt.gca()
  np.set_printoptions(precision=3)
  ax.set_xticks(np.arange(0, 5, 1))
  ax.set_yticks(np.arange(0, 11, 1))
  ax.set_xticklabels(np.around(impact_thresholds, 3).tolist())
  ax.set_yticklabels(np.around(landing_times, 3).tolist())
  ax.grid(which="minor", color='k', linestyle='-', linewidth=2)

  legend_elements = [Patch(facecolor=ps.grey, alpha=0.7, label='Fail'), Patch(facecolor=ps.blue, alpha=0.3, label='Success')]
  legend = ax.legend(handles=legend_elements, loc=1)
  plt.savefig('long_jump_success.png', dpi=240)
  plt.show()

if __name__ == "__main__":
  # main()
  # construct_success_plot()
  plot_success()
