import subprocess
import time
import fileinput

import numpy as np


def main():
  trajectory_path = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/"
  controller_type = "jumping"
  parameter = "pelvis_vel"
  simulator = 'DRAKE'

  gains_path = ''
  trajectory_name = ''
  results_folder = ''
  delay_time = 0.0
  sim_time = 0
  start_time = 0
  if controller_type == 'jumping':
    gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc_jump/"
    trajectory_name = "jumping_0.15h_0.3d"
    # results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/impact_invariance_param_study/"
    results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/0.100/"
    # results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/id_controller/0.000/"
    # results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/mujoco/stiff/"
    sim_time = delay_time + 3.0
    start_time = 0.0
  elif controller_type == 'walking':
    gains_path = "/home/yangwill/workspace/dairlib/examples/Cassie/osc/"
    trajectory_name = "walking_0.16.0"
    # results_folder = "/home/yangwill/Documents/research/projects/cassie/sim/walking/logs/impact_invariance_param_study/"
    sim_time = 0.8
    start_time = 0.0

  traj_name_controller = trajectory_name + "_processed"

  terrain_heights = np.arange(0.000, 0.030, 0.005)
  disturbances = np.arange(-0.500, 0.600, 0.100)
  penetration_allowances = np.array([1e-5, 1e-4, 1e-3])
  threshold_durations = np.arange(-0.01, 0.11, 0.01)

  # For MUJOCO
  if (simulator == 'MUJOCO'):
    penetration_allowances = np.array([1e-5])
  if parameter == "pelvis_vel":
    penetration_allowances = np.array([1e-5])
    start_time = 0.457
    sim_time = delay_time + 2.0

  realtime_rate = 0.5
  # publish_rate = 2000.0
  publish_rate = 4000.0

  # Add an extra second to the runtime of the simulator process to account for start up and stopping time
  sim_run_time = sim_time / realtime_rate + 1.0
  controller_startup_time = 1.0
  parameter_file = open(results_folder + 'command_list.txt', 'w')
  controller_cmd = ''
  simulator_cmd = ''
  lcm_logger_cmd = ''
  save_gains_cmd = ''

  # for i in range(1, threshold_durations.shape[0]):
  for i in range(0, disturbances.shape[0]):
  # for i in range(0, terrain_heights.shape[0]):
    for k in range(penetration_allowances.shape[0]):
      log_suffix = ''
      if parameter == 'terrain_height':
        log_suffix = 'height_%.4f-stiff_%.5f' % (terrain_heights[i], penetration_allowances[k])
      if simulator == 'MUJOCO':
        log_suffix = 'duration_%.3f_mujoco' % threshold_durations[i]
      if parameter == "pelvis_vel":
        log_suffix = 'pelvis_zvel_%.3f' % disturbances[i]
      log_path = results_folder + 'lcmlog-' + log_suffix
      print(log_path)
      if controller_type == 'walking':
        controller_cmd = ['bazel-bin/examples/Cassie/run_osc_walking_controller_tracking',
                          '--traj_name=%s' % trajectory_name,
                          ]
        simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_init',
                         '--folder_path=%s' % trajectory_path,
                         '--traj_name=%s' % trajectory_name,
                         '--publish_rate=%d' % publish_rate,
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
      elif controller_type == 'jumping':
        controller_cmd = ['bazel-bin/examples/Cassie/run_osc_jumping_controller',
                          '--traj_name=%s' % trajectory_name,
                          '--channel_u=CASSIE_INPUT',
                          '--delay_time=%.1f' % delay_time,
                          '--contact_based_fsm=0',
                          ]
        # controller_cmd = ['bazel-bin/examples/Cassie/run_id_jumping_controller',
        #                   '--traj_name=%s' % trajectory_name,
        #                   '--channel_u=CASSIE_INPUT',
        #                   '--delay_time=%.1f' % delay_time,
        #                   '--contact_based_fsm=0',
        #                   ]
        if simulator == 'DRAKE':
          if parameter == 'terrain_height':
            simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_init',
                             '--folder_path=%s' % trajectory_path,
                             '--traj_name=%s' % trajectory_name,
                             '--publish_rate=%d' % publish_rate,
                             '--end_time=%.3f' % sim_time,
                             '--dt=%.5f' % 8e-5,
                             '--terrain_height=%.4f' % terrain_heights[i],
                             '--penetration_allowance=%.5f' %
                             penetration_allowances[k],
                             '--target_realtime_rate=%.2f' % realtime_rate,
                             '--start_time=%.3f' % start_time,
                             ]
          if parameter == 'pelvis_vel':
            simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim_init',
                             '--folder_path=%s' % trajectory_path,
                             '--traj_name=%s' % trajectory_name,
                             '--publish_rate=%d' % publish_rate,
                             '--end_time=%.3f' % sim_time,
                             '--dt=%.5f' % 8e-5,
                             '--disturbance=%.3f' % disturbances[i],
                             '--penetration_allowance=%.5f' %
                             penetration_allowances[k],
                             '--target_realtime_rate=%.2f' % realtime_rate,
                             '--start_time=%.3f' % start_time,
                             ]
        elif simulator == 'MUJOCO':
          simulator_cmd = ['/home/yangwill/workspace/cassie-mujoco-sim/test/cassiesim',
                           '-r',
                           '-s',
                           ]
          f = open(gains_path + 'osc_jumping_gains.yaml', 'r')
          filedata = f.read()
          f.close()

          newdata = filedata.replace('impact_threshold: %.3f' % threshold_durations[i - 1],
                                     'impact_threshold: %.3f' % threshold_durations[i])
          f = open(gains_path + 'osc_jumping_gains.yaml', 'w')
          f.write(newdata)
          f.close()

        save_gains_cmd = ['cp',
                          '%s' % gains_path + 'osc_jumping_gains.yaml',
                          '%s' % results_folder + 'osc_jumping_gains' + log_suffix + '.yaml']
        # save_gains_cmd = ['cp',
        #                   '%s' % gains_path + 'id_jumping_gains.yaml',
        #                   '%s' % results_folder + 'id_jumping_gains' + log_suffix + '.yaml']
        lcm_logger_cmd = ['lcm-logger',
                          '-f',
                          '%s' % log_path,
                          ]

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
