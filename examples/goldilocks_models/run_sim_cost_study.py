import subprocess
import time
import os
from pathlib import Path

import csv
import numpy as np
import matplotlib.pyplot as plt

def run_sim_and_controller(rom_iter_idx, get_init_file):
  # simulation arguments
  sim_end_time = 8.0;
  target_realtime_rate = 0.04;
  pause_second = 2.0 if get_init_file else 0;
  init_traj_file = '' if get_init_file else '0_rom_trajectory';

  # planner arguments
  knots_per_mode = 10;
  feas_tol = 1e-2
  n_step = 2;
  planner_init_file = '' if get_init_file else '0_z.csv'

  planner_cmd = ['bazel-bin/examples/goldilocks_models/run_cassie_rom_planner_process',
                    '--fix_duration=true',
                    '--zero_touchdown_impact=true',
                    '--use_ipopt=true',
                    '--log_solver_info=false',
                    '--iter=%d' % rom_iter_idx,
                    '--knots_per_mode=%d' % knots_per_mode,
                    '--n_step=%d' % n_step,
                    '--feas_tol=%.6f' % feas_tol,
                    '--init_file=%s' % planner_init_file,
                    '--log_data=%s' % str(get_init_file).lower(),
                    '--run_one_loop_to_get_init_file=%s' % str(get_init_file).lower(),
                    ]
  controller_cmd = ['bazel-bin/examples/goldilocks_models/run_cassie_rom_controller',
                    '--const_walking_speed=true',
                    '--use_IK=false',
                    '--iter=%d' % rom_iter_idx,
                    '--init_traj_file_name=%s' % init_traj_file,
                    ]
  simulator_cmd = ['bazel-bin/examples/Cassie/multibody_sim',
                   '--end_time=%.3f' % sim_end_time,
                   '--pause_second=%.3f' % pause_second,
                   '--init_height=%.3f' % 1.0,
                   '--target_realtime_rate=%.3f' % target_realtime_rate,
                   ]
  lcm_logger_cmd = ['lcm-logger',
                    '-f',
                    lcmlog_file_path(rom_iter_idx),
                    ]

  planner_process = subprocess.Popen(planner_cmd)
  controller_process = subprocess.Popen(controller_cmd)
  if not get_init_file:
    logger_process = subprocess.Popen(lcm_logger_cmd)
  time.sleep(3)
  simulator_process = subprocess.Popen(simulator_cmd)

  if get_init_file:
    # Wait for planner to end
    while planner_process.poll() is None:  # while subprocess is alive
      time.sleep(1)
    # Kill the rest of process
    controller_process.kill()
    simulator_process.kill()
  else:
    # Wait for simluation to end
    while simulator_process.poll() is None:  # while subprocess is alive
      time.sleep(1)
    # Kill the rest of process
    planner_process.kill()
    controller_process.kill()
    logger_process.kill()


def eval_cost(rom_iter_idx):
  planner_cmd = ['bazel-bin/examples/goldilocks_models/eval_single_sim_performance',
                    lcmlog_file_path(rom_iter_idx),
                    'CASSIE_INPUT',
                    str(rom_iter_idx),
                    ]
  planner_process = subprocess.Popen(planner_cmd)

  # Wait for evaluation to end
  while planner_process.poll() is None:  # while subprocess is alive
    time.sleep(1)

def lcmlog_file_path(rom_iter_idx):
  return directory + '/lcmlog-idx_%d' % (rom_iter_idx)


def run_sim_and_generate_cost(model_indices):
  for rom_iter_idx in model_indices:
    print("run simulation for rom_iter_idx = " + str(rom_iter_idx))

    # Get the initial traj
    run_sim_and_controller(rom_iter_idx, True)
    # Run the simulation 
    run_sim_and_controller(rom_iter_idx, False)

    # Evaluate the cost
    eval_cost(rom_iter_idx)

    # Delete the lcmlog
    os.remove(lcmlog_file_path(rom_iter_idx))

def plot_cost(model_indices):
  # Get names
  with open(directory + "/cost_names.csv", newline='') as f:
    reader = csv.reader(f)
    data = list(reader)
  names = data[0]

  # Get values
  costs = np.zeros((0, len(names))) 

  for rom_iter_idx in model_indices:
    # rom_iter_idx = 35
    with open(directory + "/" + str(rom_iter_idx) + "_cost_values.csv", newline='') as f:
      reader = csv.reader(f)
      data = [float(x) for x in list(reader)[0]]
    costs = np.vstack([costs, np.array(data)])

  # import pdb; pdb.set_trace()

  figname = "Simulation cost over model iterations"
  plt.figure(figname, figsize=(6.4, 4.8))
  plt.plot(model_indices, costs)
  plt.ylabel('cost')
  plt.xlabel('model iterations')
  plt.legend(names)
  plt.show()

if __name__ == "__main__":
  global directory
  directory = "../dairlib_data/goldilocks_models/sim_cost_eval"

  # Create folder
  Path(directory).mkdir(parents=True, exist_ok=True)

  model_iter_idx_start = 1 #1
  model_iter_idx_end = 10
  idx_spacing = 5

  model_indices = list(range(model_iter_idx_start-1, model_iter_idx_end + 1, idx_spacing))
  model_indices[0] += 1
  # example list: [1, 5, 10, 15]

  # Toggle the functions here depending on whether to generate cost or plot cost
  # run_sim_and_generate_cost(model_indices)
  plot_cost(model_indices)
