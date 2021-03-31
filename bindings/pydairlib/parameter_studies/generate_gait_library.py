import subprocess
import time
import fileinput
from pydairlib.lcm import lcm_trajectory
from pydairlib.trajectory_analysis import cost_analysis
from pydairlib.common import FindResourceOrThrow

import numpy as np



def calc_cost(filename):
  filepath = FindResourceOrThrow('examples/Cassie/saved_trajectories/' + filename)
  dircon_traj = lcm_trajectory.DirconTrajectory(filepath)

  return cost_analysis.calc_total_cost(dircon_traj)[0]

def main():
  trajectory_path = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/"
  gait_type = "running"

  stride_lengths = np.arange(0.00, 0.51, 0.05)
  costs = np.inf * np.ones(stride_lengths.shape)
  costs[0] = 2

  for i, stride_length in enumerate(stride_lengths):
    # Initial seed
    if i == 0:
      continue
    load_stride_length = stride_lengths[i-1]
    save_filename = "running_%0.2f" % stride_length
    load_filename = "running_%0.2f" % load_stride_length

    trajopt_cmd = ['bazel-bin/examples/Cassie/run_dircon_running',
                   '--save_filename=%s' % save_filename,
                   '--load_filename=%s' % load_filename,
                   '--same_knotpoints=1',
                   '--stride_length=%.2f' % stride_length,
                   ]

    print(trajopt_cmd)
    trajopt_process = subprocess.run(trajopt_cmd)
    costs[i] = np.min([calc_cost(save_filename), costs[i]])





if __name__ == '__main__':
    main()

