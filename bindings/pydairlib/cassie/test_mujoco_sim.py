from scipy import interpolate
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial
from mujoco_sim import MujocoCassieSim
import numpy as np


def test_default():
  sim = MujocoCassieSim()
  sim.run_sim(0.0, 30.0)


def test_playback():

  realtime_rate = 1e-3
  sim = MujocoCassieSim(realtime_rate=realtime_rate)
  start_time = 30.595
  end_time = start_time + 0.5
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  log_num = '33'
  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')

  lcm_traj = lcm_trajectory.LcmTrajectory(folder_path + 'u_traj_' + log_num)
  u_traj = lcm_traj.GetTrajectory("controller_inputs")
  u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)

  x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
  x_init = x_interp(start_time)
  sim.run_sim_playback(x_init, start_time, end_time, )


if __name__ == '__main__':
  # test_default()
  test_playback()
  # sim.run_sim(0.0, 30.0)
