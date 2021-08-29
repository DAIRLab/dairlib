from scipy import interpolate
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial
from mujoco_sim import MujocoCassieSim
import numpy as np
from drake_to_mujoco_converter import DrakeToMujocoConverter


def test_default():
  sim = MujocoCassieSim(realtime_rate=0.1)
  sim.run_sim(0.0, 30.0)


def test_initial_state():
  sim = MujocoCassieSim(realtime_rate=1e-1)
  # q_init = np.array([1, 0, 0, 0, 0, 0, 10.0,
  #                    0.0216644, -0.0216644,
  #                    0, 0,
  #                    0.62496, 0.62496,
  #                    -1.652, -1.652,
  #                    -0.00, -0.00,
  #                    1.77012, 1.77013,
  #                    -0.0, -1.7677,
  #                    -0.0, -1.7677])
  q_init = np.array([1, 0, 0, 0, 0, 0, 10.0,
                     0.0045, -0.0045,
                     0, 0,
                     0.4973, 0.4973,
                     -1.1997, -1.1997,
                     -0.00, -0.00,
                     1.4267, 1.4267,
                     -0.0, -1.5244,
                     -0.0, -1.5244])
  # 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
  # -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
  # -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
  # -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968
  x_init = np.hstack((q_init, np.zeros(22)))
  sim.run_sim(0.0, 30.0, x_init=x_init)


def test_playback():
  realtime_rate = 0.01
  sim = MujocoCassieSim(realtime_rate=realtime_rate)
  # converter = DrakeToMujocoConverter()
  start_time = 30.495
  end_time = start_time + 0.5
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  log_num = '02'
  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')

  lcm_traj = lcm_trajectory.LcmTrajectory(folder_path + 'u_traj_' + log_num)
  u_traj = lcm_traj.GetTrajectory("controller_inputs")
  u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)

  x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=True)
  x_init = x_interp(start_time)
  # import pdb; pdb.set_trace()

  # sim.run_sim_playback(x_init, start_time, end_time)
  # sim.run_sim_playback(x_init, start_time, end_time, u_init_traj)
  # sim.visualize_IK_lower(x_init)
  sim.visualize_IK_upper(x_init)


if __name__ == '__main__':
  # test_default()
  # test_playback()
  test_initial_state()
