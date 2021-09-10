from scipy import interpolate
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial
from mujoco_sim import MujocoCassieSim
import numpy as np
from drake_to_mujoco_converter import DrakeToMujocoConverter

q_mujoco_default = np.array([1, 0, 0, 0, 0, 0, 1.01,
                                      0.0045, -0.0045,
                                      0, 0,
                                      0.4973, 0.4973,
                                      -1.1997, -1.1997,
                                      -0.00, -0.00,
                                      1.4267, 1.4267,
                                      -0.0, -1.5968,
                                      -0.0, -1.5968])


# q_mujoco_default = np.array([0.0, 0.0, 1.01,
#                              1.0, 0.0, 0.0, 0.0,
#                              0.0045, 0.0, 0.4973,
#                              0.9784830934748516, -0.016399716640763992, 0.017869691242100763, -0.2048964597373501,
#                              -1.1997, 0.0, 1.4267, 0.0, -1.5244, 1.5244, -1.5968,
#
#                              -0.0045, 0.0, 0.4973, 0.978614127766972, 0.0038600557257107214, -0.01524022001550036, -0.20510296096975877, -1.1997, 0.0, 1.4267, 0.0, -1.5244, 1.5244, -1.5968])
x_mujoco_default = np.hstack((q_mujoco_default, np.zeros(22)))

def test_default():
  sim = MujocoCassieSim(realtime_rate=0.1)
  sim.run_sim(0.0, 0.01, x_init=x_mujoco_default)
  # sim.run_sim(0.0, 0.01)


def test_initial_state():
  sim = MujocoCassieSim(realtime_rate=1e-3)
  q_init = np.array([1, 0, 0, 0, 0, 0, 1.0,
                     0.0216644, -0.0216644,
                     0, 0,
                     0.62496, 0.62496,
                     -1.652, -1.652,
                     -0.00, -0.00,
                     1.77012, 1.77013,
                     -0.0, -1.7677,
                     -0.0, -1.7677])
  # q_init = np.array([1, 0, 0, 0, 0, 0, 1.0,
  #                    0.0045, -0.0045,
  #                    0, 0,
  #                    0.4973, 0.4973,
  #                    -1.1997, -1.1997,
  #                    -0.00, -0.00,
  #                    1.4267, 1.4267,
  #                    -0.0, -1.5244,
  #                    -0.0, -1.5244])
  x_init = np.hstack((q_init, np.zeros(22)))
  # 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
  # -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
  # -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
  # -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968
  sim.run_sim(0.0, 30.0, x_init=x_init)


def test_playback():
  realtime_rate = 0.1
  sim = MujocoCassieSim(realtime_rate=realtime_rate)
  # converter = DrakeToMujocoConverter()
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  # log_num = '15'
  log_num = '11'
  start_time = 30.62
  end_time = start_time + 0.05
  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')

  lcm_traj = lcm_trajectory.LcmTrajectory(folder_path + 'u_traj_' + log_num)
  u_traj = lcm_traj.GetTrajectory("controller_inputs")
  u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)

  x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=True)
  x_init = x_interp(start_time)
  # import pdb; pdb.set_trace()

  # sim.run_sim_playback(x_init, start_time, end_time)
  sim.reinit_env(sim.default_model_directory + 'cassie_new_params.xml')
  sim.run_sim_playback(x_init, start_time, end_time, u_init_traj)
  # sim.visualize_IK_lower(x_init)
  # sim.visualize_IK_upper(x_init)

def visualize_loop_closures():
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  log_num = '28'
  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')
  sim = MujocoCassieSim(realtime_rate=1.0)
  time = 30.595
  x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=True)
  x_init = x_interp(time)

  # sim.visualize_IK_lower(x_init)
  # sim.visualize_IK_upper(x_init)
  # sim.visualize_IK_upper(x_mujoco_default)

if __name__ == '__main__':
  # test_default()
  test_playback()
  # visualize_loop_closures()
  # test_initial_state()
