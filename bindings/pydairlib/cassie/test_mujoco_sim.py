from scipy import interpolate
from pydairlib.lcm import lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial
from mujoco_sim import MujocoCassieSim
import numpy as np
from drake_to_mujoco_converter import DrakeToMujocoConverter

def test_default():
  sim = MujocoCassieSim()
  sim.run_sim(0.0, 30.0)


def test_playback():

  realtime_rate = 0.5
  sim = MujocoCassieSim(realtime_rate=realtime_rate)
  converter = DrakeToMujocoConverter()
  start_time = 30.595
  end_time = start_time + 0.25
  folder_path = "/home/yangwill/Documents/research/projects/impact_uncertainty/data/"
  log_num = '15'
  x_traj = np.load(folder_path + 'x_' + log_num + '.npy')
  t = np.load(folder_path + 't_x_' + log_num + '.npy')

  lcm_traj = lcm_trajectory.LcmTrajectory(folder_path + 'u_traj_' + log_num)
  u_traj = lcm_traj.GetTrajectory("controller_inputs")
  u_init_traj = PiecewisePolynomial.FirstOrderHold(u_traj.time_vector, u_traj.datapoints)

  x_interp = interpolate.interp1d(t[:, 0], x_traj, axis=0, bounds_error=False)
  x_init = x_interp(start_time)

  # q = np.array([-6.982604283974403e-07, 3.7865127947946413e-10, 1.0099983047518448, 0.9999999999961983, -7.632831691221423e-10, -2.7574784242863903e-06, -2.9433630913094513e-09, 0.004499854899564281, -9.410024796999619e-07, 0.49730958420660015, 0.9785035900307172, -0.016401502318493813, 0.017853282029618316, -0.20479984213730668, -1.199696028660193, -9.15076667898222e-05, 1.4269933101105614, -0.0009611134625997009, -1.5212260231752566, 1.5190997387548775, -1.5969829475874628, -0.004499851953423773, 9.22401446113535e-07, 0.4973095670201114, 0.9786345081183413, 0.003861507918928626, -0.015225317244371206, -0.20500677547674553, -1.199696036975082, -9.140908325632153e-05, 1.4269932388103483, -0.0009610714383687213, -1.5212260470160093, 1.5190997625212834, -1.5969829648661082])
  # print(q[10:14])
  # v = np.zeros(32)
  # q_init, v_init = converter.convert_to_drake(q, v)
  # x_init = np.hstack((q_init, v_init))

  sim.run_sim_playback(x_init, start_time, end_time, u_init_traj)


if __name__ == '__main__':
  # test_default()
  test_playback()
  # sim.run_sim(0.0, 30.0)
