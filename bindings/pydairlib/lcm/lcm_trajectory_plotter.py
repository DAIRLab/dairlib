import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
import numpy as np
from pydrake.trajectories import PiecewisePolynomial


def main():
  loadedTrajs = lcm_trajectory.LcmTrajectory()
  # loadedTrajs.LoadFromFile(
  #     "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d_processed")
  # loadedTrajs.LoadFromFile(
  #     "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/walking_0.16.0_processed")
  loadedTrajs.LoadFromFile(
    "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/running_0.25_processed")

  lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory0")
  lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory0")
  lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_trans_trajectory0")
  # lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_rot_trajectory0")

  # import pdb; pdb.set_trace()
  # left_foot_traj = PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[0:6],
  #                                                   lcm_left_foot_traj.datapoints[3:9])
  # right_foot_traj = PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector,
  #                                                    lcm_right_foot_traj.datapoints[0:6],
  #                                                    lcm_right_foot_traj.datapoints[3:9])
  # pelvis_traj = PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[0:6],
  #                                                lcm_pelvis_traj.datapoints[3:9])
  x_slice = slice(0,3)
  xdot_slice = slice(3,6)
  left_foot_traj = PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[x_slice],
                                                    lcm_left_foot_traj.datapoints[xdot_slice])
  right_foot_traj = PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector,
                                                     lcm_right_foot_traj.datapoints[x_slice],
                                                     lcm_right_foot_traj.datapoints[xdot_slice])
  pelvis_traj = PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[x_slice],
                                                 lcm_pelvis_traj.datapoints[xdot_slice])
  for mode in range(1, 6):
    lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory" + str(mode))
    lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory" + str(mode))
    lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_trans_trajectory" + str(mode))

    left_foot_traj.ConcatenateInTime(
      PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[x_slice],
                                       lcm_left_foot_traj.datapoints[xdot_slice]))
    right_foot_traj.ConcatenateInTime(
      PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[x_slice],
                                       lcm_right_foot_traj.datapoints[xdot_slice]))
    pelvis_traj.ConcatenateInTime(
      PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[x_slice],
                                       lcm_pelvis_traj.datapoints[xdot_slice]))

  # plt.figure('accel')
  # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,2:3])
  # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,5:6])
  # plt.plot(times, accel[:, -1])
  # plt.figure("left_foot pos")
  # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,0:3])
  # plt.legend(['x','y','z'])
  # plt.figure("left_foot vel")
  # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,3:6])
  # plt.legend(['x','y','z'])
  # plt.figure("right_foot pos")
  # plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,0:3])
  # plt.legend(['x','y','z'])
  # plt.figure("right_foot vel")
  # plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,3:6])
  # plt.legend(['x','y','z'])

  plot_trajectory(pelvis_traj, 'pelvis')
  # plot_trajectory(left_foot_traj, 'left_foot')
  # plot_trajectory(right_foot_traj, 'right_foot')
  plt.show()


def plot_trajectory(traj_of_interest, traj_name):
  times = np.arange(traj_of_interest.start_time(), traj_of_interest.end_time(), 0.001)
  # times = np.arange(traj_of_interest.start_time(), 0.2, 0.001)
  y_dim = 3
  accel = np.zeros((times.shape[0], y_dim))
  pos = np.zeros((times.shape[0], y_dim))
  vel = np.zeros((times.shape[0], y_dim))
  for i in range(times.shape[0]):
    pos[i, :] = traj_of_interest.value(times[i])[:, 0]
    vel[i, :] = traj_of_interest.EvalDerivative(times[i], 1)[:, 0]
    accel[i, :] = traj_of_interest.EvalDerivative(times[i], 2)[:, 0]

  plt.figure(traj_name + "_pos")
  plt.plot(times, pos)
  plt.legend(['x', 'y', 'z', 'xdot', 'ydot', 'zdot'])
  plt.figure(traj_name + "_vel")
  plt.plot(times, vel)
  plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
  plt.figure(traj_name + "_acc")
  plt.plot(times, accel)
  plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
  # plt.legend(['pos','vel','accel'])


if __name__ == "__main__":
  main()
