import numpy as np
import pydairlib.lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial


def loadLcmTrajs(nx, nu, n_modes):
  loadedStateTraj = pydairlib.lcm_trajectory.LcmTrajectory()
  loadedStateTraj.loadFromFile(
    "/home/yangwill/Documents/research/projects/cassie/jumping"
    "/saved_trajs/May_25_two_modes")
  # loadedStateTraj.loadFromFile(
  #   "/home/yangwill/Documents/research/projects/cassie/jumping"
  #   "/saved_trajs/target_trajs/April_19_jumping_0.2")
  state_trajs = []
  t_nominal = []
  x_points_nominal = []
  x_hybrid_trajs_nominal = []
  u_hybrid_trajs_nominal = []
  decision_vars = loadedStateTraj.getTrajectory("cassie_jumping_decision_vars")

  for i in range(n_modes):
    state_trajs.append(loadedStateTraj.getTrajectory(
      "cassie_jumping_trajectory_x_u" + str(i)))
    t_nominal.append(state_trajs[i].time_vector)
    x_points_nominal.append(state_trajs[i].datapoints)
    x_hybrid_trajs_nominal.append(PiecewisePolynomial.CubicHermite(
      state_trajs[i].time_vector,
      state_trajs[i].datapoints[:nx, :],
      state_trajs[i].datapoints[nx:2 * nx, :]))
    u_hybrid_trajs_nominal.append(PiecewisePolynomial.FirstOrderHold(
      state_trajs[i].time_vector,
      state_trajs[i].datapoints[-nu:, :]))

  t_nominal = np.array(t_nominal)
  x_points_nominal = np.array(x_points_nominal)
  t_nominal = np.reshape(t_nominal, (t_nominal.shape[1] * n_modes))
  # x_points_nominal = np.reshape(x_points_nominal.transpose(2, 1, 0),
  #                               (x_points_nominal.shape[1],
  #                                x_points_nominal.shape[2] * n_modes))
  x_points_nominal = np.reshape(x_points_nominal.transpose(0, 2, 1),
                                (x_points_nominal.shape[2] * n_modes,
                                 x_points_nominal.shape[1])).T
  x_traj_nominal = PiecewisePolynomial.CubicHermite(t_nominal,
                                                    x_points_nominal[:nx, :],
                                                    x_points_nominal[nx:2 * nx,
                                                    :])
  u_traj_nominal = PiecewisePolynomial.FirstOrderHold(t_nominal,
                                                      x_points_nominal[-nu:, :])

  return x_traj_nominal, x_hybrid_trajs_nominal, u_traj_nominal, \
         u_hybrid_trajs_nominal, decision_vars, state_trajs[0].datatypes
