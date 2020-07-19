import numpy as np
import pydairlib.lcm_trajectory
from pydrake.trajectories import PiecewisePolynomial

def load_lcm_trajs(nx, nu, n_modes, folder_path, trajectory_name, mode_name):
  loaded_state_traj = pydairlib.lcm_trajectory.LcmTrajectory()
  loaded_state_traj.loadFromFile(
    folder_path + trajectory_name)
  state_trajs = []
  t_nominal = []
  x_points_nominal = []
  x_hybrid_trajs_nominal = []
  u_hybrid_trajs_nominal = []
  decision_vars = loaded_state_traj.getTrajectory("cassie_jumping_decision_vars")

  for i in range(n_modes):
    state_trajs.append(loaded_state_traj.getTrajectory(
      mode_name + str(i)))
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
