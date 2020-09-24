import sys
import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

def main():
  # Default filename for the example
  filename = FindResourceOrThrow('../dairlib_data/goldilocks_models/find_models/robot_1/dircon_trajectory1')
  if len(sys.argv) == 2:
    filename = sys.argv[1]
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)

  # Indexing
  x_idx_start = 7
  x_idx_end = 19

  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  input_traj = dircon_traj.ReconstructInputTrajectory()
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes
  force_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0), dircon_traj.GetForceSamples(0))
  force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes
  force_c_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetCollocationForceBreaks(0), dircon_traj.GetCollocationForceSamples(0))
  force_c_datatypes = dircon_traj.GetTrajectory("collocation_force_vars0").datatypes

  # import pdb; pdb.set_trace()

  # Sampling the spline for visualization
  n_points = 500
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  # state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  state_samples = np.zeros((n_points, x_idx_end - x_idx_start))
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
  force_c_samples = np.zeros((n_points, force_c_traj.value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[x_idx_start:x_idx_end, 0]
    input_samples[i] = input_traj.value(t[i])[:, 0]
    force_samples[i] = force_traj.value(t[i])[:, 0]
    force_c_samples[i] = force_c_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories
  plt.figure("state trajectory")
  plt.plot(t, state_samples)
  plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
  plt.legend(state_datatypes[x_idx_start:x_idx_end])

  plt.figure("input trajectory")
  plt.plot(t, input_samples)
  plt.legend(input_datatypes)

  plt.figure("force trajectory")
  plt.plot(t, force_samples)
  plt.legend(force_datatypes)

  plt.figure("collocation force trajectory")
  plt.plot(t, force_c_samples)
  plt.legend(force_c_datatypes)

  plt.show()


if __name__ == "__main__":
  main()