import sys
import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np


def main():
  # Default filename for the example
  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.2h_0.25d")
  if len(sys.argv) == 2:
    filename = sys.argv[1]
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  input_traj = dircon_traj.ReconstructInputTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes

  force_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0), dircon_traj.GetForceSamples(0))
  force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes

  collocation_force_points = dircon_traj.GetCollocationForceSamples(0)

  n_points = 500
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[:, 0]
    input_samples[i] = input_traj.value(t[i])[:, 0]
    force_samples[i] = force_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories
  plt.figure("state trajectory")
  plt.plot(t, state_samples)
  plt.legend(state_datatypes)

  plt.figure("input trajectory")
  plt.plot(t, input_samples)
  plt.legend(input_datatypes)

  plt.figure("force trajectory")
  plt.plot(t, force_samples)
  plt.legend(force_datatypes)

  plt.show()


if __name__ == "__main__":
  main()
