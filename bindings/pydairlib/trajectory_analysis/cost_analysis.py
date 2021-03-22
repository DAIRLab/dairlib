import sys
import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np


def main():
  trajectory_name = sys.argv[1]
  filename = FindResourceOrThrow('examples/Cassie/saved_trajectories/' + trajectory_name)
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)

  # vel_cost = 1e-2 * compute_velocity_squared_cost(dircon_traj)
  vel_cost = 0
  u_cost = 1e-4 * compute_torque_squared_cost(dircon_traj)
  # u_cost = 0

  print(vel_cost + u_cost)

  import pdb; pdb.set_trace()

def compute_torque_squared_cost(dircon_traj):
  cost = 0
  input_samples = dircon_traj.GetInputSamples()
  input_breaks = dircon_traj.GetBreaks()
  h_vars = np.diff(input_breaks, axis=0)
  n_breaks = input_breaks.shape[0]
  print(h_vars)
  j = 0
  cost += h_vars[0] / 2 * (input_samples[:, 0] @ input_samples[:, 0].T)
  j += 1
  for i in range(1, n_breaks - 1):
    cost += (h_vars[i - 1] + h_vars[i]) / 2 * (input_samples[:, i] @ input_samples[:, i].T)
    j += 1
  cost += h_vars[-1] / 2 * (input_samples[:, -1] @ input_samples[:, -1].T)
  j += 1

  print(j)

  return cost

def compute_velocity_squared_cost(dircon_traj):
  cost = 0
  nv = 18
  j = 0
  state_traj = dircon_traj.ReconstructStateTrajectory()
  for mode in range(dircon_traj.GetNumModes()):
    state_samples = dircon_traj.GetStateSamples(mode)
    state_breaks = dircon_traj.GetStateBreaks(mode)
    h_vars = np.diff(state_breaks, axis=0)
    n_breaks = state_breaks.shape[0]
    cost += 0.5 * h_vars[0] * (state_samples[-nv:, 0] @ state_samples[-nv:, 0].T)
    print(h_vars[0])
    j += 1
    for i in range(1, n_breaks - 1):
      cost += 0.5 * (h_vars[i - 1] + h_vars[i]) * (state_samples[-nv:, i] @ state_samples[-nv:, i].T)
      print(h_vars[i])
      j += 1
    print(h_vars[-1])
    cost += 0.5 * h_vars[-1] * (state_samples[-nv:, -1] @ state_samples[-nv:, -1].T)
    j += 1
  print(j)


  return cost

if __name__ == '__main__':
  main()
