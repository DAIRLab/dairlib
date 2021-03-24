import sys
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np


def main():
  trajectory_name = sys.argv[1]
  filename = FindResourceOrThrow('examples/Cassie/saved_trajectories/' + trajectory_name)
  dircon_traj = lcm_trajectory.DirconTrajectory(filename)

  print(calc_total_cost(dircon_traj))

def calc_total_cost(dircon_traj):
  vel_cost = 1e-2 * compute_velocity_squared_cost(dircon_traj)
  u_cost = 1e-4 * compute_torque_squared_cost(dircon_traj)
  force_cost = 1e-10 * compute_force_cost(dircon_traj)
  print(vel_cost)
  return vel_cost + u_cost + force_cost


def compute_force_cost(dircon_traj):
  cost = 0
  lambda_traj = dircon_traj.ReconstructLambdaTrajectory()
  lambda_c_traj = dircon_traj.ReconstructLambdaCTrajectory()
  gamma_c_traj = dircon_traj.ReconstructGammaCTrajectory()
  for mode in range(dircon_traj.GetNumModes()):
    state_breaks = dircon_traj.GetStateBreaks(mode)
    lambda_traj_mode = lambda_traj[mode]
    lambda_c_traj_mode = lambda_c_traj[mode]
    gamma_c_traj_mode = gamma_c_traj[mode]
    for i in range(state_breaks.shape[0]):
      cost += (lambda_traj_mode.value(state_breaks[i])[:, 0] @ lambda_traj_mode.value(state_breaks[i])[:, 0].T)
      cost += (lambda_c_traj_mode.value(state_breaks[i])[:, 0] @ lambda_c_traj_mode.value(state_breaks[i])[:, 0].T)
      cost += (gamma_c_traj_mode.value(state_breaks[i])[:, 0] @ gamma_c_traj_mode.value(state_breaks[i])[:, 0].T)

  return cost

def compute_torque_squared_cost(dircon_traj):
  cost = 0
  input_samples = dircon_traj.GetInputSamples()
  input_breaks = dircon_traj.GetBreaks()
  h_vars = np.diff(input_breaks, axis=0)
  n_breaks = input_breaks.shape[0]

  cost += h_vars[0] / 2 * (input_samples[:, 0] @ input_samples[:, 0].T)
  for i in range(1, n_breaks - 1):
    cost += (h_vars[i - 1] + h_vars[i]) / 2 * (input_samples[:, i] @ input_samples[:, i].T)
  cost += h_vars[-1] / 2 * (input_samples[:, -1] @ input_samples[:, -1].T)

  return cost

def compute_velocity_squared_cost(dircon_traj):
  cost = 0
  nv = 18
  state_traj = dircon_traj.ReconstructStateTrajectory()
  for mode in range(dircon_traj.GetNumModes()):
    state_samples = dircon_traj.GetStateSamples(mode)
    state_breaks = dircon_traj.GetStateBreaks(mode)
    h_vars = np.diff(state_breaks, axis=0)
    n_breaks = state_breaks.shape[0]
    cost += 0.5 * h_vars[0] * (state_samples[-nv:, 0] @ state_samples[-nv:, 0].T)
    for i in range(1, n_breaks - 1):
      cost += 0.5 * (h_vars[i - 1] + h_vars[i]) * (state_samples[-nv:, i] @ state_samples[-nv:, i].T)
    cost += 0.5 * h_vars[-1] * (state_samples[-nv:, -1] @ state_samples[-nv:, -1].T)

  return cost

if __name__ == '__main__':
  main()
