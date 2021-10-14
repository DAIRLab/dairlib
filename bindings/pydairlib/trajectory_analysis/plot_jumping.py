import sys
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from pydairlib.common import plot_styler



def main():
  global ps

  figure_directory = '/home/yangwill/Documents/research/projects/'
  ps = plot_styler.PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  trajectory_name = sys.argv[1]
  dircon_traj_filename = FindResourceOrThrow('examples/Cassie/saved_trajectories/' + trajectory_name)
  outputs_traj_filename = FindResourceOrThrow('examples/Cassie/saved_trajectories/' + trajectory_name + "_processed")
  dircon_traj = lcm_trajectory.DirconTrajectory(dircon_traj_filename)
  output_trajs = lcm_trajectory.LcmTrajectory(outputs_traj_filename)

  l_foot_traj, r_foot_traj, l_hip_traj, r_hip_traj, l_toe_traj, r_toe_traj, pelvis_traj = construct_all_trajs(output_trajs)

  l_foot_pos, r_foot_pos = plot_foot_trajs(l_foot_traj, r_foot_traj)
  plot_pelvis_traj(pelvis_traj, l_foot_pos, r_foot_pos)

def construct_all_trajs(output_trajs):
  l_foot_traj = PiecewisePolynomial()
  r_foot_traj = PiecewisePolynomial()
  pelvis_traj = PiecewisePolynomial()
  l_hip_traj = PiecewisePolynomial()
  r_hip_traj = PiecewisePolynomial()
  l_toe_traj = PiecewisePolynomial()
  r_toe_traj = PiecewisePolynomial()

  for mode in range(3):
    lcm_l_foot_traj = output_trajs.GetTrajectory('left_foot_trajectory' + str(mode))
    lcm_r_foot_traj = output_trajs.GetTrajectory('right_foot_trajectory' + str(mode))
    lcm_l_hip_traj = output_trajs.GetTrajectory('left_hip_trajectory' + str(mode))
    lcm_r_hip_traj = output_trajs.GetTrajectory('right_hip_trajectory' + str(mode))
    lcm_l_toe_traj = output_trajs.GetTrajectory('left_toe_trajectory' + str(mode))
    lcm_r_toe_traj = output_trajs.GetTrajectory('right_toe_trajectory' + str(mode))
    lcm_pelvis_traj = output_trajs.GetTrajectory('pelvis_trans_trajectory' + str(mode))
    l_foot_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_l_foot_traj.time_vector, lcm_l_foot_traj.datapoints[0:3, :], lcm_l_foot_traj.datapoints[3:6, :]))
    r_foot_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_r_foot_traj.time_vector, lcm_r_foot_traj.datapoints[0:3, :], lcm_r_foot_traj.datapoints[3:6, :]))
    l_hip_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_l_hip_traj.time_vector, lcm_l_hip_traj.datapoints[0:3, :], lcm_l_hip_traj.datapoints[3:6, :]))
    r_hip_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_r_hip_traj.time_vector, lcm_r_hip_traj.datapoints[0:3, :], lcm_r_hip_traj.datapoints[3:6, :]))
    l_toe_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_l_toe_traj.time_vector, lcm_l_toe_traj.datapoints[0:1, :], lcm_l_toe_traj.datapoints[1:2, :]))
    r_toe_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_r_toe_traj.time_vector, lcm_r_toe_traj.datapoints[0:1, :], lcm_r_toe_traj.datapoints[1:2, :]))
    pelvis_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[0:3, :], lcm_pelvis_traj.datapoints[3:6, :]))
  return l_foot_traj, r_foot_traj, l_hip_traj, r_hip_traj, l_toe_traj, r_toe_traj, pelvis_traj

def plot_foot_trajs(l_foot_traj, r_foot_traj):

  t_samples = np.arange(l_foot_traj.start_time(), l_foot_traj.end_time(), 1e-2)
  l_foot_pos = np.zeros((t_samples.shape[0], 3))
  l_foot_vel = np.zeros((t_samples.shape[0], 3))
  l_foot_acc = np.zeros((t_samples.shape[0], 3))
  r_foot_pos = np.zeros((t_samples.shape[0], 3))
  r_foot_vel = np.zeros((t_samples.shape[0], 3))
  r_foot_acc = np.zeros((t_samples.shape[0], 3))
  for i in range(t_samples.shape[0]):
    l_foot_pos[i] = l_foot_traj.value(t_samples[i])[:, 0]
    l_foot_vel[i] = l_foot_traj.EvalDerivative(t_samples[i], 1)[:, 0]
    l_foot_acc[i] = l_foot_traj.EvalDerivative(t_samples[i], 2)[:, 0]
    r_foot_pos[i] = r_foot_traj.value(t_samples[i])[:, 0]
    r_foot_vel[i] = r_foot_traj.EvalDerivative(t_samples[i], 1)[:, 0]
    r_foot_acc[i] = r_foot_traj.EvalDerivative(t_samples[i], 2)[:, 0]
  return l_foot_pos, r_foot_pos

def plot_pelvis_traj(pelvis_traj, l_foot_pos, r_foot_pos):
  t_samples = np.arange(pelvis_traj.start_time(), pelvis_traj.end_time(), 1e-2)
  pelvis_pos = np.zeros((t_samples.shape[0], 3))
  pelvis_vel = np.zeros((t_samples.shape[0], 3))
  pelvis_acc = np.zeros((t_samples.shape[0], 3))
  for i in range(t_samples.shape[0]):
    pelvis_pos[i] = pelvis_traj.value(t_samples[i])[:, 0]
    pelvis_vel[i] = pelvis_traj.EvalDerivative(t_samples[i], 1)[:, 0]
    pelvis_acc[i] = pelvis_traj.EvalDerivative(t_samples[i], 2)[:, 0]
  plt.figure('pos')
  ps.plot(t_samples, pelvis_pos)
  plt.figure('vel')
  ps.plot(t_samples, pelvis_vel)
  plt.figure('acc')
  ps.plot(t_samples, pelvis_acc)
  ps.show_fig()
  return


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
