import sys
import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from scipy.interpolate import CubicSpline

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.multibody

from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import *


def main():
  filename = ""
  # filename = FindResourceOrThrow('../dairlib_data/goldilocks_models/planning/robot_1/data/rom_trajectory')
  # abs_path = "/home/yuming/Desktop/20200926 try to impose lipm constraint/4 penalize swing toe vel x100/robot_1"
  # filename = abs_path + "/rom_trajectory"
  # filename = "../rom_trajectory"
  if len(sys.argv) == 2 and sys.argv[1] != "save":
    filename = sys.argv[1]
  else:
    filename = FindResourceOrThrow(
      '../dairlib_data/goldilocks_models/planning/robot_1/data/rom_trajectory')

  rom_traj = pydairlib.lcm_trajectory.RomPlannerTrajectory(filename)

  # For saving figures
  global savefig, figsize, save_path
  savefig = False
  figsize = (6.4, 4.8)
  if len(sys.argv) == 2:
    if sys.argv[1] == "save":
      savefig = True
      figsize = (16, 9)
  import getpass
  username = getpass.getuser()
  save_path = "/home/" + username + "/"

  # Build MBP
  # global plant, context, world, nq, nv, nx, nu
  # builder = DiagramBuilder()
  # plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  # Parser(plant).AddModelFromFile(
  #   FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"))
  # plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
  # plant.Finalize()
  #
  # # Conext and world
  # context = plant.CreateDefaultContext()
  # world = plant.world_frame()
  # global l_toe_frame, r_toe_frame
  # global front_contact_disp, rear_contact_disp
  # global l_loop_closure, r_loop_closure
  # l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  # r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  # front_contact_disp = np.array((-0.0457, 0.112, 0))
  # rear_contact_disp = np.array((0.088, 0, 0))
  # l_loop_closure = LeftLoopClosureEvaluator(plant)
  # r_loop_closure = RightLoopClosureEvaluator(plant)
  #
  # # MBP params
  # nq = plant.num_positions()
  # nv = plant.num_velocities()
  # nx = plant.num_positions() + plant.num_velocities()
  # nu = plant.num_actuators()

  """
  States, inputs trajectories
  """
  import pdb; pdb.set_trace()
  PlotState(rom_traj, 0, 6)

  if (rom_traj.GetInputSamples().shape[0] > 0):
    PlotInput(rom_traj)

  """
  Dynamic error
  """
  # PlotDynamicError(rom_traj, nq + 6, nx)

  """
  Print the value of the solutions
  """
  PrintAllDecisionVar(rom_traj)

  # print(rom_traj.GetStateSamples(1))
  # import pdb; pdb.set_trace()

  if not savefig:
    plt.show()


def PrintAllDecisionVar(rom_traj):
  for i in range(len(rom_traj.GetTrajectory("decision_vars").datatypes)):
    print(rom_traj.GetTrajectory("decision_vars").datatypes[i] + " = " + str(
      rom_traj.GetTrajectory("decision_vars").datapoints[i, 0]))


def PlotState(rom_traj, x_idx_start=0, x_idx_end=19):
  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = rom_traj.ReconstructStateTrajectory()
  state_datatypes = rom_traj.GetTrajectory("state_traj0").datatypes

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  # state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  state_samples = np.zeros((n_points, x_idx_end - x_idx_start))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[x_idx_start:x_idx_end, 0]

  # Plotting reconstructed state trajectories
  figname = "state trajectory " + str(x_idx_start) + "-" + str(x_idx_end)
  plt.figure(figname, figsize=figsize)
  plt.plot(t, state_samples)
  for i in range(rom_traj.GetNumModes()):
    t_knot = rom_traj.GetStateBreaks(i)
    x_knot = rom_traj.GetStateSamples(i)[x_idx_start:x_idx_end, :]
    plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
  plt.xlabel('time (s)')
  plt.legend(state_datatypes[x_idx_start:x_idx_end])
  if savefig:
    plt.savefig(save_path + figname + ".png")


def PlotInput(rom_traj):
  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = rom_traj.ReconstructStateTrajectory()
  input_traj = rom_traj.ReconstructInputTrajectory()
  input_datatypes = rom_traj.GetTrajectory("input_traj").datatypes

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  for i in range(n_points):
    input_samples[i] = input_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories
  figname = "input trajectory"
  plt.figure(figname, figsize=figsize)
  plt.plot(t, input_samples)
  plt.xlabel('time (s)')
  plt.legend(input_datatypes)
  if savefig:
    plt.savefig(save_path + figname + ".png")


# def PlotDynamicError(rom_traj, x_idx_start=0, x_idx_end=19):
#   # Get data at knot points
#   t_knot = rom_traj.GetStateBreaks(0)
#   x_knot = rom_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]
#
#   # Reconstructing state and input trajectory as piecewise polynomials
#   state_traj = rom_traj.ReconstructStateTrajectory()
#   state_datatypes = rom_traj.GetTrajectory("state_traj0").datatypes
#   input_traj = rom_traj.ReconstructInputTrajectory()
#   input_datatypes = rom_traj.GetTrajectory("input_traj").datatypes
#   force_traj = PiecewisePolynomial.ZeroOrderHold(rom_traj.GetForceBreaks(0),
#     rom_traj.GetForceSamples(0))
#   force_datatypes = rom_traj.GetTrajectory("force_vars0").datatypes
#
#   # Create time and state samples
#   n_points = 10000
#   t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
#   state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
#   for i in range(n_points):
#     state_samples[i] = state_traj.value(t[i])[:, 0]
#   input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
#   for i in range(n_points):
#     input_samples[i] = input_traj.value(t[i])[:, 0]
#   force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
#   for i in range(n_points):
#     force_samples[i] = force_traj.value(t[i])[:, 0]
#
#   # Sampling the spline for visualization
#   n_points = 10000
#   t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
#
#   # From time derivatives of cubic spline
#   xdot_traj = state_traj.MakeDerivative(1)
#   xdot_from_spline = np.zeros((n_points, nx))
#   for i in range(n_points):
#     xdot_from_spline[i] = xdot_traj.value(t[i])[:, 0]
#
#   # From dynamics function
#   xdot_from_func = np.zeros((n_points, nx))
#   for i in range(n_points):
#     plant.SetPositionsAndVelocities(context, state_traj.value(t[i]))
#
#     u = input_samples[i]
#     lamb = force_samples[i]
#
#     M = plant.CalcMassMatrixViaInverseDynamics(context)
#     M_inv = np.linalg.inv(M)
#     B = plant.MakeActuationMatrix()
#     g = plant.CalcGravityGeneralizedForces(context)
#     Cv = plant.CalcBiasTerm(context)
#
#     # The constraint order should be the same as trajopt's
#     J_lt = plant.CalcJacobianTranslationalVelocity(
#       context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world,
#       world)
#     J_lh = plant.CalcJacobianTranslationalVelocity(
#       context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world,
#       world)
#     J_rt = plant.CalcJacobianTranslationalVelocity(
#       context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world,
#       world)
#     J_rh = plant.CalcJacobianTranslationalVelocity(
#       context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world,
#       world)
#     J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
#     J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
#     # J = np.vstack((J_lt, J_lh, J_rt, J_rh, J_l_loop_closure, J_r_loop_closure))
#     J = np.vstack((J_lt, J_lh, J_l_loop_closure, J_r_loop_closure))
#
#     # xdot_from_func[i, :nq] = state_samples[i][nq:]
#     # import pdb; pdb.set_trace()
#     xdot_from_func[i, nq:] = M_inv @ (-Cv + g + B @ u + J.T @ lamb)
#
#   # Plotting reconstructed state trajectories
#   figname = "xdot trajectory " + str(x_idx_start) + "-" + str(x_idx_end)
#   plt.figure(figname, figsize=figsize)
#   # plt.plot(t, (xdot_from_spline)[:, x_idx_start:x_idx_end])
#   plt.plot(t, (xdot_from_func)[:, x_idx_start:x_idx_end])
#   # plt.plot(t, (xdot_from_spline - xdot_from_func)[:, x_idx_start:x_idx_end], 'ko', markersize=1)
#   # plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
#   plt.xlabel('time (s)')
#   plt.legend(state_datatypes[x_idx_start:x_idx_end])
#   if savefig:
#     plt.savefig(save_path + figname + ".png")


# PlotStateAtKnots is for testing (TODO: delete later)
def PlotStateAtKnots(rom_traj, x_idx_start=0, x_idx_end=19):
  # Get data at knot points
  t_knot = rom_traj.GetStateBreaks(0)
  x_knot = rom_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]

  state_datatypes = rom_traj.GetTrajectory("state_traj0").datatypes

  # Plotting reconstructed state trajectories
  figname = "state trajectory " + str(x_idx_start) + "-" + str(
    x_idx_end) + "(at knots)"
  plt.figure(figname, figsize=figsize)
  plt.plot(t_knot, x_knot.T)
  plt.xlabel('time (s)')
  plt.legend(state_datatypes[x_idx_start:x_idx_end])


if __name__ == "__main__":
  main()
