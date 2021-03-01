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

  """
  Construct full order model and related variables 
  """
  # Build MBP
  global plant_FOM, context, world, nq_FOM, nv_FOM, nx_FOM, nu_FOM
  builder = DiagramBuilder()
  plant_FOM, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant_FOM).AddModelFromFile(
    FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"))
  plant_FOM.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant_FOM.Finalize()
  # Conext and world
  context = plant_FOM.CreateDefaultContext()
  world = plant_FOM.world_frame()
  global l_toe_frame, r_toe_frame
  global front_contact_disp, rear_contact_disp
  global l_loop_closure, r_loop_closure
  l_toe_frame = plant_FOM.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_FOM.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  l_loop_closure = LeftLoopClosureEvaluator(plant_FOM)
  r_loop_closure = RightLoopClosureEvaluator(plant_FOM)
  # MBP params
  nq_FOM = plant_FOM.num_positions()
  nv_FOM = plant_FOM.num_velocities()
  nx_FOM = plant_FOM.num_positions() + plant_FOM.num_velocities()
  nu_FOM = plant_FOM.num_actuators()

  """
  States, inputs trajectories
  """
  PlotState(rom_traj, 0, rom_traj.GetStateSamples(0).shape[0])

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

  """
  Testing
  """
  PlotCOM(rom_traj)

  # import pdb; pdb.set_trace()

  if not savefig:
    plt.show()


def PlotCOM(rom_traj):
  # We assume that the initial state constraint is relaxed, and here
  # we reconstruct the robot's init state by adding back the slack variable
  # value
  init_state = FindVariableByName(rom_traj, 'x0_FOM', nx_FOM)
  com_relaxed, comdot_relaxed = CalcCenterOfMass(init_state)

  # 1. If we relax the whole state
  # init_state = init_state + FindVariableByName(rom_traj, 'eps_x0_FOM', nx_FOM)
  # 2. If we relax only the velocity
  # init_state[nq_FOM:] = init_state[nq_FOM:] + FindVariableByName(rom_traj,
  #   'eps_v0_FOM', nv_FOM)
  # 3. If we relax only the floating base vel
  # init_state[nq_FOM:nq_FOM + 6] = init_state[nq_FOM:nq_FOM + 6] + \
  #                                 FindVariableByName(rom_traj, 'eps_v0_FOM',6)
  # 4. If we relax only the translational part of floating base vel
  # init_state[nq_FOM + 3:nq_FOM + 6] = init_state[nq_FOM + 3:nq_FOM + 6] + \
  #                                     FindVariableByName(rom_traj, 'eps_v0_FOM',
  #                                       3)
  # 5. If we relax only the z compoment of the floating base vel
  init_state[nq_FOM + 5:nq_FOM + 6] = init_state[nq_FOM + 5:nq_FOM + 6] + \
                                      FindVariableByName(rom_traj, 'eps_v0_FOM',
                                        1)
  # 6. If we don't relax anything
  # do nothing.

  com, comdot = CalcCenterOfMass(init_state)
  figname = "COM before and after relaxing"
  plt.figure(figname, figsize=figsize)
  plt.plot([0, 1, 2], com_relaxed, 'ro', markersize=4)
  plt.plot([0, 1, 2], com, 'bo', markersize=4)
  plt.ylabel('(m)')
  plt.xlabel('index')
  plt.legend(['com_relaxed', 'com'])
  figname = "COM vel before and after relaxing"
  plt.figure(figname, figsize=figsize)
  plt.plot([0, 1, 2], comdot_relaxed, 'ro', markersize=4)
  plt.plot([0, 1, 2], comdot, 'bo', markersize=4)
  plt.ylabel('(m/s)')
  plt.xlabel('index')
  plt.legend(['comdot_relaxed', 'comdot'])


def CalcCenterOfMass(x):
  plant_FOM.SetPositionsAndVelocities(context, x)
  com = plant_FOM.CalcCenterOfMassPosition(context)
  J = plant_FOM.CalcJacobianCenterOfMassTranslationalVelocity(context,
    JacobianWrtVariable.kV, world, world)
  comdot = J @ x[nq_FOM:]
  return com, comdot

def GetVarNameWithoutParanthesisIndex(var_name):
  length = 0
  for i in len(var_name):
    if var_name[i] == '(':
      break
    length=length+1
  return var_name[:length]


def FindVariableByName(rom_traj, name, var_length):
  # returns solutions by variable names
  index = 0
  vars = rom_traj.GetTrajectory("decision_vars")
  for i in range(len(vars.datatypes)):
    if vars.datatypes[i][:len(name)] == name:
      index = i
      break
  if index == 0:
    raise NameError(name + " doesn't exist")
  else:
    for i in range(var_length):
      if vars.datatypes[index + i][:len(name)] != name:
        raise NameError(name + "'s length is shorter than " + str(i + 1))

  # We need to make a copy here. Otherwise, the returned object is not writable
  return vars.datapoints[index:index + var_length, 0].copy()


def PrintAllDecisionVar(rom_traj):
  for i in range(len(rom_traj.GetTrajectory("decision_vars").datatypes)):
    print(rom_traj.GetTrajectory("decision_vars").datatypes[i] + " = " + str(
      rom_traj.GetTrajectory("decision_vars").datapoints[i, 0]))


def PlotState(rom_traj, x_idx_start=0, x_idx_end=6):
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


if __name__ == "__main__":
  main()
