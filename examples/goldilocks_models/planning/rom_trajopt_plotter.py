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
  global front_contact_disp, rear_contact_disp, mid_contact_disp
  global l_loop_closure, r_loop_closure
  l_toe_frame = plant_FOM.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_FOM.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  mid_contact_disp = (front_contact_disp + rear_contact_disp) / 2
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
  com_vec, comdot_vec = PlotCOM(rom_traj)
  feet_pos_vec, feet_vel_vec = PlotFeet(rom_traj)
  PlotCOMWrtStanceFoot(com_vec, comdot_vec, feet_pos_vec, feet_vel_vec)

  # import pdb; pdb.set_trace()

  if not savefig:
    plt.show()


def PlotCOMWrtStanceFoot(com_vec, comdot_vec, feet_pos_vec, feet_vel_vec,
    start_with_left_stance=True):
  com_wrt_stance_foot_vec = np.zeros(com_vec.shape)
  comdot_wrt_stance_foot_vec = np.zeros(comdot_vec.shape)

  n_mode = com_vec.shape[0]

  left_stance = start_with_left_stance
  for i in range(n_mode):
    print("start_with_left_stance" + str(left_stance))
    stance_foot_pos = feet_pos_vec[0][i] if left_stance else feet_pos_vec[1][i]
    stance_foot_vel = feet_vel_vec[0][i] if left_stance else feet_vel_vec[1][i]
    com_wrt_stance_foot_vec[i] = com_vec[i] - stance_foot_pos
    comdot_wrt_stance_foot_vec[i] = comdot_vec[i] - stance_foot_vel
    left_stance = not left_stance

  palette = ['r', 'b', 'g']
  figname = "Full model: COM wrt stance foot"
  plt.figure(figname, figsize=figsize)
  for i in range(n_mode):
    for j in range(3):
      plt.plot([i, i + 1], [com_wrt_stance_foot_vec[i][0][j],
                            com_wrt_stance_foot_vec[i][1][j]],
        palette[j], markersize=4)
  plt.ylabel('(m)')
  plt.xlabel('mode index')
  plt.legend(['x', 'y', 'z'])
  figname = "Full model: COM vel wrt stance foot"
  plt.figure(figname, figsize=figsize)
  for i in range(n_mode):
    for j in range(3):
      plt.plot([i, i + 1], [comdot_wrt_stance_foot_vec[i][0][j],
                            comdot_wrt_stance_foot_vec[i][1][j]],
        palette[j], markersize=4)
  plt.ylabel('(m/s)')
  plt.xlabel('mode index')
  plt.legend(['x', 'y', 'z'])


def PlotFeet(rom_traj):
  n_mode = rom_traj.GetNumModes()
  vars = rom_traj.GetTrajectory("decision_vars")

  feet_pos_vec = np.zeros((2, n_mode, 2, 3))
  feet_vel_vec = np.zeros((2, n_mode, 2, 3))

  toe_frames = [l_toe_frame, r_toe_frame]
  names = ["left", "right"]

  for k in range(2):
    x0_FOM = FindVariableByName(vars, 'x0_FOM', nx_FOM)
    foot_pos_relaxed, foot_vel_relaxed = CalcPointPosAndVel(x0_FOM,
      toe_frames[k],
      mid_contact_disp)
    feet_pos_vec[k][0][0] = foot_pos_relaxed
    feet_vel_vec[k][0][0] = foot_vel_relaxed

    xf_FOM = FindVariableByName(vars, 'xf_FOM')
    for i in range(n_mode):
      _, _dot = CalcPointPosAndVel(xf_FOM[i * nx_FOM: (i + 1) * nx_FOM],
        toe_frames[k], mid_contact_disp)
      feet_pos_vec[k][i][1] = _
      feet_vel_vec[k][i][1] = _dot

    vp_FOM = FindVariableByName(vars, 'vp_FOM')
    for i in range(1, n_mode):
      x0_FOM = np.hstack([xf_FOM[(i - 1) * nx_FOM: (i - 1) * nx_FOM + nq_FOM],
                          vp_FOM[(i - 1) * nv_FOM: i * nv_FOM]])
      _, _dot = CalcPointPosAndVel(x0_FOM, toe_frames[k], mid_contact_disp)
      feet_pos_vec[k][i][0] = _
      feet_vel_vec[k][i][0] = _dot

    palette = ['r', 'b', 'g']
    figname = "Full model: " + names[k] + " foot pos"
    plt.figure(figname, figsize=figsize)
    for i in range(n_mode):
      for j in range(3):
        plt.plot([i, i + 1],
          [feet_pos_vec[k][i][0][j], feet_pos_vec[k][i][1][j]],
          palette[j], markersize=4)
    plt.ylabel('(m)')
    plt.xlabel('mode index')
    plt.legend(['x', 'y', 'z'])
    figname = "Full model: " + names[k] + " foot vel"
    plt.figure(figname, figsize=figsize)
    for i in range(n_mode):
      for j in range(3):
        plt.plot([i, i + 1],
          [feet_vel_vec[k][i][0][j], feet_vel_vec[k][i][1][j]],
          palette[j], markersize=4)
    plt.ylabel('(m/s)')
    plt.xlabel('mode index')
    plt.legend(['x', 'y', 'z'])

  # import pdb; pdb.set_trace()
  return feet_pos_vec, feet_vel_vec


def PlotCOM(rom_traj):
  # We assume that the initial state constraint is relaxed, and here
  # we reconstruct the robot's init state by adding back the slack variable
  # value
  n_mode = rom_traj.GetNumModes()
  vars = rom_traj.GetTrajectory("decision_vars")

  com_vec = np.zeros((n_mode, 2, 3))
  comdot_vec = np.zeros((n_mode, 2, 3))

  x0_FOM = FindVariableByName(vars, 'x0_FOM', nx_FOM)
  com_relaxed, comdot_relaxed = CalcCenterOfMass(x0_FOM)
  com_vec[0][0] = com_relaxed
  comdot_vec[0][0] = comdot_relaxed

  # 1. If we relax the whole state
  x0_FOM = x0_FOM + FindVariableByName(vars, 'eps_x0_FOM')
  # 2. If we relax only the velocity
  # x0_FOM[nq_FOM:] = x0_FOM[nq_FOM:] + FindVariableByName(vars, 'eps_v0_FOM')
  # 3. If we relax only the floating base vel
  # x0_FOM[nq_FOM:nq_FOM + 6] = x0_FOM[nq_FOM:nq_FOM + 6] + \
  #                                 FindVariableByName(vars, 'eps_v0_FOM')
  # 4. If we relax only the translational part of floating base vel
  # x0_FOM[nq_FOM + 3:nq_FOM + 6] = x0_FOM[nq_FOM + 3:nq_FOM + 6] + \
  #                                 FindVariableByName(vars, 'eps_v0_FOM')
  # 5. If we relax only the z compoment of the floating base vel
  # x0_FOM[nq_FOM + 5:nq_FOM + 6] = x0_FOM[nq_FOM + 5:nq_FOM + 6] + \
  #                                 FindVariableByName(vars, 'eps_v0_FOM')
  # 6. If we don't relax anything
  # do nothing.

  com_non_relaxed, comdot_non_relaxed = CalcCenterOfMass(x0_FOM)

  xf_FOM = FindVariableByName(vars, 'xf_FOM')
  for i in range(n_mode):
    com, comdot = CalcCenterOfMass(xf_FOM[i * nx_FOM: (i + 1) * nx_FOM])
    com_vec[i][1] = com
    comdot_vec[i][1] = comdot

  vp_FOM = FindVariableByName(vars, 'vp_FOM')
  for i in range(1, n_mode):
    x0_FOM = np.hstack([xf_FOM[(i - 1) * nx_FOM: (i - 1) * nx_FOM + nq_FOM],
                        vp_FOM[(i - 1) * nv_FOM: i * nv_FOM]])
    com, comdot = CalcCenterOfMass(x0_FOM)
    com_vec[i][0] = com
    comdot_vec[i][0] = comdot

  palette = ['r', 'b', 'g']
  figname = "Full model: COM"
  plt.figure(figname, figsize=figsize)
  for i in range(n_mode):
    for j in range(3):
      plt.plot([i, i + 1], [com_vec[i][0][j], com_vec[i][1][j]], palette[j],
        markersize=4)
  for i in range(3):
    plt.plot([0], com_non_relaxed[i], palette[i] + 'o', markersize=4)
  plt.ylabel('(m)')
  plt.xlabel('mode index')
  plt.legend(['x', 'y', 'z'])
  figname = "Full model: COM vel"
  plt.figure(figname, figsize=figsize)
  for i in range(n_mode):
    for j in range(3):
      plt.plot([i, i + 1], [comdot_vec[i][0][j], comdot_vec[i][1][j]],
        palette[j], markersize=4)
  for i in range(3):
    plt.plot([0], comdot_non_relaxed[i], palette[i] + 'o', markersize=4)
  plt.ylabel('(m/s)')
  plt.xlabel('mode index')
  plt.legend(['x', 'y', 'z'])

  # # Only plot the init staet
  # figname = "Full model: COM before and after relaxing"
  # plt.figure(figname, figsize=figsize)
  # plt.plot([0, 1, 2], com_relaxed, 'ro', markersize=4)
  # plt.plot([0, 1, 2], com_non_relaxed, 'bo', markersize=4)
  # plt.ylabel('(m)')
  # plt.xlabel('index')
  # plt.legend(['com_relaxed', 'com'])
  # figname = "Full model: COM vel before and after relaxing"
  # plt.figure(figname, figsize=figsize)
  # plt.plot([0, 1, 2], comdot_relaxed, 'ro', markersize=4)
  # plt.plot([0, 1, 2], comdot_non_relaxed, 'bo', markersize=4)
  # plt.ylabel('(m/s)')
  # plt.xlabel('index')
  # plt.legend(['comdot_relaxed', 'comdot'])

  return com_vec, comdot_vec


def CalcPointPosAndVel(x, frame, point):
  plant_FOM.SetPositionsAndVelocities(context, x)
  pos = plant_FOM.CalcPointsPositions(context, frame, point, world)
  vel = plant_FOM.CalcJacobianTranslationalVelocity(context,
    JacobianWrtVariable.kV, frame, point, world, world) @ x[nq_FOM:]
  return pos.flatten(), vel.flatten()


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
    length = length + 1
  return var_name[:length]


def GetIndexStartGivenName(vars, name):
  index = 0
  for i in range(len(vars.datatypes)):
    if vars.datatypes[i][:len(name)] == name:
      index = i
      break
  return index


def GetVarLengthGivenName(vars, name):
  index_start = GetIndexStartGivenName(vars, name)
  length = 0
  for i in range(index_start, len(vars.datatypes)):
    if vars.datatypes[i][:len(name)] != name:
      break
    length = length + 1
  return length


def FindVariableByName(vars, name, var_length=-1):
  # returns solutions by variable names
  if var_length == -1:
    var_length = GetVarLengthGivenName(vars, name)

  start_idx = GetIndexStartGivenName(vars, name)
  if start_idx == 0:
    raise NameError(name + " doesn't exist")
  else:
    for i in range(var_length):
      if vars.datatypes[start_idx + i][:len(name)] != name:
        raise NameError(name + "'s length is shorter than " + str(i + 1))
  # We need to make a copy here. Otherwise, the returned object is not writable
  return vars.datapoints[start_idx:start_idx + var_length, 0].copy()


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
