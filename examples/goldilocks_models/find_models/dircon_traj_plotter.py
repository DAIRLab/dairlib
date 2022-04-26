import sys
import matplotlib.pyplot as plt
import pydairlib.lcm.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from scipy.interpolate import CubicSpline

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.multibody
import pydrake.common as mut

from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import *


def main():
  global filename
  filename = ""
  filepath = ""
  # filepath = FindResourceOrThrow('../dairlib_data/goldilocks_models/find_models/robot_1/dircon_trajectory_iter0')
  if len(sys.argv) >= 2 and sys.argv[1] != "save":
    arg = sys.argv[1]
    # When the argument includes the directory
    if (arg[0] == "/") or (arg[:2] == ".."):
      filepath = arg
      for i in range(len(arg))[::-1]:
        if arg[i] == "/":
          filename = arg[i+1:]
          break
    # When the argument is just the file name
    else:
      filepath = '../dairlib_data/goldilocks_models/find_models/robot_1/' + arg
      filename = arg
  else:
    filepath = FindResourceOrThrow(
      '../dairlib_data/goldilocks_models/find_models/robot_1/1_0_dircon_trajectory')

  dircon_traj = pydairlib.lcm.lcm_trajectory.DirconTrajectory(filepath)

  # For saving figures
  global savefig, figsize, save_path
  savefig = False
  figsize = (6.4, 4.8)
  for i in range(len(sys.argv)):
    if sys.argv[i] == "save":
      savefig = True
      figsize = (16, 9)
  import getpass
  username = getpass.getuser()
  save_path = "/home/" + username + "/"

  print("filename = " + filename)
  print("filepath = " + filepath)
  print("savefig = " + str(savefig))

  # Build MBP
  global plant, context, world, nq, nv, nx, nu
  mut.set_log_level("err")  # ignore warnings about joint limits
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"))
  plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # Conext and world
  context = plant.CreateDefaultContext()
  world = plant.world_frame()
  global l_toe_frame, r_toe_frame
  global front_contact_disp, rear_contact_disp
  global l_loop_closure, r_loop_closure
  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  l_loop_closure = LeftLoopClosureEvaluator(plant)
  r_loop_closure = RightLoopClosureEvaluator(plant)

  # MBP params
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  """
  States, inputs, forces trajectories
  """
  # Indexing
  PlotState(dircon_traj, 0, 7)
  PlotState(dircon_traj, 7, nq)
  PlotState(dircon_traj, nq, nq + 6)
  PlotState(dircon_traj, nq + 6, nq + 6 + 8)
  PlotState(dircon_traj, nq + 6 + 8, nx)
  PlotState(dircon_traj, nq, nq + 6, True)
  PlotState(dircon_traj, nq + 6, nq + 6 + 8, True)
  PlotState(dircon_traj, nq + 6 + 8, nx, True)

  PlotInput(dircon_traj)
  PlotForce(dircon_traj)

  """
  Center of mass trajectories
  """
  # PlotCenterOfMass(dircon_traj, True)
  PlotCenterOfMass(dircon_traj)

  """
  Pevlis trajectories
  """
  # PlotPelvis(dircon_traj)

  """
  Dynamic error
  """
  # PlotDynamicError(dircon_traj, nq + 6, nx)

  """
  Print impulse
  """
  PrintDecisionVar(dircon_traj, "impulse")

  """
  Print the value of the solutions
  """
  # PrintAllDecisionVar(dircon_traj)

  # print(dircon_traj.GetStateSamples(1))
  # import pdb; pdb.set_trace()

  if not savefig:
    plt.show()


def PrintAllDecisionVar(dircon_traj):
  for i in range(len(dircon_traj.GetTrajectory("decision_vars").datatypes)):
    print(dircon_traj.GetTrajectory("decision_vars").datatypes[i] + " = " + str(
      dircon_traj.GetTrajectory("decision_vars").datapoints[i, 0]))


def PrintDecisionVar(dircon_traj, name):
  for i in range(len(dircon_traj.GetTrajectory("decision_vars").datatypes)):
    var_name = dircon_traj.GetTrajectory("decision_vars").datatypes[i]
    if (var_name[:len(name)] == name):
      print(var_name + " = " + str(
        dircon_traj.GetTrajectory("decision_vars").datapoints[i, 0]))


def PlotState(dircon_traj, x_idx_start=0, x_idx_end=19, plot_derivatives=False):
  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes

  # Sampling for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)

  if not plot_derivatives:
    ### Plot state traj
    # Get the vel in the second mode
    state_2nd_mode = dircon_traj.GetTrajectory("state_traj1").datapoints[
                     x_idx_start:x_idx_end, 0:1]

    # Sampling the spline for visualization
    # state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
    state_samples = np.zeros((n_points, x_idx_end - x_idx_start))
    for i in range(n_points):
      state_samples[i] = state_traj.value(t[i])[x_idx_start:x_idx_end, 0]

    # Plotting reconstructed state trajectories
    figname = filename + " -- state trajectory " + str(x_idx_start) + "-" + str(x_idx_end)
    plt.figure(figname, figsize=figsize)
    plt.plot(t, state_samples)
    plt.gca().set_prop_cycle(None)
    plt.plot(t[-1], state_2nd_mode.T, 'o', markersize=4)
    plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
    plt.xlabel('time (s)')
    plt.legend(state_datatypes[x_idx_start:x_idx_end])
    if savefig:
      plt.savefig(save_path + figname + ".png")

  else:
    ### Plot derivatives of state traj
    # construct
    state_dot_traj = state_traj.MakeDerivative(1)
    state_dot_samples = np.zeros((n_points, x_idx_end - x_idx_start))
    for i in range(n_points):
      state_dot_samples[i] = state_dot_traj.value(t[i])[x_idx_start:x_idx_end, 0]
    # plot
    figname = filename + " -- state dot trajectory " + str(x_idx_start) + "-" + str(x_idx_end)
    plt.figure(figname, figsize=figsize)
    plt.plot(t, state_dot_samples)
    plt.xlabel('time (s)')
    plt.legend([name + " dot" for name in state_datatypes[x_idx_start:x_idx_end]])
    if savefig:
      plt.savefig(save_path + figname + ".png")


def PlotInput(dircon_traj):
  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  t_coll = dircon_traj.GetCollocationForceBreaks(0)

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  input_traj = dircon_traj.ReconstructInputTrajectory()
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  for i in range(n_points):
    input_samples[i] = input_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories
  figname = filename + " -- input trajectory"
  plt.figure(figname, figsize=figsize)
  plt.plot(t, input_samples)
  plt.xlabel('time (s)')
  plt.legend(input_datatypes)
  if savefig:
    plt.savefig(save_path + figname + ".png")


def PlotForce(dircon_traj):
  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  force_knot = dircon_traj.GetForceSamples(0)
  t_coll = dircon_traj.GetCollocationForceBreaks(0)
  force_coll = dircon_traj.GetCollocationForceSamples(0)

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  force_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0),
    dircon_traj.GetForceSamples(0))
  force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes
  force_c_traj = PiecewisePolynomial.ZeroOrderHold(
    dircon_traj.GetCollocationForceBreaks(0),
    dircon_traj.GetCollocationForceSamples(0))
  force_c_datatypes = dircon_traj.GetTrajectory(
    "collocation_force_vars0").datatypes

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
  force_c_samples = np.zeros((n_points, force_c_traj.value(0).shape[0]))
  for i in range(n_points):
    force_samples[i] = force_traj.value(t[i])[:, 0]
    force_c_samples[i] = force_c_traj.value(t[i])[:, 0]

  # Plotting reconstructed state trajectories

  # plt.figure("force trajectory")
  # plt.plot(t, force_samples)
  # plt.legend(force_datatypes)
  #
  # plt.figure("collocation force trajectory")
  # plt.plot(t, force_c_samples)
  # plt.legend(force_c_datatypes)

  # plt.figure("force at knots and collocation pts")
  # plt.plot(t_knot, force_knot.T)
  # plt.plot(t_coll, force_coll.T, 'ko', markersize=4)
  # plt.legend(force_datatypes + force_c_datatypes)

  # Reconstruct force at both collocation and knots
  t_knot_col = np.zeros((t_knot.shape[0] + t_coll.shape[0], 1))
  force_knot_col = np.zeros((force_knot.shape[0], t_knot_col.shape[0]))
  force_knot_col[:, 0] = force_knot[:, 0]
  for i in range(1, t_knot.shape[0]):
    t_knot_col[2 * i - 1] = t_coll[i - 1]
    t_knot_col[2 * i] = t_knot[i]
    force_knot_col[:, 2 * i - 1] = force_coll[:, i - 1]
    force_knot_col[:, 2 * i] = force_knot[:, i]
  figname = filename + " -- force at knots and collocation pts"
  plt.figure(figname, figsize=figsize)
  plt.plot(t_knot_col, force_knot_col.T)
  plt.plot(t_coll, force_coll.T, 'ko', markersize=2)
  plt.xlabel('time (s)')
  plt.legend(force_datatypes + force_c_datatypes)
  if savefig:
    plt.savefig(save_path + figname + ".png")

  ### Plot friction cone ratio
  friction_cone = np.zeros((n_points, 4))
  idx = 0
  for i in [0, 3]:
    friction_cone[:, 0+idx] = force_samples[:, 0+i] / force_samples[:, 2+i]
    friction_cone[:, 1+idx] = force_samples[:, 1+i] / force_samples[:, 2+i]
    idx += 2
  figname = filename + " -- friction cone (idx 0 to 5)"
  plt.figure(figname, figsize=figsize)
  plt.plot(t, friction_cone, ".")
  plt.legend(["pt1 x", "pt1 y", "pt2 x", "pt2 y"])
  if savefig:
    plt.savefig(save_path + figname + ".png")



def PlotCenterOfMass(dircon_traj, visualize_only_collocation_point=False):
  # Get data at knots
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)
  xdot_knot = dircon_traj.GetStateDerivativeSamples(0)

  # Compute for knot points
  com_at_knot = np.zeros((3, t_knot.shape[0]))
  comdot_at_knot = np.zeros((3, t_knot.shape[0]))
  comddot_at_knot = np.zeros((3, t_knot.shape[0]))
  for i in range(t_knot.shape[0]):
    xi = x_knot[:, i]
    plant.SetPositionsAndVelocities(context, xi)
    com_at_knot[:, i] = plant.CalcCenterOfMassPositionInWorld(context)
    J = plant.CalcJacobianCenterOfMassTranslationalVelocity(context,
      JacobianWrtVariable.kV, world, world)
    comdot_at_knot[:, i] = J @ x_knot[nq:, i]
    JdotV_i = plant.CalcBiasCenterOfMassTranslationalAcceleration(context,
      JacobianWrtVariable.kV, world, world)
    comddot_at_knot[:, i] = J @ xdot_knot[nq:, i] + JdotV_i

  # # Plot com at knot points only
  # plt.figure("com traj at knot pts")
  # plt.plot(t_knot, com_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("comdot traj at knot pts")
  # plt.plot(t_knot, comdot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("comddot traj at knot pts")
  # plt.plot(t_knot, comddot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])

  # Compute along the cubic spline
  if visualize_only_collocation_point:
    n_visualize = 1
  else:
    n_visualize = 100
  t_coll = np.zeros((t_knot.shape[0] - 1) * n_visualize)
  com_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  comdot_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  comddot_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  for i in range(t_knot.shape[0] - 1):
    # Get x and xdot at collocation points
    x_cs = CubicSpline(np.squeeze(t_knot[i: i + 2]), x_knot[:, i:i + 2].T,
      bc_type=((1, xdot_knot[:, i].T), (1, xdot_knot[:, i + 1].T)))
    # Compute com, comdot, comddot
    ratio = np.linspace(0, 1, n_visualize + 1, endpoint=False)[1:]
    for j in range(n_visualize):
      t = t_knot[i] + ratio[j] * (t_knot[i + 1] - t_knot[i])
      t_coll[i * n_visualize + j] = t
      x_col = x_cs(t[0])
      xdot_col = x_cs(t[0], 1)
      plant.SetPositionsAndVelocities(context, x_col)
      com_at_coll[:, i * n_visualize + j] = \
        plant.CalcCenterOfMassPositionInWorld(context)
      J = plant.CalcJacobianCenterOfMassTranslationalVelocity(context,
        JacobianWrtVariable.kV, world, world)
      comdot_at_coll[:, i * n_visualize + j] = J @ x_col[nq:]
      JdotV_i = plant.CalcBiasCenterOfMassTranslationalAcceleration(context,
        JacobianWrtVariable.kV, world, world)
      comddot_at_coll[:, i * n_visualize + j] = J @ xdot_col[nq:] + JdotV_i

  if visualize_only_collocation_point:
    # Plot com at both knot and collocation points
    figname = filename + " -- com traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, com_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, com_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, com_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- comdot traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, comdot_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, comdot_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, comdot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- comddot traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, comddot_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, comddot_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, comddot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
  else:
    # Plot com along the cubic splines
    figname = filename + " -- com traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, com_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, com_at_knot.T, 'ko', markersize=4)
    # plt.xlabel('com')
    plt.xlabel('time (s)')
    # plt.legend(['x', 'y', 'z'])
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- comdot traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, comdot_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, comdot_at_knot.T, 'ko', markersize=4)
    # plt.xlabel('comdot')
    plt.xlabel('time (s)')
    # plt.legend(['x', 'y', 'z'])
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- comddot traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, comddot_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, comddot_at_knot.T, 'ko', markersize=4)
    # plt.xlabel('comddot')
    plt.xlabel('time (s)')
    # plt.legend(['x', 'y', 'z'])
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")


def PlotPelvis(dircon_traj, visualize_only_collocation_point=False):
  #
  pelvis_frame = plant.GetBodyByName("pelvis").body_frame()

  # Get data at knots
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)
  xdot_knot = dircon_traj.GetStateDerivativeSamples(0)

  # Compute for knot points
  base_xyz_at_knot = np.zeros((3, t_knot.shape[0]))
  base_xyzdot_at_knot = np.zeros((3, t_knot.shape[0]))
  base_xyzddot_at_knot = np.zeros((3, t_knot.shape[0]))
  for i in range(t_knot.shape[0]):
    xi = x_knot[:, i]
    plant.SetPositionsAndVelocities(context, xi)
    base_xyz_at_knot[:, i] = plant.CalcPointsPositions(context, pelvis_frame,
      np.zeros(3), world).flatten()
    J = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, pelvis_frame, np.zeros(3), world, world)
    base_xyzdot_at_knot[:, i] = J @ x_knot[nq:, i]
    JdotV_i = plant.CalcBiasTranslationalAcceleration(context,
      JacobianWrtVariable.kV, pelvis_frame, np.zeros(3), world, world).flatten()
    base_xyzddot_at_knot[:, i] = J @ xdot_knot[nq:, i] + JdotV_i

  # # Plot base_xyz at knot points only
  # plt.figure("base_xyz traj at knot pts")
  # plt.plot(t_knot, base_xyz_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("base_xyzdot traj at knot pts")
  # plt.plot(t_knot, base_xyzdot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])
  # plt.figure("base_xyzddot traj at knot pts")
  # plt.plot(t_knot, base_xyzddot_at_knot.T)
  # plt.legend(['x', 'y', 'z'])

  # Compute along the cubic spline
  if visualize_only_collocation_point:
    n_visualize = 1
  else:
    n_visualize = 100
  t_coll = np.zeros((t_knot.shape[0] - 1) * n_visualize)
  base_xyz_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  base_xyzdot_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  base_xyzddot_at_coll = np.zeros((3, (t_knot.shape[0] - 1) * n_visualize))
  for i in range(t_knot.shape[0] - 1):
    # Get x and xdot at collocation points
    x_cs = CubicSpline(np.squeeze(t_knot[i: i + 2]), x_knot[:, i:i + 2].T,
      bc_type=((1, xdot_knot[:, i].T), (1, xdot_knot[:, i + 1].T)))
    # Compute base_xyz, base_xyzdot, base_xyzddot
    ratio = np.linspace(0, 1, n_visualize + 1, endpoint=False)[1:]
    for j in range(n_visualize):
      t = t_knot[i] + ratio[j] * (t_knot[i + 1] - t_knot[i])
      t_coll[i * n_visualize + j] = t
      x_col = x_cs(t[0])
      xdot_col = x_cs(t[0], 1)
      plant.SetPositionsAndVelocities(context, x_col)
      base_xyz_at_coll[:, i * n_visualize + j] = plant.CalcPointsPositions(
        context, pelvis_frame, np.zeros(3), world).flatten()
      J = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, pelvis_frame, np.zeros(3), world,
        world)
      base_xyzdot_at_coll[:, i * n_visualize + j] = J @ x_col[nq:]
      JdotV_i = plant.CalcBiasTranslationalAcceleration(context,
        JacobianWrtVariable.kV, pelvis_frame, np.zeros(3), world,
        world).flatten()
      base_xyzddot_at_coll[:, i * n_visualize + j] = J @ xdot_col[nq:] + JdotV_i

  if visualize_only_collocation_point:
    # Plot base_xyz at both knot and collocation points
    figname = filename + " -- base_xyz traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, base_xyz_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, base_xyz_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, base_xyz_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- base_xyzdot traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, base_xyzdot_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, base_xyzdot_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, base_xyzdot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- base_xyzddot traj at knot and coll pts"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_knot, base_xyzddot_at_knot.T)
    plt.gca().set_prop_cycle(None)  # reset color cycle
    plt.plot(t_coll, base_xyzddot_at_coll.T, 'o', markersize=2.5)
    plt.plot(t_knot, base_xyzddot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x col', 'y col', 'z col'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
  else:
    # Plot base_xyz along the cubic splines
    figname = filename + " -- base_xyz traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, base_xyz_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, base_xyz_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- base_xyzdot traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, base_xyzdot_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, base_xyzdot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")
    figname = filename + " -- base_xyzddot traj along the traj"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(t_coll, base_xyzddot_at_coll.T, 'o', markersize=2)
    plt.plot(t_knot, base_xyzddot_at_knot.T, 'ko', markersize=4)
    plt.xlabel('time (s)')
    plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])
    if savefig:
      plt.savefig(save_path + figname + ".png")


def PlotDynamicError(dircon_traj, x_idx_start=0, x_idx_end=19):
  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  input_traj = dircon_traj.ReconstructInputTrajectory()
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes
  force_traj = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0),
    dircon_traj.GetForceSamples(0))
  force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes

  # Create time and state samples
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[:, 0]
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  for i in range(n_points):
    input_samples[i] = input_traj.value(t[i])[:, 0]
  force_samples = np.zeros((n_points, force_traj.value(0).shape[0]))
  for i in range(n_points):
    force_samples[i] = force_traj.value(t[i])[:, 0]

  # Sampling the spline for visualization
  n_points = 10000
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)

  # From time derivatives of cubic spline
  xdot_traj = state_traj.MakeDerivative(1)
  xdot_from_spline = np.zeros((n_points, nx))
  for i in range(n_points):
    xdot_from_spline[i] = xdot_traj.value(t[i])[:, 0]

  # From dynamics function
  xdot_from_func = np.zeros((n_points, nx))
  for i in range(n_points):
    plant.SetPositionsAndVelocities(context, state_traj.value(t[i]))

    u = input_samples[i]
    lamb = force_samples[i]

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    B = plant.MakeActuationMatrix()
    g = plant.CalcGravityGeneralizedForces(context)
    Cv = plant.CalcBiasTerm(context)

    # The constraint order should be the same as trajopt's
    J_lt = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world,
      world)
    J_lh = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world,
      world)
    J_rt = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world,
      world)
    J_rh = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world,
      world)
    J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
    J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
    # J = np.vstack((J_lt, J_lh, J_rt, J_rh, J_l_loop_closure, J_r_loop_closure))
    J = np.vstack((J_lt, J_lh, J_l_loop_closure, J_r_loop_closure))

    # xdot_from_func[i, :nq] = state_samples[i][nq:]
    # import pdb; pdb.set_trace()
    xdot_from_func[i, nq:] = M_inv @ (-Cv + g + B @ u + J.T @ lamb)

  # Plotting reconstructed state trajectories
  figname = filename + " -- xdot trajectory " + str(x_idx_start) + "-" + str(x_idx_end)
  plt.figure(figname, figsize=figsize)
  # plt.plot(t, (xdot_from_spline)[:, x_idx_start:x_idx_end])
  plt.plot(t, (xdot_from_func)[:, x_idx_start:x_idx_end])
  # plt.plot(t, (xdot_from_spline - xdot_from_func)[:, x_idx_start:x_idx_end], 'ko', markersize=1)
  # plt.plot(t_knot, x_knot.T, 'ko', markersize=2)
  plt.xlabel('time (s)')
  plt.legend(state_datatypes[x_idx_start:x_idx_end])
  if savefig:
    plt.savefig(save_path + figname + ".png")


# PlotStateAtKnots is for testing (TODO: delete later)
def PlotStateAtKnots(dircon_traj, x_idx_start=0, x_idx_end=19):
  # Get data at knot points
  t_knot = dircon_traj.GetStateBreaks(0)
  x_knot = dircon_traj.GetStateSamples(0)[x_idx_start:x_idx_end, :]

  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes

  # Plotting reconstructed state trajectories
  figname = filename + " -- state trajectory " + str(x_idx_start) + "-" + str(
    x_idx_end) + "(at knots)"
  plt.figure(figname, figsize=figsize)
  plt.plot(t_knot, x_knot.T)
  plt.xlabel('time (s)')
  plt.legend(state_datatypes[x_idx_start:x_idx_end])


if __name__ == "__main__":
  main()
