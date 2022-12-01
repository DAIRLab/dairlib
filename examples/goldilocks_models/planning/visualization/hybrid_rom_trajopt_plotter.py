import sys
import time
import matplotlib.pyplot as plt
import pydairlib.lcm.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.multibody

from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import *

import argparse

"""
This script is modified from `rom_trajopt_plotter.py` 
"""

def main():
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
  File argumnets
  """
  ### argument parser
  parser = argparse.ArgumentParser()

  parser.add_argument('--debug', action='store_true')
  parser.add_argument('--no-debug', dest='debug', action='store_false')
  parser.set_defaults(debug=False)

  parser.add_argument("--start_idx", help="", default=0, type=int)
  parser.add_argument("--end_idx", help="", default=-1, type=int)
  # parser.add_argument("--robot_option", help="0 is five-link robot. 1 is cassie_fixed_spring", default=1, type=int, choices=[0, 1])
  parser.add_argument("--path", help="", default="", type=str)
  parser.add_argument("--folder_path", help="../dairlib_data/goldilocks_models/planning/robot_1/data/", default="", type=str)
  args = parser.parse_args()

  debug = args.debug
  start_idx = args.start_idx
  end_idx = args.end_idx if args.end_idx > 0 else start_idx + 1

  """
  Setup for saving figures
  """
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

  print("savefig = " + str(savefig))
  print("save_path = " + save_path)

  """
  Setup file path
  """
  filepath = ""
  if len(args.path) > 0:
    filepath = args.path
  else: 
    if debug:
      filepath = args.folder_path + "debug" + "_rom_trajectory"
    else:
      filepath = args.folder_path + str(start_idx) + "_rom_trajectory"

  print("filepath = " + filepath)

  """
  Some other setups
  """
  animate = (end_idx - start_idx) > 1


  """
  Parameters
  """
  lightweight_log = False

  # Manual overwrite
  # start_idx = 270
  # end_idx = 40
  # end_idx = start_idx + 1

  # filepath = args.folder_path + str(start_idx) + "_rom_trajectory"
  # filepath = args.folder_path + "debug" + "_rom_trajectory"

  """
  States, inputs trajectories
  """
  rom_traj = pydairlib.lcm.lcm_trajectory.HybridRomPlannerTrajectory(filepath, lightweight_log)
  PlotState(rom_traj, 0, rom_traj.GetStateSamples(0).shape[0])

  if not lightweight_log:
    if (rom_traj.GetInputSamples().shape[0] > 0):
      PlotInput(rom_traj)

  """
  Dynamic error
  """
  # PlotDynamicError(rom_traj, nq + 6, nx)

  """
  Print the value of the solutions
  """
  if not lightweight_log:
    PrintAllDecisionVar(rom_traj)

  """
  Testing
  """
  # com_vec, comdot_vec, comdot_end = PlotCOM(rom_traj)
  # # PlotFeet(rom_traj, True)
  # feet_pos_vec, feet_vel_vec, feet_vel_end = PlotFeet(rom_traj)
  # PlotCOMWrtStanceFoot(rom_traj, com_vec, comdot_vec, comdot_end, feet_pos_vec,
  #   feet_vel_vec, feet_vel_end, True)
  # PlotCOMWrtStanceFoot(rom_traj, com_vec, comdot_vec, comdot_end, feet_pos_vec,
  #   feet_vel_vec, feet_vel_end, False)

  """
  """
  if animate:
    x_axis_is_clock = True

    time_vec = np.zeros(end_idx - start_idx)
    stance_foot_vec = np.zeros(end_idx - start_idx)
    for j in range(start_idx, end_idx):
      filepath = args.folder_path + str(j) + "_rom_trajectory"
      rom_traj = pydairlib.lcm.lcm_trajectory.HybridRomPlannerTrajectory(filepath, lightweight_log)
      time_vec[j-start_idx] = rom_traj.get_global_com_pos_time()[0]
      stance_foot_vec[j-start_idx] = rom_traj.get_stance_foot()[0]
    loop_time = np.diff(time_vec)
    # import pdb; pdb.set_trace()

    plt.figure("Time elapsed", figsize=figsize)
    plt.title("Time elapsed in each MPC loop")
    if x_axis_is_clock:
      plt.plot(time_vec[:-1],loop_time, 'o')
      plt.xlabel('Clock (s)')
      plt.plot(time_vec, 0.1 * stance_foot_vec)
    else:
      plt.plot(loop_time, 'o')
      plt.xlabel('MPC loop idx')
      plt.plot(0.1 * stance_foot_vec)
    plt.ylabel('Time elapse (s)')
    # plt.show()


  if animate:
    # Run through some logs to get the bounds for visualization
    com_pos_max = np.zeros([2,1])
    com_pos_min = np.zeros([2,1])
    for j in range(start_idx, end_idx):
      filepath = args.folder_path + str(j) + "_rom_trajectory"
      rom_traj = pydairlib.lcm.lcm_trajectory.HybridRomPlannerTrajectory(filepath, lightweight_log)
      com_pos_max = np.max(np.hstack([rom_traj.get_global_com_pos(),com_pos_max]), 1).reshape(2,1)
      com_pos_min = np.min(np.hstack([rom_traj.get_global_com_pos(),com_pos_min]), 1).reshape(2,1)
    x_limit = [com_pos_min[0] - 1, com_pos_max[0]+1]
    y_limit = [com_pos_min[1] - 1, com_pos_max[1]+1]

    # real_time_rate = 0.1
    # prev_wall_time = -1 
    # prev_sim_time = -1
    is_first_frame = True
    for j in range(start_idx, end_idx, 1):
      filepath = args.folder_path + str(j) + "_rom_trajectory"
      rom_traj = pydairlib.lcm.lcm_trajectory.HybridRomPlannerTrajectory(filepath, lightweight_log)

      # if prev_wall_time > 0:
      #   while(real_time_rate * (time.time() - prev_wall_time) < (rom_traj.get_global_com_pos_time()[0] - prev_sim_time)):
      #     plt.pause(0.01)
      # plt.clf()
      # prev_wall_time = time.time()
      # prev_sim_time = rom_traj.get_global_com_pos_time()[0]

      PlotGlobalFeetAndCoMPosition(rom_traj, not is_first_frame, x_limit, y_limit)
 
      plt.draw()
      plt.pause(0.05)
      # plt.pause(0.1)
      # plt.pause(0.3)
      plt.clf()

      is_first_frame = False

  else:    
    PlotGlobalFeetAndCoMPosition(rom_traj)
    plt.show()


  # import pdb; pdb.set_trace()

  if not savefig:
    plt.show()

def PlotGlobalFeetAndCoMPosition(rom_traj, redraw=False, x_limit=[], y_limit=[]):
  start_with_left_stance = rom_traj.get_stance_foot()[0] < 0.5
  global_feet_pos = rom_traj.get_global_feet_pos()
  global_feet_pos_time = rom_traj.get_global_feet_pos_time()
  global_com_pos = rom_traj.get_global_com_pos()
  global_com_pos_time = rom_traj.get_global_com_pos_time()
  print("global_feet_pos = \n", global_feet_pos)
  print("global_com_pos = \n", global_com_pos)
  
  if not redraw:
    plt.figure("Feet and CoM positions", figsize=figsize)
  plt.title("current time = %.3f" % global_com_pos_time[0])

  palette = ['r', 'g', 'b']
  left_stance = start_with_left_stance
  for i in range(global_feet_pos.shape[1]):
    footmark = 'x' if left_stance else '+'
    plt.plot(global_feet_pos[0,i],global_feet_pos[1,i], palette[i] + footmark, markersize=12)
    plt.plot(global_com_pos[0,i],global_com_pos[1,i], palette[i] + 'o')
    left_stance = not left_stance

  # Draw pelvis orientation
  current_quat = rom_traj.get_current_quat_xyz_shift()
  print("current_quat = \n", current_quat)
  rot = Rotation.from_quat([current_quat[1], current_quat[2], current_quat[3], current_quat[0]]).as_matrix().T  # (x, y, z, w) format
  pelvis_x_axis = rot[0:2,0]
  pelvis_y_axis = rot[0:2,1]
  plt.arrow(x=global_feet_pos[0,0], y=global_feet_pos[1,0], dx=pelvis_x_axis[0]/5, dy=pelvis_x_axis[1]/5, width=.01) 
  plt.arrow(x=global_feet_pos[0,0], y=global_feet_pos[1,0], dx=pelvis_y_axis[0]/5, dy=pelvis_y_axis[1]/5, width=.01) 

  # Labels and stuffs
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  if len(x_limit) > 0:
    plt.xlim(x_limit)
    plt.ylim(y_limit)
  else:
    lb = np.min(np.hstack([global_feet_pos, global_com_pos]), 1)
    ub = np.min(np.hstack([global_feet_pos, global_com_pos]), 1)
    plt.xlim([lb[0] - 1, lb[1] + 1])
    plt.ylim([ub[0] - 1, ub[1] + 1])
  # Manually overwrite
  # plt.xlim([-0.2, 3])
  # plt.ylim([-1, 1])
  
  ax = plt.gca()
  ax.set_aspect('equal', adjustable='box')




# def PlotCOMWrtStanceFoot(rom_traj, com_vec, comdot_vec, comdot_end,
#     feet_pos_vec, feet_vel_vec, feet_vel_end, start_with_left_stance=True):
#   com_wrt_stance_foot_vec = np.zeros(com_vec.shape)
#   comdot_wrt_stance_foot_vec = np.zeros(comdot_vec.shape)
#   comdot_wrt_stance_foot_end = np.zeros(3)
#
#   n_mode = com_vec.shape[0]
#   x0_time = rom_traj.get_x0_time()
#
#   left_stance = start_with_left_stance
#   for i in range(n_mode):
#     print("start_with_left_stance" + str(left_stance))
#     stance_foot_pos = feet_pos_vec[0][i] if left_stance else feet_pos_vec[1][i]
#     stance_foot_vel = feet_vel_vec[0][i] if left_stance else feet_vel_vec[1][i]
#     com_wrt_stance_foot_vec[i] = com_vec[i] - stance_foot_pos
#     comdot_wrt_stance_foot_vec[i] = comdot_vec[i] - stance_foot_vel
#     left_stance = not left_stance
#   print("start_with_left_stance" + str(left_stance))
#   stance_foot_vel = feet_vel_end[0][i] if left_stance else feet_vel_end[1][i]
#   comdot_wrt_stance_foot_end = comdot_end - stance_foot_vel
#
#   suffix = " (assuming start with left foot)" if start_with_left_stance else " (assuming start with right foot)"
#
#   palette = ['r', 'b', 'g']
#   figname = "Full model: COM wrt stance foot" + suffix
#   plt.figure(figname, figsize=figsize)
#   for i in range(n_mode):
#     for j in range(3):
#       plt.plot(x0_time[i: i + 2], [com_wrt_stance_foot_vec[i][0][j],
#                                    com_wrt_stance_foot_vec[i][1][j]],
#         palette[j], markersize=4)
#   plt.ylabel('(m)')
#   plt.xlabel('mode index')
#   plt.legend(['x', 'y', 'z'])
#   figname = "Full model: COM vel wrt stance foot" + suffix
#   plt.figure(figname, figsize=figsize)
#   for i in range(n_mode):
#     for j in range(3):
#       plt.plot(x0_time[i: i + 2], [comdot_wrt_stance_foot_vec[i][0][j],
#                                    comdot_wrt_stance_foot_vec[i][1][j]],
#         palette[j], markersize=4)
#   for i in range(3):
#     plt.plot(x0_time[n_mode], comdot_wrt_stance_foot_end[i], palette[i] + 'o',
#       markersize=4)
#   plt.ylabel('(m/s)')
#   plt.xlabel('mode index')
#   plt.legend(['x', 'y', 'z'])
#
#
# def PlotFeet(rom_traj, toe_center = False):
#   disp = np.array((0, 0, 0)) if toe_center else mid_contact_disp
#
#   n_mode = rom_traj.GetNumModes()
#   vars = rom_traj.GetTrajectory("decision_vars")
#
#   feet_pos_vec = np.zeros((2, n_mode, 2, 3))
#   feet_vel_vec = np.zeros((2, n_mode, 2, 3))
#   feet_vel_end = np.zeros((2, 3))
#
#   toe_frames = [l_toe_frame, r_toe_frame]
#   names = ["left", "right"]
#
#   x0 = rom_traj.get_x0()
#   x0_time = rom_traj.get_x0_time()
#   xf = rom_traj.get_xf()
#   xf_time = rom_traj.get_xf_time()
#
#   for k in range(2):
#     for i in range(n_mode):
#       feet_pos_vec[k][i][0], feet_vel_vec[k][i][0] = CalcPointPosAndVel(
#         x0[:, i], toe_frames[k], disp)
#       feet_pos_vec[k][i][1], feet_vel_vec[k][i][1] = CalcPointPosAndVel(
#         xf[:, i], toe_frames[k], disp)
#     _, feet_vel_end[k] = CalcPointPosAndVel(x0[:, n_mode], toe_frames[k], disp)
#
#     palette = ['r', 'b', 'g']
#     figname = "Full model: " + names[k] + " foot pos"
#     plt.figure(figname, figsize=figsize)
#     for i in range(n_mode):
#       for j in range(3):
#         plt.plot(x0_time[i: i + 2],
#           [feet_pos_vec[k][i][0][j], feet_pos_vec[k][i][1][j]],
#           palette[j], markersize=4)
#     plt.ylabel('(m)')
#     plt.xlabel('mode index')
#     plt.legend(['x', 'y', 'z'])
#     figname = "Full model: " + names[k] + " foot vel"
#     plt.figure(figname, figsize=figsize)
#     for i in range(n_mode):
#       for j in range(3):
#         plt.plot(x0_time[i: i + 2],
#           [feet_vel_vec[k][i][0][j], feet_vel_vec[k][i][1][j]],
#           palette[j], markersize=4)
#     for i in range(3):
#       plt.plot(x0_time[n_mode], feet_vel_end[k][i], palette[i] + 'o',
#         markersize=4)
#     plt.ylabel('(m/s)')
#     plt.xlabel('mode index')
#     plt.legend(['x', 'y', 'z'])
#
#   return feet_pos_vec, feet_vel_vec, feet_vel_end
#
#
# def PlotCOM(rom_traj):
#   n_mode = rom_traj.GetNumModes()
#   vars = rom_traj.GetTrajectory("decision_vars")
#
#   com_vec = np.zeros((n_mode, 2, 3))
#   comdot_vec = np.zeros((n_mode, 2, 3))
#   comdot_end = np.zeros(3)
#
#   x0 = rom_traj.get_x0()
#   x0_time = rom_traj.get_x0_time()
#   xf = rom_traj.get_xf()
#   xf_time = rom_traj.get_xf_time()
#
#   for i in range(n_mode):
#     com_vec[i][0], comdot_vec[i][0] = CalcCenterOfMass(x0[:, i])
#     com_vec[i][1], comdot_vec[i][1] = CalcCenterOfMass(xf[:, i])
#   _, comdot_end = CalcCenterOfMass(x0[:, n_mode])
#
#   palette = ['r', 'b', 'g']
#   figname = "Full model: COM"
#   plt.figure(figname, figsize=figsize)
#   for i in range(n_mode):
#     for j in range(3):
#       plt.plot(x0_time[i: i + 2], [com_vec[i][0][j], com_vec[i][1][j]],
#         palette[j],
#         markersize=4)
#   plt.ylabel('(m)')
#   plt.xlabel('mode index')
#   plt.legend(['x', 'y', 'z'])
#   figname = "Full model: COM vel"
#   plt.figure(figname, figsize=figsize)
#   for i in range(n_mode):
#     for j in range(3):
#       plt.plot(x0_time[i: i + 2], [comdot_vec[i][0][j], comdot_vec[i][1][j]],
#         palette[j], markersize=4)
#   for i in range(3):
#     plt.plot(x0_time[n_mode], comdot_end[i], palette[i] + 'o', markersize=4)
#   plt.ylabel('(m/s)')
#   plt.xlabel('mode index')
#   plt.legend(['x', 'y', 'z'])
#
#   return com_vec, comdot_vec, comdot_end


def CalcPointPosAndVel(x, frame, point):
  plant_FOM.SetPositionsAndVelocities(context, x)
  pos = plant_FOM.CalcPointsPositions(context, frame, point, world)
  vel = plant_FOM.CalcJacobianTranslationalVelocity(context,
    JacobianWrtVariable.kV, frame, point, world, world) @ x[nq_FOM:]
  return pos.flatten(), vel.flatten()


def CalcCenterOfMass(x):
  plant_FOM.SetPositionsAndVelocities(context, x)
  com = plant_FOM.CalcCenterOfMassPositionInWorld(context)
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
