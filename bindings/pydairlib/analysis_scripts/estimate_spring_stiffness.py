import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import mathematicalprogram as mp
# import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import FindResourceOrThrow
from numpy.linalg import inv
from pydairlib.lcm import process_lcm_log

def main():
  global l_knee_spring_idx, r_knee_spring_idx, l_heel_spring_idx, r_heel_spring_idx
  global l_knee_spring_dot_idx, r_knee_spring_dot_idx, l_heel_spring_dot_idx, r_heel_spring_dot_idx
  global plant, context, world, l_toe_frame, r_toe_frame
  global front_contact_disp, rear_contact_disp
  global l_knee_idx, r_knee_idx, l_heel_idx, r_heel_idx
  global sample_times
  global nq, nv, nx, nu
  global filename
  global t_u_slice
  global t_slice
  global pos_map
  global vel_map
  global act_map
  global l_loop_closure, r_loop_closure

  builder = DiagramBuilder()
  # plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  # Parser(plant).AddModelFromFile(
  #   FindResourceOrThrow(
  #     "examples/Cassie/urdf/cassie_v2.urdf"))
  # plant.mutable_gravity_field().set_gravity_vector(
  #
  #   -9.81 * np.array([0, 0, 1]))
  # plant.Finalize()

  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant, scene_graph, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # relevant MBP parameters
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  context = plant.CreateDefaultContext()

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  l_knee_spring_idx = pos_map["knee_joint_left"]
  r_knee_spring_idx = pos_map["knee_joint_right"]
  l_heel_spring_idx = pos_map["ankle_spring_joint_left"]
  r_heel_spring_idx = pos_map["ankle_spring_joint_right"]
  l_knee_spring_dot_idx = vel_map["knee_joint_leftdot"]
  r_knee_spring_dot_idx = vel_map["knee_joint_rightdot"]
  l_heel_spring_dot_idx = vel_map["ankle_spring_joint_leftdot"]
  r_heel_spring_dot_idx = vel_map["ankle_spring_joint_rightdot"]
  l_knee_idx = pos_map["knee_left"]
  r_knee_idx = pos_map["knee_right"]
  l_heel_idx = pos_map["ankle_joint_left"]
  r_heel_idx = pos_map["ankle_joint_right"]

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)

  l_loop_closure = LeftLoopClosureEvaluator(plant)
  r_loop_closure = RightLoopClosureEvaluator(plant)

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  matplotlib.rcParams["savefig.directory"] = path

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log, t_lcmlog_u = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

  # Will need to manually select the data range
  t_start = t_x[10]
  t_end = t_x[-10]
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  # start_time_idx = np.argwhere(np.abs(t_u - t_start) < 2e-3)[0][0]
  # end_time_idx = np.argwhere(np.abs(t_u - t_end) < 2e-3)[0][0]
  # t_u_slice = slice(start_time_idx, end_time_idx)
  # sample_times = [215.4, 229.4, 252.8, 265.3, 282.1, 289.0]
  sample_times = np.arange(52.5, 56, 0.1)

  joint_idx = vel_map["knee_joint_leftdot"]
  act_idx = act_map["knee_left_motor"]
  xdot = np.zeros(x.shape)
  # plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes)
  # plot_force_residual(t_x, x, xdot, u_meas, joint_idx, act_idx)
  solve_for_k(x, t_x, u, t_u)
  plt.show()
  # solve_with_lambda(x, t_x, u, t_u)

def plot_force_residual(t_x, x, xdot, u_meas, joint_idx, act_idx):
  n_samples = len(3*sample_times)

  x_samples = []
  u_samples = []
  xdot_samples = []
  t_samples = []

  tau_res = np.zeros((n_samples, nv))
  tau_res_wo_damping = np.zeros((n_samples, nv))
  tau_res_wo_springs = np.zeros((n_samples, nv))
  generalized_force = np.zeros((n_samples, nv))
  Bu_force = np.zeros((n_samples, nv))
  Cv_force = np.zeros((n_samples, nv))
  g_force = np.zeros((n_samples, nv))
  J_lambda = np.zeros((n_samples, nv))
  J_lambda_spring = np.zeros((n_samples, nv))
  K_force = np.zeros((n_samples, nv))
  # Jv = np.zeros((n_samples, 2))

  for i in range(len(3*sample_times)):
    t = t_x[50] + 1e-2 * i
    t = -0.05 + (0.05 * np.mod(i,3)) + sample_times[i//3]
    ind = np.argwhere(np.abs(t_x - t) < 1e-3)[0][0]
    x_samples.append(x[ind, :])
    xdot_samples.append(xdot[ind, :])
    u_samples.append(u_meas[ind, :])
    t_samples.append(t)
    plant.SetPositionsAndVelocities(context, x[ind, :])

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    B = plant.MakeActuationMatrix()
    g = plant.CalcGravityGeneralizedForces(context)
    Cv = plant.CalcBiasTerm(context)
    J_lh = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
    J_lt = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
    J_rh = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
    J_rt = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)

    J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
    J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
    J = np.vstack((J_lh, J_lt[1:], J_rh, J_rt[1:], J_l_loop_closure, J_r_loop_closure))

    # J = np.vstack((J_l_loop_closure, J_r_loop_closure))
    JdotV_l_loop_closure = l_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV_r_loop_closure = r_loop_closure.EvalFullJacobianDotTimesV(context)
    # JdotV = np.vstack((JdotV_l_loop_closure, JdotV_r_loop_closure))
    # JdotV = np.reshape(JdotV, (2,))
    JdotV = np.zeros(12)

    # A[i, 0] = -x[ind, nq + joint_idx]
    # b[i] = M[joint_idx, joint_idx] * xdot[ind, nq + joint_idx] - u_meas[ind, act_idx]

    qdot = x[ind, nq:]
    qddot = xdot[ind, nq:]
    K = np.zeros((nv, nq))
    K[l_knee_spring_dot_idx, l_knee_spring_idx] = 1129.0
    K[r_knee_spring_dot_idx, r_knee_spring_idx] = 1290.0
    K[l_heel_spring_dot_idx, l_heel_spring_idx] = 1264.0
    K[r_heel_spring_dot_idx, r_heel_spring_idx] = 1231.0
    # K[l_knee_spring_idx, l_knee_spring_idx] = 1000
    # K[r_knee_spring_idx, r_knee_spring_idx] = 1000
    # K[l_heel_spring_idx, l_heel_spring_idx] = 1000
    # K[r_heel_spring_idx, r_heel_spring_idx] = 1000
    force_offsets = np.zeros(nv)
    force_offsets[vel_map["knee_joint_leftdot"]] = 1.88336193
    force_offsets[vel_map["knee_joint_rightdot"]] = 1.2012477
    force_offsets[vel_map["ankle_spring_joint_leftdot"]] = 0.0428804
    force_offsets[vel_map["ankle_spring_joint_rightdot"]] = 0.03423141
    K = -K
    D = np.zeros((nv, nv))
    # D[joint_idx, joint_idx] = -2.0/3

    # Compute force residuals
    lambda_implicit =            inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + K@x[ind, :nq] + D@qdot) - JdotV)
    lambda_implicit_wo_damping = inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + K@x[ind, :nq])          - JdotV)
    lambda_implicit_wo_spring =  inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + D@qdot)                 - JdotV)
    lambda_implicit_spring =  inv(J @ M_inv @ J.T) @ (- J @ M_inv @( K @ x[ind, :nq]))
    tau_res[i] =            M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit            - K@x[ind, :nq] - D@qdot - force_offsets
    tau_res_wo_damping[i] = M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit_wo_damping - K@x[ind, :nq]
    tau_res_wo_springs[i] = M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit_wo_spring  - D@qdot

    # Jv[i] = J@qdot

    generalized_force[i] = M @ qddot
    Bu_force[i] = B@u_meas[ind]
    Cv_force[i] = Cv
    g_force[i] = g
    J_lambda[i] = J.T @ lambda_implicit
    J_lambda_spring[i] = J.T @ lambda_implicit_spring
    K_force[i] = K @ x[ind, :nq]

  plt.figure("force contribution")

  # plt.plot(t_samples, generalized_force[:, joint_idx])
  # plt.plot(t_samples, Bu_force[:, joint_idx])
  # plt.plot(t_samples, Cv_force[:, joint_idx])
  # plt.plot(t_samples, g_force[:, joint_idx])
  # plt.plot(t_samples, J_lambda[:, joint_idx])
  # plt.plot(t_samples, K_force[:, joint_idx])
  plt.plot(t_samples, tau_res[:, vel_map["knee_joint_leftdot"]])
  plt.plot(t_samples, tau_res[:, vel_map["knee_joint_rightdot"]])
  plt.plot(t_samples, tau_res[:, vel_map["ankle_spring_joint_leftdot"]])
  plt.plot(t_samples, tau_res[:, vel_map["ankle_spring_joint_rightdot"]])
  # plt.plot(t_samples, J_lambda_spring[:, joint_idx])
  # plt.legend(['Mqddot', 'Bu', 'Cv', 'g', 'J.T lambda', 'Kq', 'residual', "J.T lambda_spring"])
  plt.legend(['knee_left', 'knee_right', 'ankle_spring_left', 'ankle_spring_right'])

  plt.figure("force res vs q")
  x_samples = np.array(x_samples)
  plt.plot(x_samples[:, pos_map['knee_joint_left']], tau_res[:, vel_map["knee_joint_leftdot"]])
  plt.plot(x_samples[:, pos_map['knee_joint_right']], tau_res[:, vel_map["knee_joint_rightdot"]])
  plt.plot(x_samples[:, pos_map['ankle_spring_joint_left']], tau_res[:, vel_map["ankle_spring_joint_leftdot"]])
  plt.plot(x_samples[:, pos_map['ankle_spring_joint_right']], tau_res[:, vel_map["ankle_spring_joint_rightdot"]])
  plt.legend(['knee_left', 'knee_right', 'ankle_spring_left', 'ankle_spring_right'])

  # plt.plot(x_samples[:, joint_idx + 1], tau_res[:, joint_idx], '.')
  # joint_pos_idx = pos_map["knee_joint_right"]
  # joint_vel_idx = vel_map["knee_joint_rightdot"]
  # plt.plot(x_samples[:, joint_idx + 2], tau_res[:, joint_idx  + 1], '.')
  # joint_pos_idx = pos_map["ankle_spring_joint_left"]
  # joint_vel_idx = vel_map["ankle_spring_joint_leftdot"]
  # plt.plot(x_samples[:, joint_pos_idx], tau_res[:, joint_vel_idx], '.')
  # joint_pos_idx = pos_map["ankle_spring_joint_right"]
  # joint_vel_idx = vel_map["ankle_spring_joint_rightdot"]
  # plt.plot(x_samples[:, joint_pos_idx], tau_res[:, joint_vel_idx], '.')

pass


def solve_for_k(x, t_x, u, t_u):
  n_samples = len(sample_times)
  n_samples_per_iter = 3
  # K is a diagonal matrix, do the springs act directly on the knees? If so the non-zero values will not
  # be on the diagonals
  n_k = 4
  nvars = n_k + 4
  prog = mp.MathematicalProgram()

  A = np.zeros((n_samples * n_samples_per_iter * nv, nvars))
  b = np.zeros(n_samples * n_samples_per_iter * nv)

  for i in range(n_samples):
    for j in range(n_samples_per_iter):
      delta_t = 1e-2 * j
      x_ind = np.argwhere(np.abs(t_x - (sample_times[i] + delta_t)) < 1e-2)[0][0]
      u_ind = np.argwhere(np.abs(t_u - (sample_times[i] + delta_t)) < 1e-2)[0][0]
      plant.SetPositionsAndVelocities(context, x[x_ind, :])

      M = plant.CalcMassMatrixViaInverseDynamics(context)
      M_inv = np.linalg.inv(M)
      B = plant.MakeActuationMatrix()
      g = plant.CalcGravityGeneralizedForces(context)
      J_lh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
      J_lt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
      J_rh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
      J_rt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)

      J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
      J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
      # J = np.vstack((J_lh, J_lt, J_rh, J_rt, J_l_loop_closure, J_r_loop_closure))
      J = np.vstack((J_lh, J_lt[1:], J_rh, J_rt[1:], J_l_loop_closure, J_r_loop_closure))

      row_start = i * (nv * n_samples_per_iter) + nv * j
      row_end = i * (nv * n_samples_per_iter) + nv * (j + 1)

      # lambda_implicit = inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + K@x[ind, :nq] + D@qdot) - JdotV)
      lambda_i_wo_k = - inv(J @ M_inv @ J.T) @ (J @ M_inv @ (B @ u[u_ind] + g))
      lambda_i_w_k = - inv(J @ M_inv @ J.T) @ (J @ M_inv)

      A[row_start + l_knee_spring_dot_idx, 0] = -x[x_ind, l_knee_spring_idx]
      A[row_start + r_knee_spring_dot_idx, 1] = -x[x_ind, r_knee_spring_idx]
      A[row_start + l_heel_spring_dot_idx, 2] = -x[x_ind, l_heel_spring_idx]
      A[row_start + r_heel_spring_dot_idx, 3] = -x[x_ind, r_heel_spring_idx]
      A[row_start + l_knee_spring_dot_idx, 4] = 1
      A[row_start + r_knee_spring_dot_idx, 5] = 1
      A[row_start + l_heel_spring_dot_idx, 6] = 1
      A[row_start + r_heel_spring_dot_idx, 7] = 1

      A[row_start:row_end, 0] += (J.T @ lambda_i_w_k)[:, l_knee_spring_dot_idx] * x[x_ind, l_knee_spring_idx]
      A[row_start:row_end, 1] += (J.T @ lambda_i_w_k)[:, r_knee_spring_dot_idx] * x[x_ind, r_knee_spring_idx]
      A[row_start:row_end, 2] += (J.T @ lambda_i_w_k)[:, l_heel_spring_dot_idx] * x[x_ind, l_heel_spring_idx]
      A[row_start:row_end, 3] += (J.T @ lambda_i_w_k)[:, r_heel_spring_dot_idx] * x[x_ind, r_heel_spring_idx]
      # A[row_start:row_end, 1] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, r_knee_spring_idx] * x[
      #   x_ind, r_knee_spring_idx]
      # A[row_start:row_end, 2] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, l_heel_spring_idx] * x[
      #   x_ind, l_heel_spring_idx]
      # A[row_start:row_end, 3] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, l_knee_spring_idx] * x[
      #   x_ind, r_heel_spring_idx]
      b[row_start:row_end] = - B @ u[u_ind] - g - J.T @ lambda_i_wo_k

  x = prog.NewContinuousVariables(nvars, "sigma")

  prog.AddL2NormCost(A, b, x)
  solver_id = mp.ChooseBestSolver(prog)
  print(solver_id.name())
  solver = mp.MakeSolver(solver_id)
  result = solver.Solve(prog, None, None)
  print("LSTSQ cost: ", result.get_optimal_cost())
  print("Solution result: ", result.get_solution_result())
  sol = result.GetSolution()
  K = sol[0:n_k]
  offsets = sol[-n_k:]
  # lambdas = sol[-n_k:]
  print("K: ", K)
  print("offsets: ", offsets)
  # print("lambdas: ", lambdas)

def solve_with_lambda(x, t_x, u, t_u):
  n_samples = len(sample_times)
  n_samples_per_iter = 10
  n_lambda_vars = 14
  n_offset_vars = 4
  # K is a diagonal matrix, do the springs act directly on the knees? If so the non-zero values will not
  # be on the diagonals
  n_k = 4
  nvars = n_k + n_samples * n_lambda_vars + n_offset_vars
  prog = mp.MathematicalProgram()

  A = np.zeros((n_samples * n_samples_per_iter * nv, nvars))
  b = np.zeros(n_samples * n_samples_per_iter * nv)

  for i in range(n_samples):
    for j in range(n_samples_per_iter):
      delta_t = 1e-2 * j
      x_ind = np.argwhere(np.abs(t_x - sample_times[i] + delta_t) < 1e-3)[0][0]
      u_ind = np.argwhere(np.abs(t_u - sample_times[i] + delta_t) < 1e-3)[0][0]
      plant.SetPositionsAndVelocities(context, x[x_ind, :])

      M = plant.CalcMassMatrixViaInverseDynamics(context)
      M_inv = np.linalg.inv(M)
      B = plant.MakeActuationMatrix()
      g = plant.CalcGravityGeneralizedForces(context)
      J_lh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
      J_lt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
      J_rh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
      J_rt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)

      J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
      J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
      J = np.vstack((J_lh, J_lt, J_rh, J_rt, J_l_loop_closure, J_r_loop_closure))

      # import pdb; pdb.set_trace()
      row_start = i * (nv * n_samples_per_iter) + nv * j
      row_end = i * (nv * n_samples_per_iter) + nv * (j + 1)

      A[row_start + l_knee_idx - 1, 0] = x[x_ind, l_knee_spring_idx]
      A[row_start + r_knee_idx - 1, 1] = x[x_ind, r_knee_spring_idx]
      A[row_start + l_heel_idx - 1, 2] = x[x_ind, l_heel_spring_idx]
      A[row_start + r_heel_idx - 1, 3] = x[x_ind, r_heel_spring_idx]
      A[row_start:row_end, n_k + i * n_lambda_vars: n_k + (i + 1) * n_lambda_vars] = J.T
      A[row_start + l_knee_idx - 1, -4] = 1
      A[row_start + r_knee_idx - 1, -3] = 1
      A[row_start + l_heel_idx - 1, -2] = 1
      A[row_start + r_heel_idx - 1, -1] = 1

      b[row_start:row_end] = - B @ u[u_ind] - g

  x_vars = prog.NewContinuousVariables(nvars, "sigma")

  import pdb; pdb.set_trace()
  prog.AddL2NormCost(A, b, x_vars)
  Q = 100 * np.eye(n_offset_vars)
  prog.AddQuadraticCost(Q, np.zeros(n_offset_vars), x_vars[-n_offset_vars:])
  solver_id = mp.ChooseBestSolver(prog)
  print(solver_id.name())
  solver = mp.MakeSolver(solver_id)
  result = solver.Solve(prog, None, None)
  print("LSTSQ cost: ", result.get_optimal_cost())
  print("Solution result: ", result.get_solution_result())
  sol = result.GetSolution()
  k_sol = sol[0:n_k]
  lambdas = sol[n_k:nvars - n_offset_vars]
  offsets = sol[-n_offset_vars:]
  print("K: ", k_sol)
  print("lambdas: ", lambdas)
  print("offsets: ", offsets)
  import pdb; pdb.set_trace()

  K = np.zeros((nv, nq))
  K[l_knee_idx - 1, l_knee_spring_idx] = k_sol[0]
  K[r_knee_idx - 1, r_knee_spring_idx] = k_sol[1]
  K[l_heel_idx - 1, l_heel_spring_idx] = k_sol[2]
  K[r_heel_idx - 1, r_heel_spring_idx] = k_sol[3]

  joint_pos = np.zeros((n_samples, 4))
  err = np.zeros((n_samples, 4))
  for i in range(n_samples):
    for j in range(n_samples_per_iter):
      delta_t = 1e-2 * j
      x_ind = np.argwhere(np.abs(t_x - sample_times[i] + delta_t) < 1e-3)[0][0]
      u_ind = np.argwhere(np.abs(t_u - sample_times[i] + delta_t) < 1e-3)[0][0]
      plant.SetPositionsAndVelocities(context, x[x_ind, :])

      M = plant.CalcMassMatrixViaInverseDynamics(context)
      M_inv = np.linalg.inv(M)
      B = plant.MakeActuationMatrix()
      g = plant.CalcGravityGeneralizedForces(context)
      J_lh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
      J_lt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
      J_rh = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
      J_rt = plant.CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)

      J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
      J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
      J = np.vstack((J_lh, J_lt, J_rh, J_rt, J_l_loop_closure, J_r_loop_closure))

      forces = B @ u[u_ind] + g + J.T @ lambdas[n_lambda_vars * i: n_lambda_vars * (i+1)] + K @ (x[x_ind, :nq])
    err[i, 0] = forces[l_knee_idx - 1]
    err[i, 1] = forces[r_knee_idx - 1]
    err[i, 2] = forces[l_heel_idx - 1]
    err[i, 3] = forces[r_heel_idx - 1]
    joint_pos[i, 0] = x[x_ind, l_knee_spring_idx]
    joint_pos[i, 1] = x[x_ind, r_knee_spring_idx]
    joint_pos[i, 2] = x[x_ind, l_heel_spring_idx]
    joint_pos[i, 3] = x[x_ind, r_heel_spring_idx]
  # plt.plot(err, joint_pos)

  import pdb; pdb.set_trace()
  plt.plot(joint_pos, err)
  plt.figure()
  plt.plot(sample_times, err)
  plt.plot(sample_times, joint_pos)
  plt.show()


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes):
  pos_indices = slice(0, nq)
  vel_indices = slice(23, nx)
  u_indices = slice(6, 8)
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)

  plt.figure("positions: " + filename)
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["knee_joint_right"]])
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["ankle_spring_joint_right"]])
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  # plt.plot(sample_times, np.zeros((len(sample_times),)), 'k*')
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts: " + filename)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.figure("efforts meas: " + filename)
  # plt.figure("Delay characterization")
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  # plt.legend(u_datatypes[u_indices])


if __name__ == "__main__":
  main()
