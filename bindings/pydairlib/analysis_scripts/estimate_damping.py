import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import process_lcm_log
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import FindResourceOrThrow


def main():
  global l_knee_spring_idx, r_knee_spring_idx, l_heel_spring_idx, r_heel_spring_idx
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
  global x_datatypes, u_datatypes
  global K
  global offsets

  # parameter
  global data_has_floating_base, process_with_fixed_base
  data_has_floating_base = True
  process_with_fixed_base = False

  np.set_printoptions(precision=3)

  builder = DiagramBuilder()
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant, scene_graph, not process_with_fixed_base,
    "examples/Cassie/urdf/cassie_v2.urdf", False, False)
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
  l_knee_idx = pos_map["knee_left"]
  r_knee_idx = pos_map["knee_right"]
  l_heel_idx = pos_map["ankle_joint_left"]
  r_heel_idx = pos_map["ankle_joint_right"]

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)

  l_loop_closure = LeftLoopClosureEvaluator(plant)
  r_loop_closure = RightLoopClosureEvaluator(plant)

  K = np.zeros((nv, nq))
  offsets = np.zeros(nq)
  use_springs = False
  if(use_springs):
    K[7,8] = 1097.0
    K[8,9] = 1264.0
    K[11,12] = 1196.0
    K[13,14] = 1184.0
    offsets[7] = 0.111
    offsets[8] = 0.103
    offsets[12] = 0.066
    offsets[14] = 0.070

  filename = sys.argv[1]
  if(len(sys.argv) > 2):
    controller_name = sys.argv[2]
  else:
    controller_name = "CASSIE_INPUT"
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]
  joint_name = filename[7:filename.find('_sine')]

  matplotlib.rcParams["savefig.directory"] = path

  dummy_builder = DiagramBuilder()
  plant_data, scene_graph = AddMultibodyPlantSceneGraph(dummy_builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_data, scene_graph, data_has_floating_base,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  plant_data.Finalize()
  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log = process_lcm_log.process_log(log, pydairlib.multibody.makeNameToPositionsMap(plant_data),
                                                     pydairlib.multibody.makeNameToVelocitiesMap(plant_data),
                                                     pydairlib.multibody.makeNameToActuatorsMap(plant_data),
                                                     controller_name)

  if process_with_fixed_base and data_has_floating_base:
    nq_data = plant_data.num_positions()
    x_temp = np.copy(x)
    x = np.zeros((x_temp.shape[0], nx))
    x[:, :nq] = x_temp[:, 7:nq_data]
    x[:, nq:] = x_temp[:, nq_data+6:]

  # import pdb; pdb.set_trace()

  # Will need to manually select the data range
  t_start = t_x[200]
  t_end = t_x[-200]
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  xdot = estimate_xdot_with_filtering(x, t_x)
  # plot_imu_data(t_x, cassie_out)
  # plot_state(x, t_x, u, t_u, u_meas, x_datatypes, u_datatypes)

  # solve_individual_joint(x, xdot, t_x, u_meas, pos_map[joint_name], act_map[joint_name + '_motor'])
  solve_with_lambda(x, xdot, t_x, u_meas)
  plt.show()


def estimate_xdot_with_filtering(x, t_x):
  xdot = np.zeros((x.shape))

  # import pdb; pdb.set_trace()
  vdot = np.diff(x[:, -nv:], axis=0, prepend=x[0:1, -nv:])
  dt = np.diff(t_x, axis=0, prepend=t_x[0] - 1e-4)
  for i in range(plant.num_velocities()):
    vdot[:, i] = vdot[:, i] / dt

  filter = 50
  idx = int(filter / 2)
  filter_output = np.zeros((x.shape[0], nv))
  for i in range(idx, dt.shape[0] - idx):
    # be careful when using np.average
    filter_output[i, :] = np.average(vdot[i - idx:i+idx, :], axis=0)

  plt.figure("vdot fb")
  plt.plot(t_x[t_slice], filter_output[t_slice, 0:6])

  # xdot = np.hstack((x[:, -nv:], filter_output))
  xdot = np.hstack((np.zeros((x.shape[0], nq)), filter_output))
  return xdot

def solve_with_lambda(x, xdot, t_x, u_meas):
  # n_samples = 2000
  n_samples = 10000
  n_damping_vars = nv
  nvars = n_damping_vars

  A = np.zeros((n_samples * nv, nvars))
  b = np.zeros(n_samples * nv)

  x_samples = []
  u_samples = []
  xdot_samples = []
  t_samples = []

  for i in range(n_samples):
    t = t_x[100] + 1e-3 * i
    x_ind = np.argwhere(np.abs(t_x - t) < 1e-3)[0][0]
    x_samples.append(x[x_ind, :])
    xdot_samples.append(xdot[x_ind, :])
    u_samples.append(u_meas[x_ind, :])
    t_samples.append(t)

    plant.SetPositionsAndVelocities(context, x[x_ind, :])

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    B = plant.MakeActuationMatrix()
    g = plant.CalcGravityGeneralizedForces(context)
    Cv = plant.CalcBiasTerm(context)
    Kq = K @ (offsets - x[x_ind, :nq])

    J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
    J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
    J = np.vstack((J_l_loop_closure, J_r_loop_closure))

    JdotV_l_loop_closure = l_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV_r_loop_closure = r_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV = np.vstack((JdotV_l_loop_closure, JdotV_r_loop_closure))
    JdotV = np.reshape(JdotV, (2,))

    lambda_i_wo_vars = - np.linalg.inv(J @ M_inv @ J.T) @ ((J @ M_inv) @ (B @ u_meas[x_ind] + g - Cv + Kq) - JdotV)
    lambda_i_w_vars = - np.linalg.inv(J @ M_inv @ J.T) @ (J @ M_inv)

    row_start = i * (nv)
    row_end = (i+1) * (nv)

    # Damping indices
    A[row_start:row_end, -n_damping_vars:] = np.diag(x[x_ind, nq:])
    A[row_start:row_end, -n_damping_vars:] += np.diag((J.T @ lambda_i_w_vars) @ x[x_ind, nq:])

    # Lambda indices
    b[row_start:row_end] = M @ xdot[x_ind, -nv:] + Cv - B @ u_meas[x_ind] - g - J.T @ lambda_i_wo_vars - Kq

  x_samples = np.array(x_samples)
  xdot_samples = np.array(xdot_samples)

  prog = mp.MathematicalProgram()
  x_vars = prog.NewContinuousVariables(nvars, "sigma")
  prog.AddL2NormCost(A, b, x_vars)
  prog.AddBoundingBoxConstraint(np.full((nv,), -np.inf), np.zeros(nv), x_vars)
  solver_id = mp.ChooseBestSolver(prog)
  print(solver_id.name())
  solver = mp.MakeSolver(solver_id)
  result = solver.Solve(prog, None, None)
  print("LSTSQ cost: ", result.get_optimal_cost())
  print("Solution result: ", result.get_solution_result())
  sol = result.GetSolution()
  d_sol = sol[:n_damping_vars]

  D = np.diag(d_sol)

  f_samples = np.zeros((n_samples, nv))
  Bu_force = np.zeros((n_samples, nv))
  Cv_force = np.zeros((n_samples, nv))
  g_force = np.zeros((n_samples, nv))
  D_force = np.zeros((n_samples, nv))
  K_force = np.zeros((n_samples, nv))
  J_lambda = np.zeros((n_samples, nv))
  for i in range(n_samples):
    plant.SetPositionsAndVelocities(context, x_samples[i, :])

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    B = plant.MakeActuationMatrix()
    g = plant.CalcGravityGeneralizedForces(context)
    Cv = plant.CalcBiasTerm(context)
    J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
    J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
    J = np.vstack((J_l_loop_closure, J_r_loop_closure))
    JdotV_l_loop_closure = l_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV_r_loop_closure = r_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV = np.vstack((JdotV_l_loop_closure, JdotV_r_loop_closure))
    JdotV = np.reshape(JdotV, (2,))
    Kq = K @ (offsets - x_samples[i, :nq])
    Bu = B @ u_samples[i]
    Dv = D @ x_samples[i, nq:]

    lambda_implicit = (J @ M_inv @ J.T) @ ((J @ M_inv) @ (B @ u_samples[i] + g - Cv + Kq + Dv) - JdotV)

    # f_samples[i] = M@xdot_samples[i, -nv:] + Cv - g - Bu - Dv
    f_samples[i] = M@xdot_samples[i, -nv:] + Cv - g - Bu - Dv - J.T @ lambda_implicit - Kq
    Bu_force[i] = B @ u_samples[i]
    Cv_force[i] = Cv
    g_force[i] = g
    J_lambda[i] = J.T @ lambda_implicit
    D_force[i] = Dv
    K_force[i] = Kq + J.T @ (J @ M_inv @ J.T) @ J @ M_inv @ Kq

  plt.figure("res vs joint vel")
  pos_idx = 4 #4, 11
  if not process_with_fixed_base:
    pos_idx += 7
  vel_idx = (nv-nq) + pos_idx
  print("pos_idx = " + str(pos_idx))
  print("vel_idx = " + str(vel_idx))
  plt.plot(x_samples[:,vel_idx+nq], f_samples[:,vel_idx])
  plt.plot(x_samples[:,vel_idx+nq], Bu_force[:,vel_idx])
  plt.plot(x_samples[:,vel_idx+nq], Cv_force[:,vel_idx])
  plt.plot(x_samples[:,vel_idx+nq], J_lambda[:,vel_idx])
  plt.plot(x_samples[:,vel_idx+nq], D_force[:,vel_idx])
  plt.plot(x_samples[:,vel_idx+nq], K_force[:,vel_idx])
  plt.legend(['f', 'Bu', 'Cv', 'J', 'D', 'K'])

  print("D: ", d_sol)
  plt.figure("Forces vs time")
  plt.plot(t_samples, f_samples[:,vel_idx])
  plt.plot(t_samples, Bu_force[:,vel_idx])
  plt.plot(t_samples, Cv_force[:,vel_idx])
  plt.plot(t_samples, g_force[:,vel_idx])
  plt.plot(t_samples, D_force[:,vel_idx])
  plt.plot(t_samples, K_force[:,vel_idx])
  plt.plot(t_samples, x_samples[:,vel_idx+nq])
  plt.plot(t_samples, x_samples[:,vel_idx+nq])
  plt.legend(['f', 'Bu', 'Cv', 'g', 'D', 'K', 'pos', 'vel'])

def plot_state(x, t_x, u, t_u, u_meas, x_datatypes, u_datatypes, act_idx = 4):
  pos_indices = slice(0, nq)
  vel_indices = slice(nq, nx)
  u_indices = act_idx
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)

  plt.figure("positions: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts: " + filename)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  plt.legend(u_datatypes[u_indices])


def solve_individual_joint(x, xdot, t_x, u_meas, joint_idx, act_idx):
  n_samples = 1000

  x_samples = []
  u_samples = []
  xdot_samples = []
  t_samples = []

  nvars = 1
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
  Jv = np.zeros((n_samples, 2))

  A = np.zeros((n_samples, nvars))
  b = np.zeros(n_samples)
  for i in range(n_samples):
    t = t_x[50] + 1e-2 * i
    ind = np.argwhere(np.abs(t_x - t) < 1e-3)[0][0]
    x_samples.append(x[ind, :])
    xdot_samples.append(xdot[ind, :])
    u_samples.append(u_meas[ind, :])
    t_samples.append(t)
    plant.SetPositionsAndVelocities(context, x[ind, :])

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = inv(M)
    B = plant.MakeActuationMatrix()
    g = plant.CalcGravityGeneralizedForces(context)
    Cv = plant.CalcBiasTerm(context)
    J_l_loop_closure = l_loop_closure.EvalFullJacobian(context)
    J_r_loop_closure = r_loop_closure.EvalFullJacobian(context)
    J = np.vstack((J_l_loop_closure, J_r_loop_closure))
    JdotV_l_loop_closure = l_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV_r_loop_closure = r_loop_closure.EvalFullJacobianDotTimesV(context)
    JdotV = np.vstack((JdotV_l_loop_closure, JdotV_r_loop_closure))
    JdotV = np.reshape(JdotV, (2,))

    A[i, 0] = -x[ind, nq + joint_idx]
    b[i] = M[joint_idx, joint_idx] * xdot[ind, nq + joint_idx] - u_meas[ind, act_idx]

    qdot = x[ind, nq:]
    qddot = xdot[ind, nq:]
    K = np.zeros((nq, nq))
    K[l_knee_spring_idx, l_knee_spring_idx] = 1500
    K[r_knee_spring_idx, r_knee_spring_idx] = 1500
    K[l_heel_spring_idx, l_heel_spring_idx] = 1250
    K[r_heel_spring_idx, r_heel_spring_idx] = 1250
    # K[l_knee_spring_idx, l_knee_spring_idx] = 1000
    # K[r_knee_spring_idx, r_knee_spring_idx] = 1000
    # K[l_heel_spring_idx, l_heel_spring_idx] = 1000
    # K[r_heel_spring_idx, r_heel_spring_idx] = 1000
    K = -K
    D = np.zeros((nv, nv))
    # D[joint_idx, joint_idx] = -2.0/3

    # Compute force residuals
    lambda_implicit =            inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + K@x[ind, :nq] + D@qdot) - JdotV)
    lambda_implicit_wo_damping = inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + K@x[ind, :nq])          - JdotV)
    lambda_implicit_wo_spring =  inv(J @ M_inv @ J.T) @ (- J @ M_inv @ (-Cv + g + B @ u_meas[ind] + D@qdot)                 - JdotV)
    lambda_implicit_spring =  inv(J @ M_inv @ J.T) @ (- J @ M_inv @( K @ x[ind, :nq]))
    tau_res[i] =            M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit            - K@x[ind, :nq] - D@qdot
    tau_res_wo_damping[i] = M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit_wo_damping - K@x[ind, :nq]
    tau_res_wo_springs[i] = M @ qddot + Cv - B @ u_meas[ind] - g - J.T @ lambda_implicit_wo_spring  - D@qdot

    Jv[i] = J@qdot

    generalized_force[i] = M @ qddot
    Bu_force[i] = B@u_meas[ind]
    Cv_force[i] = Cv
    g_force[i] = g
    J_lambda[i] = J.T @ lambda_implicit
    J_lambda_spring[i] = J.T @ lambda_implicit_spring
    K_force[i] = K @ x[ind, :nq]

  x_samples = np.array(x_samples)
  xdot_samples = np.array(xdot_samples)
  u_samples = np.array(u_samples)

  plt.figure("force contribution")

  plt.plot(t_samples, generalized_force[:, joint_idx])
  plt.plot(t_samples, Bu_force[:, joint_idx])
  plt.plot(t_samples, Cv_force[:, joint_idx])
  plt.plot(t_samples, g_force[:, joint_idx])
  plt.plot(t_samples, J_lambda[:, joint_idx])
  plt.plot(t_samples, K_force[:, joint_idx])
  plt.plot(t_samples, tau_res[:, joint_idx])
  plt.plot(t_samples, J_lambda_spring[:, joint_idx])
  plt.legend(['Mqddot', 'Bu', 'Cv', 'g', 'J.T lambda', "J.T lambda_spring", 'Kq', 'residual'])

  plt.figure("Jv")

  plt.plot(t_samples, Jv)


  plt.figure("force residual position x-axis: " + filename)
  plt.plot(x_samples[:, joint_idx], tau_res[:, joint_idx], 'b.')
  # plt.plot(x_samples[:, joint_idx], tau_res_wo_damping[:, joint_idx], 'r.')
  # plt.plot(x_samples[:, joint_idx], tau_res_wo_springs[:, joint_idx], 'g.')
  plt.xlabel('joint position (rad)')
  plt.ylabel('generalized force error (Nm)')

  plt.figure("force residual velocity x-axis: " + filename)
  plt.plot(x_samples[:, nq + joint_idx], tau_res[:, joint_idx], 'b.')
  plt.xlabel('joint velocity (rad/s)')
  plt.ylabel('generalized force error (Nm)')
  # plt.plot(x_samples[:, nq + joint_idx], tau_res_wo_springs[:, joint_idx], 'g.')
  # plt.plot(x_samples[:, nq + joint_idx], tau_res_wo_damping[:, joint_idx], 'r.')
  # plt.legend()


  # plt.figure("force res vs time: " + filename)
  # plt.plot(t_samples, tau_res[:, joint_idx], '-')
  # plt.plot(t_samples, x_samples[:, nq + joint_idx], 'r-')
  # plt.xlabel('time (s)')

  prog = mp.MathematicalProgram()
  x_vars = prog.NewContinuousVariables(nvars, "sigma")
  prog.AddL2NormCost(A, b, x_vars)
  solver = OsqpSolver()
  result = solver.Solve(prog, None, None)
  print("LSTSQ cost: ", result.get_optimal_cost())
  print("Solution result: ", result.get_solution_result())
  sol = result.GetSolution()
  print(sol)

def plot_imu_data(t_x, cassie_out):
  imu_data = []
  t_imu_data = []
  for i in range(len(cassie_out)):
    if(cassie_out[i].utime / 1e6 > t_x[100] and cassie_out[i].utime / 1e6 < t_x[40100]):
      imu_data.append(np.array(cassie_out[i].pelvis.vectorNav.linearAcceleration))
      t_imu_data.append(cassie_out[i].utime / 1e6)
  imu_data = np.array(imu_data)
  t_imu_data = np.array(t_imu_data)
  plt.figure("imu")
  plt.plot(t_imu_data, imu_data)

if __name__ == "__main__":
  main()
