import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import process_lcm_log
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import mathematicalprogram as mp
import pydairlib.lcm_trajectory
import pydairlib.multibody
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

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  plant.mutable_gravity_field().set_gravity_vector(

    -9.81 * np.array([0, 0, 1]))
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

  filename = sys.argv[1]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  matplotlib.rcParams["savefig.directory"] = path

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log = process_lcm_log.process_log(log, pos_map, vel_map, act_map)

  # Will need to manually select the data range
  t_start = t_x[100]
  t_end = t_x[-100]
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  # plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes)
  # plt.show()
  sample_times = [46.0, 55.6, 65.0, 69.9, 74.8, 82.7, 83.15, 92.6]
  # sample_times = [46.0]

  solve_for_k(x, t_x, u, t_u)
  solve_with_lambda(x, t_x, u, t_u)


def solve_for_k(x, t_x, u, t_u):
  n_samples = len(sample_times)
  n_samples_per_iter = 5
  # K is a diagonal matrix, do the springs act directly on the knees? If so the non-zero values will not
  # be on the diagonals
  n_k = 4
  nvars = n_k
  prog = mp.MathematicalProgram()

  A = np.zeros((n_samples * n_samples_per_iter * nv, nvars))
  b = np.zeros(n_samples * n_samples_per_iter * nv)

  for i in range(n_samples):
    for j in range(n_samples_per_iter):
      delta_t = 1e-3 * j
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

      J = np.vstack((J_lh, J_lt, J_rh, J_rt))

      row_start = i * (nv * n_samples_per_iter) + nv * j
      row_end = i * (nv * n_samples_per_iter) + nv * (j + 1)

      lambda_i_wo_k = - np.linalg.inv(J @ M_inv @ J.T) @ (J @ M_inv @ B @ u[u_ind] + J @ M_inv @ g)
      lambda_i_w_k = - np.linalg.inv(J @ M_inv @ J.T) @ (J @ M_inv)

      """ 
      Manual indexing version
      """

      # A[row_start + l_knee_idx - 1, 0] = x[x_ind, l_knee_spring_idx]
      # A[row_start + r_knee_idx - 1, 1] = x[x_ind, r_knee_spring_idx]
      # A[row_start + l_heel_idx - 1, 2] = x[x_ind, l_heel_spring_idx]
      # A[row_start + r_heel_idx - 1, 3] = x[x_ind, r_heel_spring_idx]
      #
      # print("i: ", i)
      # print("j: ", j)
      # A[row_start:row_end, 0] += (J.T @ lambda_i_w_k)[:, l_knee_spring_idx - 1] * x[x_ind, l_knee_spring_idx]
      # A[row_start:row_end, 1] += (J.T @ lambda_i_w_k)[:, r_knee_spring_idx - 1] * x[x_ind, r_knee_spring_idx]
      # A[row_start:row_end, 2] += (J.T @ lambda_i_w_k)[:, l_heel_spring_idx - 1] * x[x_ind, l_heel_spring_idx]
      # A[row_start:row_end, 3] += (J.T @ lambda_i_w_k)[:, r_heel_spring_idx - 1] * x[x_ind, r_heel_spring_idx]
      # # A[row_start:row_end, 1] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, r_knee_spring_idx] * x[
      # #   x_ind, r_knee_spring_idx]
      # # A[row_start:row_end, 2] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, l_heel_spring_idx] * x[
      # #   x_ind, l_heel_spring_idx]
      # # A[row_start:row_end, 3] += M_inv @ J.T @ np.linalg.inv((J @ M_inv @ J.T)) @ J @ M_inv[:, l_knee_spring_idx] * x[
      # #   x_ind, r_heel_spring_idx]
      # b[row_start:row_end] = (- B @ u[u_ind] - g - J.T @ lambda_i_wo_k)
      # # import pdb;
      # # pdb.set_trace()

      """ 
      Matrix version
      """

      I_nv = np.eye(nv, nv)
      Q_tilde = np.zeros((nv, n_k))
      Q_tilde[l_knee_spring_idx - 1, 0] = x[x_ind, l_knee_spring_idx]
      Q_tilde[r_knee_spring_idx - 1, 1] = x[x_ind, r_knee_spring_idx]
      Q_tilde[l_heel_spring_idx - 1, 2] = x[x_ind, l_heel_spring_idx]
      Q_tilde[r_heel_spring_idx - 1, 3] = x[x_ind, r_heel_spring_idx]
      A[row_start:row_end, :] = (I_nv + J.T @ lambda_i_w_k) @ Q_tilde
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
  lambdas = sol[n_k:nvars]
  print("K: ", K)
  print("lambdas: ", lambdas)


def solve_with_lambda(x, t_x, u, t_u):
  n_samples = len(sample_times)
  n_samples_per_iter = 10
  n_lambda_vars = 12
  # K is a diagonal matrix, do the springs act directly on the knees? If so the non-zero values will not
  # be on the diagonals
  n_k = 4
  nvars = n_k + n_samples * n_lambda_vars
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

      J = np.vstack((J_lh, J_lt, J_rh, J_rt))

      row_start = i * (nv * n_samples_per_iter) + nv * j
      row_end = i * (nv * n_samples_per_iter) + nv * (j + 1)

      A[row_start:row_end, 0] = M_inv[:, l_knee_spring_idx - 1] * x[x_ind, l_knee_spring_idx]
      A[row_start:row_end, 1] = M_inv[:, r_knee_spring_idx - 1] * x[x_ind, r_knee_spring_idx]
      A[row_start:row_end, 2] = M_inv[:, l_heel_spring_idx - 1] * x[x_ind, l_heel_spring_idx]
      A[row_start:row_end, 3] = M_inv[:, l_knee_spring_idx - 1] * x[x_ind, r_heel_spring_idx]
      A[row_start:row_end, n_k + i * n_lambda_vars: n_k + (i + 1) * n_lambda_vars] = M_inv @ J.T
      b[row_start:row_end] = M_inv @ (- B @ u[u_ind] - g)

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
  lambdas = sol[n_k:nvars]
  print("K: ", K)
  print("lambdas: ", lambdas)


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes):
  pos_indices = slice(0, 7)
  vel_indices = slice(23, 29)
  u_indices = slice(6, 8)
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)

  plt.figure("positions: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, pos_map["knee_joint_right"]])
  plt.plot(t_x[t_slice], x[t_slice, pos_map["ankle_spring_joint_right"]])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
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
