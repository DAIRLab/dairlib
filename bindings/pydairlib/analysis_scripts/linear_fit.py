import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver




def main():

  data_folder = '/home/yangwill/Documents/research/projects/cassie/hardware/sysid_data/'
  filename = sys.argv[1]
  # x = np.load(data_folder + filename + "_x.npy")
  # f = np.load(data_folder + filename + "_f.npy")
  x = np.load(data_folder + "x_stack.npy")
  f = np.load(data_folder + "f_stack.npy")
  nq = 16
  nv = 16

  # for i in range(nv):
  #   plt.figure(i)
  #   plt.plot(x[:, i], f[:, i])
  #   plt.plot(x[:, i + nv], f[:, i])
  # plt.show()

  # D_sol1 = np.load(data_folder + "lcmlog-01_D_sol.npy")
  # K_sol1 = np.load(data_folder + "lcmlog-01_K_sol.npy")
  # D_sol2 = np.load(data_folder + "lcmlog-02_D_sol.npy")
  # K_sol2 = np.load(data_folder + "lcmlog-02_K_sol.npy")
  # D_sol3 = np.load(data_folder + "lcmlog-03_D_sol.npy")
  # K_sol3 = np.load(data_folder + "lcmlog-03_K_sol.npy")



  n_vars = nv*nv + nv*nv
  n_samples = x.shape[0]
  # n_samples = 1000
  sample = np.arange(0, n_samples)

  prog = mp.MathematicalProgram()
  # D = np.zeros((nv, nv), dtype='object')
  # K = np.zeros((nv, nv), dtype='object')
  # for i in range(nv):
  #   D[i, i] = prog.NewContinuousVariables(1, "D_" + str(i) + ',' + str(i))
  #   K[i, i] = prog.NewContinuousVariables(1, "K_" + str(i) + ',' + str(i))

  # Diag version
  D = prog.NewContinuousVariables(nv, "D")
  K = prog.NewContinuousVariables(nv, "D")

  D_flat = D.flatten()
  K_flat = K.flatten()
  D_regularization_factor = 0.1
  K_regularization_factor = 1e-8
  prog.AddQuadraticCost(K_regularization_factor * K_flat.T @ K_flat)
  prog.AddQuadraticCost(D_regularization_factor * D_flat.T @ D_flat)
  for i in range(n_samples):
    for j in range(nv):
      # Square version
      # res = D[j] @ x[i, -nv:] + K[j] @ x[i, :nv] - f[i, j]
      # Diag version
      res = D[j] * x[i, nq + j] + K[j] * x[i, j] - f[i, j]
      prog.AddQuadraticCost(res ** 2)

  solver = OsqpSolver()
  result = solver.Solve(prog, None, None)
  print("LSTSQ cost: ", result.get_optimal_cost())
  D_sol = result.GetSolution(D)
  K_sol = result.GetSolution(K)
  np.save(data_folder + filename + "_D_sol_diag", D_sol)
  np.save(data_folder + filename + "_K_sol_diag", K_sol)

  import pdb; pdb.set_trace()

  f_correction = np.zeros((n_samples, nv))

  for i in range(n_samples):
    f_correction[i] = f[i] - D_sol @ x[i, -nv:] - K_sol @ x[i, :nv]

  plt.plot(sample, f_correction)
  plt.plot(sample, f[:n_samples])
  plt.show()
  import pdb; pdb.set_trace()

if __name__ == "__main__":
  main()