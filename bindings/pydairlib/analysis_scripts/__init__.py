import numpy as np
from math import *

import matplotlib.pyplot as plt


def double_integrator_value_iteration():
  N = 21
  xmax = 2
  x = np.linspace(-xmax, xmax, N)
  dt = .01
  xdot = np.linspace(-xmax, xmax, N)

  umax = 2
  M = 21 * umax
  u = np.linspace(-umax, umax, M)
  [X, XDOT] = np.meshgrid(x, xdot)

  f =

  @(x, xdot, u)

  u

  Q = [5 1
       1 1]
  R = 1
  A = [0 1
       0 0]
  B = [0
       1]

  N_x = N ** 2
  cost =

  @(x, xdot, u)

  0.5 * [x xdot] @ Q @ [x xdot].T + 0.5 * u @ R @ u

  # compute discretizations of state transition and cost
  [T, C] = discretize_system(f, cost, x, xdot, u, dt)

  # vectors containg discretized V^*, u^*. V^*(x_i) is stored in V(i), and
  # u^*(x_i) in uopt(i)
  V = np.zeros(N_x, 1)
  uopt = np.zeros(N_x, 1)

  err = 1
  iteration = 1
  gamma = .9995
  while err > 1e-5:
    # TODO: calculate \tilde V_{n+1} and store in V_next, calculate
    # \tilde u_n and store in uopt

    err = max(max(abs(V - V_next))) / max(max(V))
    V = V_next

    if np.mod(iteration, 20) == 1:
      plt.figure("Cost to go")
      plt.surf(X, XDOT, np.reshape(V, [N N]))
      # colorbar
      plt.xlabel('$x$', 'FontSize', 24, 'Interpreter', 'Latex')
      plt.ylabel('$\dot x$', 'FontSize', 24, 'Interpreter', 'Latex')

      plt.figure("Controller")
      plt.surf(X, XDOT, np.reshape(uopt, [N N]))
      plt.view(60, 60)
      plt.xlabel('$x$', 'FontSize', 24, 'Interpreter', 'Latex')
      plt.ylabel('$\dot x$', 'FontSize', 24, 'Interpreter', 'Latex')

      # refresh
      # pause(.01)
    end
    iteration += 1
    display(sprintf('iteration: %d, err: %f%', iteration, err))

  return x, xdot, V, u_opt
