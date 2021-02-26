from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve

import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
from pydrake.solvers.snopt import SnoptSolver

import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables


class LinearMPC(object):
    def __init__(self, Q, R, Qf):
        self.g = 9.81
        self.m = 1
        self.a = 0.25
        self.I = 0.0625
        self.Q = Q
        self.R = R
        self.Qf = Qf

        # Input limits
        self.umin = 0
        self.umax = 5.5

        self.n_x = 6
        self.n_u = 4
        self.S = np.zeros((6, 6))


    def x_d(self):
        # Nominal state
        return np.array([0, 0, 0, 0, 0, 0])


    def continuous_time_linearized_dynamics(self):
        # Dynamics linearized at the fixed point
        # This function returns A and B matrix
        A = np.zeros((6, 6))
        B = np.zeros((4, 4))
        return A, B

    def discrete_time_linearized_dynamics(self, T):
        # Discrete time version of the linearized dynamics at the fixed point
        # This function returns A and B matrix of the discrete time dynamics
        A_c, B_c = self.continuous_time_linearized_dynamics()
        A_d = np.identity(6) + A_c * T;
        B_d = B_c * T;

        return A_d, B_d

    def add_initial_state_constraint(self, prog, x, x_current):
        # TODO: impose initial state constraint.
        prog.AddBoundingBoxConstraint(x_current, x_current, x[0])

        pass

    def add_input_saturation_constraint(self, prog, x, u, N):
        # TODO: impose input limit constraint.
        # Use AddBoundingBoxConstraint
        # The limits are available through self.umin and self.umax
        for i in range(N-1):
            prog.AddBoundingBoxConstraint(self.umin - self.u_d(), self.umax - self.u_d(), u[i])
        pass

    def add_dynamics_constraint(self, prog, x, u, N, T):
        # TODO: impose dynamics constraint.
        # Use AddLinearEqualityConstraint(expr, value)
        A, B = self.discrete_time_linearized_dynamics(T)

        for i in range(N-1):
            xi = np.transpose(x[i])
            xi1 = np.transpose(x[i+1])
            ui = np.transpose(u[i])
            prog.AddLinearEqualityConstraint((A @ xi + B @ ui - xi1), np.zeros((6,1)))
        pass

    def add_cost(self, prog, x, u, N):
        # TODO: add cost.
        z6 = np.zeros((6,1))
        z2 = np.zeros((2,1))
        for i in range(N-1):
            prog.AddQuadraticCost(self.Q, z6, x[i])
            prog.AddQuadraticCost(self.R, z2, u[i])

        prog.AddQuadraticCost(self.Qf, z6, x[-1]);

        pass

    def compute_mpc_feedback(self, x_current):
        '''
        This function computes the MPC controller input u
        '''

        # Parameters for the QP
        N = 10
        T = 0.1

        # Initialize mathematical program and decalre decision variables
        prog = MathematicalProgram()
        x = np.zeros((N, 6), dtype="object")
        for i in range(N):
            x[i] = prog.NewContinuousVariables(6, "x_" + str(i))
        u = np.zeros((N-1, 2), dtype="object")
        for i in range(N-1):
            u[i] = prog.NewContinuousVariables(2, "u_" + str(i))

        # Add constraints and cost
        self.add_initial_state_constraint(prog, x, x_current)
        self.add_input_saturation_constraint(prog, x, u, N)
        self.add_dynamics_constraint(prog, x, u, N, T)
        self.add_cost(prog, x, u, N)


        # Solve the QP
        solver = OsqpSolver()
        result = solver.Solve(prog)

        u_mpc = np.zeros((4,))
        u_sol = result.GetSolution(u[0])
        u_mpc = self.u_d() + u_sol

        return u_mpc