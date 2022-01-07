from pydairlib.solvers.pathlcp import SolveLCP
import numpy as np
from pydrake.solvers.mathematicalprogram import (
		MathematicalProgram,
		LinearConstraint,
		Solve,
		)
from pydrake.solvers.osqp import OsqpSolver

class LCS:
	"""
			A Linear Complementarity System
			\dot x = Ax + Bu + C \lambda + d
			0 <= \lambda \perp Dx + Eu + F\lambda + c \geq 0
	"""
	def __init__(self, A, B, C, d, D, E, F, c):
		self.A = A
		self.B = B
		self.C = C
		self.D = D
		self.d = d
		self.E = E
		self.F = F
		self.c = c

		self.nx = A.shape[0]
		self.nu = B.shape[1]
		self.nl = C.shape[1]


	"""
		Solve for the next state
	"""
	def Step(self, x, u):
		l_guess = np.zeros((self.C.shape[1], 1))

		if len(x.shape) == 1:
			x = np.reshape(x, (self.nx, 1))
		if len(u.shape) == 1:
			u = np.reshape(u, (self.nu, 1))

		[result, l] = SolveLCP(self.F, self.D @ x + self.E @ u + self.c)
		
		xn = self.A @ x + self.B @ u + self.C @ l + self.d

		return xn, l, result


	"""
		Solve the MIQP MPC problem.
	"""
	def MPC_MIQP(self, x0, N, Q, R, Q_N):
		if len(x0.shape) == 1:
			x0 = np.reshape(x0, (self.nx, 1))

		prog = MathematicalProgram()
		x = []
		u = []
		l = []
		compl = []
		for i in range(0, N):
			x.append(np.reshape(prog.NewContinuousVariables(self.nx, "x(" + str(i) + ")"), (self.nx, 1)))
			u.append(np.reshape(prog.NewContinuousVariables(self.nu, "u(" + str(i) + ")"), (self.nu, 1)))
			l.append(np.reshape(prog.NewContinuousVariables(self.nl, "l(" + str(i) + ")"), (self.nl, 1)))
			compl.append(np.reshape(prog.NewBinaryVariables(self.nl, "compl(" + str(i) + ")"), (self.nl, 1)))

			
			# TODO: move this out
			# also, make the problem construction it's own class
			prog.AddBoundingBoxConstraint(np.zeros(self.nl), np.full(self.nl, np.inf), l[i])
			prog.AddBoundingBoxConstraint(1, 3, x[i][2])
			prog.AddBoundingBoxConstraint(3, 5, x[i][4])

		M = 100 # Big-M

		for i in range(0,N):
			if i == 0:
				x_prev = x0
			else:
				x_prev = x[i-1]

			dynamics = self.A @ x_prev 	+ self.B @ u[i] + self.C @ l[i] + self.d - x[i]
			# import pdb; pdb.set_trace()
			self.AddConstraints(prog, dynamics, 0, 0)

			gap = self.D @ x_prev + self.E @ u[i] + self.F @ l[i] + self.c
			self.AddConstraints(prog, gap, 0, np.inf)

			#     gap \perp l
			# Convert to
			# 		0 <= gap <= M*compl
			#			0 <= l <= (M-1)*compl
			self.AddConstraints(prog, M * compl[i] - gap, 0, np.inf)
			self.AddConstraints(prog, M * (1-compl[i]) - l[i], 0, np.inf)

			# costs
			prog.AddQuadraticCost(Q, np.zeros(self.nx), x[i], True)
			prog.AddQuadraticCost(R, np.zeros(self.nu), u[i], True)
		
		prog.AddQuadraticCost(Q_N, np.zeros(self.nx), x[-1], True)


		result = Solve(prog)

		x_sol = result.GetSolution(np.reshape(np.asarray(x), (N, self.nx))).T
		u_sol = result.GetSolution(np.reshape(np.asarray(u), (N, self.nu))).T
		l_sol = result.GetSolution(np.reshape(np.asarray(l), (N, self.nl))).T
		compl_sol = result.GetSolution(np.reshape(np.asarray(compl), (N, self.nl))).T

		return x_sol, u_sol, l_sol

	"""
		Solve the QP relaxation of the  MPC problem.
	"""
	def MPC_QP(self, x0, N, Q, R, Q_N):
		if len(x0.shape) == 1:
			x0 = np.reshape(x0, (self.nx, 1))

		prog = MathematicalProgram()
		x = []
		u = []
		l = []
		phi = []
		gamma = 1e-3
		eps = 10000

		for i in range(0, N):
			x.append(np.reshape(prog.NewContinuousVariables(self.nx, "x(" + str(i) + ")"), (self.nx, 1)))
			u.append(np.reshape(prog.NewContinuousVariables(self.nu, "u(" + str(i) + ")"), (self.nu, 1)))
			l.append(np.reshape(prog.NewContinuousVariables(self.nl, "l(" + str(i) + ")"), (self.nl, 1)))
			phi.append(np.reshape(prog.NewContinuousVariables(self.nl, "phi(" + str(i) + ")"), (self.nl, 1)))

			
			# TODO: move this out
			# also, make the problem construction it's own class
			prog.AddBoundingBoxConstraint(np.zeros(self.nl), np.full(self.nl, np.inf), l[i])
			prog.AddBoundingBoxConstraint(1, 3, x[i][2])
			prog.AddBoundingBoxConstraint(3, 5, x[i][4])

		for i in range(0,N):
			if i == 0:
				x_prev = x0
			else:
				x_prev = x[i-1]

			dynamics = self.A @ x_prev 	+ self.B @ u[i] + self.C @ l[i] + self.d - x[i]
			# import pdb; pdb.set_trace()
			self.AddConstraints(prog, dynamics, 0, 0)

			gap = self.D @ x_prev + self.E @ u[i] + self.F @ l[i] + self.c
			# self.AddConstraints(prog, gap, 0, np.inf)

			#     gap \perp l
			# Convert to
			# 		0 <= gap <= M*compl
			#			0 <= l <= (M-1)*compl
			# self.AddConstraints(prog, M * compl[i] - gap, 0, np.inf)
			# self.AddConstraints(prog, M * (1-compl[i]) - l[i], 0, np.inf)

			# costs
			prog.AddQuadraticCost(Q, np.zeros(self.nx), x[i], True)
			prog.AddQuadraticCost(R, np.zeros(self.nu), u[i], True)
			prog.AddQuadraticCost(1/eps * np.squeeze(phi[i]).dot(np.squeeze(l[i])), True)

			tmp = np.squeeze(gap - phi[i])
			
			prog.AddQuadraticCost(1/(eps * gamma) * tmp.dot(tmp), True)
		
		prog.AddQuadraticCost(Q_N, np.zeros(self.nx), x[-1], True)


		solver = OsqpSolver()
		result = solver.Solve(prog)


		x_sol = result.GetSolution(np.reshape(np.asarray(x), (N, self.nx))).T
		u_sol = result.GetSolution(np.reshape(np.asarray(u), (N, self.nu))).T
		l_sol = result.GetSolution(np.reshape(np.asarray(l), (N, self.nl))).T

		return x_sol, u_sol, l_sol


	def AddConstraints(self, prog, constraints, lb, ub):
		for constraint in np.reshape(constraints, constraints.size):
			prog.AddConstraint(constraint, lb, ub)