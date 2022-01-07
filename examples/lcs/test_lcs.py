import numpy as np
from lcs import LCS
import time
import matplotlib.pyplot as plt

# Defining main function
def main():
	nx = 6
	nu = 4
	nl = 6
	# np.random.seed	(5)
	# A = np.eye(nx)
	# B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
	# C = np.random.randn(nx, nlambda)
	# d = np.array([[0, 0, 0, 0]]).T
	# D = np.random.randn(nlambda, nx)
	# E = np.random.randn(nlambda, nu)
	# F_sqrt = np.random.randn(nlambda, nlambda)
	# F = F_sqrt @ F_sqrt.T
	# c = np.array([[1, 2]]).T

  # Alp's finger gating example
	g = 9.81
	mu = 1
	h = 0.1
	A = [[1,h,0,0,0,0],[0, 1, 0, 0, 0, 0], [0, 0, 1, h, 0, 0 ], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, h], [0, 0, 0, 0, 0, 1]]
	A = np.asarray(A)
	B = [[0,0,0,0], [0, 0, 0, 0], [h*h, 0, 0, 0], [h, 0, 0, 0], [0, h*h, 0, 0], [0, h, 0, 0]]
	B = np.asarray(B)
	C = [[0, h*h, -h*h, 0, h*h, -h*h], [0, h, -h, 0, h, -h], [0, -h*h, h*h, 0, 0, 0], [0, -h, h, 0, 0, 0], [0, 0, 0, 0, -h*h, h*h], [0, 0, 0, 0, -h, h]]
	C = np.asarray(C)
	D = [[0, 0, 0, 0, 0, 0], [0, 1, 0, -1, 0, 0], [0, -1, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, -1], [0, -1, 0, 0, 0, 1]]
	D = np.asarray(D)
	F = [[0, -1, -1, 0, 0, 0], [1, 2*h, -2*h, 0, h, -h], [1, -2*h, 2*h, 0, -h, h], [0, 0, 0, 0, -1,-1], [0, h, -h, 1, 2*h, -2*h], [0, -h, h, 1, -2*h, 2*h]]
	F = np.asarray(F)
	c = [[0],[-h*g], [h*g], [0], [-h*g], [h*g]]
	c = np.asarray(c)
	d = [[-g*h*h],[-g*h],[0],[0],[0],[0]]
	d = np.asarray(d)
	E = [[0, 0, mu, 0], [-h, 0, 0, 0], [h, 0, 0, 0], [0, 0, 0, mu], [0, -h, 0, 0], [0, h, 0, 0]]
	E = np.asarray(E)

	
	print(np.linalg.eig(F + F.T))
	

	lcs = LCS(A, B, C, d, D, E, F, c)

	# u = np.random.randn(nu, 1)
	# x = np.random.randn(nx, 1)
	# xn, l, result = lcs.Step(x, u)

	N = 10
	Q = 10*np.eye(nx)
	R = .1 * np.eye(nu)
	Q_N = Q
	#x0 = np.random.randn(nx)
	x0 = np.array([np.random.rand()*2 - 8, 0, np.random.rand()+2, 0, np.random.rand()+3, 0])
	

	T = 40
	x = np.zeros((nx, T))
	u = np.zeros((nu, T))
	l_pred = np.zeros((nl, T))
	l = np.zeros((nl, T))

	x[:,0] = x0

	# import pdb
	# pdb.set_trace()

	for i in range(1, T):
		[x_sol, u_sol, l_sol] = lcs.MPC_MIQP(x[:,i-1], N, Q, R, Q_N)
		
		u[:,i] = u_sol[:,0]
		l_pred[:,i] = l_sol[:,0]
		
		x_n, l_sim, result = lcs.Step(x[:,i-1], u[:,i])
		print(result)

		x[:,i] = np.reshape(x_n, nx)
		l[:,i] = np.reshape(l_sim, nl)

	fig = plt.figure()
	plt.plot(x.T)
	plt.title('x')
	fig.axes[0].legend(['x0','x1','x2','x3','x4','x5'])
	fig = plt.figure()
	plt.plot(u.T)
	plt.title('u')
	fig.axes[0].legend(['u0','u1','u2','u3'])
	fig = plt.figure()
	plt.plot(l.T)
	plt.title('lambda')
	fig.axes[0].legend(['l0','l1','l2','l3','l4','l5'])
	plt.show()
	
if __name__=="__main__":
	main()