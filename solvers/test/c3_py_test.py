import numpy as np

from pydairlib.solvers import LCS
from pydairlib.solvers import C3MIQP
from pydairlib.solvers import C3Options

N = 10
n = 4
m = 2
k = 1
N = 10
g = 9.81
mp = 0.411
mc = 0.978
len_p = 0.6
len_com = 0.4267
d1 = 0.35
d2 = -0.35
ks = 100
Ts = 0.01

A  = np.array([[0, 0, 0, 1], [0, 0, 0, 1], [0, g*mp/mc, 0, 0,], \
	[0, (g*(mc+mp))/(len_com*mc), 0, 0]])

B = np.array([[0, 0, 1/mc, 1/(len_com*mc)]]).T

D = np.array([[0, 0], [0, 0], \
	[(-1/mc) + (len_p/(mc*len_com)), (1/mc) - (len_p/(mc*len_com))], \
	[(-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com), -((-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com))]])

E = np.array([[-1, len_p, 0, 0], [1, -len_p, 0, 0]])

F = np.array([[1/ks, 0], [0, 1/ks]])

c = np.array([d1, -d2])

d = np.zeros((4, 1))

H = np.zeros((m, k))

Q = np.array([[10, 0, 0, 0], [0, 3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

R = np.array([[1]])

G = 0.1*np.identity(n+m+k)

px = 1000
plam = 1
pu = 0
U = [[px, 0, 0, 0, 0, 0, 0], [0, px, 0, 0, 0, 0, 0 ], [0, 0, px, 0, 0, 0, 0 ], [0, 0, 0, px, 0 ,0 ,0], [0, 0, 0, 0, plam, 0, 0], [0, 0, 0, 0, 0, plam, 0  ], [0,0,0,0,0,0,0]]
U= np.asarray(U)


Qp = []
Rp = []
Gp = []
Up = []

for i in range(N):
	Qp.append(Q)
	Rp.append(R)
	Gp.append(G)
	Up.append(U)

#change with algebraic ricatti
Qp.append(Q)


cartpole = LCS(A,B,D,d,E,F,H,c, N)

options = C3Options()

opt = C3MIQP(cartpole, Qp, Rp, Gp, Up, options)

#print(Qp)

x0 = np.zeros((4,1))

x0[1] = 0.2

delta_add = np.zeros((m, 1))
w_add = np.zeros((m, 1))

input = [5]

delta = []
w = []

for i in range(N):
	delta.append(delta_add)
	w.append(w_add)


z = opt.Solve(x0, delta, w)

print(z)

x1 = cartpole.Simulate(x0, input)

print(x1)


#c_i = np.array([[d1, -d2]]).T
#delta_c = np.array([[0, 0]]).T
#U = np.identity(7)
#ret = opt.SolveSingleProjection(U, delta_c, E, F, H, c_i)


# At the moment, the Solve() function causes a crash, somewhere in the C++
#(u, delta_1, w_1) = opt.Solve(x0, delta, w)

# import pdb; pdb.set_trace()