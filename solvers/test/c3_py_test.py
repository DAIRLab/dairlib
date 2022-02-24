import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg

from pydairlib.solvers import LCS
from pydairlib.solvers import C3MIQP
from pydairlib.solvers import C3Options

n=4
m=2
k=1
N = 10
g = 9.81
mp = 0.411
mc = 0.978
len_p = 0.6
len_com = 0.4267
d1 = 0.35
d2 = -0.35
ks= 100
Ts = 0.01
A = [[0, 0, 1, 0], [0, 0, 0, 1], [0, g*mp/mc, 0, 0], [0, g*(mc+mp)/(len_com*mc), 0, 0]]
A = np.asarray(A)
B = [[0],[0],[1/mc],[1/(len_com*mc)]]
B = np.asarray(B)
D = [[0,0], [0,0], [(-1/mc) + (len_p/(mc*len_com)), (1/mc) - (len_p/(mc*len_com)) ], [(-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com)  , -((-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com))    ]]
D = np.asarray(D)
E = [[-1, len_p, 0, 0], [1, -len_p, 0, 0 ]]
E = np.asarray(E)
F = 1/ks * np.eye(2)
F = np.asarray(F)
c = [[d1], [-d2]]
c = np.asarray(c)
d = np.zeros((4,1))
H = np.zeros((2,1))
A = np.eye(n) + Ts * A
B = Ts*B
D = Ts*D
d = Ts*d

Q = np.array([[10, 0, 0, 0], [0, 3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

R = np.array([[1]])

G = 0.1*np.identity(n+m+k)
G[6,6] = 0

px = 1000
plam = 1
pu = 0
U = [[px, 0, 0, 0, 0, 0, 0], [0, px, 0, 0, 0, 0, 0 ], [0, 0, px, 0, 0, 0, 0 ], [0, 0, 0, px, 0 ,0 ,0], [0, 0, 0, 0, plam, 0, 0], [0, 0, 0, 0, 0, plam, 0  ], [0,0,0,0,0,0,0]]
U= np.asarray(U)


Qp = []
Rp = []
Gp = []
Up = []

Ap = []
Bp = []
Dp = []
dp = []
Ep = []
Fp = []
Hp = []
cp = []

for i in range(N):
	Qp.append(Q)
	Rp.append(R)
	Gp.append(G)
	Up.append(U)
	Ap.append(A)
	Bp.append(B)
	Dp.append(D)
	dp.append(d)
	Ep.append(E)
	Fp.append(F)
	Hp.append(H)
	cp.append(c)

#change with algebraic ricatti
QN = linalg.solve_discrete_are(A, B, Q, R)
Qp.append(QN)


#cartpole = LCS(A,B,D,d,E,F,H,c,N)
cartpole = LCS(Ap, Bp, Dp, dp, Ep, Fp, Hp, cp)

options = C3Options()

opt = C3MIQP(cartpole, Qp, Rp, Gp, Up, options)

#print(Qp)

x0 = np.zeros((4,1))

x0[0] = 0.1
x0[2] = 0.3

delta_add = np.zeros((n+m+k, 1))
w_add = np.zeros((n+m+k, 1))

#input = [5]

system_iter = 500

x = np.zeros((n, system_iter+1))

x[:, [0]]  = x0

for i in range(system_iter):

	delta = []
	w = []

	for j in range(N):
		delta.append(delta_add)
		w.append(w_add)

	input = opt.Solve(x[:, [i]], delta, w)

	prediction = cartpole.Simulate(x[:, [i]], input)
	x[:, [i+1]] = np.reshape(prediction, (n,1))

	if i % 50 == 0:
		print(i)



dt = 0.01
time_x = np.arange(0, system_iter * dt + dt, dt)
plt.plot(time_x, x.T)
plt.show()



#cartpole.Simulate(x0, input)
#print(x1)
#c_i = np.array([[d1, -d2]]).T
#delta_c = np.array([[0, 0]]).T
#U = np.identity(7)
#ret = opt.SolveSingleProjection(U, delta_c, E, F, H, c_i)
# At the moment, the Solve() function causes a crash, somewhere in the C++
#(u, delta_1, w_1) = opt.Solve(x0, delta, w)
# import pdb; pdb.set_trace()