import numpy as np

from pydairlib.solvers import C3MIQP
from pydairlib.solvers import C3Options

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

options = C3Options()

opt = C3MIQP(A, B, D, d, E, F, H, c, Q, R, G, N, options)

x0 = np.zeros((4,1))

delta = [np.zeros((m, 1))] * N

w = [np.zeros((m, 1))] * N

c_i = np.array([[d1, -d2]]).T
delta_c = np.array([[0, 0]]).T
U = np.identity(7)

# import pdb; pdb.set_trace()

ret = opt.SolveSingleProjection(U=U, delta_c=delta_c, E=E, F=F, H=H, c=c_i)

print(ret)

# (u, delta_1, w_1) = opt.Solve(x0, delta, w)