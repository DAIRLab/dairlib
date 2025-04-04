"""Test script to inspect LCS, optimization problem structure, and costs for
debugging the jack falling through the table observation.

The ordering of a z vector is (x, lambda, u) where x is (q, v) and in the order:
    - q:  x_ee, y_ee, z_ee,
          qw_jack, qx_jack, qy_jack, qz_jack,
          x_jack, y_jack, z_jack
    - v:  vx_ee, vy_ee, vz_ee,
          wx_ee, wy_ee, wz_ee,
          vx_jack, vy_jack, vz_jack
    - lambda:  ee_1, ee_2, ee_3, ee_4,
               capsule_1_1, capsule_1_2, capsule_1_3, capsule_1_4,
               capsule_2_1, capsule_2_2, capsule_2_3, capsule_2_4,
               capsule_3_1, capsule_3_2, capsule_3_3, capsule_3_4
    - u:  fx, fy, fz
"""

import numpy as np
import pdb


N_Q = 10
N_V = 9
N_X = N_Q + N_V
N_U = 3
N_C = 4
N_FRICTION_DIRS = 2
N_LAMBDA = 2*N_C*N_FRICTION_DIRS    # Assumes Anitescu.
N_HORIZON = 5

DT = 0.1
MU = 0.4615     # NOTE:  This is a result of the EE and ground having mu=1 and
                # the jack having mu=0.3.  The result for all contact pairs is
                # the effective mu = (2 * mu_1 * mu_2) / (mu_1 + mu_2) = 0.4615.
Q_VECTOR = [1000, 1000, 1000,        # end effector position
            0, 0, 0, 0,              # object orientation
            10000, 10000, 10000,     # object position
            10, 10, 10,              # end effector linear velocity
            0, 0, 0,                 # object angular velocity
            0.1, 0.1, 0.1]           # object linear velocity
R_VECTOR = [0.15, 0.15, 0.1]         # input cost
X_DESIRED = np.array([
    0.22687204,
    0.49502927,
    0.14105015,
    1.0,
    0.0,
    0.0,
    0.0,
    0.43777567,
    0.1797093,
    0.0625,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
])

# ====================== Values copied from running C3 ======================= #
# Values copied over from z after SolveQP, before SolveProj.
zs = np.array([
    [0.1948, 0.4805, 0.1845, 0.9136, 0.3196, -0.3491, -0.0402, 0.2268,
     0.495, 0.061, -0.3978, 0.1356, 0.0199, -2.35E-17, 2.20E-17, -1.37E-18,
     8.24E-18, -2.65E-19, -5.91E-17, -44.1602, -20.6439, -36.6287, -28.1754,
     44.6482, 27.5952, 41.5244, 30.719, 41.5212, 30.7159, 27.592, 44.6451,
     44.6443, 27.5912, 41.5205, 30.715, 0.09295, 0.0324773, 0.371567],
    [0.21457,  0.48946,  0.15762,  1.06108,  0.0824161,  -0.182198,  -0.0236754,
     0.229088,  0.494383,  0.00700172,  0.197703,  0.0895988,  -0.268801,
     -4.95869,  3.92756,  -0.159081,  0.0228767,  -0.00617439,  -0.539983,
     -32.7974,  -16.5355,  -27.3414,  -21.9916,  32.9056,  21.5472,  30.7984,
     23.6545,  30.7957,  23.6518,  21.5445,  32.903,  32.9023,  21.5439,
     30.7951,  23.6511, 0.0120332, 0.0244512, 0.344599],
    [0.222124,  0.492881,  0.147354,  1.32031,  -0.332404,  0.113527,
     0.00170177,  0.23296,  0.49353,  -0.111789,  0.075535,  0.0342134,
     -0.102659,  -8.65421,  6.9648,  -0.314902,  0.0387241,  -0.00852229,
     -1.1879,  -22.041,  -11.8717,  -18.4627,  -15.45,  21.953,  15.144,
     20.6726,  16.4244,  20.6707,  16.4225,  15.1421,  21.9511,  21.9506,
     15.1417,  20.6703,  16.422, 0.0112736, 0.0195833, 0.262316],
    [0.225001,  0.494184,  0.143443,  1.65605,  -0.867537,  0.498889,
     0.0308746,  0.237826,  0.492661,  -0.305946,  0.0287732,  0.013032,
     -0.0391066,  -11.1458,  9.08166,  -0.443352,  0.0486577,  -0.00868858,
     -1.94157,  -12.3877,  -7.06417,  -10.4233,  -9.02863,  12.2551,
     8.85345,  11.6064,  9.50214,  11.6054,  9.50116,  8.85245,  12.2541,
     12.2539,  8.85222,  11.6052,  9.50091, 0.00698898, 0.0121809, 0.191007],
    [0.22608,  0.494673,  0.141978,  2.03551,  -1.47077,  0.936209,  0.0610719,
     0.243213,  0.491856,  -0.585247,  0.0107893,  0.00488643,  -0.014657,
     -12.5504,  10.3104,  -0.527772,  0.0538738,  -0.00805371,  -2.79301,
     -4.65068,  -2.78511,  -3.92954,  -3.50625,  4.57236,  3.43918,  4.35302,
     3.65851,  4.35302,  3.65851,  3.43918,  4.57236,  4.57236,  3.43918,
     4.35302,  3.65851, 0.00252097, 0.00482437, 0.133622]
])

# LCS setup for the initial condition.
A = np.array([
    [1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0.1,  0,  0,  0,  0,  0,  0,  0,  0],
    [0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0.1,  0,  0,  0,  0,  0,  0,  0],
    [0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0.1,  0,  0,  0,  0,  0,  0],
    [0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  -0.01598,  0.017455,  0.00201,  0,  0,  0],
    [0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0.04568,  -0.00201,  0.017455,  0,  0,  0],
    [0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0.00201,  0.04568,  0.01598,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  -0.017455,  -0.01598,  0.04568,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0.1,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0.1,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0.1],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0],
    [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1]
])
B = np.array([
    [1,  0,  0],
    [0,  1,  0],
    [0,  0,  1],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [10,  0,  0],
    [0,  10,  0],
    [0,  0,  10],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0],
    [0,  0,  0]
])
D = np.array([
    [0.000187037,  0.000187037,  0.00127042,  -0.000896344,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [-0.000583581,  0.00154408,  0.000462589,  0.000497906,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [0.00252021,  0.00207553,  0.00221338,  0.00238236,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [-0.000860438,  -0.00135969,  -0.0014974,  -0.000722723,  -0.000659272,  -0.00134868,  -0.00143878,  -0.000569175,  4.11E-05,  0.000887062,  5.22E-05,  0.000875986,  0.000910057,  0.000173953,  0.000171446,  0.000912565],
    [0.000464786,  0.00281894,  0.00144226,  0.00184146,  -2.29E-05,  0.00264496,  0.00113853,  0.00148353,  0.000514317,  0.000654401,  0.00133449,  -0.000165768,  -0.00302907,  -0.000768587,  -0.00151425,  -0.00228341],
    [-0.00182012,  -0.00102488,  -0.00260496,  -0.000240039,  -0.00171951,  -0.00114504,  -0.00270506,  -0.000159493,  0.000673313,  0.00303108,  0.00160504,  0.00209935,  -0.000518529,  -0.000316597,  -0.00118009,  0.000344964],
    [-5.34E-05,  0.00041062,  5.74E-05,  0.000299745,  -0.000232526,  0.000321035,  -0.000155686,  0.000244195,  -0.000823487,  -0.000959826,  -0.00214243,  0.000359113,  0.00110337,  0.000592227,  0.00210569,  -0.0004101],
    [-6.23E-06,  -6.23E-06,  -4.23E-05,  2.99E-05,  -4.85E-12,  -4.85E-12,  3.62E-05,  -3.62E-05,  3.62E-05,  -3.62E-05,  3.77E-12,  3.77E-12,  2.64E-12,  2.64E-12,  3.62E-05,  -3.62E-05],
    [1.95E-05,  -5.15E-05,  -1.54E-05,  -1.66E-05,  -3.62E-05,  3.62E-05,  4.85E-12,  4.85E-12,  3.77E-12,  3.77E-12,  3.62E-05,  -3.62E-05,  -3.62E-05,  3.62E-05,  3.78E-12,  3.78E-12],
    [-8.40E-05,  -6.92E-05,  -7.38E-05,  -7.94E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05,  7.85E-05],
    [0.00187037,  0.00187037,  0.0127042,  -0.00896344,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [-0.00583581,  0.0154408,  0.00462589,  0.00497906,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [0.0252021,  0.0207553,  0.0221338,  0.0238236,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [0.0121685,  0.0532943,  0.0315279,  0.0339349,  0.0038071,  0.0507287,  0.0272679,  0.0272679,  0.0145484,  0.0145484,  0.0380072,  -0.00891041,  -0.0653448,  -0.0184335,  -0.0418892,  -0.0418892],
    [-0.0370622,  -0.0312281,  -0.0561913,  -0.012099,  -0.0325543,  -0.0325543,  -0.0560151,  -0.00909359,  0.0164484,  0.0633658,  0.0399071,  0.0399071,  -0.00729786,  -0.00729787,  -0.0307534,  0.0161577],
    [-0.00948527,  0.0184292,  -0.00635236,  0.0152963,  -0.0150239,  0.0150238,  -0.0125842,  0.0125841,  -0.00671408,  0.00671413,  -0.0184172,  0.0184172,  -0.00336792,  0.00336798,  0.0193318,  -0.0193318],
    [-6.23E-05,  -6.23E-05,  -0.000423472,  0.000298781,  -4.85E-11,  -4.85E-11,  0.000362271,  -0.000362271,  0.000362271,  -0.000362271,  3.77E-11,  3.77E-11,  2.64E-11,  2.64E-11,  0.000362271,  -0.000362271],
    [0.000194527,  -0.000514692,  -0.000154196,  -0.000165969,  -0.000362271,  0.000362271,  4.85E-11,  4.85E-11,  3.77E-11,  3.77E-11,  0.000362271,  -0.000362271,  -0.000362271,  0.000362271,  3.78E-11,  3.78E-11],
    [-0.000840069,  -0.000691845,  -0.000737793,  -0.000794121,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986,  0.000784986]
])
d = np.array([
    0, 0, -0.0981, 0, 0, 0, 0, 0, 0, -0.0981, 0, 0, -0.981, 0, 0, 0, 0, 0, -0.981
])
Jn = np.array([
    [0.0794225, 0.20393, 0.975758, 0.0393339, -0.0410329, 0.00537414, -0.0794225, -0.20393, -0.975758],
    [0, 0, 0, 0.0327683, -0.0391212, 4.45E-09, -6.18E-08, 6.18E-08, 1],
    [0, 0, 0, 0.0174831, 0.0479572, -3.15E-09, 4.81E-08, 4.81E-08, 1],
    [0, 0, 0, -0.050339, -0.00877001, 2.12E-09, 3.37E-08, 4.81E-08, 1]
])
Jt = np.array([
    [0, -0.978851, 0.204577, -0.0535446, -0.00759576, -0.0363439, 0, 0.978851, -0.204577],
    [0, 0.978851, -0.204577, 0.0535446, 0.00759576, 0.0363439, 0, -0.978851, 0.204577],
    [0.996841, -0.016248, -0.0777427, -0.00313389, -0.0574069, -0.0281859, -0.996841, 0.016248, 0.0777427],
    [-0.996841, 0.016248, 0.0777427, 0.00313389, 0.0574069, 0.0281859, 0.996841, -0.016248, -0.0777427],
    [0, 0, 0, -0.0610905, -2.42E-09, -0.0391212, 0, -1, 6.18E-08],
    [0, 0, 0, 0.0610905, 2.42E-09, 0.0391212, 0, 1, -6.18E-08],
    [0, 0, 0, 2.03E-09, -0.0610905, -0.0327683, 1, 3.76E-15, 6.18E-08],
    [0, 0, 0, -2.03E-09, 0.0610905, 0.0327683, -1, -3.76E-15, -6.18E-08],
    [0, 0, 0, -8.40E-10, -0.0610853, -0.0174831, 1, 0, -4.81E-08],
    [0, 0, 0, 8.40E-10, 0.0610853, 0.0174831, -1, 0, 4.81E-08],
    [0, 0, 0, 0.0610853, -2.30E-09, -0.0479572, -2.25E-15, 1, -4.81E-08],
    [0, 0, 0, -0.0610853, 2.30E-09, 0.0479572, 2.25E-15, -1, 4.81E-08],
    [0, 0, 0, -0.0610771, -4.22E-10, -0.00877001, 0, -1, 4.81E-08],
    [0, 0, 0, 0.0610771, 4.22E-10, 0.00877001, 0, 1, -4.81E-08],
    [0, 0, 0, 1.69E-09, -0.0610771, 0.050339, 1, -1.68E-15, -3.37E-08],
    [0, 0, 0, -1.69E-09, 0.0610771, -0.050339, -1, 1.68E-15, 3.37E-08]
])
Jc = np.array([
    [0.0794225, -0.247809, 1.07017, 0.014623, -0.0445384, -0.0113986, -0.0794225, 0.247809, -1.07017],
    [0.0794225, 0.65567, 0.881346, 0.0640447, -0.0375275, 0.0221469, -0.0794225, -0.65567, -0.881346],
    [0.539465, 0.196432, 0.93988, 0.0378876, -0.0675262, -0.00763365, -0.539465, -0.196432, -0.93988],
    [-0.38062, 0.211429, 1.01164, 0.0407802, -0.0145397, 0.0183819, 0.38062, -0.211429, -1.01164],
    [0, 0, 0, 0.00457503, -0.0391212, -0.0180544, -6.18E-08, -0.4615, 1],
    [0, 0, 0, 0.0609616, -0.0391212, 0.0180545, -6.18E-08, 0.4615, 1],
    [0, 0, 0, 0.0327683, -0.0673145, -0.0151226, 0.4615, 6.18E-08, 1],
    [0, 0, 0, 0.0327683, -0.010928, 0.0151226, -0.4615, 6.18E-08, 1],
    [0, 0, 0, 0.0174831, 0.0197664, -0.00806845, 0.4615, 4.81E-08, 1],
    [0, 0, 0, 0.0174831, 0.0761481, 0.00806844, -0.4615, 4.81E-08, 1],
    [0, 0, 0, 0.0456739, 0.0479572, -0.0221323, 4.81E-08, 0.4615, 1],
    [0, 0, 0, -0.0107078, 0.0479572, 0.0221323, 4.81E-08, -0.4615, 1],
    [0, 0, 0, -0.078526, -0.00877001, -0.00404736, 3.37E-08, -0.4615, 1],
    [0, 0, 0, -0.0221519, -0.00877001, 0.00404736, 3.37E-08, 0.4615, 1],
    [0, 0, 0, -0.050339, -0.0369571, 0.0232314, 0.4615, 4.81E-08, 1],
    [0, 0, 0, -0.050339, 0.019417, -0.0232314, -0.4615, 4.81E-08, 1]
])
M = np.array([
    [0.01, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0.01, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0.01, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0.000282999, 9.29E-11, 2.41E-10, 0, 0, 0],
    [0, 0, 0, 9.29E-11, 0.000283001, -2.87E-10, 0, 0, 0],
    [0, 0, 0, 2.41E-10, -2.87E-10, 0.000283, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0.3, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0.3, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0.3]
])

# =============================== Size checks ================================ #
assert zs.shape == (N_HORIZON, N_X + N_LAMBDA + N_U), f'Expected zs shape ' + \
    f'({N_HORIZON}, {N_X + N_LAMBDA + N_U}), got {zs.shape}.'
assert A.shape == (N_X, N_X), f'Expected A shape ({N_X}, {N_X}), got {A.shape}.'
assert B.shape == (N_X, N_U), f'Expected B shape ({N_X}, {N_U}), got {B.shape}.'
assert D.shape == (N_X, N_LAMBDA), f'Expected D shape ({N_X}, {N_LAMBDA}), ' + \
    f'got {D.shape}.'
assert d.shape == (N_X,), f'Expected d shape ({N_X},), got {d.shape}.'
assert Jn.shape == (N_C, N_V), f'Expected Jn shape ({N_C}, {N_V}), got ' + \
    f'{Jn.shape}.'
assert Jt.shape == (4*N_C, N_V), f'Expected Jt shape ({4*N_C}, {N_V}), got ' + \
    f'{Jt.shape}.'
assert Jc.shape == (4*N_C, N_V), f'Expected Jc shape ({4*N_C}, {N_V}), got ' + \
    f'{Jc.shape}.'
assert M.shape == (N_V, N_V), f'Expected M shape ({N_V}, {N_V}), got {M.shape}.'

# ====================== Calculations for checking form ====================== #
def report_error(derived: None, reported: None, name: str = None):
    """Report the norm error between a derived result and reported result."""
    error = np.linalg.norm(derived - reported)
    print(f'{name} error: {error}')
    print(f'\t-> Percentage: {error / np.linalg.norm(reported) * 100} %\n')

def compute_scaling_factor(A: None, unscaled_D: None):
    """Scaling factor is norm of A divided by norm of unscaled D."""
    return np.linalg.norm(A)**2 / np.linalg.norm(unscaled_D)**2

def extract_gamma(A: None):
    """Gamma is easily extracted from A, where the upper right corner is gamma
    times the time step.  This upper right corner in A also has a term
    dt^2 * gamma @ AB_v_v, but this term is zero because AB_v_v is zero."""
    upper_right = A[:N_Q, N_Q:]
    gamma = upper_right / DT
    return gamma

def extract_AB_v(A: None, B: None):
    """Extract the terms AB_v_q, AB_v_v, and AB_v_u from A and B.  AB_v_q and
    AB_v_v are most easily extracted from A, which has the form:
    
        A = [ I + dt^2 * gamma @ AB_v_q,  dt * gamma + dt^2 *gamma @ AB_v_v ]
            [        dt * AB_v_q       ,          I + dt * AB_v_v           ]

    AB_v_q will be extracted from A's lower left corner, and AB_v_v will be
    extracted from A's lower right corner.

    AB_v_u is extracted from B, which has the form:

        B = [ dt^2 * gamma @ AB_v_u ]
            [      dt * AB_v_u      ]

    AB_v_u will be extracted from B's lower half.
    """
    A_lower_left = A[N_Q:, :N_Q]
    AB_v_q = A_lower_left / DT

    A_lower_right = A[N_Q:, N_Q:]
    AB_v_v = (A_lower_right - np.eye(N_V)) / DT

    B_lower = B[N_Q:, :]
    AB_v_u = B_lower / DT

    return AB_v_q, AB_v_v, AB_v_u

def extract_d_v(d: None):
    """Extract the d_v vector from the d vector.  The d vector is of the form:

        d = [ dt^2 * gamma @ d_v ]
            [      dt * d_v      ]

    d_v will be extracted from the lower half of d.
    """
    d_lower = d[N_Q:]
    d_v = d_lower / DT
    return d_v

def derive_Jc_matrix(Jc: None, Jn: None, Jt: None):
    """The Jc matrix is derived from the Jn and Jt matrices, which are the
    normal and tangential Jacobians.  The Jc matrix is a combination of these
    components, of the form:
    
        Jc = E_t.T @ Jn + anitescu_mu_matrix @ Jt
    """
    E_t = np.zeros((N_C, N_LAMBDA))
    for i in range(N_C):
        E_t[i, 2*i*N_FRICTION_DIRS : 2*(i + 1)*N_FRICTION_DIRS] = \
            np.ones((1, 2*N_FRICTION_DIRS))

    # NOTE:  This works because mu is the same for all 4 contacts.  If this were
    # not true, this would need to group the contacts together and set blocks.
    anitescu_mu_matrix = MU * np.eye(N_LAMBDA)

    Jc_derived = E_t.T @ Jn + anitescu_mu_matrix @ Jt
    report_error(derived=Jc_derived, reported=Jc, name='Jc')
    return Jc_derived

def derive_D_matrix(D: None, M: None, Jc: None, gamma: None, A: None):
    """The D matrix is (n_x, n_lambda) in shape, where its n_q followed by n_v
    rows are:
    
        D_unscaled = [ dt^2 * gamma @ M^-1 @ Jc.T ]
                     [      dt * M^-1 @ Jc.T      ]

    The result is then scaled by the scaling factor, which is the norm of A
    divided by the norm of D_unscaled.

    This function also reports how closely this derived result gets to the
    recorded D matrix.
    """
    Minv_J_c_T = np.linalg.solve(M, Jc.T)
    D_unscaled = np.vstack((
        DT**2 * gamma @ Minv_J_c_T,
        DT * Minv_J_c_T))
    scaling_factor = compute_scaling_factor(A=A, unscaled_D=D_unscaled)
    D_derived = scaling_factor * D_unscaled
    report_error(derived=D_derived, reported=D, name='D')
    return D_derived

def derive_A_matrix(A: None, AB_v_q: None, AB_v_v: None, gamma: None):
    """Note:  This function "derives" A but from matrices that were extracted
    from the reported A, so it's not really a derivation as much as a sanity
    check.  A has the form:
    
        A = [ I + dt^2 * gamma @ AB_v_q,  dt * gamma + dt^2 *gamma @ AB_v_v ]
            [        dt * AB_v_q       ,          I + dt * AB_v_v           ]
    """
    A_derived = np.zeros((N_X, N_X))
    A_derived[:N_Q, :N_Q] = np.eye(N_Q) + DT**2 * gamma @ AB_v_q
    A_derived[:N_Q, N_Q:] = DT * gamma + DT**2 * gamma @ AB_v_v
    A_derived[N_Q:, :N_Q] = DT * AB_v_q
    A_derived[N_Q:, N_Q:] = np.eye(N_V) + DT * AB_v_v
    report_error(derived=A_derived, reported=A, name='A')
    return A_derived

def derive_B_matrix(B: None, AB_v_u: None, gamma: None):
    """Note:  This function "derives" B but from matrices that were extracted
    from the reported B, so it's not really a derivation as much as a sanity
    check.  B has the form:

        B = [ dt^2 * gamma @ AB_v_u ]
            [      dt * AB_v_u      ]
    """
    B_derived = np.zeros((N_X, N_U))
    B_derived[:N_Q, :] = DT**2 * gamma @ AB_v_u
    B_derived[N_Q:, :] = DT * AB_v_u
    report_error(derived=B_derived, reported=B, name='B')
    return B_derived

def derive_d_vector(d: None, d_v: None, gamma: None):
    """Note:  This function "derives" d but from vectors that were extracted
    from the reported d, so it's not really a derivation as much as a sanity
    check.  d has the form:

        d = [ dt^2 * gamma @ d_v ]
            [      dt * d_v      ]
    """
    d_derived = np.zeros(N_X)
    d_derived[:N_Q] = DT**2 * gamma @ d_v
    d_derived[N_Q:] = DT * d_v
    report_error(derived=d_derived, reported=d, name='d')
    return d_derived

# ================================ Check form ================================ #
gamma = extract_gamma(A=A)
AB_v_q_derived, AB_v_v_derived, AB_v_u_derived = extract_AB_v(A=A, B=B)
d_v_derived = extract_d_v(d=d)

A_derived = derive_A_matrix(A=A, AB_v_q=AB_v_q_derived, AB_v_v=AB_v_v_derived,
                            gamma=gamma)
B_derived = derive_B_matrix(B=B, AB_v_u=AB_v_u_derived, gamma=gamma)
D_derived = derive_D_matrix(D=D, M=M, Jc=Jc, gamma=gamma, A=A)
d_derived = derive_d_vector(d=d, d_v=d_v_derived, gamma=gamma)
Jc_derived = derive_Jc_matrix(Jc=Jc, Jn=Jn, Jt=Jt)

# =============== Calculations for checking progression of zs ================ #
def split_x_lambda_u(z: None):
    """Split the z vector into x, lambda, and u."""
    x = z[:N_X]
    lam = z[N_X:N_X + N_LAMBDA]
    u = z[N_X + N_LAMBDA:]
    return x, lam, u

def calculate_next_x(z_next: None, z_curr: None, A: None, B: None, D: None,
                     d: None):
    """The next state should be the result of the following equation:
    
        x_next = A @ x + B @ u + D @ lambda + d

    Thus, given the z vectors at the current and next time steps, we can
    validate the x portion of the next z, given the current z.
    """
    x, lam, u = split_x_lambda_u(z=z_curr)
    x_next_derived = A @ x + B @ u + D @ lam + d

    x_next = z_next[:N_X]
    report_error(derived=x_next_derived, reported=x_next, name='x_next')

    return x_next

# ======================= Calculate progression of zs ======================== #
for i in range(N_HORIZON - 1):
    z_curr = zs[i, :]
    z_next = zs[i + 1, :]
    print(f'=================== STEP {i} ===================\n')
    x_next = calculate_next_x(z_next=z_next, z_curr=z_curr, A=A, B=B, D=D, d=d)
    print(f'From derived matrices:')
    x_next = calculate_next_x(z_next=z_next, z_curr=z_curr, A=A_derived,
                              B=B_derived, D=D_derived, d=d_derived)
    
# ========================== Calculations for costs ========================== #
def make_Q_from_q_vector(q_vector: None):
    assert len(q_vector) == N_X
    return np.diag(q_vector)

def make_R_from_r_vector(r_vector: None):
    assert len(r_vector) == N_U
    return np.diag(r_vector)

def compute_cost(z: None, Q: None, R: None, name: str = None):
    """The cost is given by the equation:
    
        cost = (x - x_des).T @ Q @ (x - x_des) + u.T @ R @ u
    """
    assert z.shape == (N_HORIZON, N_X + N_LAMBDA + N_U), f'Expected z shape' + \
        f' ({N_HORIZON}, {N_X + N_LAMBDA + N_U}), got {z.shape}.'
    
    cost = 0
    for i in range(N_HORIZON):
        x, _, u = split_x_lambda_u(z=z[i])
        cost += (x - X_DESIRED).T @ Q @ (x - X_DESIRED) + u.T @ R @ u

    print(f'Cost {name}: {cost}\n')
    return cost

def make_copy_of_z_without_falling(z: None):
    """Make a copy of z without the falling state."""
    z_copy = np.copy(z)
    z_copy[:, N_Q-1] = z[0, N_Q-1]
    return z_copy

# =============================== Check costs ================================ #
Q = make_Q_from_q_vector(q_vector=Q_VECTOR)
R = make_R_from_r_vector(r_vector=R_VECTOR)

pdb.set_trace()
cost = compute_cost(z=zs, Q=Q, R=R, name='from reported zs')

z_no_falling = make_copy_of_z_without_falling(z=zs)
cost_no_falling = compute_cost(z=z_no_falling, Q=Q, R=R,
                               name='with no falling zs')



