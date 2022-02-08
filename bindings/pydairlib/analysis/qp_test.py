import lcm
import numpy as np
from process_lcm_log import get_log_data
from dairlib import lcmt_qp
# import qpoases
import time
import sys
# import qpsolvers

import osqp
# import gurobipy as gp
from scipy.sparse import csc_matrix

def ConvertIneq(A_ineq, ineq_lb, ineq_ub):
    is_equality = ineq_ub - ineq_lb < 1e-6

    # Parse out equality constraints
    b_eq = ineq_ub[is_equality]
    A_eq = A_ineq[is_equality, :]
    A_ineq = A_ineq[is_equality == False, :]
    ineq_lb = ineq_lb[is_equality == False]
    ineq_ub = ineq_ub[is_equality == False]

    # Convert to Ax <= b
    ub_active = ineq_ub <= 1e10
    lb_active = ineq_lb >= -1e10
    A_ineq = np.vstack((A_ineq[ub_active,:], -A_ineq[lb_active,:]))
    b_ineq = np.hstack((ineq_ub[ub_active], -ineq_lb[lb_active]))

    return (A_ineq, b_ineq, A_eq, b_eq)

def ParseQP(data):
    qp_list = []

    for msg in data["QP_LOG"]:
        # import pdb; pdb.set_trace()
        qp = {"Q": np.asarray(msg.Q),
              "w": np.asarray(msg.w),
              "A_ineq": np.asarray(msg.A_ineq),
              "ineq_lb": np.asarray(msg.ineq_lb),
              "ineq_ub": np.asarray(msg.ineq_ub),
              "A_eq": np.asarray(msg.A_eq),
              "b_eq": np.asarray(msg.b_eq),
              "x_lb": np.asarray(msg.x_lb),
              "x_ub": np.asarray(msg.x_ub),
              }

        qp_list.append(qp)

    return qp_list

# filename = "/home/posa/workspace/osc_qp.log"
filename = sys.argv[1]
log = lcm.EventLog(filename, "r")
qp_list = \
    get_log_data(log,
                 {"QP_LOG": lcmt_qp},
                 ParseQP)


qp_list = qp_list[:200]
run_times = np.zeros((len(qp_list), 1))
obj_vals = np.zeros((len(qp_list), 1))
run_times_gr = np.zeros((len(qp_list), 1))
obj_vals_gr = np.zeros((len(qp_list), 1))
run_times_qpo = np.zeros((len(qp_list), 1))
obj_vals_qpo = np.zeros((len(qp_list), 1))
run_times_other = np.zeros((len(qp_list), 1))
obj_vals_other = np.zeros((len(qp_list), 1))


for i, qp in enumerate(qp_list):
    Q = qp["Q"] + qp["Q"].T -  np.diag(np.diag(qp["Q"]))
    Q = Q + 1e-7*np.eye(Q.shape[0])
    # Q = qp["Q"]

    # qp["A_ineq"] = np.vstack((qp["A_ineq"][:68,:], qp["A_ineq"][69:, :]))
    # qp["ineq_lb"] = np.hstack((qp["ineq_lb"][:68], qp["ineq_lb"][69:]))
    # qp["ineq_ub"] = np.hstack((qp["ineq_ub"][:68], qp["ineq_ub"][69:]))

    # if i == 0:?
    m = osqp.OSQP()
    m.setup(P=csc_matrix(Q), q=qp["w"], A=csc_matrix(qp["A_ineq"]), l=qp["ineq_lb"], u=qp["ineq_ub"])
    m.update_settings(eps_abs=1e-7, einitps_rel=1e-7, eps_prim_inf=1e-6, eps_dual_inf=1e-6, polish=1, scaled_termination=1, adaptive_rho_fraction=1, verbose=0, warm_start=0, rho=.1, check_termination=3)
    # m.update_settings(eps_abs=1e-7, eps_rel=1e-7, eps_prim_inf=1e-6, eps_dual_inf=1e-6, polish=1, scaled_termination=1, adaptive_rho_fraction=1, verbose=0, warm_start=0, rho=.1, check_termination=3)
    # else:
    #   # import pdb; pdb.set_trace()
    #   m.update(q=qp["w"], Ax=csc_matrix(qp["A_ineq"]).data, l=qp["ineq_lb"], u=qp["ineq_ub"])
    results = m.solve()
    run_times[i] = results.info.run_time
    obj_vals[i] = results.info.obj_val

    (A_ineq, b_ineq, A_eq, b_eq) = ConvertIneq(qp["A_ineq"], qp["ineq_lb"], qp["ineq_ub"])

    #
    # GUROBI
    #
    # m_gr = gp.Model()
    # m.Params.FeasibilityTol = 1e-4
    # m.Params.OptimalityTol = 1e-4
    # m_gr.Params.LogToConsole = 0
    # m_gr.Params.method = -1
    # x = m_gr.addMVar(Q.shape[0], lb=-float('inf'), ub=float('inf'))
    # obj = x @ (Q/2) @ x + qp["w"] @ x
    # obj = x @ x
    # m_gr.setObjective(obj)

    # m_gr.addConstr(A_ineq @ x <= b_ineq)
    # m_gr.addConstr(A_eq @ x == b_eq)

    # m_gr.optimize()
    # run_times_gr[i] = m_gr.Runtime
    # obj_vals_gr[i] = obj.getValue()

    #
    # QPOASES
    #

    # nWSR = np.array([10000])
    # nx = Q.shape[0]
    # m_qpo = qpoases.PyQProblem(nx, qp["A_ineq"].shape[0])
    # options = qpoases.PyOptions()
    # options.printLevel = qpoases.PyPrintLevel.NONE
    # options.terminationTolerance = 1e-6
    # options.boundTolerance = 1e-6
    # options.enableEqualities = 1
    # options.enableFarBounds = 0
    # options.enableDriftCorrection = 0
    # options.enableFlippingBounds = 0
    # options.enableInertiaCorrection = 0
    # m_qpo.setOptions(options)
    #
    # start = time.time()
    # m_qpo.init(Q, qp["w"], qp["A_ineq"].T, -np.full(nx, 1e10), np.full(nx, 1e10), qp["ineq_lb"], qp["ineq_ub"], nWSR)
    # obj_vals_qpo[i] = m_qpo.getObjVal()
    # run_times_qpo[i] = time.time() - start

    # #
    # # qpsolvers
    # #
    # solvers = ['cvxopt', 'cvxpy', 'ecos', 'gurobi', 'osqp', 'qpoases', 'quadprog', 'scs']
    # other_i = 6
    # start = time.time()
    # x_other = qpsolvers.solve_qp(Q, qp["w"], A_ineq, b_ineq, A_eq, b_eq, -np.full(nx, 1e10), np.full(nx, 1e10), solver=solvers[other_i])
    # run_times_other[i] = time.time() - start
    # # import pdb; pdb.set_trace()
    # if x_other is not None:
    #   obj_vals_other[i] = .5*x_other @ Q @ x_other + qp["w"]@x_other
    # else:
    #   obj_vals_other[i] = 1e3


print(np.mean(run_times))
print(np.mean(obj_vals))

# print(np.mean(run_times_gr))
# print(np.mean(obj_vals_gr))
#
# print(np.mean(run_times_qpo))
# print(np.mean(obj_vals_qpo))


import matplotlib.pyplot as plt
fig, axs = plt.subplots(2, 1, sharex=True)


axs[0].plot(obj_vals, label='OSQP')
axs[0].plot(obj_vals_gr, label='Gurobi')
axs[0].plot(obj_vals_qpo, label='QPOASES')
# axs[0].plot(obj_vals_other, label=solvers[other_i])
axs[0].legend()
axs[0].set_title('Objective values per QP')

axs[1].plot(run_times)
axs[1].plot(run_times_gr)
axs[1].plot(run_times_qpo)
# axs[1].plot(run_times_other)
axs[1].set_title('Solve times per QP')
plt.show()
import pdb; pdb.set_trace()
