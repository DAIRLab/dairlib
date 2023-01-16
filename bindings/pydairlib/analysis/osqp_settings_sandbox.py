import lcm
import numpy as np
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
from dairlib import lcmt_qp
import sys
import matplotlib.pyplot as plt

import osqp
import proxsuite
from scipy.sparse import csc_matrix

saved_sols_folder = '/media/yangwill/backups/home/yangwill/Documents/research/projects/cassie/hardware/gain_tuning/qp_settings/cost_tuning/qp_formulation/'


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


def ConvertIneq(A_ineq, ineq_lb, ineq_ub):
    is_equality = ineq_ub - ineq_lb < 1e-6

    # Parse out equality constraints
    b_eq = ineq_ub[is_equality]
    A_eq = A_ineq[is_equality, :]
    A_ineq = A_ineq[is_equality == False, :]
    ineq_lb = ineq_lb[is_equality == False]
    ineq_ub = ineq_ub[is_equality == False]

    return (A_ineq, ineq_lb, ineq_ub, A_eq, b_eq)


def solve_osqp(qp, settings):
    Q = qp["Q"]
    # Q = qp["Q"] + qp["Q"].T - np.diag(np.diag(qp["Q"]))
    # Q = Q + 1e-6 * np.eye(Q.shape[0])
    m = osqp.OSQP()
    m.setup(
        P=csc_matrix(Q), q=qp["w"],
        A=csc_matrix(qp["A_ineq"]),
        l=qp["ineq_lb"], u=qp["ineq_ub"],
        eps_abs=settings['eps_abs'],
        eps_rel=settings['eps_rel'],
        eps_prim_inf=settings['eps_prim_inf'],
        eps_dual_inf=settings['eps_dual_inf'],
        polish=settings['polish'],
        scaled_termination=settings['scaled_termination'],
        adaptive_rho_fraction=settings['adaptive_rho_fraction'],
        verbose=settings['verbose'],
        warm_start=settings['warm_start'],
        rho=settings['rho'],
        max_iter=settings['max_iter'],
        check_termination=settings['check_termination'])

    qp_result = m.solve()

    # if qp_result.info.run_time > 5e-3:
    #     import pdb; pdb.set_trace()
    return {'iter': qp_result.info.iter,
            'run_time': qp_result.info.run_time,
            'status': qp_result.info.status,
            'y_sol': qp_result.y,
            'obj_val': qp_result.info.obj_val,
            'pri_res': qp_result.info.pri_res,
            'dual_res': qp_result.info.dua_res,
            'sol': qp_result.x}


def solve_proxsuite(qp_list, settings):
    run_times = np.zeros((len(qp_list),))
    obj_vals = np.zeros((len(qp_list),))
    iter = np.zeros((len(qp_list),))
    prim_res = np.zeros((len(qp_list),))
    dual_res = np.zeros((len(qp_list),))
    sol = np.zeros((len(qp_list),))
    # import pdb; pdb.set_trace()

def solve_proxsuite(qp_list, settings):
    run_times = np.zeros((len(qp_list),))
    obj_vals = np.zeros((len(qp_list),))
    iter = np.zeros((len(qp_list),))
    prim_res = np.zeros((len(qp_list),))
    dual_res = np.zeros((len(qp_list),))
    sol = np.zeros((len(qp_list),60))

    for i, qp in enumerate(qp_list):
        # Q = qp["Q"] + qp["Q"].T - np.diag(np.diag(qp["Q"]))
        # Q = Q + 1e-7*np.eye(Q.shape[0])
        Q = qp["Q"]
        C, l, u, A, b = ConvertIneq(qp["A_ineq"], qp["ineq_lb"], qp["ineq_ub"])
        H = Q
        g = qp["w"]
        n = H.shape[0]
        n_eq = A.shape[0]
        n_in = C.shape[0]
        qp = proxsuite.proxqp.sparse.QP(n, n_eq, n_in)
        # qp.init(csc_matrix(H), g, csc_matrix(A), b, csc_matrix(C), l, u, rho=settings['rho'])
        qp.init(H, g, A, b, C, l, u, rho=settings['rho'])

        qp.settings.eps_abs = settings["eps_abs"]
        qp.settings.eps_rel = settings["eps_rel"]
        qp.settings.eps_primal_inf = settings["eps_prim_inf"]
        qp.settings.eps_dual_inf = settings["eps_dual_inf"]
        qp.settings.verbose = settings["verbose"]
        qp.settings.max_iter = settings["max_iter"]
        # qp.settings.default_rho = settings['rho'],
        # check_termination=settings['check_termination']
        qp.settings.initial_guess = proxsuite.proxqp.InitialGuess.NO_INITIAL_GUESS

        qp.solve()
        run_times[i] = qp.results.info.run_time * 1e-6
        obj_vals[i] = qp.results.info.objValue
        iter[i] = qp.results.info.iter
        prim_res[i] = qp.results.info.pri_res
        dual_res[i] = qp.results.info.dua_res
        sol[i] = qp.results.x

    return {
        'iter': iter,
        'run_time': run_times,
        'obj_val': obj_vals,
        'prim_res': prim_res,
        'dual_res': dual_res,
        'sol': sol,
    }

def main():
    # filename = "/home/brian/workspace/logs/qp_logging/lcmlog00"
    filename = sys.argv[1]

    log = lcm.EventLog(filename, "r")
    qp_list = get_log_data(log, {"QP_LOG": lcmt_qp}, 0, 5000, ParseQP)

    # qp_list = qp_list[:2000][::10]
    # qp_list = qp_list[:5000][::10]
    run_times = np.zeros((len(qp_list),))
    obj_vals = np.zeros((len(qp_list),))
    pri_eps_vals = np.zeros((len(qp_list, )))
    # sols = np.zeros((len(qp_list,), 60))
    sols = np.zeros((len(qp_list, ), 60))
    iters = np.zeros((len(qp_list, ),))

    osqp_settings = \
        {'rho': 1e-4,
         'sigma': 1e-6,
         'max_iter': 500,
         'eps_abs': 1e-5,
         'eps_rel': 1e-5,
         'eps_prim_inf': 1e-6,
         'eps_dual_inf': 1e-6,
         'alpha': 1.6,
         'linsys_solver': 0,
         'delta': 1e-6,
         'polish': 1,
         'polish_refine_iter': 3,
         'verbose': 0,
         'scaled_termination': 1,
         'check_termination': 25,
         'warm_start': 1,
         'adaptive_rho': 1,
         'adaptive_rho_interval': 0,
         'adaptive_rho_tolerance': 5,
         'adaptive_rho_fraction': 0.4}

    for i, qp in enumerate(qp_list):
        osqp_result = solve_osqp(qp, osqp_settings)
        run_times[i] = osqp_result['run_time']
        obj_vals[i] = osqp_result['obj_val']
        pri_eps_vals[i] = osqp_result['pri_res']
        sols[i] = osqp_result['sol']
        iters[i] = osqp_result['iter']

    osqp_result = solve_proxsuite(qp_list, osqp_settings)
    fig, axs = plt.subplots(4, 1, sharex='all')
    axs[0].plot(osqp_result['sol'][:, 32:42])
    axs[0].legend(['0', '1', '2', '3', '4'])
    axs[0].set_title('Objective values per QP')
    axs[1].plot(osqp_result['run_time'])
    axs[1].set_title('Solve times per QP')
    axs[2].plot(osqp_result['sol'][:, 22:24])
    axs[2].plot(osqp_result['sol'][:, 28:30])
    axs[3].plot(osqp_result['sol'][:, 24:28])
    axs[3].plot(osqp_result['sol'][:, 30:32])
    axs[3].set_title('QP solutions')

    # for key in osqp_result.keys():
    #     plt.figure()
    #     # plt.plot(osqp_result[key], label='osqp')
    #     plt.plot(osqp_result[key], label='proxqp')
    #     plt.title(key)
    #     plt.legend()
    # plt.show()

    # import pdb; pdb.set_trace()
    log_name = filename.split('/')[-1]
    np.save(saved_sols_folder + log_name + '_' + str(osqp_settings['max_iter']), sols)

    print(f'OSQP mean runtime: {np.mean(run_times)}')
    # print(f'PROXQP mean runtime: {np.mean(osqp_result['run_time'])})
    proxqp_runtime = osqp_result['run_time']
    print(f'PROXQP mean runtime: {np.mean(proxqp_runtime)}')
    print(f'OSQP mean objective: {np.mean(obj_vals)}')
    print(f'OSQP mean primal residuals: {np.mean(pri_eps_vals)}')

    fig, axs = plt.subplots(4, 1, sharex=True)

    # axs[0].plot(obj_vals, label='OSQP')
    # axs[0].plot(sols[:, 44:50])
    # axs[0].plot(sols[:, 55:60])
    axs[0].plot(sols[:, 32:42])
    axs[0].legend(['0', '1', '2', '3', '4'])
    axs[0].set_title('Objective values per QP')

    axs[1].plot(run_times)
    axs[1].set_title('Solve times per QP')

    # axs[2].plot(iters)
    # axs[2].set_title('QP solutions')

    axs[2].plot(sols[:, 22:24])
    axs[2].plot(sols[:, 28:30])
    axs[2].set_title('Knee Torque Solutions')

    axs[3].plot(sols[:, 24:28])
    axs[3].plot(sols[:, 30:32])
    axs[3].set_title('QP solutions')


def compare_sols():
    # sol0 = np.load(saved_sols_folder + 'lcmlog-drake0_1000.npy')
    sol0 = np.load(saved_sols_folder + 'lcmlog-drake1_1000.npy')
    # sol1 = np.load(saved_sols_folder + 'lcmlog-drake0_200.npy')
    sol1 = np.load(saved_sols_folder + 'lcmlog-drake1_200.npy')
    fig, axs = plt.subplots(5, 1, sharex=True)
    # a = np.max(sol0 - sol1, axis=0)
    a = np.average(sol0 - sol1, axis=0)

    axs[0].plot(a[0:22])
    axs[1].plot(a[22:32])
    axs[2].plot(a[32:44])
    axs[3].plot(a[44:50])
    axs[4].plot(a[50:60])


if __name__ == '__main__':
    # compare_sols()
    main()
    plt.show()
