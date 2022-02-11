import lcm
import numpy as np
from process_lcm_log import get_log_data
from dairlib import lcmt_qp

import osqp
from scipy.sparse import csc_matrix


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


def solve_osqp(qp, settings):
    Q = qp["Q"] + qp["Q"].T - np.diag(np.diag(qp["Q"]))
    Q = Q + 1e-7 * np.eye(Q.shape[0])
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
        check_termination=settings['check_termination'])

    qp_result = m.solve()
    return {'iter': qp_result.info.iter,
            'run_time': qp_result.info.run_time,
            'status': qp_result.info.status,
            'obj_val': qp_result.info.obj_val,
            'pri_res': qp_result.info.pri_res,
            'dual_res': qp_result.info.dua_res,
            'x': qp_result.x}


def main():

    # filename = "/home/brian/workspace/logs/qp_logging/lcmlog03"
    # filename = "/home/yuming/Downloads/20220208_testing_osqp/02_07_22/lcmlog-03"
    # filename = "/home/yuming/Downloads/qp_logging/qp1e-7"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/3_scale_by_previous_sol/lcmlog-2022-02-11.01"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/4_log_scaling_offline/with_stricter_tolerance/lcmlog-2022-02-11.01"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/5_add_zero_stance_toe_effort_constraint/lcmlog-2022-02-11.00"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/6_add_one_more_constraint_to_get_rid_off_null_space/lcmlog-2022-02-11.01"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/7_rerun_baseline__strict_tolerance/lcmlog-2022-02-11.00"
    filename = "/home/yuming/Downloads/20220208_testing_osqp/8_increase_accel_weight__to_see_if_remove_null_space/lcmlog-2022-02-11.01"
    log = lcm.EventLog(filename, "r")
    qp_list = get_log_data(log, {"QP_LOG": lcmt_qp}, ParseQP)


    qp_list = qp_list[:400]
    eps = np.logspace(-7, -4, num=4)
    nr = eps.size
    run_times = np.zeros((len(qp_list), nr))
    obj_vals = np.zeros((len(qp_list), nr))
    pri_eps_vals = np.zeros((len(qp_list), nr))
    x = np.zeros((len(qp_list), nr, len(qp_list[0]["x_lb"])))
    dx = np.zeros((len(qp_list), nr))

    osqp_settings = \
        {'eps_abs': 1e-7,
         'eps_rel': 1e-7,
         'eps_prim_inf': 1e-6,
         'eps_dual_inf': 1e-6,
         'polish': 1,
         'scaled_termination': 1,
         'adaptive_rho_fraction': 1,
         'verbose': 0,
         'warm_start': 0,
         'rho': .1,
         'check_termination': 10}

    for j, eps_val in enumerate(eps):
        osqp_settings['eps_abs'] = eps_val
        osqp_settings['eps_rel'] = eps_val
        osqp_settings['eps_prim_inf'] = 10*eps_val
        osqp_settings['eps_dual_inf'] = 10*eps_val
        for i, qp in enumerate(qp_list):

            osqp_result = solve_osqp(qp, osqp_settings)
            run_times[i, j] = osqp_result['run_time']
            obj_vals[i, j] = osqp_result['obj_val']
            pri_eps_vals[i, j] = osqp_result['pri_res']
            x[i, j, :] = osqp_result['x']

    for j in range(1, nr):
      for i, qp in enumerate(qp_list):
        dx[i, j] = np.linalg.norm(x[i, j, :] - x[i, 0, :])

    print(f'OSQP mean runtime: {np.mean(run_times,axis=0)}')
    print(f'OSQP mean objective: {np.mean(obj_vals, axis=0)}')
    print(f'OSQP mean primal residuals: {np.mean(pri_eps_vals, axis=0)}')

    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(4, 1, sharex=True)

    axs[0].plot(obj_vals)
    axs[0].legend(['eps = ' + str(eps_val) for eps_val in eps])
    axs[0].set_title('Objective values per QP')

    axs[1].plot(run_times)
    axs[1].set_title('Solve times per QP')

    axs[2].plot(pri_eps_vals)
    axs[2].set_title('Primal residual per QP')

    axs[3].plot(dx)
    axs[3].set_title('solution difference per QP')

    plt.show()

    import pdb;pdb.set_trace()


if __name__ == '__main__':
    main()
