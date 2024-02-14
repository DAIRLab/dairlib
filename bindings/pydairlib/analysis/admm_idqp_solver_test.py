import lcm
import numpy as np

from dairlib import lcmt_id_qp
from process_lcm_log import get_log_data


def parse_qps(data_dict):
    qps = []

    for msg in data_dict['ID_QP_LOG']:
        qp_matrices = {}
        qp_matrices['Q'] = np.array(msg.Q)
        qp_matrices['b'] = np.array(msg.b)
        qp_matrices['c'] = msg.c
        qp_matrices['M'] = np.array(msg.M)
        qp_matrices['B'] = np.array(msg.B)
        qp_matrices['Jh'] = np.array(msg.Jh)
        qp_matrices['Jc'] = np.array(msg.Jc)
        qp_matrices['Je'] = np.array(msg.Je)
        qp_matrices['Jc_active'] = np.array(msg.Jc_active)
        qp_matrices['bias'] = np.array(msg.bias)
        qp_matrices['JdotV_h'] = np.array(msg.JdotV_h)
        qp_matrices['JdotV_c'] = np.array(msg.JdotV_c)
        qps.append(qp_matrices)

    return qps


def main():
    logname = '/home/brian/logs/id_qp/id_qp_log_running'
    save = '/media/brian/Extreme SSD/id_qp_log_running'
    log = lcm.EventLog(logname, "r")
    channels = {'ID_QP_LOG': lcmt_id_qp}
    data = get_log_data(log, channels, 0, -1, parse_qps)
    np.savez(save, data)
    import pdb; pdb.set_trace()


if __name__ == '__main__':
    main()