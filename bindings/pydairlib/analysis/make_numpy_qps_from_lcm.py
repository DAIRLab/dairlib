import sys
import lcm
import numpy as np

from dairlib import lcmt_fcc_qp

from pydairlib.analysis.process_lcm_log import get_log_data

def qp_callback(data):
    qps = []
    for msg in data['FCCQP']:
        qps.append(
            {
                'Q': np.array(msg.Q),
                'b': np.array(msg.b),
                'A_eq': np.array(msg.Aeq),
                'b_eq': np.array(msg.beq),
                'friction_coeffs': msg.friction_coeffs,
                'lb': np.array(msg.lb),
                'ub': np.array(msg.ub),
            }
        )
    return qps


def main(logfile):
    log = lcm.EventLog(logfile, 'r')
    qps = get_log_data(log, {'FCCQP': lcmt_fcc_qp}, 0, -1, qp_callback)
    save_file = logfile.replace('lcmlog-05', 'id_qp_log_walking_reg.npz')
    np.savez(save_file, qps=qps)

    import pdb; pdb.set_trace()


if __name__ == '__main__':
    main(sys.argv[1])
