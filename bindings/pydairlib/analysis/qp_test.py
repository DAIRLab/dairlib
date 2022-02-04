import lcm
import numpy as np
from process_lcm_log import get_log_data
from dairlib import lcmt_qp

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

filename = "/home/posa/workspace/osc_qp.log"
log = lcm.EventLog(filename, "r")
qp_list = \
    get_log_data(log,
                 {"QP_LOG": lcmt_qp},
                 ParseQP)

import pdb; pdb.set_trace()
