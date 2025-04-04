import os
import lcm
import argparse
import numpy as np
import matplotlib.pyplot as plt

import dairlib

from pydairlib.systems import(
    make_alip_s2s_mpfc_params_from_yaml,
    AlipS2SMpfcSolution,
    AlipS2SMpfcParams,
    AlipS2SMpfcInput,
    AlipS2SMPFC,
    Stance
)

from pydairlib.geometry import (
    ConvexPolygonSet,
    ConvexPolygon
)

from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context
from pydairlib.analysis.process_lcm_log import get_log_data


def make_mpfc_params(method: str, horizon: int):
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    solver_options = 'gurobi_options_multithreaded' if method == 'miqp' else \
        'ncqp_params'
    solver_options_file = os.path.join(
        base_folder, f'gains/{solver_options}.yaml'
    )
    gains_file = os.path.join(
        base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
    )
    plant, context = make_plant_and_context()
    return make_alip_s2s_mpfc_params_from_yaml(
        gains_file,
        solver_options_file,
        plant,
        context
    )


def foothold_set_from_lcm(msg):
    footholds = []
    for f in msg.footholds:
        foothold = ConvexPolygon()
        Aeq = np.array(f.Aeq)
        A = np.array(f.A)
        b = np.array(f.b)
        foothold.SetPlane(Aeq, f.beq)
        foothold.AddFaces(A, b)
        footholds.append(foothold)
    return ConvexPolygonSet(footholds)


def extract_mpfc_inputs(data, mpfc_input_channel):
    inputs = []
    for msg in data[mpfc_input_channel]:
        mpfc_input = AlipS2SMpfcInput()
        mpfc_input.x = np.array(msg.x)
        mpfc_input.p = np.array(msg.p)
        mpfc_input.t = msg.t
        mpfc_input.vdes = np.array(msg.vdes)
        mpfc_input.tmin = msg.tmin
        mpfc_input.tmax = msg.tmax
        mpfc_input.stance = Stance.kLeft if msg.stance < 0 else Stance.kRight
        mpfc_input.footholds = foothold_set_from_lcm(msg.footholds)
        mpfc_input.p_prev_stance = np.array(msg.p_prev_stance)
        inputs.append(mpfc_input)
    return inputs


def get_mpfc_inputs(logfile: str):
    log = lcm.EventLog(logfile, "r")

    channel = "ALIP_S2S_MPFC_INPUTS"
    data_channels = {channel: dairlib.lcmt_alip_s2s_mpfc_input}
    input_data = get_log_data(
        log,
        data_channels,
        0, -1,
        extract_mpfc_inputs,
        channel
    )
    return input_data


def cost_comparison(mpfc_inputs, horizon: int):
    params_admm = make_mpfc_params('admm', horizon)
    params_miqp = make_mpfc_params('miqp', horizon)
    admm_solver = AlipS2SMPFC(params_admm)
    miqp_solver = AlipS2SMPFC(params_miqp)

    solutions_admm = [
        admm_solver.Solve(inp) for inp in mpfc_inputs
    ]
    solutions_miqp = [
        miqp_solver.Solve(inp) for inp in mpfc_inputs
    ]

    admm_costs = [sol.max_constraint_viol for sol in solutions_admm]
    miqp_costs = [sol.max_constraint_viol for sol in solutions_miqp]

    admm_success = [1 if sol.success else 0 for sol in solutions_admm]
    miqp_success = [1 if sol.success else 0 for sol in solutions_miqp]

    plt.plot(admm_costs)
    plt.plot(miqp_costs)
    plt.legend(['ADMM', 'MIQP'])

    plt.figure()
    plt.plot(admm_success)
    plt.plot(miqp_success)
    plt.legend(['ADMM', 'MIQP'])
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logfile")
    args = parser.parse_args()
    mpfc_inputs = get_mpfc_inputs(args.logfile)
    cost_comparison(mpfc_inputs, 3)


if __name__ == '__main__':
    main()
