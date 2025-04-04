import lcm
import numpy as np

import dairlib
import argparse

from pydairlib.systems import(
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

from pydairlib.analysis.process_lcm_log import get_log_data


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logfile")
    args = parser.parse_args()
    mpfc_inputs = get_mpfc_inputs(args.logfile)


if __name__ == '__main__':
    main()