import os
import time
import glob
import subprocess
from time import sleep
from copy import deepcopy
from functools import partial
from multiprocessing import Pool
from argparse import ArgumentParser

# lcmtypes
from dairlib import(
    lcmt_grid_map,
    lcmt_foothold_set,
    lcmt_robot_output,
    lcmt_alip_s2s_mpfc_debug
)

# installed
import lcm
import yaml
import numpy as np

# Plotting
import matplotlib
import seaborn as sns
import matplotlib.pyplot as plt

# dairlib
from pydairlib.analysis.mbp_plotting_utils import process_state_channel
from pydairlib.analysis.mpfc_plotting_utils import process_alip_mpfc_debug_data
from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context
from pydairlib.analysis.process_lcm_log import get_log_data


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'


def process_mpc_data(data_dict):
    plant, _ = make_plant_and_context()

    robot_output = process_state_channel(data_dict[state_channel], plant)
    mpc_debug = process_alip_mpfc_debug_data(data_dict[mpfc_debug_channel])

    return robot_output, mpc_debug


def load_log(logfile: str, start_time=0, duration=-1):
    log = lcm.EventLog(logfile, "r")
    robot_output, mpc_data = get_log_data(
        log,
        {
            state_channel: lcmt_robot_output,
            mpfc_debug_channel: lcmt_alip_s2s_mpfc_debug
        },
        start_time,
        duration,
        process_mpc_data
    )
    return robot_output, mpc_data


def velocity_tracking_plot():
    pass


def solve_time_plot():
    pass