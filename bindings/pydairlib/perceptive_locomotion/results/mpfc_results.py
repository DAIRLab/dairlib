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
from pydairlib.analysis.mbp_plotting_utils import process_state_channel, get_floating_base_velocity_in_body_frame
from pydairlib.analysis.mpfc_plotting_utils import process_alip_mpfc_debug_data
from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context
from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.analysis.video_tools import extract_frames

from pydairlib.perceptive_locomotion.results import analysis_utils as utils

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


def velocity_tracking_plot(robot_output, mpc_debug, savefile=None):
    plant, context = make_plant_and_context()
    vel = get_floating_base_velocity_in_body_frame(
        robot_output, plant, context, plant.GetBodyByName("pelvis").body_frame()
    )
    utils.setup_plots()

    mpc_time = mpc_debug['t_mpc'] - mpc_debug['t_mpc'][0]
    robot_output_time = robot_output['t_x'] - robot_output['t_x'][0]
    vdes = mpc_debug['desired_velocity']
    fig = plt.figure(figsize=(8, 6))
    plt.plot(mpc_time, vdes[:, 0], linestyle='--', color=utils.plotting_palette[0], label='$v_{des, x}$')
    plt.plot(robot_output_time, vel[:, 0], linewidth=0.5, color=utils.plotting_palette[0], label='$v_{x}$')
    plt.plot(mpc_time, vdes[:, 1], linestyle='--', color=utils.plotting_palette[2], label='$v_{des, y}$')
    plt.plot(robot_output_time, vel[:, 1], linewidth=0.5, color=utils.plotting_palette[2], label='$v_{y}$')
    plt.ylim([-0.75, 1.5])
    ax = plt.gca()
    ax.autoscale(enable=True, axis='x', tight=True)
    plt.title('MPFC Velocity Tracking', fontsize=28)
    plt.xlabel('Time (s)')
    plt.ylabel('Pelvis Velocity (m/s)')
    plt.legend(ncol=4, fontsize=20, columnspacing=0.5)
    fig.tight_layout()

    if not savefile:
        plt.show()
    else:
        plt.savefig(savefile, bbox_inches='tight')


def vel_tracking_and_tiles(base_data_folder):
    logpath = os.path.join(base_data_folder, 'others/stairs_and_grass/lcmlog-laptop-01')
    video_path = os.path.join(base_data_folder, 'others/stairs_and_grass/IMG_9136.mov')
    robot_output, mpc_debug = load_log(logpath, 1, 100.5)
    velocity_tracking_plot(
        robot_output,
        mpc_debug,
        savefile='../manuscripts/perceptive_walking_tro/figures/velocity_tracking.svg'
    )
    extract_frames(
        8.0,
        108.0,
        40,
        video_path,
        '../manuscripts/perceptive_walking_tro/figures/motion_tiles',
        prefix='stairs_and_grass',
        frame_edits='crop_square'
    )


def solve_time_plot(mpc_debug):
    pass


def main():
    parser = ArgumentParser()
    parser.add_argument('--data_root', type=str, default='')
    parser.add_argument('--logfile', type=str, default='')
    args = parser.parse_args()
    vel_tracking_and_tiles(args.data_root)


if __name__ == '__main__':
    main()
