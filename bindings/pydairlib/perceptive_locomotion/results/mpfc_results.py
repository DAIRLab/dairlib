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


# LCM Channels
state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'

# Relevant data paths (relative to the root of the data folder)
demo_trial_path = 'other/stairs_and_grass_11_16_24/'
demo_trial_log = os.path.join(demo_trial_path, 'lcmlog-laptop-01')
demo_trial_video = os.path.join(demo_trial_path, 'IMG_9136.MOV')
demo_trial_duration = 100.5

solve_time_results_config_path = 'solve_time_results_config.yaml'

# Outputs
output_folder = 'perceptive_locomotion_results_figures/'


def process_mpc_data(data_dict):
    plant, _ = make_plant_and_context()
    
    robot_output = process_state_channel(data_dict[state_channel], plant)
    mpc_debug = process_alip_mpfc_debug_data(data_dict[mpfc_debug_channel])
    
    return robot_output, mpc_debug


def load_log(logfile: str, start_time: float = 0, duration: float = -1):
    log = None
    try:
        log = lcm.EventLog(logfile, "r")
    except FileNotFoundError:
        print(f'Log not found: {logfile}')
        exit(-1)
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
    plt.title('Velocity Tracking', fontsize=28)
    plt.xlabel('Time (s)')
    plt.ylabel('Pelvis Velocity (m/s)')
    plt.legend(ncol=4, fontsize=20, columnspacing=0.5)
    fig.tight_layout()
    
    if not savefile:
        plt.show()
    else:
        plt.savefig(savefile, bbox_inches='tight')


def vel_tracking_and_tiles(base_data_folder):
    logpath = os.path.join(base_data_folder, demo_trial_log)
    video_path = os.path.join(base_data_folder, demo_trial_video)
    
    print(f'Extracting motion tiles from {video_path}')
    
    motion_tile_folder = os.path.join(output_folder, 'motion_tiles')
    utils.make_dir_if_missing(motion_tile_folder)
    
    robot_output, mpc_debug = load_log(logpath, 1, demo_trial_duration)
    velocity_tracking_plot(
        robot_output,
        mpc_debug,
        savefile=os.path.join(output_folder, 'velocity_tracking.svg')
    )
    extract_frames(
        8.0,
        108.0,
        40,
        video_path,
        motion_tile_folder,
        prefix='stairs_and_grass',
        frame_edits='crop_square'
    )


def solve_time_series_plot(base_data_folder):
    utils.setup_plots()
    fig = plt.figure(figsize=(8, 6))
    logpath = os.path.join(base_data_folder, demo_trial_log)
    robot_output, mpc_debug = load_log(logpath, 1, demo_trial_duration)
    plt.plot(
        mpc_debug['t_mpc'] - mpc_debug['t_mpc'][0],
        mpc_debug['optimizer_time'],
        label='Solve Time',
        color=utils.plotting_palette[5]
    )
    plt.plot(
        mpc_debug['t_mpc'] - mpc_debug['t_mpc'][0],
        0.01 * np.ones_like(mpc_debug['optimizer_time']),
        color=utils.plotting_palette[0],
        linestyle='--',
        linewidth=2,
        label='100 Hz.'
    )
    plt.title('MPFC Solve Times')
    ax = plt.gca()
    ax.autoscale(enable=True, axis='x', tight=True)
    plt.ylim([0, 0.014])
    plt.legend(fontsize=20, loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Solve Time (s)')
    fig.tight_layout()
    plt.savefig(
        os.path.join(output_folder, 'solve_time_series.svg'),
        bbox_inches='tight'
    )


def elevation_plot(base_data_folder):
    utils.setup_plots()
    fig = plt.figure(figsize=(8, 6))
    logpath = os.path.join(base_data_folder, demo_trial_log)
    robot_output, _ = load_log(logpath, 1, demo_trial_duration)
    plt.plot(
        robot_output['t_x'] - robot_output['t_x'][0],
        robot_output['q'][:, 6] - robot_output['q'][0, 6],
        label='Pelvis Height',
        color=utils.plotting_palette[5]
    )
    plt.title('Estimated Elevation Change')
    ax = plt.gca()
    ax.autoscale(enable=True, axis='x', tight=True)
    plt.xlabel('Time (s)')
    plt.ylabel('Elevation Change (m)')
    fig.tight_layout()
    plt.savefig(
        os.path.join(output_folder, 'pelvis_height_series.svg'),
        bbox_inches='tight'
    )


def solve_time_proportions_plot(base_data_folder):
    with open(os.path.join(base_data_folder, solve_time_results_config_path)) as stream:
        config = yaml.safe_load(stream)
    mpc_debugs = []
    total_sec = 0
    for log in config['logs']:
        _, mpc_debug = load_log(
            os.path.join(base_data_folder, log['file']),
            log['start'],
            log['duration']
        )
        total_sec += log['duration']
        mpc_debugs.append(mpc_debug)
    
    data = np.concatenate([d['optimizer_time'] for d in mpc_debugs])
    utils.setup_plots()
    fig = plt.figure(figsize=(11, 5))
    sns.histplot(
        data=data,
        bins=10,
        element='step',
        stat='proportion'
    )
    plt.title('Solve Time Distribution')
    plt.xlabel('Solve Time (s)')
    plt.savefig(
        os.path.join(output_folder, 'solve_times.svg'),
        bbox_inches='tight'
    )
    numpy_stats_to_latex(
        data,
        'MPFC Solve Time',
        os.path.join(output_folder, 'solve_time_stats.tex')
    )


def snake_case(text: str):
    return text.lower().replace(' ', '_')


def numpy_stats_to_latex(data, name, save_path=None):
    # Compute statistical measures
    stats = {
        'N': np.size(data),
        'Mean': np.mean(data),
        'Median': np.median(data),
        '99.9th Percentile': np.percentile(data, 99.9),
        'Maximum': np.max(data)
    }
    
    # Create LaTeX table
    latex_table = "\\begin{table}[h!]\n"
    latex_table += "\\centering\n"
    latex_table += "\\caption{" + name + " Summary Statistics}\n"
    latex_table += "\\begin{tabular}{|c|c|}\n"
    latex_table += "\\hline\n"
    latex_table += "\\textbf{Statistic} & \\textbf{Value} \\\\\n"
    latex_table += "\\hline\n"
    
    # Add rows for each statistic
    for stat, value in stats.items():
        if stat == 'N':
            latex_table += f"{stat} & {value} \\\\\n"
        else:
            latex_table += f"{stat} & {value:.4f} \\\\\n"
    
    latex_table += "\\hline\n"
    latex_table += ("\\end{tabular}\n")
    latex_table += ("\\label{tab:" + snake_case(name) +"_stats}\n")
    latex_table += "\\end{table}"
    
    # Save to file if output path is provided
    if save_path is not None:
        with open(save_path, 'w') as f:
            f.write(latex_table)
        print(f"LaTeX table saved to {save_path}")
    else:
        print(latex_table)


def main():
    parser = ArgumentParser()
    parser.add_argument('--data_root', type=str, default='')
    args = parser.parse_args()
    
    utils.make_dir_if_missing(output_folder)
    
    solve_time_proportions_plot(args.data_root)
    solve_time_series_plot(args.data_root)
    vel_tracking_and_tiles(args.data_root)
    elevation_plot(args.data_root)


if __name__ == '__main__':
    main()
