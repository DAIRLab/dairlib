"""
    Script to generate the simulation figures for
    "[title undecided for now]" from simulation lcm logs
"""

import sys
import lcm
import numpy as np
import matplotlib.pyplot as plt

import dairlib

from pydairlib.common import FindResourceOrThrow
from pydrake.all import (
    PiecewisePolynomial,
    Box,
    RigidTransform,
    Meshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    Simulator,
    SceneGraph,
    MultibodyPlant,
    MeshcatVisualizer,
    StartMeshcat,
    MeshcatVisualizerParams
)

import pydairlib.analysis.cassie_plotting_utils as cassie_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.analysis.process_lcm_log import get_log_data_from_first_message
from pydairlib.multibody import MultiposeVisualizer

from pydairlib.common.plot_styler import PlotStyler
from pydairlib.perceptive_locomotion.multipose_visualizer_from_lcm_log \
    import multipose_visualizer_main

stairs_up_logpath = '/home/brian/workspace/data/alip_mpc_paper/sim_stair_log_up'
stairs_up_yamlpath = stairs_up_logpath + '.yaml'
stairs_down_logpath = '/home/brian/workspace/data/alip_mpc_paper/sim_stair_log_down'
stairs_down_yamlpath = stairs_down_logpath + '.yaml'

outfolder = '/home/brian/workspace/manuscripts/tech_report/figures/'


def get_velocity_profile_data(data, plant, state_channel, cassie_out_channel, vel_scale, t_offset):
    robot_output = mbp_plots.process_state_channel(data[state_channel], plant)
    robot_output['t_x'] -= robot_output['t_x'][0]
    nvel = len(data[cassie_out_channel])
    vel = np.zeros((nvel,))
    for i, msg in enumerate(data[cassie_out_channel]):
        vel[i] = vel_scale * msg.channel[0]
    t = np.linspace(
        robot_output['t_x'][0] + t_offset,
        robot_output['t_x'][-1],
        nvel
    )
    return robot_output, {'t_vdes': t, 'vdes': vel}


def make_velocity_tracking_plot(plant, context, robot_output, vdes, title, fname=None):
    fb_vel = mbp_plots.get_floating_base_velocity_in_body_frame(
        robot_output, plant, context,
        plant.GetBodyByName("pelvis").body_frame()
    )
    v_actual = fb_vel[:,0]

    ps = PlotStyler(directory=outfolder)
    ps.plot(robot_output['t_x'], v_actual)
    ps.plot(
        vdes['t_vdes'],
        vdes['vdes'],
        linestyle='dashed',
        xlabel='Time (s)',
        ylabel='Velocity (m/s)',
        title=title,
        ylim=[-0.05, 1.4]
    )
    ps.add_legend(['Pelvis Velocity $v_{x}$', ' Desired Velocity $v_{d}$'])

    if fname:
        ps.save_fig(fname)


def log_main(logname, yamlname, plot_title, meshcat=False):
    vel_scale = 1.5
    channel_x = "CASSIE_STATE_SIMULATION"
    channel_radio = "CASSIE_VIRTUAL_RADIO"
    num_poses = 22

    vel_plot_channels = {
        channel_x: dairlib.lcmt_robot_output,
        channel_radio: dairlib.lcmt_radio_out
    }

    lcmlog = lcm.EventLog(logname, "r")

    plant, context = cassie_plots.make_plant_and_context(True, True)

    robot_output, vel_cmd = get_log_data_from_first_message(
        lcmlog, vel_plot_channels, channel_radio, -0.5, 40,
        get_velocity_profile_data,  # processing callback
        plant, channel_x, channel_radio, vel_scale, 0.5)

    fname = logname.split('/')[-1] + '.png'
    make_velocity_tracking_plot(
        plant,
        context,
        robot_output,
        vel_cmd,
        plot_title,
        fname
    )
    if meshcat:
        multipose_visualizer_main(robot_output, yamlname, num_poses)
        input('Press Enter To Continue')


def main():
    PlotStyler.set_default_styling()

    disp_meshcat = len(sys.argv) > 1
    log_main(
        stairs_down_logpath,
        stairs_down_yamlpath,
        'Velocity Tracking - Descending',
        disp_meshcat
    )
    log_main(
        stairs_up_logpath,
        stairs_up_yamlpath,
        'Velocity Tracking - Ascending',
        disp_meshcat
    )


if __name__ == '__main__':
    main()