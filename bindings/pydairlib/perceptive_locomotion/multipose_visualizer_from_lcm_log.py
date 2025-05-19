"""
    Script to visualize an LCM log with multipose visualizer
"""

import sys
import lcm
import numpy as np

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
from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.multibody import MultiposeVisualizer


def main():
    use_springs = True
    channel_x = "CASSIE_STATE_SIMULATION" # "CASSIE_STATE_DISPATCHER" # "CASSIE_STATE_SIMULATION"
    channel_u = False #"OSC_WALKING" # False #"CASSIE_INPUT"
    channel_osc = False #"OSC_DEBUG_WALKING"
    num_poses = 10

    filename_log = sys.argv[1]
    filename_stones = sys.argv[2]
    lcmlog = lcm.EventLog(filename_log, "r")
    plant, context = cassie_plots.make_plant_and_context(True, True)
    controller_plant, _ = cassie_plots.make_plant_and_context(
        floating_base=True, springs=use_springs)
    default_channels = cassie_plots.cassie_default_channels

    robot_output, robot_input, osc_debug, _ = get_log_data(
        lcmlog, default_channels, 0, -1, mbp_plots.load_default_channels,  # processing callback
        plant, controller_plant, channel_x, channel_u, channel_osc)
    # print(robot_output)
    # input("==")
    # print(robot_input)
    # input("==")
    # #print(osc_debug)
    # print(osc_debug["tracking_data"])
    # input("==")
    # t_x = robot_output['t_x']
    #vx = robot_output['v'][:,3]
    #np.save('tx.npy', t_x)
    #np.save('vx.npy', vx)
    # import matplotlib.pyplot as plt
    # from pydairlib.common.plot_styler import PlotStyler
    # t_x_slice = slice(robot_output['t_x'].size)
    # ps = PlotStyler()
    # ps.set_default_styling()
    # plot = mbp_plots.plot_floating_base_body_frame_velocities(
    #         robot_output, t_x_slice, plant, context, "pelvis")
    # plt.show()
    multipose_visualizer_main(robot_output, filename_stones, num_poses)


def multipose_visualizer_main(robot_output, fname_yaml, num_poses):

    n = robot_output['q'].shape[0]
    q_idx = np.linspace(0, n, num_poses, dtype=int)
    # q_idx = np.linspace(9000, 10000, num_poses, dtype=int)
    
    print(q_idx)
    q_idx[-1] -= 1
    q_idx[0] += 10
    # q_idx[0] += 100
    #q_idx = q_idx[:-1] 
    #q_idx = q_idx[-3:-1]
    # 10501 
    # 9000 ~ 10000
    poses = robot_output['q'][q_idx]

    alpha_scale = np.linspace(0.5, 0.5, num_poses)
    visualizer = MultiposeVisualizer(
        FindResourceOrThrow(cassie_plots.cassie_urdf),
        num_poses,
        np.square(alpha_scale), ""
    )

    # ortho_camera = Meshcat.OrthographicCamera()
    # ortho_camera.top = 1
    # ortho_camera.bottom = -0.1
    # ortho_camera.left = -1
    # ortho_camera.right = 2
    # ortho_camera.near = -10
    # ortho_camera.far = 500
    # ortho_camera.zoom = 1

    # visualizer.GetMeshcat().SetCamera(ortho_camera)
    visualizer.AddSteppingStonesFromYaml(fname_yaml)
    visualizer.DrawPoses(poses.T)
    while(True):
        continue


if __name__ == "__main__":
    main()
