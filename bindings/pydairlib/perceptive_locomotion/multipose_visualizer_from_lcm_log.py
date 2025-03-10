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
    RotationMatrix,
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

from pydairlib.systems import GridMapVisualizer
from pydairlib.geometry import ConvexPolygonVisualizer
from pydairlib.perceptive_locomotion.results import analysis_utils


def ground_truth_main():
    channel_x = "CASSIE_STATE_SIMULATION"
    num_poses = 6

    filename_log = sys.argv[1]
    filename_stones = sys.argv[2]
    lcmlog = lcm.EventLog(filename_log, "r")
    plant, context = cassie_plots.make_plant_and_context(True, True)
    default_channels = cassie_plots.cassie_default_channels

    robot_output = get_log_data(
        lcmlog, default_channels, 0, -1, mbp_plots.load_state_channel,  # processing callback
        plant, channel_x)

    visualizer = multipose_visualizer_main(robot_output, num_poses, 0.18, 0.6)
    visualizer.AddSteppingStonesFromYaml(filename_stones)
    while(True):
        continue


def perceptive_main():
    filename_log = sys.argv[1]
    grid_maps, robot_output = analysis_utils.get_grid_maps_from_log(filename_log)
    
    start_fraction = 0.1
    end_fraction = 0.95
    num_poses = 4
    visualizer = multipose_visualizer_main(
        robot_output,
        num_poses,
        start_fraction,
        end_fraction
    )
    
    grid_map_visualizer = GridMapVisualizer(visualizer.GetMeshcat(), 0.1, [])
    n = len(grid_maps)
    map_idx = np.linspace(
        int(start_fraction * n),
        int(end_fraction * n),
        num_poses,
        dtype=int
    )
    for idx in map_idx:
        grid_map_visualizer.DrawGridMap(grid_maps[idx], ['segmented_elevation'], f'{idx}_')
    
    while True:
        continue


def look_at(meshcat, point_of_interest, cam_pos_local):
    # point the camera at the poit of interest
    meshcat.SetCameraPose(
        point_of_interest + cam_pos_local, point_of_interest)
    
    # Set the lighting positions
    meshcat.SetTransform(
        "/Lights/PointLightPositiveX/<object>",
        RigidTransform(
            RotationMatrix(), point_of_interest + np.array([2.0, 0.0, 2.0])))
    meshcat.SetTransform(
        "/Lights/PointLightNegativeX/<object>",
        RigidTransform(
            RotationMatrix(), point_of_interest + np.array([-2.0, 0.0, 2.0])))


def multipose_visualizer_main(robot_output, num_poses, start_fraction, end_fraction):

    n = robot_output['q'].shape[0]
    q_idx = np.linspace(
        int(start_fraction * n),
        int(end_fraction * n),
        num_poses,
        dtype=int
    )
    q_idx[-1] -= 1
    poses = robot_output['q'][q_idx]

    alpha_scale = np.linspace(1.0, 1.0, num_poses)
    visualizer = MultiposeVisualizer(
        FindResourceOrThrow(cassie_plots.cassie_urdf),
        num_poses,
        np.square(alpha_scale), ""
    )
    
    look_at(visualizer.GetMeshcat(), np.array([4.0, 0.0, 0.5]), np.array([0.0, 2.0, 0.5]))
    
    visualizer.DrawPoses(poses.T)
    return visualizer


if __name__ == "__main__":
    if len(sys.argv) > 2:
        ground_truth_main()
    else:
        perceptive_main()
