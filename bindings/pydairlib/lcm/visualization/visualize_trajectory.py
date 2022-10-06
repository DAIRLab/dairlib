import sys
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Simulator, SceneGraph, MultibodyPlant)
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, MeshcatVisualizerParams
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.multibody import MultiposeVisualizer, ConnectTrajectoryVisualizer
from visualize_params import DirconVisualizationParams


def main():
    visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/long_jump.yaml'
    params = DirconVisualizationParams(visualization_config_file)

    builder = DiagramBuilder()
    scene_graph_wo_spr = builder.AddSystem(SceneGraph())
    # plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
    plant_wo_spr = MultibodyPlant(0.0)
    AddCassieMultibody(plant_wo_spr, scene_graph_wo_spr,
                       True, "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       False, False)

    plant_wo_spr.Finalize()
    plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
    AddCassieMultibody(plant_w_spr, scene_graph_w_spr,
                       True, "examples/Cassie/urdf/cassie_v2_shells.urdf",
                       False, False)

    plant_w_spr.Finalize()

    nq = plant_wo_spr.num_positions()
    nv = plant_wo_spr.num_velocities()
    nx = nq + nv

    filename = FindResourceOrThrow(params.filename)

    dircon_traj = lcm_trajectory.DirconTrajectory(plant_wo_spr, filename)

    optimal_traj = dircon_traj.ReconstructStateTrajectory()
    t_vec = optimal_traj.get_segment_times()

    if params.visualize_mode == 0 or params.visualize_mode == 1:
        ConnectTrajectoryVisualizer(plant_wo_spr, builder, scene_graph_wo_spr,
                                    optimal_traj)
        meschat_params = MeshcatVisualizerParams()
        meschat_params.publish_period = 1.0/60.0
        meshcat = StartMeshcat()
        visualizer = MeshcatVisualizer.AddToBuilder(
            builder, scene_graph_wo_spr, meshcat, meschat_params)
        diagram = builder.Build()

        while params.visualize_mode == 1:
            simulator = Simulator(diagram)
            simulator.set_target_realtime_rate(params.realtime_rate)
            simulator.Initialize()
            simulator.AdvanceTo(optimal_traj.end_time())

    elif params.visualize_mode == 2:
        poses = np.zeros((params.num_poses, nx))
        for i in range(params.num_poses):
            poses[i] = optimal_traj.value(t_vec[int(i * len(t_vec) / params.num_poses)])[:, 0]
        alpha_scale = np.linspace(0.2, 1.0, params.num_poses)
        visualizer = MultiposeVisualizer(FindResourceOrThrow(
            params.fixed_spring_urdf),
            params.num_poses, np.square(alpha_scale), "")
        visualizer.DrawPoses(poses.T)


if __name__ == "__main__":
    main()
