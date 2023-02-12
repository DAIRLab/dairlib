import sys
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.all import PiecewisePolynomial, Box, RigidTransform, Meshcat

import numpy as np

from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Simulator,
                         SceneGraph, MultibodyPlant)
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, \
  MeshcatVisualizerParams
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.multibody import MultiposeVisualizer, \
  ConnectTrajectoryVisualizer, CreateWithSpringsToWithoutSpringsMapPos, \
  CreateWithSpringsToWithoutSpringsMapVel
from visualize_params import DirconVisualizationParams


def main():
  # visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/long_jump.yaml'
  visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/box_jump.yaml'
  # visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/down_jump.yaml'
  # visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/jump.yaml'
  params = DirconVisualizationParams(visualization_config_file)

  builder = DiagramBuilder()
  scene_graph_wo_spr = builder.AddSystem(SceneGraph())
  scene_graph_w_spr = builder.AddSystem(SceneGraph())
  plant_wo_spr = MultibodyPlant(1e-3)
  plant_w_spr = MultibodyPlant(1e-3)
  AddCassieMultibody(plant_wo_spr, scene_graph_wo_spr,
                     True, params.fixed_spring_urdf,
                     False, False, False)
  AddCassieMultibody(plant_w_spr, scene_graph_w_spr,
                     True, params.spring_urdf,
                     False, False, False)
  plant_wo_spr.Finalize()
  plant_w_spr.Finalize()

  pos_spr_map = CreateWithSpringsToWithoutSpringsMapPos(plant_w_spr,
                                                        plant_wo_spr)
  vel_spr_map = CreateWithSpringsToWithoutSpringsMapVel(plant_w_spr,
                                                        plant_wo_spr)



  nq = plant_wo_spr.num_positions()
  nv = plant_wo_spr.num_velocities()
  nx = nq + nv
  nq_spr = plant_w_spr.num_positions()
  nv_spr = plant_w_spr.num_velocities()
  nx_spr = nq_spr + nv_spr
  state_spr_map = np.zeros((nx_spr, nx))
  state_spr_map[:nq_spr, :nq] = pos_spr_map.T
  state_spr_map[-nv_spr:, -nv:] = vel_spr_map.T


  filename = FindResourceOrThrow(params.filename)
  dircon_traj = lcm_trajectory.DirconTrajectory(plant_wo_spr, filename)

  optimal_traj = dircon_traj.ReconstructStateTrajectory()
  if params.use_springs:
    optimal_traj = dircon_traj.ReconstructStateTrajectoryWithSprings(state_spr_map)
  t_vec = optimal_traj.get_segment_times()

  vis_urdf = params.fixed_spring_urdf
  vis_plant = plant_wo_spr
  vis_scene_graph = scene_graph_wo_spr
  nx_vis = nx
  if params.use_springs:
    vis_urdf = params.spring_urdf
    nx_vis = nx_spr
    vis_plant = plant_w_spr
    vis_scene_graph = scene_graph_w_spr

  if params.visualize_mode == 0 or params.visualize_mode == 1:

    ConnectTrajectoryVisualizer(vis_plant, builder, vis_scene_graph,
                                optimal_traj)
    meschat_params = MeshcatVisualizerParams()
    meschat_params.publish_period = 1.0 / 60.0
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(
      builder, vis_scene_graph, meshcat, meschat_params)
    diagram = builder.Build()

    while params.visualize_mode == 1:
      simulator = Simulator(diagram)
      simulator.set_target_realtime_rate(params.realtime_rate)
      simulator.Initialize()
      simulator.AdvanceTo(optimal_traj.end_time())

  elif params.visualize_mode == 2:
    poses = np.zeros((params.num_poses, nx_vis))
    for i in range(params.num_poses):
      poses[i] = optimal_traj.value(
        t_vec[int(i * len(t_vec) / params.num_poses)])[:, 0]
      # poses[i, 6] += 0.5
    # alpha_scale = np.linspace(0.2, 1.0, params.num_poses)
    alpha_scale = np.linspace(1.0, 1.0, params.num_poses)
    visualizer = MultiposeVisualizer(FindResourceOrThrow(
      vis_urdf),
      params.num_poses, np.square(alpha_scale), "")
    # translation = np.array([-0.25, 0, 0.25])
    ortho_camera = Meshcat.OrthographicCamera()
    ortho_camera.top = 1
    ortho_camera.bottom = -0.1
    ortho_camera.left = -1
    ortho_camera.right = 2
    ortho_camera.near = -10
    ortho_camera.far = 500
    ortho_camera.zoom = 1
    translation = np.array([0.5, 0, 0.25])
    origin = RigidTransform(translation)
    box = Box(0.5, 1.0, 0.5)
    # visualizer.GetMeshcat().SetObject("box", box)
    # visualizer.GetMeshcat().SetTransform("box", origin)
    visualizer.GetMeshcat().SetCamera(ortho_camera)
    visualizer.DrawPoses(poses.T)
    while(True):
      continue


if __name__ == "__main__":
  main()
