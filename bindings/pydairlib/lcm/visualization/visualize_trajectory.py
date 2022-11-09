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


def plot_trajectory(traj_of_interest, traj_name, y_dim):
    times = np.arange(traj_of_interest.start_time(), traj_of_interest.end_time(), 0.001)
    # times = np.arange(traj_of_interest.start_time(), 0.2, 0.001)
    accel = np.zeros((times.shape[0], y_dim))
    pos = np.zeros((times.shape[0], y_dim))
    vel = np.zeros((times.shape[0], y_dim))
    for i in range(times.shape[0]):
        pos[i, :] = traj_of_interest.value(times[i])[:, 0]
        vel[i, :] = traj_of_interest.EvalDerivative(times[i], 1)[:, 0]
        accel[i, :] = traj_of_interest.EvalDerivative(times[i], 2)[:, 0]

    plt.figure(traj_name + "_pos")
    plt.plot(times, pos)
    plt.legend(['x', 'y', 'z', 'xdot', 'ydot', 'zdot'])
    plt.figure(traj_name + "_vel")
    plt.plot(times, vel)
    plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
    plt.figure(traj_name + "_acc")
    plt.plot(times, accel)
    plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
    # plt.legend(['pos','vel','accel'])

def main():
    visualization_config_file = 'bindings/pydairlib/lcm/visualization/visualize_configs/long_jump.yaml'
    params = DirconVisualizationParams(visualization_config_file)

    builder = DiagramBuilder()
    scene_graph_wo_spr = builder.AddSystem(SceneGraph())
    plant_wo_spr = MultibodyPlant(0.0)
    AddCassieMultibody(plant_wo_spr, scene_graph_wo_spr,
                       True, "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       False, False)
    plant_wo_spr.Finalize()

    nq = plant_wo_spr.num_positions()
    nv = plant_wo_spr.num_velocities()
    nx = nq + nv

    filename = FindResourceOrThrow(params.filename)

    lcm_traj = lcm_trajectory.LcmTrajectory()
    lcm_traj.LoadFromFile(filename)

    lcm_state_traj = lcm_traj.GetTrajectory('state_traj')
    lcm_contact_force_traj = lcm_traj.GetTrajectory('contact_force_traj')
    state_traj = PiecewisePolynomial.FirstOrderHold(lcm_state_traj.time_vector, lcm_state_traj.datapoints[:37, :])
    contact_force_traj = PiecewisePolynomial.FirstOrderHold(lcm_contact_force_traj.time_vector, lcm_contact_force_traj.datapoints[:12, :])
    # plot_trajectory(state_traj, 'state', 37)
    # plot_trajectory(contact_force_traj, 'contact_force', 12)
    # plt.show()
    print(state_traj.end_time())
    t_vec = state_traj.get_segment_times()

    if params.visualize_mode == 0 or params.visualize_mode == 1:
        ConnectTrajectoryVisualizer(plant_wo_spr, builder, scene_graph_wo_spr,
                                    state_traj)
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
            simulator.AdvanceTo(state_traj.end_time())

    elif params.visualize_mode == 2:
        poses = np.zeros((params.num_poses, nx))
        for i in range(params.num_poses):
            poses[i] = state_traj.value(t_vec[int(i * len(t_vec) / params.num_poses)])[:, 0]
        alpha_scale = np.linspace(0.2, 1.0, params.num_poses)
        visualizer = MultiposeVisualizer(FindResourceOrThrow(
            params.fixed_spring_urdf),
            params.num_poses, np.square(alpha_scale), "")
        visualizer.DrawPoses(poses.T)


if __name__ == "__main__":
    main()
