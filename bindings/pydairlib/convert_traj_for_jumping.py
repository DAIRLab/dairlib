import matplotlib.pyplot as plt
from pydairlib.lcm_trajectory import LcmTrajectory
from pydairlib.lcm_trajectory import Trajectory
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable

def main():

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
        "/cassie_fixed_springs.urdf")
    plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    # relevant MBP parameters
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = plant.num_positions() + plant.num_velocities()


    l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
    r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
    pelvis_frame = plant.GetBodyByName("pelvis").body_frame()
    world = plant.world_frame()

    # loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrajs = LcmTrajectory()
    loadedTrajs.loadFromFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/March_3_jumping")
    traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0")
    traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1")
    traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2")

    times = np.concatenate((traj_mode0.time_vector, traj_mode1.time_vector,
                            traj_mode2.time_vector))
    x = np.concatenate((traj_mode0.datapoints.T[:, 0:nx],
                        traj_mode1.datapoints.T[:, 0:nx],
                        traj_mode2.datapoints.T[:, 0:nx]))
    n_points = times.size

    context = plant.CreateDefaultContext()
    # CoM_pos = np.zeros((3, n_points))
    # CoM_vel = np.zeros((3, n_points))
    l_foot_state = np.zeros((6, n_points))
    r_foot_state = np.zeros((6, n_points))
    pelvis_state = np.zeros((6, n_points))
    # l_foot_vel = np.zeros((3, n_points))
    # r_foot_vel = np.zeros((3, n_points))


    no_offset = np.zeros(3)

    for i in range(traj_mode0.datapoints.shape[0]):
        plant.SetPositionsAndVelocities(context, x[i, :])
        # CoM_pos[:, i] = plant.CalcCenterOfMassPosition(context)
        # CoM_vel[:, i] = plant.CalcCenterOfMassJacobian(context) @ x[i, nq:nx]
        # import pdb; pdb.set_trace()
        l_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, l_toe_frame,
                                                       no_offset, world)
        r_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, r_toe_frame,
                                                       no_offset, world)
        pelvis_state[0:3, [i]] = plant.CalcPointsPositions(context,pelvis_frame,
                                                       no_offset, world)

        l_foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable.kV, l_toe_frame, no_offset, world,
            world) @ x[i, nq:nx]
        r_foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable.kV, r_toe_frame, no_offset, world,
            world) @ x[i, nq:nx]
        pelvis_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable.kV, pelvis_frame, no_offset, world,
            world) @ x[i, nq:nx]

    lfoot_traj_block = Trajectory()
    lfoot_traj_block.traj_name = "left_foot_trajectory"
    lfoot_traj_block.datapoints = l_foot_state
    lfoot_traj_block.time_vector = times
    lfoot_traj_block.datatypes = ["lfoot_x", "lfoot_y", "lfoot_z"]

    rfoot_traj_block = Trajectory()
    rfoot_traj_block.traj_name = "right_foot_trajectory"
    rfoot_traj_block.datapoints = l_foot_state
    rfoot_traj_block.time_vector = times
    rfoot_traj_block.datatypes = ["rfoot_x", "rfoot_y", "rfoot_z"]

    pelvis_traj_block = Trajectory()
    pelvis_traj_block.traj_name = "right_foot_trajectory"
    pelvis_traj_block.datapoints = l_foot_state
    pelvis_traj_block.time_vector = times
    pelvis_traj_block.datatypes = ["rfoot_x", "rfoot_y", "rfoot_z"]

    trajectories = [lfoot_traj_block, rfoot_traj_block, pelvis_traj_block]
    trajectory_names = ["left_foot_trajectory", "right_foot_trajectory",
                        "pelvis_trajectory"]

    processed_traj = LcmTrajectory(trajectories, trajectory_names,
                                   "jumping_trajectory", "Feet trajectories "
                                                         "for Cassie jumping")

    processed_traj.writeToFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping"
        "/saved_trajs/March_3_jumping_processed")



if __name__ == "__main__":
    main()
