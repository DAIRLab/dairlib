import numpy as np
import time
from pydrake.all import (
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    Trajectory,
    DiagramBuilder,
    TrajectorySource,
    LcmPublisherSystem,
    DrakeLcm,
    AbstractValue,
    Simulator,
    ZeroOrderHold,
    TriggerType,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    FixedOffsetFrame,
    DrakeLcm,
)
import lcm
from pydrake.all import LeafSystem, BasicVector
from dairlib import lcmt_robot_output, lcmt_timestamped_saved_traj, lcmt_saved_traj, lcmt_trajectory_block


robot_output = None
def my_handler(data):
    print("Handling data")
    global robot_output
    robot_output = lcmt_robot_output.decode(data)

lc = DrakeLcm("udpm://239.255.76.67:7667?ttl=0")
sub = lc.Subscribe("FRANKA_STATE", my_handler)
lc.HandleSubscriptions(10)

# Define the times and control points for the trajectory.
print(robot_output.utime)
time_in_seconds = robot_output.utime / 1e6
times = [time_in_seconds, time_in_seconds + 10.0, time_in_seconds + 20.0]

plant = MultibodyPlant(0.0)
parser = Parser(plant)
parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("panda_link0"), RigidTransform.Identity()
)
plant.AddFrame(
    FixedOffsetFrame(
        "end_effector_frame", plant.GetBodyByName("panda_link7"), RigidTransform(RotationMatrix(RollPitchYaw(np.pi, 0, -np.pi/4)), np.array([0, 0, 0.107]))
    )
)
plant.Finalize()

context = plant.CreateDefaultContext()
plant.SetPositions(context, robot_output.position)
ee_pose = plant.CalcRelativeTransform(
    context,
    plant.world_frame(),
    plant.GetFrameByName("end_effector_frame"),
)
J0 = np.concatenate([ee_pose.translation(), ee_pose.rotation().ToRollPitchYaw().vector()]).reshape(-1, 1)
J1 = J0 + np.array([0.1, 0.1, 0.1, 0.0, 0, 0.0]).reshape(-1, 1)


control_points = np.hstack((J0, J1, J0))
print(control_points)

# Create the trajectory object.
lcm_trajectory = lcmt_saved_traj()
lcm_trajectory.num_trajectories = 2

translation_trajectory = lcmt_trajectory_block()
translation_trajectory.trajectory_name = "end_effector_translation_target"
translation_trajectory.num_points = len(times)
translation_trajectory.num_datatypes = 3
translation_trajectory.time_vec = times
translation_trajectory.datapoints = control_points[:3, :]
translation_trajectory.datatypes = ["double"] * 3

lcm_trajectory.trajectories.append(translation_trajectory)
lcm_trajectory.trajectory_names.append("end_effector_translation_target")


quats = [RollPitchYaw(rpy).ToQuaternion().wxyz().reshape(-1, 1) for rpy in control_points[3:6, :].T]
breaks = np.hstack(quats)

rotation_trajectory = lcmt_trajectory_block()
rotation_trajectory.trajectory_name = "end_effector_orientation_target"
rotation_trajectory.num_points = len(times)
rotation_trajectory.num_datatypes = 4
rotation_trajectory.time_vec = times
rotation_trajectory.datapoints = breaks
rotation_trajectory.datatypes = ["double"] * 4

lcm_trajectory.trajectories.append(rotation_trajectory)
lcm_trajectory.trajectory_names.append("end_effector_orientation_target")

timestamped_trajectory = lcmt_timestamped_saved_traj()
timestamped_trajectory.utime = int(time.time() * 1e6)
timestamped_trajectory.saved_traj = lcm_trajectory

lc.Publish("TARGET_CARTESIAN_POSE_TRAJECTORY", timestamped_trajectory.encode())