try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import sys
import rospy
import apriltag
import datetime
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from tf_bag import BagTfTransformer

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from pydrake.common.eigen_geometry import Quaternion
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.math import RigidTransform, RotationMatrix

TOE_REAR = np.array([0.088, 0, 0])
TOE_FRONT = np.array([-0.0457, 0.112, 0])


# Takes a data dictionary containing N datapoints with the entries {
#   'N': number of data points
#   'world_points': Nx3 numpy array, each row is a point in world frame
#   'camera points': Nx3 numpy array, corresponding points in camera frame
#   'pelvis_orientations': list of pelvis orientations in the world frame as drake Rotation matrices
#   'pelvis positions': Nx3 nupy array, each row is the position of the pelvis in the world frame
# }
def find_camera_pose_by_constrained_optimization(data):
    prog = MathematicalProgram()
    R = prog.NewContinuousVariables(3, 3, "R")
    p = prog.NewContinuousVariables(3, 1, "p")
    X_PC = np.hstack([R, p])

    N = data['N']
    XT = data['world_points'] - data['pelvis_positions']
    for i in range(N):
        XT[i] = data['pelvis_orientations'][i].inverse().multiply(XT[i])
    X = XT.T
    Y = np.hstack([data['camera_points'], np.ones((N, 1))]).T

    linear = False
    for i in range(N):
        if linear:
            t = prog.NewContinuousVariables(3, f"t_{i}")
            for j in range(3):
                prog.AddLinearConstraint(t[j] >= 0)
                prog.AddLinearConstraint((X[:, i] - X_PC @ Y[:, i]).ravel()[j] <= t[j])
                prog.AddLinearConstraint((X_PC @ Y[:, i] - X[:, i]).ravel()[j] <= t[j])
                prog.AddLinearCost(t[j])
        else:
            prog.AddQuadraticCost((X[:, i] - X_PC @ Y[:, i]).T @
                                  (X[:, i] - X_PC @ Y[:, i]))

    # prog.AddQuadraticCost(np.trace((X - X_PC @ Y) @ (X - X_PC @ Y).T))
    orthonormal_constraint = (R.T @ R).reshape((-1, 1)) - np.eye(3).reshape((-1, 1))
    orthonormal_constraint = orthonormal_constraint.ravel()
    [prog.AddConstraint(orthonormal_constraint[i], 0, 0) for i in range(9)]

    prog.SetInitialGuess(
        R,
        RotationMatrix.MakeYRotation(0.8 * np.pi / 2).matrix()
    )
    prog.SetInitialGuess(p, np.array([0.15, 0, 0.17]))
    sol = Solve(prog)

    if sol.is_success():
        print(f"\n\nSolution found! the total normalized cost is"
              f" {sol.get_optimal_cost() / data['N']}")
    else:
        print(f"\n\nSysID failed with code {sol.get_solution_result()}")

    return RigidTransform(
        RotationMatrix(sol.GetSolution(R)),
        sol.GetSolution(p)
    )


@dataclass
class CalibrationParams:
    apriltag_family: str = "t36h11"
    margin: float = 0.025
    tag_size: float = 0.0745
    start_id: int = 78
    ntags: int = 50
    width: int = 10
    height: int = 5
    valid_pose_error_threshold: float = 0.02
    board_pose_in_world_frame: RigidTransform = RigidTransform.Identity()


# returns a dictionary mapping the tag id to that tag's position in the
# calibration board frame
# assumes cassie's right foot is on the start_id tag and there are 6 tags
def get_tag_positions_on_board(params):
    tag_positions = {}

    cell_size = params.tag_size + params.margin
    o = np.array([0.5 * cell_size, 2 * cell_size, 0])
    for i in range(params.ntags):
        x_cell = params.width - (i % params.width) - 1
        y_cell = params.height - (i // params.width) - 1
        tag_positions[params.start_id + i] = \
            np.array([x_cell * cell_size, y_cell * cell_size, 0]) - o
    return tag_positions


# Extracts data from rosbags to assemble the data dictionary used by
# find_camera_pose_by_constrained_optimization
def extract_calibration_data(hardware_rosbag_path, calibration_params,
                             time_offset, cutoff_time):

    # Load the processed (distortion corrected) camera messages
    rosbag_hardware = rosbag.Bag(hardware_rosbag_path)

    # Create an apriltag detector
    detector = apriltag.Detector()

    # get the first camera info message
    _, camera_info, _ = next(
        rosbag_hardware.read_messages(topics=['camera/color/camera_info'])
    )
    _, _, t = \
        next(rosbag_hardware.read_messages(topics=["camera/color/image_rect"]))
    start_time = t.to_sec()

    # Get the intrinsic matrix [fx 0 cx]
    #                          [0 fy cy]
    #                          [0  0  1]
    # and map to apriltag intrinsics [fx, fy, cx, cy]
    intrinsics = np.reshape(camera_info.K, (3, 3))
    apriltag_intrinsics = [intrinsics[0, 0],
                           intrinsics[0, 2],
                           intrinsics[1, 1],
                           intrinsics[1, 2]]

    timestamped_apriltag_poses = {}

    # Detect the apriltags in the (rectified) input images
    print("Detecting apriltags...")
    for topic, msg, t in \
        rosbag_hardware.read_messages(
            topics=["camera/color/image_rect"]):

        if (t.to_sec() - start_time) > cutoff_time:
            break

        valid_poses = get_valid_apriltag_poses(
            detector, msg, apriltag_intrinsics, calibration_params
        )
        if valid_poses:
            timestamped_apriltag_poses[
                msg.header.stamp + rospy.Duration(time_offset)] = valid_poses

    rosbag_hardware.close()
    # Get the pelvis transform for the timestamps with valid apriltag poses
    print("Getting pelvis pose data at apriltag pose timestamps...")
    timestamped_pelvis_poses = collect_transforms(
        "map",
        "pelvis",
        timestamped_apriltag_poses.keys(),
        hardware_rosbag_path
    )
    timestamped_left_toe_poses = collect_transforms(
        "map",
        "toe_left",
        timestamped_apriltag_poses.keys(),
        hardware_rosbag_path
    )
    timestamped_right_toe_poses = collect_transforms(
        "map",
        "toe_right",
        timestamped_apriltag_poses.keys(),
        hardware_rosbag_path
    )
    timestamped_apriltag_poses = dict(
        (k, timestamped_apriltag_poses[k]) for k in timestamped_pelvis_poses.keys()
    )
    rosbag_hardware.close()

    print(f"Assembling data matrix from {len(timestamped_apriltag_poses)} "
          f"sets of poses...")
    poses = {
        "pelvis": timestamped_pelvis_poses,
        "toe_left": timestamped_left_toe_poses,
        "toe_right": timestamped_right_toe_poses,
        "apriltags": timestamped_apriltag_poses
    }
    return poses, collate_data(timestamped_apriltag_poses,
                               timestamped_pelvis_poses,
                               timestamped_left_toe_poses,
                               timestamped_right_toe_poses,
                               calibration_params)


# produces a data dictionary containing N datapoints with the entries {
#   'N': number of data points
#   'world_points': Nx3 numpy array, each row is a point in world frame
#   'camera points': Nx3 numpy array, corresponding points in camera frame
#   'pelvis_orientations': list of pelvis poses in the world frame as drake Rotation matrices
#   'pelvis positions': Nx3 nupy array, each row is the position of the pelvis in the world frame
#
def collate_data(timestamped_apriltag_poses, timestamped_pelvis_poses,
                 timestamped_left_toe_poses, timestamped_right_toe_poses,
                 calibration_params):
    N = 0
    data = {}

    for pose_dict in timestamped_apriltag_poses.values():
        N += len(pose_dict)

    data['N'] = N
    data['world_points'] = np.zeros((N, 3))
    data['camera_points'] = np.zeros((N, 3))
    data['pelvis_orientations'] = []
    data['pelvis_positions'] = np.zeros((N, 3))

    tag_positions = get_tag_positions_on_board(calibration_params)

    row_idx = 0
    for timestamp in timestamped_apriltag_poses.keys():
        X_WR = timestamped_right_toe_poses[timestamp]
        X_WL = timestamped_left_toe_poses[timestamp]

        right_toe_rear = X_WR.multiply(TOE_REAR).ravel()
        left_toe_rear = X_WL.multiply(TOE_REAR).ravel()
        right_toe_front = X_WR.multiply(TOE_FRONT).ravel()
        left_toe_front = X_WL.multiply(np.array([-0.0457, 0.112, 0])).ravel()

        board_origin_in_world = 0.25 * (
                right_toe_front + right_toe_rear +
                left_toe_front + left_toe_rear)

        # board_origin_in_world = 0.5 * (right_toe_front + right_toe_rear) - \
        #                         tag_positions[calibration_params.start_id]

        board_x_in_world = (right_toe_front - right_toe_rear)

        board_yaw_in_world = np.arctan2(
            board_x_in_world[1],
            board_x_in_world[0])

        X_WP = timestamped_pelvis_poses[timestamp]
        R_WB = RotationMatrix.MakeZRotation(board_yaw_in_world)
        X_WB = RigidTransform(R_WB, R_WB.multiply(board_origin_in_world))

        for tag_id, pose in timestamped_apriltag_poses[timestamp].items():
            data['world_points'][row_idx] = X_WB.multiply(tag_positions[tag_id]).ravel()
            data['camera_points'][row_idx] = pose.translation().ravel()
            data['pelvis_orientations'].append(X_WP.rotation())
            data['pelvis_positions'][row_idx] = X_WP.translation().ravel()
            row_idx += 1
    return data


def get_valid_apriltag_poses(detector, msg, intrinsics, calibration_params):
    img = np.frombuffer(msg.data, dtype=np.uint8). \
        reshape(msg.height, msg.width)
    tags = detector.detect(img)
    valid_poses = {}
    for tag in tags:
        pose = detector.detection_pose(
            tag,
            intrinsics,
            tag_size=calibration_params.tag_size)
        if pose[-1] < calibration_params.valid_pose_error_threshold:
            valid_poses[tag.tag_id] = RigidTransform(pose[0])

    return valid_poses


def collect_transforms(parent_frame, child_frame, times, fname):
    transforms = {}
    bag_transformer = BagTfTransformer(fname)
    for time in times:
        try:
            translation, quat = \
                bag_transformer.lookupTransform(parent_frame, child_frame, time)
            transforms[time] = RigidTransform(
                Quaternion(quat[-1], quat[0], quat[1], quat[2]),
                np.array(translation)
            )
        except RuntimeError:
            continue
    return transforms


def plot_calibration_error(data, X_PC):
    N = data['N']
    camera_points_in_world = np.zeros((N, 3))
    for i in range(N):
        camera_points_in_world[i] = data['pelvis_positions'][i] + \
                                    data['pelvis_orientations'][i].multiply(
                                        X_PC.multiply(data['camera_points'][i])
                                    )
    error = data['world_points'] - camera_points_in_world
    plt.plot(error)
    plt.legend(['x', 'y', 'z'])
    plt.show()


def rigid_transform_to_tf_stamped(frame_id, child_frame_id, timestamp, X_AB):
    msg = TransformStamped()

    # metadata
    msg.header.stamp = timestamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    # transform
    translation = X_AB.translation().ravel()
    rotation = X_AB.rotation().ToQuaternion()
    msg.transform.translation.x = translation[0]
    msg.transform.translation.y = translation[1]
    msg.transform.translation.z = translation[2]
    msg.transform.rotation.x = rotation.x()
    msg.transform.rotation.y = rotation.y()
    msg.transform.rotation.z = rotation.z()
    msg.transform.rotation.w = rotation.w()
    return msg


def plot_z_over_time(poses):
    t0 = next(iter(poses["apriltags"])).to_sec()
    N = len(poses["apriltags"].keys())
    t = np.zeros((N,))
    zp = np.zeros((N,))
    za = np.zeros((N,))
    for i, timestamp in enumerate(poses["apriltags"].keys()):
        t[i] = timestamp.to_sec() - t0
        zp[i] = poses["pelvis"][timestamp].translation().ravel()[-1]
        za[i] = next(iter(poses["apriltags"][timestamp].items()))[1].translation().ravel()[-1]

    plt.plot(t, zp)
    plt.plot(t, za)


def write_bag_for_calibration_playback(poses, X_PC, bag_path):
    with rosbag.Bag(bag_path, 'w') as bag:
        for timestamp in poses["apriltags"].keys():
            tf_msg = TFMessage()
            tf_msg.transforms = []
            for key in ["pelvis", "toe_left", "toe_right"]:
                tf_msg.transforms.append(
                    rigid_transform_to_tf_stamped(
                        "map", key, timestamp, poses[key][timestamp]
                    )
                )
            tf_msg.transforms.append(rigid_transform_to_tf_stamped(
                "pelvis", "camera_frame", timestamp, X_PC
            ))
            for tag_id in poses["apriltags"][timestamp].keys():
                tf_msg.transforms.append(
                    rigid_transform_to_tf_stamped(
                        "camera_frame", f"apriltag_{tag_id}", timestamp,
                        poses["apriltags"][timestamp][tag_id]
                    )
                )
            bag.write('/tf', tf_msg, timestamp)


def main():
    hardware_fname = sys.argv[1]
    time_offset = 0.0
    cutoff_time = 100
    poses, data = extract_calibration_data(
        hardware_fname,
        CalibrationParams(),
        time_offset,
        cutoff_time
    )
    X_PC = find_camera_pose_by_constrained_optimization(data)
    X_CD = RigidTransform(RotationMatrix(), np.array([-0.059, 0, 0]))
    X_PD = X_PC.multiply(X_CD)

    print(f'Camera color optical frame:\n{X_PC}')
    print(f'Camera depth optical frame:\n{X_PD}')
    print("Writing validation bag...")
    plot_z_over_time(poses)
    plt.show()
    write_bag_for_calibration_playback(poses, X_PC, hardware_fname+".validate")
    print("Done.")


if __name__ == "__main__":
    main()
