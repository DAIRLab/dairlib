try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import sys
import argparse
import apriltag
import datetime
import numpy as np
from dataclasses import dataclass
from tf_bag import BagTfTransformer
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
from sklearn.linear_model import RANSACRegressor

from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RigidTransform
from pydrake.solvers import MathematicalProgram, Solve


# Takes a data dictionary containing N datapoints with the entries {
#   'N': number of data points
#   'world_points': Nx3 numpy array, each row is a point in world frame
#   'camera points': Nx3 numpy array, corresponding points in camera frame
#   'pelvis_orientations': list of pelvis poses in the world frame as drake Rotation matrices
#   'pelvis positions': Nx3 nupy array, each row is the position of the pelvis in the world frame
# }
def find_camera_pose_by_constrained_optimization(data):
    N = data['N']
    XT = data['world_points'] - data['pelvis_positions']
    for i in range(N):
        XT[i] = data['pelvis_orientations'][i].inverse().multiply(XT[i])
    X = XT.T
    Y = np.hstack([data['camera_points'], np.ones((N, 1))]).T

    prog = MathematicalProgram()
    R = prog.NewContinuousVariables(3, 3, "R")
    p = prog.NewContinuousVariables(3, 1, "p")
    X_PC = np.hstack([R, p])

    prog.AddQuadraticCost(np.trace((X - X_PC @ Y) @ (X - X_PC @ Y).T))
    prog.AddConstraint(R.T @ R == np.eye(3))
    sol = prog.Solve()


@dataclass
class CalibrationParams:
    apriltag_family: str = "t36h11"
    margin: float = 0.05
    tag_size: float = 0.174
    start_id: int = 78
    valid_pose_error_threshold: float = 0.01
    board_pose_in_world_frame: RigidTransform = RigidTransform.Identity()


# returns a dictionary mapping the tag id to that tag's position in the
# calibration board frame
# assumes cassie's right foot is on the start_id tag and there are 6 tags
def get_tag_positions_on_board(params):
    tag_positions = {}
    for i in range(6):
        x = float(i % 3) * (params.tag_size + params.margin)
        y = (params.margin + params.tag_size) / 2.0 if i > 3 else \
            -(params.margin + params.tag_size) / 2.0
        z = 0
        tag_positions[params.start_id + i] = np.array([x, y, z])
    return tag_positions


# Extracts data from rosbags to assemble the data dictionary used by
# find_camera_pose_by_constrained_optimization
def extract_calibration_data(hardware_rosbag_path, postprocessed_rosbag_path,
                             calibration_params):

    # Load the processed (distortion corrected) camera messages
    rosbag_rectified = rosbag.Bag(postprocessed_rosbag_path)

    # Create an apriltag detector
    detector = apriltag.Detector()

    # get the first camera info message
    _, camera_info, _ = next(
        rosbag_rectified.read_messages(topics=['/camera/color/camera_info'])
    )

    # Get the intrinsic matrix
    # [fx 0 cx]
    # [0 fy cy]
    # [0  0  1]
    # and map to apriltag intrinsics [fx, fy, cx, cy]
    intrinsics = np.reshape(camera_info.K, (3, 3))
    apriltag_intrinsics = [intrinsics[0, 0],
                           intrinsics[0, 2],
                           intrinsics[1, 1],
                           intrinsics[1, 2]]

    timestamped_apriltag_poses = {}

    # Detect the apriltags in the (rectified) input images
    for topic, msg, t in \
        rosbag_rectified.read_messages(topics=[
            "/camera/color/image_rect"]):

        valid_poses = get_valid_apriltag_poses(
            detector, msg, apriltag_intrinsics, calibration_params
        )
        if valid_poses:
            timestamped_apriltag_poses[msg.header.stamp] = valid_poses

    # for time in timestamped_apriltag_poses.keys():
    #     print(datetime.datetime.utcfromtimestamp(time.to_sec()).strftime("%m/%d/%Y, %H:%M:%S"))

    # Get the pelvis transform for the timestamps with valid apriltag poses
    timestamped_pelvis_poses = collect_transforms(
        "map",
        "pelvis",
        timestamped_apriltag_poses.keys(),
        hardware_rosbag_path
    )
    import pdb; pdb.set_trace()


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
    print(bag_transformer.getTransformGraphInfo())
    for time in times:
        print(
            datetime.datetime.utcfromtimestamp(time.to_sec()).
            strftime("%m/%d/%Y, %H:%M:%S")
        )
        translation, quat = \
            bag_transformer.lookupTransform(parent_frame, child_frame, time)
        transforms[time] = RigidTransform(
            Quaternion(quat.w, quat.x, quat.y, quat.z),
            translation
        )
    return transforms


def main():
    processed_fname = sys.argv[1]
    hardware_fname = sys.argv[2]
    extract_calibration_data(
        hardware_fname,
        processed_fname,
        CalibrationParams()
    )



if __name__ == "__main__":
    main()
