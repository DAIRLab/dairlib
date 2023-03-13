import sys
import rosbag
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix

from pydairlib.common import FindResourceOrThrow
from pydairlib.systems.cameras import ReadCameraPoseFromYaml
from pydrake.all import RigidTransform


def calib_to_tf_msg(calibration_yaml):
    camera_pose = ReadCameraPoseFromYaml(
        FindResourceOrThrow(calibration_yaml)
    )
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = camera_pose.rotation().matrix()
    translation = camera_pose.translation().ravel()
    transform_msg = TransformStamped()
    transform_msg.transform.translation.x = translation[0]
    transform_msg.transform.translation.y = translation[1]
    transform_msg.transform.translation.z = translation[2]
    transform_msg.transform.rotation = quaternion_from_matrix(rotation_matrix)
    return transform_msg


def replace_transforms_in_tf_topic(input_bag_path,
                                   output_bag_path,
                                   header_frame_id,
                                   child_frame_id,
                                   transform_msg):
    with rosbag.Bag(input_bag_path) as input_bag, \
         rosbag.Bag(output_bag_path, 'w') as output_bag:
        for topic, msg, t in input_bag.read_messages():
            if topic in ['/tf', '/tf_static']:
                for transform_stamped in msg.transforms:
                    if transform_stamped.header.frame_id == header_frame_id \
                     and transform_stamped.child_frame_id == child_frame_id:
                        # Replace transform with user-specified transform
                        transform_stamped.transform = transform_msg.transform
                output_bag.write(topic, msg, t)
            else:
                output_bag.write(topic, msg, t)


def main():
    filename_to_process = sys.argv[1]
    filename_processed = filename_to_process + ".recalibrated"
    calibration_yaml = \
        "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml"
    parent_frame = "pelvis"
    child_frame = "camera_depth_optical_frame"
    replace_transforms_in_tf_topic(
        filename_to_process,
        filename_processed,
        parent_frame,
        child_frame,
        calib_to_tf_msg(calibration_yaml)
    )


if __name__ == '__main__':
    main()
