try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import sys
import numpy as np
from tf_bag import BagTfTransformer
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
from sklearn.linear_model import RANSACRegressor


def get_points(pc2_msg):
    points_list = []
    for data in pc2.read_points(pc2_msg, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])
    return np.array(points_list)


def fit_plane(points):
    points_copy = np.copy(points)
    mask = np.any(np.isinf(points_copy), axis=-1)
    points_copy = points_copy[~mask, :]
    ransac = RANSACRegressor(residual_threshold=0.001)
    xy = points_copy[:, :2]
    z = points_copy[:, 2]
    try:
        ransac.fit(xy, z)
    except:
        import pdb; pdb.set_trace()
    a, b = ransac.estimator_.coef_
    c = ransac.estimator_.intercept_
    normal = np.array([a, b, -1.0])
    norm = np.linalg.norm(normal)
    print(normal / norm)
    return {"normal": normal / norm, "d": -c / norm}


def get_ground_normals(bagpath: str):
    bag = rosbag.Bag(bagpath)
    normals = []
    times = []
    points = []

    for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
        cloud = get_points(msg)
        plane = fit_plane(cloud)
        normals.append(plane["normal"])
        times.append(t)
        points.append(cloud)

    bag.close()
    return {"points": points, "normals": normals, "t": times}


def collect_transforms(parent_frame, child_frame, times, fname):
    transforms = {"translations": [], "rotations": []}
    bag_transformer = BagTfTransformer(fname)
    for time in times:
        translation, quat = \
            bag_transformer.lookupTransform(parent_frame, child_frame, time)
        transforms["translations"].append(translation)
        transforms["rotations"].append(Rotation.from_quat(quat))
    return transforms


def find_camera_orientation_in_frame_P(normals, R_WPs, world_normal):
    b = np.vstack([world_normal.T @ R.as_matrix() for R in R_WPs])
    R_CP = np.linalg.lstsq(normals, b, rcond=None)[0]
    return R_CP.T


def main():
    fname = sys.argv[1]
    normal_data = get_ground_normals(fname)
    tfs = collect_transforms(
        "map",
        "pelvis",
        normal_data["t"],
        fname
    )
    rotation = find_camera_orientation_in_frame_P(
        np.vstack(normal_data["normals"]),
        tfs["rotations"],
        np.array([0, 0, 1])
    )
    import pdb; pdb.set_trace()


if __name__ == "__main__":
    main()
