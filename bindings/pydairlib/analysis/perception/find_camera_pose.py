import pcl
try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import numpy as np
import sensor_msgs.point_cloud2 as pc2


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data


def get_ground_planes(bagpath: str):
    bag = rosbag.Bag(bagpath)
    planes = []

    topics = []
    for topic, _, _ in bag.read_messages():
        if topic not in topics:
            topics.append(topic)
    print("topics")
    [print(f"{topic}") for topic in topics]

    for _, msg, _ in bag.read_messages(topics=['/camera/depth/color/points']):
        cloud = ros_to_pcl(msg)
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.1)
        inliers, model = seg.segment()
        planes.append({"points": cloud,
                       "inliers": inliers,
                       "plane": model}) # append plane equation in matrix form to list

    bag.close()
    return planes


if __name__ == "__main__":
    planes = get_ground_planes("/home/brian/workspace/logs/ros/2022-12-02-15-32-17.bag")
    for i in planes[0]["inliers"]:
        planes[0]["points"][i][3] = 255 << 16 | 255 << 8 | 0 # set to yellow

    visual_cloud = pcl.PointCloud_PointXYZRGB()
    visual_cloud.from_array(planes[0]["points"])
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowColorCloud(visual_cloud)
    v = True
    while v:
        v = not(visual.WasStopped())