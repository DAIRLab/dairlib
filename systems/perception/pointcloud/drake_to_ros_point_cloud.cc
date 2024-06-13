//
// Created by brian on 12/3/22.
//
#include "systems/perception/pointcloud/drake_to_ros_point_cloud.h"
#include "drake/perception/point_cloud.h"
#include "ros/ros.h"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace dairlib {
namespace perception {

using drake::perception::pc_flags::kRGBs;
using drake::perception::pc_flags::kXYZs;

DrakeToRosPointCloud::DrakeToRosPointCloud(std::string frame_id)
  : frame_id_(std::move(frame_id)){

  const auto model_value = drake::perception::PointCloud(0, kRGBs | kXYZs);
  DeclareAbstractInputPort(
      "drake_point_cloud",
      drake::Value<drake::perception::PointCloud>(model_value));
  DeclareAbstractOutputPort("pcl_point_cloud",
                            &DrakeToRosPointCloud::CopyInputToPCL);
}

void DrakeToRosPointCloud::CopyInputToPCL(
    const drake::systems::Context<double> &context,
   sensor_msgs::PointCloud2* cloud) const {

  sensor_msgs::PointCloud2Modifier modifier(*cloud);

  const auto& input_cloud =
      this->EvalAbstractInput(
          context, 0)->get_value<drake::perception::PointCloud>();

  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(input_cloud.size());


  const auto& xyzs = input_cloud.xyzs();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

  for (int i = 0; i < input_cloud.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = xyzs(0, i);
    *iter_y = xyzs(1, i);
    *iter_z = xyzs(2, i);
  }


  if (input_cloud.has_rgbs()) {
    const auto &rgbs = input_cloud.rgbs();
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");

    for (int i = 0; i < input_cloud.size(); ++i, ++iter_r, ++iter_g, ++iter_b) {
      *iter_r = rgbs(0, i);
      *iter_g = rgbs(1, i);
      *iter_b = rgbs(2, i);
    }
  }

  cloud->header.frame_id = frame_id_;
  cloud->header.stamp = ros::Time::now();
}

}
}