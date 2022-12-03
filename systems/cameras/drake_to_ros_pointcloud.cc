//
// Created by brian on 12/3/22.
//

#include "drake_to_ros_pointcloud.h"
#include "drake/perception/point_cloud.h"

#include "ros/ros.h"

namespace dairlib {
namespace camera {

using drake::perception::pc_flags::kRGBs;
using drake::perception::pc_flags::kXYZs;

DrakeToRosPointCloud::DrakeToRosPointCloud(std::string frame_id)
  : frame_id_(std::move(frame_id)){

  const auto model_value = drake::perception::PointCloud(0, kRGBs & kXYZs);
  DeclareAbstractInputPort(
      "drake_point_cloud",
      drake::Value<drake::perception::PointCloud>(model_value));
  DeclareAbstractOutputPort("pcl_point_cloud",
                            &DrakeToRosPointCloud::CopyInputToPCL);
}

void DrakeToRosPointCloud::CopyInputToPCL(
    const drake::systems::Context<double> &context,
    pcl::PointCloud<pcl::PointXYZRGB>* cloud) const {

  const auto& input_cloud =
      this->EvalAbstractInput(
          context, 0)->get_value<drake::perception::PointCloud>();

  const auto& rgbs = input_cloud.rgbs();
  const auto& xyzs = input_cloud.xyzs();

  for (int i = 0; i < input_cloud.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = xyzs(i,0);
    point.y = xyzs(i,1);
    point.z = xyzs(i,2);
    point.r = rgbs(i,0);
    point.g = rgbs(i, 1);
    point.b = rgbs(i, 2);
    cloud->points.push_back(point);
  }
  cloud->header.frame_id = frame_id_;
  cloud->header.stamp = ros::Time::now();
}

}
}