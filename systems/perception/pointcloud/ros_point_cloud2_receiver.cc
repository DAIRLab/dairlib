#include "ros_point_cloud2_receiver.h"

// ros deps
#include "sensor_msgs/PointCloud2.h"

// misc
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace dairlib {
namespace perception {

using drake::systems::Context;
using drake::Value;

using pcl::PointCloud;

template <typename PointT>
RosPointCloud2Receiver<PointT>::RosPointCloud2Receiver() {
  DeclareAbstractInputPort("point_cloud_msg", Value<sensor_msgs::PointCloud2>());
  typename PointCloud<PointT>::Ptr model =
      std::make_shared<PointCloud<PointT>>();
  DeclareAbstractOutputPort(
      "PointCloudPtr", model, &RosPointCloud2Receiver<PointT>::CopyInputToPcl
  );
}


namespace {
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr convertPointCloud2ToPCLDirectly(
    const sensor_msgs::PointCloud2& cloud_msg){
  // Create a PCL PointCloud
  typename pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);

  // Iterate through the data array in the PointCloud2 message
  for (size_t i = 0; i < cloud_msg.width * cloud_msg.height; ++i)
  {
    PointT point;
    const uint8_t* data = &cloud_msg.data[i * cloud_msg.point_step];

    // Copy data from PointCloud2 to PCL PointCloud
    memcpy(&point, data, sizeof(PointT));

    pcl_cloud->push_back(point);
  }

  return pcl_cloud;
}
}

template <typename PointT>
void RosPointCloud2Receiver<PointT>::CopyInputToPcl(
    const Context<double>&context,
    typename PointCloud<PointT>::Ptr* ptr) const {
  auto cloud_msg = EvalAbstractInput(
      context, 0)->template get_value<sensor_msgs::PointCloud2>();
  auto& pcl_cloud = *ptr;
  pcl_cloud = convertPointCloud2ToPCLDirectly<PointT>(cloud_msg);
}


template class RosPointCloud2Receiver<pcl::PointXYZ>;
template class RosPointCloud2Receiver<pcl::PointXYZRGB>;
template class RosPointCloud2Receiver<pcl::PointXYZRGBConfidenceRatio>;

}
}