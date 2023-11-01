#include "lcm_to_pcl_pointcloud.h"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace dairlib {
namespace perception {

using drake::systems::Context;
using drake::lcmt_point_cloud;

template <typename PointT>
LcmToPclPointCloud<PointT>::LcmToPclPointCloud() {
  DeclareAbstractInputPort("lcmt_point_cloud",
                           drake::Value<drake::lcmt_point_cloud>());
  DeclareAbstractOutputPort("PointCloudPtr", &LcmToPclPointCloud<PointT>::Calc);
}

template <typename PointT>
void LcmToPclPointCloud<PointT>::Calc(
    const Context<double>&context, typename pcl::PointCloud<PointT>::Ptr* ptr) const {
  auto lcm_cloud = EvalAbstractInput(
      context, 0)->template get_value<lcmt_point_cloud>();
  auto& pcl_cloud = *ptr;
  pcl_cloud->header.frame_id = lcm_cloud.frame_name;
  pcl_cloud->header.stamp = lcm_cloud.utime;
  AssignFields(lcm_cloud, pcl_cloud);
}

template <typename PointT>
void LcmToPclPointCloud<PointT>::AssignFields(
    const lcmt_point_cloud& lcm_cloud,
    typename pcl::PointCloud<PointT>::Ptr& ptr) {

 int npoints = lcm_cloud.width * lcm_cloud.height;
 ptr->points.resize(npoints);

 // get points as a raw buffer
 auto points_buf = Eigen::Map<const Eigen::MatrixXf>(
     reinterpret_cast<const float*>(lcm_cloud.data.data()),
     lcm_cloud.num_fields, npoints
 );

 // TODO (@Brian-Acosta) copy more than XYZ
 for (std::size_t i = 0; i < npoints; i++) {
   ptr->points[i].getVector3fMap() = points_buf.col(i).head<3>();
 }

}

template class LcmToPclPointCloud<pcl::PointXYZ>;
template class LcmToPclPointCloud<pcl::PointXYZRGB>;
template class LcmToPclPointCloud<pcl::PointXYZRGBConfidenceRatio>;

}
}