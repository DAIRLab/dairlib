#include "drake_to_pcl_point_cloud.h"
#include "point_cloud_conversions.h"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"

namespace dairlib {
namespace perception {

using drake::systems::Context;
using drake::Value;

template <typename PointT>
DrakeToPclPointCloud<PointT>::DrakeToPclPointCloud() {
  DeclareAbstractInputPort("point_cloud", Value<drake::perception::PointCloud>());
  typename pcl::PointCloud<PointT>::Ptr model =
      std::make_shared<pcl::PointCloud<PointT>>();
  DeclareAbstractOutputPort(
      "PointCloudPtr", model, &DrakeToPclPointCloud<PointT>::Calc
  );
}

template <typename PointT>
void DrakeToPclPointCloud<PointT>::Calc(
    const Context<double>&context,
    typename pcl::PointCloud<PointT>::Ptr* ptr) const {
  auto drake_cloud = EvalAbstractInput(
      context, 0)->template get_value<drake::perception::PointCloud>();
  auto& pcl_cloud = *ptr;
  pcl_cloud->header.frame_id = "";
  pcl_cloud->header.stamp = 1e6 * context.get_time();
  AssignFields<PointT>(drake_cloud, pcl_cloud);
}

template class DrakeToPclPointCloud<pcl::PointXYZ>;
template class DrakeToPclPointCloud<pcl::PointXYZRGB>;
template class DrakeToPclPointCloud<pcl::PointXYZRGBConfidenceRatio>;

}
}