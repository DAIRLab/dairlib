#include "drake_to_pcl_pointcloud.h"
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
  AssignFields(drake_cloud, pcl_cloud);
}

template <typename PointT>
void DrakeToPclPointCloud<PointT>::AssignFields(
    const drake::perception::PointCloud& drake_cloud,
    typename pcl::PointCloud<PointT>::Ptr& ptr) {

  int npoints = drake_cloud.size();
  ptr->points.resize(npoints);

  // get points as a raw buffer
  const auto& xyzs = drake_cloud.xyzs();
  int nfinite = 0;

  // TODO (@Brian-Acosta) copy more than XYZ
  for (Eigen::Index i = 0; i < npoints; ++i) {
    if (not xyzs.col(i).array().isFinite().all()) continue;
    ptr->points[i].getVector3fMap() = xyzs.col(i).cast<float>();
    ++nfinite;
  }

  ptr->points.resize(nfinite);

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*ptr, *ptr, indices);
}

template class DrakeToPclPointCloud<pcl::PointXYZ>;
template class DrakeToPclPointCloud<pcl::PointXYZRGB>;
template class DrakeToPclPointCloud<pcl::PointXYZRGBConfidenceRatio>;

}
}