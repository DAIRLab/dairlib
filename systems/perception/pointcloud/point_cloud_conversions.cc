#include "point_cloud_conversions.h"

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "pcl/filters/filter.h"


namespace dairlib::perception {

template <typename PointT>
void AssignFields(
    const drake::perception::PointCloud& from_pc,
    typename pcl::PointCloud<PointT>::Ptr& to_pc) {

  int npoints = from_pc.size();
  to_pc->points.resize(npoints);

  // get points as a raw buffer
  const auto& xyzs = from_pc.xyzs();
  int nfinite = 0;

  // TODO (@Brian-Acosta) copy more than XYZ
  for (Eigen::Index i = 0; i < npoints; ++i) {
    if (not xyzs.col(i).array().isFinite().all()) continue;
    to_pc->points[nfinite].getVector3fMap() = xyzs.col(i).cast<float>();
    ++nfinite;
  }

  to_pc->points.resize(nfinite);

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*to_pc, *to_pc, indices);
}

template <typename PointT>
void AssignFields(
    const typename pcl::PointCloud<PointT>::Ptr& from_pc,
    drake::perception::PointCloud& to_pc) {

  int npoints = from_pc->points.size();
  to_pc.resize(npoints, true);

  // get points as a raw buffer
  auto xyzs = to_pc.mutable_xyzs();
  int nfinite = 0;

  // TODO (@Brian-Acosta) copy more than XYZ
  for (Eigen::Index i = 0; i < npoints; ++i) {
    const Eigen::Vector3f& point = from_pc->points[i].getVector3fMap();
    if (point.hasNaN() or not point.allFinite()) continue;
    xyzs.col(nfinite) = point;
    ++nfinite;
  }
  to_pc.resize(nfinite);
}

template void AssignFields<pcl::PointXYZ>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZ>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZRGB>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZRGBConfidenceRatio>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, drake::perception::PointCloud&); // NOLINT
template void AssignFields<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, drake::perception::PointCloud&); // NOLINT
template void AssignFields<pcl::PointXYZRGBConfidenceRatio>(const pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr&, drake::perception::PointCloud&); // NOLINT

}