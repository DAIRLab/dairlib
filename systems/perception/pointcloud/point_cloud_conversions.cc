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

template <typename PointT>
void AssignFields(const grid_map::GridMap& map,
                  const std::string& layer,
                  typename pcl::PointCloud<PointT>::Ptr& cloud) {

  cloud->points.clear();

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    // Get the position of the current grid cell in map coordinates
    grid_map::Position position;
    map.getPosition(*it, position);

    // Check if the current cell has a valid value for the specified layer
    if (!map.isValid(*it, layer)) {
      continue;  // Skip invalid cells
    }

    // Get the height value from the specified layer
    float height = map.at(layer, *it);

    // Create a 3D point (x, y, z)
    pcl::PointXYZL point;
    point.x = position.x();  // X-coordinate in the map
    point.y = position.y();  // Y-coordinate in the map
    point.z = height;        // Z-coordinate (height) from the specified layer

    // Add the point to the point cloud
    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
}

template void AssignFields<pcl::PointXYZ>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZ>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZRGB>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZRGBConfidenceRatio>(const drake::perception::PointCloud&, pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr&); // NOLINT
template void AssignFields<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, drake::perception::PointCloud&); // NOLINT
template void AssignFields<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, drake::perception::PointCloud&); // NOLINT
template void AssignFields<pcl::PointXYZRGBConfidenceRatio>(const pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>::Ptr&, drake::perception::PointCloud&); // NOLINT
template void AssignFields<pcl::PointXYZL>(const grid_map::GridMap&, const std::string&, pcl::PointCloud<pcl::PointXYZL>::Ptr&); // NOLINT
}