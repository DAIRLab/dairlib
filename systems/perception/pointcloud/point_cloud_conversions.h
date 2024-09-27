#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "drake/perception/point_cloud.h"

namespace dairlib::perception {

template <typename PointT>
void AssignFields(const drake::perception::PointCloud& to_pc,
                  typename pcl::PointCloud<PointT>::Ptr& from_pc);

template <typename PointT>
void AssignFields(const typename pcl::PointCloud<PointT>::Ptr& from_pc,
                  drake::perception::PointCloud& to_pc);
}

