#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "drake/lcmt_point_cloud.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

template<typename PointT>
class LcmToPclPointCloud : public drake::systems::LeafSystem<double> {

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmToPclPointCloud);
  LcmToPclPointCloud();

 private:
  void Calc(const drake::systems::Context<double> &context,
            typename pcl::PointCloud<PointT>::Ptr *ptr) const;

  static void AssignFields(const drake::lcmt_point_cloud& lcm_cloud,
                    typename pcl::PointCloud<PointT>::Ptr& ptr);

};

}
}

