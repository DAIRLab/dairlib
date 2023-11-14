#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

template<typename PointT>
class DrakeToPclPointCloud : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeToPclPointCloud);
  DrakeToPclPointCloud();

 private:
  void Calc(const drake::systems::Context<double> &context,
            typename pcl::PointCloud<PointT>::Ptr *ptr) const;

  static void AssignFields(const drake::perception::PointCloud& drake_cloud,
                           typename pcl::PointCloud<PointT>::Ptr& ptr);

};

}
}
