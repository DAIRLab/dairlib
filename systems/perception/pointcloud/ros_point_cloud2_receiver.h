#pragma once

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// drake deps
#include "drake/systems/framework/leaf_system.h"


namespace dairlib {
namespace perception {

template<typename PointT>
class RosPointCloud2Receiver : public drake::systems::LeafSystem<double> {

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosPointCloud2Receiver);
  RosPointCloud2Receiver();

 private:
  void CopyInputToPcl(const drake::systems::Context<double>& context,
                      typename pcl::PointCloud<PointT>::Ptr * ptr) const;

};

}
}
