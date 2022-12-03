#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "pcl_ros/point_cloud.h"

namespace dairlib {
namespace camera {

class DrakeToRosPointCloud : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeToRosPointCloud);
  explicit DrakeToRosPointCloud(std::string  frame_id);

 private:
  void CopyInputToPCL(const drake::systems::Context<double>& context,
                      pcl::PointCloud<pcl::PointXYZRGB>* cloud) const;
  const std::string frame_id_;
};

}
}

