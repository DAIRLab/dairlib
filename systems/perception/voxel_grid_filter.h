#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

/// Down-samples a point cloud to a set resolution
class VoxelGridFilter final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VoxelGridFilter);
  explicit VoxelGridFilter(
      double voxel_size, drake::Parallelism parallelism = false
  );
  ~VoxelGridFilter() final;

 private:
  const double voxel_size_;
  const drake::Parallelism parallelism_;
};

}  // namespace perception
}  // namespace dairlib
