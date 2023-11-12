//
// Created by brian on 11/2/23.
//

#include "systems/perception/pointcloud/voxel_grid_filter.h"

namespace dairlib {
namespace perception {

using drake::perception::PointCloud;
using drake::systems::Context;
using drake::AbstractValue;

VoxelGridFilter::VoxelGridFilter(
    double voxel_size, drake::Parallelism parallelism) :
    voxel_size_(voxel_size), parallelism_(parallelism) {
  DeclareAbstractInputPort("point cloud", drake::Value<PointCloud>());
  DeclareAbstractOutputPort(
      "filtered point cloud",
      []() {return AbstractValue::Make<PointCloud>();}, // allocator
      [this](const Context<double>& context, AbstractValue* value) {
        auto& pc_in = this->get_input_port().template Eval<PointCloud>(context);
        auto& pc_out = value->get_mutable_value<PointCloud>();
        pc_out = pc_in.VoxelizedDownSample(voxel_size_, parallelism_);
      });
}

VoxelGridFilter::~VoxelGridFilter() = default;

}
}