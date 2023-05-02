#pragma once

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_foothold_set.h"
#include "drake/systems/framework/leaf_system.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"

namespace dairlib::geometry {

class ConvexFootholdRosReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexFootholdRosReceiver);
  ConvexFootholdRosReceiver(double convexity_threshold);
 private:
  double convexity_threshold_;
  void CopyTerrain(const drake::systems::Context<double> &context,
                   ConvexFootholdSet *footholds) const;

};
}