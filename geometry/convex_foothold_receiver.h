#pragma once

#include "geometry/convex_foothold_set.h"
#include "drake/systems/framework/leaf_system.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"


namespace dairlib::geometry {
class ConvexFootholdReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexFootholdReceiver)
  ConvexFootholdReceiver();

 private:
  void CopyTerrain(const drake::systems::Context<double>& context,
                   ConvexFootholdSet* footholds) const;

};
}
