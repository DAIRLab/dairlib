#pragma once

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_foothold_set.h"
#include "drake/systems/framework/leaf_system.h"


namespace dairlib::geometry {
class ConvexFootholdReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexFootholdReceiver)
  ConvexFootholdReceiver();
 private:
  void CopyTerrain(const drake::systems::Context<double>& context,
                   ConvexFootholdSet* footholds) const;
};

class ConvexFootholdSender : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexFootholdSender)
  ConvexFootholdSender();
 private:
  void CopyTerrain(const drake::systems::Context<double>& context,
              lcmt_foothold_set* footholds) const;
};
}
