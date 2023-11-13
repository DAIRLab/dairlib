#pragma once

#include "dairlib/lcmt_foothold_set.hpp"
#include "geometry/convex_polygon_set.h"
#include "drake/systems/framework/leaf_system.h"


namespace dairlib::geometry {
class ConvexPolygonReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexPolygonReceiver)
  ConvexPolygonReceiver();
 private:
  void CopyTerrain(const drake::systems::Context<double>& context,
                   ConvexPolygonSet* footholds) const;
};

class ConvexPolygonSender : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexPolygonSender)
  ConvexPolygonSender();
 private:
  void CopyTerrain(const drake::systems::Context<double>& context,
              lcmt_foothold_set* footholds) const;
};
}
