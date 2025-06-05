#pragma once

#include "alip_utils.h"
#include "geometry/convex_polygon_set.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems {

/*
 * To account for floating base drift when providing a dummy foothold, provide
 * a foothold where the height of the foothold is always the minimum of the two
 * contact points
 */
class FlatTerrainFootholdSource : public drake::systems::LeafSystem<double> {
 public:
  FlatTerrainFootholdSource(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::vector<controllers::alip_utils::PointOnFramed> left_right_foot);

 private:
  static constexpr double alpha_ = 0.1;
  void CalcFoothold(const drake::systems::Context<double>& context,
                      geometry::ConvexPolygonSet* footholds) const;
  mutable double h_prev_ = 0;
  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable drake::systems::Context<double>* context_;
  std::vector<controllers::alip_utils::PointOnFramed> left_right_foot_;

};
}