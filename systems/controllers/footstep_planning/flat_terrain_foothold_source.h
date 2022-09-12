#pragma once

#include "alip_utils.h"
#include "geometry/convex_foothold.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems {

/*
 * To account for floating base drift when providing a dummy foothold, provide
 * a foothold where the height of the foothold is always the mininum of the two
 * contact points
 */
class FlatTerrainFootholdSource : public drake::systems::LeafSystem<double> {
 public:
  FlatTerrainFootholdSource(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::vector<controllers::alip_utils::PointOnFramed> left_right_foot);

 private:
  void CalcFoothold(const drake::systems::Context<double>& context,
                      std::vector<geometry::ConvexFoothold>* footholds) const;

  std::vector<controllers::alip_utils::PointOnFramed> left_right_foot_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable drake::systems::Context<double>* context_;

};
}