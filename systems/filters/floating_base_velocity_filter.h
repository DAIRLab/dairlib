#pragma once
#include "output_vector_filter.h"

#include "drake/multibody/plant/multibody_plant.h"

/// Specialization of Timestamped Low Pass Filter for floting base velocity
/// estimates
namespace dairlib::systems {

class FloatingBaseVelocityFilter : public OutputVectorFilter {
 public:
  FloatingBaseVelocityFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<double> tau)
      : OutputVectorFilter(plant, tau,
                           std::vector<int>{plant.num_positions() + 3,
                                            plant.num_positions() + 4,
                                            plant.num_positions() + 5}){};
};
}  // namespace dairlib::systems
