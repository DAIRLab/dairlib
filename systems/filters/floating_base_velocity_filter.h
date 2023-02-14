#pragma once
#include "output_vector_filter.h"

#include "drake/multibody/plant/multibody_plant.h"

/// Specialization of OutputVectorFilter for floating base linear velocity
/// estimates
namespace dairlib::systems {

class FloatingBaseVelocityFilter : public OutputVectorFilter {
 public:
  FloatingBaseVelocityFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<double>& tau)
      : OutputVectorFilter(plant, tau,
                           std::vector<int>{plant.num_positions() + 3,
                                            plant.num_positions() + 4,
                                            plant.num_positions() + 5}){};
};


class FloatingBaseVelocityButterworthFilter : public OutputVectorButterworthFilter {
 public:
  FloatingBaseVelocityButterworthFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      int order,
      double f_s,
      const std::vector<double>& f_c)
      : OutputVectorButterworthFilter(plant, order, f_s, f_c,
                           std::vector<int>{plant.num_positions() + 3,
                                            plant.num_positions() + 4,
                                            plant.num_positions() + 5}){};
};

}  // namespace dairlib::systems
