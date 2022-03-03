#pragma once
#include "timestamped_low_pass_filter.h"
#include "drake/multibody/plant/multibody_plant.h"

/// Specialization of Timestamped Low Pass Filter for floting base velocity
/// estimates
namespace dairlib::systems {

class FloatingBaseVelocityFilter : public TimestampedLowPassFilter {
 public:
  FloatingBaseVelocityFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<double>& tau) :
    TimestampedLowPassFilter(
        tau, plant.num_positions() + plant.num_velocities() +
                 plant.num_actuators() + 3,
    std::vector<int>{plant.num_positions() + 3,
                              plant.num_positions() + 4,
                              plant.num_positions() + 5}){};
};
}
