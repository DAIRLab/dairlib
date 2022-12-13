#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multipose_visualizer.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::Context;
using drake::systems::DiscreteValues;

namespace dairlib {

class KinematicCentroidalVisualizer
    : public drake::systems::LeafSystem<double> {
 public:
  KinematicCentroidalVisualizer(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      std::unique_ptr<multibody::MultiposeVisualizer> visualizer);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }

 private:
  drake::systems::EventStatus DrawTrajectory(
      const Context<double>& context,
      DiscreteValues<double>* discrete_state) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  std::unique_ptr<multibody::MultiposeVisualizer> visualizer_;
  drake::systems::InputPortIndex trajectory_port_;
};

}  // namespace dairlib
