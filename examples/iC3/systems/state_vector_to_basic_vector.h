#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/state_vector.h"

namespace dairlib {

class StateVectorToBasicVector : public drake::systems::LeafSystem<double> {
 public:
  explicit StateVectorToBasicVector(const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return this->get_output_port(state_output_port_);
  }

 private:
  void CopyState(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex state_input_port_;
  drake::systems::OutputPortIndex state_output_port_;
};

}  // namespace dairlib
