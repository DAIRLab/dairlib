#pragma once

#include "dairlib/lcmt_robot_output.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace franka_hand {

class FrankaHandStatusBridgeOut : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaHandStatusBridgeOut();

 private:
  void OutputFrankaHandState(const drake::systems::Context<double>& context,
                             dairlib::lcmt_robot_output* output) const;

  drake::systems::InputPortIndex franka_hand_state_input_port_;
  drake::systems::OutputPortIndex franka_hand_state_output_port_;
};

}  // namespace franka_hand
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
