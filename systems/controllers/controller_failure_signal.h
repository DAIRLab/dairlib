#pragma once

#include <limits>

#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// ControllerFailureSignal
class ControllerFailureSignal : public drake::systems::LeafSystem<double> {
 public:
  ControllerFailureSignal(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::vector<int> fsm_states_of_interest = {},
      int prev_fsm_state = -std::numeric_limits<int>::infinity(),
      bool set_current_time_until_first_state_switch = false);


 private:


};

}  // namespace systems
}  // namespace dairlib
