#pragma once

#include <string>
#include <vector>

#include "common/find_resource.h"
#include "dairlib/lcmt_c3_state.hpp"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class C3StateSender : public drake::systems::LeafSystem<double> {
 public:
  explicit C3StateSender(int state_size, std::vector<std::string> state_names);

  const drake::systems::InputPort<double>& get_input_port_target_state() const {
    return this->get_input_port(target_state_);
  }

  const drake::systems::InputPort<double>& get_input_port_actual_state() const {
    return this->get_input_port(actual_state_);
  }

  const drake::systems::OutputPort<double>& get_output_port_target_c3_state()
      const {
    return this->get_output_port(target_c3_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_actual_c3_state()
      const {
    return this->get_output_port(actual_c3_state_);
  }

 private:
  void OutputTargetState(const drake::systems::Context<double>& context,
                         dairlib::lcmt_c3_state* output) const;

  void OutputActualState(const drake::systems::Context<double>& context,
                         dairlib::lcmt_c3_state* output) const;

  drake::systems::InputPortIndex target_state_;
  drake::systems::InputPortIndex actual_state_;
  drake::systems::OutputPortIndex target_c3_state_;
  drake::systems::OutputPortIndex actual_c3_state_;

  int n_x_;
};

}  // namespace systems
}  // namespace dairlib
