#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_c3_output.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts a OutputVector object to LCM type lcmt_robot_output
class C3OutputSender : public drake::systems::LeafSystem<double> {
 public:
  C3OutputSender();

  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3_intermediates()
      const {
    return this->get_input_port(c3_intermediates_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_c3_debug() const {
    return this->get_output_port(lcm_c3_output_port_);
  }

 private:
  void OutputC3Lcm(const drake::systems::Context<double>& context,
                   dairlib::lcmt_c3_output* output) const;

  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::InputPortIndex c3_intermediates_port_;
  drake::systems::OutputPortIndex lcm_c3_output_port_;
};

}  // namespace systems
}  // namespace dairlib