#pragma once

#include <limits>

#include <dairlib/lcmt_controller_failure.hpp>

#include "systems/framework/output_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// ControllerFailureSignal compiles all the failure possiblities and outputs a
/// lcmt_controller_failure whenever a single failure occurs
class ControllerFailureAggregator : public drake::systems::LeafSystem<double> {
 public:
  ControllerFailureAggregator(std::string controller_channel_name,
                              int num_input_ports);

  const drake::systems::OutputPort<double>& get_status_output_port() const {
    return this->get_output_port(status_output_port_);
  }
 private:
  void AggregateFailureSignals(const drake::systems::Context<double>& context,
                               dairlib::lcmt_controller_failure* output) const;

  std::string controller_channel_;
  std::vector<int> input_ports_;
  int status_output_port_;
};

}  // namespace systems
}  // namespace dairlib
