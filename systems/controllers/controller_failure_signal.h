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

  int AddFailureSignalPort() {
    input_ports_.push_back(input_ports_.size());
    return input_ports_.back();
  }

 private:
  void AggregateFailureSignals(const drake::systems::Context<double>& context,
                               dairlib::lcmt_controller_failure* output) const;

  std::vector<int> input_ports_;
  int output_port_;
};

}  // namespace systems
}  // namespace dairlib
