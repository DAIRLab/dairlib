#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_sample_buffer.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts sample costs and configurations to LCM type lcmt_sample_buffer
class SampleBufferSender : public drake::systems::LeafSystem<double> {
 public:
  SampleBufferSender(int buffer_size, int n_config);

  const drake::systems::InputPort<double>& get_input_port_sample_costs() const {
    return this->get_input_port(sample_costs_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_samples() const {
    return this->get_input_port(samples_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_sample_buffer()
    const {
        return this->get_output_port(lcm_sample_buffer_output_port_);
  }

 private:
  void OutputSampleBufferLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_sample_buffer* output) const;

  int buffer_size_;
  int n_config_;

  drake::systems::InputPortIndex samples_port_;
  drake::systems::InputPortIndex sample_costs_port_;
  drake::systems::OutputPortIndex lcm_sample_buffer_output_port_;
};

}  // namespace systems
}  // namespace dairlib