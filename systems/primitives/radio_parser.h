#pragma once

#include <dairlib/lcmt_radio_out.hpp>

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class RadioParser : public drake::systems::LeafSystem<double> {
 public:
  RadioParser();

  const drake::systems::InputPort<double>& get_input_port() const {
    return drake::systems::LeafSystem<double>::get_input_port(0);
  }

  const drake::systems::OutputPort<double>& get_output_port() const {
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

 protected:
  void CalcRadioOutput(const drake::systems::Context<double>& context,
                       dairlib::lcmt_radio_out* output) const;

 private:
  bool is_abstract() const { return false; }
  int data_input_port_;
};

}  // namespace systems
}  // namespace dairlib
