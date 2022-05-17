#pragma once
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class CassieOutToRadio : public drake::systems::LeafSystem<double> {
 public:
  CassieOutToRadio();

  const drake::systems::InputPort<double> &get_input_port() const {
    return drake::systems::LeafSystem<double>::get_input_port(0);
  }

  const drake::systems::OutputPort<double> &get_output_port() const {
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

 protected:
  void CalcRadioOut(
      const drake::systems::Context<double> &context,
      dairlib::lcmt_radio_out *output) const;

 private:
  bool is_abstract() const { return false; }

  int cassie_out_input_port_;
  int radio_output_port_;
};
}
}