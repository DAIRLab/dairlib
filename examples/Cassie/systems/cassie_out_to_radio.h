#pragma once
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

 /// Convenience system to separate out the radio lcm struct from the larger
 /// lcmt_cassie_out struct. THe output port of this system will pass through
 /// the pelvis.radio field of the lcmt_cassie_out message at the system's
 /// input port
class CassieOutToRadio : public drake::systems::LeafSystem<double> {
 public:
  CassieOutToRadio();

 protected:
  void CalcRadioOut(
      const drake::systems::Context<double> &context,
      dairlib::lcmt_radio_out *output) const;

 private:
  int cassie_out_input_port_;
  int radio_output_port_;
};
}
}