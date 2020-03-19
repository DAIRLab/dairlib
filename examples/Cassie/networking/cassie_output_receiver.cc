#include <algorithm>
#include "examples/Cassie/networking/cassie_output_receiver.h"
#include "examples/Cassie/networking/udp_lcm_translator.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "dairlib/lcmt_cassie_out.hpp"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::LeafSystem;

CassieOutputReceiver::CassieOutputReceiver() {
  this->DeclareAbstractInputPort("lcmt_cassie_out",
    drake::Value<dairlib::lcmt_cassie_out>{});
  this->DeclareAbstractOutputPort(
      "cassie_out_t", &CassieOutputReceiver::CopyOutput);
}


void CassieOutputReceiver::CopyOutput(const Context<double>& context,
                                     cassie_out_t* cassie_out) const {
  const auto& message = this->EvalAbstractInput(
      context, 0)->get_value<dairlib::lcmt_cassie_out>();
  cassieOutFromLcm(message, cassie_out);
}

}  // namespace systems
}  // namespace dairlib
