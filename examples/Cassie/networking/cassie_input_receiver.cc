#include <algorithm>
#include "examples/Cassie/networking/cassie_input_receiver.h"


namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::LeafSystem;

template <typename T> void copy_vector(const T* input, T* output, int size);

CassieInputReceiver::CassieInputReceiver() {
  this->DeclareAbstractInputPort("lcmt_cassie_in",
    drake::Value<dairlib::lcmt_cassie_in>{});
  this->DeclareAbstractOutputPort(
      "cassie_in_t", &CassieInputReceiver::CopyOutput);
}

template <typename T>
void copy_vector(const T* input, T* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void CassieInputReceiver::CopyOutput(const Context<double>& context,
                                     cassie_user_in_t* cassie_in) const {
  const auto& message =
      this->EvalAbstractInput(context, 0)->get_value<dairlib::lcmt_cassie_in>();

  copy_vector(message.telemetry, cassie_in->telemetry, 9);
  copy_vector(message.torque, cassie_in->torque, 9);
}

}  // namespace systems
}  // namespace dairlib
