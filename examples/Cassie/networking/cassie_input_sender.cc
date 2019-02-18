#include <algorithm>
#include "examples/Cassie/networking/cassie_input_sender.h"


namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::LeafSystem;

CassieInputSender::CassieInputSender() {
  this->DeclareAbstractInputPort("cassie_user_in_t",
      drake::Value<cassie_user_in_t>{});
  this->DeclareAbstractOutputPort("lcmt_cassie_in",
      &CassieInputSender::Output);
}

template <typename T>
void copy_vector(const T* input, T* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void CassieInputSender::Output(const Context<double>& context,
                                     lcmt_cassie_in* output) const {
  const cassie_user_in_t& cassie_in =
    EvalAbstractInput(context, 0)->GetValue<cassie_user_in_t>();
  // using the time from the context
  output->utime = context.get_time() * 1e6;

  copy_vector(cassie_in.torque, output->torque, 10);
  copy_vector(cassie_in.telemetry, output->telemetry, 9);
}

}  // namespace systems
}  // namespace dairlib
