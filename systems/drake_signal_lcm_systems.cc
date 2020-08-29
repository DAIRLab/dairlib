#include "systems/drake_signal_lcm_systems.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using std::string;

/*--------------------------------------------------------------------------*/
// methods implementation for DrakeSignalReceiver.

DrakeSignalReceiver::DrakeSignalReceiver(int signal_size)
    : signal_size_(signal_size) {
  this->DeclareAbstractInputPort("lcmt_drake_signal",
                                 drake::Value<drake::lcmt_drake_signal>{});
  this->DeclareVectorOutputPort(BasicVector<double>(signal_size),
                                &DrakeSignalReceiver::UnpackLcmIntoVector);
}

void DrakeSignalReceiver::UnpackLcmIntoVector(
    const Context<double>& context, BasicVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<drake::lcmt_drake_signal>();
  for (int i = 0; i < signal_size_; i++) {
    output->get_mutable_value()(i) = input_msg.val[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for DrakeSignalSender.

DrakeSignalSender::DrakeSignalSender(
    const std::vector<std::string>& signal_names)
    : signal_names_(signal_names), signal_size_(signal_names.size()) {
  this->DeclareVectorInputPort(BasicVector<double>(signal_names.size()));
  this->DeclareAbstractOutputPort(&DrakeSignalSender::PackVectorIntoLcm);
}

void DrakeSignalSender::PackVectorIntoLcm(const Context<double>& context,
                                          drake::lcmt_drake_signal* msg) const {
  const auto* input_vector = this->EvalVectorInput(context, 0);

  msg->dim = signal_size_;
  msg->val.resize(signal_size_);
  msg->coord.resize(signal_size_);
  for (int i = 0; i < signal_size_; i++) {
    msg->val[i] = input_vector->get_value()(i);
    msg->coord[i] = signal_names_[i];
  }
  msg->timestamp = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
