#include "systems/primitives/radio_parser.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
RadioParser::RadioParser() {
  data_input_port_ =
      this->DeclareVectorInputPort("raw_radio", BasicVector<double>(2 + 16))
          .get_index();
  this->DeclareAbstractOutputPort("radio_out", dairlib::lcmt_radio_out(),
                                  &RadioParser::CalcRadioOutput);
}

void RadioParser::CalcRadioOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_radio_out* output) const {
  const BasicVector<double>& data =
      *this->template EvalVectorInput<BasicVector>(context, data_input_port_);
  output->radioReceiverSignalGood = data[0];
  output->receiverMedullaSignalGood = data[1];
  for (int i = 0; i < 16; ++i) {
    output->channel[i] = data[2 + i];
  }
}

RadioToVector::RadioToVector() {
  radio_port_ = this->DeclareAbstractInputPort(
                        "radio_in", drake::Value<dairlib::lcmt_radio_out>())
                    .get_index();
  this->DeclareVectorOutputPort("raw_radio", BasicVector<double>(2 + 16),
                                &RadioToVector::ConvertToVector);
}

void RadioToVector::ConvertToVector(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  output->SetZero();
  for (int i = 0; i < 16; ++i) {
    output->get_mutable_value()[i] = radio_out->channel[i];
  }
  output->get_mutable_value()[16 + 0] = radio_out->radioReceiverSignalGood;
  output->get_mutable_value()[16 + 1] = radio_out->receiverMedullaSignalGood;
}

}  // namespace systems
}  // namespace dairlib
