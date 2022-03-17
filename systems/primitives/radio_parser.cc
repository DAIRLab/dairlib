#include "systems/primitives/radio_parser.h"

#include <drake/common/eigen_types.h>

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

}  // namespace systems
}  // namespace dairlib
