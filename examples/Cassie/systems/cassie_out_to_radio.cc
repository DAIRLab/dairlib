#include "cassie_out_to_radio.h"

namespace dairlib {
namespace systems {

using drake::systems::kUseDefaultName;

CassieOutToRadio::CassieOutToRadio() {
  cassie_out_input_port_ =
      this->DeclareAbstractInputPort("lcmt_cassie_out",
                                     drake::Value<dairlib::lcmt_cassie_out>{})
          .get_index();
  radio_output_port_ =
      this->DeclareAbstractOutputPort("lcmt_radio_out",
                                      &CassieOutToRadio::CalcRadioOut)
          .get_index();
}

void CassieOutToRadio::CalcRadioOut(
    const drake::systems::Context<double> &context,
    dairlib::lcmt_radio_out *output) const {
  const dairlib::lcmt_cassie_out *cassie_output = this->
      EvalInputValue<dairlib::lcmt_cassie_out>(context, cassie_out_input_port_);
  *output = cassie_output->pelvis.radio;
}

}
}