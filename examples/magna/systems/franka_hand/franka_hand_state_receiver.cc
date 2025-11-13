#include "examples/magna/systems/franka_hand/franka_hand_state_receiver.h"

#include "drake/lcmt_schunk_wsg_status.hpp"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace franka_hand {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::InputPortIndex;
using drake::systems::LeafSystem;
using drake::systems::OutputPortIndex;

FrankaHandStateReceiver::FrankaHandStateReceiver() {
  this->DeclareAbstractInputPort("lcmt_schunk_wsg_status",
                                 drake::Value<drake::lcmt_schunk_wsg_status>{});
  this->DeclareVectorOutputPort("franka_hand_state", BasicVector<double>(2),
                                &FrankaHandStateReceiver::OutputFrankaHandState,
                                {all_sources_ticket()});
}

void FrankaHandStateReceiver::OutputFrankaHandState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<drake::lcmt_schunk_wsg_status>();
  output->SetFromVector(Eigen::VectorXd::Ones(2) *
                        input_msg.actual_position_mm / 1000.0);
}

}  // namespace franka_hand
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib