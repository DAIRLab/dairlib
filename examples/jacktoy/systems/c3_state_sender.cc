#include "examples/jacktoy/systems/c3_state_sender.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {

using drake::systems::BasicVector;
using systems::TimestampedVector;
using drake::systems::Context;

namespace systems {

C3StateSender::C3StateSender(int state_size,
                             std::vector<std::string> state_names) {
  this->set_name("c3_state_sender");

  n_x_ = state_size;
  final_target_state_ = this->DeclareVectorInputPort("final_target_state",
                                               BasicVector<double>(state_size))
                      .get_index();
  target_state_ = this->DeclareVectorInputPort("target_state",
                                               BasicVector<double>(state_size))
                      .get_index();
  actual_state_ = this->DeclareVectorInputPort("actual_state",
                                               TimestampedVector<double>(state_size))
                      .get_index();

  lcmt_c3_state default_c3_state = dairlib::lcmt_c3_state();
  default_c3_state.num_states = n_x_;
  default_c3_state.utime = 0;
  default_c3_state.state = std::vector<float>(n_x_);
  default_c3_state.state_names = state_names;
  final_target_c3_state_ = this->DeclareAbstractOutputPort(
                             "c3_final_target_output", default_c3_state,
                             &C3StateSender::OutputFinalTargetState)
                         .get_index();
  target_c3_state_ = this->DeclareAbstractOutputPort(
                             "c3_target_output", default_c3_state,
                             &C3StateSender::OutputTargetState)
                         .get_index();
  actual_c3_state_ = this->DeclareAbstractOutputPort(
                             "c3_actual_output", default_c3_state,
                             &C3StateSender::OutputActualState)
                         .get_index();
}

void C3StateSender::OutputFinalTargetState(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_state* output) const {
  const auto final_target_state = this->EvalVectorInput(context, final_target_state_);
  DRAKE_DEMAND(final_target_state->size() == n_x_);
  output->utime = context.get_time() * 1e6;
  for (int i = 0; i < n_x_; ++i) {
    output->state[i] = static_cast<float>(final_target_state->GetAtIndex(i));
  }
}

void C3StateSender::OutputTargetState(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_state* output) const {
  const auto target_state = this->EvalVectorInput(context, target_state_);
  DRAKE_DEMAND(target_state->size() == n_x_);
  output->utime = context.get_time() * 1e6;
  for (int i = 0; i < n_x_; ++i) {
    output->state[i] = static_cast<float>(target_state->GetAtIndex(i));
  }
}

void C3StateSender::OutputActualState(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_state* output) const {
  const auto actual_state = (TimestampedVector<double>*)this->EvalVectorInput(context, actual_state_);
  DRAKE_DEMAND(actual_state->get_data().size() == n_x_);
  output->utime = context.get_time() * 1e6;
  for (int i = 0; i < n_x_; ++i) {
    output->state[i] = static_cast<float>(actual_state->GetAtIndex(i));
  }
}
}  // namespace systems
}  // namespace dairlib
