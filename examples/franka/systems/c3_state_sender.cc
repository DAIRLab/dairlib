#include "examples/franka/systems/c3_state_sender.h"
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
  target_state_ = this->DeclareVectorInputPort("target_state",
                                               BasicVector<double>(state_size))
                      .get_index();
  actual_state_ = this->DeclareVectorInputPort("c3_solution",
                                               TimestampedVector<double>(state_size))
                      .get_index();

  lcmt_c3_state default_c3_state = dairlib::lcmt_c3_state();
  default_c3_state.num_states = n_x_;
  default_c3_state.utime = 0;
  default_c3_state.state = std::vector<float>(n_x_);
  default_c3_state.state_names = state_names;
  target_c3_state_ = this->DeclareAbstractOutputPort(
                             "c3_actor_trajectory_output", default_c3_state,
                             &C3StateSender::OutputTargetState)
                         .get_index();
  actual_c3_state_ = this->DeclareAbstractOutputPort(
                             "c3_object_trajectory_output", default_c3_state,
                             &C3StateSender::OutputActualState)
                         .get_index();
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
