#include "systems/controllers/time_based_fsm.h"

using std::cout;
using std::endl;

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using std::string;

namespace dairlib {
namespace systems {

TimeBasedFiniteStateMachine::TimeBasedFiniteStateMachine(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<int>& states, const std::vector<double>& state_durations,
    double t0)
    : states_(states), t0_(t0) {
  DRAKE_DEMAND(states.size() == state_durations.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &TimeBasedFiniteStateMachine::CalcFiniteState);

  // Accumulate the durations to get timestamps
  double sum = 0;
  for (auto& duration_i : state_durations) {
    sum += duration_i;
    accu_state_durations_.push_back(sum - eps_);
  }
  period_ = sum;

  // Add one more state to loop back
  accu_state_durations_.push_back(period_ + state_durations[0]);
  states_.push_back(states[0]);
}

void TimeBasedFiniteStateMachine::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_sim_time, period_);

  // Get current finite state
  VectorXd current_finite_state(1);
  if (current_sim_time >= t0_) {
    for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
      if (remainder < accu_state_durations_[i]) {
        current_finite_state << states_[i];
        break;
      }
    }
  } else {
    current_finite_state << initial_state_;
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}

TimeBasedFiniteStateMachineWithTrigger::TimeBasedFiniteStateMachineWithTrigger(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<int>& states, const std::vector<double>& state_durations,
    bool with_trigger_input_port)
    : states_(states), with_trigger_input_port_(with_trigger_input_port) {
  DRAKE_DEMAND(states.size() == state_durations.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  trigger_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  this->DeclareVectorOutputPort(
      BasicVector<double>(1),
      &TimeBasedFiniteStateMachineWithTrigger::CalcFiniteState);

  // Accumulate the durations to get timestamps
  double sum = 0;
  for (auto& duration_i : state_durations) {
    sum += duration_i;
    accu_state_durations_.push_back(sum - eps_);
  }
  period_ = sum;

  // Add one more state to loop back
  accu_state_durations_.push_back(period_ + state_durations[0]);
  states_.push_back(states[0]);

  // If not using the trigger port, then set t0 = 0.
  if (!with_trigger_input_port) {
    t0_ = 0;
  }
}

void TimeBasedFiniteStateMachineWithTrigger::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());

  if ((t0_ < 0) && with_trigger_input_port_) {
    if (this->EvalVectorInput(context, trigger_port_)->get_value()(0) > 0.5) {
      // Triggerred
      t0_ = current_sim_time;
    }
  }

  double offset = (t0_ < 0) ? current_sim_time : t0_;
  double remainder = fmod(current_sim_time - offset, period_);

  // Get current finite state
  VectorXd current_finite_state(1);
  for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
    if (remainder < accu_state_durations_[i]) {
      current_finite_state << states_[i];
      break;
    }
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}

}  // namespace systems
}  // namespace dairlib
