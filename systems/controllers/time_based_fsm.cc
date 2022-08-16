#include "systems/controllers/time_based_fsm.h"

#include "dairlib/lcmt_target_standing_height.hpp"

using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
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
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorOutputPort(
                      "fsm", BasicVector<double>(1),
                      &TimeBasedFiniteStateMachine::CalcFiniteState)
                  .get_index();

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
  VectorXd current_finite_state = VectorXd::Zero(1);
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
    bool with_trigger_input_port, double one_stride_period)
    : states_(states), one_stride_period_(one_stride_period) {
  DRAKE_DEMAND(states.size() == state_durations.size());

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorOutputPort(
                      "fsm", BasicVector<double>(1),
                      &TimeBasedFiniteStateMachineWithTrigger::CalcFiniteState)
                  .get_index();

  global_fsm_idx_port_ =
      this->DeclareVectorOutputPort(
              "global_fsm_idx", BasicVector<double>(1),
              &TimeBasedFiniteStateMachineWithTrigger::CalcGlobalFsmIdx)
          .get_index();

  if (with_trigger_input_port) {
    trigger_port_ =
        this->DeclareAbstractInputPort(
                "trigger_port",
                drake::Value<dairlib::lcmt_target_standing_height>{})
            .get_index();
    // Perstep update
    DeclarePerStepDiscreteUpdateEvent(
        &TimeBasedFiniteStateMachineWithTrigger::DiscreteVariableUpdate);
  }

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
    trigged_ = true;
    t0_ = 0;
  }
}

EventStatus TimeBasedFiniteStateMachineWithTrigger::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  // Read in lcm message time
  double current_time =
      dynamic_cast<const OutputVector<double>*>(this->EvalVectorInput(context,
          state_port_))->get_timestamp();

  if (!trigged_) {
    if (this->EvalInputValue<dairlib::lcmt_target_standing_height>(
        context, trigger_port_)->target_height < 0.5) {
      // Keep updating t0_ when the signal is still low.
      t0_ = current_time;
    } else {
      trigged_ = true;
    }
  }

  return EventStatus::Succeeded();
}


void TimeBasedFiniteStateMachineWithTrigger::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  double current_time =
      dynamic_cast<const OutputVector<double>*>(this->EvalVectorInput(context,
          state_port_))->get_timestamp();

  // Get current finite state
  VectorXd current_finite_state(1);
  if (!trigged_) {
    current_finite_state << -1;
  } else {
    double remainder = fmod(current_time - t0_, period_);

    for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
      if (remainder < accu_state_durations_[i]) {
        current_finite_state << states_[i];
        break;
      }
    }
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}


void TimeBasedFiniteStateMachineWithTrigger::CalcGlobalFsmIdx(
    const Context<double>& context, BasicVector<double>* global_fsm_idx) const {
  double current_time =
      dynamic_cast<const OutputVector<double>*>(this->EvalVectorInput(context,
          state_port_))->get_timestamp();

  global_fsm_idx->get_mutable_value() << int((current_time - t0_ + 1e-8) /
      one_stride_period_);
}


}  // namespace systems
}  // namespace dairlib
