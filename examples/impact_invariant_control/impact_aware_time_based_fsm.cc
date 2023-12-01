#include "examples/impact_invariant_control/impact_aware_time_based_fsm.h"

#include <dairlib/lcmt_radio_out.hpp>
#include <iostream>

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using std::string;

namespace dairlib {

using systems::ImpactInfoVector;
using systems::OutputVector;

ImpactTimeBasedFiniteStateMachine::ImpactTimeBasedFiniteStateMachine(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<int>& states, const std::vector<double>& state_durations,
    double t0, double near_impact_threshold, double tau, BLEND_FUNC blend_func)
    : TimeBasedFiniteStateMachine(plant, states, state_durations, t0),
      states_(states),
      state_durations_(state_durations),
      t0_(t0),
      near_impact_threshold_(near_impact_threshold),
      tau_(tau),
      blend_func_(blend_func) {
  this->set_name("ImpactAwareTimeBasedFSM");
  near_impact_port_ =
      this->DeclareVectorOutputPort(
              "near_impact", ImpactInfoVector<double>(0, 0, 0),
              &ImpactTimeBasedFiniteStateMachine::CalcNearImpact)
          .get_index();
  clock_port_ = this->DeclareVectorOutputPort(
          "clock", BasicVector<double>(1),
          &ImpactTimeBasedFiniteStateMachine::CalcClock)
      .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();

  contact_scheduler_port_ =
      this->DeclareVectorOutputPort(
              "contact_scheduler "
              "(pelvis_t0, "
              "pelvis_tf, "
              "left_t0, left_tf, "
              "right_t0, right_t0",
              BasicVector<double>(6),
              &ImpactTimeBasedFiniteStateMachine::CalcContactScheduler)
          .get_index();

  // Accumulate the durations to get timestamps
  double sum = 0;
  DRAKE_DEMAND(states.size() == state_durations.size());
  impact_times_.push_back(0);
  impact_states_.push_back(states[0]);
  accu_state_durations_.push_back(0);
  for (int i = 0; i < states.size(); ++i) {
    sum += state_durations[i];
    accu_state_durations_.push_back(sum);
    if (states[i] >= 2) {
      impact_times_.push_back(sum);
      impact_states_.push_back(states[i + 1]);
    }
  }

  period_ = sum;
}

void ImpactTimeBasedFiniteStateMachine::CalcNearImpact(
    const Context<double>& context,
    ImpactInfoVector<double>* near_impact) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_time, period_);

  // Assign the blending function ptr
  auto alpha_func = blend_func_ == kSigmoid ? &blend_sigmoid : &blend_exp;

  near_impact->set_timestamp(current_time);
  near_impact->SetCurrentContactMode(0);
  near_impact->SetAlpha(0);
  // Get current finite state
  if (current_time >= t0_) {
    for (int i = 0; i < impact_states_.size(); ++i) {
      double blend_window = blend_func_ == kSigmoid
                            ? 1.5 * near_impact_threshold_
                            : near_impact_threshold_;
      if (abs(remainder - impact_times_[i]) < blend_window) {
        if (remainder < impact_times_[i]) {
          near_impact->SetAlpha(alpha_func(remainder - impact_times_[i], tau_,
                                           near_impact_threshold_));
        } else {
          near_impact->SetAlpha(alpha_func(impact_times_[i] - remainder, tau_,
                                           near_impact_threshold_));
        }
        near_impact->SetCurrentContactMode(impact_states_[i]);
        break;
      }
    }
  }
}

void ImpactTimeBasedFiniteStateMachine::CalcClock(
    const Context<double>& context, BasicVector<double>* clock) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_time, period_);
  //  clock->get_mutable_value()(0) = remainder;
  clock->get_mutable_value()(0) = current_time;
}

void ImpactTimeBasedFiniteStateMachine::CalcContactScheduler(
    const Context<double>& context, BasicVector<double>* contact_timing) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  if (this->get_input_port(radio_port_).HasValue(context)) {
    const auto& radio_out =
        this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  }
  auto current_time = static_cast<double>(robot_output->get_timestamp());
  double remainder = fmod(current_time, period_);
  int n_periods = (int)(current_time / period_);

  contact_timing->get_mutable_value()(0) =
      n_periods * period_ + accu_state_durations_[0];
  contact_timing->get_mutable_value()(1) =
      n_periods * period_ + accu_state_durations_[1];
  contact_timing->get_mutable_value()(2) =
      n_periods * period_ + accu_state_durations_[1];
  contact_timing->get_mutable_value()(3) =
      n_periods * period_ + accu_state_durations_[4];
  contact_timing->get_mutable_value()(4) =
      (n_periods - 1) * period_ + accu_state_durations_[3];
  contact_timing->get_mutable_value()(5) =
      (n_periods - 1) * period_ + accu_state_durations_[3] +
          (accu_state_durations_[2] - accu_state_durations_[0]) +
          (accu_state_durations_[4] - accu_state_durations_[3]);
}

}  // namespace dairlib
