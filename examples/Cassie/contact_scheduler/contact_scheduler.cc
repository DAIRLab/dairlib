#include "examples/Cassie/contact_scheduler/contact_scheduler.h"

#include <iostream>

using std::cout;
using std::endl;

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using std::string;

namespace dairlib {

using systems::ImpactInfoVector;
using systems::OutputVector;

ContactScheduler::ContactScheduler(
    const drake::multibody::MultibodyPlant<double> &plant,
    double near_impact_threshold, double tau, BLEND_FUNC blend_func)
    : near_impact_threshold_(near_impact_threshold),
      blend_func_(blend_func) {
  impact_info_port_ = this->DeclareVectorOutputPort(
          "near_impact", ImpactInfoVector<double>(0, 0, 0),
          &ContactScheduler::CalcNextImpactInfo)
      .get_index();
  clock_port_ = this->DeclareVectorOutputPort("clock", BasicVector<double>(1),
                                              &ContactScheduler::CalcClock)
      .get_index();
  contact_scheduler_port_ = this->DeclareVectorOutputPort("contact_scheduler (pelvis_t0, pelvis_tf, left_t0, left_tf, right_t0, right_t0",
                                                          BasicVector<double>(6),
                                                          &ContactScheduler::CalcContactScheduler)
      .get_index();

}

double alpha_sigmoid(double t, double tau, double near_impact_threshold) {
  double x = (t + near_impact_threshold) / tau;
  return exp(x) / (1 + exp(x));
}

double alpha_exp(double t, double tau, double near_impact_threshold) {
  return 1 - exp(-(t + near_impact_threshold) / tau);
}

void ContactScheduler::CalcNextImpactInfo(
    const Context<double> &context,
    ImpactInfoVector<double> *near_impact) const {
  // Read in lcm message time
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_time, period_);

  // Assign the blending function ptr
  auto alpha_func = blend_func_ == SIGMOID ? &alpha_sigmoid : &alpha_exp;

  near_impact->set_timestamp(current_time);
  near_impact->SetCurrentContactMode(0);
  near_impact->SetAlpha(0);
  // Get current finite state
  if (current_time >= t0_) {
    for (int i = 0; i < impact_states_.size(); ++i) {
      double blend_window = blend_func_ == SIGMOID
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

void ContactScheduler::CalcClock(
    const Context<double> &context, BasicVector<double> *clock) const {
  // Read in lcm message time
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  clock->get_mutable_value()(0) = current_time;
}

void ContactScheduler::CalcContactScheduler(
    const Context<double> &context, BasicVector<double> *contact_timing) const {
  // Read in lcm message time
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, state_port_);
//  if (this->get_input_port(radio_port_).HasValue(context)) {
//    const auto &radio_out =
//        this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
//  }
  auto current_time = static_cast<double>(robot_output->get_timestamp());
  double remainder = fmod(current_time, period_);
  int n_periods = (int) (current_time / period_);

  for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
    contact_timing->get_mutable_value()(0) =
        n_periods * period_ + accu_state_durations_[i];
    contact_timing->get_mutable_value()(1) =
        n_periods * period_ + accu_state_durations_[i];
    contact_timing->get_mutable_value()(2) =
        n_periods * period_ + accu_state_durations_[1];
    contact_timing->get_mutable_value()(3) =
        n_periods * period_ + accu_state_durations_[4];
    contact_timing->get_mutable_value()(4) =
        (n_periods - 1) * period_ + accu_state_durations_[3];
    contact_timing->get_mutable_value()(5) =
        (n_periods - 1) * period_ + accu_state_durations_[3]
            + (accu_state_durations_[2] - accu_state_durations_[0])
            + (accu_state_durations_[4] - accu_state_durations_[3]);
  }
}

}  // namespace dairlib
