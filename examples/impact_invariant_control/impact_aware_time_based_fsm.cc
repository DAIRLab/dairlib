#include "examples/impact_invariant_control/impact_aware_time_based_fsm.h"

using std::cout;
using std::endl;

using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using std::string;

namespace dairlib {

using systems::OutputVector;

ImpactTimeBasedFiniteStateMachine::ImpactTimeBasedFiniteStateMachine(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<int>& states, const std::vector<double>& state_durations,
    double t0, double near_impact_threshold, BLEND_FUNC blend_func)
    : TimeBasedFiniteStateMachine(plant, states, state_durations, t0),
      states_(states),
      state_durations_(state_durations),
      t0_(t0),
      near_impact_threshold_(near_impact_threshold),
      blend_func_(blend_func){
  near_impact_port_ =
      this->DeclareVectorOutputPort(
              BasicVector<double>(2),
              &ImpactTimeBasedFiniteStateMachine::CalcNearImpact)
          .get_index();

  // Accumulate the durations to get timestamps
  double sum = 0;
  for (auto& duration_i : state_durations) {
    sum += duration_i;
    accu_state_durations_.push_back(sum);
  }
  period_ = sum;
}

double alpha_sigmoid(double t, double tau, double near_impact_threshold) {
  double x = (t + near_impact_threshold) / tau;
  return exp(x) / (1 + exp(x));
}

double alpha_exp(double t, double tau, double near_impact_threshold) {
  return 1 - exp(-(t + near_impact_threshold) / tau);
}

void ImpactTimeBasedFiniteStateMachine::CalcNearImpact(
    const Context<double>& context, BasicVector<double>* near_impact) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_time, period_);

  VectorXd near_impact_data = VectorXd::Zero(2);
  // Get current finite state
  if (current_time >= t0_) {
    //    for (unsigned int i : impact_states_) {
    for (int i = 0; i < impact_states_.size(); ++i) {
      double blend_window = blend_func_ == SIGMOID
                                ? 1.5 * near_impact_threshold_
                                : near_impact_threshold_;
      if (abs(remainder - impact_times_[i]) < blend_window) {
        if (remainder < impact_times_[i]) {
          if (blend_func_ == SIGMOID) {
            near_impact_data(0) = alpha_sigmoid(remainder - impact_times_[i],
                                                tau_, near_impact_threshold_);
          } else {
            near_impact_data(0) = alpha_exp(remainder - impact_times_[i], tau_,
                                            near_impact_threshold_);
          }
        } else {
          if (blend_func_ == SIGMOID) {
            near_impact_data(0) = alpha_sigmoid(impact_times_[i] - remainder,
                                                tau_, near_impact_threshold_);
          } else {
            near_impact_data(0) = alpha_exp(impact_times_[i] - remainder, tau_,
                                            near_impact_threshold_);
          }
        }
        near_impact_data(1) = impact_states_[i];
        break;
      }
    }
  }
  near_impact->get_mutable_value() = near_impact_data;
}

}  // namespace dairlib
