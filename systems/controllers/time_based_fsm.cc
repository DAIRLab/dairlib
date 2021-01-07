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
    double t0, double near_impact_threshold)
    : states_(states),
      state_durations_(state_durations),
      t0_(t0),
      near_impact_threshold_(near_impact_threshold) {
  DRAKE_DEMAND(states.size() == state_durations.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorOutputPort(
                      BasicVector<double>(1),
                      &TimeBasedFiniteStateMachine::CalcFiniteState)
                  .get_index();
  near_impact_port_ = this->DeclareVectorOutputPort(
                              BasicVector<double>(2),
                              &TimeBasedFiniteStateMachine::CalcNearImpact)
                          .get_index();

  // Accumulate the durations to get timestamps
  double sum = 0;
  for (auto& duration_i : state_durations) {
    sum += duration_i;
    accu_state_durations_.push_back(sum);
  }
  period_ = sum;
}

void TimeBasedFiniteStateMachine::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());

  //  double m = floor((current_sim_time - t0_) / period_);
  //  double remainder = (current_sim_time - t0_) - m * period_;
  double remainder = fmod(current_sim_time, period_);

  // Get current finite state
  VectorXd current_finite_state = VectorXd::Zero(1);
  if (current_sim_time >= t0_) {
    for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
      if (remainder < accu_state_durations_[i]) {
        //        current_finite_state << states_[i];
        current_finite_state(0) = states_[i];
        break;
      }
    }
  } else {
    current_finite_state << initial_state_;
  }

  // Assign fsm_state
  fsm_state->get_mutable_value() = current_finite_state;
}

void TimeBasedFiniteStateMachine::CalcNearImpact(
    const Context<double>& context, BasicVector<double>* near_impact) const {
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto current_sim_time = static_cast<double>(robot_output->get_timestamp());

  double remainder = fmod(current_sim_time, period_);

  VectorXd is_near_impact = VectorXd::Zero(2);
  // Get current finite state
  if (current_sim_time >= t0_) {
    for (unsigned int i = 0; i < accu_state_durations_.size(); i++) {
      if (abs(remainder - accu_state_durations_[i]) < near_impact_threshold_) {
        is_near_impact(0) = 1;
        is_near_impact(1) = states_[i];
        break;
      }
    }
  }
  near_impact->get_mutable_value() = is_near_impact;
}

}  // namespace systems
}  // namespace dairlib
