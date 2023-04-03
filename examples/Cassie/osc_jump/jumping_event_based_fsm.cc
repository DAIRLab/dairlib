#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"

#include <dairlib/lcmt_controller_switch.hpp>
#include <iostream>
#include <drake/lcmt_contact_results_for_viz.hpp>

using dairlib::systems::OutputVector;
using dairlib::systems::ImpactInfoVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
namespace examples {
namespace osc_jump {

JumpingEventFsm::JumpingEventFsm(const MultibodyPlant<double>& plant,
                                 const vector<double>& transition_times,
                                 bool contact_based, double impact_threshold,
                                 JUMPING_FSM_STATE init_state, BLEND_FUNC blend_func)
    : transition_times_(transition_times),
      contact_based_(contact_based),
      impact_threshold_(impact_threshold),
      init_state_(init_state),
      blend_func_(blend_func) {
  state_port_ =
      this->DeclareVectorInputPort("x, u, t", OutputVector<double>(plant.num_positions(),
                                                                   plant.num_velocities(),
                                                                   plant.num_actuators()))
          .get_index();

  // Configure the contact info port for the particular simulator
  contact_port_ = this->DeclareAbstractInputPort(
          "lcmt_contact_results_for_viz",
          drake::Value<drake::lcmt_contact_results_for_viz>{})
      .get_index();
  fsm_output_port_ =
      this->DeclareVectorOutputPort("fsm", BasicVector<double>(1),
                                    &JumpingEventFsm::CalcFiniteState)
          .get_index();
  clock_output_port_ =
      this->DeclareVectorOutputPort("t_clock", BasicVector<double>(1),
                                    &JumpingEventFsm::CalcClock)
          .get_index();
  near_impact_output_port =
      this->DeclareVectorOutputPort("impact_info", ImpactInfoVector<double>(0, 0, 3),
                                    &JumpingEventFsm::CalcNearImpact)
          .get_index();
  DeclarePerStepDiscreteUpdateEvent(&JumpingEventFsm::DiscreteVariableUpdate);

  BasicVector<double> init_prev_time = BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> init_state_trigger_time =
      BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> init_fsm_state = BasicVector<double>(VectorXd::Zero(2));
  init_state_trigger_time.get_mutable_value()(0) = -1.0;
  init_fsm_state.get_mutable_value()(0) = init_state_;

  prev_time_idx_ = this->DeclareDiscreteState(init_prev_time);
  guard_trigger_time_idx_ = this->DeclareDiscreteState(init_state_trigger_time);
  fsm_idx_ = this->DeclareDiscreteState(init_fsm_state);
}

EventStatus JumpingEventFsm::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Get inputs to the leaf system
  const auto state_feedback =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto& contact_info =
      this->EvalInputValue<drake::lcmt_contact_results_for_viz>(context,
                                                                contact_port_);
  // Get the discrete states
  auto fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();
  auto state_trigger_time =
      discrete_state->get_mutable_vector(guard_trigger_time_idx_)
          .get_mutable_value();

  int num_contacts = contact_info->num_point_pair_contacts;
  double timestamp = state_feedback->get_timestamp();

  // Simulator has restarted, reset FSM
  if (timestamp < prev_time(0)) {
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
  }
  prev_time << timestamp;

  if (abs(transition_times_[BALANCE] - timestamp -
      round(transition_times_[BALANCE] - timestamp)) < 1e-3) {
    std::cout << "Time until crouch: "
              << round(transition_times_[BALANCE] - timestamp) << std::endl;
  }
  if (fsm_state(0) == BALANCE) {
    if (timestamp > transition_times_[BALANCE]) {
      fsm_state << CROUCH;
      std::cout << "Current time: " << timestamp << std::endl;
      std::cout << "Setting fsm to CROUCH" << std::endl;
      std::cout << "fsm: " << (JUMPING_FSM_STATE)fsm_state(0) << std::endl;
    }
  } else if (fsm_state(0) == CROUCH) {
    if (contact_based_ ? num_contacts == 0
                       : timestamp > transition_times_[CROUCH]) {
      fsm_state << FLIGHT;
      std::cout << "Current time: " << timestamp << std::endl;
      std::cout << "First detection time: " << state_trigger_time(0) << "\n";
      std::cout << "Setting fsm to FLIGHT" << std::endl;
      std::cout << "fsm: " << (JUMPING_FSM_STATE)fsm_state(0) << std::endl;
    }
  } else if (fsm_state(0) == FLIGHT) {
    if (contact_based_ ? num_contacts != 0
                       : timestamp > transition_times_[FLIGHT]) {
      fsm_state << LAND;
      std::cout << "Current time: " << timestamp << "\n";
      std::cout << "First detection time: " << state_trigger_time(0) << "\n";
      std::cout << "Setting fsm to LAND"
                << "\n";
      std::cout << "fsm: " << (JUMPING_FSM_STATE)fsm_state(0) << "\n";
    }
  } else if (fsm_state(0) == LAND) {
    // no more transitions
  }

  return EventStatus::Succeeded();
}

void JumpingEventFsm::CalcFiniteState(const Context<double>& context,
                                      BasicVector<double>* fsm_state) const {
  fsm_state->get_mutable_value() =
      context.get_discrete_state().get_vector(fsm_idx_).get_value();
}

void JumpingEventFsm::CalcClock(const Context<double>& context,
                                BasicVector<double>* clock) const {
  clock->get_mutable_value()
      << context.get_discrete_state(prev_time_idx_).get_value();
}

double alpha_sigmoid(double t, double tau, double near_impact_threshold) {
  double x = (t + near_impact_threshold) / tau;
  return exp(x) / (1 + exp(x));
}

double alpha_exp(double t, double tau, double near_impact_threshold) {
  return 1 - exp(-(t + near_impact_threshold) / tau);
}

void JumpingEventFsm::CalcNearImpact(const Context<double>& context,
                                     ImpactInfoVector<double>* near_impact) const {
  VectorXd fsm_state = context.get_discrete_state(fsm_idx_).get_value();
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  near_impact->set_timestamp(timestamp);
  near_impact->SetCurrentContactMode(0);
  near_impact->SetAlpha(0);
  auto alpha_func = blend_func_ == kSigmoid ? &alpha_sigmoid : &alpha_exp;

  // Get current finite state
  if (abs(timestamp - transition_times_[FLIGHT]) < impact_threshold_) {
    double blend_window =
        blend_func_ == kSigmoid ? 1.5 * impact_threshold_ : impact_threshold_;
    if (abs(timestamp - transition_times_[FLIGHT]) < blend_window) {
      if (timestamp < transition_times_[FLIGHT]) {
        near_impact->SetAlpha(alpha_func(timestamp - transition_times_[FLIGHT],
                                         tau_, impact_threshold_));
      } else {
        near_impact->SetAlpha(alpha_func(transition_times_[FLIGHT] - timestamp,
                                         tau_, impact_threshold_));
      }
      near_impact->SetCurrentContactMode(LAND);
    }
  }
}

}  // namespace osc_jump
}  // namespace examples
}  // namespace dairlib
