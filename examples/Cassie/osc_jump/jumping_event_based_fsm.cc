#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"

#include <dairlib/lcmt_controller_switch.hpp>
#include <drake/lcmt_contact_results_for_viz.hpp>

using dairlib::systems::OutputVector;
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
                                 bool contact_based, double delay_time,
                                 double impact_threshold, FSM_STATE init_state)
    : transition_times_(transition_times),
      contact_based_(contact_based),
      transition_delay_(delay_time),
      impact_threshold_(impact_threshold),
      init_state_(init_state) {
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  // Configure the contact info port for the particular simulator
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();
  switch_signal_port_ = this->DeclareAbstractInputPort(
                                "lcmt_controller_switch",
                                drake::Value<dairlib::lcmt_controller_switch>{})
                            .get_index();
  fsm_output_port_ =
      this->DeclareVectorOutputPort(BasicVector<double>(1),
                                    &JumpingEventFsm::CalcFiniteState)
          .get_index();
  clock_output_port_ =
      this->DeclareVectorOutputPort(BasicVector<double>(1),
                                    &JumpingEventFsm::CalcClockTime)
          .get_index();
  near_impact_output_port =
      this->DeclareVectorOutputPort(BasicVector<double>(2),
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
  switching_time_idx_ = this->DeclareDiscreteState(0.0);
  fsm_idx_ = this->DeclareDiscreteState(init_fsm_state);
  transition_flag_idx_ = this->DeclareDiscreteState(1);
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
  //  const auto switch_signal =
  //      this->EvalInputValue<dairlib::lcmt_controller_switch>(
  //          context, switch_signal_port_);
  // Get the discrete states
  auto fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();
  auto state_trigger_time =
      discrete_state->get_mutable_vector(guard_trigger_time_idx_)
          .get_mutable_value();
  auto transition_flag =
      discrete_state->get_mutable_vector(transition_flag_idx_)
          .get_mutable_value();
  auto prev_switch_time =
      discrete_state->get_mutable_vector(switching_time_idx_)
          .get_mutable_value();

  int num_contacts = contact_info->num_point_pair_contacts;
  double timestamp = state_feedback->get_timestamp();

  // Simulator has restarted, reset FSM
  if (timestamp < prev_time(0)) {
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
    transition_flag(0) = false;
    //    prev_switch_time << switch_signal->utime;
  }
  prev_time << timestamp;

  if (abs(transition_times_[BALANCE] - timestamp -
          round(transition_times_[BALANCE] - timestamp)) < 1e-3) {
    std::cout << "Time until crouch: "
              << round(transition_times_[BALANCE] - timestamp) << std::endl;
  }
  // To test delayed switching times, there is an "intermediate" state
  // between each state change when the guard condition is first triggered
  // The fsm state will change transition_delay_ seconds after the guard
  // condition was first triggered.
  // This supports both contact-based and time-based guard conditions
  // TODO(yangwill) Remove timing delays once hardware testing is finished
  if (fsm_state(0) == BALANCE) {
    if (timestamp > transition_times_[BALANCE]) {
      fsm_state << CROUCH;
      std::cout << "Current time: " << timestamp << std::endl;
      std::cout << "Setting fsm to CROUCH" << std::endl;
      std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
      transition_flag(0) = false;
    }
  } else if (fsm_state(0) == CROUCH) {
    if (DetectGuardCondition(contact_based_
                                 ? num_contacts == 0
                                 : timestamp > transition_times_[CROUCH],
                             timestamp, discrete_state)) {
      state_trigger_time(0) = timestamp;
      transition_flag(0) = true;
    }
    if (timestamp - state_trigger_time(0) >= transition_delay_ &&
        (bool)transition_flag(0)) {
      fsm_state << FLIGHT;
      std::cout << "Current time: " << timestamp << std::endl;
      std::cout << "First detection time: " << state_trigger_time(0) << "\n";
      std::cout << "Setting fsm to FLIGHT" << std::endl;
      std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
      transition_flag(0) = false;
    }
  } else if (fsm_state(0) == FLIGHT) {
    if (DetectGuardCondition(contact_based_
                                 ? num_contacts != 0
                                 : timestamp > transition_times_[FLIGHT],
                             timestamp, discrete_state)) {
      state_trigger_time(0) = timestamp;
      transition_flag(0) = true;
    }
    if (timestamp - state_trigger_time(0) >= transition_delay_ &&
        (bool)transition_flag(0)) {
      fsm_state << LAND;
      std::cout << "Current time: " << timestamp << "\n";
      std::cout << "First detection time: " << state_trigger_time(0) << "\n";
      std::cout << "Setting fsm to LAND"
                << "\n";
      std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << "\n";
      transition_flag(0) = false;
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

void JumpingEventFsm::CalcNearImpact(const Context<double>& context,
                                     BasicVector<double>* near_impact) const {
  VectorXd fsm_state = context.get_discrete_state(fsm_idx_).get_value();
  // Read in lcm message time
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  VectorXd is_near_impact = VectorXd::Zero(2);
  // Get current finite state
  if (abs(timestamp - transition_times_[FLIGHT]) < impact_threshold_) {
    is_near_impact(0) = 1;
    is_near_impact(1) = LAND;
  }
  near_impact->get_mutable_value() = is_near_impact;
}

void JumpingEventFsm::CalcClockTime(const Context<double>& context,
                                    BasicVector<double>* clock) const {
  double robot_time =
      this->template EvalVectorInput<OutputVector>(context, state_port_)
          ->get_timestamp();
  clock->get_mutable_value()
      << robot_time - context.get_discrete_state()
                          .get_vector(switching_time_idx_)
                          .get_value()(0);
}

bool JumpingEventFsm::DetectGuardCondition(
    bool guard_condition, double current_time,
    DiscreteValues<double>* discrete_state) const {
  auto transition_flag =
      discrete_state->get_mutable_vector(transition_flag_idx_)
          .get_mutable_value();
  // Second condition is to prevent overwriting state_trigger_time
  return guard_condition && !(bool)transition_flag(0);
}

}  // namespace osc_jump
}  // namespace examples
}  // namespace dairlib
