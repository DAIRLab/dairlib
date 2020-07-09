#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"

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

JumpingEventFsm::JumpingEventFsm(const MultibodyPlant<double>& plant,
                                 const vector<double>& transition_times,
                                 bool contact_based, double delay_time,
                                 FSM_STATE init_state)
    : plant_(plant),
      transition_times_(transition_times),
      contact_based_(contact_based),
      transition_delay_(delay_time),
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
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &JumpingEventFsm::CalcFiniteState);
  DeclarePerStepDiscreteUpdateEvent(&JumpingEventFsm::DiscreteVariableUpdate);

  BasicVector<double> init_prev_time = BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> init_state_trigger_time =
      BasicVector<double>(VectorXd::Zero(1));
  BasicVector<double> init_fsm_state = BasicVector<double>(VectorXd::Zero(1));
  init_state_trigger_time.get_mutable_value()(0) = -1.0;
  init_fsm_state.get_mutable_value()(0) = init_state_;

  prev_time_idx_ = this->DeclareDiscreteState(init_prev_time);
  guard_trigger_time_idx_ = this->DeclareDiscreteState(init_state_trigger_time);
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

  int num_contacts = contact_info->num_point_pair_contacts;
  double timestamp = state_feedback->get_timestamp();

  // Simulator has restarted, reset FSM
  if (timestamp < prev_time(0)) {
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
    prev_time(0) = timestamp;
    transition_flag(0) = false;
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
      prev_time(0) = timestamp;
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
      prev_time(0) = timestamp;
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
      prev_time(0) = timestamp;
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

bool JumpingEventFsm::DetectGuardCondition(
    bool guard_condition, double current_time,
    DiscreteValues<double>* discrete_state) const {
  auto transition_flag =
      discrete_state->get_mutable_vector(transition_flag_idx_)
          .get_mutable_value();
  // Second condition is to prevent overwriting state_trigger_time
  return guard_condition && !(bool)transition_flag(0);
}

}  // namespace examples
}  // namespace dairlib
