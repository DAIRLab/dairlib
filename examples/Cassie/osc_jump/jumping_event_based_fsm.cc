#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "dairlib/lcmt_cassie_mujoco_contact.hpp"
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
                                 bool contact_based, FSM_STATE init_state,
                                 SIMULATOR simulator_type)
    : plant_(plant),
      transition_times_(transition_times),
      contact_based_(contact_based),
      init_state_(init_state),
      simulator_type_(simulator_type)
      {
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
              plant.num_velocities(),
              plant.num_actuators()))
          .get_index();
  if (simulator_type_ == DRAKE) {
    contact_port_ = this->DeclareAbstractInputPort(
                            "lcmt_contact_info",
                            drake::Value<drake::lcmt_contact_results_for_viz>{})
                        .get_index();
  } else if (simulator_type_ == MUJOCO) {
    contact_port_ = this->DeclareAbstractInputPort(
                            "lcmt_contact_info",
                            drake::Value<dairlib::lcmt_cassie_mujoco_contact>{})
                        .get_index();
  } else if (simulator_type_ == GAZEBO) {
  }
  this->DeclareVectorOutputPort(BasicVector<double>(1),
      &JumpingEventFsm::CalcFiniteState);
  DeclarePerStepDiscreteUpdateEvent(
      &JumpingEventFsm::DiscreteVariableUpdate);

  prev_time_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);
}

EventStatus JumpingEventFsm::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, contact_port_);
  DRAKE_ASSERT(input != nullptr);

  auto fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  int num_contacts = 0;
  if(simulator_type_ == DRAKE){
    const auto& contact_info_msg =
        input->get_value<drake::lcmt_contact_results_for_viz>();
    num_contacts = contact_info_msg.num_point_pair_contacts;
  }
  else if (simulator_type_ == MUJOCO){
    // MuJoCo has "persistent" contact so we have to check contact forces
    // instead of just a boolean value of on/off
    const auto& contact_info_msg =
        input->get_value<dairlib::lcmt_cassie_mujoco_contact>();
//    num_contacts = contact_info_msg.num_contact_forces;
    num_contacts = std::count_if(contact_info_msg.contact_forces.begin(),
                                 contact_info_msg.contact_forces.end(),
                                 [&](auto const& force) {
                                   double threshold = 1e-6;
                                   return std::abs(force) >= threshold;
                                 });
//    std::cout << " n contacts: " << num_contacts << std::endl;
  }
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();

  const OutputVector<double>* robot_output =
  (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  if (current_time < prev_time(0)) {  // Simulator has restarted, reset FSM
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
    prev_time(0) = current_time;
  }

  if (contact_based_) {
    switch ((FSM_STATE)fsm_state(0)) {
      case (BALANCE):
        if (current_time > transition_times_[BALANCE]) {
          fsm_state << CROUCH;
          std::cout << "Current time: " << current_time << std::endl;
          std::cout << "Setting fsm to CROUCH" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (CROUCH): // This assumes perfect knowledge about contacts
        if (num_contacts == 0 &&
            (current_time - prev_time(0)) > 0.05) {
          fsm_state << FLIGHT;
          std::cout << "Current time: " << current_time << std::endl;
          std::cout << "Setting fsm to FLIGHT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (FLIGHT):
        if (num_contacts != 0 &&
            (current_time - prev_time(0)) > 0.05) {
          fsm_state << LAND;
          std::cout << "Current time: " << current_time << std::endl;
          std::cout << "Setting fsm to LAND" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (LAND):
        break;
    }
  } else {
    switch ((FSM_STATE)fsm_state(0)) {
      case (BALANCE):
        if (current_time > transition_times_[BALANCE]) {
          fsm_state << CROUCH;
          std::cout << "Setting fsm to CROUCH" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (CROUCH):
        if (current_time > transition_times_[CROUCH]) {
          fsm_state << FLIGHT;
          std::cout << "Setting fsm to FLIGHT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (FLIGHT):
        if (current_time > transition_times_[FLIGHT]) {
          fsm_state << LAND;
          std::cout << "Setting fsm to LAND" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (LAND):
        // Sink state, should not switch to any other mode
        break;
    }
  }

  return EventStatus::Succeeded();
}

void JumpingEventFsm::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  fsm_state->get_mutable_value() =
      context.get_discrete_state().get_vector(fsm_idx_).get_value();
}

}  // namespace examples
}  // namespace dairlib