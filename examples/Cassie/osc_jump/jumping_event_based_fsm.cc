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

namespace dairlib {
namespace examples {

JumpingEventFsm::JumpingEventFsm(
    const MultibodyPlant<double>& plant, double flight_time,
    double land_time, double delay_time, bool contact_driven,
    FSM_STATE init_state)
    : plant_(plant),
      flight_time_(flight_time),
      land_time_(land_time),
      delay_time_(delay_time),
      contact_driven_(contact_driven),
      init_state_(init_state) {
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
              plant.num_velocities(),
              plant.num_actuators()))
          .get_index();
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();
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
  const auto& contact_info_msg =
      input->get_value<drake::lcmt_contact_results_for_viz>();
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();

  const OutputVector<double>* robot_output =
  (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  if (current_time < prev_time(0)) {  // Simulator has restarted
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
    prev_time(0) = current_time;
  }

  if (contact_driven_) {
    switch ((FSM_STATE)fsm_state(0)) {
      case (BALANCE):
        if (current_time > delay_time_) {
          fsm_state << CROUCH;
          std::cout << "Setting fsm to CROUCH" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (CROUCH): // This assumes perfect knowledge about contacts
        if (contact_info_msg.num_point_pair_contacts == 0 &&
            (current_time - prev_time(0)) > 0.05) {
          fsm_state << FLIGHT;
          std::cout << "Setting fsm to FLIGHT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (FLIGHT):
        if (contact_info_msg.num_point_pair_contacts != 0 &&
            (current_time - prev_time(0)) > 0.05) {
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
  } else {
    switch ((FSM_STATE)fsm_state(0)) {
      case (BALANCE):
        if (current_time > delay_time_) {
          fsm_state << CROUCH;
          std::cout << "Setting fsm to CROUCH" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (CROUCH):
        if (current_time > flight_time_) {
          fsm_state << FLIGHT;
          std::cout << "Setting fsm to FLIGHT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (FLIGHT):
        if (current_time > land_time_) {
          fsm_state << LAND;
          std::cout << "Setting fsm to LAND" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (LAND):
        break;
        // Do nothing
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