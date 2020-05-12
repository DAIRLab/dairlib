#include "examples/five_link_biped/walking_fsm.h"
#include "dairlib/lcmt_fsm_out.hpp"
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

WalkingFiniteStateMachine::WalkingFiniteStateMachine(
    const MultibodyPlant<double>& plant, double r_impact_time,
    double l_impact_time, double delay_time, bool contact_driven,
    int init_state)
    : plant_(plant),
      r_impact_time_(r_impact_time),
      l_impact_time_(l_impact_time),
      delay_time_(delay_time),
      contact_driven_(contact_driven),
      init_state_((FSM_STATE)init_state) {
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();
  fsm_output_port_ = this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &WalkingFiniteStateMachine::CalcFiniteState)
                                    .get_index();
  lcm_port_ = this->DeclareAbstractOutputPort(
                      &WalkingFiniteStateMachine::AssignLcmOutput)
                  .get_index();

  DeclarePerStepDiscreteUpdateEvent(
      &WalkingFiniteStateMachine::DiscreteVariableUpdate);
  // indices for discrete variables in drake leafsystem
  prev_time_idx_ = this->DeclareDiscreteState(1);
  contact_time_idx_ = this->DeclareDiscreteState(1);
  contact_flag_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);
}

EventStatus WalkingFiniteStateMachine::DiscreteVariableUpdate(
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
  auto contact_time =
      discrete_state->get_mutable_vector(contact_time_idx_).get_mutable_value();
  auto contact_flag =
      discrete_state->get_mutable_vector(contact_flag_idx_).get_mutable_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  if (current_time < prev_time(0)) {  // Simulator has restarted
    std::cout << "Simulator has restarted!" << std::endl;
    fsm_state << init_state_;
    prev_time(0) = current_time;
  }

  if(delay_time_ < 0.0){
    switch ((FSM_STATE)fsm_state(0)) {
      case (LEFT_FOOT):
        if (current_time >= r_impact_time_ + delay_time_) {
          fsm_state << RIGHT_FOOT;
          std::cout << "current time: " << current_time << std::endl;
          std::cout << "Setting fsm to RIGHT_FOOT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (RIGHT_FOOT):
        if (current_time >= l_impact_time_ + delay_time_) {
          fsm_state << LEFT_FOOT_2;
          std::cout << "current time: " << current_time << std::endl;
          std::cout << "Setting fsm to LEFT_FOOT_2" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (LEFT_FOOT_2):
        break;
    }
  }
  else if (contact_driven_) {
    switch ((FSM_STATE)fsm_state(0)) {
      case (LEFT_FOOT):
        if (contact_info_msg.num_point_pair_contacts != 1 &&
            (current_time - prev_time(0)) > 0.05) {
          fsm_state << RIGHT_FOOT;
          std::cout << "Setting fsm to RIGHT_FOOT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (RIGHT_FOOT):
        if (contact_info_msg.num_point_pair_contacts != 1 &&
            (current_time - prev_time(0)) > 0.05) {
          fsm_state << LEFT_FOOT_2;
          std::cout << "Setting fsm to LEFT_FOOT_2" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (LEFT_FOOT_2):
        break;
    }
  } else if (!contact_driven_) {
    switch ((FSM_STATE)fsm_state(0)) {
      case (LEFT_FOOT):
//        if (contact_info_msg.num_point_pair_contacts != 1 && !contact_flag(0) &&
        if (contact_info_msg.num_point_pair_contacts == 2 && !contact_flag(0) &&
            (current_time - prev_time(0)) > 0.02) {
          contact_time(0) = current_time;
          contact_flag(0) = true;
        }
        if (current_time - contact_time(0) >= delay_time_ && contact_flag(0)) {
          std::cout << "current time: " << current_time << std::endl;
          std::cout << "contact time: " << contact_time(0) << std::endl;
          fsm_state << RIGHT_FOOT;
          std::cout << "Setting fsm to RIGHT_FOOT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          contact_flag(0) = false;
          prev_time(0) = current_time;
        }
        break;
      case (RIGHT_FOOT):
        if (contact_info_msg.num_point_pair_contacts == 2 && !contact_flag(0) &&
            (current_time - prev_time(0)) > 0.05) {
          contact_time(0) = current_time;
          contact_flag(0) = true;
        }
        if (current_time - contact_time(0) >= delay_time_ && contact_flag(0)) {
          std::cout << "current time: " << current_time << std::endl;
          std::cout << "contact time: " << contact_time(0) << std::endl;
          fsm_state << LEFT_FOOT_2;
          std::cout << "Setting fsm to LEFT_FOOT_2" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          contact_flag(0) = false;
          prev_time(0) = current_time;
        }
        break;
      case (LEFT_FOOT_2):
        break;
    }
  } else {
    switch ((FSM_STATE)fsm_state(0)) {
      case (LEFT_FOOT):
        if (current_time > r_impact_time_) {
          fsm_state << RIGHT_FOOT;
          std::cout << "Setting fsm to RIGHT_FOOT" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
      case (RIGHT_FOOT):
        if (current_time > l_impact_time_) {
          fsm_state << LEFT_FOOT_2;
          std::cout << "Setting fsm to LEFT_FOOT_2" << std::endl;
          std::cout << "fsm: " << (FSM_STATE)fsm_state(0) << std::endl;
          prev_time(0) = current_time;
        }
        break;
    }
  }

  return EventStatus::Succeeded();
}

void WalkingFiniteStateMachine::CalcFiniteState(
    const Context<double>& context, BasicVector<double>* fsm_state) const {
  fsm_state->get_mutable_value() =
      context.get_discrete_state().get_vector(fsm_idx_).get_value();
}

void WalkingFiniteStateMachine::AssignLcmOutput(
    const Context<double>& context, dairlib::lcmt_fsm_out* output) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  output->utime = robot_output->get_timestamp();
  output->fsm_state =
      context.get_discrete_state().get_vector(fsm_idx_).get_value()(0);
}

}  // namespace examples
}  // namespace dairlib
