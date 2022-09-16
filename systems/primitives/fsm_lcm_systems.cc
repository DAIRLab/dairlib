#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/framework/output_vector.h"

namespace dairlib{
namespace systems{

using systems::OutputVector;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::multibody::MultibodyPlant;


FsmReceiver::FsmReceiver(const MultibodyPlant<double> &plant) {
  this->set_name("FSM Receiver");
  input_port_lcmt_fsm_info_ = this->DeclareAbstractInputPort(
      "lcmt_fsm_info", drake::Value<lcmt_fsm_info>({0,0,0,0})).get_index();
  input_port_state_ = this->DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant.num_positions(),
                                      plant.num_velocities(),
                                      plant.num_actuators())).get_index();

  output_port_fsm_ = this->DeclareVectorOutputPort(
          "fsm", 1, &FsmReceiver::CopyFsm)
      .get_index();
  output_port_prev_switch_time_ = this->DeclareVectorOutputPort(
          "prev_switch_time", 1, &FsmReceiver::CopyPrevSwitchTime)
      .get_index();
  output_port_next_switch_time_ = this->DeclareVectorOutputPort(
          "next_switch_time", 1, &FsmReceiver::CopyNextSwitchTime)
      .get_index();
  output_port_seconds_since_prev_switch_ = this->DeclareVectorOutputPort(
          "time_since_prev_switch", 1, &FsmReceiver::CopyTimeSinceSwitch)
      .get_index();
  output_port_seconds_until_next_switch_ = this->DeclareVectorOutputPort(
          "time_to_next_switch", 1, &FsmReceiver::CopyTimeUntilSwitch)
      .get_index();

  offset_idx_ = this->DeclareDiscreteState(1);
  DeclarePerStepUnrestrictedUpdateEvent(&FsmReceiver::UnrestrictedUpdate);
}

drake::systems::EventStatus FsmReceiver::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {
  const auto& fsm_msg = eval_fsm_port(context);
  double t = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, input_port_state_))->get_timestamp();
  double offset = t - static_cast<double>(fsm_msg.timestamp_us) * 1e-6;
  state->get_mutable_discrete_state(offset_idx_).get_mutable_value()(0) = offset;
  return drake::systems::EventStatus::Succeeded();
}

const dairlib::lcmt_fsm_info& FsmReceiver::eval_fsm_port(
    const drake::systems::Context<double>& context) const {
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, input_port_lcmt_fsm_info_);
  DRAKE_ASSERT(input != nullptr);
  const auto& fsm_msg = input->get_value<dairlib::lcmt_fsm_info>();
  return fsm_msg;
}

void FsmReceiver::CopyFsm(const Context<double> &context,
                          BasicVector<double> *y) const {
  const auto& fsm_msg = eval_fsm_port(context);
  y->get_mutable_value()(0) = fsm_msg.fsm_state;
}

void FsmReceiver::CopyPrevSwitchTime(const Context<double> &context,
                                     BasicVector<double> *y) const {
  const auto& fsm_msg = eval_fsm_port(context);
  double offset = context.get_discrete_state(offset_idx_).get_value()(0);
  y->get_mutable_value()(0) =
      static_cast<double>(fsm_msg.prev_switch_time_us) * 1e-6;// + offset;
}

void FsmReceiver::CopyNextSwitchTime(const Context<double> &context,
                                     BasicVector<double> *y) const {
  const auto& fsm_msg = eval_fsm_port(context);
  double offset = context.get_discrete_state(offset_idx_).get_value()(0);
  y->get_mutable_value()(0) =
      static_cast<double>(fsm_msg.next_switch_time_us) * 1e-6;// + offset;
}
void FsmReceiver::CopyTimeSinceSwitch(const Context<double> &context,
                                      BasicVector<double> *y) const {
  const auto& fsm_msg = eval_fsm_port(context);
  double offset = context.get_discrete_state(offset_idx_).get_value()(0);
  double t = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, input_port_state_))->get_timestamp();
  y->get_mutable_value()(0) =
      t - (static_cast<double>(fsm_msg.prev_switch_time_us) * 1e-6 + offset);
}

void FsmReceiver::CopyTimeUntilSwitch(const Context<double> &context,
                                      BasicVector<double> *y) const {
  const auto& fsm_msg = eval_fsm_port(context);
  double offset = context.get_discrete_state(offset_idx_).get_value()(0);
  double t = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, input_port_state_))->get_timestamp();
  y->get_mutable_value()(0) =
      (static_cast<double>(fsm_msg.next_switch_time_us) * 1e-6 + offset) - t;
}

FsmSender::FsmSender(const drake::multibody::MultibodyPlant<double> &plant) {
  this->set_name("FSM Sender");
  input_port_state_ = this->DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant.num_positions(),
                                      plant.num_velocities(),
                                      plant.num_actuators())).get_index();
  input_port_fsm_ = this->DeclareVectorInputPort("fsm", 1).get_index();
  input_port_prev_switch_time_ = this->DeclareVectorInputPort("prev_switch_time", 1).get_index();
  input_port_next_switch_time_ = this->DeclareVectorInputPort("next_switch_time", 1).get_index();
  output_port_fsm_info_ = this->DeclareAbstractOutputPort(
      "fsm_info", dairlib::lcmt_fsm_info(),&FsmSender::CopyInputsToMessage)
  .get_index();
}

void FsmSender::CopyInputsToMessage(const Context<double> &context,
                                    lcmt_fsm_info *msg) const {
  double t = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, input_port_state_))->get_timestamp();
  msg->timestamp_us = static_cast<long>(t * 1e6);
  msg->fsm_state = static_cast<int>(
      this->EvalVectorInput(context, input_port_fsm_)->get_value()(0));
  msg->prev_switch_time_us = static_cast<int>(
      this->EvalVectorInput(context, input_port_prev_switch_time_)
  ->get_value()(0)*1e6);
  msg->next_switch_time_us = static_cast<int>(
      this->EvalVectorInput(context, input_port_next_switch_time_)
          ->get_value()(0)*1e6);

}

}
}