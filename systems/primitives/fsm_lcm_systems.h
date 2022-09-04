#pragma once
#include "dairlib/lcmt_fsm_info.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// FsmReviever recieves FSM info messages, and outputs the fsm state, as well
/// as timing information related to the next switching time. It takes the robot
/// state OutputVector as in input in order to synchronize it's clock
/// to the clock sending the fsm commands.
class FsmReceiver : public drake::systems::LeafSystem<double> {
 public:
  FsmReceiver(const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_fsm_info() {
    return this->get_input_port(input_port_lcmt_fsm_info_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() {
    return this->get_input_port(input_port_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm() {
    return this->get_output_port(output_port_fsm_);
  }
  const drake::systems::OutputPort<double>& get_output_port_prev_switch_time() {
    return this->get_output_port(output_port_prev_switch_time_);
  }
  const drake::systems::OutputPort<double>& get_output_port_next_switch_time() {
    return this->get_output_port(output_port_next_switch_time_);
  }
  const drake::systems::OutputPort<double>& get_output_port_time_since_switch() {
    return this->get_output_port(output_port_seconds_since_prev_switch_);
  }
  const drake::systems::OutputPort<double>& get_output_port_time_until_switch() {
    return this->get_output_port(output_port_seconds_until_next_switch_);
  }

 private:
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyFsm(const drake::systems::Context<double>& context,
               drake::systems::BasicVector<double>* y) const;
  void CopyPrevSwitchTime(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* y) const;
  void CopyNextSwitchTime(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* y) const;
  void CopyTimeSinceSwitch(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* y) const;
  void CopyTimeUntilSwitch(const drake::systems::Context<double>& context,
                     drake::systems::BasicVector<double>* y) const;

  const dairlib::lcmt_fsm_info& eval_fsm_port(
      const drake::systems::Context<double>& context) const;


  drake::systems::DiscreteStateIndex offset_idx_;
  drake::systems::InputPortIndex input_port_lcmt_fsm_info_;
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::OutputPortIndex output_port_fsm_;
  drake::systems::OutputPortIndex output_port_prev_switch_time_;
  drake::systems::OutputPortIndex output_port_next_switch_time_;
  drake::systems::OutputPortIndex output_port_seconds_since_prev_switch_;
  drake::systems::OutputPortIndex output_port_seconds_until_next_switch_;

};

class FsmSender : public drake::systems::LeafSystem<double> {
 public:
  FsmSender(const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_state() {
    return this->get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() {
    return this->get_input_port(input_port_fsm_);
  }
  const drake::systems::InputPort<double>& get_input_port_prev_switch_time() {
    return this->get_input_port(input_port_prev_switch_time_);
  }
  const drake::systems::InputPort<double>& get_input_port_next_switch_time() {
    return this->get_input_port(input_port_next_switch_time_);
  }
  const drake::systems::OutputPort<double>& get_output_port_fsm_info() {
    return this->get_output_port(output_port_fsm_info_);
  }

 private:
  drake::systems::InputPortIndex input_port_fsm_;
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_prev_switch_time_;
  drake::systems::InputPortIndex input_port_next_switch_time_;
  drake::systems::OutputPortIndex output_port_fsm_info_;
  void CopyInputsToMessage(const drake::systems::Context<double>& context,
                            dairlib::lcmt_fsm_info* msg) const;

};
}
}
