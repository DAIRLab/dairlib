#pragma once

#include "examples/impact_invariant_control/impact_aware_time_based_fsm.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/impact_info_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {

namespace osc_jump {
enum JUMPING_FSM_STATE { READY, BALANCE, CROUCH, FLIGHT, LAND };

/// Event based FSM for jumping with option to change to a time-based FSM
/// @param[plant] The MultibodyPlant that this FSM operates with
/// @param[transition_times] Vector that contains the transition times for
/// each mode. For the contact driven FSM, only the transition time for the
/// initial state matters.
/// @param[contact_driven] Flag that switches between contact/event based and
/// time based FSM

class JumpingEventFsm : public drake::systems::LeafSystem<double> {
 public:
  JumpingEventFsm(const drake::multibody::MultibodyPlant<double>& plant,
                  const std::vector<double>& transition_times,
                  bool contact_based = true, double impact_threshold = 0.0,
                  JUMPING_FSM_STATE init_state = BALANCE,
                  BLEND_FUNC blend_func = kSigmoid);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_contact_input_port() const {
    return this->get_input_port(contact_port_);
  }

  const drake::systems::InputPort<double>& get_switch_input_port() const {
    return this->get_input_port(switch_signal_port_);
  }

  const drake::systems::OutputPort<double>& get_fsm_output_port() const {
    return this->get_output_port(fsm_output_port_);
  }

  const drake::systems::OutputPort<double>& get_clock_output_port() const {
    return this->get_output_port(clock_output_port_);
  }

  const drake::systems::OutputPort<double>& get_impact_output_port() const {
    return this->get_output_port(near_impact_output_port);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  void CalcClock(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* clock) const;

  void CalcNearImpact(const drake::systems::Context<double>& context,
                      systems::ImpactInfoVector<double>* fsm_state) const;

  int state_port_;
  int contact_port_;
  int switch_signal_port_;
  int fsm_output_port_;
  int clock_output_port_;
  int near_impact_output_port;
  std::vector<double> transition_times_;

  bool contact_based_;

  double tau_ = 0.0025;
  double impact_threshold_;
  int fsm_idx_;
  int prev_time_idx_;
  int guard_trigger_time_idx_;

  const JUMPING_FSM_STATE init_state_;
  BLEND_FUNC blend_func_;
};

}  // namespace osc_jump
}  // namespace examples
}  // namespace dairlib