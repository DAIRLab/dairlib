#pragma once

#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {

/// DOUBLE_L_LO (Double stance left foot liftoff)
enum FSM_STATE { LEFT, DOUBLE_L_LO, RIGHT, DOUBLE_R_LO };

/// Event based FSM for jumping with option to change to a time-based FSM
/// @param[plant] The MultibodyPlant that this FSM operates with
/// @param[transition_times] Vector that contains the transition times for
/// each mode. For the contact driven FSM, only the transition time for the
/// initial state matters.
/// @param[contact_driven] Flag that switches between contact/event based and
/// time based FSM

class WalkingEventFsm : public drake::systems::LeafSystem<double> {
 public:
  WalkingEventFsm(const drake::multibody::MultibodyPlant<double>& plant,
                  const std::vector<double>& transition_times,
                  bool contact_based = true, FSM_STATE init_state = LEFT,
                  bool print_fsm_info = false);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_contact_input_port() const {
    return this->get_input_port(contact_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  void SetNextFiniteState(Eigen::VectorBlock<Eigen::VectorXd> fsm_state,
                          double timestamp) const;
  bool DetectGuardCondition(
      bool guard_condition, double current_time,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int state_port_;
  int contact_port_;
  std::vector<double> transition_times_;

  bool contact_based_;
  const FSM_STATE init_state_;
  bool print_fsm_info_;

//  double transition_delay_;
  int fsm_idx_;
  int prev_time_idx_;
};

}  // namespace examples
}  // namespace dairlib