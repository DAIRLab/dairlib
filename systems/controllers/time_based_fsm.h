#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

// Time-based Two-state Finite State Machine

/// Constructor inputs:
/// - rigid body tree
/// - duration of each state
/// - offset of the time when the state switches

/// There are three outcome of the states, beased on the current time t.
/// If t < time_shift,
///     state = `initial_state_number_`.
/// If t is within [i * `duration_per_state` + `time_shift`,
///                 (i+1) * `duration_per_state` + `time_shift`)
///                where i is an even number,
///     state = `first_state_number_`.
/// If t is within [i * `duration_per_state` + `time_shift`,
///                 (i+1) * `duration_per_state` + `time_shift`)
///                where i is an odd number,
///     state = `second_state_number_`.
class TimeBasedFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  TimeBasedFiniteStateMachine(const RigidBodyTree<double>& tree,
                              double duration_per_state,
                              double time_shift = 0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  int state_port_;

  int first_state_number_ = 0;
  int second_state_number_ = 1;
  int initial_state_number_ = -1;
  double duration_per_state_;
  double time_shift_;
};


}  // namespace systems
}  // namespace dairlib


