#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

// Time-based Two-state Finite State Machine

/// Constructor inputs:
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
  TimeBasedFiniteStateMachine(double duration_per_state,
                              double time_shift = 0);

 private:
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  double duration_per_state_;
  double time_shift_;
  const int first_state_number_ = 0;
  const int second_state_number_ = 1;
  const int initial_state_number_ = -1;
};


}  // namespace systems
}  // namespace dairlib


