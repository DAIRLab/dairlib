#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

// Time-based Two-state Finite State Machine
class TimeBasedFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  TimeBasedFiniteStateMachine(int num_positions,
                              int num_velocities,
                              int num_inputs,
                              int first_state_number,
                              int second_state_number,
                              std::string first_state_name,
                              std::string second_state_name,
                              int start_state_number,
                              double duration_per_state,
                              double time_shift = 0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  int state_port_;

  int first_state_number_;
  int second_state_number_;
  std::string first_state_name_;
  std::string second_state_name_;
  int start_state_number_;
  double duration_per_state_;
  double time_shift_;
};


}  // namespace systems
}  // namespace dairlib


