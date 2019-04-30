#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using std::string;
using Eigen::VectorXd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {

// Time-based Two-state Finite State Machine
class TimeBaseCassieFiniteStateMachine : public LeafSystem<double> {
 public:
  TimeBaseCassieFiniteStateMachine(int num_positions,
                                   int num_velocities,
                                   int num_inputs,
                                   int first_state_number,
                                   int second_state_number,
                                   string first_state_name,
                                   string second_state_name,
                                   int start_state_number,
                                   double time_duration_per_state,
                                   double time_shift);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcFiniteState(const Context<double>& context,
                       BasicVector<double>* fsm_state) const;

  int state_port_;

  int first_state_number_;
  int second_state_number_;
  string first_state_name_;
  string second_state_name_;
  int start_state_number_;
  double time_duration_per_state_;
  double time_shift_;
};


}  // namespace systems
}  // namespace dairlib


