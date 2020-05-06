#pragma once

#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_fsm_out.hpp"

namespace dairlib {
namespace examples {

enum FSM_STATE { LEFT_FOOT, RIGHT_FOOT, LEFT_FOOT_2 };

class WalkingFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  WalkingFiniteStateMachine(
      const drake::multibody::MultibodyPlant<double>& plant,
      double r_impact_time, double l_impact_time, double delay_time = 0.0,
      bool contact_driven = true, int init_state = 0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_contact_input_port() const {
    return this->get_input_port(contact_port_);
  }

  const drake::systems::OutputPort<double>& get_fsm_output_port() const {
    return this->get_output_port(fsm_output_port_);
  }

  const drake::systems::OutputPort<double>& get_lcm_output_port() const {
    return this->get_output_port(lcm_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;
  void AssignLcmOutput(const drake::systems::Context<double>& context,
                       dairlib::lcmt_fsm_out* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  int state_port_;
  int contact_port_;
  int fsm_output_port_;
  int lcm_port_;
  double r_impact_time_;
  double l_impact_time_;
  double delay_time_;
  bool contact_driven_;

  int prev_time_idx_;
  int contact_time_idx_;
  int contact_flag_idx_;
  int fsm_idx_;

  const FSM_STATE init_state_;
};

}  // namespace examples
}  // namespace dairlib
