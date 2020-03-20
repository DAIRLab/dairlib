#pragma once

#include "systems/framework/output_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {

enum FSM_STATE { BALANCE, CROUCH, FLIGHT, LAND};

class JumpingEventFsm : public drake::systems::LeafSystem<double> {
 public:
  JumpingEventFsm(
      const drake::multibody::MultibodyPlant<double>& plant,
      double flight_time, double land_time, double delay_time = 0.0,
      bool contact_driven = true, FSM_STATE init_state = BALANCE);

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

  const drake::multibody::MultibodyPlant<double>& plant_;
  int state_port_;
  int contact_port_;
  double flight_time_;
  double land_time_;
  double delay_time_;
  bool contact_driven_;

  int fsm_idx_;
  int prev_time_idx_;

  const FSM_STATE init_state_;
};

}  // namespace examples
}  // namespace dairlib