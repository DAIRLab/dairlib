#pragma once

#include <string>

#include "systems/framework/output_vector.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {

static const double kEpislonForce = 1e-6;

/// ContactScheduler
/// Given a contact schedule, passthrough the current contact mode
/// @param contact_state_to_fsm_state_map: map from binary encoded contact state to corresponding finite state
/// ie. If the order is LeftContact RightContact, then 10 = 2 = Left Active Right Inactive
///                                                    01 = 1 = Left Inactive Right Active
/// ie. If the order is LeftFront LeftRear RightFront RightRear, then 1100 = 12 = Left Active Right Inactive
///                                                                   0011 = 3 = Left Inactive Right Active
/// Convention probably makes most sense when thinking about front/back contacts independently
class ContactScheduler : public drake::systems::LeafSystem<double> {
 public:
  ContactScheduler(
      const drake::multibody::MultibodyPlant<double> &plant,
      const std::unordered_map<int, int>& contact_state_to_fsm_state_map);

  const drake::systems::InputPort<double> &get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::InputPort<double> &get_input_port_force_trajectory() const {
    return this->get_input_port(contact_force_trajectory_input_port_);
  }

  const drake::systems::OutputPort<double> &get_output_port_fsm() const {
    return this->get_output_port(fsm_output_port_);
  }

 protected:
  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex contact_force_trajectory_input_port_;
  drake::systems::OutputPortIndex fsm_output_port_;

 private:
  void CalcFiniteState(const drake::systems::Context<double> &context,
                       drake::systems::BasicVector<double> *fsm_state) const;

  std::unordered_map<int, int> contact_state_to_fsm_state_map_;
};

}  // namespace systems
}  // namespace dairlib
