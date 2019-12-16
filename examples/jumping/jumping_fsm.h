#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace examples {
// namespace jumping {
// namespace osc {

enum FSM_STATE { NEUTRAL, CROUCH, FLIGHT, LAND };

class JumpingFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  JumpingFiniteStateMachine(const RigidBodyTree<double>& tree,
                            double wait_time,
                            double crouch_time);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  int state_port_;
  // indices for discrete variables in drake leafsystem
  int time_idx_;

  int fsm_idx_;
  double timestamp_;

  double initial_timestamp_;
  double wait_time_;
  double crouch_time_;
  const FSM_STATE init_state_ = NEUTRAL;
};

}  // namespace osc
}  // namespace jumping
// }  // namespace examples
// }  // namespace dairlib


