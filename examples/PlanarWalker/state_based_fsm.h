#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "systems/framework/output_vector.h"

namespace dairlib {

enum StanceState { left_stance_state, right_stance_state };

class StateBasedFiniteStateMachine : public drake::systems::LeafSystem<double> {
 public:
  StateBasedFiniteStateMachine(const RigidBodyTree<double>& tree,
                              double left_foot_idx,
                              Eigen::Vector3d pt_on_left_foot,
                              double right_foot_idx,
                              Eigen::Vector3d pt_on_right_foot,
                              double time_shift = 0);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* output) const;

  const RigidBodyTree<double>& tree_;
  double left_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  double right_foot_idx_;
  Eigen::Vector3d pt_on_right_foot_;

  // Input port indices
  int state_port_;
  int trajectory_port_;

  // Discrete state indices
  int fsm_state_index_;
  int prev_fsm_change_time_idx_;

  double time_shift_;

  const int left_stance_state_ = 0;
  const int right_stance_state_ = 1;
};


}  // namespace dairlib


