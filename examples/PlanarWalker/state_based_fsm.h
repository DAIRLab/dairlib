#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "systems/framework/output_vector.h"

namespace dairlib {

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
  void CalcFiniteState(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* fsm_state) const;

  const RigidBodyTree<double>& tree_;
  double left_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  double right_foot_idx_;
  Eigen::Vector3d pt_on_right_foot_;

  int state_port_;

  int first_state_number_ = 0;
  int second_state_number_ = 1;
  int initial_state_number_ = 0; // TODO: change this later
  double time_shift_;
};


}  // namespace dairlib


