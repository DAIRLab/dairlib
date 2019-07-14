#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "common/math_utils.h"
#include "attic/multibody/rigidbody_utils.h"

namespace dairlib {
namespace systems {

class LIPMTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  LIPMTrajGenerator(RigidBodyTree<double> * tree,
                    double desired_com_height,
                    double stance_duration_per_leg,
                    int left_stance_state,
                    int right_stance_state,
                    int left_foot_idx,
                    Eigen::Vector3d pt_on_left_foot,
                    int right_foot_idx,
                    Eigen::Vector3d pt_on_right_foot);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_FSM() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  // Discrete update calculates and stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::ExponentialPlusPiecewisePolynomial<double>* traj) const;

  int state_port_;
  int fsm_port_;

  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  RigidBodyTree<double>* tree_;

  double desired_com_height_;
  double stance_duration_per_leg_;

  int left_stance_state_;
  int right_stance_state_;
  int left_foot_idx_;
  int right_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  Eigen::Vector3d pt_on_right_foot_;

  bool is_quaternion_;
};

}  // namespace systems
}  // namespace dairlib


