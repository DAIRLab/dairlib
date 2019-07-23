#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace cassie {
namespace osc_walk {

/// HeadingControl calculates desired heading angle (yaw in global frame).

/// Requirement: quaternion floating-based Cassie only
class HeadingControl : public drake::systems::LeafSystem<double> {
 public:
  HeadingControl(RigidBodyTree<double>* tree,
                 int pelvis_idx,
                 Eigen::Vector2d global_target_position,
                 Eigen::Vector2d params_of_no_turning);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcHeadingAngle(const drake::systems::Context<double>& context,
                 drake::trajectories::PiecewisePolynomial<double>* traj) const;

  RigidBodyTree<double>* tree_;
  int pelvis_idx_;
  Eigen::Vector2d global_target_position_;
  Eigen::Vector2d params_of_no_turning_;

  int state_port_;
};

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


