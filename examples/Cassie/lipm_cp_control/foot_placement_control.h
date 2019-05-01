#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;


namespace dairlib {
namespace cassie {
namespace cp_control {

// For quaternion floating-based Cassie
class FootPlacementControl : public LeafSystem<double> {
 public:
  FootPlacementControl(RigidBodyTree<double> * tree,
    Vector2d global_target_position, double circle_radius_of_no_turning);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcFootPlacement(const Context<double>& context,
                         BasicVector<double>* output) const;

  int state_port_;

  RigidBodyTree<double> * tree_;

  bool is_quaternion_;

  Vector2d global_target_position_;
  double circle_radius_of_no_turning_;

  double kp_pos_sagital_;
  double kd_pos_sagital_;
  double vel_max_sagital_;
  double vel_min_sagital_;
  double k_footPlacement_ff_sagital_;
  double k_footPlacement_fb_sagital_;
  double target_position_offset_;

  double kp_pos_lateral_;
  double kd_pos_lateral_;
  double vel_max_lateral_;
  double vel_min_lateral_;
  double k_footPlacement_ff_lateral_;
  double k_footPlacement_fb_lateral_;
};

}  // namespace cp_control
}  // namespace cassie
}  // namespace dairlib


