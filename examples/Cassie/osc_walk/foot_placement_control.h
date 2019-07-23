#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace cassie {
namespace osc_walk {

// TODO(yminchen): we can replace cp with raibert style control. (feedforward
// term is v*T/2)

// TODO(yminchen): we can make global target position an input port of the
// the system if it's needed in the future.

/// FootPlacementControl calculates the deviation from capture point in order
/// to track a desired velocity.


/// Requirement: quaternion floating-based Cassie only
class FootPlacementControl : public drake::systems::LeafSystem<double> {
 public:
  FootPlacementControl(RigidBodyTree<double>* tree,
      int pelvis_idx,
      Eigen::Vector2d global_target_position,
      Eigen::Vector2d params_of_no_turning);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  void CalcFootPlacement(const drake::systems::Context<double>& context,
                         drake::systems::BasicVector<double>* output) const;

  RigidBodyTree<double>* tree_;
  int pelvis_idx_;
  Eigen::Vector2d global_target_position_;
  Eigen::Vector2d params_of_no_turning_;

  int state_port_;

  double kp_pos_sagital_;
  double kd_pos_sagital_;
  double vel_max_sagital_;
  double vel_min_sagital_;
  double k_fp_ff_sagital_;
  double k_fp_fb_sagital_;
  double target_pos_offset_;

  double kp_pos_lateral_;
  double kd_pos_lateral_;
  double vel_max_lateral_;
  double vel_min_lateral_;
  double k_fp_ff_lateral_;
  double k_fp_fb_lateral_;
};

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


