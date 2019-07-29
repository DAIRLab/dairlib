#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace systems {

/// CPTrajGenerator generates a desired 3D trajectory of swing foot.
/// The trajectory is a cubic spline (two segments of cubic polynomials).
/// In the x-y plane, the start point of the traj is the swing foot position
/// before it leaves the ground, and the end point is the capture point (CP).
/// In the z direction, the start point is the swing foot position before it
/// leaves the ground, and the mid point and end point are both specified by
/// the user.
/// The CP can be modified according to two flags
///  - `add_extra_control`
///  - `is_feet_collision_avoid`
///
/// Arguments of the constructor:
/// - rigid body tree
/// - desired height of the swing foot during mid swing phase
/// - desired height of the swing foot at the end of swing phase
/// - desired vertical velocity of the swing foot at the end of swing phase
/// - maximum distance between center of mass and CP
///     (used to restrict the CP within an area)
/// - duration of the swing phase
/// - left foot body index
/// - right foot body index
/// - position of the contact point w.r.t. left foot body
/// - position of the contact point w.r.t. right foot body
/// - pelvis body index (used to get the pelvis heading direction)
/// - a flag enabling CP modification (e.g. walking speed control)
/// - a flag enabling feet collision avoidance
/// - a flag enabling the usage of prediction of center of mass
///     (use predicted center of mass position at touchdown to calculate CP)
/// - CP offset (to avoid foot collision)
/// - center line offset (used to restrict the CP within an area)

class CPTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  CPTrajGenerator(const RigidBodyTree<double>& tree,
                  double mid_foot_height,
                  double desired_final_foot_height,
                  double desired_final_vertical_foot_velocity,
                  double max_CoM_to_CP_dist,
                  double stance_duration_per_leg,
                  int left_foot_idx,
                  Eigen::Vector3d pt_on_left_foot,
                  int right_foot_idx,
                  Eigen::Vector3d pt_on_right_foot,
                  int pelvis_idx,
                  bool add_extra_control,
                  bool is_feet_collision_avoid,
                  bool is_using_predicted_com,
                  double cp_offset,
                  double center_line_offset);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_com() const {
    return this->get_input_port(com_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fp() const {
    return this->get_input_port(fp_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  Eigen::Vector2d calculateCapturePoint(
    const drake::systems::Context<double>& context,
    const OutputVector<double>* robot_output,
    const double end_time_of_this_interval) const;

  drake::trajectories::PiecewisePolynomial<double> createSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval,
    const Eigen::Vector3d & init_swing_foot_pos,
    const Eigen::Vector2d & CP) const;

  void CalcTrajs(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* traj) const;

  int state_port_;
  int fsm_port_;
  int com_port_;
  int fp_port_;

  int prev_td_swing_foot_idx_;
  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  bool is_quaternion_;

  const RigidBodyTree<double>& tree_;
  double mid_foot_height_;
  double desired_final_foot_height_;
  double desired_final_vertical_foot_velocity_;
  double max_CoM_to_CP_dist_;
  double stance_duration_per_leg_;
  int left_foot_idx_;
  int right_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  Eigen::Vector3d pt_on_right_foot_;
  int pelvis_idx_;
  bool add_extra_control_;
  bool is_feet_collision_avoid_;
  bool is_using_predicted_com_;

  // Parameters
  const double cp_offset_;  // in meters
  const double center_line_offset_;  // in meters

  // left stance state (of finite state machine)
  // right stance state (of finite state machine)
  const int left_stance_ = 0;
  const int right_stance_ = 1;
};

}  // namespace systems
}  // namespace dairlib


