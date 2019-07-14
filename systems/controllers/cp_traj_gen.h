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

class CPTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  CPTrajGenerator(RigidBodyTree<double>* tree,
                  double mid_foot_height,
                  double desired_final_foot_height,
                  double max_CoM_to_CP_dist,
                  double stance_duration_per_leg,
                  int left_stance_state,
                  int right_stance_state,
                  int left_foot_idx,
                  int right_foot_idx,
                  int pelvis_idx,
                  bool is_walking_position_control,
                  bool is_feet_collision_avoid,
                  bool is_using_predicted_com,
                  bool is_print_info);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_FSM() const {
    return this->get_input_port(FSM_port_);
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
                 drake::trajectories::PiecewisePolynomial<double>* traj) const;

  int state_port_;
  int FSM_port_;
  int com_port_;
  int fp_port_;

  int prev_td_swing_foot_idx_;
  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  bool is_quaternion_;

  RigidBodyTree<double>* tree_;
  double mid_foot_height_;
  double desired_final_foot_height_;
  double max_CoM_to_CP_dist_;
  double stance_duration_per_leg_;
  int left_stance_;
  int right_stance_;
  int left_foot_idx_;
  int right_foot_idx_;
  int pelvis_idx_;
  bool is_walking_position_control_;
  bool is_feet_collision_avoid_;
  bool is_using_predicted_com_;
  bool is_print_info_;

  // Parameters
  const double shift_foothold_dist_ = 0.06;  // meter
  const double center_line_shift_dist_ = 0.06;  // meter
};

}  // namespace systems
}  // namespace dairlib


