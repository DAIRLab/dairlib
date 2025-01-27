#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3TrajectoryParams {
  int trajectory_type;
  bool use_changing_final_goal;
  int changing_final_goal_type;
  bool prevent_three_topples_for_random_goal_gen;
  double final_goal_time_tolerance;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  Eigen::VectorXd fixed_target_position;
  Eigen::VectorXd fixed_target_orientation;
  double step_size;
  double start_point_x;
  double start_point_y;
  double end_point_x;
  double end_point_y;
  double lookahead_step_size;
  double lookahead_angle;
  double angle_hysteresis;
  double angle_err_to_vel_factor;
  double max_step_size;
  double ee_goal_height;
  double object_half_width;
  double position_success_threshold;
  double orientation_success_threshold;
  Eigen::VectorXd random_goal_x_limits;
  Eigen::VectorXd random_goal_y_limits;
  Eigen::VectorXd random_goal_radius_limits;
  double resting_object_height;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(use_changing_final_goal));
    a->Visit(DRAKE_NVP(changing_final_goal_type));
    a->Visit(DRAKE_NVP(prevent_three_topples_for_random_goal_gen));
    a->Visit(DRAKE_NVP(final_goal_time_tolerance));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(fixed_target_position));
    a->Visit(DRAKE_NVP(fixed_target_orientation));
    a->Visit(DRAKE_NVP(step_size));
    a->Visit(DRAKE_NVP(start_point_x));
    a->Visit(DRAKE_NVP(start_point_y));
    a->Visit(DRAKE_NVP(end_point_x));
    a->Visit(DRAKE_NVP(end_point_y));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(lookahead_angle));
    a->Visit(DRAKE_NVP(angle_hysteresis));
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(max_step_size));
    a->Visit(DRAKE_NVP(ee_goal_height));
    a->Visit(DRAKE_NVP(object_half_width));
    a->Visit(DRAKE_NVP(position_success_threshold));
    a->Visit(DRAKE_NVP(orientation_success_threshold));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));
    a->Visit(DRAKE_NVP(resting_object_height));
  }
};