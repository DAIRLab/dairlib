#pragma once

// #include "solvers/c3_options.h"   Is this needed in this file?

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3TrajectoryParams {
  int trajectory_type;
  bool use_changing_final_goal_position;
  bool use_changing_final_goal_orientation;
  Eigen::VectorXd final_goal_position_tolerance;
  Eigen::VectorXd final_goal_orientation_tolerance;
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
  double angle_err_to_vel_factor;
  double max_step_size;
  double ee_goal_height;
  double object_half_width;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(use_changing_final_goal_position));
    a->Visit(DRAKE_NVP(use_changing_final_goal_orientation));
    a->Visit(DRAKE_NVP(final_goal_position_tolerance));
    a->Visit(DRAKE_NVP(final_goal_orientation_tolerance));
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
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(max_step_size));
    a->Visit(DRAKE_NVP(ee_goal_height));
    a->Visit(DRAKE_NVP(object_half_width));
  }
};