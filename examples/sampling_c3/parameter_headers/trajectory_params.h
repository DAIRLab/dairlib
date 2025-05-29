#pragma once
#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3TrajectoryParams {
  int goal_mode;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  Eigen::VectorXd fixed_target_position;
  Eigen::VectorXd fixed_target_orientation;
  double lookahead_step_size;
  double lookahead_angle;
  double angle_hysteresis;
  double angle_err_to_vel_factor;
  double ee_target_z_offset_above_object;
  double position_success_threshold;
  double orientation_success_threshold;
  Eigen::VectorXd random_goal_x_limits;
  Eigen::VectorXd random_goal_y_limits;
  Eigen::VectorXd random_goal_radius_limits;
  double resting_object_height;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(goal_mode));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(fixed_target_position));
    a->Visit(DRAKE_NVP(fixed_target_orientation));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(lookahead_angle));
    a->Visit(DRAKE_NVP(angle_hysteresis));
    a->Visit(DRAKE_NVP(angle_err_to_vel_factor));
    a->Visit(DRAKE_NVP(ee_target_z_offset_above_object));
    a->Visit(DRAKE_NVP(position_success_threshold));
    a->Visit(DRAKE_NVP(orientation_success_threshold));
    a->Visit(DRAKE_NVP(random_goal_x_limits));
    a->Visit(DRAKE_NVP(random_goal_y_limits));
    a->Visit(DRAKE_NVP(random_goal_radius_limits));
    a->Visit(DRAKE_NVP(resting_object_height));
  }
};
