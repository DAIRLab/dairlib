#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaC3ControllerParams {
  std::vector<std::string> c3_options_file;
  std::string osqp_settings_file;
  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string end_effector_simple_model;
  std::string end_effector_simple_name;
  std::string plate_model;
  std::string jack_model;
  std::string ground_model;
  std::string left_support_model;
  std::string right_support_model;

  Eigen::Vector3d tool_attachment_frame;
  Eigen::Vector3d ground_franka_frame;
  Eigen::Vector3d left_support_position;
  Eigen::Vector3d right_support_position;
  Eigen::Vector3d left_support_orientation;
  Eigen::Vector3d right_support_orientation;
  double workspace_margin;
  double end_effector_thickness;

  int scene_index;

  bool include_end_effector_orientation;
  double target_frequency;

  int trajectory_type;
  double traj_radius;
  double x_c;
  double y_c;
  double lead_angle;
  double fixed_goal_x;
  double fixed_goal_y;
  double step_size;
  double start_point_x;
  double start_point_y;
  double end_point_x;
  double end_point_y;
  double lookahead_step_size;
  double max_step_size;


  // std::vector<Eigen::Vector3d> first_target;
  // std::vector<Eigen::Vector3d> second_target;
  // std::vector<Eigen::Vector3d> third_target;
  // double x_scale;
  // double y_scale;
  // double z_scale;
  // double near_target_threshold;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_simple_model));
    a->Visit(DRAKE_NVP(end_effector_simple_name));
    a->Visit(DRAKE_NVP(plate_model));
    a->Visit(DRAKE_NVP(jack_model));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(left_support_model));
    a->Visit(DRAKE_NVP(right_support_model));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(target_frequency));

    // Trajectory parameters
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(fixed_goal_x));
    a->Visit(DRAKE_NVP(fixed_goal_y));
    a->Visit(DRAKE_NVP(step_size));
    a->Visit(DRAKE_NVP(start_point_x));
    a->Visit(DRAKE_NVP(start_point_y));
    a->Visit(DRAKE_NVP(end_point_x));
    a->Visit(DRAKE_NVP(end_point_y));
    a->Visit(DRAKE_NVP(lookahead_step_size));
    a->Visit(DRAKE_NVP(max_step_size));
    
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_franka_frame));
    a->Visit(DRAKE_NVP(scene_index));


    // a->Visit(DRAKE_NVP(first_target));
    // a->Visit(DRAKE_NVP(second_target));
    // a->Visit(DRAKE_NVP(third_target));
    // a->Visit(DRAKE_NVP(x_scale));
    // a->Visit(DRAKE_NVP(y_scale));
    // a->Visit(DRAKE_NVP(z_scale));
    // a->Visit(DRAKE_NVP(near_target_threshold));

    a->Visit(DRAKE_NVP(left_support_position));
    a->Visit(DRAKE_NVP(right_support_position));
    a->Visit(DRAKE_NVP(left_support_orientation));
    a->Visit(DRAKE_NVP(right_support_orientation));
    a->Visit(DRAKE_NVP(end_effector_thickness));
  }
};