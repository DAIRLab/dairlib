#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct DeformVisualizerParams {
  double visualizer_publish_rate;

  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;

  bool visualize_c3_workspace;
  bool visualize_c3_state;
  bool visualize_c3_target_state;
  bool visualize_c3_final_target_state;
  Eigen::VectorXd c3_state_actual_color;
  Eigen::VectorXd c3_state_target_color;
  Eigen::VectorXd c3_state_final_target_color;

  bool visualize_center_of_mass_plan;
  bool visualize_c3_forces;

  bool visualize_is_c3_mode;
  bool visualize_sample_locations;
  bool visualize_sample_buffer;
  double sample_buffer_point_size;

  bool visualize_c3_plan_object;
  bool visualize_c3_plan_robot;

  bool visualize_cost_plan_object;
  bool visualize_cost_plan_robot;

  bool visualize_mpm_points;
  double mpm_point_size;

  bool visualize_model_reduction;
  std::string model_reduction_point_model;
  Eigen::VectorXd reduced_model_color;

  std::string ee_vis_model;

  Eigen::VectorXd is_c3_mode_color;
  Eigen::VectorXd sample_color;
  Eigen::VectorXd c3_state_object_color;
  Eigen::VectorXd c3_state_ee_color;
  Eigen::VectorXd c3_object_color;
  Eigen::VectorXd c3_ee_color;
  Eigen::VectorXd df_object_color;
  Eigen::VectorXd df_ee_color;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
    a->Visit(DRAKE_NVP(visualize_c3_workspace));
    a->Visit(DRAKE_NVP(visualize_c3_state));
    a->Visit(DRAKE_NVP(visualize_c3_target_state));
    a->Visit(DRAKE_NVP(visualize_c3_final_target_state));
    a->Visit(DRAKE_NVP(c3_state_actual_color));
    a->Visit(DRAKE_NVP(c3_state_target_color));
    a->Visit(DRAKE_NVP(c3_state_final_target_color));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_is_c3_mode));
    a->Visit(DRAKE_NVP(visualize_sample_locations));
    a->Visit(DRAKE_NVP(visualize_sample_buffer));
    a->Visit(DRAKE_NVP(sample_buffer_point_size));
    a->Visit(DRAKE_NVP(visualize_c3_plan_object));
    a->Visit(DRAKE_NVP(visualize_c3_plan_robot));
    a->Visit(DRAKE_NVP(visualize_cost_plan_object));
    a->Visit(DRAKE_NVP(visualize_cost_plan_robot));
    a->Visit(DRAKE_NVP(visualize_mpm_points));
    a->Visit(DRAKE_NVP(mpm_point_size));
    a->Visit(DRAKE_NVP(visualize_model_reduction));
    a->Visit(DRAKE_NVP(model_reduction_point_model));
    a->Visit(DRAKE_NVP(reduced_model_color));
    a->Visit(DRAKE_NVP(ee_vis_model));
    a->Visit(DRAKE_NVP(is_c3_mode_color));
    a->Visit(DRAKE_NVP(sample_color));
    a->Visit(DRAKE_NVP(c3_state_object_color));
    a->Visit(DRAKE_NVP(c3_state_ee_color));
    a->Visit(DRAKE_NVP(c3_object_color));
    a->Visit(DRAKE_NVP(c3_ee_color));
    a->Visit(DRAKE_NVP(df_object_color));
    a->Visit(DRAKE_NVP(df_ee_color));
  }
};
