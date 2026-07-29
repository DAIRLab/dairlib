#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct SamplingC3VisualizerParams {
  std::string ee_vis_model;
  std::vector<std::string> object_vis_models;
  double visualizer_publish_rate;

  Eigen::VectorXd camera_pose;
  Eigen::VectorXd camera_target;

  bool visualize_c3_workspace;
  bool visualize_c3_state;

  bool visualize_center_of_mass_plan_curr;
  bool visualize_c3_forces_curr;
  bool visualize_center_of_mass_plan_best;
  bool visualize_c3_forces_best;

  bool visualize_is_c3_mode;
  bool visualize_sample_locations;
  bool visualize_sample_buffer;
  double sample_buffer_point_size;

  Eigen::VectorXd is_c3_mode_color;
  Eigen::VectorXd sample_color;

  bool visualize_execution_plan;

  bool visualize_c3_plan_curr;
  Eigen::VectorXd c3_curr_object_color;
  Eigen::VectorXd c3_curr_ee_color;
  Eigen::VectorXd df_curr_object_color;
  Eigen::VectorXd df_curr_ee_color;

  bool visualize_c3_plan_best;
  Eigen::VectorXd c3_best_object_color;
  Eigen::VectorXd c3_best_ee_color;
  Eigen::VectorXd df_best_object_color;
  Eigen::VectorXd df_best_ee_color;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(ee_vis_model));
    a->Visit(DRAKE_NVP(object_vis_models));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));
    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
    a->Visit(DRAKE_NVP(visualize_c3_workspace));
    a->Visit(DRAKE_NVP(visualize_c3_state));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan_curr));
    a->Visit(DRAKE_NVP(visualize_c3_forces_curr));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan_best));
    a->Visit(DRAKE_NVP(visualize_c3_forces_best));
    a->Visit(DRAKE_NVP(visualize_is_c3_mode));
    a->Visit(DRAKE_NVP(visualize_sample_locations));
    a->Visit(DRAKE_NVP(visualize_sample_buffer));
    a->Visit(DRAKE_NVP(sample_buffer_point_size));
    a->Visit(DRAKE_NVP(is_c3_mode_color));
    a->Visit(DRAKE_NVP(sample_color));
    a->Visit(DRAKE_NVP(visualize_execution_plan));
    a->Visit(DRAKE_NVP(visualize_c3_plan_curr));
    a->Visit(DRAKE_NVP(c3_curr_object_color));
    a->Visit(DRAKE_NVP(c3_curr_ee_color));
    a->Visit(DRAKE_NVP(df_curr_object_color));
    a->Visit(DRAKE_NVP(df_curr_ee_color));
    a->Visit(DRAKE_NVP(visualize_c3_plan_best));
    a->Visit(DRAKE_NVP(c3_best_object_color));
    a->Visit(DRAKE_NVP(c3_best_ee_color));
    a->Visit(DRAKE_NVP(df_best_object_color));
    a->Visit(DRAKE_NVP(df_best_ee_color));
  }
};
