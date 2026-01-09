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

  bool visualize_center_of_mass_plan;
  bool visualize_c3_forces;

  bool visualize_c3_plan_object;
  bool visualize_c3_plan_robot;

  bool visualize_mpm_points;
  double mpm_point_size;

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
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_c3_plan_object));
    a->Visit(DRAKE_NVP(visualize_c3_plan_robot));
    a->Visit(DRAKE_NVP(visualize_mpm_points));
    a->Visit(DRAKE_NVP(mpm_point_size));
    a->Visit(DRAKE_NVP(c3_state_object_color));
    a->Visit(DRAKE_NVP(c3_state_ee_color));
    a->Visit(DRAKE_NVP(c3_object_color));
    a->Visit(DRAKE_NVP(c3_ee_color));
    a->Visit(DRAKE_NVP(df_object_color));
    a->Visit(DRAKE_NVP(df_ee_color));
  }
};
