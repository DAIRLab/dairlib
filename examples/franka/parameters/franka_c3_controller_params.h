#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaC3ControllerParams {
  std::vector<std::string> c3_options_file;
  std::string osqp_settings_file;
  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string plate_model;
  std::string tray_model;
  std::string left_support_model;
  std::string right_support_model;
  bool include_end_effector_orientation;
  double target_frequency;
  Eigen::Vector3d tool_attachment_frame;

  int scene_index;

  Eigen::Vector3d neutral_position;
  double x_scale;
  double y_scale;
  double z_scale;

  Eigen::Vector3d left_support_position;
  Eigen::Vector3d right_support_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(plate_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(left_support_model));
    a->Visit(DRAKE_NVP(right_support_model));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(target_frequency));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(scene_index));

    a->Visit(DRAKE_NVP(neutral_position));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));

    a->Visit(DRAKE_NVP(left_support_position));
    a->Visit(DRAKE_NVP(right_support_position));
  }
};