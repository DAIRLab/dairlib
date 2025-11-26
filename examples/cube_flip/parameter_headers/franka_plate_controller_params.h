#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaPlateC3ControllerParams {

  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string end_effector_lcs_model;
  std::string object_model;

  std::string osqp_settings_file;
  std::string ic3_options_file;
  std::string c3_options_file;

  bool include_end_effector_orientation;

  Eigen::Vector3d tool_attachment_frame;
  Eigen::Vector3d ee_init_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_lcs_model));
    a->Visit(DRAKE_NVP(object_model));

    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(ic3_options_file));
    a->Visit(DRAKE_NVP(c3_options_file));

    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ee_init_position));

  }
};