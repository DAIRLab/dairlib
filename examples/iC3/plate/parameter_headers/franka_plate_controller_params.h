#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaPlateControllerParams {

  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string end_effector_lcs_model;
  std::string object_model;

  std::string osqp_settings_file;
  std::string ic3_options_file;
  std::string c3_controller_options_file;

  bool include_end_effector_orientation;
  bool run_open_loop;
  bool track_dynamically_feasible;

  bool add_noise;
  double time_to_wait;

  Eigen::Vector3d tool_attachment_frame;
  Eigen::Vector3d ee_init_position;
  Eigen::VectorXd x_target;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_lcs_model));
    a->Visit(DRAKE_NVP(object_model));

    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(ic3_options_file));
    a->Visit(DRAKE_NVP(c3_controller_options_file));
    a->Visit(DRAKE_NVP(time_to_wait));

    a->Visit(DRAKE_NVP(add_noise));

    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(run_open_loop));
    a->Visit(DRAKE_NVP(track_dynamically_feasible));

    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ee_init_position));
    a->Visit(DRAKE_NVP(x_target));

  }
};