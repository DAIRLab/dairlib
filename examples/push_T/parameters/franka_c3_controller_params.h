#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaC3ControllerParams {
  bool run_in_safe_mode;
  std::vector<std::string> c3_options_file;
  std::vector<std::string> sampling_params_file;
  std::string osqp_settings_file;
  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;
  std::string end_effector_simple_model;
  std::string end_effector_simple_name;
  std::string jack_model;
  std::string object_body_name;
  std::string ground_model;
  std::string platform_model;

  Eigen::Vector3d tool_attachment_frame;
  Eigen::Vector3d p_franka_to_ground;
 
  double workspace_margin;

  bool include_end_effector_orientation;
  double target_frequency;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(run_in_safe_mode));
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(sampling_params_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(platform_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_simple_model));
    a->Visit(DRAKE_NVP(end_effector_simple_name));
    a->Visit(DRAKE_NVP(jack_model));
    a->Visit(DRAKE_NVP(object_body_name));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(target_frequency));
    
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(p_franka_to_ground));
  }
};