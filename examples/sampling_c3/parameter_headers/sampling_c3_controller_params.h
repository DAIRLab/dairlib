#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

// TODO: @bibit this whole file should go away in favor of a newly-defined
// SamplingC3ControllerParams class.
struct SamplingC3ControllerParams {
  std::string sampling_params_file;
  std::string reposition_params_file;
  std::string osqp_settings_file;
  std::string object_model;
  std::string object_body_name;

  double workspace_margin;

  bool include_end_effector_orientation;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampling_params_file));
    a->Visit(DRAKE_NVP(reposition_params_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(object_model));
    a->Visit(DRAKE_NVP(object_body_name));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
  }
};
