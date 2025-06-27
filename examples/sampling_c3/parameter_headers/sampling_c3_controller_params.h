#pragma once

#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include <drake/common/yaml/yaml_io.h>


struct SamplingC3ControllerParams {
  std::string sampling_c3_options_file;   // C3 mode params
  std::string reposition_params_file;     // repositioning mode params
  std::string progress_params_file;       // mode switching params
  std::string sampling_params_file;       // sampling params
  std::string goal_params_file;           // goal checking/setting params

  std::string sim_params_file;            // simulation params
  std::string vis_params_file;            // visualization params
  std::string osc_params_file;            // OSC params
  std::string osqp_settings_file;
  std::string osc_qp_settings_file;
  std::string franka_driver_channels_file;
  std::string lcm_channels_hardware_file;
  std::string lcm_channels_simulation_file;

  std::string object_model;
  std::string object_body_name;

  double workspace_margin;
  bool include_end_effector_orientation;
  int control_loop_delay_ms;

  /// Store sub-parameter classes internally.
  SamplingC3Options sampling_c3_options;
  SamplingC3RepositionParams reposition_params;
  SamplingC3ProgressParams progress_params;
  SamplingParams sampling_params;
  SamplingC3GoalParams goal_params;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sampling_c3_options_file));
    a->Visit(DRAKE_NVP(reposition_params_file));
    a->Visit(DRAKE_NVP(progress_params_file));
    a->Visit(DRAKE_NVP(sampling_params_file));
    a->Visit(DRAKE_NVP(goal_params_file));
    a->Visit(DRAKE_NVP(sim_params_file));
    a->Visit(DRAKE_NVP(vis_params_file));
    a->Visit(DRAKE_NVP(osc_params_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
    a->Visit(DRAKE_NVP(franka_driver_channels_file));
    a->Visit(DRAKE_NVP(lcm_channels_hardware_file));
    a->Visit(DRAKE_NVP(lcm_channels_simulation_file));
    a->Visit(DRAKE_NVP(object_model));
    a->Visit(DRAKE_NVP(object_body_name));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(control_loop_delay_ms));

    /// Store individual parameter classes internally.
    sampling_c3_options = drake::yaml::LoadYamlFile<SamplingC3Options>(
      sampling_c3_options_file);
    reposition_params = drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
      reposition_params_file);
    progress_params = drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
      progress_params_file);
    sampling_params = drake::yaml::LoadYamlFile<SamplingParams>(
      sampling_params_file);
    goal_params = drake::yaml::LoadYamlFile<SamplingC3GoalParams>(
      goal_params_file);
  }
};
