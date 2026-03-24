#pragma once

#include <drake/common/yaml/yaml_io.h>

#include "examples/deform/parameter_headers/elastoplastic_sc3_options.h"
#include "examples/deform/parameter_headers/goal_params.h"
#include "examples/sampling_c3/parameter_headers/progress_params.h"
#include "examples/sampling_c3/parameter_headers/reposition_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct DeformControllerParams {
  std::string elastoplastic_sc3_options_file;  // SC3 params (includes C3)
  std::string reposition_params_file;          // repositioning mode params
  std::string progress_params_file;            // mode switching params
  std::string sampling_params_file;            // sampling params
  std::string goal_params_file;                // goal checking/setting params

  std::string sim_params_file;  // simulation params
  std::string vis_params_file;  // visualization params
  std::string osc_params_file;  // OSC params
  std::string osqp_settings_file;
  std::string osc_qp_settings_file;
  std::string franka_driver_channels_file;
  std::string lcm_channels_hardware_file;
  std::string lcm_channels_simulation_file;

  double workspace_margin;
  bool include_end_effector_orientation;
  int control_loop_delay_ms;

  int num_nodes;

  /// Store sub-parameter classes internally.
  ElastoPlasticSC3Options elastoplastic_sc3_options;
  SamplingC3RepositionParams reposition_params;
  SamplingC3ProgressParams progress_params;
  SamplingParams sampling_params;
  ElastoPlasticGoalParams goal_params;
  // SamplingC3ControllerParams stored internally for convenience in passing to
  // EPSC3 controller, which inherits from SamplingC3Controller.
  SamplingC3ControllerParams sampling_c3_controller_params;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(elastoplastic_sc3_options_file));
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
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(control_loop_delay_ms));

    /// Store individual parameter classes internally.
    elastoplastic_sc3_options =
        drake::yaml::LoadYamlFile<ElastoPlasticSC3Options>(
            elastoplastic_sc3_options_file);
    reposition_params = drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
        reposition_params_file);
    progress_params = drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
        progress_params_file);
    sampling_params =
        drake::yaml::LoadYamlFile<SamplingParams>(sampling_params_file);
    goal_params =
        drake::yaml::LoadYamlFile<ElastoPlasticGoalParams>(goal_params_file);

    num_nodes = 0;  // TODO

    // TODO @bibit:  This is ugly; figure out cleaner way to handle this.
    sampling_c3_controller_params = SamplingC3ControllerParams();
    sampling_c3_controller_params.sampling_c3_options_file =
        elastoplastic_sc3_options_file;
    sampling_c3_controller_params.reposition_params_file =
        reposition_params_file;
    sampling_c3_controller_params.progress_params_file = progress_params_file;
    sampling_c3_controller_params.sampling_params_file = sampling_params_file;
    sampling_c3_controller_params.goal_params_file = goal_params_file;
    sampling_c3_controller_params.sim_params_file = sim_params_file;
    sampling_c3_controller_params.vis_params_file = vis_params_file;
    sampling_c3_controller_params.osc_params_file = osc_params_file;
    sampling_c3_controller_params.osqp_settings_file = osqp_settings_file;
    sampling_c3_controller_params.osc_qp_settings_file = osc_qp_settings_file;
    sampling_c3_controller_params.franka_driver_channels_file =
        franka_driver_channels_file;
    sampling_c3_controller_params.lcm_channels_hardware_file =
        lcm_channels_hardware_file;
    sampling_c3_controller_params.lcm_channels_simulation_file =
        lcm_channels_simulation_file;
    sampling_c3_controller_params.include_end_effector_orientation =
        include_end_effector_orientation;
    sampling_c3_controller_params.control_loop_delay_ms = control_loop_delay_ms;

    sampling_c3_controller_params.sampling_c3_options =
        drake::yaml::LoadYamlFile<SamplingC3Options>(
            sampling_c3_controller_params.sampling_c3_options_file);
    sampling_c3_controller_params.reposition_params =
        drake::yaml::LoadYamlFile<SamplingC3RepositionParams>(
            sampling_c3_controller_params.reposition_params_file);
    sampling_c3_controller_params.progress_params =
        drake::yaml::LoadYamlFile<SamplingC3ProgressParams>(
            sampling_c3_controller_params.progress_params_file);
    sampling_c3_controller_params.sampling_params =
        drake::yaml::LoadYamlFile<SamplingParams>(
            sampling_c3_controller_params.sampling_params_file);
    // NOTE: Not setting sampling_c3_controller_params.goal_params since there
    // is a type difference.  May need to do an SC3-compatible version of the
    // more concise EPSC3 goal params.

    sampling_c3_controller_params.num_objects =
        sampling_c3_controller_params.base_names.size();
  }
};
