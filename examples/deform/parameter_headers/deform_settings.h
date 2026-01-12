#pragma once

#include <drake/common/yaml/yaml_io.h>

// #include "examples/deform/parameter_headers/elastoplastic_c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct DeformSettings {
  std::string deform_c3_options_file;
  std::string reduced_model_params_file;
  std::string sim_params_file;
  std::string vis_params_file;
  std::string osc_params_file;
  std::string osc_qp_settings_file;
  std::string franka_driver_channels_file;
  std::string lcm_channels_hardware_file;
  std::string lcm_channels_simulation_file;

  //   /// Store sub-parameter classes internally.
  //   ElastoPlasticC3Options deform_c3_options;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(deform_c3_options_file));
    a->Visit(DRAKE_NVP(reduced_model_params_file));
    a->Visit(DRAKE_NVP(sim_params_file));
    a->Visit(DRAKE_NVP(vis_params_file));
    a->Visit(DRAKE_NVP(osc_params_file));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
    a->Visit(DRAKE_NVP(franka_driver_channels_file));
    a->Visit(DRAKE_NVP(lcm_channels_hardware_file));
    a->Visit(DRAKE_NVP(lcm_channels_simulation_file));

    // /// Store individual parameter classes internally.
    // deform_c3_options = drake::yaml::LoadYamlFile<ElastoPlasticC3Options>(
    //     deform_c3_options_file);
  }
};
