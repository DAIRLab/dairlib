#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "solvers/c3_options.h"

using Eigen::MatrixXd;

struct FrankaC3ControllerParams {
  std::string c3_options_file;
  std::string c3_channel;
  std::string state_channel;
  std::string radio_channel;


  double mu;
  double dt;
  int num_friction_directions;
  std::string plant_urdf;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(c3_channel));
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(plant_urdf));

  }
};