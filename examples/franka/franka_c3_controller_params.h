#pragma once

#include "solvers/c3_options.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct FrankaC3ControllerParams {
  std::string c3_options_file;
  std::string c3_channel_actor;
  std::string c3_channel_object;
  std::string state_channel;
  std::string radio_channel;
  std::string franka_model;
  std::string plate_model;
  std::string tray_model;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(c3_channel_actor));
    a->Visit(DRAKE_NVP(c3_channel_object));
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(plate_model));
    a->Visit(DRAKE_NVP(tray_model));
  }
};