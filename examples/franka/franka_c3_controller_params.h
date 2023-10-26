#pragma once

#include "solvers/c3_options.h"

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
  std::string end_effector_model;
  std::string end_effector_name;
  std::string plate_model;
  std::string tray_model;
  bool include_end_effector_orientation;
  double target_frequency;

  template <typename Archive>
  void Serialize(Archive* a) {

    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(c3_channel_actor));
    a->Visit(DRAKE_NVP(c3_channel_object));
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(plate_model));
    a->Visit(DRAKE_NVP(tray_model));
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(target_frequency));
  }
};