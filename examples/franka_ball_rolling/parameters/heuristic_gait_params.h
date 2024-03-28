#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::VectorXd;

struct HeuristicGaitParams {
  double roll_phase;
  double return_phase;
  VectorXd gait_parameters;
  int axis_option;
  double orientation_degrees;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(roll_phase));
    a->Visit(DRAKE_NVP(return_phase));
    a->Visit(DRAKE_NVP(gait_parameters));
    a->Visit(DRAKE_NVP(axis_option));
    a->Visit(DRAKE_NVP(orientation_degrees));
  }
};