#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::VectorXd;

struct HeuristicGaitParams {
  double roll_phase;
  double return_phase;
  VectorXd gait_parameters;
  int axis_option;
  double tilt_degrees;
  std::vector<double> q_new_vector;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(roll_phase));
    a->Visit(DRAKE_NVP(return_phase));
    a->Visit(DRAKE_NVP(gait_parameters));
    a->Visit(DRAKE_NVP(axis_option));
    a->Visit(DRAKE_NVP(tilt_degrees));
    a->Visit(DRAKE_NVP(q_new_vector));
  }
};