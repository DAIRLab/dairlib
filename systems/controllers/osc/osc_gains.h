#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCGains {
  // costs
  double w_input;
  double w_input_reg;
  double w_accel;
  double w_soft_constraint;
  double impact_threshold;
  double mu;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_input_reg));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(mu));
  }
};