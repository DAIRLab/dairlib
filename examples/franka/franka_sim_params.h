#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct FrankaSimParams {

  double dt;
  double realtime_rate;
  std::vector<double> q_init_franka;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(q_init_franka));

  }
};