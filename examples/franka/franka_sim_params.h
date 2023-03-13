#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct FrankaSimParams {

  double dt;
  double realtime_rate;
  std::vector<double> q_init_franka;

  std::string state_channel;
  std::string controller_channel;
  double publish_rate;
  bool publish_efforts;
  double actuator_delay;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(controller_channel));
    a->Visit(DRAKE_NVP(publish_rate));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(q_init_franka));

  }
};