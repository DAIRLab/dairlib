#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct FrankaSimParams {

  std::string state_channel;
  std::string controller_channel;
  std::string tray_state_channel;
  std::string franka_model;

  double dt;
  double realtime_rate;
  double actuator_delay;
  double publish_rate;

  bool visualize;
  bool publish_efforts;

  std::vector<double> q_init_franka;
  std::vector<double> q_init_plate;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(controller_channel));
    a->Visit(DRAKE_NVP(tray_state_channel));
    a->Visit(DRAKE_NVP(franka_model));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(publish_rate));

    a->Visit(DRAKE_NVP(visualize));
    a->Visit(DRAKE_NVP(publish_efforts));

    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_plate));

  }
};