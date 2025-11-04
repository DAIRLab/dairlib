#pragma once

#include <Eigen/Dense>
#include "drake/common/yaml/yaml_read_archive.h"

struct MagnaSimParams {
  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double object_publish_rate;
  bool publish_efforts;
  Eigen::VectorXd q_init_franka;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(q_init_franka));
  }
};
