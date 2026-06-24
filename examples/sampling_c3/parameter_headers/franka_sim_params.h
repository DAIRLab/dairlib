#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaSimParams {
  std::vector<std::string> object_models;
  double dt;
  double realtime_rate;
  double actuator_delay;
  double franka_publish_rate;
  double object_publish_rate;
  bool visualize_drake_sim;
  bool publish_efforts;
  Eigen::VectorXd q_init_franka;
  std::vector<Eigen::VectorXd> q_init_objects;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(franka_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_objects));
  }
};
