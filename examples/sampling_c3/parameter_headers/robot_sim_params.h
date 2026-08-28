#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct RobotSimParams {
  std::vector<std::string> object_models;
  double dt;
  double realtime_rate;
  double actuator_delay;
  double robot_publish_rate;
  double object_publish_rate;
  bool visualize_drake_sim;
  bool publish_efforts;
  Eigen::VectorXd q_init_robot;
  std::vector<Eigen::VectorXd> q_init_objects;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(q_init_robot));
    a->Visit(DRAKE_NVP(q_init_objects));
  }
};
